/*
 * UDS.c
 *
 * UDS 层只处理完整 PDU：request[0] 是 SID，不包含 ISO-TP PCI。
 * 新增服务时，优先增加一个 handler，再在 uds_service_table 中登记，
 * 不要把所有 SID 堆成越来越长的 if / else if。
 */
#include "UDS.h"
#include "IsoTp.h"

#define UDS_NRC_POSITIVE                       0x00U
#define UDS_NRC_SERVICE_NOT_SUPPORTED          0x11U
#define UDS_NRC_SUBFUNCTION_NOT_SUPPORTED      0x12U
#define UDS_NRC_INCORRECT_LENGTH               0x13U
#define UDS_NRC_REQUEST_SEQUENCE_ERROR         0x24U
#define UDS_NRC_INVALID_KEY                    0x35U
#define UDS_NRC_SECURITY_ACCESS_DENIED         0x33U
#define UDS_NRC_SERVICE_NOT_IN_SESSION          0x7FU

#define UDS_SESSION_MASK_DEFAULT               0x01U
#define UDS_SESSION_MASK_PROGRAMMING           0x02U
#define UDS_SESSION_MASK_EXTENDED              0x04U

typedef Uint16 (*UDS_ServiceHandler)(const Uint16 *request,
                                     Uint16 request_length,
                                     Uint16 *response,
                                     Uint16 *response_length);

typedef struct
{
    Uint16 sid;
    Uint16 session_mask;
    Uint16 security_required;
    Uint16 supports_suppress_positive;
    UDS_ServiceHandler handler;
} UDS_Service;

typedef struct
{
    UDS_SESSION session;
    Uint16 security_level;
    Uint32 s3_timer_ms;

    /*
     * 0x27 的最小顺序状态：RequestSeed(0x01) 后只允许 SendKey(0x02)。
     * 真正项目还应增加错误次数、延时锁定和随机 seed。
     */
    Uint16 security_seed_requested;
    Uint16 expected_key_sub_function;
} UDS_Context;

static UDS_Context g_uds;
static Uint16 g_uds_response[ISOTP_MAX_PDU_LENGTH];

static Uint16 UDS_Service10(const Uint16 *request,
                            Uint16 request_length,
                            Uint16 *response,
                            Uint16 *response_length);
static Uint16 UDS_Service27(const Uint16 *request,
                            Uint16 request_length,
                            Uint16 *response,
                            Uint16 *response_length);

static const UDS_Service g_uds_service_table[] =
{
    {
        0x10U,
        UDS_SESSION_MASK_DEFAULT |
        UDS_SESSION_MASK_PROGRAMMING |
        UDS_SESSION_MASK_EXTENDED,
        0U,
        1U,
        UDS_Service10
    },
    {
        0x27U,
        UDS_SESSION_MASK_PROGRAMMING |
        UDS_SESSION_MASK_EXTENDED,
        0U,
        0U,
        UDS_Service27
    }
};

static Uint16 UDS_GetSessionMask(void)
{
    switch(g_uds.session)
    {
        case UDS_SESSION_DEFAULT:
            return UDS_SESSION_MASK_DEFAULT;
        case UDS_SESSION_PROGRAMMING:
            return UDS_SESSION_MASK_PROGRAMMING;
        case UDS_SESSION_EXTENDED:
            return UDS_SESSION_MASK_EXTENDED;
        default:
            return 0U;
    }
}

static void UDS_ResetSecurityState(void)
{
    g_uds.security_level = 0U;
    g_uds.security_seed_requested = 0U;
    g_uds.expected_key_sub_function = 0U;
}

static void UDS_NegativeResponse(Uint16 request_sid, Uint16 nrc)
{
    /* 标准负响应固定为：7F + 原请求 SID + NRC。 */
    g_uds_response[0] = 0x7FU;
    g_uds_response[1] = request_sid & 0xFFU;
    g_uds_response[2] = nrc & 0xFFU;
    (void)IsoTp_Send(g_uds_response, 3U);
}

static Uint16 UDS_Service10(const Uint16 *request,
                            Uint16 request_length,
                            Uint16 *response,
                            Uint16 *response_length)
{
    Uint16 sub_function;

    if(request_length != 2U)
    {
        return UDS_NRC_INCORRECT_LENGTH;
    }

    /* bit7 是 suppressPosRspMsgIndicationBit，不属于子功能编号。 */
    sub_function = request[1] & 0x7FU;
    switch(sub_function)
    {
        case 0x01U:
            g_uds.session = UDS_SESSION_DEFAULT;
            break;
        case 0x02U:
            g_uds.session = UDS_SESSION_PROGRAMMING;
            break;
        case 0x03U:
            g_uds.session = UDS_SESSION_EXTENDED;
            break;
        default:
            return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
    }

    /* 切换 Session 时，原有安全解锁状态不再沿用。 */
    UDS_ResetSecurityState();

    response[0] = 0x50U;
    response[1] = sub_function;

    /* P2ServerMax = 0x0032 ms = 50 ms。 */
    response[2] = 0x00U;
    response[3] = 0x32U;

    /* P2*ServerMax 的单位为 10 ms：0x01F4 * 10 ms = 5000 ms。 */
    response[4] = 0x01U;
    response[5] = 0xF4U;
    *response_length = 6U;
    return UDS_NRC_POSITIVE;
}

static Uint16 UDS_Service27(const Uint16 *request,
                            Uint16 request_length,
                            Uint16 *response,
                            Uint16 *response_length)
{
    Uint16 sub_function;

    if(request_length < 2U)
    {
        return UDS_NRC_INCORRECT_LENGTH;
    }

    sub_function = request[1] & 0x7FU;

    if(sub_function == 0x01U)
    {
        if(request_length != 2U)
        {
            return UDS_NRC_INCORRECT_LENGTH;
        }

        /*
         * 教学用固定 seed，只用于先跑通帧格式和请求顺序。
         * 接真实项目时应替换成不可预测 seed，并把 seed 状态绑定到连接。
         */
        response[0] = 0x67U;
        response[1] = 0x01U;
        response[2] = 0x12U;
        response[3] = 0x34U;
        response[4] = 0x56U;
        response[5] = 0x78U;
        *response_length = 6U;

        g_uds.security_seed_requested = 1U;
        g_uds.expected_key_sub_function = 0x02U;
        return UDS_NRC_POSITIVE;
    }

    if(sub_function == 0x02U)
    {
        if(request_length != 6U)
        {
            return UDS_NRC_INCORRECT_LENGTH;
        }
        if((g_uds.security_seed_requested == 0U) ||
           (g_uds.expected_key_sub_function != 0x02U))
        {
            return UDS_NRC_REQUEST_SEQUENCE_ERROR;
        }

        /*
         * 当前没有项目 Key 算法，所以任何 Key 都明确返回 InvalidKey。
         * 不能为了演示正响应而把任意 Key 当作验证成功。
         */
        g_uds.security_seed_requested = 0U;
        g_uds.expected_key_sub_function = 0U;
        return UDS_NRC_INVALID_KEY;
    }

    return UDS_NRC_SUBFUNCTION_NOT_SUPPORTED;
}

void UDS_Init(void)
{
    g_uds.session = UDS_SESSION_DEFAULT;
    g_uds.s3_timer_ms = 0UL;
    UDS_ResetSecurityState();
}

void UDS_ProcessRequest(const Uint16 *request, Uint16 request_length)
{
    Uint16 sid;
    Uint16 service_count;
    Uint16 current_session_mask;
    Uint16 response_length;
    Uint16 nrc;
    Uint16 suppress_positive;
    Uint16 i;

    if((request == (const Uint16 *)0) || (request_length == 0U))
    {
        return;
    }

    sid = request[0] & 0xFFU;
    g_uds.s3_timer_ms = 0UL;
    current_session_mask = UDS_GetSessionMask();
    service_count = (Uint16)(sizeof(g_uds_service_table) /
                             sizeof(g_uds_service_table[0]));

    for(i = 0U; i < service_count; i++)
    {
        if(g_uds_service_table[i].sid == sid)
        {
            if((g_uds_service_table[i].session_mask &
                current_session_mask) == 0U)
            {
                UDS_NegativeResponse(sid, UDS_NRC_SERVICE_NOT_IN_SESSION);
                return;
            }

            if((g_uds_service_table[i].security_required != 0U) &&
               (g_uds.security_level <
                g_uds_service_table[i].security_required))
            {
                UDS_NegativeResponse(sid, UDS_NRC_SECURITY_ACCESS_DENIED);
                return;
            }

            response_length = 0U;
            nrc = g_uds_service_table[i].handler(request,
                                                 request_length,
                                                 g_uds_response,
                                                 &response_length);
            if(nrc != UDS_NRC_POSITIVE)
            {
                UDS_NegativeResponse(sid, nrc);
                return;
            }

            suppress_positive = 0U;
            if((g_uds_service_table[i].supports_suppress_positive != 0U) &&
               (request_length >= 2U) && ((request[1] & 0x80U) != 0U))
            {
                suppress_positive = 1U;
            }

            if(suppress_positive == 0U)
            {
                (void)IsoTp_Send(g_uds_response, response_length);
            }
            return;
        }
    }

    UDS_NegativeResponse(sid, UDS_NRC_SERVICE_NOT_SUPPORTED);
}

void UDS_Task1ms(void)
{
    if(g_uds.session != UDS_SESSION_DEFAULT)
    {
        g_uds.s3_timer_ms++;
        if(g_uds.s3_timer_ms >= UDS_S3_SERVER_MS)
        {
            g_uds.session = UDS_SESSION_DEFAULT;
            g_uds.s3_timer_ms = 0UL;
            UDS_ResetSecurityState();
        }
    }
}

UDS_SESSION UDS_GetCurrentSession(void)
{
    return g_uds.session;
}

Uint16 UDS_GetSecurityLevel(void)
{
    return g_uds.security_level;
}

