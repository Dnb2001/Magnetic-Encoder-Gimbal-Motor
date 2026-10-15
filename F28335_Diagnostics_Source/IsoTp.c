/*
 * IsoTp.c
 *
 * 经典 CAN（8 Byte）、Normal Addressing 的 ISO-TP 第一版。
 *
 * 这一层只负责：
 *   1. 把完整 PDU 拆成 SF 或 FF + CF；
 *   2. 把 SF 或 FF + CF 重新拼成完整 PDU；
 *   3. 管理 FC、BS、STmin、N_Bs 和 N_Cr。
 *
 * 它不知道 UDS 的 SID，也不直接访问 eCAN 寄存器。
 */
#include "IsoTp.h"
#include "CANIF.h"

#define ISOTP_FS_CTS                 0U
#define ISOTP_FS_WAIT                1U
#define ISOTP_FS_OVFLW               2U
#define ISOTP_INVALID_STMIN          0xFFFFU

typedef struct
{
    /* ------------------------- TX ------------------------- */
    Uint16 tx_buffer[ISOTP_MAX_PDU_LENGTH];
    Uint16 tx_length;
    Uint16 tx_index;
    Uint16 tx_sn;
    Uint16 tx_bs;
    Uint16 tx_bs_counter;
    Uint16 tx_stmin_ms;
    Uint16 tx_stmin_timer;
    Uint16 tx_n_bs_timer;
    Uint16 tx_wait_counter;
    ISOTP_TX_STATE tx_state;

    /* ------------------------- RX ------------------------- */
    Uint16 rx_buffer[ISOTP_MAX_PDU_LENGTH];
    Uint16 rx_length;
    Uint16 rx_index;
    Uint16 rx_expected_sn;
    Uint16 rx_bs_counter;
    Uint16 rx_n_cr_timer;
    ISOTP_RX_STATE rx_state;
    Uint16 rx_ready;

    /* RX 方向需要通过同一个 CAN TX 邮箱回复 Flow Control。 */
    Uint16 fc_pending;
    Uint16 fc_status;

    ISOTP_ERROR error;
} ISOTP_Context;

static ISOTP_Context g_isotp;

/* 把整帧先填成固定 DLC=8，未使用字节填 0xAA。 */
static void IsoTp_PrepareTxFrame(CAN_Frame *frame)
{
    Uint16 i;

    frame->id = DIAG_RESPONSE_CAN_ID;
    frame->dlc = 8U;
    for(i = 0U; i < 8U; i++)
    {
        frame->data[i] = ISOTP_PADDING_BYTE;
    }
}

/*
 * ISO 15765-2 的 STmin：
 *   0x00~0x7F = 0~127 ms；
 *   0xF1~0xF9 = 100~900 us。
 * 当前调度周期是 1 ms，因此微秒值保守折算成 1 ms。
 */
static Uint16 IsoTp_DecodeSTmin(Uint16 stmin)
{
    stmin &= 0xFFU;
    if(stmin <= 0x7FU)
    {
        return stmin;
    }
    if((stmin >= 0xF1U) && (stmin <= 0xF9U))
    {
        return 1U;
    }
    return ISOTP_INVALID_STMIN;
}

static Uint16 IsoTp_SendSF(void)
{
    CAN_Frame frame;
    Uint16 i;

    IsoTp_PrepareTxFrame(&frame);

    /* PCI 高半字节为 0；低半字节是 SF 的 PDU 长度。 */
    frame.data[0] = g_isotp.tx_length & 0x0FU;
    for(i = 0U; i < g_isotp.tx_length; i++)
    {
        frame.data[i + 1U] = g_isotp.tx_buffer[i];
    }

    /* 邮箱忙时不改变状态，下一个 1 ms 周期重试。 */
    if(CANIF_Send(&frame) == 0U)
    {
        return 0U;
    }

    g_isotp.tx_state = ISOTP_TX_IDLE;
    return 1U;
}

static Uint16 IsoTp_SendFF(void)
{
    CAN_Frame frame;
    Uint16 i;

    IsoTp_PrepareTxFrame(&frame);

    /*
     * 12 bit FF_DL：
     *   Byte0 = 0x1 | Length[11:8]
     *   Byte1 = Length[7:0]
     */
    frame.data[0] = 0x10U | ((g_isotp.tx_length >> 8U) & 0x0FU);
    frame.data[1] = g_isotp.tx_length & 0xFFU;

    /* 8 字节 CAN 帧扣除 2 字节 PCI，FF 可带 6 字节 PDU。 */
    for(i = 0U; i < 6U; i++)
    {
        frame.data[i + 2U] = g_isotp.tx_buffer[i];
    }

    if(CANIF_Send(&frame) == 0U)
    {
        return 0U;
    }

    g_isotp.tx_index = 6U;
    g_isotp.tx_sn = 1U;
    g_isotp.tx_n_bs_timer = 0U;
    g_isotp.tx_state = ISOTP_TX_WAIT_FC;
    return 1U;
}

static Uint16 IsoTp_SendCF(void)
{
    CAN_Frame frame;
    Uint16 remaining;
    Uint16 copy_length;
    Uint16 i;

    IsoTp_PrepareTxFrame(&frame);

    /* PCI 高半字节为 2；低半字节是 0~15 循环的序号。 */
    frame.data[0] = 0x20U | (g_isotp.tx_sn & 0x0FU);

    remaining = g_isotp.tx_length - g_isotp.tx_index;
    copy_length = (remaining > 7U) ? 7U : remaining;
    for(i = 0U; i < copy_length; i++)
    {
        frame.data[i + 1U] = g_isotp.tx_buffer[g_isotp.tx_index + i];
    }

    if(CANIF_Send(&frame) == 0U)
    {
        return 0U;
    }

    g_isotp.tx_index += copy_length;
    g_isotp.tx_sn = (g_isotp.tx_sn + 1U) & 0x0FU;
    g_isotp.tx_bs_counter++;

    if(g_isotp.tx_index >= g_isotp.tx_length)
    {
        g_isotp.tx_state = ISOTP_TX_IDLE;
        return 1U;
    }

    /* BS=0 表示本次发送直到结束都不需要再次等待 FC。 */
    if((g_isotp.tx_bs != 0U) &&
       (g_isotp.tx_bs_counter >= g_isotp.tx_bs))
    {
        g_isotp.tx_bs_counter = 0U;
        g_isotp.tx_n_bs_timer = 0U;
        g_isotp.tx_state = ISOTP_TX_WAIT_FC;
        return 1U;
    }

    if(g_isotp.tx_stmin_ms == 0U)
    {
        /* 任务每 1 ms 运行一次，所以即使 STmin=0 也不会同 tick 连发。 */
        g_isotp.tx_state = ISOTP_TX_SEND_CF;
    }
    else
    {
        g_isotp.tx_stmin_timer = g_isotp.tx_stmin_ms;
        g_isotp.tx_state = ISOTP_TX_WAIT_STMIN;
    }
    return 1U;
}

static Uint16 IsoTp_SendFC(void)
{
    CAN_Frame frame;

    IsoTp_PrepareTxFrame(&frame);
    frame.data[0] = 0x30U | (g_isotp.fc_status & 0x0FU);
    frame.data[1] = ISOTP_RX_BS;
    frame.data[2] = ISOTP_RX_STMIN;

    if(CANIF_Send(&frame) == 0U)
    {
        return 0U;
    }

    g_isotp.fc_pending = 0U;
    if(g_isotp.fc_status == ISOTP_FS_CTS)
    {
        /* N_Cr 从 FC CTS 真正交给 CAN 层后开始计时。 */
        g_isotp.rx_n_cr_timer = 0U;
    }
    return 1U;
}

static void IsoTp_RxSF(const CAN_Frame *frame)
{
    Uint16 length;
    Uint16 i;

    length = frame->data[0] & 0x0FU;
    if((length == 0U) || (length > 7U) ||
       (frame->dlc < (length + 1U)))
    {
        g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
        return;
    }

    if((g_isotp.rx_ready != 0U) ||
       (g_isotp.rx_state == ISOTP_RX_WAIT_CF))
    {
        return;
    }

    for(i = 0U; i < length; i++)
    {
        g_isotp.rx_buffer[i] = frame->data[i + 1U] & 0xFFU;
    }
    g_isotp.rx_length = length;
    g_isotp.rx_index = length;
    g_isotp.rx_ready = 1U;
    g_isotp.rx_state = ISOTP_RX_IDLE;
}

static void IsoTp_RxFF(const CAN_Frame *frame)
{
    Uint16 total_length;
    Uint16 i;

    if(frame->dlc < 8U)
    {
        g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
        return;
    }

    total_length = ((frame->data[0] & 0x0FU) << 8U) |
                   (frame->data[1] & 0xFFU);
    if(total_length <= 7U)
    {
        g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
        return;
    }

    if(total_length > ISOTP_MAX_PDU_LENGTH)
    {
        g_isotp.fc_status = ISOTP_FS_OVFLW;
        g_isotp.fc_pending = 1U;
        g_isotp.error = ISOTP_ERROR_RX_OVERFLOW;
        g_isotp.rx_state = ISOTP_RX_ERROR;
        return;
    }

    if(g_isotp.rx_ready != 0U)
    {
        return;
    }

    g_isotp.rx_length = total_length;
    for(i = 0U; i < 6U; i++)
    {
        g_isotp.rx_buffer[i] = frame->data[i + 2U] & 0xFFU;
    }
    g_isotp.rx_index = 6U;
    g_isotp.rx_expected_sn = 1U;
    g_isotp.rx_bs_counter = 0U;
    g_isotp.rx_n_cr_timer = 0U;
    g_isotp.rx_state = ISOTP_RX_WAIT_CF;

    /* ISR 不发报文；这里只置 pending，由 Task1ms 发送 FC。 */
    g_isotp.fc_status = ISOTP_FS_CTS;
    g_isotp.fc_pending = 1U;
}

static void IsoTp_RxCF(const CAN_Frame *frame)
{
    Uint16 sn;
    Uint16 remaining;
    Uint16 copy_length;
    Uint16 i;

    if(g_isotp.rx_state != ISOTP_RX_WAIT_CF)
    {
        return;
    }

    sn = frame->data[0] & 0x0FU;
    if(sn != g_isotp.rx_expected_sn)
    {
        g_isotp.error = ISOTP_ERROR_WRONG_SN;
        g_isotp.rx_state = ISOTP_RX_ERROR;
        return;
    }

    remaining = g_isotp.rx_length - g_isotp.rx_index;
    copy_length = (remaining > 7U) ? 7U : remaining;
    if(frame->dlc < (copy_length + 1U))
    {
        g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
        g_isotp.rx_state = ISOTP_RX_ERROR;
        return;
    }

    for(i = 0U; i < copy_length; i++)
    {
        g_isotp.rx_buffer[g_isotp.rx_index + i] =
            frame->data[i + 1U] & 0xFFU;
    }

    g_isotp.rx_index += copy_length;
    g_isotp.rx_expected_sn = (g_isotp.rx_expected_sn + 1U) & 0x0FU;
    g_isotp.rx_bs_counter++;
    g_isotp.rx_n_cr_timer = 0U;

    if(g_isotp.rx_index >= g_isotp.rx_length)
    {
        g_isotp.rx_ready = 1U;
        g_isotp.rx_state = ISOTP_RX_IDLE;
        return;
    }

    if((ISOTP_RX_BS != 0U) &&
       (g_isotp.rx_bs_counter >= ISOTP_RX_BS))
    {
        g_isotp.rx_bs_counter = 0U;
        g_isotp.fc_status = ISOTP_FS_CTS;
        g_isotp.fc_pending = 1U;
    }
}

static void IsoTp_RxFC(const CAN_Frame *frame)
{
    Uint16 fs;
    Uint16 decoded_stmin;

    if(g_isotp.tx_state != ISOTP_TX_WAIT_FC)
    {
        return;
    }
    if(frame->dlc < 3U)
    {
        g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
        g_isotp.tx_state = ISOTP_TX_ERROR;
        return;
    }

    fs = frame->data[0] & 0x0FU;
    if(fs == ISOTP_FS_CTS)
    {
        decoded_stmin = IsoTp_DecodeSTmin(frame->data[2]);
        if(decoded_stmin == ISOTP_INVALID_STMIN)
        {
            g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
            g_isotp.tx_state = ISOTP_TX_ERROR;
            return;
        }

        g_isotp.tx_bs = frame->data[1] & 0xFFU;
        g_isotp.tx_bs_counter = 0U;
        g_isotp.tx_stmin_ms = decoded_stmin;
        g_isotp.tx_stmin_timer = 0U;
        g_isotp.tx_n_bs_timer = 0U;
        g_isotp.tx_wait_counter = 0U;
        g_isotp.tx_state = ISOTP_TX_SEND_CF;
    }
    else if(fs == ISOTP_FS_WAIT)
    {
        g_isotp.tx_wait_counter++;
        g_isotp.tx_n_bs_timer = 0U;
        if(g_isotp.tx_wait_counter > ISOTP_WFT_MAX)
        {
            g_isotp.error = ISOTP_ERROR_TOO_MANY_WAIT;
            g_isotp.tx_state = ISOTP_TX_ERROR;
        }
    }
    else if(fs == ISOTP_FS_OVFLW)
    {
        g_isotp.error = ISOTP_ERROR_FC_OVERFLOW;
        g_isotp.tx_state = ISOTP_TX_ERROR;
    }
    else
    {
        g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
        g_isotp.tx_state = ISOTP_TX_ERROR;
    }
}

static void IsoTp_OnCanFrame(const CAN_Frame *frame)
{
    Uint16 frame_type;

    if((frame->id != DIAG_REQUEST_CAN_ID) || (frame->dlc == 0U))
    {
        return;
    }

    frame_type = (frame->data[0] >> 4U) & 0x0FU;
    switch(frame_type)
    {
        case 0U:
            IsoTp_RxSF(frame);
            break;
        case 1U:
            IsoTp_RxFF(frame);
            break;
        case 2U:
            IsoTp_RxCF(frame);
            break;
        case 3U:
            IsoTp_RxFC(frame);
            break;
        default:
            g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
            break;
    }
}

void IsoTp_Init(void)
{
    g_isotp.tx_length = 0U;
    g_isotp.tx_index = 0U;
    g_isotp.tx_sn = 1U;
    g_isotp.tx_bs = 0U;
    g_isotp.tx_bs_counter = 0U;
    g_isotp.tx_stmin_ms = 0U;
    g_isotp.tx_stmin_timer = 0U;
    g_isotp.tx_n_bs_timer = 0U;
    g_isotp.tx_wait_counter = 0U;
    g_isotp.tx_state = ISOTP_TX_IDLE;

    g_isotp.rx_length = 0U;
    g_isotp.rx_index = 0U;
    g_isotp.rx_expected_sn = 1U;
    g_isotp.rx_bs_counter = 0U;
    g_isotp.rx_n_cr_timer = 0U;
    g_isotp.rx_state = ISOTP_RX_IDLE;
    g_isotp.rx_ready = 0U;

    g_isotp.fc_pending = 0U;
    g_isotp.fc_status = ISOTP_FS_CTS;
    g_isotp.error = ISOTP_ERROR_NONE;
}

void IsoTp_Task1ms(void)
{
    CAN_Frame rx_frame;

    /* 一次取空 CAN 队列，CAN ISR 本身不运行协议状态机。 */
    while(CANIF_Read(&rx_frame) != 0U)
    {
        IsoTp_OnCanFrame(&rx_frame);
    }

    /* FC 优先于本 ECU 自己的诊断响应，避免让对方等待。 */
    if(g_isotp.fc_pending != 0U)
    {
        (void)IsoTp_SendFC();
    }

    if((g_isotp.rx_state == ISOTP_RX_WAIT_CF) &&
       (g_isotp.fc_pending == 0U))
    {
        g_isotp.rx_n_cr_timer++;
        if(g_isotp.rx_n_cr_timer >= ISOTP_N_CR_TIMEOUT_MS)
        {
            g_isotp.error = ISOTP_ERROR_N_CR_TIMEOUT;
            g_isotp.rx_state = ISOTP_RX_ERROR;
        }
    }

    switch(g_isotp.tx_state)
    {
        case ISOTP_TX_IDLE:
            break;

        case ISOTP_TX_SEND_SF:
            (void)IsoTp_SendSF();
            break;

        case ISOTP_TX_SEND_FF:
            (void)IsoTp_SendFF();
            break;

        case ISOTP_TX_WAIT_FC:
            g_isotp.tx_n_bs_timer++;
            if(g_isotp.tx_n_bs_timer >= ISOTP_N_BS_TIMEOUT_MS)
            {
                g_isotp.error = ISOTP_ERROR_N_BS_TIMEOUT;
                g_isotp.tx_state = ISOTP_TX_ERROR;
            }
            break;

        case ISOTP_TX_SEND_CF:
            (void)IsoTp_SendCF();
            break;

        case ISOTP_TX_WAIT_STMIN:
            if(g_isotp.tx_stmin_timer > 0U)
            {
                g_isotp.tx_stmin_timer--;
            }
            if(g_isotp.tx_stmin_timer == 0U)
            {
                /* 归零的同一 tick 就发送，避免额外多等 1 ms。 */
                g_isotp.tx_state = ISOTP_TX_SEND_CF;
                (void)IsoTp_SendCF();
            }
            break;

        case ISOTP_TX_ERROR:
            break;

        default:
            g_isotp.error = ISOTP_ERROR_INVALID_FRAME;
            g_isotp.tx_state = ISOTP_TX_ERROR;
            break;
    }
}

Uint16 IsoTp_Send(const Uint16 *data, Uint16 length)
{
    Uint16 i;

    if((data == (const Uint16 *)0) ||
       (g_isotp.tx_state != ISOTP_TX_IDLE) ||
       (length == 0U) || (length > ISOTP_MAX_PDU_LENGTH))
    {
        return 0U;
    }

    for(i = 0U; i < length; i++)
    {
        g_isotp.tx_buffer[i] = data[i] & 0xFFU;
    }

    g_isotp.tx_length = length;
    g_isotp.tx_index = 0U;
    g_isotp.tx_sn = 1U;
    g_isotp.tx_bs = 0U;
    g_isotp.tx_bs_counter = 0U;
    g_isotp.tx_stmin_ms = 0U;
    g_isotp.tx_stmin_timer = 0U;
    g_isotp.tx_n_bs_timer = 0U;
    g_isotp.tx_wait_counter = 0U;
    g_isotp.error = ISOTP_ERROR_NONE;
    g_isotp.tx_state = (length <= 7U) ? ISOTP_TX_SEND_SF : ISOTP_TX_SEND_FF;
    return 1U;
}

Uint16 IsoTp_IsTxIdle(void)
{
    return (g_isotp.tx_state == ISOTP_TX_IDLE) ? 1U : 0U;
}

Uint16 IsoTp_IsRxReady(void)
{
    return g_isotp.rx_ready;
}

const Uint16 *IsoTp_GetRxData(void)
{
    return g_isotp.rx_buffer;
}

Uint16 IsoTp_GetRxLength(void)
{
    return g_isotp.rx_length;
}

void IsoTp_ReleaseRx(void)
{
    g_isotp.rx_ready = 0U;
    g_isotp.rx_length = 0U;
    g_isotp.rx_index = 0U;
}

ISOTP_TX_STATE IsoTp_GetTxState(void)
{
    return g_isotp.tx_state;
}

ISOTP_RX_STATE IsoTp_GetRxState(void)
{
    return g_isotp.rx_state;
}

ISOTP_ERROR IsoTp_GetError(void)
{
    return g_isotp.error;
}

void IsoTp_ClearError(void)
{
    g_isotp.error = ISOTP_ERROR_NONE;
    if(g_isotp.tx_state == ISOTP_TX_ERROR)
    {
        g_isotp.tx_state = ISOTP_TX_IDLE;
    }
    if(g_isotp.rx_state == ISOTP_RX_ERROR)
    {
        g_isotp.rx_state = ISOTP_RX_IDLE;
        g_isotp.rx_length = 0U;
        g_isotp.rx_index = 0U;
    }
}

