/*
 * Diagnostic_Config.h
 *
 * 诊断模块的项目级配置集中放在这里。
 * 修改 CAN ID、邮箱号或超时时间时，不需要改协议状态机。
 */
#ifndef DIAGNOSTIC_CONFIG_H_
#define DIAGNOSTIC_CONFIG_H_

#define DIAG_REQUEST_CAN_ID          0x7E0U
#define DIAG_RESPONSE_CAN_ID         0x7E8U

#define DIAG_ECAN_TX_MAILBOX         3U
#define DIAG_ECAN_RX_MAILBOX         4U

/*
 * 环形队列会空出一个槽区分“空/满”，16 个槽实际可存 15 帧。
 * 这足以容纳 1 个 FF 加上本 ECU 宣告的一个 BS=8 数据块。
 */
#define CANIF_RX_QUEUE_SIZE          16U
#define ISOTP_MAX_PDU_LENGTH         512U
#define ISOTP_PADDING_BYTE           0xAAU

#define ISOTP_N_BS_TIMEOUT_MS        1000U
#define ISOTP_N_CR_TIMEOUT_MS        1000U
#define ISOTP_RX_BS                  8U
#define ISOTP_RX_STMIN               0U
#define ISOTP_WFT_MAX                3U

#define UDS_S3_SERVER_MS             5000UL

#if CANIF_RX_QUEUE_SIZE < (ISOTP_RX_BS + 2U)
#error CANIF_RX_QUEUE_SIZE_must_hold_one_FF_and_one_CF_block
#endif

#if ISOTP_MAX_PDU_LENGTH > 4095U
#error Classic_CAN_12_bit_FF_length_cannot_exceed_4095
#endif

#endif /* DIAGNOSTIC_CONFIG_H_ */
