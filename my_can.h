#ifndef MY_CAN_H_
#define MY_CAN_H_
// 将 32 位数据的字节顺序完全反转
#define SWAP_BYTES32(val) ( \
    ((val & 0xFF000000) >> 24) | \
    ((val & 0x00FF0000) >> 8)  | \
    ((val & 0x0000FF00) << 8)  | \
    ((val & 0x000000FF) << 24)   \
)
#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
// 定义发送结果状态
#define CAN_SUCCESS 0
#define CAN_ERR_TIMEOUT 1
void Init_eCANB_Config(void);
Uint16 Send_Message_WithTimeout(void);
void Check_CAN_Receive(void);

// ==========================================
// 报文 0x201 接收解析结构 (修正版)
// ==========================================
union CAN_RX_MSG_201_MDL {
    Uint32 all;
    struct {
        // --- 第一个 16 位字 (Word 0: bits 0-15) ---
        Uint16 IR2136_EN:1;    // bit 0 (Byte 0)
        Uint16 Run_Flag:1;     // bit 1
        Uint16 TZ_Fault:1;     // bit 2
        Uint16 Feedforward:1;  // bit 3 前馈
        Uint16 DeadZone:1;  // bit 4 前馈
        Uint16 Mode:3;         // bit 5-7
        Uint16 Speed_Ref_L:8; // bit 8-15
        // --- 第二个 16 位字 (Word 0: bits 0-15) ---
        Uint16 Speed_Ref_H:8; // bit 16-23
        Uint16 Reserved:8;     // bit 24-31 (务必加上这行补满 32 位)

    } bit;
};
// ==========================================
// TX 报文 0x101: 电压与相电流
// ==========================================
union CAN_TX_MSG_101_MDL {
    Uint32 all;
    struct {
        Uint16 Vbus_Raw:16;     // Byte 0-1 (Unsigned, Factor 0.1, Offset 0)
        Uint16 Ia_Raw:16;       // Byte 2-3 (Signed, Factor 0.01, Offset 0)
    } bit;
};

union CAN_TX_MSG_101_MDH {
    Uint32 all;
    struct {
        Uint16 Ib_Raw:16;       // Byte 4-5 (Signed, Factor 0.01, Offset 0)
        Uint16 Ic_Raw:16;       // Byte 6-7 (Signed, Factor 0.01, Offset 0)
    } bit;
};
// ==========================================
// TX 报文 0x102: 控制环路数据
// ==========================================
union CAN_TX_MSG_102_MDL {
    Uint32 all;
    struct {
        Uint16 Theta_Raw:16;    // Byte 0-1 (Unsigned, Factor 0.01) 电角度 0~360
        Uint16 Speed_Raw:16;    // Byte 2-3 (Signed, Factor 1)
    } bit;
};

union CAN_TX_MSG_102_MDH {
    Uint32 all;
    struct {
        Uint16 Iq_Ref_Raw:16;   // Byte 4-5 (Signed, Factor 0.01)
        Uint16 Iq_Fdb_Raw:16;   // Byte 6-7 (Signed, Factor 0.01)
    } bit;
};
extern union CAN_RX_MSG_201_MDL RxMsg_0x201_MDL;
extern union CAN_TX_MSG_101_MDL TxMsg_101_MDL;
extern union CAN_TX_MSG_101_MDH TxMsg_101_MDH;
extern union CAN_TX_MSG_102_MDL TxMsg_102_MDL;
extern union CAN_TX_MSG_102_MDH TxMsg_102_MDH;
#endif /* MY_CAN_H_ */
