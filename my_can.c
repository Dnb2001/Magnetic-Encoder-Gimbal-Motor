#include<my_can.h>
#include<my_foc.h>
#include <stdint.h>

union CAN_RX_MSG_201_MDL RxMsg_0x201_MDL;
//union CAN_RX_MSG_201_MDL RxMsg_0x201_MDH;
 union CAN_TX_MSG_101_MDL TxMsg_101_MDL;
 union CAN_TX_MSG_101_MDH TxMsg_101_MDH;
 union CAN_TX_MSG_102_MDL TxMsg_102_MDL;
 union CAN_TX_MSG_102_MDH TxMsg_102_MDH;
Uint32 temp_mdl = 0x11223344;
Uint32 temp_mdh = 0x11223344;

void Init_eCANB_Config(void)
{
    EALLOW;
    ECanbRegs.CANMC.bit.CCR = 1;      // 1. 请求进入配置模式
    EDIS;
    while(ECanbRegs.CANES.bit.CCE != 1);

    EALLOW;
    ECanbRegs.CANTIOC.bit.TXFUNC = 1;  // 1: 使能 CANTX 引脚功能
    ECanbRegs.CANRIOC.bit.RXFUNC = 1;  // 1: 使能 CANRX 引脚功能

    // --- 必须在此处配置波特率和方向 ---
    ECanbRegs.CANBTC.bit.BRPREG = 9;
    ECanbRegs.CANBTC.bit.TSEG1REG = 10;
    ECanbRegs.CANBTC.bit.TSEG2REG = 2;
    ECanbRegs.CANBTC.bit.SJWREG = 1;

    // 配置 发送/接收 方向
    ECanbRegs.CANMD.all = 0x00000001;

    // 建议先关闭 STM 模式进行真实总线测试
    ECanbRegs.CANMC.bit.STM = 0;

    ECanbRegs.CANMC.bit.CCR = 0;      // 2. 退出配置模式
    EDIS;
    while(ECanbRegs.CANES.bit.CCE != 0);

    // --- 剩下的 ID 和 DLC 配置可以在此处 ---
    EALLOW;
    ECanbRegs.CANME.bit.ME0 = 0;            // 1. 修改前先禁用邮箱0
    ECanbRegs.CANME.bit.ME1 = 0;            //
    ECanbRegs.CANME.bit.ME2 = 0;
    ECanbMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
    ECanbMboxes.MBOX0.MSGID.bit.IDE = 0;

    ECanbMboxes.MBOX0.MSGID.bit.STDMSGID = 0x201;
    ECanbMboxes.MBOX0.MSGID.bit.IDE = 0;      // 关键：设为 0 (标准帧)，否则收不到 11位 ID
    ECanbMboxes.MBOX0.MSGID.bit.AME = 0;      // 关键：设为 0 (禁用屏蔽位)，只进行 ID 全匹配
    ECanbMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
    ECanbRegs.CANME.bit.ME0 = 1;      //  重新使能邮箱 1

    ECanbMboxes.MBOX1.MSGID.bit.STDMSGID = 0x101;
    ECanbMboxes.MBOX1.MSGID.bit.IDE = 0;      // 关键：设为 0 (标准帧)，否则收不到 11位 ID
    ECanbMboxes.MBOX1.MSGID.bit.AME = 0;      // 关键：设为 0 (禁用屏蔽位)，只进行 ID 全匹配
    ECanbMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
    ECanbRegs.CANME.bit.ME1 = 1;

    ECanbMboxes.MBOX2.MSGID.bit.STDMSGID = 0x102;
    ECanbMboxes.MBOX2.MSGID.bit.IDE = 0;      // 关键：设为 0 (标准帧)，否则收不到 11位 ID
    ECanbMboxes.MBOX2.MSGID.bit.AME = 0;      // 关键：设为 0 (禁用屏蔽位)，只进行 ID 全匹配
    ECanbMboxes.MBOX2.MSGCTRL.bit.DLC = 8;
    ECanbRegs.CANME.bit.ME2 = 1;
//    ECanbMboxes.MBOX3.MSGID.bit.STDMSGID = 0x103;
//    ECanbRegs.CANME.bit.ME3 = 1;

    EDIS;
}


Uint16 Send_Message_WithTimeout(void) {
    Uint32 timeout_count = 0;
    const Uint32 MAX_TIMEOUT = 10000; // 根据主频调整，150MHz 下约几百微秒

    // 1. 填充数据 (保持你之前的逻辑)

    ECanbMboxes.MBOX1.MDL.all = temp_mdl;
    ECanbMboxes.MBOX1.MDH.all = temp_mdh;
    ECanbMboxes.MBOX2.MDL.all = temp_mdl;
    ECanbMboxes.MBOX2.MDH.all = temp_mdh;
    // 2. 请求发送
    ECanbRegs.CANTRS.bit.TRS2 = 1;
    ECanbRegs.CANTRS.bit.TRS1 = 1;
    // 3. 带超时的等待
    while( (ECanbRegs.CANTA.bit.TA1 == 0) || (ECanbRegs.CANTA.bit.TA2 == 0) ) {
        timeout_count++;
        if(timeout_count > MAX_TIMEOUT) {
            // 如果超时，通常是硬件没接好、波特率不对或没有 120 欧电阻
            // 建议：在此处可以记录一个错误日志，但不卡死程序
            return CAN_ERR_TIMEOUT;
        }
    }

    // 4. 发送成功，清除标志
    ECanbRegs.CANTA.bit.TA1 = 1;
    ECanbRegs.CANTA.bit.TA2 = 1;
    return CAN_SUCCESS;
}


int16_t physical_speed;
void Check_CAN_Receive(void)
{
    if(ECanbRegs.CANRMP.bit.RMP0 == 1)
    {
        // 1. 读取并整体反转字节序
        Uint32 raw_mdl = ECanbMboxes.MBOX0.MDL.all;
        Uint32 raw_mdh = ECanbMboxes.MBOX0.MDH.all;

        RxMsg_0x201_MDL.all = SWAP_BYTES32(raw_mdl);
       // RxMsg_0x201_MDH.all = SWAP_BYTES32(raw_mdh);

        // 2. 标志位处理 (现在对应 bit 0-2 了)
        if (RxMsg_0x201_MDL.bit.IR2136_EN == 1 && sys_state == STATE_IDLE )
         {
            sys_state = STATE_CALIB;
          }
        if (RxMsg_0x201_MDL.bit.IR2136_EN == 0  ) sys_state = STATE_IDLE;

        // 3. 核心修复：手动拼接转速 (High << 8 | Low)
        Uint16 raw_speed = ((Uint16)RxMsg_0x201_MDL.bit.Speed_Ref_H << 8) |
                            (Uint16)RxMsg_0x201_MDL.bit.Speed_Ref_L;

        // 4. 物理值计算 (Raw - 32767)
        physical_speed = (int16_t)((int32_t)raw_speed - 32767);
        speed_target = physical_speed;



        // 清除标志
        ECanbRegs.CANRMP.bit.RMP0 = 1;
    }
}
