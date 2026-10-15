/*
 * CANIF.c
 *
 * F28335 eCAN-A 诊断邮箱适配�?
 *   MBOX3：发送，标准 ID 0x7E8
 *   MBOX4：接收，标准 ID 0x7E0
 *
 * 重要边界�?
 *   1. 本文件不配置 GPIO、eCAN 时钟�? CANBTC�?
 *   2. MBOX0~2 已被用户占用，任何读改写都必须保留它们；
 *   3. eCAN 控制/状态寄存器必须通过 shadow �? 32 bit 访问�?
 *   4. ISR 只搬运数据，不运�? ISO-TP �? UDS�?
 */
#include "CANIF.h"
#include "CANIF_ByteCodec.h"

static CAN_Frame g_can_rx_queue[CANIF_RX_QUEUE_SIZE];
static volatile Uint16 g_can_rx_read;
static volatile Uint16 g_can_rx_write;
static volatile Uint32 g_can_rx_overflow_count;

static Uint16 CANIF_NextQueueIndex(Uint16 index)
{
    index++;
    if(index >= CANIF_RX_QUEUE_SIZE)
    {
        index = 0U;
    }
    return index;
}

/*
 * 单生产者（ISR�?/单消费者（1 ms Task）环形队列�?
 * 数据全部复制完成后才发布 write 指针，Task 不会读到半帧数据�?
 */
static void CANIF_QueuePushFromISR(const CAN_Frame *frame)
{
    Uint16 next_write;
    Uint16 i;

    next_write = CANIF_NextQueueIndex(g_can_rx_write);
    if(next_write == g_can_rx_read)
    {
        g_can_rx_overflow_count++;
        return;
    }

    g_can_rx_queue[g_can_rx_write].id = frame->id;
    g_can_rx_queue[g_can_rx_write].dlc = frame->dlc;
    for(i = 0U; i < 8U; i++)
    {
        g_can_rx_queue[g_can_rx_write].data[i] = frame->data[i] & 0xFFU;
    }

    g_can_rx_write = next_write;
}

void CANIF_Init(void)
{
    struct ECAN_REGS ECanaShadow;

    g_can_rx_read = 0U;
    g_can_rx_write = 0U;
    g_can_rx_overflow_count = 0UL;

    EALLOW;

    /*
     * 配置 MSGID/MSGCTRL 前必须先禁用对应邮箱�?
     * 使用读改写，只清 ME3/ME4，保�? MBOX0~2 和其他邮箱�?
     */
    ECanaShadow.CANME.all = ECanbRegs.CANME.all;
    ECanaShadow.CANME.bit.ME3 = 0U;
    ECanaShadow.CANME.bit.ME4 = 0U;
    ECanbRegs.CANME.all = ECanaShadow.CANME.all;

    /*
     * 标准 11 bit ID 位于 MSGID[28:18]�?
     * 整体赋值同时令 IDE=0、AME=0、AAM=0�?
     */
    ECanbMboxes.MBOX3.MSGID.all =
        ((Uint32)(DIAG_RESPONSE_CAN_ID & 0x07FFU)) << 18U;
    ECanbMboxes.MBOX4.MSGID.all =
        ((Uint32)(DIAG_REQUEST_CAN_ID & 0x07FFU)) << 18U;

    /* TI 要求初始化时先把 MSGCTRL 的所有位清零�? */
    ECanbMboxes.MBOX3.MSGCTRL.all = 0UL;
    ECanbMboxes.MBOX4.MSGCTRL.all = 0UL;
    ECanbMboxes.MBOX3.MSGCTRL.bit.DLC = 8U;
    ECanbMboxes.MBOX4.MSGCTRL.bit.DLC = 8U;

    /* CANMD=0 是发送邮箱，CANMD=1 是接收邮箱�? */
    ECanaShadow.CANMD.all = ECanbRegs.CANMD.all;
    ECanaShadow.CANMD.bit.MD3 = 0U;
    ECanaShadow.CANMD.bit.MD4 = 1U;
    ECanbRegs.CANMD.all = ECanaShadow.CANMD.all;

    /* 只让接收邮箱 MBOX4 产生邮箱中断�? */
    ECanaShadow.CANMIM.all = ECanbRegs.CANMIM.all;
    ECanaShadow.CANMIM.bit.MIM3 = 0U;
    ECanaShadow.CANMIM.bit.MIM4 = 1U;
    ECanbRegs.CANMIM.all = ECanaShadow.CANMIM.all;

    /* MIL4=0：MBOX4 分配�? eCAN interrupt line 0�? */
    ECanaShadow.CANMIL.all = ECanbRegs.CANMIL.all;
    ECanaShadow.CANMIL.bit.MIL4 = 0U;
    ECanbRegs.CANMIL.all = ECanaShadow.CANMIL.all;

    /*
     * 单个接收邮箱不启�? OPC。新帧覆盖期间，ISR 通过�? RMP、读取�?
     * 再检�? RMP 的官方流程判断数据是否稳定�?
     */
    ECanaShadow.CANOPC.all = ECanbRegs.CANOPC.all;
    ECanaShadow.CANOPC.bit.OPC4 = 0U;
    ECanbRegs.CANOPC.all = ECanaShadow.CANOPC.all;

    /* 打开 eCAN0INT；不�? GIL 和其他全局错误中断选择�? */
    ECanaShadow.CANGIM.all = ECanbRegs.CANGIM.all;
    ECanaShadow.CANGIM.bit.I0EN = 1U;
    ECanbRegs.CANGIM.all = ECanaShadow.CANGIM.all;

    /* 清本模块使用的旧状态，�? 1 只作用于 TA3 / RMP4�? */
    ECanaShadow.CANTA.all = 0UL;
    ECanaShadow.CANTA.bit.TA3 = 1U;
    ECanbRegs.CANTA.all = ECanaShadow.CANTA.all;

    ECanaShadow.CANRMP.all = 0UL;
    ECanaShadow.CANRMP.bit.RMP4 = 1U;
    ECanbRegs.CANRMP.all = ECanaShadow.CANRMP.all;

    /* 最后启�? MBOX3/4，仍然保留所有其他邮箱的 ME 位�? */
    ECanaShadow.CANME.all = ECanbRegs.CANME.all;
    ECanaShadow.CANME.bit.ME3 = 1U;
    ECanaShadow.CANME.bit.ME4 = 1U;
    ECanbRegs.CANME.all = ECanaShadow.CANME.all;

    EDIS;
}

Uint16 CANIF_Send(const CAN_Frame *frame)
{
    struct ECAN_REGS ECanaShadow;
    Uint16 dbo;

    if((frame == (const CAN_Frame *)0) ||
       (frame->id != DIAG_RESPONSE_CAN_ID) ||
       (frame->dlc > 8U))
    {
        return 0U;
    }

    /*
     * TRS3=1 表示上一帧仍在等待发送或重发�?
     * 此时硬件会拒绝写 mailbox RAM，软件也绝不能覆盖原数据�?
     */
    ECanaShadow.CANTRS.all = ECanbRegs.CANTRS.all;
    if(ECanaShadow.CANTRS.bit.TRS3 != 0U)
    {
        return 0U;
    }

    /* 读取现有全局 DBO，不修改它，避免影响 MBOX0~2�? */
    ECanaShadow.CANMC.all = ECanbRegs.CANMC.all;
    dbo = ECanaShadow.CANMC.bit.DBO;

    ECanbMboxes.MBOX3.MDL.all = CANIF_PackMDL(frame->data, dbo);
    ECanbMboxes.MBOX3.MDH.all = CANIF_PackMDH(frame->data, dbo);
    ECanbMboxes.MBOX3.MSGCTRL.bit.DLC = frame->dlc;

    /* 清上次发送完成标志，再置�? TRS3 请求本次发送�? */
    ECanaShadow.CANTA.all = 0UL;
    ECanaShadow.CANTA.bit.TA3 = 1U;
    ECanbRegs.CANTA.all = ECanaShadow.CANTA.all;

    ECanaShadow.CANTRS.all = 0UL;
    ECanaShadow.CANTRS.bit.TRS3 = 1U;
    ECanbRegs.CANTRS.all = ECanaShadow.CANTRS.all;
    return 1U;
}

Uint16 CANIF_Read(CAN_Frame *frame)
{
    Uint16 i;

    if((frame == (CAN_Frame *)0) || (g_can_rx_read == g_can_rx_write))
    {
        return 0U;
    }

    frame->id = g_can_rx_queue[g_can_rx_read].id;
    frame->dlc = g_can_rx_queue[g_can_rx_read].dlc;
    for(i = 0U; i < 8U; i++)
    {
        frame->data[i] = g_can_rx_queue[g_can_rx_read].data[i];
    }

    g_can_rx_read = CANIF_NextQueueIndex(g_can_rx_read);
    return 1U;
}

void CANIF_RxISRHandler(void)
{
    struct ECAN_REGS ECanaShadow;
    CAN_Frame frame;
    Uint32 mdl;
    Uint32 mdh;
    Uint16 dbo;
    Uint16 attempt;

    /*
     * 官方建议：先�? RMP，再�? mailbox，读完后重新检�? RMP�?
     * 若读取期间又来了新帧，RMP4 会再次置位，本次数据可能不一致，
     * 因此重新读取。限制为 3 次，避免极端总线负载�? ISR 无限循环�?
     */
    for(attempt = 0U; attempt < 3U; attempt++)
    {
        ECanaShadow.CANRMP.all = ECanbRegs.CANRMP.all;
        if(ECanaShadow.CANRMP.bit.RMP4 == 0U)
        {
            return;
        }

        ECanaShadow.CANRML.all = ECanbRegs.CANRML.all;
        if(ECanaShadow.CANRML.bit.RML4 != 0U)
        {
            g_can_rx_overflow_count++;
        }

        ECanaShadow.CANRMP.all = 0UL;
        ECanaShadow.CANRMP.bit.RMP4 = 1U;
        ECanbRegs.CANRMP.all = ECanaShadow.CANRMP.all;

        frame.id = (Uint16)((ECanbMboxes.MBOX4.MSGID.all >> 18U) &
                            0x07FFUL);
        frame.dlc = ECanbMboxes.MBOX4.MSGCTRL.bit.DLC;
        if(frame.dlc > 8U)
        {
            frame.dlc = 8U;
        }

        mdl = ECanbMboxes.MBOX4.MDL.all;
        mdh = ECanbMboxes.MBOX4.MDH.all;
        ECanaShadow.CANMC.all = ECanbRegs.CANMC.all;
        dbo = ECanaShadow.CANMC.bit.DBO;
        CANIF_UnpackData(mdl, mdh, dbo, frame.data);

        ECanaShadow.CANRMP.all = ECanbRegs.CANRMP.all;
        if(ECanaShadow.CANRMP.bit.RMP4 == 0U)
        {
            CANIF_QueuePushFromISR(&frame);
            return;
        }
    }

    /* 新帧持续覆盖，留�? RMP4 给下一次中断再次处理�? */
    g_can_rx_overflow_count++;
}

Uint32 CANIF_GetRxOverflowCount(void)
{
    return g_can_rx_overflow_count;
}

