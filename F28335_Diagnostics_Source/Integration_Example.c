/*
 * Integration_Example.c
 *
 * 这个文件展示“放在哪里调用”，不负责初始化系统时钟、GPIO、CANBTC
 * 或 PIE 向量。把三个函数中的调用合并进你的现有工程即可。
 */
#include "Diagnostic.h"
#include "CANIF.h"

/*
 * 调用顺序：
 *   InitSysCtrl / InitECanGpio / 你现有的 eCAN 波特率初始化
 *       -> Application_InitDiagnostics
 *
 * CANIF_Init 会配置 MBOX3、MBOX4，所以必须放在基础 eCAN 初始化之后。
 */
void Application_InitDiagnostics(void)
{
    Diagnostic_Init();
}

/* 放进准确的 1 ms 周期任务，不要放进 PWM/FOC 的 10 kHz ADC ISR。 */
void Application_DiagnosticTask1ms(void)
{
    Diagnostic_Task1ms();
}

/*
 * 从你现有的 ECAN0INTA ISR 中调用此函数。
 * PIEACK 仍由最外层 ISR 统一处理，避免一个工程里重复确认中断组。
 */
void Application_ECan0InterruptBody(void)
{
    CANIF_RxISRHandler();
}

/*
 * 典型 ISR 结构（复制到你现有的中断文件，而不是取消下面的注释）：
 *
 * interrupt void ECAN0INTA_ISR(void)
 * {
 *     Application_ECan0InterruptBody();
 *     PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
 * }
 */

