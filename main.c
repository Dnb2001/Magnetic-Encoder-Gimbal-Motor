#include "DSP2833x_Device.h"  // 设备头文件
#include "DSP2833x_Examples.h"
#include <math.h>
#include "my_foc.h"
#include "my_epwm.h"
#include "my_adc.h"
#include "my_spi.h"
#include "my_can.h"
#include "my_qep.h"
Uint16 run_flag = 0;      // 1: 开始呼吸, 0: 停止呼吸
Uint16 mode = 0;          // 0: 自动呼吸模式, 1: 手动调节模式

int test_sum = 0;
int can_send_flag = 1;
// --- 虚拟角度测试变量 ---
float freq_target_hz = 1.0f;  // 目标电频率
#define PWM_FREQ  10000.0f    // PWM 频率
float TPWM = 1.0f/PWM_FREQ;
float angle_step;      // 角度增量
AS5047_FRAME_t debug_frame;

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

void ClearTzFault(void)
{
    EALLOW;
    // 清除 PWM1, 2, 3 的所有 TZ 标志位
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    // 如果故障已排除，此时波形会立即恢复
    EDIS;
}


Uint32 spi_err_cnt = 0;  // SPI 误码计数器
void main(void)
{
    fault_flag = 0;
    angle_step = freq_target_hz * 2 * 3.1415f * TPWM;      // 角度增量
InitSysCtrl();              //系统初始化
InitPieCtrl();              // 初始化 PIE 控制寄存器到默认状态
    IER = 0x0000;           // 禁用 CPU 中断
    IFR = 0x0000;           // 清除 CPU 中断标志
InitPieVectTable();         // 初始化 PIE 向量表（默认映射）

// --- 第二步：重映射中断向量 ---
    EALLOW;
    PieVectTable.ADCINT = &adc_isr;               // 登记 ADC 中断！
    PieVectTable.EPWM1_TZINT = &epwm1_tz_int_isr; // 登记地址 IR2136 fault处理 TZ保护
      //PieVectTable.EPWM1_INT = &epwm1_isr;  不再使用ePWM中断触发ADC 这样会导致竞争 改为ADC采样结束后 在ADC中断跑FOC 更新占空比
    EDIS;

// --- 第三步：配置硬件外设 ---
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;    // 开启 ADC 模块时钟
    SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // 开1时钟
    SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // 开2时钟
    SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // 开2时钟
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // 先停止同步
    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1; // 开启 SPI-A 模块的时钟
    SysCtrlRegs.PCLKCR0.bit.ECANBENCLK = 1;  // 使能eCANB时钟
    EDIS;

    InitAdc_User();             // 初始化ADC
    Init_Control_Vars();
    InitEPwm1(); // 周期 相位 计数器初始值 增减模式 时钟 CMPA初值 动作 死区 触发ADC 同步
    InitEPwm2();
    InitEPwm3();
    InitTzPwm();
    //InitSpiaGpio();
    Init_Spia_AS5047P();
    Clear_AS5047P_Error();
    Init_eCANB_Config();
    InitEQep1_AS5047P();
//  -- 配置 GPIO 引脚 功能
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;      // GPIO0 -> EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;      // GPIO1 -> EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;      // GPIO2 -> EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;      // GPIO3 -> EPWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;      // GPIO4 -> EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;      // GPIO5 -> EPWM3B
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 2;      // GPIO16 -> CANTXB
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 2;      // GPIO17 -> CANRXB
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;       // 极重要：开启 CANRXB(GPIO17) 的内部上拉电阻
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;     // 将 GPIO17 设为异步输入，避免 CPU 采样窗延迟

    // TZ保护引脚
    // 1. 将 GPIO12 设置为 TZ1 功能引脚
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;
    // 2. 核心设置：异步输入 (Asynchronous)，不经过采样窗口，响应最快
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;
    // 3. 开启内部上拉，确保在断线时不会误触发
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;

    // IR2136驱动芯片使能引脚
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;   // 必须设为 0，确保是 GPIO 模式
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;    // 设为 1，确保是输出模式
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;  // CLEAR 写入时清零关闭使能

    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;   //启动所有 PWM 的全局时钟

    EDIS;

// --- 第四步：打通中断通路 ---
// 1. 使能 PIE 组 3 的第 1 条线 (对应 ePWM1)
    //PieCtrlRegs.PIEIER3.bit.INTx1 = 1; 不要ePWM中断了
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1; // <--- 开启 PIE Group 2 的第 1 条线 (TZ 中断)
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;     // ADC 中断在第一组的第 6 根线，必须设为 1 开启
// 2. 使能 CPU 级中断 (ePWM1-6 都在 INT3)
    IER |= M_INT1; // 使能 CPU 级中断 Group 1 (ADC 在这里)
    IER |= M_INT2; // 使能 CPU 级中断 Group 2 (TZ 在这里)
    //IER |= M_INT3;  不再使用ePWM中断

    //Read_Phase_Current_Zero();  //读取霍尔零位
    sys_state = STATE_IDLE;
// 3. 开启全局中断
    EINT;   // 使能总中断
    ERTM;   // 使能实时中断

    /* 如果在flash里运行 需要下面两行
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash(); // 如果是 Flash 运行，还需要初始化 Flash 寄存器
    */
//----------------------初始化结束----------------------------//

    Alignment_flag = 0;
    //Motor_Alignment();  // 对齐

    command = 0x3FFF;

while(1)
{
    //ClearTzFault();
    //fault_flag = 0;
    //DELAY_US(10000);
    Check_CAN_Receive();
    //can_send_flag = Send_Message_WithTimeout();
    // 接收可以一直轮询，因为不怎么耗时
            Check_CAN_Receive();

            // 检查是否有 10ms 快照数据准备好
            if (can_tx_ready_flag == 1)
            {
                // 【进阶技巧：临界区保护】
                // 在把内存里的 TxMsg 搬运到 CAN 硬件寄存器的这几微秒内，
                // 关一下全局中断，防止刚好碰到下个 10ms 到来把 TxMsg 覆盖掉
                DINT;
                ECanbMboxes.MBOX1.MDL.all = SWAP_BYTES32(TxMsg_101_MDL.all);
                ECanbMboxes.MBOX1.MDH.all = SWAP_BYTES32(TxMsg_101_MDH.all);
                ECanbMboxes.MBOX2.MDL.all = SWAP_BYTES32(TxMsg_102_MDL.all);
                ECanbMboxes.MBOX2.MDH.all = SWAP_BYTES32(TxMsg_102_MDH.all);
                EINT;

                // 触发硬件发送 (邮箱2)
                ECanbRegs.CANTRS.bit.TRS1 = 1;
                ECanbRegs.CANTRS.bit.TRS2 = 1;
                // 清除标志位，等待下一个 10ms
                can_tx_ready_flag = 0;
            }

}


}

