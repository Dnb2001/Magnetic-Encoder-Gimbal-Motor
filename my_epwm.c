#include "my_epwm.h"
#include "my_adc.h"
#include "my_foc.h"

#define TBPRD_ALL 3750      // 计数周期
#define TBPRD_init 1875     // 计数器初始值

void InitTzPwm(void)
{
    EALLOW;
    // 1. 使能 OSHT (One-Shot) 触发源为 TZ1
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;
    EPwm3Regs.TZSEL.bit.OSHT1 = 1;
    // 2. 定义触发后的动作：强制 A 路和 B 路输出为低电平 (最安全)
    // 0: 高阻, 1: 强制高, 2: 强制低, 3: 无动作
    EPwm1Regs.TZCTL.bit.TZA = 2;
    EPwm1Regs.TZCTL.bit.TZB = 2;
    EPwm2Regs.TZCTL.bit.TZA = 2;
    EPwm2Regs.TZCTL.bit.TZB = 2;
    EPwm3Regs.TZCTL.bit.TZA = 2;
    EPwm3Regs.TZCTL.bit.TZB = 2;
    // 3. 使能 TZ 中断信号产生
    EPwm1Regs.TZEINT.bit.OST = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;
    EPwm3Regs.TZEINT.bit.OST = 1;
    EDIS;
}
void InitEPwm1()  // 周期 相位 计数器初始值 增减模式 时钟 CMPA初值 动作 死区 触发ADC 同步
{
    // --- TB 子模块配置 ---
    EPwm1Regs.TBPRD = TBPRD_ALL;             // 设置周期 (假设 150MHz 系统频率，得 10kHz PWM)
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000; // 相位为 0
    EPwm1Regs.TBCTR = 0x0000;            // 清零计数器

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // 增减计数模式（对称波形）
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // 系统时钟不分频
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    // --- CC 子模块配置 ---
    EPwm1Regs.CMPA.half.CMPA = TBPRD_init;     // 初始占空比 50%
    // --- AQ 子模块配置 (定义 PWM1A 的动作) ---
    // 向上计数时比较匹配拉低，向下计数时比较匹配拉高
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    // --- DB 死区模块配置 (实现互补+死区) ---
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 开启双边死区
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // 极性选择：A路不反向，B路反向（实现互补）
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // 源输入选择 EPWM1A
    EPwm1Regs.DBRED = 150;    // 上升沿死区时间 (RED) - 450个时钟周期 (约 3us)
    EPwm1Regs.DBFED = 150;    // 下降沿死区时间 (FED) - 450个时钟周期
    // 设置 SOCA (Start of Conversion A)
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // 使能 SOCA
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO; // 【关键】在计数器归零时触发 (PWM波形的波谷，通常噪音最小)
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;   // 每次事件都触发 (1st event)
    /* ... 触发CPU中断配置 ... ADC当总指挥后 也不需要了
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // 计数器减到0时产生中断
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // 使能中断
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // 每次事件都产生中断
    */
    // 同步其他EPwm2和EPwm3
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // 老大不需要同步别人
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // 计数到 0 时发信号给小弟们
}
void InitEPwm2()
{
    // --- TB 子模块配置 ---
    EPwm2Regs.TBPRD = TBPRD_ALL;             // 设置周期
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000; // 相位为 0
    EPwm2Regs.TBCTR = 0x0000;            // 清零计数器
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // 增减计数模式（对称波形）
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // 系统时钟不分频
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    // --- CC 子模块配置 ---
    EPwm2Regs.CMPA.half.CMPA = TBPRD_init;     // 初始占空比 50%
    // --- AQ 子模块配置 (定义 PWM1A 的动作) ---
    // 向上计数时比较匹配拉低，向下计数时比较匹配拉高
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    // --- DB 死区模块配置 (实现互补+死区) ---
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 开启双边死区
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // 极性选择：A路不反向，B路反向（实现互补）
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // 源输入选择 EPWM1A
    EPwm2Regs.DBRED = 150;    // 上升沿死区时间 (RED) - 4500个时钟周期 (约 3us)
    EPwm2Regs.DBFED = 150;    // 下降沿死区时间 (FED) - 450个时钟周期
    // --- 【关键：同步设置】 ---
    EPwm2Regs.TBPHS.half.TBPHS = 0;           // 相位偏移设为 0 (绝对对齐)
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;    // 使能相位加载 (听老大的同步信号)
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;       // 同步后向上计数
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // 将同步信号继续传给 ePWM3 (备用)
}
void InitEPwm3()
{
    // --- TB 子模块配置 ---
    EPwm3Regs.TBPRD = TBPRD_ALL;             // 设置周期
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000; // 相位为 0
    EPwm3Regs.TBCTR = 0x0000;            // 清零计数器
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // 增减计数模式（对称波形）
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // 系统时钟不分频
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    // --- CC 子模块配置 ---
    EPwm3Regs.CMPA.half.CMPA = TBPRD_init;     // 初始占空比 50%
    // --- AQ 子模块配置 (定义 PWM3A 的动作) ---
    // 向上计数时比较匹配拉低，向下计数时比较匹配拉高
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    // --- DB 死区模块配置 (实现互补+死区) ---
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 开启双边死区
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // 极性选择：A路不反向，B路反向（实现互补）
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // 源输入选择 EPWM3A
    EPwm3Regs.DBRED = 150;    // 上升沿死区时间 (RED) - 450个时钟周期 (约 3us)
    EPwm3Regs.DBFED = 150;    // 下降沿死区时间 (FED) - 450个时钟周期
    // --- 【关键：同步设置】 ---
    EPwm3Regs.TBPHS.half.TBPHS = 0;           // 相位偏移设为 0 (绝对对齐)
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;    // 使能相位加载 (听老大的同步信号)
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;       // 同步后向上计数
}

int fault_flag;
__interrupt void epwm1_tz_int_isr(void)
{
    fault_flag = 1;
    // 1. 软件紧急停机逻辑
    run_flag = 0; // 关闭你的主算法开关
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1; // 拉低 IR2136 EN 脚，双重保险关闭驱动器     使用的是clear 写1时置零
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // 上拉控制
    // 2. 可以在这里做一些报警记录
    // 3. 清除 TZ 中断标志
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EDIS;
    // 4. 应答 PIE 中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;     //手动清除ACK 准备下一次响应
}
