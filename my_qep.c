#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

void InitEQep1_AS5047P(void)
{
    // =========================================================
    // 1. 使能 eQEP1 外设时钟 (极其重要，否则后续寄存器无法写入)
    // =========================================================
    EALLOW;
    SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;  // 开启 eQEP1 模块时钟
    EDIS;

    // =========================================================
    // 2. GPIO 复用与方向配置 (50,51,53 给解码器，52 给 IR2136)
    // =========================================================
    EALLOW;
    // 启用内部上拉电阻
    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;   // EQEP1A
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;   // EQEP1B
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;   // EQEP1I (Z脉冲)

    // 信号同步设置 (与系统时钟 SYSCLKOUT 同步)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 0;

    // GPIO 功能复用 (GPBMUX2 控制 GPIO48-63)
    // 1 = eQEP 功能, 0 = 通用 GPIO 功能
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 1;  // 配置为 EQEP1A 输入
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 1;  // 配置为 EQEP1B 输入
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 1;  // 配置为 EQEP1I 输入

    EDIS;

    // 初始化 IR2136 使能引脚电平
    // 注意：请根据你的驱动板逻辑修改。如果是高电平使能，上电先置低电平关闭输出以保护电路
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1; // 输出低电平
    // GpioDataRegs.GPBSET.bit.GPIO52 = 1; // 如果需要输出高电平用这句

    // =========================================================
    // 3. eQEP 核心寄存器配置 (解码与找零位)
    // =========================================================
    // 最大计数值：假设测试出来是 1000线，则 1000 * 4 - 1 = 3999
    // 如果今天测试确认是 1024线，请将其改为 4096 - 1 = 4095
    EQep1Regs.QPOSMAX = 4000 - 1;
    EQep1Regs.QPOSINIT = 0;               // 初始位置设定为 0

    // QDECCTL: 解码控制
    EQep1Regs.QDECCTL.bit.QSRC = 00;      // 正交计数模式
    EQep1Regs.QDECCTL.bit.IGATE = 0;      // 禁用 Index 门控 (AS5047P的Z信号通常较宽)
    EQep1Regs.QDECCTL.bit.QAP = 0;        // A 相极性不反转
    EQep1Regs.QDECCTL.bit.QBP = 0;        // B 相极性不反转
    EQep1Regs.QDECCTL.bit.QIP = 0;        // Index 极性不反转

    // QEPCTL: 运行控制与复位逻辑
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;   // 仿真器挂起（打断点）时，位置计数器不受影响，继续运行

    // 【寻零核心配置】
    EQep1Regs.QEPCTL.bit.PCRM = 00;       // 在检测到 Index (Z脉冲) 事件时，自动复位 QPOSCNT
    EQep1Regs.QEPCTL.bit.IEI = 10;        // 指定 Index 事件由上升沿触发

    // =========================================================
    // 4. 单位定时器配置 (Unit Timer) - 用于后续计算转速
    // =========================================================
    // 假设 F28335 运行在 150MHz 系统时钟下，我们设定 1ms 锁存一次位置
    EQep1Regs.QUPRD = 150000000 / 1000;   // 1ms 对应的时钟周期数

    EQep1Regs.QEPCTL.bit.UTE = 1;         // 使能单位定时器
    EQep1Regs.QEPCTL.bit.QCLM = 1;        // 当 1ms 时间到时，硬件自动将 QPOSCNT 的值锁存到 QPOSLAT 寄存器中

    // =========================================================
    // 5. 启动模块
    // =========================================================
    EQep1Regs.QPOSCNT = 0;                // 计数器清零
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // 全面使能 eQEP1 模块开始工作
}
