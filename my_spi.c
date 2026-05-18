#include<my_spi.h>
#include<my_foc.h>
// 定义 CS 信号控制宏 (针对 GPIO 57)
// GPIO 57 属于 Group B，位偏移为 57 - 32 = 25
#define AS5047P_CS_LOW   GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1
#define AS5047P_CS_HIGH  GpioDataRegs.GPBSET.bit.GPIO57 = 1

/*  SPICCR (配置控制寄存器)
 *  15:8    Reserved
 *  7       SPISWRESET      SPI 软件复位    0: 初始化状态，此时无法发送/接收； 1: 正常运行状态。每次改配置必须先写 0 再写 1。
 *  6       CLKPOLARITY     时钟极性        0 空闲时低电平；1 空闲时高电平
 *  5       Reserved        保留
 *  4       SPILBK          内部自环回测试使能 (Loopback) 1:回环 0：关闭
 *  3:0     SPICHAR 字符长度  填入数值 N，实际发送 (N+1) 个 bit。填 1111 (0xF) 就是 16 bit
 */
/*  SPICTL (运行控制寄存器)
 *  15:5    Reserved        保留
 *  4       OVERRUNINTENA   溢出中断使能。   如果接收缓冲器满了，你还没来得及读，新的数据又“硬塞”进来了，就会发生 Overrun（溢出/覆写）。
 *                                        设为 1 可以触发溢出报错中断。用 FIFO 时一般设为 0
 *  3       CLK_PHASE       时钟相位        决定读写数据的时钟边沿，决定是在时钟的“上升沿”还是“下降沿”去锁存（读取）数据
 *  2       MASTER_SLAVE    主从模式        1为主机
 *  1       TALK            发送使能        1允许MOSI引脚出数据，0：只能接收
 *  0       SPIINTENA       SPI基础中断使能  通常在使用 FIFO 的情况下，这里设为 0（靠 FIFO 中断），如果不使用 FIFO 则可以开启
 */
/*  SPIBRR (SPI 波特率寄存器)
/*  15:7    Reserved        保留
 *  6:0     SPI_BIT_RATE    波特率分频值 (0 ~ 127)
 *  特例情况： 往 SPIBRR 写入 0、1 或 2 时，硬件强制按最高速度运行
 *          SPI_Baud = LSPCLK / 4  = 37.5MHz / 4 = 9.375 MHz
 *          写入 3 到 127 之间的值时，公式为:
 *          SPI_Baud = LSPCLK / （ SPIBRR + 1 )
 */
/*  SPITXBUF (发送缓冲寄存器)
 *  15:0    TXDATA  要发送的数据
 *  F28335 的发送数据必须“左对齐 (Left-Justified)”
 *
 *  SPIRXBUF (接收缓冲寄存器) 自动右对齐
 */
/*  SPISTS (状态寄存器)
 *  15:8    Reserved        保留
 *  7       OVERRUN_FLAG    接收溢出标志     变成 1 说明你读晚了，旧数据被新数据覆盖了
 *  6       INT_FLAG        中断标志        变成 1 说明收发完成了一个字。轮询模式下，经常写 while(SpiaRegs.SPISTS.bit.INT_FLAG != 1){} 来死等数据传完
 *  5       BUFFULL_FLAG    发送缓冲器满标志  如果你往 SPITXBUF 写得太快，上一帧还没发出去你就写新的，这里会变 1
 *  4:0     Reserved
 */
/*  SPIFFTX (FIFO 发送控制)
 *  15      SPIRST          SPI 通道复位    注意，它和 SPISWRESET 协同工作。设为 0 可以瞬间重置整个 SPI 收发状态机，设为 1 恢复
 *  14      SPIFFENA        FIFO 使能      设为 1 打开高级队列
 *  13      TXFIFO_RESET    清空发送队列     写 0 瞬间把排队要发的数据全倒掉
 *  12:8    TXFFST          当前排队数量     这是一个只读的计步器！它的值在 0~16 之间跳动。它能告诉你：“当前 TX FIFO 里还有几个数据没发出去”
 *  7:0     (中断设置)        包含清中断标志(TXFFINTCLR)、设中断阈值(TXFFIL)等
 */
/*  SPIFFRX (FIFO 接收控制)
 *  15:14   Reserved        保留
 *  13      RXFIFORESET     清空接收队列
 *  12:8    RXFFST          当前收件箱数量 (Status) 极度常用！ 它告诉你当前 FIFO 里收到了几个数据。
 *                          读 AS5047P 时，最优雅的写法是死等这个计步器：while(SpiaRegs.SPIFFRX.bit.RXFFST < 1) {} （只要收件箱里的数据不到1个，我就死等）
 *  7:0     (中断设置)        接收中断设置
 */
/*  SPIPRI (优先级控制寄存器) —— “仿真器行为管家”
 *  FREE (Bit 4) 和 SOFT (Bit 3) 组合
 *  00：遇到断点，立刻死停（不管数据传没传完，时钟直接卡死）。这可能导致 AS5047P 时序错乱报错
 *  01：遇到断点，把当前正在传的这 16 个 bit 传完，然后再停。（推荐调试时配置）
 *  1x：无视断点，SPI 硬件模块在后台继续跑
 */
void Init_Spia_AS5047P(void) {
    EALLOW;

    /* --- 1. GPIO 引脚复用配置 (Group B) --- */
    // GPBMUX2 寄存器负责 GPIO 48-63
    // GPIO 54 -> SPISOMIA (复用模式 1)
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1;
    // GPIO 55 -> SPISIMOA (复用模式 1)
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1;
    // GPIO 56 -> SPICLKA (复用模式 1)
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1;
    // GPIO 57 -> 设定为普通 GPIO (复用模式 0)
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;  // 设置为输出
    AS5047P_CS_HIGH;                     // 默认拉高，不选中传感器

    /* --- 2. SPI 模块内部配置 --- */
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;   // 软件复位开始配置
    // 16位字符长度，对应 AS5047P 的 16位数据帧
    SpiaRegs.SPICCR.bit.SPICHAR = 0xF;
    // AS5047P 要求的 Mode 1: CPOL=0, CPHA=1
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;

    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1; // DSP 作为主机
    SpiaRegs.SPICTL.bit.TALK = 1;         // 使能发送功能

    // 波特率配置 (LSPCLK 默认为 37.5MHz)
    // 37.5 / (7 + 1) = 4.68 MHz，完全满足 AS5047P 的 10MHz 限制
    SpiaRegs.SPIBRR = 15;

    SpiaRegs.SPICCR.bit.SPISWRESET = 1;   // 释放复位，SPI 就绪
    SpiaRegs.SPIPRI.bit.FREE = 1;         // 断点调试时不中断通讯

    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;    // 使能 FIFO 模式
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;      // 释放发送 FIFO
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1; // 释放接收 FIFO

    EDIS;
}

// 检查 16 位数据的偶校验是否正确
int Check_Parity(Uint16 data) {
    Uint16 count = 0;
    Uint16 temp = data;
    while(temp) {
        temp &= (temp - 1); // 快速计算 1 的个数
        count++;
    }
    return (count % 2 == 0); // 返回 1 表示校验通过
}

Uint16 response ;
Uint16 command; // 读 指令
Uint16 Read_AS5047P_Raw(void)
{
    // 【必须是 0xFFFF】: 读标志(1) + 偶校验标志(1) + 地址(0x3FFF)
    AS5047P_CS_LOW;
    DELAY_US(1);
    SpiaRegs.SPITXBUF = 0xFFFF;
    while(SpiaRegs.SPIFFRX.bit.RXFFST < 1);
    response = SpiaRegs.SPIRXBUF;
    DELAY_US(1);
    AS5047P_CS_HIGH;

    // 如果校验通过且无错误标志
    if (Check_Parity(response) && !(response & 0x4000)) {
        return (response & 0x3FFF);
    } else {
        return 0xFFFF; // 报错特权值
    }
}
extern Uint32 spi_err_cnt;
Uint16 raw_data ;
float mech_angle;
float elec_angle;
float w_elec;
float Get_Electrical_Angle(void) {
    // 使用静态变量，防止报错时污染数据
    static float last_elec_rad = 0.0f;

    raw_data = Read_AS5047P_Raw();
    if(raw_data > 16383) {
        spi_err_cnt++;
        return last_elec_rad; // 安全返回历史值

    }

    // 计算纯净弧度 (0 ~ 2*PI)
    float mech_rad = (float)raw_data * 6.2831853f / 16384.0f;
    float elec_rad = mech_rad * 7.0f;

    while(elec_rad >= 6.2831853f) elec_rad -= 6.2831853f;

    last_elec_rad = elec_rad;
    return elec_rad;
}
// 专门用于清除 AS5047P 历史错误标志的函数
void Clear_AS5047P_Error(void)
{
    // 读取 ERRFL 寄存器 (地址 0x0001)
    // 读标志位=1，地址=0x0001 -> 0x4001。
    // 0x4001 中 1 的个数为 2 (偶数)，所以第 15 位校验位=0。
    // 最终发送指令为：0x4001

    AS5047P_CS_LOW;
    DELAY_US(1);

    SpiaRegs.SPITXBUF = 0x4001;              // 发送清错指令
    while(SpiaRegs.SPIFFRX.bit.RXFFST < 1);  // 等待接收
    Uint16 dump = SpiaRegs.SPIRXBUF;         // 读出来丢弃即可，读这个动作本身就会清空 EF

    DELAY_US(1);
    AS5047P_CS_HIGH;

    DELAY_US(10); // 让芯片缓一下
}
