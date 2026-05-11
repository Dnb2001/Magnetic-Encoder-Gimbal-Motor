#include<my_spi.h>
#include<my_foc.h>
// 定义 CS 信号控制宏 (针对 GPIO 57)
// GPIO 57 属于 Group B，位偏移为 57 - 32 = 25
#define AS5047P_CS_LOW   GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1
#define AS5047P_CS_HIGH  GpioDataRegs.GPBSET.bit.GPIO57 = 1

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
