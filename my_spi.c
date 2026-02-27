#include<my_spi.h>

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
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;

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
    Uint16 command = 0xFFFF; // 读角度指令


    // 1. 发送读取请求
    AS5047P_CS_LOW;
    DELAY_US(2);
    //SpiaRegs.SPITXBUF = command | 0x4000; // 加上读位标志
    SpiaRegs.SPITXBUF = command; //a
    while(SpiaRegs.SPIFFRX.bit.RXFFST < 1); // 等待接收完成 (或查询标志位)
    response = SpiaRegs.SPIRXBUF;
    // 在你读取 SPI 的地方
    debug_frame.all = SpiaRegs.SPIRXBUF;
    AS5047P_CS_HIGH;

    // 2. 校验与解析
    // 注意：AS5047P 的第 15 位是校验位，第 14 位是错误标志位
    if (Check_Parity(response) && !(response & 0x4000)) {
        return (response & 0x3FFF); // 返回 14 位原始数据 (0-16383)
    } else {
        return 0xFFFF; // 报错返回特权值
    }
}

Uint16 raw_data ;
float mech_angle;
float elec_angle;
float Get_Electrical_Angle(void) {
    raw_data = Read_AS5047P_Raw();
    if(raw_data > 16383) return -1.0f; // 错误处理

    // 1. 计算机械角度 (0 ~ 360°)
    mech_angle = (float)raw_data * 360.0f / 16384.0f;

    // 2. 转换为电角度 (乘以极对数 7)
    elec_angle = mech_angle * 7.0f;

    // 3. 归一化到 0 ~ 360° 范围内
    while(elec_angle >= 360.0f) elec_angle -= 360.0f;

    return elec_angle;
}
