/*
 * my_spi.h
 *
 *  Created on: 2026年2月18日
 *      Author: ShiGuang
 */

#ifndef MY_SPI_H_
#define MY_SPI_H_

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

extern Uint16 raw_data ;
extern float mech_angle;
extern float elec_angle;
extern Uint16 response ;
extern Uint16 command; // 读 指令

// 1. 在头文件中定义结构体类型
typedef union {
    Uint16 all;
    struct {
        Uint16 agc:8;
        Uint16 lf:1;
        Uint16 cof:1;
        Uint16 magh:1;
        Uint16 magl:1;
        Uint16 res:2;
        Uint16 ef:1;
        Uint16 parity:1;
    } bit;
} AS5047_FRAME_t;

// 2. 告诉所有包含这个头文件的文件：有一个变量叫 debug_frame，它在别处定义
extern AS5047_FRAME_t debug_frame;

void Init_Spia_AS5047P(void);
int Check_Parity(Uint16 data);
Uint16 Read_AS5047P_Raw(void);
float Get_Electrical_Angle(void);
void Clear_AS5047P_Error(void);
float Get_Electrical_Angle(void);
#endif /* MY_SPI_H_ */
