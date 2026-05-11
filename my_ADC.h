#ifndef MY_ADC_H_
#define MY_ADC_H_

#include "DSP2833x_Device.h"

// --- 参数宏定义 (根据你的硬件修改) ---
// 假设 ADC 是 12位 (0-4095)，对应 0-3.0V
// 假设 0A 对应电压是 1.5V (也就是计数值 2048)
// 假设 电流传感器灵敏度是 X V/A
//#define ADC_OFFSET_A    2286.0f   // A相偏置值 (零点)
//#define ADC_OFFSET_B    2286.0f   // B相偏置值
//#define ADC_OFFSET_C    2286.0f   // B相偏置值
#define ADC_SCALE       0.0111f     // 比例系数 (Amps per Count) = 3.0V / 4096 / 运放放大倍数 / 采样电阻

// --- 定义电流数据结构体 ---
typedef struct {
    float Ia;   // A相真实电流 (Amps)
    float Ib;   // B相真实电流 (Amps)
    float Ic;   // C相真实电流
} MOTOR_CURRENT;

extern MOTOR_CURRENT m_current;
extern volatile float zero_offset_rad;
extern Uint16 can_tx_ready_flag;
void InitAdc_User(void);       // 初始化 ADC 模块
void Read_Phase_Currents(void); // 读取并转换电流
void Read_Phase_Current_Zero(void);
__interrupt void adc_isr(void);

#endif /* MY_ADC_H_ */
