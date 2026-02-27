#ifndef MY_FOC_H_
#define MY_FOC_H_

#include "DSP2833x_Device.h"
#include <math.h>
#define SQRT3_INV   0.577350269f

extern Uint16 run_flag;      //
extern Uint16 mode;          // 0: 较大的iq_ref  1: 较小的
extern int test_sum;         // 3个CMPA求和 判断
extern int fault_flag;       // IR2136发生错误后 TZ中断将此变量置1 便于在expression观察
extern float freq_target_hz;  // 目标电角度频率
extern float angle_step;      // 角度步进值
extern float TPWM;            // 一个开关周期的时长 1 / 10k

// --- 这里就是 PI_CONTROLLER 的定义 ---
typedef struct {
    float  Ref;      // 给定值
    float  Fdb;      // 反馈值
    float  Err;      // 误差
    float  Kp;       // 比例系数
    float  Ki;       // 积分系数
    float  Ui;       // 积分累加值
    float  Out;      // 输出值
    float  OutMax;   // 输出上限
    float  OutMin;   // 输出下限
} PI_CONTROLLER;

// --- FOC 核心变量结构
typedef struct {
    // 原始数据
    float  Ia;       // A相电流
    float  Ib;       // B相电流
    float  Ic;       // C相电流

    // 角度与三角函数
    float  Theta;
    float  SinVal;
    float  CosVal;

    // Clarke 变换结果 (电流)
    float  Ialpha;
    float  Ibeta;

    // Park 变换结果 (电流)
    float  Id;
    float  Iq;

    // PI 输出 (电压)
    float  Vd;
    float  Vq;

    // 反 Park 结果 (电压)
    float  Valpha;
    float  Vbeta;

    // SVPWM 占空比
    float  Ta;
    float  Tb;
    float  Tc;
} FOC_Handle;
extern FOC_Handle foc;
extern PI_CONTROLLER pi_id;
extern PI_CONTROLLER pi_iq;
// --- 函数声明 ---
void Run_Clarke(FOC_Handle *foc);
void Run_Park(FOC_Handle *foc);
void Run_InvPark(FOC_Handle *foc);
void Run_PI(PI_CONTROLLER *pi);
void Run_SVPWM_Simple(FOC_Handle *foc, float DC_Bus_Voltage);
void Init_Control_Vars(void); // 初始化foc参数
#endif /* MY_FOC_H_ */
