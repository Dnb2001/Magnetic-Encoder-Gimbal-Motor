#ifndef MY_FOC_H_
#define MY_FOC_H_

#include "DSP2833x_Device.h"
#include <math.h>
#define SQRT3_INV   0.577350269f
// --- 电机物理参数定义 ---
#define MOTOR_RS     0.32f       // 相电阻 (欧姆)
#define MOTOR_LS     0.00145f    // 相电感 Ld = Lq = 50uH (亨利，注意单位！)
#define MOTOR_FLUX   0.0021f     // 永磁体磁链 (韦伯)

extern Uint16 run_flag;      //
extern Uint16 mode;          // 0: 较大的iq_ref  1: 较小的
extern int test_sum;         // 3个CMPA求和 判断
extern int fault_flag;       // IR2136发生错误后 TZ中断将此变量置1 便于在expression观察
extern float freq_target_hz;  // 目标电角度频率
extern float angle_step;      // 角度步进值
extern float TPWM;            // 一个开关周期的时长 1 / 10k


// --- PI_CONTROLLER 的定义 ---
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

typedef enum {
    STATE_IDLE = 0,      // 待机：PWM关闭，等待CAN指令
    STATE_CALIB = 1,     // 标定：正在读取电流霍尔零位
    STATE_ALIGN = 2,     // 对齐：正在注入D轴电流找零位
    STATE_RUN = 3,       // 运行：双闭环闭环旋转
    STATE_FAULT = 4      // 故障：发生错误，紧急停机
} SYS_STATE_t;
extern SYS_STATE_t sys_state; // 全局状态变量

extern FOC_Handle foc;
extern PI_CONTROLLER pi_id;
extern PI_CONTROLLER pi_iq;
extern PI_CONTROLLER pi_spd;      // 速度环 PI 控制器
extern float zero_offset;         // 机械角度零位
extern int Alignment_flag;        // 对齐结束标志位
extern float speed_target;        // 目标转速 (rad/s)
extern float speed_fdb;           // 反馈转速 (经过滤波的)
extern float U_comp_A, U_comp_B, U_comp_C;
// --- 函数声明 ---
void Run_Clarke(FOC_Handle *foc);
void Run_Park(FOC_Handle *foc);
void Run_InvPark(FOC_Handle *foc);
void Run_PI(PI_CONTROLLER *pi);
void Run_SVPWM_Simple(FOC_Handle *foc, float DC_Bus_Voltage);
void Run_SVPWM_Real(FOC_Handle *foc, float V_Bus);
void Init_Control_Vars(void); // 初始化foc参数

float Calculate_Speed(float current_theta); // 速度计算函数
#endif /* MY_FOC_H_ */
