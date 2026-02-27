#include "my_foc.h"

FOC_Handle foc = {0};
PI_CONTROLLER pi_id = {0};
PI_CONTROLLER pi_iq = {0};

// 1. Clark 变换
// 参数指针改成 'foc'，含义更明确
void Run_Clarke(FOC_Handle *foc)
{
    // 公式：Ialpha = Ia
    foc->Ialpha = foc->Ia;

    // 公式：Ibeta = (Ia + 2*Ib) / sqrt(3)
    foc->Ibeta  = (foc->Ia + 2.0f * foc->Ib) * SQRT3_INV;
}

// 2. Park 变换
void Run_Park(FOC_Handle *foc)
{
    // 使用 Ialpha 和 Ibeta 计算 Id, Iq
    foc->Id =  foc->Ialpha * foc->CosVal + foc->Ibeta * foc->SinVal;
    foc->Iq = -foc->Ialpha * foc->SinVal + foc->Ibeta * foc->CosVal;
}

// 3. 反 Park 变换
void Run_InvPark(FOC_Handle *foc)
{
    // 输入是 Vd, Vq，输出是 Valpha, Vbeta
    // 这里的 V 代表 Voltage，非常清晰
    foc->Valpha = foc->Vd * foc->CosVal - foc->Vq * foc->SinVal;
    foc->Vbeta  = foc->Vd * foc->SinVal + foc->Vq * foc->CosVal;
}
// 4. PI 控制器实现
void Run_PI(PI_CONTROLLER *pi)
{
    // 计算误差
    pi->Err = pi->Ref - pi->Fdb;

    // 积分项累加
    pi->Ui += pi->Ki * pi->Err;

    // 积分限幅 (抗饱和)
    if (pi->Ui > pi->OutMax) pi->Ui = pi->OutMax;
    if (pi->Ui < pi->OutMin) pi->Ui = pi->OutMin;

    // 计算总输出 (位置式 PI)
    pi->Out = (pi->Kp * pi->Err) + pi->Ui;

    // 输出限幅
    if (pi->Out > pi->OutMax) pi->Out = pi->OutMax;
    if (pi->Out < pi->OutMin) pi->Out = pi->OutMin;
}
#warning "这里为什么一开始是SPWM"
// 5. 简易 SVPWM 生成
void Run_SVPWM_Simple(FOC_Handle *foc, float DC_Bus_Voltage)
{
    // 输入是 Valpha, Vbeta (单位: 伏特)
    // 假设 DC_Bus_Voltage 是母线电压 (例如 24V)

    float Va = foc->Valpha;
    float Vb = -0.5f * foc->Valpha + 0.8660254f * foc->Vbeta; // sqrt(3)/2
    float Vc = -0.5f * foc->Valpha - 0.8660254f * foc->Vbeta;

    // 归一化并加上 0.5 偏置 (将 -12V~+12V 映射到 0.0~1.0 的占空比)
    // 前提：SVPWM 最大输出相电压幅值是 Vdc/sqrt(3) ?
    // 这里用最简单的 SPWM 逻辑: 0V -> 50% 占空比

    foc->Ta = (Va / DC_Bus_Voltage) + 0.5f;
    foc->Tb = (Vb / DC_Bus_Voltage) + 0.5f;
    foc->Tc = (Vc / DC_Bus_Voltage) + 0.5f;
}

// --- 初始化 FOC 参数 --- 在 main.c 的 while(1) 之前调用一次这个函数
void Init_Control_Vars(void)
{   run_flag = 0;
    // 1. 初始化角度步长 (2*PI * f * T)
    angle_step = 6.2831853f * freq_target_hz * TPWM;

    // 2. 配置 D轴 PI 参数 (控制磁通/无功)
    pi_id.Kp = 0.1f;        // 试探值
    pi_id.Ki = 0.01f;
    pi_id.OutMax = 1.5f;   // 输出电压限幅 (根据你的母线电压，例如 24V 母线一半是 12V)
    pi_id.OutMin = -1.5f;

    // 3. 配置 Q轴 PI 参数 (控制转矩/有功)
    pi_iq.Kp = 0.2f;
    pi_iq.Ki = 0.02f;
    pi_iq.OutMax = 2.5f;
    pi_iq.OutMin = -2.5f;

    // 4. 清零中间变量
    foc.Theta = 0;
    foc.Id = 0; foc.Iq = 0;
}
