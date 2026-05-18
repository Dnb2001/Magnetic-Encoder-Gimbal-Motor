#include "my_foc.h"
#include "my_spi.h"
#include "DSP2833x_Examples.h"   // DELAY_US 的宏定义在这里
FOC_Handle foc = {0};
PI_CONTROLLER pi_id = {0};
PI_CONTROLLER pi_iq = {0};

PI_CONTROLLER pi_spd = {0};
float speed_target = 3.0f;   // 给一个初始测试目标转速 (电角速度 rad/s)
float speed_fdb = 0.0f;

// 定义全局的补偿电压变量
float U_comp_A = 0.0f;
float U_comp_B = 0.0f;
float U_comp_C = 0.0f;

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
/*void Run_SVPWM_Simple(FOC_Handle *foc, float DC_Bus_Voltage)
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
}*/
#warning "为什么说马鞍波是注入了三次谐波的正弦波 只是从形状上看吗 这里零序分量是怎么来的"
// 修改后的真·SVPWM 函数 (注入零序分量法)
float Va,Vb,Vc;
void Run_SVPWM_Real(FOC_Handle *foc, float V_Bus)
{
    // 1. 计算三相原始电压 (反 Clarke)
     Va = foc->Valpha;
     Vb = -0.5f * foc->Valpha + 0.8660254f * foc->Vbeta;
     Vc = -0.5f * foc->Valpha - 0.8660254f * foc->Vbeta;

     // 1.5 【核心注入点】：直接在物理相电压上叠加死区补偿
         Va = Va + U_comp_A;
         Vb = Vb + U_comp_B;
         Vc = Vc + U_comp_C;

    // 2. 找到三相中的最大值和最小值 (Min-Max 法)
    float Vmax = Va; if(Vb > Vmax) Vmax = Vb; if(Vc > Vmax) Vmax = Vc;
    float Vmin = Va; if(Vb < Vmin) Vmin = Vb; if(Vc < Vmin) Vmin = Vc;

    // 3. 计算零序电压 (这就是马鞍波的灵魂)
    float Vzero = 0.5f * (Vmax + Vmin);

    // 4. 减去零序分量并归一化到 0.0 ~ 1.0 的占空比
    // 这里的 V_Bus 就是你刚才担心的那个参数
    foc->Ta = (Va - Vzero) / V_Bus + 0.5f;
    foc->Tb = (Vb - Vzero) / V_Bus + 0.5f;
    foc->Tc = (Vc - Vzero) / V_Bus + 0.5f;
}

// --- 初始化 FOC 参数 --- 在 main.c 的 while(1) 之前调用一次这个函数
void Init_Control_Vars(void)
{   run_flag = 0;
    // 1. 初始化角度步长 (2*PI * f * T)
    angle_step = 6.2831853f * freq_target_hz * TPWM;

    // 2. 配置 D轴 PI 参数 (控制磁通/无功)
    pi_id.Kp = 1.5f;        // 试探值
    pi_id.Ki = 0.05f;
    pi_id.OutMax = 2.0f;   // 输出电压限幅 (根据你的母线电压，例如 24V 母线一半是 12V)
    pi_id.OutMin = -2.0f;

    // 3. 配置 Q轴 PI 参数 (控制转矩/有功)
    pi_iq.Kp = 1.5f;
    pi_iq.Ki = 0.05f;
    pi_iq.OutMax = 2.0f;
    pi_iq.OutMin = -2.0f;

    // 4. 清零中间变量
    foc.Theta = 0;
    foc.Id = 0; foc.Iq = 0;

    // 速度环通常 Kp 大一些，Ki 小一些
        pi_spd.Kp = 0.02f;      // 试探值，后续可能需要根据负载转动惯量调整
        pi_spd.Ki = 0.001f;
        pi_spd.OutMax = 0.8f;   // ！！速度环的输出限幅，就是允许电机输出的最大 Iq 电流 (安培) ！！
        pi_spd.OutMin = -0.8f;  // 允许最大 1.5A 的刹车/反转电流
}

// 对齐
float zero_offset = 0.0f;
int Alignment_flag = 0;

// 计算转速 (自带抗突变和低通滤波)
float Calculate_Speed(float current_theta)
{
    static float last_theta = 0.0f;
    float delta_theta;
    float raw_speed;
    static float filtered_speed = 0.0f;

    // 1. 计算角度增量
    delta_theta = current_theta - last_theta;

    // 2. 处理跨越 0 和 2PI 边界的突变 (绝对不能漏掉！)
    if (delta_theta < -3.1415926f) delta_theta += 6.2831853f;
    if (delta_theta >  3.1415926f) delta_theta -= 6.2831853f;

    last_theta = current_theta;

    // 3. 计算原始角速度
    // 假设速度环每 10 个 PWM 周期执行一次 (PWM 是 10kHz，速度环就是 1kHz)
    // 周期 T = 0.001s, 速度 = delta / T = delta * 1000.0f
    raw_speed = delta_theta * 500.0f;

    // 4. 一阶低通滤波 (滤除微分带来的高频噪声)
    // 0.95 和 0.05 是权重，0.05 越小，滤波越强，但延迟越大
    filtered_speed = filtered_speed * 0.8f + raw_speed * 0.2f;

    return filtered_speed;
}



// --- 策略一：基于指令电流角度前馈的死区补偿 ---
void Calc_DeadTime_Comp(FOC_Handle *foc, float u_comp_max, float i_thresh)
{
    // 1. 获取平滑的指令电流 (来自速度环的输出和D轴设定值)
    float id_ref = pi_id.Ref; // D轴指令，通常为0
    float iq_ref = pi_iq.Ref; // Q轴指令，速度环输出 (带符号，反映正转或刹车)

    // 2. 利用当前电角度，反算理想的三相电流 (相当于做一次纯净的反Park + 反Clarke)
    // 这一步得到的是不包含任何 PWM 噪声的“完美正弦波”
    float ialpha_ref = id_ref * foc->CosVal - iq_ref * foc->SinVal;
    float ibeta_ref  = id_ref * foc->SinVal + iq_ref * foc->CosVal;

    // 纯净的三相指令电流
    float ia_ref = ialpha_ref;
    float ib_ref = -0.5f * ialpha_ref + 0.8660254f * ibeta_ref;
    float ic_ref = -0.5f * ialpha_ref - 0.8660254f * ibeta_ref;

    // 3. 带滞环的极性判断与补偿量计算
    // 因为 ia_ref 是完美的正弦波，这里的滞环主要是为了防止 iq_ref 在 0 附近极微小震荡时发生误判

    // A相
    if (ia_ref > i_thresh)       U_comp_A = u_comp_max;
    else if (ia_ref < -i_thresh) U_comp_A = -u_comp_max;
    else                         U_comp_A = u_comp_max * (ia_ref / i_thresh); // 线性过渡

    // B相
    if (ib_ref > i_thresh)       U_comp_B = u_comp_max;
    else if (ib_ref < -i_thresh) U_comp_B = -u_comp_max;
    else                         U_comp_B = u_comp_max * (ib_ref / i_thresh);

    // C相
    if (ic_ref > i_thresh)       U_comp_C = u_comp_max;
    else if (ic_ref < -i_thresh) U_comp_C = -u_comp_max;
    else                         U_comp_C = u_comp_max * (ic_ref / i_thresh);
}
/*  计算rpm转速
 *  电角速度 = 极对数Pn * 机械角速度
 *  机械速度 = 60 * 机械角速度 / 2pi
 *         = 60 * 电角速度  / ( 2pi * 极对数Pn )
 *         = 电角速度 *  60 / ( 2pi * 7 )
 *         = 电角速度 *  60 / ( 2pi * 极对数Pn )
 *         = 1.3641852 * 电角速度
 */
