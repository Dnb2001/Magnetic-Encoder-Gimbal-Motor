#include "my_adc.h"
#include "my_foc.h"
#include "DSP2833x_Examples.h"   // DELAY_US 的宏定义在这里
// 定义全局变量实例
MOTOR_CURRENT m_current = {0.0f, 0.0f, 0.0f};

float32 ia_offset = 0.0f;       // 三相霍尔零位
float32 ib_offset = 0.0f;
float32 ic_offset = 0.0f;
Uint16  offset_calibrated = 0; // 标定完成标志

// 初始化 ADC 模块
void InitAdc_User(void)
{
    // 1. 基础复位与上电
    InitAdc(); // TI 官方库函数，负责给 ADC 上电

    EALLOW;
    // 2. 配置 ADC 时钟 (非常重要！)
    // HSPCLK = SYSCLKOUT / (2 * ADC_MODCLK)
    // 假设系统时钟 150MHz，这里设为 3，即 150 / (2*3) = 25MHz (最大值)
    SysCtrlRegs.HISPCP.all = 3;

    // 3. 配置 ADC 控制寄存器
    AdcRegs.ADCTRL1.bit.ACQ_PS = 0xF;      // 采样窗口设大一点，保证信号稳定 (16个ADCCLK)
    AdcRegs.ADCTRL1.bit.CPS = 0;           // 预分频不需再分
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;      // 级联模式 (16通道看作一个序列，方便管理)

    AdcRegs.ADCTRL3.bit.SMODE_SEL = 1;     // 【关键】开启同步采样模式 (Simultaneous)
                                           // 结果：A0->RESULT0, B0->RESULT1
    AdcRegs.ADCTRL1.bit.CONT_RUN = 0;      // 单次运行模式，等待 PWM 触发
    // --- 【修改点 2】 开启 ePWM 触发 SOC 的开关 ---
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1; // 必须加上这句，ADC 才会响应 ePWM 的 SOCA 信号

    // --- 【新增：开启 ADC 转换完成中断】 ---
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  // 使能 SEQ1 中断
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;  // 每转换完 1 次 SEQ1 就产生一次中断
    // 4. 配置采样通道
    // 0x0 代表通道 0：采样 ADCINA0 (A相) 和 ADCINB0 (B相) -> 结果放入 RESULT0, RESULT1
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;

    // 0x1 代表通道 1：采样 ADCINA1 (C相) 和 ADCINB1 (未使用) -> 结果放入 RESULT2, RESULT3
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;

    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 1;  // 转换 2 对，共 4 个结果

    EDIS;
}

// 读取并转换电流 (将在中断中调用)
float raw_Ia, raw_Ib, raw_Ic;
void Read_Phase_Currents(void)
{
    // 1. 读取原始寄存器值 (F28335 的结果是左对齐的，需要右移 4 位)
    // 只有当程序在 FLASH 运行时通常才需要检查 ADC 忙状态，但在 ISR 里通常转换已完成
    raw_Ia = ( AdcRegs.ADCRESULT0 >> 4 ) *0.2f + 0.8f * raw_Ia;
    raw_Ib = ( AdcRegs.ADCRESULT1 >> 4 ) *0.2f + 0.8f * raw_Ib;
    raw_Ic = ( AdcRegs.ADCRESULT2 >> 4 ) *0.2f + 0.8f * raw_Ic;
    // 2. 转换为真实物理量 (y = k * (x - b))
    // 注意：这里使用了 float 运算
    m_current.Ia = (-(float)raw_Ia + ia_offset) * ADC_SCALE * 0.0007326f ;
    m_current.Ib = (-(float)raw_Ib + ib_offset) * ADC_SCALE * 0.0007326f ;
    m_current.Ic = (-(float)raw_Ic + ic_offset) * ADC_SCALE * 0.0007326f ;
}


__interrupt void adc_isr(void)
{
    // 上电后读1024次电流零位

    // 1. 读取电流 (结果存入 m_current 结构体)
        Read_Phase_Currents();

        // 将采样到的物理值填入 FOC 结构体
        foc.Ia = m_current.Ia;
        foc.Ib = m_current.Ib;
        // foc.Ic = m_current.Ic; // FOC 计算只需要 Ia, Ib

        // 2. 只有在运行标志开启时才跑算法
        if (run_flag == 1)
        {
            // --- A. 生成虚拟角度 (模拟电机旋转) ---
            // 每次中断增加一点角度，产生旋转磁场
            foc.Theta += angle_step;
            // 归一化到 0 ~ 2PI
            if (foc.Theta > 6.2831853f)
                {
                foc.Theta -= 6.2831853f;
                }

            foc.SinVal = sinf(foc.Theta);
            foc.CosVal = cosf(foc.Theta);

            // --- B. 正变换 (Feedback) ---
            Run_Clarke(&foc);
            Run_Park(&foc);

            // --- C. PI 控制器 (Control) ---
            // D轴目标：通常锁定为 0
            pi_id.Ref = 0.0f;
            pi_id.Fdb = foc.Id;
            Run_PI(&pi_id);
            foc.Vd = pi_id.Out;
            //foc.Vd = 0.0;
            // Q轴目标：这是你想要打入电感的电流 (例如 2.0A)
            if(mode == 0) {
                pi_iq.Ref = 0.7f; // 自动模式：恒定 0.1A
            } else {
                pi_iq.Ref = 0.3f; // 手动模式暂时归零，或者你自己定义逻辑
            }

            pi_iq.Fdb = foc.Iq;
            Run_PI(&pi_iq);
            foc.Vq = pi_iq.Out;
            //foc.Vq = 1.0f;
            // --- D. 反变换 & 输出 (Output) ---
            Run_InvPark(&foc);

            // 假设母线电压为 24V，生成占空比
            Run_SVPWM_Simple(&foc, 24.0f);

            // --- E. 更新 PWM 寄存器 ---
            // 假设 TBPRD = 7500 (对应20kHz), 且是 UpDown 计数模式
            // 在 UpDown 模式下，全周期计数其实是 15000 个时钟，但 CMPA 还是参照 TBPRD
            // SVPWM 输出 Ta, Tb, Tc 范围是 0.0 ~ 1.0

            Uint16 period = 7500; // 你的 TBPRD

            // 简单的限幅保护
            if(foc.Ta > 0.95f) foc.Ta = 0.95f; if(foc.Ta < 0.05f) foc.Ta = 0.05f;
            if(foc.Tb > 0.95f) foc.Tb = 0.95f; if(foc.Tb < 0.05f) foc.Tb = 0.05f;
            if(foc.Tc > 0.95f) foc.Tc = 0.95f; if(foc.Tc < 0.05f) foc.Tc = 0.05f;

            EPwm1Regs.CMPA.half.CMPA = (Uint16)(foc.Ta * (float)period);
            EPwm2Regs.CMPA.half.CMPA = (Uint16)(foc.Tb * (float)period);
            EPwm3Regs.CMPA.half.CMPA = (Uint16)(foc.Tc * (float)period);
            test_sum = EPwm1Regs.CMPA.half.CMPA + EPwm2Regs.CMPA.half.CMPA + EPwm3Regs.CMPA.half.CMPA;
        }
        else // 运行标志run_flag关闭时 不跑
        {
            // 停机状态 对于半桥驱动，通常给 0 占空比意味着下管常通，上管常关（刹车）
            EPwm1Regs.CMPA.half.CMPA = 0;
            EPwm2Regs.CMPA.half.CMPA = 0;
            EPwm3Regs.CMPA.half.CMPA = 0;
            // 重置 PI 积分项，防止再次启动时“暴冲”
            pi_id.Ui = 0; pi_id.Out = 0;
            pi_iq.Ui = 0; pi_iq.Out = 0;
        }


    // 1. 复位 ADC 序列，为下一次触发做准备 ---
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    // 2. 清除 ADC 中断标志位
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    // 3. 应答 PIE Group 1 (ADCINT 所在的组)
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

      // EPwm1Regs.ETCLR.bit.INT = 1;            // 这行不再需要了，因为我们没用 ePWM 中断
      // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // 这行不再需要了，因为我们没用 ePWM 中断
}

void Read_Phase_Current_Zero(void)
{
    int i=0;
    float32 sum_ia = 0;
    float32 sum_ib = 0;
    float32 sum_ic = 0;
    for (i = 0; i < 1024; i++)
    {
        // 1. 软件强制触发一次 ADC 采样 (发令枪)
        AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;
        // 2. 等待这 1 次采样彻底完成
        while (AdcRegs.ADCST.bit.INT_SEQ1 == 0);
        // 3. 清除标志位，为下一次做准备
        AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
        // 4. 现在读到的才是最新鲜的数据
        sum_ia += (float32)(AdcRegs.ADCRESULT0>>4);
        sum_ib += (float32)(AdcRegs.ADCRESULT1>>4);
        sum_ic += (float32)(AdcRegs.ADCRESULT2>>4);
        DELAY_US(100);
     }
        ia_offset = sum_ia / 1024.0f;
        ib_offset = sum_ib / 1024.0f;
        ic_offset = sum_ic / 1024.0f;
}

