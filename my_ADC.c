#include "my_adc.h"
#include "my_foc.h"
#include "my_spi.h"
#include "my_can.h"
#include <stdint.h>
#include "DSP2833x_Examples.h"   // DELAY_US 的宏定义在这里
SYS_STATE_t sys_state = STATE_IDLE;
// 定义全局变量实例
MOTOR_CURRENT m_current = {0.0f, 0.0f, 0.0f};

float32 ia_offset = 0.0f;       // 三相霍尔零位
float32 ib_offset = 0.0f;
float32 ic_offset = 0.0f;
Uint16  offset_calibrated = 0;  // 标定完成标志 1:完成 0：对齐中
// ！！！必须放在所有函数外面，变成全局变量 ！！！
volatile Uint32 align_cnt = 0;  // 对齐过程计数
volatile float zero_offset_rad = 0.0f;      // 机械角度零位
volatile Uint16 debug_raw_data = 0; // 用来观察原始编码器值
Uint16 can_tx_ready_flag = 0;   // 打包can数据给main发送
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
float raw_Ia, raw_Ib, raw_Ic;       // ADC读取结果寄存器右移4位的结果  右移：12bit的值在16bit寄存器的高12位，所以需要右移
void Read_Phase_Currents(void)
{
    // 1. 读取原始寄存器值 (F28335 的结果是左对齐的，需要右移 4 位)
    // 只有当程序在 FLASH 运行时通常才需要检查 ADC 忙状态，但在 ISR 里通常转换已完成
    raw_Ia =  AdcRegs.ADCRESULT0 >> 4  ;
    raw_Ib =  AdcRegs.ADCRESULT1 >> 4 ;
    raw_Ic =  AdcRegs.ADCRESULT2 >> 4 ;
#warning "这里为什么去掉滤波"
    //raw_Ia = ( AdcRegs.ADCRESULT0 >> 4 ) *0.2f + 0.8f * raw_Ia;
    //raw_Ib = ( AdcRegs.ADCRESULT1 >> 4 ) *0.2f + 0.8f * raw_Ib;
    //raw_Ic = ( AdcRegs.ADCRESULT2 >> 4 ) *0.2f + 0.8f * raw_Ic;
    // 2. 转换为真实物理量 (y = k * (x - b))
    // 注意：这里使用了 float 运算
    m_current.Ia = (ia_offset - (float)raw_Ia) * ADC_SCALE  ;
    m_current.Ib = (ib_offset - (float)raw_Ib) * ADC_SCALE  ;
    m_current.Ic = (ic_offset - (float)raw_Ic) * ADC_SCALE  ;
}

#warning "这里为什么用static"
    static Uint16 speed_loop_cnt = 0; // 新增：速度环计数器
    static Uint16 can_pack_cnt = 0; // 新增：CAN数据打包计数器
__interrupt void adc_isr(void)
{
    // 1. 基础数据采集 读取电流 (结果存入 m_current 结构体)
        Read_Phase_Currents();
        // 将采样到的物理值填入 FOC 结构体
        foc.Ia = m_current.Ia;
        foc.Ib = m_current.Ib;
        // foc.Ic = m_current.Ic; // FOC 计算只需要 Ia, Ib
#warning "为什么只要Ia和Ib"

  switch(sys_state)
  {
      // 待机
      case STATE_IDLE:
            // 下管常通，上管常关（刹车）
            EPwm1Regs.CMPA.half.CMPA = 0;
            EPwm2Regs.CMPA.half.CMPA = 0;
            EPwm3Regs.CMPA.half.CMPA = 0;
            // 重置 PI 积分项，防止再次启动时“暴冲”
            pi_id.Ui = 0; pi_id.Out = 0;
            pi_iq.Ui = 0; pi_iq.Out = 0;
            align_cnt = 0;      // 每次停机清零，保证下次启动重新对齐
            debug_raw_data = raw_data; // 停机时，把传感器原始值拿出来观察！
            pi_iq.Ref = 0;
            pi_spd.Ref = 0;
            pi_spd.Ui = 0; pi_spd.Out = 0; // <--- 必须加上！彻底清除速度环历史记忆
      break;

      // 读电流零位
      case STATE_CALIB:
          GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
          Read_Phase_Current_Zero();     // 读取霍尔零位
          sys_state = STATE_ALIGN;       // 自动跳到对齐
          GpioDataRegs.GPBSET.bit.GPIO52 = 1;
          DELAY_US(5);
      break;

      // 对齐
      case STATE_ALIGN:
            #warning "怎么算出来40000是2s"
          if ( align_cnt < 40000 )
           {   // main里正在运行对齐
              foc.Vd = 0.6f;
              foc.Vq = 0.0f;
              foc.Theta = 0.0f; // 强行设电角度为 0
              foc.SinVal = 0.0f; // sin(0)
              foc.CosVal = 1.0f; // cos(0)
              align_cnt++;
           }
          else {
              zero_offset_rad = Get_Electrical_Angle();  // 电角度零位
              sys_state = STATE_RUN;
              align_cnt = 0; // 对齐结束 状态机改为运行双闭环
          }
      break;

      // 双闭环
      case STATE_RUN:
      {
              float raw_elec = Get_Electrical_Angle();
              foc.Theta = raw_elec - zero_offset_rad;    // 读取真实角度
              // 扣除后可能出现负数，需要重新归一化到 0 ~ 2*PI
              while(foc.Theta < 0.0f) foc.Theta += 6.2831853f;
              while(foc.Theta >= 6.2831853f) foc.Theta -= 6.2831853f;
              foc.SinVal = sinf(foc.Theta);
              foc.CosVal = cosf(foc.Theta);
              // --- 新增：500Hz 速度外环 ---
                              speed_loop_cnt++;
                              if (speed_loop_cnt >= 40) // 40分频执行
                              {
                                  speed_loop_cnt = 0;
                                  // 1. 计算实际速度
                                  speed_fdb = Calculate_Speed(foc.Theta);
                                  // 2. 运行速度 PI
                                  pi_spd.Ref = speed_target;
                                  pi_spd.Fdb = speed_fdb;
                                  Run_PI(&pi_spd);
                                  // 3. 速度环的输出，直接作为电流环的目标值！
                                  pi_iq.Ref = pi_spd.Out;
                              }

            // --- B. 正变换 (Feedback) ---
            Run_Clarke(&foc);
            Run_Park(&foc);

            // --- C. PI 控制器 (Control) ---
            // D轴目标：通常锁定为 0
            pi_id.Ref = 0.0f;
            pi_id.Fdb = foc.Id;
            Run_PI(&pi_id);

            // Q轴
            pi_iq.Fdb = foc.Iq;
            Run_PI(&pi_iq);


            if (RxMsg_0x201_MDL.bit.Feedforward == 1)       // 根据CAN指令开关前馈、便于对比
             {
                float omega_e = speed_fdb;
                float Vd_ff = -omega_e * MOTOR_LS * foc.Iq;
                float Vq_ff =  omega_e * MOTOR_LS * foc.Id + omega_e * MOTOR_FLUX;
                foc.Vd = pi_id.Out + Vd_ff;
                foc.Vq = pi_iq.Out + Vq_ff;
             }
            else
            {
                foc.Vd = pi_id.Out;
                foc.Vq = pi_iq.Out;
            }


            break;
            }

      case STATE_FAULT:
            // 故障状态：瞬间切断所有输出
            EPwm1Regs.CMPA.half.CMPA = 0; EPwm2Regs.CMPA.half.CMPA = 0; EPwm3Regs.CMPA.half.CMPA = 0;
            GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;
      break;
    }   // switch结束

      // 只有在对齐和运行状态，才需要把 Vd/Vq 变成 PWM
      if (sys_state == STATE_ALIGN || sys_state == STATE_RUN)
        {
        // --- D. 反变换 & 输出 (Output) ---
        Run_InvPark(&foc);


        // 利用 CAN 报文的 DeadZone 位来动态开关补偿 (1:开, 0:关)
        if (RxMsg_0x201_MDL.bit.DeadZone == 1 && sys_state == STATE_RUN)
        {
         // 参数1：foc 句柄
         // 参数2：补偿电压 (u_comp_max)。算出来是 0.48V，但我们先打折，从 0.1V 开始试！
         // 参数3：指令电流的滞环阈值。设为 0.05A 即可过滤掉速度环的极小微扰。
         Calc_DeadTime_Comp(&foc, 0.45f, 0.05f);
        }
        else
        {
         // 关掉补偿或者处于对齐状态时，补偿量清零
         U_comp_A = 0.0f; U_comp_B = 0.0f; U_comp_C = 0.0f;
        }
        // 假设母线电压为 24V，生成占空比
        Run_SVPWM_Real(&foc, 24.0f);
        // --- E. 更新 PWM 寄存器 ---
        // 假设 TBPRD = 7500 (对应20kHz), 且是 UpDown 计数模式
        // 在 UpDown 模式下，全周期计数其实是 15000 个时钟，但 CMPA 还是参照 TBPRD
        // SVPWM 输出 Ta, Tb, Tc 范围是 0.0 ~ 1.0
        Uint16 period = 3750; // 你的 TBPRD

        // 简单的限幅保护
        if(foc.Ta > 0.95f) foc.Ta = 0.95f; if(foc.Ta < 0.05f) foc.Ta = 0.05f;
        if(foc.Tb > 0.95f) foc.Tb = 0.95f; if(foc.Tb < 0.05f) foc.Tb = 0.05f;
        if(foc.Tc > 0.95f) foc.Tc = 0.95f; if(foc.Tc < 0.05f) foc.Tc = 0.05f;

        EPwm1Regs.CMPA.half.CMPA = (Uint16)(foc.Ta * (float)period);
        EPwm2Regs.CMPA.half.CMPA = (Uint16)(foc.Tb * (float)period);
        EPwm3Regs.CMPA.half.CMPA = (Uint16)(foc.Tc * (float)period);
        //test_sum = EPwm1Regs.CMPA.half.CMPA + EPwm2Regs.CMPA.half.CMPA + EPwm3Regs.CMPA.half.CMPA;
        }

      // 1. 复位 ADC 序列，为下一次触发做准备 ---
      AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
      // 2. 清除 ADC 中断标志位
      AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
      // 3. 应答 PIE Group 1 (ADCINT 所在的组)
      PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
      // EPwm1Regs.ETCLR.bit.INT = 1;            // 这行不再需要了，因为我们没用 ePWM 中断
      // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // 这行不再需要了，因为我们没用 ePWM 中断

      can_pack_cnt++;
      if (can_pack_cnt > 99) // 200分频执行 10ms一次
      {
          //CAN_TX_MSG_101_MDL.Vbus_Raw = ;
          TxMsg_101_MDL.bit.Ia_Raw = (Uint16)((int16_t)(m_current.Ia * 100.0f));// 0x101 报文打包：相电流 (Factor = 0.01)
          TxMsg_101_MDH.bit.Ib_Raw = (Uint16)((int16_t)(m_current.Ib * 100.0f));
          TxMsg_101_MDH.bit.Ic_Raw = (Uint16)((int16_t)(m_current.Ic * 100.0f));

          TxMsg_102_MDL.bit.Theta_Raw  = (Uint16)(foc.Theta * 100.0f);
          TxMsg_102_MDL.bit.Speed_Raw  = (Uint16)((int16_t)(speed_fdb * 100.f));
          TxMsg_102_MDH.bit.Iq_Ref_Raw = (Uint16)((int16_t)(pi_iq.Ref * 100.0f));
          TxMsg_102_MDH.bit.Iq_Fdb_Raw = (Uint16)((int16_t)(foc.Iq * 100.0f));

          can_pack_cnt = 0;
          can_tx_ready_flag = 1;
      }

} // 中断函数结束

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
        i = 0;
}

