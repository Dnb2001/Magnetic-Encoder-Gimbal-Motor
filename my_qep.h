#ifndef MY_QEP_H_
#define MY_QEP_H_

#include "DSP2833x_Device.h"

/*
 * AS5047P ABI 默认配置：1000 PPR，eQEP 4 倍频后为 4000 count/rev。
 * 如果以后通过 SPI 修改了 ABIRES / ABIBIN，这里必须同步修改。
 */
#define QEP_COUNTS_PER_REV          4000L
#define QEP_COUNTS_PER_REV_F        4000.0f

/* M 法：4 ms 固定窗口 -> 250 Hz 更新一次 */
#define QEP_M_SAMPLE_HZ             250U
#define QEP_M_SAMPLE_PERIOD_S       0.004f

/*
 * T 法：每 1 个 QCLK 产生一次 Unit Position Event；
 * CAPCLK = SYSCLKOUT / 128 = 1.171875 MHz (SYSCLKOUT=150 MHz)
 */
#define QEP_T_UPPS_BITS             0U      /* UPEVNT = QCLK / 1   */
#define QEP_T_UNIT_COUNTS           1.0f
#define QEP_T_CCPS_BITS             7U      /* CAPCLK = SYSCLK /128 */
#define QEP_T_CAPCLK_HZ             1171875.0f

/* M/T 切换滞环：低速用 T 法，高速用 M 法 */
#define QEP_SWITCH_TO_T_RPM         200.0f
#define QEP_SWITCH_TO_M_RPM         300.0f

/*
 * 与当前 Calculate_QEP_Angle() 中机械角度前的负号保持同一方向定义。
 * 如果以后发现反馈方向相反，只改这里的 -1.0f / +1.0f 即可。
 */
#define QEP_SPEED_DIRECTION_SIGN    (-1.0f)

typedef enum
{
    QEP_SPEED_METHOD_T = 0,
    QEP_SPEED_METHOD_M = 1
} QEP_SPEED_METHOD_e;

typedef struct
{
    /* 最终给控制使用的速度 */
    float mech_rpm;
    float elec_rad_s;

    /* M 法调试量 */
    float m_rpm_raw;
    float m_rpm_filt;
    int32 m_delta_count;
    Uint16 m_valid;

    /* T 法调试量 */
    float t_rpm_raw;
    float t_rpm_filt;
    Uint16 t_capture_period;
    Uint16 t_valid;

    /* 当前融合选择 */
    Uint16 method;

    /* 诊断计数 */
    Uint16 t_overflow_count;
    Uint16 t_dir_error_count;
} QEP_SPEED_ESTIMATOR_t;

extern int32 QEP_Offset;
extern QEP_SPEED_ESTIMATOR_t g_qep_speed;

void InitEQep1_AS5047P(void);
void Calculate_QEP_Angle(void);
void InitHallGpio(void);

/* 每次进入闭环前调用一次 */
void QEP_Speed_Reset(void);

/* 在 10 kHz ADC/FOC ISR 中每次调用一次；函数内部自己判断是否有新 M/T 数据 */
void QEP_Speed_Update(void);

#endif /* MY_QEP_H_ */
