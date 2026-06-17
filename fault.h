#ifndef FAULT_H_
#define FAULT_H_

#include "DSP28x_Project.h"     // 使用 Uint16 类型；如果你工程里不用这个，可换成 stdint.h + uint16_t


/*=========================================================
 * 故障位定义
 * 一个 Uint16 共有16bit，可以表示16种故障
 *=========================================================*/
typedef enum
{
    FAULT_NONE                        = 0x0000u,

    FAULT_OVER_VOLT                   = 0x0001u,   // bit0  母线过压
    FAULT_UNDER_VOLT                  = 0x0002u,   // bit1  母线欠压
    FAULT_OVER_CURR_HARDWARE          = 0x0004u,   // bit2  IR2136硬件过流
    FAULT_OVER_CURR_SOFTWARE          = 0x0008u,   // bit3  软件检测过流
    FAULT_OVER_CURR_CBC               = 0x0010u,   // bit4  单PWM周期过流

    FAULT_PHASE_CURR_OFFSET_CHECK     = 0x0020u,   // bit5  电流采样零点异常
    FAULT_POSITION_SENSOR             = 0x0040u,   // bit6  位置传感器异常
    FAULT_MOTOR_STALL                 = 0x0080u,   // bit7  电机堵转

    FAULT_CAN_RX_TIMEOUT              = 0x0100u,   // bit8  CAN接收超时
    FAULT_CAN_BUS_OFF                 = 0x0200u    // bit9  CAN Bus-Off

} FaultFlag_e;


/*=========================================================
 * 故障状态结构体
 *=========================================================*/
typedef struct
{
    Uint16 active;          // 当前正在发生的故障
    Uint16 latched;         // 锁存故障，发生过就保持，直到手动清除
    Uint16 warning;         // 警告类，不一定立刻停机
    Uint16 first_fault;     // 第一个触发的故障
} FaultStatus_t;


/*=========================================================
 * 全局故障变量声明
 * 注意：这里只是声明，不分配空间
 * 真正分配空间在 fault.c 里
 *=========================================================*/
extern volatile FaultStatus_t g_fault;


/*=========================================================
 * 故障分类掩码
 *=========================================================*/

/* 需要立即停PWM的严重故障 */
#define FAULT_STOP_IMMEDIATELY_MASK     \
(                                       \
    FAULT_OVER_CURR_HARDWARE        |   \
    FAULT_OVER_CURR_SOFTWARE        |   \
    FAULT_OVER_CURR_CBC             |   \
    FAULT_POSITION_SENSOR               \
)

/* 需要进入故障状态，但可以由状态机统一处理的故障 */
#define FAULT_STOP_BY_STATE_MASK         \
(                                       \
    FAULT_OVER_VOLT                 |   \
    FAULT_UNDER_VOLT                |   \
    FAULT_PHASE_CURR_OFFSET_CHECK   |   \
    FAULT_MOTOR_STALL               |   \
    FAULT_CAN_RX_TIMEOUT            |   \
    FAULT_CAN_BUS_OFF                   \
)

/* 所有需要停机的故障 */
#define FAULT_STOP_ALL_MASK              \
(                                       \
    FAULT_STOP_IMMEDIATELY_MASK     |   \
    FAULT_STOP_BY_STATE_MASK            \
)


/*=========================================================
 * 故障操作宏
 *=========================================================*/

/* 设置故障：active 和 latched 同时置位 */
#define FAULT_SET(fault_mask)                                \
do                                                           \
{                                                            \
    Uint16 fault_temp;                                       \
    fault_temp = (Uint16)(fault_mask);                       \
                                                             \
    g_fault.active  |= fault_temp;                           \
    g_fault.latched |= fault_temp;                           \
                                                             \
    if(g_fault.first_fault == FAULT_NONE)                    \
    {                                                        \
        g_fault.first_fault = fault_temp;                    \
    }                                                        \
} while(0)


/* 清除当前故障，不清除锁存故障 */
#define FAULT_CLEAR_ACTIVE(fault_mask)                       \
do                                                           \
{                                                            \
    g_fault.active &= (Uint16)(~((Uint16)(fault_mask)));      \
} while(0)


/* 清除锁存故障 */
#define FAULT_CLEAR_LATCHED(fault_mask)                      \
do                                                           \
{                                                            \
    g_fault.latched &= (Uint16)(~((Uint16)(fault_mask)));     \
} while(0)


/* 设置警告 */
#define FAULT_WARNING_SET(warning_mask)                      \
do                                                           \
{                                                            \
    g_fault.warning |= (Uint16)(warning_mask);                \
} while(0)


/* 清除警告 */
#define FAULT_WARNING_CLEAR(warning_mask)                    \
do                                                           \
{                                                            \
    g_fault.warning &= (Uint16)(~((Uint16)(warning_mask)));   \
} while(0)


/* 清除所有故障和警告 */
#define FAULT_CLEAR_ALL()                                    \
do                                                           \
{                                                            \
    g_fault.active      = FAULT_NONE;                         \
    g_fault.latched     = FAULT_NONE;                         \
    g_fault.warning     = FAULT_NONE;                         \
    g_fault.first_fault = FAULT_NONE;                         \
} while(0)


/* 判断某个当前故障是否存在 */
#define FAULT_IS_ACTIVE(fault_mask)                          \
    ((g_fault.active & (Uint16)(fault_mask)) != 0u)


/* 判断某个锁存故障是否存在 */
#define FAULT_IS_LATCHED(fault_mask)                         \
    ((g_fault.latched & (Uint16)(fault_mask)) != 0u)


/* 判断是否存在任何当前故障 */
#define FAULT_HAS_ANY_ACTIVE()                               \
    (g_fault.active != FAULT_NONE)


/* 判断是否存在任何锁存故障 */
#define FAULT_HAS_ANY_LATCHED()                              \
    (g_fault.latched != FAULT_NONE)


/* 判断是否需要立即停PWM */
#define FAULT_NEED_STOP_IMMEDIATELY()                        \
    ((g_fault.active & FAULT_STOP_IMMEDIATELY_MASK) != 0u)


/* 判断是否需要进入故障状态 */
#define FAULT_NEED_STOP()                                    \
    ((g_fault.active & FAULT_STOP_ALL_MASK) != 0u)


#endif
