#ifndef MY_EPWM_H_
#define MY_EPWM_H_

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

// 1. 函数声明（接口）
void InitTzPwm(void);
void InitAllEPwms(void);
void InitEPwm1(void);
void InitEPwm2(void);
void InitEPwm3(void);

__interrupt void epwm1_tz_int_isr(void);
// 前面加两个下划线 _ _ 表示 退出时使用 IRET 恢复现场 INTM清零 所有进栈的核心状态都会恢复
// 不加下划线  退出时使用 RET  只将程序计数器PC返回

#endif
