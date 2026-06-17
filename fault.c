#include "fault.h"


/*=========================================================
 * 全局故障变量定义
 * 这里是真正分配RAM空间的地方
 *=========================================================*/
volatile FaultStatus_t g_fault =
{
    FAULT_NONE,     // active
    FAULT_NONE,     // latched
    FAULT_NONE,     // warning
    FAULT_NONE      // first_fault
};
