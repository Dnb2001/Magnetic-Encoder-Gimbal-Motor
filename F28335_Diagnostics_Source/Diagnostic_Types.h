/*
 * Diagnostic_Types.h
 *
 * F28335 的最小可寻址单元是 16 bit，因此协议中的一个 8 bit 字节
 * 使用 Uint16 保存，并约定只有低 8 bit 有效。
 */
#ifndef DIAGNOSTIC_TYPES_H_
#define DIAGNOSTIC_TYPES_H_

#ifdef UNIT_TEST
#include <stdint.h>
typedef uint16_t Uint16;
typedef uint32_t Uint32;
#else
#include "DSP2833x_Device.h"
#endif

#include "Diagnostic_Config.h"

typedef struct
{
    Uint16 id;
    Uint16 dlc;
    Uint16 data[8];
} CAN_Frame;

#endif /* DIAGNOSTIC_TYPES_H_ */

