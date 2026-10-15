/*
 * CANIF_ByteCodec.c
 *
 * F28335 eCAN 的 CANMC.DBO 会改变 Byte0~Byte7 在 MDL/MDH 中的位置。
 * 驱动读取当前 DBO 后调用这里，不需要修改全局 DBO，也就不会影响
 * 其他邮箱。每次移位前都先转换成 Uint32；这一点对 16 位 C28x
 * 编译器尤其重要。
 */
#include "CANIF_ByteCodec.h"

Uint32 CANIF_PackMDL(const Uint16 *data, Uint16 dbo)
{
    if(dbo != 0U)
    {
        return ((Uint32)(data[0] & 0xFFU)) |
               (((Uint32)(data[1] & 0xFFU)) << 8U) |
               (((Uint32)(data[2] & 0xFFU)) << 16U) |
               (((Uint32)(data[3] & 0xFFU)) << 24U);
    }

    return (((Uint32)(data[0] & 0xFFU)) << 24U) |
           (((Uint32)(data[1] & 0xFFU)) << 16U) |
           (((Uint32)(data[2] & 0xFFU)) << 8U) |
           ((Uint32)(data[3] & 0xFFU));
}

Uint32 CANIF_PackMDH(const Uint16 *data, Uint16 dbo)
{
    if(dbo != 0U)
    {
        return ((Uint32)(data[4] & 0xFFU)) |
               (((Uint32)(data[5] & 0xFFU)) << 8U) |
               (((Uint32)(data[6] & 0xFFU)) << 16U) |
               (((Uint32)(data[7] & 0xFFU)) << 24U);
    }

    return (((Uint32)(data[4] & 0xFFU)) << 24U) |
           (((Uint32)(data[5] & 0xFFU)) << 16U) |
           (((Uint32)(data[6] & 0xFFU)) << 8U) |
           ((Uint32)(data[7] & 0xFFU));
}

void CANIF_UnpackData(Uint32 mdl, Uint32 mdh, Uint16 dbo, Uint16 *data)
{
    if(dbo != 0U)
    {
        data[0] = (Uint16)(mdl & 0xFFUL);
        data[1] = (Uint16)((mdl >> 8U) & 0xFFUL);
        data[2] = (Uint16)((mdl >> 16U) & 0xFFUL);
        data[3] = (Uint16)((mdl >> 24U) & 0xFFUL);
        data[4] = (Uint16)(mdh & 0xFFUL);
        data[5] = (Uint16)((mdh >> 8U) & 0xFFUL);
        data[6] = (Uint16)((mdh >> 16U) & 0xFFUL);
        data[7] = (Uint16)((mdh >> 24U) & 0xFFUL);
        return;
    }

    data[0] = (Uint16)((mdl >> 24U) & 0xFFUL);
    data[1] = (Uint16)((mdl >> 16U) & 0xFFUL);
    data[2] = (Uint16)((mdl >> 8U) & 0xFFUL);
    data[3] = (Uint16)(mdl & 0xFFUL);
    data[4] = (Uint16)((mdh >> 24U) & 0xFFUL);
    data[5] = (Uint16)((mdh >> 16U) & 0xFFUL);
    data[6] = (Uint16)((mdh >> 8U) & 0xFFUL);
    data[7] = (Uint16)(mdh & 0xFFUL);
}
