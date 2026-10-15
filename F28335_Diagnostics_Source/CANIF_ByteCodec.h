#ifndef CANIF_BYTE_CODEC_H_
#define CANIF_BYTE_CODEC_H_

#include "Diagnostic_Types.h"

Uint32 CANIF_PackMDL(const Uint16 *data, Uint16 dbo);
Uint32 CANIF_PackMDH(const Uint16 *data, Uint16 dbo);
void CANIF_UnpackData(Uint32 mdl, Uint32 mdh, Uint16 dbo, Uint16 *data);

#endif /* CANIF_BYTE_CODEC_H_ */
