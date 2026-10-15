#ifndef CANIF_H_
#define CANIF_H_

#include "Diagnostic_Types.h"

void CANIF_Init(void);
Uint16 CANIF_Send(const CAN_Frame *frame);
Uint16 CANIF_Read(CAN_Frame *frame);
void CANIF_RxISRHandler(void);
Uint32 CANIF_GetRxOverflowCount(void);

#endif /* CANIF_H_ */

