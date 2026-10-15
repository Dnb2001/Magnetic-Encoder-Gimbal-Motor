#ifndef ISOTP_H_
#define ISOTP_H_

#include "Diagnostic_Types.h"

typedef enum
{
    ISOTP_TX_IDLE = 0,
    ISOTP_TX_SEND_SF,
    ISOTP_TX_SEND_FF,
    ISOTP_TX_WAIT_FC,
    ISOTP_TX_SEND_CF,
    ISOTP_TX_WAIT_STMIN,
    ISOTP_TX_ERROR
} ISOTP_TX_STATE;

typedef enum
{
    ISOTP_RX_IDLE = 0,
    ISOTP_RX_WAIT_CF,
    ISOTP_RX_ERROR
} ISOTP_RX_STATE;

typedef enum
{
    ISOTP_ERROR_NONE = 0,
    ISOTP_ERROR_RX_OVERFLOW,
    ISOTP_ERROR_WRONG_SN,
    ISOTP_ERROR_N_BS_TIMEOUT,
    ISOTP_ERROR_N_CR_TIMEOUT,
    ISOTP_ERROR_FC_OVERFLOW,
    ISOTP_ERROR_TOO_MANY_WAIT,
    ISOTP_ERROR_INVALID_FRAME
} ISOTP_ERROR;

void IsoTp_Init(void);
void IsoTp_Task1ms(void);

Uint16 IsoTp_Send(const Uint16 *data, Uint16 length);
Uint16 IsoTp_IsTxIdle(void);

Uint16 IsoTp_IsRxReady(void);
const Uint16 *IsoTp_GetRxData(void);
Uint16 IsoTp_GetRxLength(void);
void IsoTp_ReleaseRx(void);

ISOTP_TX_STATE IsoTp_GetTxState(void);
ISOTP_RX_STATE IsoTp_GetRxState(void);
ISOTP_ERROR IsoTp_GetError(void);
void IsoTp_ClearError(void);

#endif /* ISOTP_H_ */

