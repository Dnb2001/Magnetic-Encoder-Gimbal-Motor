#include "Diagnostic.h"
#include "CANIF.h"
#include "IsoTp.h"
#include "UDS.h"

void Diagnostic_Init(void)
{
    CANIF_Init();
    IsoTp_Init();
    UDS_Init();
}

void Diagnostic_Task1ms(void)
{
    IsoTp_Task1ms();

    if((IsoTp_IsRxReady() != 0U) && (IsoTp_IsTxIdle() != 0U))
    {
        UDS_ProcessRequest(IsoTp_GetRxData(), IsoTp_GetRxLength());
        IsoTp_ReleaseRx();
    }

    UDS_Task1ms();
}

