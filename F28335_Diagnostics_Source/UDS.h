#ifndef UDS_H_
#define UDS_H_

#include "Diagnostic_Types.h"

typedef enum
{
    UDS_SESSION_DEFAULT = 1U,
    UDS_SESSION_PROGRAMMING = 2U,
    UDS_SESSION_EXTENDED = 3U
} UDS_SESSION;

void UDS_Init(void);
void UDS_ProcessRequest(const Uint16 *request, Uint16 request_length);
void UDS_Task1ms(void);
UDS_SESSION UDS_GetCurrentSession(void);
Uint16 UDS_GetSecurityLevel(void);

#endif /* UDS_H_ */

