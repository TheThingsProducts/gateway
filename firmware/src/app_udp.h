// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_UDP_H
#define _APP_UDP_H

#include "app.h"

#ifdef __cplusplus
extern "C"
{
#endif

void APP_UDP_Initialize(void);
void APP_UDP_Tasks(void);
void APP_UDP_Reset(void);
bool APP_UDP_IsConnected(void);
void APP_UDP_GetPacketCount(uint32_t* pup, uint32_t* pdown);
void APP_UDP_GetEUI64String(char out[17]);

#ifdef __cplusplus
}
#endif

#endif /* _APP_UDP_H */
