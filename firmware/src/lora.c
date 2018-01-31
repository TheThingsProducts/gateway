// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#include "app_lora.h"

LORA_DATA loraData = {0};

void LORA_Initialize(void)
{
    /* Place the App state machine in its initial state. */
    loraData.state = LORA_STATE_INIT;

    APP_LORA_Initialize();
}

/******************************************************************************
  Function:
    void LORA_Tasks ( void )

  Remarks:
    See prototype in lora.h.
 */

void LORA_Tasks(void)
{
    APP_LORA_Tasks();
}

/*******************************************************************************
 End of File
 */
