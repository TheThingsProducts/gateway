// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef SSM_BUTTON_H
#define SSM_BUTTON_H

uint32_t SSMButton_GetStartTick(void);
bool     SSMButton_WasPressed10Milliseconds(void);
bool     SSMButton_WasPressed2Seconds(void);
bool     SSMButton_WasPressed5Seconds(void);
bool     SSMButton_IsPressed(void);
void     SSMButton_Reset(void);
void     SSMButton_Tasks(void);

#endif /* SSM_BUTTON_H */
