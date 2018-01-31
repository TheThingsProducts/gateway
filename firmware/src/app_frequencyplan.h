// Copyright © 2016-2018 The Things Products
// Use of this source code is governed by the MIT license that can be found in
// the LICENSE file.

#ifndef _APP_FREQPLAN_H /* Guard against multiple inclusion */
#define _APP_FREQPLAN_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        APP_FREQPLAN_CONNECTING,
        APP_FREQPLAN_REQUESTING,
        APP_FREQPLAN_PARSING,
        APP_FREQPLAN_DONE,
        APP_FREQPLAN_FAILURE,
        APP_FREQPLAN_SERVER_BUSY,
        APP_FREQPLAN_ERROR,
    } APP_FREQPLAN_STATES; // REVIEW: hide internal states, let is access by test functions

    typedef struct
    {
        APP_FREQPLAN_STATES state;
    } APP_FREQPLAN_DATA;

    void   APP_FreqPlan_Initialize(void);
    int8_t APP_FreqPlan_State();
    void   APP_FreqPlan_Reset();
    void   APP_FreqPlan_Tasks(void);

#ifdef __cplusplus
}
#endif

#endif