/*
 * Copyright (C) 2022-2024 by SoftwareMotion Group Limited. All rights reserved.
 * 
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "./dim_utils.h"
void DIMGetInputValue_Float(const DIMInputValue_t inputValue,
                            const float32 fDefaultVal,
                            float32 *const pOutVal) {
    switch (inputValue.eSignalType) {
        case DIMInputSignalType_Float:
            *pOutVal = inputValue.uValue.fValue;
            break;
        default:
            *pOutVal = fDefaultVal;
            break;
    }
}

void DIMGetInputValue_Bool(const DIMInputValue_t inputValue,
                           const boolean bDefaultVal,
                           boolean *const pOutVal) {
    switch (inputValue.eSignalType) {
        case DIMInputSignalType_Bool:
            *pOutVal = (boolean)inputValue.uValue.bValue;
            break;
        default:
            *pOutVal = bDefaultVal;
            break;
    }
}

void DIMGetInputValue_UnsigedInt(const DIMInputValue_t inputValue,
                                 const uint8 uiDefaultVal,
                                 uint8 *const pOutVal) {
    switch (inputValue.eSignalType) {
        case DIMInputSignalType_Bool:
            *pOutVal = inputValue.uValue.uiValue;
            break;
        default:
            *pOutVal = uiDefaultVal;
            break;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */