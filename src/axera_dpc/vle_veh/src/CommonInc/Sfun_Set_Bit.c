// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "Sfun_Set_Bit.h"

void set_bit(uint32_T uiInitialValue,
             boolean_T bBitValueArray[],
             uint8_T bBitIdxArray[],
             uint8_T uiArrayLength,
             uint32_T *uiValueAftBitSet) {
    *uiValueAftBitSet = uiInitialValue;

    for (uint8_T i = 0; i < uiArrayLength; i++) {
        *uiValueAftBitSet |= (bBitValueArray[i] << bBitIdxArray[i]);
    }
}
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"