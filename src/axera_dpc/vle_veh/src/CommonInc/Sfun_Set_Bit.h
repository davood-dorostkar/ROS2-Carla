#ifndef SFUN_SET_BIT_H_
#define SFUN_SET_BIT_H_

#include "rtwtypes.h"
extern void set_bit(uint32_T uiInitialValue,
                    boolean_T bBitValueArray[],
                    uint8_T bBitIdxArray[],
                    uint8_T uiArrayLength,
                    uint32_T *uiValueAftBitSet);

#endif