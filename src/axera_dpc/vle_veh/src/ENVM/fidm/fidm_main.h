/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wulin <wulin1@senseauto.com>
 */
#pragma once
#ifndef FIDM_MAIN_H
#define FIDM_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
 INCLUDES
*****************************************************************************/
#include "./fidm_ext.h"
#include "../envm_consts.h"
/*****************************************************************************
 TYPEDEFS GLOBAL VARIABLE
*****************************************************************************/
// FCB Multipoint conbination
#define Bitmask_NONE 0U
#define Bitmask_F0 1U
#define Bitmask_F1 2U
#define Bitmask_F2 4U
#define Bitmask_F3 8U
#define Bitmask_F4 16U
#define Bitmask_F5 32U
#define Bitmask_F6 64U
#define Bitmask_F7 128U

typedef boolean Bit_Array[8];

typedef boolean ADCU_FID_Bit_Array[36][8];
typedef boolean PDCU_FID_Bit_Array[26][8];

typedef enum {
    FIDM_INIT,
    FIDM_OK,
} FIDMState_t;

/*****************************************************************************
  FUNCTION
*****************************************************************************/
void FIDM_Init();

void FIDConvert2SWC(Envm_FIDMOutPro_t* proPorts);

void FIDM_detect(const Envm_FIDInport_t* reqPorts, Envm_FIDMOutPro_t* proPorts);

#ifdef __cplusplus
}
#endif
#endif