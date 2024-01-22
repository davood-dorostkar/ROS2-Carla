/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wulin <wulin1@senseauto.com>
 */
#pragma once
#ifndef FIDM_EXT_H
#define FIDM_EXT_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
 INCLUDES
 *****************************************************************************/
#include "TM_Global_Types.h"
#include "envm_ext.h"

/*****************************************************************************
  FUNCTION
*****************************************************************************/
void FIDMProcess(const Envm_FIDInport_t* reqPorts, Envm_FIDMOutPro_t* proPorts);

#ifdef __cplusplus
}
#endif
#endif