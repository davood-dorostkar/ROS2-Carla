/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_H_
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcf_alp_ext.h"
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef struct {
    // unit delay
    // LateralPositionStepDetecion
    boolean ud_bPrevAdjAvailable_bool;
    float32 ud_fPrevAdjPosX_met;
    boolean ud_bPrevAdjStepDetected_bool;
    boolean ud_bPrevAdjStepDebounced_bool;
    // DetermineLateralPosition
    float32 ud_fPrevEgoLanePosX_met;
    float32 ud_fPrevAdjLanePosX_met;
    boolean ud_bPrevAdjLaneLatPosValid_met;
    // DetermineResetCondition
    float32 ud_uiPrevCounterValue_per;

    // previous input
    boolean bPrevStepDtctOnBridging_bool;
    float32 uiPrevCounterValue_perc;
    float32 fPrevLatPos_met;

    boolean bStepDebounced;
    boolean bStepDebouncedOnBridging;
    float32 fLateralPosition;
    boolean bLateralPositionValid;
    boolean bAdjLaneBridging;
    float32 uiCounterValue;
    boolean bCounterValueValid;

    float32 fStepTimer;
} sALPLaneInfo_t;

typedef sALPLaneInfo_t sALPLaneInfo_Array[eLaneNumber];

typedef struct {
    ALPState_t eALPState;
    sALPEgoLaneInfo_st_Array aEgoLanes;
    sALPAdjLaneInfo_st_Array aAdjLanes;
    sALPLaneInfo_Array aAdjLaneCal;
} sALPCalculate_t;
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_H_
