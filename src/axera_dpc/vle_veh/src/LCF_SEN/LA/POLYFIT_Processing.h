/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * mengchen <mengchen@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_LA_POLYFITPROCESSING_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_LA_POLYFITPROCESSING_H_
#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "tue_common_libs.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define POINTS_ARRAY_SIZE (100U)
#define POLYFIT_SAMPLE_POINTS_HALF (10U)
#define POLYFIT_SAMPLE_POINTS_DOUBLE (40U)

typedef struct {
    REAL32_T fX_met;
    REAL32_T fY_met;
    REAL32_T fZ_met;
} PFTPoint3D_t;

typedef struct {
    PFTPoint3D_t PFTLanePoint[POINTS_ARRAY_SIZE];
    UINT8_T PFTLanePointSize;
} sPFTLane_t;

boolean LaneFitSuccess(sPFTOutput_t* sPFTOutput);

boolean LaneFitProcess(const sPFTLane_t* sPFTLane,
                       sPFTInput_t* sPFTInput,
                       sPFTOutput_t* sPFTOutput);

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_LA_POLYFITPROCESSING_H_
