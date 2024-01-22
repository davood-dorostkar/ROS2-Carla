/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_MAIN_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_MAIN_H_
#include "LBP_Ext.h"
#include "LBP_CheckLaneProperity.h"
#include "LBP_EgoLaneGeneration.h"
#include "LBP_LaneFilterProcessing.h"
#include "LBP_UncoupledLaneProcessing.h"

/***********************************Declaration of LBP
 * function************************************/
void crvKalmanFilter(const crvKFInTypeV2 *inputs, crvKFOutType *outputs);
void laneKalmanFilter_Left(const laneKFInTypeV3 *inputs,
                           laneKFOutType *outputs);
void laneKalmanFilter_Right(const laneKFInTypeV3 *inputs,
                            laneKFOutType *outputs);
void laneKalmanFilter_Center(const laneKFInTypeV3 *inputs,
                             laneKFOutType *outputs);

// extern void LBPInit(sLBPInput_t  *pLBPInput,  sLBPParam_t *pLBPParam,
//                    sLBPOutput_t *pLBPOutput, sLBPDebug_t *pLBPDebug);

extern void INPUT_UncoupledLaneProcessing(const sLBPInput_t *pLBPInput,
                                          const sLBPParam_t *pLBPParam,
                                          const sCLPOutput_t *pCLPOutput,
                                          const sLFPOutput_t *pLFPOutput,
                                          sULPInput_t *pULPInput,
                                          sULPParam_t *pULPParam);

extern void UncoupledLaneProcessing(const sULPInput_t *pULPInput,
                                    const sULPParam_t *pULPParam,
                                    sULPOutput_t *pULPOutput,
                                    sULPDebug_t *pULPDebug);

extern void INPUT_CheckLaneProperity(const sLBPInput_t *pLBPInput,
                                     const sLBPParam_t *pLBPParam,
                                     const sULPOutput_t *pULPOutput,
                                     const sLFPOutput_t *pLFPOutput,
                                     sCLPInput_t *pCLPInput,
                                     sCLPParam_t *pCLPParam);

extern void CheckLaneProperity(sCLPInput_t const *pCLPInput,
                               sCLPParam_t const *pCLPParam,
                               sCLPOutput_t *pCLPOutput,
                               sCLPDebug_t *pCLPDebug);

extern void INPUT_LaneFilterProcessing(const sLBPInput_t *pLBPInput,
                                       const sLBPParam_t *pLBPParam,
                                       const sULPOutput_t *pULPOutput,
                                       const sCLPOutput_t *pCLPOutput,
                                       sLFPInput_t *pLFPInput,
                                       sLFPParam_t *pLFPParam);

extern void LaneFilterProcessing(sLFPInput_t const *pLFPInput,
                                 sLFPParam_t const *pLFPParam,
                                 sLFPOutput_t *pLFPOutput,
                                 sLFPDebug_t *pLFPDebug);

extern void INPUT_EgoLaneGeneration(const sLBPInput_t *pLBPInput,
                                    const sLBPParam_t *pLBPParam,
                                    const sULPOutput_t *pULPOutput,
                                    const sCLPOutput_t *pCLPOutput,
                                    const sLFPOutput_t *pLFPOutput,
                                    sELGInput_t *pELGInput,
                                    sELGParam_t *pELGParam);

extern void EgoLaneGeneration(sELGInput_t const *pELGInput,
                              sELGParam_t const *pELGParam,
                              sELGOutput_t *pELGOutput,
                              sELGDebug_t *pELGDebug);

extern void LBPOutput(const sULPOutput_t *pULPOutput,
                      const sULPDebug_t *pULPDebug,
                      const sCLPOutput_t *pCLPOutput,
                      const sCLPDebug_t *pCLPDebug,
                      const sLFPOutput_t *pLFPOutput,
                      const sLFPDebug_t *pLFPDebug,
                      const sELGOutput_t *pELGOutput,
                      const sELGDebug_t *pELGDebug,
                      sLBPOutput_t *pLBPOutput,
                      sLBPDebug_t *pLBPDebug);

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_MAIN_H_
