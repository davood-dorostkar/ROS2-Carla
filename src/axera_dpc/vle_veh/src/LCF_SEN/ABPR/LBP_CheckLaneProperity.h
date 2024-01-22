/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_CHECKLANEPROPERITY_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_CHECKLANEPROPERITY_H_

#include "tue_common_libs.h"
#include "LBP_Ext.h"

void CheckLaneProperityReset(void);

void CLP_LeftLaneExitFlag(const sCLPInput_t *pCLPInput,
                          const sCLPParam_t *pCLPParam,
                          REAL32_T fHeadingAtMaxUnCplLf,
                          REAL32_T fHeadingAtMaxUnCplRi,
                          UINT8_T bEnaByLineWidthUnCpl,
                          sCLPOutput_t *pCLPOutput);
void LineMergeDetectionRi(const sCLPInput_t *pCLPInput,
                          const sCLPParam_t *pCLPParam,
                          REAL32_T fHeadingAtMaxUnCplLf,
                          REAL32_T fHeadingAtMaxUnCplRi,
                          REAL32_T fPosYAtMaxUnCplLf,
                          REAL32_T fPosYAtMaxUnCplRi,
                          REAL32_T fCurvatureAtMaxUnCplLf,
                          REAL32_T fCurvatureAtMaxUnCplRi,
                          sCLPOutput_t *pCLPOutput,
                          sCLPDebug_t *pCLPDebug);
void LineMergeDetectionLf(const sCLPInput_t *pCLPInput,
                          const sCLPParam_t *pCLPParam,
                          REAL32_T fHeadingAtMaxUnCplLf,
                          REAL32_T fHeadingAtMaxUnCplRi,
                          REAL32_T fPosYAtMaxUnCplLf,
                          REAL32_T fPosYAtMaxUnCplRi,
                          REAL32_T fCurvatureAtMaxUnCplLf,
                          REAL32_T fCurvatureAtMaxUnCplRi,
                          sCLPOutput_t *pCLPOutput,
                          sCLPDebug_t *pCLPDebug);
void LineMergeDetection(const sCLPInput_t *pCLPInput,
                        const sCLPParam_t *pCLPParam,
                        sCLPOutput_t *pCLPOutput,
                        sCLPDebug_t *pCLPDebug);
void ExitRampDetection(const sCLPInput_t *pCLPInput,
                       const sCLPParam_t *pCLPParam,
                       sCLPOutput_t *pCLPOutput,
                       sCLPDebug_t *pCLPDebug);
void DetectSlopeChange(const sCLPInput_t *pCLPInput,
                       const sCLPParam_t *pCLPParam,
                       sCLPOutput_t *pCLPOutput,
                       sCLPDebug_t *pCLPDebug);
void LaneStraightDetection(const sCLPInput_t *pCLPInput,
                           const sCLPParam_t *pCLPParam,
                           sCLPOutput_t *pCLPOutput,
                           sCLPDebug_t *pCLPDebug);
void CalculateOtherEvent(const sCLPInput_t *pCLPInput,
                         const sCLPParam_t *pCLPParam,
                         sCLPOutput_t *pCLPOutput,
                         sCLPDebug_t *pCLPDebug);
void CLP_LeftValidCorriAfterLatDisJump(const sCLPInput_t *pCLPInput,
                                       const sCLPParam_t *pCLPParam,
                                       sCLPDebug_t *pCLPDebug,
                                       UINT8_T bLatDistStepTrigLf,
                                       UINT8_T *bRawValidNewCorrLf,
                                       UINT8_T *bLaneWidthAgainNormalLf);
void LeftLatDistStepDtct(const sCLPInput_t *pCLPInput,
                         const sCLPParam_t *pCLPParam,
                         sCLPOutput_t *pCLPOutput,
                         sCLPDebug_t *pCLPDebug);
void CLP_RightValidCorriAfterLatDisJump(const sCLPInput_t *pCLPInput,
                                        const sCLPParam_t *pCLPParam,
                                        sCLPDebug_t *pCLPDebug,
                                        UINT8_T bLatDistStepTrigRi,
                                        UINT8_T *bRawValidNewCorrRi,
                                        UINT8_T *bLaneWidthAgainNormalRi);
void RightLatDistStepDtct(const sCLPInput_t *pCLPInput,
                          const sCLPParam_t *pCLPParam,
                          sCLPOutput_t *pCLPOutput,
                          sCLPDebug_t *pCLPDebug);
void OutRangeCheck(const sCLPInput_t *pCLPInput,
                   const sCLPParam_t *pCLPParam,
                   sCLPOutput_t *pCLPOutput,
                   sCLPDebug_t *pCLPDebug);
void CheckLaneValidity(const sCLPInput_t *pCLPInput,
                       const sCLPParam_t *pCLPParam,
                       sCLPOutput_t *pCLPOutput,
                       sCLPDebug_t *pCLPDebug);
void INPUT_CheckLaneProperity(const sLBPInput_t *pLBPInput,
                              const sLBPParam_t *pLBPParam,
                              const sULPOutput_t *pULPOutput,
                              const sLFPOutput_t *pLFPOutput,
                              sCLPInput_t *pCLPInput,
                              sCLPParam_t *pCLPParam);
void CheckLaneProperity(const sCLPInput_t *pCLPInput,
                        const sCLPParam_t *pCLPParam,
                        sCLPOutput_t *pCLPOutput,
                        sCLPDebug_t *pCLPDebug);

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_CHECKLANEPROPERITY_H_
