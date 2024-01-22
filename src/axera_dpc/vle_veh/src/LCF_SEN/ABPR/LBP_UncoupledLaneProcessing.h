/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_UNCOUPLEDLANEPROCESSING_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_UNCOUPLEDLANEPROCESSING_H_

#include "tue_common_libs.h"
#include "LBP_Ext.h"

void UncoupledLaneReset(void);

void INPUT_DetermineMeasureQuality(const sULPInput_t *pULPInput,
                                   const sULPParam_t *pULPParam,
                                   sDMQInput_t *pDMQInputCplLf,
                                   sDMQParam_t *pDMQParamCplLf,
                                   sDMQInput_t *pDMQInputCplRi,
                                   sDMQParam_t *pDMQParamCplRi);

void DMQ_OverallStandardDevQuality(const sDMQInput_t *pDMQInput,
                                   REAL32_T *fMeasureQuality);

void DetermineMeasureQuality(const UINT8_T LaneIndex,
                             const sDMQInput_t *pDMQInput,
                             const sDMQParam_t *pDMQParam,
                             sDMQOutput_t *pDMQOutput,
                             sDMQDebug_t *pDMQDebug);
void INPUT_LaneMotionCompensation(const sULPInput_t *pULPInput,
                                  const sULPParam_t *pULPParam,
                                  const sDMQOutput_t *pDMQOutputCplLf,
                                  const sDMQOutput_t *pDMQOutputCplRi,
                                  sLMCInput_t *pLMCInputCplLf,
                                  sLMCParam_t *pLMCParamCplLf,
                                  sLMCInput_t *pLMCInputCplRi,
                                  sLMCParam_t *pLMCParamCplRi);
void LMC_DetrmCompensationSet(REAL32_T fLastValidLength,
                              REAL32_T fLastOverallQualityIn,
                              UINT8_T bLastAvailableIn,
                              sLMCInput_t const *pLMCInput,
                              sLMCParam_t const *pLMCParam,
                              UINT8_T *bSetLmcEnable,
                              sLMCDebug_t *pLMCDebug);

void LMC_DetrmCompensationReset(REAL32_T fLastMaxValidLengthCpmn,
                                UINT8_T bLastInVldTraj3rd,
                                UINT8_T bRstLmcByTime,
                                sLMCInput_t const *pLMCInput,
                                sLMCParam_t const *pLMCParam,
                                UINT8_T *bRstLmcEnable,
                                sLMCDebug_t *pLMCDebug);

void LMC_EnableFlagSmooth(UINT8_T bLastEnaMotionCmpn,
                          UINT8_T bEnaMotionCmpn,
                          REAL32_T fPosY03rd,
                          REAL32_T fLastPosY03rd,
                          REAL32_T fHeading3rd,
                          REAL32_T fLastHeading3rd,
                          UINT8_T bLastRawEnaApplySmth,
                          UINT8_T *bRawEnaApplySmth,
                          UINT8_T *bEnaApplySmth,
                          sLMCDebug_t *pLMCDebug);

UINT8_T LMC_Comparation(REAL32_T input, REAL32_T threshold);

void LaneMotionCompensation(const UINT8_T LaneIndex,
                            sLMCInput_t const *pLMCInput,
                            sLMCParam_t const *pLMCParam,
                            sLMCOutput_t *pLMCOutput,
                            sLMCDebug_t *pLMCDebug);
void INPUT_UncoupledLaneBridge(const sULPInput_t *pULPInput,
                               const sULPParam_t *pULPParam,
                               sULBInput_t *pULBInput,
                               sULBParam_t *pULBParam);

void ULB_LaneParallelism(const sULBInput_t *pULBInput,
                         const sULBParam_t *pULBParam,
                         sULBDebug_t *pULBDebug,
                         REAL32_T *fMaxLengthUnCpl,
                         REAL32_T *fRotPosYUnCplLf,
                         REAL32_T *fRotPosYUnCplRi,
                         UINT8_T *bRawNotParallelUnCpl,
                         UINT8_T *bNotParallelUnCpl);

void ULB_LateralDisDiffOverTine(const sULBInput_t *pULBInput,
                                const sULBParam_t *pULBParam,
                                UINT8_T bLastRawNotParallelUnCpl,
                                UINT8_T bRawNotParallelUnCpl,
                                REAL32_T fMaxLengthUnCpl,
                                REAL32_T fRotPosYUnCplLf,
                                REAL32_T fRotPosYUnCplRi,
                                sULBDebug_t *pULBDebug,
                                REAL32_T *fDevLatDistCplLf,
                                REAL32_T *fDevLatDistCplRi,
                                UINT8_T *bTrigLatDistDev);

void ULB_PenaltyStepDetection(const sULBInput_t *pULBInput,
                              const sULBParam_t *pULBParam,
                              sULBDebug_t *pULBDebug,
                              UINT8_T *bStepDtctUnCplLf,
                              UINT8_T *bStepDtctUnCplRi);

void ULB_OutputBitfiled(UINT8_T bStepDtctUnCplLf,
                        UINT8_T bStepDtctUnCplRi,
                        UINT8_T bNotParallelBridgeUnCplLf,
                        UINT8_T bNotParallelBridgeUnCplRi,
                        UINT8_T bTrigHeadingDevUnCpl,
                        UINT8_T bTrigCrvDevUnCpl,
                        sULBOutput_t *pULBOutput);

void UncoupledLaneBridge(const sULBInput_t *pULBInput,
                         const sULBParam_t *pULBParam,
                         sULBOutput_t *pULBOutput,
                         sULBDebug_t *pULBDebug);
void INPUT_DetermineLaneBoundary(const sULPInput_t *pULPInput,
                                 const sULPParam_t *pULPParam,
                                 const sDMQOutput_t *pDMQOutputCplLf,
                                 const sDMQOutput_t *pDMQOutputCplRi,
                                 const sLMCOutput_t *pLMCOutputCplLf,
                                 const sLMCOutput_t *pLMCOutputCplRi,
                                 const sULBOutput_t *pULBOutput,
                                 sDLBInput_t *pDLBInput,
                                 sDLBParam_t *pDLBParam);
void DetermineLaneBoundary(const sDLBInput_t *pDLBInput,
                           const sDLBParam_t *pDLBParam,
                           sDLBOutput_t *pDLBOutput,
                           sDLBDebug_t *pDLBDebug);
void OUTPUT_UncoupledLaneProcess(const sDMQOutput_t *pDMQOutputCplLf,
                                 const sDMQDebug_t *pDMQDebugCplLf,
                                 const sDMQOutput_t *pDMQOutputCplRi,
                                 const sDMQDebug_t *pDMQDebugCplRi,
                                 const sLMCOutput_t *pLMCOutputCplLf,
                                 const sLMCDebug_t *pLMCDebugCplLf,
                                 const sLMCOutput_t *pLMCOutputCplRi,
                                 const sLMCDebug_t *pLMCDebugCplRi,
                                 const sULBOutput_t *pULBOutput,
                                 const sULBDebug_t *pULBDebug,
                                 const sDLBOutput_t *pDLBOutput,
                                 const sDLBDebug_t *pDLBDebug,
                                 sULPOutput_t *pULPOutput,
                                 sULPDebug_t *pULPDebug);
void UncoupledLaneProcessing(const sULPInput_t *pULPInput,
                             const sULPParam_t *pULPParam,
                             sULPOutput_t *pULPOutput,
                             sULPDebug_t *pULPDebug);

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_UNCOUPLEDLANEPROCESSING_H_
