/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_MAIN_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "lcf_alp_ext.h"

void LCF_ALP_Init_Reset(void);
void LCF_ALP_Exec(const sALPInReq_st* reqPorts,
                  const sALPParam_st* params,
                  sALPOutput_st* proPorts,
                  sALPDebug_st* debugInfo);
void ALP_PreProecss(const sALPInReq_st* reqPorts,
                    const sALPParam_st* params,
                    sALPOutput_st* proPorts,
                    sALPDebug_st* debugInfo);
void ALP_MainProcess(const sALPInReq_st* reqPorts,
                     const sALPParam_st* params,
                     sALPOutput_st* proPorts,
                     sALPDebug_st* debugInfo);
void ALP_PostProcess(const sALPInReq_st* reqPorts,
                     const sALPParam_st* params,
                     sALPOutput_st* proPorts,
                     sALPDebug_st* debugInfo);

void InitAdjLaneInput(sALPAdjLaneInfo_st* pAdjLane,
                      sALPAdjLaneInfo_st* pCalAdjLane);
void InitAdjLaneDefault(sALPAdjLaneInfo_st* pAdjLane, eALPLaneIndex elaneIndex);
void InitALPDefaultOutPut(sALPOutput_st* proPorts, sALPDebug_st* debugInfo);
void AdjacentLaneStabilization(const sALPInReq_st* reqPorts,
                               const sALPParam_st* params,
                               sALPDebug_st* debugInfo,
                               eALPLaneIndex elaneIndex);

void LaterPositionStepDetection(const sALPEgoLaneInfo_st* pEgoLane,
                                const sALPAdjLaneInfo_st* pAdjLane,
                                const sALPParam_st* params,
                                sALPLaneInfo_t* pCalLaneInfo,
                                const float32 fCycleTime_sec,
                                const boolean bLaneChangeDetected,
                                boolean* pbStepDebounced);
void AdjacentLaneAliveCounter(const sALPAdjLaneInfo_st* pAdjLane,
                              const sALPParam_st* params,
                              sALPLaneInfo_t* pCalLaneInfo,
                              const float32 fCycleTime_sec,
                              boolean bLaneChangeDetected,
                              boolean bStepDebounced,
                              float32* puiCounterValue,
                              boolean* pbCounterValueValid);
void AdjacentLaneStabilizationWithEgoLaneData(
    const sALPEgoLaneInfo_st* pEgoLane,
    sALPAdjLaneInfo_st* pAdjLane,
    const sALPParam_st* params,
    float32 fLateralPosition,
    boolean bLateralPositionValid,
    eALPLaneIndex elaneIndex);
void DetermineAdjacentLaneBridgingState(const sALPAdjLaneInfo_st* pAdjLane,
                                        boolean bLateralPositionValid,
                                        boolean* pbAdjLaneBridging);

void DetermineLateralPositionAndValidity(const sALPEgoLaneInfo_st* pEgoLane,
                                         const sALPAdjLaneInfo_st* pAdjLane,
                                         sALPLaneInfo_t* pCalLaneInfo,
                                         boolean bStepDebounced,
                                         boolean bCounterValueValid,
                                         float32* pfLateralPosition,
                                         boolean* pbLateralPositionValid);
void StepDetectionOnBridging(const sALPEgoLaneInfo_st* pEgoLane,
                             const sALPAdjLaneInfo_st* pAdjLane,
                             const sALPParam_st* params,
                             sALPLaneInfo_t* pCalLaneInfo,
                             float32 fAdjLaneLatPos,
                             boolean bLaneChangeDetected,
                             boolean* pbAdjLaneLatPosValid,
                             boolean* pbStepDetectedOnBridging);
void LateralPositionFilter(const sALPParam_st* params,
                           sALPLaneInfo_t* pCalLaneInfo,
                           const float32 fCycleTime_sec,
                           float32 uiCounterValue,
                           boolean bLaneChangeDetected,
                           float32* pfAdjLaneLatPos);

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ALP_LCF_ALP_MAIN_H_
