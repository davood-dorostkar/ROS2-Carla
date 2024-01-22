/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_LANEFILTERPROCESSING_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_LANEFILTERPROCESSING_H_

#include "tue_common_libs.h"
#include "LBP_Ext.h"

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LaneFilterReset(void);

static void reset_crvKF(TUE_CML_sMatrix_t *x_crvKF, TUE_CML_sMatrix_t *P_crvKF);
static void predict_crvKF(REAL32_T dX_crvKF,
                          TUE_CML_sMatrix_t *x_crvKF,
                          TUE_CML_sMatrix_t *P_crvKF,
                          REAL32_T P_ABPLBP_CrvKFDefCurve_1pm,
                          REAL32_T P_ABPLBP_CrvKFQ11Fac_nu,
                          REAL32_T P_ABPLBP_CrvKFQ11FacStraight_nu,
                          REAL32_T curvature_1pm);
static void init_crvKF(const TUE_CML_sMatrix_t *z_crvKF,
                       TUE_CML_sMatrix_t *x_crvKF,
                       TUE_CML_sMatrix_t *P_crvKF,
                       TUE_CML_sMatrix_t *R_crvKF,
                       REAL32_T quality,
                       REAL32_T P_ABPLBP_CrvKFInitRFactor_nu,
                       REAL32_T P_ABPLBP_CrvKFMnInitQual_perc);
static void update_crvKF(const TUE_CML_sMatrix_t *z_crvKF,
                         const TUE_CML_sMatrix_t *R_crvKF,
                         REAL32_T weight,
                         TUE_CML_sMatrix_t *x_crvKF,
                         TUE_CML_sMatrix_t *P_crvKF);
static void maintenance_crvKF(REAL32_T deltaT_sec,
                              REAL32_T P_ABPLBP_CrvKFIncQual_1ps,
                              REAL32_T P_ABPLBP_CrvKFDecQualDeg_1ps,
                              REAL32_T P_ABPLBP_CrvKFDecQualPred_1ps,
                              TUE_CML_sMatrix_t *x_crvKF,
                              TUE_CML_sMatrix_t *P_crvKF);
void crvKalmanFilter(const crvKFInTypeV2 *inputs, crvKFOutType *outputs);
static void reset_laneKFLe(TUE_CML_sMatrix_t *x_laneKFLe,
                           TUE_CML_sMatrix_t *P_laneKFLe);
static void predict_laneKFLe(REAL32_T dT_laneKFLe,
                             REAL32_T dX_laneKFLe,
                             REAL32_T vehYawRate,
                             TUE_CML_sMatrix_t *x_laneKFLe,
                             TUE_CML_sMatrix_t *P_laneKFLe,
                             REAL32_T P_ABPLBP_LaneKFDynDistYFact_nu,
                             REAL32_T P_ABPLBP_LaneKFDynYawFactor_nu);
static void init_laneKFLe(const TUE_CML_sMatrix_t *z_laneKFLe,
                          TUE_CML_sMatrix_t *R_laneKFLe,
                          REAL32_T quality,
                          TUE_CML_sMatrix_t *x_laneKFLe,
                          TUE_CML_sMatrix_t *P_laneKFLe,
                          UINT8_T P_ABPLBP_LaneKFMnInitQual_perc,
                          REAL32_T P_ABPLBP_LaneKFInitRFactor_nu);
static void update_laneKFLe(const TUE_CML_sMatrix_t *z_laneKFLe,
                            const TUE_CML_sMatrix_t *R_laneKFLe,
                            REAL32_T weight,
                            TUE_CML_sMatrix_t *x_laneKFLe,
                            TUE_CML_sMatrix_t *P_laneKFLe,
                            REAL32_T P_ABPLBP_LaneKFKGainFac_nu);
static void maintenance_laneKFLe(TUE_CML_sMatrix_t *x_laneKFLe,
                                 TUE_CML_sMatrix_t *P_laneKFLe,
                                 REAL32_T P_ABPLBP_LaneKFIncQual_1ps,
                                 REAL32_T P_ABPLBP_LaneKFDecQualDeg_1ps,
                                 REAL32_T P_ABPLBP_LaneKFDecQualPred_1ps,
                                 REAL32_T deltaT_sec);
void laneKalmanFilter_Left(const laneKFInTypeV3 *inputs,
                           laneKFOutType *outputs);
static void reset_laneKFCntr(TUE_CML_sMatrix_t *x_laneKFCntr,
                             TUE_CML_sMatrix_t *P_laneKFCntr);
static void predict_laneKFCntr(REAL32_T dT_laneKFCntr,
                               REAL32_T dX_laneKFCntr,
                               REAL32_T vehYawRate,
                               TUE_CML_sMatrix_t *x_laneKFCntr,
                               TUE_CML_sMatrix_t *P_laneKFCntr,
                               REAL32_T P_ABPLBP_LaneKFDynDistYFact_nu,
                               REAL32_T P_ABPLBP_LaneKFDynYawFactor_nu,
                               REAL32_T P_ABPLBP_LaneKFDynCrvFactor_nu,
                               REAL32_T P_ABPLBP_LaneKFDynCrvRtFactor_nu);
static void init_laneKFCntr(const TUE_CML_sMatrix_t *z_laneKFCntr,
                            TUE_CML_sMatrix_t *R_laneKFCntr,
                            REAL32_T quality,
                            TUE_CML_sMatrix_t *x_laneKFCntr,
                            TUE_CML_sMatrix_t *P_laneKFCntr,
                            UINT8_T P_ABPLBP_LaneKFMnInitQual_perc,
                            REAL32_T P_ABPLBP_LaneKFInitRFactor_nu);
static void update_laneKFCntr(const TUE_CML_sMatrix_t *z_laneKFCntr,
                              const TUE_CML_sMatrix_t *R_laneKFCntr,
                              REAL32_T weight,
                              TUE_CML_sMatrix_t *x_laneKFCntr,
                              TUE_CML_sMatrix_t *P_laneKFCntr,
                              REAL32_T P_ABPLBP_LaneKFKGainFac_nu);
static void maintenance_laneKFCntr(TUE_CML_sMatrix_t *x_laneKFCntr,
                                   TUE_CML_sMatrix_t *P_laneKFCntr,
                                   REAL32_T P_ABPLBP_LaneKFIncQual_1ps,
                                   REAL32_T P_ABPLBP_LaneKFDecQualDeg_1ps,
                                   REAL32_T P_ABPLBP_LaneKFDecQualPred_1ps,
                                   REAL32_T deltaT_sec);
void LFP_DetermineTargetCorridor(const sKLMInput_t *pKLMInput,
                                 const sKLMParam_t *pKLMParam,
                                 sKLMDebug_t *pKLMDebug,
                                 REAL32_T fLastPosY0KlmCntr_klm,
                                 REAL32_T fLastHeadingKlmCntr_klm,
                                 REAL32_T fLastCrvKlmCntr_klm,
                                 REAL32_T fLastCrvRateKlmCntr_klm,
                                 UINT8_T bLastValidLaneCntr,
                                 UINT8_T *bValidLaneCntr,
                                 REAL32_T *fPosY0Cntr,
                                 REAL32_T *fHeadingCntr,
                                 REAL32_T *fCrvCntr,
                                 REAL32_T *fCrvRateCntr,
                                 REAL32_T *fOverallQualityCntr,
                                 UINT8_T *bEnaDegrUpdateCntr);
void LFP_CenterLaneKalmanFilter(const sKLMInput_t *pKLMInput,
                                const sKLMParam_t *pKLMParam,
                                sKLMOutput_t *pKLMOutput,
                                sKLMDebug_t *pKLMDebug);
void laneKalmanFilter_Center(const laneKFInTypeV3 *inputs,
                             laneKFOutType *outputs);
static void reset_laneKFRi(TUE_CML_sMatrix_t *x_laneKFRi,
                           TUE_CML_sMatrix_t *P_laneKFRi);
static void predict_laneKFRi(REAL32_T dT_laneKFRi,
                             REAL32_T dx_laneKFRi,
                             REAL32_T vehYawRate,
                             TUE_CML_sMatrix_t *x_laneKFRi,
                             TUE_CML_sMatrix_t *P_laneKFRi,
                             REAL32_T P_ABPLBP_LaneKFDynDistYFact_nu,
                             REAL32_T P_ABPLBP_LaneKFDynYawFactor_nu);
static void init_laneKFRi(const TUE_CML_sMatrix_t *z_laneKFRi,
                          TUE_CML_sMatrix_t *R_laneKFRi,
                          REAL32_T quality,
                          TUE_CML_sMatrix_t *x_laneKFRi,
                          TUE_CML_sMatrix_t *P_laneKFRi,
                          UINT8_T P_ABPLBP_LaneKFMnInitQual_perc,
                          REAL32_T P_ABPLBP_LaneKFInitRFactor_nu);
static void update_laneKFRi(const TUE_CML_sMatrix_t *z_laneKFRi,
                            const TUE_CML_sMatrix_t *R_laneKFRi,
                            REAL32_T weight,
                            TUE_CML_sMatrix_t *x_laneKFRi,
                            TUE_CML_sMatrix_t *P_laneKFRi,
                            REAL32_T P_ABPLBP_LaneKFKGainFac_nu);
static void maintenance_laneKFRi(TUE_CML_sMatrix_t *x_laneKFRi,
                                 TUE_CML_sMatrix_t *P_laneKFRi,
                                 REAL32_T P_ABPLBP_LaneKFIncQual_1ps,
                                 REAL32_T P_ABPLBP_LaneKFDecQualDeg_1ps,
                                 REAL32_T P_ABPLBP_LaneKFDecQualPred_1ps,
                                 REAL32_T deltaT_sec);
void laneKalmanFilter_Right(const laneKFInTypeV3 *inputs,
                            laneKFOutType *outputs);
void INPUT_LaneWidthValidity(const sLFPInput_t *pLFPInput,
                             const sLFPParam_t *pLFPParam,
                             const sCFVOutput_t *pCFVOutput,
                             sLWVInput_t *pLWVInput,
                             sLWVParam_t *PLWVParam);

void LWV_CheckLaneValidity(sLWVInput_t const *pLWVInput,
                           sLWVParam_t const *pLWVParam,
                           sLWVDebug_t *pLWVDebug,
                           UINT8_T *bLaneValidLf,
                           UINT8_T *bLaneValidRi);

void LWV_CheckLaneBridging(sLWVInput_t const *pLWVInput,
                           sLWVParam_t const *pLWVParam,
                           UINT8_T bLaneValidLf,
                           UINT8_T bLastLaneValidLf,
                           UINT8_T bLastLaneBridgeLf,
                           UINT8_T bLaneValidRi,
                           UINT8_T bLastLaneValidRi,
                           UINT8_T bLastLaneBridgeRi,
                           sLWVDebug_t *pLWVDebug,
                           UINT8_T *bLaneBridgeLf,
                           UINT8_T *bLaneBridgeRi);

void LaneWidthValidity(sLWVInput_t const *pLWVInput,
                       sLWVParam_t const *pLWVParam,
                       sLWVOutput_t *pLWVOutput,
                       sLWVDebug_t *pLWVDebug);
void INPUT_LaneKalmanFilters(const sLFPInput_t *pLFPInput,
                             const sLFPParam_t *pLFPParam,
                             const sLWVOutput_t *pLWVOutput,
                             const sCFVOutput_t *pCFVOutput,
                             sKLMInput_t *pKLMInput,
                             sKLMParam_t *pKLMParam);
void LaneKalmanFilters(const sKLMInput_t *pKLMInput,
                       const sKLMParam_t *pKLMParam,
                       sKLMOutput_t *pKLMOutput,
                       sKLMDebug_t *pKLMDebug);
void INPUT_CheckFilterValidity(const sLFPInput_t *pLFPInput,
                               const sLFPParam_t *pLFPParam,
                               const sLWVOutput_t *pLWVOutput,
                               const sKLMOutput_t *pKLMOutput,
                               sCFVInput_t *pCFVInput,
                               sCFVParam_t *pCFVParam);

void CFV_LeftKalmanValidation(const sCFVInput_t *pCFVInput,
                              const sCFVParam_t *pCFVParam,
                              sCFVOutput_t *pCFVOutput,
                              sCFVDebug_t *pCFVDebug);

void CFV_CenterKalmanValidation(const sCFVInput_t *pCFVInput,
                                const sCFVParam_t *pCFVParam,
                                sCFVOutput_t *pCFVOutput,
                                sCFVDebug_t *pCFVDebug);

void CheckFilterValidity(const sCFVInput_t *pCFVInput,
                         const sCFVParam_t *pCFVParam,
                         sCFVOutput_t *pCFVOutput,
                         sCFVDebug_t *pCFVDebug);
void OUTPUT_LaneFilterProcessing(const sLWVOutput_t *pLWVOutput,
                                 const sKLMOutput_t *pKLMOutput,
                                 const sCFVOutput_t *pCFVOutput,
                                 sLFPOutput_t *pLFPOutput);
void INPUT_LaneFilterProcessing(const sLBPInput_t *pLBPInput,
                                const sLBPParam_t *pLBPParam,
                                const sULPOutput_t *pULPOutput,
                                const sCLPOutput_t *pCLPOutput,
                                sLFPInput_t *pLFPInput,
                                sLFPParam_t *pLFPParam);
void LaneFilterProcessing(sLFPInput_t const *pLFPInput,
                          sLFPParam_t const *pLFPParam,
                          sLFPOutput_t *pLFPOutput,
                          sLFPDebug_t *pLFPDebug);

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_SEN_ABPR_LBP_LANEFILTERPROCESSING_H_
