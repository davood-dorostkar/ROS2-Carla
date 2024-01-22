/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#include "LBP_LaneFilterProcessing.h"
#include "string.h"

/*****************************************************************************
  MACROS
*****************************************************************************/
#ifndef FALSE
#define FALSE (0u)
#endif
#ifndef TRUE
#define TRUE (1u)
#endif
/*length of the KF state vector*/
#define STATE_LENGTH_CRVKF (2u)

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h"

const volatile UINT8_T LBP_Ku_HZ_CAl_nu = 1u;
const volatile REAL32_T LBP_Kf_MaxBridgeDist_met = 180.f;
const volatile REAL32_T LBP_Kf_MaxBridgeTime_sec = 7.f;
const volatile REAL32_T LBP_fMaxDistYStep = 0.5F;
const volatile REAL32_T LBP_fMaxHeadingStep = 0.015F;
const volatile REAL32_T LBP_fMaxCrvStep = 7.5E-4F;
const volatile REAL32_T LBP_fMaxCrvRateStep = 1.0E-4F;
const volatile REAL32_T LBP_fStdDevPosY0 = 0.00001F;
const volatile REAL32_T LBP_fStdDevHeading = 0.00001F;
const volatile REAL32_T LBP_fStdDevCrv = 0.00001F;
const volatile REAL32_T LBP_fStdDevCrvRate = 0.00001F;

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_STOP_CODE
#include "Mem_Map.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/
/*state of the KF*/
static UINT8_T valid_crvKF = 0u;
/*status_crvKF of the KF: 0: invalid, 1: valid, full update, 2: valid, degraded
 * update, 3: valid, prediction, 4: valid, init, 5: invalid, reset*/
static UINT8_T status_crvKF = 0u;
/*measurement weight*/
static REAL32_T measWeight_crvKF = 0.0f;
/*internal quality*/
static REAL32_T internalQuality_crvKF = 0.0f;
/*variable for the geometric model error*/
static REAL32_T kappa2diff_sigma_crvKF = 0.0f;

/*****************************************************************************
 LOCAL VARIABLES
 *****************************************************************************/
/*state of the KF*/
static UINT8_T valid_laneKFLe = 0u;
/*status_laneKFLe of the KF: 0: invalid, 1: valid, full update, 2: valid,
 * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid, reset*/
static UINT8_T status_laneKFLe = 0u;
/*measurement weight*/
static REAL32_T measWeight_laneKFLe = 0.0f;
/*internal quality*/
static REAL32_T internalQuality_laneKFLe = 0.0f;
/*variable for the yaw rate standard deviation - TODO: really needed?*/
static REAL32_T vehYawRateStdDev_laneKFLe = 0.0f;
/*variable for the geometric model error*/
static REAL32_T kappa2diff_sigma_laneKFLe = 0.0f;

/*****************************************************************************
 LOCAL VARIABLES
 *****************************************************************************/
/*state of the KF*/
static UINT8_T valid_laneKFCntr = 0u;
/*status_laneKFCntr of the KF: 0: invalid, 1: valid, full update, 2: valid,
 * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid, reset*/
static UINT8_T status_laneKFCntr = 0u;
/*measurement weight*/
static REAL32_T measWeight_laneKFCntr = 0.0f;
/*internal quality*/
static REAL32_T internalQuality_laneKFCntr = 0.0f;
/*variable for the yaw rate standard deviation - TODO: really needed?*/
static REAL32_T vehYawRateStdDev_laneKFCntr = 0.0f;
/*variable for the geometric model error*/
static REAL32_T kappa2diff_sigma_laneKFCntr = 0.0f;

/*****************************************************************************
 LOCAL VARIABLES
 *****************************************************************************/
/*state of the KF*/
static UINT8_T valid_laneKFRi = 0u;
/*status_laneKFRi of the KF: 0: invalid, 1: valid, full update, 2: valid,
 * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid, reset*/
static UINT8_T status_laneKFRi = 0u;
/*measurement weight*/
static REAL32_T measWeight_laneKFRi = 0.0f;
/*internal quality*/
static REAL32_T internalQuality_laneKFRi = 0.0f;
/*variable for the yaw rate standard deviation - TODO: really needed?*/
static REAL32_T vehYawRateStdDev_laneKFRi = 0.0f;
/*variable for the geometric model error*/
static REAL32_T kappa2diff_sigma_laneKFRi = 0.0f;

// LWV_CheckLaneBridging
static REAL32_T fTimerBridgeVirtualLf = 0.0F;
static REAL32_T fTimerBridgeVirtualRi = 0.0F;
static UINT8_T bLastBridgeByVirtualLf = 0U;
static UINT8_T bLastBridgeByVirtualRi = 0U;
static UINT8_T bLastRawLaneBridgeLf = 0U;
static UINT8_T bLastRawLaneBridgeRi = 0U;
static REAL32_T fLastBridgeDistanceLf = 0.0F;
static REAL32_T fLastBridgeDistanceRi = 0.0F;
static REAL32_T fLastBridgeTimeLf = 0.0F;
static REAL32_T fLastBridgeTimeRi = 0.0F;
static UINT8_T bLastEnableByDistanceLf = 0U;
static UINT8_T bLastEnableByDistanceRi = 0U;
// LaneWidthValidity
static UINT8_T bLastLaneValidLf = 0U;
static UINT8_T bLastLaneValidRi = 0U;
static UINT8_T bLastLaneBridgeLf = 0U;
static UINT8_T bLastLaneBridgeRi = 0U;
static REAL32_T fLastLaneWidth = 2.5f;
static UINT8_T bLastLowPassReset = 0U;
static REAL32_T fLastTimeLowPass = 0.0F;
static UINT8_T bLastLaneWidthValid = 0U;
// LFP_DetermineTargetCorridor
static REAL32_T fLastRawPosY0Cntr = 0.0F;
static REAL32_T fLastPosY0UlpRi =
    0.0F; /* Last right lane Y0 position by Ulp, (0, -15~15, m) */
static REAL32_T fLastPosY0UlpLf =
    0.0F; /* Last left lane Y0 position by Ulp, (0, -15~15, m) */
static REAL32_T fLastPosY0Cntr =
    0.0F; /* Center lane Y0 position by Ulp, (0, -15~15, m) */
static REAL32_T fLastHeadingCntr =
    0.0F; /* Center lane heading angle by Ulp, (0, -0.7854~0.7854, rad) */
static REAL32_T fLastCrvCntr =
    0.0F; /* Center lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
static REAL32_T fLastCrvRateCntr =
    0.0F; /* Center lane curvature rate by Ulp, (0, -0.001~0.001, 1/m^2) */
static UINT8_T bLastEnaByEdgeFail = 0U;
static UINT8_T bLastEnaBySRTrig = 0U;

// LFP_CenterLaneKalmanFilter
static REAL32_T fLastPosY0KlmCntr_klm = 0.0F;
static REAL32_T fLastHeadingKlmCntr_klm = 0.0F;
static REAL32_T fLastCrvKlmCntr_klm = 0.0F;
static REAL32_T fLastCrvRateKlmCntr_klm = 0.0F;
static UINT8_T bLastValidLaneCntr =
    0U; /* Left ego lane validity, (0, 0~1, -) */
static REAL32_T fLastDegrWeightCntr = 0.0F;
static UINT8_T bLastCrvLowPassReset = 0U;
static REAL32_T fLastEstCrvKlmCntr = 0.0F;
// LaneKalmanFilters
static REAL32_T fLastEstCrvKlmLf = 0.0F;
static REAL32_T fLastEstCrvKlmRi = 0.0F;
// CFV_LeftKalmanValidation
static REAL32_T fLastPosY0FltLf =
    0.0F; /* Last left lane Y0 position by Kalman filter (0, -15~15, m) */
static REAL32_T fLastHeadingFltLf =
    0.0F; /* Last left lane heading angle by Kalman filter, (0,
             -0.7854~0.7854, rad) */
static REAL32_T fLastCrvFltLf =
    0.0F; /* Last left lane curvature by Kalman filter, (0, -0.1~0.1, 1/m)
           */
static REAL32_T fLastCrvRateFltLf =
    0.0F; /* Last left lane curvature change by Kalman filter, (0,
             -0.001~0.001, 1/m^2) */
static UINT8_T bLastDlyValidDistYStepLf =
    0U; /* Last left lane lateral distance step validity after turn on
           delay, (0, 0~1, -)*/
static UINT8_T bLastInValidDistYStepLf =
    0U; /* Last Left lane lateral distance step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidHeadingStepLf =
    0U; /* Last left lane heading angle step validity after turn on delay,
           (0, 0~1, -)*/
static UINT8_T bLastInValidHeadingStepLf =
    0U; /* Last Left lane heading angle step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidCrvStepLf =
    0U; /* Last left lane curvature step validity after turn on delay, (0,
           0~1, -)*/
static UINT8_T bLastInValidCrvStepLf =
    0U; /* Last Left lane curvature step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidCrvRateStepLf = 0U; /* Last left lane curvature rate
                                                   step validity after turn on
                                                   delay, (0, 0~1, -)*/
static UINT8_T bLastInValidCrvRateStepLf =
    0U; /* Last Left lane curvature rate step invalidity, (0, 0~1, -)*/
static REAL32_T fTimerDistYStepValidLf =
    0.0F; /* Timer for lateral positions validity, (1, 0~60, s) */
static REAL32_T fTimerHeadingStepValidLf =
    0.0F; /* Timer for heading angle validity, (1, 0~60, s) */
static REAL32_T fTimerCrvStepValidLf =
    0.0F; /* Timer for curvature validity, (1, 0~60, s) */
static REAL32_T fTimerCrvRateStepValidLf =
    0.0F; /* Timer for curvature rate validity, (1, 0~60, s) */

// CFV_CenterKalmanValidation
static REAL32_T fLastPosY0FltCntr =
    0.0F; /* Last Center lane Y0 position by Kalman filter (0, -15~15, m) */
static REAL32_T fLastHeadingFltCntr = 0.0F; /* Last Center lane heading
                                               angle by Kalman filter, (0,
                                               -0.7854~0.7854, rad) */
static REAL32_T fLastCrvFltCntr = 0.0F; /* Last Center lane curvature by Kalman
                                           filter, (0, -0.1~0.1, 1/m) */
static REAL32_T fLastCrvRateFltCntr =
    0.0F; /* Last Center lane curvature change by Kalman filter, (0,
             -0.001~0.001, 1/m^2) */
static UINT8_T bLastDlyValidDistYStepCntr =
    0U; /* Last center lane lateral distance step validity after turn on
           delay, (0, 0~1, -)*/
static UINT8_T bLastInValidDistYStepCntr =
    0U; /* Last center lane lateral distance step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidHeadingStepCntr =
    0U; /* Last center lane heading angle step validity after turn on
           delay, (0, 0~1, -)*/
static UINT8_T bLastInValidHeadingStepCntr =
    0U; /* Last center lane heading angle step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidCrvStepCntr =
    0U; /* Last center lane curvature step validity after turn on delay, (0,
           0~1, -)*/
static UINT8_T bLastInValidCrvStepCntr =
    0U; /* Last center lane curvature step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidCrvRateStepCntr =
    0U; /* Last center lane curvature rate step validity after turn on
           delay, (0, 0~1, -)*/
static UINT8_T bLastInValidCrvRateStepCntr =
    0U; /* Last center lane curvature rate step invalidity, (0, 0~1, -)*/
static REAL32_T fTimerDistYStepValidCntr =
    0.0F; /* Timer for lateral positions validity, (1, 0~60, s) */
static REAL32_T fTimerHeadingStepValidCntr =
    0.0F; /* Timer for heading angle validity, (1, 0~60, s) */
static REAL32_T fTimerCrvStepValidCntr =
    0.0F; /* Timer for curvature validity, (1, 0~60, s) */
static REAL32_T fTimerCrvRateStepValidCntr =
    0.0F; /* Timer for curvature rate validity, (1, 0~60, s) */
// CheckFilterValidity
static REAL32_T fLastPosY0FltRi =
    0.0F; /* Last right lane Y0 position by Kalman filter (0, -15~15, m) */
static REAL32_T fLastHeadingFltRi = 0.0F; /* Last right lane heading angle
                                             by Kalman filter, (0,
                                             -0.7854~0.7854, rad) */
static REAL32_T fLastCrvFltRi = 0.0F;     /* Last right lane curvature by Kalman
                                             filter, (0, -0.1~0.1, 1/m) */
static REAL32_T fLastCrvRateFltRi = 0.0F; /* Last right lane curvature
                                             change by Kalman filter, (0,
                                             -0.001~0.001, 1/m^2) */
static UINT8_T bLastDlyValidDistYStepRi =
    0U; /* Last right lane lateral distance step validity after turn on
           delay, (0, 0~1, -)*/
static UINT8_T bLastInValidDistYStepRi =
    0U; /* Last right lane lateral distance step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidHeadingStepRi = 0U; /* Last right lane heading angle
                                                   step validity after turn on
                                                   delay, (0, 0~1, -)*/
static UINT8_T bLastInValidHeadingStepRi =
    0U; /* Last right lane heading angle step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidCrvStepRi = 0U; /* Last right lane curvature
                                               step validity after turn on
                                               delay, (0, 0~1, -)*/
static UINT8_T bLastInValidCrvStepRi =
    0U; /* Last right lane curvature step invalidity, (0, 0~1, -)*/
static UINT8_T bLastDlyValidCrvRateStepRi =
    0U; /* Last right lane curvature rate step validity after turn on
           delay, (0, 0~1, -)*/
static UINT8_T bLastInValidCrvRateStepRi =
    0U; /* Last right lane curvature rate step invalidity, (0, 0~1, -)*/

static REAL32_T fTimerDistYStepValidRi =
    0.0F; /* Timer for lateral positions validity, (1, 0~60, s) */
static REAL32_T fTimerHeadingStepValidRi =
    0.0F; /* Timer for heading angle validity, (1, 0~60, s) */
static REAL32_T fTimerCrvStepValidRi =
    0.0F; /* Timer for curvature validity, (1, 0~60, s) */
static REAL32_T fTimerCrvRateStepValidRi =
    0.0F; /* Timer for curvature rate validity, (1, 0~60, s) */
static sCFVOutput_t sCFVOutput = {0};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

void LaneFilterReset(void) {
    memset(&sCFVOutput, 0, sizeof(sCFVOutput_t));
    /*state of the KF*/
    valid_crvKF = 0u;
    /*status_crvKF of the KF: 0: invalid, 1: valid, full update, 2: valid,
     * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid,
     * reset*/
    status_crvKF = 0u;
    /*measurement weight*/
    measWeight_crvKF = 0.0f;
    /*internal quality*/
    internalQuality_crvKF = 0.0f;
    /*variable for the geometric model error*/
    kappa2diff_sigma_crvKF = 0.0f;
    /*state of the KF*/
    valid_laneKFLe = 0u;
    /*status_laneKFLe of the KF: 0: invalid, 1: valid, full update, 2: valid,
     * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid,
     * reset*/
    status_laneKFLe = 0u;
    /*measurement weight*/
    measWeight_laneKFLe = 0.0f;
    /*internal quality*/
    internalQuality_laneKFLe = 0.0f;
    /*variable for the yaw rate standard deviation - TODO: really needed?*/
    vehYawRateStdDev_laneKFLe = 0.0f;
    /*variable for the geometric model error*/
    kappa2diff_sigma_laneKFLe = 0.0f;
    /*state of the KF*/
    valid_laneKFCntr = 0u;
    /*status_laneKFCntr of the KF: 0: invalid, 1: valid, full update, 2: valid,
     * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid,
     * reset*/
    status_laneKFCntr = 0u;
    /*measurement weight*/
    measWeight_laneKFCntr = 0.0f;
    /*internal quality*/
    internalQuality_laneKFCntr = 0.0f;
    /*variable for the yaw rate standard deviation - TODO: really needed?*/
    vehYawRateStdDev_laneKFCntr = 0.0f;
    /*variable for the geometric model error*/
    kappa2diff_sigma_laneKFCntr = 0.0f;
    /*state of the KF*/
    valid_laneKFRi = 0u;
    /*status_laneKFRi of the KF: 0: invalid, 1: valid, full update, 2: valid,
     * degraded update, 3: valid, prediction, 4: valid, init, 5: invalid,
     * reset*/
    status_laneKFRi = 0u;
    /*measurement weight*/
    measWeight_laneKFRi = 0.0f;
    /*internal quality*/
    internalQuality_laneKFRi = 0.0f;
    /*variable for the yaw rate standard deviation - TODO: really needed?*/
    vehYawRateStdDev_laneKFRi = 0.0f;
    /*variable for the geometric model error*/
    kappa2diff_sigma_laneKFRi = 0.0f;
    // LWV_CheckLaneBridging
    fTimerBridgeVirtualLf = 0.0F;
    fTimerBridgeVirtualRi = 0.0F;
    bLastBridgeByVirtualLf = 0U;
    bLastBridgeByVirtualRi = 0U;
    bLastRawLaneBridgeLf = 0U;
    bLastRawLaneBridgeRi = 0U;
    fLastBridgeDistanceLf = 0.0F;
    fLastBridgeDistanceRi = 0.0F;
    fLastBridgeTimeLf = 0.0F;
    fLastBridgeTimeRi = 0.0F;
    bLastEnableByDistanceLf = 0U;
    bLastEnableByDistanceRi = 0U;
    // LaneWidthValidity
    bLastLaneValidLf = 0U;
    bLastLaneValidRi = 0U;
    bLastLaneBridgeLf = 0U;
    bLastLaneBridgeRi = 0U;
    fLastLaneWidth = 2.5f;
    bLastLowPassReset = 0U;
    fLastTimeLowPass = 0.0F;
    bLastLaneWidthValid = 0U;
    // LFP_DetermineTargetCorridor
    fLastRawPosY0Cntr = 0.0F;
    fLastPosY0UlpRi =
        0.0F; /* Last right lane Y0 position by Ulp, (0, -15~15, m) */
    fLastPosY0UlpLf =
        0.0F; /* Last left lane Y0 position by Ulp, (0, -15~15, m) */
    fLastPosY0Cntr = 0.0F; /* Center lane Y0 position by Ulp, (0, -15~15, m) */
    fLastHeadingCntr =
        0.0F; /* Center lane heading angle by Ulp, (0, -0.7854~0.7854, rad) */
    fLastCrvCntr = 0.0F; /* Center lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    fLastCrvRateCntr =
        0.0F; /* Center lane curvature rate by Ulp, (0, -0.001~0.001, 1/m^2) */
    bLastEnaByEdgeFail = 0U;
    bLastEnaBySRTrig = 0U;

    // LFP_CenterLaneKalmanFilter
    fLastPosY0KlmCntr_klm = 0.0F;
    fLastHeadingKlmCntr_klm = 0.0F;
    fLastCrvKlmCntr_klm = 0.0F;
    fLastCrvRateKlmCntr_klm = 0.0F;
    bLastValidLaneCntr = 0U; /* Left ego lane validity, (0, 0~1, -) */
    fLastDegrWeightCntr = 0.0F;
    bLastCrvLowPassReset = 0U;
    fLastEstCrvKlmCntr = 0.0F;
    // LaneKalmanFilters
    fLastEstCrvKlmLf = 0.0F;
    fLastEstCrvKlmRi = 0.0F;
    // CFV_LeftKalmanValidation
    fLastPosY0FltLf =
        0.0F; /* Last left lane Y0 position by Kalman filter (0, -15~15, m) */
    fLastHeadingFltLf = 0.0F; /* Last left lane heading angle by Kalman filter,
                                 (0, -0.7854~0.7854, rad) */
    fLastCrvFltLf =
        0.0F; /* Last left lane curvature by Kalman filter, (0, -0.1~0.1, 1/m)
               */
    fLastCrvRateFltLf = 0.0F; /* Last left lane curvature change by Kalman
                                 filter, (0, -0.001~0.001, 1/m^2) */
    bLastDlyValidDistYStepLf =
        0U; /* Last left lane lateral distance step validity after turn on
               delay, (0, 0~1, -)*/
    bLastInValidDistYStepLf =
        0U; /* Last Left lane lateral distance step invalidity, (0, 0~1, -)*/
    bLastDlyValidHeadingStepLf =
        0U; /* Last left lane heading angle step validity after turn on delay,
               (0, 0~1, -)*/
    bLastInValidHeadingStepLf =
        0U; /* Last Left lane heading angle step invalidity, (0, 0~1, -)*/
    bLastDlyValidCrvStepLf = 0U; /* Last left lane curvature step validity after
                                    turn on delay, (0, 0~1, -)*/
    bLastInValidCrvStepLf =
        0U; /* Last Left lane curvature step invalidity, (0, 0~1, -)*/
    bLastDlyValidCrvRateStepLf = 0U; /* Last left lane curvature rate step
                                        validity after turn on delay, (0,
                                        0~1, -)*/
    bLastInValidCrvRateStepLf =
        0U; /* Last Left lane curvature rate step invalidity, (0, 0~1, -)*/
    fTimerDistYStepValidLf =
        0.0F; /* Timer for lateral positions validity, (1, 0~60, s) */
    fTimerHeadingStepValidLf =
        0.0F; /* Timer for heading angle validity, (1, 0~60, s) */
    fTimerCrvStepValidLf =
        0.0F; /* Timer for curvature validity, (1, 0~60, s) */
    fTimerCrvRateStepValidLf =
        0.0F; /* Timer for curvature rate validity, (1, 0~60, s) */

    // CFV_CenterKalmanValidation
    fLastPosY0FltCntr =
        0.0F; /* Last Center lane Y0 position by Kalman filter (0, -15~15, m) */
    fLastHeadingFltCntr = 0.0F; /* Last Center lane heading
                                                 angle by Kalman filter, (0,
                                                 -0.7854~0.7854, rad) */
    fLastCrvFltCntr = 0.0F;     /* Last Center lane curvature by Kalman filter,
                                   (0, -0.1~0.1, 1/m) */
    fLastCrvRateFltCntr = 0.0F; /* Last Center lane curvature change by Kalman
                                   filter, (0, -0.001~0.001, 1/m^2) */
    bLastDlyValidDistYStepCntr =
        0U; /* Last center lane lateral distance step validity after turn on
               delay, (0, 0~1, -)*/
    bLastInValidDistYStepCntr =
        0U; /* Last center lane lateral distance step invalidity, (0, 0~1, -)*/
    bLastDlyValidHeadingStepCntr =
        0U; /* Last center lane heading angle step validity after turn on
               delay, (0, 0~1, -)*/
    bLastInValidHeadingStepCntr =
        0U; /* Last center lane heading angle step invalidity, (0, 0~1, -)*/
    bLastDlyValidCrvStepCntr = 0U; /* Last center lane curvature step validity
                                      after turn on delay, (0, 0~1, -)*/
    bLastInValidCrvStepCntr =
        0U; /* Last center lane curvature step invalidity, (0, 0~1, -)*/
    bLastDlyValidCrvRateStepCntr =
        0U; /* Last center lane curvature rate step validity after turn on
               delay, (0, 0~1, -)*/
    bLastInValidCrvRateStepCntr =
        0U; /* Last center lane curvature rate step invalidity, (0, 0~1, -)*/
    fTimerDistYStepValidCntr =
        0.0F; /* Timer for lateral positions validity, (1, 0~60, s) */
    fTimerHeadingStepValidCntr =
        0.0F; /* Timer for heading angle validity, (1, 0~60, s) */
    fTimerCrvStepValidCntr =
        0.0F; /* Timer for curvature validity, (1, 0~60, s) */
    fTimerCrvRateStepValidCntr =
        0.0F; /* Timer for curvature rate validity, (1, 0~60, s) */
              // CheckFilterValidity
    fLastPosY0FltRi =
        0.0F; /* Last right lane Y0 position by Kalman filter (0, -15~15, m) */
    fLastHeadingFltRi = 0.0F; /* Last right lane heading angle
                                               by Kalman filter, (0,
                                               -0.7854~0.7854, rad) */
    fLastCrvFltRi = 0.0F;     /* Last right lane curvature by Kalman
                                               filter, (0, -0.1~0.1, 1/m) */
    fLastCrvRateFltRi = 0.0F; /* Last right lane curvature
                                               change by Kalman filter, (0,
                                               -0.001~0.001, 1/m^2) */
    bLastDlyValidDistYStepRi =
        0U; /* Last right lane lateral distance step validity after turn on
               delay, (0, 0~1, -)*/
    bLastInValidDistYStepRi =
        0U; /* Last right lane lateral distance step invalidity, (0, 0~1, -)*/
    bLastDlyValidHeadingStepRi = 0U; /* Last right lane heading angle step
                                        validity after turn on delay, (0,
                                        0~1, -)*/
    bLastInValidHeadingStepRi =
        0U; /* Last right lane heading angle step invalidity, (0, 0~1, -)*/
    bLastDlyValidCrvStepRi = 0U; /* Last right lane curvature
                                                 step validity after turn on
                                                 delay, (0, 0~1, -)*/
    bLastInValidCrvStepRi =
        0U; /* Last right lane curvature step invalidity, (0, 0~1, -)*/
    bLastDlyValidCrvRateStepRi =
        0U; /* Last right lane curvature rate step validity after turn on
               delay, (0, 0~1, -)*/
    bLastInValidCrvRateStepRi =
        0U; /* Last right lane curvature rate step invalidity, (0, 0~1, -)*/

    fTimerDistYStepValidRi =
        0.0F; /* Timer for lateral positions validity, (1, 0~60, s) */
    fTimerHeadingStepValidRi =
        0.0F; /* Timer for heading angle validity, (1, 0~60, s) */
    fTimerCrvStepValidRi =
        0.0F; /* Timer for curvature validity, (1, 0~60, s) */
    fTimerCrvRateStepValidRi =
        0.0F; /* Timer for curvature rate validity, (1, 0~60, s) */
}

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/********************************crvKalmanFilter*************************************************
        @fn            crvKalmanFilter
        @brief         Kalman filtering of lane curvature
        @description   According to the third order spiral curve, Kalman filter
is applied to the
                                   curvature of lane centerline
        @param[in]     inputs  : Curvature, curvature rate, curvature standard
deviation,
                                                         curvature rate standard
deviation, longitudinal vehicle speed,etc
        @param[out]    outputs : Curvature, curvature rate, validity of Kalman
filter, lane quality
        @startuml
        start
        : Curvature filter inputs
        1.Center lane validity
        2.Lane quality data for target corridor
        3.Curvature and curvature rate;
        -> KalmanFilter;
        : Curvature filter outputs
        1. Curvature and curvature rate
        2. Validity of Kalman filter
        3. Lane quality;
        end
        @enduml
**************************************************************************************************/
void crvKalmanFilter(const crvKFInTypeV2 *inputs, crvKFOutType *outputs) {
    /*state Matrix x_crvKF*/
    TUE_CML_CreateMatrix_M(x_crvKF, STATE_LENGTH_CRVKF, 1)
        /*covariance matrix P_crvKF*/
        TUE_CML_CreateMatrix_M(P_crvKF, STATE_LENGTH_CRVKF, STATE_LENGTH_CRVKF)
        /*measurement matrix z_crvKF*/
        TUE_CML_CreateMatrix_M(z_crvKF, STATE_LENGTH_CRVKF, 1)
        /*measurement variance matrix R_crvKF*/
        TUE_CML_CreateMatrix_M(R_crvKF, STATE_LENGTH_CRVKF, STATE_LENGTH_CRVKF)

            REAL32_T dX_crvKF;

    /*calculate dX*/
    dX_crvKF = inputs->sf_DeltaT_sec * inputs->sf_VehVelX_mps;

    /*geometric dependent model error*/
    kappa2diff_sigma_crvKF =
        1 / inputs->sf_CrvKFErrCoeff1_nu /
        (inputs->sf_CrvKFErrCoeff2_nu * inputs->sf_DeltaT_sec) /
        (inputs->sf_CrvKFErrCoeff2_nu * inputs->sf_DeltaT_sec);

    /*predict -  will only be executed if valid state is set*/
    predict_crvKF(dX_crvKF, x_crvKF, P_crvKF, inputs->sf_CrvKFDefCurve_1pm,
                  inputs->sf_CrvKFQ11Fac_nu, inputs->sf_CrvKFQ11FacStraight_nu,
                  inputs->sf_Crv_1pm);

    /*initialize measurement vector z*/
    TUE_CML_GetMatrixElement_M(z_crvKF, 0, 0) = inputs->sf_Crv_1pm;
    TUE_CML_GetMatrixElement_M(z_crvKF, 1, 0) = inputs->sf_CrvChng_1pm2;

    /*initialize R matrix*/
    TUE_CML_GetMatrixElement_M(R_crvKF, 0, 0) =
        inputs->sf_CrvStdDev_1pm * inputs->sf_CrvStdDev_1pm;
    TUE_CML_GetMatrixElement_M(R_crvKF, 1, 1) =
        inputs->sf_CrvChngStdDev_1pm2 * inputs->sf_CrvChngStdDev_1pm2;

    /*Init or Update*/
    if (!valid_crvKF && !(inputs->sf_DegradedUpdate_bool)) {
        /*initialization - for a better initialization the R matrix is
         * multiplied with a constant (>1)*/
        init_crvKF(z_crvKF, x_crvKF, P_crvKF, R_crvKF,
                   inputs->sf_OverallMeasurementQuality_perc,
                   inputs->sf_CrvKFInitRFactor_nu,
                   inputs->sf_CrvKFMnInitQual_perc);
    }
    /*Update only if valid AND measurement quality > CrvKFLowQual_nu*/
    else if (valid_crvKF && (inputs->sf_OverallMeasurementQuality_perc >=
                             (inputs->sf_CrvKFMnUpdateQual_perc))) {
        if (inputs->sf_DegradedUpdate_bool) {
            /*degraded update - quality and weight are the 2 parameters*/
            update_crvKF(z_crvKF, R_crvKF, inputs->sf_CrvKFDegradeWeight_nu,
                         x_crvKF, P_crvKF);
        } else {
            /*full update*/
            update_crvKF(z_crvKF, R_crvKF, 1, x_crvKF, P_crvKF);
        }
    }
    /*maintenance_laneKFCntr*/
    if (!(status_crvKF == 4)) {
        maintenance_crvKF(inputs->sf_DeltaT_sec, inputs->sf_CrvKFIncQual_1ps,
                          inputs->sf_CrvKFDecQualDeg_1ps,
                          inputs->sf_CrvKFDecQualPred_1ps, x_crvKF, P_crvKF);
    }
    /*Reset filter if basic lane data are not valid*/
    if (!inputs->sf_crvDataValid_bool) reset_crvKF(x_crvKF, P_crvKF);

    /*Set outputs*/
    outputs->sf_Crv_1pm = TUE_CML_GetMatrixElement_M(x_crvKF, 0, 0);
    outputs->sf_CrvChng_1pm2 = TUE_CML_GetMatrixElement_M(x_crvKF, 1, 0);
    outputs->sf_KFStatus_btf = status_crvKF;
    outputs->sf_QualityMeasure_perc = (UINT8_T)(internalQuality_crvKF);
}

/********************************update_crvKF*************************************************
        @fn            update_crvKF
        @brief         Update Kalman filter for curvature filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        (*)-->UpdateCrvKf
         -->(*)
        @enduml
**************************************************************************************************/
static void update_crvKF(const TUE_CML_sMatrix_t *z_crvKF,
                         const TUE_CML_sMatrix_t *R_crvKF,
                         REAL32_T weight,
                         TUE_CML_sMatrix_t *x_crvKF,
                         TUE_CML_sMatrix_t *P_crvKF) {
    /*create local matrices*/
    TUE_CML_CreateMatrix_M(
        I_crvKF, STATE_LENGTH_CRVKF,
        STATE_LENGTH_CRVKF) TUE_CML_CreateMatrix_M(H_crvKF, STATE_LENGTH_CRVKF,
                                                   STATE_LENGTH_CRVKF)
        TUE_CML_CreateMatrix_M(
            H_trans_crvKF, STATE_LENGTH_CRVKF,
            STATE_LENGTH_CRVKF) TUE_CML_CreateMatrix_M(PH_trans_crvKF,
                                                       STATE_LENGTH_CRVKF,
                                                       STATE_LENGTH_CRVKF)
            TUE_CML_CreateMatrix_M(
                HPH_trans_crvKF, STATE_LENGTH_CRVKF,
                STATE_LENGTH_CRVKF) TUE_CML_CreateMatrix_M(HPH_transPlusR_crfKF,
                                                           STATE_LENGTH_CRVKF,
                                                           STATE_LENGTH_CRVKF)
                TUE_CML_CreateMatrix_M(HPH_transPlusR_inv_crvKF,
                                       STATE_LENGTH_CRVKF, STATE_LENGTH_CRVKF)
                    TUE_CML_CreateMatrix_M(K_crvKF, STATE_LENGTH_CRVKF,
                                           STATE_LENGTH_CRVKF)
                        TUE_CML_CreateMatrix_M(HX_crvKF, STATE_LENGTH_CRVKF, 1)
                            TUE_CML_CreateMatrix_M(ZSubHX_crvKF,
                                                   STATE_LENGTH_CRVKF, 1)
                                TUE_CML_CreateMatrix_M(KZSubHX_crvKF,
                                                       STATE_LENGTH_CRVKF, 1)
                                    TUE_CML_CreateMatrix_M(KH_crvKF,
                                                           STATE_LENGTH_CRVKF,
                                                           STATE_LENGTH_CRVKF)
                                        TUE_CML_CreateMatrix_M(
                                            ISubKH_crvKF, STATE_LENGTH_CRVKF,
                                            STATE_LENGTH_CRVKF)
                                            TUE_CML_CreateMatrix_M(
                                                P_tmp_crvKF, STATE_LENGTH_CRVKF,
                                                STATE_LENGTH_CRVKF)

        /*update state with measurement*/
        TUE_CML_CreateIdentityMatrix_M(I_crvKF, STATE_LENGTH_CRVKF);
    TUE_CML_CreateIdentityMatrix_M(H_crvKF, STATE_LENGTH_CRVKF);
    TUE_CML_TransposeMatrix_M(H_trans_crvKF, H_crvKF);
    /*PH'*/
    TUE_CML_MutiplyMatrices_M(PH_trans_crvKF, P_crvKF, H_trans_crvKF);
    /*H[PH']*/
    TUE_CML_MutiplyMatrices_M(HPH_trans_crvKF, H_crvKF, PH_trans_crvKF);
    /*[HPH']+R*/
    TUE_CML_AddMatrices_M(HPH_transPlusR_crfKF, HPH_trans_crvKF, R_crvKF);
    /*inv([HPH'+R]*/
    // CML_v_InvertMatrix(HPH_transPlusR_inv_crvKF, HPH_transPlusR_crfKF);
    TUE_CML_InvertMatrix_M(HPH_transPlusR_inv_crvKF, HPH_transPlusR_crfKF);
    if (HPH_transPlusR_inv_crvKF->Desc.col >
        0) { /*if inversion fails num. of cols and rows are set to 0*/
        /*K=[PH'][inv(HPH'+R)]*/
        TUE_CML_MutiplyMatrices_M(K_crvKF, PH_trans_crvKF,
                                  HPH_transPlusR_inv_crvKF);
        /*K=K*weight*/
        TUE_CML_ScaleMatrix_M(K_crvKF, weight);
        /*Hx*/
        TUE_CML_MutiplyMatrices_M(HX_crvKF, H_crvKF, x_crvKF);
        /*z-[Hx]*/
        TUE_CML_SubtractMatrices_M(ZSubHX_crvKF, z_crvKF, HX_crvKF);
        /*K[(z-Hx)]*/
        TUE_CML_MutiplyMatrices_M(KZSubHX_crvKF, K_crvKF, ZSubHX_crvKF);
        /*x=x+K(z-Hx)*/
        TUE_CML_AddMatrices_M(x_crvKF, x_crvKF, KZSubHX_crvKF);
        /*KH*/
        TUE_CML_MutiplyMatrices_M(KH_crvKF, K_crvKF, H_crvKF);
        /*I-[KH]*/
        TUE_CML_SubtractMatrices_M(ISubKH_crvKF, I_crvKF, KH_crvKF);
        /*P=[(I-K*H)]P*/
        TUE_CML_MutiplyMatrices_M(P_tmp_crvKF, ISubKH_crvKF, P_crvKF);
        TUE_CML_GetMatrixElement_M(P_crvKF, 0, 0) =
            TUE_CML_GetMatrixElement_M(P_tmp_crvKF, 0, 0);
        TUE_CML_GetMatrixElement_M(P_crvKF, 0, 1) =
            TUE_CML_GetMatrixElement_M(P_tmp_crvKF, 0, 1);
        TUE_CML_GetMatrixElement_M(P_crvKF, 1, 0) =
            TUE_CML_GetMatrixElement_M(P_tmp_crvKF, 1, 0);
        TUE_CML_GetMatrixElement_M(P_crvKF, 1, 1) =
            TUE_CML_GetMatrixElement_M(P_tmp_crvKF, 1, 1);

        measWeight_crvKF = weight;
    } else {
        reset_crvKF(x_crvKF, P_crvKF);
    }
}
/********************************init_crvKF*************************************************
        @fn            init_crvKF
        @brief         Initialization of Kalman filter for curvature filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        (*)-->init_crvKF
         -->(*)
        @enduml
**************************************************************************************************/
static void init_crvKF(const TUE_CML_sMatrix_t *z_crvKF,
                       TUE_CML_sMatrix_t *x_crvKF,
                       TUE_CML_sMatrix_t *P_crvKF,
                       TUE_CML_sMatrix_t *R_crvKF,
                       REAL32_T quality,
                       REAL32_T P_ABPLBP_CrvKFInitRFactor_nu,
                       REAL32_T P_ABPLBP_CrvKFMnInitQual_perc) {
    /*temporary matrices*/
    TUE_CML_CreateMatrix_M(tmp_crvKF, STATE_LENGTH_CRVKF, STATE_LENGTH_CRVKF)
        TUE_CML_CreateMatrix_M(R_scaled_crvKF, STATE_LENGTH_CRVKF,
                               STATE_LENGTH_CRVKF)
            TUE_CML_CreateMatrix_M(H_trans_crvKF, STATE_LENGTH_CRVKF,
                                   STATE_LENGTH_CRVKF)
                TUE_CML_CreateMatrix_M(H_crvKF, STATE_LENGTH_CRVKF,
                                       STATE_LENGTH_CRVKF)

                    if (quality > P_ABPLBP_CrvKFMnInitQual_perc) {
        /*scale R*/
        TUE_CML_InitMatrix_M(R_scaled_crvKF, STATE_LENGTH_CRVKF,
                             STATE_LENGTH_CRVKF, 0.0f);
        TUE_CML_GetMatrixElement_M(R_scaled_crvKF, 0, 0) =
            TUE_CML_GetMatrixElement_M(R_crvKF, 0, 0);
        TUE_CML_GetMatrixElement_M(R_scaled_crvKF, 1, 1) =
            TUE_CML_GetMatrixElement_M(R_crvKF, 1, 1);
        TUE_CML_ScaleMatrix_M(R_scaled_crvKF, P_ABPLBP_CrvKFInitRFactor_nu);

        /*initialize matrix H*/
        TUE_CML_CreateIdentityMatrix_M(H_crvKF, STATE_LENGTH_CRVKF);
        /*initialize matrix H'*/
        TUE_CML_TransposeMatrix_M(H_trans_crvKF, H_crvKF);
        /*tmp = H'R*/
        TUE_CML_MutiplyMatrices_M(tmp_crvKF, H_trans_crvKF, R_scaled_crvKF);
        /*[H'R]H*/
        TUE_CML_MutiplyMatrices_M(P_crvKF, tmp_crvKF, H_crvKF);
        /*fill x*/
        TUE_CML_MutiplyMatrices_M(x_crvKF, H_trans_crvKF, z_crvKF);
        /*fill other states*/
        valid_crvKF = 1U;
        internalQuality_crvKF = 0.0f;
        measWeight_crvKF = 0.0f;
        status_crvKF = 4u;
    }
}
/********************************predict_crvKF*************************************************
        @fn            predict_crvKF
        @brief         Prediction of Kalman filter for curvature filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title predict_crvKF
        (*)-->predict_crvKF
         -->(*)
        @enduml
**************************************************************************************************/
static void predict_crvKF(REAL32_T dX_crvKF,
                          TUE_CML_sMatrix_t *x_crvKF,
                          TUE_CML_sMatrix_t *P_crvKF,
                          REAL32_T P_ABPLBP_CrvKFDefCurve_1pm,
                          REAL32_T P_ABPLBP_CrvKFQ11Fac_nu,
                          REAL32_T P_ABPLBP_CrvKFQ11FacStraight_nu,
                          REAL32_T curvature_1pm) {
    /*initialize local matrices and variables*/
    TUE_CML_CreateMatrix_M(A_crvKF, STATE_LENGTH_CRVKF, STATE_LENGTH_CRVKF)
        TUE_CML_CreateMatrix_M(Q_crvKF, STATE_LENGTH_CRVKF, STATE_LENGTH_CRVKF)
            TUE_CML_CreateMatrix_M(B_crvKF, STATE_LENGTH_CRVKF, 1)

                TUE_CML_CreateMatrix_M(AX_crvKF, STATE_LENGTH_CRVKF, 1)
                    TUE_CML_CreateMatrix_M(A_trans_crvKF, STATE_LENGTH_CRVKF,
                                           STATE_LENGTH_CRVKF)
                        TUE_CML_CreateMatrix_M(PA_trans_crvKF,
                                               STATE_LENGTH_CRVKF,
                                               STATE_LENGTH_CRVKF)
                            TUE_CML_CreateMatrix_M(APA_trans_crvKF,
                                                   STATE_LENGTH_CRVKF,
                                                   STATE_LENGTH_CRVKF)

                                REAL32_T dXPow2_crvKF;
    REAL32_T dXPow3_crvKF;
    REAL32_T dXPow4_crvKF;

    REAL32_T Q00_crvKF;
    REAL32_T Q01_crvKF;

    REAL32_T Q11_crvKF;

    REAL32_T sigmaSqr_crvKF;

    if (valid_crvKF) {
        /*initialize system matrix A*/
        TUE_CML_InitMatrix_M(A_crvKF, STATE_LENGTH_CRVKF, STATE_LENGTH_CRVKF,
                             0.0f);
        dXPow2_crvKF = dX_crvKF * dX_crvKF;
        dXPow3_crvKF = dXPow2_crvKF * dX_crvKF;
        dXPow4_crvKF = dXPow3_crvKF * dX_crvKF;
        /*row 0: 1 dX_crvKF*/
        TUE_CML_GetMatrixElement_M(A_crvKF, 0, 0) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_crvKF, 0, 1) = dX_crvKF;
        /*row 1: 0 1*/
        TUE_CML_GetMatrixElement_M(A_crvKF, 1, 1) = 1.0f;
        /*initialize Q*/
        sigmaSqr_crvKF = kappa2diff_sigma_crvKF * kappa2diff_sigma_crvKF;
        Q00_crvKF = dXPow4_crvKF * sigmaSqr_crvKF / 4.0f;
        Q01_crvKF = dXPow3_crvKF * sigmaSqr_crvKF / 2.0f;
        Q11_crvKF = dXPow2_crvKF * sigmaSqr_crvKF;
        TUE_CML_GetMatrixElement_M(Q_crvKF, 0, 0) = Q00_crvKF;
        TUE_CML_GetMatrixElement_M(Q_crvKF, 0, 1) = Q01_crvKF;
        TUE_CML_GetMatrixElement_M(Q_crvKF, 1, 0) = Q01_crvKF;

        /* Smoothness and thus phase delay of the filtered curvature in
         * dependence of the absolute value of the measured curvature */
        if (TUE_CML_Abs_M(curvature_1pm) > P_ABPLBP_CrvKFDefCurve_1pm) {
            /* Driving in a curve -> Increase the filtering of kappa */
            TUE_CML_GetMatrixElement_M(Q_crvKF, 1, 1) =
                P_ABPLBP_CrvKFQ11Fac_nu *
                Q11_crvKF;  // 1.0005* or 1.0002* (smooth) or 2* (dynamic)
                            // Factor needed to increase the dynamic of kappa
        } else {
            /* Driving on a straight -> increase the dynamic of kappa*/
            TUE_CML_GetMatrixElement_M(Q_crvKF, 1, 1) =
                P_ABPLBP_CrvKFQ11FacStraight_nu *
                Q11_crvKF;  // 1.0005* or 1.0002* (smooth) or 2* (dynamic)
                            // Factor needed to increase the dynamic of kappa
        }

        /*initialize steering matrix*/
        TUE_CML_InitMatrix_M(B_crvKF, STATE_LENGTH_CRVKF, 1, 0.0f);
        /*Ax*/
        TUE_CML_MutiplyMatrices_M(AX_crvKF, A_crvKF, x_crvKF);
        /*x = Ax + Bu*/
        TUE_CML_AddMatrices_M(x_crvKF, AX_crvKF, B_crvKF);
        /*A'*/
        TUE_CML_TransposeMatrix_M(A_trans_crvKF, A_crvKF);
        /*PA'*/
        TUE_CML_MutiplyMatrices_M(PA_trans_crvKF, P_crvKF, A_trans_crvKF);
        /*APA'*/
        TUE_CML_MutiplyMatrices_M(APA_trans_crvKF, A_crvKF, PA_trans_crvKF);
        /*P=APA'+Q*/
        TUE_CML_AddMatrices_M(P_crvKF, APA_trans_crvKF, Q_crvKF);

        /*Measurement Weight*/
        measWeight_crvKF = 0.0f;
        status_crvKF = 3u;
    }
}

/********************************reset_crvKF*************************************************
        @fn            reset_crvKF
        @brief         Reset of Kalman filter for curvature filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title reset_crvKF
        (*)-->reset_crvKF
         -->(*)
        @enduml
**************************************************************************************************/
static void reset_crvKF(TUE_CML_sMatrix_t *x_crvKF,
                        TUE_CML_sMatrix_t *P_crvKF) {
    /*reset_crvKF parameters*/
    TUE_CML_InitMatrix_M(x_crvKF, STATE_LENGTH_CRVKF, 1, 0.0f);
    TUE_CML_InitMatrix_M(P_crvKF, STATE_LENGTH_CRVKF, STATE_LENGTH_CRVKF, 0.0f);
    valid_crvKF = FALSE;
    internalQuality_crvKF = 0.0f;
    measWeight_crvKF = 0.0f;
    status_crvKF = 5u;
}

/********************************maintenance_crvKF*************************************************
        @fn            maintenance_crvKF
        @brief         Maintenance of Kalman filter for curvature filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title maintenance_crvKF
        (*)-->maintenance_crvKF
         -->(*)
        @enduml
**************************************************************************************************/
static void maintenance_crvKF(REAL32_T deltaT_sec,
                              REAL32_T P_ABPLBP_CrvKFIncQual_1ps,
                              REAL32_T P_ABPLBP_CrvKFDecQualDeg_1ps,
                              REAL32_T P_ABPLBP_CrvKFDecQualPred_1ps,
                              TUE_CML_sMatrix_t *x_crvKF,
                              TUE_CML_sMatrix_t *P_crvKF) {
    if (valid_crvKF) {
        /*Full Update, Degraded Update or No Update (predict only)? */
        if (measWeight_crvKF > 0.9f) {
            /* Full Update */
            status_crvKF = 1u;
            /* Increase internal quality by P_ABPLBP_CrvKFIncQual_1ps per
             * second*/
            internalQuality_crvKF += P_ABPLBP_CrvKFIncQual_1ps * deltaT_sec;
        } else if (measWeight_crvKF > 0.0f) {
            /* Degraded Update */
            status_crvKF = 2u;
            /* Decrease internal quality by P_ABPLBP_CrvKFDecQualDeg_1ps per
             * seconds */
            internalQuality_crvKF -= P_ABPLBP_CrvKFDecQualDeg_1ps * deltaT_sec;
        } else {
            /* No Update - Predict Only */
            status_crvKF = 3u;
            /* Decrease internal quality by P_ABPLBP_CrvKFDecQualPred_1ps per
             * seconds */
            internalQuality_crvKF -= P_ABPLBP_CrvKFDecQualPred_1ps * deltaT_sec;
        }

        /* Boundary check: 0 <= internalQuality <= 100 */
        if (internalQuality_crvKF < 0.0f) {
            internalQuality_crvKF = 0.0f;
            reset_crvKF(x_crvKF, P_crvKF);
        } else if (internalQuality_crvKF > 100.0f) {
            internalQuality_crvKF = 100.0f;
        }
    } else {
        status_crvKF = 0u;
    }
}

/*length of the KF state vector*/
#define STATE_LENGTH_LANEKF (4u)

/*****************************************************************************
 FUNCTIONS
 *****************************************************************************/
/********************************laneKalmanFilter_Left*************************************************
       @fn            laneKalmanFilter_Left
       @brief         Kalman filter for left lane filter
       @description   Refer to Kalman filter algorithm for details
       @param[in]     inputs  :
       @param[out]    outputs :
       @startuml
       title laneKalmanFilter_Left
       (*)-->laneKalmanFilter_Left
        -->(*)
       @enduml
**************************************************************************************************/
void laneKalmanFilter_Left(const laneKFInTypeV3 *inputs,
                           laneKFOutType *outputs) {
    /*state Matrix x_laneKFLe*/
    TUE_CML_CreateMatrix_M(x_laneKFLe, STATE_LENGTH_LANEKF, 1)
        /*covariance matrix P_laneKFLe*/
        TUE_CML_CreateMatrix_M(P_laneKFLe, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
        /*measurement matrix z_laneKFLe*/
        TUE_CML_CreateMatrix_M(z_laneKFLe, STATE_LENGTH_LANEKF, 1)
        /*measurement variance matrix R_laneKFLe*/
        TUE_CML_CreateMatrix_M(R_laneKFLe, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)

            REAL32_T dX_laneKFLe;
    // REAL32_T PosY0Diff_laneKFLe;
    /*calculate dX_laneKFLe*/
    dX_laneKFLe = inputs->sf_DeltaT_sec * inputs->sf_VehVelX_mps;
    vehYawRateStdDev_laneKFLe = inputs->sf_VehYawRateStdDev_radps;
    /*geometric dependent model error*/
    kappa2diff_sigma_laneKFLe =
        1 / inputs->sf_LaneKFErrCoeff1_met /
        (inputs->sf_LaneKFErrCoeff2_mps * inputs->sf_DeltaT_sec) /
        (inputs->sf_LaneKFErrCoeff2_mps * inputs->sf_DeltaT_sec);

    /*In case of an lane change allow the PosY0 position to jump in the
     * corresponding cycle - only if kalman filter has been valid*/
    if (status_laneKFLe > 0 && status_laneKFLe < 4) {
        /*check lane change detection*/
        if (inputs->sf_LaneChange_bool) {
            /*a lateral position jump is forced in case of a lane change*/
            TUE_CML_GetMatrixElement_M(x_laneKFLe, 0, 0) = inputs->sf_PosY0_met;
        }
    }

    /*predict_laneKFLe -  will only be executed if valid_laneKFLe state is set*/
    predict_laneKFLe(inputs->sf_DeltaT_sec, dX_laneKFLe,
                     inputs->sf_VehYawRate_radps, x_laneKFLe, P_laneKFLe,
                     inputs->sf_LaneKFDynDistYFact_nu,
                     inputs->sf_LaneKFDynYawFactor_nu);

    /*initialize measurement vector z_laneKFLe*/
    TUE_CML_GetMatrixElement_M(z_laneKFLe, 0, 0) = inputs->sf_PosY0_met;
    TUE_CML_GetMatrixElement_M(z_laneKFLe, 1, 0) = inputs->sf_HeadingAngle_rad;
    TUE_CML_GetMatrixElement_M(z_laneKFLe, 2, 0) = inputs->sf_Crv_1pm;
    TUE_CML_GetMatrixElement_M(z_laneKFLe, 3, 0) = inputs->sf_CrvChng_1pm2;

    /*initialize R_laneKFLe matrix*/
    TUE_CML_GetMatrixElement_M(R_laneKFLe, 0, 0) =
        inputs->sf_PosY0StdDev_met * inputs->sf_PosY0StdDev_met;
    TUE_CML_GetMatrixElement_M(R_laneKFLe, 1, 1) =
        inputs->sf_HeadingAngleStdDev_rad * inputs->sf_HeadingAngleStdDev_rad;
    TUE_CML_GetMatrixElement_M(R_laneKFLe, 2, 2) =
        inputs->sf_CrvStdDev_1pm * inputs->sf_CrvStdDev_1pm;
    TUE_CML_GetMatrixElement_M(R_laneKFLe, 3, 3) =
        inputs->sf_CrvChngStdDev_1pm2 * inputs->sf_CrvChngStdDev_1pm2;

    /*Init or Update*/
    if (!valid_laneKFLe && !(inputs->sf_DegradedUpdate_bool)) {
        /*initialization - for A_laneKFLe better initialization the R_laneKFLe
         * matrix is multiplied with A_laneKFLe constant (>1)*/
        init_laneKFLe(z_laneKFLe, R_laneKFLe,
                      inputs->sf_OverallMeasurementQuality_perc, x_laneKFLe,
                      P_laneKFLe, inputs->sf_LaneKFMnInitQual_perc,
                      inputs->sf_LaneKFInitRFactor_nu);
    }
    /*Update only if valid AND measurement quality >
       sf_LaneKFMnUpdateQual_perc*/
    else if (valid_laneKFLe && (inputs->sf_OverallMeasurementQuality_perc >=
                                (inputs->sf_LaneKFMnUpdateQual_perc))) {
        if (inputs->sf_DegradedUpdate_bool) {
            /*degraded update_laneKFLe - quality and weight are the 2
             * parameters*/
            update_laneKFLe(z_laneKFLe, R_laneKFLe,
                            inputs->sf_LaneKFDegradeWeight_nu, x_laneKFLe,
                            P_laneKFLe, inputs->sf_LaneKFKGainFac_nu);
        } else {
            /*full update_laneKFLe*/
            update_laneKFLe(z_laneKFLe, R_laneKFLe, 1.0f, x_laneKFLe,
                            P_laneKFLe, inputs->sf_LaneKFKGainFac_nu);
        }
    }
    /*maintenance_laneKFLe*/
    if (!(status_laneKFLe == 4)) {
        maintenance_laneKFLe(
            x_laneKFLe, P_laneKFLe, inputs->sf_LaneKFIncQual_1ps,
            inputs->sf_LaneKFDecQualDeg_1ps, inputs->sf_LaneKFDecQualPred_1ps,
            inputs->sf_DeltaT_sec);
    }
    /*Reset filter if basic lane data are not valid*/
    if (!inputs->sf_LaneDataValid_bool) reset_laneKFLe(x_laneKFLe, P_laneKFLe);
    /*Set outputs*/
    outputs->sf_PosY0_met = TUE_CML_GetMatrixElement_M(x_laneKFLe, 0, 0);
    outputs->sf_HeadingAngle_rad = TUE_CML_GetMatrixElement_M(x_laneKFLe, 1, 0);
    outputs->sf_Crv_1pm = TUE_CML_GetMatrixElement_M(x_laneKFLe, 2, 0);
    outputs->sf_CrvChng_1pm2 = TUE_CML_GetMatrixElement_M(x_laneKFLe, 3, 0);
    outputs->sf_KFStatus_btf = status_laneKFLe;
    outputs->sf_QualityMeasure_perc = (UINT8_T)(internalQuality_laneKFLe);
}

/********************************reset_laneKFLe*************************************************
   @fn            reset_laneKFLe
   @brief         Reset of Kalman filter for left lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title reset_laneKFLe
   (*)-->reset_laneKFLe
        -->(*)
   @enduml
**************************************************************************************************/
static void reset_laneKFLe(TUE_CML_sMatrix_t *x_laneKFLe,
                           TUE_CML_sMatrix_t *P_laneKFLe) {
    /*reset_laneKFLe parameters*/
    TUE_CML_InitMatrix_M(x_laneKFLe, STATE_LENGTH_LANEKF, 1, 0.0f);
    TUE_CML_InitMatrix_M(P_laneKFLe, STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF,
                         0.0f);
    valid_laneKFLe = FALSE;
    internalQuality_laneKFLe = 0.0f;
    measWeight_laneKFLe = 0.0f;
    status_laneKFLe = 5u;
}

/********************************maintenance_laneKFLe*************************************************
   @fn            maintenance_laneKFLe
   @brief         Maintenance of Kalman filter for left lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title maintenance_laneKFLe
   (*)-->maintenance_laneKFLe
        -->(*)
   @enduml
**************************************************************************************************/
static void maintenance_laneKFLe(TUE_CML_sMatrix_t *x_laneKFLe,
                                 TUE_CML_sMatrix_t *P_laneKFLe,
                                 REAL32_T P_ABPLBP_LaneKFIncQual_1ps,
                                 REAL32_T P_ABPLBP_LaneKFDecQualDeg_1ps,
                                 REAL32_T P_ABPLBP_LaneKFDecQualPred_1ps,
                                 REAL32_T deltaT_sec) {
    if (valid_laneKFLe) {
        /*Full Update, Degraded Update or No Update (predict only)? */
        if (measWeight_laneKFLe > 0.9f) {
            /* Full Update */
            status_laneKFLe = 1u;
            /* Increase internal quality by P_ABPLBP_LaneKFIncQual_1ps per
             * second*/
            internalQuality_laneKFLe += P_ABPLBP_LaneKFIncQual_1ps * deltaT_sec;
        } else if (measWeight_laneKFLe > 0.0f) {
            /* Degraded Update */
            status_laneKFLe = 2u;
            /* Decrease internal quality by P_ABPLBP_LaneKFDecQualDeg_1ps per
             * seconds */
            internalQuality_laneKFLe -=
                P_ABPLBP_LaneKFDecQualDeg_1ps * deltaT_sec;
        } else {
            /* No Update - Predict Only */
            status_laneKFLe = 3u;
            /* Decrease internal quality by P_ABPLBP_LaneKFDecQualPred_1ps per
             * seconds */
            internalQuality_laneKFLe -=
                P_ABPLBP_LaneKFDecQualPred_1ps * deltaT_sec;
        }

        /* Boundary check: 0 <= internalQuality <= 100 */
        if (internalQuality_laneKFLe < 0.0f) {
            internalQuality_laneKFLe = 0.0f;
            reset_laneKFLe(x_laneKFLe, P_laneKFLe);
        } else if (internalQuality_laneKFLe > 100.0f) {
            internalQuality_laneKFLe = 100.0f;
        }
    } else {
        status_laneKFLe = 0u;
    }
}
/********************************update_laneKFLe*************************************************
   @fn            update_laneKFLe
   @brief         Update  of Kalman filter for left lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title update_laneKFLe
   (*)-->update_laneKFLe
        -->(*)
   @enduml
**************************************************************************************************/
static void update_laneKFLe(const TUE_CML_sMatrix_t *z_laneKFLe,
                            const TUE_CML_sMatrix_t *R_laneKFLe,
                            REAL32_T weight,
                            TUE_CML_sMatrix_t *x_laneKFLe,
                            TUE_CML_sMatrix_t *P_laneKFLe,
                            REAL32_T P_ABPLBP_LaneKFKGainFac_nu) {
    /*temporary matrices*/
    TUE_CML_CreateMatrix_M(
        I_laneKFLe, STATE_LENGTH_LANEKF,
        STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(H_laneKFLe,
                                                    STATE_LENGTH_LANEKF,
                                                    STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(
            H_trans_laneKFLe, STATE_LENGTH_LANEKF,
            STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(K_laneKFLe,
                                                        STATE_LENGTH_LANEKF,
                                                        STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(
                PH_trans_laneKFLe, STATE_LENGTH_LANEKF,
                STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(HPH_trans_laneKFLe,
                                                            STATE_LENGTH_LANEKF,
                                                            STATE_LENGTH_LANEKF)
                TUE_CML_CreateMatrix_M(HPH_transPlusR_laneKFLe,
                                       STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF)
                    TUE_CML_CreateMatrix_M(HPH_transPlusR_inv_laneKFLe,
                                           STATE_LENGTH_LANEKF,
                                           STATE_LENGTH_LANEKF)
                        TUE_CML_CreateMatrix_M(
                            HX_laneKFLe, STATE_LENGTH_LANEKF,
                            1) TUE_CML_CreateMatrix_M(ZSubHX_laneKFLe,
                                                      STATE_LENGTH_LANEKF, 1)
                            TUE_CML_CreateMatrix_M(KZSubHX_laneKFLe,
                                                   STATE_LENGTH_LANEKF, 1)
                                TUE_CML_CreateMatrix_M(KH_laneKFLe,
                                                       STATE_LENGTH_LANEKF,
                                                       STATE_LENGTH_LANEKF)
                                    TUE_CML_CreateMatrix_M(ISubKH_laneKFLe,
                                                           STATE_LENGTH_LANEKF,
                                                           STATE_LENGTH_LANEKF)
                                        TUE_CML_CreateMatrix_M(
                                            P_tmp_laneKFLe, STATE_LENGTH_LANEKF,
                                            STATE_LENGTH_LANEKF)

        /*update_laneKFLe state with measurement*/
        TUE_CML_CreateIdentityMatrix_M(I_laneKFLe, STATE_LENGTH_LANEKF);
    TUE_CML_CreateIdentityMatrix_M(H_laneKFLe, STATE_LENGTH_LANEKF);
    TUE_CML_TransposeMatrix_M(H_trans_laneKFLe, H_laneKFLe);
    /*PH'*/
    TUE_CML_MutiplyMatrices_M(PH_trans_laneKFLe, P_laneKFLe, H_trans_laneKFLe);
    /*H_laneKFLe[PH']*/
    TUE_CML_MutiplyMatrices_M(HPH_trans_laneKFLe, H_laneKFLe,
                              PH_trans_laneKFLe);
    /*[HPH']+R_laneKFLe*/
    TUE_CML_AddMatrices_M(HPH_transPlusR_laneKFLe, HPH_trans_laneKFLe,
                          R_laneKFLe);
    /*inv([HPH'+R_laneKFLe]*/
    // CML_v_InvertMatrix(HPH_transPlusR_inv_laneKFLe, HPH_transPlusR_laneKFLe);
    TUE_CML_InvertMatrix_M(HPH_transPlusR_inv_laneKFLe,
                           HPH_transPlusR_laneKFLe);
    /*if inversion fails num. of cols and rows are set to 0*/
    if (HPH_transPlusR_inv_laneKFLe->Desc.col > 0) {
        /*K_laneKFLe=[PH'][inv(HPH'+R_laneKFLe)]*/
        TUE_CML_MutiplyMatrices_M(K_laneKFLe, PH_trans_laneKFLe,
                                  HPH_transPlusR_inv_laneKFLe);
        /*K_laneKFLe=K_laneKFLe*weight*/
        TUE_CML_ScaleMatrix_M(K_laneKFLe, weight);

        /* if degradedUpdate == true -> Set first row of K_laneKFLe to
         * FACTOR*K_laneKFLe -> weight of Disty = FACTOR*weight */
        if (weight < 1.0f) {
            /*set process noise covariance matrix Q_laneKFLe row 0*/
            TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 0) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 0);
            TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 1) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 1);
            TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 2) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 2);
            TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 3) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFLe, 0, 3);
        }

        /*HX_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(HX_laneKFLe, H_laneKFLe, x_laneKFLe);
        /*z_laneKFLe-[HX_laneKFLe]*/
        TUE_CML_SubtractMatrices_M(ZSubHX_laneKFLe, z_laneKFLe, HX_laneKFLe);
        /*K_laneKFLe[(z_laneKFLe-HX_laneKFLe)]*/
        TUE_CML_MutiplyMatrices_M(KZSubHX_laneKFLe, K_laneKFLe,
                                  ZSubHX_laneKFLe);
        /*x_laneKFLe=x_laneKFLe+K_laneKFLe(z_laneKFLe-HX_laneKFLe)*/
        TUE_CML_AddMatrices_M(x_laneKFLe, x_laneKFLe, KZSubHX_laneKFLe);
        /*KH_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(KH_laneKFLe, K_laneKFLe, H_laneKFLe);
        /*I_laneKFLe-[KH_laneKFLe]*/
        TUE_CML_SubtractMatrices_M(ISubKH_laneKFLe, I_laneKFLe, KH_laneKFLe);
        /*P_laneKFLe=[(I_laneKFLe-K_laneKFLe*H_laneKFLe)]P_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(P_tmp_laneKFLe, ISubKH_laneKFLe, P_laneKFLe);
        TUE_CML_CopyMatrix_M(P_laneKFLe, P_tmp_laneKFLe);

        measWeight_laneKFLe = weight;
    } else {
        reset_laneKFLe(x_laneKFLe, P_laneKFLe);
    }
}
/********************************predict_laneKFLe*************************************************
   @fn            predict_laneKFLe
   @brief         Prediction  of Kalman filter for left lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title predict_laneKFLe
   (*)-->predict_laneKFLe
        -->(*)
   @enduml
**************************************************************************************************/
static void predict_laneKFLe(REAL32_T dT_laneKFLe,
                             REAL32_T dX_laneKFLe,
                             REAL32_T vehYawRate,
                             TUE_CML_sMatrix_t *x_laneKFLe,
                             TUE_CML_sMatrix_t *P_laneKFLe,
                             REAL32_T P_ABPLBP_LaneKFDynDistYFact_nu,
                             REAL32_T P_ABPLBP_LaneKFDynYawFactor_nu) {
    /*initialize local matrices and variables*/
    TUE_CML_CreateMatrix_M(A_laneKFLe, STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(Q_laneKFLe, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(B_laneKFLe, STATE_LENGTH_LANEKF, 1)
                TUE_CML_CreateMatrix_M(AX_laneKFLe, STATE_LENGTH_LANEKF, 1)
                    TUE_CML_CreateMatrix_M(A_trans_laneKFLe,
                                           STATE_LENGTH_LANEKF,
                                           STATE_LENGTH_LANEKF)
                        TUE_CML_CreateMatrix_M(PA_trans_laneKFLe,
                                               STATE_LENGTH_LANEKF,
                                               STATE_LENGTH_LANEKF)
                            TUE_CML_CreateMatrix_M(APA_trans_laneKFLe,
                                                   STATE_LENGTH_LANEKF,
                                                   STATE_LENGTH_LANEKF)

                                REAL32_T dXPow2_laneKFLe;
    REAL32_T dXPow3_laneKFLe;
    REAL32_T dXPow4_laneKFLe;
    REAL32_T dXPow5_laneKFLe;
    REAL32_T dXPow6_laneKFLe;
    REAL32_T dXPow7_laneKFLe;
    REAL32_T dXPow8_laneKFLe;
    REAL32_T Q00_laneKFLe;
    REAL32_T Q01_laneKFLe;
    REAL32_T Q02_laneKFLe;
    REAL32_T Q03_laneKFLe;
    REAL32_T Q11_laneKFLe;
    REAL32_T Q12_laneKFLe;
    REAL32_T Q13_laneKFLe;
    REAL32_T Q22_laneKFLe;
    REAL32_T Q23_laneKFLe;
    REAL32_T Q33_laneKFLe;
    REAL32_T sigmaSqr_laneKFLe;

    if (valid_laneKFLe) {
        /*initialize system matrix A_laneKFLe*/
        TUE_CML_InitMatrix_M(A_laneKFLe, STATE_LENGTH_LANEKF,
                             STATE_LENGTH_LANEKF, 0.0f);
        /*calculate dX_laneKFLe to the power of 2*/
        dXPow2_laneKFLe = dX_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 3*/
        dXPow3_laneKFLe = dXPow2_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 4*/
        dXPow4_laneKFLe = dXPow3_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 5*/
        dXPow5_laneKFLe = dXPow4_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 6*/
        dXPow6_laneKFLe = dXPow5_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 7*/
        dXPow7_laneKFLe = dXPow6_laneKFLe * dX_laneKFLe;
        /*calculate dX_laneKFLe to the power of 8*/
        dXPow8_laneKFLe = dXPow7_laneKFLe * dX_laneKFLe;

        /*row 0: 1 dX_laneKFLe 1/2*dX_laneKFLe^2 1/6*dX_laneKFLe^3*/
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 0, 0) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 0, 1) = dX_laneKFLe;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 0, 2) = 0.5f * dXPow2_laneKFLe;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 0, 3) =
            1.0f / 6.0f * dXPow3_laneKFLe;
        /*row 1: 0 1 -dX_laneKFLe - dX_laneKFLe^2*/
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 1, 1) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 1, 2) = dX_laneKFLe;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 1, 3) = 0.5f * dXPow2_laneKFLe;
        /*row 2: 0 0 1 dX_laneKFLe*/
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 2, 2) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 2, 3) = dX_laneKFLe;
        /*row 3: 0 0 0 1*/
        TUE_CML_GetMatrixElement_M(A_laneKFLe, 3, 3) = 1.0f;

        /*calculate sigma squared*/
        sigmaSqr_laneKFLe =
            kappa2diff_sigma_laneKFLe * kappa2diff_sigma_laneKFLe;

        /*calculate process noise covariance matrix Q_laneKFLe elements*/
        Q00_laneKFLe = dXPow8_laneKFLe * sigmaSqr_laneKFLe / 576.0f;
        Q01_laneKFLe = dXPow7_laneKFLe * sigmaSqr_laneKFLe / 48.0f;
        Q02_laneKFLe = dXPow6_laneKFLe * sigmaSqr_laneKFLe / 48.0f;
        Q03_laneKFLe = dXPow5_laneKFLe * sigmaSqr_laneKFLe / 24.0f;

        Q11_laneKFLe = dXPow6_laneKFLe * sigmaSqr_laneKFLe / 4.0f;
        Q12_laneKFLe = dXPow5_laneKFLe * sigmaSqr_laneKFLe / 4.0f;
        Q13_laneKFLe = dXPow4_laneKFLe * sigmaSqr_laneKFLe / 2.0f;

        Q22_laneKFLe = dXPow4_laneKFLe * sigmaSqr_laneKFLe / 4.0f;
        Q23_laneKFLe = dXPow3_laneKFLe * sigmaSqr_laneKFLe / 2.0f;

        Q33_laneKFLe = dXPow2_laneKFLe * sigmaSqr_laneKFLe;

        /*set process noise covariance matrix Q_laneKFLe row 0*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 0) =
            P_ABPLBP_LaneKFDynDistYFact_nu *
            Q00_laneKFLe;  // Factor needed to increase the dynamic of DistY
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 1) = Q01_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 2) = Q02_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 3) = Q03_laneKFLe;
        /*set process noise covariance matrix Q_laneKFLe row 1*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 0) = Q01_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 1) =
            P_ABPLBP_LaneKFDynYawFactor_nu * Q11_laneKFLe;  // Factor needed to
                                                            // increase the
                                                            // dynamic of
                                                            // HeadingAngle
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 2) = Q12_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 3) = Q13_laneKFLe;
        /*set process noise covariance matrix Q_laneKFLe row 2*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 2, 0) = Q02_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 2, 1) = Q12_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 2, 2) =
            100.0f *
            Q22_laneKFLe;  // Factor needed to increase the dynamic of Kappa
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 2, 3) = Q23_laneKFLe;
        /*set process noise covariance matrix Q_laneKFLe row 3*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 3, 0) = Q03_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 3, 1) = Q13_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 3, 2) = Q23_laneKFLe;
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 3, 3) = 2.0f * Q33_laneKFLe;

        /*additional noise caused by vehicle movement*/
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 0, 0) +=
            (dT_laneKFLe * dX_laneKFLe * vehYawRateStdDev_laneKFLe) *
            (dT_laneKFLe * dX_laneKFLe * vehYawRateStdDev_laneKFLe);
        TUE_CML_GetMatrixElement_M(Q_laneKFLe, 1, 1) +=
            (dT_laneKFLe * vehYawRateStdDev_laneKFLe) *
            (dT_laneKFLe * vehYawRateStdDev_laneKFLe);

        /*initialize steering matrix*/

        TUE_CML_InitMatrix_M(B_laneKFLe, STATE_LENGTH_LANEKF, 1, 0.0f);
        TUE_CML_GetMatrixElement_M(B_laneKFLe, 0, 0) =
            -dT_laneKFLe * dX_laneKFLe;
        TUE_CML_GetMatrixElement_M(B_laneKFLe, 1, 0) = -dT_laneKFLe;
        /*AX_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(AX_laneKFLe, A_laneKFLe, x_laneKFLe);
        /*Bu*/
        TUE_CML_ScaleMatrix_M(B_laneKFLe, vehYawRate);
        /*x_laneKFLe = AX_laneKFLe + Bu*/
        TUE_CML_AddMatrices_M(x_laneKFLe, AX_laneKFLe, B_laneKFLe);
        /*A_laneKFLe'*/
        TUE_CML_TransposeMatrix_M(A_trans_laneKFLe, A_laneKFLe);
        /*PA'*/
        TUE_CML_MutiplyMatrices_M(PA_trans_laneKFLe, P_laneKFLe,
                                  A_trans_laneKFLe);
        /*APA'*/
        TUE_CML_MutiplyMatrices_M(APA_trans_laneKFLe, A_laneKFLe,
                                  PA_trans_laneKFLe);
        /*P_laneKFLe=APA'+Q_laneKFLe*/
        TUE_CML_AddMatrices_M(P_laneKFLe, APA_trans_laneKFLe, Q_laneKFLe);

        /*Measurement Weight*/
        measWeight_laneKFLe = 0.0f;
        status_laneKFLe = 3u;
    }
}
/********************************init_laneKFLe*************************************************
   @fn            init_laneKFLe
   @brief         Initialization  of Kalman filter for left lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title init_laneKFLe
   (*)-->init_laneKFLe
        -->(*)
   @enduml
**************************************************************************************************/
static void init_laneKFLe(const TUE_CML_sMatrix_t *z_laneKFLe,
                          TUE_CML_sMatrix_t *R_laneKFLe,
                          REAL32_T quality,
                          TUE_CML_sMatrix_t *x_laneKFLe,
                          TUE_CML_sMatrix_t *P_laneKFLe,
                          UINT8_T P_ABPLBP_LaneKFMnInitQual_perc,
                          REAL32_T P_ABPLBP_LaneKFInitRFactor_nu) {
    TUE_CML_CreateMatrix_M(H_trans_laneKFLe, STATE_LENGTH_LANEKF,
                           STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(H_laneKFLe, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(tmp_laneKFLe, STATE_LENGTH_LANEKF,
                                   STATE_LENGTH_LANEKF)
                TUE_CML_CreateMatrix_M(R_scaled_laneKFLe, STATE_LENGTH_LANEKF,
                                       STATE_LENGTH_LANEKF)

                    if (quality > P_ABPLBP_LaneKFMnInitQual_perc) {
        /*scale R_laneKFLe*/
        TUE_CML_InitMatrix_M(R_scaled_laneKFLe, STATE_LENGTH_LANEKF,
                             STATE_LENGTH_LANEKF, 0.0f);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFLe, 0, 0) =
            TUE_CML_GetMatrixElement_M(R_laneKFLe, 0, 0);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFLe, 1, 1) =
            TUE_CML_GetMatrixElement_M(R_laneKFLe, 1, 1);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFLe, 2, 2) =
            TUE_CML_GetMatrixElement_M(R_laneKFLe, 2, 2);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFLe, 3, 3) =
            TUE_CML_GetMatrixElement_M(R_laneKFLe, 3, 3);
        TUE_CML_ScaleMatrix_M(R_scaled_laneKFLe, P_ABPLBP_LaneKFInitRFactor_nu);

        /*initialize matrix H_laneKFLe*/
        TUE_CML_CreateIdentityMatrix_M(H_laneKFLe, STATE_LENGTH_LANEKF);
        /*initialize matrix H_laneKFLe'*/
        TUE_CML_TransposeMatrix_M(H_trans_laneKFLe, H_laneKFLe);
        /*tmp_laneKFLe = H_laneKFLe'R_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(tmp_laneKFLe, H_trans_laneKFLe,
                                  R_scaled_laneKFLe);
        /*[H_laneKFLe'R_laneKFLe]H_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(P_laneKFLe, tmp_laneKFLe, H_laneKFLe);
        /*fill x_laneKFLe*/
        TUE_CML_MutiplyMatrices_M(x_laneKFLe, H_trans_laneKFLe, z_laneKFLe);
        /*fill other states*/
        valid_laneKFLe = 1U;
        internalQuality_laneKFLe = 0.0f;
        measWeight_laneKFLe = 0.0f;
        status_laneKFLe = 4u;
    }
}

/*****************************************************************************
 MACROS
 *****************************************************************************/
#ifndef FALSE
#define FALSE (0u)
#endif
#ifndef TRUE
#define TRUE (1u)
#endif

/*length of the KF state vector*/
#define STATE_LENGTH_LANEKF (4u)

/*****************************************************************************
 FUNCTIONS
 *****************************************************************************/
/********************************laneKalmanFilter_Center*************************************************
        @fn            laneKalmanFilter_Center
        @brief         Kalman filter for center lane filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title laneKalmanFilter_Center
        (*)-->laneKalmanFilter_Center
         -->(*)
        @enduml
**************************************************************************************************/
void laneKalmanFilter_Center(const laneKFInTypeV3 *inputs,
                             laneKFOutType *outputs) {
    /*state Matrix x_laneKFCntr*/
    TUE_CML_CreateMatrix_M(x_laneKFCntr, STATE_LENGTH_LANEKF, 1)
        /*covariance matrix P_laneKFCntr*/
        TUE_CML_CreateMatrix_M(P_laneKFCntr, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
        /*measurement matrix z_laneKFCntr*/
        TUE_CML_CreateMatrix_M(z_laneKFCntr, STATE_LENGTH_LANEKF, 1)
        /*measurement variance matrix R_laneKFCntr*/
        TUE_CML_CreateMatrix_M(R_laneKFCntr, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)

            REAL32_T dX_laneKFCntr;
    // REAL32_T PosY0Diff_laneKFCntr;
    /*calculate dX_laneKFCntr*/
    dX_laneKFCntr = inputs->sf_DeltaT_sec * inputs->sf_VehVelX_mps;
    vehYawRateStdDev_laneKFCntr = inputs->sf_VehYawRateStdDev_radps;
    /*geometric dependent model error*/
    kappa2diff_sigma_laneKFCntr =
        1 / inputs->sf_LaneKFErrCoeff1_met /
        (inputs->sf_LaneKFErrCoeff2_mps * inputs->sf_DeltaT_sec) /
        (inputs->sf_LaneKFErrCoeff2_mps * inputs->sf_DeltaT_sec);

    /*In case of an lane change allow the PosY0 position to jump in the
     * corresponding cycle - only if kalman filter has been valid*/
    if (status_laneKFCntr > 0 && status_laneKFCntr < 4) {
        /*check lane change detection*/
        if (inputs->sf_LaneChange_bool) {
            /*a lateral position jump is forced in case of a lane change*/
            TUE_CML_GetMatrixElement_M(x_laneKFCntr, 0, 0) =
                inputs->sf_PosY0_met;
        }
    }

    /*predict_laneKFCntr -  will only be executed if valid_laneKFCntr state is
     * set*/
    predict_laneKFCntr(
        inputs->sf_DeltaT_sec, dX_laneKFCntr, inputs->sf_VehYawRate_radps,
        x_laneKFCntr, P_laneKFCntr, inputs->sf_LaneKFDynDistYFact_nu,
        inputs->sf_LaneKFDynYawFactor_nu, inputs->sf_LaneKFDynCrvFact_nu,
        inputs->sf_LaneKFDynCrvRateFact_nu);

    /*initialize measurement vector z_laneKFCntr*/
    TUE_CML_GetMatrixElement_M(z_laneKFCntr, 0, 0) = inputs->sf_PosY0_met;
    TUE_CML_GetMatrixElement_M(z_laneKFCntr, 1, 0) =
        inputs->sf_HeadingAngle_rad;
    TUE_CML_GetMatrixElement_M(z_laneKFCntr, 2, 0) = inputs->sf_Crv_1pm;
    TUE_CML_GetMatrixElement_M(z_laneKFCntr, 3, 0) = inputs->sf_CrvChng_1pm2;

    /*initialize R_laneKFCntr matrix*/
    TUE_CML_GetMatrixElement_M(R_laneKFCntr, 0, 0) =
        inputs->sf_PosY0StdDev_met * inputs->sf_PosY0StdDev_met;
    TUE_CML_GetMatrixElement_M(R_laneKFCntr, 1, 1) =
        inputs->sf_HeadingAngleStdDev_rad * inputs->sf_HeadingAngleStdDev_rad;
    TUE_CML_GetMatrixElement_M(R_laneKFCntr, 2, 2) =
        inputs->sf_CrvStdDev_1pm * inputs->sf_CrvStdDev_1pm;
    TUE_CML_GetMatrixElement_M(R_laneKFCntr, 3, 3) =
        inputs->sf_CrvChngStdDev_1pm2 * inputs->sf_CrvChngStdDev_1pm2;

    /*Init or Update*/
    if (!valid_laneKFCntr && !(inputs->sf_DegradedUpdate_bool)) {
        /*initialization - for A_laneKFCntr better initialization the
         * R_laneKFCntr matrix is multiplied with A_laneKFCntr constant (>1)*/
        init_laneKFCntr(z_laneKFCntr, R_laneKFCntr,
                        inputs->sf_OverallMeasurementQuality_perc, x_laneKFCntr,
                        P_laneKFCntr, inputs->sf_LaneKFMnInitQual_perc,
                        inputs->sf_LaneKFInitRFactor_nu);
    }
    /*Update only if valid AND measurement quality >
       sf_LaneKFMnUpdateQual_perc*/
    else if (valid_laneKFCntr && (inputs->sf_OverallMeasurementQuality_perc >=
                                  (inputs->sf_LaneKFMnUpdateQual_perc))) {
        if (inputs->sf_DegradedUpdate_bool) {
            /*degraded update_laneKFCntr - quality and weight are the 2
             * parameters*/
            update_laneKFCntr(z_laneKFCntr, R_laneKFCntr,
                              inputs->sf_LaneKFDegradeWeight_nu, x_laneKFCntr,
                              P_laneKFCntr, inputs->sf_LaneKFKGainFac_nu);
        } else {
            /*full update_laneKFCntr*/
            update_laneKFCntr(z_laneKFCntr, R_laneKFCntr, 1.0f, x_laneKFCntr,
                              P_laneKFCntr, inputs->sf_LaneKFKGainFac_nu);
        }
    }
    /*maintenance_laneKFCntr*/
    if (!(status_laneKFCntr == 4)) {
        maintenance_laneKFCntr(
            x_laneKFCntr, P_laneKFCntr, inputs->sf_LaneKFIncQual_1ps,
            inputs->sf_LaneKFDecQualDeg_1ps, inputs->sf_LaneKFDecQualPred_1ps,
            inputs->sf_DeltaT_sec);
    }
    /*Reset filter if basic lane data are not valid*/
    if (!inputs->sf_LaneDataValid_bool)
        reset_laneKFCntr(x_laneKFCntr, P_laneKFCntr);
    /*Set outputs*/
    outputs->sf_PosY0_met = TUE_CML_GetMatrixElement_M(x_laneKFCntr, 0, 0);
    outputs->sf_HeadingAngle_rad =
        TUE_CML_GetMatrixElement_M(x_laneKFCntr, 1, 0);
    outputs->sf_Crv_1pm = TUE_CML_GetMatrixElement_M(x_laneKFCntr, 2, 0);
    outputs->sf_CrvChng_1pm2 = TUE_CML_GetMatrixElement_M(x_laneKFCntr, 3, 0);
    outputs->sf_KFStatus_btf = status_laneKFCntr;
    outputs->sf_QualityMeasure_perc = (UINT8_T)(internalQuality_laneKFCntr);
}

/********************************reset_laneKFCntr*************************************************
        @fn            reset_laneKFCntr
        @brief         Reset of Kalman filter for center lane filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title reset_laneKFCntr
        (*)-->reset_laneKFCntr
         -->(*)
        @enduml
**************************************************************************************************/
static void reset_laneKFCntr(TUE_CML_sMatrix_t *x_laneKFCntr,
                             TUE_CML_sMatrix_t *P_laneKFCntr) {
    /*reset_laneKFCntr parameters*/
    TUE_CML_InitMatrix_M(x_laneKFCntr, STATE_LENGTH_LANEKF, 1, 0.0f);
    TUE_CML_InitMatrix_M(P_laneKFCntr, STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF,
                         0.0f);
    valid_laneKFCntr = FALSE;
    internalQuality_laneKFCntr = 0.0f;
    measWeight_laneKFCntr = 0.0f;
    status_laneKFCntr = 5u;
}
/********************************maintenance_laneKFCntr*************************************************
        @fn            maintenance_laneKFCntr
        @brief         Maintenance of Kalman filter for center lane filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title maintenance_laneKFCntr
        (*)-->maintenance_laneKFCntr
         -->(*)
        @enduml
**************************************************************************************************/
static void maintenance_laneKFCntr(TUE_CML_sMatrix_t *x_laneKFCntr,
                                   TUE_CML_sMatrix_t *P_laneKFCntr,
                                   REAL32_T P_ABPLBP_LaneKFIncQual_1ps,
                                   REAL32_T P_ABPLBP_LaneKFDecQualDeg_1ps,
                                   REAL32_T P_ABPLBP_LaneKFDecQualPred_1ps,
                                   REAL32_T deltaT_sec) {
    if (valid_laneKFCntr) {
        /*Full Update, Degraded Update or No Update (predict only)? */
        if (measWeight_laneKFCntr > 0.9f) {
            /* Full Update */
            status_laneKFCntr = 1u;
            /* Increase internal quality by P_ABPLBP_LaneKFIncQual_1ps per
             * second*/
            internalQuality_laneKFCntr +=
                P_ABPLBP_LaneKFIncQual_1ps * deltaT_sec;
        } else if (measWeight_laneKFCntr > 0.0f) {
            /* Degraded Update */
            status_laneKFCntr = 2u;
            /* Decrease internal quality by P_ABPLBP_LaneKFDecQualDeg_1ps per
             * seconds */
            internalQuality_laneKFCntr -=
                P_ABPLBP_LaneKFDecQualDeg_1ps * deltaT_sec;
        } else {
            /* No Update - Predict Only */
            status_laneKFCntr = 3u;
            /* Decrease internal quality by P_ABPLBP_LaneKFDecQualPred_1ps per
             * seconds */
            internalQuality_laneKFCntr -=
                P_ABPLBP_LaneKFDecQualPred_1ps * deltaT_sec;
        }

        /* Boundary check: 0 <= internalQuality <= 100 */
        if (internalQuality_laneKFCntr < 0.0f) {
            internalQuality_laneKFCntr = 0.0f;
            reset_laneKFCntr(x_laneKFCntr, P_laneKFCntr);
        } else if (internalQuality_laneKFCntr > 100.0f) {
            internalQuality_laneKFCntr = 100.0f;
        }
    } else {
        status_laneKFCntr = 0u;
    }
}
/********************************update_laneKFCntr*************************************************
        @fn            update_laneKFCntr
        @brief         Update of Kalman filter for center lane filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title update_laneKFCntr
        (*)-->update_laneKFCntr
         -->(*)
        @enduml
**************************************************************************************************/
static void update_laneKFCntr(const TUE_CML_sMatrix_t *z_laneKFCntr,
                              const TUE_CML_sMatrix_t *R_laneKFCntr,
                              REAL32_T weight,
                              TUE_CML_sMatrix_t *x_laneKFCntr,
                              TUE_CML_sMatrix_t *P_laneKFCntr,
                              REAL32_T P_ABPLBP_LaneKFKGainFac_nu) {
    /*temporary matrices*/
    TUE_CML_CreateMatrix_M(
        I_laneKFCntr, STATE_LENGTH_LANEKF,
        STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(H_laneKFCntr,
                                                    STATE_LENGTH_LANEKF,
                                                    STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(
            H_trans_laneKFCntr, STATE_LENGTH_LANEKF,
            STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(K_laneKFCntr,
                                                        STATE_LENGTH_LANEKF,
                                                        STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(PH_trans_laneKFCntr, STATE_LENGTH_LANEKF,
                                   STATE_LENGTH_LANEKF)
                TUE_CML_CreateMatrix_M(HPH_trans_laneKFCntr,
                                       STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF)
                    TUE_CML_CreateMatrix_M(HPH_transPlusR_laneKFCntr,
                                           STATE_LENGTH_LANEKF,
                                           STATE_LENGTH_LANEKF)
                        TUE_CML_CreateMatrix_M(HPH_transPlusR_inv_laneKFCntr,
                                               STATE_LENGTH_LANEKF,
                                               STATE_LENGTH_LANEKF)
                            TUE_CML_CreateMatrix_M(HX_laneKFCntr,
                                                   STATE_LENGTH_LANEKF, 1)
                                TUE_CML_CreateMatrix_M(ZSubHX_laneKFCntr,
                                                       STATE_LENGTH_LANEKF, 1)
                                    TUE_CML_CreateMatrix_M(KZSubHX_laneKFCntr,
                                                           STATE_LENGTH_LANEKF,
                                                           1)
                                        TUE_CML_CreateMatrix_M(
                                            KH_laneKFCntr, STATE_LENGTH_LANEKF,
                                            STATE_LENGTH_LANEKF)
                                            TUE_CML_CreateMatrix_M(
                                                ISubKH_laneKFCntr,
                                                STATE_LENGTH_LANEKF,
                                                STATE_LENGTH_LANEKF)
                                                TUE_CML_CreateMatrix_M(
                                                    P_tmp_laneKFCntr,
                                                    STATE_LENGTH_LANEKF,
                                                    STATE_LENGTH_LANEKF)

        /*update_laneKFCntr state with measurement*/
        TUE_CML_CreateIdentityMatrix_M(I_laneKFCntr, STATE_LENGTH_LANEKF);
    TUE_CML_CreateIdentityMatrix_M(H_laneKFCntr, STATE_LENGTH_LANEKF);
    TUE_CML_TransposeMatrix_M(H_trans_laneKFCntr, H_laneKFCntr);
    /*PH'*/
    TUE_CML_MutiplyMatrices_M(PH_trans_laneKFCntr, P_laneKFCntr,
                              H_trans_laneKFCntr);
    /*H_laneKFCntr[PH']*/
    TUE_CML_MutiplyMatrices_M(HPH_trans_laneKFCntr, H_laneKFCntr,
                              PH_trans_laneKFCntr);
    /*[HPH']+R_laneKFCntr*/
    TUE_CML_AddMatrices_M(HPH_transPlusR_laneKFCntr, HPH_trans_laneKFCntr,
                          R_laneKFCntr);
    /*inv([HPH'+R_laneKFCntr]*/
    TUE_CML_InvertMatrix_M(HPH_transPlusR_inv_laneKFCntr,
                           HPH_transPlusR_laneKFCntr);
    /*if inversion fails num. of cols and rows are set to 0*/
    if (HPH_transPlusR_inv_laneKFCntr->Desc.col > 0) {
        /*K_laneKFCntr=[PH'][inv(HPH'+R_laneKFCntr)]*/
        TUE_CML_MutiplyMatrices_M(K_laneKFCntr, PH_trans_laneKFCntr,
                                  HPH_transPlusR_inv_laneKFCntr);
        /*K_laneKFCntr=K_laneKFCntr*weight*/
        TUE_CML_ScaleMatrix_M(K_laneKFCntr, weight);

        /* if degradedUpdate == true -> Set first row of K_laneKFCntr to
         * FACTOR*K_laneKFCntr -> weight of Disty = FACTOR*weight */
        if (weight < 1.0f) {
            /*set process noise covariance matrix Q_laneKFCntr row 0*/
            TUE_CML_GetMatrixElement_M(K_laneKFCntr, 0, 0) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFCntr, 0, 0);
            TUE_CML_GetMatrixElement_M(K_laneKFCntr, 0, 1) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFCntr, 0, 1);
            TUE_CML_GetMatrixElement_M(K_laneKFCntr, 0, 2) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFCntr, 0, 2);
            TUE_CML_GetMatrixElement_M(K_laneKFCntr, 0, 3) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKFCntr, 0, 3);
        }

        /*HX_laneKFCntr*/
        TUE_CML_MutiplyMatrices_M(HX_laneKFCntr, H_laneKFCntr, x_laneKFCntr);
        /*z_laneKFCntr-[HX_laneKFCntr]*/
        TUE_CML_SubtractMatrices_M(ZSubHX_laneKFCntr, z_laneKFCntr,
                                   HX_laneKFCntr);
        /*K_laneKFCntr[(z_laneKFCntr-HX_laneKFCntr)]*/
        TUE_CML_MutiplyMatrices_M(KZSubHX_laneKFCntr, K_laneKFCntr,
                                  ZSubHX_laneKFCntr);
        /*x_laneKFCntr=x_laneKFCntr+K_laneKFCntr(z_laneKFCntr-HX_laneKFCntr)*/
        TUE_CML_AddMatrices_M(x_laneKFCntr, x_laneKFCntr, KZSubHX_laneKFCntr);
        /*KH_laneKFCntr*/
        TUE_CML_MutiplyMatrices_M(KH_laneKFCntr, K_laneKFCntr, H_laneKFCntr);
        /*I_laneKFCntr-[KH_laneKFCntr]*/
        TUE_CML_SubtractMatrices_M(ISubKH_laneKFCntr, I_laneKFCntr,
                                   KH_laneKFCntr);
        /*P_laneKFCntr=[(I_laneKFCntr-K_laneKFCntr*H_laneKFCntr)]P_laneKFCntr*/
        TUE_CML_MutiplyMatrices_M(P_tmp_laneKFCntr, ISubKH_laneKFCntr,
                                  P_laneKFCntr);
        TUE_CML_CopyMatrix_M(P_laneKFCntr, P_tmp_laneKFCntr);

        measWeight_laneKFCntr = weight;
    } else {
        reset_laneKFCntr(x_laneKFCntr, P_laneKFCntr);
    }
}
/********************************predict_laneKFCntr*************************************************
        @fn            predict_laneKFCntr
        @brief         Prediction of Kalman filter for center lane filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title predict_laneKFCntr
        (*)-->predict_laneKFCntr
         -->(*)
        @enduml
**************************************************************************************************/
static void predict_laneKFCntr(REAL32_T dT_laneKFCntr,
                               REAL32_T dX_laneKFCntr,
                               REAL32_T vehYawRate,
                               TUE_CML_sMatrix_t *x_laneKFCntr,
                               TUE_CML_sMatrix_t *P_laneKFCntr,
                               REAL32_T P_ABPLBP_LaneKFDynDistYFact_nu,
                               REAL32_T P_ABPLBP_LaneKFDynYawFactor_nu,
                               REAL32_T P_ABPLBP_LaneKFDynCrvFactor_nu,
                               REAL32_T P_ABPLBP_LaneKFDynCrvRtFactor_nu) {
    /*initialize local matrices and variables*/
    TUE_CML_CreateMatrix_M(A_laneKFCntr, STATE_LENGTH_LANEKF,
                           STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(Q_laneKFCntr, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(B_laneKFCntr, STATE_LENGTH_LANEKF, 1)
                TUE_CML_CreateMatrix_M(AX_laneKFCntr, STATE_LENGTH_LANEKF, 1)
                    TUE_CML_CreateMatrix_M(A_trans_laneKFCntr,
                                           STATE_LENGTH_LANEKF,
                                           STATE_LENGTH_LANEKF)
                        TUE_CML_CreateMatrix_M(PA_trans_laneKFCntr,
                                               STATE_LENGTH_LANEKF,
                                               STATE_LENGTH_LANEKF)
                            TUE_CML_CreateMatrix_M(APA_trans_laneKFCntr,
                                                   STATE_LENGTH_LANEKF,
                                                   STATE_LENGTH_LANEKF)

                                REAL32_T dXPow2_laneKFCntr;
    REAL32_T dXPow3_laneKFCntr;
    REAL32_T dXPow4_laneKFCntr;
    REAL32_T dXPow5_laneKFCntr;
    REAL32_T dXPow6_laneKFCntr;
    REAL32_T dXPow7_laneKFCntr;
    REAL32_T dXPow8_laneKFCntr;
    REAL32_T Q00_laneKFCntr;
    REAL32_T Q01_laneKFCntr;
    REAL32_T Q02_laneKFCntr;
    REAL32_T Q03_laneKFCntr;
    REAL32_T Q11_laneKFCntr;
    REAL32_T Q12_laneKFCntr;
    REAL32_T Q13_laneKFCntr;
    REAL32_T Q22_laneKFCntr;
    REAL32_T Q23_laneKFCntr;
    REAL32_T Q33_laneKFCntr;
    REAL32_T sigmaSqr_laneKFCntr;

    if (valid_laneKFCntr) {
        /*initialize system matrix A_laneKFCntr*/
        TUE_CML_InitMatrix_M(A_laneKFCntr, STATE_LENGTH_LANEKF,
                             STATE_LENGTH_LANEKF, 0.0f);
        /*calculate dX_laneKFCntr to the power of 2*/
        dXPow2_laneKFCntr = dX_laneKFCntr * dX_laneKFCntr;
        /*calculate dX_laneKFCntr to the power of 3*/
        dXPow3_laneKFCntr = dXPow2_laneKFCntr * dX_laneKFCntr;
        /*calculate dX_laneKFCntr to the power of 4*/
        dXPow4_laneKFCntr = dXPow3_laneKFCntr * dX_laneKFCntr;
        /*calculate dX_laneKFCntr to the power of 5*/
        dXPow5_laneKFCntr = dXPow4_laneKFCntr * dX_laneKFCntr;
        /*calculate dX_laneKFCntr to the power of 6*/
        dXPow6_laneKFCntr = dXPow5_laneKFCntr * dX_laneKFCntr;
        /*calculate dX_laneKFCntr to the power of 7*/
        dXPow7_laneKFCntr = dXPow6_laneKFCntr * dX_laneKFCntr;
        /*calculate dX_laneKFCntr to the power of 8*/
        dXPow8_laneKFCntr = dXPow7_laneKFCntr * dX_laneKFCntr;

        /*row 0: 1 dX_laneKFCntr 1/2*dX_laneKFCntr^2 1/6*dX_laneKFCntr^3*/
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 0, 0) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 0, 1) = dX_laneKFCntr;
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 0, 2) =
            0.5f * dXPow2_laneKFCntr;
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 0, 3) =
            1.0f / 6.0f * dXPow3_laneKFCntr;
        /*row 1: 0 1 -dX_laneKFCntr - dX_laneKFCntr^2*/
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 1, 1) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 1, 2) = dX_laneKFCntr;
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 1, 3) =
            0.5f * dXPow2_laneKFCntr;
        /*row 2: 0 0 1 dX_laneKFCntr*/
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 2, 2) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 2, 3) = dX_laneKFCntr;
        /*row 3: 0 0 0 1*/
        TUE_CML_GetMatrixElement_M(A_laneKFCntr, 3, 3) = 1.0f;

        /*calculate sigma squared*/
        sigmaSqr_laneKFCntr =
            kappa2diff_sigma_laneKFCntr * kappa2diff_sigma_laneKFCntr;

        /*calculate process noise covariance matrix Q_laneKFCntr elements*/
        Q00_laneKFCntr = dXPow8_laneKFCntr * sigmaSqr_laneKFCntr / 576.0f;
        Q01_laneKFCntr = dXPow7_laneKFCntr * sigmaSqr_laneKFCntr / 48.0f;
        Q02_laneKFCntr = dXPow6_laneKFCntr * sigmaSqr_laneKFCntr / 48.0f;
        Q03_laneKFCntr = dXPow5_laneKFCntr * sigmaSqr_laneKFCntr / 24.0f;

        Q11_laneKFCntr = dXPow6_laneKFCntr * sigmaSqr_laneKFCntr / 4.0f;
        Q12_laneKFCntr = dXPow5_laneKFCntr * sigmaSqr_laneKFCntr / 4.0f;
        Q13_laneKFCntr = dXPow4_laneKFCntr * sigmaSqr_laneKFCntr / 2.0f;

        Q22_laneKFCntr = dXPow4_laneKFCntr * sigmaSqr_laneKFCntr / 4.0f;
        Q23_laneKFCntr = dXPow3_laneKFCntr * sigmaSqr_laneKFCntr / 2.0f;

        Q33_laneKFCntr = dXPow2_laneKFCntr * sigmaSqr_laneKFCntr;

        /*set process noise covariance matrix Q_laneKFCntr row 0*/
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 0, 0) =
            P_ABPLBP_LaneKFDynDistYFact_nu *
            Q00_laneKFCntr;  // Factor needed to increase the dynamic of DistY
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 0, 1) = Q01_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 0, 2) = Q02_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 0, 3) = Q03_laneKFCntr;
        /*set process noise covariance matrix Q_laneKFCntr row 1*/
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 1, 0) = Q01_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 1, 1) =
            P_ABPLBP_LaneKFDynYawFactor_nu * Q11_laneKFCntr;  // Factor needed
                                                              // to increase the
                                                              // dynamic of
                                                              // HeadingAngle
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 1, 2) = Q12_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 1, 3) = Q13_laneKFCntr;
        /*set process noise covariance matrix Q_laneKFCntr row 2*/
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 2, 0) = Q02_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 2, 1) = Q12_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 2, 2) =
            P_ABPLBP_LaneKFDynCrvFactor_nu *
            Q22_laneKFCntr;  // Factor needed to increase the dynamic of Kappa
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 2, 3) = Q23_laneKFCntr;
        /*set process noise covariance matrix Q_laneKFCntr row 3*/
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 3, 0) = Q03_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 3, 1) = Q13_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 3, 2) = Q23_laneKFCntr;
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 3, 3) =
            P_ABPLBP_LaneKFDynCrvRtFactor_nu * Q33_laneKFCntr;

        /*additional noise caused by vehicle movement*/
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 0, 0) +=
            (dT_laneKFCntr * dX_laneKFCntr * vehYawRateStdDev_laneKFCntr) *
            (dT_laneKFCntr * dX_laneKFCntr * vehYawRateStdDev_laneKFCntr);
        TUE_CML_GetMatrixElement_M(Q_laneKFCntr, 1, 1) +=
            (dT_laneKFCntr * vehYawRateStdDev_laneKFCntr) *
            (dT_laneKFCntr * vehYawRateStdDev_laneKFCntr);

        /*initialize steering matrix*/

        TUE_CML_InitMatrix_M(B_laneKFCntr, STATE_LENGTH_LANEKF, 1, 0.0f);
        TUE_CML_GetMatrixElement_M(B_laneKFCntr, 0, 0) =
            -dT_laneKFCntr * dX_laneKFCntr;
        TUE_CML_GetMatrixElement_M(B_laneKFCntr, 1, 0) = -dT_laneKFCntr;
        /*AX_laneKFCntr*/
        TUE_CML_MutiplyMatrices_M(AX_laneKFCntr, A_laneKFCntr, x_laneKFCntr);
        /*Bu*/
        TUE_CML_ScaleMatrix_M(B_laneKFCntr, vehYawRate);
        /*x_laneKFCntr = AX_laneKFCntr + Bu*/
        TUE_CML_AddMatrices_M(x_laneKFCntr, AX_laneKFCntr, B_laneKFCntr);
        /*A_laneKFCntr'*/
        TUE_CML_TransposeMatrix_M(A_trans_laneKFCntr, A_laneKFCntr);
        /*PA'*/
        TUE_CML_MutiplyMatrices_M(PA_trans_laneKFCntr, P_laneKFCntr,
                                  A_trans_laneKFCntr);
        /*APA'*/
        TUE_CML_MutiplyMatrices_M(APA_trans_laneKFCntr, A_laneKFCntr,
                                  PA_trans_laneKFCntr);
        /*P_laneKFCntr=APA'+Q_laneKFCntr*/
        TUE_CML_AddMatrices_M(P_laneKFCntr, APA_trans_laneKFCntr, Q_laneKFCntr);

        /*Measurement Weight*/
        measWeight_laneKFCntr = 0.0f;
        status_laneKFCntr = 3u;
    }
}
/********************************init_laneKFCntr*************************************************
        @fn            init_laneKFCntr
        @brief         Initialization of Kalman filter for center lane filter
        @description   Refer to Kalman filter algorithm for details
        @param[in]     inputs  :
        @param[out]    outputs :
        @startuml
        title init_laneKFCntr
        (*)-->init_laneKFCntr
         -->(*)
        @enduml
**************************************************************************************************/
static void init_laneKFCntr(const TUE_CML_sMatrix_t *z_laneKFCntr,
                            TUE_CML_sMatrix_t *R_laneKFCntr,
                            REAL32_T quality,
                            TUE_CML_sMatrix_t *x_laneKFCntr,
                            TUE_CML_sMatrix_t *P_laneKFCntr,
                            UINT8_T P_ABPLBP_LaneKFMnInitQual_perc,
                            REAL32_T P_ABPLBP_LaneKFInitRFactor_nu) {
    TUE_CML_CreateMatrix_M(H_trans_laneKFCntr, STATE_LENGTH_LANEKF,
                           STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(H_laneKFCntr, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(tmp_laneKFCntr, STATE_LENGTH_LANEKF,
                                   STATE_LENGTH_LANEKF)
                TUE_CML_CreateMatrix_M(R_scaled_laneKFCntr, STATE_LENGTH_LANEKF,
                                       STATE_LENGTH_LANEKF)

                    if (quality > P_ABPLBP_LaneKFMnInitQual_perc) {
        /*scale R_laneKFCntr*/
        TUE_CML_InitMatrix_M(R_scaled_laneKFCntr, STATE_LENGTH_LANEKF,
                             STATE_LENGTH_LANEKF, 0.0f);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFCntr, 0, 0) =
            TUE_CML_GetMatrixElement_M(R_laneKFCntr, 0, 0);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFCntr, 1, 1) =
            TUE_CML_GetMatrixElement_M(R_laneKFCntr, 1, 1);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFCntr, 2, 2) =
            TUE_CML_GetMatrixElement_M(R_laneKFCntr, 2, 2);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKFCntr, 3, 3) =
            TUE_CML_GetMatrixElement_M(R_laneKFCntr, 3, 3);
        TUE_CML_ScaleMatrix_M(R_scaled_laneKFCntr,
                              P_ABPLBP_LaneKFInitRFactor_nu);

        /*initialize matrix H_laneKFCntr*/
        TUE_CML_CreateIdentityMatrix_M(H_laneKFCntr, STATE_LENGTH_LANEKF);
        /*initialize matrix H_laneKFCntr'*/
        TUE_CML_TransposeMatrix_M(H_trans_laneKFCntr, H_laneKFCntr);
        /*tmp_laneKFCntr = H_laneKFCntr'R_laneKFCntr*/
        TUE_CML_MutiplyMatrices_M(tmp_laneKFCntr, H_trans_laneKFCntr,
                                  R_scaled_laneKFCntr);
        /*[H_laneKFCntr'R_laneKFCntr]H_laneKFCntr*/
        TUE_CML_MutiplyMatrices_M(P_laneKFCntr, tmp_laneKFCntr, H_laneKFCntr);
        /*fill x_laneKFCntr*/
        TUE_CML_MutiplyMatrices_M(x_laneKFCntr, H_trans_laneKFCntr,
                                  z_laneKFCntr);
        /*fill other states*/
        valid_laneKFCntr = 1U;
        internalQuality_laneKFCntr = 0.0f;
        measWeight_laneKFCntr = 0.0f;
        status_laneKFCntr = 4u;
    }
}

/*****************************************************************************
 MACROS
 *****************************************************************************/
#ifndef FALSE
#define FALSE (0u)
#endif
#ifndef TRUE
#define TRUE (1u)
#endif

/*length of the KF state vector*/
#define STATE_LENGTH_LANEKF (4u)

/*****************************************************************************
 FUNCTIONS
 *****************************************************************************/
/********************************laneKalmanFilter_Right*************************************************
       @fn            laneKalmanFilter_Right
       @brief         Kalman filter for right lane filter
       @description   Refer to Kalman filter algorithm for details
       @param[in]     inputs  :
       @param[out]    outputs :
       @startuml
       title laneKalmanFilter_Right
       (*)-->laneKalmanFilter_Right
        -->(*)
       @enduml
**************************************************************************************************/
void laneKalmanFilter_Right(const laneKFInTypeV3 *inputs,
                            laneKFOutType *outputs) {
    /*state Matrix x_laneKFRi*/
    TUE_CML_CreateMatrix_M(x_laneKFRi, STATE_LENGTH_LANEKF, 1)
        /*covariance matrix P_laneKFRi*/
        TUE_CML_CreateMatrix_M(P_laneKFRi, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
        /*measurement matrix z_laneKFRi*/
        TUE_CML_CreateMatrix_M(z_laneKFRi, STATE_LENGTH_LANEKF, 1)
        /*measurement variance matrix R_laneKFRi*/
        TUE_CML_CreateMatrix_M(R_laneKFRi, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)

            REAL32_T dx_laneKFRi;
    // REAL32_T posY0Diff_laneKFRi;
    /*calculate dx_laneKFRi*/
    dx_laneKFRi = inputs->sf_DeltaT_sec * inputs->sf_VehVelX_mps;
    vehYawRateStdDev_laneKFRi = inputs->sf_VehYawRateStdDev_radps;
    /*geometric dependent model error*/
    kappa2diff_sigma_laneKFRi =
        1 / inputs->sf_LaneKFErrCoeff1_met /
        (inputs->sf_LaneKFErrCoeff2_mps * inputs->sf_DeltaT_sec) /
        (inputs->sf_LaneKFErrCoeff2_mps * inputs->sf_DeltaT_sec);

    /*In case of an lane change allow the PosY0 position to jump in the
     * corresponding cycle - only if kalman filter has been valid*/
    if (status_laneKFRi > 0 && status_laneKFRi < 4) {
        /*check lane change detection*/
        if (inputs->sf_LaneChange_bool) {
            /*a lateral position jump is forced in case of a lane change*/
            TUE_CML_GetMatrixElement_M(x_laneKFRi, 0, 0) = inputs->sf_PosY0_met;
        }
    }

    /*predict_laneKF -  will only be executed if valid_laneKFRi state is set*/
    predict_laneKFRi(inputs->sf_DeltaT_sec, dx_laneKFRi,
                     inputs->sf_VehYawRate_radps, x_laneKFRi, P_laneKFRi,
                     inputs->sf_LaneKFDynDistYFact_nu,
                     inputs->sf_LaneKFDynYawFactor_nu);

    /*initialize measurement vector z_laneKFRi*/
    TUE_CML_GetMatrixElement_M(z_laneKFRi, 0, 0) = inputs->sf_PosY0_met;
    TUE_CML_GetMatrixElement_M(z_laneKFRi, 1, 0) = inputs->sf_HeadingAngle_rad;
    TUE_CML_GetMatrixElement_M(z_laneKFRi, 2, 0) = inputs->sf_Crv_1pm;
    TUE_CML_GetMatrixElement_M(z_laneKFRi, 3, 0) = inputs->sf_CrvChng_1pm2;

    /*initialize R_laneKFRi matrix*/
    TUE_CML_GetMatrixElement_M(R_laneKFRi, 0, 0) =
        inputs->sf_PosY0StdDev_met * inputs->sf_PosY0StdDev_met;
    TUE_CML_GetMatrixElement_M(R_laneKFRi, 1, 1) =
        inputs->sf_HeadingAngleStdDev_rad * inputs->sf_HeadingAngleStdDev_rad;
    TUE_CML_GetMatrixElement_M(R_laneKFRi, 2, 2) =
        inputs->sf_CrvStdDev_1pm * inputs->sf_CrvStdDev_1pm;
    TUE_CML_GetMatrixElement_M(R_laneKFRi, 3, 3) =
        inputs->sf_CrvChngStdDev_1pm2 * inputs->sf_CrvChngStdDev_1pm2;

    /*Init or Update*/
    if (!valid_laneKFRi && !(inputs->sf_DegradedUpdate_bool)) {
        /*initialization - for A_laneKF better initialization the R_laneKF
         * matrix is multiplied with A_laneKF constant (>1)*/
        init_laneKFRi(z_laneKFRi, R_laneKFRi,
                      inputs->sf_OverallMeasurementQuality_perc, x_laneKFRi,
                      P_laneKFRi, inputs->sf_LaneKFMnInitQual_perc,
                      inputs->sf_LaneKFInitRFactor_nu);
    }
    /*Update only if valid AND measurement quality >
       sf_LaneKFMnUpdateQual_perc*/
    else if (valid_laneKFRi && (inputs->sf_OverallMeasurementQuality_perc >=
                                (inputs->sf_LaneKFMnUpdateQual_perc))) {
        if (inputs->sf_DegradedUpdate_bool) {
            /*degraded update_laneKFRi - quality and weight are the 2
             * parameters*/
            update_laneKFRi(z_laneKFRi, R_laneKFRi,
                            inputs->sf_LaneKFDegradeWeight_nu, x_laneKFRi,
                            P_laneKFRi, inputs->sf_LaneKFKGainFac_nu);
        } else {
            /*full update_laneKFRi*/
            update_laneKFRi(z_laneKFRi, R_laneKFRi, 1.0f, x_laneKFRi,
                            P_laneKFRi, inputs->sf_LaneKFKGainFac_nu);
        }
    }
    /*maintenance_laneKFRi*/
    if (!(status_laneKFRi == 4)) {
        maintenance_laneKFRi(
            x_laneKFRi, P_laneKFRi, inputs->sf_LaneKFIncQual_1ps,
            inputs->sf_LaneKFDecQualDeg_1ps, inputs->sf_LaneKFDecQualPred_1ps,
            inputs->sf_DeltaT_sec);
    }
    /*Reset filter if basic lane data are not valid*/
    if (!inputs->sf_LaneDataValid_bool) reset_laneKFRi(x_laneKFRi, P_laneKFRi);
    /*Set outputs*/
    outputs->sf_PosY0_met = TUE_CML_GetMatrixElement_M(x_laneKFRi, 0, 0);
    outputs->sf_HeadingAngle_rad = TUE_CML_GetMatrixElement_M(x_laneKFRi, 1, 0);
    outputs->sf_Crv_1pm = TUE_CML_GetMatrixElement_M(x_laneKFRi, 2, 0);
    outputs->sf_CrvChng_1pm2 = TUE_CML_GetMatrixElement_M(x_laneKFRi, 3, 0);
    outputs->sf_KFStatus_btf = status_laneKFRi;
    outputs->sf_QualityMeasure_perc = (UINT8_T)(internalQuality_laneKFRi);
}

/********************************reset_laneKFRi*************************************************
   @fn            reset_laneKFRi
   @brief         Reset of Kalman filter for right lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title reset_laneKFRi
   (*)-->reset_laneKFRi
        -->(*)
   @enduml
**************************************************************************************************/
static void reset_laneKFRi(TUE_CML_sMatrix_t *x_laneKFRi,
                           TUE_CML_sMatrix_t *P_laneKFRi) {
    /*reset_laneKFRi parameters*/
    TUE_CML_InitMatrix_M(x_laneKFRi, STATE_LENGTH_LANEKF, 1, 0.0f);
    TUE_CML_InitMatrix_M(P_laneKFRi, STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF,
                         0.0f);
    valid_laneKFRi = FALSE;
    internalQuality_laneKFRi = 0.0f;
    measWeight_laneKFRi = 0.0f;
    status_laneKFRi = 5u;
}
/********************************reset_laneKFRi*************************************************
   @fn            Maintenance_laneKFRi
   @brief         Reset of Kalman filter for right lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title reset_laneKFRi
   (*)-->reset_laneKFRi
        -->(*)
   @enduml
**************************************************************************************************/
static void maintenance_laneKFRi(TUE_CML_sMatrix_t *x_laneKFRi,
                                 TUE_CML_sMatrix_t *P_laneKFRi,
                                 REAL32_T P_ABPLBP_LaneKFIncQual_1ps,
                                 REAL32_T P_ABPLBP_LaneKFDecQualDeg_1ps,
                                 REAL32_T P_ABPLBP_LaneKFDecQualPred_1ps,
                                 REAL32_T deltaT_sec) {
    if (valid_laneKFRi) {
        /*Full Update, Degraded Update or No Update (predict only)? */
        if (measWeight_laneKFRi > 0.9f) {
            /* Full Update */
            status_laneKFRi = 1u;
            /* Increase internal quality by 10% -> 10 cycles are needed to
             * increase the quality from 0% to 100% */
            /* Increase internal quality by P_ABPLBP_LaneKFIncQual_1ps per
             * second*/
            internalQuality_laneKFRi += P_ABPLBP_LaneKFIncQual_1ps * deltaT_sec;
        } else if (measWeight_laneKFRi > 0.0f) {
            /* Degraded Update */
            status_laneKFRi = 2u;
            /* Decrease internal quality by P_ABPLBP_LaneKFDecQualDeg_1ps per
             * seconds */
            internalQuality_laneKFRi -=
                P_ABPLBP_LaneKFDecQualDeg_1ps * deltaT_sec;
        } else {
            /* No Update - Predict Only */
            status_laneKFRi = 3u;
            /* Decrease internal quality by P_ABPLBP_LaneKFDecQualPred_1ps per
             * seconds */
            internalQuality_laneKFRi -=
                P_ABPLBP_LaneKFDecQualPred_1ps * deltaT_sec;
        }

        /* Boundary check: 0 <= internalQuality <= 100 */
        if (internalQuality_laneKFRi < 0.0f) {
            internalQuality_laneKFRi = 0.0f;
            reset_laneKFRi(x_laneKFRi, P_laneKFRi);
        } else if (internalQuality_laneKFRi > 100.0f) {
            internalQuality_laneKFRi = 100.0f;
        }
    } else {
        status_laneKFRi = 0u;
    }
}
/********************************update_laneKFRi*************************************************
   @fn            update_laneKFRi
   @brief         Update of Kalman filter for right lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title update_laneKFRi
   (*)-->update_laneKFRi
        -->(*)
   @enduml
**************************************************************************************************/
static void update_laneKFRi(const TUE_CML_sMatrix_t *z_laneKFRi,
                            const TUE_CML_sMatrix_t *R_laneKFRi,
                            REAL32_T weight,
                            TUE_CML_sMatrix_t *x_laneKFRi,
                            TUE_CML_sMatrix_t *P_laneKFRi,
                            REAL32_T P_ABPLBP_LaneKFKGainFac_nu) {
    /*temporary matrices*/
    TUE_CML_CreateMatrix_M(
        I_laneKF, STATE_LENGTH_LANEKF,
        STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(H_laneKF,
                                                    STATE_LENGTH_LANEKF,
                                                    STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(
            H_trans_laneKF, STATE_LENGTH_LANEKF,
            STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(K_laneKF,
                                                        STATE_LENGTH_LANEKF,
                                                        STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(
                PH_trans_laneKF, STATE_LENGTH_LANEKF,
                STATE_LENGTH_LANEKF) TUE_CML_CreateMatrix_M(HPH_trans_laneKF,
                                                            STATE_LENGTH_LANEKF,
                                                            STATE_LENGTH_LANEKF)
                TUE_CML_CreateMatrix_M(HPH_transPlusR_laneKFRi,
                                       STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF)
                    TUE_CML_CreateMatrix_M(HPH_transPlusR_inv_laneKF,
                                           STATE_LENGTH_LANEKF,
                                           STATE_LENGTH_LANEKF)
                        TUE_CML_CreateMatrix_M(
                            Hx_laneKFRi, STATE_LENGTH_LANEKF,
                            1) TUE_CML_CreateMatrix_M(ZSubHx_laneKFRi,
                                                      STATE_LENGTH_LANEKF, 1)
                            TUE_CML_CreateMatrix_M(KZSubHx_laneKFRi,
                                                   STATE_LENGTH_LANEKF, 1)
                                TUE_CML_CreateMatrix_M(KH_laneKF,
                                                       STATE_LENGTH_LANEKF,
                                                       STATE_LENGTH_LANEKF)
                                    TUE_CML_CreateMatrix_M(ISubKH_laneKF,
                                                           STATE_LENGTH_LANEKF,
                                                           STATE_LENGTH_LANEKF)
                                        TUE_CML_CreateMatrix_M(
                                            P_tmP_laneKFRi, STATE_LENGTH_LANEKF,
                                            STATE_LENGTH_LANEKF)

        /*update_laneKF state with measurement*/
        TUE_CML_CreateIdentityMatrix_M(I_laneKF, STATE_LENGTH_LANEKF);
    TUE_CML_CreateIdentityMatrix_M(H_laneKF, STATE_LENGTH_LANEKF);
    TUE_CML_TransposeMatrix_M(H_trans_laneKF, H_laneKF);
    /*PH'*/
    TUE_CML_MutiplyMatrices_M(PH_trans_laneKF, P_laneKFRi, H_trans_laneKF);
    /*H_laneKF[PH']*/
    TUE_CML_MutiplyMatrices_M(HPH_trans_laneKF, H_laneKF, PH_trans_laneKF);
    /*[HPH']+R_laneKFRi*/
    TUE_CML_AddMatrices_M(HPH_transPlusR_laneKFRi, HPH_trans_laneKF,
                          R_laneKFRi);
    /*inv([HPH'+R_laneKFRi]*/
    TUE_CML_InvertMatrix_M(HPH_transPlusR_inv_laneKF, HPH_transPlusR_laneKFRi);
    /*if inversion fails num. of cols and rows are set to 0*/
    if (HPH_transPlusR_inv_laneKF->Desc.col > 0) {
        /*K_laneKF=[PH'][inv(HPH'+R_laneKFRi)]*/
        TUE_CML_MutiplyMatrices_M(K_laneKF, PH_trans_laneKF,
                                  HPH_transPlusR_inv_laneKF);
        /*K_laneKF=K_laneKF*weight*/
        TUE_CML_ScaleMatrix_M(K_laneKF, weight);

        /* if degradedUpdate == true -> Set first row of K_laneKF to
         * FACTOR*K_laneKF -> weight of Disty = FACTOR*weight */
        if (weight < 1.0f) {
            /*set process noise covariance matrix Q_laneKF row 0*/
            TUE_CML_GetMatrixElement_M(K_laneKF, 0, 0) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKF, 0, 0);
            TUE_CML_GetMatrixElement_M(K_laneKF, 0, 1) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKF, 0, 1);
            TUE_CML_GetMatrixElement_M(K_laneKF, 0, 2) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKF, 0, 2);
            TUE_CML_GetMatrixElement_M(K_laneKF, 0, 3) =
                P_ABPLBP_LaneKFKGainFac_nu *
                TUE_CML_GetMatrixElement_M(K_laneKF, 0, 3);
        }

        /*Hx_laneKFRi*/
        TUE_CML_MutiplyMatrices_M(Hx_laneKFRi, H_laneKF, x_laneKFRi);
        /*z_laneKFRi-[Hx_laneKFRi]*/
        TUE_CML_SubtractMatrices_M(ZSubHx_laneKFRi, z_laneKFRi, Hx_laneKFRi);
        /*K_laneKF[(z_laneKFRi-Hx_laneKFRi)]*/
        TUE_CML_MutiplyMatrices_M(KZSubHx_laneKFRi, K_laneKF, ZSubHx_laneKFRi);
        /*x_laneKFRi=x_laneKFRi+K_laneKF(z_laneKFRi-Hx_laneKFRi)*/
        TUE_CML_AddMatrices_M(x_laneKFRi, x_laneKFRi, KZSubHx_laneKFRi);
        /*KH_laneKF*/
        TUE_CML_MutiplyMatrices_M(KH_laneKF, K_laneKF, H_laneKF);
        /*I_laneKF-[KH_laneKF]*/
        TUE_CML_SubtractMatrices_M(ISubKH_laneKF, I_laneKF, KH_laneKF);
        /*P_laneKFRi=[(I_laneKF-K_laneKF*H_laneKF)]P_laneKFRi*/
        TUE_CML_MutiplyMatrices_M(P_tmP_laneKFRi, ISubKH_laneKF, P_laneKFRi);
        TUE_CML_CopyMatrix_M(P_laneKFRi, P_tmP_laneKFRi);

        measWeight_laneKFRi = weight;
    } else {
        reset_laneKFRi(x_laneKFRi, P_laneKFRi);
    }
}
/********************************predict_laneKFRi*************************************************
   @fn            predict_laneKFRi
   @brief         Prediction of Kalman filter for right lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title predict_laneKFRi
   (*)-->predict_laneKFRi
        -->(*)
   @enduml
**************************************************************************************************/
static void predict_laneKFRi(REAL32_T dT_laneKFRi,
                             REAL32_T dx_laneKFRi,
                             REAL32_T vehYawRate,
                             TUE_CML_sMatrix_t *x_laneKFRi,
                             TUE_CML_sMatrix_t *P_laneKFRi,
                             REAL32_T P_ABPLBP_LaneKFDynDistYFact_nu,
                             REAL32_T P_ABPLBP_LaneKFDynYawFactor_nu) {
    /*initialize local matrices and variables*/
    TUE_CML_CreateMatrix_M(A_laneKF, STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(Q_laneKF, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(B_laneKF, STATE_LENGTH_LANEKF, 1)
                TUE_CML_CreateMatrix_M(Ax_laneKFRi, STATE_LENGTH_LANEKF, 1)
                    TUE_CML_CreateMatrix_M(A_trans_laneKF, STATE_LENGTH_LANEKF,
                                           STATE_LENGTH_LANEKF)
                        TUE_CML_CreateMatrix_M(PA_trans_laneKF,
                                               STATE_LENGTH_LANEKF,
                                               STATE_LENGTH_LANEKF)
                            TUE_CML_CreateMatrix_M(APA_trans_laneKF,
                                                   STATE_LENGTH_LANEKF,
                                                   STATE_LENGTH_LANEKF)

                                REAL32_T dXPow2_laneKF;
    REAL32_T dXPow3_laneKF;
    REAL32_T dXPow4_laneKF;
    REAL32_T dXPow5_laneKF;
    REAL32_T dXPow6_laneKF;
    REAL32_T dXPow7_laneKF;
    REAL32_T dXPow8_laneKF;
    REAL32_T Q00_laneKF;
    REAL32_T Q01_laneKF;
    REAL32_T Q02_laneKF;
    REAL32_T Q03_laneKF;
    REAL32_T Q11_laneKF;
    REAL32_T Q12_laneKF;
    REAL32_T Q13_laneKF;
    REAL32_T Q22_laneKF;
    REAL32_T Q23_laneKF;
    REAL32_T Q33_laneKF;
    REAL32_T sigmaSqR_laneKFRi;

    if (valid_laneKFRi) {
        /*initialize system matrix A_laneKF*/
        TUE_CML_InitMatrix_M(A_laneKF, STATE_LENGTH_LANEKF, STATE_LENGTH_LANEKF,
                             0.0f);
        /*calculate dx_laneKFRi to the power of 2*/
        dXPow2_laneKF = dx_laneKFRi * dx_laneKFRi;
        /*calculate dx_laneKFRi to the power of 3*/
        dXPow3_laneKF = dXPow2_laneKF * dx_laneKFRi;
        /*calculate dx_laneKFRi to the power of 4*/
        dXPow4_laneKF = dXPow3_laneKF * dx_laneKFRi;
        /*calculate dx_laneKFRi to the power of 5*/
        dXPow5_laneKF = dXPow4_laneKF * dx_laneKFRi;
        /*calculate dx_laneKFRi to the power of 6*/
        dXPow6_laneKF = dXPow5_laneKF * dx_laneKFRi;
        /*calculate dx_laneKFRi to the power of 7*/
        dXPow7_laneKF = dXPow6_laneKF * dx_laneKFRi;
        /*calculate dx_laneKFRi to the power of 8*/
        dXPow8_laneKF = dXPow7_laneKF * dx_laneKFRi;

        /*row 0: 1 dx_laneKFRi 1/2*dx_laneKFRi^2 1/6*dx_laneKFRi^3*/
        TUE_CML_GetMatrixElement_M(A_laneKF, 0, 0) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKF, 0, 1) = dx_laneKFRi;
        TUE_CML_GetMatrixElement_M(A_laneKF, 0, 2) = 0.5f * dXPow2_laneKF;
        TUE_CML_GetMatrixElement_M(A_laneKF, 0, 3) =
            1.0f / 6.0f * dXPow3_laneKF;
        /*row 1: 0 1 -dx_laneKFRi - dx_laneKFRi^2*/
        TUE_CML_GetMatrixElement_M(A_laneKF, 1, 1) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKF, 1, 2) = dx_laneKFRi;
        TUE_CML_GetMatrixElement_M(A_laneKF, 1, 3) = 0.5f * dXPow2_laneKF;
        /*row 2: 0 0 1 dx_laneKFRi*/
        TUE_CML_GetMatrixElement_M(A_laneKF, 2, 2) = 1.0f;
        TUE_CML_GetMatrixElement_M(A_laneKF, 2, 3) = dx_laneKFRi;
        /*row 3: 0 0 0 1*/
        TUE_CML_GetMatrixElement_M(A_laneKF, 3, 3) = 1.0f;

        /*calculate sigma squared*/
        sigmaSqR_laneKFRi =
            kappa2diff_sigma_laneKFRi * kappa2diff_sigma_laneKFRi;

        /*calculate process noise covariance matrix Q_laneKF elements*/
        Q00_laneKF = dXPow8_laneKF * sigmaSqR_laneKFRi / 576.0f;
        Q01_laneKF = (dXPow7_laneKF * sigmaSqR_laneKFRi) / 48.0f;
        Q02_laneKF = dXPow6_laneKF * sigmaSqR_laneKFRi / 48.0f;
        Q03_laneKF = dXPow5_laneKF * sigmaSqR_laneKFRi / 24.0f;

        Q11_laneKF = dXPow6_laneKF * sigmaSqR_laneKFRi / 4.0f;
        Q12_laneKF = dXPow5_laneKF * sigmaSqR_laneKFRi / 4.0f;
        Q13_laneKF = dXPow4_laneKF * sigmaSqR_laneKFRi / 2.0f;

        Q22_laneKF = dXPow4_laneKF * sigmaSqR_laneKFRi / 4.0f;
        Q23_laneKF = dXPow3_laneKF * sigmaSqR_laneKFRi / 2.0f;

        Q33_laneKF = dXPow2_laneKF * sigmaSqR_laneKFRi;

        /*set process noise covariance matrix Q_laneKF row 0*/
        TUE_CML_GetMatrixElement_M(Q_laneKF, 0, 0) =
            P_ABPLBP_LaneKFDynDistYFact_nu *
            Q00_laneKF;  // Factor needed to increase the dynamic of DistY
        TUE_CML_GetMatrixElement_M(Q_laneKF, 0, 1) = Q01_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 0, 2) = Q02_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 0, 3) = Q03_laneKF;
        /*set process noise covariance matrix Q_laneKF row 1*/
        TUE_CML_GetMatrixElement_M(Q_laneKF, 1, 0) = Q01_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 1, 1) =
            P_ABPLBP_LaneKFDynYawFactor_nu * Q11_laneKF;  // Factor needed to
                                                          // increase the
                                                          // dynamic of
                                                          // HeadingAngle
        TUE_CML_GetMatrixElement_M(Q_laneKF, 1, 2) = Q12_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 1, 3) = Q13_laneKF;
        /*set process noise covariance matrix Q_laneKF row 2*/
        TUE_CML_GetMatrixElement_M(Q_laneKF, 2, 0) = Q02_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 2, 1) = Q12_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 2, 2) =
            100.0f *
            Q22_laneKF;  // Factor needed to increase the dynamic of Kappa
        TUE_CML_GetMatrixElement_M(Q_laneKF, 2, 3) = Q23_laneKF;
        /*set process noise covariance matrix Q_laneKF row 3*/
        TUE_CML_GetMatrixElement_M(Q_laneKF, 3, 0) = Q03_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 3, 1) = Q13_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 3, 2) = Q23_laneKF;
        TUE_CML_GetMatrixElement_M(Q_laneKF, 3, 3) = 2.0f * Q33_laneKF;

        /*additional noise caused by vehicle movement*/
        TUE_CML_GetMatrixElement_M(Q_laneKF, 0, 0) +=
            (dT_laneKFRi * dx_laneKFRi * vehYawRateStdDev_laneKFRi) *
            (dT_laneKFRi * dx_laneKFRi * vehYawRateStdDev_laneKFRi);
        TUE_CML_GetMatrixElement_M(Q_laneKF, 1, 1) +=
            (dT_laneKFRi * vehYawRateStdDev_laneKFRi) *
            (dT_laneKFRi * vehYawRateStdDev_laneKFRi);

        /*initialize steering matrix*/

        TUE_CML_InitMatrix_M(B_laneKF, STATE_LENGTH_LANEKF, 1, 0.0f);
        TUE_CML_GetMatrixElement_M(B_laneKF, 0, 0) = -dT_laneKFRi * dx_laneKFRi;
        TUE_CML_GetMatrixElement_M(B_laneKF, 1, 0) = -dT_laneKFRi;
        /*Ax_laneKFRi*/
        TUE_CML_MutiplyMatrices_M(Ax_laneKFRi, A_laneKF, x_laneKFRi);
        /*Bu*/
        TUE_CML_ScaleMatrix_M(B_laneKF, vehYawRate);
        /*x_laneKFRi = Ax_laneKFRi + Bu*/
        TUE_CML_AddMatrices_M(x_laneKFRi, Ax_laneKFRi, B_laneKF);
        /*A_laneKF'*/
        TUE_CML_TransposeMatrix_M(A_trans_laneKF, A_laneKF);
        /*PA'*/
        TUE_CML_MutiplyMatrices_M(PA_trans_laneKF, P_laneKFRi, A_trans_laneKF);
        /*APA'*/
        TUE_CML_MutiplyMatrices_M(APA_trans_laneKF, A_laneKF, PA_trans_laneKF);
        /*P_laneKFRi=APA'+Q_laneKF*/
        TUE_CML_AddMatrices_M(P_laneKFRi, APA_trans_laneKF, Q_laneKF);

        /*Measurement Weight*/
        measWeight_laneKFRi = 0.0f;
        status_laneKFRi = 3u;
    }
}
/********************************init_laneKFRi*************************************************
   @fn            init_laneKFRi
   @brief         Initialization of Kalman filter for right lane filter
   @description   Refer to Kalman filter algorithm for details
   @param[in]     inputs  :
   @param[out]    outputs :
   @startuml
   title init_laneKFRi
   (*)-->init_laneKFRi
        -->(*)
   @enduml
**************************************************************************************************/
static void init_laneKFRi(const TUE_CML_sMatrix_t *z_laneKFRi,
                          TUE_CML_sMatrix_t *R_laneKFRi,
                          REAL32_T quality,
                          TUE_CML_sMatrix_t *x_laneKFRi,
                          TUE_CML_sMatrix_t *P_laneKFRi,
                          UINT8_T P_ABPLBP_LaneKFMnInitQual_perc,
                          REAL32_T P_ABPLBP_LaneKFInitRFactor_nu) {
    TUE_CML_CreateMatrix_M(H_trans_laneKF, STATE_LENGTH_LANEKF,
                           STATE_LENGTH_LANEKF)
        TUE_CML_CreateMatrix_M(H_laneKF, STATE_LENGTH_LANEKF,
                               STATE_LENGTH_LANEKF)
            TUE_CML_CreateMatrix_M(tmP_laneKFRi, STATE_LENGTH_LANEKF,
                                   STATE_LENGTH_LANEKF)
                TUE_CML_CreateMatrix_M(R_scaled_laneKF, STATE_LENGTH_LANEKF,
                                       STATE_LENGTH_LANEKF)

                    if (quality > P_ABPLBP_LaneKFMnInitQual_perc) {
        /*scale R_laneKFRi*/
        TUE_CML_InitMatrix_M(R_scaled_laneKF, STATE_LENGTH_LANEKF,
                             STATE_LENGTH_LANEKF, 0.0f);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKF, 0, 0) =
            TUE_CML_GetMatrixElement_M(R_laneKFRi, 0, 0);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKF, 1, 1) =
            TUE_CML_GetMatrixElement_M(R_laneKFRi, 1, 1);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKF, 2, 2) =
            TUE_CML_GetMatrixElement_M(R_laneKFRi, 2, 2);
        TUE_CML_GetMatrixElement_M(R_scaled_laneKF, 3, 3) =
            TUE_CML_GetMatrixElement_M(R_laneKFRi, 3, 3);
        TUE_CML_ScaleMatrix_M(R_scaled_laneKF, P_ABPLBP_LaneKFInitRFactor_nu);

        /*initialize matrix H_laneKF*/
        TUE_CML_CreateIdentityMatrix_M(H_laneKF, STATE_LENGTH_LANEKF);
        /*initialize matrix H_laneKF'*/
        TUE_CML_TransposeMatrix_M(H_trans_laneKF, H_laneKF);
        /*tmP_laneKFRi = H_laneKF'R_laneKFRi*/
        TUE_CML_MutiplyMatrices_M(tmP_laneKFRi, H_trans_laneKF,
                                  R_scaled_laneKF);
        /*[H_laneKF'R_laneKFRi]H_laneKF*/
        TUE_CML_MutiplyMatrices_M(P_laneKFRi, tmP_laneKFRi, H_laneKF);
        /*fill x_laneKFRi*/
        TUE_CML_MutiplyMatrices_M(x_laneKFRi, H_trans_laneKF, z_laneKFRi);
        /*fill other states*/
        valid_laneKFRi = 1U;
        internalQuality_laneKFRi = 0.0f;
        measWeight_laneKFRi = 0.0f;
        status_laneKFRi = 4u;
    }
}

/****************************3.Lane filter process
 * ***************************************/
/****************************************************************************************
        @fn           INPUT_LaneWidthValidity
        @brief        Input for LaneWidthValidity
        @description  LaneWidthValidity input interface
        @param[in]    pLFPInput : Input for ELG
        @param[in]    pLFPParam : Parameter for ELG
        @param[in]    pCFVOutput : Parameter for ELG
        @param[out]   pELGOutput: Output for ELG
        @param[out]   pELGDebug : Debug(measurement) for ELG
        @return       void
        @startuml
        title EgoLaneGeneration
        (*)-->1.ULP(UncoupledLaneProcessing)
           --> (*)
        (*)-->2.CLP(CheckBasicLaneBoundaryProperites)
           -->(*)
        (*)-->3.LFP(AnyBoundaryFilteringAndPlausibilization)
           -->(*)
        @enduml
 ******************************************************************************************/
void INPUT_LaneWidthValidity(const sLFPInput_t *pLFPInput,
                             const sLFPParam_t *pLFPParam,
                             const sCFVOutput_t *pCFVOutput,
                             sLWVInput_t *pLWVInput,
                             sLWVParam_t *PLWVParam) {
    pLWVInput->fVehVelX = pLFPInput->fVehVelX;
    pLWVInput->bNewCorridorValid = pLFPInput->bNewCorridorValid;
    pLWVInput->bUpDownHillDegrade = pLFPInput->bUpDownHillDegrade;
    pLWVInput->bBridgePossible = pCFVOutput->bPossibleLaneBridgeCntr;
    pLWVInput->fPosY0Lf = pLFPInput->fPosY0Lf;
    pLWVInput->fHeadingLf = pLFPInput->fHeadingLf;
    pLWVInput->uQualityLf = pLFPInput->uQualityLf;
    pLWVInput->uRangeCheckQualifierLf = pLFPInput->uRangeCheckQualifierLf;
    pLWVInput->bNotAvailableLf = pLFPInput->bNotAvailableLf;
    pLWVInput->bDistYStepDtctLf = pLFPInput->bDistYStepDtctLf;
    pLWVInput->bLengthInvalidLf = pLFPInput->bLengthInvalidLf;
    pLWVInput->bBridgeUnCplLf = pLFPInput->bBridgeUnCplLf;
    pLWVInput->bQualityNotValidLf = pLFPInput->bQualityNotValidLf;
    pLWVInput->fPosY0Ri = pLFPInput->fPosY0Ri;
    pLWVInput->fHeadingRi = pLFPInput->fHeadingRi;
    pLWVInput->uQualityRi = pLFPInput->uQualityRi;
    pLWVInput->uRangeCheckQualifierRi = pLFPInput->uRangeCheckQualifierRi;
    pLWVInput->bNotAvailableRi = pLFPInput->bNotAvailableRi;
    pLWVInput->bDistYStepDtctRi = pLFPInput->bDistYStepDtctRi;
    pLWVInput->bLengthInvalidRi = pLFPInput->bLengthInvalidRi;
    pLWVInput->bBridgeUnCplRi = pLFPInput->bBridgeUnCplRi;
    pLWVInput->bQualityNotValidRi = pLFPInput->bQualityNotValidRi;
    pLWVInput->bLaneChangeDtct = pLFPInput->bLaneChangeDtct;

    /*  */
    PLWVParam->uBitMaskRangeCheckLf = 341U;
    PLWVParam->uBitMaskRangeCheckRi = 682U;
    PLWVParam->fTiBridgeByVirtual = 0.2F;
    PLWVParam->fMaxBridgeDistance = 20.0F;
    PLWVParam->fMaxBridgeTime = 255.0F;
    PLWVParam->fDefaultLaneWidth = 2.5F;
    PLWVParam->fLowPassTimeConst = 5.0F;
    PLWVParam->fMaxLaneWidth = 4.8F;
    PLWVParam->fMaxLaneWidthHyst = 0.2F;
    PLWVParam->fMinLaneWidth = 2.5F;
    PLWVParam->fMinLaneWidthHyst = 0.0F;
    PLWVParam->fSysCycleTime = pLFPParam->fSysCycleTime;
}

void LWV_CheckLaneValidity(sLWVInput_t const *pLWVInput,
                           sLWVParam_t const *pLWVParam,
                           sLWVDebug_t *pLWVDebug,
                           UINT8_T *bLaneValidLf,
                           UINT8_T *bLaneValidRi) {
    UINT8_T bValidByRangeLf = 0U;
    UINT8_T bValidByRangeRi = 0U;

    /********************CheckLaneValidity*****************/
    if ((pLWVInput->uRangeCheckQualifierLf & pLWVParam->uBitMaskRangeCheckLf) ==
        0U) {
        bValidByRangeLf = 1U;
    } else {
        bValidByRangeLf = 0U;
    }

    if ((bValidByRangeLf == 1U) && (pLWVInput->bNotAvailableLf == 0U) &&
        (pLWVInput->bDistYStepDtctLf == 0U) &&
        (pLWVInput->bLengthInvalidLf == 0U) &&
        (pLWVInput->bBridgeUnCplRi == 0U) &&
        (pLWVInput->bQualityNotValidLf == 0U)) {
        *bLaneValidLf = 1U;
    } else {
        *bLaneValidLf = 0U;
    }

    if ((pLWVInput->uRangeCheckQualifierRi & pLWVParam->uBitMaskRangeCheckRi) ==
        0U) {
        bValidByRangeRi = 1U;
    } else {
        bValidByRangeRi = 0U;
    }

    if ((bValidByRangeRi == 1U) && (pLWVInput->bNotAvailableRi == 0U) &&
        (pLWVInput->bDistYStepDtctRi == 0U) &&
        (pLWVInput->bLengthInvalidRi == 0U) &&
        (pLWVInput->bBridgeUnCplLf == 0U) &&
        (pLWVInput->bQualityNotValidRi == 0U)) {
        *bLaneValidRi = 1U;
    } else {
        *bLaneValidRi = 0U;
    }

    pLWVDebug->bValidByRangeLf = bValidByRangeLf;
    pLWVDebug->bValidByRangeRi = bValidByRangeRi;
}

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
                           UINT8_T *bLaneBridgeRi) {
    UINT8_T bLaneVirtualLf = 0U;
    UINT8_T bLaneVirtualRi = 0U;
    UINT8_T bBridgeByVirtualLf = 0U;
    UINT8_T bBridgeByVirtualRi = 0U;
    UINT8_T bRawLaneBridgeLf = 0U;
    UINT8_T bRawLaneBridgeRi = 0U;
    REAL32_T fBridgeDistanceLf = 0.0F;
    REAL32_T fBridgeDistanceRi = 0.0F;
    UINT8_T bEnableByDistanceLf = 0U;
    UINT8_T bEnableByDistanceRi = 0U;
    REAL32_T fBridgeTimeLf = 0.0F;
    REAL32_T fBridgeTimeRi = 0.0F;
    UINT8_T bEnableByTimeLf = 0U;
    UINT8_T bEnableByTimeRi = 0U;
    /**********************CheckLaneBridging*******************/
    /* Lane bridging will be applied if:
            1.Lane is not LD virtual lane (time constraint is considered);
            2.lane bridging is possible;
            3.lane is valid or the lane has been bridged in the previous step;
            4.Up Down Hill Degrade is not enable.
    */
    if (pLWVInput->uQualityLf == 1U) {
        bLaneVirtualLf = 1U;
    } else {
        bLaneVirtualLf = 0U;
    }

    bBridgeByVirtualLf = TUE_CML_TurnOnDelay_M(
        bLaneVirtualLf, pLWVParam->fTiBridgeByVirtual, pLWVParam->fSysCycleTime,
        &fTimerBridgeVirtualLf, bLastBridgeByVirtualLf);

    if ((bBridgeByVirtualLf == 0U) && (pLWVInput->bBridgePossible == 1U) &&
        (((bLaneValidLf == 0U) && (bLastLaneValidLf == 1U)) ||
         (bLastLaneBridgeLf == 1U)) &&
        (pLWVInput->bUpDownHillDegrade == 0)) {
        bRawLaneBridgeLf = 1U;
    } else {
        bRawLaneBridgeLf = 0U;
    }

    if (pLWVInput->uQualityRi == 1U) {
        bLaneVirtualRi = 1U;
    } else {
        bLaneVirtualRi = 0U;
    }
    bBridgeByVirtualRi = TUE_CML_TurnOnDelay_M(
        bLaneVirtualRi, pLWVParam->fTiBridgeByVirtual, pLWVParam->fSysCycleTime,
        &fTimerBridgeVirtualRi, bLastBridgeByVirtualRi);

    if ((bBridgeByVirtualRi == 0U) && (pLWVInput->bBridgePossible == 1U) &&
        (((bLaneValidRi == 0U) && (bLastLaneValidRi == 1U)) ||
         (bLastLaneBridgeRi == 1U)) &&
        (pLWVInput->bUpDownHillDegrade == 0U)) {
        bRawLaneBridgeRi = 1U;
    } else {
        bRawLaneBridgeRi = 0U;
    }

    if (LBP_Ku_HZ_CAl_nu) {
        // Left Lane Bridge
        if ((bLastRawLaneBridgeLf == 0) && (bRawLaneBridgeLf == 1)) {
            fBridgeDistanceLf = LBP_Kf_MaxBridgeDist_met;
            fBridgeTimeLf = LBP_Kf_MaxBridgeTime_sec;
        } else {
            fBridgeDistanceLf =
                fLastBridgeDistanceLf -
                TUE_CML_Abs_M(pLWVInput->fVehVelX) * pLWVParam->fSysCycleTime;
            fBridgeDistanceLf = TUE_CML_Limit_M(fBridgeDistanceLf, 0.0F,
                                                LBP_Kf_MaxBridgeDist_met);
            fBridgeTimeLf = fLastBridgeTimeLf - pLWVParam->fSysCycleTime;
            fBridgeTimeLf =
                TUE_CML_Limit_M(fBridgeTimeLf, 0.0F, LBP_Kf_MaxBridgeTime_sec);
        }
        if (fBridgeDistanceLf > 0.0F) {
            bEnableByDistanceLf = 1U;
        } else {
            bEnableByDistanceLf = 0U;
        }
        if (fBridgeTimeLf > 0.0F) {
            bEnableByTimeLf = 1U;
        } else {
            bEnableByTimeLf = 0U;
        }

        // Right Line Bridge
        if ((bLastRawLaneBridgeRi == 0) && (bRawLaneBridgeRi == 1)) {
            fBridgeDistanceRi = LBP_Kf_MaxBridgeDist_met;
            fBridgeTimeRi = LBP_Kf_MaxBridgeTime_sec;
        } else {
            fBridgeDistanceRi =
                fLastBridgeDistanceRi -
                TUE_CML_Abs_M(pLWVInput->fVehVelX) * pLWVParam->fSysCycleTime;
            fBridgeDistanceRi = TUE_CML_Limit_M(fBridgeDistanceRi, 0.0F,
                                                LBP_Kf_MaxBridgeDist_met);
            fBridgeTimeRi = fLastBridgeTimeRi - pLWVParam->fSysCycleTime;
            fBridgeTimeRi =
                TUE_CML_Limit_M(fBridgeTimeRi, 0.0F, LBP_Kf_MaxBridgeTime_sec);
        }
        if (fBridgeDistanceRi > 0.0F) {
            bEnableByDistanceRi = 1U;
        } else {
            bEnableByDistanceRi = 0U;
        }
        if (fBridgeTimeRi > 0.0F) {
            bEnableByTimeRi = 1U;
        } else {
            bEnableByTimeRi = 0U;
        }

        /* CheckLaneBridging */
        if ((bRawLaneBridgeLf == 1U) &&
            ((bEnableByDistanceLf == 1U) && (bEnableByTimeLf == 1U))) {
            *bLaneBridgeLf = 1U;
        } else {
            *bLaneBridgeLf = 0U;
        }

        if ((bRawLaneBridgeRi == 1U) &&
            ((bEnableByDistanceRi == 1U) && (bEnableByTimeRi == 1U))) {
            *bLaneBridgeRi = 1U;
        } else {
            *bLaneBridgeRi = 0U;
        }
    } else {
        /* AllowBridgingForLimitedDistance */
        if ((bLastRawLaneBridgeLf == 0) && (bRawLaneBridgeLf == 1)) {
            fBridgeDistanceLf = pLWVParam->fMaxBridgeDistance;
        } else {
            fBridgeDistanceLf =
                fLastBridgeDistanceLf -
                TUE_CML_Abs_M(pLWVInput->fVehVelX) * pLWVParam->fSysCycleTime;
            fBridgeDistanceLf = TUE_CML_Limit_M(fBridgeDistanceLf, 0.0F,
                                                pLWVParam->fMaxBridgeDistance);
        }

        if (fBridgeDistanceLf > 0.0F) {
            bEnableByDistanceLf = 1U;
        } else {
            bEnableByDistanceLf = 0U;
        }

        if ((bLastRawLaneBridgeRi == 0) && (bRawLaneBridgeRi == 1)) {
            fBridgeDistanceRi = pLWVParam->fMaxBridgeDistance;
        } else {
            fBridgeDistanceRi =
                fLastBridgeDistanceRi -
                TUE_CML_Abs_M(pLWVInput->fVehVelX) * pLWVParam->fSysCycleTime;
            fBridgeDistanceRi = TUE_CML_Limit_M(fBridgeDistanceRi, 0.0F,
                                                pLWVParam->fMaxBridgeDistance);
        }
        if (fBridgeDistanceRi > 0.0F) {
            bEnableByDistanceRi = 1U;
        } else {
            bEnableByDistanceRi = 0U;
        }

        /* AllowBridgingForLimitedTime */
        if ((bLastEnableByDistanceLf == 1) && (fBridgeDistanceLf == 0)) {
            fBridgeTimeLf = pLWVParam->fMaxBridgeTime;
        } else {
            fBridgeTimeLf = fLastBridgeTimeLf - pLWVParam->fSysCycleTime;
            fBridgeTimeLf =
                TUE_CML_Limit_M(fBridgeTimeLf, 0.0F, pLWVParam->fMaxBridgeTime);
        }
        if (fBridgeTimeLf > 0.0F) {
            bEnableByTimeLf = 1U;
        } else {
            bEnableByTimeLf = 0U;
        }

        if ((bLastEnableByDistanceRi == 1) && (fBridgeDistanceRi == 0)) {
            fBridgeTimeRi = pLWVParam->fMaxBridgeTime;
        } else {
            fBridgeTimeRi = fLastBridgeTimeRi - pLWVParam->fSysCycleTime;
            fBridgeTimeRi =
                TUE_CML_Limit_M(fBridgeTimeRi, 0.0F, pLWVParam->fMaxBridgeTime);
        }
        if (fBridgeTimeRi > 0.0F) {
            bEnableByTimeRi = 1U;
        } else {
            bEnableByTimeRi = 0U;
        }

        /* CheckLaneBridging */
        if ((bRawLaneBridgeLf == 1U) &&
            ((bEnableByDistanceLf == 1U) || (bEnableByTimeLf == 1U))) {
            *bLaneBridgeLf = 1U;
        } else {
            *bLaneBridgeLf = 0U;
        }

        if ((bRawLaneBridgeRi == 1U) &&
            ((bEnableByDistanceRi == 1U) || (bEnableByTimeRi == 1U))) {
            *bLaneBridgeRi = 1U;
        } else {
            *bLaneBridgeRi = 0U;
        }
    }

    pLWVDebug->bLaneVirtualLf = bLaneVirtualLf;
    pLWVDebug->bLaneVirtualRi = bLaneVirtualRi;
    pLWVDebug->bBridgeByVirtualLf = bBridgeByVirtualLf;
    pLWVDebug->bBridgeByVirtualRi = bBridgeByVirtualRi;
    pLWVDebug->bRawLaneBridgeLf = bRawLaneBridgeLf;
    pLWVDebug->bRawLaneBridgeRi = bRawLaneBridgeRi;
    pLWVDebug->fBridgeDistanceLf = fBridgeDistanceLf;
    pLWVDebug->fBridgeDistanceRi = fBridgeDistanceRi;
    pLWVDebug->bEnableByDistanceLf = bEnableByDistanceLf;
    pLWVDebug->bEnableByDistanceRi = bEnableByDistanceRi;
    pLWVDebug->fBridgeTimeLf = fBridgeTimeLf;
    pLWVDebug->fBridgeTimeRi = fBridgeTimeRi;
    pLWVDebug->bEnableByTimeLf = bEnableByTimeLf;
    pLWVDebug->bEnableByTimeRi = bEnableByTimeRi;

    bLastBridgeByVirtualLf = bBridgeByVirtualLf;
    bLastBridgeByVirtualRi = bBridgeByVirtualRi;
    bLastRawLaneBridgeLf = bRawLaneBridgeLf;
    bLastRawLaneBridgeRi = bRawLaneBridgeRi;
    fLastBridgeDistanceLf = fBridgeDistanceLf;
    fLastBridgeDistanceRi = fBridgeDistanceRi;
    bLastEnableByDistanceLf = bEnableByDistanceLf;
    bLastEnableByDistanceRi = bEnableByDistanceRi;
    fLastBridgeTimeLf = fBridgeTimeLf;
    fLastBridgeTimeRi = fBridgeTimeRi;
}

/**************************************************
        @fn           LaneWidthValidity(LaneWidthAndValidQualifierDetermination)
        @brief        Lane valid qualifier and lane width, lane width validity
        @description  Lane filter processing:
                                          1.Lane valid qualifier:
                                                  1.1 Check lane validity(From
 CLP);
                                                  1.2 Check lane bridge;
                                                  1.3 Determine lane valid
 qualifier.
                                          2.lane width, lane width validity.
        @param[in]    pLWVInput : Input for LaneWidthValidity
        @param[in]    pLWVParam : Parameter for LaneWidthValidity
        @param[out]   pLWVOutput: Output for LaneWidthValidity
        @param[out]   pLWVDebug : Debug(measurement) for LaneWidthValidity
        @return       void
        @startuml
        title LaneWidthValidity
        (*)-->1.1CheckLaneValidity
           -->1.2CheckLaneBridge
           -->1.3DetermineLaneValidQualifier
           -->2.LaneWidthAndLaneWidthValidity
           -->(*)
        1.1CheckLaneValidity -->1.3DetermineLaneValidQualifier
        @enduml
 ***************************************************/
void LaneWidthValidity(sLWVInput_t const *pLWVInput,
                       sLWVParam_t const *pLWVParam,
                       sLWVOutput_t *pLWVOutput,
                       sLWVDebug_t *pLWVDebug) {
    UINT8_T bLaneValidLf = 0U;
    UINT8_T bLaneValidRi = 0U;
    UINT8_T bLaneBridgeLf = 0U;
    UINT8_T bLaneBridgeRi = 0U;
    UINT8_T bLaneWidthReset = 0U;
    REAL32_T fRawLaneWidth = 0.0F;
    UINT8_T bLowPassReset = 0U;
    REAL32_T fTimeLowPass = 0.0F;
    REAL32_T fRawTimeLowPass = 0.0F;
    REAL32_T fMaxLaneWidth = 0.0F;
    REAL32_T fMinLaneWidth = 0.0F;
    REAL32_T fCoeffLowPass = 0.0F;

    /**********************CheckLaneValidity**************************/
    LWV_CheckLaneValidity(pLWVInput, pLWVParam, pLWVDebug, &bLaneValidLf,
                          &bLaneValidRi);

    /**********************CheckLaneBridging**************************/
    /* Lane bridging will be applied if:
            1.Lane is not LD virtual lane (time constraint is considered);
            2.lane bridging is possible;
            3.lane is valid or the lane has been bridged in the previous step;
            4.Up Down Hill Degrade is not enable.
    */
    LWV_CheckLaneBridging(pLWVInput, pLWVParam, bLaneValidLf, bLastLaneValidLf,
                          bLastLaneBridgeLf, bLaneValidRi, bLastLaneValidRi,
                          bLastLaneBridgeRi, pLWVDebug, &bLaneBridgeLf,
                          &bLaneBridgeRi);

    /********************DetermineLaneValidQualifier******************/
    if ((bLaneValidLf == 1U) && (bLaneValidRi == 1U)) {
        pLWVOutput->uLaneValidQualifer = LANE_BOTH_VALID;
    } else if (bLaneValidLf == 1U) {
        if (bLaneBridgeRi == 1U) {
            pLWVOutput->uLaneValidQualifer = LANE_RIGHT_VIRTUAL;
        } else {
            pLWVOutput->uLaneValidQualifer = LANE_LEFT_VALID;
        }
    } else if (bLaneValidRi == 1U) {
        if (bLaneBridgeLf == 1U) {
            pLWVOutput->uLaneValidQualifer = LANE_LEFT_VIRTUAL;
        } else {
            pLWVOutput->uLaneValidQualifer = LANE_RIGHT_VALID;
        };
    } else {
        pLWVOutput->uLaneValidQualifer = LANE_NO_VALID;
    }

    /**********************LaneWidthDetermination***********/
    if ((pLWVOutput->uLaneValidQualifer == LANE_NO_VALID) ||
        (pLWVOutput->uLaneValidQualifer == LANE_RIGHT_VALID) ||
        (pLWVOutput->uLaneValidQualifer == LANE_LEFT_VALID)) {
        bLaneWidthReset = 1U;
        fRawLaneWidth = pLWVParam->fDefaultLaneWidth;
    } else {
        bLaneWidthReset = 0U;
        if (pLWVOutput->uLaneValidQualifer == LANE_BOTH_VALID) {
            if (pLWVInput->bLaneChangeDtct) {
                bLaneWidthReset = 1U;
            }
            fRawLaneWidth =
                pLWVInput->fPosY0Lf * TUE_CML_Cos_M(pLWVInput->fHeadingLf) -
                pLWVInput->fPosY0Ri * TUE_CML_Cos_M(pLWVInput->fHeadingRi);
        } else {
            fRawLaneWidth = fLastLaneWidth;
        }
    } /* (bLaneWidthReset == 1) */

    if ((bLaneWidthReset == 1U))
    // || (pLWVInput->bNewCorridorValid == 1U)
    {
        bLowPassReset = 1U;
    } else {
        bLowPassReset = 0U;
    }

    if (bLowPassReset == 1U) {
        fRawTimeLowPass = 0.12F;
    } else {
        fRawTimeLowPass = pLWVParam->fLowPassTimeConst;
    }
    fTimeLowPass =
        TUE_CML_GradLimit_M(fRawTimeLowPass, 1.0F, -100.0F,
                            pLWVParam->fSysCycleTime, fLastTimeLowPass);

    if ((bLowPassReset == 1U) || (bLastLowPassReset == 1U)) {
        pLWVOutput->fLaneWidth = fRawLaneWidth;
    } else {
        fCoeffLowPass = (pLWVParam->fSysCycleTime) /
                        TUE_CML_Max_M(fTimeLowPass, pLWVParam->fSysCycleTime);
        pLWVOutput->fLaneWidth = fRawLaneWidth * fCoeffLowPass +
                                 fLastLaneWidth * (1.0F - fCoeffLowPass);
    }

    if (bLaneWidthReset == 1U) {
        pLWVOutput->bLaneWidthValid = 0U;
    } else {
        if (bLastLaneWidthValid == 1U) {
            fMaxLaneWidth =
                pLWVParam->fMaxLaneWidth + pLWVParam->fMaxLaneWidthHyst;
            fMinLaneWidth =
                pLWVParam->fMinLaneWidth - pLWVParam->fMinLaneWidthHyst;
        } else {
            fMaxLaneWidth = pLWVParam->fMaxLaneWidth;
            fMinLaneWidth = pLWVParam->fMinLaneWidth;
        }

        if ((pLWVOutput->fLaneWidth <= fMaxLaneWidth) &&
            (pLWVOutput->fLaneWidth >= fMinLaneWidth)) {
            pLWVOutput->bLaneWidthValid = 1U;
        } else {
            pLWVOutput->bLaneWidthValid = 0U;
        }
    }

    /*************************DebugAndOutput**********************/
    pLWVDebug->bLaneValidLf = bLaneValidLf;
    pLWVDebug->bLaneValidRi = bLaneValidRi;
    pLWVDebug->bLaneBridgeLf = bLaneBridgeLf;
    pLWVDebug->bLaneBridgeRi = bLaneBridgeRi;
    pLWVDebug->bLaneWidthReset = bLaneWidthReset;
    pLWVDebug->fRawLaneWidth = fRawLaneWidth;
    pLWVDebug->bLowPassReset = bLowPassReset;
    pLWVDebug->fTimeLowPass = fTimeLowPass;
    pLWVDebug->fRawTimeLowPass = fRawTimeLowPass;
    pLWVDebug->fMaxLaneWidth = fMaxLaneWidth;
    pLWVDebug->fMinLaneWidth = fMinLaneWidth;
    pLWVDebug->fCoeffLowPass = fCoeffLowPass;

    /*************************SaveLastValue************************/
    bLastLaneValidLf = bLaneValidLf;
    bLastLaneValidRi = bLaneValidRi;
    bLastLaneBridgeLf = bLaneBridgeLf;
    bLastLaneBridgeRi = bLaneBridgeRi;
    fLastLaneWidth = pLWVOutput->fLaneWidth;
    bLastLowPassReset = bLowPassReset;
    fLastTimeLowPass = fTimeLowPass;
    bLastLaneWidthValid = pLWVOutput->bLaneWidthValid;
}

/****************************************************************************************
        @fn           INPUT_LaneKalmanFilters
        @brief        Input for LaneKalmanFilters
        @description  LaneKalmanFilters input interface
        @param[in]    pLFPInput : Input for LFP input
        @param[in]    pLFPParam : Input for LFP param
        @param[in]    pLWVOutput : input from LWV Output
        @param[out]   pKLMInput: Output for KLM
        @param[out]   pKLMParam : Debug(measurement) for KLM
        @return       void
        @startuml
        title EgoLaneGeneration
        (*)-->1.Input from external
           --> (*)
        (*)-->2.pLWVOutput
           -->(*)
        @enduml
 ******************************************************************************************/
void INPUT_LaneKalmanFilters(const sLFPInput_t *pLFPInput,
                             const sLFPParam_t *pLFPParam,
                             const sLWVOutput_t *pLWVOutput,
                             const sCFVOutput_t *pCFVOutput,
                             sKLMInput_t *pKLMInput,
                             sKLMParam_t *pKLMParam) {
    pKLMInput->uLaneValidQualifer = pLWVOutput->uLaneValidQualifer;
    pKLMInput->bValidNewCorr = pLFPInput->bNewCorridorValid;
    pKLMInput->bUpDownHillDegrade = pLFPInput->bUpDownHillDegrade;
    pKLMInput->bValidKlmFltCntr = pCFVOutput->bValidKalmanFilterCntr;
    pKLMInput->fOverallQualityLf = pLFPInput->fOverallQualityLf;
    pKLMInput->fOverallQualityRi = pLFPInput->fOverallQualityRi;
    pKLMInput->fPosY0UlpLf = pLFPInput->fPosY0UlpLf;
    pKLMInput->fHeadingUlpLf = pLFPInput->fHeadingUlpLf;
    pKLMInput->fCrvUlpLf = pLFPInput->fCrvUlpLf;
    pKLMInput->fCrvRateUlpLf = pLFPInput->fCrvRateUlpLf;
    pKLMInput->fValidLengthUlpLf = pLFPInput->fValidLengthUlpLf;
    pKLMInput->fPosY0UlpRi = pLFPInput->fPosY0UlpRi;
    pKLMInput->fHeadingUlpRi = pLFPInput->fHeadingUlpRi;
    pKLMInput->fCrvUlpRi = pLFPInput->fCrvUlpRi;
    pKLMInput->fCrvRateUlpRi = pLFPInput->fCrvRateUlpRi;
    pKLMInput->fValidLengthUlpRi = pLFPInput->fValidLengthUlpRi;
    pKLMInput->fLaneWidth = pLWVOutput->fLaneWidth;
    pKLMInput->fVehYawRateStd = pLFPInput->fVehYawRateStd;
    pKLMInput->fVehVelX = pLFPInput->fVehVelX;
    pKLMInput->fVehYawRate = pLFPInput->fVehYawRate;
    pKLMInput->bLaneChangeDtct = pLFPInput->bLaneChangeDtct;
    pKLMInput->fStraightDtct = pLFPInput->fStraightDtct;

    /* */
    pKLMParam->bUseDegrUpdateForFlt = 0U;
    pKLMParam->fThdQualityForDegrUpdate = 50.0F;
    pKLMParam->fStdDevPosY0 = LBP_fStdDevPosY0;
    pKLMParam->fStdDevHeading = LBP_fStdDevHeading;
    pKLMParam->fStdDevCrv = LBP_fStdDevCrv;
    pKLMParam->fStdDevCrvRate = LBP_fStdDevCrvRate;
    pKLMParam->bUseVdyYawRateStdDev = 0U;
    pKLMParam->fStdDevVehYawRate = 0.01F * 10.0F;
    pKLMParam->fMinVelForKlmFilt = 0.01F;
    pKLMParam->fErrCoeff1 = 650.0F;
    pKLMParam->fErrCoeff2 = 35.0F;
    pKLMParam->fInitRFactor = 3.0F;
    pKLMParam->fDegradeWeight = 0.2F;
    pKLMParam->fMnUpdateQual = 30.0F;
    pKLMParam->fMnInitQual = 35.0F;
    pKLMParam->fIncQual = 33.0F;
    pKLMParam->fDecQualDeg = 33.0F;
    pKLMParam->fDecQualPred = 300.0F;
    pKLMParam->fKGainFac = 1.0F;
    pKLMParam->fDynYawFactor = 2.0F;
    pKLMParam->fDynDistYFact = 2500.0F;
    pKLMParam->fDynCrvFact = 100.0F;
    pKLMParam->fDynCrvRateFact = 2.0F;
    pKLMParam->fThdLatDistDev = 0.2F;
    pKLMParam->fDiffFadingFactor = 60.0F;
    pKLMParam->bUseGradientLimit = 1U;
    pKLMParam->fThdPosYGrd = 0.66F * 10.0F;
    pKLMParam->fThdHeadingGrd = 0.067F * 10.0F;
    pKLMParam->fThdCrvGrd = 0.067F * 10.0F;
    pKLMParam->fThdCrvRateGrd = 2.5E-5F * 10.0F;
    pKLMParam->bUseDegrUpdate = 0U;
    pKLMParam->bThdDegrUpdateQuality = 50.0F;
    pKLMParam->bUseStraightDtct = 0U;
    pKLMParam->fFacStraightDtct = 0.0F;
    pKLMParam->bUseCrvEstimation = 0U;
    pKLMParam->fStartLengthForCrvEst = 5.0F;
    pKLMParam->fEndLengthForCrvEst = 35.0F;
    pKLMParam->fMinDistForCrvEst = 5.0F;
    pKLMParam->fStartFltCrv = 5E-4F;
    pKLMParam->fEndFltCrv = 0.002F;
    pKLMParam->fTiFltForStraight = 0.25F;
    pKLMParam->fTiFltForCurve = 0.08F;
    pKLMParam->bUseCrvKlm = 0U;
    pKLMParam->fStdDevCrvCntr = 9E-4F;
    pKLMParam->fStdDevCrvRateCntr = 7E-5F;
    pKLMParam->fErrCoeff1Cntr = 650.0F;
    pKLMParam->fErrCoeff2Cntr = 35.0F;
    pKLMParam->fThdCrvCntr = 1E-4F;
    pKLMParam->fFacMatrixQCntr = 1.0005F;
    pKLMParam->fFacMatrixQStraightCntr = 1.0002F;
    pKLMParam->fFacInitRCntr = 3.0F;
    pKLMParam->fMinInitQual = 35.0F;
    pKLMParam->fMinUpdateQualCntr = 30.0F;
    pKLMParam->fWeightDegradeCntr = 0.2F;
    pKLMParam->fQualityIncCntr = 33.0F;
    pKLMParam->fQualityDecCntr = 16.66F;
    pKLMParam->fQualityDecPredCntr = 100.0F;
    pKLMParam->fSysCycleTime = pLFPParam->fSysCycleTime;
}

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
                                 UINT8_T *bEnaDegrUpdateCntr) {
    REAL32_T fDelatYaw = 0.0F;
    REAL32_T fDeltaPosX = 0.0F;
    REAL32_T fDeltaPosY = 0.0F;
    REAL32_T fRawPredPosY = 0.0F;
    REAL32_T fPredPosY = 0.0F;
    REAL32_T fPredLatDistLf = 0.0F;
    REAL32_T fPredLatDistRi = 0.0F;
    UINT8_T bEnaByLfKlmDiffCntr =
        0U; /* Enable flag for center lane lateral distance deviation by left
               lane Kalman filter Y predicted difference, (0, 0~1, -) */
    UINT8_T bEnaByRiKlmDiffCntr =
        0U; /* Enable flag for center lateral distance deviation by right lane
               Kalman filter Y predicted difference, (0, 0~1, -) */
    UINT8_T bLatDistDevLf = 0U;
    UINT8_T bLatDistDevRi = 0U;
    REAL32_T fTempKLM = 0.0F;
    REAL32_T fFacFadingCntr = 0.0F;
    REAL32_T fRawPosY0Cntr =
        0.0F; /* Center lane Y0 position by Ulp, (0, -15~15, m) */
    REAL32_T fRawHeadingCntr =
        0.0F; /* Center lane heading angle by Ulp, (0, -0.7854~0.7854, rad) */
    REAL32_T fRawCrvCntr =
        0.0F; /* Center lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    REAL32_T fRawCrvRateCntr =
        0.0F; /* Center lane curvature rate by Ulp, (0, -0.001~0.001, 1/m^2) */
    UINT8_T bResetRateLimitCntr = 0U;
    UINT8_T bRawEnaDegrUpdate = 0U;
    UINT8_T bEnaByEdgeFail = 0U;
    UINT8_T bEnaBySRTrig = 0U;

    /***************************2.1 Determine target
     * corridor******************************/
    /* Determine lane validity */
    if ((pKLMInput->uLaneValidQualifer == LANE_LEFT_VIRTUAL) ||
        (pKLMInput->uLaneValidQualifer == LANE_RIGHT_VIRTUAL) ||
        (pKLMInput->uLaneValidQualifer == LANE_BOTH_VALID)) {
        *bValidLaneCntr = 1U;
    } else {
        *bValidLaneCntr = 0U;
    }

    /* Determine lateral distance deviation validity*/
    /* Predict lateral position */
    fDelatYaw = pKLMInput->fVehYawRate * pKLMParam->fSysCycleTime;
    fDeltaPosX = pKLMInput->fVehVelX * pKLMParam->fSysCycleTime *
                 TUE_CML_Cos_M(fDelatYaw);
    fDeltaPosY = pKLMInput->fVehVelX * pKLMParam->fSysCycleTime *
                 TUE_CML_Sin_M(fDelatYaw);
    fRawPredPosY = TUE_CML_PosY3rd_M(
        fDeltaPosX, fLastPosY0KlmCntr_klm, fLastHeadingKlmCntr_klm,
        fLastCrvKlmCntr_klm, fLastCrvRateKlmCntr_klm);
    fPredPosY = fRawPredPosY * TUE_CML_Cos_M(fDelatYaw) -
                fDeltaPosX * TUE_CML_Sin_M(fDelatYaw) - fDeltaPosY;

    /* Predicted left lane lateral distance */
    fPredLatDistLf =
        fPredPosY + pKLMInput->fLaneWidth / 2.0F - pKLMInput->fPosY0UlpLf;
    fPredLatDistLf = TUE_CML_Abs_M(fPredLatDistLf);
    if ((fPredLatDistLf > pKLMParam->fThdLatDistDev) &&
        (pKLMInput->uLaneValidQualifer == LANE_BOTH_VALID)) {
        bEnaByLfKlmDiffCntr = 1U;
    } else {
        bEnaByLfKlmDiffCntr = 0U;
    }

    /* Predicted right lane lateral distance */
    fPredLatDistRi =
        fPredPosY - pKLMInput->fLaneWidth / 2.0F - pKLMInput->fPosY0UlpRi;
    fPredLatDistRi = TUE_CML_Abs_M(fPredLatDistRi);
    if ((fPredLatDistRi > pKLMParam->fThdLatDistDev) &&
        (pKLMInput->uLaneValidQualifer == LANE_BOTH_VALID)) {
        bEnaByRiKlmDiffCntr = 1U;
    } else {
        bEnaByRiKlmDiffCntr = 0U;
    }

    /* Enable flag by difference between prediction and measurement */
    if ((pKLMInput->bValidNewCorr == 0U) &&
        (pKLMInput->bLaneChangeDtct == 0U) &&
        (pKLMInput->bUpDownHillDegrade == 0U) &&
        (pKLMInput->bValidKlmFltCntr == 1U) && (bEnaByLfKlmDiffCntr == 1U)) {
        bLatDistDevLf = 1U;
    } else {
        bLatDistDevLf = 0U;
    }
    // pKLMOutput->bLatDistDevLf = bLatDistDevLf;

    if ((pKLMInput->bValidNewCorr == 0U) &&
        (pKLMInput->bLaneChangeDtct == 0U) &&
        (pKLMInput->bUpDownHillDegrade == 0U) &&
        (pKLMInput->bValidKlmFltCntr == 1U) && (bEnaByRiKlmDiffCntr == 1U)) {
        bLatDistDevRi = 1U;
    } else {
        bLatDistDevRi = 0U;
    }

    // pKLMOutput->bLatDistDevRi = bLatDistDevRi;

    /* Determine target corridor clothoid */
    /* Determine fading factor */
    if (pKLMInput->bUpDownHillDegrade == 1U) {
        fFacFadingCntr = 0.5F;
    } else {
        fTempKLM = pKLMInput->fOverallQualityLf - pKLMInput->fOverallQualityRi;
        fTempKLM = fTempKLM / pKLMParam->fDiffFadingFactor;
        fTempKLM = TUE_CML_Limit_M(fTempKLM, 1.0F, -1.0F);
        fFacFadingCntr = 0.5F + fTempKLM * 0.5F;
    }

    if (pKLMInput->uLaneValidQualifer == LANE_BOTH_VALID) {
        fRawPosY0Cntr =
            (pKLMInput->fPosY0UlpLf + pKLMInput->fPosY0UlpRi) / 2.0F;
        fRawHeadingCntr = (pKLMInput->fHeadingUlpRi) * (1 - fFacFadingCntr) +
                          (pKLMInput->fHeadingUlpLf) * fFacFadingCntr;
        fRawCrvCntr = (pKLMInput->fCrvUlpRi) * (1 - fFacFadingCntr) +
                      (pKLMInput->fCrvUlpLf) * fFacFadingCntr;
        fRawCrvRateCntr = (pKLMInput->fCrvRateUlpRi) * (1 - fFacFadingCntr) +
                          (pKLMInput->fCrvRateUlpLf) * fFacFadingCntr;
    } else if (pKLMInput->uLaneValidQualifer == LANE_LEFT_VIRTUAL) {
        if (pKLMInput->bLaneChangeDtct) {
            fRawPosY0Cntr =
                pKLMInput->fPosY0UlpRi + 0.5F * pKLMInput->fLaneWidth;
        } else {
            fRawPosY0Cntr =
                fLastRawPosY0Cntr + (pKLMInput->fPosY0UlpRi - fLastPosY0UlpRi);
        }
        fRawHeadingCntr = pKLMInput->fHeadingUlpRi;
        fRawCrvCntr = pKLMInput->fCrvUlpRi;
        fRawCrvRateCntr = pKLMInput->fCrvRateUlpRi;
    } else {
        if (pKLMInput->bLaneChangeDtct) {
            fRawPosY0Cntr =
                pKLMInput->fPosY0UlpLf - 0.5F * pKLMInput->fLaneWidth;
        } else {
            fRawPosY0Cntr =
                fLastRawPosY0Cntr + (pKLMInput->fPosY0UlpLf - fLastPosY0UlpLf);
        }
        fRawHeadingCntr = pKLMInput->fHeadingUlpLf;
        fRawCrvCntr = pKLMInput->fCrvUlpLf;
        fRawCrvRateCntr = pKLMInput->fCrvRateUlpLf;
    }
    /***************************2.2 Gradient
     * limiter***************************************/
    if ((pKLMInput->bLaneChangeDtct == 1U) ||
        // (pKLMInput->bValidNewCorr == 1U) ||
        (*bValidLaneCntr == 0U) ||
        ((bLastValidLaneCntr == 0U) && (*bValidLaneCntr == 1U))) {
        bResetRateLimitCntr = 1U;
    } else {
        bResetRateLimitCntr = 0U;
    }

    if ((pKLMParam->bUseGradientLimit == 1U) && (bResetRateLimitCntr == 0U)) {
        *fPosY0Cntr = TUE_CML_GradLimit_M(
            fRawPosY0Cntr, pKLMParam->fThdPosYGrd, (-pKLMParam->fThdPosYGrd),
            pKLMParam->fSysCycleTime, fLastPosY0Cntr);

        *fHeadingCntr =
            TUE_CML_GradLimit_M(fRawHeadingCntr, pKLMParam->fThdHeadingGrd,
                                (-pKLMParam->fThdHeadingGrd),
                                pKLMParam->fSysCycleTime, fLastHeadingCntr);

        *fCrvCntr = TUE_CML_GradLimit_M(fRawCrvCntr, pKLMParam->fThdCrvGrd,
                                        (-pKLMParam->fThdCrvGrd),
                                        pKLMParam->fSysCycleTime, fLastCrvCntr);

        *fCrvRateCntr =
            TUE_CML_GradLimit_M(fRawCrvRateCntr, pKLMParam->fThdCrvRateGrd,
                                (-pKLMParam->fThdCrvRateGrd),
                                pKLMParam->fSysCycleTime, fLastCrvRateCntr);
    } else {
        *fPosY0Cntr = fRawPosY0Cntr;
        *fHeadingCntr = fRawHeadingCntr;
        *fCrvCntr = fRawCrvCntr;
        *fCrvRateCntr = fRawCrvRateCntr;
    }

    /***************************2.3 Evaluate lane quality for target
     * corridor**************/
    *fOverallQualityCntr =
        (pKLMInput->fOverallQualityLf + pKLMInput->fOverallQualityRi) / 2.0F;
    if (pKLMParam->bUseDegrUpdate == 1U) {
        if (*fOverallQualityCntr < pKLMParam->bThdDegrUpdateQuality) {
            bRawEnaDegrUpdate = 1U;
        } else {
            bRawEnaDegrUpdate = 0U;
        }

        if ((bRawEnaDegrUpdate == 0U) || (bLastEnaByEdgeFail == 1U)) {
            bEnaByEdgeFail = 1U;
        } else {
            bEnaByEdgeFail = 0U;
        }

        bEnaBySRTrig = TUE_CML_SRTrigger_M(bEnaByEdgeFail, (!(*bValidLaneCntr)),
                                           bLastEnaBySRTrig);

        if ((bRawEnaDegrUpdate == 1U) && (bEnaBySRTrig == 1U)) {
            *bEnaDegrUpdateCntr = 1U;
        } else {
            *bEnaDegrUpdateCntr = 0U;
        }
    } else {
        *bEnaDegrUpdateCntr = 0U;
    }

    pKLMDebug->fDelatYaw = fDelatYaw;
    pKLMDebug->fDeltaPosX = fDeltaPosX;
    pKLMDebug->fDeltaPosY = fDeltaPosY;
    pKLMDebug->fRawPredPosY = fRawPredPosY;
    pKLMDebug->fPredPosY = fPredPosY;
    pKLMDebug->fPredLatDistLf = fPredLatDistLf;
    pKLMDebug->fPredLatDistRi = fPredLatDistRi;
    pKLMDebug->bEnaByLfKlmDiffCntr = bEnaByLfKlmDiffCntr;
    pKLMDebug->bEnaByRiKlmDiffCntr = bEnaByRiKlmDiffCntr;
    pKLMDebug->bLatDistDevLf = bLatDistDevLf;
    pKLMDebug->bLatDistDevRi = bLatDistDevRi;
    pKLMDebug->fFacFadingCntr = fFacFadingCntr;
    pKLMDebug->fRawPosY0Cntr = fRawPosY0Cntr;
    pKLMDebug->fRawHeadingCntr = fRawHeadingCntr;
    pKLMDebug->fRawCrvCntr = fRawCrvCntr;
    pKLMDebug->fRawCrvRateCntr = fRawCrvRateCntr;
    pKLMDebug->bResetRateLimitCntr = bResetRateLimitCntr;

    pKLMDebug->bRawEnaDegrUpdate = bRawEnaDegrUpdate;
    pKLMDebug->bEnaByEdgeFail = bEnaByEdgeFail;
    pKLMDebug->bEnaBySRTrig = bEnaBySRTrig;

    fLastPosY0UlpLf = pKLMInput->fPosY0UlpLf;
    fLastPosY0UlpRi = pKLMInput->fPosY0UlpRi;
    fLastRawPosY0Cntr = fRawPosY0Cntr;
    fLastPosY0Cntr = *fPosY0Cntr;
    fLastHeadingCntr = *fHeadingCntr;
    fLastCrvCntr = *fCrvCntr;
    fLastCrvRateCntr = *fCrvRateCntr;
    bLastEnaByEdgeFail = bEnaByEdgeFail;
    bLastEnaBySRTrig = bEnaBySRTrig;
}

void LFP_CenterLaneKalmanFilter(const sKLMInput_t *pKLMInput,
                                const sKLMParam_t *pKLMParam,
                                sKLMOutput_t *pKLMOutput,
                                sKLMDebug_t *pKLMDebug) {
    UINT8_T bValidLaneCntr = 0U; /* Center ego lane validity, (0, 0~1, -) */
    REAL32_T fTempKLM = 0.0F;
    REAL32_T fPosY0Cntr =
        0.0F; /* Center lane Y0 position by Ulp, (0, -15~15, m) */
    REAL32_T fHeadingCntr =
        0.0F; /* Center lane heading angle by Ulp, (0, -0.7854~0.7854, rad) */
    REAL32_T fCrvCntr =
        0.0F; /* Center lane curvature by Ulp, (0, -0.1~0.1, 1/m) */
    REAL32_T fCrvRateCntr =
        0.0F; /* Center lane curvature rate by Ulp, (0, -0.001~0.001, 1/m^2) */
    REAL32_T fOverallQualityCntr = 0.0F;
    UINT8_T bEnaDegrUpdateCntr = 0U;
    REAL32_T fCoeffByStraightDtct = 0.0F;
    REAL32_T fRawDegrWeightCntr = 0.0F;
    REAL32_T fRawCrvKlmCntr = 0.0F;
    REAL32_T fRawCrvRateKlmCntr = 0.0F;
    REAL32_T fLengthValidCntr = 0.0F;
    REAL32_T fStartPosXForCrvCntr = 0.0F;
    REAL32_T fEndPosXForCrvCntr = 0.0F;
    REAL32_T fPosXForCrvCntr[3] = {0.0F};
    REAL32_T fPosYForCrvCntr[3] = {0.0F};
    REAL32_T fCoeffForCrvCntr[3] = {0.0F};
    REAL32_T fRawEstCrvKlmCntr = 0.0F;
    UINT8_T bCrvLowPassReset = 0U;
    REAL32_T fEstCrvKlmCntr = 0.0F;
    REAL32_T fRawTiFltCrvEstCntr = 0.0F;
    REAL32_T fTiFltCrvEstCntr = 0.0F;
    REAL32_T fCrvByCrvFltCntr = 0.0F;
    REAL32_T fCrvRateByCrvFltCntr = 0.0F;
    UINT8_T uBtfStatusByCrvFltLf;
    REAL32_T fQualityByCrvFltLf;

    laneKFInTypeV3 sLKFInputCntr = {0};
    laneKFOutType sLKFOutputCntr = {0};
    crvKFInTypeV2 sCrvInputCntr = {0};
    crvKFOutType sCrvOutputCntr = {0};

    /***************************2.Center lane Kalman
     * filter********************************/
    /***************************2.1 Determine target
     * corridor******************************/

    LFP_DetermineTargetCorridor(
        pKLMInput, pKLMParam, pKLMDebug, fLastPosY0KlmCntr_klm,
        fLastHeadingKlmCntr_klm, fLastCrvKlmCntr_klm, fLastCrvRateKlmCntr_klm,
        bLastValidLaneCntr, &bValidLaneCntr, &fPosY0Cntr, &fHeadingCntr,
        &fCrvCntr, &fCrvRateCntr, &fOverallQualityCntr, &bEnaDegrUpdateCntr);

    /***************************2.4 Center lane Kalman
     * filter******************************/
    sLKFInputCntr.sf_PosY0_met = fPosY0Cntr;
    sLKFInputCntr.sf_HeadingAngle_rad = fHeadingCntr;

    if (pKLMParam->bUseStraightDtct == 1U) {
        fCoeffByStraightDtct =
            (1.0F - pKLMInput->fStraightDtct / 100.0F) +
            pKLMParam->fFacStraightDtct * pKLMInput->fStraightDtct / 100.0F;
        sLKFInputCntr.sf_Crv_1pm = fCrvCntr * fCoeffByStraightDtct;
        sLKFInputCntr.sf_CrvChng_1pm2 = fCrvRateCntr * fCoeffByStraightDtct;
    } else {
        sLKFInputCntr.sf_Crv_1pm = fCrvCntr;
        sLKFInputCntr.sf_CrvChng_1pm2 = fCrvRateCntr;
    }

    sLKFInputCntr.sf_Length_met = 0.0F;
    sLKFInputCntr.sf_PosY0StdDev_met = pKLMParam->fStdDevPosY0;
    sLKFInputCntr.sf_HeadingAngleStdDev_rad = pKLMParam->fStdDevHeading;
    sLKFInputCntr.sf_CrvStdDev_1pm = pKLMParam->fStdDevCrv;
    sLKFInputCntr.sf_CrvChngStdDev_1pm2 = pKLMParam->fStdDevCrvRate;

    if (pKLMParam->bUseVdyYawRateStdDev == 1U) {
        sLKFInputCntr.sf_VehYawRateStdDev_radps = pKLMInput->fVehYawRateStd;
    } else {
        sLKFInputCntr.sf_VehYawRateStdDev_radps = pKLMParam->fStdDevVehYawRate;
    }

    sLKFInputCntr.sf_VehVelX_mps =
        TUE_CML_Max_M(pKLMInput->fVehVelX, pKLMParam->fMinVelForKlmFilt);
    sLKFInputCntr.sf_VehYawRate_radps = pKLMInput->fVehYawRate;
    sLKFInputCntr.sf_DeltaT_sec = pKLMParam->fSysCycleTime;
    sLKFInputCntr.sf_LaneDataValid_bool = bValidLaneCntr;

    if (bEnaDegrUpdateCntr == 1U) {
        sLKFInputCntr.sf_DegradedUpdate_bool = 1U;
        fRawDegrWeightCntr = 0.001F;
    } else {
        fRawDegrWeightCntr = 1.0F;
        if ((bLastValidLaneCntr == 0U) && (bValidLaneCntr == 1U)) {
            sLKFInputCntr.sf_LaneKFDegradeWeight_nu = fRawDegrWeightCntr;
        } else {
            sLKFInputCntr.sf_LaneKFDegradeWeight_nu = TUE_CML_GradLimit_M(
                fRawDegrWeightCntr, 0.5F, -100.0F, pKLMParam->fSysCycleTime,
                fLastDegrWeightCntr);
        }

        if (sLKFInputCntr.sf_LaneKFDegradeWeight_nu <= 0.9F) {
            sLKFInputCntr.sf_DegradedUpdate_bool = 1U;
        } else {
            sLKFInputCntr.sf_DegradedUpdate_bool = 0U;
        }
    }

    sLKFInputCntr.sf_OverallMeasurementQuality_perc =
        (UINT8_T)fOverallQualityCntr;
    if (  //(pKLMInput->bValidNewCorr == 1U) ||
        (pKLMInput->bLaneChangeDtct == 1U)) {
        sLKFInputCntr.sf_LaneChange_bool = 1U;
    } else {
        sLKFInputCntr.sf_LaneChange_bool = 0U;
    }

    sLKFInputCntr.sf_LaneKFErrCoeff1_met = pKLMParam->fErrCoeff1;
    sLKFInputCntr.sf_LaneKFErrCoeff2_mps = pKLMParam->fErrCoeff2;
    sLKFInputCntr.sf_LaneKFInitRFactor_nu = pKLMParam->fInitRFactor;
    sLKFInputCntr.sf_LaneKFMnUpdateQual_perc =
        (UINT8_T)pKLMParam->fMnUpdateQual;
    sLKFInputCntr.sf_LaneKFMnInitQual_perc = (UINT8_T)pKLMParam->fMnInitQual;
    sLKFInputCntr.sf_LaneKFIncQual_1ps = pKLMParam->fIncQual;
    sLKFInputCntr.sf_LaneKFDecQualDeg_1ps = pKLMParam->fDecQualDeg;
    sLKFInputCntr.sf_LaneKFDecQualPred_1ps = pKLMParam->fDecQualPred;
    sLKFInputCntr.sf_LaneKFKGainFac_nu = pKLMParam->fKGainFac;
    sLKFInputCntr.sf_LaneKFDynYawFactor_nu = pKLMParam->fDynYawFactor;
    sLKFInputCntr.sf_LaneKFDynDistYFact_nu = pKLMParam->fDynDistYFact;
    sLKFInputCntr.sf_LaneKFDynCrvFact_nu = pKLMParam->fDynCrvFact;
    sLKFInputCntr.sf_LaneKFDynCrvRateFact_nu = pKLMParam->fDynCrvRateFact;

    laneKalmanFilter_Center(&sLKFInputCntr, &sLKFOutputCntr);

    /*  */
    pKLMOutput->fPosY0KlmCntr = sLKFOutputCntr.sf_PosY0_met;
    pKLMOutput->fHeadingKlmCntr = sLKFOutputCntr.sf_HeadingAngle_rad;
    fRawCrvKlmCntr = sLKFOutputCntr.sf_Crv_1pm;
    fRawCrvRateKlmCntr = sLKFOutputCntr.sf_CrvChng_1pm2;
    pKLMOutput->uBtfStatusKlmCntr = sLKFOutputCntr.sf_KFStatus_btf;
    pKLMOutput->fQualityMeasureCntr = sLKFOutputCntr.sf_QualityMeasure_perc;

    /* Center lane curvature estimation */
    fLengthValidCntr = TUE_CML_Max_M(pKLMInput->fValidLengthUlpLf,
                                     pKLMInput->fValidLengthUlpRi);

    fTempKLM = fLengthValidCntr - pKLMParam->fStartLengthForCrvEst;
    if ((fTempKLM < pKLMParam->fMinDistForCrvEst) ||
        (fLengthValidCntr < pKLMParam->fStartLengthForCrvEst)) {
        fStartPosXForCrvCntr = 0.0F;
    } else {
        fStartPosXForCrvCntr = pKLMParam->fStartLengthForCrvEst;
    }
    fEndPosXForCrvCntr =
        TUE_CML_Min_M(fLengthValidCntr, pKLMParam->fEndLengthForCrvEst);

    for (UINT8_T ii = 0U; ii < 3U; ii++) {
        fPosXForCrvCntr[ii] =
            fStartPosXForCrvCntr +
            ii * (fEndPosXForCrvCntr - fStartPosXForCrvCntr) / 2.0F;
        fPosYForCrvCntr[ii] = TUE_CML_PosY3rd_M(
            fPosXForCrvCntr[ii], sLKFOutputCntr.sf_PosY0_met,
            sLKFOutputCntr.sf_HeadingAngle_rad, sLKFOutputCntr.sf_Crv_1pm,
            sLKFOutputCntr.sf_CrvChng_1pm2);
    }

    /* calculate 2nd Order Polynomial */
    TUE_CML_Interp2_M(fPosXForCrvCntr, fPosYForCrvCntr, fCoeffForCrvCntr);
    fTempKLM =
        2 * fCoeffForCrvCntr[0] * fPosXForCrvCntr[1] + fCoeffForCrvCntr[1];
    fTempKLM = (1 + fTempKLM * fTempKLM);
    fRawEstCrvKlmCntr =
        (2 * fCoeffForCrvCntr[0]) / (TUE_CML_Sqrt_M(fTempKLM) * fTempKLM);

    /* Curvature low pass filter */
    if (pKLMParam->bUseStraightDtct == 1U) {
        fTempKLM = pKLMInput->fStraightDtct / 100.0F;
        fTempKLM = (1.0F - fTempKLM) + pKLMParam->fFacStraightDtct * fTempKLM;
        fRawEstCrvKlmCntr = fRawEstCrvKlmCntr * fTempKLM;
    } else {
        ;
    }

    bCrvLowPassReset = (sLKFOutputCntr.sf_KFStatus_btf == KF_STATE_INIT);

    if (bLastCrvLowPassReset) {
        fEstCrvKlmCntr = fRawEstCrvKlmCntr;
    } else {
        fTempKLM = (pKLMParam->fEndFltCrv - pKLMParam->fStartFltCrv);
        fTempKLM =
            1.0F + (TUE_CML_Abs_M(fRawEstCrvKlmCntr) - pKLMParam->fEndFltCrv) /
                       fTempKLM;
        fRawTiFltCrvEstCntr =
            TUE_CML_Limit_M(fTempKLM, 1.0F, 0.0F) * pKLMParam->fTiFltForCurve;
        fTiFltCrvEstCntr =
            TUE_CML_Max_M(fRawTiFltCrvEstCntr, pKLMParam->fSysCycleTime);
        fTiFltCrvEstCntr =
            TUE_CML_Max_M(fTiFltCrvEstCntr, pKLMParam->fTiFltForStraight);

        fEstCrvKlmCntr = TUE_CML_LowPassFilter_M(
            fRawEstCrvKlmCntr, fTiFltCrvEstCntr, pKLMParam->fSysCycleTime,
            fLastEstCrvKlmCntr);
    }

    /* Curve kalman filter */
    if (pKLMParam->bUseStraightDtct == 1U) {
        fTempKLM = pKLMInput->fStraightDtct / 100.0F;
        fTempKLM = (1.0F - fTempKLM) + pKLMParam->fFacStraightDtct * fTempKLM;
        sCrvInputCntr.sf_Crv_1pm = fCrvCntr * fTempKLM;
        sCrvInputCntr.sf_CrvChng_1pm2 = fCrvRateCntr * fTempKLM;
    } else {
        sCrvInputCntr.sf_Crv_1pm = fCrvCntr;
        sCrvInputCntr.sf_CrvChng_1pm2 = fCrvRateCntr;
    }
    sCrvInputCntr.sf_CrvStdDev_1pm = pKLMParam->fStdDevCrvCntr;
    sCrvInputCntr.sf_CrvChngStdDev_1pm2 = pKLMParam->fStdDevCrvRateCntr;
    sCrvInputCntr.sf_VehVelX_mps = pKLMInput->fVehVelX;
    sCrvInputCntr.sf_DeltaT_sec = pKLMParam->fSysCycleTime;
    sCrvInputCntr.sf_crvDataValid_bool = bValidLaneCntr;
    sCrvInputCntr.sf_DegradedUpdate_bool = 0U;
    sCrvInputCntr.sf_OverallMeasurementQuality_perc =
        (UINT8_T)fOverallQualityCntr;
    sCrvInputCntr.sf_CrvKFErrCoeff1_nu = pKLMParam->fErrCoeff1Cntr;
    sCrvInputCntr.sf_CrvKFErrCoeff2_nu = pKLMParam->fErrCoeff2Cntr;
    sCrvInputCntr.sf_CrvKFDefCurve_1pm = pKLMParam->fThdCrvCntr;
    sCrvInputCntr.sf_CrvKFQ11Fac_nu = pKLMParam->fFacMatrixQCntr;
    sCrvInputCntr.sf_CrvKFQ11FacStraight_nu =
        pKLMParam->fFacMatrixQStraightCntr;
    sCrvInputCntr.sf_CrvKFInitRFactor_nu = pKLMParam->fFacInitRCntr;
    sCrvInputCntr.sf_CrvKFMnInitQual_perc = (UINT8_T)pKLMParam->fMinInitQual;
    sCrvInputCntr.sf_CrvKFMnUpdateQual_perc =
        (UINT8_T)pKLMParam->fMinUpdateQualCntr;
    sCrvInputCntr.sf_CrvKFDegradeWeight_nu = pKLMParam->fWeightDegradeCntr;
    sCrvInputCntr.sf_CrvKFIncQual_1ps = pKLMParam->fQualityIncCntr;
    sCrvInputCntr.sf_CrvKFDecQualDeg_1ps = pKLMParam->fQualityDecCntr;
    sCrvInputCntr.sf_CrvKFDecQualPred_1ps = pKLMParam->fQualityDecPredCntr;

    crvKalmanFilter(&sCrvInputCntr, &sCrvOutputCntr);
    fCrvByCrvFltCntr = sCrvOutputCntr.sf_Crv_1pm;
    fCrvRateByCrvFltCntr = sCrvOutputCntr.sf_CrvChng_1pm2;
    uBtfStatusByCrvFltLf = sCrvOutputCntr.sf_KFStatus_btf;
    fQualityByCrvFltLf = sCrvOutputCntr.sf_QualityMeasure_perc;

    if (pKLMParam->bUseCrvKlm == 1U) {
        pKLMOutput->fCrvKlmCntr = fCrvByCrvFltCntr;
        pKLMOutput->fCrvRateKlmCntr = fCrvRateByCrvFltCntr;
    } else if (pKLMParam->bUseCrvEstimation == 1U) {
        pKLMOutput->fCrvKlmCntr = fEstCrvKlmCntr;
        pKLMOutput->fCrvRateKlmCntr = fRawCrvRateKlmCntr;
    } else {
        pKLMOutput->fCrvKlmCntr = fRawCrvKlmCntr;
        pKLMOutput->fCrvRateKlmCntr = fRawCrvRateKlmCntr;
    }

    pKLMDebug->bValidLaneCntr = bValidLaneCntr;
    pKLMDebug->fPosY0Cntr = fPosY0Cntr;
    pKLMDebug->fHeadingCntr = fHeadingCntr;
    pKLMDebug->fCrvCntr = fCrvCntr;
    pKLMDebug->fCrvRateCntr = fCrvRateCntr;
    pKLMDebug->fOverallQualityCntr = fOverallQualityCntr;
    pKLMDebug->bEnaDegrUpdateCntr = bEnaDegrUpdateCntr;
    pKLMDebug->fRawCrvKlmLf = sLKFOutputCntr.sf_Crv_1pm;
    pKLMDebug->fCoeffByStraightDtct = fCoeffByStraightDtct;
    pKLMDebug->fRawDegrWeightCntr = fRawDegrWeightCntr;
    pKLMDebug->fLengthValidCntr = fLengthValidCntr;
    pKLMDebug->fStartPosXForCrvCntr = fStartPosXForCrvCntr;
    pKLMDebug->fEndPosXForCrvCntr = fEndPosXForCrvCntr;
    pKLMDebug->fRawEstCrvKlmCntr = fRawEstCrvKlmCntr;
    pKLMDebug->fRawTiFltCrvEstCntr = fRawTiFltCrvEstCntr;
    pKLMDebug->fTiFltCrvEstCntr = fTiFltCrvEstCntr;
    pKLMDebug->fEstCrvKlmCntr = fEstCrvKlmCntr;
    pKLMDebug->fCrvByCrvFltCntr = fCrvByCrvFltCntr;
    pKLMDebug->fCrvRateByCrvFltCntr = fCrvRateByCrvFltCntr;
    pKLMDebug->uBtfStatusByCrvFltLf = uBtfStatusByCrvFltLf;
    pKLMDebug->fQualityByCrvFltLf = (UINT8_T)fQualityByCrvFltLf;

    bLastValidLaneCntr = bValidLaneCntr;
    fLastDegrWeightCntr = sLKFInputCntr.sf_LaneKFDegradeWeight_nu;
    fLastEstCrvKlmCntr = fEstCrvKlmCntr;
    bLastCrvLowPassReset = bCrvLowPassReset;
    fLastPosY0KlmCntr_klm = pKLMOutput->fPosY0KlmCntr;
    fLastHeadingKlmCntr_klm = pKLMOutput->fHeadingKlmCntr;
    fLastCrvKlmCntr_klm = pKLMOutput->fCrvKlmCntr;
    fLastCrvRateKlmCntr_klm = pKLMOutput->fCrvRateKlmCntr;
}
/****************************************************************************************
        @fn           LaneKalmanFilters
        @brief        Kalman filtering on ego lane
        @description  Lane Kalman filters:
                                          1.Left lane Kalman filter;
                                          2.Center lane Kalman filter;
                                          3.Right lane Kalman filter;
        @param[in]    pLFPInput : Input for LaneKalmanFilters
        @param[in]    pLFPParam : Parameter for LaneKalmanFilters
        @param[out]   pLFPOutput: Output for LaneKalmanFilters
        @param[out]   pLFPDebug : Debug(measurement) for LaneKalmanFilters
        @return       void
        @startuml
        title LaneKalmanFilters
        (*)-->1.Left lane Kalman filter
           --> (*)
        (*)-->2.Center lane Kalman filter
           -->(*)
        (*)-->3.Right lane Kalman filter
           -->(*)
        @enduml
 ******************************************************************************************/
void LaneKalmanFilters(const sKLMInput_t *pKLMInput,
                       const sKLMParam_t *pKLMParam,
                       sKLMOutput_t *pKLMOutput,
                       sKLMDebug_t *pKLMDebug) {
    laneKFInTypeV3 sLKFInputLf = {0};
    laneKFOutType sLKFOutputLf = {0};
    laneKFInTypeV3 sLKFInputRi = {0};
    laneKFOutType sLKFOutputRi = {0};

    UINT8_T bValidLaneLf = 0U;     /* Left ego lane validity, (0, 0~1, -) */
    UINT8_T bValidLaneRi = 0U;     /* Right ego lane validity, (0, 0~1, -) */
    UINT8_T bEnaDegrUpdateLf = 0U; /* Enable flag for degraded update in center
                                      lane kalman filter, (0, 0~1, -) */
    UINT8_T bEnaDegrUpdateRi = 0U; /* Enable flag for degraded update in center
                                      lane kalman filter, (0, 0~1, -) */
    REAL32_T fTempKLM = 0.0F;
    REAL32_T fPosXForCrvLf[3] = {0.0F};
    REAL32_T fPosXForCrvRi[3] = {0.0F};
    REAL32_T fStartPosXForCrvLf = 0.0F;
    REAL32_T fStartPosXForCrvRi = 0.0F;
    REAL32_T fEndPosXForCrvLf = 0.0F;
    REAL32_T fEndPosXForCrvRi = 0.0F;

    REAL32_T fPosYForCrvLf[3] = {0.0F};
    REAL32_T fPosYForCrvRi[3] = {0.0F};
    REAL32_T fCoeffForCrvLf[3] = {0.0F};
    REAL32_T fCoeffForCrvRi[3] = {0.0F};

    REAL32_T fTiFltCrvEstLf = 0.0F;
    REAL32_T fTiFltCrvEstRi = 0.0F;
    REAL32_T fRawTiFltCrvEstLf = 0.0F;
    REAL32_T fRawTiFltCrvEstRi = 0.0F;
    REAL32_T fRawCrvKlmLf = 0.0F;
    REAL32_T fRawEstCrvKlmLf = 0.0F;
    REAL32_T fEstCrvKlmLf = 0.0F;

    REAL32_T fRawCrvKlmRi = 0.0F;
    REAL32_T fRawEstCrvKlmRi = 0.0F;
    REAL32_T fEstCrvKlmRi = 0.0F;

    /***************************1.Left lane Kalman
     * filter**********************************/
    /* Determine lane bound quality and validity */
    if ((pKLMInput->uLaneValidQualifer == LANE_LEFT_VALID) ||
        (pKLMInput->uLaneValidQualifer == LANE_RIGHT_VIRTUAL) ||
        (pKLMInput->uLaneValidQualifer == LANE_BOTH_VALID)) {
        bValidLaneLf = 1U;
    } else {
        bValidLaneLf = 0U;
    }

    if ((pKLMParam->bUseDegrUpdateForFlt == 1U) &&
        (pKLMInput->fOverallQualityLf < pKLMParam->fThdQualityForDegrUpdate)) {
        bEnaDegrUpdateLf = 1U;
    } else {
        bEnaDegrUpdateLf = 0U;
    }

    /* Left lane kalman filter */
    sLKFInputLf.sf_PosY0_met = pKLMInput->fPosY0UlpLf;
    sLKFInputLf.sf_HeadingAngle_rad = pKLMInput->fHeadingUlpLf;
    sLKFInputLf.sf_Crv_1pm = pKLMInput->fCrvUlpLf;
    sLKFInputLf.sf_CrvChng_1pm2 = pKLMInput->fCrvRateUlpLf;
    sLKFInputLf.sf_Length_met = 0.0F;
    sLKFInputLf.sf_PosY0StdDev_met = pKLMParam->fStdDevPosY0;
    sLKFInputLf.sf_HeadingAngleStdDev_rad = pKLMParam->fStdDevHeading;
    sLKFInputLf.sf_CrvStdDev_1pm = pKLMParam->fStdDevCrv;
    sLKFInputLf.sf_CrvChngStdDev_1pm2 = pKLMParam->fStdDevCrvRate;

    if (pKLMParam->bUseVdyYawRateStdDev == 1U) {
        sLKFInputLf.sf_VehYawRateStdDev_radps = pKLMInput->fVehYawRateStd;
    } else {
        sLKFInputLf.sf_VehYawRateStdDev_radps = pKLMParam->fStdDevVehYawRate;
    }

    sLKFInputLf.sf_VehVelX_mps =
        TUE_CML_Max_M(pKLMInput->fVehVelX, pKLMParam->fMinVelForKlmFilt);
    sLKFInputLf.sf_VehYawRate_radps = pKLMInput->fVehYawRate;
    sLKFInputLf.sf_DeltaT_sec = pKLMParam->fSysCycleTime;
    sLKFInputLf.sf_LaneDataValid_bool = bValidLaneLf;
    sLKFInputLf.sf_DegradedUpdate_bool = bEnaDegrUpdateLf;
    sLKFInputLf.sf_OverallMeasurementQuality_perc =
        (UINT8_T)pKLMInput->fOverallQualityLf;
    sLKFInputLf.sf_LaneChange_bool = pKLMInput->bLaneChangeDtct;
    sLKFInputLf.sf_LaneKFErrCoeff1_met = pKLMParam->fErrCoeff1;
    sLKFInputLf.sf_LaneKFErrCoeff2_mps = pKLMParam->fErrCoeff2;
    sLKFInputLf.sf_LaneKFInitRFactor_nu = pKLMParam->fInitRFactor;
    sLKFInputLf.sf_LaneKFDegradeWeight_nu = pKLMParam->fDegradeWeight;
    sLKFInputLf.sf_LaneKFMnUpdateQual_perc = (UINT8_T)pKLMParam->fMnUpdateQual;
    sLKFInputLf.sf_LaneKFMnInitQual_perc = (UINT8_T)pKLMParam->fMnInitQual;
    sLKFInputLf.sf_LaneKFIncQual_1ps = pKLMParam->fIncQual;
    sLKFInputLf.sf_LaneKFDecQualDeg_1ps = pKLMParam->fDecQualDeg;
    sLKFInputLf.sf_LaneKFDecQualPred_1ps = pKLMParam->fDecQualPred;
    sLKFInputLf.sf_LaneKFKGainFac_nu = pKLMParam->fKGainFac;
    sLKFInputLf.sf_LaneKFDynYawFactor_nu = pKLMParam->fDynYawFactor;
    sLKFInputLf.sf_LaneKFDynDistYFact_nu = pKLMParam->fDynDistYFact;
    sLKFInputLf.sf_LaneKFDynCrvFact_nu = pKLMParam->fDynCrvFact;
    sLKFInputLf.sf_LaneKFDynCrvRateFact_nu = pKLMParam->fDynCrvRateFact;

    /*  */
    laneKalmanFilter_Left(&sLKFInputLf, &sLKFOutputLf);

    pKLMOutput->fPosY0KlmLf = sLKFOutputLf.sf_PosY0_met;
    pKLMOutput->fHeadingKlmLf = sLKFOutputLf.sf_HeadingAngle_rad;
    fRawCrvKlmLf = sLKFOutputLf.sf_Crv_1pm;
    pKLMOutput->fCrvRateKlmLf = sLKFOutputLf.sf_CrvChng_1pm2;
    pKLMOutput->uBtfStatusKlmLf = sLKFOutputLf.sf_KFStatus_btf;
    pKLMOutput->fQualityMeasureLf = sLKFOutputLf.sf_QualityMeasure_perc;

    /* Left lane curvature estimation */
    fTempKLM = pKLMInput->fValidLengthUlpLf - pKLMParam->fStartLengthForCrvEst;
    if ((fTempKLM < pKLMParam->fMinDistForCrvEst) ||
        (pKLMInput->fValidLengthUlpLf < pKLMParam->fStartLengthForCrvEst)) {
        fStartPosXForCrvLf = 0.0F;
    } else {
        fStartPosXForCrvLf = pKLMParam->fStartLengthForCrvEst;
    }
    fEndPosXForCrvLf = TUE_CML_Min_M(pKLMInput->fValidLengthUlpLf,
                                     pKLMParam->fEndLengthForCrvEst);

    for (UINT8_T ii = 0U; ii < 3U; ii++) {
        fPosXForCrvLf[ii] = fStartPosXForCrvLf +
                            ii * (fEndPosXForCrvLf - fStartPosXForCrvLf) / 2.0F;
        fPosYForCrvLf[ii] = TUE_CML_PosY3rd_M(
            fPosXForCrvLf[ii], sLKFOutputLf.sf_PosY0_met,
            sLKFOutputLf.sf_HeadingAngle_rad, sLKFOutputLf.sf_Crv_1pm,
            sLKFOutputLf.sf_CrvChng_1pm2);
    }

    /* calculate 2nd Order Polynomial */
    TUE_CML_Interp2_M(fPosXForCrvLf, fPosYForCrvLf, fCoeffForCrvLf);
    fTempKLM = 2 * fCoeffForCrvLf[0] * fPosXForCrvLf[1] + fCoeffForCrvLf[1];
    fTempKLM = (1 + fTempKLM * fTempKLM);
    fRawEstCrvKlmLf =
        (2 * fCoeffForCrvLf[0]) / (TUE_CML_Sqrt_M(fTempKLM) * fTempKLM);

    /* Curvature low pass filter */
    fTempKLM = (pKLMParam->fEndFltCrv - pKLMParam->fStartFltCrv);
    fTempKLM = 1.0F + (TUE_CML_Abs_M(fRawEstCrvKlmLf) - pKLMParam->fEndFltCrv) /
                          fTempKLM;
    fRawTiFltCrvEstLf =
        TUE_CML_Limit_M(fTempKLM, 1.0F, 0.0F) * pKLMParam->fTiFltForCurve;
    fTiFltCrvEstLf = TUE_CML_Max_M(fRawTiFltCrvEstLf, pKLMParam->fSysCycleTime);
    fTiFltCrvEstLf =
        TUE_CML_Max_M(fTiFltCrvEstLf, pKLMParam->fTiFltForStraight);

    fEstCrvKlmLf =
        TUE_CML_LowPassFilter_M(fRawEstCrvKlmLf, fTiFltCrvEstLf,
                                pKLMParam->fSysCycleTime, fLastEstCrvKlmLf);

    if (pKLMParam->bUseCrvEstimation == 0U) {
        pKLMOutput->fCrvKlmLf = fRawCrvKlmLf;
    } else {
        pKLMOutput->fCrvKlmLf = fEstCrvKlmLf;
    }

    /***************************2.Center lane Kalman
     * filter********************************/
    LFP_CenterLaneKalmanFilter(pKLMInput, pKLMParam, pKLMOutput, pKLMDebug);

    /***************************3.Right lane Kalman
     * filter*********************************/
    if ((pKLMInput->uLaneValidQualifer == LANE_RIGHT_VALID) ||
        (pKLMInput->uLaneValidQualifer == LANE_LEFT_VIRTUAL) ||
        (pKLMInput->uLaneValidQualifer == LANE_BOTH_VALID)) {
        bValidLaneRi = 1U;
    } else {
        bValidLaneRi = 0U;
    }

    if ((pKLMParam->bUseDegrUpdateForFlt == 1U) &&
        (pKLMInput->fOverallQualityRi < pKLMParam->fThdQualityForDegrUpdate)) {
        bEnaDegrUpdateRi = 1U;
    } else {
        bEnaDegrUpdateRi = 0U;
    }

    /* Right lane kalman filter */
    sLKFInputRi.sf_PosY0_met = pKLMInput->fPosY0UlpRi;
    sLKFInputRi.sf_HeadingAngle_rad = pKLMInput->fHeadingUlpRi;
    sLKFInputRi.sf_Crv_1pm = pKLMInput->fCrvUlpRi;
    sLKFInputRi.sf_CrvChng_1pm2 = pKLMInput->fCrvRateUlpRi;
    sLKFInputRi.sf_Length_met = 0.0F;
    sLKFInputRi.sf_PosY0StdDev_met = pKLMParam->fStdDevPosY0;
    sLKFInputRi.sf_HeadingAngleStdDev_rad = pKLMParam->fStdDevHeading;
    sLKFInputRi.sf_CrvStdDev_1pm = pKLMParam->fStdDevCrv;
    sLKFInputRi.sf_CrvChngStdDev_1pm2 = pKLMParam->fStdDevCrvRate;

    if (pKLMParam->bUseVdyYawRateStdDev == 1U) {
        sLKFInputRi.sf_VehYawRateStdDev_radps = pKLMInput->fVehYawRateStd;
    } else {
        sLKFInputRi.sf_VehYawRateStdDev_radps = pKLMParam->fStdDevVehYawRate;
    }

    sLKFInputRi.sf_VehVelX_mps =
        TUE_CML_Max_M(pKLMInput->fVehVelX, pKLMParam->fMinVelForKlmFilt);
    sLKFInputRi.sf_VehYawRate_radps = pKLMInput->fVehYawRate;
    sLKFInputRi.sf_DeltaT_sec = pKLMParam->fSysCycleTime;
    sLKFInputRi.sf_LaneDataValid_bool = bValidLaneRi;
    sLKFInputRi.sf_DegradedUpdate_bool = bEnaDegrUpdateRi;
    sLKFInputRi.sf_OverallMeasurementQuality_perc =
        (UINT8_T)pKLMInput->fOverallQualityRi;

    sLKFInputRi.sf_LaneChange_bool = pKLMInput->bLaneChangeDtct;

    sLKFInputRi.sf_LaneKFErrCoeff1_met = pKLMParam->fErrCoeff1;
    sLKFInputRi.sf_LaneKFErrCoeff2_mps = pKLMParam->fErrCoeff2;
    sLKFInputRi.sf_LaneKFInitRFactor_nu = pKLMParam->fInitRFactor;
    sLKFInputRi.sf_LaneKFDegradeWeight_nu = pKLMParam->fDegradeWeight;
    sLKFInputRi.sf_LaneKFMnUpdateQual_perc = (UINT8_T)pKLMParam->fMnUpdateQual;
    sLKFInputRi.sf_LaneKFMnInitQual_perc = (UINT8_T)pKLMParam->fMnInitQual;
    sLKFInputRi.sf_LaneKFIncQual_1ps = pKLMParam->fIncQual;
    sLKFInputRi.sf_LaneKFDecQualDeg_1ps = pKLMParam->fDecQualDeg;
    sLKFInputRi.sf_LaneKFDecQualPred_1ps = pKLMParam->fDecQualPred;
    sLKFInputRi.sf_LaneKFKGainFac_nu = pKLMParam->fKGainFac;
    sLKFInputRi.sf_LaneKFDynYawFactor_nu = pKLMParam->fDynYawFactor;
    sLKFInputRi.sf_LaneKFDynDistYFact_nu = pKLMParam->fDynDistYFact;
    sLKFInputRi.sf_LaneKFDynCrvFact_nu = pKLMParam->fDynCrvFact;
    sLKFInputRi.sf_LaneKFDynCrvRateFact_nu = pKLMParam->fDynCrvRateFact;

    laneKalmanFilter_Right(&sLKFInputRi, &sLKFOutputRi);

    pKLMOutput->fPosY0KlmRi = sLKFOutputRi.sf_PosY0_met;
    pKLMOutput->fHeadingKlmRi = sLKFOutputRi.sf_HeadingAngle_rad;
    fRawCrvKlmRi = sLKFOutputRi.sf_Crv_1pm;
    pKLMOutput->fCrvRateKlmRi = sLKFOutputRi.sf_CrvChng_1pm2;
    pKLMOutput->uBtfStatusKlmRi = sLKFOutputRi.sf_KFStatus_btf;
    pKLMOutput->fQualityMeasureRi = sLKFOutputRi.sf_QualityMeasure_perc;

    /* Right lane curvature estimation */
    fTempKLM = pKLMInput->fValidLengthUlpRi - pKLMParam->fStartLengthForCrvEst;
    if ((fTempKLM < pKLMParam->fMinDistForCrvEst) ||
        (pKLMInput->fValidLengthUlpRi < pKLMParam->fStartLengthForCrvEst)) {
        fStartPosXForCrvRi = 0.0F;
    } else {
        fStartPosXForCrvRi = pKLMParam->fStartLengthForCrvEst;
    }
    fEndPosXForCrvRi = TUE_CML_Min_M(pKLMInput->fValidLengthUlpRi,
                                     pKLMParam->fEndLengthForCrvEst);

    for (UINT8_T ii = 0U; ii < 3U; ii++) {
        fPosXForCrvRi[ii] = fStartPosXForCrvRi +
                            ii * (fEndPosXForCrvRi - fStartPosXForCrvRi) / 2.0F;
        fPosYForCrvRi[ii] = TUE_CML_PosY3rd_M(
            fPosXForCrvRi[ii], sLKFOutputRi.sf_PosY0_met,
            sLKFOutputRi.sf_HeadingAngle_rad, sLKFOutputRi.sf_Crv_1pm,
            sLKFOutputRi.sf_CrvChng_1pm2);
    }

    /* calculate 2nd Order Polynomial */
    TUE_CML_Interp2_M(fPosXForCrvRi, fPosYForCrvRi, fCoeffForCrvRi);
    fTempKLM = 2 * fCoeffForCrvRi[0] * fPosXForCrvRi[1] + fCoeffForCrvRi[1];
    fTempKLM = (1 + fTempKLM * fTempKLM);
    fRawEstCrvKlmRi =
        (2 * fCoeffForCrvRi[0]) / (TUE_CML_Sqrt_M(fTempKLM) * fTempKLM);

    /* Curvature low pass filter */
    fTempKLM = (pKLMParam->fEndFltCrv - pKLMParam->fStartFltCrv);
    fTempKLM = 1.0F + (TUE_CML_Abs_M(fRawEstCrvKlmRi) - pKLMParam->fEndFltCrv) /
                          fTempKLM;
    fRawTiFltCrvEstRi =
        TUE_CML_Limit_M(fTempKLM, 1.0F, 0.0F) * pKLMParam->fTiFltForCurve;
    fTiFltCrvEstRi = TUE_CML_Max_M(fRawTiFltCrvEstRi, pKLMParam->fSysCycleTime);
    fTiFltCrvEstRi =
        TUE_CML_Max_M(fTiFltCrvEstRi, pKLMParam->fTiFltForStraight);

    fEstCrvKlmRi =
        TUE_CML_LowPassFilter_M(fRawEstCrvKlmRi, fTiFltCrvEstRi,
                                pKLMParam->fSysCycleTime, fLastEstCrvKlmRi);

    if (pKLMParam->bUseCrvEstimation == 0U) {
        pKLMOutput->fCrvKlmRi = fRawCrvKlmRi;
    } else {
        pKLMOutput->fCrvKlmRi = fEstCrvKlmRi;
    }

    /***************************Out and
     * Debug*********************************************/
    pKLMDebug->bValidLaneLf = bValidLaneLf;
    pKLMDebug->bEnaDegrUpdateLf = bEnaDegrUpdateLf;
    pKLMDebug->fRawCrvKlmLf = fRawCrvKlmLf;
    pKLMDebug->fStartPosXForCrvLf = fStartPosXForCrvLf;
    pKLMDebug->fEndPosXForCrvLf = fEndPosXForCrvLf;
    pKLMDebug->fRawEstCrvKlmLf = fRawEstCrvKlmLf;
    pKLMDebug->fRawTiFltCrvEstLf = fRawTiFltCrvEstLf;
    pKLMDebug->fTiFltCrvEstLf = fTiFltCrvEstLf;
    pKLMDebug->fEstCrvKlmLf = fEstCrvKlmLf;

    pKLMDebug->bValidLaneRi = bValidLaneRi;
    pKLMDebug->bEnaDegrUpdateRi = bEnaDegrUpdateRi;
    pKLMDebug->fRawCrvKlmRi = fRawCrvKlmRi;
    pKLMDebug->fStartPosXForCrvRi = fStartPosXForCrvRi;
    pKLMDebug->fEndPosXForCrvRi = fEndPosXForCrvRi;
    pKLMDebug->fRawEstCrvKlmRi = fRawEstCrvKlmRi;
    pKLMDebug->fRawTiFltCrvEstRi = fRawTiFltCrvEstRi;
    pKLMDebug->fTiFltCrvEstRi = fTiFltCrvEstRi;
    pKLMDebug->fEstCrvKlmRi = fEstCrvKlmRi;

    /***************************Save last
     * value*********************************************/
    fLastEstCrvKlmLf = fEstCrvKlmLf;
    fLastEstCrvKlmRi = fEstCrvKlmRi;
}

/****************************************************************************************
        @fn           INPUT_CheckFilterValidity
        @brief        CheckFilterValidity input iterface
        @description  CheckFilterValidity input iterface
        @param[in]    pLFPInput : Input from LFP input
        @param[in]    pLFPParam : Input from LFP param
        @param[in]    pLWVOutput : Input from LWV output
        @param[in]    pKLMOutput : Input from KLM output
        @param[out]   pCFVInput: Output for INPUT_CheckFilterValidity
        @param[out]   pCFVParam : Parameter for INPUT_CheckFilterValidity
        @return       void
        @startuml
        title INPUT_CheckFilterValidity
        (*)-->1.pLFPInput
           --> (*)
        (*)-->2.pLFPParam
           -->(*)
        (*)-->3.pLWVOutput
           -->(*)
        (*)-->4.pKLMOutput
           -->(*)
        @enduml
 ******************************************************************************************/
void INPUT_CheckFilterValidity(const sLFPInput_t *pLFPInput,
                               const sLFPParam_t *pLFPParam,
                               const sLWVOutput_t *pLWVOutput,
                               const sKLMOutput_t *pKLMOutput,
                               sCFVInput_t *pCFVInput,
                               sCFVParam_t *pCFVParam) {
    pCFVInput->bLaneChangeDtct = pLFPInput->bLaneChangeDtct;
    pCFVInput->bValidLaneWidth = pLWVOutput->bLaneWidthValid;
    pCFVInput->uBtfKalmanFilterLf = pKLMOutput->uBtfStatusKlmLf;
    pCFVInput->fQualityMeasureLf = pKLMOutput->fQualityMeasureLf;
    pCFVInput->fPosY0FltLf = pKLMOutput->fPosY0KlmLf;
    pCFVInput->fHeadingFltLf = pKLMOutput->fHeadingKlmLf;
    pCFVInput->fCrvFltLf = pKLMOutput->fCrvKlmLf;
    pCFVInput->fCrvRateFltLf = pKLMOutput->fCrvRateKlmLf;
    pCFVInput->uBtfKalmanFilterCntr = pKLMOutput->uBtfStatusKlmCntr;
    pCFVInput->fQualityMeasureCntr = pKLMOutput->fQualityMeasureCntr;
    pCFVInput->fPosY0FltCntr = pKLMOutput->fPosY0KlmCntr;
    pCFVInput->fHeadingFltCntr = pKLMOutput->fHeadingKlmCntr;
    pCFVInput->fCrvFltCntr = pKLMOutput->fCrvKlmCntr;
    pCFVInput->fCrvRateFltCntr = pKLMOutput->fCrvRateKlmCntr;
    pCFVInput->uBtfKalmanFilterRi = pKLMOutput->uBtfStatusKlmRi;
    pCFVInput->fQualityMeasureRi = pKLMOutput->fQualityMeasureRi;
    pCFVInput->fPosY0FltRi = pKLMOutput->fPosY0KlmRi;
    pCFVInput->fHeadingFltRi = pKLMOutput->fHeadingKlmRi;
    pCFVInput->fCrvFltRi = pKLMOutput->fCrvKlmRi;
    pCFVInput->fCrvRateFltRi = pKLMOutput->fCrvRateKlmRi;

    /*  */
    pCFVParam->fMinKalmanQuality = 3.0F;
    pCFVParam->fMaxDistYStep = LBP_fMaxDistYStep;
    pCFVParam->fMaxHeadingStep = LBP_fMaxHeadingStep;
    pCFVParam->fMaxCrvStep = LBP_fMaxCrvStep;
    pCFVParam->fMaxCrvRateStep = LBP_fMaxCrvRateStep;
    pCFVParam->fTdDistYStepValid = 1.0F;
    pCFVParam->fTdHeadingStepValid = 1.0F;
    pCFVParam->fTdCrvStepValid = 1.0F;
    pCFVParam->fTdCrvRateStepValid = 1.0F;
    pCFVParam->fMinQualityForBridge = 30.0F;
    pCFVParam->fSysCycleTime = pLFPParam->fSysCycleTime;
}

void CFV_LeftKalmanValidation(const sCFVInput_t *pCFVInput,
                              const sCFVParam_t *pCFVParam,
                              sCFVOutput_t *pCFVOutput,
                              sCFVDebug_t *pCFVDebug) {
    UINT8_T bEnaStepDtctLf =
        0U; /* Enable flag for left lane step detection, (0, 0~1, -) */
    REAL32_T fAbsDistYStepLf = 0.0F;
    UINT8_T bRawInValidDistYStepLf =
        0U; /* Raw last Left lane lateral distance step invalidity, (0, 0~1,
               -)*/
    UINT8_T bDlyValidDistYStepLf = 0U; /* Left lane lateral distance step
                                          invalidity after turn on delay, (0,
                                          0~1, -)*/
    UINT8_T bResetDistYStepLf = 0U;
    REAL32_T fAbsHeadingStepLf = 0.0F;
    UINT8_T bRawInValidHeadingStepLf =
        0U; /* Raw last left lane heading angle step invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidHeadingStepLf = 0U; /* Left lane heading angle step
                                            invalidity after turn on delay, (0,
                                            0~1, -)*/
    UINT8_T bResetHeadingStepLf = 0U;
    REAL32_T fAbsCrvStepLf = 0.0F;
    UINT8_T bRawInValidCrvStepLf =
        0U; /* Raw last Left lane curvature step invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidCrvStepLf = 0U; /* Left lane curvature step invalidity
                                        after turn on delay, (0, 0~1, -)*/
    UINT8_T bResetCrvStepLf = 0U;
    REAL32_T fAbsCrvRateStepLf = 0.0F;
    UINT8_T bRawInValidCrvRateStepLf =
        0U; /* Raw last left lane curvature rate step invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidCrvRateStepLf = 0U; /* Left lane curvature rate step
                                            invalidity after turn on delay, (0,
                                            0~1, -)*/
    UINT8_T bResetCrvRateStepLf = 0U;

    /***************************1.Left lane Kalman
     * validation******************************/
    /***************************1.1 Determine Kalman filter status
     * validity****************/
    if ((pCFVInput->uBtfKalmanFilterLf == KF_STATE_FULL_UPDATE) ||
        (pCFVInput->uBtfKalmanFilterLf == KF_STATE_DEGR_UPDATE) ||
        (pCFVInput->uBtfKalmanFilterLf == KF_STATE_PREDICTION)) {
        if (pCFVInput->fQualityMeasureLf >= pCFVParam->fMinKalmanQuality) {
            pCFVOutput->bValidKalmanFilterLf = 1U;
        } else {
            pCFVOutput->bValidKalmanFilterLf = 0U;
        }
    } else {
        pCFVOutput->bValidKalmanFilterLf = 0U;
    }

    /***************************1.2 Kalman filter step
     * detection***************************/
    /* Enable step detection if
            (1) the lane kalman filter is valid
            (2) No lane change has been detected */
    if ((pCFVOutput->bValidKalmanFilterLf == 1U) &&
        (pCFVInput->bLaneChangeDtct == 0U)) {
        bEnaStepDtctLf = 1U;
    } else {
        bEnaStepDtctLf = 0U;
    }

    /***************************1.2.1 Left lane lateral distance step
     * detection***************/
    /* Lateral position step exceeds maximum threshold */
    fAbsDistYStepLf = TUE_CML_Abs_M(pCFVInput->fPosY0FltLf - fLastPosY0FltLf);
    if ((fAbsDistYStepLf > pCFVParam->fMaxDistYStep) &&
        (bEnaStepDtctLf == 1U)) {
        /* Invalid flag for lateral position */
        bRawInValidDistYStepLf = 1U;
    } else {
        bRawInValidDistYStepLf = 0U;
    }

    /* Turn on delay for left lateral position valid flag */
    bDlyValidDistYStepLf = TUE_CML_TurnOnDelay_M(
        (!bRawInValidDistYStepLf), pCFVParam->fTdDistYStepValid,
        pCFVParam->fSysCycleTime, &fTimerDistYStepValidLf,
        bLastDlyValidDistYStepLf);

    /* S-R flip flops for left lateral position step*/
    if ((bDlyValidDistYStepLf == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetDistYStepLf = 1U;
    } else {
        bResetDistYStepLf = 0U;
    }
    pCFVOutput->bInValidDistYStepLf = TUE_CML_SRTrigger_M(
        bRawInValidDistYStepLf, bResetDistYStepLf, bLastInValidDistYStepLf);

    /***************************1.2.2 Left lane heading angle step
     * detection***************/
    /* Left lane heading angle step exceeds maximum threshold */
    fAbsHeadingStepLf =
        TUE_CML_Abs_M(pCFVInput->fHeadingFltLf - fLastHeadingFltLf);
    if ((fAbsHeadingStepLf > pCFVParam->fMaxHeadingStep) &&
        (bEnaStepDtctLf == 1U)) {
        /* Invalid flag for left lane heading angle */
        bRawInValidHeadingStepLf = 1U;
    } else {
        bRawInValidHeadingStepLf = 0U;
    }

    /* Valid flag for left lane heading angle */
    /* Turn on delay for left lane heading angle valid flag */
    bDlyValidHeadingStepLf = TUE_CML_TurnOnDelay_M(
        (!bRawInValidHeadingStepLf), pCFVParam->fTdHeadingStepValid,
        pCFVParam->fSysCycleTime, &fTimerHeadingStepValidLf,
        bLastDlyValidHeadingStepLf);
    /* S-R Flip Flops for left lane heading angle */
    if ((bDlyValidHeadingStepLf == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetHeadingStepLf = 1U;
    } else {
        bResetHeadingStepLf = 0U;
    }
    pCFVOutput->bInValidHeadingStepLf =
        TUE_CML_SRTrigger_M(bRawInValidHeadingStepLf, bResetHeadingStepLf,
                            bLastInValidHeadingStepLf);

    /***************************1.2.3 Left lane curvature step
     * detection***************/
    /* Left lane curvature step exceeds maximum threshold */
    fAbsCrvStepLf = TUE_CML_Abs_M(pCFVInput->fCrvFltLf - fLastCrvFltLf);
    if ((fAbsCrvStepLf > pCFVParam->fMaxCrvStep) && (bEnaStepDtctLf == 1U)) {
        /* Invalid flag for left lane curvature */
        bRawInValidCrvStepLf = 1U;
    } else {
        bRawInValidCrvStepLf = 0U;
    }

    /* Valid flag for left lane curvature */
    /* Turn on delay for left lane curvature valid flag */
    bDlyValidCrvStepLf = TUE_CML_TurnOnDelay_M(
        (!bRawInValidCrvStepLf), pCFVParam->fTdCrvStepValid,
        pCFVParam->fSysCycleTime, &fTimerCrvStepValidLf,
        bLastDlyValidCrvStepLf);
    /* S-R Flip Flops for left lane curvature */
    if ((bDlyValidCrvStepLf == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetCrvStepLf = 1U;
    } else {
        bResetCrvStepLf = 0U;
    }
    pCFVOutput->bInValidCrvStepLf = TUE_CML_SRTrigger_M(
        bRawInValidCrvStepLf, bResetCrvStepLf, bLastInValidCrvStepLf);

    /***************************1.2.3 Left lane curvature rate step
     * detection***************/
    /* Left lane curvature rate step exceeds maximum threshold */
    fAbsCrvRateStepLf =
        TUE_CML_Abs_M(pCFVInput->fCrvRateFltLf - fLastCrvRateFltLf);
    if ((fAbsCrvRateStepLf > pCFVParam->fMaxCrvRateStep) &&
        (bEnaStepDtctLf == 1U)) {
        /* Invalid flag for left lane curvature rate */
        bRawInValidCrvRateStepLf = 1U;
    } else {
        bRawInValidCrvRateStepLf = 0U;
    }

    /* Valid flag for left lane curvature rate */
    /* Turn on delay for left lane curvature rate valid flag */
    bDlyValidCrvRateStepLf = TUE_CML_TurnOnDelay_M(
        (!bRawInValidCrvRateStepLf), pCFVParam->fTdCrvRateStepValid,
        pCFVParam->fSysCycleTime, &fTimerCrvRateStepValidLf,
        bLastDlyValidCrvRateStepLf);
    /* S-R Flip Flops for left lane curvature rate */
    if ((bDlyValidCrvRateStepLf == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetCrvRateStepLf = 1U;
    } else {
        bResetCrvRateStepLf = 0U;
    }
    pCFVOutput->bInValidCrvRateStepLf =
        TUE_CML_SRTrigger_M(bRawInValidCrvRateStepLf, bResetCrvRateStepLf,
                            bLastInValidCrvRateStepLf);

    pCFVDebug->bEnaStepDtctLf = bEnaStepDtctLf;
    pCFVDebug->fAbsDistYStepLf = fAbsDistYStepLf;
    pCFVDebug->bRawInValidDistYStepLf = bRawInValidDistYStepLf;
    pCFVDebug->bDlyValidDistYStepLf = bDlyValidDistYStepLf;
    pCFVDebug->fTimerDistYStepValidLf = fTimerDistYStepValidLf;
    pCFVDebug->bResetDistYStepLf = bResetDistYStepLf;

    pCFVDebug->fAbsHeadingStepLf = fAbsHeadingStepLf;
    pCFVDebug->bRawInValidHeadingStepLf = bRawInValidHeadingStepLf;
    pCFVDebug->bDlyValidHeadingStepLf = bDlyValidHeadingStepLf;
    pCFVDebug->fTimerHeadingStepValidLf = fTimerHeadingStepValidLf;
    pCFVDebug->bResetHeadingStepLf = bResetHeadingStepLf;

    pCFVDebug->fAbsCrvStepLf = fAbsCrvStepLf;
    pCFVDebug->bRawInValidCrvStepLf = bRawInValidCrvStepLf;
    pCFVDebug->bDlyValidCrvStepLf = bDlyValidCrvStepLf;
    pCFVDebug->fTimerCrvStepValidLf = fTimerCrvStepValidLf;
    pCFVDebug->bResetCrvStepLf = bResetCrvStepLf;

    pCFVDebug->fAbsCrvRateStepLf = fAbsCrvRateStepLf;
    pCFVDebug->bRawInValidCrvRateStepLf = bRawInValidCrvRateStepLf;
    pCFVDebug->bDlyValidCrvRateStepLf = bDlyValidCrvRateStepLf;
    pCFVDebug->fTimerCrvRateStepValidLf = fTimerCrvRateStepValidLf;
    pCFVDebug->bResetCrvRateStepLf = bResetCrvRateStepLf;

    fLastPosY0FltLf = pCFVInput->fPosY0FltLf;
    fLastHeadingFltLf = pCFVInput->fHeadingFltLf;
    fLastCrvFltLf = pCFVInput->fCrvFltLf;
    fLastCrvRateFltLf = pCFVInput->fCrvRateFltLf;

    bLastDlyValidDistYStepLf = bDlyValidDistYStepLf;
    bLastInValidDistYStepLf = pCFVOutput->bInValidDistYStepLf;
    bLastDlyValidHeadingStepLf = bDlyValidHeadingStepLf;
    bLastInValidHeadingStepLf = pCFVOutput->bInValidHeadingStepLf;
    bLastDlyValidCrvStepLf = bDlyValidCrvStepLf;
    bLastInValidCrvStepLf = pCFVOutput->bInValidCrvStepLf;
    bLastDlyValidCrvRateStepLf = bDlyValidCrvRateStepLf;
    bLastInValidCrvRateStepLf = pCFVOutput->bInValidCrvRateStepLf;
}

void CFV_CenterKalmanValidation(const sCFVInput_t *pCFVInput,
                                const sCFVParam_t *pCFVParam,
                                sCFVOutput_t *pCFVOutput,
                                sCFVDebug_t *pCFVDebug) {
    UINT8_T bEnaStepDtctCntr =
        0U; /* Enable flag for center lane step detection, (0, 0~1, -) */
    REAL32_T fAbsDistYStepCntr = 0.0F;
    UINT8_T bRawInValidDistYStepCntr =
        0U; /* Raw last center lane lateral distance step invalidity, (0, 0~1,
               -)*/
    UINT8_T
    bDlyValidDistYStepCntr = 0U; /* center lane lateral distance step invalidity
                                    after turn on delay, (0, 0~1, -)*/
    UINT8_T bResetDistYStepCntr = 0U;
    REAL32_T fAbsHeadingStepCntr = 0.0F;
    UINT8_T bRawInValidHeadingStepCntr =
        0U; /* Raw last center lane heading angle step invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidHeadingStepCntr = 0U; /* center lane heading angle step
                                              invalidity after turn on delay,
                                              (0, 0~1, -)*/
    UINT8_T bResetHeadingStepCntr = 0U;
    REAL32_T fAbsCrvStepCntr = 0.0F;
    UINT8_T bRawInValidCrvStepCntr =
        0U; /* Raw last center lane curvature step invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidCrvStepCntr = 0U; /* center lane curvature step invalidity
                                          after turn on delay, (0, 0~1, -)*/
    UINT8_T bResetCrvStepCntr = 0U;
    REAL32_T fAbsCrvRateStepCntr = 0.0F;
    UINT8_T bRawInValidCrvRateStepCntr = 0U; /* Raw last center lane curvature
                                                rate step invalidity, (0, 0~1,
                                                -)*/
    UINT8_T bDlyValidCrvRateStepCntr = 0U;   /* center lane curvature rate step
                                                invalidity after turn on delay,
                                                (0, 0~1, -)*/
    UINT8_T bResetCrvRateStepCntr = 0U;

    /***************************3.1 Determine Kalman
     * filter status validity****************/
    if ((pCFVInput->uBtfKalmanFilterCntr == KF_STATE_FULL_UPDATE) ||
        (pCFVInput->uBtfKalmanFilterCntr == KF_STATE_DEGR_UPDATE) ||
        (pCFVInput->uBtfKalmanFilterCntr == KF_STATE_PREDICTION)) {
        if (pCFVInput->fQualityMeasureCntr >= pCFVParam->fMinKalmanQuality) {
            pCFVOutput->bValidKalmanFilterCntr = 1U;
        } else {
            pCFVOutput->bValidKalmanFilterCntr = 0U;
        }
    } else {
        pCFVOutput->bValidKalmanFilterCntr = 0U;
    }

    /***************************1.2 Kalman filter step
     * detection***************************/
    /* Enable step detection if
            (1) the lane kalman filter is valid
            (2) No lane change has been detected */
    if ((pCFVOutput->bValidKalmanFilterCntr == 1U) &&
        (pCFVInput->bLaneChangeDtct == 0U)) {
        bEnaStepDtctCntr = 1U;
    } else {
        bEnaStepDtctCntr = 0U;
    }

    /***************************1.2.1 center lane lateral distance step
     * detection***************/
    /* Lateral position step exceeds maximum threshold */
    fAbsDistYStepCntr =
        TUE_CML_Abs_M(pCFVInput->fPosY0FltCntr - fLastPosY0FltCntr);
    if ((fAbsDistYStepCntr > pCFVParam->fMaxDistYStep) &&
        (bEnaStepDtctCntr == 1U)) {
        /* Invalid flag for lateral position */
        bRawInValidDistYStepCntr = 1U;
    } else {
        bRawInValidDistYStepCntr = 0U;
    }

    /* Valid flag for lateral position */
    /* Turn on delay for center lateral position valid flag */
    bDlyValidDistYStepCntr = TUE_CML_TurnOnDelay_M(
        (!bRawInValidDistYStepCntr), pCFVParam->fTdDistYStepValid,
        pCFVParam->fSysCycleTime, &fTimerDistYStepValidCntr,
        bLastDlyValidDistYStepCntr);
    /* S-R flip flops for center lateral position step*/
    if ((bDlyValidDistYStepCntr == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetDistYStepCntr = 1U;
    } else {
        bResetDistYStepCntr = 0U;
    }
    pCFVOutput->bInValidDistYStepCntr =
        TUE_CML_SRTrigger_M(bRawInValidDistYStepCntr, bResetDistYStepCntr,
                            bLastInValidDistYStepCntr);

    /***************************1.2.2 center lane heading angle step
     * detection***************/
    /* center lane heading angle step exceeds maximum threshold */
    fAbsHeadingStepCntr =
        TUE_CML_Abs_M(pCFVInput->fHeadingFltCntr - fLastHeadingFltCntr);
    if ((fAbsHeadingStepCntr > pCFVParam->fMaxHeadingStep) &&
        (bEnaStepDtctCntr == 1U)) {
        /* Invalid flag for center lane heading angle */
        bRawInValidHeadingStepCntr = 1U;
    } else {
        bRawInValidHeadingStepCntr = 0U;
    }

    /* Valid flag for center lane heading angle */
    /* Turn on delay for center lane heading angle valid flag */
    bDlyValidHeadingStepCntr = TUE_CML_TurnOnDelay_M(
        (!bRawInValidHeadingStepCntr), pCFVParam->fTdHeadingStepValid,
        pCFVParam->fSysCycleTime, &fTimerHeadingStepValidCntr,
        bLastDlyValidHeadingStepCntr);
    /* S-R Flip Flops for center lane heading angle */
    if ((bDlyValidHeadingStepCntr == 1U) ||
        (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetHeadingStepCntr = 1U;
    } else {
        bResetHeadingStepCntr = 0U;
    }
    pCFVOutput->bInValidHeadingStepCntr =
        TUE_CML_SRTrigger_M(bRawInValidHeadingStepCntr, bResetHeadingStepCntr,
                            bLastInValidHeadingStepCntr);

    /***************************1.2.3 center lane curvature step
     * detection***************/
    /* center lane curvature step exceeds maximum threshold */
    fAbsCrvStepCntr = TUE_CML_Abs_M(pCFVInput->fCrvFltCntr - fLastCrvFltCntr);
    if ((fAbsCrvStepCntr > pCFVParam->fMaxCrvStep) &&
        (bEnaStepDtctCntr == 1U)) {
        /* Invalid flag for center lane curvature */
        bRawInValidCrvStepCntr = 1U;
    } else {
        bRawInValidCrvStepCntr = 0U;
    }

    /* Valid flag for center lane curvature */
    /* Turn on delay for center lane curvature valid flag */
    bDlyValidCrvStepCntr = TUE_CML_TurnOnDelay_M(
        (!bRawInValidCrvStepCntr), pCFVParam->fTdCrvStepValid,
        pCFVParam->fSysCycleTime, &fTimerCrvStepValidCntr,
        bLastDlyValidCrvStepCntr);
    /* S-R Flip Flops for center lane curvature */
    if ((bDlyValidCrvStepCntr == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetCrvStepCntr = 1U;
    } else {
        bResetCrvStepCntr = 0U;
    }
    pCFVOutput->bInValidCrvStepCntr = TUE_CML_SRTrigger_M(
        bRawInValidCrvStepCntr, bResetCrvStepCntr, bLastInValidCrvStepCntr);

    /***************************1.2.3 center lane curvature rate step
     * detection***************/
    /* center lane curvature rate step exceeds maximum threshold */
    fAbsCrvRateStepCntr =
        TUE_CML_Abs_M(pCFVInput->fCrvRateFltCntr - fLastCrvRateFltCntr);
    if ((fAbsCrvRateStepCntr > pCFVParam->fMaxCrvRateStep) &&
        (bEnaStepDtctCntr == 1U)) {
        /* Invalid flag for center lane curvature rate */
        bRawInValidCrvRateStepCntr = 1U;
    } else {
        bRawInValidCrvRateStepCntr = 0U;
    }

    /* Valid flag for center lane curvature rate */
    /* Turn on delay for center lane curvature rate valid flag */
    bDlyValidCrvRateStepCntr = TUE_CML_TurnOnDelay_M(
        (!bRawInValidCrvRateStepCntr), pCFVParam->fTdCrvRateStepValid,
        pCFVParam->fSysCycleTime, &fTimerCrvRateStepValidCntr,
        bLastDlyValidCrvRateStepCntr);
    /* S-R Flip Flops for center lane curvature rate */
    if ((bDlyValidCrvRateStepCntr == 1U) ||
        (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetCrvRateStepCntr = 1U;
    } else {
        bResetCrvRateStepCntr = 0U;
    }
    pCFVOutput->bInValidCrvRateStepCntr =
        TUE_CML_SRTrigger_M(bRawInValidCrvRateStepCntr, bResetCrvRateStepCntr,
                            bLastInValidCrvRateStepCntr);

    /* Determine lane bridge after Kalman validation */
    /* Lane bridging is possible if
            - lane kalman filter exceeds a certain threshold
            - the lane width is in a valid range
            - the kalman filter data are valid
            - quality exceeds a threshold
            - predict, update or degraded update has been triggered
            - kalman filter output data step detection is valid (no steps in
       output data detected)*/
    if ((pCFVInput->fQualityMeasureCntr > pCFVParam->fMinQualityForBridge) &&
        (pCFVInput->bValidLaneWidth == 1U) &&
        (pCFVOutput->bValidKalmanFilterCntr == 1U) &&
        (pCFVOutput->bInValidDistYStepCntr == 0U) &&
        (pCFVOutput->bInValidHeadingStepCntr == 0U) &&
        (pCFVOutput->bInValidCrvStepCntr == 0U) &&
        (pCFVOutput->bInValidCrvRateStepCntr == 0U)) {
        pCFVOutput->bPossibleLaneBridgeCntr = 1U;
    } else {
        pCFVOutput->bPossibleLaneBridgeCntr = 0U;
    }

    pCFVDebug->bEnaStepDtctCntr = bEnaStepDtctCntr;
    pCFVDebug->fAbsDistYStepCntr = fAbsDistYStepCntr;
    pCFVDebug->bRawInValidDistYStepCntr = bRawInValidDistYStepCntr;
    pCFVDebug->bDlyValidDistYStepCntr = bDlyValidDistYStepCntr;
    pCFVDebug->fTimerDistYStepValidCntr = fTimerDistYStepValidCntr;
    pCFVDebug->bResetDistYStepCntr = bResetDistYStepCntr;

    pCFVDebug->fAbsHeadingStepCntr = fAbsHeadingStepCntr;
    pCFVDebug->bRawInValidHeadingStepCntr = bRawInValidHeadingStepCntr;
    pCFVDebug->bDlyValidHeadingStepCntr = bDlyValidHeadingStepCntr;
    pCFVDebug->fTimerHeadingStepValidCntr = fTimerHeadingStepValidCntr;
    pCFVDebug->bResetHeadingStepCntr = bResetHeadingStepCntr;

    pCFVDebug->fAbsCrvStepCntr = fAbsCrvStepCntr;
    pCFVDebug->bRawInValidCrvStepCntr = bRawInValidCrvStepCntr;
    pCFVDebug->bDlyValidCrvStepCntr = bDlyValidCrvStepCntr;
    pCFVDebug->fTimerCrvStepValidCntr = fTimerCrvStepValidCntr;
    pCFVDebug->bResetCrvStepCntr = bResetCrvStepCntr;

    pCFVDebug->fAbsCrvRateStepCntr = fAbsCrvRateStepCntr;
    pCFVDebug->bRawInValidCrvRateStepCntr = bRawInValidCrvRateStepCntr;
    pCFVDebug->bDlyValidCrvRateStepCntr = bDlyValidCrvRateStepCntr;
    pCFVDebug->fTimerCrvRateStepValidCntr = fTimerCrvRateStepValidCntr;
    pCFVDebug->bResetCrvRateStepCntr = bResetCrvRateStepCntr;

    fLastPosY0FltCntr = pCFVInput->fPosY0FltCntr;
    fLastHeadingFltCntr = pCFVInput->fHeadingFltCntr;
    fLastCrvFltCntr = pCFVInput->fCrvFltCntr;
    fLastCrvRateFltCntr = pCFVInput->fCrvRateFltCntr;

    bLastDlyValidDistYStepCntr = bDlyValidDistYStepCntr;
    bLastInValidDistYStepCntr = pCFVOutput->bInValidDistYStepCntr;
    bLastDlyValidHeadingStepCntr = bDlyValidHeadingStepCntr;
    bLastInValidHeadingStepCntr = pCFVOutput->bInValidHeadingStepCntr;
    bLastDlyValidCrvStepCntr = bDlyValidCrvStepCntr;
    bLastInValidCrvStepCntr = pCFVOutput->bInValidCrvStepCntr;
    bLastDlyValidCrvRateStepCntr = bDlyValidCrvRateStepCntr;
    bLastInValidCrvRateStepCntr = pCFVOutput->bInValidCrvRateStepCntr;
}

/****************************************************************************************
        @fn           CheckFilterValidity(CheckKalmanFilterValidity)
        @brief        Check the lane validity after Kalman filtering
        @description  CheckFilterValidity:
                                          1.Left lane Kalman validation;
                                          2.Center lane Kalman validation;
                                          3.Right lane Kalman validation.
        @param[in]    CFVInput  : Input for CheckFilterValidity
        @param[in]    CFVParam  : Parameter for CheckFilterValidity
        @param[out]   CFVOutput : Output for LaneWidthValidity
        @param[out]   CFVDebug  : Debug(measurement) for CheckFilterValidity
        @return       void
        @startuml
        title CheckFilterValidity
        (*)--> 1.Left lane Kalman validation
           --> (*)
        (*)-->2.Center lane Kalman validation
           -->(*)
        (*)-->3.Right lane Kalman validation
           -->(*)
        @enduml
 *****************************************************************************************/
void CheckFilterValidity(const sCFVInput_t *pCFVInput,
                         const sCFVParam_t *pCFVParam,
                         sCFVOutput_t *pCFVOutput,
                         sCFVDebug_t *pCFVDebug) {
    UINT8_T bEnaStepDtctRi =
        0U; /* Enable flag for right lane step detection, (0, 0~1, -) */
    REAL32_T fAbsDistYStepRi = 0.0F;
    UINT8_T bRawInValidDistYStepRi =
        0U; /* Raw last right lane lateral distance step invalidity, (0, 0~1,
               -)*/
    UINT8_T bDlyValidDistYStepRi = 0U; /* right lane lateral distance step
                                          invalidity after turn on delay, (0,
                                          0~1, -)*/
    UINT8_T bResetDistYStepRi = 0U;
    REAL32_T fAbsHeadingStepRi = 0.0F;
    UINT8_T bRawInValidHeadingStepRi =
        0U; /* Raw last right lane heading angle step invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidHeadingStepRi = 0U; /* right lane heading angle step
                                            invalidity after turn on delay, (0,
                                            0~1, -)*/
    UINT8_T bResetHeadingStepRi = 0U;
    REAL32_T fAbsCrvStepRi = 0.0F;
    UINT8_T bRawInValidCrvStepRi =
        0U; /* Raw last right lane curvature step invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidCrvStepRi = 0U; /* right lane curvature step invalidity
                                        after turn on delay, (0, 0~1, -)*/
    UINT8_T bResetCrvStepRi = 0U;
    REAL32_T fAbsCrvRateStepRi = 0.0F;
    UINT8_T bRawInValidCrvRateStepRi =
        0U; /* Raw last right lane curvature rate step invalidity, (0, 0~1, -)*/
    UINT8_T bDlyValidCrvRateStepRi =
        0U; /* right lane curvature rate step invalidity after turn on delay,
               (0, 0~1, -)*/
    UINT8_T bResetCrvRateStepRi = 0U;

    /***************************1.Left lane Kalman
     * validation******************************/
    CFV_LeftKalmanValidation(pCFVInput, pCFVParam, pCFVOutput, pCFVDebug);

    /***************************2.Center lane Kalman
     * validation****************************/
    CFV_CenterKalmanValidation(pCFVInput, pCFVParam, pCFVOutput, pCFVDebug);

    /***************************3.Right lane Kalman
     * validation*****************************/
    /***************************3.1 Determine Kalman filter status
     * validity****************/
    if ((pCFVInput->uBtfKalmanFilterRi == KF_STATE_FULL_UPDATE) ||
        (pCFVInput->uBtfKalmanFilterRi == KF_STATE_DEGR_UPDATE) ||
        (pCFVInput->uBtfKalmanFilterRi == KF_STATE_PREDICTION)) {
        if (pCFVInput->fQualityMeasureRi >= pCFVParam->fMinKalmanQuality) {
            pCFVOutput->bValidKalmanFilterRi = 1U;
        } else {
            pCFVOutput->bValidKalmanFilterRi = 0U;
        }
    } else {
        pCFVOutput->bValidKalmanFilterRi = 0U;
    }

    /***************************1.2 Kalman filter step
     * detection***************************/
    /* Enable step detection if
            (1) the lane kalman filter is valid
            (2) No lane change has been detected */
    if ((pCFVOutput->bValidKalmanFilterRi == 1U) &&
        (pCFVInput->bLaneChangeDtct == 0U)) {
        bEnaStepDtctRi = 1U;
    } else {
        bEnaStepDtctRi = 0U;
    }

    /***************************1.2.1 Right lane lateral distance step
     * detection***************/
    /* Lateral position step exceeds maximum threshold */
    fAbsDistYStepRi = TUE_CML_Abs_M(pCFVInput->fPosY0FltRi - fLastPosY0FltRi);
    if ((fAbsDistYStepRi > pCFVParam->fMaxDistYStep) &&
        (bEnaStepDtctRi == 1U)) {
        /* Invalid flag for lateral position */
        bRawInValidDistYStepRi = 1U;
    } else {
        bRawInValidDistYStepRi = 0U;
    }

    /* Valid flag for lateral position */
    /* Turn on delay for Right lateral position valid flag */
    bDlyValidDistYStepRi = TUE_CML_TurnOnDelay_M(
        (!bRawInValidDistYStepRi), pCFVParam->fTdDistYStepValid,
        pCFVParam->fSysCycleTime, &fTimerDistYStepValidRi,
        bLastDlyValidDistYStepRi);
    /* S-R flip flops for Right lateral position step*/
    if ((bDlyValidDistYStepRi == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetDistYStepRi = 1U;
    } else {
        bResetDistYStepRi = 0U;
    }
    pCFVOutput->bInValidDistYStepRi = TUE_CML_SRTrigger_M(
        bRawInValidDistYStepRi, bResetDistYStepRi, bLastInValidDistYStepRi);

    /***************************1.2.2 Right lane heading angle step
     * detection***************/
    /* Right lane heading angle step exceeds maximum threshold */
    fAbsHeadingStepRi =
        TUE_CML_Abs_M(pCFVInput->fHeadingFltRi - fLastHeadingFltRi);
    if ((fAbsHeadingStepRi > pCFVParam->fMaxHeadingStep) &&
        (bEnaStepDtctRi == 1U)) {
        /* Invalid flag for Right lane heading angle */
        bRawInValidHeadingStepRi = 1U;
    } else {
        bRawInValidHeadingStepRi = 0U;
    }

    /* Valid flag for Right lane heading angle */
    /* Turn on delay for Right lane heading angle valid flag */
    bDlyValidHeadingStepRi = TUE_CML_TurnOnDelay_M(
        (!bRawInValidHeadingStepRi), pCFVParam->fTdHeadingStepValid,
        pCFVParam->fSysCycleTime, &fTimerHeadingStepValidRi,
        bLastDlyValidHeadingStepRi);
    /* S-R Flip Flops for Right lane heading angle */
    if ((bDlyValidHeadingStepRi == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetHeadingStepRi = 1U;
    } else {
        bResetHeadingStepRi = 0U;
    }
    pCFVOutput->bInValidHeadingStepRi =
        TUE_CML_SRTrigger_M(bRawInValidHeadingStepRi, bResetHeadingStepRi,
                            bLastInValidHeadingStepRi);

    /***************************1.2.3 Right lane curvature step
     * detection***************/
    /* Right lane curvature step exceeds maximum threshold */
    fAbsCrvStepRi = TUE_CML_Abs_M(pCFVInput->fCrvFltRi - fLastCrvFltRi);
    if ((fAbsCrvStepRi > pCFVParam->fMaxCrvStep) && (bEnaStepDtctRi == 1U)) {
        /* Invalid flag for Right lane curvature */
        bRawInValidCrvStepRi = 1U;
    } else {
        bRawInValidCrvStepRi = 0U;
    }

    /* Valid flag for Right lane curvature */
    /* Turn on delay for Right lane curvature valid flag */
    bDlyValidCrvStepRi = TUE_CML_TurnOnDelay_M(
        (!bRawInValidCrvStepRi), pCFVParam->fTdCrvStepValid,
        pCFVParam->fSysCycleTime, &fTimerCrvStepValidRi,
        bLastDlyValidCrvStepRi);
    /* S-R Flip Flops for Right lane curvature */
    if ((bDlyValidCrvStepRi == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetCrvStepRi = 1U;
    } else {
        bResetCrvStepRi = 0U;
    }
    pCFVOutput->bInValidCrvStepRi = TUE_CML_SRTrigger_M(
        bRawInValidCrvStepRi, bResetCrvStepRi, bLastInValidCrvStepRi);

    /***************************1.2.3 Right lane curvature rate step
     * detection***************/
    /* Right lane curvature rate step exceeds maximum threshold */
    fAbsCrvRateStepRi =
        TUE_CML_Abs_M(pCFVInput->fCrvRateFltRi - fLastCrvRateFltRi);
    if ((fAbsCrvRateStepRi > pCFVParam->fMaxCrvRateStep) &&
        (bEnaStepDtctRi == 1U)) {
        /* Invalid flag for Right lane curvature rate */
        bRawInValidCrvRateStepRi = 1U;
    } else {
        bRawInValidCrvRateStepRi = 0U;
    }

    /* Valid flag for Right lane curvature rate */
    /* Turn on delay for Right lane curvature rate valid flag */
    bDlyValidCrvRateStepRi = TUE_CML_TurnOnDelay_M(
        (!bRawInValidCrvRateStepRi), pCFVParam->fTdCrvRateStepValid,
        pCFVParam->fSysCycleTime, &fTimerCrvRateStepValidRi,
        bLastDlyValidCrvRateStepRi);
    /* S-R Flip Flops for Right lane curvature rate */
    if ((bDlyValidCrvRateStepRi == 1U) || (pCFVInput->bLaneChangeDtct == 1U)) {
        bResetCrvRateStepRi = 1U;
    } else {
        bResetCrvRateStepRi = 0U;
    }
    pCFVOutput->bInValidCrvRateStepRi =
        TUE_CML_SRTrigger_M(bRawInValidCrvRateStepRi, bResetCrvRateStepRi,
                            bLastInValidCrvRateStepRi);

    /**************************************Output and
     * debug******************************************************/

    pCFVDebug->bEnaStepDtctRi = bEnaStepDtctRi;
    pCFVDebug->fAbsDistYStepRi = fAbsDistYStepRi;
    pCFVDebug->bRawInValidDistYStepRi = bRawInValidDistYStepRi;
    pCFVDebug->bDlyValidDistYStepRi = bDlyValidDistYStepRi;
    pCFVDebug->fTimerDistYStepValidRi = fTimerDistYStepValidRi;
    pCFVDebug->bResetDistYStepRi = bResetDistYStepRi;

    pCFVDebug->fAbsHeadingStepRi = fAbsHeadingStepRi;
    pCFVDebug->bRawInValidHeadingStepRi = bRawInValidHeadingStepRi;
    pCFVDebug->bDlyValidHeadingStepRi = bDlyValidHeadingStepRi;
    pCFVDebug->fTimerHeadingStepValidRi = fTimerHeadingStepValidRi;
    pCFVDebug->bResetHeadingStepRi = bResetHeadingStepRi;

    pCFVDebug->fAbsCrvStepRi = fAbsCrvStepRi;
    pCFVDebug->bRawInValidCrvStepRi = bRawInValidCrvStepRi;
    pCFVDebug->bDlyValidCrvStepRi = bDlyValidCrvStepRi;
    pCFVDebug->fTimerCrvStepValidRi = fTimerCrvStepValidRi;
    pCFVDebug->bResetCrvStepRi = bResetCrvStepRi;

    pCFVDebug->fAbsCrvRateStepRi = fAbsCrvRateStepRi;
    pCFVDebug->bRawInValidCrvRateStepRi = bRawInValidCrvRateStepRi;
    pCFVDebug->bDlyValidCrvRateStepRi = bDlyValidCrvRateStepRi;
    pCFVDebug->fTimerCrvRateStepValidRi = fTimerCrvRateStepValidRi;
    pCFVDebug->bResetCrvRateStepRi = bResetCrvRateStepRi;

    /**************************************Save last
     * value************************************************/
    fLastPosY0FltRi = pCFVInput->fPosY0FltRi;
    fLastHeadingFltRi = pCFVInput->fHeadingFltRi;
    fLastCrvFltRi = pCFVInput->fCrvFltRi;
    fLastCrvRateFltRi = pCFVInput->fCrvRateFltRi;
    bLastDlyValidDistYStepRi = bDlyValidDistYStepRi;
    bLastInValidDistYStepRi = pCFVOutput->bInValidDistYStepRi;
    bLastDlyValidHeadingStepRi = bDlyValidHeadingStepRi;
    bLastInValidHeadingStepRi = pCFVOutput->bInValidHeadingStepRi;
    bLastDlyValidCrvStepRi = bDlyValidCrvStepRi;
    bLastInValidCrvStepRi = pCFVOutput->bInValidCrvStepRi;
    bLastDlyValidCrvRateStepRi = bDlyValidCrvRateStepRi;
    bLastInValidCrvRateStepRi = pCFVOutput->bInValidCrvRateStepRi;
}
/****************************************************************************************
        @fn           OUTPUT_LaneFilterProcessing
        @brief        LaneFilterProcessing output data
        @description  LaneFilterProcessing output data
        @param[in]    pLWVOutput : Input from LWV
        @param[in]    pKLMOutput : Input from KLM
        @param[in]    pCFVOutput : Input from CFV
        @param[out]   pLFPOutput: Output for pLFPOutput
        @return       void
        @startuml
        title OUTPUT_LaneFilterProcessing
        (*)--> pLWVOutput
           --> (*)
        (*)--> pKLMOutput
           --> (*)
        (*)--> pCFVOutput
           --> (*)
        @enduml
 ******************************************************************************************/
void OUTPUT_LaneFilterProcessing(const sLWVOutput_t *pLWVOutput,
                                 const sKLMOutput_t *pKLMOutput,
                                 const sCFVOutput_t *pCFVOutput,
                                 sLFPOutput_t *pLFPOutput) {
    pLFPOutput->uLaneValidQualifier = pLWVOutput->uLaneValidQualifer;
    pLFPOutput->uBridgePossible = pCFVOutput->bPossibleLaneBridgeCntr;
    pLFPOutput->bKalmanValidLf = pCFVOutput->bValidKalmanFilterLf;
    pLFPOutput->bDistYStepDebouncedLf = pCFVOutput->bInValidDistYStepLf;
    pLFPOutput->bHeadingStepDebouncedLf = pCFVOutput->bInValidHeadingStepLf;
    pLFPOutput->bCrvStepDebouncedLf = pCFVOutput->bInValidCrvStepLf;
    pLFPOutput->bCrvRateStepDebouncedLf = pCFVOutput->bInValidCrvRateStepLf;
    pLFPOutput->bKalmanValidCntr = pCFVOutput->bValidKalmanFilterCntr;
    pLFPOutput->bKalmanValidRi = pCFVOutput->bValidKalmanFilterRi;
    pLFPOutput->bDistYStepDebouncedRi = pCFVOutput->bInValidDistYStepRi;
    pLFPOutput->bHeadingStepDebouncedRi = pCFVOutput->bInValidHeadingStepRi;
    pLFPOutput->bCrvStepDebouncedRi = pCFVOutput->bInValidCrvStepRi;
    pLFPOutput->bCrvRateStepDebouncedRi = pCFVOutput->bInValidCrvRateStepRi;

    pLFPOutput->bDistYStepDebouncedCntr = pCFVOutput->bInValidDistYStepCntr;
    pLFPOutput->bHeadingStepDebouncedCntr = pCFVOutput->bInValidHeadingStepCntr;
    pLFPOutput->bCrvStepDebouncedCntr = pCFVOutput->bInValidCrvStepCntr;
    pLFPOutput->bCrvRateStepDebouncedCntr = pCFVOutput->bInValidCrvRateStepCntr;

    pLFPOutput->fFltQualityCntr = pKLMOutput->fQualityMeasureCntr;
    pLFPOutput->uFltStatusCntr = pKLMOutput->uBtfStatusKlmCntr;

    pLFPOutput->fPosX0FltLf = 0.0F;
    pLFPOutput->fPosY0FltLf = pKLMOutput->fPosY0KlmLf;
    pLFPOutput->fHeadingFltLf = pKLMOutput->fHeadingKlmLf;
    pLFPOutput->fCrvFltLf = pKLMOutput->fCrvKlmLf;
    pLFPOutput->fCrvRateFltLf = pKLMOutput->fCrvRateKlmLf;
    pLFPOutput->bLatDistDevLf = pKLMOutput->bLatDistDevLf;

    pLFPOutput->fPosX0FltCntr = 0.0F;
    pLFPOutput->fPosY0FltCntr = pKLMOutput->fPosY0KlmCntr;
    pLFPOutput->fHeadingFltCntr = pKLMOutput->fHeadingKlmCntr;
    pLFPOutput->fCrvFltCntr = pKLMOutput->fCrvKlmCntr;
    pLFPOutput->fCrvRateFltCntr = pKLMOutput->fCrvRateKlmCntr;

    pLFPOutput->fPosX0FltRi = 0.0F;
    pLFPOutput->fPosY0FltRi = pKLMOutput->fPosY0KlmRi;
    pLFPOutput->fHeadingFltRi = pKLMOutput->fHeadingKlmRi;
    pLFPOutput->fCrvFltRi = pKLMOutput->fCrvKlmRi;
    pLFPOutput->fCrvRateFltRi = pKLMOutput->fCrvRateKlmRi;
    pLFPOutput->bLatDistDevRi = pKLMOutput->bLatDistDevRi;

    pLFPOutput->fLaneWidth = pLWVOutput->fLaneWidth;
}

/****************************************************************************************
        @fn           INPUT_LaneFilterProcessing
        @brief        Input for LaneFilterProcessing
        @description  Input for LaneFilterProcessing:
                                        The input signal is processed uniformly
        @param[in]    pLBPInput : Input from external input
        @param[in]    pLBPParam : Input for external parameter
        @param[in]    pCLPOutput : Input from CLP output
        @param[out]   pLFPInput:  Input for INPUT_LaneFilterProcessing
        @param[out]   pLFPParam : Parameter for INPUT_LaneFilterProcessing
        @return       void
        @startuml
        title INPUT_LaneFilterProcessing
        (*)--> 1.pLBPInput
           --> (*)
        (*)--> 2.pLBPParam
           --> (*)
        (*)--> 3.pULPOutput
           --> (*)
        (*)--> 4.pCLPOutput
           --> (*)
        @enduml
 ******************************************************************************************/
void INPUT_LaneFilterProcessing(const sLBPInput_t *pLBPInput,
                                const sLBPParam_t *pLBPParam,
                                const sULPOutput_t *pULPOutput,
                                const sCLPOutput_t *pCLPOutput,
                                sLFPInput_t *pLFPInput,
                                sLFPParam_t *pLFPParam) {
    pLFPInput->fVehVelX = pLBPInput->fVehVelX;
    pLFPInput->bNewCorridorValid = pCLPOutput->bNewCorridorValid;  // Wrong
    pLFPInput->bUpDownHillDegrade = pCLPOutput->bUpDownHillDegrade;
    pLFPInput->fPosY0Lf = pULPOutput->fPosY0Lf;
    pLFPInput->fHeadingLf = pULPOutput->fHeadingLf;
    pLFPInput->uQualityLf = (UINT8_T)pULPOutput->fQualityLf;
    pLFPInput->uRangeCheckQualifierLf = pCLPOutput->uRangeCheckQualifier;
    pLFPInput->bNotAvailableLf = pCLPOutput->bNotAvailableLf;
    pLFPInput->bDistYStepDtctLf = pCLPOutput->bDistYStepDtctLf;
    pLFPInput->bLengthInvalidLf = pCLPOutput->bLengthInvalidLf;
    pLFPInput->bBridgeUnCplLf = pCLPOutput->bLaneVirtualCplLf;
    pLFPInput->bQualityNotValidLf = pCLPOutput->bLnQualityInvalidLf;
    pLFPInput->fPosY0Ri = pULPOutput->fPosY0Ri;
    pLFPInput->fHeadingRi = pULPOutput->fHeadingRi;
    pLFPInput->uQualityRi = (UINT8_T)pULPOutput->fQualityRi;
    pLFPInput->uRangeCheckQualifierRi = pCLPOutput->uRangeCheckQualifier;
    pLFPInput->bNotAvailableRi = pCLPOutput->bNotAvailableRi;
    pLFPInput->bDistYStepDtctRi = pCLPOutput->bDistYStepDtctRi;
    pLFPInput->bLengthInvalidRi = pCLPOutput->bLengthInvalidRi;
    pLFPInput->bBridgeUnCplRi = pCLPOutput->bLaneVirtualCplRi;
    pLFPInput->bQualityNotValidRi = pCLPOutput->bLnQualityInvalidRi;

    pLFPInput->bValidKlmFltCntr = 0U;
    pLFPInput->fOverallQualityLf = pULPOutput->fOverallQualityLf;
    pLFPInput->fOverallQualityRi = pULPOutput->fOverallQualityRi;
    pLFPInput->fPosY0UlpLf = pULPOutput->fPosY0Lf;
    pLFPInput->fHeadingUlpLf = pULPOutput->fHeadingLf;
    pLFPInput->fCrvUlpLf = pULPOutput->fCrvLf;
    pLFPInput->fCrvRateUlpLf = pULPOutput->fCrvRateLf;
    pLFPInput->fValidLengthUlpLf = pULPOutput->fValidLengthLf;
    pLFPInput->fPosY0UlpRi = pULPOutput->fPosY0Ri;
    pLFPInput->fHeadingUlpRi = pULPOutput->fHeadingRi;
    pLFPInput->fCrvUlpRi = pULPOutput->fCrvRi;
    pLFPInput->fCrvRateUlpRi = pULPOutput->fCrvRateRi;
    pLFPInput->fValidLengthUlpRi = pULPOutput->fValidLengthRi;
    pLFPInput->fVehYawRateStd = pLBPInput->fVehYawRateStd;
    pLFPInput->fVehYawRate = pLBPInput->fVehYawRate;
    pLFPInput->bLaneChangeDtct = pCLPOutput->bLaneChangeDtct;
    pLFPInput->fStraightDtct = pCLPOutput->fPercStraightDtct;

    /* */
    pLFPParam->fSysCycleTime = pLBPParam->fSysCycleTime;
}

/****************************************************************************************
        @fn LaneFilterProcessing(AnyBoundaryFilteringAndPlausibilization)
        @brief        Kalman filtering on ego lane and verify rationality
        @description  Lane filter processing:
                                          1.Lane width validity;
                                          2.Kalman filtering;
                                          3.CheckFilterValidity;
        @param[in]    pLFPInput : Input for LaneFilterProcessing
        @param[in]    pLFPParam : Parameter for LaneFilterProcessing ;
        @param[out]   pLFPOutput: Output for LaneFilterProcessing
        @param[out]   pLFPDebug : Debug(measurement) for LaneFilterProcessing
        @return       void
        @startuml
        title LaneFilterProcessing
        (*)-->1.LaneWidthValidity
           -->2.KalmanFiltering
           -->3.CheckFilterValidity
           -->(*)
        1.LaneWidthValidity -->3.CheckFilterValidity
        @enduml
 ******************************************************************************************/
void LaneFilterProcessing(sLFPInput_t const *pLFPInput,
                          sLFPParam_t const *pLFPParam,
                          sLFPOutput_t *pLFPOutput,
                          sLFPDebug_t *pLFPDebug) {
    sLWVInput_t sLWVInput = {0};
    sLWVParam_t sLWVParam = {0};
    sLWVOutput_t sLWVOutput = {0};
    sLWVDebug_t sLWVDebug = {0};

    sKLMInput_t sKLMInput = {0};
    sKLMParam_t sKLMParam = {0};
    sKLMOutput_t sKLMOutput = {0};
    sKLMDebug_t sKLMDebug = {0};

    sCFVInput_t sCFVInput = {0};
    sCFVParam_t sCFVParam = {0};
    sCFVDebug_t sCFVDebug = {0};

    /****************************1.Lane width
     * validity**************************************/
    INPUT_LaneWidthValidity(pLFPInput, pLFPParam, &sCFVOutput, &sLWVInput,
                            &sLWVParam);
    LaneWidthValidity(&sLWVInput, &sLWVParam, &sLWVOutput, &sLWVDebug);

    /****************************2.Kalman
     * filtering*****************************************/
    INPUT_LaneKalmanFilters(pLFPInput, pLFPParam, &sLWVOutput, &sCFVOutput,
                            &sKLMInput, &sKLMParam);
    LaneKalmanFilters(&sKLMInput, &sKLMParam, &sKLMOutput, &sKLMDebug);

    /****************************3.CheckFilterValidity**************************************/
    INPUT_CheckFilterValidity(pLFPInput, pLFPParam, &sLWVOutput, &sKLMOutput,
                              &sCFVInput, &sCFVParam);
    CheckFilterValidity(&sCFVInput, &sCFVParam, &sCFVOutput, &sCFVDebug);

    /****************************Output and
     * debug*******************************************/
    OUTPUT_LaneFilterProcessing(&sLWVOutput, &sKLMOutput, &sCFVOutput,
                                pLFPOutput);

    TUE_CML_MemoryCopy_M((void *)&sLWVInput, (void *)&(pLFPDebug->sLWVInput),
                         sizeof(sLWVInput_t));
    TUE_CML_MemoryCopy_M((void *)&sLWVParam, (void *)&(pLFPDebug->sLWVParam),
                         sizeof(sLWVParam_t));
    TUE_CML_MemoryCopy_M((void *)&sLWVOutput, (void *)&(pLFPDebug->sLWVOutput),
                         sizeof(sLWVOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sLWVDebug, (void *)&(pLFPDebug->sLWVDebug),
                         sizeof(sLWVDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sKLMInput, (void *)&(pLFPDebug->sKLMInput),
                         sizeof(sKLMInput_t));
    TUE_CML_MemoryCopy_M((void *)&sKLMParam, (void *)&(pLFPDebug->sKLMParam),
                         sizeof(sKLMParam_t));
    TUE_CML_MemoryCopy_M((void *)&sKLMOutput, (void *)&(pLFPDebug->sKLMOutput),
                         sizeof(sKLMOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sKLMDebug, (void *)&(pLFPDebug->sKLMDebug),
                         sizeof(sKLMDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sCFVInput, (void *)&(pLFPDebug->sCFVInput),
                         sizeof(sCFVInput_t));
    TUE_CML_MemoryCopy_M((void *)&sCFVParam, (void *)&(pLFPDebug->sCFVParam),
                         sizeof(sCFVParam_t));
    TUE_CML_MemoryCopy_M((void *)&sCFVOutput, (void *)&(pLFPDebug->sCFVOutput),
                         sizeof(sCFVOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sCFVDebug, (void *)&(pLFPDebug->sCFVDebug),
                         sizeof(sCFVDebug_t));
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
