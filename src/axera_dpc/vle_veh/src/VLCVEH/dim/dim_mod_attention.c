/*
 * Copyright (C) 2022-2024 by SoftwareMotion Group Limited. All rights reserved.
 * guotao 
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "./dim.h"
#include "./dim_mod_attention.h"
#include "dim_utils.h"

/*************************************************************************************************************************
  Functionname:    DIMAttentionEvalInputData */ /*!

                                                                             @brief
                                                                           Functional
                                                                           Summary

                                                                             @description
                                                                           Detailed
                                                                           Design

                                                                             @return
                                                                           static
                                                                           void

                                                                             @param[in]
                                                                           pDIM_attention_par
                                                                           :
                                                                             @param[in,out]
                                                                           pInternalData
                                                                           :
                                                                             @param[in]
                                                                           bDriverBraking
                                                                           :
                                                                             @param[in]
                                                                           fGasPedalPosition
                                                                           :
                                                                             @param[in]
                                                                           fGasPedalGradient
                                                                           :
                                                                             @param[in]
                                                                           uiTurnIndicator
                                                                           :
                                                                             @param[in]
                                                                           fAbsSteeringWheelGrad
                                                                           :
                                                                             @param[in]
                                                                           fLongAccel
                                                                           :
                                                                             @param[in]
                                                                           fLongVelocity
                                                                           :

                                                                             @pre
                                                                           [
                                                                           None
                                                                           ]
                                                                             @post
                                                                           [
                                                                           None
                                                                           ]
                                                                           *************************************************************************************************************************/
static void DIMAttentionEvalInputData(
    const DIM_ATTENTION_PAR_struct_t *pDIM_attention_par,
    DIMInternalDataModAttention_t *pInternalData,
    boolean bDriverBraking,
    float32 fGasPedalPosition,
    float32 fGasPedalGradient,
    uint8 uiTurnIndicator,
    float32 fAbsSteeringWheelGrad,
    float32 fLongAccel,
    float32 fLongVelocity);

/*************************************************************************************************************************
  Functionname:    DIMAttentionGetInputValues */ /*!

                                                                            @brief
                                                                          Functional
                                                                          Summary

                                                                            @description
                                                                          Detailed
                                                                          Design

                                                                            @return
                                                                          static
                                                                          void

                                                                            @param[in,out]
                                                                          pInternalData
                                                                          :
                                                                            @param[in]
                                                                          pInputData
                                                                          :
                                                                            @param[in,out]
                                                                          *bDriverBraking
                                                                          :
                                                                            @param[in,out]
                                                                          *fGasPedalPosition
                                                                          :
                                                                            @param[in,out]
                                                                          *fGasPedalGradient
                                                                          :
                                                                            @param[in,out]
                                                                          *uiTurnIndicator
                                                                          :
                                                                            @param[in,out]
                                                                          *fAbsSteeringWheelGrad
                                                                          :
                                                                            @param[in,out]
                                                                          *fLongAccel
                                                                          :
                                                                            @param[in,out]
                                                                          *fLongVelocity
                                                                          :

                                                                            @pre
                                                                          [ None
                                                                          ]
                                                                            @post
                                                                          [ None
                                                                          ]
                                                                          *************************************************************************************************************************/
static void DIMAttentionGetInputValues(
    DIMInternalDataModAttention_t *pInternalData,
    const DIMInputDataGlobal_t *pInputData,
    boolean *bDriverBraking,
    float32 *fGasPedalPosition,
    float32 *fGasPedalGradient,
    uint8 *uiTurnIndicator,
    float32 *fAbsSteeringWheelGrad,
    float32 *fLongAccel,
    float32 *fLongVelocity);

/*****************************************************************************
  @fn             DIMInitModuleAttention                                  */ /*!

    @brief          DIMInitModuleAttention

    @description

    @param[in]      pInternalData

    @return         error

    @pre            --

  ******************************************************************************/
eGDBError_t DIMInitModuleAttention(
    DIMInternalDataModAttention_t *pInternalData) {
    eGDBError_t eRetValue = GDB_ERROR_NONE;
    if (pInternalData != NULL) {
        pInternalData->fHighKeepTime = 0;
        pInternalData->fLowKeepTime = 0;
        pInternalData->fVeryHighKeepTime = 0;
        pInternalData->fHigherKeepTime = 0;
        pInternalData->fConstVelTime = 0;
        pInternalData->fNoGasPedalGradTime = 0;
        pInternalData->fConstVelocity = 0;
        pInternalData->iConfidence = 0;
        pInternalData->iProbability = 0;
    } else {
        eRetValue = GDB_ERROR_POINTER_NULL;
    }
    return eRetValue;
}

/*****************************************************************************
  @fn             DIMAttentionGetInputValues                                  */ /*!

@brief          collects input values from pInputData and make error handling

@description    Collects DriverBraking, TurnIndicator, SteeringWheelGrad,
LongVelocity,
                and LongAccel unconditionally.
                Aditionally external SpeedLimiter and KickDown is used for
conditionally setting
                GasPedalPosition and GasPedalGradient. Briefly in SpeedLimiter
mode GasPedal inputs
                are ignored for keeping Attention low (unknown). If SpeedLimiter
and KickDown is active
                GasPedalPosition is overwritten with GasPedalPosition for
KickDown to trigger
                very high attention (even in SpeedLimiter mode).

@param[in,out]   pInternalData
@param[in]       pInputData
@param[out]      bDriverBraking
@param[out]      fGasPedalPosition
@param[out]      fGasPedalGradient
@param[out]      uiTurnIndicator
@param[out]      fAbsSteeringWheelGrad
@param[out]      fLongAccel
@param[out]      fLongVelocity

@return         error

@pre            --

******************************************************************************/
static void DIMAttentionGetInputValues(
    DIMInternalDataModAttention_t *pInternalData,
    const DIMInputDataGlobal_t *pInputData,
    boolean *bDriverBraking,
    float32 *fGasPedalPosition,
    float32 *fGasPedalGradient,
    uint8 *uiTurnIndicator,
    float32 *fAbsSteeringWheelGrad,
    float32 *fLongAccel,
    float32 *fLongVelocity) {
    /*default supplementary signals*/
    boolean bSpeedLimiterActive = FALSE;

    pInternalData->iConfidence = DIM_ATTENTION_PAR_NormalConfidence;

    /*--- supplementary signals -> no Confidence degradation ---*/

    /*get SpeedLimiter signal, in case of NOK input keep NotActive (default) ->
     * normal DirverMonitoring, no degradation*/
    if (pInputData->SpeedLimiter.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Bool(pInputData->SpeedLimiter, FALSE,
                              &bSpeedLimiterActive);
    }

    /*--- primary signals -> with Confidence degradation ---*/

    /*get DriverBraking signal, in case of NOK keep default FALSE -> normal
     * DriverMonitoring "not braking"*/
    if (pInputData->DriverBraking.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Bool(pInputData->DriverBraking,
                              DIM_ATTENTION_DEFAULT_DrvBraking, bDriverBraking);
    } else {
        pInternalData->iConfidence -= DIM_ATTENTION_PAR_MissingConfidenceDelta;
    }

    /*get GasPedalPosition signal, in case of NOK keep default 100% -> normal
     * DriverMonitoring "full throttle"*/
    if (pInputData->GasPedalPosition.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Float(pInputData->GasPedalPosition,
                               DIM_ATTENTION_DEFAULT_GasPedalPos,
                               fGasPedalPosition);
    } else {
        pInternalData->iConfidence -= DIM_ATTENTION_PAR_MissingConfidenceDelta;
    }

    /*get GasPedalGradient signal, in case of NOK keep default 0 -> normal
     * DriverMonitoring "no gradient"*/
    if (pInputData->GasPedalGradient.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Float(pInputData->GasPedalGradient,
                               DIM_ATTENTION_DEFAULT_GasPedalGrad,
                               fGasPedalGradient);
    } else {
        pInternalData->iConfidence -= DIM_ATTENTION_PAR_MissingConfidenceDelta;
    }

    /*in case of a valid turn indicator signal - use this for evaluation - else
     * use the default "0.0" value*/
    if (pInputData->TurnIndicator.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_UnsigedInt(pInputData->TurnIndicator,
                                    DIM_ATTENTION_DEFAULT_TurnIndicator,
                                    uiTurnIndicator);
    } else {
        pInternalData->iConfidence -= DIM_ATTENTION_PAR_MissingConfidenceDelta;
    }

    /*in case of a valid steering wheel gradient signal - use this for
     * evaluation - else use the default "0.0" value*/
    if (pInputData->SteeringWheelGradient.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Float(pInputData->SteeringWheelGradient,
                               DIM_ATTENTION_DEFAULT_SteeringWheelGrad,
                               fAbsSteeringWheelGrad);
        (*fAbsSteeringWheelGrad) = fABS((*fAbsSteeringWheelGrad));
    } else if (pInputData->SteeringWheelGradient.eSignalQuality ==
               (uint8)DIMInputSignalState_BadQuality) {
        (*fAbsSteeringWheelGrad) =
            0; /*Multi-Turn-Loss shall not influence attention-confidence*/
    } else {
        pInternalData->iConfidence -= DIM_ATTENTION_PAR_MissingConfidenceDelta;
    }

    /*in case of a valid ego accel signal - use this for evaluation - else use
     * the default "0.0" value*/
    if (pInputData->VehicleAcceleration.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Float(pInputData->VehicleAcceleration,
                               DIM_ATTENTION_DEFAULT_LongAccel, fLongAccel);
    } else {
        pInternalData->iConfidence -= DIM_ATTENTION_PAR_MissingConfidenceDelta;
    }

    /*in case of a valid ego velocity signal - use this for evaluation - else
     * use the default "0.0" value*/
    if (pInputData->VehicleVelocity.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Float(pInputData->VehicleVelocity,
                               DIM_ATTENTION_DEFAULT_LongVelocity,
                               fLongVelocity);
    } else {
        pInternalData->iConfidence -= DIM_ATTENTION_PAR_MissingConfidenceDelta;
    }

    /*safeguarding Confidence degradation below 0%*/
    pInternalData->iConfidence = BML_Max(
        0,
        pInternalData->iConfidence); /*parameters can change -> safeguarding*/

    /*--- further signal treatment ---*/

    /*ignore GasPedal inputs while SpeedLimiter is active and no KickDown
     * detected*/
    if (TRUE == bSpeedLimiterActive) {
        if ((*fGasPedalPosition) <= DIM_ATTENTION_PAR_GasPedal_KickDown) {
            /*no KickDown detected*/
            /* Keep the pedal position at low value to avoid pedal-off
             * recognition */
            (*fGasPedalPosition) = DIM_ATTENTION_PAR_GasPedal_Passive_Value;
            (*fGasPedalGradient) = 0;
        }
    }
}

/*****************************************************************************
  @fn             DIMAttentionEvalInputData                                  */ /*!

 @brief          DIMAttentionEvalInputData

 @description

 @param[in]      pDIM_attention_par
 @param[in]      pInternalData
 @param[in]      bDriverBraking
 @param[in]      fGasPedalPosition
 @param[in]      fGasPedalGradient
 @param[in]      uiTurnIndicator
 @param[in]      fAbsSteeringWheelGrad
 @param[in]      fLongAccel
 @param[in]      fLongVelocity

 @return         error

 @pre            --

******************************************************************************/
static void DIMAttentionEvalInputData(
    const DIM_ATTENTION_PAR_struct_t *pDIM_attention_par,
    DIMInternalDataModAttention_t *pInternalData,
    boolean bDriverBraking,
    float32 fGasPedalPosition,
    float32 fGasPedalGradient,
    uint8 uiTurnIndicator,
    float32 fAbsSteeringWheelGrad,
    float32 fLongAccel,
    float32 fLongVelocity) {
    float32 fTemp = CML_f_CalculatePolygonValue(
        DIM_ATTENTION_PAR_AccelCurve_POINTS, pDIM_attention_par->fAccelCurve,
        fLongVelocity);
    if (fLongAccel > fTemp) {
        pInternalData->fVeryHighKeepTime = pDIM_attention_par->fVeryHighTime;
        pInternalData->fHigherKeepTime = pDIM_attention_par->fHigherTime;
        pInternalData->fHighKeepTime = pDIM_attention_par->fHighTime;
        pInternalData->fLowKeepTime = pDIM_attention_par->fLowTime;
    } else {
        /* check for higher attention */
        if ((bDriverBraking != FALSE) &&
            (fLongVelocity < pDIM_attention_par->fBrakePedalMaxVelocity) &&
            (fLongAccel <= pDIM_attention_par->fDriverBrakingMaxAcceleration)) {
            pInternalData->fHigherKeepTime = pDIM_attention_par->fHigherTime;
            pInternalData->fHighKeepTime = pDIM_attention_par->fHighTime;
            pInternalData->fLowKeepTime = pDIM_attention_par->fLowTime;
        } else {
            fTemp = CML_f_CalculatePolygonValue(
                DIM_ATTENTION_PAR_GasPedalPosCurve_POINTS,
                pDIM_attention_par->fGasPedalPosCurve, fLongVelocity);
            /* check for high attention */
            if ((bDriverBraking != FALSE) || (fGasPedalPosition > fTemp) ||
                (((uiTurnIndicator == eTurnIndicator_Left) ||
                  (uiTurnIndicator == eTurnIndicator_Right)) &&
                 (fLongVelocity >=
                  pDIM_attention_par->fTurnIndicatorMinVelocity))) {
                pInternalData->fHighKeepTime = pDIM_attention_par->fHighTime;
                pInternalData->fLowKeepTime = pDIM_attention_par->fLowTime;
            } else {
                /* check for low attention */
                if ((fGasPedalGradient >
                     pDIM_attention_par->fGasPedalGradLowPositive) ||
                    (fGasPedalGradient <
                     pDIM_attention_par->fGasPedalGradLowNegative) ||
                    ((fGasPedalGradient <
                      pDIM_attention_par->fGasPedalLowNegMeasured) &&
                     (fGasPedalPosition <
                      pDIM_attention_par->fGasPedalLowNegMeasuredPos)) ||
                    (fAbsSteeringWheelGrad >
                     pDIM_attention_par->fSteeringGradLow)) {
                    pInternalData->fLowKeepTime = pDIM_attention_par->fLowTime;
                } else {
                    /*nothing to do for unclear situations*/
                }
            }
        }
    }

    /* check for drive robot */

    /* check for low or unknown state and constant velocity */
    if ((!(pInternalData->fHighKeepTime > 0)) &&
        (!(pInternalData->fHigherKeepTime > 0)) &&
        (!(pInternalData->fVeryHighKeepTime > 0)) && (fLongVelocity > 0) &&
        (fLongVelocity >
         (pInternalData->fConstVelocity -
          pDIM_attention_par->fRobotControlledVelocityThresh)) &&
        (fLongVelocity <
         (pInternalData->fConstVelocity +
          pDIM_attention_par->fRobotControlledVelocityThresh))) {
        /* velocity currently constant */
        if (pInternalData->fConstVelTime > 0) {
            /* velocity not constant for a certain time
            reset timer for gas pedal gradient */
            pInternalData->fNoGasPedalGradTime =
                pDIM_attention_par->fNoGasPedalGradientTime;
        } else {
            /* velocity constant for a certain time
            check for test case conditions: gas pedal position > 0 && no gas
            pedal gradient */
            if ((fGasPedalPosition > 0) &&
                (fGasPedalGradient <
                 pDIM_attention_par->fNoGasPedalGradientThresh) &&
                (fGasPedalGradient >
                 -pDIM_attention_par->fNoGasPedalGradientThresh)) {
                /* conditions for decreasing timer of gas pedal gradient met */
                if (pInternalData->fNoGasPedalGradTime > 0) {
                    /* there was a gas pedal gradient within a certain time*/
                } else {
                    /* now all conditions for robot detection are met */
                    pInternalData->fLowKeepTime =
                        MIN_FLOAT(pInternalData->fLowKeepTime,
                                  pDIM_attention_par->fLowKeepRobotTime);
                }
            } else {
                /* conditions for deccreasing timer of gas pedal gradient not
                 * met: reset timer */
                pInternalData->fNoGasPedalGradTime =
                    pDIM_attention_par->fNoGasPedalGradientTime;
            }
        }
    } else {
        /* velocity not constant enough -> reset timer and store current
         * velocity*/
        pInternalData->fConstVelTime =
            pDIM_attention_par->fRobotControlledVelocityTime;
        pInternalData->fNoGasPedalGradTime =
            pDIM_attention_par->fNoGasPedalGradientTime;
        pInternalData->fConstVelocity = fLongVelocity;
    }
}

/*****************************************************************************
  @fn             DIMRunModuleAttention                                  */ /*!

     @brief          DIMRunModuleAttention

     @description

     @param[in]      fCycleTime
     @param[in]      pInputData
     @param[in]      pOutHypothesis
     @param[in]      pInternalData
     @param[in]      pDIM_attention_par

     @return         error

     @pre            --

   ******************************************************************************/
eGDBError_t DIMRunModuleAttention(
    const float32 fCycleTime,
    const DIMInputDataGlobal_t *pInputData,
    GDB_DMHypothesis_t *pOutHypothesis,
    DIMInternalDataModAttention_t *pInternalData,
    const DIM_ATTENTION_PAR_struct_t *pDIM_attention_par) {
    eGDBError_t eRetValue = GDB_ERROR_NONE;

    /*default value for driver braking*/
    boolean bDriverBraking = DIM_ATTENTION_DEFAULT_DrvBraking;
    /*default value for gas pedal position*/
    float32 fGasPedalPosition = DIM_ATTENTION_DEFAULT_GasPedalPos;
    /*default value for gas pedal gradient*/
    float32 fGasPedalGradient = DIM_ATTENTION_DEFAULT_GasPedalGrad;
    /*default value for turn indicator 0 - means off*/
    uint8 uiTurnIndicator = DIM_ATTENTION_DEFAULT_TurnIndicator;
    /*default value for steering wheel gradient*/
    float32 fAbsSteeringWheelGrad = DIM_ATTENTION_DEFAULT_SteeringWheelGrad;
    float32 fLongAccel =
        DIM_ATTENTION_DEFAULT_LongAccel; /*default value for long accel*/
    float32 fLongVelocity =
        DIM_ATTENTION_DEFAULT_LongVelocity; /*default value for long velocity*/

    if ((pInputData != NULL) && (pOutHypothesis != NULL) &&
        (pInternalData != NULL)) {
        DIMAttentionGetInputValues(pInternalData, pInputData, &bDriverBraking,
                                   &fGasPedalPosition, &fGasPedalGradient,
                                   &uiTurnIndicator, &fAbsSteeringWheelGrad,
                                   &fLongAccel, &fLongVelocity);

        pInternalData->fVeryHighKeepTime =
            MAX_FLOAT(pInternalData->fVeryHighKeepTime - fCycleTime, 0);
        pInternalData->fHigherKeepTime =
            MAX_FLOAT(pInternalData->fHigherKeepTime - fCycleTime, 0);
        pInternalData->fHighKeepTime =
            MAX_FLOAT(pInternalData->fHighKeepTime - fCycleTime, 0);
        pInternalData->fLowKeepTime =
            MAX_FLOAT(pInternalData->fLowKeepTime - fCycleTime, 0);
        pInternalData->fConstVelTime =
            MAX_FLOAT(pInternalData->fConstVelTime - fCycleTime, 0);
        pInternalData->fNoGasPedalGradTime =
            MAX_FLOAT(pInternalData->fNoGasPedalGradTime - fCycleTime, 0);

        /* check for very high attention */
        DIMAttentionEvalInputData(
            pDIM_attention_par, pInternalData, bDriverBraking,
            fGasPedalPosition, fGasPedalGradient, uiTurnIndicator,
            fAbsSteeringWheelGrad, fLongAccel, fLongVelocity);

        if (pInternalData->fVeryHighKeepTime > 0) {
            pInternalData->iProbability = DIM_ATTENTION_PAR_VeryHighPercentage;
        } else if (pInternalData->fHigherKeepTime > 0) {
            pInternalData->iProbability = DIM_ATTENTION_PAR_HigherPercentage;
        } else if (pInternalData->fHighKeepTime > 0) {
            pInternalData->iProbability = DIM_ATTENTION_PAR_HighPercentage;
        } else if (pInternalData->fLowKeepTime > 0) {
            pInternalData->iProbability = DIM_ATTENTION_PAR_LowPercentage;
        } else {
            pInternalData->iProbability = DIM_ATTENTION_PAR_UnknownPercentage;
        }

        pOutHypothesis->eType = DIMHypoType_Attention;
        pOutHypothesis->Confidence = (percentage_t)pInternalData->iConfidence;
        pOutHypothesis->Probability = (sint8)pInternalData->iProbability;
    } else {
        eRetValue = GDB_ERROR_POINTER_NULL;
    }

    return eRetValue;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */