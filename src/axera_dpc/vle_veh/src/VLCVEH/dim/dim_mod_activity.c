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
#include "./dim_mod_activity.h"
#include "./dim_utils.h"

static void DIMActivityGetInputValues(
    DIMInternalDataModActivity_t *pInternalData,
    const DIMInputDataGlobal_t *pInputData,
    float32 *fAbsSteeringWheelAngle,
    float32 *fAbsSteeringWheelGrad,
    float32 *fLongVelocity);

static void DIMActivityEvalInputSignals(
    const DIM_ACTIVITY_PAR_struct_t *pDIM_activity_par,
    DIMInternalDataModActivity_t *pInternalData,
    float32 fAbsSteeringWheelAngle,
    float32 fAbsSteeringWheelGrad,
    float32 fLongVelocity,
    const float32 fCycleTime);
/*****************************************************************************
  @fn             DIMInitModuleActivity                                  */ /*!

     @brief          DIMInitModuleActivity

     @description    this function is used to initialise the DIM module activity
   hypothesis internal data parameters

     @param[out]     pInternalData

     @return         error

     @pre            --

   ******************************************************************************/
eGDBError_t DIMInitModuleActivity(DIMInternalDataModActivity_t *pInternalData) {
    eGDBError_t eRetValue = GDB_ERROR_NONE;
    if (pInternalData != NULL) {
        pInternalData->fActivityTimer = 0;
        pInternalData->fActivityHoldTimer = 0;
        pInternalData->fGradPeakTimer = 0;
        pInternalData->iConfidence = 0;
        pInternalData->iProbability = 0;
        pInternalData->fEmergencySteerTimer = 0;
        pInternalData->fGradLowPassOutput = 0;
        pInternalData->fGradLowPassTimer = 0;
    } else {
        eRetValue = GDB_ERROR_POINTER_NULL;
    }
    return eRetValue;
}

/*****************************************************************************
  @fn             DIMActivityGetInputValues                                  */ /*!

 @brief          DIMActivityGetInputValues

 @description    this function checks the quality of various input signals
related to DIM activity hypothesis
                 and save in the corresponding variables if the signals quality
is OK

 @param[out]     pInternalData
 @param[in]      pInputData
 @param[out]     fAbsSteeringWheelAngle
 @param[out]     fAbsSteeringWheelGrad
 @param[out]     fLongVelocity

 @return         void

 @pre            --

******************************************************************************/
static void DIMActivityGetInputValues(
    DIMInternalDataModActivity_t *pInternalData,
    const DIMInputDataGlobal_t *pInputData,
    float32 *fAbsSteeringWheelAngle,
    float32 *fAbsSteeringWheelGrad,
    float32 *fLongVelocity) {
    pInternalData->iConfidence = DIM_ACTIVITY_PAR_NormalConfidence;

    /*in case of a valid steering wheel gradient signal - use this for
     * evaluation - else use the default "0.0" value*/
    if (pInputData->SteeringWheelGradient.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Float(pInputData->SteeringWheelGradient,
                               DIM_ACTIVITY_DEFAULT_SteeringWheelGrad,
                               fAbsSteeringWheelGrad);
        (*fAbsSteeringWheelGrad) = fABS((*fAbsSteeringWheelGrad));
    } else if (pInputData->SteeringWheelGradient.eSignalQuality ==
               (uint8)DIMInputSignalState_BadQuality) {
        (*fAbsSteeringWheelGrad) =
            0; /*Multi-Turn-Loss shall not influence actitvity-confidence*/
    } else {
        pInternalData->iConfidence -= DIM_ACTIVITY_PAR_MissingConfidenceDelta;
    }
    /*in case of a valid steering wheel angle signal - use this for evaluation -
     * else use the default "0.0" value*/
    if (pInputData->SteeringWheelAngle.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Float(pInputData->SteeringWheelAngle,
                               DIM_ACTIVITY_DEFAULT_SteeringWheelAngle,
                               fAbsSteeringWheelAngle);
        (*fAbsSteeringWheelAngle) = fABS((*fAbsSteeringWheelAngle));
    } else if (pInputData->SteeringWheelAngle.eSignalQuality ==
               (uint8)DIMInputSignalState_BadQuality) {
        (*fAbsSteeringWheelAngle) =
            0; /*Multi-Turn-Loss shall not influence actitvity-confidence*/
    } else {
        pInternalData->iConfidence -= DIM_ACTIVITY_PAR_MissingConfidenceDelta;
    }
    /*in case of a valid ego velocity signal - use this for evaluation - else
     * use the default "0.0" value*/
    if (pInputData->VehicleVelocity.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        DIMGetInputValue_Float(pInputData->VehicleVelocity,
                               DIM_ACTIVITY_DEFAULT_LongVelocity,
                               fLongVelocity);
    } else {
        pInternalData->iConfidence -= DIM_ACTIVITY_PAR_MissingConfidenceDelta;
    }

    if (pInternalData->iConfidence < 0) {
        pInternalData->iConfidence = 0;
    }
}

/*****************************************************************************
  @fn             DIMActivityEvalInputSignals */ /*!

                                @brief          DIMActivityEvalInputSignals

                                @description    this function evaluates the
                              various input signals of the DIM module activity
                              hypothesis
                                                like fAbsSteeringWheelAngle,
                              fAbsSteeringWheelGrad and fLongVelocity. Based on
                              the evaluation
                                                it sets various timers like
                              fActivityTimer, fActivityHoldTimer,
                              fEmergencySteerTimer and
                                                fGradPeakTimer

                                @param[in]      pDIM_activity_par
                                @param[out]     pInternalData
                                @param[in]      fAbsSteeringWheelAngle
                                @param[in]      fAbsSteeringWheelGrad
                                @param[in]      fLongVelocity
                                @param[in]      fCycleTime

                                @return         void

                                @pre            --

                              ******************************************************************************/
static void DIMActivityEvalInputSignals(
    const DIM_ACTIVITY_PAR_struct_t *pDIM_activity_par,
    DIMInternalDataModActivity_t *pInternalData,
    float32 fAbsSteeringWheelAngle,
    float32 fAbsSteeringWheelGrad,
    float32 fLongVelocity,
    const float32 fCycleTime) {
    if (fAbsSteeringWheelAngle >
        CML_f_CalculatePolygonValue(DIM_ACTIVITY_PAR_FronSteerThres_POINTS,
                                    pDIM_activity_par->fFronSteerThres,
                                    fLongVelocity)) {
        pInternalData->fActivityTimer =
            MAX_FLOAT(pDIM_activity_par->fAngleShutDownTime,
                      pInternalData->fActivityTimer);
    }

    if (fAbsSteeringWheelGrad >
        CML_f_CalculatePolygonValue(DIM_ACTIVITY_PAR_SteerAngleGradThres_POINTS,
                                    pDIM_activity_par->fSteerAngleGradThres,
                                    fLongVelocity)) {
        /* Stabilize DIM against short gradient peaks */
        pInternalData->fGradPeakTimer =
            pInternalData->fGradPeakTimer + fCycleTime;

        if (pInternalData->fGradPeakTimer > pDIM_activity_par->fGradPeakTime) {
            pInternalData->fActivityTimer =
                MAX_FLOAT(pDIM_activity_par->fGradShutDownTime,
                          pInternalData->fActivityTimer);
        }
    } else {
        pInternalData->fGradPeakTimer = 0;
    }

    if (fAbsSteeringWheelAngle > pDIM_activity_par->fFronSteerThresStraight) {
        pInternalData->fActivityHoldTimer =
            MAX_FLOAT(pDIM_activity_par->fAngleHoldTime,
                      pInternalData->fActivityHoldTimer);
    }

    if (fAbsSteeringWheelGrad >
        pDIM_activity_par->fFronSteerGradThresStraight) {
        pInternalData->fActivityHoldTimer =
            MAX_FLOAT(pDIM_activity_par->fGradHoldTime,
                      pInternalData->fActivityHoldTimer);
    }

    if (fAbsSteeringWheelGrad >
        CML_f_CalculatePolygonValue(
            DIM_ACTIVITY_PAR_SteerAngleGradEMThres_POINTS,
            pDIM_activity_par->fSteerAngleGradEMThres, fLongVelocity)) {
        pInternalData->fEmergencySteerTimer =
            MAX_FLOAT(pDIM_activity_par->fGradShutDownTimeEM,
                      pInternalData->fEmergencySteerTimer);
    }

    /* calculate the low pass filter output for the signal
     * "fAbsSteeringWheelGrad" */
    pInternalData->fGradLowPassOutput =
        pInternalData->fGradLowPassOutput +
        (pDIM_activity_par->fFilterFactorSteeringGrad *
         (fAbsSteeringWheelGrad - pInternalData->fGradLowPassOutput));

    /* check if the threshold is met regarding the low pass filtered signal */
    if (pInternalData->fGradLowPassOutput >
        CML_f_CalculatePolygonValue(
            DIM_ACTIVITY_PAR_fSteeringAngleGradFilterThres_POINTS,
            pDIM_activity_par->fSteeringAngleGradFilterThres, fLongVelocity)) {
        pInternalData->fGradLowPassTimer =
            MAX_FLOAT(pInternalData->fGradLowPassTimer,
                      pDIM_activity_par->fSteeringGradFiltHoldTime);
    }
}

/*****************************************************************************
  @fn             DIMRunModuleActivity                                  */ /*!

      @brief          DIMRunModuleActivity

      @description    this function runs the DIM Activity module and calculates
    various hypothesis
                      parameters like Probability, Confidence and eType

      @param[in]      fCycleTime
      @param[in]      pInputData
      @param[out]     pOutHypothesis
      @param[out]     pInternalData
      @param[in]      pDIM_activity_par

      @return         error

      @pre            --

    ******************************************************************************/
eGDBError_t DIMRunModuleActivity(
    const float32 fCycleTime,
    const DIMInputDataGlobal_t *pInputData,
    GDB_DMHypothesis_t *pOutHypothesis,
    DIMInternalDataModActivity_t *pInternalData,
    const DIM_ACTIVITY_PAR_struct_t *pDIM_activity_par) {
    eGDBError_t eRetValue = GDB_ERROR_NONE;

    /*default value for steering wheel angle*/
    float32 fAbsSteeringWheelAngle = DIM_ACTIVITY_DEFAULT_SteeringWheelAngle;
    /*default value for steering wheel gradient*/
    float32 fAbsSteeringWheelGrad = DIM_ACTIVITY_DEFAULT_SteeringWheelGrad;
    float32 fLongVelocity =
        DIM_ACTIVITY_DEFAULT_LongVelocity; /*default value for long velocity*/

    if ((pInputData != NULL) && (pOutHypothesis != NULL) &&
        (pInternalData != NULL)) {
        DIMActivityGetInputValues(pInternalData, pInputData,
                                  &fAbsSteeringWheelAngle,
                                  &fAbsSteeringWheelGrad, &fLongVelocity);

        pInternalData->fActivityTimer =
            MAX_FLOAT(pInternalData->fActivityTimer - fCycleTime, 0);
        pInternalData->fEmergencySteerTimer =
            MAX_FLOAT(pInternalData->fEmergencySteerTimer - fCycleTime, 0);
        pInternalData->fActivityHoldTimer =
            MAX_FLOAT(pInternalData->fActivityHoldTimer - fCycleTime, 0);
        pInternalData->fGradLowPassTimer =
            MAX_FLOAT(pInternalData->fGradLowPassTimer - fCycleTime, 0);

        DIMActivityEvalInputSignals(
            pDIM_activity_par, pInternalData, fAbsSteeringWheelAngle,
            fAbsSteeringWheelGrad, fLongVelocity, fCycleTime);

        if (pInternalData->fActivityTimer > 0) {
            pInternalData->iProbability = DIM_ACTIVITY_PAR_VeryActivePercentage;
        } else if (pInternalData->fGradLowPassTimer > 0) {
            pInternalData->iProbability = DIM_ACTIVITY_PAR_ActivePercentage;
        } else {
            pInternalData->iProbability = 0;
        }

        /*in case of constant straight steering angle for the last time, the
         * activity shall be 0*/
        if ((fAbsSteeringWheelAngle <
             pDIM_activity_par->fFronSteerThresStraight) &&
            (fAbsSteeringWheelGrad <
             pDIM_activity_par->fFronSteerGradThresStraight) &&
            (pInternalData->fEmergencySteerTimer < C_F32_DELTA) &&
            (pInternalData->fActivityHoldTimer < C_F32_DELTA)) {
            pInternalData->iProbability = 0;
            /* reset the timer for consistency */
            pInternalData->fActivityTimer = 0;
            /* reset the Steering wheel angle gradient filter and the
             * corresponding timer */
            pInternalData->fGradLowPassOutput = 0;
            pInternalData->fGradLowPassTimer = 0;
        }

        /*in case of an emergency steering situation (and all signals
         * available), the probability is always 100%!*/
        if ((pInternalData->fEmergencySteerTimer > 0) &&
            (pInternalData->iConfidence == DIM_ACTIVITY_PAR_NormalConfidence)) {
            pInternalData->iProbability =
                DIM_ACTIVITY_PAR_EmergenySteeringPercentage;
        }

        pOutHypothesis->eType = DIMHypoType_Activity;
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