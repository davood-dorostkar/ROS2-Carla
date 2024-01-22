/*
 * Copyright (C) 2022-2024 by SoftwareMotion Group Limited. All rights reserved.
 * guotao 
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "../dim.h"
#include "./dim_mod_feedback.h"
#include "./dim_utils.h"

static void DIMFeedbackGetInputValues(
    const DIMInputDataGlobal_t* pInputData,
    DIMInternalDataModFeedback_t* pInternalData,
    boolean* bDriverBraking,
    float32* fGasPedalPosition,
    float32* fGasPedalGradient,
    float32* fLongAccel);
/*****************************************************************************
  @fn             DIMInitModuleFeedback                                  */ /*!

     @brief          DIMInitModuleFeedback

     @description

     @param[in]      pInternalData

     @return         error

     @pre            --

   ******************************************************************************/
eGDBError_t DIMInitModuleFeedback(DIMInternalDataModFeedback_t* pInternalData) {
    eGDBError_t eRetValue = GDB_ERROR_NONE;
    if (pInternalData != NULL) {
        pInternalData->iConfidence = 0;
        pInternalData->iProbability = 0;
        pInternalData->fPosFeedbackTime = 0;
        pInternalData->fNegFeedbackTime = 0;
        pInternalData->fNegWeakFeedbackTime = 0;

        pInternalData->fFallIntoBrakeTime = 0;
        pInternalData->fCurrentAccelGrad = 0;
        pInternalData->bDriverWasBraking = FALSE;
        pInternalData->uiAccelBufferPointer = 0;
        for (uint8 uIndex = 0; uIndex < DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE;
             ++uIndex) {
            pInternalData->iAccelBuffer[uIndex] = 0;
            pInternalData->bBoolBuffer[uIndex] = FALSE;
        }
    } else {
        eRetValue = GDB_ERROR_POINTER_NULL;
    }
    return eRetValue;
}

/*****************************************************************************
  @fn             DIMRunModuleFeedback                                  */ /*!

      @brief          DIMRunModuleFeedback

      @description

      @param[in]      fCycleTime
      @param[in]      pInputData
      @param[in]      pOutHypothesis
      @param[in]      pInternalData
      @param[in]      pDIM_feedback_par

      @return         error

      @pre            --

    ******************************************************************************/
eGDBError_t DIMRunModuleFeedback(
    float32 fCycleTime,
    const DIMInputDataGlobal_t* pInputData,
    GDB_DMHypothesis_t* pOutHypothesis,
    DIMInternalDataModFeedback_t* pInternalData,
    const DIM_FEEDBACK_PAR_struct_t* pDIM_feedback_par) {
    eGDBError_t eRetValue = GDB_ERROR_NONE;

    /*default value for driver braking*/
    boolean bDriverBraking = DIM_FEEDBACK_DEFAULT_DrvBraking;
    /*default value for gas pedal position*/
    float32 fGasPedalPosition = DIM_FEEDBACK_DEFAULT_GasPedalPos;
    /*default value for gas pedal gradient*/
    float32 fGasPedalGradient = DIM_FEEDBACK_DEFAULT_GasPedalGrad;
    /*default value for long accel*/
    float32 fLongAccel = DIM_FEEDBACK_DEFAULT_LongAccel;

    if ((pInputData != NULL) && (pOutHypothesis != NULL) &&
        (pInternalData != NULL)) {
        /*check for missing update*/
        if (fCycleTime < BML_f_Delta) {
            /*this error should never occur. Missing cycle time (derived from
             * scheduler control data must be caught in the input port
             * monitoring*/
            /*division by zero protection put here for failsafe -> assuming
             * normal call timing*/
            eRetValue = GDB_ERROR_ZERO_DEVISION;
            fCycleTime = VLC_VEH_CYCLE_TIME;
        }

        pInternalData->iConfidence = DIM_FEEDBACK_PAR_NormalConfidence;

        //  Get Input Signal of DIM Feedback
        DIMFeedbackGetInputValues(pInputData, pInternalData, &bDriverBraking,
                                  &fGasPedalPosition, &fGasPedalGradient,
                                  &fLongAccel);
        pInternalData->iConfidence =
            (pInternalData->iConfidence < 0) ? 0 : pInternalData->iConfidence;
        /*count down the timers*/
        pInternalData->fNegFeedbackTime =
            MAX_FLOAT(pInternalData->fNegFeedbackTime - fCycleTime, 0);
        pInternalData->fNegWeakFeedbackTime =
            MAX_FLOAT(pInternalData->fNegWeakFeedbackTime - fCycleTime, 0);

        pInternalData->fPosFeedbackTime =
            MAX_FLOAT(pInternalData->fPosFeedbackTime - fCycleTime, 0);

        /*---- monitor current vehicle accel ----*/

        /*move pointer to new location*/
        pInternalData->uiAccelBufferPointer =
            (pInternalData->uiAccelBufferPointer + 1u) %
            DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE;

        /*getting nr of frames from the timer*/
        float32 fTemp =
            (DIM_FEEDBACK_DEF_GRAD_FILTER_TIME + (0.5f * fCycleTime)) /
            fCycleTime; /* physical constant C. Obst 26.08.14 */

        uint8 uiNrOfFrames = (uint8)fTemp;
        uiNrOfFrames =
            MAX(MIN(DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE, uiNrOfFrames), 1u);

        /*getting information from the buffer*/

        pInternalData->bDriverWasBraking = FALSE;
        for (uint32 uiTemp = 0u; (uiTemp < uiNrOfFrames) &&
                                 (pInternalData->bDriverWasBraking == FALSE);
             uiTemp++) {
            if (pInternalData->bBoolBuffer
                    [(((uint32)pInternalData->uiAccelBufferPointer +
                       DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE) -
                      uiTemp) %
                     DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE] == TRUE) {
                pInternalData->bDriverWasBraking = TRUE;
            }
        }

        /*calculating gradient between current accel and accel uiNrOfFrames
         * before*/
        fTemp = (fLongAccel -
                 (((float32)pInternalData
                       ->iAccelBuffer[((pInternalData->uiAccelBufferPointer +
                                        DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE) -
                                       uiNrOfFrames) %
                                      DIM_FEEDBACK_DEF_ACCELBUFFER_SIZE]) /
                  DIM_FEEDBACK_ACCEL_BUFFER_SCALE)) /
                ((float32)uiNrOfFrames * fCycleTime);

        /*store current accel grad to internal data*/
        pInternalData->fCurrentAccelGrad = fTemp;
        /*this error should never occur. Missing cycle time (derived from
         * scheduler control data must be caught in the input port monitoring*/
        /*division by zero protection put here for failsafe*/

        /*setting the jerk buffer to neutral value*/

        /*store current values into the list*/
        fTemp = fLongAccel * DIM_FEEDBACK_ACCEL_BUFFER_SCALE;
        fTemp = MINMAX_FLOAT(
            -120.0f, 120.0f,
            fTemp); /* make sure that sint8 isn't exceeded C. Obst 26.08.14 */
        pInternalData->iAccelBuffer[pInternalData->uiAccelBufferPointer] =
            (sint8)fTemp;
        pInternalData->bBoolBuffer[pInternalData->uiAccelBufferPointer] =
            bDriverBraking;

        /*---- evaluate driver feedback ----*/
        if ((  // normal threshold in case of no auto-brake jerk or driver was
               // braking
                ((fGasPedalPosition >= pDIM_feedback_par->fHighGasPedalPos) ||
                 (fGasPedalGradient >=
                  pDIM_feedback_par
                      ->fHighGasPedalGrad)) &&  // to avoid negative feedback
                                                // due to "falling" into gas
                                                // pedal after a autobrake jerk
                ((pInternalData->bDriverWasBraking == TRUE)))
            /* higher threshold is always active*/
            || (fGasPedalPosition >= pDIM_feedback_par->fVeryHighGasPedalPos) ||
            (fGasPedalGradient >= pDIM_feedback_par->fVeryHighGasPedalGrad)) {
            pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckNeg;
            pInternalData->fNegFeedbackTime = pDIM_feedback_par->fNegFdbckTime;
        } else if (pInternalData->fNegFeedbackTime > 0) {
            pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckNeg;
        } else if ((fGasPedalPosition >=
                    pDIM_feedback_par->fHighGasPedalPosWeakNegH) ||
                   ((fGasPedalGradient >=
                     pDIM_feedback_par->fHighGasPedalGradWeakNeg) &&
                    (fGasPedalPosition >=
                     pDIM_feedback_par->fHighGasPedalPosWeakNegL))) {
            pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckWeakNeg;
            pInternalData->fNegWeakFeedbackTime =
                pDIM_feedback_par->fNegFdbckTime;
        } else if (pInternalData->fNegWeakFeedbackTime > 0) {
            float32 fProb = CML_f_CalculatePolygonValue(
                DIM_FEEDBACK_PAR_WeakNegCurve_POINTS,
                pDIM_feedback_par->fWeakNegCurve,
                pInternalData->fNegWeakFeedbackTime);
            pInternalData->iProbability = (sint16)(ROUND_TO_INT(fProb));
            /* Driver braking active */
        } else if (bDriverBraking == TRUE) {
            pInternalData->fPosFeedbackTime = 0;
            pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckStrongPos;
            /* No accelerator, check accelerator gradient */
        } else if (fGasPedalPosition < pDIM_feedback_par->fGasPedalUsedPos) {
            /* Left accelerator, init positive feedback keep timer */
            if (fGasPedalGradient < pDIM_feedback_par->fGasPedelGradNegative) {
                pInternalData->fPosFeedbackTime =
                    pDIM_feedback_par->fPosFdbckTime;
            }
            /* Just left accelerator */
            if (pInternalData->fPosFeedbackTime > 0) {
                pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckPos;
            } else {
                /* Left accelerator some time ago */
                pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckNoPos;
            }
            /* almost constant accelerator */
        } else if (fABS(fGasPedalGradient) <
                   pDIM_feedback_par->fHighGasPedalGrad) {
            pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckNoNeg;
            /* Leaving accelerator */
        } else if (fGasPedalGradient <=
                   (-pDIM_feedback_par->fHighGasPedalGrad)) {
            pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckWeakPos;
            /* This should not happen, go to save level */
        } else {
            pInternalData->iProbability = DIM_FEEDBACK_PAR_FdbckNoPos;
        }

        pOutHypothesis->eType = DIMHypoType_Feedback;

        pOutHypothesis->Confidence = (percentage_t)pInternalData->iConfidence;
        pOutHypothesis->Probability = (sint8)(pInternalData->iProbability);
    } else {
        eRetValue = GDB_ERROR_POINTER_NULL;
    }
    return eRetValue;
}

static void DIMFeedbackGetInputValues(
    const DIMInputDataGlobal_t* pInputData,
    DIMInternalDataModFeedback_t* pInternalData,
    boolean* bDriverBraking,
    float32* fGasPedalPosition,
    float32* fGasPedalGradient,
    float32* fLongAccel) {
    /*in case of a valid driver braking signal - use this for evaluation - else
     * use the default "false" value*/
    if (pInputData->DriverBraking.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        // DIMGetInputValue(pInputData->DriverBraking, bDriverBraking,
        // DIM_FEEDBACK_DEFAULT_DrvBraking, boolean);
        DIMGetInputValue_Bool(pInputData->DriverBraking,
                              DIM_FEEDBACK_DEFAULT_DrvBraking, bDriverBraking);
    } else {
        pInternalData->iConfidence -= DIM_FEEDBACK_PAR_MissingConfidenceDelta;
    }
    /*in case of a valid gas pedal position signal - use this for evaluation -
     * else use the default "100.0" value*/
    if (pInputData->GasPedalPosition.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        // DIMGetInputValue(pInputData->GasPedalPosition, fGasPedalPosition,
        // DIM_FEEDBACK_DEFAULT_GasPedalPos, float32);
        DIMGetInputValue_Float(pInputData->GasPedalPosition,
                               DIM_FEEDBACK_DEFAULT_GasPedalPos,
                               fGasPedalPosition);
    } else {
        pInternalData->iConfidence -= DIM_FEEDBACK_PAR_MissingConfidenceDelta;
    }
    /*in case of a valid gas pedal gradient signal - use this for evaluation -
     * else use the default "0.0" value*/
    if (pInputData->GasPedalGradient.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        // DIMGetInputValue(pInputData->GasPedalGradient, fGasPedalGradient,
        // DIM_FEEDBACK_DEFAULT_GasPedalGrad, float32);
        DIMGetInputValue_Float(pInputData->GasPedalGradient,
                               DIM_FEEDBACK_DEFAULT_GasPedalGrad,
                               fGasPedalGradient);
    } else {
        pInternalData->iConfidence -= DIM_FEEDBACK_PAR_MissingConfidenceDelta;
    }
    /*in case of a valid ego accel signal - use this for evaluation - else use
     * the default "0.0" value*/
    if (pInputData->VehicleAcceleration.eSignalQuality ==
        (uint8)DIMInputSignalState_OK) {
        // DIMGetInputValue(pInputData->VehicleAcceleration, fLongAccel,
        // DIM_FEEDBACK_DEFAULT_LongAccel, float32);        /* redundant casting
        // needed for type safety */
        DIMGetInputValue_Float(pInputData->VehicleAcceleration,
                               DIM_FEEDBACK_DEFAULT_LongAccel, fLongAccel);
    } else {
        pInternalData->iConfidence -= DIM_FEEDBACK_PAR_MissingConfidenceDelta;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */