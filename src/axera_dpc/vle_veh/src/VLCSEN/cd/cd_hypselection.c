/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include <string.h>

#include "cd.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

static void CDCalculateHypothesisRelevance(
    CDIntHypothesis_t *pHypothesis,
    const CDObjectData_t *pObjectData,
    const CDInternalObjectList_t *rgObjInternal);

static fTime_t CDCorrectSingleTimingValueByLatency(fTime_t timingValue);

/* ***********************************************************************
  @fn            CDCalculateHypothesisRelevance */
static void CDCalculateHypothesisRelevance(
    CDIntHypothesis_t *pHypothesis,
    const CDObjectData_t *pObjectData,
    const CDInternalObjectList_t *rgObjInternal) {
    static const float32 CDHypRelevance[CD_NUMBER_OF_HYPOTHESES_TYPES] = {
        0,        /* CDNoHypothesis */
        10000.0f, /* CDRunUpHypothesis */
        10000.0f, /* CDRunUpBrakingHypothesis */
        8000.0f,  /* CDRunUpStationaryHypothesis */
        3000.0f,  /* CDStaticHypothesis */
        7500.0f,  /* CDAccHypothesis */
        1000.0f,  /* CDPassHypothesis */
        5000.0f,  /* CDCutInHypothesis */
        20000.0f, /* CDCollisionHypothesis */
        40000.0f, /* CDCollisionUnavoidableHypothesis */
        20000.0f, /* CDPedCollision */
        10000.0f, /* CDPedPass */
        15000.0f, /* CDCrossingHypothesis */
        20000.0f  /* CDBicycleCollision */
    };

    pHypothesis->fRelevance =
        (CDHypRelevance[pHypothesis->eType] *
         pHypothesis->fHypothesisProbability *
         ((float32)CD_EBA_OBJ_QUALITY(pObjectData, pHypothesis->iObjectRef) *
          (1.f / 100.0f))) -
        (*rgObjInternal)[pHypothesis->iObjectRef].LongNecAccel;
    return;
}

/* **********************************************************************
  @fn            CDHypothesesSelection */
void CDHypothesesSelection(CDIntHypothesis_t *pHypothesis,
                           const CDObjectData_t *pObjectData,
                           CDInternalStatus_t *pInternalStatus) {
    uint8 uiHypothesis;
    uint8 uiLeastCritHypothesis;
    boolean bHypNotRelevant;

    bHypNotRelevant = FALSE;
    uiHypothesis = 0u;

    /* get criticality of hypothesis */
    CDCalculateHypothesisRelevance(
        pHypothesis, pObjectData,
        (const CDInternalObjectList_t *)pInternalStatus->rgObjInternal);

    /* pre-filter: do not store pass hypothesis on object if a run-up or cut-in
     * exists for the object */
    if (pHypothesis->eType == CDHypothesisType_Pass) {
        for (uiHypothesis = 0u;
             uiHypothesis < pInternalStatus->uiNofRelevantHypotheses;
             uiHypothesis++) {
            const CDIntHypothesis_t *const currentRelHypo = &(
                (*pInternalStatus->rgIntRelevantHypothesesList)[uiHypothesis]);
            if ((currentRelHypo->iObjectRef == pHypothesis->iObjectRef) &&
                ((currentRelHypo->eType == CDHypothesisType_RunUp) ||
                 (currentRelHypo->eType == CDHypothesisType_RunUpBraking) ||
                 (currentRelHypo->eType == CDHypothesisType_CutIn)) &&
                (currentRelHypo->fHypothesisProbability >
                 CD_COMMON_HYP_PROB_IRREL_THRES)) {
                bHypNotRelevant = TRUE;
            }
        }
    } else if (pHypothesis->eType == CDHypothesisType_CutIn) {
        for (uiHypothesis = 0u;
             uiHypothesis < pInternalStatus->uiNofRelevantHypotheses;
             uiHypothesis++) {
            const CDIntHypothesis_t *const currentRelHypo = &(
                (*pInternalStatus->rgIntRelevantHypothesesList)[uiHypothesis]);
            if ((currentRelHypo->iObjectRef == pHypothesis->iObjectRef) &&
                ((currentRelHypo->eType == CDHypothesisType_RunUp) ||
                 (currentRelHypo->eType == CDHypothesisType_RunUpBraking)) &&
                (currentRelHypo->fHypothesisProbability >
                 CD_COMMON_HYP_PROB_IRREL_THRES)) {
                bHypNotRelevant = TRUE;
            }
        }
    } else {
        /* Nothing to Do! */
        /* pre-filter: do not store pass hypothesis on object if a run-up or
         * cut-in exists for the object */
    }

    /* prefilter: do not store cut-in hypothesis on object if a run-up exists
     * for the object */

    /* two cases has to be handled:
       case a : there are 'free' slot in the array (just add the hypothesis)
       case b : array is full. remove least critical hypothesis (this may be the
       new one)
     */
    if (bHypNotRelevant == TRUE) {
        /* do nothing, a more precise hypothesis already exists*/
    } else {
        if (pInternalStatus->uiNofRelevantHypotheses <
            CD_NUMBER_OF_HYPOTHESES) /* case a */
        {
            /* copy hypothesis to first free slot*/
            (*pInternalStatus->rgIntRelevantHypothesesList)
                [pInternalStatus->uiNofRelevantHypotheses] = *pHypothesis;
            /* increase number of found hypothesis*/
            pInternalStatus->uiNofRelevantHypotheses++;
        } else /* case b */
        {
            /* find least important hypothesis */
            /* initialize least important hypothesis with hypothesis 0 */
            uiLeastCritHypothesis = 0u;
            /* compare hypothesis with least critical hypothesis */
            for (uiHypothesis = 0u; uiHypothesis < CD_NUMBER_OF_HYPOTHESES;
                 uiHypothesis++) {
                /* if criticality of hypothesis is below the current least
                   critical one,
                   update the least critical hypothesis index*/
                if ((*pInternalStatus
                          ->rgIntRelevantHypothesesList)[uiHypothesis]
                        .fRelevance <
                    (*pInternalStatus
                          ->rgIntRelevantHypothesesList)[uiLeastCritHypothesis]
                        .fRelevance) {
                    uiLeastCritHypothesis = uiHypothesis;
                }
            }
            /* if new hypothesis is more critical than least critical hypothesis
               from array,
               replace the least critical one */
            if ((*pInternalStatus
                      ->rgIntRelevantHypothesesList)[uiLeastCritHypothesis]
                    .fRelevance < pHypothesis->fRelevance) {
                (*pInternalStatus
                      ->rgIntRelevantHypothesesList)[uiLeastCritHypothesis] =
                    *pHypothesis;
            }
        }
    }

    return;
}

/* ***********************************************************************
  @fn            CDPrepareCycleHypothesesSelection */ /*!


                       @brief         initializes the relevant hypotheses data

                       @description   Function resets the array
                     CDRelevantHypothesis and the NumberOf hypotheses

                       @param[in,out] pInternalStatus Pointer to the CD internal
                     status

                       @return        void

                       @pre           [none]

                       @post          [none]

                     ****************************************************************************
                     */
void CDPrepareCycleHypothesesSelection(CDInternalStatus_t *pInternalStatus) {
    uint32 uiHypothesis;

    /* store hypotheses from last cycle */
    (void)memcpy(pInternalStatus->rgPreviousHypothesesList,
                 pInternalStatus->rgIntRelevantHypothesesList,
                 sizeof(CDIntHypothesisList_t));
    pInternalStatus->uiNofPreviousHypotheses =
        pInternalStatus->uiNofRelevantHypotheses;

    /* resets hypotheses */
    pInternalStatus->uiNofRelevantHypotheses = 0u;

    /* resets array*/
    for (uiHypothesis = 0u; uiHypothesis < CD_NUMBER_OF_HYPOTHESES;
         uiHypothesis++) {
        CDIntHypothesis_t *const currentRelHypo =
            &((*pInternalStatus->rgIntRelevantHypothesesList)[uiHypothesis]);
        currentRelHypo->eType = CDHypothesisType_No;
        currentRelHypo->fRelevance = 0;
        currentRelHypo->iObjectRef = 0;
        currentRelHypo->eObjectClass = Envm_GEN_OBJECT_CLASS_UNCLASSIFIED;
        currentRelHypo->fHypothesisProbability = 0;
        currentRelHypo->fHypothesisLifetime = 0;
    }
}

/* ****************************************************************************
  @fn              CDCorrectSingleTimingValueByLatency                      */ /*!
   @brief           -
   @description     -
   @param[in,out]   timingValue
   @return          fTime_t
   @pre             -
   @post            -
 **************************************************************************** */
static fTime_t CDCorrectSingleTimingValueByLatency(fTime_t timingValue) {
    if (timingValue < (CD_TIME_MAX - C_F32_DELTA)) {
        timingValue = timingValue - CD_LATENCY_SYSTEM;
        timingValue = MAX_FLOAT(timingValue, 0);
    }
    return timingValue;
}

/* ***********************************************************************
  @fn            CDSortHypotheses */
void CDSortHypotheses(const CDInternalStatus_t *pInternalStatus,
                      Hypothesis_array_t *rgSortedHypothesesList,
                      CDInternalMeasurementData_t *pMeasurementData,
                      const CDObjectData_t *pObjectData,
                      const CDInternalObjectList_t *rgObjInternal) {
    uint8 uiSortedIndex[CD_NUMBER_OF_HYPOTHESES];
    uint8 uiOuter;
    uint8 uiInner;
    _PARAM_UNUSED(rgObjInternal);

    /* 1 sort the hypotheses */

    /* 1.1 init sortedIndex */
    for (uiOuter = 0u; uiOuter < CD_NUMBER_OF_HYPOTHESES; uiOuter++) {
        uiSortedIndex[uiOuter] = uiOuter;
    }

    /* 1.2 sort sortedIndex using Relevance as sorting criteria */
    for (uiOuter = 0u;
         (uiOuter + 1u) < (pInternalStatus->uiNofRelevantHypotheses);
         uiOuter++) {
        for (uiInner = (uint8)(uiOuter + 1u);
             uiInner < pInternalStatus->uiNofRelevantHypotheses; uiInner++) {
            if ((*pInternalStatus
                      ->rgIntRelevantHypothesesList)[uiSortedIndex[uiOuter]]
                    .fRelevance <
                (*pInternalStatus
                      ->rgIntRelevantHypothesesList)[uiSortedIndex[uiInner]]
                    .fRelevance) {
                /* swap inner and outer */
                const uint8 uiTmpObjIndex = uiSortedIndex[uiInner];
                uiSortedIndex[uiInner] = uiSortedIndex[uiOuter];
                uiSortedIndex[uiOuter] = uiTmpObjIndex;
            }
        }
    }

    /* 2.1 copy data to output*/
    for (uiOuter = 0u; uiOuter < pInternalStatus->uiNofRelevantHypotheses;
         uiOuter++) {
        const CDIntHypothesis_t *const currentRelHypo =
            &((*pInternalStatus
                    ->rgIntRelevantHypothesesList)[uiSortedIndex[uiOuter]]);
        const ObjNumber_t iCurrentRelObjRef = currentRelHypo->iObjectRef;
        const CDInternalObject_t *const pCurrentInternalObject =
            &(*pInternalStatus->rgObjInternal)[iCurrentRelObjRef];
        Hypothesis_t *const pCurrentHypo = &(*rgSortedHypothesesList)[uiOuter];
        const Envm_t_GenObjKinEnvmatics *const pCurrentEMKinematics =
            CDGetPointer_Kinematic(pObjectData, iCurrentRelObjRef);
        const Envm_t_GenObjQualifiers *const pCurrentEMQualifiers = &(
            (pObjectData)->pGenObjList->aObject[iCurrentRelObjRef].Qualifiers);
        const float32 fCurrentHypoProb =
            currentRelHypo->fHypothesisProbability * 100.0f;

        pCurrentHypo->uiHypothesisProbability =
            (uint8)ROUND_TO_UINT(fCurrentHypoProb);
        pCurrentHypo->uiObjectRef = currentRelHypo->iObjectRef;
        pCurrentHypo->eType = currentRelHypo->eType;
        pCurrentHypo->uiObjectProbability =
            CD_EBA_OBJ_QUALITY(pObjectData, iCurrentRelObjRef);
        switch (currentRelHypo->eObjectClass) {
            case (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_POINT:
            case (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_WIDE:
            case (
                Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_UNCLASSIFIED:
                pCurrentHypo->eEBAObjectClass = EBAObjectClass_Obstacle;
                break;

            case (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_CAR:
            case (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_TRUCK:
                pCurrentHypo->eEBAObjectClass = EBAObjectClass_Vehicle;
                break;
            // wulin add 20220316
            case (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_MOTORCYCLE:
            case (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_BICYCLE:
                pCurrentHypo->eEBAObjectClass = EBAObjectClass_Cyclist;

                break;
            case (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_PEDESTRIAN:
                pCurrentHypo->eEBAObjectClass = EBAObjectClass_Pedestrian;
                break;

            default:
                pCurrentHypo->eEBAObjectClass = EBAObjectClass_NotAvail;
                break;
        }

        switch (CDGetPointer_Attributes(pObjectData, iCurrentRelObjRef)
                    ->eDynamicProperty) {
            case (Envm_t_GenObjDynamicProperty)
                Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY:
            case (Envm_t_GenObjDynamicProperty)
                Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED:
                pCurrentHypo->eEBADynProp = EBADynProp_Stat;
                break;
            case (Envm_t_GenObjDynamicProperty)
                Envm_GEN_OBJECT_DYN_PROPERTY_MOVING:
            case (Envm_t_GenObjDynamicProperty)
                Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN: /* Currently workaround to
                                                         enable Objects with
                                                         not-set dyn-prob*/
            case (Envm_t_GenObjDynamicProperty)
                Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_LEFT:
            case (Envm_t_GenObjDynamicProperty)
                Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_RIGHT:
            case (Envm_t_GenObjDynamicProperty)
                Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING:
                pCurrentHypo->eEBADynProp = EBADynProp_Mov;
                break;
            default:
                pCurrentHypo->eEBADynProp = EBADynProp_NotAvail;
                break;
        }
        pCurrentHypo->eEBAInhibitionMask =
            pCurrentEMQualifiers->eEbaInhibitionMask;

        pCurrentHypo->fHypothesisLifetime = currentRelHypo->fHypothesisLifetime;

        /* adding information to output hypothesis */
        pCurrentHypo->fTTC =
            CDCorrectSingleTimingValueByLatency(pCurrentInternalObject->TTC);

        pCurrentHypo->fTTC2 =
            CDCorrectSingleTimingValueByLatency(pCurrentInternalObject->TTC2);

        pCurrentHypo->fTTC3 =
            CDCorrectSingleTimingValueByLatency(pCurrentInternalObject->TTC3);

        /* Don't add latency to TTC4 since it should only be the time gap
         * between Ego and the object */
        pCurrentHypo->fTTC4 = pCurrentInternalObject->TTC4;
        pCurrentHypo->fTTBPre =
            CDCorrectSingleTimingValueByLatency(pCurrentInternalObject->TTBPre);
        pCurrentHypo->fTTBAcute = CDCorrectSingleTimingValueByLatency(
            pCurrentInternalObject->TTBAcute);
        pCurrentHypo->fTTSPre =
            CDCorrectSingleTimingValueByLatency(pCurrentInternalObject->TTSPre);
        pCurrentHypo->fTTSAcute = CDCorrectSingleTimingValueByLatency(
            pCurrentInternalObject->TTSAcute);
        pCurrentHypo->fLongNecAccel = pCurrentInternalObject->LongNecAccel;

        pCurrentHypo->fLatNecAccel = pCurrentInternalObject->LatNecAccel;
        pCurrentHypo->fDistX = CD_GET_DIST_X(pObjectData, iCurrentRelObjRef);

        pCurrentHypo->fDistX = pCurrentHypo->fDistX + CD_LONG_OFFSET_SIMU;

        pCurrentHypo->fDistXStd =
            CD_OBJ_DISTX_STDDEV(pObjectData, iCurrentRelObjRef);
        pCurrentHypo->fVrelX = pCurrentEMKinematics->fVrelX;
        pCurrentHypo->fVrelXStd = pCurrentEMKinematics->fVrelXStd;
        pCurrentHypo->fDistY = CD_GET_DIST_Y(pObjectData, iCurrentRelObjRef);

        pCurrentHypo->fDistY = pCurrentHypo->fDistY + CD_LAT_OFFSET_SIMU;

        pCurrentHypo->fDistYStd =
            CD_OBJ_DISTY_STDDEV(pObjectData, iCurrentRelObjRef);
        pCurrentHypo->fArelX = pCurrentEMKinematics->fArelX;
        pCurrentHypo->fArelXStd = pCurrentEMKinematics->fArelXStd;
        pCurrentHypo->fVrelY = pCurrentEMKinematics->fVrelY;
        pCurrentHypo->fVrelYStd = pCurrentEMKinematics->fVrelYStd;
        pCurrentHypo->fArelY = pCurrentEMKinematics->fArelY;
        pCurrentHypo->fArelYStd = pCurrentEMKinematics->fArelYStd;
        pCurrentHypo->fClosingVelocity =
            pCurrentInternalObject->ClosingVelocity;
        pCurrentHypo->fClosingVelocityStd =
            0; /*!< @todo: Fix this with real standard deviation! */

        /*  2.2 pass them to MTS structure and sorted Index*/
        if (pMeasurementData != NULL) /*write data to measurement structure*/
        {
            CDHypothesisMeasurement_t *const currentHypoMeasurement =
                &pMeasurementData->rgHypotheses[uiOuter];
            currentHypoMeasurement->TrackAssigned =
                pCurrentInternalObject->TrackAssigned;
        }
    }

    /*clear the rest of the hypothesis*/
    for (uiOuter = pInternalStatus->uiNofRelevantHypotheses;
         uiOuter < CD_NUMBER_OF_HYPOTHESES; uiOuter++) {
        Hypothesis_t *const currentSortedHypo =
            &(*rgSortedHypothesesList)[uiOuter];
        currentSortedHypo->uiObjectRef = 0;
        currentSortedHypo->eType = CDHypothesisType_No;
        currentSortedHypo->uiObjectProbability = 0u;
        currentSortedHypo->eEBAObjectClass = EBAObjectClass_NotAvail;
        currentSortedHypo->eEBADynProp = EBADynProp_NotAvail;
        currentSortedHypo->eEBAInhibitionMask = FPS_EBA_INH_ALL;

        currentSortedHypo->uiHypothesisProbability = 0u;
        currentSortedHypo->fHypothesisLifetime = 0;

        currentSortedHypo->fTTC = CD_TIME_MAX;
        currentSortedHypo->fTTCStd = CD_TIME_MAX;
        currentSortedHypo->fTTC2 = CD_TIME_MAX;
        currentSortedHypo->fTTC3 = CD_TIME_MAX;
        currentSortedHypo->fTTC4 = CD_TIME_MAX;

        currentSortedHypo->fTTBPre = CD_TIME_MAX;
        currentSortedHypo->fTTBPreStd = CD_TIME_MAX;
        currentSortedHypo->fTTBAcute = CD_TIME_MAX;
        currentSortedHypo->fTTBAcuteStd = CD_TIME_MAX;
        currentSortedHypo->fTTSPre = CD_TIME_MAX;
        currentSortedHypo->fTTSPreStd = CD_TIME_MAX;
        currentSortedHypo->fTTSAcute = CD_TIME_MAX;
        currentSortedHypo->fTTSAcuteStd = CD_TIME_MAX;
        currentSortedHypo->fLongNecAccel = 0;
        currentSortedHypo->fLatNecAccel = 0;
        currentSortedHypo->fDistX = CD_DIST_MAX;
        currentSortedHypo->fDistXStd = CD_DIST_MAX;
        currentSortedHypo->fVrelX = 0;
        currentSortedHypo->fVrelXStd = 0;
        currentSortedHypo->fArelX = 0;
        currentSortedHypo->fArelXStd = 0;
        currentSortedHypo->fDistY = 0;
        currentSortedHypo->fDistYStd = 0;
        currentSortedHypo->fVrelY = 0;
        currentSortedHypo->fVrelYStd = 0;
        currentSortedHypo->fArelY = 0;
        currentSortedHypo->fArelYStd = 0;
        currentSortedHypo->fClosingVelocity = 0;
        currentSortedHypo->fClosingVelocityStd = 0;

        if (pMeasurementData != NULL) /*write data to measurement structure*/
        {
            CDHypothesisMeasurement_t *const currentHypoMeasurement =
                &pMeasurementData->rgHypotheses[uiOuter];
            currentHypoMeasurement->TrackAssigned = (uint8)0;
        }
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                     */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */