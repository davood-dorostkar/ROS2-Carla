/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cd.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! @brief       Local identifier for right lane

    @conseq      Not Applicable

    @attention   Not Applicable
*/
#define CD_RUN_UP_STAT_RIGHT_LANE (1u)

/*! @brief       Local identifier for left lane

    @conseq      Not Applicable

    @attention   Not Applicable
*/
#define CD_RUN_UP_STAT_LEFT_LANE (2u)

/*! @brief       Number of sampling points for run up hypothesis probability

    @conseq      Not Applicable

    @attention   Not Applicable
*/
#define CD_RUN_UP_STAT_HYP_PROB_NO_POINTS (9u)

/*****************************************************************************
  FUNCTION
*****************************************************************************/
static boolean CDHypoRunUpStationaryFilter(
    ObjNumber_t iObjectIndex,
    const CDObjectData_t *pObjectData,
    const CDInternalObject_t *pLocalObject);
static void CDCalculateProbRunUpStationaryHypo(
    ObjNumber_t iObjectIndex,
    CDIntHypothesis_t *pHypothesis,
    const CDObjectData_t *pObjectData,
    CDInternalObject_t *pLocalObject);

/* ***********************************************************************
  @fn            CDHypoRunUpStationaryFilter */
static boolean CDHypoRunUpStationaryFilter(
    ObjNumber_t iObjectIndex,
    const CDObjectData_t *pObjectData,
    const CDInternalObject_t *pLocalObject) {
    boolean bReturn;
    const float32 fObjDistX = CD_GET_DIST_X(pObjectData, iObjectIndex);
    const boolean isStationaryOrWasOncoming = (boolean)(
        (CDGetPointer_Attributes(pObjectData, iObjectIndex)->eDynamicProperty ==
         (Envm_t_GenObjDynamicProperty)
             Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY) ||
        ((CDGetPointer_Attributes(pObjectData, iObjectIndex)
              ->eDynamicProperty ==
          (Envm_t_GenObjDynamicProperty)Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED)
         /*&& ( pLocalObject->HypothesisHist.WasOncomming == TRUE )*/)  // stopped
                                                                        // object
                                                                        // should
                                                                        // trigger
                                                                        // run
                                                                        // up
                                                                        // stationary
                                                                        // hypothesis
                                                                        // guotao
                                                                        // 20200922
        || (pLocalObject->HypothesisHist.RunUpStationary == TRUE));
    const boolean isStoppedAndWasCrossing = (boolean)((
        (CDGetPointer_Attributes(pObjectData, iObjectIndex)->eDynamicProperty ==
         (Envm_t_GenObjDynamicProperty)Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED) &&
        (pLocalObject->HypothesisHist.WasCrossing == TRUE)));

    if ((isStationaryOrWasOncoming != FALSE) &&
        (isStoppedAndWasCrossing != TRUE)  // If the Object was Crossing and is
                                           // now Stopped, it is NOT relevant
                                           // for the RunUpStationary Hypothesis
        && ((pLocalObject->TTSPreLeft < CD_RUN_UP_STAT_MAX_TTS) ||
            (pLocalObject->TTSPreRight < CD_RUN_UP_STAT_MAX_TTS) ||
            (((pLocalObject->LongNecAccel < 0) ||
              (pLocalObject->TTC4 < CD_COMMON_TTC_THRES)) &&
             (pLocalObject->HypothesisHist.RunUpStationary == 1))) &&
        (pLocalObject->TrackAssigned >= (uint8)CD_RUN_UP_MIN_TRACK_ASSIGNED) &&
        ((fObjDistX > CD_COMMON_MIN_DISTX) ||
         (pLocalObject->HypothesisHist.RunUpMoving == 1) ||
         (pLocalObject->HypothesisHist.RunUpStationary == 1) ||
         (pLocalObject->HypothesisHist.CutIn == 1))) {
        bReturn = TRUE;
    } else {
        bReturn = FALSE;
    }
    return bReturn;
}

/* ***********************************************************************
  @fn            CDCalculateProbRunUpStationaryHypo */
static void CDCalculateProbRunUpStationaryHypo(
    ObjNumber_t iObjectIndex,
    CDIntHypothesis_t *pHypothesis,
    const CDObjectData_t *pObjectData,
    CDInternalObject_t *pLocalObject) {
    uint8 uiTrackMask;
    uint8 uiTrackHist;
    static const float32
        fCDRunUpStatHypProb[CD_RUN_UP_STAT_HYP_PROB_NO_POINTS] = {
            0, 0.25f, 0.5f, 0.57f, 0.6f, 0.61f, 0.62f, 0.63f, 1.0f};

    _PARAM_UNUSED(pObjectData);
    _PARAM_UNUSED(iObjectIndex);

    /* search first zero bit */
    uiTrackMask = 128u;
    uiTrackHist = 0u;
    /*while loop will terminate after 8th shift*/
    while ((pLocalObject->TrackAssigned & uiTrackMask) != 0u) {
        uiTrackMask = uiTrackMask >> 1;
        uiTrackHist++;
    }
    /* uiTrackHist is the (zero based) number of the first 0 bit; if no bit is
     * zero uiTrackHist is 8 */

    pHypothesis->fHypothesisProbability = fCDRunUpStatHypProb[uiTrackHist];

    /* done */
    return;
}

/* ***********************************************************************
  @fn            CDHypoRunUpStationaryMain */ /*!

                               @brief         handles the CGEB situation
                             analysis
                                              hypothesis run up stationary

                               @description   Handles the run up stationary
                             hypothesis
                                              (testing the hypothesis for each
                             object)

                               @param[in]     iObjectIndex The index of the
                             object
                               @param[in]     bObjFilterMatched If TRUE out
                             object filter matched so hypothesis shall be
                             calculated. If FALSE reset history (if exists)
                               @param[in]     pInputData Pointer to CD input
                             data
                               @param[in]     pInternalStatus Pointer to CD
                             internal status
                               @param[in]     pExternalFunctions

                               @return        void

                               @pre           [none]

                               @post          [none]

                             ****************************************************************************
                             */
void CDHypoRunUpStationaryMain(
    ObjNumber_t iObjectIndex,
    boolean bObjFilterMatched,
    const CDInputData_t *pInputData,
    CDInternalStatus_t *pInternalStatus,
    const CDExternalFunctions_t *pExternalFunctions) {
    CDIntHypothesis_t Hypothesis;
    CDInternalObject_t *const pLocalObject =
        &(*pInternalStatus->rgObjInternal)[iObjectIndex];

    _PARAM_UNUSED(pExternalFunctions);
    pLocalObject->sHypRunUpStatData.bObstacleAtLeft = FALSE;
    pLocalObject->sHypRunUpStatData.bObstacleAtRight = FALSE;

    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypPresel),
                     (uint32)CDHypothesisType_RunUpStationary);
    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypActive),
                     (uint32)CDHypothesisType_RunUpStationary);

    /* handle only hypothesis relevant objects */
    if (bObjFilterMatched != FALSE) {
        VLCSEN_SERVICE_ADD_EVENT(
            e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_HYP_RUNUP_STAT_SINGLE,
            (uint8)(iObjectIndex)); /* start profiling for Hypothesis */
        if (CDHypoRunUpStationaryFilter(iObjectIndex, pInputData->pObjectData,
                                        pLocalObject) != FALSE) {
            CD_SET_HYP_BIT(&(pLocalObject->bitHypPresel),
                           (uint32)CDHypothesisType_RunUpStationary);

            Hypothesis.fRelevance = 0;
            Hypothesis.fHypothesisProbability = 0;
            Hypothesis.fHypothesisLifetime = 0;

            /* link object to hypothesis */
            Hypothesis.iObjectRef = iObjectIndex;

            /* store object class */
            Hypothesis.eObjectClass =
                CDGetPointer_Attributes(pInputData->pObjectData, iObjectIndex)
                    ->eClassification;

            /* set hypothesis type */
            Hypothesis.eType = CDHypothesisType_RunUpStationary;

            /* calculate Hypothesis probability */
            CDCalculateProbRunUpStationaryHypo(iObjectIndex, &Hypothesis,
                                               pInputData->pObjectData,
                                               pLocalObject);

            /* store hypothesis (if relevant) */
            if (Hypothesis.fHypothesisProbability > 0) {
                CD_SET_HYP_BIT(&(pLocalObject->bitHypActive),
                               (uint32)CDHypothesisType_RunUpStationary);
                CDHypothesesSelection(&Hypothesis, pInputData->pObjectData,
                                      pInternalStatus);
                pLocalObject->HypothesisHist.RunUpStationary = 1u;
            } else {
                pLocalObject->HypothesisHist.RunUpStationary = 0u;
            }
        } else {
            pLocalObject->HypothesisHist.RunUpStationary = 0u;
        }
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                 VLCSEN_RTA_CD_HYP_RUNUP_STAT_SINGLE,
                                 (uint8)(iObjectIndex));
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                     */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */