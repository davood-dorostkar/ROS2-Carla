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

static boolean CDHypoStaticObjectFilter(ObjNumber_t iObjectIndex,
                                        const CDObjectData_t *pObjectData,
                                        const CDInternalObject_t *pLocalObj);

static void CDHypoStaticCalculateProb(ObjNumber_t iObjectIndex,
                                      CDIntHypothesis_t *pHypothesis);

/* ***********************************************************************
  @fn            CDHypoStaticObjectFilter */
static boolean CDHypoStaticObjectFilter(ObjNumber_t iObjectIndex,
                                        const CDObjectData_t *pObjectData,
                                        const CDInternalObject_t *pLocalObj) {
    boolean bReturn;
    const Envm_t_GenObjKinEnvmatics *const pObjKinematic =
        CDGetPointer_Kinematic(pObjectData, iObjectIndex);
    const float32 fObjDistX = CD_GET_DIST_X(pObjectData, iObjectIndex);
    const Envm_t_GenObjAttributes *const pObjAttribs =
        CDGetPointer_Attributes(pObjectData, iObjectIndex);
    /* Allow following hypothesis if
     * a) object has min. required quality
     * b) object is moving
     * c) vrel is in following range (CD_FOLLOWING_MIN_VREL,
     * CD_FOLLOWING_MAX_VREL)
     * d) object is close enough in long. direction
     * e) object was seen at least one cycle in track
     * f) previous following hypothesis is kept until vrel is greater than
     * treshold
     */
    if ((pObjAttribs->eDynamicProperty ==
         Envm_GEN_OBJECT_DYN_PROPERTY_MOVING) &&
        (fObjDistX < CD_FOLLOWING_MAX_DIST) &&
        (pObjKinematic->fVrelX > CD_FOLLOWING_MIN_VREL) &&
        ((pObjKinematic->fVrelX < CD_FOLLOWING_MAX_VREL) ||
         ((pObjKinematic->fVrelX < CD_FOLLOWING_MAX_VREL_KEEP) &&
          (pLocalObj->HypothesisHist.Following == 1))) &&
        ((pLocalObj->TrackAssigned & (uint8)CD_FOLLOWING_MIN_TRACK_ASSIGNED) !=
         0u)) {
        bReturn = TRUE;
    } else {
        bReturn = FALSE;
    }
    return bReturn;
}

/* ***********************************************************************
  @fn            CDHypoStaticCalculateProb */
static void CDHypoStaticCalculateProb(ObjNumber_t iObjectIndex,
                                      CDIntHypothesis_t *pHypothesis) {
    _PARAM_UNUSED(iObjectIndex);
    pHypothesis->fHypothesisProbability = 1.0f;
}

/* ***********************************************************************
  @fn            CDHypoStaticMain */ /*!

                                        @brief         handles the CGEB
                                      situation analysis hypothesis Follow

                                        @description   Handles the Follow
                                      hypothesis

                                        @param[in]     iObjectIndex The index of
                                      the object
                                        @param[in]     bObjFilterMatched If TRUE
                                      out object filter matched so hypothesis
                                      shall be calculated. If FALSE reset
                                      history (if exists)
                                        @param[in]     pInputData Pointer to CD
                                      input data
                                        @param[in]     pInternalStatus Pointer
                                      to CD internal status
                                        @param[in]     pExternalFunctions
                                      Pointer to the external functions

                                        @return        void

                                        @pre           [none]

                                        @post          [none]

                                      ****************************************************************************
                                      */
void CDHypoStaticMain(ObjNumber_t iObjectIndex,
                      boolean bObjFilterMatched,
                      const CDInputData_t *pInputData,
                      CDInternalStatus_t *pInternalStatus,
                      const CDExternalFunctions_t *pExternalFunctions) {
    CDIntHypothesis_t Hypothesis;
    CDInternalObject_t *const pLocalObject =
        &(*pInternalStatus->rgObjInternal)[iObjectIndex];

    _PARAM_UNUSED(pExternalFunctions);
    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypPresel),
                     (uint32)CDHypothesisType_Static);
    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypActive),
                     (uint32)CDHypothesisType_Static);

    /* handle only hypothesis relevant objects */
    if (bObjFilterMatched != FALSE) {
        VLCSEN_SERVICE_ADD_EVENT(
            e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_HYP_STATIC_SINGLE,
            (uint8)(iObjectIndex)); /* start profiling for Hypothesis */
        if (CDHypoStaticObjectFilter(iObjectIndex, pInputData->pObjectData,
                                     pLocalObject) != FALSE) {
            CD_SET_HYP_BIT(&(pLocalObject->bitHypPresel),
                           (uint32)CDHypothesisType_Static);

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
            Hypothesis.eType = CDHypothesisType_Static;

            /* calculate Hypothesis probability */
            CDHypoStaticCalculateProb(iObjectIndex, &Hypothesis);

            /* store hypothesis (if relevant) */
            if (Hypothesis.fHypothesisProbability > CD_COMMON_MIN_HYP_PROB) {
                CD_SET_HYP_BIT(&(pLocalObject->bitHypActive),
                               (uint32)CDHypothesisType_Static);
                CDHypothesesSelection(&Hypothesis, pInputData->pObjectData,
                                      pInternalStatus);
                pLocalObject->HypothesisHist.Following = 1u;
            } else {
                pLocalObject->HypothesisHist.Following = 0u;
            }
        } else {
            pLocalObject->HypothesisHist.Following = 0u;
        }
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                 VLCSEN_RTA_CD_HYP_STATIC_SINGLE,
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