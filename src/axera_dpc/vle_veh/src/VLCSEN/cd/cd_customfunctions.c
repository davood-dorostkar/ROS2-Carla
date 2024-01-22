
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
/*! @brief       ONE_HALF
    @general     factor 0.5
    @attention   [None]
    @typical     0.5
    @unit        [None]

       */
#define ONE_HALF (0.5f)

/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/

static fDistance_t CDCalcMaxDistHRZ(fDistance_t fMaxLatErrDist,
                                    fDistance_t fMaxDist);
static fDistance_t CDCalcMaxDistVIS(fDistance_t fMaxDist);
static boolean CDCheckObjectFilter(const CDInternalStatus_t *pInternalStatus,
                                   const CDHypoHandler_t *const pCurHypHandler,
                                   const ObjNumber_t iObjectIndex,
                                   const CDInputData_t *pInputData);

/* ***********************************************************************
  @fn            CDInitCustomerData                           */ /*!

            @brief         Init of customer specific data.

            @description   Init of customer specific data.

            @return        void

            @pre           [none]

            @post          [none]
          ****************************************************************************
          */
void CDInitCustomerData(void) {}

/* ***********************************************************************
  @fn            CDCheckObjectFilter                                */ /*!

      @brief         returns whether an object is relevant to create hyp

      @param[in]     pInternalStatus
      @param[in]     pCurHypHandler
      @param[in]     iObjectIndex
      @param[in]     pInputData

      @return        boolean

      @pre           [none]

      @post          [none]

    ****************************************************************************
    */
static boolean CDCheckObjectFilter(const CDInternalStatus_t *pInternalStatus,
                                   const CDHypoHandler_t *const pCurHypHandler,
                                   const ObjNumber_t iObjectIndex,
                                   const CDInputData_t *pInputData) {
    boolean bObjFilterMatched = FALSE;

    /* check relevant qualities and class conf. > min. required class conf. */
    if ((CD_EBA_OBJ_QUALITY(pInputData->pObjectData, iObjectIndex) >=
         pCurHypHandler->uiGenObjQuality) &&
        (!(OBJ_IS_DELETED(iObjectIndex)))
        /* no more need for separation...only EBA quality exists... */
        && (CDGetPointer_Attributes(pInputData->pObjectData, iObjectIndex)
                ->uiClassConfidence >= pCurHypHandler->uiMinClassConf)) {
        uint8 uiPrevHypIndex;
        boolean bObjInPrevHypList = FALSE;
        uint8 uiClassFilterCnt;
        uint8 uiClassFilterIdx;
        Envm_t_GenObjClassification eClass;
        bObjFilterMatched = TRUE;

        /* If iObjectIndex matches one iObjectRef in rgPreviousHypothesesList
        for the current
        hypothesis the hypothesis must be evaluated without check of
        rgClassFilter. */
        for (uiPrevHypIndex = 0;
             (bObjInPrevHypList == FALSE) &&
             (uiPrevHypIndex < pInternalStatus->uiNofPreviousHypotheses) &&
             (uiPrevHypIndex < CD_NUMBER_OF_HYPOTHESES);
             uiPrevHypIndex++) {
            const CDIntHypothesis_t *const currentPrevHypo =
                &((*pInternalStatus->rgPreviousHypothesesList)[uiPrevHypIndex]);
            if (((((uint32)(pCurHypHandler->uiHypTypes)) &
                  (1u << (uint32)currentPrevHypo->eType)) > 0) &&
                (iObjectIndex == currentPrevHypo->iObjectRef)) {
                bObjInPrevHypList = TRUE;
            }
        }

        /* check classification filter only if object has not already been in
        previous hypothesis
        list and the class filter for the current hypothesis is valid */
        if (bObjInPrevHypList == FALSE) {
            /* check hypothesis category from FPS */
            if ((((uint32)pCurHypHandler->bitHypCat) &
                 ((uint32)pInputData->pObjectData->pGenObjList
                      ->aObject[iObjectIndex]
                      .Qualifiers.eEbaHypCat)) == 0) {
                bObjFilterMatched = FALSE;
            }

            /* maybe check the classification */
            uiClassFilterCnt = pCurHypHandler->uiClassFilterCnt;

            /* check classification filter if the class confidence is high
            enough and the class
            filter for the current hypothesis is valid */
            if ((bObjFilterMatched == TRUE) &&
                (pCurHypHandler->rgClassFilter != NULL) &&
                (uiClassFilterCnt > 0u)) {
                /* for positive filter, filter match should be FALSE until
                object class is found,
                for negative filter, filter match should be TRUE until object
                class is found */
                bObjFilterMatched = pCurHypHandler->bNegativeFilter;

                for (uiClassFilterIdx = 0u;
                     (uiClassFilterIdx < uiClassFilterCnt) &&
                     (bObjFilterMatched == pCurHypHandler->bNegativeFilter);
                     uiClassFilterIdx++) {
                    eClass = pCurHypHandler->rgClassFilter[uiClassFilterIdx]
                                 ->eClassification; /*array with known length*/

                    /* check if class is in filter */
                    if (CDGetPointer_Attributes(pInputData->pObjectData,
                                                iObjectIndex)
                            ->eClassification == eClass) {
                        /* for positive filter, filter matches when object is
                         * found */
                        if (pCurHypHandler->bNegativeFilter == FALSE) {
                            bObjFilterMatched = TRUE;
                        }
                        /* for negative filter, filter does not match when
                           object is found */
                        else {
                            bObjFilterMatched = FALSE;
                        }
                    }
                }
            }
        }
    }

    return bObjFilterMatched;
}

/* ***********************************************************************
  @fn            CDCustomerHypothesisHandler                           */ /*!

   @brief         Configurable main loop for hypothesis handling

   @param[in]     pInputData Pointer to the CD input data.
   @param[in]     pInternalStatus Pointer to the internal CD status.
   @param[in]     pExternalFunctions Pointer to the external functions.

   @return        void

   @pre           [none]

   @post          [none]


   @attention     The hypothesis handlers must be executed in specfic order.
                  Otherwise the hypothesis selection will fail.
                  Each hypothesis handler must be called for all relevant object
                  before the next handler is considerred. Also ther is an order
                  between the handlers which is determined by the selection
                  algorithm.
 **************************************************************************** */
void CDCustomerHypothesisHandler(
    const CDInputData_t *pInputData,
    CDInternalStatus_t *pInternalStatus,
    const CDExternalFunctions_t *pExternalFunctions) {
    uint8 uiHypHandlerIdx;
    ObjNumber_t iObjectIndex;
    boolean bObjFilterMatched;

    if (GET_EGO_RAW_DATA_PTR->MotionState.MotState !=
        VED_LONG_MOT_STATE_MOVE_RWD) {
        for (uiHypHandlerIdx = 0u; uiHypHandlerIdx < CD_HYP_HANDLERS_NO;
             uiHypHandlerIdx++) {
            const CDHypoHandler_t *const pCurHypHandler =
                CD_HYP_HANDLERS[uiHypHandlerIdx];
            if (pCurHypHandler != NULL) {
                if (pCurHypHandler->fpHypHandler != NULL) {
                    VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart,
                                             pCurHypHandler->uRTACheckpoint, 0);
                    /* for all objects try to calculate hypothesis */
                    for (iObjectIndex = 0;
                         iObjectIndex <
                         pInputData->pObjectData->iNumberOfObjects;
                         iObjectIndex++) {
                        bObjFilterMatched =
                            CDCheckObjectFilter(pInternalStatus, pCurHypHandler,
                                                iObjectIndex, pInputData);
                        /* call the hypothesis handler */
                        pCurHypHandler->fpHypHandler(
                            iObjectIndex, bObjFilterMatched, pInputData,
                            pInternalStatus, pExternalFunctions);
                    }
                    VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                             pCurHypHandler->uRTACheckpoint, 0);
                }
            }
        }
    }
}

/* ***********************************************************************
  @fn              CDCalcCustomerFunctions                           */ /*!

     @brief           Calculates Customer Specific Functions

     @description     Calculates Customer Specific Functions

     @param[in]       pInternalStatus Pointer to the internal CD status.
     @param[in]       pObjectData Pointer to the CD internal object data.
     @param[in]       pExternalFunctions Pointer to the external functions.

     @return          void

     @pre             [none]

     @post            [none]

   ****************************************************************************
   */
void CDCalcCustomerFunctions(
    CDInternalStatus_t
        *pInternalStatus, /*not modified, but could be in some projects*/
    const CDObjectData_t *pObjectData,
    const CDExternalFunctions_t *pExternalFunctions) {
    _PARAM_UNUSED(pObjectData);
    _PARAM_UNUSED(pInternalStatus);
    _PARAM_UNUSED(pExternalFunctions);
}

/* ***********************************************************************
  @fn              CDGetCustomerParameters                           */
void CDGetCustomerParameters(
    CDInternalStatus_t
        *pInternalStatus) /*not modified, but could be in some projects*/
{
    _PARAM_UNUSED(pInternalStatus);
}

/* ***********************************************************************
  @fn            CDCalcCustomerPerfDegradation                           */ /*!

 @brief         Calculate the performance degradation information

 @description   The performance degradation information shall be used to degrade
 the function.

 @param[in]     pInputData Pointer to the input data.
 @param[in]     pOutputData Pointer to the output data.
 @param[in]     pExternalFunctions Pointer to the external functions.

 @return        void

 @pre           [none]

 @post          Sets the customer specific performance degradation information.
*/
void CDCalcCustomerPerfDegradation(
    const CDInputData_t *pInputData,
    CDOutputData_t *pOutputData,
    const CDExternalFunctions_t *pExternalFunctions) {
    _PARAM_UNUSED(pExternalFunctions);
    _PARAM_UNUSED(pInputData);

    pOutputData->pDegradation->Safety.fMaxDist = CD_PERF_DEG_MAX_DIST;
    pOutputData->pDegradation->Safety.fMaxDistALN = CD_PERF_DEG_MAX_DIST;
    pOutputData->pDegradation->Safety.fMaxDistVIS = 0.0F;
    pOutputData->pDegradation->Safety.fMaxDistHRZ = CD_PERF_DEG_MAX_DIST;

    pOutputData->pDegradation->Performance.fMaxDist = CD_PERF_DEG_MAX_DIST;
    pOutputData->pDegradation->Performance.fMaxDistALN = CD_PERF_DEG_MAX_DIST;
    pOutputData->pDegradation->Performance.fMaxDistVIS =
        CD_PERF_DEG_MAX_DIST;  // Temporary
    pOutputData->pDegradation->Performance.fMaxDistHRZ = CD_PERF_DEG_MAX_DIST;
}

/* ***********************************************************************
  @fn            CDCalcMaxDistVIS                           */
static fDistance_t CDCalcMaxDistVIS(fDistance_t fMaxDist) {
    fDistance_t fMaxDistVIS = fMaxDist;
    /* done */
    return fMaxDistVIS;
}

/* ************************************************************************* */
/*   Copyright Tuerme                     */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */