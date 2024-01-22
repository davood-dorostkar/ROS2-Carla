/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cd.h"

/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/

static eGDBError_t CDCheckPointers(
    const CDInputData_t *pInputData,
    const CDInternalStatus_t *pInternalStatus,
    const CDOutputData_t *pOutputData,
    const CDParameters_t *pParameters,
    const CDExternalFunctions_t *pExternalFunctions);
static void CDPrepareCycle(CDInternalStatus_t *pInternalStatus,
                           const CDInputData_t *pInputData);
static void CDUpdateHypothesisHistory(
    const CDInternalStatus_t *pInternalStatus);

/* ***********************************************************************
  @fn            CDPrepareCycle */ /*!


                                          @brief         Initializes CD  at the
                                        beginning of each loop

                                          @description   The function
                                        initializes values from the CD component

                                          @param[in]     pInternalStatus Pointer
                                        to the internal CD status
                                          @param[in]     pInputData Pointer to
                                        CD input data

                                          @return        void

                                          @pre           [none]

                                          @post          [none]

                                        ****************************************************************************
                                        */
static void CDPrepareCycle(CDInternalStatus_t *pInternalStatus,
                           const CDInputData_t *pInputData) {
    _PARAM_UNUSED(pInputData);
    CDPrepareCycleHypothesesSelection(pInternalStatus);

    // VLC_fEgoVehicleWidth = GET_EGO_STATIC_DATA_PTR->VehParAdd.VehicleWidth;
    // VLC_fEgoVehicleLength = GET_EGO_STATIC_DATA_PTR->VehParAdd.VehicleLength;
    // change input from parameter
    VLC_fEgoVehicleWidth = GET_VLCSEN_PARAMETERS->VLCSEN_Kf_VehWidth;
    VLC_fEgoVehicleLength = GET_VLCSEN_PARAMETERS->VLCSEN_Kf_VehLength;

    /*set default values in case of implausible input data*/
    if ((VLC_fEgoVehicleWidth < CD_PLAUS_EGO_WIDTH) ||
        (VLC_fEgoVehicleLength < CD_PLAUS_EGO_LENGTH)) {
        VLC_fEgoVehicleWidth = CD_COMMON_EGO_WIDTH;
        VLC_fEgoVehicleLength = CD_COMMON_EGO_LENGTH;
    }
}

/* ***********************************************************************
  @fn            CDCheckPointers */ /*!

                                         @brief         The function checks the
                                       pointers and returns an error code if
                                       there are NULL pointers

                                         @description   The function checks the
                                       pointers and returns an error code if
                                       there are NULL pointers

                                         @param[in]     pInputData Pointer to CD
                                       input data
                                         @param[in]     pInternalStatus Pointer
                                       to CD internal data
                                         @param[in]     pOutputData Pointer to
                                       CD output data
                                         @param[in]     pParameters Pointer to
                                       CD parameters
                                         @param[in]     pExternalFunctions
                                       Pointer to CD external functions

                                         @return        GDB_ERROR_NONE is
                                       returned when pointers are okay,
                                       otherwise an error code is returned.

                                         @pre           [none]

                                         @post          [none]

                                       ****************************************************************************
                                       */
static eGDBError_t CDCheckPointers(
    const CDInputData_t *pInputData,
    const CDInternalStatus_t *pInternalStatus,
    const CDOutputData_t *pOutputData,
    const CDParameters_t *pParameters,
    const CDExternalFunctions_t *pExternalFunctions) {
    eGDBError_t local_error;
    local_error = GDB_ERROR_NONE;

    /*check pExternalFunctions*/
    CD_CHECK_NULL(&local_error, (const void *)pExternalFunctions);
    /*check pParameters*/
    CD_CHECK_NULL(&local_error, (const void *)pParameters);
    CD_CHECK_NULL(&local_error, (const void *)pParameters->pAdjSafeDistance);
    /*check output data*/
    CD_CHECK_NULL(&local_error, (const void *)pOutputData);
    CD_CHECK_NULL(&local_error,
                  (const void *)pOutputData->rgRelevantHypothesesList);
    /*check pInternalStatus*/
    CD_CHECK_NULL(&local_error, (const void *)pInternalStatus);
    CD_CHECK_NULL(&local_error, (const void *)pInternalStatus->rgObjInternal);
    CD_CHECK_NULL(&local_error,
                  (const void *)pInternalStatus->rgIntRelevantHypothesesList);
    CD_CHECK_NULL(&local_error,
                  (const void *)pInternalStatus->rgPreviousHypothesesList);
    /*check input data*/
    CD_CHECK_NULL(&local_error, (const void *)pInputData);
    CD_CHECK_NULL(&local_error, (const void *)pInputData->pEgoData);
    CD_CHECK_NULL(&local_error,
                  (const void *)pInputData->pEgoData->pEgoDynObjSync);
    CD_CHECK_NULL(&local_error, (const void *)pInputData->pEgoData->pEgoDynRaw);
    CD_CHECK_NULL(&local_error, (const void *)pInputData->pObjectData);
    CD_CHECK_NULL(&local_error,
                  (const void *)pInputData->pObjectData->pGenObjList);
    CD_CHECK_NULL(&local_error,
                  (const void *)pInputData->pObjectData->pARSObjList);
    CD_CHECK_NULL(&local_error, (const void *)pOutputData->pDegradation);

    return local_error;
}

/* ***********************************************************************
  @fn            CDRun */ /*!

                                                   @brief         The function
                                                 handles the ContiGuard
                                                 situation analysis

                                                   @description   The function
                                                 is called by the global main
                                                 routine and calls
                                                                  all necessary
                                                 functions of the CD component.

                                                   @param[in]     pInputData
                                                 Pointer to CD input data
                                                   @param[in]
                                                 pInternalStatus Pointer to CD
                                                 internal data
                                                   @param[in]     pOutputData
                                                 Pointer to CD output data
                                                   @param[in]     pParameters
                                                 Pointer to CD parameters
                                                   @param[in]
                                                 pExternalFunctions Pointer to
                                                 CD external functions
                                                   @param[in] pMeasurementData

                                                   @return        GDB_ERROR_NONE
                                                 is returned when pointers are
                                                 okay, otherwise an error code
                                                 is returned.

                                                   @pre           [none]

                                                   @post          [none]

                                                 ****************************************************************************
                                                 */
eGDBError_t CDRun(const CDInputData_t *pInputData,
                  CDInternalStatus_t *pInternalStatus,
                  CDOutputData_t *pOutputData,
                  const CDParameters_t *pParameters,
                  const CDExternalFunctions_t *pExternalFunctions,
                  CDInternalMeasurementData_t *pMeasurementData) {
    eGDBError_t local_error;
    local_error = CDCheckPointers(pInputData, pInternalStatus, pOutputData,
                                  pParameters, pExternalFunctions);

    if (local_error == GDB_ERROR_NONE) {
        /* initialize the CD situation analysis variables each loop */
        CDPrepareCycle(pInternalStatus, pInputData);

        /* Get Customer Specific Parameters */
        CDGetCustomerParameters(pInternalStatus);

        CDCalcCustomerPerfDegradation(pInputData, pOutputData,
                                      pExternalFunctions);

        /*----------------------------------------  Update Object History
         * ----------------------------------------*/
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_UPDATE_EMP,
                                 0); /*start profiling for UpdateEMPData*/
        CDUpdateEMPData(pInputData, pInternalStatus);
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd, VLCSEN_RTA_CD_UPDATE_EMP,
                                 0);

        /*----------------------------------------  calculate the CGEB relevant
         * object properties  ----------------------------------------*/
        VLCSEN_SERVICE_ADD_EVENT(
            e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_CALCULATE_OBJECT_PROP,
            0); /* start profiling for CDCalculateObjectProperties */
        CDCalculateObjectProperties(pInputData, pInternalStatus, pParameters);
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                 VLCSEN_RTA_CD_CALCULATE_OBJECT_PROP, 0);

        /*----------------------------------------  tests hypothesis types for
         * all relevant objects  ----------------------------------------*/
        VLCSEN_SERVICE_ADD_EVENT(
            e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_HYPO_HANDLER,
            0); /* start profiling for CDCustomerHypothesisHandler */

        CDCustomerHypothesisHandler(pInputData, pInternalStatus,
                                    pExternalFunctions);
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd, VLCSEN_RTA_CD_HYPO_HANDLER,
                                 0);

        /*----------------------------------------  update hypothesis history
         * ----------------------------------------*/
        VLCSEN_SERVICE_ADD_EVENT(
            e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_HYPO_UPDATE,
            0); /* start profiling for CDUpdateHypothesisHistory */
        CDUpdateHypothesisHistory(pInternalStatus);
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd, VLCSEN_RTA_CD_HYPO_UPDATE,
                                 0);

        /*----------------------------------------  copy the most important
         * hypotheses to outputs  ----------------------------------------*/
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_SORT_HYPO,
                                 0);
        CDSortHypotheses(
            pInternalStatus, pOutputData->rgRelevantHypothesesList,
            pMeasurementData, pInputData->pObjectData,
            (const CDInternalObjectList_t *)pInternalStatus->rgObjInternal);
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd, VLCSEN_RTA_CD_SORT_HYPO, 0);

        /*----------------------------------------  calc customer specific
         * functions  ----------------------------------------*/
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoStart,
                                 VLCSEN_RTA_CD_CUSTOMER_FUNCTIONS,
                                 0); /* start profiling for CDSortHypotheses */

        CDCalcCustomerFunctions(pInternalStatus, pInputData->pObjectData,
                                pExternalFunctions);

        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                 VLCSEN_RTA_CD_CUSTOMER_FUNCTIONS, 0);
        pOutputData->uiNumberOfHypotheses = MIN(
            pInternalStatus->uiNofRelevantHypotheses, CD_NUMBER_OF_HYPOTHESES);
    } /*local_error == GDB_ERROR_NONE*/

    return local_error;
}

/* ***********************************************************************
  @fn            CDUpdateHypothesisHistory */ /*!

                               @brief         Update the history information for
                             the relevant hypotheses

                               @description   The function updates the
                             hypothesis history (lifetime) of all relevant
                             hypotheses

                               @param[in]     pInternalStatus Pointer to CD
                             internal data

                               @return        void

                               @pre           Hypothesis calculation has been
                             executed

                               @post          [none]

                             ****************************************************************************
                             */
static void CDUpdateHypothesisHistory(
    const CDInternalStatus_t *pInternalStatus) {
    uint32 uiRelHypIndex;
    uint32 uiPrevHypIndex;

    for (uiRelHypIndex = 0u;
         uiRelHypIndex < pInternalStatus->uiNofRelevantHypotheses;
         uiRelHypIndex++) {
        CDIntHypothesis_t *const currentRelHypo =
            &((*pInternalStatus->rgIntRelevantHypothesesList)[uiRelHypIndex]);

        for (uiPrevHypIndex = 0u;
             uiPrevHypIndex < pInternalStatus->uiNofPreviousHypotheses;
             uiPrevHypIndex++) {
            const CDIntHypothesis_t *const currentPrevHypo =
                &((*pInternalStatus->rgPreviousHypothesesList)[uiPrevHypIndex]);

            if ((currentRelHypo->eType == currentPrevHypo->eType) &&
                (currentRelHypo->iObjectRef == currentPrevHypo->iObjectRef)) {
                currentRelHypo->fHypothesisLifetime =
                    currentPrevHypo->fHypothesisLifetime + VLC_CYCLE_TIME;
            }
        }
    }
}

/* ***********************************************************************
  @fn            CDDeleteInternalObject */ /*!

                                  @brief         Delete all Informations of CD
                                if objects will be deleted

                                  @description   The function delete values from
                                the CD component

                                  @param[in]     pLocalObject

                                  @return        void

                                  @pre           [none]

                                  @post          [none]

                                ****************************************************************************
                                */
void CDDeleteInternalObject(CDInternalObject_t *const pLocalObject) {
    CDInitInternalObjData(pLocalObject);

    pLocalObject->TrackAssigned =
        0u; /* Shifting Register for Track Assignment */
    pLocalObject->HypothesisHist.CutIn = 0u;
    pLocalObject->HypothesisHist.Following = 0u;
    pLocalObject->HypothesisHist.Pass = 0u;
    pLocalObject->HypothesisHist.RunUpMoving = 0u;
    pLocalObject->HypothesisHist.RunUpStationary = 0u;
    pLocalObject->HypothesisHist.WasOncomming = 0u;
    pLocalObject->HypothesisHist.PedColl = 0u;
    pLocalObject->HypothesisHist.PedPass = 0u;
    pLocalObject->HypothesisHist.CrossingLeft = 0u;
    pLocalObject->HypothesisHist.CrossingRight = 0u;
    pLocalObject->HypothesisHist.WasCrossing = 0u;
    pLocalObject->uiCriticalTimeAfterMerge = 0u;

    pLocalObject->bitHypPresel = 0x0u;

    CDHypoRunUpInitInternalData(&pLocalObject->sHypRunUpData);
}

/* ************************************************************************* */
/*   Copyright Tuerme                     */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */