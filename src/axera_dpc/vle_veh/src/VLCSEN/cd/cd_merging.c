/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cd.h"

/*************************************************************************************************************************
  Functionname:    CDMergeInternalObjects */ /*!

      @brief           Merging of internal objects

      @description     Selected objects are pObjectToKeep and pObjectToDeletee.
                       Significant information from both objects will be merged
                       into object pObjectToKeep.

      @return          void

      @param[in,out]   pObjectToKeep : The object to keep after merge.
      @param[in]       pObjectToDelete : The object to delete after merge.


      @todo            Fill CD-merging functionality

    *************************************************************************************************************************/
void CDMergeInternalObjects(CDInternalObject_t *const pObjectToKeep,
                            const CDInternalObject_t *const pObjectToDelete) {
    pObjectToKeep->HypothesisHist.CutIn |=
        pObjectToDelete->HypothesisHist.CutIn;
    pObjectToKeep->HypothesisHist.Following |=
        pObjectToDelete->HypothesisHist.Following;
    pObjectToKeep->HypothesisHist.Pass |= pObjectToDelete->HypothesisHist.Pass;
    pObjectToKeep->HypothesisHist.RunUpMoving |=
        pObjectToDelete->HypothesisHist.RunUpMoving;
    pObjectToKeep->HypothesisHist.RunUpStationary |=
        pObjectToDelete->HypothesisHist.RunUpStationary;
    pObjectToKeep->HypothesisHist.WasOncomming |=
        pObjectToDelete->HypothesisHist.WasOncomming;
    pObjectToKeep->HypothesisHist.PedColl |=
        pObjectToDelete->HypothesisHist.PedColl;
    pObjectToKeep->HypothesisHist.PedPass |=
        pObjectToDelete->HypothesisHist.PedPass;
    pObjectToKeep->HypothesisHist.CrossingLeft |=
        pObjectToDelete->HypothesisHist.CrossingLeft;
    pObjectToKeep->HypothesisHist.CrossingRight |=
        pObjectToDelete->HypothesisHist.CrossingRight;
    pObjectToKeep->HypothesisHist.WasCrossing |=
        pObjectToDelete->HypothesisHist.WasCrossing;
    // wulin add 20220316
    pObjectToKeep->HypothesisHist.BicycleColl |=
        pObjectToDelete->HypothesisHist.BicycleColl;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
