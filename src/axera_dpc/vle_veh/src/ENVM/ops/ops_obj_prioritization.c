/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"
#include "ops.h"
#include "ops_par.h"
#include "ops_obj_prioritization.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
sint8 a_EnvmObjIDToFCTObjIDConvert[(uint32)Envm_NR_PRIVOBJECTS];

/*! Mapping of EM object IDs to FCT object IDs in the last cycle */
static sint8 a_EMLastObjIDToFCTObjIDConvert[Envm_NR_PRIVOBJECTS];

/*! Merged object handling: Store at the i-th position the object into which the
 * i-th object is merged, if there is a special handling necessary in
 * em_obj_output.c */
static sint8 a_ObjMergeToThisObj[Envm_NR_PRIVOBJECTS]; /*!< Remark: Needs to be
                                                          global since access
                                                          via an
                                                          external function
                                                          (FPSGetIDRefToMerge)
                                                          */

/*! EM object ID list, which contains only the EM object ID's of the selected
   objects for FCT (length: Envm_N_OBJECTS); Here, also merged objects are
   considered since this information need to be brought to FCT. Remark: The
   position of the same object, which is selected in consecutive cycles, must be
   constant (position in this list equals the position in the FCT object list
   and the FCT object ID) */
static Envm_t_ObjectPrioIndexArray a_ObjectPrioIndex;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*************************************************************************************************************************
  Functionname:    FPSGetIDRefToMerge */
sint8 FPSGetIDRefToMerge(const sint8 FCTObjNr, const sint8 EMObjNr) {
    uint32 i;
    sint8 EMObjNrRef, EMObjNrMergedObj;

    /*! EM-ID of the object into which the object with FCTObjNr in the last
       cycle was merged. Merge-pointer in the object list should point to
       EMObjNrRef. Default: No reference */
    EMObjNrRef = OBJ_INDEX_NO_OBJECT;

    /*! Check if EM- and FCT-ID valid; default of FCTObjNr is
     * OBJ_INDEX_NO_OBJECT */
    if ((EMObjNr >= 0) && (EMObjNr < Envm_NR_PRIVOBJECTS) &&
        (FCTObjNr > OBJ_INDEX_NO_OBJECT)) {
        EMObjNrMergedObj = OBJ_INDEX_NO_OBJECT; /*!< Default value of EM-ID of
                                                   object that was merged */
        for (i = 0u; (i < (uint32)Envm_NR_PRIVOBJECTS) &&
                     (EMObjNrMergedObj == OBJ_INDEX_NO_OBJECT);
             i++) {
            /*! FCTObjNr is the FCT-ID of the merged object since the new object
               was set on the same position in the FCT list as the merged object
                -> EM-ID of the merged object is the EM-ID of the merged object
               in the last cycle */
            if (a_EMLastObjIDToFCTObjIDConvert[i] == FCTObjNr) {
                /*! EM-ID of object that was merged. If no merge occurred:
                   EMObjNrMergedObj is the EM-ID of the object with the same
                   FCT-ID in the last cycle (should be the same as EMObjNr) */
                EMObjNrMergedObj = (sint8)i;
            }
        }

        /*! In the last cycle there was an object at the FCTObjNr-Position. In
         * case of a merge, this object was merged into the object EMObjNrRef */
        if (EMObjNrMergedObj != OBJ_INDEX_NO_OBJECT) {
            /*! EM-ID of the object into which the object with FCTObjNr in the
               last cycle was merged; Merge-pointer in the object list should
               point to EMObjNrRef. If no merge occurred, EMObjNrRef is
               OBJ_INDEX_NO_OBJECT */
            EMObjNrRef = a_ObjMergeToThisObj[EMObjNrMergedObj];
        }
    }

    return EMObjNrRef;
}

/*************************************************************************************************************************
  Functionname:    FPS_i_GetFCTObjIDLastCycle */
sint8 FPS_i_GetFCTObjIDLastCycle(uint32 const ui_Index) {
    sint8 i_ObjIDLastCycle;

    /*! Set default value */
    i_ObjIDLastCycle = OBJ_INDEX_NO_OBJECT;

    /*! If ui_Index is valid, set corresponding element of
     * a_EMLastObjIDToFCTObjIDConvert */
    if (ui_Index < (uint32)Envm_NR_PRIVOBJECTS) {
        i_ObjIDLastCycle = a_EMLastObjIDToFCTObjIDConvert[ui_Index];
    }

    return i_ObjIDLastCycle;
}

/*************************************************************************************************************************
  Functionname:    TUE_OPS_PrioListOutputProcess */
void TUE_OPS_PrioListOutputProcess(void) {
    uint32 uiIndex, uiCountObjPrio, uiNumObjMergedObjToMergeNew;
    sint8 iObjNumber;
    BML_t_TrajRefPoint DistVDYPoint2Circle;
    // float32 f_DistToTrajAbs;

    Envm_t_ObjectPrioIndexArray a_ObjectPrioIndexRangeSorted;
    // boolean ab_ObjSelecteByPrio[Envm_NR_PRIVOBJECTS];

    uiCountObjPrio = 0u;
    uiNumObjMergedObjToMergeNew = 0u;

    // Init object distX sorted array
    for (uiIndex = 0; uiIndex < (uint32)Envm_N_OBJECTS; uiIndex++) {
        a_ObjectPrioIndexRangeSorted[uiIndex] = Envm_INVALID_ID_INDEX;
    }

    // Update Last EM array to FCT array and Set default value
    for (uiIndex = 0; uiIndex < (uint32)Envm_NR_PRIVOBJECTS; uiIndex++) {
        // ab_ObjSelecteByPrio[uiIndex] = FALSE;
        a_EMLastObjIDToFCTObjIDConvert[uiIndex] =
            a_EnvmObjIDToFCTObjIDConvert[uiIndex];
        a_EnvmObjIDToFCTObjIDConvert[uiIndex] = OBJ_INDEX_NO_OBJECT;
        a_ObjMergeToThisObj[uiIndex] = OBJ_INDEX_NO_OBJECT;
    }

    // Init Output Array Value
    for (uiIndex = 0; uiIndex < Envm_NR_PRIVOBJECTS; uiIndex++) {
        a_ObjectPrioIndex[uiIndex] = (uint8)uiIndex;
        a_EnvmObjIDToFCTObjIDConvert[uiIndex] = (sint8)uiIndex;
    }
    uiCountObjPrio = Envm_NR_PRIVOBJECTS;

    // Get range sorted object id
    OPS_v_GenerateRangeSortedObjPrioList(a_ObjectPrioIndexRangeSorted);
    // Update OPS result to EM Object Data
    Envm_v_ObjOutSetPrioIndexList(a_ObjectPrioIndex,
                                  a_ObjectPrioIndexRangeSorted);
}

/*************************************************************************************************************************
  Functionname:    OPS_v_GenerateRangeSortedObjPrioList */
static void OPS_v_GenerateRangeSortedObjPrioList(
    Envm_t_ObjectPrioIndexArray pa_ObjectPrioIndexRangeSorted) {
    uint32 uiIndex;
    uint32 uiIndexSort;
    sint8 iObjNumber;

    uiIndexSort = 0;
    for (uiIndex = 0; uiIndex < EM_INT_OBJ_NUMBER_OF_OBJ_USED; uiIndex++) {
        iObjNumber = Envm_INT_OBJ_INDEX_DISTX_SORTED[uiIndex];

        if ((a_EnvmObjIDToFCTObjIDConvert[iObjNumber] > OBJ_INDEX_NO_OBJECT) &&
            (a_EnvmObjIDToFCTObjIDConvert[iObjNumber] <
             (sint8)Envm_N_OBJECTS) &&
            (uiIndexSort < (uint32)Envm_N_OBJECTS)) {
            // Set FCT object id
            pa_ObjectPrioIndexRangeSorted[uiIndexSort] =
                (uint8)a_EnvmObjIDToFCTObjIDConvert[iObjNumber];
            uiIndexSort++;
        }
    }
}
/*************************************************************************************************************************
  Functionname:    TUE_OPSInitObjectPrioritization */
void OPSInitObjectPrioritization(void) {
    /*! Initialization of static variables for the object priorization */
    FPS_v_InitObjPrioStaticData();
    /*! Set default data for output list (take first objects) */
    Envm_v_ObjOutSetPrioIndexList(NULL, NULL);
}

/*************************************************************************************************************************
  Functionname:    OPS_v_InitObjPrioStaticData */
void FPS_v_InitObjPrioStaticData(void) {
    uint32 ui_Index;

    for (ui_Index = 0; ui_Index < (uint32)Envm_N_OBJECTS; ui_Index++) {
        /*! EM object ID list, which contains only the EM object ID's of the
         * selected objects for FCT (length: Envm_N_OBJECTS) */
        a_ObjectPrioIndex[ui_Index] = Envm_INVALID_ID_INDEX;
    }

    for (ui_Index = 0; ui_Index < (uint32)Envm_NR_PRIVOBJECTS; ui_Index++) {
        /*! Mapping of EM object IDs to FCT object IDs */
        a_EMLastObjIDToFCTObjIDConvert[ui_Index] = OBJ_INDEX_NO_OBJECT;

        a_ObjMergeToThisObj[ui_Index] = OBJ_INDEX_NO_OBJECT;
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */