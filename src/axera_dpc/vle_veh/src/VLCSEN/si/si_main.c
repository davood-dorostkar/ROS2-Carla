/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "si.h"
#include "si_par.h"
#include "stddef.h"
#include "TM_Global_Types.h"
/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! Local declaration of si custom debug data */
SET_MEMSEC_VAR(SICustomOutputDebugData)
/*! @VADDR: SI_CUSTOM_OUTPUT_DEBUG_DATA_VADDR @CYCLEID: VLC_ENV */
SICustomOutputDebugInfo_t SICustomOutputDebugData;

/*! sub-module state */
SET_MEMSEC_VAR(SIState)
SIState_t SIState;

SET_MEMSEC_VAR(SIBasePreselObjList)
BasePreselObjList_t SIBasePreselObjList[Envm_N_OBJECTS];

SET_MEMSEC_VAR(SITrajectoryData)

CPTrajectoryData_t SITrajectoryData;
SET_MEMSEC_VAR(SICourseData)

CPCourseData_t SICourseData;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/
#define SI_PREDICTED_LATERAL_DISPLACEMENT (999.f)

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

static void SIInitOOIList(void);
static void SIInitSelectionMarks(void);

/*************************************************************************************************************************
  Functionname:    SIInitOOIList */
static void SIInitOOIList(void) {
    ObjNumber_t nr;

    /*clear all objects in ooi*/
    for (nr = 0; nr < SiAnzOOI; nr++) {
        OBJ_GET_OOI_LIST_OBJ_IDX(nr) = OBJ_INDEX_NO_OBJECT;
    }

    /*clear all SI related information in TPObject*/
    for (nr = (ObjNumber_t)(Envm_N_OBJECTS - 1); nr >= 0; nr--) {
        SIDeleteObject(nr);
        SI_OBJ_SET_OBJ_OF_INTEREST(nr, OBJ_NOT_OOI);
        OBJ_GET_SI(nr).fPredictedLatDispl = SI_PREDICTED_LATERAL_DISPLACEMENT;
        OBJ_GET_CUT_IN_POTENTIAL(nr) = 0u;
        OBJ_GET_CUT_OUT_POTENTIAL(nr) = 0u;
        OBJ_ABS_VELO_X(nr) = 0.f;  /*initialize to 0m/s*/
        OBJ_ABS_ACCEL_X(nr) = 0.f; /*initialize to 0m/(s^2)*/
        OBJ_GET_SI(nr).ObjLaneAccStatus.fCorridorRelevantTime = 0.f;

        OBJ_GET_SI(nr).ObjLaneAccStatus.SIInlaneState = OBJ_STATE_OUTLANE;
        OBJ_GET_SI(nr).ObjLaneAccStatus.SIActLaneState = OBJ_STATE_OUTLANE;
        OBJ_GET_SI(nr).ObjLaneAccStatus.In2OutlaneTransition = 0u;
        OBJ_GET_SI(nr).Bool.Moving = 0u;
        OBJ_GET_SI(nr).Bool.Oncoming = 0u;
        OBJ_GET_SI(nr).Bool.Stationary = 0u;
        OBJ_GET_SI(nr).Bool.SelectedAsOOI = 0u;
        OBJ_GET_SI(nr).Bool.AlreadyOOI = 0u;
        OBJ_GET_SI(nr).Bool.SelectedByPathDecision = 0u;
        OBJ_GET_SI(nr).Bool.FctPreselTG = 0u;
        OBJ_GET_SI(nr).Bool.InLOccValue = 0u;
        OBJ_GET_SI(nr).Bool.InLCustomValue = 0u;
        OBJ_GET_SI(nr).Bool.InLQualityValue = 0u;
        OBJ_GET_SI(nr).Bool.InLObjOccValue = 0u;
        OBJ_GET_SI(nr).Bool.InLLaneOccValue = 0u;
        OBJ_GET_SI(nr).Bool.InLTimeValue = 0u;
        OBJ_GET_SI(nr).Bool.OutLOccValue = 0u;
        OBJ_GET_SI(nr).Bool.OutLCustomValue = 0u;
        OBJ_GET_SI(nr).Bool.OutLObjOccValue = 0u;
        OBJ_GET_SI(nr).Bool.OutLLaneOccValue = 0u;
        OBJ_GET_SI(nr).Bool.StatObjWasOncoming = 0u;
        OBJ_GET_SI(nr).Bool.OccludedByTrace = 0u;
        OBJ_GET_SI(nr).StatObjWasOncoming.uiOncomingCounter = 0u;
        OBJ_GET_SI(nr).BlockedPathDecision.PathSelectionTimer = 0u;
    }
}

/*************************************************************************************************************************
  Functionname:    SIReInit */
void SIReInit(void) {
    ObjNumber_t i;

    /*clear all SI related information in TPObject if not selected as OOI*/
    for (i = (ObjNumber_t)(Envm_N_OBJECTS - 1); i >= 0; i--) {
        if (OBJ_GET_SI(i).Bool.SelectedAsOOI == FALSE) {
            SI_OBJ_SET_OBJ_OF_INTEREST(i, OBJ_NOT_OOI);
            OBJ_GET_SI(i).fPredictedLatDispl =
                SI_PREDICTED_LATERAL_DISPLACEMENT;
            OBJ_GET_CUT_IN_POTENTIAL(i) = 0u;
            OBJ_GET_CUT_OUT_POTENTIAL(i) = 0u;
            OBJ_GET_SI(i).Bool.SelectedAsOOI = 0u;
            OBJ_GET_SI(i).Bool.AlreadyOOI = 0u;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIInitSelectionMarks */
static void SIInitSelectionMarks(void) {
    ObjNumber_t i;

    for (i = (ObjNumber_t)(Envm_N_OBJECTS - 1); i >= 0; i--) {
        OBJ_GET_SI(i).Bool.SelectedAsOOI = 0u;
    }
}

/*************************************************************************************************************************
  Functionname:    SIInit */
void SIInit(void) {
    /* Initialize ACC function preselection */
    SIInitAccFunPreselection();

    /* Initialize lane change detection module */
    SIInitLaneChange();

    /* init OOI list */
    SIInitOOIList();

    /* Set no object selected as relevant object */
    SISeReObPutRelTrckObjNumber(OBJ_INDEX_NO_OBJECT);

    /* Module Observation Relevant Object initialization */
    SIObReObInit();

    /* Module Observation Relevant Object initialization */
    SIReSiDaInit();

    /* initializes all Object to Trajectory related variables*/
    SIObj2TrajInit();

    /* initialize customer functions */
    SIInitCustomerFunctions();

    /*! Initialize blocked path relevant infos */
    SIInitBlockedPath();
}

/*************************************************************************************************************************
  Functionname:    VLCSIProcess */
void VLCSIProcess(void) {
    VLC_PUB_OBJ_LIST_VERSION = VLC_ASSESSED_OBJ_INTFVER;

    /* Verify that SI is in a state where it is allowed to run, and the input
    object list from EM is set to valid */
    if (((SIState == SI_OK) || (SIState == SI_RED_QUAL)) &&
        (GET_Envm_PUB_OBJ_DATA_PTR->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
        /* init OOI list cycle marks */
        SIInitSelectionMarks();

        /* Do lane change detection so SI can query result */
        SIDetectLaneChange();

        /* Perform object-to-object (O2O) relation based lane association */
        SI_v_PerformO2OLaneAssociation();

        /* Perform object pre-selection (filtering) */
        SIObjectPreselection();

        /* lane association */
        SILaneAssociation();

        /* main configurable processing */
        SICustomProcess();

        /* filter object acceleration using merged ego acceleration */
        SICalcObjAttributes();

        /* generate outputdata*/
        SIGenerateOutputData();

        /* freeze si custom output debug data */
        SIFreezeCustomOutputDebugData();

        /* Set state of output data together with cycle counter */
        GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.uiTimeStamp =
            GET_Envm_PUB_OBJ_DATA_PTR->sSigHeader.uiTimeStamp;
        GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.uiMeasurementCounter =
            GET_Envm_PUB_OBJ_DATA_PTR->sSigHeader.uiMeasurementCounter;
        GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.uiCycleCounter =
            VLCSenFrame.uiCycleCounter;
        GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.eSigStatus = AL_SIG_STATE_OK;
        VLC_PUB_OBJ_LIST_NUM_OBJS = OBJ_NUMBER_OF_OBJ_USED;
    } else {
        if (SIState == SI_INIT) {
            /* Initialize SI internal variables */
            SIInit();
        }
        /* Set output data to invalid */
        GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.uiTimeStamp =
            GET_Envm_PUB_OBJ_DATA_PTR->sSigHeader.uiTimeStamp;
        GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.uiMeasurementCounter =
            GET_Envm_PUB_OBJ_DATA_PTR->sSigHeader.uiMeasurementCounter;
        GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.uiCycleCounter =
            VLCSenFrame.uiCycleCounter;
        GET_VLC_PUB_OBJ_DATA_PTR->sSigHeader.eSigStatus =
            ((SIState == SI_INIT) ? AL_SIG_STATE_INIT : AL_SIG_STATE_INVALID);

        for (int i = 0; i < Acc_max_number_ooi; i++) {
            GET_VLC_PUB_OBJ_DATA_PTR->HeaderAssessedObjList.aiOOIList[0] = -1;
        }

        VLC_PUB_OBJ_LIST_NUM_OBJS = 0;

        SIDeleteOOIData(&GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLong.Kinematic,
                        &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLong.Attributes);
        SIDeleteOOIData(&GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLeft.Kinematic,
                        &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLeft.Attributes);
        SIDeleteOOIData(&GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextRight.Kinematic,
                        &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextRight.Attributes);
        SIDeleteOOIData(
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLong.Kinematic,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLong.Attributes);
        SIDeleteOOIData(
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLeft.Kinematic,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLeft.Attributes);
        SIDeleteOOIData(
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextRight.Kinematic,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextRight.Attributes);
    }
}

/*************************************************************************************************************************
  Functionname:    SIMergeObjects */
void SIMergeObjects(ObjNumber_t iObjectToKeep, ObjNumber_t iObjectToDelete) {
    /*! Take care: If new information considered here, check if the same
     * information should be considered in SIMergeDeleteObjectSameVLCID(...) !!!
     */

    /* keep object relevance */
    if (OBJ_GET_RELEVANT(iObjectToDelete)) {
        SI_OBJ_SET_OBJ_OF_INTEREST(iObjectToKeep, OBJ_NEXT_OOI);

        /* Write information of relevance (OOI-0) */
        SISeReObPutRelTrckObjNumber(iObjectToKeep);

        /* The duration of relevance must be taken over.  */
        OBJ_GET_SI(iObjectToKeep).ObjCor.TrackVehicle.fLatTrackLimitExpandFac =
            OBJ_GET_SI(iObjectToDelete)
                .ObjCor.TrackVehicle.fLatTrackLimitExpandFac;
    }

    /* Merge boolean flags of seen moving/oncomming/stationary */
    if (OBJ_GET_SI(iObjectToDelete).Bool.Moving) {
        OBJ_GET_SI(iObjectToKeep).Bool.Moving = 1u;
    }
    if (OBJ_GET_SI(iObjectToDelete).Bool.Oncoming) {
        OBJ_GET_SI(iObjectToKeep).Bool.Oncoming = 1u;
    }
    if (OBJ_GET_SI(iObjectToDelete).Bool.Stationary) {
        OBJ_GET_SI(iObjectToKeep).Bool.Stationary = 1u;
    }

    /* value of path selection timer must be kept */
    if (OBJ_GET_SI(iObjectToKeep).BlockedPathDecision.PathSelectionTimer <
        OBJ_GET_SI(iObjectToDelete).BlockedPathDecision.PathSelectionTimer) {
        OBJ_GET_SI(iObjectToKeep).BlockedPathDecision.PathSelectionTimer =
            OBJ_GET_SI(iObjectToDelete).BlockedPathDecision.PathSelectionTimer;
    }

    /* Merge object corridor data pulling merged object toward ego lane (i.e.:
     * prioritizing inlane) */
    OBJ_GET_SI(iObjectToKeep).ObjLaneAccStatus.SIInlaneState =
        MIN(OBJ_GET_SI(iObjectToKeep).ObjLaneAccStatus.SIInlaneState,
            OBJ_GET_SI(iObjectToDelete).ObjLaneAccStatus.SIInlaneState);
    /* Keep higher of two inlane to outlane transition counters */
    OBJ_GET_SI(iObjectToKeep).ObjLaneAccStatus.In2OutlaneTransition =
        MAX(OBJ_GET_SI(iObjectToKeep).ObjLaneAccStatus.In2OutlaneTransition,
            OBJ_GET_SI(iObjectToDelete).ObjLaneAccStatus.In2OutlaneTransition);
    /* Keep higher of two corridor relevant timers */
    OBJ_GET_SI(iObjectToKeep).ObjLaneAccStatus.fCorridorRelevantTime =
        MAX_FLOAT(
            OBJ_GET_SI(iObjectToKeep).ObjLaneAccStatus.fCorridorRelevantTime,
            OBJ_GET_SI(iObjectToDelete).ObjLaneAccStatus.fCorridorRelevantTime);
    /* Keep higher of two corridor relevant distances */
    OBJ_GET_SI(iObjectToKeep).ObjLaneAccStatus.fCorridorRelevantDist =
        MAX_FLOAT(
            OBJ_GET_SI(iObjectToKeep).ObjLaneAccStatus.fCorridorRelevantDist,
            OBJ_GET_SI(iObjectToDelete).ObjLaneAccStatus.fCorridorRelevantDist);

    /* merge customer specific data */
    SIMergeCustomObjects(iObjectToKeep, iObjectToDelete);
}

/*************************************************************************************************************************
  Functionname:    SIMergeDeleteObjectSameVLCID */
void SIMergeDeleteObjectSameVLCID(ObjNumber_t ObjNr) {
    /*! Take care: If new information considered here, check if the same
     * information should be considered in SIMergeObjects(...) !!! */

    /*! First: Store the information locally which is considered for merging */
    boolean b_ObjIsRelevant = (boolean)OBJ_GET_RELEVANT(ObjNr);
    float32 f_RelSpurErweiterungsFaktor =
        OBJ_GET_SI(ObjNr).ObjCor.TrackVehicle.fLatTrackLimitExpandFac;
    boolean b_Moving = (boolean)OBJ_GET_SI(ObjNr).Bool.Moving;
    boolean b_Oncoming = (boolean)OBJ_GET_SI(ObjNr).Bool.Oncoming;
    boolean b_Stationary = (boolean)OBJ_GET_SI(ObjNr).Bool.Stationary;
    uint8 ui_PathSelectionTimer =
        OBJ_GET_SI(ObjNr).BlockedPathDecision.PathSelectionTimer;
    SILaneState_t SILateState =
        OBJ_GET_SI(ObjNr).ObjLaneAccStatus.SIInlaneState;
    uint8 ui_In2OutlaneTransition =
        OBJ_GET_SI(ObjNr).ObjLaneAccStatus.In2OutlaneTransition;
    float32 f_CorridorRelevantTime =
        OBJ_GET_SI(ObjNr).ObjLaneAccStatus.fCorridorRelevantTime;
    float32 f_CorridorRelevantDist =
        OBJ_GET_SI(ObjNr).ObjLaneAccStatus.fCorridorRelevantDist;

    /*! Store customer specific data locally (compare to SIMergeCustomObjects(
     * iObjectToKeep , iObjectToDelete ) ) */

    /*! Second: Delete the VLC-object */
    SIDeleteObject(ObjNr);

    /*! Third: Decide which merge-information should be copied */

    /*! Keep object relevance */
    if (b_ObjIsRelevant == TRUE) {
        SI_OBJ_SET_OBJ_OF_INTEREST(ObjNr, OBJ_NEXT_OOI);

        /*! Write the object id of the relevant tracked object */
        SISeReObPutRelTrckObjNumber(ObjNr);

        /*! Duration of relevance must be retained */
        OBJ_GET_SI(ObjNr).ObjCor.TrackVehicle.fLatTrackLimitExpandFac =
            f_RelSpurErweiterungsFaktor;
    }

    /*! Merge boolean flags of seen moving/oncomming/stationary */
    if (b_Moving == TRUE) {
        OBJ_GET_SI(ObjNr).Bool.Moving = 1u;
    }
    if (b_Oncoming == TRUE) {
        OBJ_GET_SI(ObjNr).Bool.Oncoming = 1u;
    }
    if (b_Stationary == TRUE) {
        OBJ_GET_SI(ObjNr).Bool.Stationary = 1u;
    }

    /*! Value of path selection timer must be kept */
    if (OBJ_GET_SI(ObjNr).BlockedPathDecision.PathSelectionTimer <
        ui_PathSelectionTimer) {
        OBJ_GET_SI(ObjNr).BlockedPathDecision.PathSelectionTimer =
            ui_PathSelectionTimer;
    }

    /*! Merge object corridor data pulling merged object toward ego lane (i.e.:
     * prioritizing inlane) */
    OBJ_GET_SI(ObjNr).ObjLaneAccStatus.SIInlaneState =
        MIN(OBJ_GET_SI(ObjNr).ObjLaneAccStatus.SIInlaneState, SILateState);
    /*! Keep higher of two inlane to outlane transition counters */
    OBJ_GET_SI(ObjNr).ObjLaneAccStatus.In2OutlaneTransition =
        MAX(OBJ_GET_SI(ObjNr).ObjLaneAccStatus.In2OutlaneTransition,
            ui_In2OutlaneTransition);
    /*! Keep higher of two corridor relevant timers */
    OBJ_GET_SI(ObjNr).ObjLaneAccStatus.fCorridorRelevantTime =
        MAX_FLOAT(OBJ_GET_SI(ObjNr).ObjLaneAccStatus.fCorridorRelevantTime,
                  f_CorridorRelevantTime);
    /*! Keep higher of two corridor relevant distances */
    OBJ_GET_SI(ObjNr).ObjLaneAccStatus.fCorridorRelevantDist =
        MAX_FLOAT(OBJ_GET_SI(ObjNr).ObjLaneAccStatus.fCorridorRelevantDist,
                  f_CorridorRelevantDist);

    /*! Merge customer specific data: It might be necessary based on the
      implementation of SIMergeCustomObjects to merge the custom specific data.
      The function SIMergeCustomObjects( iObjectToKeep , iObjectToDelete ) must
      be adapted for the case that iObjectToKeep = iObjectToDelete */
}

/*************************************************************************************************************************
  Functionname:    SIGetCourseData */
CPCourseData_t *SIGetCourseData(void) { return &SICourseData; }

/*************************************************************************************************************************
  Functionname:    SIGetTrajectoryData */
CPTrajectoryData_t *SIGetTrajectoryData(void) { return &SITrajectoryData; }

/*************************************************************************************************************************
  Functionname:    SIProcessTrajectoriesMeas */
void SIProcessTrajectoriesMeas(CPTrajMeasInfo_t *pTrajectoriesMeas) {
    CPCopyCourse2Meas(&SICourseData, &pTrajectoriesMeas->FiltCourse);
    CPCopyTraj2Meas(&SITrajectoryData, &pTrajectoriesMeas->Trajectory);
    pTrajectoriesMeas->LaneWidth = SIGetBaseSeekLaneWidth();
}

/*************************************************************************************************************************
  Functionname:    SITrajGetObjToRefDistance */
void SITrajGetObjToRefDistance(ObjNumber_t ObjId,
                               float32 *fDist,
                               float32 *fDistVar) {
    *fDist = CPTrajGetObjToRefDistMeas(&OBJ_GET_CP(ObjId).TrajDist);
    *fDistVar = CPTrajGetObjToRefDistFiltVar(&OBJ_GET_CP(ObjId).TrajDist);
}

/*************************************************************************************************************************
  Functionname:    SITrajGetObjToRefDistanceGradient */
void SITrajGetObjToRefDistanceGradient(ObjNumber_t ObjId,
                                       float32 *fDistGrad,
                                       float32 *fDistGradVar) {
    *fDistGrad = CPTrajGetObjToRefDistGradFilt(&OBJ_GET_CP(ObjId).TrajDist);
    *fDistGradVar = CPTrajGetObjToRefDistGradFiltVar(
        &OBJ_GET_CP(ObjId).TrajDist); /*!< Ramark: fDistGradVar >= 0 */
}

/*************************************************************************************************************************
  Functionname:    SIDeleteObject */
void SIDeleteObject(ObjNumber_t ObjId) {
    /* initialize SI attributes */
    SICorridorObjInit(&OBJ_GET_SI(ObjId).ObjCor);
    SI_OBJ_SET_OBJ_OF_INTEREST(ObjId, OBJ_NOT_OOI);
    OBJ_GET_SI(ObjId).fPredictedLatDispl = SI_PREDICTED_LATERAL_DISPLACEMENT;
    OBJ_GET_CUT_IN_POTENTIAL(ObjId) = 0u;
    OBJ_GET_CUT_OUT_POTENTIAL(ObjId) = 0u;
    OBJ_ABS_VELO_X(ObjId) = 0.0F;
    OBJ_ABS_ACCEL_X(ObjId) = 0.0F;
    OBJ_GET_SI(ObjId).ObjLaneAccStatus.fCorridorRelevantTime = 0.0f;

    OBJ_GET_SI(ObjId).ObjLaneAccStatus.fCorridorRelevantDist = 0.0f;
    OBJ_GET_SI(ObjId).ObjLaneAccStatus.SIInlaneState = OBJ_STATE_OUTLANE;
    OBJ_GET_SI(ObjId).ObjLaneAccStatus.SIActLaneState = OBJ_STATE_OUTLANE;
    OBJ_GET_SI(ObjId).ObjLaneAccStatus.In2OutlaneTransition = 0u;
    OBJ_GET_SI(ObjId).Bool.Moving = 0u;
    OBJ_GET_SI(ObjId).Bool.Oncoming = 0u;
    OBJ_GET_SI(ObjId).Bool.Stationary = 0u;
    OBJ_GET_SI(ObjId).Bool.SelectedAsOOI = 0u;
    OBJ_GET_SI(ObjId).Bool.AlreadyOOI = 0u;
    OBJ_GET_SI(ObjId).Bool.SelectedByPathDecision = 0u;
    OBJ_GET_SI(ObjId).Bool.FctPreselTG = 0u;
    OBJ_GET_SI(ObjId).Bool.InLOccValue = 0u;
    OBJ_GET_SI(ObjId).Bool.InLCustomValue = 0u;
    OBJ_GET_SI(ObjId).Bool.InLQualityValue = 0u;
    OBJ_GET_SI(ObjId).Bool.InLObjOccValue = 0u;
    OBJ_GET_SI(ObjId).Bool.InLLaneOccValue = 0u;
    OBJ_GET_SI(ObjId).Bool.InLTimeValue = 0u;
    OBJ_GET_SI(ObjId).Bool.OutLOccValue = 0u;
    OBJ_GET_SI(ObjId).Bool.OutLCustomValue = 0u;
    OBJ_GET_SI(ObjId).Bool.OutLObjOccValue = 0u;
    OBJ_GET_SI(ObjId).Bool.OutLLaneOccValue = 0u;
    OBJ_GET_SI(ObjId).Bool.StatObjWasOncoming = 0u;
    OBJ_GET_SI(ObjId).Bool.OccludedByTrace = 0u;

    OBJ_GET_SI(ObjId).StatObjWasOncoming.uiOncomingCounter = 0u;
    OBJ_GET_SI(ObjId).BlockedPathDecision.PathSelectionTimer = 0u;
}

/*************************************************************************************************************************
  Functionname:    SIMeasCallback */
void SIMeasCallback(void) { return; }

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */