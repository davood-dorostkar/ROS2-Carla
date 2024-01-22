/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "stddef.h"
#include "TM_Global_Types.h"
#include "si.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(SIRelObject)
SIRelObject_t SIRelObject;

static SI_MeasObjs_t m_obj;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/
#define SI_PICKUP_DISTANCE_HARD_BRAKE_EGO_DECELERATION (4.f)
#define SI_PICKUP_DISTANCE_COMFORT_BRAKE_EGO_DECELERATION (2.f)
#define SI_PICKUP_DISTANCE_GAP_TIME (1.8f)
#define SI_PICKUP_DISTANCE_LATENCY_TIME (1.0f)

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/
SET_MEMSEC_CONST(SICustomOutputDebug_MeasInfo)
static const MEASInfo_t SICustomOutputDebug_MeasInfo =

    {
        SI_CUSTOM_OUTPUT_DEBUG_DATA_VADDR, /* .VirtualAddress */
        sizeof(SICustomOutputDebugData),   /* .Length */
        SI_MEAS_FUNC_ID,                   /* .FuncID */
        SI_MEAS_FUNC_CHAN_ID               /* .FuncChannelID */
};

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static ObjNumber_t iTRCKGetracktesObjekt;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void SIReSiDaRelObjInit(void);
static void SIFillOOIKinematic(const ObjNumber_t ObjId,
                               AccOOIGenKinematics_t* pKinematic);
static void SIFillOOIAttributes(const ObjNumber_t ObjId,
                                const eObjOOI_t eObjOOI,
                                AccOOIGenAttributes_t* pAttributes);
void SIDeleteOOIData(AccOOIGenKinematics_t* pKinematic,
                     AccOOIGenAttributes_t* pAttributes);

/*************************************************************************************************************************
  Functionname:    SIFreezeData */
void SIFreezeData(void) {
    eObjOOI_t onr;
    float32 fDummy;

    // static const MEASInfo_t SIOOIList_MeasInfo = {
    //    SI_OOI_LIST_MEAS_VADDR, /* .VirtualAddress */
    //    sizeof(m_obj),          /* .Length */
    //    SI_MEAS_FUNC_ID,        /* .FuncID */
    //    SI_MEAS_FUNC_CHAN_ID    /* .FuncChannelID */
    //};

    for (onr = OBJ_NEXT_OOI; onr <= OBJ_NEXT_LAT_RIGHT_OOI; onr++) {
        SI_Meas_t* const pMeasObj = &m_obj[onr];
        const ObjNumber_t CurObjId = OBJ_GET_OOI_LIST_OBJ_IDX(onr);

        if (CurObjId >= 0) {
            pMeasObj->object_id = CurObjId;
            pMeasObj->lane = OBJ_GET_ASSOCIATED_LANE(CurObjId);
            pMeasObj->object_class = OBJ_CLASSIFICATION(CurObjId);
            pMeasObj->object_type = OBJ_DYNAMIC_PROPERTY(CurObjId);
            pMeasObj->object_sub_prop = OBJ_DYNAMIC_SUB_PROPERTY(CurObjId);
            if (onr > OBJ_HIDDEN_NEXT_OOI) {
                pMeasObj->potential = OBJ_GET_CUT_IN_POTENTIAL(CurObjId);
            } else {
                pMeasObj->potential = OBJ_GET_CUT_OUT_POTENTIAL(CurObjId);
            }
            if (OBJ_IS_MOVING_TO_STATIONARY(CurObjId)) {
                pMeasObj->seen_moving = TRUE;
            } else {
                pMeasObj->seen_moving = FALSE;
            }

            pMeasObj->pred_lat_displ = OBJ_GET_SI(CurObjId).fPredictedLatDispl;
            pMeasObj->long_displacement = OBJ_LONG_DISPLACEMENT(CurObjId);
            pMeasObj->lat_displacement = OBJ_LAT_DISPLACEMENT(CurObjId);
            pMeasObj->rel_long_acceleration = OBJ_LONG_AREL(CurObjId);
            pMeasObj->rel_long_velocity = OBJ_LONG_VREL(CurObjId);
            SITrajGetObjToRefDistanceGradient(
                CurObjId, &pMeasObj->rel_lat_velocity, &fDummy);

#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "Remove negation, when visualization adapted to AUTOSAR coordinates!")
#endif
            pMeasObj->fTraceBracketLeft =
                -VLCObjectList[CurObjId].SI.ObjCor.TrackVehicle.fLatTrackLimitL;
            pMeasObj->fTraceBracketRight =
                -VLCObjectList[CurObjId].SI.ObjCor.TrackVehicle.fLatTrackLimitR;
        } else {
            pMeasObj->object_id = OBJ_INDEX_NO_OBJECT;
            pMeasObj->lane = ASSOC_LANE_UNKNOWN;
            pMeasObj->object_class = CR_OBJCLASS_UNCLASSIFIED;
            pMeasObj->object_type = CR_OBJECT_PROPERTY_MOVING;
            pMeasObj->object_sub_prop = CR_OBJECT_SUBPROP_UNIFAL;
            pMeasObj->potential = 0u;
            pMeasObj->seen_moving = FALSE;

            pMeasObj->pred_lat_displ = 0.f;
            pMeasObj->long_displacement = 0.f;
            pMeasObj->lat_displacement = 0.f;
            pMeasObj->rel_long_acceleration = 0.f;
            pMeasObj->rel_long_velocity = 0.f;
            pMeasObj->rel_lat_velocity = 0.f;

            pMeasObj->fTraceBracketLeft = SIGetBaseSeekLaneWidth();
            pMeasObj->fTraceBracketRight = 0.f;
        }

        /* Call custom function to fill custom meas data field */
        SICustFillMeasOOI(CurObjId, &pMeasObj->custom);
    }

    //(void) VLC_FREEZE_DATA(&SIOOIList_MeasInfo, m_obj, &SIMeasCallback);
}

/*************************************************************************************************************************
  Functionname:    SIFreezeCustomOutputDebugData */
void SIFreezeCustomOutputDebugData(void) {
    //(void)VLC_FREEZE_DATA(&SICustomOutputDebug_MeasInfo, (void
    //*)&SICustomOutputDebugData, &SIMeasCallback);
}

/*************************************************************************************************************************
  Functionname:    SIGenerateOutputData */
void SIGenerateOutputData(void) {
    if (OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI) >= 0) {
        const ObjNumber_t CurObjId = OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI);
        SIRelObject.ObjValid = TRUE;
        if (OBJ_DYNAMIC_PROPERTY(CurObjId) == CR_OBJECT_PROPERTY_STATIONARY) {
            SIRelObject.StatObj = TRUE;
        } else {
            SIRelObject.StatObj = FALSE;
        }

        if (OBJ_IS_MOVING_TO_STATIONARY(CurObjId)) {
            SIRelObject.MovingToStat = TRUE;
        } else {
            SIRelObject.MovingToStat = FALSE;
        }

        SIRelObject.DistX = OBJ_LONG_DISPLACEMENT(CurObjId);
        SIRelObject.RelSpeedX = OBJ_LONG_VREL(CurObjId);
        SIRelObject.RelAcclX = OBJ_LONG_AREL(CurObjId);

        SIRelObject.ObjSpeed = OBJ_ABS_VELO_X(CurObjId);

        SIRelObject.ObjAccl = OBJ_ABS_ACCEL_X(CurObjId);

        SIRelObject.ObjAngle = OBJ_ANGLE(CurObjId);
        SIRelObject.ObjDistY = OBJ_LAT_DISPLACEMENT(CurObjId);
        SIRelObject.RelSpeedY = OBJ_LAT_VREL(CurObjId);

        SIRelObject.LatDisplRoadBordL =
            OBJ_GET_SI(CurObjId).ObjCor.TrackVehicle.fLatTrackLimitL;
        SIRelObject.LatDisplRoadBordR =
            OBJ_GET_SI(CurObjId).ObjCor.TrackVehicle.fLatTrackLimitR;

        /* On relevant object change, store the first pickup distance (formerly
        stored seperately for each object in dAbst_RelevantNeu */
        if (SIRelObject.ObjectNr != CurObjId) {
            SIRelObject.RelPickupDist = OBJ_LONG_DISPLACEMENT(CurObjId);
        }

        SIRelObject.ObjectNr = CurObjId;

        SISeReObPutRelTrckObjNumber(CurObjId);

        /* Project specific */
    } else {
        /* Initialisierung des relevanten Objekts */
        SIReSiDaRelObjInit();
        SISeReObPutRelTrckObjNumber(OBJ_INDEX_NO_OBJECT);
    }

    SIRelObject.LossReason = SIObOOIGetOOILossReason(OBJ_NEXT_OOI);

    /*! Fill VLCAccOOIData*/
    /*! Reset all stuctures */

    /* OBJ_NEXT_OOI */
    /*! Fill lost reason even if no OOI0*/
    GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLong.eRelObjLossReason =
        SIObOOIGetOOILossReason(OBJ_NEXT_OOI);
    if (OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI) > OBJ_INDEX_NO_OBJECT) {
        SIFillOOIKinematic(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI),
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLong.Kinematic);
        SIFillOOIAttributes(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI), OBJ_NEXT_OOI,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLong.Attributes);
    } else {
        SIDeleteOOIData(&GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLong.Kinematic,
                        &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLong.Attributes);
    }

    /* OBJ_HIDDEN_NEXT_OOI */
    if (OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_HIDDEN_NEXT_OOI) > OBJ_INDEX_NO_OBJECT) {
        SIFillOOIKinematic(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_HIDDEN_NEXT_OOI),
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLong.Kinematic);
        SIFillOOIAttributes(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_HIDDEN_NEXT_OOI), OBJ_HIDDEN_NEXT_OOI,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLong.Attributes);
    } else {
        SIDeleteOOIData(
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLong.Kinematic,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLong.Attributes);
    }

    /* OBJ_NEXT_LONG_LEFT_OOI */
    if (OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_LEFT_OOI) >
        OBJ_INDEX_NO_OBJECT) {
        SIFillOOIKinematic(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_LEFT_OOI),
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLeft.Kinematic);
        SIFillOOIAttributes(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_LEFT_OOI),
            OBJ_NEXT_LONG_LEFT_OOI,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLeft.Attributes);
    } else {
        SIDeleteOOIData(&GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLeft.Kinematic,
                        &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextLeft.Attributes);
    }

    /* OBJ_NEXT_LONG_RIGHT_OOI */
    if (OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_RIGHT_OOI) >
        OBJ_INDEX_NO_OBJECT) {
        SIFillOOIKinematic(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_RIGHT_OOI),
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextRight.Kinematic);
        SIFillOOIAttributes(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LONG_RIGHT_OOI),
            OBJ_NEXT_LONG_RIGHT_OOI,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextRight.Attributes);
    } else {
        SIDeleteOOIData(&GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextRight.Kinematic,
                        &GET_VLC_AVLC_OOI_DATA_PTR->AccOOINextRight.Attributes);
    }

    /* OBJ_NEXT_LAT_LEFT_OOI */
    if (OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LAT_LEFT_OOI) > OBJ_INDEX_NO_OBJECT) {
        SIFillOOIKinematic(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LAT_LEFT_OOI),
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLeft.Kinematic);
        SIFillOOIAttributes(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LAT_LEFT_OOI),
            OBJ_NEXT_LAT_LEFT_OOI,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLeft.Attributes);
    } else {
        SIDeleteOOIData(
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLeft.Kinematic,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextLeft.Attributes);
    }

    /* OBJ_NEXT_LAT_RIGHT_OOI */
    if (OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LAT_RIGHT_OOI) >
        OBJ_INDEX_NO_OBJECT) {
        SIFillOOIKinematic(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LAT_RIGHT_OOI),
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextRight.Kinematic);
        SIFillOOIAttributes(
            OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_LAT_RIGHT_OOI),
            OBJ_NEXT_LAT_RIGHT_OOI,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextRight.Attributes);
    } else {
        SIDeleteOOIData(
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextRight.Kinematic,
            &GET_VLC_AVLC_OOI_DATA_PTR->AccOOIHiddenNextRight.Attributes);
    }
}

/*************************************************************************************************************************
  Functionname:    SIFillOOIKinematic */
static void SIFillOOIKinematic(const ObjNumber_t ObjId,
                               AccOOIGenKinematics_t* pKinematic) {
    /*! Fill Kinematics */
    pKinematic->fDistX = OBJ_LONG_DISPLACEMENT(ObjId);
    pKinematic->fDistY = OBJ_LAT_DISPLACEMENT(ObjId);
    pKinematic->fVrelX = OBJ_LONG_VREL(ObjId);
    pKinematic->fVrelY = OBJ_LAT_VREL(ObjId);
    pKinematic->fArelX = OBJ_LONG_AREL(ObjId);
    pKinematic->fArelY = OBJ_LAT_AREL(ObjId);
    pKinematic->fVabsX = OBJ_ABS_VELO_X(ObjId);
    pKinematic->fVabsY = OBJ_KINEMATIC(ObjId).fVabsY;
    pKinematic->fAabsX = OBJ_ABS_ACCEL_X(ObjId);
    pKinematic->fAabsY = OBJ_KINEMATIC(ObjId).fAabsY;
}

/*************************************************************************************************************************
  Functionname:    SIFillOOIAttributes */
static void SIFillOOIAttributes(const ObjNumber_t ObjId,
                                const eObjOOI_t eObjOOI,
                                AccOOIGenAttributes_t* pAttributes) {
    /*! Fill Attributes */
    if ((eObjOOI == OBJ_NEXT_OOI) || (eObjOOI == OBJ_HIDDEN_NEXT_OOI)) {
        pAttributes->uiCutInOutProbability = OBJ_GET_CUT_OUT_POTENTIAL(ObjId);
    } else {
        pAttributes->uiCutInOutProbability = OBJ_GET_CUT_IN_POTENTIAL(ObjId);
    }

    pAttributes->eDynamicProperty = OBJ_ATTRIBUTES(ObjId).eDynamicProperty;
    pAttributes->fLifeTime = OBJ_LIFETIME_SEC(ObjId);
    pAttributes->uiLifeCycles = OBJ_LIFECYCLES(ObjId);
    pAttributes->eMaintenanceState = OBJ_MAINTENANCE_STATE(ObjId);
    pAttributes->uiObjectID = ObjId;
}

/*************************************************************************************************************************
  Functionname:    SIDeleteOOIData */
void SIDeleteOOIData(AccOOIGenKinematics_t* pKinematic,
                     AccOOIGenAttributes_t* pAttributes) {
    /*! Reset Kinematics */
    pKinematic->fDistX = 0.f;
    pKinematic->fDistY = 0.f;
    pKinematic->fVrelX = 0.f;
    pKinematic->fVrelY = 0.f;
    pKinematic->fArelX = 0.f;
    pKinematic->fArelY = 0.f;
    pKinematic->fVabsX = 0.f;
    pKinematic->fVabsY = 0.f;
    pKinematic->fAabsX = 0.f;
    pKinematic->fAabsY = 0.f;

    /*! Reset Attributes */
    pAttributes->uiCutInOutProbability = 0u;
    pAttributes->eDynamicProperty = Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN;
    pAttributes->fLifeTime = 0.f;
    pAttributes->uiLifeCycles = 0u;
    pAttributes->eMaintenanceState = Envm_GEN_OBJECT_MT_STATE_DELETED;
    pAttributes->uiObjectID = OBJ_INDEX_NO_OBJECT;
    pAttributes->eUsageState = OBJ_USAGE_IDLE;
}

/*************************************************************************************************************************
  Functionname:    SIReSiDaRelObjInit */
static void SIReSiDaRelObjInit(void) {
    /*--- FUNKTIONSLOKALE SYMBOLISCHE KONSTANTEN ---*/

    /*--- FUNKTIONSLOKALE KONSTANTEN ---*/

    /*--- FUNKTIONSLOKALE VARIABLEN ---*/

    SIRelObject.ObjValid = FALSE;
    SIRelObject.StatObj = FALSE;
    SIRelObject.MovingToStat = FALSE;

    SIRelObject.DistX = 0.0;
    SIRelObject.RelSpeedX = 0.0;
    SIRelObject.RelAcclX = 0.0;

    SIRelObject.ObjSpeed = 0.0;
    SIRelObject.ObjAccl = 0.0;

    SIRelObject.ObjAngle = 0.0;
    SIRelObject.ObjDistY = 0.0;
    SIRelObject.RelSpeedY = 0.0;

    SIRelObject.LatDisplRoadBordL = 0.0;
    SIRelObject.LatDisplRoadBordR = 0.0;

    SIRelObject.RelPickupDist = 0.f;

    SIRelObject.ObjectNr = OBJ_INDEX_NO_OBJECT;

    /*--- AUFHEBUNG FUNKTIONSLOKALE SYMBOLISCHE KONSTANTEN ---*/
}

/*************************************************************************************************************************
  Functionname:    SIReSiDaInit */
void SIReSiDaInit(void) {
    /* Initialisierung des relevanten Objekts */
    SIReSiDaRelObjInit();
}

/*************************************************************************************************************************
  Functionname:    SIReSiDaGetRelevantObject */
const SIRelObject_t* SIReSiDaGetRelevantObject(void) { return &SIRelObject; }

/*************************************************************************************************************************
  Functionname:    SISeReObGetRelTrckObjNumber */
ObjNumber_t SISeReObGetRelTrckObjNumber(void) { return iTRCKGetracktesObjekt; }

/*************************************************************************************************************************
  Functionname:    SISeReObPutRelTrckObjNumber */
void SISeReObPutRelTrckObjNumber(ObjNumber_t RelTrckObjNr) {
    iTRCKGetracktesObjekt = RelTrckObjNr;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  AUFHEBUNG MODULLOKALER SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
