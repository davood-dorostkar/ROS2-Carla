/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "si.h"
#include "cp_si.h"
#include "si_par.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! Internal configuration switch to directly access ME input structure to
output ME object class based wide object output */
#define SI_CFG_USE_ME_CLASS_DIRECTLY 0

/*! Value for extension brackets that are not filled */
#ifndef INITVALUE_BRACKETPOSITION
#define INITVALUE_BRACKETPOSITION (999.f)
#endif

/*****************************************************************************
 APPLICATION PARAMETERS
*****************************************************************************/

/* Measurement message for timegap filling function
 * (SICustomControlDistanceInfo) defines */
#define SI_BMW_MIN_CONTROL_DIST 40.f   /*Minimum Control Distance*/
#define SI_BMW_MAX_CONTROL_DIST 150.0f /*Maximum Control Distance*/
#define SI_BMW_SLOPE_FACTOR 2.2f       /*Slope of Control Distance*/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

static void SICustomControlDistanceInfo(VLCCustomOutput_t* pVLCCustomOutput);

/*************************************************************************************************************************
  Functionname:    SIInitCustomerFunctions */
void SIInitCustomerFunctions(void) {
    if (VLCSEN_pCustomOutput != NULL) {
        VLCSEN_pCustomOutput->AccSelectionInfo.fLongControlDistance = 0.f;
        VLCSEN_pCustomOutput->AccSelectionInfo.fRangeFactor = 0.f;
        VLCSEN_pCustomOutput->AccSelectionInfo.fVisibilityRestriction = 0.f;

        VLCSEN_pCustomOutput->PredictedTrajectory.fCurveC0 = 0.f;
        VLCSEN_pCustomOutput->PredictedTrajectory.fCurveC1 = 0.f;
        VLCSEN_pCustomOutput->PredictedTrajectory.fAngle = 0.f;
        VLCSEN_pCustomOutput->PredictedTrajectory.ucPredictState =
            PRED_AVLC_TRAJ_EGO_ONLY;
    }
}

/*************************************************************************************************************************
  Functionname:    SICustomControlDistanceInfo */
static void SICustomControlDistanceInfo(VLCCustomOutput_t* pVLCCustomOutput) {
    float32 Control_Dist;
    const CPTrajectoryData_t* pAccTraj = SIGetTrajectoryData();

    Control_Dist = SI_BMW_SLOPE_FACTOR * EGO_SPEED_X_OBJ_SYNC;
    Control_Dist += SI_BMW_MIN_CONTROL_DIST;

    pVLCCustomOutput->AccSelectionInfo.fLongControlDistance =
        MIN_FLOAT(Control_Dist, SI_BMW_MAX_CONTROL_DIST);
    pVLCCustomOutput->AccSelectionInfo.fRangeFactor = fRangeFac;

    pVLCCustomOutput->AccSelectionInfo.fVisibilityRestriction =
        SIGetMovingObjPickupRange();

    pVLCCustomOutput->AccSelectionInfo.fSeekLaneWidth =
        SIGetBaseSeekLaneWidth();

    pVLCCustomOutput->PredictedTrajectory.fCurveC0 = pAccTraj->Current.fTrajC0;
    pVLCCustomOutput->PredictedTrajectory.fCurveC1 = pAccTraj->Current.fTrajC1;
    pVLCCustomOutput->PredictedTrajectory.fAngle = pAccTraj->Current.fTrajAngle;
    pVLCCustomOutput->PredictedTrajectory.ucPredictState =
        PRED_AVLC_TRAJ_EGO_ONLY;
    /* Decode the ego course information */
    if (!pAccTraj->State.EgoCourseOnly) {
        if (pAccTraj->State.FusionRoadstimation) {
            pVLCCustomOutput->PredictedTrajectory.ucPredictState |=
                PRED_AVLC_TRAJ_ROAD_ESTI;
        }
        if (pAccTraj->State.FusionTraces) {
            pVLCCustomOutput->PredictedTrajectory.ucPredictState |=
                PRED_AVLC_TRAJ_OBJ_TRACES;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SICustomProcess */
void SICustomProcess(void) {
    /* Base selection of the objects of interest */
    SISelectBaseObjectsOfInterest();

    /* Calculate additional object features (cut-in, cut-out....) */
    SICalcBaseFeatures();

    /*Calculate Control and Visibility Distance*/
    SICustomControlDistanceInfo(VLCSEN_pCustomOutput);
}

/*************************************************************************************************************************
  Functionname:    SIMergeCustomObjects */
void SIMergeCustomObjects(ObjNumber_t uiObjectToKeep,
                          ObjNumber_t uiObjectToDelete) {
    /* merge customer specific object data */
    // uiObjectToKeep = uiObjectToKeep;

    // uiObjectToDelete = uiObjectToDelete;
}

/*************************************************************************************************************************
  Functionname:    SICustMergePreselection */
boolean SICustMergePreselection(ObjNumber_t ObjNr,
                                ACCObjectQuality_t uiAccQual,
                                boolean bFunPresel) {
    boolean Select;

    if ((OBJ_DYNAMIC_PROPERTY(ObjNr) == CR_OBJECT_PROPERTY_STATIONARY) &&
        (!OBJ_IS_MOVING_TO_STATIONARY(ObjNr))) {
        uiAccQual = 0u;
    }
    //#ifdef _MSC_VER
    /* Keep MS-VC happy, even though currently this parameter is not used */
    //  ObjNr = ObjNr;
    //#endif
    if ((uiAccQual >= FUN_PRESEL_AVLC_MIN_OBJ_QUAL) && (bFunPresel)) {
        Select = TRUE;
    } else {
        Select = FALSE;
    }
    return Select;
}

/*************************************************************************************************************************
  Functionname:    SICheckCustomInlaneCriteria */
boolean SICheckCustomInlaneCriteria(ObjNumber_t iObjNr,
                                    const CPTrajOccupancy_t* pOccupancy) {
    boolean retval;

    /* Predicted Lane Association not for stationary objects or occluded objects
       or objects occluded by trace analysis. Consider occlusion only if the
       object can become the OOI-0 based in the longitudinal distance in order
       to reduce the influence on the OOI-1. */
    if ((OBJ_DYNAMIC_PROPERTY(iObjNr) != CR_OBJECT_PROPERTY_STATIONARY) &&
        ((OBJ_ATTRIBUTES(iObjNr).eObjectOcclusion <
          Envm_GEN_OBJECT_OCCL_PARTLY) &&
         (OBJ_GET_SI(iObjNr).Bool.OccludedByTrace == 0u) &&
         ((OBJ_GET_RELEVANT_OBJ_NR == OBJ_INDEX_NO_OBJECT) ||
          ((OBJ_GET_RELEVANT_OBJ_NR != OBJ_INDEX_NO_OBJECT) &&
           (OBJ_LONG_DISPLACEMENT(OBJ_GET_RELEVANT_OBJ_NR) +
                OT_GET_OBJ_LENGTH(OBJ_GET_RELEVANT_OBJ_NR) >
            OBJ_LONG_DISPLACEMENT(iObjNr)))))) {
        retval = SICheckPredictedInlaneCriteria(iObjNr, pOccupancy);
    } else {
        retval = FALSE;
    }

    return (retval);
}

/*************************************************************************************************************************
  Functionname:    SICheckCustomOutlaneCriteria */
boolean SICheckCustomOutlaneCriteria(ObjNumber_t iObjNr,
                                     const CPTrajOccupancy_t* pOccupancy) {
    boolean retval;

    /* Predicted Lane Association not for stationary objects */
    if (OBJ_DYNAMIC_PROPERTY(iObjNr) != CR_OBJECT_PROPERTY_STATIONARY) {
        retval = SICheckPredictedOutlaneCriteria(iObjNr, pOccupancy);
    } else {
        retval = FALSE;
    }

    return (retval);
}

/*************************************************************************************************************************
  Functionname:    SICustomCorridorPreProcessing */
void SICustomCorridorPreProcessing(ObjNumber_t iObj,
                                   SIBracketFuncEnable_t* pBracketFuncEnable) {
    /* Check if course prediction overtake assist indicates that this object is
    being overtaken. If yes, reset all trace bracket extensions for it */
    if (CPSICheckObjForOvertake(iObj)) {
        /* Restrict base corridor width to the seeklane-width only. The
       seeklane-width is smaller compared to the tracklane-width and thus,
       objects can be released earlier in e.g. overtake-scenarios. */
        pBracketFuncEnable->bUseSeekLaneWidthOnly = TRUE;

        pBracketFuncEnable->bEnableAddRestrictionCurveOuterBorder =
            SI_CORR_BRACKET_FUNC_DISABLED;
        pBracketFuncEnable->bEnableExtensionRoadBorder =
            SI_CORR_BRACKET_FUNC_DISABLED;
        pBracketFuncEnable->bEnableAddExtensionRelevantObject =
            SI_CORR_BRACKET_FUNC_DISABLED;
        pBracketFuncEnable->bEnableAddExtensionHighTunnelProb =
            SI_CORR_BRACKET_FUNC_DISABLED;
    }
}

/*************************************************************************************************************************
  Functionname:    SICustFillMeasOOI */
void SICustFillMeasOOI(ObjNumber_t ObjId, SICustMeasOOI_t* pDestMeasData) {
    if (ObjId > OBJ_INDEX_NO_OBJECT) {
        pDestMeasData->fAbsSpeedX = OBJ_ABS_VELO_X(ObjId);
        pDestMeasData->fAbsAccelX = OBJ_ABS_ACCEL_X(ObjId);
        /* Calculate simple TTC (without relative acceleration) for given object
         * when possible */
        if (OBJ_LONG_VREL(ObjId) < -C_F32_DELTA) {
            pDestMeasData->fTTC =
                (OBJ_LONG_DISPLACEMENT(ObjId) / -OBJ_LONG_VREL(ObjId));
        } else {
            pDestMeasData->fTTC = 0.f;
        }
        /* Calculate ego-speed based time-gap time in seconds for given object
         * when possible */
        if (EGO_SPEED_X_OBJ_SYNC > C_F32_DELTA) {
            pDestMeasData->fTimeGap =
                (OBJ_LONG_DISPLACEMENT(ObjId) / EGO_SPEED_X_OBJ_SYNC);
        } else {
            pDestMeasData->fTimeGap = 0.f;
        }
    } else {
        pDestMeasData->fAbsSpeedX = 0.f;
        pDestMeasData->fAbsAccelX = 0.f;
        pDestMeasData->fTTC = 0.f;
        pDestMeasData->fTimeGap = 0.f;
    }
}

/*************************************************************************************************************************
  Functionname:    SI_GetPickupDistSamplePoints */
const GDBVector2_t* SI_GetPickupDistSamplePoints(uint8* pui_NumPointsCurve) {
    const GDBVector2_t* p_SamplePointsDefault =
        SI_a_SamplePointDistancePara; /*!< Get the default sample points */

    /* Depending on Eco-mode use different ramps:
      -> return custom i/o specific sample points in case of Eco-mode, return
      the default sample points otherwise */
    /* Depending on the switch in si_cfg.h */
    {
        *pui_NumPointsCurve = SI_NUM_PICKUP_DIST_POINTS;
        return p_SamplePointsDefault; /* return default sample points */
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */