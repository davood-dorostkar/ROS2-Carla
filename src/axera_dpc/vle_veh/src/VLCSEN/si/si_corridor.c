/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "si.h"
#include "si_par.h"
// #include "vlc_par.h"
#include "fip_ext.h"
/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#include "stddef.h"
#include "TM_Global_Types.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! Definition of the close Range [m] for special handling of stationary Objects
 */
#define SI_STAT_CLOSE_RANGE (10.0F)

/* Constants for reflection point correction */
#define REF_CORR_FAC_MIN (0.0F)
#define REF_CORR_FAC_MAX (1.0F)
#define REF_CORR_MIN_ANGLE (7.0F)
#define REF_CORR_MAX_ANGLE (20.0F)

#define REF_CORR_MIN_LIFETIME (15.F)
#define REF_CORR_MAX_LIFETIME (20.F)

/* Filter constant used for lowpass filtering camera based lane width */
#define CAM_LANE_FILTER_CONST (0.5f / SI_CYCLE_TIME)
/* Possible divide-by-0 condition because of SI_CYCLE_TIME: May have to be
 * handled */

// const volatile float DIST_FAC_MIN = 10;
// const volatile float DIST_FAC_MAX = 100;
// const volatile float EXP_DIST_FAC_MIN = 0.5;
// const volatile float EXP_DIST_FAC_MAX = 1;

#define DIST_FAC_MIN 20.F
#define DIST_FAC_MAX 50.F
#define EXP_DIST_FAC_MIN 0.5F
#define EXP_DIST_FAC_MAX 1.0F

const volatile float32 CAL_OBJ_WIDTH_FAC = 1.5f;
#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*! Modes of trace bracket manipulation (used by SIRelTraSetLeftBracket and
SIRelTraSetRightBracket functions) */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
typedef enum {
    DEFAULT_MODE,     /*!< Default mode: set to given value */
    RESTRICTION_MODE, /*!< Trace bracket restrction mode: default init to left:
                         -max right: +max,
                           when updating, take smaller of bracket values
                         (restrict) */
    EXTENSION_MODE /*!< Trace bracket extension mode: default init to left: +max
                      right: -max,
                         when updating, take greater of bracket values (extend
                      existing) */
} SITraceBracketMode_t;

SET_MEMSEC_CONST(TRCK_Seek_ExpandFac_DistAdapted)
static const GDBLFunction_t TRCK_Seek_ExpandFac_DistAdapted = {

    EXP_DIST_FAC_MIN, /*!< A1 */
    EXP_DIST_FAC_MAX, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */

    (EXP_DIST_FAC_MAX - EXP_DIST_FAC_MIN) / (DIST_FAC_MAX - DIST_FAC_MIN),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    EXP_DIST_FAC_MIN - (((EXP_DIST_FAC_MAX - EXP_DIST_FAC_MIN) /
                         (DIST_FAC_MAX - DIST_FAC_MIN)) *
                        DIST_FAC_MIN)};

/* ramp for correcting the object reference point to its center at field of
 * views edge */
SET_MEMSEC_CONST(ReflectionCorrectionAngleFactor)
static const GDBLFunction_t ReflectionCorrectionAngleFactor = {

    REF_CORR_FAC_MIN, /*!< A1 */
    REF_CORR_FAC_MAX, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */

    (REF_CORR_FAC_MAX - REF_CORR_FAC_MIN) /
        (REF_CORR_MAX_ANGLE - REF_CORR_MIN_ANGLE),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */

    REF_CORR_FAC_MIN - (((REF_CORR_FAC_MAX - REF_CORR_FAC_MIN) /
                         (REF_CORR_MAX_ANGLE - REF_CORR_MIN_ANGLE)) *
                        REF_CORR_MIN_ANGLE)};

/* ramp for correcting the object reference point to its center at field of
 * views edge dependent on LifeTime*/
SET_MEMSEC_CONST(ReflectionCorrectionLifeFactor)
static const GDBLFunction_t ReflectionCorrectionLifeFactor = {

    REF_CORR_FAC_MIN, /*!< A1 */
    REF_CORR_FAC_MAX, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */
    (REF_CORR_FAC_MIN - REF_CORR_FAC_MAX) /
        (REF_CORR_MAX_LIFETIME - REF_CORR_MIN_LIFETIME),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    REF_CORR_FAC_MAX - (((REF_CORR_FAC_MIN - REF_CORR_FAC_MAX) /
                         (REF_CORR_MAX_LIFETIME - REF_CORR_MIN_LIFETIME)) *
                        REF_CORR_MIN_ANGLE)
    /*!< Remark: Is REF_CORR_MIN_LIFETIME instead of REF_CORR_MIN_ANGLE
       correct? For keeping the same performance, this parameter is not
       changed */
};

SET_MEMSEC_CONST(TRCK_Seek_LaneWidth_SpeedAdapted)
static const GDBLFunction_t TRCK_Seek_LaneWidth_SpeedAdapted = {
    SI_HIGHWAYLANEWIDTHSEEK_MIN, /*!< A1 */
    SI_HIGHWAYLANEWIDTHSEEK_MAX, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */
    (SI_HIGHWAYLANEWIDTHSEEK_MAX - SI_HIGHWAYLANEWIDTHSEEK_MIN) /
        (SI_MAX_SPEED_HIGWAYLANEWIDTH - SI_MIN_SPEED_HIGWAYLANEWIDTH),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    SI_HIGHWAYLANEWIDTHSEEK_MIN -
        (((SI_HIGHWAYLANEWIDTHSEEK_MAX - SI_HIGHWAYLANEWIDTHSEEK_MIN) /
          (SI_MAX_SPEED_HIGWAYLANEWIDTH - SI_MIN_SPEED_HIGWAYLANEWIDTH)) *
         SI_MIN_SPEED_HIGWAYLANEWIDTH)};

SET_MEMSEC_CONST(TRCK_Track_Offset_SpeedAdapted)
static const GDBLFunction_t TRCK_Track_Offset_SpeedAdapted = {

    SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MAX, /*!< A1 */
    SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MIN, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */
    (SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MIN -
     SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MAX) /
        (SI_MAX_SPEED_HIGWAYLANEWIDTH - SI_MIN_SPEED_HIGWAYLANEWIDTH),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MAX -
        (((SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MIN -
           SI_HIGHWAYLANEWIDTH_TRCK_OFFSET_MAX) /
          (SI_MAX_SPEED_HIGWAYLANEWIDTH - SI_MIN_SPEED_HIGWAYLANEWIDTH)) *
         SI_MIN_SPEED_HIGWAYLANEWIDTH)};

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*! Global seek lane width variable. Cached here to avoid recalculating linear
interpolation ramp over and over again */
SET_MEMSEC_VAR(fSISeekLaneWidth)
static float32 fSISeekLaneWidth;
/*! Global track lane width variable. Cached here to avoid recalculation */
SET_MEMSEC_VAR(fSITrackLaneWidth)
static float32 fSITrackLaneWidth;

/*! Check how many poles are configred for stationary object course filtering.
Note: check for greater one, since one pole would mean identity transform,
that is no filtering */
/*! Local low-pass filtered trajectory for stationary objects */
SET_MEMSEC_VAR(SIStatLowPassTraj)
static struct SIStatLowPassTraj {
    float32 afCrvSamples[SI_PAR_STAT_COURSE_FIR_POLES];
    float32 fSumCrv;
    uint8 uNextIdx;
    GDBTrajectoryData_t Traj;
} SIStatLowPassTraj;

/*****************************************************************************
  FUNCTION
*****************************************************************************/
static void SICalculateBaseLaneWidths(void);
static void SIConstructCorridorEnvironment(
    AssTraEnvironment_t* const pEnvironment);
static void SIConstructDefaultTrajectory(RelTraCurve_t* const pRelTraCurve);
static void SIConstructAlternativeTrajectory(RelTraCurve_t* const pRelTraCurve);
static void SIUpdateRelevanceForAllObjects(void);
static void SIAdjustObjectPropertiesToTrajectory(
    RelTraObjInput_t* const pObjectProperties,
    const RelTraCurve_t* const pTrajectory,
    const CPTrajectoryData_t* const pTrajectoryData);
static void SIConstructObjectVariables(const ObjNumber_t ObjNr,
                                       RelTraObjOutput_t* const pRelTraObj);
static void SIConstructObjectProperties(const ObjNumber_t ObjNr,
                                        RelTraObjInput_t* const pRelTraObj);
static void SIInitCriteriaMatrix(SICriteriaMatrix_t* const pCriteriaMatrix);
static void SIRelTraInitCriteriaOutput(SIBracketOutput_t* const pOutput,
                                       const SITraceBracketMode_t eMode);
static SIBracketOutput_t SICalculateBracketsForCurrentObject(
    const ObjNumber_t ObjNr,
    RelTraObjInput_t* const pObjectProperties,
    const RelTraCurve_t* const pTrajectory,
    AssTraEnvironment_t* const pEnvironment,
    const SICriteriaMatrixAllObj_t* const pCritMatrixAllObj);
static void SIRecalculateBracketsUsingAlternativeTrajectory(
    const ObjNumber_t ObjNr,
    RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    AssTraEnvironment_t* const pEnvironment,
    SICriteriaMatrix_t* const pBracketFuncResults,
    SIBracketFuncEnable_t* const pBracketFuncEnableFlags,
    const SICriteriaMatrixAllObj_t* const pCritMatrixAllObj);
static CPDistanceWidth_t SIGenerateCorridorDescriptionFromBrackets(
    const ObjNumber_t ObjNr,
    const SIBracketOutput_t* const pBracketPositions,
    const RelTraObjInput_t* const pObjectProperties);
static void SIPerformReflectionCorrection(
    const ObjNumber_t ObjNr, CPDistanceWidth_t* const pCorridorDescription);

static void SIInitCorridorCritModule(void);
static void SIEnableAllBracketFunctions(
    SIBracketFuncEnable_t* const pBracketFuncEnable);
static float32 SICalcRelTraTrackwidthSeek(void);
static float32 SICalcRelTraTrackwidthTrack(void);
static void SIManipulateLaneClassBasedOnCam(
    FIP_t_LaneWidthClass* const pLaneWidthClass);

static float32 SIGetObjWidthVarForCorridor(const ObjNumber_t iObjNr);
static float32 SIGetReflectionOffset(const ObjNumber_t iObjNr);
static void SICalcStatLowPassTraj(void);
static void SIUpdateRelevancyTimers(const ObjNumber_t ObjNr,
                                    SIObjCorridor_t* const pObjCor);
static void SIUpdateTimeTrackLaneExtFactor(const ObjNumber_t ObjNr,
                                           SIObjCorridor_t* const pObjCor);
static void SIUpdateDistTrackLaneExtFactor(const ObjNumber_t ObjNr,
                                           SIObjCorridor_t* const pObjCor);

static void SIRelTraObjOutputToObject(const RelTraObjOutput_t* const pRelTraObj,
                                      SIObjCorridor_t* const pObjCor);

/*************************************************************************************************************************
  Functionname:    SICorridorInit */
void SICorridorInit(void) {
    uint8 i;

    /* Call new function of si_corridor_crit.c to initialize that module's
     * statics */
    SIInitCorridorCritModule();

    for (i = 0u; i < SI_PAR_STAT_COURSE_FIR_POLES; i++) {
        SIStatLowPassTraj.afCrvSamples[i] = 0.f;
    }
    SIStatLowPassTraj.fSumCrv = 0.f;
    SIStatLowPassTraj.uNextIdx = 0u;
    SIStatLowPassTraj.Traj.fTrajC0 = 0.f;
    SIStatLowPassTraj.Traj.fTrajC1 = 0.f;
    SIStatLowPassTraj.Traj.fTrajAngle = 0.f;
}

/*************************************************************************************************************************
  Functionname:    SI_Calculate_AVLC_Corridor */
void SI_Calculate_AVLC_Corridor(CPDistanceWidth_t pDistWidth[]) {
    ObjNumber_t ObjNr;
    AssTraEnvironment_t Environment;
    RelTraCurve_t Trajectory;

    /* Criteria Matrix: values which are the same for all objects */
    SICriteriaMatrixAllObj_t CritMatrixAllObj;

    /* Calculate two types of lane widths on which the corridor calculation is
       going to be based.
       A wider corridor is used for objects which are being tracked, while a
       more narrow one is considered for non-tracked objects. */
    SICalculateBaseLaneWidths();

    /* Initialization of the environment variable */
    SIConstructCorridorEnvironment(&Environment);

    /* Save parameters of ACC trajectority (includes fusion of VDY and ROAD) */
    SIConstructDefaultTrajectory(&Trajectory);

    /* Initialization of Criteria Matrix which is equal for all objects as well
       as
        activation / deactivation of corridor bracket functions*/
    SIInitCriteriaMatrixAllObj(&CritMatrixAllObj);

    /* Determine Criteria Matrix which is equal for all objects */
    SIRelTraCheckCriteriaAllObj(&CritMatrixAllObj);

    /* Update for all objects relevant/not relevant time, lane extension factor,
     * and distance extension factor */
    SIUpdateRelevanceForAllObjects();

    /* For each object a distinct description of the corridor is calculated.
       The description consists of the corridor width, the object's width and
       the distance from the object to the center of the corridor. */
    for (ObjNr = (ObjNumber_t)(Envm_N_OBJECTS - 1); ObjNr >= 0; ObjNr--) {
        SIBracketOutput_t BracketPositions;
        CPDistanceWidth_t CorridorDescription;
        RelTraObjInput_t ObjectProperties;

        /* Collect properties relating to the current object and combine them in
           a single data structure.
           The fDistToCourse property depends on the trajectory, therefore it
           is set separately. */
        SIConstructObjectProperties(ObjNr, &ObjectProperties);

        /* Lateral component of the point on the trajectory with the shortest
         * distance to the object */
        SIAdjustObjectPropertiesToTrajectory(&ObjectProperties, &Trajectory,
                                             &SITrajectoryData);

        /* For the current object calculate the left and right bracket position,
           which mark the boundaries of the corridor.
           Using the base lane widths the brackets are positioned and then
           further extended/restricted based on whether specific case scenarios
           are detected. */
        BracketPositions = SICalculateBracketsForCurrentObject(
            ObjNr, &ObjectProperties, &Trajectory, &Environment,
            &CritMatrixAllObj);

        /* Use the bracket positions calculated for the current object to
           generate a corresponding corridor description.
           This description consists of the corridor width, the object's width
           and the distance from the object to the center of the corridor. */
        CorridorDescription = SIGenerateCorridorDescriptionFromBrackets(
            ObjNr, &BracketPositions, &ObjectProperties);

        /* Reflection correction */
        SIPerformReflectionCorrection(ObjNr, &CorridorDescription);

        /* Store the final corridor description for the current object */
        pDistWidth[ObjNr] = CorridorDescription;
    }
}

float32 SIGetObjCorridorWidth(const RelTraObjInput_t* const pObjectProperties) {
    float fDistToCourse_Min = pObjectProperties->CornerPoint[0].fDistToCourse;
    float fDistToCourse_Max = pObjectProperties->CornerPoint[0].fDistToCourse;
    for (uint8 i = 1; i < 4; i++) {
        if (pObjectProperties->CornerPoint[i].fDistToCourse <
            fDistToCourse_Min) {
            fDistToCourse_Min = pObjectProperties->CornerPoint[i].fDistToCourse;
        }

        if (pObjectProperties->CornerPoint[i].fDistToCourse >
            fDistToCourse_Max) {
            fDistToCourse_Max = pObjectProperties->CornerPoint[i].fDistToCourse;
        }
    }

    return fDistToCourse_Max - fDistToCourse_Min;
}

/*************************************************************************************************************************
  Functionname:    SIGetObjWidthForCorridor */
float32 SIGetObjWidthForCorridor(const ObjNumber_t iObj) {
    float32 fObjectWidth;
    const Envm_t_CR_Classification eObjClass = OBJ_CLASSIFICATION(iObj);

    if (eObjClass == CR_OBJCLASS_CAR) {
        if ((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) &&
            (OBJ_IS_MOVING_TO_STATIONARY(iObj) == FALSE)) {
            /*! stationary (not stopped) objects get a variable width */
            fObjectWidth =
                MINMAX_FLOAT(SI_MIN_OBJ_WIDTH_CAR_STAT,
                             SI_MAX_OBJ_WIDTH_CAR_STAT, OT_GET_OBJ_WIDTH(iObj));
        } else {
            /*! moving/stopped object seems to be a car, so calculate an object
             * width of 1.8m */
            fObjectWidth = SI_OBJ_WIDTH_CAR_MOVE;
        }
    } else if (eObjClass == CR_OBJCLASS_TRUCK) {
        /*! object seems to be a truck, so calculate an object width of 2.5 */
        fObjectWidth = SI_OBJ_WIDTH_TRUCK;
    } else if ((eObjClass == CR_OBJCLASS_POINT) ||
               (eObjClass == CR_OBJCLASS_MOTORCYCLE) ||
               (eObjClass == CR_OBJCLASS_PEDESTRIAN)) {
        fObjectWidth = SI_OBJ_WIDTH_SMALL_OBJ;
    } else {
        if (OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) {
            fObjectWidth = MINMAX_FLOAT(SI_MIN_OBJ_WIDTH_OTHER_CLASS_STAT,
                                        SI_MAX_OBJ_WIDTH_OTHER_CLASS_STAT,
                                        OT_GET_OBJ_WIDTH(iObj));
        } else {
            fObjectWidth = SI_OBJ_WIDTH_MOVE;
            /*!<  OBJCLASS_UNDEFINED   = 0U,
                  CR_OBJCLASS_WIDE    = 5U,
                  OBJCLASS_BRIDGE      = 7U */
        }
    }

    if (OBJ_LONG_DISPLACEMENT(iObj) > SI_AVLC_MIN_DIST_OBJ_WIDTH_DIST_ADAPTED) {
        fObjectWidth = MAX_FLOAT(SI_AVLC_MIN_OBJ_WIDTH_FAR_DIST, fObjectWidth);
    }

    /* Limit the Width of the stationary Objects that is used to calculate the
     * Overlaps */
    if ((OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_STATIONARY) &&
        (!OBJ_IS_MOVING_TO_STATIONARY(iObj))) {
        fObjectWidth = MINMAX_FLOAT(SI_MIN_OBJ_WIDTH_STAT,
                                    SI_MAX_OBJ_WIDTH_STAT, fObjectWidth);
    }

    return fObjectWidth;
}

/*************************************************************************************************************************
  Functionname:    SIGetObjWidthVarForCorridor */
static float32 SIGetObjWidthVarForCorridor(const ObjNumber_t iObjNr) {
    float32 fVar;

    if ((OBJ_DYNAMIC_PROPERTY(iObjNr) == CR_OBJECT_PROPERTY_STATIONARY) &&
        (!OBJ_IS_MOVING_TO_STATIONARY(iObjNr)) &&
        (OBJ_LONG_DISPLACEMENT(iObjNr) > SI_STAT_CLOSE_RANGE)) {
        fVar = 0.f;
    } else {
        fVar = MINMAX(
            SI_OBJ_WIDTH_VAR_MOVE_NEAR, SI_OBJ_WIDTH_VAR_MOVE_FAR,
            dGDBmathGewichtGerade(SI_OBJ_NEAR_RANGE, SI_OBJ_WIDTH_VAR_MOVE_NEAR,
                                  SI_OBJ_FAR_RANGE, SI_OBJ_WIDTH_VAR_MOVE_FAR,
                                  ABS(OBJ_LONG_DISPLACEMENT(iObjNr))));
    }

    return (fVar * SI_OBJ_WIDTH_VAR_FACTOR);
}

/*************************************************************************************************************************
  Functionname:    SIGetReflectionOffset */
static float32 SIGetReflectionOffset(const ObjNumber_t iObjNr) {
    const float32 fAngleCorrFactor = dGDBmathLineareFunktion(
        &ReflectionCorrectionAngleFactor, fABS(OBJ_ANGLE(iObjNr)));
    const float32 fLifeTimeCorrFactor = dGDBmathLineareFunktion(
        &ReflectionCorrectionLifeFactor, (float32)(OBJ_LIFECYCLES(iObjNr)));
    float32 fDistReflectionOffset;

    fDistReflectionOffset = 0.5F * SIGetObjWidthForCorridor(iObjNr) *
                            fAngleCorrFactor * fLifeTimeCorrFactor;
    return fDistReflectionOffset;
}

/*************************************************************************************************************************
  Functionname:    SICalcStatLowPassTraj */
static void SICalcStatLowPassTraj(void) {
    /* Load current ego curvature */
    const float32 fCurCrv = EGO_CURVE_RAW;
    /* Read old curvature from buffer (the value we are about to overwrite) */
    const float32 fOldCrv =
        SIStatLowPassTraj.afCrvSamples[SIStatLowPassTraj.uNextIdx];

    /* Write the new curvature value into the buffer */
    SIStatLowPassTraj.afCrvSamples[SIStatLowPassTraj.uNextIdx] = fCurCrv;
    /* Update the next index in the buffer to write to */
    SIStatLowPassTraj.uNextIdx = ((SIStatLowPassTraj.uNextIdx + 1u) %
                                  (uint8)SI_PAR_STAT_COURSE_FIR_POLES);
    /* Update summ of values in buffer */
    SIStatLowPassTraj.fSumCrv += (fCurCrv - fOldCrv);
    /* And write output data struct (simple curve with no angle and change rate)
     */
    SIStatLowPassTraj.Traj.fTrajAngle = 0.f;
    SIStatLowPassTraj.Traj.fTrajC0 =
        SIStatLowPassTraj.fSumCrv *
        (1.f / (float32)SI_PAR_STAT_COURSE_FIR_POLES);
    SIStatLowPassTraj.Traj.fTrajC1 = 0.f;
}

/*************************************************************************************************************************
  Functionname:    SIConstructCorridorEnvironment */
static void SIConstructCorridorEnvironment(
    AssTraEnvironment_t* const pEnvironment) {
    ObjNumber_t RelTrckObjNr;

    pEnvironment->iNumberLanesLeft = 1;
    pEnvironment->iNumberLanesRight = 1;
    pEnvironment->dNoLaneProbL = 0.5f;
    pEnvironment->dNoLaneProbR = 0.5f;
    pEnvironment->dEOBorderLeft = -1.f;
    pEnvironment->dEOBorderRight = -1.f;

    RelTrckObjNr = SISeReObGetRelTrckObjNumber();
    if (RelTrckObjNr != OBJ_INDEX_NO_OBJECT) {
        pEnvironment->iRelObjNr = RelTrckObjNr;
        pEnvironment->ucRelObjDynamicProperty =
            OBJ_DYNAMIC_PROPERTY(RelTrckObjNr);
        pEnvironment->fRelObjDistX = OBJ_LONG_DISPLACEMENT(RelTrckObjNr);
        pEnvironment->fRelObjDistY = OBJ_LAT_DISPLACEMENT(RelTrckObjNr);
        pEnvironment->fRelObjTargetFusionHoldTime =
            OBJ_GET_SI(RelTrckObjNr).ObjCor.fTargetFusionHoldTime;
    } else {
        pEnvironment->iRelObjNr = OBJ_INDEX_NO_OBJECT;
        pEnvironment->ucRelObjDynamicProperty = CR_OBJECT_PROPERTY_MOVING;
        pEnvironment->fRelObjDistX = 0.0f;
        pEnvironment->fRelObjDistY = 0.0f;
        pEnvironment->fRelObjTargetFusionHoldTime = 0.0f;
    }
}

/*************************************************************************************************************************
  Functionname:    SIConstructDefaultTrajectory */
static void SIConstructDefaultTrajectory(RelTraCurve_t* const pRelTraCurve) {
    /* Setup Curve struct to SI fusion course (EGO + ROAD + ..) */
    pRelTraCurve->dCurve = SITrajectoryData.Current.fTrajC0;
    pRelTraCurve->dCurve_abs = fABS(SITrajectoryData.Current.fTrajC0);
    pRelTraCurve->iOWVflag = OWVKRIT_AKTIV;
}

/*************************************************************************************************************************
  Functionname:    SIUpdateRelevanceForAllObjects */
static void SIUpdateRelevanceForAllObjects(void) {
    ObjNumber_t iObj;
    SIObjCorridor_t* pObjCor;

    for (iObj = (ObjNumber_t)(Envm_N_OBJECTS - 1); iObj >= 0; iObj--) {
        if (!OBJ_IS_DELETED(iObj)) {
            /* Update relevancy timers (relevant/not relevant time counters) */
            pObjCor = &OBJ_GET_SI(iObj).ObjCor;

            SIUpdateRelevancyTimers(iObj, pObjCor);

            SIUpdateTimeTrackLaneExtFactor(iObj, pObjCor);

            SIUpdateDistTrackLaneExtFactor(iObj, pObjCor);
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIConstructObjectProperties */
static void SIConstructObjectProperties(const ObjNumber_t ObjNr,
                                        RelTraObjInput_t* const pRelTraObj) {
    /* pObjCor : pointer to an SIObjCorridor_t type structure with
                    relevance time and trace bracket extension factor
       information */
    const SIObjCorridor_t* pObjCor = &OBJ_GET_SI(ObjNr).ObjCor;
    const TraceID_t iObjTrace = OBJ_GET_STATIC_TRACE_ID(ObjNr);
    /*TraceID_t iTracingID;*/
    pRelTraObj->iTracingID = iObjTrace;

    /*ObjNumber_t iObjNr;*/
    pRelTraObj->iObjNr = ObjNr;
    /*uint8 ucDynamicProperty;*/
    pRelTraObj->ucDynamicProperty = OBJ_DYNAMIC_PROPERTY(ObjNr);
    /*uint8 uiStoppedConfidence;*/
    pRelTraObj->uiStoppedConfidence = OBJ_MOVING_TO_STATIONARY(ObjNr);

    /* Check if link obj<->trace is valid */
    if (iObjTrace < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES) {
        const FIP_MOVING_OBJ_STATIC_TRACES_TYPE* const pTrace =
            FIP_GET_MOVING_OBJ_STATIC_TRACE_PTR(iObjTrace);
        pRelTraObj->iTraceReachedEgoVeh = pTrace->Legacy.TraceReachEgoVeh;
        pRelTraObj->dYIntersec = FIP_STATIC_TRACE_GET_Y_INTERSEC(iObjTrace);
        pRelTraObj->dYIntersecGradFilt =
            FIP_STATIC_TRACE_GET_Y_INTERSEC_GRAD_FILT(iObjTrace);
    } else {
        pRelTraObj->iTraceReachedEgoVeh = FALSE;
        pRelTraObj->dYIntersec = 0.0F;
        pRelTraObj->dYIntersecGradFilt = 0.0F;
    }

    /* ubit8_t iRelevant: 1 -> Bit field of size 1 */
    if (OBJ_GET_RELEVANT(ObjNr)) {
        pRelTraObj->iRelevant = TRUE;
    } else {
        pRelTraObj->iRelevant = FALSE;
    }

    /*AlgoCycleCounter_t iObjectLifeCycles;*/
    pRelTraObj->iObjectLifeCycles = OBJ_LIFECYCLES(ObjNr);

    pRelTraObj->fDistYStdDev = OBJ_LAT_DISPLACEMENT_STD(ObjNr);
    pRelTraObj->fDistY = OBJ_LAT_DISPLACEMENT(ObjNr);
    pRelTraObj->fDistX = OBJ_LONG_DISPLACEMENT(ObjNr);
    pRelTraObj->fVRelY = OBJ_LONG_VREL(ObjNr);
    pRelTraObj->fOrientation = OBJ_ANGLE(ObjNr);
    pRelTraObj->fVelY = GET_Envm_GEN_OBJ(ObjNr).Kinematic.fVabsY;
    pRelTraObj->fLatTrackLimitR = pObjCor->TrackVehicle.fLatTrackLimitR;
    pRelTraObj->fLatTrackLimitL = pObjCor->TrackVehicle.fLatTrackLimitL;
    pRelTraObj->fRefCourseDistY = 0.f;
    pRelTraObj->fRelevantTime = pObjCor->fRelevantTime;
    pRelTraObj->fNotRelevantTime = pObjCor->fNotRelevantTime;
    pRelTraObj->fLatTrackLimitExpandFac =
        pObjCor->TrackVehicle.fLatTrackLimitExpandFac;
    pRelTraObj->fLatTrackLimitDistanceExpandFac =
        pObjCor->TrackVehicle.fLatTrackLimitDistanceExpandFac;

    for (uint8 i = 0; i < 4; i++) {
        pRelTraObj->CornerPoint[i].fDistX =
            GET_Envm_GEN_OBJ(ObjNr).Geometry.aShapePointCoordinates[i].fPosX;
        pRelTraObj->CornerPoint[i].fDistY =
            GET_Envm_GEN_OBJ(ObjNr).Geometry.aShapePointCoordinates[i].fPosY;
        pRelTraObj->CornerPoint[i].fDistOnCourse = 0;
        pRelTraObj->CornerPoint[i].fDistToCourse = 0;
        pRelTraObj->CornerPoint[i].fRefCourseDistX = 0;
        pRelTraObj->CornerPoint[i].fRefCourseDistY = 0;
    }
}

/*************************************************************************************************************************
  Functionname:    SIGenerateCorridorDescriptionFromBrackets */
static CPDistanceWidth_t SIGenerateCorridorDescriptionFromBrackets(
    const ObjNumber_t ObjNr,
    const SIBracketOutput_t* const pBracketPositions,
    const RelTraObjInput_t* const pObjectProperties) {
    CPDistanceWidth_t Result;
    float32 CorridorWidth;
    float32 CorridorCenter;
    float32 ObjToRefDist;
    float32 ObjToRefDistVar;
    float32 fDistExtensionOffset;

    /* Get estimated Object width and associated Variance */
    Result.fObjectWidth =
        SIGetObjWidthForCorridor(ObjNr); /* Replaces: OT_GET_OBJ_WIDTH(iObj); */
    Result.fObjectWidthVar = 0.0f;

    Result.fObjectCorridorWidth = MINMAX_FLOAT(
        Result.fObjectWidth, CAL_OBJ_WIDTH_FAC * Result.fObjectWidth,
        SIGetObjCorridorWidth(pObjectProperties));
    Result.fObjectCorridorWidthVar = SIGetObjWidthVarForCorridor(ObjNr);

    /* Get the trajectory distance */
    SITrajGetObjToRefDistance(
        ObjNr, &ObjToRefDist,
        &ObjToRefDistVar); /*! Remark: ObjToRefDistVar > 0 */

    /* Fill in low pass trajectory information */
    SICalcStatLowPassTraj();

    /* For stationary objects that are already assigned to the ego lane also use
    a low-pass filtered trajectory and select smaller distance to trajectory.
    This
    improves stationary approach performance on the proving ground, where no
    lane
    markers or stationary obstacle based road estimation is available.
    Only apply this to stationary (non-stopped) objects, as those already have
    extended trace brackets, which should assure correct selection */
    // if ((OBJ_GET_SI(ObjNr).ObjLaneAccStatus.SIInlaneState ==
    //      OBJ_STATE_INLANE) &&
    //     (OBJ_DYNAMIC_PROPERTY(ObjNr) == CR_OBJECT_PROPERTY_STATIONARY) &&
    //     (!OBJ_IS_MOVING_TO_STATIONARY(ObjNr))) {
    //     const float32 fObjDistX =
    //         OBJ_LONG_DISPLACEMENT(ObjNr) + VLC_fBumperToCoG;
    //     const float32 fObjDistY = OBJ_LAT_DISPLACEMENT(ObjNr);
    //     CPTrajRefPoint_t DistToLowPassTraj;

    //     CPCalculateDistance2Traj(fObjDistX, fObjDistY, TRUE,
    //                              &SIStatLowPassTraj.Traj,
    //                              &DistToLowPassTraj);
    //     if (fABS(DistToLowPassTraj.fDistToTraj) < fABS(ObjToRefDist)) {
    //         ObjToRefDist = DistToLowPassTraj.fDistToTraj;
    //         ObjToRefDistVar = 0.f;
    //     }
    // }

    /* Save it in global output */
    OBJ_GET_OBJ_TO_REF_DISTANCE(ObjNr) = pObjectProperties->fRefCourseDistY;

    /* this offset corrects the asymmetric extensions of lane borders */
    /* distance is assumed to be measured to lane center. therefor asymmetric
     * extensions must be considered as an offset. */
    CorridorCenter = (pBracketPositions->BracketPositionLeft +
                      pBracketPositions->BracketPositionRight) *
                     0.5f;
    Result.fDistance = pObjectProperties->fDistToCourse - CorridorCenter;
    Result.fDistanceVar = SIGetObjWidthVarForCorridor(ObjNr);

    /* Obtain corridor width */
    CorridorWidth = fABS(pBracketPositions->BracketPositionLeft -
                         pBracketPositions->BracketPositionRight);
    Result.fTrajectoryWidth = CorridorWidth;
    Result.fTrajectoryWidthVar =
        0.0f; /* 0.0f as long as extensions are working and track width is not
                 measured or estimated*/

    return Result;
}

/*************************************************************************************************************************
  Functionname:    SIPerformReflectionCorrection */
static void SIPerformReflectionCorrection(
    const ObjNumber_t ObjNr, CPDistanceWidth_t* const pCorridorDescription) {
    float32 fDistReflectionOffset;

    /* this offset corrects the reflection point to the center of the object */
    fDistReflectionOffset = SIGetReflectionOffset(
        ObjNr); /* Replaces: OT_GET_REFLECTION_OFFSET(iObj); */
    if (OBJ_ANGLE(ObjNr) > 0.0F) {
        pCorridorDescription->fDistance += fDistReflectionOffset;
    } else {
        pCorridorDescription->fDistance -= fDistReflectionOffset;
    }
}

/*************************************************************************************************************************
  Functionname:    SICalculateBaseLaneWidths */
static void SICalculateBaseLaneWidths(void) {
    fSISeekLaneWidth = SICalcRelTraTrackwidthSeek();
    fSITrackLaneWidth = SICalcRelTraTrackwidthTrack();
}

/*************************************************************************************************************************
  Functionname:    SICalcRelTraTrackwidthSeek */
static float32 SICalcRelTraTrackwidthSeek(void) {
    FIP_t_FusedRoadType iRoadTypeLevel_1;
    FIP_t_RoadWorks iRoadTypeLevel_2;
    FIP_t_LaneWidthSource t_LaneWidthSource = FIP_t_Get_LaneWidthSource();
    float32 f_LaneWidth = FIP_f_Get_LaneWidth();
    // FIP_t_LaneWidthClass sLaneWidthClass = FIP_t_Get_LaneWidthClass();

    float32 fret;
    float32 fclasswid;

    /* First calculate lane width seek based on ego speed */
    fclasswid = dGDBmathLineareFunktion(&TRCK_Seek_LaneWidth_SpeedAdapted,
                                        EGO_SPEED_X_OBJ_SYNC);

    fret = fclasswid;

    /* If road provides a valid lane width estimation (when source is not
    Source0, and within an acceptable range of
    [ROADWORKSLANEWIDTHSEEK ... STANDARDLANEWIDTHSEEK]) */
    if ((t_LaneWidthSource != FIP_SOURCE_0) &&
        (f_LaneWidth >= ROADWORKSLANEWIDTHSEEK) &&
        (f_LaneWidth <= STANDARDLANEWIDTHSEEK)) {
        /* Calculate the maximum permitted seek width based on the lane width
         * provided by road */
        const float32 fMaxLaneWidth =
            SI_PAR_SEEK_WIDTH_TO_CAM_LANE_WIDTH_TOLERANCE * f_LaneWidth;
        /* Calculate the minimum permitted seek width based on the lane width
         * provided by road */
        const float32 fMinLaneWidth =
            (1.f / SI_PAR_SEEK_WIDTH_TO_CAM_LANE_WIDTH_TOLERANCE) * f_LaneWidth;
        /* Clamp the returned lane width between the min-max just calculated */
        fret = MIN_FLOAT(fMaxLaneWidth, fret);
        fret = MAX_FLOAT(fMinLaneWidth, fret);
    }

    return fret;
}

/*************************************************************************************************************************
  Functionname:    SIGetBaseSeekLaneWidth */
float32 SIGetBaseSeekLaneWidth(void) { return (fSISeekLaneWidth); }

/*************************************************************************************************************************
  Functionname:    get_fSITrackLaneWidth */
float32 get_fSITrackLaneWidth(void) { return (fSITrackLaneWidth); }

/*************************************************************************************************************************
  Functionname:    SIInitCorridorCritModule */
static void SIInitCorridorCritModule(void) {
    SIInitCorridorCamParameter();
    /* Initialize Debug variable */

    fSISeekLaneWidth = 0.f;
    fSITrackLaneWidth = 0.f;
}

/*************************************************************************************************************************
  Functionname:    SICalcRelTraTrackwidthTrack */
static float32 SICalcRelTraTrackwidthTrack(void) {
    float32 fret;

    /* No road available : assume default highway widths */
    fret = dGDBmathLineareFunktion(&TRCK_Seek_LaneWidth_SpeedAdapted,
                                   EGO_SPEED_X_OBJ_SYNC);
    fret += dGDBmathLineareFunktion(&TRCK_Track_Offset_SpeedAdapted,
                                    EGO_SPEED_X_OBJ_SYNC);

    return fret;
}

/*************************************************************************************************************************
  Functionname:    SIInitCriteriaMatrix */
static void SIInitCriteriaMatrix(SICriteriaMatrix_t* const pCriteriaMatrix) {
    /*! Kriterien legen Spurklammererweiterung oder -beschr�nkung basierend auf
     * bisheriger Position fest */
    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->AddExtensionRelevantObject),
                               DEFAULT_MODE); /*!< VorbeifahrtRelObjekt */

    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->AddExtensionObjectFastCutIn),
                               DEFAULT_MODE); /*!< Approx */
    SIRelTraInitCriteriaOutput(
        &(pCriteriaMatrix->AddRestrictionCurveOuterBorder),
        DEFAULT_MODE); /*!< KurveAussenrand */
    SIRelTraInitCriteriaOutput(
        &(pCriteriaMatrix->AddExtensionHighspeedApproach),
        DEFAULT_MODE); /*!< Highspeed approaches */
    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->AddExtensionHighTunnelProb),
                               DEFAULT_MODE); /*!< Highspeed approaches */

    /*! Kriterien legen neue Spurklammerposition fest */
    SIRelTraInitCriteriaOutput(
        &(pCriteriaMatrix->RestrictionTargetOutsideBrackets),
        RESTRICTION_MODE); /*!< ZielAblage */
    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->RestrictionAnalogRoadBorder),
                               RESTRICTION_MODE); /*!< AnalogerStrassenrand */
    SIRelTraInitCriteriaOutput(
        &(pCriteriaMatrix->RestrictionNeighbourhoodRelObj),
        RESTRICTION_MODE); /*!< UmfeldRelevantesObjekt */

    SIRelTraInitCriteriaOutput(
        &(pCriteriaMatrix->AddExtensionLowSpeedFusedBrd),
        EXTENSION_MODE); /*!< Low speed short range extension */

    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->RestrictionCityNearRange),
                               RESTRICTION_MODE); /*!< Restriction in the near
                                                     range for country road
                                                     /city scenarios */
    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->ExtensionRoadBorder),
                               EXTENSION_MODE); /*!< Richtungsfahrspuren */
    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->ExtensionCurveInnerBorder),
                               EXTENSION_MODE); /*!< KurveInnenrand */
    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->ExtensionRoadBorderCI),
                               EXTENSION_MODE); /*!< RIchtungsfahrspurenKI */
    SIRelTraInitCriteriaOutput(
        &(pCriteriaMatrix->ExtensionFollowObjectIntoCurve),
        EXTENSION_MODE); /*!< ObjektWinkelVerfolgung */

    /*! Initialization: Adaption trace brackets based on in-lane position of
     * ego-vehicle (camera information), object specific variable */
    SIRelTraInitCriteriaOutputScale(
        &(pCriteriaMatrix
              ->RatioEgoPositionInLaneCam)); /*!< RatioEgoPositionInLaneCam */
    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->BaseCorridor), DEFAULT_MODE);
    SIRelTraInitCriteriaOutput(&(pCriteriaMatrix->FinalBracketPositions),
                               DEFAULT_MODE);
}

/*************************************************************************************************************************
  Functionname:    SIRelTraInitCriteriaOutput */
static void SIRelTraInitCriteriaOutput(SIBracketOutput_t* const pOutput,
                                       const SITraceBracketMode_t eMode) {
    switch (eMode) {
        case RESTRICTION_MODE:
            pOutput->BracketPositionLeft = INITVALUE_BRACKETPOSITION;
            pOutput->BracketPositionRight = -INITVALUE_BRACKETPOSITION;
            break;

        case EXTENSION_MODE:
            pOutput->BracketPositionLeft = -INITVALUE_BRACKETPOSITION;
            pOutput->BracketPositionRight = INITVALUE_BRACKETPOSITION;
            break;

        case DEFAULT_MODE:
        default:
            pOutput->BracketPositionLeft = 0.F;
            pOutput->BracketPositionRight = 0.F;
            break;
    }
}

/*************************************************************************************************************************
  Functionname:    SICorridorObjInit */
void SICorridorObjInit(SIObjCorridor_t* const pObjCor) {
    pObjCor->fRelevantTime = 0.0F;
    pObjCor->fNotRelevantTime = 0.0F;
    pObjCor->fTargetFusionHoldTime = 0.0F;

    pObjCor->TrackVehicle.iLwFolge = 0;
    pObjCor->TrackVehicle.fLatTrackLimitL = -INITVALUE_BRACKETPOSITION;
    pObjCor->TrackVehicle.fLatTrackLimitR = INITVALUE_BRACKETPOSITION;

    pObjCor->TrackVehicle.fLatTrackLimitExpandFac = 0.0F;
    pObjCor->TrackVehicle.fLatTrackLimitDistanceExpandFac = 0.0F;
}

/*************************************************************************************************************************
  Functionname:    SICalculateBracketsForCurrentObject */
static SIBracketOutput_t SICalculateBracketsForCurrentObject(
    const ObjNumber_t ObjNr,
    RelTraObjInput_t* const pObjectProperties,
    const RelTraCurve_t* const pTrajectory,
    AssTraEnvironment_t* const pEnvironment,
    const SICriteriaMatrixAllObj_t* const pCritMatrixAllObj) {
    SICriteriaMatrix_t BracketFuncResults;
    RelTraObjOutput_t ObjectVariables;
    SIBracketFuncEnable_t BracketFuncEnableFlags;

    /* Initialize lane assignment criteria matrix */
    SIInitCriteriaMatrix(&BracketFuncResults);

    /* Construct object variables from corridor criteria persistent struct */
    SIConstructObjectVariables(ObjNr, &ObjectVariables);

    /* Enable all corridor bracket functions */
    SIEnableAllBracketFunctions(&BracketFuncEnableFlags);

    /* Call to custom preprocessing which allows to enable and disable
       selected corridor bracket functions*/
    SICustomCorridorPreProcessing(ObjNr, &BracketFuncEnableFlags);

    /* Fill the entire CriteriaMatrix for the current object, the Matrix also
       contains the (preliminary) final bracket positions,
       Usually this will also be the final result, except for the special case
       of recalculation with fast trajectory and custom matrix processing. */
    /* This is the 'main' part in determining the corridor bracket positions ..
     */
    SIEvaluateBracketFunctions(pObjectProperties, &ObjectVariables, pTrajectory,
                               pEnvironment, &BracketFuncResults,
                               &BracketFuncEnableFlags, pCritMatrixAllObj);

    /* Determine final bracket positions given all calculated extensions and
     * restrictions */
    SIDetermineFinalBracketPositions(pObjectProperties, &ObjectVariables,
                                     &BracketFuncResults);

    /* In case the previously relevant object is lost now, the corridor is
       re-calculated
       with a VDY-only trajectory. */
    if ((pObjectProperties->iRelevant == TRUE) &&
        (ObjectVariables.iObjektSpur_Zyklus != ASSOC_LANE_EGO)) {
        SIRecalculateBracketsUsingAlternativeTrajectory(
            ObjNr, pObjectProperties, &ObjectVariables, pEnvironment,
            &BracketFuncResults, &BracketFuncEnableFlags, pCritMatrixAllObj);
    }

    /* write back variable object properties to global variables,
       fFusionTargetHoldTime is accessed via
       AssTraEnvironment.fRelObjTargetFusionHoldTime in in
       RestrictNeighbourhoodRelObj,
       also needed to save iLwFolge value for next cycle for
       ExtensionFollowObjectIntoCurve,
       also needs fLatTrackLimitL/R values from last cycle in
       ExtensionsCurveInnerBorder and ExtensionRoadBorder */
    SIRelTraObjOutputToObject(&ObjectVariables, &OBJ_GET_SI(ObjNr).ObjCor);

    /* return final bracket positions for current object */
    return BracketFuncResults.FinalBracketPositions;
}

/*************************************************************************************************************************
  Functionname:    SIAdjustObjectPropertiesToTrajectory */
static void SIAdjustObjectPropertiesToTrajectory(
    RelTraObjInput_t* const pObjectProperties,
    const RelTraCurve_t* const pTrajectory,
    const CPTrajectoryData_t* const pTrajectoryData) {
    float32 fX_Sum = 0;
    float32 fY_Sum = 0;
    CPTrajRefPoint_t TrajRefPoint;
    if (pTrajectory->iOWVflag == OWVKRIT_AKTIV) /* USE ACC Trajectory */
    {
        /* fDistToCourse is computed with ACC trajectory as reference */
        for (uint8 i = 0; i < 4; i++) {
            CPCalculateDistance2Traj(
                pObjectProperties->fDistX +
                    pObjectProperties->CornerPoint[i].fDistX,
                pObjectProperties->fDistY +
                    pObjectProperties->CornerPoint[i].fDistY,
                (boolean)pTrajectoryData->State.EgoCourseOnly,
                &pTrajectoryData->Current, &TrajRefPoint);

            pObjectProperties->CornerPoint[i].fDistOnCourse =
                TrajRefPoint.fDistOnTraj;
            pObjectProperties->CornerPoint[i].fDistToCourse =
                TrajRefPoint.fDistToTraj;
            pObjectProperties->CornerPoint[i].fRefCourseDistX = TrajRefPoint.fX;
            pObjectProperties->CornerPoint[i].fRefCourseDistY = TrajRefPoint.fY;

            fX_Sum += pObjectProperties->CornerPoint[i].fDistX;
            fY_Sum += pObjectProperties->CornerPoint[i].fDistY;
        }

        CPCalculateDistance2Traj(pObjectProperties->fDistX + fX_Sum / 4,
                                 pObjectProperties->fDistY + fY_Sum / 4,
                                 (boolean)pTrajectoryData->State.EgoCourseOnly,
                                 &pTrajectoryData->Current, &TrajRefPoint);

    } else if (pTrajectory->iOWVflag == OWVKRIT_INAKTIV) /* USE EGO Course */
    {
        /* fDistToCourse is computed with EGO-Course as reference */
        for (uint8 i = 0; i < 4; i++) {
            TrajRefPoint = CPCalculateDistancePoint2Circle(
                pObjectProperties->fDistX +
                    pObjectProperties->CornerPoint[i].fDistX,
                pObjectProperties->fDistY +
                    pObjectProperties->CornerPoint[i].fDistY,
                pTrajectory->dCurve);

            pObjectProperties->CornerPoint[i].fDistOnCourse =
                TrajRefPoint.fDistOnTraj;
            pObjectProperties->CornerPoint[i].fDistToCourse =
                TrajRefPoint.fDistToTraj;
            pObjectProperties->CornerPoint[i].fRefCourseDistX = TrajRefPoint.fX;
            pObjectProperties->CornerPoint[i].fRefCourseDistY = TrajRefPoint.fY;

            fX_Sum += pObjectProperties->CornerPoint[i].fDistX;
            fY_Sum += pObjectProperties->CornerPoint[i].fDistY;
        }

        TrajRefPoint = CPCalculateDistancePoint2Circle(
            pObjectProperties->fDistX + fX_Sum / 4,
            pObjectProperties->fDistY + fY_Sum / 4, pTrajectory->dCurve);
    }
    pObjectProperties->fDistToCourse = TrajRefPoint.fDistToTraj;
}

/*************************************************************************************************************************
  Functionname:    SIConstructObjectVariables */
static void SIConstructObjectVariables(const ObjNumber_t ObjNr,
                                       RelTraObjOutput_t* const pRelTraObj) {
    const SIObjCorridor_t* pObjCor = &OBJ_GET_SI(ObjNr).ObjCor;

    pRelTraObj->iLwFolge = pObjCor->TrackVehicle.iLwFolge;

    pRelTraObj->iObjektSpur_Zyklus = ASSOC_LANE_RIGHT;

    pRelTraObj->fLatTrackLimitL = 0.F;
    pRelTraObj->fLatTrackLimitR = 0.F;

    pRelTraObj->fTargetFusionHoldTime = pObjCor->fTargetFusionHoldTime;
}

/*************************************************************************************************************************
  Functionname:    SIRecalculateBracketsUsingAlternativeTrajectory */
static void SIRecalculateBracketsUsingAlternativeTrajectory(
    const ObjNumber_t ObjNr,
    RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    AssTraEnvironment_t* const pEnvironment,
    SICriteriaMatrix_t* const pBracketFuncResults,
    SIBracketFuncEnable_t* const pBracketFuncEnableFlags,
    const SICriteriaMatrixAllObj_t* const pCritMatrixAllObj) {
    SICriteriaMatrix_t AlternativeBracketFuncResults;
    RelTraObjOutput_t AlternativeObjectVariables;
    RelTraCurve_t AlternativeTrajectory;
    float32 Default_fRefCourseDistY;

    /* Initialize bracket functions */
    SIInitCriteriaMatrix(&AlternativeBracketFuncResults);

    /* Construct alternative trajectory and modify accordingly the objects
     * properties (fDistToCourse) */
    /* beforehand save AblageRadius for default trajectory to possibly restore
     * it later */
    Default_fRefCourseDistY = pObjectProperties->fRefCourseDistY;
    SIConstructAlternativeTrajectory(&AlternativeTrajectory);
    SIAdjustObjectPropertiesToTrajectory(
        pObjectProperties, &AlternativeTrajectory, &SITrajectoryData);

    /* Copy object data from corridor criteria persistent struct */
    SIConstructObjectVariables(ObjNr, &AlternativeObjectVariables);

    /* Calculation of priority and default corridor functions */
    SIEvaluateBracketFunctions(pObjectProperties, &AlternativeObjectVariables,
                               &AlternativeTrajectory, pEnvironment,
                               &AlternativeBracketFuncResults,
                               pBracketFuncEnableFlags, pCritMatrixAllObj);

    /* Determine final bracket positions given all calculated extensions and
     * restrictions */
    SIDetermineFinalBracketPositions(pObjectProperties,
                                     &AlternativeObjectVariables,
                                     &AlternativeBracketFuncResults);

    /* Check if object is now within brackets, eventually restore previous
     * results */
    if (AlternativeObjectVariables.iObjektSpur_Zyklus == ASSOC_LANE_EGO) {
        /* Use alternative trajectory recalculation results (copy structure) */
        *pObjectVariables = AlternativeObjectVariables;
        *pBracketFuncResults = AlternativeBracketFuncResults;
    } else {
        /* Omit results of recalculation */
        /* Restore the default trajectory value for the dAblageRadius property
         */
        pObjectProperties->fRefCourseDistY = Default_fRefCourseDistY;
    }
}

/*************************************************************************************************************************
  Functionname:    SIConstructAlternativeTrajectory */
static void SIConstructAlternativeTrajectory(
    RelTraCurve_t* const pRelTraCurve) {
    /* Copy ego curve to local curve stuct */
    pRelTraCurve->dCurve = EGO_CURVE_OBJ_SYNC;
    pRelTraCurve->dCurve_abs = fABS(EGO_CURVE_OBJ_SYNC);
    pRelTraCurve->iOWVflag = OWVKRIT_INAKTIV;
}

/*************************************************************************************************************************
  Functionname:    SIEnableAllBracketFunctions */
static void SIEnableAllBracketFunctions(
    SIBracketFuncEnable_t* const pBracketFuncEnable) {
    /* Restrict base corridor width to the seek lane-width only. The seek
       lane-width
       is smaller compared to the track lane-width and thus, objects can be
       released
       earlier in e.g. overtake-scenarios. */
    pBracketFuncEnable->bUseSeekLaneWidthOnly = FALSE;

    /* Enable / Disable corridor bracket functions */
    pBracketFuncEnable->bEnableAddExtensionObjectFastCutIn =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableAddRestrictionCurveOuterBorder =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableRestrictionTargetOutsideBrackets =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableRestrictionAnalogRoadBorder =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableRestrictionNeighbourhoodRelObj =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableRestrictionCityNearRange =
        SI_CORR_BRACKET_FUNC_DISABLED;
    pBracketFuncEnable->bEnableExtensionRoadBorder =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableExtensionCurveInnerBorder =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableExtensionRoadBorderCI =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableExtensionFollowObjectIntoCurve =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableExtensionGuardRailRoadBorder =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableAddExtensionRelevantObject =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableExtensionHighspeedApproach =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableAddExtensionHighTunnelProb =
        SI_CORR_BRACKET_FUNC_ENABLED;
    pBracketFuncEnable->bEnableExtensionLowSpeedFusedBrd =
        SI_CORR_BRACKET_FUNC_ENABLED;
}

/*************************************************************************************************************************
  Functionname:    SIUpdateRelevancyTimers */
static void SIUpdateRelevancyTimers(const ObjNumber_t ObjNr,
                                    SIObjCorridor_t* const pObjCor) {
    if (OBJ_GET_RELEVANT(ObjNr)) {
        if (pObjCor->fNotRelevantTime > C_F32_DELTA) {
            pObjCor->fNotRelevantTime = 0.F;
            pObjCor->fRelevantTime = 0.F;
        }
        pObjCor->fRelevantTime = pObjCor->fRelevantTime + SI_CYCLE_TIME;
        pObjCor->fRelevantTime =
            MIN_FLOAT(RELEVANTZEIT_MAX, pObjCor->fRelevantTime);
    } else {
        if (pObjCor->fRelevantTime > C_F32_DELTA) {
            pObjCor->fNotRelevantTime =
                pObjCor->fNotRelevantTime + SI_CYCLE_TIME;
            pObjCor->fNotRelevantTime =
                MIN_FLOAT(RELEVANTZEIT_MAX, pObjCor->fNotRelevantTime);
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIUpdateTimeTrackLaneExtFactor */
static void SIUpdateTimeTrackLaneExtFactor(const ObjNumber_t ObjNr,
                                           SIObjCorridor_t* const pObjCor) {
    /*--- VARIABLES ---*/
    float32 dIncDecFaktor;
    float32 dAbstFaktor;

    if (OBJ_GET_RELEVANT(ObjNr)) {
        /* Objekt innerhalb der Spurbreite fuer eigene Spur */

        /* dAbstFaktor gibt die Abstandsabhaengigkeit des
         * Inkrement/Dekrementanteils vor   */
        /* 0 = Schnelles Inkrement/Dekrement 1 = Langsames Inkrement/Dekrement
         */
        /* Bei dAbstFaktor = 0 gilt die SPURZEIT_MIN, bei = 1 die SPURZEIT_MAX,
         * dazwischen */
        /* linear. SPURZEIT_MIN gibt die kuerzeste Zeit zum Erreichen des
         * Maximums von     */
        /* fLatTrackLimitExpandFac an, SPURZEIT_MAX die laengste Zeit. */

        if (OBJ_LONG_DISPLACEMENT(ObjNr) < SPUR_SICHERHEIT) {
            /* Objekt innerhalb des Abstands fuer Spursicherheit */
            /* je naeher, desto kleiner der dAbstFaktor          */
            dAbstFaktor =
                OBJ_LONG_DISPLACEMENT(ObjNr) * (1.F / SPUR_SICHERHEIT);

            /* positiver dIncDecFaktor da fLatTrackLimitExpandFac 0->1 */
            dIncDecFaktor =
                SI_CYCLE_TIME /
                (SPURZEIT_MIN + ((SPURZEIT_MAX - SPURZEIT_MIN) * dAbstFaktor));

        } else {
            /* Objekt ausserhalb des Abstands fuer Spursicherheit */
            /* je weiter entfernt, desto kleiner der dAbstFaktor  */
            dAbstFaktor =
                1.F - ((OBJ_LONG_DISPLACEMENT(ObjNr) - SPUR_SICHERHEIT) /
                       (RW_VLC_MAX - SPUR_SICHERHEIT));

            /* negativer dIncDecFaktor da fLatTrackLimitExpandFac 1->0 */
            if (fABS(SPURZEIT_MIN + ((SPURZEIT_MAX - SPURZEIT_MIN) *
                                     dAbstFaktor)) > C_F32_DELTA) {
                dIncDecFaktor =
                    -(SI_CYCLE_TIME /
                      (SPURZEIT_MIN +
                       ((SPURZEIT_MAX - SPURZEIT_MIN) * dAbstFaktor)));
            } else {
                dIncDecFaktor = 0.f;
            }
        }
    } else {
        /* Objekt ausserhalb der Spurbreite fuer eigene Spur             */
        /* negativer dIncDecFaktor damit fLatTrackLimitExpandFac 1->0 */
        dIncDecFaktor = -(SI_CYCLE_TIME * (1.F / SPURZEIT_DEC));
    }

    pObjCor->TrackVehicle.fLatTrackLimitExpandFac += dIncDecFaktor;

    float max_fac = dGDBmathLineareFunktion(&TRCK_Seek_ExpandFac_DistAdapted,
                                            OBJ_LONG_DISPLACEMENT(ObjNr));

    pObjCor->TrackVehicle.fLatTrackLimitExpandFac = MINMAX_FLOAT(
        0.F, max_fac, pObjCor->TrackVehicle.fLatTrackLimitExpandFac);
}

/*************************************************************************************************************************
  Functionname:    SIUpdateDistTrackLaneExtFactor */
static void SIUpdateDistTrackLaneExtFactor(const ObjNumber_t ObjNr,
                                           SIObjCorridor_t* const pObjCor) {
    /*--- VARIABLES ---*/
    float32 dAbstandDiff = 0.F;

    if ((SIRelObject.ObjectNr == ObjNr) && (OBJ_GET_RELEVANT(ObjNr)) &&
        ((OBJ_LONG_VREL(ObjNr)) < SI_MAX_VREL_REL_LANE_EXT_FACT)) {
        /* Object within trace brackets (ego lane) and we are approaching it */
        dAbstandDiff =
            (SIRelObject.RelPickupDist - OBJ_LONG_DISPLACEMENT(ObjNr));

        /* 10m - 30 m => RelSpurAbstandErweitFaktor 0-1 */
        /* Der RelSpurAbstandErweitFaktor wird am Ende der Funktion auf Werte
         * zwischen 0 und 1 begrenzt */
        pObjCor->TrackVehicle.fLatTrackLimitDistanceExpandFac =
            (dAbstandDiff - SPUR_ABSTANDDIFF_MIN) *
            (1.f / (SPUR_ABSTANDDIFF_MAX - SPUR_ABSTANDDIFF_MIN));

        pObjCor->TrackVehicle.fLatTrackLimitDistanceExpandFac = MINMAX_FLOAT(
            0.F, 1.F, pObjCor->TrackVehicle.fLatTrackLimitDistanceExpandFac);
    } else {
        /* Object not relevant or we are not approaching it with a sufficiently
        large
        speed difference, reset extension factor */
        pObjCor->TrackVehicle.fLatTrackLimitDistanceExpandFac = 0.0F;
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraObjOutputToObject */
static void SIRelTraObjOutputToObject(const RelTraObjOutput_t* const pRelTraObj,
                                      SIObjCorridor_t* const pObjCor) {
    pObjCor->TrackVehicle.iLwFolge = pRelTraObj->iLwFolge;

    pObjCor->TrackVehicle.fLatTrackLimitL = pRelTraObj->fLatTrackLimitL;
    pObjCor->TrackVehicle.fLatTrackLimitR = pRelTraObj->fLatTrackLimitR;
    pObjCor->fTargetFusionHoldTime = pRelTraObj->fTargetFusionHoldTime;
}

/*************************************************************************************************************************
  Functionname:    SIManipulateLaneClassBasedOnCam */
static void SIManipulateLaneClassBasedOnCam(
    FIP_t_LaneWidthClass* const pLaneWidthClass) {
    static float32 fLastCycleCamLaneWidth = STANDARDLANEWIDTHSEEK;
    /* Obtain lane width measured by camera */
    const float32 fCamLaneWidth = FIP_f_GetWidthCamLane();
    /* Check if camera lane width is available */
    float32 fCamLaneWidthFiltered = 0.0f;
    if (fCamLaneWidth > C_F32_DELTA) {
        /* Apply low pass filter to smooth the used lane width. Apply filter
           only if the last camera lane width value is valid.
           Otherwise, initialize filtered camera lane width to current camera
           lane width value. */
        if (fLastCycleCamLaneWidth > C_F32_DELTA) {
            fCamLaneWidthFiltered = GDB_FILTER(
                fCamLaneWidth, fLastCycleCamLaneWidth, CAM_LANE_FILTER_CONST);
        } else {
            fCamLaneWidthFiltered = fCamLaneWidth;
        }

        fLastCycleCamLaneWidth = fCamLaneWidthFiltered;
        switch (*pLaneWidthClass) {
            case FIP_LANE_WIDTH_CLASS_NARROW:
                if ((fCamLaneWidthFiltered <=
                     SI_CAM_LANEWIDTH_MORE_NARROW_LANE_MAX) &&
                    (EGO_SPEED_X_OBJ_SYNC < SI_CAM_LANEWIDTH_MAX_SPEED)) {
                    *pLaneWidthClass = FIP_LANE_WIDTH_CLASS_MORE_NARROW;
                } else {
                    /* do nothing */
                }
                break;
            case FIP_LANE_WIDTH_CLASS_MORE_NARROW:
                /* do nothing */
                break;
            /* Map measured camera lane width to lane class, if camera indicates
             * a smaller lane class than the radar-based estimation */
            case FIP_LANE_WIDTH_CLASS_UNKNOWN:
            case FIP_LANE_WIDTH_CLASS_NORMAL:
            default:
                if ((fCamLaneWidthFiltered <=
                     SI_CAM_LANEWIDTH_MORE_NARROW_LANE_MAX) &&
                    (EGO_SPEED_X_OBJ_SYNC < SI_CAM_LANEWIDTH_MAX_SPEED)) {
                    *pLaneWidthClass = FIP_LANE_WIDTH_CLASS_MORE_NARROW;
                } else if ((fCamLaneWidthFiltered <=
                            SI_CAM_LANEWIDTH_NARROW_LANE_MAX) &&
                           (EGO_SPEED_X_OBJ_SYNC <
                            SI_CAM_LANEWIDTH_MAX_SPEED)) {
                    *pLaneWidthClass = FIP_LANE_WIDTH_CLASS_NARROW;
                } else {
                    /* do nothing */
                }
                break;
        }
    } else {
        /*! Set last camera lane width to zero if camera lane width is invalid
         */
        fLastCycleCamLaneWidth = 0.f;
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