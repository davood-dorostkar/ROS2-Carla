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
#include "si_lanechange.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/
#ifndef _PARAM_UNUSED
#define _PARAM_UNUSED(x) (void)(x)
#endif

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*----------------------------------------------------------------------------
  EO_LaneChange.c
----------------------------------------------------------------------------*/
/* Initial value for the correction with reference to time: */
/*   Number of cycles for the correction of the lane calculation matrix ( + 1 !
 * ) */
#define iFSW_KANF 4u

/* Constant lower basic value of velocity: */
#define FSW_VEIGEN_UE ((80.F / C_KMH_MS))
#define FSW_VEIGEN_OE ((160.F / C_KMH_MS))
/* Almost straight ahead (constant lower corner value of velocity): */
#define FSW_GERADE_SPUR_UE (+1500.F)
/* Almost straight ahead (constant upper corner value of velocity): */
#define FSW_GERADE_SPUR_OE (+2500.F)

/* Chase ride: */
#define dFSW_FZ_TOLERANZ (0.9F)

/* Ableitung der Kurvendetektion per Faktor aus Geradeausfahrt (Hysterese): */
#define FSW_NACH_SEITE_FAKTOR (0.8F)

/* Endwert fuer die Korrektur mit zeitlichem Bezug: */
#define iFSW_KEND 1u

/* Neutralwert fuer die Korrektur */
#define iFSW_KNEUTRAL 0u

/* State transition times for SALaneChangeSideState_t enumeration */
#define FSW_EINLENKEN_1_ZEIT 20u
#define FSW_EINLENKEN_2_ZEIT 25u
#define FSW_GERADELENKEN_ZEIT 50u
#define FSW_LCSTATE_TIMER_MAX 255u

/* State transition times for SARelObjLaneState_t enumeration */
#define FSW_RELOBJSTATE_1_TO_2 10u
#define FSW_RELOBJSTATE_2_TO_NORM 40u
#define FSW_RELOBJSTATE_TIMER_MAX 255u

/***************************************************/
/*! Calculation of lane change probability */
#define SI_LC_PROB_MIN_DIST_CAM_COURSE                              \
    (7.f) /*! Minimal camera course distance (visibility range) for \
             considering camera infos */
#define SI_LC_PROB_TIME_CAM_LANE_MARKER_CROSSED                           \
    (0.5f) /*! Time for test if camera lane marker is crossed within this \
              time */
#define SI_LC_PROB_MAX_DIFF_PSI                                          \
    (0.02f) /*! Maximum reliable difference in heading angle between two \
               timestamps which is reliable */
#define SI_LC_PROB_FILTER_CONST_DIFF_LAT_CURVE                              \
    (35.f / 3.3f) /*!< Filter constant for filtering the lateral difference \
                     based on curves */
#define SI_LC_PROB_FILTER_CONST_CURVE_LEVEL                                 \
    (200.f / 3.3f) /*!< Filter constant for the curve level ("curve level": \
                      Should be zero on a straight road and non-zero in a   \
                      curve) */
#define SI_LC_PROB_MIN_DIFF_CURVE_ANALYSE_DISTLAT                          \
    (0.015f) /*!< Minimum difference between ego curve and driver intended \
                curve to analyse driven distance in lateral direction */
#define SI_LC_PROB_MIN_DIST_DIFF_CURVE_ANALYSE_DISTLAT                       \
    (0.1f) /*!< Minimum driven lateral distance based on difference between  \
              ego curve and driver intended curve to analyse driven distance \
              in lateral direction */
#define SI_LC_PROB_MIN_LCPROB_ANALYSE_DISTLAT                               \
    (40) /*!< Minimum lane change probablity (last cycle) to analyse driven \
            distance in lateral direction */
#define SI_LC_PROB_MIN_CURVE_CURVE_LEVEL \
    (0.004f) /*!< Maximum curve to determine mean curve level */
#define SI_LC_PROB_MAX_CURVE_CURVE_LEVEL \
    (0.03f) /*!< Minimum curve to determine mean curve level */
#define SI_LC_PROB_MIN_EGOVELO_SWCURVE_EVAL \
    (5.f) /*!< Minimum velocity for evaluation of the mean curve level */
#define SI_LC_PROB_FILTER_CONST_EGO_CURVE                                      \
    (60.0f / 3.3f) /*!< Time constant for filtering the ego curvature to mimic \
                      behavior of old ARS300 signals */

/* PT1 Element Back Transformations to X */
/* X = 0.975 --> T = 0.78 (for 20ms) */
/* pDynPar->fEgoCurvePT1X = (float32) 0.78f / ( (float32) 0.78f + fCycleTime );
 */
#define SI_LC_PROB_EGO_CURVE_PT1_X \
    (0.78f) /*!< Filter coefficient for ego curve */
#define SI_LC_PROB_LOW_SPEED_LEVEL                                       \
    (20.f) /*!< Below this threshold the ego velocity in considered "low \
              speed" */
#define SI_LC_PROB_DIFF_CURVE_FACTOR                                        \
    (2.5f) /*!< Parameter to increase the difference between ego and driver \
              intended curve in order to reach 100 LC prob in case of a LC */
#define SI_LC_PROB_MAX_TIME_TURN_LIGHT_ON \
    (30.f) /*!< Maximum time that a turn light is considered to be enabled */
#define SI_LC_PROB_MAX_TIME_AFTER_TURN_LIGHT_ACTIVE \
    (100.f) /*!< Timer value which leads to a lane change probability of 0 */
/*!< Calculation of lane change probability */
/***************************************************/

/***************************************************/
/*! Detection of the camera lane marker crossing */
#define SI_LC_MIN_DIFF_DIST_CAM_LANE_MARKER                                  \
    (1.f) /*!< Minimum difference in the camera lane marker distance between \
             the current and the last cycle */
#define SI_LC_MIN_MAX_INTERVAL_DIST_CAM_LANE_MARKER \
    (0.75f) /*!< Maximum interval for the camera lane marker distance */
#define SI_LC_CAM_LANE_MARKER_HISTORY_BUFFER                                 \
    (5u) /*!< Size of the history buffer for the distance to the camera lane \
            markers */
#define SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST \
    (99.f) /*!< Default distance to camera lane marker */
#define SI_LC_CAM_MOVE_PRE_MIN_DIVENDIST_TO_SIDE \
    (0.1f) /*!< Minimal distance the ego vehicle is driven to one side */
#define SI_LC_CAM_MOVE_PRE_MAX_DIVENDIST_TO_SIDE \
    (1.f) /*!< Maximal distance the ego vehicle is driven to one side */
#define SI_LC_CAM_MOVE_PRE_LM_MIN_LC_PROB                              \
    (30) /*!< Minimal lane change probability to consider it as a lane \
            change for the FIP lane matrix */
#define SI_LC_CAM_MOVE_PRE_MAX_DIST_LANE_MARKER_LC                        \
    (0.9f) /*!< Maximal distance to the camera lane marker to consider it \
              as a lane change for the FIP lane matrix */
/*!< Detection of the camera lane marker crossing */
/***************************************************/

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*! State enumeration of a lane change process per side */
typedef enum SALaneChangeSideStateTag {
    FSW_ST_WAIT,
    FSW_ST_STRAIGHT,
    FSW_ST_STEER_INTO_1,
    FSW_ST_STEER_INTO_2,
    FSW_ST_GERADELENKEN
} SALaneChangeSideState_t;

/*! State enumeration of lane change process single */
typedef enum SARelObjLaneStateTag {
    FSW_RELOBJ_WAIT,   /*!< Waiting for a relevant object (none tracked by lane
                          change algo, init state) */
    FSW_RELOBJ_INLANE, /*!< Releavnt object inlane */
    FSW_RELOBJ_AUSSPUR_L1, /*!< Releavnt object out-of-lane left for short
                              period of time */
    FSW_RELOBJ_AUSSPUR_L2, /*!< Relevant object out-of-lane left for long period
                              of time */
    FSW_RELOBJ_AUSSPUR_R1, /*!< Relevant object out-of-lane right for short
                              period of time */
    FSW_RELOBJ_AUSSPUR_R2 /*!< Relevant object out-of-lane right for long period
                             of time */
} SARelObjLaneState_t;

/*! Lane change information for one side (L/R), formerly FahrspurwechselSeite_t
 */
typedef struct {
    SALaneChangeSideState_t
        eLaneChangeState; /*!< State-machine for lane change process */
    uint8 uiLCStateTimer; /*!< Timer for state transitions
                             @max:FSW_LCSTATE_TIMER_MAX @unit:cycles */
    uint8 uiCorrection;   /*!< Signal for correction
                            = 0     = no correction
                            > 0     = correction with timed expiration!
                            = KANF  = Initialization value, decremented in every
                            cycle.
                            @min:0 @max:iFSW_KANF */
} SALaneChangeSide_t;

/*! Lane change information, formerly known as Fahrspurwechsel_t */
typedef struct {
    float32 dGeradeAus;
    float32 dNachSeite;

    SARelObjLaneState_t eRelObjLaneState; /*!< State of relevant object */

    uint8 uiRelObjLaneStateTimer; /*!< Timer for relevant object lane state
                                     @unit:cycles @max:
                                     FSW_RELOBJSTATE_TIMER_MAX */

    ObjNumber_t iLaneChangeTrRelObj; /*!< To recognise relevant object changes
                                        (stores ID of rel obj or
                                        OBJ_INDEX_NO_OBJECT if none) */

    SALaneChangeSide_t LeftSide;

    SALaneChangeSide_t RightSide;

} SALaneChange_t;

/*! History data for the camera lane crossing detection */
typedef struct SILCCamLaneMarkerHistData_t {
    boolean b_CrossingLeft;  /*!< Info if camera lane is crossed driving to the
                                left direction */
    boolean b_CrossingRight; /*!< Info if camera lane is crossed driving to the
                                right direction */
    float32 f_Dist;          /*!< Distance to the camera lane marker */
    uint32 LaneMarkerPoE;    /*!< State to the camera lane marker */
} t_SILCCamLaneMarkerHistData;

/*! Global data for camera lane crossing detection */
typedef struct SICamLaneCrossedData_t {
    t_SILCCamLaneMarkerHistData LeftLaneMarkerLastCycle; /*!< History data for
                                                            the camera lane
                                                            crossing detection
                                                            based on the left
                                                            camera lane marker
                                                            */
    t_SILCCamLaneMarkerHistData
        RightLaneMarkerLastCycle; /*!< History data for the camera lane crossing
                                     detection based on the left camera lane
                                     marker */
    t_SILCStateCamLaneMarkerCrossed
        CamLaneMarkerCrossed; /*!< Camera lane marker crossing state */
} t_SICamLaneCrossedData;

typedef struct SILCCamPreMoveneData_t {
    float32 af_LeftCamMarkerDistHistory
        [SI_LC_CAM_LANE_MARKER_HISTORY_BUFFER]; /*!< Buffer: Distance to the
                                                   lane marker on the left side
                                                   */
    float32 af_RightCamMarkerDistHistory
        [SI_LC_CAM_LANE_MARKER_HISTORY_BUFFER]; /*!< Buffer: Distance to the
                                                   lane marker on the right side
                                                   */
} t_SILCCamPreMoveneData;

/*! Lane change pre state: state if moving towards the new lane during ego lane
 * change, before entering the new lane */
SET_MEMSEC_VAR(SILaneChangeCamPreMoveState)
static t_SILaneChangeCamPreMove SILaneChangeCamPreMoveState;

/*!< Calculation of lane change probability */
/***************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/* Konstantenfeld fuer Fahrspurwechsel: */
SET_MEMSEC_CONST(FSW_GERADE_GRENZEN)
static const GDBLFunction_t FSW_GERADE_GRENZEN = {

    /* Ausgangswert A1: */
    FSW_GERADE_SPUR_UE,
    /* Ausgangswert A2: */
    FSW_GERADE_SPUR_OE,
    /* Steigung der Anpassungsgerade:        (A2-A1)/(E2-E1) */
    (FSW_GERADE_SPUR_OE - FSW_GERADE_SPUR_UE) / (FSW_VEIGEN_OE - FSW_VEIGEN_UE),
    /* Achsabschnitt der Anpassungsgerade:   A1 - (A2-A1)/(E2-E1) * E1 */
    FSW_GERADE_SPUR_UE - (((FSW_GERADE_SPUR_OE - FSW_GERADE_SPUR_UE) /
                           (FSW_VEIGEN_OE - FSW_VEIGEN_UE)) *
                          FSW_VEIGEN_UE)};

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*! Lane change information used for brackets, formerly known as
 * OTFahrspurWechsel  */
SET_MEMSEC_VAR(SALaneChangeInst)
static SALaneChange_t SALaneChangeInst;

SET_MEMSEC_VAR(SICamLaneCrossedData)
static t_SICamLaneCrossedData SICamLaneCrossedData; /*!< Global data for the
                                                       detection of crossed
                                                       camera lane markers */

SET_MEMSEC_VAR(SILCCamMovePreGlobalData)
static t_SILCCamPreMoveneData
    SILCCamMovePreGlobalData; /*!< Structure to store global data for lane
                                 change camera move pre */

/*****************************************************************************
  Modullocal Functions
*****************************************************************************/

static void SACheckLaneChangeState(SALaneChangeSide_t* pSide,
                                   const float32 dCurvature,
                                   const float32 dGradient);

static void SIInitLCProbDataGlobal(void);
static void SIInitLCProbDataLocal(const float32 fCycleTime,
                                  SILCProbDataLocal_t* pSILCProbDataLocal);
static void SIInitLCProbDetectors(SILCProbDetectors_t* pSILCProbDetectors);
static void SIInitLCProbInputData(SILCProbInputData_t* pSILCProbInputData);
static void SILCProbSetTurnLightPhase(const float32 fCycleTime,
                                      const SILCProbDirection_t LCProbDirection,
                                      const boolean bTLStatusActive,
                                      SILCProbTurnLightEval_t* pTLEval);
static void SILCProbGetCamLaneInfo(SILCProbCamLaneInfo_t* pCamLaneEval);
static void SILCProbGetEgoMotionInfo(
    const SILCProbDataLocal_t* pSILCProbDataLocal,
    float32* pfEgoCurvePT1,
    float32* pfDrvIntCurvePT1);
static void SILCProbGetDrivenDistVeloLateralDuringLC(
    const float32 fCycleTime,
    const float32 fEgoCurvePT1,
    const float32 fDrvIntCurvePT1,
    float32* pfDistLat);
static void SICalcLaneChangeProbability(const float32 fCycleTime);

static void SI_v_InitGlobalCameraLaneCrossingData(void);
static boolean SI_b_GetValidCamLane(uint32 u_LaneMarkerPoELastCycle,
                                    uint32 u_LaneMarkerPoECurrentCycle);
static void SI_v_DetectCamLaneCrossing(
    boolean* pb_LCLeftMarkerLeft,
    boolean* pb_LCLeftMarkerRight,
    boolean* pb_LCRightMarkerLeft,
    boolean* pb_LCRightMarkerRight,
    const boolean b_LaneMarkerLeftValid,
    const boolean b_LaneMarkerRightValid,
    const t_SILCCamLaneMarkerHistData* p_LeftLaneMarkerLastCycle,
    const t_SILCCamLaneMarkerHistData* p_RightLaneMarkerLastCycle);
static void SI_v_StateCamLaneCrossing(
    t_SILCStateCamLaneMarkerCrossed* p_StateCamLaneMarkerCrossed,
    t_SILCCamLaneMarkerHistData* p_LeftLaneMarkerLastCycle,
    t_SILCCamLaneMarkerHistData* p_RightLaneMarkerLastCycle);

static void SI_v_InitGlobLCCamLanePreMove(void);
static void SI_v_SetLaneChangeMovePre(
    t_SILaneChangeCamPreMove* p_SILaneChangeCamPreMoveState);

/*************************************************************************************************************************
  Functionname:    SIInitLaneChangeCam */
void SIInitLaneChangeCam(void) {
    /*--- FUNKTIONSLOKALE SYMBOLISCHE KONSTANTEN ---*/

    /*--- FUNKTIONSLOKALE KONSTANTEN ---*/

    /*--- FUNKTIONSLOKALE VARIABLEN ---*/

    SALaneChangeInst.LeftSide.eLaneChangeState = FSW_ST_WAIT;

    SALaneChangeInst.RightSide.eLaneChangeState = FSW_ST_WAIT;

    SALaneChangeInst.LeftSide.uiLCStateTimer = 0u;

    SALaneChangeInst.LeftSide.uiCorrection = 0u;

    SALaneChangeInst.RightSide.uiLCStateTimer = 0u;

    SALaneChangeInst.RightSide.uiCorrection = 0u;

    SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_WAIT;

    SALaneChangeInst.iLaneChangeTrRelObj = OBJ_INDEX_NO_OBJECT;

    SALaneChangeInst.dGeradeAus = (0.5f / FSW_GERADE_SPUR_UE);
    SALaneChangeInst.dNachSeite =
        (0.5f / (FSW_GERADE_SPUR_UE * FSW_NACH_SEITE_FAKTOR));

    SALaneChangeInst.uiRelObjLaneStateTimer = 0u;

    /*--- AUFHEBUNG FUNKTIONSLOKALE SYMBOLISCHE KONSTANTEN ---*/

    /*! Initialize the global data for lane change probability calculation */
    SIInitLCProbDataGlobal();

    /*! Initialize the global data for the camera lane crossing detection */
    SI_v_InitGlobalCameraLaneCrossingData();

    /*! Initialize global information for lane change camera move pre state */
    SI_v_InitGlobLCCamLanePreMove();
}

/*************************************************************************************************************************
  Functionname:    SACheckLaneChangeState */
static void SACheckLaneChangeState(SALaneChangeSide_t* pSide,
                                   const float32 dCurvature,
                                   const float32 dGradient) {
    /*--- FUNKTIONSLOKALE SYMBOLISCHE KONSTANTEN ---*/

    /*--- FUNKTIONSLOKALE KONSTANTEN ---*/

    /*--- FUNKTIONSLOKALE VARIABLEN ---*/

    if (pSide->uiLCStateTimer < FSW_LCSTATE_TIMER_MAX) {
        pSide->uiLCStateTimer++;
    }

    /* Recognition lane change: */
    switch (pSide->eLaneChangeState) {
        case FSW_ST_WAIT:
            /* Next state driving straight, with condition of curvature below
             * threshold: */
            if (fABS(dCurvature) < SALaneChangeInst.dGeradeAus) {
                pSide->eLaneChangeState = FSW_ST_STRAIGHT;
            }
            break;

        case FSW_ST_STRAIGHT:
            /* Curve or lane change beginning: */
            if (dCurvature > SALaneChangeInst.dNachSeite) {
                /* Next state: curvature goes into the given state: */
                pSide->eLaneChangeState = FSW_ST_STEER_INTO_1;
                pSide->uiLCStateTimer = 0u;
            } else {
                if (dCurvature < -SALaneChangeInst.dNachSeite) {
                    /* Curvature points to other side, go back to initial state
                     * (wait) */
                    pSide->eLaneChangeState = FSW_ST_WAIT;
                }
            }
            break;

        case FSW_ST_STEER_INTO_1:
            /* Check for abort using timer */
            if (pSide->uiLCStateTimer > FSW_EINLENKEN_1_ZEIT) {
                pSide->eLaneChangeState = FSW_ST_WAIT;
            }

            /* Check for next state (when curvature change gradient goes back)
             */
            if (dGradient < C_F32_DELTA) {
                pSide->uiLCStateTimer = 0u;
                pSide->eLaneChangeState = FSW_ST_STEER_INTO_2;
            }
            break;

        case FSW_ST_STEER_INTO_2:
            /* Abbruch */
            /* Diese Abbruchzeit hat Auswirkungen auf Funktion bei niedrigen
             * Gewchwindigkeiten */
            /* Zeit eventuell abh�ngig machen vom minimalem Kurvenradius */
            /* Dateien: c:\daten\spurwechsel\V220-679\19_05_99\19059909 */
            /*          c:\daten\spurwechsel\V220-679\19_05_99\1905990a */
            if (pSide->uiLCStateTimer > FSW_EINLENKEN_2_ZEIT) {
                /* Abort due to timer too high */
                pSide->eLaneChangeState = FSW_ST_WAIT;
            }

            /* Naechster Zustand: */
            /* Mehrere Moeglichkeiten, je nach gewuenschtem Ansprechverhalten:
             */
            /* Fruehestes Ansprechen:  if (dCurvature <
             * SALaneChangeInst.dNachSeite) */
            /* Spaeteres Ansprechen:   if (fABS(dCurvature) <
             * SALaneChangeInst.dGeradeAus) */
            /* Wirkliche Geradeausfahrt bzw Gegenlenken, ganz spaetes
             * Ansprechen: */
            /*                         if (dSpurRadius < C_F32_DELTA) */
            if (dCurvature < SALaneChangeInst.dNachSeite) {
                pSide->uiLCStateTimer = 0u;
                pSide->eLaneChangeState = FSW_ST_GERADELENKEN;
            }
            break;

        case FSW_ST_GERADELENKEN:
            /* Abort */
            if (pSide->uiLCStateTimer > FSW_GERADELENKEN_ZEIT) {
                pSide->eLaneChangeState = FSW_ST_WAIT;
            }
            break;

        default:
            pSide->eLaneChangeState = FSW_ST_WAIT;
            break;
    }

    /*--- AUFHEBUNG FUNKTIONSLOKALE SYMBOLISCHE KONSTANTEN ---*/
}

/*************************************************************************************************************************
  Functionname:    SIDetectLaneChangeCam */
void SIRelObjLaneStateChoose(const ObjNumber_t RelTrckObjNr) {
    float32 fRelObjDist2Course = OBJ_LAT_DISPLACEMENT(RelTrckObjNr);
    switch (SALaneChangeInst.eRelObjLaneState) {
        case FSW_RELOBJ_WAIT:
            if (fABS(fRelObjDist2Course) < dFSW_FZ_TOLERANZ) {
                SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_INLANE;
                SALaneChangeInst.iLaneChangeTrRelObj = RelTrckObjNr;
            }
            break;
        case FSW_RELOBJ_INLANE:
            if (fRelObjDist2Course < (-2.0F * dFSW_FZ_TOLERANZ)) {
                SALaneChangeInst.uiRelObjLaneStateTimer = 0u;
                SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_AUSSPUR_R1;
            }
            if (fRelObjDist2Course > (2.0F * dFSW_FZ_TOLERANZ)) {
                SALaneChangeInst.uiRelObjLaneStateTimer = 0u;
                SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_AUSSPUR_L1;
            }
            break;

        case FSW_RELOBJ_AUSSPUR_L1:
            if (SALaneChangeInst.uiRelObjLaneStateTimer >
                FSW_RELOBJSTATE_1_TO_2) {
                SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_AUSSPUR_L2;
            }
            break;

        case FSW_RELOBJ_AUSSPUR_R1:
            if (SALaneChangeInst.uiRelObjLaneStateTimer >
                FSW_RELOBJSTATE_1_TO_2) {
                SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_AUSSPUR_R2;
            }
            break;

        case FSW_RELOBJ_AUSSPUR_L2:
        case FSW_RELOBJ_AUSSPUR_R2:
            if (SALaneChangeInst.uiRelObjLaneStateTimer >
                FSW_RELOBJSTATE_2_TO_NORM) {
                SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_WAIT;
            }
            break;
        default:
            break;
    } /* endswitch */
}

/*************************************************************************************************************************
  Functionname:    SIDetectLaneChangeCam */
void SIDetectLaneChangeCam(void) {
    const fVelocity_t fEgoSpeed = EGO_SPEED_X_OBJ_SYNC;
    fCurve_t fCurveStandard;
    float32 fGradientOfCurveStandard;
    const ObjNumber_t RelTrckObjNr = OBJ_GET_RELEVANT_OBJ_NR;

    /* calculate target curvature and gradient of curvature for lane change
     * threshold*/
    fCurveStandard = (-0.5f) * EGO_CURVE_OBJ_SYNC;
    fGradientOfCurveStandard = (-0.5f) * EGO_CURVE_GRAD_OBJ_SYNC;

    /* calculate Threshold through speed interpolation */
    SALaneChangeInst.dGeradeAus =
        dGDBmathLineareFunktion(&FSW_GERADE_GRENZEN, fEgoSpeed);
    SALaneChangeInst.dGeradeAus = (0.5f / SALaneChangeInst.dGeradeAus);
    SALaneChangeInst.dNachSeite =
        SALaneChangeInst.dGeradeAus / FSW_NACH_SEITE_FAKTOR;

    if (RelTrckObjNr >= 0) {
        if (OBJ_DYNAMIC_PROPERTY(RelTrckObjNr) != CR_OBJECT_PROPERTY_MOVING) {
            SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_WAIT;
            SALaneChangeInst.iLaneChangeTrRelObj = OBJ_INDEX_NO_OBJECT;
        } else {
            if (SALaneChangeInst.uiRelObjLaneStateTimer <
                FSW_RELOBJSTATE_TIMER_MAX) {
                SALaneChangeInst.uiRelObjLaneStateTimer++;
            }
            if (SALaneChangeInst.iLaneChangeTrRelObj != RelTrckObjNr) {
                SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_WAIT;
                SALaneChangeInst.iLaneChangeTrRelObj = OBJ_INDEX_NO_OBJECT;
            }

            /* Code above replaces former: trace code since traces were all
             * invalid due to no copy of  TRACE_VALID_LANEMATRIX in EM*/
            SIRelObjLaneStateChoose(RelTrckObjNr);
        }
    } else {
        SALaneChangeInst.eRelObjLaneState = FSW_RELOBJ_WAIT;
        SALaneChangeInst.iLaneChangeTrRelObj = OBJ_INDEX_NO_OBJECT;
    }

    /* calculate the probability of left lane change based on curvatrue and
     * threshold */
    SACheckLaneChangeState(&(SALaneChangeInst.LeftSide), -fCurveStandard,
                           -fGradientOfCurveStandard);

    /* calculate the probability of right lane change based on curvatrue and
     * threshold */
    SACheckLaneChangeState(&(SALaneChangeInst.RightSide), fCurveStandard,
                           fGradientOfCurveStandard);

    /* Recognize the correction state and age */
    if ((SALaneChangeInst.LeftSide.eLaneChangeState == FSW_ST_GERADELENKEN) &&
        (SALaneChangeInst.eRelObjLaneState == FSW_RELOBJ_AUSSPUR_R2)) {
        if (SALaneChangeInst.LeftSide.uiCorrection == iFSW_KNEUTRAL) {
            SALaneChangeInst.LeftSide.uiCorrection = iFSW_KANF;
        } else if (SALaneChangeInst.LeftSide.uiCorrection > iFSW_KEND) {
            SALaneChangeInst.LeftSide.uiCorrection--;
        } else {
        }
    } else {
        SALaneChangeInst.LeftSide.uiCorrection = iFSW_KNEUTRAL;
    }

    if ((SALaneChangeInst.RightSide.eLaneChangeState == FSW_ST_GERADELENKEN) &&
        (SALaneChangeInst.eRelObjLaneState == FSW_RELOBJ_AUSSPUR_L2)) {
        if (SALaneChangeInst.RightSide.uiCorrection == iFSW_KNEUTRAL) {
            SALaneChangeInst.RightSide.uiCorrection = iFSW_KANF;
        } else if (SALaneChangeInst.RightSide.uiCorrection > iFSW_KEND) {
            SALaneChangeInst.RightSide.uiCorrection--;
        } else {
        }
    } else {
        SALaneChangeInst.RightSide.uiCorrection = iFSW_KNEUTRAL;
    }

    /*! Calculation of lane change probability */
    SICalcLaneChangeProbability(VLC_CYCLE_TIME);

    /*! Determine if camera lane markers are crossed */
    SI_v_StateCamLaneCrossing(&(SICamLaneCrossedData.CamLaneMarkerCrossed),
                              &(SICamLaneCrossedData.LeftLaneMarkerLastCycle),
                              &(SICamLaneCrossedData.RightLaneMarkerLastCycle));

    /*! State if ego vehicle moves towards the new lane during the ego lane
      change,
      i.e. before entering the new lane */
    SI_v_SetLaneChangeMovePre(&SILaneChangeCamPreMoveState);

} /* end of EODetectLaneChange */

/*************************************************************************************************************************
  Functionname:    SIInitLCProbDataGlobal */
static void SIInitLCProbDataGlobal(void) {
    SILCProbDataGlobal.iTLLeftPrevious = 0;
    SILCProbDataGlobal.iTLRightPrevious = 0;

    SILCProbDataGlobal.iProbability = 0;
    SILCProbDataGlobal.iProbabilityBefore = 0;

    SILCProbDataGlobal.fDistLat = 0.f;
    SILCProbDataGlobal.fVelLat = 0.f;
    SILCProbDataGlobal.fCurveLevel = 0.f;
    SILCProbDataGlobal.fLCPsiBefore = 0.f;
    SILCProbDataGlobal.fLCPsiBeforeBuffer = 0.f;
    SILCProbDataGlobal.fFilteredEgoCurve = 0.f;
    SILCProbDataGlobal.fEgoCurvePT1Previous = 0.f;
    SILCProbDataGlobal.fDrvIntCurvePT1Previous = 0.f;

    SILCProbDataGlobal.afLatDiffFilteredCurvesFilt[0] = 0.f;
    SILCProbDataGlobal.afLatDiffFilteredCurvesFilt[1] = 0.f;
    SILCProbDataGlobal.afLatDiffCamCurveFilt[0] = 0.f;
    SILCProbDataGlobal.afLatDiffCamCurveFilt[1] = 0.f;

    SILCProbDataGlobal.fLCKinematicProb = 0.f;
    SILCProbDataGlobal.fLCProbLeft = 0.f;
    SILCProbDataGlobal.fLCProbRight = 0.f;
    SILCProbDataGlobal.fLCProbFollow = 0.f;

    SILCProbDataGlobal.LCTurnLightEvalLeft.State = SI_LC_TURN_LIGHT_NOT_ACTIVE;
    SILCProbDataGlobal.LCTurnLightEvalLeft.fTimeAfterTLActivation =
        SI_LC_PROB_MAX_TIME_AFTER_TURN_LIGHT_ACTIVE;
    SILCProbDataGlobal.LCTurnLightEvalRight.State = SI_LC_TURN_LIGHT_NOT_ACTIVE;
    SILCProbDataGlobal.LCTurnLightEvalRight.fTimeAfterTLActivation =
        SI_LC_PROB_MAX_TIME_AFTER_TURN_LIGHT_ACTIVE;
}

/*************************************************************************************************************************
  Functionname:    SIInitLCProbDataLocal */
static void SIInitLCProbDataLocal(const float32 fCycleTime,
                                  SILCProbDataLocal_t* pSILCProbDataLocal) {
    uint16 i;
    /*! Lane Change probability left/right */
    pSILCProbDataLocal->iProbabilityLaneChangeLeft = 0;
    pSILCProbDataLocal->iProbabilityLaneChangeRight = 0;

    /*! Initialize parameters which depend on the cycle time */
    pSILCProbDataLocal->DynPar.fEgoCurvePT1X = 1.f;
    if (fABS(SI_LC_PROB_EGO_CURVE_PT1_X + fCycleTime) > C_F32_DELTA) {
        pSILCProbDataLocal->DynPar.fEgoCurvePT1X =
            SI_LC_PROB_EGO_CURVE_PT1_X /
            (SI_LC_PROB_EGO_CURVE_PT1_X + fCycleTime);
    }

    for (i = 0u; i < (uint16)SI_LC_PROB_POINTS_DRIVER_INT_CURVE_FILTER_TABLE;
         i++) {
        pSILCProbDataLocal->DynPar.DriverIntCurveFilterConst[i].f0 =
            SILCProbParData.SI_LC_PROB_TABLE_DRIVER_INT_CURVE_FILTER_CONST[i]
                .f0;
        pSILCProbDataLocal->DynPar.DriverIntCurveFilterConst[i].f1 = 1.f;
        if (fABS(SILCProbParData
                     .SI_LC_PROB_TABLE_DRIVER_INT_CURVE_FILTER_CONST[i]
                     .f1 +
                 fCycleTime) > C_F32_DELTA) {
            pSILCProbDataLocal->DynPar.DriverIntCurveFilterConst[i].f1 =
                SILCProbParData
                    .SI_LC_PROB_TABLE_DRIVER_INT_CURVE_FILTER_CONST[i]
                    .f1 /
                (SILCProbParData
                     .SI_LC_PROB_TABLE_DRIVER_INT_CURVE_FILTER_CONST[i]
                     .f1 +
                 fCycleTime);
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIInitLCProbDetectors */
static void SIInitLCProbDetectors(SILCProbDetectors_t* pSILCProbDetectors) {
    pSILCProbDetectors->iLatDiffFilteredCurvesProb = 0u;
    pSILCProbDetectors->iLatDiffCamCurveProb = 0u;
    pSILCProbDetectors->iVelLatCamLaneMarkerProb = 0u;
    pSILCProbDetectors->iTurnLightProb = 0u;
    pSILCProbDetectors->iDistLatProb = 0u;
    pSILCProbDetectors->iCamLaneMarkerCrossedProb = 0u;
    pSILCProbDetectors->iCurveProb = 0u;
}

/*************************************************************************************************************************
  Functionname:    SIInitLCProbInputData */
static void SIInitLCProbInputData(SILCProbInputData_t* pSILCProbInputData) {
    pSILCProbInputData->fEgoCurvePT1 = 0.f;
    pSILCProbInputData->fDrvIntCurvePT1 = 0.f;
    pSILCProbInputData->fDistLat = 0.f;
    pSILCProbInputData->fCurveLevel = 0.f;
    pSILCProbInputData->CamLaneInfo.bCamLaneVisible = FALSE;
    pSILCProbInputData->CamLaneInfo.bCamLaneChangeLeft = FALSE;
    pSILCProbInputData->CamLaneInfo.bCamLaneChangeRight = FALSE;
    pSILCProbInputData->CamLaneInfo.fCameraCurvature = 0.f;
    pSILCProbInputData->CamLaneInfo.fLateralVelocityToCamLaneMarker = 0.f;
}

/*************************************************************************************************************************
  Functionname:    SILCProbSetTurnLightPhase */
static void SILCProbSetTurnLightPhase(const float32 fCycleTime,
                                      const SILCProbDirection_t LCProbDirection,
                                      const boolean bTLStatusActive,
                                      SILCProbTurnLightEval_t* pTLEval) {
    sint8 iTLStatus, iTLStatusChange;

    /*! Determine turn light status (integer value for calculation) */
    iTLStatus = (bTLStatusActive == TRUE) ? (sint8)1 : (sint8)0;

    /*! Determine turn light status change (integer value for calculation) */
    iTLStatusChange = iTLStatus - SILCProbDataGlobal.iTLLeftPrevious;

    /*! Set history buffer */
    if (LCProbDirection == LC_PROB_LEFT) {
        SILCProbDataGlobal.iTLLeftPrevious = iTLStatus;
    } else {
        SILCProbDataGlobal.iTLRightPrevious = iTLStatus;
    }

    /*! State machine: Set state and time after activation of turn light */
    switch (pTLEval->State) {
        /*! If turn light was ACTIVE in the last cycle */
        case SI_LC_TURN_LIGHT_ACTIVE:
            if ((iTLStatusChange < 0) ||
                (pTLEval->fTimeAfterTLActivation >
                 SI_LC_PROB_MAX_TIME_TURN_LIGHT_ON) ||
                (bTLStatusActive == FALSE)) {
                /* Transition from ACTIVE to NON_ACTIVE:
                    if turn light switched off
                    if turn light is on for a longer time than the maximum time
                   for considering a turn light on
                    if turn light is off */
                /*! Set timer to the maximum value for considering the turn
                 * light ACTIVE */
                pTLEval->fTimeAfterTLActivation =
                    SI_LC_PROB_MAX_TIME_TURN_LIGHT_ON;
                /*! Set state */
                pTLEval->State = SI_LC_TURN_LIGHT_NOT_ACTIVE;
            } else if (iTLStatusChange > 0) { /*! If turn light activated
                                                 although
                                                 SI_LC_TURN_LIGHT_ACTIVE phase
                                                 still active -> reset timer */
                pTLEval->fTimeAfterTLActivation = 0.f;
            } else {
                /*! Remain ACTIVE, increase timer (if below maximum value) */
                if (pTLEval->fTimeAfterTLActivation <
                    SI_LC_PROB_MAX_TIME_AFTER_TURN_LIGHT_ACTIVE) {
                    pTLEval->fTimeAfterTLActivation += fCycleTime;
                }
            }
            break;
        /*! If turn light was NOT ACTIVE in the last cycle */
        case SI_LC_TURN_LIGHT_NOT_ACTIVE:
            if ((iTLStatusChange > 0) && (bTLStatusActive == TRUE)) {
                /*! Transition from NON ACTIVE to ACTIVE
                    if turn light change indicates the activation
                    if turn light active */
                /*! Set timer, which indicates the time after activation of the
                 * turn light, to zero */
                pTLEval->fTimeAfterTLActivation = 0.f;
                /*! Set state */
                pTLEval->State = SI_LC_TURN_LIGHT_ACTIVE;
            } else if (iTLStatusChange < 0) {
                /*! Rest timer to maximum value if turn light change indicates
                 * the deactivation of the turn light although state already NON
                 * ACTIVE */
                pTLEval->fTimeAfterTLActivation =
                    MAX_FLOAT(SI_LC_PROB_MAX_TIME_TURN_LIGHT_ON,
                              pTLEval->fTimeAfterTLActivation);
            } else {
                /*! Remain INACTIVE, increase timer (if below maximum value) */
                if (pTLEval->fTimeAfterTLActivation <
                    SI_LC_PROB_MAX_TIME_AFTER_TURN_LIGHT_ACTIVE) {
                    pTLEval->fTimeAfterTLActivation += fCycleTime;
                }
            }
            break;
        default:
            /*! Set default values */
            pTLEval->State = SI_LC_TURN_LIGHT_NOT_ACTIVE;
            pTLEval->fTimeAfterTLActivation =
                SI_LC_PROB_MAX_TIME_AFTER_TURN_LIGHT_ACTIVE;
            break;
    }
}

/*************************************************************************************************************************
  Functionname:    SILCProbGetCamLaneInfo */
static void SILCProbGetCamLaneInfo(SILCProbCamLaneInfo_t* pCamLaneEval) {
    float32 fPsi;
    const float32 f_CamLaneVisibilityDist =
        FIP_f_GetVisibilityDistCamLane(); /*!< Visibility distance of camera
                                             lane */
    const float32 f_CamLaneC0 =
        FIP_f_GetCurveCamLane(); /*!< Curvature of camera lane */
    const float32 f_CamLaneAngle =
        FIP_f_GetHeadingAngleCamLane(); /*!< Heading angle of camera lane */

    /*! Set default value */
    fPsi = 0.f;

    if (f_CamLaneVisibilityDist > SI_LC_PROB_MIN_DIST_CAM_COURSE) {
        float32 fDeltaPsi;

        pCamLaneEval->fCameraCurvature = f_CamLaneC0;
        fPsi = -f_CamLaneAngle;

        /* Since course has a valid estimation length, set camera lane
         * visibility to TRUE */
        pCamLaneEval->bCamLaneVisible = TRUE;

        /*! Calculate the camera lane relative lateral speed using the course
           orientation angle and the ego speed
            -> lateral velocity towards camera lane markers */
        pCamLaneEval->fLateralVelocityToCamLaneMarker =
            SIN_HD_(fPsi) * EGO_SPEED_X_OBJ_SYNC;

        /* Check if we will cross the left lane marker within our prediction
         * time */
        pCamLaneEval->bCamLaneChangeLeft = FALSE;
        if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                 .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
            ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                  .f_MarkerDist -
              (pCamLaneEval->fLateralVelocityToCamLaneMarker *
               SI_LC_PROB_TIME_CAM_LANE_MARKER_CROSSED)) < 0.f)) {
            pCamLaneEval->bCamLaneChangeLeft = TRUE;
        }

        /* Check if we will cross the right lane marker within our prediction
         * time */
        pCamLaneEval->bCamLaneChangeRight = FALSE;
        if ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                 .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
            (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK) &&
            ((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                  .f_MarkerDist -
              (pCamLaneEval->fLateralVelocityToCamLaneMarker *
               SI_LC_PROB_TIME_CAM_LANE_MARKER_CROSSED)) > 0.f)) {
            pCamLaneEval->bCamLaneChangeRight = TRUE;
        }

        /* Update of psi buffer (only if change detected, since other cycle time
         * than VLC) */
        fDeltaPsi = SILCProbDataGlobal.fLCPsiBeforeBuffer - fPsi;
        if (fABS(fDeltaPsi) > C_F32_DELTA) {
            SILCProbDataGlobal.fLCPsiBefore =
                SILCProbDataGlobal.fLCPsiBeforeBuffer;
        }
        /* Check plausibility: Difference in angle is not allowed to be higher
         * than a threshold SI_LC_PROB_MAX_DIFF_PSI*/
        if (fABS(SILCProbDataGlobal.fLCPsiBefore - fPsi) >
            SI_LC_PROB_MAX_DIFF_PSI) {
            pCamLaneEval->bCamLaneChangeLeft = FALSE;
            pCamLaneEval->bCamLaneChangeRight = FALSE;
        }
    }

    SILCProbDataGlobal.fLCPsiBeforeBuffer = fPsi;
}

/*************************************************************************************************************************
  Functionname:    SILCProbGetEgoMotionInfo */
static void SILCProbGetEgoMotionInfo(
    const SILCProbDataLocal_t* pSILCProbDataLocal,
    float32* pfEgoCurvePT1,
    float32* pfDrvIntCurvePT1) {
    float32
        fPT1ConstDrvIntCurve; /*!< Filter constant for driver intended curve */

    /*! Filter ego curve */
    /*! Filter the ego curvature to mimic behavior of old ARS300 signals, where
     * the VDY yaw rate had a latency of approximately 200ms */
    SILCProbDataGlobal.fFilteredEgoCurve =
        GDB_FILTER(EGO_CURVE_OBJ_SYNC, SILCProbDataGlobal.fFilteredEgoCurve,
                   SI_LC_PROB_FILTER_CONST_EGO_CURVE);

    /*! Filter the ego curvature as done in ARS300 */
    (*pfEgoCurvePT1) = ((SILCProbDataGlobal.fEgoCurvePT1Previous -
                         SILCProbDataGlobal.fFilteredEgoCurve) *
                        pSILCProbDataLocal->DynPar.fEgoCurvePT1X) +
                       SILCProbDataGlobal.fFilteredEgoCurve;

    /*! Store result in history buffer */
    SILCProbDataGlobal.fEgoCurvePT1Previous = (*pfEgoCurvePT1);

    /*! Filter driver intended curve */
    /*! Get filter constant for driver intended curve based on driver intended
     * curve variance (Remark: EGO_DRV_INT_CURVE_VAR_OBJ_SYNC >= 0) */
    fPT1ConstDrvIntCurve = GDB_Math_CalculatePolygonValue(
        SI_LC_PROB_POINTS_DRIVER_INT_CURVE_FILTER_TABLE,
        pSILCProbDataLocal->DynPar.DriverIntCurveFilterConst,
        SQRT_(EGO_DRV_INT_CURVE_VAR_OBJ_SYNC));

    /*! Filter driver intended curve */
    (*pfDrvIntCurvePT1) = ((SILCProbDataGlobal.fDrvIntCurvePT1Previous -
                            EGO_DRV_INT_CURVE_OBJ_SYNC) *
                           fPT1ConstDrvIntCurve) +
                          EGO_DRV_INT_CURVE_OBJ_SYNC;

    /*! Store result in history buffer */
    SILCProbDataGlobal.fDrvIntCurvePT1Previous = (*pfDrvIntCurvePT1);
}

/*************************************************************************************************************************
  Functionname:    SILCProbGetDrivenDistVeloLateralDuringLC */
static void SILCProbGetDrivenDistVeloLateralDuringLC(
    const float32 fCycleTime,
    const float32 fEgoCurvePT1,
    const float32 fDrvIntCurvePT1,
    float32* pfDistLat) {
    float32 fDistCurveDiff;

    /*! Calculation of distance driven in lateral direction after a LC-maneuver
     * started */
    /*! Determine the difference in lateral distance between filtered ego curve
     * and filtered driver intended curve in 1s */
    if (((SILCProbDataGlobal.fVelLat >= 0.f) &&
         ((fDrvIntCurvePT1 - SILCProbDataGlobal.fCurveLevel) > 0.f)) ||
        ((SILCProbDataGlobal.fVelLat <= 0.f) &&
         ((fDrvIntCurvePT1 - SILCProbDataGlobal.fCurveLevel) < 0.f))) {
        fDistCurveDiff = fABS(0.5f * (fEgoCurvePT1 - fDrvIntCurvePT1) *
                              EGO_SPEED_X_OBJ_SYNC * EGO_SPEED_X_OBJ_SYNC *
                              SI_LC_PROB_DIFF_CURVE_FACTOR);
    } else {
        fDistCurveDiff = 0.f;
    }

    /*! Determine the lateral velocity/driven distance in lane change situation
        (difference between ego curve and driver intended curve higher than a
       threshold, fDistCurveDiff higher than a threshold,
         LC prob of last cycle higher than a threshold ); otherwise set values
       to zeros */
    if ((fABS((fDrvIntCurvePT1 - SILCProbDataGlobal.fCurveLevel)) >
         SI_LC_PROB_MIN_DIFF_CURVE_ANALYSE_DISTLAT) ||
        (fDistCurveDiff > SI_LC_PROB_MIN_DIST_DIFF_CURVE_ANALYSE_DISTLAT) ||
        (ABS(SILCProbDataGlobal.iProbabilityBefore) >
         (sint16)SI_LC_PROB_MIN_LCPROB_ANALYSE_DISTLAT)) {
        float32 fXSqr;
        /*! Determine the lateral velocity */
        SILCProbDataGlobal.fVelLat =
            (VLCSEN_pEgoDynRaw->Lateral.Accel.LatAccel * fCycleTime) +
            SILCProbDataGlobal.fVelLat;

        /*! Determine the driven distance in lateral direction */
        if (EGO_SPEED_X_OBJ_SYNC > C_F32_DELTA) {
            fXSqr = SQR(EGO_SPEED_X_OBJ_SYNC * fCycleTime);
            SILCProbDataGlobal.fDistLat += 0.5f * EGO_CURVE_OBJ_SYNC * fXSqr;
        }
    } else {
        SILCProbDataGlobal.fDistLat = 0.f;
        SILCProbDataGlobal.fVelLat = 0.f;
    }

    *pfDistLat = SILCProbDataGlobal.fDistLat;
}

static void SICamLaneCrossedProb(
    uint8 uiLCDirection,
    SILCProbInputData_t SILCProbInputOneSide,
    SILCProbDetectors_t* p_SILCProbDetectorsOneSide) {
    /* LC probability: camera lane crossed -> set LC probability to 100 if lane
     * change is detected */
    p_SILCProbDetectorsOneSide->iCamLaneMarkerCrossedProb = 0u;
    if (uiLCDirection == (uint8)LC_PROB_LEFT &&
        SILCProbInputOneSide.CamLaneInfo.bCamLaneChangeLeft == TRUE) {
        p_SILCProbDetectorsOneSide->iCamLaneMarkerCrossedProb = 100u;
    } else if (uiLCDirection == (uint8)LC_PROB_RIGHT &&
               SILCProbInputOneSide.CamLaneInfo.bCamLaneChangeRight == TRUE) {
        p_SILCProbDetectorsOneSide->iCamLaneMarkerCrossedProb = 100u;
    }
}

/*************************************************************************************************************************
  Functionname:    SICalcLaneChangeProbability */
static void SICalcLaneChangeProbability(const float32 fCycleTime) {
    boolean bTurnLightLeft, bTurnLightRight;
    uint8 uiLCDirection; /*!< Info about which lane change direction is
                            considered -> Initialization within a for loop */
    uint8 iLCProbTemp;
    float32 fTimeTurnLightPhase;
    float32 fLatDiffFilteredCurves, fLatDiffFilteredCurvesFilt,
        fLatDiffFilteredCurvesFiltLastCycle;
    float32 fLatDiffCamCurve, fLatDiffCamCurveFilt;
    float32 fABS_SILCProbInput_fDrvIntCurvePT1;

    /*! Input data for probability calculation (SILCProbInputOneSide ->
     * initialzation in the for loop later in the code) */
    SILCProbInputData_t SILCProbInput, SILCProbInputOneSide;
    /*! Lane Change probability values for the individual detectors for one side
     * (left or right) -> initialzation in the for loop later in the code */
    SILCProbDetectors_t SILCProbDetectorsOneSide;
    /*! Local LC probability data */
    SILCProbDataLocal_t SILCProbDataLocal;

    /*! Initialization of structures */
    SIInitLCProbInputData(&SILCProbInput);
    SIInitLCProbDataLocal(fCycleTime, &SILCProbDataLocal);

    /*! Set turn light state based on input data */
    bTurnLightLeft = FALSE;
    bTurnLightRight = FALSE;
    if (VLCSEN_pCustomInput->eTurnIndicator == eTurnIndicator_Left) {
        bTurnLightLeft = TRUE;
    }
    if (VLCSEN_pCustomInput->eTurnIndicator == eTurnIndicator_Right) {
        bTurnLightRight = TRUE;
    }

    /*! Determine lane change phase based on turn light information */
    SILCProbSetTurnLightPhase(fCycleTime, LC_PROB_LEFT, bTurnLightLeft,
                              &(SILCProbDataGlobal.LCTurnLightEvalLeft));
    SILCProbSetTurnLightPhase(fCycleTime, LC_PROB_RIGHT, bTurnLightRight,
                              &(SILCProbDataGlobal.LCTurnLightEvalRight));

    /*! Get camera lane information for lane change probability calculation */
    SILCProbGetCamLaneInfo(&(SILCProbInput.CamLaneInfo));

    /*! Filtering of ego curve and driver intended curve */
    SILCProbGetEgoMotionInfo(&SILCProbDataLocal, &(SILCProbInput.fEgoCurvePT1),
                             &(SILCProbInput.fDrvIntCurvePT1));

    /*! Calculation of "curve level": Should be zero on a straight road and
     * non-zero in a curve */
    /*! Adapt curve level only if not close to standstill or if not turning
        -> ego speed must be higher than a threshold, fDrvIntCurvePT1 must be
       lower than a threshold */
    fABS_SILCProbInput_fDrvIntCurvePT1 = fABS(SILCProbInput.fDrvIntCurvePT1);
    if ((EGO_SPEED_X_OBJ_SYNC > SI_LC_PROB_MIN_EGOVELO_SWCURVE_EVAL) &&
        (fABS_SILCProbInput_fDrvIntCurvePT1 <
         SI_LC_PROB_MAX_CURVE_CURVE_LEVEL)) {
        /*! Filter the curve over time */
        /*! If curve lower than a threshold and velocity low, consider the curve
         * as zeros */
        if ((fABS(SILCProbInput.fDrvIntCurvePT1) >
             SI_LC_PROB_MIN_CURVE_CURVE_LEVEL) ||
            (EGO_SPEED_X_OBJ_SYNC >= SI_LC_PROB_LOW_SPEED_LEVEL)) {
            SILCProbDataGlobal.fCurveLevel = GDB_FILTER(
                SILCProbInput.fDrvIntCurvePT1, SILCProbDataGlobal.fCurveLevel,
                SI_LC_PROB_FILTER_CONST_CURVE_LEVEL);
        } else {
            SILCProbDataGlobal.fCurveLevel =
                GDB_FILTER(0.f, SILCProbDataGlobal.fCurveLevel,
                           SI_LC_PROB_FILTER_CONST_CURVE_LEVEL);
        }
    } else {
        /* Leave fCurveLevel unchanged (freeze) */
    }
    SILCProbInput.fCurveLevel = SILCProbDataGlobal.fCurveLevel;

    /*! Calculation of distance driven in lateral direction after a LC-maneuver
     * started */
    SILCProbGetDrivenDistVeloLateralDuringLC(
        fCycleTime, SILCProbInput.fEgoCurvePT1, SILCProbInput.fDrvIntCurvePT1,
        &(SILCProbInput.fDistLat));

    /*! Firstly determine LC probability for lane change in left direction,
     * secondly for lane change in right direction */
    for (uiLCDirection = (uint8)LC_PROB_LEFT;
         uiLCDirection <= (uint8)LC_PROB_RIGHT; uiLCDirection++) {
        /*! Initialization of structures which are filled for each iteration of
         * loop */
        SIInitLCProbInputData(&SILCProbInputOneSide);
        SIInitLCProbDetectors(&SILCProbDetectorsOneSide);

        /*! Adapt signs of values based on LC side */
        if (uiLCDirection == (uint8)LC_PROB_LEFT) {
            SILCProbInputOneSide = SILCProbInput;
            fTimeTurnLightPhase =
                SILCProbDataGlobal.LCTurnLightEvalLeft.fTimeAfterTLActivation;
            /* lane change probability based on blinker should become zero if
             * blinker in the opposite direction was activated afterwards ->
             * Level is set*/
            if (SILCProbDataGlobal.LCTurnLightEvalLeft.fTimeAfterTLActivation >
                SILCProbDataGlobal.LCTurnLightEvalRight
                    .fTimeAfterTLActivation) {
                fTimeTurnLightPhase =
                    SI_LC_PROB_MAX_TIME_AFTER_TURN_LIGHT_ACTIVE;
            }
        } else {
            SILCProbInputOneSide.fEgoCurvePT1 = -SILCProbInput.fEgoCurvePT1;
            SILCProbInputOneSide.fDrvIntCurvePT1 =
                -SILCProbInput.fDrvIntCurvePT1;
            SILCProbInputOneSide.fDistLat = -SILCProbInput.fDistLat;
            SILCProbInputOneSide.fCurveLevel = -SILCProbInput.fCurveLevel;

            /* Camera relevant information (Remark: if no camera data available,
             * CamLaneInfo has initial values)*/
            SILCProbInputOneSide.CamLaneInfo.bCamLaneVisible =
                SILCProbInput.CamLaneInfo.bCamLaneVisible;
            SILCProbInputOneSide.CamLaneInfo.bCamLaneChangeLeft =
                SILCProbInput.CamLaneInfo.bCamLaneChangeLeft;
            SILCProbInputOneSide.CamLaneInfo.bCamLaneChangeRight =
                SILCProbInput.CamLaneInfo.bCamLaneChangeRight;

            SILCProbInputOneSide.CamLaneInfo.fCameraCurvature =
                -SILCProbInput.CamLaneInfo.fCameraCurvature;
            SILCProbInputOneSide.CamLaneInfo.fLateralVelocityToCamLaneMarker =
                -SILCProbInput.CamLaneInfo.fLateralVelocityToCamLaneMarker;

            fTimeTurnLightPhase =
                SILCProbDataGlobal.LCTurnLightEvalRight.fTimeAfterTLActivation;
            /* lane change probability based on blinker should become zero if
             * blinker in the opposite direction was activated afterwards ->
             * Level is set*/
            if (SILCProbDataGlobal.LCTurnLightEvalRight.fTimeAfterTLActivation >
                SILCProbDataGlobal.LCTurnLightEvalLeft.fTimeAfterTLActivation) {
                fTimeTurnLightPhase =
                    SI_LC_PROB_MAX_TIME_AFTER_TURN_LIGHT_ACTIVE;
            }
        }

        /*! LC probability: difference in lateral distance between filtered ego
         * curve and filtered driver intended curve */
        /*! Difference in lateral distance between filtered ego curve and
           filtered driver intended curve in 1s
            -> this difference is usually high before a lane change;
            SI_LC_PROB_DIFF_CURVE_FACTOR: Factor to increase the difference ->
           to reach 100 when LC */
        fLatDiffFilteredCurves = (-0.5f) *
                                 (SILCProbInputOneSide.fEgoCurvePT1 -
                                  SILCProbInputOneSide.fDrvIntCurvePT1) *
                                 EGO_SPEED_X_OBJ_SYNC * EGO_SPEED_X_OBJ_SYNC *
                                 SI_LC_PROB_DIFF_CURVE_FACTOR;

        /*! Filter difference in lateral distance between filtered ego curve and
         * filtered driver intended curve */
        fLatDiffFilteredCurvesFiltLastCycle =
            SILCProbDataGlobal.afLatDiffFilteredCurvesFilt[uiLCDirection];
        fLatDiffFilteredCurvesFilt = GDB_FILTER(
            fLatDiffFilteredCurves, fLatDiffFilteredCurvesFiltLastCycle,
            SI_LC_PROB_FILTER_CONST_DIFF_LAT_CURVE);
        SILCProbDataGlobal.afLatDiffFilteredCurvesFilt[uiLCDirection] =
            fLatDiffFilteredCurvesFilt;

        /*! Get lane change probability based on lookup table: Interpolate or
         * use border values */
        SILCProbDetectorsOneSide.iLatDiffFilteredCurvesProb =
            (uint8)ROUND_TO_INT(GDB_Math_CalculatePolygonValue(
                SI_LC_PROB_POINTS_DIFF_LAT_FILT_CURVES_TABLE,
                SILCProbParData.SI_LC_PROB_TABLE_DIFF_LAT_FILT_CURVES,
                fLatDiffFilteredCurvesFilt));

        /*! LC probability: difference in lateral distance between camera curve
         * and filtered driver intended curve */
        if (SILCProbInputOneSide.CamLaneInfo.bCamLaneVisible == TRUE) {
            /*! Difference in lateral distance between camera curve and filtered
              driver intended curve in 1s
              -> this difference is usually high before the camera lane markers
              are crossed */
            fLatDiffCamCurve =
                (-0.5f) *
                (SILCProbInputOneSide.CamLaneInfo.fCameraCurvature -
                 SILCProbInputOneSide.fDrvIntCurvePT1) *
                EGO_SPEED_X_OBJ_SYNC * EGO_SPEED_X_OBJ_SYNC;
        } else {
            fLatDiffCamCurve = 0.f;
        }
        /*! Filter: difference in lateral distance between camera curve and
         * filtered driver intended curve */
        fLatDiffCamCurveFilt =
            SILCProbDataGlobal.afLatDiffCamCurveFilt[uiLCDirection];
        fLatDiffCamCurve = GDB_FILTER(fLatDiffCamCurve, fLatDiffCamCurveFilt,
                                      SI_LC_PROB_FILTER_CONST_DIFF_LAT_CURVE);
        SILCProbDataGlobal.afLatDiffCamCurveFilt[uiLCDirection] =
            fLatDiffCamCurve;

        /*! Get lane change probability based on lookup table: Interpolate or
         * use border values */
        SILCProbDetectorsOneSide.iLatDiffCamCurveProb =
            (uint8)ROUND_TO_INT(GDB_Math_CalculatePolygonValue(
                SI_LC_PROB_POINTS_DIFF_LAT_FILT_CURVES_TABLE,
                SILCProbParData.SI_LC_PROB_TABLE_DIFF_LAT_FILT_CURVES,
                fLatDiffCamCurve));

        /*! LC probability: lateral velocity towards camera lane markers */
        /*! Get lane change probability based on lookup table: Interpolate or
         * use border values */
        SILCProbDetectorsOneSide.iVelLatCamLaneMarkerProb =
            (uint8)ROUND_TO_INT(GDB_Math_CalculatePolygonValue(
                SI_LC_PROB_POINTS_LAT_VEL_CAM_LANE_MARKER_TABLE,
                SILCProbParData.SI_LC_PROB_TABLE_LAT_VEL_CAM_LANE_MARKER,
                SILCProbInputOneSide.CamLaneInfo
                    .fLateralVelocityToCamLaneMarker));

        /*! LC probability: turn lights */
        /*! Get lane change probability based on lookup table: Interpolate or
         * use border values */
        SILCProbDetectorsOneSide.iTurnLightProb =
            (uint8)ROUND_TO_INT(GDB_Math_CalculatePolygonValue(
                SI_LC_PROB_POINTS_TURN_LIGHT_TABLE,
                SILCProbParData.SI_LC_PROB_TABLE_TURN_LIGHT,
                fTimeTurnLightPhase));

        /*! LC probability: driven distance in lateral direction */
        /*! Get lane change probability based on lookup table: Interpolate or
         * use border values */
        SILCProbDetectorsOneSide.iDistLatProb =
            (uint8)ROUND_TO_INT(GDB_Math_CalculatePolygonValue(
                SI_LC_PROB_POINTS_LAT_DIST_TABLE,
                SILCProbParData.SI_LC_PROB_TABLE_LAT_DIST,
                SILCProbInputOneSide.fDistLat));

        /* LC probability: camera lane crossed -> set LC probability to 100 if
         * lane change is detected */
        SICamLaneCrossedProb(uiLCDirection, SILCProbInputOneSide,
                             &SILCProbDetectorsOneSide);

        /*! LC probability: Curve */
        /*! Get lane change probability based on lookup table: Interpolate or
         * use border values */
        SILCProbDetectorsOneSide.iCurveProb =
            (uint8)ROUND_TO_INT(GDB_Math_CalculatePolygonValue(
                SI_LC_PROB_POINTS_CURVE_TABLE,
                SILCProbParData.SI_LC_PROB_TABLE_CURVE,
                (SILCProbInputOneSide.fDrvIntCurvePT1 -
                 SILCProbInputOneSide.fCurveLevel)));

        /*! Combine lane change probabilities of the individual detectors
         * (different parameters for "high speed" and "low speed") */
        if (EGO_SPEED_X_OBJ_SYNC > SI_LC_PROB_LOW_SPEED_LEVEL) {
            /*! If camera information is available */
            if (SILCProbInputOneSide.CamLaneInfo.bCamLaneVisible == TRUE) {
                iLCProbTemp = CML_Bayes2(
                    SILCProbDetectorsOneSide.iLatDiffCamCurveProb,
                    SILCProbDetectorsOneSide.iVelLatCamLaneMarkerProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_CAM_PROB); /*!< Camera based
                                                             probabilities */
                iLCProbTemp = BML_Bayes3(
                    SILCProbDetectorsOneSide.iLatDiffFilteredCurvesProb,
                    iLCProbTemp, SILCProbDetectorsOneSide.iTurnLightProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_ALL_LATDIFFCURVE_TURNLIGHT); /*<
                                                                               iLatDiffFilteredCurvesProb, iLCProbTemp, iTurnLightProb */
            } else {
                /*! If no camera information is available */
                iLCProbTemp = CML_Bayes2(
                    SILCProbDetectorsOneSide.iLatDiffFilteredCurvesProb,
                    SILCProbDetectorsOneSide.iTurnLightProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_LATDIFFCURVE_TURNLIGHT); /*<
                                                                           iLatDiffFilteredCurvesProb,
                                                                           iTurnLightProb
                                                                           */
            }

            /*! If camera information is available */
            if (SILCProbInputOneSide.CamLaneInfo.bCamLaneVisible == TRUE) {
                iLCProbTemp = CML_Bayes2(
                    iLCProbTemp,
                    SILCProbDetectorsOneSide.iCamLaneMarkerCrossedProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_MARKERCROSSING); /*!< iLCProbTemp
                                                                   and detected
                                                                   crossing of
                                                                   lane markers
                                                                   */
            }
        } else {
            /*! If camera information is available */
            if (SILCProbInputOneSide.CamLaneInfo.bCamLaneVisible == TRUE) {
                iLCProbTemp = CML_Bayes2(
                    SILCProbDetectorsOneSide.iLatDiffCamCurveProb,
                    SILCProbDetectorsOneSide.iVelLatCamLaneMarkerProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_CAM_PROB); /*!< Camera based
                                                             probabilities */
                iLCProbTemp = CML_Bayes2(
                    iLCProbTemp, SILCProbDetectorsOneSide.iTurnLightProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_LOWSPEED_TURNLIGHT); /*!< Turn
                                                                       light */
                iLCProbTemp = BML_Bayes3(
                    iLCProbTemp,
                    SILCProbDetectorsOneSide.iLatDiffFilteredCurvesProb,
                    SILCProbDetectorsOneSide.iDistLatProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_LATDIST_LATDIFFCURVE); /*!<
                                                                         iLatDiffFilteredCurvesProb
                                                                         and
                                                                         distance
                                                                         in
                                                                         lateral
                                                                         direction
                                                                         */
                iLCProbTemp = CML_Bayes2(
                    iLCProbTemp, SILCProbDetectorsOneSide.iCurveProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_CURVE); /*!< Curvature */
                iLCProbTemp = CML_Bayes2(
                    iLCProbTemp,
                    SILCProbDetectorsOneSide.iCamLaneMarkerCrossedProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_MARKERCROSSING); /*!< iLCProbTemp
                                                                   and detected
                                                                   crossing of
                                                                   lane markers
                                                                   */
            } else {
                /*! If no camera information is available */
                iLCProbTemp = BML_Bayes3(
                    SILCProbDetectorsOneSide.iDistLatProb,
                    SILCProbDetectorsOneSide.iLatDiffFilteredCurvesProb,
                    SILCProbDetectorsOneSide.iTurnLightProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_LATDIST_TURNLIGHT_LATDIFFCURVE); /*!< iDistLatProb, iLatDiffFilteredCurvesProb, iTurnLightProb */
                iLCProbTemp = CML_Bayes2(
                    iLCProbTemp, SILCProbDetectorsOneSide.iCurveProb,
                    SILCProbParData
                        .SI_LC_PROB_TABLE_COMB_CURVE); /*!< Curvature */
            }
        }

        /*! Store results */
        if (uiLCDirection == (uint8)LC_PROB_LEFT) {
            SILCProbDataLocal.iProbabilityLaneChangeLeft = (sint16)iLCProbTemp;
        } else {
            SILCProbDataLocal.iProbabilityLaneChangeRight = (sint16)iLCProbTemp;
        }
    } /*!< End: for(SILCDirection = LC_PROB_LEFT; SILCDirection <=
         LC_PROB_RIGHT; SILCDirection++) */

    /*! Get one lane change probability for left and right
        -> Lane change LEFT: [0, 100]; lane change RIGHT: [-100, 0] */
    SILCProbDataGlobal.iProbability =
        SILCProbDataLocal.iProbabilityLaneChangeLeft -
        SILCProbDataLocal.iProbabilityLaneChangeRight;

    /*! Store result in history buffer */
    SILCProbDataGlobal.iProbabilityBefore = SILCProbDataGlobal.iProbability;
}

/*************************************************************************************************************************
  Functionname:    SILCGetLaneChangeProbability */
sint16 SILCGetLaneChangeProbability(void) {
    return SILCProbDataGlobal.iProbability;
}

/*************************************************************************************************************************
  Functionname:    SI_v_InitGlobalCameraLaneCrossingData */
static void SI_v_InitGlobalCameraLaneCrossingData(void) {
    /*! Camera lane crossing state */
    SICamLaneCrossedData.CamLaneMarkerCrossed = UNKNOWN_CROSS_CAMLANE;
    /*! History data for left side */
    SICamLaneCrossedData.LeftLaneMarkerLastCycle.b_CrossingLeft = FALSE;
    SICamLaneCrossedData.LeftLaneMarkerLastCycle.b_CrossingRight = FALSE;
    SICamLaneCrossedData.LeftLaneMarkerLastCycle.f_Dist = 0.f;
    SICamLaneCrossedData.LeftLaneMarkerLastCycle.LaneMarkerPoE = 0u;
    /*! History data for right side */
    SICamLaneCrossedData.RightLaneMarkerLastCycle.b_CrossingLeft = FALSE;
    SICamLaneCrossedData.RightLaneMarkerLastCycle.b_CrossingRight = FALSE;
    SICamLaneCrossedData.RightLaneMarkerLastCycle.f_Dist = 0.f;
    SICamLaneCrossedData.RightLaneMarkerLastCycle.LaneMarkerPoE = 0u;
}

/*************************************************************************************************************************
  Functionname:    SI_b_GetValidCamLane */
static boolean SI_b_GetValidCamLane(uint32 u_LaneMarkerPoELastCycle,
                                    uint32 u_LaneMarkerPoECurrentCycle) {
    boolean b_LaneMarkerValid;

    /*! Set default: Camera lane marker is invalid */
    b_LaneMarkerValid = FALSE;

    /*! Consider the camera lane marker state as valid if the state is valid for
     * the last and the current cycle */
    if ((u_LaneMarkerPoELastCycle >= FIP_CAM_LANE_POE_LEVEL) &&
        (u_LaneMarkerPoECurrentCycle >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->sSigHeader.eSigStatus == AL_SIG_STATE_OK)) {
        b_LaneMarkerValid = TRUE;
    }

    return b_LaneMarkerValid;
}

/*************************************************************************************************************************
  Functionname:    SI_v_DetectCamLaneCrossing */
static void SI_v_DetectCamLaneCrossing(
    boolean* pb_LCLeftMarkerLeft,
    boolean* pb_LCLeftMarkerRight,
    boolean* pb_LCRightMarkerLeft,
    boolean* pb_LCRightMarkerRight,
    const boolean b_LaneMarkerLeftValid,
    const boolean b_LaneMarkerRightValid,
    const t_SILCCamLaneMarkerHistData* p_LeftLaneMarkerLastCycle,
    const t_SILCCamLaneMarkerHistData* p_RightLaneMarkerLastCycle) {
    /*! Initialize output values */
    (*pb_LCLeftMarkerLeft) = FALSE;   /*!< Ego vehicle crossing the left lane
                                         marker into to left direction */
    (*pb_LCRightMarkerLeft) = FALSE;  /*!< Ego vehicle crossing the left lane
                                         marker into to right direction */
    (*pb_LCLeftMarkerRight) = FALSE;  /*!< Ego vehicle crossing the right lane
                                         marker into to left direction */
    (*pb_LCRightMarkerRight) = FALSE; /*!< Ego vehicle crossing the right lane
                                         marker into to right direction */

    /*! Detect the crossing of left camera lane marker */
    if (b_LaneMarkerLeftValid == TRUE) {
        /*! Consider it a crossing into the left direction, if the difference in
        the distance to the camera lane markers between the current and
        the last cycle is higher than a threshold;
        and if the distance to the left camera lane marker was within an
        interval for the last cycle */
        if (((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                  .f_MarkerDist -
              p_LeftLaneMarkerLastCycle->f_Dist) >
             SI_LC_MIN_DIFF_DIST_CAM_LANE_MARKER) &&
            (fABS(p_LeftLaneMarkerLastCycle->f_Dist) <
             SI_LC_MIN_MAX_INTERVAL_DIST_CAM_LANE_MARKER)) {
            (*pb_LCLeftMarkerLeft) = TRUE;
        } else if (((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                         .f_MarkerDist -
                     p_LeftLaneMarkerLastCycle->f_Dist) <
                    -SI_LC_MIN_DIFF_DIST_CAM_LANE_MARKER) &&
                   (fABS(
                        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
                            .f_MarkerDist) <
                    SI_LC_MIN_MAX_INTERVAL_DIST_CAM_LANE_MARKER)) {
            /*! Consider it a crossing into the right direction, if the
            difference in the distance to the camera lane markers between the
            current and
            the last cycle is lower than a threshold;
            and if the distance to the left camera lane marker is within an
            interval for the current cycle */
            (*pb_LCRightMarkerLeft) = TRUE;
        } else {
            /*! Nothing */
        }
    }

    /*! Detect the crossing of right camera lane marker */
    if (b_LaneMarkerRightValid == TRUE) {
        /*! Consider it a crossing into the left direction, if the difference in
        the distance to the camera lane markers between the current and
        the last cycle is higher than a threshold;
        and if the distance to the right camera lane marker was within an
        interval for the last cycle */
        if (((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                  .f_MarkerDist -
              p_RightLaneMarkerLastCycle->f_Dist) >
             SI_LC_MIN_DIFF_DIST_CAM_LANE_MARKER) &&
            (fABS(VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                      .f_MarkerDist) <
             SI_LC_MIN_MAX_INTERVAL_DIST_CAM_LANE_MARKER)) {
            (*pb_LCLeftMarkerRight) = TRUE;
        } else if (((VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
                         .f_MarkerDist -
                     p_RightLaneMarkerLastCycle->f_Dist) <
                    -SI_LC_MIN_DIFF_DIST_CAM_LANE_MARKER) &&
                   (fABS(p_RightLaneMarkerLastCycle->f_Dist) <
                    SI_LC_MIN_MAX_INTERVAL_DIST_CAM_LANE_MARKER)) {
            /*! Consider it a crossing into the right direction, if the
            difference in the distance to the camera lane markers between the
            current and
            the last cycle is lower than a threshold;
            and if the distance to the right camera lane marker is within an
            interval for the current cycle */
            (*pb_LCRightMarkerRight) = TRUE;
        } else {
            /*! Nothing */
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SI_v_StateCamLaneCrossing */
static void SI_v_StateCamLaneCrossing(
    t_SILCStateCamLaneMarkerCrossed* p_StateCamLaneMarkerCrossed,
    t_SILCCamLaneMarkerHistData* p_LeftLaneMarkerLastCycle,
    t_SILCCamLaneMarkerHistData* p_RightLaneMarkerLastCycle) {
    boolean b_LaneMarkerLeftValid, b_LaneMarkerRightValid;
    boolean b_LCLeftMarkerLeft, b_LCLeftMarkerRight, b_LCRightMarkerLeft,
        b_LCRightMarkerRight;
    boolean b_LCLeftMarkerLeftAll, b_LCRightMarkerLeftAll,
        b_LCLeftMarkerRightAll, b_LCRightMarkerRightAll;

    /*! Determine if the left camera lane marker is valid */
    b_LaneMarkerLeftValid = SI_b_GetValidCamLane(
        p_LeftLaneMarkerLastCycle->LaneMarkerPoE,
        (uint32)VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
            .u_ExistanceProbability);

    /*! Determine if the right camera lane marker is valid */
    b_LaneMarkerRightValid = SI_b_GetValidCamLane(
        p_RightLaneMarkerLastCycle->LaneMarkerPoE,
        (uint32)VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
            .u_ExistanceProbability);

    /*! Determine if the camera lane markers are crossed into the left/right
     * direction based on the analysis of the left and/or right camera lane
     * markers */
    SI_v_DetectCamLaneCrossing(
        &b_LCLeftMarkerLeft, &b_LCLeftMarkerRight, &b_LCRightMarkerLeft,
        &b_LCRightMarkerRight, b_LaneMarkerLeftValid, b_LaneMarkerRightValid,
        p_LeftLaneMarkerLastCycle, p_RightLaneMarkerLastCycle);

    /*! Set camera lane marker crossing state */
    /*! Default: UNKNOWN_CROSS_CAMLANE */
    (*p_StateCamLaneMarkerCrossed) = UNKNOWN_CROSS_CAMLANE;

    if ((b_LaneMarkerLeftValid == TRUE) && (b_LaneMarkerRightValid == TRUE)) {
        /*! If the left and right camera lane markers are valid, both markers
         * have to indicate the crossing */

        /*! Combine information of detected camera lane crossing from the last
         * cycle and the current cycle */
        /*! Default values */
        b_LCLeftMarkerLeftAll = FALSE;
        b_LCRightMarkerLeftAll = FALSE;
        b_LCLeftMarkerRightAll = FALSE;
        b_LCRightMarkerRightAll = FALSE;
        if ((b_LCLeftMarkerLeft == TRUE) ||
            (p_LeftLaneMarkerLastCycle->b_CrossingLeft == TRUE)) {
            b_LCLeftMarkerLeftAll = TRUE;
        }
        if ((b_LCRightMarkerLeft == TRUE) ||
            (p_LeftLaneMarkerLastCycle->b_CrossingRight == TRUE)) {
            b_LCRightMarkerLeftAll = TRUE;
        }
        if ((b_LCLeftMarkerRight == TRUE) ||
            (p_RightLaneMarkerLastCycle->b_CrossingLeft == TRUE)) {
            b_LCLeftMarkerRightAll = TRUE;
        }
        if ((b_LCRightMarkerRight == TRUE) ||
            (p_RightLaneMarkerLastCycle->b_CrossingRight == TRUE)) {
            b_LCRightMarkerRightAll = TRUE;
        }

        /*! The left and right camera lane markers indicate a crossing in the
         * left direction */
        if ((b_LCLeftMarkerLeftAll == TRUE) &&
            (b_LCLeftMarkerRightAll == TRUE)) {
            (*p_StateCamLaneMarkerCrossed) = LEFT_TWO_LANE_CONF_CROSS_CAMLANE;
        } else if ((b_LCRightMarkerLeftAll == TRUE) &&
                   (b_LCRightMarkerRightAll == TRUE)) {
            /*! The left and right camera lane markers indicate a crossing in
             * the right direction */
            (*p_StateCamLaneMarkerCrossed) = RIGHT_TWO_LANE_CONF_CROSS_CAMLANE;
        } else {
            /*! Nothing */
        }
    } else if ((b_LaneMarkerLeftValid == TRUE) &&
               (b_LCLeftMarkerLeft == TRUE)) {
        /*! Only the left camera lane marker is valid and the left camera lane
         * marker indicates the crossing into the left direction */
        (*p_StateCamLaneMarkerCrossed) = LEFT_ONE_LANE_CONF_CROSS_CAMLANE;
    } else if ((b_LaneMarkerRightValid == TRUE) &&
               (b_LCRightMarkerRight == TRUE)) {
        /*! Only the right camera lane marker is valid and the right camera lane
         * marker indicates the crossing into the right direction */
        (*p_StateCamLaneMarkerCrossed) = RIGHT_ONE_LANE_CONF_CROSS_CAMLANE;
    } else {
        /*! Nothing */
    }

    /*! Update the global data with the information from this cycle */
    p_LeftLaneMarkerLastCycle->b_CrossingLeft = b_LCLeftMarkerLeft;
    p_LeftLaneMarkerLastCycle->b_CrossingRight = b_LCRightMarkerLeft;
    p_LeftLaneMarkerLastCycle->f_Dist =
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT].f_MarkerDist;
    p_LeftLaneMarkerLastCycle->LaneMarkerPoE =
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
            .u_ExistanceProbability;
    p_RightLaneMarkerLastCycle->b_CrossingLeft = b_LCLeftMarkerRight;
    p_RightLaneMarkerLastCycle->b_CrossingRight = b_LCRightMarkerRight;
    p_RightLaneMarkerLastCycle->f_Dist =
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT].f_MarkerDist;
    p_RightLaneMarkerLastCycle->LaneMarkerPoE =
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
            .u_ExistanceProbability;
}

/*************************************************************************************************************************
  Functionname:    SI_t_GetCamLaneMarkerCrossed */
t_SILCStateCamLaneMarkerCrossed SI_t_GetCamLaneMarkerCrossed(void) {
    return SICamLaneCrossedData.CamLaneMarkerCrossed;
}

/*************************************************************************************************************************
  Functionname:    SI_v_SetLaneChangeMovePre */
static void SI_v_SetLaneChangeMovePre(
    t_SILaneChangeCamPreMove* p_SILaneChangeCamPreMoveState) {
    uint8 i;
    boolean b_DrvLeftCamLane, b_DrvRightCamLane;
    sint16 s_LCProb;
    float32 f_DistCamLaneMarkerLeft, f_DistCamLaneMarkerRight,
        f_AbsDistCamLaneMarkerLeft, f_AbsDistCamLaneMarkerRight, f_CamLaneWidth;
    float32 f_SumDistDrivenLeft, f_SumDistDrivenRight;
    float32 f_CamLaneVisibilityDist;

    /*! Set default: No lane change */
    *p_SILaneChangeCamPreMoveState = LANE_CHANGE_CAM_PRE_MOVE_NO;

    /*! Store distance to camera lane markers, width of the camera lane and
        visibility distance of the camera lane in a local variable */
    f_DistCamLaneMarkerLeft =
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT].f_MarkerDist;
    f_DistCamLaneMarkerRight =
        VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT].f_MarkerDist;
    f_AbsDistCamLaneMarkerLeft = fABS(f_DistCamLaneMarkerLeft);
    f_AbsDistCamLaneMarkerRight = fABS(f_DistCamLaneMarkerRight);
    f_CamLaneWidth = FIP_f_GetWidthCamLane(); /*!< Width of camera lane */
    f_CamLaneVisibilityDist =
        FIP_f_GetVisibilityDistCamLane(); /*!< Visibility distance of camera
                                             lane */

    /*! Fill camera lane history buffer */
    /*! First: Move entries for left and right buffer */
    for (i = SI_LC_CAM_LANE_MARKER_HISTORY_BUFFER - 1u; i >= 1u; i--) {
        SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i] =
            SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i - 1u];
        SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i] =
            SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i - 1u];
    }

    /*! Second: Set the new value at the 0-th entry -> If the state is invalid,
     * set the default value */
    /*! Left side */
    if ((f_CamLaneVisibilityDist > C_F32_DELTA) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL)) {
        SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[0u] =
            f_DistCamLaneMarkerLeft;
    } else {
        SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[0u] =
            SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST;
    }
    /*! Right side */
    if ((f_CamLaneVisibilityDist > C_F32_DELTA) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL)) {
        SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[0u] =
            f_DistCamLaneMarkerRight;
    } else {
        SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[0u] =
            SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST;
    }

    /*! Determine if the ego vehicle is driving to the left or right side based
       on the distance to the camera lane markers
        -> the ego vehicle is considered to drive to one side, if all entries in
       the histroy buffer indicate this
        (i.e. the distance to the lane markers must become higher or lower for
       all entries in the history buffer) */
    b_DrvLeftCamLane = TRUE;
    b_DrvRightCamLane = TRUE;
    f_SumDistDrivenLeft = 0.f;
    f_SumDistDrivenRight = 0.f;
    for (i = 0u; i < (SI_LC_CAM_LANE_MARKER_HISTORY_BUFFER - 1u); i++) {
        /*! Determine if the ego vehicle is driving to the left */
        if ((SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i] >
             (SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST - C_F32_DELTA)) ||
            (SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i + 1u] >
             (SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST - C_F32_DELTA)) ||
            ((SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i] -
              SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i + 1u]) >
             C_F32_DELTA)) {
            b_DrvLeftCamLane = FALSE;
        } else {
            f_SumDistDrivenLeft +=
                (SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i + 1u] -
                 SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i]);
        }
        /*! Determine if the ego vehicle is driving to the right */
        if ((SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i] >
             (SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST - C_F32_DELTA)) ||
            (SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i + 1u] >
             (SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST - C_F32_DELTA)) ||
            ((SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i] -
              SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i + 1u]) <
             -C_F32_DELTA)) {
            b_DrvRightCamLane = FALSE;
        } else {
            f_SumDistDrivenRight +=
                (SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i] -
                 SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i + 1u]);
        }
    }

    /*! Reset results if the driven distance to the left/right side is not
     * sufficient high or too high (camera lane marker are crossed)*/
    if ((b_DrvLeftCamLane == TRUE) &&
        ((f_SumDistDrivenLeft < SI_LC_CAM_MOVE_PRE_MIN_DIVENDIST_TO_SIDE) ||
         (f_SumDistDrivenLeft > SI_LC_CAM_MOVE_PRE_MAX_DIVENDIST_TO_SIDE))) {
        b_DrvLeftCamLane = FALSE;
    }
    if ((b_DrvRightCamLane == TRUE) &&
        ((f_SumDistDrivenRight < SI_LC_CAM_MOVE_PRE_MIN_DIVENDIST_TO_SIDE) ||
         (f_SumDistDrivenLeft > SI_LC_CAM_MOVE_PRE_MAX_DIVENDIST_TO_SIDE))) {
        b_DrvRightCamLane = FALSE;
    }

    /*! Reset results due to implausibility */
    if ((b_DrvLeftCamLane == TRUE) && (b_DrvRightCamLane == TRUE)) {
        b_DrvLeftCamLane = FALSE;
        b_DrvRightCamLane = FALSE;
    }

    /*! Determine the lane change direction (only if the lane marker information
       are reliable
        -> valid lane markers, visibility distance higher than a threshold,
       valid camera lane width) */
    if ((f_CamLaneVisibilityDist > C_F32_DELTA) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_LEFT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (VLCSEN_pCamLaneData->LaneMarkerInfo[CL_CAM_LANE_MK_RIGHT]
             .u_ExistanceProbability >= FIP_CAM_LANE_POE_LEVEL) &&
        (f_CamLaneWidth > C_F32_DELTA)) {
        /*! Get the lane change probability */
        s_LCProb = SILCGetLaneChangeProbability();

        /*! Lane Change left: Considered only until the lane marker is crossed
           and when the ego vehicle is still
            at the left side of the lane (fABS(f_DistCamLaneMarkerLeft) <
           fABS(f_DistCamLaneMarkerRight));
            Situation after the lane marker is crossed is handled by the
           function FIP_v_LaneMatrixFilteringOneSide(...) */
        if ((f_AbsDistCamLaneMarkerLeft < f_AbsDistCamLaneMarkerRight) &&
            (b_DrvLeftCamLane == TRUE) && (s_LCProb > 0) &&
            ((s_LCProb > SI_LC_CAM_MOVE_PRE_LM_MIN_LC_PROB) ||
             (f_DistCamLaneMarkerLeft <
              SI_LC_CAM_MOVE_PRE_MAX_DIST_LANE_MARKER_LC))) {
            *p_SILaneChangeCamPreMoveState = LANE_CHANGE_CAM_PRE_MOVE_LEFT;
        }

        /*! Lane Change right: Considered only until the lane marker is crossed
        and when the ego vehicle is still
        at the right side of the lane (fABS(f_DistCamLaneMarkerRight) <
        fABS(f_DistCamLaneMarkerLeft));
        Situation after the lane marker is crossed is handled by the function
        FIP_v_LaneMatrixFilteringOneSide(...) */
        if ((f_AbsDistCamLaneMarkerRight < f_AbsDistCamLaneMarkerLeft) &&
            (b_DrvRightCamLane == TRUE) && (s_LCProb < 0) &&
            ((s_LCProb < -SI_LC_CAM_MOVE_PRE_LM_MIN_LC_PROB) ||
             (f_DistCamLaneMarkerRight >
              -SI_LC_CAM_MOVE_PRE_MAX_DIST_LANE_MARKER_LC))) {
            *p_SILaneChangeCamPreMoveState = LANE_CHANGE_CAM_PRE_MOVE_RIGHT;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SI_v_InitGlobLCCamLanePreMove */
static void SI_v_InitGlobLCCamLanePreMove(void) {
    uint8 i;
    /* Initialize camera lane marker distance history buffer */
    for (i = 0u; i < SI_LC_CAM_LANE_MARKER_HISTORY_BUFFER; i++) {
        SILCCamMovePreGlobalData.af_LeftCamMarkerDistHistory[i] =
            SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST;
        SILCCamMovePreGlobalData.af_RightCamMarkerDistHistory[i] =
            SI_LC_CAM_DEFAULT_CAM_LANE_MARKER_DIST;
    }

    /* Initialize SILaneChangeCamPreMoveState */
    SILaneChangeCamPreMoveState = LANE_CHANGE_CAM_PRE_MOVE_NO;
}

/*************************************************************************************************************************
  Functionname:    SI_t_GetLaneChangeMovePre */
t_SILaneChangeCamPreMove SI_t_GetLaneChangeMovePre(void) {
    return SILaneChangeCamPreMoveState;
}

/*****************************************************************************
  UNDEF MODULE LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */