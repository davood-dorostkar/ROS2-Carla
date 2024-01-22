

#ifndef _SI_CORRIDOR_CRIT_EXT_INCLUDED
#define _SI_CORRIDOR_CRIT_EXT_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cp_ext.h"
#include "si_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMBOLISCHE KONSTANTEN (KOMPONENTENINTERN)
*****************************************************************************/

/*****************************************************************************
  SYMBOLISCHE KONSTANTEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  Schnittstellen defines
*****************************************************************************/

#define SI_SET_OBJ_TRAJ_REF_POINT(iObj, pRefPoint) \
    ((OBJ_GET_SI(iObj).ObjTrajRefPoint) =          \
         (pRefPoint)) /*TPObject[iObj].SI.ObjTrajRefPoint = pRefPoint*/
#define SI_SET_OBJ_TRAJ_REF_POINT_LAST(iObj, pRefPoint)              \
    ((OBJ_GET_SI(iObj).ObjTrajRefPointLastCycle) =                   \
         (pRefPoint)) /*TPObject[iObj].SI.ObjTrajRefPointLastCycle = \
                         pRefPoint*/

/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
OLD SA
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*-- SIAdReOb.c --*/

/*-- SIRelTra.c --*/
#define OWVKRIT_AKTIV (1L)
#define OWVKRIT_INAKTIV (0L)

/*! Flag to disable specific corridor bracket functions */
#define SI_CORR_BRACKET_FUNC_ENABLED (TRUE)
#define SI_CORR_BRACKET_FUNC_DISABLED (FALSE)

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
OLD SA
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/*****************************************************************************
  MACROS (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS (KOMPONENTENEXTERN)
*****************************************************************************/
#ifndef INITVALUE_BRACKETPOSITION
#define INITVALUE_BRACKETPOSITION (999.F)
#endif
#define SI_BRACKETPOS_VALID_VAL_COMPARE (0.5f * INITVALUE_BRACKETPOSITION)

/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
OLD SA
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/
/*----------------------------------------------------------------------------
  Objekt-Strukturen Situationanalysis
----------------------------------------------------------------------------*/
typedef struct {
    float32 fLatTrackLimitExpandFac; /*!< Factor for widening seek lane width
                                        to relevant obj track lane width
                                        depending on time and distance @min:0
                                        @max:1 */
    float32
        fLatTrackLimitDistanceExpandFac; /*!< Factor for widening seek lane
                                     width to relevant object track lane width
                                     depending on distance difference @min:0
                                     @max:1 */

    sint32
        iLwFolge; /*!< Counter flag for object being held over steering speed */

    float32 fLatTrackLimitL; /*!< Maximal allowed deviation to left from
                                     ideal trajectory @unit:m */
    float32 fLatTrackLimitR; /*!< Maximal allowed deviation to right from
                                     ideal trajectory @unit:m */

} OTTrack_Vehicle_t;

typedef struct {
    float32 BracketPositionLeft;  /*!< Lane/Corridor Bracket in left position
                                     [-PI/2*RW_VLC_MAX ... PI/2*RW_VLC_MAX]
                                     @unit:m */
    float32 BracketPositionRight; /*!< Lane/Corridor Bracket in Right position
                                     [-PI/2*RW_VLC_MAX ... PI/2*RW_VLC_MAX]
                                     @unit:m */

} SIBracketOutput_t;

typedef struct {
    boolean bUseSeekLaneWidthOnly : 1;
    boolean bEnableAddExtensionObjectFastCutIn : 1;
    boolean bEnableAddRestrictionCurveOuterBorder : 1;
    boolean bEnableRestrictionTargetOutsideBrackets : 1;
    boolean bEnableRestrictionAnalogRoadBorder : 1;
    boolean bEnableRestrictionNeighbourhoodRelObj : 1;
    boolean bEnableRestrictionCityNearRange : 1;
    boolean bEnableExtensionRoadBorder : 1;
    boolean bEnableExtensionCurveInnerBorder : 1;
    boolean bEnableExtensionRoadBorderCI : 1;
    boolean bEnableExtensionFollowObjectIntoCurve : 1;
    boolean bEnableExtensionGuardRailRoadBorder : 1;
    boolean bEnableAddExtensionRelevantObject : 1;
    boolean bEnableExtensionHighspeedApproach : 1;
    boolean bEnableAddExtensionHighTunnelProb : 1;
    boolean bEnableExtensionLowSpeedFusedBrd : 1;
} SIBracketFuncEnable_t;

/*! State of trace bracket adaption */
typedef enum SIScaleBracketState {
    UNKNOWN, /*!< Unknown-> default value without functional influence */
    PRE_LANE_CHANGE_LEFT,  /*!< Lane change to the left based on blinker feature
                              (usually before internal LC prob is high) */
    PRE_LANE_CHANGE_RIGHT, /*!< Lane change to the right based on blinker
                              feature (usually before internal LC prob is high)
                              */
    POST_LANE_CHANGE_LEFT, /*!< Lane change to the left based on internal LC
                              prob */
    POST_LANE_CHANGE_RIGHT, /*!< Lane change to the right based on internal LC
                               prob */
    NO_LANE_CHANGE          /*!< No lane change -> adaption of trace bracket */
} SIScaleBracketState_t;
/*! Struct to describe the trace bracket adaption based on the ego position in
 * the lane */
typedef struct SIScaleBracketOutput {
    float32 fScaleBracketLeft;  /*! Factor to scale the left trace bracket */
    float32 fScaleBracketRight; /*! Factor to scale the right trace bracket */
    SIScaleBracketState_t
        StateScaleBracket; /*!< State of trace bracket adaption */
} SIScaleBracketOutput_t;
/*! State to describe the lane change for the trace bracket adaptio based on the
 * ego position in lane */
typedef enum SILaneChangeTraceBracketState {
    SI_NO_TB_LANE_CHANGE,   /*! No lane change */
    SI_TB_LANE_CHANGE_LEFT, /*! Lane change left */
    SI_TB_LANE_CHANGE_RIGHT /*! Lane change right */
} SILaneChangeTraceBracketState_t;

typedef struct {
    SIBracketOutput_t BaseCorridor;          /*!< Base corridor brackets */
    SIBracketOutput_t FinalBracketPositions; /*!< Final corridor brackets */
    /*! Kriterien legen Spurklammererweiterung oder -beschr�nkung basierend auf
     * bisheriger Position fest */
    SIBracketOutput_t AddExtensionRelevantObject;  /*!< VorbeifahrtRelObjekt */
    SIBracketOutput_t AddExtensionObjectFastCutIn; /*!< Approx */
    SIBracketOutput_t AddRestrictionCurveOuterBorder; /*!< KurveAussenrand */

    /*! Kriterien legen neue Spurklammerposition fest */
    SIBracketOutput_t RestrictionTargetOutsideBrackets; /*!< ZielAblage */
    SIBracketOutput_t RestrictionAnalogRoadBorder; /*!< AnalogerStrassenrand */
    SIBracketOutput_t
        RestrictionNeighbourhoodRelObj; /*!< UmfeldRelevantesObjekt */
    SIBracketOutput_t
        RestrictionCityNearRange; /*!< Restriction for cut-out in near range */

    SIBracketOutput_t ExtensionRoadBorder;       /*!< Richtungsfahrspuren */
    SIBracketOutput_t ExtensionCurveInnerBorder; /*!< KurveInnenrand */
    SIBracketOutput_t ExtensionRoadBorderCI;     /*!< RIchtungsfahrspurenKI */
    SIBracketOutput_t
        ExtensionFollowObjectIntoCurve; /*!< ObjektWinkelVerfolgung */
    SIBracketOutput_t ExtensionGuardRailRoadBorder; /*!< GuardRailRoadBorder */
    SIScaleBracketOutput_t
        RatioEgoPositionInLaneCam; /*!< RatioEgoPositionInLaneCam */

    SIBracketOutput_t AddExtensionHighspeedApproach; /*!< Extension for
                                                        highspeed approaches */

    SIBracketOutput_t AddExtensionHighTunnelProb; /*!< Extension for tunnels */

    SIBracketOutput_t AddExtensionLowSpeedFusedBrd; /*!< Extension for low speed
                                                       and fused road boarder */

} SICriteriaMatrix_t;

typedef struct {
    SIScaleBracketOutput_t
        RatioEgoPositionInLaneCam; /*!< RatioEgoPositionInLaneCam */

} SICriteriaMatrixAllObj_t;

typedef struct {
    float32 fRelevantTime;
    float32 fNotRelevantTime;
    OTTrack_Vehicle_t TrackVehicle;
    float32 fTargetFusionHoldTime;
} SIObjCorridor_t;

typedef struct {
    /* EOBorder */
    float32 dEOBorderLeft;  /*!< When positive, the distance to the road border
                               left */
    float32 dEOBorderRight; /*!< When positive, the distance to the road border
                               right*/

    /* Lane Matrix */
    sint32 iNumberLanesLeft;
    sint32 iNumberLanesRight;

    float32 dNoLaneProbL;
    float32 dNoLaneProbR;

    ObjNumber_t iRelObjNr;
    Envm_t_CR_DynamicProperty ucRelObjDynamicProperty;
    float32 fRelObjDistX;
    float32 fRelObjDistY;
    float32 fRelObjTargetFusionHoldTime;
} AssTraEnvironment_t;

#define DISABLED_FOR_TESTING_REMOVABILITY_OF_CURRENT_TARGET_FROM_VLC \
    (!VLC_USE_EM_GENERIC_OBJECT_LIST)
typedef struct {
    float32 fDistX;
    float32 fDistY;
    float32 fRefCourseDistX;
    float32 fRefCourseDistY;
    float32 fDistOnCourse;
    float32 fDistToCourse;
} RelObjShapePointCoord_t;

typedef struct {
    ObjNumber_t iObjNr;
    Envm_t_CR_DynamicProperty ucDynamicProperty;
    uint8 uiStoppedConfidence;
    TraceID_t iTracingID;
    ubit8_t iTraceReachedEgoVeh : 1;
    ubit8_t iRelevant : 1;
    AlgoCycleCounter_t iObjectLifeCycles;

    float32 fDistYStdDev;
    float32 fDistY;
    float32 fDistX;
    float32 fVRelY;
    float32 fOrientation;
    float32 fVelY;
    float32 fLatTrackLimitR;
    float32 fLatTrackLimitL;
    float32 fRefCourseDistY;
    float32 fDistToCourse;
    float32 fRelevantTime;
    float32 fNotRelevantTime;
    float32 fLatTrackLimitExpandFac;
    float32 fLatTrackLimitDistanceExpandFac;
    float32 dYIntersec;
    float32 dYIntersecGradFilt;
    RelObjShapePointCoord_t CornerPoint[4];
} RelTraObjInput_t;

typedef struct {
    eAssociatedLane_t iObjektSpur_Zyklus;

    sint32 iLwFolge;

    float32 fLatTrackLimitL;
    float32 fLatTrackLimitR;
    float32 fTargetFusionHoldTime;

} RelTraObjOutput_t;

/*! Structure describing the trajectory which is considered for trace bracket
 * calculation */
typedef struct {
    float32 dCurve; /*!< Curvature of considered trajectory for trace bracket
                       calculation */
    float32 dCurve_abs; /*!< Absolute curvature value of considered trajectory
                           for trace bracket calculation */
    sint32 iOWVflag;    /*!< Flag describing which trajectory hypothesis is
                           considered */
} RelTraCurve_t;

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
OLD SA
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/*****************************************************************************
  KONSTANTEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  VARIABLEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  FUNKTIONEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*----------------------------------------------------------------------------
  si_corridor.c
----------------------------------------------------------------------------*/
extern void SICorridorObjInit(SIObjCorridor_t* const pObjCor);
extern float32 SIGetBaseSeekLaneWidth(void);
extern float32 get_fSITrackLaneWidth(void);
/*----------------------------------------------------------------------------
  si_corridor_crit.c
----------------------------------------------------------------------------*/
extern void SIRelTraCheckCriteriaAllObj(
    SICriteriaMatrixAllObj_t* pCriteriaMatrixAllObj);
extern void SIInitCriteriaMatrixAllObj(
    SICriteriaMatrixAllObj_t* pCriteriaMatrixAllObj);

extern void SIEvaluateBracketFunctions(
    const RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    const RelTraCurve_t* const pTrajectory,
    AssTraEnvironment_t* const pEnvironment,
    SICriteriaMatrix_t* const pBracketFuncResults,
    SIBracketFuncEnable_t* const pBracketFuncEnableFlags,
    const SICriteriaMatrixAllObj_t* const pCritMatrixAllObj);
extern void SIDetermineFinalBracketPositions(
    const RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    SICriteriaMatrix_t* const pBracketFuncResults);

extern void SIExecuteRadarRoadBracketFunctions(
    const RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    const RelTraCurve_t* const pTrajectory,
    AssTraEnvironment_t* const pEnvironment,
    SICriteriaMatrix_t* const pBracketFuncResults,
    const SIBracketFuncEnable_t* const pBracketFuncEnableFlags);

extern void SICalculateCorridorBaseBrackets(
    const RelTraObjInput_t* const pObjectProperties,
    const RelTraCurve_t* const pTrajectory,
    SIBracketOutput_t* const pBaseBrackets,
    const SIBracketFuncEnable_t* const pBracketFuncEnableFlags);
extern void SIExecuteAdditiveBracketFunctions(
    const RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    const RelTraCurve_t* const pTrajectory,
    const AssTraEnvironment_t* pEnvironment,
    SICriteriaMatrix_t* const pBracketFuncResults,
    const SIBracketFuncEnable_t* const pBracketFuncEnableFlags);

/*--- si_corridor_scene.c ----*/
extern void SI_v_ExecuteSceneBracketFuncstions(
    RelTraObjInput_t const* p_ObjectProperties,
    AssTraEnvironment_t const* const p_Environment,
    SIBracketFuncEnable_t const* p_BracketFuncEnableFlags,
    SICriteriaMatrix_t* p_BracketFuncResults);

/*! Functions related to trace bracket adaption based on the ego position in
 * lane */

/* Init function */
extern void SIRelTraInitCriteriaOutputScale(
    SIScaleBracketOutput_t* const pOutput);

extern void SIRelTraRatioEgoPositionInLaneCam(
    SIScaleBracketOutput_t* const pScaleBracketOut);
extern void SIInitCorridorCamParameter(void);
extern SIScaleBracketState_t SIReturnStateScaleBracket(void);

extern boolean SIFindObjRelevantForTraceBracketAdaptPosInLaneCam(
    RelTraObjInput_t const* pObj,
    AssTraEnvironment_t const* pEnvironment,
    const SIScaleBracketOutput_t* pRatioEgoPosInLine);
extern SIScaleBracketOutput_t SIAdaptRatioEgoPosInLaneCamToObj(
    RelTraObjInput_t const* pObjInput,
    SIScaleBracketOutput_t const ScaleBracketIn,
    AssTraEnvironment_t const* pEnvironment);
extern void SIRelTraSetTrackWidthScale(
    const float32 fYPosCenterBracket,
    float32* pTrackWidthLeft,
    float32* pTrackWidthRight,
    const SIScaleBracketOutput_t* pScaleBracket);

#ifdef __cplusplus
};
#endif

#endif
