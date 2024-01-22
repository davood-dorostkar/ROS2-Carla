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
//#include "vlc_par.h"
#include "fip_ext.h"
/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

#ifndef _PARAM_UNUSED
#define _PARAM_UNUSED(x) (void)(x)
#endif

#define TraceOccupancyPickupThresh (0.5f)

#define SI_PERC_LENGTH_REFLECTION_INNER_CURVE_EXT (0.45f)
#define SI_MIN_CURVE_INNER_CURVE_EXT (1.f / 250.f)
#define SI_TIMECONST_INNER_CURVE_EXT (0.3f)
#define FSZ_A2_RICHTUNGSFAHRSPUREN_KI (1.0F / 500.F)

/* helps in narrow curves or roundabouds */
#define ECIBO_MAX_CURVE                                                  \
    (1.0f / 175.0f) /*!< maximal curve radius for activation of extended \
                       curveinnerborder criterion */
#define ECIBO_MAX_DISTX                                             \
    (40.0f) /*!< maximal object distance for activation of extended \
               curveinnerborder criterion */
#define ECIBO_MAX_VELOCITY                                              \
    (60.0f / C_KMH_MS) /*!< maximal velocity for activation of extended \
                          curveinnerborder criterion */
#define ECIBO_MAX_EXTENTIONANGLE_TAN \
    (39.0f) /*!< maximal extention of trace bracket (tangens(39 degree)) */

#define FSZ_ZIELABLAGE_ABSTAND (50.F)
#define FSZ_ZIELABLAGE_KRUEMMUNG (1.0F / 3000.F)
#define FSZ_Y_STDDEV_WEIGHT_FACTOR (0.125f)

/* Im Seekzustand wird maximal auf den enger am Radius liegenden Wert */
/* erweitert, welcher aus dem Winkel oder aus Radius + Ablage         */
/* bestimmt wird.                                                     */
#define SEEK_RFS_ABLAGE_NARROWCURV_MAX (7.0F)
/* Filterzeit in sec. der Wirkung eines Schaltvorgangs bei anliegendem */
/* Strassenrand.                                                       */
#define FSZ_RFS_FILTER_ZEIT (0.3F)

#define ABST_KEINESPUR_MIN (15.F)
#define ABST_KEINESPUR_RELOBJ (25.F)
#define ABST_KEINESPUR_MAX (130.F)

/* Winkel fuer Definition Objekte sind am Keulenrand */
#define FSZ_WINKEL_RICHTUNGSFAHRSPUREN (LOBE_ANGLE * 0.8F)
/* Kruemmung ab der am Kurvenaussenrand keine Erweiterung fuer am Kurveninneren
 */
/* Keulenrand befindeliche Objekte mehr stattfindet. */
#define FSZ_A2_RICHTUNGSFAHRSPUREN (1.0F / 1000.F)

#define FSZ_RFS_NARROW_CURVE (1.0F / 900.F)
#define FSZ_RFS_NARROW_CURVE_REL (1.0F / 700.F)
#define FSZ_RFS_NARROW_CURVE_GRADIENT (0.001F)

#define OWV_SPURGRAD_MIN_HALTEN_MAXWINKEL (0.00002F)
/* ARS2xx: #define OWV_SPURGRAD_WINKEL_MAX       (float32)(3.35F) */
#define OWV_SPURGRAD_WINKEL_MAX (LOBE_ANGLE - 0.5f)
/* Spurkruemmung fuer Abfrage GERADEAUS*/
#define SPURKRUEMMUNG_MIN_GERADEAUS (1.0F / 20000.F)

/* Mindestobjektlebensdauer fuer das OWV-Kriterium */
#define OWV_OBJLEB (50u)
/* max. Abstand in dem die Spurnachfuehrung funktioniert */
#define MAX_ABSTSTEIG (120.F)
/* min. Abstand in dem die Spurnachfuehrung funktioniert */
#define MIN_ABSTSTEIG (30.F)
/* min. Abstand, Vrel und Veigen fuer Spurerweiterung RFS, RFSKI, OWV */
#define MIN_ABST_VREL (60.F)
#define MIN_OWV_VREL (25.F / C_KMH_MS)
#define MIN_OWV_VEIGEN (25.F / C_KMH_MS)
/* max. relative speed for trace criterion for OWV */
#define MAX_OWV_VREL_TRACE_CRIT (15.F / C_KMH_MS)
/* min. Spurvektorsteigung abgestimmt auf mehr als 1 Digit LW */
#define MIN_SPURSTEIG (0.0004F)
/* max. Haltezeit der Erweit., wenn Steigung im Haltezustand */
#define MAX_HALTEZEIT (5L)
/* Unterhalb dieser Spurabweichung keine Erweiterung mehr */
#define MIN_SPURABWEICH (0.5F)
/* Oberhalb dieser Spurabweichung keine Erweiterung mehr */
#define MAX_SPURABWEICH (2.F)
/*! Minimum lane change probability */
#define SI_MIN_LC_PROB_FOLLOW_OBJ_INTO_CURVE (80)

/* Konstanten fuer SpurGradHaltenKruemmung */
#define OWV_SPURGRAD_MIN_HALTEN (0.00002F)
#define OWV_SPURGRAD_MAX_HALTEN (0.0004F)
#define OWV_SPURGRAD_KRUEMMUNG_MIN (1.0F / 5000.F)
#define OWV_SPURGRAD_KRUEMMUNG_MAX (1.0F / 2500.F)

/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
OLD SA
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

/*-- SIRelTra.c --*/
/* Konstanten fuer SRandUnsicherAbstand */
#define FSZ_SRAND_ABST_MIN (30.F)
#define FSZ_SRAND_ABST_MAX (150.F)
#define FSZ_SRANDUNSICHER_MIN (1.5F)
#define FSZ_SRANDUNSICHER_MAX (4.0F)

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
OLD SA
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/*! Minimum road border status for use in road border extension function */
#define SI_GUARDRAIL_ROAD_BORDER_EXT_MIN_TRK_STATUS 4u

/*! Maximal lateral displacement to predicted ACC course for road border
criteria @unit:m */
#define SI_MAX_ROAD_BORD_LAT_NO_TRACE 5.0f
/*! Maximum lateral displacement to predicted ACC course for a given relevant
object to be able to activate OWV criteria over it's trace @unit:m */
#define SI_MAX_TRACE_LAT_DISPLACEMENT 5.0f

/*! Maximum lateral displacement of trace bracket due to OWV criteria */
#define SI_MAX_OVW_ABLAGE 5.0f

#define SI_TB_EXTENSION_ROADBORDER_MIN \
    (1.5f) /*!< Minimal extension of trace brackets for Road Border */
#define SI_TB_EXTENSION_ROADBORDER_MAX \
    (3.5f) /*!< Maximal extension of trace brackets for Road Border */
#define SI_DISTX_TB_EXTENSION_ROADBORDER_MIN                             \
    ABST_KEINESPUR_MIN /*!< Distance to object for minimal trace bracket \
                          extension */
#define SI_DISTX_TB_EXTENSION_ROADBORDER_MAX \
    (50.f) /*!< Distance to object for maximal trace bracket extension */

#define SI_DISTX_FAR_RANGE_ANGLE_TB_EXT                                 \
    (70.f) /*!< If longitudinal distance if higher, the far range angle \
                is considered for the trace bracket extension */
#define SI_ANGLE_NEAR_RANGE_TB_EXT                           \
    (25.f) /*!< Angle that is used in the near range for the \
                trace bracket extension, Remark: ARS301 = 25Grad */

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/
/*! Enum to describe which FOV to consider for a specific corridor function */
typedef enum SICorrFOVTypeExtension {
    SI_FAR_RANGE_ONLY_LEFT, /*!< Consider far range only on the left FOV side */
    SI_FAR_AND_NEAR_RANGE_LEFT, /*!< Consider far and near range on the left FOV
                                   side */
    SI_FAR_RANGE_ONLY_RIGHT, /*!< Consider far range only on the right FOV side
                              */
    SI_FAR_AND_NEAR_RANGE_RIGHT /*!< Consider far and near range on the right
                                   FOV side */
} SICorrFOVTypeExtension_t;

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/
/*-- SIRelTra.c --*/
/* Rampe zur Veraenderung der SpurGradienten-Schwelle in Abh. der Spurkruemmung
 */
SET_MEMSEC_CONST(SpurGradHaltenKruemmung)
static const GDBLFunction_t SpurGradHaltenKruemmung = {

    OWV_SPURGRAD_MIN_HALTEN, /*!< A1 */
    OWV_SPURGRAD_MAX_HALTEN, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */
    (OWV_SPURGRAD_MAX_HALTEN - OWV_SPURGRAD_MIN_HALTEN) /
        (OWV_SPURGRAD_KRUEMMUNG_MAX - OWV_SPURGRAD_KRUEMMUNG_MIN),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    OWV_SPURGRAD_MIN_HALTEN -
        (((OWV_SPURGRAD_MAX_HALTEN - OWV_SPURGRAD_MIN_HALTEN) /
          (OWV_SPURGRAD_KRUEMMUNG_MAX - OWV_SPURGRAD_KRUEMMUNG_MIN)) *
         OWV_SPURGRAD_KRUEMMUNG_MIN)};

/*! Extension of Road Border relative to distance between ZO and EGO */
SET_MEMSEC_CONST(SI_TB_EXTENSION_ROADBORDER)
static const GDBLFunction_t SI_TB_EXTENSION_ROADBORDER = {

    SI_TB_EXTENSION_ROADBORDER_MIN, /*!< A1 */
    SI_TB_EXTENSION_ROADBORDER_MAX, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */
    (SI_TB_EXTENSION_ROADBORDER_MAX - SI_TB_EXTENSION_ROADBORDER_MIN) /
        (SI_DISTX_TB_EXTENSION_ROADBORDER_MAX -
         SI_DISTX_TB_EXTENSION_ROADBORDER_MIN),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    SI_TB_EXTENSION_ROADBORDER_MIN -
        (((SI_TB_EXTENSION_ROADBORDER_MAX - SI_TB_EXTENSION_ROADBORDER_MIN) /
          (SI_DISTX_TB_EXTENSION_ROADBORDER_MAX -
           SI_DISTX_TB_EXTENSION_ROADBORDER_MIN)) *
         SI_DISTX_TB_EXTENSION_ROADBORDER_MIN)};

/*! Parameters for restriction in the near range for country road /city
scenarios to improve the release of objects that take a turn */
/* Parameter to describe the restriction based on the longitudinal distance to
 * the object */
SET_MEMSEC_CONST(t_SI_RestictCityNearRange)
static const GDBLFunction_t t_SI_RestictCityNearRange = {

    SI_REST_CITY_NEAR_RANGE_MIN_RESTICT, /*!< A1 */
    SI_REST_CITY_NEAR_RANGE_MAX_RESTICT, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */
    (SI_REST_CITY_NEAR_RANGE_MAX_RESTICT -
     SI_REST_CITY_NEAR_RANGE_MIN_RESTICT) /
        (SI_REST_CITY_NEAR_RANGE_MAX_DIST_X -
         SI_REST_CITY_NEAR_RANGE_MIN_DIST_X),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    SI_REST_CITY_NEAR_RANGE_MIN_RESTICT -
        (((SI_REST_CITY_NEAR_RANGE_MAX_RESTICT -
           SI_REST_CITY_NEAR_RANGE_MIN_RESTICT) /
          (SI_REST_CITY_NEAR_RANGE_MAX_DIST_X -
           SI_REST_CITY_NEAR_RANGE_MIN_DIST_X)) *
         SI_REST_CITY_NEAR_RANGE_MIN_DIST_X)};

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*! Global seek lane width variable. Cached here to avoid recalculating linear
interpolation ramp over and over again */
/*! Global track lane width variable. Cached here to avoid recalculation */

SET_MEMSEC_CONST(SRandUnsicherAbstand)
static const GDBLFunction_t SRandUnsicherAbstand = {

    FSZ_SRANDUNSICHER_MIN, /*!< A1 */
    FSZ_SRANDUNSICHER_MAX, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */
    (FSZ_SRANDUNSICHER_MAX - FSZ_SRANDUNSICHER_MIN) /
        (FSZ_SRAND_ABST_MAX - FSZ_SRAND_ABST_MIN),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    FSZ_SRANDUNSICHER_MIN - (((FSZ_SRANDUNSICHER_MAX - FSZ_SRANDUNSICHER_MIN) /
                              (FSZ_SRAND_ABST_MAX - FSZ_SRAND_ABST_MIN)) *
                             FSZ_SRAND_ABST_MIN)};

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void SIRelTraExtensionRoadBorderCI(
    RelTraObjInput_t const *pObjInput,
    AssTraEnvironment_t const *pEnvironment,
    RelTraCurve_t const *pRelTraCurve,
    SIBracketOutput_t *pBracketOut);

static float32 SIRelTraGetYDistFOVAtXDist(float32 f_DistX,
                                          SICorrFOVTypeExtension_t t_FOVType);

/* Bracket functions */

static void SIRelTraRestrictionTargetOutsideBrackets(
    RelTraObjInput_t const *pObjInput,
    RelTraObjOutput_t const *pObjOutput,
    RelTraCurve_t const *pRelTraCurve,
    SIBracketOutput_t *pBracketOut);
static void SIRelTraRestrictionAnalogRoadBorder(
    RelTraObjInput_t const *pObjInput,
    AssTraEnvironment_t const *pEnvironment,
    SIBracketOutput_t *pBracketOut);
static void SI_v_RelTraRestictCityNearRange(RelTraObjInput_t const *p_ObjInput,
                                            SIBracketOutput_t *p_BracketOut);
static void SIRelTraExtensionRoadBorder(RelTraObjInput_t const *pObjInput,
                                        AssTraEnvironment_t const *pEnvironment,
                                        RelTraCurve_t const *pRelTraCurve,
                                        SIBracketOutput_t *pBracketOut);
static void SIRelTraExtensionCurveInnerBorder(
    RelTraObjInput_t const *pObjInput,
    AssTraEnvironment_t const *pEnvironment,
    RelTraCurve_t const *pRelTraCurve,
    SIBracketOutput_t *pBracketOut);

static void SIRelCheckOWV(RelTraObjInput_t const *pObjInput,
                          RelTraObjOutput_t *pObjOutput,
                          AssTraEnvironment_t const *pEnvironment,
                          RelTraCurve_t const *pRelTraCurve);
static void SIRelTraExtensionFollowObjectIntoCurve(
    RelTraObjInput_t const *pObjInput,
    const RelTraObjOutput_t *pObjOutput,
    SIBracketOutput_t *pBracketOut,
    RelTraCurve_t const *pRelTraCurve);
static boolean SICheckTraceCriterion(const ObjNumber_t iObj);
static void SICalculateTraceBorders(const ObjNumber_t iObj,
                                    CPDistanceWidth_t *pDistanceWidth);

/*************************************************************************************************************************
  Functionname:    SIExecuteRadarRoadBracketFunctions */
void SIExecuteRadarRoadBracketFunctions(
    const RelTraObjInput_t *const pObjectProperties,
    RelTraObjOutput_t *const pObjectVariables,
    const RelTraCurve_t *const pTrajectory,
    AssTraEnvironment_t *const pEnvironment,
    SICriteriaMatrix_t *const pBracketFuncResults,
    const SIBracketFuncEnable_t *const pBracketFuncEnableFlags) {
    /*********************** RESTRICTIONS ******************************/

    /* RestrictionAnalogRoadBorder */
    if (pBracketFuncEnableFlags->bEnableRestrictionAnalogRoadBorder !=
        SI_CORR_BRACKET_FUNC_DISABLED) {
        SIRelTraRestrictionAnalogRoadBorder(
            pObjectProperties, pEnvironment,
            &pBracketFuncResults->RestrictionAnalogRoadBorder);
    }

    /* RestrictionTargetOutsideBrackets */
    if (pBracketFuncEnableFlags->bEnableRestrictionTargetOutsideBrackets !=
        SI_CORR_BRACKET_FUNC_DISABLED) {
        SIRelTraRestrictionTargetOutsideBrackets(
            pObjectProperties, pObjectVariables, pTrajectory,
            &pBracketFuncResults->RestrictionTargetOutsideBrackets);
    }

    /*! Restriction in the near range for country road /city scenarios */
    if (pBracketFuncEnableFlags->bEnableRestrictionCityNearRange !=
        SI_CORR_BRACKET_FUNC_DISABLED) {
        SI_v_RelTraRestictCityNearRange(
            pObjectProperties, &pBracketFuncResults->RestrictionCityNearRange);
    }

    /*********************** EXTENSIONS ******************************/

    /* ExtensionCurveInnerBorder */
    if (pBracketFuncEnableFlags->bEnableExtensionCurveInnerBorder !=
        SI_CORR_BRACKET_FUNC_DISABLED) {
        SIRelTraExtensionCurveInnerBorder(
            pObjectProperties, pEnvironment, pTrajectory,
            &pBracketFuncResults->ExtensionCurveInnerBorder);
    }

    /* ExtensionRoadBorderCI */
    if (pBracketFuncEnableFlags->bEnableExtensionRoadBorderCI !=
        SI_CORR_BRACKET_FUNC_DISABLED) {
        SIRelTraExtensionRoadBorderCI(
            pObjectProperties, pEnvironment, pTrajectory,
            &pBracketFuncResults->ExtensionRoadBorderCI);
    }

    /* ExtensionFollowObjectIntoCurve */
    if (pBracketFuncEnableFlags->bEnableExtensionFollowObjectIntoCurve !=
        SI_CORR_BRACKET_FUNC_DISABLED) {
        /* Check first if the object should be further followed */
        SIRelCheckOWV(pObjectProperties, pObjectVariables, pEnvironment,
                      pTrajectory);
        /* In case the check was successful, extend the trace brackets
         * accordingly */
        SIRelTraExtensionFollowObjectIntoCurve(
            pObjectProperties, pObjectVariables,
            &pBracketFuncResults->ExtensionFollowObjectIntoCurve, pTrajectory);
    }

    /* ExtensionRoadBorder */
    if (pBracketFuncEnableFlags->bEnableExtensionRoadBorder !=
        SI_CORR_BRACKET_FUNC_DISABLED) {
        SIRelTraExtensionRoadBorder(pObjectProperties, pEnvironment,
                                    pTrajectory,
                                    &pBracketFuncResults->ExtensionRoadBorder);
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraRestrictionTargetOutsideBrackets */
static void SIRelTraRestrictionTargetOutsideBrackets(
    RelTraObjInput_t const *pObjInput,
    RelTraObjOutput_t const *pObjOutput,
    RelTraCurve_t const *pRelTraCurve,
    SIBracketOutput_t *pBracketOut) {
    /* Currently function internal implementation removed to test removability
    of VLC dependence on current target information */
    if (pObjInput->iRelevant == FALSE) {
        if (pObjOutput->iObjektSpur_Zyklus ==
            ASSOC_LANE_EGO) { /* Objekt ist aktuell innerhalb der Spurgrenzen */
            if (pObjInput->fDistX >
                FSZ_ZIELABLAGE_ABSTAND) { /* Abstand des Objekts groesser ca.
                                             50m */
                if (pObjInput->fLatTrackLimitExpandFac <
                    C_F32_DELTA) { /* Objekt war in letzter Zeit nicht relevant
                                    */
                    if (pRelTraCurve->dCurve < -FSZ_ZIELABLAGE_KRUEMMUNG) {
                        /* Nur am Kurvenaussenrand der Rechtskurve */
                        if ((pObjInput->fDistY + (pObjInput->fDistYStdDev *
                                                  FSZ_Y_STDDEV_WEIGHT_FACTOR)) >
                            pObjOutput->fLatTrackLimitL) {
                            /* Linke Spurgrenze bis auf Mitte des relevanten
                             * Radius verschieben */
                            pBracketOut->BracketPositionLeft =
                                pObjInput->fRefCourseDistY;
                        }
                    } else if (pRelTraCurve->dCurve >
                               FSZ_ZIELABLAGE_KRUEMMUNG) {
                        /* Nur am Kurvenaussenrand der Linkskurve */
                        if ((pObjInput->fDistY - (pObjInput->fDistYStdDev *
                                                  FSZ_Y_STDDEV_WEIGHT_FACTOR)) <
                            pObjOutput->fLatTrackLimitR) {
                            /* Rechte Spurgrenze bis auf Mitte des relevanten
                             * Radius verschieben */
                            pBracketOut->BracketPositionRight =
                                pObjInput->fRefCourseDistY;
                        }
                    } else {
                        /*! Nothing */
                    }
                }
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraRestrictionAnalogRoadBorder */
static void SIRelTraRestrictionAnalogRoadBorder(
    RelTraObjInput_t const *pObjInput,
    AssTraEnvironment_t const *pEnvironment,
    SIBracketOutput_t *pBracketOut) {
    /* Objekte die ausserhalb des Strassenrands liegen abschneiden */

    if (pEnvironment->dEOBorderRight >= 0.f) {
        /* Spurklammerposition schreiben */
        pBracketOut->BracketPositionRight =
            (pObjInput->fRefCourseDistY - pEnvironment->dEOBorderRight) -
            dGDBmathLineareFunktion(&SRandUnsicherAbstand, pObjInput->fDistX);
    }

    if (pEnvironment->dEOBorderLeft >= 0.f) {
        /* Spurklammerposition schreiben */
        pBracketOut->BracketPositionLeft =
            pObjInput->fRefCourseDistY + pEnvironment->dEOBorderLeft +
            dGDBmathLineareFunktion(&SRandUnsicherAbstand, pObjInput->fDistX);
    }
}

/*************************************************************************************************************************
  Functionname:    SI_v_RelTraRestictCityNearRange */
static void SI_v_RelTraRestictCityNearRange(RelTraObjInput_t const *p_ObjInput,
                                            SIBracketOutput_t *p_BracketOut) {
    float32 f_RestrictionCityNearRange, f_ObjDist2Traj;

    /* Initialization of local variable */
    f_RestrictionCityNearRange = 0.f; /*!< Restriction value */
    /* Get distance of object to ACC trajectory */
    f_ObjDist2Traj =
        CPTrajGetObjToRefDistMeas(&OBJ_GET_CP(p_ObjInput->iObjNr).TrajDist);
    /* Get EM road type */

    /*! Apply restriction if:
        -driving on a highway / country road
        -ego speed lower than threshold
        -object is no truck or point (position of these objects are more often
       wrong due to tracking issues (reflexion point instable))
        -object is moving
        -object is near (longitudinal distance lower 15m)
        -object's relative velocity is lower than a threshold (object is driving
       slower than ego vehicle to
                  prevent late cut-ins and to get the situation that the object
       is taking a turn)
        -ego curve is lower than a threshold (if following object in a curve, no
       restriction should be applied)
    */
    if ((EGO_SPEED_X_OBJ_SYNC < SI_REST_CITY_NEAR_RANGE_MAX_VELOCITY) &&
        (OBJ_CLASSIFICATION(p_ObjInput->iObjNr) != CR_OBJCLASS_TRUCK) &&
        (OBJ_CLASSIFICATION(p_ObjInput->iObjNr) != CR_OBJCLASS_POINT) &&
        (p_ObjInput->ucDynamicProperty == CR_OBJECT_PROPERTY_MOVING) &&
        (OBJ_LONG_DISPLACEMENT(p_ObjInput->iObjNr) <
         SI_REST_CITY_NEAR_RANGE_MAX_DIST_X) &&
        (OBJ_LONG_VREL(p_ObjInput->iObjNr) <
         SI_REST_CITY_NEAR_RANGE_MAX_VREL) &&
        (((f_ObjDist2Traj > 0.f) &&
          (EGO_CURVE_OBJ_SYNC < SI_REST_CITY_NEAR_RANGE_MAX_CURVE)) ||
         ((f_ObjDist2Traj < 0.f) &&
          (EGO_CURVE_OBJ_SYNC > -SI_REST_CITY_NEAR_RANGE_MAX_CURVE)))) {
        /* Get the restriction value based on the longitudinal distance */
        f_RestrictionCityNearRange =
            dGDBmathLineareFunktion(&t_SI_RestictCityNearRange,
                                    OBJ_LONG_DISPLACEMENT(p_ObjInput->iObjNr));
        /* Apply the restriction on the corresponding side */
        if (f_ObjDist2Traj > 0.f) {
            p_BracketOut->BracketPositionLeft =
                (p_ObjInput->fRefCourseDistY + f_RestrictionCityNearRange);
        } else {
            p_BracketOut->BracketPositionRight =
                (p_ObjInput->fRefCourseDistY - f_RestrictionCityNearRange);
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraExtensionRoadBorder */
static void SIRelTraExtensionRoadBorder(RelTraObjInput_t const *pObjInput,
                                        AssTraEnvironment_t const *pEnvironment,
                                        RelTraCurve_t const *pRelTraCurve,
                                        SIBracketOutput_t *pBracketOut) {
    /*--- VARIABLES ---*/

    float32 dAblage_rechts, dAblage_links;
    float32 dAblageSeek;

    float32 fABS_pObjInput_fVRelY;
    /* Calculate distance to estimated ego course */
    const float32 fLatDisplToCourse =
        (pObjInput->fDistY - pObjInput->fRefCourseDistY);

    if (pRelTraCurve->iOWVflag == OWVKRIT_AKTIV) {
        /* Die maximale Spurklammererweiterung wird in Abhaengigkeit der
         * RELEVANZ der Objekte entschieden */

        /* Ist Objekt relevant und besitzt eine gewisse Spursicherheit? */
        if (pObjInput->iRelevant == (ubit8_t)TRUE) {
            float32 fLatDisplFovLeft, fLatDisplFovRight;
            /* Get field of view information for object */
            fLatDisplFovLeft = SIRelTraGetYDistFOVAtXDist(
                pObjInput->fDistX, SI_FAR_AND_NEAR_RANGE_LEFT);
            fLatDisplFovRight = SIRelTraGetYDistFOVAtXDist(
                pObjInput->fDistX, SI_FAR_AND_NEAR_RANGE_RIGHT);

            /* Depending on driven curvature use different extensions */
            if (pRelTraCurve->dCurve_abs < FSZ_RFS_NARROW_CURVE_REL) {
                /* When driving curves with larger radius, limit maximum lateral
                deviation from predicted course
                to a maximum (SEEK_RFS_ABLAGE_MAX) */
                dAblageSeek = dGDBmathLineareFunktion(
                    &SI_TB_EXTENSION_ROADBORDER, pObjInput->fDistX);

                /*! Calculate right trace bracket by subtracting the given
                   maximum lateral deviation,
                    Remark: the value can be either negative or positive */
                dAblage_rechts = pObjInput->fRefCourseDistY - dAblageSeek;
                /* Take the larger of the two = take the one closer to zero */
                dAblage_rechts = MAX_FLOAT(dAblage_rechts, fLatDisplFovRight);

                /*! Calculate the left trace bracket by adding the given maximum
                   lateral deviation,
                    Remark: the value can be either positive or negative */
                dAblage_links = pObjInput->fRefCourseDistY + dAblageSeek;
                /* Take the smaller of the two = take the one closer to zero */
                dAblage_links = MIN_FLOAT(dAblage_links, fLatDisplFovLeft);
            } else {
                /* In tight curves extend trace brackets up to a given maximal
                lateral distance
                from the predicted course */
                dAblage_rechts =
                    pObjInput->fRefCourseDistY - SEEK_RFS_ABLAGE_NARROWCURV_MAX;
                dAblage_rechts = MAX_FLOAT(dAblage_rechts, fLatDisplFovRight);

                dAblage_links =
                    pObjInput->fRefCourseDistY + SEEK_RFS_ABLAGE_NARROWCURV_MAX;
                dAblage_links = MIN_FLOAT(dAblage_links, fLatDisplFovLeft);
            }

            /* Nutzen der Richtungsfahrspuren Information keine Spur links oder
             * rechts */

            /* 1.) Objektabstand muss groesser als ABST_KEINESPUR_MIN (ca. 30 m)
             * sein ! */
            /* 2.) Objektabstand muss kleiner als ABST_KEINESPUR_MAX (ca. 130 m)
             * sein ! */
            /* 3.) Eigengeschwindigkeit muss groesser sein als MIN_OWV_VEIGEN
             * (ca. 25 kph)! */
            /* 4.) Objektabstand muss groesser als ca. 63 m oder absolut Wert
             * von Relativgeschwindigkeit */
            /*     kleiner als MIN_OWV_VREL (ca. 25 kph) */
            fABS_pObjInput_fVRelY = fABS(pObjInput->fVRelY);
            if ((pObjInput->fDistX > ABST_KEINESPUR_MIN) &&
                (pObjInput->fDistX < ABST_KEINESPUR_MAX) &&
                (EGO_SPEED_X_OBJ_SYNC > MIN_OWV_VEIGEN) &&
                ((fABS_pObjInput_fVRelY < MIN_OWV_VREL) ||
                 (pObjInput->fDistX > MIN_ABST_VREL))) {
                /* Keine Richtungsspur rechts vom Fahrzeug */
                if ((pObjInput->iRelevant == (ubit8_t)FALSE) &&
                    (pObjInput->fOrientation <
                     -FSZ_WINKEL_RICHTUNGSFAHRSPUREN) &&
                    (pRelTraCurve->dCurve < -FSZ_A2_RICHTUNGSFAHRSPUREN)) {
                    /* Nicht relevante Objekte am kurveninneren Keulenrand
                     * erhalten keine Erweiterung */
                    /* der Fahrspur am Kurvenaussenrand bei Radien kleiner 1000m
                     */
                } else {
                    if ((pEnvironment->iNumberLanesRight == 0L) &&
                        (fLatDisplToCourse > -SI_MAX_ROAD_BORD_LAT_NO_TRACE)) {
                        if (dAblage_rechts < pObjInput->fLatTrackLimitR) {
                            float32 fFilterTimeConst = 0;
                            if (SI_CYCLE_TIME > C_F32_DELTA) {
                                fFilterTimeConst =
                                    (FSZ_RFS_FILTER_ZEIT / SI_CYCLE_TIME);
                            }
                            dAblage_rechts = GDB_FILTER(
                                dAblage_rechts, pObjInput->fLatTrackLimitR,
                                fFilterTimeConst);
                        }

                        /* Spurklammerposition schreiben */
                        pBracketOut->BracketPositionRight = dAblage_rechts;
                    }
                }

                /* Keine Richtungsspur links vom Fahrzeug */
                if ((pObjInput->iRelevant == (ubit8_t)FALSE) &&
                    (pObjInput->fOrientation >
                     FSZ_WINKEL_RICHTUNGSFAHRSPUREN) &&
                    (pRelTraCurve->dCurve > FSZ_A2_RICHTUNGSFAHRSPUREN)) {
                    /* Nicht relevante Objekte am kurveninneren Keulenrand
                     * erhalten keine Erweiterung */
                    /* der Fahrspur am Kurvenaussenrand bei Radien kleiner 1000m
                     */
                } else {
                    if ((pEnvironment->iNumberLanesLeft == 0L) &&
                        (fLatDisplToCourse < SI_MAX_ROAD_BORD_LAT_NO_TRACE)

                    ) {
                        if (dAblage_links > pObjInput->fLatTrackLimitL) {
                            float32 fFilterTimeConst = 0;
                            if (SI_CYCLE_TIME > C_F32_DELTA) {
                                fFilterTimeConst =
                                    (FSZ_RFS_FILTER_ZEIT / SI_CYCLE_TIME);
                            }
                            dAblage_links = GDB_FILTER(
                                dAblage_links, pObjInput->fLatTrackLimitL,
                                fFilterTimeConst);
                        }

                        /* Spurklammerposition schreiben */
                        pBracketOut->BracketPositionLeft = dAblage_links;
                    }
                }
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraExtensionCurveInnerBorder */
static void SIRelTraExtensionCurveInnerBorder(
    RelTraObjInput_t const *pObjInput,
    AssTraEnvironment_t const *pEnvironment,
    RelTraCurve_t const *pRelTraCurve,
    SIBracketOutput_t *pBracketOut) {
    /*--- VARIABLES ---*/
    float32 fAngleCurve2Y;
    float32 fLengthObj;
    float32 fDistReflectionMoveDueToCurve;
    float32 fExtensionFoV;

    if (pEnvironment->iRelObjNr == pObjInput->iObjNr) {
        /*! Angle between the slope at the curve (at the position of the object)
         * and the y-axis */
        fAngleCurve2Y = ATAN2_((pRelTraCurve->dCurve * pObjInput->fDistX), 1.f);
        /*! Length of object (depending on the object class) */
        fLengthObj = SIGetObjectLength(pObjInput->iObjNr);
        /*! Estimation of how far the reflection point moves due to the
         * refection from the side */
        fDistReflectionMoveDueToCurve =
            SI_PERC_LENGTH_REFLECTION_INNER_CURVE_EXT * COS_(fAngleCurve2Y) *
            fLengthObj;

        /*! In case of a right curve */
        if ((pRelTraCurve->dCurve < -ECIBO_MAX_CURVE) &&
            (pObjInput->fDistX < ECIBO_MAX_DISTX) &&
            (EGO_SPEED_X_OBJ_SYNC < ECIBO_MAX_VELOCITY)) {
            /*! Use a wider angle */
            fExtensionFoV = ((pObjInput->fDistX *
                              TAN_(DEG2RAD(-ECIBO_MAX_EXTENTIONANGLE_TAN))) +
                             SENSOR_Y_POSITION);
        } else {
            /*! Use the far range angle only */
            fExtensionFoV = SIRelTraGetYDistFOVAtXDist(pObjInput->fDistX,
                                                       SI_FAR_RANGE_ONLY_RIGHT);
        }

        /*! Right curve: If curvature higher than a threshold */
        if (pRelTraCurve->dCurve < -SI_MIN_CURVE_INNER_CURVE_EXT) {
            /*! Open trace bracket to the border of the FoV (with filter) */
            if (fExtensionFoV < pObjInput->fLatTrackLimitR) {
                float32 fFilterTimeConst = 0;
                if (SI_CYCLE_TIME > C_F32_DELTA) {
                    fFilterTimeConst =
                        (SI_TIMECONST_INNER_CURVE_EXT / SI_CYCLE_TIME);
                }
                fExtensionFoV =
                    GDB_FILTER(fExtensionFoV, pObjInput->fLatTrackLimitR,
                               fFilterTimeConst);
            }

            /*! If not steering (EGO_DRV_INT_CURVE_GRAD_RAW) clearly in the
               other direction of the curvature,
                apply trace bracket extension */
            if (EGO_DRV_INT_CURVE_GRAD_RAW < -EGO_DRV_INT_CURVE_RAW) {
                /*! Set right trace bracket: Minimum of border to FoV and
                 * estimation of moved reflection point due to curve */
                pBracketOut->BracketPositionRight = MAX_FLOAT(
                    (pObjInput->fDistY - fDistReflectionMoveDueToCurve),
                    fExtensionFoV);
            }
        }

        /*! In case of a left curve */
        if ((pRelTraCurve->dCurve > ECIBO_MAX_CURVE) &&
            (pObjInput->fDistX < ECIBO_MAX_DISTX) &&
            (EGO_SPEED_X_OBJ_SYNC < ECIBO_MAX_VELOCITY)) {
            /*! Use a wider angle */
            fExtensionFoV = ((pObjInput->fDistX *
                              TAN_(DEG2RAD(ECIBO_MAX_EXTENTIONANGLE_TAN))) +
                             SENSOR_Y_POSITION);
        } else {
            /*! Use the far range angle only */
            fExtensionFoV = SIRelTraGetYDistFOVAtXDist(pObjInput->fDistX,
                                                       SI_FAR_RANGE_ONLY_LEFT);
        }

        /*! Left curve: If curvature higher than a threshold */
        if (pRelTraCurve->dCurve > SI_MIN_CURVE_INNER_CURVE_EXT) {
            /*! Open trace bracket to the border of the FoV (with filter) */
            if (fExtensionFoV > pObjInput->fLatTrackLimitL) {
                float32 fFilterTimeConst = 0;
                if (SI_CYCLE_TIME > C_F32_DELTA) {
                    fFilterTimeConst =
                        (SI_TIMECONST_INNER_CURVE_EXT / SI_CYCLE_TIME);
                }
                fExtensionFoV =
                    GDB_FILTER(fExtensionFoV, pObjInput->fLatTrackLimitL,
                               fFilterTimeConst);
            }

            /*! If not steering (EGO_DRV_INT_CURVE_GRAD_RAW) clearly in the
               other direction of the curvature,
                apply trace bracket extension */
            if (EGO_DRV_INT_CURVE_GRAD_RAW > -EGO_DRV_INT_CURVE_RAW) {
                /*! Set left trace bracket: Minimum of border to FoV and
                 * estimation of moved reflection point due to curve */
                pBracketOut->BracketPositionLeft = MIN_FLOAT(
                    (pObjInput->fDistY + fDistReflectionMoveDueToCurve),
                    fExtensionFoV);
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraExtensionRoadBorderCI */
static void SIRelTraExtensionRoadBorderCI(
    RelTraObjInput_t const *pObjInput,
    AssTraEnvironment_t const *pEnvironment,
    RelTraCurve_t const *pRelTraCurve,
    SIBracketOutput_t *pBracketOut) {
    if (pObjInput->iRelevant == (ubit8_t)TRUE) {
        if ((fABS(pObjInput->fVRelY) < MIN_OWV_VREL) ||
            (pObjInput->fDistX > MIN_ABST_VREL) ||
            ((EGO_SPEED_X_OBJ_SYNC + pObjInput->fVRelY) > MIN_OWV_VEIGEN)) {
            /* Oeffnen des rechten Spurrandes zum max. Erfassungsbereichs bei */
            /* Rechtskurve */

            if (pRelTraCurve->dCurve < -FSZ_A2_RICHTUNGSFAHRSPUREN_KI) {
                if (pEnvironment->iNumberLanesRight == 0L) {
                    /* Spurklammerposition schreiben !!! - TPGetSensorYPosition
                     * because we are not autosar here !!! */
                    /*! Right border of the FoV */
                    pBracketOut->BracketPositionRight =
                        SIRelTraGetYDistFOVAtXDist(pObjInput->fDistX,
                                                   SI_FAR_AND_NEAR_RANGE_RIGHT);
                }
            }

            /* dito bei Linkskurve */

            if (pRelTraCurve->dCurve > FSZ_A2_RICHTUNGSFAHRSPUREN_KI) {
                if (pEnvironment->iNumberLanesLeft == 0L) {
                    /* Spurklammerposition schreiben */
                    /*! Left border of the FoV */
                    pBracketOut->BracketPositionLeft =
                        SIRelTraGetYDistFOVAtXDist(pObjInput->fDistX,
                                                   SI_FAR_AND_NEAR_RANGE_LEFT);
                }
            }
        } else {
            /* keine Aktion */
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelCheckOWV */
static void SIRelCheckOWV(RelTraObjInput_t const *pObjInput,
                          RelTraObjOutput_t *pObjOutput,
                          AssTraEnvironment_t const *pEnvironment,
                          RelTraCurve_t const *pRelTraCurve) {
    /*--- VARIABLES ---*/
    float32 dSpurGradHalten;
    float32 dAbweichung = 0.f;
    sint32 iNumLanes;
    boolean bTraceCrit = FALSE;
    float32 fABS_pObjInput_fVRelY;
    const sint16 s_LCProb =
        SILCGetLaneChangeProbability(); /*!< Lane change probability */

    /* OWVKRIT_AKTIV is only set for the trajectory calculation with */
    /* SITrajectoryData. It is not recalculated for the VDY-only trajectory. */
    if (pRelTraCurve->iOWVflag == OWVKRIT_AKTIV) {
        if (pObjInput->iRelevant != (ubit8_t)TRUE) {
            /* Ruecksetzen der iLwFolge */
            pObjOutput->iLwFolge = 0;
        } else {
            const t_SILaneChangeCamPreMove t_LaneChangeMovePreState =
                SI_t_GetLaneChangeMovePre(); /*!< Lane change direction
                                                based on FIP lane matrix */

            /* Objektwinkel und Spurvektor haben gleiche Bewegungsrichtung */
            /* Erweitern der Spurbreite in Bewegungsrichtung bis zum max.
             * Erfassungsbereich */
            fABS_pObjInput_fVRelY = fABS(pObjInput->fVRelY);
            if ((pObjInput->fLatTrackLimitExpandFac >= (1.F - C_F32_DELTA)) ||
                (pObjInput->iObjectLifeCycles >= OWV_OBJLEB)) {
                if ((pObjInput->fDistX < MAX_ABSTSTEIG) &&
                    (EGO_SPEED_X_OBJ_SYNC > MIN_OWV_VEIGEN) &&
                    ((fABS_pObjInput_fVRelY < MIN_OWV_VREL) ||
                     (pObjInput->fDistX > MIN_ABST_VREL))) {
                    dAbweichung =
                        (pObjInput->fDistY - pObjInput->fRefCourseDistY);

                    /* Check trace criterion */
                    bTraceCrit = SICheckTraceCriterion(pObjInput->iObjNr);
                    if (bTraceCrit != FALSE) {
                        if (dAbweichung < 0.F) {
                            /* Movement to the right */
                            iNumLanes =
                                pEnvironment
                                    ->iNumberLanesRight; /* Eqvivalent to
                                                            EOGetLaneMatrixLaneNumberRight(&iNumLanes);
                                                            */
                            /* Check the OWV (follow object into curve/objekt
                             * weiter verfolgen) for following the object into
                             * right curve */
                            if (((SICourseData.fCurveGradient <
                                  -MIN_SPURSTEIG) &&
                                 (pObjInput->fOrientation >
                                  -(LOBE_ANGLE - C_F32_DELTA)) &&
                                 (pObjInput->fDistX > MIN_ABSTSTEIG)) ||
                                (((iNumLanes <= 0) ||
                                  (pObjOutput->iLwFolge >
                                   0)) /*!< No lane, or unknown number of
                                          lanes on the given side */
                                 && (SICourseData.fCurveGradient <
                                     0.f) /*!< Driver steering towards object */
                                 &&
                                 (fABS_pObjInput_fVRelY <
                                  (MAX_OWV_VREL_TRACE_CRIT)) /*!< Object's
                                                                relative speed
                                                                not too large */
                                 &&
                                 (dAbweichung >
                                  -SI_MAX_TRACE_LAT_DISPLACEMENT)) /*!< Object
                                                                      not too
                                                                      far away
                                                                      from
                                                                      predicted
                                                                      course */
                            ) {
                                /* Objekt und Kruemmung wandern nach rechts */
                                pObjOutput->iLwFolge = MAX_HALTEZEIT;
                            }
                        } else {
                            /* Movement to the left */
                            iNumLanes =
                                pEnvironment
                                    ->iNumberLanesLeft; /* Eqvivalent of
                                                           EOGetLaneMatrixLaneNumberLeft(&iNumLanes);
                                                           */
                            /* Ueberpruefung der OWV-Kriterien fuer Halten eines
                             * Objekts bei Bewegung nach nach links */
                            if (((SICourseData.fCurveGradient >
                                  MIN_SPURSTEIG) &&
                                 (pObjInput->fOrientation <
                                  (LOBE_ANGLE - C_F32_DELTA)) &&
                                 (pObjInput->fDistX > MIN_ABSTSTEIG)) ||
                                (((iNumLanes <= 0) ||
                                  (pObjOutput->iLwFolge <
                                   0)) /*!< No lane, or unknown number of
                                          lanes on the given side */
                                 && (SICourseData.fCurveGradient >
                                     0.f) /*!< Driver steering towards object */
                                 &&
                                 (fABS_pObjInput_fVRelY <
                                  (MAX_OWV_VREL_TRACE_CRIT)) /*!< Object's
                                                                relative speed
                                                                not too large */
                                 &&
                                 (dAbweichung <
                                  SI_MAX_TRACE_LAT_DISPLACEMENT)) /*!< Object
                                                                     not too far
                                                                     away from
                                                                     predicted
                                                                     course */
                            ) {
                                /* Objekt und Kruemmung wandern nach links */
                                pObjOutput->iLwFolge = -MAX_HALTEZEIT;
                            }
                        }
                    }

                    /* Check if ego lane change based on lane change probability
                     * or lane change direction in FIP lane matrix */
                    if ((ABS(s_LCProb) >
                         SI_MIN_LC_PROB_FOLLOW_OBJ_INTO_CURVE) ||
                        (t_LaneChangeMovePreState !=
                         LANE_CHANGE_CAM_PRE_MOVE_NO)) {
                        /* Bei Fahrspurwechsel kein OWV ausloesen */
                        pObjOutput->iLwFolge = 0L;
                    }

                    /* Schwellwert fuer SpurGradienten festlegen */

                    if (ABS(pObjInput->fOrientation) >
                        OWV_SPURGRAD_WINKEL_MAX) {
                        dSpurGradHalten = OWV_SPURGRAD_MIN_HALTEN_MAXWINKEL;
                    } else {
                        dSpurGradHalten =
                            dGDBmathLineareFunktion(&SpurGradHaltenKruemmung,
                                                    fABS(SICourseData.fCurve));
                    }

                    /* Objektverfolgung nach rechts, iLwFolge verarbeiten */

                    if (pObjOutput->iLwFolge > 0L) {
                        /* unter folgenden Bedingungen wird heruntergezaehlt */
                        if ((SICourseData.fCurveGradient > dSpurGradHalten) ||
                            (dAbweichung > -MIN_SPURABWEICH) ||
                            (dAbweichung < -MAX_SPURABWEICH)) {
                            pObjOutput->iLwFolge--;
                        }
                    }

                    /* Objektverfolgung nach links, iLwFolge verarbeiten */

                    if (pObjOutput->iLwFolge < 0L) {
                        /* unter folgenden Bedingungen wird heruntergezaehlt */
                        if ((SICourseData.fCurveGradient < -dSpurGradHalten) ||
                            (dAbweichung < MIN_SPURABWEICH) ||
                            (dAbweichung > MAX_SPURABWEICH)) {
                            pObjOutput->iLwFolge++;
                        }
                    }

                } else {
                    pObjOutput->iLwFolge = 0;
                }
            }
        }
    } else {
        _PARAM_UNUSED(pEnvironment);
        _PARAM_UNUSED(pRelTraCurve);
        _PARAM_UNUSED(dAbweichung);
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraExtensionFollowObjectIntoCurve */
static void SIRelTraExtensionFollowObjectIntoCurve(
    RelTraObjInput_t const *pObjInput,
    const RelTraObjOutput_t *pObjOutput,
    SIBracketOutput_t *pBracketOut,
    RelTraCurve_t const *pRelTraCurve) {
    /*--- VARIABLES ---*/
    float32 fDistYaxWinkel;
    float32 fABS_pObjInput_fVRelY;

    if (pRelTraCurve->iOWVflag == OWVKRIT_AKTIV) {
        if (pObjInput->iRelevant == (ubit8_t)TRUE) {
            /* Objektwinkel und Spurvektor haben gleiche Bewegungsrichtung */
            /* Erweitern der Spurbreite in Bewegungsrichtung bis zum max.
             * Erfassungsbereich */
            fABS_pObjInput_fVRelY = fABS(pObjInput->fVRelY);

            if ((pObjInput->fLatTrackLimitExpandFac >= (1.F - C_F32_DELTA)) ||
                (pObjInput->iObjectLifeCycles >= OWV_OBJLEB)) {
                if ((pObjInput->fDistX < MAX_ABSTSTEIG) &&
                    (EGO_SPEED_X_OBJ_SYNC > MIN_OWV_VEIGEN) &&
                    ((fABS_pObjInput_fVRelY < MIN_OWV_VREL) ||
                     (pObjInput->fDistX > MIN_ABST_VREL))) {
                    /* Objektverfolgung nach rechts, iLwFolge verarbeiten */
                    if (pObjOutput->iLwFolge > 0L) {
                        /* Max. Oeffnungsbreiten links und rechts bei analoger
                         * Winkelmessung */
                        /* Use the far range anlge only */
                        fDistYaxWinkel = SIRelTraGetYDistFOVAtXDist(
                            pObjInput->fDistX, SI_FAR_RANGE_ONLY_RIGHT);

                        /* Spurklammerposition schreiben */
                        pBracketOut->BracketPositionRight = MAX_FLOAT(
                            (pObjInput->fRefCourseDistY - SI_MAX_OVW_ABLAGE),
                            fDistYaxWinkel);
                    }

                    /* Objektverfolgung nach links, iLwFolge verarbeiten */
                    if (pObjOutput->iLwFolge < 0L) {
                        /* Max. Oeffnungsbreiten links und rechts bei analoger
                         * Winkelmessung */
                        /* Use the far range anlge only */
                        fDistYaxWinkel = SIRelTraGetYDistFOVAtXDist(
                            pObjInput->fDistX, SI_FAR_RANGE_ONLY_LEFT);
                        /* Spurklammerposition schreiben */
                        pBracketOut->BracketPositionLeft = MIN_FLOAT(
                            (pObjInput->fRefCourseDistY + SI_MAX_OVW_ABLAGE),
                            fDistYaxWinkel);
                    }
                }
            }
        }
    }
}
/*************************************************************************************************************************
  Functionname:    SICheckTraceCriterion */
static boolean SICheckTraceCriterion(const ObjNumber_t iObj) {
    CPDistanceWidth_t DistWidth;
    CPTrajOccupancy_t Occupancy;
    boolean ret;

    SICalculateTraceBorders(iObj, &DistWidth);
    CPCalculateOverlap(&DistWidth, &Occupancy);
    if (Occupancy.fObjectOccupancy > TraceOccupancyPickupThresh) {
        ret = TRUE;
    } else {
        ret = FALSE;
    }
    return ret;
}

/*************************************************************************************************************************
  Functionname:    SICalculateTraceBorders */
static void SICalculateTraceBorders(const ObjNumber_t iObj,
                                    CPDistanceWidth_t *pDistanceWidth) {
    const TraceID_t iObjTrace = OBJ_GET_STATIC_TRACE_ID(iObj);

    pDistanceWidth->fObjectWidth = EGO_VEHICLE_WIDTH;
    pDistanceWidth->fObjectWidthVar = 0.0f;

    if (iObjTrace < (TraceID_t)FIP_STATIC_TRACE_NO_OF_TRACES) {
        pDistanceWidth->fDistance =
            FIP_STATIC_TRACE_GET_YAXIS_PRED_INTERSECTION(iObjTrace);
        pDistanceWidth->fDistanceVar =
            FIP_STATIC_TRACE_GET_YAXIS_PRED_INTERSECTION_VAR(iObjTrace);
    } else {
        pDistanceWidth->fDistance = INITVALUE_BRACKETPOSITION;
        pDistanceWidth->fDistanceVar = INITVALUE_BRACKETPOSITION;
    }
    pDistanceWidth->fTrajectoryWidth = SIGetBaseSeekLaneWidth();
    pDistanceWidth->fTrajectoryWidthVar = 0.0f;
}
/*************************************************************************************************************************
  Functionname:    SIRelTraGetYDistFOVAtXDist */
static float32 SIRelTraGetYDistFOVAtXDist(float32 f_DistX,
                                          SICorrFOVTypeExtension_t t_FOVType) {
    float32 f_BeamTan;
    float32 f_AngleFOV = 0.f; /* Initialize FOV angle */

    /* Set FOV angle based on FOV type input parameter and given x-distance */
    /* Fist, consider left side of FOV */
    if ((t_FOVType == SI_FAR_RANGE_ONLY_LEFT) ||
        (t_FOVType == SI_FAR_AND_NEAR_RANGE_LEFT)) {
        /* If only the far range should be considered or if the x distance is
          higher than a threshold,
          consider the far range FOV angle */
        if ((t_FOVType == SI_FAR_RANGE_ONLY_LEFT) ||
            (f_DistX > SI_DISTX_FAR_RANGE_ANGLE_TB_EXT)) {
            f_AngleFOV = MAX_ANGLE;
        } else {
            /* Use near range angle */
            f_AngleFOV = SI_ANGLE_NEAR_RANGE_TB_EXT;
        }
    } /* Then consider right side of FOV */
    else {
        /* If only the far range should be considered or if the x distance is
          higher than a threshold,
          consider the far range FOV angle */
        if ((t_FOVType == SI_FAR_RANGE_ONLY_RIGHT) ||
            (f_DistX > SI_DISTX_FAR_RANGE_ANGLE_TB_EXT)) {
            f_AngleFOV = -MAX_ANGLE;
        } else {
            /* Use near range angle */
            f_AngleFOV = -SI_ANGLE_NEAR_RANGE_TB_EXT;
        }
    }

    /* Calculate tangents of FOV angle */
    f_BeamTan = TAN_(DEG2RAD(f_AngleFOV));

    return ((f_DistX * f_BeamTan) + SENSOR_Y_POSITION);
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */