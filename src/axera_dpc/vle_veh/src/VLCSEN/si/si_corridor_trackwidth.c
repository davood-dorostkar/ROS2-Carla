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
// #include "vlc_par.h"

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

/* Maximale Reduzierung der Normalen Spurbreite am Kurvenaussenrand bei */
/* Radien kleiner diesem Wert                                           */
#define SB_KA_RADIUS_MIN (1000.F)
/* Beginnende Reduzierung ab diesem Radius                              */
#define SB_KA_RADIUS_MAX (2500.F)
/* Naeher diesem Abstand wird keine Reduzierung durchgefuehrt           */
#define SB_KA_ABST_MIN (30.F)
/* Ab diesem Abstand volle Reduzierung                                  */
#define SB_KA_ABST_MAX (50.F)
#define SB_KA_RADIUS_BREITE (0.5F)

#define SI_MIN_HALF_SEEK_LANE_WIDTH_STAT \
    (1.5f) /*!< Minimum half seek lane width for stationary objects */
#define SI_SCALE_HALF_SEEK_LANE_WIDTH_STAT                                \
    (0.9f) /*!< Scale factor for seek lane width of statinoary objects if \
              stopped confidence low */

#define SI_TB_EXT_MIN_DIST (90.0f)
#define SI_TB_EXT_MAX_DIST (200.0f)
#define SI_TB_EXT_MIN_EXTENSION (1.8f)
#define SI_TB_EXT_MAX_EXTENSION (2.5f)

#define SI_TB_RB_EXT_MIN_NEAR_DIST (90.0f)
#define SI_TB_RB_EXT_MAX_NEAR_DIST (140.0f)
#define SI_TB_RB_EXT_MIN_NEAR_EXTENSION (0.0f)
#define SI_TB_RB_EXT_MAX_NEAR_EXTENSION (1.0f)

#define SI_TB_RB_EXT_MIN_DIST (140.0f)
#define SI_TB_RB_EXT_MAX_DIST (200.0f)
#define SI_TB_RB_EXT_MIN_EXTENSION (1.0f)
#define SI_TB_RB_EXT_MAX_EXTENSION (2.0f)

#define SI_TB_EXT_MIN_VREL (1.0f / C_KMH_MS)
#define SI_TB_EXT_MAX_VREL (-90.f / C_KMH_MS)
#define SI_TB_EXT_MAX_CURVATURE_INNER_LANE (0.0015f)
/*#define SI_TB_EXT_MAX_GRADIENT  (0.000001f)*/

#define SI_ADD_EXT_TUNNEL_MIN_REL_TIME (3.0f)
#define SI_ADD_EXT_TUNNEL_MAX_REL_TIME (6.0f)
#define SI_TUNNEL_EXT_MIN_EXTENSION (0.5f)
#define SI_TUNNEL_EXT_MAX_EXTENSION (1.2f)

#define SI_TUNNEL_EXT_MIN_EGO_VELOCITY (30.0f / C_KMH_MS)
#define SI_TUNNEL_EXT_MIN_OBJ_DIST (25.0f)
#define SI_TUNNEL_EXT_MIN_VREL (30.0f / C_KMH_MS)
#define SI_TUNNEL_EXT_MIN_XDIST (50.f)

#define SI_CUTIN_LAT_VEL_MIN (0.2f)
#define SI_CUTIN_LAT_VEL_MAX (0.8f)
#define SI_CUTIN_LAT_VEL_MIN_EXTENSION (0.0f)
#define SI_CUTIN_LAT_VEL_MAX_EXTENSION (1.5f)

#define SI_CUTIN_HEADING_ANGLE 2.85f
#define SI_CUTIN_LAT_VEL 0.25f
#define SI_CUTIN_LAT_DIST 1.f
#define SI_CUTIN_CURVE 0.005f
#define SI_CUTIN_XDIST_MIN 2.0f
#define SI_CUTIN_XDIST_MAX 50.0f

/*! Trace bracket extension for a straight course and an highspeed
    approach. The extension is approximated by the deviation of the
    trajectory given a curvature of . The resulting offset at 200 m
    is about 2.5m and at 120m about 0.75m. */
SET_MEMSEC_CONST(ExtensionHighspeedApproach)
static const GDBLFunction_t ExtensionHighspeedApproach = {

    SI_TB_EXT_MIN_EXTENSION, SI_TB_EXT_MAX_EXTENSION,
    (SI_TB_EXT_MAX_EXTENSION - SI_TB_EXT_MIN_EXTENSION) /
        (SI_TB_EXT_MAX_DIST - SI_TB_EXT_MIN_DIST),
    SI_TB_EXT_MIN_EXTENSION -
        (((SI_TB_EXT_MAX_EXTENSION - SI_TB_EXT_MIN_EXTENSION) /
          (SI_TB_EXT_MAX_DIST - SI_TB_EXT_MIN_DIST)) *
         SI_TB_EXT_MIN_DIST)};

/*! If the highspeed approach extension is applied, we need to check whether
  adjacent lanes exist. In order to be able to detect a lane change maneouver
  early,
  the trace brackets should not be selected too wide. */
SET_MEMSEC_CONST(ExtensionRoadBorderHighspeedApproach)
static const GDBLFunction_t ExtensionRoadBorderHighspeedApproach = {

    SI_TB_RB_EXT_MIN_EXTENSION, SI_TB_RB_EXT_MAX_EXTENSION,
    (SI_TB_RB_EXT_MAX_EXTENSION - SI_TB_RB_EXT_MIN_EXTENSION) /
        (SI_TB_RB_EXT_MAX_DIST - SI_TB_RB_EXT_MIN_DIST),
    SI_TB_RB_EXT_MIN_EXTENSION -
        (((SI_TB_RB_EXT_MAX_EXTENSION - SI_TB_RB_EXT_MIN_EXTENSION) /
          (SI_TB_RB_EXT_MAX_DIST - SI_TB_RB_EXT_MIN_DIST)) *
         SI_TB_RB_EXT_MIN_DIST)};

SET_MEMSEC_CONST(ExtensionRoadBorderNearDistHighspeedApproach)
static const GDBLFunction_t ExtensionRoadBorderNearDistHighspeedApproach = {

    SI_TB_RB_EXT_MIN_NEAR_EXTENSION, SI_TB_RB_EXT_MAX_NEAR_EXTENSION,
    (SI_TB_RB_EXT_MAX_NEAR_EXTENSION - SI_TB_RB_EXT_MIN_NEAR_EXTENSION) /
        (SI_TB_RB_EXT_MAX_NEAR_DIST - SI_TB_RB_EXT_MIN_NEAR_DIST),
    SI_TB_RB_EXT_MIN_NEAR_EXTENSION -
        (((SI_TB_RB_EXT_MAX_NEAR_EXTENSION - SI_TB_RB_EXT_MIN_NEAR_EXTENSION) /
          (SI_TB_RB_EXT_MAX_NEAR_DIST - SI_TB_RB_EXT_MIN_NEAR_DIST)) *
         SI_TB_RB_EXT_MIN_NEAR_DIST)};

/*! Trace bracket extension for high tunnel probability */
static const GDBLFunction_t ExtensionHighTunnelProb = {
    SI_TUNNEL_EXT_MIN_EXTENSION, SI_TUNNEL_EXT_MAX_EXTENSION,
    (SI_TUNNEL_EXT_MAX_EXTENSION - SI_TUNNEL_EXT_MIN_EXTENSION) /
        (SI_ADD_EXT_TUNNEL_MAX_REL_TIME - SI_ADD_EXT_TUNNEL_MIN_REL_TIME),
    SI_TUNNEL_EXT_MIN_EXTENSION -
        (((SI_TUNNEL_EXT_MAX_EXTENSION - SI_TUNNEL_EXT_MIN_EXTENSION) /
          (SI_ADD_EXT_TUNNEL_MAX_REL_TIME - SI_ADD_EXT_TUNNEL_MIN_REL_TIME)) *
         SI_ADD_EXT_TUNNEL_MIN_REL_TIME)};

static const GDBLFunction_t ExtensionObjectFastCutIn = {
    SI_CUTIN_LAT_VEL_MIN_EXTENSION, SI_CUTIN_LAT_VEL_MAX_EXTENSION,
    (SI_CUTIN_LAT_VEL_MAX_EXTENSION - SI_CUTIN_LAT_VEL_MIN_EXTENSION) /
        (SI_CUTIN_LAT_VEL_MAX - SI_CUTIN_LAT_VEL_MIN),
    SI_CUTIN_LAT_VEL_MIN_EXTENSION -
        (((SI_CUTIN_LAT_VEL_MAX_EXTENSION - SI_CUTIN_LAT_VEL_MIN_EXTENSION) /
          (SI_CUTIN_LAT_VEL_MAX - SI_CUTIN_LAT_VEL_MIN)) *
         SI_CUTIN_LAT_VEL_MIN)};
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
OLD SA
vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

/*-- SIRelTra.c --*/
/* Konstanten fuer SRandUnsicherAbstand */

/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
OLD SA
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/* Symbolic constants used by function SIRelTraAddExtensionRelevantObject */
#define ZVS_DETECT_VREL_DIFF_MAX (3.F / C_KMH_MS)
#define ZVS_DETECT_VEIGEN_MIN (30.F / C_KMH_MS)
#define ZVS_DETECT_VREL_MAX (50.F / C_KMH_MS)
#define ZVS_DETECT_RANGE_MAX (90.F)
#define ZVS_DETECT_RANGE_DIFF_MAX (10.F)
#define ZVS_DETECT_OBJ_LIFECYCLES_MIN (2u)
#define ZVS_DETECT_OBJ_LIFECYCLES_ACTIVATE (20u)
#define ZVS_DETECT_OBJ_RELTIME_MIN (2.F)
#define ZVS_DETECT_GRENZWINKEL_RELOBJ (LOBE_ANGLE * 0.75F)
#define ZVS_DETECT_GRENZWINKEL_NEIGHOBJ (1.5F)
#define VFR_LIFECYCLES_MIN (10u)
/*! Konstanten fuer Spurerweiterung des rel. Obj. bei Vorbeifahrten */
#define ABST_DIFF_VORBEIFAHRT_NAEHER (-10.F)
#define ABST_DIFF_VORBEIFAHRT_WEITER (3.F)
#define VREL_DIFF_VORBEIFAHRT (15.F / C_KMH_MS)
#define SEEKBREITE_VORBEIFAHRT (0.5F)
#define ZVS_HALTEZEIT_CRIT (23.F) /* (1.5F) */
#define ZVS_HALTEZEIT_MAX (38.F)  /* (2.5F) */
#define ZVS_HALTEZEIT_MIN (16.F)  /* (1.F) */
#define ZVS_VREL_MAX (20.F / C_KMH_MS)
#define ZVS_VREL_MIN (5.F / C_KMH_MS)

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/* Rampe zur Veraenderung der ZVS-Haltezeit in Abhaengigkeit der Vrel */
SET_MEMSEC_CONST(ZVSHaltezeitVrel)
static const GDBLFunction_t ZVSHaltezeitVrel = {

    ZVS_HALTEZEIT_MAX, /*!< A1 */
    ZVS_HALTEZEIT_MIN, /*!< A2 */
    /*! (A2-A1)/(E2-E1) */
    (ZVS_HALTEZEIT_MIN - ZVS_HALTEZEIT_MAX) / (ZVS_VREL_MAX - ZVS_VREL_MIN),
    /*! A1 - (A2-A1)/(E2-E1) * E1 */
    ZVS_HALTEZEIT_MAX - (((ZVS_HALTEZEIT_MIN - ZVS_HALTEZEIT_MAX) /
                          (ZVS_VREL_MAX - ZVS_VREL_MIN)) *
                         ZVS_VREL_MIN)};

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void SIRelTraAddRestrictionCurveOuterBorder(
    RelTraObjInput_t const *pObjInput,
    RelTraCurve_t const *pRelTraCurve,
    SIBracketOutput_t *pBracketOut);

static void SIRelTraAddExtensionRelevantObject(
    RelTraObjInput_t const *pObjInput,
    RelTraObjOutput_t *pObjOutput,
    SIBracketOutput_t *pBracketOut);

static void SIRelTraAddExtensionHighspeedApproach(
    RelTraObjInput_t const *pObjInput,
    const AssTraEnvironment_t *pEnvironment,
    SIBracketOutput_t *pBracketOut,
    RelTraCurve_t const *pRelTraCurve);

static void SIRelTraAddExtensionHighTunnelProb(
    RelTraObjInput_t const *pObjInput,
    SIBracketOutput_t *pBracketOut,
    RelTraCurve_t const *pRelTraCurve);

static void SIRelTraAddExtensionObjectFastCutIn(
    RelTraObjInput_t const *pObjInput,
    SIBracketOutput_t *pBracketOut,
    RelTraCurve_t const *pRelTraCurve);

/*************************************************************************************************************************
  Functionname:    SICalculateCorridorBaseBrackets */
void SICalculateCorridorBaseBrackets(
    const RelTraObjInput_t *const pObjectProperties,
    const RelTraCurve_t *const pTrajectory,
    SIBracketOutput_t *const pBaseBrackets,
    const SIBracketFuncEnable_t *const pBracketFuncEnableFlags) {
    /* Lanewidth as a function of road type and ego velocity */
    float32 fSISeekLaneWidth = SIGetBaseSeekLaneWidth();
    float32 fSeekHalfLaneWidth = fSISeekLaneWidth * 0.5f;
    float32 fTrackHalfLaneWidth = get_fSITrackLaneWidth() * 0.5f;
    float32 fBaseHalfLaneWidth = fSeekHalfLaneWidth;

    /* Moving objects */
    if (pObjectProperties->ucDynamicProperty == CR_OBJECT_PROPERTY_MOVING) {
        /* Use the larger extension factor of the two */
        float32 ExtensionFactor =
            MAX_FLOAT(pObjectProperties->fLatTrackLimitExpandFac,
                      pObjectProperties->fLatTrackLimitDistanceExpandFac);

        /* Check if object is relevant or has been relevant recently */
        if ((ExtensionFactor > C_F32_DELTA) &&
            (pBracketFuncEnableFlags->bUseSeekLaneWidthOnly == FALSE)) {
            /* Linear interpolation between TrackLaneWidth and SeekLaneWidth */
            fBaseHalfLaneWidth =
                fSeekHalfLaneWidth +
                (ExtensionFactor * (fTrackHalfLaneWidth - fSeekHalfLaneWidth));
        } else {
            /* Object is not relevant or has not been recently */
            fBaseHalfLaneWidth = fSeekHalfLaneWidth;
        }
    }
    /* Stationary objects */
    else if (pObjectProperties->ucDynamicProperty ==
             CR_OBJECT_PROPERTY_STATIONARY) {
        /* Slightly different calculation whether default trajectory of
         * alternative trajectory is used */
        if (pTrajectory->iOWVflag == OWVKRIT_AKTIV) {
            if (pObjectProperties->uiStoppedConfidence >=
                VLC_AVLC_PAR_OBJ_STOPPED_MIN_CONF) {
                fBaseHalfLaneWidth = fSeekHalfLaneWidth;
            } else if ((OBJ_CLASSIFICATION(pObjectProperties->iObjNr) !=
                        CR_OBJCLASS_POINT) ||
                       (pObjectProperties->fDistX <
                        SI_PAR_MAX_DISTX_STAT_POINT_SMALL_SEEK)) {
                /* For stationary objects limit the seek lane width. As most
                roadside stationaries are seen
                as 'point' class objects in large distances, limit lane width
                more for far away points.
                Note: in tight turns valid in-lane objects also tend to be
                points, thus do not limit near
                range objects that much */
                /* Limit the seek lane width to 'SI_AVLC_TRAJECTORY_WIDTH_STAT'
                 * at most */
                fBaseHalfLaneWidth =
                    MIN_FLOAT(fSISeekLaneWidth, SI_AVLC_TRAJECTORY_WIDTH_STAT) *
                    0.5F;
            } else {
                /* Limit the seek lane width to
                 * 'SI_AVLC_TRAJECTORY_NARROWWIDTH_STAT' at most */
                fBaseHalfLaneWidth =
                    MIN_FLOAT(fSISeekLaneWidth,
                              SI_AVLC_TRAJECTORY_NARROWWIDTH_STAT) *
                    0.5F;
            }
        } else if (pTrajectory->iOWVflag == OWVKRIT_INAKTIV) {
            if (pObjectProperties->uiStoppedConfidence >=
                VLC_AVLC_PAR_OBJ_STOPPED_MIN_CONF) {
                fBaseHalfLaneWidth = MAX_FLOAT(
                    fSeekHalfLaneWidth, SI_MIN_HALF_SEEK_LANE_WIDTH_STAT);
            } else {
                fBaseHalfLaneWidth =
                    SI_SCALE_HALF_SEEK_LANE_WIDTH_STAT * fSeekHalfLaneWidth;
            }
        } else {
        }
    } else {
    }

    pBaseBrackets->BracketPositionLeft =
        pObjectProperties->fRefCourseDistY + fBaseHalfLaneWidth;
    pBaseBrackets->BracketPositionRight =
        pObjectProperties->fRefCourseDistY - fBaseHalfLaneWidth;
}

/*************************************************************************************************************************
  Functionname:    SIExecuteAdditiveBracketFunctions */

void SIExecuteAdditiveBracketFunctions(
    const RelTraObjInput_t *const pObjectProperties,
    RelTraObjOutput_t *const pObjectVariables,
    const RelTraCurve_t *const pTrajectory,
    const AssTraEnvironment_t *pEnvironment,
    SICriteriaMatrix_t *const pBracketFuncResults,
    const SIBracketFuncEnable_t *const pBracketFuncEnableFlags) {
    /*********************** RESTRICTIONS ****************************/
    /* Calculate relative restrictions */
    if ((pObjectProperties->ucDynamicProperty ==
         CR_OBJECT_PROPERTY_STATIONARY) ||
        (OBJ_IS_MOVING_TO_STATIONARY(pObjectProperties->iObjNr))) {
    } /* ONLY for Stationary or Stopped objects */

    /* ONLY for Moving or Stopped Objects */
    if ((pObjectProperties->ucDynamicProperty == CR_OBJECT_PROPERTY_MOVING) ||
        ((pObjectProperties->ucDynamicProperty ==
          CR_OBJECT_PROPERTY_STATIONARY) &&
         (OBJ_IS_MOVING_TO_STATIONARY(pObjectProperties->iObjNr)))) {
        /* Calculate relative restrictions */
        if (pBracketFuncEnableFlags->bEnableAddRestrictionCurveOuterBorder !=
            SI_CORR_BRACKET_FUNC_DISABLED) {
            SIRelTraAddRestrictionCurveOuterBorder(
                pObjectProperties, pTrajectory,
                &pBracketFuncResults->AddRestrictionCurveOuterBorder);
        }

        /*********************** EXTENSIONS ******************************/
        /* Calculate relative extensions */

        if (pBracketFuncEnableFlags->bEnableAddExtensionRelevantObject !=
            SI_CORR_BRACKET_FUNC_DISABLED) {
            SIRelTraAddExtensionRelevantObject(
                pObjectProperties, pObjectVariables,
                &pBracketFuncResults->AddExtensionRelevantObject);
        }

        if (pBracketFuncEnableFlags->bEnableExtensionHighspeedApproach !=
            SI_CORR_BRACKET_FUNC_DISABLED) {
            SIRelTraAddExtensionHighspeedApproach(
                pObjectProperties, pEnvironment,
                &pBracketFuncResults->AddExtensionHighspeedApproach,
                pTrajectory);
        }

        if (pBracketFuncEnableFlags->bEnableAddExtensionHighTunnelProb !=
            SI_CORR_BRACKET_FUNC_DISABLED) {
            SIRelTraAddExtensionHighTunnelProb(
                pObjectProperties,
                &pBracketFuncResults->AddExtensionHighTunnelProb, pTrajectory);
        }

        if (pBracketFuncEnableFlags->bEnableAddExtensionObjectFastCutIn !=
            SI_CORR_BRACKET_FUNC_DISABLED) {
            SIRelTraAddExtensionObjectFastCutIn(
                pObjectProperties,
                &pBracketFuncResults->AddExtensionObjectFastCutIn, pTrajectory);
        }
    } /* ONLY for Moving or Stopped objects */
}

/*************************************************************************************************************************
  Functionname:    SIRelTraAddRestrictionCurveOuterBorder */
static void SIRelTraAddRestrictionCurveOuterBorder(
    RelTraObjInput_t const *pObjInput,
    RelTraCurve_t const *pRelTraCurve,
    SIBracketOutput_t *pBracketOut) {
    /*--- VARIABLES ---*/
    float32 dRadiusKA;
    float32 dRadius;
    float32 dRadiusFaktor, dAbstFaktor;

    float32 ExtensionFactor =
        MAX_FLOAT(pObjInput->fLatTrackLimitExpandFac,
                  pObjInput->fLatTrackLimitDistanceExpandFac);

    /* Check if object is not relevant (ExtensionFactor < C_F32_Delta) and
       the absolute angle w.r.t. the object is beyond the field of view of one
       sidelobe (LOBE_ANGLE)
    */
    if ((ExtensionFactor < C_F32_DELTA) &&
        (ABS(pObjInput->fOrientation) > (LOBE_ANGLE - C_F32_DELTA))) {
        /* Beeinflussung des Kurvenaeusseren-Fahrbahnrandes durch den Spurradius
         * im Seek-Fall     */

        /* Abstandsfaktor bei Abstand < SB_KA_ABST_MIN (30m) = 0.0 */
        /*                    Abstand >= SB_KA_ABST_MIN & Abstand <=
         * SB_KA_ABST_MAX = 0.0 bis 1.0 */
        /*                    Abstand > SB_KA_ABST_MAX (50m) = 1.0 */

        dAbstFaktor = ((pObjInput->fDistX - SB_KA_ABST_MIN) *
                       (1.F / (SB_KA_ABST_MAX - SB_KA_ABST_MIN)));
        dAbstFaktor = MINMAX_FLOAT(0.F, 1.F, dAbstFaktor);

        /* Radiusfaktor bei Radius_abs < SB_KA_RADIUS_MIN (1000m) = 1.0 */
        /*                  Radius_abs >= SB_KA_RADIUS_MIN & Radius_abs <=
         * SB_KA_RADIUS_MAX = 1.0 bis 0.0 */
        /*                  Radius_abs > SB_KA_RADIUS_MAX = 0.0 */

        dRadius = A2_TO_RAD(pRelTraCurve->dCurve);

        dRadiusFaktor = 1.F - ((fABS(dRadius) - SB_KA_RADIUS_MIN) *
                               (1.F / (SB_KA_RADIUS_MAX - SB_KA_RADIUS_MIN)));
        dRadiusFaktor = MINMAX_FLOAT(0.F, 1.F, dRadiusFaktor);

        /* Der minimale Faktor aus Radius und Abstand den Multiplikator zur
         * Verkleinerung des Kurvenaussen- */
        /* Fahrbahnrandes */

        dRadiusKA = MIN_FLOAT(dRadiusFaktor, dAbstFaktor);

        /* Faktor mit tatsaechlicher Breite in m multiplizieren */

        dRadiusKA *= SB_KA_RADIUS_BREITE;

        /* Faktor mit groesserer Auswirkung auf Fahrbahnbreite aus Drift und
         * Spurradius ist am Kurvenaussenrand wirksam */
        /* Kurvenaussenrand-Seite wird in Abhaengigkeit vom Radius bestimmt.
         * Beschraenkung ist nur dort wirksam         */

        if (dRadiusKA > C_F32_DELTA) {
            if (dRadius > C_F32_DELTA) {
                /* Linkskurve, rechter KA muss beeinflusst werden */
                /* Spurklammerposition schreiben */
                pBracketOut->BracketPositionRight = dRadiusKA;

            } else if (dRadius < -C_F32_DELTA) {
                /* Rechtskurve, linker KA muss beeinflusst werden */
                /* Spurklammerposition schreiben */
                pBracketOut->BracketPositionLeft = -dRadiusKA;

            } else {
                /* Radius = 0 (gesetzt) bedeutet Geradeaus, hier wird der
                 * Kurvenaussenrand nicht bearbeitet. */
            }
        } else {
            /* Keine Begrenzung notwenig */
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraAddExtensionRelevantObject */
static void SIRelTraAddExtensionRelevantObject(
    RelTraObjInput_t const *pObjInput,
    RelTraObjOutput_t *pObjOutput,
    SIBracketOutput_t *pBracketOut) {
    boolean bVorbeifahrtLinks, bVorbeifahrtRechts;
    float32 dAbstandDifferenz;
    float32 dWinkelDifferenz;
    float32 dVrelDifferenz;
    sint32 i;
    float32 dHaltezeitTemp;
    float32 dHaltezeit;

    /*! Number of lanes left/right*/
    const sint8 s_NumLaneLeft = FIP_s_GetLMLeftNumLane();
    const sint8 s_NumLaneRight = FIP_s_GetLMRightNumLane();
    bVorbeifahrtLinks = FALSE;
    bVorbeifahrtRechts = FALSE;

    if (pObjInput->fLatTrackLimitExpandFac > C_F32_DELTA) {
        /* Befinden sich Nachbarobjekte im Umfeld dieses Objekts */
        for (i = (Envm_N_OBJECTS - 1); i >= 0L; i--) {
            if ((i != pObjInput->iObjNr) &&
                (OBJ_DYNAMIC_PROPERTY(i) == CR_OBJECT_PROPERTY_MOVING)) {
                dVrelDifferenz = fABS(OBJ_LONG_VREL(i) - pObjInput->fVRelY);

                if (dVrelDifferenz <= VREL_DIFF_VORBEIFAHRT) {
                    dAbstandDifferenz =
                        OBJ_LONG_DISPLACEMENT(i) - pObjInput->fDistX;

                    /* ZVS-Situation erkennen */
                    if ((pObjInput->iRelevant == (ubit8_t)TRUE) &&
                        (EGO_SPEED_X_OBJ_SYNC > ZVS_DETECT_VEIGEN_MIN) &&
                        (pObjInput->fVRelY < ZVS_DETECT_VREL_MAX) &&
                        (pObjInput->fVRelY > (-ZVS_DETECT_VREL_MAX)) &&
                        (pObjInput->fDistX < ZVS_DETECT_RANGE_MAX) &&
                        (dVrelDifferenz < ZVS_DETECT_VREL_DIFF_MAX) &&
                        (dAbstandDifferenz < ZVS_DETECT_RANGE_DIFF_MAX) &&
                        (dAbstandDifferenz > (-ZVS_DETECT_RANGE_DIFF_MAX)) &&
                        (OBJ_IS_SHADOW(i) == FALSE) &&
                        (pObjInput->fRelevantTime >
                         ZVS_DETECT_OBJ_RELTIME_MIN) &&
                        (ABS(pObjInput->fOrientation) <
                         ZVS_DETECT_GRENZWINKEL_RELOBJ) &&
                        (((OBJ_LIFECYCLES(i) >
                           ZVS_DETECT_OBJ_LIFECYCLES_ACTIVATE) &&
                          (((OBJ_ANGLE(i) <= ZVS_DETECT_GRENZWINKEL_NEIGHOBJ) &&
                            (OBJ_ANGLE(i) >=
                             (-ZVS_DETECT_GRENZWINKEL_NEIGHOBJ))) ||
                           ((OBJ_ANGLE(i) < -ZVS_DETECT_GRENZWINKEL_NEIGHOBJ) &&
                            (s_NumLaneRight != 0L)) ||
                           ((OBJ_ANGLE(i) > ZVS_DETECT_GRENZWINKEL_NEIGHOBJ) &&
                            (s_NumLaneLeft != 0L)))) ||
                         (pObjOutput->fTargetFusionHoldTime > C_F32_DELTA))) {
                        /* Haltezeit in Abhaengigkeit der Vrel ermitteln */
                        dHaltezeitTemp = dGDBmathLineareFunktion(
                            &ZVSHaltezeitVrel, fABS(pObjInput->fVRelY));

                        dHaltezeit = (float32)ROUND_TO_INT(
                            dHaltezeitTemp); /*ROUND not needed here?!*/

                        /* Bei detektierten Nachbarobjekten mit sehr geringer
                         * Lebensdauer */
                        /* kuerzere Haltezeit auswaehlen */
                        if (OBJ_LIFECYCLES(i) > ZVS_DETECT_OBJ_LIFECYCLES_MIN) {
                            /* Haltezeit uebernehmen, wenn ermittelte Haltezeit
                             * groesser als bisherige */
                            pObjOutput->fTargetFusionHoldTime = MAX_FLOAT(
                                pObjOutput->fTargetFusionHoldTime, dHaltezeit);
                        } else {
                            /* Vrel-abhaengige Haltezeit auf ZVS_HALTEZEIT_CRIT
                             * begrenzen */
                            dHaltezeit =
                                MIN_FLOAT(ZVS_HALTEZEIT_CRIT, dHaltezeit);

                            /* Haltezeit uebernehmen, wenn ermittelte Haltezeit
                             * groesser als bisherige */
                            pObjOutput->fTargetFusionHoldTime = MAX_FLOAT(
                                pObjOutput->fTargetFusionHoldTime, dHaltezeit);
                        }
                    } else {
                        /* keine ZVS-Situation */
                    }

                    if ((dAbstandDifferenz >= ABST_DIFF_VORBEIFAHRT_NAEHER) &&
                        (dAbstandDifferenz <= ABST_DIFF_VORBEIFAHRT_WEITER) &&
                        (OBJ_LIFECYCLES(i) > VFR_LIFECYCLES_MIN)) {
                        dWinkelDifferenz =
                            OBJ_ANGLE(i) - pObjInput->fOrientation;

                        if ((dWinkelDifferenz <= 0.F) &&
                            (bVorbeifahrtRechts == FALSE)) {
                            /* Vorbeifahrendes Objekt liegt rechts vom rel. Obj
                             */
                            /* Rechte Begrenzung */
                            /* Spurklammerposition schreiben */
                            pBracketOut->BracketPositionRight =
                                -SEEKBREITE_VORBEIFAHRT;

                            bVorbeifahrtRechts = TRUE;

                        } else if ((dWinkelDifferenz >= 0.F) &&
                                   (bVorbeifahrtLinks == FALSE)) {
                            /* Vorbeifahrendes Objekt liegt links vom rel. Obj
                             */
                            /* Linke Begrenzung */
                            /* Spurklammerposition schreiben */
                            pBracketOut->BracketPositionLeft =
                                +SEEKBREITE_VORBEIFAHRT;

                            bVorbeifahrtLinks = TRUE;
                        } else {
                        }
                    }
                }
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraAddExtensionHighspeedApproach */
static void SIRelTraAddExtensionHighspeedApproach(
    RelTraObjInput_t const *pObjInput,
    const AssTraEnvironment_t *pEnvironment,
    SIBracketOutput_t *pBracketOut,
    RelTraCurve_t const *pRelTraCurve) {
    float32 fExtendBrackets = 0.0f;
    float32 fExtendBracketsLeft, fExtendBracketsRight;

    const SIScaleBracketState_t LC_State = SIReturnStateScaleBracket();

    /* only for the relevant object   */
    /* only for objects which we are (almost) approaching (vrel < 1 km/h) */
    /* only for an almost straight curse (radius < 2000m) */
    /* no lane change is detected    */
    /* only for a velocity > 145 km/h */
    /* object is more than 90m away   */
    if ((pObjInput->iRelevant == (ubit8_t)TRUE) &&
        (pObjInput->fVRelY < SI_TB_EXT_MIN_VREL) &&
        (pObjInput->fVRelY > SI_TB_EXT_MAX_VREL)
        /* && (pRelTraCurve->dCurve_abs  < SI_TB_EXT_MAX_CURVATURE )*/
        && ((LC_State == NO_LANE_CHANGE) || (LC_State == UNKNOWN)) &&
        (EGO_SPEED_X_OBJ_SYNC > SI_TB_EXT_MIN_VELOCITY) &&
        (pObjInput->fDistX > SI_TB_EXT_MIN_DIST)) {
        // Default trace bracket extension
        fExtendBrackets = dGDBmathLineareFunktion(&ExtensionHighspeedApproach,
                                                  pObjInput->fDistX);
        fExtendBracketsLeft = fExtendBrackets;
        fExtendBracketsRight = -fExtendBrackets;

        if (pRelTraCurve->dCurve_abs < SI_TB_EXT_MAX_CURVATURE) {
            // Set full extension first and then check whether we are on an
            // outer lane
            // If neighboring lanes exist, we want to be able to detect a lane
            // change of the
            // object earlier.
            if (pEnvironment->iNumberLanesLeft > 0) {
                // There is a left lane, thus restrict the left tracebracket
                // Two-segmented approach. The corridor extension is larger for
                // objects which have a larger distance
                if (pObjInput->fDistX > SI_TB_RB_EXT_MAX_NEAR_DIST) {
                    fExtendBracketsLeft = dGDBmathLineareFunktion(
                        &ExtensionRoadBorderHighspeedApproach,
                        pObjInput->fDistX);
                } else {
                    fExtendBracketsLeft = dGDBmathLineareFunktion(
                        &ExtensionRoadBorderNearDistHighspeedApproach,
                        pObjInput->fDistX);
                }
            }

            if (pEnvironment->iNumberLanesRight > 0) {
                // There is a right lane, thus restrict the right tracebracket
                // Two-segmented approach. The corridor extension is larger for
                // objects which have a larger distance
                if (pObjInput->fDistX > SI_TB_RB_EXT_MAX_NEAR_DIST) {
                    fExtendBracketsRight = -dGDBmathLineareFunktion(
                        &ExtensionRoadBorderHighspeedApproach,
                        pObjInput->fDistX);
                } else {
                    fExtendBracketsRight = -dGDBmathLineareFunktion(
                        &ExtensionRoadBorderNearDistHighspeedApproach,
                        pObjInput->fDistX);
                }
            }
        } else if ((pRelTraCurve->dCurve >
                    -SI_TB_EXT_MAX_CURVATURE_INNER_LANE) &&
                   (pRelTraCurve->dCurve < 0.0f) &&
                   (pEnvironment->iNumberLanesRight == 0)) {
            /* Stronger right curve, but we are on the right outer lane.
             * Therefore, extend only the right trace bracket */
            fExtendBracketsLeft = 0.0f;
        } else if ((pRelTraCurve->dCurve <
                    SI_TB_EXT_MAX_CURVATURE_INNER_LANE) &&
                   (pRelTraCurve->dCurve > 0.0f) &&
                   (pEnvironment->iNumberLanesLeft == 0)) {
            /* Stronger left curve, but we are on the left outer lane.
             * Therefore, extend only the left trace bracket */
            fExtendBracketsRight = 0.0f;
        } else {
            /* Curvature not fulfilled. Set extension to zero. */
            fExtendBracketsLeft = 0.0f;
            fExtendBracketsRight = 0.0f;
        }

        pBracketOut->BracketPositionLeft = fExtendBracketsLeft;
        pBracketOut->BracketPositionRight = fExtendBracketsRight;
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraAddExtensionHighTunnelProb */
static void SIRelTraAddExtensionHighTunnelProb(
    RelTraObjInput_t const *pObjInput,
    SIBracketOutput_t *pBracketOut,
    RelTraCurve_t const *pRelTraCurve) {
    float32 fExtendBrackets = 0.0f;
    float32 fExtendBracketsLeft, fExtendBracketsRight;

    const SIScaleBracketState_t LC_State = SIReturnStateScaleBracket();

    /* only at high tunnel probability*/
    /* only for the ACC trajectory    */
    /* only for the relevant object   */
    /* only for objects with low vrel (vrel < 15 km/h) */
    /* only in high distances */
    /* no lane change is detected    */
    /* only for a velocity > 30 km/h */
    /* object is more than 45m away   */
    if ((TUNNEL_PROBABILITY >= 0.5f) &&
        (pRelTraCurve->iOWVflag == OWVKRIT_AKTIV) &&
        (pObjInput->iRelevant == (ubit8_t)TRUE) &&
        (pObjInput->fVRelY < SI_TUNNEL_EXT_MIN_VREL) &&
        (OBJ_LONG_DISPLACEMENT(pObjInput->iObjNr) > SI_TUNNEL_EXT_MIN_XDIST) &&
        ((LC_State == NO_LANE_CHANGE) || (LC_State == UNKNOWN)) &&
        (EGO_SPEED_X_OBJ_SYNC > SI_TUNNEL_EXT_MIN_EGO_VELOCITY) &&
        (pObjInput->fDistX > SI_TUNNEL_EXT_MIN_OBJ_DIST)) {
        /* trace bracket extension */
        fExtendBrackets = dGDBmathLineareFunktion(&ExtensionHighTunnelProb,
                                                  pObjInput->fRelevantTime);
        fExtendBracketsLeft = fExtendBrackets;
        fExtendBracketsRight = -fExtendBrackets;

        pBracketOut->BracketPositionLeft = fExtendBracketsLeft;
        pBracketOut->BracketPositionRight = fExtendBracketsRight;
    }
}

void SIRelTraAddExtensionObjectFastCutIn(RelTraObjInput_t const *pObjInput,
                                         SIBracketOutput_t *pBracketOut,
                                         RelTraCurve_t const *pRelTraCurve) {
    if (pObjInput->fDistX > SI_CUTIN_XDIST_MIN &&
        pObjInput->fDistX < SI_CUTIN_XDIST_MAX &&
        fABS(pRelTraCurve->dCurve) < SI_CUTIN_CURVE) {
        if (((pObjInput->fOrientation > SI_CUTIN_HEADING_ANGLE ||
              pObjInput->fVelY > SI_CUTIN_LAT_VEL) &&
             pObjInput->fDistToCourse < -SI_CUTIN_LAT_DIST) ||
            ((pObjInput->fOrientation < -SI_CUTIN_HEADING_ANGLE ||
              pObjInput->fVelY < -SI_CUTIN_LAT_VEL) &&
             pObjInput->fDistToCourse > SI_CUTIN_LAT_DIST)) {
            pBracketOut->BracketPositionLeft = dGDBmathLineareFunktion(
                &ExtensionObjectFastCutIn, pObjInput->fVelY);
            pBracketOut->BracketPositionRight = -dGDBmathLineareFunktion(
                &ExtensionObjectFastCutIn, pObjInput->fVelY);
        }
    }
}

/* ************************************************************************* */
/*   Copyright                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */