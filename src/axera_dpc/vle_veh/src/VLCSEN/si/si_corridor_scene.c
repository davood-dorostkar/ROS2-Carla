/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "si.h"
#include "si_corridor_crit.h"
#include "si_par.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/* Distance around the relevant object */
#define DIST_DIFF_NEIGHBORHOOD_REL_OBJ (5.0F + C_F32_DELTA)
/* Seek width when passing the relevant object */
#define SEEKWIDTH_NEIGHBORHOOD_RELOBJ (0.2F)

/*! maximal extension in situations which are clear */
#define MAX_LOW_SPEED_FUSEDBRD_EXTENSION (3.5f)
/*! minimal extension in situations which are unclear, i.e. signals
 * contradictory */
#define MIN_LOW_SPEED_FUSEDBRD_EXTENSION (2.5f)
/*! default extension */
#define DEFAULT_LOW_SPEED_FUSEDBRD_EXTENSION (3.7f)
/*! maximum speed up to which extension is executed */
#define MAX_SPEED_LOW_SPEED_FUSEDBRD_EXTENSION (25.f / C_KMH_MS)
/*! maximum speed up to which default extension is executed */
#define MAX_SPEED_DEFAULT_LOW_SPEED_FUSEDBRD_EXTENSION (5.f / C_KMH_MS)
/*! maximum distance of relevant object up to which extension is executed */
#define MAX_DISTX_LOW_SPEED_FUSEDBRD_EXTENSION (20.f)
/*! maximum distance for default extension */
#define MAX_DISTX_DEFAULT_LOW_SPEED_FUSEDBRD_EXTENSION (10.f)
/*! maximum gap between fused road border and relevant object */
#define MAX_GAP_LOW_SPEED_FUSEDBRD_EXTENSION (4.5f)
#define MIN_GAP_LOW_SPEED_FUSEDBRD_EXTENSION (2.5f)
/* maximum curve radius up to which extension is executed */
#define MAX_CURVE_RADIUS_LOW_SPEED_FUSEDBRD_EXTENSION (0.1f)

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

static void SIRelTraRestrictNeighbourhoodRelObj(
    RelTraObjInput_t const *p_ObjInput,
    AssTraEnvironment_t const *p_Environment,
    SIBracketOutput_t *p_BracketOut);

/*************************************************************************************************************************
  Functionname:    SI_v_ExecuteSceneBracketFuncstions */
void SI_v_ExecuteSceneBracketFuncstions(
    RelTraObjInput_t const *p_ObjectProperties,
    AssTraEnvironment_t const *const p_Environment,
    SIBracketFuncEnable_t const *p_BracketFuncEnableFlags,
    SICriteriaMatrix_t *p_BracketFuncResults) {
    /* RestrictionNeighbourhoodRelObj */
    if (p_BracketFuncEnableFlags->bEnableRestrictionNeighbourhoodRelObj !=
        SI_CORR_BRACKET_FUNC_DISABLED) {
        SIRelTraRestrictNeighbourhoodRelObj(
            p_ObjectProperties, p_Environment,
            &p_BracketFuncResults->RestrictionNeighbourhoodRelObj);
    }
}

/*************************************************************************************************************************
  Functionname:    SIRelTraRestrictNeighbourhoodRelObj */
static void SIRelTraRestrictNeighbourhoodRelObj(
    RelTraObjInput_t const *p_ObjInput,
    AssTraEnvironment_t const *p_Environment,
    SIBracketOutput_t *p_BracketOut) {
    /*--- VARIABLES ---*/
    float32 f_LatDistDiff;
    float32 f_LongDistDiff;
    float32 fABS_pEnvironment_fRelObjTargetFusionHoldTime;

    if (p_Environment->iRelObjNr != OBJ_INDEX_NO_OBJECT) {
        if ((p_Environment->ucRelObjDynamicProperty ==
             CR_OBJECT_PROPERTY_MOVING) &&
            (p_Environment->iRelObjNr != p_ObjInput->iObjNr) &&
            (p_ObjInput->ucDynamicProperty == CR_OBJECT_PROPERTY_MOVING)) {
            f_LongDistDiff = p_Environment->fRelObjDistX - p_ObjInput->fDistX;

            /* Restriction: only for object for which the longitudinal distance
               is less than the longitudinal distance of the relevant object
               or if the relevant object was neighboring object during object
               separation (Zielverschmelzer) (if this extension is considered
               as shown by the config switch) */
            fABS_pEnvironment_fRelObjTargetFusionHoldTime =
                fABS(p_Environment->fRelObjTargetFusionHoldTime);
            if ((f_LongDistDiff > C_F32_DELTA) ||
                (fABS_pEnvironment_fRelObjTargetFusionHoldTime <=
                 C_F32_DELTA)) {
                if (fABS(f_LongDistDiff) <= DIST_DIFF_NEIGHBORHOOD_REL_OBJ) {
                    f_LatDistDiff =
                        p_Environment->fRelObjDistY - p_ObjInput->fDistY;

                    if (f_LatDistDiff > 0.F) {
                        /* Right boundary */
                        /* Write trace bracket position */
                        p_BracketOut->BracketPositionRight =
                            (p_ObjInput->fRefCourseDistY -
                             SEEKWIDTH_NEIGHBORHOOD_RELOBJ);

                    } else {
                        /* Left boundary */
                        /* Write trace bracket position */
                        p_BracketOut->BracketPositionLeft =
                            (p_ObjInput->fRefCourseDistY +
                             SEEKWIDTH_NEIGHBORHOOD_RELOBJ);
                    }
                }
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    SI_v_PerformO2OLaneAssociation */
void SI_v_PerformO2OLaneAssociation(void) {}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */