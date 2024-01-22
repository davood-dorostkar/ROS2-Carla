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
#include "fip_ext.h"
/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/
static void SIDetermObjBehindRoadEst(void);

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/
/* maximal distance of object in meter */
#define SI_ROADESTPLAUS_MAXDIST (50.0f)
/* maximal inner distance from RoadEstimation im meter */
#define SI_ROADESTPLAUS_MAXLATOFFSET (0.5f)
/* Timer Constant if Plausibilation is false */
#define SI_ROADESTPLAUS_TIMER (33u)
/* min confidence of road estimation for consideration */
#define SI_ROADEST_CUTIN_MIN_CONF (60u)

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

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

/*************************************************************************************************************************
  Functionname:    SICalcBaseFeatures */
void SICalcBaseFeatures(void) {
    SIDetermObjBehindRoadEst();

    SICalcBasePotentials();
}

/*************************************************************************************************************************
  Functionname:    SICalcBasePotentials */
void SICalcBasePotentials(void) {
    sint8 nr;
    SIPredictedDistance_t p_dist;
    float32 LowPotLine;
    float32 HiPotLine;
    float32 prediction_time;
    float32 EgoSpeed = EGO_SPEED_X_OBJ_SYNC;

    SICutInObjectDataInit();

    /* Look for list of objects of interest */
    for (nr = 0; nr < SiAnzOOI; nr++) {
        ObjNumber_t const iCurObj = OBJ_GET_OOI_LIST_OBJ_IDX(nr);

        /* Check if object exists (is selected) */
        if (iCurObj != OBJ_INDEX_NO_OBJECT) {
            switch (nr) {
                case OBJ_NEXT_OOI:

                    /* set 100% and 0% line for cut out potential */
                    HiPotLine = Y_DIST_MAX_CUT_OUT_POT;
                    LowPotLine = Y_DIST_MIN_CUT_OUT_POT;

                    prediction_time = SILimitPredictionTimeDist(iCurObj);

                    SICalcPredDisplToCutOut(prediction_time,
                                            OBJ_CLASSIFICATION(iCurObj),
                                            iCurObj, &p_dist);

                    p_dist.pdist =
                        (p_dist.pdist - SQRT_(p_dist.pdist_var)) +
                        SI_TOLERABLE_STDDEV_CI_CO_POT; /*!< Remark:
                                                          p_dist.pdist_var >= 0
                                                          */

                    /* Calculate Cut-out potential */
                    SiCalculateCutOutPotFiltered(p_dist.pdist, HiPotLine,
                                                 LowPotLine, iCurObj);

                    /* Save predicted lateral displacement */
                    OBJ_GET_SI(iCurObj).fPredictedLatDispl = p_dist.pdist;

                    OBJ_GET_CUT_IN_POTENTIAL(iCurObj) = 0u;
                    break;

                case OBJ_HIDDEN_NEXT_OOI:
                    /* set 100% and 0% line for cut out potential */
                    HiPotLine = Y_DIST_MAX_CUT_OUT_POT;
                    LowPotLine = Y_DIST_MIN_CUT_OUT_POT;

                    p_dist.pdist = 0.0f;

                    /* Calculate Cut-out potential */
                    SiCalculateCutOutPotFiltered(p_dist.pdist, HiPotLine,
                                                 LowPotLine, iCurObj);

                    /* Save predicted lateral displacement */
                    OBJ_GET_SI(iCurObj).fPredictedLatDispl = p_dist.pdist;

                    OBJ_GET_CUT_IN_POTENTIAL(iCurObj) = 0u;
                    break;

                case OBJ_NEXT_LONG_LEFT_OOI:
                case OBJ_NEXT_LONG_RIGHT_OOI:
                case OBJ_NEXT_LAT_LEFT_OOI:
                case OBJ_NEXT_LAT_RIGHT_OOI:

                    /* calculate cut -in potential only for moving and stopped
                     * objects */
                    if (((OBJ_DYNAMIC_PROPERTY(iCurObj) ==
                          CR_OBJECT_PROPERTY_MOVING) ||
                         (OBJ_IS_MOVING_TO_STATIONARY(iCurObj))) &&
                        ((EgoSpeed + OBJ_LONG_VREL(iCurObj)) >=
                         CUTINPOTENTIAL_MIN_OBJ_ABSVEL)) {
                        /* Use new cut-in potential calculation algo */
                        OBJ_GET_CUT_IN_POTENTIAL(iCurObj) =
                            SICalcCutInNeighborObj(iCurObj);

                    } else {
                        /* set cut - in potential for stationary objects to 0 */
                        OBJ_GET_CUT_IN_POTENTIAL(iCurObj) = 0u;
                    }
                    OBJ_GET_CUT_OUT_POTENTIAL(iCurObj) = 0u;
                    break;

                default:
                    break;
            }
        }
    }

    /* Reset cut-in & cut-out potential to zero for non OOI objects  */
    for (nr = 0; nr < Envm_N_OBJECTS; nr++) {
        if (!VLCObjectList[nr].SI.Bool.SelectedAsOOI) {
            OBJ_GET_CUT_IN_POTENTIAL(nr) = 0u;
            OBJ_GET_CUT_OUT_POTENTIAL(nr) = 0u;
        }
    }

    SICutInObjectDataFreeze();
}

/*************************************************************************************************************************
  Functionname:    SIDetermObjBehindRoadEst */
static void SIDetermObjBehindRoadEst(void) {
    ObjNumber_t iObj;
    boolean bGRD;
    boolean b_RoadEstValid = FALSE;

    if (b_RoadEstValid && (TUNNEL_PROBABILITY < SI_TUNNEL_PROB_THRES)) {
        for (iObj = 0; iObj < Envm_N_OBJECTS; iObj++) {
            if ((!OBJ_IS_DELETED(iObj)) &&
                (OBJ_DYNAMIC_PROPERTY(iObj) == CR_OBJECT_PROPERTY_MOVING) &&
                (OBJ_LONG_DISPLACEMENT(iObj) < SI_ROADESTPLAUS_MAXDIST)) {
                /* Local vars */
                bGRD = TRUE;

                if (bGRD == FALSE) {
                    /* set counter for suppressing cutin potential */
                    OBJ_GET_SI(iObj)
                        .ObjLaneAccStatus.In2OutlaneTransition =
                        SI_ROADESTPLAUS_TIMER;
                }
            }
        }
    }
    return;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
