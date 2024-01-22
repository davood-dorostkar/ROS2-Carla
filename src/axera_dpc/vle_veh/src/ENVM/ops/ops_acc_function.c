/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"
#include "ops.h"
#include "ops_par.h"
#include "envm_common_utils.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
/*! Size of EM private object list */
#define Envm_NR_PRIVOBJECTS (40)

#define C_KMH_MS (3.6F)
/*! Start Speed of the first Range mode */
#define FPS_STATOBST_EFFECTIVE_RANGE1_SPEED (30.0f / C_KMH_MS)
/*! Start Speed of the second Range mode */
#define FPS_STATOBST_EFFECTIVE_RANGE2_SPEED (50.0f / C_KMH_MS)
/*! Start Speed of the third Range mode */
#define FPS_STATOBST_EFFECTIVE_RANGE3_SPEED (105.0f / C_KMH_MS)

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

#define FPS_ACC_MIN_POBS_FOR_OBST_QUALITY        \
    (50u) /*< Minimal pobs for an object to have \
          FUN_PRESEL_ACC_STAT_OBSTACLE ACC quality */

/*! slope and offset of the straight line determing the first Range Mode */
#define FPS_STATOBST_EFF_RANGE1_SLOPE                                          \
    ((FPS_STATOBST_EFFECTIVE_RANGE_MIDLE - FPS_STATOBST_EFFECTIVE_RANGE_MIN) / \
     (FPS_STATOBST_EFFECTIVE_RANGE2_SPEED -                                    \
      FPS_STATOBST_EFFECTIVE_RANGE1_SPEED))
#define FPS_STATOBST_EFF_RANGE1_OFFSET  \
    (FPS_STATOBST_EFFECTIVE_RANGE_MIN - \
     (FPS_STATOBST_EFF_RANGE1_SLOPE * FPS_STATOBST_EFFECTIVE_RANGE1_SPEED))
/*! slope and offset of the straight line determing the second Range Mode */
#define FPS_STATOBST_EFF_RANGE2_SLOPE                                          \
    ((FPS_STATOBST_EFFECTIVE_RANGE_MAX - FPS_STATOBST_EFFECTIVE_RANGE_MIDLE) / \
     (FPS_STATOBST_EFFECTIVE_RANGE3_SPEED -                                    \
      FPS_STATOBST_EFFECTIVE_RANGE2_SPEED))
#define FPS_STATOBST_EFF_RANGE2_OFFSET    \
    (FPS_STATOBST_EFFECTIVE_RANGE_MIDLE - \
     (FPS_STATOBST_EFF_RANGE2_SLOPE * FPS_STATOBST_EFFECTIVE_RANGE2_SPEED))
/*! Parameters for special handling of stationary objects in tunnels */

static const BML_t_LinFunctionArgs FPSMovOnBridge_VLatOrientRamp =

    {(FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_ORTHO),
     (FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_PARALL),
     ((FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_PARALL -
       FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_ORTHO) /
      (FPS_MOV_ON_BRIDGE_ANGLE_DELTA_PARALL -
       FPS_MOV_ON_BRIDGE_ANGLE_DELTA_ORTHO)),
     (FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_ORTHO) -
         (((FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_PARALL -
            FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_ORTHO) /
           (FPS_MOV_ON_BRIDGE_ANGLE_DELTA_PARALL -
            FPS_MOV_ON_BRIDGE_ANGLE_DELTA_ORTHO)) *
          FPS_MOV_ON_BRIDGE_ANGLE_DELTA_ORTHO)};

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! @vname: FPSPreselData @vaddr: Envm_MEAS_ID_FUN_PRESEL_ACC_DATA @cycleid:
 * Envm_ENV */
FctPreselAccState_t FctPreselStates[Envm_NR_PRIVOBJECTS];

/*! Maximum stationary object (not including stopped) selection distance @unit:m
 */
static f32_t fFPSStatSelMaxDist;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/
static void FPSACCPresel(void);
// static void    FPS_v_SetStatObstacleACCQual(const sint8 s_ObjNr, const uint8
// t_AccSenObjQual, uint8 *p_AccObjQual);
static void FPSACCUpdatePreselTimers(const sint8 ObjNr);
static bool_t FPSCheckLongVehicleType(const sint8 ObjNr);
static ui32_t FPSSetPOBSThreshold(const sint8 ObjNr);
static bool_t FPSCheckPOBS(const sint8 s_ObjNr);
static bool_t FPSCheckPedestrians(const sint8 ObjNr);
static bool_t FPSCheckGridConfirmation(const sint8 iObj);
static boolean FPSCheckPOBSStationaryObj(const sint8 s_ObjNr);
static void v_PreselectMovingObj(const sint8 s_ObjNr,
                                 const boolean b_POBSValue,
                                 boolean *b_LVValue,
                                 const boolean b_PEDValue,
                                 boolean *b_SUMValue,
                                 uint8 *u_PreselBits,
                                 const boolean b_CAMValue);

static void v_PreselectStationaryObj(const sint8 s_ObjNr,
                                     const boolean b_POBSValue,
                                     const boolean b_PEDValue,
                                     boolean *b_SUMValue,
                                     uint8 *u_PreselBits,
                                     boolean *b_CAMValue);

static void FPS_v_SetStatObstacleACCQual(const sint8 s_ObjNr,
                                         const uint8 t_AccSenObjQual,
                                         uint8 *p_AccObjQual) {
    /* For stationary only which have no ACC quality. Objects needs to have ACC
       sensor quality or
       an obstacle quality above 50 */
    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(s_ObjNr) == OBJECT_PROPERTY_STATIONARY) &&
        (!Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(s_ObjNr)) &&
        (*p_AccObjQual == FUN_PRESEL_ACC_DROP_QUAL) &&
        ((EM_INT_OBJ_OBSTACLE_PROBABILITY(s_ObjNr) >
          FPS_ACC_MIN_POBS_FOR_OBST_QUALITY) ||
         (t_AccSenObjQual >= FUN_PRESEL_ACC_MIN_INLANE_OBJ_QUAL))) {
        *p_AccObjQual = FUN_PRESEL_ACC_STAT_OBSTACLE;
    }
}

/*************************************************************************************************************************
  Functionname:    FPSInitACC */
void FPSInitACC(void) {
    sint8 ObjNr;

    /* Go through all objects, checking if individual criteria pass */
    for (ObjNr = (Envm_NR_PRIVOBJECTS - 1); ObjNr >= 0L; ObjNr--) {
        FPSInitACCObject(ObjNr);
    }
}

/*************************************************************************************************************************
  Functionname:    FPSACCPresel */
static void FPSACCPresel(void) {
    sint8 ObjNr;
    /* Go through all objects, checking if individual criteria pass */
    for (ObjNr = (Envm_NR_PRIVOBJECTS - 1); ObjNr >= 0L; ObjNr--) {
        uint8 AccObjQual = 0u;

        if (!EM_INT_OBJ_IS_DELETED(ObjNr)) {
            uint8 AccSenObjQual, AccFunObjQual;

            /* Update relevancy timer for given object */
            FPSACCUpdatePreselTimers(ObjNr);

            /* First get sensor object quality */
            AccSenObjQual = FPSACCSensorPresel(ObjNr);

            /* Next get function object quality */
            AccFunObjQual = FPSACCFunctionPresel(ObjNr);

            /* The final ACC object quality is the smaller of the two */
            AccObjQual = MIN(AccSenObjQual, AccFunObjQual);

            /* Set stationary obstacle ACC Quality */
            FPS_v_SetStatObstacleACCQual(ObjNr, AccSenObjQual, &AccObjQual);
        }

        /* Store final ACC object quality */
        Envm_p_GetPrivObject(ObjNr)->ACCPresel.ucAccObjQuality = AccObjQual;
    }
}

/*************************************************************************************************************************
  Functionname:    FPSACCProcess */
void FPSACCProcess(void) {
    /* Call function preselection pre-processing */
    FPSACCFuncPreSelPreProc();
    /* Call sensor preselection pre-processing */

    FPSACCSenPreSelPreProc();
    /* Calculate ACC quality */
    FPSACCPresel();
}

/*************************************************************************************************************************
  Functionname:    FPSInitACCObject */
void FPSInitACCObject(sint8 ObjNr) {
    FctPreselAccState_t *const pState = &FctPreselStates[ObjNr];

    /* Reset state fields */
    pState->Bool.FctPreselPOBS = 0u;
    pState->Bool.FctPreselLV = 0u;
    pState->Bool.FctPreselPED = 0u;
    pState->Bool.FctPreselCam = 0u;

    /* Bits filled in by sensor pre-selection */
    pState->Bool.SensorPreselPOE = 0u;
    pState->Bool.SensorPreselRCS = 0u;
    pState->Bool.SensorPreselMTF = 0u;
    pState->Bool.SensorPreselLATV = 0u;
    pState->Bool.SensorPreselMTD = 0u;
    pState->Bool.SensorPreselLTV = 0u;
    pState->Bool.SensorPreselNoVrelAmbigObj = 0u;
    pState->Bool.SensorPreselNoMirrorGhost = 0u;
    /* Summary bits */
    pState->Bool.FctPreselSum = 0u;
    pState->Bool.SensorPreselSum = 0u;
    /* Other values */
    pState->uiCyclesBelowRCSThresh = 0u;
    pState->fRelevantTime = 0;

    Envm_p_GetPrivObject(ObjNr)->ACCPresel.ucAccObjQuality = 0u;
}

/*************************************************************************************************************************
  Functionname:    FPSACCUpdatePreselTimers */
static void FPSACCUpdatePreselTimers(const sint8 ObjNr) {
    FctPreselAccState_t *const pState = &FctPreselStates[ObjNr];

    if (OBJ_FCT_GET_RELEVANT(ObjNr) != FALSE) {
        /* object is relevant, update it's relevancy time */
        pState->fRelevantTime += EM_f_GetCycleTime();
    } else {
        /* object is not relevant, note: timer not reset in this case */
    }
}
/*************************************************************************************************************************
  Functionname:    v_PreselectMovingObj */

static void v_PreselectMovingObj(const sint8 s_ObjNr,
                                 const boolean b_POBSValue,
                                 boolean *b_LVValue,
                                 const boolean b_PEDValue,
                                 boolean *b_SUMValue,
                                 uint8 *u_PreselBits,
                                 const boolean b_CAMValue)

{
    *b_LVValue = FPSCheckLongVehicleType(s_ObjNr);

    /* Set bits in preselection bits if tests satisifed */
    if (*b_LVValue) {
        *u_PreselBits |= FPS_ACC_BIT_LV;
    }

    /* For non stationary object selection criteria is obstacle probability,
    not part of long vehicle, not near range artifact and pedestrian */
    *b_SUMValue =
        ((b_POBSValue) && (*b_LVValue) && (b_PEDValue)) ? TRUE : FALSE;

    /* If camera confirms object, then it passes pre-selection */
    if (b_CAMValue) {
        *b_SUMValue = TRUE;
    }
}
/*************************************************************************************************************************
  Functionname:    v_PreselectStationaryObj */
static void v_PreselectStationaryObj(const sint8 s_ObjNr,
                                     const boolean b_POBSValue,
                                     const boolean b_PEDValue,
                                     boolean *b_SUMValue,
                                     uint8 *u_PreselBits,
                                     boolean *b_CAMValue)

{
    /* Set the bits for tests not performed for stationaries */
    *u_PreselBits |= (FPS_ACC_BIT_LV);

    /* For stationary objects selection criteria is obstacle probability
    and not a supressed pedestrian */
    *b_SUMValue = ((b_POBSValue) && (b_PEDValue)) ? TRUE : FALSE;

    if (*b_CAMValue) {
        *b_SUMValue = TRUE;
    }
}

/*************************************************************************************************************************
  Functionname:    FPSACCFunctionPresel */
uint8 FPSACCFunctionPresel(const sint8 ObjNr) {
    uint8 ReturnValue = 0u;
    boolean b_POBSValue, b_LVValue = FALSE, b_PEDValue, b_SUMValue = FALSE,
                         b_TypeValue = FALSE;

    //#if ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) ||
    //((CFG_Envm_CAMOBJ_FUSION_LEVEL == CFG_Envm_FUSION_LEVEL_1) &&
    //(FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 == SWITCH_ON)))
    boolean b_CAMValue;
    //#endif

    /* Reset the bits that are going to be set in this function (and also set
     * removed NR-bit) */
    //  uint8 u_PreselBits =
    //  (ui8_t)((Envm_INT_GET_Envm_OBJ(ObjNr).ACCPresel.ucAccSelBits &
    //  ((ui8_t)~((ui8_t)FPS_ACC_BIT_LV | (ui8_t)FPS_ACC_BIT_PED |
    //  (ui8_t)FPS_ACC_BIT_POBS))) | (ui8_t)FPS_ACC_BIT_NR);

    uint8 u_PreselBits = 0;
    //#if ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) ||
    //((CFG_Envm_CAMOBJ_FUSION_LEVEL == CFG_Envm_FUSION_LEVEL_1) &&
    //(FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 == SWITCH_ON)))
    /* Check camera confirmation of object */
    b_CAMValue = Envm_INT_GET_Envm_OBJ(ObjNr).SensorSpecific.bCamConfirmed;
    //#endif

    if (Envm_INT_GET_Envm_OBJ(ObjNr).Attributes.eClassification ==
        Envm_GEN_OBJECT_CLASS_PEDESTRIAN) {
        b_TypeValue = FALSE;
    } else {
        b_TypeValue = TRUE;
    }

    b_PEDValue = FPSCheckPedestrians(ObjNr);
    b_POBSValue = FPSCheckPOBS(ObjNr);

    /* Check if object is likely to be moving on bridge and should be suppressed
     */
    if (b_POBSValue == TRUE) {
        b_POBSValue = FPSSuppressMovOnBridge(ObjNr);
    }

    /* Fundamentally stationary and non-stationary objects have different
    selection
    criteria. Check object's dynamic property to see which needs to be used */
    if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) != OBJECT_PROPERTY_STATIONARY) {
        /******************************************************************/
        /* Preselect moving objects                                       */
        /******************************************************************/
        v_PreselectMovingObj(ObjNr, b_POBSValue, &b_LVValue, b_PEDValue,
                             &b_SUMValue, &u_PreselBits, b_CAMValue);
    } else {
        /******************************************************************/
        /* Preselect stationary objects                                   */
        /******************************************************************/
        v_PreselectStationaryObj(ObjNr, b_POBSValue, b_PEDValue, &b_SUMValue,
                                 &u_PreselBits, &b_CAMValue);
    }

    /* Fill in bits of preselection information */
    if (b_PEDValue) {
        u_PreselBits |= FPS_ACC_BIT_PED;
    }
    if (b_POBSValue) {
        u_PreselBits |= FPS_ACC_BIT_POBS;
    }
    /* Writeback the bits for debugging */
    Envm_INT_GET_Envm_OBJ(ObjNr).ACCPresel.ucAccSelBits = u_PreselBits;
    /* Save Preselection decisions for debugging, display & query */
    FctPreselStates[ObjNr].Bool.FctPreselPOBS = b_POBSValue;
    FctPreselStates[ObjNr].Bool.FctPreselLV = b_LVValue;
    FctPreselStates[ObjNr].Bool.FctPreselPED = b_PEDValue;
    FctPreselStates[ObjNr].Bool.FctPreselCam = b_CAMValue;

    FctPreselStates[ObjNr].Bool.FctPreselSum = b_SUMValue;

    if (b_SUMValue && b_TypeValue) {
        /* In ARS300 code si_laneassociation did an extra check for stationary
        objects before making any of them inlane. Mimic that behaviour here,
        doing the extra grid check */
        /* Add alternative check via video confirmation to allow stationary
           inlane objects */
        if (b_CAMValue) {
            /* Object confirmed by camera, may be selected as in-lane object */
            ReturnValue = FUN_PRESEL_ACC_MIN_INLANE_OBJ_QUAL;
        } else if (FPSCheckGridConfirmation(ObjNr)) {
            /* Object confirmed by grid, may be selected as in-lane object */
            ReturnValue = FUN_PRESEL_ACC_MIN_INLANE_OBJ_QUAL;
        } else {
            /* Object not confirmed. In old ARS300 code this leads to
            object never passing SICheckStateFlowOutlaneToInlane, respectively
            always failing SICheckStateFlowInlaneToOutlane always returning
            TRUE */
            ReturnValue = FUN_PRESEL_ACC_MIN_OBJ_QUAL;
        }
    } else {
        /* Summary boolean indicates object not selectable */
        ReturnValue = 0u;
    }

    return ReturnValue;
}

/*************************************************************************************************************************
  Functionname:    FPSACCFuncPreSelPreProc */
void FPSACCFuncPreSelPreProc(void) {
    // liuyang 2018-11-29 remove, need to do more confirm
    const float32 fSpeed = EGO_SPEED_X_OBJ_SYNC;

    if (fSpeed > FPS_STATOBST_EFFECTIVE_RANGE3_SPEED) {
        fFPSStatSelMaxDist = FPS_STATOBST_EFFECTIVE_RANGE_MAX;
    } else if (fSpeed > FPS_STATOBST_EFFECTIVE_RANGE2_SPEED) {
        fFPSStatSelMaxDist = (FPS_STATOBST_EFF_RANGE2_SLOPE * fSpeed) +
                             FPS_STATOBST_EFF_RANGE2_OFFSET;
    } else if (fSpeed > FPS_STATOBST_EFFECTIVE_RANGE1_SPEED) {
        fFPSStatSelMaxDist = (FPS_STATOBST_EFF_RANGE1_SLOPE * fSpeed) +
                             FPS_STATOBST_EFF_RANGE1_OFFSET;
    } else {
        fFPSStatSelMaxDist = FPS_STATOBST_EFFECTIVE_RANGE_MIN;
    }
}

/*************************************************************************************************************************
  Functionname:    FPSCheckLongVehicleType */
static bool_t FPSCheckLongVehicleType(const sint8 ObjNr) {
    bool_t bRet;

    bRet = (TRUE);

    return bRet;
}

/*************************************************************************************************************************
  Functionname:    FPSSetPOBSThreshold */
static ui32_t FPSSetPOBSThreshold(const sint8 ObjNr) {
    ui32_t value;

    if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) == OBJECT_PROPERTY_MOVING) {
        if (OBJ_FCT_GET_OOI_POS(ObjNr) > OBJ_NOT_OOI) {
            value = FPS_ACC_POBS_MOVING_DROP_THRESHOLD;
        } else {
            value = FPS_ACC_POBS_MOVING_PICKUP_THRESHOLD;
        }
    } else if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) ==
               OBJECT_PROPERTY_STATIONARY) {
        if (OBJ_FCT_GET_OOI_POS(ObjNr) > OBJ_NOT_OOI) {
            value = FPS_ACC_POBS_STAT_DROP_THRESHOLD;
        } else {
            value = FPS_ACC_POBS_STAT_PICKUP_THRESHOLD;
        }
    } else {
        if (OBJ_FCT_GET_OOI_POS(ObjNr) > OBJ_NOT_OOI) {
            value = FPS_ACC_POBS_MOVING_DROP_THRESHOLD;
        } else {
            value = FPS_ACC_POBS_MOVING_PICKUP_THRESHOLD;
        }
    }
    return (value);
}

/*************************************************************************************************************************
  Functionname:    FPSCheckPOBSStationaryObj */
static boolean FPSCheckPOBSStationaryObj(const sint8 s_ObjNr) {
    boolean b_CheckObstProb =
        FALSE; /* FALSE by default, the most common outcome */
    const boolean b_TunnelDetected = Envm_b_IsTunnelDetected();

    /* Get object displacements */
    const float32 f_ObjLongDisp = EM_INT_OBJ_LONG_DISPLACEMENT(s_ObjNr);
    const float32 f_ObjLatDisp = Envm_INT_OBJ_LAT_DISPLACEnvmENT(s_ObjNr);

    /* Get EGO Curvature, Curvature Gradient and Speed Values */
    const float32 f_AbsCurve =
        fABS(EGO_CURVE_OBJ_SYNC); /* EGO absolute course             */
    const float32 f_AbsCurveGradient =
        fABS(EGO_CURVE_GRAD_OBJ_SYNC); /* EGO absolute Curvature Gradient */
    const float32 f_AbsSpeed =
        fABS(EGO_SPEED_X_OBJ_SYNC); /* EGO absolute velocity           */

    const float32 f_TunnelDist = 0.f;

    /* Detect dynamic situations */
    const boolean b_IsSituationDynamic =
        ((((SQR(f_ObjLongDisp)) + (SQR(f_ObjLatDisp))) >
          SQR(SI_STAT_DYNA_MIN_RANGE)) &&
         ((f_AbsCurve > SI_STAT_MAX_CURVATURE) ||
          ((f_AbsCurveGradient > SI_STAT_MAX_CURV_GRADIENT) &&
           (!OBJ_FCT_GET_RELEVANT(s_ObjNr))) ||
          ((f_AbsCurveGradient > SI_STAT_MAX_CURV_GRADIENT_REL) &&
           (OBJ_FCT_GET_RELEVANT(s_ObjNr)))))
            ? TRUE
            : FALSE;

    /* prevent during a dynamic situation an stationary object from being
     * relevant */
    if (b_IsSituationDynamic == TRUE) {
    }

    /* prevent during a parking process a stationary object from being relevant
    This check basically forces all stationaries to fail the check when driving
    curves tighter than given radius over a certain speed. The reason for the
    speed check is to avoid losing a relevant object at standstill with the
    driver playing around with the steering wheel. */
    else if ((f_AbsCurve > SI_STAT_MAX_PARKING_CURVATURE) &&
             (f_AbsSpeed > SI_STAT_MIN_PARKING_SPEED)) {
    }
    /* A relevant stationary object should not loose its relevance because of
       the obstacle Probabilty */
    else if ((OBJ_FCT_GET_RELEVANT(s_ObjNr)) &&
             (FctPreselStates[s_ObjNr].fRelevantTime >
              FPS_ACC_STAT_SECURE_RELEVANT_TIME)) {
        b_CheckObstProb = TRUE;
    }
    /* Reject stationary objects while ego-vehicle is not in standstill or slow
       moving to become relevant*/
    else if ((f_ObjLongDisp >= FPS_PAR_ACC_NO_CAM_CONF_STAT_MAX_DIST) ||
             (f_AbsSpeed > FPS_PAR_ACC_NO_CAM_CONF_STAT_MAX_VEL)) {
    }

    /* Ignore stationary objects in tunnels, code formerly in
       od_obst_objassess.c moved to FPS, as in reality a functional aspect */
    else if ((b_TunnelDetected == TRUE) && (f_ObjLongDisp >= f_TunnelDist)) {
        /* Suppress object in tunnel */
    }

    /* Limit our effective range, code formerly in od_obst_objassess.c moved to
       FPS, as in reality a functional aspect */
    else if (f_ObjLongDisp >= fFPSStatSelMaxDist) {
        /* Suppress objects too far away */
    } else {
        /* Bayes' Network : A priori Knowledge for consideration of Guardrail */
        static const uint8 a_FPSObstGuardrailBayesTable[4] = {0u, 100u, 0u,
                                                              30u};
        const uint8 u_GuardRailMergedObstProb = AlgoMathBayes2(
            EM_INT_OBJ_OBSTACLE_PROBABILITY(s_ObjNr),
            0 /*OBJ_ROADSIDE_PROB(s_ObjNr)*/, a_FPSObstGuardrailBayesTable);

        if (u_GuardRailMergedObstProb >= (ui8_t)FPSSetPOBSThreshold(s_ObjNr)) {
            b_CheckObstProb = TRUE;
        }
    }
    return b_CheckObstProb;
}

/*************************************************************************************************************************
  Functionname:    FPSCheckPOBS */
static bool_t FPSCheckPOBS(const sint8 s_ObjNr) {
    boolean b_CheckObstProb = FALSE;
    ui32_t u_POBSThreshold;

    /* special FPS State FPS_RED_QUAL (reduced quality check): don't check
     * obstacle probability */
    if (StateFPS == FPS_RED_QUAL) {
        b_CheckObstProb = TRUE;
    } else if ((EM_INT_OBJ_DYNAMIC_PROPERTY(s_ObjNr) ==
                OBJECT_PROPERTY_STATIONARY) &&
               (!Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(s_ObjNr))) {
        b_CheckObstProb = FPSCheckPOBSStationaryObj(s_ObjNr);
    } else {
        u_POBSThreshold = FPSSetPOBSThreshold(s_ObjNr);
        if (EM_INT_OBJ_OBSTACLE_PROBABILITY(s_ObjNr) >= u_POBSThreshold) {
            b_CheckObstProb = TRUE;
        }
    }

    return b_CheckObstProb;
}

/*************************************************************************************************************************
  Functionname:    FPSCheckPedestrians */
static bool_t FPSCheckPedestrians(const sint8 ObjNr) {
    bool_t bRet;
    f32_t Veigen = EGO_SPEED_X_OBJ_SYNC;

    /*! Pedestrian decision delay counters (in cycles) */
    static ui8_t FPSPedestrianDecisionWaitCounter[Envm_NR_PRIVOBJECTS];

    /* If object is new, reset decision counter */
    if ((EM_INT_OBJ_MAINTENANCE_STATE(ObjNr) == MT_STATE_NEW) ||
        (EM_INT_OBJ_MAINTENANCE_STATE(ObjNr) == MT_STATE_MERGE_NEW)) {
        FPSPedestrianDecisionWaitCounter[ObjNr] = 0u;
    }

    /* Is object classified as pedestrian ? */
    if ((Envm_INT_OBJ_CLASSIFICATION(ObjNr) == OBJCLASS_PEDESTRIAN))
    // levi 2018-12-08 removed   //   ||(OBJ_IS_PEDESTRIAN(ObjNr) > 0u))
    {
        /* Yes, so we don't have to wait for a decision for this classification
         * any longer */
        /* --> reset the wait counter */
        FPSPedestrianDecisionWaitCounter[ObjNr] = 0u;
        /* Do not select pedestrians above velocities of ego car above
          SI_MAX_SPEED_PED_SELECTION and
          over distances of SI_MAX_DIST_PED_SELECTION. Both values are defined
          in si_par.h */
        if ((Veigen > SI_MAX_SPEED_PED_SELECTION) ||
            (EM_INT_OBJ_LONG_DISPLACEMENT(ObjNr) > SI_MAX_DIST_PED_SELECTION)) {
            /* velocity or distance is bigger than above mentioned */
            /* --> do not select this pedestrian object */
            bRet = (FALSE);
        } else {
            /* select pedestrian object*/
            bRet = (TRUE);
        }
    }
    /* objects of class "point" that have already a pedestrianProbability of
    more than
    SI_WAIT_FOR_PED_DECISION_PROB (defined in fps_par.h), are possibly on the
    way to become a
    pedestrian object. Hold these objects back for selection until pedestrian
    classification can be certain.
    Number of cycles to wait is defined by SI_MAX_WAIT_CYCLES_FOR_PED in
    si_par.h */
    else if (Envm_INT_OBJ_CLASSIFICATION(ObjNr) == OBJCLASS_POINT) {
        /* if object is already selected, it is too late now to supress it */
        if (OBJ_FCT_GET_OOI_POS(ObjNr) == OBJ_NOT_OOI) {
            if (FPSPedestrianDecisionWaitCounter[ObjNr] >
                SI_MAX_WAIT_CYCLES_FOR_PED) {
                /* we have waited for decision, that did not come, so select
                 * object */
                bRet = (TRUE);
            } else {
                /* possible pedestrian object supresed by same circumstances as
                 * "real" pedestrian object*/
                FPSPedestrianDecisionWaitCounter[ObjNr]++;
                if ((Veigen > SI_MAX_SPEED_PED_SELECTION) ||
                    (EM_INT_OBJ_LONG_DISPLACEMENT(ObjNr) >
                     SI_MAX_DIST_PED_SELECTION)) {
                    /* velocity or distance is bigger than above mentioned */
                    /* --> do not select this possible pedestrian object */
                    bRet = (FALSE);
                } else {
                    /* select possible pedestrian object */
                    bRet = (TRUE);
                }
            }
        } else {
            /* object has already been an OOI, so do not supress it */
            FPSPedestrianDecisionWaitCounter[ObjNr] = 0u;
            bRet = (TRUE);
        }
    } else {
        /* no pedestrian object --> no suppression --> select it */
        FPSPedestrianDecisionWaitCounter[ObjNr] = 0u;
        bRet = (TRUE);
    }
    return bRet;
}

/*************************************************************************************************************************
  Functionname:    FPSSuppressMovOnBridge */
bool_t FPSSuppressMovOnBridge(const sint8 ObjNr) {
    boolean b_MovOnBridgeCriteria =
        TRUE;                    /* object not moving on bridge per default */
    float32 f_OrientationAbs;    /* absolute value of object orientation */
    float32 f_OrientationMapped; /* absolute value of object orientation */
    float32 f_VLatAbs;      /* absolute value of lateral relative velocity */
    float32 f_VlatAbsThres; /* orientation dependant threshold of lateral
                               relative velocity */

    /* suppression of moving objects on bridge only in case following general
     * conditions fulfilled */
    if ((EGO_SPEED_X_OBJ_SYNC > FPS_MOV_ON_BRIDGE_VEGO_MIN) &&
        (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) != OBJECT_PROPERTY_STATIONARY) &&
        (EM_INT_OBJ_LIFETIME(ObjNr) < FPS_MOV_ON_BRIDGE_LT_MAX) &&
        (fABS(EGO_CURVE_OBJ_SYNC) < FPS_MOV_ON_BRIDGE_CURVE_MAX) &&
        (fABS(EGO_YAW_RATE_OBJ_SYNC) < FPS_MOV_ON_BRIDGE_YAWRATE_MAX)) {
        /* take absolute value of object orientation, this criteria is
         * independant of traffic orientation */
        f_OrientationAbs = fABS(RAD2DEG(Envm_INT_OBJ_ORIENTATION_VALID(ObjNr)));
        /* take absolute value of object lateral relative velocity */
        f_VLatAbs = fABS(Envm_INT_OBJ_LAT_VREL(ObjNr));

        /* Map object orientation to one single quadrant */
        if (f_OrientationAbs > 90.f) {
            /* Orientation above 90 */
            f_OrientationMapped = 180.f - f_OrientationAbs;
        } else {
            /* Orientation below 90 */
            f_OrientationMapped = f_OrientationAbs;
        }

        /* Check general orientation and absolute lateral vrel boundaries for
         * moving objects on bridge */
        if ((f_OrientationMapped > FPS_MOV_ON_BRIDGE_ANGLE_DELTA_PARALL) &&
            (f_VLatAbs > FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_ORTHO)) {
            /* Lateral vrel threshold is dependent on the object orientation. A
               more parallel orientation needs
               a higher lateral vrel so that object is moving on a bridge with
               high probability */
            f_VlatAbsThres = dGDBmathLinearFunction(
                &FPSMovOnBridge_VLatOrientRamp, f_OrientationMapped);

            /* Check if object lateral vrel is above threshold for objects
             * moving on a bridge */
            if (f_VLatAbs > f_VlatAbsThres) {
                b_MovOnBridgeCriteria = FALSE;
            }
        }

        /* If object has not been classified as moving on a bridge, do an
         * additional check */
        if (b_MovOnBridgeCriteria == TRUE) {
            Objects_t const *const pObjA = Envm_p_GetPrivObject(ObjNr);
            float32 const fVabsX =
                fABS(pObjA->Kinematic.fVrelX + EM_f_GetEgoObjSyncVelX());

            /* Only do this check for objects with have
               - short lifetime
               - DistX above threshold
               - a low speed over ground */
            if ((pObjA->General.fLifeTime < FPS_MOV_ON_BRIDGE_LIFETIME) &&
                (pObjA->Kinematic.fDistX > FPS_MOV_ON_BRIDGE_DISTX_MIN) &&
                (fVabsX < FPS_MOV_ON_BRIDGE_VABS_MAX)) {
                sint8 i;
                sint8 iObj;
                boolean bAbortLoop = FALSE;
                uint8 uMovObjCount = 0u;
                Objects_t const *pObjB;

                /* loop over all objects */
                for (i = 0; (i < EM_INT_OBJ_NUMBER_OF_OBJ_USED) &&
                            (bAbortLoop == FALSE);
                     ++i) {
                    /* get ID of object B from sorted object list */
                    iObj = Envm_INT_OBJ_INDEX_DISTX_SORTED[i];

                    if ((EM_INT_OBJ_IS_DELETED(iObj) == FALSE) &&
                        (ObjNr != iObj)) {
                        /* Get pointer to private object */
                        pObjB = Envm_p_GetPrivObject(iObj);

                        /* The second object is too close, continue with the
                         * next object */
                        if ((pObjB->Kinematic.fDistX -
                             pObjA->Kinematic.fDistX) <
                            -FPS_MOV_ON_BRIDGE_DISTX_GATE) {
                            /* Do nothing */
                        } else if ((pObjB->Kinematic.fDistX -
                                    pObjA->Kinematic.fDistX) >
                                   FPS_MOV_ON_BRIDGE_DISTX_GATE) {
                            /* Second object is too far away abort search */
                            bAbortLoop = TRUE;
                        } else {
                            /* Check whether the second object fulfills some
                             * conditions which indicate a probability that it
                             * moves on a bridge */
                            float32 fVrelYThresh =
                                FPS_MOV_ON_BRIDGE_VRELY_GATE_HIGH;

                            /* Check speed and position relative to the first
                             * object */
                            if ((fABS(pObjB->Kinematic.fVrelY) >
                                 fVrelYThresh) &&
                                (fABS(pObjB->Kinematic.fDistY -
                                      pObjA->Kinematic.fDistY) <
                                 FPS_MOV_ON_BRIDGE_DISTY_GATE) &&
                                (fABS(pObjB->Kinematic.fVrelX -
                                      pObjA->Kinematic.fVrelX) <
                                 FPS_MOV_ON_BRIDGE_VRELX_GATE)) {
                                /* An object is found, count up */
                                ++uMovObjCount;
                            }
                        }
                    }
                }
                /* Check whether a sufficient number of objects were found */
                if (uMovObjCount >= 2u) {
                    /* Set moving on bridge condition flag to false*/
                    b_MovOnBridgeCriteria = FALSE;
                }
            }
        }
    }

    return b_MovOnBridgeCriteria;
}

/*************************************************************************************************************************
  Functionname:    FPSCheckGridConfirmation */
static bool_t FPSCheckGridConfirmation(const sint8 iObj) {
    bool_t GridCheck;

    GridCheck = TRUE;
    return GridCheck;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */