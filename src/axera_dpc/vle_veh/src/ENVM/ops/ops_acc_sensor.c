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
#include "ops.h"
#include "ops_par.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "assert.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#ifndef FUN_PRESEL_ACC_HIGHEST_CLUST_VAR_OBJ_QUAL
#define FUN_PRESEL_ACC_HIGHEST_CLUST_VAR_OBJ_QUAL \
    (74u) /* To be removed here, if available in algo_constants.h */
#endif

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

static const FPS_POE_Thresholds_t FPSAccPoEThresholds = {
    FPS_ACC_POE_MOVING_PICKUP_THRESHOLD, /*f32_t MovingPickUp;*/
    FPS_ACC_POE_MOVING_DROP_THRESHOLD,   /*f32_t MovingDrop;*/
    FPS_ACC_POE_STAT_PICKUP_THRESHOLD,   /*f32_t StationaryPickUp;*/
    FPS_ACC_POE_STAT_DROP_THRESHOLD,     /*f32_t StationaryDrop;*/
    FPS_ACC_POE_MOVING_PICKUP_THRESHOLD, /*f32_t DefaultPickUp;*/
    FPS_ACC_POE_MOVING_DROP_THRESHOLD,   /*f32_t DefaultDrop;*/
};

static const FPS_MTF_Thresholds_t FPSAccMTFThresholds = {
    FPS_ACC_MTF_MOVING_PICKUP_THRESHOLD, /*ui32_t MovingPickUp;*/
    FPS_ACC_MTF_MOVING_DROP_THRESHOLD,   /*ui32_t MovingDrop;*/
    FPS_ACC_MTF_STAT_PICKUP_THRESHOLD,   /*ui32_t StoppedPickUp;*/
    FPS_ACC_MTF_MOVING_DROP_THRESHOLD,   /*ui32_t StoppedDrop;*/
    FPS_ACC_MTF_STAT_PICKUP_THRESHOLD,   /*ui32_t StationaryPickUp;*/
    FPS_ACC_MTF_STAT_DROP_THRESHOLD,     /*ui32_t StationaryDrop;*/
    FPS_ACC_MTF_MOVING_PICKUP_THRESHOLD, /*ui32_t DefaultPickUp;*/
    FPS_ACC_MTF_MOVING_DROP_THRESHOLD,   /*ui32_t DefaultDrop;*/
};

static const FPS_MTD_Thresholds_t FPSAccMTDThresholds = {
    FPS_ACC_MTD_MOVING_PICKUP_THRESHOLD, /*i32_t MovingPickUp;*/
    FPS_ACC_MTD_MOVING_DROP_THRESHOLD,   /*i32_t MovingDrop;*/
    FPS_ACC_MTD_STAT_PICKUP_THRESHOLD,   /*i32_t StationaryPickUp;*/
    FPS_ACC_MTD_STAT_DROP_THRESHOLD,     /*i32_t StationaryDrop;*/
    FPS_ACC_MTD_MOVING_PICKUP_THRESHOLD, /*i32_t DefaultPickUp;*/
    FPS_ACC_MTD_MOVING_DROP_THRESHOLD,   /*i32_t DefaultDrop;*/

};

static const FPS_Lifetime_Thresholds_t FPSAccLTThresholds = {
    FPS_LTIME_MOVING_PICKUP_THRESHOLD, /*ui16_t MovingPickUp;*/
    FPS_LTIME_MOVING_DROP_THRESHOLD,   /*ui16_t MovingDrop;*/
    FPS_LTIME_STAT_PICKUP_THRESHOLD,   /*ui16_t StationaryPickUp;*/
    FPS_LTIME_STAT_DROP_THRESHOLD,     /*ui16_t StationaryDrop;*/
    FPS_LTIME_MOVING_PICKUP_THRESHOLD, /*ui16_t OncomingPickUp;*/
    FPS_LTIME_MOVING_DROP_THRESHOLD,   /*ui16_t OncomingDrop;*/
    FPS_LTIME_MOVING_PICKUP_THRESHOLD, /*ui16_t DefaultPickUp;*/
    FPS_LTIME_MOVING_DROP_THRESHOLD,   /*ui16_t DefaultDrop;*/
};

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    FPSACCSenPreSelPreProc */
void FPSACCSenPreSelPreProc(void) {}

/*************************************************************************************************************************
  Functionname:    FPSACCSensorPresel */
uint8 FPSACCSensorPresel(const sint8 s_ObjNr) {
    uint8 u_RetVal = 0;

    boolean b_POEValue = FALSE, b_RCSValue = FALSE, b_MTFValue = FALSE,
            b_MTDValue = FALSE, b_LTValue = FALSE,

            b_SUMValue = FALSE;

    boolean b_MirrorGhostValue = TRUE;
    boolean b_NoVrelAmbigObj = TRUE;

    /* get pointer to object list interface */
    // ObjectList_t *p_EnvmPublicObjectList = GET_Envm_INT_OBJ_DATA_PTR;

    boolean b_HysteresisActive =
        (OBJ_FCT_GET_OOI_POS(s_ObjNr) > OBJ_NOT_OOI) ? TRUE : FALSE;

    float32 f_POEThreshold =
        FPSACCSetPOEThresh(s_ObjNr, b_HysteresisActive, &FPSAccPoEThresholds);
    uint32 u_MTFThreshold =
        FPSACCSetMTFThresh(s_ObjNr, b_HysteresisActive, &FPSAccMTFThresholds);
    float32 f_RCSThreshold =
        FPSSenCritSetRCSThresh(s_ObjNr, b_HysteresisActive);
    sint32 s_MTDThreshold = FPSSenCritSetMTDThresh(s_ObjNr, b_HysteresisActive,
                                                   &FPSAccMTDThresholds);
    uint16 u_LTThreshold = FPSSenCritSetLifetimeThresh(
        s_ObjNr, b_HysteresisActive, &FPSAccLTThresholds);

    /* Reset bits that are about to be calculated in the following logic */
    uint8 ucPreselBits =
        (ui8_t)(Envm_INT_GET_Envm_OBJ(s_ObjNr).ACCPresel.ucAccSelBits &
                (ui8_t)(~((ui8_t)FPS_ACC_BIT_POE | (ui8_t)FPS_ACC_BIT_RCS |
                          (ui8_t)FPS_ACC_BIT_TGT | (ui8_t)FPS_ACC_BIT_LATV)));

    if (EM_INT_OBJ_DYNAMIC_PROPERTY(s_ObjNr) != OBJECT_PROPERTY_STATIONARY) {
        /* Non-stationary object pre-selection */
        b_POEValue = FPSSenCritCheckPOE(s_ObjNr, f_POEThreshold);
        b_MTFValue = FPSSenCritCheckMTF(s_ObjNr, u_MTFThreshold);
        b_RCSValue =
            FPSSenCritCheckRCS(s_ObjNr, f_RCSThreshold, b_HysteresisActive);
        b_MTDValue = FPSSenCritCheckMTD(s_ObjNr, s_MTDThreshold);
        b_LTValue = FPSSenCritCheckLifetime(s_ObjNr, u_LTThreshold);

        b_SUMValue = (b_POEValue && b_MTFValue && b_RCSValue && b_MTDValue &&
                      b_LTValue && b_NoVrelAmbigObj && b_MirrorGhostValue)
                         ? TRUE
                         : FALSE;
    } else {
        /* Stationary object sensor pre-selection */

        b_MTDValue = FPSSenCritCheckMTD(s_ObjNr, s_MTDThreshold);
        b_MTFValue = FPSSenCritCheckMTF(s_ObjNr, u_MTFThreshold);
        b_RCSValue =
            FPSSenCritCheckRCS(s_ObjNr, f_RCSThreshold, b_HysteresisActive);
        b_POEValue = FPSSenCritCheckPOE(s_ObjNr, f_POEThreshold);
        b_LTValue = FPSSenCritCheckLifetime(s_ObjNr, u_LTThreshold);

        b_SUMValue =
            (b_POEValue && b_RCSValue && b_MTDValue && b_LTValue && b_MTFValue)
                ? TRUE
                : FALSE;
    }

    if (b_POEValue) {
        ucPreselBits |= FPS_ACC_BIT_POE;
    }
    if (b_RCSValue) {
        ucPreselBits |= FPS_ACC_BIT_RCS;
    }
    if (b_MTFValue && b_MTDValue) {
        ucPreselBits |= FPS_ACC_BIT_TGT;
    }
    if (b_LTValue) {
        ucPreselBits |= FPS_ACC_BIT_LATV;
    }
    Envm_INT_GET_Envm_OBJ(s_ObjNr).ACCPresel.ucAccSelBits = ucPreselBits;

    /* Save Pre-selection decisions for debugging and display */
    FctPreselStates[s_ObjNr].Bool.SensorPreselPOE = b_POEValue;
    FctPreselStates[s_ObjNr].Bool.SensorPreselRCS = b_RCSValue;
    FctPreselStates[s_ObjNr].Bool.SensorPreselMTF = b_MTFValue;
    FctPreselStates[s_ObjNr].Bool.SensorPreselMTD = b_MTDValue;
    FctPreselStates[s_ObjNr].Bool.SensorPreselLTV = b_LTValue;
    FctPreselStates[s_ObjNr].Bool.SensorPreselNoVrelAmbigObj = b_NoVrelAmbigObj;
    FctPreselStates[s_ObjNr].Bool.SensorPreselNoMirrorGhost =
        b_MirrorGhostValue;

    FctPreselStates[s_ObjNr].Bool.SensorPreselSum = b_SUMValue;

    if (b_SUMValue) {
        u_RetVal = FUN_PRESEL_ACC_MIN_INLANE_OBJ_QUAL;

    } else {
        u_RetVal = 0u;
    }

    return u_RetVal;
}

/*************************************************************************************************************************
  Functionname:    FPSACCSetPOEThresh */
f32_t FPSACCSetPOEThresh(const sint8 ObjNr,
                         const bool_t HysteresisActive,
                         const FPS_POE_Thresholds_t *Thresholds) {
    f32_t thresh;
    if ((HysteresisActive)  //(OBJ_FCT_GET_OOI_POS(ObjNr) > OBJ_NOT_OOI)
        && (OBJ_FCT_GET_RELEVANT(ObjNr)) &&
        (FctPreselStates[ObjNr].fRelevantTime > FPS_ACC_RELEVANT_MIRROR_TIME)) {
        /* do not supress relevant objects, because of mirror recognition, after
         * certain relevant time */
        thresh = FPS_ACC_POE_MOVING_MIRROR_DROP_THRESHOLD;
    } else {
        thresh = FPSSenCritSetPOEThresh(ObjNr, HysteresisActive, Thresholds);
    }
    return (thresh);
}

/*************************************************************************************************************************
  Functionname:    FPSACCSetMTFThresh */
ui32_t FPSACCSetMTFThresh(const sint8 ObjNr,
                          const bool_t HysteresisActive,
                          const FPS_MTF_Thresholds_t *Thresholds) {
    ui32_t thresh;

    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) == OBJECT_PROPERTY_STATIONARY) &&
        (HysteresisActive) && (!Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(ObjNr)) &&
        (EM_INT_OBJ_LONG_DISPLACEMENT(ObjNr) <
         FPS_ACC_MTF_STAT_DROP_NEAR_DIST)) {
        thresh = FPS_ACC_MTF_STAT_DROP_NEAR_THRESHOLD;
    } else {
        thresh = FPSSenCritSetMTFThresh(ObjNr, HysteresisActive, Thresholds);
    }
    return (thresh);
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */