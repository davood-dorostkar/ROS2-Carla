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
#include "ops.h"
#include "ops_par.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*************************************************************************************************************************
Functionname:    FPSSenCritSetPOEThresh */
f32_t FPSSenCritSetPOEThresh(const sint8 ObjNr,
                             const bool_t HysteresisActive,
                             const FPS_POE_Thresholds_t *Thresholds) {
    f32_t thresh;

    if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) == OBJECT_PROPERTY_MOVING) {
        if (HysteresisActive) {
            thresh = Thresholds->MovingDrop;
        } else {
            thresh = Thresholds->MovingPickUp;
        }
    } else if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) ==
               OBJECT_PROPERTY_STATIONARY) {
        if (HysteresisActive) {
            thresh = Thresholds->StationaryDrop;
        } else {
            thresh = Thresholds->StationaryPickUp;
        }
    } else if (!EM_INT_OBJ_IS_DELETED(ObjNr)) {
        if (HysteresisActive) {
            thresh = Thresholds->DefaultDrop;
        } else {
            thresh = Thresholds->DefaultPickUp;
        }
    } else {
        thresh = Thresholds->DefaultPickUp;
    }

    return (thresh);
}

/*************************************************************************************************************************
Functionname:    FPSSenCritSetMTFThresh */
ui32_t FPSSenCritSetMTFThresh(const sint8 ObjNr,
                              const bool_t HysteresisActive,
                              const FPS_MTF_Thresholds_t *Thresholds) {
    ui32_t thresh = FPS_PRESEL_OBJ_FREQ_VAL_DEFAULT_THRES;

    if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) == OBJECT_PROPERTY_MOVING) {
        if (HysteresisActive) {
            thresh = Thresholds->MovingDrop;
        } else {
            thresh = Thresholds->MovingPickUp;
        }
    } else if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) ==
               OBJECT_PROPERTY_STATIONARY) {
        if (HysteresisActive) {
            if (Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(ObjNr)) {
                thresh = Thresholds->StoppedDrop;
            } else {
                thresh = Thresholds->StationaryDrop;
            }
        } else {
            if (Envm_INT_OBJ_IS_MOVING_TO_STATIONARY(ObjNr)) {
                thresh = Thresholds->StoppedPickUp;
            } else {
                thresh = Thresholds->StationaryPickUp;
            }
        }
    } else if (!EM_INT_OBJ_IS_DELETED(ObjNr)) {
        if (HysteresisActive) {
            thresh = Thresholds->DefaultDrop;
        } else {
            thresh = Thresholds->DefaultPickUp;
        }
    } else {
        thresh = FPS_PRESEL_OBJ_FREQ_VAL_DEFAULT_THRES;
    }

    return (thresh);
}

/*************************************************************************************************************************
Functionname:    FPSSenCritSetRCSThresh */
f32_t FPSSenCritSetRCSThresh(const sint8 ObjNr, const bool_t HysteresisActive) {
    float32 RCSThresholdLow;
    float32 RCSThresholdNewRel = SI_RCSAdd_MOVING;
    f32_t thresh = 0.0f;
    i32_t RangeGateNo = SI_NOF_RG;
    RCSThresholdLow = Envm_INT_OBJ_RCS_TGT_TRESHOLD_UNCOMP(ObjNr) -
                      Envm_f_GetRCSThresholdReduction(SYS_SCAN_NEAR);

    if ((EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) != OBJECT_PROPERTY_STATIONARY) &&
        (!EM_INT_OBJ_IS_DELETED(ObjNr))) {
        RCSThresholdNewRel = RCSThresholdLow + SI_RCSAdd_MOVING;

        if (HysteresisActive) {
            thresh = RCSThresholdLow;
        } else {
            thresh = RCSThresholdNewRel;
        }
    } else if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) ==
               OBJECT_PROPERTY_STATIONARY) {
        RangeGateNo = (i32_t)ROUND_TO_INT(EM_INT_OBJ_LONG_DISPLACEMENT(ObjNr) *
                                          (1.f / SI_RG_LENGTH));
        RangeGateNo = TUE_CML_MinMax(0, (SI_NOF_RG - 1L), RangeGateNo);

        /* Set default threshold */
        thresh = RCSThresholdLow;

        if (!HysteresisActive) {
            /* Add range gate dependent offset to the detection threshold */
            thresh += RCSThreshTargetOffsetStat[RangeGateNo];
        }
    } else {
        thresh = 0.0f;
    }

    return (thresh);
}

/*************************************************************************************************************************
Functionname:    FPSSenCritSetMTDThresh */
i32_t FPSSenCritSetMTDThresh(const sint8 ObjNr,
                             const bool_t HysteresisActive,
                             const FPS_MTD_Thresholds_t *Thresholds) {
    i32_t thresh = -FPS_PRESEL_OBJ_DENSITY_DEFAULT_THRES;

    if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) == OBJECT_PROPERTY_MOVING) {
        if (HysteresisActive) {
            thresh = Thresholds->MovingDrop;
        } else {
            thresh = Thresholds->MovingPickUp;
        }
    } else if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) ==
               OBJECT_PROPERTY_STATIONARY) {
        if (HysteresisActive) {
            thresh = Thresholds->StationaryDrop;
        } else {
            thresh = Thresholds->StationaryPickUp;
        }
    } else if (!EM_INT_OBJ_IS_DELETED(ObjNr)) {
        if (HysteresisActive) {
            thresh = Thresholds->DefaultDrop;
        } else {
            thresh = Thresholds->DefaultPickUp;
        }
    } else {
        thresh = FPS_PRESEL_OBJ_DENSITY_DEFAULT_THRES;
    }

    return (thresh);
}

/*************************************************************************************************************************
  Functionname:    FPSSenCritSetLifetimeThresh */
ui16_t FPSSenCritSetLifetimeThresh(
    const sint8 ObjNr,
    const bool_t HysteresisActive,
    const FPS_Lifetime_Thresholds_t *Thresholds) {
    ui16_t thresh = 0u;

    if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) == OBJECT_PROPERTY_MOVING) {
        if (HysteresisActive) {
            thresh = Thresholds->MovingDrop;
        } else {
            thresh = Thresholds->MovingPickUp;
        }
    } else if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) ==
               OBJECT_PROPERTY_STATIONARY) {
        if (HysteresisActive) {
            thresh = Thresholds->StationaryDrop;
        } else {
            thresh = Thresholds->StationaryPickUp;
        }
    } else if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) == OBJECT_PROPERTY_ONCOMING) {
        if (HysteresisActive) {
            thresh = Thresholds->OncomingDrop;
        } else {
            thresh = Thresholds->OncomingPickUp;
        }
    } else if (!EM_INT_OBJ_IS_DELETED(ObjNr)) {
        if (HysteresisActive) {
            thresh = Thresholds->DefaultDrop;
        } else {
            thresh = Thresholds->DefaultPickUp;
        }
    } else {
        thresh = FPS_PRESEL_OBJ_LIFETIME_THRES;
    }

    return (thresh);
}

/* Stop EM default section for code and data (back to default)
   This has to be at the very end of every EM c code file.      */

#define EM_STOP_DEFAULT_SECTION
//#include "envm_section_macros.h"
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */