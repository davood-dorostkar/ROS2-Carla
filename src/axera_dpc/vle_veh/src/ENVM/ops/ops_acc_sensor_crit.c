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

/* typedef eDynamicProperty_t */

//#define OBJ_GET_OT(iObj) EnvmInternalObj->Objects[iObj].Private
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
  Functionname:    FPSSenCritCheckPOE */
bool_t FPSSenCritCheckPOE(const sint8 ObjNr, const f32_t threshold) {
    bool_t bRet;
    if (Envm_INT_OBJ_PROBABILITY_OF_EXIST(ObjNr) < threshold) {
        bRet = (FALSE);
    } else {
        bRet = (TRUE);
    }

    /* do not select moving objects if fUpdateQuality is low */
    // if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) == OBJECT_PROPERTY_MOVING)
    //{
    //  if (OBJ_FCT_GET_OOI_POS(ObjNr) > OBJ_NOT_OOI)
    //  {

    //    /* object is already OOI -> disregard fUpdateQuality*/
    //  }
    //  else
    //  {
    //    // liuyang   nedd more amend  //if (OBJ_GET_OT(ObjNr).fUpdateQuality <
    //    FPS_MOVING_OBJ_UPDATE_QUALITY_THRES)
    //    {
    //      bRet = FALSE;
    //    }
    //  }
    //}

    return bRet;
}

/*************************************************************************************************************************
  Functionname:    FPSSenCritCheckMTF */
bool_t FPSSenCritCheckMTF(const sint8 ObjNr, const ui32_t threshold) {
    bool_t bRet;

    /* get 'combined' target confirmation density */
    uint8 u_tgtDensity = Envm_u_GetTgtConfirmDensity(ObjNr);

    if (u_tgtDensity < threshold) {
        bRet = (FALSE);
    } else {
        bRet = (TRUE);
    }
    return bRet;
}

/*************************************************************************************************************************
  Functionname:    FPSSenCritCheckRCS */
bool_t FPSSenCritCheckRCS(const sint8 ObjNr,
                          const f32_t threshold,
                          const bool_t HysteresisActive) {
    bool_t bRet;

    /* special SI State SI_RED_QUAL (reduced quality check): don't check RCS
     * (EOL) */
    if ((StateFPS == FPS_RED_QUAL)
        /* Do not allow de-selection of objects that are already OOI based on
           RCS criteria */
        || (HysteresisActive)) {
        bRet = (TRUE);
    } else {
        /* Branch for moving/stationary */
        if (EM_INT_OBJ_DYNAMIC_PROPERTY(ObjNr) != OBJECT_PROPERTY_STATIONARY) {
            /* Non stationary criteria */
            if (((Envm_INT_OBJ_RCS(ObjNr) >= threshold)
                 /*&& (  (OBJ_CURRENT_TRT_RCS(ObjNr) >= threshold)
                     || (OBJ_CURRENT_TRT_RCS(ObjNr) < RCS_NO_TARGET)
                     || (  (EM_INT_OBJ_LIFETIME(ObjNr) > RCS_LIFETIME_MIN)
                        && (EM_INT_OBJ_LONG_DISPLACEMENT(ObjNr) >
                    RCS_RANGE_LIFETIME)
                        )
                     )*/
                 )) {
                bRet = (TRUE);
            } else {
                bRet = (FALSE);
            }
        } else {
            /* Stationary object branch : these need to satisfy the RCS criteria
             */
            if (Envm_INT_OBJ_RCS(ObjNr) < threshold) {
                bRet = (FALSE);
            } else {
                bRet = (TRUE);
            }
        }
    }
    return bRet;
}

/*************************************************************************************************************************
  Functionname:    FPSSenCritCheckMTD */
bool_t FPSSenCritCheckMTD(const sint8 ObjNr, const i32_t threshold) {
    bool_t bRet;

    /* get 'combined' target confirmation density */
    uint8 u_tgtDensity = Envm_u_GetTgtConfirmDensity(ObjNr);

    /* Use lookup table to get weighted detection value */
    sint8 iTargetDensity = FPSLookupMTF2MTD
        [u_tgtDensity]; /* Formerly Envm_INT_OT_GET_TGT_CONFIRM_DENSITY(ObjNr)
                           */
    // sint8 iTargetDensity = 1;
    if (iTargetDensity <
        threshold) /*FPSSetMeasuredTargetDensityThresh(ObjNr))*/
    {
        bRet = (FALSE);
    } else {
        bRet = (TRUE);
    }
    return bRet;
}

/*************************************************************************************************************************
  Functionname:    FPSSenCritCheckLifetime */
bool_t FPSSenCritCheckLifetime(const sint8 ObjNr, const ui16_t threshold) {
    bool_t bRet;

    if (EM_INT_OBJ_LIFETIME(ObjNr) < threshold) /*FPSSetLifetimeThresh(ObjNr))*/
    {
        bRet = (FALSE);
    } else {
        bRet = (TRUE);
    }
    return bRet;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */