/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \addtogroup tuePrvMotionType
 * \{
 * \file    tue_prv_egoMotion.c
 * \brief  This is a structure definition file for the object fusion ego motion
 * interface that defines the motion of the ego vehicle for a given period of
 * time.
 *
 * Note: At the moment, the structure definition is preliminary and will not
 * hold all necessary information, e.g. the period of time (dt) or the latency
 * are missing. Further, object fusion will require the ego motion history in
 * the future, thus a list structure definition should follow.
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
/*
 */
/* PRQA S 0292 -- */ /* MKS */
                     /*
                      *
                      * <br>=====================================================<br>
                      * <b>Copyright 2014 by Tuerme.</b>
                      * <br>
                      * All rights reserved. Property of Tuerme.<br>
                      * Restricted rights to use, duplicate or disclose of this code<br>
                      * are granted through contract.
                      * <br>=====================================================<br>
                      */
/*==================[inclusions]============================================*/

#include "tue_prv_motionType.h"
#include "tue_prv_fusion_math.h"
#include "tue_prv_error_management.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_ParameterInterface.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"

#define ObjFusn_START_SEC_CODE

/* PRQA S 1532 2 */ /* Library Function */
uint32 tue_prv_motionType_calculateMotionType(CONSTP2VAR(
    TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    boolean bIsStationary = TRUE;
    float32 f32ObjVelX = FLT_ZERO;
    float32 f32ObjVelY = FLT_ZERO;
    float32 _f32Speed = FLT_ZERO;
    float32 _f32Heading = FLT_ZERO;
    uint16 u16MotionTypeOld = TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN;
    float32 f32ThresholdStationary = FLT_ZERO;

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if (NULL_PTR == pTrkbl) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_MOTION_TYPE,
            TUEOBJFUSN_AAU_MOTION_TYPE_SET_MOTION_TYPE_TRKBL);
    } else
#endif
    {
        f32ObjVelX = pTrkbl->vecX.data[TRACKABLE_VELX];
        f32ObjVelY = pTrkbl->vecX.data[TRACKABLE_VELY];
        _f32Speed = (f32ObjVelX * f32ObjVelX) + (f32ObjVelY * f32ObjVelY);
        _f32Heading = tue_prv_fusion_abs(pTrkbl->f32Heading);
        u16MotionTypeOld = pTrkbl->u16MotionType;

        if (((u16MotionTypeOld ==
              TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY) ||
             (u16MotionTypeOld ==
              TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_STOPPED)) ||
            (u16MotionTypeOld == TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN)) {
            f32ThresholdStationary = TUEOBJFUSN_MOTIONTYPE_TO_MOVING;
        } else {
            f32ThresholdStationary = TUEOBJFUSN_MOTIONTYPE_TO_STATIONARY;
        }

        if (pTrkbl->u8RadarMotionTypeInput ==
            TU_SENSOR_MOTIONG_TYPE_INPUT_STATIONARY) {
            f32ThresholdStationary *= 2;
        }

        if ((f32ThresholdStationary * f32ThresholdStationary) > _f32Speed) {
            bIsStationary = TRUE;
        } else {
            bIsStationary = FALSE;
        }

        /*
         * Previous Motion Type is UNKNOWN, possible transitions are
         *  - CROSSING
         *  - DRIVING
         *  - STATIONARY
         *  - ONCOMING
         */
        if (u16MotionTypeOld == TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN) {
            if (((pTrkbl->u32SensorsHist &
                  TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) > 0u) &&
                ((pTrkbl->u32SensorsHist &
                  TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) == 0u)) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN;
            } else if (TRUE == bIsStationary) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY;
            } else if (_f32Heading <
                       TUEOBJFUSN_MOTIONTYPE_CROSSING_TO_DRIVING) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING;
            } else if (_f32Heading <
                       TUEOBJFUSN_MOTIONTYPE_CROSSING_TO_ONCOMING) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING;
            } else {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_ONCOMING;
            }
        }
        /*
         *  Previous Motion Type is STATIONARY, possible transitions are
         *  - STATIONARY
         *  - ONCOMING
         *  - CROSSING
         *  - DRIVING
         */
        else if (u16MotionTypeOld ==
                 TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY) {
            if (TRUE == bIsStationary) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY;
            } else if (_f32Heading <
                       TUEOBJFUSN_MOTIONTYPE_CROSSING_TO_DRIVING) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING;
            } else if (_f32Heading <
                       TUEOBJFUSN_MOTIONTYPE_CROSSING_TO_ONCOMING) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING;
            } else {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_ONCOMING;
            }
        }
        /*
         *  Previous Motion Type is ONCOMING, possible transitions are
         *  - ONCOMING
         *  - CROSSING
         *  - STATIONARY
         */
        else if (u16MotionTypeOld ==
                 TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_ONCOMING) {
            if (TRUE == bIsStationary) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY;
            } else if (_f32Heading <
                       TUEOBJFUSN_MOTIONTYPE_ONCOMING_TO_CROSSING) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING;
            } else {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_ONCOMING;
            }
        }
        /*
         *  Previous Motion Type is CROSSING, possible transitions are
         *  - ONCOMING
         *  - CROSSING
         *  - STATIONARY
         *  - DRIVING
         */
        else if (u16MotionTypeOld ==
                 TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING) {
            if (TRUE == bIsStationary) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY;
            } else if (_f32Heading <
                       TUEOBJFUSN_MOTIONTYPE_CROSSING_TO_DRIVING) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING;
            } else if (_f32Heading >=
                       TUEOBJFUSN_MOTIONTYPE_CROSSING_TO_ONCOMING) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_ONCOMING;
            } else {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING;
            }
        }
        /*
         *  Previous Motion Type is DRIVING, possible transitions are
         *  - DRIVING
         *  - CROSSING
         *  - STOPPED
         */
        else if (
            u16MotionTypeOld ==
            TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING) {
            if (TRUE == bIsStationary) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_STOPPED;
            } else if (_f32Heading >
                       TUEOBJFUSN_MOTIONTYPE_DRIVING_TO_CROSSING) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING;
            } else {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING;
            }
        }
        /*
         *  Previous Motion Type is STOPPED, possible transitions are
         *  - DRIVING
         *  - REVERSING
         *  - STOPPED
         */
        else if (
            u16MotionTypeOld ==
            TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_STOPPED) {
            if (TRUE == bIsStationary) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_STOPPED;
            } else if (f32ObjVelX < FLT_ZERO) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_REVERSING;
            } else {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING;
            }
        }
        /*
         *  Previous Motion Type is REVERSING, possible transitions are
         *  - STOPPED
         *  - REVERSING
         *  - UNKNOWN
         */
        else if (
            u16MotionTypeOld ==
            TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_REVERSING) {
            if (TRUE == bIsStationary) {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_STOPPED;
            } else if (f32ObjVelX > FLT_ZERO) {
                /* Not supposed to happen, consider this as an error state.
                 * Reset state machine to UNKNOWN in this case */
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN;
            } else {
                pTrkbl->u16MotionType =
                    TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_REVERSING;
            }
        } else {
            pTrkbl->u16MotionType = TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN;
        }
    }

    return u32Success;
}

#define ObjFusn_STOP_SEC_CODE

/** \} */
