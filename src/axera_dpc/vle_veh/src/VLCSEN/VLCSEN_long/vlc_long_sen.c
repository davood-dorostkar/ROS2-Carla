/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "vlc_long_sen_ext.h"
#include "stddef.h"

//#include "cart_ext.h"

#include "acc_ext.h"
#include "acc_par.h"
//#include "acc_out_ext.h"
//#include "obj_ext.h"
//#include "switch_ext.h"
//#include "mat_std_ext.h"
#include "vlc_inhibit_ext.h"
//#include "vlc_par.h"
#include "mat_std_ext.h"

/*****************************************************************************
  MODULE LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/

#ifndef OBJ_NUMBER_LENGTH_OF_OOI_LIST
#define OBJ_NUMBER_LENGTH_OF_OOI_LIST 6u
#endif

/*****************************************************************************
  MODULE LOCAL MACROS
*****************************************************************************/
#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "In long term remove resource locking macros! Assume input signals properly copied/secured!")

#endif

/* Definition of the required MTS alignment, added to unions to assure that
the alignment requirements are met */
#define MTS_ALIGNMENT_DUMMY uint32 MTS_DUMMY;

/*! Define cycle-id for MEAS output of VLC_LONG (formerly COMP_ID_VLC60) */
#define VLC_LONG_SEN_FUNC_ID VLC_MEAS_FUNC_ID

/*! Define channel-ids for different input/output/internal channels of VLC_LONG
 */
#define VLC_ENV_INPUT_CHANID VLC_MEAS_FUNC_CHAN_ID
#define VLC_ENV_OUTPUT_CHANID VLC_MEAS_FUNC_CHAN_ID
#define VLC_ENV_INTERN_CHANID VLC_MEAS_FUNC_CHAN_ID

/* Hysteresis factor for performance degradation */
#define VLC_LONG_MAX_DIST_PERF_HYST 0.1f
#define EGO_ACCEL_FILTER_TIME 0u

/*****************************************************************************
  MODULE GLOBAL TYPEDEFS
*****************************************************************************/
#define CAL_START_CODE
#include "Mem_Map.h"
const volatile uint8 object_dynamic_calculate_mode =
    1;  // 0 for raw data, 1 for filtered data
const volatile uint16 OBJECT_ACCEL_FILTER_TIME = 400u;
const volatile uint16 OBJECT_SPEED_FILTER_TIME = 100u;
#define CAL_STOP_CODE
#include "Mem_Map.h"

/*****************************************************************************
  MODULE GLOBAL VARIABLES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
static union {
    VLC_acc_object_t AVLC_ALERT_OBJECT; /*!< The real ACC alert object data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gAVLC_ALERT_OBJECT;   /*!< @VADDR:0x20020000 @CYCLEID:VLC_ENV */
static const MEASInfo_t MEASINFO_AVLC_ALERT_OBJECT = {
    0x20020000u, sizeof(gAVLC_ALERT_OBJECT), VLC_LONG_SEN_FUNC_ID,
    VLC_ENV_INTERN_CHANID};

static union {
    VLC_acc_object_t
        AVLC_DISPLAY_OBJECT; /*!< The real ACC display object data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gAVLC_DISPLAY_OBJECT;
static const MEASInfo_t MEASINFO_AVLC_DISPLAY_OBJECT = {
    0x20020100u, sizeof(gAVLC_DISPLAY_OBJECT), VLC_LONG_SEN_FUNC_ID,
    VLC_ENV_INTERN_CHANID};

static union {
    VLC_acc_object_t AVLC_OBJECT_LIST[Acc_max_number_ooi];  //< The ACC object
                                                            // of interest data
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gAVLC_OBJECT_LIST;    /*!< @VADDR:0x20020200 @CYCLEID:VLC_ENV */
static const MEASInfo_t MEASINFO_AVLC_OBJECT_LIST = {
    0x20020200u, sizeof(gAVLC_OBJECT_LIST), VLC_LONG_SEN_FUNC_ID,
    VLC_ENV_INTERN_CHANID};

static union {
    acc_input_data_t AVLC_INPUT_DATA; /*!< The real ACC input data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gAVLC_INPUT_DATA;     /*!< @VADDR:0x20020800 @CYCLEID:VLC_ENV */
static const MEASInfo_t MEASINFO_AVLC_INPUT_DATA = {
    0x20020800u, sizeof(gAVLC_INPUT_DATA), VLC_LONG_SEN_FUNC_ID,
    VLC_ENV_INPUT_CHANID};

static union {
    acc_status_t AVLC_STATUS; /*!< The real ACC status information data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gAVLC_STATUS;         /*!< @VADDR:0x20020A00 @CYCLEID:VLC_ENV */
static const MEASInfo_t MEASINFO_AVLC_STATUS = {
    0x20020A00u, sizeof(gAVLC_STATUS), VLC_LONG_SEN_FUNC_ID,
    VLC_ENV_INTERN_CHANID};

static union {
    VLC_acc_output_data_t AVLC_OUTPUT_DATA; /*!< The real data */
    MTS_ALIGNMENT_DUMMY /*!< Needed to align the data to the MTS requirement */
} gAVLC_OUTPUT_DATA;    /*!< @VADDR:0x20020900 @CYCLEID:VLC_ENV */
static const MEASInfo_t MEASINFO_AVLC_OUTPUT_DATA = {
    0x20020900u, sizeof(gAVLC_OUTPUT_DATA), VLC_LONG_SEN_FUNC_ID,
    VLC_ENV_OUTPUT_CHANID};

static union {
    struct {
        uint16 Inhibit_nr;
        VLC_OP_MODE_t ExtOpMode;
        VLC_OP_MODE_t IntOpMode;
        vlc_inhibition_t Inhibition0;
        vlc_inhibition_t Inhibition1;
    } VLC_SEN_DEBUG_DATA;
    MTS_ALIGNMENT_DUMMY
} gVLC_SEN_DEBUG_DATA; /*!< @VADDR:0x20029940 @CYCLEID:VLC_ENV */
static const MEASInfo_t MEASINFO_VLC_SEN_DEBUG_DATA = {
    0x20029940u, sizeof(gVLC_SEN_DEBUG_DATA), VLC_LONG_SEN_FUNC_ID,
    VLC_ENV_INTERN_CHANID};

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static vlc_inhibit_storage_t INHIBIT_BUFFER;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MODULE LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static boolean VLC_LONG_SEN_INITIALIZED = FALSE;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MODULE LOCAL FUNCTIONS
*****************************************************************************/

static void AVLC_OUT_INIT(VLC_acc_output_data_t *acc_output_data);
static void AVLC_COPY_OBJECT_USAGE(VLCSenAccOOI_t *pVLCAccOOIData,
                                   const ObjNumber_t AVLC_LONG_OBJECT_ID,
                                   const VLCObjUsageState_t eUsageState);
static void VLC_COPY_INPUT_OBJ(ObjNumber_t ObjNr,
                               const Envm_t_GenObject *pEmObj,
                               const EM_t_ARSObject *pEmARSObj,
                               const VLCPubObject_t *pFctObj,
                               VLC_acc_object_t *pDestObj,
                               const times_t cycle_time,
                               const AssessedObjList_t *pAssessedObjList);

static void VLC_LONG_SEN_INIT(void);
static void VLC_LONG_MTS_CALLBACK(void);

/*************************************************************************************************************************
  Functionname:    AVLC_OUT_INIT */
static void AVLC_OUT_INIT(VLC_acc_output_data_t *acc_output_data) {
    acc_output_data->AVLC_OUTPUT_STATUS.ALERT = FALSE;
    acc_output_data->AVLC_OUTPUT_STATUS.ALLOW_INIT = FALSE;
    acc_output_data->AVLC_OUTPUT_STATUS.INHIBITED = FALSE;
    acc_output_data->AVLC_OUTPUT_STATUS.INHIBITION_REASON = Acc_inhibition_none;
    acc_output_data->DISTANCE_CTRL_ACCEL_MAX =
        Acc_max_allowed_accel; /*neutral acc accleration*/
    acc_output_data->DISTANCE_CTRL_ACCEL_MIN =
        Acc_max_allowed_accel; /*neutral acc accleration*/
    acc_output_data->HEADWAY_SETTING =
        Acc_default_headway_setting; /*default headway setting*/
    acc_output_data->MAX_AVLC_ACCELERATION =
        Acc_max_allowed_accel; /*neutral limit*/
    acc_output_data->MAX_AVLC_DECELERATION =
        Acc_max_allowed_accel;                          /*neutral limit*/
    acc_output_data->REQUESTED_DISTANCE = Distance_max; /*max distance*/
    acc_output_data->REQUESTED_MAX_INTRUSION =
        Distance_max;                              /*neutral distance*/
    acc_output_data->REQUESTED_TIMEGAP = Time_max; /*max timegap*/
    acc_output_data->SITUATION_CLASS.CRITICALITY = 0u;
    acc_output_data->SITUATION_CLASS.SITUATION = Acc_sit_class_undefined;
}

/*************************************************************************************************************************
  Functionname:    VLC_LONG_SEN_INIT */
static void VLC_LONG_SEN_INIT(void) {
    AVLC_OUT_INIT(&gAVLC_OUTPUT_DATA.AVLC_OUTPUT_DATA);
    AVLC_INIT(&gAVLC_DISPLAY_OBJECT.AVLC_DISPLAY_OBJECT,
              &gAVLC_ALERT_OBJECT.AVLC_ALERT_OBJECT, &gAVLC_STATUS.AVLC_STATUS);
    gAVLC_INPUT_DATA.AVLC_INPUT_DATA.VLC_ACCEL_LIMIT = 2000;
    gAVLC_INPUT_DATA.AVLC_INPUT_DATA.VLC_DECEL_LIMIT = -5000;
    gAVLC_INPUT_DATA.AVLC_INPUT_DATA.DRIVER_CONTROLS.HEADWAY_SETTING =
        Acc_default_headway_setting;

    VLC_INHIBIT_INIT(&INHIBIT_BUFFER);
}

/*************************************************************************************************************************
  Functionname:    VLC_COPY_INPUT_OBJ */
static void VLC_COPY_INPUT_OBJ(ObjNumber_t ObjNr,
                               const Envm_t_GenObject *pEmObj,
                               const EM_t_ARSObject *pEmARSObj,
                               const VLCPubObject_t *pFctObj,
                               VLC_acc_object_t *pDestObj,
                               const times_t cycle_time,
                               const AssessedObjList_t *pAssessedObjList) {
    /* Check if object exists */
    if ((ObjNr >= 0) && (ObjNr < Envm_N_OBJECTS) && (pEmObj != NULL) &&
        (pFctObj != NULL)) {
        /* Save last cycle object ID */
        const ObjNumber_t LastObjId = pDestObj->AUTOSAR.OBJECT_ID;
        boolean ObjMeasured;
        float32 fObjWidth;
        acceleration_t obj_abs_accel_raw;
        velocity_t obj_abs_speed_raw;

        if (pEmARSObj->SensorSpecific.ucMeasuredSources != CR_MEAS_SEN_NONE) {
            ObjMeasured = TRUE;
        } else {
            ObjMeasured = FALSE;
        }

        /* Determine raw absolute object speed */

        /* Internal filtering disabled: use absolute object speed from SI/or
         * derive a simple one */
        obj_abs_speed_raw =
            (velocity_t)(OBJ_ABS_VELO_X(ObjNr) * (float32)Speed_s);

        /* Determine raw absolute object acceleration */
        /* Internal filtering disabled: use absolute object acceleration from
         * SI/or derive a simple one */
        obj_abs_accel_raw =
            (acceleration_t)(OBJ_ABS_ACCEL_X(ObjNr) * (float32)Acceleration_s);

        /*! Object lost information for relevant object */
        if (pAssessedObjList->HeaderAssessedObjList.aiOOIList[OBJ_NEXT_OOI] ==
            OBJ_INDEX_NO_OBJECT) {
            if (pAssessedObjList->HeaderAssessedObjList.eRelObjLossReason !=
                OBJ_LOSS_NO_INFO) {
                gAVLC_INPUT_DATA.AVLC_INPUT_DATA.INPUT_STATUS.OBJECT_LOST =
                    TRUE;
            } else {
                gAVLC_INPUT_DATA.AVLC_INPUT_DATA.INPUT_STATUS.OBJECT_LOST =
                    FALSE;
            }
        }

        pDestObj->AUTOSAR.LAT_DISPLACEMENT =
            (distance_t)(pEmObj->Kinematic.fDistY * (float32)Distance_s);
        /*! Use hypotenuse instead of DistX, use only positive values of DistX
         * for calculation */
        pDestObj->AUTOSAR.LONG_DISPLACEMENT =
            (distance_t)(SQRT(SQR(MAX(0.F, pEmObj->Kinematic.fDistX)) +
                              SQR(pEmObj->Kinematic.fDistY)) *
                         (float32)Distance_s);

        pDestObj->AUTOSAR.OBJECT_ID = ObjNr;
        pDestObj->LAST_OBJECT_ID = LastObjId;
        pDestObj->AUTOSAR.OBJECT_STATUS.DETECTED = TRUE;
        pDestObj->AUTOSAR.OBJECT_STATUS.MEASURED = ObjMeasured;
        if (LastObjId != ObjNr) {
            pDestObj->AUTOSAR.OBJECT_STATUS.NEW = TRUE;
        } else {
            pDestObj->AUTOSAR.OBJECT_STATUS.NEW = FALSE;
        }
        if (!ObjMeasured) {
            pDestObj->AUTOSAR.OBJECT_STATUS.TRACKED = TRUE;
        } else {
            pDestObj->AUTOSAR.OBJECT_STATUS.TRACKED = FALSE;
        }
        /* Decide between stop/standing */

        switch (pEmObj->Attributes.eDynamicProperty) {
            case Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY:
                pDestObj->AUTOSAR.OBJECT_STATUS.STANDING = TRUE;
                pDestObj->AUTOSAR.OBJECT_STATUS.STOPPED = FALSE;
                pDestObj->AUTOSAR.OBJECT_STATUS.MOVING = FALSE;
                break;
            case Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED:
                pDestObj->AUTOSAR.OBJECT_STATUS.STANDING = FALSE;
                pDestObj->AUTOSAR.OBJECT_STATUS.STOPPED = TRUE;
                pDestObj->AUTOSAR.OBJECT_STATUS.MOVING = FALSE;
                break;
            case Envm_GEN_OBJECT_DYN_PROPERTY_MOVING:
                pDestObj->AUTOSAR.OBJECT_STATUS.STANDING = FALSE;
                pDestObj->AUTOSAR.OBJECT_STATUS.STOPPED = FALSE;
                pDestObj->AUTOSAR.OBJECT_STATUS.MOVING = TRUE;
                break;
            case Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING:
            case Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_LEFT:
            case Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_RIGHT:
            case Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN:
            case Envm_GEN_OBJECT_DYN_PROPERTY_MAX_DIFF_TYPES:
            default:
                pDestObj->AUTOSAR.OBJECT_STATUS.STANDING = FALSE;
                pDestObj->AUTOSAR.OBJECT_STATUS.STOPPED = FALSE;
                pDestObj->AUTOSAR.OBJECT_STATUS.MOVING = FALSE;
                break;
        }

        /* Determine object width to use. At default use width as provided by EM
         */
        fObjWidth = pEmARSObj->Geometry.fWidth;

        /* If class confidence exceeds a minimum level, use class to limit width
         */
        if (pEmObj->Attributes.uiClassConfidence > 20u) {
            switch (pEmObj->Attributes.eClassification) {
                case Envm_GEN_OBJECT_CLASS_UNCLASSIFIED:
                    break;
                case Envm_GEN_OBJECT_CLASS_CAR:
                    fObjWidth = MINMAX(1.0f, 2.5f, fObjWidth);
                    break;
                case Envm_GEN_OBJECT_CLASS_TRUCK:
                    fObjWidth = MINMAX(1.5f, 3.0f, fObjWidth);
                    break;
                case Envm_GEN_OBJECT_CLASS_MOTORCYCLE:
                case Envm_GEN_OBJECT_CLASS_PEDESTRIAN:
                case Envm_GEN_OBJECT_CLASS_WIDE:
                case Envm_GEN_OBJECT_CLASS_POINT:
                case Envm_GEN_OBJECT_CLASS_BICYCLE:
                case Envm_GEN_OBJECT_CLASS_MAX_DIFF_TYPES:
                default:
                    break;
            }
        }

        pDestObj->AUTOSAR.WIDTH = (distance_t)(fObjWidth * (float32)Distance_s);

        switch (pFctObj->LaneInformation.eAssociatedLane) {
            case ASSOC_LANE_EGO:
                pDestObj->LANE_INFORMATION = Obj_lane_same;
                pDestObj->AVLC_CUT_IN_OUT_POTENTIAL =
                    (percentage_t)pFctObj->LaneInformation.uiCutOutProbability;
                break;
            case ASSOC_LANE_RIGHT:
                pDestObj->LANE_INFORMATION = Obj_lane_right1;
                pDestObj->AVLC_CUT_IN_OUT_POTENTIAL =
                    (percentage_t)pFctObj->LaneInformation.uiCutInProbability;
                break;
            case ASSOC_LANE_LEFT:
                pDestObj->LANE_INFORMATION = Obj_lane_left1;
                pDestObj->AVLC_CUT_IN_OUT_POTENTIAL =
                    (percentage_t)pFctObj->LaneInformation.uiCutInProbability;
                break;
            case ASSOC_LANE_FAR_LEFT:
            case ASSOC_LANE_FAR_RIGHT:
            case ASSOC_LANE_UNKNOWN:
            default:
                pDestObj->LANE_INFORMATION = Obj_lane_same; /*!!!not defined?*/
                pDestObj->AVLC_CUT_IN_OUT_POTENTIAL =
                    (percentage_t)pFctObj->LaneInformation.uiCutOutProbability;
                break;
        }

        pDestObj->LAT_DISPL_FROM_LANE =
            (distance_t)(pFctObj->Legacy.fDistToRef * (float32)Distance_s);

        /* Note: this filtering took place only for relevant object in ARS300,
        with the other 5 OOI objects using the absolute acceleration as filtered
        by SI. It has now been extended to all objects to have identical
        filtering applied */

        /*! If relevant Object changes, the filter will be initialized with 0
         * m/s2 acceleration */

        if (object_dynamic_calculate_mode == 0) {
            pDestObj->LONG_SPEED = obj_abs_speed_raw;
            pDestObj->LONG_ACCEL = obj_abs_accel_raw;
        } else {
            if (LastObjId == ObjNr) {
                for (uint8 i = VLC_ACC_OBJ_HIST_NUM - 1; i > 0; i--) {
                    pDestObj->LONG_SPEED_HISTORY[i] =
                        pDestObj->LONG_SPEED_HISTORY[i - 1];
                }
                pDestObj->LONG_SPEED_HISTORY[0] = pDestObj->LONG_SPEED;

                pDestObj->LONG_SPEED = MAT_FILT(
                    (sint32)obj_abs_speed_raw, (sint32)pDestObj->LONG_SPEED,
                    (sint32)(OBJECT_SPEED_FILTER_TIME / cycle_time));

                if (pDestObj->LONG_SPEED < 0) {
                    pDestObj->LONG_SPEED = 0;
                }

                obj_abs_accel_raw =
                    (pDestObj->LONG_SPEED -
                     pDestObj->LONG_SPEED_HISTORY[VLC_ACC_OBJ_HIST_NUM - 1]) *
                    Time_s * Acceleration_s /
                    (cycle_time * VLC_ACC_OBJ_HIST_NUM) / Velocity_s;

                if (MAT_ABS(obj_abs_accel_raw) < Acc_object_hyst_accel) {
                    obj_abs_accel_raw = 0;
                } else if (obj_abs_accel_raw > Acc_max_object_possible_accel) {
                    obj_abs_accel_raw = Acc_max_object_possible_accel;
                } else if (obj_abs_accel_raw < Acc_min_object_possible_accel) {
                    obj_abs_accel_raw = Acc_min_object_possible_accel;
                }

                pDestObj->LONG_ACCEL = (acceleration_t)MAT_FILT(
                    (sint32)obj_abs_accel_raw, (sint32)pDestObj->LONG_ACCEL,
                    (sint32)(OBJECT_ACCEL_FILTER_TIME / cycle_time));
            } else {
                pDestObj->LONG_ACCEL = (acceleration_t)0;
                pDestObj->LONG_SPEED = (velocity_t)obj_abs_speed_raw;

                if (pDestObj->LONG_SPEED < 0) {
                    pDestObj->LONG_SPEED = 0;
                }

                for (uint8 i = 0; i < VLC_ACC_OBJ_HIST_NUM; i++) {
                    pDestObj->LONG_SPEED_HISTORY[i] = pDestObj->LONG_SPEED;
                }
            }
        }

        pDestObj->AUTOSAR.QUALITY = (confidence_t)255;
        pDestObj->AUTOSAR.REL_LAT_ACCEL =
            (acceleration_t)(pEmObj->Kinematic.fArelY *
                             (float32)Acceleration_s);
        pDestObj->AUTOSAR.REL_LAT_SPEED =
            (velocity_t)(pEmObj->Kinematic.fVrelY * (float32)Velocity_s);
        pDestObj->AUTOSAR.REL_LONG_ACCEL =
            pDestObj->LONG_ACCEL -
            gAVLC_INPUT_DATA.AVLC_INPUT_DATA.LONG_ACCELERATION;
        pDestObj->AUTOSAR.REL_LONG_SPEED =
            pDestObj->LONG_SPEED -
            gAVLC_INPUT_DATA.AVLC_INPUT_DATA.LONG_VELOCITY;

    } else {
        /* No object currently in given position, delete the object */
        AVLC_DELETE_OBJECT(pDestObj);
    }

    /*internal values*/
    pDestObj->USAGE_STATUS.INTEREST = 0u;
    if (pFctObj != NULL) {
        pDestObj->USAGE_STATUS.LOST_REASON = SIObOOIGetOOILossReason(
            pFctObj->ObjOfInterest.eObjOOI); /*!< to be clarify */
    } else {
        pDestObj->USAGE_STATUS.LOST_REASON = 0u;
    }
    pDestObj->USAGE_STATUS.USE_FOR_ALERT = 0u;
    pDestObj->USAGE_STATUS.USE_FOR_CONTROL = 0u;
    pDestObj->TTC = Acc_si_max_relevant_ttc;
    pDestObj->NEEDED_DECEL = 0;
    pDestObj->CONTROL_ACCEL = Acc_max_allowed_accel;
    pDestObj->MAX_ALLOWED_ACCEL = Acc_max_allowed_accel;
    pDestObj->MAX_ALLOWED_DECEL = Acc_max_allowed_decel;
}

/*************************************************************************************************************************
  Functionname:    VLC_LONG_MTS_CALLBACK */
static void VLC_LONG_MTS_CALLBACK(void) {}

/*************************************************************************************************************************
  Functionname:    AVLC_COPY_OBJECT_USAGE */
static void AVLC_COPY_OBJECT_USAGE(VLCSenAccOOI_t *pVLCAccOOIData,
                                   const ObjNumber_t AVLC_LONG_OBJECT_ID,
                                   const VLCObjUsageState_t eUsageState) {
    /*! Copy object usage to VLCAccOOIData */
    if (AVLC_LONG_OBJECT_ID != OBJ_INDEX_NO_OBJECT) {
        if (pVLCAccOOIData->AccOOINextLong.Attributes.uiObjectID ==
            AVLC_LONG_OBJECT_ID) {
            pVLCAccOOIData->AccOOINextLong.Attributes.eUsageState |=
                eUsageState;
        } else if (pVLCAccOOIData->AccOOIHiddenNextLong.Attributes.uiObjectID ==
                   AVLC_LONG_OBJECT_ID) {
            pVLCAccOOIData->AccOOIHiddenNextLong.Attributes.eUsageState |=
                eUsageState;
        } else if (pVLCAccOOIData->AccOOINextLeft.Attributes.uiObjectID ==
                   AVLC_LONG_OBJECT_ID) {
            pVLCAccOOIData->AccOOINextLeft.Attributes.eUsageState |=
                eUsageState;
        } else if (pVLCAccOOIData->AccOOINextRight.Attributes.uiObjectID ==
                   AVLC_LONG_OBJECT_ID) {
            pVLCAccOOIData->AccOOINextRight.Attributes.eUsageState |=
                eUsageState;
        } else if (pVLCAccOOIData->AccOOIHiddenNextLeft.Attributes.uiObjectID ==
                   AVLC_LONG_OBJECT_ID) {
            pVLCAccOOIData->AccOOIHiddenNextLeft.Attributes.eUsageState |=
                eUsageState;
        } else if (pVLCAccOOIData->AccOOIHiddenNextRight.Attributes
                       .uiObjectID == AVLC_LONG_OBJECT_ID) {
            pVLCAccOOIData->AccOOIHiddenNextRight.Attributes.eUsageState |=
                eUsageState;
        } else {
            /* nothing to do */
        }
    }
}

/*****************************************************************************

  @fn             VLC_LONG_EXEC */ /*!

                                              @description    Execute
                                              longitudinal functions (object
                                              dependent functions)

                                              @param[in]      cycle_time the
                                              cycle time since last call

                                              @param[in]      bSensorBlocked
                                              boolean set to TRUE if sensor is
                                              blocked

                                              @param[in]      pAlnMon pointer to
                                              the alignment monitoring output
                                            */
/*!
  @param[in]      pEmGenObjList
*/

/*!
  @param[in]      pFctObjList
  @param[in]      pVehDyn
  */
/*!
  @param[in]      pDFVLongOut

  @param[out]     pAccDisplayObj pointer to the ACC display object passed to
                  the vehicle cycle CC code

  @param[out]     pAccOutput pointer to the ACC control output data passed to
                  the vehilce cycle CC code

  @return         void

*****************************************************************************/
void VLC_LONG_EXEC(const times_t cycle_time,
                   const boolean bSensorBlocked,
                   const Envm_t_GenObjectList *pEmGenObjList,
                   const AssessedObjList_t *pFctObjList,
                   const VED_VehDyn_t *pVehDyn,
                   const VLC_DFV2SenInfo_t *pDFVLongOut,
                   const VLCCustomInput_t *pVLCCustomInput,
                   VLC_acc_object_t *pAccDisplayObj,
                   VLCSenAccOOI_t *pVLCAccOOIData,
                   VLC_acc_output_data_t *pAccOutput,
                   OvertakeAssistInfo *p_overtake_assist_info) {
    acc_input_data_t *const acc_input_data_ptr =
        &gAVLC_INPUT_DATA.AVLC_INPUT_DATA;
    VLC_acc_object_t acc_display_object;
    VLC_acc_output_data_t acc_output_data;
    SI_LC_t_LaneChangePhaseInfo SILaneChangeInfo;
    uint8 onr;

    vlc_inhibit_t LocalInhibitionBuffer;
    boolean CheckState;

    _PARAM_UNUSED(pVehDyn);

    pAccOutput->uiVersionNumber = VLC_SEN_INTFVER;
    pAccDisplayObj->uiVersionNumber = VLC_SEN_INTFVER;

    /* Initialize the local inhibition buffer */
    VLC_INHIBIT_START_CYCLE(&LocalInhibitionBuffer);

    /*! Execute Init function if SW or HW reset  */
    if ((VLCSenFrame.eVLCState == VLC_SEN_INIT) ||
        (VLC_LONG_SEN_INITIALIZED == FALSE)) {
        VLC_LONG_SEN_INIT();
        VLC_LONG_SEN_INITIALIZED = TRUE;
    }

    /* Check sensor system state */
    CheckState = (boolean)((bSensorBlocked));  // sensor is blocked or self test
                                               // is running or failed

/* Do not check the blockage and alignment states during simulation */
#if (defined(_MSC_VER))
    CheckState = FALSE;
#endif

    VLC_INHIBIT_ADD_INHIBITION(&LocalInhibitionBuffer, CheckState,
                               Fct_inhibit_FctACC | Fct_inhibit_FctFCA |
                                   Fct_inhibit_FctDM | Fct_inhibit_DspTmpNotAv);

    {
        gAVLC_OUTPUT_DATA.AVLC_OUTPUT_DATA.AVLC_OUTPUT_STATUS
            .INHIBITION_REASON = Acc_inhibition_none;
    }
    /*! Add blockage inhibition reason if exists */
    if (bSensorBlocked) {
        gAVLC_OUTPUT_DATA.AVLC_OUTPUT_DATA.AVLC_OUTPUT_STATUS
            .INHIBITION_REASON |= Acc_inhibition_blockage;
    } else {
        gAVLC_OUTPUT_DATA.AVLC_OUTPUT_DATA.AVLC_OUTPUT_STATUS
            .INHIBITION_REASON |= Acc_inhibition_none;
    }

#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "Clarify if state signals in cartronic long ctrl output needed!")
#endif

#if (defined(_MSC_VER))
#pragma COMPILEMSG("Fix all VDATA state queries here!!!!")
#endif

    gVLC_SEN_DEBUG_DATA.VLC_SEN_DEBUG_DATA.Inhibit_nr =
        LocalInhibitionBuffer.INHiBIT_NR;

    /*end reading data from VDATA*/
    /*!!! copy data thread save*/

    acc_display_object = gAVLC_DISPLAY_OBJECT.AVLC_DISPLAY_OBJECT;
    acc_output_data = gAVLC_OUTPUT_DATA.AVLC_OUTPUT_DATA;

    /*wrapper set AVLC_INPUT_DATA*/
    acc_input_data_ptr->VLC_ACCEL_LIMIT = pDFVLongOut->MaxAccelLimit;
    acc_input_data_ptr->VLC_DECEL_LIMIT = pDFVLongOut->MinAccelLimit;
    acc_input_data_ptr->DRIVER_CONTROLS.HEADWAY_SETTING =
        pDFVLongOut->HeadwaySetting;

    /*! Get lane change information from SI, convert to local data structure */
    SILaneChangeInfo = SIGetLaneChangeTimeGap();

    if ((SILaneChangeInfo.t_LCPhaseState == LC_LEFT) &&
        (SILaneChangeInfo.t_LCTrafficOrientation == LC_TRAFFIC_ORIENT_RIGHT)) {
        acc_input_data_ptr->DRIVER_CONTROLS.P_LANE_CHANGE_LEFT =
            (percentage_t)MINMAX(Percentage_min, Percentage_max,
                                 ROUND_TO_UINT(SILaneChangeInfo.f_LCPhaseProb));

        acc_input_data_ptr->DRIVER_CONTROLS.P_LANE_CHANGE_RIGHT =
            (percentage_t)0;
    } else if ((SILaneChangeInfo.t_LCPhaseState == LC_RIGHT) &&
               (SILaneChangeInfo.t_LCTrafficOrientation ==
                LC_TRAFFIC_ORIENT_LEFT)) {
        acc_input_data_ptr->DRIVER_CONTROLS.P_LANE_CHANGE_LEFT =
            (percentage_t)0;
        acc_input_data_ptr->DRIVER_CONTROLS.P_LANE_CHANGE_RIGHT =
            (percentage_t)MINMAX(Percentage_min, Percentage_max,
                                 ROUND_TO_UINT(SILaneChangeInfo.f_LCPhaseProb));

    } else {
        acc_input_data_ptr->DRIVER_CONTROLS.P_LANE_CHANGE_LEFT =
            (percentage_t)0;
        acc_input_data_ptr->DRIVER_CONTROLS.P_LANE_CHANGE_RIGHT =
            (percentage_t)0;
    }

    acc_input_data_ptr->INPUT_STATUS.AVLC_ON = pDFVLongOut->AccOn;
    acc_input_data_ptr->INPUT_STATUS.INHIBIT = FALSE;
    acc_input_data_ptr->INPUT_STATUS.VLC_DECEL_LIM_OVERRIDE =
        pDFVLongOut->DecelLimOverride;
    acc_input_data_ptr->INPUT_STATUS.AVLC_CONTROL_TO_FIRST_OBJECT =
        pDFVLongOut->CtrlToRelevObj;
    acc_input_data_ptr->INPUT_STATUS.OBJECT_EFFECTIVE =
        pDFVLongOut->ObjectEffective;
    acc_input_data_ptr->INPUT_STATUS.OBJECT_LOST =
        FALSE; /*will be set later in code*/

    acc_input_data_ptr->LODM_STAT.STANDSTILL = pDFVLongOut->StandStill;
    acc_input_data_ptr->LODM_STAT.OVERRIDE_ACCEL = pDFVLongOut->OverrideAccel;

    acc_input_data_ptr->LONG_ACCELERATION = (acceleration_t)MAT_LIM(
        MAT_FILT(ROUND_TO_INT(pVehDyn->Longitudinal.MotVar.Accel *
                              (float32)Acceleration_s),
                 (sint32)acc_input_data_ptr->LONG_ACCELERATION,
                 (sint32)(EGO_ACCEL_FILTER_TIME / cycle_time)),
        Accel_min, Accel_max);

    acc_input_data_ptr->LONG_VELOCITY = (velocity_t)MAT_LIM(
        ROUND_TO_INT(pVehDyn->Longitudinal.VeloCorr.corrVelo *
                     (float32)Velocity_s),
        Velocity_min, Velocity_max);

    acc_input_data_ptr->VISIBILITY_RANGE = (distance_t)(180 * Distance_s);

#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "Additional verification needed that new filters in object copy VLC_COPY_INPUT_OBJ correspond to this old code!")
#endif

    /* Go through OOI list */
    for (onr = 0; onr < Acc_max_number_ooi; onr++) {
        // uint8           last_id =
        // gAVLC_OBJECT_LIST.AVLC_OBJECT_LIST[onr].AUTOSAR.OBJECT_ID;
        // AVLC_DELETE_OBJECT(&gAVLC_OBJECT_LIST.AVLC_OBJECT_LIST[onr]);
        if (onr < OBJ_NUMBER_LENGTH_OF_OOI_LIST) {
            const ObjNumber_t CurObjId =
                pFctObjList->HeaderAssessedObjList.aiOOIList[onr];
            const Envm_t_GenObject *pCurEmObj;
            const VLCPubObject_t *pCurFctObj;
            const fDistance_t f_MaxDistPerf = 250.0f;
            const fDistance_t f_MinDistPerf = 250.0f;

            /*! Pointer Init */
            pCurEmObj = NULL;
            pCurFctObj = NULL;

            if ((CurObjId >= 0) && (CurObjId < Envm_N_OBJECTS)) {
                if (/*! Distance hysteresis for performance degradation */
                    (pEmGenObjList->aObject[CurObjId].Kinematic.fDistX <
                     f_MinDistPerf) ||
                    ((gAVLC_OBJECT_LIST.AVLC_OBJECT_LIST[onr]
                          .AUTOSAR.OBJECT_ID == CurObjId) &&
                     (pEmGenObjList->aObject[CurObjId].Kinematic.fDistX <
                      (f_MaxDistPerf +
                       f_MaxDistPerf * VLC_LONG_MAX_DIST_PERF_HYST))) ||
                    ((gAVLC_OBJECT_LIST.AVLC_OBJECT_LIST[onr]
                          .AUTOSAR.OBJECT_ID != CurObjId) &&
                     (pEmGenObjList->aObject[CurObjId].Kinematic.fDistX <
                      f_MaxDistPerf))) {
                    pCurEmObj = &pEmGenObjList->aObject[CurObjId];
                    pCurFctObj = &pFctObjList->ObjList[CurObjId];
                }
            }
            VLC_COPY_INPUT_OBJ(
                CurObjId, pCurEmObj, &VLCSEN_pEmARSObjList->aObject[CurObjId],
                pCurFctObj, &gAVLC_OBJECT_LIST.AVLC_OBJECT_LIST[onr],
                cycle_time, pFctObjList);
        }
    }

    /*------------------------------- data wrapping done
     * ----------------------------------------------*/

    VLC_INHIBIT_FINISH_CYCLE(&LocalInhibitionBuffer, &INHIBIT_BUFFER,
                             Fct_inhibit_task_LONG_EXEC);

    /* Store some debug data for meas output */
    gVLC_SEN_DEBUG_DATA.VLC_SEN_DEBUG_DATA.ExtOpMode =
        INHIBIT_BUFFER.EXTERNAL_OP_MODE;
    gVLC_SEN_DEBUG_DATA.VLC_SEN_DEBUG_DATA.IntOpMode =
        INHIBIT_BUFFER.INTERNAL_OP_MODE;
    gVLC_SEN_DEBUG_DATA.VLC_SEN_DEBUG_DATA.Inhibition0 =
        INHIBIT_BUFFER.INHIBIT[0].INHIBITION;
    gVLC_SEN_DEBUG_DATA.VLC_SEN_DEBUG_DATA.Inhibition1 =
        INHIBIT_BUFFER.INHIBIT[1].INHIBITION;

    /* If our local inhibition buffer indicates that ACC is inhibited, then set
     * the inhibition boolean */
    /* acc_input_data_ptr->INPUT_STATUS.INHIBIT |=
     * VLC_INHIBIT_GET_INHIBITION(LocalInhibitionBuffer, Fct_inhibit_FctACC); */
    /* without ignition cycle */
    acc_input_data_ptr->INPUT_STATUS.INHIBIT =
        VLC_INHIBIT_GET_INHIBITION(LocalInhibitionBuffer, Fct_inhibit_FctACC);

    AVLC_SELECT_PARAM_SET(VLC_BSW_ALGO_PARAM_PTR->Fct.General.FnSwitchBits);

    AVLC_EXEC(cycle_time, pVLCCustomInput, pDFVLongOut, acc_input_data_ptr,
              gAVLC_OBJECT_LIST.AVLC_OBJECT_LIST, &acc_output_data,
              &gAVLC_ALERT_OBJECT.AVLC_ALERT_OBJECT, &acc_display_object,
              &gAVLC_STATUS.AVLC_STATUS, p_overtake_assist_info);

    /*! Copy usage attribute to VLCSenAccOOI Port */
    /*! CONTROL_OBJECT */
    AVLC_COPY_OBJECT_USAGE(
        pVLCAccOOIData,
        gAVLC_STATUS.AVLC_STATUS.AVLC_CONTROL_OBJECT.AUTOSAR.OBJECT_ID,
        OBJ_USAGE_CONTROL);

    /*! DISPLAY__OBJECT */
    AVLC_COPY_OBJECT_USAGE(pVLCAccOOIData, acc_display_object.AUTOSAR.OBJECT_ID,
                           OBJ_USAGE_DISPLAY);

    /*! ALERT__OBJECT */
    AVLC_COPY_OBJECT_USAGE(
        pVLCAccOOIData, gAVLC_ALERT_OBJECT.AVLC_ALERT_OBJECT.AUTOSAR.OBJECT_ID,
        OBJ_USAGE_ALERT);

    /* Copy ACC display object to destination buffer */
    *pAccDisplayObj = acc_display_object;

    /* Copy ACC output data to destination buffer */
    *pAccOutput = acc_output_data;

    gAVLC_DISPLAY_OBJECT.AVLC_DISPLAY_OBJECT = acc_display_object;
    gAVLC_OUTPUT_DATA.AVLC_OUTPUT_DATA = acc_output_data;

    //(void)VLC_FREEZE_DATA(&MEASINFO_AVLC_ALERT_OBJECT,
    //&gAVLC_ALERT_OBJECT.MTS_DUMMY,    &VLC_LONG_MTS_CALLBACK);
    //(void)VLC_FREEZE_DATA(&MEASINFO_AVLC_DISPLAY_OBJECT,
    //&gAVLC_DISPLAY_OBJECT.MTS_DUMMY,&VLC_LONG_MTS_CALLBACK);
    //(void)VLC_FREEZE_DATA(&MEASINFO_AVLC_OBJECT_LIST,
    //&gAVLC_OBJECT_LIST.MTS_DUMMY,      &VLC_LONG_MTS_CALLBACK);
    //(void)VLC_FREEZE_DATA(&MEASINFO_AVLC_INPUT_DATA,
    //&gAVLC_INPUT_DATA.MTS_DUMMY,    &VLC_LONG_MTS_CALLBACK);
    //(void)VLC_FREEZE_DATA(&MEASINFO_AVLC_OUTPUT_DATA,
    //&gAVLC_OUTPUT_DATA.MTS_DUMMY,  &VLC_LONG_MTS_CALLBACK);
    //(void)VLC_FREEZE_DATA(&MEASINFO_AVLC_STATUS, &gAVLC_STATUS.MTS_DUMMY,
    //&VLC_LONG_MTS_CALLBACK);
    //(void)VLC_FREEZE_DATA(&MEASINFO_VLC_SEN_DEBUG_DATA,
    //&gVLC_SEN_DEBUG_DATA.MTS_DUMMY, &VLC_LONG_MTS_CALLBACK);
}

/*----- global functions-----*/

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */