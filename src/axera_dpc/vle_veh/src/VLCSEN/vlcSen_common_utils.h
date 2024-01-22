#pragma once

#ifndef VLCSEN_COMMON_UTILS_H
#define VLCSEN_COMMON_UTILS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "tue_common_libs.h"

#define Envm_N_OBJECTS 40

//==vlc_sen.h
#define GET_Envm_VLC_CYCLE_MODE_PTR VLCSEN_pECAMtCyclEnvmode
#define GET_Envm_PUB_OBJ_DATA_PTR VLCSEN_pEmGenObjList
#define GET_EM_ARS_OBJ_LIST_PTR VLCSEN_pEmARSObjList

#define GET_EGO_OBJ_SYNC_DATA_PTR VLCSEN_pEgoDynObjSync
#define GET_EGO_RAW_DATA_PTR VLCSEN_pEgoDynRaw
#define GET_EGO_STATIC_DATA_PTR VLCSEN_pGlobEgoStatic
#define GET_ROAD_DATA_PTR VLCSEN_pRoadData

#define RSP_GET_KONTEXT_DATA_PTR VLCSEN_pRSPContextData
#define ALN_MON_INPUT_PTR VLCSEN_pAlignmentMonInput

#define GET_VLC_PUB_OBJ_DATA_PTR VLCSEN_pPubFctObjList
#define GET_SPM_DATA_PTR VLCSEN_pSysPerfMonStates

#define GET_RSP_PD_OUTPUT_DATA_PTR VLCSEN_pRSPOutputPD

#define OBJ_GET_CP(iObj) GET_VLC_OBJ(iObj).CP

#define OBJ_GET_SI(iObj) GET_VLC_OBJ(iObj).SI

#define GET_VLC_AVLC_OOI_DATA_PTR VLCSEN_pAccOOIData

/* component defines */
#define CP_CYCLE_TIME TASK_CYCLE_TIME   // TPGetCycleTime()
#define SPM_CYCLE_TIME TASK_CYCLE_TIME  // TPGetCycleTime()

#define VLC_BSW_ALGO_PARAM_PTR VLCSEN_pBswAlgoParameters

#define VLC_CYCLE_TIME TASK_CYCLE_TIME  // TPGetCycleTime

#define OBJ_ABS_VELO_X(iObj) \
    VLCSEN_pCustomOutput->CustObjData[iObj].AbsolutKinematics.fAbsVelocityX
#define OBJ_ABS_ACCEL_X(iObj) \
    VLCSEN_pCustomOutput->CustObjData[iObj].AbsolutKinematics.fAbsAccelerationX

/*! Macros for accessing ACC Assessed Object List per object (based on
 * GET_VLC_OBJ_PUB(iObj)) */
#define OBJ_GET_ASSOCIATED_LANE(iObj) \
    GET_VLC_OBJ_PUB(iObj)             \
        .LaneInformation.eAssociatedLane /*!< Remark: SIGetAssociateLane */
#define OBJ_GET_FUNC_LANE(iObj) \
    GET_VLC_OBJ_PUB(iObj).LaneInformation.eFuncAssociatedLane
#define OBJ_GET_OOI_POS(iObj) GET_VLC_OBJ_PUB(iObj).ObjOfInterest.eObjOOI
#define OBJ_GET_RELEVANT(iObj)                                               \
    (OBJ_GET_OOI_POS(iObj) ==                                                \
     OBJ_NEXT_OOI) /*!< Remark: ((iObj >= 0) && (iObj == EMRelObjPrevCycle)) \
                      */
#define OBJ_GET_OBJ_TO_REF_DISTANCE(iObj)                              \
    GET_VLC_OBJ_PUB(iObj)                                              \
        .Legacy.fDistToRef /*!< Remark:                                \
                              SITrajGetObjToRefDistance((ui32_t)i_Obj, \
                              &fDistLat, &fDistLatVar) */
#define OBJ_GET_CUT_IN_POTENTIAL(iObj) \
    GET_VLC_OBJ_PUB(iObj).LaneInformation.uiCutInProbability
#define OBJ_GET_CUT_OUT_POTENTIAL(iObj) \
    GET_VLC_OBJ_PUB(iObj).LaneInformation.uiCutOutProbability
#define OBJ_GET_EXTERNAL_OBJ_ID(iObj) \
    GET_VLC_OBJ_PUB(iObj).ObjOfInterest.cExternalID

/*! Macros for accessing ACC Assessed Object List object of interest array
 * (based on GET_VLC_PUB_OBJ_DATA_PTR) */
#define OBJ_GET_OOI_LIST_OBJ_IDX(iOoiPos) \
    GET_VLC_PUB_OBJ_DATA_PTR->HeaderAssessedObjList.aiOOIList[iOoiPos]
#define OBJ_GET_RELEVANT_OBJ_NR \
    OBJ_GET_OOI_LIST_OBJ_IDX(OBJ_NEXT_OOI) /*!< Remark: iRelObjNr */

//====define
#ifndef MEASFreezeDataDLG
#define MEASFreezeDataDLG(x, y, z) NULL_FUNC()
#endif

#ifndef TASK_ID_ALGO_SEN_CYCLE
#define TASK_ID_ALGO_SEN_CYCLE 208U
#endif

/* component defines */

#ifndef CP_CYCLE_TIME
#define CP_CYCLE_TIME TASK_CYCLE_TIME  // TPGetCycleTime()
#endif

#ifndef SPM_CYCLE_TIME
#define SPM_CYCLE_TIME TASK_CYCLE_TIME  // TPGetCycleTime()
#endif

#define GET_Envm_PUB_OBJ_DATA_PTR VLCSEN_pEmGenObjList
#define GET_Envm_GEN_OBJ(iObj) GET_Envm_PUB_OBJ_DATA_PTR->aObject[iObj]

/* MACROS (GLOBAL) */
#ifndef SET_MEMSEC_VAR_A
#define SET_MEMSEC_VAR_A(v)
#endif
#ifndef SET_MEMSEC_VAR
#define SET_MEMSEC_VAR(v)
#endif
#ifndef SET_MEMSEC_CONST_A
#define SET_MEMSEC_CONST_A(v)
#endif
#ifndef SET_MEMSEC_CONST
#define SET_MEMSEC_CONST(v)
#endif
#ifndef MEMSEC_REF
#define MEMSEC_REF
#endif

#ifndef MAX_NUM_OF_HYPS
#define MAX_NUM_OF_HYPS 6
#endif

#ifndef VLC_CAM_LANE_NUM_LANES
#define VLC_CAM_LANE_NUM_LANES 4
#endif

#ifndef CL_CAM_LANE_MK_ADJ_LEFT
#define CL_CAM_LANE_MK_ADJ_LEFT 0U
#endif
#ifndef CL_CAM_LANE_MK_LEFT
#define CL_CAM_LANE_MK_LEFT 1U
#endif
#ifndef CL_CAM_LANE_MK_RIGHT
#define CL_CAM_LANE_MK_RIGHT 2U
#endif
#ifndef CL_CAM_LANE_MK_ADJ_RIGHT
#define CL_CAM_LANE_MK_ADJ_RIGHT 3U
#endif

#ifndef CL_MARKER_COLOR_UNKNOWN
#define CL_MARKER_COLOR_UNKNOWN 0U
#endif
#ifndef CL_MARKER_COLOR_WHITE
#define CL_MARKER_COLOR_WHITE 1U
#endif
#ifndef CL_MARKER_COLOR_YELLOW
#define CL_MARKER_COLOR_YELLOW 2U
#endif
#ifndef CL_MARKER_COLOR_BLUE
#define CL_MARKER_COLOR_BLUE 3U
#endif
#ifndef CL_MARKER_COLOR_GREEN
#define CL_MARKER_COLOR_GREEN 4U
#endif
#ifndef CL_MARKER_COLOR_RED
#define CL_MARKER_COLOR_RED 5U
#endif
#ifndef CL_MARKER_COLOR_SNA
#define CL_MARKER_COLOR_SNA 6U
#endif
#ifndef CL_MARKER_COLOR_UNDEFINED1
#define CL_MARKER_COLOR_UNDEFINED1 7U
#endif
#ifndef CL_MARKER_COLOR_UNDEFINED2
#define CL_MARKER_COLOR_UNDEFINED2 8U
#endif
#ifndef CL_MARKER_COLOR_UNDEFINED3
#define CL_MARKER_COLOR_UNDEFINED3 9U
#endif

#ifndef CL_MARKER_TYPE_CONTINUOUS
#define CL_MARKER_TYPE_CONTINUOUS 0U
#endif
#ifndef CL_MARKER_TYPE_DASHED
#define CL_MARKER_TYPE_DASHED 1U
#endif
#ifndef CL_MARKER_TYPE_RESERVED1
#define CL_MARKER_TYPE_RESERVED1 2U
#endif
#ifndef CL_MARKER_TYPE_RESERVED2
#define CL_MARKER_TYPE_RESERVED2 3U
#endif
#ifndef CL_MARKER_TYPE_NOLINEDETECTED
#define CL_MARKER_TYPE_NOLINEDETECTED 4U
#endif
#ifndef CL_MARKER_TYPE_UNCLASSIFIED
#define CL_MARKER_TYPE_UNCLASSIFIED 5U
#endif
#ifndef CL_MARKER_TYPE_DOTTED
#define CL_MARKER_TYPE_DOTTED 6U
#endif
#ifndef CL_MARKER_TYPE_DECORATION
#define CL_MARKER_TYPE_DECORATION 7U
#endif
#ifndef CL_MARKER_TYPE_SEPARATING
#define CL_MARKER_TYPE_SEPARATING 8U
#endif
#ifndef CL_MARKER_TYPE_NARROWDASHED
#define CL_MARKER_TYPE_NARROWDASHED 9U
#endif
#ifndef CL_MARKER_TYPE_LOWCURB
#define CL_MARKER_TYPE_LOWCURB 10U
#endif
#ifndef CL_MARKER_TYPE_HIGHCURB
#define CL_MARKER_TYPE_HIGHCURB 11U
#endif
#ifndef CL_MARKER_TYPE_CRASHBARRIER
#define CL_MARKER_TYPE_CRASHBARRIER 12U
#endif
#ifndef CL_MARKER_TYPE_WALL
#define CL_MARKER_TYPE_WALL 13U
#endif
#ifndef CL_MARKER_TYPE_ROADSHOULDER
#define CL_MARKER_TYPE_ROADSHOULDER 14U
#endif
#ifndef CL_MARKER_TYPE_SNA
#define CL_MARKER_TYPE_SNA 15U
#endif
#ifndef CL_MARKER_TYPE_GRASS
#define CL_MARKER_TYPE_GRASS 16U
#endif
#ifndef CL_MARKER_TYPE_MISCELLANEOUS
#define CL_MARKER_TYPE_MISCELLANEOUS 17U
#endif

#define TRACE_VALID_NO_OBJ_ID ((uint8)249)

#ifndef TRAFFICORIENTATION_UNKNOWN
#define TRAFFICORIENTATION_UNKNOWN 0U
#endif

#define CONV_CYCLES_TO_LIFETIME(c) (c)

//======

#ifndef _PARAM_UNUSED
#define _PARAM_UNUSED(x) (void)(x)
#endif

#ifndef TRACE_NO_OF_TRACES
#define TRACE_NO_OF_TRACES 10
#endif

#ifndef OBJ_NOT_OOI
#define OBJ_NOT_OOI -1
#endif
#ifndef OBJ_NEXT_OOI
#define OBJ_NEXT_OOI 0
#endif
#ifndef OBJ_HIDDEN_NEXT_OOI
#define OBJ_HIDDEN_NEXT_OOI 1
#endif
#ifndef OBJ_NEXT_LONG_LEFT_OOI
#define OBJ_NEXT_LONG_LEFT_OOI 2
#endif
#ifndef OBJ_NEXT_LONG_RIGHT_OOI
#define OBJ_NEXT_LONG_RIGHT_OOI 3
#endif
#ifndef OBJ_NEXT_LAT_LEFT_OOI
#define OBJ_NEXT_LAT_LEFT_OOI 4
#endif
#ifndef OBJ_NEXT_LAT_RIGHT_OOI
#define OBJ_NEXT_LAT_RIGHT_OOI 5
#endif

#ifndef ASSOC_LANE_UNKNOWN
#define ASSOC_LANE_UNKNOWN 0U
#endif
#ifndef ASSOC_LANE_FAR_LEFT
#define ASSOC_LANE_FAR_LEFT 1U
#endif
#ifndef ASSOC_LANE_LEFT
#define ASSOC_LANE_LEFT 2U
#endif
#ifndef ASSOC_LANE_EGO
#define ASSOC_LANE_EGO 3U
#endif
#ifndef ASSOC_LANE_RIGHT
#define ASSOC_LANE_RIGHT 4U
#endif
#ifndef ASSOC_LANE_FAR_RIGHT
#define ASSOC_LANE_FAR_RIGHT 5U
#endif

#ifndef AL_SIG_STATE_INIT
#define AL_SIG_STATE_INIT 0U
#endif
#ifndef AL_SIG_STATE_OK
#define AL_SIG_STATE_OK 1U
#endif
#ifndef AL_SIG_STATE_INVALID
#define AL_SIG_STATE_INVALID 2U
#endif

#ifndef VED_LONG_MOT_STATE_MOVE
#define VED_LONG_MOT_STATE_MOVE 0U
#endif
#ifndef VED_LONG_MOT_STATE_MOVE_FWD
#define VED_LONG_MOT_STATE_MOVE_FWD 1U
#endif
#ifndef VED_LONG_MOT_STATE_MOVE_RWD
#define VED_LONG_MOT_STATE_MOVE_RWD 2U
#endif
#ifndef VED_LONG_MOT_STATE_STDST
#define VED_LONG_MOT_STATE_STDST 3U
#endif

#define EGO_CURVE_OBJ_SYNC GET_EGO_OBJ_SYNC_DATA_PTR->Lateral.Curve.Curve

#ifndef Envm_GEN_OBJECT_MT_STATE_DELETED
#define Envm_GEN_OBJECT_MT_STATE_DELETED 0U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_NEW
#define Envm_GEN_OBJECT_MT_STATE_NEW 1U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_MEASURED
#define Envm_GEN_OBJECT_MT_STATE_MEASURED 2U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_PREDICTED
#define Envm_GEN_OBJECT_MT_STATE_PREDICTED 3U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_INACTIVE
#define Envm_GEN_OBJECT_MT_STATE_INACTIVE 4U
#endif
#ifndef Envm_GEN_OBJECT_MT_STATE_MAX_DIFF_TYPES
#define Envm_GEN_OBJECT_MT_STATE_MAX_DIFF_TYPES 5U
#endif

// RTE_OPSAdapter
#ifndef CR_OBJECT_PROPERTY_MOVING
#define CR_OBJECT_PROPERTY_MOVING 0U
#endif
#ifndef CR_OBJECT_PROPERTY_STATIONARY
#define CR_OBJECT_PROPERTY_STATIONARY 1U
#endif
#ifndef CR_OBJECT_PROPERTY_ONCOMING
#define CR_OBJECT_PROPERTY_ONCOMING 2U
#endif

#ifndef CR_OBJECT_SUBPROP_UNIFAL
#define CR_OBJECT_SUBPROP_UNIFAL 0U
#endif
#ifndef CR_OBJECT_SUBPROP_CROSSING
#define CR_OBJECT_SUBPROP_CROSSING 1U
#endif

#ifndef CR_OBJECT_MOVSTATE_STATIONARY
#define CR_OBJECT_MOVSTATE_STATIONARY 0U
#endif
#ifndef CR_OBJECT_MOVSTATE_STOPPED
#define CR_OBJECT_MOVSTATE_STOPPED 1U
#endif
#ifndef CR_OBJECT_MOVSTATE_MOVING
#define CR_OBJECT_MOVSTATE_MOVING 2U
#endif

#ifndef CR_OBJCLASS_POINT
#define CR_OBJCLASS_POINT 0U
#endif
#ifndef CR_OBJCLASS_CAR
#define CR_OBJCLASS_CAR 1U
#endif
#ifndef CR_OBJCLASS_TRUCK
#define CR_OBJCLASS_TRUCK 2U
#endif
#ifndef CR_OBJCLASS_PEDESTRIAN
#define CR_OBJCLASS_PEDESTRIAN 3U
#endif
#ifndef CR_OBJCLASS_MOTORCYCLE
#define CR_OBJCLASS_MOTORCYCLE 4U
#endif
#ifndef CR_OBJCLASS_BICYCLE
#define CR_OBJCLASS_BICYCLE 5U
#endif
#ifndef CR_OBJCLASS_WIDE
#define CR_OBJCLASS_WIDE 6U
#endif
#ifndef CR_OBJCLASS_UNCLASSIFIED
#define CR_OBJCLASS_UNCLASSIFIED 7U
#endif

#ifndef SI_CUSTOM_OUTPUT_DEBUG_DATA_VADDR
#define SI_CUSTOM_OUTPUT_DEBUG_DATA_VADDR 539362432U
#endif
#ifndef SI_CUTINOBJ_DATA_MEAS_VADDR
#define SI_CUTINOBJ_DATA_MEAS_VADDR 539362560U
#endif
#ifndef SI_MEAS_ADDR_CALIB_SEEK_WIDTH
#define SI_MEAS_ADDR_CALIB_SEEK_WIDTH 539363072U
#endif
#ifndef SI_MEAS_ADDR_DEBUG_OBJ_SCORES
#define SI_MEAS_ADDR_DEBUG_OBJ_SCORES 539366400U
#endif
#ifndef SI_MEAS_ADDR_LANE_CHANGE
#define SI_MEAS_ADDR_LANE_CHANGE 539363328U
#endif
#ifndef SI_MEAS_ADDR_OBJECT_SCORES
#define SI_MEAS_ADDR_OBJECT_SCORES 539365376U
#endif
#ifndef SI_MEAS_ADDR_OBJ_SCORE_EGO_DYN
#define SI_MEAS_ADDR_OBJ_SCORE_EGO_DYN 539367424U
#endif
#ifndef SI_MEAS_ADDR_SCORE_CAM_LANE
#define SI_MEAS_ADDR_SCORE_CAM_LANE 539368960U
#endif
#ifndef SI_MEAS_ADDR_SCORE_ROAD_ESTI
#define SI_MEAS_ADDR_SCORE_ROAD_ESTI 539367680U
#endif
#ifndef SI_MEAS_ADDR_SCORE_TRACES
#define SI_MEAS_ADDR_SCORE_TRACES 539367936U
#endif
#ifndef SI_MEAS_ADDR_SCORING_TRACE_LANES
#define SI_MEAS_ADDR_SCORING_TRACE_LANES 539369216U
#endif
#ifndef SI_MEAS_ID_SI_COURSE_DATA
#define SI_MEAS_ID_SI_COURSE_DATA 539364096U
#endif
#ifndef SI_MEAS_ID_SI_TRAJECTORY
#define SI_MEAS_ID_SI_TRAJECTORY 539363584U
#endif
#ifndef SI_OOI_LIST_MEAS_VADDR
#define SI_OOI_LIST_MEAS_VADDR 539361536U
#endif
#ifndef SI_RANGE_FACTOR_MEAS_VADDR
#define SI_RANGE_FACTOR_MEAS_VADDR 539362304U
#endif

#ifndef OBJ_LOSS_NO_INFO
#define OBJ_LOSS_NO_INFO 0U
#endif
#ifndef OBJ_LOSS_DISAPPEARED
#define OBJ_LOSS_DISAPPEARED 1U
#endif
#ifndef OBJ_LOSS_LANE_CHG_LEFT
#define OBJ_LOSS_LANE_CHG_LEFT 2U
#endif
#ifndef OBJ_LOSS_LANE_CHG_RIGHT
#define OBJ_LOSS_LANE_CHG_RIGHT 3U
#endif
#ifndef OBJ_LOSS_CURVE_LEFT
#define OBJ_LOSS_CURVE_LEFT 4U
#endif
#ifndef OBJ_LOSS_CURVE_RIGHT
#define OBJ_LOSS_CURVE_RIGHT 5U
#endif
#ifndef OBJ_LOSS_CURVE_LEFT_AHEAD
#define OBJ_LOSS_CURVE_LEFT_AHEAD 6U
#endif
#ifndef OBJ_LOSS_CURVE_RIGHT_AHEAD
#define OBJ_LOSS_CURVE_RIGHT_AHEAD 7U
#endif
#ifndef OBJ_LOSS_STEER_LEFT
#define OBJ_LOSS_STEER_LEFT 8U
#endif
#ifndef OBJ_LOSS_STEER_RIGHT
#define OBJ_LOSS_STEER_RIGHT 9U
#endif
#ifndef OBJ_LOSS_RANGE_REDUCTION
#define OBJ_LOSS_RANGE_REDUCTION 10U
#endif

#ifndef TRAFFICORIENTATION_UNKNOWN
#define TRAFFICORIENTATION_UNKNOWN 0U
#endif
#ifndef GDB_TRAFFICORIENTATION_RIGHT_HAND
#define GDB_TRAFFICORIENTATION_RIGHT_HAND 1U
#endif
#ifndef GDB_TRAFFICORIENTATION_LEFT_HAND
#define GDB_TRAFFICORIENTATION_LEFT_HAND 2U
#endif

#ifndef CR_MEAS_SEN_NONE
#define CR_MEAS_SEN_NONE 0U
#endif
#ifndef CR_MEAS_SEN_FCRCAN
#define CR_MEAS_SEN_FCRCAN 1U
#endif
#ifndef CR_MEAS_SEN_NECRCAN
#define CR_MEAS_SEN_NECRCAN 2U
#endif
#ifndef CR_MEAS_SEN_GRID
#define CR_MEAS_SEN_GRID 4U
#endif
#ifndef CR_MEAS_SEN_CAM
#define CR_MEAS_SEN_CAM 8U
#endif
#ifndef CR_MEAS_SEN_1
#define CR_MEAS_SEN_1 16U
#endif
#ifndef CR_MEAS_SEN_2
#define CR_MEAS_SEN_2 32U
#endif
#ifndef CR_MEAS_SEN_3
#define CR_MEAS_SEN_3 64U
#endif
#ifndef CR_MEAS_SEN_4
#define CR_MEAS_SEN_4 128U
#endif

#ifndef CR_LONGVEHICLE_TYPE_UNIFAL
#define CR_LONGVEHICLE_TYPE_UNIFAL 0U
#endif
#ifndef CR_LONGVEHICLE_TYPE_REAL
#define CR_LONGVEHICLE_TYPE_REAL 1U
#endif
#ifndef CR_LONGVEHICLE_TYPE_MIRROR
#define CR_LONGVEHICLE_TYPE_MIRROR 2U
#endif
#ifndef CR_LONGVEHICLE_TYPE_SHADOW
#define CR_LONGVEHICLE_TYPE_SHADOW 3U
#endif
#ifndef CR_LONGVEHICLE_TYPE_MIDDLE
#define CR_LONGVEHICLE_TYPE_MIDDLE 4U
#endif

//#RTE_DIAGSRC
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_MOVING
#define Envm_GEN_OBJECT_DYN_PROPERTY_MOVING 0U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY
#define Envm_GEN_OBJECT_DYN_PROPERTY_STATIONARY 1U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING
#define Envm_GEN_OBJECT_DYN_PROPERTY_ONCOMING 2U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_LEFT
#define Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_LEFT 3U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_RIGHT
#define Envm_GEN_OBJECT_DYN_PROPERTY_CROSSING_RIGHT 4U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN
#define Envm_GEN_OBJECT_DYN_PROPERTY_UNKNOWN 5U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED
#define Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED 6U
#endif
#ifndef Envm_GEN_OBJECT_DYN_PROPERTY_MAX_DIFF_TYPES
#define Envm_GEN_OBJECT_DYN_PROPERTY_MAX_DIFF_TYPES 7U
#endif
#ifndef eTurnIndicator_Off
#define eTurnIndicator_Off 0U
#endif
#ifndef eTurnIndicator_Left
#define eTurnIndicator_Left 1U
#endif
#ifndef eTurnIndicator_Right
#define eTurnIndicator_Right 2U
#endif
#ifndef eTurnIndicator_Both
#define eTurnIndicator_Both 3U
#endif
#ifndef eTurnIndicator_Invalid
#define eTurnIndicator_Invalid 4U
#endif

#ifndef Envm_GEN_OBJECT_OCCL_NONE
#define Envm_GEN_OBJECT_OCCL_NONE 0U
#endif
#ifndef Envm_GEN_OBJECT_OCCL_UNKNOWN
#define Envm_GEN_OBJECT_OCCL_UNKNOWN 1U
#endif
#ifndef Envm_GEN_OBJECT_OCCL_PARTLY
#define Envm_GEN_OBJECT_OCCL_PARTLY 2U
#endif
#ifndef Envm_GEN_OBJECT_OCCL_FULL
#define Envm_GEN_OBJECT_OCCL_FULL 3U
#endif
#ifndef Envm_GEN_OBJECT_OCCL_MAX_DIFF_TYPES
#define Envm_GEN_OBJECT_OCCL_MAX_DIFF_TYPES 4U
#endif

#ifndef PRED_AVLC_TRAJ_EGO_ONLY
#define PRED_AVLC_TRAJ_EGO_ONLY 0U
#endif
#ifndef PRED_AVLC_TRAJ_ROAD_ESTI
#define PRED_AVLC_TRAJ_ROAD_ESTI 1U
#endif
#ifndef PRED_AVLC_TRAJ_OBJ_TRACES
#define PRED_AVLC_TRAJ_OBJ_TRACES 2U
#endif

#ifndef Envm_GEN_OBJECT_CLASS_POINT
#define Envm_GEN_OBJECT_CLASS_POINT 0U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_CAR
#define Envm_GEN_OBJECT_CLASS_CAR 1U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_TRUCK
#define Envm_GEN_OBJECT_CLASS_TRUCK 2U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_PEDESTRIAN
#define Envm_GEN_OBJECT_CLASS_PEDESTRIAN 3U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_MOTORCYCLE
#define Envm_GEN_OBJECT_CLASS_MOTORCYCLE 4U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_BICYCLE
#define Envm_GEN_OBJECT_CLASS_BICYCLE 5U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_WIDE
#define Envm_GEN_OBJECT_CLASS_WIDE 6U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_UNCLASSIFIED
#define Envm_GEN_OBJECT_CLASS_UNCLASSIFIED 7U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_TL
#define Envm_GEN_OBJECT_CLASS_TL 8U
#endif
#ifndef Envm_GEN_OBJECT_CLASS_MAX_DIFF_TYPES
#define Envm_GEN_OBJECT_CLASS_MAX_DIFF_TYPES 9U
#endif

#ifndef FPS_EBA_HYP_CAT_NONE
#define FPS_EBA_HYP_CAT_NONE 0U
#endif
#ifndef FPS_EBA_HYP_CAT_PED
#define FPS_EBA_HYP_CAT_PED 1U
#endif
#ifndef FPS_EBA_HYP_CAT_VCL
#define FPS_EBA_HYP_CAT_VCL 2U
#endif
#ifndef FPS_EBA_HYP_CAT_XING
#define FPS_EBA_HYP_CAT_XING 4U
#endif
#ifndef FPS_EBA_HYP_CAT_ONC
#define FPS_EBA_HYP_CAT_ONC 8U
#endif
#ifndef FPS_EBA_HYP_CAT_STAT
#define FPS_EBA_HYP_CAT_STAT 16U
#endif
#ifndef FPS_EBA_HYP_CAT_CYCL
#define FPS_EBA_HYP_CAT_CYCL 32U
#endif
#ifndef FPS_EBA_HYP_CAT_ALL
#define FPS_EBA_HYP_CAT_ALL 255U
#endif

// RTE_SenAdpter
#ifndef Envm_GEN_OBJECT_SM_STATE_NONE
#define Envm_GEN_OBJECT_SM_STATE_NONE 0U
#endif

#ifndef VLC_MEAS_ID_VLC_SEN_AVLC_OOI
#define VLC_MEAS_ID_VLC_SEN_AVLC_OOI 539072256U
#endif

#ifndef VLC_MEAS_ID_CD_CUSTOM_OUTPUT
#define VLC_MEAS_ID_CD_CUSTOM_OUTPUT 539496448U
#endif

#ifndef VLC_MEAS_ID_CUSTOM_OUTPUT
#define VLC_MEAS_ID_CUSTOM_OUTPUT 539492352U
#endif

#ifndef VLC_MEAS_ID_SEN_ERROR_OUT_VADDR
#define VLC_MEAS_ID_SEN_ERROR_OUT_VADDR 539063552U
#endif

#ifndef VLC_MEAS_ID_SEN_HYPO_OUT
#define VLC_MEAS_ID_SEN_HYPO_OUT 539791616U
#endif

#ifndef VLC_MEAS_ID_SEN_INPUT_SIGHEADERS
#define VLC_MEAS_ID_SEN_INPUT_SIGHEADERS 539427840U
#endif

#ifndef VLC_MEAS_ID_SEN_FRAME_DATA
#define VLC_MEAS_ID_SEN_FRAME_DATA 539426816U
#endif

#ifndef VLC_MEAS_ID_TRAJECTORIES
#define VLC_MEAS_ID_TRAJECTORIES 539230208U
#endif

#ifndef VLC_MEAS_ID_PUBLIC_OBJECT_LIST
#define VLC_MEAS_ID_PUBLIC_OBJECT_LIST 539033600U
#endif

#ifndef VLC_MEAS_ID_AVLC_DISPLAY_OBJ
#define VLC_MEAS_ID_AVLC_DISPLAY_OBJ 539071232U
#endif

#ifndef VLC_MEAS_ID_AVLC_OUTPUT_DATA
#define VLC_MEAS_ID_AVLC_OUTPUT_DATA 539070976U
#endif

#ifndef VLC_SEN_ERROR_OUT_INTFVER
#define VLC_SEN_ERROR_OUT_INTFVER 15U
#endif

#ifndef VLC_SEN_INTFVER
#define VLC_SEN_INTFVER 20220826
#endif

#ifndef Acc_inhibition_none
#define Acc_inhibition_none 0U
#endif
#ifndef Acc_inhibition_blockage
#define Acc_inhibition_blockage 1U
#endif
#ifndef Acc_inhibition_alignment_init
#define Acc_inhibition_alignment_init 2U
#endif
#ifndef Acc_inhibition_alignment
#define Acc_inhibition_alignment 4U
#endif
#ifndef Acc_inhibition_partial_blockage
#define Acc_inhibition_partial_blockage 8U
#endif

#ifndef AVLC_DISPLAY_OBJECT_INTFVER
#define AVLC_DISPLAY_OBJECT_INTFVER 1U
#endif

#ifndef Obj_lane_left1
#define Obj_lane_left1 -1
#endif
#ifndef Obj_lane_same
#define Obj_lane_same 0
#endif
#ifndef Obj_lane_right1
#define Obj_lane_right1 1
#endif

#ifndef VLC_MEAS_ID_CGEB_CD_DATA
#define VLC_MEAS_ID_CGEB_CD_DATA 539140096U
#endif
#ifndef VLC_MEAS_ID_CGEB_CD_WRAP_DATA
#define VLC_MEAS_ID_CGEB_CD_WRAP_DATA 539295744U
#endif

#ifndef OBJ_USAGE_IDLE
#define OBJ_USAGE_IDLE 0U
#endif
#ifndef OBJ_USAGE_CONTROL
#define OBJ_USAGE_CONTROL 1U
#endif
#ifndef OBJ_USAGE_DISPLAY
#define OBJ_USAGE_DISPLAY 2U
#endif
#ifndef OBJ_USAGE_ALERT
#define OBJ_USAGE_ALERT 4U
#endif

#ifndef VLC_AVLC_LEVER_INTFVER
#define VLC_AVLC_LEVER_INTFVER 7U
#endif
#ifndef VLC_AVLC_OUTPUT_DATA_INTFVER
#define VLC_AVLC_OUTPUT_DATA_INTFVER 3U
#endif
#ifndef VLC_ASSESSED_OBJ_INTFVER
#define VLC_ASSESSED_OBJ_INTFVER 13U
#endif

#ifndef CDHypothesisType_No
#define CDHypothesisType_No 0U
#endif
#ifndef CDHypothesisType_RunUp
#define CDHypothesisType_RunUp 1U
#endif
#ifndef CDHypothesisType_RunUpBraking
#define CDHypothesisType_RunUpBraking 2U
#endif
#ifndef CDHypothesisType_RunUpStationary
#define CDHypothesisType_RunUpStationary 3U
#endif
#ifndef CDHypothesisType_Static
#define CDHypothesisType_Static 4U
#endif
#ifndef CDHypothesisType_ACC
#define CDHypothesisType_ACC 5U
#endif
#ifndef CDHypothesisType_Pass
#define CDHypothesisType_Pass 6U
#endif
#ifndef CDHypothesisType_CutIn
#define CDHypothesisType_CutIn 7U
#endif
#ifndef CDHypothesisType_Collision
#define CDHypothesisType_Collision 8U
#endif
#ifndef CDHypothesisType_CollisionUnavoidable
#define CDHypothesisType_CollisionUnavoidable 9U
#endif
#ifndef CDHypothesisType_PedCollision
#define CDHypothesisType_PedCollision 10U
#endif
#ifndef CDHypothesisType_PedPass
#define CDHypothesisType_PedPass 11U
#endif
#ifndef CDHypothesisType_Crossing
#define CDHypothesisType_Crossing 12U
#endif
#ifndef CDHypothesisType_CyclColl
#define CDHypothesisType_CyclColl 13U
#endif

#ifndef EBAObjectClass_NotAvail
#define EBAObjectClass_NotAvail 0U
#endif
#ifndef EBAObjectClass_Pedestrian
#define EBAObjectClass_Pedestrian 1U
#endif
#ifndef EBAObjectClass_Vehicle
#define EBAObjectClass_Vehicle 2U
#endif
#ifndef EBAObjectClass_Obstacle
#define EBAObjectClass_Obstacle 3U
#endif
#ifndef EBAObjectClass_Cyclist
#define EBAObjectClass_Cyclist 4U
#endif

#ifndef EBADynProp_NotAvail
#define EBADynProp_NotAvail 0U
#endif
#ifndef EBADynProp_Mov
#define EBADynProp_Mov 1U
#endif
#ifndef EBADynProp_Stat
#define EBADynProp_Stat 2U
#endif

#ifndef FPS_EBA_INH_NONE
#define FPS_EBA_INH_NONE 0U
#endif
#ifndef FPS_EBA_INH_LAT_WARN
#define FPS_EBA_INH_LAT_WARN 1U
#endif
#ifndef FPS_EBA_INH_PRE_WARN
#define FPS_EBA_INH_PRE_WARN 2U
#endif
#ifndef FPS_EBA_INH_ACU_WARN
#define FPS_EBA_INH_ACU_WARN 4U
#endif
#ifndef FPS_EBA_INH_PRE_FILL
#define FPS_EBA_INH_PRE_FILL 8U
#endif
#ifndef FPS_EBA_INH_HBA_THRD
#define FPS_EBA_INH_HBA_THRD 16U
#endif
#ifndef FPS_EBA_INH_HBA_TBRK
#define FPS_EBA_INH_HBA_TBRK 32U
#endif
#ifndef FPS_EBA_INH_PRECRASH
#define FPS_EBA_INH_PRECRASH 64U
#endif
#ifndef FPS_EBA_INH_BRAKE_L1
#define FPS_EBA_INH_BRAKE_L1 128U
#endif
#ifndef FPS_EBA_INH_BRAKE_L2
#define FPS_EBA_INH_BRAKE_L2 256U
#endif
#ifndef FPS_EBA_INH_BRAKE_L3
#define FPS_EBA_INH_BRAKE_L3 512U
#endif
#ifndef FPS_EBA_INH_ALL
#define FPS_EBA_INH_ALL 65535U
#endif

#ifndef VLC_MOD_STARTUP
#define VLC_MOD_STARTUP 0U
#endif
#ifndef VLC_MOD_INIT
#define VLC_MOD_INIT 1U
#endif
#ifndef VLC_MOD_RUNNING
#define VLC_MOD_RUNNING 2U
#endif
#ifndef VLC_MOD_SHUTDOWN
#define VLC_MOD_SHUTDOWN 3U
#endif
#ifndef VLC_MOD_PAUSE
#define VLC_MOD_PAUSE 4U
#endif

#ifndef Acc_sit_class_undefined
#define Acc_sit_class_undefined 0U
#endif
#ifndef Acc_sit_class_freemode
#define Acc_sit_class_freemode 1U
#endif
#ifndef Acc_sit_class_follow
#define Acc_sit_class_follow 2U
#endif
#ifndef Acc_sit_class_crawl
#define Acc_sit_class_crawl 3U
#endif
#ifndef Acc_sit_class_stop
#define Acc_sit_class_stop 4U
#endif
#ifndef Acc_sit_class_go
#define Acc_sit_class_go 5U
#endif
#ifndef Acc_sit_class_overtake
#define Acc_sit_class_overtake 6U
#endif

#ifndef VLC_SEN_AVLC_OOI_INTFVER
#define VLC_SEN_AVLC_OOI_INTFVER 4U
#endif

#ifndef VLC_HYPOTHESIS_INTFVER
#define VLC_HYPOTHESIS_INTFVER 20U
#endif

#ifndef VLC_CD_OUTPUT_CUSTOM_INTFVER
#define VLC_CD_OUTPUT_CUSTOM_INTFVER 20U
#endif

#ifndef VLC_CUSTOM_OUTPUT_INTFVER
#define VLC_CUSTOM_OUTPUT_INTFVER 26U
#endif

// RTE_Veh_Adapter_type
#ifndef SEATBELT_DRIVER_OPEN
#define SEATBELT_DRIVER_OPEN 0U
#endif
#ifndef SEATBELT_DRIVER_CLOSED
#define SEATBELT_DRIVER_CLOSED 1U
#endif

#ifndef VED_SOUT_POS_CURVE
#define VED_SOUT_POS_CURVE 0U
#endif
#ifndef VED_SOUT_POS_YWR
#define VED_SOUT_POS_YWR 1U
#endif
#ifndef VED_SOUT_POS_DRCRV
#define VED_SOUT_POS_DRCRV 2U
#endif
#ifndef VED_SOUT_POS_VEL
#define VED_SOUT_POS_VEL 3U
#endif
#ifndef VED_SOUT_POS_ACCEL
#define VED_SOUT_POS_ACCEL 4U
#endif
#ifndef VED_SOUT_POS_MSTAT
#define VED_SOUT_POS_MSTAT 5U
#endif
#ifndef VED_SOUT_POS_VCORR
#define VED_SOUT_POS_VCORR 6U
#endif
#ifndef VED_SOUT_POS_SSA
#define VED_SOUT_POS_SSA 7U
#endif
#ifndef VED_SOUT_POS_LATACC
#define VED_SOUT_POS_LATACC 8U
#endif
#ifndef VED_SOUT_POS_MAX
#define VED_SOUT_POS_MAX 12U
#endif

// Algo_const
#define FUN_PRESEL_AVLC_DROP_QUAL (0u)
#define FUN_PRESEL_AVLC_STAT_OBSTACLE (30u)
#define FUN_PRESEL_AVLC_MIN_KEEP_OBJ_QUAL (40u)
#define FUN_PRESEL_AVLC_MIN_OBJ_QUAL (70u)
#define FUN_PRESEL_AVLC_HIGH_CLUST_VAR_OBJ_QUAL (75u)
#define FUN_PRESEL_AVLC_LOW_CLUST_VAR_OBJ_QUAL (76u)

#define FUN_PRESEL_AVLC_OCCLUSION_OBJ_QUAL (80u)
#define FUN_PRESEL_AVLC_MIN_INLANE_OBJ_QUAL (85u)

// RTE_Algn_Type
#ifndef AL_ERR_STATE_UNKNOWN
#define AL_ERR_STATE_UNKNOWN 0U
#endif
#ifndef AL_ERR_STATE_ACTIVE
#define AL_ERR_STATE_ACTIVE 1U
#endif
#ifndef AL_ERR_STATE_INACTIVE
#define AL_ERR_STATE_INACTIVE 2U
#endif

#ifndef VED_IO_STATE_VALID
#define VED_IO_STATE_VALID 0U
#endif
#ifndef VED_IO_STATE_INVALID
#define VED_IO_STATE_INVALID 1U
#endif
#ifndef VED_IO_STATE_NOTAVAIL
#define VED_IO_STATE_NOTAVAIL 2U
#endif
#ifndef VED_IO_STATE_DECREASED
#define VED_IO_STATE_DECREASED 3U
#endif
#ifndef VED_IO_STATE_SUBSTITUE
#define VED_IO_STATE_SUBSTITUE 4U
#endif
#ifndef VED_IO_STATE_INPLAUSIBLE
#define VED_IO_STATE_INPLAUSIBLE 5U
#endif
#ifndef VED_IO_STATE_INIT
#define VED_IO_STATE_INIT 15U
#endif
#ifndef VED_IO_STATE_MAX
#define VED_IO_STATE_MAX 255U
#endif

#ifndef FN_AP_EBA_COUNTRY_0
#define FN_AP_EBA_COUNTRY_0 0U
#endif
#ifndef FN_AP_AVLC_COUNTRY_0
#define FN_AP_AVLC_COUNTRY_0 0U
#endif
#ifndef FN_AP_EBA_COUNTRY_1
#define FN_AP_EBA_COUNTRY_1 1U
#endif
#ifndef FN_AP_EBA_COUNTRY_2
#define FN_AP_EBA_COUNTRY_2 2U
#endif
#ifndef FN_AP_EBA_COUNTRY_3
#define FN_AP_EBA_COUNTRY_3 3U
#endif
#ifndef FN_AP_PCS_COUNTRY_MASK
#define FN_AP_PCS_COUNTRY_MASK 3U
#endif
#ifndef FN_AP_EBA_COUNTRY_MASK
#define FN_AP_EBA_COUNTRY_MASK 15U
#endif
#ifndef FN_AP_AVLC_COUNTRY_1
#define FN_AP_AVLC_COUNTRY_1 16U
#endif
#ifndef FN_AP_AVLC_COUNTRY_2
#define FN_AP_AVLC_COUNTRY_2 32U
#endif
#ifndef FN_AP_AVLC_COUNTRY_3
#define FN_AP_AVLC_COUNTRY_3 48U
#endif
#ifndef FN_AP_AVLC_COUNTRY_4
#define FN_AP_AVLC_COUNTRY_4 64U
#endif
#ifndef FN_AP_AVLC_COUNTRY_MASK
#define FN_AP_AVLC_COUNTRY_MASK 240U
#endif

// RTE_BSW_TYPE
#ifndef Display_op_none
#define Display_op_none 0U
#endif
#ifndef Display_op_cc_none
#define Display_op_cc_none 1U
#endif
#ifndef Display_op_cc_valid
#define Display_op_cc_valid 2U
#endif
#ifndef Display_op_cc_active
#define Display_op_cc_active 3U
#endif
#ifndef Display_op_cc_invalid
#define Display_op_cc_invalid 4U
#endif
#ifndef Display_op_cc_disengage
#define Display_op_cc_disengage 5U
#endif
#ifndef Display_op_cc_override
#define Display_op_cc_override 6U
#endif
#ifndef Display_op_cc_recom_speed
#define Display_op_cc_recom_speed 7U
#endif
#ifndef Display_op_lim_none
#define Display_op_lim_none 8U
#endif
#ifndef Display_op_lim_valid
#define Display_op_lim_valid 9U
#endif
#ifndef Display_op_lim_active
#define Display_op_lim_active 10U
#endif
#ifndef Display_op_lim_invalid
#define Display_op_lim_invalid 11U
#endif
#ifndef Display_op_lim_disengage
#define Display_op_lim_disengage 12U
#endif
#ifndef Display_op_plim
#define Display_op_plim 13U
#endif

// TM_Algo_Type
#define IS_SIGNAL_STATUS_OK(status) (status == VED_IO_STATE_VALID)

// vlc_par
/*! @brief Maximale VLC Reichweite (der Antenne) */
#define RW_VLC_MAX (200.F) /* ARS2xx: (150.F) */

/*******************************/
/*! vehicle related parameters */
/*******************************/
/*! default value for vehicle width */
//#define TRACKWIDTHFRONT_DEFAULT 2.00f
/*! default value for sensor frontoverhang (dist. from front axle to sensor) */
//#define FRONTOVERHANG_DEFAULT 0.80f
/*! default value for wheelbase (dist. between front/rear axle) */
#define VLC_WHEELBASE_DEFAULT 2.85f
/*! default value for axle load distribution */
//#define AXLELOADDISTRIBUTION_DEFAULT 0.5f

/*! The stopped confidence threshold for ACC function */
#define VLC_AVLC_PAR_OBJ_STOPPED_MIN_CONF 75u

/*! Delay time after all preconditions of sensor power reduction are satisfied
until power reduction is actually entered. Note: a setting of zero disables
delay
timing, meaning if sensor power reduction conditions are satisifed, power is
reduced without delay @unit:ms */
#define VLC_PAR_SENSOR_POWER_REDUCTION_DELAY_MS 0

#ifdef __cplusplus
}
#endif
#endif
