#pragma once

#ifndef VLCVEH_COMMON_UTILS_H
#define VLCVEH_COMMON_UTILS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "tue_common_libs.h"

#ifndef MAX_NUM_OF_HYPS
#define MAX_NUM_OF_HYPS 6
#endif
//====define

#ifndef MEASFreezeDataDLG
#define MEASFreezeDataDLG(x, y, z) NULL_FUNC()
#endif

// vlc_veh.h
#define VLC_FREEZE_DATA(pInfo, pData, Callback) \
    MEASFreezeDataDLG((pInfo), (pData),         \
                      (Callback)) /*!<macro to internal measfreeze procedure*/

#define GET_EBA_BUS_DEBUG_DATA                                              \
    VLCVEH_pEbaBusDebugData /*!<alias for pointer to EBABusDebugData OUTPUT \
                               interface*/
#define GET_LKA_BUS_DEBUG_DATA                                              \
    VLCVEH_pLkaBusDebugData /*!<alias for pointer to LKABusDebugData OUTPUT \
                               interface*/

#define GET_EGO_RAW_DATA_PTR                                               \
    VLCVEH_pEgoDynRaw /*!<alias for pointer to raw ego dynamics data INPUT \
                         interface*/
#define GET_EGO_STATIC_DATA_PTR                                           \
    VLCVEH_pGlobEgoStatic /*!<alias for pointer to ego vehicle parameters \
                             INPUT interface*/

#define GET_VLC_HEAD_CUST_OUT_DATA_PTR                               \
    VLC_pHEADCustDataOut /*!<alias for pointer to HEAD custom OUTPUT \
                            interface*/
#define GET_VLC_HEAD_GENERIC_OUT_DATA_PTR                                \
    VLC_pHEADGenericDataOut /*!<alias for pointer to HEAD generic OUTPUT \
                               interface*/
#define GET_VLC_HEAD_GENERIC_IN_DATA_PTR                               \
    VLC_pHEADGenericDataIn /*!<alias for pointer to HEAD generic INPUT \
                              interface*/
#define GET_VLC_HEAD_CUST_IN_DATA_PTR \
    VLC_pHEADCustDataIn /*!<alias for pointer to HEAD custom INPUT interface*/

#define GET_VLC_UDW_GENERIC_IN_DATA_PTR                              \
    VLC_pUDWGenericDataIn /*!<alias for pointer to UDW generic INPUT \
                             interface*/
#define GET_VLC_UDW_GENERIC_OUTPUT_DATA_PTR                            \
    VLC_pUDWGenericDataOut /*!<alias for pointer to UDW generic OUTPUT \
                              interface*/
#define GET_VLC_UDW_LANE_INPUT_DATA_PTR \
    VLC_pUDWLaneInput /*!<alias for pointer to UDW Lane INPUT interface*/
#define GET_VLC_UDW_CB_INPUT_DATA_PTR                                 \
    VLC_pUDWBlockageInput /*!<alias for pointer to UDW blockage INPUT \
                             interface*/
#define GET_VLC_UDW_ALDW_OUTPUT_DATA_PTR \
    VLC_pUDWALDWOutput /*!<alias for pointer to UDW ALDW INPUT interface*/

#define GET_VLC_DIM_CUST_OUT_DATA_PTR \
    VLC_pDIMCustDataOut /*!<alias for pointer to DIM custom OUTPUT interface*/
#define GET_VLC_DIM_CUST_IN_DATA_PTR \
    VLC_pDIMCustDataIn /*!<alias for pointer to DIM custom INPUT interface*/
#define GET_VLC_DIM_GENERIC_IN_DATA_PTR                              \
    VLC_pDIMGenericDataIn /*!<alias for pointer to DIM generic INPUT \
                             interface*/
#define GET_VLC_HMI_DATA_PTR \
    VLC_pHMIData /*!<alias for pointer to HMI INPUT interface*/
#define GET_VLC_LKA_LANE_INPUT_DATA_PTR \
    VLC_pLKALaneInput /*!<alias for pointer to LKA lane INPUT interface*/
#define GET_VLC_LKA_CB_INPUT_DATA_PTR                                 \
    VLC_pLKABlockageInput /*!<alias for pointer to LKA blockage INPUT \
                             interface*/
#define GET_VLC_LKA_INPUT_GENERIC_DATA_PTR                          \
    VLC_pLKAInputGeneric /*!<alias for pointer to LKA generic INPUT \
                            interface*/
#define GET_VLC_LKA_CODABLE_PARAM_PTR                                          \
    VLC_pLKACodableParam /*!<alias for pointer to LKA codable parameters INPUT \
                            interface*/
#define GET_VLC_LKA_OUTPUT_DATA_PTR                                   \
    VLC_pLKAOutputGeneric /*!<alias for pointer to LKA generic OUTPUT \
                             interface*/

#define VLC_BSW_ALGO_PARAM_PTR                                             \
    VLCVEH_pBswAlgoParameters /*!<alias for pointer to BSW algo parameters \
                                 interface*/

#define VLC_CPAR_VLC_PARAM_PTR                                                \
    VLCVEH_pCPAR_VLC_Parameters /*!<alias for pointer to EBA CParameter INPUT \
                                   interface*/
#define VLC_CPAR_VLC_LKS_PARAM_PTR \
    VLCVEH_pCPAR_VLC_LKS_Parameters /*!<VLCVEH_pCPAR_VLC_LKS_Parameters*/

/* component defines */
#define VLC_VEH_CYCLE_TIME \
    (0.020F) /*!<alias to VLCVeh calling cycle time in s (0.02f)*/

#define OBJ_ABS_VELO_X(iObj)                                                  \
    VLCVEH_pCustomOutput->CustObjData[iObj]                                   \
        .AbsolutKinematics.fAbsVelocityX /*!<alias for accessing longitudinal \
                                            absolute ego velocity*/
#define OBJ_ABS_ACCEL_X(iObj)                                             \
    VLCVEH_pCustomOutput->CustObjData[iObj]                               \
        .AbsolutKinematics.fAbsAccelerationX /*!<alias for accessing      \
                                                longitudinal absolute ego \
                                                acceleration*/

/* task monitoring checkpoints */
#define VLC_CHECKPOINT_PROCESS_INPUT /* add one */           /*!<not used*/
#define VLC_CHECKPOINT_PROCESS_OUTPUT /* add one */          /*!<not used*/
#define VLC_EVALUATE_CHECKPOINT /* TPEvaluateCheckpoint() */ /*!<not used*/

// RTE_SenAdpter
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

#ifndef VLC_LONG_CTRL_OUTPUT_INTFVER
#define VLC_LONG_CTRL_OUTPUT_INTFVER 13U
#endif
#ifndef VLC_VEH_OUT_ARBIT_INTFVER
#define VLC_VEH_OUT_ARBIT_INTFVER 6U
#endif
#ifndef VLC_DIM_OUTPUT_CUSTOM_INTFVER
#define VLC_DIM_OUTPUT_CUSTOM_INTFVER 6U
#endif
#ifndef VLC_SAD_OUTPUT_GENERIC_INTFVER
#define VLC_SAD_OUTPUT_GENERIC_INTFVER 2U
#endif

#ifndef VLC_VEH_ERROR_OUT_INTFVER
#define VLC_VEH_ERROR_OUT_INTFVER 2U
#endif

#ifndef VLC_VEH_INTFVER
#define VLC_VEH_INTFVER 20220720
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

#ifndef VLC_MEAS_ID_CGEB_DIM_DATA
#define VLC_MEAS_ID_CGEB_DIM_DATA 539142144U
#endif

#ifndef eDriverFeedbackState_Negative
#define eDriverFeedbackState_Negative 0U
#endif
#ifndef eDriverFeedbackState_WeakNegative
#define eDriverFeedbackState_WeakNegative 1U
#endif
#ifndef eDriverFeedbackState_NoNegative
#define eDriverFeedbackState_NoNegative 2U
#endif
#ifndef eDriverFeedbackState_NoPositive
#define eDriverFeedbackState_NoPositive 3U
#endif
#ifndef eDriverFeedbackState_WeakPositive
#define eDriverFeedbackState_WeakPositive 4U
#endif
#ifndef eDriverFeedbackState_Positive
#define eDriverFeedbackState_Positive 5U
#endif
#ifndef eDriverFeedbackState_StrongPositive
#define eDriverFeedbackState_StrongPositive 6U
#endif

#ifndef eDriverAttentionState_Unknown
#define eDriverAttentionState_Unknown 0U
#endif
#ifndef eDriverAttentionState_Low
#define eDriverAttentionState_Low 1U
#endif
#ifndef eDriverAttentionState_High
#define eDriverAttentionState_High 2U
#endif
#ifndef eDriverAttentionState_Higher
#define eDriverAttentionState_Higher 3U
#endif
#ifndef eDriverAttentionState_VeryHigh
#define eDriverAttentionState_VeryHigh 4U
#endif
#ifndef eDriverAttentionState_Unattended
#define eDriverAttentionState_Unattended 5U
#endif

#ifndef eDriverActivity_Inactive
#define eDriverActivity_Inactive 0U
#endif
#ifndef eDriverActivity_MissingConf
#define eDriverActivity_MissingConf 1U
#endif
#ifndef eDriverActivity_VeryActive
#define eDriverActivity_VeryActive 2U
#endif
#ifndef eDriverActivity_EmergencySteer
#define eDriverActivity_EmergencySteer 3U
#endif

#ifndef eDimMonState_NotPossible
#define eDimMonState_NotPossible 0U
#endif
#ifndef eDimMonState_Limited
#define eDimMonState_Limited 1U
#endif
#ifndef eDimMonState_Unlimited
#define eDimMonState_Unlimited 2U
#endif

#ifndef VLC_SAD_OUTPUT_CUSTOM_INTFVER
#define VLC_SAD_OUTPUT_CUSTOM_INTFVER 39U
#endif

#ifndef VLC_MEAS_ID_CGEB_SAD_GSM_DATA
#define VLC_MEAS_ID_CGEB_SAD_GSM_DATA 539500544U
#endif

#ifndef VLC_MEAS_ID_CGEB_SAD_COND_TIMER_DATA
#define VLC_MEAS_ID_CGEB_SAD_COND_TIMER_DATA 539504128U
#endif

#ifndef VLC_MEAS_ID_CGEB_SAD_OUTFUNC_DATA
#define VLC_MEAS_ID_CGEB_SAD_OUTFUNC_DATA 539144192U
#endif

#ifndef VLC_MEAS_ID_CGEB_SAD_CUSTOM_DATA
#define VLC_MEAS_ID_CGEB_SAD_CUSTOM_DATA 539501568U
#endif

#ifndef VLC_MEAS_ID_CGEB_SAD_DATA
#define VLC_MEAS_ID_CGEB_SAD_DATA 539143168U
#endif

#ifndef VLC_MEAS_ID_CGEB_SAD_COMMON_OUT
#define VLC_MEAS_ID_CGEB_SAD_COMMON_OUT 539502080U
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

// RTE_BSW_TYPE
#ifndef Cc_no_error
#define Cc_no_error 0U
#endif

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

// RTE_Veh_Adapter_type
#ifndef AVLC_LEVER_NO_ACCEL
#define AVLC_LEVER_NO_ACCEL 0U
#endif
#ifndef AVLC_LEVER_ACCEL_LOW
#define AVLC_LEVER_ACCEL_LOW 1U
#endif
#ifndef AVLC_LEVER_ACCEL_HIGH
#define AVLC_LEVER_ACCEL_HIGH 2U
#endif
#ifndef AVLC_LEVER_NO_DECEL
#define AVLC_LEVER_NO_DECEL 0U
#endif
#ifndef AVLC_LEVER_DECEL_LOW
#define AVLC_LEVER_DECEL_LOW 1U
#endif
#ifndef AVLC_LEVER_DECEL_HIGH
#define AVLC_LEVER_DECEL_HIGH 2U
#endif

#ifndef DAS_FAILURE_NONE
#define DAS_FAILURE_NONE 0U
#endif
#ifndef DAS_FAILURE_TMP_NOT_AVAIL
#define DAS_FAILURE_TMP_NOT_AVAIL 1U
#endif
#ifndef DAS_FAILURE_PERF_DEGR
#define DAS_FAILURE_PERF_DEGR 2U
#endif
#ifndef DAS_FAILURE_SRV_REQUESTED
#define DAS_FAILURE_SRV_REQUESTED 3U
#endif

#ifndef DM_STATUS_OFF
#define DM_STATUS_OFF 0U
#endif
#ifndef DM_STATUS_ON_INACTIVE
#define DM_STATUS_ON_INACTIVE 1U
#endif
#ifndef DM_STATUS_ON_ACTIVE
#define DM_STATUS_ON_ACTIVE 2U
#endif

#ifndef DAS_STATUS_OFF
#define DAS_STATUS_OFF 0U
#endif
#ifndef DAS_STATUS_PASSIVE
#define DAS_STATUS_PASSIVE 7U
#endif
#ifndef DAS_STATUS_STANDBY
#define DAS_STATUS_STANDBY 1U
#endif
#ifndef DAS_STATUS_ACTIVE
#define DAS_STATUS_ACTIVE 2U
#endif
#ifndef DAS_STATUS_OVERRIDE
#define DAS_STATUS_OVERRIDE 3U
#endif
#ifndef DAS_STATUS_STAND_ACTIVE
#define DAS_STATUS_STAND_ACTIVE 4U
#endif
#ifndef DAS_STATUS_STAND_WAIT
#define DAS_STATUS_STAND_WAIT 5U
#endif
#ifndef DAS_STATUS_FAULT
#define DAS_STATUS_FAULT 6U
#endif

#ifndef AEB_STATUS_SELFCHECK
#define AEB_STATUS_SELFCHECK 0U
#endif
#ifndef AEB_STATUS_OFF
#define AEB_STATUS_OFF 1U
#endif
#ifndef AEB_STATUS_STANDBY
#define AEB_STATUS_STANDBY 2U
#endif
#ifndef AEB_STATUS_ACTIVE
#define AEB_STATUS_ACTIVE 3U
#endif
#ifndef AEB_STATUS_FAULT
#define AEB_STATUS_FAULT 4U
#endif

#ifndef FCW_STATUS_SELFCHECK
#define FCW_STATUS_SELFCHECK 0U
#endif
#ifndef FCW_STATUS_OFF
#define FCW_STATUS_OFF 1U
#endif
#ifndef FCW_STATUS_STANDBY
#define FCW_STATUS_STANDBY 2U
#endif
#ifndef FCW_STATUS_ACTIVE
#define FCW_STATUS_ACTIVE 3U
#endif
#ifndef FCW_STATUS_FAULT
#define FCW_STATUS_FAULT 4U
#endif

#ifndef AVLC_OFF
#define AVLC_OFF 0U
#endif
#ifndef AVLC_STANDBY
#define AVLC_STANDBY 1U
#endif
#ifndef AVLC_ENGAGED
#define AVLC_ENGAGED 2U
#endif
#ifndef AVLC_OVERRIDE
#define AVLC_OVERRIDE 3U
#endif
#ifndef AVLC_DISENGAGE
#define AVLC_DISENGAGE 4U
#endif
#ifndef AVLC_STANDSTILL
#define AVLC_STANDSTILL 5U
#endif
#ifndef AVLC_FAILURE
#define AVLC_FAILURE 6U
#endif

#ifndef Cc_no_error
#define Cc_no_error 0U
#endif
#ifndef Cc_performance_degradation
#define Cc_performance_degradation 1U
#endif
#ifndef Cc_temp_unavailable
#define Cc_temp_unavailable 2U
#endif
#ifndef Cc_error_service
#define Cc_error_service 3U
#endif

#ifndef DC_STATUS_AVAILABLE
#define DC_STATUS_AVAILABLE 0U
#endif
#ifndef DC_STATUS_TMP_NOT_AVAIL
#define DC_STATUS_TMP_NOT_AVAIL 1U
#endif
#ifndef DC_STATUS_NOT_AVAIL
#define DC_STATUS_NOT_AVAIL 2U
#endif

#ifndef SPD_UNIT_KMH
#define SPD_UNIT_KMH 0U
#endif
#ifndef SPD_UNIT_MPH
#define SPD_UNIT_MPH 1U
#endif

#ifndef eThrtLvl_No
#define eThrtLvl_No 0U
#endif
#ifndef eThrtLvl_Low
#define eThrtLvl_Low 1U
#endif
#ifndef eThrtLvl_Mid
#define eThrtLvl_Mid 2U
#endif
#ifndef eThrtLvl_High
#define eThrtLvl_High 3U
#endif

#ifndef eAvailableOff
#define eAvailableOff 0U
#endif
#ifndef eAvailableOn
#define eAvailableOn 1U
#endif
#ifndef eUnavailableError
#define eUnavailableError 2U
#endif
#ifndef eUnavailableCoded
#define eUnavailableCoded 3U
#endif

#ifndef eDriverSetting_Early
#define eDriverSetting_Early 0U
#endif
#ifndef eDriverSetting_Middle
#define eDriverSetting_Middle 1U
#endif
#ifndef eDriverSetting_Late
#define eDriverSetting_Late 2U
#endif
#ifndef eDriverSetting_Invalid
#define eDriverSetting_Invalid 3U
#endif

#ifndef SEATBELT_DRIVER_CLOSED
#define SEATBELT_DRIVER_CLOSED 0U
#endif
#ifndef SEATBELT_DRIVER_OPEN
#define SEATBELT_DRIVER_OPEN 1U
#endif

#ifndef DOOR_DRIVER_CLOSED
#define DOOR_DRIVER_CLOSED 0U
#endif
#ifndef DOOR_DRIVER_OPEN
#define DOOR_DRIVER_OPEN 1U
#endif

#ifndef HOOD_DRIVER_CLOSED
#define HOOD_DRIVER_CLOSED 0U
#endif
#ifndef HOOD_DRIVER_OPEN
#define HOOD_DRIVER_OPEN 1U
#endif

#ifndef TRUNK_DRIVER_CLOSED
#define TRUNK_DRIVER_CLOSED 0U
#endif
#ifndef TRUNK_DRIVER_OPEN
#define TRUNK_DRIVER_OPEN 1U
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

#ifndef eEBAFctChan_Unknown
#define eEBAFctChan_Unknown 0U
#endif
#ifndef eEBAFctChan_Vehicle
#define eEBAFctChan_Vehicle 1U
#endif
#ifndef eEBAFctChan_Pedestrian
#define eEBAFctChan_Pedestrian 2U
#endif
#ifndef eEBAFctChan_Unclassified
#define eEBAFctChan_Unclassified 4U
#endif
#ifndef eEBAFctChan_Crossing
#define eEBAFctChan_Crossing 8U
#endif
#ifndef eEBAFctChan_Cyclist
#define eEBAFctChan_Cyclist 16U
#endif

#ifndef eEBAOn_Inactive
#define eEBAOn_Inactive 0U
#endif
#ifndef eEBAOn_Moving
#define eEBAOn_Moving 1U
#endif
#ifndef eEBAOn_Standing
#define eEBAOn_Standing 2U
#endif
#ifndef eEBAOn_Halted
#define eEBAOn_Halted 3U
#endif
#ifndef eEBAOn_CrossFrLeft
#define eEBAOn_CrossFrLeft 4U
#endif
#ifndef eEBAOn_CrossFrRight
#define eEBAOn_CrossFrRight 5U
#endif

#ifndef eEBAGenerator_No
#define eEBAGenerator_No 0U
#endif
#ifndef eEBAGenerator_KeepVoltage
#define eEBAGenerator_KeepVoltage 1U
#endif
#ifndef eEBAGenerator_BoostVoltage
#define eEBAGenerator_BoostVoltage 2U
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

#ifndef eEBAHaptSensNone
#define eEBAHaptSensNone 0U
#endif
#ifndef eEBAHaptSensLow
#define eEBAHaptSensLow 1U
#endif
#ifndef eEBAHaptSensMid
#define eEBAHaptSensMid 2U
#endif
#ifndef eEBAHaptSensHigh
#define eEBAHaptSensHigh 3U
#endif

#ifndef eObjectSwitch_Pedestrians
#define eObjectSwitch_Pedestrians 1U
#endif
#ifndef eObjectSwitch_Vehicles
#define eObjectSwitch_Vehicles 2U
#endif
#ifndef eObjectSwitch_Obstacle
#define eObjectSwitch_Obstacle 4U
#endif
#ifndef eObjectSwitch_Cyclist
#define eObjectSwitch_Cyclist 8U
#endif
#ifndef eObjectSwitch_MaxValue
#define eObjectSwitch_MaxValue 4294967295U
#endif

#ifndef eMainSwitch_Active
#define eMainSwitch_Active 0U
#endif
#ifndef eMainSwitch_Inactive
#define eMainSwitch_Inactive 1U
#endif
#ifndef eMainSwitch_NotUsed
#define eMainSwitch_NotUsed 2U
#endif
#ifndef eMainSwitch_Invalid
#define eMainSwitch_Invalid 3U
#endif

#ifndef eFunctionSwitch_PedWarn
#define eFunctionSwitch_PedWarn 1U
#endif
#ifndef eFunctionSwitch_PedPreFill
#define eFunctionSwitch_PedPreFill 2U
#endif
#ifndef eFunctionSwitch_PedHBA
#define eFunctionSwitch_PedHBA 4U
#endif
#ifndef eFunctionSwitch_PedBrakeL1
#define eFunctionSwitch_PedBrakeL1 8U
#endif
#ifndef eFunctionSwitch_PedBrakeL2
#define eFunctionSwitch_PedBrakeL2 16U
#endif
#ifndef eFunctionSwitch_PedBrakeL3
#define eFunctionSwitch_PedBrakeL3 32U
#endif
#ifndef eFunctionSwitch_PedPreCrash
#define eFunctionSwitch_PedPreCrash 64U
#endif
#ifndef eFunctionSwitch_VehWarn
#define eFunctionSwitch_VehWarn 128U
#endif
#ifndef eFunctionSwitch_VehLatentWarn
#define eFunctionSwitch_VehLatentWarn 256U
#endif
#ifndef eFunctionSwitch_VehPreFill
#define eFunctionSwitch_VehPreFill 512U
#endif
#ifndef eFunctionSwitch_VehHBA
#define eFunctionSwitch_VehHBA 1024U
#endif
#ifndef eFunctionSwitch_VehBrakeL1
#define eFunctionSwitch_VehBrakeL1 2048U
#endif
#ifndef eFunctionSwitch_VehBrakeL2
#define eFunctionSwitch_VehBrakeL2 4096U
#endif
#ifndef eFunctionSwitch_VehBrakeL3
#define eFunctionSwitch_VehBrakeL3 8192U
#endif
#ifndef eFunctionSwitch_VehPreCrash
#define eFunctionSwitch_VehPreCrash 16384U
#endif
#ifndef eFunctionSwitch_ObjWarn
#define eFunctionSwitch_ObjWarn 32768U
#endif
#ifndef eFunctionSwitch_ObjLatentWarn
#define eFunctionSwitch_ObjLatentWarn 65536U
#endif
#ifndef eFunctionSwitch_ObjPreFill
#define eFunctionSwitch_ObjPreFill 131072U
#endif
#ifndef eFunctionSwitch_ObjHBA
#define eFunctionSwitch_ObjHBA 262144U
#endif
#ifndef eFunctionSwitch_ObjBrakeL1
#define eFunctionSwitch_ObjBrakeL1 524288U
#endif
#ifndef eFunctionSwitch_ObjBrakeL2
#define eFunctionSwitch_ObjBrakeL2 1048576U
#endif
#ifndef eFunctionSwitch_ObjBrakeL3
#define eFunctionSwitch_ObjBrakeL3 2097152U
#endif
#ifndef eFunctionSwitch_ObjPreCrash
#define eFunctionSwitch_ObjPreCrash 4194304U
#endif
#ifndef eFunctionSwitch_CyclWarn
#define eFunctionSwitch_CyclWarn 8388608U
#endif
#ifndef eFunctionSwitch_CyclPreFill
#define eFunctionSwitch_CyclPreFill 16777216U
#endif
#ifndef eFunctionSwitch_CyclHBA
#define eFunctionSwitch_CyclHBA 33554432U
#endif
#ifndef eFunctionSwitch_CyclBrakeL1
#define eFunctionSwitch_CyclBrakeL1 67108864U
#endif
#ifndef eFunctionSwitch_CyclBrakeL2
#define eFunctionSwitch_CyclBrakeL2 134217728U
#endif
#ifndef eFunctionSwitch_CyclBrakeL3
#define eFunctionSwitch_CyclBrakeL3 268435456U
#endif
#ifndef eFunctionSwitch_CyclPreCrash
#define eFunctionSwitch_CyclPreCrash 536870912U
#endif
#ifndef eFunctionSwitch_MaxValue
#define eFunctionSwitch_MaxValue 4294967295U
#endif

#ifndef EBA_CODING_GEN_LOW_SPEED
#define EBA_CODING_GEN_LOW_SPEED 1U
#endif
#ifndef EBA_CODING_GEN_COUNTRY_A
#define EBA_CODING_GEN_COUNTRY_A 2U
#endif
#ifndef EBA_CODING_GEN_COUNTRY_B
#define EBA_CODING_GEN_COUNTRY_B 4U
#endif
#ifndef EBA_CODING_GEN_IGNR_SFTY_CHCKS
#define EBA_CODING_GEN_IGNR_SFTY_CHCKS 8U
#endif
#ifndef EBA_CODING_GEN_DEBUGMESSAGES
#define EBA_CODING_GEN_DEBUGMESSAGES 16U
#endif
#ifndef EBA_CODING_GEN_DISTANCELIMIT
#define EBA_CODING_GEN_DISTANCELIMIT 32U
#endif
#ifndef EBA_CODING_GEN_IGNORE_ACT_SWITCH
#define EBA_CODING_GEN_IGNORE_ACT_SWITCH 128U
#endif
#ifndef EBA_CODING_GEN_MAX_VALUE
#define EBA_CODING_GEN_MAX_VALUE 4294967295U
#endif

#ifndef EBA_CODING_FMOD_STAT_WARN
#define EBA_CODING_FMOD_STAT_WARN 1U
#endif
#ifndef EBA_CODING_FMOD_STAT_BRAKE
#define EBA_CODING_FMOD_STAT_BRAKE 2U
#endif
#ifndef EBA_CODING_FMOD_STAT_PRECRASH
#define EBA_CODING_FMOD_STAT_PRECRASH 4U
#endif
#ifndef EBA_CODING_FMOD_STAT_PREFILL
#define EBA_CODING_FMOD_STAT_PREFILL 8U
#endif
#ifndef EBA_CODING_FMOD_STAT_HBA
#define EBA_CODING_FMOD_STAT_HBA 16U
#endif
#ifndef EBA_CODING_FMOD_OBJ_WARN
#define EBA_CODING_FMOD_OBJ_WARN 32U
#endif
#ifndef EBA_CODING_FMOD_OBJ_LATENT_WARN
#define EBA_CODING_FMOD_OBJ_LATENT_WARN 64U
#endif
#ifndef EBA_CODING_FMOD_OBJ_BRAKE
#define EBA_CODING_FMOD_OBJ_BRAKE 128U
#endif
#ifndef EBA_CODING_FMOD_OBJ_PRECRASH
#define EBA_CODING_FMOD_OBJ_PRECRASH 256U
#endif
#ifndef EBA_CODING_FMOD_OBJ_PREFILL
#define EBA_CODING_FMOD_OBJ_PREFILL 512U
#endif
#ifndef EBA_CODING_FMOD_OBJ_HBA
#define EBA_CODING_FMOD_OBJ_HBA 1024U
#endif
#ifndef EBA_CODING_FMOD_PED_WARN
#define EBA_CODING_FMOD_PED_WARN 2048U
#endif
#ifndef EBA_CODING_FMOD_PED_BRAKE
#define EBA_CODING_FMOD_PED_BRAKE 4096U
#endif
#ifndef EBA_CODING_FMOD_PED_PRECRASH
#define EBA_CODING_FMOD_PED_PRECRASH 8192U
#endif
#ifndef EBA_CODING_FMOD_PED_PREFILL
#define EBA_CODING_FMOD_PED_PREFILL 16384U
#endif
#ifndef EBA_CODING_FMOD_PED_HBA
#define EBA_CODING_FMOD_PED_HBA 32768U
#endif
#ifndef EBA_CODING_FMOD_VCL_WARN
#define EBA_CODING_FMOD_VCL_WARN 65536U
#endif
#ifndef EBA_CODING_FMOD_VCL_LATENT_WARN
#define EBA_CODING_FMOD_VCL_LATENT_WARN 131072U
#endif
#ifndef EBA_CODING_FMOD_VCL_BRAKE
#define EBA_CODING_FMOD_VCL_BRAKE 262144U
#endif
#ifndef EBA_CODING_FMOD_VCL_PRECRASH
#define EBA_CODING_FMOD_VCL_PRECRASH 524288U
#endif
#ifndef EBA_CODING_FMOD_VCL_PREFILL
#define EBA_CODING_FMOD_VCL_PREFILL 1048576U
#endif
#ifndef EBA_CODING_FMOD_VCL_HBA
#define EBA_CODING_FMOD_VCL_HBA 2097152U
#endif
#ifndef EBA_CODING_FMOD_XING_WARN
#define EBA_CODING_FMOD_XING_WARN 4194304U
#endif
#ifndef EBA_CODING_FMOD_XING_BRAKE
#define EBA_CODING_FMOD_XING_BRAKE 8388608U
#endif
#ifndef EBA_CODING_FMOD_XING_PRECRASH
#define EBA_CODING_FMOD_XING_PRECRASH 16777216U
#endif
#ifndef EBA_CODING_FMOD_XING_PREFILL
#define EBA_CODING_FMOD_XING_PREFILL 33554432U
#endif
#ifndef EBA_CODING_FMOD_XING_HBA
#define EBA_CODING_FMOD_XING_HBA 67108864U
#endif
#ifndef EBA_CODING_FMOD_CYCL_WARN
#define EBA_CODING_FMOD_CYCL_WARN 134217728U
#endif
#ifndef EBA_CODING_FMOD_CYCL_BRAKE
#define EBA_CODING_FMOD_CYCL_BRAKE 268435456U
#endif
#ifndef EBA_CODING_FMOD_CYCL_PRECRASH
#define EBA_CODING_FMOD_CYCL_PRECRASH 536870912U
#endif
#ifndef EBA_CODING_FMOD_CYCL_PREFILL
#define EBA_CODING_FMOD_CYCL_PREFILL 1073741824U
#endif
#ifndef EBA_CODING_FMOD_CYCL_HBA
#define EBA_CODING_FMOD_CYCL_HBA 2147483648U
#endif
#ifndef EBA_CODING_FMOD_MAX_VALUE
#define EBA_CODING_FMOD_MAX_VALUE 4294967295U
#endif

// RTE_DiagSrv
#ifndef DIMInputSignalState_OK
#define DIMInputSignalState_OK 0U
#endif
#ifndef DIMInputSignalState_Default
#define DIMInputSignalState_Default 1U
#endif
#ifndef DIMInputSignalState_Missing
#define DIMInputSignalState_Missing 2U
#endif
#ifndef DIMInputSignalState_BadQuality
#define DIMInputSignalState_BadQuality 3U
#endif
#ifndef DIMInputSignalState_Suspicious
#define DIMInputSignalState_Suspicious 4U
#endif
#ifndef DIMInputSignalState_Max
#define DIMInputSignalState_Max 5U
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

#ifndef eVLC_STATE_SIG_INACTIVE
#define eVLC_STATE_SIG_INACTIVE 0U
#endif
#ifndef eVLC_STATE_SIG_ACTIVE
#define eVLC_STATE_SIG_ACTIVE 1U
#endif
#ifndef eVLC_STATE_SIG_INVALID
#define eVLC_STATE_SIG_INVALID 2U
#endif

// Gobal_Comp_id
#define TASK_ID_ALGO_RAW_DATA_CYCLE 204U
#define TASK_ID_ALGO_DATA_PROC_CYCLE 205U
#define TASK_ID_ALGO_RHC_CYCLE 206U
#define TASK_ID_ALGO_VEH_CYCLE 207U
#define TASK_ID_ALGO_SEN_CYCLE 208U
#define TASK_ID_SW_EVERY_10MS 209U
#define TASK_ID_SW_EVERY_20MS 210U
#define TASK_ID_SW_RADAR_CYCLE 211U
#define TASK_ID_ALGO_PEAK_LIST 212U
#define TASK_ID_DSP 213U
#define TASK_ID_SW_EVERY_100MS 214U
#define TASK_ID_SW_EVERY_10S 215U
#define TASK_ID_SW_06 216U

// RTE_Algn_Type
#ifndef AL_SIG_STATE_INIT
#define AL_SIG_STATE_INIT 0U
#endif
#ifndef AL_SIG_STATE_OK
#define AL_SIG_STATE_OK 1U
#endif

#ifndef AL_SIG_STATE_INVALID
#define AL_SIG_STATE_INVALID 2U
#endif
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

#define IS_SIGNAL_STATUS_OK(status) (status == VED_IO_STATE_VALID)

#ifdef __cplusplus
}
#endif
#endif
