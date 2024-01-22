/* **********************************************************************
Autosar Memory Map
**************************************************************************** */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "veh_sim.h"
#include "cc.h"
#include "cc_par.h"
#include "mat_std_ext.h"
#include "mat_param_ext.h"

// #define DLMU1_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#define Cc_lat_accel_grad_filter (4)

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
#if VLC_LONG_VEH_DEBUG == 0
static LaneMarkInfo left_lane_mark;
static LaneMarkInfo right_lane_mark;
#endif
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/******************************************************************************
  @fn                   VLC_LIMIT_LATERAL_ACCEL */ /*!

                               @description     Limit the longitudinal
                             acceleration so that the lateral
                                               acceleration is acceptable for
                             the driven speed

                               @param[in]      cycle_time
                               @param[in]      input
                               @param[out]     acc_output
                               @param[in]      das_input
                               @param[out]     cc_status
                               @return         void

                             *****************************************************************************/
void VLC_LIMIT_LATERAL_ACCEL(const times_t cycle_time,
                             const cc_input_data_t* input,
                             const VLC_acc_output_data_t* acc_output,
                             const cart_das_input_data_t* das_input,
                             const t_CamLaneInputData* pCamLaneData,
                             cc_status_t* cc_status,
                             PACCInfo* p_pacc_info) {
    // velocity_t vehicle_speed;
    acceleration_t lat_accel;
    // acceleration_t lat_accel_max;
    times_t alat_filter_time;
    gradient_t lat_accel_grad;

    // vehicle_speed = das_input->VEHICLE_SPEED;
    lat_accel = (acceleration_t)MAT_ABS((sint32)input->LATERAL_ACCELERATION);

    lat_accel_grad = (((gradient_t)lat_accel -
                       (gradient_t)cc_status->VLC_LAT_ACCEL_LAST_CYCLE) /
                      (gradient_t)cycle_time);
    lat_accel_grad =
        MAT_FILT(lat_accel_grad, cc_status->VLC_LAT_ACCEL_GRAD_LAST_CYCLE,
                 (sint32)Cc_lat_accel_grad_filter);

    /* Filter LateralAcceleration */
    if (lat_accel_grad < (gradient_t)0) {
        /* coming out of a curve: slow filter */
        alat_filter_time = Cc_neg_lateral_accel_filter_time;
    } else {
        /* going into a curve, being in a curve: fast filter */
        alat_filter_time = Cc_pos_lateral_accel_filter_time;
    }

    lat_accel = (acceleration_t)MAT_FILT(
        (sint32)lat_accel, (sint32)cc_status->VLC_LAT_ACCEL_LAST_CYCLE,
        (sint32)alat_filter_time / cycle_time);

    /* Proposal 1
       V veh / A lat table -> A lat max
       (A lat max - A lat host) -> A long max
    */
    // lat_accel_max = MAT_CALCULATE_PARAM_VALUE1D(
    //     Cc_alat_speed_curve, (uint16)Cc_alat_speed_curve_points,
    //     (sint16)MAT_ABS((sint32)vehicle_speed));

    // left lane
    left_lane_mark.geometry.view_range =
        pCamLaneData->CourseInfo[0].CourseInfoSegNear.f_Length;
    left_lane_mark.geometry.lateral_deviation =
        pCamLaneData->LaneMarkerInfo[0].f_MarkerDist;
    if (fABS(pCamLaneData->CourseInfo[0].f_Angle) < 1.5) {
        left_lane_mark.geometry.heading_angle =
            pCamLaneData->CourseInfo[0].f_Angle;
    }
    if (fABS(pCamLaneData->CourseInfo[0].CourseInfoSegNear.f_C0) < 0.1) {
        left_lane_mark.geometry.curvature =
            pCamLaneData->CourseInfo[0].CourseInfoSegNear.f_C0;
    }
    if (fABS(pCamLaneData->CourseInfo[0].CourseInfoSegNear.f_C1) < 0.1) {
        left_lane_mark.geometry.curvature_rate =
            pCamLaneData->CourseInfo[0].CourseInfoSegNear.f_C1;
    }
    left_lane_mark.property.confidence =
        pCamLaneData->LaneMarkerInfo[0].u_ExistanceProbability;

    // right lane
    right_lane_mark.geometry.view_range =
        pCamLaneData->CourseInfo[1].CourseInfoSegNear.f_Length;
    right_lane_mark.geometry.lateral_deviation =
        pCamLaneData->LaneMarkerInfo[1].f_MarkerDist;
    if (fABS(pCamLaneData->CourseInfo[1].f_Angle) < 1.5) {
        right_lane_mark.geometry.heading_angle =
            pCamLaneData->CourseInfo[1].f_Angle;
    }
    if (fABS(pCamLaneData->CourseInfo[1].CourseInfoSegNear.f_C0) < 0.1) {
        right_lane_mark.geometry.curvature =
            pCamLaneData->CourseInfo[1].CourseInfoSegNear.f_C0;
    }
    if (fABS(pCamLaneData->CourseInfo[1].CourseInfoSegNear.f_C1) < 0.1) {
        right_lane_mark.geometry.curvature_rate =
            pCamLaneData->CourseInfo[1].CourseInfoSegNear.f_C1;
    }
    right_lane_mark.property.confidence =
        pCamLaneData->LaneMarkerInfo[1].u_ExistanceProbability;

    PACCProcess(
        cycle_time,
        (uint8)(cc_status->VLC_DRIVER_REQUESTS.SPEEDOMETER_VEHICLE_SPEED /
                Scale_100),
        (float32)das_input->VEHICLE_SPEED / Scale_100,
        (float32)input->LATERAL_ACCELERATION / Scale_1000, &left_lane_mark,
        &right_lane_mark, p_pacc_info);

    cc_status->VLC_ACCEL_CONTROL_DATA.MAXIMUM_ACCELERATION_LATERAL_LIMITED =
        (sint16)(p_pacc_info->pacc_acceleration * Scale_1000);

    /* Lateral acceleration limiter active ? */
    if ((cc_status->VLC_ACCEL_CONTROL_DATA
             .MAXIMUM_ACCELERATION_LATERAL_LIMITED <
         cc_status->VLC_CONTROL_DATA.VLC_ACCELERATION) &&
        (cc_status->VLC_ACCEL_CONTROL_DATA
             .MAXIMUM_ACCELERATION_LATERAL_LIMITED <
         acc_output->DISTANCE_CTRL_ACCEL_MAX)) {
        cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.LAT_ACCEL_LIM_ACTIVE =
            TRUE;
    } else {
        cc_status->VLC_ACCEL_CONTROL_DATA.ACCEL_STATUS.LAT_ACCEL_LIM_ACTIVE =
            FALSE;
    }

    cc_status->VLC_LAT_ACCEL_LAST_CYCLE = lat_accel;
    cc_status->VLC_LAT_ACCEL_GRAD_LAST_CYCLE = lat_accel_grad;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU1_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */