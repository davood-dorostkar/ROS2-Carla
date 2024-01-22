/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "acc.h"
#include "acc_par.h"
#include "mat_std_ext.h"
#include "mat_param_ext.h"

#define Fct_acc_alert_max_sim_steps 15

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static times_t AccAlertOutputTimer =
    (times_t)0; /*!< the acc alert shall be output a defined period of time,
                     to avoid to short acoustical and visual signals */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

static void AVLC_DETERMINE_ALERT(acc_alert_data_t* alert_data,
                                 const VLC_acc_object_t* alert_object,
                                 VLC_acc_output_data_t* output,
                                 times_t cycle_time,
                                 const acc_input_data_t* input);
static void AVLC_DETERMINE_ESTIMATED_DISTANCE(
    acc_alert_data_t* alert_data,
    const VLC_DFV2SenInfo_t* pDFVLongOut,
    const VLC_acc_object_t* alert_object,
    const acc_input_data_t* input);
static void AVLC_DETERMINE_ALERT_TRESHOLD(acc_alert_data_t* alert_data,
                                          const acc_input_data_t* input);

/*************************************************************************************************************************
  Functionname:    AVLC_DETERMINE_ALERT */
static void AVLC_DETERMINE_ALERT(acc_alert_data_t* alert_data,
                                 const VLC_acc_object_t* alert_object,
                                 VLC_acc_output_data_t* output,
                                 times_t cycle_time,
                                 const acc_input_data_t* input) {
    times_t Acc_alert_supress_time;

    Acc_alert_supress_time = (times_t)MAT_CALCULATE_PARAM_VALUE1D(
        (const sint16*)Acc_alert_supress_alert_time,
        (uint16)Acc_alert_supress_alert_time_points, input->LONG_VELOCITY);

    static times_t alert_time_gap, alert_ttc;
    if (MAT_DIV(alert_object->AUTOSAR.LONG_DISPLACEMENT, Distance_s,
                input->LONG_VELOCITY, Velocity_s,
                Time_s) > Acc_headway_time_thres) {
        alert_time_gap = Acc_headway_time_thres;
    } else {
        alert_time_gap =
            MAT_DIV(alert_object->AUTOSAR.LONG_DISPLACEMENT, Distance_s,
                    input->LONG_VELOCITY, Velocity_s, Time_s);
    }

    if (alert_object->AUTOSAR.REL_LONG_SPEED < 0) {
        if (MAT_DIV(alert_object->AUTOSAR.LONG_DISPLACEMENT, Distance_s,
                    -alert_object->AUTOSAR.REL_LONG_SPEED, Velocity_s,
                    Time_s) > Acc_headway_time_thres) {
            alert_ttc = Acc_headway_time_thres;
        } else {
            alert_ttc = MAT_DIV(
                alert_object->AUTOSAR.LONG_DISPLACEMENT, Distance_s,
                -alert_object->AUTOSAR.REL_LONG_SPEED, Velocity_s, Time_s);
        }
    } else {
        alert_ttc = Acc_headway_time_thres;
    }

    if ((alert_time_gap <
             MAT_CALCULATE_PARAM_VALUE1D(Acc_min_alert_thres_time_gap,
                                         Acc_min_alert_thres_time_gap_points,
                                         input->LONG_VELOCITY) ||
         alert_ttc < MAT_CALCULATE_PARAM_VALUE1D(Acc_min_alert_thres_ttc,
                                                 Acc_min_alert_thres_ttc_points,
                                                 input->LONG_VELOCITY) ||
         alert_object->ACCEL_REQUEST_FUZZY <
             MAT_CALCULATE_PARAM_VALUE1D(Acc_min_alert_thres_accel,
                                         Acc_min_alert_thres_accel_points,
                                         input->LONG_VELOCITY)) &&
        input->LONG_VELOCITY >= Acc_alert_active_speed_low_limit &&
        // (alert_object->AUTOSAR.OBJECT_STATUS.MOVING == TRUE ||
        //  alert_object->AUTOSAR.OBJECT_STATUS.STOPPED == TRUE ||
        //  alert_object->AUTOSAR.OBJECT_STATUS.STANDING == TRUE &&
        //      input->LONG_VELOCITY < Acc_decel_on_stationary_speed) &&
        alert_data->ALERT_SUPRESS_TIME == 0) {
        output->AVLC_OUTPUT_STATUS.ALERT = TRUE;
        AccAlertOutputTimer = Acc_alert_min_output_time;
    }

    if (output->AVLC_OUTPUT_STATUS.ALERT == FALSE) {
        if (alert_data->ALERT_SUPRESS_TIME >= cycle_time) {
            alert_data->ALERT_SUPRESS_TIME =
                (times_t)(alert_data->ALERT_SUPRESS_TIME - cycle_time);
        } else {
            alert_data->ALERT_SUPRESS_TIME = 0;
        }
    } else {
        if (((alert_time_gap >= MAT_CALCULATE_PARAM_VALUE1D(
                                    Acc_max_alert_thres_time_gap,
                                    Acc_max_alert_thres_time_gap_points,
                                    input->LONG_VELOCITY) &&
              alert_ttc >=
                  MAT_CALCULATE_PARAM_VALUE1D(Acc_max_alert_thres_ttc,
                                              Acc_max_alert_thres_ttc_points,
                                              input->LONG_VELOCITY)) &&
             alert_object->ACCEL_REQUEST_FUZZY >=
                 MAT_CALCULATE_PARAM_VALUE1D(Acc_max_alert_thres_accel,
                                             Acc_max_alert_thres_accel_points,
                                             input->LONG_VELOCITY) &&
             AccAlertOutputTimer == 0) ||
            input->LONG_VELOCITY < Acc_alert_active_speed_low_limit) {
            output->AVLC_OUTPUT_STATUS.ALERT = FALSE;
            alert_data->ALERT_SUPRESS_TIME = Acc_alert_supress_time;
        }
    }
    /*! Count down the AccAlertOutputTimer */
    SWITCH_SET_COUNTER(cycle_time, &AccAlertOutputTimer);
}

/*************************************************************************************************************************
  Functionname:    AVLC_DETERMINE_ESTIMATED_DISTANCE */
static void AVLC_DETERMINE_ESTIMATED_DISTANCE(
    acc_alert_data_t* alert_data,
    const VLC_DFV2SenInfo_t* pDFVLongOut,
    const VLC_acc_object_t* alert_object,
    const acc_input_data_t* input) {
    sint32 help_var;
    uint8 index;
    uint8 sim_steps;
    acceleration_t a_own, a_comm, a_req;
    velocity_t v_own;
    distance_t s_own;
    distance_t s_host;
    VLC_acc_object_t sim_object;
    uint8 stop_sim;
    times_t sim_time;
    uint8 was_below_alert_dist;

    was_below_alert_dist = FALSE;

    index = (uint8)0;
    sim_steps = (uint8)0;
    sim_object = *alert_object;

    stop_sim = FALSE;

    if (sim_object.AUTOSAR.OBJECT_STATUS.DETECTED == TRUE) {
        /* Initialization */
        a_own = input->LONG_ACCELERATION;
        a_comm = a_own;
        a_req = a_own;
        v_own = input->LONG_VELOCITY;
        s_own = (distance_t)0;

        s_host = sim_object.AUTOSAR.LONG_DISPLACEMENT;

        /*host faster than target*/
        // if (sim_object.AUTOSAR.REL_LONG_SPEED <
        //     (velocity_t)MAT_CALCULATE_PARAM_VALUE1D(
        //         Acc_vrel_estim_min, Acc_vrel_estim_min_points, v_own)) {
        //     sim_time = (times_t)MAT_CALCULATE_PARAM_VALUE1D(
        //         (const sint16*)Acc_alert_max_sim_time,
        //         (uint16)Acc_max_sim_time_points, v_own);
        // } else {
        sim_time = (times_t)0;
        // }
        /*calculate discrete sim steps*/
        sim_steps = (uint8)((sim_time + (Acc_alert_sim_time_step >> 1)) /
                            Acc_alert_sim_time_step);

        if (sim_steps > Fct_acc_alert_max_sim_steps) {
            sim_steps = Fct_acc_alert_max_sim_steps;
        }

        /* Start Simulation */
        for (index = 0; (index < sim_steps) && (stop_sim == (uint8)FALSE);
             index++) {
            a_comm = AVLC_DETERMINE_DIST_CONTROL_ACCEL(
                &sim_object, pDFVLongOut->MovingTime,
                input->DRIVER_CONTROLS.HEADWAY_SETTING, a_own, v_own);
            /*limit aown to allowed limits*/
            a_comm = (acceleration_t)MAT_LIM(
                (sint32)a_comm, (sint32)sim_object.MAX_ALLOWED_DECEL,
                (sint32)sim_object.MAX_ALLOWED_ACCEL);

            /*limit change rate*/
            a_req = (acceleration_t)MAT_LIM_GRAD(
                (sint32)a_comm, (sint32)a_req,
                (sint32)MAT_CALCULATE_PARAM_VALUE1D(Acc_max_neg_grad,
                                                    Acc_neg_grad_points, v_own),
                (sint32)MAT_CALCULATE_PARAM_VALUE1D(
                    Acc_max_pos_grad_neg_accel, Acc_pos_grad_neg_accel_points,
                    v_own),
                Acc_alert_sim_time_step);

            /*use a simple vehicle model to filter a_own*/
            a_own = (acceleration_t)MAT_FILT(
                (sint32)a_req, (sint32)a_own,
                Acc_alert_sim_vehicle_filter_time / Acc_alert_sim_time_step);

            /* Predict Path of host vehicle for the next step */
            help_var = (sint32)v_own * (sint32)Acc_alert_sim_time_step;
            help_var /= ((sint32)Time_s);
            help_var *= ((sint32)Distance_s);
            help_var /= ((sint32)Velocity_s);
            s_own =
                (distance_t)MAT_LIM(((sint32)s_own + help_var),
                                    (sint32)Distance_min, (sint32)Distance_max);
            help_var = (sint32)a_own * (sint32)Acc_alert_sim_time_step;
            help_var /= (((sint32)2) * ((sint32)Time_s));
            help_var *= ((sint32)Acc_alert_sim_time_step);
            help_var /= ((sint32)Time_s);
            help_var *= ((sint32)Distance_s);
            help_var /= ((sint32)Acceleration_s);
            s_own =
                (distance_t)MAT_LIM(((sint32)s_own + help_var),
                                    (sint32)Distance_min, (sint32)Distance_max);

            /* Predict Speed of host vehicle for the next step */
            help_var = (sint32)a_own * (sint32)Acc_alert_sim_time_step;
            help_var /= ((sint32)Acceleration_s);
            help_var *= ((sint32)Velocity_s);
            help_var /= ((sint32)Time_s);
            v_own = (velocity_t)MAT_LIM(((sint32)v_own + help_var), (sint32)0,
                                        (sint32)Speed_max);

            /* Predict Distance of In path Object for the next step */
            help_var =
                (sint32)sim_object.LONG_SPEED * (sint32)Acc_alert_sim_time_step;
            help_var /= ((sint32)Time_s);
            help_var *= ((sint32)Distance_s);
            help_var /= ((sint32)Velocity_s);
            s_host =
                (distance_t)MAT_LIM(((sint32)s_host + help_var),
                                    (sint32)Distance_min, (sint32)Distance_max);
            help_var = (sint32)sim_object.LONG_ACCEL_MODIFIED *
                       (sint32)Acc_alert_sim_time_step;
            help_var /= (((sint32)2) * ((sint32)Time_s));
            help_var *= ((sint32)Acc_alert_sim_time_step);
            help_var /= ((sint32)Time_s);
            help_var *= ((sint32)Distance_s);
            help_var /= ((sint32)Acceleration_s);
            s_host =
                (distance_t)MAT_LIM(((sint32)s_host + help_var),
                                    (sint32)Distance_min, (sint32)Distance_max);
            sim_object.AUTOSAR.LONG_DISPLACEMENT =
                (distance_t)MAT_LIM(((sint32)s_host - (sint32)s_own),
                                    (sint32)Distance_min, (sint32)Distance_max);

            /* Predict Speed of In path Object for the next step */
            help_var = (sint32)sim_object.LONG_ACCEL_MODIFIED *
                       (sint32)Acc_alert_sim_time_step;
            help_var /= ((sint32)Time_s);
            help_var *= ((sint32)Velocity_s);
            help_var /= ((sint32)Acceleration_s);
            sim_object.LONG_SPEED =
                (velocity_t)MAT_LIM(((sint32)sim_object.LONG_SPEED + help_var),
                                    (sint32)0, (sint32)Speed_max);

            /* Estimated RelativeSpeed is difference in ObjectSpeed and v_own */
            sim_object.AUTOSAR.REL_LONG_SPEED =
                (velocity_t)(sim_object.LONG_SPEED - v_own);

            /*save values...*/
            // if ((sim_object.AUTOSAR.LONG_DISPLACEMENT <
            //      (distance_t)MAT_CALCULATE_PARAM_VALUE2D(
            //          Acc_min_alert_thres, Acc_min_alert_thres_host_speed,
            //          Acc_min_alert_thres_rel_speed,
            //          Acc_min_alert_thres_host_speed_points,
            //          Acc_min_alert_thres_rel_speed_points, (float32)v_own,
            //          (float32)sim_object.AUTOSAR.REL_LONG_SPEED)) &&
            //     (was_below_alert_dist == (uint8)FALSE)) {
            //     /* Set flag that we were already below alert threshold */
            //     was_below_alert_dist = TRUE;
            //     /* Store the alert data */
            //     alert_data->ESTIM_DISTANCE =
            //         sim_object.AUTOSAR.LONG_DISPLACEMENT;
            //     alert_data->ESTIM_REL_SPEED =
            //     sim_object.AUTOSAR.REL_LONG_SPEED;
            //     alert_data->ESTIM_HOST_SPEED = v_own;
            // }

            /* Stop simulation */
            if (((sim_object.AUTOSAR.LONG_DISPLACEMENT <= (distance_t)0)) ||
                (sim_object.AUTOSAR.REL_LONG_SPEED >= (velocity_t)0) ||
                (v_own == (velocity_t)0)) {
                stop_sim = TRUE;
            }

            /*avoid s_host exceeding the upper limit*/
            if (s_host > s_own) {
                s_host = (distance_t)(s_host - s_own);
                s_own = (distance_t)0;
            }
        }

        /*if estimated distance distance goes not below 0 - use saved values*/
        if ((was_below_alert_dist != (uint8)TRUE) ||
            (sim_object.AUTOSAR.LONG_DISPLACEMENT <= (distance_t)0)) {
            alert_data->ESTIM_DISTANCE = sim_object.AUTOSAR.LONG_DISPLACEMENT;
            alert_data->ESTIM_REL_SPEED = sim_object.AUTOSAR.REL_LONG_SPEED;
            alert_data->ESTIM_HOST_SPEED = v_own;
        }
    } else {
        alert_data->ESTIM_DISTANCE = Distance_max;

        alert_data->ESTIM_REL_SPEED = (velocity_t)0;

        alert_data->ESTIM_HOST_SPEED = (velocity_t)0;
    }
}

/*************************************************************************************************************************
  Functionname:    AVLC_DETERMINE_ALERT_TRESHOLD */
static void AVLC_DETERMINE_ALERT_TRESHOLD(acc_alert_data_t* alert_data,
                                          const acc_input_data_t* input) {
    // alert_data->ALERT_REL_SPEED = MAT_CALCULATE_PARAM_VALUE1D(
    //     Acc_vrel_estim_max, Acc_vrel_estim_max_points,
    //     alert_data->ESTIM_HOST_SPEED);

    // alert_data->ALERT_DISTANCE = (distance_t)MAT_CALCULATE_PARAM_VALUE2D(
    //     Acc_max_alert_thres, Acc_max_alert_thres_host_speed,
    //     Acc_max_alert_thres_rel_speed, Acc_max_alert_thres_host_speed_points,
    //     Acc_max_alert_thres_rel_speed_points,
    //     (float32)alert_data->ESTIM_HOST_SPEED,
    //     (float32)alert_data->ESTIM_REL_SPEED);

    // distance_t MinDist, MaxDist;
    // MinDist = MAT_CALCULATE_PARAM_VALUE1D(Acc_min_alert_thres,
    //                                       Acc_min_alert_thres_points,
    //                                       alert_data->ESTIM_HOST_SPEED);
    // MaxDist = MAT_CALCULATE_PARAM_VALUE1D(Acc_max_alert_thres,
    //                                       Acc_max_alert_thres_points,
    //                                       alert_data->ESTIM_HOST_SPEED);

    // if (input->INPUT_STATUS.AVLC_ON == FALSE) {
    //     /*set position between Acc_min_alert_thres curve and
    //     Acc_max_alert_thres
    //      * curve to 0 (Acc_min_alert_thres)*/
    //     alert_data->ALERT_THRES_FACTOR = 0;
    // } else {
    //     factor_t NewFactor;

    //     if (alert_data->ESTIM_DISTANCE >= MaxDist) {
    //         NewFactor = Factor_s;
    //     } else {
    //         if (alert_data->ESTIM_DISTANCE <= MinDist) {
    //             NewFactor = 0;
    //         } else {
    //             NewFactor = (factor_t)(((sint32)Factor_s *
    //                                     ((sint32)alert_data->ESTIM_DISTANCE -
    //                                      (sint32)MinDist)) /
    //                                    ((sint32)MaxDist - (sint32)MinDist));
    //         }
    //     }
    //     alert_data->ALERT_THRES_FACTOR = (factor_t)MAT_MAX(
    //         (sint32)NewFactor, (sint32)alert_data->ALERT_THRES_FACTOR);
    // }

    // alert_data->ALERT_DISTANCE =
    //     (distance_t)(MinDist +
    //                  (distance_t)((((sint32)MaxDist - (sint32)MinDist) *
    //                                (sint32)alert_data->ALERT_THRES_FACTOR) /
    //                               (sint32)Factor_s));
}

/*************************************************************************************************************************
  Functionname:    AVLC_ALERT_EXEC */
void AVLC_ALERT_EXEC(const acc_input_data_t* input,
                     const VLC_acc_object_t* alert_object,
                     const VLC_DFV2SenInfo_t* pDFVLongOut,
                     VLC_acc_output_data_t* output,
                     acc_alert_data_t* acc_alert_data,
                     times_t cycle_time) {
    // AVLC_DETERMINE_ESTIMATED_DISTANCE(acc_alert_data, pDFVLongOut,
    // alert_object,
    //                                   input);

    // AVLC_DETERMINE_ALERT_TRESHOLD(acc_alert_data, input);

    AVLC_DETERMINE_ALERT(acc_alert_data, alert_object, output, cycle_time,
                         input);
}

/*************************************************************************************************************************
  Functionname:    AVLC_ALERT_INIT */
void AVLC_ALERT_INIT(acc_alert_data_t* acc_alert_data) {
    acc_alert_data->ESTIM_REL_SPEED = (velocity_t)0;
    acc_alert_data->ESTIM_DISTANCE = (distance_t)0;
    acc_alert_data->ESTIM_HOST_SPEED = (velocity_t)0;
    acc_alert_data->ALERT_THRES_FACTOR = (factor_t)Factor_s;
    acc_alert_data->ALERT_DISTANCE = (distance_t)0;
    acc_alert_data->ALERT_SUPRESS_TIME = Acc_alert_sim_vehicle_filter_time;

    AccAlertOutputTimer = (times_t)0;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */