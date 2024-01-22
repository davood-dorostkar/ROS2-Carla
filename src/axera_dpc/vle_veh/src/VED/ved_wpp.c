
#include "ved_consts.h"
#include "ved_wpp.h"
#include "ved_wpp_private.h"
#include <ved.h>
#include "ved_rt_matrixlib.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/*
 * Output and update for atomic system:
 *    '<Root>/Get_IO_State10'
 *    '<Root>/Get_IO_State11'
 *    '<Root>/Get_IO_State12'
 *    '<Root>/Get_IO_State5'
 *    '<Root>/Get_IO_State6'
 *    '<Root>/Get_IO_State7'
 *    '<Root>/Get_IO_State8'
 *    '<Root>/Get_IO_State9'
 */
void ved__wpp_Get_IO_State10(const uint8_T rtu_state_in[32],
                             rtB_Get_IO_State10_ved__wpp *localB,
                             uint32_T rtp_Filter) {
    /* MultiPortSwitch: '<S1>/Index Vector' incorporates:
     *  Constant: '<S1>/Constant1'
     */
    localB->IndexVector = rtu_state_in[(rtp_Filter)];
}

/*
 * Initial conditions for atomic system:
 *    '<Root>/front_left_wheel_speed_fusion'
 *    '<Root>/front_right_wheel_speed_fusion'
 *    '<Root>/rear_left_wheel_speed_fusion'
 *    '<Root>/rear_right_wheel_speed_fusion'
 */
void wheel_speed_fusion_Init(rtDW_wheel_speed_fusion *localDW) {
    int32_T i;

    /* InitializeConditions for UnitDelay: '<S69>/FixPt Unit Delay2' */
    localDW->FixPtUnitDelay2_DSTATE = 1U;

    /* InitializeConditions for UnitDelay: '<S37>/init_delay' */
    localDW->init_delay_DSTATE = TRUE;

    /* InitializeConditions for UnitDelay: '<S56>/P_delay' */
    for (i = 0; i < 4; i++) {
        localDW->P_delay_DSTATE[(i)] = ved__wpp_P_init_p[(i)];
    }

    /* InitializeConditions for UnitDelay: '<S41>/FixPt Unit Delay2' */
    localDW->FixPtUnitDelay2_DSTATE_o = 15U;
}

/*
 * Start for atomic system:
 *    '<Root>/front_left_wheel_speed_fusion'
 *    '<Root>/front_right_wheel_speed_fusion'
 *    '<Root>/rear_left_wheel_speed_fusion'
 *    '<Root>/rear_right_wheel_speed_fusion'
 */
void wheel_speed_fusion_Start(rtDW_wheel_speed_fusion *localDW) {
    /* Start for ifaction SubSystem: '<S64>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S71>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &localDW->PHt_HPHt_R_DWORK4[0];
        for (i = 0; i < 4; i++) {
            *dWork++ = 0.0;
        }

        dWork = &localDW->PHt_HPHt_R_DWORK4[0];
        while (dWork < &localDW->PHt_HPHt_R_DWORK4[0] + 4) {
            *dWork = 1;
            dWork += 3;
        }
    }

    /* end of Start for SubSystem: '<S64>/calculate the gain' */
}

/*
 * Output and update for atomic system:
 *    '<Root>/front_left_wheel_speed_fusion'
 *    '<Root>/front_right_wheel_speed_fusion'
 *    '<Root>/rear_left_wheel_speed_fusion'
 *    '<Root>/rear_right_wheel_speed_fusion'
 */
void wheel_speed_fusion(real32_T rtu_wheel_velocity,
                        real32_T rtu_CycleTime,
                        real32_T rtu_whl_Circumference,
                        uint8_T rtu_number_wheel_pulse,
                        uint8_T rtu_wheel_velocity_valid,
                        uint8_T rtu_use_pulse,
                        real32_T rtu_aqua_slip_correction,
                        uint8_T rtu_diff_wheel_pulse,
                        rtB_wheel_speed_fusion *localB,
                        rtDW_wheel_speed_fusion *localDW) {
    /* local block i/o variables */
    real32_T rtb_Divide;
    real32_T rtb_HPH_R[4];
    real32_T rtb_P_predHt[4];
    real32_T rtb_K[4];
    real32_T rtb_exponent;
    real32_T rtb_exponent_d;
    real32_T rtb_exponent_h;

    /* local scratch DWork variables */
    real32_T PHt_HPHt_R_DWORK1[4];
    real32_T PHt_HPHt_R_DWORK3[4];
    real32_T PHt_HPHt_R_DWORK5[4];
    int32_T PHt_HPHt_R_DWORK2[2];
    real32_T wheel_puls_velocity_out;
    real32_T counter_out;
    real32_T m_out;
    real32_T V_pred;
    real32_T rtb_nu[2];
    real32_T rtb_x_pred[2];
    real32_T rtb_th_Delay;
    real32_T rtb_th_Delay_g;
    real32_T rtb_th_Delay_c;
    real32_T rtb_th_Delay1;
    real32_T rtb_th_Delay1_b;
    real32_T rtb_th_Delay1_h;
    uint8_T rtb_y_n;
    real32_T rtb_Product1_k;
    real32_T rtb_Divide_m;
    real32_T rtb_T2;
    real32_T rtb_T1;
    real32_T rtb_T0;
    real32_T rtb_last_accel_corr;
    real32_T rtb_single_puls_velocity_out;
    real32_T rtb_TmpSignalConversionAtSFun_a[2];
    real32_T rtb_diff;
    real32_T rtb_A[4];
    int8_T rtb_KH_n[4];
    int8_T rtb_H[4];
    real32_T rtb_P_pred[4];
    int32_T i;
    real32_T rtb_TmpSignalConversionAtSFun_0[2];
    real32_T rtb_nu_0[2];
    real32_T rtb_A_0[4];
    real32_T rtb_TmpSignalConversionAtSFun_1[4];
    real32_T rtb_A_1[4];
    int32_T i_0;
    int32_T i_1;
    static int8_T tmp[4] = {1, 1, 0, 0};

    static int8_T tmp_0[4] = {1, 0, 0, 1};

    /* Embedded MATLAB: '<S51>/make_A_matrix' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/A/make_A_matrix': '<S60>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S60>:1:5' */
    rtb_A[0] = 1.0F;
    rtb_A[2] = rtu_CycleTime;
    for (i = 0; i < 2; i++) {
        rtb_A[1 + (i << 1)] = (real32_T)i;
    }

    /* UnitDelay: '<S69>/FixPt Unit Delay1' */
    for (i = 0; i < 2; i++) {
        rtb_nu[i] = localDW->FixPtUnitDelay1_DSTATE[(i)];
    }

    /* Switch: '<S69>/Init' incorporates:
     *  Constant: '<S56>/x_init_a'
     *  UnitDelay: '<S69>/FixPt Unit Delay2'
     */
    if (localDW->FixPtUnitDelay2_DSTATE != 0) {
        rtb_nu[0] = rtu_wheel_velocity;
        rtb_nu[1] = ved__wpp_x_init_p;
    }

    /* Product: '<S56>/Ax' */
    for (i = 0; i < 2; i++) {
        rtb_x_pred[i] = 0.0F;
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_x_pred[i] =
                (rtb_A[(i_0 << 1) + i] * rtb_nu[i_0]) + rtb_x_pred[i];
        }
    }

    /* Embedded MATLAB: '<S56>/hx' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/kalman_filter/hx': '<S68>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S68>:1:6' */
    for (i = 0; i < 4; i++) {
        rtb_H[i] = tmp[i];
    }

    /* '<S68>:1:9' */

    /* Product: '<S52>/Divide' incorporates:
     *  Constant: '<S52>/Constant'
     *  Product: '<S52>/Product1'
     */
    rtb_Divide = (rtu_CycleTime * rtu_CycleTime) / 2.0F;

    /* UnitDelay: '<S77>/2th_Delay' */
    rtb_th_Delay = localDW->th_Delay_DSTATE_b;

    /* UnitDelay: '<S77>/3th_Delay' */
    rtb_th_Delay_g = localDW->th_Delay_DSTATE_f;

    /* UnitDelay: '<S77>/4th_Delay' */
    rtb_th_Delay_c = localDW->th_Delay_DSTATE_o;

    /* UnitDelay: '<S77>/1th_Delay1' */
    rtb_th_Delay1 = localDW->th_Delay1_DSTATE;

    /* UnitDelay: '<S77>/2th_Delay1' */
    rtb_th_Delay1_b = localDW->th_Delay1_DSTATE_f;

    /* UnitDelay: '<S77>/3th_Delay1' */
    rtb_th_Delay1_h = localDW->th_Delay1_DSTATE_j;

    /* Embedded MATLAB: '<S37>/init_wheel_pulse' incorporates:
     *  UnitDelay: '<S37>/init_delay'
     */
    /* Embedded MATLAB Function 'wheel pulses to speed/init_wheel_pulse':
     * '<S76>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((int32_T)localDW->init_delay_DSTATE) == 1) {
        /* '<S76>:1:4' */
        /* '<S76>:1:5' */
        /* '<S76>:1:6' */
        rtb_y_n = 0U;
    } else {
        /* '<S76>:1:8' */
        /* '<S76>:1:9' */
        rtb_y_n = rtu_diff_wheel_pulse;
    }

    /* Product: '<S78>/Product1' */
    rtb_Product1_k = rtu_CycleTime * ((real32_T)rtu_number_wheel_pulse);

    /* Embedded MATLAB: '<S78>/divide' incorporates:
     *  Product: '<S78>/Product'
     */
    /* Embedded MATLAB Function 'puls to speed/divide': '<S80>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((real32_T)fabs(rtb_Product1_k)) >= 0.0001F) {
        /* '<S80>:1:5' */
        /* '<S80>:1:6' */
        rtb_Product1_k =
            (((real32_T)rtb_y_n) * rtu_whl_Circumference) / rtb_Product1_k;
    } else {
        /* '<S80>:1:8' */
        rtb_Product1_k = 0.0F;
    }

    /* Embedded MATLAB: '<S37>/puls_velocity_pre_processing' incorporates:
     *  UnitDelay: '<S37>/counter_delay'
     *  UnitDelay: '<S37>/m_delay'
     *  UnitDelay: '<S37>/puls_speed_delay'
     *  UnitDelay: '<S37>/single_puls_velocity_delay'
     */
    /* Embedded MATLAB Function 'wheel pulses to
     * speed/puls_velocity_pre_processing': '<S79>:1' */
    /*  This block calculates the wheel pulse velocity out of the pulses */
    if (rtu_number_wheel_pulse == 0) {
        /* '<S79>:1:3' */
        /*  increase the counter if no pulse is detected */
        /* '<S79>:1:5' */
        counter_out = localDW->counter_delay_DSTATE + 1.0F;

        /*  if no pulse is detected for some cycles, set all to zero */
        if (counter_out > ved__wpp_puls_velocity_para_p[0]) {
            /* '<S79>:1:7' */
            /* '<S79>:1:8' */
            wheel_puls_velocity_out = 0.0F;

            /* '<S79>:1:9' */
            rtb_single_puls_velocity_out = 0.0F;

            /* '<S79>:1:10' */
            m_out = 0.0F;
        } else {
            /*  The calculated gradient between the last two wheel pulse dirac
             * velocities  */
            /*  is used to increase or drop down the output puls velocity */
            /* '<S79>:1:14' */
            wheel_puls_velocity_out =
                localDW->puls_speed_delay_DSTATE + localDW->m_delay_DSTATE;

            /*  check if the wheel pulse speed is below zero */
            if (wheel_puls_velocity_out < 0.0F) {
                /* '<S79>:1:16' */
                /* '<S79>:1:17' */
                wheel_puls_velocity_out = 0.0F;
            }

            /*  save the wheel velocity of the last pulse dirac */
            /* '<S79>:1:20' */
            rtb_single_puls_velocity_out =
                localDW->single_puls_velocity_delay_DSTA;

            /* '<S79>:1:21' */
            m_out = localDW->m_delay_DSTATE;
        }
    } else {
        /*  if the first pulse is detected after a long time */
        if (((real32_T)fabs(localDW->puls_speed_delay_DSTATE)) <= 0.0001F) {
            /* '<S79>:1:25' */
            /*  with only one pulse it is not possible to calculate a suitable
             * pulse velocity */
            /*  but if one pulse is detected the vehicle is moving so the
             * velocity is increased  */
            /*  by a constant gradient  */
            /* '<S79>:1:29' */
            m_out = ved__wpp_puls_velocity_para_p[1] /
                    ved__wpp_puls_velocity_para_p[2];

            /* '<S79>:1:30' */
            wheel_puls_velocity_out = localDW->puls_speed_delay_DSTATE + m_out;

            /* '<S79>:1:31' */
            rtb_single_puls_velocity_out = 0.0F;

            /*  reset the counter */
            /* '<S79>:1:33' */
            counter_out = 1.0F;
        } else {
            /*  pulse velocity gradient = act pulse velocity - last_pulse
             * velocity / cycles between pulses */
            /* '<S79>:1:36' */
            m_out = (((rtb_Product1_k / localDW->counter_delay_DSTATE) -
                      localDW->single_puls_velocity_delay_DSTA) /
                     localDW->counter_delay_DSTATE) /
                    ved__wpp_puls_velocity_para_p[3];

            /*  only one pulse is detected, the puls velocity for this first
             * pulse can not be calculated so the single pulse velocity is zero
             */
            if (((real32_T)fabs(localDW->single_puls_velocity_delay_DSTA)) <=
                0.0001F) {
                /* '<S79>:1:38' */
                /* '<S79>:1:39' */
                wheel_puls_velocity_out =
                    localDW->puls_speed_delay_DSTATE + m_out;
            } else {
                /* '<S79>:1:41' */
                wheel_puls_velocity_out =
                    rtb_Product1_k / localDW->counter_delay_DSTATE;
            }

            /*  the single pulse velocity is the constant veloctiy between the
             */
            /*  last and the current pulse */
            /* '<S79>:1:45' */
            rtb_single_puls_velocity_out =
                rtb_Product1_k / localDW->counter_delay_DSTATE;

            /*  reset the counter */
            /* '<S79>:1:47' */
            counter_out = 1.0F;
        }
    }

    /* Product: '<S77>/Divide' incorporates:
     *  Constant: '<S77>/Constant'
     *  Sum: '<S77>/Add'
     *  UnitDelay: '<S77>/1th_Delay'
     */
    rtb_Divide_m =
        (((((((localDW->th_Delay_DSTATE + localDW->th_Delay_DSTATE_b) +
              localDW->th_Delay_DSTATE_f) +
             localDW->th_Delay_DSTATE_o) +
            localDW->th_Delay1_DSTATE) +
           localDW->th_Delay1_DSTATE_f) +
          localDW->th_Delay1_DSTATE_j) +
         wheel_puls_velocity_out) *
        0.125F;

    /* UnitDelay: '<S74>/T2' */
    rtb_T2 = localDW->T2_DSTATE;

    /* UnitDelay: '<S74>/T1' */
    rtb_T1 = localDW->T1_DSTATE;

    /* UnitDelay: '<S74>/T0' */
    rtb_T0 = localDW->T0_DSTATE;

    /* Product: '<S74>/Divide' incorporates:
     *  Constant: '<S74>/filter_length'
     *  Sum: '<S74>/Add'
     *  UnitDelay: '<S74>/T3'
     */
    rtb_Product1_k =
        (((localDW->T3_DSTATE + localDW->T2_DSTATE) + localDW->T1_DSTATE) +
         localDW->T0_DSTATE) *
        0.25F;

    /* Embedded MATLAB: '<S58>/q_gain_delta' incorporates:
     *  UnitDelay: '<S58>/Unit Delay'
     */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/q_gain/q_gain_delta': '<S75>:1' */
    /*  the difference between the mean filtered last velocity and the actual
     * wheel velocity */
    /* '<S75>:1:3' */
    rtb_diff = rtu_wheel_velocity - rtb_Product1_k;

    /*  predict the wheel velocity by the estimated wheel acceleration */
    /* '<S75>:1:5' */
    V_pred =
        (ved__wpp_Q_gain_p[2] * localDW->UnitDelay_DSTATE) + rtb_Product1_k;

    /*  if the predicted velocity is below zero raise the model Variance */
    /*  to protect velocity below zero */
    if (V_pred < 0.0F) {
        /* '<S75>:1:8' */
        /* '<S75>:1:9' */
        rtb_diff = (((real32_T)fabs(V_pred)) * ved__wpp_Q_gain_p[3]) +
                   ((real32_T)fabs(rtb_diff));
        if (((real32_T)fabs(rtb_Product1_k)) < ved__wpp_Q_gain_p[4]) {
            /* '<S75>:1:10' */
            /* '<S75>:1:11' */
            rtb_diff = ved__wpp_Q_gain_p[5];
        }
    } else {
        /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition
         * below is rearranged */
        /*  if no zero under-run is detected but the wheel velocity is in the
         * low speed range */
        if ((((real32_T)fabs(rtb_Divide_m)) >= 0.0001F) &&
            (rtb_Divide_m <= ved__wpp_Q_gain_p[6])) {
            /* '<S75>:1:16' */
            /*  the diff gain control factor is raised with to zero */
            /* '<S75>:1:18' */
            /* '<S75>:1:19' */
            rtb_diff = ((real32_T)fabs(ved__wpp_Q_gain_p[6] - rtb_Divide_m)) +
                       ved__wpp_Q_gain_p[7];
        }

        /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition
         * below is rearranged */
        /*  if wheel velocity in is below 0.6 and wheel puls velocity is zero
         * (no pulse available) */
        if ((((real32_T)fabs(rtb_Divide_m)) <= 0.0001F) &&
            (rtu_wheel_velocity <= ved__wpp_Q_gain_p[6])) {
            /* '<S75>:1:24' */
            /*  set diff value to const value */
            /* '<S75>:1:26' */
            rtb_diff = 2.0F;
        }
    }

    /* Embedded MATLAB: '<S58>/get_gain_bias' incorporates:
     *  Constant: '<S36>/ved__wpp_Q_gain'
     */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/q_gain/get_gain_bias': '<S73>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S73>:1:5' */
    /* '<S73>:1:6' */

    /* Sum: '<S58>/Sum2' incorporates:
     *  Abs: '<S58>/Abs'
     *  Product: '<S58>/Product'
     */
    rtb_Product1_k = (((real32_T)fabs(rtb_diff)) * ved__wpp_Q_gain_p[0]) +
                     ved__wpp_Q_gain_p[1];

    rtb_TmpSignalConversionAtSFun_a[0] = rtb_Divide;
    rtb_TmpSignalConversionAtSFun_a[1] = rtu_CycleTime;

    /* Embedded MATLAB: '<S52>/makeQ' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/Q/makeQ': '<S61>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S61>:1:5' */

    /* Sum: '<S56>/APA_Q' incorporates:
     *  Math: '<S56>/At'
     *  Product: '<S56>/APAt'
     *  UnitDelay: '<S56>/P_delay'
     */
    for (i = 0; i < 2; i++) {
        /* Product: '<S52>/Product' incorporates:
         *  Constant: '<S52>/sigma_model'
         */
        rtb_diff = ved__wpp_Q_sigmas_p[(i)] * rtb_Product1_k;
        rtb_TmpSignalConversionAtSFun_0[i] =
            rtb_TmpSignalConversionAtSFun_a[i] * rtb_diff;
        rtb_nu_0[i] = rtb_diff * rtb_TmpSignalConversionAtSFun_a[i];
        rtb_nu[i] = rtb_diff;
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_A_0[i + (i_0 << 1)] = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_A_0[i + (i_0 << 1)] =
                    (rtb_A[(i_1 << 1) + i] *
                     localDW->P_delay_DSTATE[(i_0 << 1) + i_1]) +
                    rtb_A_0[(i_0 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_TmpSignalConversionAtSFun_1[i_0 + (i << 1)] =
                rtb_TmpSignalConversionAtSFun_0[i_0] * rtb_nu_0[i];
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_A_1[i + (i_0 << 1)] = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_A_1[i + (i_0 << 1)] =
                    (rtb_A_0[(i_1 << 1) + i] * rtb_A[(i_1 << 1) + i_0]) +
                    rtb_A_1[(i_0 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_P_pred[i_0 + (i << 1)] =
                rtb_TmpSignalConversionAtSFun_1[(i << 1) + i_0] +
                rtb_A_1[(i << 1) + i_0];
        }
    }

    /* Math: '<S56>/Ht' */
    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_KH_n[i_0 + (i << 1)] = rtb_H[(i_0 << 1) + i];
        }
    }

    /* Embedded MATLAB: '<S53>/calc_exponent' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/calc_wheel_velocity_R_variance/calc_exponent':
     * '<S63>:1' */
    /*  if wheel velocity is high variance is low variance */
    /*  R22 = 40*exp(wheel_velocity_in * -10) + 0.002 */
    /*  check exponent is in single floating point range */
    /* '<S63>:1:6' */
    rtb_exponent = rtu_wheel_velocity * ved__wpp_R_p[1];
    if (rtb_exponent < -80.0F) {
        /* '<S63>:1:7' */
        /* '<S63>:1:8' */
        rtb_exponent = -80.0F;
    }

    if (rtb_exponent > 80.0F) {
        /* '<S63>:1:10' */
        /* '<S63>:1:11' */
        rtb_exponent = 80.0F;
    }

    /* Embedded MATLAB: '<S53>/calc_R' incorporates:
     *  S-Function (ex_sfun_exp): '<S53>/S-Function'
     */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/calc_wheel_velocity_R_variance/calc_R':
     * '<S62>:1' */
    /*  R22 = 40*exp(wheel_velocity_in * -10) + 0.002 */
    /* '<S62>:1:4' */
    rtb_Product1_k = (VED_GDBexp((real32_T)rtb_exponent) * ved__wpp_R_p[2]) +
                     ved__wpp_R_p[3];

    /* Embedded MATLAB: '<S36>/if_else_use_wheel_pulse' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/if_else_use_wheel_pulse': '<S55>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((uint32_T)rtu_use_pulse) == ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S55>:1:5' */
        /* '<S55>:1:6' */
        rtb_A[0] = ved__wpp_R_p[0];
        rtb_A[2] = 0.0F;
        rtb_A[1] = 0.0F;
        rtb_A[3] = rtb_Product1_k;

        /* '<S55>:1:7' */
        rtb_nu[0] = rtb_Divide_m;
        rtb_nu[1] = rtu_wheel_velocity;
    } else {
        /* '<S55>:1:9' */
        rtb_A[0] = ved__wpp_R_p[0];
        rtb_A[2] = 0.0F;
        rtb_A[1] = 0.0F;
        rtb_A[3] = rtb_Product1_k;

        /* '<S55>:1:10' */
        rtb_nu[0] = 0.0F;
        rtb_nu[1] = rtu_wheel_velocity;
    }

    /* Sum: '<S56>/HPH_R' incorporates:
     *  Product: '<S56>/HPHt'
     */
    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_A_0[i + (i_0 << 1)] = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_A_0[i + (i_0 << 1)] = (((real32_T)rtb_H[(i_1 << 1) + i]) *
                                           rtb_P_pred[(i_0 << 1) + i_1]) +
                                          rtb_A_0[(i_0 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_Product1_k = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_Product1_k += rtb_A_0[(i_1 << 1) + i] *
                                  ((real32_T)rtb_KH_n[(i_0 << 1) + i_1]);
            }

            rtb_HPH_R[i + (i_0 << 1)] = rtb_A[(i_0 << 1) + i] + rtb_Product1_k;
        }
    }

    /* Embedded MATLAB: '<S64>/calculate determinant' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/kalman_filter/Calculate Kalman gain
     * PHt_(HPHt_R)/calculate determinant': '<S70>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /* '<S70>:1:3' */
    V_pred = (rtb_HPH_R[0] * rtb_HPH_R[3]) - (rtb_HPH_R[1] * rtb_HPH_R[2]);

    /* Embedded MATLAB: '<S56>/Reset_x_pred' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/kalman_filter/Reset_x_pred': '<S66>:1'
     */
    if (((real32_T)fabs(V_pred)) <= 1.0E-16F) {
        /* '<S66>:1:4' */
        /* '<S66>:1:6' */
        rtb_TmpSignalConversionAtSFun_a[0] = rtu_wheel_velocity;
        rtb_TmpSignalConversionAtSFun_a[1] = 0.0F;
    } else {
        /* '<S66>:1:10' */
        for (i = 0; i < 2; i++) {
            rtb_TmpSignalConversionAtSFun_a[i] = rtb_x_pred[i];
        }
    }

    /* Product: '<S56>/P_pred*Ht' */
    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_P_predHt[i + (i_0 << 1)] = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_P_predHt[i + (i_0 << 1)] =
                    (rtb_P_pred[(i_1 << 1) + i] *
                     ((real32_T)rtb_KH_n[(i_0 << 1) + i_1])) +
                    rtb_P_predHt[(i_0 << 1) + i];
            }
        }
    }

    /* If: '<S64>/If' incorporates:
     *  ActionPort: '<S71>/Action Port'
     *  ActionPort: '<S72>/Action Port'
     *  SubSystem: '<S64>/calculate the gain'
     *  SubSystem: '<S64>/set gain to default value'
     */
    if (V_pred > 0.0F) {
        /* Product: '<S71>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {2, 2, 2, 2, 2, 2};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5[0], rtb_HPH_R,
                            &localDW->PHt_HPHt_R_DWORK4[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1[0],
                            &PHt_HPHt_R_DWORK2[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K, rtb_P_predHt, &PHt_HPHt_R_DWORK5[0],
                             &dims[3]);
        }
    } else {
        /* Constant: '<S72>/Constant' */
        for (i = 0; i < 4; i++) {
            rtb_K[i] = ved__wpp_ConstP.pooled1[(i)];
        }
    }

    /* Sum: '<S56>/x_Knu' incorporates:
     *  Product: '<S56>/Knu'
     *  Sum: '<S56>/z_Hx'
     */
    for (i = 0; i < 2; i++) {
        rtb_Product1_k = 0.0F;
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_Product1_k += ((real32_T)tmp[(i_0 << 1) + i]) * rtb_x_pred[i_0];
        }

        rtb_nu_0[i] = rtb_nu[i] - rtb_Product1_k;
    }

    for (i = 0; i < 2; i++) {
        rtb_Product1_k = 0.0F;
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_Product1_k += rtb_K[(i_0 << 1) + i] * rtb_nu_0[i_0];
        }

        rtb_nu[i] = rtb_TmpSignalConversionAtSFun_a[i] + rtb_Product1_k;
    }

    /* Embedded MATLAB: '<S36>/velocity' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/velocity': '<S59>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((real32_T)fabs(rtb_nu[0])) < 0.02F) {
        /* '<S59>:1:4' */
        /* '<S59>:1:5' */
        localB->y = 0.0F;
    } else {
        /* '<S59>:1:7' */
        localB->y = rtb_nu[0];
    }

    /* UnitDelay: '<S41>/FixPt Unit Delay1' */
    rtb_Product1_k = localDW->FixPtUnitDelay1_DSTATE_e;

    /* Switch: '<S41>/Init' incorporates:
     *  Constant: '<S42>/Constant'
     *  Constant: '<S43>/Constant'
     *  Constant: '<S44>/Constant'
     *  Constant: '<S45>/Constant'
     *  Constant: '<S46>/Constant'
     *  Logic: '<S41>/Logical Operator'
     *  Logic: '<S41>/Logical Operator1'
     *  Logic: '<S41>/Logical Operator2'
     *  RelationalOperator: '<S42>/Compare'
     *  RelationalOperator: '<S43>/Compare'
     *  RelationalOperator: '<S44>/Compare'
     *  RelationalOperator: '<S45>/Compare'
     *  RelationalOperator: '<S46>/Compare'
     *  UnitDelay: '<S41>/FixPt Unit Delay2'
     */
    if ((((rtu_wheel_velocity_valid == 3) || (rtu_wheel_velocity_valid == 4)) ||
         (rtu_wheel_velocity_valid == 0)) &&
        ((localDW->FixPtUnitDelay2_DSTATE_o == 15) ||
         (localDW->FixPtUnitDelay2_DSTATE_o == 1))) {
        rtb_Product1_k = localB->y;
    }

    /* Embedded MATLAB: '<S38>/divide' incorporates:
     *  Sum: '<S38>/Add'
     */
    /* Embedded MATLAB Function 'acceleration correction/calc_abs_accel/divide':
     * '<S40>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((real32_T)fabs(rtu_CycleTime)) >= 0.0001F) {
        /* '<S40>:1:5' */
        /* '<S40>:1:6' */
        rtb_Product1_k = (localB->y - rtb_Product1_k) / rtu_CycleTime;
    } else {
        /* '<S40>:1:8' */
        rtb_Product1_k = 0.0F;
    }

    /* Abs: '<S38>/Abs' */
    rtb_Divide_m = (real32_T)fabs(rtb_Product1_k);

    /* UnitDelay: '<S39>/last_accel_corr' */
    rtb_last_accel_corr = localDW->last_accel_corr_DSTATE;

    /* Embedded MATLAB: '<S39>/calc_exponent' incorporates:
     *  UnitDelay: '<S39>/last_accel_corr_step'
     */
    /* Embedded MATLAB Function 'acceleration
     * correction/peak_filter_abs_acceleration/calc_exponent': '<S48>:1' */
    /* '<S48>:1:3' */
    rtb_Product1_k = localDW->last_accel_corr_step_DSTATE;

    /* '<S48>:1:4' */
    rtb_exponent_h = 0.0F;
    if (localDW->last_accel_corr_step_DSTATE < 100.0F) {
        /* '<S48>:1:5' */
        /* '<S48>:1:6' */
        rtb_exponent_h =
            localDW->last_accel_corr_step_DSTATE / ved__wpp_accel_correct_p;
    }

    /* Embedded MATLAB: '<S39>/accel_peak_filter' incorporates:
     *  S-Function (ex_sfun_exp): '<S39>/S-Function'
     *  UnitDelay: '<S39>/last_accel_corr_max'
     */
    /* Embedded MATLAB Function 'acceleration
     * correction/peak_filter_abs_acceleration/accel_peak_filter': '<S47>:1' */
    /*  filters the abs wheel acceleration */
    /*  all acceleration peaks are positive, in case of zero crossing of the
     * acceleration in the abs signal is there is a valley */
    /*  to reduce these waves a peak filter is used */
    if (rtb_Divide_m > localDW->last_accel_corr_max_DSTATE) {
        /* '<S47>:1:11' */
        /*  a new higher peak is detected */
        /* '<S47>:1:13' */
        rtb_last_accel_corr = rtb_Divide_m;

        /* '<S47>:1:14' */
        rtb_diff = 1.0F;

        /* '<S47>:1:15' */
    } else {
        /*  slow down the correction factor from the peak, by an exp funktion */
        /*  after slow doen 100 cycles the correction factor should be zero
         * because of exp overflow */
        if (rtb_Product1_k < 100.0F) {
            /* '<S47>:1:19' */
            /* '<S47>:1:20' */
            rtb_Divide_m =
                rtb_last_accel_corr * VED_GDBexp((real32_T)rtb_exponent_h);

            /* '<S47>:1:21' */
            /* '<S47>:1:22' */
            rtb_diff = rtb_Product1_k + 1.0F;
        } else {
            /* '<S47>:1:24' */
            rtb_Divide_m = 0.0F;

            /* '<S47>:1:25' */
            /* '<S47>:1:26' */
            rtb_diff = 101.0F;
        }
    }

    /* Embedded MATLAB: '<S35>/calc_exponent' */
    /* Embedded MATLAB Function 'correct wheel uncertainty/calc_exponent':
     * '<S49>:1' */
    /* '<S49>:1:3' */
    rtb_exponent_d = 0.0F;

    /*  check for exp overflow */
    if ((localB->y / ved__wpp_P_correct_p[2]) < 100.0F) {
        /* '<S49>:1:5' */
        /* '<S49>:1:6' */
        rtb_exponent_d = localB->y / ved__wpp_P_correct_p[2];
    }

    /* Embedded MATLAB: '<S56>/eye' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/kalman_filter/eye': '<S67>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S67>:1:5' */

    /* Embedded MATLAB: '<S56>/Reset_P_pred' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/kalman_filter/Reset_P_pred': '<S65>:1'
     */
    if (((real32_T)fabs(V_pred)) <= 1.0E-16F) {
        /* '<S65>:1:4' */
        /* '<S65>:1:6' */
        for (i = 0; i < 4; i++) {
            rtb_P_pred[i] = 0.0F;
        }
    } else {
        /* '<S65>:1:10' */
    }

    /* Product: '<S56>/P_pred_(1_KH)' incorporates:
     *  Product: '<S56>/K*H'
     *  Sum: '<S56>/1_KH'
     */
    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_Product1_k = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_Product1_k +=
                    rtb_K[(i_1 << 1) + i] * ((real32_T)rtb_H[(i_0 << 1) + i_1]);
            }

            rtb_A_0[i + (i_0 << 1)] =
                ((real32_T)tmp_0[(i_0 << 1) + i]) - rtb_Product1_k;
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            localDW->P_delay_DSTATE[i + (i_0 << 1)] = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                localDW->P_delay_DSTATE[i + (i_0 << 1)] =
                    (rtb_A_0[(i_1 << 1) + i] * rtb_P_pred[(i_0 << 1) + i_1]) +
                    localDW->P_delay_DSTATE[(i_0 << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S36>/diag_P' */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/diag_P': '<S54>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S54>:1:5' */

    /* Embedded MATLAB: '<S35>/correct_estimated_ws_uncertainty' incorporates:
     *  S-Function (ex_sfun_exp): '<S35>/S-Function'
     */
    /* Embedded MATLAB Function 'correct wheel
     * uncertainty/correct_estimated_ws_uncertainty': '<S50>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /*  if the wheel velocity is available and valid */
    if (((uint32_T)rtu_wheel_velocity_valid) ==
        ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S50>:1:6' */
        /*  if the vehicle is near stand still  */
        if (((real32_T)fabs(rtu_wheel_velocity)) <= 0.001F) {
            /* '<S50>:1:8' */
            if (((uint32_T)rtu_use_pulse) != ((uint32_T)VED_IO_STATE_VALID)) {
                /* '<S50>:1:9' */
                /*  if wheel velocity is low and no wheel pulse are available */
                /* '<S50>:1:11' */
                localB->corrected_wheel_velocity_uncert =
                    localDW->P_delay_DSTATE[0] + ved__wpp_P_correct_p[0];
            } else {
                /*  if pulse are available */
                /* '<S50>:1:14' */
                rtb_Product1_k = 0.0F;

                /*  check for exp overflow */
                if ((localB->y / ved__wpp_P_correct_p[2]) < 100.0F) {
                    /* '<S50>:1:16' */
                    /* '<S50>:1:17' */
                    rtb_Product1_k = VED_GDBexp((real32_T)rtb_exponent_d);
                }

                /* '<S50>:1:19' */
                rtb_Product1_k *= ved__wpp_P_correct_p[1];

                /*  the exp value is raised at low speed */
                /* '<S50>:1:21' */
                /*  the corrected wheel veloctiy variance consitsts of the
                 * estimated wheel variance the wheel acceleration factor in
                 * case of abs */
                /*  and the exp value */
                /* '<S50>:1:24' */
                /* '<S50>:1:25' */
                localB->corrected_wheel_velocity_uncert =
                    ((rtb_Divide_m * rtb_Divide_m) +
                     localDW->P_delay_DSTATE[0]) +
                    (rtb_Product1_k * rtb_Product1_k);
            }
        } else {
            /*  if no pulse are used but velocity is below a specified value */
            if ((((uint32_T)rtu_use_pulse) != ((uint32_T)VED_IO_STATE_VALID)) &&
                (rtu_wheel_velocity <= ved__wpp_P_correct_p[3])) {
                /* '<S50>:1:29' */
                /* '<S50>:1:30' */
                /*  add constant value to wheel velocity variance
                 * (ved__wpp_P_correct_p(5)) the velocity is really not known */
                /* '<S50>:1:32' */
                localB->corrected_wheel_velocity_uncert =
                    (localDW->P_delay_DSTATE[0] + ved__wpp_P_correct_p[4]) -
                    (((localDW->P_delay_DSTATE[0] + ved__wpp_P_correct_p[4]) -
                      (((rtb_Divide_m * rtb_Divide_m) +
                        localDW->P_delay_DSTATE[0]) +
                       rtu_aqua_slip_correction)) *
                     rtu_wheel_velocity);
            } else {
                /*  best case pulse are available  */
                /* '<S50>:1:35' */
                /*  the corrected variance consists out of the estimated
                 * variance the wheel acceleration correction (ABS) */
                /*  and the aquaplaning and slip correction factor */
                /* '<S50>:1:38' */
                localB->corrected_wheel_velocity_uncert =
                    ((rtb_Divide_m * rtb_Divide_m) +
                     localDW->P_delay_DSTATE[0]) +
                    rtu_aqua_slip_correction;
            }
        }

        /*  limit variance to avoid getting very huge numbers in later ve
         * calculations */
        if (localB->corrected_wheel_velocity_uncert > ved__wpp_P_correct_p[5]) {
            /* '<S50>:1:42' */
            /* '<S50>:1:43' */
            localB->corrected_wheel_velocity_uncert = ved__wpp_P_correct_p[5];
        }
    } else {
        /*  no wheel velocity is available, set variance to a default value */
        /* '<S50>:1:47' */
        localB->corrected_wheel_velocity_uncert = ved__wpp_P_correct_p[5];
    }

    /* Embedded MATLAB: '<S36>/limit_accel' incorporates:
     *  Update for UnitDelay: '<S58>/Unit Delay'
     */
    /* Embedded MATLAB Function 'front left wheel speed
     * fusion/puls_wheel_velocity_fusion/limit_accel': '<S57>:1' */
    /* if accel is below 0.001 set to zero */
    if (((real32_T)fabs(rtb_nu[1])) < 0.001F) {
        /* '<S57>:1:3' */
        /* '<S57>:1:4' */
        localDW->UnitDelay_DSTATE = 0.0F;
    } else {
        /* '<S57>:1:6' */
        localDW->UnitDelay_DSTATE = rtb_nu[1];
    }

    /* Update for UnitDelay: '<S69>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S69>/FixPt Constant'
     */
    localDW->FixPtUnitDelay2_DSTATE = 0U;

    /* Update for UnitDelay: '<S69>/FixPt Unit Delay1' */
    for (i = 0; i < 2; i++) {
        localDW->FixPtUnitDelay1_DSTATE[(i)] = rtb_nu[i];
    }

    /* Update for UnitDelay: '<S77>/1th_Delay' */
    localDW->th_Delay_DSTATE = rtb_th_Delay;

    /* Update for UnitDelay: '<S77>/2th_Delay' */
    localDW->th_Delay_DSTATE_b = rtb_th_Delay_g;

    /* Update for UnitDelay: '<S77>/3th_Delay' */
    localDW->th_Delay_DSTATE_f = rtb_th_Delay_c;

    /* Update for UnitDelay: '<S77>/4th_Delay' */
    localDW->th_Delay_DSTATE_o = rtb_th_Delay1;

    /* Update for UnitDelay: '<S77>/1th_Delay1' */
    localDW->th_Delay1_DSTATE = rtb_th_Delay1_b;

    /* Update for UnitDelay: '<S77>/2th_Delay1' */
    localDW->th_Delay1_DSTATE_f = rtb_th_Delay1_h;

    /* Update for UnitDelay: '<S77>/3th_Delay1' */
    localDW->th_Delay1_DSTATE_j = wheel_puls_velocity_out;

    /* Update for UnitDelay: '<S37>/init_delay' */
    localDW->init_delay_DSTATE = FALSE;

    /* Update for UnitDelay: '<S37>/puls_speed_delay' */
    localDW->puls_speed_delay_DSTATE = wheel_puls_velocity_out;

    /* Update for UnitDelay: '<S37>/counter_delay' */
    localDW->counter_delay_DSTATE = counter_out;

    /* Update for UnitDelay: '<S37>/m_delay' */
    localDW->m_delay_DSTATE = m_out;

    /* Update for UnitDelay: '<S37>/single_puls_velocity_delay' */
    localDW->single_puls_velocity_delay_DSTA = rtb_single_puls_velocity_out;

    /* Update for UnitDelay: '<S74>/T3' */
    localDW->T3_DSTATE = rtb_T2;

    /* Update for UnitDelay: '<S74>/T2' */
    localDW->T2_DSTATE = rtb_T1;

    /* Update for UnitDelay: '<S74>/T1' */
    localDW->T1_DSTATE = rtb_T0;

    /* Update for UnitDelay: '<S74>/T0' */
    localDW->T0_DSTATE = localB->y;

    /* Update for UnitDelay: '<S41>/FixPt Unit Delay2' */
    localDW->FixPtUnitDelay2_DSTATE_o = rtu_wheel_velocity_valid;

    /* Update for UnitDelay: '<S41>/FixPt Unit Delay1' */
    localDW->FixPtUnitDelay1_DSTATE_e = localB->y;

    /* Update for UnitDelay: '<S39>/last_accel_corr' */
    localDW->last_accel_corr_DSTATE = rtb_Divide_m;

    /* Update for UnitDelay: '<S39>/last_accel_corr_step' */
    localDW->last_accel_corr_step_DSTATE = rtb_diff;

    /* Update for UnitDelay: '<S39>/last_accel_corr_max' */
    localDW->last_accel_corr_max_DSTATE = rtb_last_accel_corr;
}

/* Model step function */
void ved__wpp_step(BlockIO_ved__wpp *ved__wpp_B,
                   D_Work_ved__wpp *ved__wpp_DWork,
                   VED_InputData_t *ved__wpp_U_VED_InputData,
                   VED_InternalData_t *ved__wpp_U_VED_InternalData,
                   VED_InternalData_t *ved__wpp_Y_VED_InternalData_out) {
    /* local block i/o variables */
    real32_T rtb_WhlVelFrLeft;
    real32_T rtb_WhlVelFrRight;
    real32_T rtb_WhlVelReLeft;
    real32_T rtb_WhlVelReRight;
    real32_T rtb_WhlCircumference;
    real32_T rtb_front_to_rear_right;
    real32_T rtb_factor;
    real32_T rtb_rear_to_front_right;
    real32_T rtb_front_to_rear_left_c;
    real32_T rtb_factor_h;
    real32_T rtb_rear_to_front_left_p;
    real32_T rtb_corrected_front_left_value;
    real32_T rtb_corrected_front_right_value;
    real32_T rtb_corrected_rear_left_value;
    real32_T rtb_corrected_rear_right_value;
    real32_T rtb_Time2Sec;
    uint8_T rtb_State[32];
    uint8_T rtb_WhlTicksDevFrLeft;
    uint8_T rtb_WhlTicksDevFrRight;
    uint8_T rtb_WhlTicksDevReLeft;
    uint8_T rtb_WhlTicksDevReRight;
    uint8_T rtb_WhlTcksPerRev;
    uint8_T rtb_State_n[32];
    real32_T mean_velocity_left;
    boolean_T LeftReference;
    real32_T rtb_WhlVelFrLeft_l;
    real32_T rtb_WhlVelFrRight_m;
    real32_T rtb_WhlVelReLeft_a;
    real32_T rtb_WhlVelReRight_p;
    uint8_T rtb_aqua_slip_state_FL_out;
    uint8_T rtb_aqua_slip_state_FR_out;
    uint8_T rtb_aqua_slip_state_RL_out;
    uint8_T rtb_aqua_slip_state_RR_out;
    real32_T rtb_y;
    real32_T rtb_front_to_rear_left;
    real32_T rtb_rear_to_front_left;
    real32_T rtb_slip_aqua_correct_front_lef;
    int32_T rtb_slip_aqua_correct_front_l_p;
    int32_T rtb_slip_aqua_correct_rear_le_e;
    real32_T rtb_front_to_rear_right_l;
    real32_T rtb_rear_to_front_right_l;
    int32_T rtb_slip_aqua_correct_front_r_g;
    int32_T rtb_slip_aqua_correct_rear_ri_o;
    int32_T i;

    /* BusSelector: '<Root>/Bus Selector2' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlVelFrLeft = (*ved__wpp_U_VED_InputData).Signals.WhlVelFrLeft;
    rtb_WhlVelFrRight = (*ved__wpp_U_VED_InputData).Signals.WhlVelFrRight;
    rtb_WhlVelReLeft = (*ved__wpp_U_VED_InputData).Signals.WhlVelReLeft;
    rtb_WhlVelReRight = (*ved__wpp_U_VED_InputData).Signals.WhlVelReRight;

    /* Outputs for atomic SubSystem: '<Root>/Time2Sec' */

    /* Product: '<S10>/Time2Sec' incorporates:
     *  Constant: '<S10>/Constant'
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_Time2Sec =
        ((real32_T)(*ved__wpp_U_VED_InputData).Frame.CycleTime) / 1000.0F;

    /* end of Outputs for SubSystem: '<Root>/Time2Sec' */

    /* BusSelector: '<Root>/Bus Selector1' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State[i] = (*ved__wpp_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<Root>/Get_IO_State9' */
    ved__wpp_Get_IO_State10(rtb_State, &ved__wpp_B->Get_IO_State9,
                            ((uint32_T)VED_SIN_POS_WVEL_FL));

    /* end of Outputs for SubSystem: '<Root>/Get_IO_State9' */

    /* BusSelector: '<Root>/Bus Selector7' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlVelFrLeft_l = (*ved__wpp_U_VED_InputData).Signals.WhlVelFrLeft;
    rtb_WhlVelFrRight_m = (*ved__wpp_U_VED_InputData).Signals.WhlVelFrRight;
    rtb_WhlVelReLeft_a = (*ved__wpp_U_VED_InputData).Signals.WhlVelReLeft;
    rtb_WhlVelReRight_p = (*ved__wpp_U_VED_InputData).Signals.WhlVelReRight;

    /* Embedded MATLAB Function 'aqua slip correction/four_wheel_mean':
     * '<S17>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S17>:1:5' */
    rtb_y = ((((*ved__wpp_U_VED_InputData).Signals.WhlVelFrLeft +
               (*ved__wpp_U_VED_InputData).Signals.WhlVelReLeft) +
              (*ved__wpp_U_VED_InputData).Signals.WhlVelFrRight) +
             (*ved__wpp_U_VED_InputData).Signals.WhlVelReRight) *
            0.25F;

    /* Embedded MATLAB: '<S21>/left_side_aqua_slip_correction1' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    mean_velocity_left = (*ved__wpp_U_VED_InputData).Signals.WhlVelFrLeft;
    rtb_rear_to_front_left = (*ved__wpp_U_VED_InputData).Signals.WhlVelReLeft;

    /* Embedded MATLAB Function 'aqua slip
     * correction/left_side_aqua_slip_correction/calc_factor/left_side_aqua_slip_correction1':
     * '<S23>:1' */
    /*  check for diff zero */
    if (((real32_T)fabs((*ved__wpp_U_VED_InputData).Signals.WhlVelReLeft)) <
        0.0001F) {
        /* '<S23>:1:6' */
        /* '<S23>:1:7' */
        rtb_rear_to_front_left = 0.0001F;
    }

    if (((real32_T)fabs((*ved__wpp_U_VED_InputData).Signals.WhlVelFrLeft)) <
        0.0001F) {
        /* '<S23>:1:9' */
        /* '<S23>:1:10' */
        mean_velocity_left = 0.0001F;
    }

    /*  check if exponent is (fl/rl) in range of float */
    /* '<S23>:1:14' */
    rtb_front_to_rear_left_c = (mean_velocity_left / rtb_rear_to_front_left) /
                               ved__wpp_aqua_slip_correct_p[0];
    if (rtb_front_to_rear_left_c > 80.0F) {
        /* '<S23>:1:15' */
        /* '<S23>:1:16' */
        rtb_front_to_rear_left_c = 80.0F;
    }

    /*  check if exponent is (rl/fl) in range of float */
    /* '<S23>:1:20' */
    rtb_rear_to_front_left_p = (rtb_rear_to_front_left / mean_velocity_left) /
                               ved__wpp_aqua_slip_correct_p[0];
    if (rtb_rear_to_front_left_p > 80.0F) {
        /* '<S23>:1:21' */
        /* '<S23>:1:22' */
        rtb_rear_to_front_left_p = 80.0F;
    }

    /* '<S23>:1:24' */
    rtb_factor_h = 0.98F / ved__wpp_aqua_slip_correct_p[0];

    /* Embedded MATLAB: '<S21>/left_side_aqua_slip_correction2' incorporates:
     *  S-Function (ex_sfun_exp): '<S21>/S-Function'
     *  S-Function (ex_sfun_exp): '<S21>/S-Function1'
     *  S-Function (ex_sfun_exp): '<S21>/S-Function2'
     */
    /* Embedded MATLAB Function 'aqua slip
     * correction/left_side_aqua_slip_correction/calc_factor/left_side_aqua_slip_correction2':
     * '<S24>:1' */
    /* '<S24>:1:4' */
    rtb_front_to_rear_left = VED_GDBexp((real32_T)rtb_front_to_rear_left_c) -
                             VED_GDBexp((real32_T)rtb_factor_h);

    /* '<S24>:1:5' */
    rtb_rear_to_front_left = VED_GDBexp((real32_T)rtb_rear_to_front_left_p) -
                             VED_GDBexp((real32_T)rtb_factor_h);

    /* Embedded MATLAB: '<S19>/left_side_aqua_slip_correction' */
    /* Embedded MATLAB Function 'aqua slip
     * correction/left_side_aqua_slip_correction/left_side_aqua_slip_correction':
     * '<S22>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S22>:1:8' */
    mean_velocity_left = (rtb_WhlVelFrLeft_l + rtb_WhlVelReLeft_a) / 2.0F;

    /*  the trend to the front or the rear wheel speed */
    /*  left side  */
    /*  LeftReference true = front, false = rear */
    /*  Vm four wheel mean velocity */
    /*  Vl left wheel mean velocity */
    /*  R velocity rear */
    /*  F velocity front */
    /* '<S22>:1:16' */
    LeftReference = TRUE;

    /*  check the 8 cases */
    if (rtb_y < mean_velocity_left) {
        /* '<S22>:1:18' */
        if (rtb_WhlVelFrLeft_l > mean_velocity_left) {
            /* '<S22>:1:19' */
            /*  use the rear axle as reference */

            if (rtb_WhlVelReLeft_a > rtb_y) {
                /* '<S22>:1:22' */
                /* '<S22>:1:23' */
                LeftReference = FALSE;
            }

            /*  use the rear axle as reference */

            if (rtb_WhlVelReLeft_a < rtb_y) {
                /* '<S22>:1:27' */
                /* '<S22>:1:28' */
                LeftReference = FALSE;
            }
        }

        if (rtb_WhlVelReLeft_a > mean_velocity_left) {
            /* '<S22>:1:32' */
            /*  use the front axle as reference */

            if (rtb_WhlVelFrLeft_l < rtb_y) {
                /* '<S22>:1:35' */
                /* '<S22>:1:36' */
                LeftReference = TRUE;
            }

            /*  use the front axle as reference */

            if (rtb_WhlVelFrLeft_l > rtb_y) {
                /* '<S22>:1:40' */
                /* '<S22>:1:41' */
                LeftReference = TRUE;
            }
        }
    } else {
        if ((rtb_WhlVelFrLeft_l > mean_velocity_left) &&
            (rtb_WhlVelFrLeft_l > rtb_y)) {
            /* '<S22>:1:45' */
            /*  use the front axis as reference */

            /*  use the front axle as reference */

            /* '<S22>:1:53' */
            /* '<S22>:1:54' */
            LeftReference = TRUE;
        }

        if (rtb_WhlVelReLeft_a > mean_velocity_left) {
            /* '<S22>:1:58' */
            /*  use the rear axle as reference */

            if (rtb_WhlVelReLeft_a > rtb_y) {
                /* '<S22>:1:61' */
                /* '<S22>:1:62' */
                LeftReference = FALSE;
            }

            /*  use the rear axle as reference */

            if (rtb_WhlVelReLeft_a < rtb_y) {
                /* '<S22>:1:66' */
                /* '<S22>:1:67' */
                LeftReference = FALSE;
            }
        }
    }

    /*  if LeftReference == false then use the rear axle as reference */
    if (((int32_T)LeftReference) == 0) {
        /* '<S22>:1:74' */
        /*  Left front wheel slips */
        if ((rtb_WhlVelReLeft_a - rtb_WhlVelFrLeft_l) <= 0.0F) {
            /* '<S22>:1:76' */
            /* '<S22>:1:77' */
            rtb_slip_aqua_correct_front_lef = rtb_front_to_rear_left;

            /* '<S22>:1:78' */
            rtb_slip_aqua_correct_front_l_p = 2;

            /* '<S22>:1:80' */
            rtb_front_to_rear_left = rtb_rear_to_front_left;

            /* '<S22>:1:81' */
            rtb_slip_aqua_correct_rear_le_e = 1;
        } else {
            /*  left front wheel aqua */
            /* '<S22>:1:83' */
            rtb_slip_aqua_correct_front_lef = rtb_rear_to_front_left;

            /* '<S22>:1:84' */
            rtb_slip_aqua_correct_front_l_p = 0;

            /* '<S22>:1:86' */
            /* '<S22>:1:87' */
            rtb_slip_aqua_correct_rear_le_e = 1;
        }
    } else {
        /*  use the front axle as reference */
        /*  Left rear wheel slips */
        if ((rtb_WhlVelFrLeft_l - rtb_WhlVelReLeft_a) <= 0.0F) {
            /* '<S22>:1:91' */
            /* '<S22>:1:92' */
            rtb_slip_aqua_correct_front_lef = rtb_front_to_rear_left;

            /* '<S22>:1:93' */
            rtb_slip_aqua_correct_front_l_p = 1;

            /* '<S22>:1:95' */
            rtb_front_to_rear_left = rtb_rear_to_front_left;

            /* '<S22>:1:96' */
            rtb_slip_aqua_correct_rear_le_e = 2;
        } else {
            /*  left rear wheel aqua */
            /* '<S22>:1:98' */
            rtb_slip_aqua_correct_front_lef = rtb_rear_to_front_left;

            /* '<S22>:1:99' */
            rtb_slip_aqua_correct_front_l_p = 1;

            /* '<S22>:1:101' */
            /* '<S22>:1:102' */
            rtb_slip_aqua_correct_rear_le_e = 0;
        }
    }

    /* Embedded MATLAB: '<S25>/right_side_aqua_slip_correction_factors' */
    mean_velocity_left = rtb_WhlVelFrRight_m;
    rtb_rear_to_front_left = rtb_WhlVelReRight_p;

    /* Embedded MATLAB Function 'aqua slip
     * correction/right_side_aqua_slip_correction/calc_factor/right_side_aqua_slip_correction_factors':
     * '<S28>:1' */
    /*  check for diff zero */
    if (((real32_T)fabs(rtb_WhlVelReRight_p)) < 0.0001F) {
        /* '<S28>:1:6' */
        /* '<S28>:1:7' */
        rtb_rear_to_front_left = 0.0001F;
    }

    if (((real32_T)fabs(rtb_WhlVelFrRight_m)) < 0.0001F) {
        /* '<S28>:1:9' */
        /* '<S28>:1:10' */
        mean_velocity_left = 0.0001F;
    }

    /*  check if exponent is (fr/rr) in range of float */
    /* '<S28>:1:14' */
    rtb_front_to_rear_right = (mean_velocity_left / rtb_rear_to_front_left) /
                              ved__wpp_aqua_slip_correct_p[0];
    if (rtb_front_to_rear_right > 80.0F) {
        /* '<S28>:1:15' */
        /* '<S28>:1:16' */
        rtb_front_to_rear_right = 80.0F;
    }

    /*  check if exponent is (rr/fr) in range of float */
    /* '<S28>:1:20' */
    rtb_rear_to_front_right = (rtb_rear_to_front_left / mean_velocity_left) /
                              ved__wpp_aqua_slip_correct_p[0];
    if (rtb_rear_to_front_right > 80.0F) {
        /* '<S28>:1:21' */
        /* '<S28>:1:22' */
        rtb_rear_to_front_right = 80.0F;
    }

    /* '<S28>:1:24' */
    rtb_factor = 0.98F / ved__wpp_aqua_slip_correct_p[1];

    /* Embedded MATLAB: '<S25>/right_side_aqua_slip_correction_exp'
     * incorporates: S-Function (ex_sfun_exp): '<S25>/S-Function' S-Function
     * (ex_sfun_exp): '<S25>/S-Function1' S-Function (ex_sfun_exp):
     * '<S25>/S-Function2'
     */
    /* Embedded MATLAB Function 'aqua slip
     * correction/right_side_aqua_slip_correction/calc_factor/right_side_aqua_slip_correction_exp':
     * '<S27>:1' */
    /* '<S27>:1:4' */
    rtb_front_to_rear_right_l = VED_GDBexp((real32_T)rtb_front_to_rear_right) -
                                VED_GDBexp((real32_T)rtb_factor);

    /* '<S27>:1:5' */
    rtb_rear_to_front_right_l = VED_GDBexp((real32_T)rtb_rear_to_front_right) -
                                VED_GDBexp((real32_T)rtb_factor);

    /* Embedded MATLAB: '<S20>/right_side_aqua_slip_crrection_org' */
    /* Embedded MATLAB Function 'aqua slip
     * correction/right_side_aqua_slip_correction/right_side_aqua_slip_crrection_org':
     * '<S26>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S26>:1:8' */
    rtb_rear_to_front_left = (rtb_WhlVelFrRight_m + rtb_WhlVelReRight_p) / 2.0F;

    /*  the trend to the front or the rear wheel speed */
    /*  right side  */
    /*  RightReference true = front, false = rear */
    /*  Vm four wheel mean velocity */
    /*  Vr right wheel mean velocity */
    /*  R velocity rear */
    /*  F velocity front */
    /* '<S26>:1:16' */
    LeftReference = TRUE;

    /*  check the 8 cases */
    if (rtb_y < rtb_rear_to_front_left) {
        /* '<S26>:1:18' */
        if (rtb_WhlVelFrRight_m > rtb_rear_to_front_left) {
            /* '<S26>:1:19' */
            /*  use the rear axle as reference */

            if (rtb_WhlVelReRight_p > rtb_y) {
                /* '<S26>:1:22' */
                /* '<S26>:1:23' */
                LeftReference = FALSE;
            }

            /*  use the rear axle as reference */

            if (rtb_WhlVelReRight_p < rtb_y) {
                /* '<S26>:1:27' */
                /* '<S26>:1:28' */
                LeftReference = FALSE;
            }
        }

        if (rtb_WhlVelReRight_p > rtb_rear_to_front_left) {
            /* '<S26>:1:32' */
            /*  use the front axle as reference */

            if (rtb_WhlVelFrRight_m < rtb_y) {
                /* '<S26>:1:35' */
                /* '<S26>:1:36' */
                LeftReference = TRUE;
            }

            /*  use the front axle as referenece */

            if (rtb_WhlVelFrRight_m > rtb_y) {
                /* '<S26>:1:40' */
                /* '<S26>:1:41' */
                LeftReference = TRUE;
            }
        }
    } else {
        if ((rtb_WhlVelFrRight_m > rtb_rear_to_front_left) &&
            (rtb_WhlVelFrRight_m > rtb_y)) {
            /* '<S26>:1:45' */
            /*  use the front axle as reference */

            /*  use the front axle as reference */

            /* '<S26>:1:53' */
            /* '<S26>:1:54' */
            LeftReference = TRUE;
        }

        if (rtb_WhlVelReRight_p > rtb_rear_to_front_left) {
            /* '<S26>:1:58' */
            /*  use the rear axle as referenece */

            if (rtb_WhlVelReRight_p > rtb_y) {
                /* '<S26>:1:61' */
                /* '<S26>:1:62' */
                LeftReference = FALSE;
            }

            /*  use the rear axle as referenece */

            if (rtb_WhlVelReRight_p < rtb_y) {
                /* '<S26>:1:66' */
                /* '<S26>:1:67' */
                LeftReference = FALSE;
            }
        }
    }

    /*  if RightReference == false then use the rear axle as reference */
    if (((int32_T)LeftReference) == 0) {
        /* '<S26>:1:74' */
        /*  Right front wheel slips */
        if ((rtb_WhlVelReRight_p - rtb_WhlVelFrRight_m) <= 0.0F) {
            /* '<S26>:1:76' */
            /* '<S26>:1:77' */
            mean_velocity_left = rtb_front_to_rear_right_l;

            /* '<S26>:1:78' */
            rtb_slip_aqua_correct_front_r_g = 2;

            /* '<S26>:1:80' */
            rtb_front_to_rear_right_l = rtb_rear_to_front_right_l;

            /* '<S26>:1:81' */
            rtb_slip_aqua_correct_rear_ri_o = 1;
        } else {
            /*  right front wheel aqua */
            /* '<S26>:1:83' */
            mean_velocity_left = rtb_rear_to_front_right_l;

            /* '<S26>:1:84' */
            rtb_slip_aqua_correct_front_r_g = 0;

            /* '<S26>:1:86' */
            /* '<S26>:1:87' */
            rtb_slip_aqua_correct_rear_ri_o = 1;
        }
    } else {
        /*  use the front axle as reference */
        /*  Right rear wheel slips */
        if ((rtb_WhlVelFrRight_m - rtb_WhlVelReRight_p) <= 0.0F) {
            /* '<S26>:1:91' */
            /* '<S26>:1:92' */
            mean_velocity_left = rtb_front_to_rear_right_l;

            /* '<S26>:1:93' */
            rtb_slip_aqua_correct_front_r_g = 1;

            /* '<S26>:1:95' */
            rtb_front_to_rear_right_l = rtb_rear_to_front_right_l;

            /* '<S26>:1:96' */
            rtb_slip_aqua_correct_rear_ri_o = 2;
        } else {
            /*  right rear wheel aqua */
            /* '<S26>:1:98' */
            mean_velocity_left = rtb_rear_to_front_right_l;

            /* '<S26>:1:99' */
            rtb_slip_aqua_correct_front_r_g = 1;

            /* '<S26>:1:101' */
            /* '<S26>:1:102' */
            rtb_slip_aqua_correct_rear_ri_o = 0;
        }
    }

    /* Embedded MATLAB: '<S11>/left_right_correction_fusion' */
    /* Embedded MATLAB Function 'aqua slip
     * correction/left_right_correction_fusion': '<S18>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if ((((((real32_T)fabs(rtb_WhlVelFrLeft_l)) < 0.01F) ||
          (((real32_T)fabs(rtb_WhlVelFrRight_m)) < 0.01F)) ||
         (((real32_T)fabs(rtb_WhlVelReLeft_a)) < 0.01F)) ||
        (((real32_T)fabs(rtb_WhlVelReRight_p)) < 0.01F)) {
        /* '<S18>:1:22' */
        /* '<S18>:1:23' */
        rtb_corrected_front_left_value = 0.0F;

        /* '<S18>:1:24' */
        rtb_slip_aqua_correct_front_l_p = 1;

        /* '<S18>:1:25' */
        rtb_corrected_front_right_value = 0.0F;

        /* '<S18>:1:26' */
        rtb_slip_aqua_correct_front_r_g = 1;

        /* '<S18>:1:27' */
        rtb_corrected_rear_left_value = 0.0F;

        /* '<S18>:1:28' */
        rtb_slip_aqua_correct_rear_le_e = 1;

        /* '<S18>:1:29' */
        rtb_corrected_rear_right_value = 0.0F;

        /* '<S18>:1:30' */
        rtb_slip_aqua_correct_rear_ri_o = 1;
    } else {
        /* '<S18>:1:32' */
        rtb_corrected_front_left_value =
            (real32_T)fabs(rtb_slip_aqua_correct_front_lef);

        /* '<S18>:1:33' */
        /* '<S18>:1:34' */
        rtb_corrected_front_right_value = (real32_T)fabs(mean_velocity_left);

        /* '<S18>:1:35' */
        /* '<S18>:1:36' */
        rtb_corrected_rear_left_value = (real32_T)fabs(rtb_front_to_rear_left);

        /* '<S18>:1:37' */
        /* '<S18>:1:38' */
        rtb_corrected_rear_right_value =
            (real32_T)fabs(rtb_front_to_rear_right_l);

        /* '<S18>:1:39' */
    }

    /* BusSelector: '<Root>/Bus Selector9' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlTicksDevFrLeft =
        (*ved__wpp_U_VED_InputData).Signals.WhlTicksDevFrLeft;
    rtb_WhlTicksDevFrRight =
        (*ved__wpp_U_VED_InputData).Signals.WhlTicksDevFrRight;
    rtb_WhlTicksDevReLeft =
        (*ved__wpp_U_VED_InputData).Signals.WhlTicksDevReLeft;
    rtb_WhlTicksDevReRight =
        (*ved__wpp_U_VED_InputData).Signals.WhlTicksDevReRight;

    /* BusSelector: '<Root>/Bus Selector8' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlTcksPerRev =
        (*ved__wpp_U_VED_InputData).Parameter.VED_Ku_WhlTcksPerRev_nu;
    for (i = 0; i < 32; i++) {
        rtb_State_n[i] = (*ved__wpp_U_VED_InputData).Signals.State[(i)];
    }

    rtb_WhlCircumference =
        (*ved__wpp_U_VED_InputData).Parameter.VED_Kf_WhlCircumference_met;

    /* Outputs for atomic SubSystem: '<Root>/Get_IO_State5' */
    ved__wpp_Get_IO_State10(rtb_State_n, &ved__wpp_B->Get_IO_State5,
                            ((uint32_T)VED_SIN_POS_WTCKS_FL));

    /* end of Outputs for SubSystem: '<Root>/Get_IO_State5' */

    /* Outputs for atomic SubSystem: '<Root>/front_left_wheel_speed_fusion' */
    wheel_speed_fusion(rtb_WhlVelFrLeft, rtb_Time2Sec, rtb_WhlCircumference,
                       rtb_WhlTcksPerRev, ved__wpp_B->Get_IO_State9.IndexVector,
                       ved__wpp_B->Get_IO_State5.IndexVector,
                       rtb_corrected_front_left_value, rtb_WhlTicksDevFrLeft,
                       &ved__wpp_B->front_left_wheel_speed_fusion,
                       &ved__wpp_DWork->front_left_wheel_speed_fusion);

    /* end of Outputs for SubSystem: '<Root>/front_left_wheel_speed_fusion' */

    /* Outputs for atomic SubSystem: '<Root>/Get_IO_State7' */
    ved__wpp_Get_IO_State10(rtb_State, &ved__wpp_B->Get_IO_State7,
                            ((uint32_T)VED_SIN_POS_WVEL_FR));

    /* end of Outputs for SubSystem: '<Root>/Get_IO_State7' */

    /* Outputs for atomic SubSystem: '<Root>/Get_IO_State6' */
    ved__wpp_Get_IO_State10(rtb_State_n, &ved__wpp_B->Get_IO_State6,
                            ((uint32_T)VED_SIN_POS_WTCKS_FR));

    /* end of Outputs for SubSystem: '<Root>/Get_IO_State6' */

    /* Outputs for atomic SubSystem: '<Root>/front_right_wheel_speed_fusion' */
    wheel_speed_fusion(rtb_WhlVelFrRight, rtb_Time2Sec, rtb_WhlCircumference,
                       rtb_WhlTcksPerRev, ved__wpp_B->Get_IO_State7.IndexVector,
                       ved__wpp_B->Get_IO_State6.IndexVector,
                       rtb_corrected_front_right_value, rtb_WhlTicksDevFrRight,
                       &ved__wpp_B->front_right_wheel_speed_fusion,
                       &ved__wpp_DWork->front_right_wheel_speed_fusion);

    /* end of Outputs for SubSystem: '<Root>/front_right_wheel_speed_fusion' */

    /* Outputs for atomic SubSystem: '<Root>/Get_IO_State10' */
    ved__wpp_Get_IO_State10(rtb_State, &ved__wpp_B->Get_IO_State10,
                            ((uint32_T)VED_SIN_POS_WVEL_RL));

    /* end of Outputs for SubSystem: '<Root>/Get_IO_State10' */

    /* Outputs for atomic SubSystem: '<Root>/Get_IO_State8' */
    ved__wpp_Get_IO_State10(rtb_State_n, &ved__wpp_B->Get_IO_State8,
                            ((uint32_T)VED_SIN_POS_WTCKS_RL));

    /* end of Outputs for SubSystem: '<Root>/Get_IO_State8' */

    /* Outputs for atomic SubSystem: '<Root>/rear_left_wheel_speed_fusion' */
    wheel_speed_fusion(
        rtb_WhlVelReLeft, rtb_Time2Sec, rtb_WhlCircumference, rtb_WhlTcksPerRev,
        ved__wpp_B->Get_IO_State10.IndexVector,
        ved__wpp_B->Get_IO_State8.IndexVector, rtb_corrected_rear_left_value,
        rtb_WhlTicksDevReLeft, &ved__wpp_B->rear_left_wheel_speed_fusion,
        &ved__wpp_DWork->rear_left_wheel_speed_fusion);

    /* end of Outputs for SubSystem: '<Root>/rear_left_wheel_speed_fusion' */

    /* Outputs for atomic SubSystem: '<Root>/Get_IO_State11' */
    ved__wpp_Get_IO_State10(rtb_State, &ved__wpp_B->Get_IO_State11,
                            ((uint32_T)VED_SIN_POS_WVEL_RR));

    /* end of Outputs for SubSystem: '<Root>/Get_IO_State11' */

    /* Outputs for atomic SubSystem: '<Root>/Get_IO_State12' */
    ved__wpp_Get_IO_State10(rtb_State_n, &ved__wpp_B->Get_IO_State12,
                            ((uint32_T)VED_SIN_POS_WTCKS_RR));

    /* end of Outputs for SubSystem: '<Root>/Get_IO_State12' */

    /* Outputs for atomic SubSystem: '<Root>/rear_right_wheel_speed_fusion' */
    wheel_speed_fusion(
        rtb_WhlVelReRight, rtb_Time2Sec, rtb_WhlCircumference,
        rtb_WhlTcksPerRev, ved__wpp_B->Get_IO_State11.IndexVector,
        ved__wpp_B->Get_IO_State12.IndexVector, rtb_corrected_rear_right_value,
        rtb_WhlTicksDevReRight, &ved__wpp_B->rear_right_wheel_speed_fusion,
        &ved__wpp_DWork->rear_right_wheel_speed_fusion);

    /* end of Outputs for SubSystem: '<Root>/rear_right_wheel_speed_fusion' */

    /* Sum: '<S29>/Sum' incorporates:
     *  Gain: '<S29>/Gain'
     *  Sum: '<S29>/Diff'
     *  UnitDelay: '<S29>/UD'
     */
    ved__wpp_DWork->UD_DSTATE = ((ved__wpp_DWork->UD_DSTATE -
                                  ((real32_T)rtb_slip_aqua_correct_front_l_p)) *
                                 0.8F) +
                                ((real32_T)rtb_slip_aqua_correct_front_l_p);

    /* Sum: '<S30>/Sum' incorporates:
     *  Gain: '<S30>/Gain'
     *  Sum: '<S30>/Diff'
     *  UnitDelay: '<S30>/UD'
     */
    ved__wpp_DWork->UD_DSTATE_o =
        ((ved__wpp_DWork->UD_DSTATE_o -
          ((real32_T)rtb_slip_aqua_correct_front_r_g)) *
         0.8F) +
        ((real32_T)rtb_slip_aqua_correct_front_r_g);

    /* Sum: '<S31>/Sum' incorporates:
     *  Gain: '<S31>/Gain'
     *  Sum: '<S31>/Diff'
     *  UnitDelay: '<S31>/UD'
     */
    ved__wpp_DWork->UD_DSTATE_b =
        ((ved__wpp_DWork->UD_DSTATE_b -
          ((real32_T)rtb_slip_aqua_correct_rear_le_e)) *
         0.8F) +
        ((real32_T)rtb_slip_aqua_correct_rear_le_e);

    /* Sum: '<S32>/Sum' incorporates:
     *  Gain: '<S32>/Gain'
     *  Sum: '<S32>/Diff'
     *  UnitDelay: '<S32>/UD'
     */
    ved__wpp_DWork->UD_DSTATE_f =
        ((ved__wpp_DWork->UD_DSTATE_f -
          ((real32_T)rtb_slip_aqua_correct_rear_ri_o)) *
         0.8F) +
        ((real32_T)rtb_slip_aqua_correct_rear_ri_o);

    /* Embedded MATLAB: '<S12>/threshold_aqua_slip_state' */
    /* Embedded MATLAB Function
     * 'aqua_slip_state_output/threshold_aqua_slip_state': '<S33>:1' */
    /*  if the filtered aqua slip state is above 95% set the output to this
     * state */
    /* '<S33>:1:3' */
    /* '<S33>:1:4' */
    if (ved__wpp_DWork->UD_DSTATE > 1.95F) {
        /* '<S33>:1:8' */
        /* '<S33>:1:9' */
        rtb_aqua_slip_state_FL_out = 2U;
    } else if (ved__wpp_DWork->UD_DSTATE < 0.05F) {
        /* '<S33>:1:10' */
        /* '<S33>:1:11' */
        rtb_aqua_slip_state_FL_out = 0U;
    } else {
        /* '<S33>:1:13' */
        rtb_aqua_slip_state_FL_out = 1U;
    }

    if (ved__wpp_DWork->UD_DSTATE_o > 1.95F) {
        /* '<S33>:1:16' */
        /* '<S33>:1:17' */
        rtb_aqua_slip_state_FR_out = 2U;
    } else if (ved__wpp_DWork->UD_DSTATE_o < 0.05F) {
        /* '<S33>:1:18' */
        /* '<S33>:1:19' */
        rtb_aqua_slip_state_FR_out = 0U;
    } else {
        /* '<S33>:1:21' */
        rtb_aqua_slip_state_FR_out = 1U;
    }

    if (ved__wpp_DWork->UD_DSTATE_b > 1.95F) {
        /* '<S33>:1:24' */
        /* '<S33>:1:25' */
        rtb_aqua_slip_state_RL_out = 2U;
    } else if (ved__wpp_DWork->UD_DSTATE_b < 0.05F) {
        /* '<S33>:1:26' */
        /* '<S33>:1:27' */
        rtb_aqua_slip_state_RL_out = 0U;
    } else {
        /* '<S33>:1:29' */
        rtb_aqua_slip_state_RL_out = 1U;
    }

    if (ved__wpp_DWork->UD_DSTATE_f > 1.95F) {
        /* '<S33>:1:32' */
        /* '<S33>:1:33' */
        rtb_aqua_slip_state_RR_out = 2U;
    } else if (ved__wpp_DWork->UD_DSTATE_f < 0.05F) {
        /* '<S33>:1:34' */
        /* '<S33>:1:35' */
        rtb_aqua_slip_state_RR_out = 0U;
    } else {
        /* '<S33>:1:37' */
        rtb_aqua_slip_state_RR_out = 1U;
    }

    /* BusAssignment: '<Root>/BusAssign' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    (*ved__wpp_Y_VED_InternalData_out) = (*ved__wpp_U_VED_InternalData);
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.wheel_velo_front_left =
        ved__wpp_B->front_left_wheel_speed_fusion.y;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.wheel_velo_front_left_var =
        ved__wpp_B->front_left_wheel_speed_fusion
            .corrected_wheel_velocity_uncert;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.wheel_velo_front_right =
        ved__wpp_B->front_right_wheel_speed_fusion.y;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.wheel_velo_front_right_var =
        ved__wpp_B->front_right_wheel_speed_fusion
            .corrected_wheel_velocity_uncert;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.wheel_velo_rear_left =
        ved__wpp_B->rear_left_wheel_speed_fusion.y;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.wheel_velo_rear_left_var =
        ved__wpp_B->rear_left_wheel_speed_fusion
            .corrected_wheel_velocity_uncert;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.wheel_velo_rear_right =
        ved__wpp_B->rear_right_wheel_speed_fusion.y;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.wheel_velo_rear_right_var =
        ved__wpp_B->rear_right_wheel_speed_fusion
            .corrected_wheel_velocity_uncert;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.aqua_slip_state_front_left =
        rtb_aqua_slip_state_FL_out;
    (*ved__wpp_Y_VED_InternalData_out)
        .ved__wpp_out.aqua_slip_state_front_right = rtb_aqua_slip_state_FR_out;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.aqua_slip_state_rear_left =
        rtb_aqua_slip_state_RL_out;
    (*ved__wpp_Y_VED_InternalData_out).ved__wpp_out.aqua_slip_state_rear_right =
        rtb_aqua_slip_state_RR_out;
}

/* Model initialize function */
void ved__wpp_initialize(boolean_T firstTime,
                         RT_MODEL_ved__wpp *const ved__wpp_M,
                         BlockIO_ved__wpp *ved__wpp_B,
                         D_Work_ved__wpp *ved__wpp_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__wpp_M, (NULL));

    /* block I/O */
    (void)memset(((void *)ved__wpp_B), 0, sizeof(BlockIO_ved__wpp));

    /* states (dwork) */
    (void)memset((void *)ved__wpp_DWork, 0, sizeof(D_Work_ved__wpp));

    /* Start for atomic SubSystem: '<Root>/front_left_wheel_speed_fusion' */
    wheel_speed_fusion_Start(&ved__wpp_DWork->front_left_wheel_speed_fusion);

    /* end of Start for SubSystem: '<Root>/front_left_wheel_speed_fusion' */

    /* Start for atomic SubSystem: '<Root>/front_right_wheel_speed_fusion' */
    wheel_speed_fusion_Start(&ved__wpp_DWork->front_right_wheel_speed_fusion);

    /* end of Start for SubSystem: '<Root>/front_right_wheel_speed_fusion' */

    /* Start for atomic SubSystem: '<Root>/rear_left_wheel_speed_fusion' */
    wheel_speed_fusion_Start(&ved__wpp_DWork->rear_left_wheel_speed_fusion);

    /* end of Start for SubSystem: '<Root>/rear_left_wheel_speed_fusion' */

    /* Start for atomic SubSystem: '<Root>/rear_right_wheel_speed_fusion' */
    wheel_speed_fusion_Start(&ved__wpp_DWork->rear_right_wheel_speed_fusion);

    /* end of Start for SubSystem: '<Root>/rear_right_wheel_speed_fusion' */

    /* InitializeConditions for atomic SubSystem:
     * '<Root>/front_left_wheel_speed_fusion' */
    wheel_speed_fusion_Init(&ved__wpp_DWork->front_left_wheel_speed_fusion);

    /* end of InitializeConditions for SubSystem:
     * '<Root>/front_left_wheel_speed_fusion' */

    /* InitializeConditions for atomic SubSystem:
     * '<Root>/front_right_wheel_speed_fusion' */
    wheel_speed_fusion_Init(&ved__wpp_DWork->front_right_wheel_speed_fusion);

    /* end of InitializeConditions for SubSystem:
     * '<Root>/front_right_wheel_speed_fusion' */

    /* InitializeConditions for atomic SubSystem:
     * '<Root>/rear_left_wheel_speed_fusion' */
    wheel_speed_fusion_Init(&ved__wpp_DWork->rear_left_wheel_speed_fusion);

    /* end of InitializeConditions for SubSystem:
     * '<Root>/rear_left_wheel_speed_fusion' */

    /* InitializeConditions for atomic SubSystem:
     * '<Root>/rear_right_wheel_speed_fusion' */
    wheel_speed_fusion_Init(&ved__wpp_DWork->rear_right_wheel_speed_fusion);

    /* end of InitializeConditions for SubSystem:
     * '<Root>/rear_right_wheel_speed_fusion' */
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
