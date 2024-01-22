
#include "ved_consts.h"
#include "ved_ve.h"
#include "ved_ve_private.h"
#include "ved.h"
#include "ved_rt_matrixlib.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/*
 * Output and update for atomic system:
 *    '<S3>/Get_IO_State8'
 *    '<S8>/Get_IO_State1'
 *    '<S8>/Get_IO_State2'
 *    '<S8>/Get_IO_State3'
 *    '<S8>/Get_IO_State8'
 */
void ved__ve_Get_IO_State8(const uint8_T rtu_state_in[32],
                           rtB_Get_IO_State8_ved__ve *localB,
                           uint32_T rtp_Filter) {
    /* MultiPortSwitch: '<S16>/Index Vector' incorporates:
     *  Constant: '<S16>/Constant1'
     */
    localB->IndexVector = rtu_state_in[(rtp_Filter)];
}

/*
 * Output and update for atomic system:
 *    '<S36>/check_threshold'
 *    '<S37>/check_threshold'
 *    '<S38>/check_threshold'
 *    '<S39>/check_threshold'
 */
void ved__ve_check_threshold(real32_T rtu_raw_var,
                             real32_T rtu_filt_var,
                             real32_T *rty_out_var,
                             rtB_check_threshold_ved__ve *localB) {
    /* Embedded MATLAB: '<S36>/check_threshold' */
    /* Embedded MATLAB Function
     * 'make_R_matrix/check_variance_fl/check_threshold': '<S41>:1' */
    /*  If raw variance is below secified Value use the filtered var  */
    if (rtu_raw_var <= 250.0F) {
        /* '<S41>:1:3' */
        /* '<S41>:1:4' */
        (*rty_out_var) = rtu_filt_var;

        /* '<S41>:1:5' */
        localB->ext_reset = 0U;
    } else {
        /* '<S41>:1:7' */
        (*rty_out_var) = rtu_raw_var;

        /* '<S41>:1:8' */
        localB->ext_reset = 1U;
    }
}

/*
 * Output and update for atomic system:
 *    '<S42>/check_reset_condition'
 *    '<S47>/check_reset_condition'
 *    '<S52>/check_reset_condition'
 *    '<S57>/check_reset_condition'
 */
void ved__ve_check_reset_condition(real32_T rtu_u_in,
                                   uint8_T rtu_ext_reset,
                                   real32_T rtu_u_filt,
                                   rtB_check_reset_condition_ved__v *localB) {
    /* Embedded MATLAB: '<S42>/check_reset_condition' */
    /* Embedded MATLAB Function
     * 'make_R_matrix/check_variance_fl/tp_filter_peaks/check_reset_condition':
     * '<S43>:1' */
    if ((rtu_u_in > rtu_u_filt) || (rtu_ext_reset == 1)) {
        /* '<S43>:1:3' */
        /* '<S43>:1:4' */
        localB->Reset = 1U;

        /* '<S43>:1:5' */
        localB->init_value = rtu_u_in;
    } else {
        /* '<S43>:1:7' */
        localB->Reset = 0U;

        /* '<S43>:1:8' */
        localB->init_value = 0.0F;
    }
}

/* Model step function */
void ved__ve_step(BlockIO_ved__ve *ved__ve_B,
                  D_Work_ved__ve *ved__ve_DWork,
                  VED_InputData_t *ved__ve_U_VED_InputData,
                  VED_InternalData_t *ved__ve_U_VED_InternalData,
                  VED_InternalData_t *ved__ve_Y_VED_InternalData_out) {
    /* local block i/o variables */
    real32_T rtb_wheel_velo_front_left_var;
    real32_T rtb_wheel_velo_front_right_var;
    real32_T rtb_wheel_velo_rear_left_var;
    real32_T rtb_wheel_velo_rear_right_var;
    real32_T rtb_UnitDelay_h;
    real32_T rtb_Sum_n;
    real32_T rtb_UnitDelay_f;
    real32_T rtb_Sum_a;
    real32_T rtb_UnitDelay_m;
    real32_T rtb_Sum_c;
    real32_T rtb_UnitDelay_h4;
    real32_T rtb_Sum_j;
    real32_T rtb_create_vector_p[4];
    real32_T rtb_HPH_R[16];
    real32_T rtb_P_predHt[8];
    real32_T rtb_K[8];
    uint8_T rtb_State[32];
    uint8_T rtb_State_n[32];
    uint8_T rtb_UnitDelay_k;
    uint8_T rtb_UnitDelay_n;
    uint8_T rtb_UnitDelay_e;
    uint8_T rtb_UnitDelay_ef;

    /* local scratch DWork variables */
    real32_T PHt_HPHt_R_DWORK1[16];
    real32_T PHt_HPHt_R_DWORK3[16];
    real32_T PHt_HPHt_R_DWORK5[16];
    int32_T PHt_HPHt_R_DWORK2[4];
    int32_T j;
    boolean_T isodd;
    int8_T ipiv[4];
    int32_T mmj;
    int32_T jj;
    int32_T jp1j;
    int32_T jpiv_offset;
    int32_T b_c;
    int32_T c_k;
    int32_T iy;
    int32_T jA;
    real32_T rtb_create_vector[4];
    real32_T rtb_Time2Sec;
    real32_T rtb_y;
    real32_T rtb_Sum;
    real32_T rtb_gain;
    real32_T rtb_TmpSignalConversionAtSFunct[2];
    real32_T rtb_x_pred[2];
    real32_T rtb_out;
    real32_T rtb_Knu[2];
    real32_T rtb_A[4];
    int8_T rtb_H[8];
    int8_T rtb_Ht[8];
    real32_T rtb_P_pred[4];
    real32_T rtb_y_f[16];
    int32_T i;
    real32_T rtb_TmpSignalConversionAtSFun_0[2];
    real32_T rtb_Knu_0[2];
    real32_T rtb_A_0[4];
    real32_T rtb_TmpSignalConversionAtSFun_1[4];
    real32_T rtb_A_1[4];
    real32_T rtb_H_0[8];
    static int8_T tmp[8] = {1, 1, 1, 1, 0, 0, 0, 0};

    static int8_T tmp_0[4] = {1, 0, 0, 1};

    /* BusSelector: '<S11>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_create_vector[0] =
        (*ved__ve_U_VED_InternalData).ved__wpp_out.wheel_velo_front_left;
    rtb_create_vector[1] =
        (*ved__ve_U_VED_InternalData).ved__wpp_out.wheel_velo_front_right;
    rtb_create_vector[2] =
        (*ved__ve_U_VED_InternalData).ved__wpp_out.wheel_velo_rear_left;
    rtb_create_vector[3] =
        (*ved__ve_U_VED_InternalData).ved__wpp_out.wheel_velo_rear_right;

    /* Embedded MATLAB: '<S11>/four_wheel_mean' */
    /* Embedded MATLAB Function 'make_z_vektor/four_wheel_mean': '<S61>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S61>:1:5' */
    rtb_y = (((rtb_create_vector[0] + rtb_create_vector[1]) +
              rtb_create_vector[2]) +
             rtb_create_vector[3]) /
            4.0F;

    /* BusSelector: '<Root>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State[i] = (*ved__ve_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<Root>/Time2Sec' */

    /* Product: '<S4>/Time2Sec' incorporates:
     *  Constant: '<S4>/Constant'
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_Time2Sec =
        ((real32_T)(*ved__ve_U_VED_InputData).Frame.CycleTime) / 1000.0F;

    /* end of Outputs for SubSystem: '<Root>/Time2Sec' */

    /* Embedded MATLAB: '<S1>/make_A_matrix' */
    /* Embedded MATLAB Function 'A/make_A_matrix': '<S15>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S15>:1:5' */
    rtb_A[0] = 1.0F;
    rtb_A[2] = rtb_Time2Sec;
    for (i = 0; i < 2; i++) {
        rtb_A[1 + (i << 1)] = (real32_T)i;
    }

    /* UnitDelay: '<S32>/FixPt Unit Delay1' */
    for (i = 0; i < 2; i++) {
        rtb_Knu[i] = ved__ve_DWork->FixPtUnitDelay1_DSTATE[(i)];
    }

    /* Switch: '<S32>/Init' incorporates:
     *  Constant: '<S9>/x_init_a'
     *  UnitDelay: '<S32>/FixPt Unit Delay2'
     */
    if (ved__ve_DWork->FixPtUnitDelay2_DSTATE != 0) {
        rtb_Knu[0] = rtb_y;
        rtb_Knu[1] = ved__ve_x_init_p;
    }

    /* Product: '<S9>/Ax' */
    for (i = 0; i < 2; i++) {
        rtb_x_pred[i] = 0.0F;
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_x_pred[i] =
                (rtb_A[(jpiv_offset << 1) + i] * rtb_Knu[jpiv_offset]) +
                rtb_x_pred[i];
        }
    }

    /* Embedded MATLAB: '<S9>/hx' */
    /* Embedded MATLAB Function 'four_wheel_speed_fusion/hx': '<S31>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S31>:1:6' */
    for (i = 0; i < 8; i++) {
        rtb_H[i] = tmp[i];
    }

    /* '<S31>:1:11' */

    /* BusSelector: '<S3>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_n[i] = (*ved__ve_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S3>/Get_IO_State8' */
    ved__ve_Get_IO_State8(rtb_State_n, &ved__ve_B->Get_IO_State8,
                          ((uint32_T)VED_SIN_POS_VEHACL_EXT));

    /* end of Outputs for SubSystem: '<S3>/Get_IO_State8' */

    /* Sum: '<S20>/Sum' incorporates:
     *  Gain: '<S20>/Gain'
     *  Sum: '<S20>/Diff'
     *  UnitDelay: '<S20>/UD'
     *  UnitDelay: '<S7>/Unit Delay'
     */
    rtb_Sum =
        ((ved__ve_DWork->UD_DSTATE - ved__ve_DWork->UnitDelay_DSTATE) * 0.7F) +
        ved__ve_DWork->UnitDelay_DSTATE;

    /* Embedded MATLAB: '<S7>/gain_ramp' incorporates:
     *  Abs: '<S7>/Abs'
     *  Sum: '<S7>/Add'
     */
    /* Embedded MATLAB Function 'calc_diff_q_gain/gain_ramp': '<S21>:1' */
    /*  zero crossing detection */
    /*  if the veh velocity is below zero set it to specified value */
    /* '<S21>:1:4' */
    rtb_out = (((real32_T)fabs(rtb_y - rtb_Sum)) * ved__ve_Q_gain_p[2]) + 1.0F;

    /* Outputs for atomic SubSystem: '<S8>/Get_IO_State8' */
    ved__ve_Get_IO_State8(rtb_State, &ved__ve_B->Get_IO_State8_i,
                          ((uint32_T)VED_SIN_POS_WTCKS_FL));

    /* end of Outputs for SubSystem: '<S8>/Get_IO_State8' */

    /* Outputs for atomic SubSystem: '<S8>/Get_IO_State1' */
    ved__ve_Get_IO_State8(rtb_State, &ved__ve_B->Get_IO_State1,
                          ((uint32_T)VED_SIN_POS_WTCKS_FR));

    /* end of Outputs for SubSystem: '<S8>/Get_IO_State1' */

    /* Outputs for atomic SubSystem: '<S8>/Get_IO_State2' */
    ved__ve_Get_IO_State8(rtb_State, &ved__ve_B->Get_IO_State2,
                          ((uint32_T)VED_SIN_POS_WTCKS_RL));

    /* end of Outputs for SubSystem: '<S8>/Get_IO_State2' */

    /* Outputs for atomic SubSystem: '<S8>/Get_IO_State3' */
    ved__ve_Get_IO_State8(rtb_State, &ved__ve_B->Get_IO_State3,
                          ((uint32_T)VED_SIN_POS_WTCKS_RR));

    /* end of Outputs for SubSystem: '<S8>/Get_IO_State3' */

    /* Embedded MATLAB: '<Root>/calc_Q_gain' incorporates:
     *  Logic: '<S8>/Logical Operator'
     */
    /* Embedded MATLAB Function 'calc_Q_gain': '<S6>:1' */
    /* '<S6>:1:2' */
    rtb_gain = rtb_out;

    /*  if the mean velocity is below a specified value 0.4 */
    /*  the model variance gain is raised */
    if (((real32_T)fabs(rtb_y)) <= ved__ve_Q_gain_p[0]) {
        /* '<S6>:1:6' */
        /* '<S6>:1:7' */
        rtb_gain =
            (real32_T)fabs((ved__ve_Q_gain_p[0] - rtb_y) * ved__ve_Q_gain_p[1]);
    }

    /*  if no wheel puls counters are available */
    if ((((uint32_T)((((ved__ve_B->Get_IO_State8_i.IndexVector != 0) &&
                       (ved__ve_B->Get_IO_State1.IndexVector != 0)) &&
                      (ved__ve_B->Get_IO_State2.IndexVector != 0)) &&
                     (ved__ve_B->Get_IO_State3.IndexVector != 0))) !=
         ((uint32_T)VED_IO_STATE_VALID)) &&
        (((real32_T)fabs(rtb_y)) <= 0.7F)) {
        /* '<S6>:1:11' */
        /*  from v=0 ->g=14 to v=0.7 ->g=0 add diff_q_gain */
        /* '<S6>:1:13' */
        rtb_gain = (14.0F - ((real32_T)fabs(rtb_y * -20.0F))) + rtb_out;
    }

    /* Product: '<S3>/Product' incorporates:
     *  Constant: '<S3>/sigma_model'
     */
    for (i = 0; i < 2; i++) {
        rtb_Knu[i] = ved__ve_Q_sigmas_p[(i)] * rtb_gain;
    }

    /* Embedded MATLAB: '<S3>/Increase_Q2' incorporates:
     *  Constant: '<S3>/Constant1'
     *  Inport: '<Root>/VED_InputData'
     *  Product: '<S3>/Product3'
     */
    /* Embedded MATLAB Function 'Q/Increase_Q2': '<S17>:1' */
    /* % Increase of Q for Dynamic Accelaration */
    if ((((real32_T)fabs((*ved__ve_U_VED_InputData).Signals.VehLongAccelExt)) >
         0.5F) &&
        (((uint32_T)ved__ve_B->Get_IO_State8.IndexVector) ==
         ((uint32_T)VED_IO_STATE_VALID))) {
        /* '<S17>:1:3' */
        /* '<S17>:1:5' */
        for (i = 0; i < 2; i++) {
            rtb_Knu[i] = 35.0F * rtb_Knu[i];
        }
    } else {
        /* '<S17>:1:9' */
    }

    rtb_TmpSignalConversionAtSFunct[0] = (rtb_Time2Sec * rtb_Time2Sec) * 0.5F;
    rtb_TmpSignalConversionAtSFunct[1] = rtb_Time2Sec;

    /* Embedded MATLAB: '<S3>/makeQ' */
    /* Embedded MATLAB Function 'Q/makeQ': '<S18>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S18>:1:5' */

    /* Sum: '<S9>/APA_Q' incorporates:
     *  Math: '<S9>/At'
     *  Product: '<S9>/APAt'
     *  UnitDelay: '<S9>/P_delay'
     */
    for (i = 0; i < 2; i++) {
        rtb_TmpSignalConversionAtSFun_0[i] =
            rtb_TmpSignalConversionAtSFunct[i] * rtb_Knu[i];
        rtb_Knu_0[i] = rtb_Knu[i] * rtb_TmpSignalConversionAtSFunct[i];
    }

    for (i = 0; i < 2; i++) {
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_A_0[i + (jpiv_offset << 1)] = 0.0F;
            for (iy = 0; iy < 2; iy++) {
                rtb_A_0[i + (jpiv_offset << 1)] =
                    (rtb_A[(iy << 1) + i] *
                     ved__ve_DWork->P_delay_DSTATE[(jpiv_offset << 1) + iy]) +
                    rtb_A_0[(jpiv_offset << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_TmpSignalConversionAtSFun_1[jpiv_offset + (i << 1)] =
                rtb_TmpSignalConversionAtSFun_0[jpiv_offset] * rtb_Knu_0[i];
        }
    }

    for (i = 0; i < 2; i++) {
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_A_1[i + (jpiv_offset << 1)] = 0.0F;
            for (iy = 0; iy < 2; iy++) {
                rtb_A_1[i + (jpiv_offset << 1)] =
                    (rtb_A_0[(iy << 1) + i] * rtb_A[(iy << 1) + jpiv_offset]) +
                    rtb_A_1[(jpiv_offset << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_P_pred[jpiv_offset + (i << 1)] =
                rtb_TmpSignalConversionAtSFun_1[(i << 1) + jpiv_offset] +
                rtb_A_1[(i << 1) + jpiv_offset];
        }
    }

    /* Math: '<S9>/Ht' */
    for (i = 0; i < 4; i++) {
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_Ht[jpiv_offset + (i << 1)] = rtb_H[(jpiv_offset << 2) + i];
        }
    }

    /* BusSelector: '<S10>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_wheel_velo_front_left_var =
        (*ved__ve_U_VED_InternalData).ved__wpp_out.wheel_velo_front_left_var;
    rtb_wheel_velo_front_right_var =
        (*ved__ve_U_VED_InternalData).ved__wpp_out.wheel_velo_front_right_var;
    rtb_wheel_velo_rear_left_var =
        (*ved__ve_U_VED_InternalData).ved__wpp_out.wheel_velo_rear_left_var;
    rtb_wheel_velo_rear_right_var =
        (*ved__ve_U_VED_InternalData).ved__wpp_out.wheel_velo_rear_right_var;

    /* UnitDelay: '<S36>/Unit Delay' */
    rtb_UnitDelay_k = ved__ve_DWork->UnitDelay_DSTATE_pd;

    /* UnitDelay: '<S42>/Unit Delay' */
    rtb_UnitDelay_h = ved__ve_DWork->UnitDelay_DSTATE_l;

    /* Embedded MATLAB: '<S42>/check_reset_condition' */
    ved__ve_check_reset_condition(rtb_wheel_velo_front_left_var,
                                  rtb_UnitDelay_k, rtb_UnitDelay_h,
                                  &ved__ve_B->sf_check_reset_condition);

    /* Switch: '<S45>/Init' incorporates:
     *  Logic: '<S45>/FixPt Logical Operator'
     *  UnitDelay: '<S45>/FixPt Unit Delay1'
     *  UnitDelay: '<S45>/FixPt Unit Delay2'
     */
    if ((ved__ve_B->sf_check_reset_condition.Reset != 0) ||
        (ved__ve_DWork->FixPtUnitDelay2_DSTATE_l != 0)) {
        rtb_gain = ved__ve_B->sf_check_reset_condition.init_value;
    } else {
        rtb_gain = ved__ve_DWork->FixPtUnitDelay1_DSTATE_o;
    }

    /* Sum: '<S44>/Sum' incorporates:
     *  Gain: '<S44>/Gain'
     *  Sum: '<S44>/Diff'
     */
    rtb_Sum_n = ((rtb_gain - rtb_wheel_velo_front_left_var) * 0.8F) +
                rtb_wheel_velo_front_left_var;

    /* Embedded MATLAB: '<S36>/check_threshold' */
    ved__ve_check_threshold(rtb_wheel_velo_front_left_var, rtb_Sum_n,
                            &rtb_create_vector_p[0],
                            &ved__ve_B->sf_check_threshold);

    /* UnitDelay: '<S37>/Unit Delay' */
    rtb_UnitDelay_n = ved__ve_DWork->UnitDelay_DSTATE_n;

    /* UnitDelay: '<S47>/Unit Delay' */
    rtb_UnitDelay_f = ved__ve_DWork->UnitDelay_DSTATE_j;

    /* Embedded MATLAB: '<S47>/check_reset_condition' */
    ved__ve_check_reset_condition(rtb_wheel_velo_front_right_var,
                                  rtb_UnitDelay_n, rtb_UnitDelay_f,
                                  &ved__ve_B->sf_check_reset_condition_m);

    /* Switch: '<S50>/Init' incorporates:
     *  Logic: '<S50>/FixPt Logical Operator'
     *  UnitDelay: '<S50>/FixPt Unit Delay1'
     *  UnitDelay: '<S50>/FixPt Unit Delay2'
     */
    if ((ved__ve_B->sf_check_reset_condition_m.Reset != 0) ||
        (ved__ve_DWork->FixPtUnitDelay2_DSTATE_n != 0)) {
        rtb_gain = ved__ve_B->sf_check_reset_condition_m.init_value;
    } else {
        rtb_gain = ved__ve_DWork->FixPtUnitDelay1_DSTATE_n;
    }

    /* Sum: '<S49>/Sum' incorporates:
     *  Gain: '<S49>/Gain'
     *  Sum: '<S49>/Diff'
     */
    rtb_Sum_a = ((rtb_gain - rtb_wheel_velo_front_right_var) * 0.8F) +
                rtb_wheel_velo_front_right_var;

    /* Embedded MATLAB: '<S37>/check_threshold' */
    ved__ve_check_threshold(rtb_wheel_velo_front_right_var, rtb_Sum_a,
                            &rtb_create_vector_p[1],
                            &ved__ve_B->sf_check_threshold_m);

    /* UnitDelay: '<S38>/Unit Delay' */
    rtb_UnitDelay_e = ved__ve_DWork->UnitDelay_DSTATE_m;

    /* UnitDelay: '<S52>/Unit Delay' */
    rtb_UnitDelay_m = ved__ve_DWork->UnitDelay_DSTATE_p;

    /* Embedded MATLAB: '<S52>/check_reset_condition' */
    ved__ve_check_reset_condition(rtb_wheel_velo_rear_left_var, rtb_UnitDelay_e,
                                  rtb_UnitDelay_m,
                                  &ved__ve_B->sf_check_reset_condition_f);

    /* Switch: '<S55>/Init' incorporates:
     *  Logic: '<S55>/FixPt Logical Operator'
     *  UnitDelay: '<S55>/FixPt Unit Delay1'
     *  UnitDelay: '<S55>/FixPt Unit Delay2'
     */
    if ((ved__ve_B->sf_check_reset_condition_f.Reset != 0) ||
        (ved__ve_DWork->FixPtUnitDelay2_DSTATE_h != 0)) {
        rtb_gain = ved__ve_B->sf_check_reset_condition_f.init_value;
    } else {
        rtb_gain = ved__ve_DWork->FixPtUnitDelay1_DSTATE_f;
    }

    /* Sum: '<S54>/Sum' incorporates:
     *  Gain: '<S54>/Gain'
     *  Sum: '<S54>/Diff'
     */
    rtb_Sum_c = ((rtb_gain - rtb_wheel_velo_rear_left_var) * 0.8F) +
                rtb_wheel_velo_rear_left_var;

    /* Embedded MATLAB: '<S38>/check_threshold' */
    ved__ve_check_threshold(rtb_wheel_velo_rear_left_var, rtb_Sum_c,
                            &rtb_create_vector_p[2],
                            &ved__ve_B->sf_check_threshold_b);

    /* UnitDelay: '<S39>/Unit Delay' */
    rtb_UnitDelay_ef = ved__ve_DWork->UnitDelay_DSTATE_k;

    /* UnitDelay: '<S57>/Unit Delay' */
    rtb_UnitDelay_h4 = ved__ve_DWork->UnitDelay_DSTATE_o;

    /* Embedded MATLAB: '<S57>/check_reset_condition' */
    ved__ve_check_reset_condition(rtb_wheel_velo_rear_right_var,
                                  rtb_UnitDelay_ef, rtb_UnitDelay_h4,
                                  &ved__ve_B->sf_check_reset_condition_g);

    /* Switch: '<S60>/Init' incorporates:
     *  Logic: '<S60>/FixPt Logical Operator'
     *  UnitDelay: '<S60>/FixPt Unit Delay1'
     *  UnitDelay: '<S60>/FixPt Unit Delay2'
     */
    if ((ved__ve_B->sf_check_reset_condition_g.Reset != 0) ||
        (ved__ve_DWork->FixPtUnitDelay2_DSTATE_ln != 0)) {
        rtb_gain = ved__ve_B->sf_check_reset_condition_g.init_value;
    } else {
        rtb_gain = ved__ve_DWork->FixPtUnitDelay1_DSTATE_ox;
    }

    /* Sum: '<S59>/Sum' incorporates:
     *  Gain: '<S59>/Gain'
     *  Sum: '<S59>/Diff'
     */
    rtb_Sum_j = ((rtb_gain - rtb_wheel_velo_rear_right_var) * 0.8F) +
                rtb_wheel_velo_rear_right_var;

    /* Embedded MATLAB: '<S39>/check_threshold' */
    ved__ve_check_threshold(rtb_wheel_velo_rear_right_var, rtb_Sum_j,
                            &rtb_create_vector_p[3],
                            &ved__ve_B->sf_check_threshold_be);

    /* Embedded MATLAB: '<S10>/diag' */
    /* Embedded MATLAB Function 'make_R_matrix/diag': '<S40>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S40>:1:5' */
    memset((void *)(&rtb_y_f[0]), (int32_T)0.0F, sizeof(real32_T) << 4U);
    for (j = 0; j < 4; j++) {
        rtb_y_f[j + (j << 2)] = rtb_create_vector_p[j];
    }

    /* Sum: '<S9>/HPH_R' incorporates:
     *  Product: '<S9>/HPHt'
     */
    for (i = 0; i < 4; i++) {
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_H_0[i + (jpiv_offset << 2)] = 0.0F;
            for (iy = 0; iy < 2; iy++) {
                rtb_H_0[i + (jpiv_offset << 2)] =
                    (((real32_T)rtb_H[(iy << 2) + i]) *
                     rtb_P_pred[(jpiv_offset << 1) + iy]) +
                    rtb_H_0[(jpiv_offset << 2) + i];
            }
        }
    }

    for (i = 0; i < 4; i++) {
        for (jpiv_offset = 0; jpiv_offset < 4; jpiv_offset++) {
            rtb_gain = 0.0F;
            for (iy = 0; iy < 2; iy++) {
                rtb_gain += rtb_H_0[(iy << 2) + i] *
                            ((real32_T)rtb_Ht[(jpiv_offset << 1) + iy]);
            }

            rtb_HPH_R[i + (jpiv_offset << 2)] =
                rtb_y_f[(jpiv_offset << 2) + i] + rtb_gain;
        }
    }

    /* Embedded MATLAB: '<S26>/calculate determinant' */
    /* Embedded MATLAB Function 'four_wheel_speed_fusion/Calculate Kalman gain
     * PHt_(HPHt_R)/calculate determinant': '<S33>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /* '<S33>:1:3' */
    memcpy((void *)(&rtb_y_f[0]), (void *)(&rtb_HPH_R[0]),
           sizeof(real32_T) << 4U);
    for (i = 0; i < 4; i++) {
        ipiv[i] = (int8_T)(1 + i);
    }

    for (j = 0; j < 3; j++) {
        mmj = 3 - j;
        jj = (j * 5) + 1;
        jp1j = jj + 1;
        jpiv_offset = mmj + 1;
        iy = 1;
        i = jj;
        rtb_gain = (real32_T)fabs(rtb_y_f[jj - 1]);
        for (c_k = 2; c_k <= jpiv_offset; c_k++) {
            i++;
            rtb_Time2Sec = (real32_T)fabs(rtb_y_f[i - 1]);
            if (rtb_Time2Sec > rtb_gain) {
                iy = c_k;
                rtb_gain = rtb_Time2Sec;
            }
        }

        jpiv_offset = iy - 1;
        if (((real_T)rtb_y_f[(jj + jpiv_offset) - 1]) != 0.0) {
            if (jpiv_offset != 0) {
                ipiv[j] = (int8_T)((j + 1) + jpiv_offset);
                i = 1 + j;
                iy = i + jpiv_offset;
                for (jpiv_offset = 0; jpiv_offset < 4; jpiv_offset++) {
                    rtb_gain = rtb_y_f[i - 1];
                    rtb_y_f[i - 1] = rtb_y_f[iy - 1];
                    rtb_y_f[iy - 1] = rtb_gain;
                    i += 4;
                    iy += 4;
                }
            }

            iy = (mmj - 1) + jp1j;
            for (i = jp1j; i <= iy; i++) {
                rtb_y_f[i - 1] = rtb_y_f[i - 1] / rtb_y_f[jj - 1];
            }
        }

        b_c = 3 - j;
        jA = jj + 4;
        jpiv_offset = jj + 4;
        for (iy = 1; iy <= b_c; iy++) {
            if (((real_T)rtb_y_f[jpiv_offset - 1]) != 0.0) {
                rtb_gain = rtb_y_f[jpiv_offset - 1] * -1.0F;
                i = jp1j;
                c_k = mmj + jA;
                for (jj = 1 + jA; jj <= c_k; jj++) {
                    rtb_y_f[jj - 1] =
                        (rtb_y_f[i - 1] * rtb_gain) + rtb_y_f[jj - 1];
                    i++;
                }
            }

            jpiv_offset += 4;
            jA += 4;
        }
    }

    rtb_Time2Sec = rtb_y_f[0];
    for (jpiv_offset = 2; jpiv_offset < 5; jpiv_offset++) {
        rtb_Time2Sec *= rtb_y_f[((jpiv_offset - 1) << 2) + (jpiv_offset - 1)];
    }

    isodd = FALSE;
    for (jpiv_offset = 0; jpiv_offset < 3; jpiv_offset++) {
        if (ipiv[jpiv_offset] > (jpiv_offset + 1)) {
            isodd = !isodd;
        }
    }

    if (isodd) {
        rtb_Time2Sec = -rtb_Time2Sec;
    }

    /* Embedded MATLAB: '<S9>/Reset_x_pred' */
    /* Embedded MATLAB Function 'four_wheel_speed_fusion/Reset_x_pred':
     * '<S28>:1' */
    if (((real32_T)fabs(rtb_Time2Sec)) <= 1.0E-16F) {
        /* '<S28>:1:4' */
        /* '<S28>:1:6' */
        rtb_Knu[0] = rtb_y;
        rtb_Knu[1] = 0.0F;
    } else {
        /* '<S28>:1:10' */
        for (i = 0; i < 2; i++) {
            rtb_Knu[i] = rtb_x_pred[i];
        }
    }

    /* Product: '<S9>/P_pred*Ht' */
    for (i = 0; i < 2; i++) {
        for (jpiv_offset = 0; jpiv_offset < 4; jpiv_offset++) {
            rtb_P_predHt[i + (jpiv_offset << 1)] = 0.0F;
            for (iy = 0; iy < 2; iy++) {
                rtb_P_predHt[i + (jpiv_offset << 1)] =
                    (rtb_P_pred[(iy << 1) + i] *
                     ((real32_T)rtb_Ht[(jpiv_offset << 1) + iy])) +
                    rtb_P_predHt[(jpiv_offset << 1) + i];
            }
        }
    }

    /* If: '<S26>/If' incorporates:
     *  ActionPort: '<S34>/Action Port'
     *  ActionPort: '<S35>/Action Port'
     *  SubSystem: '<S26>/calculate the gain'
     *  SubSystem: '<S26>/set gain to default value'
     */
    if (rtb_Time2Sec > 1.0E-16F) {
        /* Product: '<S34>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {4, 4, 4, 2, 4, 4};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5[0], rtb_HPH_R,
                            &ved__ve_DWork->PHt_HPHt_R_DWORK4[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1[0],
                            &PHt_HPHt_R_DWORK2[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K, rtb_P_predHt, &PHt_HPHt_R_DWORK5[0],
                             &dims[3]);
        }
    } else {
        /* Constant: '<S35>/Constant' */
        for (i = 0; i < 8; i++) {
            rtb_K[i] = ved__ve_ConstP.Constant_Value_a[(i)];
        }
    }

    /* Sum: '<S9>/x_Knu' incorporates:
     *  Product: '<S9>/Knu'
     *  Sum: '<S9>/z_Hx'
     */
    for (i = 0; i < 4; i++) {
        rtb_A[i] = 0.0F;
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_A[i] = (((real32_T)tmp[(jpiv_offset << 2) + i]) *
                        rtb_x_pred[jpiv_offset]) +
                       rtb_A[i];
        }

        rtb_A_0[i] = rtb_create_vector[i] - rtb_A[i];
    }

    for (i = 0; i < 2; i++) {
        rtb_gain = 0.0F;
        for (jpiv_offset = 0; jpiv_offset < 4; jpiv_offset++) {
            rtb_gain += rtb_K[(jpiv_offset << 1) + i] * rtb_A_0[jpiv_offset];
        }

        ved__ve_DWork->FixPtUnitDelay1_DSTATE[(i)] = rtb_Knu[i] + rtb_gain;
    }

    /* Embedded MATLAB: '<Root>/zero_correction_v' */
    /* Embedded MATLAB Function 'zero_correction_v': '<S14>:1' */
    /*  zero crossing detection */
    /*  if the veh velocity is below zero set is to specified value */
    if (((ved__ve_DWork->FixPtUnitDelay1_DSTATE[0] <= ved__ve_a_v_zero_p[0]) ||
         (((real32_T)fabs(ved__ve_DWork->FixPtUnitDelay1_DSTATE[0])) <
          ved__ve_a_v_zero_p[3])) ||
        (((real32_T)fabs(rtb_y)) < ved__ve_a_v_zero_p[3])) {
        /* '<S14>:1:4' */
        /* '<S14>:1:5' */
        ved__ve_DWork->UnitDelay_DSTATE = ved__ve_a_v_zero_p[6];
    } else {
        /* '<S14>:1:7' */
        ved__ve_DWork->UnitDelay_DSTATE =
            ved__ve_DWork->FixPtUnitDelay1_DSTATE[0];
    }

    /* BusAssignment: '<Root>/Bus Assignment' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    (*ved__ve_Y_VED_InternalData_out) = (*ved__ve_U_VED_InternalData);
    (*ved__ve_Y_VED_InternalData_out).ved__ve_out.veh_velo =
        ved__ve_DWork->UnitDelay_DSTATE;

    /* Embedded MATLAB: '<Root>/zero_correcction_a' */
    /* Embedded MATLAB Function 'zero_correcction_a': '<S13>:1' */
    /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition below
     * is rearranged */
    /*  zero crossing detection */
    /*  if the veh acceleration is below zero set is to specified value */
    if ((((real32_T)fabs(ved__ve_DWork->FixPtUnitDelay1_DSTATE[1])) <
         ved__ve_a_v_zero_p[4]) &&
        (ved__ve_DWork->UnitDelay_DSTATE == ved__ve_a_v_zero_p[1])) {
        /* '<S13>:1:6' */
        /* '<S13>:1:7' */
        rtb_gain = ved__ve_a_v_zero_p[7];
    } else {
        /* '<S13>:1:9' */
        rtb_gain = ved__ve_DWork->FixPtUnitDelay1_DSTATE[1];
    }

    /* BusAssignment: '<Root>/Bus Assignment1' */
    (*ved__ve_Y_VED_InternalData_out).ved__ve_out.veh_accel = rtb_gain;

    /* Embedded MATLAB: '<S9>/eye' */
    /* Embedded MATLAB Function 'four_wheel_speed_fusion/eye': '<S30>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S30>:1:5' */

    /* Embedded MATLAB: '<S9>/Reset_P_pred' */
    /* Embedded MATLAB Function 'four_wheel_speed_fusion/Reset_P_pred':
     * '<S27>:1' */
    if (((real32_T)fabs(rtb_Time2Sec)) <= 1.0E-16F) {
        /* '<S27>:1:4' */
        /* '<S27>:1:6' */
        for (i = 0; i < 4; i++) {
            rtb_P_pred[i] = 0.0F;
        }
    } else {
        /* '<S27>:1:10' */
    }

    /* Product: '<S9>/P_pred_(1_KH)' incorporates:
     *  Product: '<S9>/K*H'
     *  Sum: '<S9>/1_KH'
     */
    for (i = 0; i < 2; i++) {
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            rtb_gain = 0.0F;
            for (iy = 0; iy < 4; iy++) {
                rtb_gain += rtb_K[(iy << 1) + i] *
                            ((real32_T)rtb_H[(jpiv_offset << 2) + iy]);
            }

            rtb_A[i + (jpiv_offset << 1)] =
                ((real32_T)tmp_0[(jpiv_offset << 1) + i]) - rtb_gain;
        }
    }

    for (i = 0; i < 2; i++) {
        for (jpiv_offset = 0; jpiv_offset < 2; jpiv_offset++) {
            ved__ve_DWork->P_delay_DSTATE[i + (jpiv_offset << 1)] = 0.0F;
            for (iy = 0; iy < 2; iy++) {
                ved__ve_DWork->P_delay_DSTATE[i + (jpiv_offset << 1)] =
                    (rtb_A[(iy << 1) + i] *
                     rtb_P_pred[(jpiv_offset << 1) + iy]) +
                    ved__ve_DWork->P_delay_DSTATE[(jpiv_offset << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S9>/diag' */
    /* Embedded MATLAB Function 'four_wheel_speed_fusion/diag': '<S29>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S29>:1:5' */

    /* Embedded MATLAB: '<S12>/v_var_correct' incorporates:
     *  UnitDelay: '<S12>/last_v_var'
     */
    /* Embedded MATLAB Function 'v_var_correct/v_var_correct': '<S62>:1' */
    /* check if varianc is negative */
    if (!(ved__ve_DWork->P_delay_DSTATE[0] <= 0.0F)) {
        /*  if the veh velocity is blow the specified value set the velocity
         * variance to a fix value */
        if ((ved__ve_DWork->UnitDelay_DSTATE <= ved__ve_a_v_zero_p[2]) &&
            (ved__ve_DWork->P_delay_DSTATE[0] >= ved__ve_a_v_zero_p[5])) {
            /* '<S62>:1:7' */
            /* '<S62>:1:8' */
            ved__ve_DWork->last_v_var_DSTATE = ved__ve_a_v_zero_p[5];
        } else {
            /* '<S62>:1:10' */
            ved__ve_DWork->last_v_var_DSTATE = ved__ve_DWork->P_delay_DSTATE[0];
        }
    } else {
        /* '<S62>:1:3' */
        /* '<S62>:1:4' */
    }

    /* BusAssignment: '<Root>/Bus Assignment2' */
    (*ved__ve_Y_VED_InternalData_out).ved__ve_out.veh_velo_var =
        ved__ve_DWork->last_v_var_DSTATE;

    /* Embedded MATLAB: '<S5>/a_var_correct' incorporates:
     *  UnitDelay: '<S5>/last_a_var'
     */
    /* Embedded MATLAB Function 'a_var_correct/a_var_correct': '<S19>:1' */
    /* check if varianc is negative */
    if (!(ved__ve_DWork->P_delay_DSTATE[3] <= 0.0F)) {
        /*  if the veh velocity is blow the specified value set the velocity
         * variance to a fix value */
        if (ved__ve_DWork->UnitDelay_DSTATE <= ved__ve_a_v_zero_p[2]) {
            /* '<S19>:1:7' */
            /* '<S19>:1:8' */
            ved__ve_DWork->last_a_var_DSTATE = ved__ve_a_v_zero_p[8];
        } else {
            /* '<S19>:1:10' */
            ved__ve_DWork->last_a_var_DSTATE = ved__ve_DWork->P_delay_DSTATE[3];
        }
    } else {
        /* '<S19>:1:3' */
        /* '<S19>:1:4' */
    }

    /* BusAssignment: '<Root>/veh_velo' */
    (*ved__ve_Y_VED_InternalData_out).ved__ve_out.veh_accel_var =
        ved__ve_DWork->last_a_var_DSTATE;

    /* Switch: '<S45>/Reset' incorporates:
     *  Update for UnitDelay: '<S45>/FixPt Unit Delay1'
     */
    if (ved__ve_B->sf_check_reset_condition.Reset != 0) {
        ved__ve_DWork->FixPtUnitDelay1_DSTATE_o =
            ved__ve_B->sf_check_reset_condition.init_value;
    } else {
        ved__ve_DWork->FixPtUnitDelay1_DSTATE_o = rtb_Sum_n;
    }

    /* Switch: '<S50>/Reset' incorporates:
     *  Update for UnitDelay: '<S50>/FixPt Unit Delay1'
     */
    if (ved__ve_B->sf_check_reset_condition_m.Reset != 0) {
        ved__ve_DWork->FixPtUnitDelay1_DSTATE_n =
            ved__ve_B->sf_check_reset_condition_m.init_value;
    } else {
        ved__ve_DWork->FixPtUnitDelay1_DSTATE_n = rtb_Sum_a;
    }

    /* Switch: '<S55>/Reset' incorporates:
     *  Update for UnitDelay: '<S55>/FixPt Unit Delay1'
     */
    if (ved__ve_B->sf_check_reset_condition_f.Reset != 0) {
        ved__ve_DWork->FixPtUnitDelay1_DSTATE_f =
            ved__ve_B->sf_check_reset_condition_f.init_value;
    } else {
        ved__ve_DWork->FixPtUnitDelay1_DSTATE_f = rtb_Sum_c;
    }

    /* Switch: '<S60>/Reset' incorporates:
     *  Update for UnitDelay: '<S60>/FixPt Unit Delay1'
     */
    if (ved__ve_B->sf_check_reset_condition_g.Reset != 0) {
        ved__ve_DWork->FixPtUnitDelay1_DSTATE_ox =
            ved__ve_B->sf_check_reset_condition_g.init_value;
    } else {
        ved__ve_DWork->FixPtUnitDelay1_DSTATE_ox = rtb_Sum_j;
    }

    /* Update for UnitDelay: '<S32>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S32>/FixPt Constant'
     */
    ved__ve_DWork->FixPtUnitDelay2_DSTATE = 0U;

    /* Update for UnitDelay: '<S20>/UD' */
    ved__ve_DWork->UD_DSTATE = rtb_Sum;

    /* Update for UnitDelay: '<S36>/Unit Delay' */
    ved__ve_DWork->UnitDelay_DSTATE_pd =
        ved__ve_B->sf_check_threshold.ext_reset;

    /* Update for UnitDelay: '<S42>/Unit Delay' */
    ved__ve_DWork->UnitDelay_DSTATE_l = rtb_Sum_n;

    /* Update for UnitDelay: '<S45>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S45>/FixPt Constant'
     */
    ved__ve_DWork->FixPtUnitDelay2_DSTATE_l = 0U;

    /* Update for UnitDelay: '<S37>/Unit Delay' */
    ved__ve_DWork->UnitDelay_DSTATE_n =
        ved__ve_B->sf_check_threshold_m.ext_reset;

    /* Update for UnitDelay: '<S47>/Unit Delay' */
    ved__ve_DWork->UnitDelay_DSTATE_j = rtb_Sum_a;

    /* Update for UnitDelay: '<S50>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S50>/FixPt Constant'
     */
    ved__ve_DWork->FixPtUnitDelay2_DSTATE_n = 0U;

    /* Update for UnitDelay: '<S38>/Unit Delay' */
    ved__ve_DWork->UnitDelay_DSTATE_m =
        ved__ve_B->sf_check_threshold_b.ext_reset;

    /* Update for UnitDelay: '<S52>/Unit Delay' */
    ved__ve_DWork->UnitDelay_DSTATE_p = rtb_Sum_c;

    /* Update for UnitDelay: '<S55>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S55>/FixPt Constant'
     */
    ved__ve_DWork->FixPtUnitDelay2_DSTATE_h = 0U;

    /* Update for UnitDelay: '<S39>/Unit Delay' */
    ved__ve_DWork->UnitDelay_DSTATE_k =
        ved__ve_B->sf_check_threshold_be.ext_reset;

    /* Update for UnitDelay: '<S57>/Unit Delay' */
    ved__ve_DWork->UnitDelay_DSTATE_o = rtb_Sum_j;

    /* Update for UnitDelay: '<S60>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S60>/FixPt Constant'
     */
    ved__ve_DWork->FixPtUnitDelay2_DSTATE_ln = 0U;
}

/* Model initialize function */
void ved__ve_initialize(boolean_T firstTime,
                        RT_MODEL_ved__ve *const ved__ve_M,
                        BlockIO_ved__ve *ved__ve_B,
                        D_Work_ved__ve *ved__ve_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__ve_M, (NULL));

    /* block I/O */
    (void)memset(((void *)ved__ve_B), 0, sizeof(BlockIO_ved__ve));

    /* states (dwork) */
    (void)memset((void *)ved__ve_DWork, 0, sizeof(D_Work_ved__ve));

    /* Start for ifaction SubSystem: '<S26>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S34>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &ved__ve_DWork->PHt_HPHt_R_DWORK4[0];
        for (i = 0; i < 16; i++) {
            *dWork++ = 0.0;
        }

        dWork = &ved__ve_DWork->PHt_HPHt_R_DWORK4[0];
        while (dWork < &ved__ve_DWork->PHt_HPHt_R_DWORK4[0] + 16) {
            *dWork = 1;
            dWork += 5;
        }
    }

    /* end of Start for SubSystem: '<S26>/calculate the gain' */
    {
        int32_T i;

        /* InitializeConditions for UnitDelay: '<S32>/FixPt Unit Delay2' */
        ved__ve_DWork->FixPtUnitDelay2_DSTATE = 1U;

        /* InitializeConditions for UnitDelay: '<S9>/P_delay' */
        for (i = 0; i < 4; i++) {
            ved__ve_DWork->P_delay_DSTATE[(i)] = ved__ve_P_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S45>/FixPt Unit Delay2' */
        ved__ve_DWork->FixPtUnitDelay2_DSTATE_l = 1U;

        /* InitializeConditions for UnitDelay: '<S50>/FixPt Unit Delay2' */
        ved__ve_DWork->FixPtUnitDelay2_DSTATE_n = 1U;

        /* InitializeConditions for UnitDelay: '<S55>/FixPt Unit Delay2' */
        ved__ve_DWork->FixPtUnitDelay2_DSTATE_h = 1U;

        /* InitializeConditions for UnitDelay: '<S60>/FixPt Unit Delay2' */
        ved__ve_DWork->FixPtUnitDelay2_DSTATE_ln = 1U;

        /* InitializeConditions for UnitDelay: '<S12>/last_v_var' */
        ved__ve_DWork->last_v_var_DSTATE = 1.0F;

        /* InitializeConditions for UnitDelay: '<S5>/last_a_var' */
        ved__ve_DWork->last_a_var_DSTATE = 1.0F;
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
