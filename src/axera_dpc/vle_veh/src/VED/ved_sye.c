
#include "ved_consts.h"
#include "ved_sye.h"
#include "ved_sye_private.h"
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
 *    '<S13>/make_A_matrix'
 *    '<S51>/make_A_matrix'
 */
void ved__sye_make_A_matrix(real32_T rtu_CycleTime,
                            rtB_make_A_matrix_ved__sye *localB) {
    int32_T i;

    /* Embedded MATLAB: '<S13>/make_A_matrix' */
    /* Embedded MATLAB Function 'Q_A_addpation_curve/A/make_A_matrix': '<S18>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S18>:1:5' */
    localB->A[0] = 1.0F;
    localB->A[2] = rtu_CycleTime;
    for (i = 0; i < 2; i++) {
        localB->A[1 + (i << 1)] = (real32_T)i;
    }
}

/*
 * Output and update for atomic system:
 *    '<S2>/Time2Sec'
 *    '<S26>/Time2Sec'
 *    '<S44>/Time2Sec'
 *    '<S9>/Time2Sec'
 */
void ved__sye_Time2Sec(uint16_T rtu_u, rtB_Time2Sec_ved__sye *localB) {
    /* Product: '<S15>/Time2Sec' incorporates:
     *  Constant: '<S15>/Constant'
     */
    localB->Time2Sec = ((real32_T)rtu_u) / 1000.0F;
}

/*
 * Output and update for atomic system:
 *    '<S17>/get_gain_bias'
 *    '<S54>/get_gain_bias'
 */
void ved__sye_get_gain_bias(const real32_T rtu_q_gain[3],
                            rtB_get_gain_bias_ved__sye *localB) {
    /* Embedded MATLAB: '<S17>/get_gain_bias' */
    /* Embedded MATLAB Function 'Q_A_addpation_curve/q_gain/get_gain_bias':
     * '<S22>:1' */
    /*  extract parameters */
    /* '<S22>:1:3' */
    localB->gain = rtu_q_gain[0];

    /* '<S22>:1:4' */
    localB->bias = rtu_q_gain[1];

    /* '<S22>:1:5' */
    localB->default_diff = rtu_q_gain[2];
}

/*
 * Output and update for atomic system:
 *    '<S30>/Get_IO_State1'
 *    '<S28>/Get_IO_State2'
 *    '<S45>/Get_IO_State2'
 */
void ved__sye_Get_IO_State1(const uint8_T rtu_state_in[32],
                            rtB_Get_IO_State1_ved__sye *localB,
                            uint32_T rtp_Filter) {
    /* MultiPortSwitch: '<S35>/Index Vector' incorporates:
     *  Constant: '<S35>/Constant1'
     */
    localB->IndexVector = rtu_state_in[(rtp_Filter)];
}

/*
 * Output and update for atomic system:
 *    '<S43>/get_init_control_mode'
 *    '<S9>/get_init_control_mode'
 */
void ved__sye_get_init_control_mode(uint16_T rtu_ved__ctrl_mode,
                                    rtB_get_init_control_mode_ved__s *localB) {
    /* Embedded MATLAB: '<S43>/get_init_control_mode' */
    /* Embedded MATLAB Function
     * 'build_Q_R_A_z/CaculateAddVar/get_init_control_mode': '<S49>:1' */
    /*  if the ved__ctrl mode is init set the output too true */
    if (rtu_ved__ctrl_mode == ((uint16_T)VED_CTRL_STATE_INIT)) {
        /* '<S49>:1:3' */
        /* '<S49>:1:4' */
        localB->y = TRUE;
    } else {
        /* '<S49>:1:6' */
        localB->y = FALSE;
    }
}

/*
 * Output and update for atomic system:
 *    '<S73>/Get_NVM_IO_State'
 *    '<S91>/Get_NVM_IO_State'
 */
void ved__sye_Get_NVM_IO_State(uint32_T rtu_pos,
                               uint32_T rtu_state_in,
                               uint32_T rtu_VED_IOBitMask,
                               rtB_Get_NVM_IO_State_ved__sye *localB) {
    int32_T x;
    uint32_T tmp;

    /* Embedded MATLAB: '<S73>/Get_NVM_IO_State' */
    /* Embedded MATLAB Function
     * 'extract_understeer_grad/Get_SlfStGrad_IO_State/GetIOState/Get_NVM_IO_State':
     * '<S74>:1' */
    /*  position in state array */
    /* '<S74>:1:4' */
    tmp = rtu_pos;
    if (rtu_pos > 2147483647U) {
        tmp = 2147483647U;
    }

    x = (int32_T)tmp;

    /* '<S74>:1:5' */
    /*  get state info */
    /* '<S74>:1:8' */
    localB->state = (rtu_state_in >> (x - ((x >> 5) << 5))) & rtu_VED_IOBitMask;
}

/*
 * Output and update for atomic system:
 *    '<S11>/At'
 *    '<S12>/At'
 */
void ved__sye_At(const real32_T rtu_u[4], rtB_At_ved__sye *localB) {
    int32_T i;
    int32_T i_0;

    /* Embedded MATLAB: '<S11>/At' */
    /* Embedded MATLAB Function 'swa_curve_EKF/At': '<S93>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S93>:1:5' */
    for (i_0 = 0; i_0 < 2; i_0++) {
        for (i = 0; i < 2; i++) {
            localB->y[i + (i_0 << 1)] = rtu_u[(i << 1) + i_0];
        }
    }
}

/*
 * Output and update for action system:
 *    '<S94>/calculate the gain'
 *    '<S104>/calculate the gain'
 */
void ved__sye_calculatethegain(const real32_T rtu_In2[2],
                               real32_T rtu_In1,
                               real32_T rty_Out1[2]) {
    int32_T i;

    /* Product: '<S101>/PHt_(HPHt_R)' */
    for (i = 0; i < 2; i++) {
        rty_Out1[(i)] = rtu_In2[(i)] / rtu_In1;
    }
}

/*
 * Output and update for atomic system:
 *    '<S94>/calculate determinant'
 *    '<S104>/calculate determinant'
 * Common block description:
 *   calculate determinant
 */
void ved__sye_calculatedeterminant(real32_T rtu_u,
                                   rtB_calculatedeterminant_ved__sy *localB) {
    /* Embedded MATLAB: '<S94>/calculate determinant' */
    /* Embedded MATLAB Function 'swa_curve_EKF/Calculate Kalman gain
     * PHt_(HPHt_R)/calculate determinant': '<S100>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /* '<S100>:1:3' */
    localB->y = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S11>/Ht'
 *    '<S12>/Ht'
 */
void ved__sye_Ht(const real32_T rtu_u[2], rtB_Ht_ved__sye *localB) {
    int32_T i;

    /* Embedded MATLAB: '<S11>/Ht' */
    /* Embedded MATLAB Function 'swa_curve_EKF/Ht': '<S95>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S95>:1:5' */
    for (i = 0; i < 2; i++) {
        localB->y[(i)] = rtu_u[(i)];
    }
}

/*
 * Output and update for atomic system:
 *    '<S11>/Reset_P_pred'
 *    '<S12>/Reset_P_pred'
 */
void ved__sye_Reset_P_pred(const real32_T rtu_P_pred_in[4],
                           real32_T rtu_u_det,
                           rtB_Reset_P_pred_ved__sye *localB) {
    int32_T i;

    /* Embedded MATLAB: '<S11>/Reset_P_pred' */
    /* Embedded MATLAB Function 'swa_curve_EKF/Reset_P_pred': '<S96>:1' */
    if (((real32_T)fabs(rtu_u_det)) <= 1.0E-16F) {
        /* '<S96>:1:4' */
        /* '<S96>:1:6' */
        for (i = 0; i < 4; i++) {
            localB->P_pred_out[(i)] = 0.0F;
        }
    } else {
        /* '<S96>:1:10' */
        for (i = 0; i < 4; i++) {
            localB->P_pred_out[(i)] = rtu_P_pred_in[(i)];
        }
    }
}

/*
 * Output and update for atomic system:
 *    '<S11>/eye'
 *    '<S12>/eye'
 */
void ved__sye_eye(rtB_eye_ved__sye *localB) {
    int32_T i;
    static int8_T tmp[4] = {1, 0, 0, 1};

    /* Embedded MATLAB: '<S11>/eye' */
    /* Embedded MATLAB Function 'swa_curve_EKF/eye': '<S98>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S98>:1:5' */
    for (i = 0; i < 4; i++) {
        localB->y[(i)] = (real32_T)tmp[i];
    }
}

/* Model step function */
void ved__sye_step(BlockIO_ved__sye *ved__sye_B,
                   D_Work_ved__sye *ved__sye_DWork,
                   VED_InputData_t *ved__sye_U_VED_InputData,
                   VED_InternalData_t *ved__sye_U_VED_InternalData_in,
                   VED_NvData_t *ved__sye_U_VED_NVData_in,
                   VED_InternalData_t *ved__sye_Y_VED_InternalData_out,
                   VED_NvData_t *ved__sye_Y_VED_NVData_out) {
    /* local block i/o variables */
    real32_T rtb_HPHt_R[4];
    real32_T rtb_P_predHt[6];
    real32_T rtb_K[6];
    real32_T rtb_P_pred_a[4];
    real32_T rtb_HPHt_R_l;
    real32_T rtb_P_predHt_b[2];
    real32_T rtb_K_l[2];
    real32_T rtb_KH_a[4];
    real32_T rtb_P_pred_d[4];
    real32_T rtb_HPHt_R_n;
    real32_T rtb_P_predHt_h[2];
    real32_T rtb_K_ld[2];
    real32_T rtb_KH_m[4];
    real32_T rtb_H[2];
    real32_T rtb_H_p[2];
    uint32_T rtb_State;
    uint32_T rtb_State_p;
    uint16_T rtb_CtrlMode;
    uint16_T rtb_CycleTime;
    uint16_T rtb_CycleTime_b;
    uint16_T rtb_CycleTime_e;
    uint16_T rtb_CtrlMode_j;
    uint16_T rtb_CycleTime_k;
    uint8_T rtb_State_a[32];
    uint8_T rtb_State_a4[32];
    uint8_T rtb_State_l[32];

    /* local scratch DWork variables */
    real32_T PHt_HPHt_R_DWORK1[4];
    real32_T PHt_HPHt_R_DWORK3[4];
    real32_T PHt_HPHt_R_DWORK5[4];
    int32_T PHt_HPHt_R_DWORK2[2];
    int32_T k;
    uint32_T c;
    real32_T rtb_rat[2];
    real32_T rtb_xpost[3];
    real32_T rtb_y_m[3];
    real32_T rtb_x_pred[3];
    real32_T rtb_Divide;
    real32_T rtb_T3;
    real32_T rtb_T2;
    real32_T rtb_T1;
    real32_T rtb_T0;
    real32_T rtb_T7;
    real32_T rtb_T6;
    real32_T rtb_T5;
    real32_T rtb_T4;
    real32_T rtb_z_out_a[2];
    real32_T rtb_hx_g[2];
    uint8_T rtb_DataTypeConversion;
    int32_T rtb_DataTypeConversion3;
    real32_T rtb_x_pred_i[2];
    real32_T rtb_T2_e;
    real32_T rtb_T1_p;
    real32_T rtb_T0_a;
    real32_T rtb_R_out_l;
    real32_T rtb_z_out_c;
    real32_T rtb_x_pred_n[2];
    real32_T rtb_T2_m;
    real32_T rtb_T1_f;
    real32_T rtb_T0_k;
    real32_T rtb_addVariance;
    real32_T rtb_hx_d;
    real32_T rtb_Reset;
    real32_T rtb_Reset_m;
    real32_T rtb_SlfStGradDisc_out;
    uint8_T rtb_EGDyn;
    real32_T rtb_y_hc[2];
    real32_T rtb_y_o;
    real32_T rtb_SlfStGradMax_out;
    real32_T rtb_SlfStGradMin_out;
    real32_T rtb_SlfStGradNVM_out;
    real32_T rtb_sigma_EG;
    real32_T rtb_add_trav_dist;
    real32_T rtb_Reset_l;
    uint8_T rtb_init_out;
    boolean_T rtb_trav_dist_reset_f;
    boolean_T rtb_R_onoff_n;
    real32_T rtb_init_SSG;
    uint8_T rtb_init;
    uint16_T rtb_y_a;
    uint32_T rtb_state;
    real32_T rtb_A[9];
    real32_T rtb_H_a[6];
    real32_T rtb_P_pred[9];
    real32_T rtb_y_f[6];
    real32_T rtb_APAt[4];
    real32_T rtb_R_out[4];
    real32_T rtb_Sum_c[4];
    int32_T i;
    real32_T rtb_rat_0[2];
    real32_T rtb_rat_1[4];
    real32_T rtb_A_0[9];
    real32_T rtb_rat_2[9];
    real32_T rtb_A_1[9];
    real32_T rtb_H_a_0[6];
    real32_T rtb_z_out_a_0[2];
    static int8_T tmp[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    static int8_T tmp_0[3] = {0, 1, 0};

    static int8_T tmp_1[3] = {0, 0, 1};

    /* BusSelector: '<S28>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_a[i] = (*ved__sye_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S28>/Get_IO_State2' */
    ved__sye_Get_IO_State1(rtb_State_a, &ved__sye_B->Get_IO_State2,
                           ((uint32_T)VED_SIN_POS_SWA));

    /* end of Outputs for SubSystem: '<S28>/Get_IO_State2' */

    /* Embedded MATLAB: '<S28>/make_z_vector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'build_EG_Q_R_A_z/make_z_vektor/make_z_vector':
     * '<S42>:1' */
    /*  if the steering wheel angle is availabel use the */
    /*  offst compensated steering wheel angle  */
    /*  otherwise set to zero */
    if (((uint32_T)ved__sye_B->Get_IO_State2.IndexVector) ==
        ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S42>:1:5' */
        /* '<S42>:1:6' */
        rtb_R_onoff_n = TRUE;

        /* '<S42>:1:7' */
        rtb_z_out_a[0] =
            (*ved__sye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate;
        rtb_z_out_a[1] = (*ved__sye_U_VED_InputData).Signals.StWheelAngle -
                         (*ved__sye_U_VED_InternalData_in)
                             .ved__offsets_in.ved__swa_offset.offset;
    } else {
        /* '<S42>:1:9' */
        rtb_R_onoff_n = FALSE;

        /* '<S42>:1:10' */
        rtb_z_out_a[0] =
            (*ved__sye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate;
        rtb_z_out_a[1] = 0.0F;
    }

    /* Embedded MATLAB: '<S27>/adapt_R_matrix' */
    /* Embedded MATLAB Function 'build_EG_Q_R_A_z/make_R/adapt_R_matrix':
     * '<S40>:1' */
    /*  if the steering wheel angle is invalid */
    /* R_out = single([0.01^2 0.0;... */
    /*                 0.0 0.018^2]); */
    /* '<S40>:1:5' */
    for (i = 0; i < 4; i++) {
        rtb_R_out[i] = ved__sye_R_SSG_p[(i)];
    }

    if (((int32_T)rtb_R_onoff_n) == 0) {
        /* '<S40>:1:6' */
        /* '<S40>:1:7' */
        rtb_R_out[3] = 100.0F;
    }

    /* BusSelector: '<S10>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CtrlMode = (*ved__sye_U_VED_InputData).Frame.CtrlMode;
    for (i = 0; i < 2; i++) {
        rtb_rat[i] =
            (*ved__sye_U_VED_InputData).Parameter.SteeringRatio.swa.rat[(i)];
    }

    /* BusSelector: '<S26>/Bus Selector3' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_a4[i] = (*ved__sye_U_VED_InputData).Signals.State[(i)];
    }

    rtb_CycleTime = (*ved__sye_U_VED_InputData).Frame.CycleTime;

    /* Outputs for atomic SubSystem: '<S26>/Time2Sec' */
    ved__sye_Time2Sec(rtb_CycleTime, &ved__sye_B->Time2Sec);

    /* end of Outputs for SubSystem: '<S26>/Time2Sec' */

    /* Embedded MATLAB: '<S29>/make_A_matrix' */
    /* Embedded MATLAB Function
     * 'build_EG_Q_R_A_z/Q_A_adapation_curve/A/make_A_matrix': '<S34>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S34>:1:5' */
    rtb_A[0] = 1.0F;
    rtb_A[3] = ved__sye_B->Time2Sec.Time2Sec;
    rtb_A[6] = 0.0F;
    for (i = 0; i < 3; i++) {
        rtb_A[1 + (3 * i)] = (real32_T)tmp_0[i];
    }

    for (i = 0; i < 3; i++) {
        rtb_A[2 + (3 * i)] = (real32_T)tmp_1[i];
    }

    /* UnitDelay: '<S10>/x_delay_eg' */
    for (i = 0; i < 3; i++) {
        rtb_xpost[i] = ved__sye_DWork->x_delay_eg_DSTATE[(i)];
    }

    /* BusSelector: '<S10>/Bus Selector2' incorporates:
     *  Inport: '<Root>/VED_NVData_in'
     */
    rtb_State = (*ved__sye_U_VED_NVData_in).Read.State;

    /* Embedded MATLAB: '<S91>/Get_NVM_IO_State' */
    ved__sye_Get_NVM_IO_State(((uint32_T)VED_NVM_POS_SSG), rtb_State, 3U,
                              &ved__sye_B->sf_Get_NVM_IO_State);

    /* Embedded MATLAB: '<S10>/check_init_state' incorporates:
     *  Inport: '<Root>/VED_NVData_in'
     *  UnitDelay: '<S10>/init_nvm_eg_delay'
     */
    /* Embedded MATLAB Function 'swa_EG_EKF/check_init_state': '<S85>:1' */
    /*  check if NVMSSG is valid and first init is true */
    if ((ved__sye_B->sf_Get_NVM_IO_State.state ==
         ((uint32_T)VED_IO_STATE_VALID)) &&
        (ved__sye_DWork->init_nvm_eg_delay_DSTATE == 0)) {
        /* '<S85>:1:3' */
        /* '<S85>:1:4' */
        rtb_init_SSG = (*ved__sye_U_VED_NVData_in).Read.SlfstGrad.SlfStGrad;

        /* '<S85>:1:5' */
        rtb_init = 1U;

        /* '<S85>:1:6' */
        rtb_init_out = 1U;
    } else {
        /* '<S85>:1:8' */
        rtb_init_SSG = 0.005F;

        /* '<S85>:1:9' */
        rtb_init = 0U;

        /* '<S85>:1:10' */
        rtb_init_out = ved__sye_DWork->init_nvm_eg_delay_DSTATE;
    }

    /* Embedded MATLAB: '<S10>/Range check of the SSG' */
    /* Embedded MATLAB Function 'swa_EG_EKF/Range check of the SSG': '<S81>:1'
     */
    /*  check for understeer range violences */
    /* '<S81>:1:3' */
    for (i = 0; i < 3; i++) {
        rtb_y_m[i] = rtb_xpost[i];
    }

    /*  if understeer gradient is below 0.1  */
    if (rtb_xpost[2] < 0.00175F) {
        /* '<S81>:1:5' */
        /* '<S81>:1:6' */
        rtb_y_m[2] = 0.00175F;
    }

    /*  if understeer gradient is over 0.45 */
    if (rtb_xpost[2] > 0.00785F) {
        /* '<S81>:1:9' */
        /* '<S81>:1:10' */
        rtb_y_m[2] = 0.00785F;
    }

    /*  init self steering gradient if init is true */
    if (rtb_init == 1) {
        /* '<S81>:1:14' */
        /* '<S81>:1:15' */
        rtb_y_m[2] = rtb_init_SSG;
    }

    /* Product: '<S10>/Ax' */
    for (i = 0; i < 3; i++) {
        rtb_x_pred[i] = 0.0F;
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 3;
             rtb_DataTypeConversion3++) {
            rtb_x_pred[i] = (rtb_A[(3 * rtb_DataTypeConversion3) + i] *
                             rtb_y_m[rtb_DataTypeConversion3]) +
                            rtb_x_pred[i];
        }
    }

    /* Embedded MATLAB: '<S10>/hx' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_init_SSG = (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo;

    /* Embedded MATLAB Function 'swa_EG_EKF/hx': '<S87>:1' */
    /*  calculate transformation from state space to measurement space */
    if (((real32_T)fabs(
            (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo)) <=
        0.0001F) {
        /* '<S87>:1:4' */
        /* '<S87>:1:5' */
        rtb_init_SSG = 0.0001F;
    }

    /* '<S87>:1:8' */
    /*  non linear equation  */
    /* '<S87>:1:10' */
    rtb_hx_g[0] = rtb_x_pred[1];
    rtb_hx_g[1] =
        ((((rtb_init_SSG * rtb_init_SSG) * rtb_x_pred[2]) +
          (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
         (rtb_rat[1] * rtb_x_pred[1])) /
        rtb_init_SSG;

    /*  linearised transformation matrix H */
    /* '<S87>:1:13' */
    for (i = 0; i < 3; i++) {
        rtb_H_a[i << 1] = (real32_T)tmp_0[i];
    }

    rtb_H_a[1] = 0.0F;
    rtb_H_a[3] = (rtb_rat[1] *
                  (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) /
                 rtb_init_SSG;
    rtb_H_a[5] =
        (rtb_rat[1] *
         (*ved__sye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate) *
        rtb_init_SSG;

    /* Product: '<S31>/Divide' incorporates:
     *  Constant: '<S31>/Constant'
     *  Product: '<S31>/Product1'
     */
    rtb_Divide =
        (ved__sye_B->Time2Sec.Time2Sec * ved__sye_B->Time2Sec.Time2Sec) * 0.5F;

    /* UnitDelay: '<S37>/T3' */
    rtb_T3 = ved__sye_DWork->T3_DSTATE;

    /* UnitDelay: '<S37>/T2' */
    rtb_T2 = ved__sye_DWork->T2_DSTATE;

    /* UnitDelay: '<S37>/T1' */
    rtb_T1 = ved__sye_DWork->T1_DSTATE;

    /* UnitDelay: '<S37>/T0' */
    rtb_T0 = ved__sye_DWork->T0_DSTATE;

    /* UnitDelay: '<S37>/T7' */
    rtb_T7 = ved__sye_DWork->T7_DSTATE;

    /* UnitDelay: '<S37>/T6' */
    rtb_T6 = ved__sye_DWork->T6_DSTATE;

    /* UnitDelay: '<S37>/T5' */
    rtb_T5 = ved__sye_DWork->T5_DSTATE;

    /* UnitDelay: '<S37>/T4' */
    rtb_T4 = ved__sye_DWork->T4_DSTATE;

    /* Outputs for atomic SubSystem: '<S30>/Get_IO_State1' */
    ved__sye_Get_IO_State1(rtb_State_a4, &ved__sye_B->Get_IO_State1,
                           ((uint32_T)VED_SIN_POS_LATA));

    /* end of Outputs for SubSystem: '<S30>/Get_IO_State1' */

    /* Abs: '<S30>/Abs' incorporates:
     *  BusSelector: '<S26>/Bus Selector2'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_init_SSG = (real32_T)fabs(
        (*ved__sye_U_VED_InternalData_in).ved__ye_out.veh_lat_accel);

    /* Embedded MATLAB: '<S33>/get_bit' incorporates:
     *  Constant: '<S33>/bitmask'
     *  Inport: '<Root>/VED_InputData'
     *  S-Function (sfix_bitop): '<S33>/Bitwise Operator'
     */
    /* Embedded MATLAB Function 'get_bit/get_bit': '<S39>:1' */
    /*  get state of a specified bit */
    if (((*ved__sye_U_VED_InputData).Frame.CaliMode &
         ((uint16_T)VED_CAL_SWA_GRAD)) == ((uint16_T)VED_CAL_SWA_GRAD)) {
        /* '<S39>:1:3' */
        /* '<S39>:1:4' */
        rtb_y_a = 1U;
    } else {
        /* '<S39>:1:6' */
        rtb_y_a = 0U;
    }

    /* Embedded MATLAB: '<S30>/controll_ssg_noice' incorporates:
     *  Abs: '<S30>/Abs1'
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     *  Sum: '<S26>/Add'
     */
    /* Embedded MATLAB Function
     * 'build_EG_Q_R_A_z/Q_A_adapation_curve/Build_EG_State/controll_ssg_noice':
     * '<S36>:1' */
    /*  this function calculates the model noise parameter EG velocity for the
     * EG understeer */
    /*  gradient, if the driving situation is suitable for EG learning */
    /*  the EG velocity is tuned up otherwise the EG velocity is very small */
    /*  if on of the imput signal used for driving situation detection */
    /*  is not abailable the velocty is small */

    /*  %    sigma_EG = single(0.0000000000001); % 1 exp -8 */
    /*      sigma_EG = ved__sye_Q_SSG_range_p(6); */
    /*      r = single(0.0); */
    /*      vmax = single(0.0); */
    /*      vq = single(0.0); */
    /*      vmin = single(0.0); */
    /*      EGDyn = single(0.0); */
    /*  else */
    /*      VelocityQ = velocity*velocity; */
    /*      vq = VelocityQ; */
    /*      LatAccelRadius = VelocityQ / lateral_accel; */
    /*      r = LatAccelRadius; */
    /*  %    MaxVeloQ = LatAccelRadius * single(4.0); */
    /*      MaxVeloQ = LatAccelRadius * ved__sye_Q_SSG_range_p(4); */
    /*      vmax = MaxVeloQ; */
    /*  %    MinVeloQ = LatAccelRadius * single(0.5); */
    /*      MinVeloQ = LatAccelRadius * ved__sye_Q_SSG_range_p(3); */
    /*      vmin = MinVeloQ; */
    /*  %    if ((LatAccelRadius >= 25) && (LatAccelRadius <= 500) && (VelocityQ
     * <= MaxVeloQ) && (VelocityQ >= MinVeloQ)) */
    /*      if ((LatAccelRadius >=  ved__sye_Q_SSG_range_p(1)) &&
     * (LatAccelRadius <=  ved__sye_Q_SSG_range_p(2)) && (VelocityQ <= MaxVeloQ)
     * && (VelocityQ >= MinVeloQ)) */
    /*  %         sigma_EG = single(0.000000001); */
    /*          sigma_EG = ved__sye_Q_SSG_range_p(5); */
    /*          EGDyn = single(1.0); */
    /*      else */
    /*  %         sigma_EG = single(0.0000000000001); % 1 exp -8 */
    /*          sigma_EG = ved__sye_Q_SSG_range_p(6); */
    /*          EGDyn = single(0.0); */
    /*      end; */
    /*   */
    /*  end */
    /*  if the steering wheel angle is below a specified value */
    /*  and the lateral acceleration is in a specified range then the EG
     * learning is possible */
    if ((((((real32_T)fabs((*ved__sye_U_VED_InputData).Signals.StWheelAngle -
                           (*ved__sye_U_VED_InternalData_in)
                               .ved__offsets_in.ved__swa_offset.offset)) <
           ved__sye_Q_SSG_range_p[0]) &&
          (rtb_init_SSG >= ved__sye_Q_SSG_range_p[1])) &&
         (rtb_init_SSG <= ved__sye_Q_SSG_range_p[2])) &&
        ((*ved__sye_U_VED_InternalData_in)
             .ved__offsets_in.ved__swa_offset.state >= 2)) {
        /* '<S36>:1:44' */
        /* '<S36>:1:45' */
        rtb_sigma_EG = ved__sye_Q_SSG_range_p[3];

        /* '<S36>:1:46' */
        /* '<S36>:1:47' */
        /* '<S36>:1:48' */
        /* '<S36>:1:49' */
        /* '<S36>:1:50' */
        rtb_EGDyn = 0U;
    } else {
        /* '<S36>:1:52' */
        rtb_sigma_EG = ved__sye_Q_SSG_range_p[4];

        /* '<S36>:1:53' */
        /* '<S36>:1:54' */
        /* '<S36>:1:55' */
        /* '<S36>:1:56' */
        /* '<S36>:1:57' */
        rtb_EGDyn = 1U;
    }

    if (rtb_y_a == 1) {
        /* '<S36>:1:60' */
        /* '<S36>:1:61' */
        rtb_EGDyn = 1U;
    }

    /* Product: '<S37>/Divide' incorporates:
     *  Constant: '<S37>/filter_length'
     *  Sum: '<S37>/Add'
     */
    rtb_init_SSG =
        ((((((((rtb_T3 + rtb_T2) + rtb_T1) + rtb_T0) + rtb_T7) + rtb_T6) +
           rtb_T5) +
          rtb_T4) +
         rtb_sigma_EG) *
        0.11111F;

    rtb_rat[0] = rtb_Divide;
    rtb_rat[1] = ved__sye_B->Time2Sec.Time2Sec;

    /* Embedded MATLAB: '<S31>/makeQ' incorporates:
     *  Constant: '<S31>/sigma_model'
     */
    /* Embedded MATLAB Function 'build_EG_Q_R_A_z/Q_A_adapation_curve/Q/makeQ':
     * '<S38>:1' */
    /*  calculate the model covariance matrix Q */
    /* '<S38>:1:3' */
    /* '<S38>:1:4' */

    /* Embedded MATLAB: '<S10>/At' */
    /* Embedded MATLAB Function 'swa_EG_EKF/At': '<S77>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S77>:1:5' */

    /* Sum: '<S10>/APA_Q' incorporates:
     *  Product: '<S10>/APAt'
     *  UnitDelay: '<S10>/P_delay_eg'
     */
    for (i = 0; i < 2; i++) {
        rtb_rat_0[i] = rtb_rat[i] * ved__sye_Q_SSG_sigmas_p[(i)];
        rtb_z_out_a_0[i] = ved__sye_Q_SSG_sigmas_p[(i)] * rtb_rat[i];
    }

    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_rat_1[rtb_DataTypeConversion3 + (i << 1)] =
                rtb_rat_0[rtb_DataTypeConversion3] * rtb_z_out_a_0[i];
        }
    }

    for (i = 0; i < 3; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 3;
             rtb_DataTypeConversion3++) {
            rtb_A_0[i + (3 * rtb_DataTypeConversion3)] = 0.0F;
            for (k = 0; k < 3; k++) {
                rtb_A_0[i + (3 * rtb_DataTypeConversion3)] =
                    (rtb_A[(3 * k) + i] *
                     ved__sye_DWork
                         ->P_delay_eg_DSTATE[(3 * rtb_DataTypeConversion3) +
                                             k]) +
                    rtb_A_0[(3 * rtb_DataTypeConversion3) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_rat_2[rtb_DataTypeConversion3 + (3 * i)] =
                rtb_rat_1[(i << 1) + rtb_DataTypeConversion3];
        }
    }

    for (i = 0; i < 2; i++) {
        rtb_rat_2[i + 6] = 0.0F;
    }

    rtb_rat_2[2] = 0.0F;
    rtb_rat_2[5] = 0.0F;
    rtb_rat_2[8] =
        ((ved__sye_B->Time2Sec.Time2Sec * ved__sye_B->Time2Sec.Time2Sec) *
         rtb_init_SSG) *
        rtb_init_SSG;
    for (i = 0; i < 3; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 3;
             rtb_DataTypeConversion3++) {
            rtb_A_1[i + (3 * rtb_DataTypeConversion3)] = 0.0F;
            for (k = 0; k < 3; k++) {
                rtb_A_1[i + (3 * rtb_DataTypeConversion3)] =
                    (rtb_A_0[(3 * k) + i] *
                     rtb_A[(3 * k) + rtb_DataTypeConversion3]) +
                    rtb_A_1[(3 * rtb_DataTypeConversion3) + i];
            }
        }
    }

    for (i = 0; i < 3; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 3;
             rtb_DataTypeConversion3++) {
            rtb_P_pred[rtb_DataTypeConversion3 + (3 * i)] =
                rtb_rat_2[(3 * i) + rtb_DataTypeConversion3] +
                rtb_A_1[(3 * i) + rtb_DataTypeConversion3];
        }
    }

    /* Embedded MATLAB: '<S10>/Ht' */
    /* Embedded MATLAB Function 'swa_EG_EKF/Ht': '<S80>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S80>:1:5' */
    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 3;
             rtb_DataTypeConversion3++) {
            rtb_y_f[rtb_DataTypeConversion3 + (3 * i)] =
                rtb_H_a[(rtb_DataTypeConversion3 << 1) + i];
        }
    }

    /* Sum: '<S10>/HPH_R' incorporates:
     *  Product: '<S10>/HPHt'
     */
    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 3;
             rtb_DataTypeConversion3++) {
            rtb_H_a_0[i + (rtb_DataTypeConversion3 << 1)] = 0.0F;
            for (k = 0; k < 3; k++) {
                rtb_H_a_0[i + (rtb_DataTypeConversion3 << 1)] =
                    (rtb_H_a[(k << 1) + i] *
                     rtb_P_pred[(3 * rtb_DataTypeConversion3) + k]) +
                    rtb_H_a_0[(rtb_DataTypeConversion3 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_Divide = 0.0F;
            for (k = 0; k < 3; k++) {
                rtb_Divide += rtb_H_a_0[(k << 1) + i] *
                              rtb_y_f[(3 * rtb_DataTypeConversion3) + k];
            }

            rtb_HPHt_R[i + (rtb_DataTypeConversion3 << 1)] =
                rtb_R_out[(rtb_DataTypeConversion3 << 1) + i] + rtb_Divide;
        }
    }

    /* Product: '<S10>/P_pred*Ht' */
    for (i = 0; i < 3; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_P_predHt[i + (3 * rtb_DataTypeConversion3)] = 0.0F;
            for (k = 0; k < 3; k++) {
                rtb_P_predHt[i + (3 * rtb_DataTypeConversion3)] =
                    (rtb_P_pred[(3 * k) + i] *
                     rtb_y_f[(3 * rtb_DataTypeConversion3) + k]) +
                    rtb_P_predHt[(3 * rtb_DataTypeConversion3) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S78>/calculate determinant' */
    /* Embedded MATLAB Function 'swa_EG_EKF/Calculate Kalman gain
     * PHt_(HPHt_R)/calculate determinant': '<S88>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /*  y = single(det(u)); */
    /* '<S88>:1:4' */
    rtb_init_SSG =
        (rtb_HPHt_R[0] * rtb_HPHt_R[3]) - (rtb_HPHt_R[1] * rtb_HPHt_R[2]);

    /* If: '<S78>/If' incorporates:
     *  ActionPort: '<S89>/Action Port'
     *  ActionPort: '<S90>/Action Port'
     *  SubSystem: '<S78>/calculate the gain'
     *  SubSystem: '<S78>/set gain to default value'
     */
    if (rtb_init_SSG > 1.0E-16F) {
        /* Product: '<S89>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {2, 2, 2, 3, 2, 2};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5[0], rtb_HPHt_R,
                            &ved__sye_DWork->PHt_HPHt_R_DWORK4[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1[0],
                            &PHt_HPHt_R_DWORK2[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K, rtb_P_predHt, &PHt_HPHt_R_DWORK5[0],
                             &dims[3]);
        }
    } else {
        /* Constant: '<S90>/Constant' */
        for (i = 0; i < 6; i++) {
            rtb_K[i] = ved__sye_ConstP.Constant_Value[(i)];
        }
    }

    /* Embedded MATLAB: '<S10>/eye' */
    /* Embedded MATLAB Function 'swa_EG_EKF/eye': '<S86>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S86>:1:5' */

    /* Embedded MATLAB: '<S10>/Reset_P_pred' */
    /* Embedded MATLAB Function 'swa_EG_EKF/Reset_P_pred': '<S82>:1' */
    if (((real32_T)fabs(rtb_init_SSG)) <= 1.0E-16F) {
        /* '<S82>:1:4' */
        /* '<S82>:1:6' */
        for (i = 0; i < 9; i++) {
            rtb_P_pred[i] = 0.0F;
        }
    } else {
        /* '<S82>:1:10' */
    }

    /* Product: '<S10>/P_pred_(1_KH)' incorporates:
     *  Product: '<S10>/K*H'
     *  Sum: '<S10>/1_KH'
     */
    for (i = 0; i < 3; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 3;
             rtb_DataTypeConversion3++) {
            rtb_Divide = 0.0F;
            for (k = 0; k < 2; k++) {
                rtb_Divide += rtb_K[(3 * k) + i] *
                              rtb_H_a[(rtb_DataTypeConversion3 << 1) + k];
            }

            rtb_A_0[i + (3 * rtb_DataTypeConversion3)] =
                ((real32_T)tmp[(3 * rtb_DataTypeConversion3) + i]) - rtb_Divide;
        }
    }

    for (i = 0; i < 3; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 3;
             rtb_DataTypeConversion3++) {
            rtb_A[i + (3 * rtb_DataTypeConversion3)] = 0.0F;
            for (k = 0; k < 3; k++) {
                rtb_A[i + (3 * rtb_DataTypeConversion3)] =
                    (rtb_A_0[(3 * k) + i] *
                     rtb_P_pred[(3 * rtb_DataTypeConversion3) + k]) +
                    rtb_A[(3 * rtb_DataTypeConversion3) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S9>/diag_EG' */
    /* Embedded MATLAB Function 'extract_understeer_grad/diag_EG': '<S68>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S68>:1:4' */

    /* Embedded MATLAB: '<S9>/get_init_control_mode' */
    ved__sye_get_init_control_mode(rtb_CtrlMode,
                                   &ved__sye_B->sf_get_init_control_mode);

    /* Embedded MATLAB: '<S10>/Reset_x_pred' */
    /* Embedded MATLAB Function 'swa_EG_EKF/Reset_x_pred': '<S83>:1' */
    if (((real32_T)fabs(rtb_init_SSG)) <= 1.0E-16F) {
        /* '<S83>:1:4' */
        /* '<S83>:1:6' */
        rtb_x_pred[0] = 0.0F;
        rtb_x_pred[1] = 0.0F;
        rtb_x_pred[2] = rtb_xpost[2];
    } else {
        /* '<S83>:1:10' */
    }

    /* Embedded MATLAB: '<S10>/check_eg' incorporates:
     *  Product: '<S10>/Knu'
     *  Sum: '<S10>/x_Knu'
     *  Sum: '<S10>/z_Hx'
     */
    /* Embedded MATLAB Function 'swa_EG_EKF/check_eg': '<S84>:1' */
    /*  If EG_Control is 1, so no EG should be leard. */
    /*  and the EG state in the filter is not updated */
    /* '<S84>:1:5' */
    for (i = 0; i < 2; i++) {
        rtb_z_out_a_0[i] = rtb_z_out_a[i] - rtb_hx_g[i];
    }

    for (i = 0; i < 3; i++) {
        rtb_Divide = 0.0F;
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_Divide += rtb_K[(3 * rtb_DataTypeConversion3) + i] *
                          rtb_z_out_a_0[rtb_DataTypeConversion3];
        }

        rtb_xpost[i] = rtb_x_pred[i] + rtb_Divide;
    }

    /*  if 0 don't update eg */
    if (rtb_EGDyn == 1) {
        /* '<S84>:1:7' */
        /* '<S84>:1:8' */
        rtb_xpost[2] = rtb_x_pred[2];
    }

    /* Switch: '<S75>/Reset' incorporates:
     *  Constant: '<S75>/Initial Condition'
     *  MinMax: '<S64>/MinMax'
     *  UnitDelay: '<S75>/FixPt Unit Delay1'
     */
    if (ved__sye_B->sf_get_init_control_mode.y) {
        rtb_Reset = 0.0F;
    } else {
        rtb_Reset = (rtb_xpost[2] >= ved__sye_DWork->FixPtUnitDelay1_DSTATE)
                        ? rtb_xpost[2]
                        : ved__sye_DWork->FixPtUnitDelay1_DSTATE;
    }

    /* Switch: '<S76>/Reset' incorporates:
     *  Constant: '<S76>/Initial Condition'
     *  MinMax: '<S65>/MinMax'
     *  UnitDelay: '<S76>/FixPt Unit Delay1'
     */
    if (ved__sye_B->sf_get_init_control_mode.y) {
        rtb_Reset_m = 0.008F;
    } else {
        rtb_Reset_m = (rtb_xpost[2] <= ved__sye_DWork->FixPtUnitDelay1_DSTATE_b)
                          ? rtb_xpost[2]
                          : ved__sye_DWork->FixPtUnitDelay1_DSTATE_b;
    }

    /* BusSelector: '<S9>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_NVData_in'
     */
    rtb_State_p = (*ved__sye_U_VED_NVData_in).Read.State;
    rtb_init = (*ved__sye_U_VED_NVData_in).Read.SlfstGrad.SlfStGradMax;
    rtb_DataTypeConversion =
        (*ved__sye_U_VED_NVData_in).Read.SlfstGrad.SlfStGradMin;

    /* Embedded MATLAB: '<S73>/Get_NVM_IO_State' */
    ved__sye_Get_NVM_IO_State(((uint32_T)VED_NVM_POS_SSG), rtb_State_p, 3U,
                              &ved__sye_B->sf_Get_NVM_IO_State_g);

    i = rtb_init;

    rtb_DataTypeConversion3 = rtb_DataTypeConversion;

    /* BusSelector: '<S9>/Bus Selector3' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CycleTime_b = (*ved__sye_U_VED_InputData).Frame.CycleTime;

    /* Outputs for atomic SubSystem: '<S9>/Time2Sec' */
    ved__sye_Time2Sec(rtb_CycleTime_b, &ved__sye_B->Time2Sec_i);

    /* end of Outputs for SubSystem: '<S9>/Time2Sec' */

    /* UnitDelay: '<S9>/trav_dist_reset' */
    rtb_R_onoff_n = ved__sye_DWork->trav_dist_reset_DSTATE;

    /* Switch: '<S70>/Init' incorporates:
     *  Constant: '<S70>/Initial Condition'
     *  Logic: '<S70>/FixPt Logical Operator'
     *  UnitDelay: '<S70>/FixPt Unit Delay1'
     *  UnitDelay: '<S70>/FixPt Unit Delay2'
     */
    if (ved__sye_DWork->trav_dist_reset_DSTATE ||
        (ved__sye_DWork->FixPtUnitDelay2_DSTATE != 0)) {
        rtb_init_SSG = 0.0F;
    } else {
        rtb_init_SSG = ved__sye_DWork->FixPtUnitDelay1_DSTATE_g;
    }

    /* Sum: '<S9>/add_trav_dist' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  Product: '<S9>/inc_dist'
     */
    rtb_add_trav_dist =
        (ved__sye_B->Time2Sec_i.Time2Sec *
         (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo) +
        rtb_init_SSG;

    /* Embedded MATLAB: '<S9>/check_with_read_values' incorporates:
     *  Inport: '<Root>/VED_NVData_in'
     *  UnitDelay: '<S9>/last_SlfStGradDisc'
     */
    /* Embedded MATLAB Function
     * 'extract_understeer_grad/check_with_read_values': '<S67>:1' */
    /*  extract min max values */
    /* '<S67>:1:3' */
    rtb_T3 = ((real32_T)rtb_DataTypeConversion3) / 25000.0F;

    /* '<S67>:1:4' */
    rtb_init_SSG = ((real32_T)i) / 25000.0F;

    /*  if the actual min value is below the read nvm value use the actual */
    /* '<S67>:1:7' */
    if (rtb_Reset_m < rtb_T3) {
        /* '<S67>:1:8' */
        /* '<S67>:1:9' */
        rtb_T3 = rtb_Reset_m;
    }

    /*  if the actual max value is above the read nvm value use the actual */
    /* '<S67>:1:14' */
    if (rtb_Reset > rtb_init_SSG) {
        /* '<S67>:1:15' */
        /* '<S67>:1:16' */
        rtb_init_SSG = rtb_Reset;
    }

    /*  calc new min max values */
    /* '<S67>:1:21' */
    /* '<S67>:1:22' */
    /* '<S67>:1:24' */
    rtb_trav_dist_reset_f = FALSE;

    /*  if self steering gradiend is read form the NVM check with actual
     * calculated values */
    if (ved__sye_B->sf_Get_NVM_IO_State_g.state ==
        ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S67>:1:26' */
        /*  if the difference of the self steering gradien between the last
         * saved NVM SSG and the actual estimated SSG is above 0.1 deg  */
        /*  and the traveled distances is above 30000.0m (30km) */
        if ((((real32_T)fabs(
                 rtb_xpost[2] -
                 (*ved__sye_U_VED_NVData_in).Read.SlfstGrad.SlfStGrad)) >=
             0.00017453F) &&
            (rtb_add_trav_dist > 30000.0F)) {
            /* '<S67>:1:29' */
            /* '<S67>:1:30' */
            rtb_SlfStGradNVM_out = rtb_xpost[2];

            /* '<S67>:1:31' */
            rtb_SlfStGradMax_out = rtb_init_SSG * 25000.0F;

            /* '<S67>:1:32' */
            rtb_SlfStGradMin_out = rtb_T3 * 25000.0F;

            /* '<S67>:1:33' */
            rtb_trav_dist_reset_f = TRUE;

            /* '<S67>:1:34' */
            rtb_state = ((uint32_T)VED_IO_STATE_VALID);
        } else {
            /* '<S67>:1:36' */
            rtb_SlfStGradNVM_out =
                (*ved__sye_U_VED_NVData_in).Read.SlfstGrad.SlfStGrad;

            /* '<S67>:1:37' */
            rtb_SlfStGradMax_out = (real32_T)i;

            /* '<S67>:1:38' */
            rtb_SlfStGradMin_out = (real32_T)rtb_DataTypeConversion3;

            /* '<S67>:1:39' */
            rtb_state = ((uint32_T)VED_IO_STATE_INVALID);
        }
    } else {
        /* '<S67>:1:42' */
        rtb_SlfStGradNVM_out =
            (*ved__sye_U_VED_NVData_in).Read.SlfstGrad.SlfStGrad;

        /* '<S67>:1:43' */
        rtb_SlfStGradMax_out = (real32_T)i;

        /* '<S67>:1:44' */
        rtb_SlfStGradMin_out = (real32_T)rtb_DataTypeConversion3;

        /* '<S67>:1:45' */
        rtb_state = ((uint32_T)VED_IO_STATE_INVALID);
    }

    /*  discret ext self steer grad output */
    if (((real32_T)fabs(rtb_xpost[2] -
                        ved__sye_DWork->last_SlfStGradDisc_DSTATE)) >=
        0.00017453F) {
        /* '<S67>:1:49' */
        /* '<S67>:1:50' */
        rtb_SlfStGradDisc_out = rtb_xpost[2];
    } else {
        /* '<S67>:1:52' */
        rtb_SlfStGradDisc_out = ved__sye_DWork->last_SlfStGradDisc_DSTATE;
    }

    /* BusSelector: '<S45>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_l[i] = (*ved__sye_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S45>/Get_IO_State2' */
    ved__sye_Get_IO_State1(rtb_State_l, &ved__sye_B->Get_IO_State2_k,
                           ((uint32_T)VED_SIN_POS_SWA));

    /* end of Outputs for SubSystem: '<S45>/Get_IO_State2' */

    /* Embedded MATLAB: '<S45>/make_z_vector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'build_Q_R_A_z/make_z_vektor/make_z_vector':
     * '<S61>:1' */
    /*  if the steering wheel angle is available, use the */
    /*  offset compensated steering wheel angle, */
    /*  otherwise set to zero */
    if (((((uint32_T)ved__sye_B->Get_IO_State2_k.IndexVector) ==
          ((uint32_T)VED_IO_STATE_INVALID)) ||
         (((uint32_T)ved__sye_B->Get_IO_State2_k.IndexVector) ==
          ((uint32_T)VED_IO_STATE_NOTAVAIL))) ||
        (((uint32_T)ved__sye_B->Get_IO_State2_k.IndexVector) ==
         ((uint32_T)VED_IO_STATE_INIT))) {
        /* '<S61>:1:5' */
        /* '<S61>:1:6' */
        rtb_init = 0U;

        /* '<S61>:1:7' */
        rtb_z_out_c = 0.0F;
    } else if ((*ved__sye_U_VED_InternalData_in)
                   .ved__offsets_in.ved__swa_offset.state > 1) {
        /* '<S61>:1:9' */
        /* '<S61>:1:10' */
        rtb_init = 1U;

        /* '<S61>:1:11' */
        rtb_z_out_c = (*ved__sye_U_VED_InputData).Signals.StWheelAngle -
                      (*ved__sye_U_VED_InternalData_in)
                          .ved__offsets_in.ved__swa_offset.offset;
    } else {
        /* '<S61>:1:13' */
        rtb_init = 2U;

        /* '<S61>:1:14' */
        rtb_z_out_c = (*ved__sye_U_VED_InputData).Signals.StWheelAngle;
    }

    /* Embedded MATLAB: '<S3>/adapt_R_matrix' */
    /* Embedded MATLAB Function 'R/adapt_R_matrix': '<S25>:1' */
    /*  if the steering wheel angle is invalid */
    /* '<S25>:1:3' */
    rtb_R_out_l = ved__sye_R_p[0];
    if (rtb_init == 0) {
        /* '<S25>:1:4' */
        /* '<S25>:1:5' */
        rtb_R_out_l = ved__sye_R_p[1];
    }

    /* BusSelector: '<S44>/Bus Selector2' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CycleTime_e = (*ved__sye_U_VED_InputData).Frame.CycleTime;

    /* Outputs for atomic SubSystem: '<S44>/Time2Sec' */
    ved__sye_Time2Sec(rtb_CycleTime_e, &ved__sye_B->Time2Sec_b);

    /* end of Outputs for SubSystem: '<S44>/Time2Sec' */

    /* Embedded MATLAB: '<S51>/make_A_matrix' */
    ved__sye_make_A_matrix(ved__sye_B->Time2Sec_b.Time2Sec,
                           &ved__sye_B->sf_make_A_matrix_i);

    /* Embedded MATLAB: '<S12>/hx1' */
    /* Embedded MATLAB Function 'swa_yaw_EKF/hx1': '<S109>:1' */
    /*  calculate the transformation from the state space to the measurement
     * space */
    /* '<S109>:1:3' */
    for (i = 0; i < 2; i++) {
        /* UnitDelay: '<S12>/x_delay_yaw' */
        rtb_z_out_a[i] = ved__sye_DWork->x_delay_yaw_DSTATE[(i)];

        /* Product: '<S12>/Ax' */
        rtb_x_pred_i[i] = 0.0F;
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_x_pred_i[i] =
                (ved__sye_B->sf_make_A_matrix_i
                     .A[(rtb_DataTypeConversion3 << 1) + i] *
                 ved__sye_DWork
                     ->x_delay_yaw_DSTATE[(rtb_DataTypeConversion3)]) +
                rtb_x_pred_i[i];
        }

        rtb_H[i] = (real32_T)i;
    }

    /* '<S109>:1:4' */
    rtb_T3 = 0.0F;
    rtb_DataTypeConversion3 = 1;
    i = 1;

    rtb_rat[0] =
        (ved__sye_B->Time2Sec_b.Time2Sec * ved__sye_B->Time2Sec_b.Time2Sec) /
        2.0F;
    rtb_rat[1] = ved__sye_B->Time2Sec_b.Time2Sec;

    /* Embedded MATLAB: '<S52>/makeQ' incorporates:
     *  Constant: '<S52>/sigma_model'
     */
    /* Embedded MATLAB Function 'build_Q_R_A_z/Q_A_adaptation_yaw/Q/makeQ':
     * '<S56>:1' */
    /*  calculate the model covariance matrix Q */
    /* '<S56>:1:3' */
    for (k = 0; k < 2; k++) {
        rtb_T3 += ((real32_T)((int8_T)(rtb_DataTypeConversion3 - 1))) *
                  rtb_x_pred_i[i - 1];
        rtb_DataTypeConversion3++;
        i++;
        rtb_rat_0[k] = rtb_rat[k] * ved__sye_Q_sigmas_p[(k)];
        rtb_z_out_a_0[k] = ved__sye_Q_sigmas_p[(k)] * rtb_rat[k];
    }

    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_R_out[rtb_DataTypeConversion3 + (i << 1)] =
                rtb_rat_0[rtb_DataTypeConversion3] * rtb_z_out_a_0[i];
        }
    }

    /* UnitDelay: '<S58>/T2' */
    rtb_T2_e = ved__sye_DWork->T2_DSTATE_f;

    /* UnitDelay: '<S58>/T1' */
    rtb_T1_p = ved__sye_DWork->T1_DSTATE_k;

    /* UnitDelay: '<S58>/T0' */
    rtb_T0_a = ved__sye_DWork->T0_DSTATE_f;

    /* Product: '<S58>/Divide' incorporates:
     *  Constant: '<S58>/filter_length'
     *  Sum: '<S58>/Add'
     *  UnitDelay: '<S58>/T3'
     */
    rtb_init_SSG =
        (((ved__sye_DWork->T3_DSTATE_c + ved__sye_DWork->T2_DSTATE_f) +
          ved__sye_DWork->T1_DSTATE_k) +
         ved__sye_DWork->T0_DSTATE_f) *
        0.25F;

    /* Embedded MATLAB: '<S54>/get_gain_bias' */
    ved__sye_get_gain_bias((&(ved__sye_Q_gain_p[0])),
                           &ved__sye_B->sf_get_gain_bias);

    /* Embedded MATLAB: '<S54>/q_gain_delta' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function
     * 'build_Q_R_A_z/Q_A_adaptation_yaw/q_gain/q_gain_delta': '<S59>:1' */
    /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition below
     * is rewritten with multiple if-elseif expressions. */
    /*  calculate the difference between the mean filtered last yaw rate and  */
    /*  the actual yaw rate */
    if (((real32_T)fabs(
            (*ved__sye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate)) >=
        1.0E-7F) {
        /* '<S59>:1:6' */
        if (((real32_T)fabs(rtb_init_SSG)) >= 1.0E-7F) {
            /* '<S59>:1:7' */
            /* '<S59>:1:8' */
            rtb_init_SSG =
                (*ved__sye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate -
                rtb_init_SSG;
        } else {
            /* '<S59>:1:10' */
            rtb_init_SSG = ved__sye_B->sf_get_gain_bias.default_diff;
        }
    } else {
        /* '<S59>:1:13' */
        rtb_init_SSG = ved__sye_B->sf_get_gain_bias.default_diff;
    }

    /* Sum: '<S54>/Sum2' incorporates:
     *  Abs: '<S54>/Abs'
     *  Product: '<S54>/Product'
     */
    rtb_init_SSG =
        (((real32_T)fabs(rtb_init_SSG)) * ved__sye_B->sf_get_gain_bias.gain) +
        ved__sye_B->sf_get_gain_bias.bias;
    for (i = 0; i < 4; i++) {
        /* UnitDelay: '<S12>/P_delay_yaw' */
        rtb_APAt[i] = ved__sye_DWork->P_delay_yaw_DSTATE[(i)];
        rtb_R_out[i] = (rtb_R_out[i] * rtb_init_SSG) + ved__sye_Q_add_p[(i)];
    }

    /* Embedded MATLAB: '<S12>/At' */
    ved__sye_At(ved__sye_B->sf_make_A_matrix_i.A, &ved__sye_B->sf_At_d);

    /* Sum: '<S12>/APA_Q' incorporates:
     *  Product: '<S12>/APAt'
     */
    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_rat_1[i + (rtb_DataTypeConversion3 << 1)] = 0.0F;
            for (k = 0; k < 2; k++) {
                rtb_rat_1[i + (rtb_DataTypeConversion3 << 1)] =
                    (ved__sye_B->sf_make_A_matrix_i.A[(k << 1) + i] *
                     rtb_APAt[(rtb_DataTypeConversion3 << 1) + k]) +
                    rtb_rat_1[(rtb_DataTypeConversion3 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_Divide = 0.0F;
            for (k = 0; k < 2; k++) {
                rtb_Divide +=
                    rtb_rat_1[(k << 1) + i] *
                    ved__sye_B->sf_At_d.y[(rtb_DataTypeConversion3 << 1) + k];
            }

            rtb_P_pred_a[i + (rtb_DataTypeConversion3 << 1)] =
                rtb_R_out[(rtb_DataTypeConversion3 << 1) + i] + rtb_Divide;
        }
    }

    /* Embedded MATLAB: '<S12>/Ht' */
    ved__sye_Ht(rtb_H, &ved__sye_B->sf_Ht_d);

    /* Sum: '<S12>/HPH_R' incorporates:
     *  Product: '<S12>/HPHt'
     */
    rtb_Divide = 0.0F;
    for (i = 0; i < 2; i++) {
        rtb_z_out_a_0[i] = 0.0F;
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_z_out_a_0[i] =
                (rtb_P_pred_a[(i << 1) + rtb_DataTypeConversion3] *
                 rtb_H[rtb_DataTypeConversion3]) +
                rtb_z_out_a_0[i];
        }

        rtb_Divide += rtb_z_out_a_0[i] * ved__sye_B->sf_Ht_d.y[(i)];

        /* Product: '<S12>/P_pred*Ht' */
        rtb_P_predHt_b[i] = 0.0F;
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_P_predHt_b[i] =
                (rtb_P_pred_a[(rtb_DataTypeConversion3 << 1) + i] *
                 ved__sye_B->sf_Ht_d.y[(rtb_DataTypeConversion3)]) +
                rtb_P_predHt_b[i];
        }
    }

    rtb_HPHt_R_l = rtb_R_out_l + rtb_Divide;

    /* Embedded MATLAB: '<S104>/calculate determinant' */
    ved__sye_calculatedeterminant(rtb_HPHt_R_l,
                                  &ved__sye_B->sf_calculatedeterminant_e);

    /* If: '<S104>/If' incorporates:
     *  ActionPort: '<S112>/Action Port'
     *  ActionPort: '<S113>/Action Port'
     *  SubSystem: '<S104>/calculate the gain'
     *  SubSystem: '<S104>/set gain to default value'
     */
    if (ved__sye_B->sf_calculatedeterminant_e.y > 1.0E-16F) {
        ved__sye_calculatethegain(rtb_P_predHt_b, rtb_HPHt_R_l, rtb_K_l);
    } else {
        /* Constant: '<S113>/Constant' */
        for (i = 0; i < 2; i++) {
            rtb_K_l[i] = ved__sye_ConstP.pooled2[(i)];
        }
    }

    /* Product: '<S12>/K*H' */
    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_KH_a[rtb_DataTypeConversion3 + (i << 1)] =
                rtb_K_l[rtb_DataTypeConversion3] * rtb_H[i];
        }
    }

    /* Embedded MATLAB: '<S12>/eye' */
    ved__sye_eye(&ved__sye_B->sf_eye_d);

    /* Sum: '<S12>/1_KH' */
    for (i = 0; i < 4; i++) {
        rtb_R_out[i] = ved__sye_B->sf_eye_d.y[(i)] - rtb_KH_a[i];
    }

    /* Embedded MATLAB: '<S12>/Reset_P_pred' */
    ved__sye_Reset_P_pred(rtb_P_pred_a, ved__sye_B->sf_calculatedeterminant_e.y,
                          &ved__sye_B->sf_Reset_P_pred_d);

    /* Product: '<S12>/P_pred_(1_KH)' */
    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_APAt[i + (rtb_DataTypeConversion3 << 1)] = 0.0F;
            for (k = 0; k < 2; k++) {
                rtb_APAt[i + (rtb_DataTypeConversion3 << 1)] =
                    (rtb_R_out[(k << 1) + i] *
                     ved__sye_B->sf_Reset_P_pred_d
                         .P_pred_out[(rtb_DataTypeConversion3 << 1) + k]) +
                    rtb_APAt[(rtb_DataTypeConversion3 << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/diag_yaw_variance' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'diag_yaw_variance': '<S8>:1' */
    /*  tune output variances */
    if (rtb_init >= 1) {
        /* '<S8>:1:3' */
        /* '<S8>:1:4' */
        for (i = 0; i < 2; i++) {
            rtb_y_hc[i] = rtb_APAt[3 * i];
        }

        /*  if no swa offset is available add 20 perc to all variances */
        if (rtb_init == 2) {
            /* '<S8>:1:6' */
            /* '<S8>:1:7' */
            for (i = 0; i < 2; i++) {
                rtb_y_hc[i] = rtb_APAt[3 * i] * 1.2F;
            }
        }

        if ((*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo <=
            ved__sye_P_correct_p[1]) {
            /* '<S8>:1:10' */
            /* '<S8>:1:11' */
            for (i = 0; i < 2; i++) {
                rtb_y_hc[i] =
                    (((rtb_y_hc[i] - ved__sye_P_correct_p[2]) /
                      ved__sye_P_correct_p[1]) *
                     (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo) +
                    ved__sye_P_correct_p[2];
            }
        }

        /* '<S8>:1:13' */
        for (i = 0; i < 2; i++) {
            rtb_y_hc[i] = rtb_y_hc[i] + ved__sye_P_correct_p[3];
        }
    } else {
        /*  if input values are not valid set output variance to high value */
        /* '<S8>:1:16' */
        for (i = 0; i < 2; i++) {
            rtb_y_hc[i] = ved__sye_P_correct_p[0];
        }
    }

    /* Embedded MATLAB: '<S12>/Reset_x_pred' */
    /* Embedded MATLAB Function 'swa_yaw_EKF/Reset_x_pred': '<S107>:1' */
    if (((real32_T)fabs(ved__sye_B->sf_calculatedeterminant_e.y)) <= 1.0E-16F) {
        /* '<S107>:1:4' */
        /* '<S107>:1:6' */
        rtb_x_pred_i[0] = 0.0F;
        rtb_x_pred_i[1] = rtb_z_out_a[1];
    } else {
        /* '<S107>:1:10' */
    }

    /* Embedded MATLAB: '<S12>/make_z' incorporates:
     *  BusSelector: '<S12>/Bus Selector'
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'swa_yaw_EKF/make_z': '<S110>:1' */
    /*  calculate the swa yaw rate with the parameters and the offset comp swa
     */
    /*  if the learned understeer gradient should be used */
    if (((uint32_T)VED__USE_LEARNED_UNDERSTEER_GRAD) == 1U) {
        /* '<S110>:1:6' */
        /*  if the vehicle velocity is not zero */
        if (((real32_T)fabs(
                (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo)) >=
            0.0001F) {
            /* '<S110>:1:8' */
            /* '<S110>:1:9' */
            /*  the meas input swa yaw rate */
            /* '<S110>:1:11' */
            rtb_init_SSG =
                ((((*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo *
                   (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo) *
                  rtb_SlfStGradDisc_out) +
                 (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
                (*ved__sye_U_VED_InputData).Parameter.SteeringRatio.swa.rat[1];
            if (((real32_T)fabs(rtb_init_SSG)) <= 0.0001F) {
                /* '<S110>:1:12' */
                /* '<S110>:1:13' */
                rtb_init_SSG = 0.0F;
            } else {
                /* '<S110>:1:15' */
                rtb_init_SSG =
                    (rtb_z_out_c *
                     (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo) /
                    rtb_init_SSG;
            }
        } else {
            /* '<S110>:1:18' */
            rtb_init_SSG = 0.0F;
        }
    } else {
        /*  if the understeer gradient from the parameter input struct should be
         * used */
        /*  if the vehicle velocity is not zero */
        if (((real32_T)fabs(
                (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo)) >=
            0.0001F) {
            /* '<S110>:1:23' */
            /* '<S110>:1:24' */
            /*  the meas input swa yaw rate */
            /* '<S110>:1:26' */
            rtb_init_SSG =
                ((((*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo *
                   (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo) *
                  (*ved__sye_U_VED_InputData)
                      .Parameter.VED_Kf_SelfSteerGrad_nu) +
                 (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
                (*ved__sye_U_VED_InputData).Parameter.SteeringRatio.swa.rat[1];
            if (((real32_T)fabs(rtb_init_SSG)) <= 0.0001F) {
                /* '<S110>:1:27' */
                /* '<S110>:1:28' */
                rtb_init_SSG = 0.0F;
            } else {
                /* '<S110>:1:30' */
                rtb_init_SSG =
                    (rtb_z_out_c *
                     (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo) /
                    rtb_init_SSG;
            }
        } else {
            /* '<S110>:1:33' */
            rtb_init_SSG = 0.0F;
        }
    }

    /* Sum: '<S12>/z_Hx' */
    rtb_init_SSG -= rtb_T3;

    /* Sum: '<S12>/x_Knu' incorporates:
     *  Product: '<S12>/Knu'
     */
    for (i = 0; i < 2; i++) {
        rtb_x_pred_i[i] = (rtb_K_l[i] * rtb_init_SSG) + rtb_x_pred_i[i];
    }

    /* Embedded MATLAB: '<Root>/correct_yaw_rate' */
    /* Embedded MATLAB Function 'correct_yaw_rate': '<S6>:1' */
    /*  correct output yaw rate */
    if (rtb_init >= 1) {
        /* '<S6>:1:3' */
        /* '<S6>:1:4' */
        rtb_y_o = rtb_x_pred_i[1];
    } else {
        /* '<S6>:1:6' */
        rtb_y_o = 0.0F;
    }

    /* BusSelector: '<S43>/Bus Selector1' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CtrlMode_j = (*ved__sye_U_VED_InputData).Frame.CtrlMode;

    /* Embedded MATLAB: '<S43>/get_init_control_mode' */
    ved__sye_get_init_control_mode(rtb_CtrlMode_j,
                                   &ved__sye_B->sf_get_init_control_mode_g);

    /* Abs: '<S43>/Abs' incorporates:
     *  Sum: '<S43>/Add'
     *  UnitDelay: '<S43>/last_steering_angle'
     */
    rtb_init_SSG = (real32_T)fabs(rtb_z_out_c -
                                  ved__sye_DWork->last_steering_angle_DSTATE);

    /* Embedded MATLAB: '<S43>/check_for_zero_values' incorporates:
     *  UnitDelay: '<S43>/last_not_zero_min'
     */
    /* Embedded MATLAB Function
     * 'build_Q_R_A_z/CaculateAddVar/check_for_zero_values': '<S48>:1' */
    /*  check if the input DiffAbs steering wheel angle is not zero */
    /*  if it is zero use the last min Value as output */
    if (rtb_init_SSG <= 0.0001F) {
        /* '<S48>:1:4' */
        /* '<S48>:1:5' */
        rtb_init_SSG = ved__sye_DWork->last_not_zero_min_DSTATE;
    } else {
        /* '<S48>:1:7' */
    }

    /* Switch: '<S50>/Reset' incorporates:
     *  Constant: '<S50>/Initial Condition'
     *  MinMax: '<S47>/MinMax'
     *  UnitDelay: '<S50>/FixPt Unit Delay1'
     */
    if (ved__sye_B->sf_get_init_control_mode_g.y) {
        rtb_Reset_l = 10.0F;
    } else {
        rtb_Reset_l = (rtb_init_SSG <= ved__sye_DWork->FixPtUnitDelay1_DSTATE_l)
                          ? rtb_init_SSG
                          : ved__sye_DWork->FixPtUnitDelay1_DSTATE_l;
    }

    /* Embedded MATLAB: '<S43>/AddVariance' incorporates:
     *  BusSelector: '<S43>/Bus Selector'
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'build_Q_R_A_z/CaculateAddVar/AddVariance':
     * '<S46>:1' */
    /*  calculate the steering wheel curve for the resolution of the swa sensor
     */
    /*  if the learned understeer gradient should be used */
    if (((uint32_T)VED__USE_LEARNED_UNDERSTEER_GRAD) == 1U) {
        /* '<S46>:1:4' */
        /* '<S46>:1:5' */
        /*  Estimation of the variance to 3*SteeringAngleResolution */
        /* '<S46>:1:7' */
        rtb_init_SSG =
            ((((*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo *
               (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo) *
              rtb_SlfStGradDisc_out) +
             (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
            (*ved__sye_U_VED_InputData).Parameter.SteeringRatio.swa.rat[1];
        if (((real32_T)fabs(rtb_init_SSG)) >= 0.0001F) {
            /* '<S46>:1:8' */
            /* '<S46>:1:9' */
            rtb_addVariance = (rtb_Reset_l / rtb_init_SSG) * 3.0F;
        } else {
            /* '<S46>:1:11' */
            rtb_addVariance = 0.002F;
        }
    } else {
        /*  if the understeer gradient from the parameter input struct should be
         * used */
        /* '<S46>:1:15' */
        /*  Estimation of the variance to 3*SteeringAngleResolution */
        /* '<S46>:1:17' */
        rtb_init_SSG =
            ((((*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo *
               (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo) *
              (*ved__sye_U_VED_InputData).Parameter.VED_Kf_SelfSteerGrad_nu) +
             (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
            (*ved__sye_U_VED_InputData).Parameter.SteeringRatio.swa.rat[1];
        if (((real32_T)fabs(rtb_init_SSG)) >= 0.0001F) {
            /* '<S46>:1:18' */
            /* '<S46>:1:19' */
            rtb_addVariance = (rtb_Reset_l / rtb_init_SSG) * 3.0F;
        } else {
            /* '<S46>:1:21' */
            rtb_addVariance = 0.002F;
        }
    }

    /* '<S46>:1:24' */
    rtb_addVariance *= rtb_addVariance;

    /* BusSelector: '<S11>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 2; i++) {
        rtb_rat[i] =
            (*ved__sye_U_VED_InputData).Parameter.SteeringRatio.swa.rat[(i)];
    }

    /* BusSelector: '<S2>/Bus Selector3' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CycleTime_k = (*ved__sye_U_VED_InputData).Frame.CycleTime;

    /* Outputs for atomic SubSystem: '<S2>/Time2Sec' */
    ved__sye_Time2Sec(rtb_CycleTime_k, &ved__sye_B->Time2Sec_f);

    /* end of Outputs for SubSystem: '<S2>/Time2Sec' */

    /* Embedded MATLAB: '<S13>/make_A_matrix' */
    ved__sye_make_A_matrix(ved__sye_B->Time2Sec_f.Time2Sec,
                           &ved__sye_B->sf_make_A_matrix_d);
    for (i = 0; i < 2; i++) {
        /* UnitDelay: '<S11>/x_delay_curve' */
        rtb_hx_g[i] = ved__sye_DWork->x_delay_curve_DSTATE[(i)];

        /* Product: '<S11>/Ax' */
        rtb_x_pred_n[i] = 0.0F;
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_x_pred_n[i] =
                (ved__sye_B->sf_make_A_matrix_d
                     .A[(rtb_DataTypeConversion3 << 1) + i] *
                 ved__sye_DWork
                     ->x_delay_curve_DSTATE[(rtb_DataTypeConversion3)]) +
                rtb_x_pred_n[i];
        }
    }

    /* Embedded MATLAB: '<S11>/hx' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'swa_curve_EKF/hx': '<S99>:1' */
    /*  calculate transformation from state space to measurement space */
    /*  if the learned understeer gradient should be used */
    if (((uint32_T)VED__USE_LEARNED_UNDERSTEER_GRAD) == 1U) {
        /* '<S99>:1:4' */
        /* '<S99>:1:5' */
        rtb_init_SSG = (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo *
                       (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo;

        /*  non linear equation */
        /* '<S99>:1:7' */
        rtb_hx_d =
            ((rtb_SlfStGradDisc_out * rtb_init_SSG) +
             (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
            (rtb_rat[1] * rtb_x_pred_n[0]);

        /*  linearised transformation matrix H */
        /* '<S99>:1:9' */
        rtb_H_p[0] =
            ((rtb_SlfStGradDisc_out * rtb_init_SSG) +
             (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
            rtb_rat[1];
        rtb_H_p[1] = 0.0F;
    } else {
        /*  if the understeer gradient from the parameter input struct should be
         * used */
        /* '<S99>:1:12' */
        rtb_init_SSG = (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo *
                       (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo;

        /*  non linear equation */
        /* '<S99>:1:14' */
        rtb_hx_d =
            (((*ved__sye_U_VED_InputData).Parameter.VED_Kf_SelfSteerGrad_nu *
              rtb_init_SSG) +
             (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
            (rtb_rat[1] * rtb_x_pred_n[0]);

        /*  linearised transformation matrix H */
        /* '<S99>:1:16' */
        rtb_H_p[0] =
            (((*ved__sye_U_VED_InputData).Parameter.VED_Kf_SelfSteerGrad_nu *
              rtb_init_SSG) +
             (*ved__sye_U_VED_InputData).Parameter.VED_Kf_WheelBase_met) *
            rtb_rat[1];
        rtb_H_p[1] = 0.0F;
    }

    /* Embedded MATLAB: '<S19>/sigma_velocity_gain_controll' incorporates:
     *  Constant: '<S19>/sigma_model'
     *  Constant: '<S19>/sigma_velo_gain'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function
     * 'Q_A_addpation_curve/Q/gain_sigmas_over_velocity/sigma_velocity_gain_controll':
     * '<S21>:1' */
    /*  if gain controll over velocity is selected */
    if (ved__sye_Q_di_sigmas_velo_gain_p[0] == 1.0F) {
        /* '<S21>:1:3' */
        /*     if velocity is  below the max gain controled velocity */
        if ((*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo <=
            ved__sye_Q_di_sigmas_velo_gain_p[3]) {
            /* '<S21>:1:5' */
            /*  calculate the new sigmas controled by the velocity */
            /* '<S21>:1:7' */
            /* '<S21>:1:8' */
            rtb_init_SSG =
                ((((ved__sye_Q_di_sigmas_velo_gain_p[1] -
                    ved__sye_Q_di_sigmas_velo_gain_p[2]) /
                   (ved__sye_Q_di_sigmas_velo_gain_p[3] *
                    ved__sye_Q_di_sigmas_velo_gain_p[3])) *
                  ((*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                   ved__sye_Q_di_sigmas_velo_gain_p[3])) *
                 ((*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                  ved__sye_Q_di_sigmas_velo_gain_p[3])) +
                ved__sye_Q_di_sigmas_velo_gain_p[2];

            /* '<S21>:1:9' */
            rtb_rat[0] = rtb_init_SSG;
            rtb_rat[1] = rtb_init_SSG;
        } else {
            /*  constant sigma if above the threshold velocity */
            /* '<S21>:1:12' */
            rtb_rat[0] = ved__sye_Q_di_sigmas_velo_gain_p[2];
            rtb_rat[1] = ved__sye_Q_di_sigmas_velo_gain_p[2];
        }
    } else {
        /*  not velocity gain controlled */
        /* '<S21>:1:16' */
        for (i = 0; i < 2; i++) {
            rtb_rat[i] = ved__sye_Q_di_sigmas_p[(i)];
        }
    }

    rtb_z_out_a[0] =
        (ved__sye_B->Time2Sec_f.Time2Sec * ved__sye_B->Time2Sec_f.Time2Sec) /
        2.0F;
    rtb_z_out_a[1] = ved__sye_B->Time2Sec_f.Time2Sec;

    /* Embedded MATLAB: '<S14>/makeQ' */
    /* Embedded MATLAB Function 'Q_A_addpation_curve/Q/makeQ': '<S20>:1' */
    /*  calculate the model covariance matrix Q */
    /* '<S20>:1:3' */

    /* Embedded MATLAB: '<S2>/divide' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'Q_A_addpation_curve/divide': '<S16>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((real32_T)fabs(
            (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo)) >=
        0.0001F) {
        /* '<S16>:1:5' */
        /* '<S16>:1:6' */
        rtb_init_SSG =
            (*ved__sye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate /
            (*ved__sye_U_VED_InternalData_in).ved__ve_out.veh_velo;
    } else {
        /* '<S16>:1:8' */
        rtb_init_SSG = 0.0F;
    }

    /* UnitDelay: '<S23>/T2' */
    rtb_T2_m = ved__sye_DWork->T2_DSTATE_e;

    /* UnitDelay: '<S23>/T1' */
    rtb_T1_f = ved__sye_DWork->T1_DSTATE_h;

    /* UnitDelay: '<S23>/T0' */
    rtb_T0_k = ved__sye_DWork->T0_DSTATE_l;

    /* Product: '<S23>/Divide' incorporates:
     *  Constant: '<S23>/filter_length'
     *  Sum: '<S23>/Add'
     *  UnitDelay: '<S23>/T3'
     */
    rtb_T3 = (((ved__sye_DWork->T3_DSTATE_f + ved__sye_DWork->T2_DSTATE_e) +
               ved__sye_DWork->T1_DSTATE_h) +
              ved__sye_DWork->T0_DSTATE_l) *
             0.25F;

    /* Embedded MATLAB: '<S17>/get_gain_bias' */
    ved__sye_get_gain_bias((&(ved__sye_Q_di_gain_p[0])),
                           &ved__sye_B->sf_get_gain_bias_k);

    /* Embedded MATLAB: '<S17>/q_gain_delta' */
    /* Embedded MATLAB Function 'Q_A_addpation_curve/q_gain/q_gain_delta':
     * '<S24>:1' */
    /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition below
     * is rewritten with multiple if-elseif expressions. */
    /*  calculate the difference between the mean filtered last yaw rate and  */
    /*  the actual yaw rate */
    if (((real32_T)fabs(rtb_init_SSG)) >= 1.0E-7F) {
        /* '<S24>:1:6' */
        if (((real32_T)fabs(rtb_T3)) >= 1.0E-7F) {
            /* '<S24>:1:7' */
            /* '<S24>:1:8' */
            rtb_init_SSG -= rtb_T3;
        } else {
            /* '<S24>:1:10' */
            rtb_init_SSG = ved__sye_B->sf_get_gain_bias_k.default_diff;
        }
    } else {
        /* '<S24>:1:13' */
        rtb_init_SSG = ved__sye_B->sf_get_gain_bias_k.default_diff;
    }

    /* Sum: '<S17>/Sum2' incorporates:
     *  Abs: '<S17>/Abs'
     *  Product: '<S17>/Product'
     */
    rtb_init_SSG =
        (((real32_T)fabs(rtb_init_SSG)) * ved__sye_B->sf_get_gain_bias_k.gain) +
        ved__sye_B->sf_get_gain_bias_k.bias;

    /* Sum: '<S14>/Sum' incorporates:
     *  Constant: '<S14>/additional'
     *  Product: '<S14>/Product'
     */
    for (i = 0; i < 2; i++) {
        rtb_z_out_a_0[i] = rtb_z_out_a[i] * rtb_rat[i];
        rtb_rat_0[i] = rtb_rat[i] * rtb_z_out_a[i];
    }

    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_rat_1[rtb_DataTypeConversion3 + (i << 1)] =
                rtb_z_out_a_0[rtb_DataTypeConversion3] * rtb_rat_0[i];
        }
    }

    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_Sum_c[rtb_DataTypeConversion3 + (i << 1)] =
                (rtb_rat_1[(i << 1) + rtb_DataTypeConversion3] * rtb_init_SSG) +
                ved__sye_Q_di_add_p[(i << 1) + rtb_DataTypeConversion3];
        }
    }

    /* UnitDelay: '<S11>/P_delay_curve' */
    for (i = 0; i < 4; i++) {
        rtb_R_out[i] = ved__sye_DWork->P_delay_curve_DSTATE[(i)];
    }

    /* Embedded MATLAB: '<S11>/At' */
    ved__sye_At(ved__sye_B->sf_make_A_matrix_d.A, &ved__sye_B->sf_At_dl);

    /* Sum: '<S11>/APA_Q' incorporates:
     *  Product: '<S11>/APAt'
     */
    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_rat_1[i + (rtb_DataTypeConversion3 << 1)] = 0.0F;
            for (k = 0; k < 2; k++) {
                rtb_rat_1[i + (rtb_DataTypeConversion3 << 1)] =
                    (ved__sye_B->sf_make_A_matrix_d.A[(k << 1) + i] *
                     rtb_R_out[(rtb_DataTypeConversion3 << 1) + k]) +
                    rtb_rat_1[(rtb_DataTypeConversion3 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_Divide = 0.0F;
            for (k = 0; k < 2; k++) {
                rtb_Divide +=
                    rtb_rat_1[(k << 1) + i] *
                    ved__sye_B->sf_At_dl.y[(rtb_DataTypeConversion3 << 1) + k];
            }

            rtb_P_pred_d[i + (rtb_DataTypeConversion3 << 1)] =
                rtb_Sum_c[(rtb_DataTypeConversion3 << 1) + i] + rtb_Divide;
        }
    }

    /* Embedded MATLAB: '<S11>/Ht' */
    ved__sye_Ht(rtb_H_p, &ved__sye_B->sf_Ht_k);

    /* Sum: '<S11>/HPH_R' incorporates:
     *  Product: '<S11>/HPHt'
     */
    rtb_Divide = 0.0F;
    for (i = 0; i < 2; i++) {
        rtb_z_out_a_0[i] = 0.0F;
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_z_out_a_0[i] =
                (rtb_P_pred_d[(i << 1) + rtb_DataTypeConversion3] *
                 rtb_H_p[rtb_DataTypeConversion3]) +
                rtb_z_out_a_0[i];
        }

        rtb_Divide += rtb_z_out_a_0[i] * ved__sye_B->sf_Ht_k.y[(i)];

        /* Product: '<S11>/P_pred*Ht' */
        rtb_P_predHt_h[i] = 0.0F;
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_P_predHt_h[i] =
                (rtb_P_pred_d[(rtb_DataTypeConversion3 << 1) + i] *
                 ved__sye_B->sf_Ht_k.y[(rtb_DataTypeConversion3)]) +
                rtb_P_predHt_h[i];
        }
    }

    rtb_HPHt_R_n = rtb_R_out_l + rtb_Divide;

    /* Embedded MATLAB: '<S94>/calculate determinant' */
    ved__sye_calculatedeterminant(rtb_HPHt_R_n,
                                  &ved__sye_B->sf_calculatedeterminant_k);

    /* If: '<S94>/If' incorporates:
     *  ActionPort: '<S101>/Action Port'
     *  ActionPort: '<S102>/Action Port'
     *  SubSystem: '<S94>/calculate the gain'
     *  SubSystem: '<S94>/set gain to default value'
     */
    if (ved__sye_B->sf_calculatedeterminant_k.y > 1.0E-16F) {
        ved__sye_calculatethegain(rtb_P_predHt_h, rtb_HPHt_R_n, rtb_K_ld);
    } else {
        /* Constant: '<S102>/Constant' */
        for (i = 0; i < 2; i++) {
            rtb_K_ld[i] = ved__sye_ConstP.pooled2[(i)];
        }
    }

    /* Product: '<S11>/K*H' */
    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            rtb_KH_m[rtb_DataTypeConversion3 + (i << 1)] =
                rtb_K_ld[rtb_DataTypeConversion3] * rtb_H_p[i];
        }
    }

    /* Embedded MATLAB: '<S11>/eye' */
    ved__sye_eye(&ved__sye_B->sf_eye_g);

    /* Sum: '<S11>/1_KH' */
    for (i = 0; i < 4; i++) {
        rtb_R_out[i] = ved__sye_B->sf_eye_g.y[(i)] - rtb_KH_m[i];
    }

    /* Embedded MATLAB: '<S11>/Reset_P_pred' */
    ved__sye_Reset_P_pred(rtb_P_pred_d, ved__sye_B->sf_calculatedeterminant_k.y,
                          &ved__sye_B->sf_Reset_P_pred_m);

    /* Product: '<S11>/P_pred_(1_KH)' */
    for (i = 0; i < 2; i++) {
        for (rtb_DataTypeConversion3 = 0; rtb_DataTypeConversion3 < 2;
             rtb_DataTypeConversion3++) {
            ved__sye_DWork
                ->P_delay_curve_DSTATE[i + (rtb_DataTypeConversion3 << 1)] =
                0.0F;
            for (k = 0; k < 2; k++) {
                ved__sye_DWork
                    ->P_delay_curve_DSTATE[i + (rtb_DataTypeConversion3 << 1)] =
                    (rtb_R_out[(k << 1) + i] *
                     ved__sye_B->sf_Reset_P_pred_m
                         .P_pred_out[(rtb_DataTypeConversion3 << 1) + k]) +
                    ved__sye_DWork
                        ->P_delay_curve_DSTATE[(rtb_DataTypeConversion3 << 1) +
                                               i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/diag_curve_variance' */
    /* Embedded MATLAB Function 'diag_curve_variance': '<S7>:1' */
    /*  tune output variances */
    if (rtb_init >= 1) {
        /* '<S7>:1:3' */
        /* '<S7>:1:4' */
        for (i = 0; i < 2; i++) {
            rtb_rat[i] = ved__sye_DWork->P_delay_curve_DSTATE[(3 * i)];
        }

        /*  if swa offset is not available add 20 perc to all variances */
        if (rtb_init == 2) {
            /* '<S7>:1:6' */
            /* '<S7>:1:7' */
            for (i = 0; i < 2; i++) {
                rtb_rat[i] =
                    ved__sye_DWork->P_delay_curve_DSTATE[(3 * i)] * 1.2F;
            }
        }
    } else {
        /*  if input values are not valid set output variance to high value */
        /* '<S7>:1:11' */
        for (i = 0; i < 2; i++) {
            rtb_rat[i] = ved__sye_P_correct_p[0];
        }
    }

    /* Embedded MATLAB: '<S11>/Reset_x_pred' */
    /* Embedded MATLAB Function 'swa_curve_EKF/Reset_x_pred': '<S97>:1' */
    if (((real32_T)fabs(ved__sye_B->sf_calculatedeterminant_k.y)) <= 1.0E-16F) {
        /* '<S97>:1:4' */
        /* '<S97>:1:6' */
        for (i = 0; i < 2; i++) {
            rtb_x_pred_n[i] = rtb_hx_g[i];
        }
    } else {
        /* '<S97>:1:10' */
    }

    /* Sum: '<S11>/z_Hx' */
    rtb_init_SSG = rtb_z_out_c - rtb_hx_d;

    /* Sum: '<S11>/x_Knu' incorporates:
     *  Product: '<S11>/Knu'
     */
    for (i = 0; i < 2; i++) {
        ved__sye_DWork->x_delay_curve_DSTATE[(i)] =
            (rtb_K_ld[i] * rtb_init_SSG) + rtb_x_pred_n[i];
    }

    /* BusAssignment: '<Root>/Bus Assignment3' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  Sum: '<Root>/Add3'
     */
    (*ved__sye_Y_VED_InternalData_out) = (*ved__sye_U_VED_InternalData_in);
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_understeer_grad_var =
        rtb_A[8];
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_understeer_grad_max =
        rtb_Reset;
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_understeer_grad_min =
        rtb_Reset_m;
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_understeer_grad =
        rtb_xpost[2];
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_understeer_grad_disc =
        rtb_SlfStGradDisc_out;
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_understeer_control =
        rtb_EGDyn;
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_yaw_rate_var =
        rtb_y_hc[1];
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_yaw_rate = rtb_y_o;
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_curve_var =
        rtb_addVariance + rtb_rat[0];
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_curve_grad_var =
        rtb_rat[1];
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_curve =
        ved__sye_DWork->x_delay_curve_DSTATE[0];
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.stw_curve_grad =
        ved__sye_DWork->x_delay_curve_DSTATE[1];
    (*ved__sye_Y_VED_InternalData_out).ved__sye_out.r_On_Off_control = rtb_init;

    /* Embedded MATLAB: '<S71>/Get_IO_State' incorporates:
     *  Constant: '<S62>/VED__SIN_POS'
     *  Constant: '<S71>/VED_IO_STATE_BITMASK'
     */
    /* Embedded MATLAB Function
     * 'extract_understeer_grad/Get_IO_State3/GetIOState/Get_IO_State':
     * '<S72>:1' */
    /*  set the value at pos in the state array */
    /* '<S72>:1:4' */
    c = ((uint32_T)VED_NVM_POS_SSG);
    if (((uint32_T)VED_NVM_POS_SSG) > 2147483647U) {
        c = 2147483647U;
    }

    i = (int32_T)c;
    rtb_DataTypeConversion3 = i - ((i >> 5) << 5);

    /* '<S72>:1:5' */
    /* '<S72>:1:6' */
    c = 0U;
    if (rtb_DataTypeConversion3 < 0) {
        i = -rtb_DataTypeConversion3;
        if (i > 255) {
            i = 255;
        }

        rtb_init = (uint8_T)i;
        if (rtb_init < 32) {
            c = (3U >> ((uint32_T)rtb_init));
        }
    } else {
        i = rtb_DataTypeConversion3;
        if (rtb_DataTypeConversion3 <= 0) {
            i = 0;
        } else {
            if (rtb_DataTypeConversion3 > 255) {
                i = 255;
            }
        }

        rtb_init = (uint8_T)i;
        if (rtb_init < 32) {
            c = (3U << ((uint32_T)rtb_init));
        }
    }

    /* '<S72>:1:7' */
    /* '<S72>:1:8' */

    (*ved__sye_Y_VED_NVData_out) = (*ved__sye_U_VED_NVData_in);
    (*ved__sye_Y_VED_NVData_out).Write.State =
        ((c ^ MAX_uint32_T) & (*ved__sye_U_VED_NVData_in).Write.State) |
        (rtb_state << rtb_DataTypeConversion3);
    (*ved__sye_Y_VED_NVData_out).Write.SlfstGrad.SlfStGrad =
        rtb_SlfStGradNVM_out;
    (*ved__sye_Y_VED_NVData_out).Write.SlfstGrad.SlfStGradMax =
        (uint8_T)rtb_SlfStGradMax_out;
    (*ved__sye_Y_VED_NVData_out).Write.SlfstGrad.SlfStGradMin =
        (uint8_T)rtb_SlfStGradMin_out;

    /* Switch: '<S70>/Reset' incorporates:
     *  Constant: '<S70>/Initial Condition'
     *  Update for UnitDelay: '<S70>/FixPt Unit Delay1'
     */
    if (rtb_R_onoff_n) {
        ved__sye_DWork->FixPtUnitDelay1_DSTATE_g = 0.0F;
    } else {
        ved__sye_DWork->FixPtUnitDelay1_DSTATE_g = rtb_add_trav_dist;
    }

    /* Update for UnitDelay: '<S10>/x_delay_eg' */
    for (i = 0; i < 3; i++) {
        ved__sye_DWork->x_delay_eg_DSTATE[(i)] = rtb_xpost[i];
    }

    /* Update for UnitDelay: '<S10>/init_nvm_eg_delay' */
    ved__sye_DWork->init_nvm_eg_delay_DSTATE = rtb_init_out;

    /* Update for UnitDelay: '<S37>/T3' */
    ved__sye_DWork->T3_DSTATE = rtb_T2;

    /* Update for UnitDelay: '<S37>/T2' */
    ved__sye_DWork->T2_DSTATE = rtb_T1;

    /* Update for UnitDelay: '<S37>/T1' */
    ved__sye_DWork->T1_DSTATE = rtb_T0;

    /* Update for UnitDelay: '<S37>/T0' */
    ved__sye_DWork->T0_DSTATE = rtb_T7;

    /* Update for UnitDelay: '<S37>/T7' */
    ved__sye_DWork->T7_DSTATE = rtb_T6;

    /* Update for UnitDelay: '<S37>/T6' */
    ved__sye_DWork->T6_DSTATE = rtb_T5;

    /* Update for UnitDelay: '<S37>/T5' */
    ved__sye_DWork->T5_DSTATE = rtb_T4;

    /* Update for UnitDelay: '<S37>/T4' */
    ved__sye_DWork->T4_DSTATE = rtb_sigma_EG;

    /* Update for UnitDelay: '<S10>/P_delay_eg' */
    for (i = 0; i < 9; i++) {
        ved__sye_DWork->P_delay_eg_DSTATE[(i)] = rtb_A[i];
    }

    /* Update for UnitDelay: '<S75>/FixPt Unit Delay1' */
    ved__sye_DWork->FixPtUnitDelay1_DSTATE = rtb_Reset;

    /* Update for UnitDelay: '<S76>/FixPt Unit Delay1' */
    ved__sye_DWork->FixPtUnitDelay1_DSTATE_b = rtb_Reset_m;

    /* Update for UnitDelay: '<S9>/trav_dist_reset' */
    ved__sye_DWork->trav_dist_reset_DSTATE = rtb_trav_dist_reset_f;

    /* Update for UnitDelay: '<S70>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S70>/FixPt Constant'
     */
    ved__sye_DWork->FixPtUnitDelay2_DSTATE = 0U;

    /* Update for UnitDelay: '<S9>/last_SlfStGradDisc' */
    ved__sye_DWork->last_SlfStGradDisc_DSTATE = rtb_SlfStGradDisc_out;

    /* Update for UnitDelay: '<S12>/x_delay_yaw' */
    for (i = 0; i < 2; i++) {
        ved__sye_DWork->x_delay_yaw_DSTATE[(i)] = rtb_x_pred_i[i];
    }

    /* Update for UnitDelay: '<S58>/T3' */
    ved__sye_DWork->T3_DSTATE_c = rtb_T2_e;

    /* Update for UnitDelay: '<S58>/T2' */
    ved__sye_DWork->T2_DSTATE_f = rtb_T1_p;

    /* Update for UnitDelay: '<S58>/T1' */
    ved__sye_DWork->T1_DSTATE_k = rtb_T0_a;

    /* Update for UnitDelay: '<S58>/T0' */
    ved__sye_DWork->T0_DSTATE_f = rtb_x_pred_i[1];

    /* Update for UnitDelay: '<S12>/P_delay_yaw' */
    for (i = 0; i < 4; i++) {
        ved__sye_DWork->P_delay_yaw_DSTATE[(i)] = rtb_APAt[i];
    }

    /* Update for UnitDelay: '<S43>/last_not_zero_min' */
    ved__sye_DWork->last_not_zero_min_DSTATE = rtb_Reset_l;

    /* Update for UnitDelay: '<S43>/last_steering_angle' */
    ved__sye_DWork->last_steering_angle_DSTATE = rtb_z_out_c;

    /* Update for UnitDelay: '<S50>/FixPt Unit Delay1' */
    ved__sye_DWork->FixPtUnitDelay1_DSTATE_l = rtb_Reset_l;

    /* Update for UnitDelay: '<S23>/T3' */
    ved__sye_DWork->T3_DSTATE_f = rtb_T2_m;

    /* Update for UnitDelay: '<S23>/T2' */
    ved__sye_DWork->T2_DSTATE_e = rtb_T1_f;

    /* Update for UnitDelay: '<S23>/T1' */
    ved__sye_DWork->T1_DSTATE_h = rtb_T0_k;

    /* Update for UnitDelay: '<S23>/T0' */
    ved__sye_DWork->T0_DSTATE_l = ved__sye_DWork->x_delay_curve_DSTATE[0];
}

/* Model initialize function */
void ved__sye_initialize(boolean_T firstTime,
                         RT_MODEL_ved__sye *const ved__sye_M,
                         BlockIO_ved__sye *ved__sye_B,
                         D_Work_ved__sye *ved__sye_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__sye_M, (NULL));

    /* block I/O */
    (void)memset(((void *)ved__sye_B), 0, sizeof(BlockIO_ved__sye));

    /* states (dwork) */
    (void)memset((void *)ved__sye_DWork, 0, sizeof(D_Work_ved__sye));

    /* Start for ifaction SubSystem: '<S78>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S89>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &ved__sye_DWork->PHt_HPHt_R_DWORK4[0];
        for (i = 0; i < 4; i++) {
            *dWork++ = 0.0;
        }

        dWork = &ved__sye_DWork->PHt_HPHt_R_DWORK4[0];
        while (dWork < &ved__sye_DWork->PHt_HPHt_R_DWORK4[0] + 4) {
            *dWork = 1;
            dWork += 3;
        }
    }

    /* end of Start for SubSystem: '<S78>/calculate the gain' */
    {
        int32_T i;

        /* InitializeConditions for UnitDelay: '<S10>/x_delay_eg' */
        for (i = 0; i < 3; i++) {
            ved__sye_DWork->x_delay_eg_DSTATE[(i)] = ved__sye_x_SSG_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S10>/P_delay_eg' */
        for (i = 0; i < 9; i++) {
            ved__sye_DWork->P_delay_eg_DSTATE[(i)] = ved__sye_P_SSG_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S76>/FixPt Unit Delay1' */
        ved__sye_DWork->FixPtUnitDelay1_DSTATE_b = 0.008F;

        /* InitializeConditions for UnitDelay: '<S70>/FixPt Unit Delay2' */
        ved__sye_DWork->FixPtUnitDelay2_DSTATE = 1U;

        /* InitializeConditions for UnitDelay: '<S12>/x_delay_yaw' */
        for (i = 0; i < 2; i++) {
            ved__sye_DWork->x_delay_yaw_DSTATE[(i)] = ved__sye_x_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S12>/P_delay_yaw' */
        for (i = 0; i < 4; i++) {
            ved__sye_DWork->P_delay_yaw_DSTATE[(i)] = ved__sye_P_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S43>/last_not_zero_min' */
        ved__sye_DWork->last_not_zero_min_DSTATE = 10.0F;

        /* InitializeConditions for UnitDelay: '<S50>/FixPt Unit Delay1' */
        ved__sye_DWork->FixPtUnitDelay1_DSTATE_l = 10.0F;

        /* InitializeConditions for UnitDelay: '<S11>/x_delay_curve' */
        for (i = 0; i < 2; i++) {
            ved__sye_DWork->x_delay_curve_DSTATE[(i)] =
                ved__sye_x_di_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S11>/P_delay_curve' */
        for (i = 0; i < 4; i++) {
            ved__sye_DWork->P_delay_curve_DSTATE[(i)] =
                ved__sye_P_di_init_p[(i)];
        }
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */