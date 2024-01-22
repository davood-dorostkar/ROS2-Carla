
#include "ved_consts.h"
#include "ved_wye.h"
#include "ved_wye_private.h"
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
 *    '<S23>/calculate determinant'
 *    '<S34>/calculate determinant'
 * Common block description:
 *   calculate determinant
 */
void ved__wye_calculatedeterminant(const real32_T rtu_u[4],
                                   rtB_calculatedeterminant_ved__wy *localB) {
    /* Embedded MATLAB: '<S23>/calculate determinant' */
    /* Embedded MATLAB Function 'KF_whl_yaw/Calculate Kalman gain
     * PHt_(HPHt_R)/calculate determinant': '<S26>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /* '<S26>:1:3' */
    localB->y = (rtu_u[0] * rtu_u[3]) - (rtu_u[1] * rtu_u[2]);
}

/*
 * Output and update for atomic system:
 *    '<S45>/Get_NVM_IO_State'
 *    '<S58>/Get_NVM_IO_State'
 */
void ved__wye_Get_NVM_IO_State(uint32_T rtu_pos,
                               uint32_T rtu_state_in,
                               uint32_T rtu_VED_IOBitMask,
                               rtB_Get_NVM_IO_State_ved__wye *localB) {
    int32_T x;
    uint32_T tmp;

    /* Embedded MATLAB: '<S45>/Get_NVM_IO_State' */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/EKF/Get_WhlLoad_IO_State/GetIOState/Get_NVM_IO_State':
     * '<S46>:1' */
    /*  position in state array */
    /* '<S46>:1:4' */
    tmp = rtu_pos;
    if (rtu_pos > 2147483647U) {
        tmp = 2147483647U;
    }

    x = (int32_T)tmp;

    /* '<S46>:1:5' */
    /*  get state info */
    /* '<S46>:1:8' */
    localB->state = (rtu_state_in >> (x - ((x >> 5) << 5))) & rtu_VED_IOBitMask;
}

/*
 * Output and update for atomic system:
 *    '<S31>/Time2Sec'
 *    '<S33>/Time2Sec'
 *    '<S9>/Time2Sec'
 *    '<S10>/Time2Sec'
 */
void ved__wye_Time2Sec(uint16_T rtu_u, rtB_Time2Sec_ved__wye *localB) {
    /* Product: '<S53>/Time2Sec' incorporates:
     *  Constant: '<S53>/Constant'
     */
    localB->Time2Sec = ((real32_T)rtu_u) / 1000.0F;
}

/*
 * Output and update for atomic system:
 *    '<S60>/make_A_matrix'
 *    '<S74>/make_A_matrix'
 */
void ved__wye_make_A_matrix(real32_T rtu_CycleTime,
                            rtB_make_A_matrix_ved__wye *localB) {
    int32_T i;
    static int8_T tmp[3] = {0, 1, 0};

    static int8_T tmp_0[3] = {0, 0, 1};

    /* Embedded MATLAB: '<S60>/make_A_matrix' */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/make_A_Q_R1/A/make_A_matrix': '<S67>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S67>:1:5' */
    localB->A[0] = 1.0F;
    localB->A[3] = rtu_CycleTime;
    localB->A[6] = 0.0F;
    for (i = 0; i < 3; i++) {
        localB->A[1 + (3 * i)] = (real32_T)tmp[i];
    }

    for (i = 0; i < 3; i++) {
        localB->A[2 + (3 * i)] = (real32_T)tmp_0[i];
    }
}

/*
 * Output and update for atomic system:
 *    '<S64>/get_bit'
 *    '<S78>/get_bit'
 */
void ved__wye_get_bit(uint16_T rtu_value_in,
                      uint16_T rtu_bitmask,
                      rtB_get_bit_ved__wye *localB) {
    /* Embedded MATLAB: '<S64>/get_bit' */
    /* Embedded MATLAB Function 'get_bit/get_bit': '<S73>:1' */
    /*  get state of a specified bit */
    if (rtu_value_in == rtu_bitmask) {
        /* '<S73>:1:3' */
        /* '<S73>:1:4' */
        localB->y = 1U;
    } else {
        /* '<S73>:1:6' */
        localB->y = 0U;
    }
}

/*
 * Output and update for atomic system:
 *    '<S11>/Get_IO_State'
 *    '<S11>/Get_IO_State1'
 */
void ved__wye_Get_IO_State(const uint8_T rtu_state_in[32],
                           rtB_Get_IO_State_ved__wye *localB,
                           uint32_T rtp_Filter) {
    /* MultiPortSwitch: '<S97>/Index Vector' incorporates:
     *  Constant: '<S97>/Constant1'
     */
    localB->IndexVector = rtu_state_in[(rtp_Filter)];
}

/* Model step function */
void ved__wye_step(BlockIO_ved__wye *ved__wye_B,
                   D_Work_ved__wye *ved__wye_DWork,
                   VED_InputData_t *ved__wye_U_VED_InputData,
                   VED_InternalData_t *ved__wye_U_VED_InternalData_in,
                   VED_NvData_t *ved__wye_U_VED_NvData_in,
                   VED_InternalData_t *ved__wye_Y_VED_InternalData_out,
                   VED_NvData_t *ved__wye_Y_VED_NVData_out) {
    /* local block i/o variables */
    real32_T rtb_HPHt_R[4];
    real32_T rtb_P_predHt[6];
    real32_T rtb_K[6];
    real32_T rtb_HPHt_R_o[9];
    real32_T rtb_P_predHt_m[9];
    real32_T rtb_K_d[9];
    real32_T rtb_HPHt_R_m[4];
    real32_T rtb_P_predHt_h[4];
    real32_T rtb_K_n[4];
    uint32_T rtb_State;
    uint32_T rtb_State_n;
    uint16_T rtb_CycleTime;
    uint16_T rtb_BitwiseOperator;
    uint16_T rtb_CycleTime_j;
    uint16_T rtb_BitwiseOperator_c;
    uint16_T rtb_CycleTime_g;
    uint16_T rtb_CycleTime_c;
    uint8_T rtb_State_f[32];

    /* local scratch DWork variables */
    real32_T PHt_HPHt_R_DWORK1[4];
    real32_T PHt_HPHt_R_DWORK3[4];
    real32_T PHt_HPHt_R_DWORK5[4];
    real32_T PHt_HPHt_R_DWORK1_l[4];
    real32_T PHt_HPHt_R_DWORK3_c[4];
    real32_T PHt_HPHt_R_DWORK5_h[4];
    real32_T PHt_HPHt_R_DWORK1_f[9];
    real32_T PHt_HPHt_R_DWORK3_l[9];
    real32_T PHt_HPHt_R_DWORK5_m[9];
    int32_T PHt_HPHt_R_DWORK2[2];
    int32_T PHt_HPHt_R_DWORK2_e[2];
    int32_T PHt_HPHt_R_DWORK2_o[3];
    real32_T frfilt;
    real32_T refilt;
    real32_T kf_quad;
    real32_T vx_quad;
    real32_T WldFactFront;
    real32_T sigma_model_gain[2];
    int32_T bitpos;
    uint32_T c;
    real32_T rtb_Yk1;
    real32_T rtb_frfiltWithWldEst;
    real32_T rtb_refiltWithWldEst;
    real32_T rtb_x_post[3];
    real32_T rtb_x_post_h[3];
    real32_T rtb_x_pred[3];
    real32_T rtb_R_rear;
    real32_T rtb_hx[2];
    real32_T rtb_Init;
    real32_T rtb_x_pred_m[3];
    real32_T rtb_Abs3;
    real32_T rtb_x_out[3];
    uint8_T rtb_wye_yaw_off_control;
    uint8_T rtb_Reset;
    int32_T rtb_TP_yaw_off_control_out;
    real32_T rtb_frWithWldEst_wye;
    real32_T rtb_reWithWldEst_wye;
    real32_T rtb_Sum;
    real32_T rtb_Sum_p;
    real32_T rtb_Sum_c;
    real32_T rtb_TmpSignalConversionAtSFun_c[2];
    real32_T rtb_x_post_j[2];
    int32_T rtb_TP_init;
    uint8_T rtb_init_out;
    uint8_T rtb_counter_out;
    int8_T rtb_sign_out;
    boolean_T rtb_R_onoff_front;
    boolean_T rtb_R_onoff_rear;
    boolean_T rtb_R_yaw_valid;
    uint8_T rtb_init;
    uint8_T rtb_Rroff;
    boolean_T rtb_R_onoff;
    real32_T rtb_x_out_i;
    uint32_T rtb_state;
    real32_T rtb_Q_g[9];
    real32_T rtb_H[6];
    real32_T rtb_Ht[6];
    real32_T rtb_P_pred[9];
    real32_T rtb_R_out_p[4];
    real32_T rtb_Q_m[9];
    real32_T rtb_P_pred_n[9];
    real32_T rtb_R_out_e[9];
    real32_T rtb_A[4];
    real32_T rtb_P_pred_nb[4];
    int32_T i;
    real32_T sigma_model_gain_0[2];
    real32_T rtb_TmpSignalConversionAtSFun_0[4];
    real32_T tmp[9];
    real32_T rtb_H_0[6];
    real32_T tmp_0[3];
    real32_T rtb_frfiltWithWldEst_0[3];
    real32_T rtb_A_0[4];
    real32_T rtb_A_1[4];
    real32_T rtb_frWithWldEst_wye_0[2];
    int32_T i_0;
    real32_T rtb_VectorConcatenate_idx;
    real32_T rtb_VectorConcatenate_idx_0;
    real32_T rtb_VectorConcatenate_idx_1;
    static int8_T tmp_1[3] = {0, 1, 0};

    /* UnitDelay: '<S49>/UD' */
    rtb_Yk1 = ved__wye_DWork->UD_DSTATE;

    /* Embedded MATLAB: '<S11>/divide_front_whl' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'make_z_vektor/divide_front_whl': '<S100>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((*ved__wye_U_VED_InternalData_in)
             .ved__wpp_out.wheel_velo_front_right >= 3.0F) &&
        ((*ved__wye_U_VED_InternalData_in).ved__wpp_out.wheel_velo_front_left >=
         3.0F)) {
        /* '<S100>:1:5' */
        /* '<S100>:1:6' */
        rtb_VectorConcatenate_idx = (*ved__wye_U_VED_InternalData_in)
                                        .ved__wpp_out.wheel_velo_front_left /
                                    (*ved__wye_U_VED_InternalData_in)
                                        .ved__wpp_out.wheel_velo_front_right;

        /* '<S100>:1:7' */
        rtb_R_onoff_front = TRUE;
    } else {
        /* '<S100>:1:9' */
        rtb_VectorConcatenate_idx = 1.0F;

        /* '<S100>:1:10' */
        rtb_R_onoff_front = FALSE;
    }

    /* Embedded MATLAB: '<S11>/divide_rear_whl' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'make_z_vektor/divide_rear_whl': '<S101>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((*ved__wye_U_VED_InternalData_in).ved__wpp_out.wheel_velo_rear_right >=
         3.0F) &&
        ((*ved__wye_U_VED_InternalData_in).ved__wpp_out.wheel_velo_rear_left >=
         3.0F)) {
        /* '<S101>:1:5' */
        /* '<S101>:1:6' */
        rtb_VectorConcatenate_idx_0 = (*ved__wye_U_VED_InternalData_in)
                                          .ved__wpp_out.wheel_velo_rear_left /
                                      (*ved__wye_U_VED_InternalData_in)
                                          .ved__wpp_out.wheel_velo_rear_right;

        /* '<S101>:1:7' */
        rtb_R_onoff_rear = TRUE;
    } else {
        /* '<S101>:1:9' */
        rtb_VectorConcatenate_idx_0 = 1.0F;

        /* '<S101>:1:10' */
        rtb_R_onoff_rear = FALSE;
    }

    /* BusSelector: '<S11>/Bus Selector1' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_f[i] = (*ved__wye_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S11>/Get_IO_State' */
    ved__wye_Get_IO_State(rtb_State_f, &ved__wye_B->Get_IO_State,
                          ((uint32_T)VED_SIN_POS_YWR));

    /* end of Outputs for SubSystem: '<S11>/Get_IO_State' */

    /* Outputs for atomic SubSystem: '<S11>/Get_IO_State1' */
    ved__wye_Get_IO_State(rtb_State_f, &ved__wye_B->Get_IO_State1,
                          ((uint32_T)VED_SIN_POS_YWRINT));

    /* end of Outputs for SubSystem: '<S11>/Get_IO_State1' */

    /* Embedded MATLAB: '<S11>/R_yaw_valid' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    /* Embedded MATLAB Function 'make_z_vektor/R_yaw_valid': '<S99>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((uint32_T)CFG_VED__INT_GYRO) == 0U) {
        /* '<S99>:1:5' */
        if (ved__wye_B->Get_IO_State.IndexVector == 0) {
            /* '<S99>:1:6' */
            /* '<S99>:1:7' */
            rtb_R_yaw_valid = TRUE;

            /* '<S99>:1:8' */
            rtb_VectorConcatenate_idx_1 =
                (*ved__wye_U_VED_InputData).Signals.YawRate;
        } else {
            /* '<S99>:1:10' */
            rtb_R_yaw_valid = FALSE;

            /* '<S99>:1:11' */
            rtb_VectorConcatenate_idx_1 = 0.0F;
        }
    } else if (ved__wye_B->Get_IO_State1.IndexVector == 0) {
        /* '<S99>:1:14' */
        /* '<S99>:1:15' */
        rtb_R_yaw_valid = TRUE;

        /* '<S99>:1:16' */
        rtb_VectorConcatenate_idx_1 =
            (*ved__wye_U_VED_InputData).Signals.YawRateInt;
    } else {
        /* '<S99>:1:18' */
        rtb_R_yaw_valid = FALSE;

        /* '<S99>:1:19' */
        rtb_VectorConcatenate_idx_1 = 0.0F;
    }

    /* Sum: '<S49>/Sum' incorporates:
     *  Gain: '<S49>/Gain'
     *  Sum: '<S49>/Diff'
     */
    rtb_Sum = ((rtb_Yk1 - rtb_VectorConcatenate_idx) * 0.98F) +
              rtb_VectorConcatenate_idx;

    /* Sum: '<S50>/Sum' incorporates:
     *  Gain: '<S50>/Gain'
     *  Sum: '<S50>/Diff'
     *  UnitDelay: '<S50>/UD'
     */
    rtb_Sum_p =
        ((ved__wye_DWork->UD_DSTATE_a - rtb_VectorConcatenate_idx_0) * 0.98F) +
        rtb_VectorConcatenate_idx_0;

    /* Embedded MATLAB: '<S30>/calc_fr_re_yaw' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     *  UnitDelay: '<S8>/last_est_wld'
     */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/calc_yaw_front_rear/calc_fr_re_yaw':
     * '<S48>:1' */
    /*  use this equations if no wheel load dependancty should be used */
    /*  Qf = */
    /* ->   1/(yaw*s+vx)*(-(yaw*s+vx)*(yaw*s-vx))^(1/2)*Kf */
    /*     -1/(yaw*s+vx)*(-(yaw*s+vx)*(yaw*s-vx))^(1/2)*Kf */
    /*    */
    /*  Qr = */
    /*  -Kr*(yaw*s-2*vx)/(yaw*s+2*vx) */
    /*    */
    /*  H = */
    /*  ->[ 0, -vx*Kf*s/(yaw*s+vx)/(-yaw^2*s^2+vx^2)^(1/2),
     * 1/(yaw*s+vx)*(-yaw^2*s^2+vx^2)^(1/2),  0,  0] */
    /*  [ 0,  vx*Kf*s/(yaw*s+vx)/(-yaw^2*s^2+vx^2)^(1/2),
     * -1/(yaw*s+vx)*(-yaw^2*s^2+vx^2)^(1/2),  0,  0] */
    /*  [ 0,                   -4*s*Kr*vx/(yaw*s+2*vx)^2,  0,
     * -(yaw*s-2*vx)/(yaw*s+2*vx),  0] */
    /*  [ 0,                                           1,  0, 0, -1] */
    /*  use this equations if the estimated wheel load dependancty for the front
     * axis should be used */
    /*  WldFactFront =
     * 1+1/1000000*est_wld_dep*VED_Kf_CntrOfGravHeight_met*VED_Kf_VehWeight_kg*AxisLoadDist*vx^2/TW_F^2
     */
    /*  WldFactRear = single(1.0) */
    /*  use this equations if the parameter wheel load dependancty for both axis
     * should be used */
    /*  WldFactFront =
     * 1+1/1000000*VED_Kf_WhlLoadDepFrontAxle_nu*VED_Kf_CntrOfGravHeight_met*VED_Kf_VehWeight_kg*AxisLoadDist*vx^2/TW_F^2
     */
    /*  WldFactRear =
     * 1+1/1000000*VED_Kf_WhlLoadDepRearAxle_nu*VED_Kf_CntrOfGravHeight_met*VED_Kf_VehWeight_kg*(1-AxisLoadDist)*vx^2/TW_R^2
     */
    /*  front = -(Qf^2-k_front^2)/(Qf^2+k_front^2)*vx/TW_F/WldFactFront */
    /*  rear = -2*(Qr-k_rear)/(Qr+k_rear)*vx/TW_R/WldFactRear */
    /*    */
    /*  Qf = ->
     * 1/(yaw*TW_F*WldFactFront+vx)*(-(yaw*TW_F*WldFactFront+vx)*(yaw*TW_F*WldFactFront-vx))^(1/2)*k_front
     */
    /*       -1/(yaw*TW_F*WldFactFront+vx)*(-(yaw*TW_F*WldFactFront+vx)*(yaw*TW_F*WldFactFront-vx))^(1/2)*k_front
     */
    /*   */
    /*  Qr = -k_rear*(yaw*s*WldFactRear-2*vx)/(yaw*s*WldFactRear+2*vx) */
    /*    */
    /*  h =
     * 1/(yaw*TW_F*WldFactFront+vx)*(-(yaw*TW_F*WldFactFront+vx)*(yaw*TW_F*WldFactFront-vx))^(1/2)*k_front
     */
    /*                                                -k_rear*(yaw*s*WldFactRear-2*vx)/(yaw*s*WldFactRear+2*vx)
     */
    /*                                                                                               yaw+yawoff
     */
    /*    */
    /*  H1 =[ 0,
     * -vx*k_front*TW_F*WldFactFront/(yaw*TW_F*WldFactFront+vx)/(-yaw^2*TW_F^2*WldFactFront^2+vx^2)^(1/2),
     * 0] */
    /*      [ 0, -4*s*WldFactRear*k_rear*vx/(yaw*s*WldFactRear+2*vx)^2, 0] */
    /*      [ 0, 1, 1] */
    /*  fr1 = -((qf^2 - 1) / (qf^2 + 1))*(1/twf)*vx; */
    /*   */
    /*  re = -((qr - 1) / (qr + 1))*(2/twr)*vx; */
    /*  fr1 = -((qf^2 - 1) / (qf^2 + 1))*(1/twf)*vx; */
    /*   */
    /*  re = -((qr - 0.9995) / (qr + 0.9995))*(2/twr)*vx; */
    /*   */
    /* fWld = 2.675; */
    /* rWld = 1.0;%1.725; */
    /* fr = -((qf^2 - kf^2) / (qf^2 + kf^2))*(1/twf)*vx; */
    /* re = -((qr - kr) / (qr + kr))*(2/twr)*vx; */
    /* '<S48>:1:61' */
    rtb_Yk1 = rtb_Sum * rtb_Sum;

    /* '<S48>:1:62' */
    rtb_VectorConcatenate_idx *= rtb_VectorConcatenate_idx;

    /* '<S48>:1:64' */
    kf_quad = (*ved__wye_U_VED_InternalData_in)
                  .ved__offsets_in.ved__whs_offset.offset_ratio_front *
              (*ved__wye_U_VED_InternalData_in)
                  .ved__offsets_in.ved__whs_offset.offset_ratio_front;

    /* '<S48>:1:65' */
    vx_quad = (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo *
              (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo;

    /*  calculate the wheel load dep factor for the different configurations */
    /* '<S48>:1:68' */
    switch (((uint32_T)VED__USE_EST_WLD_DEP)) {
        case 0U:
            /*  no wheel load dep is used */
            /* '<S48>:1:70' */
            WldFactFront = 1.0F;

            /* '<S48>:1:72' */
            vx_quad = 1.0F;
            break;

        case 1U:
            /*  use the estimated wheel load dep for front axis */
            /*  calc wld factor front      */
            /* '<S48>:1:75' */
            WldFactFront =
                (((((ved__wye_DWork->last_est_wld_DSTATE *
                     (*ved__wye_U_VED_InputData)
                         .Parameter.VED_Kf_CntrOfGravHeight_met) *
                    (*ved__wye_U_VED_InputData).Parameter.VED_Kf_VehWeight_kg) *
                   (*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_AxisLoadDistr_per) *
                  vx_quad) /
                 (((*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_TrackWidthFront_met *
                   (*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_TrackWidthFront_met) *
                  1.0E+6F)) +
                1.0F;

            /* '<S48>:1:77' */
            vx_quad = 1.0F;
            break;

        case 2U:
            /*  use the wheel load parameter only for front axis */
            /*  calc wld factor front */
            /* '<S48>:1:80' */
            WldFactFront =
                ((((((*ved__wye_U_VED_InputData)
                         .Parameter.VED_Kf_WhlLoadDepFrontAxle_nu *
                     (*ved__wye_U_VED_InputData)
                         .Parameter.VED_Kf_CntrOfGravHeight_met) *
                    (*ved__wye_U_VED_InputData).Parameter.VED_Kf_VehWeight_kg) *
                   (*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_AxisLoadDistr_per) *
                  vx_quad) /
                 (((*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_TrackWidthFront_met *
                   (*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_TrackWidthFront_met) *
                  1.0E+6F)) +
                1.0F;

            /* '<S48>:1:82' */
            vx_quad = 1.0F;
            break;

        case 3U:
            /*  Use the wheel load parameter for front and rear axis */
            /*  calc wld factor front         */
            /* '<S48>:1:86' */
            WldFactFront =
                ((((((*ved__wye_U_VED_InputData)
                         .Parameter.VED_Kf_WhlLoadDepFrontAxle_nu *
                     (*ved__wye_U_VED_InputData)
                         .Parameter.VED_Kf_CntrOfGravHeight_met) *
                    (*ved__wye_U_VED_InputData).Parameter.VED_Kf_VehWeight_kg) *
                   (*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_AxisLoadDistr_per) *
                  vx_quad) /
                 (((*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_TrackWidthFront_met *
                   (*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_TrackWidthFront_met) *
                  1.0E+6F)) +
                1.0F;

            /*  calc wld factor rear         */
            /* '<S48>:1:89' */
            vx_quad =
                ((((((*ved__wye_U_VED_InputData)
                         .Parameter.VED_Kf_WhlLoadDepRearAxle_nu *
                     (*ved__wye_U_VED_InputData)
                         .Parameter.VED_Kf_CntrOfGravHeight_met) *
                    (*ved__wye_U_VED_InputData).Parameter.VED_Kf_VehWeight_kg) *
                   (1.0F - (*ved__wye_U_VED_InputData)
                               .Parameter.VED_Kf_AxisLoadDistr_per)) *
                  vx_quad) /
                 (((*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_TrackWidthRear_met *
                   (*ved__wye_U_VED_InputData)
                       .Parameter.VED_Kf_TrackWidthRear_met) *
                  1.0E+6F)) +
                1.0F;
            break;

        default:
            /*  no wheel load dep is used */
            /* '<S48>:1:91' */
            WldFactFront = 1.0F;

            /* '<S48>:1:92' */
            vx_quad = 1.0F;
            break;
    }

    /*  calculate the filtered wheel yaw rate for the front and rear axis */
    /* '<S48>:1:97' */
    frfilt =
        ((-((rtb_Yk1 - kf_quad) / (rtb_Yk1 + kf_quad))) *
         (1.0F /
          (*ved__wye_U_VED_InputData).Parameter.VED_Kf_TrackWidthFront_met)) *
        (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo;

    /*  calculate the filtered wheel yaw rate for the front and rear axis for
     * wheel yaw rate estimation */
    /* '<S48>:1:100' */
    /*  front filtered yaw rate with wheel load dep */
    if (((real32_T)fabs(WldFactFront)) >= 1.0E-7F) {
        /* '<S48>:1:103' */
        /* '<S48>:1:104' */
        rtb_frfiltWithWldEst = frfilt / WldFactFront;

        /* '<S48>:1:105' */
        rtb_frWithWldEst_wye =
            (((-((rtb_VectorConcatenate_idx - kf_quad) /
                 (rtb_VectorConcatenate_idx + kf_quad))) *
              (1.0F / (*ved__wye_U_VED_InputData)
                          .Parameter.VED_Kf_TrackWidthFront_met)) *
             (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo) /
            WldFactFront;
    } else {
        /* '<S48>:1:107' */
        rtb_frfiltWithWldEst = 1.4F;

        /*  max ca. 80 grad / s  */
        /* '<S48>:1:108' */
        rtb_frWithWldEst_wye = 1.4F;

        /*  max ca. 80 grad / s  */
    }

    /*  rear filtered yaw rate */
    /* '<S48>:1:112' */
    refilt =
        ((-((rtb_Sum_p -
             (*ved__wye_U_VED_InternalData_in)
                 .ved__offsets_in.ved__whs_offset.offset_ratio_rear) /
            (rtb_Sum_p +
             (*ved__wye_U_VED_InternalData_in)
                 .ved__offsets_in.ved__whs_offset.offset_ratio_rear))) *
         (2.0F /
          (*ved__wye_U_VED_InputData).Parameter.VED_Kf_TrackWidthRear_met)) *
        (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo;

    /*  rear filtered yaw rate for wheel yaw rate estimation */
    /* '<S48>:1:115' */
    /*  rear filtered yaw rate with wheel load dep */
    if (((real32_T)fabs(vx_quad)) >= 1.0E-7F) {
        /* '<S48>:1:118' */
        /* '<S48>:1:119' */
        rtb_refiltWithWldEst = refilt / vx_quad;

        /* '<S48>:1:120' */
        rtb_reWithWldEst_wye =
            (((-((rtb_VectorConcatenate_idx_0 -
                  (*ved__wye_U_VED_InternalData_in)
                      .ved__offsets_in.ved__whs_offset.offset_ratio_rear) /
                 (rtb_VectorConcatenate_idx_0 +
                  (*ved__wye_U_VED_InternalData_in)
                      .ved__offsets_in.ved__whs_offset.offset_ratio_rear))) *
              (2.0F / (*ved__wye_U_VED_InputData)
                          .Parameter.VED_Kf_TrackWidthRear_met)) *
             (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo) /
            vx_quad;
    } else {
        /* '<S48>:1:122' */
        rtb_refiltWithWldEst = 1.4F;

        /*  max ca. 80 grad / s  */
        /* '<S48>:1:123' */
        rtb_reWithWldEst_wye = 1.4F;

        /*  max ca. 80 grad / s  */
    }

    /*  mean front rear yaw rate  */
    /* '<S48>:1:127' */
    /* frWld = -((qf^2 - kf^2) / (qf^2 + kf^2))*(1/twf)*vx/(1 + fWld * COG *
     * (Weight*AxlDist/1000.0) * vx.^2 / (1000*twf^2)) ; */
    /* reWld = -((qr - kr) / (qr + kr))*(2/twr)*vx/(1 + rWld * COG *
     * (Weight*(1-AxlDist)/1000.0) * vx.^2 / (1000*twr^2)); */
    /* frWldcal = -((qf^2 - kf^2) / (qf^2 + kf^2))*(1/twf)*vx/(1 + wld * COG *
     * (Weight*AxlDist/1000.0) * vx.^2 / (1000*twf^2)) ; */

    /* Abs: '<S32>/Abs3' incorporates:
     *  Sum: '<S32>/Sub'
     */
    rtb_Abs3 = (real32_T)fabs(rtb_frfiltWithWldEst - rtb_refiltWithWldEst);

    /* BusSelector: '<S33>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CycleTime = (*ved__wye_U_VED_InputData).Frame.CycleTime;

    /* Outputs for atomic SubSystem: '<S33>/Time2Sec' */
    ved__wye_Time2Sec(rtb_CycleTime, &ved__wye_B->Time2Sec);

    /* end of Outputs for SubSystem: '<S33>/Time2Sec' */

    /* Embedded MATLAB: '<S60>/make_A_matrix' */
    ved__wye_make_A_matrix(ved__wye_B->Time2Sec.Time2Sec,
                           &ved__wye_B->sf_make_A_matrix);

    /* UnitDelay: '<S29>/x_delay_wld' */
    for (i = 0; i < 3; i++) {
        rtb_x_post[i] = ved__wye_DWork->x_delay_wld_DSTATE[(i)];
    }

    /* BusSelector: '<S29>/Bus Selector2' incorporates:
     *  Inport: '<Root>/VED_NvData_in'
     */
    rtb_State = (*ved__wye_U_VED_NvData_in).Read.State;

    /* Embedded MATLAB: '<S45>/Get_NVM_IO_State' */
    ved__wye_Get_NVM_IO_State(((uint32_T)VED_NVM_POS_WLD), rtb_State, 3U,
                              &ved__wye_B->sf_Get_NVM_IO_State);

    /* Embedded MATLAB: '<S29>/check_init_state' incorporates:
     *  Inport: '<Root>/VED_NvData_in'
     *  UnitDelay: '<S29>/init_nvm_wld_delay'
     */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/EKF/check_init_state': '<S38>:1' */
    /*  check if NVMSSG is valid and first init is true */
    if ((ved__wye_B->sf_Get_NVM_IO_State.state ==
         ((uint32_T)VED_IO_STATE_VALID)) &&
        (ved__wye_DWork->init_nvm_wld_delay_DSTATE == 0)) {
        /* '<S38>:1:3' */
        /* '<S38>:1:4' */
        rtb_Yk1 = (*ved__wye_U_VED_NvData_in).Read.Wld.Wld_front;

        /* '<S38>:1:5' */
        rtb_init = 1U;

        /* '<S38>:1:6' */
        rtb_init_out = 1U;
    } else {
        /* '<S38>:1:8' */
        rtb_Yk1 = 1.5F;

        /* '<S38>:1:9' */
        rtb_init = 0U;

        /* '<S38>:1:10' */
        rtb_init_out = ved__wye_DWork->init_nvm_wld_delay_DSTATE;
    }

    /* Embedded MATLAB: '<S29>/init_x_yaw' */
    /* Embedded MATLAB Function 'estimate_wheel_load_dep_front/EKF/init_x_yaw':
     * '<S40>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S40>:1:5' */
    for (i = 0; i < 3; i++) {
        rtb_x_post_h[i] = rtb_x_post[i];
    }

    /*  if  */
    if (rtb_x_post[2] < 0.5F) {
        /* '<S40>:1:8' */
        /* '<S40>:1:9' */
        rtb_x_post_h[2] = 0.5F;
    }

    /*  if  */
    if (rtb_x_post[2] > 3.0F) {
        /* '<S40>:1:13' */
        /* '<S40>:1:14' */
        rtb_x_post_h[2] = 3.0F;
    }

    /*  init wheel load dependency if init is true */
    if (rtb_init == 1) {
        /* '<S40>:1:18' */
        /* '<S40>:1:19' */
        rtb_x_post_h[2] = rtb_Yk1;
    }

    /* Product: '<S29>/Ax' */
    for (i = 0; i < 3; i++) {
        rtb_x_pred[i] = 0.0F;
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_x_pred[i] = (ved__wye_B->sf_make_A_matrix.A[(3 * bitpos) + i] *
                             rtb_x_post_h[bitpos]) +
                            rtb_x_pred[i];
        }
    }

    /* Sum: '<S11>/Add' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_VectorConcatenate_idx_0 = (*ved__wye_U_VED_InternalData_in)
                                      .ved__wpp_out.wheel_velo_front_left_var +
                                  (*ved__wye_U_VED_InternalData_in)
                                      .ved__wpp_out.wheel_velo_front_right_var;

    /* Sum: '<S11>/Add1' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_R_rear = (*ved__wye_U_VED_InternalData_in)
                     .ved__wpp_out.wheel_velo_rear_left_var +
                 (*ved__wye_U_VED_InternalData_in)
                     .ved__wpp_out.wheel_velo_rear_right_var;

    /* Sum: '<S70>/Sum' incorporates:
     *  Gain: '<S70>/Gain'
     *  Sum: '<S70>/Diff'
     *  UnitDelay: '<S70>/UD'
     */
    rtb_Sum_c =
        ((ved__wye_DWork->UD_DSTATE_b - rtb_R_rear) * 0.998F) + rtb_R_rear;

    /* Embedded MATLAB: '<S62>/adapt_R_matrix' incorporates:
     *  Constant: '<S62>/R'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/make_A_Q_R1/R/adapt_R_matrix': '<S69>:1'
     */
    /*  build the R matrix */
    /* '<S69>:1:3' */
    for (i = 0; i < 4; i++) {
        rtb_R_out_p[i] = ved__wye_R_wld_p[(i)];
    }

    if ((((int32_T)rtb_R_onoff_front) == 0) ||
        (((int32_T)rtb_R_onoff_rear) == 0)) {
        /* '<S69>:1:4' */
        /* '<S69>:1:5' */
        for (i = 0; i < 4; i++) {
            rtb_R_out_p[i] = ved__wye_R_wld_p[(i)] * 10000.0F;
        }
    }

    if ((rtb_R_rear > (rtb_Sum_c * 1.05F)) ||
        (((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__whs_offset.offset_ratio_front_dev >=
          1.0F) &&
         ((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__whs_offset.offset_ratio_rear_dev >=
          1.0F))) {
        /* '<S69>:1:10' */
        /* '<S69>:1:11' */
        rtb_Rroff = 1U;
    } else {
        /* '<S69>:1:13' */
        rtb_Rroff = 0U;

        /*  if sum variance of ratio front is above */
        if (rtb_VectorConcatenate_idx_0 > ved__wye_R_control_p) {
            /* '<S69>:1:15' */
            /* '<S69>:1:16' */
            rtb_R_out_p[0] = rtb_R_out_p[0] * 100000.0F;

            /* '<S69>:1:17' */
            rtb_Rroff = 1U;
        }

        /*  if sum variance of ratio rear is above */
        if (rtb_R_rear > ved__wye_R_control_p) {
            /* '<S69>:1:21' */
            /* '<S69>:1:22' */
            rtb_R_out_p[3] = rtb_R_out_p[3] * 100000.0F;

            /* '<S69>:1:23' */
            rtb_Rroff = 1U;
        }

        /*  if yaw rate difference front rear is above specified value turn off
         * the estimation */
        if (rtb_Abs3 > ved__wye_Q_sigmas_dyn_off_p[4]) {
            /* '<S69>:1:27' */
            /* '<S69>:1:28' */
            rtb_Rroff = 1U;
        }
    }

    /* Embedded MATLAB: '<S39>/hx_func' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'estimate_wheel_load_dep_front/EKF/hx/hx_func':
     * '<S47>:1' */
    /*  Transformation from state space to measurement space */
    /*  Wheel Load dep filter */
    /* '<S47>:1:4' */
    /* '<S47>:1:5' */
    /*  yawFmeas = -(Qf2^2-Kf2^2)/(Qf2^2+Kf2^2)*vx/s */
    /*  yawRmeas = -2*(Qr-Kr)/(Qr+Kr)*vx/s */
    /*   */
    /*  yawXFmeas = yaw*(1+1/1000000*K*h*m*vx^2/s^2) */
    /*    */
    /*  z = -(Qf2^2-Kf2^2)/(Qf2^2+Kf2^2)*vx/s */
    /*             -2*(Qr-Kr)/(Qr+Kr)*vx/s */
    /*    */
    /*  h =  yaw*(1+1/1000000*K*h*m*vx^2/s^2) */
    /*                                yaw */
    /*    */
    /*  H1 =[ 0, 1/1000000*(1000000*s^2+K*h*m*vx^2)/s^2,
     * 1/1000000*yaw*h*m*vx^2/s^2] */
    /*      [ 0,                                      1, 0] */
    /*     */
    /* '<S47>:1:23' */
    rtb_Yk1 = (*ved__wye_U_VED_InputData).Parameter.VED_Kf_VehWeight_kg *
              (*ved__wye_U_VED_InputData).Parameter.VED_Kf_AxisLoadDistr_per;

    /* '<S47>:1:24' */
    vx_quad = (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo *
              (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo;

    /* '<S47>:1:25' */
    rtb_VectorConcatenate_idx =
        (*ved__wye_U_VED_InputData).Parameter.VED_Kf_TrackWidthFront_met *
        (*ved__wye_U_VED_InputData).Parameter.VED_Kf_TrackWidthFront_met;

    /* '<S47>:1:26' */
    rtb_hx[0] = ((((((1.0E-6F * rtb_x_pred[2]) *
                     (*ved__wye_U_VED_InputData)
                         .Parameter.VED_Kf_CntrOfGravHeight_met) *
                    rtb_Yk1) *
                   vx_quad) /
                  rtb_VectorConcatenate_idx) +
                 1.0F) *
                rtb_x_pred[1];
    rtb_hx[1] = rtb_x_pred[1];

    /* '<S47>:1:29' */
    rtb_H[0] = 0.0F;
    rtb_H[2] =
        (((((rtb_x_pred[2] * (*ved__wye_U_VED_InputData)
                                 .Parameter.VED_Kf_CntrOfGravHeight_met) *
            rtb_Yk1) *
           vx_quad) +
          (1.0E+6F * rtb_VectorConcatenate_idx)) *
         1.0E-6F) /
        rtb_VectorConcatenate_idx;
    rtb_H[4] =
        ((((1.0E-6F * rtb_x_pred[1]) *
           (*ved__wye_U_VED_InputData).Parameter.VED_Kf_CntrOfGravHeight_met) *
          rtb_Yk1) *
         vx_quad) /
        rtb_VectorConcatenate_idx;
    for (i = 0; i < 3; i++) {
        rtb_H[1 + (i << 1)] = (real32_T)tmp_1[i];
    }

    /* Product: '<S61>/Divide' incorporates:
     *  Constant: '<S61>/Constant'
     *  Product: '<S61>/Product1'
     */
    rtb_Yk1 =
        (ved__wye_B->Time2Sec.Time2Sec * ved__wye_B->Time2Sec.Time2Sec) * 0.5F;

    /* Embedded MATLAB: '<S33>/get_gain_bias' */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/make_A_Q_R1/get_gain_bias': '<S65>:1' */
    /*  extract the gain control parameters */
    /* '<S65>:1:3' */
    /* '<S65>:1:4' */
    /* '<S65>:1:5' */

    /* Embedded MATLAB: '<S33>/q_gain_delta' */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/make_A_Q_R1/q_gain_delta': '<S66>:1' */
    /*  this feature is deactivated the diff is always  */
    /* '<S66>:1:3' */
    /* single(0.0000001);   */

    /* Sum: '<S33>/Sum1' incorporates:
     *  Abs: '<S33>/Abs1'
     *  Product: '<S33>/Product'
     */
    rtb_VectorConcatenate_idx = (((real32_T)fabs(ved__wye_Q_gain_wld_p[2])) *
                                 ved__wye_Q_gain_wld_p[0]) +
                                ved__wye_Q_gain_wld_p[1];

    /* S-Function (sfix_bitop): '<S64>/Bitwise Operator' incorporates:
     *  Constant: '<S64>/bitmask'
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_BitwiseOperator =
        (uint16_T)((*ved__wye_U_VED_InputData).Frame.CaliMode &
                   ((uint16_T)VED_CAL_WHS_LOAD));

    /* Embedded MATLAB: '<S64>/get_bit' */
    ved__wye_get_bit(rtb_BitwiseOperator, ((uint16_T)VED_CAL_WHS_LOAD),
                     &ved__wye_B->sf_get_bit);

    /* Embedded MATLAB: '<S71>/hold_last_10_values' incorporates:
     *  UnitDelay: '<S71>/hold_counter'
     *  UnitDelay: '<S71>/hold_value'
     */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/make_A_Q_R1/R/hold_last_10_values/hold_last_10_values':
     * '<S72>:1' */
    /*  Hold value 10 cycle if input value is zero */
    if (ved__wye_B->sf_get_bit.y == 1) {
        /* '<S72>:1:4' */
        /* '<S72>:1:5' */
        rtb_Rroff = 1U;

        /* '<S72>:1:6' */
        rtb_counter_out = 0U;
    } else if ((rtb_Rroff != 0) || (ved__wye_DWork->hold_counter_DSTATE > 10)) {
        /* '<S72>:1:8' */
        /* '<S72>:1:9' */
        /* '<S72>:1:10' */
        rtb_counter_out = 0U;
    } else {
        /* '<S72>:1:12' */
        rtb_Rroff = ved__wye_DWork->hold_value_DSTATE;

        /* '<S72>:1:13' */
        rtb_counter_out = (uint8_T)(ved__wye_DWork->hold_counter_DSTATE + 1);
    }

    rtb_TmpSignalConversionAtSFun_c[0] = rtb_Yk1;
    rtb_TmpSignalConversionAtSFun_c[1] = ved__wye_B->Time2Sec.Time2Sec;

    /* Embedded MATLAB: '<S61>/makeQ' incorporates:
     *  Constant: '<S61>/sigma_model'
     */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/make_A_Q_R1/Q/makeQ': '<S68>:1' */
    /*  calculates the model noise matrix Q */
    /*  and the noise for the drift */
    /* '<S68>:1:4' */
    sigma_model_gain[0] =
        ved__wye_Q_sigmas_wld_p[0] * rtb_VectorConcatenate_idx;
    sigma_model_gain[1] =
        ved__wye_Q_sigmas_wld_p[1] * rtb_VectorConcatenate_idx;

    /*  if the velocity is over a specified value und the lat_accel too use the
     * normal sigma value for the offset */
    /*  otherwise make the model offset drift noise very small so that it does
     * nothing */
    /*  if ((veh_velocity >= ved__wye_Q_sigmas_dyn_wld_p(1)) && (veh_lat_accel
     * <= ved__wye_Q_sigmas_dyn_wld_p(2) )) */
    /*  Q1 = (G.*sigma_model_gain)*(sigma_model_gain.*G)'; */
    /*  Q = [Q1 [0; 0]; 0 0 sigma_model(3)*sigma_model(3)*CycleTime*CycleTime];
     */
    /*  else */
    /*  Q1 = (G.*sigma_model_gain)*(sigma_model_gain.*G)'; */
    /*  Q = [Q1 [0; 0]; 0 0
     * ved__wye_Q_sigmas_dyn_off_p(3)*ved__wye_Q_sigmas_dyn_off_p(3)*CycleTime*CycleTime];
     */
    /*  end; */
    if (rtb_Rroff == 0) {
        /* '<S68>:1:14' */
        /* '<S68>:1:15' */
        /* '<S68>:1:16' */
        for (i = 0; i < 2; i++) {
            rtb_x_post_j[i] =
                rtb_TmpSignalConversionAtSFun_c[i] * sigma_model_gain[i];
            sigma_model_gain_0[i] =
                sigma_model_gain[i] * rtb_TmpSignalConversionAtSFun_c[i];
        }

        for (i = 0; i < 2; i++) {
            for (bitpos = 0; bitpos < 2; bitpos++) {
                rtb_TmpSignalConversionAtSFun_0[bitpos + (i << 1)] =
                    rtb_x_post_j[bitpos] * sigma_model_gain_0[i];
            }
        }

        for (i = 0; i < 2; i++) {
            for (bitpos = 0; bitpos < 2; bitpos++) {
                rtb_Q_g[bitpos + (3 * i)] =
                    rtb_TmpSignalConversionAtSFun_0[(i << 1) + bitpos];
            }
        }

        for (i = 0; i < 2; i++) {
            rtb_Q_g[i + 6] = 0.0F;
        }

        rtb_Q_g[2] = 0.0F;
        rtb_Q_g[5] = 0.0F;
        rtb_Q_g[8] =
            ((ved__wye_Q_sigmas_wld_p[2] * ved__wye_Q_sigmas_wld_p[2]) *
             ved__wye_B->Time2Sec.Time2Sec) *
            ved__wye_B->Time2Sec.Time2Sec;
    } else {
        /* '<S68>:1:18' */
        /* '<S68>:1:19' */
        for (i = 0; i < 2; i++) {
            rtb_x_post_j[i] =
                rtb_TmpSignalConversionAtSFun_c[i] * sigma_model_gain[i];
            sigma_model_gain_0[i] =
                sigma_model_gain[i] * rtb_TmpSignalConversionAtSFun_c[i];
        }

        for (i = 0; i < 2; i++) {
            for (bitpos = 0; bitpos < 2; bitpos++) {
                rtb_TmpSignalConversionAtSFun_0[bitpos + (i << 1)] =
                    rtb_x_post_j[bitpos] * sigma_model_gain_0[i];
            }
        }

        for (i = 0; i < 2; i++) {
            for (bitpos = 0; bitpos < 2; bitpos++) {
                rtb_Q_g[bitpos + (3 * i)] =
                    rtb_TmpSignalConversionAtSFun_0[(i << 1) + bitpos];
            }
        }

        for (i = 0; i < 2; i++) {
            rtb_Q_g[i + 6] = 0.0F;
        }

        rtb_Q_g[2] = 0.0F;
        rtb_Q_g[5] = 0.0F;
        rtb_Q_g[8] = ((ved__wye_Q_sigmas_dyn_off_wld_p *
                       ved__wye_Q_sigmas_dyn_off_wld_p) *
                      ved__wye_B->Time2Sec.Time2Sec) *
                     ved__wye_B->Time2Sec.Time2Sec;
    }

    /* Sum: '<S29>/APA_Q' incorporates:
     *  Math: '<S29>/At'
     *  Product: '<S29>/APAt'
     *  UnitDelay: '<S29>/P_delay_wld'
     */
    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            tmp[i + (3 * bitpos)] = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                tmp[i + (3 * bitpos)] =
                    (ved__wye_B->sf_make_A_matrix.A[(3 * i_0) + i] *
                     ved__wye_DWork->P_delay_wld_DSTATE[(3 * bitpos) + i_0]) +
                    tmp[(3 * bitpos) + i];
            }
        }
    }

    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_VectorConcatenate_idx = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_VectorConcatenate_idx +=
                    tmp[(3 * i_0) + i] *
                    ved__wye_B->sf_make_A_matrix.A[(3 * i_0) + bitpos];
            }

            rtb_P_pred[i + (3 * bitpos)] =
                rtb_Q_g[(3 * bitpos) + i] + rtb_VectorConcatenate_idx;
        }
    }

    /* Math: '<S29>/Ht' */
    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_Ht[bitpos + (3 * i)] = rtb_H[(bitpos << 1) + i];
        }
    }

    /* Sum: '<S29>/HPH_R' incorporates:
     *  Product: '<S29>/HPHt'
     */
    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_H_0[i + (bitpos << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_H_0[i + (bitpos << 1)] =
                    (rtb_H[(i_0 << 1) + i] * rtb_P_pred[(3 * bitpos) + i_0]) +
                    rtb_H_0[(bitpos << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_VectorConcatenate_idx = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_VectorConcatenate_idx +=
                    rtb_H_0[(i_0 << 1) + i] * rtb_Ht[(3 * bitpos) + i_0];
            }

            rtb_HPHt_R[i + (bitpos << 1)] =
                rtb_R_out_p[(bitpos << 1) + i] + rtb_VectorConcatenate_idx;
        }
    }

    /* Embedded MATLAB: '<S34>/calculate determinant' */
    ved__wye_calculatedeterminant(rtb_HPHt_R,
                                  &ved__wye_B->sf_calculatedeterminant);

    /* Embedded MATLAB: '<S29>/Reset_x_pred' */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/EKF/Reset_x_pred': '<S37>:1' */
    if (((real32_T)fabs(ved__wye_B->sf_calculatedeterminant.y)) <= 1.0E-16F) {
        /* '<S37>:1:4' */
        /* '<S37>:1:6' */
        rtb_x_pred[0] = 0.0F;
        rtb_x_pred[1] = 0.0F;
        rtb_x_pred[2] = rtb_x_post[2];
    } else {
        /* '<S37>:1:10' */
    }

    /* Product: '<S29>/P_pred*Ht' */
    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_P_predHt[i + (3 * bitpos)] = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_P_predHt[i + (3 * bitpos)] =
                    (rtb_P_pred[(3 * i_0) + i] * rtb_Ht[(3 * bitpos) + i_0]) +
                    rtb_P_predHt[(3 * bitpos) + i];
            }
        }
    }

    /* If: '<S34>/If' incorporates:
     *  ActionPort: '<S43>/Action Port'
     *  ActionPort: '<S44>/Action Port'
     *  SubSystem: '<S34>/calculate the gain'
     *  SubSystem: '<S34>/set gain to default value'
     */
    if (ved__wye_B->sf_calculatedeterminant.y > 1.0E-16F) {
        /* Product: '<S43>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {2, 2, 2, 3, 2, 2};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5[0], rtb_HPHt_R,
                            &ved__wye_DWork->PHt_HPHt_R_DWORK4[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1[0],
                            &PHt_HPHt_R_DWORK2[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K, rtb_P_predHt, &PHt_HPHt_R_DWORK5[0],
                             &dims[3]);
        }
    } else {
        /* Constant: '<S44>/Constant' */
        for (i = 0; i < 6; i++) {
            rtb_K[i] = ved__wye_ConstP.Constant_Value_a[(i)];
        }
    }

    /* Embedded MATLAB: '<S29>/init_x_yaw1' incorporates:
     *  Product: '<S29>/Knu'
     *  Sum: '<S29>/x_Knu'
     *  Sum: '<S29>/z_Hx'
     */
    /* Embedded MATLAB Function 'estimate_wheel_load_dep_front/EKF/init_x_yaw1':
     * '<S41>:1' */
    /*  If Wld_Control is 1, so no wld should be leard. */
    /*  and the wld state in the filter is not updated */
    /* '<S41>:1:5' */
    rtb_x_post_j[0] = frfilt;
    rtb_x_post_j[1] = refilt;
    for (i = 0; i < 2; i++) {
        sigma_model_gain_0[i] = rtb_x_post_j[i] - rtb_hx[i];
    }

    for (i = 0; i < 3; i++) {
        rtb_VectorConcatenate_idx = 0.0F;
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_VectorConcatenate_idx +=
                rtb_K[(3 * bitpos) + i] * sigma_model_gain_0[bitpos];
        }

        rtb_x_out[i] = rtb_x_pred[i] + rtb_VectorConcatenate_idx;
    }

    /*  if 1 don't update wld */
    if (rtb_Rroff == 1) {
        /* '<S41>:1:7' */
        /* '<S41>:1:8' */
        rtb_x_out[2] = rtb_x_pred[2];
    }

    /* Embedded MATLAB: '<S76>/adapt_R_matrix' incorporates:
     *  Constant: '<S76>/R'
     */
    /* Embedded MATLAB Function 'make_A_Q_R/R/adapt_R_matrix': '<S83>:1' */
    /*  build the R matrix */
    /* '<S83>:1:3' */
    for (i = 0; i < 9; i++) {
        rtb_R_out_e[i] = ved__wye_R_p[(i)];
    }

    /* '<S83>:1:4' */
    rtb_R_onoff = TRUE;
    if (((((int32_T)rtb_R_onoff_front) == 0) ||
         (((int32_T)rtb_R_onoff_rear) == 0)) ||
        (((int32_T)rtb_R_yaw_valid) == 0)) {
        /* '<S83>:1:5' */
        /* '<S83>:1:6' */
        for (i = 0; i < 9; i++) {
            rtb_R_out_e[i] = ved__wye_R_p[(i)] * 10000.0F;
        }

        /* '<S83>:1:7' */
        rtb_R_onoff = FALSE;
    }

    /*  if sum variance of ratio front is above  */
    if (rtb_VectorConcatenate_idx_0 > ved__wye_R_control_p) {
        /* '<S83>:1:11' */
        /* '<S83>:1:12' */
        rtb_R_out_e[0] = rtb_R_out_e[0] * 100000.0F;
    }

    /*  if sum variance of ratio rear is above  */
    if (rtb_R_rear > ved__wye_R_control_p) {
        /* '<S83>:1:16' */
        /* '<S83>:1:17' */
        rtb_R_out_e[4] = rtb_R_out_e[4] * 100000.0F;
    }

    /*  if (R_onoff_front == true) */
    /*      R_out(1,1) = R_in(1,1)*R_quot_front;     */
    /*  else */
    /*      R_out(1,1) = R_in(1,1) * 10000; */
    /*  end; */
    /*   */
    /*  if (R_onoff_rear == true) */
    /*      R_out(2,2) = R_in(2,2)*R_quot_rear;     */
    /*  else */
    /*      R_out(2,2) = R_in(2,2) * 10000; */
    /*  end; */
    /*   */
    /*  if (R_yaw_valid == false) */
    /*      R_out(3,3) = R_in(3,3) * 10000; */
    /*  end; */

    /* BusSelector: '<S9>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CycleTime_j = (*ved__wye_U_VED_InputData).Frame.CycleTime;

    /* Outputs for atomic SubSystem: '<S9>/Time2Sec' */
    ved__wye_Time2Sec(rtb_CycleTime_j, &ved__wye_B->Time2Sec_l);

    /* end of Outputs for SubSystem: '<S9>/Time2Sec' */

    /* Product: '<S75>/Divide' incorporates:
     *  Constant: '<S75>/Constant'
     *  Product: '<S75>/Product1'
     */
    rtb_Yk1 =
        (ved__wye_B->Time2Sec_l.Time2Sec * ved__wye_B->Time2Sec_l.Time2Sec) *
        0.5F;

    /* Embedded MATLAB: '<S9>/get_gain_bias' */
    /* Embedded MATLAB Function 'make_A_Q_R/get_gain_bias': '<S79>:1' */
    /*  extract the gain control parameters */
    /* '<S79>:1:3' */
    /* '<S79>:1:4' */
    /* '<S79>:1:5' */

    /* Embedded MATLAB: '<S9>/q_gain_delta' */
    /* Embedded MATLAB Function 'make_A_Q_R/q_gain_delta': '<S80>:1' */
    /*  this feature is deactivated the diff is always  */
    /* '<S80>:1:3' */
    /* single(0.0000001);   */

    /* Sum: '<S9>/Sum1' incorporates:
     *  Abs: '<S9>/Abs1'
     *  Product: '<S9>/Product'
     */
    rtb_VectorConcatenate_idx =
        (((real32_T)fabs(ved__wye_Q_gain_p[2])) * ved__wye_Q_gain_p[0]) +
        ved__wye_Q_gain_p[1];

    /* BusSelector: '<S75>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    kf_quad = (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo;
    WldFactFront = (*ved__wye_U_VED_InternalData_in).ved__ye_out.veh_lat_accel;
    rtb_Init = (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_accel;

    /* S-Function (sfix_bitop): '<S78>/Bitwise Operator' incorporates:
     *  Constant: '<S78>/bitmask'
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_BitwiseOperator_c =
        (uint16_T)((*ved__wye_U_VED_InputData).Frame.CaliMode &
                   ((uint16_T)VED_CAL_YWR_OFFS_DYN));

    /* Embedded MATLAB: '<S78>/get_bit' */
    ved__wye_get_bit(rtb_BitwiseOperator_c, ((uint16_T)VED_CAL_YWR_OFFS_DYN),
                     &ved__wye_B->sf_get_bit_i);

    rtb_hx[0] = rtb_Yk1;
    rtb_hx[1] = ved__wye_B->Time2Sec_l.Time2Sec;

    /* Embedded MATLAB: '<S75>/makeQ' incorporates:
     *  Abs: '<S75>/Abs'
     *  Abs: '<S75>/Abs1'
     *  Abs: '<S75>/Abs2'
     *  Constant: '<S75>/sigma_model'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'make_A_Q_R/Q/makeQ': '<S82>:1' */
    /*  calculates the model noise matrix Q */
    /*  and the noise for the drift */
    /* '<S82>:1:4' */
    sigma_model_gain[0] = ved__wye_Q_sigmas_p[0] * rtb_VectorConcatenate_idx;
    sigma_model_gain[1] = ved__wye_Q_sigmas_p[1] * rtb_VectorConcatenate_idx;

    /*  if the velocity is over a specified value und the lat_accel too use the
     * normal sigma value for the offset */
    /*  otherwise make the model offset drift noise very small so that it does
     * nothing */
    if ((((((real32_T)fabs(kf_quad)) >= ved__wye_Q_sigmas_dyn_off_p[0]) &&
          (((real32_T)fabs(WldFactFront)) <= ved__wye_Q_sigmas_dyn_off_p[1])) &&
         (((real32_T)fabs(rtb_Init)) <= ved__wye_Q_sigmas_dyn_off_p[3])) &&
        (rtb_Abs3 <= ved__wye_Q_sigmas_dyn_off_p[4])) {
        /* '<S82>:1:7' */
        /* '<S82>:1:8' */
        /* '<S82>:1:9' */
        for (i = 0; i < 2; i++) {
            rtb_x_post_j[i] = rtb_hx[i] * sigma_model_gain[i];
            sigma_model_gain_0[i] = sigma_model_gain[i] * rtb_hx[i];
        }

        for (i = 0; i < 2; i++) {
            for (bitpos = 0; bitpos < 2; bitpos++) {
                rtb_TmpSignalConversionAtSFun_0[bitpos + (i << 1)] =
                    rtb_x_post_j[bitpos] * sigma_model_gain_0[i];
            }
        }

        for (i = 0; i < 2; i++) {
            for (bitpos = 0; bitpos < 2; bitpos++) {
                rtb_Q_m[bitpos + (3 * i)] =
                    rtb_TmpSignalConversionAtSFun_0[(i << 1) + bitpos];
            }
        }

        for (i = 0; i < 2; i++) {
            rtb_Q_m[i + 6] = 0.0F;
        }

        rtb_Q_m[2] = 0.0F;
        rtb_Q_m[5] = 0.0F;
        rtb_Q_m[8] = ((ved__wye_Q_sigmas_p[2] * ved__wye_Q_sigmas_p[2]) *
                      ved__wye_B->Time2Sec_l.Time2Sec) *
                     ved__wye_B->Time2Sec_l.Time2Sec;

        /* '<S82>:1:10' */
        rtb_init = 0U;
    } else {
        /* '<S82>:1:12' */
        /* '<S82>:1:13' */
        for (i = 0; i < 2; i++) {
            rtb_x_post_j[i] = rtb_hx[i] * sigma_model_gain[i];
            sigma_model_gain_0[i] = sigma_model_gain[i] * rtb_hx[i];
        }

        for (i = 0; i < 2; i++) {
            for (bitpos = 0; bitpos < 2; bitpos++) {
                rtb_TmpSignalConversionAtSFun_0[bitpos + (i << 1)] =
                    rtb_x_post_j[bitpos] * sigma_model_gain_0[i];
            }
        }

        for (i = 0; i < 2; i++) {
            for (bitpos = 0; bitpos < 2; bitpos++) {
                rtb_Q_m[bitpos + (3 * i)] =
                    rtb_TmpSignalConversionAtSFun_0[(i << 1) + bitpos];
            }
        }

        for (i = 0; i < 2; i++) {
            rtb_Q_m[i + 6] = 0.0F;
        }

        rtb_Q_m[2] = 0.0F;
        rtb_Q_m[5] = 0.0F;
        rtb_Q_m[8] =
            ((ved__wye_Q_sigmas_dyn_off_p[2] * ved__wye_Q_sigmas_dyn_off_p[2]) *
             ved__wye_B->Time2Sec_l.Time2Sec) *
            ved__wye_B->Time2Sec_l.Time2Sec;

        /* '<S82>:1:14' */
        rtb_init = 1U;
    }

    /*  if the deviation of the front or rear axis offset is above 1.0 do not
     * use the */
    if (((*ved__wye_U_VED_InternalData_in)
             .ved__offsets_in.ved__whs_offset.offset_ratio_front_dev >= 1.0F) ||
        ((*ved__wye_U_VED_InternalData_in)
             .ved__offsets_in.ved__whs_offset.offset_ratio_rear_dev >= 1.0F)) {
        /* '<S82>:1:18' */
        /* '<S82>:1:19' */
        rtb_init = 1U;
    }

    /*  if the ywr off dyn calibration is on, switch off dyn yaw rate offset
     * estimation */
    if (ved__wye_B->sf_get_bit_i.y == 1) {
        /* '<S82>:1:23' */
        /* '<S82>:1:24' */
        rtb_init = 1U;
    }

    /* Embedded MATLAB: '<S74>/make_A_matrix' */
    ved__wye_make_A_matrix(ved__wye_B->Time2Sec_l.Time2Sec,
                           &ved__wye_B->sf_make_A_matrix_d);

    /* Embedded MATLAB: '<S2>/init_P_yaw' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  UnitDelay: '<S2>/P_delay_dyn_off'
     */
    /* Embedded MATLAB Function 'KF/init_P_yaw': '<S18>:1' */
    /*  Init P matrix with external yaw rate offset variance */
    /* '<S18>:1:4' */
    for (i = 0; i < 9; i++) {
        rtb_Q_g[i] = ved__wye_DWork->P_delay_dyn_off_DSTATE[(i)];
    }

    if ((((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__yaw_offset.state == 1) ||
         ((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__yaw_offset.state == 2)) ||
        ((*ved__wye_U_VED_InternalData_in)
             .ved__offsets_in.ved__yaw_offset.state == 3)) {
        /* '<S18>:1:5' */
        /* '<S18>:1:6' */
        rtb_Q_g[8] = (*ved__wye_U_VED_InternalData_in)
                         .ved__offsets_in.ved__yaw_offset.var;
    }

    /* Sum: '<S2>/APA_Q' incorporates:
     *  Math: '<S2>/At'
     *  Product: '<S2>/APAt'
     */
    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            tmp[i + (3 * bitpos)] = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                tmp[i + (3 * bitpos)] =
                    (ved__wye_B->sf_make_A_matrix_d.A[(3 * i_0) + i] *
                     rtb_Q_g[(3 * bitpos) + i_0]) +
                    tmp[(3 * bitpos) + i];
            }
        }
    }

    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_VectorConcatenate_idx = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_VectorConcatenate_idx +=
                    tmp[(3 * i_0) + i] *
                    ved__wye_B->sf_make_A_matrix_d.A[(3 * i_0) + bitpos];
            }

            rtb_P_pred_n[i + (3 * bitpos)] =
                rtb_Q_m[(3 * bitpos) + i] + rtb_VectorConcatenate_idx;
        }
    }

    /* Sum: '<S2>/HPH_R' incorporates:
     *  Constant: '<S2>/H_const'
     *  Product: '<S2>/HPHt'
     */
    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            tmp[i + (3 * bitpos)] = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                tmp[i + (3 * bitpos)] =
                    (ved__wye_ConstP.H_const_Value[(3 * i_0) + i] *
                     rtb_P_pred_n[(3 * bitpos) + i_0]) +
                    tmp[(3 * bitpos) + i];
            }
        }
    }

    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_VectorConcatenate_idx = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_VectorConcatenate_idx +=
                    tmp[(3 * i_0) + i] * ved__wye_ConstB.Ht[(3 * bitpos) + i_0];
            }

            rtb_HPHt_R_o[i + (3 * bitpos)] =
                rtb_R_out_e[(3 * bitpos) + i] + rtb_VectorConcatenate_idx;
        }
    }

    /* Product: '<S2>/P_pred*Ht' */
    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_P_predHt_m[i + (3 * bitpos)] = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_P_predHt_m[i + (3 * bitpos)] =
                    (rtb_P_pred_n[(3 * i_0) + i] *
                     ved__wye_ConstB.Ht[(3 * bitpos) + i_0]) +
                    rtb_P_predHt_m[(3 * bitpos) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S14>/calculate determinant' */
    /* Embedded MATLAB Function 'KF/Calculate Kalman gain PHt_(HPHt_R)/calculate
     * determinant': '<S20>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /*  y = single(det(u)); */
    /* '<S20>:1:4' */
    rtb_Yk1 = ((((rtb_HPHt_R_o[4] * rtb_HPHt_R_o[8]) -
                 (rtb_HPHt_R_o[7] * rtb_HPHt_R_o[5])) *
                rtb_HPHt_R_o[0]) -
               (((rtb_HPHt_R_o[1] * rtb_HPHt_R_o[8]) -
                 (rtb_HPHt_R_o[7] * rtb_HPHt_R_o[2])) *
                rtb_HPHt_R_o[3])) +
              (((rtb_HPHt_R_o[1] * rtb_HPHt_R_o[5]) -
                (rtb_HPHt_R_o[4] * rtb_HPHt_R_o[2])) *
               rtb_HPHt_R_o[6]);

    /* If: '<S14>/If' incorporates:
     *  ActionPort: '<S21>/Action Port'
     *  ActionPort: '<S22>/Action Port'
     *  SubSystem: '<S14>/calculate the gain'
     *  SubSystem: '<S14>/set gain to default value'
     */
    if (rtb_Yk1 > 1.0E-16F) {
        /* Product: '<S21>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {3, 3, 3, 3, 3, 3};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5_m[0], rtb_HPHt_R_o,
                            &ved__wye_DWork->PHt_HPHt_R_DWORK4_b[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1_f[0],
                            &PHt_HPHt_R_DWORK2_o[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3_l[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K_d, rtb_P_predHt_m, &PHt_HPHt_R_DWORK5_m[0],
                             &dims[3]);
        }
    } else {
        /* Constant: '<S22>/Constant' */
        for (i = 0; i < 9; i++) {
            rtb_K_d[i] = ved__wye_ConstP.Constant_Value[(i)];
        }
    }

    /* Embedded MATLAB: '<S2>/Reset_P_pred' */
    /* Embedded MATLAB Function 'KF/Reset_P_pred': '<S15>:1' */
    if (((real32_T)fabs(rtb_Yk1)) <= 1.0E-16F) {
        /* '<S15>:1:4' */
        /* '<S15>:1:6' */
        for (i = 0; i < 9; i++) {
            rtb_P_pred_n[i] = 0.0F;
        }
    } else {
        /* '<S15>:1:10' */
    }

    /* Product: '<S2>/P_pred_(1_KH)' incorporates:
     *  Constant: '<S2>/H_const'
     *  Constant: '<S2>/eye'
     *  Product: '<S2>/K*H'
     *  Sum: '<S2>/1_KH'
     */
    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_VectorConcatenate_idx = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_VectorConcatenate_idx +=
                    rtb_K_d[(3 * i_0) + i] *
                    ved__wye_ConstP.H_const_Value[(3 * bitpos) + i_0];
            }

            tmp[i + (3 * bitpos)] = ved__wye_ConstP.pooled6[(3 * bitpos) + i] -
                                    rtb_VectorConcatenate_idx;
        }
    }

    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_Q_g[i + (3 * bitpos)] = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                rtb_Q_g[i + (3 * bitpos)] =
                    (tmp[(3 * i_0) + i] * rtb_P_pred_n[(3 * bitpos) + i_0]) +
                    rtb_Q_g[(3 * bitpos) + i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/correct_variance' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'correct_variance': '<S5>:1' */
    /*  the output variance for this module is rasied at low speed */
    /*  if input values are not valid set output variance to high value */
    /*  if the velocity is in a low speed range eg. between 1 and 10 m/s  */
    /*  the variance is linear increasing to the bottom limit with the velocity
     */
    /*  below this botom limit the variance is constant */
    /*  if the velocity if above the upper limit nothing is done */
    if (((int32_T)rtb_R_onoff) == 1) {
        /* '<S5>:1:8' */
        if (((*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo >
             ved__wye_P_correct_p[0]) &&
            ((*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo <
             ved__wye_P_correct_p[1])) {
            /* '<S5>:1:9' */
            /* '<S5>:1:10' */
            rtb_VectorConcatenate_idx =
                ((*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                 ved__wye_P_correct_p[0]) *
                ved__wye_P_correct_p[2];
            for (i = 0; i < 3; i++) {
                rtb_x_pred[i] =
                    (rtb_Q_g[i << 2] +
                     (ved__wye_P_correct_p[3] - rtb_VectorConcatenate_idx)) +
                    6.0E-5F;
            }
        } else if ((*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo <=
                   ved__wye_P_correct_p[0]) {
            /* '<S5>:1:11' */
            /* '<S5>:1:12' */
            for (i = 0; i < 3; i++) {
                rtb_x_pred[i] =
                    (rtb_Q_g[i << 2] + ved__wye_P_correct_p[3]) + 6.0E-5F;
            }
        } else {
            /* '<S5>:1:14' */
            for (i = 0; i < 3; i++) {
                rtb_x_pred[i] = rtb_Q_g[i << 2] + 6.0E-5F;
            }
        }
    } else {
        /* '<S5>:1:17' */
        for (i = 0; i < 3; i++) {
            rtb_x_pred[i] = ved__wye_P_correct_p[3];
        }
    }

    /* Embedded MATLAB: '<S2>/init_x_yaw' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  UnitDelay: '<S2>/x_delay_dyn_off'
     */
    /* Embedded MATLAB Function 'KF/init_x_yaw': '<S19>:1' */
    /*  Init x state with external yaw rate offset variance */
    /*  and limit estimated yaw rate offset */
    /* '<S19>:1:5' */
    for (i = 0; i < 3; i++) {
        rtb_x_post[i] = ved__wye_DWork->x_delay_dyn_off_DSTATE[(i)];
    }

    /*  if  */
    if (ved__wye_DWork->x_delay_dyn_off_DSTATE[2] < -0.04F) {
        /* '<S19>:1:8' */
        /* '<S19>:1:9' */
        rtb_x_post[2] = -0.04F;
    }

    /*  if  */
    if (ved__wye_DWork->x_delay_dyn_off_DSTATE[2] > 0.05F) {
        /* '<S19>:1:12' */
        /* '<S19>:1:13' */
        rtb_x_post[2] = 0.05F;
    }

    if ((((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__yaw_offset.state == 1) ||
         ((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__yaw_offset.state == 2)) ||
        ((*ved__wye_U_VED_InternalData_in)
             .ved__offsets_in.ved__yaw_offset.state == 3)) {
        /* '<S19>:1:16' */
        /* '<S19>:1:17' */
        rtb_x_post[2] = (*ved__wye_U_VED_InternalData_in)
                            .ved__offsets_in.ved__yaw_offset.offset;
    }

    /* Product: '<S2>/Ax' */
    for (i = 0; i < 3; i++) {
        rtb_x_pred_m[i] = 0.0F;
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_x_pred_m[i] =
                (ved__wye_B->sf_make_A_matrix_d.A[(3 * bitpos) + i] *
                 rtb_x_post[bitpos]) +
                rtb_x_pred_m[i];
        }
    }

    /* Embedded MATLAB: '<S2>/Reset_x_pred' incorporates:
     *  UnitDelay: '<S2>/x_delay_dyn_off'
     */
    /* Embedded MATLAB Function 'KF/Reset_x_pred': '<S16>:1' */
    if (((real32_T)fabs(rtb_Yk1)) <= 1.0E-16F) {
        /* '<S16>:1:4' */
        /* '<S16>:1:6' */
        rtb_x_post[0] = 0.0F;
        rtb_x_post[1] = 0.0F;
        rtb_x_post[2] = ved__wye_DWork->x_delay_dyn_off_DSTATE[2];
    } else {
        /* '<S16>:1:10' */
        for (i = 0; i < 3; i++) {
            rtb_x_post[i] = rtb_x_pred_m[i];
        }
    }

    /* Embedded MATLAB: '<S2>/check_yaw_offset' incorporates:
     *  Constant: '<S2>/H_const'
     *  Product: '<S2>/Knu'
     *  Product: '<S2>/Product'
     *  Sum: '<S2>/x_Knu'
     *  Sum: '<S2>/z_Hx'
     */
    /* Embedded MATLAB Function 'KF/check_yaw_offset': '<S17>:1' */
    /*  If EG_Control is 1, so no EG should be leard. */
    /*  and the EG state in the filter is not updated */
    /* '<S17>:1:5' */
    rtb_x_post_h[0] = rtb_frfiltWithWldEst;
    rtb_x_post_h[1] = rtb_refiltWithWldEst;
    rtb_x_post_h[2] = rtb_VectorConcatenate_idx_1;
    for (i = 0; i < 3; i++) {
        tmp_0[i] = 0.0F;
        for (bitpos = 0; bitpos < 3; bitpos++) {
            tmp_0[i] = (ved__wye_ConstP.H_const_Value[(3 * bitpos) + i] *
                        rtb_x_pred_m[bitpos]) +
                       tmp_0[i];
        }

        rtb_frfiltWithWldEst_0[i] = rtb_x_post_h[i] - tmp_0[i];
    }

    for (i = 0; i < 3; i++) {
        rtb_VectorConcatenate_idx = 0.0F;
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_VectorConcatenate_idx +=
                rtb_K_d[(3 * bitpos) + i] * rtb_frfiltWithWldEst_0[bitpos];
        }

        rtb_x_post_h[i] = rtb_x_post[i] + rtb_VectorConcatenate_idx;
    }

    /*  if 0 don't update eg */
    if (rtb_init == 1) {
        /* '<S17>:1:7' */
        /* '<S17>:1:8' */
        rtb_x_post_h[2] = rtb_x_post[2];
    }

    /* Embedded MATLAB: '<S1>/init_x_yaw' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  UnitDelay: '<S1>/last_dyn_yaw_offset'
     */
    /* Embedded MATLAB Function 'Check_yaw_offset/init_x_yaw': '<S13>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S13>:1:5' */
    rtb_x_out_i = ved__wye_DWork->last_dyn_yaw_offset_DSTATE;
    if ((((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__yaw_offset.state == 1) ||
         ((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__yaw_offset.state == 2)) ||
        ((*ved__wye_U_VED_InternalData_in)
             .ved__offsets_in.ved__yaw_offset.state == 3)) {
        /* '<S13>:1:8' */
        /* '<S13>:1:9' */
        rtb_x_out_i = (*ved__wye_U_VED_InternalData_in)
                          .ved__offsets_in.ved__yaw_offset.offset;
    }

    /* Embedded MATLAB: '<S1>/check_yaw_offset' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  UnitDelay: '<S1>/dyn_yaw_off_overt_count'
     *  UnitDelay: '<S1>/last_dyn_yaw_offset_sign'
     */
    /* Embedded MATLAB Function 'Check_yaw_offset/check_yaw_offset': '<S12>:1'
     */
    /*  check if the yaw rate offset is above or below a resolution threshold */
    /*  if yes start a counter and count a specified time */
    /*  after this time use the next yaw rate offset resolution step. */
    /* '<S12>:1:5' */
    /* '<S12>:1:7' */
    rtb_Yk1 = rtb_x_post_h[2] - rtb_x_out_i;

    /* '<S12>:1:8' */
    rtb_sign_out = ved__wye_DWork->last_dyn_yaw_offset_sign_DSTATE;
    if (((real32_T)fabs(rtb_Yk1)) > ved__wye_yaw_offset_const_p[0]) {
        /* '<S12>:1:10' */
        if (((real32_T)fabs(ved__wye_DWork->dyn_yaw_off_overt_count_DSTATE)) <=
            0.0001F) {
            /* '<S12>:1:11' */
            if (rtb_Yk1 > 0.0F) {
                /* '<S12>:1:12' */
                /* '<S12>:1:13' */
                rtb_sign_out = 1;
            } else {
                if (rtb_Yk1 < 0.0F) {
                    /* '<S12>:1:14' */
                    /* '<S12>:1:15' */
                    rtb_sign_out = -1;
                }
            }
        }

        /* '<S12>:1:18' */
        rtb_VectorConcatenate_idx_1 =
            ved__wye_DWork->dyn_yaw_off_overt_count_DSTATE + 1.0F;
    } else {
        /* '<S12>:1:20' */
        rtb_VectorConcatenate_idx_1 = 0.0F;
    }

    /* '<S12>:1:23' */
    if (((rtb_VectorConcatenate_idx_1 *
          ((real32_T)(*ved__wye_U_VED_InputData).Frame.CycleTime)) /
         1000.0F) > ved__wye_yaw_offset_const_p[1]) {
        /* '<S12>:1:25' */
        if (ved__wye_DWork->last_dyn_yaw_offset_sign_DSTATE == 1) {
            /* '<S12>:1:26' */
            /* '<S12>:1:27' */
            rtb_x_out_i += ved__wye_yaw_offset_const_p[0];
        } else if (ved__wye_DWork->last_dyn_yaw_offset_sign_DSTATE == -1) {
            /* '<S12>:1:28' */
            /* '<S12>:1:29' */
            rtb_x_out_i -= ved__wye_yaw_offset_const_p[0];
        } else {
            /* '<S12>:1:31' */
        }
    }

    /* Embedded MATLAB: '<S87>/adapt_R_matrix' incorporates:
     *  Constant: '<S87>/R'
     *  Inport: '<Root>/VED_InternalData_in'
     *  UnitDelay: '<S87>/whl_yaw_off_control'
     */
    /* Embedded MATLAB Function 'make_whl_yaw_A_Q_R/R/adapt_R_matrix': '<S94>:1'
     */
    /*  build the R matrix */
    /* '<S94>:1:3' */
    for (i = 0; i < 4; i++) {
        rtb_R_out_p[i] = ved__wye_yaw_R_p[(i)];
    }

    /* '<S94>:1:4' */
    rtb_R_yaw_valid = TRUE;
    if ((((int32_T)rtb_R_onoff_front) == 0) ||
        (((int32_T)rtb_R_onoff_rear) == 0)) {
        /* '<S94>:1:6' */
        /* '<S94>:1:7' */
        for (i = 0; i < 4; i++) {
            rtb_R_out_p[i] = ved__wye_yaw_R_p[(i)] * 10000.0F;
        }

        /* '<S94>:1:8' */
        rtb_R_yaw_valid = FALSE;
    }

    /*  if sum variance of ratio front is above  */
    if (rtb_VectorConcatenate_idx_0 > ved__wye_R_control_p) {
        /* '<S94>:1:12' */
        /* '<S94>:1:13' */
        rtb_R_out_p[0] = rtb_R_out_p[0] * 100000.0F;
    }

    /*  if sum variance of ratio rear is above  */
    if (rtb_R_rear > ved__wye_R_control_p) {
        /* '<S94>:1:17' */
        /* '<S94>:1:18' */
        rtb_R_out_p[3] = rtb_R_out_p[3] * 100000.0F;
    }

    /*  if both sum variance of ratio front is above  */
    if ((rtb_VectorConcatenate_idx_0 > ved__wye_R_control_p) &&
        (rtb_R_rear > ved__wye_R_control_p)) {
        /* '<S94>:1:22' */
        /* '<S94>:1:23' */
        for (i = 0; i < 4; i++) {
            rtb_R_out_p[i] = ved__wye_yaw_R_p[(i)] * 10000.0F;
        }

        /* '<S94>:1:24' */
        rtb_R_yaw_valid = FALSE;
    }

    /* '<S94>:1:28' */
    rtb_TP_yaw_off_control_out = 0;

    /* '<S94>:1:29' */
    rtb_Reset = 0U;

    /* '<S94>:1:30' */
    rtb_TP_init = 0;

    /*  if the deviation of the front or rear axis offset is above 1.0 do not
     * use the */
    /*  wheel yaw rate */
    if ((((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__whs_offset.offset_ratio_front_dev >=
          1.0F) ||
         ((*ved__wye_U_VED_InternalData_in)
              .ved__offsets_in.ved__whs_offset.offset_ratio_rear_dev >=
          1.0F)) ||
        (((int32_T)rtb_R_yaw_valid) == 0)) {
        /* '<S94>:1:34' */
        /* '<S94>:1:36' */
        rtb_TP_yaw_off_control_out = 1;

        /* '<S94>:1:37' */
        rtb_Reset = 1U;

        /* '<S94>:1:38' */
        rtb_TP_init = 1;
    }

    /*  if the TP filtered yaw off control is below 20 % Use the wheel yaw rate
     */
    if (ved__wye_DWork->whl_yaw_off_control_DSTATE < 0.2F) {
        /* '<S94>:1:42' */
        /* '<S94>:1:43' */
        rtb_wye_yaw_off_control = 0U;
    } else {
        /* '<S94>:1:45' */
        rtb_wye_yaw_off_control = 1U;
    }

    /* BusSelector: '<S10>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CycleTime_g = (*ved__wye_U_VED_InputData).Frame.CycleTime;

    /* Outputs for atomic SubSystem: '<S10>/Time2Sec' */
    ved__wye_Time2Sec(rtb_CycleTime_g, &ved__wye_B->Time2Sec_d);

    /* end of Outputs for SubSystem: '<S10>/Time2Sec' */

    /* UnitDelay: '<S90>/T2' */
    WldFactFront = ved__wye_DWork->T2_DSTATE;

    /* UnitDelay: '<S90>/T1' */
    rtb_VectorConcatenate_idx_0 = ved__wye_DWork->T1_DSTATE;

    /* UnitDelay: '<S90>/T0' */
    vx_quad = ved__wye_DWork->T0_DSTATE;

    /* Product: '<S90>/Divide' incorporates:
     *  Constant: '<S90>/filter_length'
     *  Sum: '<S90>/Add'
     *  UnitDelay: '<S90>/T3'
     */
    rtb_Yk1 = (((ved__wye_DWork->T3_DSTATE + ved__wye_DWork->T2_DSTATE) +
                ved__wye_DWork->T1_DSTATE) +
               ved__wye_DWork->T0_DSTATE) *
              0.25F;

    /* Embedded MATLAB: '<S10>/q_gain_delta' incorporates:
     *  UnitDelay: '<S10>/diff_raw_whl_est_whl_yaw'
     *  UnitDelay: '<S10>/last_est_whl_yaw_rate'
     */
    /* Embedded MATLAB Function 'make_whl_yaw_A_Q_R/q_gain_delta': '<S91>:1' */
    /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition below
     * is rewritten with multiple if-elseif expressions. */
    /*  calculate the diff between estimated and real value */
    if (((real32_T)fabs(rtb_Yk1)) >= 1.0E-7F) {
        /* '<S91>:1:5' */
        if (((real32_T)fabs(ved__wye_DWork->last_est_whl_yaw_rate_DSTATE)) >=
            1.0E-7F) {
            /* '<S91>:1:6' */
            /* '<S91>:1:7' */
            kf_quad = rtb_Yk1 - ved__wye_DWork->last_est_whl_yaw_rate_DSTATE;
        } else {
            /* '<S91>:1:9' */
            kf_quad = ved__wye_DWork->diff_raw_whl_est_whl_yaw_DSTATE;
        }
    } else {
        /* '<S91>:1:12' */
        kf_quad = ved__wye_DWork->diff_raw_whl_est_whl_yaw_DSTATE;
    }

    /* Embedded MATLAB: '<S10>/get_gain_bias' */
    /* Embedded MATLAB Function 'make_whl_yaw_A_Q_R/get_gain_bias': '<S89>:1' */
    /*  extract the gain control parameters */
    /* '<S89>:1:3' */
    /* '<S89>:1:4' */

    /* Sum: '<S10>/Sum1' incorporates:
     *  Abs: '<S10>/Abs1'
     *  Product: '<S10>/Product'
     */
    rtb_Yk1 = (((real32_T)fabs(kf_quad)) * ved__wye_yaw_Q_gain_p[0]) +
              ved__wye_yaw_Q_gain_p[1];

    rtb_hx[0] =
        (ved__wye_B->Time2Sec_d.Time2Sec * ved__wye_B->Time2Sec_d.Time2Sec) *
        0.5F;
    rtb_hx[1] = ved__wye_B->Time2Sec_d.Time2Sec;

    /* Embedded MATLAB: '<S86>/makeQ' incorporates:
     *  Constant: '<S86>/sigma_model'
     */
    /* Embedded MATLAB Function 'make_whl_yaw_A_Q_R/Q/makeQ': '<S93>:1' */
    /*  calculates the model noise matrix Q */
    /*  and the noise for the drift */
    /* '<S93>:1:4' */
    sigma_model_gain[0] = ved__wye_yaw_Q_sigmas_p[0] * rtb_Yk1;
    sigma_model_gain[1] = ved__wye_yaw_Q_sigmas_p[1] * rtb_Yk1;

    /* '<S93>:1:5' */

    /* Embedded MATLAB: '<S85>/make_A_matrix' */
    /* Embedded MATLAB Function 'make_whl_yaw_A_Q_R/A/make_A_matrix': '<S92>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S92>:1:5' */
    rtb_A[0] = 1.0F;
    rtb_A[2] = ved__wye_B->Time2Sec_d.Time2Sec;
    for (i = 0; i < 2; i++) {
        rtb_A[1 + (i << 1)] = (real32_T)i;
    }

    /* Sum: '<S3>/APA_Q' incorporates:
     *  Math: '<S3>/At'
     *  Product: '<S3>/APAt'
     *  UnitDelay: '<S3>/P_delay_whl_yaw'
     */
    for (i = 0; i < 2; i++) {
        rtb_x_post_j[i] = rtb_hx[i] * sigma_model_gain[i];
        sigma_model_gain_0[i] = sigma_model_gain[i] * rtb_hx[i];
    }

    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_A_0[i + (bitpos << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_A_0[i + (bitpos << 1)] =
                    (rtb_A[(i_0 << 1) + i] *
                     ved__wye_DWork
                         ->P_delay_whl_yaw_DSTATE[(bitpos << 1) + i_0]) +
                    rtb_A_0[(bitpos << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_TmpSignalConversionAtSFun_0[bitpos + (i << 1)] =
                rtb_x_post_j[bitpos] * sigma_model_gain_0[i];
        }
    }

    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_A_1[i + (bitpos << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_A_1[i + (bitpos << 1)] =
                    (rtb_A_0[(i_0 << 1) + i] * rtb_A[(i_0 << 1) + bitpos]) +
                    rtb_A_1[(bitpos << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_P_pred_nb[bitpos + (i << 1)] =
                rtb_TmpSignalConversionAtSFun_0[(i << 1) + bitpos] +
                rtb_A_1[(i << 1) + bitpos];
        }
    }

    /* Sum: '<S3>/HPH_R' incorporates:
     *  Constant: '<S3>/H_const'
     *  Product: '<S3>/HPHt'
     */
    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_TmpSignalConversionAtSFun_0[i + (bitpos << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_TmpSignalConversionAtSFun_0[i + (bitpos << 1)] =
                    (ved__wye_ConstP.H_const_Value_b[(i_0 << 1) + i] *
                     rtb_P_pred_nb[(bitpos << 1) + i_0]) +
                    rtb_TmpSignalConversionAtSFun_0[(bitpos << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_VectorConcatenate_idx = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_VectorConcatenate_idx +=
                    rtb_TmpSignalConversionAtSFun_0[(i_0 << 1) + i] *
                    ved__wye_ConstB.Ht_a[(bitpos << 1) + i_0];
            }

            rtb_HPHt_R_m[i + (bitpos << 1)] =
                rtb_R_out_p[(bitpos << 1) + i] + rtb_VectorConcatenate_idx;
        }
    }

    /* Product: '<S3>/P_pred*Ht' */
    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_P_predHt_h[i + (bitpos << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_P_predHt_h[i + (bitpos << 1)] =
                    (rtb_P_pred_nb[(i_0 << 1) + i] *
                     ved__wye_ConstB.Ht_a[(bitpos << 1) + i_0]) +
                    rtb_P_predHt_h[(bitpos << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S23>/calculate determinant' */
    ved__wye_calculatedeterminant(rtb_HPHt_R_m,
                                  &ved__wye_B->sf_calculatedeterminant_b);

    /* If: '<S23>/If' incorporates:
     *  ActionPort: '<S27>/Action Port'
     *  ActionPort: '<S28>/Action Port'
     *  SubSystem: '<S23>/calculate the gain'
     *  SubSystem: '<S23>/set gain to default value'
     */
    if (ved__wye_B->sf_calculatedeterminant_b.y > 1.0E-16F) {
        /* Product: '<S27>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {2, 2, 2, 2, 2, 2};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5_h[0], rtb_HPHt_R_m,
                            &ved__wye_DWork->PHt_HPHt_R_DWORK4_i[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1_l[0],
                            &PHt_HPHt_R_DWORK2_e[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3_c[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K_n, rtb_P_predHt_h, &PHt_HPHt_R_DWORK5_h[0],
                             &dims[3]);
        }
    } else {
        /* Constant: '<S28>/Constant' */
        for (i = 0; i < 4; i++) {
            rtb_K_n[i] = ved__wye_ConstP.Constant_Value_d[(i)];
        }
    }

    /* Embedded MATLAB: '<S3>/Reset_P_pred' */
    /* Embedded MATLAB Function 'KF_whl_yaw/Reset_P_pred': '<S24>:1' */
    if (((real32_T)fabs(ved__wye_B->sf_calculatedeterminant_b.y)) <= 1.0E-16F) {
        /* '<S24>:1:4' */
        /* '<S24>:1:6' */
        for (i = 0; i < 4; i++) {
            rtb_P_pred_nb[i] = 0.0F;
        }
    } else {
        /* '<S24>:1:10' */
    }

    /* Product: '<S3>/P_pred_(1_KH)' incorporates:
     *  Constant: '<S3>/H_const'
     *  Constant: '<S3>/eye'
     *  Product: '<S3>/K*H'
     *  Sum: '<S3>/1_KH'
     */
    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_VectorConcatenate_idx = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_VectorConcatenate_idx +=
                    rtb_K_n[(i_0 << 1) + i] *
                    ved__wye_ConstP.H_const_Value_b[(bitpos << 1) + i_0];
            }

            rtb_TmpSignalConversionAtSFun_0[i + (bitpos << 1)] =
                ved__wye_ConstP.eye_Value[(bitpos << 1) + i] -
                rtb_VectorConcatenate_idx;
        }
    }

    for (i = 0; i < 2; i++) {
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_R_out_p[i + (bitpos << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_R_out_p[i + (bitpos << 1)] =
                    (rtb_TmpSignalConversionAtSFun_0[(i_0 << 1) + i] *
                     rtb_P_pred_nb[(bitpos << 1) + i_0]) +
                    rtb_R_out_p[(bitpos << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/correct_variance1' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'correct_variance1': '<S6>:1' */
    /*  the output variance for this module is raised at low speed */
    /*  if input values are not valid set output variance to high value */
    /*  if the velocity is in a low speed range eg. between 1 and 10 m/s  */
    /*  the variance is linear increasing to the bottom limit with the velocity
     */
    /*  below this bottom limit the variance is constant */
    /*  if the velocity if above the upper limit nothing is done */
    if (rtb_wye_yaw_off_control == 0) {
        /* '<S6>:1:8' */
        if (((*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo >
             ved__wye_yaw_P_correct_p[0]) &&
            ((*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo <
             ved__wye_yaw_P_correct_p[1])) {
            /* '<S6>:1:9' */
            /* '<S6>:1:10' */
            rtb_VectorConcatenate_idx =
                ((*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                 ved__wye_yaw_P_correct_p[0]) *
                ved__wye_yaw_P_correct_p[2];
            for (i = 0; i < 2; i++) {
                rtb_hx[i] = (rtb_R_out_p[3 * i] + (ved__wye_yaw_P_correct_p[3] -
                                                   rtb_VectorConcatenate_idx)) +
                            ved__wye_yaw_P_correct_p[4];
            }
        } else if ((*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo <=
                   ved__wye_yaw_P_correct_p[0]) {
            /* '<S6>:1:11' */
            /* '<S6>:1:12' */
            for (i = 0; i < 2; i++) {
                rtb_hx[i] = (rtb_R_out_p[3 * i] + ved__wye_yaw_P_correct_p[3]) +
                            ved__wye_yaw_P_correct_p[4];
            }
        } else {
            /* '<S6>:1:14' */
            for (i = 0; i < 2; i++) {
                rtb_hx[i] = rtb_R_out_p[3 * i] + ved__wye_yaw_P_correct_p[4];
            }
        }
    } else {
        /* '<S6>:1:17' */
        for (i = 0; i < 2; i++) {
            rtb_hx[i] = ved__wye_yaw_P_correct_p[3];
        }
    }

    /* Product: '<S3>/Ax' incorporates:
     *  UnitDelay: '<S3>/x_delay_whl_yaw'
     */
    for (i = 0; i < 2; i++) {
        rtb_TmpSignalConversionAtSFun_c[i] = 0.0F;
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_TmpSignalConversionAtSFun_c[i] =
                (rtb_A[(bitpos << 1) + i] *
                 ved__wye_DWork->x_delay_whl_yaw_DSTATE[(bitpos)]) +
                rtb_TmpSignalConversionAtSFun_c[i];
        }
    }

    /* Embedded MATLAB: '<S3>/Reset_x_pred' incorporates:
     *  UnitDelay: '<S3>/x_delay_whl_yaw'
     */
    /* Embedded MATLAB Function 'KF_whl_yaw/Reset_x_pred': '<S25>:1' */
    if (((real32_T)fabs(ved__wye_B->sf_calculatedeterminant_b.y)) <= 1.0E-16F) {
        /* '<S25>:1:4' */
        /* '<S25>:1:6' */
        sigma_model_gain[0] = ved__wye_DWork->x_delay_whl_yaw_DSTATE[0];
        sigma_model_gain[1] = 0.0F;
    } else {
        /* '<S25>:1:10' */
        for (i = 0; i < 2; i++) {
            sigma_model_gain[i] = rtb_TmpSignalConversionAtSFun_c[i];
        }
    }

    /* Sum: '<S3>/x_Knu' incorporates:
     *  Constant: '<S3>/H_const'
     *  Product: '<S3>/Knu'
     *  Product: '<S3>/Product'
     *  Sum: '<S3>/z_Hx'
     */
    rtb_x_post_j[0] = rtb_frWithWldEst_wye;
    rtb_x_post_j[1] = rtb_reWithWldEst_wye;
    for (i = 0; i < 2; i++) {
        sigma_model_gain_0[i] = 0.0F;
        for (bitpos = 0; bitpos < 2; bitpos++) {
            sigma_model_gain_0[i] =
                (ved__wye_ConstP.H_const_Value_b[(bitpos << 1) + i] *
                 rtb_TmpSignalConversionAtSFun_c[bitpos]) +
                sigma_model_gain_0[i];
        }

        rtb_frWithWldEst_wye_0[i] = rtb_x_post_j[i] - sigma_model_gain_0[i];
    }

    for (i = 0; i < 2; i++) {
        rtb_VectorConcatenate_idx = 0.0F;
        for (bitpos = 0; bitpos < 2; bitpos++) {
            rtb_VectorConcatenate_idx +=
                rtb_K_n[(bitpos << 1) + i] * rtb_frWithWldEst_wye_0[bitpos];
        }

        rtb_x_post_j[i] = sigma_model_gain[i] + rtb_VectorConcatenate_idx;
    }

    /* Embedded MATLAB: '<Root>/correct_yaw_rate' */
    /* Embedded MATLAB Function 'correct_yaw_rate': '<S7>:1' */
    /*  if input values are not valid set output yaw rate to 0 */
    if (rtb_wye_yaw_off_control == 0) {
        /* '<S7>:1:3' */
        /* '<S7>:1:4' */
        rtb_Yk1 = rtb_x_post_j[0];
    } else {
        /* '<S7>:1:6' */
        rtb_Yk1 = 0.0F;
    }

    /* BusAssignment: '<Root>/Bus Assignment' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    (*ved__wye_Y_VED_InternalData_out) = (*ved__wye_U_VED_InternalData_in);
    (*ved__wye_Y_VED_InternalData_out)
        .ved__wye_out.front_whl_yaw_rate_filt_wld = rtb_frfiltWithWldEst;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.diff_whl_yaw_front_rear =
        rtb_Abs3;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.rear_whl_yaw_rate_filt_wld =
        rtb_refiltWithWldEst;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.front_whl_yaw_rate_filt =
        frfilt;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.rear_whl_yaw_rate_filt =
        refilt;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.est_whl_load_dep_front =
        rtb_x_out[2];
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.wld_control = rtb_Rroff;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.gier_yaw_rate_offset_var =
        rtb_x_pred[2];
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.dyn_yaw_off_control =
        rtb_init;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.raw_est_yaw_offset =
        rtb_x_post_h[2];
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.gier_yaw_rate_offset =
        rtb_x_out_i;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.whl_yaw_rate_var =
        rtb_hx[0];
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.whl_yaw_rate = rtb_Yk1;
    (*ved__wye_Y_VED_InternalData_out).ved__wye_out.r_On_Off_control =
        rtb_wye_yaw_off_control;

    /* BusSelector: '<S31>/Bus Selector2' incorporates:
     *  Inport: '<Root>/VED_NvData_in'
     */
    rtb_State_n = (*ved__wye_U_VED_NvData_in).Read.State;

    /* Embedded MATLAB: '<S58>/Get_NVM_IO_State' */
    ved__wye_Get_NVM_IO_State(((uint32_T)VED_NVM_POS_WLD), rtb_State_n, 3U,
                              &ved__wye_B->sf_Get_NVM_IO_State_k);

    /* BusSelector: '<S31>/Bus Selector3' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_CycleTime_c = (*ved__wye_U_VED_InputData).Frame.CycleTime;

    /* Outputs for atomic SubSystem: '<S31>/Time2Sec' */
    ved__wye_Time2Sec(rtb_CycleTime_c, &ved__wye_B->Time2Sec_n);

    /* end of Outputs for SubSystem: '<S31>/Time2Sec' */

    /* Switch: '<S55>/Init' incorporates:
     *  Constant: '<S55>/Initial Condition'
     *  Logic: '<S55>/FixPt Logical Operator'
     *  UnitDelay: '<S31>/trav_dist_reset'
     *  UnitDelay: '<S55>/FixPt Unit Delay1'
     *  UnitDelay: '<S55>/FixPt Unit Delay2'
     */
    if (ved__wye_DWork->trav_dist_reset_DSTATE ||
        (ved__wye_DWork->FixPtUnitDelay2_DSTATE != 0)) {
        rtb_Init = 0.0F;
    } else {
        rtb_Init = ved__wye_DWork->FixPtUnitDelay1_DSTATE;
    }

    /* Sum: '<S31>/add_trav_dist' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  Product: '<S31>/inc_dist'
     */
    ved__wye_DWork->FixPtUnitDelay1_DSTATE =
        (ved__wye_B->Time2Sec_n.Time2Sec *
         (*ved__wye_U_VED_InternalData_in).ved__ve_out.veh_velo) +
        rtb_Init;

    /* Embedded MATLAB: '<S31>/check_with_read_values' incorporates:
     *  Inport: '<Root>/VED_NvData_in'
     */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/check_wld_for_nv_save/check_with_read_values':
     * '<S54>:1' */
    /* '<S54>:1:2' */
    rtb_R_yaw_valid = FALSE;

    /*  if wheel load dependency is read from the NVM, check with currently
     * calculated values */
    if (ved__wye_B->sf_Get_NVM_IO_State_k.state ==
        ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S54>:1:4' */
        /*  if the difference of the wheel load dependancy between the last
         * saved NVM WLD and the currently estimated WLD is above 0.05   */
        /*  and the traveled distance is above 30000.0 m (30 km), store the new
         * value */
        if ((((real32_T)fabs(rtb_x_out[2] -
                             (*ved__wye_U_VED_NvData_in).Read.Wld.Wld_front)) >=
             0.05F) &&
            (ved__wye_DWork->FixPtUnitDelay1_DSTATE > 30000.0F)) {
            /* '<S54>:1:7' */
            /* '<S54>:1:8' */
            rtb_Yk1 = rtb_x_out[2];

            /* '<S54>:1:9' */
            rtb_R_yaw_valid = TRUE;

            /* '<S54>:1:10' */
            rtb_state = ((uint32_T)VED_IO_STATE_VALID);
        } else {
            /* '<S54>:1:12' */
            rtb_Yk1 = (*ved__wye_U_VED_NvData_in).Read.Wld.Wld_front;

            /* '<S54>:1:13' */
            rtb_state = ((uint32_T)VED_IO_STATE_INVALID);
        }
    } else {
        /* '<S54>:1:16' */
        rtb_Yk1 = (*ved__wye_U_VED_NvData_in).Read.Wld.Wld_front;

        /* '<S54>:1:17' */
        rtb_state = ((uint32_T)VED_IO_STATE_INVALID);
    }

    /* Embedded MATLAB: '<S56>/Get_IO_State' incorporates:
     *  Constant: '<S51>/VED__SIN_POS'
     *  Constant: '<S56>/VED_IO_STATE_BITMASK'
     */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/check_wld_for_nv_save/Get_IO_State3/GetIOState/Get_IO_State':
     * '<S57>:1' */
    /*  set the value at pos in the state array */
    /* '<S57>:1:4' */
    c = ((uint32_T)VED_NVM_POS_WLD);
    if (((uint32_T)VED_NVM_POS_WLD) > 2147483647U) {
        c = 2147483647U;
    }

    i = (int32_T)c;
    bitpos = i - ((i >> 5) << 5);

    /* '<S57>:1:5' */
    /* '<S57>:1:6' */
    c = 0U;
    if (bitpos < 0) {
        i = -bitpos;
        if (i > 255) {
            i = 255;
        }

        rtb_init = (uint8_T)i;
        if (rtb_init < 32) {
            c = (3U >> ((uint32_T)rtb_init));
        }
    } else {
        i = bitpos;
        if (bitpos <= 0) {
            i = 0;
        } else {
            if (bitpos > 255) {
                i = 255;
            }
        }

        rtb_init = (uint8_T)i;
        if (rtb_init < 32) {
            c = (3U << ((uint32_T)rtb_init));
        }
    }

    /* '<S57>:1:7' */
    /* '<S57>:1:8' */

    /* BusAssignment: '<S31>/Bus Assignment1' incorporates:
     *  Inport: '<Root>/VED_NvData_in'
     */
    (*ved__wye_Y_VED_NVData_out) = (*ved__wye_U_VED_NvData_in);
    (*ved__wye_Y_VED_NVData_out).Write.State =
        ((c ^ MAX_uint32_T) & (*ved__wye_U_VED_NvData_in).Write.State) |
        (rtb_state << bitpos);
    (*ved__wye_Y_VED_NVData_out).Write.Wld.Wld_front = rtb_Yk1;

    /* Embedded MATLAB: '<S29>/Reset_P_pred' */
    /* Embedded MATLAB Function
     * 'estimate_wheel_load_dep_front/EKF/Reset_P_pred': '<S36>:1' */
    if (((real32_T)fabs(ved__wye_B->sf_calculatedeterminant.y)) <= 1.0E-16F) {
        /* '<S36>:1:4' */
        /* '<S36>:1:6' */
        for (i = 0; i < 9; i++) {
            rtb_P_pred[i] = 0.0F;
        }
    } else {
        /* '<S36>:1:10' */
    }

    /* Switch: '<S55>/Reset' incorporates:
     *  Constant: '<S55>/Initial Condition'
     *  UnitDelay: '<S31>/trav_dist_reset'
     *  Update for UnitDelay: '<S55>/FixPt Unit Delay1'
     */
    if (ved__wye_DWork->trav_dist_reset_DSTATE) {
        ved__wye_DWork->FixPtUnitDelay1_DSTATE = 0.0F;
    }

    /* Switch: '<S96>/Init' incorporates:
     *  Logic: '<S96>/FixPt Logical Operator'
     *  UnitDelay: '<S96>/FixPt Unit Delay1'
     *  UnitDelay: '<S96>/FixPt Unit Delay2'
     */
    if ((rtb_Reset != 0) || (ved__wye_DWork->FixPtUnitDelay2_DSTATE_h != 0)) {
        rtb_Init = (real32_T)rtb_TP_init;
    } else {
        rtb_Init = ved__wye_DWork->FixPtUnitDelay1_DSTATE_d;
    }

    /* Sum: '<S95>/Sum' incorporates:
     *  Gain: '<S95>/Gain'
     *  Sum: '<S95>/Diff'
     */
    ved__wye_DWork->whl_yaw_off_control_DSTATE =
        ((rtb_Init - ((real32_T)rtb_TP_yaw_off_control_out)) * 0.6F) +
        ((real32_T)rtb_TP_yaw_off_control_out);

    /* Switch: '<S96>/Reset' incorporates:
     *  Update for UnitDelay: '<S96>/FixPt Unit Delay1'
     */
    if (rtb_Reset != 0) {
        ved__wye_DWork->FixPtUnitDelay1_DSTATE_d = (real32_T)rtb_TP_init;
    } else {
        ved__wye_DWork->FixPtUnitDelay1_DSTATE_d =
            ved__wye_DWork->whl_yaw_off_control_DSTATE;
    }

    /* Update for UnitDelay: '<S49>/UD' */
    ved__wye_DWork->UD_DSTATE = rtb_Sum;

    /* Update for UnitDelay: '<S50>/UD' */
    ved__wye_DWork->UD_DSTATE_a = rtb_Sum_p;

    /* Update for UnitDelay: '<S8>/last_est_wld' */
    ved__wye_DWork->last_est_wld_DSTATE = rtb_x_out[2];

    /* Update for UnitDelay: '<S29>/x_delay_wld' */
    for (i = 0; i < 3; i++) {
        ved__wye_DWork->x_delay_wld_DSTATE[(i)] = rtb_x_out[i];
    }

    /* Update for UnitDelay: '<S29>/init_nvm_wld_delay' */
    ved__wye_DWork->init_nvm_wld_delay_DSTATE = rtb_init_out;

    /* Update for UnitDelay: '<S70>/UD' */
    ved__wye_DWork->UD_DSTATE_b = rtb_Sum_c;

    /* Update for UnitDelay: '<S71>/hold_value' */
    ved__wye_DWork->hold_value_DSTATE = rtb_Rroff;

    /* Update for UnitDelay: '<S71>/hold_counter' */
    ved__wye_DWork->hold_counter_DSTATE = rtb_counter_out;

    /* Update for UnitDelay: '<S29>/P_delay_wld' incorporates:
     *  Constant: '<S29>/eye'
     *  Product: '<S29>/K*H'
     *  Product: '<S29>/P_pred_(1_KH)'
     *  Sum: '<S29>/1_KH'
     */
    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            rtb_VectorConcatenate_idx = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_VectorConcatenate_idx +=
                    rtb_K[(3 * i_0) + i] * rtb_H[(bitpos << 1) + i_0];
            }

            tmp[i + (3 * bitpos)] = ved__wye_ConstP.pooled6[(3 * bitpos) + i] -
                                    rtb_VectorConcatenate_idx;
        }
    }

    for (i = 0; i < 3; i++) {
        for (bitpos = 0; bitpos < 3; bitpos++) {
            ved__wye_DWork->P_delay_wld_DSTATE[i + (3 * bitpos)] = 0.0F;
            for (i_0 = 0; i_0 < 3; i_0++) {
                ved__wye_DWork->P_delay_wld_DSTATE[i + (3 * bitpos)] =
                    (tmp[(3 * i_0) + i] * rtb_P_pred[(3 * bitpos) + i_0]) +
                    ved__wye_DWork->P_delay_wld_DSTATE[(3 * bitpos) + i];
            }
        }
    }

    /* Update for UnitDelay: '<S2>/P_delay_dyn_off' */
    for (i = 0; i < 9; i++) {
        ved__wye_DWork->P_delay_dyn_off_DSTATE[(i)] = rtb_Q_g[i];
    }

    /* Update for UnitDelay: '<S2>/x_delay_dyn_off' */
    for (i = 0; i < 3; i++) {
        ved__wye_DWork->x_delay_dyn_off_DSTATE[(i)] = rtb_x_post_h[i];
    }

    /* Update for UnitDelay: '<S1>/last_dyn_yaw_offset' */
    ved__wye_DWork->last_dyn_yaw_offset_DSTATE = rtb_x_out_i;

    /* Update for UnitDelay: '<S1>/dyn_yaw_off_overt_count' */
    ved__wye_DWork->dyn_yaw_off_overt_count_DSTATE =
        rtb_VectorConcatenate_idx_1;

    /* Update for UnitDelay: '<S1>/last_dyn_yaw_offset_sign' */
    ved__wye_DWork->last_dyn_yaw_offset_sign_DSTATE = rtb_sign_out;

    /* Update for UnitDelay: '<S10>/diff_raw_whl_est_whl_yaw' */
    ved__wye_DWork->diff_raw_whl_est_whl_yaw_DSTATE = kf_quad;

    /* Update for UnitDelay: '<S90>/T3' */
    ved__wye_DWork->T3_DSTATE = WldFactFront;

    /* Update for UnitDelay: '<S90>/T2' */
    ved__wye_DWork->T2_DSTATE = rtb_VectorConcatenate_idx_0;

    /* Update for UnitDelay: '<S90>/T1' */
    ved__wye_DWork->T1_DSTATE = vx_quad;

    /* Update for UnitDelay: '<S90>/T0' incorporates:
     *  Constant: '<S10>/Constant'
     *  Product: '<S10>/Product1'
     *  Sum: '<S10>/Add'
     */
    ved__wye_DWork->T0_DSTATE =
        (rtb_frWithWldEst_wye + rtb_reWithWldEst_wye) * 0.5F;

    /* Update for UnitDelay: '<S10>/last_est_whl_yaw_rate' */
    ved__wye_DWork->last_est_whl_yaw_rate_DSTATE = rtb_x_post_j[0];

    /* Update for UnitDelay: '<S3>/P_delay_whl_yaw' */
    for (i = 0; i < 4; i++) {
        ved__wye_DWork->P_delay_whl_yaw_DSTATE[(i)] = rtb_R_out_p[i];
    }

    /* Update for UnitDelay: '<S3>/x_delay_whl_yaw' */
    for (i = 0; i < 2; i++) {
        ved__wye_DWork->x_delay_whl_yaw_DSTATE[(i)] = rtb_x_post_j[i];
    }

    /* Update for UnitDelay: '<S31>/trav_dist_reset' */
    ved__wye_DWork->trav_dist_reset_DSTATE = rtb_R_yaw_valid;

    /* Update for UnitDelay: '<S55>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S55>/FixPt Constant'
     */
    ved__wye_DWork->FixPtUnitDelay2_DSTATE = 0U;

    /* Update for UnitDelay: '<S96>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S96>/FixPt Constant'
     */
    ved__wye_DWork->FixPtUnitDelay2_DSTATE_h = 0U;
}

/* Model initialize function */
void ved__wye_initialize(boolean_T firstTime,
                         RT_MODEL_ved__wye *const ved__wye_M,
                         BlockIO_ved__wye *ved__wye_B,
                         D_Work_ved__wye *ved__wye_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__wye_M, (NULL));

    /* block I/O */
    (void)memset(((void *)ved__wye_B), 0, sizeof(BlockIO_ved__wye));

    /* states (dwork) */
    (void)memset((void *)ved__wye_DWork, 0, sizeof(D_Work_ved__wye));

    /* Start for ifaction SubSystem: '<S34>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S43>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &ved__wye_DWork->PHt_HPHt_R_DWORK4[0];
        for (i = 0; i < 4; i++) {
            *dWork++ = 0.0;
        }

        dWork = &ved__wye_DWork->PHt_HPHt_R_DWORK4[0];
        while (dWork < &ved__wye_DWork->PHt_HPHt_R_DWORK4[0] + 4) {
            *dWork = 1;
            dWork += 3;
        }
    }

    /* end of Start for SubSystem: '<S34>/calculate the gain' */

    /* Start for ifaction SubSystem: '<S14>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S21>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &ved__wye_DWork->PHt_HPHt_R_DWORK4_b[0];
        for (i = 0; i < 9; i++) {
            *dWork++ = 0.0;
        }

        dWork = &ved__wye_DWork->PHt_HPHt_R_DWORK4_b[0];
        while (dWork < &ved__wye_DWork->PHt_HPHt_R_DWORK4_b[0] + 9) {
            *dWork = 1;
            dWork += 4;
        }
    }

    /* end of Start for SubSystem: '<S14>/calculate the gain' */

    /* Start for ifaction SubSystem: '<S23>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S27>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &ved__wye_DWork->PHt_HPHt_R_DWORK4_i[0];
        for (i = 0; i < 4; i++) {
            *dWork++ = 0.0;
        }

        dWork = &ved__wye_DWork->PHt_HPHt_R_DWORK4_i[0];
        while (dWork < &ved__wye_DWork->PHt_HPHt_R_DWORK4_i[0] + 4) {
            *dWork = 1;
            dWork += 3;
        }
    }

    /* end of Start for SubSystem: '<S23>/calculate the gain' */
    {
        int32_T i;

        /* InitializeConditions for UnitDelay: '<S49>/UD' */
        ved__wye_DWork->UD_DSTATE = 1.0F;

        /* InitializeConditions for UnitDelay: '<S50>/UD' */
        ved__wye_DWork->UD_DSTATE_a = 1.0F;

        /* InitializeConditions for UnitDelay: '<S8>/last_est_wld' */
        ved__wye_DWork->last_est_wld_DSTATE = 1.5F;

        /* InitializeConditions for UnitDelay: '<S29>/x_delay_wld' */
        for (i = 0; i < 3; i++) {
            ved__wye_DWork->x_delay_wld_DSTATE[(i)] =
                ved__wye_x_init_wld_p[(i)];
        }

        for (i = 0; i < 9; i++) {
            /* InitializeConditions for UnitDelay: '<S29>/P_delay_wld' */
            ved__wye_DWork->P_delay_wld_DSTATE[(i)] =
                ved__wye_P_init_wld_p[(i)];

            /* InitializeConditions for UnitDelay: '<S2>/P_delay_dyn_off' */
            ved__wye_DWork->P_delay_dyn_off_DSTATE[(i)] =
                ved__wye_P_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S2>/x_delay_dyn_off' */
        for (i = 0; i < 3; i++) {
            ved__wye_DWork->x_delay_dyn_off_DSTATE[(i)] =
                ved__wye_x_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S3>/P_delay_whl_yaw' */
        for (i = 0; i < 4; i++) {
            ved__wye_DWork->P_delay_whl_yaw_DSTATE[(i)] =
                ved__wye_yaw_P_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S3>/x_delay_whl_yaw' */
        for (i = 0; i < 2; i++) {
            ved__wye_DWork->x_delay_whl_yaw_DSTATE[(i)] =
                ved__wye_yaw_x_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S55>/FixPt Unit Delay2' */
        ved__wye_DWork->FixPtUnitDelay2_DSTATE = 1U;

        /* InitializeConditions for UnitDelay: '<S96>/FixPt Unit Delay2' */
        ved__wye_DWork->FixPtUnitDelay2_DSTATE_h = 1U;
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */