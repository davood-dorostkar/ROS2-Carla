
#include "ved_consts.h"
#include "ved_ye.h"
#include "ved_ye_private.h"
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
 *    '<S2>/diag_variance'
 *    '<S14>/diag_R'
 */
void ved__ye_diag_variance(const real32_T rtu_u[16],
                           rtB_diag_variance_ved__ye *localB) {
    int32_T i;

    /* Embedded MATLAB: '<S2>/diag_variance' */
    /* Embedded MATLAB Function 'R/diag_variance': '<S21>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S21>:1:5' */
    for (i = 0; i < 4; i++) {
        localB->y[(i)] = rtu_u[(5 * i)];
    }
}

/*
 * Output and update for atomic system:
 *    '<S6>/At'
 *    '<S6>/Ht'
 *    '<S13>/At'
 */
void ved__ye_At(const real32_T rtu_u[4], rtB_At_ved__ye *localB) {
    int32_T i;
    int32_T i_0;

    /* Embedded MATLAB: '<S6>/At' */
    /* Embedded MATLAB Function 'curve_KF/At': '<S24>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S24>:1:5' */
    for (i_0 = 0; i_0 < 2; i_0++) {
        for (i = 0; i < 2; i++) {
            localB->y[i + (i_0 << 1)] = rtu_u[(i << 1) + i_0];
        }
    }
}

/*
 * Output and update for atomic system:
 *    '<S6>/eye'
 *    '<S13>/eye'
 */
void ved__ye_eye(rtB_eye_ved__ye *localB) {
    int32_T i;
    static int8_T tmp[4] = {1, 0, 0, 1};

    /* Embedded MATLAB: '<S6>/eye' */
    /* Embedded MATLAB Function 'curve_KF/eye': '<S29>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S29>:1:5' */
    for (i = 0; i < 4; i++) {
        localB->y[(i)] = (real32_T)tmp[i];
    }
}

/*
 * Output and update for atomic system:
 *    '<Root>/diag_curve_variance'
 *    '<Root>/diag_yaw_variance'
 */
void ved__ye_diag_curve_variance(const real32_T rtu_u[4],
                                 rtB_diag_curve_variance_ved__ye *localB) {
    int32_T i;

    /* Embedded MATLAB: '<Root>/diag_curve_variance' */
    /* Embedded MATLAB Function 'diag_curve_variance': '<S7>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S7>:1:5' */
    for (i = 0; i < 2; i++) {
        localB->y[(i)] = rtu_u[(3 * i)];
    }
}

/*
 * Output and update for atomic system:
 *    '<S34>/make_A_matrix'
 *    '<S49>/make_A_matrix'
 */
void ved__ye_make_A_matrix(real32_T rtu_CycleTime,
                           rtB_make_A_matrix_ved__ye *localB) {
    int32_T i;

    /* Embedded MATLAB: '<S34>/make_A_matrix' */
    /* Embedded MATLAB Function 'make_A_Q_curve/A/make_A_matrix': '<S37>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S37>:1:5' */
    localB->A[0] = 1.0F;
    localB->A[2] = rtu_CycleTime;
    for (i = 0; i < 2; i++) {
        localB->A[1 + (i << 1)] = (real32_T)i;
    }
}

/* Model step function */
void ved__ye_step(BlockIO_ved__ye *ved__ye_B,
                  D_Work_ved__ye *ved__ye_DWork,
                  VED_InputData_t *ved__ye_U_VED_InputData,
                  VED_InternalData_t *ved__ye_U_VED_InternalData_in,
                  VED_InternalData_t *ved__ye_Y_VED_InternalData_out,
                  real32_T ved__ye_Y_K_yaw[8],
                  uint8_T *ved__ye_Y_K_yaw_fault,
                  real32_T ved__ye_Y_K_curve[4],
                  uint8_T *ved__ye_Y_K_curve_fault) {
    /* local block i/o variables */
    real32_T rtb_HPHt_R[16];
    real32_T rtb_HPHt_R_i[16];
    real32_T rtb_P_predHt[8];
    real32_T rtb_K[8];
    real32_T rtb_KH[4];
    real32_T rtb_P_post[4];
    real32_T rtb_Abs_a;
    real32_T rtb_HPHt_R_n[4];
    real32_T rtb_P_predHt_a[4];
    real32_T rtb_K_m[4];
    real32_T rtb_KH_g[4];
    real32_T rtb_P_post_o[4];
    real32_T rtb_H_g[4];
    real32_T rtb_Time2Sec;
    real32_T rtb_R_out_k[16];
    real32_T rtb_Product1_j;

    /* local scratch DWork variables */
    real32_T PHt_HPHt_R_DWORK1[16];
    real32_T PHt_HPHt_R_DWORK3[16];
    real32_T PHt_HPHt_R_DWORK5[16];
    real32_T PHt_HPHt_R_DWORK1_h[4];
    real32_T PHt_HPHt_R_DWORK3_k[4];
    real32_T PHt_HPHt_R_DWORK5_f[4];
    int32_T PHt_HPHt_R_DWORK2[4];
    int32_T PHt_HPHt_R_DWORK2_g[2];
    int32_T Count;
    int32_T j;
    real32_T R_out[16];
    boolean_T isodd;
    int8_T ipiv[4];
    int32_T mmj;
    int32_T jj;
    int32_T jp1j;
    int32_T b_c;
    int32_T c_k;
    int32_T jA;
    real32_T ramp_fact_out;
    real32_T rtb_Yk1;
    real32_T rtb_xpost[2];
    real32_T rtb_x_pred[2];
    real32_T rtb_nu[4];
    real32_T rtb_T2;
    real32_T rtb_T1;
    real32_T rtb_T0;
    real32_T rtb_Divide_f;
    real32_T rtb_ay_yaw_rate_var;
    real32_T rtb_stw_yaw_rate_var;
    real32_T rtb_z_yaw_rate_curve;
    uint8_T rtb_Reset;
    uint8_T rtb_Value_in_uint;
    real32_T rtb_sigma_model_out[2];
    real32_T rtb_gye_usage;
    real32_T rtb_wye_usage;
    real32_T rtb_aye_usage;
    real32_T rtb_sye_usage;
    real32_T rtb_x_post[2];
    uint8_T rtb_K_yaw_fault;
    uint8_T rtb_K_curve_fault;
    real32_T rtb_TmpSignalConversionAtSFun_l[2];
    real32_T rtb_TmpSignalConversionAtSFun_h[4];
    real32_T rtb_Sum_l;
    real32_T rtb_Sum_lq;
    real32_T rtb_Sum_n;
    real32_T rtb_Sum_d;
    real32_T rtb_Sum_di;
    real32_T rtb_TmpSignalConversionAtSFun_f[2];
    uint8_T rtb_hold_counter_out;
    uint8_T rtb_ramp_counter_out;
    real32_T rtb_y_l;
    real32_T rtb_Abs;
    real32_T rtb_APAt[4];
    int8_T rtb_H[8];
    real32_T rtb_P_pred[4];
    int8_T rtb_y[8];
    real32_T rtb_HPHt[16];
    real32_T rtb_R_out_j[16];
    int32_T i;
    real32_T tmp[4];
    real32_T tmp_0[4];
    real32_T rtb_TmpSignalConversionAtSFun_0[2];
    real32_T rtb_H_0[8];
    real32_T rtb_sigma_model_out_0[2];
    int32_T i_0;
    static int8_T tmp_1[8] = {0, 0, 0, 0, 1, 1, 1, 1};

    static int8_T tmp_2[4] = {1, 1, 0, 0};

    /* UnitDelay: '<S15>/UD' */
    rtb_Yk1 = ved__ye_DWork->UD_DSTATE;

    /* Outputs for atomic SubSystem: '<Root>/Time2Sec' */

    /* Product: '<S3>/Time2Sec' incorporates:
     *  Constant: '<S3>/Constant'
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_Time2Sec =
        ((real32_T)(*ved__ye_U_VED_InputData).Frame.CycleTime) / 1000.0F;

    /* end of Outputs for SubSystem: '<Root>/Time2Sec' */

    /* Embedded MATLAB: '<S49>/make_A_matrix' */
    ved__ye_make_A_matrix(rtb_Time2Sec, &ved__ye_B->sf_make_A_matrix);
    for (i = 0; i < 2; i++) {
        /* UnitDelay: '<S13>/x_delay' */
        rtb_xpost[i] = ved__ye_DWork->x_delay_DSTATE[(i)];

        /* Product: '<S13>/Ax' */
        rtb_x_pred[i] = 0.0F;
        for (Count = 0; Count < 2; Count++) {
            rtb_x_pred[i] = (ved__ye_B->sf_make_A_matrix.A[(Count << 1) + i] *
                             ved__ye_DWork->x_delay_DSTATE[(Count)]) +
                            rtb_x_pred[i];
        }
    }

    /* Embedded MATLAB: '<S13>/hx' */
    /* Embedded MATLAB Function 'yaw_KF/hx': '<S71>:1' */
    /*  transformation from state space to measurement space  */
    /* '<S71>:1:3' */
    for (i = 0; i < 8; i++) {
        rtb_H[i] = tmp_1[i];
    }

    /* '<S71>:1:8' */

    /* Embedded MATLAB: '<S12>/build_z_vector' */
    /* Embedded MATLAB Function 'make_z_vektor/build_z_vector': '<S64>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S64>:1:5' */

    /* Sum: '<S13>/z_Hx' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    tmp[0] = (*ved__ye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate;
    tmp[1] = (*ved__ye_U_VED_InternalData_in).ved__wye_out.whl_yaw_rate;
    tmp[2] = (*ved__ye_U_VED_InternalData_in).ved__aye_out.ay_yaw_rate;
    tmp[3] = (*ved__ye_U_VED_InternalData_in).ved__sye_out.stw_yaw_rate;
    for (i = 0; i < 4; i++) {
        tmp_0[i] = 0.0F;
        for (Count = 0; Count < 2; Count++) {
            tmp_0[i] =
                (((real32_T)tmp_1[(Count << 2) + i]) * rtb_x_pred[Count]) +
                tmp_0[i];
        }

        rtb_nu[i] = tmp[i] - tmp_0[i];
    }

    rtb_TmpSignalConversionAtSFun_l[0] = (rtb_Time2Sec * rtb_Time2Sec) * 0.5F;
    rtb_TmpSignalConversionAtSFun_l[1] = rtb_Time2Sec;

    /* Embedded MATLAB: '<S50>/makeQ' incorporates:
     *  Constant: '<S50>/sigma_model'
     */
    /* Embedded MATLAB Function 'make_A_Q_yaw/Q/makeQ': '<S54>:1' */
    /*  calculate the model covariance matrix Q */
    /* '<S54>:1:3' */

    rtb_TmpSignalConversionAtSFun_h[0] =
        (*ved__ye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate;
    rtb_TmpSignalConversionAtSFun_h[1] =
        (*ved__ye_U_VED_InternalData_in).ved__wye_out.whl_yaw_rate;
    rtb_TmpSignalConversionAtSFun_h[2] =
        (*ved__ye_U_VED_InternalData_in).ved__aye_out.ay_yaw_rate;
    rtb_TmpSignalConversionAtSFun_h[3] =
        (*ved__ye_U_VED_InternalData_in).ved__sye_out.stw_yaw_rate;

    /* Embedded MATLAB: '<S10>/mean' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'make_A_Q_yaw/mean': '<S51>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /*  Validate the individual yaw rate using the r_On_Off flags and decide the
     */
    /*  counter for calculating the mean. */
    /* '<S51>:1:8' */
    Count = 0;
    if ((*ved__ye_U_VED_InternalData_in).ved__aye_out.r_On_Off_control != 0) {
        /* '<S51>:1:10' */
        /* '<S51>:1:11' */
        Count = 1;
    }

    if ((*ved__ye_U_VED_InternalData_in).ved__gye_out.r_On_Off_control) {
        /* '<S51>:1:13' */
        /* '<S51>:1:14' */
        Count++;
    }

    if ((*ved__ye_U_VED_InternalData_in).ved__wye_out.r_On_Off_control != 1) {
        /* '<S51>:1:16' */
        /* '<S51>:1:17' */
        Count++;
    }

    if ((*ved__ye_U_VED_InternalData_in).ved__sye_out.r_On_Off_control != 0) {
        /* '<S51>:1:19' */
        /* '<S51>:1:20' */
        Count++;
    }

    if (Count == 0) {
        /* '<S51>:1:23' */
        /* '<S51>:1:24' */
        Count = 1;
    }

    /* '<S51>:1:27' */
    rtb_y_l = (((rtb_TmpSignalConversionAtSFun_h[0] +
                 rtb_TmpSignalConversionAtSFun_h[1]) +
                rtb_TmpSignalConversionAtSFun_h[2]) +
               rtb_TmpSignalConversionAtSFun_h[3]) /
              ((real32_T)Count);

    /* UnitDelay: '<S56>/T2' */
    rtb_T2 = ved__ye_DWork->T2_DSTATE;

    /* UnitDelay: '<S56>/T1' */
    rtb_T1 = ved__ye_DWork->T1_DSTATE;

    /* UnitDelay: '<S56>/T0' */
    rtb_T0 = ved__ye_DWork->T0_DSTATE;

    /* Product: '<S56>/Divide' incorporates:
     *  Constant: '<S56>/filter_length'
     *  Sum: '<S56>/Add'
     *  UnitDelay: '<S56>/T3'
     */
    rtb_Divide_f = (((ved__ye_DWork->T3_DSTATE + ved__ye_DWork->T2_DSTATE) +
                     ved__ye_DWork->T1_DSTATE) +
                    ved__ye_DWork->T0_DSTATE) *
                   0.25F;

    /* Embedded MATLAB: '<S52>/get_gain_bias' incorporates:
     *  Constant: '<S10>/ved__ye_Q_gain'
     */
    /* Embedded MATLAB Function 'make_A_Q_yaw/q_gain/get_gain_bias': '<S55>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S55>:1:5' */
    /* '<S55>:1:6' */
    /* '<S55>:1:7' */

    /* Embedded MATLAB: '<S52>/q_gain_delta' */
    /* Embedded MATLAB Function 'make_A_Q_yaw/q_gain/q_gain_delta': '<S57>:1' */
    /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition below
     * is rewritten with multiple if-elseif expressions. */
    /*  calculate the difference between the mean filtered last yaw rate and  */
    /*  the actual yaw rate */
    if (((real32_T)fabs(rtb_y_l)) >= 1.0E-7F) {
        /* '<S57>:1:6' */
        if (((real32_T)fabs(rtb_Divide_f)) >= 1.0E-7F) {
            /* '<S57>:1:7' */
            /* '<S57>:1:8' */
            rtb_y_l -= rtb_Divide_f;
        } else {
            /* '<S57>:1:10' */
            rtb_y_l = ved__ye_Q_gain_p[2];
        }
    } else {
        /* '<S57>:1:13' */
        rtb_y_l = ved__ye_Q_gain_p[2];
    }

    /* Abs: '<S52>/Abs' */
    rtb_Abs = (real32_T)fabs(rtb_y_l);

    /* Embedded MATLAB: '<S52>/threshold_diff' incorporates:
     *  UnitDelay: '<S52>/Unit Delay'
     */
    /* Embedded MATLAB Function 'make_A_Q_yaw/q_gain/threshold_diff': '<S58>:1'
     */
    /*  check if abs diff is above threshold and limit it to last valid value */
    /* '<S58>:1:3' */
    if (rtb_Abs > ved__ye_Q_gain_p[3]) {
        /* '<S58>:1:4' */
        /* '<S58>:1:5' */
        rtb_Abs = ved__ye_DWork->UnitDelay_DSTATE;
    }

    /* Sum: '<S52>/Sum2' incorporates:
     *  Product: '<S52>/Product'
     */
    rtb_y_l = (rtb_Abs * ved__ye_Q_gain_p[0]) + ved__ye_Q_gain_p[1];

    /* Sum: '<S50>/Sum' incorporates:
     *  Constant: '<S50>/additional'
     *  Product: '<S50>/Product'
     */
    for (i = 0; i < 2; i++) {
        rtb_TmpSignalConversionAtSFun_0[i] =
            rtb_TmpSignalConversionAtSFun_l[i] * ved__ye_Q_sigmas_p[(i)];
        rtb_sigma_model_out_0[i] =
            ved__ye_Q_sigmas_p[(i)] * rtb_TmpSignalConversionAtSFun_l[i];
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_APAt[Count + (i << 1)] =
                rtb_TmpSignalConversionAtSFun_0[Count] *
                rtb_sigma_model_out_0[i];
        }
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_TmpSignalConversionAtSFun_h[Count + (i << 1)] =
                (rtb_APAt[(i << 1) + Count] * rtb_y_l) +
                ved__ye_Q_add_p[(i << 1) + Count];
        }
    }

    /* UnitDelay: '<S13>/P_delay' */
    for (i = 0; i < 4; i++) {
        rtb_APAt[i] = ved__ye_DWork->P_delay_DSTATE[(i)];
    }

    /* Embedded MATLAB: '<S13>/At' */
    ved__ye_At(ved__ye_B->sf_make_A_matrix.A, &ved__ye_B->sf_At);

    /* Sum: '<S13>/APA_Q' incorporates:
     *  Product: '<S13>/APAt'
     */
    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            tmp[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                tmp[i + (Count << 1)] =
                    (ved__ye_B->sf_make_A_matrix.A[(i_0 << 1) + i] *
                     rtb_APAt[(Count << 1) + i_0]) +
                    tmp[(Count << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_y_l = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_y_l += tmp[(i_0 << 1) + i] *
                           ved__ye_B->sf_At.y[(Count << 1) + i_0];
            }

            rtb_P_pred[i + (Count << 1)] =
                rtb_TmpSignalConversionAtSFun_h[(Count << 1) + i] + rtb_y_l;
        }
    }

    /* Embedded MATLAB: '<S13>/Ht' */
    /* Embedded MATLAB Function 'yaw_KF/Ht': '<S67>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S67>:1:5' */
    for (i = 0; i < 4; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_y[Count + (i << 1)] = rtb_H[(Count << 2) + i];
        }
    }

    /* Product: '<S13>/HPHt' */
    for (i = 0; i < 4; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_H_0[i + (Count << 2)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_H_0[i + (Count << 2)] = (((real32_T)rtb_H[(i_0 << 2) + i]) *
                                             rtb_P_pred[(Count << 1) + i_0]) +
                                            rtb_H_0[(Count << 2) + i];
            }
        }
    }

    for (i = 0; i < 4; i++) {
        for (Count = 0; Count < 4; Count++) {
            rtb_HPHt[i + (Count << 2)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_HPHt[i + (Count << 2)] =
                    (rtb_H_0[(i_0 << 2) + i] *
                     ((real32_T)rtb_y[(Count << 1) + i_0])) +
                    rtb_HPHt[(Count << 2) + i];
            }
        }
    }

    /* BusSelector: '<S2>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_y_l = (*ved__ye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate_var;
    rtb_Divide_f =
        (*ved__ye_U_VED_InternalData_in).ved__wye_out.whl_yaw_rate_var;
    rtb_ay_yaw_rate_var =
        (*ved__ye_U_VED_InternalData_in).ved__aye_out.ay_yaw_rate_var;
    rtb_stw_yaw_rate_var =
        (*ved__ye_U_VED_InternalData_in).ved__sye_out.stw_yaw_rate_var;

    rtb_TmpSignalConversionAtSFun_h[0] =
        (*ved__ye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate_var;
    rtb_TmpSignalConversionAtSFun_h[1] =
        (*ved__ye_U_VED_InternalData_in).ved__wye_out.whl_yaw_rate_var;
    rtb_TmpSignalConversionAtSFun_h[2] =
        (*ved__ye_U_VED_InternalData_in).ved__aye_out.ay_yaw_rate_var;
    rtb_TmpSignalConversionAtSFun_h[3] =
        (*ved__ye_U_VED_InternalData_in).ved__sye_out.stw_yaw_rate_var;

    /* Embedded MATLAB: '<S2>/diag_R' */
    /* Embedded MATLAB Function 'R/diag_R': '<S20>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S20>:1:5' */
    memset((void *)(&rtb_R_out_j[0]), (int32_T)0.0F, sizeof(real32_T) << 4U);
    for (j = 0; j < 4; j++) {
        rtb_R_out_j[j + (j << 2)] = rtb_TmpSignalConversionAtSFun_h[j];
    }

    /* Sum: '<S2>/HPHt_R' */
    for (i = 0; i < 16; i++) {
        rtb_HPHt_R[i] = rtb_HPHt[i] + rtb_R_out_j[i];
    }

    /* Embedded MATLAB: '<S2>/diag_variance' */
    ved__ye_diag_variance(rtb_HPHt_R, &ved__ye_B->sf_diag_variance);

    /* Product: '<S2>/PHt_(HPHt_R)1' incorporates:
     *  Product: '<S2>/Product'
     */
    for (i = 0; i < 4; i++) {
        rtb_TmpSignalConversionAtSFun_h[i] =
            (rtb_nu[i] * rtb_nu[i]) / ved__ye_B->sf_diag_variance.y[(i)];
    }

    /* Embedded MATLAB: '<S2>/meas_matrix_adj1' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'R/meas_matrix_adj1': '<S22>:1' */
    /*  check the four mahalanobis distances for threshold */
    /*  and raise then linear with this distance the measument covariance matrix
     */
    /* '<S22>:1:4' */
    memcpy((void *)(&R_out[0]), (void *)(&rtb_R_out_j[0]),
           sizeof(real32_T) << 4U);

    /*  only if the velocity is above a threshold  */
    if ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo >=
        ved__ye_mahala_para_p[0]) {
        /* '<S22>:1:6' */
        /*  adjustment of the gier yaw rate */
        if (rtb_TmpSignalConversionAtSFun_h[0] > ved__ye_mahala_para_p[1]) {
            /* '<S22>:1:8' */
            /* '<S22>:1:9' */
            R_out[0] = (rtb_TmpSignalConversionAtSFun_h[0] *
                        ved__ye_mahala_para_p[5]) +
                       rtb_R_out_j[0];

            /* (nu_2(1)/dist_threshold)-HPHt_in(1,1); */
        }

        /*  adjustment of the wheel speed sensors yaw rate */
        if (rtb_TmpSignalConversionAtSFun_h[1] > ved__ye_mahala_para_p[2]) {
            /* '<S22>:1:12' */
            /* '<S22>:1:13' */
            R_out[5] = (rtb_TmpSignalConversionAtSFun_h[1] *
                        ved__ye_mahala_para_p[6]) +
                       R_out[5];

            /* (nu_2(2)/dist_threshold)-HPHt_in(2,2); */
        }

        /*  adjustment of the lateral acceleration sensor yaw rate */
        if (rtb_TmpSignalConversionAtSFun_h[2] > ved__ye_mahala_para_p[3]) {
            /* '<S22>:1:16' */
            /* '<S22>:1:17' */
            R_out[10] = (rtb_TmpSignalConversionAtSFun_h[2] *
                         ved__ye_mahala_para_p[7]) +
                        R_out[10];

            /* (nu_2(3)/dist_threshold)-HPHt_in(3,3); */
        }

        /*  adjustment of the steering wheel angle yaw rate */
        if (rtb_TmpSignalConversionAtSFun_h[3] > ved__ye_mahala_para_p[4]) {
            /* '<S22>:1:20' */
            /* '<S22>:1:21' */
            R_out[15] = (rtb_TmpSignalConversionAtSFun_h[3] *
                         ved__ye_mahala_para_p[8]) +
                        R_out[15];

            /* (nu_2(4)/dist_threshold)-HPHt_in(4,4); */
        }
    }

    /* '<S22>:1:24' */
    /* '<S22>:1:25' */
    /* '<S22>:1:26' */
    /* '<S22>:1:27' */

    /* Sum: '<S15>/Sum' incorporates:
     *  Gain: '<S15>/Gain'
     *  Sum: '<S15>/Diff'
     */
    rtb_Sum_l = ((rtb_Yk1 - R_out[0]) * 0.96F) + R_out[0];

    /* Sum: '<S16>/Sum' incorporates:
     *  Gain: '<S16>/Gain'
     *  Sum: '<S16>/Diff'
     *  UnitDelay: '<S16>/UD'
     */
    rtb_Sum_lq = ((ved__ye_DWork->UD_DSTATE_m - R_out[5]) * 0.96F) + R_out[5];

    /* Sum: '<S18>/Sum' incorporates:
     *  Gain: '<S18>/Gain'
     *  Sum: '<S18>/Diff'
     *  UnitDelay: '<S18>/UD'
     */
    rtb_Sum_n = ((ved__ye_DWork->UD_DSTATE_h - R_out[10]) * 0.96F) + R_out[10];

    /* Sum: '<S17>/Sum' incorporates:
     *  Gain: '<S17>/Gain'
     *  Sum: '<S17>/Diff'
     *  UnitDelay: '<S17>/UD'
     */
    rtb_Sum_d = ((ved__ye_DWork->UD_DSTATE_j - R_out[15]) * 0.96F) + R_out[15];

    rtb_TmpSignalConversionAtSFun_h[0] = rtb_Sum_l;
    rtb_TmpSignalConversionAtSFun_h[1] = rtb_Sum_lq;
    rtb_TmpSignalConversionAtSFun_h[2] = rtb_Sum_n;
    rtb_TmpSignalConversionAtSFun_h[3] = rtb_Sum_d;

    /* Embedded MATLAB: '<S2>/change_new_R_values' */
    /* Embedded MATLAB Function 'R/change_new_R_values': '<S19>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S19>:1:5' */
    memcpy((void *)(&rtb_R_out_k[0]), (void *)(&rtb_R_out_j[0]),
           sizeof(real32_T) << 4U);

    /* '<S19>:1:6' */
    rtb_R_out_k[0] = rtb_TmpSignalConversionAtSFun_h[0];

    /* '<S19>:1:7' */
    rtb_R_out_k[5] = rtb_TmpSignalConversionAtSFun_h[1];

    /* '<S19>:1:8' */
    rtb_R_out_k[10] = rtb_TmpSignalConversionAtSFun_h[2];

    /* '<S19>:1:9' */
    rtb_R_out_k[15] = rtb_TmpSignalConversionAtSFun_h[3];

    /* Embedded MATLAB: '<S14>/diag_R' */
    ved__ye_diag_variance(rtb_R_out_k, &ved__ye_B->sf_diag_R_i);

    rtb_TmpSignalConversionAtSFun_h[0] = rtb_y_l;
    rtb_TmpSignalConversionAtSFun_h[1] = rtb_Divide_f;
    rtb_TmpSignalConversionAtSFun_h[2] = rtb_ay_yaw_rate_var;
    rtb_TmpSignalConversionAtSFun_h[3] = rtb_stw_yaw_rate_var;

    /* Embedded MATLAB: '<S14>/mean_var' */
    /* Embedded MATLAB Function 'yaw_rate_usage/mean_var': '<S76>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S76>:1:5' */
    rtb_y_l = (((rtb_TmpSignalConversionAtSFun_h[0] +
                 rtb_TmpSignalConversionAtSFun_h[1]) +
                rtb_TmpSignalConversionAtSFun_h[2]) +
               rtb_TmpSignalConversionAtSFun_h[3]) *
              0.25F;

    /* Embedded MATLAB: '<S14>/norm_usage' */
    /* Embedded MATLAB Function 'yaw_rate_usage/norm_usage': '<S77>:1' */
    /*  norm the new by the mahalanobis distance controlled measurement
     * variances  */
    /*  to the mean input variance */
    /* sum = single(R_new(1)+R_new(2)+R_new(3)+R_new(4)); */
    /* '<S77>:1:5' */
    rtb_gye_usage = ved__ye_B->sf_diag_R_i.y[0] / rtb_y_l;

    /* '<S77>:1:6' */
    rtb_wye_usage = ved__ye_B->sf_diag_R_i.y[1] / rtb_y_l;

    /* '<S77>:1:7' */
    rtb_aye_usage = ved__ye_B->sf_diag_R_i.y[2] / rtb_y_l;

    /* '<S77>:1:8' */
    rtb_sye_usage = ved__ye_B->sf_diag_R_i.y[3] / rtb_y_l;

    /* Sum: '<S13>/HPH_R' */
    for (i = 0; i < 16; i++) {
        rtb_HPHt_R_i[i] = rtb_R_out_k[i] + rtb_HPHt[i];
    }

    /* Product: '<S13>/P_pred*Ht' */
    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 4; Count++) {
            rtb_P_predHt[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_P_predHt[i + (Count << 1)] =
                    (rtb_P_pred[(i_0 << 1) + i] *
                     ((real32_T)rtb_y[(Count << 1) + i_0])) +
                    rtb_P_predHt[(Count << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S66>/calculate determinant' */
    /* Embedded MATLAB Function 'yaw_KF/Calculate Kalman gain/calculate
     * determinant': '<S72>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /* '<S72>:1:3' */
    memcpy((void *)(&R_out[0]), (void *)(&rtb_HPHt_R_i[0]),
           sizeof(real32_T) << 4U);
    for (i = 0; i < 4; i++) {
        ipiv[i] = (int8_T)(1 + i);
    }

    for (j = 0; j < 3; j++) {
        mmj = 3 - j;
        jj = (j * 5) + 1;
        jp1j = jj + 1;
        Count = mmj + 1;
        i_0 = 1;
        i = jj;
        rtb_y_l = (real32_T)fabs(R_out[jj - 1]);
        for (c_k = 2; c_k <= Count; c_k++) {
            i++;
            rtb_Divide_f = (real32_T)fabs(R_out[i - 1]);
            if (rtb_Divide_f > rtb_y_l) {
                i_0 = c_k;
                rtb_y_l = rtb_Divide_f;
            }
        }

        Count = i_0 - 1;
        if (((real_T)R_out[(jj + Count) - 1]) != 0.0) {
            if (Count != 0) {
                ipiv[j] = (int8_T)((j + 1) + Count);
                i = 1 + j;
                Count += i;
                for (i_0 = 0; i_0 < 4; i_0++) {
                    rtb_y_l = R_out[i - 1];
                    R_out[i - 1] = R_out[Count - 1];
                    R_out[Count - 1] = rtb_y_l;
                    i += 4;
                    Count += 4;
                }
            }

            Count = (mmj - 1) + jp1j;
            for (i = jp1j; i <= Count; i++) {
                R_out[i - 1] = R_out[i - 1] / R_out[jj - 1];
            }
        }

        b_c = 3 - j;
        jA = jj + 4;
        Count = jj + 4;
        for (i_0 = 1; i_0 <= b_c; i_0++) {
            if (((real_T)R_out[Count - 1]) != 0.0) {
                rtb_y_l = R_out[Count - 1] * -1.0F;
                i = jp1j;
                c_k = mmj + jA;
                for (jj = 1 + jA; jj <= c_k; jj++) {
                    R_out[jj - 1] = (R_out[i - 1] * rtb_y_l) + R_out[jj - 1];
                    i++;
                }
            }

            Count += 4;
            jA += 4;
        }
    }

    rtb_y_l = R_out[0];
    for (Count = 2; Count < 5; Count++) {
        rtb_y_l *= R_out[((Count - 1) << 2) + (Count - 1)];
    }

    isodd = FALSE;
    for (Count = 0; Count < 3; Count++) {
        if (ipiv[Count] > (Count + 1)) {
            isodd = !isodd;
        }
    }

    if (isodd) {
        rtb_y_l = -rtb_y_l;
    }

    /* If: '<S66>/If' incorporates:
     *  ActionPort: '<S73>/Action Port'
     *  ActionPort: '<S74>/Action Port'
     *  SubSystem: '<S66>/calculate the gain'
     *  SubSystem: '<S66>/set gain to default value'
     */
    if (rtb_y_l > 1.0E-16F) {
        /* Product: '<S73>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {4, 4, 4, 2, 4, 4};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5[0], rtb_HPHt_R_i,
                            &ved__ye_DWork->PHt_HPHt_R_DWORK4[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1[0],
                            &PHt_HPHt_R_DWORK2[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K, rtb_P_predHt, &PHt_HPHt_R_DWORK5[0],
                             &dims[3]);
        }

        /* Constant: '<S73>/K_normal' */
        rtb_K_yaw_fault = 0U;
    } else {
        /* Constant: '<S74>/Constant' */
        for (i = 0; i < 8; i++) {
            rtb_K[i] = ved__ye_ConstP.Constant_Value_k1[(i)];
        }

        /* Constant: '<S74>/K_disturbed' */
        rtb_K_yaw_fault = 1U;
    }

    /* Product: '<S13>/K*H' */
    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_KH[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 4; i_0++) {
                rtb_KH[i + (Count << 1)] =
                    (rtb_K[(i_0 << 1) + i] *
                     ((real32_T)rtb_H[(Count << 2) + i_0])) +
                    rtb_KH[(Count << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S13>/eye' */
    ved__ye_eye(&ved__ye_B->sf_eye);

    /* Embedded MATLAB: '<S13>/Reset_P_pred' */
    /* Embedded MATLAB Function 'yaw_KF/Reset_P_pred': '<S68>:1' */
    if (((real32_T)fabs(rtb_y_l)) <= 1.0E-16F) {
        /* '<S68>:1:4' */
        /* '<S68>:1:6' */
        for (i = 0; i < 4; i++) {
            rtb_P_pred[i] = 0.0F;
        }
    } else {
        /* '<S68>:1:10' */
    }

    /* Product: '<S13>/P_pred_(1_KH)' incorporates:
     *  Sum: '<S13>/1_KH'
     */
    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            tmp[Count + (i << 1)] = ved__ye_B->sf_eye.y[(i << 1) + Count] -
                                    rtb_KH[(i << 1) + Count];
        }
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_P_post[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_P_post[i + (Count << 1)] =
                    (tmp[(i_0 << 1) + i] * rtb_P_pred[(Count << 1) + i_0]) +
                    rtb_P_post[(Count << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/diag_yaw_variance' */
    ved__ye_diag_curve_variance(rtb_P_post, &ved__ye_B->sf_diag_yaw_variance);

    /* Embedded MATLAB: '<S13>/Reset_x_pred' */
    /* Embedded MATLAB Function 'yaw_KF/Reset_x_pred': '<S69>:1' */
    if (((real32_T)fabs(rtb_y_l)) <= 1.0E-16F) {
        /* '<S69>:1:4' */
        /* '<S69>:1:6' */
        rtb_x_pred[0] = 0.0F;
        rtb_x_pred[1] = rtb_xpost[1];
    } else {
        /* '<S69>:1:10' */
    }

    /* Sum: '<S13>/x_Knu' incorporates:
     *  Product: '<S13>/Knu'
     */
    for (i = 0; i < 2; i++) {
        rtb_y_l = 0.0F;
        for (Count = 0; Count < 4; Count++) {
            rtb_y_l += rtb_K[(Count << 1) + i] * rtb_nu[Count];
        }

        rtb_x_post[i] = rtb_x_pred[i] + rtb_y_l;
    }

    /* BusSelector: '<S11>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_xpost[1] = (*ved__ye_U_VED_InternalData_in).ved__sye_out.stw_curve;

    /* Abs: '<S11>/Abs' */
    rtb_Abs_a = (real32_T)fabs(ved__ye_B->sf_diag_yaw_variance.y[1]);

    /* Embedded MATLAB: '<S59>/Divide' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  S-Function (ex_sfun_sqrt): '<S11>/sqrt_s_function'
     */
    /* Embedded MATLAB Function 'make_crv_z_vektor_R/Divide/Divide': '<S63>:1'
     */
    /*  This block supports the Embedded MATLAB subset. */
    /*  See the help menu for details.  */
    if ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo <= 1.0F) {
        /* '<S63>:1:5' */
        /* '<S63>:1:6' */
        rtb_y_l = SQRT((real32_T)rtb_Abs_a);
    } else {
        /* '<S63>:1:8' */
        rtb_y_l = SQRT((real32_T)rtb_Abs_a) /
                  (*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo;
    }

    /* Product: '<S11>/sqr' */
    rtb_y_l *= rtb_y_l;

    /* Sum: '<S62>/Sum' incorporates:
     *  Gain: '<S62>/Gain'
     *  Sum: '<S62>/Diff'
     *  UnitDelay: '<S62>/UD'
     */
    rtb_Sum_di = ((ved__ye_DWork->UD_DSTATE_hz - rtb_y_l) * 0.96F) + rtb_y_l;

    /* Embedded MATLAB: '<S11>/make_R' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'make_crv_z_vektor_R/make_R': '<S61>:1' */
    /*  calculate the measurement covariance matrix out of the input variances
     * if the velocity is above a threshold */
    /*  or raise up the yaw rate curve and drop down the driver intended
     * curvatures variances */
    /*  z = [yaw_rate_curve, di_curve] */
    /* '<S61>:1:5' */
    for (i = 0; i < 4; i++) {
        rtb_TmpSignalConversionAtSFun_h[i] = 0.0F;
    }

    /* '<S61>:1:8' */
    rtb_Divide_f = ved__ye_R_curve_p[0] - ved__ye_R_curve_p[2];

    /* '<S61>:1:9' */
    rtb_ay_yaw_rate_var = ((ved__ye_R_curve_p[1] - 1.0F) * rtb_Sum_di) /
                          (rtb_Divide_f * rtb_Divide_f);

    /*  merge yaw rate if steering input is available or the vehicle speed is
     * small */
    if (((*ved__ye_U_VED_InternalData_in).ved__sye_out.stw_curve_var < 99.0F) ||
        ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo < 0.5F)) {
        /* '<S61>:1:12' */
        /*  steering input available, merge curvature */
        if ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo <
            ved__ye_R_curve_p[2]) {
            /* '<S61>:1:14' */
            /*  below a specifed velocity only the Driver intended curve should
             * be used (ackerman curvature) */
            /* '<S61>:1:16' */
            rtb_TmpSignalConversionAtSFun_h[0] =
                ((rtb_ay_yaw_rate_var * rtb_Divide_f) * rtb_Divide_f) +
                rtb_Sum_di;

            /* '<S61>:1:17' */
            rtb_TmpSignalConversionAtSFun_h[3] = rtb_Sum_di;
        } else if ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo <
                   ved__ye_R_curve_p[0]) {
            /* '<S61>:1:18' */
            /*  below a specifed velocity overload the variance by an parabolic
             * function from the yaw rate curve  */
            /*  to the driver indended curvature, so at standstill the ackerman
             * curvature is available   */
            /* '<S61>:1:21' */
            rtb_y_l = ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                       ved__ye_R_curve_p[2]) -
                      rtb_Divide_f;

            /* '<S61>:1:22' */
            rtb_TmpSignalConversionAtSFun_h[0] =
                ((rtb_y_l * rtb_y_l) * rtb_ay_yaw_rate_var) + rtb_Sum_di;

            /* '<S61>:1:23' */
            rtb_y_l = (*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                      ved__ye_R_curve_p[2];

            /* '<S61>:1:24' */
            rtb_TmpSignalConversionAtSFun_h[3] =
                ((rtb_y_l * rtb_y_l) * rtb_ay_yaw_rate_var) + rtb_Sum_di;
        } else {
            /*  otherwise use the yaw rate curve as meas input the driver
             * intended curvature is not used  */
            /*  so the measurement variance of the di curve is high */
            /* '<S61>:1:28' */
            rtb_TmpSignalConversionAtSFun_h[0] = rtb_y_l;
            rtb_TmpSignalConversionAtSFun_h[2] = 0.0F;
            rtb_TmpSignalConversionAtSFun_h[1] = 0.0F;
            rtb_TmpSignalConversionAtSFun_h[3] =
                ((rtb_ay_yaw_rate_var * rtb_Divide_f) * rtb_Divide_f) +
                rtb_Sum_di;
        }
    } else {
        /*  no steering input, use only yaw rate for curvature */
        /* '<S61>:1:32' */
        rtb_TmpSignalConversionAtSFun_h[0] = rtb_y_l;
        rtb_TmpSignalConversionAtSFun_h[2] = 0.0F;
        rtb_TmpSignalConversionAtSFun_h[1] = 0.0F;
        rtb_TmpSignalConversionAtSFun_h[3] =
            ((rtb_ay_yaw_rate_var * rtb_Divide_f) * rtb_Divide_f) + rtb_Sum_di;
    }

    /* Embedded MATLAB: '<S34>/make_A_matrix' */
    ved__ye_make_A_matrix(rtb_Time2Sec, &ved__ye_B->sf_make_A_matrix_l);
    for (i = 0; i < 2; i++) {
        /* UnitDelay: '<S6>/x_delay' */
        rtb_TmpSignalConversionAtSFun_l[i] =
            ved__ye_DWork->x_delay_DSTATE_k[(i)];

        /* Product: '<S6>/Ax' */
        rtb_x_pred[i] = 0.0F;
        for (Count = 0; Count < 2; Count++) {
            rtb_x_pred[i] = (ved__ye_B->sf_make_A_matrix_l.A[(Count << 1) + i] *
                             ved__ye_DWork->x_delay_DSTATE_k[(Count)]) +
                            rtb_x_pred[i];
        }
    }

    /* Embedded MATLAB: '<S6>/hx' */
    /* Embedded MATLAB Function 'curve_KF/hx': '<S30>:1' */
    /*  transformation from the state space to the measurement space  */
    /* '<S30>:1:3' */
    for (i = 0; i < 4; i++) {
        rtb_H_g[i] = (real32_T)tmp_2[i];
    }

    /* '<S30>:1:5' */

    /* Product: '<S35>/Divide' incorporates:
     *  Constant: '<S35>/Constant'
     *  Product: '<S35>/Product1'
     */
    rtb_Divide_f = (rtb_Time2Sec * rtb_Time2Sec) * 0.5F;

    /* Embedded MATLAB: '<S38>/sigma_velocity_gain_controll' incorporates:
     *  Constant: '<S38>/sigma_model'
     *  Constant: '<S38>/sigma_velo_gain'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function
     * 'make_A_Q_curve/Q/gain_sigmas_over_velocity/sigma_velocity_gain_controll':
     * '<S40>:1' */
    /*  if gain controll over velocity is selected */
    if (ved__ye_Q_sigmas_velo_gain_p[0] == 1.0F) {
        /* '<S40>:1:3' */
        /*     if velocity is  below the max gain controled velocity */
        if ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo <=
            ved__ye_Q_sigmas_velo_gain_p[3]) {
            /* '<S40>:1:5' */
            /*  calculate the new sigmas controled by the velocity */
            /* '<S40>:1:7' */
            /* '<S40>:1:8' */
            rtb_y_l = ((((ved__ye_Q_sigmas_velo_gain_p[1] -
                          ved__ye_Q_sigmas_velo_gain_p[2]) /
                         (ved__ye_Q_sigmas_velo_gain_p[3] *
                          ved__ye_Q_sigmas_velo_gain_p[3])) *
                        ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                         ved__ye_Q_sigmas_velo_gain_p[3])) *
                       ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                        ved__ye_Q_sigmas_velo_gain_p[3])) +
                      ved__ye_Q_sigmas_velo_gain_p[2];

            /* '<S40>:1:9' */
            rtb_sigma_model_out[0] = rtb_y_l;
            rtb_sigma_model_out[1] = rtb_y_l;
        } else {
            /*  constant sigma if above the threshold velocity */
            /* '<S40>:1:12' */
            rtb_sigma_model_out[0] = ved__ye_Q_sigmas_velo_gain_p[2];
            rtb_sigma_model_out[1] = ved__ye_Q_sigmas_velo_gain_p[2];
        }
    } else {
        /*  not velocity gain controlled */
        /* '<S40>:1:16' */
        for (i = 0; i < 2; i++) {
            rtb_sigma_model_out[i] = ved__ye_Q_sigmas_curve_p[(i)];
        }
    }

    /* UnitDelay: '<S36>/delay' */
    rtb_y_l = ved__ye_DWork->delay_DSTATE;

    /* Embedded MATLAB: '<S11>/divide_yaw_rate' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  UnitDelay: '<S11>/hold_counter'
     *  UnitDelay: '<S11>/last_yaw_rate_curve'
     *  UnitDelay: '<S11>/ramp_counter'
     *  UnitDelay: '<S11>/ramp_factor'
     */
    /* Embedded MATLAB Function 'make_crv_z_vektor_R/divide_yaw_rate': '<S60>:1'
     */
    /*  calculate the yaw rate curve out of the yaw rate and the vehicle
     * velocity */
    /*  velocity is zero and below use the last yaw rate curve as measurement
     * input */
    if ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo <= 0.015F) {
        /* '<S60>:1:4' */
        /* '<S60>:1:5' */
        rtb_hold_counter_out = ved__ye_DWork->hold_counter_DSTATE;

        /* '<S60>:1:6' */
        rtb_ramp_counter_out = ved__ye_DWork->ramp_counter_DSTATE;

        /* '<S60>:1:7' */
        ramp_fact_out = ved__ye_DWork->ramp_factor_DSTATE;

        /*  calculate the ramp factor only once  */
        if ((ved__ye_DWork->hold_counter_DSTATE == 0) &&
            (ved__ye_DWork->ramp_counter_DSTATE == 0)) {
            /* '<S60>:1:9' */
            /* '<S60>:1:10' */
            ramp_fact_out = ved__ye_DWork->last_yaw_rate_curve_DSTATE /
                            ((real32_T)ved__ye_yaw_curve_ramp_para_p[1]);
        }

        /*  hold the last yaw rate curve a specified time */
        if (ved__ye_DWork->hold_counter_DSTATE <
            ved__ye_yaw_curve_ramp_para_p[0]) {
            /* '<S60>:1:13' */
            /* '<S60>:1:14' */
            rtb_hold_counter_out =
                (uint8_T)(ved__ye_DWork->hold_counter_DSTATE + 1);

            /* '<S60>:1:15' */
            rtb_z_yaw_rate_curve = ved__ye_DWork->last_yaw_rate_curve_DSTATE;
        } else if ((ved__ye_DWork->ramp_counter_DSTATE <
                    ved__ye_yaw_curve_ramp_para_p[1]) &&
                   (ved__ye_DWork->hold_counter_DSTATE ==
                    ved__ye_yaw_curve_ramp_para_p[0])) {
            /* '<S60>:1:16' */
            /*  afterwards ramp out the yaw rate curve         */
            /* '<S60>:1:18' */
            rtb_ramp_counter_out =
                (uint8_T)(ved__ye_DWork->ramp_counter_DSTATE + 1);

            /* '<S60>:1:19' */
            rtb_z_yaw_rate_curve =
                ved__ye_DWork->last_yaw_rate_curve_DSTATE - ramp_fact_out;
        } else {
            /*  if yaw rate is  ramped out set all to zero */
            /* '<S60>:1:22' */
            rtb_ramp_counter_out = ved__ye_yaw_curve_ramp_para_p[1];

            /* '<S60>:1:23' */
            rtb_z_yaw_rate_curve = 0.0F;
        }
    } else {
        /* '<S60>:1:27' */
        rtb_hold_counter_out = 0U;

        /* '<S60>:1:28' */
        rtb_ramp_counter_out = 0U;

        /* '<S60>:1:29' */
        ramp_fact_out = 0.0F;

        /* '<S60>:1:30' */
        rtb_z_yaw_rate_curve =
            rtb_x_post[1] /
            (*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo;
    }

    rtb_xpost[0] = rtb_z_yaw_rate_curve;

    /* Product: '<S36>/Divide' incorporates:
     *  Sum: '<S41>/Diff'
     *  UnitDelay: '<S41>/UD'
     */
    ved__ye_B->Divide =
        (rtb_xpost[1] - ved__ye_DWork->UD_DSTATE_i) / rtb_Time2Sec;

    /* Weighted Moving Average Block: '<S36>/Weighted Moving Average'
     *
     *  Finite Impulse Response Filter
     *
     * Input0  Data Type:  Floating Point real32_T
     * Output0 Data Type:  Floating Point real32_T
     *
     * Parameter: Gain
     *   Data Type:  Floating Point real32_T
     *
     */
    {
        real32_T yTemp1;
        rtb_Product1_j = ved__ye_B->Divide * 0.125F;

        {
            int_T i1;
            real32_T *dw_TapDelayU =
                &ved__ye_DWork->WeightedMovingAverage_TapDelayU[1];
            for (i1 = 0; i1 < 7; i1++) {
                yTemp1 = dw_TapDelayU[((i1)-1)] *
                         ved__ye_ConstP.WeightedMovingAverage[i1 + 1];
                rtb_Product1_j = rtb_Product1_j + yTemp1;
            }
        }
    }

    /* Sum: '<S36>/Add1' */
    rtb_Product1_j = rtb_y_l - rtb_Product1_j;

    /* Abs: '<S36>/Abs' */
    rtb_y_l = (real32_T)fabs(rtb_Product1_j);

    /* Embedded MATLAB: '<S44>/check_reset_condition' incorporates:
     *  UnitDelay: '<S44>/Unit Delay'
     */
    /* Embedded MATLAB Function
     * 'make_A_Q_curve/calculate_Q_gain/tp_filter_peaks1/check_reset_condition':
     * '<S46>:1' */
    if (rtb_y_l > ved__ye_DWork->UnitDelay_DSTATE_j) {
        /* '<S46>:1:3' */
        /* '<S46>:1:4' */
        rtb_Reset = 1U;

        /* '<S46>:1:5' */
        rtb_Yk1 = rtb_y_l;
    } else {
        /* '<S46>:1:7' */
        rtb_Reset = 0U;

        /* '<S46>:1:8' */
        rtb_Yk1 = 0.0F;
    }

    /* Switch: '<S48>/Init' incorporates:
     *  Logic: '<S48>/FixPt Logical Operator'
     *  UnitDelay: '<S48>/FixPt Unit Delay1'
     *  UnitDelay: '<S48>/FixPt Unit Delay2'
     */
    if ((rtb_Reset != 0) || (ved__ye_DWork->FixPtUnitDelay2_DSTATE != 0)) {
        rtb_Product1_j = rtb_Yk1;
    } else {
        rtb_Product1_j = ved__ye_DWork->FixPtUnitDelay1_DSTATE;
    }

    /* Sum: '<S47>/Sum' incorporates:
     *  Gain: '<S47>/Gain'
     *  Sum: '<S47>/Diff'
     */
    rtb_ay_yaw_rate_var = ((rtb_Product1_j - rtb_y_l) * 0.95F) + rtb_y_l;

    /* Embedded MATLAB: '<S36>/threshold_gain' */
    /* Embedded MATLAB Function
     * 'make_A_Q_curve/calculate_Q_gain/threshold_gain': '<S43>:1' */
    if (rtb_ay_yaw_rate_var > 0.002F) {
        /* '<S43>:1:3' */
        /* '<S43>:1:4' */
        rtb_stw_yaw_rate_var = rtb_ay_yaw_rate_var * ved__ye_Q_curve_gain_p;
    } else {
        /* '<S43>:1:6' */
        rtb_stw_yaw_rate_var = 1.0F;

        /* 0.001 */
    }

    rtb_Value_in_uint = (uint8_T)rtb_stw_yaw_rate_var;

    /* Embedded MATLAB Function
     * 'make_A_Q_curve/calculate_Q_gain/hold_last_5_values/hold_last_5_values':
     * '<S45>:1' */
    /*  Hold value 5 cycle if input value is one */
    /*  Added Value_in_uint and Last_value_uint to remove QAC Level 4 warnings
     */
    if ((((rtb_Value_in_uint == 1) &&
          (((uint8_T)ved__ye_DWork->hold_value_DSTATE) != 1)) &&
         (ved__ye_DWork->hold_counter_DSTATE_e == 0)) ||
        ((ved__ye_DWork->hold_counter_DSTATE_e > 0) &&
         (ved__ye_DWork->hold_counter_DSTATE_e <= 5))) {
        /* '<S45>:1:5' */
        if ((ved__ye_DWork->hold_counter_DSTATE_e >= 1) &&
            (rtb_Value_in_uint != 1)) {
            /* '<S45>:1:6' */
            /* '<S45>:1:7' */
            /* '<S45>:1:8' */
            rtb_Value_in_uint = 0U;
        } else {
            /* '<S45>:1:10' */
            rtb_stw_yaw_rate_var = ved__ye_DWork->hold_value_DSTATE;

            /* '<S45>:1:11' */
            rtb_Value_in_uint =
                (uint8_T)(ved__ye_DWork->hold_counter_DSTATE_e + 1);
        }
    } else {
        /* '<S45>:1:14' */
        /* '<S45>:1:15' */
        rtb_Value_in_uint = 0U;
    }

    /* Product: '<S35>/Product' */
    for (i = 0; i < 2; i++) {
        rtb_sigma_model_out[i] = rtb_sigma_model_out[i] * rtb_stw_yaw_rate_var;
    }

    rtb_TmpSignalConversionAtSFun_f[0] = rtb_Divide_f;
    rtb_TmpSignalConversionAtSFun_f[1] = rtb_Time2Sec;

    /* Embedded MATLAB: '<S35>/makeQ' */
    /* Embedded MATLAB Function 'make_A_Q_curve/Q/makeQ': '<S39>:1' */
    /*  calculate the model covariance matrix Q */
    /* '<S39>:1:3' */

    /* UnitDelay: '<S6>/P_delay' */
    for (i = 0; i < 4; i++) {
        rtb_APAt[i] = ved__ye_DWork->P_delay_DSTATE_l[(i)];
    }

    /* Embedded MATLAB: '<S6>/At' */
    ved__ye_At(ved__ye_B->sf_make_A_matrix_l.A, &ved__ye_B->sf_At_i);

    /* Sum: '<S6>/APA_Q' incorporates:
     *  Product: '<S6>/APAt'
     */
    for (i = 0; i < 2; i++) {
        rtb_TmpSignalConversionAtSFun_0[i] =
            rtb_TmpSignalConversionAtSFun_f[i] * rtb_sigma_model_out[i];
        rtb_sigma_model_out_0[i] =
            rtb_sigma_model_out[i] * rtb_TmpSignalConversionAtSFun_f[i];
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            tmp[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                tmp[i + (Count << 1)] =
                    (ved__ye_B->sf_make_A_matrix_l.A[(i_0 << 1) + i] *
                     rtb_APAt[(Count << 1) + i_0]) +
                    tmp[(Count << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_APAt[Count + (i << 1)] =
                rtb_TmpSignalConversionAtSFun_0[Count] *
                rtb_sigma_model_out_0[i];
        }
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            tmp_0[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                tmp_0[i + (Count << 1)] =
                    (tmp[(i_0 << 1) + i] *
                     ved__ye_B->sf_At_i.y[(Count << 1) + i_0]) +
                    tmp_0[(Count << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_P_pred[Count + (i << 1)] =
                rtb_APAt[(i << 1) + Count] + tmp_0[(i << 1) + Count];
        }
    }

    /* Embedded MATLAB: '<S6>/Ht' */
    ved__ye_At(rtb_H_g, &ved__ye_B->sf_Ht_k);

    /* Sum: '<S6>/HPH_R' incorporates:
     *  Product: '<S6>/HPHt'
     */
    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            tmp[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                tmp[i + (Count << 1)] =
                    (rtb_H_g[(i_0 << 1) + i] * rtb_P_pred[(Count << 1) + i_0]) +
                    tmp[(Count << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_y_l = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_y_l += tmp[(i_0 << 1) + i] *
                           ved__ye_B->sf_Ht_k.y[(Count << 1) + i_0];
            }

            rtb_HPHt_R_n[i + (Count << 1)] =
                rtb_TmpSignalConversionAtSFun_h[(Count << 1) + i] + rtb_y_l;
        }
    }

    /* Product: '<S6>/P_pred*Ht' */
    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_P_predHt_a[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_P_predHt_a[i + (Count << 1)] =
                    (rtb_P_pred[(i_0 << 1) + i] *
                     ved__ye_B->sf_Ht_k.y[(Count << 1) + i_0]) +
                    rtb_P_predHt_a[(Count << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S25>/calculate determinant' */
    /* Embedded MATLAB Function 'curve_KF/Calculate Kalman gain/calculate
     * determinant': '<S31>:1' */
    /* '<S31>:1:3' */
    rtb_y_l = (rtb_HPHt_R_n[0] * rtb_HPHt_R_n[3]) -
              (rtb_HPHt_R_n[1] * rtb_HPHt_R_n[2]);

    /* If: '<S25>/If' incorporates:
     *  ActionPort: '<S32>/Action Port'
     *  ActionPort: '<S33>/Action Port'
     *  SubSystem: '<S25>/calculate the gain'
     *  SubSystem: '<S25>/set gain to default value'
     */
    if (rtb_y_l > 1.0E-16F) {
        /* Product: '<S32>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {2, 2, 2, 2, 2, 2};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5_f[0], rtb_HPHt_R_n,
                            &ved__ye_DWork->PHt_HPHt_R_DWORK4_m[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1_h[0],
                            &PHt_HPHt_R_DWORK2_g[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3_k[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K_m, rtb_P_predHt_a, &PHt_HPHt_R_DWORK5_f[0],
                             &dims[3]);
        }

        /* Constant: '<S32>/K_normal' */
        rtb_K_curve_fault = 0U;
    } else {
        /* Constant: '<S33>/Constant' */
        for (i = 0; i < 4; i++) {
            rtb_K_m[i] = ved__ye_ConstP.Constant_Value_k[(i)];
        }

        /* Constant: '<S33>/K_disturbed' */
        rtb_K_curve_fault = 1U;
    }

    /* Product: '<S6>/K*H' */
    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_KH_g[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_KH_g[i + (Count << 1)] =
                    (rtb_K_m[(i_0 << 1) + i] * rtb_H_g[(Count << 1) + i_0]) +
                    rtb_KH_g[(Count << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S6>/eye' */
    ved__ye_eye(&ved__ye_B->sf_eye_j);

    /* Embedded MATLAB: '<S6>/Reset_P_pred' */
    /* Embedded MATLAB Function 'curve_KF/Reset_P_pred': '<S27>:1' */
    if (((real32_T)fabs(rtb_y_l)) <= 1.0E-16F) {
        /* '<S27>:1:4' */
        /* '<S27>:1:6' */
        for (i = 0; i < 4; i++) {
            rtb_P_pred[i] = 0.0F;
        }
    } else {
        /* '<S27>:1:10' */
    }

    /* Product: '<S6>/P_pred_(1_KH)' incorporates:
     *  Sum: '<S6>/1_KH'
     */
    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            tmp[Count + (i << 1)] = ved__ye_B->sf_eye_j.y[(i << 1) + Count] -
                                    rtb_KH_g[(i << 1) + Count];
        }
    }

    for (i = 0; i < 2; i++) {
        for (Count = 0; Count < 2; Count++) {
            rtb_P_post_o[i + (Count << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_P_post_o[i + (Count << 1)] =
                    (tmp[(i_0 << 1) + i] * rtb_P_pred[(Count << 1) + i_0]) +
                    rtb_P_post_o[(Count << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/diag_curve_variance' */
    ved__ye_diag_curve_variance(rtb_P_post_o,
                                &ved__ye_B->sf_diag_curve_variance);

    /* Embedded MATLAB: '<S6>/Reset_x_pred' */
    /* Embedded MATLAB Function 'curve_KF/Reset_x_pred': '<S28>:1' */
    if (!(((real32_T)fabs(rtb_y_l)) <= 1.0E-16F)) {
        /* '<S28>:1:10' */
        for (i = 0; i < 2; i++) {
            rtb_TmpSignalConversionAtSFun_l[i] = rtb_x_pred[i];
        }
    } else {
        /* '<S28>:1:4' */
        /* '<S28>:1:6' */
    }

    /* Sum: '<S6>/x_Knu' incorporates:
     *  Product: '<S6>/Knu'
     *  Sum: '<S6>/z_Hx'
     */
    for (i = 0; i < 2; i++) {
        rtb_y_l = 0.0F;
        for (Count = 0; Count < 2; Count++) {
            rtb_y_l += ((real32_T)tmp_2[(Count << 1) + i]) * rtb_x_pred[Count];
        }

        rtb_TmpSignalConversionAtSFun_0[i] = rtb_xpost[i] - rtb_y_l;
    }

    for (i = 0; i < 2; i++) {
        rtb_y_l = 0.0F;
        for (Count = 0; Count < 2; Count++) {
            rtb_y_l += rtb_K_m[(Count << 1) + i] *
                       rtb_TmpSignalConversionAtSFun_0[Count];
        }

        ved__ye_DWork->x_delay_DSTATE_k[(i)] =
            rtb_TmpSignalConversionAtSFun_l[i] + rtb_y_l;
    }

    /* Product: '<S5>/Product1' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  Product: '<S5>/Product'
     */
    rtb_Product1_j = ((*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo *
                      (*ved__ye_U_VED_InternalData_in).ved__ve_out.veh_velo) *
                     ved__ye_DWork->x_delay_DSTATE_k[0];

    /* BusAssignment: '<Root>/usage_bus_ass1' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    (*ved__ye_Y_VED_InternalData_out) = (*ved__ye_U_VED_InternalData_in);
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_gye_rate_usage =
        rtb_gye_usage;
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_wye_rate_usage =
        rtb_wye_usage;
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_aye_rate_usage =
        rtb_aye_usage;
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_sye_rate_usage =
        rtb_sye_usage;
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_yaw_rate_var =
        ved__ye_B->sf_diag_yaw_variance.y[1];
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_yaw_rate = rtb_x_post[1];
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_merge_curve_var =
        ved__ye_B->sf_diag_curve_variance.y[0];
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_merge_curve_grad_var =
        ved__ye_B->sf_diag_curve_variance.y[1];
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_lat_accel =
        rtb_Product1_j;
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_merge_curve =
        ved__ye_DWork->x_delay_DSTATE_k[0];
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_merge_curve_grad =
        ved__ye_DWork->x_delay_DSTATE_k[1];

    /* Embedded MATLAB: '<S4>/check_stat_and_quality' incorporates:
     *  BusSelector: '<S4>/Bus Selector'
     */
    /* Embedded MATLAB Function 'TuneYawRateVar/check_stat_and_quality':
     * '<S23>:1' */
    /*  if external yaw rate sensor is used */
    /* '<S23>:1:3' */
    rtb_y_l = (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_yaw_rate_var;
    if ((*ved__ye_Y_VED_InternalData_out)
            .ved__offsets_in.ved__yaw_offset.quality == 1.5F) {
        /* '<S23>:1:4' */
        /* '<S23>:1:5' */
        rtb_y_l =
            (((real32_T)fabs((*ved__ye_Y_VED_InternalData_out)
                                 .ved__offsets_in.ved__yaw_offset.offset -
                             (*ved__ye_Y_VED_InternalData_out)
                                 .ved__wye_out.gier_yaw_rate_offset)) *
             ved__yaw_rate_var_tune_p) +
            (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_yaw_rate_var;
    }

    /* BusAssignment: '<S4>/Bus Assignment' incorporates:
     *  BusSelector: '<S4>/Bus Selector'
     */
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_yaw_rate_var_org =
        (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_yaw_rate_var;
    (*ved__ye_Y_VED_InternalData_out).ved__ye_out.veh_yaw_rate_var = rtb_y_l;

    /* Outport: '<Root>/K_yaw' */
    for (i = 0; i < 8; i++) {
        ved__ye_Y_K_yaw[(i)] = rtb_K[i];
    }

    /* Outport: '<Root>/K_yaw_fault' */
    (*ved__ye_Y_K_yaw_fault) = rtb_K_yaw_fault;

    /* Outport: '<Root>/K_curve' */
    for (i = 0; i < 4; i++) {
        ved__ye_Y_K_curve[(i)] = rtb_K_m[i];
    }

    /* Outport: '<Root>/K_curve_fault' */
    (*ved__ye_Y_K_curve_fault) = rtb_K_curve_fault;

    /* Switch: '<S48>/Reset' */
    if (rtb_Reset == 0) {
        rtb_Yk1 = rtb_ay_yaw_rate_var;
    }

    /* Update for UnitDelay: '<S15>/UD' */
    ved__ye_DWork->UD_DSTATE = rtb_Sum_l;

    /* Update for UnitDelay: '<S13>/x_delay' */
    for (i = 0; i < 2; i++) {
        ved__ye_DWork->x_delay_DSTATE[(i)] = rtb_x_post[i];
    }

    /* Update for UnitDelay: '<S52>/Unit Delay' */
    ved__ye_DWork->UnitDelay_DSTATE = rtb_Abs;

    /* Update for UnitDelay: '<S56>/T3' */
    ved__ye_DWork->T3_DSTATE = rtb_T2;

    /* Update for UnitDelay: '<S56>/T2' */
    ved__ye_DWork->T2_DSTATE = rtb_T1;

    /* Update for UnitDelay: '<S56>/T1' */
    ved__ye_DWork->T1_DSTATE = rtb_T0;

    /* Update for UnitDelay: '<S56>/T0' */
    ved__ye_DWork->T0_DSTATE = rtb_x_post[1];

    /* Update for UnitDelay: '<S13>/P_delay' */
    for (i = 0; i < 4; i++) {
        ved__ye_DWork->P_delay_DSTATE[(i)] = rtb_P_post[i];
    }

    /* Update for UnitDelay: '<S16>/UD' */
    ved__ye_DWork->UD_DSTATE_m = rtb_Sum_lq;

    /* Update for UnitDelay: '<S18>/UD' */
    ved__ye_DWork->UD_DSTATE_h = rtb_Sum_n;

    /* Update for UnitDelay: '<S17>/UD' */
    ved__ye_DWork->UD_DSTATE_j = rtb_Sum_d;

    /* Update for UnitDelay: '<S62>/UD' */
    ved__ye_DWork->UD_DSTATE_hz = rtb_Sum_di;

    /* Update for UnitDelay: '<S36>/delay' */
    ved__ye_DWork->delay_DSTATE = ved__ye_DWork->x_delay_DSTATE_k[1];

    /* Update for UnitDelay: '<S11>/hold_counter' */
    ved__ye_DWork->hold_counter_DSTATE = rtb_hold_counter_out;

    /* Update for UnitDelay: '<S11>/ramp_counter' */
    ved__ye_DWork->ramp_counter_DSTATE = rtb_ramp_counter_out;

    /* Update for UnitDelay: '<S11>/ramp_factor' */
    ved__ye_DWork->ramp_factor_DSTATE = ramp_fact_out;

    /* Update for UnitDelay: '<S11>/last_yaw_rate_curve' */
    ved__ye_DWork->last_yaw_rate_curve_DSTATE = rtb_z_yaw_rate_curve;

    /* Update for UnitDelay: '<S41>/UD' */
    ved__ye_DWork->UD_DSTATE_i = rtb_xpost[1];

    /* Weighted Moving Average Block: '<S36>/Weighted Moving Average'
     */
    {
        int32_T iObj;

        /*
         * shift all the discrete states on time delay
         *  being careful not to overwrite a value before it
         *  has been moved.
         */
        for (iObj = (6); iObj > 0; iObj--) {
            ved__ye_DWork->WeightedMovingAverage_TapDelayU[iObj] =
                ved__ye_DWork->WeightedMovingAverage_TapDelayU[iObj - 1];
        }

        /*
         * the top state is the current input
         */
        ved__ye_DWork->WeightedMovingAverage_TapDelayU[0] = ved__ye_B->Divide;
    }

    /* Update for UnitDelay: '<S44>/Unit Delay' */
    ved__ye_DWork->UnitDelay_DSTATE_j = rtb_ay_yaw_rate_var;

    /* Update for UnitDelay: '<S48>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S48>/FixPt Constant'
     */
    ved__ye_DWork->FixPtUnitDelay2_DSTATE = 0U;

    /* Update for UnitDelay: '<S48>/FixPt Unit Delay1' */
    ved__ye_DWork->FixPtUnitDelay1_DSTATE = rtb_Yk1;

    /* Update for UnitDelay: '<S42>/hold_value' */
    ved__ye_DWork->hold_value_DSTATE = rtb_stw_yaw_rate_var;

    /* Update for UnitDelay: '<S42>/hold_counter' */
    ved__ye_DWork->hold_counter_DSTATE_e = rtb_Value_in_uint;

    /* Update for UnitDelay: '<S6>/P_delay' */
    for (i = 0; i < 4; i++) {
        ved__ye_DWork->P_delay_DSTATE_l[(i)] = rtb_P_post_o[i];
    }
}

/* Model initialize function */
void ved__ye_initialize(boolean_T firstTime,
                        RT_MODEL_ved__ye *const ved__ye_M,
                        BlockIO_ved__ye *ved__ye_B,
                        D_Work_ved__ye *ved__ye_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__ye_M, (NULL));

    /* block I/O */
    (void)memset(((void *)ved__ye_B), 0, sizeof(BlockIO_ved__ye));

    /* states (dwork) */
    (void)memset((void *)ved__ye_DWork, 0, sizeof(D_Work_ved__ye));

    /* Start for ifaction SubSystem: '<S66>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S73>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &ved__ye_DWork->PHt_HPHt_R_DWORK4[0];
        for (i = 0; i < 16; i++) {
            *dWork++ = 0.0;
        }

        dWork = &ved__ye_DWork->PHt_HPHt_R_DWORK4[0];
        while (dWork < &ved__ye_DWork->PHt_HPHt_R_DWORK4[0] + 16) {
            *dWork = 1;
            dWork += 5;
        }
    }

    /* end of Start for SubSystem: '<S66>/calculate the gain' */

    /* Start for ifaction SubSystem: '<S25>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S32>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &ved__ye_DWork->PHt_HPHt_R_DWORK4_m[0];
        for (i = 0; i < 4; i++) {
            *dWork++ = 0.0;
        }

        dWork = &ved__ye_DWork->PHt_HPHt_R_DWORK4_m[0];
        while (dWork < &ved__ye_DWork->PHt_HPHt_R_DWORK4_m[0] + 4) {
            *dWork = 1;
            dWork += 3;
        }
    }

    /* end of Start for SubSystem: '<S25>/calculate the gain' */
    {
        int32_T i;

        /* InitializeConditions for UnitDelay: '<S15>/UD' */
        ved__ye_DWork->UD_DSTATE = 2.5E-7F;

        /* InitializeConditions for UnitDelay: '<S13>/x_delay' */
        for (i = 0; i < 2; i++) {
            ved__ye_DWork->x_delay_DSTATE[(i)] = ved__ye_x_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S13>/P_delay' */
        for (i = 0; i < 4; i++) {
            ved__ye_DWork->P_delay_DSTATE[(i)] = ved__ye_P_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S16>/UD' */
        ved__ye_DWork->UD_DSTATE_m = 2.5E-7F;

        /* InitializeConditions for UnitDelay: '<S18>/UD' */
        ved__ye_DWork->UD_DSTATE_h = 2.5E-7F;

        /* InitializeConditions for UnitDelay: '<S17>/UD' */
        ved__ye_DWork->UD_DSTATE_j = 2.5E-7F;

        /* InitializeConditions for UnitDelay: '<S6>/x_delay' */
        for (i = 0; i < 2; i++) {
            ved__ye_DWork->x_delay_DSTATE_k[(i)] = ved__ye_x_curve_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S48>/FixPt Unit Delay2' */
        ved__ye_DWork->FixPtUnitDelay2_DSTATE = 1U;

        /* InitializeConditions for UnitDelay: '<S42>/hold_value' */
        ved__ye_DWork->hold_value_DSTATE = 1.0F;

        /* InitializeConditions for UnitDelay: '<S6>/P_delay' */
        for (i = 0; i < 4; i++) {
            ved__ye_DWork->P_delay_DSTATE_l[(i)] = ved__ye_P_curve_init_p[(i)];
        }
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */