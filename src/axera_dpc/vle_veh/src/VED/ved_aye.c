#include <ved_consts.h>
#include "tue_common_libs.h"
#include "ved_aye.h"
#include "ved_aye_private.h"
#include "ved_rt_matrixlib.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Model step function */
void ved__aye_step(D_Work_ved__aye *ved__aye_DWork,
                   VED_InputData_t *ved__aye_U_VED_InputData,
                   VED_InternalData_t *ved__aye_U_VED_InternalData_in,
                   VED_InternalData_t *ved__aye_Y_ved__aye_out) {
    /* local block i/o variables */
    real32_T rtb_HPHt_R[4];
    real32_T rtb_P_predHt[8];
    real32_T rtb_K[8];
    real32_T rtb_y_b;

    /* local scratch DWork variables */
    real32_T PHt_HPHt_R_DWORK1[4];
    real32_T PHt_HPHt_R_DWORK3[4];
    real32_T PHt_HPHt_R_DWORK5[4];
    int32_T PHt_HPHt_R_DWORK2[2];
    real32_T rtb_MatrixConcatenate[16];
    real32_T rtb_Time2Sec;
    real32_T rtb_xpost[4];
    real32_T rtb_x_out[4];
    real32_T rtb_x_pred[4];
    real32_T rtb_y_i;
    real32_T rtb_z_out[2];
    real32_T rtb_hx[2];
    uint8_T rtb_R_onoff;
    real32_T rtb_Sum;
    real32_T rtb_H[8];
    real32_T rtb_Ht[8];
    real32_T rtb_P_pred[16];
    real32_T rtb_R_out[4];
    int32_T i;
    real32_T rtb_x_out_0[16];
    real32_T rtb_MatrixConcatenate_0[16];
    real32_T rtb_H_0[8];
    int32_T i_0;
    real32_T rtb_z_out_0[2];
    int32_T i_1;
    real32_T tmp;
    static int8_T tmp_0[4] = {0, 1, 0, 0};

    /* Embedded MATLAB: '<S9>/gnerate_z_vector' incorporates:
     *  Constant: '<S22>/Constant1'
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     *  MultiPortSwitch: '<S22>/Index Vector'
     */
    /* Embedded MATLAB Function 'make_z_vektor/gnerate_z_vector': '<S23>:1' */
    /*  check if the velocity is above 3.0 m/s */
    if ((*ved__aye_U_VED_InternalData_in).ved__ve_out.veh_velo >= 3.0F) {
        /* '<S23>:1:3' */
        /*  check if the lateral acceleration signal is valid */
        if (((uint32_T)(*ved__aye_U_VED_InputData)
                 .Signals.State[(((uint32_T)VED_SIN_POS_LATA))]) ==
            ((uint32_T)VED_IO_STATE_VALID)) {
            /* '<S23>:1:5' */
            /*  if the lat offset state is above 1 */
            if ((*ved__aye_U_VED_InternalData_in)
                    .ved__offsets_in.ved__ay_offset.state > 1) {
                /* '<S23>:1:7' */
                /* '<S23>:1:8' */
                rtb_R_onoff = 1U;

                /* '<S23>:1:9' */
                rtb_z_out[0] = (*ved__aye_U_VED_InputData).Signals.LatAccel -
                               (*ved__aye_U_VED_InternalData_in)
                                   .ved__offsets_in.ved__ay_offset.offset;
                rtb_z_out[1] = (*ved__aye_U_VED_InternalData_in)
                                   .ved__gye_out.gier_yaw_rate;
            } else {
                /* '<S23>:1:11' */
                rtb_R_onoff = 2U;

                /* '<S23>:1:12' */
                rtb_z_out[0] = (*ved__aye_U_VED_InputData).Signals.LatAccel;
                rtb_z_out[1] = (*ved__aye_U_VED_InternalData_in)
                                   .ved__gye_out.gier_yaw_rate;
            }
        } else {
            /* '<S23>:1:15' */
            rtb_R_onoff = 0U;

            /* '<S23>:1:16' */
            for (i = 0; i < 2; i++) {
                rtb_z_out[i] = 0.0F;
            }
        }
    } else {
        /* '<S23>:1:19' */
        rtb_R_onoff = 0U;

        /* '<S23>:1:20' */
        for (i = 0; i < 2; i++) {
            rtb_z_out[i] = 0.0F;
        }
    }

    /* Embedded MATLAB: '<S5>/adapt_R_matrix' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    for (i = 0; i < 4; i++) {
        rtb_R_out[i] = ved__aye_R_p[(i)];
    }

    /* Embedded MATLAB Function 'R/adapt_R_matrix': '<S21>:1' */
    /*  if the lateral acceleration signal is valid */
    if (rtb_R_onoff >= 1) {
        /* '<S21>:1:3' */
        /* '<S21>:1:4' */
        rtb_R_out[3] =
            (*ved__aye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate_var;

        /* '<S21>:1:5' */
    } else {
        /* '<S21>:1:7' */
        rtb_R_out[0] = ved__aye_R_ay_invalid_p;

        /* '<S21>:1:8' */
        rtb_R_out[3] =
            (*ved__aye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate_var;

        /* '<S21>:1:9' */
    }

    rtb_MatrixConcatenate[0] = 1.0F;

    rtb_MatrixConcatenate[1] = 0.0F;

    rtb_MatrixConcatenate[2] = 0.0F;

    rtb_MatrixConcatenate[3] = 0.0F;

    /* Outputs for atomic SubSystem: '<Root>/Time2Sec' */

    /* Product: '<S6>/Time2Sec' incorporates:
     *  Constant: '<S6>/Constant'
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_Time2Sec =
        ((real32_T)(*ved__aye_U_VED_InputData).Frame.CycleTime) / 1000.0F;

    /* end of Outputs for SubSystem: '<Root>/Time2Sec' */

    rtb_MatrixConcatenate[4] = rtb_Time2Sec;

    rtb_MatrixConcatenate[5] = 1.0F;

    rtb_MatrixConcatenate[6] = 0.0F;

    rtb_MatrixConcatenate[7] = 0.0F;

    rtb_MatrixConcatenate[8] = 0.0F;

    rtb_MatrixConcatenate[9] = 0.0F;

    rtb_MatrixConcatenate[10] = 1.0F;

    rtb_MatrixConcatenate[11] = 0.0F;

    rtb_MatrixConcatenate[12] = 0.0F;

    rtb_MatrixConcatenate[13] = 0.0F;

    rtb_MatrixConcatenate[14] = 0.0F;

    rtb_MatrixConcatenate[15] = 1.0F;

    /* Embedded MATLAB: '<S2>/check_track_bent_range' */
    /* Embedded MATLAB Function 'EKF/check_track_bent_range': '<S14>:1' */
    /*  check if the track bent is in valid range */
    /* '<S14>:1:3' */
    for (i = 0; i < 4; i++) {
        /* UnitDelay: '<S2>/x_delay' */
        rtb_xpost[i] = ved__aye_DWork->x_delay_DSTATE[(i)];
        rtb_x_out[i] = ved__aye_DWork->x_delay_DSTATE[(i)];
    }

    /*  if  */
    if (ved__aye_DWork->x_delay_DSTATE[2] < -0.5F) {
        /* '<S14>:1:6' */
        /* '<S14>:1:7' */
        rtb_x_out[2] = -0.5F;
    }

    /*  if  */
    if (ved__aye_DWork->x_delay_DSTATE[2] > 0.5F) {
        /* '<S14>:1:11' */
        /* '<S14>:1:12' */
        rtb_x_out[2] = 0.5F;
    }

    /* Product: '<S2>/Ax' */
    for (i = 0; i < 4; i++) {
        rtb_x_pred[i] = 0.0F;
        for (i_1 = 0; i_1 < 4; i_1++) {
            rtb_x_pred[i] =
                (rtb_MatrixConcatenate[(i_1 << 2) + i] * rtb_x_out[i_1]) +
                rtb_x_pred[i];
        }
    }

    /* Embedded MATLAB: '<S15>/x(3)' */
    /* Embedded MATLAB Function 'EKF/hx/x(3)': '<S20>:1' */
    /* '<S20>:1:2' */
    rtb_y_b = rtb_x_pred[2];

    /* Embedded MATLAB: '<S15>/hx' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     *  S-Function (ex_sfun_cos32): '<S15>/cos'
     *  S-Function (ex_sfun_sin32): '<S15>/sin'
     */
    /* Embedded MATLAB Function 'EKF/hx/hx': '<S19>:1' */
    /*  non linear transformation function from state space to measurement space
     */
    /* '<S19>:1:3' */
    rtb_hx[0] = (((*ved__aye_U_VED_InternalData_in).ved__ve_out.veh_velo *
                  rtb_x_pred[1]) -
                 (SIN_((real32_T)rtb_y_b) * 9.81F)) -
                rtb_x_pred[3];
    rtb_hx[1] = rtb_x_pred[1];

    /*  linearized by the Jacobi matrix */
    /* '<S19>:1:7' */
    rtb_H[0] = 0.0F;
    rtb_H[2] = (*ved__aye_U_VED_InternalData_in).ved__ve_out.veh_velo;
    rtb_H[4] = (-COS_((real32_T)rtb_y_b)) * 9.81F;
    rtb_H[6] = 1.0F;
    for (i = 0; i < 4; i++) {
        rtb_H[1 + (i << 1)] = (real32_T)tmp_0[i];
    }

    /* Product: '<S4>/Product2' incorporates:
     *  Constant: '<S4>/Constant'
     *  Constant: '<S4>/Constant1'
     *  Constant: '<S4>/Constant2'
     *  Constant: '<S4>/sigma_model'
     *  Product: '<S4>/Divide'
     *  Product: '<S4>/Product1'
     */
    rtb_x_out[0] =
        ((rtb_Time2Sec * rtb_Time2Sec) * 0.5F) * ved__aye_Q_sigmas_p[0];
    rtb_x_out[1] = rtb_Time2Sec * ved__aye_Q_sigmas_p[1];
    rtb_x_out[2] = 0.0F;
    rtb_x_out[3] = 0.0F;

    /* Sum: '<S25>/Sum' incorporates:
     *  Gain: '<S25>/Gain'
     *  Sum: '<S25>/Diff'
     *  UnitDelay: '<S10>/delay'
     *  UnitDelay: '<S25>/UD'
     */
    rtb_Sum =
        ((ved__aye_DWork->UD_DSTATE - ved__aye_DWork->delay_DSTATE) * 0.7F) +
        ved__aye_DWork->delay_DSTATE;

    /* Embedded MATLAB: '<S10>/q_gain_delta' incorporates:
     *  Constant: '<S10>/Constant3'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'q_gain/q_gain_delta': '<S24>:1' */
    /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition below
     * is rewritten with multiple if-elseif expressions. */
    /*  calculate the difference between the mean filtered last yaw rate and  */
    /*  the actual yaw rate */
    if (((real32_T)fabs(
            (*ved__aye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate)) >=
        1.0E-7F) {
        /* '<S24>:1:6' */
        if (((real32_T)fabs(rtb_Sum)) >= 1.0E-7F) {
            /* '<S24>:1:7' */
            /* '<S24>:1:8' */
            rtb_Time2Sec =
                (*ved__aye_U_VED_InternalData_in).ved__gye_out.gier_yaw_rate -
                rtb_Sum;
        } else {
            /* '<S24>:1:10' */
            rtb_Time2Sec = 0.2F;
        }
    } else {
        /* '<S24>:1:13' */
        rtb_Time2Sec = 0.2F;
    }

    /* Sum: '<S10>/Sum1' incorporates:
     *  Abs: '<S10>/Abs1'
     *  Constant: '<Root>/ved__aye_Q_gain'
     *  Constant: '<S10>/Constant1'
     *  Constant: '<S10>/Constant2'
     *  MultiPortSwitch: '<S10>/bias_value'
     *  MultiPortSwitch: '<S10>/gain_factor'
     *  Product: '<S10>/Product1'
     */
    rtb_Time2Sec = (((real32_T)fabs(rtb_Time2Sec)) * ved__aye_Q_gain_p[0]) +
                   ved__aye_Q_gain_p[1];

    /* Sum: '<S2>/APA_Q' incorporates:
     *  Constant: '<S4>/additional'
     *  Math: '<S2>/At'
     *  Product: '<S2>/APAt'
     *  Product: '<S4>/Product'
     *  Product: '<S4>/Product3'
     *  Sum: '<S4>/Sum'
     *  UnitDelay: '<S2>/P_delay'
     */
    for (i = 0; i < 4; i++) {
        for (i_1 = 0; i_1 < 4; i_1++) {
            rtb_x_out_0[i_1 + (i << 2)] = rtb_x_out[i_1] * rtb_x_out[i];
        }
    }

    for (i = 0; i < 4; i++) {
        for (i_1 = 0; i_1 < 4; i_1++) {
            rtb_MatrixConcatenate_0[i + (i_1 << 2)] = 0.0F;
            for (i_0 = 0; i_0 < 4; i_0++) {
                rtb_MatrixConcatenate_0[i + (i_1 << 2)] =
                    (rtb_MatrixConcatenate[(i_0 << 2) + i] *
                     ved__aye_DWork->P_delay_DSTATE[(i_1 << 2) + i_0]) +
                    rtb_MatrixConcatenate_0[(i_1 << 2) + i];
            }
        }
    }

    for (i = 0; i < 4; i++) {
        for (i_1 = 0; i_1 < 4; i_1++) {
            tmp = 0.0F;
            for (i_0 = 0; i_0 < 4; i_0++) {
                tmp += rtb_MatrixConcatenate_0[(i_0 << 2) + i] *
                       rtb_MatrixConcatenate[(i_0 << 2) + i_1];
            }

            rtb_P_pred[i + (i_1 << 2)] =
                ((rtb_x_out_0[(i_1 << 2) + i] * rtb_Time2Sec) +
                 ved__aye_Q_add_p[(i_1 << 2) + i]) +
                tmp;
        }
    }

    /* Math: '<S2>/Ht' */
    for (i = 0; i < 2; i++) {
        for (i_1 = 0; i_1 < 4; i_1++) {
            rtb_Ht[i_1 + (i << 2)] = rtb_H[(i_1 << 1) + i];
        }
    }

    /* Sum: '<S2>/HPH_R' incorporates:
     *  Product: '<S2>/HPHt'
     */
    for (i = 0; i < 2; i++) {
        for (i_1 = 0; i_1 < 4; i_1++) {
            rtb_H_0[i + (i_1 << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 4; i_0++) {
                rtb_H_0[i + (i_1 << 1)] =
                    (rtb_H[(i_0 << 1) + i] * rtb_P_pred[(i_1 << 2) + i_0]) +
                    rtb_H_0[(i_1 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_1 = 0; i_1 < 2; i_1++) {
            tmp = 0.0F;
            for (i_0 = 0; i_0 < 4; i_0++) {
                tmp += rtb_H_0[(i_0 << 1) + i] * rtb_Ht[(i_1 << 2) + i_0];
            }

            rtb_HPHt_R[i + (i_1 << 1)] = rtb_R_out[(i_1 << 1) + i] + tmp;
        }
    }

    /* Product: '<S2>/P_pred*Ht' */
    for (i = 0; i < 4; i++) {
        for (i_1 = 0; i_1 < 2; i_1++) {
            rtb_P_predHt[i + (i_1 << 2)] = 0.0F;
            for (i_0 = 0; i_0 < 4; i_0++) {
                rtb_P_predHt[i + (i_1 << 2)] =
                    (rtb_P_pred[(i_0 << 2) + i] * rtb_Ht[(i_1 << 2) + i_0]) +
                    rtb_P_predHt[(i_1 << 2) + i];
            }
        }
    }

    /* Embedded MATLAB: '<S11>/calculate determinant' */
    /* Embedded MATLAB Function 'EKF/Calculate Kalman gain
     * PHt_(HPHt_R)/calculate determinant': '<S16>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /*  y = single(det(u)); */
    /* '<S16>:1:5' */
    rtb_y_i = (rtb_HPHt_R[0] * rtb_HPHt_R[3]) - (rtb_HPHt_R[1] * rtb_HPHt_R[2]);

    /* If: '<S11>/If' incorporates:
     *  ActionPort: '<S17>/Action Port'
     *  ActionPort: '<S18>/Action Port'
     *  SubSystem: '<S11>/calculate the gain'
     *  SubSystem: '<S11>/set gain to default value'
     */
    if (rtb_y_i > 1.0E-16F) {
        /* Product: '<S17>/PHt_(HPHt_R)' */
        {
            static const int_T dims[6] = {2, 2, 2, 4, 2, 2};

            rt_MatDivRR_Sgl(&PHt_HPHt_R_DWORK5[0], rtb_HPHt_R,
                            &ved__aye_DWork->PHt_HPHt_R_DWORK4[0],
                            (real32_T *)&PHt_HPHt_R_DWORK1[0],
                            &PHt_HPHt_R_DWORK2[0],
                            (real32_T *)&PHt_HPHt_R_DWORK3[0], &dims[0]);
            rt_MatMultRR_Sgl(rtb_K, rtb_P_predHt, &PHt_HPHt_R_DWORK5[0],
                             &dims[3]);
        }
    } else {
        /* Constant: '<S18>/Constant' */
        for (i = 0; i < 8; i++) {
            rtb_K[i] = ved__aye_ConstP.Constant_Value[(i)];
        }
    }

    /* Embedded MATLAB: '<S2>/Reset_P_pred' */
    /* Embedded MATLAB Function 'EKF/Reset_P_pred': '<S12>:1' */
    if (((real32_T)fabs(rtb_y_i)) <= 1.0E-16F) {
        /* '<S12>:1:4' */
        /* '<S12>:1:6' */
        memset((void *)(&rtb_P_pred[0]), (int32_T)0.0F, sizeof(real32_T) << 4U);
    } else {
        /* '<S12>:1:10' */
    }

    /* Product: '<S2>/P_pred_(1_KH)' incorporates:
     *  Constant: '<S2>/eye'
     *  Product: '<S2>/K*H'
     *  Sum: '<S2>/1_KH'
     */
    for (i = 0; i < 4; i++) {
        for (i_1 = 0; i_1 < 4; i_1++) {
            tmp = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                tmp += rtb_K[(i_0 << 2) + i] * rtb_H[(i_1 << 1) + i_0];
            }

            rtb_MatrixConcatenate[i + (i_1 << 2)] =
                ved__aye_ConstP.eye_Value[(i_1 << 2) + i] - tmp;
        }
    }

    for (i = 0; i < 4; i++) {
        for (i_1 = 0; i_1 < 4; i_1++) {
            ved__aye_DWork->P_delay_DSTATE[i + (i_1 << 2)] = 0.0F;
            for (i_0 = 0; i_0 < 4; i_0++) {
                ved__aye_DWork->P_delay_DSTATE[i + (i_1 << 2)] =
                    (rtb_MatrixConcatenate[(i_0 << 2) + i] *
                     rtb_P_pred[(i_1 << 2) + i_0]) +
                    ved__aye_DWork->P_delay_DSTATE[(i_1 << 2) + i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/correct_variance' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'correct_variance': '<S7>:1' */
    /*  the output variance for this module is raised at low speed */
    /*  if the input values are not valid set a high constant */
    /*  if the velocity is in a low speed range eg. between 1 and 10 m/s */
    /*  the variance is linear increasing to the bottom limit with the velocity
     */
    /*  below this bottom limit the variance is constant */
    /*  if the velocity if above the upper limit nothing is done */
    if (rtb_R_onoff >= 1) {
        /* '<S7>:1:8' */
        if (((*ved__aye_U_VED_InternalData_in).ved__ve_out.veh_velo <
             ved__aye_P_correct_p[1]) &&
            ((*ved__aye_U_VED_InternalData_in).ved__ve_out.veh_velo >
             ved__aye_P_correct_p[0])) {
            /* '<S7>:1:9' */
            /* '<S7>:1:10' */
            rtb_Time2Sec =
                ((*ved__aye_U_VED_InternalData_in).ved__ve_out.veh_velo -
                 ved__aye_P_correct_p[0]) *
                ved__aye_P_correct_p[2];
            for (i = 0; i < 4; i++) {
                rtb_x_out[i] = ved__aye_DWork->P_delay_DSTATE[(5 * i)] +
                               (ved__aye_P_correct_p[3] - rtb_Time2Sec);
            }
        } else if ((*ved__aye_U_VED_InternalData_in).ved__ve_out.veh_velo <=
                   ved__aye_P_correct_p[0]) {
            /* '<S7>:1:11' */
            /* '<S7>:1:12' */
            for (i = 0; i < 4; i++) {
                rtb_x_out[i] = ved__aye_DWork->P_delay_DSTATE[(5 * i)] +
                               ved__aye_P_correct_p[3];
            }
        } else {
            /*  if the ay offset is not valid */
            if (rtb_R_onoff == 2) {
                /* '<S7>:1:15' */
                /* '<S7>:1:16' */
                for (i = 0; i < 4; i++) {
                    rtb_x_out[i] =
                        ved__aye_DWork->P_delay_DSTATE[(5 * i)] * 1.2F;
                }
            } else {
                /* '<S7>:1:18' */
                for (i = 0; i < 4; i++) {
                    rtb_x_out[i] = ved__aye_DWork->P_delay_DSTATE[(5 * i)];
                }
            }
        }
    } else {
        /* '<S7>:1:22' */
        for (i = 0; i < 4; i++) {
            rtb_x_out[i] = ved__aye_P_correct_p[3];
        }
    }

    /* BusAssignment: '<Root>/Bus Assignment1' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    (*ved__aye_Y_ved__aye_out) = (*ved__aye_U_VED_InternalData_in);
    (*ved__aye_Y_ved__aye_out).ved__aye_out.ay_yaw_rate_var = rtb_x_out[1];

    /* BusAssignment: '<Root>/Bus Assignment3' */
    (*ved__aye_Y_ved__aye_out).ved__aye_out.track_bent_var = rtb_x_out[2];

    /* Embedded MATLAB: '<S2>/Reset_x_pred' */
    /* Embedded MATLAB Function 'EKF/Reset_x_pred': '<S13>:1' */
    if (((real32_T)fabs(rtb_y_i)) <= 1.0E-16F) {
        /* '<S13>:1:4' */
        /* '<S13>:1:6' */
        rtb_x_pred[0] = 0.0F;
        rtb_x_pred[1] = rtb_xpost[1];
        rtb_x_pred[2] = rtb_xpost[2];
        rtb_x_pred[3] = 0.0F;
    } else {
        /* '<S13>:1:10' */
    }

    /* Sum: '<S2>/x_Knu' incorporates:
     *  Product: '<S2>/Knu'
     *  Sum: '<S2>/z_Hx'
     */
    for (i = 0; i < 2; i++) {
        rtb_z_out_0[i] = rtb_z_out[i] - rtb_hx[i];
    }

    for (i = 0; i < 4; i++) {
        tmp = 0.0F;
        for (i_1 = 0; i_1 < 2; i_1++) {
            tmp += rtb_K[(i_1 << 2) + i] * rtb_z_out_0[i_1];
        }

        ved__aye_DWork->x_delay_DSTATE[(i)] = rtb_x_pred[i] + tmp;
    }

    /* Embedded MATLAB: '<Root>/correct_yaw_rate' */
    /* Embedded MATLAB Function 'correct_yaw_rate': '<S8>:1' */
    /*  the output yaw rate is set to 0 */
    /*  if the input values are not valid */
    if (rtb_R_onoff >= 1) {
        /* '<S8>:1:4' */
        /* '<S8>:1:5' */
        rtb_Time2Sec = ved__aye_DWork->x_delay_DSTATE[1];
    } else {
        /* '<S8>:1:7' */
        rtb_Time2Sec = 0.0F;
    }

    /* BusAssignment: '<Root>/Bus Assignment' */
    (*ved__aye_Y_ved__aye_out).ved__aye_out.ay_yaw_rate = rtb_Time2Sec;
    (*ved__aye_Y_ved__aye_out).ved__aye_out.r_On_Off_control = rtb_R_onoff;

    /* BusAssignment: '<Root>/ay_yaw_rate' */
    (*ved__aye_Y_ved__aye_out).ved__aye_out.track_bent =
        ved__aye_DWork->x_delay_DSTATE[2];

    /* Update for UnitDelay: '<S25>/UD' */
    ved__aye_DWork->UD_DSTATE = rtb_Sum;

    /* Update for UnitDelay: '<S10>/delay' */
    ved__aye_DWork->delay_DSTATE = ved__aye_DWork->x_delay_DSTATE[1];
}

/* Model initialize function */
void ved__aye_initialize(boolean_T firstTime,
                         RT_MODEL_ved__aye *const ved__aye_M,
                         D_Work_ved__aye *ved__aye_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__aye_M, (NULL));

    /* states (dwork) */
    (void)memset((void *)ved__aye_DWork, 0, sizeof(D_Work_ved__aye));

    /* Start for ifaction SubSystem: '<S11>/calculate the gain' */
    /* Create Identity Matrix for Block: '<S17>/PHt_(HPHt_R)' */
    {
        int_T i;
        real32_T *dWork = &ved__aye_DWork->PHt_HPHt_R_DWORK4[0];
        for (i = 0; i < 4; i++) {
            *dWork++ = 0.0;
        }

        dWork = &ved__aye_DWork->PHt_HPHt_R_DWORK4[0];
        while (dWork < &ved__aye_DWork->PHt_HPHt_R_DWORK4[0] + 4) {
            *dWork = 1;
            dWork += 3;
        }
    }

    /* end of Start for SubSystem: '<S11>/calculate the gain' */
    {
        int32_T i;

        /* InitializeConditions for UnitDelay: '<S2>/x_delay' */
        for (i = 0; i < 4; i++) {
            ved__aye_DWork->x_delay_DSTATE[(i)] = ved__aye_x_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S2>/P_delay' */
        for (i = 0; i < 16; i++) {
            ved__aye_DWork->P_delay_DSTATE[(i)] = ved__aye_P_init_p[(i)];
        }
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
