#include "ved_consts.h"

#include "ved_gye.h"
#include "ved_gye_private.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* QAC Fixes */

/*
 * Output and update for atomic system:
 *    '<S10>/Get_IO_State1'
 *    '<S10>/Get_IO_State2'
 */
void ved__gye_Get_IO_State1(const uint8_T rtu_state_in[32],
                            rtB_Get_IO_State1_ved__gye *localB,
                            uint32_T rtp_Filter) {
    /* MultiPortSwitch: '<S31>/Index Vector' incorporates:
     *  Constant: '<S31>/Constant1'
     */
    localB->IndexVector = rtu_state_in[(rtp_Filter)];
}

/* Model step function */
void ved__gye_step(BlockIO_ved__gye *ved__gye_B,
                   D_Work_ved__gye *ved__gye_DWork,
                   VED_InputData_t *ved__gye_U_VED_InputData,
                   VED_InternalData_t *ved__gye_U_VED_InternalData_in,
                   VED_InternalData_t *ved__gye_Y_VED_InternalData_out) {
    /* local block i/o variables */
    real32_T rtb_Abs1;
    uint8_T rtb_State[32];
    boolean_T rtb_R_onoff;
    real32_T rtb_delay;
    real32_T rtb_Time2Sec;
    uint8_T rtb_Reset;
    real32_T rtb_nu;
    real32_T rtb_gain;
    uint8_T rtb_Value_in_uint;
    real32_T rtb_MatrixConcatenate[4];
    real32_T rtb_Merge;
    real32_T rtb_y[2];
    real32_T rtb_x_pred_out[2];
    real32_T rtb_off_comp_yaw_rate;
    real32_T rtb_init_value;
    real32_T rtb_Sum;
    real32_T rtb_Sum_j;
    real32_T rtb_P_pred[4];
    real32_T rtb_K[2];
    real32_T rtb_P_post[4];
    int32_T i;
    real32_T rtb_Time2Sec_0[4];
    real32_T rtb_MatrixConcatenate_0[4];
    int32_T i_0;
    int32_T i_1;

    /* BusSelector: '<S10>/Bus Selector1' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State[i] = (*ved__gye_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S10>/Get_IO_State1' */
    ved__gye_Get_IO_State1(rtb_State, &ved__gye_B->Get_IO_State1,
                           ((uint32_T)VED_SIN_POS_YWR));

    /* end of Outputs for SubSystem: '<S10>/Get_IO_State1' */

    /* Outputs for atomic SubSystem: '<S10>/Get_IO_State2' */
    ved__gye_Get_IO_State1(rtb_State, &ved__gye_B->Get_IO_State2,
                           ((uint32_T)VED_SIN_POS_YWRINT));

    /* end of Outputs for SubSystem: '<S10>/Get_IO_State2' */

    /* Sum: '<S10>/Sum' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     */
    rtb_off_comp_yaw_rate =
        (*ved__gye_U_VED_InputData).Signals.YawRate -
        (*ved__gye_U_VED_InternalData_in).ved__wye_out.gier_yaw_rate_offset;

    /* Embedded MATLAB: '<S10>/R_yaw_valid' incorporates:
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     *  Sum: '<S10>/Sum1'
     */
    /* Embedded MATLAB Function 'make_z_vektor/R_yaw_valid': '<S33>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /*  check the internal yaw rate processing switch */
    if (((uint32_T)CFG_VED__INT_GYRO) == 0U) {
        /* '<S33>:1:6' */
        if (((((uint32_T)ved__gye_B->Get_IO_State1.IndexVector) ==
              ((uint32_T)VED_IO_STATE_VALID)) ||
             (((uint32_T)ved__gye_B->Get_IO_State1.IndexVector) ==
              ((uint32_T)VED_IO_STATE_DECREASED))) ||
            (((uint32_T)ved__gye_B->Get_IO_State1.IndexVector) ==
             ((uint32_T)VED_IO_STATE_SUBSTITUE))) {
            /* '<S33>:1:7' */
            /*  use the external yaw rate signal */
            /* '<S33>:1:9' */
            rtb_R_onoff = TRUE;

            /* '<S33>:1:10' */
        } else {
            /* '<S33>:1:12' */
            rtb_R_onoff = FALSE;

            /* '<S33>:1:13' */
            rtb_off_comp_yaw_rate = 0.0F;
        }
    } else {
        /*  if internal yaw rate sensor processing is switched on, */
        /*  but the external yaw rate is valid use the external yaw rate signal.
         */
        if (((uint32_T)ved__gye_B->Get_IO_State1.IndexVector) ==
            ((uint32_T)VED_IO_STATE_VALID)) {
            /* '<S33>:1:18' */
            /*  use the external yaw rate signal */
            /* '<S33>:1:20' */
            rtb_R_onoff = TRUE;

            /* '<S33>:1:21' */
        } else if (((uint32_T)ved__gye_B->Get_IO_State2.IndexVector) ==
                   ((uint32_T)VED_IO_STATE_VALID)) {
            /* '<S33>:1:23' */
            /*  use the internal yaw rate signal */
            /* '<S33>:1:25' */
            rtb_R_onoff = TRUE;

            /* '<S33>:1:26' */
            rtb_off_comp_yaw_rate =
                (*ved__gye_U_VED_InputData).Signals.YawRateInt -
                (*ved__gye_U_VED_InternalData_in)
                    .ved__wye_out.gier_yaw_rate_offset;
        } else {
            /* '<S33>:1:28' */
            rtb_R_onoff = FALSE;

            /* '<S33>:1:29' */
            rtb_off_comp_yaw_rate = 0.0F;
        }
    }

    if (((int32_T)rtb_R_onoff) > 0) {
        /* Embedded MATLAB: '<S20>/calc_resolution_var' */
        /* Embedded MATLAB Function 'R/If Action true/calc_resolution_var':
         * '<S21>:1' */
        /*  the input R variance */
        /* '<S21>:1:3' */
        rtb_Merge = ved__gye_R_p[0];
    } else {
        /* MultiPortSwitch: '<S19>/second_element' incorporates:
         *  Constant: '<S19>/Constant2'
         *  Constant: '<S19>/ved__gye_P_correct_p'
         */
        rtb_Merge = ved__gye_R_p[1];
    }

    /* Outputs for atomic SubSystem: '<Root>/Time2Sec' */

    /* Product: '<S6>/Time2Sec' incorporates:
     *  Constant: '<S6>/Constant'
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_Time2Sec =
        ((real32_T)(*ved__gye_U_VED_InputData).Frame.CycleTime) / 1000.0F;

    /* end of Outputs for SubSystem: '<Root>/Time2Sec' */

    /* Embedded MATLAB: '<S17>/make_Q_matrix' */
    /* Embedded MATLAB Function 'Q/make_Q/make_Q_matrix': '<S18>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S18>:1:5' */
    /* '<S18>:1:6' */

    /* UnitDelay: '<S7>/delay' */
    rtb_delay = ved__gye_DWork->delay_DSTATE;

    /* Product: '<S7>/Divide' incorporates:
     *  Sum: '<S22>/Diff'
     *  UnitDelay: '<S22>/UD'
     */
    ved__gye_B->Divide =
        (rtb_off_comp_yaw_rate - ved__gye_DWork->UD_DSTATE) / rtb_Time2Sec;

    /* Weighted Moving Average Block: '<S7>/Weighted Moving Average'
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
        rtb_Abs1 = ved__gye_B->Divide * 0.125F;

        {
            int_T i1;
            real32_T *dw_TapDelayU =
                &ved__gye_DWork->WeightedMovingAverage_TapDelayU[1];
            for (i1 = 0; i1 < 7; i1++) {
                yTemp1 = dw_TapDelayU[((i1)-1)] *
                         ved__gye_ConstP.WeightedMovingAverage[i1 + 1];
                rtb_Abs1 = rtb_Abs1 + yTemp1;
            }
        }
    }

    /* Abs: '<S7>/Abs' incorporates:
     *  Sum: '<S7>/Add1'
     */
    rtb_delay = (real32_T)fabs(rtb_delay - rtb_Abs1);

    /* Embedded MATLAB: '<S26>/check_reset_condition' incorporates:
     *  UnitDelay: '<S26>/Unit Delay'
     */
    /* Embedded MATLAB Function
     * 'calculate_Q_gain/tp_filter_peaks/check_reset_condition': '<S28>:1' */
    if (rtb_delay > ved__gye_DWork->UnitDelay_DSTATE) {
        /* '<S28>:1:3' */
        /* '<S28>:1:4' */
        rtb_Reset = 1U;

        /* '<S28>:1:5' */
        rtb_init_value = rtb_delay;
    } else {
        /* '<S28>:1:7' */
        rtb_Reset = 0U;

        /* '<S28>:1:8' */
        rtb_init_value = 0.0F;
    }

    /* Switch: '<S30>/Init' incorporates:
     *  Logic: '<S30>/FixPt Logical Operator'
     *  UnitDelay: '<S30>/FixPt Unit Delay1'
     *  UnitDelay: '<S30>/FixPt Unit Delay2'
     */
    if ((rtb_Reset != 0) || (ved__gye_DWork->FixPtUnitDelay2_DSTATE != 0)) {
        rtb_nu = rtb_init_value;
    } else {
        rtb_nu = ved__gye_DWork->FixPtUnitDelay1_DSTATE;
    }

    /* Sum: '<S29>/Sum' incorporates:
     *  Gain: '<S29>/Gain'
     *  Sum: '<S29>/Diff'
     */
    rtb_Sum = ((rtb_nu - rtb_delay) * 0.95F) + rtb_delay;

    /* Abs: '<S7>/Abs1' */
    rtb_Abs1 = (real32_T)fabs(rtb_Abs1);

    /* Sum: '<S23>/Sum' incorporates:
     *  Gain: '<S23>/Gain'
     *  Sum: '<S23>/Diff'
     *  UnitDelay: '<S23>/UD'
     */
    rtb_Sum_j = ((ved__gye_DWork->UD_DSTATE_h - rtb_Abs1) * 0.95F) + rtb_Abs1;

    /* Embedded MATLAB: '<S7>/threshold_gain' */
    /* Embedded MATLAB Function 'calculate_Q_gain/threshold_gain': '<S25>:1' */
    if (rtb_Sum > 0.05F) {
        /* '<S25>:1:3' */
        if (rtb_Sum_j > 0.03F) {
            /* '<S25>:1:4' */
            /* '<S25>:1:5' */
            rtb_gain = rtb_Sum * ved__gye_Q_gain_p;
        } else if (rtb_Sum_j > 0.02F) {
            /* '<S25>:1:6' */
            /* '<S25>:1:7' */
            rtb_gain = (rtb_Sum * ved__gye_Q_gain_p) * 0.5F;
        } else if (rtb_Sum_j > 0.01F) {
            /* '<S25>:1:8' */
            /* '<S25>:1:9' */
            rtb_gain = (rtb_Sum * ved__gye_Q_gain_p) * 0.1F;
        } else {
            /* '<S25>:1:11' */
            rtb_gain = 1.0F;
        }
    } else {
        /* '<S25>:1:14' */
        rtb_gain = 1.0F;

        /* 0.000125 */
    }

    rtb_Value_in_uint = (uint8_T)rtb_gain;

    /* Embedded MATLAB Function
     * 'calculate_Q_gain/hold_last_5_values/hold_last_5_values': '<S27>:1' */
    /*  Hold value 5 cycle if input value is one */
    /*  Added Value_in_uint and Last_value_uint to remove QAC Level 4 warnings
     */
    if ((((rtb_Value_in_uint == 1) &&
          (((uint8_T)ved__gye_DWork->hold_value_DSTATE) != 1)) &&
         (ved__gye_DWork->hold_counter_DSTATE == 0)) ||
        ((ved__gye_DWork->hold_counter_DSTATE > 0) &&
         (ved__gye_DWork->hold_counter_DSTATE <= 5))) {
        /* '<S27>:1:5' */
        if ((ved__gye_DWork->hold_counter_DSTATE >= 1) &&
            (rtb_Value_in_uint != 1)) {
            /* '<S27>:1:6' */
            /* '<S27>:1:7' */
            /* '<S27>:1:8' */
            rtb_Value_in_uint = 0U;
        } else {
            /* '<S27>:1:10' */
            rtb_gain = ved__gye_DWork->hold_value_DSTATE;

            /* '<S27>:1:11' */
            rtb_Value_in_uint =
                (uint8_T)(ved__gye_DWork->hold_counter_DSTATE + 1);
        }
    } else {
        /* '<S27>:1:14' */
        /* '<S27>:1:15' */
        rtb_Value_in_uint = 0U;
    }

    /* Constant: '<S1>/Constant' */
    rtb_MatrixConcatenate[0] = 1.0F;

    /* Constant: '<S1>/Constant1' */
    rtb_MatrixConcatenate[1] = 0.0F;

    rtb_MatrixConcatenate[2] = rtb_Time2Sec;

    /* Constant: '<S1>/Constant2' */
    rtb_MatrixConcatenate[3] = 1.0F;

    /* Sum: '<S2>/APA_Q' incorporates:
     *  Math: '<S2>/At'
     *  Product: '<S2>/APAt'
     *  Product: '<S4>/Product'
     *  UnitDelay: '<S2>/P_delay'
     */
    rtb_Time2Sec_0[0] = ((rtb_Time2Sec * rtb_Time2Sec) * rtb_Time2Sec) / 3.0F;
    rtb_Time2Sec_0[2] = (rtb_Time2Sec * rtb_Time2Sec) / 2.0F;
    rtb_Time2Sec_0[1] = (rtb_Time2Sec * rtb_Time2Sec) / 2.0F;
    rtb_Time2Sec_0[3] = rtb_Time2Sec;
    for (i = 0; i < 2; i++) {
        for (i_1 = 0; i_1 < 2; i_1++) {
            rtb_MatrixConcatenate_0[i + (i_1 << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_MatrixConcatenate_0[i + (i_1 << 1)] =
                    (rtb_MatrixConcatenate[(i_0 << 1) + i] *
                     ved__gye_DWork->P_delay_DSTATE[(i_1 << 1) + i_0]) +
                    rtb_MatrixConcatenate_0[(i_1 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_1 = 0; i_1 < 2; i_1++) {
            rtb_nu = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_nu += rtb_MatrixConcatenate_0[(i_0 << 1) + i] *
                          rtb_MatrixConcatenate[(i_0 << 1) + i_1];
            }

            rtb_P_pred[i + (i_1 << 1)] =
                ((rtb_Time2Sec_0[(i_1 << 1) + i] * 0.0025F) * rtb_gain) +
                rtb_nu;
        }
    }

    /* Sum: '<S2>/HPH_R' incorporates:
     *  Constant: '<S2>/H_const'
     *  Product: '<S2>/HPHt'
     */
    rtb_nu = 0.0F;
    for (i = 0; i < 2; i++) {
        rtb_y[i] = 0.0F;
        for (i_1 = 0; i_1 < 2; i_1++) {
            rtb_y[i] = (rtb_P_pred[(i << 1) + i_1] *
                        ved__gye_ConstP.H_const_Value[(i_1)]) +
                       rtb_y[i];
        }

        rtb_nu += rtb_y[i] * ved__gye_ConstB.Ht[(i)];
    }

    rtb_delay = rtb_Merge + rtb_nu;

    /* Embedded MATLAB: '<S11>/calculate determinant' */
    /* Embedded MATLAB Function 'KF/Calculate Kalman gain/calculate
     * determinant': '<S14>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /* '<S14>:1:3' */

    /* If: '<S11>/If' incorporates:
     *  ActionPort: '<S15>/Action Port'
     *  ActionPort: '<S16>/Action Port'
     *  SubSystem: '<S11>/calculate the gain'
     *  SubSystem: '<S11>/set gain to default value'
     */
    for (i = 0; i < 2; i++) {
        if (rtb_delay > 1.0E-16F) {
            /* Product: '<S15>/PHt_(HPHt_R)' incorporates:
             *  Product: '<S2>/P_pred*Ht'
             */
            rtb_nu = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_nu +=
                    rtb_P_pred[(i_1 << 1) + i] * ved__gye_ConstB.Ht[(i_1)];
            }

            rtb_K[i] = rtb_nu / rtb_delay;
        } else {
            /* Constant: '<S16>/Constant' */
            rtb_K[i] = ved__gye_ConstP.Constant_Value[(i)];
        }
    }

    /* Embedded MATLAB: '<S2>/Reset_P_pred' */
    /* Embedded MATLAB Function 'KF/Reset_P_pred': '<S12>:1' */
    if (((real32_T)fabs(rtb_delay)) <= 1.0E-16F) {
        /* '<S12>:1:4' */
        /* '<S12>:1:6' */
        for (i = 0; i < 4; i++) {
            rtb_P_pred[i] = 0.0F;
        }
    } else {
        /* '<S12>:1:10' */
    }

    /* Product: '<S2>/P_pred_(1_KH)' incorporates:
     *  Constant: '<S2>/H_const'
     *  Constant: '<S2>/H_const1'
     *  Product: '<S2>/K*H'
     *  Sum: '<S2>/1_KH'
     */
    for (i = 0; i < 2; i++) {
        for (i_1 = 0; i_1 < 2; i_1++) {
            rtb_Time2Sec_0[i + (i_1 << 1)] =
                ved__gye_ConstP.H_const1_Value[(i_1 << 1) + i] -
                (rtb_K[i] * ved__gye_ConstP.H_const_Value[(i_1)]);
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_1 = 0; i_1 < 2; i_1++) {
            rtb_P_post[i + (i_1 << 1)] = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_P_post[i + (i_1 << 1)] = (rtb_Time2Sec_0[(i_0 << 1) + i] *
                                              rtb_P_pred[(i_1 << 1) + i_0]) +
                                             rtb_P_post[(i_1 << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/diag_uncertainty' */
    /* Embedded MATLAB Function 'diag_uncertainty': '<S9>:1' */
    /*  correct the ouput variance if the input values are not valid */
    if (((int32_T)rtb_R_onoff) == 1) {
        /* '<S9>:1:3' */
        /* '<S9>:1:4' */
        for (i = 0; i < 2; i++) {
            rtb_y[i] = rtb_P_post[3 * i] + ved__gye_P_correct_p[1];
        }
    } else {
        /* '<S9>:1:6' */
        for (i = 0; i < 2; i++) {
            rtb_y[i] = ved__gye_P_correct_p[0];
        }
    }

    /* BusAssignment: '<Root>/gier_yaw_rate' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    (*ved__gye_Y_VED_InternalData_out) = (*ved__gye_U_VED_InternalData_in);
    (*ved__gye_Y_VED_InternalData_out).ved__gye_out.gier_yaw_rate_var =
        rtb_y[0];

    /* Product: '<S2>/Ax' incorporates:
     *  UnitDelay: '<S2>/x_delay'
     */
    for (i = 0; i < 2; i++) {
        rtb_y[i] = 0.0F;
        for (i_1 = 0; i_1 < 2; i_1++) {
            rtb_y[i] = (rtb_MatrixConcatenate[(i_1 << 1) + i] *
                        ved__gye_DWork->x_delay_DSTATE[(i_1)]) +
                       rtb_y[i];
        }
    }

    /* Embedded MATLAB: '<S2>/Reset_x_pred' incorporates:
     *  UnitDelay: '<S2>/x_delay'
     */
    /* Embedded MATLAB Function 'KF/Reset_x_pred': '<S13>:1' */
    if (((real32_T)fabs(rtb_delay)) <= 1.0E-16F) {
        /* '<S13>:1:4' */
        /* '<S13>:1:6' */
        rtb_x_pred_out[0] = ved__gye_DWork->x_delay_DSTATE[0];
        rtb_x_pred_out[1] = 0.0F;
    } else {
        /* '<S13>:1:10' */
        for (i = 0; i < 2; i++) {
            rtb_x_pred_out[i] = rtb_y[i];
        }
    }

    /* Sum: '<S2>/z_Hx' incorporates:
     *  Constant: '<S2>/H_const'
     *  Product: '<S2>/Product'
     */
    rtb_nu = 0.0F;
    for (i = 0; i < 2; i++) {
        rtb_nu += ved__gye_ConstP.H_const_Value[(i)] * rtb_y[i];
    }

    rtb_nu = rtb_off_comp_yaw_rate - rtb_nu;

    /* Sum: '<S2>/x_Knu' incorporates:
     *  Product: '<S2>/Knu'
     */
    for (i = 0; i < 2; i++) {
        rtb_x_pred_out[i] = (rtb_K[i] * rtb_nu) + rtb_x_pred_out[i];
    }

    /* Embedded MATLAB: '<Root>/correct_output' */
    /* Embedded MATLAB Function 'correct_output': '<S8>:1' */
    /*  correct the ouput if the input values are not valid */
    if (((int32_T)rtb_R_onoff) == 1) {
        /* '<S8>:1:3' */
        /* '<S8>:1:4' */
        rtb_delay = rtb_x_pred_out[0];
    } else {
        /* '<S8>:1:6' */
        rtb_delay = 0.0F;
    }

    /* BusAssignment: '<Root>/Bus Assignment' */
    (*ved__gye_Y_VED_InternalData_out).ved__gye_out.gier_yaw_rate = rtb_delay;
    (*ved__gye_Y_VED_InternalData_out).ved__gye_out.r_On_Off_control =
        rtb_R_onoff;

    /* Switch: '<S30>/Reset' */
    if (rtb_Reset == 0) {
        rtb_init_value = rtb_Sum;
    }

    /* Update for UnitDelay: '<S7>/delay' */
    ved__gye_DWork->delay_DSTATE = rtb_x_pred_out[1];

    /* Update for UnitDelay: '<S22>/UD' */
    ved__gye_DWork->UD_DSTATE = rtb_off_comp_yaw_rate;

    /* Weighted Moving Average Block: '<S7>/Weighted Moving Average'
     */
    {
        int32_T iObj;

        /*
         * shift all the discrete states on time delay
         *  being careful not to overwrite a value before it
         *  has been moved.
         */
        for (iObj = (6); iObj > 0; iObj--) {
            ved__gye_DWork->WeightedMovingAverage_TapDelayU[iObj] =
                ved__gye_DWork->WeightedMovingAverage_TapDelayU[iObj - 1];
        }

        /*
         * the top state is the current input
         */
        ved__gye_DWork->WeightedMovingAverage_TapDelayU[0] = ved__gye_B->Divide;
    }

    /* Update for UnitDelay: '<S26>/Unit Delay' */
    ved__gye_DWork->UnitDelay_DSTATE = rtb_Sum;

    /* Update for UnitDelay: '<S30>/FixPt Unit Delay2' incorporates:
     *  Constant: '<S30>/FixPt Constant'
     */
    ved__gye_DWork->FixPtUnitDelay2_DSTATE = 0U;

    /* Update for UnitDelay: '<S30>/FixPt Unit Delay1' */
    ved__gye_DWork->FixPtUnitDelay1_DSTATE = rtb_init_value;

    /* Update for UnitDelay: '<S23>/UD' */
    ved__gye_DWork->UD_DSTATE_h = rtb_Sum_j;

    /* Update for UnitDelay: '<S24>/hold_value' */
    ved__gye_DWork->hold_value_DSTATE = rtb_gain;

    /* Update for UnitDelay: '<S24>/hold_counter' */
    ved__gye_DWork->hold_counter_DSTATE = rtb_Value_in_uint;

    /* Update for UnitDelay: '<S2>/P_delay' */
    for (i = 0; i < 4; i++) {
        ved__gye_DWork->P_delay_DSTATE[(i)] = rtb_P_post[i];
    }

    /* Update for UnitDelay: '<S2>/x_delay' */
    for (i = 0; i < 2; i++) {
        ved__gye_DWork->x_delay_DSTATE[(i)] = rtb_x_pred_out[i];
    }
}

/* Model initialize function */
void ved__gye_initialize(boolean_T firstTime,
                         RT_MODEL_ved__gye *const ved__gye_M,
                         BlockIO_ved__gye *ved__gye_B,
                         D_Work_ved__gye *ved__gye_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__gye_M, (NULL));

    /* block I/O */
    (void)memset(((void *)ved__gye_B), 0, sizeof(BlockIO_ved__gye));

    /* states (dwork) */
    (void)memset((void *)ved__gye_DWork, 0, sizeof(D_Work_ved__gye));

    {
        int32_T i;

        /* InitializeConditions for UnitDelay: '<S30>/FixPt Unit Delay2' */
        ved__gye_DWork->FixPtUnitDelay2_DSTATE = 1U;

        /* InitializeConditions for UnitDelay: '<S24>/hold_value' */
        ved__gye_DWork->hold_value_DSTATE = 1.0F;

        /* InitializeConditions for UnitDelay: '<S2>/P_delay' */
        for (i = 0; i < 4; i++) {
            ved__gye_DWork->P_delay_DSTATE[(i)] = ved__gye_P_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S2>/x_delay' */
        for (i = 0; i < 2; i++) {
            ved__gye_DWork->x_delay_DSTATE[(i)] = ved__gye_x_init_p[(i)];
        }
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */