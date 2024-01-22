
#include "ved_consts.h"
#include "ved_sae.h"
#include "ved_sae_private.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/* Model step function */
void ved__sae_step(D_Work_ved__sae *ved__sae_DWork,
                   VED_InputData_t *ved__sae_U_VED_InputData,
                   VED_InternalData_t *ved__sae_U_VED_InternalData_in,
                   VED_InternalData_t *ved__sae_Y_VED_InternalData_out) {
    real32_T rtb_x_pred[2];
    real32_T rtb_Time2Sec;
    real32_T rtb_division;
    real32_T rtb_Product;
    real32_T rtb_K[2];
    real32_T rtb_TmpSignalConversionAtSFunct[2];
    real32_T rtb_A[4];
    int8_T rtb_H[2];
    real32_T rtb_P_pred[4];
    int32_T i;
    real32_T rtb_TmpSignalConversionAtSFun_0[2];
    real32_T tmp[2];
    real32_T rtb_TmpSignalConversionAtSFun_1[4];
    real32_T rtb_A_0[4];
    int32_T i_0;
    int32_T i_1;
    static int8_T tmp_0[4] = {1, 0, 0, 1};

    /* Embedded MATLAB: '<S5>/adapt_R_matrix' */
    /* Embedded MATLAB Function 'R/adapt_R_matrix': '<S22>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S22>:1:5' */

    /* Outputs for atomic SubSystem: '<Root>/Time2Sec' */

    /* Product: '<S6>/Time2Sec' incorporates:
     *  Constant: '<S6>/Constant'
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_Time2Sec =
        ((real32_T)(*ved__sae_U_VED_InputData).Frame.CycleTime) / 1000.0F;

    /* end of Outputs for SubSystem: '<Root>/Time2Sec' */

    /* Embedded MATLAB: '<S1>/make_A_matrix' */
    /* Embedded MATLAB Function 'A/make_A_matrix': '<S10>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S10>:1:5' */
    rtb_A[0] = 1.0F;
    rtb_A[2] = rtb_Time2Sec;
    for (i = 0; i < 2; i++) {
        rtb_A[1 + (i << 1)] = (real32_T)i;
    }

    /* Embedded MATLAB: '<S2>/hx' */
    /* Embedded MATLAB Function 'KF/hx': '<S17>:1' */
    /*  transformation from state space to measurement space */
    /* '<S17>:1:3' */
    /* '<S17>:1:5' */
    for (i = 0; i < 2; i++) {
        /* Product: '<S2>/Ax' incorporates:
         *  UnitDelay: '<S2>/x_delay'
         */
        rtb_x_pred[i] = 0.0F;
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_x_pred[i] = (rtb_A[(i_0 << 1) + i] *
                             ved__sae_DWork->x_delay_DSTATE[(i_0)]) +
                            rtb_x_pred[i];
        }

        rtb_H[i] = (int8_T)((-i) + 1);
    }

    rtb_TmpSignalConversionAtSFunct[0] = (rtb_Time2Sec * rtb_Time2Sec) / 2.0F;
    rtb_TmpSignalConversionAtSFunct[1] = rtb_Time2Sec;

    /* Embedded MATLAB: '<S4>/makeQ' incorporates:
     *  Constant: '<S4>/sigma_model'
     */
    /* Embedded MATLAB Function 'Q/makeQ': '<S21>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S21>:1:5' */

    /* Product: '<S8>/Product1' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_Time2Sec =
        (*ved__sae_U_VED_InputData).Parameter.VED_Kf_WheelBase_met *
        (*ved__sae_U_VED_InputData).Parameter.VED_Kf_AxisLoadDistr_per;

    /* Embedded MATLAB: '<S8>/divide_velocity1' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    /* Embedded MATLAB Function 'make_z_vector/divide_velocity1': '<S24>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (((real32_T)fabs(
            (*ved__sae_U_VED_InternalData_in).ved__ve_out.veh_velo)) <=
        0.0001F) {
        /* '<S24>:1:5' */
        /* '<S24>:1:6' */
        rtb_division = 0.0F;
    } else {
        /* '<S24>:1:8' */
        rtb_division = rtb_Time2Sec /
                       (*ved__sae_U_VED_InternalData_in).ved__ve_out.veh_velo;
    }

    /* Product: '<S8>/Product' incorporates:
     *  Constant: '<S8>/ved__sae_Cr'
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     *  Product: '<S8>/Divide'
     *  Product: '<S8>/Product2'
     *  Product: '<S8>/Product3'
     *  Sum: '<S8>/Add'
     *  Sum: '<S8>/Add1'
     */
    rtb_Product =
        (rtb_division -
         (((((*ved__sae_U_VED_InputData).Parameter.VED_Kf_WheelBase_met -
             rtb_Time2Sec) *
            (*ved__sae_U_VED_InternalData_in).ved__ve_out.veh_velo) *
           (*ved__sae_U_VED_InputData).Parameter.VED_Kf_VehWeight_kg) /
          ((*ved__sae_U_VED_InputData).Parameter.VED_Kf_WheelBase_met *
           ved__sae_Cr_p))) *
        (*ved__sae_U_VED_InternalData_in).ved__ye_out.veh_yaw_rate;

    /* Product: '<S26>/Divide' incorporates:
     *  Constant: '<S26>/filter_length'
     *  Sum: '<S26>/Add'
     *  UnitDelay: '<S26>/T0'
     *  UnitDelay: '<S26>/T1'
     *  UnitDelay: '<S26>/T2'
     *  UnitDelay: '<S26>/T3'
     */
    rtb_Time2Sec = (((ved__sae_DWork->T3_DSTATE + ved__sae_DWork->T2_DSTATE) +
                     ved__sae_DWork->T1_DSTATE) +
                    ved__sae_DWork->T0_DSTATE) *
                   0.25F;

    /* Embedded MATLAB: '<S9>/get_gain_bias' incorporates:
     *  Constant: '<Root>/ved__gye_Q_gain'
     */
    /* Embedded MATLAB Function 'q_gain/get_gain_bias': '<S25>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S25>:1:5' */
    /* '<S25>:1:6' */
    /* '<S25>:1:7' */

    /* Embedded MATLAB: '<S9>/q_gain_delta' */
    /* Embedded MATLAB Function 'q_gain/q_gain_delta': '<S27>:1' */
    /*  Note: In order to eliminate QAC Level-4 warnings, the if-condition below
     * is rewritten with multiple if-elseif expressions. */
    /*  calculate the difference between the mean filtered last yaw rate and  */
    /*  the actual yaw rate */
    if (((real32_T)fabs(rtb_Product)) >= 1.0E-7F) {
        /* '<S27>:1:6' */
        if (((real32_T)fabs(rtb_Time2Sec)) >= 1.0E-7F) {
            /* '<S27>:1:7' */
            /* '<S27>:1:8' */
            rtb_Time2Sec = rtb_Product - rtb_Time2Sec;
        } else {
            /* '<S27>:1:10' */
            rtb_Time2Sec = ved__sae_Q_gain_p[2];
        }
    } else {
        /* '<S27>:1:13' */
        rtb_Time2Sec = ved__sae_Q_gain_p[2];
    }

    /* Sum: '<S9>/Sum2' incorporates:
     *  Abs: '<S9>/Abs'
     *  Product: '<S9>/Product'
     */
    rtb_Time2Sec = (((real32_T)fabs(rtb_Time2Sec)) * ved__sae_Q_gain_p[0]) +
                   ved__sae_Q_gain_p[1];

    /* Embedded MATLAB: '<S2>/At' */
    /* Embedded MATLAB Function 'KF/At': '<S11>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S11>:1:5' */

    /* Sum: '<S2>/APA_Q' incorporates:
     *  Constant: '<S4>/additional'
     *  Product: '<S2>/APAt'
     *  Product: '<S4>/Product'
     *  Sum: '<S4>/Sum'
     *  UnitDelay: '<S2>/P_delay'
     */
    for (i = 0; i < 2; i++) {
        rtb_TmpSignalConversionAtSFun_0[i] =
            rtb_TmpSignalConversionAtSFunct[i] * ved__sae_Q_sigmas_p[(i)];
        tmp[i] = ved__sae_Q_sigmas_p[(i)] * rtb_TmpSignalConversionAtSFunct[i];
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_TmpSignalConversionAtSFun_1[i_0 + (i << 1)] =
                rtb_TmpSignalConversionAtSFun_0[i_0] * tmp[i];
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_A_0[i + (i_0 << 1)] = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_A_0[i + (i_0 << 1)] =
                    (rtb_A[(i_1 << 1) + i] *
                     ved__sae_DWork->P_delay_DSTATE[(i_0 << 1) + i_1]) +
                    rtb_A_0[(i_0 << 1) + i];
            }
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_division = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                rtb_division +=
                    rtb_A_0[(i_1 << 1) + i] * rtb_A[(i_1 << 1) + i_0];
            }

            rtb_P_pred[i + (i_0 << 1)] =
                ((rtb_TmpSignalConversionAtSFun_1[(i_0 << 1) + i] *
                  rtb_Time2Sec) +
                 ved__sae_Q_add_p[(i_0 << 1) + i]) +
                rtb_division;
        }
    }

    /* Embedded MATLAB: '<S2>/Ht' */
    /* Embedded MATLAB Function 'KF/Ht': '<S13>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S13>:1:5' */

    /* Sum: '<S2>/HPH_R' incorporates:
     *  Product: '<S2>/HPHt'
     */
    rtb_division = 0.0F;
    for (i = 0; i < 2; i++) {
        rtb_TmpSignalConversionAtSFun_0[i] = 0.0F;
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_TmpSignalConversionAtSFun_0[i] =
                (rtb_P_pred[(i << 1) + i_0] * ((real32_T)rtb_H[i_0])) +
                rtb_TmpSignalConversionAtSFun_0[i];
        }

        rtb_division +=
            rtb_TmpSignalConversionAtSFun_0[i] * ((real32_T)rtb_H[i]);
    }

    rtb_Time2Sec = ved__sae_R_p + rtb_division;

    /* Embedded MATLAB: '<S12>/calculate determinant' */
    /* Embedded MATLAB Function 'KF/Calculate Kalman gain PHt_(HPHt_R)/calculate
     * determinant': '<S18>:1' */
    /* # calculate the determinant of the HPHt*R matrix to check if it can be
     * inverted. */
    /* '<S18>:1:3' */

    /* If: '<S12>/If' incorporates:
     *  ActionPort: '<S19>/Action Port'
     *  ActionPort: '<S20>/Action Port'
     *  SubSystem: '<S12>/calculate the gain'
     *  SubSystem: '<S12>/set gain to default value'
     */
    for (i = 0; i < 2; i++) {
        if (rtb_Time2Sec > 1.0E-16F) {
            /* Product: '<S19>/PHt_(HPHt_R)' incorporates:
             *  Product: '<S2>/P_pred*Ht'
             */
            rtb_division = 0.0F;
            for (i_0 = 0; i_0 < 2; i_0++) {
                rtb_division +=
                    rtb_P_pred[(i_0 << 1) + i] * ((real32_T)rtb_H[i_0]);
            }

            rtb_K[i] = rtb_division / rtb_Time2Sec;
        } else {
            /* Constant: '<S20>/Constant' */
            rtb_K[i] = ved__sae_ConstP.Constant_Value[(i)];
        }
    }

    /* Embedded MATLAB: '<S2>/eye' */
    /* Embedded MATLAB Function 'KF/eye': '<S16>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S16>:1:5' */

    /* Embedded MATLAB: '<S2>/Reset_P_pred' */
    /* Embedded MATLAB Function 'KF/Reset_P_pred': '<S14>:1' */
    if (((real32_T)fabs(rtb_Time2Sec)) <= 1.0E-16F) {
        /* '<S14>:1:4' */
        /* '<S14>:1:6' */
        for (i = 0; i < 4; i++) {
            rtb_P_pred[i] = 0.0F;
        }
    } else {
        /* '<S14>:1:10' */
    }

    /* Product: '<S2>/P_pred_(1_KH)' incorporates:
     *  Product: '<S2>/K*H'
     *  Sum: '<S2>/1_KH'
     */
    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            rtb_A[i_0 + (i << 1)] = ((real32_T)tmp_0[(i << 1) + i_0]) -
                                    (rtb_K[i_0] * ((real32_T)rtb_H[i]));
        }
    }

    for (i = 0; i < 2; i++) {
        for (i_0 = 0; i_0 < 2; i_0++) {
            ved__sae_DWork->P_delay_DSTATE[i + (i_0 << 1)] = 0.0F;
            for (i_1 = 0; i_1 < 2; i_1++) {
                ved__sae_DWork->P_delay_DSTATE[i + (i_0 << 1)] =
                    (rtb_A[(i_1 << 1) + i] * rtb_P_pred[(i_0 << 1) + i_1]) +
                    ved__sae_DWork->P_delay_DSTATE[(i_0 << 1) + i];
            }
        }
    }

    /* Embedded MATLAB: '<Root>/diag_uncertainty' */
    /* Embedded MATLAB Function 'diag_uncertainty': '<S7>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S7>:1:5' */

    /* BusAssignment: '<Root>/slip_angle1' incorporates:
     *  Inport: '<Root>/VED_InternalData_in'
     */
    (*ved__sae_Y_VED_InternalData_out) = (*ved__sae_U_VED_InternalData_in);
    (*ved__sae_Y_VED_InternalData_out).ved__sae_out.est_slip_angle_var =
        ved__sae_DWork->P_delay_DSTATE[0];

    /* Embedded MATLAB: '<S2>/Reset_x_pred' incorporates:
     *  UnitDelay: '<S2>/x_delay'
     */
    /* Embedded MATLAB Function 'KF/Reset_x_pred': '<S15>:1' */
    if (((real32_T)fabs(rtb_Time2Sec)) <= 1.0E-16F) {
        /* '<S15>:1:4' */
        /* '<S15>:1:6' */
        rtb_TmpSignalConversionAtSFunct[0] = ved__sae_DWork->x_delay_DSTATE[0];
        rtb_TmpSignalConversionAtSFunct[1] = 0.0F;
    } else {
        /* '<S15>:1:10' */
        for (i = 0; i < 2; i++) {
            rtb_TmpSignalConversionAtSFunct[i] = rtb_x_pred[i];
        }
    }

    /* Sum: '<S2>/z_Hx' */
    rtb_Time2Sec = rtb_Product - rtb_x_pred[0];

    /* Sum: '<S2>/x_Knu' incorporates:
     *  Product: '<S2>/Knu'
     */
    for (i = 0; i < 2; i++) {
        ved__sae_DWork->x_delay_DSTATE[(i)] =
            (rtb_K[i] * rtb_Time2Sec) + rtb_TmpSignalConversionAtSFunct[i];
    }

    /* BusAssignment: '<Root>/slip_angle' */
    (*ved__sae_Y_VED_InternalData_out).ved__sae_out.est_slip_angle =
        ved__sae_DWork->x_delay_DSTATE[0];

    /* BusAssignment: '<Root>/store_raw_slip_angle' */
    (*ved__sae_Y_VED_InternalData_out).ved__sae_out.raw_slip_angle =
        rtb_Product;

    /* Update for UnitDelay: '<S26>/T3' incorporates:
     *  UnitDelay: '<S26>/T2'
     */
    ved__sae_DWork->T3_DSTATE = ved__sae_DWork->T2_DSTATE;

    /* Update for UnitDelay: '<S26>/T2' incorporates:
     *  UnitDelay: '<S26>/T1'
     */
    ved__sae_DWork->T2_DSTATE = ved__sae_DWork->T1_DSTATE;

    /* Update for UnitDelay: '<S26>/T1' incorporates:
     *  UnitDelay: '<S26>/T0'
     */
    ved__sae_DWork->T1_DSTATE = ved__sae_DWork->T0_DSTATE;

    /* Update for UnitDelay: '<S26>/T0' */
    ved__sae_DWork->T0_DSTATE = ved__sae_DWork->x_delay_DSTATE[0];
}

/* Model initialize function */
void ved__sae_initialize(boolean_T firstTime,
                         RT_MODEL_ved__sae *const ved__sae_M,
                         D_Work_ved__sae *ved__sae_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__sae_M, (NULL));

    /* states (dwork) */
    (void)memset((void *)ved__sae_DWork, 0, sizeof(D_Work_ved__sae));

    {
        int32_T i;

        /* InitializeConditions for UnitDelay: '<S2>/x_delay' */
        for (i = 0; i < 2; i++) {
            ved__sae_DWork->x_delay_DSTATE[(i)] = ved__sae_x_init_p[(i)];
        }

        /* InitializeConditions for UnitDelay: '<S2>/P_delay' */
        for (i = 0; i < 4; i++) {
            ved__sae_DWork->P_delay_DSTATE[(i)] = ved__sae_P_init_p[(i)];
        }
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
