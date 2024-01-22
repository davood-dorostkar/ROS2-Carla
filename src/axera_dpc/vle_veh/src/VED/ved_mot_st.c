#include "ved_consts.h"

#include "ved_mot_st.h"
#include "ved_mot_st_private.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* QAC Fixes */

/*
 * Output and update for atomic system:
 *    '<S1>/Get_IO_State1'
 *    '<S1>/Get_IO_State2'
 *    '<S1>/Get_IO_State3'
 *    '<S1>/Get_IO_State4'
 *    '<S1>/Get_IO_State8'
 *    '<S33>/Get_IO_State1'
 *    '<S33>/Get_IO_State2'
 *    '<S33>/Get_IO_State3'
 *    '<S33>/Get_IO_State8'
 *    '<S4>/Get_IO_State2'
 * ...
 */
void ved__mot_st_Get_IO_State1(const uint8_T rtu_state_in[32],
                               rtB_Get_IO_State1_ved__mot_st *localB,
                               uint32_T rtp_Filter) {
    /* MultiPortSwitch: '<S16>/Index Vector' incorporates:
     *  Constant: '<S16>/Constant1'
     */
    localB->IndexVector = rtu_state_in[(rtp_Filter)];
}

/*
 * Output and update for action system:
 *    '<S33>/whl_dir'
 *    '<S33>/ALN_dir'
 */
void ved__mot_st_whl_dir(const ved__mot_states_t *rtu_0,
                         ved__mot_states_t *rty_Out1) {
    /* Inport: '<S40>/In1' */
    (*rty_Out1) = (*rtu_0);
}

/*
 * Output and update for atomic system:
 *    '<S59>/get_percentage'
 *    '<S60>/get_percentage'
 */
void ved__mot_st_get_percentage(const uint8 rtu_veh_velo_percentage[15],
                                rtB_get_percentage_ved__mot_st *localB) {
    /* Embedded MATLAB: '<S59>/get_percentage' */
    /* Embedded MATLAB Function 'veh_velo/fwd_p_down/get_percentage': '<S67>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S67>:1:5' */
    localB->y1 = rtu_veh_velo_percentage[0];

    /* '<S67>:1:6' */
    localB->y2 = rtu_veh_velo_percentage[1];

    /* '<S67>:1:7' */
    localB->dT = rtu_veh_velo_percentage[3];
}

/*
 * Output and update for atomic system:
 *    '<S61>/get_percentage'
 *    '<S62>/get_percentage'
 */
void ved__mot_st_get_percentage_h(const uint8 rtu_veh_velo_percentage[15],
                                  rtB_get_percentage_ved__mot_st_n *localB) {
    /* Embedded MATLAB: '<S61>/get_percentage' */
    /* Embedded MATLAB Function 'veh_velo/rws_p_down/get_percentage': '<S71>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S71>:1:5' */
    localB->y1 = rtu_veh_velo_percentage[10];

    /* '<S71>:1:6' */
    localB->y2 = rtu_veh_velo_percentage[11];

    /* '<S71>:1:7' */
    localB->dT = rtu_veh_velo_percentage[13];
}

/*
 * Output and update for atomic system:
 *    '<S63>/get_percentage'
 *    '<S64>/get_percentage'
 */
void ved__mot_st_get_percentage_b(const uint8 rtu_veh_velo_percentage[15],
                                  rtB_get_percentage_ved__mot_st_k *localB) {
    /* Embedded MATLAB: '<S63>/get_percentage' */
    /* Embedded MATLAB Function 'veh_velo/ss_p_dwon/get_percentage': '<S75>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S75>:1:5' */
    localB->y1 = rtu_veh_velo_percentage[5];

    /* '<S75>:1:6' */
    localB->y2 = rtu_veh_velo_percentage[6];

    /* '<S75>:1:7' */
    localB->dT = rtu_veh_velo_percentage[8];
}

/*
 * Output and update for atomic system:
 *    '<S8>/whl_fl_motion_percentage'
 *    '<S8>/whl_fr_motion_percentage'
 *    '<S8>/whl_rl_motion_percentage'
 *    '<S8>/whl_rr_motion_percentage'
 */
void ved__mo_whl_fl_motion_percentage(
    uint8_T rtu_whl_direction,
    uint8_T rtu_whl_direction_valid,
    const uint8 rtu_whl_direction_percentage[12],
    rtB_whl_fl_motion_percentage_vd *localB) {
    /* Embedded MATLAB: '<S8>/whl_fl_motion_percentage' */
    /* Embedded MATLAB Function 'whl_direction/whl_fl_motion_percentage':
     * '<S82>:1' */
    /*  compute the wheel direction quality */
    if (((uint32_T)rtu_whl_direction_valid) == ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S82>:1:3' */
        /* '<S82>:1:4' */
        switch (rtu_whl_direction) {
            case 0U:
                /* unknown */
                /* '<S82>:1:6' */
                localB->fwd_percentage = rtu_whl_direction_percentage[2];

                /* '<S82>:1:7' */
                localB->ss_percentage = rtu_whl_direction_percentage[6];

                /* '<S82>:1:8' */
                localB->rws_percentage = rtu_whl_direction_percentage[10];
                break;

            case 1U:
                /* fwd     */
                /* '<S82>:1:10' */
                localB->fwd_percentage = rtu_whl_direction_percentage[0];

                /* '<S82>:1:11' */
                localB->ss_percentage = rtu_whl_direction_percentage[4];

                /* '<S82>:1:12' */
                localB->rws_percentage = rtu_whl_direction_percentage[8];
                break;

            case 2U:
                /* rws */
                /* '<S82>:1:14' */
                localB->fwd_percentage = rtu_whl_direction_percentage[1];

                /* '<S82>:1:15' */
                localB->ss_percentage = rtu_whl_direction_percentage[5];

                /* '<S82>:1:16' */
                localB->rws_percentage = rtu_whl_direction_percentage[9];
                break;

            default:
                /*  invalid */
                /* '<S82>:1:18' */
                localB->fwd_percentage = rtu_whl_direction_percentage[3];

                /* '<S82>:1:19' */
                localB->ss_percentage = rtu_whl_direction_percentage[7];

                /* '<S82>:1:20' */
                localB->rws_percentage = rtu_whl_direction_percentage[11];
                break;
        }
    } else {
        /*  invalid */
        /* '<S82>:1:23' */
        localB->fwd_percentage = rtu_whl_direction_percentage[3];

        /* '<S82>:1:24' */
        localB->ss_percentage = rtu_whl_direction_percentage[7];

        /* '<S82>:1:25' */
        localB->rws_percentage = rtu_whl_direction_percentage[11];
    }
}

/*
 * Output and update for atomic system:
 *    '<S94>/get_percentage'
 *    '<S101>/get_percentage'
 *    '<S108>/get_percentage'
 *    '<S115>/get_percentage'
 */
void ved__mot_st_get_percentage_i(const uint8 rtu_whl_puls_percentage[15],
                                  rtB_get_percentage_ved__mot_st_m *localB) {
    /* Embedded MATLAB: '<S94>/get_percentage' */
    /* Embedded MATLAB Function
     * 'whl_pulse/whl_puls_fl_percentage/fwd_p_down/get_percentage': '<S98>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S98>:1:5' */
    localB->y1 = rtu_whl_puls_percentage[0];

    /* '<S98>:1:6' */
    localB->y2 = rtu_whl_puls_percentage[1];

    /* '<S98>:1:7' */
    localB->dT = rtu_whl_puls_percentage[2];
}

/*
 * Output and update for atomic system:
 *    '<S95>/get_percentage'
 *    '<S102>/get_percentage'
 *    '<S109>/get_percentage'
 *    '<S116>/get_percentage'
 */
void ved__mot_st_get_percentage_k(const uint8 rtu_whl_puls_percentage[15],
                                  rtB_get_percentage_ved__mot_st_g *localB) {
    /* Embedded MATLAB: '<S95>/get_percentage' */
    /* Embedded MATLAB Function
     * 'whl_pulse/whl_puls_fl_percentage/rws_p_down/get_percentage': '<S99>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S99>:1:5' */
    localB->y1 = rtu_whl_puls_percentage[10];

    /* '<S99>:1:6' */
    localB->y2 = rtu_whl_puls_percentage[11];

    /* '<S99>:1:7' */
    localB->dT = rtu_whl_puls_percentage[12];
}

/*
 * Output and update for atomic system:
 *    '<S96>/get_percentage'
 *    '<S103>/get_percentage'
 *    '<S110>/get_percentage'
 *    '<S117>/get_percentage'
 */
void ved__mot_st_get_percentage_c(const uint8 rtu_whl_puls_percentage[15],
                                  rtB_get_percentage_ved__mot_st_p *localB) {
    /* Embedded MATLAB: '<S96>/get_percentage' */
    /* Embedded MATLAB Function
     * 'whl_pulse/whl_puls_fl_percentage/ss_p_down/get_percentage': '<S100>:1'
     */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S100>:1:5' */
    localB->y1 = rtu_whl_puls_percentage[5];

    /* '<S100>:1:6' */
    localB->y2 = rtu_whl_puls_percentage[6];

    /* '<S100>:1:7' */
    localB->dT = rtu_whl_puls_percentage[7];
}

/*
 * Output and update for atomic system:
 *    '<S90>/whl_puls_fl_percentage'
 *    '<S91>/whl_puls_fl_percentage'
 *    '<S92>/whl_puls_fl_percentage'
 *    '<S93>/whl_puls_fl_percentage'
 */
void ved__mot__whl_puls_fl_percentage(
    uint8_T rtu_diff_whl_puls,
    uint8_T rtu_whl_puls_valid,
    const uint8 rtu_whl_puls_percentage[15],
    uint8_T rtu_cnt_ramp_in,
    uint8_T rtu_fwd_p,
    uint8_T rtu_ss_p,
    uint8_T rtu_rws_p,
    uint8_T rtu_cnt_delay_in,
    rtB_whl_puls_fl_percentage_ved__ *localB) {
    /* Embedded MATLAB: '<S90>/whl_puls_fl_percentage' */
    /* Embedded MATLAB Function
     * 'whl_pulse/whl_puls_fl_percentage/whl_puls_fl_percentage': '<S97>:1' */
    /*  calculate the wheel puls quality */
    if (((uint32_T)rtu_whl_puls_valid) == ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S97>:1:3' */
        int32_T tmp;
        if (rtu_diff_whl_puls != 0) {
            /* '<S97>:1:4' */
            /* '<S97>:1:5' */
            localB->cnt_ramp_out = 0U;

            /* '<S97>:1:6' */
            localB->cnt_delay_out = 0U;

            /* '<S97>:1:7' */
            localB->fwd_percentage = rtu_whl_puls_percentage[0];

            /* '<S97>:1:8' */
            localB->ss_percentage = rtu_whl_puls_percentage[5];

            /* '<S97>:1:9' */
            localB->rws_percentage = rtu_whl_puls_percentage[10];
        } else if (rtu_cnt_delay_in == rtu_whl_puls_percentage[3]) {
            /* '<S97>:1:11' */
            /* '<S97>:1:12' */
            tmp = (int32_T)(((uint32_T)rtu_cnt_ramp_in) + 1U);
            if (((uint32_T)tmp) > 255U) {
                tmp = 255;
            }

            localB->cnt_ramp_out = (uint8_T)tmp;

            /* '<S97>:1:13' */
            localB->cnt_delay_out = rtu_cnt_delay_in;
            if (localB->cnt_ramp_out <= rtu_whl_puls_percentage[2]) {
                /* '<S97>:1:14' */
                /* '<S97>:1:15' */
                localB->fwd_percentage = rtu_fwd_p;

                /* '<S97>:1:16' */
                localB->ss_percentage = rtu_ss_p;

                /* '<S97>:1:17' */
                localB->rws_percentage = rtu_rws_p;
            } else {
                /* '<S97>:1:19' */
                tmp = (int32_T)(((uint32_T)rtu_whl_puls_percentage[2]) + 1U);
                if (((uint32_T)tmp) > 255U) {
                    tmp = 255;
                }

                localB->cnt_ramp_out = (uint8_T)tmp;

                /* '<S97>:1:20' */
                localB->fwd_percentage = rtu_whl_puls_percentage[1];

                /* '<S97>:1:21' */
                localB->ss_percentage = rtu_whl_puls_percentage[6];

                /* '<S97>:1:22' */
                localB->rws_percentage = rtu_whl_puls_percentage[11];
            }
        } else {
            /* '<S97>:1:25' */
            localB->cnt_ramp_out = 0U;

            /* '<S97>:1:26' */
            tmp = (int32_T)(((uint32_T)rtu_cnt_delay_in) + 1U);
            if (((uint32_T)tmp) > 255U) {
                tmp = 255;
            }

            localB->cnt_delay_out = (uint8_T)tmp;

            /* '<S97>:1:27' */
            localB->fwd_percentage = rtu_whl_puls_percentage[0];

            /* '<S97>:1:28' */
            localB->ss_percentage = rtu_whl_puls_percentage[5];

            /* '<S97>:1:29' */
            localB->rws_percentage = rtu_whl_puls_percentage[10];
        }
    } else {
        /* '<S97>:1:33' */
        localB->cnt_ramp_out = 0U;

        /* '<S97>:1:34' */
        localB->cnt_delay_out = 0U;

        /* '<S97>:1:35' */
        localB->fwd_percentage = rtu_whl_puls_percentage[4];

        /* '<S97>:1:36' */
        localB->ss_percentage = rtu_whl_puls_percentage[9];

        /* '<S97>:1:37' */
        localB->rws_percentage = rtu_whl_puls_percentage[4];
    }
}

/* Model step function */
void ved__mot_st_step(BlockIO_ved__mot_st *ved__mot_st_B,
                      D_Work_ved__mot_st *ved__mot_st_DWork,
                      VED_InputData_t *ved__mot_st_U_VED_InputData,
                      VED_InternalData_t *ved__mot_st_U_VED_InternalData_in,
                      VEDALN_Monitoring_t *ved__mot_st_U_VED_ALNData,
                      ved__bayes_mot_states_t *ved__mot_st_Y_ved__mot_st_out) {
    /* local block i/o variables */
    ved__mot_states_t rtb_BusCreator_e;
    ved__mot_states_t rtb_BusCreator_c;
    ved__mot_states_t rtb_Merge;
    uint8_T rtb_WhlTicksDevFrLeft;
    uint8_T rtb_State[32];
    uint8_T rtb_WhlTicksDevFrRight;
    uint8_T rtb_State_p[32];
    uint8_T rtb_WhlTicksDevReLeft;
    uint8_T rtb_State_o[32];
    uint8_T rtb_WhlTicksDevReRight;
    uint8_T rtb_State_op[32];
    uint8_T rtb_cnt_ramp;
    uint8_T rtb_DataTypeConversion;
    uint8_T rtb_DataTypeConversion_f;
    uint8_T rtb_DataTypeConversion_n;
    uint8_T rtb_cnt_delay;
    uint8_T rtb_cnt_ramp_g;
    uint8_T rtb_DataTypeConversion_g;
    uint8_T rtb_DataTypeConversion_gi;
    uint8_T rtb_DataTypeConversion_o;
    uint8_T rtb_cnt_delay_p;
    uint8_T rtb_cnt_ramp_l;
    uint8_T rtb_DataTypeConversion_k;
    uint8_T rtb_DataTypeConversion_h;
    uint8_T rtb_DataTypeConversion_k1;
    uint8_T rtb_cnt_delay_b;
    uint8_T rtb_cnt_ramp_b;
    uint8_T rtb_DataTypeConversion_c;
    uint8_T rtb_DataTypeConversion_ng;
    uint8_T rtb_DataTypeConversion_hd;
    uint8_T rtb_cnt_delay_f;
    uint8_T rtb_WhlDirFrLeft;
    uint8_T rtb_State_f[32];
    uint8_T rtb_WhlDirFrRight;
    uint8_T rtb_State_a[32];
    uint8_T rtb_WhlDirReLeft;
    uint8_T rtb_State_k[32];
    uint8_T rtb_WhlDirReRight;
    uint8_T rtb_State_i[32];
    uint8_T rtb_State_g[32];
    uint8_T rtb_State_b[32];
    uint8_T rtb_State_ak[32];
    uint8_T rtb_State_e[32];
    uint8_T rtb_State_d[32];
    uint8 rtb_fwd;
    uint8 rtb_fwd_n;
    uint8 rtb_fwd_o;
    uint8 rtb_fwd_d;
    uint8 rtb_ss;
    uint8 rtb_ss_p;
    uint8 rtb_ss_g;
    uint8 rtb_ss_c;
    uint8 rtb_rvs;
    uint8 rtb_rvs_o;
    uint8 rtb_rvs_g;
    uint8 rtb_rvs_f;
    uint8 rtb_fwd_e;
    uint8 rtb_fwd_j;
    uint8 rtb_ss_o;
    uint8 rtb_ss_h;
    uint8 rtb_rvs_cw;
    uint8 rtb_rvs_e;
    uint8 rtb_fwd_f;
    uint8 rtb_fwd_b;
    uint8 rtb_fwd_l;
    uint8 rtb_fwd_em;
    uint8 rtb_ss_g0;
    uint8 rtb_ss_i;
    uint8 rtb_ss_l;
    uint8 rtb_ss_ov;
    uint8 rtb_rvs_m;
    uint8 rtb_rvs_oo;
    uint8 rtb_rvs_a;
    uint8 rtb_rvs_jo;
    uint8_T rtb_State_m[32];
    uint8 rtb_fwd_o3;
    uint8 rtb_fwd_m;
    uint8 rtb_ss_gr;
    uint8 rtb_ss_gp;
    uint8 rtb_rvs_ay;
    uint8 rtb_rvs_ge;
    uint8 rtb_fwd_ct;
    uint8 rtb_fwd_ot;
    uint8 rtb_ss_d1;
    uint8 rtb_ss_f;
    uint8 rtb_rvs_ei;
    uint8 rtb_rvs_n;
    uint8 rtb_fwd_k;
    uint8 rtb_fwd_f2;
    uint8 rtb_fwd_kj;
    uint8 rtb_fwd_lv;
    uint8 rtb_ss_fb;
    uint8 rtb_ss_lh;
    uint8 rtb_ss_j;
    uint8 rtb_ss_m;
    uint8 rtb_rvs_h;
    uint8 rtb_rvs_ep;
    uint8 rtb_rvs_jy;
    uint8 rtb_rvs_ca;
    uint8_T cnt_out_down;
    uint8_T cnt_out_up;
    int8_T mot_counter;
    real32_T rtb_mult_ss;
    uint8 rtb_whl_puls_fl_percentage_fwd;
    uint8 rtb_whl_puls_fl_percentage_ss;
    uint8 rtb_whl_puls_fl_percentage_rvs;
    uint8 rtb_whl_puls_fr_percentage_fwd;
    uint8 rtb_whl_puls_fr_percentage_ss;
    uint8 rtb_whl_puls_fr_percentage_rvs;
    uint8 rtb_whl_puls_fl_percentage_i_fw;
    uint8 rtb_whl_puls_fl_percentage_i_ss;
    uint8 rtb_whl_puls_fl_percentage_i_rv;
    uint8 rtb_BusCreator4_rear_right_fwd;
    uint8 rtb_BusCreator4_rear_right_ss;
    uint8 rtb_BusCreator4_rear_right_rvs;
    uint8_T rtb_cnt_delay_down;
    uint8_T rtb_Switch_j;
    uint8_T rtb_cnt_delay_up;
    uint8 rtb_BusCreator_g_fwd;
    uint8 rtb_BusCreator_g_ss;
    uint8 rtb_BusCreator_g_rvs;
    uint8_T rtb_fwd_percentage_a;
    uint8_T rtb_ss_percentage_o;
    uint8_T rtb_rws_percentage_e;
    uint8 rtb_front_left_fwd;
    uint8 rtb_front_left_ss;
    uint8 rtb_front_left_rvs;
    uint8 rtb_front_right_fwd;
    uint8 rtb_front_right_ss;
    uint8 rtb_front_right_rvs;
    uint8 rtb_rear_left_fwd;
    uint8 rtb_rear_left_ss;
    uint8 rtb_rear_left_rvs;
    uint8 rtb_BusCreator4_d_rear_right_fw;
    uint8 rtb_BusCreator4_d_rear_right_ss;
    uint8 rtb_BusCreator4_d_rear_right_rv;
    boolean_T rtb_whl_dir_valid;
    boolean_T rtb_RelationalOperator;
    uint8_T rtb_fwd_percentage_o;
    real32_T rtb_Switch3;
    uint8_T rtb_ss_percentage_k;
    real32_T rtb_Switch3_b;
    uint8_T rtb_rws_percentage_ko;
    uint8_T rtb_ss_percentage_c;
    uint8_T rtb_rws_percentage_ek;
    uint8_T rtb_ss_percentage_l;
    uint8_T rtb_rws_percentage_ic;
    uint8_T rtb_fwd_percentage_n;
    uint8_T rtb_ss_percentage_g;
    uint8_T rtb_rws_percentage_i;
    uint8_T rtb_fwd_percentage_f;
    uint8_T rtb_ss_percentage_m;
    uint8_T rtb_rws_percentage_k;
    real32_T rtb_Sum;
    uint8_T rtb_DataTypeConversion_p;
    real32_T rtb_T2;
    real32_T rtb_T1;
    real32_T rtb_T0;
    real32_T rtb_T4;
    real32_T rtb_T5;
    real32_T rtb_T6;
    uint8_T rtb_ss_percentage;
    uint8_T rtb_rws_percentage;
    uint8 rtb_mot_st_bayes_in_ALN_dir_sta;
    uint8 rtb_mot_st_bayes_in_ALN_dir_s_0;
    uint8 rtb_mot_st_bayes_in_ALN_dir_s_1;
    uint8 rtb_BusCreator_m_fwd;
    uint8 rtb_BusCreator_m_ss;
    uint8 rtb_BusCreator_m_rvs;
    uint8 rtb_BusCreator_p_fwd;
    uint8 rtb_BusCreator_p_ss;
    uint8 rtb_BusCreator_p_rvs;
    uint8 rtb_mot_st_bayes_out_stage_3_fw;
    uint8 rtb_mot_st_bayes_out_stage_3_ss;
    uint8 rtb_mot_st_bayes_out_stage_3_rv;
    uint8 rtb_mot_st_bayes_out_stage_5_fw;
    uint8_T rtb_Switch_f;
    uint8_T rtb_Switch_k;
    uint8_T rtb_Switch;
    int32_T i;
    uint8_T rtb_DataTypeConversion_na_0;
    uint8_T rtb_DataTypeConversion_gy_0;
    uint8_T rtb_DataTypeConversion_cz_0;

    /* BusSelector: '<S9>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlTicksDevFrLeft =
        (*ved__mot_st_U_VED_InputData).Signals.WhlTicksDevFrLeft;
    rtb_WhlTicksDevFrRight =
        (*ved__mot_st_U_VED_InputData).Signals.WhlTicksDevFrRight;
    rtb_WhlTicksDevReLeft =
        (*ved__mot_st_U_VED_InputData).Signals.WhlTicksDevReLeft;
    rtb_WhlTicksDevReRight =
        (*ved__mot_st_U_VED_InputData).Signals.WhlTicksDevReRight;
    for (i = 0; i < 32; i++) {
        rtb_State[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
        rtb_State_p[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
        rtb_State_o[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
        rtb_State_op[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S9>/Get_IO_State1' */
    ved__mot_st_Get_IO_State1(rtb_State, &ved__mot_st_B->Get_IO_State1,
                              ((uint32_T)VED_SIN_POS_WTCKS_FL));

    /* end of Outputs for SubSystem: '<S9>/Get_IO_State1' */

    /* UnitDelay: '<S90>/cnt_ramp' */
    rtb_cnt_ramp = ved__mot_st_DWork->cnt_ramp_DSTATE;

    /* Embedded MATLAB: '<S94>/get_percentage' */
    ved__mot_st_get_percentage_i((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage.y2 -
                            ved__mot_st_B->sf_get_percentage.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage.dT)) *
               ((real32_T)rtb_cnt_ramp)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion = 0U;
        }
    } else {
        rtb_DataTypeConversion = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S96>/get_percentage' */
    ved__mot_st_get_percentage_c((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_c);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_c.y2 -
                            ved__mot_st_B->sf_get_percentage_c.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_c.dT)) *
               ((real32_T)rtb_cnt_ramp)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_c.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_f = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_f = 0U;
        }
    } else {
        rtb_DataTypeConversion_f = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S95>/get_percentage' */
    ved__mot_st_get_percentage_k((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_k);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_k.y2 -
                            ved__mot_st_B->sf_get_percentage_k.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_k.dT)) *
               ((real32_T)rtb_cnt_ramp)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_k.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_n = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_n = 0U;
        }
    } else {
        rtb_DataTypeConversion_n = MAX_uint8_T;
    }

    /* UnitDelay: '<S90>/cnt_delay' */
    rtb_cnt_delay = ved__mot_st_DWork->cnt_delay_DSTATE;

    /* Embedded MATLAB: '<S90>/whl_puls_fl_percentage' */
    ved__mot__whl_puls_fl_percentage(
        rtb_WhlTicksDevFrLeft, ved__mot_st_B->Get_IO_State1.IndexVector,
        (&(ved__whl_puls_to_perc_p[0])), rtb_cnt_ramp, rtb_DataTypeConversion,
        rtb_DataTypeConversion_f, rtb_DataTypeConversion_n, rtb_cnt_delay,
        &ved__mot_st_B->sf_whl_puls_fl_percentage);

    /* BusCreator: '<S90>/Bus Creator' */
    rtb_whl_puls_fl_percentage_fwd =
        ved__mot_st_B->sf_whl_puls_fl_percentage.fwd_percentage;
    rtb_whl_puls_fl_percentage_ss =
        ved__mot_st_B->sf_whl_puls_fl_percentage.ss_percentage;
    rtb_whl_puls_fl_percentage_rvs =
        ved__mot_st_B->sf_whl_puls_fl_percentage.rws_percentage;

    /* Outputs for atomic SubSystem: '<S9>/Get_IO_State2' */
    ved__mot_st_Get_IO_State1(rtb_State_p, &ved__mot_st_B->Get_IO_State2,
                              ((uint32_T)VED_SIN_POS_WTCKS_FR));

    /* end of Outputs for SubSystem: '<S9>/Get_IO_State2' */

    /* UnitDelay: '<S91>/cnt_ramp' */
    rtb_cnt_ramp_g = ved__mot_st_DWork->cnt_ramp_DSTATE_l;

    /* Embedded MATLAB: '<S101>/get_percentage' */
    ved__mot_st_get_percentage_i((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_l);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_l.y2 -
                            ved__mot_st_B->sf_get_percentage_l.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_l.dT)) *
               ((real32_T)rtb_cnt_ramp_g)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_l.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_g = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_g = 0U;
        }
    } else {
        rtb_DataTypeConversion_g = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S103>/get_percentage' */
    ved__mot_st_get_percentage_c((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_f);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_f.y2 -
                            ved__mot_st_B->sf_get_percentage_f.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_f.dT)) *
               ((real32_T)rtb_cnt_ramp_g)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_f.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_gi = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_gi = 0U;
        }
    } else {
        rtb_DataTypeConversion_gi = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S102>/get_percentage' */
    ved__mot_st_get_percentage_k((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_d);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_d.y2 -
                            ved__mot_st_B->sf_get_percentage_d.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_d.dT)) *
               ((real32_T)rtb_cnt_ramp_g)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_d.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_o = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_o = 0U;
        }
    } else {
        rtb_DataTypeConversion_o = MAX_uint8_T;
    }

    /* UnitDelay: '<S91>/cnt_delay' */
    rtb_cnt_delay_p = ved__mot_st_DWork->cnt_delay_DSTATE_j;

    /* Embedded MATLAB: '<S91>/whl_puls_fl_percentage' */
    ved__mot__whl_puls_fl_percentage(
        rtb_WhlTicksDevFrRight, ved__mot_st_B->Get_IO_State2.IndexVector,
        (&(ved__whl_puls_to_perc_p[0])), rtb_cnt_ramp_g,
        rtb_DataTypeConversion_g, rtb_DataTypeConversion_gi,
        rtb_DataTypeConversion_o, rtb_cnt_delay_p,
        &ved__mot_st_B->sf_whl_puls_fl_percentage_o);

    /* BusCreator: '<S91>/Bus Creator' */
    rtb_whl_puls_fr_percentage_fwd =
        ved__mot_st_B->sf_whl_puls_fl_percentage_o.fwd_percentage;
    rtb_whl_puls_fr_percentage_ss =
        ved__mot_st_B->sf_whl_puls_fl_percentage_o.ss_percentage;
    rtb_whl_puls_fr_percentage_rvs =
        ved__mot_st_B->sf_whl_puls_fl_percentage_o.rws_percentage;

    /* Outputs for atomic SubSystem: '<S9>/Get_IO_State3' */
    ved__mot_st_Get_IO_State1(rtb_State_o, &ved__mot_st_B->Get_IO_State3,
                              ((uint32_T)VED_SIN_POS_WTCKS_RL));

    /* end of Outputs for SubSystem: '<S9>/Get_IO_State3' */

    /* UnitDelay: '<S92>/cnt_ramp' */
    rtb_cnt_ramp_l = ved__mot_st_DWork->cnt_ramp_DSTATE_k;

    /* Embedded MATLAB: '<S108>/get_percentage' */
    ved__mot_st_get_percentage_i((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_l2);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_l2.y2 -
                            ved__mot_st_B->sf_get_percentage_l2.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_l2.dT)) *
               ((real32_T)rtb_cnt_ramp_l)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_l2.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_k = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_k = 0U;
        }
    } else {
        rtb_DataTypeConversion_k = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S110>/get_percentage' */
    ved__mot_st_get_percentage_c((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_e);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_e.y2 -
                            ved__mot_st_B->sf_get_percentage_e.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_e.dT)) *
               ((real32_T)rtb_cnt_ramp_l)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_e.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_h = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_h = 0U;
        }
    } else {
        rtb_DataTypeConversion_h = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S109>/get_percentage' */
    ved__mot_st_get_percentage_k((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_d2);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_d2.y2 -
                            ved__mot_st_B->sf_get_percentage_d2.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_d2.dT)) *
               ((real32_T)rtb_cnt_ramp_l)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_d2.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_k1 = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_k1 = 0U;
        }
    } else {
        rtb_DataTypeConversion_k1 = MAX_uint8_T;
    }

    /* UnitDelay: '<S92>/cnt_delay' */
    rtb_cnt_delay_b = ved__mot_st_DWork->cnt_delay_DSTATE_ju;

    /* Embedded MATLAB: '<S92>/whl_puls_fl_percentage' */
    ved__mot__whl_puls_fl_percentage(
        rtb_WhlTicksDevReLeft, ved__mot_st_B->Get_IO_State3.IndexVector,
        (&(ved__whl_puls_to_perc_p[0])), rtb_cnt_ramp_l,
        rtb_DataTypeConversion_k, rtb_DataTypeConversion_h,
        rtb_DataTypeConversion_k1, rtb_cnt_delay_b,
        &ved__mot_st_B->sf_whl_puls_fl_percentage_e);

    /* BusCreator: '<S92>/Bus Creator' */
    rtb_whl_puls_fl_percentage_i_fw =
        ved__mot_st_B->sf_whl_puls_fl_percentage_e.fwd_percentage;
    rtb_whl_puls_fl_percentage_i_ss =
        ved__mot_st_B->sf_whl_puls_fl_percentage_e.ss_percentage;
    rtb_whl_puls_fl_percentage_i_rv =
        ved__mot_st_B->sf_whl_puls_fl_percentage_e.rws_percentage;

    /* Outputs for atomic SubSystem: '<S9>/Get_IO_State8' */
    ved__mot_st_Get_IO_State1(rtb_State_op, &ved__mot_st_B->Get_IO_State8,
                              ((uint32_T)VED_SIN_POS_WTCKS_RR));

    /* end of Outputs for SubSystem: '<S9>/Get_IO_State8' */

    /* UnitDelay: '<S93>/cnt_ramp' */
    rtb_cnt_ramp_b = ved__mot_st_DWork->cnt_ramp_DSTATE_d;

    /* Embedded MATLAB: '<S115>/get_percentage' */
    ved__mot_st_get_percentage_i((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_ln);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_ln.y2 -
                            ved__mot_st_B->sf_get_percentage_ln.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_ln.dT)) *
               ((real32_T)rtb_cnt_ramp_b)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_ln.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_c = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_c = 0U;
        }
    } else {
        rtb_DataTypeConversion_c = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S117>/get_percentage' */
    ved__mot_st_get_percentage_c((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_n);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_n.y2 -
                            ved__mot_st_B->sf_get_percentage_n.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_n.dT)) *
               ((real32_T)rtb_cnt_ramp_b)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_n.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_ng = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_ng = 0U;
        }
    } else {
        rtb_DataTypeConversion_ng = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S116>/get_percentage' */
    ved__mot_st_get_percentage_k((&(ved__whl_puls_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_g);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_g.y2 -
                            ved__mot_st_B->sf_get_percentage_g.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_g.dT)) *
               ((real32_T)rtb_cnt_ramp_b)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_g.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_hd = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_hd = 0U;
        }
    } else {
        rtb_DataTypeConversion_hd = MAX_uint8_T;
    }

    /* UnitDelay: '<S93>/cnt_delay' */
    rtb_cnt_delay_f = ved__mot_st_DWork->cnt_delay_DSTATE_f;

    /* Embedded MATLAB: '<S93>/whl_puls_fl_percentage' */
    ved__mot__whl_puls_fl_percentage(
        rtb_WhlTicksDevReRight, ved__mot_st_B->Get_IO_State8.IndexVector,
        (&(ved__whl_puls_to_perc_p[0])), rtb_cnt_ramp_b,
        rtb_DataTypeConversion_c, rtb_DataTypeConversion_ng,
        rtb_DataTypeConversion_hd, rtb_cnt_delay_f,
        &ved__mot_st_B->sf_whl_puls_fl_percentage_p);

    /* BusCreator: '<S9>/Bus Creator4' incorporates:
     *  BusCreator: '<S93>/Bus Creator'
     */
    rtb_BusCreator4_rear_right_fwd =
        ved__mot_st_B->sf_whl_puls_fl_percentage_p.fwd_percentage;
    rtb_BusCreator4_rear_right_ss =
        ved__mot_st_B->sf_whl_puls_fl_percentage_p.ss_percentage;
    rtb_BusCreator4_rear_right_rvs =
        ved__mot_st_B->sf_whl_puls_fl_percentage_p.rws_percentage;

    /* UnitDelay: '<S7>/cnt_delay_down' */
    rtb_cnt_delay_down = ved__mot_st_DWork->cnt_delay_down_DSTATE;

    /* Sum: '<S66>/FixPt Sum1' incorporates:
     *  Constant: '<S66>/FixPt Constant'
     */
    rtb_Switch_j =
        (uint8_T)(((uint32_T)ved__mot_st_DWork->cnt_delay_down_DSTATE) + 1U);

    /* Embedded MATLAB: '<S59>/get_percentage' */
    ved__mot_st_get_percentage((&(ved__veh_velocity_to_perc_p[0])),
                               &ved__mot_st_B->sf_get_percentage_du);

    rtb_Sum = ((real32_T)ved__mot_st_B->sf_get_percentage_du.y2) -
              ((((real32_T)(ved__mot_st_B->sf_get_percentage_du.y2 -
                            ved__mot_st_B->sf_get_percentage_du.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_du.dT)) *
               ((real32_T)rtb_Switch_j));
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_ss_percentage_c = (uint8_T)rtb_Sum;
        } else {
            rtb_ss_percentage_c = 0U;
        }
    } else {
        rtb_ss_percentage_c = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S63>/get_percentage' */
    ved__mot_st_get_percentage_b((&(ved__veh_velocity_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_b);

    rtb_Sum = ((real32_T)ved__mot_st_B->sf_get_percentage_b.y2) -
              ((((real32_T)(ved__mot_st_B->sf_get_percentage_b.y2 -
                            ved__mot_st_B->sf_get_percentage_b.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_b.dT)) *
               ((real32_T)((uint8_T)(((uint32_T)rtb_cnt_delay_down) + 1U))));
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_rws_percentage_ek = (uint8_T)rtb_Sum;
        } else {
            rtb_rws_percentage_ek = 0U;
        }
    } else {
        rtb_rws_percentage_ek = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S61>/get_percentage' */
    ved__mot_st_get_percentage_h((&(ved__veh_velocity_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_h);

    rtb_Sum = ((real32_T)ved__mot_st_B->sf_get_percentage_h.y2) -
              ((((real32_T)(ved__mot_st_B->sf_get_percentage_h.y2 -
                            ved__mot_st_B->sf_get_percentage_h.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_h.dT)) *
               ((real32_T)((uint8_T)(((uint32_T)rtb_cnt_delay_down) + 1U))));
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_rws_percentage_e = (uint8_T)rtb_Sum;
        } else {
            rtb_rws_percentage_e = 0U;
        }
    } else {
        rtb_rws_percentage_e = MAX_uint8_T;
    }

    /* UnitDelay: '<S7>/cnt_delay_up' */
    rtb_cnt_delay_up = ved__mot_st_DWork->cnt_delay_up_DSTATE;

    /* Sum: '<S68>/FixPt Sum1' incorporates:
     *  Constant: '<S68>/FixPt Constant'
     */
    rtb_Switch_j =
        (uint8_T)(((uint32_T)ved__mot_st_DWork->cnt_delay_up_DSTATE) + 1U);

    /* Embedded MATLAB: '<S60>/get_percentage' */
    ved__mot_st_get_percentage((&(ved__veh_velocity_to_perc_p[0])),
                               &ved__mot_st_B->sf_get_percentage_n0);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_n0.y2 -
                            ved__mot_st_B->sf_get_percentage_n0.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_n0.dT)) *
               ((real32_T)rtb_Switch_j)) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_n0.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_fwd_percentage_a = (uint8_T)rtb_Sum;
        } else {
            rtb_fwd_percentage_a = 0U;
        }
    } else {
        rtb_fwd_percentage_a = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S64>/get_percentage' */
    ved__mot_st_get_percentage_b((&(ved__veh_velocity_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_o);

    rtb_Sum = ((((real32_T)(ved__mot_st_B->sf_get_percentage_o.y2 -
                            ved__mot_st_B->sf_get_percentage_o.y1)) /
                ((real32_T)ved__mot_st_B->sf_get_percentage_o.dT)) *
               ((real32_T)((uint8_T)(((uint32_T)rtb_cnt_delay_up) + 1U)))) +
              ((real32_T)ved__mot_st_B->sf_get_percentage_o.y1);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_ss_percentage_o = (uint8_T)rtb_Sum;
        } else {
            rtb_ss_percentage_o = 0U;
        }
    } else {
        rtb_ss_percentage_o = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S62>/get_percentage' */
    ved__mot_st_get_percentage_h((&(ved__veh_velocity_to_perc_p[0])),
                                 &ved__mot_st_B->sf_get_percentage_br);

    /* Embedded MATLAB Function 'veh_velo/veh_velocity_percentage': '<S65>:1' */
    /*  calculate the vehicle velocity motion state quality */
    if ((*ved__mot_st_U_VED_InternalData_in).ved__ve_out.veh_velo_var <=
        2500.0F) {
        /* '<S65>:1:3' */
        if ((*ved__mot_st_U_VED_InternalData_in).ved__ve_out.veh_velo >=
            (((real32_T)ved__veh_velocity_to_perc_p[2]) / 100.0F)) {
            /* '<S65>:1:4' */
            /* '<S65>:1:5' */
            cnt_out_up = 0U;

            /* '<S65>:1:7' */
            i = (int32_T)(((uint32_T)rtb_cnt_delay_down) + 1U);
            if (((uint32_T)i) > 255U) {
                i = 255;
            }

            cnt_out_down = (uint8_T)i;
            if (cnt_out_down <= ved__veh_velocity_to_perc_p[3]) {
                /* '<S65>:1:8' */
                /* '<S65>:1:9' */
                rtb_fwd_percentage_a = rtb_ss_percentage_c;

                /* '<S65>:1:10' */
                rtb_ss_percentage_o = rtb_rws_percentage_ek;

                /* '<S65>:1:11' */
            } else {
                /* '<S65>:1:13' */
                i = (int32_T)(((uint32_T)ved__veh_velocity_to_perc_p[3]) + 1U);
                if (((uint32_T)i) > 255U) {
                    i = 255;
                }

                cnt_out_down = (uint8_T)i;

                /* '<S65>:1:14' */
                rtb_fwd_percentage_a = ved__veh_velocity_to_perc_p[0];

                /* '<S65>:1:15' */
                rtb_ss_percentage_o = ved__veh_velocity_to_perc_p[5];

                /* '<S65>:1:16' */
                rtb_rws_percentage_e = ved__veh_velocity_to_perc_p[10];
            }
        } else {
            /* '<S65>:1:19' */
            cnt_out_down = 0U;

            /* '<S65>:1:21' */
            i = (int32_T)(((uint32_T)rtb_cnt_delay_up) + 1U);
            if (((uint32_T)i) > 255U) {
                i = 255;
            }

            cnt_out_up = (uint8_T)i;
            if (cnt_out_up <= ved__veh_velocity_to_perc_p[3]) {
                /* '<S65>:1:22' */
                /* '<S65>:1:23' */
                /* '<S65>:1:24' */
                /* '<S65>:1:25' */
                rtb_Sum =
                    ((((real32_T)(ved__mot_st_B->sf_get_percentage_br.y2 -
                                  ved__mot_st_B->sf_get_percentage_br.y1)) /
                      ((real32_T)ved__mot_st_B->sf_get_percentage_br.dT)) *
                     ((real32_T)((uint8_T)(((uint32_T)rtb_cnt_delay_up) +
                                           1U)))) +
                    ((real32_T)ved__mot_st_B->sf_get_percentage_br.y1);
                if (rtb_Sum < 256.0F) {
                    if (rtb_Sum >= 0.0F) {
                        rtb_rws_percentage_e = (uint8_T)rtb_Sum;
                    } else {
                        rtb_rws_percentage_e = 0U;
                    }
                } else {
                    rtb_rws_percentage_e = MAX_uint8_T;
                }
            } else {
                /* '<S65>:1:27' */
                i = (int32_T)(((uint32_T)ved__veh_velocity_to_perc_p[3]) + 2U);
                if (((uint32_T)i) > 255U) {
                    i = 255;
                }

                cnt_out_up = (uint8_T)i;

                /* '<S65>:1:28' */
                rtb_fwd_percentage_a = ved__veh_velocity_to_perc_p[1];

                /* '<S65>:1:29' */
                rtb_ss_percentage_o = ved__veh_velocity_to_perc_p[6];

                /* '<S65>:1:30' */
                rtb_rws_percentage_e = ved__veh_velocity_to_perc_p[11];
            }
        }
    } else {
        /* '<S65>:1:34' */
        cnt_out_down = 0U;

        /* '<S65>:1:35' */
        cnt_out_up = 0U;

        /* '<S65>:1:36' */
        rtb_fwd_percentage_a = ved__veh_velocity_to_perc_p[4];

        /* '<S65>:1:37' */
        rtb_ss_percentage_o = ved__veh_velocity_to_perc_p[9];

        /* '<S65>:1:38' */
        rtb_rws_percentage_e = ved__veh_velocity_to_perc_p[14];
    }

    /* BusSelector: '<S8>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlDirFrLeft = (*ved__mot_st_U_VED_InputData).Signals.WhlDirFrLeft;
    for (i = 0; i < 32; i++) {
        rtb_State_f[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S8>/Get_IO_State5' */
    ved__mot_st_Get_IO_State1(rtb_State_f, &ved__mot_st_B->Get_IO_State5,
                              ((uint32_T)VED_SIN_POS_WDIR_FL));

    /* end of Outputs for SubSystem: '<S8>/Get_IO_State5' */

    /* Embedded MATLAB: '<S8>/whl_fl_motion_percentage' */
    ved__mo_whl_fl_motion_percentage(
        rtb_WhlDirFrLeft, ved__mot_st_B->Get_IO_State5.IndexVector,
        (&(ved__whl_direction_to_perc_p[0])),
        &ved__mot_st_B->sf_whl_fl_motion_percentage);

    /* BusCreator: '<S8>/Bus Creator' */
    rtb_front_left_fwd =
        ved__mot_st_B->sf_whl_fl_motion_percentage.fwd_percentage;
    rtb_front_left_ss =
        ved__mot_st_B->sf_whl_fl_motion_percentage.ss_percentage;
    rtb_front_left_rvs =
        ved__mot_st_B->sf_whl_fl_motion_percentage.rws_percentage;

    /* BusSelector: '<S8>/Bus Selector1' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlDirFrRight = (*ved__mot_st_U_VED_InputData).Signals.WhlDirFrRight;
    for (i = 0; i < 32; i++) {
        rtb_State_a[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S8>/Get_IO_State6' */
    ved__mot_st_Get_IO_State1(rtb_State_a, &ved__mot_st_B->Get_IO_State6,
                              ((uint32_T)VED_SIN_POS_WDIR_FR));

    /* end of Outputs for SubSystem: '<S8>/Get_IO_State6' */

    /* Embedded MATLAB: '<S8>/whl_fr_motion_percentage' */
    ved__mo_whl_fl_motion_percentage(
        rtb_WhlDirFrRight, ved__mot_st_B->Get_IO_State6.IndexVector,
        (&(ved__whl_direction_to_perc_p[0])),
        &ved__mot_st_B->sf_whl_fr_motion_percentage);

    /* BusCreator: '<S8>/Bus Creator1' */
    rtb_front_right_fwd =
        ved__mot_st_B->sf_whl_fr_motion_percentage.fwd_percentage;
    rtb_front_right_ss =
        ved__mot_st_B->sf_whl_fr_motion_percentage.ss_percentage;
    rtb_front_right_rvs =
        ved__mot_st_B->sf_whl_fr_motion_percentage.rws_percentage;

    /* BusSelector: '<S8>/Bus Selector2' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlDirReLeft = (*ved__mot_st_U_VED_InputData).Signals.WhlDirReLeft;
    for (i = 0; i < 32; i++) {
        rtb_State_k[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S8>/Get_IO_State7' */
    ved__mot_st_Get_IO_State1(rtb_State_k, &ved__mot_st_B->Get_IO_State7,
                              ((uint32_T)VED_SIN_POS_WDIR_RL));

    /* end of Outputs for SubSystem: '<S8>/Get_IO_State7' */

    /* Embedded MATLAB: '<S8>/whl_rl_motion_percentage' */
    ved__mo_whl_fl_motion_percentage(
        rtb_WhlDirReLeft, ved__mot_st_B->Get_IO_State7.IndexVector,
        (&(ved__whl_direction_to_perc_p[0])),
        &ved__mot_st_B->sf_whl_rl_motion_percentage);

    /* BusCreator: '<S8>/Bus Creator2' */
    rtb_rear_left_fwd =
        ved__mot_st_B->sf_whl_rl_motion_percentage.fwd_percentage;
    rtb_rear_left_ss = ved__mot_st_B->sf_whl_rl_motion_percentage.ss_percentage;
    rtb_rear_left_rvs =
        ved__mot_st_B->sf_whl_rl_motion_percentage.rws_percentage;

    /* BusSelector: '<S8>/Bus Selector3' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    rtb_WhlDirReRight = (*ved__mot_st_U_VED_InputData).Signals.WhlDirReRight;
    for (i = 0; i < 32; i++) {
        rtb_State_i[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S8>/Get_IO_State8' */
    ved__mot_st_Get_IO_State1(rtb_State_i, &ved__mot_st_B->Get_IO_State8_d,
                              ((uint32_T)VED_SIN_POS_WDIR_RR));

    /* end of Outputs for SubSystem: '<S8>/Get_IO_State8' */

    /* Embedded MATLAB: '<S8>/whl_rr_motion_percentage' */
    ved__mo_whl_fl_motion_percentage(
        rtb_WhlDirReRight, ved__mot_st_B->Get_IO_State8_d.IndexVector,
        (&(ved__whl_direction_to_perc_p[0])),
        &ved__mot_st_B->sf_whl_rr_motion_percentage);

    /* BusCreator: '<S8>/Bus Creator4' incorporates:
     *  BusCreator: '<S8>/Bus Creator3'
     */
    rtb_BusCreator4_d_rear_right_fw =
        ved__mot_st_B->sf_whl_rr_motion_percentage.fwd_percentage;
    rtb_BusCreator4_d_rear_right_ss =
        ved__mot_st_B->sf_whl_rr_motion_percentage.ss_percentage;
    rtb_BusCreator4_d_rear_right_rv =
        ved__mot_st_B->sf_whl_rr_motion_percentage.rws_percentage;

    /* BusSelector: '<S1>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_g[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S1>/Get_IO_State8' */
    ved__mot_st_Get_IO_State1(rtb_State_g, &ved__mot_st_B->Get_IO_State8_f,
                              ((uint32_T)VED_SIN_POS_WDIR_FL));

    /* end of Outputs for SubSystem: '<S1>/Get_IO_State8' */

    /* Outputs for atomic SubSystem: '<S1>/Get_IO_State1' */
    ved__mot_st_Get_IO_State1(rtb_State_g, &ved__mot_st_B->Get_IO_State1_g,
                              ((uint32_T)VED_SIN_POS_WDIR_FR));

    /* end of Outputs for SubSystem: '<S1>/Get_IO_State1' */

    /* Outputs for atomic SubSystem: '<S1>/Get_IO_State3' */
    ved__mot_st_Get_IO_State1(rtb_State_g, &ved__mot_st_B->Get_IO_State3_o,
                              ((uint32_T)VED_SIN_POS_WDIR_RL));

    /* end of Outputs for SubSystem: '<S1>/Get_IO_State3' */

    /* Outputs for atomic SubSystem: '<S1>/Get_IO_State4' */
    ved__mot_st_Get_IO_State1(rtb_State_g, &ved__mot_st_B->Get_IO_State4,
                              ((uint32_T)VED_SIN_POS_WDIR_RR));

    /* end of Outputs for SubSystem: '<S1>/Get_IO_State4' */

    /* Logic: '<S1>/whl_dir_valid' */
    rtb_whl_dir_valid = ((((ved__mot_st_B->Get_IO_State8_f.IndexVector != 0) &&
                           (ved__mot_st_B->Get_IO_State1_g.IndexVector != 0)) &&
                          (ved__mot_st_B->Get_IO_State3_o.IndexVector != 0)) &&
                         (ved__mot_st_B->Get_IO_State4.IndexVector != 0));

    /* Switch: '<S21>/Switch' incorporates:
     *  Inport: '<Root>/VED_ALNData'
     *  UnitDelay: '<S21>/ALN_e_Direction_Prev'
     *  UnitDelay: '<S21>/Init2'
     */
    if (ved__mot_st_DWork->Init2_DSTATE != 0) {
        rtb_Switch_j = (*ved__mot_st_U_VED_ALNData).Direction.e_Direction;
    } else {
        rtb_Switch_j = ved__mot_st_DWork->ALN_e_Direction_Prev_DSTATE;
    }

    /* RelationalOperator: '<S11>/Relational Operator' incorporates:
     *  Inport: '<Root>/VED_ALNData'
     */
    rtb_RelationalOperator =
        (rtb_Switch_j != (*ved__mot_st_U_VED_ALNData).Direction.e_Direction);

    /* Switch: '<S11>/Switch2' incorporates:
     *  Constant: '<S11>/Constant1'
     *  Constant: '<S11>/Constant2'
     *  Sum: '<S11>/fwd_p1'
     *  UnitDelay: '<S11>/FWD_Init1'
     */
    if (rtb_RelationalOperator) {
        rtb_mult_ss = 0.0F;
    } else {
        rtb_mult_ss = 1.0F + ved__mot_st_DWork->FWD_Init1_DSTATE;
    }

    /* Switch: '<S11>/Switch3' incorporates:
     *  Constant: '<S11>/Constant3'
     *  RelationalOperator: '<S11>/Relational Operator1'
     */
    if (rtb_mult_ss > 15.0F) {
        rtb_Switch3 = 15.0F;
    } else {
        rtb_Switch3 = rtb_mult_ss;
    }

    /* BusSelector: '<S1>/Bus Selector2' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_b[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S1>/Get_IO_State2' */
    ved__mot_st_Get_IO_State1(rtb_State_b, &ved__mot_st_B->Get_IO_State2_m,
                              ((uint32_T)VED_SIN_POS_GEAR));

    /* end of Outputs for SubSystem: '<S1>/Get_IO_State2' */

    /* Embedded MATLAB: '<S1>/ALN_direction_percentage2' incorporates:
     *  Constant: '<S1>/ved__ALN_to_perc_p'
     *  Inport: '<Root>/VED_ALNData'
     *  Inport: '<Root>/VED_InputData'
     *  Inport: '<Root>/VED_InternalData_in'
     *  UnitDelay: '<S1>/FWD_Init7'
     *  UnitDelay: '<S1>/RWS_Init5'
     *  UnitDelay: '<S1>/SS_Init6'
     *  UnitDelay: '<S1>/mot_count_delay'
     */
    mot_counter = ved__mot_st_DWork->mot_count_delay_DSTATE;

    /* Embedded MATLAB Function 'ALN_direction/ALN_direction_percentage2':
     * '<S15>:1' */
    /*  compute the direction quality with */
    /*  To go to reverse state, vehicle has to come down to velo (0.0)m/s. */
    /*  max_counter(100) is a limit for fwd/rws condition. */
    /*  mot_counter counts fwd/rws cycles & is incremented/decremented */
    /*  respectively based on ALN direction & veh Velo. It is reset to zero once
     */
    /*  the ALN state is unknown/invalid/sdst. VED mot state is updated
     * according */
    /*  At velo < 0.1m/s & velo > 20kmph, ALN data is not considered. */
    /*  NOTE : Consider velocity variance in future issues as it is removed in
     */
    /*  to mot_counter & veh Velo. */
    /*  current implementation. */
    /*  ALN is providing */
    /* '<S15>:1:15' */
    rtb_fwd_percentage_o = ved__ALN_to_perc_p[2];

    /*  the value is 33 percentage    % Unknown */
    /* '<S15>:1:16' */
    rtb_ss_percentage_k = ved__ALN_to_perc_p[7];

    /*  the value is 34 percentage */
    /* '<S15>:1:17' */
    rtb_rws_percentage_ko = ved__ALN_to_perc_p[12];

    /*  the value is 33 percentage */
    /* '<S15>:1:19' */
    if (((*ved__mot_st_U_VED_InternalData_in).ved__ve_out.veh_velo >= 0.0F) &&
        ((*ved__mot_st_U_VED_InternalData_in).ved__ve_out.veh_velo < 0.1F)) {
        /* '<S15>:1:21' */
        /*  at this condition, vehicle is considered standstill because of
         * negligible velocity. */
        /* '<S15>:1:22' */
        mot_counter = 0;

        /*  sdst */
        /* '<S15>:1:23' */
        rtb_fwd_percentage_o = ved__ALN_to_perc_p[4];

        /*  the value is 2 percentage */
        /* '<S15>:1:24' */
        rtb_ss_percentage_k = ved__ALN_to_perc_p[9];

        /*  the value is 98 percentage */
        /* '<S15>:1:25' */
        rtb_rws_percentage_ko = ved__ALN_to_perc_p[14];

        /*  the value is 2 percentage */
    } else if (((*ved__mot_st_U_VED_InternalData_in).ved__ve_out.veh_velo >=
                0.1F) &&
               ((*ved__mot_st_U_VED_InternalData_in).ved__ve_out.veh_velo <
                5.55555534F)) {
        /* '<S15>:1:26' */
        if ((*ved__mot_st_U_VED_ALNData).Direction.e_Direction == 1) {
            /* '<S15>:1:27' */
            /*  fwd */
            if (ved__mot_st_DWork->mot_count_delay_DSTATE < 100) {
                /* '<S15>:1:28' */
                /* '<S15>:1:29' */
                mot_counter =
                    (int8_T)(ved__mot_st_DWork->mot_count_delay_DSTATE + 1);
            } else {
                /* '<S15>:1:31' */
                mot_counter = 100;

                /*  continue to maintain 100 value */
            }
        } else {
            if ((*ved__mot_st_U_VED_ALNData).Direction.e_Direction == 2) {
                /* '<S15>:1:33' */
                /*  rws */
                if (ved__mot_st_DWork->mot_count_delay_DSTATE > -100) {
                    /* '<S15>:1:34' */
                    /* '<S15>:1:35' */
                    mot_counter =
                        (int8_T)(ved__mot_st_DWork->mot_count_delay_DSTATE - 1);
                } else {
                    /* '<S15>:1:37' */
                    mot_counter = -100;

                    /*  continue to maintain -100 value */
                }
            }
        }

        if ((((uint32_T)ved__mot_st_B->Get_IO_State2_m.IndexVector) ==
             ((uint32_T)VED_IO_STATE_VALID)) &&
            ((((*ved__mot_st_U_VED_InputData).Signals.ActGearPos == 2) &&
              (mot_counter < -4)) ||
             (((*ved__mot_st_U_VED_InputData).Signals.ActGearPos == 3) &&
              (mot_counter > 4)))) {
            /* '<S15>:1:41' */
            if ((*ved__mot_st_U_VED_InputData).Signals.ActGearPos == 2) {
                /* '<S15>:1:42' */
                /* FWD */
                /* '<S15>:1:43' */
                mot_counter = 15;
            }

            if ((*ved__mot_st_U_VED_InputData).Signals.ActGearPos == 3) {
                /* '<S15>:1:46' */
                /* REV */
                /* '<S15>:1:47' */
                mot_counter = -15;
            }
        }

        if (mot_counter > 4) {
            /* '<S15>:1:51' */
            /*  fwd */
            /* '<S15>:1:52' */
            rtb_fwd_percentage_o = ved__ALN_to_perc_p[0];

            /* '<S15>:1:53' */
            rtb_ss_percentage_k = ved__ALN_to_perc_p[5];

            /* '<S15>:1:54' */
            rtb_rws_percentage_ko = ved__ALN_to_perc_p[10];
        } else {
            if (mot_counter < -4) {
                /* '<S15>:1:55' */
                /*  rws */
                /* '<S15>:1:56' */
                rtb_fwd_percentage_o = ved__ALN_to_perc_p[1];

                /* '<S15>:1:57' */
                rtb_ss_percentage_k = ved__ALN_to_perc_p[6];

                /* '<S15>:1:58' */
                rtb_rws_percentage_ko = ved__ALN_to_perc_p[11];
            }
        }
    } else {
        /*  velo > 20kmph and Gear info is valid ,also ALN and Gear info is */
        /*  contradicting ,then give priority to Gear Info also freeze the */
        /*  counter to +5 for FWD motion and -5 for RWS motion. */
        /*  else start considering prev values. And keep the counter at the same
         * value - no reseting. */
        if (((((uint32_T)ved__mot_st_B->Get_IO_State2_m.IndexVector) ==
              ((uint32_T)VED_IO_STATE_VALID)) &&
             ((*ved__mot_st_U_VED_ALNData).Direction.e_Direction == 0)) &&
            ((((*ved__mot_st_U_VED_InputData).Signals.ActGearPos == 2) &&
              (ved__mot_st_DWork->mot_count_delay_DSTATE < -4)) ||
             (((*ved__mot_st_U_VED_InputData).Signals.ActGearPos == 3) &&
              (ved__mot_st_DWork->mot_count_delay_DSTATE > 4)))) {
            /* '<S15>:1:67' */
            if ((*ved__mot_st_U_VED_InputData).Signals.ActGearPos == 2) {
                /* '<S15>:1:68' */
                /* FWD */
                /* '<S15>:1:69' */
                mot_counter = 15;

                /* '<S15>:1:70' */
                rtb_fwd_percentage_o = ved__ALN_to_perc_p[0];

                /* '<S15>:1:71' */
                rtb_ss_percentage_k = ved__ALN_to_perc_p[5];

                /* '<S15>:1:72' */
                rtb_rws_percentage_ko = ved__ALN_to_perc_p[10];
            }

            if ((*ved__mot_st_U_VED_InputData).Signals.ActGearPos == 3) {
                /* '<S15>:1:75' */
                /* REV */
                /* '<S15>:1:76' */
                mot_counter = -15;

                /* '<S15>:1:77' */
                rtb_fwd_percentage_o = ved__ALN_to_perc_p[1];

                /* '<S15>:1:78' */
                rtb_ss_percentage_k = ved__ALN_to_perc_p[6];

                /* '<S15>:1:79' */
                rtb_rws_percentage_ko = ved__ALN_to_perc_p[11];
            }
        } else {
            /* '<S15>:1:82' */
            rtb_fwd_percentage_o = ved__mot_st_DWork->FWD_Init7_DSTATE;

            /* '<S15>:1:83' */
            rtb_ss_percentage_k = ved__mot_st_DWork->SS_Init6_DSTATE;

            /* '<S15>:1:84' */
            rtb_rws_percentage_ko = ved__mot_st_DWork->RWS_Init5_DSTATE;
        }
    }

    /* '<S15>:1:88' */

    /* Switch: '<S11>/Switch1' incorporates:
     *  UnitDelay: '<S1>/FWD_Init1'
     */
    if (rtb_RelationalOperator) {
        rtb_Switch_f = ved__mot_st_DWork->FWD_Init1_DSTATE_l;
    } else {
        /* Switch: '<S22>/Switch' incorporates:
         *  UnitDelay: '<S22>/FWD_Init'
         *  UnitDelay: '<S22>/Init2'
         */
        if (ved__mot_st_DWork->Init2_DSTATE_o != 0) {
            rtb_Switch_f = rtb_fwd_percentage_o;
        } else {
            rtb_Switch_f = ved__mot_st_DWork->FWD_Init_DSTATE;
        }
    }

    rtb_Sum = ((((real32_T)(rtb_fwd_percentage_o - rtb_Switch_f)) / 15.0F) *
               rtb_Switch3) +
              ((real32_T)rtb_Switch_f);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_na_0 = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_na_0 = 0U;
        }
    } else {
        rtb_DataTypeConversion_na_0 = MAX_uint8_T;
    }

    /* Switch: '<S23>/Switch' incorporates:
     *  Inport: '<Root>/VED_ALNData'
     *  UnitDelay: '<S23>/ALN_e_Direction_Prev'
     *  UnitDelay: '<S23>/Init2'
     */
    if (ved__mot_st_DWork->Init2_DSTATE_j != 0) {
        rtb_Switch_j = (*ved__mot_st_U_VED_ALNData).Direction.e_Direction;
    } else {
        rtb_Switch_j = ved__mot_st_DWork->ALN_e_Direction_Prev_DSTATE_p;
    }

    /* RelationalOperator: '<S12>/Relational Operator' incorporates:
     *  Inport: '<Root>/VED_ALNData'
     */
    rtb_RelationalOperator =
        (rtb_Switch_j != (*ved__mot_st_U_VED_ALNData).Direction.e_Direction);

    /* Switch: '<S12>/Switch2' incorporates:
     *  Constant: '<S12>/Constant1'
     *  Constant: '<S12>/Constant2'
     *  Sum: '<S12>/fwd_p1'
     *  UnitDelay: '<S12>/FWD_Init1'
     */
    if (rtb_RelationalOperator) {
        rtb_mult_ss = 0.0F;
    } else {
        rtb_mult_ss = 1.0F + ved__mot_st_DWork->FWD_Init1_DSTATE_g;
    }

    /* Switch: '<S12>/Switch3' incorporates:
     *  Constant: '<S12>/Constant3'
     *  RelationalOperator: '<S12>/Relational Operator1'
     */
    if (rtb_mult_ss > 15.0F) {
        rtb_Switch3_b = 15.0F;
    } else {
        rtb_Switch3_b = rtb_mult_ss;
    }

    /* Switch: '<S12>/Switch1' incorporates:
     *  UnitDelay: '<S1>/FWD_Init3'
     */
    if (rtb_RelationalOperator) {
        rtb_Switch_k = ved__mot_st_DWork->FWD_Init3_DSTATE;
    } else {
        /* Switch: '<S24>/Switch' incorporates:
         *  UnitDelay: '<S24>/FWD_Init'
         *  UnitDelay: '<S24>/Init2'
         */
        if (ved__mot_st_DWork->Init2_DSTATE_i != 0) {
            rtb_Switch_k = rtb_ss_percentage_k;
        } else {
            rtb_Switch_k = ved__mot_st_DWork->FWD_Init_DSTATE_m;
        }
    }

    rtb_Sum = ((((real32_T)(rtb_ss_percentage_k - rtb_Switch_k)) / 15.0F) *
               rtb_Switch3_b) +
              ((real32_T)rtb_Switch_k);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_gy_0 = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_gy_0 = 0U;
        }
    } else {
        rtb_DataTypeConversion_gy_0 = MAX_uint8_T;
    }

    /* Switch: '<S25>/Switch' incorporates:
     *  Inport: '<Root>/VED_ALNData'
     *  UnitDelay: '<S25>/ALN_e_Direction_Prev'
     *  UnitDelay: '<S25>/Init2'
     */
    if (ved__mot_st_DWork->Init2_DSTATE_b != 0) {
        rtb_Switch_j = (*ved__mot_st_U_VED_ALNData).Direction.e_Direction;
    } else {
        rtb_Switch_j = ved__mot_st_DWork->ALN_e_Direction_Prev_DSTATE_h;
    }

    /* RelationalOperator: '<S13>/Relational Operator' incorporates:
     *  Inport: '<Root>/VED_ALNData'
     */
    rtb_RelationalOperator =
        (rtb_Switch_j != (*ved__mot_st_U_VED_ALNData).Direction.e_Direction);

    /* Switch: '<S13>/Switch2' incorporates:
     *  Constant: '<S13>/Constant1'
     *  Constant: '<S13>/Constant2'
     *  Sum: '<S13>/fwd_p1'
     *  UnitDelay: '<S13>/FWD_Init1'
     */
    if (rtb_RelationalOperator) {
        rtb_mult_ss = 0.0F;
    } else {
        rtb_mult_ss = 1.0F + ved__mot_st_DWork->FWD_Init1_DSTATE_i;
    }

    /* Switch: '<S13>/Switch3' incorporates:
     *  Constant: '<S13>/Constant3'
     *  RelationalOperator: '<S13>/Relational Operator1'
     */
    if (rtb_mult_ss > 15.0F) {
        rtb_mult_ss = 15.0F;
    }

    /* Switch: '<S13>/Switch1' incorporates:
     *  UnitDelay: '<S1>/FWD_Init4'
     */
    if (rtb_RelationalOperator) {
        rtb_Switch = ved__mot_st_DWork->FWD_Init4_DSTATE;
    } else {
        /* Switch: '<S26>/Switch' incorporates:
         *  UnitDelay: '<S26>/FWD_Init'
         *  UnitDelay: '<S26>/Init2'
         */
        if (ved__mot_st_DWork->Init2_DSTATE_l != 0) {
            rtb_Switch = rtb_rws_percentage_ko;
        } else {
            rtb_Switch = ved__mot_st_DWork->FWD_Init_DSTATE_p;
        }
    }

    rtb_Sum = ((((real32_T)(rtb_rws_percentage_ko - rtb_Switch)) / 15.0F) *
               rtb_mult_ss) +
              ((real32_T)rtb_Switch);
    if (rtb_Sum < 256.0F) {
        if (rtb_Sum >= 0.0F) {
            rtb_DataTypeConversion_cz_0 = (uint8_T)rtb_Sum;
        } else {
            rtb_DataTypeConversion_cz_0 = 0U;
        }
    } else {
        rtb_DataTypeConversion_cz_0 = MAX_uint8_T;
    }

    /* Embedded MATLAB: '<S1>/ALN_direction_percentage' incorporates:
     *  Constant: '<S1>/ved__ALN_to_perc_p1'
     */
    /* Embedded MATLAB Function 'ALN_direction/ALN_direction_percentage':
     * '<S14>:1' */
    /*  compute the direction quality with  */
    if (rtb_whl_dir_valid) {
        /* '<S14>:1:3' */
        /* '<S14>:1:4' */
        rtb_cnt_delay_down = rtb_DataTypeConversion_na_0;

        /*  the value is 33 percentage                */
        /* '<S14>:1:5' */
        rtb_ss_percentage_c = rtb_DataTypeConversion_gy_0;

        /*  the value is 34 percentage */
        /* '<S14>:1:6' */
        rtb_rws_percentage_ek = rtb_DataTypeConversion_cz_0;

        /*  the value is 33 percentage */
    } else {
        /* '<S14>:1:8' */
        rtb_cnt_delay_down = ved__ALN_to_perc_p[3];

        /*  the value is 10 percentage */
        /* '<S14>:1:9' */
        rtb_ss_percentage_c = ved__ALN_to_perc_p[8];

        /*  the value is 0 percentage */
        /* '<S14>:1:10' */
        rtb_rws_percentage_ek = ved__ALN_to_perc_p[13];

        /*  the value is 0 percentage */
    }

    /* BusCreator: '<S1>/Bus Creator' */
    rtb_BusCreator_e.fwd = rtb_cnt_delay_down;
    rtb_BusCreator_e.ss = rtb_ss_percentage_c;
    rtb_BusCreator_e.rvs = rtb_rws_percentage_ek;

    /* BusSelector: '<S4>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_ak[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S4>/Get_IO_State2' */
    ved__mot_st_Get_IO_State1(rtb_State_ak, &ved__mot_st_B->Get_IO_State2_d,
                              ((uint32_T)VED_SIN_POS_BRAKE));

    /* end of Outputs for SubSystem: '<S4>/Get_IO_State2' */

    /* Embedded MATLAB: '<S45>/get_percentage' incorporates:
     *  Constant: '<S4>/ved__brake_torque_to_perc'
     */
    /* Embedded MATLAB Function
     * 'brake_torque/brake_torque_percentage/fwd_brake_torque_to_percentage/get_percentage':
     * '<S48>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S48>:1:5' */
    /* '<S48>:1:6' */
    /* '<S48>:1:7' */
    /* '<S48>:1:8' */

    /* Embedded MATLAB: '<S47>/get_percentage' incorporates:
     *  Constant: '<S4>/ved__brake_torque_to_perc'
     */
    /* Embedded MATLAB Function
     * 'brake_torque/brake_torque_percentage/ss_brake_torque_to_percentage/get_percentage':
     * '<S50>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S50>:1:5' */
    /* '<S50>:1:6' */
    /* '<S50>:1:7' */
    /* '<S50>:1:8' */

    /* Embedded MATLAB: '<S46>/get_percentage' incorporates:
     *  Constant: '<S4>/ved__brake_torque_to_perc'
     */
    /* Embedded MATLAB Function
     * 'brake_torque/brake_torque_percentage/rws_brake_torque_to_percentage/get_percentage':
     * '<S49>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S49>:1:5' */
    /* '<S49>:1:6' */
    /* '<S49>:1:7' */
    /* '<S49>:1:8' */

    /* Embedded MATLAB Function
     * 'brake_torque/brake_torque_percentage/brake_torque_percentage_fcn':
     * '<S44>:1' */
    /*  compute the brake torque motion state quality */
    if (((uint32_T)ved__mot_st_B->Get_IO_State2_d.IndexVector) ==
        ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S44>:1:3' */
        if ((*ved__mot_st_U_VED_InputData).Signals.BrakeActLevel == 0) {
            /* '<S44>:1:4' */
            /* '<S44>:1:5' */
            rtb_rws_percentage_ek = ved__brake_torque_to_perc_p[0];

            /* '<S44>:1:6' */
            rtb_ss_percentage_l = ved__brake_torque_to_perc_p[7];

            /* '<S44>:1:7' */
            rtb_rws_percentage_ic = ved__brake_torque_to_perc_p[14];
        } else if ((*ved__mot_st_U_VED_InputData).Signals.BrakeActLevel <=
                   ved__brake_torque_to_perc_p[3]) {
            /* '<S44>:1:8' */
            /* '<S44>:1:9' */
            rtb_rws_percentage_ek = ved__brake_torque_to_perc_p[1];

            /* '<S44>:1:10' */
            rtb_ss_percentage_l = ved__brake_torque_to_perc_p[8];

            /* '<S44>:1:11' */
            rtb_rws_percentage_ic = ved__brake_torque_to_perc_p[15];
        } else if ((*ved__mot_st_U_VED_InputData).Signals.BrakeActLevel <=
                   1600) {
            /* '<S44>:1:13' */
            /* '<S44>:1:14' */
            rtb_Sum =
                (((real32_T)(*ved__mot_st_U_VED_InputData)
                      .Signals.BrakeActLevel) /
                 (((real32_T)((((int32_T)(250U *
                                          ((uint32_T)
                                               ved__brake_torque_to_perc_p[4])))
                               << 2) -
                              ved__brake_torque_to_perc_p[3])) /
                  ((real32_T)(ved__brake_torque_to_perc_p[2] -
                              ved__brake_torque_to_perc_p[1])))) +
                ((real32_T)ved__brake_torque_to_perc_p[1]);
            if (rtb_Sum < 256.0F) {
                if (rtb_Sum >= 0.0F) {
                    rtb_rws_percentage_ek = (uint8_T)rtb_Sum;
                } else {
                    rtb_rws_percentage_ek = 0U;
                }
            } else {
                rtb_rws_percentage_ek = MAX_uint8_T;
            }

            /* '<S44>:1:15' */
            rtb_Sum =
                (((real32_T)(*ved__mot_st_U_VED_InputData)
                      .Signals.BrakeActLevel) /
                 (((real32_T)((((int32_T)(250U *
                                          ((uint32_T)ved__brake_torque_to_perc_p
                                               [11])))
                               << 2) -
                              ved__brake_torque_to_perc_p[10])) /
                  ((real32_T)(ved__brake_torque_to_perc_p[9] -
                              ved__brake_torque_to_perc_p[8])))) +
                ((real32_T)ved__brake_torque_to_perc_p[8]);
            if (rtb_Sum < 256.0F) {
                if (rtb_Sum >= 0.0F) {
                    rtb_ss_percentage_l = (uint8_T)rtb_Sum;
                } else {
                    rtb_ss_percentage_l = 0U;
                }
            } else {
                rtb_ss_percentage_l = MAX_uint8_T;
            }

            /* '<S44>:1:16' */
            rtb_Sum =
                (((real32_T)(*ved__mot_st_U_VED_InputData)
                      .Signals.BrakeActLevel) /
                 (((real32_T)((((int32_T)(250U *
                                          ((uint32_T)ved__brake_torque_to_perc_p
                                               [18])))
                               << 2) -
                              ved__brake_torque_to_perc_p[17])) /
                  ((real32_T)(ved__brake_torque_to_perc_p[16] -
                              ved__brake_torque_to_perc_p[15])))) +
                ((real32_T)ved__brake_torque_to_perc_p[15]);
            if (rtb_Sum < 256.0F) {
                if (rtb_Sum >= 0.0F) {
                    rtb_rws_percentage_ic = (uint8_T)rtb_Sum;
                } else {
                    rtb_rws_percentage_ic = 0U;
                }
            } else {
                rtb_rws_percentage_ic = MAX_uint8_T;
            }
        } else {
            /* '<S44>:1:17' */
            if ((*ved__mot_st_U_VED_InternalData_in).ved__ve_out.veh_velo >=
                0.2F) {
                /* '<S44>:1:17' */
                /* '<S44>:1:18' */
                rtb_rws_percentage_ek = ved__brake_torque_to_perc_p[6];

                /* '<S44>:1:19' */
                rtb_ss_percentage_l = ved__brake_torque_to_perc_p[13];

                /* '<S44>:1:20' */
                rtb_rws_percentage_ic = ved__brake_torque_to_perc_p[20];
            } else {
                /* '<S44>:1:22' */
                rtb_rws_percentage_ek = ved__brake_torque_to_perc_p[2];

                /* '<S44>:1:23' */
                rtb_ss_percentage_l = ved__brake_torque_to_perc_p[9];

                /* '<S44>:1:24' */
                rtb_rws_percentage_ic = ved__brake_torque_to_perc_p[16];
            }
        }

        if ((*ved__mot_st_U_VED_InternalData_in).ved__ve_out.veh_velo <= 0.2F) {
            /* '<S44>:1:28' */
            /* '<S44>:1:29' */
            rtb_rws_percentage_ek = ved__brake_torque_to_perc_p[2];

            /* '<S44>:1:30' */
            rtb_ss_percentage_l = ved__brake_torque_to_perc_p[9];

            /* '<S44>:1:31' */
            rtb_rws_percentage_ic = ved__brake_torque_to_perc_p[16];
        }
    } else {
        /* '<S44>:1:35' */
        rtb_rws_percentage_ek = ved__brake_torque_to_perc_p[5];

        /* '<S44>:1:36' */
        rtb_ss_percentage_l = ved__brake_torque_to_perc_p[12];

        /* '<S44>:1:37' */
        rtb_rws_percentage_ic = ved__brake_torque_to_perc_p[19];
    }

    /* BusSelector: '<S5>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_e[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S5>/Get_IO_State2' */
    ved__mot_st_Get_IO_State1(rtb_State_e, &ved__mot_st_B->Get_IO_State2_d0,
                              ((uint32_T)VED_SIN_POS_GEAR));

    /* end of Outputs for SubSystem: '<S5>/Get_IO_State2' */

    /* Outputs for atomic SubSystem: '<S5>/Get_IO_State8' */
    ved__mot_st_Get_IO_State1(rtb_State_e, &ved__mot_st_B->Get_IO_State8_m,
                              ((uint32_T)VED_SIN_POS_WDIR_FL));

    /* end of Outputs for SubSystem: '<S5>/Get_IO_State8' */

    /* Outputs for atomic SubSystem: '<S5>/Get_IO_State1' */
    ved__mot_st_Get_IO_State1(rtb_State_e, &ved__mot_st_B->Get_IO_State1_h,
                              ((uint32_T)VED_SIN_POS_WDIR_FR));

    /* end of Outputs for SubSystem: '<S5>/Get_IO_State1' */

    /* Outputs for atomic SubSystem: '<S5>/Get_IO_State3' */
    ved__mot_st_Get_IO_State1(rtb_State_e, &ved__mot_st_B->Get_IO_State3_b,
                              ((uint32_T)VED_SIN_POS_WDIR_RL));

    /* end of Outputs for SubSystem: '<S5>/Get_IO_State3' */

    /* Outputs for atomic SubSystem: '<S5>/Get_IO_State4' */
    ved__mot_st_Get_IO_State1(rtb_State_e, &ved__mot_st_B->Get_IO_State4_b,
                              ((uint32_T)VED_SIN_POS_WDIR_RR));

    /* end of Outputs for SubSystem: '<S5>/Get_IO_State4' */

    /* Embedded MATLAB: '<S5>/gear_shift_percentage' incorporates:
     *  Constant: '<S5>/ved__gear_shift_to_perc'
     *  Inport: '<Root>/VED_InputData'
     *  Logic: '<S5>/Logical Operator'
     */
    /* Embedded MATLAB Function 'gear_shift/gear_shift_percentage': '<S56>:1' */
    /*  compute the gear shift position motion state quality */
    if ((((ved__mot_st_B->Get_IO_State8_m.IndexVector != 0) &&
          (ved__mot_st_B->Get_IO_State1_h.IndexVector != 0)) &&
         (ved__mot_st_B->Get_IO_State3_b.IndexVector != 0)) &&
        (ved__mot_st_B->Get_IO_State4_b.IndexVector != 0)) {
        /* '<S56>:1:3' */
        if (((uint32_T)ved__mot_st_B->Get_IO_State2_d0.IndexVector) ==
            ((uint32_T)VED_IO_STATE_VALID)) {
            /* '<S56>:1:4' */
            /* '<S56>:1:5' */
            switch ((*ved__mot_st_U_VED_InputData).Signals.ActGearPos) {
                case 0U:
                    /*  N */
                    /* '<S56>:1:7' */
                    rtb_fwd_percentage_n = ved__gear_shift_to_perc_p[1];

                    /* '<S56>:1:8' */
                    rtb_ss_percentage_g = ved__gear_shift_to_perc_p[7];

                    /* '<S56>:1:9' */
                    rtb_rws_percentage_i = ved__gear_shift_to_perc_p[13];
                    break;

                case 1U:
                    /*  P       */
                    /* '<S56>:1:11' */
                    rtb_fwd_percentage_n = ved__gear_shift_to_perc_p[0];

                    /* '<S56>:1:12' */
                    rtb_ss_percentage_g = ved__gear_shift_to_perc_p[6];

                    /* '<S56>:1:13' */
                    rtb_rws_percentage_i = ved__gear_shift_to_perc_p[12];
                    break;

                case 2U:
                    /*  D */
                    /* '<S56>:1:15' */
                    rtb_fwd_percentage_n = ved__gear_shift_to_perc_p[3];

                    /* '<S56>:1:16' */
                    rtb_ss_percentage_g = ved__gear_shift_to_perc_p[9];

                    /* '<S56>:1:17' */
                    rtb_rws_percentage_i = ved__gear_shift_to_perc_p[15];
                    break;

                case 3U:
                    /*  R */
                    /* '<S56>:1:19' */
                    rtb_fwd_percentage_n = ved__gear_shift_to_perc_p[2];

                    /* '<S56>:1:20' */
                    rtb_ss_percentage_g = ved__gear_shift_to_perc_p[8];

                    /* '<S56>:1:21' */
                    rtb_rws_percentage_i = ved__gear_shift_to_perc_p[14];
                    break;

                default:
                    /* '<S56>:1:23' */
                    rtb_fwd_percentage_n = ved__gear_shift_to_perc_p[4];

                    /* '<S56>:1:24' */
                    rtb_ss_percentage_g = ved__gear_shift_to_perc_p[10];

                    /* '<S56>:1:25' */
                    rtb_rws_percentage_i = ved__gear_shift_to_perc_p[16];
                    break;
            }
        } else {
            /* '<S56>:1:28' */
            rtb_fwd_percentage_n = ved__gear_shift_to_perc_p[4];

            /* '<S56>:1:29' */
            rtb_ss_percentage_g = ved__gear_shift_to_perc_p[10];

            /* '<S56>:1:30' */
            rtb_rws_percentage_i = ved__gear_shift_to_perc_p[16];
        }
    } else {
        /* '<S56>:1:33' */
        rtb_fwd_percentage_n = ved__gear_shift_to_perc_p[5];

        /* '<S56>:1:34' */
        rtb_ss_percentage_g = ved__gear_shift_to_perc_p[11];

        /* '<S56>:1:35' */
        rtb_rws_percentage_i = ved__gear_shift_to_perc_p[17];
    }

    /* BusSelector: '<S6>/Bus Selector' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_d[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S6>/Get_IO_State2' */
    ved__mot_st_Get_IO_State1(rtb_State_d, &ved__mot_st_B->Get_IO_State2_e,
                              ((uint32_T)VED_SIN_POS_PBRK));

    /* end of Outputs for SubSystem: '<S6>/Get_IO_State2' */

    /* Embedded MATLAB: '<S6>/parking_break_percentage' incorporates:
     *  Constant: '<S6>/ved__parking_break_to_perc'
     *  Inport: '<Root>/VED_InputData'
     */
    /* Embedded MATLAB Function 'parking_brake/parking_break_percentage':
     * '<S58>:1' */
    /*  compute the parking brake state motion state quality */
    if (((uint32_T)ved__mot_st_B->Get_IO_State2_e.IndexVector) ==
        ((uint32_T)VED_IO_STATE_VALID)) {
        /* '<S58>:1:3' */
        /* '<S58>:1:4' */
        switch ((*ved__mot_st_U_VED_InputData).Signals.ParkBrakeState) {
            case 0U:
                /*  brake is off */
                /* '<S58>:1:6' */
                rtb_fwd_percentage_f = ved__parking_break_to_perc_p[0];

                /* '<S58>:1:7' */
                rtb_ss_percentage_m = ved__parking_break_to_perc_p[3];

                /* '<S58>:1:8' */
                rtb_rws_percentage_k = ved__parking_break_to_perc_p[6];
                break;

            case 1U:
                /*  brake is on      */
                /* '<S58>:1:10' */
                rtb_fwd_percentage_f = ved__parking_break_to_perc_p[1];

                /* '<S58>:1:11' */
                rtb_ss_percentage_m = ved__parking_break_to_perc_p[4];

                /* '<S58>:1:12' */
                rtb_rws_percentage_k = ved__parking_break_to_perc_p[7];
                break;

            default:
                /* '<S58>:1:14' */
                rtb_fwd_percentage_f = ved__parking_break_to_perc_p[2];

                /* '<S58>:1:15' */
                rtb_ss_percentage_m = ved__parking_break_to_perc_p[5];

                /* '<S58>:1:16' */
                rtb_rws_percentage_k = ved__parking_break_to_perc_p[8];
                break;
        }
    } else {
        /* '<S58>:1:19' */
        rtb_fwd_percentage_f = ved__parking_break_to_perc_p[2];

        /* '<S58>:1:20' */
        rtb_ss_percentage_m = ved__parking_break_to_perc_p[5];

        /* '<S58>:1:21' */
        rtb_rws_percentage_k = ved__parking_break_to_perc_p[8];
    }

    /* Embedded MATLAB: '<S10>/threshold_yaw_rate' incorporates:
     *  Abs: '<S10>/Abs'
     *  Inport: '<Root>/VED_InternalData_in'
     *  Sum: '<S122>/Diff'
     *  UnitDelay: '<S122>/UD'
     */
    /* Embedded MATLAB Function 'yaw_rate/threshold_yaw_rate': '<S125>:1' */
    /*  if the abs diff yaw rate is below the threshold output is zero */
    /* '<S125>:1:3' */
    i = 1;
    if (((real32_T)fabs(
            (*ved__mot_st_U_VED_InternalData_in).ved__ye_out.veh_yaw_rate -
            ved__mot_st_DWork->UD_DSTATE_k)) <=
        (((real32_T)ved__yaw_rate_to_perc_p[0]) * 1.0E-7F)) {
        /* '<S125>:1:4' */
        /* '<S125>:1:5' */
        i = 0;
    }

    /* Sum: '<S123>/Sum' incorporates:
     *  Gain: '<S123>/Gain'
     *  Sum: '<S123>/Diff'
     *  UnitDelay: '<S123>/UD'
     */
    rtb_Sum = ((ved__mot_st_DWork->UD_DSTATE - ((real32_T)i)) * 0.935F) +
              ((real32_T)i);

    rtb_DataTypeConversion_p = (uint8_T)(rtb_Sum * 50.0F);

    /* UnitDelay: '<S124>/T2' */
    rtb_T2 = ved__mot_st_DWork->T2_DSTATE;

    /* UnitDelay: '<S124>/T1' */
    rtb_T1 = ved__mot_st_DWork->T1_DSTATE;

    /* UnitDelay: '<S124>/T0' */
    rtb_T0 = ved__mot_st_DWork->T0_DSTATE;

    /* UnitDelay: '<S124>/T4' */
    rtb_T4 = ved__mot_st_DWork->T4_DSTATE;

    /* UnitDelay: '<S124>/T5' */
    rtb_T5 = ved__mot_st_DWork->T5_DSTATE;

    /* UnitDelay: '<S124>/T6' */
    rtb_T6 = ved__mot_st_DWork->T6_DSTATE;

    /* Embedded MATLAB Function 'yaw_rate/yaw_rate_percentage': '<S126>:1' */
    /*  compute the yaw rate motion state quality */
    if (((real32_T)fabs((((((((ved__mot_st_DWork->T3_DSTATE +
                               ved__mot_st_DWork->T2_DSTATE) +
                              ved__mot_st_DWork->T1_DSTATE) +
                             ved__mot_st_DWork->T0_DSTATE) +
                            ved__mot_st_DWork->T4_DSTATE) +
                           ved__mot_st_DWork->T5_DSTATE) +
                          ved__mot_st_DWork->T6_DSTATE) +
                         (*ved__mot_st_U_VED_InternalData_in)
                             .ved__ye_out.veh_yaw_rate_var) *
                        0.125F)) >= 1.0E-10F) {
        /* '<S126>:1:3' */
        /* '<S126>:1:4' */
        /* '<S126>:1:5' */
        rtb_ss_percentage = (uint8_T)((1.0F - rtb_Sum) * 100.0F);

        /* '<S126>:1:6' */
        rtb_rws_percentage = rtb_DataTypeConversion_p;
    } else {
        /* '<S126>:1:8' */
        rtb_DataTypeConversion_p = ved__yaw_rate_to_perc_p[1];

        /* '<S126>:1:9' */
        rtb_ss_percentage = ved__yaw_rate_to_perc_p[3];

        /* '<S126>:1:10' */
        rtb_rws_percentage = ved__yaw_rate_to_perc_p[5];
    }

    /* BusCreator: '<Root>/Bus Creator' */
    rtb_mot_st_bayes_in_ALN_dir_sta = rtb_BusCreator_e.fwd;
    rtb_mot_st_bayes_in_ALN_dir_s_0 = rtb_BusCreator_e.ss;
    rtb_mot_st_bayes_in_ALN_dir_s_1 = rtb_BusCreator_e.rvs;

    /* BusSelector: '<S32>/Bus Selector1' */
    rtb_fwd = rtb_whl_puls_fl_percentage_fwd;
    rtb_fwd_n = rtb_whl_puls_fr_percentage_fwd;
    rtb_fwd_o = rtb_whl_puls_fl_percentage_i_fw;
    rtb_fwd_d = rtb_BusCreator4_rear_right_fwd;

    /* BusSelector: '<S32>/Bus Selector' */
    rtb_ss = rtb_whl_puls_fl_percentage_ss;
    rtb_ss_p = rtb_whl_puls_fr_percentage_ss;
    rtb_ss_g = rtb_whl_puls_fl_percentage_i_ss;
    rtb_ss_c = rtb_BusCreator4_rear_right_ss;

    /* BusSelector: '<S32>/Bus Selector2' */
    rtb_rvs = rtb_whl_puls_fl_percentage_rvs;
    rtb_rvs_o = rtb_whl_puls_fr_percentage_rvs;
    rtb_rvs_g = rtb_whl_puls_fl_percentage_i_rv;
    rtb_rvs_f = rtb_BusCreator4_rear_right_rvs;

    /* BusCreator: '<S32>/Bus Creator' incorporates:
     *  BusSelector: '<S32>/Bus Selector'
     *  BusSelector: '<S32>/Bus Selector1'
     *  BusSelector: '<S32>/Bus Selector2'
     *  Constant: '<S32>/ved__cpt_whl_pulse'
     *  S-Function (ex_sfun_bayes4): '<S32>/fwd_puls_bayes'
     *  S-Function (ex_sfun_bayes4): '<S32>/rws_puls_bayes'
     *  S-Function (ex_sfun_bayes4): '<S32>/ss_puls_bayes'
     */
    rtb_BusCreator_m_fwd = ((uint8_T)VED__MAT_PROB_BAYES4(
        (uint8_T)rtb_fwd, (uint8_T)rtb_fwd_n, (uint8_T)rtb_fwd_o,
        (uint8_T)rtb_fwd_d, (uint8_T *)(&(ved__cpt_whl_pulse_p[0]))));
    rtb_BusCreator_m_ss = ((uint8_T)VED__MAT_PROB_BAYES4(
        (uint8_T)rtb_ss, (uint8_T)rtb_ss_p, (uint8_T)rtb_ss_g,
        (uint8_T)rtb_ss_c, (uint8_T *)(&(ved__cpt_whl_pulse_p[0]))));
    rtb_BusCreator_m_rvs = ((uint8_T)VED__MAT_PROB_BAYES4(
        (uint8_T)rtb_rvs, (uint8_T)rtb_rvs_o, (uint8_T)rtb_rvs_g,
        (uint8_T)rtb_rvs_f, (uint8_T *)(&(ved__cpt_whl_pulse_p[0]))));

    /* BusSelector: '<S29>/Bus Selector1' incorporates:
     *  BusSelector: '<S32>/Bus Selector1'
     *  Constant: '<S32>/ved__cpt_whl_pulse'
     *  S-Function (ex_sfun_bayes4): '<S32>/fwd_puls_bayes'
     */
    rtb_fwd_e = ((uint8_T)VED__MAT_PROB_BAYES4(
        (uint8_T)rtb_fwd, (uint8_T)rtb_fwd_n, (uint8_T)rtb_fwd_o,
        (uint8_T)rtb_fwd_d, (uint8_T *)(&(ved__cpt_whl_pulse_p[0]))));

    /* BusSelector: '<S29>/Bus Selector4' incorporates:
     *  BusCreator: '<S7>/Bus Creator'
     */
    rtb_fwd_j = rtb_fwd_percentage_a;

    /* BusSelector: '<S29>/Bus Selector' */
    rtb_ss_o = rtb_BusCreator_m_ss;

    /* BusSelector: '<S29>/Bus Selector3' incorporates:
     *  BusCreator: '<S7>/Bus Creator'
     */
    rtb_ss_h = rtb_ss_percentage_o;

    /* BusSelector: '<S29>/Bus Selector2' */
    rtb_rvs_cw = rtb_BusCreator_m_rvs;

    /* BusSelector: '<S29>/Bus Selector5' incorporates:
     *  BusCreator: '<S7>/Bus Creator'
     */
    rtb_rvs_e = rtb_rws_percentage_e;

    /* BusCreator: '<S29>/Bus Creator' incorporates:
     *  BusSelector: '<S29>/Bus Selector'
     *  BusSelector: '<S29>/Bus Selector1'
     *  BusSelector: '<S29>/Bus Selector2'
     *  BusSelector: '<S29>/Bus Selector3'
     *  BusSelector: '<S29>/Bus Selector4'
     *  BusSelector: '<S29>/Bus Selector5'
     *  Constant: '<S29>/ved__cpt_puls_velo'
     *  S-Function (ex_sfun_bayes2): '<S29>/fwd_direction_bayes'
     *  S-Function (ex_sfun_bayes2): '<S29>/rws_direction_bayes'
     *  S-Function (ex_sfun_bayes2): '<S29>/ss_direction_bayes'
     */
    rtb_BusCreator_g_fwd =
        ((uint8_T)VED__MAT_PROB_BAYES2((uint8)rtb_fwd_e, (uint8)rtb_fwd_j,
                                       (uint8 *)(&(ved__cpt_pulse_velo_p[0]))));
    rtb_BusCreator_g_ss =
        ((uint8_T)VED__MAT_PROB_BAYES2((uint8)rtb_ss_o, (uint8)rtb_ss_h,
                                       (uint8 *)(&(ved__cpt_pulse_velo_p[0]))));
    rtb_BusCreator_g_rvs =
        ((uint8_T)VED__MAT_PROB_BAYES2((uint8)rtb_rvs_cw, (uint8)rtb_rvs_e,
                                       (uint8 *)(&(ved__cpt_pulse_velo_p[0]))));

    /* BusSelector: '<S31>/Bus Selector1' */
    rtb_fwd_f = rtb_front_left_fwd;
    rtb_fwd_b = rtb_front_right_fwd;
    rtb_fwd_l = rtb_rear_left_fwd;
    rtb_fwd_em = rtb_BusCreator4_d_rear_right_fw;

    /* BusSelector: '<S31>/Bus Selector' */
    rtb_ss_g0 = rtb_front_left_ss;
    rtb_ss_i = rtb_front_right_ss;
    rtb_ss_l = rtb_rear_left_ss;
    rtb_ss_ov = rtb_BusCreator4_d_rear_right_ss;

    /* BusSelector: '<S31>/Bus Selector2' */
    rtb_rvs_m = rtb_front_left_rvs;
    rtb_rvs_oo = rtb_front_right_rvs;
    rtb_rvs_a = rtb_rear_left_rvs;
    rtb_rvs_jo = rtb_BusCreator4_d_rear_right_rv;

    /* BusCreator: '<S31>/Bus Creator' incorporates:
     *  BusSelector: '<S31>/Bus Selector'
     *  BusSelector: '<S31>/Bus Selector1'
     *  BusSelector: '<S31>/Bus Selector2'
     *  Constant: '<S31>/ved__cpt_whl_pulse'
     *  S-Function (ex_sfun_bayes4): '<S31>/fwd_direction_bayes'
     *  S-Function (ex_sfun_bayes4): '<S31>/rws_direction_bayes'
     *  S-Function (ex_sfun_bayes4): '<S31>/ss_direction_bayes'
     */
    rtb_BusCreator_c.fwd = ((uint8_T)VED__MAT_PROB_BAYES4(
        (uint8_T)rtb_fwd_f, (uint8_T)rtb_fwd_b, (uint8_T)rtb_fwd_l,
        (uint8_T)rtb_fwd_em, (uint8_T *)(&(ved__cpt_whl_pulse_p[0]))));
    rtb_BusCreator_c.ss = ((uint8_T)VED__MAT_PROB_BAYES4(
        (uint8_T)rtb_ss_g0, (uint8_T)rtb_ss_i, (uint8_T)rtb_ss_l,
        (uint8_T)rtb_ss_ov, (uint8_T *)(&(ved__cpt_whl_pulse_p[0]))));
    rtb_BusCreator_c.rvs = ((uint8_T)VED__MAT_PROB_BAYES4(
        (uint8_T)rtb_rvs_m, (uint8_T)rtb_rvs_oo, (uint8_T)rtb_rvs_a,
        (uint8_T)rtb_rvs_jo, (uint8_T *)(&(ved__cpt_whl_pulse_p[0]))));

    /* BusSelector: '<S33>/Bus Selector5' incorporates:
     *  Inport: '<Root>/VED_InputData'
     */
    for (i = 0; i < 32; i++) {
        rtb_State_m[i] = (*ved__mot_st_U_VED_InputData).Signals.State[(i)];
    }

    /* Outputs for atomic SubSystem: '<S33>/Get_IO_State8' */
    ved__mot_st_Get_IO_State1(rtb_State_m, &ved__mot_st_B->Get_IO_State8_b,
                              ((uint32_T)VED_SIN_POS_WDIR_FL));

    /* end of Outputs for SubSystem: '<S33>/Get_IO_State8' */

    /* Outputs for atomic SubSystem: '<S33>/Get_IO_State1' */
    ved__mot_st_Get_IO_State1(rtb_State_m, &ved__mot_st_B->Get_IO_State1_k,
                              ((uint32_T)VED_SIN_POS_WDIR_FR));

    /* end of Outputs for SubSystem: '<S33>/Get_IO_State1' */

    /* Outputs for atomic SubSystem: '<S33>/Get_IO_State2' */
    ved__mot_st_Get_IO_State1(rtb_State_m, &ved__mot_st_B->Get_IO_State2_m1,
                              ((uint32_T)VED_SIN_POS_WDIR_RL));

    /* end of Outputs for SubSystem: '<S33>/Get_IO_State2' */

    /* Outputs for atomic SubSystem: '<S33>/Get_IO_State3' */
    ved__mot_st_Get_IO_State1(rtb_State_m, &ved__mot_st_B->Get_IO_State3_e,
                              ((uint32_T)VED_SIN_POS_WDIR_RR));

    /* end of Outputs for SubSystem: '<S33>/Get_IO_State3' */

    /* If: '<S33>/If' incorporates:
     *  ActionPort: '<S35>/Action Port'
     *  ActionPort: '<S40>/Action Port'
     *  Logic: '<S33>/whl_dir_valid'
     *  SubSystem: '<S33>/ALN_dir'
     *  SubSystem: '<S33>/whl_dir'
     */
    if (!((((ved__mot_st_B->Get_IO_State8_b.IndexVector != 0) &&
            (ved__mot_st_B->Get_IO_State1_k.IndexVector != 0)) &&
           (ved__mot_st_B->Get_IO_State2_m1.IndexVector != 0)) &&
          (ved__mot_st_B->Get_IO_State3_e.IndexVector != 0))) {
        ved__mot_st_whl_dir(&rtb_BusCreator_c, &rtb_Merge);
    } else {
        ved__mot_st_whl_dir(&rtb_BusCreator_e, &rtb_Merge);
    }

    /* BusSelector: '<S30>/Bus Selector1' */
    rtb_fwd_o3 = rtb_BusCreator_g_fwd;

    /* BusSelector: '<S30>/Bus Selector4' */
    rtb_fwd_m = rtb_Merge.fwd;

    /* BusSelector: '<S30>/Bus Selector' */
    rtb_ss_gr = rtb_BusCreator_g_ss;

    /* BusSelector: '<S30>/Bus Selector3' */
    rtb_ss_gp = rtb_Merge.ss;

    /* BusSelector: '<S30>/Bus Selector2' */
    rtb_rvs_ay = rtb_BusCreator_g_rvs;

    /* BusSelector: '<S30>/Bus Selector5' */
    rtb_rvs_ge = rtb_Merge.rvs;

    /* BusCreator: '<S30>/Bus Creator' incorporates:
     *  BusSelector: '<S30>/Bus Selector'
     *  BusSelector: '<S30>/Bus Selector1'
     *  BusSelector: '<S30>/Bus Selector2'
     *  BusSelector: '<S30>/Bus Selector3'
     *  BusSelector: '<S30>/Bus Selector4'
     *  BusSelector: '<S30>/Bus Selector5'
     *  Constant: '<S30>/ved__cpt_puls_velo_dir'
     *  S-Function (ex_sfun_bayes2): '<S30>/fwd_bayes'
     *  S-Function (ex_sfun_bayes2): '<S30>/rws_bayes'
     *  S-Function (ex_sfun_bayes2): '<S30>/ss_bayes'
     */
    rtb_BusCreator_p_fwd = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_fwd_o3, (uint8)rtb_fwd_m,
        (uint8 *)(&(ved__cpt_pulse_velo_dir_p[0]))));
    rtb_BusCreator_p_ss = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_ss_gr, (uint8)rtb_ss_gp,
        (uint8 *)(&(ved__cpt_pulse_velo_dir_p[0]))));
    rtb_BusCreator_p_rvs = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_rvs_ay, (uint8)rtb_rvs_ge,
        (uint8 *)(&(ved__cpt_pulse_velo_dir_p[0]))));

    /* BusSelector: '<S27>/Bus Selector1' incorporates:
     *  BusSelector: '<S30>/Bus Selector1'
     *  BusSelector: '<S30>/Bus Selector4'
     *  Constant: '<S30>/ved__cpt_puls_velo_dir'
     *  S-Function (ex_sfun_bayes2): '<S30>/fwd_bayes'
     */
    rtb_fwd_ct = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_fwd_o3, (uint8)rtb_fwd_m,
        (uint8 *)(&(ved__cpt_pulse_velo_dir_p[0]))));

    /* BusSelector: '<S27>/Bus Selector4' incorporates:
     *  BusCreator: '<S43>/Bus Creator'
     */
    rtb_fwd_ot = rtb_rws_percentage_ek;

    /* BusSelector: '<S27>/Bus Selector' */
    rtb_ss_d1 = rtb_BusCreator_p_ss;

    /* BusSelector: '<S27>/Bus Selector3' incorporates:
     *  BusCreator: '<S43>/Bus Creator'
     */
    rtb_ss_f = rtb_ss_percentage_l;

    /* BusSelector: '<S27>/Bus Selector2' */
    rtb_rvs_ei = rtb_BusCreator_p_rvs;

    /* BusSelector: '<S27>/Bus Selector5' incorporates:
     *  BusCreator: '<S43>/Bus Creator'
     */
    rtb_rvs_n = rtb_rws_percentage_ic;

    /* BusCreator: '<S27>/Bus Creator' incorporates:
     *  BusSelector: '<S27>/Bus Selector'
     *  BusSelector: '<S27>/Bus Selector2'
     *  BusSelector: '<S27>/Bus Selector3'
     *  BusSelector: '<S27>/Bus Selector5'
     *  Constant: '<S27>/ved__cpt_pre_beak_torque'
     *  S-Function (ex_sfun_bayes2): '<S27>/rws_bayes'
     *  S-Function (ex_sfun_bayes2): '<S27>/ss_bayes'
     */
    rtb_cnt_delay_down = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_ss_d1, (uint8)rtb_ss_f,
        (uint8 *)(&(ved__cpt_pre_break_torque_p[0]))));
    rtb_cnt_delay_up = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_rvs_ei, (uint8)rtb_rvs_n,
        (uint8 *)(&(ved__cpt_pre_break_torque_p[0]))));

    /* BusCreator: '<S3>/Bus Creator' incorporates:
     *  BusSelector: '<S27>/Bus Selector'
     *  BusSelector: '<S27>/Bus Selector1'
     *  BusSelector: '<S27>/Bus Selector2'
     *  BusSelector: '<S27>/Bus Selector3'
     *  BusSelector: '<S27>/Bus Selector4'
     *  BusSelector: '<S27>/Bus Selector5'
     *  Constant: '<S27>/ved__cpt_pre_beak_torque'
     *  S-Function (ex_sfun_bayes2): '<S27>/fwd_bayes'
     *  S-Function (ex_sfun_bayes2): '<S27>/rws_bayes'
     *  S-Function (ex_sfun_bayes2): '<S27>/ss_bayes'
     */
    rtb_mot_st_bayes_out_stage_3_fw = rtb_Merge.fwd;
    rtb_mot_st_bayes_out_stage_3_ss = rtb_Merge.ss;
    rtb_mot_st_bayes_out_stage_3_rv = rtb_Merge.rvs;
    rtb_mot_st_bayes_out_stage_5_fw = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_fwd_ct, (uint8)rtb_fwd_ot,
        (uint8 *)(&(ved__cpt_pre_break_torque_p[0]))));
    rtb_Switch_j = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_ss_d1, (uint8)rtb_ss_f,
        (uint8 *)(&(ved__cpt_pre_break_torque_p[0]))));
    rtb_ss_percentage_c = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_rvs_ei, (uint8)rtb_rvs_n,
        (uint8 *)(&(ved__cpt_pre_break_torque_p[0]))));

    /* BusSelector: '<S28>/Bus Selector1' incorporates:
     *  BusSelector: '<S27>/Bus Selector1'
     *  BusSelector: '<S27>/Bus Selector4'
     *  Constant: '<S27>/ved__cpt_pre_beak_torque'
     *  S-Function (ex_sfun_bayes2): '<S27>/fwd_bayes'
     */
    rtb_fwd_k = ((uint8_T)VED__MAT_PROB_BAYES2(
        (uint8)rtb_fwd_ct, (uint8)rtb_fwd_ot,
        (uint8 *)(&(ved__cpt_pre_break_torque_p[0]))));

    /* BusSelector: '<S28>/Bus Selector4' incorporates:
     *  BusCreator: '<S5>/Bus Creator'
     */
    rtb_fwd_f2 = rtb_fwd_percentage_n;

    /* BusSelector: '<S28>/Bus Selector8' incorporates:
     *  BusCreator: '<S6>/Bus Creator'
     */
    rtb_fwd_kj = rtb_fwd_percentage_f;

    /* BusSelector: '<S28>/Bus Selector9' incorporates:
     *  BusCreator: '<S10>/Bus Creator'
     */
    rtb_fwd_lv = rtb_DataTypeConversion_p;

    /* BusSelector: '<S28>/Bus Selector' */
    rtb_ss_fb = rtb_cnt_delay_down;

    /* BusSelector: '<S28>/Bus Selector3' incorporates:
     *  BusCreator: '<S5>/Bus Creator'
     */
    rtb_ss_lh = rtb_ss_percentage_g;

    /* BusSelector: '<S28>/Bus Selector7' incorporates:
     *  BusCreator: '<S6>/Bus Creator'
     */
    rtb_ss_j = rtb_ss_percentage_m;

    /* BusSelector: '<S28>/Bus Selector10' incorporates:
     *  BusCreator: '<S10>/Bus Creator'
     */
    rtb_ss_m = rtb_ss_percentage;

    /* BusSelector: '<S28>/Bus Selector2' */
    rtb_rvs_h = rtb_cnt_delay_up;

    /* BusSelector: '<S28>/Bus Selector5' incorporates:
     *  BusCreator: '<S5>/Bus Creator'
     */
    rtb_rvs_ep = rtb_rws_percentage_i;

    /* BusSelector: '<S28>/Bus Selector6' incorporates:
     *  BusCreator: '<S6>/Bus Creator'
     */
    rtb_rvs_jy = rtb_rws_percentage_k;

    /* BusSelector: '<S28>/Bus Selector11' incorporates:
     *  BusCreator: '<S10>/Bus Creator'
     */
    rtb_rvs_ca = rtb_rws_percentage;

    /* Embedded MATLAB: '<S34>/state_decider' incorporates:
     *  BusCreator: '<S28>/Bus Creator'
     *  BusSelector: '<S28>/Bus Selector'
     *  BusSelector: '<S28>/Bus Selector1'
     *  BusSelector: '<S28>/Bus Selector10'
     *  BusSelector: '<S28>/Bus Selector11'
     *  BusSelector: '<S28>/Bus Selector2'
     *  BusSelector: '<S28>/Bus Selector3'
     *  BusSelector: '<S28>/Bus Selector4'
     *  BusSelector: '<S28>/Bus Selector5'
     *  BusSelector: '<S28>/Bus Selector6'
     *  BusSelector: '<S28>/Bus Selector7'
     *  BusSelector: '<S28>/Bus Selector8'
     *  BusSelector: '<S28>/Bus Selector9'
     *  Constant: '<S28>/ved__cpt_pre_gear_parking'
     *  S-Function (ex_sfun_bayes4): '<S28>/fwd_bayes'
     *  S-Function (ex_sfun_bayes4): '<S28>/rws_bayes'
     *  S-Function (ex_sfun_bayes4): '<S28>/ss_bayes'
     */
    /* Embedded MATLAB Function 'bayes_net/state_decider/state_decider':
     * '<S41>:1' */
    /*  decide if the vehicle motion state moving(forward or reverse), forward,
     * reverse or stand still */
    if ((((uint8_T)VED__MAT_PROB_BAYES4(
             (uint8_T)rtb_fwd_k, (uint8_T)rtb_fwd_f2, (uint8_T)rtb_fwd_kj,
             (uint8_T)rtb_fwd_lv,
             (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))) >=
         ((uint8_T)VED__MAT_PROB_BAYES4(
             (uint8_T)rtb_ss_fb, (uint8_T)rtb_ss_lh, (uint8_T)rtb_ss_j,
             (uint8_T)rtb_ss_m,
             (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))))) &&
        (((uint8_T)VED__MAT_PROB_BAYES4(
             (uint8_T)rtb_fwd_k, (uint8_T)rtb_fwd_f2, (uint8_T)rtb_fwd_kj,
             (uint8_T)rtb_fwd_lv,
             (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))) >
         ((uint8_T)VED__MAT_PROB_BAYES4(
             (uint8_T)rtb_rvs_h, (uint8_T)rtb_rvs_ep, (uint8_T)rtb_rvs_jy,
             (uint8_T)rtb_rvs_ca,
             (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))))) {
        /* '<S41>:1:3' */
        /*  if the quality for forward is geater or equal to stand still and
         * reverse is less then forward */
        /*  the motion state is forward */
        /* '<S41>:1:6' */
        rtb_cnt_delay_down = ((uint8_T)VED_LONG_MOT_STATE_MOVE_FWD);

        /* uint8(1); */
        /* '<S41>:1:7' */
        rtb_cnt_delay_up = ((uint8_T)VED__MAT_PROB_BAYES4(
            (uint8_T)rtb_fwd_k, (uint8_T)rtb_fwd_f2, (uint8_T)rtb_fwd_kj,
            (uint8_T)rtb_fwd_lv,
            (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))));
    } else if ((((uint8_T)VED__MAT_PROB_BAYES4(
                    (uint8_T)rtb_ss_fb, (uint8_T)rtb_ss_lh, (uint8_T)rtb_ss_j,
                    (uint8_T)rtb_ss_m,
                    (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))) >
                ((uint8_T)VED__MAT_PROB_BAYES4(
                    (uint8_T)rtb_fwd_k, (uint8_T)rtb_fwd_f2,
                    (uint8_T)rtb_fwd_kj, (uint8_T)rtb_fwd_lv,
                    (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))))) &&
               (((uint8_T)VED__MAT_PROB_BAYES4(
                    (uint8_T)rtb_ss_fb, (uint8_T)rtb_ss_lh, (uint8_T)rtb_ss_j,
                    (uint8_T)rtb_ss_m,
                    (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))) >
                ((uint8_T)VED__MAT_PROB_BAYES4(
                    (uint8_T)rtb_rvs_h, (uint8_T)rtb_rvs_ep,
                    (uint8_T)rtb_rvs_jy, (uint8_T)rtb_rvs_ca,
                    (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))))) {
        /* '<S41>:1:8' */
        /*  elseif the quality for stand still is greater as forward and reverse
         */
        /*  the motion state is stand still */
        /* '<S41>:1:11' */
        rtb_cnt_delay_down = ((uint8_T)VED_LONG_MOT_STATE_STDST);

        /*  uint8(0); */
        /* '<S41>:1:12' */
        rtb_cnt_delay_up = ((uint8_T)VED__MAT_PROB_BAYES4(
            (uint8_T)rtb_ss_fb, (uint8_T)rtb_ss_lh, (uint8_T)rtb_ss_j,
            (uint8_T)rtb_ss_m, (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))));
    } else if ((((uint8_T)VED__MAT_PROB_BAYES4(
                    (uint8_T)rtb_rvs_h, (uint8_T)rtb_rvs_ep,
                    (uint8_T)rtb_rvs_jy, (uint8_T)rtb_rvs_ca,
                    (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))) >
                ((uint8_T)VED__MAT_PROB_BAYES4(
                    (uint8_T)rtb_fwd_k, (uint8_T)rtb_fwd_f2,
                    (uint8_T)rtb_fwd_kj, (uint8_T)rtb_fwd_lv,
                    (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))))) &&
               (((uint8_T)VED__MAT_PROB_BAYES4(
                    (uint8_T)rtb_rvs_h, (uint8_T)rtb_rvs_ep,
                    (uint8_T)rtb_rvs_jy, (uint8_T)rtb_rvs_ca,
                    (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))) >=
                ((uint8_T)VED__MAT_PROB_BAYES4(
                    (uint8_T)rtb_ss_fb, (uint8_T)rtb_ss_lh, (uint8_T)rtb_ss_j,
                    (uint8_T)rtb_ss_m,
                    (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))))) {
        /* '<S41>:1:13' */
        /*  elseif the quality for reverse is greater or equal to stand still
         * and forward is less then reverse */
        /*  the motion state is reverse */
        /* '<S41>:1:16' */
        rtb_cnt_delay_down = ((uint8_T)VED_LONG_MOT_STATE_MOVE_RWD);

        /* uint8(-1); */
        /* '<S41>:1:17' */
        rtb_cnt_delay_up = ((uint8_T)VED__MAT_PROB_BAYES4(
            (uint8_T)rtb_rvs_h, (uint8_T)rtb_rvs_ep, (uint8_T)rtb_rvs_jy,
            (uint8_T)rtb_rvs_ca,
            (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))));
    } else if (((uint8_T)VED__MAT_PROB_BAYES4(
                   (uint8_T)rtb_rvs_h, (uint8_T)rtb_rvs_ep, (uint8_T)rtb_rvs_jy,
                   (uint8_T)rtb_rvs_ca,
                   (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0])))) ==
               ((uint8_T)VED__MAT_PROB_BAYES4(
                   (uint8_T)rtb_fwd_k, (uint8_T)rtb_fwd_f2, (uint8_T)rtb_fwd_kj,
                   (uint8_T)rtb_fwd_lv,
                   (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))))) {
        /* '<S41>:1:18' */
        /*  elseif the quality of reverse is equal to forward */
        /*  the motion state is moving */
        /* '<S41>:1:21' */
        rtb_cnt_delay_down = ((uint8_T)VED_LONG_MOT_STATE_MOVE);

        /* uint8(0.5); */
        /* '<S41>:1:22' */
        rtb_cnt_delay_up = ((uint8_T)VED__MAT_PROB_BAYES4(
            (uint8_T)rtb_fwd_k, (uint8_T)rtb_fwd_f2, (uint8_T)rtb_fwd_kj,
            (uint8_T)rtb_fwd_lv,
            (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))));
    } else {
        /* '<S41>:1:24' */
        rtb_cnt_delay_down = ((uint8_T)VED_LONG_MOT_STATE_MOVE);

        /* '<S41>:1:25' */
        rtb_cnt_delay_up = 0U;
    }

    /* Outport: '<Root>/ved__mot_st_out' incorporates:
     *  BusCreator: '<Root>/Bus Creator1'
     *  BusCreator: '<S10>/Bus Creator'
     *  BusCreator: '<S28>/Bus Creator'
     *  BusCreator: '<S34>/Bus Creator'
     *  BusCreator: '<S43>/Bus Creator'
     *  BusCreator: '<S5>/Bus Creator'
     *  BusCreator: '<S6>/Bus Creator'
     *  BusCreator: '<S7>/Bus Creator'
     *  BusSelector: '<S28>/Bus Selector'
     *  BusSelector: '<S28>/Bus Selector1'
     *  BusSelector: '<S28>/Bus Selector10'
     *  BusSelector: '<S28>/Bus Selector11'
     *  BusSelector: '<S28>/Bus Selector2'
     *  BusSelector: '<S28>/Bus Selector3'
     *  BusSelector: '<S28>/Bus Selector4'
     *  BusSelector: '<S28>/Bus Selector5'
     *  BusSelector: '<S28>/Bus Selector6'
     *  BusSelector: '<S28>/Bus Selector7'
     *  BusSelector: '<S28>/Bus Selector8'
     *  BusSelector: '<S28>/Bus Selector9'
     *  Constant: '<S28>/ved__cpt_pre_gear_parking'
     *  S-Function (ex_sfun_bayes4): '<S28>/fwd_bayes'
     *  S-Function (ex_sfun_bayes4): '<S28>/rws_bayes'
     *  S-Function (ex_sfun_bayes4): '<S28>/ss_bayes'
     */
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.front_left.fwd =
        rtb_whl_puls_fl_percentage_fwd;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.front_left.ss =
        rtb_whl_puls_fl_percentage_ss;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.front_left.rvs =
        rtb_whl_puls_fl_percentage_rvs;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.front_right.fwd =
        rtb_whl_puls_fr_percentage_fwd;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.front_right.ss =
        rtb_whl_puls_fr_percentage_ss;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.front_right.rvs =
        rtb_whl_puls_fr_percentage_rvs;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.rear_left.fwd =
        rtb_whl_puls_fl_percentage_i_fw;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.rear_left.ss =
        rtb_whl_puls_fl_percentage_i_ss;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.rear_left.rvs =
        rtb_whl_puls_fl_percentage_i_rv;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.rear_right.fwd =
        rtb_BusCreator4_rear_right_fwd;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.rear_right.ss =
        rtb_BusCreator4_rear_right_ss;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_puls_states.rear_right.rvs =
        rtb_BusCreator4_rear_right_rvs;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.veh_velocity_state.fwd =
        rtb_fwd_percentage_a;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.veh_velocity_state.ss =
        rtb_ss_percentage_o;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.veh_velocity_state.rvs =
        rtb_rws_percentage_e;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.front_left.fwd = rtb_front_left_fwd;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.front_left.ss = rtb_front_left_ss;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.front_left.rvs = rtb_front_left_rvs;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.front_right.fwd = rtb_front_right_fwd;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.front_right.ss = rtb_front_right_ss;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.front_right.rvs = rtb_front_right_rvs;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.rear_left.fwd = rtb_rear_left_fwd;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.rear_left.ss = rtb_rear_left_ss;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.rear_left.rvs = rtb_rear_left_rvs;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.rear_right.fwd =
        rtb_BusCreator4_d_rear_right_fw;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.rear_right.ss =
        rtb_BusCreator4_d_rear_right_ss;
    (*ved__mot_st_Y_ved__mot_st_out)
        .mot_st_bayes_in.whl_dir_states.rear_right.rvs =
        rtb_BusCreator4_d_rear_right_rv;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.ALN_dir_states.fwd =
        rtb_mot_st_bayes_in_ALN_dir_sta;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.ALN_dir_states.ss =
        rtb_mot_st_bayes_in_ALN_dir_s_0;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.ALN_dir_states.rvs =
        rtb_mot_st_bayes_in_ALN_dir_s_1;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.brake_torque_state.fwd =
        rtb_rws_percentage_ek;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.brake_torque_state.ss =
        rtb_ss_percentage_l;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.brake_torque_state.rvs =
        rtb_rws_percentage_ic;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.gear_shift_state.fwd =
        rtb_fwd_percentage_n;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.gear_shift_state.ss =
        rtb_ss_percentage_g;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.gear_shift_state.rvs =
        rtb_rws_percentage_i;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.park_brake_state.fwd =
        rtb_fwd_percentage_f;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.park_brake_state.ss =
        rtb_ss_percentage_m;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.park_brake_state.rvs =
        rtb_rws_percentage_k;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.yaw_rate_state.fwd =
        rtb_DataTypeConversion_p;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.yaw_rate_state.ss =
        rtb_ss_percentage;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_in.yaw_rate_state.rvs =
        rtb_rws_percentage;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_1.fwd =
        rtb_BusCreator_m_fwd;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_1.ss =
        rtb_BusCreator_m_ss;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_1.rvs =
        rtb_BusCreator_m_rvs;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_2.fwd =
        rtb_BusCreator_g_fwd;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_2.ss =
        rtb_BusCreator_g_ss;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_2.rvs =
        rtb_BusCreator_g_rvs;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_3.fwd =
        rtb_mot_st_bayes_out_stage_3_fw;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_3.ss =
        rtb_mot_st_bayes_out_stage_3_ss;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_3.rvs =
        rtb_mot_st_bayes_out_stage_3_rv;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_4.fwd =
        rtb_BusCreator_p_fwd;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_4.ss =
        rtb_BusCreator_p_ss;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_4.rvs =
        rtb_BusCreator_p_rvs;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_5.fwd =
        rtb_mot_st_bayes_out_stage_5_fw;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_5.ss = rtb_Switch_j;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_bayes_out.stage_5.rvs =
        rtb_ss_percentage_c;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_out.fwd =
        ((uint8_T)VED__MAT_PROB_BAYES4(
            (uint8_T)rtb_fwd_k, (uint8_T)rtb_fwd_f2, (uint8_T)rtb_fwd_kj,
            (uint8_T)rtb_fwd_lv,
            (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))));
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_out.ss =
        ((uint8_T)VED__MAT_PROB_BAYES4(
            (uint8_T)rtb_ss_fb, (uint8_T)rtb_ss_lh, (uint8_T)rtb_ss_j,
            (uint8_T)rtb_ss_m, (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))));
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_out.rvs =
        ((uint8_T)VED__MAT_PROB_BAYES4(
            (uint8_T)rtb_rvs_h, (uint8_T)rtb_rvs_ep, (uint8_T)rtb_rvs_jy,
            (uint8_T)rtb_rvs_ca,
            (uint8_T *)(&(ved__cpt_pre_gear_parking_p[0]))));
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_out.mot_state = rtb_cnt_delay_down;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_out.mot_quality = rtb_cnt_delay_up;
    (*ved__mot_st_Y_ved__mot_st_out).mot_st_out.mot_counter = mot_counter;

    /* Update for UnitDelay: '<S90>/cnt_ramp' */
    ved__mot_st_DWork->cnt_ramp_DSTATE =
        ved__mot_st_B->sf_whl_puls_fl_percentage.cnt_ramp_out;

    /* Update for UnitDelay: '<S90>/cnt_delay' */
    ved__mot_st_DWork->cnt_delay_DSTATE =
        ved__mot_st_B->sf_whl_puls_fl_percentage.cnt_delay_out;

    /* Update for UnitDelay: '<S91>/cnt_ramp' */
    ved__mot_st_DWork->cnt_ramp_DSTATE_l =
        ved__mot_st_B->sf_whl_puls_fl_percentage_o.cnt_ramp_out;

    /* Update for UnitDelay: '<S91>/cnt_delay' */
    ved__mot_st_DWork->cnt_delay_DSTATE_j =
        ved__mot_st_B->sf_whl_puls_fl_percentage_o.cnt_delay_out;

    /* Update for UnitDelay: '<S92>/cnt_ramp' */
    ved__mot_st_DWork->cnt_ramp_DSTATE_k =
        ved__mot_st_B->sf_whl_puls_fl_percentage_e.cnt_ramp_out;

    /* Update for UnitDelay: '<S92>/cnt_delay' */
    ved__mot_st_DWork->cnt_delay_DSTATE_ju =
        ved__mot_st_B->sf_whl_puls_fl_percentage_e.cnt_delay_out;

    /* Update for UnitDelay: '<S93>/cnt_ramp' */
    ved__mot_st_DWork->cnt_ramp_DSTATE_d =
        ved__mot_st_B->sf_whl_puls_fl_percentage_p.cnt_ramp_out;

    /* Update for UnitDelay: '<S93>/cnt_delay' */
    ved__mot_st_DWork->cnt_delay_DSTATE_f =
        ved__mot_st_B->sf_whl_puls_fl_percentage_p.cnt_delay_out;

    /* Update for UnitDelay: '<S7>/cnt_delay_down' */
    ved__mot_st_DWork->cnt_delay_down_DSTATE = cnt_out_down;

    /* Update for UnitDelay: '<S7>/cnt_delay_up' */
    ved__mot_st_DWork->cnt_delay_up_DSTATE = cnt_out_up;

    /* Update for UnitDelay: '<S21>/Init2' incorporates:
     *  Constant: '<S21>/Constant'
     */
    ved__mot_st_DWork->Init2_DSTATE = 0U;

    /* Update for UnitDelay: '<S21>/ALN_e_Direction_Prev' incorporates:
     *  Update for Inport: '<Root>/VED_ALNData'
     */
    ved__mot_st_DWork->ALN_e_Direction_Prev_DSTATE =
        (*ved__mot_st_U_VED_ALNData).Direction.e_Direction;

    /* Update for UnitDelay: '<S11>/FWD_Init1' */
    ved__mot_st_DWork->FWD_Init1_DSTATE = rtb_Switch3;

    /* Update for UnitDelay: '<S1>/FWD_Init7' */
    ved__mot_st_DWork->FWD_Init7_DSTATE = rtb_fwd_percentage_o;

    /* Update for UnitDelay: '<S1>/SS_Init6' */
    ved__mot_st_DWork->SS_Init6_DSTATE = rtb_ss_percentage_k;

    /* Update for UnitDelay: '<S1>/RWS_Init5' */
    ved__mot_st_DWork->RWS_Init5_DSTATE = rtb_rws_percentage_ko;

    /* Update for UnitDelay: '<S1>/mot_count_delay' */
    ved__mot_st_DWork->mot_count_delay_DSTATE = mot_counter;

    /* Update for UnitDelay: '<S1>/FWD_Init1' */
    ved__mot_st_DWork->FWD_Init1_DSTATE_l = rtb_DataTypeConversion_na_0;

    /* Update for UnitDelay: '<S22>/Init2' incorporates:
     *  Constant: '<S22>/Constant'
     */
    ved__mot_st_DWork->Init2_DSTATE_o = 0U;

    /* Update for UnitDelay: '<S22>/FWD_Init' */
    ved__mot_st_DWork->FWD_Init_DSTATE = rtb_Switch_f;

    /* Update for UnitDelay: '<S23>/Init2' incorporates:
     *  Constant: '<S23>/Constant'
     */
    ved__mot_st_DWork->Init2_DSTATE_j = 0U;

    /* Update for UnitDelay: '<S23>/ALN_e_Direction_Prev' incorporates:
     *  Update for Inport: '<Root>/VED_ALNData'
     */
    ved__mot_st_DWork->ALN_e_Direction_Prev_DSTATE_p =
        (*ved__mot_st_U_VED_ALNData).Direction.e_Direction;

    /* Update for UnitDelay: '<S12>/FWD_Init1' */
    ved__mot_st_DWork->FWD_Init1_DSTATE_g = rtb_Switch3_b;

    /* Update for UnitDelay: '<S1>/FWD_Init3' */
    ved__mot_st_DWork->FWD_Init3_DSTATE = rtb_DataTypeConversion_gy_0;

    /* Update for UnitDelay: '<S24>/Init2' incorporates:
     *  Constant: '<S24>/Constant'
     */
    ved__mot_st_DWork->Init2_DSTATE_i = 0U;

    /* Update for UnitDelay: '<S24>/FWD_Init' */
    ved__mot_st_DWork->FWD_Init_DSTATE_m = rtb_Switch_k;

    /* Update for UnitDelay: '<S25>/Init2' incorporates:
     *  Constant: '<S25>/Constant'
     */
    ved__mot_st_DWork->Init2_DSTATE_b = 0U;

    /* Update for UnitDelay: '<S25>/ALN_e_Direction_Prev' incorporates:
     *  Update for Inport: '<Root>/VED_ALNData'
     */
    ved__mot_st_DWork->ALN_e_Direction_Prev_DSTATE_h =
        (*ved__mot_st_U_VED_ALNData).Direction.e_Direction;

    /* Update for UnitDelay: '<S13>/FWD_Init1' */
    ved__mot_st_DWork->FWD_Init1_DSTATE_i = rtb_mult_ss;

    /* Update for UnitDelay: '<S1>/FWD_Init4' */
    ved__mot_st_DWork->FWD_Init4_DSTATE = rtb_DataTypeConversion_cz_0;

    /* Update for UnitDelay: '<S26>/Init2' incorporates:
     *  Constant: '<S26>/Constant'
     */
    ved__mot_st_DWork->Init2_DSTATE_l = 0U;

    /* Update for UnitDelay: '<S26>/FWD_Init' */
    ved__mot_st_DWork->FWD_Init_DSTATE_p = rtb_Switch;

    /* Update for UnitDelay: '<S123>/UD' */
    ved__mot_st_DWork->UD_DSTATE = rtb_Sum;

    /* Update for UnitDelay: '<S122>/UD' incorporates:
     *  Update for Inport: '<Root>/VED_InternalData_in'
     */
    ved__mot_st_DWork->UD_DSTATE_k =
        (*ved__mot_st_U_VED_InternalData_in).ved__ye_out.veh_yaw_rate;

    /* Update for UnitDelay: '<S124>/T3' */
    ved__mot_st_DWork->T3_DSTATE = rtb_T2;

    /* Update for UnitDelay: '<S124>/T2' */
    ved__mot_st_DWork->T2_DSTATE = rtb_T1;

    /* Update for UnitDelay: '<S124>/T1' */
    ved__mot_st_DWork->T1_DSTATE = rtb_T0;

    /* Update for UnitDelay: '<S124>/T0' */
    ved__mot_st_DWork->T0_DSTATE = rtb_T4;

    /* Update for UnitDelay: '<S124>/T4' */
    ved__mot_st_DWork->T4_DSTATE = rtb_T5;

    /* Update for UnitDelay: '<S124>/T5' */
    ved__mot_st_DWork->T5_DSTATE = rtb_T6;

    /* Update for UnitDelay: '<S124>/T6' incorporates:
     *  Update for Inport: '<Root>/VED_InternalData_in'
     */
    ved__mot_st_DWork->T6_DSTATE =
        (*ved__mot_st_U_VED_InternalData_in).ved__ye_out.veh_yaw_rate_var;
}

/* Model initialize function */
void ved__mot_st_initialize(boolean_T firstTime,
                            RT_MODEL_ved__mot_st *const ved__mot_st_M,
                            BlockIO_ved__mot_st *ved__mot_st_B,
                            D_Work_ved__mot_st *ved__mot_st_DWork) {
    (void)firstTime;

    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(ved__mot_st_M, (NULL));

    /* block I/O */
    (void)memset(((void *)ved__mot_st_B), 0, sizeof(BlockIO_ved__mot_st));

    /* states (dwork) */
    (void)memset((void *)ved__mot_st_DWork, 0, sizeof(D_Work_ved__mot_st));

    /* InitializeConditions for UnitDelay: '<S21>/Init2' */
    ved__mot_st_DWork->Init2_DSTATE = 1U;

    /* InitializeConditions for UnitDelay: '<S1>/FWD_Init7' */
    ved__mot_st_DWork->FWD_Init7_DSTATE = 33U;

    /* InitializeConditions for UnitDelay: '<S1>/SS_Init6' */
    ved__mot_st_DWork->SS_Init6_DSTATE = 33U;

    /* InitializeConditions for UnitDelay: '<S1>/RWS_Init5' */
    ved__mot_st_DWork->RWS_Init5_DSTATE = 33U;

    /* InitializeConditions for UnitDelay: '<S1>/FWD_Init1' */
    ved__mot_st_DWork->FWD_Init1_DSTATE_l = 33U;

    /* InitializeConditions for UnitDelay: '<S22>/Init2' */
    ved__mot_st_DWork->Init2_DSTATE_o = 1U;

    /* InitializeConditions for UnitDelay: '<S22>/FWD_Init' */
    ved__mot_st_DWork->FWD_Init_DSTATE = 33U;

    /* InitializeConditions for UnitDelay: '<S23>/Init2' */
    ved__mot_st_DWork->Init2_DSTATE_j = 1U;

    /* InitializeConditions for UnitDelay: '<S1>/FWD_Init3' */
    ved__mot_st_DWork->FWD_Init3_DSTATE = 33U;

    /* InitializeConditions for UnitDelay: '<S24>/Init2' */
    ved__mot_st_DWork->Init2_DSTATE_i = 1U;

    /* InitializeConditions for UnitDelay: '<S24>/FWD_Init' */
    ved__mot_st_DWork->FWD_Init_DSTATE_m = 33U;

    /* InitializeConditions for UnitDelay: '<S25>/Init2' */
    ved__mot_st_DWork->Init2_DSTATE_b = 1U;

    /* InitializeConditions for UnitDelay: '<S1>/FWD_Init4' */
    ved__mot_st_DWork->FWD_Init4_DSTATE = 33U;

    /* InitializeConditions for UnitDelay: '<S26>/Init2' */
    ved__mot_st_DWork->Init2_DSTATE_l = 1U;

    /* InitializeConditions for UnitDelay: '<S26>/FWD_Init' */
    ved__mot_st_DWork->FWD_Init_DSTATE_p = 33U;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */