// Copyright [2021] <Copyright Owner>
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcf_dmc_local_main.h"  //NOLINT
#include <string.h>
#ifdef UBUNTU_SYSTEM
#include "Rte_CtApLCFVEH.h"
#endif
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
#define CAL_START_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
const volatile uint16_T Test_Dmc_configuration_mode_par = 64U;
// the Integral maximum please modify the const volatile
// 'Lat_oc_max_delta_offset' in 'LaDMC.c' Lat_oc_max_delta_offset: maximum Front
// wheel angle offset, unit:degree
const volatile uint16_T Test_Dmc_offset_calibration_mode_par =
    0U;  // xxxx xxx1 open offset,only nvram
         // xxxx xx1x open offset use Lateral error as Integrator Input
         // xxxx x1xx open offset use Kappa as Integrator Input
const volatile real32_T Test_Dmc_DeltaF_offset_par = 0.0F;
const volatile real32_T Test_Lat_oc_kappa_cmd_filter_coeff_par = 0.1F;
const volatile real32_T Test_Lat_oc_max_kappa_dys_par = 0.005F;
const volatile real32_T Test_Lat_oc_ki_par =
    0.0015F;  // offset Integrator factor
const volatile real32_T Test_Lat_oc_delta_offset_filter_omega_par = 0.005F;
const volatile real32_T Test_Lat_oc_max_driver_torque_par = 0.5F;
const volatile real32_T Test_Dyc_state_filter_time_constant_par = 0.054F;
const volatile uint16_T Test_Dyc_compensation_mode_par = 310U;
const volatile real32_T Test_Dyc_kappa_a2_pos_corr_factor_par = 1.F;
const volatile real32_T Test_Dyc_kappa_a2_x_scheduling_par[12] = {
    0.F, 10.F, 20.F, 30.F, 40.F, 50.F, 60.F, 80.F, 100.F, 120.F, 180.F, 250.F};
const volatile real32_T Test_Dyc_kappa_a2_y_scheduling_par[12] = {
    0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F, 0.2F};
const volatile real32_T Test_Dyc_kappa_a2_factor_par = 1.F;
const volatile real32_T Test_Dyc_kappa_angle_gen_corr_factor_par = 1.F;
const volatile real32_T Test_Dyc_kappa_angle_t_x_schedul_gen_par[12] = {
    0.F, 10.F, 20.F, 30.F, 40.F, 50.F, 60.F, 80.F, 100.F, 120.F, 180.F, 250.F};
const volatile real32_T Test_Dyc_kappa_angle_t_y_schedul_gen_par[12] = {
    155.38F,   155.38F,   155.38F,   155.38F, 155.38F,   155.38F,
    161.0243F, 188.3895F, 215.7548F, 243.12F, 325.2157F, 420.994F};
const volatile real32_T Test_Dyc_kappa_angle_gen_cor_fct_neg_par = 1.F;
const volatile real32_T Test_Dyc_kappa_angle_t_y_sch_gen_neg_par[12] = {
    128.0492F, 128.0492F, 128.0492F, 128.0492F, 128.0492F, 128.0492F,
    128.0492F, 128.0492F, 138.6003F, 149.1515F, 180.8051F, 217.7342F};
const volatile real32_T Test_Lat_max_ay_par = 20.F;
const volatile uint32_T Test_Sac_controller_mode_1_par = 1115U;
const volatile real32_T Test_Sat_max_cmd_factor_x_scheduling_par[13] = {
    0.F, 10.F, 20.F, 30.F, 40.F, 50.F, 60.F, 80.F, 100.F, 120.F, 180.F, 250.F};
const volatile real32_T Test_Sat_max_cmd_factor_y_scheduling_par[13] = {
    1.F, 1.F, 1.F, 1.F, 1.F, 1.F, 1.F, 1.F, 1.F, 1.F, 1.F, 1.F, 1.F};
const volatile real32_T Test_Sat_max_delta_f_cmd_par = 20.F;
const volatile real32_T Test_VEH_Vehicle_Selfsteering_Factor_par = 0.00506F;
const volatile real32_T Test_Sat_max_angle_offset_x_par[4] = {0.F, 20.F, 100.F,
                                                              350.F};
const volatile real32_T Test_Sat_max_angle_offset_y_par[4] = {4.F, 4.F, 2.F,
                                                              2.F};
const volatile real32_T Test_Sat_max_angle_low_bound_x_sched_par[3] = {
    0.F, 15.F, 327.F};
const volatile real32_T Test_Sat_max_angle_low_bound_y_sched_par[3] = {
    40.F, 40.F, 40.F};
const volatile real32_T Test_Tdf_min_steer_torque_class_par = 0.3F;
const volatile uint16_T Test_Tdf_derating_mode_par = 20U;
const volatile real32_T Test_Tdf_torque_request_sign_slope_par = 10.F;
const volatile real32_T Test_Tdf_steer_torque_sign_slope_par = 10.F;
const volatile real32_T Test_Tdf_torque_derating_threshold_par = 0.3F;
const volatile real32_T Test_Tdf_trq_derating_threshold_dp_par = 0.0015F;
const volatile real32_T Test_Tdf_trq_der_thrs_dp_hi_sens_par = 0.0015F;
const volatile real32_T Test_Tdf_torque_derating_slope_ldp_par = 0.7F;
const volatile real32_T Test_Tdf_torque_derating_slope_par = 0.8333F;
const volatile real32_T Test_TDF_Derating_Switch_Thresh2_par = 5.F;
const volatile real32_T Test_TDF_Derating_Switch_Thresh1_par = 3.F;
const volatile real32_T Test_Sac_delta_f_cmd_grad_barrier_par = 2.F;
const volatile real32_T Test_Sac_delta_f_cmd_min_grad_par = 2.F;

#define CAL_STOP_CODE
#include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
// LaDMC module global values
static sTJADMCInReq_t g_LCF_DMC_Input = {0};
static sTJADMCParam_t g_LCF_DMC_Param = {0};
static sTJADMCOutPro_t g_LCF_DMC_Output = {0};
static sTJADMCDebug_t g_LCF_DMC_Debug = {0};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* ****************************************************************************

  Functionname:     LcfDMCExec

  @brief            LCF_DMC main function with input and output parameters

  @description      -

  @param[in]        reqPorts : pointer on the required ports structure
  @param[in,out]    proPorts : pointer on the provided ports structure

  @return           -
**************************************************************************** */
void LcfDMCExec(const reqLcfDmcPrtList_t* const reqPorts,
                const reqLcfDmcParams* reqParams,
                proLcfDmcPrtList_t* const proPorts,
                proLcfDmcDebug_t* proDebugs) {
    /*! At this place it is necessary to test lcfOpMode directly to get an
     * initialized component */
    switch (reqPorts->sBaseCtrlData.eOpMode) {
        case (uint8)BASE_OM_RESET:
            LcfDMCReset(reqParams);
            break;  // no calculation should be done in RESET
        case (uint8)BASE_OM_IDLE:
            // COMP_STATE_NOT_RUNNING shall be set if the algo is called in the
            // idle op mode;
            proPorts->pAlgoCompState.eCompState = COMP_STATE_NOT_RUNNING;
            proPorts->pAlgoCompState.eGenAlgoQualifier = ALGO_QUAL_OK;
            // Except for the comp state and the NVM structure the signal state
            // shall be 'AL_SIG_STATE_INVALID'.
            proPorts->pLcfDmcOutputData.sSigHeader.eSigStatus =
                AL_SIG_STATE_INVALID;
            // proPorts->pLcfSenOutputToVehData.sSigHeader.eSigStatus =
            // AL_SIG_STATE_INVALID;
            break;  // no calculation shall be done in IDLE
        case (uint8)BASE_OM_DEMO:
            proPorts->pAlgoCompState.eCompState = COMP_STATE_SUCCESS;
            proPorts->pAlgoCompState.eGenAlgoQualifier = ALGO_QUAL_OK;
            // (username): Stuff." [readability/todo]fill demo values for
            // all outputs
            break;                // no calculation shall be done in DEMO
        case (uint8)BASE_OM_RUN:  // check for other op modes
            // After the LCFVEH run first then some input is ready
            LcfDMCProcess(reqPorts, reqParams, proPorts, proDebugs);
            // printf("LCFDMC3 is Running \n");
            // }
            // else {}
            break;
        default:
            break;
    }
}

/* ****************************************************************************

  Functionname:     LcfDMCReset

  @brief            Reset of the component

  @description      Initialization of all internal data storage.
                                        Shall be called once before processing
starts

  @param[in]
  @param[in,out]    proPorts : pointer on the provided ports structure

  @return           -

  @pre              -
  @post             All internal values and all interfaces are initialized
                                        to default values


**************************************************************************** */
void LcfDMCReset(const reqLcfDmcParams* reqParams) {
    // DMC module global values
    memset(&g_LCF_DMC_Input, 0, sizeof(sTJADMCInReq_t));
    memset(&g_LCF_DMC_Param, 0, sizeof(sTJADMCParam_t));
    memset(&g_LCF_DMC_Output, 0, sizeof(sTJADMCOutPro_t));
    // set parameter values for DMC
    g_LCF_DMC_Param.uTemp_nu = 0u;
    LCF_TJADMC_Reset();
}

/* ****************************************************************************

 Functionname:     LcfDMCProcess

 @brief            Template Main processing

 @description      Calls all processing functions of lcf sen and sub-components

 @param[in]        reqPorts : pointer on the required ports structure
 @param[in,out]    proPorts : pointer on the provided ports structure
 @param[in]        services : pointer on the services structure

 @return           -

 @pre              All states must be set
 @post             -

 @todo             review for unused code

 @author

**************************************************************************** */
static void LcfDMCProcess(const reqLcfDmcPrtList_t* const reqPorts,
                          const reqLcfDmcParams* reqParams,
                          proLcfDmcPrtList_t* const proPorts,
                          proLcfDmcDebug_t* proDebugs) {
    LaDMCInputWrapper(reqPorts, reqParams, &g_LCF_DMC_Input);
    // printf("--------------------DMC Input--------------------\n");

    LCF_TJADMC_Exec(&g_LCF_DMC_Input, &g_LCF_DMC_Param, &g_LCF_DMC_Output,
                    &g_LCF_DMC_Debug);
    LaDMCOutputWrapper(g_LCF_DMC_Output, g_LCF_DMC_Debug, proPorts, proDebugs);

    proPorts->pAlgoCompState.sSigHeader.uiMeasurementCounter =
        reqPorts->sBaseCtrlData.sSigHeader.uiMeasurementCounter;
}

/* ****************************************************************************
 Functionname:     LaDMCInputWrapper
 @brief: the input wrapper function for the LaDMC process

 @description: the input wrapper function for the LaDMC process

 @param[in]
                        reqLcfVehPrtList_t* pLCFVehInput:the input of the LCF
whole module
 @param[in]
                        sTJATCTOut_st* pTCOutput:the output of the TC sub-module
 @param[out]
                        sTJADMCInReq_t* pDMCInput: the input of the LaDMC
sub-module

 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LaDMCInputWrapper(const reqLcfDmcPrtList_t* pLCFDmcInput,
                              const reqLcfDmcParams* reqParams,
                              sTJADMCInReq_t* pDMCInput) {
    // pDMCInput->fDriverTorque_nm = 0.f;
    // pDMCInput->fSteerAngleRequest_deg = pTCOutput->TJATCT_DeltaFCmd;
    // pDMCInput->fSteerAngle_deg = pLCFVehInput->sLCFVehDyn.fSteerWheelAgl_rad;
    // pDMCInput->fVehicleVelX_ms = pLCFVehInput->sLCFVehDyn.fEgoVelX_mps;
    // pDMCInput->uiDriverMode_nu = 1u;
    // pDMCInput->uiModeRequest_nu = pTCOutput->LGC_EnableCtrl_nu;
    pDMCInput->VEH_Vehicle_Speed = pLCFDmcInput->sLCFVehDyn.fEgoVelX_mps;
    // The curvature of the target traj. Currently from the TP
    // may from TC after debuging
    pDMCInput->LKC_Kappa_Command =
        pLCFDmcInput->sLcfInputFromSenVehData.sTPData.fTrajTgtCurve_1pm;
    pDMCInput->VEH_Yaw_Rate = pLCFDmcInput->sLCFVehDyn.fEgoYawRate_rps;
    pDMCInput->Veh_Lat_Acc = 0;  // should latral accelaration be received
    pDMCInput->VEH_Steer_Torque = pLCFDmcInput->sLCFVehDyn.fManuActuTrqEPS_nm;
    // Expected from LCF , not exist currently
    pDMCInput->LAT_Stiffness_Request_Factor = 100;
    pDMCInput->VEH_SteerAngle = pLCFDmcInput->sLCFVehDyn.fWhlSteerAngleVdy_rad;
    pDMCInput->CAM_Latral_Error_Qf = 1;  // default valid
    pDMCInput->CAM_Lateral_Error =
        pLCFDmcInput->sLcfInputFromSenVehData.sLBPData.fPosY0CtrlCntr;
    pDMCInput->VEH_Delta_F_Dot = pLCFDmcInput->sLCFVehDyn.fSteerWhlAglspd_rads;
    pDMCInput->LKC_Delta_Ys =
        pLCFDmcInput->sLcfInputFromSenVehData.sTCData.CDC_CtrlErrDistY;
    pDMCInput->LKC_Delta_Psi =
        pLCFDmcInput->sLcfInputFromSenVehData.sTCData.CDC_CtrlErrHeading;
    pDMCInput->LKC_DgrSide =
        pLCFDmcInput->sLcfInputFromSenVehData.sLDPSAData.LDPSC_DgrSide_St;
    pDMCInput->LDP_Status =
        pLCFDmcInput->sLcfInputFromSenVehData.sLDPSAData.LDPSC_SysOut_St;
    pDMCInput->CAM_Kappa_Cmd =
        pLCFDmcInput->sLcfInputFromSenVehData.sLBPData.fCrvCtrlCntr;
    pDMCInput->CAM_Kappa_Cmd_Qf = 1;  // TODO(zhanghanqi): default valid
    // It could from the vehicle parameters later. Take a default value
    // currently
    pDMCInput->EPS_Gear_Ratio = reqParams->LCFVeh_Kf_SteeringRatio_nu;
    pDMCInput->LaKMC_kappaP_cmd = 0;  // The timebased dot of kappa , not needed
    // The max limitation scale of the derating and fading factor default value.
    pDMCInput->LaKMC_angle_req_max_limit_scale = 100;
    // The scale to merge the max grad and min grad of the angel command
    pDMCInput->LaKMC_angle_req_max_grad_scale = 100;
    // pDMCInput->LAT_Delta_F_Cmd
    //= pLCFDmcInput->sLcfInputFromSenVehData.sTCData.TJATCT_DeltaFCmd;
    pDMCInput->ADAS_Delta_F_Cmd =
        pLCFDmcInput->sLcfInputFromSenVehData.sTCData.TJATCT_DeltaFCmd;
    pDMCInput->ADAS_Delta_F_Enable_flag =
        pLCFDmcInput->sLcfInputFromSenVehData.sTCData
            .LGC_EnableCtrl_nu;  // TODO(zhanghanqi): default valid
    pDMCInput->ADAS_Lon_Acc_Cmd =
        pLCFDmcInput->sLCFInputFromVLCData.ADAS_Lon_Acc_Cmd;
    pDMCInput->ADAS_Lon_Acc_Enable_flag =
        pLCFDmcInput->sLCFInputFromVLCData.ADAS_Lon_Acc_Enable_flag;
    pDMCInput->NOP_Delta_F_Cmd =
        pLCFDmcInput->sLCFInputFromNOPData.NOP_Delta_F_Cmd;
    pDMCInput->NOP_Delta_F_Enable_flag =
        pLCFDmcInput->sLCFInputFromNOPData.NOP_Delta_F_Enable_flag;
    pDMCInput->NOP_Lon_Acc_Cmd =
        pLCFDmcInput->sLCFInputFromNOPData.NOP_Lon_Acc_Cmd;
    pDMCInput->NOP_Lon_Acc_Enable_flag =
        pLCFDmcInput->sLCFInputFromNOPData.NOP_Lon_Acc_Enable_flag;

    pDMCInput->CAM_Heading_Error =
        pLCFDmcInput->sLcfInputFromSenVehData.sLBPData.fHeadingCtrlCntr;
    pDMCInput->CAM_Heading_Error_Qf = 1;  // defauld valid
    pDMCInput->VEH_cycletime =
        reqParams->LCFVeh_Kf_fSysCycleTime_sec;  // cycle time
    pDMCInput->TJALatCtrlMode_nu =
        pLCFDmcInput->sLcfInputFromSenVehData.sTJASAState.uiLatCtrlMode_nu;
    pDMCInput->Dmc_configuration_mode_par = Test_Dmc_configuration_mode_par;

    pDMCInput->Dmc_offset_calibration_mode_par =
        Test_Dmc_offset_calibration_mode_par;
    pDMCInput->Dmc_DeltaF_offset_par =
        pLCFDmcInput->sNVRAMData.LCFVeh_Nf_SensitivityReserved1_nu;

    pDMCInput->Lat_oc_kappa_cmd_filter_coeff_par =
        Test_Lat_oc_kappa_cmd_filter_coeff_par;
    pDMCInput->Lat_oc_max_kappa_dys_par = Test_Lat_oc_max_kappa_dys_par;
    pDMCInput->Lat_oc_ki_par = Test_Lat_oc_ki_par;
    pDMCInput->Lat_oc_delta_off_flt_initial_omega_par =
        pLCFDmcInput->sLCF_DmcCalibrations
            .Lat_oc_delta_off_flt_initial_omega_par;
    pDMCInput->Lat_oc_delta_offset_filter_omega_par =
        Test_Lat_oc_delta_offset_filter_omega_par;
    pDMCInput->Lat_oc_max_driver_torque_par = Test_Lat_oc_max_driver_torque_par;
    pDMCInput->Dyc_state_filter_time_constant_par =
        Test_Dyc_state_filter_time_constant_par;
    pDMCInput->Dyc_compensation_mode_par = Test_Dyc_compensation_mode_par;
    pDMCInput->Dyc_kappa_a2_pos_corr_factor_par =
        Test_Dyc_kappa_a2_pos_corr_factor_par;
    memcpy(pDMCInput->Dyc_kappa_a2_x_scheduling_par,
           Test_Dyc_kappa_a2_x_scheduling_par, 12 * sizeof(float32));
    memcpy(pDMCInput->Dyc_kappa_a2_y_scheduling_par,
           Test_Dyc_kappa_a2_y_scheduling_par, 12 * sizeof(float32));
    pDMCInput->Dyc_kappa_a2_factor_par = Test_Dyc_kappa_a2_factor_par;
    pDMCInput->Dyc_kappa_angle_gen_corr_factor_par =
        Test_Dyc_kappa_angle_gen_corr_factor_par;
    memcpy(pDMCInput->Dyc_kappa_angle_t_x_schedul_gen_par,
           Test_Dyc_kappa_angle_t_x_schedul_gen_par, 12 * sizeof(float32));
    memcpy(pDMCInput->Dyc_kappa_angle_t_y_schedul_gen_par,
           Test_Dyc_kappa_angle_t_y_schedul_gen_par, 12 * sizeof(float32));
    pDMCInput->Dyc_kappa_angle_gen_cor_fct_neg_par =
        Test_Dyc_kappa_angle_gen_cor_fct_neg_par;
    memcpy(pDMCInput->Dyc_kappa_angle_t_y_sch_gen_neg_par,
           Test_Dyc_kappa_angle_t_y_sch_gen_neg_par, 12 * sizeof(float32));
    pDMCInput->Lat_max_ay_par = Test_Lat_max_ay_par;
    pDMCInput->Sac_controller_mode_1_par = Test_Sac_controller_mode_1_par;
    memcpy(pDMCInput->Sat_max_cmd_factor_x_scheduling_par,
           Test_Sat_max_cmd_factor_x_scheduling_par, 13 * sizeof(float32));
    memcpy(pDMCInput->Sat_max_cmd_factor_y_scheduling_par,
           Test_Sat_max_cmd_factor_y_scheduling_par, 13 * sizeof(float32));
    pDMCInput->Sat_max_delta_f_cmd_par = Test_Sat_max_delta_f_cmd_par;
    pDMCInput->VEH_Vehicle_Selfsteering_Factor_par =
        Test_VEH_Vehicle_Selfsteering_Factor_par;
    memcpy(pDMCInput->Sat_max_angle_offset_x_par,
           Test_Sat_max_angle_offset_x_par, 4 * sizeof(float32));
    memcpy(pDMCInput->Sat_max_angle_offset_y_par,
           Test_Sat_max_angle_offset_y_par, 4 * sizeof(float32));
    memcpy(pDMCInput->Sat_max_angle_low_bound_x_sched_par,
           Test_Sat_max_angle_low_bound_x_sched_par, 3 * sizeof(float32));
    memcpy(pDMCInput->Sat_max_angle_low_bound_y_sched_par,
           Test_Sat_max_angle_low_bound_y_sched_par, 3 * sizeof(float32));
    pDMCInput->Tdf_min_steer_torque_class_par =
        Test_Tdf_min_steer_torque_class_par;
    pDMCInput->Tdf_derating_mode_par = Test_Tdf_derating_mode_par;
    pDMCInput->Tdf_torque_request_sign_slope_par =
        Test_Tdf_torque_request_sign_slope_par;
    pDMCInput->Tdf_steer_torque_sign_slope_par =
        Test_Tdf_steer_torque_sign_slope_par;
    pDMCInput->Tdf_torque_derating_threshold_par =
        Test_Tdf_torque_derating_threshold_par;
    pDMCInput->Tdf_trq_derating_threshold_dp_par =
        Test_Tdf_trq_derating_threshold_dp_par;
    pDMCInput->Tdf_trq_der_thrs_dp_hi_sens_par =
        Test_Tdf_trq_der_thrs_dp_hi_sens_par;
    pDMCInput->Tdf_torque_derating_slope_ldp_par =
        Test_Tdf_torque_derating_slope_ldp_par;
    pDMCInput->Tdf_torque_derating_slope_par =
        Test_Tdf_torque_derating_slope_par;
    pDMCInput->TDF_Derating_Switch_Thresh2_par =
        Test_TDF_Derating_Switch_Thresh2_par;
    pDMCInput->TDF_Derating_Switch_Thresh1_par =
        Test_TDF_Derating_Switch_Thresh1_par;
    pDMCInput->Sac_delta_f_cmd_grad_barrier_par =
        Test_Sac_delta_f_cmd_grad_barrier_par;
    pDMCInput->Sac_delta_f_cmd_min_grad_par = Test_Sac_delta_f_cmd_min_grad_par;
    pDMCInput->DMC_NVRAMReset_par = pLCFDmcInput->DMC_NVRAMReset;
}

/* ****************************************************************************
 Functionname:     LaDMCOutputWrapper
 @brief:		   the output wrapper function for the TC process

 @description:	   convert TC sub-module's result to LCF Veh output structure

 @param[in]		   const sTJADMCOutPro_t sDMCOutput: LaDMC sub-module's
output
 @param[out]       proLcfSenPrtList_t* LCFSenOutput: LCF Sen output pointer
 @return           -
 @pre
 @post             -
 @author
**************************************************************************** */
static void LaDMCOutputWrapper(const sTJADMCOutPro_t sDMCOutput,
                               const sTJADMCDebug_t sDMCDebug,
                               proLcfDmcPrtList_t* pLCFDmcOutput,
                               proLcfDmcDebug_t* pLcfDmcDebug) {
    pLCFDmcOutput->pAlgoCompState.sSigHeader.uiCycleCounter += 1;
    pLCFDmcOutput->pLcfDmcOutputData.sDMCOutput.fSteerAngle_deg =
        sDMCOutput.fSteerAngle_deg;
    pLCFDmcOutput->pLcfDmcOutputData.sDMCOutput.fSteerWhlAngle_deg =
        sDMCOutput.fSteerWhlAngle_deg;
    pLCFDmcOutput->pLcfDmcOutputData.sDMCOutput.uiEPSRequest_nu =
        sDMCOutput.uiEPSRequest_nu;
    pLCFDmcOutput->pLcfDmcOutputData.sDMCOutput.uiLonAccRequest_nu =
        sDMCOutput.uiLonAccRequest_nu;
    pLCFDmcOutput->pLcfDmcOutputData.sDMCOutput.fLonAccCmd =
        sDMCOutput.fLonAccCmd;
    pLCFDmcOutput->sNVRAMData.LCFVeh_Nf_SensitivityReserved1_nu =
        sDMCDebug.LAT_Oc_Integrator_Sat_Out;
    pLcfDmcDebug->ADP_Dyc_Corr_Factor = sDMCDebug.ADP_Dyc_Corr_Factor;
    pLcfDmcDebug->CAM_Lateral_Error_Sign = sDMCDebug.CAM_Lateral_Error_Sign;
    pLcfDmcDebug->DYC_Filter_Kappa_Command = sDMCDebug.DYC_Filter_Kappa_Command;
    pLcfDmcDebug->DYC_Steer_Angle_Feedforward =
        sDMCDebug.DYC_Steer_Angle_Feedforward;
    pLcfDmcDebug->HEC_Yaw_Rate_Filter = sDMCDebug.HEC_Yaw_Rate_Filter;
    pLcfDmcDebug->Initialisation_Flag = sDMCDebug.Initialisation_Flag;
    pLcfDmcDebug->LAT_Kappa_Linz_Filter_Output =
        sDMCDebug.LAT_Kappa_Linz_Filter_Output;
    pLcfDmcDebug->LAT_Sat_Dynamic_Threshold_int =
        sDMCDebug.LAT_Sat_Dynamic_Threshold_int;
    pLcfDmcDebug->Lateral_Error_Delta = sDMCDebug.Lateral_Error_Delta;
    pLcfDmcDebug->Lateral_Error_Mean = sDMCDebug.Lateral_Error_Mean;
    pLcfDmcDebug->Mean_Sample_Update_sum = sDMCDebug.Mean_Sample_Update_sum;
    pLcfDmcDebug->Mean_Vehicle_Velocity = sDMCDebug.Mean_Vehicle_Velocity;
    pLcfDmcDebug->Mean_Vehicle_Velocity_sum =
        sDMCDebug.Mean_Vehicle_Velocity_sum;
    pLcfDmcDebug->Mean_kappa_command = sDMCDebug.Mean_kappa_command;
    pLcfDmcDebug->Mean_kappa_command_sum = sDMCDebug.Mean_kappa_command_sum;
    pLcfDmcDebug->New_Update_Aval = sDMCDebug.New_Update_Aval;
    pLcfDmcDebug->New_Update_Aval_sum = sDMCDebug.New_Update_Aval_sum;
    pLcfDmcDebug->SAC_Angle_Command_Corr = sDMCDebug.SAC_Angle_Command_Corr;
    pLcfDmcDebug->SAC_Angle_Command_Yawrate_Fback =
        sDMCDebug.SAC_Angle_Command_Yawrate_Fback;
    pLcfDmcDebug->SAC_Arbitrated_Angle_Cmd = sDMCDebug.SAC_Arbitrated_Angle_Cmd;
    pLcfDmcDebug->SAC_Arbitrated_Angle_Cmd_Raw =
        sDMCDebug.SAC_Arbitrated_Angle_Cmd_Raw;
    pLcfDmcDebug->SAC_Compensation_Angle_Command =
        sDMCDebug.SAC_Compensation_Angle_Command;
    pLcfDmcDebug->SAC_Control_Error = sDMCDebug.SAC_Control_Error;
    pLcfDmcDebug->SAC_Derated_Angle_Command =
        sDMCDebug.SAC_Derated_Angle_Command;
    pLcfDmcDebug->SAC_Integrator_Sat_Out = sDMCDebug.SAC_Integrator_Sat_Out;
    pLcfDmcDebug->SAC_Rate_Lim_Angle_Command =
        sDMCDebug.SAC_Rate_Lim_Angle_Command;
    pLcfDmcDebug->SAC_Trq_Derating_Factor = sDMCDebug.SAC_Trq_Derating_Factor;
    pLcfDmcDebug->SAC_Yrc_Angle_Command = sDMCDebug.SAC_Yrc_Angle_Command;
    pLcfDmcDebug->SAC_Yrc_Control_Error = sDMCDebug.SAC_Yrc_Control_Error;
    pLcfDmcDebug->SAT_Req_Dyn_Steer_Angle_Max =
        sDMCDebug.SAT_Req_Dyn_Steer_Angle_Max;
    pLcfDmcDebug->SAT_Req_Steer_Angle_Max = sDMCDebug.SAT_Req_Steer_Angle_Max;
    pLcfDmcDebug->SAT_Saturated_Angle_Command =
        sDMCDebug.SAT_Saturated_Angle_Command;
    pLcfDmcDebug->TDF_Composite_Derating_Factor =
        sDMCDebug.TDF_Composite_Derating_Factor;
    pLcfDmcDebug->TDF_Idle_Derating_Factor = sDMCDebug.TDF_Idle_Derating_Factor;
    pLcfDmcDebug->TDF_Selected_State_Derating_Factor =
        sDMCDebug.TDF_Selected_State_Derating_Factor;
    pLcfDmcDebug->VEH_Delta_F_Oc = sDMCDebug.VEH_Delta_F_Oc;
    pLcfDmcDebug->VEH_Delta_F_Offset = sDMCDebug.VEH_Delta_F_Offset;
    pLcfDmcDebug->TDF_Driver_Counter_Steering =
        sDMCDebug.TDF_Driver_Counter_Steering;
    pLcfDmcDebug->TDF_Max_Steer_Torque = sDMCDebug.TDF_Max_Steer_Torque;
    pLcfDmcDebug->TDF_Selected_Torque_Source =
        sDMCDebug.TDF_Selected_Torque_Source;
    pLcfDmcDebug->TDF_Torque_Der_Factor_HF_Path =
        sDMCDebug.TDF_Torque_Der_Factor_HF_Path;
    pLcfDmcDebug->TDF_Torque_Derating_Factor =
        sDMCDebug.TDF_Torque_Derating_Factor;
    pLcfDmcDebug->TDF_Torque_Derating_Slop_Arb =
        sDMCDebug.TDF_Torque_Derating_Slop_Arb;
    pLcfDmcDebug->TDF_Trq_Derating_Threshold_Arb =
        sDMCDebug.TDF_Trq_Derating_Threshold_Arb;
    pLcfDmcDebug->TDF_Vehicle_Steer_Torque_Factor =
        sDMCDebug.TDF_Vehicle_Steer_Torque_Factor;
    pLcfDmcDebug->TDF_Torque_Request_Factor =
        sDMCDebug.TDF_Torque_Request_Factor;
    pLcfDmcDebug->VEH_Steer_Torque_Comp = sDMCDebug.VEH_Steer_Torque_Comp;
    pLcfDmcDebug->LAT_Oc_Cal_Hold_Flag = sDMCDebug.LAT_Oc_Cal_Hold_Flag;
    pLcfDmcDebug->LAT_Oc_Cal_Hold_Flag_Shrt =
        sDMCDebug.LAT_Oc_Cal_Hold_Flag_Shrt;
    pLcfDmcDebug->LAT_Oc_Disable_Flag = sDMCDebug.LAT_Oc_Disable_Flag;
    pLcfDmcDebug->LAT_Oc_Dys_Active = sDMCDebug.LAT_Oc_Dys_Active;
    pLcfDmcDebug->LAT_Oc_Filtered_Kappa_Cam =
        sDMCDebug.LAT_Oc_Filtered_Kappa_Cam;
    pLcfDmcDebug->LAT_Oc_High_Driver_Torque =
        sDMCDebug.LAT_Oc_High_Driver_Torque;
    pLcfDmcDebug->LAT_Oc_Implaus_Lateral_Error =
        sDMCDebug.LAT_Oc_Implaus_Lateral_Error;
    pLcfDmcDebug->LAT_Oc_Integrator_Input = sDMCDebug.LAT_Oc_Integrator_Input;
    pLcfDmcDebug->LAT_Oc_Integrator_Input_Kappa =
        sDMCDebug.LAT_Oc_Integrator_Input_Kappa;
    pLcfDmcDebug->LAT_Oc_Integrator_Input_Kappa_dbg =
        sDMCDebug.LAT_Oc_Integrator_Input_Kappa_dbg;
    pLcfDmcDebug->LAT_Oc_Integrator_Output = sDMCDebug.LAT_Oc_Integrator_Output;
    pLcfDmcDebug->LAT_Oc_Integrator_Sat_Out =
        sDMCDebug.LAT_Oc_Integrator_Sat_Out;
    pLcfDmcDebug->LAT_Oc_Kappa_Active = sDMCDebug.LAT_Oc_Kappa_Active;
    pLcfDmcDebug->LAT_Oc_Kappa_Con_Enb_Flag =
        sDMCDebug.LAT_Oc_Kappa_Con_Enb_Flag;
    pLcfDmcDebug->LAT_Oc_Kappa_Latency_state =
        sDMCDebug.LAT_Oc_Kappa_Latency_state;
    pLcfDmcDebug->LAT_Oc_Kappa_Status_dbg = sDMCDebug.LAT_Oc_Kappa_Status_dbg;
    pLcfDmcDebug->LAT_Oc_Max_Delta_F_Dot_Flag =
        sDMCDebug.LAT_Oc_Max_Delta_F_Dot_Flag;
    pLcfDmcDebug->LAT_Oc_Max_Drv_Trq_Flag = sDMCDebug.LAT_Oc_Max_Drv_Trq_Flag;
    pLcfDmcDebug->LAT_Oc_Max_Hea_Err_Flag = sDMCDebug.LAT_Oc_Max_Hea_Err_Flag;
    pLcfDmcDebug->LAT_Oc_Max_Kappa_Cmd_Flag =
        sDMCDebug.LAT_Oc_Max_Kappa_Cmd_Flag;
    pLcfDmcDebug->LAT_Oc_Max_Lat_Acc_Flag = sDMCDebug.LAT_Oc_Max_Lat_Acc_Flag;
    pLcfDmcDebug->LAT_Oc_Sat_Integrator_state =
        sDMCDebug.LAT_Oc_Sat_Integrator_state;
    pLcfDmcDebug->LAT_Oc_Trigger_Flag_Kappa =
        sDMCDebug.LAT_Oc_Trigger_Flag_Kappa;
    pLcfDmcDebug->LAT_Oc_Trigger_Flag_Kappa_state =
        sDMCDebug.LAT_Oc_Trigger_Flag_Kappa_state;
    pLcfDmcDebug->LAT_Oc_offset_filter_omega =
        sDMCDebug.LAT_Oc_offset_filter_omega;
    pLcfDmcDebug->LAT_Oc_Min_Veh_Vel_Flag = sDMCDebug.LAT_Oc_Min_Veh_Vel_Flag;
    pLcfDmcDebug->LAT_Oc_Max_Flt_Kappa_Cmd_Flag =
        sDMCDebug.LAT_Oc_Max_Flt_Kappa_Cmd_Flag;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */