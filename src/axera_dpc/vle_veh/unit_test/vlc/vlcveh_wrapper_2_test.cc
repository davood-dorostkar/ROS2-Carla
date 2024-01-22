#include "gtest/gtest.h"
#include "vlcVeh_ext.h"

VLC_LongCtrlInput_t LongCtrlResp_debug{0}; /*!< Dynamic controller response */

TEST(VlcVehWrapperTest2, Test10)
{
    VLCCtrlData_t pVehCtrlData{0};        /*!< VLC_VEH operation mode control data */
    VED_VehDyn_t pEgoDynRaw{0};           /*!< The dynamic vehicle ego data raw */
    VED_VehPar_t pEgoStaticData{0};       /*!< the static vehicle ego data */
                                          /* Longitudinal control input ports */
    VLC_AccLeverInput_t pAccLever{0};     /*!< ACC lever information (input from driver) */
    VLC_LongCtrlInput_t pLongCtrlResp{0}; /*!< Dynamic controller response */
    VLC_acc_object_t pAccDisplayObj{0};   /*!< Display object data output by VLC_SEN */
    VLC_acc_output_data_t pAccOutput{0};  /*!< ACC output data by VLC_SEN */
    VLCSenAccOOI_t pVLCAccOOIData{0};
    t_CamLaneInputData pCamLaneData{0}; /*!< Camera lane input data */

    VLC_DIMInputGeneric_t pDIMInputGeneric{0};  /*!< Generic DIM input */
    VLC_DIMInputCustom_t pDIMInputCustom{0};    /*!< Custom DIM input */
    VLC_HypothesisIntf_t pVLCCDHypotheses{0};   /*!< CD hypotheses */
    VLC_SADInputGeneric_t pSADInputGeneric{0};  /*!< Generic HEAD input */
    VLC_SADInputCustom_t pSADInputCustom{0};    /*!< Custom HEAD input */
                                                /* algo parameters from BSW */
    Com_AlgoParameters_t pBswAlgoParameters{0}; /*!< Input algo parameters from BSW */
    VLC_Parameters_t pCPAR_VLC_Parameters{0};   /*!< VLC Coding Parameters */
    VED_VehSig_t pVehSig{0};                    /*!< Direct access to vehicle signals */
    ST_VLCVeh_NVRAMData_t pVLCVehNvRams{0};     /* vlc veh function nvram data*/
    AEB_VehSig_t pAebVehSig{0};

    reqVLCVehPrtList_t pRequirePorts = {
        &pVehCtrlData,         &pEgoDynRaw,       &pEgoStaticData,   &pAccLever,       &pLongCtrlResp,
        &pAccDisplayObj,       &pAccOutput,       &pVLCAccOOIData,   &pCamLaneData,    &pDIMInputGeneric,
        &pDIMInputCustom,      &pVLCCDHypotheses, &pSADInputGeneric, &pSADInputCustom, &pBswAlgoParameters,
        &pCPAR_VLC_Parameters, &pVehSig,          &pVLCVehNvRams,    &pAebVehSig};

    VLCVeh_Parameters_t pVLCParameters{0};

    VLC_DFV2SenInfo_t pDFVLongOut{0};               /*!< Internal info passed from VLC_VEH to VLC_SEN */
    VLC_LongCtrlOutput_t pLongCtrlOutput{0};        /*!< Longitudinal controller output data */
    VLC_DIMOutputCustom_t pDIMOutputCustom{0};      /*!< Custom DIM output */
    VLC_SADOutputGeneric_t pHEADOutputGeneric{0};   /*!< Generic HEAD output */
    VLC_SADOutputCustom_t pHEADOutputCustom{0};     /*!< Custom HEAD output */
    VLC_DFVErrorOut_t pErrorOut{0};                 /*!< VLC error output */
    VLC_DFVOutArbitrated_t pVLCVehOutArbitrated{0}; /*!< Aribrated output for vehicle functions */
    ISAInfo p_isa_info{0};
    PACCInfo p_pacc_info{0};
    VLC_LODMCOutput_t pLODMCOutput{0};

    proVLCVehPrtList_t pProvidePorts = {
        &pDFVLongOut,          &pLongCtrlOutput, &pDIMOutputCustom, &pHEADOutputGeneric, &pHEADOutputCustom, &pErrorOut,
        &pVLCVehOutArbitrated, &pVLCVehNvRams,   &p_isa_info,       &p_pacc_info,        &pLODMCOutput};

    cc_acceleration_control_data_t pVlcACCCtrlData{0};
    cc_control_data_t pVlcCCCtrlData{0};
    vlcVeh_ext_sm_debug_info VlcSmDebugInfo{0};
    reqVLCVehDebugList_t pVLCVehDebugPorts = {&pVlcACCCtrlData, &pVlcCCCtrlData, &VlcSmDebugInfo};

    pLongCtrlResp.KinCtrlDynInput.acc_enable = (boolean)1;
    pVLCVehNvRams.VLCVEH_Nb_ACCOnOffSwitch = (boolean)1;

    pEgoDynRaw.Longitudinal.MotVar.Velocity = (float32)1.0f;

    VLCVeh_Exec(&pRequirePorts, &pVLCParameters, &pProvidePorts, &pVLCVehDebugPorts);

    pAccLever.MainSwitch = (boolean)0;
    pAccLever.SetSwitch = (boolean)0;

    pLongCtrlResp.KinCtrlDynInput.acc_inhibit = (boolean)0;

    VLCVeh_Exec(&pRequirePorts, &pVLCParameters, &pProvidePorts, &pVLCVehDebugPorts);

    pAccLever.MainSwitch = (boolean)1;
    pAccLever.SetSwitch = (boolean)1;
    pLongCtrlResp.KinCtrlDynInput.acc_inhibit = (boolean)0;

    pLongCtrlResp.sSigHeader.uiCycleCounter = 66;
    pLongCtrlResp.uiVersionNumber = 666;
    pLongCtrlResp.KinCtrlDynInput.longi_initialization_accel = 6;
    pLongCtrlResp.Custom.LongCtrlInputCustom.AEB_Active = 1;
    memcpy(&LongCtrlResp_debug, (pRequirePorts.pLongCtrlResp), sizeof(VLC_LongCtrlInput_t));

    // test inhibit signal
    // pLongCtrlResp.KinCtrlDynInput.driver_override_decel_pedal = (boolean)1; // 1

    for (int i = 0; i < 10; ++i) {
        VLCVeh_Exec(&pRequirePorts, &pVLCParameters, &pProvidePorts, &pVLCVehDebugPorts);
    }

    // pLongCtrlResp.KinCtrlDynInput.driver_override_accel_pedal = (boolean)1; // das_status = 3
    // pLongCtrlResp.KinCtrlDynInput.brk_sw = (boolean)1;                       // 2
    pLongCtrlResp.KinCtrlDynInput.driver_override_decel_pedal = (boolean)1;  // 1
    // pLongCtrlResp.KinCtrlDynInput.hood_state = (boolean)1;            // 2
    // pLongCtrlResp.KinCtrlDynInput.stand_still_detected = (boolean)1;  // 2
    // pLongCtrlResp.KinCtrlDynInput.driver_braking = (boolean)1;        // 2

    for (int i = 0; i < 10; ++i) {
        VLCVeh_Exec(&pRequirePorts, &pVLCParameters, &pProvidePorts, &pVLCVehDebugPorts);
    }
    pAccLever.MainSwitch = (boolean)0;
    pAccLever.SetSwitch = (boolean)0;
    pLongCtrlResp.KinCtrlDynInput.acc_inhibit = (boolean)0;

    for (int i = 0; i < 10; ++i) {
        VLCVeh_Exec(&pRequirePorts, &pVLCParameters, &pProvidePorts, &pVLCVehDebugPorts);
    }

    EXPECT_TRUE(1);
}