#pragma once

#ifndef VLCVEH_CONSTS_H
#define VLCVEH_CONSTS_H
#ifdef __cplusplus
extern "C" {
#endif
#include "vlcVeh_ext.h"
// #include "TM_Global_TypeDefs.h"
#include "TM_Global_Types.h"

/*! VLC-Veh Sync-ref structure. Declared here so that simulation can instantiate
 */
typedef struct VLCVeh_SyncRef {
    SignalHeader_t sSigHeader;  /*!< Signal Header of Veh_SyncRef */
    SignalHeader_t VLCCtrlData; /*!< VLC_VEH operation mode control data */
    SignalHeader_t VehDyn;      /*!< The dynamic vehicle ego data raw */
    SignalHeader_t VehPar;      /*!< the static vehicle ego data */
                                /* Longitudinal control input ports */
    SignalHeader_t
        AccLeverInput; /*!< ACC lever information (input from driver) */
    SignalHeader_t LongCtrlInput; /*!< Dynamic controller response */
    SignalHeader_t acc_object;    /*!< Display object data output by VLC_SEN */
    SignalHeader_t acc_output_data; /*!< ACC output data by VLC_SEN */
    SignalHeader_t VLCAccOOIData /*! OOI Data from VLC_SEN cycle */;
    SignalHeader_t DIMInputGeneric;  /*!< Generic DIM input */
    SignalHeader_t DIMInputCustom;   /*!< Custom DIM input */
    SignalHeader_t HypothesisIntf;   /*!< CD hypotheses */
    SignalHeader_t HEADInputGeneric; /*!< Generic HEAD input */
    SignalHeader_t HEADInputCustom;  /*!< Custom HEAD input */
    SignalHeader_t ALN_S_Monitoring; /*!< Alignment monitoring output data */
    /* algo parameters from BSW */
    SignalHeader_t BSW_s_AlgoParameters; /*!< Input algo parameters from BSW */
    SignalHeader_t CPAR_VLC_Parameters;  /*!< VLC Coding Parameters */
    SignalHeader_t VehSig; /*!< Direct access to vehicle signals */
} VLCVeh_SyncRef_t;        /*!< @vaddr:VLC_MEAS_ID_VEH_INPUT_SIGHEADERS
                              @cycleid:VLC_VEH @vname:VLCVeh_SyncRef */

// VLCVehFrame_t VLCVehFrame;                            /*!<internal structure
// that stores the status of the Algo.*/
extern const VED_VehDyn_t* VLCVEH_pEgoDynRaw; /*!<internal pointer to external
                                                 raw ego dynamics INPUT
                                                 interface.*/
extern const VED_VehPar_t*
    VLCVEH_pGlobEgoStatic; /*!<internal pointer to external vehicle parameters
                              INPUT interface.*/
extern const VLC_HypothesisIntf_t*
    VLC_pCDHypothesesVeh; /*!<internal pointer to external hypothesis INPUT
                             interface.*/
extern VLC_DIMOutputCustom_t*
    VLC_pDIMCustDataOut; /*!<internal pointer to external DIM custom OUTPUT
                            interface.*/
extern const VLC_DIMInputCustom_t* VLC_pDIMCustDataIn; /*!<internal pointer to
                                                          external DIM custom
                                                          INPUT
                                                          interface.input.*/
extern const VLC_DIMInputGeneric_t*
    VLC_pDIMGenericDataIn; /*!<internal pointer to external DIM generic INPUT
                              interface. VLC DIM generic input.*/
extern VLC_SADOutputCustom_t* VLC_pHEADCustDataOut; /*!<internal pointer to
                                                       external HEAD custom
                                                       OUTPUT interface. VLC
                                                       HEAD custom output.*/
extern VLC_SADOutputGeneric_t*
    VLC_pHEADGenericDataOut; /*!<internal pointer to external HEAD generic
                                OUTPUT interface. VLC HEAD generic output.*/
extern const VLC_SADInputGeneric_t*
    VLC_pHEADGenericDataIn; /*!<internal pointer to external HEAD generic INPUT
                               interface. VLC HEAD generic input.*/
extern const VLC_SADInputCustom_t*
    VLC_pHEADCustDataIn; /*!<internal pointer to external HEAD custom INPUT
                            interface. VLC HEAD custom input.*/

extern const Com_AlgoParameters_t*
    VLCVEH_pBswAlgoParameters; /*!< Input algo parameters from BSW */
extern const VLC_Parameters_t*
    VLCVEH_pCPAR_VLC_Parameters; /*!<internal pointer to external EBA CParameter
                                    INPUT interface. VLC Coding Parameters.*/
extern const VED_VehSig_t*
    VLCVEH_pVehSig; /*!<internal pointer to external VDY sensor signals INPUT.
                       Pointer to raw vehicle signals.*/
extern const PowerTrain_t* VLCVEH_pVehSigPowerTrain;
extern const VLC_AccLeverInput_t* VLCVEH_pAccLever;
extern const VLC_LongCtrlInput_t* VLCVEH_pLongCtrlResp;
extern const VLC_acc_object_t* VLCVEH_pAccDisplayObj;
extern const VLC_acc_output_data_t* VLCVEH_pAccOutput;
extern VLC_DFV2SenInfo_t* VLCVEH_pDFVLongOut;
extern VLC_LongCtrlOutput_t* VLCVEH_pLongCtrlOutput;
extern VLC_LODMCOutput_t* VLCVEH_pLODMCOutput;
extern const VLCSenAccOOI_t* VLCVEH_pAccOOIData;
extern VLC_DFVOutArbitrated_t* VLCVEH_pVLCVehOutArbitrated;

extern const VLCVeh_Parameters_t* pVLCVEH_Parameters;
#ifdef __cplusplus
}
#endif

#endif