

/* Bedingte Einbindung */
#ifndef _VLC_VEH_H_INCLUDED
#define _VLC_VEH_H_INCLUDED

/* Additional check to assure that the VLC pars running in two different task
contexts do not try to access the same data - possibly corrupting each other's
states, leading to reproducibility issues in simulation */
#ifdef _VLC_SEN_H_INCLUDED
#error vlc_sen.h and vlc_veh.h shall not be included in the same context!
#endif

/*****************************************************************************
  INCLUDES
*****************************************************************************/

// #include "vlc_veh_ext.h"
// #include "frame_veh_custom_types.h"

// #include "TM_Global_Types.h"
// #include "TM_PlatF_Types.h"
// #include "Compiler.h"

// #include "Compiler_Cfg.h"
// #include "TM_Global_Types.h"
// #include "TM_PlatF_Types.h"
// #include "TM_Soc_Ips.h"
// #include "glob.h"
// #include "Project_specific_Types.h"
// #include "TM_Std_Types.h"
// #include "TM_MamMap.h"
// #include "stddef.h"

// #include "Tuerme.h"
// #include "TM_Rte.h"
// #include "TM_Rte_Types.h"
// #include "TM_Base_Bayes.h"
// #include "assert.h"
// #include "TM_Base_Cfg.h"
// #include "TM_Base_Complex.h"
// #include "TM_Base_Const.h"
// #include "TM_Base_Emul.h"
// #include "TM_Base_Fourier.h"
// #include "TM_Base_Kafi.h"
// #include "TM_Base_Mapping.h"
// #include "TM_Base_Mat.h"
// #include "TM_Base_Misc.h"
// #include "TM_Base_Mtrx.h"
// #include "TM_Base_Stat.h"
// #include "TM_Base_Trigo.h"
// #include "TM_Base_Vector.h"
// #include "TM_Base_Interpol.h"
// #include "TM_Base_Ver.h"
// #include "TM_Base_Ext.h"
// #include "Tuerme.h"
#ifdef __cplusplus
/* C++ extern C declarations, to allow directly including them in C++ sources.
 */
extern "C" {
#endif

#include "vlcVeh_common_utils.h"
#include "vlcVeh_ext.h"
#include "vlc_glob_ext.h"

#include "head_ext.h"
#include "dim_ext.h"
#include "vlc_long_veh_ext.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/* switch off UDW code modules if not compiled in this part of VLC */

#define VLC_FREEZE_DATA(pInfo, pData, Callback) \
    MEASFreezeDataDLG((pInfo), (pData),         \
                      (Callback)) /*!<macro to internal measfreeze procedure*/

#define GET_EBA_BUS_DEBUG_DATA                                              \
    VLCVEH_pEbaBusDebugData /*!<alias for pointer to EBABusDebugData OUTPUT \
                               interface*/
#define GET_LKA_BUS_DEBUG_DATA                                              \
    VLCVEH_pLkaBusDebugData /*!<alias for pointer to LKABusDebugData OUTPUT \
                               interface*/

#define GET_EGO_RAW_DATA_PTR                                               \
    VLCVEH_pEgoDynRaw /*!<alias for pointer to raw ego dynamics data INPUT \
                         interface*/
#define GET_EGO_STATIC_DATA_PTR                                           \
    VLCVEH_pGlobEgoStatic /*!<alias for pointer to ego vehicle parameters \
                             INPUT interface*/
#define GET_VLCVEH_PARAMETERS                                                \
    pVLCVEH_Parameters /*!<alias for pointer to ego vehicle parameters INPUT \
                          interface*/

#define GET_VLC_HEAD_CUST_OUT_DATA_PTR                               \
    VLC_pHEADCustDataOut /*!<alias for pointer to HEAD custom OUTPUT \
                            interface*/
#define GET_VLC_HEAD_GENERIC_OUT_DATA_PTR                                \
    VLC_pHEADGenericDataOut /*!<alias for pointer to HEAD generic OUTPUT \
                               interface*/
#define GET_VLC_HEAD_GENERIC_IN_DATA_PTR                               \
    VLC_pHEADGenericDataIn /*!<alias for pointer to HEAD generic INPUT \
                              interface*/
#define GET_VLC_HEAD_CUST_IN_DATA_PTR \
    VLC_pHEADCustDataIn /*!<alias for pointer to HEAD custom INPUT interface*/

#define GET_VLC_UDW_GENERIC_IN_DATA_PTR                              \
    VLC_pUDWGenericDataIn /*!<alias for pointer to UDW generic INPUT \
                             interface*/
#define GET_VLC_UDW_GENERIC_OUTPUT_DATA_PTR                            \
    VLC_pUDWGenericDataOut /*!<alias for pointer to UDW generic OUTPUT \
                              interface*/
#define GET_VLC_UDW_LANE_INPUT_DATA_PTR \
    VLC_pUDWLaneInput /*!<alias for pointer to UDW Lane INPUT interface*/
#define GET_VLC_UDW_CB_INPUT_DATA_PTR                                 \
    VLC_pUDWBlockageInput /*!<alias for pointer to UDW blockage INPUT \
                             interface*/
#define GET_VLC_UDW_ALDW_OUTPUT_DATA_PTR \
    VLC_pUDWALDWOutput /*!<alias for pointer to UDW ALDW INPUT interface*/

#define GET_VLC_DIM_CUST_OUT_DATA_PTR \
    VLC_pDIMCustDataOut /*!<alias for pointer to DIM custom OUTPUT interface*/
#define GET_VLC_DIM_CUST_IN_DATA_PTR \
    VLC_pDIMCustDataIn /*!<alias for pointer to DIM custom INPUT interface*/
#define GET_VLC_DIM_GENERIC_IN_DATA_PTR                              \
    VLC_pDIMGenericDataIn /*!<alias for pointer to DIM generic INPUT \
                             interface*/
#define GET_VLC_HMI_DATA_PTR \
    VLC_pHMIData /*!<alias for pointer to HMI INPUT interface*/
#define GET_VLC_LKA_LANE_INPUT_DATA_PTR \
    VLC_pLKALaneInput /*!<alias for pointer to LKA lane INPUT interface*/
#define GET_VLC_LKA_CB_INPUT_DATA_PTR                                 \
    VLC_pLKABlockageInput /*!<alias for pointer to LKA blockage INPUT \
                             interface*/
#define GET_VLC_LKA_INPUT_GENERIC_DATA_PTR                          \
    VLC_pLKAInputGeneric /*!<alias for pointer to LKA generic INPUT \
                            interface*/
#define GET_VLC_LKA_CODABLE_PARAM_PTR                                          \
    VLC_pLKACodableParam /*!<alias for pointer to LKA codable parameters INPUT \
                            interface*/
#define GET_VLC_LKA_OUTPUT_DATA_PTR                                   \
    VLC_pLKAOutputGeneric /*!<alias for pointer to LKA generic OUTPUT \
                             interface*/

#define VLC_BSW_ALGO_PARAM_PTR                                             \
    VLCVEH_pBswAlgoParameters /*!<alias for pointer to BSW algo parameters \
                                 interface*/

#define VLC_CPAR_VLC_PARAM_PTR                                                \
    VLCVEH_pCPAR_VLC_Parameters /*!<alias for pointer to EBA CParameter INPUT \
                                   interface*/
#define VLC_CPAR_VLC_LKS_PARAM_PTR \
    VLCVEH_pCPAR_VLC_LKS_Parameters /*!<VLCVEH_pCPAR_VLC_LKS_Parameters*/

/* task monitoring checkpoints */
#define VLC_CHECKPOINT_PROCESS_INPUT /* add one */           /*!<not used*/
#define VLC_CHECKPOINT_PROCESS_OUTPUT /* add one */          /*!<not used*/
#define VLC_EVALUATE_CHECKPOINT /* TPEvaluateCheckpoint() */ /*!<not used*/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*! operating modes of sub-component */
typedef enum {
    VLC_VEH_INIT,    /*!< initialize all    */
    VLC_VEH_RUN,     /*!< normal processing */
    VLC_VEH_SHUTDOWN /*!< shutdown (system failure) */
} VLCVehState_t;

typedef struct {
    VLCSwVersion_t uiHEAD;           /*!< SW verion of HEAD*/
    VLCSwVersion_t uiDIM;            /*!< SW verion of DIM*/
    VLCSwVersion_t FctVersionNumVar; /*!< VLCSwVersion_t*/
    VLCSwVersion_t uiVLCVEH;         /*!< SW verion of VLCVeh*/
} VLCVersions_t;

/*! The function frame @VADDR:VLC_MEAS_ID_VEH_FRAME_DATA @VNAME:VLCVehFrameData
 * @CYCLEID: VLC_VEH */
typedef struct VLCVehFrame_t {
    AlgoCycleCounter_t uiCycleCounter; /*!< The VLC_VEH cycle counter */
    boolean bFirstCycleDone;  /*!< Boolean flag used for first cycle check */
    VLC_OP_MODE_t eVLCOpMode; /*!< VLC operation mode */
    VLCVehState_t eVLCState;  /*!< VLC current operation mode */
    VLCVersions_t Versions;
} VLCVehFrame_t;

/*! Estimate whether OOI has disappeared suddenly. if yes, ACC will exit. */
typedef enum {
    Exist,
    Not_Exist,
    Disappear_Sudden,
    Disappear_Normal
} ACC_OOI_Status_t;

/*****************************************************************************
  TYPEDEFS (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! The VLC-Veh FuncID used for meas-freezes */
#define VLC_MEAS_FUNC_ID \
    TASK_ID_ALGO_VEH_CYCLE /*!< alias to VLCVehicle ID (for measfreezing)*/
/*! The VLC-Veh FuncChannelID used for meas-freezes, clarify this later! */
#define VLC_MEAS_FUNC_CHAN_ID 20u

#define VLC_MEAS_LKA_ID                     \
    COMP_ID_ALDW /*!< alias to ALDW ID (for \
                    measfreezing)*/
#define VLC_MEAS_LKA_CHAN_ID \
    TASK_ID_ALDW /*!< alias to LKA channel ID (for measfreezing)*/
/*****************************************************************************
  VARIABLES
*****************************************************************************/
/*! @cond Doxygen_Suppress */
extern MEMSEC_REF const VED_VehDyn_t* VLCVEH_pEgoDynRaw;
extern MEMSEC_REF const VED_VehPar_t*
    VLCVEH_pGlobEgoStatic; /* The static vehicle ego data */
extern MEMSEC_REF VLCCDOutputCustom_t*
    VLCSEN_pCDCustomOutput; /* CD custom output data */
extern MEMSEC_REF const VLC_HypothesisIntf_t*
    VLC_pCDHypothesesVeh; /* VLC CD hypotheses */
extern MEMSEC_REF VLC_DIMOutputCustom_t*
    VLC_pDIMCustDataOut; /* VLC DIM custom output */
extern MEMSEC_REF const VLC_DIMInputGeneric_t*
    VLC_pDIMGenericDataIn; /* VLC DIM generic input */
extern MEMSEC_REF const VLC_DIMInputCustom_t*
    VLC_pDIMCustDataIn; /* VLC DIM custom input */
extern MEMSEC_REF VLC_SADOutputCustom_t*
    VLC_pHEADCustDataOut; /* VLC HEAD custom output*/
extern MEMSEC_REF VLC_SADOutputGeneric_t*
    VLC_pHEADGenericDataOut; /* VLC HEAD generic output*/
extern MEMSEC_REF const VLC_SADInputGeneric_t*
    VLC_pHEADGenericDataIn; /* VLC HEAD generic input*/
extern MEMSEC_REF const VLC_SADInputCustom_t*
    VLC_pHEADCustDataIn; /* VLC HEAD custom input*/
extern MEMSEC_REF const Com_AlgoParameters_t*
    VLCVEH_pBswAlgoParameters; /* Input algo parameters from BSW */
extern MEMSEC_REF const VLC_Parameters_t*
    VLCVEH_pCPAR_VLC_Parameters; /* VLC Coding Parameters */
extern MEMSEC_REF const VED_VehSig_t*
    VLCVEH_pVehSig; /* Pointer to raw vehicle signals */
extern MEMSEC_REF const PowerTrain_t* VLCVEH_pVehSigPowerTrain;
extern MEMSEC_REF const VLC_AccLeverInput_t* VLCVEH_pAccLever;
extern MEMSEC_REF const VLC_LongCtrlInput_t* VLCVEH_pLongCtrlResp;
extern MEMSEC_REF const VLC_acc_object_t* VLCVEH_pAccDisplayObj;
extern MEMSEC_REF const VLC_acc_output_data_t* VLCVEH_pAccOutput;
extern MEMSEC_REF VLC_DFV2SenInfo_t* VLCVEH_pDFVLongOut;
extern MEMSEC_REF VLC_LongCtrlOutput_t* VLCVEH_pLongCtrlOutput;
extern MEMSEC_REF VLC_LODMCOutput_t* VLCVEH_pLODMCOutput;
extern MEMSEC_REF const VLCSenAccOOI_t* VLCVEH_pAccOOIData;
extern MEMSEC_REF VLC_DFVOutArbitrated_t* VLCVEH_pVLCVehOutArbitrated;

/* frame (cycle time, cycle counter, opmode ...) */
extern MEMSEC_REF VLCVehFrame_t VLCVehFrame;

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*! Process Custom VLC Veh Output. */
extern void VLCVEHProcessCustomOutput(void);
/* Function to fill error output */
extern void VLCVehFillErrorOut(VLC_DFVErrorOut_t* pDest);

#ifdef __cplusplus
};
#endif

/* Include has to be located at the end of vlc_veh.h
   Typedefs in this file should be moved to a seperate file and this include
   should be moved to the top.*/
#include "vlc_veh_access_func.h"

/* END of #ifndef _VLC_VEH_H_INCLUDED */
#endif
/*! @endcond Doxygen_Suppress */
/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
