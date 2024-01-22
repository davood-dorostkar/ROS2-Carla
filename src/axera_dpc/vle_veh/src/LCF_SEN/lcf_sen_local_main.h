#ifndef LCF_SEN_LOCAL_MAIN_H
#define LCF_SEN_LOCAL_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcf_sen_local_ext.h"
#include "tue_common_libs.h"
#include "odpr_fop.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/* typedef uEgoMotionState_nu */
#ifndef VED_LONG_MOT_STATE_MOVE
#define VED_LONG_MOT_STATE_MOVE 0U
#endif
#ifndef VED_LONG_MOT_STATE_MOVE_FWD
#define VED_LONG_MOT_STATE_MOVE_FWD 1U
#endif
#ifndef VED_LONG_MOT_STATE_MOVE_RWD
#define VED_LONG_MOT_STATE_MOVE_RWD 2U
#endif
#ifndef VED_LONG_MOT_STATE_STDST
#define VED_LONG_MOT_STATE_STDST 3U
#endif

/* typedef BaseOpMode_t */
#ifndef BASE_OM_IDLE
#define BASE_OM_IDLE (0)
#endif
#ifndef BASE_OM_RESET
#define BASE_OM_RESET (1)
#endif
#ifndef BASE_OM_MAX_RUNTIME
#define BASE_OM_MAX_RUNTIME (2)
#endif
#ifndef BASE_OM_DEMO
#define BASE_OM_DEMO (3)
#endif
#ifndef BASE_OM_RUN
#define BASE_OM_RUN (4)
#endif

/* typedef eCompState */
#ifndef COMP_STATE_NOT_INITIALIZED
#define COMP_STATE_NOT_INITIALIZED (0)
#endif
#ifndef COMP_STATE_RUNNING
#define COMP_STATE_RUNNING (1)
#endif
#ifndef COMP_STATE_TEMPORARY_ERROR
#define COMP_STATE_TEMPORARY_ERROR (2)
#endif
#ifndef COMP_STATE_PERMANENT_ERROR
#define COMP_STATE_PERMANENT_ERROR (3)
#endif
#ifndef COMP_STATE_SUCCESS
#define COMP_STATE_SUCCESS (4)
#endif
#ifndef COMP_STATE_REDUCED_AVAILABILITY
#define COMP_STATE_REDUCED_AVAILABILITY (5)
#endif
#ifndef COMP_STATE_NOT_RUNNING
#define COMP_STATE_NOT_RUNNING (6)
#endif

/* typedef eGenAlgoQualifier */
#ifndef ALGO_QUAL_OK
#define ALGO_QUAL_OK (0)
#endif
#ifndef ALGO_QUAL_CRITICAL_INPUT_ERROR
#define ALGO_QUAL_CRITICAL_INPUT_ERROR (1)
#endif
#ifndef ALGO_QUAL_NOT_CRITICAL_INPUT_ERROR
#define ALGO_QUAL_NOT_CRITICAL_INPUT_ERROR (2)
#endif
#ifndef ALGO_QUAL_BLOCKAGE
#define ALGO_QUAL_BLOCKAGE (4)
#endif
#ifndef ALGO_QUAL_PARTIAL_BLOCKAGE
#define ALGO_QUAL_PARTIAL_BLOCKAGE (8)
#endif
#ifndef ALGO_QUAL_CALIBRATION_ERROR_TOO_HIGH
#define ALGO_QUAL_CALIBRATION_ERROR_TOO_HIGH (16)
#endif
#ifndef ALGO_QUAL_GENERAL_FUNCTION_ERROR
#define ALGO_QUAL_GENERAL_FUNCTION_ERROR (32)
#endif
#ifndef ALGO_QUAL_NO_VISIBILITY
#define ALGO_QUAL_NO_VISIBILITY (64)
#endif
#ifndef ALGO_QUAL_LIMITED_VISIBILITY
#define ALGO_QUAL_LIMITED_VISIBILITY (128)
#endif
#ifndef ALGO_QUAL_CALIBRATION_RUNNING
#define ALGO_QUAL_CALIBRATION_RUNNING (256)
#endif
#ifndef ALGO_QUAL_MAX
#define ALGO_QUAL_MAX (4294967295)
#endif

/* typedef eSigStatus */
#ifndef AL_SIG_STATE_INIT
#define AL_SIG_STATE_INIT (0)
#endif
#ifndef AL_SIG_STATE_OK
#define AL_SIG_STATE_OK (1)
#endif
#ifndef AL_SIG_STATE_INVALID
#define AL_SIG_STATE_INVALID (2)
#endif

/* typedef eShedulerSubModeRequest */
#ifndef BASE_SSM_NONE
#define BASE_SSM_NONE (0)
#endif
#ifndef BASE_SSM_BLOCKAGE
#define BASE_SSM_BLOCKAGE (1)
#endif
#ifndef BASE_SSM_DEGRADED
#define BASE_SSM_DEGRADED (2)
#endif
#ifndef BASE_SSM_STANDALONE
#define BASE_SSM_STANDALONE (4)
#endif
#ifndef BASE_SSM_PARALLEL
#define BASE_SSM_PARALLEL (8)
#endif
#ifndef BASE_SSM_NOT_CALIBRATED
#define BASE_SSM_NOT_CALIBRATED (16)
#endif

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#if 1
// ABPR module global values
extern sLBPInput_t g_LCF_LBPInput;
extern sLBPParam_t g_LCF_LBPParam;
extern sLBPOutput_t g_LCF_LBPOutput;

// ALP module global values
extern sALPInReq_st g_LCF_ALPInput;
extern sALPParam_st g_LCF_ALPParam;
extern sALPOutput_st g_LCF_ALPOutput;

// ODPR module global values
extern ODPRInReq_t g_LCF_ODPR_Input;
extern ODPRParam_t g_LCF_ODPR_Param;
extern ODPROutPro_t g_LCF_ODPR_Output;

// VSDP module global values
extern sVSDPInput_t g_LCF_VSDP_Input;
extern sVSDPParam_t g_LCF_VSDP_Param;
extern sVSDPOutput_t g_LCF_VSDP_Output;

// LDWSA module global values
extern sLDWSAInReq_t g_LCF_LDWSA_Input;
extern sLDWSAParam_t g_LCF_LDWSA_Param;
extern sLDWSAOutPro_t g_LCF_LDWSA_Output;

// LDPSA module global values
extern sLDPSAInput_t g_LCF_LDPSA_Input;
extern sLDPSAParam_t g_LCF_LDPSA_Param;
extern sLDPSAOutput_t g_LCF_LDPSA_Output;

// TJASA module global values
extern sTJASAInReq_t g_LCF_TJASA_Input;
extern sTJASAParam_t g_LCF_TJASA_Param;
extern sTJASAOutPro_t g_LCF_TJASA_Output;

// MCTLFC module global values
extern sMCTLFCInReq_st g_LCF_MCTLFC_Input;
extern sMCTLFCParam_st g_LCF_MCTLFC_Param;
extern sMCTLFCOut_st g_LCF_MCTLFC_Output;
#endif

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void LcfSenProcess(const reqLcfSenPrtList_t* const reqPorts,
                          const reqLcfSenParams* reqParams,
                          proLcfSenPrtList_t* const proPorts,
                          reqLcfSenDebug_t* proDebugs);
static void LBPInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                            sLBPInput_t* pLBPInput);
static void LBPOutputWrapper(const sLBPOutput_t sLBPOutput,
                             proLcfSenPrtList_t* LCFSenOutput);
static void ALPInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                            const sLBPOutput_t sLBPOutput,
                            const reqLcfSenParams* pParams,
                            sALPInReq_st* pALPInput);
static void ALPOutputWrapper(const sALPOutput_st sALPOutput,
                             proLcfSenPrtList_t* LCFSenOutput);
static void VSDPInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                             const reqLcfSenParams* pParams,
                             sVSDPInput_t* pVSDPInput);
static void LDWSAInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                              const sVSDPOutput_t sVSDPOutput,
                              const sLBPOutput_t sLBPOutput,
                              const reqLcfSenParams* pParams,
                              const sTJASAOutPro_t sTJASAOutput,
                              const sTJASADebug_t debug,
                              sLDWSAInReq_t* sLDWSAInput);
static void LDWSAOutputWrapper(const sLDWSAOutPro_t sLDWSAOutput,
                               proLcfSenPrtList_t* pLCFSenOutput);
static void LDPSAInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                              const sVSDPOutput_t sVSDPOutput,
                              const sLBPOutput_t sLBPOutput,
                              const reqLcfSenParams* pParams,
                              const sTJASAOutPro_t sTJASAOutput,
                              const sTJASADebug_t debug,
                              sLDPSAInput_t* pLDPSAInput);
static void LDPSAOutputWrapper(const sLDPSAOutput_t sLDPSAOutput,
                               proLcfSenPrtList_t* pLCFSenOutput);
static void ODPRInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                             const sLBPOutput_t sLBPOutput,
                             const reqLcfSenParams* pParams,
                             ODPRInReq_t* pODPRInput);
static void TJASAInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                              const reqLcfSenParams* pLCFParam,
                              const sLBPOutput_t sLBPOutput,
                              const sALPOutput_st sALPOutput,
                              const ODPROutPro_t sODPROutput,
                              const sVSDPOutput_t sVSDPOutput,
                              const sLCCRAOutPro_t sLCCRAOUtput,
                              sTJASAInReq_t* pTJASAInput);
static void TJASAOutputWrapper(const sTJASAOutPro_t sTJASAOutput,
                               proLcfSenPrtList_t* pLCFSenOutput);
static void MCTLFCInputWrapper(const reqLcfSenPrtList_t* pLCFInput,
                               const reqLcfSenParams* pLCFParam,
                               const sTJASAOutPro_t* sTJASAOutput,
                               const sLDPSAOutput_t sLDPSAOutput,
                               const sLDPSADebug_t sLDPSADebug,
                               sMCTLFCInReq_st* pMCTLFCInput);
static void MCTLFCOutputWrapper(const sMCTLFCOut_st sMCTLFCOutput,
                                proLcfSenPrtList_t* pLCFSenOutput);
static void LCFOPSInputWrapper(const reqLcfSenPrtList_t* LCFInput,
                               sLCFOPSInReq_t* LCFOPSIn);
static void LCCRAInputWrapper(const reqLcfSenPrtList_t* LCFInput,
                              sLCFOPSOutPro_t LCFOPSOutput,
                              sLBPOutput_t LBPOutput,
                              sALPOutput_st ALPOutput,
                              sLCCRAInReq_t* LCCRAIn);
static void LCF_SEN_ResetParams(const reqLcfSenParams* pLCFInputParam,
                                sLBPParam_t* pLBPParam,
                                sALPParam_st* pALPParam,
                                ODPRParam_t* pODPRParam,
                                sVSDPParam_t* pVSDPParam,
                                sLDWSAParam_t* pLDWSAParam,
                                sLDPSAParam_t* pLDPSAParam,
                                sTJASAParam_t* pTJASAParam,
                                sMCTLFCParam_st* pMCTLFCParam,
                                sLCCRAParam_t* pLCCRAParam,
                                sLCFOPSParam_t* pLCFOPSParam);
static void LcfSenSetInfoDataProPorts(const reqLcfSenPrtList_t* const reqPorts,
                                      proLcfSenPrtList_t* const proPorts,
                                      uint16 uiCycleCnt);
void LcfSenSetSigHeader(LCF_SenSignalHeader_t* const psSigHeader,
                        uint8 uiSigState,
                        uint32 uiCtrlTmp,
                        uint16 uiCtrlMeasCnt,
                        uint16 uiCycleCnt);

#ifdef __cplusplus
}
#endif
#endif
