// Copyright [2021] <Copyright Owner>" [legal/copyright]
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LCF_VEH_LOCAL_MAIN_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LCF_VEH_LOCAL_MAIN_H_
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "./lcf_veh_local_ext.h"

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
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void LcfVehProcess(const reqLcfVehPrtList_t* const reqPorts,
                          const reqLcfVehParams* reqParams,
                          proLcfVehPrtList_t* const proPorts,
                          reqLcfVehDebug_t* proDebugs);
static void TPInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                           const reqLcfVehParams* pLcfVehParams,
                           TRJPLN_TrajectoryPlanInReq_t* pTPInput);
static void TPOutputWrapper(const TRJPLN_TrajectoryPlanInReq_t sTPInput,
                            const TRJPLN_TrajectoryPlanOutPro_t sTPOutput,
                            const TRJPLN_TrajectoryPlanDebug_t sTPDebug,
                            proLcfVehPrtList_t* pLCFVehOutput);
static void TCInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                           const reqLcfVehParams* pLcfVehParams,
                           const TRJPLN_TrajectoryPlanOutPro_t* pTPOutput,
                           sTJATCTInReq_st* pTCInput);
static void TCOutputWrapper(const sTJATCTDebug_st sTCDebug,
                            const sTJATCTOut_st sTCOutput,
                            proLcfVehPrtList_t* pLCFVehOutput);
static void TCInputOutWrapper(const sTJATCTInReq_st pTCInput,
                              proLcfVehPrtList_t* pLCFVehOutput);
static void LCKInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                            const reqLcfVehParams* pLcfVehParams,
                            sLCKInput_t* pLCKInput);
static void LCDInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                            const reqLcfVehParams* pLcfVehParams,
                            const sLCKInput_t* pLCKInput,
                            const sLCKOutput_t* pLCKOutput,
                            sLCDInput_t* pLCDInput);
static void LCDOutputWrapper(const sLCDOutput_t sLCDOutput,
                             proLcfVehPrtList_t* pLCFVehOutput);
static void LCF_VEH_ResetParams(const reqLcfVehParams* pLCFInputParam,
                                TRJPLN_TrajectoryPlanParam_t* pTPParam,
                                sTJATCTParam_st* pTCParam,
                                sLCKParam_t* pLCKParams,
                                sLCDParams_t* pLCDParams);
static void LcfVehSetInfoDataProPorts(const reqLcfVehPrtList_t* const reqPorts,
                                      proLcfVehPrtList_t* const proPorts,
                                      uint16 uiCycleCnt);
void LcfVehSetSigHeader(LCF_VehSignalHeader_t* const psSigHeader,
                        uint8 uiSigState,
                        uint32 uiCtrlTmp,
                        uint16 uiCtrlMeasCnt,
                        uint16 uiCycleCnt);
static void HODInputWrapper(const reqLcfVehPrtList_t* pLCFVehInput,
                            const reqLcfVehParams* pLcfVehParams,
                            sHODInput_t* pHODInput);
static void HODOutputWrapper(sHODDebug_t sHODDebug,
                             sHODOutput_t sHODOutput,
                             proLcfVehPrtList_t* pLCFVehOutput);

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_
        // DECISION_SRC_LCF_VEH_LCF_VEH_LOCAL_MAIN_H_
