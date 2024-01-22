// Copyright [2021] <Copyright Owner>" [legal/copyright]
#pragma once
#ifndef SRC_DMC_MODULE_LCF_DMC_LCF_DMC_LOCAL_MAIN_H_
#define SRC_DMC_MODULE_LCF_DMC_LCF_DMC_LOCAL_MAIN_H_
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "./lcf_dmc_local_ext.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

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

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void LcfDMCProcess(const reqLcfDmcPrtList_t* const reqPorts,
                          const reqLcfDmcParams* reqParams,
                          proLcfDmcPrtList_t* const proPorts,
                          proLcfDmcDebug_t* proDebugs);
static void LaDMCInputWrapper(const reqLcfDmcPrtList_t* pLCFDmcInput,
                              const reqLcfDmcParams* reqParams,
                              sTJADMCInReq_t* pDMCInput);
static void LaDMCOutputWrapper(const sTJADMCOutPro_t sDMCOutput,
                               const sTJADMCDebug_t sDMCDebug,
                               proLcfDmcPrtList_t* pLCFDmcOutput,
                               proLcfDmcDebug_t* pLcfDmcDebug);
#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_
        // DECISION_SRC_LCF_DMC_LCF_DMC_LOCAL_MAIN_H_
