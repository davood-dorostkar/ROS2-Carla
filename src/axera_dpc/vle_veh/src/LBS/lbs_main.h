#pragma once
#ifndef LBS_MAIN_H
#define LBS_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif

//#include "lbs.h"
//#include "lbs_ext.h"
#include "lbs_external.h"
#include "lbs_calculation.h"
// Test
//#include "tue_common_libs.h"

// #define LBS_DEVELOPMENT_DEBUG TRUE
#if LBS_DEVELOPMENT_DEBUG
void LBSInputProcess(LBSInReq_st* reqPorts, const LBSParam_st* params);
#endif

#ifndef Rte_TypeDef_StbM_TimeStampType
typedef struct
{
  uint8 timeBaseStatus;
  uint32 nanoseconds;
  uint32 seconds;
  uint16 secondsHi;
} StbM_TimeStampType;
#  define Rte_TypeDef_StbM_TimeStampType
#endif
#ifndef  Rte_TypeDef_StbM_UserDataType
typedef struct
{
  uint8 userDataLength;
  uint8 userByte0;
  uint8 userByte1;
  uint8 userByte2;
} StbM_UserDataType;
#  define Rte_TypeDef_StbM_UserDataType
#endif
//typedef unsigned  long long  uint64;

void LBSProcess(const LBSInReq_st* reqPorts,
                const LBSParam_st* params,
                LBSOutPro_t* proPorts,
                LBSDebug_t* debugInfo);
void LBSProcessLCAWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params,
                           LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo,
                           LBSCalculate_st* pLBSCalc);
boolean LBSSetFunctionOutput(boolean bFunctionActive,
                             boolean bEgoSpeedCondition);
void LBSProcessBSDWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params,
                           LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo,
                           LBSCalculate_st* pLBSCalc);
void LBSProcessOSEWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params,
                           LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo,
                           LBSCalculate_st* pLBSCalc);
void LBSProcessRCWWarnings(const LBSInReq_st* reqPorts,
                           const LBSParam_st* params,
                           LBSOutPro_t* proPorts,
                           LBSDebug_t* debugInfo,
                           LBSCalculate_st* pLBSCalc);
boolean LBSOSEWarnCalcNextState(uint8 uWarnLevel,
                                boolean bWarnThisCycle,
                                const float32 fCycletime,
                                const LBSParam_st* params,
                                LBSCalculate_st* pLBSCalc);

void LBSPostProcess(const LBSInReq_st* reqPorts,
                    const LBSParam_st* params,
                    LBSOutPro_t* proPorts,
                    LBSDebug_t* debugInfo,
                    LBSCalculate_st* pLBSCalc);
boolean LBSCheckEgoSpeedActThreshold(float32 fEgoSpeed,
                                     float32 fDisableThresh,
                                     float32 fEnableThresh,
                                     boolean* bEgoSpeedConLastCycle);
void LBSInitObjInfo(uint8 uObjectIndex);
#ifdef __cplusplus
}
#endif
#endif