#ifndef MCTLFC_MAIN_H
#define MCTLFC_MAIN_H
#include "MCTLFC_ext.h"

#define MCTLFC_VIRSION 20210316

void MCTLFC_Init(void);
void MCTLFC_Input_initialize(const sMCTLFCInReq_st* reqPorts);
void MCTLFC_Output_initialize(sMCTLFCOut_st* proPorts,
                              sMCTLFCDebug_st* debugInfo);
void MCTLFC_Exec(const sMCTLFCInReq_st* reqPorts,
                 const sMCTLFCParam_st* params,
                 sMCTLFCOut_st* proPorts,
                 sMCTLFCDebug_st* debugInfo);

#endif