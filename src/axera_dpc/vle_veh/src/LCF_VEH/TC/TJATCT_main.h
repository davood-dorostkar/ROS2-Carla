#ifndef TJATCT_MAIN_H
#define TJATCT_MAIN_H
#include "TJATCT_ext.h"

#define TJATCT_VIRSION 20210316

void TJATCT_Init(void);
void TJATCT_Input_initialize(const sTJATCTInReq_st* reqPorts);
void TJATCT_Output_initialize(sTJATCTOut_st* proPorts,
                              sTJATCTDebug_st* debugInfo);
void TJATCT_Exec(const sTJATCTInReq_st* reqPorts,
                 const sTJATCTParam_st* params,
                 sTJATCTOut_st* proPorts,
                 sTJATCTDebug_st* debugInfo);

#endif