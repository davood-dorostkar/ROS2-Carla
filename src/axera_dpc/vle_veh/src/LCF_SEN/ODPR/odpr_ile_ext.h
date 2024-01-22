#ifndef ODPR_ILE_EXT_H
#define ODPR_ILE_EXT_H
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "odpr_ile.h"

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
extern void ODPR_ILE_Reset(void);
extern void ODPR_ILE_Exec(const ILEInReq_t* reqPorts,
                          const ODPRParam_t* param,
                          ILEOutPro_t* proPorts,
                          ILEDebug_t* debugInfo);

#endif