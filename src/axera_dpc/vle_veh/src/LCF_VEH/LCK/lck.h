

#ifndef _LCK_H_INCLUDED
#define _LCK_H_INCLUDED
/*** START OF SINGLE INCLUDE SECTION ****************************************/

/*****************************************************************************
  INCLUDES
*****************************************************************************/
//#include "fct_veh.h"
#include "lck_ext.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/
#ifndef C_PI
#define C_PI ((float32)3.14159265359f)
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg_) ((deg_) * (C_PI / 180.F))
#endif
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
 *                                                                           *
 *     LCK input data                                                        *
 *                                                                           *
 *****************************************************************************/

/*****************************************************************************
  GLOBAL CONSTANTS (EXTERNAL SCOPE)
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES (EXTERNAL SCOPE)
*****************************************************************************/
/* debug output */
static sLCKDebug_t sLCKDebug;
extern sLCKDebugData_t *pLCKGlobalDebugData;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
extern eGDBError_t LCKRun(sLCKInput_t const *pInputData,
                          sLCKParam_t const *pParam,
                          sLCKOutput_t *pLCKOutput,
                          float32 const fCycleTime);

// extern  P2VAR(FCTLKSOutputLCD_t , AUTOMATIC,   RTE_CODE)
// Rte_IRead_SWCLKA_RPort_FCTLKSOUTLCD(void);
//#define Rte_IRead_RunEntNorm_RPort_FCTLKSOUTLCD
// Rte_IRead_SWCLKA_RPort_FCTLKSOUTLCD
//
// extern  P2VAR(FCTLKSOutputLCD_t , AUTOMATIC,   RTE_CODE)
// Rte_IWrite_SWCLKA_RPort_FCTLKSOUTLCD(void);
//#define Rte_IWrite_RunEntNorm_RPort_FCTLKSOUTLCD
// Rte_IWrite_SWCLKA_RPort_FCTLKSOUTLCD
//
// extern  P2VAR(FCTLKSOutputLCK_t , AUTOMATIC,   RTE_CODE)
// Rte_IRead_SWCLKA_RPort_FCTLCKOUTLCK(void);
//#define Rte_IRead_RunEntNorm_RPort_FCTLCKOUTLCK
// Rte_IRead_SWCLKA_RPort_FCTLCKOUTLCK
//
// extern  P2VAR(FCTLKSOutputLCK_t , AUTOMATIC,   RTE_CODE)
// Rte_IWrite_SWCLKA_RPort_FCTLCKOUTPUTLCK(void);
//#define Rte_IWrite_RunEntNorm_RPort_FCTLCKOUTPUTLCK
// Rte_IWrite_SWCLKA_RPort_FCTLCKOUTPUTLCK
//
// extern  P2VAR(sLCKInput_t, AUTOMATIC,   RTE_CODE)
// Rte_IRead_SWCLKA_RPort_FCTLKSINPUTLCK(void);
//#define Rte_IRead_RunEntNorm_RPort_FCTLKSINPUTLCK
// Rte_IRead_SWCLKA_RPort_FCTLKSINPUTLCK
//
// extern  P2VAR(sLCKInput_t, AUTOMATIC,   RTE_CODE)
// Rte_IWrite_SWCLKA_RPort_FCTLKSINPUTLCK(void);
//#define Rte_IWrite_RunEntNorm_RPort_FCTLKSINPUTLCK
// Rte_IWrite_SWCLKA_RPort_FCTLKSINPUTLCK
//
//
// extern  P2VAR(VehDyn_t, AUTOMATIC, RTE_CODE)
// Rte_IWriteRef_SWCEmAdapt_RunEntEmMain_PPortEmObjSyncVehDyn_DEPVehDynEmObjSync(void);
//#define Rte_IWriteRef_RunEntEmMain_PPortEmObjSyncVehDyn_DEPVehDynEmObjSync
// Rte_IWriteRef_SWCEmAdapt_RunEntEmMain_PPortEmObjSyncVehDyn_DEPVehDynEmObjSync
//
// extern P2VAR(VehDyn_t, AUTOMATIC, RTE_CODE)
// Rte_IRead_SWCDFVAdapt_RunEntDFVMain_RPortVehDynVdy_DEPVehDynVdy(void);
//#define Rte_IRead_RunEntDFVMain_RPortVehDynVdy_DEPVehDynVdy
// Rte_IRead_SWCDFVAdapt_RunEntDFVMain_RPortVehDynVdy_DEPVehDynVdy

/*****************************************************************************
  INLINE FUNCTION
*****************************************************************************/

/*** END OF SINLGE INCLUDE SECTION ******************************************/
#endif /* _LCK_H_INCLUDED */
