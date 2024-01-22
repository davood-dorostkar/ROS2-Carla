#ifndef LCF_SEN_EXT_H
#define LCF_SEN_EXT_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"

#include "rtwtypes.h"
#include "lcf_sen_local_ext.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
/* typedef BaseReturnCode_t */
#ifndef LCF_SEN_FRAMEWORK_VERSION
#define LCF_SEN_FRAMEWORK_VERSION 20210610
#endif

#ifndef BASE_RETURN_OK
#define BASE_RETURN_OK (0)
#endif
#ifndef BASE_RETURN_ERROR
#define BASE_RETURN_ERROR (4294967295)
#endif

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LCF_SenReset(void);
uint32 LCF_SenProcess(void);

const proLcfSenPrtList_t* Rte_IRead_SWLCF_ProLcfSen_Out(
    void);  // LCF sen output RTE wrapper

#ifdef __cplusplus
}
#endif
#endif
