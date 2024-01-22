#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LCF_VEH_EXT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LCF_VEH_EXT_H_
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "rtwtypes.h"
#include "lcf_veh_local_ext.h"

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
void LCF_VehReset(void);

uint32 LCF_VehProcess(void);

const proLcfVehPrtList_t* Rte_IRead_SWLCF_ProLcfVeh_Out(
    void);  // LCF veh output RTE wrapper

#ifdef __cplusplus
}
#endif
#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_LCF_VEH_EXT_H_
