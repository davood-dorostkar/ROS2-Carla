/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "TM_Global_Const.h"
#include "vlcSen_consts.h"
#include "vlcSen_common_utils.h"
#include "fip.h"
#include "stddef.h"
#include "TM_Global_Types.h"

/*****************************************************************************
  GLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(t_FIPTrafficOrientation)
static eTrafficOrientation_t t_FIPTrafficOrientation;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  LOCAL SYMOBLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  LOCAL MACROS
*****************************************************************************/

/*****************************************************************************
  LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
  LOCAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  LOCAL TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    FIP_v_InitGlobalLaneTrafficOrientation */
void FIP_v_InitGlobalLaneTrafficOrientation(void) {
    t_FIPTrafficOrientation = TRAFFICORIENTATION_UNKNOWN;
    FIP_v_Init_CustomTrafficOrientation();
}

/*************************************************************************************************************************
  Functionname:    FIP_v_TrafficOrientationProcess */
void FIP_v_TrafficOrientationProcess(void) {
    /* Get radar traffic orientation as default for FIP traffic estimation*/
    t_FIPTrafficOrientation = TRAFFICORIENTATION_UNKNOWN;

    /* Fill the Custom Traffic Orientation signals */
    FIP_v_Set_CustomTrafficOrientation();
}

/*************************************************************************************************************************
  Functionname:    FIP_t_GetTrafficOrientation */
eTrafficOrientation_t FIP_t_GetTrafficOrientation(void) {
    return t_FIPTrafficOrientation;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */