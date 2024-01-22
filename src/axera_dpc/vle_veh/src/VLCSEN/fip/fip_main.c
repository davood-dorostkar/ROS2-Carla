/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "TM_Global_Types.h"
#include "si_ext.h"
#include "fip.h"
#include "fip_ext.h"

/*****************************************************************************
  GLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(FIPState)
FIPState_t FIPState; /*!< Operating modes of FIP sub-component */
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

static void FIPInit(void); /*!< Initialization of VLC Input Preprocessing */

/*************************************************************************************************************************
  Functionname:    FIPProcess */
void FIPProcess(void) {
    switch (FIPState) {
        case FIP_OK:
            /*! FIP Object Trace Preprocessing */
            FIP_v_ObjTraceProcess();
            /* Traffic Orientation Processing */
            FIP_v_TrafficOrientationProcess();
            /* Road Type Processing */
            FIP_v_RoadProcess();
            FIP_v_Process_FuseRoadType();
            /*! VLC Camera Lane Data Preprocessing */
            FIP_v_CamLaneDataProcess();

            /*! FIP Lane Matrix Preprocessing */
            FIP_v_LaneMatrixProcess();
            break;

        case FIP_INIT:
        /*! Initialization */
        default:
            /*! Default */
            FIPInit();
            break;
    }
}

/*************************************************************************************************************************
  Functionname:    FIPInit */
static void FIPInit(void) {
    /*! Initialization of global FIP object trace data */
    FIP_v_InitGlobalObjTraceData();
    /*! Initialization of global NAVI data */

    /*! Initialization of global camera lane data */
    FIP_v_InitGlobalCamLaneData();

    /*! Initialization of global FIP lane matrix data */
    FIP_v_InitGlobalLaneMatrixData();
    FIP_v_InitLaneWidthInfo();
    /*! Initialization of global FIP traffic orientation data */
    FIP_v_InitGlobalLaneTrafficOrientation();
    /*! Initialization of global FIP road type data */
    FIP_v_Init_Road();
    FIP_v_Init_FuseRoadType();

    return;
}

/* ************************************************************************* */
/*   Copyright                                               */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */