/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "fip.h"

#include "TM_Global_Const.h"
#include "vlcSen_consts.h"
#include "vlcSen_common_utils.h"
#include "fip_ext.h"

/*****************************************************************************
  GLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES
*****************************************************************************/

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
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(FIPFusedRoadEstimation)
static FIP_t_RoadEstimation FIPRoadEstimation; /*! FIP Road Estimation */

SET_MEMSEC_VAR(FIPFusedRoadEstimation)
static FIP_t_RoadFusedBorder FIPRoadFusedBorder; /*! FIP Road Fused Border */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
const FIP_t_RoadEstimation
    *FIP_p_RoadEstimation; /*! Pointer to FIP_t_RoadEstimation structure */
const FIP_t_RoadFusedBorder
    *FIP_p_RoadFusedBorder; /*! Pointer to FIP_t_RoadFusedBorder structure */

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void FIP_v_Init_FIPRoadEstimation(
    void); /*! Initialize FIP Road Estimation Struture */
static void FIP_v_Init_FIPRoadBorder(
    void); /*! Initialize FIP Road Fused Border Struture */

static void FIP_v_SetRoadEstimation(
    void); /*! Set FIP Road Estimation struture with EM Road Estimation Values
              */
static void FIP_v_SetRoadBorder(
    void); /*! Set FIP Road Fused Border struture with EM Fused Road Border*/

/*****************************************************************************
  FUNCTION DEFINITIONS
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:     FIP_v_Init_Road */
void FIP_v_Init_Road(void) {
    /*! Initialize FIP Road Estimation */
    FIP_v_Init_FIPRoadEstimation();
    /*! Initialize FIP Road Border */
    FIP_v_Init_FIPRoadBorder();
}

/*************************************************************************************************************************
  Functionname:    FIP_v_Road_Process */
void FIP_v_RoadProcess(void) {
    /*! Set FIP Road Estimation with EM Road Estimation values */
    FIP_v_SetRoadEstimation();
    /*! Set FIP Road Border with EM Road Border values */
    FIP_v_SetRoadBorder();
}

/*************************************************************************************************************************
  Functionname:    FIP_v_Init_FIPRoadEstimation */
static void FIP_v_Init_FIPRoadEstimation(void) {
    FIPRoadEstimation.fC0 = 0.f;
    FIPRoadEstimation.fC1 = 0.f;
    FIPRoadEstimation.fYawAngle = 0.f;
    FIPRoadEstimation.fRangeMaxRight = 0.f;
    FIPRoadEstimation.fRangeMaxLeft = 0.f;
    FIPRoadEstimation.fYLeft = 0.f;
    FIPRoadEstimation.fYRight = 0.f;
    FIPRoadEstimation.uiTrackingStatus = 0u;
    FIPRoadEstimation.uiTrackingStatusLeft = 0u;
    FIPRoadEstimation.uiTrackingStatusRight = 0u;
    FIPRoadEstimation.uiConfidence = 0u;
    FIPRoadEstimation.fminXLeft = 0.f;
    FIPRoadEstimation.fminXRight = 0.f;
    FIPRoadEstimation.fmaxXLeftCompensated = 0.f;
    FIPRoadEstimation.fmaxXRightCompensated = 0.f;
    FIPRoadEstimation.fLatStdDevFiltered = 0.f;
    FIPRoadEstimation.fConfidenceLeft = 0.f;
    FIPRoadEstimation.fConfidenceRight = 0.f;

    FIP_p_RoadEstimation = &FIPRoadEstimation;
}

/*************************************************************************************************************************
  Functionname:    FIP_v_Init_FIPRoadBorder */
static void FIP_v_Init_FIPRoadBorder(void) {
    FIPRoadFusedBorder.fDistRight = 0.f;
    FIPRoadFusedBorder.fDistRightStd = 0.f;
    FIPRoadFusedBorder.fDistLeft = 0.f;
    FIPRoadFusedBorder.fDistLeftStd = 0.f;
    FIPRoadFusedBorder.bStatusRight = 0u;
    FIPRoadFusedBorder.bStatusLeft = 0u;

    FIP_p_RoadFusedBorder = &FIPRoadFusedBorder;
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetRoadEstimation */
static void FIP_v_SetRoadEstimation(void) {
    /* Use Radar Road values if availble */
    FIPRoadEstimation.fC0 = 0.f;
    FIPRoadEstimation.fC1 = 0.f;
    FIPRoadEstimation.fYawAngle = 0.f;
    FIPRoadEstimation.fRangeMaxRight = 0.f;
    FIPRoadEstimation.fRangeMaxLeft = 0.f;
    FIPRoadEstimation.fYLeft = 0.f;
    FIPRoadEstimation.fYRight = 0.f;
    FIPRoadEstimation.uiTrackingStatus = 0u;
    FIPRoadEstimation.uiTrackingStatusLeft = 0u;
    FIPRoadEstimation.uiTrackingStatusRight = 0u;
    FIPRoadEstimation.uiConfidence = 0u;
    FIPRoadEstimation.fminXLeft = 0.f;
    FIPRoadEstimation.fminXRight = 0.f;
    FIPRoadEstimation.fmaxXLeftCompensated = 0.f;
    FIPRoadEstimation.fmaxXRightCompensated = 0.f;
    FIPRoadEstimation.fLatStdDevFiltered = 0.f;
    FIPRoadEstimation.fConfidenceLeft = 0.f;
    FIPRoadEstimation.fConfidenceRight = 0.f;
}

/*************************************************************************************************************************
  Functionname:    FIP_v_SetRoadBorder */
static void FIP_v_SetRoadBorder(void) {
    FIPRoadFusedBorder.fDistRight = 0.f;
    FIPRoadFusedBorder.fDistRightStd = 0.f;
    FIPRoadFusedBorder.fDistLeft = 0.f;
    FIPRoadFusedBorder.fDistLeftStd = 0.f;
    FIPRoadFusedBorder.bStatusRight = 0u;
    FIPRoadFusedBorder.bStatusLeft = 0u;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */