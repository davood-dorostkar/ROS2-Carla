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
#include "cp_ext.h"
#include "emp_ext.h"
#include "si_ext.h"
#include "fip_ext.h"
#include "frame_sen_custom_types.h"
#include "vlc_ver.h"

#include "fip_object_traces.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
LOCAL SYMOBLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
LOCAL TYPEDEFSfip_object_traces.c
*****************************************************************************/

/*****************************************************************************
GLOBAL VARIABLES
*****************************************************************************/

/*****************************************************************************
LOCAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*ego motion attributes structures*/
SET_MEMSEC_VAR(FIP_TrMat2DCOFFwdTgtSync)
static GDBTrafoMatrix2D_t FIP_TrMat2DCOFFwdTgtSync;

SET_MEMSEC_VAR(FIP_TrMat2DCOFForJitTgtSync)
static GDBTrafoMatrix2D_t FIP_TrMat2DCOFForJitTgtSync;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
FUNCTION PROTOTYPES
*****************************************************************************/

/*get Variance of Object depending on Distance*/
float32 FIP_f_GetObjObservationVariance(ObjNumber_t iObj);
/* update the motion attributes for the Ego vehicle*/
static void FIP_v_UpdateEgoMotion(void);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/
/*************************************************************************************************************************
  Functionname:    FIP_v_ObjTraceProcess */
void FIP_v_ObjTraceProcess(void) {
    /*Update Ego motion Matrices*/
    FIP_v_UpdateEgoMotion();

    FIP_v_CalculateMovingObjectStaticTraces();
}

/*************************************************************************************************************************
  Functionname:    FIP_v_InitGlobalObjTraceData */
void FIP_v_InitGlobalObjTraceData(void) {
    /* Static traces either from EM or computed in FIP */
    FIP_v_Init_Static_Traces();
}

/*************************************************************************************************************************
  Functionname:    FIP_f_GetObjObservationVariance */
float32 FIP_f_GetObjObservationVariance(ObjNumber_t iObj) {
    /*assuming object Angle-Uncertainty of 1?
    approx of Y-Uncertainty with linear function => tan(0.5?*/
    const float32 fTanUncAngle = 0.008726867790759f;
    float32 f_ret;

    f_ret = SQR(fTanUncAngle * OBJ_LONG_DISPLACEMENT(iObj));
    f_ret = MAX_FLOAT(f_ret, OBJ_LAT_DISPLACEMENT_VAR(iObj));
    return f_ret;
}

/* **********************************************************************
Functionname:     FIPGetTrafoMatrix2DCOFFwdTgtSync               */
const GDBTrafoMatrix2D_t* FIPGetTrafoMatrix2DCOFFwdTgtSync(void) {
    return &FIP_TrMat2DCOFFwdTgtSync;
}

/*************************************************************************************************************************
  Functionname:    FIPGetTrafoMatrix2DCOFForJitTgtSync */
const GDBTrafoMatrix2D_t* FIPGetTrafoMatrix2DCOFForJitTgtSync(void) {
    return &FIP_TrMat2DCOFForJitTgtSync;
}

/*************************************************************************************************************************
  Functionname:    FIP_v_UpdateEgoMotion */
static void FIP_v_UpdateEgoMotion(void) {
    /*!@todo adaption of all egomotion matrices to new ego functions*/
    GDBmathCalcCOFEgomotionMatrices(
        &FIP_TrMat2DCOFFwdTgtSync, &FIP_TrMat2DCOFForJitTgtSync,
        EGO_SPEED_X_OBJ_SYNC, EGO_ACCEL_X_OBJ_SYNC, EGO_YAW_RATE_OBJ_SYNC,
        EGO_YAW_RATE_VAR_OBJ_SYNC, EGO_YAW_RATE_MAX_JITTER_OBJ_SYNC,
        SENSOR_X_POSITION_CoG, SENSOR_Y_POSITION, EGO_SIDE_SLIP_OBJ_SYNC,
        EGO_SIDE_SLIP_VAR_OBJ_SYNC, TASK_CYCLE_TIME);
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */