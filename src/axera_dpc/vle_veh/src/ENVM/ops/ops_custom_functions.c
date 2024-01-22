/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "ops.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
/* Scenario detection constants */
#define FPSCUSTFCT_SCENARIO_VEH_ABS_YAWRATE_MAX (float32)(0.05f)
#define FPSCUSTFCT_SCENARIO_VEH_ABS_YAWRATE_KEEP (float32)(0.02f)
#define FPSCUSTFCT_SCENARIO_VEH_EGO_VEL_MAX (float32)(82.5f / C_KMH_MS)
#define FPSCUSTFCT_SCENARIO_VEH_EGO_VEL_MIN (float32)(60.0f / C_KMH_MS)
#define FPSCUSTFCT_SCENARIO_VEH_EGO_ACCEL_MAX (float32)(0.8f)
#define FPSCUSTFCT_SCENARIO_VEH_DRV_DIST_MAX (2500.f)
#define FPSCUSTFCT_SCENARIO_VEH_DRV_DIST_VEL_RESET (float32)(10.0f / C_KMH_MS)

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* Scenario detection variables */

static float32 fDistanceSinceStopped = 0.f;
static boolean bYawRateInCorridor = TRUE;
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/
void FPS_v_InitCustomObjectList(void);

/*************************************************************************************************************************
  Functionname:    FPSInitCustom */
void FPSInitCustom(void) { FPS_v_InitCustomQuality(); }

/* ****************************************************************************

  @fn                FPS_v_ScenarioDetection                          */
void FPS_v_ScenarioDetection(void) {
    // initially assume to be on NCAP test track
    bool_t bScanarioConditionsFulfilled = TRUE;

    // get cycle time
    const float32 fCycleTime = EM_f_GetCycleTime();

    // get ego velocity of the vehicle
    const float32 fVEgo = EM_f_GetEgoObjSyncVelX();
    // get ego acceleration of the vehicle
    const float32 fAccelEgo = EM_f_GetEgoObjSyncAccelX();

    float32 fMaxAbsYawRate = FPSCUSTFCT_SCENARIO_VEH_ABS_YAWRATE_MAX;

    /* Check the distance since the last Vehicle Stop */
    if ((EGO_MOTION_STATE_RAW == VED_LONG_MOT_STATE_STDST) ||
        (fVEgo < FPSCUSTFCT_SCENARIO_VEH_DRV_DIST_VEL_RESET)) {
        fDistanceSinceStopped = 0.f;
    } else {
        fDistanceSinceStopped = fDistanceSinceStopped + (fVEgo * fCycleTime);
    }

    /* Check Ego Yawrate */
    if (bYawRateInCorridor == TRUE) {
        fMaxAbsYawRate = FPSCUSTFCT_SCENARIO_VEH_ABS_YAWRATE_MAX;
    } else {
        fMaxAbsYawRate = FPSCUSTFCT_SCENARIO_VEH_ABS_YAWRATE_KEEP;
    }

    if (fABS(EnvmData.pEgoDynObjSync->Lateral.YawRate.YawRate) <=
        fMaxAbsYawRate) {
        bYawRateInCorridor = TRUE;
    } else {
        bYawRateInCorridor = FALSE;
    }

    if ((bYawRateInCorridor == TRUE) &&
        (fAccelEgo <= FPSCUSTFCT_SCENARIO_VEH_EGO_ACCEL_MAX) &&
        (fDistanceSinceStopped <= FPSCUSTFCT_SCENARIO_VEH_DRV_DIST_MAX) &&
        (fVEgo >= FPSCUSTFCT_SCENARIO_VEH_EGO_VEL_MIN) &&
        (fVEgo <= FPSCUSTFCT_SCENARIO_VEH_EGO_VEL_MAX)) {
        bScanarioConditionsFulfilled = TRUE;
    } else {
        bScanarioConditionsFulfilled = FALSE;
    }

    // if all NCAP conditions are fullfiled in this cycle
    if (bScanarioConditionsFulfilled == TRUE) {
        // increase the time duration in which the NCAP conditions are fullfiled
        // by the current cycle-time
        EnvmData.pPrivGlob->f_TimeNCAPVehicleConditionsFullfiled =
            EnvmData.pPrivGlob->f_TimeNCAPVehicleConditionsFullfiled +
            fCycleTime;
    } else {
        // NCAP conditions are not met in this cycle, therefore, reset time
        // duration in which the NCAP conditions are fullfiled
        EnvmData.pPrivGlob->f_TimeNCAPVehicleConditionsFullfiled = 0.0f;
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */