/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cp_si.h"
/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
/*! application parameters for restriction of trace bracket extensions*/
typedef struct {
    uint8 LaneChangeProbThresMin; /*!< Min. Threshold to restrict trace bracket
                                     extentions   @min:0 @max: 100 */
    uint8 LaneChangeProbThresMax; /*!< Max Threshold to restrict trace bracket
                                     extentions   @min:0 @max: 100 */
    uint8 LaneChangeTimer; /*!< Timer not to make object relevant at once again
                              @min:0 @max: 100 */
} CPCustomParameterLaneChange_t; /*!< @allow: all_cust */

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/********************************/
/* lane change parameters       */
/********************************/

/*! Threshold to restrict trace bracket extentions    min:00  max: 100 */
#define CP_CUSTOM_LANE_CHANGE_MIN_PROB_THRES 0u
#define CP_CUSTOM_LANE_CHANGE_MAX_PROB_THRES 1u
/*! Timer not to make object relevant at once again     @min:00 @max: 100 */
#define CP_CUSTOM_LANE_CHANGE_TIMER 20u

#define VLC_MEAS_ID_CP_CUSTOM_OVERTAKE 0x202B1000

/*****************************************************************************
  VARIABLES
*****************************************************************************/
SET_MEMSEC_VAR(CPCustomLaneChangeParameters)
/*! application parameters for restriction of trace bracket extensions*/
static const CPCustomParameterLaneChange_t CPCustomLaneChangeParameters =

    {
        CP_CUSTOM_LANE_CHANGE_MIN_PROB_THRES,
        CP_CUSTOM_LANE_CHANGE_MAX_PROB_THRES, CP_CUSTOM_LANE_CHANGE_TIMER,
};
SET_MEMSEC_VAR(CPCustomOvertakeAssistance)
/*! The lane change overtake assist states */
static struct {
    uint8 uiLaneChgProb;   /*!< The current lane change probability  @min:0
                              @max:100 */
    uint8 uiLaneChgProbLC; /*!< The lane change probability from the last cycle
                              @min:0 @max:100 */
    ObjNumber_t iOvertakeObj; /*!< The ID of the object being overtaken (recent
                                 relevant object ID) */
    uint8 ucLaneChgDownCnt;   /*!< Lane change down-counter (when non-zero, then
                                 lane change is active) */
} CPCustomOvertakeAssistance;

/*****************************************************************************
  FUNCTION
*****************************************************************************/

static void CPSIInit(CPCourseData_t* pCourseData,
                     CPTrajectoryData_t* pTrajectoryData);
static void CPSICustomCourseProc(CPCourseData_t* pCourseData);

/*************************************************************************************************************************
  Functionname:    CPSIInit */
static void CPSIInit(CPCourseData_t* pCourseData,
                     CPTrajectoryData_t* pTrajectoryData) {
    _PARAM_UNUSED(pCourseData);
    _PARAM_UNUSED(pTrajectoryData);
    /* CPRESTRUCTED: one init-function is enough */
}

/*************************************************************************************************************************
  Functionname:    CPSICustomCourseProc */
static void CPSICustomCourseProc(CPCourseData_t* pCourseData) {
    // pCourseData = pCourseData;
    _PARAM_UNUSED(pCourseData);
}

/*************************************************************************************************************************
  Functionname:    CPSICheckObjForOvertake */
boolean CPSICheckObjForOvertake(ObjNumber_t iObj) {
    _PARAM_UNUSED(iObj);
    return FALSE;
}

/*************************************************************************************************************************
  Functionname:    CPSIGetCustomOvertakeAssistanceState */
boolean CPSIGetCustomOvertakeAssistanceState(void) { return FALSE; }

/*************************************************************************************************************************
  Functionname:    CPSIProcess */
void CPSIProcess(void) {
    CPCourseData_t* pCourseData = SIGetCourseData();
    CPTrajectoryData_t* pTrajectoryData = SIGetTrajectoryData();

    ObjNumber_t iObj;
    CPTrajectoryInputParameter_t TrajInputPara;
    TrajInputPara.bSuppressFusion = FALSE;

    if (CPState == CP_OK) {
        /* get SA course data */
        CPGetCourseDataEgo(
            pCourseData,
            (boolean)CFG_SA_USE_SLIPANGLE /*, GET_EGO_OBJ_SYNC_DATA_PTR */);

        /* Call CP custom course processing */
        CPSICustomCourseProc(pCourseData);

        /* calc TRAJECTORY */
        CPCalculateTrajectory(pCourseData, &TrajInputPara, pTrajectoryData);

#pragma COMPILEMSG( \
    "Move this code to SI, since it is up to user of course to determine how to filter dist to traj!")

        for (iObj = 0; iObj < Envm_N_OBJECTS; iObj++) {
            if (!OBJ_IS_DELETED(iObj)) {
                CPUpdateObjDist2Traj(iObj, 3.0f * OBJ_KALMAN_MAX_ACCEL_Y(iObj) +
                                               SI_AVLC_MAXACCELTRAJDIST,
                                     pTrajectoryData,
                                     &OBJ_GET_CP(iObj).TrajDist);
            }
        }
    } else if (CPState == CP_INIT)

    {
        /* Initialize CP component */
        CPSIInit(pCourseData, pTrajectoryData);
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */