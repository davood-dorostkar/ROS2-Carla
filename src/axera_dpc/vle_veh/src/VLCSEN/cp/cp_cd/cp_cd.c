// calibration
#define CAL_START_CODE
#include "Mem_Map.h"

const volatile float AEB_FCW_Yaw_Threshold = 0.01f;

#define CAL_STOP_CODE
#include "Mem_Map.h"
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cp_cd.h"
#include "cp_cd_curvefilter.h" /* Introduce CPCDCurveFilterDat_t */
#include "cd_ext.h"
#include "TM_Base_Interpol.h"
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
SET_MEMSEC_VAR(CPCDTrajectoryEgo)
SET_MEMSEC_VAR(CPCDTrajectoryRoad)
SET_MEMSEC_VAR(bCPCDInitDone)
SET_MEMSEC_VAR(CPCDObjToTrajRelationsSupportData)
SET_MEMSEC_VAR(CPCDObjDistToTraj)
SET_MEMSEC_VAR(CPCDObjToTrajRelationsRoad)

/*!  @cond Doxygen_Suppress */
static CPCourseData_t CPCDTrajectoryEgo;
static CPCourseData_t CPCDTrajectoryRoad;

static boolean bCPCDInitDone = FALSE;
static CPTrajectoryData_t CPCDObjToTrajRelationsSupportData;

static CPObjDist2Traj_t CPCDObjDistToTraj[Envm_N_OBJECTS];

static CPCDObjToTrajRelation_t CPCDObjToTrajRelationsEgo[Envm_N_OBJECTS];
static CPCDObjToTrajRelation_t CPCDObjToTrajRelationsRoad[Envm_N_OBJECTS];
/*! @endcond */

SET_MEMSEC_VAR(CPCDEgoTrajFilterDat)
SET_MEMSEC_VAR(CPCDCurveFilterOutput)
/*!  @cond Doxygen_Suppress */
static CPCDCurveFilterDat_t CPCDEgoTrajFilterDat;
static CPCDCurveFilterOut_t CPCDCurveFilterOutput;
/*! @endcond */

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define CP_RESET_VEL_TO_TRAJECTORY_THRESHOLD_X_VALUES (2L)
// DistX threshold for setting VelToTrajectory to zero. Input: RelativeVelX,
// Output: DistX threshold
const BML_t_Vector2D CP_RESET_VEL_TO_TRAJECTORY_THRESHOLD
    [CP_RESET_VEL_TO_TRAJECTORY_THRESHOLD_X_VALUES] = {{-20.0f, 50.0f},
                                                       {-10.0f, 30.0f}};
/*****************************************************************************
  FUNCTION
*****************************************************************************/

static void CPCDInitTrajectories(void);
static void CPCDInitObjToTrajRelations(void);

static void CPCDCalculateTrajectoryEgo(void);
static void CPCDCalculateTrajectoryRoad(void);

static void CPCDUpdateObjToTrajRelationsEgo(void);
static void CPCDUpdateObjToTrajRelationsRoad(void);

/*************************************************************************************************************************
  Functionname:    CPCDInitTrajectories */

static void CPCDInitTrajectories(void) {
    uint8 uiObj;
    /* Initialize CPCDObjDistToTraj Struct */
    for (uiObj = 0u; uiObj < Envm_N_OBJECTS; uiObj++) {
        CPCDObjDistToTraj[uiObj].TrajDistFilt.X.f0 = 0;
        CPCDObjDistToTraj[uiObj].TrajDistFilt.X.f1 = 0;
        CPCDObjDistToTraj[uiObj].TrajDistFilt.P.f00 = 0;
        CPCDObjDistToTraj[uiObj].TrajDistFilt.P.f01 = 0;
        CPCDObjDistToTraj[uiObj].TrajDistFilt.P.f11 = 0;
        CPCDObjDistToTraj[uiObj].ObjDistOnTraj = 0;
    }
    /* Initialize CPCDTrajectoryEgo Struct */
    CPCDTrajectoryEgo.fCurve = 0;
    CPCDTrajectoryEgo.fCurveGradient = 0;
    CPCDTrajectoryEgo.fCurveVar = 0;
    CPCDTrajectoryEgo.SideSlipAngle = 0;
    CPCDTrajectoryEgo.SideSlipAngleVariance = 0;

    /* Initialize CPCDTrajectoryRoad Struct */
    CPCDTrajectoryRoad.fCurve = 0;
    CPCDTrajectoryRoad.fCurveGradient = 0;
    CPCDTrajectoryRoad.fCurveVar = 0;
    CPCDTrajectoryRoad.SideSlipAngle = 0;
    CPCDTrajectoryRoad.SideSlipAngleVariance = 0;

    /* Initialize CPCDEgoTrajFilterDat Struct */
    CPCDEgoTrajFilterDat.fCumSum = 0;
    CPCDEgoTrajFilterDat.fGainVel = 0;
    CPCDEgoTrajFilterDat.fGainMan = 0;
    CPCDEgoTrajFilterDat.bfirstCycle = TRUE;
    CPCDEgoTrajFilterDat.bUpdate = TRUE;

    /* Initialize CPCDCurveFilterOutput Struct */
    CPCDCurveFilterOutput.fCurve = 0;
    CPCDCurveFilterOutput.fCurveGradient = 0;
}

/*************************************************************************************************************************
  Functionname:    CPCDInitObjToTrajRelations */

static void CPCDInitObjToTrajRelations(void) {
    ObjNumber_t iObj;
    CPTrajectoryInit(&CPCDTrajectoryEgo, FALSE, FALSE, FALSE, FALSE,
                     &CPCDObjToTrajRelationsSupportData);

    for (iObj = 0; iObj < (ObjNumber_t)Envm_N_OBJECTS; iObj++) {
        CPCDObjToTrajRelationsEgo[iObj].fDistToTraj = 0;
        CPCDObjToTrajRelationsEgo[iObj].fDistToTrajVar = 0;
        CPCDObjToTrajRelationsEgo[iObj].fVelocityToTraj = 0;
        CPCDObjToTrajRelationsEgo[iObj].fVelocityToTrajVar = 0;

        CPCDObjToTrajRelationsRoad[iObj].fDistToTraj = 0;
        CPCDObjToTrajRelationsRoad[iObj].fDistToTrajVar = 0;
        CPCDObjToTrajRelationsRoad[iObj].fVelocityToTraj = 0;
        CPCDObjToTrajRelationsRoad[iObj].fVelocityToTrajVar = 0;
    }
}

/*************************************************************************************************************************
  Functionname:    CPCDProcess */

void CPCDProcess(void) {
    if (bCPCDInitDone == FALSE) {
        CDState = CD_STATE_INIT;
    }

    switch (CDState) {
        case CD_STATE_OK: {
            /* Calculate Trajectories */
            CPCDCalculateTrajectoryEgo();
            CPCDCalculateTrajectoryRoad();

            /* Calculate Object To Trajectory Relations */
            CPCDUpdateObjToTrajRelationsEgo();
            CPCDUpdateObjToTrajRelationsRoad();
        } break;
        case CD_STATE_INIT:
        default: {
            CPCDInitTrajectories();
            CPCDInitObjToTrajRelations();
            bCPCDInitDone = TRUE;
        } break;
    }
}

/*************************************************************************************************************************
  Functionname:    CPCDCalculateTrajectoryEgo */

static void CPCDCalculateTrajectoryEgo(void) {
    CPGetCourseDataEgo(&CPCDTrajectoryEgo, CPCD_USE_SLIPANGLE);

    CPCDCurveFilterRun(&CPCDTrajectoryEgo, &CPCDCurveFilterOutput,
                       &CPCDEgoTrajFilterDat);
}

/*************************************************************************************************************************
  Functionname:    CPCDCalculateTrajectoryRoad */

static void CPCDCalculateTrajectoryRoad(void) {
    /* Retrieve SI-Trajectory */
    const CPTrajectoryData_t *pSITrajInfo;
    pSITrajInfo = SIGetTrajectoryData();

    /* Fill modullocal Struct */
    CPCDTrajectoryRoad.fCurve = pSITrajInfo->Current.fTrajC0;
    CPCDTrajectoryRoad.fCurveGradient = pSITrajInfo->Current.fTrajC1;
    CPCDTrajectoryRoad.SideSlipAngle = pSITrajInfo->Current.fTrajAngle;
}

/*************************************************************************************************************************
  Functionname:    CPCDUpdateObjToTrajRelationsEgo */

static void CPCDUpdateObjToTrajRelationsEgo(void) {
    ObjNumber_t iObj;

    CPPrepareTrajectoryData(&CPCDObjToTrajRelationsSupportData);

    CPCDTrajectoryEgo.fCurveGradient =
        0; /* In last code the kalman-coefficients were set to zero... */
    CPCDObjToTrajRelationsSupportData.Current.fTrajC0 =
        CPCDTrajectoryEgo.fCurve;
    CPCDObjToTrajRelationsSupportData.Current.fTrajC1 =
        CPCDTrajectoryEgo.fCurveGradient;

    GDBKalmanSetMat(&CPCDObjToTrajRelationsSupportData.KafiEst.XVec, sTRAJ_C0,
                    (uint8)0, CPCDTrajectoryEgo.fCurve);
    GDBKalmanSetMat(&CPCDObjToTrajRelationsSupportData.KafiEst.XVec, sTRAJ_C1,
                    (uint8)0, 0);

    GDBKalmanSetMat(&CPCDObjToTrajRelationsSupportData.KafiEst.XsVec, sTRAJ_C0,
                    (uint8)0, CPCDTrajectoryEgo.fCurve);
    GDBKalmanSetMat(&CPCDObjToTrajRelationsSupportData.KafiEst.XsVec, sTRAJ_C1,
                    (uint8)0, 0);

    GDBKalmanSetMat(&CPCDObjToTrajRelationsSupportData.KafiEst.UDMat, sTRAJ_C0,
                    sTRAJ_C0, CPCDTrajectoryEgo.fCurveVar);
    GDBKalmanSetMat(&CPCDObjToTrajRelationsSupportData.KafiEst.UDMat, sTRAJ_C1,
                    sTRAJ_C1, 0);

    for (iObj = 0; iObj < (ObjNumber_t)Envm_N_OBJECTS; iObj++) {
        if (!OBJ_IS_DELETED(iObj)) {
            CPUpdateObjDist2Traj(iObj, fCPCDMaxAccelDist2Traj,
                                 &CPCDObjToTrajRelationsSupportData,
                                 &(CPCDObjDistToTraj[iObj]));

            CPCDObjToTrajRelationsEgo[iObj].fDistToTraj =
                CPCDObjDistToTraj[iObj].TrajDistFilt.X.f0;
            CPCDObjToTrajRelationsEgo[iObj].fDistToTrajVar =
                CPCDObjDistToTraj[iObj].TrajDistFilt.P.f00;
            CPCDObjToTrajRelationsEgo[iObj].fVelocityToTraj =
                CPCDObjDistToTraj[iObj].TrajDistFilt.X.f1;
            CPCDObjToTrajRelationsEgo[iObj].fVelocityToTrajVar =
                CPCDObjDistToTraj[iObj].TrajDistFilt.P.f11;
            // set VelocityToTrajectory to zero
            // when object DistX more than threshold(in a long range) because
            // the deviation of YawRate
            extern float g_fPreBrakeDecel;
            float32 fDistXThreshold = CML_f_CalculatePolygonValue(
                CP_RESET_VEL_TO_TRAJECTORY_THRESHOLD_X_VALUES,
                CP_RESET_VEL_TO_TRAJECTORY_THRESHOLD, OBJ_LONG_VREL(iObj));
            // if (OBJ_LONG_DISPLACEMENT(iObj) >= fDistXThreshold ||
            //     g_fPreBrakeDecel <
            //         -2.0f)  // set VelocityToTrajectory to zero when AEB
            // deceleration is triggered. Consider yawrate error
            // caused by acute deceleration
            // if (g_fPreBrakeDecel < -2.0f &&
            //     TUE_F_Abs(EGO_YAW_RATE_RAW) > AEB_FCW_Yaw_Threshold) {
            //     CPCDObjToTrajRelationsEgo[iObj].fVelocityToTraj = 0.f;
            // }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    CPCDUpdateObjToTrajRelationsRoad */

static void CPCDUpdateObjToTrajRelationsRoad(void) {
    ObjNumber_t iObj;

    for (iObj = 0; iObj < (ObjNumber_t)Envm_N_OBJECTS; iObj++) {
        if (!OBJ_IS_DELETED(iObj)) {
            CPCDObjToTrajRelationsRoad[iObj].fDistToTraj =
                OBJ_GET_CP(iObj).TrajDist.TrajDistFilt.X.f0;
            CPCDObjToTrajRelationsRoad[iObj].fDistToTrajVar =
                OBJ_GET_CP(iObj).TrajDist.TrajDistFilt.P.f00;
            CPCDObjToTrajRelationsRoad[iObj].fVelocityToTraj =
                OBJ_GET_CP(iObj).TrajDist.TrajDistFilt.X.f1;
            CPCDObjToTrajRelationsRoad[iObj].fVelocityToTrajVar =
                OBJ_GET_CP(iObj).TrajDist.TrajDistFilt.P.f11;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    CPCDCriticalTimeAfterMerge */

void CPCDCriticalTimeAfterMerge(ObjNumber_t iObjId) {
    CPCDObjToTrajRelationsEgo[iObjId].fVelocityToTraj = 0;
    CPCDObjToTrajRelationsRoad[iObjId].fVelocityToTraj = 0;
}

/*************************************************************************************************************************
  Functionname:    CPCDProcessTrajectoriesMeas */

void CPCDProcessTrajectoriesMeas(CPTrajMeasInfo_t *pTrajectoriesMeas) {
    CPCopyTraj2Meas(&CPCDObjToTrajRelationsSupportData,
                    &pTrajectoriesMeas->Trajectory);
    CPCopyCourse2Meas(&CPCDTrajectoryEgo, &pTrajectoriesMeas->FiltCourse);
    pTrajectoriesMeas->LaneWidth = CPCD_RUN_UP_TRACK_WIDTH;
}

/*************************************************************************************************************************
  Functionname:    CPCDGetCurvatureEgo */

float32 CPCDGetCurvatureEgo(const float32 fXPosition) {
    float32 fCurvatureAtXPosition;

    fCurvatureAtXPosition = CPGetCurvature(&CPCDTrajectoryEgo, fXPosition);

    return fCurvatureAtXPosition;
}

/*************************************************************************************************************************
  Functionname:    CPCDGetCurvatureRoad */

float32 CPCDGetCurvatureRoad(const float32 fXPosition) {
    float32 fCurvatureAtXPosition;

    fCurvatureAtXPosition = CPGetCurvature(&CPCDTrajectoryRoad, fXPosition);

    return fCurvatureAtXPosition;
}

/*************************************************************************************************************************
  Functionname:    CPCDGetObjToTrajRelationEgo */

const CPCDObjToTrajRelation_t *CPCDGetObjToTrajRelationEgo(ObjNumber_t iObjId) {
    return &(CPCDObjToTrajRelationsEgo[iObjId]);
}

/*************************************************************************************************************************
  Functionname:    CPCDGetObjToTrajRelationRoad */

const CPCDObjToTrajRelation_t *CPCDGetObjToTrajRelationRoad(
    ObjNumber_t iObjId) {
    return &(CPCDObjToTrajRelationsRoad[iObjId]);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */