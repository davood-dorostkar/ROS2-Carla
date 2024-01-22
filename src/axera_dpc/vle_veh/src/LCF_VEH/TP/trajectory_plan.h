#ifndef TRAJECTORY_PLAN_H
#define TRAJECTORY_PLAN_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "trajectory_plan_ext.h"
#include "trjpln_consts.h"
#include "trjpln_latency_compensation.h"
#include "trjpln_calculation_enable.h"
#include "trjpln_frenet_transform.h"
#include "trjpln_trajectory_calculation.h"
#include "trjpln_frenet_back.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LatencyCompensationInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    TRJPLN_LatencyCompensationInReq_t* pLatCompInput);
void CalculationEnableInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    const TRJPLN_LatencyCompensationOutPro_t* pLatCompOutput,
    const float32 fCtrlErrHeadAglPrev_rad,
    const float32 fCtrlErrDistY_met,
    const boolean bTrajecotryEnd,
    TRJPLN_CalculationEnableInReq_t* pCalEnableInput);
void FrenetTransformInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    const TRJPLN_LatencyCompensationOutPro_t* pLatCompOutput,
    const TRJPLN_CalculationEnableOutPro_t* pCalcEnableOutput,
    const TRJPLN_FrenetBackOutPro_t* pFrenBackOutput,
    TRJPLN_FrenetTransformInReq_t* pFrenetTrans);
void TrajectoryCalcInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    const TRJPLN_CalculationEnableOutPro_t* pCalcEnableOutput,
    const TRJPLN_LatencyCompensationOutPro_t* pLatCompOutput,
    const TRJPLN_FrenetTransformOutPro_t* pFrenetTransfOutput,
    TRJPLN_TrajectoryCalcInReq_t* pTrajectoryCalcInput);
void FrenetBackInputWrapper(
    const TRJPLN_TrajectoryPlanInReq_t* pTrajPlanInput,
    const TRJPLN_CalculationEnableOutPro_t* pCalcEnableOutput,
    const TRJPLN_LatencyCompensationOutPro_t* pLatCompOutput,
    const TRJPLN_TrajectoryCalcOutPro_t* pTrajCalcOutput,
    const TRJPLN_FrenetTransformOutPro_t* pFrenetTransfOutput,
    TRJPLN_FrenetBackInReq_t* pFrenetBackInput);
void TrajectoryOutputWrapper(
    const TRJPLN_FrenetBackOutPro_t* pFrenetBackOutput,
    const TRJPLN_CalculationEnableOutPro_t* pCalcEnableOutput,
    TRJPLN_TrajectoryPlanOutPro_t* pTrajectoryPlanOutput);

#ifdef __cplusplus
}
#endif
#endif
