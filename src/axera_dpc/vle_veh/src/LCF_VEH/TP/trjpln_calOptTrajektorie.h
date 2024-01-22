/*********************************************************************
MODULE:         Trajectory Planning

Type:           C-Source

CPU:            All

DESCRIPTION:

********************************************************************************

<HISTORY>
#<version>  <modify-date>   <owner>     <comment>
<NEW>
</NEW>
</HISTORY>

*********************************************************************/

#ifndef TRJPLN_CALOPTTRAJEKTORIE_H
#define TRJPLN_CALOPTTRAJEKTORIE_H

/* -----------------------------------------------------------------------------

        I N C L U D E S

 * -----------------------------------------------------------------------------
*/

#include "trjpln_calOptTrajektorie_TypeDef.h"
#include "trjpln_consts.h"

/* -----------------------------------------------------------------------------

        D E C L A R A T I O N   F U N C T I O N S

 * -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
        \brief Calculate optimal trajectory.

        \param Inputs Trajectory planning inputs

        \return Calculated trajectory
 * -----------------------------------------------------------------------------
*/
TRJPLN_calOptOutTypeV4_t TrajCalc_CalcOptTrajectory(
    const TRJPLN_calOptInTypeV3_t* Inputs);

/* -----------------------------------------------------------------------------
        \brief Calculate max. acceleration along trajectory.

        \param p2 Trajectory parameter
        \param p3 Trajectory parameter
        \param p4 Trajectory parameter
        \param p5 Trajectory parameter
        \param Kr
        \param Vx Vehicle long. velocity

        \return Max. acceleration (Ax

 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_CalcMaxTrajAccel(const float32 p2,
                                const float32 p3,
                                const float32 p4,
                                const float32 p5,
                                const float32 t,
                                const float32 Kr,
                                const float32 Vx);

/* -----------------------------------------------------------------------------
        \brief Initialize output structure.

        \param Outputs Output structure

        \return -
 * -----------------------------------------------------------------------------
*/
void TRJPLN_InitializeOutputs(TRJPLN_calOptOutTypeV4_t* Outputs);

/* -----------------------------------------------------------------------------
        \brief Calculation of matrix Ax and vector Bx

        \param

        \return -

 * -----------------------------------------------------------------------------
*/
void TRJPLN_CalcAxBx(float32 A[3][3],
                     float32 B[3],
                     const float32 Param[6],
                     const float32 t,
                     float32 const d,
                     float32 const d_dot,
                     float32 const d_ddot);

/* -----------------------------------------------------------------------------
        \brief Calculate parameters p3, p4, p5

        \param

        \return

 * -----------------------------------------------------------------------------
*/
boolean_T TRJPLN_CalcParameters(float32 Param[6],
                                float32 A[][3],
                                const float32 B[3]);

/* -----------------------------------------------------------------------------
        \brief Calculate cost function

        \param Param
        \param t
        \param Kd
        \param Kt
        \param d
        \param d_k

        \return Calculated cost

 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_CalcCostFunction(const float32 Param[6],
                                const float32 t,
                                const float32 Kd,
                                const float32 Kt,
                                const float32 d,
                                const float32 d_k);

/* -----------------------------------------------------------------------------
        \brief Check tire potential vector.

        \param

        \return

 * -----------------------------------------------------------------------------
*/
boolean_T TRJPLN_CheckPotVector(const float32 Param_akt[6],
                                float32* AMax,
                                const float32 Kr,
                                const float32 te,
                                const float32 VehVelX,
                                const float32 FTireAclMin,
                                const float32 FTireAclMax,
                                const boolean_T PlanArcLength,
                                boolean_T* FTireAclMinOk,
                                boolean_T* FTireAclMaxOk);

/* Jerk Calculation*/
float32 TRJPLN_CalcTrajJerk(const float32 p3,
                            const float32 p4,
                            const float32 p5,
                            const float32 t,
                            const float32 CrvChng,
                            const float32 Vx);

/* Jerk Check*/
boolean_T TRJPLN_CheckJerk(const float32 curParam[6],
                           float32* JerkMax,
                           const float32 CrvChng,
                           const float32 te,
                           const float32 VehVelX,
                           const float32 AllwdJerkMax,
                           const boolean_T PlanArcLength);

boolean_T TRJPLN_CheckNewBestPoint(const uint8 uiTrajStatus_nu,
                                   const boolean_T StatusTrajPlan,
                                   const float32 curCost,
                                   const float32 fOptimalCost_nu);
/* -----------------------------------------------------------------------------
        \brief Calculate optimal trajectory (internal)

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_CalcOptTrajectoryInt(const float32 dz_new,
                                 const float32 dz_dot,
                                 const float32 dz_ddot,
                                 const float32 te,
                                 boolean_T* NewBestPoint,
                                 const float32 d_AW,
                                 const float32 d_AWdot,
                                 const float32 d_AWddot,
                                 const float32 Kd,
                                 const float32 Kt,
                                 const float32 dz_original,
                                 const boolean_T CheckFTire,
                                 const float32 Kr,
                                 const float32 VehVelX,
                                 const float32 FTireAclMin,
                                 const float32 FTireAclMax,
                                 const boolean_T PlanArcLength,
                                 TRJPLN_calOptOutTypeV4_t* Outputs,
                                 const float32 PH_VisLength,
                                 const float32 PH_Traj,
                                 const boolean_T CheckCollCridr,
                                 const boolean_T CheckCollObj,
                                 const TRJPLN_calOptInTypeV3_t* Inputs,
                                 const float32 CrvChng,
                                 const float32 AllwdJerkMax,
                                 const boolean_T CheckJerk,
                                 const boolean_T CheckLaneCross);

/* -----------------------------------------------------------------------------
        \brief Check for collision of trajectory with available corridor

        \param

        \return

 * -----------------------------------------------------------------------------
*/
boolean_T TRJPLN_CheckCollisionCorridor(const boolean_T PlanArcLength,
                                        const float32 PH_Traj,
                                        const float32 PH_VisLength,
                                        const float32 CurTgtPoint,
                                        const uint8 EndIndexLeftCridr,
                                        float32 CircleRadius,
                                        const float32 Param[6],
                                        const float32 LeftDistY[100],
                                        boolean_T* CollCheckRightOk,
                                        boolean_T* CollCheckLeftOk);

/* -----------------------------------------------------------------------------
        \brief Check for collision of trajectory with obstacle

        \param

        \return

 * -----------------------------------------------------------------------------
*/
boolean_T TRJPLN_CheckCollisionObject(const float32 t_DeadTime,
                                      const float32 v_Traj,
                                      const float32 a_Traj,
                                      const float32 v_Obj,
                                      const float32 a_Obj,
                                      const float32 ObstDistX,
                                      const float32 ObstDistY,
                                      const float32 HindernisWidth,
                                      const float32 CircleRadius,
                                      const float32 VisLength,
                                      const float32 TgtPoint,
                                      const float32 P[6]);

boolean_T TRJPLN_CheckLaneCross(const float32 PH_VisLength,
                                const float32 CurTgtPoint,
                                const float32 VehicleWidth,
                                const float32 Param[6],
                                const float32 AllowedTimetoCross,
                                const float32 LaneWidth);

/* -----------------------------------------------------------------------------

        H E L P E R   F U N C T I O N S

 * -----------------------------------------------------------------------------
*/

float32 TRJPLN_MinFloat(const float32 A, const float32 B);

#endif /* TRJPLN_CALOPTTRAJEKTORIE_H */
