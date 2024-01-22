/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/*********************************************************************
MODULE:         Frenet Transformation

Type:           C-Header

CPU:            All

DESCRIPTION:

********************************************************************************

<HISTORY>
#<version>  <modify-date>   <owner>     <comment>

<NEW>
</NEW>
</HISTORY>

*********************************************************************/

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_TP_TRJPLN_CALFRENETTRANSFORMATION_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_TP_TRJPLN_CALFRENETTRANSFORMATION_H_

#include "trjpln_calFrenetTransformation_TypeDef.h"
#include "trjpln_consts.h"

/* -----------------------------------------------------------------------------
        \brief Frenet Transformation

        \param

        \return
 * -----------------------------------------------------------------------------
*/
TRJPLN_calFTOutType_t TRJPLN_calFrenetTransformation(
    const TRJPLN_calFTInTypeV3_t* Inputs);

/* -----------------------------------------------------------------------------
        \brief Calculation of Heading

        \param

        \return

 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_FT_Theta(const float32 theta0,
                        const float32 K0,
                        const float32 K_prime,
                        const float32 x,
                        const float32 x0);

/* -----------------------------------------------------------------------------
        \brief Calculation of lateral distance

        \param

        \return

 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_FT_Y(const float32 Y0,
                    const float32 theta0,
                    const float32 K0,
                    const float32 K_prime,
                    const float32 x,
                    const float32 x0);

/* -----------------------------------------------------------------------------
        \brief Calculation of curvature K

        \param

        \return

 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_FT_K(const float32 K0,
                    const float32 K_prime,
                    const float32 x,
                    const float32 x0);

/* -----------------------------------------------------------------------------
        \brief Simplified calculation of heading angle for right corridor

        \param

        \return

        Initial heading angle of the right corridor is always zero. Thus a
 simplified
        calculation is used to reduce CPU load.

 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_FT_Thetar(const float32 theta0,
                         const float32 K0,
                         const float32 K_prime,
                         const float32 x,
                         const float32 x0,
                         const boolean_T UseCridrRiX0);

/* -----------------------------------------------------------------------------
        \brief Simplified calculation of lateral distance for right corridor

        \param

        \return

        Initial heading angle of the right corridor is always zero. Thus a
 simplified
        calculation is used to reduce CPU load.

 * -----------------------------------------------------------------------------
*/
float32 TRJPLN_FT_Yr(const float32 Y0,
                     const float32 theta0,
                     const float32 K0,
                     const float32 K_prime,
                     const float32 x,
                     const float32 x0,
                     const boolean_T UseCridrRiX0);

/* -----------------------------------------------------------------------------
        \brief Frenet

        \param

        \return
 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_InitializeOutputs(TRJPLN_calFTOutType_t* Outputs);

/* -----------------------------------------------------------------------------
        \brief Frenet

        \param

        \return
 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_InitializeIntVectors(float32 A[61],
                                    float32 B[61],
                                    float32 C[61],
                                    float32 D[61]);

/* -----------------------------------------------------------------------------
        \brief Frenet Transformation of current values

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_TransformCurValues(const boolean_T PlanArcLength,
                                  const float32 DevDistY,
                                  const float32 DevHeading,
                                  const float32 Vx,
                                  const float32 Ax,
                                  const float32 Kr,
                                  const float32 Kr_prime,
                                  const float32 Kist,
                                  float32* dist,
                                  float32* dist1stDeriv,
                                  float32* dist2ndDeriv,
                                  float32* v_traj,
                                  float32* a_traj,
                                  float32 CurHdngAndDelta,
                                  float32* DeltaHdng1stDeriv,
                                  float32* DeltaHdng2ndDeriv,
                                  float32 Kstart,
                                  float32 Ksoll);

/* -----------------------------------------------------------------------------
        \brief Calculation of target lateral distances and derivatives

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcTargetCurvePoints(const boolean_T PlanArcLength,
                                     const float32 TgtSg1X,
                                     const float32 TgtSg1Length,
                                     const float32 CridrRiSg1Heading,
                                     const float32 Kr,
                                     const float32 Kr_prime,
                                     const float32 PlanHorizon,
                                     const float32 CridrRiSg1Length,
                                     const float32 TgtTrajSg1Y,
                                     const float32 TgtTrajSg1Hdng,
                                     const float32 TgtTrajSg1Crv,
                                     const float32 TgtTrajSg1ChngOfCrv,
                                     const float32 v_traj,
                                     const float32 a_traj,
                                     TRJPLN_calFTOutType_t* Outputs,
                                     const boolean_T UseTgtTrajX0,
                                     const float32 CridrRiSg1X,
                                     const boolean_T UseCridrRiX0,
                                     const float32 CridrRiSg1Y);

/* -----------------------------------------------------------------------------
        \brief Calculation of start point of trajectory (arc length based
 planning)

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcStartPoint_ArcLength(float32* x_Start,
                                        float32* t_Start,
                                        float32* s_Start,
                                        float32* PlanAreaTgt,
                                        const float32 TgtSg1X,
                                        const float32 TgtSg1Length,
                                        const float32 CridrRiSg1Heading,
                                        const float32 Kr,
                                        const float32 Kr_prime,
                                        const float32 CridrRiSg1X,
                                        const boolean_T UseCridrRiX0);

/* -----------------------------------------------------------------------------
        \brief Calculation of start point of trajectory (time based planning)

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcStartPoint_Time(float32* x_Start,
                                   float32* t_Start,
                                   float32* s_Start,
                                   float32* PlanAreaTgt,
                                   const float32 v_traj,
                                   const float32 a_traj,
                                   const float32 TgtSg1X,
                                   const float32 TgtSg1Length,
                                   const float32 CridrRiSg1Heading,
                                   const float32 Kr,
                                   const float32 Kr_prime,
                                   const float32 CridrRiSg1X,
                                   const boolean_T UseCridrRiX0);

/* -----------------------------------------------------------------------------
        \brief Calculation of target lateral distances and derivatives (arc
 length based)

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcTargetCurve_ArcLength(uint8* NumTgtPoints,
                                         float32 TgtPoints[101],
                                         float32 TgtDistYPoint[101],
                                         float32 TgtDistY1stDerivPoint[101],
                                         float32 TgtDistY2ndDerivPoint[101],
                                         float32* PlanHorizonFT,
                                         float32* PlanHorizonVisRange,
                                         const float32 x_Start,
                                         const float32 s_Start,
                                         const float32 CridrRiSg1Heading,
                                         const float32 Kr,
                                         const float32 Kr_prime,
                                         const float32 CridrRiSg1Length,
                                         const float32 PlanAreaTgt,
                                         const float32 TgtTrajSg1Y,
                                         const float32 TgtTrajSg1Hdng,
                                         const float32 TgtTrajSg1Crv,
                                         const float32 TgtTrajSg1ChngOfCrv,
                                         float32 v_traj,
                                         const float32 CridrRiSg1X,
                                         const boolean_T UseCridrRiX0,
                                         const float32 CridrRiSg1Y);

/* -----------------------------------------------------------------------------
        \brief Calculation of target lateral distances and derivatives (time
 based)

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcTargetCurve_Time(uint8* NumTgtPoints,
                                    float32 TgtPoints[101],
                                    float32 TgtDistYPoint[101],
                                    float32 TgtDistY1stDerivPoint[101],
                                    float32 TgtDistY2ndDerivPoint[101],
                                    float32* PlanHorizonFT,
                                    float32* PlanHorizonVisRange,
                                    const float32 x_Start,
                                    const float32 s_Start,
                                    const float32 t_Start,
                                    const float32 CridrRiSg1Heading,
                                    const float32 Kr,
                                    const float32 Kr_prime,
                                    const float32 CridrRiSg1Length,
                                    const float32 PlanAreaTgt,
                                    const float32 TgtTrajSg1Y,
                                    const float32 TgtTrajSg1Hdng,
                                    const float32 TgtTrajSg1Crv,
                                    const float32 TgtTrajSg1ChngOfCrv,
                                    const float32 v_traj,
                                    const float32 a_traj,
                                    const float32 PlanHorizon,
                                    const float32 CridrRiSg1X,
                                    const boolean_T UseCridrRiX0,
                                    const float32 CridrRiSg1Y);

/* -----------------------------------------------------------------------------
        \brief Calculation of target lateral distances and derivatives

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcLatDistDerivs(float32* dTgt_prime,
                                 float32* dTgt_primeprime,
                                 float32* TgtDistYPoint,
                                 const float32 Xi,
                                 const float32 CridrRiSg1Heading,
                                 const float32 Kr,
                                 const float32 Kr_prime,
                                 const float32 TgtTrajSg1Y,
                                 const float32 TgtTrajSg1Hdng,
                                 const float32 TgtTrajSg1Crv,
                                 const float32 TgtTrajSg1ChngOfCrv,
                                 const float32 X0,
                                 const float32 CridrRiSg1X,
                                 const boolean_T UseCridrRiX0,
                                 const float32 CridrRiSg1Y);

/* -----------------------------------------------------------------------------
        \brief Transformation of trajectory values from last cycle to Frenet
 coords

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcTrajLastCycle(float32* FT_DistYTrajPrev,
                                 float32* FT_DistYTrajPrev_dot,
                                 float32* FT_DistYTrajPrev_ddot,
                                 const float32 DistYTrajPrev,
                                 const float32 HdngTrajPrev,
                                 const boolean_T PlanArcLength,
                                 const float32 Kr,
                                 const float32 Kr_prime,
                                 const float32 Ktraj,
                                 const float32 vx,
                                 const float32 ax,
                                 float32* v_traj_Tgt,
                                 float32* a_traj_Tgt);

/* -----------------------------------------------------------------------------
        \brief Calculate preview of current values in Frenet coords

        \param

        \return

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcPreviewCurValues(const float32 PreviewTimeKF,
                                    const float32 CridrRiSg1Heading,
                                    const float32 Kr,
                                    const float32 Kr_prime,
                                    const float32 IstY,
                                    const float32 IstHeading,
                                    const float32 CurCrv,
                                    const float32 v_traj,
                                    const float32 a_traj,
                                    TRJPLN_calFTOutType_t* Outputs,
                                    const float32 CridrRiSg1X,
                                    const boolean_T UseCridrRiX0,
                                    const float32 CridrRiSg1Y);

/* -----------------------------------------------------------------------------
        \brief Frenet Transformation of left corridor boundary

        \param

        \return

        Calculate the lateral distances from the left corridor boundary at
 different points

 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcLeftCorridor(TRJPLN_calFTOutType_t* Outputs,
                                const float32 CridrRiSg1Crv,
                                const float32 CridrRiSg1ChgOfCrv,
                                const float32 CridrLeSg1Length,
                                const float32 CridrRiSg1Length,
                                const float32 CridrLeSg1Y,
                                const float32 CridrLeSg1Hdng,
                                const float32 CridrLeSg1Crv,
                                const float32 CridrLeSg1ChgOfCrv,
                                const float32 v_Traj,
                                const float32 a_Traj,
                                const boolean_T PlanArcLength,
                                const float32 DeadTime,
                                const float32 CridrRiSg1X,
                                const boolean_T UseCridrRiX0,
                                const float32 CridrRiSg1Heading,
                                const float32 CridrRiSg1Y,
                                const float32 CridrLeSg1X);

/* -----------------------------------------------------------------------------
        \brief Frenet

        \param

        \return
 * -----------------------------------------------------------------------------
*/
void TRJPLN_FT_CalcRightCorridor(const boolean_T PlanArcLength,
                                 const float32 v_Traj,
                                 const float32 a_Traj,
                                 const float32 CridrRiSg1Crv,
                                 const float32 CridrRiSg1ChgOfCrv,
                                 const float32 CridrRiSg1Length,
                                 TRJPLN_calFTOutType_t* Outputs,
                                 const float32 CridrRiSg1X,
                                 const boolean_T UseCridrRiX0,
                                 const float32 CridrRiSg1Heading);

/* -----------------------------------------------------------------------------

        H E L P E R   F U N C T I O N S

 * -----------------------------------------------------------------------------
*/

float32 TRJPLN_FT_AvoidZero(const float32 input);

float32 TRJPLN_FT_MinFloat(const float32 A, const float32 B);

float32 TrajCalc_FT_MaxFloat(const float32 A, const float32 B);

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_LCF_VEH_TP_TRJPLN_CALFRENETTRANSFORMATION_H_
