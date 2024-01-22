/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
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

/* -----------------------------------------------------------------------------
        M I S R A
 * -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------

        I N C L U D E S

 * -----------------------------------------------------------------------------
*/

#include "trjpln_calOptTrajektorie.h"

/* -----------------------------------------------------------------------------

        D E C L A R A T I O N   F U N C T I O N S

 * -----------------------------------------------------------------------------
*/

#define START_SEC_CODE

/* -----------------------------------------------------------------------------

        F U N C T I O N   D E F I N I T I O N S

 * -----------------------------------------------------------------------------
*/

/*****************************************************************************
  Functionname: TRJPLN_InitializeOutputs */ /*!

@brief: optimal trajectory calculation module output initialize function

@description: optimal trajectory calculation module output initialize function

@param[in/out]
TRJPLN_calOptOutTypeV4_t* Outputs: the output pointer of
optimal trajectory calculation module
@param[out]


@return
*****************************************************************************/
void TRJPLN_InitializeOutputs(TRJPLN_calOptOutTypeV4_t* Outputs) {
    Outputs->uiTrajStatus_nu = false;
    {
        uint8 i;
        for (i = 0; i < 6; i++) {
            Outputs->afTrajParam_nu[i] = 0.0F;
        }
    }
    Outputs->fMaxAclTraj_mps2 = 20.0F;
    Outputs->fTrajCalcTimeTrajEnd_sec = 100.0F;
    Outputs->fArcLengthTrajEnd_met = 100.0F;
    Outputs->fOptimalTolerance_nu = 0.f;
    Outputs->fOptimalCost_nu = 99e+10F;
    Outputs->uiFtireMinOK_nu = true;
    Outputs->uiFtireMaxOK_nu = true;
    Outputs->uiCollDetRightOK_nu = true;
    Outputs->uiCollDetLeftOK_nu = true;
    Outputs->uiCollDetObjOK_nu = true;
    Outputs->uiMatrixInverseOK_nu = false;
    Outputs->uiTrajLengthOK_nu = true;
    Outputs->fMaxJerkTraj_mps3 = 10.0F;
    Outputs->uiMaxJerkOK_nu = true;
    Outputs->uiLaneCrossOK_nu = true;
}

/*****************************************************************************
  Functionname: TRJPLN_CalcAxBx                                           */ /*!

          @brief: Calculation of matrix A and B

          @description:  we need to calculate matrix A and B value, in order to
        calculate the parameters p3, p4, p5.
                                P = A * B

          @param[in]

          @param[out]


          @return
        *****************************************************************************/
void TRJPLN_CalcAxBx(float32 A[3][3],
                     float32 B[3],
                     const float32 Param[6],
                     const float32 t,
                     const float32 d,
                     const float32 d_dot,
                     const float32 d_ddot) {
    float32 t_2, t_3, t_4, t_5;
    t_2 = t * t;
    t_3 = t_2 * t;
    t_4 = t_3 * t;
    t_5 = t_4 * t;
    A[0][0] = t_3;
    A[0][1] = t_4;
    A[0][2] = t_5;
    A[1][0] = 3 * t_2;
    A[1][1] = 4 * t_3;
    A[1][2] = 5 * t_4;
    A[2][0] = 6 * t;
    A[2][1] = 12 * t_2;
    A[2][2] = 20 * t_3;

    B[0] = d - Param[0] - (t * Param[1]) - (t_2 * Param[2]);
    B[1] = d_dot - Param[1] - ((2 * t) * Param[2]);
    B[2] = d_ddot - (2 * Param[2]);
}

/*****************************************************************************
  Functionname: TRJPLN_CalcParameters */ /*!

@brief: Calculation of parameters p3, p4, p5

@description:  calculate the parameters P3, P4, P5 of optimal trajectory based
on equation:
        P = Inverse(A) * B

@param[in]

@param[out]


@return
*****************************************************************************/
boolean_T TRJPLN_CalcParameters(float32 Param[6],
                                float32 A[][3],
                                const float32 B[3]) {
    boolean_T bIsMatrixInvertible;
    float32 determinant =
        ((A[0][0] * ((A[1][1] * A[2][2]) - (A[2][1] * A[1][2]))) -
         (A[0][1] * ((A[1][0] * A[2][2]) - (A[1][2] * A[2][0])))) +
        (A[0][2] * ((A[1][0] * A[2][1]) - (A[1][1] * A[2][0])));
    /* Math Functions */
    if ((float32)FD_FABS(determinant) > 1.0e-10F) {
        {
            float32 A_inv[3][3];
            float32 invdet = 1.0f / determinant;
            A_inv[0][0] = ((A[1][1] * A[2][2]) - (A[2][1] * A[1][2])) * invdet;
            A_inv[0][1] = -((A[0][1] * A[2][2]) - (A[0][2] * A[2][1])) * invdet;
            A_inv[0][2] = ((A[0][1] * A[1][2]) - (A[0][2] * A[1][1])) * invdet;
            A_inv[1][0] = -((A[1][0] * A[2][2]) - (A[1][2] * A[2][0])) * invdet;
            A_inv[1][1] = ((A[0][0] * A[2][2]) - (A[0][2] * A[2][0])) * invdet;
            A_inv[1][2] = -((A[0][0] * A[1][2]) - (A[1][0] * A[0][2])) * invdet;
            A_inv[2][0] = ((A[1][0] * A[2][1]) - (A[2][0] * A[1][1])) * invdet;
            A_inv[2][1] = -((A[0][0] * A[2][1]) - (A[2][0] * A[0][1])) * invdet;
            A_inv[2][2] = ((A[0][0] * A[1][1]) - (A[1][0] * A[0][1])) * invdet;
            /* At any time the parameters p3, p4, p5 shall be calculated by
             * solving the following equations */
            Param[3] = (A_inv[0][0] * B[0]) + (A_inv[0][1] * B[1]) +
                       (A_inv[0][2] * B[2]);
            Param[4] = (A_inv[1][0] * B[0]) + (A_inv[1][1] * B[1]) +
                       (A_inv[1][2] * B[2]);
            Param[5] = (A_inv[2][0] * B[0]) + (A_inv[2][1] * B[1]) +
                       (A_inv[2][2] * B[2]);
            bIsMatrixInvertible = 1;
        }
    } else {
        bIsMatrixInvertible = 0;
        /* At any time the parameters p3, p4, p5 shall be calculated by solving
         * the following equations */
        Param[3] = 0.0f;
        Param[4] = 0.0f;
        Param[5] = 0.0f;
    }
    A[0][0] = 0.0f;

    return bIsMatrixInvertible;
}

/*****************************************************************************
  Functionname: TRJPLN_CalcCostFunction */ /*!

@brief: calculate the cost of planned fifth order polynomial trajectory

@description:
Here cost is related to three factors.
1, the cumulative value of lateral acceleration rate of change
2, the trajectory planning end point and the target corridor
lateral distance
difference (because the lateral distance has a certain
redundancy, does not need to
be exactly the same as the target corridor)
3, the trajectory planning path total required travel time or
distance (respectively
based on arc length based or time based planning)

@param[in]
const float32 Param[6]: the parameters of planned trajectory,
fifth order polynomial equation
const float32 t: the end time/distance of planned trajectory
const float32 Kd: the weighted factor of lateral distance
difference
const float32 Kt: the weighted factor of required travel time
or distance (respectively
based on arc length based or time based planning)
const float32 d: the lateral distance of planned trajectory at
the end of planned trajectory
const float32 d_k: the lateral distance of target corridor at
the end of planned trajectory
@param[out]

@return
calculated cost value
*****************************************************************************/
float32 TRJPLN_CalcCostFunction(const float32 Param[6],
                                const float32 t,
                                const float32 Kd,
                                const float32 Kt,
                                const float32 d,
                                const float32 d_k) {
    float32 fCost, t_2, t_3, t_4, t_5, P3_2, P4_2, P5_2;
    t_2 = t * t;
    t_3 = t_2 * t;
    t_4 = t_3 * t;
    t_5 = t_4 * t;
    P3_2 = Param[3] * Param[3];
    P4_2 = Param[4] * Param[4];
    P5_2 = Param[5] * Param[5];
    fCost = ((18 * P3_2) * t) + (((72 * Param[3]) * Param[4]) * t_2) +
            ((((120 * Param[3]) * Param[5]) + (96 * P4_2)) * t_3) +
            (((360 * Param[4]) * Param[5]) * t_4) + ((360 * P5_2) * t_5) +
            (Kd * (float32)FD_SQR(d - d_k)) + (Kt * t);

    return fCost;
}

float32 TRJPLN_CalcMaxTrajAccel(const float32 p2,
                                const float32 p3,
                                const float32 p4,
                                const float32 p5,
                                const float32 t,
                                const float32 Kr,
                                const float32 Vx) {
    float32 fAmax, t_2, t_3;
    t_2 = t * t;
    t_3 = t_2 * t;
    fAmax =
        (float32)FD_FABS(((((2 * p2) + ((6 * p3) * t)) + ((12 * p4) * t_2)) +
                          ((20 * p5) * t_3)) +
                         (Kr * (float32)FD_SQR(Vx)));

    return fAmax;
}

/*****************************************************************************
  Functionname: TRJPLN_CheckPotVector */ /*!

@brief:
        check the max acceleration of planned to trajectory

@description:
        return false if the max acceleration higher than threshold.
return true if the acceleration
        lower than threshold

@param[in]

@param[out]


@return
*****************************************************************************/
boolean_T TRJPLN_CheckPotVector(const float32 curParam[6],
                                float32* AMax,
                                const float32 Kr,
                                const float32 te,
                                const float32 VehVelX,
                                const float32 FTireAclMin,
                                const float32 FTireAclMax,
                                const boolean_T PlanArcLength,
                                boolean_T* FTireAclMinOk,
                                boolean_T* FTireAclMaxOk) {
    boolean_T StatusPotVecCheck;

    float32 t_max1, t_max2, a_max[4], fTmpAMax, P4_2;
    t_max2 = 0;
    t_max1 = 0;
    P4_2 = curParam[4] * curParam[4];

    /*  Calculation of extreme value tm */
    if ((float32)FD_FABS(curParam[5]) > 1.0e-10F) {
        if (((2 * P4_2) - (5 * curParam[3] * curParam[5])) >= 0) {
            t_max1 = ((-2.0F * curParam[4]) +
                      ((float32)FD_SQRT(2) *
                       (float32)FD_SQRT((2 * P4_2) -
                                        (5 * curParam[3] * curParam[5])))) /
                     (10 * curParam[5]);
            t_max2 = ((-2.0F * curParam[4]) -
                      ((float32)FD_SQRT(2) *
                       (float32)FD_SQRT((2 * P4_2) -
                                        (5 * curParam[3] * curParam[5])))) /
                     (10 * curParam[5]);

            t_max1 = (te < t_max1)
                         ? te
                         : t_max1; /* t_max1 = max(min(te[j],t_max1),0); */
            t_max1 = (t_max1 > 0) ? t_max1 : 0;

            t_max2 = (te < t_max2)
                         ? te
                         : t_max2; /* t_max2 = max(min(te[j],t_max2),0); */
            t_max2 = (t_max2 > 0) ? t_max2 : 0;
        }
    } else {
        if ((float32)FD_FABS(curParam[4]) > 1.0e-10F) {
            t_max2 = -curParam[3] / (4 * curParam[4]);
            t_max2 = (te < t_max2) ? te : t_max2;
            t_max2 = (t_max2 > 0) ? t_max2 : 0;
        }
    }

    /*Calculation of max. acceleration of all tm */
    a_max[0] = TRJPLN_CalcMaxTrajAccel(curParam[2], curParam[3], curParam[4],
                                       curParam[5], 0.0F, Kr, VehVelX);
    a_max[1] = TRJPLN_CalcMaxTrajAccel(curParam[2], curParam[3], curParam[4],
                                       curParam[5], t_max1, Kr, VehVelX);
    a_max[2] = TRJPLN_CalcMaxTrajAccel(curParam[2], curParam[3], curParam[4],
                                       curParam[5], t_max2, Kr, VehVelX);
    a_max[3] = TRJPLN_CalcMaxTrajAccel(curParam[2], curParam[3], curParam[4],
                                       curParam[5], te, Kr, VehVelX);
    {
        uint8 iter;
        fTmpAMax = a_max[0];
        for (iter = 1; iter < 4; iter++) {
            fTmpAMax = (fTmpAMax > a_max[iter]) ? fTmpAMax : a_max[iter];
        }
    }
    *AMax = fTmpAMax;

    /*Calculation of flag to check pot. vectors */
    if (PlanArcLength == true) {
        StatusPotVecCheck = true;
        *FTireAclMinOk = true;
        *FTireAclMaxOk = true;
    } else {
        if (fTmpAMax >= FTireAclMin) {
            *FTireAclMinOk = true;
        } else {
            *FTireAclMinOk = false;
        }
        if (fTmpAMax <= FTireAclMax) {
            *FTireAclMaxOk = true;
        } else {
            *FTireAclMaxOk = false;
        }
        StatusPotVecCheck = (*FTireAclMaxOk) & (*FTireAclMinOk);
    }

    return StatusPotVecCheck;
}

float32 TRJPLN_CalcTrajJerk(const float32 p3,
                            const float32 p4,
                            const float32 p5,
                            const float32 t,
                            const float32 CrvChng,
                            const float32 Vx) {
    float32 fJerk, t_2;
    t_2 = t * t;
    fJerk = (float32)FD_FABS((6 * p3 + (24 * p4) * t + (60 * p5) * t_2) +
                             CrvChng * (float32)FD_THIRDPOWER(Vx));

    return fJerk;
}

/*****************************************************************************
  Functionname: TRJPLN_CheckJerk                                           */ /*!

      @brief:
                            check the max acceleration jerk of planned to
    trajectory

      @description:
                            return false if the max acceleration jerk higher
    than
    threshold. return true if the acceleration
                            jerk lower than threshold

      @param[in]

      @param[out]


      @return
    *****************************************************************************/
boolean_T TRJPLN_CheckJerk(const float32 curParam[6],
                           float32* JerkMax,
                           const float32 CrvChng,
                           const float32 te,
                           const float32 VehVelX,
                           const float32 AllwdJerkMax,
                           const boolean_T PlanArcLength) {
    boolean_T StatusJerkCheck;
    float32 t_m[3];
    float32 t_Jerkmax, TempJerkMax, TempJerk[3], t, SumJerk, t_offset;

    t_m[0] = 0.0F;
    /* Calculation of extreme value tm */
    if ((float32)FD_FABS(curParam[5]) > 1.0e-10F) {
        t_m[1] = -24 * curParam[4] / (120 * curParam[5]);
    } else {
        t_m[1] = 0.0F;
    }
    t_m[2] = te;
    /* Calculation of max. Jerk */
    TempJerk[0] = TRJPLN_CalcTrajJerk(curParam[3], curParam[4], curParam[5],
                                      t_m[0], CrvChng, VehVelX);
    TempJerk[1] = TRJPLN_CalcTrajJerk(curParam[3], curParam[4], curParam[5],
                                      t_m[1], CrvChng, VehVelX);
    TempJerk[2] = TRJPLN_CalcTrajJerk(curParam[3], curParam[4], curParam[5],
                                      t_m[2], CrvChng, VehVelX);
    {
        uint8 iter;
        TempJerkMax = TempJerk[0];
        t_Jerkmax = t_m[0];
        for (iter = 1; iter < 3; iter++) {
            if (TempJerk[iter] > TempJerkMax) {
                TempJerkMax = TempJerk[iter];
                t_Jerkmax = t_m[iter];
            }
        }
    }
    *JerkMax = TempJerkMax;
    /* Check max. ave. Jerk */
    if ((TempJerkMax > AllwdJerkMax) & (PlanArcLength == false)) {
        if (t_Jerkmax < 0.25F) {
            t_offset = 0.0F;
        } else if (t_Jerkmax >= (te - 0.25F)) {
            t_offset = te - 0.5F;
        } else {
            t_offset = t_Jerkmax - 0.25F;
        }
        SumJerk = 0.0F;
        t = t_offset;
        SumJerk =
            SumJerk + TRJPLN_CalcTrajJerk(curParam[3], curParam[4], curParam[5],
                                          t, CrvChng, VehVelX);
        {
            uint8 jter;
            for (jter = 1; jter < 26; jter++) {
                t = t + 0.02F;
                SumJerk = SumJerk +
                          TRJPLN_CalcTrajJerk(curParam[3], curParam[4],
                                              curParam[5], t, CrvChng, VehVelX);
            }
        }
        *JerkMax = SumJerk / 26.f;
        if (*JerkMax > AllwdJerkMax) {
            StatusJerkCheck = false;
        } else {
            StatusJerkCheck = true;
        }
    } else {
        StatusJerkCheck = true;
    }

    return StatusJerkCheck;
}

/*****************************************************************************
  Functionname: TRJPLN_CheckCollisionCorridor */ /*!

@brief:
check the potential collision with left and right corridor boundary

@description:
return false if the planned trajectory would cause potential
collision with left or
right corridor boundary . return true if no potential collision
found

@param[in]

@param[out]


@return
*****************************************************************************/
boolean_T TRJPLN_CheckCollisionCorridor(const boolean_T PlanArcLength,
                                        const float32 PH_Traj,
                                        const float32 PH_VisLength,
                                        const float32 CurTgtPoint,
                                        const uint8 EndIndexLeftCridr,
                                        float32 CircleRadius,
                                        const float32 Param[6],
                                        const float32 LeftDistY[100],
                                        boolean_T* CollCheckRightOk,
                                        boolean_T* CollCheckLeftOk) {
    uint8 MaxNumPoints;
    float32 TestPoints, VisLength;
    uint8 i;
    float32 TestPoints_2, TestPoints_3, TestPoints_4, TestPoints_5;
    boolean_T EndOfCheck, CollRightCridr, CollLeftCridr;
    float32 delta_s, delta_t, Y_Traj;
    boolean_T NoCollision;

    /* Init */
    MaxNumPoints = 100;
    EndOfCheck = false;
    CollRightCridr = false;
    CollLeftCridr = false;

    /* Definition of delta_s and delta_t */
    delta_s = 0.7f;
    delta_t = 0.05f;
    i = 0;

    while ((EndOfCheck != true) && (CollRightCridr != true) &&
           (CollLeftCridr != true)) {
        /*Calculate sample points for each trajectory. The sample points must be
         * within visible length */
        if (PlanArcLength == true) {
            VisLength = PH_Traj;
            TestPoints = (float32)(i + 1) * delta_s;  // in Meter
        } else {
            VisLength = PH_VisLength;
            TestPoints = (float32)(i + 1) * delta_t;  // in Seconds
        }

        if ((TestPoints <= VisLength) && (TestPoints <= CurTgtPoint) &&
            (i < EndIndexLeftCridr) && (i < MaxNumPoints)) {
            /*Calculate lateral distance for each test point */
            TestPoints_2 = TestPoints * TestPoints;
            TestPoints_3 = TestPoints_2 * TestPoints;
            TestPoints_4 = TestPoints_3 * TestPoints;
            TestPoints_5 = TestPoints_4 * TestPoints;
            Y_Traj = (((((Param[0]) + ((Param[1]) * TestPoints)) +
                        ((Param[2]) * TestPoints_2)) +
                       ((Param[3]) * TestPoints_3)) +
                      ((Param[4]) * TestPoints_4)) +
                     ((Param[5]) * TestPoints_5);

            /*
             * Check if lateral distance of test point collides with right
             * corridor boundary
             */
            if (Y_Traj < CircleRadius) {
                CollRightCridr = true;
            }
            /*
             * Check if lateral distance of test point collides with left
             * corridor boundary
             */
            if (Y_Traj > (LeftDistY[i] - CircleRadius)) {
                CollLeftCridr = true;
            }

            /*
             *  Next point
             */
            i = i + 1;
        } else {
            EndOfCheck = true;
        }
    }

    /* Bit 2 of signal FasQcBplBtb_QualifierStatusBPL */
    if (CollRightCridr != true) {
        *CollCheckRightOk = true;
    } else {
        *CollCheckRightOk = false;
    }

    /*Bit 3 of signal FasQcBplBtb_QualifierStatusBPL */
    if (CollLeftCridr != true) {
        *CollCheckLeftOk = true;
    } else {
        *CollCheckLeftOk = false;
    }

    NoCollision = (*CollCheckRightOk) & (*CollCheckLeftOk);

    return NoCollision;
}

/*****************************************************************************
  Functionname: TRJPLN_CheckCollisionObject */ /*!

@brief:
check the potential collision with object

@description:
return false if the planned trajectory would cause potential
collision with object.
return true if no potential collision found

@param[in]

@param[out]


@return
*****************************************************************************/
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
                                      const float32 P[6]) {
    boolean_T NoCollision, EndOfCheck;
    float32 delta_t, TestPoints, TestPoints_2, TestPoints_3, TestPoints_4,
        TestPoints_5, t_DeadTime_2, min_t;
    uint8 i;
    float32 X0_Traj, X0_Obj;
    float32 X_Traj, Y_Traj, X_Obj, Y_Obj, X_TrajObj, Y_TrajObj;

    delta_t = 0.05f; /* in Seconds */
    i = 0;
    NoCollision = true;
    EndOfCheck = false;

    /* Calculate start point of trajectory and object according to dead time */
    t_DeadTime_2 = t_DeadTime * t_DeadTime;
    X0_Traj = (v_Traj * t_DeadTime) + ((0.5f * a_Traj) * t_DeadTime_2);
    X0_Obj = (v_Obj * t_DeadTime) + ((0.5f * a_Obj) * t_DeadTime_2);

    while ((EndOfCheck != true) && (NoCollision == true))  //
    {
        /* Calculate sample points for time based optimization (collision check
         * only if time based planning) */
        TestPoints = (float32)(i + 1) * delta_t; /* in Seconds */
        TestPoints_2 = TestPoints * TestPoints;
        TestPoints_3 = TestPoints_2 * TestPoints;
        TestPoints_4 = TestPoints_3 * TestPoints;
        TestPoints_5 = TestPoints_4 * TestPoints;

        /* Calculate x-distance and y-distance of trajectory */
        min_t = TRJPLN_MinFloat((1.f - t_DeadTime), TestPoints);
        X_Traj = (X0_Traj + (v_Traj * TestPoints)) +
                 ((0.5f * a_Traj) * (min_t * min_t));
        Y_Traj = ((((P[0] + (P[1] * TestPoints)) + (P[2] * TestPoints_2)) +
                   (P[3] * TestPoints_3)) +
                  (P[4] * TestPoints_4)) +
                 (P[5] * TestPoints_5);

        /* Calculate x-distance of object */
        X_Obj = (X0_Obj + (v_Obj * TestPoints)) +
                ((0.5f * a_Obj) * (min_t * min_t));
        Y_Obj = 0.f;

        /* Calculate relative motion */
        X_TrajObj = X_Traj - X_Obj;
        Y_TrajObj = Y_Traj - Y_Obj;

        if (X_TrajObj >= (ObstDistX - CircleRadius)) {
            if ((Y_TrajObj <=
                 ((ObstDistY + (0.5f * HindernisWidth)) + CircleRadius)) &&
                (Y_TrajObj >=
                 ((ObstDistY - (0.5 * HindernisWidth)) - CircleRadius))) {
                NoCollision = false;
            }
        }
        /* Next point */
        if ((TestPoints >= VisLength) || (TestPoints >= TgtPoint)) {
            EndOfCheck = true;
        } else {
            i++;
        }
    }
    return NoCollision;
}

/*****************************************************************************
  Functionname: TRJPLN_CheckLaneCross */ /*!

@brief:
        check whether lane cross would be happened with current
planned trajectory

@description:
        return false if lateral distance of test point cross the
right/left corridor boundary

@param[in]

@param[out]


@return
*****************************************************************************/
boolean_T TRJPLN_CheckLaneCross(const float32 PH_VisLength,
                                const float32 CurTgtPoint,
                                const float32 VehicleWidth,
                                const float32 Param[6],
                                const float32 AllowedTimetoCross,
                                const float32 LaneWidth) {
    float32 TestPoints;
    boolean_T CrossRightCridr, CrossLeftCridr;
    boolean_T NoCross;

    CrossRightCridr = false;
    CrossLeftCridr = false;
    NoCross = true;
    // VisLength = PH_VisLength;
    TestPoints = AllowedTimetoCross;

    if (TestPoints >= 0.01) {
        /* Calculate lateral distance for test point */
        float32 TestPoints_2 = TestPoints * TestPoints;
        float32 TestPoints_3 = TestPoints_2 * TestPoints;
        float32 TestPoints_4 = TestPoints_3 * TestPoints;
        float32 TestPoints_5 = TestPoints_4 * TestPoints;
        float32 Y_Traj = (((((Param[0]) + ((Param[1]) * TestPoints)) +
                            ((Param[2]) * TestPoints_2)) +
                           ((Param[3]) * TestPoints_3)) +
                          ((Param[4]) * TestPoints_4)) +
                         ((Param[5]) * TestPoints_5);

        if ((Param[0]) < 0) {
            /* Check if lateral distance of test point cross the right corridor
             * boundary */
            if (Y_Traj < (-LaneWidth + 0.5F * VehicleWidth)) {
                CrossRightCridr = true;
            }
        } else {
            /* Check if lateral distance of test point cross the left corridor
             * boundary */
            if (Y_Traj > (2.f * LaneWidth - 0.5F * VehicleWidth)) {
                CrossLeftCridr = true;
            }
        }
    }

    /*  */
    if ((CrossRightCridr != true) && (CrossLeftCridr != true)) {
        NoCross = true;
    } else {
        NoCross = false;
    }

    return NoCross;
}

/*****************************************************************************
  Functionname: TrajCalc_CalcOptTrajectory */ /*!

@brief: optimal trajectory calculation based on input

@description: calculate the best parameter of fifth order polynomial equation,
which describe
the optimal trajectory
y = A + BX + CX^2 + DX^3 + EX^4 + FX^5

@param[in]
const TRJPLN_calOptInTypeV3_t* Inputs: the input of the
trajectory calculation module
@param[out]

@return
the calculated optimal trajectory parameters
@uml
@startuml
start
:TRJPLN_InitializeOutputs;
note:Initialize all output variables
if(trajectory plan related sample points is invalid) then (yes)
:set trajectory plan status to invalid;
else (no, start trajectory calculation)
:get time and lateral distance weight factor;
:set target sample points size(max is 15);
while(Iterate over all target endpoints)
:TRJPLN_CalcOptTrajectoryInt;
note:calculate trajectory equation \nand evaluate cost
:update best trajectory if cost is lower;
endwhile
if(optimum point was found) then (yes)
:search left tolerance range;
note:Based on the best vertical endpoint, \nsearch horizontally to the
left for endpoints \nwithin the tolerance range;
:search right tolerance range;
note:Based on the best vertical endpoint, \nsearch horizontally to the
right for endpoints \nwithin the tolerance range;
endif
endif
end
@enduml
*****************************************************************************/
TRJPLN_calOptOutTypeV4_t TrajCalc_CalcOptTrajectory(
    const TRJPLN_calOptInTypeV3_t* Inputs) {
    TRJPLN_calOptOutTypeV4_t Outputs;
    uint8 VecLength;
    boolean_T NewBestPoint;
    float32 temp;
    VecLength = 15;

    /* Initialize all variables */
    TRJPLN_InitializeOutputs(&Outputs);
    NewBestPoint = false;
    temp = Inputs->fTrajCalcPlanHorizonTraj_sec;

    if ((Inputs->uiTrajCalcNumTgtPoints_nu == 0) ||
        ((Inputs->uiTrajCalcNumTgtPoints_nu == 1) &&
         (temp < 0.000001f)))  // --> No valid sample points
    {
        Outputs.uiTrajStatus_nu = false;
        Outputs.uiMatrixInverseOK_nu = true;
        Outputs.uiTrajLengthOK_nu = false;

    } else {
        /* Calculate trajectory parameters */
        uint8 ind_dTgtk, k_max, ind_opt;
        float32 te;
        float32 dz_dot, dz_ddot;
        float32 K_d, K_t;

        if (Inputs->uiReplanModeArcLength_nu == true) {
            K_d = Inputs->fKd_nu;
            K_t = Inputs->fKt_nu * Inputs->fWeightArcLengthPlan_nu;
        } else {
            float32 afTableInputX[TPLTJC_VEHVELXKT_TABLENUM_NU] =
                TPLTJC_VEHVELXKT_TABLEX_MPS;
            float32 afTableInputY[TPLTJC_VEHVELXKT_TABLENUM_NU] =
                TPLTJC_KT_TABLEY_MPS2;
            float32 fKtFactorVelX = TUE_CML_LookUpTable2D(
                Inputs->fTrajCalcEgoVelX_mps, afTableInputX, afTableInputY,
                TPLTJC_VEHVELXKT_TABLENUM_NU);
            K_d = Inputs->fKd_nu;
            K_t = Inputs->fKt_nu * fKtFactorVelX;
        }

        k_max = Inputs->uiTrajCalcNumTgtPoints_nu;
        if (k_max > VecLength) {
            k_max = VecLength; /* maximum VecLength is 15. */
        }

        ind_opt = 255; /* --> Initialize index with optimum point to 255 */
        for (ind_dTgtk = 0; ind_dTgtk < k_max; ind_dTgtk++) {
            te = Inputs->afteTrajCalcTgt_nu[ind_dTgtk];
            float32 dTgt_k = Inputs->afTrajCalcTgtDistY_met[ind_dTgtk];
            dz_dot = Inputs->afTrajCalcTgtDistY1stDeriv_nu[ind_dTgtk];
            dz_ddot = Inputs->afrajCalcTgtDistY2ndDeriv_nu[ind_dTgtk];
            float32 dz_toleranz = dTgt_k + 0.f;
            TRJPLN_CalcOptTrajectoryInt(
                dz_toleranz, dz_dot, dz_ddot, te, &NewBestPoint,
                Inputs->fDistYInit_nu, Inputs->fDistY1stDerivInit_nu,
                Inputs->fDistY2ndDerivInit_nu, K_d, K_t, dTgt_k,
                Inputs->uiCheckFtire_nu,
                Inputs->fLatCtrlCoordCridrRightSeg1_Crv_1pm,
                Inputs->fTrajCalcEgoVelX_mps, Inputs->fAclPotVecMin_mps2,
                Inputs->fAclPotVecMax_mps2, Inputs->uiReplanModeArcLength_nu,
                &Outputs, Inputs->fTrajCalcPlanHorizonVisibility_sec,
                Inputs->fTrajCalcPlanHorizonTraj_sec,
                Inputs->uiCheckCridrBoundaries_nu, Inputs->uiCheckObjects_nu,
                Inputs, Inputs->fCrvChng_1pm2, Inputs->fAllwdJerkMax_mps3,
                Inputs->uiCheckJerk_nu, Inputs->uiCheckLaneCross_nu);
            if (NewBestPoint == true) {
                ind_opt = ind_dTgtk;
            }
        } /* Until here all target points have been calculate an an "optimum"
             trajectory has been found (if it exists) */

        /*
         *  Extension of lateral tolerance area
         */
        if (ind_opt != 255) /* at least one optimum point was found */
        {
            uint8 maxLoop, curLoop;
            float32 dz_alt_opt, dz_new, dz_new_opt;
            dz_dot = Inputs->afTrajCalcTgtDistY1stDeriv_nu[ind_opt];
            dz_ddot = Inputs->afrajCalcTgtDistY2ndDeriv_nu[ind_opt];
            te = Inputs->afteTrajCalcTgt_nu[ind_opt];
            /* false means old optimum point: no new point was found */
            NewBestPoint = false;
            maxLoop = 8;

            if ((float32)FD_FABS(Inputs->fTolDistYLeft_met) >=
                0.00000f) { /* Tolerance left */
                dz_alt_opt = Inputs->afTrajCalcTgtDistY_met[ind_opt];
                dz_new = Inputs->afTrajCalcTgtDistY_met[ind_opt] +
                         Inputs->fTolDistYLeft_met;
                TRJPLN_CalcOptTrajectoryInt(
                    dz_new, dz_dot, dz_ddot, te, &NewBestPoint,
                    Inputs->fDistYInit_nu, Inputs->fDistY1stDerivInit_nu,
                    Inputs->fDistY2ndDerivInit_nu, K_d, K_t,
                    Inputs->afTrajCalcTgtDistY_met[ind_opt],
                    Inputs->uiCheckFtire_nu,
                    Inputs->fLatCtrlCoordCridrRightSeg1_Crv_1pm,
                    Inputs->fTrajCalcEgoVelX_mps, Inputs->fAclPotVecMin_mps2,
                    Inputs->fAclPotVecMax_mps2,
                    Inputs->uiReplanModeArcLength_nu, &Outputs,
                    Inputs->fTrajCalcPlanHorizonVisibility_sec,
                    Inputs->fTrajCalcPlanHorizonTraj_sec,
                    Inputs->uiCheckCridrBoundaries_nu,
                    Inputs->uiCheckObjects_nu, Inputs, Inputs->fCrvChng_1pm2,
                    Inputs->fAllwdJerkMax_mps3, Inputs->uiCheckJerk_nu,
                    Inputs->uiCheckLaneCross_nu);
                dz_new = (dz_alt_opt + dz_new) / 2.0f;
                if (NewBestPoint == true) {
                    dz_alt_opt = Inputs->afTrajCalcTgtDistY_met[ind_opt] +
                                 Inputs->fTolDistYLeft_met;
                } else {
                    dz_alt_opt = Inputs->afTrajCalcTgtDistY_met[ind_opt];
                }
                TRJPLN_CalcOptTrajectoryInt(
                    dz_new, dz_dot, dz_ddot, te, &NewBestPoint,
                    Inputs->fDistYInit_nu, Inputs->fDistY1stDerivInit_nu,
                    Inputs->fDistY2ndDerivInit_nu, K_d, K_t,
                    Inputs->afTrajCalcTgtDistY_met[ind_opt],
                    Inputs->uiCheckFtire_nu,
                    Inputs->fLatCtrlCoordCridrRightSeg1_Crv_1pm,
                    Inputs->fTrajCalcEgoVelX_mps, Inputs->fAclPotVecMin_mps2,
                    Inputs->fAclPotVecMax_mps2,
                    Inputs->uiReplanModeArcLength_nu, &Outputs,
                    Inputs->fTrajCalcPlanHorizonVisibility_sec,
                    Inputs->fTrajCalcPlanHorizonTraj_sec,
                    Inputs->uiCheckCridrBoundaries_nu,
                    Inputs->uiCheckObjects_nu, Inputs, Inputs->fCrvChng_1pm2,
                    Inputs->fAllwdJerkMax_mps3, Inputs->uiCheckJerk_nu,
                    Inputs->uiCheckLaneCross_nu);
                dz_new_opt = dz_new;

                for (curLoop = 1; curLoop < maxLoop; curLoop++) {
                    dz_new = (dz_alt_opt + dz_new_opt) / 2.0f;

                    TRJPLN_CalcOptTrajectoryInt(
                        dz_new, dz_dot, dz_ddot, te, &NewBestPoint,
                        Inputs->fDistYInit_nu, Inputs->fDistY1stDerivInit_nu,
                        Inputs->fDistY2ndDerivInit_nu, K_d, K_t,
                        Inputs->afTrajCalcTgtDistY_met[ind_opt],
                        Inputs->uiCheckFtire_nu,
                        Inputs->fLatCtrlCoordCridrRightSeg1_Crv_1pm,
                        Inputs->fTrajCalcEgoVelX_mps,
                        Inputs->fAclPotVecMin_mps2, Inputs->fAclPotVecMax_mps2,
                        Inputs->uiReplanModeArcLength_nu, &Outputs,
                        Inputs->fTrajCalcPlanHorizonVisibility_sec,
                        Inputs->fTrajCalcPlanHorizonTraj_sec,
                        Inputs->uiCheckCridrBoundaries_nu,
                        Inputs->uiCheckObjects_nu, Inputs,
                        Inputs->fCrvChng_1pm2, Inputs->fAllwdJerkMax_mps3,
                        Inputs->uiCheckJerk_nu, Inputs->uiCheckLaneCross_nu);

                    if (NewBestPoint == true) {
                        dz_alt_opt = dz_new_opt;
                        dz_new_opt = dz_new;
                    } else {
                        dz_alt_opt = dz_new;
                        dz_new_opt = dz_new_opt;
                    }
                }
            }

            if ((float32)FD_FABS(Inputs->fTolDistYRight_met) >= 0.00000f) {
                dz_alt_opt = Inputs->afTrajCalcTgtDistY_met[ind_opt];
                dz_new = Inputs->afTrajCalcTgtDistY_met[ind_opt] -
                         Inputs->fTolDistYRight_met;
                TRJPLN_CalcOptTrajectoryInt(
                    dz_new, dz_dot, dz_ddot, te, &NewBestPoint,
                    Inputs->fDistYInit_nu, Inputs->fDistY1stDerivInit_nu,
                    Inputs->fDistY2ndDerivInit_nu, K_d, K_t,
                    Inputs->afTrajCalcTgtDistY_met[ind_opt],
                    Inputs->uiCheckFtire_nu,
                    Inputs->fLatCtrlCoordCridrRightSeg1_Crv_1pm,
                    Inputs->fTrajCalcEgoVelX_mps, Inputs->fAclPotVecMin_mps2,
                    Inputs->fAclPotVecMax_mps2,
                    Inputs->uiReplanModeArcLength_nu, &Outputs,
                    Inputs->fTrajCalcPlanHorizonVisibility_sec,
                    Inputs->fTrajCalcPlanHorizonTraj_sec,
                    Inputs->uiCheckCridrBoundaries_nu,
                    Inputs->uiCheckObjects_nu, Inputs, Inputs->fCrvChng_1pm2,
                    Inputs->fAllwdJerkMax_mps3, Inputs->uiCheckJerk_nu,
                    Inputs->uiCheckLaneCross_nu);

                dz_new = (dz_alt_opt + dz_new) / 2.0f;

                if (NewBestPoint == true) {
                    dz_alt_opt = Inputs->afTrajCalcTgtDistY_met[ind_opt] -
                                 Inputs->fTolDistYRight_met;
                } else {
                    dz_alt_opt = Inputs->afTrajCalcTgtDistY_met[ind_opt];
                }

                TRJPLN_CalcOptTrajectoryInt(
                    dz_new, dz_dot, dz_ddot, te, &NewBestPoint,
                    Inputs->fDistYInit_nu, Inputs->fDistY1stDerivInit_nu,
                    Inputs->fDistY2ndDerivInit_nu, K_d, K_t,
                    Inputs->afTrajCalcTgtDistY_met[ind_opt],
                    Inputs->uiCheckFtire_nu,
                    Inputs->fLatCtrlCoordCridrRightSeg1_Crv_1pm,
                    Inputs->fTrajCalcEgoVelX_mps, Inputs->fAclPotVecMin_mps2,
                    Inputs->fAclPotVecMax_mps2,
                    Inputs->uiReplanModeArcLength_nu, &Outputs,
                    Inputs->fTrajCalcPlanHorizonVisibility_sec,
                    Inputs->fTrajCalcPlanHorizonTraj_sec,
                    Inputs->uiCheckCridrBoundaries_nu,
                    Inputs->uiCheckObjects_nu, Inputs, Inputs->fCrvChng_1pm2,
                    Inputs->fAllwdJerkMax_mps3, Inputs->uiCheckJerk_nu,
                    Inputs->uiCheckLaneCross_nu);
                dz_new_opt = dz_new;

                for (curLoop = 1; curLoop < maxLoop; curLoop++) {
                    dz_new = (dz_alt_opt + dz_new_opt) / 2.0f;

                    TRJPLN_CalcOptTrajectoryInt(
                        dz_new, dz_dot, dz_ddot, te, &NewBestPoint,
                        Inputs->fDistYInit_nu, Inputs->fDistY1stDerivInit_nu,
                        Inputs->fDistY2ndDerivInit_nu, K_d, K_t,
                        Inputs->afTrajCalcTgtDistY_met[ind_opt],
                        Inputs->uiCheckFtire_nu,
                        Inputs->fLatCtrlCoordCridrRightSeg1_Crv_1pm,
                        Inputs->fTrajCalcEgoVelX_mps,
                        Inputs->fAclPotVecMin_mps2, Inputs->fAclPotVecMax_mps2,
                        Inputs->uiReplanModeArcLength_nu, &Outputs,
                        Inputs->fTrajCalcPlanHorizonVisibility_sec,
                        Inputs->fTrajCalcPlanHorizonTraj_sec,
                        Inputs->uiCheckCridrBoundaries_nu,
                        Inputs->uiCheckObjects_nu, Inputs,
                        Inputs->fCrvChng_1pm2, Inputs->fAllwdJerkMax_mps3,
                        Inputs->uiCheckJerk_nu, Inputs->uiCheckLaneCross_nu);

                    if (NewBestPoint == true) {
                        dz_alt_opt = dz_new_opt;
                        dz_new_opt = dz_new;
                    } else {
                        dz_alt_opt = dz_new;
                        dz_new_opt = dz_new_opt;
                    }
                }
            }
        }

    } /* --> End of calculation of parameters of new trajectory */
    return Outputs;
}

boolean_T TRJPLN_CheckNewBestPoint(const uint8 uiTrajStatus_nu,
                                   const boolean_T StatusTrajPlan,
                                   const float32 curCost,
                                   const float32 fOptimalCost_nu) {
    if (uiTrajStatus_nu == true) {
        if (StatusTrajPlan == true) {
            if (curCost < fOptimalCost_nu) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    } else {
        if (StatusTrajPlan == true) {
            return true;
        } else {
            if (curCost < fOptimalCost_nu) {
                return true;
            } else {
                return false;
            }
        }
    }
}
/*****************************************************************************
  Functionname: TRJPLN_CalcOptTrajectoryInt */ /*!

@brief: calculate the optimal trajectory if the target trajectory is existed

@description: update optimal trajectory parameters if the new found trajectory's
cost is lower
than old trajectory's cost

@param[in]

@param[out]


@return
*****************************************************************************/
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
                                 const boolean_T CheckLaneCross) {
    boolean_T matrixInvertible;
    boolean_T StatusPotVecCheck, FTireAclMinOk, FTireAclMaxOk;
    boolean_T StatusCollCheckCridr, CollCheckRightOk, CollCheckLeftOk,
        StatusCollCheckObj;
    boolean_T StatusLaneCrossCheck;
    boolean_T StatusTrajPlan;
    float32 curTolerance;
    float32 curParams[6];
    float32 Ax[3][3];
    float32 Bx[3];
    float32 curCost, tmpAmax;
    uint8 i, j;
    boolean_T StatusJerkCheck;
    float32 JerkMax;

    /* Initialisierung der Variablen */
    tmpAmax = 0.0f;
    JerkMax = 0.0f;
    StatusPotVecCheck = true;
    StatusCollCheckCridr = true;
    StatusCollCheckObj = true;
    FTireAclMinOk = true;
    FTireAclMaxOk = true;
    CollCheckRightOk = true;
    CollCheckLeftOk = true;
    StatusJerkCheck = true;
    StatusLaneCrossCheck = true;

    curTolerance = dz_new - dz_original;
    for (i = 0; i < 3; i++) {
        Bx[i] = 0.0F;
        for (j = 0; j < 3; j++) {
            Ax[i][j] = 0.0F;
        }
    }

    /* Calculation of parameters p0, p1, p2 */
    curParams[0] = d_AW;
    curParams[1] = d_AWdot;
    curParams[2] = d_AWddot / 2;

    /* Calculation of parameters p3, p4, p5 */
    TRJPLN_CalcAxBx(Ax, Bx, curParams, te, dz_new, dz_dot, dz_ddot);
    matrixInvertible = TRJPLN_CalcParameters(curParams, Ax, Bx);

    /* Calculation of cost function */

    if (matrixInvertible != false) {
        curCost =
            TRJPLN_CalcCostFunction(curParams, te, Kd, Kt, dz_new, dz_original);
    } else {
        curCost = 99e+10F;
    }

    /* Check of limitations */
    if ((CheckFTire == true) && (matrixInvertible != false)) {
        /* Check potential vector (friction value) */
        StatusPotVecCheck = TRJPLN_CheckPotVector(
            curParams, &tmpAmax, Kr, te, VehVelX, FTireAclMin, FTireAclMax,
            PlanArcLength, &FTireAclMinOk, &FTireAclMaxOk);
    }

    /* Jerk Check */
    if ((CheckJerk == true) && (matrixInvertible != false)) {
        StatusJerkCheck =
            TRJPLN_CheckJerk(curParams, &JerkMax, CrvChng, te, VehVelX,
                             AllwdJerkMax, PlanArcLength);
    }

    /* Check collision with corridor boundaries */
    if ((CheckCollCridr == true) && (matrixInvertible != false)) {
        StatusCollCheckCridr = TRJPLN_CheckCollisionCorridor(
            PlanArcLength, PH_Traj, PH_VisLength, te,
            Inputs->uiTrajCalcNumCridrPointsLeft_nu,
            Inputs->fTrajCalcCircleRadius_met, curParams,
            Inputs->afTrajCalcDistYLeft_met, &CollCheckRightOk,
            &CollCheckLeftOk);
    }

    /* Check collision with object */
    if ((CheckCollObj == true) && (matrixInvertible != false) &&
        (PlanArcLength == false)) {
        StatusCollCheckObj = TRJPLN_CheckCollisionObject(
            Inputs->fTrajCalcDeadTime_sec, Inputs->fTrajCalcTrajVel_mps,
            Inputs->fTrajCalcTrajAcl_mps2, Inputs->fTrajCalcObjVelX_mps,
            Inputs->fTrajCalcObjAclX_mps2, Inputs->fObstacle_DistX_met,
            Inputs->fObstacle_DistY_met, Inputs->fObstacle_Width_met,
            Inputs->fTrajCalcCircleRadius_met, PH_VisLength, te, curParams);
    }

    /* Lane crossing check */
    if ((CheckLaneCross == true) && (matrixInvertible != false) &&
        (PlanArcLength == false)) {
        StatusLaneCrossCheck = TRJPLN_CheckLaneCross(
            PH_VisLength, te, Inputs->fVehicleWidth_met, curParams,
            Inputs->fAllowedTimetoCross_sec, Inputs->fLaneWidth_met);
    }

    /* Select optimum trajectory */
    if ((StatusPotVecCheck == true) && (StatusCollCheckCridr == true) &&
        (StatusCollCheckObj == true) && (StatusJerkCheck == true) &&
        (StatusLaneCrossCheck == true) && (matrixInvertible != false)) {
        StatusTrajPlan = true;
    } else {
        StatusTrajPlan = false;
    }

    *NewBestPoint =
        TRJPLN_CheckNewBestPoint(Outputs->uiTrajStatus_nu, StatusTrajPlan,
                                 curCost, Outputs->fOptimalCost_nu);

    if (*NewBestPoint == true) {
        uint8 iter;
        for (iter = 0; iter < 6; iter++) {
            Outputs->afTrajParam_nu[iter] = curParams[iter];
        }
        if (PlanArcLength == true) {
            Outputs->fArcLengthTrajEnd_met = te;
        } else {
            Outputs->fTrajCalcTimeTrajEnd_sec = te;
        }
        Outputs->fMaxAclTraj_mps2 = tmpAmax;
        Outputs->fOptimalCost_nu = curCost;
        Outputs->fOptimalTolerance_nu = curTolerance;
        Outputs->uiTrajStatus_nu = StatusTrajPlan;
        Outputs->uiFtireMinOK_nu = FTireAclMinOk;
        Outputs->uiFtireMaxOK_nu = FTireAclMaxOk;
        Outputs->uiCollDetRightOK_nu = CollCheckRightOk;
        Outputs->uiCollDetLeftOK_nu = CollCheckLeftOk;
        Outputs->uiCollDetObjOK_nu = StatusCollCheckObj;
        Outputs->fMaxJerkTraj_mps3 = JerkMax;
        Outputs->uiMaxJerkOK_nu = StatusJerkCheck;
        Outputs->uiLaneCrossOK_nu = StatusLaneCrossCheck;
    }

    if (Outputs->uiMatrixInverseOK_nu == false) {
        Outputs->uiMatrixInverseOK_nu = matrixInvertible;
    }
}

/*****************************************************************************
  Functionname: TRJPLN_MinFloat                                           */ /*!

          @brief:
                                return minimum value of input two values

          @description:
                                return minimum value of input two values

          @param[in]

          @param[out]


          @return
        *****************************************************************************/
float32 TRJPLN_MinFloat(const float32 A, const float32 B) {
    float32 C;
    if (A >= B) {
        C = B;
    } else {
        C = A;
    }
    return C;
}

#define STOP_SEC_CODE

/*-------------------------------------------------------------------------------------------*
  END OF FILE
 *-------------------------------------------------------------------------------------------*/
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
