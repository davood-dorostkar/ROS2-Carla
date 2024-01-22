/*********************************************************************

PROJECT:        TJA Base Development

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

/* -----------------------------------------------------------------------------

        I N C L U D E S

 * -----------------------------------------------------------------------------
*/
#include "trjpln_calFrenetTransformation.h"

/* -----------------------------------------------------------------------------

        D E C L A R A T I O N   F U N C T I O N S

 * -----------------------------------------------------------------------------
*/

#define START_SEC_CODE

/* -----------------------------------------------------------------------------
        M I S R A
 *__________________________________________
*/

/* -----------------------------------------------------------------------------

        F U N C T I O N S

 * -----------------------------------------------------------------------------
*/

/*****************************************************************************
  Functionname: TRJPLN_FT_InitializeOutputs */ /*!

                                @brief: initial output process of frenet
                                transformation module

                                @description: set default value to the output of
                                frenet transformation module

                                @param[in/out]
                                    TRJPLN_calFTOutType_t* Outputs: the output
                                pointer of frenet
                                transformation module
                                @param[out]

                                @return
                                *****************************************************************************/
void TRJPLN_FT_InitializeOutputs(TRJPLN_calFTOutType_t* Outputs) {
    uint8 i, j;
    for (i = 0; i < 15; i++) {
        Outputs->afTgtDistY_met[i] = 99.0F;
        Outputs->afTgtDistY1stDeriv_nu[i] = 99.0F;
        Outputs->afTgtDistY2ndDeriv_nu[i] = 99.0F;
        Outputs->afTgtPoints_nu[i] = 99.0F;
        Outputs->afCridrRight_Crv_1pm[i] = 0.0F;
    }

    for (j = 0; j < 100; j++) {
        Outputs->afDistYLeft_met[j] = 99.0F;
    }
    Outputs->fDistYCurValue_met = 0.0F;
    Outputs->fDistY1stDerivCurValue_nu = 0.0F;
    Outputs->fDistY2ndDerivCurValue_nu = 0.0F;
    Outputs->fTrajVel_mps = 0.0F;
    Outputs->fTrajAcl_mps2 = 0.0F;
    Outputs->fTrajDistYPrevious_met = 0.0F;
    Outputs->fTrajDistYPrev1stDeriv_nu = 0.0F;
    Outputs->fTrajDistYPrev2ndDeriv_nu = 0.0F;
    Outputs->uiNumTgtPoints_nu = 0;
    Outputs->fTrajPlanningHorizon_sec = 0.0F;
    Outputs->fPlanHorizonVisRange_sec = 0.0F;
    Outputs->f2ndDerivDevHeading_nu = 0.0F;
    Outputs->f1stDerivDevHeading_nu = 0.0F;
    Outputs->fDstYCurValuePreview_met = 0.0f;
    Outputs->fDstY1stDrvCurValPrvw_mps = 0.0f;
    Outputs->fFrenTrafHeadingPrvw_sec = 0.0f;
    Outputs->uiNumPointsCridrLeft_nu = 0;
    Outputs->uiNumPointsCridrRight_nu = 0;
}

/*****************************************************************************
  Functionname: TRJPLN_FT_InitializeIntVectors */ /*!

                             @brief: array initialize util function

                             @description:array initialize util function

                             @param[in/out]
                             float32 A[61], float32 B[61], float32 C[61],
                             float32 D[61]:
                             initialized array
                             @param[out]

                             @return
                             *****************************************************************************/
void TRJPLN_FT_InitializeIntVectors(float32 A[101],
                                    float32 B[101],
                                    float32 C[101],
                                    float32 D[101]) {
    uint8 indx;
    for (indx = 0; indx < 101; indx++) {
        A[indx] = 0.0f;
        B[indx] = 0.0f;
        C[indx] = 0.0f;
        D[indx] = 0.0f;
    }
}

/*****************************************************************************
  Functionname: TRJPLN_FT_TransformCurValues */ /*!

                               @brief: calculate ego movement data in right
                               corridor based frenet coordinate
                               system

                               @description: we will calculate ego DistY, VelY,
                               AccelY, VelX, AccelX,
                               DeltaHeading first derivative
                                 and DeltaHeading second derivative in right
                               corridor based
                               coordinate system .

                               @param[in]
                                 const boolean_T PlanArcLength: the trajectory
                               plan is based on
                               ArcLength or Time. depends on ego velocity
                                 const float32 DevDistY: vehicle ego lateral
                               distance in right
                               corridor coordinate system
                                 const float32 DevHeading: vehicle heading angle
                               in right corridor
                               coordinate system
                                 const float32 Vx: ego velocity in cartesian
                               coordinate system
                                 const float32 Ax: ego acceleration in cartesian
                               coordinate system
                                 const float32 Kr: right corridor curvature in
                               cartesian coordinate
                               system
                                 const float32 Kr_prime: right corridor
                               curvature change in
                               cartesian coordinate system
                                 const float32 Kist: ego curvature in cartesian
                               coordinate system
                                 float32 CurHdngAndDelta: current vehicle
                               heading angle which is
                               previewed by last cycle output
                                 float32 Kstart: the start planned curvature,
                               normally it would be
                               ego curvature
                                 float32 Ksoll: previewed target corridor's
                               curvature in last cycle
                               @param[out]
                                 float32* dist: vehicle ego lateral distance in
                               right corridor
                               coordinate system
                                 float32* dist1stDeriv: vehicle ego lateral
                               velocity in right
                               corridor coordinate system
                                 float32* dist2ndDeriv: vehicle ego lateral
                               acceleration in right
                               corridor coordinate system
                                 float32* v_traj: vehicle ego tangent
                               velocity(velocity X) in right
                               corridor coordinate system
                                 float32* a_traj:  vehicle ego tangent
                               acceleration (acceleration X)
                               in right corridor coordinate system
                                 float32* DeltaHdng1stDeriv: vehicle delta
                               heading angle first
                               derivative with right corridor coordinate system
                                 float32* DeltaHdng2ndDeriv: vehicle delta
                               heading angle second
                               derivative with right corridor coordinate system
                               @return
                               *****************************************************************************/
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
                                  float32 Ksoll) {
    float32 OneminusKrd, CosDeltathetaCur, SinDeltathetaCur, TanDeltathetaCur,
        dist_prime, temp_1, v_traj_2, CosDeltathetaCur_2;
    float32 CosHeadingCurAndDeltaHdg, SinHeadingCurAndDeltaHdg,
        TanHeadingCurAndDeltaHdg, dist_primeHeadingCurAndDeltaHdg,
        CosHeadingCurAndDeltaHdg_2;

    *dist = DevDistY;

    CosDeltathetaCur = TRJPLN_FT_AvoidZero((float32)FD_COS(DevHeading));
    SinDeltathetaCur = (float32)FD_SIN(DevHeading);
    TanDeltathetaCur = (float32)FD_TAN(DevHeading);
    CosDeltathetaCur_2 = CosDeltathetaCur * CosDeltathetaCur;

    /* Control error Heading */
    CosHeadingCurAndDeltaHdg =
        TRJPLN_FT_AvoidZero((float32)FD_COS(CurHdngAndDelta));
    SinHeadingCurAndDeltaHdg = (float32)FD_SIN(CurHdngAndDelta);
    TanHeadingCurAndDeltaHdg = (float32)FD_TAN(CurHdngAndDelta);
    CosHeadingCurAndDeltaHdg_2 =
        CosHeadingCurAndDeltaHdg * CosHeadingCurAndDeltaHdg;

    /* Temporary values: (1-Kr*d) */
    OneminusKrd = 1.0F - (Kr * (*dist));

    OneminusKrd = TRJPLN_FT_AvoidZero(OneminusKrd);

    *v_traj = (Vx * CosDeltathetaCur) / OneminusKrd;
    temp_1 = CosDeltathetaCur / OneminusKrd;

    dist_prime = OneminusKrd * TanDeltathetaCur;

    /* Control error Heading */
    dist_primeHeadingCurAndDeltaHdg = OneminusKrd * TanHeadingCurAndDeltaHdg;
    v_traj_2 = (*v_traj) * (*v_traj);

    *a_traj = temp_1 * (Ax - ((v_traj_2 / CosDeltathetaCur) *
                              ((dist_prime * ((Kist / temp_1) - Kr)) -
                               ((Kr_prime * (*dist)) + (Kr * dist_prime)))));

    /*
     * Arc length based planning
     */
    if (PlanArcLength == true) {
        *dist1stDeriv = dist_prime;

        *dist2ndDeriv = ((-((Kr_prime * (*dist)) + (Kr * (*dist1stDeriv)))) *
                         TanDeltathetaCur) +
                        ((OneminusKrd / CosDeltathetaCur_2) *
                         (((OneminusKrd / CosDeltathetaCur) * Kstart) - Kr));

        /* Control error Heading */
        *DeltaHdng1stDeriv = dist_primeHeadingCurAndDeltaHdg;
        *DeltaHdng2ndDeriv =
            ((-((Kr_prime * (*dist)) + (Kr * (*DeltaHdng1stDeriv)))) *
             TanHeadingCurAndDeltaHdg) +
            ((OneminusKrd / CosHeadingCurAndDeltaHdg_2) *
             (((OneminusKrd / CosHeadingCurAndDeltaHdg) * Ksoll) - Kr));
    } else {
        /*
         * Time based planning
         */
        *dist1stDeriv = Vx * SinDeltathetaCur;

        *dist2ndDeriv =
            (Ax * SinDeltathetaCur) +
            ((Vx * CosDeltathetaCur) * ((Kstart * Vx) - (Kr * (*v_traj))));

        /* Control error Heading */
        *DeltaHdng1stDeriv = Vx * SinHeadingCurAndDeltaHdg;
        *DeltaHdng2ndDeriv = (Ax * SinHeadingCurAndDeltaHdg) +
                             ((Vx * CosHeadingCurAndDeltaHdg) *
                              ((Ksoll * Vx) - (Kr * (*v_traj))));
    }
}

/*****************************************************************************
  Functionname: TRJPLN_FT_CalcStartPoint_ArcLength */ /*!

                         @brief: Calculate start point for trajectory

                         @description:
                         Calculate start point for target trajectory in
                         arc-length based plan
                         mode.
                         @param[in]

                         @param[out]

                         @return
                         *****************************************************************************/
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
                                        const boolean_T UseCridrRiX0) {
    float32 Xi, Xi_pre, Si, Si_pre, deltaS, deltaSSmall, thetar_x_pre;
    uint8 i;
    boolean_T TgtSg1XFound;

    /* Initial Start points and intermediate values*/
    *x_Start = 0;
    *s_Start = 0;
    *PlanAreaTgt = 0;
    Si = 0;
    Xi = 0;
    i = 0;
    deltaS = 0.6f;
    deltaSSmall = 0.2f;
    TgtSg1XFound = false;
    Xi_pre = 0;
    Si_pre = 0;

    while (TgtSg1XFound == false) {
        Xi_pre = Xi;
        Si_pre = Si;
        Si = (float32)i * deltaS;
        thetar_x_pre = TRJPLN_FT_Thetar(CridrRiSg1Heading, Kr, Kr_prime, Xi_pre,
                                        CridrRiSg1X, UseCridrRiX0);
        Xi = Xi_pre + ((Si - Si_pre) * (float32)FD_COS(thetar_x_pre));

        /* To avoid endless go at most up to 70m */
        if ((Xi >= TgtSg1X) || (Xi >= 70.0f)) {
            TgtSg1XFound = 1;
        } else {
            i++;
        }
    }  // ----> End while(!TgtSg1XFound)

    if ((float32)FD_FABS(Xi - TgtSg1X) <= 0.0001f) {
        *s_Start = Si;
        *x_Start = TgtSg1X;
        *PlanAreaTgt = (*x_Start) + TgtSg1Length;
    } else if (Xi > TgtSg1X) {
        /* Go back one step and search with smaller step size */
        TgtSg1XFound = false;
        Xi = Xi_pre;
        Si = Si_pre;
        while (TgtSg1XFound == false) {
            Xi_pre = Xi;
            Si_pre = Si;
            Si = Si + deltaSSmall;
            thetar_x_pre = TRJPLN_FT_Thetar(CridrRiSg1Heading, Kr, Kr_prime,
                                            Xi_pre, CridrRiSg1X, UseCridrRiX0);
            Xi = Xi_pre + ((Si - Si_pre) * (float32)FD_COS(thetar_x_pre));

            /* To avoid endless go at most up to 70m */
            if ((Xi >= TgtSg1X) || (Xi >= 70.0f)) {
                TgtSg1XFound = 1;
            }
        }
        *s_Start = Si;
        *x_Start = Xi;
        *PlanAreaTgt = (*x_Start) + TgtSg1Length;
    } else {
        *s_Start = Si;
        *x_Start = Xi;
        *PlanAreaTgt = (*x_Start) + TgtSg1Length;
    }
    /* No time for arc-length based planning */
    *t_Start = 0.0f;
}

/*****************************************************************************
  Functionname: TRJPLN_FT_CalcStartPoint_Time */ /*!

                              @brief: Calculate start point for trajectory

                              @description:
                              Calculate start point for target trajectory in
                              time based plan mode.
                              @param[in]

                              @param[out]

                              @return
                              *****************************************************************************/
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
                                   const boolean_T UseCridrRiX0) {
    float32 Xi, Xi_pre, Si, Si_pre, thetar_x_pre, deltaT, deltaTSmall, ti, temp,
        temp_2;
    uint8 i;
    boolean_T TgtSg1XFound;

    /* Initial start points and intermediate values */
    *x_Start = 0;
    *s_Start = 0;
    *t_Start = 0;
    *PlanAreaTgt = 0;
    Xi_pre = 0;
    Si_pre = 0;
    deltaT = 0.2f;
    deltaTSmall = 0.02f;
    Si = 0.0f;
    Xi = 0.0f;
    i = 1;
    TgtSg1XFound = false;
    ti = 0;

    while (TgtSg1XFound == false) {
        Xi_pre = Xi;
        Si_pre = Si;
        ti = deltaT * (float32)i;
        temp = TRJPLN_FT_MinFloat(1.0f, ti);
        temp_2 = temp * temp;
        Si = (v_traj * ti) + ((0.5f * a_traj) * temp_2);
        thetar_x_pre = TRJPLN_FT_Thetar(CridrRiSg1Heading, Kr, Kr_prime, Xi_pre,
                                        CridrRiSg1X, UseCridrRiX0);
        Xi = Xi_pre + ((Si - Si_pre) * (float32)FD_COS(thetar_x_pre));

        /* To avoid endless go at most up to 70m */
        if ((Xi >= TgtSg1X) || (Xi >= 70.0f)) {
            TgtSg1XFound = 1;
        } else {
            i++;
        }
    }
    if ((float32)FD_FABS(Xi - TgtSg1X) <= 0.0001f) {
        *s_Start = Si;
        *x_Start = TgtSg1X;
        *t_Start = deltaT * (float32)i;
        *PlanAreaTgt = (*x_Start) + TgtSg1Length;
    } else if (Xi > TgtSg1X) {
        /* Go back one step and search with smaller step size */
        TgtSg1XFound = false;
        Si = Si_pre;
        Xi = Xi_pre;
        ti = deltaT * (float32)(i - 1);

        while (TgtSg1XFound == false) {
            ti = ti + deltaTSmall;
            temp = TRJPLN_FT_MinFloat(1.0f, ti);
            temp_2 = temp * temp;
            Si = (v_traj * ti) + ((0.5f * a_traj) * temp_2);
            thetar_x_pre = TRJPLN_FT_Thetar(CridrRiSg1Heading, Kr, Kr_prime,
                                            Xi_pre, CridrRiSg1X, UseCridrRiX0);
            Xi = Xi_pre + ((Si - Si_pre) * (float32)FD_COS(thetar_x_pre));

            /* To avoid endless go at most up to 70m */
            if ((Xi >= TgtSg1X) || (Xi >= 70.0f)) {
                TgtSg1XFound = 1;
            }
        }
        *s_Start = Si;
        *x_Start = Xi;
        *PlanAreaTgt = (*x_Start) + TgtSg1Length;
        *t_Start = ti;
    } else {
        *s_Start = Si;
        *x_Start = Xi;
        *PlanAreaTgt = (*x_Start) + TgtSg1Length;
        *t_Start = ti;
    }
}

/*****************************************************************************
  Functionname: TRJPLN_FT_CalcTargetCurvePoints */ /*!

                            @brief: Transformation of target curve to Frenet

                            @description:
                            calculate related value sample point of target
                            trajectory, including
                            DistY,
                            DistY 1st derivative, DistY 2nd derivative...
                            @param[in]

                            @param[out]

                            @return
                            *****************************************************************************/
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
                                     const float32 CridrRiSg1Y) {
    float32 x_Start, t_Start, s_Start, PlanAreaTgt;
    float32 TrajPlanHorizon, PlanHorizonVisRange;
    float32 TgtDistYPoint[101], TgtDistY1stDerivPoint[101],
        TgtDistY2ndDerivPoint[101], TgtPoints[101];
    uint8 i, NumTgtPoints, VectorLength;
    float32 ZieltrajSg1YNew, ZieltrajSg1HeadingNew, ZieltrajSg1KruemmungNew;
    TRJPLN_FT_InitializeIntVectors(TgtDistYPoint, TgtDistY1stDerivPoint,
                                   TgtDistY2ndDerivPoint, TgtPoints);

    /* Initial start points and intermediate values */
    x_Start = 0;
    t_Start = 0;
    s_Start = 0;
    PlanAreaTgt = 0;
    NumTgtPoints = 0;
    VectorLength = 15;

    /* if long distance less than zero no need to continue search */
    if (TgtSg1X <= 0) {
        x_Start = 0;
        t_Start = 0;
        s_Start = 0;
        PlanAreaTgt = TgtSg1Length;
    } else {
        if (PlanArcLength == true) {
            /*  arc length based: Calculate start point for trajectory */
            TRJPLN_FT_CalcStartPoint_ArcLength(
                &x_Start, &t_Start, &s_Start, &PlanAreaTgt, TgtSg1X,
                TgtSg1Length, CridrRiSg1Heading, Kr, Kr_prime, CridrRiSg1X,
                UseCridrRiX0);
        } else {
            /* time based: Calculate start point for trajectory */
            TRJPLN_FT_CalcStartPoint_Time(&x_Start, &t_Start, &s_Start,
                                          &PlanAreaTgt, v_traj, a_traj, TgtSg1X,
                                          TgtSg1Length, CridrRiSg1Heading, Kr,
                                          Kr_prime, CridrRiSg1X, UseCridrRiX0);
        }
    }
    /* Correction of target trajectory X0 */
    if (UseTgtTrajX0 == true) {
        ZieltrajSg1YNew =
            TRJPLN_FT_Y(TgtTrajSg1Y, TgtTrajSg1Hdng, TgtTrajSg1Crv,
                        TgtTrajSg1ChngOfCrv, x_Start, TgtSg1X);
        ZieltrajSg1HeadingNew =
            TRJPLN_FT_Theta(TgtTrajSg1Hdng, TgtTrajSg1Crv, TgtTrajSg1ChngOfCrv,
                            x_Start, TgtSg1X);
        ZieltrajSg1KruemmungNew =
            TRJPLN_FT_K(TgtTrajSg1Crv, TgtTrajSg1ChngOfCrv, x_Start, TgtSg1X);
    } else {
        ZieltrajSg1YNew = TgtTrajSg1Y;
        ZieltrajSg1HeadingNew = TgtTrajSg1Hdng;
        ZieltrajSg1KruemmungNew = TgtTrajSg1Crv;
    }

    if (PlanArcLength == true) {
        /*  arc length based: Calculate target lateral distances and derivatives
         */
        TRJPLN_FT_CalcTargetCurve_ArcLength(
            &NumTgtPoints, TgtPoints, TgtDistYPoint, TgtDistY1stDerivPoint,
            TgtDistY2ndDerivPoint, &TrajPlanHorizon, &PlanHorizonVisRange,
            x_Start, s_Start, CridrRiSg1Heading, Kr, Kr_prime, CridrRiSg1Length,
            PlanAreaTgt, ZieltrajSg1YNew, ZieltrajSg1HeadingNew,
            ZieltrajSg1KruemmungNew, TgtTrajSg1ChngOfCrv, v_traj, CridrRiSg1X,
            UseCridrRiX0, CridrRiSg1Y);
    } else {
        /* time based: Calculate target lateral distances and derivatives */
        TRJPLN_FT_CalcTargetCurve_Time(
            &NumTgtPoints, TgtPoints, TgtDistYPoint, TgtDistY1stDerivPoint,
            TgtDistY2ndDerivPoint, &TrajPlanHorizon, &PlanHorizonVisRange,
            x_Start, s_Start, t_Start, CridrRiSg1Heading, Kr, Kr_prime,
            CridrRiSg1Length, PlanAreaTgt, ZieltrajSg1YNew,
            ZieltrajSg1HeadingNew, ZieltrajSg1KruemmungNew, TgtTrajSg1ChngOfCrv,
            v_traj, a_traj, PlanHorizon, CridrRiSg1X, UseCridrRiX0,
            CridrRiSg1Y);
    }

    Outputs->uiNumTgtPoints_nu = (uint8)FD_CEIL((float32)NumTgtPoints / 2.f);
    Outputs->fTrajPlanningHorizon_sec = TrajPlanHorizon;
    Outputs->fPlanHorizonVisRange_sec = PlanHorizonVisRange;

    /* Always send last 15 points. */

    /* If NumTgtPoints <= VectorLength : */
    if (Outputs->uiNumTgtPoints_nu <= VectorLength) {
        for (i = 0; i < Outputs->uiNumTgtPoints_nu; i++) {
            Outputs->afTgtDistY_met[i] = TgtDistYPoint[2 * i];
            Outputs->afTgtDistY1stDeriv_nu[i] = TgtDistY1stDerivPoint[2 * i];
            Outputs->afTgtDistY2ndDeriv_nu[i] = TgtDistY2ndDerivPoint[2 * i];
            Outputs->afTgtPoints_nu[i] = TgtPoints[2 * i];
        }
    }
    /* If NumTgtPoints > VectorLength : */
    else {
        uint8 rest, index;
        rest = Outputs->uiNumTgtPoints_nu - VectorLength;
        for (i = 0; i < VectorLength; i++) {
            index = rest + i;
            Outputs->afTgtDistY_met[i] = TgtDistYPoint[2 * index];
            Outputs->afTgtDistY1stDeriv_nu[i] =
                TgtDistY1stDerivPoint[2 * index];
            Outputs->afTgtDistY2ndDeriv_nu[i] =
                TgtDistY2ndDerivPoint[2 * index];
            Outputs->afTgtPoints_nu[i] = TgtPoints[2 * index];
        }
        Outputs->uiNumTgtPoints_nu = VectorLength;
    }
}

/*****************************************************************************
  Functionname: TRJPLN_FT_CalcTargetCurve_Time */ /*!

                             @brief: Calculate target lateral values

                             @description: Calculate target lateral distances
                             and derivatives in time based
                             plan mode

                             @param[in]

                             @param[out]

                             @return
                             *****************************************************************************/
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
                                    const float32 CridrRiSg1Y) {
    float32 Xi, Xi_pre, Si, Si_pre, thetar_x_pre, DeltaT, Sti_dot, ti, tmax,
        tmpPlanHorizon, temp1, temp2;
    uint8 i;
    boolean_T PlanningEnd;
    boolean_T LengthEnd;
    float32 TgtDistY, TgtDistY_prime, TgtDistY_primeprime;
    float32 temp;
    float32 minFT_t1;

    /* Initial start points and intermediate values */
    temp = 0;
    *NumTgtPoints = 0;
    *PlanHorizonVisRange = 0;
    *PlanHorizonFT = 0;
    PlanningEnd = false;
    LengthEnd = false;
    DeltaT = 0.1f;
    i = 0;
    Xi = x_Start;
    Si = s_Start;
    float32 afTableInputX[TPLTJC_DELTATVEL_TABLENUM_NU] =
        TPLTJC_VEHVELXDELTAT_TABLEX_MPS;
    float32 afTableInputY[TPLTJC_DELTATVEL_TABLENUM_NU] =
        TPLTJC_DELTATVEL_TABLEY_MPS2;
    DeltaT = TUE_CML_LookUpTable2D(v_traj, afTableInputX, afTableInputY,
                                   TPLTJC_DELTATVEL_TABLENUM_NU);

    temp2 = (float32)FD_FABS(TRJPLN_FT_MinFloat(PlanAreaTgt, CridrRiSg1Length));

    tmpPlanHorizon =
        (float32)FD_FLOOR((float32)(PlanHorizon / (2.0f * DeltaT))) *
        (2.0f * DeltaT);
    tmax = tmpPlanHorizon + t_Start;

    while (PlanningEnd == false) {
        Xi_pre = Xi;
        Si_pre = Si;
        thetar_x_pre = TRJPLN_FT_Thetar(CridrRiSg1Heading, Kr, Kr_prime, Xi_pre,
                                        CridrRiSg1X, UseCridrRiX0);
        ti = t_Start + (DeltaT * (float32)i);

        TgtPoints[i] = ti;

        minFT_t1 = TRJPLN_FT_MinFloat(1, ti);
        Sti_dot = v_traj + (a_traj * minFT_t1);
        Si = (v_traj * ti) + ((0.5f * a_traj) * (minFT_t1 * minFT_t1));
        Xi = Xi_pre + ((Si - Si_pre) * (float32)FD_COS(thetar_x_pre));
        temp1 = (float32)FD_FABS(x_Start);
        if (((temp1 < 0.000001f) && (temp2 < 0.000001f)) ||
            (i > 99)) /*i > 59*/ {
            LengthEnd = true;
        }

        if (LengthEnd != true) {
            /*    d_Tgt */
            /*    d_Tgtprime */
            /*    d_Tgtprimeprime */
            TRJPLN_FT_CalcLatDistDerivs(
                &TgtDistY_prime, &TgtDistY_primeprime, &TgtDistY, Xi,
                CridrRiSg1Heading, Kr, Kr_prime, TgtTrajSg1Y, TgtTrajSg1Hdng,
                TgtTrajSg1Crv, TgtTrajSg1ChngOfCrv, x_Start, CridrRiSg1X,
                UseCridrRiX0, CridrRiSg1Y);

            /*   d_Tgt */
            TgtDistYPoint[i] = TgtDistY;
            /*   d_Tgtpunkt */
            TgtDistY1stDerivPoint[i] = TgtDistY_prime * Sti_dot;
            /*   d_Tgtpunktpunkt */
            TgtDistY2ndDerivPoint[i] =
                (a_traj * TgtDistY_prime) +
                (Sti_dot * Sti_dot * TgtDistY_primeprime);
            *NumTgtPoints = i + 1;
            *PlanHorizonVisRange = ti;
            temp = ti;
        }

        /*  End condition */
        if (Xi >= TRJPLN_FT_MinFloat(PlanAreaTgt, CridrRiSg1Length)) {
            LengthEnd = true;
        }

        if (LengthEnd == true) {
            if (i > 0) {
                TgtDistYPoint[i] = TgtDistYPoint[i - 1];
                TgtDistY1stDerivPoint[i] = 0;
                TgtDistY2ndDerivPoint[i] = 0;
                *NumTgtPoints = i + 1;
                *PlanHorizonVisRange = temp - DeltaT;
            }
        }

        /* End condition */
        if (((tmax - ti) >= 0.000001f) && (i < 100)) {
            i = i + 1;
        } else {
            PlanningEnd = true;
        }

        *PlanHorizonFT = tmpPlanHorizon + t_Start;
    }
}

/*****************************************************************************
  Functionname: TRJPLN_FT_CalcTargetCurve_ArcLength */ /*!

                        @brief: Calculate target lateral values

                        @description: Calculate target lateral distances and
                        derivatives in arc-length
                        based plan mode

                        @param[in]

                        @param[out]

                        @return
                        *****************************************************************************/
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
                                         const float32 CridrRiSg1Y) {
    float32 Xi, Xi_pre, Si, Si_pre, thetar_x_pre, DeltaS, temp1, temp2;
    uint8 i;
    boolean_T PlanningEnd;
    float32 TgtDistY_prime, TgtDistY_primeprime, TgtDistY;

    /* Initial start points and intermediate values */
    *NumTgtPoints = 0;
    *PlanHorizonVisRange = 0;
    *PlanHorizonFT = 0;
    PlanningEnd = false;

    /*  Dynamic step for arc length based planning */
    DeltaS = v_traj * 0.1f;  // delta_S = Bahngeschwindigkeit * delta_t
    DeltaS = TrajCalc_FT_MaxFloat(DeltaS, 0.5f);

    i = 0;
    Xi = x_Start;
    Si = s_Start;
    temp2 = (float32)FD_FABS(TRJPLN_FT_MinFloat(PlanAreaTgt, CridrRiSg1Length));
    while (PlanningEnd == false) {
        Xi_pre = Xi;
        Si_pre = Si;
        thetar_x_pre = TRJPLN_FT_Thetar(CridrRiSg1Heading, Kr, Kr_prime, Xi_pre,
                                        CridrRiSg1X, UseCridrRiX0);
        Si = s_Start + ((float32)i * DeltaS);

        TgtPoints[i] = Si;
        Xi = Xi_pre + ((Si - Si_pre) * (float32)FD_COS(thetar_x_pre));

        *PlanHorizonFT = Si;
        *PlanHorizonVisRange = Xi;
        temp1 = (float32)fABS(x_Start);
        if (((temp1 < 0.000001f) && (temp2 < 0.000001f))) {
            PlanningEnd = true;
        } else {
            /* \   TgtDistY */
            /* \   TgtDistY_prime */
            /* \   TgtDistY_primeprime */
            TRJPLN_FT_CalcLatDistDerivs(
                &TgtDistY_prime, &TgtDistY_primeprime, &TgtDistY, Xi,
                CridrRiSg1Heading, Kr, Kr_prime, TgtTrajSg1Y, TgtTrajSg1Hdng,
                TgtTrajSg1Crv, TgtTrajSg1ChngOfCrv, x_Start, CridrRiSg1X,
                UseCridrRiX0, CridrRiSg1Y);
            /* \   TgtDistY */
            TgtDistYPoint[i] = TgtDistY;
            /* \   TgtDistY Dot */
            TgtDistY1stDerivPoint[i] = TgtDistY_prime;
            /* \   TgtDistY Dot Dot */
            TgtDistY2ndDerivPoint[i] = TgtDistY_primeprime;
            *NumTgtPoints = i + 1;
        }
        /* \  End condition */
        if ((Xi >= TRJPLN_FT_MinFloat(PlanAreaTgt, CridrRiSg1Length)) ||
            (i >= 100)) {
            PlanningEnd = true;
        }
        i++;
    }
}

/*****************************************************************************
  Functionname: TRJPLN_FT_CalcLatDistDerivs */ /*!

                                @brief: calculate lateral distance related data

                                @description: calculate DistY, DistY 1st
                                derivative, DistY 2nd derivative values

                                @param[in]

                                @param[out]

                                @return
                                *****************************************************************************/
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
                                 const float32 CridrRiSg1Y) {
    float32 Thetar_Xi, Yr_Xi, Kr_Xi;
    float32 Yziel_Xi, ThetaZiel_Xi, Kziel_Xi;

    float32 deltaTheta_RefTgt, CosdeltaTheta_RefTgt, deltaY_RefTgt, temp_3;

    Thetar_Xi = TRJPLN_FT_Thetar(CridrRiSg1Heading, Kr, Kr_prime, Xi,
                                 CridrRiSg1X, UseCridrRiX0);
    Yr_Xi = TRJPLN_FT_Yr(CridrRiSg1Y, CridrRiSg1Heading, Kr, Kr_prime, Xi,
                         CridrRiSg1X, UseCridrRiX0);
    Kr_Xi = TRJPLN_FT_K(Kr, Kr_prime, Xi, CridrRiSg1X);

    Yziel_Xi = TRJPLN_FT_Y(TgtTrajSg1Y, TgtTrajSg1Hdng, TgtTrajSg1Crv,
                           TgtTrajSg1ChngOfCrv, Xi, X0);
    ThetaZiel_Xi = TRJPLN_FT_Theta(TgtTrajSg1Hdng, TgtTrajSg1Crv,
                                   TgtTrajSg1ChngOfCrv, Xi, X0);
    Kziel_Xi = TRJPLN_FT_K(TgtTrajSg1Crv, TgtTrajSg1ChngOfCrv, Xi, X0);

    deltaTheta_RefTgt = ThetaZiel_Xi - Thetar_Xi;
    CosdeltaTheta_RefTgt =
        TRJPLN_FT_AvoidZero((float32)FD_COS(deltaTheta_RefTgt));
    deltaY_RefTgt = Yziel_Xi - Yr_Xi;

    if ((Thetar_Xi * ThetaZiel_Xi) > 0) {
        *TgtDistYPoint = (deltaY_RefTgt * (float32)FD_COS(ThetaZiel_Xi)) /
                         CosdeltaTheta_RefTgt;
    } else {
        *TgtDistYPoint =
            (deltaY_RefTgt * (float32)FD_COS(ThetaZiel_Xi)) /
            TRJPLN_FT_AvoidZero((float32)FD_COS((float32)FD_FABS(ThetaZiel_Xi) +
                                                (float32)FD_FABS(Thetar_Xi)));
    }

    *dTgt_prime =
        (1 - (Kr_Xi * (*TgtDistYPoint))) * (float32)FD_TAN(deltaTheta_RefTgt);
    temp_3 = (1.0F - (Kr_Xi * (*TgtDistYPoint))) / CosdeltaTheta_RefTgt;
    *dTgt_primeprime =
        ((-((Kr_prime * (*TgtDistYPoint)) + (Kr_Xi * (*dTgt_prime)))) *
         (float32)FD_TAN(deltaTheta_RefTgt)) +
        ((temp_3 / CosdeltaTheta_RefTgt) * ((temp_3 * Kziel_Xi) - Kr_Xi));
}

/*****************************************************************************
  Functionname: TRJPLN_FT_CalcTrajLastCycle */ /*!

                                @brief: Transformation of trajectory values of
                                last cycle to Frenet

                                @description:
                                    calculate ego movement in frenet coordinate
                                system, based on
                                history movement
                                    data in last cycle

                                @param[in]

                                @param[out]

                                @return
                                *****************************************************************************/
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
                                 float32* a_traj_Tgt) {
    float32 CosDeltathetaPrev, SinDeltathetaPrev, TanDeltathetaPrev;
    float32 OneminusKrd, dist_prime, temp_1, v_traj_Tgt_2;
    CosDeltathetaPrev = TRJPLN_FT_AvoidZero((float32)FD_COS((HdngTrajPrev)));
    SinDeltathetaPrev = (float32)FD_SIN((HdngTrajPrev));
    TanDeltathetaPrev = (float32)FD_TAN((HdngTrajPrev));

    *FT_DistYTrajPrev = DistYTrajPrev;

    /* Temporary values: (1-Kr*d) */
    OneminusKrd = 1.0F - (Kr * DistYTrajPrev);

    OneminusKrd = TRJPLN_FT_AvoidZero(OneminusKrd);

    /* velocity of the reference line based on target value*/
    *v_traj_Tgt = (vx * CosDeltathetaPrev) / OneminusKrd;
    temp_1 = CosDeltathetaPrev / OneminusKrd;

    dist_prime = OneminusKrd * TanDeltathetaPrev;

    v_traj_Tgt_2 = (*v_traj_Tgt) * (*v_traj_Tgt);
    /* acceleration of the reference line based on target value*/
    *a_traj_Tgt =
        temp_1 *
        (ax - ((v_traj_Tgt_2 / CosDeltathetaPrev) *
               ((dist_prime * ((Ktraj / TRJPLN_FT_AvoidZero(temp_1)) - Kr)) -
                ((Kr_prime * DistYTrajPrev) + (Kr * dist_prime)))));

    if (PlanArcLength == true) {
        *FT_DistYTrajPrev_dot = (1 - (Kr * DistYTrajPrev)) * TanDeltathetaPrev;
        *FT_DistYTrajPrev_ddot =
            ((-((Kr_prime * DistYTrajPrev) + (Kr * (*FT_DistYTrajPrev_dot)))) *
             TanDeltathetaPrev) +
            (((1 - (Kr * DistYTrajPrev)) /
              (CosDeltathetaPrev * CosDeltathetaPrev)) *
             ((((1 - (Kr * DistYTrajPrev)) / CosDeltathetaPrev) * Ktraj) - Kr));
    } else {
        *FT_DistYTrajPrev_dot = vx * SinDeltathetaPrev;
        *FT_DistYTrajPrev_ddot =
            (ax * SinDeltathetaPrev) +
            ((vx * ((Ktraj * vx) - (Kr * (*v_traj_Tgt)))) * CosDeltathetaPrev);
    }
}

/*****************************************************************************
  Functionname: TRJPLN_FT_CalcLeftCorridor */ /*!

                                 @brief: calculate the left corridor parameters
                                 at right corridor based frenet
                                 coordinate system

                                 @description:
                                       calculate the left corridor parameters at
                                 right corridor based
                                 frenet coordinate system

                                 @param[in]

                                 @param[out]

                                 @return
                                 *****************************************************************************/
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
                                const float32 CridrLeSg1X) {
    uint8 ind_Left;
    boolean_T CalcEnd;
    float32 SiLeft, XiLeft, SiLeft_pre, XiLeft_pre, Thetar_XiLeft_pre,
        Yr_XiLeft, Thetar_XiLeft, Yl_XiLeft, Thetal_XiLeft;
    float32 deltaTheta_Refl, CosdeltaTheta_Refl, deltaY_Refl;
    boolean_T StartPointFound;
    float32 t_LeftDeadTime, s_LeftDeadTime, x_LeftDeadTime,
        Thetar_x_LeftDeadTime, s_LeftDeadTime_pre, x_LeftDeadTime_pre,
        Thetar_x_LeftDeadTime_pre;
    float32 delta_t, delta_sec, StepsLeft;
    float32 minRunLength;

    /* Initialize intermediate values */
    CalcEnd = false;
    StartPointFound = false;
    delta_t = 0.05f; /* in Seconds */
    minRunLength = TRJPLN_FT_MinFloat(CridrLeSg1Length, CridrRiSg1Length);

    /*
     * Calculation start point due to dead time
     */
    s_LeftDeadTime = 0.0f;
    x_LeftDeadTime = 0.0f;
    t_LeftDeadTime = 0.0f;
    Thetar_x_LeftDeadTime =
        TRJPLN_FT_Thetar(CridrRiSg1Heading, CridrRiSg1Crv, CridrRiSg1ChgOfCrv,
                         0.0f, CridrRiSg1X, UseCridrRiX0);
    while (StartPointFound != true) {
        s_LeftDeadTime_pre = s_LeftDeadTime;
        s_LeftDeadTime = (t_LeftDeadTime * v_Traj) +
                         ((0.5f * a_Traj) * (t_LeftDeadTime * t_LeftDeadTime));
        x_LeftDeadTime_pre = x_LeftDeadTime;
        Thetar_x_LeftDeadTime_pre = Thetar_x_LeftDeadTime;
        x_LeftDeadTime =
            x_LeftDeadTime_pre + ((s_LeftDeadTime - s_LeftDeadTime_pre) *
                                  (float32)FD_COS(Thetar_x_LeftDeadTime_pre));
        Thetar_x_LeftDeadTime = TRJPLN_FT_Thetar(
            CridrRiSg1Heading, CridrRiSg1Crv, CridrRiSg1ChgOfCrv,
            x_LeftDeadTime, CridrRiSg1X, UseCridrRiX0);
        if (t_LeftDeadTime >= DeadTime) {
            StartPointFound = true;
        } else {
            t_LeftDeadTime = t_LeftDeadTime + delta_t;
        }
    }

    /*
     *  Calculate left corridor
     */
    ind_Left = 1;
    delta_sec = 0.7f; /* in Meters */
    SiLeft = s_LeftDeadTime;
    XiLeft = x_LeftDeadTime;
    Thetar_XiLeft = Thetar_x_LeftDeadTime;
    while (CalcEnd != true) {
        SiLeft_pre = SiLeft;
        if (PlanArcLength == true) {
            SiLeft = SiLeft + delta_sec;
        } else {
            float32 min_t;
            StepsLeft = (float32)ind_Left * delta_t;
            min_t = TRJPLN_FT_MinFloat(
                StepsLeft, 1.0f - t_LeftDeadTime); /* accelerate within 1s */
            SiLeft = s_LeftDeadTime + (StepsLeft * v_Traj) +
                     ((0.5f * a_Traj) * (min_t * min_t));
        }
        XiLeft_pre = XiLeft;
        Thetar_XiLeft_pre = Thetar_XiLeft;
        XiLeft = XiLeft_pre +
                 ((SiLeft - SiLeft_pre) * (float32)FD_COS(Thetar_XiLeft_pre));
        Yr_XiLeft =
            TRJPLN_FT_Yr(CridrRiSg1Y, CridrRiSg1Heading, CridrRiSg1Crv,
                         CridrRiSg1ChgOfCrv, XiLeft, CridrRiSg1X, UseCridrRiX0);
        Thetar_XiLeft = TRJPLN_FT_Thetar(CridrRiSg1Heading, CridrRiSg1Crv,
                                         CridrRiSg1ChgOfCrv, XiLeft,
                                         CridrRiSg1X, UseCridrRiX0);

        if (XiLeft >= minRunLength) {
            CalcEnd = 1; /* End loop */
            Outputs->uiNumPointsCridrLeft_nu = ind_Left;
        }
        /*  \  Yl, Thetal*/
        Yl_XiLeft = TRJPLN_FT_Y(CridrLeSg1Y, CridrLeSg1Hdng, CridrLeSg1Crv,
                                CridrLeSg1ChgOfCrv, XiLeft, CridrLeSg1X);
        Thetal_XiLeft =
            TRJPLN_FT_Theta(CridrLeSg1Hdng, CridrLeSg1Crv, CridrLeSg1ChgOfCrv,
                            XiLeft, CridrLeSg1X);

        /*  \  deltaTheta_Refl, deltaY_Refl */
        deltaTheta_Refl = Thetal_XiLeft - Thetar_XiLeft;
        CosdeltaTheta_Refl =
            TRJPLN_FT_AvoidZero((float32)FD_COS(deltaTheta_Refl));
        deltaY_Refl = Yl_XiLeft - Yr_XiLeft;
        if ((Thetar_XiLeft * Thetal_XiLeft) > 0) {
            Outputs->afDistYLeft_met[ind_Left - 1] =
                (deltaY_Refl * (float32)FD_COS(Thetal_XiLeft)) /
                CosdeltaTheta_Refl;
        } else {
            Outputs->afDistYLeft_met[ind_Left - 1] =
                (deltaY_Refl * (float32)FD_COS(Thetal_XiLeft)) /
                TRJPLN_FT_AvoidZero(
                    (float32)FD_COS((float32)FD_FABS(Thetal_XiLeft) +
                                    (float32)FD_FABS(Thetar_XiLeft)));
        }
        if (ind_Left < 100) {
            ind_Left = ind_Left + 1;
        } else {
            CalcEnd = true; /* End loop. Max. 100 points */
            Outputs->uiNumPointsCridrLeft_nu = ind_Left;
        }
    }
}

/*****************************************************************************
  Functionname: TRJPLN_calFrenetTransformation */ /*!

                             @brief: frenet transformation core function

                             @description: we transform normally cartesian
                             coordinate system to right
                             corridor based
                             frenet coordinate system, which can help us ignore
                             the influence of
                             curve factor while
                             trajectory planning.

                             @param[in]
                             const TRJPLN_calFTInTypeV3_t* Inputs: the input of
                             frenet
                             transformation module
                             @param[out]


                             @return
                             TRJPLN_calFTOutType_t: the output of the frenet
                             transformation module
                             @uml
                             @startuml
                             start
                             :TRJPLN_FT_InitializeOutputs;
                             note:initialize all output
                             if (trajectory plan is enabled) then (yes)
                             :TRJPLN_FT_TransformCurValues;
                             note:Transformation of current values \nto Frenet
                             :TRJPLN_FT_CalcPreviewCurValues;
                             note:Transformation of current value \npreview to
                             Frenet
                             :TRJPLN_FT_CalcTrajLastCycle;
                             note:Transformation of trajectory values \nof last
                             cycle to Frenet
                             if (replan current values is not triggered) then
                             (yes)
                             :ego velocity/acceleration is replaced by history
                             value;
                             endif
                             if (replan is triggered or target trajectory
                             calculation is triggered) then
                             (yes)
                             if(target corridor calculation is enabled) then
                             (yes)
                             :TRJPLN_FT_CalcTargetCurvePoints;
                             note:Transformation of target curve to Frenet
                             endif
                             if(replan is triggered) then (yes)
                             :TRJPLN_FT_CalcRightCorridor;
                             note:Calculation of curvature of right corridor
                             endif
                             if(replan is enabled and left lane calculation is
                             enabled) then (yes)
                             :TRJPLN_FT_CalcLeftCorridor;
                             note:Transformation of left corridor boundary
                             endif
                             endif
                             endif
                             end
                             @enduml
                             *****************************************************************************/
TRJPLN_calFTOutType_t TRJPLN_calFrenetTransformation(
    const TRJPLN_calFTInTypeV3_t* Inputs) {
    TRJPLN_calFTOutType_t Outputs;

    /* Initialize all output parameters */
    TRJPLN_FT_InitializeOutputs(&Outputs);

    if (Inputs->uiTrajPlanEnbl_nu == true) {
        boolean_T PlanArcLength;
        float32 DevY, DevTheta, Vx, Ax, Kr, Kr_prime, Kist, Kstart, Ksoll;

        float32 dist, dist1stDeriv, dist2ndDeriv, v_traj, a_traj;
        float32 DistYTrajPrev, DistYTrajPrev_dot, DistYTrajPrev_ddot;
        float32 FirstDerivDeltaHeading, SecondDerivDeltaHeading;
        float32 v_traj_Tgt, a_traj_Tgt;

        /* Initialize and intermediate values */
        a_traj = 0;
        v_traj = 0;
        dist = 0;
        dist1stDeriv = 0;
        dist2ndDeriv = 0;
        DistYTrajPrev = 0;
        DistYTrajPrev_dot = 0;
        DistYTrajPrev_ddot = 0;
        FirstDerivDeltaHeading = 0;
        SecondDerivDeltaHeading = 0;
        v_traj_Tgt = 0;
        a_traj_Tgt = 0;

        PlanArcLength = Inputs->uiReplanModeArcLength_nu;
        DevY = Inputs->fDevDistY_met;
        DevTheta = Inputs->fDevHeading_rad;
        Vx = Inputs->fEgoVelX_mps;
        Ax = Inputs->fEgoAclX_mps2;
        Kr = Inputs->fCridrRightSg1_Crv_1pm;
        Kr_prime = Inputs->fCridrRightSg1_ChOfCrv_1pm2;
        Kist = Inputs->fCurCrvVehMotion_1pm;
        Kstart = Inputs->fStartCrv_1pm;
        Ksoll = Inputs->fTgtCrvTrajPrev_1pm;

        /* \  Transformation of current values to Frenet */
        TRJPLN_FT_TransformCurValues(
            PlanArcLength, DevY, DevTheta, Vx, Ax, Kr, Kr_prime, Kist, &dist,
            &dist1stDeriv, &dist2ndDeriv, &v_traj, &a_traj,
            Inputs->fCurHdngAndDeltaHdng_rad, &FirstDerivDeltaHeading,
            &SecondDerivDeltaHeading, Kstart, Ksoll);

        /* \  Transformation of current value preview to Frenet */
        TRJPLN_FT_CalcPreviewCurValues(
            Inputs->fPreviewTimeHeading_sec, 0.0, Kr, Kr_prime, DevY, DevTheta,
            Kist, v_traj, a_traj, &Outputs, Inputs->fCridrRightSg1_DistX_met,
            Inputs->uiUseCridrRightX0_nu, Inputs->fCridrRightSg1_DistY_met);

        Outputs.fDstYCurValuePreview_met = dist;
        Outputs.fDstY1stDrvCurValPrvw_mps = dist1stDeriv;

        // TRJPLN_FT_CalcPreviewCurValues(
        //     0.0, 0.0, Kr, Kr_prime, DevY, DevTheta, Kist, v_traj, a_traj,
        //     &Outputs, Inputs->fCridrRightSg1_DistX_met,
        //     Inputs->uiUseCridrRightX0_nu, Inputs->fCridrRightSg1_DistY_met);

        /* \  Transformation of trajectory values of last cycle to Frenet */
        TRJPLN_FT_CalcTrajLastCycle(
            &DistYTrajPrev, &DistYTrajPrev_dot, &DistYTrajPrev_ddot,
            Inputs->fDistYTrajPrev_met, Inputs->fHeadingTrajPrev_rad,
            PlanArcLength, Kr, Kr_prime, Inputs->fTgtCrvTrajPrev_1pm, Vx, Ax,
            &v_traj_Tgt, &a_traj_Tgt);

        Outputs.fDistYCurValue_met = dist;
        Outputs.fDistY1stDerivCurValue_nu = dist1stDeriv;
        Outputs.fDistY2ndDerivCurValue_nu = dist2ndDeriv;

        /* Select the velocity and acceleration of the reference line*/
        if (Inputs->uiReplanCurValues_nu != true) {
            v_traj = v_traj_Tgt;
            a_traj = a_traj_Tgt;
        }

        Outputs.fTrajVel_mps = v_traj;
        Outputs.fTrajAcl_mps2 = a_traj;

        Outputs.fTrajDistYPrevious_met = DistYTrajPrev;
        Outputs.fTrajDistYPrev1stDeriv_nu = DistYTrajPrev_dot;
        Outputs.fTrajDistYPrev2ndDeriv_nu = DistYTrajPrev_ddot;

        Outputs.f1stDerivDevHeading_nu = FirstDerivDeltaHeading;
        Outputs.f2ndDerivDevHeading_nu = SecondDerivDeltaHeading;

        if ((Inputs->uiTrigRecalc_nu == true) ||
            (Inputs->uiTrigTgtTrajCalc_nu == true)) {
            float32 TgtSg1X, TgtSg1Length, CridrRiSg1Heading, PlanHorizon,
                CridrRiSg1Length, TgtTrajSg1Y, TgtTrajSg1Hdng, TgtTrajSg1Crv,
                TgtTrajSg1ChngOfCrv, CridrRiSg1X, CridrLeSg1X, CridrRiSg1Y;
            float32 CridrLeSg1Length, CridrLeSg1Y, CridrLeSg1Hdng,
                CridrLeSg1Crv, CridrLeSg1ChgOfCrv;
            TgtSg1X = Inputs->fTgtTrajSg1_DistX_met;
            TgtSg1Length = Inputs->fTgtTrajSg1_Length_met;
            TgtTrajSg1ChngOfCrv = Inputs->fTgtTrajSg1_ChngOfCrv_1pm2;
            TgtTrajSg1Y = Inputs->fTgtTrajSg1_DistY_met;
            TgtTrajSg1Hdng = Inputs->fTgtTrajSg1_Heading_rad;
            TgtTrajSg1Crv = Inputs->fTgtTrajSg1_Crv_1pm;
            CridrRiSg1X = Inputs->fCridrRightSg1_DistX_met;
            CridrRiSg1Y = Inputs->fCridrRightSg1_DistY_met;
            CridrRiSg1Heading = Inputs->fCridrRightSg1_Heading_rad;
            CridrRiSg1Length = Inputs->fCridrRightSg1_Length_met;
            PlanHorizon = Inputs->fPlanningHorizon_sec;
            CridrLeSg1Length = Inputs->fCridrLeftSg1_Length_met;
            CridrLeSg1X = Inputs->fCridrLeftSg1_DistX_met;
            CridrLeSg1Y = Inputs->fCridrLeftSg1_DistY_met;
            CridrLeSg1Hdng = Inputs->fCridrLeftSg1_Heading_rad;
            CridrLeSg1Crv = Inputs->fCridrLeftSg1_Crv_1pm;
            CridrLeSg1ChgOfCrv = Inputs->fCridrLeftSg1_ChOfCrv_1pm2;
            if ((Inputs->uiTrigTgtTrajCalc_nu == true) &&
                (Inputs->fTgtTrajSg1_Length_met >= 0) &&
                (Inputs->fCridrRightSg1_Length_met >
                 0))  // JIRA: FASSIEBEN-7347 Feher bei der Ausgabe der
                      // Zielwerte, wenn die Laufl�nge negativ ist
            {
                /* \   Transformation of target curve to Frenet */
                TRJPLN_FT_CalcTargetCurvePoints(
                    PlanArcLength, TgtSg1X, TgtSg1Length, CridrRiSg1Heading, Kr,
                    Kr_prime, PlanHorizon, CridrRiSg1Length, TgtTrajSg1Y,
                    TgtTrajSg1Hdng, TgtTrajSg1Crv, TgtTrajSg1ChngOfCrv, v_traj,
                    a_traj, &Outputs, Inputs->uiUseTgtTrajX0_nu, CridrRiSg1X,
                    Inputs->uiUseCridrRightX0_nu, CridrRiSg1Y);
            }

            /* \ Calculation of curvature of right corridor */
            if (Inputs->uiTrigRecalc_nu == true) {
                TRJPLN_FT_CalcRightCorridor(
                    PlanArcLength, v_traj, a_traj, Kr, Kr_prime,
                    CridrRiSg1Length, &Outputs, CridrRiSg1X,
                    Inputs->uiUseCridrRightX0_nu, CridrRiSg1Heading);
            }

            /* \  */
            if ((Inputs->uiTrigRecalc_nu == true) &&
                (Inputs->uiLeftCridrActive_nu == true)) {
                /* \  Transformation of left corridor boundary */
                TRJPLN_FT_CalcLeftCorridor(
                    &Outputs, Kr, Kr_prime, CridrLeSg1Length, CridrRiSg1Length,
                    CridrLeSg1Y, CridrLeSg1Hdng, CridrLeSg1Crv,
                    CridrLeSg1ChgOfCrv, v_traj, a_traj, PlanArcLength,
                    Inputs->fDeadTime_sec, CridrRiSg1X,
                    Inputs->uiUseCridrRightX0_nu, CridrRiSg1Heading,
                    CridrRiSg1Y, CridrLeSg1X);
            }
        } /* --> End of If */
    }

    return Outputs;
}

/*****************************************************************************
  Functionname: TRJPLN_FT_Theta                                           */ /*!

        @brief: calculate the heading angel at distance x

        @description: calculate theheading agnel in new point x, based on
      current
      curvature and
                              curvature change
                              HeadingAngle = ARCTAN( (TAN(CurrentHeadingAngle) +
      curvature * DeltaDistX) +
                              (0.5 * CurvatureChange * (DeltaDistX)^2))

        @param[in]

        @param[out]

        @return
      *****************************************************************************/
float32 TRJPLN_FT_Theta(const float32 theta0,
                        const float32 K0,
                        const float32 K_prime,
                        const float32 x,
                        const float32 x0) {
    float32 result;
    float32 deltaX;

    deltaX = (x - x0);
    result = (float32)(FD_ATAN(((float32)FD_TAN(theta0) + (K0 * deltaX)) +
                               ((0.5F * K_prime) * (deltaX * deltaX))));
    return result;
}

/*****************************************************************************
  Functionname: TRJPLN_FT_Y                                           */ /*!

                    @brief: calculate the distance Y at distance x

                    @description: calculate the distance Y in new point x, based
                  on
                  current
                  curvature and
                                          curvature change
                                          DistY = CurrentDistY +
                  (TAN(HeadingAngle) *
                  DeltaDistX) + 0.5 * curvature * (DeltaDistX)^2 +
                                          1/6 * CurvatureChange * (DeltaDistX)^3

                    @param[in]

                    @param[out]

                    @return
                  *****************************************************************************/
float32 TRJPLN_FT_Y(const float32 Y0,
                    const float32 theta0,
                    const float32 K0,
                    const float32 K_prime,
                    const float32 x,
                    const float32 x0) {
    float32 result;
    float32 deltaX;
    float32 deltaXpow2;

    deltaX = (x - x0);
    deltaXpow2 = deltaX * deltaX;
    result = ((Y0 + ((float32)FD_TAN(theta0) * (deltaX))) +
              ((0.5F * K0) * deltaXpow2)) +
             (((float32)(1 / 6) * K_prime) * (deltaXpow2 * deltaX));
    return result;
}

/*****************************************************************************
  Functionname: TRJPLN_FT_K                                           */ /*!

                    @brief: calculate the curvature at distance x

                    @description: calculate the curvature in new point x, based
                  on
                  current
                  curvature and
                                          curvature change
                                          NewCurvature = CurrentCurvature +
                  (CurvatureChange
                  * DeltaDistX)

                    @param[in]

                    @param[out]

                    @return
                  *****************************************************************************/
float32 TRJPLN_FT_K(const float32 K0,
                    const float32 K_prime,
                    const float32 x,
                    const float32 x0) {
    float32 result;
    result = K0 + (K_prime * (x - x0));
    return result;
}

/*****************************************************************************
  Functionname: TRJPLN_FT_AvoidZero                                           */ /*!

@brief: return a minimum number if the input is almost zero

@description: a protection that we often used in division operation

@param[in]
                  const float32 input: the value we need to check whether is
almost zero
@param[out]


@return
                  would return a minimum value if the input is almost zero,
or return the input value directly
*****************************************************************************/
float32 TRJPLN_FT_AvoidZero(const float32 input) {
    float32 result;
    if ((float32)FD_FABS(input) < 0.000001F) {
        if (input >= 0) {
            result = 0.000001F;
        } else {
            result = -0.000001F;
        }
    } else {
        result = input;
    }
    return result;
}

/*****************************************************************************
  Functionname: TRJPLN_FT_MinFloat                                           */ /*!

@brief:
                     return minimum value of input two values

@description:
                     return minimum value of input two values

@param[in]

@param[out]


@return
*****************************************************************************/
float32 TRJPLN_FT_MinFloat(const float32 A, const float32 B) {
    float32 C;
    if (A >= B) {
        C = B;
    } else {
        C = A;
    }
    return C;
}

/*****************************************************************************
  Functionname: TrajCalc_FT_MaxFloat */ /*!

                                       @brief:
                                                         return maximum value of
                                       input two values

                                       @description:
                                                         return maximum value of
                                       input two values

                                       @param[in]

                                       @param[out]


                                       @return
                                       *****************************************************************************/
float32 TrajCalc_FT_MaxFloat(const float32 A, const float32 B) {
    float32 C;
    if (A >= B) {
        C = A;
    } else {
        C = B;
    }
    return C;
}

float32 TRJPLN_FT_Thetar(const float32 theta0,
                         const float32 K0,
                         const float32 K_prime,
                         const float32 x,
                         const float32 x0,
                         const boolean_T UseCridrRiX0) {
    float32 result, deltaX;

    if (UseCridrRiX0 == true) {
        deltaX = x - x0;
        result = ((float32)FD_TAN(theta0) + (K0 * deltaX)) +
                 ((0.5F * K_prime) * (deltaX * deltaX));
    } else {
        deltaX = x;
        result = (K0 * deltaX) + ((0.5F * K_prime) * (deltaX * deltaX));
    }
    result = (float32)FD_ATAN(result);
    return result;
}

float32 TRJPLN_FT_Yr(const float32 Y0,
                     const float32 theta0,
                     const float32 K0,
                     const float32 K_prime,
                     const float32 x,
                     const float32 x0,
                     const boolean_T UseCridrRiX0) {
    float32 result;
    float32 deltaX;
    float32 deltaXpow2;

    if (UseCridrRiX0 == true) {
        deltaX = x - x0;
        deltaXpow2 = deltaX * deltaX;
        result = ((Y0 + ((float32)FD_TAN(theta0) * (deltaX))) +
                  ((0.5F * K0) * deltaXpow2)) +
                 (((float32)(1 / 6) * K_prime) * (deltaXpow2 * deltaX));
    } else {
        deltaX = x;
        deltaXpow2 = deltaX * deltaX;
        result = ((0.5F * K0) * deltaXpow2) +
                 (((float32)(1 / 6) * K_prime) * (deltaXpow2 * deltaX));
    }
    return result;
}

void TRJPLN_FT_CalcPreviewCurValues(const float32 PreviewTimeKF,
                                    const float32 CridrRiSg1Heading,
                                    const float32 Kr,
                                    const float32 Kr_prime,
                                    const float32 CurY,
                                    const float32 CurHeading,
                                    const float32 CurCrv,
                                    const float32 v_traj,
                                    const float32 a_traj,
                                    TRJPLN_calFTOutType_t* Outputs,
                                    const float32 CridrRiSg1X,
                                    const boolean_T UseCridrRiX0,
                                    const float32 CridrRiSg1Y) {
    float32 Xi, Xi_pre, Si, Si_pre, thetar_x_pre, DeltaT, Sti_dot, ti, tmax,
        PreviewTime, x_Start;
    uint8 i;
    boolean_T PreviewEnd;
    float32 CurYPreview, CurYPreview_prime, CurYPreview_primeprime,
        MaxPreviewTime, CurCrvChg;
    float32 minBft_t1;

    PreviewEnd = false;

    DeltaT = 0.1f;
    i = 0;
    Xi = 0.0f;
    Si = 0.0f;
    MaxPreviewTime = 2.0f;
    CurCrvChg = 0.0f;
    x_Start = 0.0f;
    Sti_dot = 0.0f;
    PreviewTime =
        (float32)FD_CEIL((float32)(PreviewTimeKF / (DeltaT))) * (DeltaT);
    tmax = TRJPLN_FT_MinFloat(PreviewTime, MaxPreviewTime);

    while (PreviewEnd == false) {
        Xi_pre = Xi;
        Si_pre = Si;
        thetar_x_pre = TRJPLN_FT_Thetar(CridrRiSg1Heading, Kr, Kr_prime, Xi_pre,
                                        CridrRiSg1X, UseCridrRiX0);
        ti = DeltaT * (float32)i;

        minBft_t1 = TRJPLN_FT_MinFloat(1, ti);
        Sti_dot = v_traj + (a_traj * minBft_t1);
        Si = (v_traj * ti) + ((0.5f * a_traj) * (minBft_t1 * minBft_t1));
        Xi = Xi_pre + ((Si - Si_pre) * (float32)FD_COS(thetar_x_pre));

        if ((tmax - ti) >= 0.000001f) {
            i = i + 1;
        } else {
            PreviewEnd = true;
        }
    }
    TRJPLN_FT_CalcLatDistDerivs(
        &CurYPreview_prime, &CurYPreview_primeprime, &CurYPreview, Xi,
        CridrRiSg1Heading, Kr, Kr_prime, CurY, CurHeading, CurCrv, CurCrvChg,
        x_Start, CridrRiSg1X, UseCridrRiX0, CridrRiSg1Y);

    /* Set outputs */
    Outputs->fDstYCurValuePreview_met = CurYPreview;
    Outputs->fDstY1stDrvCurValPrvw_mps = CurYPreview_prime * Sti_dot;
    Outputs->fFrenTrafHeadingPrvw_sec = tmax;
}

void TRJPLN_FT_CalcRightCorridor(const boolean_T PlanArcLength,
                                 const float32 v_Traj,
                                 const float32 a_Traj,
                                 const float32 CridrRiSg1Crv,
                                 const float32 CridrRiSg1ChgOfCrv,
                                 const float32 CridrRiSg1Length,
                                 TRJPLN_calFTOutType_t* Outputs,
                                 const float32 CridrRiSg1X,
                                 const boolean_T UseCridrRiX0,
                                 const float32 CridrRiSg1Heading) {
    float32 delta_sec, delta_t;
    float32 SiRight, XiRight, ThetaRight, SiRight_pre, XiRight_pre;
    boolean_T CalcEnd;
    float32 Time, KRight;
    uint8 Index, Index_Output, MaxIter;

    /* Initialize Variables */
    MaxIter = 75;
    delta_sec = 0.7f; /* in Meters */
    delta_t = 0.05f;  /* in Seconds */
    CalcEnd = false;
    SiRight = 0.0f;
    XiRight = 0.0f;
    ThetaRight = 0.0f;
    Index = 0;
    Index_Output = 0;
    Time = 0.0f;
    KRight = 0.0f;
    while (CalcEnd != true) {
        SiRight_pre = SiRight;

        if (PlanArcLength == true) /* arc length based */ {
            SiRight = (float32)(Index + 1) * delta_sec;
        } else /* time based */ {
            float32 min_t;
            Time = (float32)(Index + 1) * delta_t;
            min_t = TRJPLN_FT_MinFloat(Time, 1.0f);
            SiRight = (v_Traj * Time) + ((0.5f * a_Traj) * (min_t * min_t));
        }
        XiRight_pre = XiRight;
        float32 ThetaRight_pre = ThetaRight;
        XiRight = XiRight_pre +
                  ((SiRight - SiRight_pre) * (float32)FD_COS(ThetaRight_pre));
        ThetaRight = TRJPLN_FT_Thetar(CridrRiSg1Heading, CridrRiSg1Crv,
                                      CridrRiSg1ChgOfCrv, XiRight, CridrRiSg1X,
                                      UseCridrRiX0);
        KRight = TRJPLN_FT_K(CridrRiSg1Crv, CridrRiSg1ChgOfCrv, XiRight,
                             CridrRiSg1X);

        if ((((Index + 1) % 5) == 0) && (Index_Output < 15)) {
            Outputs->afCridrRight_Crv_1pm[Index_Output] = KRight;
            Index_Output = Index_Output + 1;
            Outputs->uiNumPointsCridrRight_nu = Index_Output;
        }

        if ((XiRight >= CridrRiSg1Length) || (Index >= (MaxIter - 1))) {
            CalcEnd = true;
        } else {
            Index = Index + 1;
        }
    }
}

#define STOP_SEC_CODE
/*-------------------------------------------------------------------------------------------*
  END OF FILE
 *-------------------------------------------------------------------------------------------*/
