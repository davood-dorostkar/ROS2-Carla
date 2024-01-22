/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
/*********************************************************************

PROJECT:        TJA Base Development

MODULE:         Trajectory Planning

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
        M I S R A
/* -----------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------

        I N C L U D E S

 * -----------------------------------------------------------------------------
*/

#include "trjpln_calFrenetBackTransformation.h"

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
  Functionname: TRJPLN_calFrenetBackTransformation */ /*!

                         @brief: frenet transform back module

                         @description: transform planned trajectory from right
                         corridor based frenet
                         coordinate system
                         to cartesian coordinate system. we will output current
                         control related
                         data based on
                         planned fifth order polynomial.

                         @param[in]
                         const TRJPLN_calFBTInType_t* Inputs: the input of
                         frenet back transform
                         module
                         @param[out]


                         @return TRJPLN_calFBTOutType_t: the output of frenet
                         back transform module
                         @uml
                         @startuml
                         start
                         if(trajectory is enabled and planned trajectory exists)
                         then (yes)
                         if (trajectory plan mode is ArcLength based) then (yes)
                         :calculate lateral distance relate value;
                         else (no, time based mode)
                         :calculate lateral distance relate value;
                         endif
                         :calculate other output value;
                         else (no)
                         :output default zero value;
                         endif
                         end
                         @enduml
                         *****************************************************************************/
TRJPLN_calFBTOutType_t TRJPLN_calFrenetBackTransformation(
    const TRJPLN_calFBTInType_t* Inputs) {
    TRJPLN_calFBTOutType_t Outputs;
    float32 v_traj, a_traj;
    float32 one_minus_kappa_d;
    float32 d_prime, d_prime_TrajPrevHdg;
    float32 d_primeprime, d_primeprime_TrajPrevKappaAndDeadTime,
        d_primeprime_TrajPrevKappa;
    float32 d_primeprimeprime, d_primeprimeprime_TrajPrevKappaAndDeadTime;

    v_traj = TRJPLN_FBT_AvoidZero(Inputs->fTPLFBT_TrajVelRefCurve_mps);
    a_traj = Inputs->fTPLFBT_TrajAclRefCurve_mps2;

    if ((Inputs->uiTPLFBT_TrajGuiEnable_nu == true) &&
        (Inputs->uiNoTrajFound_nu == false)) {
        Outputs.fTrajDistY_met = Inputs->fTPLFBT_TrajDistY_met;

        one_minus_kappa_d =
            1.0F - (Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm *
                    Inputs->fTPLFBT_TrajDistY_met);  // 1-RightCurvature * d
        one_minus_kappa_d = TRJPLN_FBT_AvoidZero(one_minus_kappa_d);

        if (Inputs->uiReplanModeArcLength_nu == true) {
            d_prime = Inputs->fTPLFBT_TrajDistY1stDeriv_mps;
            d_prime_TrajPrevHdg = Inputs->fYDotTrajFromHeadingPreview_mps;

            d_primeprime = Inputs->fTPLFBT_TrajDistY2ndDeriv_nu;
            d_primeprime_TrajPrevKappaAndDeadTime =
                Inputs->fYDot2TrajFromKappaPrevAndDT_mps2;
            d_primeprime_TrajPrevKappa =
                Inputs->fYDotDotTrajFromKappaPreview_mps2;

            Outputs.fHeadingCurValuePreview_rad = Inputs->fDevHeading_rad;
            d_primeprimeprime_TrajPrevKappaAndDeadTime =
                Inputs->fYDot3TrajFromKappaPrevAndDT_nu;
            d_primeprimeprime = Inputs->fTPLFBT_TrajDistY3rdDeriv_nu;
        } else {
            float32 d_prime_CurValPreview;
            float32 v_traj_3;
            v_traj_3 = v_traj * v_traj * v_traj;

            d_prime = Inputs->fTPLFBT_TrajDistY1stDeriv_mps / v_traj;
            d_prime_CurValPreview =
                Inputs->fDistY1stDerivCurValuePreview_mps / v_traj;
            d_prime_TrajPrevHdg =
                Inputs->fYDotTrajFromHeadingPreview_mps / v_traj;

            d_primeprime =
                (Inputs->fTPLFBT_TrajDistY2ndDeriv_nu - (a_traj * d_prime)) /
                (v_traj * v_traj);
            d_primeprime_TrajPrevKappaAndDeadTime =
                (Inputs->fYDot2TrajFromKappaPrevAndDT_mps2 -
                 (a_traj * d_prime)) /
                (v_traj * v_traj);
            d_primeprime_TrajPrevKappa =
                (Inputs->fYDotDotTrajFromKappaPreview_mps2 -
                 (a_traj * d_prime)) /
                (v_traj * v_traj);
            d_primeprimeprime_TrajPrevKappaAndDeadTime =
                (Inputs->fYDot3TrajFromKappaPrevAndDT_nu -
                 (((3 * a_traj) * v_traj) *
                  d_primeprime_TrajPrevKappaAndDeadTime)) /
                v_traj_3;
            d_primeprimeprime = (Inputs->fTPLFBT_TrajDistY3rdDeriv_nu -
                                 (((3 * a_traj) * v_traj) * d_primeprime)) /
                                v_traj_3;

            Outputs.fHeadingCurValuePreview_rad =
                (float32)FD_ATAN(d_prime_CurValPreview / one_minus_kappa_d);
        }
        Outputs.fTrajHeading_rad =
            (float32)FD_ATAN(d_prime / one_minus_kappa_d);
        Outputs.fTrajHeadingInclPreview_rad =
            (float32)FD_ATAN(d_prime_TrajPrevHdg / one_minus_kappa_d);
        Outputs.fTrajTgtCrv_1pm =
            (((d_primeprime +
               (((Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm * d_prime) +
                 (Inputs->fTPLFBT_CridrRightSeg1_ChngOfCrv_1pm2 *
                  Inputs->fTPLFBT_TrajDistY_met)) *
                (float32)FD_TAN(Outputs.fTrajHeading_rad))) *
              (float32)FD_THIRDPOWER(FD_COS(Outputs.fTrajHeading_rad))) /
             (one_minus_kappa_d * one_minus_kappa_d)) +
            (((float32)FD_COS(Outputs.fTrajHeading_rad) / one_minus_kappa_d) *
             Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm);  // is being avoided
                                                       // through line 77
                                                       // through
                                                       // one_minus_kappa_d
        Outputs.fDistYCurValuePreview_met = Inputs->fDevDistY_met;

        Outputs.fTgtCrvInclPreviewAndDeadTime_1pm =
            (((d_primeprime_TrajPrevKappaAndDeadTime +
               (((Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm * d_prime) +
                 (Inputs->fTPLFBT_CridrRightSeg1_ChngOfCrv_1pm2 *
                  Inputs->fTPLFBT_TrajDistY_met)) *
                (float32)FD_TAN(Outputs.fTrajHeading_rad))) *
              (float32)FD_THIRDPOWER(FD_COS(Outputs.fTrajHeading_rad))) /
             (one_minus_kappa_d * one_minus_kappa_d)) +
            (((float32)FD_COS(Outputs.fTrajHeading_rad) / one_minus_kappa_d) *
             Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm);

        Outputs.fTgtCrvTrajInclPreview_1pm =
            (((d_primeprime_TrajPrevKappa +
               (((Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm * d_prime) +
                 (Inputs->fTPLFBT_CridrRightSeg1_ChngOfCrv_1pm2 *
                  Inputs->fTPLFBT_TrajDistY_met)) *
                (float32)FD_TAN(Outputs.fTrajHeading_rad))) *
              (float32)FD_THIRDPOWER(FD_COS(Outputs.fTrajHeading_rad))) /
             (one_minus_kappa_d * one_minus_kappa_d)) +
            (((float32)FD_COS(Outputs.fTrajHeading_rad) / one_minus_kappa_d) *
             Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm);

        Outputs.fTgtCrvGrdTrajInclPrevAndDT_1pms =
            TRJPLN_FBT_CalcTrajChgOfCurvature(
                Outputs.fTrajHeading_rad,
                Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm,
                Inputs->fTPLFBT_CridrRightSeg1_ChngOfCrv_1pm2,
                Inputs->fTPLFBT_TrajDistY_met, d_prime, d_primeprime,
                d_primeprimeprime_TrajPrevKappaAndDeadTime,
                Outputs.fTrajTgtCrv_1pm, v_traj);
        Outputs.fTrajTgtCrvGrd_1pms = TRJPLN_FBT_CalcTrajChgOfCurvature(
            Outputs.fTrajHeading_rad, Inputs->fTPLFBT_CridrRightSeg1_Crv_1pm,
            Inputs->fTPLFBT_CridrRightSeg1_ChngOfCrv_1pm2,
            Inputs->fTPLFBT_TrajDistY_met, d_prime, d_primeprime,
            d_primeprimeprime, Outputs.fTrajTgtCrv_1pm, v_traj);
    } else {
        Outputs.fTrajDistY_met = 0.0f;
        Outputs.fTrajHeading_rad = 0.0f;
        Outputs.fTrajHeadingInclPreview_rad = 0.0f;
        Outputs.fTrajTgtCrv_1pm = 0.0f;
        Outputs.fTgtCrvInclPreviewAndDeadTime_1pm = 0.0f;
        Outputs.fTrajTgtCrvGrd_1pms = 0.0f;
        Outputs.fTgtCrvGrdTrajInclPrevAndDT_1pms = 0.0f;
        Outputs.fTgtCrvTrajInclPreview_1pm = 0.0f;
        Outputs.fHeadingCurValuePreview_rad = Inputs->fDevHeading_rad;
        Outputs.fDistYCurValuePreview_met = Inputs->fDevDistY_met;
    }

    return Outputs;
}

/*****************************************************************************
  Functionname: TRJPLN_FBT_AvoidZero */ /*!

                                       @brief: return a minimum number if the
                                       input is almost zero

                                       @description: a protection that we often
                                       used in division operation

                                       @param[in]
                                                         const float32 input:
                                       the value we need to check whether is
                                       almost zero
                                       @param[out]


                                       @return
                                                         would return a minimum
                                       value if the input is almost zero,
                                       or return the input value directly
                                       *****************************************************************************/
float32 TRJPLN_FBT_AvoidZero(const float32 input) {
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
  Functionname: TRJPLN_FBT_CalcTrajChgOfCurvature */ /*!

                          @brief: calculate the chagne of curvature

                          @description: calculate the chagne of curvature

                          Kt_p = v_Bahn * ((((A_p * B) + (B_p * A)) + (C_p *
                          Kr)) + (Kr_p * C)).
                          A_p means the first derivative of A, same for B and C

                          A = d_pp + (Krpd_plus_Krdp * Tantheta)
                          B = (C * C) * Costheta
                          C = Costheta / one_minus_krd

                          @param[in]
                          float32 deltaTheta: heading angle delta between target
                          trajectory and
                          reference trajectory
                          float32 Kr: curvature of reference corridor
                          float32 Kr_p: first derivative of curvature of
                          reference corridor
                          float32 d: lateral distance
                          float32 d_p: first derivative of lateral distance
                          float32 d_pp: second derivative of lateral distance
                          float32 d_ppp: third derivative of lateral distance
                          float32 Kt: curvature of target trajectory
                          float32 v_Bahn: tangent velocity
                          @param[out]


                          @return
                          would return the calculated change of curvature
                          *****************************************************************************/
float32 TRJPLN_FBT_CalcTrajChgOfCurvature(float32 deltaTheta,
                                          float32 Kr,
                                          float32 Kr_p,
                                          float32 d,
                                          float32 d_p,
                                          float32 d_pp,
                                          float32 d_ppp,
                                          float32 Kt,
                                          float32 v_Bahn) {
    float32 Kt_p;
    float32 Costheta, Sintheta, Tantheta;
    float32 one_minus_krd, Krpd_plus_Krdp, one_minus_krd_2, one_minus_krd_3,
        deltatheta_p;
    float32 A, A_p, B, B_p, C, C_p;

    Costheta = TRJPLN_FBT_AvoidZero((float32)FD_COS(deltaTheta));
    Sintheta = (float32)FD_SIN(deltaTheta);
    Tantheta = (float32)FD_TAN(deltaTheta);
    one_minus_krd = TRJPLN_FBT_AvoidZero((1 - (Kr * d)));
    one_minus_krd_2 = one_minus_krd * one_minus_krd;
    one_minus_krd_3 = one_minus_krd_2 * one_minus_krd;
    Krpd_plus_Krdp = (Kr_p * d) + (Kr * d_p);

    A = d_pp + (Krpd_plus_Krdp * Tantheta);
    C = Costheta / one_minus_krd;
    B = (C * C) * Costheta;
    deltatheta_p = (Kt / TRJPLN_FBT_AvoidZero(C)) - Kr;
    A_p = (d_ppp + ((((2 * d_p) * Kr_p) + (d_pp * Kr)) * Tantheta)) +
          ((Krpd_plus_Krdp * ((1 / Costheta) / Costheta)) * deltatheta_p);
    B_p = ((((2 * Costheta) * Costheta) *
            ((Costheta * Krpd_plus_Krdp) -
             ((Sintheta * deltatheta_p) * one_minus_krd))) /
           one_minus_krd_3) -
          (((Sintheta * deltatheta_p) * C) * C);
    C_p = ((Costheta * Krpd_plus_Krdp) -
           ((Sintheta * deltatheta_p) * one_minus_krd)) /
          one_minus_krd_2;
    Kt_p = v_Bahn * ((((A_p * B) + (B_p * A)) + (C_p * Kr)) + (Kr_p * C));

    return Kt_p;
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
