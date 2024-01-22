/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
INCLUDES
*****************************************************************************/
#include "lck.h"

#include "lck_par.h"
//#include "cml_const.h"
/*****************************************************************************
  MODULE QAC messages suppression
*****************************************************************************/

/* QAC suppression of Msg(2:3121) Hard-coded 'magic' floating constant '0.0f' */
/* QAC suppression of Msg(2:3211) The global identifier '...' is defined here
 * but is not used in this translation unit. */
/* PRQA S 3121,3211 ++ */

/* 01-Dec-14, MaPa (uidw6143):
   The basic idea of the parameter file is to define parameters, but not to
   use it. */

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/
/* QAC suppression of Msg(2:3429) A function-like macro is being defined. */
/* QAC suppression of Msg(2:3453) A function could probably be used instead of
 * this function-like macro. */
/* PRQA S 3429, 3453 1 */
#define LCK_KPH2MS(kph_) ((kph_) / 3.6f)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*! @brief       Max Steering Wheel Angle [deg]
    @general     Absolute value for max steering wheel angle.
    @typical     90.0f  @unit  deg   @min -   @max -   */
// SET_MEMSEC_VAR(LCK_PAR_MAX_REF_STR_ANG)

/*! @brief       Max Steering Wheel Angle Velocity [deg/s]
    @general     Absolute value for max steering wheel angle velocity.
    @typical     180.0f  @unit  deg/s   @min -   @max -   */

/*! @brief      Vehicle speed array to interpolate state feedback gains.
    @general    Vehicle speed array to interpolate state feedback gains
    @typical -  @unit  m/s   @min -   @max -   */
float32 fLCKParVelArray_c[LCK_PAR_NO_SUPPORT_POINTS] = {
    LCK_KPH2MS(30.0f),  LCK_KPH2MS(50.0f),  LCK_KPH2MS(80.0f),
    LCK_KPH2MS(100.0f), LCK_KPH2MS(120.0f), LCK_KPH2MS(150.0f),
    LCK_KPH2MS(180.0f), LCK_KPH2MS(200.0f)};

/*! @brief      Array of steering angle feedback gains.
    @general    Array of steering angle feedback gains. Based on the current
                vehicle speed the steering angle feedback gain is interpolated
                based on the values of this array (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
float32 fLCKParStrAngGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS] = {
    /* 0.3009f,   0.5076f,  0.5819f,   0.3370f,  0.4540f,  0.3885f,  0.4286f,
       0.4518f     560A */
    /* 0.0536f,   0.0714f,   0.0999f,  0.1206f,  0.1393f,  0.1230f,  0.1311f,
       0.1357f     Lexus GS450h: Regensburg Test 2014/cw46 */
    /* 0.0429f,   0.0571f,   0.0799f,  0.0965f,  0.1114f,  0.0984f,  0.1049f,
       0.1086f     Lexus GS450h: Yokohama Joint Test 2014/cw47 */
    /* 0.1372f,   0.2166f,   0.3685f,  0.4981f,  0.6236f,  0.5847f,  0.6603f,
       0.7051f     Lexus GS450h: DEKRA1 parameter set from 2015/cw08 */
    /* 0.34298f,  0.54149f,  0.92113f, 1.2452f,  1.5591f,  1.4617f,  1.6508f,
       1.7628f     Lexus GS450h: DEKRA1 parameter set from 2015/cw08 for tuning
       */
    /* 0.2744f,   0.4332f,   0.6448f,  0.7471f,  0.7795f,  0.5847f,  0.6603f,
       0.5288f     Lexus GS450h: ATP1 parameter set from 2015/cw17 */
    /* 0.2823f,   0.4411f,   0.6339f,  0.7063f,  0.7013f,  0.6523f,  0.5482f,
       0.5828f     Lexus GS450h: Final (ATP2 parameter set from 2015/cw17) */
    /* 0.40335f,  0.63017f,  1.0565f,  1.4126f,  1.7532f,  1.6307f,  1.8274f,
       1.9427f     Lexus GS450h: ATP2 parameter set from 2015/cw17 for tuning */
    0.2823f, 0.4411f, 0.6339f, 0.7063f, 0.7013f,
    0.6523f, 0.5482f, 0.5828f /* Geely       : 1st version, from base */
};

/*! @brief       steering angle feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParStrAngGainsArray_c is used as steering
                 angle feedback gain.
    @typical     2.81000f  @unit  -   @min -   @max -   */

/*! @brief       Steering angle feed forward master gain: [-]
    @general     Master gain for feed forward by Steering angle
    @typical     1.0  @unit  -   @min -   @max -   */

/*! @brief      Array of steering velocity feedback gains.
    @general    Array of steering velocity feedback gains. Based on the current
                vehicle speed the steering velocity feedback gain is
   interpolated
                based on the values of this array (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
LCK_PAR_CONST float32 fLCKParStrVelGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/*! @brief       Steering velocity feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParStrAngGainsArray_c is used as steering
                 angle feedback gain.
    @typical     2.81000f  @unit  -   @min -   @max -   */

/*! @brief      Array of yaw rate feedback gains.
    @general    Array of yaw rate feedback gains. Based on the current
                vehicle speed the yaw rate feedback gain is interpolated
                based on the values of this array (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
LCK_PAR_CONST float32 fLCKParYawRateGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS] = {
    /*  0.0398f,   0.0798f,   0.1209f,   0.0820f,  0.1002f,  0.0913f,
       0.1051f,  0.1134f     560A */
    /*  0.0091f,   0.0153f,   0.0249f,   0.0311f,  0.0372f,  0.0387f,
       0.0435f,  0.0464f     Lexus GS450h: Regensburg Test 2014/cw46 */
    /*  0.0073f,   0.0122f,   0.0199f,   0.0249f,  0.0298f,  0.0310f,
       0.0348f,  0.0371f     Lexus GS450h: Yokohama Joint Test 2014/cw47
       */
    /*  0.0110f,   0.0186f,   0.0329f,   0.0435f,  0.0548f,  0.0645f,
       0.0775f,  0.0858f     Lexus GS450h: DEKRA1 parameter set from
       2015/cw08            */
    /*  0.027384f, 0.046506f, 0.08225f,  0.10871f, 0.13708f, 0.16117f,
       0.19381f, 0.21439f    Lexus GS450h: DEKRA1 parameter set from
       2015/cw08 for tuning */
    /*  0.0219f,   0.0372f,   0.0576f,   0.0652f,  0.0685f,  0.0645f,
       0.0775f,  0.0643f     Lexus GS450h: ATP1 parameter set from
       2015/cw17              */
    /*  0.0231f,   0.0394f,   0.0597f,   0.0656f,  0.0659f,  0.0765f,
       0.0684f,  0.0754f     Lexus GS450h: Final (ATP2 parameter set
       from 2015/cw17)      */
    /*  0.033048f, 0.056289f, 0.099459f, 0.13115f, 0.16472f, 0.19117f,
       0.22814f, 0.25123f    Lexus GS450h: ATP2 parameter set from
       2015/cw17 for tuning   */
    0.0231f, 0.0394f, 0.0597f, 0.0656f, 0.0659f,
    0.0765f, 0.0684f, 0.0754f /* Geely       : 1st version, from base */
};

/*! @brief       Feedback master gain: [-]
    @general     Feedback master gain.
    @typical     1.0  @unit  -   @min -   @max -   */

/*! @brief       Yaw rate feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParYawRateGainsArray_c is used as yaw
                 rate feedback gain.
    @typical     1.0  @unit  -   @min -   @max -   */

/*! @brief       Yaw rate feed forward master gain: [-]
    @general     Master gain for feed forward by yaw rate
    @typical     1.0  @unit  -   @min -   @max -   */

/*! @brief      Array of heading feedback gains.
    @general    Array of heading feedback gains. Based on the current
                vehicle speed the heading feedback gain (vehicles orientation
                within the lane) is interpolated based on the values of this
                array (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
float32 fLCKParHeadingGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS] = {
    /*  0.2675f,  0.3536f,  0.4143f,  0.2616f,  0.3007f,  0.2626f,  0.2973f,
       0.3197f     560A */
    /*  0.1235f,  0.1540f,  0.2083f,  0.2458f,  0.2829f,  0.3053f,  0.3449f,
       0.3702f     Lexus GS450h: Regensburg Test 2014/cw46 */
    /*  0.0988f,  0.1232f,  0.1666f,  0.1966f,  0.2263f,  0.2442f,  0.2759f,
       0.2962f     Lexus GS450h: Yokohama Joint Test 2014/cw47 */
    /*  0.2879f,  0.3076f,  0.2795f,  0.2637f,  0.2377f,  0.2567f,  0.2893f,
       0.3107f     Lexus GS450h: DEKRA1 parameter set from 2015/cw08 */
    /*  0.35992f, 0.38447f, 0.46582f, 0.52749f, 0.59422f, 0.64175f, 0.72326f,
       0.77684f    Lexus GS450h: DEKRA1 parameter set from 2015/cw08 for tuning
       */
    /*  0.2879f,  0.3076f,  0.3261f,  0.3165f,  0.2971f,  0.2567f,  0.2893f,
       0.2331f     Lexus GS450h: ATP1 parameter set from 2015/cw17 */
    /*  0.3060f,  0.3302f,  0.3457f,  0.3272f,  0.2950f,  0.3174f,  0.2677f,
       0.2870f     Lexus GS450h: Final (ATP2 parameter set from 2015/cw17) */
    /*  0.43719f, 0.47171f, 0.57623f, 0.65434f, 0.73755f, 0.79355f, 0.8922f,
       0.95662f    Lexus GS450h: ATP2 parameter set from 2015/cw17 for tuning */
    0.3060f, 0.3302f, 0.3457f, 0.3272f, 0.2950f,
    0.3174f, 0.2677f, 0.2870f /* Geely       : 1st version, from base */
};

/*! @brief       Heading feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParHeadingGainsArray_c is used as
                 heading angle feedback gain.
    @typical     2.6347f  @unit  -   @min -   @max -   */

/*! @brief      Array of lateral deviation feedback gains.
    @general    Array of lateral deviation gains. Based on the current
                vehicle speed the lateral deviation (from target trajectory)
                feedback gain is interpolated based on the values of this array
                (gain scheduling).
    @typical -  @unit  -   @min -   @max -   */
float32 fLCKParLatDevGainsArray_c[LCK_PAR_NO_SUPPORT_POINTS] = {
    /*  0.0154f,   0.0154f,   0.0152f,   0.0086f,   0.0086f,   0.0064f,
       0.0064f,   0.0064f       560A */
    /*  0.0262f,   0.0253f,   0.0245f,   0.0242f,   0.0239f,   0.0233f,
       0.0230f,   0.0228f       Lexus GS450h: Regensburg Test 2014/cw46 */
    /*  0.0210f,   0.0202f,   0.0196f,   0.0194f,   0.0191f,   0.0186f,
       0.0184f,   0.0182f       Lexus GS450h: Yokohama Joint Test 2014/cw47 */
    /*  0.0126f,   0.0126f,   0.0095f,   0.0079f,   0.0063f,   0.0063f,
       0.0063f,   0.0063f       Lexus GS450h: DEKRA1 parameter set from
       2015/cw08            */
    /*  0.015811f, 0.015811f, 0.015811f, 0.015811f, 0.015811f, 0.015811f,
       0.015811f, 0.015811f     Lexus GS450h: DEKRA1 parameter set from
       2015/cw08 for tuning */
    /*  0.0126f,   0.0126f,   0.0111f,   0.0095f,   0.0079f,   0.0063f,
       0.0063f,   0.0047f       Lexus GS450h: ATP1 parameter set from 2015/cw17
       */
    /*  0.0155f,   0.0154f,   0.0130f,   0.0108f,   0.0086f,   0.0085f,
       0.0064f,   0.0063f       Lexus GS450h: Final (ATP2 parameter set from
       2015/cw17)      */
    /*  0.022122f, 0.021952f, 0.021742f, 0.021612f, 0.021505f, 0.02133f,
       0.021207f, 0.021135f     Lexus GS450h: ATP2 parameter set from 2015/cw17
       for tuning   */
    0.0155f, 0.0154f, 0.0130f, 0.0108f, 0.0086f,
    0.0085f, 0.0064f, 0.0063f /* Geely       : 1st version, from base */
};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*! @brief       Lateral deviation feedback master gain: [-]
    @general     Master gain after being multiplied with
                 fLCKParLatDevGainsArray_c is used as
                 Lateral deviation feedback gain.
    @typical     0.24616f  @unit  -   @min -   @max -   */

/*! @brief       Linear fading coefficient: [-]
    @general     Linear fading coefficient for kinematic controller
                 output. A very high value of e.g. 10000 will switch
                 off this functionality, a small value will result in
                 a slow approach of the kinematic controller output.
    @typical     0.2  @unit  -   @min 0.0   @max -   */

/*! @brief       Filter co-efficient for the curvature LPF: [-]
    @general     The curvature signal from the lane is very noisy and
                 leads to oscillations on the steering wheel. By low
                 pass filtering this signal the feedforward steering
                 angle is free from Oscillations.
    @typical     0.2  @unit  -   @min 0.0   @max -   */

/*! @brief       Self Steering Gradient selection: [-]
    @general     The self steering gradient of the vehcle can either
                 be calculated or recieved as a parameter from VDY.
                 This uint8 parameter specifies the SSG used in the
                 kinematic controller
                   1: Calculated from bicycle model
                   2: Use the parameter from VDY
    @typical     1u  @unit  -   @min 1u   @max 2u   */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
