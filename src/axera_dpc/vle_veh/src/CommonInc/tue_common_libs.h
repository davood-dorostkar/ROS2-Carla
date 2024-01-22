#pragma once
#ifndef TUE_COMMON_LIBS_H
#define TUE_COMMON_LIBS_H

//#include "TM_Global_Types.h"
/*! \file **********************************************************************

  PROJECT:                   COMMON

  CPU:                       CPU-Independent

  COMPONENT:                 COMMON TOOLS

  MODULNAME:                 tue_common_libs.h

  DESCRIPTION:               common libs for tue development

  AUTHOR:                    $Author: tao.guo

  CREATION DATE:             $Date: 2020/12/25

  VERSION:                   $Revision: 1.0.0


  CHANGES:
  ---*/ /*---
  CHANGE:                    $Log: tue_common_libs.h
  CHANGE:                    Initial version

**************************************************************************** */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#ifndef SWITCH_ON
#define SWITCH_ON 1u
#endif

#ifndef SWITCH_OFF
#define SWITCH_OFF 0u
#endif

/*****************************************************************************
  DEBUG PRINT
*****************************************************************************/
#ifdef DEBUG
#include <stdio.h>
#define DEBUG_Print(fmt, ...)                                       \
    fprintf(stderr, "DebugPrint(%s:%d):\t" fmt, __func__, __LINE__, \
            ##__VA_ARGS__)
#else
//#define DEBUG_Print(fmt, ...) ((void)0)
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/
typedef enum tue_switch_state {
    TUE_SWITCH_STATE_OFF,       /*!<TUE_SWITCH_STATE_OFF*/
    TUE_SWITCH_STATE_ON,        /*!<TUE_SWITCH_STATE_ON*/
    TUE_SWITCH_STATE_ACTION_OFF /*!< Special return value (3state): switch
                                   action but false condition */
} tue_switch_state_t;

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
#include "TM_Global_Types.h"
// #ifndef glob_type_H
// // typedef unsigned char Boolean;
// typedef unsigned char uint8;
// typedef unsigned short uint16;
// typedef unsigned long uint32;
// typedef signed long sint32;
// typedef float float32;
// typedef uint8 AlgoSignalState_t;
// typedef uint16 AlgoCycleCounter_t;
// typedef uint32 AlgoDataTimeStamp_t;
// typedef unsigned int ubit32_t;
// typedef unsigned char boolean;
// #else
// // typedef unsigned int ubit32_t;
// #endif
typedef struct {
    float32 f0;  // First Element of the 2D vector
    float32 f1;  // Second Element of the 2D vector
} TUE_CML_Vector2D_t;

typedef struct TUE_CML_PolyResult {
    float32 fC0;     /*!<fC0 */
    float32 fC1;     /*!<fC1 */
    float32 fC2;     /*!<fC2 */
    boolean isValid; /*!<isValid */
} TUE_CML_PolyResult_t;

typedef struct TUE_CML_Switch_t {
    ubit32_t AKT_STATUS : 1;           /*!<AKT_STATUS*/
    ubit32_t LAST_STATUS : 1;          /*!<LAST_STATUS*/
    ubit32_t OK_WHILE_SWITCHED_ON : 1; /*!<OK_WHILE_SWITCHED_ON*/
    ubit32_t CYCLE_TIMER : 9;
    /*!< max cycle timer 510 cycles (=10s at 20ms cycles)*/ /*%unit:cycles*/
    ubit32_t DURATION_TIME_INACTIVE : 10;
    /*!< max duration measurement 1022 cycles (=20s at 20ms cycles)*/ /*%unit:cycles*/
    ubit32_t DURATION_TIME_ACTIVE : 10;
    /*!< max duration measurement 1022 cycles (=20s at 20ms cycles)*/ /*%unit:cycles*/
} TUE_CML_Switch_t;
typedef struct {
    float32 dAmin;  // Minimum output value

    float32 dAmax;  // Maximum output value

    float32 dM;  // Slope of the line (Amax-Amin)/(Emax-Emin)

    float32 dB;  // Intercept value of the line (Amax-Amin)/(Emax-Emin) * Emin
} TUE_CML_t_LinFunctionArgs;

/*****************************************************************************
  CONSTS
*****************************************************************************/
#ifndef CML_MatrixBoundsCheckOn
#define CML_MatrixBoundsCheckOn FALSE
#endif
#define TUE_C_F32_VALUE_INVALID (1000.0f)
#define TUE_C_UI16_VALUE_MAX (65530u)
#define TUE_C_UI16_VALUE_INVALID (65535u)
#define TUE_C_UI8_VALUE_MAX (250u)
#define TUE_C_UI8_VALUE_INVALID (255u)
#define TUE_C_I8_VALUE_INVALID (127)
#define TUE_C_SIXTH (1.0f / 6.0f)
#define TUE_C_MS_KMH ((float32)0.277778f)

#define TUE_C_F32_DELTA ((float32)0.0001f)
#define TUE_C_F32_EXT_DELTA 1e-8f
#define TUE_CML_SqrtApprox_NumExpo (23u)
#define TUE_CML_SqrtApprox_MantissaMask (0x007fffffu)
#define TUE_CML_SqrtApprox_ExponentOffset (0x7f)
#define TUE_CML_SqrtApprox_AlmostZero (1e-20f)
#define TUE_CML_GaussianCDFMinSigma 0.000001f
#define TUE_CML_SQRT_OF_2 (1.414213562373095f) /* square root of 2 */
#define TUE_CML_AlmostZero (1e-15f)
#define TUE_CML_AlmostNegZero (-1e-15f)
#define TUE_CML_GaussErrFctMaxX 1.99f
#define TUE_CML_GaussErrFctConst0 0.002289f
#define TUE_CML_GaussErrFctConst1 1.146f
#define TUE_CML_GaussErrFctConst2 0.1092f
#define TUE_CML_GaussErrFctConst3 0.2841f
#define TUE_CML_GaussErrFctConst4 0.08869f
#define TUE_CML_Pi 3.14159265359f
#define TUE_CML_ModuloEps 0.0000001f
#define TUE_CML_LONG_MAX 2147483647L
#define TUE_CML_TAN_56_C1 (-3.16783027F)
#define TUE_CML_TAN_56_C2 0.13451612F
#define TUE_CML_TAN_56_C3 (-4.03332198F)
#define TUE_CML_SWITCH_CYCLETIMER_INIT 511
#define TUE_CML_SWITCH_TIME_MAX 1023
/*! magic constant no. 1 for calculating cos() with 3.2 decimals of accuracy */
#define C_COS_32_C1 0.99940307f
/*! magic constant no. 2 for calculating cos() with 3.2 decimals of accuracy */
#define C_COS_32_C2 (-0.49558072f)
/*! magic constant no. 3 for calculating cos() with 3.2 decimals of accuracy */
#define C_COS_32_C3 0.03679168f
/*! magic constant no. 1 for calculating tan() with 3.2 decimals of accuracy */
#define C_TAN_32_C1 (-3.6112171f)
/*! magic constant no. 2 for calculating tan() with 3.2 decimals of accuracy */
#define C_TAN_32_C2 (-4.6133253f)
/*! magic constant no. 1 for calculating cos() with 5.2 decimals of accuracy */
#define C_COS_52_C1 0.99999329F
/*! magic constant no. 2 for calculating cos() with 5.2 decimals of accuracy */
#define C_COS_52_C2 (-0.49991243F)
/*! magic constant no. 3 for calculating cos() with 5.2 decimals of accuracy */
#define C_COS_52_C3 0.04148774F
/*! magic constant no. 4 for calculating cos() with 5.2 decimals of accuracy */
#define C_COS_52_C4 (-0.00127120F)
/*! optimized TAN-algorithm */
/*! magic constant no. 1 for calculating tan() with 5.6 decimals of accuracy */
#define C_TAN_56_C1 (-3.16783027F)
/*! magic constant no. 2 for calculating tan() with 5.6 decimals of accuracy */
#define C_TAN_56_C2 0.13451612F
/*! magic constant no. 3 for calculating tan() with 5.6 decimals of accuracy */
#define C_TAN_56_C3 (-4.03332198F)
/*! optimized ArTAN-algorithm */
/*! magic constant no. 1 for calculating atan() with 6.6 decimals of accuracy */
#define C_ATAN_66_C1 1.68676291F
/*! magic constant no. 2 for calculating atan() with 6.6 decimals of accuracy */
#define C_ATAN_66_C2 0.43784973F
/*! magic constant no. 3 for calculating atan() with 6.6 decimals of accuracy */
#define C_ATAN_66_C3 1.68676331F
/*! pi/6.0, used in atan routines     */
#ifndef C_SIXTHPI
#define C_SIXTHPI (TUE_CML_Pi / 6.0F)
#endif
/*! tan(pi/6), used in atan routines  */
#define C_TANSIXTHPI 0.57735026F
/*! tan(pi/12), used in atan routines */
#define C_TANTWELFTHPI 0.26794919F
/*****************************************************************************
  MACROS
*****************************************************************************/

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#define TUE_CML_Sqr(x) ((x) * (x))
#define TUE_CML_Sign(x) (((x) == (0)) ? (0) : (((x) > (0)) ? (1) : (-1)))
#define TUE_CML_Abs(x) (((x) < (0L)) ? (-(x)) : (x))
#define TUE_CML_Min(x, y) (((x) < (y)) ? (x) : (y))
#define TUE_CML_Max(x, y) (((x) > (y)) ? (x) : (y))
#define TUE_CML_MinMax(min, max, value) \
    (TUE_CML_Min(TUE_CML_Max(min, value), max))
#define TUE_CML_IsZero(value) (TUE_CML_Abs(value) < TUE_CML_AlmostZero)
#define TUE_CML_IsNonZero(value) (TUE_CML_Abs(value) >= TUE_CML_AlmostZero)
#define TUE_CML_MultAdd(a, b, d) ((a * b) + d)

#define TUE_RAD2DEG(rad_) ((rad_) * (180.F / TUE_CML_Pi))
#define TUE_DEG2RAD(deg_) ((deg_) * (TUE_CML_Pi / 180.F))
#define TUE_ROUND(x) TUE_CML_Round2FloatGen(x)
#define TUE_ROUND_TO_INT(x) TUE_CML_Round2IntGen(x)
#define TUE_F_Abs(x) (((x) < (0.0f)) ? (-(x)) : (x))
#define TUE_F_IsNonZero(value) (TUE_F_Abs(value) >= TUE_CML_AlmostZero)
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
//���� ����ʽ ��ֵ����
float32 TUE_CML_CalculatePolygonValue2D(sint32 s_NrOfTableRows,
                                        const TUE_CML_Vector2D_t a_Table[],
                                        float32 f_InputValue);
float32 TUE_CML_LinearInterpolation(
    float32 f_X1, float32 f_Y1, float32 f_X2, float32 f_Y2, float32 f_XPos);
float32 TUE_CML_BoundedLinInterpol(
    TUE_CML_t_LinFunctionArgs const* const p_Params, const float32 f_Value);
float32 TUE_CML_BoundedLinInterpol2(float32 f_IVal,
                                    float32 f_Imin,
                                    float32 f_Imax,
                                    float32 f_Omin,
                                    float32 f_Omax);
float32 TUE_CML_CalcStdGaussianCDF(float32 f_value,
                                   float32 f_avg,
                                   float32 f_sigma);
float32 TUE_CML_CalcGaussErrorFunction(float32 f_value);
void TUE_CML_CalcPointApproxPolyL2(TUE_CML_PolyResult_t* pPoly,
                                   const float32 pafX[],
                                   const float32 pafY[],
                                   uint8 uNumPts);
float32 TUE_CML_CalcDistYOfClothoidsCurve(float32 fHeadingAngle,
                                          float32 fCurve,
                                          float32 fCurveDer,
                                          float32 fDistX,
                                          float32 fDistY0);
boolean TUE_CML_RisingEdgeSwitch(const boolean bSwitch, boolean* pPrevSwitch);
boolean TUE_CML_FallingEdgeSwitch(const boolean bSwitch,
                                  boolean* pPrevSwitch);  //�½��ж�
// tue_switch_state_t TUE_CML_HoldRepeatSwitch(TUE_CML_Switch_t * const pSwitch,
// const boolean StartCondition, const boolean HoldCondition, uint16 StartTime,
// uint16 RepeatTime);//���ӳ١�Debounce����
void TUE_CML_InitSwitch(
    TUE_CML_Switch_t* const pSwitch);  //�������½���������Switch�źų�ʼ������
void TUE_CML_SetStateSwitch(TUE_CML_Switch_t* const pSwitch,
                            const boolean State);  //�����źŸ�ֵ

boolean TUE_CML_RSFlipFlop(const boolean S,
                           const boolean R,
                           boolean* pPrevQ);  // R-S Flip-Flop
float32 TUE_CML_RateLimiter(const float32 fInput,
                            const float32 fLimPos,
                            const float32 fLimNeg,
                            const float32 fTs,
                            float32* pPrevOutput);  // Rate limiter
boolean TUE_CML_HysteresisFloat(const float32 fInput,
                                const float32 fThresHigh,
                                const float32 fThresLow,
                                boolean* pPrevOutput);  // HysteresisFloat

//���Ǻ�������
#ifndef COS_
#define COS_(x) TUE_CML_GDB_cos32(x)  //�;���
#endif
#ifndef SIN_
#define SIN_(x) TUE_CML_GDB_sin32(x)  //�;���
#endif
#ifndef TAN_
#define TAN_(x) TUE_CML_GDB_tan32(x)  //�;���
#endif
#ifndef COS_HD_
#define COS_HD_(x) TUE_CML_GDBcos_52(x)  //�߾���
#endif
#ifndef SIN_HD_
#define SIN_HD_(x) TUE_CML_GDBsin_52(x)  //�߾���
#endif
#ifndef TAN_HD_
#define TAN_HD_(x) TUE_CML_GDBtan_52(x)  //�߾���
#endif
#ifndef ATAN_
#define ATAN_(x) TUE_CML_GDBatan_66(x)  //�߾���
#endif
#ifndef ATAN2_
#define ATAN2_(y, x) TUE_CML_GDBatan2_66(y, x)  //
#endif
#ifndef ASIN_
#define ASIN_(x) TUE_CML_GDBasin_66(x)  //�߾���
#endif
#ifndef ACOS_
#define ACOS_(x) TUE_CML_GDBacos_66(x)  //�߾���
#endif

void TUE_CML_LowPassFilter(float32* f_Old, float32 f_New, float32 f_Alpha);
float32 TUE_CML_SqrtApprox(float32 f_radicand);
float32 TUE_CML_ModTrig(float32 f_dividend, float32 f_divisor);  //�������
float32 TUE_CML_GDB_cos32(float32 f_angle);                      //
float32 TUE_CML_GDB_sin32(float32 f_angle);                      //
float32 TUE_CML_GDB_tan32(float32 f_angle);                      //
float32 TUE_CML_GDB_tan32s(float32 f_angle);                     //
float32 TUE_CML_GDBtan_52(float32 f_angle);                      //
float32 TUE_CML_GDBsin_52(float32 f_angle);                      //
float32 TUE_CML_GDBcos_52(float32 f_angle);                      //
float32 TUE_CML_GDBtan_56s(float32 f_angle);                     //
float32 TUE_CML_GDBatan_66(float32 f_tan);                       //
float32 TUE_CML_GDBacos_66(float32 f_cos);                       //
float32 TUE_CML_GDBasin_66(float32 f_sin);                       //
float32 TUE_CML_GDBatan2_66(float32 f_yaxis, float32 f_xaxis);  //
//����
sint32 TUE_CML_Round2IntGen(float32 x);
float32 TUE_CML_Round2FloatGen(float32 x);

// simulation related functions
// Computes object distance and velocity value based on defined object
// trajectory
void SimulationObjectGenerator(float32 fObjTrajHeadingAngle_rad,
                               float32 fObjTrajCurve_1pm,
                               float32 fObjTrajCurveDer_nu,
                               float32 fObjInitDistX0_met,
                               float32 fObjInitDistY0_met,
                               float32 fObjAbsVelX_mps,
                               float32 fEgoVelX_mps,
                               float32 fPassedTime_sec,
                               float32* fOutObjDistX_met,
                               float32* fOutObjDistY_met,
                               float32* fOutObjRelVelX_mps,
                               float32* fOutObjRelVelY_mps);
boolean TUE_CML_TimerRetrigger(float32 fDeltaTime_sec,
                               boolean bReset,
                               float32 fTimeLimit_sec,
                               float32* fRemainTime_sec);
/*****************************************************************************
  CONSTS ADD WRAPPER
*****************************************************************************/
float32 SafeDiv(float32 fDivisor);

#ifndef GDBmathLinFuncLimBounded
#define GDBmathLinFuncLimBounded(f_XPos, f_X1, f_X2, f_Y1, f_Y2) \
    TUE_CML_BoundedLinInterpol2(f_XPos, f_X1, f_X2, f_Y1,        \
                                f_Y2)  // linear interpolation
#endif

#ifndef CML_f_CalculatePolyValue
#define CML_f_CalculatePolyValue(sTableRows, aTables, fInputValue) \
    TUE_CML_CalculatePolygonValue2D(sTableRows, aTables,           \
                                    fInputValue)  // 2d look-up table
#endif

#ifndef GDB_Math_CalculatePolygonValue
#define GDB_Math_CalculatePolygonValue(sTableRows, aTables, fInputValue) \
    CML_f_CalculatePolyValue(sTableRows, aTables,                        \
                             fInputValue)  // 2d look-up table
#endif

#ifndef GDB_Math_LowPassFilter
#define GDB_Math_LowPassFilter(f_Old, f_New, f_Alpha) \
    TUE_CML_LowPassFilter(f_Old, f_New, f_Alpha)  // Low Pass Filter
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) TUE_RAD2DEG(rad)  // Rad to Degree, 1 rad -> 57.3 degree
#endif

#ifndef DEG2RAD
#define DEG2RAD(deg) TUE_DEG2RAD(deg)  // Degree to Rad, 57.3 degree -> 1 rad
#endif

#ifndef fABS
#define fABS(x) TUE_CML_Abs(x)  // Calculate Abs value ,-1 -> 1
#endif

#ifndef MIN
#define MIN(x, y) TUE_CML_Min(x, y)  // Calculate Minimum number
#endif

#ifndef MAX
#define MAX(x, y) TUE_CML_Max(x, y)  // Calculate Maximum number
#endif

#ifndef SQR
#define SQR(x) TUE_CML_Sqr(x)  // Calculate the square, 2^2 = 4
#endif

#ifndef SQRT
#define SQRT(x) TUE_CML_SqrtApprox(x)  // Calculate square root, sqrt(4) = 2
#endif

#ifndef ROUND
#define ROUND(x) TUE_ROUND(x)  // round float to float
#endif

#ifndef ROUND_TO_INT
#define ROUND_TO_INT(x) TUE_ROUND_TO_INT(x)  // round float to int
#endif

#ifndef GDBVector2_t
#define GDBVector2_t TUE_CML_Vector2D_t  // 2d table
#endif
#ifndef C_MS_KMH
#define C_MS_KMH TUE_C_MS_KMH
#endif
/********************************Tuerme Common method
library************************************* 001:TUE_CML_Abs_M Absolute value
calculation 002:TUE_CML_Min_M                      Calculate the minimum of two
numbers 003:TUE_CML_Max_M                      Calculate the maximum of two
numbers 004:TUE_CML_SetBit_M                   Set specific binary bit to 1
    005:TUE_CML_ClcBit_M                   Clear specific binary bit to 0
    006:TUE_CML_IsNonZero_M                Judge whether it is 0
    007:TUE_CML_Limit_M                    Limit input signal to the upper and
lower saturation values 008:TUE_CML_GradLimit_M                Limit rising and
failing rates of signal 009:TUE_CML_DivProtection_M            Division zero
protection 010:TUE_CML_Hysteresis_M               Hysteresis function
    011:TUE_CML_TurnOnDelay_M              Turn on delay function
    012:TUE_CML_TurnOffDelay_M             Turn off delay function
    014:TUE_CML_SRTrigger_M                SR flip-flop
    015:TUE_CML_LowPassFilter_M            Low pass filtering
    016:TUE_CML_LookUpTable2D              Two dimensional table lookup function
    017:TUE_CML_LookUpTable3D              Three dimensional table lookup
function 018:TUE_CML_PosY3rd_M                  Calculate position Y by lane
clothoid 019:TUE_CML_Yaw3rd_M                   Calculate Yaw angle by lane
clothoid 020:TUE_CML_Crv3rd_M                   Calculate curvature by lane
clothoid 021:TUE_CML_Interp2_M                  Quadratic polynomial
interpolation 022:TUE_CML_Sin_M                      Sine of argument in radians
    023:TUE_CML_Cos_M                      Cosine of argument in radians
    024:TUE_CML_Tan_M                      Tangent of argument in radians
    025:TUE_CML_Tan_M                      Square root
**************************************************************************************************/
#ifndef C_SIXTH
#define C_SIXTH (1.0F / 6.0F)
#endif

#ifndef C_F32_DELTA
#define C_F32_DELTA ((float32)0.0001f)
#endif

#ifndef C_KMH_MS
#define C_KMH_MS (3.6F)
#endif

#ifndef INT8_T
typedef signed char INT8_T;
#endif

#ifndef INT16_T
typedef signed short INT16_T;
#endif

#ifndef INT32_T
typedef signed int INT32_T;
#endif

#ifndef UINT8_T
typedef unsigned char UINT8_T;
#endif

#ifndef UINT16_T
typedef unsigned short UINT16_T;
#endif

#ifndef UINT32_T
typedef unsigned int UINT32_T;
#endif

#ifndef REAL32_T
typedef float REAL32_T;
#endif

#define TUE_CML_Abs_M(fInput) (((fInput) < (0.0F)) ? (-(fInput)) : (fInput))
#define TUE_CML_Min_M(fInput1, fInput2) \
    (((fInput1) < (fInput2)) ? (fInput1) : (fInput2))
#define TUE_CML_Max_M(fInput1, fInput2) \
    (((fInput1) > (fInput2)) ? (fInput1) : (fInput2))
#define TUE_CML_Setbit_M(uData, uBit) (uData |= (0x01 << (uBit)))
#define TUE_CML_GetBit_M(uData, uBit) (((uData >> uBit) & (0x01)) == (0x01))
#define TUE_CML_Clrbit_M(uData, uBit) (uData &= (~(0x01 << (uBit))))
#define TUE_CML_IsNonZero_M(fInput) (TUE_CML_Abs_M(fInput) >= 1E-15F)

REAL32_T TUE_CML_Limit_M(REAL32_T fInput, REAL32_T fMin, REAL32_T fMax);
REAL32_T TUE_CML_GradLimit_M(REAL32_T fInput,
                             REAL32_T fUpperLimit,
                             REAL32_T fLowerLimit,
                             REAL32_T fSysTime,
                             REAL32_T fLastOutput);
REAL32_T TUE_CML_DivProtection_M(REAL32_T fNumerator,
                                 REAL32_T fDenominator,
                                 REAL32_T fDefault);
UINT8_T TUE_CML_Hysteresis_M(REAL32_T fInput,
                             REAL32_T fThresHigh,
                             REAL32_T fThresLow,
                             UINT8_T bLastOutput);  // HysteresisFloat
UINT8_T TUE_CML_TurnOnDelay_M(UINT8_T bInput,
                              REAL32_T fDelayTime,
                              REAL32_T fCycleTime,
                              REAL32_T* Timer,
                              UINT8_T bLastOutput);
UINT8_T TUE_CML_TurnOffDelay_M(UINT8_T bInput,
                               REAL32_T fDelayTime,
                               REAL32_T fCycleTime,
                               REAL32_T* Timer,
                               UINT8_T bLastOutput);
UINT8_T TUE_CML_SRTrigger_M(UINT8_T bSet, UINT8_T bReset, UINT8_T bLastOutput);
REAL32_T TUE_CML_LowPassFilter_M(REAL32_T fInput,
                                 REAL32_T fTimeFilter,
                                 REAL32_T fTimeSys,
                                 REAL32_T fLastOutput);
REAL32_T TUE_CML_LookUpTable2D(REAL32_T u0,
                               const REAL32_T bp0[],
                               const REAL32_T table[],
                               UINT32_T maxIndex);

REAL32_T TUE_CML_PosY3rd_M(REAL32_T fPosX,
                           REAL32_T fPosY0,
                           REAL32_T fYaw,
                           REAL32_T fCrv,
                           REAL32_T fCrvRate);
REAL32_T TUE_CML_Yaw3rd_M(REAL32_T fPosX,
                          REAL32_T fYaw,
                          REAL32_T fCrv,
                          REAL32_T fCrvRate);
REAL32_T TUE_CML_Crv3rd_M(REAL32_T fPosX, REAL32_T fCrv, REAL32_T fCrvRate);
void TUE_CML_Interp2_M(const REAL32_T fX[],
                       const REAL32_T fY[],
                       REAL32_T PolyCoeff[]);

REAL32_T TUE_CML_Sin_M(REAL32_T fAngle);
REAL32_T TUE_CML_Cos_M(REAL32_T fAngle);
REAL32_T TUE_CML_Tan_M(REAL32_T fAngle);
REAL32_T TUE_CML_Sqrt_M(REAL32_T fInput);

void TUE_CML_MemoryCopy_M(INT8_T* FromPtr, INT8_T* ToPtr, INT16_T ByteCnt);

/********************************Tuerme Common Matrix
library************************************* 001:TUE_CML_CreateMatrix_M Create
matrix 002:TUE_CML_GetMatrixElement_M         Access to matrix element
    003:TUE_CML_AddMatrices_M              Matrix addition (in place/out place)
    004:TUE_CML_CopyMatrix_M               Copy data from one matrix to another
    005:TUE_CML_TransposeMatrix_M
    006:TUE_CML_SubtractMatrices_M         Matrix subtraction (in place/out
place) 007:TUE_CML_ScaleMatrix_M              Matrix multiplication with scalar
    008:TUE_CML_MutiplyMatrices_M          Matrix multiplication
    009:TUE_CML_InitMatrix_M               Matrix initialization with a const
value 010:TUE_CML_CreateIdentityMatrix_M     Initializes matrix with identity
matrix 011:TUE_CML_MultAddGen_M 012:TUE_CML_InvertMatrixCramer2_M      Compute
matrix inverse for matrix size 2x2 014:TUE_CML_InvertMatrixCramer3_M Compute
matrix inverse for matrix size 3x3
**************************************************************************************************/
/* The description of the matrix(row count, column count and max size) required
 * for matrix operations */
typedef struct {
    UINT8_T col;      // Number of columns in the matrix
    UINT8_T row;      // Number of rows in the matrix
    UINT8_T maxsize;  // Maximum memory size allocated for this matrix
} TUE_CML_sMtrxInfo_t;

/* The matrix data structure contains the descriptor structure element
which contains row count, column count and max size required for matrix
operations. The matrix data is linked via a pointer.This allows using a single
matrix data type, for all matrix dimensions. It also allows the separation of
data and header for saving memory space : the matrix data can be stored on the
heap, while the wrapper can be created temporarily on the stack when needed
for computations.  */

typedef struct {
    TUE_CML_sMtrxInfo_t Desc;  // Matrix descriptor (dimensions, size, etc.)
    REAL32_T*
        pData;  // Pointer to the memory location where the data is stored */
} TUE_CML_sMatrix_t;

/* creates matrix and allocates new payload data with global scope */
#define TUE_CML_CreateMatrix_M(name, rows, cols)                          \
    static REAL32_T fMtrxData##name[(UINT32_T)(rows) * (UINT32_T)(cols)]; \
    TUE_CML_sMatrix_t AlgoMtrxHeader##name = {                            \
        {(UINT8_T)(cols), (UINT8_T)(rows),                                \
         (UINT16_T)((UINT32_T)(rows) * (UINT32_T)(cols))},                \
        fMtrxData##name};                                                 \
    TUE_CML_sMatrix_t* name =                                             \
        &AlgoMtrxHeader##name;  ///< Creates matrix and allocates new payload
                                ///< data with global scope

#define TUE_CML_GetMatrixElement_M(name, Row, Col) \
    (name)->pData[(UINT32_T)(Col) +                \
                  ((UINT32_T)(Row) *               \
                   (name)->Desc.col)]  ///<  Access to matrix element

void TUE_CML_AddMatrices_M(TUE_CML_sMatrix_t* p_MatrixRes,
                           const TUE_CML_sMatrix_t* p_MatrixA,
                           const TUE_CML_sMatrix_t* p_MatrixB);
void TUE_CML_CopyMatrix_M(TUE_CML_sMatrix_t* p_MatrixRes,
                          const TUE_CML_sMatrix_t* p_MatrixA);
void TUE_CML_TransposeMatrix_M(TUE_CML_sMatrix_t* p_MatrixRes,
                               const TUE_CML_sMatrix_t* p_MatrixA);
void TUE_CML_SubtractMatrices_M(TUE_CML_sMatrix_t* p_MatrixRes,
                                const TUE_CML_sMatrix_t* p_MatrixA,
                                const TUE_CML_sMatrix_t* p_MatrixB);
void TUE_CML_ScaleMatrix_M(TUE_CML_sMatrix_t* p_MatrixA, REAL32_T f_Val);
void TUE_CML_MutiplyMatrices_M(TUE_CML_sMatrix_t* p_MatrixRes,
                               const TUE_CML_sMatrix_t* p_MatrixA,
                               const TUE_CML_sMatrix_t* p_MatrixB);
void TUE_CML_InitMatrix_M(TUE_CML_sMatrix_t* p_Matrix,
                          UINT32_T u_RowNr,
                          UINT32_T u_ColNr,
                          REAL32_T f_Val);
void TUE_CML_CreateIdentityMatrix_M(TUE_CML_sMatrix_t* p_Matrix,
                                    UINT32_T u_Size);

REAL32_T TUE_CML_MultAddGen_M(REAL32_T a, REAL32_T b, REAL32_T d);
UINT8_T TUE_CML_InvertMatrixCramer2_M(REAL32_T a_res[4], REAL32_T a_in[4]);
UINT8_T TUE_CML_InvertMatrixCramer3_M(REAL32_T a_res[9], REAL32_T a_in[9]);

/********************************Tuerme common algorithm
library********************************* 001:TUE_CML_PolyFit_M Polynomial
fitting

**************************************************************************************************/

/* order+1 (number of coefficients) of fitted polynomial */
#define POLYFIT_ORDER_3RD (4U)
#define POLYFIT_ORDER_2ND (3U)
#define POLYFIT_ORDER_1ST (2U)
/* number of used data samples for polyfit */
#define POLYFIT_SAMPLE_POINTS (20U)

typedef struct {
    UINT8_T bEnable1st;        /*enable 1st order fit*/
    UINT8_T bEnable2nd;        /*enable 2nd order fit*/
    UINT8_T bEnable3rd;        /*enable 3rd order fit*/
    REAL32_T fFctCrvDecay;     /* Curvature change decay factor for polyfit */
    REAL32_T fFctCrvChngDecay; /* Curvature change decay factor for polyfit */
    REAL32_T fPosXArray[POLYFIT_SAMPLE_POINTS]; /*array of x values*/
    REAL32_T fPosYArray[POLYFIT_SAMPLE_POINTS]; /*array of y values*/
    REAL32_T fFctWeight[POLYFIT_SAMPLE_POINTS]; /*weighting for each point*/
} sPFTInput_t;

typedef struct {
    REAL32_T fPosY01st;
    REAL32_T fPosY02nd;
    REAL32_T fPosY03rd;
    REAL32_T fHeading1st;
    REAL32_T fHeading2nd;
    REAL32_T fHeading3rd;
    REAL32_T fCrv2nd;
    REAL32_T fCrv3rd;
    REAL32_T fChngOfCrv3rd;
    UINT8_T bTrajInvalid1st;
    UINT8_T bTrajInvalid2nd;
    UINT8_T bTrajInvalid3rd;
    REAL32_T fDevToTraj1st;
    REAL32_T fDevToTraj2nd;
    REAL32_T fDevToTraj3rd;
} sPFTOutput_t;

extern void TUE_CML_PolyFit_M(const sPFTInput_t* pPFTInput,
                              sPFTOutput_t* pPFTOutput);
UINT8_T TUE_CML_CalcInvertMatrix_M(TUE_CML_sMatrix_t* p_MatrixA,
                                   REAL32_T* p_DataA,
                                   REAL32_T* p_DataRes);
extern void TUE_CML_InvertMatrix_M(TUE_CML_sMatrix_t* p_MatrixRes,
                                   TUE_CML_sMatrix_t* p_MatrixA);

#define SenseTime_Memcpy(source, dst, cpySize) memcpy(dst, source, cpySize)

#endif
