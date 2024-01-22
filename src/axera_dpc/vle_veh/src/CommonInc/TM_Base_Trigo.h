#ifdef PRQA_SIZE_T
#pragma PRQA_NO_SIDE_EFFECTS GDB_cos32
#pragma PRQA_NO_SIDE_EFFECTS GDB_sin32
#pragma PRQA_NO_SIDE_EFFECTS GDB_tan32
#pragma PRQA_NO_SIDE_EFFECTS GDB_tan32s
#pragma PRQA_NO_SIDE_EFFECTS GDBcos_52
#pragma PRQA_NO_SIDE_EFFECTS GDBsin_52
#pragma PRQA_NO_SIDE_EFFECTS GDBtan_52
#pragma PRQA_NO_SIDE_EFFECTS GDBatan2_66
#pragma PRQA_NO_SIDE_EFFECTS GDBatan_66
#pragma PRQA_NO_SIDE_EFFECTS GDBasin_66
#pragma PRQA_NO_SIDE_EFFECTS GDBacos_66
#pragma PRQA_NO_SIDE_EFFECTS GDBexp_power
#pragma PRQA_NO_SIDE_EFFECTS GDBexp_100
#pragma PRQA_NO_SIDE_EFFECTS GDBexp
#endif

#ifndef _BML_EXT_INCLUDED
#endif

#ifndef _BML_TRIGO_INCLUDED
#define _BML_TRIGO_INCLUDED

#define GDB_TRIG_OPTIMIZED 1

#define GDB_TRIG_SMALL_SERIES 1

#define GDB_TRIG_WITH_EXP 1

#ifndef BML_NO_RTE_DEPENDENCY
#ifndef b_FALSE
#define b_FALSE ((boolean)0)
#endif
#ifndef b_TRUE
#define b_TRUE ((boolean)1)
#endif
#endif

BML_INLINE float32 BML_atan18(float32 f_x, float32 f_y);
BML_INLINE float32 BML_atan18(float32 f_x, float32 f_y) {
    float32 f_q;
    float32 f_Absq;
    float32 f_Result;
    const float32 f_MagicNumber1 = 0.2447F;
    const float32 f_MagicNumber2 = 0.0663F;
    boolean b_Reciprocal = (BML_f_Abs(f_x) > BML_f_Abs(f_y)) ? b_TRUE : b_FALSE;
    if (b_Reciprocal == b_TRUE) {
        f_q = f_y / f_x;
    } else {
        f_q = f_x / f_y;
    }
    f_Absq = BML_f_Abs(f_q);
    f_Result =
        ((BML_f_Pi * 0.25F) * f_q) -
        f_q * (f_Absq - 1.0F) * (f_MagicNumber1 + (f_MagicNumber2 * f_Absq));
    if (b_Reciprocal == b_TRUE) {
        f_Result = (f_q > 0.0F) ? (BML_f_Half_Pi - f_Result)
                                : (-BML_f_Half_Pi - f_Result);
    }
    return f_Result;
}

BML_INLINE float32 BML_sin33_Core(float32 f_Angle);
BML_INLINE float32 BML_sin33_Core(float32 f_Angle) {
    const float32 f_C0 = 0.63661977F;
    const float32 f_C1 = 1.28318531F;
    const float32 f_C2 = 0.14159265F;

    float32 f_y = f_Angle * f_C0;
    float32 f_y2 = f_y * f_y;
    float32 f_z = 0.5F * f_y * (BML_f_Pi - (f_y2 * (f_C1 - f_y2 * f_C2)));

    return f_z;
}

BML_INLINE float32 BML_cos33_Core(float32 f_Angle);
BML_INLINE float32 BML_cos33_Core(float32 f_Angle) {
    const float32 f_C0 = 0.63661977F;
    const float32 f_C1 = 1.28318531F;
    const float32 f_C2 = 0.14159265F;

    float32 f_y = (BML_f_Abs(f_Angle) - BML_f_Half_Pi) * f_C0;
    float32 f_y2 = f_y * f_y;
    float32 f_z = -0.5F * f_y * (BML_f_Pi - (f_y2 * (f_C1 - f_y2 * f_C2)));

    return f_z;
}

#if (GDB_TRIG_OPTIMIZED)
#if (GDB_TRIG_SMALL_SERIES)
#ifndef COS_
#define COS_(x) GDB_cos32(x)
#endif
#ifndef SIN_
#define SIN_(x) GDB_sin32(x)
#endif
#ifndef TAN_
#define TAN_(x) GDB_tan32(x)
#endif
#else
#define COS_(x) GDBcos_52(x)
#define SIN_(x) GDBsin_52(x)
#define TAN_(x) GDBtan_52(x)
#endif
#ifndef COS_HD_
#define COS_HD_(x) GDBcos_52(x)
#endif
#ifndef SIN_HD_
#define SIN_HD_(x) GDBsin_52(x)
#endif
#ifndef TAN_HD_
#define TAN_HD_(x) GDBtan_52(x)
#endif
#ifndef ATAN2_
#define ATAN2_(y, x) GDBatan2_66(y, x)
#endif
#ifndef ATAN_
#define ATAN_(x) GDBatan_66(x)
#endif
#ifndef ASIN_
#define ASIN_(x) GDBasin_66(x)
#endif
#ifndef ACOS_
#define ACOS_(x) GDBacos_66(x)
#endif
#if (GDB_TRIG_WITH_EXP)
#define EXP_(x) (GDBexp(x))
#else
#define EXP_(x) ((float32)exp((float32)x))
#endif
#else

#include "math.h"

#define ATAN2_(y, x) ((float32)atan2((float32)(y), (float32)(x)))
#define ATAN_(x) ((float32)atan((float32)(x)))
#define TAN_(x) ((float32)tan((float32)(x)))
#define ASIN_(x) ((float32)asin((float32)(x)))
#define SIN_(x) ((float32)sin((float32)(x)))
#define COS_(x) ((float32)cos((float32)(x)))
#define ACOS_(x) ((float32)acos((float32)(x)))

#define COS_HD_(x) ((float32)cos((float64)(x)))
#define SIN_HD_(x) ((float32)sin((float64)(x)))
#define TAN_HD_(x) ((float32)tan((float64)(x)))

#define EXP_(x) (((float32))exp(((float32))(x)))

#endif

#if (GDB_TRIG_OPTIMIZED)

extern float32 GDB_cos32(float32 f_angle);
extern float32 GDB_sin32(float32 f_angle);
extern float32 GDB_tan32(float32 f_angle);

extern float32 GDBcos_52(float32 f_angle);
extern float32 GDBsin_52(float32 f_angle);
extern float32 GDBtan_52(float32 f_angle);

extern float32 CML_tanh58(float32 f_Arg);

extern float32 GDBatan2_66(float32 f_yaxis, float32 f_xaxis);
extern float32 GDBatan_66(float32 f_tan);
extern float32 GDBacos_66(float32 f_cos);
extern float32 GDBasin_66(float32 f_sin);
extern float32 CML_sin66_Core(float32 f_Angle);
extern float32 BML_cos66_Core(float32 f_Angle);
extern float32 CML_sin66(float32 f_Angle);
extern float32 BML_cos66(float32 f_Angle);

extern float32 GDBexp_power(float32 f_base, uint32 u_power);

#if (GDB_TRIG_WITH_EXP)
extern float32 GDBexp_100(float32 f_power);
extern float32 GDBexp(float32 f_power);
#endif
#endif

#if (GDB_TRIG_OPTIMIZED)

#define C_LONG_MAX 2147483647L

#define C_TWOPI (2.0F * BML_f_Pi)

#define C_TWO_OVER_PI (2.0F / BML_f_Pi)

#define C_HALFPI (0.5F * BML_f_Pi)

#define C_QUARTERPI (0.25F * BML_f_Pi)

#define C_FOUR_OVER_PI (4.0F / BML_f_Pi)

#define C_THREEHALFPI (1.5F * BML_f_Pi)

#define C_TENPI (10.0F * BML_f_Pi)

#define C_COS_32_C1 0.99940307f

#define C_COS_32_C2 (-0.49558072f)

#define C_COS_32_C3 0.03679168f

#define C_TAN_32_C1 (-3.6112171f)

#define C_TAN_32_C2 (-4.6133253f)

#define C_COS_52_C1 0.99999329F

#define C_COS_52_C2 (-0.49991243F)

#define C_COS_52_C3 0.04148774F

#define C_COS_52_C4 (-0.00127120F)

#define C_TAN_56_C1 (-3.16783027F)

#define C_TAN_56_C2 0.13451612F

#define C_TAN_56_C3 (-4.03332198F)

#define C_ATAN_66_C1 1.68676291F

#define C_ATAN_66_C2 0.43784973F

#define C_ATAN_66_C3 1.68676331F
#ifndef C_SIXTHPI
#define C_SIXTHPI (BML_f_Pi / 6.0F)
#endif
#define C_TANSIXTHPI 0.57735026F

#define C_TANTWELFTHPI 0.26794919F

#ifndef C_SIXTH
#define C_SIXTH (1.0F / 6.0F)
#endif

#if (GDB_TRIG_WITH_EXP)

#define C_EXP_100_ROOT_E 1.28402541F

#define C_EXP_100_A 1.00000003F

#define C_EXP_100_B 0.49999798F

#define C_EXP_100_C 0.16670407F

#define C_EXP_100_D 0.04136541F

#define C_EXP_100_E 9.41927345E-3
#endif
#endif

extern void GDBsincos(float32 f_val, float32 *p_sin, float32 *p_cos);

#endif
