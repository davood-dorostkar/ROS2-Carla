/*
 * \file       tue_prv_fusion_math.c
 *
 * \{
 *
 * \brief      Platform dependent implementation file for mathematical functions
 *
 */
/* PRQA S 0292 ++ */ /* MKS */
/*

*/
/* PRQA S 0292 -- */ /* MKS */
                     /*
                      * <br>=====================================================<br>
                      * <b>Copyright 2014 by Tuerme.</b>
                      * <br>
                      * All rights reserved. Property of Tuerme.<br>
                      * Restricted rights to use, duplicate or disclose of this code<br>
                      * are granted through contract.
                      * <br>=====================================================<br>
                      *
                      */

/*==================[inclusions]============================================*/
#include "tue_prv_fusion_math.h"
#include "tue_prv_fusion_math_int.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
#define ObjFusn_START_SEC_ROM

/* PRQA S 0759 1 */ /* Union required to define NaN */
LOCAL CONST(TueObjFusn_f32UnionType,
            ObjFusn_CONST) uf32NaN = {TUE_PRV_FUSION_MATH_FLOAT32_NAN_DEFAULT};
#define ObjFusn_STOP_SEC_ROM

/*==================[functions]=============================================*/
#define ObjFusn_START_SEC_CODE

float32 tue_prv_fusion_abs(
    const float32 f32x) /* PRQA S 1503 */ /* library fcn*/
{
    ///* PRQA S 0759 2 *//* Union required here */
    // TueObjFusn_f32UnionType uf32Union;   /* PRQA S 3204 */ /* union members
    // are modified */

    // uf32Union.f32Val = f32x;
    // uf32Union.u32Val = (uf32Union.u32Val &
    // 0x7FFFFFFFu);//(TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK |
    // TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK));// 0x7FFFFFFFu

    // return uf32Union.f32Val;

    if (f32x >= 0) {
        return f32x;
    } else {
        return -f32x;
    }
}

/* PRQA S 1505 2 */ /* Internal function */
uint32 tue_prv_fusion_exp(const float32 f32x) {
    /* Shift by number of mantissa bits, exponent is now on bit position 0 to 7,
     * sign bit is on position 8 */
    /* PRQA S 0310 1 */ /* Cast to Union type required */
    return (((((const TueObjFusn_f32UnionType *)&f32x)->u32Val) >>
             TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS) &
            TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF);
}

/* PRQA S 1505 2 */ /* Internal function */
boolean tue_prv_fusion_isInf(const float32 f32x) {
    /* PRQA S 0759 2 */ /* Union required here */
    TueObjFusn_f32UnionType uf32Union;
    /* PRQA S 3204 */ /* union members are modified */
    boolean bRes;

    uf32Union.f32Val = f32x;

    /** All exponend bits should be set and all mantissa bits should be zero to
     * code Inf */
    if ((uf32Union.u32Val &
         (0x7FFFFFFFu /*TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK | TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK*/)) ==
        TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK) {
        bRes = TRUE;
    } else {
        bRes = FALSE;
    }

    return bRes;
}

/* PRQA S 1505 2 */ /* Internal function */
boolean tue_prv_fusion_isNaN(const float32 f32x) {
    /* PRQA S 0759 2 */ /* Union required here */
    TueObjFusn_f32UnionType uf32Union;
    /* PRQA S 3204 */ /* union members are modified */
    boolean bRes;

    uf32Union.f32Val = f32x;

    /* All exponent bits set to one, at least one mantissa bit set to one */
    if (((uf32Union.u32Val & TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK) ==
         TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK) &&
        ((uf32Union.u32Val & TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK) > 0u)) {
        bRes = TRUE;
    } else {
        bRes = FALSE;
    }

    return bRes;
}

/* PRQA S 1505 2 */ /* Internal function */
float32 tue_prv_fusion_copysign(const float32 f32x, const float32 f32Sign) {
    /* PRQA S 0759 3 */ /* Union required here */
    TueObjFusn_f32UnionType uf32UnionX;
    /* PRQA S 3204 */ /* union members are modified */

    uf32UnionX.f32Val = f32x;

    uf32UnionX.u32Val &=
        0x7FFFFFFFu;    // (TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK |
                        // TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK);
    /* PRQA S 0310 1 */ /* Cast to Union type required */
    uf32UnionX.u32Val |=
        ((((const TueObjFusn_f32UnionType *)&f32Sign)->u32Val) &
         TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_MASK);

    return uf32UnionX.f32Val;
}

/* PRQA S 1505 2 */ /* Internal function */
float32 tue_prv_fusion_frexp(const float32 f32x,
                             CONSTP2VAR(sint32, AUTOMATIC, ObjFusn_VAR_NOINIT)
                                 ps32Exp) {
    float32 f32Res;
    const uint32 u32Exp = tue_prv_fusion_exp(f32x);

    /* Exponent is zero, inputted number is either zero or denormalized value */
    if (u32Exp == 0u) {
        *ps32Exp = 0;
        /* Return +- Zero depending on input sign */
        f32Res = FLT_ZERO;
    } else if (u32Exp == TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF) {
        /* Infinite value */
        f32Res = f32x;
    } else {
        /* Unbiased exponent */
        *ps32Exp =
            (((sint32)u32Exp) - TUE_PRV_FUSION_MATH_FLOAT32_EXP_BIAS) + 1;
        f32Res = tue_prv_fusion_set_exp(
            f32x, (uint32)(TUE_PRV_FUSION_MATH_FLOAT32_EXP_BIAS - 1));
    }

    f32Res = tue_prv_fusion_copysign(f32Res, f32x);

    return f32Res;
}

/* PRQA S 1532 2 */ /* Library Function */
float32 tue_prv_fusion_interp1(
    const float32 f32x0,
    const float32 f32y0,
    const float32 f32x1,
    const float32 f32y1,
    const float32 f32x) /* PRQA S 1503 */ /* library fcn*/
{
    float32 f32Res = FLT_ZERO;
    const float32 f32Denominator = f32x1 - f32x0;
    const float32 f32AbsDenominator = tue_prv_fusion_abs(f32Denominator);

    /* PRQA S 0759 1 */ /* Union required to define Inf */
    const TueObjFusn_f32UnionType uf32Inf = {
        TUE_PRV_FUSION_MATH_FLOAT32_INF_DEFAULT};

    if (f32AbsDenominator < TUE_PRV_FUSION_MATH_COMPARE_TO_ZERO) {
        f32Res = uf32Inf.f32Val;
    } else if (f32x1 < f32x0) {
        f32Res = uf32NaN.f32Val;
    } else if ((f32x < f32x0) || (f32x > f32x1)) {
        f32Res = uf32NaN.f32Val;
    } else {
        f32Res = (f32y1 - f32y0) * (f32x - f32x0);
        f32Res /= f32Denominator;
        f32Res += f32y0;
    }

    return f32Res;
}

/* PRQA S 1505 2 */ /* Internal function */
float32 tue_prv_fusion_set_exp(const float32 f32x, const uint32 u32Exp) {
    /* PRQA S 0759 2 */ /* Union required here */
    TueObjFusn_f32UnionType uf32Union;
    /* PRQA S 3204 */ /* union members are modified */

    uf32Union.f32Val = f32x;

    /* Clear exponent */
    uf32Union.u32Val &=
        (0x807FFFFF /*TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK | TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_MASK*/);
    /* Apply new exponent */
    uf32Union.u32Val |= ((u32Exp & TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF)
                         << TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS);

    return uf32Union.f32Val;
}

/* PRQA S 1505 2 */ /* Internal function */
float32 tue_prv_fusion_pack(const boolean bSign,
                            const uint32 u32Exp,
                            const uint32 u32Mant) {
    /* PRQA S 0759 2 */ /* Union required here */
    TueObjFusn_f32UnionType uf32Union;
    /* PRQA S 3204 */ /* union members are modified */

    uf32Union.u32Val =
        ((uint32)bSign << TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_POSITION) |
        ((u32Exp & TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF)
         << TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS) |
        (u32Mant & (TUE_PRV_FUSION_MATH_FLOAT32_MANT_ADD_IMPLICIT_ONE - 1u));
    return uf32Union.f32Val;
}

/* PRQA S 1505 2 */ /* Internal function */
boolean tue_prv_fusion_isZero(const float32 f32x) {
    boolean bResult;

    /* PRQA S 0310 1 */ /* Cast to Union type required */
    if ((((const TueObjFusn_f32UnionType *)&f32x)->u32Val &
         (0x7FFFFFFFu /*TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK | TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK*/)) ==
        0u) {
        bResult = TRUE;
    } else {
        bResult = FALSE;
    }

    return bResult;
}

/* PRQA S 1505 2 */ /* Internal function */
boolean tue_prv_fusion_signbit(const float32 f32x) {
    boolean bRes;

    /* PRQA S 0310 1 */ /* Cast to Union type required */
    if (((((const TueObjFusn_f32UnionType *)&f32x)->u32Val) &
         TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_MASK) ==
        TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_MASK) {
        bRes = TRUE;
    } else {
        bRes = FALSE;
    }

    return bRes;
}

/* PRQA S 1505 2 */ /* Internal function */
LOCAL float32 tue_prv_fusion_modff(const float32 f32x,
                                   CONSTP2VAR(float32,
                                              AUTOMATIC,
                                              ObjFusn_VAR_NOINIT) pf32IntVal) {
    /* PRQA S 0759 2 */ /* Union type required here */
    TueObjFusn_f32UnionType uf32Union;
    /* PRQA S 3204 */ /* union members are modified */
    uint32 u32l;
    sint32 s32Exp;
    uint32 u32Tmp;
    float32 f32Res;
    const boolean bIsNan = tue_prv_fusion_isNaN(f32x);

    uf32Union.f32Val = f32x;
    u32l = uf32Union.u32Val;

    u32Tmp = (u32l >> TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS) &
             TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF;
    s32Exp = (sint32)u32Tmp - TUE_PRV_FUSION_MATH_FLOAT32_EXP_BIAS;

    if (TRUE == bIsNan) {
        f32Res = uf32NaN.f32Val;
        *pf32IntVal = uf32NaN.f32Val;
    }
    /** Negative unbiased exponent means that the resulting floating point value
       is < 1 */
    else if (s32Exp < 0) {
        *pf32IntVal = tue_prv_fusion_copysign(FLT_ZERO, f32x);
        f32Res = f32x;
    }
    /*
     * for IEEE-754 single precision, an (unbiased!) exponent >=
     * FLOAT_FRACTION_SIZE tells us
     * that the fraction == 0.0
     * So check if the value EXCEEDS +-1 * 2 ^ 22
     */
    /** Single precision numbers with unbiased exponents greater than the number
       of mantissa bits */
    /** Do not have a franctional part anymore but are rounded to closest
       integer with gap 1 */
    else if (s32Exp >= (sint32)TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS) {
        *pf32IntVal = f32x;
        f32Res = tue_prv_fusion_copysign(FLT_ZERO, f32x);
    } else {
        /* here the float is
         * <-1*2^FLOAT_FRACTION_SIZE..+1*2^FLOAT_FRACTION_SIZE> */
        /* now clear the least significant bits, which would drop off at the
         * integer */
        /* that are the last FLOAT_FRACTION_SIZE - exp bits */
        u32l &= U32_MAX << (TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS -
                            (uint32)s32Exp);
        uf32Union.u32Val = u32l;
        *pf32IntVal = uf32Union.f32Val;
        f32Res = tue_prv_fusion_copysign(f32x - (*pf32IntVal), f32x);
    }

    return f32Res;
}

/* PRQA S 1505 2 */ /* Internal function */
uint32 tue_prv_fusion_mant(const float32 f32x) {
    /* Extract lower 23 Bits */
    /* PRQA S 0310 1 */ /* Cast to Union type required */
    return ((((const TueObjFusn_f32UnionType *)&f32x)->u32Val) &
            TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK);
}

/* PRQA S 1505 2 */ /* Internal function */
LOCAL float32 tue_prv_fusion_remainder(const float32 f32x, const float32 f32y) {
    const uint32 u32ExpX = tue_prv_fusion_exp(f32x);
    const uint32 u32ExpY = tue_prv_fusion_exp(f32y);
    uint32 u32MantX = tue_prv_fusion_mant(f32x);
    uint32 u32MantY = tue_prv_fusion_mant(f32y);
    uint32 u32Exp = u32ExpX;
    boolean bSign = tue_prv_fusion_signbit(f32x);
    uint32 u32Remain;
    uint32 u32Remain2;
    uint32 u32Mant;
    float32 f32Result;

    if (u32ExpY == TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF) {
        // f32y is NaN or Infinity
        if (u32MantY == 0u) {
            // f32y is +- Infinity
            if (u32ExpX != TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF) {
                // remainder( f32x, +-Infinity ) -> d1
                f32Result = f32x;
            } else {
                // remainder (+-Inf / +-NaN, +-Inf) -> 0
                f32Result = FLT_ZERO;
            }
        } else {
            // f32y is NaN, just return it.
            f32Result = f32y;
        }
    } else if (u32ExpX == TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF) {
        if (u32MantX == 0u) {
            // remquo(+-Infinity, d2) -> NaN
            f32Result = uf32NaN.f32Val;
        } else {
            // d1 is NaN, just return it.
            f32Result = f32x;
        }
    } else if (u32ExpY == 0u) {
        // remquo(f32x, zero) -> NaN
        f32Result = FLT_ZERO;
    } else if ((u32ExpX == 0u) || ((u32ExpX + 1u) < u32ExpY)) {
        f32Result = f32x;
    } else {
        u32MantX =
            u32MantX |
            TUE_PRV_FUSION_MATH_FLOAT32_MANT_ADD_IMPLICIT_ONE; /* add implicit 1
                                                                  to mantissa */
        u32MantY =
            u32MantY |
            TUE_PRV_FUSION_MATH_FLOAT32_MANT_ADD_IMPLICIT_ONE; /* add implicit 1
                                                                  to mantissa */
        u32Remain = u32MantX;
        /* Calculate the remainder. The numerator op1 is scaled in advance to
         * make sure the quotient always contains P bits.
         */
        if (u32Remain < u32MantY) {
            u32Remain <<= 1u;
            u32Exp--;
        } else {
            /* MISRA */
        }

        if (u32Exp >= u32ExpY) {
            while (u32Exp > u32ExpY) {
                if (u32Remain >= u32MantY) {
                    u32Remain -= u32MantY;
                } else {
                    /* MISRA */
                }

                u32Remain <<= 1u;
                u32Exp--;
            }

            if (u32Remain >= u32MantY) {
                u32Remain -= u32MantY;
            } else {
                /* MISRA */
            }

            /* normalize */
            while ((u32Remain != 0u) &&
                   ((u32Remain >>
                     TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS) == 0u)) {
                u32Remain <<= 1u;
                u32Exp--;
            }
        } else {
            /* MISRA */
        }

        if ((u32Exp == 0u) || (u32Remain == 0u)) {
            /* no denormals */
            u32Exp = 0u;
            u32Remain = 0u;
        } else {
            /* MISRA */
        }

        /* The remainder can be almost twice as large as
         * mant2 (divisor) in which case continuing the division would yield
         * an endless stream of '1's.
         * When the remaining dividend is larger than the divisor we should
         * round up. The special case dividend == divisor cannot occur because
         * that would imply an exact quotient of P + 1 bits starting and ending
         * with '1': There is no multiplier which would yield an exact P bit
         * representable product (==original dividend) for that.
         */
        if ((u32Exp == u32ExpY) ||
            ((u32Exp == (u32ExpY - 1u)) &&
             ((u32Remain > u32MantY) || ((u32Remain == u32MantY))))) {
            /* Round quotient up => extra subtract => negative remainder.
             * The exact quotient can never have P+1 leading '1's so no
             * need to test for a final carry into the exponent.
             */
            u32Remain2 = u32MantY;

            if (u32Exp == (u32ExpY - 1u)) {
                u32Remain2 = u32MantY << 1u;
            } else {
                /* MISRA */
            }

            if (TRUE == bSign) {
                bSign = FALSE;
            } else {
                bSign = TRUE;
            }

            if (u32Remain > u32Remain2) {
                u32Remain -= u32Remain2;
            } else {
                u32Remain = u32Remain2 - u32Remain;
            }

            /* normalize again.. */
            if ((u32Remain >>
                 (TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS + 1u)) > 0u) {
                u32Remain >>= 1u;
                u32Exp++;
            } else {
                /* MISRA */
            }

            while ((u32Remain != 0u) &&
                   ((u32Remain >>
                     TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS) == 0u)) {
                u32Remain <<= 1u;
                u32Exp--;
            }
        } else if ((u32Remain >>
                    (TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS + 1u)) >
                   0u) {
            u32Remain >>= 1u;
            u32Exp++;
        } else {
            /* MISRA */
        }

        u32Mant =
            u32Remain & (TUE_PRV_FUSION_MATH_FLOAT32_MANT_ADD_IMPLICIT_ONE -
                         1u); /* hide hidden bit again */
        f32Result = tue_prv_fusion_pack(bSign, u32Exp, u32Mant);
    }

    return f32Result;
}

/* PRQA S 1505 2 */ /* Internal function */
LOCAL float32 tue_prv_fusion_fmod(const float32 f32x, const float32 f32y) {
    if (f32y != 0) {
        return f32x - (int)(f32x / f32y) * f32y;
    } else {
        return 0;
    }
    // boolean  bSignBit;
    // float32  f32Result;
    // const boolean bYisZero  = tue_prv_fusion_isZero(f32y);
    // const float32 f32AbsX   = tue_prv_fusion_abs(f32x);
    // const float32 f32AbsY   = tue_prv_fusion_abs(f32y);

    // f32Result               = tue_prv_fusion_remainder(f32AbsX, f32AbsY);
    // bSignBit                = tue_prv_fusion_signbit(f32Result);

    // if (bSignBit == TRUE)
    //{
    //   f32Result += f32AbsY;
    //}
    // else
    //{
    //   /* MISRA */
    //}

    // f32Result = tue_prv_fusion_copysign(f32Result, f32x);

    // if (TRUE == bYisZero)
    //{
    //   /* Encode NaN */
    //   f32Result = uf32NaN.f32Val;
    //}
    // else
    //{
    //   /* MISRA */
    //}

    // return f32Result;
}

/* PRQA S 1503 2 */ /* Library Function */
float32 tue_prv_fusion_sqrt(float32 f32x) {
    float32 f32Z;
    sint32 s32Exp;
    uint32 u32Tmp;

    const uint32 u32Exp = tue_prv_fusion_exp(f32x);
    const boolean bIsInf = tue_prv_fusion_isInf(f32x);
    const boolean bIsNan = tue_prv_fusion_isNaN(f32x);

    if (u32Exp == 0u) {
        f32x = tue_prv_fusion_copysign(FLT_ZERO, f32x);
    } else if (TRUE == bIsInf) {
        /* Simply return Inf --> return x */
    } else if (TRUE == bIsNan) {
        /* Simply return NaN --> return x */
    } else if (f32x < FLT_ZERO) {
        /* Encode NaN */
        f32x = uf32NaN.f32Val;
    } else {
        f32Z = tue_prv_fusion_frexp(f32x, &s32Exp);

        if (0u != (((uint32)s32Exp & 1u))) {
            /* Clear lowest bit due to subsequent right shift */
            f32Z += f32Z;
            s32Exp--;
        } else {
            /* MISRA */
        }

        u32Tmp = ((uint32)s32Exp) >> 1u;
        s32Exp = (sint32)u32Tmp;

        if (f32Z > TUE_PRV_FUSION_MATH_SQRT2) {
            f32Z -= FLT_TWO;

            f32x = (((((((((((TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_1 *
                              f32Z) +
                             TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_2) *
                            f32Z) -
                           TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_3) *
                          f32Z) +
                         TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_4) *
                        f32Z) -
                       TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_5) *
                      f32Z) +
                     TUE_PRV_FUSION_MATH_SQRT_POLYNOM_1_CONSTANT_6) *
                    f32Z) +
                   TUE_PRV_FUSION_MATH_SQRT2;
        } else if (f32Z > TUE_PRV_FUSION_MATH_INV_SQRT2) {
            /* z is between sqrt(2)/2 and sqrt(2). */
            f32Z -= FLT_ONE;

            f32x = (((((((((((((TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_1 *
                                f32Z) -
                               TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_2) *
                              f32Z) +
                             TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_3) *
                            f32Z) -
                           TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_4) *
                          f32Z) +
                         TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_5) *
                        f32Z) -
                       TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_6) *
                      f32Z) *
                     f32Z) +
                    (TUE_PRV_FUSION_MATH_SQRT_POLYNOM_2_CONSTANT_7 * f32Z)) +
                   FLT_ONE;
        } else {
            /* z is between 0.5 and sqrt(2)/2. */
            f32Z -= FLT_ONE_HALF;

            f32x = (((((((((((TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_1 *
                              f32Z) +
                             TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_2) *
                            f32Z) -
                           TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_3) *
                          f32Z) +
                         TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_4) *
                        f32Z) -
                       TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_5) *
                      f32Z) +
                     TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_6) *
                    f32Z) +
                   TUE_PRV_FUSION_MATH_SQRT_POLYNOM_3_CONSTANT_7;
        }

        s32Exp += (sint32)tue_prv_fusion_exp(f32x);
        f32x = tue_prv_fusion_set_exp(f32x, (uint32)s32Exp);
    }
    return f32x;

    /// Approximates the square root of x
    /// Maximum error is 6.07%. No input checks are made.
    /// The caller must guarantee that the input is valid.
    /// @param[in]       f_x   A positive real number.
    /// @return          A rough approximation of x^(1/2)
    // BML_INLINE static float32 CML_STL_Sqrt_VeryFast(const float32 & f_x)
    /*BML_t_FloatAsUnsigned u;
    uint32 u_HexRepresentation;
    u.f_d = f32x;
    u_HexRepresentation = u.u_x;
    u_HexRepresentation -= 1U << 23U;
    u_HexRepresentation /= 2U;
    u_HexRepresentation += 1U << 29U;
    u.u_x = u_HexRepresentation;
    return u.f_d;*/
}

float32 tue_prv_fusion_log(float32 f32x) /* PRQA S 1503 */ /* library fcn*/
{
    float32 f32Zsq;
    sint32 s32Exp;
    const uint32 u32Exp = tue_prv_fusion_exp(f32x);
    const boolean bIsZero = tue_prv_fusion_isZero(f32x);

    if (f32x <= FLT_ZERO) {
        if (bIsZero == TRUE) {
            /* Return - Inf */
            f32x = tue_prv_fusion_pack(
                TRUE, TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF, 0x0u);
        } else {
            f32x = uf32NaN.f32Val;
        }
    } else if (u32Exp == TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF) {
        /*
         * We have to handle infinity separately since this limit case is not
         * handled properly by the polynomials below.
         * simply return x
         */
    } else {
        f32x = tue_prv_fusion_frexp(f32x, &s32Exp);
        if (f32x < TUE_PRV_FUSION_MATH_INV_SQRT2) {
            f32x = FLT_TWO * f32x;
            s32Exp = s32Exp - 1;
        } else {
            /* MISRA */
        }

        f32x = (f32x - FLT_ONE) / (f32x + FLT_ONE);
        f32Zsq = f32x * f32x;

        f32x = (((TUE_PRV_FUSION_MATH_LOG_COEFFICIENTS_P1 * f32Zsq) +
                 TUE_PRV_FUSION_MATH_LOG_COEFFICIENTS_P0) /
                (f32Zsq + TUE_PRV_FUSION_MATH_LOG_COEFFICIENTS_Q0)) *
               f32x;
        f32x += ((float32)s32Exp) * TUE_PRV_FUSION_MATH_LOG2F;
    }

    return f32x;
}

/* PRQA S 1532 2 */ /* Library Function */
float32 tue_prv_fusion_rtod(const float32 f32x) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return (TUE_PRV_FUSION_MATH_DEGREES_PER_RAD *
            ((f32x < FLT_ZERO) ? -f32x : f32x));
}

/* PRQA S 1503 2 */ /* Library Function */
float32 tue_prv_fusion_cos(const float32 f32x) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return tue_prv_fusion_sin_impl(((f32x < FLT_ZERO) ? -f32x : f32x), 1);
}

/* PRQA S 1503 2 */ /* Library Function */
float32 tue_prv_fusion_sin(const float32 f32x) {
    return tue_prv_fusion_sin_impl(f32x, 0);
}

/* PRQA S 1505 2 */ /* Internal Function */
LOCAL float32 tue_prv_fusion_sin_impl(float32 f32x, sint32 s32Quad) {
    float32 f32y;
    float32 f32Tmp;
    uint32 u32Tmp;

    const uint32 u32Exp = tue_prv_fusion_exp(f32x);
    const boolean bIsNan = tue_prv_fusion_isNaN(f32x);

    if (u32Exp == TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF) {
        if (TRUE == bIsNan) {
            // The code below is not safe for NaN, so we have to handle it
            // separately.
            // Simply return x as x == NaN
        } else {
            // C11 F.10.1.6: sin(+-Infinity) -> NaN

            /* Encode NaN */
            f32x = uf32NaN.f32Val;
        }
    } else {
        if (f32x < FLT_ZERO) {
            f32x = -f32x;
            s32Quad = s32Quad + 2; /* PRQA S 3120 1 */ /* math */
        } else {
            /* MISRA */
        }

        f32x = f32x * TUE_PRV_FUSION_MATH_SIN_SCALING_FACTOR; /* underflow ? */

        if (f32x > TUE_PRV_FUSION_MATH_SIN_MAX_SCALING) {
            f32y = tue_prv_fusion_modff(f32x, &f32Tmp);
            /* From here, we don't need the original 'x' anymore
             * So we just use it for other purposes
             */
            f32x = f32Tmp + (float32)s32Quad;
            (void)tue_prv_fusion_modff(FLT_ONE_QUARTER * f32x, &f32Tmp);
            f32x -= (FLT_FOUR * f32Tmp);
            s32Quad = (sint32)f32x;
        } else {
            f32y = f32x - ((float32)((sint32)f32x));
            u32Tmp = ((uint32)s32Quad + (uint32)f32x) & 0x03u;
            /* PRQA S 3120 1 */ /* math */
            s32Quad = (sint32)u32Tmp;
        }

        /* Quadrant 1 or 3 */
        if (0u != ((uint32)s32Quad & 0x01u)) {
            f32y = FLT_ONE - f32y;
        } else {
            /* MISRA */
        }

        if (s32Quad > 1) {
            f32y = -f32y;
        } else {
            /* MISRA */
        }

        f32x = f32y * f32y; /* y-square */

        f32x = (((((TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P2 * f32x) +
                   TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P1) *
                  f32x) +
                 TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_P0) *
                f32y) /
               (((f32x + TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_Q1) * f32x) +
                TUE_PRV_FUSION_MATH_SIN_COEFFICIENTS_Q0);
    }

    return f32x;
}

/* PRQA S 1503 2 */ /* library fcn*/
float32 tue_prv_fusion_norm_angle(const float32 f32x) {
    float32 f32Temp;

    f32Temp = tue_prv_fusion_fmod(f32x + TUE_PRV_FUSION_MATH_PI,
                                  TUE_PRV_FUSION_MATH_TWO_PI) +
              TUE_PRV_FUSION_MATH_TWO_PI;  // modulo 360deg
    f32Temp = tue_prv_fusion_fmod(f32Temp,
                                  TUE_PRV_FUSION_MATH_TWO_PI);  // modulo 360deg
    f32Temp -= TUE_PRV_FUSION_MATH_PI;  // substract 180deg

    return f32Temp;
}

/* PRQA S 1505 2 */ /* library fcn*/
LOCAL float32 tue_prv_fusion_atan(float32 f32x) {
    float32 f32Res;
    float32 f32argsq;
    float32 f32Tmp;

    if (f32x < TUE_PRV_FUSION_MATH_ATAN_SQ2M1) {
        f32argsq = f32x * f32x;
        f32Res = (TUE_PRV_FUSION_MATH_ATAN_P1 * f32argsq) +
                 TUE_PRV_FUSION_MATH_ATAN_P0;
        f32Res /= ((f32argsq + TUE_PRV_FUSION_MATH_ATAN_Q1) * f32argsq) +
                  TUE_PRV_FUSION_MATH_ATAN_Q0;
        f32Res *= f32x;
    } else if (f32x > TUE_PRV_FUSION_MATH_ATAN_SQ2P1) {
        f32x = FLT_ONE / f32x;
        f32argsq = f32x * f32x;
        f32Res = (TUE_PRV_FUSION_MATH_ATAN_P1 * f32argsq) +
                 TUE_PRV_FUSION_MATH_ATAN_P0;
        f32Res /= ((f32argsq + TUE_PRV_FUSION_MATH_ATAN_Q1) * f32argsq) +
                  TUE_PRV_FUSION_MATH_ATAN_Q0;
        f32Res *= f32x;
        f32Res = TUE_PRV_FUSION_MATH_PI_HALF - f32Res;
    } else {
        f32Tmp = (f32x - FLT_ONE) / (f32x + FLT_ONE);
        f32argsq = f32Tmp * f32Tmp;
        f32x = f32Tmp;
        f32Res = (TUE_PRV_FUSION_MATH_ATAN_P1 * f32argsq) +
                 TUE_PRV_FUSION_MATH_ATAN_P0;
        f32Res /= ((f32argsq + TUE_PRV_FUSION_MATH_ATAN_Q1) * f32argsq) +
                  TUE_PRV_FUSION_MATH_ATAN_Q0;
        f32Res *= f32x;
        f32Res += TUE_PRV_FUSION_MATH_PI_QUARTER;
    }

    return f32Res;
}

/* PRQA S 1503 2 */ /* library fcn*/
float32 tue_prv_fusion_atan2(const float32 f32y, const float32 f32x) {
    float32 f32Res;
    const boolean bIsInfY = tue_prv_fusion_isInf(f32y);
    const boolean bIsInfX = tue_prv_fusion_isInf(f32x);
    const boolean bIsZeroX = tue_prv_fusion_isZero(f32x);
    const boolean bIsZeroY = tue_prv_fusion_isZero(f32y);
    const boolean bSignBitX = tue_prv_fusion_signbit(f32x);

    if (TRUE == bIsInfY) {
        if ((TRUE == bIsInfX) && (TRUE == bSignBitX)) {
            f32Res = tue_prv_fusion_copysign(
                TUE_PRV_FUSION_MATH_PI_THREE_QUARTER, f32y);
        } else if ((TRUE == bIsInfX) && (FALSE == bSignBitX)) {
            f32Res =
                tue_prv_fusion_copysign(TUE_PRV_FUSION_MATH_PI_QUARTER, f32y);
        } else {
            f32Res = tue_prv_fusion_copysign(TUE_PRV_FUSION_MATH_PI_HALF, f32y);
        }
    } else if (TRUE == bIsZeroX) {
        if (TRUE == bIsZeroY) {
            if (TRUE == bSignBitX) {
                /* y is +-0, x is -0, return +- pi */
                f32Res = tue_prv_fusion_copysign(TUE_PRV_FUSION_MATH_PI, f32y);
            } else {
                f32Res = f32y;
            }
        } else {
            f32Res = tue_prv_fusion_copysign(TUE_PRV_FUSION_MATH_PI_HALF, f32y);
        }
    } else if (f32x < FLT_ZERO) {
        if (TRUE == bIsZeroY) {
            f32Res = tue_prv_fusion_copysign(TUE_PRV_FUSION_MATH_PI, f32y);
        } else if (f32y > FLT_ZERO) {
            f32Res = TUE_PRV_FUSION_MATH_PI - tue_prv_fusion_atan(-f32y / f32x);
        } else {
            f32Res = -TUE_PRV_FUSION_MATH_PI + tue_prv_fusion_atan(f32y / f32x);
        }
    } else if (f32y > FLT_ZERO) {
        f32Res = tue_prv_fusion_atan(f32y / f32x);
    } else {
        f32Res = -tue_prv_fusion_atan(-f32y / f32x);
    }

    return f32Res;
}

float32 tue_prv_fusion_pow2(
    const float32 f32x) /* PRQA S 1503 */ /* library fcn*/
{
    return (f32x * f32x);
}

/* -----------------------------------------------------------------------*/
/**
 * \typedef          MAX
 *
 * \brief               maximum of two numbers
 *
 *-----------------------------------------------------------------------*/

/* PRQA S 1532 2 */ /* Library Function */
uint16 tue_prv_fusion_max_U16(const uint16 A, const uint16 B) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return ((A > B) ? A : B);
}

float32 tue_prv_fusion_max_F32(const float32 A, const float32 B) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return ((A > B) ? A : B);
}

float32 tue_prv_fusion_min_F32(const float32 A, const float32 B) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return ((A < B) ? A : B);
}

/* -----------------------------------------------------------------------*/
/**
 * \typedef          MIN
 *
 * \brief               minimum of two numbers
 *
 * -----------------------------------------------------------------------*/

/* PRQA S 1532 2 */ /* Library Function */
uint8 tue_prv_fusion_min_U8(const uint8 A, const uint8 B) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return ((A < B) ? A : B);
}

uint16 tue_prv_fusion_min_U16(const uint16 A, const uint16 B) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return ((A < B) ? A : B);
}

/* -----------------------------------------------------------------------*/
/**
 * \typedef          TUE_CML_MinMax
 *
 * \brief            number inside boundary
 *
 */
/* -----------------------------------------------------------------------*/

/* PRQA S 1503 2 */ /* Library Function */
sint16 tue_prv_fusion_min_max_S16(const sint16 A,
                                  const sint16 minVal,
                                  const sint16 maxVal) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return ((A < minVal) ? minVal : ((A > maxVal) ? maxVal : A));
}

float32 tue_prv_fusion_min_max_F32(const float32 A,
                                   const float32 minVal,
                                   const float32 maxVal) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return ((A < minVal) ? minVal : ((A > maxVal) ? maxVal : A));
}

/* PRQA S 1532 2 */ /* Library Function */
uint32 tue_prv_fusion_min_max_U32(const uint32 A,
                                  const uint32 minVal,
                                  const uint32 maxVal) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return ((A < minVal) ? minVal : ((A > maxVal) ? maxVal : A));
}
#define ObjFusn_STOP_SEC_CODE

/**
 * \}
 */
