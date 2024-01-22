/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
#include "envm_common_utils.h"

/************************************************************************
  Functionname:    Tue_CML_CalculateDistancePoint2Circle               */
BML_t_TrajRefPoint Tue_CML_CalculateDistancePoint2Circle(float32 f_X,
                                                         float32 f_Y,
                                                         float32 f_C0) {
    float32 f_Radius = RADIUS_INIT_VALUE;
    float32 f_R = 0.f;
    float32 f_DistToCourse = 0.f;
    float32 f_DistOnCourse = 0.f;
    float32 f_RefCourseDistX = 0.f;
    float32 f_RefCourseDistY = 0.f;
    BML_t_TrajRefPoint ReferencePoint;

    if (fABS(f_C0) > CURVATURE_USE_CIRCLE_EQUATION) {
        float32 f_NormVecX = 0.f;
        float32 f_NormVecY = 0.f;
        f_Radius = 1.0f / (f_C0);
        /* Object Transform to Moment Pole Coordinates */
        f_Y -= f_Radius;
        f_R = SQRT(SQR(f_X) + SQR(f_Y));

        /* Check for divison by zero */
        /* Check for divison by zero */
        if (TUE_CML_IsZero(f_R)) {
            /* Distances are zero */
        } else {
            /* NormVec to Course always pointing to the left side of course */
            if (f_C0 > 0.0f) {
                f_NormVecX = -f_X / f_R;
                f_NormVecY = -f_Y / f_R;
                f_DistToCourse = (fABS(f_Radius) - f_R);
                f_DistOnCourse =
                    f_Radius * ((0.5f * TUE_CML_Pi) + ATAN2_(f_Y, f_X));
            } else {
                f_NormVecX = f_X / f_R;
                f_NormVecY = f_Y / f_R;
                f_DistToCourse = -(fABS(f_Radius) - f_R);
                f_DistOnCourse =
                    f_Radius * (ATAN2_(f_Y, f_X) - (0.5f * TUE_CML_Pi));
            }
            /* DistCourse (fRadius-fR) positive when object left of course;
             * negative when object right of course*/

            f_RefCourseDistX = f_X - (f_NormVecX * f_DistToCourse);
            f_RefCourseDistY = (f_Y - (f_NormVecY * f_DistToCourse)) + f_Radius;
        }
    } else {
        /* use old parabolic approximation for wide curves and distance in Y-
         * Direction*/
        f_RefCourseDistX = f_X;
        f_RefCourseDistY = SQR(f_X) * (f_C0) * (0.5f);
        f_DistToCourse = (f_Y - f_RefCourseDistY);
        /*instead of integral 0 to fX of function SQRT(1+(fC0*x)^2) dx*/
        f_DistOnCourse = f_X;
    }
    ReferencePoint.f_X = f_RefCourseDistX;
    ReferencePoint.f_Y = f_RefCourseDistY;
    ReferencePoint.f_DistToTraj = f_DistToCourse;
    ReferencePoint.f_DistOnTraj = f_DistOnCourse;

    return ReferencePoint;
}

uint8 Tue_CML_Bayes2(uint8 u_ProbabilityA,
                     uint8 u_ProbabilityB,
                     const uint8 a_CPT[CML_BAYES2_CPT_SIZE]) {
    uint32 u_temp;
    uint32 u_p2;
    uint8 u_ProbabilityNotA, u_ProbabilityNotB;

    /* range check: probability must not be greater than 100% */
    if (u_ProbabilityA > Percentage_max) {
        u_ProbabilityA = Percentage_max;
    }
    if (u_ProbabilityB > Percentage_max) {
        u_ProbabilityB = Percentage_max;
    }

    u_ProbabilityNotA = Percentage_max - u_ProbabilityA;
    u_ProbabilityNotB = Percentage_max - u_ProbabilityB;

    /* Calculate the probability P(C) by the formula of total probability. */
    /* The events A and B are assumed independent, so P(A, B) = P(A) * P(B) */
    u_temp = (uint32)(u_ProbabilityNotA) * (uint32)(u_ProbabilityNotB) *
             (uint32)a_CPT[0];
    u_temp += ((uint32)(u_ProbabilityA) * (uint32)(u_ProbabilityNotB) *
               (uint32)a_CPT[1]);
    u_temp += ((uint32)(u_ProbabilityNotA) * (uint32)(u_ProbabilityB) *
               (uint32)a_CPT[2]);
    u_temp += ((uint32)(u_ProbabilityA) * (uint32)(u_ProbabilityB) *
               (uint32)a_CPT[3]);

    /* fixed point scaling */
    u_p2 = Percentage_max * Percentage_max;
    u_temp += (u_p2 / 2u);
    u_temp /= u_p2;
    if (u_temp > Percentage_max) {
        u_temp = Percentage_max;
    }

    return (uint8)u_temp;
}

/*****************************************************************************
  Functionname:    Tue_CML_BoundedLinInterpol                             */ /*!

    @brief           bounded linear interpolation between two given points

    @description     This function computes the bounded linear interpolation
  value
                     between two given points. The minimum and maximum boundary
                     values are taken from the input structure.

    @param[in]       p_Params : structure for parameters
                                Range for p_Params->dAmin is [Full range of
  float32]
                                Range for p_Params->dAmax is [Full range of
  float32]
                                Range for p_Params-> dM is [Full range of
  float32]
                                Range for p_Params-> dB is [Full range of
  float32]
                                Overflow may occur if all the input values to
  the function
                                are at maximum possible value at the same time.

    @param[in]       f_Value : x-value to interpolate
                               [Full range of float32]

    @return          the bounded interpolated value


  *****************************************************************************/
float32 Tue_CML_BoundedLinInterpol(BML_t_LinFunctionArgs const* const p_Params,
                                   const float32 f_Value) {
    const float32 f_min = p_Params->dAmin;
    const float32 f_max = p_Params->dAmax;

    /* Geradengleichung: */
    float32 f_BoundedValue =
        TUE_CML_MultAdd(p_Params->dM, f_Value, p_Params->dB);

    /* Grenzwerte: */
    if (f_min < f_max) {
        /*    /-- */
        /* --/    */
        if (f_BoundedValue <= f_min) {
            f_BoundedValue = f_min;
        } else if (f_BoundedValue > f_max) {
            f_BoundedValue = f_max;
        } else {
        }
    } else {
        /* --\    */
        /*    \-- */
        if (f_BoundedValue <= f_max) {
            f_BoundedValue = f_max;
        } else if (f_BoundedValue > f_min) {
            f_BoundedValue = f_min;
        } else {
        }
    }

    return f_BoundedValue;
} /* Tue_CML_BoundedLinInterpol() */

/*****************************************************************************
  Functionname:    Tue_CML_LowPassFilter2                                 */ /*!

    @brief           simple first order lowpass filter

    @description     This function is an implementation of simple first order
                     lowpass filter. This determines the output sample in terms
  of
                     the input sample and preceding output.
                     So if x = input, y = output and z = previous output, and
                     a = filter coefficient, then,
                     y = (a*x) + ((1-a)*z)

    @param[in,out]   f_Old :   old value (in), filtered value (out)
                               Valid float pointer.
                               Supported value for data [Full range of float32]
    @param[in]       f_New :   new value
                               Supported value [Full range of float32]
    @param[in]       f_Alpha : filter coefficient
                               Optimal Values [0,..,1]

    @return          none


  *****************************************************************************/
void Tue_CML_LowPassFilter2(float32* f_Old, float32 f_New, float32 f_Alpha) {
    float32 f_Dummy;

    f_Dummy = (f_Alpha * f_New) + ((1.f - f_Alpha) * (*f_Old));
    *f_Old = f_Dummy;
}

/*****************************************************************************
  Functionname:    Tue_CML_BoundedLinInterpol2                            */
float32 Tue_CML_BoundedLinInterpol2(float32 f_IVal,
                                    float32 f_Imin,
                                    float32 f_Imax,
                                    float32 f_Omin,
                                    float32 f_Omax) {
    float32 f_OVal;

    if (TUE_CML_IsZero(f_Imax - f_Imin)) {
        f_OVal = (f_Omin + f_Omax) * 0.5f;
    } else {
        f_OVal = f_Omin +
                 ((f_IVal - f_Imin) * ((f_Omax - f_Omin) / (f_Imax - f_Imin)));
    }

    /* Bound output */
    if (f_Omin < f_Omax) {
        f_OVal = (TUE_CML_MinMax(f_Omin, f_Omax, f_OVal));
    } else {
        f_OVal = (TUE_CML_MinMax(f_Omax, f_Omin, f_OVal));
    }
    return f_OVal;
} /* Tue_CML_BoundedLinInterpol2() */

/* ****************************************************************************

  Functionname:     TUE_CML_EMPCalcVariance                               */ /*!

     @brief        Calculate Variance of the given numbers stored in an array

     @param[in]    afValues                 Array values
     @param[in]    uiArraySize              Array size

     @return       variance of given numbers


   ****************************************************************************
   */
float32 TUE_CML_EMPCalcVariance(const float32 afValues[], uint16 uiArraySize) {
    float32 fSumforAverage = 0;
    float32 fAverage = 0;
    float32 fSumforVariance = 0;
    float32 fVariance = 0;
    uint16 uiCurrID = 0u;

    for (uiCurrID = 0u; uiCurrID < uiArraySize; ++uiCurrID) {
        fSumforAverage += afValues[uiCurrID];
    }

    fAverage = fSumforAverage / (float32)uiArraySize;

    for (uiCurrID = 0u; uiCurrID < uiArraySize; ++uiCurrID) {
        fSumforVariance += SQR(afValues[uiCurrID] - fAverage);
    }

    fVariance = fSumforVariance / (uiArraySize - 1);

    return fVariance;
}

/* PRQA S 0759 1 */ /* Union required to define NaN */
LOCAL CONST(TueObjFusn_f32UnionType,
            ObjFusn_CONST) uf32NaN = {TUE_PRV_FUSION_MATH_FLOAT32_NAN_DEFAULT};

/* PRQA S 1503 2 */ /* Library Function */
float32 tue_prv_em_cos(const float32 f32x) {
    /* PRQA S 3492 1 */ /* Tertiary operator outside of macro: function is
                           inlined */
    return tue_prv_fusion_sin_impl(((f32x < FLT_ZERO) ? -f32x : f32x), 1);
}

/* PRQA S 1503 2 */ /* Library Function */
float32 tue_prv_em_sin(const float32 f32x) {
    return tue_prv_fusion_sin_impl(f32x, 0);
}

/* PRQA S 1505 2 */ /* Internal function */
uint32 tue_prv_em_exp(const float32 f32x) {
    /* Shift by number of mantissa bits, exponent is now on bit position 0 to 7,
     * sign bit is on position 8 */
    /* PRQA S 0310 1 */ /* Cast to Union type required */
    return (((((const TueObjFusn_f32UnionType*)&f32x)->u32Val) >>
             TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_NUM_BITS) &
            TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK_INF);
}

/* PRQA S 1505 2 */ /* Internal function */
boolean tue_prv_em_isNaN(const float32 f32x) {
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
float32 tue_prv_em_copysign(const float32 f32x, const float32 f32Sign) {
    /* PRQA S 0759 3 */ /* Union required here */
    TueObjFusn_f32UnionType uf32UnionX;
    /* PRQA S 3204 */ /* union members are modified */

    uf32UnionX.f32Val = f32x;

    uf32UnionX.u32Val &=
        0x7FFFFFFFu;    // (TUE_PRV_FUSION_MATH_FLOAT32_MANTISSA_MASK |
                        // TUE_PRV_FUSION_MATH_FLOAT32_EXP_MASK);
    /* PRQA S 0310 1 */ /* Cast to Union type required */
    uf32UnionX.u32Val |= ((((const TueObjFusn_f32UnionType*)&f32Sign)->u32Val) &
                          TUE_PRV_FUSION_MATH_FLOAT32_SIGN_BIT_MASK);

    return uf32UnionX.f32Val;
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
    const boolean bIsNan = tue_prv_em_isNaN(f32x);

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
        *pf32IntVal = tue_prv_em_copysign(FLT_ZERO, f32x);
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
        f32Res = tue_prv_em_copysign(FLT_ZERO, f32x);
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
        f32Res = tue_prv_em_copysign(f32x - (*pf32IntVal), f32x);
    }

    return f32Res;
}

/* PRQA S 1505 2 */ /* Internal Function */
LOCAL float32 tue_prv_fusion_sin_impl(float32 f32x, sint32 s32Quad) {
    float32 f32y;
    float32 f32Tmp;
    uint32 u32Tmp;

    const uint32 u32Exp = tue_prv_em_exp(f32x);
    const boolean bIsNan = tue_prv_em_isNaN(f32x);

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
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */