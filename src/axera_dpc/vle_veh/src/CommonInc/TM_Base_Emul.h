

/**
@defgroup ecu_emul ECU_EMUL (Emulation of ECU Math Routines)
  @ingroup cml
@{ */

/*****************************************************************************
  QA-C
*****************************************************************************/

#ifdef PRQA_SIZE_T
/* Switch off QA-C warnings on side effects for functions, which don't have
   any side effects and will sure never have some. */
#pragma PRQA_NO_SIDE_EFFECTS BML_s_Round2Int_MPC5675K_Emu
#pragma PRQA_NO_SIDE_EFFECTS BML_u_Round2Uint_MPC5675K_Emu
#pragma PRQA_NO_SIDE_EFFECTS BML_f_MultAdd_MPC5675K_Emu
#pragma PRQA_NO_SIDE_EFFECTS BML_f_Sqrt_MPC5675K_Emu
#pragma PRQA_NO_SIDE_EFFECTS BML_f_AbsGen
#pragma PRQA_NO_SIDE_EFFECTS BML_f_MinGen
#pragma PRQA_NO_SIDE_EFFECTS BML_f_MaxGen
#pragma PRQA_NO_SIDE_EFFECTS BML_f_Round2FloatGen
#pragma PRQA_NO_SIDE_EFFECTS BML_s_Round2IntGen
#pragma PRQA_NO_SIDE_EFFECTS BML_u_Round2UintGen
#pragma PRQA_NO_SIDE_EFFECTS BML_f_Floor2FloatGen
#pragma PRQA_NO_SIDE_EFFECTS BML_f_Ceil2FloatGen
#pragma PRQA_NO_SIDE_EFFECTS BML_f_MultAddGen
#endif

/*****************************************************************************
  INCLUDE PROTECTION
*****************************************************************************/

/* allow inclusion of cml sub-headers only if external cml header is included */
#ifndef _BML_EXT_INCLUDED
#endif /* #ifdef _BML_EXT_INCLUDED */

#ifndef _BML_ECU_EnvmUL_INCLUDED
#define _BML_ECU_EnvmUL_INCLUDED

/* include glob_type.h which contains HW accelerated functions only in the
 * target build */
#if (defined(__DVLC__))
//#include "TM_Global_Types.h"
#endif

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#if defined(_MSC_VER) /* Microsoft compiler -> code only for simulation */
#if (defined(CPU_MPC5675K_EMU) ||                                     \
     defined(CPU_MPC5775N_EMU)) /* emulation of Freescale Golddust or \
                                   Racerunner ECU specific functions */
#include <math.h> /* use the standard sqrt to emulate the ECU specific sqrt in the simulation */
#endif /* ECU switch for Microsoft compiler */
#endif /* compiler switch */

/*****************************************************************************
  CONSTANTS
*****************************************************************************/

/* Defines for inlining functions*/
#if defined(_MSC_VER) /* Microsoft compiler -> code only for simulation */
#define BML_INLINE __inline

#elif defined(__GNUC__) /**GNU C compiler -> code for CortexA8 per gcc */
#define BML_INLINE static inline

#elif (defined(__POWERPC__) &&                                                \
       defined(__MWERKS__)) /* Freescale Metrowerks compiler for PowerPC-ECUs \
                               -> code only for ECU (e.g. ARS301) */
#define BML_INLINE inline

#elif (defined(__DVLC__))
#define BML_INLINE inline

#elif defined(_TMS320C6X)
#define BML_INLINE inline

#elif (                                                                      \
    defined(__STDC_VERSION__)) /* C99 compatible compiler has to have inline \
                                  keyword with proper non-extern linkage */
#if (__STDC_VERSION__ >= 199901)
#define BML_INLINE inline
#else /* #if (__STDC_VERSION__ >= 199901) */
#define BML_INLINE static inline
#endif /* #if/else (__STDC_VERSION__ >= 199901) */

#else /* unknown compiler -> no INLINE */
#define BML_INLINE static
#endif

/*****************************************************************************
  MACROS
*****************************************************************************/

#if 1  // and by liuyang 2019-03-07 for debug
extern float32 BML_f_SqrtApprox(float32 f_radicand);

BML_INLINE float32 BML_f_AbsGen(float32 x);
BML_INLINE float32 BML_f_AbsGen(float32 x) { return (x < 0.f) ? -x : x; }

// PRQA S 3406 2
BML_INLINE float32 BML_f_MinGen(float32 x, float32 y);
BML_INLINE float32 BML_f_MinGen(float32 x, float32 y) {
    return (x < y) ? x : y;
}

// PRQA S 3406 2
BML_INLINE float32 BML_f_MaxGen(float32 x, float32 y);
BML_INLINE float32 BML_f_MaxGen(float32 x, float32 y) {
    return (x > y) ? x : y;
}

// PRQA S 3406 2
BML_INLINE float32 BML_f_Round2FloatGen(float32 x);
BML_INLINE float32 BML_f_Round2FloatGen(float32 x) {
    return (x >= 0.f) ? (float32)(sint32)(x + 0.5f)
                      : (float32)(sint32)(x - 0.5f);
}

// PRQA S 3406 2
BML_INLINE sint32 BML_s_Round2IntGen(float32 x);
BML_INLINE sint32 BML_s_Round2IntGen(float32 x) {
    return (x >= 0.f) ? (sint32)(x + 0.5f) : (sint32)(x - 0.5f);
}

// PRQA S 3406 2
BML_INLINE uint32 BML_u_Round2UintGen(float32 x);
BML_INLINE uint32 BML_u_Round2UintGen(float32 x) {
    // BML_ASSERT(x >= 0.f); /* PRQA S 3112 */
    return (uint32)(x + 0.5f);
}

/*! Generic function to compute floor of positive and negative numbers. Works
 * only for x in the range [-2147483647 ... 2147483647] */
// PRQA S 3406 2
BML_INLINE float32 BML_f_Floor2FloatGen(float32 f_x);
BML_INLINE float32 BML_f_Floor2FloatGen(float32 f_x) {
    return (((f_x - (float32)((sint32)f_x)) >= 0.0f)
                ? ((float32)((sint32)f_x))
                : ((float32)((sint32)f_x) - 1.0f));
}

/*! Generic function to compute ceil of positive and negative numbers. Works
 * only for x in the range [-2147483647 ... 2147483647] */

// PRQA S 3406 2
BML_INLINE float32 BML_f_Ceil2FloatGen(float32 f_x);
BML_INLINE float32 BML_f_Ceil2FloatGen(float32 f_x) {
    return (((f_x - (float32)((sint32)f_x)) > 0.0f)
                ? ((float32)((sint32)f_x) + 1.0f)
                : ((float32)((sint32)f_x)));
}

// PRQA S 3406 2
BML_INLINE float32 BML_f_MultAddGen(float32 a, float32 b, float32 d);
BML_INLINE float32 BML_f_MultAddGen(float32 a, float32 b, float32 d) {
    return (a * b) + d;
}
#endif

#if defined(_MSC_VER) /* Microsoft compiler -> code only for simulation */

/*! absolute value for floating point */
#define BML_f_Abs(x) BML_f_AbsGen(x)

/*! minimum of two values */
#define BML_f_Min(x, y) BML_f_MinGen(x, y)

/*! maximum of two values */
#define BML_f_Max(x, y) BML_f_MaxGen(x, y)

/*! Negative number */
#define BML_f_Neg(x) (-(x))

#if (defined(CPU_MPC5675K_EMU) ||                                     \
     defined(CPU_MPC5775N_EMU)) /* emulation of Freescale Golddust or \
                                   Racerunner ECU specific functions */

/*! rounds floating point and converts to signed integer */
#define BML_s_Round2Int(x) BML_s_Round2Int_MPC5675K_Emu(x)

/*! rounds floating point and converts to unsigned integer */
#define BML_u_Round2Uint(x) BML_u_Round2Uint_MPC5675K_Emu(x)

/*! floating point multiply and add: (a * b) + d */
#define BML_f_MultAdd(a, b, d) BML_f_MultAdd_MPC5675K_Emu(a, b, d)

/*! square root */
#define BML_f_Sqrt(x) BML_f_Sqrt_MPC5675K_Emu(x)

/*! Rounds of positive and negative numbers */
#define BML_Round(x) (float32) BML_s_Round2Int_MPC5675K_Emu(x)

/*! Computes floor of positive and negative numbers. Works only for x in the
 * range [-2147483647 ... 2147483647] */
#define BML_f_Floor(x) (float32) BML_s_Round2Int_MPC5675K_Emu(x - 0.5f)

/*! Computes ceiling of positive and negative numbers. Works only for x in the
 * range [-2147483647 ... 2147483647] */
#define BML_f_Ceil(x) (float32) BML_s_Round2Int_MPC5675K_Emu(x + 0.5f)

#else /* default simulation code (none of the above listed ECU emulations) */

/*! rounds floating point and converts to signed integer */
#define BML_s_Round2Int(x) BML_s_Round2IntGen(x)

/*! rounds floating point and converts to unsigned integer */
#define BML_u_Round2Uint(x) BML_u_Round2UintGen(x)

/* floating point multiply and add: (a * b) + d */
#define BML_f_MultAdd(a, b, d) BML_f_MultAddGen(a, b, d)

/*! square root */
#define BML_f_Sqrt(x) BML_f_SqrtApprox(x)

/*! Rounds of positive and negative numbers */
#define BML_Round(x) BML_f_Round2FloatGen(x)

/*! Computes floor of positive and negative numbers. Works only for x in the
 * range [-2147483647 ... 2147483647] */
#define BML_f_Floor(x) BML_f_Floor2FloatGen(x)

/*! Computes ceiling of positive and negative numbers. Works only for x in the
 * range [-2147483647 ... 2147483647] */
#define BML_f_Ceil(x) BML_f_Ceil2FloatGen(x)

#endif /* ECU switch for Microsoft compiler */

#elif (defined(__DVLC__))

/*! absolute value for floating point */
#define BML_f_Abs(x) fABS_HW(x)

/*! minimum of two values */
#define BML_f_Min(x, y) MIN_FLOAT_HW(x, y)

/*! maximum of two values */
#define BML_f_Max(x, y) MAX_FLOAT_HW(x, y)

/*! Rounds of positive and negative numbers */
#define BML_Round(x) (float32) ROUND_TO_INT_HW(x)

/*! Computes floor of positive and negative numbers. Works only for x in the
 * range [-2147483647 ... 2147483647] */
#define BML_f_Floor(x) (float32) ROUND_TO_INT_HW(x - 0.5f)

/*! Computes ceiling of positive and negative numbers. Works only for x in the
 * range [-2147483647 ... 2147483647] */
#define BML_f_Ceil(x) (float32) ROUND_TO_INT_HW(x + 0.5f)

/*! rounds floating point and converts to signed integer */
#define BML_s_Round2Int(x) ROUND_TO_INT_HW(x)

/*! rounds floating point and converts to unsigned integer */
#define BML_u_Round2Uint(x) ROUND_TO_UINT_HW(x)

/*! floating point multiply and add: (a * b) + d */
#define BML_f_MultAdd(a, b, d) MUL_ADD_FLOAT_HW(a, b, d)

/*! square root */
#define BML_f_Sqrt(x) SQRT_HW(x)

/*! Negative number */
#define BML_f_Neg(x) (NEG_FLOAT_HW(x))

#else /* other compilers */

/*! absolute value for floating point */
#define BML_f_Abs(x) BML_f_AbsGen(x)

/*! minimum of two values */
#define BML_f_Min(x, y) BML_f_MinGen(x, y)

/*! maximum of two values */
#define BML_f_Max(x, y) BML_f_MaxGen(x, y)

/*! Rounds of positive and negative numbers */
#define BML_Round(x) BML_f_Round2FloatGen(x)

/*! Computes floor of positive and negative numbers. Works only for x in the
 * range [-2147483647 ... 2147483647] */
#define BML_f_Floor(x) BML_f_Floor2FloatGen(x)

/*! Computes ceiling of positive and negative numbers. Works only for x in the
 * range [-2147483647 ... 2147483647] */
#define BML_f_Ceil(x) BML_f_Ceil2FloatGen(x)

/*! rounds floating point and converts to signed integer */
#define BML_s_Round2Int(x) BML_s_Round2IntGen(x)

/*! rounds floating point and converts to unsigned integer */
#define BML_u_Round2Uint(x) BML_u_Round2UintGen(x)

/* floating point multiply and add: (a * b) + d */
#define BML_f_MultAdd(a, b, d) BML_f_MultAddGen(a, b, d)

/*! square root */
#define BML_f_Sqrt(x) BML_f_SqrtApprox(x)

/*! Negative number */
#define BML_f_Neg(x) (-(x))

#endif

/*****************************************************************************
  INLINE FUNCTIONS
*****************************************************************************/

#if defined(_MSC_VER) /* Microsoft compiler -> code only for simulation */

#if (defined(CPU_MPC5675K_EMU) ||                                     \
     defined(CPU_MPC5775N_EMU)) /* emulation of Freescale Golddust or \
                                   Racerunner ECU specific functions */

BML_INLINE sint32 BML_s_Round2Int_MPC5675K_Emu(float32 x) {
    __int32 retval;

    if (x < (float32)(1u << 31)) {
        if (x > -(float32)(1u << 31)) {
            __asm fld x __asm fistp retval
        } else {
            /* Case RD 32: 63 = 0x80000000, overflow max negative */
            retval = (1 << 31);
        }
    } else {
        /* 0x7FFFFFFF, overflow max positive */
        retval = 0x7FFFFFFF;
    }
    return retval;
}

BML_INLINE uint32 BML_u_Round2Uint_MPC5675K_Emu(float32 x) {
    uint32 retval = 0UL;

    if (x >= 0.f) {
        if (x < (float32)0xFFFFFFFFu) {
            if (x < (float32)0x80000000u) {
                __asm fld x __asm fistp retval
            } else {
                x -= (float32)0x80000000u;
                __asm fld x __asm fistp retval retval += 0x80000000u;
            }
        } else {
            retval = 0xFFFFFFFFu;
        }
    } else {
        /* PowerPC returns 0, but as this is implementation defined according to
        ISO-IEC C, this shouldn't be relied on. An assertion would be a good
        idea here */
        retval = 0u;
    }
    return retval;
}

BML_INLINE float32 BML_f_MultAdd_MPC5675K_Emu(float32 fA,
                                              float32 fB,
                                              float32 fD) {
    typedef union {
#pragma warning(push)
#pragma warning(disable : 4214)
        struct {
            uint32 mantissa : 23;
            uint32 exponent : 8;
            uint32 sign : 1;
        } _bits;
#pragma warning(pop)
        uint32 _ui32;
        float32 _float;
    } float_rep_t;
    float_rep_t fa, fb, fd;
    float32 fTemp;

    fa._float = fA;
    fb._float = fB;
    fd._float = fD;
    if ((fa._bits.exponent == 0) || (fb._bits.exponent == 0)) {
        /* Denormalized number : intermediate product properly signed zero
        Final result : added to corresponding element of RD */
        if (fd._bits.exponent >= 255) {
            if (fd._bits.sign == 0) {
                fd._ui32 = 0x7F7FFFFFu;
            } else {
                fd._ui32 = 0xFF7FFFFFu;
            }
            fTemp = fd._float;
        } else if (fd._bits.exponent == 0) {
            /* The result would be a denormalized number */
            fTemp = 0.f;
        } else {
            /* The result is 'fD' */
            fTemp = fD;
        }
    } else if ((fa._bits.exponent >= 255) || (fb._bits.exponent >= 255)) {
        /* Infinity / NaN */
        if (fa._bits.sign == fb._bits.sign) {
            fa._ui32 = 0x7F7FFFFFu;
            fTemp = fa._float;
        } else {
            fa._ui32 = 0xFF7FFFFFu;
            fTemp = fa._float;
        }
    } else {
        if (fd._bits.exponent >= 255u) {
            /* fD is NaN/infinity */
            if (fd._bits.sign == 0) {
                fd._ui32 = 0x7F7FFFFFu;
            } else {
                fd._ui32 = 0xFF7FFFFFu;
            }
            fTemp = fd._float;
        } else if ((fa._bits.exponent + fb._bits.exponent) >= (255u + 254u)) {
            /* fA * fB would overflow */
            if (fa._bits.sign == fb._bits.sign) {
                fd._ui32 = 0x7F7FFFFFu;
            } else {
                fd._ui32 = 0xFF7FFFFFu;
            }
            fTemp = fd._float;
        } else {
            /* Normal fA, fB : intermediate result is the multiplication
            Use SSE2 instruction set to be independent of FPU precission */
            __asm movss xmm0, fA __asm movss xmm1, fB __asm cvtps2pd xmm0,
                xmm0 __asm cvtps2pd xmm1, xmm1 __asm mulsd xmm0,
                xmm1 __asm movss xmm1, fD __asm cvtps2pd xmm1,
                xmm1 __asm addsd xmm0, xmm1 __asm cvtpd2ps xmm0,
                xmm0 __asm movss fTemp, xmm0
            /* fTemp = (fA * fB) + fD; */
        }
    }
    return fTemp;
}

BML_INLINE float32 BML_f_Sqrt_MPC5675K_Emu(float32 x) {
    /* Call the standard sqrt from math.h but return 0 instead of IND for
     * negative radicand */

    float32 fTemp;

    BML_ASSERT(x >= 0.f);

    if (x <= 0.f) {
        fTemp = 0.f;
    } else {
        fTemp = (float32)sqrt(x);
    }
    return fTemp;
}

#endif /* ECU switch for Microsoft compiler */

#endif /* compiler switch */

#endif /* #ifndef _BML_ECU_EnvmUL_INCLUDED */

/** @} end defgroup */
