/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#include "ved_rt_matrixlib.h"
#include "string.h"
#include "tue_common_libs.h"

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/* Function: rt_BackwardSubstitutionRR_Sgl =====================================
 * Abstract: Backward substitution: Solving Ux=b
 *           U: real, double
 *           b: real, double
 *           U is an upper (or unit upper) triangular full matrix.
 *           The entries in the lower triangle are ignored.
 *           U is a NxN matrix
 *           X is a NxP matrix
 *           B is a NxP matrix
 */
void rt_BackwardSubstitutionRR_Sgl(real32_T* pU,
                                   const real32_T* pb,
                                   real32_T* x,
                                   int_T N,
                                   int_T P,
                                   boolean_T unit_upper) {
    int_T i, k;

    for (k = P; k > 0; k--) {
        real32_T* pUcol = pU;
        for (i = 0; i < N; i++) {
            real32_T* xj = x + k * N - 1;
            real32_T s = 0.0;
            real32_T* pUrow = pUcol--; /* access current row of U */

            {
                int_T j = i;
                while (j-- > 0) {
                    s += *pUrow * *xj--;
                    pUrow -= N;
                }
            }

            if (unit_upper) {
                *xj = *pb-- - s;
            } else {
                *xj = (*pb-- - s) / *pUrow;
            }
        }
    }
}

/* Function: rt_ForwardSubstitutionRR_Sgl ======================================
 * Abstract: Forward substitution: solving Lx=b
 *           L: Real, double
 *           b: Real, double
 *           L is a lower (or unit lower) triangular full matrix.
 *           The entries in the upper triangle are ignored.
 *           L is a NxN matrix
 *           X is a NxP matrix
 *           B is a NxP matrix
 */
void rt_ForwardSubstitutionRR_Sgl(real32_T* pL,
                                  const real32_T* pb,
                                  real32_T* x,
                                  int_T N,
                                  int_T P,
                                  const int32_T* piv,
                                  boolean_T unit_lower) {
    /* Real inputs: */
    int_T i, k;
    for (k = 0; k < P; k++) {
        real32_T* pLcol = pL;
        for (i = 0; i < N; i++) {
            real32_T* xj = x + k * N;
            real32_T s = 0.0;
            real32_T* pLrow = pLcol++; /* access current row of L */

            {
                int_T j = i;
                while (j-- > 0) {
                    s += *pLrow * *xj++;
                    pLrow += N;
                }
            }

            if (unit_lower) {
                *xj = pb[piv[i]] - s;
            } else {
                *xj = (pb[piv[i]] - s) / *pLrow;
            }
        }
        pb += N;
    }
}

void rt_lu_real_sgl(real32_T* A,   /* in and out                         */
                    const int_T n, /* number or rows = number of columns */
                    int32_T* piv)  /* pivote vector                      */
{
    int_T k;

    /* initialize row-pivot indices: */
    for (k = 0; k < n; k++) {
        piv[k] = k;
    }

    /* Loop over each column: */
    for (k = 0; k < n; k++) {
        const int_T kn = k * n;
        int_T p = k;

        /* Scan the lower triangular part of this column only
         * Record row of largest value
         */
        {
            int_T i;
            real32_T Amax =
                (real32_T)fABS((real_T)A[p + kn]); /* assume diag is max */
            for (i = k + 1; i < n; i++) {
                real32_T q = (real32_T)fABS((real_T)A[i + kn]);
                if (q > Amax) {
                    p = i;
                    Amax = q;
                }
            }
        }

        /* swap rows if required */
        if (p != k) {
            int_T j;
            int32_T t1;
            for (j = 0; j < n; j++) {
                real32_T t;
                const int_T j_n = j * n;
                t = A[p + j_n];
                A[p + j_n] = A[k + j_n];
                A[k + j_n] = t;
            }
            /* swap pivot row indices */
            t1 = piv[p];
            piv[p] = piv[k];
            piv[k] = t1;
        }

        /* column reduction */
        {
            real32_T Adiag = A[k + kn];

            if (Adiag != 0.0F) { /* non-zero diagonal entry */
                int_T i, j;
                /* divide lower triangular part of column by max */
                Adiag = 1.0F / Adiag;
                for (i = k + 1; i < n; i++) {
                    A[i + kn] *= Adiag;
                }

                /* subtract multiple of column from remaining columns */
                for (j = k + 1; j < n; j++) {
                    int_T j_n = j * n;
                    for (i = k + 1; i < n; i++) {
                        A[i + j_n] -= A[i + kn] * A[k + j_n];
                    }
                }
            }
        }
    }
}
/*
 * Function: rt_MatDivRR_Sgl
 * Abstract:
 *      2-real double input matrix division function
 */
void rt_MatDivRR_Sgl(real32_T* Out,
                     const real32_T* In1,
                     const real32_T* In2,
                     real32_T* lu,
                     int32_T* piv,
                     real32_T* x,
                     const int_T dims[3]) {
    int_T N = dims[0];
    int_T N2 = N * N;
    int_T P = dims[2];
    int_T NP = N * P;
    const boolean_T unit_upper = FALSE;
    const boolean_T unit_lower = TRUE;

    (void)memcpy(lu, In1, N2 * sizeof(real32_T));

    rt_lu_real_sgl(lu, N, piv);

    rt_ForwardSubstitutionRR_Sgl(lu, In2, x, N, P, piv, unit_lower);

    rt_BackwardSubstitutionRR_Sgl(lu + N2 - 1, x + NP - 1, Out, N, P,
                                  unit_upper);
}

/*
 * Function: rt_MatMultRR_Sgl
 * Abstract:
 *      2-input matrix multiply function
 *      Input 1: Real, single-precision
 *      Input 2: Real, single-precision
 */
void rt_MatMultRR_Sgl(real32_T* y,
                      const real32_T* A,
                      const real32_T* B,
                      const int_T dims[3]) {
    int_T k;
    for (k = dims[2]; k-- > 0;) {
        const real32_T* A1 = A;
        int_T i;
        for (i = dims[0]; i-- > 0;) {
            const real32_T* A2 = A1;
            const real32_T* B1 = B;
            real32_T acc = (real32_T)0.0;
            int_T j;
            A1++;
            for (j = dims[1]; j-- > 0;) {
                acc += *A2 * *B1;
                B1++;
                A2 += dims[0];
            }
            *y++ = acc;
        }
        B += dims[1];
    }
}

/************************************************************************
  Functionname:    VED_GDBexp                                          */ /*!

  @brief           Fast approximation exponential function

  @description     This function does a fast approximation of the
                   exponential function with the help of some predetermined
                   constant coefficients.

  @param[in]       f_power :  any number
                              Supported values [Full range of float32]

  @return          Approximation of exp(f_power)

  @pre             x < 80, IEEE754 Floating Point format

  @post            Postcondition: none
**************************************************************************** */
float32 VED_GDBexp(float32 f_power) {
    float32 f_exp; /* Function result value */
    /* handling of out-of-range arguments */
    if (f_power < -80.0F) {
        f_exp = 0.0F;
    } else {
        sint32 s_2base_exp = 0L; /* Integer part for two base exponent */
        sint32 s_sign;           /* Sign bit: xsb=0 -> x>=0, xsb=1 -> x<0 */
        uint32 u_iVal;           /* Integer interpretation of float value */
        /* Limit input. */
        f_power = MIN(f_power, 80.0f);

        /* Reinterprete value as Dword */
        {
            const uint32* ui_pointer = (uint32*)(&f_power);
            u_iVal = *ui_pointer;
        }

        uint32 ui_tmp = (u_iVal >> (uint32)31UL) & (uint32)1UL;
        s_sign = (sint32)ui_tmp; /* extract sign bit of input value */
        u_iVal &= (uint32)0x7FFFFFFFUL;

        /* Test if argument is outside the desired reduction range */
        if (u_iVal > 0x3EB17218UL) {
            /* Natural logarithm of 2 as high and low value to increase accuracy
             * for both sign. */
            const float32 a_ln2HiPrt_c[2] = {+6.9313812256e-01F,
                                             -6.9313812256e-01F};
            const float32 a_ln2LoPrt_c[2] = {+9.0580006145e-06F,
                                             -9.0580006145e-06F};

            float32 f_hi = 0.F; /* High part of reduced value */
            float32 f_lo = 0.F; /* Low part of reduced value */

            if (u_iVal < 0x3F851592UL) {
                /*  Simple reduction via addition or substraction */
                f_hi = f_power - a_ln2HiPrt_c[s_sign];
                f_lo = a_ln2LoPrt_c[s_sign];
                s_2base_exp = 1L - (s_sign + s_sign);
            } else {
                /* Complete reduction is necessary */
                const float32 f_invln2_c = 1.4426950216F; /* 1/ln2 */
                const float32 a_halF_c[2] = {0.5F, -0.5F};

                /* Reduce to 0..ln2 and shift to -0.5*ln2 .. +0.5*ln2 */
                float32 f_tmp = (f_invln2_c * f_power) + a_halF_c[s_sign];
                s_2base_exp = (sint32)f_tmp;
                f_hi = f_power - (((float32)s_2base_exp) * a_ln2HiPrt_c[0]);
                f_lo = ((float32)s_2base_exp) * a_ln2LoPrt_c[0];
            }
            f_power = f_hi - f_lo; /* Combine both parts */
        } else {
            /* Input argument is already within reduction range.  */
            s_2base_exp = 0L;
        }

        /* x is now in primary range */
        { /*  Approximation of exp(r) by a polynom on the interval
             [-0.34658,+0.34658] */
            const float32 f_a1_c = 0.0013793724F;
            const float32 f_a2_c = 0.0083682816F;
            const float32 f_a3_c = 0.0416686266F;
            const float32 f_a4_c = 0.1666652424F;
            const float32 f_a5_c = 0.4999999297F;

            /* Calculate polynom with horner schema */
            f_exp =
                (((((((((((f_power * f_a1_c) + f_a2_c) * f_power) + f_a3_c) *
                       f_power) +
                      f_a4_c) *
                     f_power) +
                    f_a5_c) *
                   f_power) +
                  1.F) *
                 f_power) +
                1.F;
        }

        /*  Scale back to obtain exp(x) = 2^k * exp(r) */
        {
            const uint32* ui_pointer = (uint32*)(&f_exp);
            u_iVal = *ui_pointer;
        }
        u_iVal += ((*((uint32*)&(s_2base_exp))) << 23UL);
        {
            const float32* f_pointer = (float32*)(&u_iVal);
            f_exp = *f_pointer;
        }
    } /* if/else (x < -80.0F) */

    return f_exp;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */