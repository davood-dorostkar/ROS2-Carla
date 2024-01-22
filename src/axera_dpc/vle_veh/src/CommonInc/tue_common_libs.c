/*! \file **********************************************************************

  PROJECT:                   COMMON

  CPU:                       CPU-Independent

  COMPONENT:                 COMMON TOOLS

  MODULNAME:                 tue_common_libs.c

  DESCRIPTION:               common libs for tue development

  AUTHOR:                    $Author: tao.guo

  CREATION DATE:             $Date: 2020/07/04

  VERSION:                   $Revision: 1.0.0


  CHANGES:
  ---*/ /*---
  CHANGE:                    $Log: tue_common_libs.c
  CHANGE:                    Initial version

**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "tue_common_libs.h"

/*****************************************************************************
  Functionname:    TUE_CML_CalculatePolygonValue2D                          */ /*!

  @brief           Calculates the value of a polygon at a specific position

  @description     This function calculates the value of a polygon at a given
                   position. The polygon is specified by the 2D sampling points
                   (X,Y). The X-position is given by fInputValue.
                   The function returns the Y-Value at the given X-Position
                   based on the provided sampling points.

  @param[in]       s_NrOfTableRows : number of samplings points in parameter pTable
                   This value must be a positive integer greater than or equal to 1.
  @param[in]       a_Table : pointer to the sampling points. Note: has to be
                   sorted in X (X ascending)!
                   Range for a_Table[].f0 is [Full range of float32]
                   Range for a_Table[].f1 is [Full range of float32]
  @param[in]       f_InputValue : the X value for which the linear interpolated
                   Y bases on 'pTable' shall be returned.
                   [Full range of float32]

  @return          The y-value at the specified x-position
*****************************************************************************/
float32 TUE_CML_CalculatePolygonValue2D(sint32 s_NrOfTableRows,
                                        const TUE_CML_Vector2D_t a_Table[],
                                        float32 f_InputValue) {
    /*get table value*/
    float32 f_Result;

    /* If the x-position is left of the smallest sampling point,return the
     * smallest sampling point y-value */
    if (f_InputValue <= a_Table[0].f0) {
        f_Result = a_Table[0].f1;
    }
    /* If the x-position is right of the biggest sampling point,return the
       biggest sampling point y-value */
    else if (f_InputValue >= a_Table[s_NrOfTableRows - 1].f0) {
        f_Result = a_Table[s_NrOfTableRows - 1].f1;
    }
    /* In all other cases return the interpolated value between the matching
       sampling points */
    else {
        sint32
            s_Right; /* The index of the nearest sampling point to the right */
        float32 f_WeightRight;
        float32 f_dx;

        s_Right = 1; /* Since we already know that index 0 is greater */
        while (a_Table[s_Right].f0 < f_InputValue) {
            s_Right++;
        }
        f_dx = a_Table[s_Right].f0 - a_Table[s_Right - 1].f0;

        if (f_dx > TUE_C_F32_DELTA) {
            f_WeightRight = (f_InputValue - a_Table[s_Right - 1].f0) / (f_dx);
        } else {
            f_WeightRight = 0.0f;
        }

        f_Result = ((a_Table[s_Right - 1].f1) * (1.0f - f_WeightRight)) +
                   ((a_Table[s_Right].f1) * f_WeightRight);
    }
    return f_Result;
}

/*****************************************************************************
  Functionname:    TUE_CML_LowPassFilter                                 */ /*!

  @brief           simple first order lowpass filter

  @description     This function is an implementation of simple first order
				   lowpass filter. This determines the output sample in terms of
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
void TUE_CML_LowPassFilter(float32* f_Old, float32 f_New, float32 f_Alpha) {
    float32 f_Dummy;

    f_Dummy = (f_Alpha * f_New) + ((1.f - f_Alpha) * (*f_Old));
    *f_Old = f_Dummy;
}

/*****************************************************************************
  Functionname:    TUE_CML_SqrtApprox                                 */ /*!

  @brief           approx calculate of square root

  @description     approx calculate of square root

  @param[in]       f_radicand : input square value

  @return          square result of input value


*****************************************************************************/
float32 TUE_CML_SqrtApprox(float32 f_radicand) {
    sint32 s_expo;
    uint32 u_tmp;
    float32 f_Sample, f_SampleSquare;
    float32 f_A, f_B, f_A_plus_B;
    float32 f_ret;

    /*! union for doing bit-wise manipulation on the floating point
     * representation */

    union {
        float32 f_value; /* this is the number of interest as float */
        uint32 u_value;  /* in here we hold the same number as int on which we
                            can do bit-wise manipulations */
    } x_tmp;

    /* check for x < 0 or x = NaN */
    if (f_radicand < 0) {
        return 0.f;
    }

    /* copy input value to union where we can manipulate it directly */
    x_tmp.f_value = f_radicand;

    /* check for negative or zero or denormalized */
    if (((x_tmp.u_value >> 31) > 0U)      /* sign bit set? */
        || ((x_tmp.u_value >> 23) == 0u)) /* exponent is zero? */
    {
        f_ret = 0.f;
    }
    /* check for infinity */
    else if ((x_tmp.u_value >> 23) >= 0xffu) {
        x_tmp.u_value = 0x7f7fffffu;
        f_ret = x_tmp.f_value; /* return max_float */
    } else {
        /* calculate start value by dividing exponent by two */
        u_tmp = x_tmp.u_value >> TUE_CML_SqrtApprox_NumExpo;
        s_expo = (sint32)(u_tmp);
        s_expo -= TUE_CML_SqrtApprox_ExponentOffset;
        if (s_expo < 0) {
            s_expo >>= 1; /* without the following line this would be unsafe
                             because right shift of an unsigned int is machine
                             dependent (sign fill vs. zero fill) */
            s_expo |=
                (sint32)0x80000000U; /* -> fill highest bit with sign (one)
                                        after shift to make it safe */
        } else {
            s_expo >>=
                1; /* for positive expo sign = 0 -> sign fill = zero fill */
        }
        s_expo += TUE_CML_SqrtApprox_ExponentOffset;
        x_tmp.u_value &= TUE_CML_SqrtApprox_MantissaMask;
        x_tmp.u_value += ((uint32)s_expo << TUE_CML_SqrtApprox_NumExpo);
        f_Sample = x_tmp.f_value;

        /* two iterations are enough */
        /* iteration one */
        f_SampleSquare = TUE_CML_Sqr(f_Sample);
        f_A = 2.f * (f_SampleSquare + f_radicand);
        f_B = f_SampleSquare - f_radicand;
        f_A_plus_B = f_A + f_B;
        if (TUE_CML_Abs(f_A_plus_B) < TUE_CML_SqrtApprox_AlmostZero) {
            f_ret = 0.f;
        } else {
            f_Sample *= ((f_A - f_B) / (f_A_plus_B));
            /* iteration two */
            f_SampleSquare = TUE_CML_Sqr(f_Sample);
            f_A = 2.f * (f_SampleSquare + f_radicand);
            f_B = f_SampleSquare - f_radicand;

            f_A_plus_B = f_A + f_B;
            f_Sample *= ((f_A - f_B) / (f_A_plus_B));
            f_ret = f_Sample;
        }
    }

    return f_ret;
}

/*****************************************************************************
  Functionname:    TUE_CML_LinearInterpolation                            */ /*!

  @brief           linear interpolation between two given points

  @description     This function computes the linear interpolation value
				   between two given points. The interpolated value can be
				   calculated using the following formula:
				   y = mx + c;
				   where 'm' is the slope and 'c' is the offset.

  @param[in]       f_X1 :  x-coordinate of first point
						   [Full range of float32]
  @param[in]       f_Y1 :  y-coordinate of first point
						   [Full range of float32]
  @param[in]       f_X2 :  x-coordinate of second point
						   [Full range of float32]
  @param[in]       f_Y2 :  y-coordinate of second point
						   [Full range of float32]
  @param[in]       f_XPos :  x-value to interpolate
							 [Full range of float32]

  @return          the interpolated value


*****************************************************************************/
float32 TUE_CML_LinearInterpolation(
    float32 f_X1, float32 f_Y1, float32 f_X2, float32 f_Y2, float32 f_XPos) {
    float32 f_Slope, f_Offset, f_ret;

    if (!TUE_CML_IsZero(f_X2 - f_X1)) {
        /* slope */
        f_Slope = (f_Y2 - f_Y1) / (f_X2 - f_X1);
        /* offset */
        f_Offset = f_Y1 - (f_Slope * f_X1);
        /* interpolate */
        f_ret = (f_Slope * f_XPos) + f_Offset;
    } else {
        f_ret = (f_Y1 + f_Y2) * 0.5f;
    }
    return f_ret;
} /* BML_f_LinearInterpolation() */

/*****************************************************************************
  Functionname:    TUE_CML_CalcStdGaussianCDF                             */ /*!

  @brief           Calculate the value of the standard Gaussian CDF

  @description     This function calculate the value of the standard Gaussian
				   cumulative distribution function
				   CDF = 0.5 ( 1 + errorfunction ( ( x - aver ) / (sigma * sqrt(2) ) )

  @param[in]       f_value : input to the CDF
							 [Full range of float]
  @param[in]       f_avg   : mean of the Gaussian distribution
							 [Full range of float]
  @param[in]       f_sigma : standard deviation of the Gaussian distribution
							 [Full range of float]

  @return          standard Gaussian CDF at the given value


*****************************************************************************/
float32 TUE_CML_CalcStdGaussianCDF(float32 f_value,
                                   float32 f_avg,
                                   float32 f_sigma) {
    float32 f_temp;

    if (TUE_CML_Abs(f_sigma) < TUE_CML_GaussianCDFMinSigma) {
        if (f_value < f_avg) {
            f_temp = 0.0f;
        } else {
            f_temp = 1.0f;
        }
    } else {
        f_temp = (f_value - f_avg) / (f_sigma * TUE_CML_SQRT_OF_2);

        /* check for negative values */
        if (f_temp < 0.0f) {
            f_temp = -TUE_CML_CalcGaussErrorFunction(-f_temp);
        } else {
            f_temp = TUE_CML_CalcGaussErrorFunction(f_temp);
        }

        f_temp = (0.5f * f_temp + 0.5f);
    }

    return f_temp;
}

/*****************************************************************************
  Functionname:    TUE_CML_CalcGaussErrorFunction                         */ /*!

  @brief           Calculate the Gauss Error Function

  @description     This function calculate the Gauss Error Function
				   Aproximate with a 4th order polynomial, the return value
				   G = ( ( ( (C4 * x^4) - (C3 * x^3) ) - (C2 * x^2) ) + (C1 * x) ) + C0,
				   where the coefficients C0, C1, C2, C3 and C4 are predefined
				   values.

  @param[in]       f_value : input to the Gauss error function
							 Supported values for f_value [-F_MAX...F_MAX]
							 where F_MAX is the fourth root of the maximum value of float32.

  @return          Gauss error function value


*****************************************************************************/
float32 TUE_CML_CalcGaussErrorFunction(float32 f_value) {
    float32 f_temp2, f_temp3, f_temp, f_temp4;

    if (f_value >= TUE_CML_GaussErrFctMaxX) {
        f_temp = 1.0f;
    } else {
        f_temp2 = f_value * f_value; /* x^2 */
        f_temp3 = f_temp2 * f_value; /* x^3 */
        f_temp4 = f_temp2 * f_temp2; /* x^4 */
        f_temp = ((((TUE_CML_GaussErrFctConst4 * f_temp4) -
                    (TUE_CML_GaussErrFctConst3 * f_temp3)) -
                   (TUE_CML_GaussErrFctConst2 * f_temp2)) +
                  (TUE_CML_GaussErrFctConst1 * f_value)) +
                 TUE_CML_GaussErrFctConst0;

        f_temp = TUE_CML_Min(f_temp, 1.0f);
    }

    return f_temp;
}

/*****************************************************************************
  Functionname:    TUE_CML_CalcPointApproxPolyL2                         */ /*!

  @brief           Calculate 2nd power polynomial for approximating sample points

  @description     This function calculates the approximate polynomial fitting
                   the sample points using least square fit. The calculated
                   polynomial has the form f(x) = fC0 + fC1*x + fC2*x^2

  @param[in,out]   pPoly : Pointer to structure storing the second degree polynomial approximation of a trace [CPTracePolyL2_t as defined in cp_ext.h]
						pPoly->fC2 : Coefficient of second-order term                                      [-1f...+1f]
						pPoly->fC1 : Coefficient of first-order term                                       [-1f...+1f]
						pPoly->fC0 : Coefficient of zeroth-order term                                      [-PI/2*RW_FCT_MAX ... PI/2*RW_FCT_MAX]
						pPoly->isValid : Flag to indicate whether the trace polynomial is valid            [TRUE, FALSE]
  @param[in]       pafX[] : array of x coordinates of trace points                  [-5*RW_FCT_MAX ... 5*RW_FCT_MAX] of size [0 ... FIP_STATIC_TRACE_NO_OF_POINTS[
  @param[in]       pafY[] : array of y coordinates of trace points                  [-PI/2*RW_FCT_MAX ... PI/2*RW_FCT_MAX] of size [0 ... FIP_STATIC_TRACE_NO_OF_POINTS[
  @param[in]       uNumPts : Number of trace points                                  [0 ... FIP_STATIC_TRACE_NO_OF_POINTS]

  @return          null
*****************************************************************************/
void TUE_CML_CalcPointApproxPolyL2(TUE_CML_PolyResult_t* pPoly,
                                   const float32 pafX[],
                                   const float32 pafY[],
                                   uint8 uNumPts) {
    sint32 i, j, k;
    float32 fXPow4Sum, fXPow3Sum, fXPow2Sum, fXSum;
    float32 fXYSum, fX2YSum, fYSum, fNumPts;
    float32 fLinEqMatrix[3][4];

    /* Verify that we have at least 3 points, below that just use a line
     * extrapolation */
    if (uNumPts > 2u) {
        /* First calculate the necessary terms for our linear equation set
        to use for least squares fit */
        fXPow4Sum = 0.f;
        fXPow3Sum = 0.f;
        fXPow2Sum = 0.f;
        fXSum = 0.f;
        fXYSum = 0.f;
        fX2YSum = 0.f;
        fYSum = 0.f;
        fNumPts = (float32)uNumPts;
        /* Initialize return polynomial C0 to Y0 (plays a role when X
         * coordinates all zero) */
        pPoly->fC0 = *pafY;
        /* Go through all points and calculate the sums */
        while (uNumPts > 0u) {
            const float32 fCurX = *pafX;
            const float32 fCurY = *pafY;
            const float32 fCurX2 = TUE_CML_Sqr(fCurX);
            pafX++;

            pafY++;

            /* Update sums */
            fXSum += fCurX;
            fXPow2Sum += fCurX2;
            fXPow3Sum += fCurX2 * fCurX;
            fXPow4Sum += TUE_CML_Sqr(fCurX2);
            fYSum += fCurY;
            fXYSum += fCurX * fCurY;
            fX2YSum += fCurX2 * fCurY;
            /* Decrease remaining number of points */
            uNumPts--;
        }
        /* Now we have a linear equation set:
        fXPow4Sum*C2 + fXPow3Sum*C1 + fXPow2Sum*C0 = fX2YSum
        fXPow3Sum*C2 + fXPow2Sum*C1 + fXSum*C0     = fXYSum
        fXPow2Sum*C2 + fXSum*C1     + NumPts*C0    = fYSum
        Notice how the diagonal of the matrix is always positive, if there is
        at least one point with an X coordinate other than 0. Also note that
        the inverse of fXPowySum always exists (due to this) */
        if (TUE_CML_Abs((fXPow2Sum * fXPow4Sum) - (fXPow3Sum * fXPow3Sum)) >
            TUE_C_F32_DELTA) {
            fLinEqMatrix[0][0] = fXPow4Sum;
            fLinEqMatrix[0][1] = fXPow3Sum;
            fLinEqMatrix[0][2] = fXPow2Sum;
            fLinEqMatrix[0][3] = fX2YSum;
            fLinEqMatrix[1][0] = fXPow3Sum;
            fLinEqMatrix[1][1] = fXPow2Sum;
            fLinEqMatrix[1][2] = fXSum;
            fLinEqMatrix[1][3] = fXYSum;
            fLinEqMatrix[2][0] = fXPow2Sum;
            fLinEqMatrix[2][1] = fXSum;
            fLinEqMatrix[2][2] = fNumPts;
            fLinEqMatrix[2][3] = fYSum;
            /* Now solve it, first converting the matrix to upper triangular
            form and then using elimination to solve it (Gauss elimination) */
            for (i = 0; i < 2; i++) {
                float32 fDivisor = fLinEqMatrix[i][i];
                float32 fInvColumnMax;
                /* Calculate inverse of column max once here, prevent devision
                 * by zero */
                if (TUE_CML_Abs(fDivisor) < TUE_C_F32_DELTA) {
                    if (fDivisor < 0.0F) {
                        fDivisor = -TUE_C_F32_DELTA;
                    } else {
                        fDivisor = TUE_C_F32_DELTA;
                    }
                }
                fInvColumnMax = 1.f / fDivisor;
                /* Now do forward substitution */
                for (j = 3; j >= i; j--) {
                    for (k = i + 1; k < 3; k++) {
                        fLinEqMatrix[k][j] -= fLinEqMatrix[k][i] *
                                              fInvColumnMax *
                                              fLinEqMatrix[i][j];
                    }
                }
            }
            /* Now do reverse elimination */
            for (i = 2; i >= 0; i--) {
                /* Verify that we have a leading non-zero value in the row */
                if ((fLinEqMatrix[i][i] > TUE_C_F32_EXT_DELTA) ||
                    (fLinEqMatrix[i][i] < -TUE_C_F32_EXT_DELTA)) {
                    /* Calculate inverse of the diagonal element currently
                     * processed */
                    const float32 fInvCurDiagVal = 1.f / fLinEqMatrix[i][i];
                    fLinEqMatrix[i][3] = fLinEqMatrix[i][3] * fInvCurDiagVal;
                    fLinEqMatrix[i][i] = 1.f;
                    for (j = i - 1; j >= 0; j--) {
                        fLinEqMatrix[j][3] -=
                            fLinEqMatrix[j][i] * fLinEqMatrix[i][3];
                        fLinEqMatrix[j][i] = 0.f;
                    }
                } else {
                    fLinEqMatrix[i][3] = 0.f;
                }
            }
            /* Fill in result */
            pPoly->fC2 = fLinEqMatrix[0][3];
            pPoly->fC1 = fLinEqMatrix[1][3];
            pPoly->fC0 = fLinEqMatrix[2][3];
            pPoly->isValid = TRUE;
        } else {
            /* fXPow2Sum is zero -> all points have zero X coordinate */
            pPoly->fC2 = 0.f;
            pPoly->fC1 = 0.f;
            pPoly->isValid = FALSE;
            /* C0 initialization value to Y coordinate already OK */
        }
    } else {
        /* Initialize default return value */
        pPoly->fC2 = 0.f;
        pPoly->fC1 = 0.f;
        pPoly->fC0 = pafY[0];
        pPoly->isValid = FALSE;
        /* Approximate object movement by a simple line */
        if (uNumPts > 1u) {
            const float32 fDeltaX = pafX[1] - pafX[0];
            const float32 fDeltaY = pafY[1] - pafY[0];
            if ((fDeltaX > TUE_C_F32_DELTA) || (fDeltaX < -TUE_C_F32_DELTA)) {
                pPoly->fC2 = 0.f;
                pPoly->fC1 = (fDeltaY / fDeltaX);
                pPoly->fC0 = pafY[0] - (pafX[0] * pPoly->fC1);
                pPoly->isValid = TRUE;
            }
        }
    }
}

/*****************************************************************************
  Functionname:    TUE_CML_ModTrig                                        */ /*!

  @brief           Calculates the remainder of x when divided by y as needed
					by trigonometric functions

  @description     This function calculates the remainder of x when divided by y
				   Works only for y > 0
				   The function is equivalent to rem() function in Matlab.


  @param[in]       f_dividend : The Dividend
								Supported values are [Full range of float32]
								Overflow may occur at higher values.
  @param[in]       f_divisor  : The Divisor
								Supported values are [Full range of float32]
								Overflow may occur at very small values.

  @return          remainder of f_dividend when divided by f_divisor

*****************************************************************************/
float32 TUE_CML_ModTrig(float32 f_dividend, float32 f_divisor) {
    float32 f_quotient, f_ret;
    sint32 s_quotient;

    if (f_divisor < TUE_CML_ModuloEps) {
        f_ret = 0.f;
    } else {
        f_quotient = f_dividend / f_divisor;

        if (TUE_CML_Abs(f_quotient) > (float32)TUE_CML_LONG_MAX) {
            f_ret = 0.f;
        } else {
            s_quotient = (sint32)(f_quotient);
            f_ret = (f_dividend - ((float32)s_quotient * f_divisor));
        }
    }

    return f_ret;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBtan_56s */ /*!

  @brief           computes tan(pi*x/4)

  @description     Accurate to about 5.6 decimal digits over
				   the range [0, pi/4].
				   Note that the function computes tan(pi*x/4),
				   NOT tan(x); it's up to the range
				   reduction algorithm that calls this to scale
				   things properly.
				   Algorithm: tan(x)= x(c1 + c2*x^2)/(c3 + x^2)

  @param[in]       f_angle : the angle (times pi/4) for which we want to know
							 the tangent, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE is cube root of max value of float32

  @return          parameter for tan_56(f_angle)

  @pre             GDBtan_56(x)

*****************************************************************************/
float32 TUE_CML_GDBtan_56s(float32 f_angle) {
    return (f_angle *
            (TUE_CML_TAN_56_C1 + (TUE_CML_TAN_56_C2 * TUE_CML_Sqr(f_angle)))) /
           (TUE_CML_TAN_56_C3 + TUE_CML_Sqr(f_angle));
}
/*****************************************************************************
  Functionname:    TUE_CML_GDBtan_52 */ /*!

  @brief           Computes the tangent of x with accuracy of about 5.6
				   decimal digits

  @description     This is the main tangent approximation "driver".
				   It reduces the input argument's range to [0, pi/4],
				   and then calls the approximator.
					WARNING: We do not test for the tangent approaching
				   infinity,  which it will at x=pi/2 and x=3*pi/2.
				   If this is a problem in your application, take
				   appropriate action.

  @param[in]       f_angle : the angle for which we want to know the
							 tangent, radians
							 Supported values are [Full range of float32]
							 except ((2*n) + 1)*(0.5f*TUE_CML_Pi), n is any integer.

  @return          the tangent of f_angle

*****************************************************************************/
float32 TUE_CML_GDBtan_52(float32 f_angle) {
    /*--- VARIABLES ---*/
    float32 f_tan, f_tmp;
    uint32 u_octant;
    boolean b_sign = FALSE;

    if (f_angle < 0.0F) {
        f_angle = -f_angle;
        b_sign = TRUE;
    }

    /* limit to two pi */
    if (f_angle > (10.0f * TUE_CML_Pi)) {
        f_angle = TUE_CML_ModTrig(f_angle, (2.0f * TUE_CML_Pi));
    } else {
        while (f_angle >= (2.0f * TUE_CML_Pi)) {
            f_angle -= (2.0f * TUE_CML_Pi);
        }
    }

    f_tmp = f_angle * (4.0f / TUE_CML_Pi);
    u_octant = (uint32)f_tmp;

    switch (u_octant) {
        case 1:
            f_tan = 1.0F / TUE_CML_GDBtan_56s(((0.5f * TUE_CML_Pi) - f_angle) *
                                              (4.0f / TUE_CML_Pi));
            break;
        case 2:
            f_tan = -1.0F / TUE_CML_GDBtan_56s((f_angle - (0.5f * TUE_CML_Pi)) *
                                               (4.0f / TUE_CML_Pi));
            break;
        case 3:
            f_tan = -TUE_CML_GDBtan_56s((TUE_CML_Pi - f_angle) *
                                        (4.0f / TUE_CML_Pi));
            break;
        case 4:
            f_tan = TUE_CML_GDBtan_56s((f_angle - TUE_CML_Pi) *
                                       (4.0f / TUE_CML_Pi));
            break;
        case 5:
            f_tan = 1.0F / TUE_CML_GDBtan_56s(((1.5f * TUE_CML_Pi) - f_angle) *
                                              (4.0f / TUE_CML_Pi));
            break;
        case 6:
            f_tan = -1.0F / TUE_CML_GDBtan_56s((f_angle - (1.5f * TUE_CML_Pi)) *
                                               (4.0f / TUE_CML_Pi));
            break;
        case 7:
            f_tan = -TUE_CML_GDBtan_56s(((2.0f * TUE_CML_Pi) - f_angle) *
                                        (4.0f / TUE_CML_Pi));
            break;
        default:
            /*Case 0*/
            f_tan = TUE_CML_GDBtan_56s(f_angle * (4.0f / TUE_CML_Pi));
            break;
    }

    if (b_sign == TRUE) {
        f_tan = -f_tan;
    }

    return f_tan;
}

/*****************************************************************************
  Functionname:    TUE_CML_CalcPointApproxPolyL2                         */ /*!

  @brief           Calculate 3nd clothoid model DistY value in target DistX

  @description     the form Y = Y0 + aX + 0.5 * C0 * X^2 + 1/6 * C1 * X^3

  @param[in]       

  @return          fDistY
*****************************************************************************/
float32 TUE_CML_CalcDistYOfClothoidsCurve(float32 fHeadingAngle,
                                          float32 fCurve,
                                          float32 fCurveDer,
                                          float32 fDistX,
                                          float32 fDistY0) {
    float32 y_s;
    y_s = fDistY0 + (TUE_CML_GDBtan_52(fHeadingAngle)) * fDistX +
          (0.5f * fDistX * fDistX * fCurve) +
          ((1.0f / 6.0f) * fDistX * fDistX * fDistX * fCurveDer);
    return y_s;
}

/*****************************************************************************
  @fn				TUE_CML_RisingEdgeSwitch */ /*!

  @description		checks if the switch is switched on

  @param[in]		bSwitch		Switch to check (TRUE, FALSE)
  @param[in,out]    bPrevSwitch	Previous status of switch (TRUE, FALSE)
								Define a global variable and input its address.
								Note: The initial value should be TRUE

  @return			pSwitchChk	Flag that the switch is switched on

*****************************************************************************/
boolean TUE_CML_RisingEdgeSwitch(const boolean bSwitch, boolean* pPrevSwitch) {
    boolean bSwitchChk = FALSE;

    if ((*pPrevSwitch == FALSE) && (bSwitch == TRUE)) {
        bSwitchChk = TRUE;
    }

    *pPrevSwitch = bSwitch;

    return bSwitchChk;
}

/*****************************************************************************
  @fn				TUE_CML_FallingEdgeSwitch */ /*!

  @description		checks if the switch is switched off

  @param[in]		bSwitch		Switch to check (TRUE, FALSE)
  @param[in,out]    bPrevSwitch	Previous status of switch (TRUE, FALSE)
								Define a global variable and input its address.
								Note: The initial value should be FALSE

  @return			pSwitchChk	Flag that the switch is switched off

*****************************************************************************/
boolean TUE_CML_FallingEdgeSwitch(const boolean bSwitch, boolean* pPrevSwitch) {
    boolean bSwitchChk = FALSE;

    if ((*pPrevSwitch == TRUE) && (bSwitch == FALSE)) {
        bSwitchChk = TRUE;
    }

    *pPrevSwitch = bSwitch;

    return bSwitchChk;
}

/*****************************************************************************
  @fn           TUE_CML_HoldRepeatSwitch */ /*!

  @description  returns SWTICH_STATE_ON, if the switch was hold long enough to get repeated signals from that switch

  @param[in]    pSwitch         Switch to check
  @param[in]    StartCondition  Condition that allows the switch to be switched on (0 = false, 1 = true)
  @param[in]    HoldCondition   Condition that allows the switch to send repeated signals after has been switched on (0 = false, 1 = true)
  @param[in]    StartTime       Time, the switch needs to be in on state before it returns true for the first time
  @param[in]    RepeatTime      Time, between the repeated signals after the switch has returned true the first time (if 0 -> function returns only one true after start time)

  @return       TUE_SWITCH_STATE_OFF(0) = FALSE,
				TUE_SWITCH_STATE_ON(1) = TRUE,
				TUE_SWITCH_STATE_ACTION_OFF(2) = currently not returned

*****************************************************************************/
// tue_switch_state_t TUE_CML_HoldRepeatSwitch(TUE_CML_Switch_t * const pSwitch,
// const boolean StartCondition, const boolean HoldCondition, uint16 StartTime,
// uint16 RepeatTime)
// {
//	tue_switch_state_t retValue = TUE_SWITCH_STATE_OFF;
//
// if (
//		(pSwitch->AKT_STATUS == FALSE)  /*switch off*/
//		||
//		((HoldCondition == FALSE) && (pSwitch->CYCLE_TIMER !=
// TUE_CML_SWITCH_CYCLETIMER_INIT))
//		||
//		(StartCondition != TRUE)       /*not all conditions true*/
//		)
//	{
//		/*init cycle timer*/
//		pSwitch->CYCLE_TIMER = TUE_CML_SWITCH_CYCLETIMER_INIT;
//	}
//	else    /*button pressed with all conditions met*/
//	{
//		const tue_switch_state_t StartOK =
// TUE_CML_RisingEdgeSwitch(pSwitch->AKT_STATUS, StartCondition);
//
//		if (
//			(pSwitch->CYCLE_TIMER == TUE_CML_SWITCH_CYCLETIMER_INIT)
///*cycle timer not initialized*/
//			&&
//			(StartOK == TUE_SWITCH_STATE_ON)              /*start
// conditions true*/
//			)
//		{ /*initialize cycle with startTime*/
//			pSwitch->CYCLE_TIMER =
//(uint16)TUE_CML_Min(TUE_CML_SWITCH_CYCLETIMER_INIT - 1, (sint32)StartTime);
//		}
//		else
//		{
//			if (
//				(pSwitch->CYCLE_TIMER > (uint16)0) /*cycle timer
//> 0*/
//				&&
//				(pSwitch->CYCLE_TIMER <
// TUE_CML_SWITCH_CYCLETIMER_INIT) /*cycle timer is initialized*/
//				&&
//				(HoldCondition == TRUE) /*hold condition true*/
//				)
//			{
//				pSwitch->CYCLE_TIMER--; /*decrease cycle timer*/
//			}
//
//			if (pSwitch->CYCLE_TIMER == (uint16)0) /*cycle timer
// 0ed*/ 			{ /*initialize cycle timer with repeat time*/
// pSwitch->CYCLE_TIMER = (uint16)TUE_CML_Min(TUE_CML_SWITCH_CYCLETIMER_INIT -
// 1,
//(sint32)RepeatTime); 				if (pSwitch->CYCLE_TIMER ==
//(uint16)0)
//				{
//					pSwitch->CYCLE_TIMER =
// TUE_CML_SWITCH_CYCLETIMER_INIT; /*reinitialize cycle timer (returns only one
// true after start time if repeattime=0)*/
//				}
//				retValue = TUE_SWITCH_STATE_ON;
//			}
//		}
//	}
//	return retValue;
//}

/*****************************************************************************
  @fn             SWITCH_INIT_SWITCH */ /*!

  @description    initialize switches

  @param[in]      pSwitch   Switch that shall be initialized

  @return         void

*****************************************************************************/
void TUE_CML_InitSwitch(TUE_CML_Switch_t* const pSwitch) {
    pSwitch->AKT_STATUS = FALSE;
    pSwitch->LAST_STATUS = FALSE;
    pSwitch->CYCLE_TIMER = TUE_CML_SWITCH_CYCLETIMER_INIT;
    pSwitch->DURATION_TIME_INACTIVE = TUE_CML_SWITCH_TIME_MAX;
    pSwitch->DURATION_TIME_ACTIVE = 0u;
    pSwitch->OK_WHILE_SWITCHED_ON = FALSE;
}

/*****************************************************************************
  @fn             TUE_CML_SetStateSwitch */ /*!

  @description    sets a new switch state for a specific switch

  @param[in,out]  pSwitch  Switch that shall be set
  @param[in]      State    the new state for the switch (TRUE, FALSE)

  @return         void

*****************************************************************************/
void TUE_CML_SetStateSwitch(TUE_CML_Switch_t* const pSwitch,
                            const boolean State) {
    const boolean lastState = (boolean)pSwitch->AKT_STATUS;

    /*save old value*/
    pSwitch->LAST_STATUS = pSwitch->AKT_STATUS;
    /*Set new value*/
    pSwitch->AKT_STATUS = State;

    /*count cycles of (in)activity*/
    if (State == TRUE) {
        if (pSwitch->DURATION_TIME_ACTIVE < TUE_CML_SWITCH_TIME_MAX) {
            pSwitch->DURATION_TIME_ACTIVE++;
        }
    } else {
        if (pSwitch->DURATION_TIME_INACTIVE < TUE_CML_SWITCH_TIME_MAX) {
            pSwitch->DURATION_TIME_INACTIVE++;
        }
    }

    if (lastState == FALSE) {
        pSwitch->DURATION_TIME_ACTIVE = (uint16)0;
    } else {
        pSwitch->DURATION_TIME_INACTIVE = (uint16)0;
    }
}

/*****************************************************************************
  @fn				TUE_CML_RSFlipFlop */ /*!

  @description		R-S Flip-Flop

  @param[in]		S		Set input of R-S Flip-Flop(TRUE, FALSE)
  @param[in]		R		Reset input of R-S Flip-Flop(TRUE, FALSE)
  @param[in,out]    pPrevQ	Previous output Q of R-S Flip-Flop(TRUE, FALSE)
							Define a global variable and input its address.
							Note: The initial value should be FALSE.

  @return           Q		Output Q of R-S Flip-Flop(TRUE, FALSE)
*****************************************************************************/
boolean TUE_CML_RSFlipFlop(const boolean S, const boolean R, boolean* pPrevQ) {
    boolean Q = FALSE;

    if (R != FALSE) {
        Q = FALSE;
    } else {
        Q = S || (*pPrevQ);
    }

    *pPrevQ = Q;

    return Q;
}

/*****************************************************************************
  @fn				TUE_CML_RateLimiter */ /*!

  @description		Rate limiter

  @param[in]		fInput		Input to limit
  @param[in]		fLimPos		Max positive limit of rate
  @param[in]		fLimNeg 	Min negative limit of rate							   
  @param[in]		fTs 		Schedule time
  @param[in,out]	pPrevOutput Previous output of RateLimiter
								Define a global variable and input its address.
								Note: The initial value should be 0.f

  @return           fOutput		Output after rate limiting
*****************************************************************************/
float32 TUE_CML_RateLimiter(const float32 fInput,
                            const float32 fLimPos,
                            const float32 fLimNeg,
                            const float32 fTs,
                            float32* pPrevOutput) {
    float32 fOutput = 0.f;

    fOutput = fInput - (*pPrevOutput);
    fOutput = TUE_CML_MinMax(-fLimNeg * fTs, fLimPos * fTs, fOutput);
    fOutput = fOutput + (*pPrevOutput);

    *pPrevOutput = fOutput;

    return fOutput;
}

/*****************************************************************************
  @fn				TUE_CML_HysteresisFloat */ /*!

  @description		Hysteresis

  @param[in]		fInput		 Input to limit
  @param[in]		fThresHigh	 High threshold 
  @param[in]		fThresLow 	 Low threshold 
  @param[in,out]	pPrevOutput  Previous output of Hysteresis
								 Define a global variable and input its address.
								 Note: The initial value should be FALSE

  @return           *pPrevOutput Hysteresis output
*****************************************************************************/
boolean TUE_CML_HysteresisFloat(const float32 fInput,
                                const float32 fThresHigh,
                                const float32 fThresLow,
                                boolean* pPrevOutput) {
    if (fThresHigh >= fThresLow) {
        if ((*pPrevOutput == FALSE) && (fInput > fThresHigh)) {
            *pPrevOutput = TRUE;
        } else if ((*pPrevOutput == TRUE) && (fInput < fThresLow)) {
            *pPrevOutput = FALSE;
        } else {
        }
    } else {
        if ((*pPrevOutput == FALSE) && (fInput > fThresLow)) {
            *pPrevOutput = TRUE;
        } else if ((*pPrevOutput == TRUE) && (fInput < fThresHigh)) {
            *pPrevOutput = FALSE;
        } else {
        }
    }

    return *pPrevOutput;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDB_cos32 */ /*!

  @brief           Calculates the cosine with 3.2 decimals accuracy

  @description     It reduces the input argument's range to [0, pi/2],
				   and then performs the approximation.
				   Algorithm:
						   cos(x)= c1 + c2*x**2 + c3*x**4
				   which is the same as:
						   cos(x)= c1 + x**2(c2 + c3*x**2)

  @param[in]       f_angle : input angle for which we would like to know the cosine, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE = [max range of uint32] * CML_f_two_Pi
  @return          the cosine of f_angle
*****************************************************************************/
float32 TUE_CML_GDB_cos32(float32 f_angle) {
    /*--- VARIABLES ---*/
    uint32 u_n;
    float32 f_angle_square, f_tmp;
    float32 f_Ret;

    /* remove sign, as COS function is symmetric */
    f_angle = TUE_CML_Abs(f_angle);

    /* Calculate approximation depending on quadrant. First, check if f_angle is
       in 1st one. */
    if (f_angle < (TUE_CML_Pi / 2.0f)) {
        f_angle_square = TUE_CML_Sqr(f_angle);
        f_tmp = (C_COS_32_C3 * f_angle_square + C_COS_32_C2);
        f_Ret = (f_angle_square * f_tmp + C_COS_32_C1);
    } else if (f_angle < (TUE_CML_Pi + (TUE_CML_Pi * 0.5f))) {
        /* 2nd and 3rd quadrant. */
        f_angle_square = TUE_CML_Pi - f_angle;
        f_angle_square = TUE_CML_Abs(f_angle_square);
        f_angle_square = TUE_CML_Sqr(f_angle_square);
        f_tmp = TUE_CML_MultAdd((-1.0f) * C_COS_32_C3, f_angle_square,
                                (-1.0f) * C_COS_32_C2);
        f_Ret = TUE_CML_MultAdd(f_angle_square, f_tmp, (-1.0f) * C_COS_32_C1);
    } else if (f_angle < (TUE_CML_Pi * 2.0f)) {
        /* 4th quadrant. */
        f_angle_square = (TUE_CML_Pi * 2.0f) - f_angle;
        f_angle_square = TUE_CML_Sqr(f_angle_square);
        f_tmp = (C_COS_32_C3 * f_angle_square + C_COS_32_C2);
        f_Ret = (f_angle_square * f_tmp + C_COS_32_C1);
    } else {
        /* f_angle is out of 1st period. --> Shift it to [-PI..+PI] and use
       symmetry of COS. */
        /* limit to two_pi : f_angle = mod(f_angle, two_pi) limitation: quotient
           shall no exceed C_LONG_MAX. => f_angle < (LONG_MAX * (2.0f *
           TUE_CML_Pi)) Regarding to float32 accuracy of about 7 decimals, the
           reasonable threshold for f_angle is reached much earlier. */
        f_tmp = f_angle * (1.0f / (TUE_CML_Pi * 2.0f));
        u_n = (uint32)(f_tmp);

        /* Shift f_angle to [-PI..PI]. Due to symmetry of COS, it's enough to
           evaluate [0..PI]. */
        f_angle = (f_angle - ((float32)u_n * (TUE_CML_Pi * 2.0f))) - TUE_CML_Pi;
        f_angle = TUE_CML_Abs(f_angle);

        /* Calculate approximation depending on quadrant. First, check if
           f_angle is in 2nd (or 3rd) one. */
        if (f_angle > (TUE_CML_Pi * 0.5f)) {
            f_angle_square = TUE_CML_Pi - f_angle;
            f_angle_square = TUE_CML_Sqr(f_angle_square);
            f_tmp = (C_COS_32_C3 * f_angle_square + C_COS_32_C2);
            f_Ret = (f_angle_square * f_tmp + C_COS_32_C1);
        } else {
            /* 1st (or 4th) quadrant). */
            f_angle_square = TUE_CML_Sqr(f_angle);
            f_tmp = (C_COS_32_C3 * f_angle_square + C_COS_32_C2);
            f_Ret = -(f_angle_square * f_tmp + C_COS_32_C1);
        }
    }

    return (f_Ret);
} /* TUE_CML_GDB_cos32() */

/*****************************************************************************
  Functionname:    TUE_CML_GDB_sin32 */ /*!

  @brief           Calculates the sine with 3.2 decimals accuracy

  @description     This function calculates the sine with 3.2 decimals
				   accuracy.
				   The sine is just cosine shifted a half-pi,
				   so the argument is adjusted and the cosine
				   approximation is called.

  @param[in]       f_angle : input angle for which we would like to know the
							 sine, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE =
							 ([max range of uint32] * CML_f_two_Pi)-(0.5f * TUE_CML_Pi)
  @return          the sine of f_angle
*****************************************************************************/
float32 TUE_CML_GDB_sin32(float32 f_angle) {
    return TUE_CML_GDB_cos32((TUE_CML_Pi * 0.5f) - f_angle);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDB_tan32 */ /*!

  @brief           Computes the tangent of x with accuracy of about 3.2 decimal digits

  @description     This is the main tangent approximation "driver".
				   It reduces the input argument's range to [0, pi/4],
				   and then calls the approximator.
				   WARNING: We do not test for the tangent approaching
				   infinity,  which it will at x=pi/2 and x=3*pi/2.
				   If this is a problem in your application, take
				   appropriate action.

  @param[in]       f_angle : the angle for which we want to know the
							 tangent, radians
							 Supported values are [Full range of float32]
							 except ((2*n) + 1)*(0.5f * TUE_CML_Pi), n is any integer.

  @return          the tangent of f_angle
*****************************************************************************/
float32 TUE_CML_GDB_tan32(float32 f_angle) {
    /*--- VARIABLES ---*/
    float32 f_tan;          /*!< return value */
    uint32 u_octant;        /*!< what octant are we in? */
    boolean b_sign = FALSE; /*!< TRUE, if arg was < 0 */
    float32 f_tmp;

    if (f_angle < 0.0f) {
        f_angle = -f_angle;
        b_sign = TRUE;
    }

    /* linit to two pi */
    if (f_angle > (10.0f * TUE_CML_Pi)) {
        f_angle = TUE_CML_ModTrig(f_angle, (2.0f * TUE_CML_Pi));
    } else {
        while (f_angle >= (2.0f * TUE_CML_Pi)) {
            f_angle -= (2.0f * TUE_CML_Pi);
        }
    }

    /*! Get octant # (0 to 7) */
    f_tmp = f_angle * (4.0f / TUE_CML_Pi);
    u_octant = (uint32)f_tmp;

    switch (u_octant) {
        case 1:
            f_tan = 1.0f / TUE_CML_GDB_tan32s((((0.5f * TUE_CML_Pi) - f_angle) *
                                               (4.0f / TUE_CML_Pi)));
            break;
        case 2:
            f_tan =
                -1.0f / TUE_CML_GDB_tan32s(((f_angle - (0.5f * TUE_CML_Pi)) *
                                            (4.0f / TUE_CML_Pi)));
            break;
        case 3:
            f_tan = -TUE_CML_GDB_tan32s(
                ((TUE_CML_Pi - f_angle) * (4.0f / TUE_CML_Pi)));
            break;
        case 4:
            f_tan = TUE_CML_GDB_tan32s(
                ((f_angle - TUE_CML_Pi) * (4.0f / TUE_CML_Pi)));
            break;
        case 5:
            f_tan = 1.0f / TUE_CML_GDB_tan32s((((1.5f * TUE_CML_Pi) - f_angle) *
                                               (4.0f / TUE_CML_Pi)));
            break;
        case 6:
            f_tan =
                -1.0f / TUE_CML_GDB_tan32s(((f_angle - (1.5f * TUE_CML_Pi)) *
                                            (4.0f / TUE_CML_Pi)));
            break;
        case 7:
            f_tan = -TUE_CML_GDB_tan32s(
                (((2.0f * TUE_CML_Pi) - f_angle) * (4.0f / TUE_CML_Pi)));
            break;
        default:
            /*Case 0*/
            f_tan = TUE_CML_GDB_tan32s((f_angle * (4.0f / TUE_CML_Pi)));
            break;
    }

    if (b_sign == TRUE) {
        f_tan = -f_tan;
    }

    return (f_tan);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDB_tan32s */ /*!

  @brief           Computes tan(pi *x/4)

  @description     Accurate to about 3.2 decimal digits over the range [0, pi/4].
				   Note that the function computes tan(pi*x/4),
				   NOT tan(x); it's up to the range reduction algorithm that
				   calls this to scale things properly.
				   Algorithm:    tan(x)= x*c1/(c2 + x^2)

  @param[in]       f_angle : the angle (times pi/4) for which we want to know
							 the tangent, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE is square root of max value of float32

  @return          tan(pi*f_angle/4)
*****************************************************************************/
float32 TUE_CML_GDB_tan32s(float32 f_angle) {
    /*--- VARIABLES ---*/
    float32 f_angle_square;

    f_angle_square = TUE_CML_Sqr(f_angle);
    return ((f_angle * C_TAN_32_C1) / (C_TAN_32_C2 + f_angle_square));
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBcos_52 */ /*!

  @brief           Calculates the cosine with 5.2 decimals accuracy

  @description     It reduces the input argument's range to [0, pi/2],
				   and then performs the approximation.
				   Algorithm:
				   cos(x)= c1 + c2*x^2 + c3*x^4 + c4*x^6
				   which is the same as:
				   cos(x)= c1 + x^2(c2 + c3*x^2 + c4*x^4)
				   cos(x)= c1 + x^2(c2 + x^2(c3 + c4*x^2))

  @param[in]       f_angle : angle for which cosine has to be found
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE = [max range of uint32] * CML_f_two_Pi

  @return          cosine of f_angle (double)
*****************************************************************************/
float32 TUE_CML_GDBcos_52(float32 f_angle) {
    /*--- VARIABLES ---*/
    uint32 u_quad;
    float32 f_angle_square, f_tmp;
    float32 f_resultValue; /* result value */

    if (f_angle < 0.0F) {
        f_angle = -f_angle;
    }

    /* limit to two pi */
    if (f_angle > (10.0f * TUE_CML_Pi)) {
        f_angle = TUE_CML_ModTrig(f_angle, (2.0f * TUE_CML_Pi));
    } else {
        /* sensible argument, use faster while loop */
        while (f_angle >= (2.0f * TUE_CML_Pi)) {
            f_angle -= (2.0f * TUE_CML_Pi);
        }
    }

    f_tmp = f_angle * (2.0f / TUE_CML_Pi);
    u_quad = (uint32)f_tmp;
    switch (u_quad) {
        case 1:
            f_angle_square = (TUE_CML_Pi - f_angle) * (TUE_CML_Pi - f_angle);
            f_resultValue =
                -(C_COS_52_C1 +
                  (f_angle_square *
                   (C_COS_52_C2 +
                    (f_angle_square *
                     (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
            break;
        case 2:
            f_angle_square = (f_angle - TUE_CML_Pi) * (f_angle - TUE_CML_Pi);
            f_resultValue =
                -(C_COS_52_C1 +
                  (f_angle_square *
                   (C_COS_52_C2 +
                    (f_angle_square *
                     (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
            break;
        case 3:
            f_angle_square = ((2.0f * TUE_CML_Pi) - f_angle) *
                             ((2.0f * TUE_CML_Pi) - f_angle);
            f_resultValue =
                (C_COS_52_C1 +
                 (f_angle_square *
                  (C_COS_52_C2 +
                   (f_angle_square *
                    (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
            break;
        default:
            /*Case 0*/
            f_angle_square = f_angle * f_angle;
            f_resultValue =
                (C_COS_52_C1 +
                 (f_angle_square *
                  (C_COS_52_C2 +
                   (f_angle_square *
                    (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
            break;
    }

    return f_resultValue;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBsin_52 */ /*!

  @brief           Calculates the sine with 5.2 decimals accuracy

  @description     The sine is just cosine shifted a half-pi,
				   so we'll adjust the argument and call the cosine approximation.

  @param[in]       f_angle : input angle for which we would like to know the
							 sine, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE =
							 ([max range of uint32] * CML_f_two_Pi)-(0.5f * TUE_CML_Pi)

  @return          the sine of f_angle
*****************************************************************************/
float32 TUE_CML_GDBsin_52(float32 f_angle) {
    return TUE_CML_GDBcos_52((0.5f * TUE_CML_Pi) - f_angle);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBatan_66 */ /*!

  @brief           computes atan(x) with about 6.6 decimal digits accuracy

  @description     The input argument's range is reduced to [0, pi/12]
				   before the approximation takes place
				   Algorithm: atan(x)= x(c1 + c2*x^2)/(c3 + x^2)

  @param[in]       f_tan : the "secant length" for which we want to know the
						   corresponding angle, radians
						   Optimal values are [-MAX_ANGLE,..,MAX_ANGLE],
						   where MAX_ANGLE is square root of max value of float32

  @return          arctangent of f_tan
*****************************************************************************/
float32 TUE_CML_GDBatan_66(float32 f_tan) {
    /*--- VARIABLES ---*/
    float32 f_angle;              /* return from atan__s function */
    float32 f_tan_square;         /* The input argument squared */
    boolean b_complement = FALSE; /* TRUE if arg was >1 */
    boolean b_region = FALSE;     /* TRUE depending on region arg is in */
    boolean b_sign = FALSE;       /* TRUE if arg was < 0 */

    /* reduce input argument */
    if (f_tan < 0.0F) {
        f_tan = -f_tan;
        b_sign = TRUE; /* argtan(-x) = - arctan(x) */
    }
    if (f_tan > 1.0F) {
        f_tan = 1.0F / f_tan;
        b_complement = TRUE; /* keep arg between 0 and 1 */
    }
    if (f_tan > C_TANTWELFTHPI) {
        f_tan = ((f_tan - C_TANSIXTHPI) /
                 (1.F +
                  (C_TANSIXTHPI * f_tan))); /* reduce arg to under tan(pi/12) */
        b_region = TRUE;
    }

    /* do the approximation on the reduced argument */
    f_tan_square = TUE_CML_Sqr(f_tan);
    f_angle = (f_tan * (C_ATAN_66_C1 + (f_tan_square * C_ATAN_66_C2))) /
              (C_ATAN_66_C3 + f_tan_square);

    /* put result back together */
    if (b_region == TRUE) {
        f_angle += C_SIXTHPI; /* correct for region we are in */
    }
    if (b_complement == TRUE) {
        f_angle =
            (0.5f * TUE_CML_Pi) - f_angle; /* correct for 1/x if we did that */
    }
    if (b_sign == TRUE) {
        f_angle = -f_angle; /* correct for negative arg */
    }

    return (f_angle);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBacos_66 */ /*!

  @brief           implements the acos() function with 6.6 decimals of accuracy

  @description     This function uses the relationships between trigonomtric
				   and inverse trigonometric functions.
				   tan(arccos x) = sqrt(1 - x^2) / x
				   tan(arcsin x) = x / sqrt(1 - x^2)

  @param[in]       f_cos : value for which we want the inverse cosinus
						   Ideal values are [-1,..,0,..,1]

  @return          arccosinus corresponding to the value f_cos, in radians
*****************************************************************************/
float32 TUE_CML_GDBacos_66(float32 f_cos) {
    float32 f_angle; /* result value */

    /*! catch invalid input ranges and prevent division by zero below */
    if (f_cos >= 1.0F) {
        f_angle = 0.0F;
    }

    else if (f_cos <= -1.0F) {
        f_angle = TUE_CML_Pi;
    }

    else {
        f_angle =
            (0.5f * TUE_CML_Pi) -
            TUE_CML_GDBatan_66(
                f_cos / TUE_CML_SqrtApprox(1.0F - TUE_CML_SqrtApprox(f_cos)));
    }

    return f_angle;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBasin_66 */ /*!

  @brief           implements the asin() function with 6.6 decimals of accuracy

  @description     This function uses the relationships between trigonomtric
				   and inverse trigonometric functions.
				   tan(arccos x) = sqrt(1 - x^2) / x
				   tan(arcsin x) = x / sqrt(1 - x^2)

  @param[in]       f_sin : value for which we want the inverse sinus
						   Ideal values are [-1,..,0,..,1]

  @return          arcsinus corresponding to the value f_sin, in radians
*****************************************************************************/
float32 TUE_CML_GDBasin_66(float32 f_sin) {
    float32 f_angle; /* result value */

    /*! catch invalid input ranges and prevent division by zero below */
    if (f_sin >= 1.0F) {
        f_angle = (0.5f * TUE_CML_Pi);
    }

    else if (f_sin <= -1.0F) {
        f_angle = -(0.5f * TUE_CML_Pi);
    }

    else {
        f_angle = TUE_CML_GDBatan_66(
            f_sin / TUE_CML_SqrtApprox(1.0F - TUE_CML_Sqr(f_sin)));
    }

    return f_angle;
}

/*****************************************************************************
  Functionname:    TUE_CML_BoundedLinInterpol                 */ /*!

  @brief           bounded linear interpolation between two given points

  @description     This function computes the bounded linear interpolation value
				   between two given points. The minimum and maximum boundary
				   values are taken from the input structure.

  @param[in]       p_Params : structure for parameters
							  Range for p_Params->dAmin is [Full range of float32]
							  Range for p_Params->dAmax is [Full range of float32]
							  Range for p_Params-> dM is [Full range of float32]
							  Range for p_Params-> dB is [Full range of float32]
							  Overflow may occur if all the input values to the function
							  are at maximum possible value at the same time.

  @param[in]       f_Value : x-value to interpolate
							 [Full range of float32]

  @return          the bounded interpolated value


*****************************************************************************/
float32 TUE_CML_BoundedLinInterpol(
    TUE_CML_t_LinFunctionArgs const* const p_Params, const float32 f_Value) {
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
} /* TUE_CML_BoundedLinInterpol() */

/*****************************************************************************
  Functionname:    TUE_CML_BoundedLinInterpol2 */ /*!

  @brief

  @description

  @param[in]

  @return
*****************************************************************************/
float32 TUE_CML_BoundedLinInterpol2(float32 f_IVal,
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
} /* BML_f_BoundedLinInterpol2() */

/*****************************************************************************
  Functionname:    TUE_CML_Round2IntGen */ /*!

  @brief          

  @description     

  @param[in]      

  @return         
*****************************************************************************/
sint32 TUE_CML_Round2IntGen(float32 x) {
    return (x >= 0.f) ? (sint32)(x + 0.5f) : (sint32)(x - 0.5f);
}

/*****************************************************************************
  Functionname:    TUE_CML_Round2FloatGen */ /*!

  @brief

  @description

  @param[in]

  @return
*****************************************************************************/
float32 TUE_CML_Round2FloatGen(float32 x) {
    return (x >= 0.f) ? (float32)(sint32)(x + 0.5f)
                      : (float32)(sint32)(x - 0.5f);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBatan2_66                                        */ /*!

  @brief           computes the four-quadrant atan(y/x) with
				   about 6.6 decimal digits accuracy

  @description     This function computes the four-quandrant arctangent with
				   about 6.6 decimal digits accuracy.
				   The input arguments are x and y. The situation y=0 is
				   handled correctly.

  @param[in]       f_xaxis : any number
							 Optimal values are [-MAX_ANGLE,..,MAX_ANGLE]
  @param[in]       f_yaxis : any number
							 Optimal values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE is cube root of max value of float32

  @return          the four-quadrant arctangent of f_yaxis/f_xaxis in
				   radians [-Pi, Pi]
				   if x=0 and y=0 the result is 0

*****************************************************************************/
float32 TUE_CML_GDBatan2_66(float32 f_yaxis, float32 f_xaxis) {
    float32 f_angle;

    /* handle x = 0 */
    if (f_xaxis > TUE_CML_AlmostZero) {
        /* compute arctangent */
        f_angle = TUE_CML_GDBatan_66(f_yaxis / f_xaxis);
    }

    else {
        if (f_xaxis < TUE_CML_AlmostNegZero) {
            /* compute arctangent */
            f_angle = TUE_CML_GDBatan_66(f_yaxis / -f_xaxis);

            if (f_yaxis < TUE_CML_AlmostNegZero) {
                f_angle = -TUE_CML_Pi - f_angle;
            }

            else {
                f_angle = TUE_CML_Pi - f_angle;
            }
        }

        else {
            if (f_yaxis < TUE_CML_AlmostNegZero) {
                f_angle = -TUE_CML_Pi / 2.0F;
            }

            else if (f_yaxis > TUE_CML_AlmostZero) {
                f_angle = TUE_CML_Pi / 2.0F;
            }

            else {
                f_angle = 0.0F;
            }
        }
    }

    return f_angle;
}

/*****************************************************************************
  Functionname: SafeDiv                                  */ /*!

  @brief: Return a value which is verified not zero regardless of sign

  @description: Return a value which is verified not zero or close to zero regardless of sign

  @param[in]

  @return
*****************************************************************************/
float32 SafeDiv(float32 fDivisor) {
    if (TUE_CML_Abs(fDivisor) < TUE_C_F32_DELTA) {
        if (fDivisor < 0.f) {
            fDivisor = -TUE_C_F32_DELTA;
        } else {
            fDivisor = TUE_C_F32_DELTA;
        }
    }
    return fDivisor;
}

/*****************************************************************************
  Functionname:    SimulationObjectGenerator */ /*!

  @brief           Computes object distance and velocity value based on
				   defined object trajectory

  @description     we will calculate predicated object's detect value based on
				   defined object's trajectory. the target of this function is
				   used for unit test of object related functions.

  @param[in]        fObjTrajHeadingAngle_rad: object trajectory's heading angle of clothoid equation
					fObjTrajCurve_1pm: object trajectory's curve of clothoid equation
					fObjTrajCurveDer_nu: object trajectory's curve derivative of clothoid equation
					fObjInitDistX0_met: object initial distance X
					fObjInitDistY0_met: object inital distance Y
					fObjAbsVelX_mps: object absolute velocity X
					fEgoVelX_mps: ego velocity X
					fPassedTime_sec: passed time since object inital point

  @param[out]
					fOutObjDistX_met: object distance X in current time set
					fOutObjDistY_met: object distance Y in current time set
					fOutObjRelVelX_mps: object velocity X in current time set
					fOutObjRelVelY_mps: object velocity Y in current time set
  @return

*****************************************************************************/
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
                               float32* fOutObjRelVelY_mps) {
    *fOutObjRelVelX_mps = fObjAbsVelX_mps - fEgoVelX_mps;
    *fOutObjRelVelY_mps = 0.f;
    *fOutObjDistX_met =
        fObjInitDistX0_met + *fOutObjRelVelX_mps * fPassedTime_sec;
    *fOutObjDistY_met =
        fObjInitDistY0_met +
        (TUE_CML_GDBtan_52(fObjTrajHeadingAngle_rad)) * (*fOutObjDistX_met) +
        (0.5f * (*fOutObjDistX_met) * (*fOutObjDistX_met) * fObjTrajCurve_1pm) +
        ((1.0f / 6.0f) * (*fOutObjDistX_met) * (*fOutObjDistX_met) *
         (*fOutObjDistX_met) * fObjTrajCurveDer_nu);
}

/*****************************************************************************
  Functionname: TUE_CML_TimerRetrigger                                  */ /*!

  @brief: the function is designed for the TIMERRETRIGGER module

  @description: we will set a timer for the specific DeltaTime and TimeLimit, 
			the RemainTime would go back to TimeLimie value while reset is TRUE, 
			and the function output is TRUE while RemainTime higher than zero.
			TIMERRETRIGGER_RE

  @param[in]
			float32 fDeltaTime_sec; time reduce value for every invoke
			boolean bReset: reset timer flag
			float32 fTimeLimit_sec: default time upper limit
  @param[out]
			float32* fRemainTime_sec: remain time value for this timer
  @return: 
*****************************************************************************/
boolean TUE_CML_TimerRetrigger(float32 fDeltaTime_sec,
                               boolean bReset,
                               float32 fTimeLimit_sec,
                               float32* fRemainTime_sec) {
    if (fDeltaTime_sec < 0.f) {
        return FALSE;
    }
    if (bReset) {
        *fRemainTime_sec = fTimeLimit_sec;
    } else {
        *fRemainTime_sec = *fRemainTime_sec > fDeltaTime_sec
                               ? (*fRemainTime_sec - fDeltaTime_sec)
                               : 0.f;
    }
    return *fRemainTime_sec > 0.f;
}

/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/

REAL32_T TUE_CML_Limit_M(REAL32_T fInput, REAL32_T fMin, REAL32_T fMax) {
    if (fMax >= fMin) {
        // 正常情况
        if (fInput > fMax) {
            return fMax;
        } else if (fInput < fMin) {
            return fMin;
        } else {
            return fInput;
        }
    } else {
        // 错误限值情况
        if (fInput > fMin) {
            return fMin;
        } else if (fInput < fMax) {
            return fMax;
        } else {
            return fInput;
        }
    }
}

REAL32_T TUE_CML_GradLimit_M(REAL32_T fInput,
                             REAL32_T fUpperLimit,
                             REAL32_T fLowerLimit,
                             REAL32_T fSysTime,
                             REAL32_T fLastOutput) {
    REAL32_T fTemp = 0.0F;

    fTemp = (fInput - fLastOutput) / fSysTime;

    if (fUpperLimit > fLowerLimit) {
        if (fTemp > fUpperLimit) {
            fTemp = fUpperLimit;
        } else if (fTemp < fLowerLimit) {
            fTemp = fLowerLimit;
        } else {
        }
    } else {
        if (fTemp < fUpperLimit) {
            fTemp = fUpperLimit;
        } else if (fTemp > fLowerLimit) {
            fTemp = fLowerLimit;
        } else {
        }
    }

    fTemp = fTemp * fSysTime + fLastOutput;

    return fTemp;
}

REAL32_T TUE_CML_DivProtection_M(REAL32_T fNumerator,
                                 REAL32_T fDenominator,
                                 REAL32_T fDefault) {
    REAL32_T fTemp = 0.0F;

    if (fDenominator > 1E-15F) {
        fTemp = fNumerator / fDenominator;
    } else {
        fTemp = fDefault;
    }

    return fTemp;
}

UINT8_T TUE_CML_Hysteresis_M(REAL32_T fInput,
                             REAL32_T fThresHigh,
                             REAL32_T fThresLow,
                             UINT8_T bLastOutput) {
    UINT8_T fResTemp = 0U;

    if (fThresHigh > fThresLow) {
        if ((bLastOutput == 0U) && (fInput > fThresHigh)) {
            fResTemp = 1U;
        } else if ((bLastOutput == 1U) && (fInput < fThresLow)) {
            fResTemp = 0U;
        } else {
            fResTemp = bLastOutput;
        }
    } else {
        if ((bLastOutput == 0U) && (fInput > fThresLow)) {
            fResTemp = 1U;
        } else if ((bLastOutput == 1U) && (fInput < fThresHigh)) {
            fResTemp = 0U;
        } else {
            fResTemp = bLastOutput;
        }
    }

    return fResTemp;
}

UINT8_T TUE_CML_TurnOnDelay_M(UINT8_T bInput,
                              REAL32_T fDelayTime,
                              REAL32_T fCycleTime,
                              REAL32_T* Timer,
                              UINT8_T bLastOutput) {
    if (bInput == 1U) {
        if (bLastOutput == bInput) {
            *Timer = 0;
        } else if (*Timer < fDelayTime) {
            *Timer = (*Timer + fCycleTime);
        } else {
            bLastOutput = bInput;
            *Timer = 0;
        }
    } else if (bLastOutput == 0) {
        *Timer = 0;
    } else if (*Timer < 0) /* Not use */
    {
        *Timer = (*Timer + fCycleTime);
    } else {
        bLastOutput = 0;
        *Timer = 0;
    }

    return bLastOutput;
} /*TUE_CML_TurnOnDelay */

UINT8_T TUE_CML_TurnOffDelay_M(UINT8_T bInput,
                               REAL32_T fDelayTime,
                               REAL32_T fCycleTime,
                               REAL32_T* Timer,
                               UINT8_T bLastOutput) {
    if (bInput == 1) /* Not use */
    {
        if (bLastOutput == bInput) {
            *Timer = 0;
        } else if (*Timer < 0) {
            *Timer = (*Timer + fCycleTime);
        } else {
            bLastOutput = bInput;
            *Timer = 0;
        }
    } else if (bLastOutput == 0) {
        *Timer = 0;
    } else if (*Timer < fDelayTime) {
        *Timer = (*Timer + fCycleTime);
    } else {
        bLastOutput = 0;
        *Timer = 0;
    }

    return bLastOutput;
} /*TUE_CML_TurnOffDelay */

UINT8_T TUE_CML_SRTrigger_M(UINT8_T bSet, UINT8_T bReset, UINT8_T bLastOutput) {
    UINT8_T bTemp = 0U;

    if (bReset == 1U) {
        bTemp = 0U;
    } else {
        if ((bSet == 1U) || (bLastOutput == 1U)) {
            bTemp = 1U;
        } else {
            bTemp = 0U;
        }
    }

    return bTemp;
}

REAL32_T TUE_CML_PosY3rd_M(REAL32_T fPosX,
                           REAL32_T fPosY0,
                           REAL32_T fYaw,
                           REAL32_T fCrv,
                           REAL32_T fCrvRate) {
    fYaw = TUE_CML_Limit_M(fYaw, -0.7854F, 0.7854F);
    return ((fPosY0) + TUE_CML_Tan_M(fYaw) * (fPosX) +
            (fCrv) * (fPosX) * (fPosX) * (0.5F) +
            (fCrvRate) * (fPosX) * (fPosX) * (fPosX) / 6.0F);
}

REAL32_T TUE_CML_Yaw3rd_M(REAL32_T fPosX,
                          REAL32_T fYaw,
                          REAL32_T fCrv,
                          REAL32_T fCrvRate) {
    fYaw = TUE_CML_Limit_M(fYaw, -0.7854F, 0.7854F);
    return (TUE_CML_Tan_M(fYaw) + (fCrv) * (fPosX) +
            (fCrvRate) * (fPosX) * (fPosX)*0.5F);
}

REAL32_T TUE_CML_Crv3rd_M(REAL32_T fPosX, REAL32_T fCrv, REAL32_T fCrvRate) {
    return ((fCrv) + (fCrvRate) * (fPosX));
}

void TUE_CML_Interp2_M(const REAL32_T fX[],
                       const REAL32_T fY[],
                       REAL32_T PolyCoeff[]) {
    REAL32_T fCoeff1 = 0.0F;
    REAL32_T fCoeff2 = 0.0F;
    REAL32_T fCoeff3 = 0.0F;

    fCoeff1 = fY[0];
    fCoeff2 = (fY[1] - fY[0]) / (fX[1] - fX[0]);
    fCoeff3 =
        (fY[2] - fY[1]) / (fX[2] - fX[1]) - (fY[1] - fY[0]) / (fX[1] - fX[0]);
    fCoeff3 = fCoeff3 / (fX[2] - fX[0]);

    PolyCoeff[0] = fCoeff3;
    PolyCoeff[1] = fCoeff2 - (fX[0] + fX[1]) * fCoeff3;
    PolyCoeff[2] = fCoeff1 - fCoeff2 * fX[0] - fCoeff3 * fX[0] * fX[1];
}

/*****************************************************************************
  Functionname:    TUE_CML_ModTrig                                        */ /*!

  @brief           Calculates the remainder of x when divided by y as needed
					by trigonometric functions

  @description     This function calculates the remainder of x when divided by y
				   Works only for y > 0
				   The function is equivalent to rem() function in Matlab.


  @param[in]       f_dividend : The Dividend
								Supported values are [Full range of REAL32_T]
								Overflow may occur at higher values.
  @param[in]       f_divisor  : The Divisor
								Supported values are [Full range of REAL32_T]
								Overflow may occur at very small values.

  @return          remainder of f_dividend when divided by f_divisor

*****************************************************************************/
REAL32_T TUE_CML_ModTrig_M(REAL32_T fDividend, REAL32_T fDivisor) {
    UINT32_T TUE_CML_LONG_MAX_P = 2147483647L;
    REAL32_T TUE_CML_ModuloEps_P = 0.0000001F;

    REAL32_T f_quotient, f_ret;
    INT32_T s_quotient;

    if (fDivisor < TUE_CML_ModuloEps_P) {
        f_ret = 0.f;
    } else {
        f_quotient = fDividend / fDivisor;

        if (TUE_CML_Abs_M(f_quotient) > (REAL32_T)TUE_CML_LONG_MAX_P) {
            f_ret = 0.f;
        } else {
            s_quotient = (INT32_T)(f_quotient);
            f_ret = (fDividend - ((REAL32_T)s_quotient * fDivisor));
        }
    }

    return f_ret;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBcos_52 */ /*!

  @brief           Calculates the cosine with 5.2 decimals accuracy

  @description     It reduces the input argument's range to [0, pi/2],
				   and then performs the approximation.
				   Algorithm:
				   cos(x)= c1 + c2*x^2 + c3*x^4 + c4*x^6
				   which is the same as:
				   cos(x)= c1 + x^2(c2 + c3*x^2 + c4*x^4)
				   cos(x)= c1 + x^2(c2 + x^2(c3 + c4*x^2))

  @param[in]       f_angle : angle for which cosine has to be found
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE = [max range of uint32] * CML_f_two_Pi

  @return          cosine of f_angle (double)
*****************************************************************************/
REAL32_T TUE_CML_Cos_M(REAL32_T fAngle) {
    /*--- VARIABLES ---*/
    UINT32_T u_quad;
    REAL32_T f_angle_square, f_tmp;
    REAL32_T f_resultValue; /* result value */
    // REAL32_T TUE_CML_Pi = 3.14159265359F;
    // REAL32_T C_COS_52_C1 = 0.99999329F;
    // REAL32_T C_COS_52_C2 = -0.49991243F;
    // REAL32_T C_COS_52_C3 = 0.04148774F;
    // REAL32_T C_COS_52_C4 = -0.00127120F;
    REAL32_T fTemp = 0.0F;

    if (fAngle < 0.0F) {
        fAngle = -fAngle;
    }

    /* limit to two pi */
    if (fAngle > (10.0f * TUE_CML_Pi)) {
        fTemp = 2.0F * TUE_CML_Pi;
        fAngle = TUE_CML_ModTrig_M(fAngle, fTemp);
    } else {
        /* sensible argument, use faster while loop */
        while (fAngle >= (2.0f * TUE_CML_Pi)) {
            fAngle -= (2.0f * TUE_CML_Pi);
        }
    }

    f_tmp = fAngle * (2.0f / TUE_CML_Pi);
    u_quad = (UINT32_T)f_tmp;
    switch (u_quad) {
        case 1:
            f_angle_square = (TUE_CML_Pi - fAngle) * (TUE_CML_Pi - fAngle);
            f_resultValue =
                -(C_COS_52_C1 +
                  (f_angle_square *
                   (C_COS_52_C2 +
                    (f_angle_square *
                     (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
            break;
        case 2:
            f_angle_square = (fAngle - TUE_CML_Pi) * (fAngle - TUE_CML_Pi);
            f_resultValue =
                -(C_COS_52_C1 +
                  (f_angle_square *
                   (C_COS_52_C2 +
                    (f_angle_square *
                     (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
            break;
        case 3:
            f_angle_square =
                ((2.0f * TUE_CML_Pi) - fAngle) * ((2.0f * TUE_CML_Pi) - fAngle);
            f_resultValue =
                (C_COS_52_C1 +
                 (f_angle_square *
                  (C_COS_52_C2 +
                   (f_angle_square *
                    (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
            break;
        default:
            /*Case 0*/
            f_angle_square = fAngle * fAngle;
            f_resultValue =
                (C_COS_52_C1 +
                 (f_angle_square *
                  (C_COS_52_C2 +
                   (f_angle_square *
                    (C_COS_52_C3 + (C_COS_52_C4 * f_angle_square))))));
            break;
    }

    return f_resultValue;
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBsin_52 */ /*!

  @brief           Calculates the sine with 5.2 decimals accuracy

  @description     The sine is just cosine shifted a half-pi,
				   so we'll adjust the argument and call the cosine approximation.

  @param[in]       f_angle : input angle for which we would like to know the
							 sine, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE =
							 ([max range of uint32] * CML_f_two_Pi)-(0.5f * TUE_CML_Pi)

  @return          the sine of f_angle
*****************************************************************************/
REAL32_T TUE_CML_Sin_M(REAL32_T fAngle) {
    /*REAL32_T TUE_CML_Pi = 3.14159265359F;*/
    return TUE_CML_Cos_M((0.5F * TUE_CML_Pi) - fAngle);
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBtan_56s */ /*!

  @brief           computes tan(pi*x/4)

  @description     Accurate to about 5.6 decimal digits over
				   the range [0, pi/4].
				   Note that the function computes tan(pi*x/4),
				   NOT tan(x); it's up to the range
				   reduction algorithm that calls this to scale
				   things properly.
				   Algorithm: tan(x)= x(c1 + c2*x^2)/(c3 + x^2)

  @param[in]       f_angle : the angle (times pi/4) for which we want to know
							 the tangent, radians
							 Supported values are [-MAX_ANGLE,..,MAX_ANGLE],
							 where MAX_ANGLE is cube root of max value of REAL32_T

  @return          parameter for tan_56(f_angle)

  @pre             GDBtan_56(x)

*****************************************************************************/
REAL32_T TUE_CML_GDBtan_56s_M(REAL32_T f_angle) {
    REAL32_T TUE_CML_TAN_56_C1_P = -3.16783027F;
    REAL32_T TUE_CML_TAN_56_C2_P = 0.13451612F;
    REAL32_T TUE_CML_TAN_56_C3_P = -4.03332198F;

    return (f_angle * (TUE_CML_TAN_56_C1_P +
                       (TUE_CML_TAN_56_C2_P * (f_angle * f_angle)))) /
           (TUE_CML_TAN_56_C3_P + (f_angle * f_angle));
}

/*****************************************************************************
  Functionname:    TUE_CML_GDBtan_52 */ /*!

  @brief           Computes the tangent of x with accuracy of about 5.6
				   decimal digits

  @description     This is the main tangent approximation "driver".
				   It reduces the input argument's range to [0, pi/4],
				   and then calls the approximator.
					WARNING: We do not test for the tangent approaching
				   infinity,  which it will at x=pi/2 and x=3*pi/2.
				   If this is a problem in your application, take
				   appropriate action.

  @param[in]       f_angle : the angle for which we want to know the
							 tangent, radians
							 Supported values are [Full range of REAL32_T]
							 except ((2*n) + 1)*(0.5f*TUE_CML_Pi), n is any integer.

  @return          the tangent of f_angle

*****************************************************************************/
REAL32_T TUE_CML_Tan_M(REAL32_T fAngle) {
    /*--- VARIABLES ---*/
    REAL32_T TUE_CML_Pi_P = 3.14159265359F;
    REAL32_T f_tan, f_tmp;
    UINT32_T u_octant;
    UINT8_T b_sign = 0U;
    REAL32_T fTemp = 0.0F;

    if (fAngle < 0.0F) {
        fAngle = -fAngle;
        b_sign = 1U;
    }

    /* limit to two pi */
    if (fAngle > (10.0F * TUE_CML_Pi_P)) {
        fTemp = 2.0f * TUE_CML_Pi_P;
        fAngle = TUE_CML_ModTrig_M(fAngle, fTemp);
    } else {
        while (fAngle >= (2.0f * TUE_CML_Pi_P)) {
            fAngle -= (2.0f * TUE_CML_Pi);
        }
    }

    f_tmp = fAngle * (4.0f / TUE_CML_Pi_P);
    u_octant = (UINT32_T)f_tmp;

    switch (u_octant) {
        case 1:
            f_tan =
                1.0F / TUE_CML_GDBtan_56s_M(((0.5f * TUE_CML_Pi_P) - fAngle) *
                                            (4.0f / TUE_CML_Pi_P));
            break;
        case 2:
            f_tan =
                -1.0F / TUE_CML_GDBtan_56s_M((fAngle - (0.5f * TUE_CML_Pi_P)) *
                                             (4.0f / TUE_CML_Pi_P));
            break;
        case 3:
            f_tan = -TUE_CML_GDBtan_56s_M((TUE_CML_Pi_P - fAngle) *
                                          (4.0f / TUE_CML_Pi_P));
            break;
        case 4:
            f_tan = TUE_CML_GDBtan_56s_M((fAngle - TUE_CML_Pi_P) *
                                         (4.0f / TUE_CML_Pi_P));
            break;
        case 5:
            f_tan =
                1.0F / TUE_CML_GDBtan_56s_M(((1.5f * TUE_CML_Pi_P) - fAngle) *
                                            (4.0f / TUE_CML_Pi_P));
            break;
        case 6:
            f_tan =
                -1.0F / TUE_CML_GDBtan_56s_M((fAngle - (1.5f * TUE_CML_Pi_P)) *
                                             (4.0f / TUE_CML_Pi_P));
            break;
        case 7:
            f_tan = -TUE_CML_GDBtan_56s_M(((2.0f * TUE_CML_Pi_P) - fAngle) *
                                          (4.0f / TUE_CML_Pi_P));
            break;
        default:
            /*Case 0*/
            f_tan = TUE_CML_GDBtan_56s_M(fAngle * (4.0f / TUE_CML_Pi_P));
            break;
    }

    if (b_sign == 1U) {
        f_tan = -f_tan;
    }

    return f_tan;
}

/*****************************************************************************
  Functionname:    TUE_CML_SqrtApprox                                 */ /*!

  @brief           approx calculate of square root

  @description     approx calculate of square root

  @param[in]       f_radicand : input square value

  @return          square result of input value


*****************************************************************************/
REAL32_T TUE_CML_Sqrt_M(REAL32_T fInput) {
    INT32_T s_expo;
    UINT32_T u_tmp;
    REAL32_T f_Sample, f_SampleSquare;
    REAL32_T f_A, f_B, f_A_plus_B;
    REAL32_T f_ret;
    UINT32_T TUE_CML_SqrtApprox_NumExpo_P = 23U;
    UINT32_T TUE_CML_SqrtApprox_MantissaMask_P = (0x007fffffu);
    UINT32_T TUE_CML_SqrtApprox_ExponentOffset_P = (0x7f);
    REAL32_T TUE_CML_SqrtApprox_AlmostZero_P = (1e-20f);
    // REAL32_T TUE_CML_GaussianCDFMinSigma_P = 0.000001f;
    // REAL32_T TUE_CML_SQRT_OF_2_P = (1.414213562373095f);/* square root of 2
    // */
    /*! union for doing bit-wise manipulation on the floating point
     * representation */

    union {
        REAL32_T f_value; /* this is the number of interest as float */
        UINT32_T u_value; /* in here we hold the same number as int on which we
                             can do bit-wise manipulations */
    } x_tmp;

    /* check for x < 0 or x = NaN */
    if (fInput < 0) {
        return 0.f;
    }

    /* copy input value to union where we can manipulate it directly */
    x_tmp.f_value = fInput;

    /* check for negative or zero or denormalized */
    if (((x_tmp.u_value >> 31) > 0U)      /* sign bit set? */
        || ((x_tmp.u_value >> 23) == 0u)) /* exponent is zero? */
    {
        f_ret = 0.f;
    }
    /* check for infinity */
    else if ((x_tmp.u_value >> 23) >= 0xffu) {
        x_tmp.u_value = 0x7f7fffffu;
        f_ret = x_tmp.f_value; /* return max_float */
    } else {
        /* calculate start value by dividing exponent by two */
        u_tmp = x_tmp.u_value >> TUE_CML_SqrtApprox_NumExpo_P;
        s_expo = (INT32_T)(u_tmp);
        s_expo -= TUE_CML_SqrtApprox_ExponentOffset_P;
        if (s_expo < 0) {
            s_expo >>= 1; /* without the following line this would be unsafe
                             because right shift of an unsigned int is machine
                             dependent (sign fill vs. zero fill) */
            s_expo |=
                (INT32_T)0x80000000U; /* -> fill highest bit with sign (one)
                                         after shift to make it safe */
        } else {
            s_expo >>=
                1; /* for positive expo sign = 0 -> sign fill = zero fill */
        }
        s_expo += TUE_CML_SqrtApprox_ExponentOffset_P;
        x_tmp.u_value &= TUE_CML_SqrtApprox_MantissaMask_P;
        x_tmp.u_value += ((UINT32_T)s_expo << TUE_CML_SqrtApprox_NumExpo_P);
        f_Sample = x_tmp.f_value;

        /* two iterations are enough */
        /* iteration one */
        f_SampleSquare =
            f_Sample * f_Sample;  // f_SampleSquare = TUE_CML_Sqr(f_Sample);
        f_A = 2.f * (f_SampleSquare + fInput);
        f_B = f_SampleSquare - fInput;
        f_A_plus_B = f_A + f_B;
        if (TUE_CML_Abs_M(f_A_plus_B) < TUE_CML_SqrtApprox_AlmostZero_P) {
            f_ret = 0.f;
        } else {
            f_Sample *= ((f_A - f_B) / (f_A_plus_B));
            /* iteration two */
            f_SampleSquare = f_Sample * f_Sample;
            // f_SampleSquare = TUE_CML_Sqr(f_Sample);
            f_A = 2.f * (f_SampleSquare + fInput);
            f_B = f_SampleSquare - fInput;

            f_A_plus_B = f_A + f_B;
            f_Sample *= ((f_A - f_B) / (f_A_plus_B));
            f_ret = f_Sample;
        }
    }

    return f_ret;
}

// 内存拷贝函数
// 调用参数
//      FromPtr：        源数据区指针；
//      ToPtr：          目标数据区指针；
//      ByteCnt：        数据字节数（小于32767）；
// 函数输出
//      无。
void TUE_CML_MemoryCopy_M(INT8_T* FromPtr, INT8_T* ToPtr, INT16_T ByteCnt) {
    // 不进行任何检查，直接按字节赋值
    while (ByteCnt > 0) {
        *ToPtr = *FromPtr;
        ToPtr++;
        FromPtr++;
        ByteCnt--;
    }
}

/*****************************************************************************
  @fn				TUE_CML_HysteresisFloat */ /*!

  @description		In >= fThresHigh + fHyst,                    bOutput = 0;
							In <= fThresLow - fHyst,                      bOutput = 0;
							fThresLow<In<fThresHigh,                   bOutput = 0;
							fThresLow - fHyst <In<fThresLow,       bOutput = Last output;
							fThresHigh <In<fThresHigh + fHyst,    bOutput = Last output;

  @param[in]		fInput		     Input to limit
  @param[in]		fThresHigh	 High threshold
  @param[in]		fThresLow 	 Low threshold
  @param[in]		fHyst 	         Hysteresis value
  @param[in]	    bPrevOutput  Previous output of Hysteresis

  @return            bPrevOutput Hysteresis output
*****************************************************************************/
UINT8_T TUE_CML_BilateralHysteresis(REAL32_T fInput,
                                    REAL32_T fThresHigh,
                                    REAL32_T fThresLow,
                                    REAL32_T fHyst,
                                    UINT8_T bPrevOutput) {
    if ((fInput < fThresHigh) && (fInput > fThresLow)) {
        bPrevOutput = 1;
    } else if ((fInput >= fThresHigh + fHyst) ||
               (fInput <= fThresLow - fHyst)) {
        bPrevOutput = 0;
    } else {
        // bOutput = Last output;
    }
    return bPrevOutput;
}  // TUE_CML_BilateralHysteresis

// const volatile real32_T HOD_VelXVeh_X[8] = {
// 0.0F, 8.33F, 16.66F, 25.0F, 33.33F,
//  41.66F, 50.0F, 58.33F };        /* Referenced by: '<S36>/1-D Lookup Table'
//  */
// HOD_CoeffVelXFlt = TUE_CML_LookUpTable2D(HOD_VelXVeh, ((const REAL32_T*)
//    &(HOD_VelXVeh_X[0])), ((const REAL32_T*)&(HOD_CoeffVelXLowPass_M[0])),
//    7U);
REAL32_T TUE_CML_LookUpTable2D(REAL32_T u0,
                               const REAL32_T bp0[],
                               const REAL32_T table[],
                               UINT32_T maxIndex) {
    REAL32_T frac;
    UINT32_T iRght;
    UINT32_T iLeft;
    UINT32_T bpIdx;

    maxIndex = maxIndex - 1U;

    /* Column-major Lookup 1-D
       Search method: 'binary'
       Use previous index: 'off'
       Interpolation method: 'Linear point-slope'
       Extrapolation method: 'Clip'
       Use last breakpoint for index at or above upper limit: 'off'
       Remove protection against out-of-range input in generated code: 'off'
     */
    /* Pre lookup - Index and Fraction
           Index Search method: 'binary'
           Extrapolation method: 'Clip'
           Use previous index: 'off'
           Use last breakpoint for index at or above upper limit: 'off'
           Remove protection against out-of-range input in generated code: 'off'
     */
    if (u0 <= bp0[0U]) {
        iLeft = 0U;
        frac = 0.0F;
    } else if (u0 < bp0[maxIndex]) {
        /* Binary Search */
        bpIdx = maxIndex >> 1U;
        iLeft = 0U;
        iRght = maxIndex;
        while (iRght - iLeft > 1U) {
            if (u0 < bp0[bpIdx]) {
                iRght = bpIdx;
            } else {
                iLeft = bpIdx;
            }

            bpIdx = (iRght + iLeft) >> 1U;
        }

        frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
    } else {
        iLeft = maxIndex - 1U;
        frac = 1.0F;
    }

    /* Column-major Interpolation 1-D
       Interpolation method: 'Linear point-slope'
       Use last breakpoint for index at or above upper limit: 'off'
       Overflow mode: 'portable wrapping'
     */
    return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

REAL32_T TUE_CML_LowPassFilter_M(REAL32_T fInput,
                                 REAL32_T fTimeFilter,
                                 REAL32_T fTimeSys,
                                 REAL32_T fLastOutput) {
    if ((fTimeFilter < 1E-5F) || (fTimeFilter < fTimeSys)) {
        return fInput;
    } else {
        return fLastOutput + (fInput - fLastOutput) * fTimeSys / fTimeFilter;
    }
}

/*****************************************************************************
  Functionname:    TUE_CML_AddMatrices_M                                    */ /*!

  @brief           Matrix addition (in place/out place). A or B can be same as Res

  @description     This function performs matrix addition (inplace/outplace) of
				   two matrices A and B with same dimesions and store the result
				   in a resultant matrix.
				   The matrix A or B can be same as resultant matrix.
				   [Res] = [A] + [B]

  @param[in]       p_MatrixA : First Addend matrix (source)
							   Range for p_MatrixA->Desc.row [Full range of uint8]
							   Range for p_MatrixA->Desc.col [Full range of uint8]
							   Range for p_MatrixA->Desc.maxsize [Full range of uint16]
							   Range for p_MatrixA->pData
							   [Valid pointer with data in full range of REAL32_T]
  @param[in]       p_MatrixB : Second Addend matrix (source)
							   Range for p_MatrixB->Desc.row [Full range of uint8]
							   Range for p_MatrixB->Desc.col [Full range of uint8]
							   Range for p_MatrixB->Desc.maxsize [Full range of uint16]
							   Range for p_MatrixB->pData
							   [Valid pointer with data in full range of REAL32_T]
							   Overflow may occur when one or more input values in both
							   matrices are at the defined range extremities.

  @param[out]      p_MatrixRes : Result Sum matrix

  @return          void

*****************************************************************************/
void TUE_CML_AddMatrices_M(TUE_CML_sMatrix_t* p_MatrixRes,
                           const TUE_CML_sMatrix_t* p_MatrixA,
                           const TUE_CML_sMatrix_t* p_MatrixB) {
    UINT32_T u_Idx;
    UINT32_T u_size =
        (UINT32_T)p_MatrixA->Desc.col * (UINT32_T)p_MatrixA->Desc.row;

    REAL32_T* p_DataA = p_MatrixA->pData; /* get pointer to matrix data */
    REAL32_T* p_DataB = p_MatrixB->pData; /* get pointer to matrix data */
    REAL32_T* p_DataRes =
        p_MatrixRes->pData; /* get pointer to result matrix data */

#if CML_MatrixBoundsCheckOn
    /* check if matrices are equal and Res is big enough */
    if ((p_MatrixA->Desc.col == p_MatrixB->Desc.col) &&
        (p_MatrixA->Desc.row == p_MatrixB->Desc.row) &&
        (p_MatrixRes->Desc.maxsize >= u_size)) {
#endif

        /* add elements */
        if (p_DataA == p_DataRes) {
            for (u_Idx = 0u; u_Idx < u_size; u_Idx++) {
                p_DataRes[u_Idx] += p_DataB[u_Idx];
            }
        } else if (p_DataB == p_DataRes) {
            for (u_Idx = 0u; u_Idx < u_size; u_Idx++) {
                p_DataRes[u_Idx] += p_DataA[u_Idx];
            }
        } else {
            for (u_Idx = 0u; u_Idx < +u_size; u_Idx++) {
                p_DataRes[u_Idx] = p_DataA[u_Idx] + p_DataB[u_Idx];
            }
        }

        /* create description for result matrix */
        p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
        p_MatrixRes->Desc.row = p_MatrixA->Desc.row;

#if CML_MatrixBoundsCheckOn
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;
        CML_ASSERT(b_0U);
    }
#endif
}

/*****************************************************************************
  Functionname:    TUE_CML_CopyMatrix_M                                     */ /*!

  @brief           Matrix copy

  @description     This function copies data from one matrix to another.

  @param[in]       p_MatrixA : matrix to be copied
							   Range for p_MatrixA->Desc.row [Full range of uint8]
							   Range for p_MatrixA->Desc.col [Full range of uint8]
							   Range for p_MatrixA->Desc.maxsize [Full range of uint16]
							   Range for p_MatrixA->pData
							   [Valid pointer with data in full range of REAL32_T]
  @param[out]      p_MatrixRes : destination matrix
								 Range for p_MatrixRes->Desc.maxsize [Full range of uint16]

  @return          void

*****************************************************************************/
void TUE_CML_CopyMatrix_M(TUE_CML_sMatrix_t* p_MatrixRes,
                          const TUE_CML_sMatrix_t* p_MatrixA) {
    UINT32_T u_Idx;
    UINT32_T size =
        (UINT32_T)p_MatrixA->Desc.col * (UINT32_T)p_MatrixA->Desc.row;

    REAL32_T* p_DataA = p_MatrixA->pData; /* get pointer to matrix data */
    REAL32_T* p_DataRes =
        p_MatrixRes->pData; /* get pointer to result matrix data */

#if CML_MatrixBoundsCheckOn
    if (p_MatrixRes->Desc.maxsize >= size) {
#endif

        /* copy elements */
        for (u_Idx = 0u; u_Idx < size; u_Idx++) {
            p_DataRes[u_Idx] = p_DataA[u_Idx];
        }

        /* create description for result matrix */
        p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
        p_MatrixRes->Desc.row = p_MatrixA->Desc.row;

#if CML_MatrixBoundsCheckOn
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

void TUE_CML_TransposeMatrix_M(TUE_CML_sMatrix_t* p_MatrixRes,
                               const TUE_CML_sMatrix_t* p_MatrixA) {
    UINT32_T u_Idx2, u_Idx1;

    REAL32_T* p_DataA = p_MatrixA->pData; /* get pointer to matrix data */
    REAL32_T* p_DataRes =
        p_MatrixRes->pData; /* get pointer to result matrix data */

#if CML_MatrixBoundsCheckOn
    /* check if source is different from destination */
    if ((p_MatrixA->pData != p_MatrixRes->pData) &&
        (p_MatrixRes->Desc.maxsize >=
         p_MatrixA->Desc.col * p_MatrixA->Desc.row)) {
#endif

        /* transpose while copying */
        for (u_Idx1 = 0UL; u_Idx1 < p_MatrixA->Desc.row; u_Idx1++) {
            for (u_Idx2 = 0UL; u_Idx2 < p_MatrixA->Desc.col; u_Idx2++) {
                p_DataRes[u_Idx1 + (u_Idx2 * (UINT32_T)p_MatrixA->Desc.row)] =
                    *p_DataA;
                p_DataA++;
            }
        }
        /* create description for result matrix */
        p_MatrixRes->Desc.col = p_MatrixA->Desc.row;
        p_MatrixRes->Desc.row = p_MatrixA->Desc.col;

#if CML_MatrixBoundsCheckOn
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

/*****************************************************************************
  Functionname:    TUE_CML_SubtractMatrices_M                               */ /*!

  @brief           Matrix substraction Res = A - B

  @description     This function performs matrix subtraction (inplace/outplace)
				   of two matrices A and B with same dimesions and store the result
				   in a resultant matrix.
				   [Res] = [A] - [B]

  @param[in]       p_MatrixA : Minuend matrix
							   Range for p_MatrixA->Desc.row [Full range of uint8]
							   Range for p_MatrixA->Desc.col [Full range of uint8]
							   Range for p_MatrixA->Desc.maxsize [Full range of uint16]
							   Range for p_MatrixA->pData
							   [Valid pointer with data in full range of REAL32_T]
  @param[in]       p_MatrixB : Subtrahend matrix
							   Range for p_MatrixB->Desc.row [Full range of uint8]
							   Range for p_MatrixB->Desc.col [Full range of uint8]
							   Range for p_MatrixB->Desc.maxsize [Full range of uint16]
							   Range for p_MatrixB->pData
							   [Valid pointer with data in full range of REAL32_T]
							   Overflow may occur when one or more input values in both
							   matrices are at the defined range extremities.
  @param[out]      p_MatrixRes : Result Difference matrix

  @return          void

*****************************************************************************/
void TUE_CML_SubtractMatrices_M(TUE_CML_sMatrix_t* p_MatrixRes,
                                const TUE_CML_sMatrix_t* p_MatrixA,
                                const TUE_CML_sMatrix_t* p_MatrixB) {
    UINT32_T u_Idx;
    UINT32_T u_size =
        (UINT32_T)p_MatrixA->Desc.col * (UINT32_T)p_MatrixA->Desc.row;

    REAL32_T* p_DataA = p_MatrixA->pData; /* get pointer to matrix data */
    REAL32_T* p_DataB = p_MatrixB->pData; /* get pointer to matrix data */
    REAL32_T* p_DataRes =
        p_MatrixRes->pData; /* get pointer to result matrix data */

#if CML_MatrixBoundsCheckOn
    /* check if matrices are equal */
    if ((p_MatrixA->Desc.col == p_MatrixB->Desc.col) &&
        (p_MatrixA->Desc.row == p_MatrixB->Desc.row) &&
        (p_MatrixRes->Desc.maxsize >=
         p_MatrixA->Desc.col * p_MatrixA->Desc.row)) {
#endif

        /* substract elements */
        if (p_DataA == p_DataRes) {
            for (u_Idx = 0u; u_Idx < u_size; u_Idx++) {
                p_DataRes[u_Idx] -= p_DataB[u_Idx];
            }
        } else {
            for (u_Idx = 0u; u_Idx < u_size; u_Idx++) {
                p_DataRes[u_Idx] = p_DataA[u_Idx] - p_DataB[u_Idx];
            }
        }

        /* create description for result matrix */
        p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
        p_MatrixRes->Desc.row = p_MatrixA->Desc.row;

#if CML_MatrixBoundsCheckOn
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

/*****************************************************************************
  Functionname:    TUE_CML_ScaleMatrix_M                                    */ /*!

  @brief           Matrix multiplication with scalar

  @description     This function does an inplace matrix multiplication
				   with a given scalar. If [A] is the matrix, and p is
				   the scalar, then,
				   [A] = p * [A]

  @param[in,out]   p_MatrixA : Matrix o be multiplied
							   Range for p_MatrixA->Desc.row [Full range of uint8]
							   Range for p_MatrixA->Desc.col [Full range of uint8]
							   Range for p_MatrixA->pData
							   [Valid pointer with data in full range of REAL32_T]
							   Overflow may occur when one or more input values in both
							   matrices are at the defined range extremities.
  @param[in]       f_Val :     scalar
							   [Full range of REAL32_T]

  @return          void

*****************************************************************************/
void TUE_CML_ScaleMatrix_M(TUE_CML_sMatrix_t* p_MatrixA, REAL32_T f_Val) {
    UINT32_T u_Idx;
    UINT32_T u_size =
        (UINT32_T)p_MatrixA->Desc.col * (UINT32_T)p_MatrixA->Desc.row;
    REAL32_T* p_DataA; /* get pointer to matrix data */

    /* scale elements */
    p_DataA = p_MatrixA->pData;
    for (u_Idx = 0u; u_Idx < u_size; u_Idx++) {
        p_DataA[u_Idx] *= f_Val;
    }
}

/*****************************************************************************
  Functionname:    TUE_CML_MutiplyMatrices_M                               */ /*!

  @brief           Matrix multiplication

  @description     This function performs matrix multiplication (outplace)
				   of two matrices A and B and store the result in a
				   resultant matrix.
				   [Res] = [A] X [B]

  @param[in]       p_MatrixA : Multiplicand matrix (source)
							   Range for p_MatrixA->Desc.row [Full range of uint8]
							   Range for p_MatrixA->Desc.col [Full range of uint8]
							   Range for p_MatrixA->Desc.maxsize [Full range of uint16]
							   Range for p_MatrixA->pData
							   [Valid pointer with data in full range of REAL32_T]
  @param[in]       p_MatrixB : Multiplier matrix (source)
							   Range for p_MatrixB->Desc.row [Full range of uint8]
							   Range for p_MatrixB->Desc.col [Full range of uint8]
							   Range for p_MatrixB->Desc.maxsize [Full range of uint16]
							   Range for p_MatrixB->pData
							   [Valid pointer with data in full range of REAL32_T]
							   Overflow may occur when one or more input values in both
							   matrices are at the defined range extremities.
  @param[out]      p_MatrixRes : Result Product matrix

  @return          void

*****************************************************************************/
void TUE_CML_MutiplyMatrices_M(TUE_CML_sMatrix_t* p_MatrixRes,
                               const TUE_CML_sMatrix_t* p_MatrixA,
                               const TUE_CML_sMatrix_t* p_MatrixB) {
    UINT8_T u_RegAcol = p_MatrixA->Desc.col;
    UINT8_T u_RegBcol = p_MatrixB->Desc.col;
    UINT8_T u_RegArow = p_MatrixA->Desc.row;
    UINT32_T u_count;
    REAL32_T* p_DataA;
    REAL32_T* p_DataB;
    REAL32_T* p_LineA = p_MatrixA->pData;
    REAL32_T* p_LineB = p_MatrixB->pData;
    REAL32_T* p_LineBk;
    REAL32_T* p_DataRes = p_MatrixRes->pData; /* get pointer to matrix data */
    REAL32_T f_Tmp;

#if CML_MatrixBoundsCheckOn
    /* check if matrix dimensions fit */
    /* and matrices are distinct */
    if ((p_MatrixA->Desc.col != (uint8)0) &&
        (p_MatrixA->Desc.col == p_MatrixB->Desc.row) &&
        (p_MatrixA->pData != p_MatrixRes->pData) &&
        (p_MatrixB->pData != p_MatrixRes->pData) &&
        (p_MatrixRes->Desc.maxsize >=
         p_MatrixB->Desc.col * p_MatrixA->Desc.row)) {
#endif

        if ((u_RegAcol > 0U) && (u_RegBcol > 0U) && (u_RegArow > 0U)) {
            /* multiply matrix elements */
            u_count = u_RegArow;
            do {
                p_LineBk = p_LineB;
                do {
                    REAL32_T f_DataA;
                    REAL32_T f_DataB;

                    p_DataA = p_LineA;
                    p_DataB = p_LineBk;
                    p_LineBk++;
                    f_Tmp = 0.0F;
                    do {
                        f_DataA = *p_DataA;
                        f_DataB = *p_DataB;
                        p_DataA++;
                        p_DataB += u_RegBcol; /* goto next line */
                        /* Floating point multiply and add: y = a * b + d */
                        f_Tmp = TUE_CML_MultAddGen_M(f_DataA, f_DataB, f_Tmp);

                        /* <ln_offset:+2 MISRA Rule 17.2: reviewer name: Daniel
                         * Meschenmoser date: 2012-09-12 reason: matrix
                         * multiplication runtime optimized by Uwe-Juergen
                         * Zunker */
                        /* <ln_offset:+1 MISRA Rule 17.3: reviewer name: Daniel
                         * Meschenmoser date: 2012-09-12 reason: matrix
                         * multiplication runtime optimized by Uwe-Juergen
                         * Zunker */
                    } while (p_DataA < (p_LineA + u_RegAcol));
                    *p_DataRes = f_Tmp;
                    p_DataRes++; /* go to next pRes element */
                    /* <ln_offset:+2 MISRA Rule 17.2: reviewer name: Daniel
                     * Meschenmoser date: 2012-09-12 reason: matrix
                     * multiplication runtime optimized by Uwe-Juergen Zunker */
                    /* <ln_offset:+1 MISRA Rule 17.3: reviewer name: Daniel
                     * Meschenmoser date: 2012-09-12 reason: matrix
                     * multiplication runtime optimized by Uwe-Juergen Zunker */
                } while (p_LineBk < (p_LineB + u_RegBcol));
                p_LineA += (UINT32_T)u_RegAcol; /* go to next line of A */

                u_count--;
            } while (u_count > 0UL);
        } else {
            /* error */
        }

        /* create description for result matrix */
        p_MatrixRes->Desc.col = u_RegBcol;
        p_MatrixRes->Desc.row = u_RegArow;

#if CML_MatrixBoundsCheckOn
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (uint8)0;
        p_MatrixRes->Desc.row = (uint8)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

/*****************************************************************************
  Functionname:    TUE_CML_InitMatrix_M                                      */ /*!

  @brief           Matrix initialization with a const value

  @description     This function initializes all the elements of the matrix
				   with a const value.
				   NOTE: RowNr and ColNr are expected to be values not
				   exceeding 8 bits.
				   CAUTION: u_RowNr*u_ColNr must not exceed A->Desc.maxsize

  @param[in,out]   p_Matrix :  matrix o be filled
							   Range for p_Matrix->Desc.maxsize [Full range of uint16]
  @param[out]      f_Val :     value used for filling
							   [Full range of REAL32_T]
  @param[in]       u_RowNr :   Row dimension of the matrix to be created
							   [Full range of uint8]
  @param[in]       u_ColNr :   Column dimension of the matrix to be created
							   [Full range of uint8]

  @return          void

*****************************************************************************/
void TUE_CML_InitMatrix_M(TUE_CML_sMatrix_t* p_Matrix,
                          UINT32_T u_RowNr,
                          UINT32_T u_ColNr,
                          REAL32_T f_Val) {
    UINT32_T u_Idx;
    UINT32_T u_size = u_ColNr * u_RowNr;
    REAL32_T* p_MatrixData; /* pointer to matrix data */

#if CML_MatrixBoundsCheckOn
    if (p_Matrix->Desc.maxsize >= u_size) {
#endif

        /* set new dimension */
        p_Matrix->Desc.col = (UINT8_T)u_ColNr;
        p_Matrix->Desc.row = (UINT8_T)u_RowNr;

        /* init elements */
        p_MatrixData = p_Matrix->pData;
        for (u_Idx = 0u; u_Idx < u_size; u_Idx++) {
            p_MatrixData[u_Idx] = f_Val;
        }

#if CML_MatrixBoundsCheckOn
    } else {
        /* Return empty matrix */
        p_Matrix->Desc.col = (UINT8_T)0;
        p_Matrix->Desc.row = (UINT8_T)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

///< Calculating the square (x*x) of a number

/*****************************************************************************
  Functionname:    TUE_CML_CreateIdentityMatrix_M                           */ /*!

  @brief           Initializes matrix with identity matrix

  @description     This function initializes the given matrix with an
				   identity matrix of the provided size.
				   NOTE: Value for row/column is expected not to exceed 8 bits
				   CAUTION: Size of the matrix (rows x columns) must not
				   exceed A->Desc.maxsize

  @param[in,out]   p_Matrix :  matrix o be filled (square matrix)
							   Range for p_Matrix->Desc.maxsize [Full range of uint16]
  @param[in]       u_Size :    no. of row/col square matrix (u_Size x u_Size)
							   [Full range of uint8]

  @return          void

*****************************************************************************/
void TUE_CML_CreateIdentityMatrix_M(TUE_CML_sMatrix_t* p_Matrix,
                                    UINT32_T u_Size) {
    UINT32_T u_Idx;
    UINT32_T u_SizeSquare = u_Size * u_Size;
    REAL32_T* p_MatrixData; /* pointer to matrix data */

#if CML_MatrixBoundsCheckOn
    if (p_Matrix->Desc.maxsize >= u_SizeSquare) {
#endif

        /* set new dimension */
        p_Matrix->Desc.col = (UINT8_T)u_Size;
        p_Matrix->Desc.row = (UINT8_T)u_Size;

        /* fill with zero */
        TUE_CML_InitMatrix_M(p_Matrix, u_Size, u_Size, 0.0F);

        /* set diagonal to one */
        p_MatrixData = p_Matrix->pData;
        for (u_Idx = 0u; u_Idx < u_SizeSquare;
             u_Idx += ((UINT32_T)p_Matrix->Desc.col + 1u)) {
            p_MatrixData[u_Idx] = 1.0F;
        }

#if CML_MatrixBoundsCheckOn
    } else {
        /* Set empty matrix */
        p_Matrix->Desc.col = (UINT8_T)0;
        p_Matrix->Desc.row = (UINT8_T)0;
        CML_ASSERT(b_FALSE);
    }
#endif
}

/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/
/********************************************************************/

REAL32_T TUE_CML_MultAddGen_M(REAL32_T a, REAL32_T b, REAL32_T d) {
#if defined(_MSC_VER)
    REAL32_T f_tmp1, f_tmp2;
    f_tmp1 = a * b;
    f_tmp2 = f_tmp1 + d;
    return f_tmp2;
#else
    return (a * b) + d;
#endif
}

/*****************************************************************************
  Functionname:    TUE_CML_InvertMatrixCramer2_M                        */ /*!

  @brief           Compute matrix inverse for matrix size 2x2

  @description     This function compute matrix inverse: Res = inv(A)
				   Uses Cramers Rule for matrix size 2x2.
				   If A = |a b| and det(A) = (ad-bc),
						  |c d|
				   det(A) should be a non-zero value, then,
				   Inverse of A, inv(A) = (1/det(A))* | d -b|
													  |-c  a|

  @param[in,out]   a_in : matrix to be inversed.
						  [Full range of float32]
					   ATTENTION! This matrix is overwritten with the identity matrix.
  @param[in,out]   a_res : result matrix, containing the inverse of A

  @return          void

*****************************************************************************/
UINT8_T TUE_CML_InvertMatrixCramer2_M(REAL32_T a_res[4], REAL32_T a_in[4]) {
    REAL32_T f_temp = 0.0f; /* temporary variable */
    UINT8_T b_ret = 0U;     /* return value */

    /* Cramers Rule for matrix size == 2 */
    f_temp = (a_in[0] * a_in[3]) - (a_in[1] * a_in[2]);
    if (TUE_CML_IsNonZero_M(f_temp))  // GeGr: Modified precision
    {
        f_temp = 1.0F / f_temp;

        a_res[0] = a_in[3] * f_temp;
        a_res[1] = -a_in[1] * f_temp;
        a_res[2] = -a_in[2] * f_temp;
        a_res[3] = a_in[0] * f_temp;

        b_ret = 1U;
    } /* if(TUE_CML_IsNonZero_M(f_temp)) */

    return b_ret;
} /* TUE_CML_InvertMatrixCramer2_M() */

/*****************************************************************************
  Functionname:    TUE_CML_InvertMatrixCramer3_M */ /*!

  @brief           Compute matrix inverse for matrix size 3x3

  @description     This function compute matrix inverse: Res = inv(A)
				   Uses Cramers Rule for matrix size 3x3.
				   If A is a 3x3 matrix and det(A) is the determinant of matrix,
				   which must be non-zero, then inverse of A will be equal to
				   Adjoint matrix of A divided by det(A).

  @param[in,out]   a_in : matrix to be inversed.
						  [Full range of float32]
					   ATTENTION! This matrix is overwritten with the identity matrix.
  @param[in,out]   a_res : result matrix, containing the inverse of A

  @return          void

*****************************************************************************/
UINT8_T TUE_CML_InvertMatrixCramer3_M(REAL32_T a_res[9], REAL32_T a_in[9]) {
    REAL32_T f_temp = 0.0f; /* temporary variable */
    UINT8_T b_ret = 0U;     /* return value */

    /* Cramers Rule for matrix size == 3 */
    f_temp = (((a_in[0] * a_in[4]) - (a_in[3] * a_in[1])) * a_in[8]) +
             (((a_in[3] * a_in[7]) - (a_in[6] * a_in[4])) * a_in[2]) +
             (((a_in[6] * a_in[1]) - (a_in[0] * a_in[7])) * a_in[5]);

    if (TUE_CML_IsNonZero_M(f_temp))  // AlFe: Modified precision
    {
        f_temp = 1.0F / f_temp;

        a_res[0] = ((a_in[4] * a_in[8]) - (a_in[5] * a_in[7])) * f_temp;
        a_res[1] = ((a_in[2] * a_in[7]) - (a_in[1] * a_in[8])) * f_temp;
        a_res[2] = ((a_in[1] * a_in[5]) - (a_in[2] * a_in[4])) * f_temp;
        a_res[3] = ((a_in[5] * a_in[6]) - (a_in[3] * a_in[8])) * f_temp;
        a_res[4] = ((a_in[0] * a_in[8]) - (a_in[2] * a_in[6])) * f_temp;
        a_res[5] = ((a_in[2] * a_in[3]) - (a_in[0] * a_in[5])) * f_temp;
        a_res[6] = ((a_in[3] * a_in[7]) - (a_in[4] * a_in[6])) * f_temp;
        a_res[7] = ((a_in[1] * a_in[6]) - (a_in[0] * a_in[7])) * f_temp;
        a_res[8] = ((a_in[0] * a_in[4]) - (a_in[1] * a_in[3])) * f_temp;

        b_ret = 1U;
    } /* if(TUE_CML_IsNonZero_M(f_temp)) */

    return b_ret;
} /* TUE_CML_InvertMatrixCramer3_M() */

UINT8_T TUE_CML_CalcInvertMatrix_M(TUE_CML_sMatrix_t* p_MatrixA,
                                   REAL32_T* p_DataA,
                                   REAL32_T* p_DataRes) {
    UINT8_T bRet = 1U;
    UINT32_T u_Idx1, u_Idx2, u_col, u_row, u_pos1, u_pos2;
    REAL32_T f_Temp, f_MaxElem;
    REAL32_T f_PivElem = 1.0F;
    REAL32_T f_InvPivElem = 1.0F;
    const REAL32_T f_Tol = 1e-20F;
    /* tolerance */  // StAR - avoid division by zero

    u_col = 0UL;
    do {
        /* find largest element on the selected column */
        /* and use as pivot element                    */
        u_row = u_col;
        u_pos1 = u_col + (u_col * (UINT32_T)p_MatrixA->Desc.col);
        f_MaxElem = 0.0F;
        for (u_Idx1 = u_col; u_Idx1 < (UINT32_T)p_MatrixA->Desc.col; u_Idx1++) {
            f_Temp = TUE_CML_Abs_M(p_DataA[u_pos1]);
            if (f_Temp > f_MaxElem) {
                f_MaxElem = f_Temp;
                f_PivElem = p_DataA[u_pos1];
                u_row = u_Idx1;
            }
            u_pos1 += (UINT32_T)p_MatrixA->Desc.col;
        }

        /* exit routine if pivot element is very small => matrix not
         * inversible */
        if (f_MaxElem >= f_Tol) {
            /* do pivoting to reduce column to identity matrix */
            bRet = 1U;

            /* now swap rows to put the pivot element on the diagonal */
            /* do the same operation for the result matrix */
            if (u_row != u_col) {
                /* get pointer to matrix data */
                u_pos1 = (UINT32_T)p_MatrixA->Desc.col * u_row;
                u_pos2 = (UINT32_T)p_MatrixA->Desc.col * u_col;

                for (u_Idx1 = u_col; u_Idx1 < (UINT32_T)p_MatrixA->Desc.col;
                     u_Idx1++) /* only nonzero elements */
                {
                    f_Temp = p_DataA[u_Idx1 + u_pos1];
                    p_DataA[u_Idx1 + u_pos1] = p_DataA[u_Idx1 + u_pos2];
                    p_DataA[u_Idx1 + u_pos2] = f_Temp;
                }
                for (u_Idx1 = 0U; u_Idx1 < (UINT32_T)p_MatrixA->Desc.col;
                     u_Idx1++) /* all elements */
                {
                    f_Temp = p_DataRes[u_Idx1 + u_pos1];
                    p_DataRes[u_Idx1 + u_pos1] = p_DataRes[u_Idx1 + u_pos2];
                    p_DataRes[u_Idx1 + u_pos2] = f_Temp;
                }
            }

            /* divide row by the pivot element => pivot becomes 1 */
            /* do the same operation for the result matrix */
            u_pos1 = u_col * (UINT32_T)p_MatrixA->Desc.col;
            f_InvPivElem = 1.0F / f_PivElem;
            for (u_Idx1 = u_col; u_Idx1 < (UINT32_T)p_MatrixA->Desc.col;
                 u_Idx1++) /* only nonzero elements */
            {
                p_DataA[u_Idx1 + u_pos1] *= f_InvPivElem;
            }
            for (u_Idx1 = 0UL; u_Idx1 < (UINT32_T)p_MatrixA->Desc.col;
                 u_Idx1++) /* all elements */
            {
                p_DataRes[u_Idx1 + u_pos1] *= f_InvPivElem;
            }

            /* now multiply the row by the right amount and substract
             * from    */
            /* each other row to make all the remaining elements in the
             * pivot */
            /* column zero */
            for (u_Idx1 = 0UL; u_Idx1 < (UINT32_T)p_MatrixA->Desc.col;
                 u_Idx1++) /* loop other rows */
            {
                if (u_Idx1 != u_col) {
                    u_pos1 = u_Idx1 * (UINT32_T)p_MatrixA->Desc.col;
                    u_pos2 = u_col * (UINT32_T)p_MatrixA->Desc.col;

                    /* use first element is row as scaling coefficient
                     */
                    f_Temp = p_DataA[u_col + u_pos1];

                    /* substract pivot row multiplied by scaling from
                     * other row */
                    /* do the same operation for the result matrix */
                    for (u_Idx2 = u_col; u_Idx2 < (UINT32_T)p_MatrixA->Desc.col;
                         u_Idx2++) /* only nonzero elements */
                    {
                        p_DataA[u_Idx2 + u_pos1] -=
                            p_DataA[u_Idx2 + u_pos2] * f_Temp;
                    }
                    for (u_Idx2 = 0UL; u_Idx2 < (UINT32_T)p_MatrixA->Desc.col;
                         u_Idx2++) /* all elements */
                    {
                        p_DataRes[u_Idx2 + u_pos1] -=
                            p_DataRes[u_Idx2 + u_pos2] * f_Temp;
                    }
                }
            }

            /* goto next column */
            u_col++;
        }

        else {
            bRet = 0U;
        }
    } while (
        bRet &&
        (u_col < (UINT32_T)p_MatrixA->Desc.col)); /* quit if finished or if
                                                     matrix isn't inversible
                                                     */
    return bRet;
}

/*****************************************************************************
  Functionname:    CML_v_InvertMatrix                                   */ /*!

  @brief           Compute matrix inverse.

  @description     Compute matrix inverse: Res = inv(A)
				   Uses Gauss-Jordan elimination with partial pivoting.
				   For matrices upto 3x3, determinant is found, singularity is
				   checked and processing is done, whereas for higher order
				   matrices, matrix singularity is determined during the
				   processing.

  @param[in,out]   p_MatrixA : matrix to be inversed.
							   Range for p_MatrixA->Desc.row [Full range of uint8]
							   Range for p_MatrixA->Desc.col [Full range of uint8]
							   Range for p_MatrixA->Desc.maxsize [Full range of uint16]
							   Range for p_MatrixA->pData
							   [Valid pointer with data in full range of float32]
				   The largest element on the each column MUST BE greater than
				   the tolerance value (1e-10F). Otherwise function call will
				   result in assertion fail.
				   ATTENTION! This matrix is overwritten with the identity matrix.
  @param[in,out]   p_MatrixRes : result matrix, containing the inverse of A

  @return          void

*****************************************************************************/
void TUE_CML_InvertMatrix_M(TUE_CML_sMatrix_t* p_MatrixRes,
                            TUE_CML_sMatrix_t* p_MatrixA) {
    UINT8_T bRet = 0U;
    REAL32_T* p_DataA = p_MatrixA->pData;     /* get pointer to matrix data */
    REAL32_T* p_DataRes = p_MatrixRes->pData; /* get pointer to matrix data */

#if CML_MatrixBoundsCheckOn
    /* check if matrix is square */
    if ((p_MatrixA->Desc.col != (uint8)0) &&
        (p_MatrixA->Desc.col == p_MatrixA->Desc.row) &&
        (p_MatrixA->pData != p_MatrixRes->pData) &&
        (p_MatrixRes->Desc.maxsize >=
         (p_MatrixA->Desc.col * p_MatrixA->Desc.row))) {
#endif

        if (p_MatrixA->Desc.col == (UINT8_T)1U) {
            /* simple division for matrix size == 1 */
            if (TUE_CML_IsNonZero_M(p_DataA[0])) {
                *p_DataRes = 1.0F / (*p_DataA);
                bRet = 1U;
            }
        } else if (p_MatrixA->Desc.col == (UINT8_T)2U) {
            /* Cramers Rule for matrix size == 2 */
            bRet = TUE_CML_InvertMatrixCramer2_M(p_DataRes, p_DataA);
        } else if (p_MatrixA->Desc.col == (UINT8_T)3U) {
            /* Cramers Rule for matrix size == 3 */
            bRet = TUE_CML_InvertMatrixCramer3_M(p_DataRes, p_DataA);
        } else /* (A->Desc.col > (uint8)2U) */
        {
            /* Gauss-Jordan elimination with partial pivoting */

            bRet = 1U;
            TUE_CML_CreateIdentityMatrix_M(
                p_MatrixRes, (UINT32_T)p_MatrixA->Desc
                                 .row); /* set result matrix as identity */
            bRet = TUE_CML_CalcInvertMatrix_M(p_MatrixA, p_DataA, p_DataRes);
        }

#if CML_MatrixBoundsCheckOn
    }
#endif

    if (bRet) {
        /* create description for result matrix */
        p_MatrixRes->Desc.col = p_MatrixA->Desc.col;
        p_MatrixRes->Desc.row = p_MatrixA->Desc.row;
    } else {
        /* set empty matrix */
        p_MatrixRes->Desc.col = (UINT8_T)0;
        p_MatrixRes->Desc.row = (UINT8_T)0;
        /* Deactivate QA-C warning 3112; Reviewer: S. Schwarzkopf;
           Date: 04.12.2014; Reason: macro will be routed to assert() in
           simulation environment. Review-ID: 3942463 */
        /* PRQA S 3112 1 */
        // CML_ASSERT(FALSE);
    }
}

/********************************Tuerme common algorithm
 * library*************************************/

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static UINT8_T calculatePolyfit1st(TUE_CML_sMatrix_t* Weight,
                                   TUE_CML_sMatrix_t* mtrxX,
                                   TUE_CML_sMatrix_t* vecY,
                                   TUE_CML_sMatrix_t* vecPolyCoeff1st);
static UINT8_T calculatePolyfit2nd(TUE_CML_sMatrix_t* Weight,
                                   TUE_CML_sMatrix_t* mtrxX,
                                   TUE_CML_sMatrix_t* vecY,
                                   TUE_CML_sMatrix_t* vecPolyCoeff2nd);
static UINT8_T calculatePolyfit3rd(TUE_CML_sMatrix_t* Weight,
                                   TUE_CML_sMatrix_t* mtrxX,
                                   TUE_CML_sMatrix_t* vecY,
                                   TUE_CML_sMatrix_t* vecPolyCoeff3rd);
static void createEquationMatrix1st(TUE_CML_sMatrix_t* mtrxX,
                                    TUE_CML_sMatrix_t* vecX);
static void createEquationMatrix2nd(TUE_CML_sMatrix_t* mtrxX,
                                    TUE_CML_sMatrix_t* vecX,
                                    REAL32_T pCrvDecay_nu);
static void createEquationMatrix3rd(TUE_CML_sMatrix_t* mtrxX,
                                    TUE_CML_sMatrix_t* vecX,
                                    REAL32_T pCrvDecay_nu,
                                    REAL32_T pCrvChngDecay_nu);
static REAL32_T getDevToTraj(TUE_CML_sMatrix_t* mtrxX,
                             TUE_CML_sMatrix_t* vecY,
                             TUE_CML_sMatrix_t* vecPolyCoeff,
                             TUE_CML_sMatrix_t* Weight);
static REAL32_T getArcTanSmallAng(REAL32_T x);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/* ****************************************************************************
Functionname:    calculatePolyfit1st

@brief           calculates a polynomial fit 1st order using least square method

@description     the measured x-/y-object positions are stored and ego motion
compensated using the driven delta ego distance and rotated delta ego yaw angle
                                 y = X * a --> a = (X^T * (W*X))^-1 * (X^T *
(W*y))

@param[in]       Weight : weighting factor vector (diagonal matrix elements)

@param[in]       mtrxX : model equation matrix X

@param[in]       vecY : y sample points

@return          bMtrxInvFailed : FALSE if matrix inversion was successful
**************************************************************************** */
static UINT8_T calculatePolyfit1st(TUE_CML_sMatrix_t* Weight,
                                   TUE_CML_sMatrix_t* mtrxX,
                                   TUE_CML_sMatrix_t* vecY,
                                   TUE_CML_sMatrix_t* vecPolyCoeff1st) {
    TUE_CML_CreateMatrix_M(mtrxX_T, POLYFIT_ORDER_1ST,
                           POLYFIT_SAMPLE_POINTS) /*transposed X^T matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(mtrxX_I, POLYFIT_ORDER_1ST,
                               POLYFIT_ORDER_1ST) /*inverted X^-1 matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(tmpMtrxWX, POLYFIT_SAMPLE_POINTS,
                               POLYFIT_ORDER_1ST) /*temp (W*X) matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxXTWX, POLYFIT_ORDER_1ST,
            POLYFIT_ORDER_1ST) /*temp (X^T)*(W*X) matrix using local memory
                                  allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxWy, POLYFIT_SAMPLE_POINTS,
            1) /*temp (W*y) vector using local memory allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxXTWy, POLYFIT_ORDER_1ST,
            1) /*temp (X^T)*(W*y) vector using local memory allocation*/
        TUE_CML_CreateMatrix_M(
            vecLastPolyCoeff, POLYFIT_ORDER_1ST,
            1) /*temp vector used for weighting using local memory allocation*/

        UINT8_T row,
        col;
    UINT8_T bMtrxInvFailed = 0U;

    TUE_CML_TransposeMatrix_M(mtrxX_T, mtrxX);

    for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
        for (col = 0; col < POLYFIT_ORDER_1ST; col++) {
            TUE_CML_GetMatrixElement_M(tmpMtrxWX, row, col) =
                TUE_CML_GetMatrixElement_M(Weight, row, 0) *
                TUE_CML_GetMatrixElement_M(mtrxX, row, col);
        }
    }
    TUE_CML_MutiplyMatrices_M(tmpMtrxXTWX, mtrxX_T, tmpMtrxWX);
    TUE_CML_InvertMatrix_M(
        mtrxX_I, tmpMtrxXTWX); /* from cml_adapted.c --> tmpMtrxXTWX gets
                                  overwritten with identity matrix */
    if ((mtrxX_I->Desc.row == 0u) || (mtrxX_I->Desc.col == 0u)) {
        bMtrxInvFailed = 1U;
    } else {
        for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
            TUE_CML_GetMatrixElement_M(tmpMtrxWy, row, 0) =
                TUE_CML_GetMatrixElement_M(Weight, row, 0) *
                TUE_CML_GetMatrixElement_M(vecY, row, 0);
        }
        TUE_CML_MutiplyMatrices_M(tmpMtrxXTWy, mtrxX_T, tmpMtrxWy);

        TUE_CML_MutiplyMatrices_M(vecPolyCoeff1st, mtrxX_I, tmpMtrxXTWy);

        bMtrxInvFailed = 0U;
    }
    return bMtrxInvFailed;
}
/* ****************************************************************************
Functionname:    calculatePolyfit2nd

@brief           calculates a polynomial fit using least square method

@description     the measured x-/y-object positions are stored and ego motion
compensated using the driven delta ego distance and rotated delta ego yaw angle
                                 y = X * a --> a = (X^T * (W*X))^-1 * (X^T *
(W*y))

@param[in]       Weight : weighting factor vector (diagonal matrix elements)

@param[in]       mtrxX : model equation matrix X

@param[in]       vecY : y sample points

@return          bMtrxInvFailed : FALSE if matrix inversion was successful
**************************************************************************** */
static UINT8_T calculatePolyfit2nd(TUE_CML_sMatrix_t* Weight,
                                   TUE_CML_sMatrix_t* mtrxX,
                                   TUE_CML_sMatrix_t* vecY,
                                   TUE_CML_sMatrix_t* vecPolyCoeff2nd) {
    TUE_CML_CreateMatrix_M(mtrxX_T, POLYFIT_ORDER_2ND,
                           POLYFIT_SAMPLE_POINTS) /*transposed X^T matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(mtrxX_I, POLYFIT_ORDER_2ND,
                               POLYFIT_ORDER_2ND) /*inverted X^-1 matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(tmpMtrxWX, POLYFIT_SAMPLE_POINTS,
                               POLYFIT_ORDER_2ND) /*temp (W*X) matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxXTWX, POLYFIT_ORDER_2ND,
            POLYFIT_ORDER_2ND) /*temp (X^T)*(W*X) matrix using local memory
                                  allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxWy, POLYFIT_SAMPLE_POINTS,
            1) /*temp (W*y) vector using local memory allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxXTWy, POLYFIT_ORDER_2ND,
            1) /*temp (X^T)*(W*y) vector using local memory allocation*/
        TUE_CML_CreateMatrix_M(
            vecLastPolyCoeff, POLYFIT_ORDER_2ND,
            1) /*temp vector used for weighting using local memory allocation*/

        UINT8_T row,
        col;
    UINT8_T bMtrxInvFailed = 0U;

    TUE_CML_TransposeMatrix_M(mtrxX_T, mtrxX);

    for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
        for (col = 0; col < POLYFIT_ORDER_2ND; col++) {
            TUE_CML_GetMatrixElement_M(tmpMtrxWX, row, col) =
                TUE_CML_GetMatrixElement_M(Weight, row, 0) *
                TUE_CML_GetMatrixElement_M(mtrxX, row, col);
        }
    }
    TUE_CML_MutiplyMatrices_M(tmpMtrxXTWX, mtrxX_T, tmpMtrxWX);
    TUE_CML_InvertMatrix_M(
        mtrxX_I, tmpMtrxXTWX); /* from cml_adapted.c --> tmpMtrxXTWX gets
                                  overwritten with identity matrix */
    if ((mtrxX_I->Desc.row == 0u) || (mtrxX_I->Desc.col == 0u)) {
        bMtrxInvFailed = 1U;
    } else {
        for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
            TUE_CML_GetMatrixElement_M(tmpMtrxWy, row, 0) =
                TUE_CML_GetMatrixElement_M(Weight, row, 0) *
                TUE_CML_GetMatrixElement_M(vecY, row, 0);
        }
        TUE_CML_MutiplyMatrices_M(tmpMtrxXTWy, mtrxX_T, tmpMtrxWy);

        TUE_CML_MutiplyMatrices_M(vecPolyCoeff2nd, mtrxX_I, tmpMtrxXTWy);

        bMtrxInvFailed = 0U;
    }
    return bMtrxInvFailed;
}
/* ****************************************************************************
Functionname:    calculatePolyfit3rd

@brief           calculates a polynomial fit using least square method

@description     the measured x-/y-object positions are stored and ego motion
compensated using the driven delta ego distance and rotated delta ego yaw angle
                                 y = X * a --> a = (X^T * (W*X))^-1 * (X^T *
(W*y))

@param[in]       Weight : weighting factor vector (diagonal matrix elements)

@param[in]       mtrxX : model equation matrix X

@param[in]       vecY : y sample points

@return          bMtrxInvFailed : FALSE if matrix inversion was successful
**************************************************************************** */
static UINT8_T calculatePolyfit3rd(TUE_CML_sMatrix_t* Weight,
                                   TUE_CML_sMatrix_t* mtrxX,
                                   TUE_CML_sMatrix_t* vecY,
                                   TUE_CML_sMatrix_t* vecPolyCoeff3rd) {
    TUE_CML_CreateMatrix_M(mtrxX_T, POLYFIT_ORDER_3RD,
                           POLYFIT_SAMPLE_POINTS) /*transposed X^T matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(mtrxX_I, POLYFIT_ORDER_3RD,
                               POLYFIT_ORDER_3RD) /*inverted X^-1 matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(tmpMtrxWX, POLYFIT_SAMPLE_POINTS,
                               POLYFIT_ORDER_3RD) /*temp (W*X) matrix using
                                                     local memory allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxXTWX, POLYFIT_ORDER_3RD,
            POLYFIT_ORDER_3RD) /*temp (X^T)*(W*X) matrix using local memory
                                  allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxWy, POLYFIT_SAMPLE_POINTS,
            1) /*temp (W*y) vector using local memory allocation*/
        TUE_CML_CreateMatrix_M(
            tmpMtrxXTWy, POLYFIT_ORDER_3RD,
            1) /*temp (X^T)*(W*y) vector using local memory allocation*/
        TUE_CML_CreateMatrix_M(
            vecLastPolyCoeff, POLYFIT_ORDER_3RD,
            1) /*temp vector used for weighting using local memory allocation*/

        UINT8_T row,
        col;
    UINT8_T bMtrxInvFailed = 0U;

    TUE_CML_TransposeMatrix_M(mtrxX_T, mtrxX);

    for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
        for (col = 0; col < POLYFIT_ORDER_3RD; col++) {
            TUE_CML_GetMatrixElement_M(tmpMtrxWX, row, col) =
                TUE_CML_GetMatrixElement_M(Weight, row, 0) *
                TUE_CML_GetMatrixElement_M(mtrxX, row, col);
        }
    }
    TUE_CML_MutiplyMatrices_M(tmpMtrxXTWX, mtrxX_T, tmpMtrxWX);
    TUE_CML_InvertMatrix_M(
        mtrxX_I, tmpMtrxXTWX); /* from cml_adapted.c --> tmpMtrxXTWX gets
                                  overwritten with identity matrix */
    if ((mtrxX_I->Desc.row == 0u) || (mtrxX_I->Desc.col == 0u)) {
        bMtrxInvFailed = 1U;
    } else {
        for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
            TUE_CML_GetMatrixElement_M(tmpMtrxWy, row, 0) =
                TUE_CML_GetMatrixElement_M(Weight, row, 0) *
                TUE_CML_GetMatrixElement_M(vecY, row, 0);
        }
        TUE_CML_MutiplyMatrices_M(tmpMtrxXTWy, mtrxX_T, tmpMtrxWy);

        TUE_CML_MutiplyMatrices_M(vecPolyCoeff3rd, mtrxX_I, tmpMtrxXTWy);

        bMtrxInvFailed = 0U;
    }
    return bMtrxInvFailed;
}

/* ****************************************************************************
Functionname:    createEquationMatrix1st

@brief           create equation matrix X with model equations

@description     create equation matrix X with model equations for 1st order

@param[in/out]   mtrxX : model equation matrix X (out --> updated)

@param[in]       vecX : x sample points

**************************************************************************** */
static void createEquationMatrix1st(TUE_CML_sMatrix_t* mtrxX,
                                    TUE_CML_sMatrix_t* vecX) {
    // UINT8_T row, col;
    UINT8_T row;
    /*fill equation matrix X */
    for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
        /*for(col = 0; col < POLY_ORDER; col++){
        }*/
        /* alternatively without second for-loop for columns to avoid CML
        dependency (CML_f_expPower and factorial) */
        TUE_CML_GetMatrixElement_M(mtrxX, row, 0) = 1.0f;
        TUE_CML_GetMatrixElement_M(mtrxX, row, 1) =
            TUE_CML_GetMatrixElement_M(vecX, row, 0);
    }
}

/* ****************************************************************************
Functionname:    createEquationMatrix2nd

@brief           create equation matrix X with model equations

@description     create equation matrix X with model equations for 2nd order

@param[in/out]   mtrxX : model equation matrix X (out --> updated)

@param[in]       vecX : x sample points

**************************************************************************** */
static void createEquationMatrix2nd(TUE_CML_sMatrix_t* mtrxX,
                                    TUE_CML_sMatrix_t* vecX,
                                    REAL32_T pCrvDecay_nu) {
    // UINT8_T row, col;
    UINT8_T row;
    /*fill equation matrix X */
    for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
        /*for(col = 0; col < POLY_ORDER; col++){
        }*/
        /* alternatively without second for-loop for columns to avoid CML
        dependency (CML_f_expPower and factorial) */
        TUE_CML_GetMatrixElement_M(mtrxX, row, 0) = 1.0f;
        TUE_CML_GetMatrixElement_M(mtrxX, row, 1) =
            TUE_CML_GetMatrixElement_M(vecX, row, 0);
        /* use pCrvDecay_nu for damping of curvature */
        if (pCrvDecay_nu > 0.0001f) {
            TUE_CML_GetMatrixElement_M(mtrxX, row, 2) =
                TUE_CML_GetMatrixElement_M(vecX, row, 0) *
                TUE_CML_GetMatrixElement_M(vecX, row, 0) * 0.5f / pCrvDecay_nu;
        } else {
            TUE_CML_GetMatrixElement_M(mtrxX, row, 2) =
                TUE_CML_GetMatrixElement_M(vecX, row, 0) *
                TUE_CML_GetMatrixElement_M(vecX, row, 0) * 0.5f;
        }
    }
}

/* ****************************************************************************
Functionname:    createEquationMatrix3rd

@brief           create equation matrix X with model equations for 3rd order

@description     create equation matrix X with model equations for 3rd order

@param[in/out]   mtrxX : model equation matrix X (out --> updated)

@param[in]       vecX : x sample points

**************************************************************************** */
static void createEquationMatrix3rd(TUE_CML_sMatrix_t* mtrxX,
                                    TUE_CML_sMatrix_t* vecX,
                                    REAL32_T pCrvDecay_nu,
                                    REAL32_T pCrvChngDecay_nu) {
    // UINT8_T row, col;
    UINT8_T row;
    /*fill equation matrix X */
    for (row = 0; row < POLYFIT_SAMPLE_POINTS; row++) {
        /*for(col = 0; col < POLY_ORDER; col++){
        }*/
        /* alternatively without second for-loop for columns to avoid CML
        dependency (CML_f_expPower and factorial) */
        TUE_CML_GetMatrixElement_M(mtrxX, row, 0) = 1.0f;
        TUE_CML_GetMatrixElement_M(mtrxX, row, 1) =
            TUE_CML_GetMatrixElement_M(vecX, row, 0);
        /* use pCrvDecay_nu for damping of curvature */
        if (pCrvDecay_nu > 0.0001f) {
            TUE_CML_GetMatrixElement_M(mtrxX, row, 2) =
                TUE_CML_GetMatrixElement_M(vecX, row, 0) *
                TUE_CML_GetMatrixElement_M(vecX, row, 0) * 0.5f / pCrvDecay_nu;
        } else {
            TUE_CML_GetMatrixElement_M(mtrxX, row, 2) =
                TUE_CML_GetMatrixElement_M(vecX, row, 0) *
                TUE_CML_GetMatrixElement_M(vecX, row, 0) * 0.5f;
        }
        /* use pCrvChngDecay_nu for damping of curvature change*/
        if (pCrvChngDecay_nu > 0.0001f) {
            TUE_CML_GetMatrixElement_M(mtrxX, row, 3) =
                TUE_CML_GetMatrixElement_M(vecX, row, 0) *
                TUE_CML_GetMatrixElement_M(vecX, row, 0) *
                TUE_CML_GetMatrixElement_M(vecX, row, 0) / 6.0f /
                pCrvChngDecay_nu;
        } else {
            TUE_CML_GetMatrixElement_M(mtrxX, row, 3) =
                TUE_CML_GetMatrixElement_M(vecX, row, 0) *
                TUE_CML_GetMatrixElement_M(vecX, row, 0) *
                TUE_CML_GetMatrixElement_M(vecX, row, 0) / 6.0f;
        }
    }
}

/* ****************************************************************************
Functionname:    getMeanDevToTraj

@brief           compensate ego motion between 2 cycles

@description     first fDeltaX_met and fDeltaY_met are subtracted,
                                 then both points are rotated with
fDeltaYawAng_rad

@param[in]       fEgoYawRate_rps : ego yaw rate

@param[in]       fEgoVelX_mps : ego velocity

@param[in]       fCycleTime_sec : cycle time

@param[in/out]   vecX : global x sample points (out --> updated)

@param[in/out]   vecY : global y sample points (out --> updated)

**************************************************************************** */
static REAL32_T getDevToTraj(TUE_CML_sMatrix_t* mtrxX,
                             TUE_CML_sMatrix_t* vecY,
                             TUE_CML_sMatrix_t* vecPolyCoeff,
                             TUE_CML_sMatrix_t* Weight) {
    TUE_CML_CreateMatrix_M(tmpVecXa, POLYFIT_SAMPLE_POINTS,
                           1) /*temp (X*a) Vec using local memory allocation*/
        REAL32_T fDevTraj_met = 0.0;
    UINT8_T i;

    TUE_CML_MutiplyMatrices_M(tmpVecXa, mtrxX, vecPolyCoeff);

    for (i = 0; i < POLYFIT_SAMPLE_POINTS; i++) {
        if (TUE_CML_GetMatrixElement_M(Weight, i, 0) > 0.0001f) {
            fDevTraj_met +=
                TUE_CML_Abs_M(TUE_CML_GetMatrixElement_M(tmpVecXa, i, 0) -
                              TUE_CML_GetMatrixElement_M(vecY, i, 0));
        }
    }

    return fDevTraj_met;
}

/* ****************************************************************************
Functionname:    getArcTanSmallAng
                                 arctan approximation using equation 10 of
                                 "Efficient Approximations for the Arctangent
Function" with smaller errors around 0 degree

**************************************************************************** */
static REAL32_T getArcTanSmallAng(REAL32_T x) {
    REAL32_T fArcTan = 0.0f;
    /* − 1 ≤ x ≤ 1 */
    if (x > 1.0f) {
        x = 1.0f;
    } else if (x < -1.0f) {
        x = -1.0f;
    }
    fArcTan = x / (1.0F + (0.28086F * x * x));
    return fArcTan;
}

/* ****************************************************************************
Function name:    polyfitLCF

@brief           calculates a 1st, 2nd and 3rd order polynomial fit using least
square method

@description     the measured x-/y-object positions are stored and ego motion
compensated using the driven delta ego distance and rotated delta ego yaw angle.
                                 The equation matrix X is filled with the x/y
sample points and the polynomial coefficient vector is calculated

@param[in]       fObjXPos_met : target object x position

@param[in]       fObjYPos_met : target object y position

@param[out]      vecPolyCoeff : calculated polynomial coefficients

@author          lfj
**************************************************************************** */
void TUE_CML_PolyFit_M(const sPFTInput_t* pPFTInput, sPFTOutput_t* pPFTOutput) {
    TUE_CML_CreateMatrix_M(
        mtrxX1st, POLYFIT_SAMPLE_POINTS,
        POLYFIT_ORDER_1ST) /*create X matrix with sample point equations using
                              static memory allocation*/
        TUE_CML_CreateMatrix_M(
            mtrxX2nd, POLYFIT_SAMPLE_POINTS,
            POLYFIT_ORDER_2ND) /*create X matrix with sample point equations
                                  using static memory allocation*/
        TUE_CML_CreateMatrix_M(
            mtrxX3rd, POLYFIT_SAMPLE_POINTS,
            POLYFIT_ORDER_3RD) /*create X matrix with sample point equations
                                  using static memory allocation*/
        TUE_CML_CreateMatrix_M(
            vecY, POLYFIT_SAMPLE_POINTS,
            1) /* vector for object y-position storage which gets filled from
                  input struct and transformed each cycle */
        TUE_CML_CreateMatrix_M(
            vecX, POLYFIT_SAMPLE_POINTS,
            1) /* vector for object x-position storage which gets filled from
                  input struct and transformed each cycle */
        TUE_CML_CreateMatrix_M(vecPolyCoeff1st, POLYFIT_ORDER_1ST,
                               1) /* calculated polynomial coefficients which
                                     are written to the output struct */
        TUE_CML_CreateMatrix_M(vecPolyCoeff2nd, POLYFIT_ORDER_2ND,
                               1) /* calculated polynomial coefficients which
                                     are written to the output struct */
        TUE_CML_CreateMatrix_M(vecPolyCoeff3rd, POLYFIT_ORDER_3RD,
                               1) /* calculated polynomial coefficients which
                                     are written to the output struct */
        TUE_CML_CreateMatrix_M(
            Weight, POLYFIT_SAMPLE_POINTS,
            1) /* weighting factors --> diagonal elements of matrix stored as
                  vector for element-wise calculations */

        /*initialize local variables*/
        UINT8_T bTrajInvalid1st_bool = 0U;
    UINT8_T bTrajInvalid2nd_bool = 0U;
    UINT8_T bTrajInvalid3rd_bool = 0U;

    int i;
    /*Initialize matrices*/
    for (i = 0; i < POLYFIT_SAMPLE_POINTS; i++) {
        TUE_CML_GetMatrixElement_M(vecX, i, 0) = pPFTInput->fPosXArray[i];
        TUE_CML_GetMatrixElement_M(vecY, i, 0) = pPFTInput->fPosYArray[i];
        TUE_CML_GetMatrixElement_M(Weight, i, 0) = pPFTInput->fFctWeight[i];
    }

    /* calculate 1st order fit */
    if (pPFTInput->bEnable1st) {
        createEquationMatrix1st(mtrxX1st, vecX);
        bTrajInvalid1st_bool =
            calculatePolyfit1st(Weight, mtrxX1st, vecY, vecPolyCoeff1st);

        if (!bTrajInvalid1st_bool) {
            pPFTOutput->fPosY01st =
                TUE_CML_GetMatrixElement_M(vecPolyCoeff1st, 0, 0);
            pPFTOutput->fHeading1st = getArcTanSmallAng(
                TUE_CML_GetMatrixElement_M(vecPolyCoeff1st, 1, 0));
            pPFTOutput->fDevToTraj1st =
                getDevToTraj(mtrxX1st, vecY, vecPolyCoeff1st, Weight);

        } else {
            pPFTOutput->fPosY01st = 0.0f;
            pPFTOutput->fHeading1st = 0.0f;
            pPFTOutput->fDevToTraj1st = 0.0f;
        }
        pPFTOutput->bTrajInvalid1st = bTrajInvalid1st_bool;
    } else {
        pPFTOutput->bTrajInvalid1st = 1U;
        pPFTOutput->fPosY01st = 0.0f;
        pPFTOutput->fHeading1st = 0.0f;
        pPFTOutput->fDevToTraj1st = 0.0f;
    }

    /* calculate 2nd order fit */
    if (pPFTInput->bEnable2nd) {
        createEquationMatrix2nd(mtrxX2nd, vecX, pPFTInput->fFctCrvDecay);
        bTrajInvalid2nd_bool =
            calculatePolyfit2nd(Weight, mtrxX2nd, vecY, vecPolyCoeff2nd);

        if (!bTrajInvalid2nd_bool) {
            pPFTOutput->fPosY02nd =
                TUE_CML_GetMatrixElement_M(vecPolyCoeff2nd, 0, 0);
            pPFTOutput->fHeading2nd = getArcTanSmallAng(
                TUE_CML_GetMatrixElement_M(vecPolyCoeff2nd, 1, 0));
            pPFTOutput->fCrv2nd =
                TUE_CML_GetMatrixElement_M(vecPolyCoeff2nd, 2, 0);

            pPFTOutput->fDevToTraj2nd =
                getDevToTraj(mtrxX2nd, vecY, vecPolyCoeff2nd, Weight);

        } else {
            pPFTOutput->fPosY02nd = 0.0f;
            pPFTOutput->fHeading2nd = 0.0f;
            pPFTOutput->fCrv2nd = 0.0f;
            pPFTOutput->fDevToTraj2nd = 0.0f;
        }
        pPFTOutput->bTrajInvalid2nd = bTrajInvalid2nd_bool;
    } else {
        pPFTOutput->bTrajInvalid2nd = 1U;
        pPFTOutput->fPosY02nd = 0.0f;
        pPFTOutput->fHeading2nd = 0.0f;
        pPFTOutput->fCrv2nd = 0.0f;
        pPFTOutput->fDevToTraj2nd = 0.0f;
    }

    /*	calculate 3rd order fit*/
    if (pPFTInput->bEnable3rd) {
        createEquationMatrix3rd(mtrxX3rd, vecX, pPFTInput->fFctCrvDecay,
                                pPFTInput->fFctCrvChngDecay);
        bTrajInvalid3rd_bool =
            calculatePolyfit3rd(Weight, mtrxX3rd, vecY, vecPolyCoeff3rd);

        if (!bTrajInvalid3rd_bool) {
            pPFTOutput->fPosY03rd =
                TUE_CML_GetMatrixElement_M(vecPolyCoeff3rd, 0, 0);
            pPFTOutput->fHeading3rd = getArcTanSmallAng(
                TUE_CML_GetMatrixElement_M(vecPolyCoeff3rd, 1, 0));
            pPFTOutput->fCrv3rd =
                TUE_CML_GetMatrixElement_M(vecPolyCoeff3rd, 2, 0);
            pPFTOutput->fChngOfCrv3rd =
                TUE_CML_GetMatrixElement_M(vecPolyCoeff3rd, 3, 0);

            pPFTOutput->fDevToTraj3rd =
                getDevToTraj(mtrxX3rd, vecY, vecPolyCoeff3rd, Weight);

        } else {
            pPFTOutput->fPosY03rd = 0.0f;
            pPFTOutput->fHeading3rd = 0.0f;
            pPFTOutput->fCrv3rd = 0.0f;
            pPFTOutput->fChngOfCrv3rd = 0.0f;
            pPFTOutput->fDevToTraj3rd = 0.0f;
        }
        pPFTOutput->bTrajInvalid3rd = bTrajInvalid3rd_bool;
    } else {
        pPFTOutput->bTrajInvalid3rd = 1U;
        pPFTOutput->fPosY03rd = 0.0f;
        pPFTOutput->fHeading3rd = 0.0f;
        pPFTOutput->fCrv3rd = 0.0f;
        pPFTOutput->fChngOfCrv3rd = 0.0f;
        pPFTOutput->fDevToTraj3rd = 0.0f;
    }
}

/*****************************************************************************
  Functionname:    NULL_FUNC                                            */ /*!

  @brief          null function for data freeze complier pass
  @description    null function for data freeze complier pass
  @param[in]      void
  @param[out]     void
  @return         void

*****************************************************************************/
void NULL_FUNC(void) {}

// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"