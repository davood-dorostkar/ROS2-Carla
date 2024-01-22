/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

/*! @todo: later clean-up include structure to only include "cp.h" and let it
include the files necessary for our run-time environment */
#include "TM_Global_Types.h"
#include "TM_Global_Types.h"
#include "stddef.h"
#include "cp.h"

/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*! Static switch to enable extending point lists at both ends to infinite
distance. Note: this switch has to be on, so functions
CPCalculatePoint2SamplesDist,
CPCalculatePoint2TraceDist etc. boolean parameter for extending end segments
has any effect! */
#define CP_CFG_EXTEND_SAMPLE_END_LINE_SEGS 1

/*! Pre-processor switch to select extending traces using circle equation
if possible. If disabled, traces are line-extended. */
#define CP_CFG_EXTEND_TRACES_USING_CIRCLE_APPROX 0

/*! Extended F32 delta testing value, needed for the Gaussian elimination
calculation for zero division testing. Due to high range dynamics, this needs
to be significantly smaller then the default 1e-4f */
#define C_F32_EXT_DELTA 1e-8f

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*! Infinite radius returned when no real circle radius can be detected */
#define CP_INFINITE_RADIUS 1e6f

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/

/* The following static functions are only used for testing - but may be
activated later for other use */

/* from here on functions are not needed in SRR */

/*************************************************************************************************************************
  Functionname:    CPCalculatePoint2SamplesDist */
fDistance_t CPCalculatePoint2SamplesDist(const CPPosSamples_t *pSamples,
                                         fDistance_t fPointX,
                                         fDistance_t fPointY,
                                         boolean bExtendSampleEnds) {
    return CPCalculatePoint2LineSegCoordArr(pSamples->fx, pSamples->fy,
                                            pSamples->nb_samples, fPointX,
                                            fPointY, bExtendSampleEnds);
}

/*************************************************************************************************************************
  Functionname:    CPCalculateRadii */
void CPCalculateRadii(const float32 pSamplesX[],
                      const float32 pSamplesY[],
                      uint8 NumSamples,
                      fDistance_t *pfRadius) {
    uint8 i;
    /* For every three points (which form a valid triangle) we can determine the
    radius of the circumscribed circle. This of course requires that we have
    more
    than three points to begin with */
    if (NumSamples >= 3u) {
        /* Now taking three points and marking their coordinates (X1,Y1) (X2,Y2)
        and (X3,Y3),
        then the coordinates of the circle's center are
        XC = X1 + 0.5*(X2-X1) - 0.5*(Y2-Y1)*fSubExp
        YC = Y1 + 0.5*(Y2-Y1) + 0.5*(X2-X1)*fSubExp,
        where fSubExp = 0.5f * ((X3-X1)(X3-X2) + (Y3-Y1)(Y3-Y2)) /
        (((X3-X2)*(Y2-Y1)) - ((X2-X1)*(Y3-Y2)))
        (The formula comes from calculation of the common point of the
        perpendicular halving lines
        of the sides 1-2 and 2-3, and is valid as long as Y3 != Y2)
        As can be seen, the later part of the equations is the same, so it can
        be reused.
        If only the unsigned radius is the question, then the formula
        R = 0.5 * (a*b*c)/sqrt(s(s-a)(s-b)(s-c)),
        where a,b,c are the sides of the triangle and s=(a+b+c)/2
        can also be used. Note: sign detection could then be done over
        calculation
        of crossproduct or similar...
        Now: outside of processing loop allocate delta X2-X3 and delta Y2-Y3,
        since these values can be re-used among iterations (shifting them)
        */
        float32 fX3X2, fY3Y2;

        fX3X2 = pSamplesX[1] - pSamplesX[0];
        fY3Y2 = pSamplesY[1] - pSamplesY[0];

        for (i = 0u; i < (uint8)(NumSamples - 2u); i++) {
            float32 fX2X1, fY2Y1, fX3X1, fY3Y1, fDivisor, fSubExp, fRadius;
            /* Reuse delta coordinates (side length) from last iteration */
            fX2X1 = fX3X2;
            fY2Y1 = fY3Y2;

            /* Calculate delta coordinates */
            fX3X2 = pSamplesX[2] - pSamplesX[1];
            fY3Y2 = pSamplesY[2] - pSamplesY[1];

            fX3X1 = pSamplesX[2] - pSamplesX[0];
            fY3Y1 = pSamplesY[2] - pSamplesY[0];

            /* The equation described above is only valid, when fY3Y2 is
            non-zero.
            If it is zero, we need to modify the calculation, but in that case
            we
            need an extra check to verify that points 2 and 3 are not the same
            */
            if ((fY3Y2 >= C_F32_DELTA) || (fY3Y2 <= -C_F32_DELTA)) {
                /* Formula valid, calculate sub-expression divisor first */
                fDivisor = ((fX3X2 * fY2Y1) - (fX2X1 * fY3Y2));
                fSubExp = 0.5f * ((fX3X2 * fX3X1) + (fY3Y2 * fY3Y1));
                /* Check if divisor valid */
                if ((fDivisor >= C_F32_DELTA) || (fDivisor <= -C_F32_DELTA)) {
                    float32 fXC, fYC;
                    /* We can now perform the division (since sign safe) */
                    fSubExp /= fDivisor;
                    /* And using the value of fSubExp, we can easily find
                     * circle's center */
                    fXC = (pSamplesX[0] + (0.5f * fX2X1)) + (fY2Y1 * fSubExp);
                    fYC = (pSamplesY[0] + (0.5f * fY2Y1)) - (fX2X1 * fSubExp);
                    /* Calculate unsigned radius based on coordinates */
                    fRadius = SQRT_(SQR(fXC - pSamplesX[0]) +
                                    SQR(fYC - pSamplesY[0]));
                    /* Depending on the sign of the cross product of the vectors
                    from the sides 1-2 and 2-3 change the sign of radius to
                    negative */
                    if (((fX2X1 * fY3Y2) - (fX3X2 * fY2Y1)) < 0.f) {
                        fRadius = -fRadius;
                    }
                } else {
                    fRadius = CP_INFINITE_RADIUS;
                }
            } else if ((fX3X2 >= C_F32_DELTA) || (fX3X2 <= -C_F32_DELTA)) {
                /* Formula plagued by division by zero issue, but using this
                we can simplify the sub-formula: fSubExp = 0.5f*fX3X1/fY2Y1
                (if other side also zero, we don't have a triangle) */
                fDivisor = fY2Y1;
                fSubExp = 0.5f * fX3X1;
                /* Check if divisor valid */
                if ((fDivisor >= C_F32_DELTA) || (fDivisor <= -C_F32_DELTA)) {
                    float32 fXC, fYC;
                    /* We can now perform the division (since sign safe) */
                    fSubExp /= fDivisor;
                    /* And using the value of fSubExp, we can easily find
                     * circle's center */
                    fXC = (pSamplesX[0] + (0.5f * fX2X1)) + (fY2Y1 * fSubExp);
                    fYC = (pSamplesY[0] + (0.5f * fY2Y1)) - (fX2X1 * fSubExp);
                    /* Calculate unsigned radius based on coordinates */
                    fRadius = SQRT_(SQR(fXC - pSamplesX[0]) +
                                    SQR(fYC - pSamplesY[0]));
                    /* Depending on the sign of the cross product of the vectors
                    from the sides 1-2 and 2-3 change the sign of radius to
                    negative */
                    if (((fX2X1 * fY3Y2) - (fX3X2 * fY2Y1)) < 0.f) {
                        fRadius = -fRadius;
                    }
                } else {
                    fRadius = CP_INFINITE_RADIUS;
                }
            } else {
                /* Special case: points 2 and 3 are the same */
                fRadius = CP_INFINITE_RADIUS;
            }

            /* Advance source point pointers all by one */
            pSamplesX++;

            pSamplesY++;

            /* Store result and advance pointer by one (to fill next) */
            *pfRadius = fRadius;
            pfRadius++;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    CPCalcPointApproxPolyL2 */
void CPCalcPointApproxPolyL2(CPTracePolyL2_t *pPoly,
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
            const float32 fCurX2 = SQR(fCurX);
            pafX++;

            pafY++;

            /* Update sums */
            fXSum += fCurX;
            fXPow2Sum += fCurX2;
            fXPow3Sum += fCurX2 * fCurX;
            fXPow4Sum += SQR(fCurX2);
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
        if (fABS((fXPow2Sum * fXPow4Sum) - (fXPow3Sum * fXPow3Sum)) >
            C_F32_DELTA) {
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
            form
            and then using elimination to solve it (Gauss elimination) */
            for (i = 0; i < 2; i++) {
                float32 fDivisor = fLinEqMatrix[i][i];
                float32 fInvColumnMax;
                /* Calculate inverse of column max once here, prevent devision
                 * by zero */
                if (fABS(fDivisor) < C_F32_DELTA) {
                    if (fDivisor < 0.0F) {
                        fDivisor = -C_F32_DELTA;
                    } else {
                        fDivisor = C_F32_DELTA;
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
                if ((fLinEqMatrix[i][i] > C_F32_EXT_DELTA) ||
                    (fLinEqMatrix[i][i] < -C_F32_EXT_DELTA)) {
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
            if ((fDeltaX > C_F32_DELTA) || (fDeltaX < -C_F32_DELTA)) {
                pPoly->fC2 = 0.f;
                pPoly->fC1 = (fDeltaY / fDeltaX);
                pPoly->fC0 = pafY[0] - (pafX[0] * pPoly->fC1);
                pPoly->isValid = TRUE;
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    CPCalcPointApproxPolyL2Ext */
void CPCalcPointApproxPolyL2Ext(CPTracePolyL2_t *pPoly,
                                const float32 pafX[],
                                const float32 pafY[],
                                const float32 *pafYStdDev,
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
        /* Initilaize return poly C0 to Y0 (plays a role when X coordinates all
         * zero) */
        pPoly->fC0 = *pafY;
        /* Go through all points and calculate the sums */
        while (uNumPts > 0u) {
            const float32 fCurX = *pafX;
            const float32 fCurY = *pafY;
            const float32 fCurStdDev = *pafYStdDev;
            const float32 fCurX2 = SQR(fCurX);
            /* Use 1/stddev to weight the current sample */
            const float32 fInvCurStdDev =
                (1.f / (MAX_FLOAT(C_F32_DELTA, fCurStdDev)));
            pafX++;

            pafY++;

            pafYStdDev++;

            /* Update sums */
            fXSum += fCurX * fInvCurStdDev;
            fXPow2Sum += fCurX2 * fInvCurStdDev;
            fXPow3Sum += fCurX2 * fCurX * fInvCurStdDev;
            fXPow4Sum += SQR(fCurX2) * fInvCurStdDev;
            fYSum += fCurY * fInvCurStdDev;
            fXYSum += fCurX * fCurY * fInvCurStdDev;
            fX2YSum += fCurX2 * fCurY * fInvCurStdDev;
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
        if (fABS((fXPow2Sum * fXPow4Sum) - (fXPow3Sum * fXPow3Sum)) >
            C_F32_DELTA) {
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
            form
            and then using elimination to solve it (Gauss elimination) */
            for (i = 0; i < 2; i++) {
                /* Calculate inverse of column max once here */
                float32 fDivisor = fLinEqMatrix[i][i];
                float32 fInvColumnMax;
                /* Calculate inverse of column max once here, prevent devision
                 * by zero */
                if (fABS(fDivisor) < C_F32_DELTA) {
                    if (fDivisor < 0.0F) {
                        fDivisor = -C_F32_DELTA;
                    } else {
                        fDivisor = C_F32_DELTA;
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
                if ((fLinEqMatrix[i][i] > C_F32_EXT_DELTA) ||
                    (fLinEqMatrix[i][i] < -C_F32_EXT_DELTA)) {
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
        } else {
            /* fXPow2Sum is zero -> all points have zero X coordinate */
            pPoly->fC2 = 0.f;
            pPoly->fC1 = 0.f;
            /* C0 initialization value to Y coordinate already OK */
        }
    } else {
        /* Initialize default return value */
        pPoly->fC2 = 0.f;
        pPoly->fC1 = 0.f;
        pPoly->fC0 = pafY[0];
        /* Approximate object movement by a simple line */
        if (uNumPts > 1u) {
            const float32 fDeltaX = pafX[1] - pafX[0];
            const float32 fDeltaY = pafY[1] - pafY[0];
            if ((fDeltaX > C_F32_DELTA) || (fDeltaX < -C_F32_DELTA)) {
                pPoly->fC2 = 0.f;
                pPoly->fC1 = (fDeltaY / fDeltaX);
                pPoly->fC0 = pafY[0] - (pafX[0] * pPoly->fC1);
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    CPCalcDistToPolyL2Y */
float32 CPCalcDistToPolyL2Y(const CPTracePolyL2_t *const pPoly,
                            const float32 fX,
                            const float32 fY) {
    return fY - ((((pPoly->fC2 * fX) + pPoly->fC1) * fX) + pPoly->fC0);
}

/*************************************************************************************************************************
  Functionname:    CPCalcSumSqDistance */
float32 CPCalcSumSqDistance(const CPTracePolyL2_t *const pPoly,
                            const float32 *pfPointX,
                            const float32 *pfPointY,
                            uint8 uNumPoints) {
    float32 fSumDistance = 0.f;

    /* Go through the points, and calculate the distances */
    while (uNumPoints > 0u) {
        /* Calculate signed distance of current point */
        const float32 fCurDist =
            CPCalcDistToPolyL2Y(pPoly, *pfPointX, *pfPointY);
        /* Add the current squared distance to the sum */
        fSumDistance += SQR(fCurDist);
        /* Advance point coordinate pointers and update remaining number of
         * points */
        pfPointX++;

        pfPointY++;

        uNumPoints--;
    }
    return fSumDistance;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */