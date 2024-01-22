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

#ifndef C_F32_EXT_DELTA
/*! Extended F32 delta testing value, needed for the gaussian elimination
calculation for zero division testing. Due to high range dynamics, this needs
to be significantly smaller then the default 1e-4f */
#define C_F32_EXT_DELTA 1e-8f
#endif

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
  Functionname:    CPCalculatePoint2LineSegListDist */
fDistance_t CPCalculatePoint2LineSegListDist(const GDBVector2_t pSamples[],
                                             uint8 NumSamples,
                                             fDistance_t fPointX,
                                             fDistance_t fPointY,
                                             boolean bExtendEndLineSegs) {
    fDistance_t fSamplesDist;

    /* Initialize return value to infinite distance */
    fSamplesDist = CP_INFINITE_DIST;
    /* If more then two samples are present, then these samples form a
     * line-segment(s) */
    if (NumSamples > (uint8)0) {
        fDistance_t fSmallestAbsSqDist;
        fDistance_t fSmallestDistSign;
        fDistance_t fLastSamplePtSqDist;
        const GDBVector2_t* pPrevSample = pSamples;
        uint8 uRemainSegs;

        /* Initialize last sample square distance to distance to first sample */
        fLastSamplePtSqDist =
            (SQR(fPointX - pSamples[0].f0) + SQR(fPointY - pSamples[0].f1));

        /* Also set this as the minimum (initialization) */
        fSmallestAbsSqDist = fLastSamplePtSqDist;

        /* Also set sign of distance to positive (with a single point) we
        can only have positive signs, as no directions yet known */
        fSmallestDistSign = 1.0f;

        /* Loop through line segments. Loop runs (NumSamples-1) times */
        for (uRemainSegs = (uint8)(NumSamples - 1u); uRemainSegs > (uint8)0;
             uRemainSegs--) {
            float32 fCurLineSegDistSq;

            /* Get current sample from previous one */
            const GDBVector2_t* const pCurSample = &pPrevSample[1];
            /* Calculate delta X and delta Y between the sample points */
            const fDistance_t fSampleDX = (pCurSample->f0 - pPrevSample->f0);
            const fDistance_t fSampleDY = (pCurSample->f1 - pPrevSample->f1);
            /* Based on delta X & Y calculate squared distance of sample points
             */
            const float32 fSamplesSqDist = SQR(fSampleDX) + SQR(fSampleDY);
            /* Calculate current sample point square distance */
            const float32 fCurSamplePtSqDist =
                (SQR(fPointX - pCurSample->f0) + SQR(fPointY - pCurSample->f1));
            /* Verify that the samples are not the same (if they are, we can
            skip
            processing, since results from last line segment still apply */
            if (fSamplesSqDist > C_F32_DELTA) {
                /* Calculate cross product of line vector and vector from line
                segment
                starting point (previous sample) to the point */
                const float32 fCrossProd =
                    ((fSampleDX * (pPrevSample->f1 - fPointY)) -
                     (fSampleDY * (pPrevSample->f0 - fPointX)));
                /* Using the pythagoras theorem verify that the triangle between
                the previous
                sample, current sample and the point has angles smaller equal 90
                degrees in
                the samples' corners */
                if (((fSamplesSqDist + fLastSamplePtSqDist) >=
                     fCurSamplePtSqDist) ||
                    ((bExtendEndLineSegs == TRUE) &&
                     (pPrevSample == pSamples))) {
                    if (((fSamplesSqDist + fCurSamplePtSqDist) >=
                         fLastSamplePtSqDist) ||
                        ((bExtendEndLineSegs == TRUE) &&
                         (pCurSample == &pSamples[NumSamples - (uint8)1]))) {
                        /* Both angles in the sample points are smaller 90
                        degrees,
                        the distance to the line segment is the distance to the
                        line.
                        Based on the cross product we can calculate the square
                        distance
                        by dividing the cross products square with the length of
                        the
                        sample delta vector length's square */
                        fCurLineSegDistSq = (SQR(fCrossProd) / fSamplesSqDist);
                    } else {
                        /* The angle in the current sample point is > 90
                        degrees,
                        the distance to the line segment is the distance to the
                        current sample point */
                        fCurLineSegDistSq = fCurSamplePtSqDist;
                    }
                } else {
                    /* The angle in the last sample point is > 90 degrees,
                    the distance to the line segment is the distance to the
                    last sample point */
                    fCurLineSegDistSq = fLastSamplePtSqDist;
                }
                /* Now check if the distance to the current line segment is
                smaller then the minimum found so far, if yes, store new
                minimum value NOTE: smaller equal used to catch the special
                case of when the first point is being processed: then if
                the end-point is the nearmost, then
                fCurLineSegDistSq == fSmallestAbsSqDist, but we do not yet
                know the sign, thus updated that now. */
                if (fCurLineSegDistSq <= fSmallestAbsSqDist) {
                    fSmallestAbsSqDist = fCurLineSegDistSq;
                    fSmallestDistSign = ((fCrossProd >= 0.f) ? 1.f : -1.f);
                }
            }
            /* Store last data for next iteration */
            pPrevSample = pCurSample;
            fLastSamplePtSqDist = fCurSamplePtSqDist;
        }

        /* When we reach this point, we know that fSmallesAbsSqDist is the
        square of the distance we are looking for, while fSmallestDistSign
        is the sign of it */
        fSamplesDist = SQRT_(fSmallestAbsSqDist) * fSmallestDistSign;
    }
    return fSamplesDist;
}

/*************************************************************************************************************************
  Functionname:    CPCalculatePoint2LineSegCoordArr */
fDistance_t CPCalculatePoint2LineSegCoordArr(const float32* pSamplesX,
                                             const float32* pSamplesY,
                                             uint8 NumSamples,
                                             fDistance_t fPointX,
                                             fDistance_t fPointY,
                                             boolean bExtendEndLineSegs) {
    fDistance_t fSamplesDist;
    /* Initialize return value to infinite distance */
    fSamplesDist = CP_INFINITE_DIST;
    /* If more then two samples are present, then these samples form a
     * line-segment(s) */
    if (NumSamples > (uint8)0) {
        fDistance_t fSmallestAbsSqDist;
        fDistance_t fSmallestDistSign;
        fDistance_t fLastSamplePtSqDist;
        float32 fPrevSampleX = *pSamplesX;
        float32 fPrevSampleY = *pSamplesY;
        uint8 uRemainSegs;

        pSamplesX++;

        pSamplesY++;

        /* Initialize last sample square distance to distance to first sample */
        fLastSamplePtSqDist =
            (SQR(fPointX - fPrevSampleX) + SQR(fPointY - fPrevSampleY));

        /* Also set this as the minimum (initialization) */
        fSmallestAbsSqDist = fLastSamplePtSqDist;

        /* Also set sign of distance to positive (with a single point) we
        can only have positive signs, as no directions yet known */
        fSmallestDistSign = 1.0f;

        /* Loop through line segments. Loop runs (NumSamples-1) times */
        for (uRemainSegs = (uint8)(NumSamples - 1u); uRemainSegs > (uint8)0;
             uRemainSegs--) {
            float32 fCurLineSegDistSq;

            /* Calculate delta X and delta Y between the sample points */
            const fDistance_t fSampleDX = (*pSamplesX - fPrevSampleX);
            const fDistance_t fSampleDY = (*pSamplesY - fPrevSampleY);
            /* Based on delta X & Y calculate squared distance of sample points
             */
            const float32 fSamplesSqDist = SQR(fSampleDX) + SQR(fSampleDY);
            /* Calculate current sample point square distance */
            const float32 fCurSamplePtSqDist =
                (SQR(fPointX - *pSamplesX) + SQR(fPointY - *pSamplesY));
            /* Verify that the samples are not the same (if they are, we can
            skip
            processing, since results from last line segment still apply */
            if (fSamplesSqDist > C_F32_DELTA) {
                /* Calculate cross product of line vector and vector from line
                segment
                starting point (previous sample) to the point */
                const float32 fCrossProd =
                    ((fSampleDX * (fPrevSampleY - fPointY)) -
                     (fSampleDY * (fPrevSampleX - fPointX)));
                /* Using the pythagoras theorem verify that the triangle between
                the previous
                sample, current sample and the point has angles smaller equal 90
                degrees in
                the samples' corners */
                if (((fSamplesSqDist + fLastSamplePtSqDist) >=
                     fCurSamplePtSqDist) ||
                    ((bExtendEndLineSegs == TRUE) &&
                     (uRemainSegs == (uint8)(NumSamples - 1u)))) {
                    if (((fSamplesSqDist + fCurSamplePtSqDist) >=
                         fLastSamplePtSqDist) ||
                        ((bExtendEndLineSegs == TRUE) &&
                         (uRemainSegs == (uint8)1u))) {
                        /* Both angles in the sample points are smaller 90
                        degrees,
                        the distance to the line segment is the distance to the
                        line.
                        Based on the cross product we can calculate the square
                        distance
                        by dividing the cross products square with the length of
                        the
                        sample delta vector length's square */
                        fCurLineSegDistSq = (SQR(fCrossProd) / fSamplesSqDist);
                    } else {
                        /* The angle in the current sample point is > 90
                        degrees,
                        the distance to the line segment is the distance to the
                        current sample point */
                        fCurLineSegDistSq = fCurSamplePtSqDist;
                    }
                } else {
                    /* The angle in the last sample point is > 90 degrees,
                    the distance to the line segment is the distance to the
                    last sample point */
                    fCurLineSegDistSq = fLastSamplePtSqDist;
                }
                /* Now check if the distance to the current line segment is
                smaller then the minimum found so far, if yes, store new
                minimum value NOTE: smaller equal used to catch the special
                case of when the first point is being processed: then if
                the end-point is the nearmost, then
                fCurLineSegDistSq == fSmallestAbsSqDist, but we do not yet
                know the sign, thus updated that now. */
                if (fCurLineSegDistSq <= fSmallestAbsSqDist) {
                    fSmallestAbsSqDist = fCurLineSegDistSq;
                    fSmallestDistSign = ((fCrossProd >= 0.f) ? 1.f : -1.f);
                }
            }
            /* Store last data for next iteration */
            fPrevSampleX = *pSamplesX;
            pSamplesX++;

            fPrevSampleY = *pSamplesY;
            pSamplesY++;

            fLastSamplePtSqDist = fCurSamplePtSqDist;
        }

        /* When we reach this point, we know that fSmallesAbsSqDist is the
        square of the distance we are looking for, while fSmallestDistSign
        is the sign of it */
        fSamplesDist = (SQRT_(fSmallestAbsSqDist) * fSmallestDistSign);
    }
    return fSamplesDist;
}

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */