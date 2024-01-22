

/*****************************************************************************
  INCLUDES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "TM_Global_Types.h"
#include "TM_Global_defs.h"
#include "TM_CML_Types.h"
#include "TM_Base_Const.h"
#include "TM_Base_Emul.h"
#include "TM_Base_Mapping.h"
#include "TM_Base_Mat.h"
#include "TM_Base_Misc.h"

/*****************************************************************************
  GLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  LOCAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  LOCAL MACROS
*****************************************************************************/

/*****************************************************************************
  LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
  LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  LOCAL FUNCTIONS
*****************************************************************************/

/*****************************************************************************
  FUNCTION DEFINITIONS
*****************************************************************************/

/*****************************************************************************
  Functionname:    BML_f_LinearInterpolation                            */ /*!

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
float32 BML_f_LinearInterpolation(
    float32 f_X1, float32 f_Y1, float32 f_X2, float32 f_Y2, float32 f_XPos) {
    float32 f_Slope, f_Offset, f_ret;

    if (!BML_f_IsZero(f_X2 - f_X1)) {
        /* slope */
        f_Slope = (f_Y2 - f_Y1) / (f_X2 - f_X1);
        /* offset */
        f_Offset = f_Y1 - (f_Slope * f_X1);
        /* interpolate */
        f_ret = BML_f_MultAdd(f_Slope, f_XPos, f_Offset);
    } else {
        f_ret = (f_Y1 + f_Y2) * 0.5f;
    }
    return f_ret;
} /* BML_f_LinearInterpolation() */

/*****************************************************************************
  Functionname:    BML_f_BoundedLinInterpol                             */ /*!

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
float32 BML_f_BoundedLinInterpol(BML_t_LinFunctionArgs const* const p_Params,
                                 const float32 f_Value) {
    const float32 f_min = p_Params->dAmin;
    const float32 f_max = p_Params->dAmax;

    /* Geradengleichung: */
    float32 f_BoundedValue = BML_f_MultAdd(p_Params->dM, f_Value, p_Params->dB);

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
} /* BML_f_BoundedLinInterpol() */

/*****************************************************************************
  Functionname:    BML_f_BoundedLinInterpol2                            */
float32 BML_f_BoundedLinInterpol2(float32 f_IVal,
                                  float32 f_Imin,
                                  float32 f_Imax,
                                  float32 f_Omin,
                                  float32 f_Omax) {
    float32 f_OVal;

    if (BML_f_IsZero(f_Imax - f_Imin)) {
        f_OVal = (f_Omin + f_Omax) * 0.5f;
    } else {
        f_OVal = f_Omin +
                 ((f_IVal - f_Imin) * ((f_Omax - f_Omin) / (f_Imax - f_Imin)));
    }

    /* Bound output */
    if (f_Omin < f_Omax) {
        f_OVal = (BML_f_MinMax(f_Omin, f_Omax, f_OVal));
    } else {
        f_OVal = (BML_f_MinMax(f_Omax, f_Omin, f_OVal));
    }
    return f_OVal;
} /* BML_f_BoundedLinInterpol2() */

/*****************************************************************************
  Functionname:    BML_f_BoundedLinInterpol3                            */ /*!

  @brief           calculate an lin. interpolated value out of two points

  @description     This function calculates the linear interpolated value out 
                   of two points (X1,Y1) and (X2,Y2). The input value is
                   compared against the 'X' values of the given points, the
                   interpolated Y value is calculated bounded by the given
                   points and returned.

  @param[in]       Point1 : first input point
                            Optimal Range for Point1.X is [-F_MAX..F_MAX]
                            where F_MAX is one-third of the max range of float.
                            Optimal Range for Point1.Y is [-F_MAX..F_MAX]
                            where F_MAX is one-third of the max range of float.
  @param[in]       Point2 : second input point
                            Optimal Range for Point2.X is [-F_MAX..F_MAX]
                            where F_MAX is one-third of the max range of float.
                            Optimal Range for Point2.Y is [-F_MAX..F_MAX]
                            where F_MAX is one-third of the max range of float.
  @param[in]       f_In   : x-value to be interpolated
                            Optimal Range for f_In is [-F_MAX..F_MAX]
                            where F_MAX is one-third of the max range of float.

  @return          interpolated value


*****************************************************************************/
float32 BML_f_BoundedLinInterpol3(BML_t_Point2D Point1,
                                  BML_t_Point2D Point2,
                                  float32 f_In) {
    /* variables */
    float32 f_retval;
    float32 f_DeltaPX;
    BML_t_Point2D Point1_h, Point2_h;

    /* first point.x greater than second one ? */
    if (Point1.X > Point2.X) {
        /* swap points */
        Point1_h = Point2;
        Point2_h = Point1;
    } else {
        /* use points */
        Point1_h = Point1;
        Point2_h = Point2;
    }

    /* check range */
    if (f_In < Point1_h.X) {
        f_retval = Point1_h.Y;
    } else if (f_In > Point2_h.X) {
        f_retval = Point2_h.Y;
    } else {
        f_DeltaPX = Point2_h.X - Point1_h.X;
        if (!BML_f_IsZero(f_DeltaPX)) {
            f_retval =
                Point1_h.Y +
                (((f_In - Point1_h.X) * (Point2_h.Y - Point1_h.Y)) / f_DeltaPX);
        } else {
            f_retval = (Point1_h.Y + Point2_h.Y) * 0.5f;
        }
    }

    return (f_retval);
} /* BML_f_BoundedLinInterpol3() */

/*****************************************************************************
  Functionname:    BML_f_BoundedLinInterpol4                            */ /*!

  @brief           calculate an lin. interpolated value out of two vectors

  @description     This function calculate the linear interpolated value out 
                   of two given vectors (X1,Y1) and (X2,Y2). The input value is
                   compared against the 'X' distances of the given vector, the
                   interpolated Y distance is calculated bounded by the given
                   vectors and returned. The X and Y distances of vectors are 
                   defined by float values.

  @param[in]       Point1 :  first input vector
                             Optimal Range for Point1.fXDist is [-F_MAX..F_MAX]
                             where F_MAX is one-third of the max range of float.
                             Optimal Range for Point1.fYDist is [-F_MAX..F_MAX]
                             where F_MAX is one-third of the max range of float.
  @param[in]       Point2 :  second input vector
                             Optimal Range for Point2.fXDist is [-F_MAX..F_MAX]
                             where F_MAX is one-third of the max range of float.
                             Optimal Range for Point2.fYDist is [-F_MAX..F_MAX]
                             where F_MAX is one-third of the max range of float.
  @param[in]       f_In :    x-value to be interpolated
                             Optimal Range for f_In is [-F_MAX..F_MAX]
                             where F_MAX is one-third of the max range of float.

  @return          interpolated value


*****************************************************************************/
float32 BML_f_BoundedLinInterpol4(Vector2_f32_t Point1,
                                  Vector2_f32_t Point2,
                                  float32 f_In) {
    /* variables */
    float32 f_retval;
    float32 f_DeltaPX;
    Vector2_f32_t Point1_h, Point2_h;

    /* first point.x greater than second one ? */
    if (Point1.fXDist > Point2.fXDist) {
        /* swap points */
        Point1_h = Point2;
        Point2_h = Point1;
    } else {
        /* use points */
        Point1_h = Point1;
        Point2_h = Point2;
    }

    /* check range */
    if (f_In < Point1_h.fXDist) {
        f_retval = Point1_h.fYDist;
    } else if (f_In > Point2_h.fXDist) {
        f_retval = Point2_h.fYDist;
    } else {
        f_DeltaPX = Point2_h.fXDist - Point1_h.fXDist;
        if (!BML_f_IsZero(f_DeltaPX)) {
            f_retval =
                Point1_h.fYDist + (((f_In - Point1_h.fXDist) *
                                    (Point2_h.fYDist - Point1_h.fYDist)) /
                                   f_DeltaPX);
        } else {
            f_retval = (Point1_h.fYDist + Point2_h.fYDist) * 0.5f;
        }
    }

    return (f_retval);
} /* BML_f_BoundedLinInterpol4() */

/*****************************************************************************
  Functionname:    BML_f_BoundedLinInterpol5                            */ /*!

  @brief           calculate an lin. interpolated value out of two vectors

  @description     This function calculate the linear interpolated value out 
                   of two given vectors (X1,Y1) and (X2,Y2). The input value is
                   compared against the 'X' distances of the given vector, the
                   interpolated Y distance is calculated bounded by the given
                   vectors and returned. The X and Y distances of vectors are 
                   defined by integer values.

  @param[in]       Point1 :  first input vector
                             Range for Point1.nXDist is [Full range of signed long]
                             Range for Point1.nYDist is [Full range of signed long]
  @param[in]       Point2 :  second input vector
                             Range for Point2.nXDist is [Full range of signed long]
                             Range for Point2.nYDist is [Full range of signed long]
  @param[out]      f_In :    x-value to be interpolated
                             Optimal Range for f_In is [-F_MAX..F_MAX]
                             where F_MAX is one-third of the max range of float.

  @return          interpolated value


*****************************************************************************/
float32 BML_f_BoundedLinInterpol5(Vector2_i32_t Point1,
                                  Vector2_i32_t Point2,
                                  float32 f_In) {
    /* variables */
    float32 f_retval;
    sint32 s_DeltaPX;
    Vector2_i32_t Point1_h, Point2_h;

    /* first point.x greater than second one ? */
    if (Point1.nXDist > Point2.nXDist) {
        /* swap points */
        Point1_h = Point2;
        Point2_h = Point1;
    } else {
        /* use points */
        Point1_h = Point1;
        Point2_h = Point2;
    }

    /* check range */
    if (f_In < (float32)Point1_h.nXDist) {
        f_retval = (float32)Point1_h.nYDist;
    } else if (f_In > (float32)Point2_h.nXDist) {
        f_retval = (float32)Point2_h.nYDist;
    } else {
        s_DeltaPX = Point2_h.nXDist - Point1_h.nXDist;
        if (!BML_f_IsZero((float32)s_DeltaPX)) {
            f_retval =
                (float32)Point1_h.nYDist +
                (((f_In - (float32)Point1_h.nXDist) *
                  ((float32)Point2_h.nYDist - (float32)Point1_h.nYDist)) /
                 (float32)s_DeltaPX);
        } else {
            f_retval =
                ((float32)Point1_h.nYDist + (float32)Point2_h.nYDist) * 0.5f;
        }
    }

    return (f_retval);
} /* BML_f_BoundedLinInterpol5() */

/*****************************************************************************
  Functionname:    CML_f_CalculatePolygonValue                          */ /*!

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
float32 CML_f_CalculatePolygonValue(sint32 s_NrOfTableRows,
                                    const GDBVector2_t a_Table[],
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

        if (f_dx > C_F32_DELTA) {
            f_WeightRight = (f_InputValue - a_Table[s_Right - 1].f0) / (f_dx);
        } else {
            f_WeightRight = 0.0f;
        }

        f_Result = ((a_Table[s_Right - 1].f1) * (1.0f - f_WeightRight)) +
                   ((a_Table[s_Right].f1) * f_WeightRight);
    }
    return f_Result;
} /* CML_f_CalculatePolygonValue() */

void CML_CCSC_CalcBMatrix(const uint32 u_NrOfShapePoints,
                          float32 f_DeltaX_i,
                          float32 f_DeltaX_im1Abs,
                          float32 f_DeltaX_im1,
                          uint32 i,
                          float32 a_BMatrix[]) {
    if (((fABS(f_DeltaX_i) > C_F32_DELTA) || (i >= (u_NrOfShapePoints - 1u))) &&
        ((f_DeltaX_im1Abs > C_F32_DELTA) || (i == 0u))) {
        /* Set the i-1-th element in each column */
        if (i > 1u) {
            a_BMatrix[(i * u_NrOfShapePoints) + (i - 1u)] =
                -1.f / f_DeltaX_im1; /*!< Remark: No devision by zero possible
                                        since (i > 1u) (see line 1478) */
        } else if (i == 1u) {
            a_BMatrix[i * u_NrOfShapePoints] = 0.f; /*!< Remark: (i - 1u) = 0 */
        } else {
            /* i==0 -> i-1 is invalid -> Nothing */
        }

        /* Set the i-th element in each column */
        if ((i > 0u) && (i < (u_NrOfShapePoints - 1u))) {
            a_BMatrix[(i * u_NrOfShapePoints) + i] =
                2.f * ((1.f / f_DeltaX_im1) +
                       (1.f / f_DeltaX_i)); /* Remark: No devision by zero
                                               possible since (i > 0u) and
                                               (i < (ui_NrOfShapePoints -
                                               1u)) (see line 1477f) */
        } else {
            a_BMatrix[(i * u_NrOfShapePoints) + i] = 1.f;
        }

        /* Set the i+1-th element in each column */
        if ((i + 2U) < u_NrOfShapePoints) {
            a_BMatrix[(i * u_NrOfShapePoints) + (i + 1u)] =
                1.f / f_DeltaX_i; /* Remark: No devision by zero possible
                                     since (i < (ui_NrOfShapePoints - 2u))
                                     (see line 1477) */
        } else if ((i + 2U) == u_NrOfShapePoints) {
            a_BMatrix[(i * u_NrOfShapePoints) + (i + 1u)] = 0.f;
        } else {
            /* i==ui_NrOfTableRows - 1u -> (i - 1u) * ui_NrOfTableRows + i +
             * 1u is invalid -> Nothing */
        }
    } else {
        a_BMatrix[(i * u_NrOfShapePoints) + i] = 1.f;
    }
}

/* ***********************************************************************
  Functionname:    CML_CalculateCubicSplineClamped                  */ /*!

  @brief           Determine a "clamped" cubic spline (y'[0] = f_SlopeStart 
                   and y'[end] = f_SlopeEnd) based on given shape points

  @description     This function determine a "clamped" cubic spline 
                   (y'[0] = f_SlopeStart and y'[end] = f_SlopeEnd) based on 
                   given shape points. The coefficient arrays from the 
                   following formula are calculated.
                   y = A[i] + B[i] * (x - x_i) + C[i]*(x - x_i)^2 + D[i]*(x - x_i)^3 
                   for x in [x_i x_(i+1)]
                   i = 0 till (ui_NrOfShapePoints - 1)

  @param[in]       u_NrOfShapePoints :     Number of shape points
                                           optimal range of values [0,..,255]
  @param[in]       a_ShapePointTable :     Table containing the x- and y-coordinates 
                                           of the shape points; size: ui_NrOfShapePoints x 1
                                           [Valid float vector array]
  @param[in]       f_SlopeStart :          Slope at the first shape point
                                           [Full range of float32]
                                           Overflow may occur when input values 
                                           are at the defined range extremities.
  @param[in]       f_SlopeEnd :            Slope at the last shape point
                                           [Full range of float32]
                                           Overflow may occur when input values 
                                           are at the defined range extremities.
  @param[in]       a_BMatrix :             Matrix for calculating a_ParaB; 
                                           [Valid float array]
                                           size: ui_NrOfShapePoints x ui_NrOfShapePoints
  @param[in]       a_BMatrixInv :          Inverse matrix of a_BMatrix -> 
                                           a_BMatrixInv = a_BMatrix^-1;
                                           [Valid float array]
                                           size: ui_NrOfShapePoints x ui_NrOfShapePoints
  @param[in]       a_Temp :                Temporary matrix for calculating a_BMatrixInv; 
                                           [Valid float array]
                                           size: ui_NrOfShapePoints x ui_NrOfShapePoints
  @param[in]       a_YVector :             Vector for calculating a_ParaB, contains 
                                           x- and y-differences between shape points; 
                                           [Valid float array]
                                           size: ui_NrOfShapePoints x 1
  @param[in]       a_BInvXYVec :           Vector for calculating 
                                           a_ParaB -> a_BInvXYVec = a_BMatrixInv * a_YVector; 
                                           [Valid float array]
                                           size: ui_NrOfShapePoints x 1
  @param[in]       a_SegmentSplineValid :  array with data if a spline segment is valid or not.
                                           [Valid Boolean array]

  @param[out]      a_ParaA :               Coefficient A for each segment
                                           [Valid float array]
  @param[out]      a_ParaB :               Coefficient B for each segment 
                                           [Valid float array]
  @param[out]      a_ParaC :               Coefficient C for each segment 
                                           [Valid float array]
  @param[out]      a_ParaD :               Coefficient D for each segment
                                           [Valid float array]
                                           (number of segments = ui_NrOfShapePoints - 1)


  @return          b_ResultValid :         Info if results are valid
**************************************************************************** */
boolean CML_CalculateCubicSplineClamped(const uint32 u_NrOfShapePoints,
                                        const Vector2_f32_t a_ShapePointTable[],
                                        const float32 f_SlopeStart,
                                        const float32 f_SlopeEnd,
                                        float32 a_BMatrix[],
                                        float32 a_BMatrixInv[],
                                        float32 a_Temp[],
                                        float32 a_YVector[],
                                        float32 a_BInvXYVec[],
                                        float32 a_ParaA[],
                                        float32 a_ParaB[],
                                        float32 a_ParaC[],
                                        float32 a_ParaD[],
                                        boolean a_SegmentSplineValid[]) {
    boolean b_ResultValid;
    uint32 i, j;
    float32 f_DeltaX_i, f_DeltaY_i, f_DeltaX_im1, f_DeltaY_im1;
    float32 f_DeltaX_im1Abs;

    /* Initialization of output structures */
    for (i = 0u; i < (u_NrOfShapePoints - 1u); i++) {
        a_ParaA[i] = 0.f;
        a_ParaB[i] = 0.f;
        a_ParaC[i] = 0.f;
        a_ParaD[i] = 0.f;
        a_SegmentSplineValid[i] = FALSE;
    }

    /* Generate a_YVector; contains x- and y-differences between shape points;
     * size: ui_NrOfTableRows x 1 -> a_BMatrix * B = a_YVector */
    a_YVector[0u] = f_SlopeStart;
    a_YVector[u_NrOfShapePoints - 1u] = f_SlopeEnd;

    /* f_DeltaX_im1: Difference of x-coordinates between the i-th and i-1-th
     * sample point */
    f_DeltaX_im1 = a_ShapePointTable[1].fXDist - a_ShapePointTable[0].fXDist;
    /* f_DeltaY_im1: Difference of y-coordinates between the i-th and i-1-th
     * sample point */
    f_DeltaY_im1 = a_ShapePointTable[1u].fYDist - a_ShapePointTable[0u].fYDist;
    for (i = 1u; i < (u_NrOfShapePoints - 1u); i++) {
        /* f_DeltaX_i: Difference of x-coordinates between the i+1-th and i-th
         * sample point */
        f_DeltaX_i =
            a_ShapePointTable[i + 1u].fXDist - a_ShapePointTable[i].fXDist;
        /* f_DeltaY_i: Difference of y-coordinates between the i+1-th and i-th
         * sample point */
        f_DeltaY_i =
            a_ShapePointTable[i + 1u].fYDist - a_ShapePointTable[i].fYDist;

        /* Compare to page 17 */
        f_DeltaX_im1Abs = fABS(f_DeltaX_im1);
        if ((fABS(f_DeltaX_i) > C_F32_DELTA) &&
            (f_DeltaX_im1Abs > C_F32_DELTA)) {
            a_YVector[i] = 3.f * ((f_DeltaY_im1 / SQR(f_DeltaX_im1)) +
                                  (f_DeltaY_i / SQR(f_DeltaX_i)));
        } else {
            a_YVector[i] = 0.f;
        }

        /* Set f_DeltaX_im1 and f_DeltaY_im1 for the next iteration */
        f_DeltaX_im1 = f_DeltaX_i;
        f_DeltaY_im1 = f_DeltaY_i;
    }

    /* Generate a_BMatrix; size: ui_NrOfTableRows x ui_NrOfTableRows ->
     * a_BMatrix * B = a_YVector, compare to page 17 */
    f_DeltaX_im1 = 0.f;
    for (i = 0u; i < u_NrOfShapePoints; i++) {
        if (i < (u_NrOfShapePoints - 1u)) {
            f_DeltaX_i =
                a_ShapePointTable[i + 1u].fXDist - a_ShapePointTable[i].fXDist;
        } else {
            f_DeltaX_i = 0.f;
        }

        /* Initialize each column to zero */
        for (j = 0u; j < u_NrOfShapePoints; j++) {
            a_BMatrix[(i * u_NrOfShapePoints) + j] = 0.f;
        }

        /* Only if f_DeltaX_i and f_DeltaX_im1 are valid; otherwise the column
           is set the the identity vector
            -> later on, the parameters of the segments with f_DeltaX_i == 0 are
           set to zero */
        f_DeltaX_im1Abs = fABS(f_DeltaX_im1);
        CML_CCSC_CalcBMatrix(u_NrOfShapePoints, f_DeltaX_i, f_DeltaX_im1Abs,
                             f_DeltaX_im1, i, a_BMatrix);

        /* Set f_DeltaX_im1 and f_DeltaY_im1 for the next iteration */
        f_DeltaX_im1 = f_DeltaX_i;
    }

    /* a_BMatrixInv = a_BMatrix^-1 */
    /* Set validity value of results to FALSE if matrix inverting invalid */
    b_ResultValid = (boolean)BML_u_MatrixInversion(
        a_BMatrix, a_Temp, a_BMatrixInv, (uint8)u_NrOfShapePoints);

    /* a_BInvXYVec = a_BMatrixInv * a_YVector -> a_ParaB[0:ui_NrOfTableRows -
     * 2u] */
    if (b_ResultValid == TRUE) {
        b_ResultValid = BML_u_MatrixMultiplication(
            a_BMatrixInv, a_YVector, a_BInvXYVec, (uint8)u_NrOfShapePoints,
            (uint8)u_NrOfShapePoints, 1u);
    }

    if (b_ResultValid == TRUE) {
        /* a_ParaB -> size: ui_NrOfTableRows-1 since ui_NrOfTableRows number of
         * sample points and ui_NrOfTableRows-1 splines */
        for (i = 0u; i < (u_NrOfShapePoints - 1u); i++) {
            f_DeltaX_i =
                a_ShapePointTable[i + 1u].fXDist - a_ShapePointTable[i].fXDist;
            if (fABS(f_DeltaX_i) > C_F32_DELTA) {
                a_ParaB[i] = a_BInvXYVec[i];
            }
            /*else
            {
              a_ParaB[i] = 0.f;
            }*/
        }

        /* a_ParaA/a_ParaC/a_ParaD */
        /* size a_ParaA/a_ParaC/a_ParaD: ui_NrOfTableRows-1 since
         * ui_NrOfTableRows number of sample points and ui_NrOfTableRows-1
         * splines */
        for (i = 0u; i < (u_NrOfShapePoints - 1u); i++) {
            f_DeltaX_i =
                a_ShapePointTable[i + 1u].fXDist - a_ShapePointTable[i].fXDist;
            if (fABS(f_DeltaX_i) > C_F32_DELTA) {
                /* Parameters for segment are valid if fABS(f_DeltaX_i) >
                 * C_F32_DELTA */
                a_SegmentSplineValid[i] = TRUE;

                /* a_ParaA */
                a_ParaA[i] = a_ShapePointTable[i].fYDist;

                /* a_ParaC, a_ParaD */
                f_DeltaY_i = a_ShapePointTable[i + 1u].fYDist -
                             a_ShapePointTable[i].fYDist;
                if ((i + 2U) < u_NrOfShapePoints) {
                    /* a_ParaC */
                    a_ParaC[i] =
                        (((3.f * (f_DeltaY_i / f_DeltaX_i)) - a_ParaB[i + 1u]) -
                         (2.f * a_ParaB[i])) /
                        f_DeltaX_i;
                    /* a_ParaD */
                    a_ParaD[i] = ((a_ParaB[i + 1u] + a_ParaB[i]) -
                                  (2.f * (f_DeltaY_i / f_DeltaX_i))) /
                                 (SQR(f_DeltaX_i));
                } else {
                    /* i == ui_NrOfTableRows -2u -> a_ParaC[i + 1] =
                     * a_ParaC[ui_NrOfTableRows - 1u] doesn't exit and is
                     * assumed to be 0 */
                    /* pf_ParaBC*/
                    a_ParaC[i] = (((3.f * (f_DeltaY_i / f_DeltaX_i)) -
                                   a_BInvXYVec[i + 1u]) -
                                  (2.f * a_ParaB[i])) /
                                 f_DeltaX_i;
                    /* a_ParaD */
                    a_ParaD[i] = ((a_BInvXYVec[i + 1u] + a_ParaB[i]) -
                                  (2.f * (f_DeltaY_i / f_DeltaX_i))) /
                                 (SQR(f_DeltaX_i));
                }
            }
        }
    }

    return b_ResultValid;
} /* CML_CalculateCubicSplineClamped() */

// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"