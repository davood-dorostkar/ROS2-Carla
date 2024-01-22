/*---

**************************************************************************** */

#ifndef _CP_CD_CURVE_FILTER_INCLUDED
#define _CP_CD_CURVE_FILTER_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cp.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CPCDCurveFilterPar_t

    @general

    @conseq [ None ]

    @attention [ None ]

    */
typedef struct {
    float32 cum_sum_min; /*!< Lin-Gain-Ramp In:  min value of cumulated sum */
    float32 cum_sum_max; /*!< Lin-Gain-Ramp In:  max value of cumulated sum */
    float32 gain_min; /*!< Lin-Gain-Ramp Out: min gain at min cumulated sum */
    float32 gain_max; /*!< Lin-Gain-Ramp Out: max gain at max cumulated sum */
    float32 inv_dist_settled; /*!< Inverse distance where filter is settled
                                 after a step input */
    float32 minVelo; /*!< Minimum velocity used for filter gain calculation */
    float32 cum_sum_limit; /*!< Upper bound value for cumsum calculation */
    float32 cum_sum_drift; /*!< Aging (forgetting) of cumsm value */
} CPCDCurveFilterPar_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CPCDCurveFilterIn_t

    @general Curve filter Input data

    @conseq [ None ]

    @attention [ None ]
*/
typedef struct {
    float32 fCurve;         /*!< Driven curve as inverse radius */
    float32 fCurveGradient; /*!< Curve gradient related to time */
    float32 fVelocityVeh;   /*!< Vehicle velocity  */
} CPCDCurveFilterIn_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CPCDCurveFilterOut_t

    @general Curve filter output data

    @conseq [ None ]

    @attention [ None ]

    */
/*!  */
typedef struct {
    float32 fCurve;         /*!< Filtered curve  */
    float32 fCurveGradient; /*!< Filtered curve gradient */
} CPCDCurveFilterOut_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief CPCDCurveFilterDat_t

    @general Curve filter internal data

    @conseq [ None ]

    @attention [ None ]

    */
typedef struct {
    float32 fCumSum;     /*!< Cumulated sum of absolute filter innovations  */
    float32 fGainVel;    /*!< Default gain, velocity dependent */
    float32 fGainMan;    /*!< Gain during maneuver */
    boolean bfirstCycle; /*!< First cycle of filter execution */
    boolean bUpdate;     /*!< Update filter gain */
} CPCDCurveFilterDat_t;

/*****************************************************************************
  CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

extern void CPCDCurveFilterRun(CPCourseData_t *pTraj,
                               CPCDCurveFilterOut_t *CPCDCurveFilterOutput,
                               CPCDCurveFilterDat_t *pDat);

#endif /* end of #ifndef _CP_CD_CURVE_FILTER_INCLUDED */
