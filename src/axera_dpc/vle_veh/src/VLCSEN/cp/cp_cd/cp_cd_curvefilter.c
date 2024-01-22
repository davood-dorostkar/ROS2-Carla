/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*---

**************************************************************************** */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cp.h"
#include "cp_cd_curvefilter.h"
#include "cp_cd_par.h"
/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/
static void CPCDCurveFilterRunInt(const CPCDCurveFilterIn_t *pIn,
                                  CPCDCurveFilterOut_t *pOut,
                                  CPCDCurveFilterDat_t *pDat,
                                  const CPCDCurveFilterPar_t *pPar);
static float32 CPCDCurveFilterCalcGainInt(const CPCDCurveFilterIn_t *pIn,
                                          const CPCDCurveFilterOut_t *pOut,
                                          CPCDCurveFilterDat_t *pDat,
                                          const CPCDCurveFilterPar_t *pPar);

/* ********************************************************************** 
Functionname               CPCDCurveFilterRun */ /*!

                          @brief         Run filter
                          @description   -

                          @return		   void

                            @param[in, out   pTraj: \n
                                                             pTraj->fCurveGradient:
                          Curve gradient related to time
                          [full range of float]\n
                                                             pTraj->fCurve:
                          Driven curve as inverse radius
                          [full range of float]\n
                            @param[in]       CPCDCurveFilterOutput: \n
                                                             CPCDCurveFilterOutput->fCurve:
                          Filtered curve
                          [full range of float]\n
                                                             CPCDCurveFilterOutput->fCurveGradient:
                          Filtered curve gradient
                          [full range of float]\n
                            @param[in]       pPar: \n
                            @param[in,out]   pDat: \n
                                                             pDat->bUpdate:
                          Update filter gain
                          [TRUE, FALSE]\n

                            @c_switch_full   VLC_CFG_COURSE_PREDICTION :
                          Configuration switch for enabling vlc course
                          prediction processing




                          @pre           -

                          @post          -
                          ****************************************************************************
                          */
void CPCDCurveFilterRun(CPCourseData_t *pTraj,
                        CPCDCurveFilterOut_t *CPCDCurveFilterOutput,
                        CPCDCurveFilterDat_t *pDat) {
    CPCDCurveFilterIn_t CPCDCurveFilterInput;

    CPCDCurveFilterInput.fCurve = pTraj->fCurve;
    CPCDCurveFilterInput.fCurveGradient = pTraj->fCurveGradient;
    CPCDCurveFilterInput.fVelocityVeh = EGO_SPEED_X_CORRECTED;
    pDat->bUpdate = TRUE;

    CPCDCurveFilterRunInt(&CPCDCurveFilterInput, CPCDCurveFilterOutput, pDat,
                          &CPCDCurveFilterPar);

    pTraj->fCurve = CPCDCurveFilterOutput->fCurve;
    pTraj->fCurveGradient = CPCDCurveFilterOutput->fCurveGradient;
}

/*********************************************************************************************************************
  Functionname               CPCDCurveFilterCalcGain */ /*!

                                                                 @brief
                                                               Calculates filter
                                                               gain i. e.
                                                               weighting of new
                                                               input data

                                                                 @description
                                                               Gain is dependent
                                                               on vehicle
                                                               velocity and a
                                                               maneuver
                                                               detection

                                                                 @return
                                                               float


                                                                 @param[in]
                                                               pIn: \n
                                                                                                  pIn->fVelocityVeh: vehicle velocity														[full range of float]\n
                                                                                                  pIn->fCurve: Driven curve as inverse radius												[full range of float]\n
                                                                 @param[in]
                                                               pOut: \n
                                                                                                  pOut->fCurve: Filtered curve																[full range of float]\n
                                                                 @param[in]
                                                               pPar: \n
                                                                                                  pPar->minVelo: Minimum velocity used for filter gain calculation							[full range of float]\n
                                                                                                  pPar->inv_dist_settled: Inverse distance where filter is settled after a step input		[full range of float]\n
                                                                                              pPar->cum_sum_limit: Upper bound value for cumsum calculation							[full range of float]\n
                                                                                                  pPar->cum_sum_drift: Aging (forgetting) of cumsm value									[full range of float]\n
                                                                                                  pPar->cum_sum_min: Lin-Gain-Ramp In:  min value of cumulated sum							[full range of float]\n
                                                                                                  pPar->cum_sum_max: Lin-Gain-Ramp In:  max value of cumulated sum							[full range of float]\n
                                                                                                  pPar->gain_min: Lin-Gain-Ramp Out: min gain at min cumulated sum							[full range of float]\n
                                                                                                  pPar->gain_max: Lin-Gain-Ramp Out: max gain at max cumulated sum							[full range of float]\n
                                                                 @param[in,out]
                                                               pDat: \n
                                                                                                  pDat->bUpdate: Update filter gain														[TRUE, FALSE]\n
                                                                                                  pDat->fGainVel: Default gain, velocity dependent											[full range of float]\n
                                                                                              pDat->fCumSum: Cumulated sum of absolute filter innovations								[full range of float]\n
                                                                                                  pDat->fGainMan: Gain during maneuver														[full range of float]\n

                                                                 @glob_in None
                                                                 @glob_out None

                                                                 @c_switch_full
                                                               VLC_CFG_COURSE_PREDICTION
                                                               : Configuration
                                                               switch for
                                                               enabling vlc
                                                               course prediction
                                                               processing

                                                                 @pre None
                                                                 @post None

                                                               *********************************************************************************************************************/
static float32 CPCDCurveFilterCalcGainInt(const CPCDCurveFilterIn_t *pIn,
                                          const CPCDCurveFilterOut_t *pOut,
                                          CPCDCurveFilterDat_t *pDat,
                                          const CPCDCurveFilterPar_t *pPar) {
    float32 filtVelo;
    float32 Gain;

    if (pDat->bUpdate != FALSE) {
        /* Limit velocity to lower bound to avoid freezing of filter (gain = 0)
         */
        filtVelo = MAX_FLOAT(pIn->fVelocityVeh, pPar->minVelo);

        /* Calculate filter gain in dependence of velocity */
        pDat->fGainVel = 1.0F - GDBexp(-VLC_CYCLE_TIME *
                                       (filtVelo * pPar->inv_dist_settled));

        /* Use cumulated sum of deviation between input and filter state for
         * maneuver detection */
        pDat->fCumSum += fABS(pIn->fCurve - pOut->fCurve);
        // pDat->cumSum = MIN_FLOAT(pDat->cumSum, +0.8F);
        pDat->fCumSum = MIN_FLOAT(pDat->fCumSum, pPar->cum_sum_limit);

        /* Aging of maneuver detection to ensure return to normal mode */
        pDat->fCumSum -= pPar->cum_sum_drift;
        pDat->fCumSum = MAX_FLOAT(pDat->fCumSum, 0);

        /* Use linear ramp to get filter gain in dependence of maneuver
         * detection */
        pDat->fGainMan = GDBmathLinFuncLimBounded(
            pDat->fCumSum, pPar->cum_sum_min, pPar->cum_sum_max, pPar->gain_min,
            pPar->gain_max);
    }

    /* Use maximum gain of maneuver and normal gain */
    Gain = MAX_FLOAT(pDat->fGainMan, pDat->fGainVel);

    return Gain;
}

/* ***********************************************************************
  Functionname               CPCDCurveFilterRun */ /*!

                          @brief         Run filter
                          @description   Filter entry point of execution

                          @param[in]       pIn: \n
                                                           pIn->fCurveGradient:
                        Curve gradient related to time
                        [full range of float]\n
                                                           pIn->fCurve: Driven
                        curve as inverse radius
                        [full range of float]\n
                          @param[in]       pOut: \n
                                                           pOut->fCurve:
                        Filtered curve
                        [full range of float]\n
                                                           pOut->fCurveGradient:
                        Filtered curve gradient
                        [full range of float]\n
                          @param[in]       pPar: \n
                          @param[in,out]   pDat: \n
                                                           pDat->bfirstCycle:
                        First cycle of filter execution
                        [TRUE, FALSE]\n

                          @c_switch_full   VLC_CFG_COURSE_PREDICTION :
                        Configuration switch for enabling vlc course prediction
                        processing

                          @pre           [none]

                          @post          [none]
                        ****************************************************************************
                        */
static void CPCDCurveFilterRunInt(const CPCDCurveFilterIn_t *pIn,
                                  CPCDCurveFilterOut_t *pOut,
                                  CPCDCurveFilterDat_t *pDat,
                                  const CPCDCurveFilterPar_t *pPar) {
    float32 Gain;

    if (pDat->bfirstCycle != FALSE) {
        /* During first cycle use input value directly to avoid long settling
         * time (starting from zero) */
        pDat->bfirstCycle = FALSE;
        pOut->fCurve = pIn->fCurve;
        pOut->fCurveGradient = pIn->fCurveGradient;
    } else {
        /* Calculate filter gain */
        Gain = CPCDCurveFilterCalcGainInt(pIn, pOut, pDat, pPar);

        /* Apply filter */
        pOut->fCurve = pOut->fCurve + (Gain * (pIn->fCurve - pOut->fCurve));
        pOut->fCurveGradient =
            pOut->fCurveGradient +
            (Gain * (pIn->fCurveGradient - pOut->fCurveGradient));
    }
    return;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */