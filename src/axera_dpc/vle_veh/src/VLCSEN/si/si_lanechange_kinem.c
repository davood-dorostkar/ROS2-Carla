/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "si.h"
#include "si_lanechange.h"

/*****************************************************************************
  MODULEGLOBAL CONSTANTS
*****************************************************************************/

/*! Yawrate limits for left and right lane changes...*/

#define YAWRATE_RIGHTLANECHANGE_LOWLIMIT (-0.05f)
#define YAWRATE_RIGHTLANECHANGE_HIGHLIMIT (0.03f)

#define YAWRATE_LEFTLANECHANGE_LOWLIMIT (-0.03f)
#define YAWRATE_LEFTLANECHANGE_HIGHLIMIT (0.05f)

/*! Minimum value for DMMRYAW value  */
#define SI_DMMRYAW_MINVAL (1.e8f)

/*! Minimum speed for calculation of kinematic lane change probability */
#define SI_LC_MIN_SPEED_KINEM (60.0f / C_KMH_MS)

/*! Parameters for the GEV-Distribution of raw yaw rate - Left lane change */
#define SI_LC_LEFT_YAW_RATE_K (0.0808861f)     /*(0.128861f)*/
#define SI_LC_LEFT_YAW_RATE_SIGMA (0.0072599f) /*(0.012599f)*/
#define SI_LC_LEFT_YAW_RATE_MU (0.0195819f)    /*(0.0195819f)*/

/*! Parameters for the GEV-Distribution of raw yaw rate - Right lane change */
#define SI_LC_RIGHT_YAW_RATE_K (-0.4520063f)      /*(-0.530063f)*/
#define SI_LC_RIGHT_YAW_RATE_SIGMA (0.012154861f) /*(0.014861f)*/
#define SI_LC_RIGHT_YAW_RATE_MU (-0.0301606f)     /*(-0.029606f)*/

/*! Parameters for the standard normal distribution of raw yaw rate - Follow
 * lane */
#define SI_LC_FOLLOW_YAW_RATE_MU (0.0f)
#define SI_LC_FOLLOW_YAW_RATE_SIGMA (0.025f)

/*! Parameters for the GEV-Distribution of DMMR Yaw rate - Left lane change */
#define SI_LC_LEFT_DMMR_YAW_RATE_K (0.861009f)
#define SI_LC_LEFT_DMMR_YAW_RATE_SIGMA (0.00118437f)
#define SI_LC_LEFT_DMMR_YAW_RATE_MU (0.00107065f)

/*! Parameters for the GEV-Distribution of DMMR Yaw rate - Right lane change */
#define SI_LC_RIGHT_DMMR_YAW_RATE_K (0.823817f)
#define SI_LC_RIGHT_DMMR_YAW_RATE_SIGMA (0.00104879f)
#define SI_LC_RIGHT_DMMR_YAW_RATE_MU (0.00103833f)

/*! Parameters for the standard normal Distribution of DMMR Yaw rate - Follow
 * lane*/
#define SI_LC_FOLLOW_DMMR_YAW_RATE_MU (0.001f)
#define SI_LC_FOLLOW_DMMR_YAW_RATE_SIGMA (0.00056f)

/*! Parameters for the GEV-Distribution of cumulative sum distance to road
 * border- Left lane change */
#define SI_LC_LEFT_CUMSUM_DIST_ROAD_MU (0.35f)
#define SI_LC_LEFT_CUMSUM_DIST_ROAD_SIGMA (0.3f)

/*! Parameters for the GEV-Distribution of cumulative sum distance to road
 * border - Right lane change */
#define SI_LC_RIGHT_CUMSUM_DIST_ROAD_MU (-0.35f)
#define SI_LC_RIGHT_CUMSUM_DIST_ROAD_SIGMA (0.3f)

/*! Parameters for the standard normal Distribution of DMMR  distance to road
 * border - Follow lane*/
#define SI_LC_FOLLOW_CUMSUM_DIST_ROAD_MU (0.0f)
#define SI_LC_FOLLOW_CUMSUM_DIST_ROAD_SIGMA (0.3f)

/*! Parameters for the GEV-Distribution of aflat - Left lane change */
#define SI_LC_LEFT_AFLAT_K (-0.457638f)
#define SI_LC_LEFT_AFLAT_SIGMA (0.184667f)
#define SI_LC_LEFT_AFLAT_MU (0.452301f)

/*! Parameters for the GEV-Distribution of aflat - Right lane change */
#define SI_LC_RIGHT_AFLAT_K (-0.207147f)
#define SI_LC_RIGHT_AFLAT_SIGMA (0.1650719f)
#define SI_LC_RIGHT_AFLAT_MU (-0.562272f)

/*! Parameters for the standard normal Distribution of aflat - Follow lane*/
#define SI_LC_FOLLOW_AFLAT_MU (0.0f)
#define SI_LC_FOLLOW_AFLAT_SIGMA (0.357567f)

/*! Counter for the approximated yaw rate curve during lane change */
#define SI_LC_LCS_YAW_COUNTER_MAX (20)

/*! Max number of samples for the DMMR yaw calculation */
#define SI_LC_DMMR_YAW_MAX_SAMPLES (5)

/*! Max number of samples for the cumulative sum distance calculation */
#define SI_LC_CUMSUM_DIST_MAX_SAMPLES (10)

/*! Minimum Road Border confidence */
#define SI_LC_ROAD_BORDER_MIN_STAT (84u)

/*! Minimum Road Border tracking status*/
#define SI_LC_ROAD_BORDER_MIN_TRACKING_STAT (7u)

/* Maximum summation value for DMMR distance */
#define SI_LC_CUMSUM_DIST_MAX_SUM (3.0f)

/*****************************************************************************
  MODULGLOBAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SET_MEMSEC_VAR(SILaneChangeSILaneChangeCUMSUMDistDMMRDist)
static struct SILaneChangeCUMSUMDist {
    float32 fDMMRSamples[SI_LC_CUMSUM_DIST_MAX_SAMPLES];
    float32 fLastValue;
    uint8 uiRunIndex;
} SILaneChangeCUMSUMDist;

SET_MEMSEC_VAR(SILaneChangeDMMRYaw)
static struct SILaneChangeDMMRYaw {
    float32 fDMMRSamples[SI_LC_DMMR_YAW_MAX_SAMPLES];
    float32 fLastValue;
    uint8 uiRunIndex;
} SILaneChangeDMMRYaw;

SET_MEMSEC_VAR(fMovingAverageYawRate)
static float32 fMovingAverageYawRate;

SET_MEMSEC_VAR(fLCProbOverall)
static float32 fLCProbOverall;

SET_MEMSEC_VAR(fLCDetectState)
static float32 fLCDetectState;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MODULE LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL MACROS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL TYPES
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void SICalculateInputSignals(const float32* const pfYawRate,
                                    const float32* const pfDistToRoad,
                                    boolean bDistValid,
                                    float32* const pfDMMRYaw,
                                    float32* const pfDMMRDist);

static float32 SICalcCUMSUMDistance(const float32 fCurrentVal);
static float32 SICalcDMMRYaw(const float32 fCurrentVal);
static void SIInitRoadParameters(void);

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* ***********************************************************************
  @fn             SIInitLaneChangeKinem                           */ /*!

        @brief          Initialize kinematic lane change detection module

        @description    Function initializes kinematic lane change detection
      module

        @return         None

        @pre            None

      ****************************************************************************
      */
void SIInitLaneChangeKinem(void) {
    uint8 i;

    /* Initialize static variables */
    fMovingAverageYawRate = 0.0f;
    fLCProbOverall = 0.0f;
    fLCDetectState = 0.0f;

    for (i = 0; i < SI_LC_DMMR_YAW_MAX_SAMPLES; i++) {
        SILaneChangeDMMRYaw.fDMMRSamples[i] = 0.0f;
    }

    for (i = 0; i < SI_LC_CUMSUM_DIST_MAX_SAMPLES; i++) {
        SILaneChangeCUMSUMDist.fDMMRSamples[i] = 0.0f;
    }

    SILaneChangeCUMSUMDist.fLastValue = 0.0f;
    SILaneChangeDMMRYaw.fLastValue = 0.0f;

    SILaneChangeCUMSUMDist.uiRunIndex = 0u;
    SILaneChangeDMMRYaw.uiRunIndex = 0u;
}

/* ***********************************************************************
  @fn             SIInitRoadParameters                           */ /*!

         @brief          Initialize road border specific parameters

         @description    Initialize road border specific parameters

         @param          -

         @return         None

         @pre            None

       ****************************************************************************
       */
static void SIInitRoadParameters(void) {
    uint8 i;

    for (i = 0; i < SI_LC_CUMSUM_DIST_MAX_SAMPLES; i++) {
        SILaneChangeCUMSUMDist.fDMMRSamples[i] = 0.0f;
    }

    SILaneChangeCUMSUMDist.fLastValue = 0.0f;
    SILaneChangeCUMSUMDist.uiRunIndex = 0u;
}

/* ***********************************************************************
  @fn             SICalculateKinematicLCProb                           */ /*!

   @brief          Calculate the lane change probability based on kinematic
 signals

   @description    Calculate the lane change probability based on kinematic
 signals

   @return         None

   @pre            None

 **************************************************************************** */
void SICalculateKinematicLCProb(void) {
    float32 fRoadBorder;
    float32 fYawRate;
    float32 afLatDiffFilterCurve;
    float32 fDMMRYaw, fDMMRDist;
    float32 fLCProbLeft, fLCProbRight, fLCProbFollow, fLCProbSum;
    float32 fLCPaLeft, fLCPaRight, fLCPaFollow;
    const float32 fEgoSpeed = EGO_SPEED_X_OBJ_SYNC;
    float32 fLCDMMRYaw, fLCDMMRDist, fLCYaw, fLCAflat;

    boolean bRoadBorderValid = FALSE;
    boolean bUsedTurnIndicator = FALSE;

    const eTurnIndicator_t t_FiltTurnIndState = SI_LC_t_GetFilterTurnIndState();
    /* Weighting for moving average */
    const float32 fAlpha = 0.15f;

    /* Init probability values */
    fLCProbLeft = 0.0f;
    fLCProbRight = 0.0f;
    fLCProbFollow = 1.0f;

    /* Get yaw rate */
    fYawRate = EGO_YAW_RATE_OBJ_SYNC;

    /* Get lateral difference of filtered curve */
    afLatDiffFilterCurve = SILCProbDataGlobal.afLatDiffFilteredCurvesFilt[0];

    if (fEgoSpeed > SI_LC_MIN_SPEED_KINEM) {
        /* Set a priori probabilities for left and right lane change as well as
         * lane follow */
        /* These probabilities are chosen to yield a good trade-off between
           performance and false positive
            rate. They do not reflect the true a priori values.  */
        /* Right lane changes are harder to detect which is reasoned in the way
           they are executed, i.e. the
           variation is much larger compared to a left lane change. In order to
           improve the false positive
           rate, the a priori probability for a right lane change without turn
           signal indicator is set to zero. */
        if (bRoadBorderValid != FALSE) {
            /* temporarily switched off to fulfill time gap criteria. Switch on
             * when complete structure is available */
            fLCPaLeft = 0.0f;
            fLCPaRight = 0.0f;
            fLCPaFollow = 1.0f;
        } else {
            fLCPaLeft = 0.0f;
            fLCPaRight = 0.0f;
            fLCPaFollow = 1.0f;

            SIInitRoadParameters();
        }

        if (t_FiltTurnIndState == eTurnIndicator_Left) {
            fLCPaLeft = 0.70f;
            fLCPaRight = 0.01f;
            fLCPaFollow = 0.29f;
            bUsedTurnIndicator = TRUE;
        }

        if (t_FiltTurnIndState == eTurnIndicator_Right) {
            fLCPaLeft = 0.01f;
            fLCPaRight = 0.70f;
            fLCPaFollow = 0.29f;
            bUsedTurnIndicator = TRUE;
        }

        /* Calculate all required input signals. Currently based on yaw rate and
         * distance to road border */
        SICalculateInputSignals(&fYawRate, &fRoadBorder, bRoadBorderValid,
                                &fDMMRYaw, &fDMMRDist);

        /* Calculate lane change probability based on kinematic signals */

        if (((fYawRate > YAWRATE_LEFTLANECHANGE_LOWLIMIT) &&
             (fYawRate < YAWRATE_LEFTLANECHANGE_HIGHLIMIT)) ||
            (bUsedTurnIndicator != FALSE)) {
            /* Only calculate the lc prob for a yaw rate within given limits.
             * Otherwise false alarm rate is very high */
            fLCYaw = SICalcGEVDProb(fYawRate, SI_LC_LEFT_YAW_RATE_K,
                                    SI_LC_LEFT_YAW_RATE_SIGMA,
                                    SI_LC_LEFT_YAW_RATE_MU);
        } else {
            fLCYaw = 0.0f;
        }
        fLCDMMRYaw = SICalcGEVDProb(fDMMRYaw, SI_LC_LEFT_DMMR_YAW_RATE_K,
                                    SI_LC_LEFT_DMMR_YAW_RATE_SIGMA,
                                    SI_LC_LEFT_DMMR_YAW_RATE_MU);
        if (bRoadBorderValid != FALSE) {
            fLCDMMRDist =
                SICalcGDProb(fDMMRDist, SI_LC_LEFT_CUMSUM_DIST_ROAD_MU,
                             SI_LC_LEFT_CUMSUM_DIST_ROAD_SIGMA);
        } else {
            /* Road is not available. This should not affect the overall
             * probability. */
            fLCDMMRDist = 1.0f;
        }

        fLCAflat = SICalcGEVDProb(afLatDiffFilterCurve, SI_LC_LEFT_AFLAT_K,
                                  SI_LC_LEFT_AFLAT_SIGMA, SI_LC_LEFT_AFLAT_MU);

        fLCProbLeft = fLCYaw * fLCDMMRYaw * fLCDMMRDist * fLCAflat * fLCPaLeft;

        if (((fYawRate > YAWRATE_RIGHTLANECHANGE_LOWLIMIT) &&
             (fYawRate < YAWRATE_RIGHTLANECHANGE_HIGHLIMIT)) ||
            (bUsedTurnIndicator != FALSE)) {
            /* Only calculate the lc prob for a yaw rate within given limits.
             * Otherwise false alarm rate is very high */
            fLCYaw = SICalcGEVDProb(fYawRate, SI_LC_RIGHT_YAW_RATE_K,
                                    SI_LC_RIGHT_YAW_RATE_SIGMA,
                                    SI_LC_RIGHT_YAW_RATE_MU);
        } else {
            fLCYaw = 0.0f;
        }
        fLCDMMRYaw = SICalcGEVDProb(fDMMRYaw, SI_LC_RIGHT_DMMR_YAW_RATE_K,
                                    SI_LC_RIGHT_DMMR_YAW_RATE_SIGMA,
                                    SI_LC_RIGHT_DMMR_YAW_RATE_MU);
        if (bRoadBorderValid != FALSE) {
            fLCDMMRDist =
                SICalcGDProb(fDMMRDist, SI_LC_RIGHT_CUMSUM_DIST_ROAD_MU,
                             SI_LC_RIGHT_CUMSUM_DIST_ROAD_SIGMA);
        } else {
            /* Road is not available. This should not affect the overall
             * probability. */
            fLCDMMRDist = 1.0f;
        }
        fLCAflat =
            SICalcGEVDProb(afLatDiffFilterCurve, SI_LC_RIGHT_AFLAT_K,
                           SI_LC_RIGHT_AFLAT_SIGMA, SI_LC_RIGHT_AFLAT_MU);

        fLCProbRight =
            fLCYaw * fLCDMMRYaw * fLCDMMRDist * fLCAflat * fLCPaRight;

        /***************************************************************
         *    LANE FOLLOW                                              *
         ***************************************************************/
        fLCYaw = SICalcGDProb(fYawRate, SI_LC_FOLLOW_YAW_RATE_MU,
                              SI_LC_FOLLOW_YAW_RATE_SIGMA);
        fLCDMMRYaw = SICalcGDProb(fDMMRYaw, SI_LC_FOLLOW_DMMR_YAW_RATE_MU,
                                  SI_LC_FOLLOW_DMMR_YAW_RATE_SIGMA);
        if (bRoadBorderValid != FALSE) {
            fLCDMMRDist =
                SICalcGDProb(fDMMRDist, SI_LC_FOLLOW_CUMSUM_DIST_ROAD_MU,
                             SI_LC_FOLLOW_CUMSUM_DIST_ROAD_SIGMA);
        } else {
            /* Road is not available. */
            fLCDMMRDist = 1.0f;
        }

        fLCAflat = SICalcGDProb(afLatDiffFilterCurve, SI_LC_FOLLOW_AFLAT_MU,
                                SI_LC_FOLLOW_AFLAT_SIGMA);

        fLCProbFollow =
            fLCYaw * fLCDMMRYaw * fLCDMMRDist * fLCAflat * fLCPaFollow;

        fLCProbSum = fLCProbLeft + fLCProbRight + fLCProbFollow;
    } else {
        /* We are too slow, thus the kinematic observer doesn't work reliably.
          Mostly caused by missing
          input such as a reliable road border estimation. */
        fLCProbSum = 0.0f;
    }

    if (fLCProbSum > BML_f_AlmostZero) {
        fLCProbLeft = fLCProbLeft / fLCProbSum;
        fLCProbRight = fLCProbRight / fLCProbSum;
        fLCProbFollow = fLCProbFollow / fLCProbSum;
    } else {
        /* The overall probabilities are close to zero. In order to keep the
           false positive rate happy
           we say its lane follow */
        fLCProbLeft = 0.0f;
        fLCProbRight = 0.0f;
        fLCProbFollow = 1.0f;
    }

    /* Calculate moving average for overall lane change probability */
    fLCProbOverall = (fAlpha * MAX(fLCProbLeft, fLCProbRight)) +
                     ((1 - fAlpha) * fLCProbOverall);

    /* Determine the lane change state (LC_LEFT, LC_RIGHT, LC_FOLLOW) */
    if (fLCProbOverall > 0.001f) {
        if (fLCProbLeft > fLCProbRight) {
            fLCDetectState += LC_LEFT;
        } else {
            fLCDetectState += LC_RIGHT;
        }
    }

    /* Save probability */
    SILCProbDataGlobal.fLCKinematicProb = 100.0f * fLCProbOverall;

    SILCProbDataGlobal.fLCProbLeft = fLCProbLeft;
    SILCProbDataGlobal.fLCProbRight = fLCProbRight;
    SILCProbDataGlobal.fLCProbFollow = fLCProbFollow;
}

/* ***********************************************************************
  @fn             SICalculateInputSignals                           */ /*!

      @brief          Calculate additional input signals for the Bayes
    classifier

      @description    Calculate additional input signals for the Bayes
    classifier

      @param[in]      fYawRate : Raw signal yaw rate

      @param[in]      fDistToRoad : Raw signal distance to road border

      @param[in]      bDistValid : Flag to indicate whether distance to road is
    plausible

      @param[out]     pfDMMRYaw : DMMR yaw rate

      @param[out]     pfDMMRDist : DMMR Distance to road border

      @return         None

    ****************************************************************************
    */
static void SICalculateInputSignals(const float32* const pfYawRate,
                                    const float32* const pfDistToRoad,
                                    boolean bDistValid,
                                    float32* const pfDMMRYaw,
                                    float32* const pfDMMRDist) {
    /* Weighting for moving average */
    const float32 fAlpha = 0.1f;

    /* Calculate the moving average of yaw rate */
    fMovingAverageYawRate =
        fAlpha * *pfYawRate + (1 - fAlpha) * fMovingAverageYawRate;

    /* Calculate the DMMR for yaw rate */
    *pfDMMRYaw = SICalcDMMRYaw(*pfYawRate);

    /* Calculate the DMMR for distance to road border */
    if (bDistValid != FALSE) {
        *pfDMMRDist = SICalcCUMSUMDistance(*pfDistToRoad);
    } else {
        *pfDMMRDist = 0.0f;
    }
}

/* ***********************************************************************
  @fn             SIGetLaneChangeStateKinem                           */ /*!

    @brief          Returns the lane change state counter

    @description    Returns the lane change state counter

    @return         None

  ****************************************************************************
  */
float32 SIGetLaneChangeStateKinem(void) { return fLCDetectState; }

/* ***********************************************************************
  @fn             SIResetLaneChangeState                           */ /*!

       @brief          Sets the lane change state counter

       @description    Sets the lane change state counter

       @return         None

     ****************************************************************************
     */
void SIResetLaneChangeStateKinem(void) { fLCDetectState = 0.0f; }

/* ***********************************************************************
  @fn             SICalcCUMSUMDistance                           */ /*!

         @brief          Calculate the differential minimum maximum rate for
       distance to road border

         @description     Calculate the differential minimum maximum rate for
       distance to road border.

         @param          fCurrentVal : Input signal of current cycle

         @return         fSumVal : differential minimum maximum rate for
       distance to road border.

       ****************************************************************************
       */
static float32 SICalcCUMSUMDistance(const float32 fCurrentVal) {
    float32 fRetVal, fDiffVal;
    float32 fSumVal;
    uint8 i, uiCurrentPos;

    /* Calculate the difference between previous and current sample */
    fDiffVal = SILaneChangeCUMSUMDist.fLastValue - fCurrentVal;

    /* Save current difference value */
    uiCurrentPos =
        SILaneChangeCUMSUMDist.uiRunIndex % SI_LC_CUMSUM_DIST_MAX_SAMPLES;
    SILaneChangeCUMSUMDist.fDMMRSamples[uiCurrentPos] = fDiffVal;

    /* Increment counter and prevent overflow */
    if ((uiCurrentPos == 0) && (SILaneChangeCUMSUMDist.uiRunIndex > 0)) {
        SILaneChangeCUMSUMDist.uiRunIndex = 0;
    } else {
        SILaneChangeCUMSUMDist.uiRunIndex++;
    }

    /* Save current value for the next cycle */
    SILaneChangeCUMSUMDist.fLastValue = fCurrentVal;

    /* Init sum value */
    fSumVal = 0.0f;
    for (i = 0; i < SI_LC_CUMSUM_DIST_MAX_SAMPLES; i++) {
        fSumVal += SILaneChangeCUMSUMDist.fDMMRSamples[i];
    }

    /* Sanity check for summation value. If value is too large, it is likely
       that the road-border
       just changed to a new value or is not available. A summation value of
       zero results in a
       higher probability for lane follow. */
    if (fABS(fSumVal) > SI_LC_CUMSUM_DIST_MAX_SUM) {
        fSumVal = 0.0f;
    }

    fRetVal = fSumVal;

    return fRetVal;
}

/* ***********************************************************************
  @fn             SICalcDMMRYaw                           */ /*!

                @brief          Calculate the differential minimum maximum rate
              for yaw rate.

                @description     Calculate the differential minimum maximum rate
              for yaw rate.

                @param          fLastVal : Input signal of previous cycle

                @param          fCurrentVal : Input signal of current cycle

                @return        (fMaxVal - fMinVal) : differential minimum
              maximum rate for yaw rate.

              ****************************************************************************
              */
static float32 SICalcDMMRYaw(const float32 fCurrentVal) {
    float32 fRetVal, fDiffVal;
    float32 fMinVal, fMaxVal;
    uint8 i, uiCurrentPos;

    /* Calculate the difference between previous and current sample */
    fDiffVal = SILaneChangeDMMRYaw.fLastValue - fCurrentVal;

    /* Save current difference value */
    uiCurrentPos = SILaneChangeDMMRYaw.uiRunIndex % SI_LC_DMMR_YAW_MAX_SAMPLES;
    SILaneChangeDMMRYaw.fDMMRSamples[uiCurrentPos] = fDiffVal;

    /* Increment counter and prevent overflow*/
    if ((uiCurrentPos == 0) && (SILaneChangeDMMRYaw.uiRunIndex > 0)) {
        SILaneChangeDMMRYaw.uiRunIndex = 0;
    } else {
        SILaneChangeDMMRYaw.uiRunIndex++;
    }

    /* Save current value for the next cycle */
    SILaneChangeDMMRYaw.fLastValue = fCurrentVal;

    /* Init min and max value */
    fMinVal = SI_DMMRYAW_MINVAL;
    fMaxVal = 0.0f;

    /* Find min and max value within the last SI_LC_DMMR_MAX_SAMPLES */
    for (i = 0; i < SI_LC_DMMR_YAW_MAX_SAMPLES; i++) {
        fMinVal = MIN(fMinVal, SILaneChangeDMMRYaw.fDMMRSamples[i]);
        fMaxVal = MAX(fMaxVal, SILaneChangeDMMRYaw.fDMMRSamples[i]);
    }

    fRetVal = fMaxVal - fMinVal;

    return fRetVal;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
