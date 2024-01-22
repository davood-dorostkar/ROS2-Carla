/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "si_lanechange.h"
#include "stddef.h"
#include "TM_Global_Types.h"

/*****************************************************************************
  MODULEGLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULGLOBAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
SILCProbDataGlobal_t SILCProbDataGlobal;

SI_LC_t_LaneChange t_SILaneChange;
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
static const GDBLFunction_t SI_LC_KINEM_TIENV_PROB = {

    0.0f, 40.0f, (40.0f) / (100.0f - 70.0f),
    -(((40.0f) / (100.0f - 70.0f)) * 70.0f)};

/*****************************************************************************
  MODULE LOCAL CONSTANTS
*****************************************************************************/
/*! Prefactor for Gaussian distribution SQRT_(2.0f * CML_F_pi) */
#define SI_LC_GAUSS_PREFAC (2.50662827f)

/*! Maximum time to determine the lane change state */
#define SI_LC_MAX_TIME_STATE (0.5f)

/*! Minimum time for high time-gap-probability */
#define SI_LC_MIN_TIME_TIMEGAP (5.0f)

/*! Minimum lane change time gap probability based on the input signals
    turn signal indicator and lane matrix */
#define SI_LC_MIN_TI_TIMEGAP_PROB (60)

/*! Minimum speed for which the lane change probability based on turn signal
    indicator and environment is calculated */
#define SI_LC_MIN_SPEED_TIENV (60.0f / C_KMH_MS)
/*****************************************************************************
  MODULE LOCAL TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static SI_LC_t_TurnIndicatorFilter t_FiltTurnInd;
static float32 f_LaneChangeStateTimer;
static boolean b_LaneChangeStateFixed;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

static void SI_LC_v_FilterTurnIndicator(void);
static void SISetOutputValues(const SI_LC_t_LaneChangeState t_LCState);
static void SIDetectLaneChangeTIEnv(void);
static void SIDetermineTimeGapLaneChange(void);
static SI_LC_t_LaneChangeState SISetLaneChangeState(void);
eTurnIndicator_t SI_LC_t_GetFilterTurnIndState(void);

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* ***********************************************************************
  @fn             SI_LC_t_GetFilterTurnIndState                           */ /*!

@brief          Returns the filtered turn indicator signal

@description    Returns the filtered turn indicator signal.

@return         eTurnIndicator_t

@pre            None

**************************************************************************** */
eTurnIndicator_t SI_LC_t_GetFilterTurnIndState(void) {
    return t_FiltTurnInd.t_StateTurnInd;
}

/* ***********************************************************************
  @fn             SI_LC_v_FilterTurnIndicator                           */ /*!

  @brief          Filter the turn indicator signal

  @description    Filter the turn indicator signal. Prolongs the signal
                  for a given time.

  @param          -

  @return         None

  @pre            None

**************************************************************************** */
static void SI_LC_v_FilterTurnIndicator(void) {
    /* eTurnIndicator_Right=2,eTurnIndicator_Invalid=4,eTurnIndicator_Left=1,eTurnIndicator_Off=0,eTurnIndicator_Both=3,
     */
    if ((VLCSEN_pCustomInput->eTurnIndicator == eTurnIndicator_Left) ||
        (VLCSEN_pCustomInput->eTurnIndicator == eTurnIndicator_Right)) {
        t_FiltTurnInd.f_TimerFiltTurnInd = SI_LC_MIN_TIME_TIMEGAP;
        t_FiltTurnInd.t_StateTurnInd = VLCSEN_pCustomInput->eTurnIndicator;
    } else {
        if (t_FiltTurnInd.f_TimerFiltTurnInd > BML_f_Delta) {
            t_FiltTurnInd.f_TimerFiltTurnInd -= SI_CYCLE_TIME;
        } else {
            t_FiltTurnInd.t_StateTurnInd = eTurnIndicator_Off;
        }
    }
}

/* ***********************************************************************
  @fn             SIInitLaneChange                           */ /*!

             @brief          Initialize lane change detection module

             @description    Function initializes lane change detection module

             @return         None

             @pre            None

           ****************************************************************************
           */
void SIInitLaneChange(void) {
    t_FiltTurnInd.f_TimerFiltTurnInd = 0.f;
    t_FiltTurnInd.t_StateTurnInd = eTurnIndicator_Off;

    t_SILaneChange.f_LCProb = 0.f;
    t_SILaneChange.t_LCState = LC_FOLLOW;
    t_SILaneChange.t_LCPhase = LC_DEFAULT;
    t_SILaneChange.t_TimeGap.f_LCPhaseProb = 0.0f;
    t_SILaneChange.t_TimeGap.t_LCPhaseState = LC_FOLLOW;

    t_SILaneChange.t_Release.f_LCPhaseProb = 0.0f;
    t_SILaneChange.t_Release.t_LCPhaseState = LC_FOLLOW;
    t_SILaneChange.t_Release.t_LCTrafficOrientation = LC_TRAFFIC_ORIENT_UNKNOWN;

    f_LaneChangeStateTimer = 0.0f;
    b_LaneChangeStateFixed = FALSE;

    /* Initialize camera lane change observer */
    SIInitLaneChangeCam();

    /* Initialize kinematic lane change observer */
    SIInitLaneChangeKinem();
}

/* ***********************************************************************
  @fn             SIDetectLaneChange                           */ /*!

           @brief          Detect ego lane change

           @description    Function combines the probabilities of each observer
         to generate
                           a combined lane change probability.

           @return         None

           @pre            None

           @post           Data updated for current cycle.

         ****************************************************************************
         */
void SIDetectLaneChange(void) {
    SI_LC_t_LaneChangeState t_LCState = LC_FOLLOW;

    /* Calculate filtered turn indicator signal */
    SI_LC_v_FilterTurnIndicator();

    /**********************************************************************
     *      Calculate lane change based on different observers            *
     **********************************************************************/

    /* Detect lane change (deprecated call for backward compatibility) */
    SIDetectLaneChangeCam();

    /* Detect lane change based on turn signal indicator and environment */
    SIDetectLaneChangeTIEnv();

    /* Detect lane change based on kinematic signals */
    SICalculateKinematicLCProb();

    /**********************************************************************
    *      Calculate additional lane change signals                       *
    **********************************************************************/

    /* Determine the lane change state (LC_LEFT, LC_RIGHT, LC_FOLLOW) */
    t_LCState = SISetLaneChangeState();

    /* Determine the lane change phase (TIMEGAP, RELEASE, STEERBACK) */
    /* todo  */

    /* Determine the time gap lane change event */
    SIDetermineTimeGapLaneChange();

    /**********************************************************************
     *      Prepare output signals                                        *
     **********************************************************************/

    /* Set output values */
    SISetOutputValues(t_LCState);
}

/* ***********************************************************************
  @fn             SISetLaneChangeState                           */ /*!

         @brief          Map probabilities to output variable

         @description    Map probabilities to output variable

         @param          -

         @pre            SIDetectLaneChange must have been done in the current
       cycle
                         (to update internal state)

         @post           None

         @return         t_LCState : updated lane change state based on,
       kinematic observer and turn signal indicator.

       ****************************************************************************
       */
static SI_LC_t_LaneChangeState SISetLaneChangeState(void) {
    float32 f_LaneChangeKinemCounter = 0.0f;
    SI_LC_t_LaneChangeState t_LCState;
    const eTurnIndicator_t t_FiltTurnIndState = SI_LC_t_GetFilterTurnIndState();

    /* Set the old value as default value */
    t_LCState = t_SILaneChange.t_LCState;

    /*************************************************************************
     *       Lane change state based on kinematic observer                   *
     *************************************************************************/
    if ((f_LaneChangeStateTimer >= SI_LC_MAX_TIME_STATE) &&
        (b_LaneChangeStateFixed == FALSE)) {
        /* Get the lane change state counter of the kinematic observer */
        f_LaneChangeKinemCounter = SIGetLaneChangeStateKinem();

        /* Determine the state of the lane change as a function of the lane
           change counter.
           If the counter is positive, we have a left lane change. Vice versa,
           if we
           the counter is negative, we have a right lane change. */
        if (f_LaneChangeKinemCounter > 0) {
            t_LCState = LC_LEFT;
        } else {
            t_LCState = LC_RIGHT;
        }

        /* Remember to fix the lane change state for this lane change event */
        b_LaneChangeStateFixed = TRUE;
    } else if (f_LaneChangeStateTimer < BML_f_Delta) {
        t_LCState = LC_FOLLOW;
        b_LaneChangeStateFixed = FALSE;
    } else {
        /* Lane change state timer is below SI_LC_MAX_TIME_STATE.
           Do not try to determine the lane change state during the early phase
           of a lane change! */
    }

    /*************************************************************************
     *       Lane change state based on turn signal indicator                *
     *************************************************************************/

    /* If we have a turn signal indicator, it will override the decision of the
       kinematic
       observer. Additionally, we stop looking. */
    if (t_FiltTurnIndState == eTurnIndicator_Left) {
        t_LCState = LC_LEFT;
        b_LaneChangeStateFixed = TRUE;
        f_LaneChangeStateTimer = SI_LC_MAX_TIME_STATE;
    }

    if (t_FiltTurnIndState == eTurnIndicator_Right) {
        t_LCState = LC_RIGHT;
        b_LaneChangeStateFixed = TRUE;
        f_LaneChangeStateTimer = SI_LC_MAX_TIME_STATE;
    }

    /* Increment lane change state timer */
    if (MAX(SILCProbDataGlobal.fLCKinematicProb,
            SILCProbDataGlobal.fLCTIEnvProb) > 0.1f) {
        /* Increment counter for the waiting period until threshold */
        if (f_LaneChangeStateTimer < SI_LC_MAX_TIME_STATE) {
            f_LaneChangeStateTimer += SI_CYCLE_TIME;
        }

    } else {
        /* Set everything to default values */
        f_LaneChangeStateTimer = 0.0f;
        b_LaneChangeStateFixed = FALSE;
        t_LCState = LC_FOLLOW;

        /* Reset the kinematic lane change state counter */
        SIResetLaneChangeStateKinem();
    }

    return t_LCState;
}

/* ***********************************************************************
  @fn             SISetOutputValues                           */ /*!

            @brief          Map probabilities to output variable

            @description    Map probabilities to output variable

            @param          t_LCSTate : State of the lane change

            @pre            SIDetectLaneChange must have been done in the
          current cycle
                            (to update internal state)

            @post           None

          ****************************************************************************
          */
static void SISetOutputValues(const SI_LC_t_LaneChangeState t_LCState) {
    t_SILaneChange.f_LCProb = SILCProbDataGlobal.fLCTimeGapProb;
    t_SILaneChange.t_LCState = t_LCState;
    t_SILaneChange.t_LCPhase = LC_TIMEGAP;

    t_SILaneChange.t_TimeGap.f_LCPhaseProb = SILCProbDataGlobal.fLCTimeGapProb;
    t_SILaneChange.t_TimeGap.t_LCPhaseState = t_LCState;

    t_SILaneChange.t_Release.f_LCPhaseProb =
        SILCProbDataGlobal.fLCKinematicProb;
    t_SILaneChange.t_Release.t_LCPhaseState = t_LCState;
}

/* ***********************************************************************
  @fn             SIDetectLaneChangeTIEnv                           */ /*!

      @brief          Detect lane change based on turn signal indicator and
    environment

      @description    Detect lane change based on turn signal indicator and
    environment.
                      A lane change is likely to happen if turn signal indicator
    is switched
                      on. However, if no adjacent lane is available, the lane
    change
                      probability should remain low.

      @param          -

      @return         TRUE if turn indicator is switched on and corresponding
    adjacent
                      lane is available

    ****************************************************************************
    */
static void SIDetectLaneChangeTIEnv(void) {
    float32 fLCProbTIEnv = 0.0f;
    const float32 fEgoSpeed = EGO_SPEED_X_OBJ_SYNC;
    const eTurnIndicator_t t_FiltTurnIndState = SI_LC_t_GetFilterTurnIndState();

    /* Initialize number of left/right lanes with default value */
    sint32 iNumberLanesLeft = -1;
    sint32 iNumberLanesRight = -1;

    iNumberLanesLeft = FIP_s_GetLMLeftNumLane();
    iNumberLanesRight = FIP_s_GetLMRightNumLane();

    if (fEgoSpeed > SI_LC_MIN_SPEED_TIENV) {
        if (t_FiltTurnIndState == eTurnIndicator_Left) {
            /* -> Only assign a probability if we are certain that the number of
               adjacent lanes is NOT zero.
               -> If we detect a high kinematic probability, most likely
               lanematrix is not correct.
               -> If we already assigned a probability for fLCProbTiEnv in the
               last cycle, then ignore lanematrix as well
               In case we don't know the number of lanes (-1) or more lanes
               are available, a lane change probability of at least
               SI_LC_MIN_TI_TIMEGAP_PROB is set, else it is zero.*/
            if ((iNumberLanesLeft != 0) ||
                (SILCProbDataGlobal.fLCKinematicProb > 75.0f) ||
                ((iNumberLanesLeft == 0) &&
                 (SILCProbDataGlobal.fLCTIEnvProb > BML_f_Delta)))

            {
                fLCProbTIEnv = SI_LC_MIN_TI_TIMEGAP_PROB;
            }
        }

        if (t_FiltTurnIndState == eTurnIndicator_Right) {
            if ((iNumberLanesRight != 0) ||
                (SILCProbDataGlobal.fLCKinematicProb > 75.0f) ||
                ((iNumberLanesLeft == 0) &&
                 (SILCProbDataGlobal.fLCTIEnvProb > BML_f_Delta))) {
                fLCProbTIEnv = SI_LC_MIN_TI_TIMEGAP_PROB;
            }
        }
    }

    SILCProbDataGlobal.fLCTIEnvProb = fLCProbTIEnv;
}

/* ***********************************************************************
  @fn             SIDetermineTimeGapLaneChange                           */ /*!

 @brief          Determine the time gap lane change event

 @description    Determine the time gap lane change event. Additional
information
                 can be used here to determine the final time gap lane change
                 probability

 @param          -

 @pre            SIDetectLaneChange must have been done in the current cycle
                 (to update internal state)

 @post           None

**************************************************************************** */
static void SIDetermineTimeGapLaneChange(void) {
    float32 f_AddProbKinem = 0.0f;

    /* An activated turn indicator results in a lane change probability of
      SI_LC_MIN_TI_TIMEGAP_PROB,
      which is below 100%, the difference will be added by the remaining
      observers. Only an
      active lane change will result in a time-gap-lane-change probability
      larger than SI_LC_MIN_TI_TIMEGAP_PROB. */
    f_AddProbKinem = dGDBmathLineareFunktion(
        &SI_LC_KINEM_TIENV_PROB, t_SILaneChange.t_Release.f_LCPhaseProb);

    /* Set temporary default values */
    SILCProbDataGlobal.fLCTimeGapProb =
        SILCProbDataGlobal.fLCTIEnvProb + f_AddProbKinem;
}

/* ***********************************************************************
  @fn             SIGetLaneChangeTimeGap                           */ /*!

       @brief          Get time gap lane change signals

       @description    Returns the relevant information for time gap lane change
     event

       @return         All relevent signals for time gap lane change event

       @pre            SIDetectLaneChange must have been done in the current
     cycle
                       (to update internal state)

       @post           None

     ****************************************************************************
     */
SI_LC_t_LaneChangePhaseInfo SIGetLaneChangeTimeGap(void) {
    SI_LC_t_LaneChangePhaseInfo t_LCInfoTimeGap;
    eTrafficOrientation_t t_TrafficOrientation;
    /* Set temporary default values */
    t_LCInfoTimeGap.f_LCPhaseProb = t_SILaneChange.t_TimeGap.f_LCPhaseProb;
    t_LCInfoTimeGap.t_LCPhaseState = t_SILaneChange.t_TimeGap.t_LCPhaseState;

    /* Feed forward of the traffic orientation. The headway control only reacts
       on
       a high lane change probability in combination with the correct traffic
       orientation.
       In case of right hand traffic, a change to from left to right lane should
       not
       result in a headway distance reduction. */
    t_TrafficOrientation =
        FIP_t_GetTrafficOrientation(); /* Get fused traffic orientation */

    switch (t_TrafficOrientation) {
        case GDB_TRAFFICORIENTATION_RIGHT_HAND:
            t_LCInfoTimeGap.t_LCTrafficOrientation = LC_TRAFFIC_ORIENT_RIGHT;
            break;
        case GDB_TRAFFICORIENTATION_LEFT_HAND:
            t_LCInfoTimeGap.t_LCTrafficOrientation = LC_TRAFFIC_ORIENT_LEFT;
            break;
        case TRAFFICORIENTATION_UNKNOWN:
        default:
            t_LCInfoTimeGap.t_LCTrafficOrientation = LC_TRAFFIC_ORIENT_UNKNOWN;
            break;
    }

    return t_LCInfoTimeGap;
}

/* ***********************************************************************
  @fn             SICalcGEVDProb                           */ /*!

               @brief          Calculate the generalized extreme value
             distribution for a given input

               @description    Calculate the generalized extreme value
             distribution for a given input

               @param          fVal : Input parameter

               @param          fKappa : Shape parameter

               @param          fSigma :  Scale parameter

               @param          fMu : Location parameter

             ****************************************************************************
             */
float32 SICalcGEVDProb(float32 fVal,
                       float32 fKappa,
                       float32 fSigma,
                       float32 fMu) {
    float32 fRetVal;
    float32 fAuxM, fAuxN;
    const float32 fSigmaInv = 1 / fSigma;
    const float32 fKappaInv = 1 / fKappa;

    /* Calculate first auxiliary variable */
    fAuxM = fABS((((fVal - fMu) * fSigmaInv) * fKappa) + 1.0f);

    /* Calculate second auxiliary variable */
    if (fAuxM > BML_f_AlmostZero) {
        fAuxN = -1.0f * fKappaInv * BML_f_fastlog(fAuxM);

        /* Calculate the value obtained by the gev-distribution */
        fRetVal = fSigmaInv * (1.f / (fAuxM)) * GDBexp(fAuxN - GDBexp(fAuxN));
    } else {
        fRetVal = 0.0f;
    }
    return fRetVal;
}

/* ***********************************************************************
  @fn             SICalcGDProb                           */ /*!

                 @brief          Calculate the normal distribution for a given
               input.

                 @description    Calculate the normal distribution for a given
               input.

                 @param          fVal : Input parameter

                 @param          fMean : Mean value

                 @param          fSigma :  Standard deviation

               ****************************************************************************
               */
float32 SICalcGDProb(float32 fVal, float32 fMean, float32 fSigma) {
    float32 fRetVal = 0.0f;
    float32 faux = 0.0f;

    if (fSigma > BML_f_AlmostZero) {
        /* Calculate the value obtained by the Gaussian distribution */
        faux = -0.5f * (SQR((fVal - fMean) / fSigma));
        faux = GDBexp(faux);
        fRetVal = faux / (SI_LC_GAUSS_PREFAC * fSigma);
    }

    return fRetVal;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */