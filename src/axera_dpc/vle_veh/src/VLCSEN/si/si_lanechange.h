

#ifndef _SI_LANECHANGE_H_INCLUDED
#define _SI_LANECHANGE_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "si.h"

#include "si_par.h"
/*****************************************************************************
  MODULEGLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULGLOBAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL MACROS
*****************************************************************************/

/*****************************************************************************
  MODULE LOCAL TYPES
*****************************************************************************/

typedef enum SI_LC_LaneChangePhaseEnum {
    LC_TIMEGAP,
    LC_RELEASE,
    LC_STEERBACK,
    LC_DEFAULT
} SI_LC_t_LaneChangePhaseEnum;

typedef struct SI_LC_LaneChange {
    float32 f_LCProb;
    SI_LC_t_LaneChangeState t_LCState;
    SI_LC_t_LaneChangePhaseEnum t_LCPhase;
    SI_LC_t_LaneChangePhaseInfo t_TimeGap;
    SI_LC_t_LaneChangePhaseInfo t_Release;
} SI_LC_t_LaneChange;

/*! States for turn light evaluation */
typedef enum SILCTurnLightPhase {
    SI_LC_TURN_LIGHT_ACTIVE, /*!< Lane change probable due to active turn light
                                */
    SI_LC_TURN_LIGHT_NOT_ACTIVE /*!< Lane change not probable due to inactive
                                   turn light */
} SILCTurnLightState_t;

/*! Parameter for calculation of the LC probability, which depend on the cycle
 * time */
typedef struct {
    float32 fEgoCurvePT1X; /*!< Filter coefficient for ego curve */
    GDBVector2_t DriverIntCurveFilterConst
        [SI_LC_PROB_POINTS_DRIVER_INT_CURVE_FILTER_TABLE]; /*!< Filter
                                                              coefficient for
                                                              driver intended
                                                              curve */
} SILaneChangeDynPar_t;

/*! LC probability evaluation based on turn light information */
typedef struct SILCProbTurnLightEval {
    SILCTurnLightState_t
        State; /*!< State of LC phase based on turn light information */
    float32 fTimeAfterTLActivation; /*!< Duration after the turn light was
                                       switched on the last time */
} SILCProbTurnLightEval_t;

/*! Global data for lane change probability calculation, which need to be known
 * beyond one cycle or which are used in other modules */
typedef struct {
    sint8
        iTLLeftPrevious; /*!< Turn light status of left turn light last cycle */
    sint8 iTLRightPrevious;    /*!< Turn light status of right turn light last
                                  cycle */
    sint16 iProbability;       /*!< Lane change probability [-100 100] ->
                                  probabilities for a LC to the left and right side
                                  are combined */
    sint16 iProbabilityBefore; /*!< Lane change probability in last cycle*/
    float32 fDistLat;     /*!< Distance driven in lateral direction during LC */
    float32 fVelLat;      /*!< Velocity in lateral direction during LC */
    float32 fCurveLevel;  /*!< "Mean" curve level: Should be zero on a straight
                             road and non-zero in a curve */
    float32 fLCPsiBefore; /*!< Last heading angle independent of cycle time*/
    float32 fLCPsiBeforeBuffer;      /*!< Heading angle in last cycle*/
    float32 fFilteredEgoCurve;       /*!< Filtered ego curve */
    float32 fEgoCurvePT1Previous;    /*!< Filtered ego curve last cycle */
    float32 fDrvIntCurvePT1Previous; /*!< Filtered driver intended curve last
                                        cycle */
    float32
        afLatDiffFilteredCurvesFilt[2]; /*!< Filtered difference in lateral
                                           distance between filtered ego curve
                                           and filtered driver intended curve */
    float32 afLatDiffCamCurveFilt[2];   /*!< Filtered difference in lateral
                                           distance between camera curve and
                                           filtered driver intended curve */
    float32 fLCKinematicProb; /*!< Lane change probability based on kinematic
                                 signals */
    float32 fLCTIEnvProb;     /*!< Lane change probability based on turn signal
                                 indicator and environment */
    float32 fLCTimeGapProb;   /*!< Probability for time-gap lane change */
    float32 fLCProbLeft;
    float32 fLCProbRight;
    float32 fLCProbFollow;
    SILCProbTurnLightEval_t LCTurnLightEvalLeft;  /*!< LC phase based on turn
                                                     light for evaluation of a LC
                                                     left */
    SILCProbTurnLightEval_t LCTurnLightEvalRight; /*!< LC phase based on turn
                                                     light for evaluation of a
                                                     LC right */
} SILCProbDataGlobal_t;

/*! Data for calculating the LC probability */
typedef struct {
    sint16
        iProbabilityLaneChangeLeft; /*!< Lane change probability for turning
                                       left [0 100] */
    sint16
        iProbabilityLaneChangeRight; /*!< Lane change probability for turning
                                        right [0 100] */
    SILaneChangeDynPar_t DynPar;     /*!< Filter constants which need to be
                                        calculated for each cycle (depend on cycle
                                        time) */
} SILCProbDataLocal_t;

/*! Enum to describe which lane change direction is considered */
typedef enum SILCProbDirection {
    LC_PROB_LEFT = 0, /*!< LC left */
    LC_PROB_RIGHT = 1 /*!< LC right */
} SILCProbDirection_t;

/*! Camera lane information for lane change probability calculation */
typedef struct SILCProbCamLaneEval {
    boolean bCamLaneVisible;    /*!< Indicates if camera lane visibility is high
                                   enough for using information */
    boolean bCamLaneChangeLeft; /*!< Indicates if a lane change to the left side
                                   was detected by the camera */
    boolean bCamLaneChangeRight; /*!< Indicates if a lane change to the right
                                    side was detected by the camera */
    float32 fCameraCurvature;    /*!< Curvature if the camera lane */
    float32 fLateralVelocityToCamLaneMarker; /*!< Lateral velocity of the ego
                                                vehicle based on the camera */
} SILCProbCamLaneInfo_t;

/*!< Input data to calculate the probability values */
typedef struct SILCProbInputData {
    float32 fEgoCurvePT1;    /*!< Filtered ego curve */
    float32 fDrvIntCurvePT1; /*!< Filtered driver intended curve */
    float32 fDistLat;        /*!< Driven distance in lateral direction */
    float32 fCurveLevel; /*!< "Mean" curve level: Should be zero on a straight
                            road and non-zero in a curve */
    SILCProbCamLaneInfo_t CamLaneInfo; /*!< Camera based infos */
} SILCProbInputData_t;

/*! Lane change probabilities for individual detectors */
typedef struct SILCProbDetectors {
    uint8
        iLatDiffFilteredCurvesProb; /*!< LC probability based on difference in
                                       lateral distance between filtered ego
                                       curve and filtered driver intended curve
                                       */
    uint8 iLatDiffCamCurveProb;     /*!< LC probability based on difference in
                                       lateral distance between camera curve and
                                       filtered driver intended curve */
    uint8 iVelLatCamLaneMarkerProb; /*!< LC probability based on lateral
                                       velocity of the ego vehicle towards
                                       camera lane markers */
    uint8 iTurnLightProb; /*!< LC probability based on turn light information */
    uint8
        iDistLatProb; /*!< LC probability based on driven distance in lateral
                         direction (only for low speed) */
    uint8
        iCamLaneMarkerCrossedProb; /*!< LC probability based on crossed camera
                                      lane marker */
    uint8 iCurveProb; /*!< LC probability based on curve (only for low speed) */
} SILCProbDetectors_t;

/*! Global data for lane change probability calculation, which need to be known
 * beyond one cycle */
extern MEMSEC_REF SILCProbDataGlobal_t SILCProbDataGlobal;

typedef struct SI_LC_TurnIndicatorFilter {
    float32 f_TimerFiltTurnInd;
    eTurnIndicator_t t_StateTurnInd;
} SI_LC_t_TurnIndicatorFilter;

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
/*----------------------------------------------------------------------------
  si_lanechange.c
----------------------------------------------------------------------------*/
extern float32 SICalcGEVDProb(float32 fVal,
                              float32 fKappa,
                              float32 fSigma,
                              float32 fMu);
extern float32 SICalcGDProb(float32 fVal, float32 fMean, float32 fSigma);
extern eTurnIndicator_t SI_LC_t_GetFilterTurnIndState(void);

/*----------------------------------------------------------------------------
  si_lanechange_cam.c
----------------------------------------------------------------------------*/
extern void SIInitLaneChangeCam(void);
extern void SIDetectLaneChangeCam(void);

/*----------------------------------------------------------------------------
  si_lanechange_kinem.c
----------------------------------------------------------------------------*/
extern void SIInitLaneChangeKinem(void);
extern void SICalculateKinematicLCProb(void);
extern float32 SIGetLaneChangeStateKinem(void);
extern void SIResetLaneChangeStateKinem(void);

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

#ifdef __cplusplus
};
#endif

#endif
