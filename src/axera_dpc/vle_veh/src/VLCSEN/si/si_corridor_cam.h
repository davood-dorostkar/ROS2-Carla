

#ifndef _SI_CORRIDOR_CAM_H_INCLUDED
#define _SI_CORRIDOR_CAM_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  MODULEGLOBAL CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MODULGLOBAL VARIABLES
*****************************************************************************/
/*! Global Lane Change State for trace bracket adaption */
extern MEMSEC_REF SIScaleBracketState_t SILaneChangeState;

/*****************************************************************************
  MODULE LOCAL SYMBOLIC CONSTANTS
*****************************************************************************/

/*! Constants */
#define SI_MIN_CAM_COURSE_DIST (7.0f)
#define SI_LC_PROB_LEVEL_LOW_SPEED (90)
#define SI_LC_PROB_LEVEL_HIGH_SPEED (83)
#define SI_LC_PROB_LEVEL_HYSTERESIS (10)
#define SI_CP_PAR_LC_MAX_MARKER_DIST (0.5f)
#define SI_LEVEL_LOW_SPEED (20.f)
#define SI_LANE_CHANGE_HISTORY_BUFFER (5u)
#define SI_LC_PROB_LEVEL_LOW (25)
#define SI_LEVEL_CURVE_DIFF_CAM_EGO (0.004f)
#define SI_LC_LEFT_DIFF_DIST_MARKER_HIST_LEVEL (0.01f)
#define SI_LC_RIGHT_DIFF_DIST_MARKER_HIST_LEVEL (-0.01f)
#define SI_TB_RATIO_MIN_CAM_LANE_WIDTH (1.5f)
#define SI_TB_RATIO_MAX_CAM_LANE_WIDTH (4.5f)
#define SI_MIN_DIST_MARKER_INLANE (0.9f)
#define SI_MIN_FACTOR_DIST_MARKER_INLANE (0.3f)
#define SI_PAR_INVALIDE_LANE_MARKER_DIST (-99.9f)
#define SI_FAC_LANEWIDTH_OBJECT_SELECTION (0.8f)
#define SI_TB_LANE_ASSO_CAM_COURSE_LEVEL (0.0002f)
#define SI_TB_LANE_ASSO_CAM_COURSE_EGO_COURSE_DIFF_LEVEL (0.002f)
#define SI_TB_ASSO_LAT_DIST_COMP_PARA_MULTI_LANE (0.7f)
#define SI_TB_ASSO_LAT_DIST_PARA_MULTI_LANE (0.8f)
#define SI_TB_ASSO_LAT_DIST_COMP_PARA_STRAIGHT_LANE (0.45f)
#define SI_TB_ASSO_LAT_DIST_PARA_STRAIGHT_LANE (0.6f)
#define SI_TB_ASSO_LC_LAT_VREL_PARA (-1.1f)
#define SI_TB_ASSO_LC_LONG_VREL_PARA (-7.f)
#define SI_TB_ASSO_LC_LAT_DIST_MIN_PARA (-2.f)
#define SI_TB_ASSO_LC_COURSE_LAT_DIST_MIN_PARA (-1.f)
#define SI_TB_ASSO_LC_LAT_DIST_MAX_AVLC_LANE_PARA (0.f)
#define SI_TB_ASSO_LC_LAT_DIST_MIN_AVLC_LANE_PARA (-2.f)
#define SI_TB_ASSO_LC_LONG_DIST_MAX_PARALL (70.f)
#define SI_TB_ASSO_POST_LC_TIME_RELEVANT_PARA_MIN (1.f)
#define SI_TB_ASSO_LC_TIME_RELEVANT_PARA_MAX (5.f)
#define SI_TB_ASSO_PRE_LC_MIN_TIME_RELEVANT_PARA (0.4f)
#define SI_TB_ASSO_PRE_LC_MIN_MIN_TIME_RELEVANT_PARA (0.2f)
#define SI_TB_ASSO_POST_LC_LAT_DIST_CAM_LANE_PARA (-1.5f)
#define SI_TB_ASSO_PRE_LC_LAT_DIST_CAM_LANE_PARA (0.f)
#define SI_TB_ASSO_PRE_LC_LAT_DIST_PARA (0.f)
#define SI_TB_ADAPT_FACTOR_LONG_DIST (70.f)
#define SI_TB_LEFT_LC_PROB_LEVEL_RESET_HIST (-30)
#define SI_TB_RIGHT_LC_PROB_LEVEL_RESET_HIST (30)
#define SI_TB_ABORT_BLINKER_FEATURE_LC_PROB_LEVEL (10)
#define SI_TB_INCREASE_FACTOR_BLINKER_FEATURE (0.65f)
#define SI_TB_DECREASE_FACTOR_BLINKER_FEATURE \
    (1.f - SI_TB_INCREASE_FACTOR_BLINKER_FEATURE)

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

typedef enum SIInLaneDecision {
    UNKNOWN_LANE,
    EGO_LANE,
    LEFT_LANE,
    RIGHT_LANE
} SIInLaneDecision_t; /*!< Enum for lane assignment (object relative to
                         ego-lane) */

/*****************************************************************************
  MODULE LOCAL VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

#ifdef __cplusplus
};
#endif

#endif /*_SI_CORRIDOR_CAM_H_INCLUDED*/
