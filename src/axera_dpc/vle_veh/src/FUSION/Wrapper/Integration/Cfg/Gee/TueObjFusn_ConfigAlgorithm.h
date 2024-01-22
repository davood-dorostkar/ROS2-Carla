/*
 * \file       TueObjFusn_ConfigAlgorithm.h
 *
 *
 *
 * This file provides a set of parameters to all fusion algorithms.
 * Note that boolean values for initialization and parameterization may
 * superimpose each other. So first of all for each variable choose the zero
 * initialization. Then add either diag(1,1,1,1) to make the matrix parameter
 * unitary or add other values for user-defined initialization. If, however,
 * too many properties are chosen to be STD_ON, some elements of the matrix may
 * end up being a sum of many different things which won't neccessarily make
 * sense if you're not careful.
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */
/** \addtogroup tuePrvlkf
 *  @{
 */
#ifndef TUEOBJFUSN_CONFIGALGORITHM_H
#define TUEOBJFUSN_CONFIGALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tue_prv_common_types.h"

/********************************************************************
 * Motion Type thresholds
 ********************************************************************/
/** \name motion type thresholds **/
/**
 * Define threshold at between stationary/ stopped and moving object
 */
#define TUEOBJFUSN_MOTIONTYPE_TO_MOVING (1.70f)
#define TUEOBJFUSN_MOTIONTYPE_TO_STATIONARY (1.38f)
#define TUEOBJFUSN_MOTIONTYPE_CROSSING_TO_DRIVING \
    (0.698f) /* Threshold for transition driving to crossing (40 degree)  */
#define TUEOBJFUSN_MOTIONTYPE_DRIVING_TO_CROSSING \
    (0.7853f) /* Threshold for transition crossing to driving (45 degree)  */
#define TUEOBJFUSN_MOTIONTYPE_ONCOMING_TO_CROSSING \
    (2.4434f) /* Threshold for transition crossing to driving (140 degree) */
#define TUEOBJFUSN_MOTIONTYPE_CROSSING_TO_ONCOMING \
    (2.5307f) /* Threshold for transition crossing to driving (145 degree) */

/** \}*/

/********************************************************************
 * ID Provider
 ********************************************************************/

/** first valid u16ID */
#define TUEOBJFUSN_IDPROVIDER_U16ID_MIN (0u)
/** maximum value for u16ID */
#define TUEOBJFUSN_IDPROVIDER_U16ID_MAX \
    (600u)  // change to 140 from 100 for EP30 requirement by guotao 20190509

/********************************************************************
 * Age
 ********************************************************************/

#define TUEOBJFUSN_MAX_AGE (65001u)

/********************************************************************
 * Object Selection Bins
 ********************************************************************/

/**
 * distance threshold for radar only objects
 */

#define TUE_PRV_OBJECTSELECTION_LONG_DISTANCE_ROI1_THRESHOLD_OBJ (70.0f)
#define TUE_PRV_OBJECTSELECTION_LONG_DISTANCE_ROI0_THRESHOLD_OBJ (200.0f)
#define TUE_PRV_OBJECTSELECTION_ANGLE_ROI2_THRESHOLD_OBJ (10.0f)
#define TUE_PRV_OBJECTSELECTION_ANGLE_ROI1_THRESHOLD_OBJ (22.5f)
#define TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE6 (50.0f)
#define TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE5 (100.0f)
#define TUE_PRV_OBJECTSELECTION_RANGE_THRESH_OBSZONE4 (150.0f)

/********************************************************************
 * Distance Score
 ********************************************************************/

/**
 * Adds a penalty term in case of different ids
 */
#define TUE_PRV_DISTANCE_SCORE_USE_ID_INFORMATION (STD_ON)

/**
 * Adds a penalty term in case of different class types
 */
#define TUE_PRV_DISTANCE_SCORE_USE_CLASS_INFORMATION (STD_ON)

/**
 * Adds a penalty term in case of low obstacle probability from radar
 */
#define TUE_PRV_DISTANCE_SCORE_USE_OBSTACLE_PROBABILITY (STD_OFF)

/**
 * Adds a penalty term in case of different class types
 */
#define TUE_PRV_DISTANCE_SCORE_USE_PENALTY_TERM_FOR_MAHALANOBIS (STD_ON)

/**
 * Number of states used in calculation of Mahalanobis distance
 */
#define TUE_PRV_DISTANCE_SCORE_MAHALANOBIS_STATE_SIZE (3u)

/** Max value to be added if the object to track class differs */
#define TUE_PRV_DISTANCE_SCORE_MAX_CLASS_PENALTY (FLT_ONE)

/** Constant value to be added to distance score if track was updated by a
   measurement with a different ID
    in previous cycles */
#define TUE_PRV_DISTANCE_SCORE_DIFFERENT_ID_PENALTY (0.84f)

/**
 * Maximal coarse distance for rectangular gating.
 * Unit: m
 */
#define TUE_PRV_DISTANCE_SCORE_MAX_COARSE_DIST_X (15.0f)
#define TUE_PRV_DISTANCE_SCORE_MAX_COARSE_DIST_Y (2.0f)
/**
 * Maximal coarse longitudinal velocity for gating.
 * Unit: m/s
 */
#define TUE_PRV_DISTANCE_SCORE_MAX_COARSE_VEL_X (15.0f)

/********************************************************************
 * Kalman Filter Parameters
 ********************************************************************/
/** \name Kalman Filter P-Matrix Parameters
 \{**/
/* Default Value for P-Matrix initialization if signal is not provided by sensor
 */
#define TUE_PRV_LKF_FILTER_P_DEFAULT_DIAG (FLT_ONE)
/** \}*/

/********************************************************************
 * Coordinated Turn Model
 ********************************************************************/
/** \name Coordinated Turn Model constant parameters
 \{**/
/** Min squared target speed over ground to update coordinated turn model */
#define TUE_PRV_COORDINATED_TURN_MIN_TARGET_SPEED (2.25f)
/** \}*/

/** \{**/
/** Variance for Q used in predictin step of heading */
#define TUE_PRV_COORDINATED_TURN_VARIANCE_IN_HEADING_FOR_Q (0.025f)
/** \}*/

/********************************************************************
 * Error Management
 ********************************************************************/
/** @name Error Management constant parameters
 @{**/
/** Min target speed over ground to update coordinated turn model */
#define TUE_PRV_ERROR_MANAGEMENT_MAX_ERROR_AGE (50u)
/** @}*/

/********************************************************************
 * Kinematics
 ********************************************************************/
/** @name Kinematics Config parameters
 \{**/
/** Enables and disables the covariances that are (potentially) sent by the
 * sensors. If set to STD_OFF all covariances are assumed to be zero */
#define TUE_PRV_KINEMATICS_USE_SENSOR_COVARIANCES (STD_OFF)

/** If set to STD_ON, the variance of the longitudinal velocity and acceleration
 * is linearly increased depending on ego acceleration */
#define TUE_PRV_KINEMATICS_USE_EGO_MOTION_FOR_VARIANCE (STD_ON)
/** \}*/

/********************************************************************
 * Coasting
 ********************************************************************/
/** @name Coasting algorithm parameters
\{**/
/** Enables / Disables coasting of internal tracks */
#define TUE_PRV_COASTING_ENABLE_COASTING (STD_ON)

/** Sets sensors that shall be coasted. Currently set to Fused */
#define TUE_PRV_COASTING_SENSOR_PATTERN (0x0402u)

/** Lifespan of coasted objects in fusion cycles */
#define TUE_PRV_COASTING_LIFESPAN (5u)

/** \}*/

/********************************************************************
 * Camerea Gain Compensation
 ********************************************************************/
/** @name Camera Gain algorithm parameters
\{**/
/** Enables / Disables calculate of camera gain */
#define TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION (STD_OFF)

/** \}*/

/********************************************************************
 * Track Merge
 ********************************************************************/
/** @name Track Merge algorithm parameters
\{**/
/** Enables / Disables track merge algorithm*/
#define TUE_PRV_TRACK_MERGE_ENABLE_TRACK_MERGE (STD_ON)

/** \}*/

/********************************************************************
 * Object Selection
 ********************************************************************/
/** @name Object Selection
\{**/
/** Enables / Disables down selection to TUEOBJFUSN_MPF_OBJLISTINPUT_MAX_OBJECTS
 */
#define TUE_PRV_OBJECT_SELCTION_ENABLE_OBJECT_SELECTION_FOR_OUTPUT (STD_ON)

/** Resulting maximum number of output objects in case object selection is
 * active */
#define TUE_PRV_OBJECT_SELECTION_NUMBER_OF_OUTPUT_OBJECTS (100u)

/** \}*/

/********************************************************************
 * Clear vision information after xx cycles
 **
 ********************************************************************/
/** @name Clear Vision Information if trackable is not updated by vision for
multiple cycles
\{**/
/** If enabled, radar motion types are used depending on the motion type of the
 * previous cycle */
#define TUE_PRV_LKF_TRACK_MANAGEMENT_NUM_CYCLES_TO_CLEAR_VISION (10u)
/** \}*/

/********************************************************************
 * Clear radar information after xx cycles
 **
 ********************************************************************/
/** @name Clear radar Information if trackable is not updated by radar for
multiple cycles
\{**/
#define TUE_PRV_LKF_TRACK_MANAGEMENT_NUM_CYCLES_TO_CLEAR_RADAR (10u)
/** \}*/

/********************************************
 * Set Bit pattern for ASIL relevant checks  *
 *********************************************/
/** @name Sets Bit pattern for ASIL relevant checks
\{**/

#define TUEOBJLIST_MVS_BITPATTERN_ASIL (0xF0u)
#define TUEOBJLIST_RAD_BITPATTERN_ASIL (0x0Fu)

/***********************************************************
 * Set threshold for association using obstacle probability *
 ************************************************************/
/** \name Set threshold for association using obstacle probability
\{**/

#define DIST_SCORE_LOW_OBSTACLE_PENALTY (30.f)
#define TUE_PRV_DISTANCE_SCORE_LOW_OBSTACLE_VEH_THRESHOLD (45.f)
#define TUE_PRV_DISTANCE_SCORE_LOW_OBSTACLE_PED_THRESHOLD (25.f)

/***********************************************************
 * LKF Track Management AAU                                 *
 ************************************************************/
/** \name Algorithm Parameter used in LKF Track Management AAU */
/** \{ */
#define TUE_PRV_LKF_TRACK_MANAGEMENT_ADDITIONAL_Q_ON_DIAGONAL (0.02f)

#ifdef __cplusplus
}
#endif

#endif /* TUEOBJFUSN_CONFIGALGORITHM_H */
