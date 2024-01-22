
#ifndef _OPS_EBA_QUALITY_PAR_H
#define _OPS_EBA_QUALITY_PAR_H

/* ***********************************************************************/ /*!
   Target Confirmation Update Parameters
**************************************************************************** */
/* Minimum target confirmation frequency (Ziel-Bestaetigungsdichte) for
 * general objects. If the object is confirmed by targets during this
 * cycle, the target confirmation frequency is equal or above this value.
 */
#define OPS_EBA_OBJ_MIN_TGT_CURR_CONFIRM_FREQ (128uL)

/* ***********************************************************************/ /*!
   Quality Calculation Mode
**************************************************************************** */
/* Enable/disable old OPS EBA quality calculation for backwards compatibility
 *
 * @remarks New, non carry-over projects shall use the new FPS calculation.
 * @attention If the new approach is used, the following steps are necessary
 * @li a new time filter function must be implemented
 * @li the conditional parameters must be adjusted
 */
// changed to witch on by guotao 20190418
#define OPS_EBA_USE_OLD_QUAL_CALC SWITCH_ON
// changed by guotao for VS compile problem 20190114
#if OPS_EBA_USE_OLD_QUAL_CALC
/*! Maximum CD object safety value */
#ifndef OPS_EBA_OBJ_MAX_SAFETY
#define OPS_EBA_OBJ_MAX_SAFETY (255u)
#endif
/* Object Safety Flags */
#ifndef OPS_EBA_OBJ_SAFETY_POE_BIT
#define OPS_EBA_OBJ_SAFETY_POE_BIT (128u)
#endif
#ifndef OPS_EBA_OBJ_SAFETY_RCS_BIT
#define OPS_EBA_OBJ_SAFETY_RCS_BIT (64u)
#endif
#ifndef OPS_EBA_OBJ_SAFETY_OBS_BIT
#define OPS_EBA_OBJ_SAFETY_OBS_BIT (32u)
#endif
#ifndef OPS_EBA_OBJ_SAFETY_LFT_BIT
#define OPS_EBA_OBJ_SAFETY_LFT_BIT (16u)
#endif
#define OPS_EBA_OBJ_SAFETY_TAS_BIT (8u)
/* Bit 0 .. 2 not used in ARS301
#define OPS_EBA_OBJ_SAFETY_BP_BIT                       (         4u)
#define OPS_EBA_OBJ_SAFETY_OGS_BIT                      (         2u)
#define OPS_EBA_OBJ_SAFETY_NR_BIT                       (         1u)
*/
#endif /* FPS_EBA_USE_OLD_QUAL_CALC */

/* ***********************************************************************/ /*!
   Object Quality
**************************************************************************** */
/* Minimum object quality for camera unconfirmed object */
#define OPS_EBA_UNCONFIRMED_OBJ_MIN_QUALITY (0.24f)
/* Minimum object quality */
#define OPS_EBA_OBJ_MIN_QUALITY (0.48f)
/* Grid gate confirmation for stationary objects */
#define OPS_EBA_HALF_GATE_GRID_CONFIRMATION (1.0F)
/* Invalid object distance. This distance is used as a marker for invalid
 * objects. */
#define OPS_EBA_INVALID_OBJ_DIST (2000.0f)

/* ***********************************************************************/ /*!
        Common Observer Parameters
**************************************************************************** */
/* Maximum stop-and-go velocity */
#define OPS_EBA_MAX_STOP_AND_GO_SPEED (36.0f / C_KMH_MS)
/* Distance to drop RCS Thresholds to minimum */
#define OPS_EBA_OBJ_MAX_LOW_DIST (10.0f)
/* Distance to drop RCS Thresholds to minimum */
#define OPS_EBA_OBJ_MAX_ULTRA_LOW_DIST (10.0f)
/* Maximum velocity of assumed slow object approach */
#define OPS_EBA_OBJ_MAX_SLOW_APPROACH_SPEED (10.0f)
/*! Minimum object probability */
#define OPS_EBA_OBJ_MIN_PROBABILITY (0.99999f)
/* Scan association of object in current cycle */
#define OPS_EBA_OBJ_CONFIRMED_BY_SCAN (128u)

/* Absolute Observers */
/* ***********************************************************************/ /*!
        Obstacle Observer (OBS) Parameters
**************************************************************************** */
/* Obstacle probability threshold for moving objects */
#define OPS_EBA_OBSPROB_THRES_MOVE (53u)
/* Obstacle probability threshold for stationary objects */
#define OPS_EBA_OBSPROB_THRES_STAT (60u)
/* Obstacle probability threshold decrement for stationary objects */
#define OPS_EBA_OBSPROB_THRES_DECR_STAT (10u)
/* Maximum RCS threshold for objects with moving to stationary transition */
#define OPS_EBA_MAX_RCS_THRES_MOVE2STAT (10.0f)

/* Fusion of obstacle and guardrail probability used by OBS-EBA */
#define OPS_STATOBST_GUARD_RAIL_TABLE \
    { 0u, 100u, 0u, 30u }

/* ***********************************************************************/ /*!
  Probability of Existence Observer (PoE) Parameters
**************************************************************************** */
/* Number of POE thresholds */
#define OPS_EBA_NUMBER_OF_POE_THRES (16uL)
/* POE thresholds for short Lifetime */
#define OPS_EBA_POE_THRES_OFFSET_00 (85U)
#define OPS_EBA_POE_THRES_OFFSET_01 (85U)
#define OPS_EBA_POE_THRES_OFFSET_02 (85U)
#define OPS_EBA_POE_THRES_OFFSET_03 (85U)
#define OPS_EBA_POE_THRES_OFFSET_04 (90U)
#define OPS_EBA_POE_THRES_OFFSET_05 (90U)
#define OPS_EBA_POE_THRES_OFFSET_06 (90U)
#define OPS_EBA_POE_THRES_OFFSET_07 (95U)
#define OPS_EBA_POE_THRES_OFFSET_08 (95U)
#define OPS_EBA_POE_THRES_OFFSET_09 (95U)
#define OPS_EBA_POE_THRES_OFFSET_10 (95U)
#define OPS_EBA_POE_THRES_OFFSET_11 (97U)
#define OPS_EBA_POE_THRES_OFFSET_12 (97U)
#define OPS_EBA_POE_THRES_OFFSET_13 (97U)
#define OPS_EBA_POE_THRES_OFFSET_14 (98U)
#define OPS_EBA_POE_THRES_OFFSET_15 (99U)
/* Default POE threshold */
#define OPS_EBA_DEFAULT_POE_THRES (99U)
/* Lower limit of PoE threshold for stopped objects */
#define OPS_EBA_STOPPED_POE_THRES_DECR (15U)

/* ***********************************************************************/ /*!
  Lifetime Observer (LFT) Parameters
**************************************************************************** */
/* Minimum lifetime for moving objects. Below that lifetime
 * the objects get a lower safety */
#define OPS_EBA_OBJ_MIN_LIFETIME_MOVE ((13u /*20u*/))
#define OPS_EBA_OBJ_MIN_LIFETIME_STAT ((10u /*16u*/))
#define OPS_EBA_OBJ_MIN_LIFETIME_SAFE ((6u /*10u*/))

/* ***********************************************************************/ /*!
        Radar Cross Section Observer (RCS) Parameters
**************************************************************************** */
/* Additional of RCS treshold for moving objects */
#define OPS_EBA_RCS_THRES_INCR_MOVE (5.0f)
/* Additional of RCS treshold for stationary objects */
#define OFPS_EBA_RCS_THRES_INCR_STAT (10.0f)
/* Maximum critical distance for checking stationary to move transitions */
#define OPS_EBA_CRIT_MAX_STAT2MOVE_DIST (25.0f)
/* Maximum critical object velocity for checking stationary to move transitions
 */
#define OPS_EBA_CRIT_MAX_STAT2MOVE_OBJ_SPEED (5.0f)
/* Minimum critical ego velocity for checking stationary to move transitions */
#define OPS_EBA_CRIT_MIN_STAT2MOVE_EGO_SPEED (25.0f / C_KMH_MS)
/* Maximum critical ego velocity for checking stationary to move transitions */
#define OPS_EBA_CRIT_MAX_STAT2MOVE_EGO_SPEED (60.0f / C_KMH_MS)
/* RCS treshold increment for stationary object that become moving */
#define OPS_EBA_CRIT_MAX_STAT2MOVE_RCS_THRES_INCR (10.0f)
/* Enable/disable requiring higher RCS tresholds for stationary objects
 * that become moving */
#define OPS_EBA_HIGHER_RCS_FOR_STAT2MOVE SWITCH_ON

/* ***********************************************************************/ /*!
  Oncoming Observer Parameters
**************************************************************************** */
/* Maximum lateral distance variance for objects to be considered safe */
#define OPS_EBA_LAT_VAR_FOR_REDUCE_ONCOMING_OBJ_QUAL (1.0f)

#define OPS_EBA_OBJ_NON_OBSERVER_ONCOMING_PROB (80U)

/* ***********************************************************************/ /*!
  Wide Observer Parameters
**************************************************************************** */

#define OPS_EBA_OBJ_NON_OBSERVER_WIDE_PROB (0U)

#endif /*_OPS_EBA_QUALITY_PAR_H */
