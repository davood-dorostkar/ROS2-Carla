
#ifndef _OPS_PAR_H
#define _OPS_PAR_H
/*****************************************************************************
  INCLUDE
*****************************************************************************/

#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
#include "envm_consts.h"
#include "ops_cfg.h"
#include "TM_Global_Types.h"
#include "stddef.h"

/*****************************************************************************
  VARIABLES DECLARTIONS
*****************************************************************************/

/*! Min probability of existence to activate crossing hypcat */
#define OPS_EBA_HYPCAT_CROSS_MIN_POE_QUAL (0.9f)
/*! Suppress stopped crossing objects for vehicle hypcat  */
#define OPS_EBA_HYPCAT_VCL_SUPPRESS_CROSSING_STOPPED_OBJECTS 1
/*! Suppress stopped crossing objects for stationary hypcat  */
#define OPS_EBA_HYPCAT_STAT_SUPPRESS_CROSSING_STOPPED_OBJECTS 1

/* Schwellen fuer Radarquerschnitt (RCS) in dBsm; Ziele bzw. neu
 * relevantwerdende Obfekte muessen   |*/
/* abhaengig vom Zieltyp wenigstens diesen Wert haben -- sonst werden sie
 * verworfen;                |*/
/*die Schwellen liegen im Nahbereich tiefer, da dort oft geringere RCS-Werte auftreten */ /*|*/
/*|*/
/* 1) RCS-Offset in dBsm fuer Aufnahme stehender Objekte Rangegate abhängig */ /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_00 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_01 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_02 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_03 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_04 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_05 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_06 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_07 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_08 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_09 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_10 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_11 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_12 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_13 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_14 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_15 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_16 9.8f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_17 9.6f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_18 9.4f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_19 9.2f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_20 9.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_21 8.8f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_22 8.6f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_23 8.4f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_24 8.2f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_25 8.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_26 7.8f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_27 7.6f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_28 7.4f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_29 7.2f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_30 7.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_31 6.8f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_32 6.6f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_33 6.4f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_34 6.2f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_35 6.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_36 5.8f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_37 5.6f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_38 5.4f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_39 5.2f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_40 5.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_41 5.4f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_42 5.6f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_43 5.8f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_44 6.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_45 6.2f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_46 6.4f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_47 6.6f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_48 6.8f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_49 7.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_50 7.2f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_51 7.4f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_52 7.6f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_53 7.8f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_54 8.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_55 8.5f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_56 9.0f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_57 9.5f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_58 9.5f            /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_59 10.0f           /*|*/
#define SI_RCSAdd_STATIONARY_OFFSET_60 10.0f           /*|*/
                                                       /*|*/
/*! 2. RCS Offset für die Aufnahme fahrender Objekte*/ /*|*/
#define SI_RCSAdd_MOVING (5.0f)                        /*|*/
                                                       /*|*/
/*! 4. Laenge eines Rangegates in m */                 /*|*/
#define SI_RG_LENGTH (1.0f)                            /*|*/
                                                       /*|*/
/*! Anzahl der Rangegates:*/                           /*|*/
/*! Achtung: dises Groesse muss gleich sein wie SYS_NOF_RG in SysGlob.h; die symbolische    */ /*|*/
/*! Konstante SYS_NOF_RG kann hier nicht benutzt werden, weil im MCU ein unterschiedliches/ */ /*|*/
/*! SysGlob.h eingebunden wird                                                              */ /*|*/
#define SI_NOF_RG 61L                                      /*|*/
                                                           /*|*/
/*-- RCS-Schwelle --*/                                     /*|*/
#define RCS_NO_TARGET (GDB_SUBS_VAL_FOR_LOG_OF_ZERO + 1.F) /*|*/
#define RCS_LIFETIME_MIN ((100u))                          /*|*/
#define RCS_RANGE_LIFETIME (40.F)                          /*|*/
                                                           /*|*/
                                                           /*|*/
/* Definitions for getting RCS value of object */          /*|*/
#define RCS_OBJLEBDAUER_MIN (2.64f)                        /* 40 cycles */
#define RCS_OBJLEBDAUER_MAX (3.30f)                        /* 50 cycles */
/*-------------------- Definition of RCS values for RCS criteria
 * ------------------------------------*/

/*--------------- Definitions for horizontal offset
 * criteria----------------------------------------|*/

/*--------------- Definitions for horizontal offset
 * criteria----------------------------------------|*/

/* use POE suppression for non relevant objects with probability of existence < threshold */ /*|*/
#define FPS_ACC_POE_MOVING_DROP_THRESHOLD (0.80F)
#define FPS_ACC_POE_MOVING_PICKUP_THRESHOLD (0.98F)
#define FPS_ACC_POE_STAT_DROP_THRESHOLD (0.80F)
#define FPS_ACC_POE_STAT_PICKUP_THRESHOLD (0.98F)

/* use measured targets frequency value for suppression of non relevant
 * objects*/

/* measured targets frequqncy value resides in TPObject[i].OA.uZielBestaetDichte
 */
/* the 8 last measurement cycles are documented in this value */
/* a one in a bit position means target was measured, a 0 menas target was not
 * measured */
/* e.g. the binary value 1000 0000 = 128d means only in last cycle target was
 * measured */

#define FPS_ACC_MTF_MOVING_PICKUP_THRESHOLD                                 \
    (216u) /* Last 3 targets measured or last 2 and the two before that and \
              high target density */
#define FPS_ACC_MTF_MOVING_DROP_THRESHOLD (0u)
#define FPS_ACC_MTF_STAT_PICKUP_THRESHOLD                                   \
    (216u) /* Last 3 targets measured or last 2 and the two before that and \
              high target density */
#define FPS_ACC_MTF_STAT_DROP_THRESHOLD \
    (32u) /* 0 target in the last 3 cycles */
#define FPS_ACC_MTF_STAT_DROP_NEAR_THRESHOLD (0u)
#define FPS_ACC_MTF_STAT_DROP_NEAR_DIST                                       \
    (5.0f) /* Under this distance the FPS_ACC_MTF_STAT_DROP_NEAR_THRESHOLD is \
              used */

/* Do not check MTF value for stationary objects for distances bigger than
 * ....*/

#define FPS_ACC_MTD_MOVING_PICKUP_THRESHOLD (0L)
#define FPS_ACC_MTD_MOVING_DROP_THRESHOLD (-36L)
#define FPS_ACC_MTD_STAT_PICKUP_THRESHOLD (0L)
#define FPS_ACC_MTD_STAT_DROP_THRESHOLD (-36L)

#define FPS_ACC_POBS_MOVING_DROP_THRESHOLD (51u)
#define FPS_ACC_POBS_MOVING_PICKUP_THRESHOLD (53u)

#define FPS_ACC_POBS_MOVING_DROP_THRESHOLD_TUNNEL \
    (76u) /* moving OBSTACLE drop in Tunnel */
#define FPS_ACC_POBS_MOVING_PICKUP_THRESHOLD_TUNNEL \
    (78u) /* moving OBSTACLE pickup in Tunnel*/

#if (ALGO_SensorType == ALGO_CFG_CR400Entry)
#define FPS_ACC_POBS_STAT_DROP_THRESHOLD (53u)
#define FPS_ACC_POBS_STAT_PICKUP_THRESHOLD (60u)
#else
#define FPS_ACC_POBS_STAT_DROP_THRESHOLD (54u)
#define FPS_ACC_POBS_STAT_PICKUP_THRESHOLD (66u)
#endif
#define FPS_LTIME_MOVING_DROP_THRESHOLD (0u)
#define FPS_LTIME_MOVING_PICKUP_THRESHOLD (10u)
#define FPS_LTIME_STAT_DROP_THRESHOLD (0u)
#define FPS_LTIME_STAT_PICKUP_THRESHOLD (10u)

/*! Grid gate confirmation parameters */
#define FPS_ACC_HALF_GATE_GRID_CONFIRMATION (1.0F)

/*! Minimum Speed to recognize a Parking process */
#define SI_STAT_MIN_PARKING_SPEED (3.0f / C_KMH_MS)
/*! Maximum curvature to allow selection of stationary objects. It excludes a
 * classification during Parking */
#define SI_STAT_MAX_PARKING_CURVATURE 0.033f /* R=30m */
/*! Maximum curvature to allow selection of stationary objects */
#define SI_STAT_MAX_CURVATURE 0.005f /* R=200m */
/*! Maximum curvature gradient to allow selection of stationary objects */
#define SI_STAT_MAX_CURV_GRADIENT 0.003f
/*! Maximum curvature gradient to keep selection of relevant stationary objects
 */
#define SI_STAT_MAX_CURV_GRADIENT_REL 0.008f
/*! By dynamic Situations only allow the selection of stationary objects closer
 * than SI_STAT_DYNA_MIN_RANGE */
#define SI_STAT_DYNA_MIN_RANGE 30.0f

/*--------------- Definitions for Low speed and near range stationary pedestrian
 * detection---------------------*/

/*----------------------------- pedestrian supression
 * ---------------------------------------------------------*/
/* minimum pedestrian probability value for point objects to be a possible
 * pedestrian objects to wait for */
#define SI_WAIT_FOR_PED_DECISION_PROB (0.20f)
/* number of cycles to wait for a point object to get a pedestrian object */
#define SI_MAX_WAIT_CYCLES_FOR_PED (3u)
/* max. speed to select pedestrians as OOI */
#define SI_MAX_SPEED_PED_SELECTION (30.0F / C_KMH_MS)
/* max. distance to select pedestrians as OOI */
#define SI_MAX_DIST_PED_SELECTION (30.0f)

/*----------------------------- mirror supression
 * -------------------------------------------------------------*/
/* time after that a relevant object can not be supressed because of Mirror
 * recognition */
#define FPS_ACC_RELEVANT_MIRROR_TIME (1.0f)
/* mirror recognition influences POE --> to keep relevant objects although
mirror recognition DROP value for POE
must be forced down (moving and stationary objects) */
#define FPS_ACC_POE_MOVING_MIRROR_DROP_THRESHOLD (0.7f - C_F32_DELTA)

#if (FPS_CFG_SUPPRESS_MOV_ON_BRIDGE)
#define FPS_MOV_ON_BRIDGE_VEGO_MIN                                        \
    (80.F / C_KMH_MS) /*<! minimum ego velocity for suppression of moving \
                         objects on bridge    */
#define FPS_MOV_ON_BRIDGE_LT_MAX                                             \
    (100u) /*<! maximum object lifetime for suppression of moving objects on \
              bridge */
#define FPS_MOV_ON_BRIDGE_CURVE_MAX                                      \
    (0.004F) /*<! maximum ego curve for suppression of moving objects on \
                bridge       */
#define FPS_MOV_ON_BRIDGE_YAWRATE_MAX                                      \
    (0.08F) /*<! maximum ego yaw rate for suppression of moving objects on \
               bridge    */

#define FPS_MOV_ON_BRIDGE_ANGLE_DELTA_ORTHO                                  \
    (25.f) /*<! lower orientation delta boundary for objects with orthogonal \
              orientation e.g. min 25 degree */
#define FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_ORTHO                                \
    (                                                                       \
        2.6F) /*<! minimum absolute lateral vrel for suppression of objects \
                 with orthogonal orientation */

#define FPS_MOV_ON_BRIDGE_ANGLE_DELTA_PARALL                               \
    (10.f) /*<! lower orientation delta boundary for objects with parallel \
              orientation e.g. min 10 degree  */
#define FPS_MOV_ON_BRIDGE_LAT_VREL_ABS_PARALL                                \
    (5.F) /*<! minimum absolute lateral vrel for suppression of objects with \
             parallel orientation */

/* Defines for moving object search on bridge */
#define FPS_MOV_ON_BRIDGE_DISTX_GATE \
    (3.0F) /*<! Max x distance between the objects */
#define FPS_MOV_ON_BRIDGE_DISTY_GATE \
    (12.0F) /*<! Max y distance between the objects */
#define FPS_MOV_ON_BRIDGE_VRELX_GATE \
    (2.0F) /*<! Max x velocity gap between the objects */
#define FPS_MOV_ON_BRIDGE_VRELY_GATE_LOW \
    (0.1F) /*<! Min y velocity if bridge is detected */
#define FPS_MOV_ON_BRIDGE_VRELY_GATE_HIGH \
    (0.3F) /*<! Min y velocity if no bridge is detected */
#define FPS_MOV_ON_BRIDGE_LIFETIME (5.0F)   /*<! Max object lifetime */
#define FPS_MOV_ON_BRIDGE_DISTX_MIN (40.0F) /*<! Min object x distance */
#define FPS_MOV_ON_BRIDGE_VABS_MAX (3.0F)   /*<! Max overground velocity */
#endif

#if FPS_CFG_SUPPRESS_VRELAMBIG_ON_GUARDRAIL
#define FPS_VRELAMBIG_MAX_DISTX (30.0f)     /*<! meter */
#define FPS_VRELAMBIG_VREL_THRESHOLD (3.0f) /*<! meter/sec */
#define FPS_VRELAMBIG_ROAD_EST_MIN_TRACKSTAT                           \
    (                                                                  \
        5u) /*<! Minimal tracking status for the Road Estimation to be \
               considered */
#define FPS_VRELAMBIG_ROAD_EST_MIN_CONF \
    (60u) /*<! Minimal confidence for the Road Estimation to be considered */
#define FPS_VRELAMBIG_ROADEST_LATERALGATE                                  \
    (0.75f) /*<! Maximal distance between lateral object position and road \
               estimation clothoid */
#endif

/*--------------------------------------- grid confirmation parameters
 * ---------------------------------------*/

/*! After this delay, a relevant stationary object can not loose its relevance
 * because of its Obstacle proability anymore */
#define FPS_ACC_STAT_SECURE_RELEVANT_TIME (1.2f) /* [s] */

/* after object is relevant that time, shaddow flag of OT is not respected any
 * more for preselection*/
#define FPS_ACC_RELEVANT_SHADOW_TIME (1.0F)

/*--------------- Definitions for Low speed and near range stationary pedestrian
 * detection---------------------*/
#if (CFG_LOW_SPEED_STAT_PEDESTRIAN)
/* max. speed [km/h] to select stationary pedestrians as OOI */
#define FPS_MAX_SPEED_STAT_PED_SELECTION (5.0f / C_KMH_MS)
/* max. distance [m] to select stationary pedestrians as OOI */
#define FPS_MAX_DIST_STAT_PED_SELECTION 5.0f
/* min Lifetime [cycles] to select stationary pedestrians as OOI */
#define FPS_MIN_LTIME_STAT_PED_SELECTION (5u)
#endif

#if (CFG_CAMERA_RADAR_FUSION)
/*! The camera confirmation gate used for checking camera object quality */
#define FPS_PAR_ACC_CAM_CONF_QUALITY_GATE 30u
/*! The camera confirmation gate used for OOI objects */
#define FPS_PAR_ACC_CAM_CONF_QUALITY_GATE_OOI 10u

#if (FPS_CFG_CAMERA_CONF_HOLD_WORKAROUND)
#define FPS_PAR_MIN_DISTX_CAM_CONF_HOLD 25.f
#endif
#if (FPS_CFG_SUPPRESS_STAT_OUT_CORRIDOR)
/*! Max. lateral distance for a stationary objects without cam-confirmation */
#define FPS_PAR_ACC_NO_CAM_CONF_STAT_MAX_LAT_DIST 0.95f
#define FPS_PAR_ACC_NO_CAM_CONF_STAT_HYST_LAT_DIST 1.2f
#endif
#endif

#if (((CFG_CAMERA_RADAR_FUSION) &&                 \
      (FPS_CFG_ONLY_SELECT_CAM_CONFIRMED_STAT)) || \
     (FPS_CFG_RADAR_STATOBST_BELOW_VEL_THRES))
/*! The speed below which radar only stationary objects can be selected
 * @unit:m/s */
#define FPS_PAR_ACC_NO_CAM_CONF_STAT_MAX_VEL (5.f / C_KMH_MS)
/*! The distance below which radar only stationary objects can be selected
 * @unit:m */
#define FPS_PAR_ACC_NO_CAM_CONF_STAT_MAX_DIST (5.f)
#endif

/*! Tunnel probability threshold, from which stationary objects get ignored
 * @min:0 @max: 1 */
#define FPS_TUNNEL_PROB_THRES 0.50f

#if (CFG_Envm_PERIO_COUNT == 1)
/*! Minimum number of periodic Cycles for a stationary object */
#define FPS_PERIO_COUNT 10u
#endif

/*! Maximal distance for relevant stationary Obstacles */
#define FPS_STATOBST_EFFECTIVE_RANGE_MAX 90.0f
/*! Distance threshold for swithing effective range modi of relevant stationary
 * Obstacles */
#define FPS_STATOBST_EFFECTIVE_RANGE_MIDLE 60.0f
/*! Minimal distance for relevant stationary Obstacles */
#define FPS_STATOBST_EFFECTIVE_RANGE_MIN 40.0f

/*! Start Speed of the first Range mode */
#define FPS_STATOBST_EFFECTIVE_RANGE1_SPEED (30.0f / C_KMH_MS)
/*! Start Speed of the second Range mode */
#define FPS_STATOBST_EFFECTIVE_RANGE2_SPEED (50.0f / C_KMH_MS)
/*! Start Speed of the third Range mode */
#define FPS_STATOBST_EFFECTIVE_RANGE3_SPEED (105.0f / C_KMH_MS)

#if (FPS_CFG_ENABLE_RAM_PARAM)
#define FPS_DECL_PARAM(type_, name_) extern type_ name_;
#define FPS_DEF_PARAM(type_, name_, value_) type_ name_ = (value_);
#else
#define FPS_DECL_PARAM(type_, name_) extern const type_ name_;
#define FPS_DEF_PARAM(type_, name_, value_) const type_ name_ = (value_);
#endif

/*****************************************************************************
  VARIABLES DECLARTIONS
*****************************************************************************/

/*! Parameter: Enforce video confirmation for all radar objects */
#define EM_FPS_PAR_CAM_CONFIRM_ALW_ON EmFpsParCamConfirmAlwOn_c
#define EM_FPS_PAR_CAM_CONFIRM_AWL_ON_DEFAULT FALSE
FPS_DECL_PARAM(boolean, EM_FPS_PAR_CAM_CONFIRM_ALW_ON)

/*! Parameter: no reaction on Pedestrian classified objects */
#define EM_FPS_PAR_NO_PED_FUNCTION SWITCH_ON

/*! Parameter: no reaction on Pedestrian classified objects */
#define EM_FPS_PAR_NO_PREBRAKE_ON_ONCOMING SWITCH_ON

#define EM_FPS_PAR_SUPPRESS_PSEUDO_MOVING SWITCH_OFF

/*! Parameter: observes Stopped Objects in Tunnels*/
#define EM_FPS_PAR_TUNNEL_STOPPED_OBSERVER SWITCH_ON
#define FPS_EBA_STOPPED_TUNNELPROB_MIN 0.70f

/*! Enable reaction on point classified objects */
#define EM_FPS_PAR_POINT_OBJ_ON EmFpsParPointObjOn_c
#define EM_FPS_PAR_POINT_OBJ_ON_DEFAULT FALSE
FPS_DECL_PARAM(boolean, EM_FPS_PAR_POINT_OBJ_ON)

/*! Enable full reaction on every tracked objects */
#define EM_FPS_PAR_FULL_QUAL_OBJ EmFpsParFullQualObj_c
#define EM_FPS_PAR_FULL_QUAL_OBJ_DEFAULT 0u
FPS_DECL_PARAM(ui8_t, EM_FPS_PAR_FULL_QUAL_OBJ)

// extern const sint8 FPSLookupMTF2MTD[256];
extern const float32 RCSThreshTargetOffsetStat[SI_NOF_RG];

/* parameter for min EBA-Quality to be mapped to DAI-specific ObjQual */
#define FPS_CMS_OBJQUAL_MIN (85u)

/*! fps_obj_priorization.c */
#if (FPS_CFG_QUOTA_PRIO_VERSION)

/*! Definition of quotas for object prioritization */
/*! By the quota for objects of a specific object class it is guaranteed that as
   least this number of objects of the object class is selected for FCT
    (if there are so many objects in the EM-list) */
/*! Sum of the following + default quotas MUST NOT exceed Envm_N_OBJECTS */
#if (CFG_OD_CAM_NUMBER_OF_CAM_ONLY_OBJECTS > 0u)
/* max. 3 Camera Only Objects */
#define FPS_PRIO_QUOTA_CUSTOM                                             \
    (                                                                     \
        CFG_OD_CAM_NUMBER_OF_CAM_ONLY_OBJECTS) /*!< Quota in FCT-list for \
                                                  custom specific objects */
#define FPS_PRIO_QUOTA_ACC_MOVING_OOI_CANDIDATE_OR_TRACE                     \
    (                                                                        \
        18u -                                                                \
        (CFG_OD_CAM_NUMBER_OF_CAM_ONLY_OBJECTS - 1u)) /*!< Quota in FCT-list \
                                                         for objects with a  \
                                                         certain quality and \
                                                         close the the ego   \
                                                         course OR Trace*/
#define FPS_PRIO_QUOTA_EBA_MOVING_OOI_CANDIDATE                              \
    (2u) /*!< Quota in FCT-list for objects with a certain quality and close \
            the the ego course*/
#define FPS_PRIO_QUOTA_ACC_STATIONARY_OOI_CANDIDATE                        \
    (                                                                      \
        12u) /*!< Quota in FCT-list for objects with a certain quality and \
                close the the ego course*/
#define FPS_PRIO_QUOTA_EBA_STATIONARY_OOI_CANDIDATE                          \
    (2u) /*!< Quota in FCT-list for objects with a certain quality and close \
            the the ego course*/
#define FPS_PRIO_QUOTA_MOVING_QUAL_DIST2TRAJ                                 \
    (0u) /*!< Quota in FCT-list for moving objects with a certain quality or \
            close the the ego course */
#define FPS_PRIO_QUOTA_ONCOMING_QUAL_DIST2TRAJ                                 \
    (5u) /*!< Quota in FCT-list for oncoming objects with a certain quality or \
            close the the ego course */
#define FPS_PRIO_QUOTA_STATIONARY_QUAL_DIST2TRAJ                         \
    (                                                                    \
        0u) /*!< Quota in FCT-list for stationary objects with a certain \
               quality or close the the ego course */

#else
#define FPS_PRIO_QUOTA_CUSTOM \
    (0u) /*!< Quota in FCT-list for custom specific objects */
#define FPS_PRIO_QUOTA_ACC_MOVING_OOI_CANDIDATE_OR_TRACE                   \
    (                                                                      \
        19u) /*!< Quota in FCT-list for objects with a certain quality and \
                close the the ego course OR Trace*/
#define FPS_PRIO_QUOTA_EBA_MOVING_OOI_CANDIDATE                              \
    (2u) /*!< Quota in FCT-list for objects with a certain quality and close \
            the the ego course*/
#define FPS_PRIO_QUOTA_ACC_STATIONARY_OOI_CANDIDATE                        \
    (                                                                      \
        12u) /*!< Quota in FCT-list for objects with a certain quality and \
                close the the ego course*/
#define FPS_PRIO_QUOTA_EBA_STATIONARY_OOI_CANDIDATE                          \
    (2u) /*!< Quota in FCT-list for objects with a certain quality and close \
            the the ego course*/
#define FPS_PRIO_QUOTA_MOVING_QUAL_DIST2TRAJ                                 \
    (0u) /*!< Quota in FCT-list for moving objects with a certain quality or \
            close the the ego course */
#define FPS_PRIO_QUOTA_ONCOMING_QUAL_DIST2TRAJ                                 \
    (5u) /*!< Quota in FCT-list for oncoming objects with a certain quality or \
            close the the ego course */
#define FPS_PRIO_QUOTA_STATIONARY_QUAL_DIST2TRAJ                         \
    (                                                                    \
        0u) /*!< Quota in FCT-list for stationary objects with a certain \
               quality or close the the ego course */
#endif

#endif

#if defined(FPS_CFG_OBJECT_MOSA_CHECK) && (FPS_CFG_OBJECT_MOSA_CHECK == 1)
/************************************************************************/
/* Moving safe defines                                                  */
/************************************************************************/
/* Activate handling of moving objects.*/
#define FPSEBA_MOSA_SET_PERMISSION_HANDLE_MOVING (1)

/*! Activate handling of stopped objects.
(Should be deactivated in camera projects, if stopped objects are handled
identical to stationary objects) */
#define FPSEBA_MOSA_SET_PERMISSION_HANDLE_STOPPED (1)

/*! Activate usage of DistY check (default). Can be switched of to reduce memory
 * consumption */
#define FPSEBA_MOSA_USE_DISTY (1)

/*! Activate usage of "velocity vs. position change" plausibility check. Can be
 * switched of to reduce memory consumption */
#define FPSEBA_MOSA_USE_VELO_WAY_PLAUSIBILIZATION (1)
#endif

#endif

/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */
