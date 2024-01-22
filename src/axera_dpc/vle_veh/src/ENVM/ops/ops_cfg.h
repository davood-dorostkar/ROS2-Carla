
#ifndef OPS_CFG_H_INCLUDED
#define OPS_CFG_H_INCLUDED

/*****************************************************************************
  Config: FPS Object Description GLOBAL
*****************************************************************************/

/*! Configuration switch for enabling selection of stationary objects only
when they are camera confirmed. (note: only has an effect when switch
CFG_CAMERA_RADAR_FUSION is enabled) */
#define FPS_CFG_ONLY_SELECT_CAM_CONFIRMED_STAT                     \
    ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) ||    \
     ((CFG_Envm_CAMOBJ_FUSION_LEVEL == CFG_Envm_FUSION_LEVEL_1) && \
      (FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 == SWITCH_ON)))

/*! Configuration switch to enable special work-around for disabling the
camera confirmation for motorcycle-class objects for BMW. Leave this switch
disabled, unless the special functionality of supressing motorcycle class
camera objects confirmation is desired */
#define FPS_CFG_CAMERA_OBJ_MOTORCYCLE_WORKAROUND 0

/*! Configuration switch to enable work-around for allowing objects once
confirmed by the camera to continue passing pre-selection, even if the
camera confirmation disappears in the meantime */
#define FPS_CFG_CAMERA_CONF_HOLD_WORKAROUND                        \
    ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) ||    \
     ((CFG_Envm_CAMOBJ_FUSION_LEVEL == CFG_Envm_FUSION_LEVEL_1) && \
      (FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 == SWITCH_ON)))

/*! Configuration switch requiring oncomming stopped objects (i.e.: objects
that were never seen moving) to be camera confirmed */
#define FPS_CFG_USE_CAM_FOR_ONCOME_STOPPED                         \
    ((CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2) ||    \
     ((CFG_Envm_CAMOBJ_FUSION_LEVEL == CFG_Envm_FUSION_LEVEL_1) && \
      (FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 == SWITCH_ON)))

/*! Configuration switch for enabling selection of stationary objects without
 * cam-confirmation within a narrow corridor. */
#define FPS_CFG_SUPPRESS_STAT_OUT_CORRIDOR \
    (CFG_Envm_CAMOBJ_FUSION_LEVEL >= CFG_EM_FUSION_LEVEL_2)

/*! Configuration switch which enables hold of relevant stationary objects
 * without camera confirmation */
#define FPS_CFG_HOLD_RELEVANT_STAT_OBJ 0

/*! Configuration switch for selection of stationary objects through camera
 * confirmation only within effective range for stationary objects */
#define FPS_CFG_CAM_CONF_CHAR_CONSIDER_EFFECTIVE_RANGE_STAT_OBJ 0

/*! Enable support for RAM tunable parameters */
#define FPS_CFG_ENABLE_RAM_PARAM 1

/*! Enable usage of tracker update quality in addition to probability of
    existence (POE) for eba quality calculation. This is used to suppress
    split-off objects */
#define FPS_CFG_USE_UPDATE_QUALITY 1

/*! Enables dynamic reconfiguration of eba quality calculation dependent on
 * camera availabilty */
#define FPS_CFG_EBA_DYN_CONFIG_CAM 1

/*! Switch to enable the object priorisation based on given quotas */
#define FPS_CFG_QUOTA_PRIO_VERSION                                         \
    (1) /*!< To be verified if the switched off code can be removed on the \
           long run */

/*! activate custom reject criteria in object prioritization: reject object
 * outside of trumpet */
// changed to 0 by guotao 20190418. We do not have em module anymore
#define FPS_PRIO_REJECT_TRUMPET (0)

/*! activate custom reject criteria in object prioritization: reject objects
 * outside of field-of-view */
#define FPS_PRIO_REJECT_FOV (0)

/*! activate custom reject criteria in object prioritization: reject remote
 * mirror */
#define FPS_PRIO_REJECT_MIRROR (0)

/*! Enable temporary fix for suppressing ghost targets due to wrong AzAng
 * hypotheses in B1 samples, remove once not needed any more */
#define FPS_CFG_SUPPRESS_GHOST_TARGETS (0)

/*! Suppression of mirror ghost objects. This logic was developed for the entry
   to suppress ghosts
   due to angle ambiguities which are common e.g. on the Tokyo highway */
#define FPS_CFG_SUPPRESS_MIRROR_GHOST_TARGETS (0)

/*! Hack to prevent nacom object to become relevant ACC object. This switch
 * statement must not be active in any series project! */
#define FPS_CFG_SUPPRESS_NACOM_REL_OBJECT (0)

/*! Prevent moving objects on bridges to become relevant ACC object. */
#define FPS_CFG_SUPPRESS_MOV_ON_BRIDGE (1)

/* Suppress moving objects on guardrail caused by vrel ambiguity in one scan */
#define FPS_CFG_SUPPRESS_VRELAMBIG_ON_GUARDRAIL (0)

/* Suppress pedestrians for EBA in Tunnels */
#define FPS_CFG_EBA_SUPPRESS_PED_IN_TUNNEL (1)

/* Enable relevant stationary objects while ego-vehicle is in standstill or
 * v_ego is below a certain threshold */
#define FPS_CFG_RADAR_STATOBST_BELOW_VEL_THRES (1)

/* Suppression of ACC quality of a priori stationary objects when carpark
 * detected */
#define FPS_CFG_ACC_NO_STAT_OBJ_IN_CARPARK (0)

/* Selection of stationary objects for fusion level CFG_Envm_FUSION_LEVEL_1
 * instead of CFG_EM_FUSION_LEVEL_2 */
#define FPS_CFG_ACC_STAT_OBJ_FUSION_LEVEL_1 (0)

/*! Activate usage of moving safe object check */
#define FPS_CFG_OBJECT_MOSA_CHECK (1)

#endif /*  FPS_CFG_H_INCLUDED  */
