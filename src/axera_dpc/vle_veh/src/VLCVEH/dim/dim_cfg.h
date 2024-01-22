
#ifndef VLC_DIM_CFG_H_INCLUDED
#define VLC_DIM_CFG_H_INCLUDED

#include "TM_Global_Types.h"
#include "stddef.h"
#include "vlc_config.h"

/*
  Config: Component DIM Driver Intention Monitoring
**************************************************************************/

/*! @brief       VLC_DIM_CFG_HYPO_EBA
    @general     Enable DIM for EBA
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_ON   @unit -     @min -   @max -   */
#define VLC_DIM_CFG_HYPO_EBA VLC_CFG_HYPOTHESIS_EVAL_AND_DECISION

/*! @brief       VLC_DIM_CFG_HYPO_SI
    @general     Enable DIM for SI
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_OFF   @unit -     @min -   @max -   */
#define VLC_DIM_CFG_HYPO_SI VLC_CFG_DIM_OUTPUT_CUSTOM_VLC_SEN_INPUT

/*! @brief       VLC_DIM_CFG_HYPO_ALDW
    @general     Enable DIM for ALDW
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_ON   @unit -     @min -   @max -   */
#define VLC_DIM_CFG_HYPO_ALDW SWITCH_OFF

/*
  Config: Select Hypothesis
**************************************************************************/

/* TODO: Switch does not work */
/*! @brief       VLC_CFG_HYPOTHESIS_SPORTSTYLE
    @general     VLC hypothesis driver SportStyle calculation
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_ON   @unit -     @min -   @max -   */
#define VLC_CFG_HYPOTHESIS_SPORTSTYLE SWITCH_OFF

#define VLC_CFG_HYPOTHESIS_LANECHANGE SWITCH_OFF

/* TODO: Switch does not work */
/*! @brief       VLC_CFG_HYPOTHESIS_DISTRACTION
    @general     VLC hypothesis driver distraction calculation
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_OFF   @unit -     @min -   @max -   */
#define VLC_CFG_HYPOTHESIS_DISTRACTION SWITCH_OFF

/*! @brief       VLC_CFG_DIM_OUT_CALIB
    @general     VLC driver hypothesis output calibration
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_ON   @unit -     @min -   @max -   */
#define VLC_CFG_DIM_OUT_CALIB SWITCH_ON

/*! @brief       VLC_CFG_DIM_IN_PREPROC
    @general     VLC DIM input preprocessing module
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_ON   @unit -     @min -   @max -   */
#define VLC_CFG_DIM_IN_PREPROC SWITCH_ON

/*
  Config: Select Optional Functionalities
**************************************************************************/

/*! @brief       VLC_CFG_DIM_USE_KICKDOWN_INPUT
    @general     Enable Usage of KickDown Boolean Information
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_OFF   @unit -     @min -   @max -   */
#define VLC_CFG_DIM_USE_KICKDOWN_INPUT SWITCH_OFF

/*! @brief       VLC_CFG_DIM_USE_DRV_OVRRDE_INPUT
    @general     Allow external driver override signal
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     SWITCH_OFF   @unit -     @min -   @max -   */
#define VLC_CFG_DIM_USE_DRV_OVRRDE_INPUT SWITCH_OFF

#endif
