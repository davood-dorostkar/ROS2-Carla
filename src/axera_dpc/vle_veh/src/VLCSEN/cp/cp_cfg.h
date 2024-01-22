
#ifndef CP_CFG_H_INCLUDED
#define CP_CFG_H_INCLUDED

#include "vlc_config.h"

/*****************************************************************************
  Config: CP (Course Prediction)
*****************************************************************************/

/*! @brief  use slip angle information for trajectory calculation */
#define CFG_SA_USE_SLIPANGLE 0

/*!@brief   Configuration switch to enable calculation of last object position
in
function CPCalculateObjectReference. This value is not used anywhere in the
software. A seperate switch for it was created to clarify if this functionality
is needed in the future. @todo: Clarify if needed! */
#define CP_CFG_TRAJECTORY_USE_LAST_OBJ_POS 0

#define CP_CFG_ENABLE_EXTEND_ROAD_FUSION_DIST 0

/*! @brief  Configuration switch to enable a set of routines useful for testing
in
cp_lineseg.c */
#define CP_CFG_LINESEG_TEST_ROUTINES 0

/*! @brief  Configuration switch to enable support for camera lane fusion
features
when changing lanes */
#define CP_CFG_LANE_CHG_CAM_FUSION \
    VLC_CFG_DEPENDENT_SWITCH(VLC_CFG_SEN_CAM_LANE_INTERFACE, 1)

/*! @brief  Configuration switch to enable trace fusion in tunnels. If set to
off,
then tunnel probability of over 50% will disable trace fusion */
#define CP_CFG_ENABLE_TRACE_FUSION_IN_TUNNEL 0

#define CP_CFG_USE_LOWER_VEL_ROAD_FUSION 0

/*!@brief   Configuration switch to use EM kinematics instead of Kalman
 * calculation for Dist2Traj */
#define CP_CFG_DIST2TRAJ_EM 0

#endif
