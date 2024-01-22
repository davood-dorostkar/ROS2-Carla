
#ifndef AVLC_CFG_H_INCLUDED
#define AVLC_CFG_H_INCLUDED

/*****************************************************************************
  Config: ACC (Adaptive Cruise Control)
*****************************************************************************/

/*! Straps to enable/disable specific functionality within the Adaptive Cruise
 * Control function */
#define CFG_VLC_AVLC_ALERT_STATIONARY_OBJ                                  \
    (                                                                      \
        1) /*acc alert also to (stationary) objektcs (alert will always be \
              occur to stopped objects!)*/
#define CFG_VLC_USE_ONLY_FIRST_OBJECT_FOR_ALERT \
    (1) /*acc alert only to first object in current lane*/

/*****************************************************************************
  Config: ACC (CARTRONIC Interface within CC)
  These defines were formerly in vlc_glob_ext.h, but are only used in ACC
*****************************************************************************/

/*! use only next object for control */
#define CFG_VLC_AVLC_USE_ONLY_FIRST_OBJECT (0)

/*! use only objects in vehicles lane for control*/
#define CFG_VLC_AVLC_USE_ONLY_OBJECTS_IN_LANE (1)

/*! objects in the adjacent lanes are always used for control, else: they are
 * only used if no moving object is present in the host lane */
#define CFG_VLC_AVLC_USE_OBJECTS_IN_ALL_LANES (1)

/*! use driver intention for host lane objects requested distance modification
 */
#define CFG_VLC_AVLC_USE_DRVINT_FOR_HOSTLANE_OBJ (1)

/*! use cut out potential for host lane objects requested distance modification
 */
#define CFG_VLC_AVLC_USE_CUTOUT_FOR_HOSTLANE_OBJ (0)

/*! modify requested distanc to current distance and ramp it up to requested
 * distance (e.g. in standstill) */
#define CFG_VLC_AVLC_MODIFY_REQ_DIST_TO_CURR_DIST (1)

/* Switch On reaction on stationary Objects */
#define CFG_VLC_AVLC_REACT_ON_STANDING_OBJECTS (1)

/*! use time gap independent intrusion, intrusion is constant for different time
 * gap settings */
#define CFG_VLC_AVLC_INTRUSION_TIME_GAP_INDEPENDENT (0)

/*! display only the next object if available, do not display objects on
 * adjacent lanes */
#define CFG_VLC_AVLC_DISPLAY_ONLY_FIRST_OBJECT (0)

/*! use driver mode to adapt ACC Parameter */
#define CFG_VLC_AVLC_USE_DRIVE_MODE (0)

/*! use control criticality for traffic situation analysis */
#define CFG_VLC_AVLC_USE_CONTROL_CRITICALITY (1)

/*! use custom ACC acceleration limits */
#define CFG_VLC_AVLC_USE_CUSTOM_ACCEL_LIMITS (0)

/*! use lane change probability for control (e.g. overtake feature) */
#define CFG_VLC_AVLC_USE_LC_PROB_FOR_CONTROL (0)

#endif
