
#ifndef VLC_CFG_H_INCLUDED
#define VLC_CFG_H_INCLUDED

/*****************************************************************************
  Config: CC (Cruise Control)
*****************************************************************************/

/*! Straps to enable/disable specific functionality within the Cruise Control
 * function */
#define CFG_VLC_VLC_USE_BRAKE (1)   /*use brakes also fot cruise controll*/
#define CFG_VLC_VLC_USE_LAT_LIM (1) /*use lateral limiter*/
#define CFG_VLC_VLC_USE_DECEL_LIM_OVERRIDE                                   \
    (1) /*do not brake more than XXX after driver override (also to relevant \
           objects)*/
#define CFG_VLC_VLC_USE_DECEL_LIM_ENGAGE                                    \
    (1) /*do not brake more than XXX after engagement and override (without \
           objects)*/
#define CFG_VLC_VLC_USE_BRAKES_FOR_LAT_LIM                \
    (0) /*use brakes for lateral limiter (only valid with \
           CFG_VLC_VLC_USE_LAT_LIM)*/
#define CFG_VLC_VLC_BRAKE_LAT_LIM_AFTER_OR                                  \
    (                                                                       \
        0) /*use brakes for lateral limiter also after override (only valid \
              with CFG_VLC_VLC_USE_BRAKES_FOR_LAT_LIM)*/
#define CFG_VLC_VLC_USE_SMOOTH_JERKS \
    (1) /*use logic to use smooth gradients for acceleration request*/
#define CFG_VLC_VLC_USE_MIN_MAX_DISENGAGEMENT                                 \
    (0) /*normal cartronic interface at disengagement the max_req_accel ramps \
           out to maximum, the min_req_accel to minimum, else both to zero*/
#define CFG_VLC_VLC_USE_ACCEL_BAND_MODIFICATION                                \
    (0) /*spread acceleration request band (min/max acceleration) to allow the \
           vehicle to "roll" as long as its acceleration is within the band*/
#define CFG_VLC_VLC_CHANGE_SETSPEED_WHILE_READY_MODE \
    (0) /* Allow the change of the setspeed even in ready mode*/
#define CFG_VLC_INIT_ACCEL_AFTER_STANDSTILL (1) /*Init Accel*/
#define CFG_VLC_VLC_NO_ACCEL_LIM_IN_DISENGAGE                                 \
    (1) /*do not limit accel request in disengage mode (band will ever run to \
           Acc_max_accel_disengage and Acc_min_accel_disengage or until       \
           acknowledge)*/
#define CFG_VLC_VLC_USE_HIGH_MIN_MAX_ACCEL_LIMIT                          \
    (0) /*this strap raises the maximum limit of a_req_max and lowers the \
           minimum limit of a_req_min to avoid brake intervention*/
#define CFG_VLC_VLC_USE_LONG_ACCEL_CUSTOM_LIMIT                             \
    (0) /* if custom limitation of longitudinal acceleration is needed, use \
          VLC_LIMIT_LONG_ACCEL_CUSTOM fuction in custom part                \
          the specific customer requirements should be available for using  \
          this function */
#define CFG_VLC_VLC_USE_CONTROL_CRITICALITY                              \
    (1) /* To get a faster reaction of the longitudinal controller, the  \
           smoothing of the jerk is reduced, if the traffic situation is \
           critical (e.g. leading vehicle brakes strongly) */
#define CFG_VLC_VLC_SMOOTH_ACCEL_REQUEST                                       \
    (1) /* Filter to prevent oscillations and other discontinuous waveforms in \
           acceleration request, which could lead to uncomfortable system      \
           behavior*/
#define CFG_VLC_VLC_USE_DRIVE_MODE                                            \
    (                                                                         \
        0) /* Custom drive mode for different control modes like comfort, eco \
              and sport */
#define CFG_VLC_VLC_USE_CUSTOM_ACCEL_LIMITS \
    (0) /* Use custom CC acceleration limits */
#define GFG_VLC_VLC_USE_PLIM_CONTROL_STATE                           \
    (0) /* If 1 permanent limiter afect state mashine and engagement \
           conditions */
#define CFG_VLC_VLC_ALLOW_INIT_ACCEL_REQUEST                             \
    (1) /* If newly engaged, init requested acceleration to host vehicle \
           acceleration */

/*****************************************************************************
  Config: Cart/CC (CARTRONIC Interface within CC)
  Formerly in vlc_glob_ext.h
*****************************************************************************/

/*! use handshake (disengagement request / acknowledge) over CARTRONIC */
#define CFG_VLC_CART_USE_DISENGAGE_HANDSHAKE (1)

/*! allow to jump always to A_INIT, not only in some special cases */
#define CFG_VLC_ALWAYS_ALLOW_INITIALISATION (0)

/*! use accel gain functionality for accelration low speed range*/
#define CFG_VLC_USE_ACCEL_GAIN (1)

/*! speed limiter support */
#define CFG_VLC_LIM (0)

/*! FLIM support */
#define CFG_VLC_FLIM (0)

/*! PLIM support */
#define CFG_VLC_PLIM (0)

#endif
