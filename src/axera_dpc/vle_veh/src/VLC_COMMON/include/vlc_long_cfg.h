
#ifndef VLC_LONG_CFG_H_INCLUDED
#define VLC_LONG_CFG_H_INCLUDED

/*****************************************************************************
  Config: VLC_LONG (Longitudinal functions)
  Note: these were formerly in vlc_config.h
*****************************************************************************/

#define CFG_VLC_CC (1)
#define CFG_VLC_ACC (1)
/* Note: currently not used anywhere: #define CFG_VLC_ESRACC (0) */
#define CFG_VLC_FSRACC (1)
#define CFG_VLC_DM (0)
#define CFG_VLC_FCA                                                     \
    (0) /* @todo: currently disabled since access to VehSig needed, see \
           VLC_SEN_CFG_VEH_SIG_INPUT */
#define CFG_VLC_LODM (0)

//#define CFG_VLC_LDW                             (0)
//#define CFG_VLC_HC                              (0)
//#define CFG_VLC_LADM                            (0)

//# pragma COMPILEMSG("Remove CFG_VLC_DIE and it's dependencies from vlc-long!")
//#define CFG_VLC_DIE                             (1)

/*****************************************************************************
  Config: Long (Longitudinal functions)
*****************************************************************************/

/*! Obsolete configuration switch for enabling object class in ACC/CC objects.
The value was never used in any code, but in case it's needed it's copy/init
can be re-enabled via this switch */
#define OBJ_CLASS_IN_LONG_CTRL_NEEDED 0

/*! Use a special accel/decel mode where a specific acceleration request is used
 * as long as the driver wants so */
#define CFG_VLC_VLC_USE_ACCEL_DECEL_MODE (0)

/*! Ramp ReqAcceleration to -2m/s?to hold vehicle in standstill as dynamic part
 * is not able to... */
#define CFG_VLC_VLC_USE_HOLD_STILLSTAND_MOD (0)

/*! Reset Setspeed */
#define CFG_VLC_VLC_RST_SETSPEED_DECEL_ONLY (0)

/*! Use host accel from VDY, otherwise aInit will be used for velocity
 * prediction */
#define CFG_VLC_USE_VDY_HOST_ACCEL (1)

/*! use velocity of VDY component */
#define CFG_VLC_USE_VDY_HOST_VELOCITY (1)

/*! Only display object in host lane */
#define CFG_VLC_DISPLAY_OBJ_ONLY_IN_HOST_LANE (1)

/*! Use predictied host velocity for lateral accelration calculation */
#define CFG_VLC_USE_PRED_VELO_FOR_ALAT (0)

/*! Use internal object absolute speed filtering */
#define LONG_CFG_INTERNAL_FILTER_ABS_OBJ_SPEED (0)

/*! Use internal object absolute acceleration filtering */
#define LONG_CFG_INTERNAL_FILTER_ABS_OBJ_ACCEL (0)

#define LONG_CFG_USE_EXTERN_AVLC_STATE_MACHINE (!CFG_AVLC_LEVER_INPUT)

/*! Use error reporting strategy with differentiation between performance
    degradation, temporary- or permanent failure */
#define LONG_CFG_USE_DRIVER_DATA_ERROR_REPORTING (0)

/*! Use output controlled arbitration interface for projects with
    external conventional cruise function, running on a separate ECU.*/
#define LONG_CFG_USE_OUTPUT_CONTROLLED_ARBITRATION_WITH_EXT_CC (1)

/*! Use linear interpolation for output controlled arbitration interface
    for projects with external conventional cruise function, running on
    a separate ECU. */
#define LONG_CFG_USE_LIN_INTERPOL_ACCEL_CURVE_FOR_ARBIT_WITH_EXT_CC          \
    (1) /*0 -> stepwise curve between vertices,                              \
          1 -> linear interpolation between vertices                         \
          note: needs LONG_CFG_USE_OUTPUT_CONTROLLED_ARBITRATION_WITH_EXT_CC \
          (1)*/

/*! Internal control state is ACTIVE even if external control state is STANDBY*/
#define LONG_CFG_CONTROLLER_IS_ACTIVE_IN_STANDBY (0)

#define ARS410SW18 (1)

#define ARSDP01 (0)

#endif
