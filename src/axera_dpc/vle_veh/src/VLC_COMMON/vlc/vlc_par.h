
#ifndef VLC_PAR_H_INCLUDED
#define VLC_PAR_H_INCLUDED

/*! @brief Maximale VLC Reichweite (der Antenne) */
#define RW_VLC_MAX (200.F) /* ARS2xx: (150.F) */

/*******************************/
/*! vehicle related parameters */
/*******************************/
/*! default value for vehicle width */
//#define TRACKWIDTHFRONT_DEFAULT 2.00f
/*! default value for sensor frontoverhang (dist. from front axle to sensor) */
//#define FRONTOVERHANG_DEFAULT 0.80f
/*! default value for wheelbase (dist. between front/rear axle) */
#define VLC_WHEELBASE_DEFAULT 2.85f
/*! default value for axle load distribution */
//#define AXLELOADDISTRIBUTION_DEFAULT 0.5f

/*! The stopped confidence threshold for ACC function */
#define VLC_AVLC_PAR_OBJ_STOPPED_MIN_CONF 75u

/*! Delay time after all preconditions of sensor power reduction are satisfied
until power reduction is actually entered. Note: a setting of zero disables
delay
timing, meaning if sensor power reduction conditions are satisifed, power is
reduced without delay @unit:ms */
#define VLC_PAR_SENSOR_POWER_REDUCTION_DELAY_MS 0

#endif
