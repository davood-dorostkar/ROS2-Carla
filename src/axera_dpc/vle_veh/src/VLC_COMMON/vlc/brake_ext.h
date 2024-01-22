
/** @defgroup vlc_brake VLC_BRAKE
   @ingroup acc_long_veh

@{ */
#ifndef BRAKE_EXT_H
#define BRAKE_EXT_H

/* includes            */

/*! @brief Brake status bits */
typedef struct brake_status_t {
    ubit8_t BRAKE_FAILED : 1;
    ubit8_t PEDAL_INIT_TRAVEL : 1;
    ubit8_t : 5;
} brake_status_t;

#endif
/** @} end defgroup */
