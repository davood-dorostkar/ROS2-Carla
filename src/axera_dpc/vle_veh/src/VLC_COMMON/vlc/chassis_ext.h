
/** @defgroup vlc_chassis VLC_CHASSIS
   @ingroup acc_long_veh

@{ */
#ifndef CHASSIS_EXT_H
#define CHASSIS_EXT_H

/* includes            */

/*! @brief Chassis status bits */
typedef struct chassis_status_t {
    ubit8_t ABS_ACT : 1;
    ubit8_t TCS_ACT : 1;
    ubit8_t ESP_ACT : 1;
    ubit8_t PB_ACT : 1;
    ubit8_t TCS_ESP_OFF : 1;
    ubit8_t : 3;
} chassis_status_t;

#endif
/** @} end defgroup */