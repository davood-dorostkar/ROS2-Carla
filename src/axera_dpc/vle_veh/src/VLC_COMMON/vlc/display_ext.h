
/** @defgroup vlc_display VLC_DISPLAY
   @ingroup acc_long_veh

@{ */
#ifndef DISPLAY_EXT_H
#define DISPLAY_EXT_H

/* Assume that #include "vlc_glob_ext.h" was already performed */

/* Alert level */
#define Display_alert_level0 (display_alert_level_t)0
#define Display_alert_level1 (display_alert_level_t)1
#define Display_alert_level2 (display_alert_level_t)2
#define Display_alert_level3 (display_alert_level_t)3
#define Display_alert_level4 (display_alert_level_t)4
typedef enum_t display_alert_level_t;

/*! State variable of the HMI state machine */
#define Display_state_cc_off ((display_state_t)0)
#define Display_state_cc_active ((display_state_t)1)
#define Display_state_cc_override ((display_state_t)2)
#define Display_state_cc_disengage ((display_state_t)3)
#define Display_state_lim_off ((display_state_t)4)
#define Display_state_lim_active ((display_state_t)5)
#define Display_state_lim_override ((display_state_t)6)
#define Display_state_lim_disengage ((display_state_t)7)
typedef enum_t
    display_state_t; /*!< @values:
                        Display_state_cc_off,Display_state_cc_active,Display_state_cc_override,Display_state_cc_disengage,Display_state_lim_off,Display_state_lim_active,Display_state_lim_override,Display_state_lim_disengage
                        */

#if (defined(_MSC_VER))
#pragma COMPILEMSG( \
    "display_op_status_t type can be removed from display_ext.h, since now contained in RTE!")
#endif

/*! @brief display telltale status */
typedef struct display_telltale_status_t {
    ubit8_t OBJECT_DETECTED : 1;
    ubit8_t OBJECT_STATIONARY : 1;
    ubit8_t OBJECT_EFFECTIVE : 1;
    ubit8_t DRIVE_OFF_POSSIBLE : 1;

    ubit8_t VEHICLE_STANDSTILL : 1;
    ubit8_t : 3;
} display_telltale_status_t;

/* GLOBAL VARIABLES (KOMPONENT EXTERNAL) */

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */

#endif
/** @} end defgroup */
