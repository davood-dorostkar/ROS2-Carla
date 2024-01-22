
#ifndef AVLC_LIB_EXT_H
#define AVLC_LIB_EXT_H

/* includes */

#include "vlc_glob_ext.h"
#include "acc_obj_ext.h"

/* GLOBAL VARIABLES (KOMPONENT EXTERNAL) */

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern acceleration_t AVLC_DETERMINE_DIST_CONTROL_ACCEL(
    VLC_acc_object_t* const Object,
    times_t MovingTime,
    percentage_t headway_setting,
    acceleration_t a_own,
    velocity_t v_own);
extern times_t AVLC_DETERMINE_TIME_TO_STOP(const acceleration_t acceleration,
                                           const velocity_t velocity);

#endif
