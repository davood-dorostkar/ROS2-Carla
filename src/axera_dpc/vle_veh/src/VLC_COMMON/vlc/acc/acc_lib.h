
#ifndef AVLC_LIB_H
#define AVLC_LIB_H

/* includes */

/* GLOBAL VARIABLES (KOMPONENT EXTERNAL) */

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */

extern distance_t AVLC_GET_ALERT_DISTANCE(const VLC_acc_object_t *object);
extern distance_t AVLC_DETERMINE_MAX_INTRUSION(distance_t AlertDistance,
                                               distance_t RequestedDistance,
                                               velocity_t VehicleSpeed,
                                               percentage_t headway_setting,
                                               velocity_t Relative_Speed,
                                               factor_t modification_factor);

#endif
