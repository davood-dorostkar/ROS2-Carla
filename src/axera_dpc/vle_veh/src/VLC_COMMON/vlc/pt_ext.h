
/** @defgroup vlc_pt VLC_PT
   @ingroup acc_long_veh

@{ */
#ifndef PT_EXT_H
#define PT_EXT_H

/* includes */
#include "vlc_glob_ext.h"

#ifndef DYN_GEAR_NEUTRAL_GEAR
typedef enum {
    DYN_GEAR_NEUTRAL_GEAR = 0,
    DYN_GEAR_FIRST_GEAR = 1,
    DYN_GEAR_SECOND_GEAR = 2,
    DYN_GEAR_THIRD_GEAR = 3,
    DYN_GEAR_FOURTH_GEAR = 4,
    DYN_GEAR_FIFTH_GEAR = 5,
    DYN_GEAR_SIXTH_GEAR = 6,
    DYN_GEAR_SEVENTH_GEAR = 7,
    DYN_GEAR_EIGHTH_GEAR = 8,
    DYN_GEAR_NINTH_GEAR = 9,
    DYN_GEAR_REVERSE_GEAR = 10,
    DYN_GEAR_PARK_GEAR = 11,
    DYN_GEAR_POWER_FREE = 12,
} eTransmissionGear_t;
#endif
/* WORK-AROUND ENDS */

/*! Gear */
#define Pt_gear_neutral DYN_GEAR_NEUTRAL_GEAR
#define Pt_gear_first DYN_GEAR_FIRST_GEAR
#define Pt_gear_second DYN_GEAR_SECOND_GEAR
#define Pt_gear_third DYN_GEAR_THIRD_GEAR
#define Pt_gear_fourth DYN_GEAR_FOURTH_GEAR
#define Pt_gear_fifth DYN_GEAR_FIFTH_GEAR
#define Pt_gear_sixth DYN_GEAR_SIXTH_GEAR
#define Pt_gear_seventh DYN_GEAR_SEVENTH_GEAR
#define Pt_gear_eighth                                                     \
    DYN_GEAR_EIGHTH_GEAR /* @todo: Verify consequent support everywhere in \
                            code */
#define Pt_gear_ninth DYN_GEAR_NINTH_GEAR
#define Pt_gear_reverse DYN_GEAR_REVERSE_GEAR /* Value change 9=>10 */
#define Pt_gear_park DYN_GEAR_PARK_GEAR       /* Value change 10=>11 */

typedef TransmissionGear_t
    pt_gear_t; /*!< @todo: Remove pt_gear_t and replace with proper RTE type */

/*! @brief Power train status bits */
typedef struct pt_status_t {
    ubit8_t FAIL_REVERSABLE : 1;
    ubit8_t FAIL_IRREVERSABLE : 1;
    ubit8_t KICKDOWN : 1;
    ubit8_t CLUTCH_OPEN : 1;
    ubit8_t SHIFT_IN_PROGRESS : 1;
    ubit8_t : 3;
} pt_status_t;

#endif
/** @} end defgroup */
