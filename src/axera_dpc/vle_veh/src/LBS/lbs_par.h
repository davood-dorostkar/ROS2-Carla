#pragma once
#ifndef LBS_PAR_H
#define LBS_PAR_H

/*****************************************************************************
  CONSTS
*****************************************************************************/
#define LBS_TTC_INVALID TUE_C_F32_VALUE_INVALID

// LowPassFilter alpha defines
#define LBS_LPF_TTCFILTERED_ALPHA (0.25f)
#define LBS_LPF_VRELXY_ALPHA (0.1f)

#define LBS_UPDATERATE_FILTER_UP (0.05f)
#define LBS_UPDATERATE_FILTER_DOWN (0.025f)
#define LBS_ASSOCPROB_FILTER_UP (0.05f)
#define LBS_ASSOCPROB_FILTER_DOWN (0.015f)

#define LBS_MIN_UPDATERATE_TTC_ACCEL \
    (0.75f)  // the min update rate threshold for TTC calculate

#define LBS_OBJSEL_TARGETRANGE_MAX (71.0f)
#define LBS_OBJSEL_VEGO_MIN (-100.0f)
#define LBS_OBJSEL_VEGO_MAX (100.0f)
#define LBS_OBJSEL_VTARGETMIN (0.83f)
#define LBS_OBJSEL_XMAX_BREAKTHROUGH (7.0f)
#define LBS_OBJSEL_XMIN_BREAKTHROUGH (0.0f)

#define LBS_MAXSPD_OVERGND_LIFETIME_THRESH (100u)
#define LBS_MAXSPD_OVERGND_FASTILTER_MIN_FMRTE (1.0f)
#define LBS_MAXSPD_OVERGND_FASTFILTER \
    (0.01f)  // max over ground speed filter up alpha
#define LBS_MAXSPD_OVERGND_SLOWFILTER \
    (0.00001f)  // max over ground speed filter down alpha
#define LBS_MAXSPD_OVERGND_ROADTYPE_CONF_MIN (0.7f)

#define LBS_FIRSTDYNPROPERTY_POE_THRESH (0.9f)
#define LBS_FIRSTDYNPROPERTY_LIFETIME_THRESH (20u)

#define LBS_FIRSTDETECTDIST_LIFETIME_THRESH (1u)

#define EM_GEN_OBJ_MT_STATE_DELETED (0u)
#define EM_GEN_OBJ_MT_STATE_MEASURED (2u)

#define EM_GEN_OBJECT_DYN_PROPERTY_MOVING (0u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY (1u)
#define EM_GEN_OBJECT_DYN_PROPERTY_ONCOMING (2u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STATIONARY_CANDIDATE (3u)
#define EM_GEN_OBJECT_DYN_PROPERTY_UNKNOWN (4u)
#define EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_STATIONARY (5u)
#define EM_GEN_OBJECT_DYN_PROPERTY_CROSSING_MOVING (6u)
#define EM_GEN_OBJECT_DYN_PROPERTY_STOPPED (7u)

#define LBS_EM_GEN_OBJECT_CLASS_POINT (0u)
#define LBS_EM_GEN_OBJECT_CLASS_CAR (1u)
#define LBS_EM_GEN_OBJECT_CLASS_TRUCK (2u)
#define LBS_EM_GEN_OBJECT_CLASS_PED (3u)
#define LBS_EM_GEN_OBJECT_CLASS_MOTOCYCLE (4u)
#define LBS_EM_GEN_OBJECT_CLASS_BICYCLE (5u)
#define LBS_EM_GEN_OBJECT_CLASS_WIDE (6u)
#define LBS_EM_GEN_OBJECT_CLASS_RESERVED (7u)

/* definition of classification dependent minimum, maximum and default
 * dimensions */
/* class car minimum, maximum and default dimensions */
#define LBS_EM_GEN_CLASS_CAR_MIN_LENGTH 3.0f
#define LBS_EM_GEN_CLASS_CAR_MAX_LENGTH 7.0f
#define LBS_EM_GEN_CLASS_CAR_DEFAULT_LENGTH 5.0f
#define LBS_EM_GEN_CLASS_CAR_MIN_WIDTH 1.8f
#define LBS_EM_GEN_CLASS_CAR_MAX_WIDTH 2.2f
#define LBS_EM_GEN_CLASS_CAR_DEFAULT_WIDTH 1.8f
/* class truck minimum, maximum and default dimensions */
#define LBS_EM_GEN_CLASS_TRUCK_MIN_LENGTH 5.0f
#define LBS_EM_GEN_CLASS_TRUCK_MAX_LENGTH 25.0f
#define LBS_EM_GEN_CLASS_TRUCK_DEFAULT_LENGTH 18.0f
#define LBS_EM_GEN_CLASS_TRUCK_MIN_WIDTH 2.0f
#define LBS_EM_GEN_CLASS_TRUCK_MAX_WIDTH 2.8f
#define LBS_EM_GEN_CLASS_TRUCK_DEFAULT_WIDTH 2.5f
/* class pedestrian minimum, maximum and default dimensions length and width
 * treated same */
#define LBS_EM_GEN_CLASS_PED_MIN_DIMENSION 0.4f
#define LBS_EM_GEN_CLASS_PED_MAX_DIMENSION 0.8f
#define LBS_EM_GEN_CLASS_PED_DEFAULT_DIMENSION 0.6f
/* class motorcycle minimum, maximum and default dimensions */
#define LBS_EM_GEN_CLASS_MOTORCYCLE_MIN_LENGTH 2.0f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_MAX_LENGTH 4.0f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_DEFAULT_LENGTH 2.5f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_MIN_WIDTH 0.5f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_MAX_WIDTH 1.2f
#define LBS_EM_GEN_CLASS_MOTORCYCLE_DEFAULT_WIDTH 1.0f
/* class bicile minimum, maximum and default dimensions */
#define LBS_EM_GEN_CLASS_BICICLE_MIN_LENGTH \
    LBS_EM_GEN_CLASS_MOTORCYCLE_MIN_LENGTH
#define LBS_EM_GEN_CLASS_BICICLE_MAX_LENGTH \
    LBS_EM_GEN_CLASS_MOTORCYCLE_MAX_LENGTH
#define LBS_EM_GEN_CLASS_BICICLE_DEFAULT_LENGTH \
    LBS_EM_GEN_CLASS_MOTORCYCLE_DEFAULT_LENGTH
#define LBS_EM_GEN_CLASS_BICICLE_MIN_WIDTH LBS_EM_GEN_CLASS_MOTORCYCLE_MIN_WIDTH
#define LBS_EM_GEN_CLASS_BICICLE_MAX_WIDTH LBS_EM_GEN_CLASS_MOTORCYCLE_MAX_WIDTH
#define LBS_EM_GEN_CLASS_BICICLE_DEFAULT_WIDTH \
    LBS_EM_GEN_CLASS_MOTORCYCLE_DEFAULT_WIDTH
/* class point minimum, maximum and default dimensions length and width treated
 * same */
#define LBS_EM_GEN_CLASS_POINT_MIN_DIMENSION 0.1f
#define LBS_EM_GEN_CLASS_POINT_MAX_DIMENSION 0.5f
#define LBS_EM_GEN_CLASS_POINT_DEFAULT_DIMENSION 0.4f
/* class unclassified minimum, maximum and default dimensions length and width
 * treated same */
#define LBS_EM_GEN_CLASS_UNCLASSIFIED_MIN_DIMENSION 0.1f
#define LBS_EM_GEN_CLASS_UNCLASSIFIED_MAX_DIMENSION 50.0f
#define LBS_EM_GEN_CLASS_UNCLASSIFIED_DEFAULT_DIMENSION 50.0f

#define LBS_OSE_NUM_OF_BREAK_LINES (2u)
#endif
