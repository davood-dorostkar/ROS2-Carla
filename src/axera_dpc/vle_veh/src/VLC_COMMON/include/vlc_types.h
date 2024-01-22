/*! \file **********************************************************************

  COMPANY:                Tuerme

  PROJECT:                All

  CPU:                    All

  COMPONENT:              VLC

  MODULNAME:              vlc_types.h

  DESCRIPTION:            VLC types

  CREATION DATE:          11.09.2007


  ---*/
#ifndef VLC_TYPES_H
#define VLC_TYPES_H
#include "TM_Base_Cfg.h"
#include "TM_Global_TypeDefs.h"
/* Define to access assert functions in VLC */
#define VLC_ASSERT(x) BML_ASSERT(x)

#ifndef ALGO_INLINE
/* Defines for inlining functions*/
#if defined(_MSC_VER) /* Microsoft compiler -> code only for simulation */
/*! microsoft compliler -> simulation*/
#define ALGO_INLINE __inline
#elif (defined(__POWERPC__) &&                                                \
       defined(__MWERKS__)) /* Freescale Metrowerks compiler for PowerPC-ECUs \
                               -> code only for ECU */
/*! Freescale compiler use inline*/
#define ALGO_INLINE inline
#elif (defined(__ghs__)) /* Greenhills compiler */
/*! Greenhills compoiler use inline*/
#define ALGO_INLINE inline
#elif (defined(__STDC_VERSION__) &&                                         \
       (__STDC_VERSION__ >=                                                 \
        199901)) /* C99 compatible compiler has to have inline keyword with \
                    proper non-extern linkage */
/*! C99 compiler use inline*/
#define ALGO_INLINE inline
#elif (defined(__ARP32__)) || (defined(__TI_ARM__)) || (defined(_TMS320C6X))
/*! TI compiler use inline*/
#define ALGO_INLINE inline
#elif (defined(__GNUC__))
/*! GNU compiler use inline*/
#define ALGO_INLINE inline
#else /* unknown compiler -> no INLINE */
/*! unknown compiler NO inlining*/
#define ALGO_INLINE static
#endif
#endif

/* ***************************************************************************************************
 */
/* ***************************************************************************************************
 */
/* TEVES TYPES                      */
/* The use of the following types are limited to    */
/* software components which have to run on A.D.C.      */
/* MCU's and TEVES MCU's                                */
/* DO NOT USE (MIX) BOTH STYLES IN ONE COMPONENT        */
/*! TEVES STYLE: Limited to components which have to run on A.D.C. MCU's and
 * TEVES MCU's */
/* ***************************************************************************************************
 */

/* TYPEDEFS (GLOBAL) */
/* Old typedefs signed_char_t, unsigned_char_t, signed_int8_t, unsigned_int8_t
etc. have been removed as replaced by standard Autosar types */

/* DATA DICTIONARY */
/*! alias to scale 1e6*/
#define Scale_1000000 (1000000)
/*! alias to scale 1e5*/
#define Scale_100000 (100000)
/*! alias to scale 1e4*/
#define Scale_10000 (10000)
/*! alias to scale 1e3*/
#define Scale_1000 (1000)
/*! alias to scale 1e2*/
#define Scale_100 (100)
/*! alias to scale 1e1*/
#define Scale_10 (10)
/*! alias to scale 1e0*/
#define Scale_1 (1)

/*! scale for Acceleration_t */
#define Acceleration_s Scale_1000
/*! minimum for Acceleration_t */
#define Accel_min (-32767)
/*! maximum for Acceleration_t */
#define Accel_max (32766)

/*! Gradient
@description   Change rate (d/dt) of signal. Attention, gradient is scaled same
as signal it is derived from:
@resolution    0.001
@unit          [?/s]
@min           -2147483.648
@max           +2147483.647 */
typedef sint32 gradient_t; /*%scale:0.001 unit:x/s*/

/*! scale for velocity_t */
#define Velocity_s Scale_100
/*! minimum for velocity_t */
#define Velocity_min (-32767)
/*! maximum for velocity_t */
#define Velocity_max (32766)

/*! speed
@description   number of revolutions per time unit, forward / left positive
@resolution    0.01
@unit          [1/s]
@min           -327.67
@max           327.66    */
typedef sint16 speed_t; /*%scale:0.01 unit:1/s*/
/*! scale for speed_t */
#define Speed_s Scale_100
/*! minimum for speed_t */
#define Speed_min (-32767)
/*! maximum for speed_t */
#define Speed_max (32766)

typedef sint16 speedometer_speed_t; /*%scale:0.01 unit:km/h|mph*/
/*! scale for speedometer_speed_t */
#define Speedo_speed_s Scale_100
/*! minimum for speedometer_speed_t */
#define Speedo_speed_min (-32767)
/*! maximum for speedometer_speed_t */
#define Speedo_speed_max (32766)

// typedef sint16         factor_t;    /*%scale:0.001*/
/*! scale for factor_t */
#define Factor_s Scale_1000
/*! minimum for factor_t */
/*! Speed_conv_factor_xxx as opposed to what one would assume, these are not
scaled by the factor of 'Speed_s', but rather based on code make the assumption
that it is scaled by 'Factor_s'. Highly misleading, but as this seems to have
been always this way, left as is */
/*! @brief      Speed_conv_factor_kmh  */
#ifndef Speed_conv_factor_mph
#define Speed_conv_factor_mph ((factor_t)2237)
#endif
#ifndef Speed_conv_factor_kmh
/*! @brief      Speed_conv_factor_kmh  */
#define Speed_conv_factor_kmh ((factor_t)3600)
#endif

/*! scale for times_t */
#define Time_s Scale_1000
/*! minimum for times_t */
#define Time_min (0u)
/*! maximum for times_t */
#define Time_max (65535u)

typedef uint8 setspeed_t; /*%unit:km/h|mph*/
/*! scale for setspeed_t */
#define Setspeed_s Scale_1

typedef uint16 cc_setspeed16_t;
/*! scale for setspeed16_t */
#define Setspeed16_s Scale_100

typedef uint8 wheel_impulse_counter_t;
/*! scale for wheel_impulse_counter_t */
#define Wheel_impulse_s Scale_1

typedef uint16 length_t; /*%scale:0.001 unit:m*/
/*! scale for length_t */
#define Length_s Scale_1000

typedef sint16 curve_radius_t; /*%unit:m*/
/*! scale for curve_radius_t */
#define Curve_radius_s Scale_1

// typedef sint16 distance_t;    /*%scale:0.01 unit:m*/
/*! scale for distance_t */
#define Distance_s Scale_100
/*! minimum for distance_t */
#define Distance_min (-32767)
/*! maximum for distance_t */
#define Distance_max (32766)

typedef sint16 torque_t; /*%unit:Nm*/
/*! scale for torque_t */
#define Torque_s Scale_1

typedef uint16 ratio_t; /*%scale:0.01*/
/*! scale for ratio_t */
#define Ratio_s Scale_100

typedef uint16 weight_t; /*%scale:0.1 unit:kg*/
/*! scale for weight_t */
#define Weight_s Scale_10
/*! minimum for weight_t */
#define Weight_min (0)
/*! maximum for weight_t */
#define Weight_max (65535)

typedef sint16 energy_t; /*%unit:Nms*/
/*! scale for energy_t */
#define Energy_s Scale_1

// typedef uint8 confidence_t;      /*%scale:0.392156862745 unit:%*/
/*! scale for confidence_t */
#define Confidence_s (255)
/*! minimum for confidence_t */
#define Confidence_min (0)
/*! maximum for confidence_t */
#define Confidence_max (255)

typedef sint32 angle_t; /*%scale:0.0001 unit:rad*/
/*! scale for angle_t */
#define Angle_s Scale_10000
/*! minimum for angle_t */
#define Angle_min (-314160)
/*! maximum for angle_t */
#define Angle_max (314160)

/*! Former 'Pi' define renamed so that it is properly visible that this is the
fixed-point PI used in 'mat' and 'phys' sub-components */
#define Pi_fixpoint_angle ((angle_t)31416)

typedef sint16 curvature_t; /*%scale:0.00001 unit:1/m*/
/*! scale for curvature_t */
#define Curvature_s Scale_100000
/*! minimum for curvature_t */
#define Curvature_min (-32767)
/*! maximum for curvature_t */
#define Curvature_max (+32766)

// typedef uint8 percentage_t;     /*%unit:%*/
#ifndef Percentage_s
/*! scale for percentage_t */
#define Percentage_s Scale_1
#endif
#ifndef Percentage_min
/*! minimum for percentage_t */
#define Percentage_min (0)
#endif
#ifndef Percentage_max
/*! maximum for percentage_t */
#define Percentage_max (100)
#endif

/*! enum to define labels for driver monitoring Hypothesis types */
typedef enum {
    DIMHypoType_No = 0u, /*!< empty hypothesis*/
    DIMHypoType_Feedback =
        1u, /*!< driver feedback - value -100: strong negative feedback, +100
               strong positive feedback*/
    DIMHypoType_Activity =
        2u, /*!< driver activity - value 0: no activity, 100: high activity*/
    DIMHypoType_Attention = 3u, /*!< driver attention - value -100: distracted,
                                   +100: high attention*/
    DIMHypoType_LaneChangeLeft =
        4u, /*!< driver wants to change the lane to the left - value 0: no lane
               change, 100: high probability*/
    DIMHypoType_LaneChangeRight =
        5u, /*!< driver wants to change the lane to the right - value 0: no lane
               change, 100: high probability*/
    DIMHypoType_Drowsieness =
        6u, /*!< driver is drowsy: 0: low probability, 100: high probability*/
    DIMHypoType_Overtake = 7u, /*!< driver plans to overtake: 0: driver will not
                                  overtake, driver will overtake*/
    DIMHypoType_Distraction = 8u, /*!< driver distraction - value 0: no
                                     distraction, 100: high distraction*/
    DIMHypoType_SportStyle = 9u,  /*!< driver sportstyle - value 0: no
                                     sportstyle, 100: high sportstyle*/
    DIMHypoType_LaneChange =
        10u, /*!< driver changes lane - value -100: langange left, 0: no
                lanechange, 100: langange right*/
    DIMHypoType_Max = 11u /*!< counter for the enum*/
} eDIMHypoType_t;

/*! Error type, currently only used by CD @todo: TBD if needed on global level
 */
typedef enum {
    GDB_ERROR_NONE = 0u,              /*!< no error*/
    GDB_ERROR_POINTER_NULL = 1u,      /*!< null pointer error*/
    GDB_ERROR_FUNC_POINTER_NULL = 2u, /*!< null function pointer error*/
    GDB_ERROR_CAST_VALUE_TO_HIGH =
        3u, /*!< while casting a variable, the value would be truncated because
               it was to high (example (signed char)200)*/
    GDB_ERROR_CAST_VALUE_TO_LOW =
        4u, /*!< while casting a variable, the value would be truncated because
               it was to low  (example (signed char)-200)*/
    GDB_ERROR_UNKNOWN_TYPE = 5u, /*!< enum type handling unknown*/
    GDB_ERROR_FILTER_DOESNT_MATCH =
        6u, /*!< a filter function doesn't match any result*/
    GDB_ERROR_LOW_QUALITY =
        7u, /*!< at least one signal didn't have a sufficient quality*/
    GDB_ERROR_ARRAY_OVERFLOW = 8u, /*!< the array index is out of bounds*/
    GDB_ERROR_ZERO_DEVISION = 9u,  /*!< Devision by Zero requested*/
    GDB_ERROR_VALUE_RANGE = 10u    /*!< a value is outside expeccted range */
} eGDBError_t;

/*! typedef for CD hypothesis data */
typedef struct {
    sint8 Probability;       /*!< -100 -> +100%*/
    percentage_t Confidence; /*!< hypothesis confidence*/
    eDIMHypoType_t eType;    /*!< type of the hypothesis (see eDIMHypoType_t)*/
    eGDBError_t eGDBError; /*!< error type (GDB_ERROR_NONE if module processing
                              was OK) */
} GDB_DMHypothesis_t;

#endif
