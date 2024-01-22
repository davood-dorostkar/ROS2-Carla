/** \addtogroup tueFusion
 *  \{
 * \file TUEOBJFUSN_PARAMS.h
 * \brief Fusion parameters
 *
 * Comments:
 *
 * There are several options for parameter handling in Simulink (as far as we
 * know):
 *   - Block mask: enter parameters in a block mask, they will be passed to the
 * C code through the SimStruct
 *   - Bus: Pass parameters through a bus
 *
 * Block mask:
 *   - Parameters can be edited manually via block mask
 *   - Different handling of parameters set before startup and runtime
 * parameters
 *   - Parameter passing to C can be implemented manually or with the S-function
 * generator
 *   - Manual implementation via SimStruct looks tedious
 *   - Generated code is very naïve (copying each value in the struct with a
 * separate memcopy) and cannot handle lists
 *
 * Bus:
 *   - Block needs an additional input for the parameter bus (similar to TUE
 * object struct/list)
 *   - Bus creation can be done manually in Simulink
 *   - No (easy) separation between startup and runtime parameters
 *   - Parameters could be read from file and fed to fusion module via the bus
 * input
 *
 * General notes:
 *   - Independent from any chosen parameter concept we need to map parameters
 * from Simulink to our C fusion module.
 * Either we parse them out of the SimStruct (block mask option) and pass them
 * through the S-function wrapper to
 * our module or we use the bus concept (similar handling of parameters as
 * already done for the TUE object struct list).
 *   - Thus, we can choose our parameter concept for the fusion module
 * independently from Simulink since we need to
 * implement a conversion anyway. All three concepts can be realized/combined
 * with Simulink with help of this conversion
 * (as far as we understood the Simulink parameter handling for now).
 *
 *
 *
 * This is a structure definition file that defines parameters for the fusion
 * module.
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef TUEOBJFUSN_PARAMS_H
#define TUEOBJFUSN_PARAMS_H

#include "tue_prv_common_types.h"

/***************/
/* Sensor mode */
/***************/
/** @name Sensor mode macros
 * Sensor mode  parameters
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_U32SENSORMODE_DEFAULT (0x7FFFFFFFu)
/** minimal value */
#define TUEOBJFUSN_PARAMS_U32SENSORMODE_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_PARAMS_U32SENSORMODE_MAX (0x7FFFFFFFu)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_U32SENSORMODE_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_U32SENSORMODE_DESCR                            \
    ("Decodes which sensors will be used in the fusion module (using a " \
     "sensor pattern)")
/** \}*/

/*******************/
/* Match Gate      */
/*******************/
/** @name Match Gate macros
 * Match Gate parameters
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_F32MATCHGATE_DEFAULT (12.0f)
/** minimum value */
#define TUEOBJFUSN_PARAMS_F32MATCHGATE_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_PARAMS_F32MATCHGATE_MAX (FLT_MAX)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_F32MATCHGATE_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_F32MATCHGATE_DESCR ("Gate for matching")
/** \}*/

/*******************/
/* Process Noise in x */
/*******************/
/** @name Post Prediction macros
 * Process Noise of the highest derivation in x in the
 Wiener-sequence-acceleration model
 * depends on value of bAccelerationX
 * False: constant velocity model - process noise specifies acceleration
 variance (constant velocity model)
 * TRUE: constant velocity model - process noise specifies jerk variance
 (constant acceleration model)
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_DEFAULT (1.0f)
/** minimum value */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_MAX (FLT_MAX)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINXFORQ_DESCR \
    ("Variance in X (highest derivation) for Q matrix in case of Pedestrians")
/** \}*/

/*******************/
/* Process Noise in y */
/*******************/
/** @name Post Prediction macros
 * Process Noise of the highest derivation in y in the
 Wiener-sequence-acceleration model
 * depends on value of bAccelerationY
 * False: constant velocity model - process noise specifies acceleration
 variance (constant velocity model)
 * TRUE: constant velocity model - process noise specifies jerk variance
 (constant acceleration model)
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_DEFAULT (1.0f)
/** minimum value */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_MAX (FLT_MAX)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_F32PEDESTRIANVARIANCEINYFORQ_DESCR \
    ("Variance in Y (highest derivation) for Q matrix in case of Pedestrians")
/** \}*/

/**********************/
/* Process Noise in x */
/**********************/
/** \@name Post Prediction macros
 * Process Noise of the highest derivation in x in the
 Wiener-sequence-acceleration model
 * depends on value of bAccelerationX
 * False: constant velocity model - process noise specifies acceleration
 variance (constant velocity model)
 * TRUE: constant velocity model - process noise specifies jerk variance
 (constant acceleration model)
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINXFORQ_DEFAULT (1.0f)
/** minimum value */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINXFORQ_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINXFORQ_MAX (FLT_MAX)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINXFORQ_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINXFORQ_DESCR \
    ("Variance in X (highest derivation) for Q matrix in case of vehicles")
/** \}*/

/*******************/
/* Process Noise in y */
/*******************/
/** \name Post Prediction macros
 * Process Noise of the highest derivation in y in the
 Wiener-sequence-acceleration model
 * depends on value of bAccelerationY
 * False: constant velocity model - process noise specifies acceleration
 variance (constant velocity model)
 * TRUE: constant velocity model - process noise specifies jerk variance
 (constant acceleration model)
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINYFORQ_DEFAULT (1.0f)
/** minimum value */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINYFORQ_MIN (0.0f)
/** maximum value */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINYFORQ_MAX (FLT_MAX)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINYFORQ_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_F32VEHICLEVARIANCEINYFORQ_DESCR \
    ("Variance in Y (highest derivation) for Q matrix in case of vehicles")
/** \}*/

/*************************/
/* Kinematic Transformation */
/*************************/
/** @name kinematic transformation in x macro
 * Bool which enables to output in overground metric
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_DEFAULT (FALSE)
/** minimum value */
#define TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_MIN (FALSE)
/** maximum value */
#define TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_MAX (TRUE)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_BOUTPUTISOVERGROUND_DESCR \
    ("Bool which enables to output in overground metric")
///\}

/** @name Enables and disables a track merging algorithm
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_BUSETRACKMERGE_DEFAULT (FALSE)
/** minimum value */
#define TUEOBJFUSN_PARAMS_BUSETRACKMERGE_MIN (FALSE)
/** maximum value */
#define TUEOBJFUSN_PARAMS_BUSETRACKMERGE_MAX (TRUE)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_BUSETRACKMERGE_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_BUSETRACKMERGE_DESCR \
    ("Enables and disables a track merging algorithm")
///\}

/** @name Enables and disables a coasting algorithm
 \{**/
/** Default value */
#define TUEOBJFUSN_PARAMS_BUSECOASTING_DEFAULT (FALSE)
/** minimum value */
#define TUEOBJFUSN_PARAMS_BUSECOASTING_MIN (FALSE)
/** maximum value */
#define TUEOBJFUSN_PARAMS_BUSECOASTING_MAX (TRUE)
/** flag indicating if this field is an enumeration */
#define TUEOBJFUSN_PARAMS_BUSECOASTING_ISENUM (0)
/** Brief description string */
#define TUEOBJFUSN_PARAMS_BUSECOASTING_DESCR \
    ("Enables and disables a coasting algorithm")
///\}

/** a structure for holding all fusion parameters */
typedef struct TueObjFusn_ParametersTypeTag {
    uint32 u32SensorMode;
    float32 f32MatchGate;
    float32 f32PedestrianVarianceInXForQ;
    float32 f32PedestrianVarianceInYForQ;
    float32 f32VehicleVarianceInXForQ;
    float32 f32VehicleVarianceInYForQ;

    boolean bOutputIsOverground;

    /*
    This parameter enables track merge
    */
    boolean bUseTrackMerge;
    /*
    This parameter enables coasting
    */
    boolean bUseCoasting;

} TueObjFusn_ParametersType;
#endif  ///\} TUEOBJFUSN_PARAMS_H
