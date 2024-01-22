/*
 * \file        $Id: tue_prv_common_object.h 1.2 2017/04/09 11:36:11CEST Martin
 * Cooke (martin.cooke) in_test  $
 *
 *
 *
 *
 * This is a structure definition file that defines a single object
 * as it is seen by the sensor. There is no guarantee that any
 * stTueObject_t object corresponds to a real-life object nor is
 * there a guarantee that each of the real-life objects is represented
 * within an instance of stTueObject_t.
 * Furthermore an object list is defined. Any object list is an array
 * with a constant capacity. The array is filled from the beginning
 * the number of valid elements is indicated by an NumObjects integer
 * value. Any object that is valid according to this definition may
 * still be set to invalid by object quality attribute.
 *
 * Note tue_prv_common_object_defines for commonly used values of
 * certain object attributes.
 *
 * <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 * <br>
 * All rights reserved. Property of Tuerme.<br>
 * Restricted rights to use, duplicate or disclose of this code<br>
 * are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUE_PRV_COMMON_OBJECT_H
#define TUE_PRV_COMMON_OBJECT_H
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"
//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
#include "tue_prv_common_types.h"

/*==================[macros]================================================*/
#define TUE_PRV_COMMON_OBJECT_MAX_OBJECTS (112) /* 80 Radar + 32 Vision */

/* VERSION
 * to be updated by hand if you change any of the the structs
 * please try to make it similar to the mks file version but as an integer
 * number
 * but do not forget to add +1 as the mks version above is updated AFTER your
 * checkin
 * Usage: it could be used for SUB_MEDIA_SUBTYPE checks and for checks in the
 * code
 *        it could also be a member of the structs itself, TBDiscussed */
#define TUE_PRV_COMMON_OBJECT_VERSION_NUMBER (123)

/* Representation of a single object in the vicinity of the ego vehicle. All
 * values in this object
 * structure are estimates on how the sensors perceive the environment of the
 * car. */
typedef struct {
    /* Object Information */
    uint16 u16ID; /* u16ID is an identifier number that allows for tracking of
                     persistent objects over time. 0 is a valid ID. */
    uint16 u16UpdateCounter; /* Counter that is incremented whenever a module
                                outputs an updated object (starting from 0,
                                maximum value is 60.000, counter will overflow
                                to 0 afterwards) */
    uint16 u16Age;     /* Time [ms] that has passed since initialization of
                          underlying track: 0...65000 is 0...65sec; 65001 is more
                          than 65sec (saturation value) */
    sint16 s16Quality; /* Object quality/confidence/probability of existence: 0
                          is "whole object invalid/not existing"; 1...99 is
                          "probability value"; 101 is "quality
                          undecided/unavailable" */
    uint32 u32SensorsCurr; /* Bit pattern that indicates which sensors have seen
                              this object during their last measurements. Each
                              of the first 31 bits has a specific meaning (see
                              defines). */
    uint32 u32SensorsHist; /* Bit pattern that indicates which sensors have seen
                              this object at some point in time. Each of the 31
                              LSB Bits has a specific meaning (see defines). */
    /* Position */
    float32 f32XPos; /* relative target object position [m] in x-direction
                        (front) */
    float32
        f32YPos; /* relative target object position [m] in y-direction (left) */
    float32
        f32ZPos; /* relative target object position [m] in z-direction (up) */
    float32 f32PosVarX;  /* variance (=StdDev? [m�] */
    float32 f32PosVarY;  /* variance (=StdDev? [m�] */
    float32 f32PosVarZ;  /* variance (=StdDev? [m�] */
    float32 f32PosCovXY; /* covariance [m�] */
    float32 f32PosCovXZ; /* covariance [m�] */
    float32 f32PosCovYZ; /* covariance [m�] */
    /* Velocity */
    float32 f32XVel; /* relative target object velocity in x-direction [m/s] */
    float32 f32YVel; /* relative target object velocity in y-direction [m/s] */
    float32 f32ZVel; /* relative target object velocity in z-direction [m/s] */
    float32 f32VelVarX;  /* variance (=StdDev? [m?s�] */
    float32 f32VelVarY;  /* variance (=StdDev? [m?s�] */
    float32 f32VelVarZ;  /* variance (=StdDev? [m?s�] */
    float32 f32VelCovXY; /* covariance [m?s�] */
    float32 f32VelCovXZ; /* covariance [m?s�] */
    float32 f32VelCovYZ; /* covariance [m?s�] */
    /* Acceleration */
    float32
        f32XAcc; /* relative target object acceleration in x-direction [m/s�] */
    float32
        f32YAcc; /* relative target object acceleration in y-direction [m/s�] */
    float32
        f32ZAcc; /* relative target object acceleration in z-direction [m/s�] */
    float32 f32AccVarX;  /* variance (=StdDev? [m?s^4] */
    float32 f32AccVarY;  /* variance (=StdDev? [m?s^4] */
    float32 f32AccVarZ;  /* variance (=StdDev? [m?s^4] */
    float32 f32AccCovXY; /* covariance [m?s^4] */
    float32 f32AccCovXZ; /* covariance [m?s^4] */
    float32 f32AccCovYZ; /* covariance [m?s^4] */
    /* Geometry */
    float32 f32Width;      /* [m] */
    float32 f32WidthStd;   /* standard deviation [m] */
    float32 f32Length;     /* [m] */
    float32 f32LengthStd;  /* standard deviation [m] */
    float32 f32Height;     /* [m] */
    float32 f32HeightStd;  /* standard deviation [m] */
    float32 f32Orient;     /* orientation of the front of the target w.r.t. the
                              front of the ego vehicle [rad] */
    float32 f32OrientStd;  /* standard deviation [rad] */
    float32 f32OrientRate; /* rotation rate of object [rad/s] */
    float32 f32OrientRateStd; /* standard deviation [rad/s] */
    uint16 u16RefPoint; /* CoG (Center of Gravity), nearest corner,...  ToDo:
                           define this! */
    uint16 u16Spacer1;  /* padding field to ensure 32bit alignment; shall always
                           be set to 0x00 */
    /* Object Definition */
    float32
        f32DistEgoLane; /* distance of object to ego-lane in y-direction [m] */
    float32 f32DistEgoLaneStd; /* standard deviation [m] */
    sint8 s8LaneAssign; /* Lane assignment relative to ego lane; 32 is default
                           invalid/undecided; 0 is ego lane; +1 is one lane to
                           the left, +2 is two lanes to the left, (...); -1 is
                           one lane to the right, -2 is two lanes to the right,
                           (...) */
    sint8 s8LaneAssignQuality; /* Object quality/confidence/probability of
                                  existence: 0 is [default value, shall not be
                                  used]; 1...100 is a probability value; 101 is
                                  "quality undecided/unavailable" */
    uint16 u16MotionType;      /* stationary, moving, stopped,... (find the
                                  definition in: tue_prv_common_object_defines.h) */
    uint16 u16Class; /* Object Class: vehicle, two-wheeler, pedestrian,... (find
                        the definition in: tue_prv_common_object_defines.h) */
    uint16 u16SensorError; /* 0: side-lobe, 1: mirror object, 2: shadow,...
                              ToDo: define this! */
    float32 f32Ttc;        /* time [s] to collision = TTI (time to impact) */
    float32 f32Thw;        /* time [s] headway */
} stTueObject_t;

/* Variable-sized list of objects that a single object source (e.g. a sensor,
 * the fusion, ...)
 * sees in the vicinity of the ego vehicle. All objects are valid at the same
 * point in time
 * (indicated by f32MeasurementLatency).
 * The capacity of this list is limited by TUE_PRV_COMMON_STRUCTS_MAX_OBJECTS.
 */
typedef struct {
    uint16
        u16StructVersion; /* Version number of the structure definition. Needs
                             to be set to TUE_PRV_COMMON_OBJECT_VERSION_NUMBER
                             especially when dealing with recordings */
    uint16 u16ListUpdateCounter; /* Counter that is incremented whenever a
                                    module outputs an updated object (starting
                                    from 0, maximum value is 60.000, counter
                                    will overflow to 0 afterwards) */
    float32
        f32MeasurementLatency; /* Common age [s] of all object estimations
                                  within the data array: positive latencies
                                  indicate outdated information; negative
                                  latencies indicate predicted object data. */
    uint16 u16NumObjects;      /* Only the first NumObjects elements in the data
                                  array contain information */
    uint16 u16ListSpacer1;     /* padding field to ensure 64bit alignment; shall
                                  always be set to 0x00 */
    uint32 u32ListSpacer1;     /* padding field to ensure 64bit alignment; shall
                                  always be set to 0x00 */
    stTueObject_t aObject[TUE_PRV_COMMON_OBJECT_MAX_OBJECTS]; /* object array
                                                                 with limited
                                                                 capacity */
} stTueObjectList_t;

#endif /* TUE_PRV_COMMON_OBJECT_H */