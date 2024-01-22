#ifndef TUEOBJFUSN_TRACKABLETYPE_H_
#define TUEOBJFUSN_TRACKABLETYPE_H_

#include "tue_prv_common_types.h"
#include "tue_prv_common_matrix.h"
#include "TueObjFusn_ConfigVehicle.h"

/** Representation of a single track and object.  All values in this object are
 * necessary for the
 * the Kalman filter or are representative of object properties.
 * The trackable is an internal structure to store all necessary processing data
 */
typedef struct TueObjFusn_TrackableTypeTag {
    /* Object Information */
    boolean bUpdated;         /** Updated in current cycle */
    uint8 u8CoordSystem;      /** Indicates in which coordinate system the ego
                                 position lives */
    uint8 u8TrkblBinPosition; /** Indicate the objects bin position for object
                                 selection */
    uint8
        u8VisionIdx; /** vision idx for vision ID mapping in output converter */

    uint16 u16ID;  /** u16ID is an identifier number that allows for tracking of
                      persistent objects over time. 0 is a valid ID. */
    uint16 u16Age; /** Time [ms] that has passed since initialization of
                      underlying track: 0...65000 is 0...65sec; 65001 is more
                      than 65sec (saturation value) */
    uint16 u16Lifespan; /** Lifespan of trackable to handle coasting and to see
                           if trackable is valid or not */
    uint16 u16RefPoint; /** Indicates which point of the target object is given
                           by the coordinates */
    uint8 u8CyclesNoVision; /** number of consequitve vision updates without
                               confirmation of this object -> reset to 0 if
                               object is updated */
    uint8 u8CyclesNoRadar;  /** number of consequitve radar updates without
                               confirmation of this object -> reset to 0 if
                               object is updated */

    uint16 au16SensorID[TUE_PRV_FUSION_MAX_INPUTS]; /** u16ID of the sensor
                                                       input object. 0 is a
                                                       valid ID. The order must
                                                       always map to the sensor
                                                       pattern array in
                                                       trackabListUtils*/
    uint16 au16SensorIDLast
        [TUE_PRV_FUSION_MAX_INPUTS]; /** u16ID of the sensor nput object from
                                        the last update. 0 is a valid ID. The
                                        order must always map to the sensor
                                        pattern array in trackabListUtils*/

    uint32 u32SensorsCurr; /** Bit pattern that indicates which sensors have
                              seen this object during their last measurements.
                              Each of the first 31 bits has a specific meaning
                              (see defines). */
    uint32 u32SensorsHist; /** Bit pattern that indicates which sensors have
                              seen this object at some point in time. Each of
                              the 31 LSB Bits has a specific meaning (see
                              defines). */

    /* Kinematics */
    stf32Vec_t vecX; /** state vector from Kalman filter including position,
                        velocity, acceleration */

    stf32SymMatrix_t matP; /** P matrix from Kalman filter including variances
                              and covariances of all states */

    float32 f32Gain;    /** camera gain for this object */
    float32 f32GainVar; /** Variance of camera gain for this object */

    /* Coordinated Turn Model */
    float32 f32YawRate;             /** Yaw rate of target vehicle */
    float32 f32YawRateVar;          /** Variance of yaw rate for this object */
    float32 f32Heading;             /** Heading for this object */
    float32 f32HeadingVar;          /** Variance of heading for this object */
    float32 f32CovarHeadingYawRate; /** Covariance of heading and yaw rate for
                                       this object */

    /* Qualities */
    float32 f32ObstacleProbability; /** Track Quality **/
    float32 f32ExistenceQuality;    /** Existence Quality **/
    float32 fRCS;  // set RCS value for radar, new value for the tuerme
    uint8 eObjMaintenanceState;  // set eObjMaintenanceState value for radar,
                                 // new value for the tuerme

    /* Geometry */
    float32 f32Width;  /** [m] */
    float32 f32Length; /** [m] */
    float32 f32Height; /** [m] */

    /* Object Definition */
    uint16 u16MotionType; /** stationary, moving, stopped,... (find the
                             definition in: TueObjFusn_ObjListInputConstants.h)
                             */
    uint16 u16Class;      /** Object Class: vehicle, two-wheeler, pedestrian,...
                             (find the definition in:
                             TueObjFusn_ObjListInputConstants.h) */
    uint16 u16ClassProb;  /** classification probability (0 is [default]; 1...99
                             is "probability value"; 101 is "probability
                             undecided/unavailable" */

    uint8 u8RadarMotionTypeInput;  // get motion type from radar input, new
                                   // value for the tuerme. set value from enum
                                   // TueFusion_SensorInputMotion
    /*¡¡customer requirement */
    boolean bCIPVFlag; /** if the object is the key car */
    boolean bMCPFlag;  /** if the object is the key ped */
    uint16 uFusionSts; /** object fusion status : fusion /camera / radar */
    uint16 uIDrecord;  /** the record the id of fisrt detected the object*/
    uint16 uLifeCycle; /** the life cycle of the object */
} TueObjFusn_TrackableType;

typedef enum {
    TU_SENSOR_MOTIONG_TYPE_INPUT_UNKNOWN,
    TU_SENSOR_MOTIONG_TYPE_INPUT_STATIONARY,
    TU_SENSOR_MOTIONG_TYPE_INPUT_MOVING,
    TU_SENSOR_MOTIONG_TYPE_INPUT_STOP
} TueFusion_RadarInputMotion;
#endif /**\} TUEOBJFUSN_TRACKABLETYPE_H_ */
