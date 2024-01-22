/** \addtogroup config
 *  @{ */
//* \file        TueObjFusn_ConfigVehicle.h
//*
//*
//*
//* <br>=====================================================<br>
//* <b>Copyright 2016 by Tuerme.</b>
//*
//*  All rights reserved. Property of Tuerme.<br>
//*  Restricted rights to use, duplicate or disclose of this code<br>
//*  are granted through contract.
//* <br>=====================================================<br>
//*/

#ifndef TUEOBJFUSN_CONFIGVEHICLE_H
#define TUEOBJFUSN_CONFIGVEHICLE_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
/*==================[macros]================================================*/
/**
 * Define maximum number of sensor input object lists.
 * strongly affects memory footprint of certain fusion modes
 */
#define TUE_PRV_FUSION_MAX_INPUTS (6u)  // 5R1V

/**
 * maximum number of objects from any of the radar sensors
 */
#define TUE_PRV_FUSION_MAX_OBJECTS_RADAR (128u)
#define TUE_PRV_FUSION_MAX_OBJECTS_RADAR_LEFT_FRONT (12u)
#define TUE_PRV_FUSION_MAX_OBJECTS_RADAR_LEFT_REAR (16u)
#define TUE_PRV_FUSION_MAX_OBJECTS_RADAR_RIGHT_FRONT (12u)
#define TUE_PRV_FUSION_MAX_OBJECTS_RADAR_RIGHT_REAR (16u)

// the maximum number of objects from camera pedestrian structure
#define TUE_PRV_CAMERA_PEDESTRIAN_MAX_OBJECTS (7u)

// the maximum number of objects from camera other car structure
#define TUE_PRV_CAMERA_OTHER_CAR_MAX_OBJECTS (16u)

// the maximum number of used other car number for camera objects
#define TUE_PRV_CAMERA_USED_OTHER_CAR_MAX_OBJECTS (5u)

// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/31 changed by
// changchang 20200603 start
// the maximum number of used pedestrian number for pedestrian objects
#define TUE_PRV_CAMERA_USED_PEDESTRIAN_MAX_OBJECTS (7u)
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/31 changed by
// changchang 20200603 end

/**
 * maximum number of objects from any of the vision sensors
 */
#define TUE_PRV_FUSION_MAX_OBJECTS_VISION \
    (20u) /* \todo: set to 14! */  // change to 14 from 24 by guotao 20190508

/**
 * number of buffer items for ego motion storage.
 * Ego motion data is buffered at the fusion cycle rate (usually 100Hz).
 * The buffer must be large enough to store ego motion for at least the
 * duration of maximum expected input measurement latency
 */
#define TUEOBJFUSN_EGOMOTIONHISTORY_SIZE (30u)  // TODO what is a good value

/**
 * High ego acceleration and deceleration result in an increased uncertainity
 * of the
 * longitudinal acceleration measured by the sensor.
 * To reflect this uncertainity, the acceleration variance calculated by the
 * sensor model shall be
 * corrected using the increased uncertainity caused by the acceleration and
 * deceleration of the ego vehicle.
 * This is done using a linear model with offset and slope
 */
#define TUEOBJFUSN_EGOMOTION_VARIANCE_ACC_SLOPE (0.65f)

/**
 * High yaw rate of the ego vehicle causes a higher uncertainity in
 * the lateral velocity. This is especially true for statioary radar targets
 */
#define TUEOBJFUSN_EGOMOTION_VARIANCE_YAW_SLOPE (0.0f)

/**
 * distance from front bumper to rear axle of the vehicle
 */
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/5 changed by guotao
// 20200422 start
//#ifdef FUSION_SIMULATION_VS_CONF
//#define TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_FRONT_BUMBER  (0.0f)
//#else
//#define TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_FRONT_BUMBER  (3.455f)
//#endif
#define TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_FRONT_BUMBER (0.0f)
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/5 changed by guotao
// 20200422 end
/**
 * Offset of camera to front bumber, i.e. front bumber is xx m in front of MVS
 */
#define TUE_PRV_VEHICLE_CAM_TO_FRONT_BUMBER (1.59723f)

/**
 * Lateral offset of MVS sensor to x-axis of front bumber coordinate system
 * Identical to offset to rear axle coordinate system. Offset to driver side
 */
#define TUE_PRV_VEHICLE_LATERAL_OFFSET_CAM (-0.017f)

/**
 * Longitudinal value of origin of camera coordinate system in rear axle
 * coordinate system
 */
#define TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_CAMERA                          \
    (TUE_PRV_VEHICLE_DISTANCE_REAR_AXLE_TO_FRONT_BUMBER -                     \
     TUE_PRV_VEHICLE_CAM_TO_FRONT_BUMBER) /*1 .730 is from front bumper to    \
                                             camera -> distance is approx. 2m \
                                             in front of rear axle */

/**
 * Buffer Size used in error management for the ring buffer implementation
 */

#define TUE_PRV_ERROR_MANAGEMENT_BUFFER_SIZE (5u)

/*
 * A sensor timeout is assumed as soon as the last sensor update is older than
 * specified
 */
#define TUE_PRV_VEHICLE_SENSOR_TIMEOUT (0.300f)

#ifdef ALGO_SAFETY_DISTANCE_FOR_TEST_ONLY
#define ALGO_EXTEND_SAFETY_DISTANCEX_FOR_TEST (15.0f)
#endif

/*==================[type definitions]======================================*/
/*==================[functions]============================================*/
/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUEOBJFUSN_CONFIGVEHICLE_H */
       /**@}==================[end of
        * file]===========================================*/
