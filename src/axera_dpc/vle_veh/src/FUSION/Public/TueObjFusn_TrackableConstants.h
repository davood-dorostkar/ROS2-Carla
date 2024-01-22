/** \addtogroup objectlist
 *  @{
 * \file       TueObjFusn_ObjListInputConstants.h
 * \brief  This is is a list of preprocessor definitions to be used together
 * with TueObjFusn_ObjListInput.h
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2014 by Tuerme.</b>
 * <br>
 * All rights reserved. Property of Tuerme.<br>
 * Restricted rights to use, duplicate or disclose of this code<br>
 * are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUEOBJFUSN_TRACKABLELISTCONSTANTS_H_
#define TUEOBJFUSN_TRACKABLELISTCONSTANTS_H_

/******************/
/* u32SensorsCurr */
/* u32SensorsHist */
/******************/
/** 00000000 00000000 00000000 00000001: radar on the front left */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_LEFT (0x00000001u)
/** 00000000 00000000 00000000 00000010: radar centered on the front bumper */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER (0x00000002u)
/** 00000000 00000000 00000000 00000100: radar on the front right */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_RIGHT (0x00000004u)
/** 00000000 00000000 00000000 00001000: radar on the right side of the car */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_MIDDLE_RIGHT (0x00000008u)
/** 00000000 00000000 00000000 00010000: radar on the rear right */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_RIGHT (0x00000010u)
/** 00000000 00000000 00000000 00100000: radar centered on the rear bumper */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_CENTER (0x00000020u)
/** 00000000 00000000 00000000 01000000: radar on the rear left */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_LEFT (0x00000040u)
/** 00000000 00000000 00000000 10000000: radar on the left side of the car */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_MIDDLE_LEFT (0x00000080u)
/** 00000000 00000000 00000001 00000000: [reserved] */
/** 00000000 00000000 00000010 00000000: [reserved] */
/** 00000000 00000000 00000100 00000000: forward looking mono vision system */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_FRONT (0x00000400u)
/** 00000000 00000000 00001000 00000000: mono vision system on the right side of
 * the car*/
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_RIGHT (0x00000800u)
/** 00000000 00000000 00010000 00000000: backwards looking mono vision system */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_REAR (0x00001000u)
/** 00000000 00000000 00100000 00000000: mono vision system on the left side of
 * the car*/
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_LEFT (0x00002000u)
/** 00000000 00000000 01000000 00000000: forward looking stereo vision system */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS_FRONT (0x00004000u)
/** 00000000 00000000 10000000 00000000: stereo vision system on the right side
 * of the car*/
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS_RIGHT (0x00008000u)
/** 00000000 00000001 00000000 00000000: backwards looking stereo vision system
 */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS_REAR (0x00010000u)
/** 00000000 00000010 00000000 00000000: stereo vision system on the left side
 * of the car*/
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS_LEFT (0x00020000u)
/** 00000000 00000100 00000000 00000000: forward looking night vision system */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_NV_FRONT (0x00040000u)
/** 00000000 00001000 00000000 00000000: night vision system on the right side
 * of the car*/
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_NV_RIGHT (0x00080000u)
/* 00000000 00010000 00000000 00000000: backwards looking night vision system */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_NV_REAR (0x00100000u)
/* 00000000 00100000 00000000 00000000: night vision system on the left side of
 * the car*/
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_NV_LEFT (0x00200000u)
/* 00000000 01000000 00000000 00000000: [reserved] */
/* 00000000 10000000 00000000 00000000: [reserved] */
/* 00000001 00000000 00000000 00000000: [reserved] */
/* 00000010 00000000 00000000 00000000: [reserved] */
/* 00000100 00000000 00000000 00000000: [reserved] */
/* 00001000 00000000 00000000 00000000: [reserved] */
/* 00010000 00000000 00000000 00000000: [reserved] */
/* 00100000 00000000 00000000 00000000: [reserved] */
/* 01000000 00000000 00000000 00000000: [reserved] */

/* Combinations of sensors */
/** any radar sensor */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR             \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_LEFT |   \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_RIGHT |  \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_MIDDLE_RIGHT | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_RIGHT |   \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_CENTER |  \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_LEFT |    \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_MIDDLE_LEFT)
/** any MVS */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS      \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_FRONT | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_RIGHT | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_REAR |  \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_LEFT)
/** any SVS */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS      \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS_FRONT | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS_RIGHT | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS_REAR |  \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS_LEFT)
/** any NV */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_NV      \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_NV_FRONT | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_NV_RIGHT | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_NV_REAR |  \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_NV_LEFT)
/** and Vision */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_VISION                                  \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS | TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS | \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_NV)

/** any camera */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS | TUEOBJFUSN_TRACKABLE_U32SENSOR_SVS)

#define TUEOBJFUSN_TRACKABLE_U32SENSOR_INVALID (0x00000000u)

/** Same pattern is used to identify coasted data */
#define TUEOBJFUSN_TRACKABLE_U32SENSOR_COASTED (0x00000000u)

#define TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_VISION \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR |         \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA)

// this bit cannot be used for a sensor since it decodes the default value for
// the u32SensorsCurr and the u32SensorsHist fields!
/* 10000000 00000000 00000000 00000000: [reserved] */

/*****************/
/* u16MotionType */
/*****************/
/* motion type could not be determined */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN (0x0000u)
/* motion type is moving traffic in the direction of ego driving. Target going
 * forward */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING (0x0111u)
/* motion type is moving traffic in the direction of ego driving. Target not
 * moving at the moment*/
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_STOPPED (0x0112u)
/* motion type is moving traffic in the direction of ego driving. Target
 * reversing */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_REVERSING \
    (0x0113u)
/* motion type is moving traffic in the opposite direction of ego driving */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_ONCOMING (0x0120u)
/* motion type is crossing traffic with respect to the ego driving direction */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING (0x0130u)
/* motion type is not moving and it will most likely stay like this */
#define TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY (0x0200u)

/************/
/* u16Class */
/************/
/* the object class could be determined */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN (0x0000u)
/* the object is no traffic participant but cannot be further classified */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_OBSTACLE_UNKNOWN (0x0100u)
/* the object is an obstacle that may neither be overrun nor be underrun  (like
 * trees, ...)*/
#define TUEOBJFUSN_TRACKABLE_U16CLASS_OBSTACLE_PATHBLOCKING (0x0110u)
/* the object is an obstacle that may be overrun (like cans, drain covers, ...*/
#define TUEOBJFUSN_TRACKABLE_U16CLASS_OBSTACLE_OVERRUNNABLE (0x0120u)
/* the object is an obstacle that may be underrun (like bridges, highway traffic
 * signs, ...)*/
#define TUEOBJFUSN_TRACKABLE_U16CLASS_OBSTACLE_UNDERRUNNABLE (0x0130u)
/* the object is a vehicle that cannot be further classified */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_UNKNOWN (0x0200u)
/* the object is a car */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_CAR (0x0210u)
/* the object is a truck */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_TRUCK (0x0220u)
/* the object is a trailer (the car or the truck pulling the trailer is a
 * separate object) */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_TRAILER (0x0230u)
/* the object is a 2 wheeler that cannot be further classified */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_UNKNOWN (0x0400u)
/* the object is a bicycle */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_BICYCLE (0x0410u)
/* the object is a motor cycle */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_MOTORCYCLE (0x0420u)
/* the object is a pedestrian that cannot be further classified */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_UNKNOWN (0x0800u)
/* the object is a child */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_CHILD (0x0810u)
/* the object is an adult */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_ADULT (0x0820u)
/* the object is a animal that cannot be further classified */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_ANIMAL_UNKNOWN (0x1000u)
/* the object is a large animal that poses a significant threat to traffic
 * safety (e.g. Alces alces) */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_ANIMAL_LARGE (0x1010u)
/* the object is a small animal that may be overrun in otherwise critical
 * situations */
#define TUEOBJFUSN_TRACKABLE_U16CLASS_ANIMAL_SMALL (0x1020u)

/***************/
/* u16RefPoint */
/***************/
/* The target has no dimensions and no orientation. it's just a point. */
#define TUEOBJFUSN_TRACKABLE_U16REFPOINT_POINT_OBJECT (0u)
/* The center point of the cuboid defined by length, width (& height).
 * Orientation is given */
#define TUEOBJFUSN_TRACKABLE_U16REFPOINT_CENTROID (1u)
/* The target is seen from the front or rear. The point provided (position,
 * velocities, ...)
 * lies at the center of the line closer to the ego vehicle (front or rear).
 * Width & orientation are reliably given, length may be provided. */
#define TUEOBJFUSN_TRACKABLE_U16REFPOINT_CENTER_OF_WIDTH (2u)

/* Maps to indices in vector and regular matrix structure */
#define TRACKABLE_POSX (0u)
#define TRACKABLE_POSY (1u)
#define TRACKABLE_VELX (2u)
#define TRACKABLE_VELY (3u)
#define TRACKABLE_ACCX (4u)
#define TRACKABLE_ACCY (5u)

/**
 * Symmetrix Matrix is stored as lower triangular matrix with the following
 * mapping
 *
 *        x   y  vx vy ax ay
 *  x   ( 0                  )
 *  y   ( 1   2              )
 *  vx  ( 3   4  5           )
 *  vy  ( 6   7  8  9        )
 *  ax  (10  11 12 13 14     )
 *  ay  (15  16 17 18 19  20 )
 *
 */

/* Maps to indices in symmetrix matrix structure */
#define TRACKABLE_INDEX_VARIANCE_POSX (0u)
#define TRACKABLE_INDEX_VARIANCE_POSY (2u)
#define TRACKABLE_INDEX_VARIANCE_VELX (5u)
#define TRACKABLE_INDEX_VARIANCE_VELY (9u)
#define TRACKABLE_INDEX_VARIANCE_ACCX (14u)
#define TRACKABLE_INDEX_VARIANCE_ACCY (20u)

#define TRACKABLE_INDEX_COVARIANCE_POSX_POSY (1u)
#define TRACKABLE_INDEX_COVARIANCE_POSX_VELX (3u)
#define TRACKABLE_INDEX_COVARIANCE_POSX_VELY (6u)
#define TRACKABLE_INDEX_COVARIANCE_POSX_ACCX (10u)
#define TRACKABLE_INDEX_COVARIANCE_POSX_ACCY (15u)

#define TRACKABLE_INDEX_COVARIANCE_POSY_VELX (4u)
#define TRACKABLE_INDEX_COVARIANCE_POSY_VELY (7u)
#define TRACKABLE_INDEX_COVARIANCE_POSY_ACCX (11u)
#define TRACKABLE_INDEX_COVARIANCE_POSY_ACCY (16u)

#define TRACKABLE_INDEX_COVARIANCE_VELX_VELY (8u)
#define TRACKABLE_INDEX_COVARIANCE_VELX_ACCX (12u)
#define TRACKABLE_INDEX_COVARIANCE_VELX_ACCY (17u)

#define TRACKABLE_INDEX_COVARIANCE_VELY_ACCX (13u)
#define TRACKABLE_INDEX_COVARIANCE_VELY_ACCY (18u)

#define TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY (19u)

/* Maps to variances and covariances in symmetric matrix structure */

/**************/
/* u8CoordSystem */
/**************/
/* origin of cooridinate system is at ego front bumper and measured values are
 * relative */
#define TUEOBJFUSN_U8COORDSYSTEM_FRONTBUMP_RELATIVE (0x00u)
/* origin of cooridinate system is at ego front bumper and measured values are
 * overground */
#define TUEOBJFUSN_U8COORDSYSTEM_FRONTBUMP_OVERGROUND (0x01u)
/* origin of cooridinate system is at ego rear axis and measured values are
 * relative */
#define TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_RELATIVE (0x10u)
/* origin of cooridinate system is at ego rear axis and measured values are
 * overground */
#define TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_OVERGROUND (0x11u)
/* origin of cooridinate system is at sensor position and measured values are
 * relative */
#define TUEOBJFUSN_U8COORDSYSTEM_SENSOR_RELATIVE (0x20u)
/* origin of cooridinate system is at sensor position and measured values are
 * overground */
#define TUEOBJFUSN_U8COORDSYSTEM_SENSOR_OVERGROUND (0x21u)
/* the coordinate system is global  */
#define TUEOBJFUSN_U8COORDSYSTEM_GLOBAL (0x40u)

/**************/
/* u16Lifespan */
/**************/
/* no track */
#define TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FREE_SLOT (0u)
/* coasted track */
#define TUEOBJFUSN_TRACKABLE_U16LIFESPAN_COASTED \
    (TUE_PRV_COASTING_LIFESPAN + 1u)
/* track has to die */
#define TUEOBJFUSN_TRACKABLE_U16LIFESPAN_DEAD \
    (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FREE_SLOT + 1u)
/* new track */
#define TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW \
    (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_DEAD + 1u)

// set camera object rcs value according to the object class
// car from rear	20 dBm2
// truck from rear	40 dbm2
// motorbike	10 dbm2
// bicycle		5 dbm2
// pedastrian	0 dbm2

#define CAMERA_OBJECT_RCS_VALUE_TYPE_CAR 20
#define CAMERA_OBJECT_RCS_VALUE_TYPE_TRUCK 40
#define CAMERA_OBJECT_RCS_VALUE_TYPE_MOTORBIKE 10
#define CAMERA_OBJECT_RCS_VALUE_TYPE_BICYCLE 5
#define CAMERA_OBJECT_RCS_VALUE_TYPE_PEDESTRIAN 0

// other car location data buffer size for speed calculating needed
#define CAMERA_OTHER_CAR_LOCATION_BUFFER_SIZE 20

/* definition of classification dependent minimum, maximum and default
 * dimensions */
/* class car minimum, maximum and default dimensions */
#define FCT_SEN_CLASS_CAR_MIN_LENGTH 3.0f
#define FCT_SEN_CLASS_CAR_MAX_LENGTH 7.0f
#define FCT_SEN_CLASS_CAR_DEFAULT_LENGTH 5.0f
#define FCT_SEN_CLASS_CAR_MIN_WIDTH 1.8f
#define FCT_SEN_CLASS_CAR_MAX_WIDTH 2.2f
#define FCT_SEN_CLASS_CAR_DEFAULT_WIDTH 1.8f
/* class truck minimum, maximum and default dimensions */
#define FCT_SEN_CLASS_TRUCK_MIN_LENGTH 5.0f
#define FCT_SEN_CLASS_TRUCK_MAX_LENGTH 25.0f
#define FCT_SEN_CLASS_TRUCK_DEFAULT_LENGTH 18.0f
#define FCT_SEN_CLASS_TRUCK_MIN_WIDTH 2.0f
#define FCT_SEN_CLASS_TRUCK_MAX_WIDTH 2.8f
#define FCT_SEN_CLASS_TRUCK_DEFAULT_WIDTH 2.5f
/* class pedestrian minimum, maximum and default dimensions length and width
 * treated same */
#define FCT_SEN_CLASS_PED_MIN_DIMENSION 0.4f
#define FCT_SEN_CLASS_PED_MAX_DIMENSION 0.8f
#define FCT_SEN_CLASS_PED_DEFAULT_DIMENSION 0.6f
/* class motorcycle minimum, maximum and default dimensions */
#define FCT_SEN_CLASS_MOTORCYCLE_MIN_LENGTH 2.0f
#define FCT_SEN_CLASS_MOTORCYCLE_MAX_LENGTH 4.0f
#define FCT_SEN_CLASS_MOTORCYCLE_DEFAULT_LENGTH 2.5f
#define FCT_SEN_CLASS_MOTORCYCLE_MIN_WIDTH 0.5f
#define FCT_SEN_CLASS_MOTORCYCLE_MAX_WIDTH 1.2f
#define FCT_SEN_CLASS_MOTORCYCLE_DEFAULT_WIDTH 1.0f
/* class bicile minimum, maximum and default dimensions */
#define FCT_SEN_CLASS_BICICLE_MIN_LENGTH FCT_SEN_CLASS_MOTORCYCLE_MIN_LENGTH
#define FCT_SEN_CLASS_BICICLE_MAX_LENGTH FCT_SEN_CLASS_MOTORCYCLE_MAX_LENGTH
#define FCT_SEN_CLASS_BICICLE_DEFAULT_LENGTH \
    FCT_SEN_CLASS_MOTORCYCLE_DEFAULT_LENGTH
#define FCT_SEN_CLASS_BICICLE_MIN_WIDTH FCT_SEN_CLASS_MOTORCYCLE_MIN_WIDTH
#define FCT_SEN_CLASS_BICICLE_MAX_WIDTH FCT_SEN_CLASS_MOTORCYCLE_MAX_WIDTH
#define FCT_SEN_CLASS_BICICLE_DEFAULT_WIDTH \
    FCT_SEN_CLASS_MOTORCYCLE_DEFAULT_WIDTH
/* class point minimum, maximum and default dimensions length and width treated
 * same */
#define FCT_SEN_CLASS_POINT_MIN_DIMENSION 0.1f
#define FCT_SEN_CLASS_POINT_MAX_DIMENSION 0.5f
#define FCT_SEN_CLASS_POINT_DEFAULT_DIMENSION 0.4f
/* class unclassified minimum, maximum and default dimensions length and width
 * treated same */
#define FCT_SEN_CLASS_UNCLASSIFIED_MIN_DIMENSION 0.1f
#define FCT_SEN_CLASS_UNCLASSIFIED_MAX_DIMENSION 50.0f
#define FCT_SEN_CLASS_UNCLASSIFIED_DEFAULT_DIMENSION 50.0f
#endif /** TUEOBJFUSN_TRACKABLELISTCONSTANTS_H_ */