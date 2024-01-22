

/**
@defgroup emp EMP (Environment Model Prediction)
  @ingroup vlc_sen
@{ */

#ifndef _EMP_EXT_H_INCLUDED
#define _EMP_EXT_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
//#include "vlc_sen_ext.h"
#include "vlcSen_ext.h"
#include "emp_cfg.h"
#include "emp_par.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*********/
/* Enums */
/*********/

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief EMPManeuverType_t

    @general Different maneuver types for the EMP module

    */
typedef enum {
    EMP_MANEUVER_KinematicsUnchanged, /*!< EMP maneuver of type Kinematics
                                         Unchanged */
    EMP_MANEUVER_KinematicsWithoutAcceleration, /*!< EMP maneuver of type
                                                   Kinematics Without
                                                   Acceleration */
    EMP_MANEUVER_ComfortBraking,   /*!< EMP maneuver of type Comfort Braking */
    EMP_MANEUVER_FullBraking,      /*!< EMP maneuver of type Full Braking */
    EMP_MANEUVER_PrecisionBraking, /*!< EMP maneuver of type Precision Braking
                                      */
    EMP_MANEUVER_MediumAvoidCollSteerBrake, /*!< EMP maneuver of type Medium
                                               Avoid Collision Steer Brake */
    EMP_MANEUVER_MaximumAvoidCollSteerBrake /*!< EMP maneuver of type Maximum
                                               Avoid Collision Steer Break */
} EMPManeuverType_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP polynomial of degree 1

    */
typedef struct EMPPolyDeg1 /* fC1*x + fC0 */
{
    float32 fC1; /*!< EMP polynomial degree1 velocity constant */
    float32 fC0; /*!< EMP polynomial degree1 distance constant */
} EMPPolyDeg1_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP polynomial of degree 2

    */
typedef struct EMPPolyDeg2 /* fC2*x^2 fC1*x + fC0 */
{
    float32 fC2; /*!< EMP polynomial degree2 accelaration constant */
    float32 fC1; /*!< EMP polynomial degree2 velocity constant */
    float32 fC0; /*!< EMP polynomial degree2 distance constant */
} EMPPolyDeg2_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP point

    */
typedef struct EMPPoint {
    float32 fX; /*!< EMP point x coordinate */
    float32 fY; /*!< EMP point y coordinate */
} EMPPos2D_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining 2-Dimensional Variance

    */
typedef struct EMPVar2D {
    float32 fX; /*!< x value of the variance */
    float32 fY; /*!< y value of the variance */
} EMPVar2D_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining 2-Dimensional Velocity

    */
typedef struct EMPVel2D {
    float32 fVelX; /*!< velocity in x direction */
    float32 fVelY; /*!< velocity in y direction */
} EMPVel2D_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for EMP Trajectory Prediction

    */
typedef struct EMPTrajPred {
    EMPPolyDeg2_t XofT; /*!< Structure for constants defination of the Degree2
                           polynomial of x(t)  */
    EMPPolyDeg2_t YofT; /*!< Structure for constants defination of the Degree2
                           polynomial of y(t)  */
    EMPPolyDeg2_t VarXofT; /*!< Structure for constants defination of the
                              Degree2 polynomial of varX(t)  */
    EMPPolyDeg2_t VarYofT; /*!< Structure for constants defination of the
                              Degree2 polynomial of vaY(t)  */
} EMPTrajPred_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP object size(geometry)

    */
typedef struct EMPSize2D {
    float32 fWidth;  /*!< Width of the object */
    float32 fLength; /*!< Length of the object */
} EMPSize2D_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP ego object kinematics

    */
typedef struct EMPKinEgo {
    float32 fVel;     /*!< Velocity of the ego object */
    float32 fAccel;   /*!< Acceleration of the ego object */
    float32 fYawRate; /*!< YawRate of the ego object */
} EMPKinEgo_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP target object kinematics

    */
typedef struct EMPKinObj {
    float32 fPosX; /*!< X distance of the target object from ego object */
    float32
        fPosXVar;  /*!< Variation in X distance of the target object from ego
                      object */
    float32 fPosY; /*!< Y distance of the target object from ego object */
    float32
        fPosYVar;  /*!< Variation in Y distance of the target object from ego
                      object */
    float32 fVelX; /*!< X direction velocity of the target object */
    float32
        fVelXVar;  /*!< X direction velocity variation of the target object */
    float32 fVelY; /*!< Y direction velocity of the target object */
    float32
        fVelYVar; /*!< Y direction velocity variation of the target object */
} EMPKinObj_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for EMP object description

    */
typedef struct EMPObjDesc {
    boolean bExists;       /*!< Object existence */
    EMPKinObj_t Kinematic; /*!< Object kinematic */
} EMPObjDesc_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP object prediction

    */
typedef struct EMPObjPred {
    const EMPTrajPred_t* pTrajPred; /*!< Trajectory prediction */
    const EMPSize2D_t* pGeometry;   /*!< Geometry of the object */
} EMPObjPred_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP object to trajectory relation

    */
typedef struct EMPObjToTrajRelation {
    float32 fDistToTraj;     /*!< lateral object displacement to a trajectory*/
    float32 fDistToTrajVar;  /*!< variance of lateral object displacement to a
                                trajectory*/
    float32 fVelocityToTraj; /*!< lateral object velocity to a trajectory*/
    float32 fVelocityToTrajVar; /*!< variance of lateral object velocity to a
                                   trajectory*/
    float32 fDistOnTraj; /*!< longitudinal object displacement on a trajectory*/
    float32
        fDistOnTrajVar; /*!< variance of longitudinal object displacement on a
                           trajectory*/
    float32 fVelocityOnTraj; /*!< longitudinal object velocity to a trajectory*/
    float32
        fVelocityOnTrajVar; /*!< variance of longitudinal object velocity to a
                               trajectory*/
} EMPObjToTrajRelation_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP distance and Trajectory- and Object
   width

    */
typedef struct EMPDistanceWidth {
    float32 fDistance;
    float32 fDistanceVar;
    float32 fTrajectoryWidth;
    float32 fTrajectoryWidthVar;
    float32 fObjectWidth;
    float32 fObjectWidthVar;
} EMPDistanceWidth_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure used for defining EMP overlap and Occupancy

    */
typedef struct EMPTrajOccupancy {
    float32 fOverlap;
    float32 fOverlapVar;
    float32 fObjectOccupancy;
    float32 fObjectOccupancyVar;
    float32 fTrajectoryOccupancy;
    float32 fTrajectoryOccupancyVar;
} EMPTrajOccupancy_t;

/*****************************************************************************
  PUBLIC FUNCTIONS
*****************************************************************************/
extern void EMPPredictEgoTraj(const EMPKinEgo_t* pIn_KinEgo,
                              EMPManeuverType_t ManeuverType,
                              EMPTrajPred_t* pOut_TrajPred);

extern boolean EMPPredictObjTraj(const EMPKinObj_t* pIn_KinObj,
                                 EMPManeuverType_t eAssumption,
                                 EMPTrajPred_t* pOut_TrajPred);

extern boolean EMPCalcMinDistTime(const EMPTrajPred_t* pIn_Obj1,
                                  const EMPTrajPred_t* pIn_Obj2,
                                  float32* pOut_MinDistTime);
extern float32 EMPCalcObjObjDistAtTime(float32 fTime,
                                       const EMPTrajPred_t* pIn_Obj1,
                                       const EMPTrajPred_t* pIn_Obj2);

extern boolean EMPCalcCollProbObjObj(float32 fTime,
                                     const EMPObjPred_t* pIn_Obj1,
                                     const EMPObjPred_t* pIn_Obj2,
                                     float32* pOut_CollisionProb);

extern boolean EMPCalcObjToEgoTrajRelation(
    const EMPKinEgo_t* pIn_KinEgo,
    const EMPObjDesc_t* pIn_Obj,
    EMPObjToTrajRelation_t* pOut_ObjToTrajRelation);

/*!  @cond Doxygen_Suppress */
extern boolean EMPCalcCollProbPed(float32 fPedWidth,
                                  float32 fPedDistY,
                                  float32 fPedVrelY,
                                  float32 fPedVrelYStd,
                                  float32 fPedTTC,
                                  float32* fCollProb);
/*! @endcond */

/* Utility Functions */
extern void EMPResetObjDesc(EMPObjDesc_t* pIn_Obj);
extern void EMPResetTrajPred(EMPTrajPred_t* pIn_TrajPred);

extern void EMPCalcPositionAtTime(float32 fTime,
                                  const EMPTrajPred_t* pIn_Obj,
                                  EMPPos2D_t* pOut_Position);

/* EMPLEGACY */
extern void EMPLEGCalcCorridorOccupancy(float32 fObjDistToCourse,
                                        float32 fObjWidth,
                                        float32 fTrajWidth,
                                        float32* pOut_CorridorOccupancy);
extern void EMPLEGCalcObjOccupancy(float32 fObjDistToCourse,
                                   float32 fObjWidth,
                                   float32 fTrajWidth,
                                   float32* pOut_ObjOccupancy);

extern void EMPCPCalculateOverlap(
    EMPDistanceWidth_t const* const pDistanceWidth,
    EMPTrajOccupancy_t* const pOccupancy);
#endif
/** @} end defgroup */
