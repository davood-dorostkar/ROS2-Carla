/**
 *
 * \file    MVSCamConverter_int.h
 * \brief   This is the internal header file for the TUE monovision Converter.
            All function declerations made here shall not be visible to other
 AAUs.
 *
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

#ifndef MVS_CAM_CONVERTER_INT_H_
#define MVS_CAM_CONVERTER_INT_H_

/*==================[inclusions]============================================*/
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "fusion_ext.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"
//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
#include "tue_prv_lkf_int.h"

/*==================[macros]================================================*/

/*******************/
/* u8DetectionType */
/*******************/
/// \name detectionType
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8DETECTIONTYPE_MIN_CAMERA (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8DETECTIONTYPE_MAX_CAMERA (5u)
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8DETECTIONTYPE_MIN_CAMERA_PEDESTRIAN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8DETECTIONTYPE_MAX_CAMERA_PEDESTRIAN (7u)

/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8DETECTIONTYPE_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8DETECTIONTYPE_MAX (3u)
/** default value */
#define MVS_CONVERTER_OBJECTS_U8DETECTIONTYPE_DEFAULT (0u)
/// \}

/*****************/
/* u8ObjectClass */
/*****************/
/// \name objectClass
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8OBJECTCLASS_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8OBJECTCLASS_MAX (5u)
/** default value */
#define MVS_CONVERTER_OBJECTS_U8OBJECTCLASS_DEFAULT (0u)
/// \}

/****************/
/* u8MotionMode */
/****************/
/// \name motionMode
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8MOTIONMODE_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8MOTIONMODE_MAX (4u)
/** default value */
#define MVS_CONVERTER_OBJECTS_U8MOTIONMODE_DEFAULT (0u)
/// \}

/*************/
/* u8ObjPose */
/*************/
/// \name objPose
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8OBJPOSE_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8OBJPOSE_MAX (2u)
/** default value */
#define MVS_CONVERTER_OBJECTS_U8OBJPOSE_DEFAULT (0u)
/// \}

/********************/
/* u8TrackingStatus */
/********************/
/// \name trackingStatus
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8TRACKINGSTATUS_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8TRACKINGSTATUS_MAX (3u)
/** default value */
#define MVS_CONVERTER_OBJECTS_U8TRACKINGSTATUS_DEFAULT (0u)
/// \}

/********/
/* u8Id */
/********/
/// \name id
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8ID_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8ID_MAX (255u)
/** default value */
#define MVS_CONVERTER_OBJECTS_U8ID_DEFAULT (0u)
/// \}

/***********/
/* f32PosX */
/***********/
/// \name xPos
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32POSX_MIN_CAMERA (0.f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32POSX_MAX_CAMERA (250.f)

/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32POSX_MIN (-41.96f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32POSX_MAX (163.82f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32POSX_DEFAULT (FLT_ZERO)
/// \}

/***********/
/* f32PosY */
/***********/
/// \name yPos
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32POSY_MIN_CAMERA (-31.9375f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32POSY_MAX_CAMERA (31.9375f)

/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32POSY_MIN (-81.92f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32POSY_MAX (81.91f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32POSY_DEFAULT (FLT_ZERO)
/// \}

/**************/
/* f32RelVelX */
/**************/
/// \name xRelVel
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32RELVELX_MIN_CAMERA (-127.93f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32RELVELX_MAX_CAMERA (127.93f)

/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32RELVELX_MIN (-102.40f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32RELVELX_MAX (102.30f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32RELVELX_DEFAULT (FLT_ZERO)
/// \}

/**************/
/* f32RelVelY */
/**************/
/// \name yRelVel
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32RELVELY_MIN (-51.20f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32RELVELY_MAX (51.15f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32RELVELY_DEFAULT (FLT_ZERO)
/// \}

/**************/
/* f32PosXStd */
/**************/
/// \name xPosStd
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32POSXSTD_MIN (FLT_ZERO)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32POSXSTD_MAX (20.f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32POSXSTD_DEFAULT (FLT_ZERO)
/// \}

/*****************/
/* f32RelVelXStd */
/*****************/
/// \name xRelVelStd
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32RELVELXSTD_MIN (FLT_ZERO)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32RELVELXSTD_MAX (20.f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32RELVLEXSTD_DEFAULT (FLT_ZERO)
/// \}

/***************/
/* f32ObjAccel */
/***************/
/// \name ObjAccel
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32OBJACCEL_MIN_CAMERA (-14.97f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32OBJACCEL_MAX_CAMERA (14.97f)

/***************/
/* f32ObjWidth */
/***************/
/// \name objWidth
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32OBJWIDTH_MIN_CAMERA (FLT_ZERO)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32OBJWIDTH_MAX_CAMERA (12.5f)

/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32OBJWIDTH_MIN (FLT_ZERO)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32OBJWIDTH_MAX (3.15f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32OBJWIDTH_DEFAULT (FLT_ZERO)
/// \}

/*****************/
/* f32ObjYawRate */
/*****************/
/// \name objYawRate
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32OBJYAWRATE_MIN (-2.56f)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32OBJYAWRATE_MAX (2.55f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32OBJYAWRATE_DEFAULT (FLT_ZERO)
/// \}

/********************/
/* u8AgeConsecutive */
/********************/
/// \name ageConsecutive
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8AGECONSECUTIVE_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8AGECONSECUTIVE_MAX (255u)
/** default value */
#define MVS_CONVERTER_OBJECTS_U8AGECONSECUTIVE_DEFAULT (0u)
/// \}

/*****************************/
/* f32VehicleClassConfidence */
/*****************************/
/// \name vehicleClassConfidence
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32VEHICLECLASSCONFIDENCE_MIN (FLT_ZERO)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32VEHICLECLASSCONFIDENCE_MAX (1.f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32VEHICLECLASSCONFIDENCE_DEFAULT (FLT_ZERO)
/// \}

/*****************************/
/* f32VehicleClassConfidence */
/*****************************/
/// \name vehicleClassConfidence
/** minimum value */
#define MVS_CONVERTER_OBJECTS_F32PEDESTRIANCLASSCONFIDENCE_MIN (FLT_ZERO)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_F32PEDESTRIANCLASSCONFIDENCE_MAX (4.f)
/** default value */
#define MVS_CONVERTER_OBJECTS_F32PEDESTRIANCLASSCONFIDENCE_DEFAULT (FLT_ZERO)
/// \}

/*********************/
/* u8FormApproxError */
/*********************/
/// \name formApproxError
/** minimum value */
#define MVS_CONVERTER_OBJECTS_U8FORMAPPROXERROR_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_U8FORMAPPROXERROR_MAX (255u)
/** default value */
#define MVS_CONVERTER_OBJECTS_U8FORMAPPROXERROR_DEFAULT (0u)
/// \}

/*******************/
/* bIsOccludedLeft */
/*******************/
/// \name isOccludedLeft
/** minimum value */
#define MVS_CONVERTER_OBJECTS_BISOCCLUDEDLEFT_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_BISOCCLUDEDLEFT_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_OBJECTS_BISOCCLUDEDLEFT_DEFAULT (FALSE)
/// \}

/********************/
/* bIsOccludedRight */
/********************/
/// \name isOccludedRight
/** minimum value */
#define MVS_CONVERTER_OBJECTS_BISOCCLUDEDRIGHT_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_BISOCCLUDEDRIGHT_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_OBJECTS_BISOCCLUDEDRIGHT_DEFAULT (FALSE)
/// \}

/*******************/
/* bIsObservedLeft */
/*******************/
/// \name isObservedLeft
/** minimum value */
#define MVS_CONVERTER_OBJECTS_BISOBSERVEDLEFT_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_BISOBSERVEDLEFT_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_OBJECTS_BISOBSERVEDLEFT_DEFAULT (FALSE)
/// \}

/********************/
/* bIsObservedRight */
/********************/
/// \name isObservedRight
/** minimum value */
#define MVS_CONVERTER_OBJECTS_BISOBSERVEDRIGHT_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_BISOBSERVEDRIGHT_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_OBJECTS_BISOBSERVEDRIGHT_DEFAULT (FALSE)
/// \}

/*******************/
/* bIsAsilVerified */
/*******************/
/// \name isAsilVerified
/** minimum value */
#define MVS_CONVERTER_OBJECTS_BISASILVERIFIED_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_BISASILVERIFIED_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_OBJECTS_BISASILVERIFIED_DEFAULT (FALSE)
/// \}

/****************/
/* bIsPredicted */
/****************/
/// \name isPredicted
/** minimum value */
#define MVS_CONVERTER_OBJECTS_BISPREDICTED_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_BISPREDICTED_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_OBJECTS_BISPREDICTED_DEFAULT (FALSE)
/// \}

/***************************/
/* bIsAtCrossWalkRiskPoint */
/***************************/
/// \name isAtCrosswalkRiskPoint
/** minimum value */
#define MVS_CONVERTER_OBJECTS_BISATCROSSWALKRISKPOINT_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_OBJECTS_BISATCROSSWALKRISKPOINT_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_OBJECTS_BISATCROSSWALKRISKPOINT_DEFAULT (FALSE)
/// \}

/****************/
/* bIsAvailable */
/****************/
/// \name isAvailable
/** minimum value */
#define MVS_CONVERTER_STATUS_BISAVAILABLE_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_STATUS_BISAVAILABLE_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_STATUS_BISAVAILABLE_DEFAULT (FALSE)
/// \}

/**************/
/* bIsBlocked */
/**************/
/// \name isBlocked
/** minimum value */
#define MVS_CONVERTER_STATUS_BISBLOCKED_MIN (FALSE)
/** maximum value */
#define MVS_CONVERTER_STATUS_BISBLOCKED_MAX (TRUE)
/** default value */
#define MVS_CONVERTER_STATUS_BISBLOCKED_DEFAULT (FALSE)
/// \}

/****************/
/* u32Timestamp */
/****************/
/// \name timestamp
/** minimum value */
#define MVS_CONVERTER_STATUS_U32TIMESTAMP_MIN (0u)
/** maximum value */
#define MVS_CONVERTER_STATUS_U32TIMESTAMP_MAX (0xFFFFFFFFu)
/** default value */
#define MVS_CONVERTER_STATUS_U32TIMESTAMP_DEFAULT (0u)
/// \}

/**************/
/* f32Latency */
/**************/
/// \name latency
/** minimum value */
#define MVS_CONVERTER_STATUS_F32LATENCY_MIN (FLT_ZERO)
/** maximum value */
#define MVS_CONVERTER_STATUS_F32LATENCY_MAX (10.f)
/** default value */
#define MVS_CONVERTER_STATUS_F32LATENCY_DEFAULT (0.f)
/// \}

// calculate other car's speed based on thier location data for mini-eye only
typedef struct OtherCarKalmanStru {
    stf32Vec_t vecX; /** state vector from Kalman filter including position */
    stf32SymMatrix_t matP; /** P matrix from Kalman filter including variances
                              and covariances of all states */
    sint8
        ID; /** Original object ID get from camera other car object structure*/
    uint32 uiTimeStamp; /** Time stamp for this data*/
} OtherCarKalmanStru_t;

// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/31 changed by
// changchang 20200603 start
typedef struct PedestrianKalmanStru {
    stf32Vec_t vecX; /** state vector from Kalman filter including position */
    stf32SymMatrix_t matP; /** P matrix from Kalman filter including variances
                              and covariances of all states */
    sint8
        ID; /** Original object ID get from camera other car object structure*/
    uint32 uiTimeStamp; /** Time stamp for this data*/
} PedestrianKalmanStru;
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/31 changed by
// changchang 20200603 end

/*==================[type definitions]======================================*/
/*==================[function]==============================================*/

#define ObjFusn_START_SEC_SLOW_CODE

LOCAL void MVSObjToTrackableConverter_convertObject(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pNewObject,
    CONSTP2CONST(CamObject, AUTOMATIC, ObjFusn_VAR_NOINIT) pOldObject,
    uint32 Age);
LOCAL boolean validateVisionObject(CONSTP2CONST(CamObject,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pVisionObj);

LOCAL void calculateVariances_MVS(CONSTP2VAR(TueObjFusn_TrackableType,
                                             AUTOMATIC,
                                             ObjFusn_VAR_NOINIT) pCamObj);
#define ObjFusn_STOP_SEC_SLOW_CODE

/*==================[external constants]====================================*/

#endif /* MVS_CAM_CONVERTER_INT_H_ */