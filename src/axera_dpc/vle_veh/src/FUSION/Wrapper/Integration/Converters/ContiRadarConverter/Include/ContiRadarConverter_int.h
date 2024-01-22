/**
 *
 * \file    ContiRadarConverter_int.h
 * \brief   This is a internal header file for the Conti Radar Converter. All
 * function declerations made here shall not be
 *          visible to other AAUs
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

#ifndef CONTI_RADAR_CONVERTER_INT_H_
#define CONTI_RADAR_CONVERTER_INT_H_

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableType.h"
#include "Converters_types.h"
#include "TueObjFusn_ConfigVehicle.h"

/*==================[macros]================================================*/

/****************/
/* u8NumObjects */
/****************/
/// \name numObjects
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MIN (0u)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX \
    (TUE_PRV_FUSION_MAX_OBJECTS_RADAR)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_DEFAULT (0u)
/// \}

/*************/
/* u8objType */
/*************/
/// \name objtype
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8OBJECTTYPE_MIN (0u)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8OBJECTTYPE_MAX (3u)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8OBJECTTYPE_DEFAULT (0u)
/// \}

/*******************/
/* u8MotionPattern */
/*******************/
/// \name motionPattern
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8MOTIONPATTERN_MIN (0u)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8MOTIONPATTERN_MAX (7u)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8MOTIONPATTERN_DEFAULT (0u)
/// \}

/****************/
/* u8updateFlag */
/****************/
/// \name updateFlag
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8UPDATEFLAG_MIN (FALSE)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8UPDATEFLAG_MAX (TRUE)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8UPDATEFLAG_DEFAULT (FALSE)
/// \}

/****************/
/* u8ValidFlag */
/****************/
/// \name validFlag
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8VALIDFLAG_MIN (FALSE)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8VALIDFLAG_MAX (TRUE)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8VALIDFLAG_DEFAULT (FALSE)
/// \}

/****************/
/* u8MeasFlag */
/****************/
/// \name measFlag
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8MEASFLAG_MIN (FALSE)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8MEASFLAG_MAX (TRUE)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8MEASFLAG_DEFAULT (FALSE)
/// \}

/********/
/* u8ID */
/********/
/// \name id
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8ID_MIN (0x00u)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8ID_MAX (0xFFu)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_U8ID_DEFAULT (0xFFu)
/// \}

/***********/
/* f32PosX */
/***********/
/// \name xPos
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32POSX_MIN (FLT_ZERO)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32POSX_MAX (255.984f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32POSX_DEFAULT (FLT_ZERO)
/// \}

/***********/
/* f32PosY */
/***********/
/// \name yPos
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32POSY_MIN (-64.0f)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32POSY_MAX (63.9844f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32POSY_DEFAULT (FLT_ZERO)
/// \}

/**************/
/* f32VelRelX */
/**************/
/// \name xVelRel
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32VELRELX_MIN (-102.4)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32VELRELX_MAX (102.3f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32VELRELX_DEFAULT (FLT_ZERO)
/// \}

/**************/
/* f32VelRelY */
/**************/
/// \name yVelRel
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32VELRELY_MIN (-102.4f)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32VELRELY_MAX (102.3f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32VELRELY_DEFAULT (FLT_ZERO)
/// \}

/**************/
/* f32AccRelX */
/**************/
/// \name xAccRel
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32ACCRELX_MIN (-19.6f)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32ACCRELX_MAX (19.45f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32ACCRELX_DEFAULT (FLT_ZERO)
/// \}

/*****************/
/* f32StdDevPosX */
/*****************/
/// \name xPosStd
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEVPOSX_MIN (FLT_ZERO)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEVPOSX_MAX (12.7f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEVPOSX_DEFAULT (FLT_ZERO)
/// \}

/*****************/
/* f32StdDevPosY */
/*****************/
/// \name yPosStd
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEVPOSY_MIN (FLT_ZERO)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEVPOSY_MAX (12.7f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEVPOSY_DEFAULT (FLT_ZERO)
/// \}

/*****************/
/* f32StdDevVelX */
/*****************/
/// \name xVelRelStd
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEV_RELVELX_MIN (FLT_ZERO)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEV_RELVELX_MAX (6.35f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32STDDEV_RELVELX_DEFAULT (FLT_ZERO)
/// \}

/*******************/
/* f32ObstacleProp */
/*******************/
/// \name obstacleProb
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32OBSTACLE_PROP_MIN (FLT_ZERO)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32OBSTACLE_PROP_MAX (100.f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32OBSTACLE_PROP_DEFAULT (FLT_ZERO)
/// \}

/******************/
/* f32ObjExstProp */
/******************/
/// \name objExstProb
/** minimum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32OBJEXSTPROP_MIN (FLT_ZERO)
/** maximum value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32OBJEXSTPROP_MAX (100.f)
/** default value */
#define CONTI_RADAR_CONVERTER_OBJECTS_F32OBJEXSTPROP_DEFAULT (FLT_ZERO)
/// \}

/*****************/
/* u8MeasEnabled */
/*****************/
/// \name measEnabled
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8MEASENABLED_MIN (FALSE)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8MEASENABLED_MAX (TRUE)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_U8MEASENABLED_DEFAULT (TRUE)
/// \}

/*****************/
/* u8SGUFailed */
/*****************/
/// \name SGUFailed
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8SGUFAILED_MIN (FALSE)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8SGUFAILED_MAX (TRUE)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_U8SGUFAILED_DEFAULT (FALSE)
/// \}

/********************/
/* u8StatusMisalign */
/********************/
/// \name statusMisalign
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUSMISALIGN_MIN (FALSE)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUSMISALIGN_MAX (TRUE)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUSMISALIGN_DEFAULT (FALSE)
/// \}

/*******************/
/* u8StatusBlkProg */
/*******************/
/// \name statusBlkProg
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUSBLKPROG_MIN (FALSE)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUSBLKPROG_MAX (TRUE)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUSBLKPROG_DEFAULT (FALSE)
/// \}

/*******************/
/* u8StatusHWError */
/*******************/
/// \name statusHWErr
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUS_HWERROR_MIN (FALSE)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUS_HWERROR_MAX (TRUE)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_U8STATUS_HWERROR_DEFAULT (FALSE)
/// \}

/**************/
/* f32Latency */
/**************/
/// \name latency
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_F32LATENCY_MIN (FLT_ZERO)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_F32LATENCY_MAX (126.f)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_F32LATENCY_DEFAULT (FLT_ZERO)
/// \}

/****************/
/* u32Timestamp */
/****************/
/// \name timestamp
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_U32TIMESTAMP_MIN (0u)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_U32TIMESTAMP_MAX (655350u)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_U32TIMESTAMP_DEFAULT (0u)
/// \}

/****************/
/* f32HostSpeed */
/****************/
/// \name hostSpeed
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_F32HOSTSPEED_MIN (-20.f)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_F32HOSTSPEED_MAX (82.375f)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_F32HOSTSPEED_DEFAULT (FLT_ZERO)
/// \}

/**************/
/* f32HostYaw */
/**************/
/// \name hostYaw
/** minimum value */
#define CONTI_RADAR_CONVERTER_STATUS_F32HOSTYAW_MIN (-102.4f)
/** maximum value */
#define CONTI_RADAR_CONVERTER_STATUS_F32HOSTYAW_MAX (102.3f)
/** default value */
#define CONTI_RADAR_CONVERTER_STATUS_F32HOSTYAW_DEFAULT (FLT_ZERO)
/// \}

/*==================[type definitions]======================================*/

/** Assign this label to all tracks that are not associated to a certain cluster
 * during
 * road boundary detection
 */
#define TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_NOISE (1234u)

/* Initially, all tracks are not labelled */
#define TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_UNKNOWN (0u)

/*==================[function]==============================================*/

#define ObjFusn_START_SEC_SLOW_CODE

LOCAL void ContiRaObjToTrackableConverter(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pNewObject,
    CONSTP2CONST(FusObjects_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pOldObject,
    int ID,
    uint32 Age,
    uint8 uRadarID);

LOCAL uint16 ConvertContiRadarObjClass(const uint8 objType);

LOCAL boolean validateContiObject(CONSTP2CONST(FusObjects_t,
                                               AUTOMATIC,
                                               ObjFusn_VAR_NOINIT) pObject,
                                  uint8 uRadarID);

LOCAL void getTracksToProcess(CONSTP2CONST(FusObjectList_t,
                                           AUTOMATIC,
                                           ObjFusn_VAR_NOINIT) pRdrObjList,
                              boolean abProcessTracks[],
                              uint8 uRadarID);

LOCAL void calculateVariances_Radar(CONSTP2VAR(TueObjFusn_TrackableType,
                                               AUTOMATIC,
                                               ObjFusn_VAR_NOINIT) pRdrObj);

#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
LOCAL uint16 findNeighbors(
    CONSTP2CONST(RaObjListType, AUTOMATIC, ObjFusn_VAR_NOINIT) pOldObjList,
    const uint16 u16IdxObj,
    uint16 au16Neighbors[],
    const uint16 au16Clusters[],
    const boolean abRelevantTracks[],
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) pu16Offset);

LOCAL float32 calculateDistanceClustering(
    CONSTP2CONST(RaObjType, AUTOMATIC, ObjFusn_VAR_NOINIT) pObjOne,
    CONSTP2CONST(RaObjType, AUTOMATIC, ObjFusn_VAR_NOINIT) pObjTwo);

LOCAL void runClustering(CONSTP2CONST(RaObjListType,
                                      AUTOMATIC,
                                      ObjFusn_VAR_NOINIT) pOldObjList,
                         const boolean abRelevantTracks[],
                         uint16 au16Clusters[],
                         uint16 au16NumTracksPerCluster[]);

LOCAL void findRelevantTracks(CONSTP2CONST(RaObjListType,
                                           AUTOMATIC,
                                           ObjFusn_VAR_NOINIT) pRdrObjList,
                              boolean abRelevantTracks[],
                              const boolean abValidTracks[]);

LOCAL uint16 runLinearRegression(CONSTP2CONST(RaObjListType,
                                              AUTOMATIC,
                                              ObjFusn_VAR_NOINIT) pRdrObjList,
                                 const uint16 au16Clusters[],
                                 const uint16 au16NumTracksPerCluster[],
                                 float32 af32CoefficientsB0[],
                                 float32 af32CoefficientsB1[]);

LOCAL void runRoadBoundaryDetection(
    CONSTP2CONST(RaObjListType, AUTOMATIC, ObjFusn_VAR_NOINIT) pRdrObjList,
    const boolean abValidTracks[],
    boolean abProcessTracks[]);

LOCAL boolean array_contains_u16(const uint16 u16Arr[],
                                 const uint16 u16ArrSize,
                                 const uint16 u16Element);
#endif

#define ObjFusn_STOP_SEC_SLOW_CODE

/*==================[external constants]====================================*/

#endif /* CONTI_RADAR_CONVERTER_INT_H_ */