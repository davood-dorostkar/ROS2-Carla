/******************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------

Copyright Tuerme Inc. All rights reserved.

*******************************************************************************
C-File Template Version:
******************************************************************************/

/*
 * Explanation:
 *    Converter from MVS Object list to TUE Fusion Object list
 */
/* PRQA S 0292 ++ */ /* MKS */
/*
 */
/* PRQA S 0292 -- */ /* MKS */
/*!****************************************************************************

\details
   Input adapter for TUE object fusion, converts from MVS Object List to
   TUE Fusion object list.

******************************************************************************/

/*==================[inclusions]============================================*/
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
#include "MVSCamConverter.h"
#include "MVSCamConverter_Cfg.h"
#include "MVSCamConverter_int.h"
#include "TueObjFusn_TrackableProps.h"
#include "TueObjFusn_ObjectListProps.h"
#include "tue_prv_fusion_math.h"
#include "tue_prv_error_management.h"
// #include "TM_Global_Types.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/
/*==================[function]==============================================*/

boolean MVSObjListToTueObjListConverter(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pNewObjList,
    CONSTP2CONST(CamObjectList, AUTOMATIC, ObjFusn_VAR_NOINIT) inputCamObjList,
    const uint32 u32globalTimeStamp) {
    boolean bSuccess = TRUE;
    // boolean  bObjectIsValid = TRUE;
    static uint16 u16UpdateCount = 0u;
    static uint32 u32CamTimeStampPrev = 0u;
    float32 f32CorrLatency = FLT_ZERO;
    uint16 u16NumConvertedObjects = 0u;

    if (NULL_PTR == pNewObjList) {
        bSuccess = FALSE;
    } else if (NULL_PTR == inputCamObjList) {
        /* Ensure the converted object list from the previous cycle is not
         * processed again as the update counter
         * and the number of objects is not resetted
         */
        // pNewObjList->u16NumObjects          = 0u;
        // pNewObjList->u16ListUpdateCounter   = u16UpdateCount;
        pNewObjList->u16NumObjects = 0u;
        pNewObjList->u32SensorPattern =
            TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_FRONT;
        pNewObjList->u16ListUpdateCounter = u16UpdateCount;
        bSuccess = FALSE;
    } else {
        if (u32CamTimeStampPrev != inputCamObjList->uiTimeStamp) {
            u16UpdateCount = ((u16UpdateCount + 1u) %
                              TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MAX);
            u32CamTimeStampPrev = (inputCamObjList->uiTimeStamp);
        } else {
            /* MISRA */
        }

        /* TODO: Check sensor failure flags  */

        if (inputCamObjList->eSigStatus == FALSE) {
            bSuccess = FALSE;
        } else {
            pNewObjList->u16ListUpdateCounter = u16UpdateCount;

            /* correct latency using global timestamp */
            if (u32globalTimeStamp >= u32CamTimeStampPrev) {
                f32CorrLatency =
                    (float32)(u32globalTimeStamp - u32CamTimeStampPrev);
            } else {
                f32CorrLatency = (float32)(0xFFFFFFFF - u32CamTimeStampPrev +
                                           u32globalTimeStamp);
            }
            f32CorrLatency = MAX(0, MIN(1000000, (f32CorrLatency)));
            // pNewObjList->f32MeasurementLatency  =
            // ((pOldObjList.statusFlags.latency) + f32CorrLatency) /
            // ALVOBJLIST_MVS_LATENCY_TO_SECONDS; //convert latency from ms to s
            /* convert latency from ms to s*/
            pNewObjList->f32MeasurementLatency =
                (f32CorrLatency) / TUEOBJLIST_MVS_LATENCY_TO_SECONDS /
                    TUEOBJLIST_MVS_LATENCY_TO_SECONDS +
                TUEOBJLIST_MVS_DEFAULT_LATENCY_SECONDS;  // convert latency from
                                                         // us to s

            pNewObjList->u32SensorPattern =
                TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_FRONT;

            for (uint8 i = 0; i < TUE_PRV_FUSION_MAX_OBJECTS_VISION; i++) {
                boolean bObjectIsValid = /*TRUE;*/
                    validateVisionObject(&(inputCamObjList->aObject[i]));
                if (bObjectIsValid) {
                    MVSObjToTrackableConverter_convertObject(
                        &(pNewObjList->aTrackable[u16NumConvertedObjects]),
                        &(inputCamObjList->aObject[i]),
                        f32CorrLatency / 1000);  // us -> ms
                    u16NumConvertedObjects++;
                } else {
                    /* MISRA */
                }
            }
            pNewObjList->u16NumObjects = u16NumConvertedObjects;
        }
    }
    return bSuccess;
}

LOCAL boolean validateVisionObject(
    CONSTP2CONST(CamObject, AUTOMATIC, ObjFusn_VAR_NOINIT) pVisionObj) {
    boolean bIsValid = TRUE;

    if(pVisionObj->bCamObjValid == FALSE){
        bIsValid = FALSE;
    }

    if (pVisionObj->Attributes.percentageProbOfExist == 0 &&
        pVisionObj->Kinematic.fDistX == 0 &&
        pVisionObj->Kinematic.fDistY == 0 && pVisionObj->Kinematic.fVx == 0 &&
        pVisionObj->Kinematic.fVy == 0 && pVisionObj->Kinematic.fAx == 0 &&
        pVisionObj->Kinematic.fAy == 0 && pVisionObj->Geometry.fWidth == 0) {
        bIsValid = FALSE;
    }

    if (((pVisionObj->Attributes.percentageProbOfExist) < 0) ||
        ((pVisionObj->Attributes.percentageProbOfExist) > 100)) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (((pVisionObj->Kinematic.fDistY) < -100) ||
        ((pVisionObj->Kinematic.fDistY) > 100)) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (((pVisionObj->Kinematic.fVx) < -127) ||
        ((pVisionObj->Kinematic.fVx) > 127)) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }
    if (((pVisionObj->Geometry.fWidth) < 0) ||
        ((pVisionObj->Geometry.fWidth) > 20)) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }
    // if (pVisionObj->Classification.ObjectType < 0 ||
    //     pVisionObj->Classification.ObjectType > 15) {
    //     bIsValid = FALSE;
    // } else {
    //     /* MISRA */
    // }

    if (pVisionObj->Kinematic.fAx < -20 || pVisionObj->Kinematic.fAx > 20) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pVisionObj->Kinematic.fAy < -10 || pVisionObj->Kinematic.fAy > 10) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }
    return bIsValid;
}

static const uint16 CamClassLookup[] = {
    TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN,
    TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_ADULT,
    TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_ADULT,
    TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_BICYCLE,
    TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_MOTORCYCLE,
    TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_CAR,
    TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_TRUCK,
    TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_TRAILER,
    TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_UNKNOWN,
    TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN};
LOCAL void MVSObjToTrackableConverter_convertObject(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pNewObject,
    CONSTP2CONST(CamObject, AUTOMATIC, ObjFusn_VAR_NOINIT) pOldObject,
    uint32 Age) {
    pNewObject->fRCS = 20.f;
    pNewObject->eObjMaintenanceState = MT_STATE_MEASURED;

    /* Object Information */
    pNewObject->u16ID = pOldObject->ObjectID;
    pNewObject->u16Age = (uint16)(Age);
    pNewObject->u16Lifespan = TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW;
    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/5 changed by
    // guotao 20200422 start
    // #ifdef FUSION_SIMULATION_VS_CONF
    // pNewObject->u8CoordSystem = TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_RELATIVE;
    // #else
    // pNewObject->u8CoordSystem = TUEOBJFUSN_U8COORDSYSTEM_FRONTBUMP_RELATIVE;
    // #endif
    pNewObject->u8CoordSystem = TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_RELATIVE;
    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/5 changed by
    // guotao 20200422 end
    pNewObject->u32SensorsCurr = TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_FRONT;
    pNewObject->u32SensorsHist = 0u;
    pNewObject->f32ExistenceQuality =
        pOldObject->Attributes.percentageProbOfExist >= 90
            ? 99
            : pOldObject->Attributes.percentageProbOfExist;
    pNewObject->f32ObstacleProbability =
        TUEOBJFUSN_TRACKABLE_F32OBSTACLEPROBABILITY_MAX;

    /* Position */
    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/5 changed by
    // guotao 20200422 start
    // #ifdef FUSION_SIMULATION_VS_CONF
    // pNewObject->vecX.data[TRACKABLE_POSX] = (pOldObject->VehiclePosX);
    // pNewObject->vecX.data[TRACKABLE_POSY] = (pOldObject->VehiclePosY);
    // #else
    // pNewObject->vecX.data[TRACKABLE_POSX] = (pOldObject->VehiclePosX) -
    // TUE_PRV_VEHICLE_CAM_TO_FRONT_BUMBER;
    // pNewObject->vecX.data[TRACKABLE_POSY] = (pOldObject->VehiclePosY) -
    // TUE_PRV_VEHICLE_LATERAL_OFFSET_CAM;
    // #endif
    pNewObject->vecX.data[TRACKABLE_POSX] = (pOldObject->Kinematic.fDistX);
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/13 added by guotao
// 20200428 start
#ifdef ALGO_SAFETY_DISTANCE_FOR_TEST_ONLY
    pNewObject->vecX.data[TRACKABLE_POSX] -=
        ALGO_EXTEND_SAFETY_DISTANCEX_FOR_TEST;
#endif
    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/13 added by guotao
    // 20200428 end
    pNewObject->vecX.data[TRACKABLE_POSY] = (pOldObject->Kinematic.fDistY);
    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/5 changed by
    // guotao 20200422 end
    /* Velocity */
    pNewObject->vecX.data[TRACKABLE_VELX] = pOldObject->Kinematic.fVx;
    pNewObject->vecX.data[TRACKABLE_VELY] = pOldObject->Kinematic.fVy;

    /* Acceleration */
    pNewObject->vecX.data[TRACKABLE_ACCX] = pOldObject->Kinematic.fAx;
    pNewObject->vecX.data[TRACKABLE_ACCY] = pOldObject->Kinematic.fAy;

    pNewObject->vecX.nRows = 4u; /* PRQA S 3120 */ /* fixed vector size */

    /* Object Definition */
    pNewObject->u16MotionType = TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN;

    pNewObject->u16Class =
        CamClassLookup[pOldObject->Classification.ObjectType];
    /************* GAC new feature *************/
    pNewObject->bCIPVFlag = pOldObject->Classification.CIPVFlag;
    pNewObject->bMCPFlag = pOldObject->Classification.MCPFlag;
    pNewObject->f32Height = pOldObject->Geometry.fHeight;
    /************* GAC new feature *************/
    if (pNewObject->u16Class ==
        TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_ADULT) {
        pNewObject->vecX.data[TRACKABLE_VELX] = pOldObject->Kinematic.fVx;
    }

    calculateVariances_MVS(pNewObject);

    /* Geometry */
    pNewObject->f32Width = pOldObject->Geometry.fWidth;
    pNewObject->f32Length = pOldObject->Geometry.fLength;
    pNewObject->f32Heading = TUEOBJFUSN_TRACKABLE_F32HEADING_DEFAULT;
    pNewObject->f32HeadingVar = TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_DEFAULT;
    pNewObject->f32YawRate = TUEOBJFUSN_TRACKABLE_F32YAWRATE_DEFAULT;
    pNewObject->f32YawRateVar = TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_DEFAULT;
    pNewObject->u16RefPoint = TUEOBJFUSN_TRACKABLE_U16REFPOINT_CENTER_OF_WIDTH;

    pNewObject->u16ClassProb = (uint16)(100);
    pNewObject->u8RadarMotionTypeInput = TU_SENSOR_MOTIONG_TYPE_INPUT_UNKNOWN;
}

LOCAL void calculateVariances_MVS(CONSTP2VAR(TueObjFusn_TrackableType,
                                             AUTOMATIC,
                                             ObjFusn_VAR_NOINIT) pCamObj) {
    float32 f32PosX = FLT_ZERO;
    float32 f32PosY = FLT_ZERO;
    float32 f32PosVarX = FLT_ZERO;
    float32 f32PosVarY = FLT_ZERO;
    float32 f32VelVarX = FLT_ZERO;
    float32 f32VelVarY = FLT_ZERO;
    float32 f32Angle = FLT_ZERO;
    float32 f32Sin = FLT_ZERO;
    float32 f32Cos = FLT_ZERO;

    if (pCamObj != NULL_PTR) {
        f32PosX = pCamObj->vecX.data[TRACKABLE_POSX];
        f32PosY = pCamObj->vecX.data[TRACKABLE_POSY];
        f32Angle = tue_prv_fusion_atan2(f32PosY, f32PosX);
        f32Sin = tue_prv_fusion_sin(f32Angle);
        f32Cos = tue_prv_fusion_cos(f32Angle);

        if (((pCamObj->u16Class) &
             TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_UNKNOWN) > 0u) {
            f32PosVarX = TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_X_BETA_0;
            f32PosVarX +=
                TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_X_BETA_1 * f32PosX;
            f32PosVarX +=
                TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_X_BETA_2 * f32PosY;
            f32PosVarX += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_X_BETA_3 *
                          (f32PosX * f32PosX);
            f32PosVarX += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_X_BETA_4 *
                          (f32PosY * f32PosY);
            f32PosVarX += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_X_BETA_5 *
                          (f32PosY * f32PosX);

            f32PosVarY = TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_Y_BETA_0;
            f32PosVarY +=
                TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_Y_BETA_1 * f32PosX;
            f32PosVarY +=
                TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_Y_BETA_2 * f32PosY;
            f32PosVarY += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_Y_BETA_3 *
                          (f32PosX * f32PosX);
            f32PosVarY += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_Y_BETA_4 *
                          (f32PosY * f32PosY);
            f32PosVarY += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_Y_BETA_5 *
                          (f32PosY * f32PosX);

            f32VelVarX = TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_X_BETA_0;
            f32VelVarX +=
                TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_X_BETA_1 * f32PosX;
            f32VelVarX +=
                TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_X_BETA_2 * f32PosY;
            f32VelVarX += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_X_BETA_3 *
                          (f32PosX * f32PosX);
            f32VelVarX += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_X_BETA_4 *
                          (f32PosY * f32PosY);
            f32VelVarX += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_X_BETA_5 *
                          (f32PosY * f32PosX);

            f32VelVarY = TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_Y_BETA_0;
            f32VelVarY +=
                TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_Y_BETA_1 * f32PosX;
            f32VelVarY +=
                TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_Y_BETA_2 * f32PosY;
            f32VelVarY += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_Y_BETA_3 *
                          (f32PosX * f32PosX);
            f32VelVarY += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_Y_BETA_4 *
                          (f32PosY * f32PosY);
            f32VelVarY += TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_Y_BETA_5 *
                          (f32PosY * f32PosX);

            pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_POSX] =
                tue_prv_fusion_min_max_F32(
                    f32PosVarX, TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_X_MIN,
                    TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_X_MAX);

            pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_POSY] =
                tue_prv_fusion_min_max_F32(
                    f32PosVarY, TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_Y_MIN,
                    TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_POS_Y_MAX);

            pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_VELX] =
                tue_prv_fusion_min_max_F32(
                    f32VelVarX, TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_X_MIN,
                    TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_X_MAX);

            pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_VELY] =
                tue_prv_fusion_min_max_F32(
                    f32VelVarY, TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_Y_MIN,
                    TUEOBJLIST_MVS_SENSOR_MODEL_VEHICLE_VEL_Y_MAX);
        } else {
            f32PosVarX = TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_X_BETA_0;
            f32PosVarX +=
                TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_X_BETA_1 * f32PosX;
            f32PosVarX +=
                TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_X_BETA_2 * f32PosY;
            f32PosVarX += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_X_BETA_3 *
                          (f32PosX * f32PosX);
            f32PosVarX += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_X_BETA_4 *
                          (f32PosY * f32PosY);
            f32PosVarX += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_X_BETA_5 *
                          (f32PosY * f32PosX);

            f32PosVarY = TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_Y_BETA_0;
            f32PosVarY +=
                TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_Y_BETA_1 * f32PosX;
            f32PosVarY +=
                TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_Y_BETA_2 * f32PosY;
            f32PosVarY += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_Y_BETA_3 *
                          (f32PosX * f32PosX);
            f32PosVarY += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_Y_BETA_4 *
                          (f32PosY * f32PosY);
            f32PosVarY += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_Y_BETA_5 *
                          (f32PosY * f32PosX);

            f32VelVarX = TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_X_BETA_0;
            f32VelVarX +=
                TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_X_BETA_1 * f32PosX;
            f32VelVarX +=
                TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_X_BETA_2 * f32PosY;
            f32VelVarX += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_X_BETA_3 *
                          (f32PosX * f32PosX);
            f32VelVarX += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_X_BETA_4 *
                          (f32PosY * f32PosY);
            f32VelVarX += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_X_BETA_5 *
                          (f32PosY * f32PosX);

            f32VelVarY = TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_Y_BETA_0;
            f32VelVarY +=
                TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_Y_BETA_1 * f32PosX;
            f32VelVarY +=
                TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_Y_BETA_2 * f32PosY;
            f32VelVarY += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_Y_BETA_3 *
                          (f32PosX * f32PosX);
            f32VelVarY += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_Y_BETA_4 *
                          (f32PosY * f32PosY);
            f32VelVarY += TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_Y_BETA_5 *
                          (f32PosY * f32PosX);

            pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_POSX] =
                tue_prv_fusion_min_max_F32(
                    f32PosVarX,
                    TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_X_MIN,
                    TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_X_MAX);

            pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_POSY] =
                tue_prv_fusion_min_max_F32(
                    f32PosVarY,
                    TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_Y_MIN,
                    TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_POS_Y_MAX);

            pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_VELX] =
                tue_prv_fusion_min_max_F32(
                    f32VelVarX,
                    TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_X_MIN,
                    TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_X_MAX);

            pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_VELY] =
                tue_prv_fusion_min_max_F32(
                    f32VelVarY,
                    TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_Y_MIN,
                    TUEOBJLIST_MVS_SENSOR_MODEL_PEDESTRIAN_VEL_Y_MAX);
        }

        pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY] = FLT_ZERO;

        /* Covariances */
        /* set only if sensor delivers valid covariances */
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] = FLT_ZERO;
        pCamObj->matP.u16Size = 4u; /* PRQA S 3120 */ /* fixed vector size */

        (void)f32RotateSymMat(&(pCamObj->matP), &(pCamObj->matP), f32Sin,
                              f32Cos);

        /* Reset Covariances as negative co-variances may result in negative
         * variances in LKF*/
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] = FLT_ZERO;
        pCamObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] = FLT_ZERO;

    } else {
        /* MISRA */
    }
}
/******************************************************************************
End Of File
*****************************************************************************/
