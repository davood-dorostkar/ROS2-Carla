/******************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------

Copyright Tuerme Inc. All rights reserved.

*******************************************************************************
C-File Template Version:
******************************************************************************/
/* PRQA S 0292 ++ */ /* MKS */
                     /*
                      * Explanation:
                      *    Converter from Radar Object list to TUE Fusion Object list
                      */
/*
 */
/* PRQA S 0292 -- */ /* MKS */
/*!****************************************************************************

@details
Input adapter for TUE object fusion, converts from Conti Radar Object List to
TUE Fusion object list.

******************************************************************************/
/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
#include "ContiRadarConverter.h"
#include "ContiRadarConverter_Cfg.h"
#include "ContiRadarConverter_int.h"
#include "TueObjFusn_TrackableProps.h"
#include "TueObjFusn_ObjectListProps.h"
#include "tue_prv_fusion_math.h"
#include "tue_prv_error_management.h"
// #include "TM_Global_Types.h"

/******************************************************************************
MODULE DEFINES
******************************************************************************/

/******************************************************************************
MODULE TYPES
******************************************************************************/

/******************************************************************************
DECLARATION OF LOCAL FUNCTIONS
******************************************************************************/

/******************************************************************************
DEFINITION OF LOCAL VARIABLES
******************************************************************************/

/******************************************************************************
DEFINITION OF EXPORTED VARIABLES
******************************************************************************/

/******************************************************************************
DEFINITION OF LOCAL CONSTANT DATA
******************************************************************************/

/******************************************************************************
DEFINITION OF EXPORTED CONSTANT DATA
******************************************************************************/

/******************************************************************************
DEFINITION OF APIs
******************************************************************************/
#define ObjFusn_START_SEC_SLOW_CODE

#include "MVSCamConverter_Cfg.h"

// /*!****************************************************************************
// * @details
// *     convert TUE Fusion object format to Conti Radar Object
// * @param[in]
// *     pOldObject input pointer to fusion object
// * @param[out]
// *     pNewObject output pointer to radar object
// * @return
// *     none
// * @constraints
// *     <>
// * @invocation_criteria
// *     <>
// ******************************************************************************/
// boolean TueFusionObjListToContiRaObjListConverter(
//     CONSTP2VAR(FusObjects_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pNewObjList,
//     CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
//         pOldObjList) {
//     // Kinematic(12)
//     pNewObjList->Kinematic.fArelX = pOldObjList->vecX.data[TRACKABLE_ACCX];
//     pNewObjList->Kinematic.fArelXStd =
//         pOldObjList->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX];
//     pNewObjList->Kinematic.fArelY = pOldObjList->vecX.data[TRACKABLE_ACCY];
//     pNewObjList->Kinematic.fArelYStd =
//         pOldObjList->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY];

// #ifdef AEB_TEST_SAFETY_DISTANCE_OPEN
//     // pNewObjList->Kinematic.fDistX = pOldObjList->vecX.data[TRACKABLE_POSX]
//     -
//     // 5;
//     pNewObjList->Kinematic.fDistX =
//         pOldObjList->vecX.data[TRACKABLE_POSX] >= 0.2f
//             ? MAX(pOldObjList->vecX.data[TRACKABLE_POSX] - 15.0f, 0.2f)
//             : pOldObjList->vecX.data[TRACKABLE_POSX];
// #else
//     pNewObjList->Kinematic.fDistX = pOldObjList->vecX.data[TRACKABLE_POSX];
// #endif
//     pNewObjList->Kinematic.fDistXStd =
//         pOldObjList->matP.data[TRACKABLE_INDEX_VARIANCE_POSX];
//     pNewObjList->Kinematic.fDistY = pOldObjList->vecX.data[TRACKABLE_POSY];
//     pNewObjList->Kinematic.fDistYStd =
//         pOldObjList->matP.data[TRACKABLE_INDEX_VARIANCE_POSY];

//     pNewObjList->Kinematic.fVrelX = pOldObjList->vecX.data[TRACKABLE_VELX];
//     pNewObjList->Kinematic.fVrelXStd =
//         pOldObjList->matP.data[TRACKABLE_INDEX_VARIANCE_VELX];
//     pNewObjList->Kinematic.fVrelY = pOldObjList->vecX.data[TRACKABLE_VELY];
//     pNewObjList->Kinematic.fVrelYStd =
//         pOldObjList->matP.data[TRACKABLE_INDEX_VARIANCE_VELY];

//     // Geometry(4)
//     pNewObjList->Geometry.fLength = MAX(pOldObjList->f32Length, 1);

//     pNewObjList->Geometry.fOrientation = pOldObjList->f32Heading;
//     pNewObjList->Geometry.fOrientationStd = pOldObjList->f32HeadingVar;

//     // Attributes(2)
//     pNewObjList->Attributes.uiClassConfidence = pOldObjList->u16ClassProb;
//     switch (pOldObjList->u16Class) {
//         case TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN:
//             pNewObjList->Attributes.eClassification =
//                 Envm_GEN_OBJECT_CLASS_UNCLASSIFIED;
//             pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width,
//                 FCT_SEN_CLASS_POINT_MIN_DIMENSION);
//             break;
//         case TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_CAR:
//             pNewObjList->Attributes.eClassification =
//             Envm_GEN_OBJECT_CLASS_CAR; pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width, FCT_SEN_CLASS_CAR_MIN_WIDTH);
//             break;
//         case TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_TRUCK:
//             pNewObjList->Attributes.eClassification =
//                 Envm_GEN_OBJECT_CLASS_TRUCK;
//             pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width, FCT_SEN_CLASS_TRUCK_MIN_WIDTH);
//             break;
//         case TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_UNKNOWN:
//             pNewObjList->Attributes.eClassification =
//                 Envm_GEN_OBJECT_CLASS_PEDESTRIAN;
//             pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width, FCT_SEN_CLASS_PED_MIN_DIMENSION);
//             pNewObjList->EBAPresel.bCrossingPedEbaPresel = TRUE;
//         case TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_CHILD:
//             pNewObjList->Attributes.eClassification =
//                 Envm_GEN_OBJECT_CLASS_PEDESTRIAN;
//             pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width, FCT_SEN_CLASS_PED_MIN_DIMENSION);
//             pNewObjList->EBAPresel.bCrossingPedEbaPresel = TRUE;
//         case TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_ADULT:
//             pNewObjList->Attributes.eClassification =
//                 Envm_GEN_OBJECT_CLASS_PEDESTRIAN;
//             pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width, FCT_SEN_CLASS_PED_MIN_DIMENSION);
//             pNewObjList->EBAPresel.bCrossingPedEbaPresel = TRUE;
//             break;
//         case TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_UNKNOWN:
//             pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width,
//                 FCT_SEN_CLASS_MOTORCYCLE_MIN_WIDTH);
//             pNewObjList->Attributes.eClassification =
//                 Envm_GEN_OBJECT_CLASS_MOTORCYCLE;
//             break;
//         case TUEOBJFUSN_TRACKABLE_U16CLASS_OBSTACLE_PATHBLOCKING:
//             pNewObjList->Attributes.eClassification =
//                 Envm_GEN_OBJECT_CLASS_WIDE;
//             pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width, FCT_SEN_CLASS_TRUCK_MIN_WIDTH);
//         default:
//             pNewObjList->Attributes.eClassification =
//                 Envm_GEN_OBJECT_CLASS_UNCLASSIFIED;
//             pNewObjList->Geometry.fWidth =
//                 MAX(pOldObjList->f32Width,
//                 FCT_SEN_CLASS_POINT_MIN_DIMENSION);
//             break;
//     }

//     switch (pOldObjList->u16MotionType) {
//         case TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN:
//             pNewObjList->Attributes.eDynamicProperty =
//                 TU_OBJECT_DYNPROP_STATIONARY;
//             break;
//         case TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING:
//             pNewObjList->Attributes.eDynamicProperty =
//             TU_OBJECT_DYNPROP_MOVING; break;
//         case TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_STOPPED:
//             pNewObjList->Attributes.eDynamicProperty =
//                 TU_OBJECT_DYNPROP_STOPPED;
//             break;
//         case
//         TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_REVERSING:
//             pNewObjList->Attributes.eDynamicProperty =
//             TU_OBJECT_DYNPROP_MOVING; break;
//         case TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_ONCOMING:
//             pNewObjList->Attributes.eDynamicProperty =
//                 TU_OBJECT_DYNPROP_ONCOMING;
//             break;
//         case TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING:
//             pNewObjList->Attributes.eDynamicProperty =
//                 TU_OBJECT_DYNPROP_CROSSING_MOVING;
//             break;
//         case TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY:
//             pNewObjList->Attributes.eDynamicProperty =
//                 TU_OBJECT_DYNPROP_STATIONARY;
//             break;
//         default:
//             pNewObjList->Attributes.eDynamicProperty =
//             TU_OBJECT_DYNPROP_MOVING; break;
//     }

//     // General(1)
//     pNewObjList->General.eObjMaintenanceState =
//         pOldObjList->eObjMaintenanceState;
//     pNewObjList->General.fLifeTime =
//         (float32)(pOldObjList->u16Age) / 1000;  // 1s = 1000ms

//     // Qualifiers(1)
//     pNewObjList->Qualifiers.fProbabilityOfExistence =
//         pOldObjList->f32ExistenceQuality / 100;
//     pNewObjList->Qualifiers.ucObstacleProbability =
//         BML_u_Round2Uint(pOldObjList->f32ObstacleProbability);
//     // SensorSpecific(1)
//     pNewObjList->SensorSpecific.fRCS = pOldObjList->fRCS;
//     // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/150 changed by
//     // guotao for the camera confirmed logic change start
//     pNewObjList->SensorSpecific.bCamConfirmed =
//         (pOldObjList->u8CyclesNoVision <
//              TUE_CAMERA_CONFIRMED_LIFECYCLE_THRESHOLD &&
//          pOldObjList->u16ClassProb == 100u);
// // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/150 changed by
// guotao
// // for the camera confirmed logic change end
// //#if ALGO_CARSIM_SWITCH
// #if 1  // temp to close camera confirm
//     pNewObjList->SensorSpecific.bCamConfirmed = TRUE;
// #endif
//     // Legacy(1)
//     pNewObjList->Legacy.uiLifeTime =
//         pOldObjList->u16Age / (TASK_CYCLE_TIME_50 * 1000);  // 1000ms = 1s
//     // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/34 changed by
//     // guotao 20200601 start
//     if (pOldObjList->u16Age -
//             pNewObjList->Legacy.uiLifeTime * TASK_CYCLE_TIME_50 * 1000 >
//         TUE_PRV_FUSION_MATH_COMPARE_TO_ZERO) {
//         pNewObjList->Legacy.uiLifeTime += 1;  // round up the life cycle
//         number
//     }
//     // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/34 changed by
//     // guotao 20200601 end

//     // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/94 changed by
//     // guotao 20200601 start
//     if (pNewObjList->SensorSpecific.bCamConfirmed &&
//         pNewObjList->Attributes.eDynamicProperty ==
//             TU_OBJECT_DYNPROP_STATIONARY &&
//         (pNewObjList->Attributes.eClassification ==
//              Envm_GEN_OBJECT_CLASS_TRUCK ||
//          pNewObjList->Attributes.eClassification ==
//              Envm_GEN_OBJECT_CLASS_CAR)) {
//         // change stationary camera confirmed object to stopped for ACC OOI
//         lost
//         // caused by dynamic property mistake
//         pNewObjList->Attributes.eDynamicProperty = TU_OBJECT_DYNPROP_STOPPED;
//     }
//     // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/94 changed by
//     // guotao 20200601 end
//     // new property ID
//     pNewObjList->ObjectId = pOldObjList->u16ID;
//     return TRUE;
// }
/*!****************************************************************************
 * @details
 *     convert Conti Radar Object to TUE Fusion object format
 * @param[in]
 *     pOldObject input pointer to radar object
 * @param[out]
 *     pNewObject output pointer to fusion object
 * @return
 *     none
 * @constraints
 *     <>
 * @invocation_criteria
 *     <>
 ******************************************************************************/
LOCAL void ContiRaObjToTrackableConverter(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pNewObject,
    CONSTP2CONST(FusObjects_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pOldObject,
    int ID,
    uint32 Age,
    uint8 uRadarID) {
    pNewObject->fRCS = pOldObject->SensorSpecific.fRCS;
    pNewObject->eObjMaintenanceState = pOldObject->General.eObjMaintenanceState;

    pNewObject->u16ID = ID;
    pNewObject->u16Age = Age;
    pNewObject->u16Lifespan = TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW;
    if (uRadarID == 0u) {
        pNewObject->u32SensorsCurr =
            TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER;
    } else if (uRadarID == 1u) {
        pNewObject->u32SensorsCurr =
            TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_LEFT;
    } else if (uRadarID == 2u) {
        pNewObject->u32SensorsCurr =
            TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_RIGHT;
    } else if (uRadarID == 3u) {
        pNewObject->u32SensorsCurr =
            TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_LEFT;
    } else if (uRadarID == 4u) {
        pNewObject->u32SensorsCurr =
            TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_RIGHT;
    } else {
        // do nothing
    }

    pNewObject->u32SensorsHist = 0u;

    /* Position */
    pNewObject->vecX.data[TRACKABLE_POSX] = pOldObject->Kinematic.fDistX;
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/13 added by guotao
// 20200428 start
#ifdef ALGO_SAFETY_DISTANCE_FOR_TEST_ONLY
    pNewObject->vecX.data[TRACKABLE_POSX] -=
        ALGO_EXTEND_SAFETY_DISTANCEX_FOR_TEST;
#endif
    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/13 added by guotao
    // 20200428 end
    pNewObject->vecX.data[TRACKABLE_POSY] = pOldObject->Kinematic.fDistY;

    /* Velocity */
    pNewObject->vecX.data[TRACKABLE_VELX] = pOldObject->Kinematic.fVrelX;
    pNewObject->vecX.data[TRACKABLE_VELY] = pOldObject->Kinematic.fVrelY;

    /* Acceleration */
    pNewObject->vecX.data[TRACKABLE_ACCX] = pOldObject->Kinematic.fArelX;
    pNewObject->vecX.data[TRACKABLE_ACCY] = pOldObject->Kinematic.fArelY;

    calculateVariances_Radar(pNewObject);

#if TUEOBJLIST_RAD_USE_ACCELERATION == STD_ON
    pNewObject->vecX.nRows = 5u; /* PRQA S 3120 */   /* fixed vector size */
    pNewObject->matP.u16Size = 5u; /* PRQA S 3120 */ /* fixed vector size */
#else
    pNewObject->vecX.nRows = 4u; /* PRQA S 3120 */   /* fixed vector size */
    pNewObject->matP.u16Size = 4u; /* PRQA S 3120 */ /* fixed vector size */
#endif

    pNewObject->u8CoordSystem = TUEOBJFUSN_U8COORDSYSTEM_REARAXLE_RELATIVE;

    /* Geometry */
    pNewObject->f32Heading = TUEOBJFUSN_TRACKABLE_F32HEADING_DEFAULT;
    pNewObject->f32HeadingVar = TUEOBJFUSN_TRACKABLE_F32HEADINGVAR_DEFAULT;
    pNewObject->f32YawRate = TUEOBJFUSN_TRACKABLE_F32YAWRATE_DEFAULT;
    pNewObject->f32YawRateVar = TUEOBJFUSN_TRACKABLE_F32YAWRATEVAR_DEFAULT;
    pNewObject->f32ExistenceQuality =
        pOldObject->Qualifiers.fProbabilityOfExistence * 100;
    pNewObject->f32ObstacleProbability = TUE_CML_MinMax(
        0.f, 100.f, (float32)pOldObject->Qualifiers.ucObstacleProbability);

    pNewObject->bUpdated = TRUE;

    pNewObject->u16Class =
        ConvertContiRadarObjClass(pOldObject->Attributes.eClassification);
    pNewObject->u16ClassProb = (uint16)(80);
    if (pNewObject->u16Class ==
            TUEOBJFUSN_TRACKABLE_U16CLASS_OBSTACLE_UNKNOWN ||
        pNewObject->u16Class == TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN) {
        // considering radar's classification is unstable,
        // change moving and oncoming radar object's class to car if radar
        // object's class is obstacle or unknown
        if (pOldObject->Attributes.eDynamicProperty ==
                TU_OBJECT_DYNPROP_MOVING ||
            pOldObject->Attributes.eDynamicProperty ==
                TU_OBJECT_DYNPROP_ONCOMING) {
            pNewObject->u16Class = TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_CAR;
        }
    } else {
        // 20200106 car, pedestrian and camera object could have a higher class
        // probability, probability deceleration every cycle.
        // for example, first input radar object is car and changed to be
        // obstacle objcet in the next 20 cycles, the track's class would
        // be car class, and change to obstacle class in the 20th cycles.
        pNewObject->u16ClassProb = (uint16)(99);  // set higher class
                                                  // probability if it's car,
                                                  // pedestrian or cycliest.
    }

    switch (pOldObject->Attributes.eDynamicProperty) {
        case TU_OBJECT_DYNPROP_MOVING:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_DRIVING;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_MOVING;
            break;
        case TU_OBJECT_DYNPROP_STATIONARY:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_ONCOMING:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_ONCOMING;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_MOVING;
            break;
        case TU_OBJECT_DYNPROP_STATIONARY_CANDITATE:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_STATIONARY;
            break;
        case TU_OBJECT_DYNPROP_UNKNOWN:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_UNKNOWN;
            break;
        case TU_OBJECT_DYNPROP_CROSSING_STATIONARY:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_STATIONARY;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_MOVING;
            break;
        case TU_OBJECT_DYNPROP_CROSSING_MOVING:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_CROSSING;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_MOVING;
            break;
        case TU_OBJECT_DYNPROP_STOPPED:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_MOVING_EGODIRECTION_STOPPED;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_STOP;
            break;
        default:
            pNewObject->u16MotionType =
                TUEOBJFUSN_TRACKABLE_U16MOTIONTYPE_UNKNOWN;
            pNewObject->u8RadarMotionTypeInput =
                TU_SENSOR_MOTIONG_TYPE_INPUT_UNKNOWN;
            break;
    }
}
/*!****************************************************************************
 * @details
 *     convert Conti Radar object class to TUE fusion object class property
 * @param[in]
 *     conti object class property
 * @param[out]
 *     fusion object class property
 * @return
 *     none
 * @constraints
 *     <>
 * @invocation_criteria
 *     <>
 ******************************************************************************/
LOCAL uint16 ConvertContiRadarObjClass(const uint8 objType) {
    /* PRQA S 3223 8 */ /* static variable requires function-scope only */
    static const uint16 CamClassLookup[] = {
        TUEOBJFUSN_TRACKABLE_U16CLASS_OBSTACLE_UNKNOWN,
        TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_CAR,
        TUEOBJFUSN_TRACKABLE_U16CLASS_VEHICLE_TRUCK,
        TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN,
        TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_MOTORCYCLE,
        TUEOBJFUSN_TRACKABLE_U16CLASS_2WHEELER_BICYCLE,
        TUEOBJFUSN_TRACKABLE_U16CLASS_OBSTACLE_PATHBLOCKING,
        TUEOBJFUSN_TRACKABLE_U16CLASS_PEDESTRIAN_ADULT,
        TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN};
    uint16 u16class;

    if ((sizeof(CamClassLookup) / sizeof(uint16)) > objType) {
        u16class = CamClassLookup[objType];
    } else {
        u16class = TUEOBJFUSN_TRACKABLE_U16CLASS_UNKNOWN;
    }

    return u16class;
}
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
boolean abProcessTracks[CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX];
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

/*!****************************************************************************
 * @details
 *     convert Conti Radar object list to TUE fusion object list
 * @param[in]
 *     pOldObjList input pointer to Radar object list
 * @param[out]
 *     pNewObjList output pointer to Fusion object list
 * @return
 *     none
 * @constraints
 *     <>
 * @invocation_criteria
 *     <>
 ******************************************************************************/
/* PRQA S 1503 1 */ /**< Init method, only external function calls */
boolean ContiRaObjListToTueObjListConverter(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pNewObjList,
    CONSTP2CONST(FusObjectList_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pOldObjList,
    const uint32 u32globalTimeStamp,
    uint8 uRadarID) {
    uint16 u16i = 0u;
    boolean bSuccess = TRUE;
    uint16 u16NumObj = 0u;
    float32 f32CorrLatency = FLT_ZERO;

    // boolean abProcessTracks[CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX];

    /* PRQA S 3223 1 */ /* static variable requires function-scope only */
    static uint16 u16updateCount = 0u;
    /* PRQA S 3223 1 */ /* static variable requires function-scope only */
    static uint32 u32radarTimeStampPrev = 0u;

    if ((NULL_PTR == pNewObjList) || (NULL_PTR == pOldObjList)) {
        bSuccess = FALSE;
    }
    /* else if(Envm_NR_PRIVOBJECTS >
     CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX)
     {
        bSuccess = FALSE;
        pNewObjList->u16NumObjects        = 0u;
        pNewObjList->u32SensorPattern     =
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER;
        pNewObjList->u16ListUpdateCounter = u16updateCount;

                    uart_debug1("object number of FPS module > object number of
     Fusion module, error!!! \n");
     }*/
    else {
        if (u32radarTimeStampPrev != pOldObjList->uiTimeStamp) {
            u16updateCount = ((u16updateCount + 1u) %
                              TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MAX);
            u32radarTimeStampPrev = (pOldObjList->uiTimeStamp);
        } else {
            /* MISRA */
        }

        /* Check sensor failure flags  */
        if (pOldObjList->eSigStatus == FALSE) {
            bSuccess = FALSE;
            pNewObjList->u16NumObjects = 0u;

        } else {
            pNewObjList->u16ListUpdateCounter = u16updateCount;

            /* correct latency using global timestamp */
            if (u32globalTimeStamp >= u32radarTimeStampPrev) {
                f32CorrLatency =
                    (float32)(u32globalTimeStamp - u32radarTimeStampPrev);
            } else {
                f32CorrLatency = (float32)(0xFFFFFFFF - u32radarTimeStampPrev +
                                           u32globalTimeStamp);
            }
            f32CorrLatency = MAX(0, MIN(1000000, (f32CorrLatency)));
            pNewObjList->f32MeasurementLatency =
                (f32CorrLatency) / TUEOBJLIST_RAD_LATENCY_TO_SECONDS /
                    TUEOBJLIST_RAD_LATENCY_TO_SECONDS +
                TUEOBJLIST_RADAR_DEFAULT_LATENCY_SECONDS;  // convert latency
                                                           // from us to s
            // pNewObjList->u32SensorPattern =
            //     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER;
            if (uRadarID == 0u) {
                pNewObjList->u32SensorPattern =
                    TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER;
            } else if (uRadarID == 1u) {
                pNewObjList->u32SensorPattern =
                    TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_LEFT;
            } else if (uRadarID == 2u) {
                pNewObjList->u32SensorPattern =
                    TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_RIGHT;
            } else if (uRadarID == 3u) {
                pNewObjList->u32SensorPattern =
                    TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_LEFT;
            } else if (uRadarID == 4u) {
                pNewObjList->u32SensorPattern =
                    TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_REAR_RIGHT;
            } else {
                // do nothing
            }

            getTracksToProcess(pOldObjList, abProcessTracks, uRadarID);

            for (u16i = 0u;
                 u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
                 u16i++) {
                if (TRUE == abProcessTracks[u16i]) {
                    ContiRaObjToTrackableConverter(
                        &(pNewObjList->aTrackable[u16NumObj]),
                        &(pOldObjList->Objects[u16i]),
                        pOldObjList->Objects[u16i].ObjectId,
                        f32CorrLatency / 1000,  // us -> ms
                        uRadarID);
                    u16NumObj++;
                } else {
                    /* Ignore this object */
                }
            }
            pNewObjList->u16NumObjects = u16NumObj;
        }
    }

    return bSuccess;
}

LOCAL boolean validateContiObject(CONSTP2CONST(FusObjects_t,
                                               AUTOMATIC,
                                               ObjFusn_VAR_NOINIT) pObject,
                                  uint8 uRadarID) {
    boolean bIsValid = TRUE;

    if (pObject->Kinematic.fDistX == 0 && pObject->Kinematic.fDistY == 0 &&
        pObject->Kinematic.fVrelX == 0 && pObject->Kinematic.fVrelY == 0 &&
        pObject->Kinematic.fArelX == 0 && pObject->Kinematic.fArelY == 0 &&
        /*pObject->SensorSpecific.fRCS == 0 &&*/ pObject->Geometry.fWidth == 0 &&
        pObject->Geometry.fLength == 0 &&
        pObject->Qualifiers.fProbabilityOfExistence == 0) {
        bIsValid = FALSE;
    }

    if (pObject->Kinematic.fDistX < -500 ||
        pObject->Kinematic.fDistX > 1138.2) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pObject->Kinematic.fDistY < -204.6 ||
        pObject->Kinematic.fDistY > 204.8) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pObject->Kinematic.fVrelX < -128.00 ||
        pObject->Kinematic.fVrelX > 128.00) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pObject->Kinematic.fVrelY < -64.00 ||
        pObject->Kinematic.fVrelY > 63.75) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pObject->SensorSpecific.fRCS < -64.00 ||
        pObject->SensorSpecific.fRCS > 63.75) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pObject->Kinematic.fArelX < -20.00 ||
        pObject->Kinematic.fArelX > 20.00) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pObject->Kinematic.fArelY < -20 || pObject->Kinematic.fArelY > 20) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pObject->Geometry.fWidth < 0.0 || pObject->Geometry.fWidth > 51.0) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (pObject->Geometry.fLength < 0.0 || pObject->Geometry.fLength > 51.0) {
        bIsValid = FALSE;
    } else {
        /* MISRA */
    }

    if (uRadarID != 0u) { // corner radar
        if (pObject->Attributes.eDynamicProperty == TU_OBJECT_DYNPROP_STATIONARY) {
            bIsValid = FALSE;
        }

        if (pObject->Qualifiers.fProbabilityOfExistence < 0.6f) {
            bIsValid = FALSE;
        }

        if (pObject->Qualifiers.ucObstacleProbability < 60u) {
            bIsValid = FALSE;
        }
    }

    return bIsValid;
}

#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
LOCAL float32 calculateDistanceClustering(
    CONSTP2CONST(RaObjType, AUTOMATIC, ObjFusn_VAR_NOINIT) pObjOne,
    CONSTP2CONST(RaObjType, AUTOMATIC, ObjFusn_VAR_NOINIT) pObjTwo) {
    float32 f32Dist = FLT_MAX;
    float32 f32DeltaX = FLT_ZERO;
    float32 f32DeltaY = FLT_ZERO;

    if ((pObjOne == NULL_PTR) || (pObjTwo == NULL_PTR)) {
        /* MISRA */
    } else {
        f32DeltaX = (pObjOne->xPos - pObjTwo->xPos);
        f32DeltaY = (pObjOne->yPos - pObjTwo->yPos);

        if (f32DeltaX < FLT_ZERO) {
            f32DeltaX = -f32DeltaX;
        } else {
            /* MISRA */
        }

        if (f32DeltaY < FLT_ZERO) {
            f32DeltaY = -f32DeltaY;
        } else {
            /* MISRA */
        }

        f32Dist =
            (TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_DIST_WEIGHT_X * f32DeltaX) +
            (TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_DIST_WEIGHT_Y * f32DeltaY);
    }

    return f32Dist;
}
#endif

#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
LOCAL uint16 findNeighbors(
    CONSTP2CONST(RaObjListType, AUTOMATIC, ObjFusn_VAR_NOINIT) pOldObjList,
    const uint16 u16IdxObj,
    uint16 au16Neighbors[],
    const uint16 au16Clusters[],
    const boolean abRelevantTracks[],
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) pu16Offset) {
    uint16 u16NumNeighbors = 0u;
    uint16 u16i = 0u;
    float32 f32Dist = FLT_MAX;
    boolean bIsNeighbor = FALSE;
    P2CONST(RaObjType, AUTOMATIC, ObjFusn_VAR_NOINIT) pCurrObj;

    /* No Initialization of au16Neighbors needed, array is assumed to be large
     * enough */

    if ((NULL_PTR == pOldObjList) || (NULL_PTR == pu16Offset)) {
        /* NULL_PTR check */
    } else if (u16IdxObj >= CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX) {
        /* Invalid Index */
    } else {
        pCurrObj = &(pOldObjList->objectsVec.objects[u16IdxObj]);

        for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
             u16i++) {
            if ((u16i != u16IdxObj) && (TRUE == abRelevantTracks[u16i])) {
                f32Dist = calculateDistanceClustering(
                    pCurrObj, &(pOldObjList->objectsVec.objects[u16i]));

                if (f32Dist < TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_DIST) {
                    bIsNeighbor =
                        array_contains_u16(au16Neighbors, (*pu16Offset), u16i);
                    u16NumNeighbors++;

                    if (((FALSE == bIsNeighbor) &&
                         ((*pu16Offset) <
                          CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX)) &&
                        ((au16Clusters[u16i] ==
                          TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_UNKNOWN) ||
                         (au16Clusters[u16i] ==
                          TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_NOISE))) {
                        au16Neighbors[*pu16Offset] = u16i;
                        (*pu16Offset)++;
                    } else {
                        /* MISRA */
                    }
                } else {
                    /* MISRA */
                }
            } else {
                /* MISRA */
            }
        }
    }

    return u16NumNeighbors;
}
#endif

/**
* \fn   void runClustering(  cosnt RaObjListType * const pOldObjList,
const uint16 au16RelevantTracks[], uint16 au16Clusters[], const uint16 u16Size,
uint16 * const pCurrLabel)
*
* \brief   Creates the match index with the help of the distance matrix.
*          Different association can be used which are set in the config
parameters.
*
* \param   pSdist_mat                 const stDistMatrix_t * const, distance
matrix
* \param   f32MatchGate               f32_t const, match gate set in the
parameter interface
* \param   u8AssociationMode          u8_t const, association mode set in the
parameter interface
* \param  [out] stMatchesArray[]   stMatchIndex_t, match index array filled
within association
* \param   u16MatchArraySize          u16_t const, size of the match index array
* \param  [out] pNumMatches        u16_t * const, number of matches found in the
association
*
* \return  TRUE (ok) or FALSE (error occured)
*/
#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
LOCAL void runClustering(CONSTP2CONST(RaObjListType,
                                      AUTOMATIC,
                                      ObjFusn_VAR_NOINIT) pOldObjList,
                         const boolean abRelevantTracks[],
                         uint16 au16Clusters[],
                         uint16 au16NumTracksPerCluster[]) {
    uint16 au16Neighbors[CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX];
    uint16 u16i = 0u;
    uint16 u16j = 0u;
    uint16 u16NumNeighbors = 0u;
    uint16 u16Offset = 0u;
    uint16 u16OldOffset = 0u;
    uint16 u16ClusterIndex = 0u;
    uint16 u16CurrentLabel = 0u;

    /* au16RelevantTracks and au16Clusters are assumed to be correctly
     * initialized */
    for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
         u16i++) {
        if (FALSE == abRelevantTracks[u16i]) {
            /* Track is not located in Region of interest, skip this one */
        } else if (au16Clusters[u16i] !=
                   TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_UNKNOWN) {
            /* Already labelled, nothing to do */
        } else {
            u16Offset = 0u;
            u16OldOffset = 0u;

            u16NumNeighbors =
                findNeighbors(pOldObjList, u16i, au16Neighbors, au16Clusters,
                              abRelevantTracks, &u16Offset);

            if (u16NumNeighbors <
                TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MIN_NUM_NEIGHBORS) {
                /* Not enough neighbors, label as noise */
                au16Clusters[u16i] =
                    TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_NOISE;
            } else {
                u16ClusterIndex = u16CurrentLabel;

                /* Get new label for cluster */
                u16CurrentLabel++;

                /* Assign label to track */
                au16Clusters[u16i] = u16CurrentLabel;
                au16NumTracksPerCluster[u16ClusterIndex]++;

                /* Process all neighboring tracks */
                for (u16j = 0u; u16j < u16Offset; u16j++) {
                    if (au16Clusters[au16Neighbors[u16j]] ==
                        TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_NOISE) {
                        /* Track previously labelled as noise -> assumed to be
                         * boundary point of the cluster */
                        au16Clusters[au16Neighbors[u16j]] = u16CurrentLabel;
                        au16NumTracksPerCluster[u16ClusterIndex]++;
                    } else if (
                        au16Clusters[au16Neighbors[u16j]] !=
                        TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_UNKNOWN) {
                        /* Already part of a different cluster, do nothing */
                    } else {
                        /* Label neighbor to be part of the cluster */
                        au16Clusters[au16Neighbors[u16j]] = u16CurrentLabel;
                        au16NumTracksPerCluster[u16ClusterIndex]++;

                        /* Find neighbors */
                        u16OldOffset = u16Offset;
                        u16NumNeighbors = findNeighbors(
                            pOldObjList, au16Neighbors[u16j], au16Neighbors,
                            au16Clusters, abRelevantTracks, &u16Offset);

                        if (u16NumNeighbors <
                            TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MIN_NUM_NEIGHBORS) {
                            /* Only add neighboring points to seed set if enough
                             * neighbors -> density check */
                            u16Offset = u16OldOffset;
                        } else {
                            /* MISRA */
                        }
                    }
                }
            }
        }
    }
}
#endif

#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
LOCAL void findRelevantTracks(CONSTP2CONST(RaObjListType,
                                           AUTOMATIC,
                                           ObjFusn_VAR_NOINIT) pRdrObjList,
                              boolean abRelevantTracks[],
                              const boolean abValidTracks[]) {
    uint16 u16i = 0u;

    float32 f32PosX = FLT_ZERO;
    float32 f32PosY = FLT_ZERO;
    uint8 u8MotionType = 0u;

    if (NULL_PTR == pRdrObjList) {
    } else {
        for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
             u16i++) {
            abRelevantTracks[u16i] = FALSE;
        }

        for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
             u16i++) {
            f32PosX = pRdrObjList->objectsVec.objects[u16i].xPos;
            f32PosY = pRdrObjList->objectsVec.objects[u16i].yPos;
            u8MotionType =
                pRdrObjList->objectsVec.objects[u16i].objMotionPattern;

            if (f32PosY < FLT_ZERO) {
                f32PosY = -f32PosY;
            } else {
                /* MISRA */
            }

            if ((((f32PosX > TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_OFFSET_X) &&
                  (f32PosX <
                   (TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_OFFSET_X +
                    TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LENGTH))) &&
                 ((f32PosY > TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_OFFSET_Y) &&
                  (f32PosY <
                   (TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_OFFSET_Y +
                    TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_WIDTH)))) &&
                ((TRUE == abValidTracks[u16i]) &&
                 (u8MotionType == RaObjMotionPattern_STATIONARY))) {
                abRelevantTracks[u16i] = TRUE;
            } else {
                /* MISRA */
            }
        }
    }
}
#endif

#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
LOCAL uint16 runLinearRegression(CONSTP2CONST(RaObjListType,
                                              AUTOMATIC,
                                              ObjFusn_VAR_NOINIT) pRdrObjList,
                                 const uint16 au16Clusters[],
                                 const uint16 au16NumTracksPerCluster[],
                                 float32 af32CoefficientsB0[],
                                 float32 af32CoefficientsB1[]) {
    float32 af32MeanX[TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS];
    float32 af32MeanY[TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS];
    uint16 au16TracksPerClusterTmp
        [TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS];

    /* Tmp arrays to store nominator and denominator when calculating
     * coefficient b1 */
    float32 af32CoefficientsB1_Nominator
        [TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS];
    float32 af32CoefficientsB1_Denominator
        [TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS];

    uint16 u16i = 0u;
    uint16 u16ClusterLabel = 0u;
    uint16 u16ClusterIndex = 0u;
    float32 f32x = FLT_ZERO;
    float32 f32y = FLT_ZERO;
    uint16 u16NumLines = 0u;
    float32 f32Slope = FLT_ZERO;
    float32 f32OffsetY = FLT_ZERO;

    if (NULL_PTR == pRdrObjList) {
    } else {
        for (u16i = 0u;
             u16i < TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS;
             u16i++) {
            af32MeanX[u16i] = FLT_ZERO;
            af32MeanY[u16i] = FLT_ZERO;
            au16TracksPerClusterTmp[u16i] = 0u;
            af32CoefficientsB0[u16i] = FLT_ZERO;
            af32CoefficientsB1[u16i] = FLT_ZERO;
            af32CoefficientsB1_Nominator[u16i] = FLT_ZERO;
            af32CoefficientsB1_Denominator[u16i] = FLT_ZERO;
        }

        /* First iteration to calcualte mean values */
        for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
             u16i++) {
            u16ClusterLabel = au16Clusters[u16i];

            if (((u16ClusterLabel !=
                  TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_UNKNOWN) &&
                 (u16ClusterLabel !=
                  TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_NOISE)) &&
                (u16ClusterLabel <
                 TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS)) {
                /* Valid labels start with 1u, subtract 1u to map to array index
                 */
                u16ClusterIndex = u16ClusterLabel - 1u;

                if (au16NumTracksPerCluster[u16ClusterIndex] <
                    TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MIN_TRACKS_PER_CLUSTER) {
                    /* MISRA */
                } else if (au16TracksPerClusterTmp[u16ClusterIndex] == 0u) {
                    af32MeanX[u16ClusterIndex] =
                        pRdrObjList->objectsVec.objects[u16i].xPos;
                    af32MeanY[u16ClusterIndex] =
                        pRdrObjList->objectsVec.objects[u16i].yPos;

                    au16TracksPerClusterTmp[u16ClusterIndex]++;
                } else {
                    af32MeanX[u16ClusterIndex] +=
                        (pRdrObjList->objectsVec.objects[u16i].xPos -
                         af32MeanX[u16ClusterIndex]) /
                        (((float32)au16TracksPerClusterTmp[u16ClusterIndex]) +
                         FLT_ONE);

                    af32MeanY[u16ClusterIndex] +=
                        (pRdrObjList->objectsVec.objects[u16i].yPos -
                         af32MeanY[u16ClusterIndex]) /
                        (((float32)au16TracksPerClusterTmp[u16ClusterIndex]) +
                         FLT_ONE);

                    au16TracksPerClusterTmp[u16ClusterIndex]++;
                }

            } else {
                /* MISRA */
            }
        }

        /* Now run second iteration to calculate linear approximation y = b_0 +
         * b_1 * x */
        for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
             u16i++) {
            u16ClusterLabel = au16Clusters[u16i];

            if (((u16ClusterLabel !=
                  TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_UNKNOWN) &&
                 (u16ClusterLabel !=
                  TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_NOISE)) &&
                (u16ClusterLabel <
                 TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS)) {
                u16ClusterIndex = u16ClusterLabel - 1u;

                if (au16NumTracksPerCluster[u16ClusterIndex] <
                    TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MIN_TRACKS_PER_CLUSTER) {
                    /* MISRA */
                } else {
                    f32x = pRdrObjList->objectsVec.objects[u16i].xPos;
                    f32y = pRdrObjList->objectsVec.objects[u16i].yPos;

                    /* Calculate coefficient b_1 as b_1 = (sum (x_i * mean_x) *
                     * (y_i * mean_y)) / (sum (x_i - mean_x) * (x_i - mean_x))
                     */
                    af32CoefficientsB1_Nominator[u16ClusterIndex] +=
                        (f32x - af32MeanX[u16ClusterIndex]) *
                        (f32y - af32MeanY[u16ClusterIndex]);
                    af32CoefficientsB1_Denominator[u16ClusterIndex] +=
                        (f32x - af32MeanX[u16ClusterIndex]) *
                        (f32x - af32MeanX[u16ClusterIndex]);
                }
            } else {
                /* MISRA */
            }
        }

        /* Calcualte coefficients b1 and b0 */
        for (u16i = 0u;
             u16i < TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS;
             u16i++) {
            if (au16NumTracksPerCluster[u16i] <
                TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MIN_TRACKS_PER_CLUSTER) {
                /* MISRA */
            } else if (af32CoefficientsB1_Denominator[u16i] > FLT_EPSILON) {
                f32Slope = af32CoefficientsB1_Nominator[u16i] /
                           af32CoefficientsB1_Denominator[u16i];
                f32OffsetY = (af32MeanY[u16i]) - (f32Slope * (af32MeanX[u16i]));

                if (f32Slope < FLT_ZERO) {
                    f32Slope = -f32Slope;
                } else {
                    /* MISRA */
                }

                if (f32OffsetY < FLT_ZERO) {
                    f32OffsetY = -f32OffsetY;
                } else {
                    /* MISRA */
                }

                /* Reject lines not parallel to the ego-vehicle */
                if ((f32Slope <
                     TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_SLOPE) &&
                    (f32OffsetY >
                     TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_OFFSET_Y)) {
                    af32CoefficientsB1[u16NumLines] =
                        (af32CoefficientsB1_Nominator[u16i]) /
                        (af32CoefficientsB1_Denominator[u16i]);
                    af32CoefficientsB0[u16NumLines] =
                        (af32MeanY[u16i]) -
                        ((af32CoefficientsB1[u16i]) * (af32MeanX[u16i]));
                    u16NumLines++;
                } else {
                    /* MISRA */
                }
            } else {
                /* MISRA */
            }
        }
    }

    return u16NumLines;
}
#endif

LOCAL void getTracksToProcess(CONSTP2CONST(FusObjectList_t,
                                           AUTOMATIC,
                                           ObjFusn_VAR_NOINIT) pRdrObjList,
                              boolean abProcessTracks[],
                              uint8 uRadarID) {
    uint16 u16i = 0u;

#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
    boolean abIsValid[CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX];
#endif

    if (NULL_PTR == pRdrObjList) {
        /* MISRA */
    } else {
        /* Initialize arrays used for road boundary detection */
        for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
             u16i++) {
#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
            if (((pRdrObjList->statusFlags).hostYaw) <
                TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_YAW) {
                abIsValid[u16i] = validateContiObject(
                    &(pRdrObjList->objectsVec.objects[u16i]));
            } else {
                abProcessTracks[u16i] = validateContiObject(
                    &(pRdrObjList->objectsVec.objects[u16i]));
            }
#else
            abProcessTracks[u16i] =
                validateContiObject(&(pRdrObjList->Objects[u16i]), uRadarID);
#endif
        }

#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
        if (((pRdrObjList->statusFlags).hostYaw) <
            TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_YAW) {
            runRoadBoundaryDetection(pRdrObjList, abIsValid, abProcessTracks);
        } else {
            /* MISRA */
        }
#endif
    }
}

#if STD_ON == TUEOBJLIST_RAD_ENABLE_ROAD_BOUNDARY_DETECTION
LOCAL void runRoadBoundaryDetection(
    CONSTP2CONST(RaObjListType, AUTOMATIC, ObjFusn_VAR_NOINIT) pRdrObjList,
    const boolean abValidTracks[],
    boolean abProcessTracks[]) {
    boolean abRelevantTracks[CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX];
    uint16 au16Clusters[CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX];
    uint16
        au16NumTracksPerCluster[CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX];
    float32 af32CoefficientsB0
        [TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS];
    float32 af32CoefficientsB1
        [TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS];
    float32 af32MaxDistance
        [TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_NUM_CLUSTERS];

    float32 f32PosX = FLT_ZERO;
    float32 f32PosY = FLT_ZERO;
    float32 f32PosY_Model = FLT_ZERO;

    uint16 u16i = 0u;
    uint16 u16j = 0u;
    uint16 u16NumLines = 0u;

    /* Initialize arrays used for road boundary detection */
    for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
         u16i++) {
        abRelevantTracks[u16i] = FALSE;
        au16Clusters[u16i] =
            TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_LABEL_UNKNOWN;
        au16NumTracksPerCluster[u16i] = 0u;
        abProcessTracks[u16i] = TRUE;
    }

    if (NULL_PTR == pRdrObjList) {
        /* MISRA */
    } else {
        /* Now search for relevant tracks in road boundary area */
        findRelevantTracks(pRdrObjList, abRelevantTracks, abValidTracks);

        /* run Clustering */
        runClustering(pRdrObjList, abRelevantTracks, au16Clusters,
                      au16NumTracksPerCluster);

        /* Run simple linear regression for line fitting */
        u16NumLines = runLinearRegression(
            pRdrObjList, au16Clusters, au16NumTracksPerCluster,
            af32CoefficientsB0, af32CoefficientsB1);

        /* Calculate a maximum distance (longitudinal) for each line, i.e.
         * tracks beyond this distance are not removed */
        for (u16i = 0u; u16i < u16NumLines; u16i++) {
            /* Check if approximated line will intersect with boundary of ROI,
             * i.e. positive offset (left of vehicle) but  */
            /* negative slope intersects ROI at some longitudinal distance */
            if (((af32CoefficientsB0[u16i] > FLT_ZERO) &&
                 ((af32CoefficientsB1[u16i]) < FLT_ZERO)) ||
                ((af32CoefficientsB0[u16i] < FLT_ZERO) &&
                 ((af32CoefficientsB1[u16i]) > FLT_ZERO))) {
                af32MaxDistance[u16i] =
                    (TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_OFFSET_Y -
                     af32CoefficientsB0[u16i]) /
                    af32CoefficientsB1[u16i];
            } else {
                af32MaxDistance[u16i] =
                    CONTI_RADAR_CONVERTER_OBJECTS_F32POSX_MAX;
            }
        }

        /* Check each track to be valid and within detected ROI, i.e. not beyond
         * road boundaries */
        for (u16i = 0u; u16i < CONTI_RADAR_CONVERTER_OBJECTS_U8NUMOBJECTS_MAX;
             u16i++) {
            if (FALSE == abValidTracks[u16i]) {
                abProcessTracks[u16i] = FALSE;
            } else {
                f32PosX = pRdrObjList->objectsVec.objects[u16i].xPos;
                f32PosY = pRdrObjList->objectsVec.objects[u16i].yPos;

                /* Now check all available lines */
                for (u16j = 0u;
                     (u16j < u16NumLines) && (TRUE == abProcessTracks[u16i]);
                     u16j++) {
                    f32PosY_Model = (af32CoefficientsB0[u16j]) +
                                    ((af32CoefficientsB1[u16j]) * f32PosX);

                    if (((f32PosY > FLT_ZERO) && (f32PosY_Model > FLT_ZERO)) &&
                        (f32PosX < af32MaxDistance[u16j])) {
                        /** ToDo: should we check for f32PosY_Model >
                        TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_OFFSET_Y or
                        f32PosY_Model > FLT_ZERO ?? */
                        if (f32PosY > f32PosY_Model) {
                            abProcessTracks[u16i] = FALSE;
                        } else if (
                            (f32PosY_Model - f32PosY) <
                            TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_DIST_TO_LINE) {
                            abProcessTracks[u16i] = FALSE;
                        } else {
                            /* MISRA */
                        }
                    } else if (((f32PosY < FLT_ZERO) &&
                                (f32PosY_Model < FLT_ZERO)) &&
                               (f32PosX < af32MaxDistance[u16j])) {
                        if (f32PosY < f32PosY_Model) {
                            abProcessTracks[u16i] = FALSE;
                        } else if (
                            (f32PosY - f32PosY_Model) <
                            TUEOBJLIST_RAD_ROAD_BOUNDARY_DETECTION_MAX_DIST_TO_LINE) {
                            abProcessTracks[u16i] = FALSE;
                        } else {
                            /* MISRA */
                        }
                    } else {
                        /* MISRA */
                    }
                }
            }
        }
    }
}

LOCAL boolean array_contains_u16(const uint16 u16Arr[],
                                 const uint16 u16ArrSize,
                                 const uint16 u16Element) {
    boolean bFound = FALSE;
    uint16 u16i = 0u;

    for (u16i = 0u; (u16i < u16ArrSize) && (bFound == FALSE); u16i++) {
        if (u16Arr[u16i] == u16Element) {
            bFound = TRUE;
        } else {
            /* MISRA */
        }
    }

    return bFound;
}

#endif

LOCAL void calculateVariances_Radar(CONSTP2VAR(TueObjFusn_TrackableType,
                                               AUTOMATIC,
                                               ObjFusn_VAR_NOINIT) pRdrObj) {
    float32 f32PosX = FLT_ZERO;
    float32 f32PosY = FLT_ZERO;
    float32 f32VarX = FLT_ZERO;
    float32 f32VarY = FLT_ZERO;
    float32 f32VarVx = FLT_ZERO;
    float32 f32VarVy = FLT_ZERO;
    float32 f32VarAx = FLT_ZERO;
    float32 f32Angle = FLT_ZERO;
    float32 f32Sin = FLT_ZERO;
    float32 f32Cos = FLT_ZERO;

    if (NULL_PTR != pRdrObj) {
        f32Angle = tue_prv_fusion_atan2(f32PosY, f32PosX);
        f32Sin = tue_prv_fusion_sin(f32Angle);
        f32Cos = tue_prv_fusion_cos(f32Angle);

        f32PosX = pRdrObj->vecX.data[TRACKABLE_POSX];
        f32PosY = pRdrObj->vecX.data[TRACKABLE_POSY];

        f32VarX = TUEOBJLIST_RAD_SENSOR_MODEL_POS_X_BETA_0;
        f32VarX += TUEOBJLIST_RAD_SENSOR_MODEL_POS_X_BETA_1 * f32PosX;
        f32VarX += TUEOBJLIST_RAD_SENSOR_MODEL_POS_X_BETA_2 * f32PosY;
        f32VarX +=
            TUEOBJLIST_RAD_SENSOR_MODEL_POS_X_BETA_3 * (f32PosX * f32PosX);
        f32VarX +=
            TUEOBJLIST_RAD_SENSOR_MODEL_POS_X_BETA_4 * (f32PosY * f32PosY);
        f32VarX +=
            TUEOBJLIST_RAD_SENSOR_MODEL_POS_X_BETA_5 * (f32PosX * f32PosY);

        f32VarY = TUEOBJLIST_RAD_SENSOR_MODEL_POS_Y_BETA_0;
        f32VarY += TUEOBJLIST_RAD_SENSOR_MODEL_POS_Y_BETA_1 * f32PosX;
        f32VarY += TUEOBJLIST_RAD_SENSOR_MODEL_POS_Y_BETA_2 * f32PosY;
        f32VarY +=
            TUEOBJLIST_RAD_SENSOR_MODEL_POS_Y_BETA_3 * (f32PosX * f32PosX);
        f32VarY +=
            TUEOBJLIST_RAD_SENSOR_MODEL_POS_Y_BETA_4 * (f32PosY * f32PosY);
        f32VarY +=
            TUEOBJLIST_RAD_SENSOR_MODEL_POS_Y_BETA_5 * (f32PosX * f32PosY);

        f32VarVx = TUEOBJLIST_RAD_SENSOR_MODEL_VEL_X_BETA_0;
        f32VarVx += TUEOBJLIST_RAD_SENSOR_MODEL_VEL_X_BETA_1 * f32PosX;
        f32VarVx += TUEOBJLIST_RAD_SENSOR_MODEL_VEL_X_BETA_2 * f32PosY;
        f32VarVx +=
            TUEOBJLIST_RAD_SENSOR_MODEL_VEL_X_BETA_3 * (f32PosX * f32PosX);
        f32VarVx +=
            TUEOBJLIST_RAD_SENSOR_MODEL_VEL_X_BETA_4 * (f32PosY * f32PosY);
        f32VarVx +=
            TUEOBJLIST_RAD_SENSOR_MODEL_VEL_X_BETA_5 * (f32PosX * f32PosY);

        f32VarVy = TUEOBJLIST_RAD_SENSOR_MODEL_VEL_Y_BETA_0;
        f32VarVy += TUEOBJLIST_RAD_SENSOR_MODEL_VEL_Y_BETA_1 * f32PosX;
        f32VarVy += TUEOBJLIST_RAD_SENSOR_MODEL_VEL_Y_BETA_2 * f32PosY;
        f32VarVy +=
            TUEOBJLIST_RAD_SENSOR_MODEL_VEL_Y_BETA_3 * (f32PosX * f32PosX);
        f32VarVy +=
            TUEOBJLIST_RAD_SENSOR_MODEL_VEL_Y_BETA_4 * (f32PosY * f32PosY);
        f32VarVy +=
            TUEOBJLIST_RAD_SENSOR_MODEL_VEL_Y_BETA_5 * (f32PosX * f32PosY);

        f32VarAx = TUEOBJLIST_RAD_SENSOR_MODEL_ACC_X_BETA_0;
        f32VarAx += TUEOBJLIST_RAD_SENSOR_MODEL_ACC_X_BETA_1 * f32PosX;
        f32VarAx += TUEOBJLIST_RAD_SENSOR_MODEL_ACC_X_BETA_2 * f32PosY;
        f32VarAx +=
            TUEOBJLIST_RAD_SENSOR_MODEL_ACC_X_BETA_3 * (f32PosX * f32PosX);
        f32VarAx +=
            TUEOBJLIST_RAD_SENSOR_MODEL_ACC_X_BETA_4 * (f32PosY * f32PosY);
        f32VarAx +=
            TUEOBJLIST_RAD_SENSOR_MODEL_ACC_X_BETA_5 * (f32PosX * f32PosY);

        pRdrObj->matP.data[TRACKABLE_INDEX_VARIANCE_POSX] =
            tue_prv_fusion_min_max_F32(f32VarX,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_POS_X_MIN,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_POS_X_MAX);

        pRdrObj->matP.data[TRACKABLE_INDEX_VARIANCE_POSY] =
            tue_prv_fusion_min_max_F32(f32VarY,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_POS_Y_MIN,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_POS_Y_MAX);

        pRdrObj->matP.data[TRACKABLE_INDEX_VARIANCE_VELX] =
            tue_prv_fusion_min_max_F32(f32VarVx,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_VEL_X_MIN,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_VEL_X_MAX);

        pRdrObj->matP.data[TRACKABLE_INDEX_VARIANCE_VELY] =
            tue_prv_fusion_min_max_F32(f32VarVy,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_VEL_Y_MIN,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_VEL_Y_MAX);

        pRdrObj->matP.data[TRACKABLE_INDEX_VARIANCE_ACCX] =
            tue_prv_fusion_min_max_F32(f32VarAx,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_ACC_X_MIN,
                                       TUEOBJLIST_RAD_SENSOR_MODEL_ACC_X_MAX);

        pRdrObj->matP.data[TRACKABLE_INDEX_VARIANCE_ACCY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCX] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCX] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCX] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCX] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_ACCY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_ACCY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_ACCY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELY_ACCY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_ACCX_ACCY] = FLT_ZERO;
        pRdrObj->matP.u16Size = 5u; /* PRQA S 3120 */ /* fixed vector size */

        (void)f32RotateSymMat(&(pRdrObj->matP), &(pRdrObj->matP), f32Sin,
                              f32Cos);

        /* Reset Covariances as negative co-variances may result in negative
         * variances in LKF*/
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_POSY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELX] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELX] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSX_VELY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_POSY_VELY] = FLT_ZERO;
        pRdrObj->matP.data[TRACKABLE_INDEX_COVARIANCE_VELX_VELY] = FLT_ZERO;
    } else {
        /* MISRA */
    }
}

#define ObjFusn_STOP_SEC_SLOW_CODE
