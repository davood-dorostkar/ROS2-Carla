/******************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------

Copyright Tuerme Inc. All rights reserved.

*******************************************************************************
C-File Template Version:
******************************************************************************/
/* PRQA S 0288 ++ */
/*
 * Explanation:
 *    Source file for s-function wrapper Fusion Core in CADS4 simulink
 */
/*
 */
/* PRQA S 0288 -- */
/*!****************************************************************************

@details
   <Describes details of this module 'Template.c' file within overall
    context of component implementation>

******************************************************************************/
/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/
// #include "envm_ext.h"
// #include "envm_consts.h"
// #include "tue_common_libs.h"
#include "sfObjectFusionMpf_func.h"
// #include "TM_Global_Types.h"
#include "stddef.h"

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
DEFINITION OF EXPORTED VARIABLES
******************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static sint32 iFusionObjIDManageArray[TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX * 2];
static sint32 iFusionObjIndexManageArray[TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX];
static uint16 uLifeCycle[TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX] = {0u};
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/******************************************************************************
DEFINITION OF LOCAL CONSTANT DATA
******************************************************************************/

/******************************************************************************
DEFINITION OF EXPORTED CONSTANT DATA
******************************************************************************/

/******************************************************************************
MODULE FUNCTION-LIKE MACROS
******************************************************************************/

/******************************************************************************
DEFINITION OF LOCAL FUNCTION
******************************************************************************/

/******************************************************************************
DEFINITION OF APIs
******************************************************************************/

/*!****************************************************************************
 * @details
 *     Initialize TUE object fusion with input parameters
 * @param[in]
 *     const uint32 u32SensorMode
 * @param[in]
 *     const uint8  u8FusionMode
 * @param[in]
 *     const float32 f32PostPrediction
 * @param[in]
 *     const uint8  u8WeightMode
 * @param[in]
 *     const float32 f32MatchGate
 * @param[in]
 *     const uint8 u8OosmMode
 * @param[in]
 *     const uint8 u8AssociationMode
 * @param[in]
 *     const boolean bUseForMotionAccX
 * @param[in]
 *     const boolean bUseForMotionAccY
 * @param[out]
 *     none
 * @return
 *     none
 * @constraints
 *     <>
 * @invocation_criteria
 *     <>
 ******************************************************************************/

void sfObjectFusionMpf_Init_wrapper(const uint32 u32SensorMode,
                                    const float32 f32MatchGate,
                                    const float32 f32PedestrianVarianceInXForQ,
                                    const float32 f32PedestrianVarianceInYForQ,
                                    const float32 f32VehicleVarianceInXForQ,
                                    const float32 f32VehicleVarianceInYForQ,
                                    const boolean bUseTrackMerge,
                                    const float32 f32AdditionalQonDiagonal,
                                    const boolean bOutputIsOverground,
                                    const boolean bUseCoasting,
                                    const uint32 u32CoastedSensor) {
    uint32 u32Success = TRUE;

    Fusion_init();

    u32Success = Fusion_set_u32SensorMode(u32SensorMode);
    u32Success = Fusion_set_f32MatchGate(20.f /*f32MatchGate*/);

    u32Success = Fusion_set_f32PedestrianVarianceInXForQ(
        0.01 /*f32PedestrianVarianceInXForQ*/);
    u32Success = Fusion_set_f32PedestrianVarianceInYForQ(
        0.01 /*f32PedestrianVarianceInYForQ*/);

    u32Success = Fusion_set_f32VehicleVarianceInXForQ(
        0.01 /*f32VehicleVarianceInXForQ*/);
    u32Success = Fusion_set_f32VehicleVarianceInYForQ(
        0.01 /*f32VehicleVarianceInYForQ*/);

    u32Success = Fusion_set_bUseTrackMerge(TRUE /*bUseTrackMerge*/);

    u32Success = Fusion_set_bOutputIsOverground(FALSE /*bOutputIsOverground*/);
    u32Success = Fusion_set_bUseCoasting(TRUE /*bUseCoasting*/);

    for (uint8 i = 0u; i < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; i++){
        uLifeCycle[i] = 0u;
    }
    memset(iFusionObjIDManageArray, -1,
           sizeof(sint32) * (TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX * 2));
    memset(iFusionObjIndexManageArray, -1,
           sizeof(sint32) * TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX);
#if (defined(CAMERA_MINIEYE) && CAMERA_MINIEYE == 1)
    OtherCarLocHistoryInit();  // Init Other car loaction history for other car
                               // velocity calculation

    // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/31 changed by
    // changchang 20200603 start
    PedestrianLocHistoryInit();  // Init pedestrian loaction history for
                                 // pedestrian position and velocity calculation
// Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/31 changed by
// changchang 20200603 end
#endif
}

/*!****************************************************************************
 * @details
 *     Call input adapters, Object Fusion and output adapter
 * @param[in]
 *     const stTueObjectList_t *pCamObjLists_Input
 * @param[in]
 *     const stTueObjectList_t *pRadarObjLists_Input
 * @param[in]
 *     TueObjFusn_EgoMotionType *EgoMotion
 * @param[in]
 *     TueObjFusn_EgoMotionType *EgoMotion
 * @param[out]
 *     const uint32 u32TimeStamp
 * @param[out]
 *     stRaCamObjectVcc_t *pRACAMObjListOut
 * @return
 *     none
 * @constraints
 *     <>
 * @invocation_criteria
 *     <>
 ******************************************************************************/
// #define SENSORFUSION_SWC_START_SEC_VAR_FAST_CLEARED_UNSPECIFIED
// #include "SensorFusion_Swc_MemMap.h"
void sfObjectFusionMpf_Outputs_wrapper(
    const Fusion_Radar_input_lists *const pRadarObjLists_Input,
    const CamObjectList *const pCamObjLists_Input,
    const TueObjFusn_EgoMotionType *const EgoMotion,
    const uint32 u32TimeStamp,
    TueObjFusn_ObjectListType *const pFusnObjLists_Output,
    // TueObjFusn_ObjectListType *const pFusnObjLists_ManaIDOutput,
    TueObjFusn_ErrorBufferType *const pErrorBuffer) {
    float32 CycleDeltaTime;
    uint32 u32Success = 0u;
    static uint32 ui32_OldExecTime = 0;
    static TueObjFusn_ObjectListType *inObjListVector[7];
    /* array of pointers to lists */ /* only internal, unused in Simulink
                                        implementation */
    static TueObjFusn_ObjectListType
        radarObjectsOut[MULTI_SENSOR_RADAR_FUSION_QUANTITY];
    static TueObjFusn_ObjectListType camObjectsOut;

    // init camera and radar object list for the Fusion function
    Trackable_initObjectList(&camObjectsOut);
    // Trackable_initObjectList(&radarObjectsOut);
    Trackable_initObjectList(pFusnObjLists_Output);

    // converte input original radar/camera object structure to fusion needed
    // structure
    MVSObjListToTueObjListConverter(&camObjectsOut, pCamObjLists_Input,
                                    u32TimeStamp);
    // ContiRaObjListToTueObjListConverter(&radarObjectsOut,
    // pRadarObjLists_Input,
    //                                     u32TimeStamp);

    for (uint8 index = 0u; index < MULTI_SENSOR_RADAR_FUSION_QUANTITY;
         index++) {
        Trackable_initObjectList(&radarObjectsOut[index]);
        ContiRaObjListToTueObjListConverter(
            &radarObjectsOut[index],
            &pRadarObjLists_Input->Radarobjlists[index], u32TimeStamp, index);
        inObjListVector[index + 1] = &radarObjectsOut[index];
    }

    // set camera/radar object list for fusion input array
    // if (pCamObjLists_Input == NULL) {
    //     inObjListVector[0] =
    //         &radarObjectsOut[0];  // camera object list is null,
    //                               // use radar object in case
    //                               // error happed in fusion
    // } else {
    inObjListVector[0] = &camObjectsOut;
    // }
    // inObjListVector[1] = &radarObjectsOut;
    inObjListVector[6] =
        (TueObjFusn_ObjectListType *)NULL_PTR; /* the fusion cycle method
                                                  expects the input list to be
                                                  NULL terminated (unless n ==
                                                  N) */

    if (ui32_OldExecTime == 0) {
        CycleDeltaTime = TASK_CYCLE_TIME_50;
    } else {
        if (u32TimeStamp >= ui32_OldExecTime) {
            CycleDeltaTime =
                (float32)(u32TimeStamp - ui32_OldExecTime) / (1000 * 1000);
        } else {
            CycleDeltaTime =
                (float32)(0xFFFFFFFF - ui32_OldExecTime + u32TimeStamp) /
                (1000 * 1000);
        }
    }
    ui32_OldExecTime = u32TimeStamp;
    CycleDeltaTime = MIN(TASK_CYCLE_TIME_50 * 3,
                         MAX(TASK_CYCLE_TIME_50 * 0.1, CycleDeltaTime));
    // // excute fusion process
    u32Success = PerformFusionCycle(inObjListVector, EgoMotion, CycleDeltaTime,
                                    pFusnObjLists_Output, pErrorBuffer);
    // EnvmData.pECAMtCyclEnvmode->fECAMtCycleTime = CycleDeltaTime;

    /************* GAC new feature *************/
    // Fusion_IDmanage(pFusnObjLists_Output, pFusnObjLists_ManaIDOutput);
    Fusion_postprocess(pFusnObjLists_Output);
    /************* GAC new feature *************/
}
// #define SENSORFUSION_SWC_STOP_SEC_VAR_FAST_CLEARED_UNSPECIFIED
// #include "SensorFusion_Swc_MemMap.h"
/************* new feature *************/
void Fusion_postprocess(TueObjFusn_ObjectListType *const pFusnObjLists_Output) {
    // uint16 uObjunmber = pFusnObjLists_Output->u16NumObjects;
    for(uint8 index = 0u; index < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; index++) {
        // uFusionSts output
        // if((pFusnObjLists_Output->aTrackable[index].au16SensorID[0] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT)
        //     && (pFusnObjLists_Output->aTrackable[index].au16SensorID[1] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
        //         || pFusnObjLists_Output->aTrackable[index].au16SensorID[2] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
        //         || pFusnObjLists_Output->aTrackable[index].au16SensorID[3] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
        //         || pFusnObjLists_Output->aTrackable[index].au16SensorID[4] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
        //         || pFusnObjLists_Output->aTrackable[index].au16SensorID[5] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT)){
        //             pFusnObjLists_Output->aTrackable[index].uFusionSts = 0x1; // fusion
        // } else if(pFusnObjLists_Output->aTrackable[index].au16SensorID[0] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT) {
        //     pFusnObjLists_Output->aTrackable[index].uFusionSts = 0x0; // only camera
        // } else if(pFusnObjLists_Output->aTrackable[index].au16SensorID[1] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
        //         || pFusnObjLists_Output->aTrackable[index].au16SensorID[2] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
        //         || pFusnObjLists_Output->aTrackable[index].au16SensorID[3] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
        //         || pFusnObjLists_Output->aTrackable[index].au16SensorID[4] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
        //         || pFusnObjLists_Output->aTrackable[index].au16SensorID[5] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT) {
        //     pFusnObjLists_Output->aTrackable[index].uFusionSts = 0x0; // only radar
        //         }
        // else {
        //     pFusnObjLists_Output->aTrackable[index].uFusionSts = 0x0; // no
        // }
        uint8 ufusionstatus_t = 0u;
        if (pFusnObjLists_Output->aTrackable[index].au16SensorID[0] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT) {
            if (pFusnObjLists_Output->aTrackable[index].au16SensorID[1] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                || pFusnObjLists_Output->aTrackable[index].au16SensorID[2] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                || pFusnObjLists_Output->aTrackable[index].au16SensorID[3] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                || pFusnObjLists_Output->aTrackable[index].au16SensorID[4] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                || pFusnObjLists_Output->aTrackable[index].au16SensorID[5] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT) {
                    ufusionstatus_t = 1u; // fusion_v
            } else {
                ufusionstatus_t = 3u; // only_camera
            }
        } else if(pFusnObjLists_Output->aTrackable[index].au16SensorID[1] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[2] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[3] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[4] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[5] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT) {
                        if((pFusnObjLists_Output->aTrackable[index].au16SensorID[1] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                && (pFusnObjLists_Output->aTrackable[index].au16SensorID[2] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[3] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[4] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[5] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT)) ||
                            (pFusnObjLists_Output->aTrackable[index].au16SensorID[2] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                && (pFusnObjLists_Output->aTrackable[index].au16SensorID[3] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[4] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[5] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT)) ||
                            (pFusnObjLists_Output->aTrackable[index].au16SensorID[3] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                && (pFusnObjLists_Output->aTrackable[index].au16SensorID[4] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                    || pFusnObjLists_Output->aTrackable[index].au16SensorID[5] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT)) ||
                            (pFusnObjLists_Output->aTrackable[index].au16SensorID[4] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT
                                && (pFusnObjLists_Output->aTrackable[index].au16SensorID[5] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT))) {
                                       ufusionstatus_t = 2u; // fusion_R
                        } else {
                            if (pFusnObjLists_Output->aTrackable[index].au16SensorID[1] != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT) {
                                ufusionstatus_t = 4u; // only_front_radar
                            } else {
                                ufusionstatus_t = 5u; // only_corner_radar
                            }
                        }
        } else {
            ufusionstatus_t = 0u; // invalid
        }
        
        // IDrecord temp to store ufusionstatus_t
        pFusnObjLists_Output->aTrackable[index].uIDrecord = pFusnObjLists_Output->aTrackable[index].au16SensorID[0];
        pFusnObjLists_Output->aTrackable[index].uFusionSts = ufusionstatus_t;

        // uLifeCycle
        if (pFusnObjLists_Output->aTrackable[index].u16Lifespan != 0u &&
            pFusnObjLists_Output->aTrackable[index].uLifeCycle < 61234u) {
            uLifeCycle[index] += 1u;
        } else {
            uLifeCycle[index] = 0u;
        }
        pFusnObjLists_Output->aTrackable[index].uLifeCycle = uLifeCycle[index];
    }
}

void Fusion_IDmanage(const TueObjFusn_ObjectListType *const pFusnObjLists_Output,
                        TueObjFusn_ObjectListType *const pFusnObjLists_ManaIDOutput) {
    boolean iFusionCurrentCycleObjIDFlag[TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX * 2] = {0};
    pFusnObjLists_ManaIDOutput->f32MeasurementLatency = pFusnObjLists_Output->f32MeasurementLatency;
    pFusnObjLists_ManaIDOutput->u16ListUpdateCounter = pFusnObjLists_Output->u16ListUpdateCounter;
    pFusnObjLists_ManaIDOutput->u16NumObjects = pFusnObjLists_Output->u16NumObjects;
    pFusnObjLists_ManaIDOutput->u32SensorPattern = pFusnObjLists_Output->u32SensorPattern;
    // if the num of input objects is 0, memset fusion outputs are 0
    if (pFusnObjLists_Output->u16NumObjects == 0) {
        for (uint8 i = 0u; i < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; i++) {
            Trackable_init(&pFusnObjLists_ManaIDOutput->aTrackable[i]);
        }
    } else {
        // Step1 : find if object ID exits in the last cycle
        for (uint8 i = 0u; i < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; i++) {
            boolean bFindTarget = FALSE;
            for (uint8 j = 0; j < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX * 2; j++) {
                if (pFusnObjLists_Output->aTrackable[i].u16ID ==
                    iFusionObjIDManageArray[j]) {
                    bFindTarget = TRUE;
                    iFusionCurrentCycleObjIDFlag[j] = 1;
                    break;
                }
            }
            // find vacant index for increasing ID
            if ((!bFindTarget) && (pFusnObjLists_Output->aTrackable[i].u16ID != TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT)) {
                for (uint8 k = 0u; k < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX * 2; k++) {
                    if (iFusionObjIDManageArray[k] == -1) {
                        iFusionObjIDManageArray[k] =
                            pFusnObjLists_Output->aTrackable[i].u16ID;
                        iFusionCurrentCycleObjIDFlag[k] = 1;
                        break;
                    }
                }
            }
        }
        // step2 : delet ID of non-existent Last cycles' objects in current cycle
        for (uint8 i = 0u; i < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX * 2; i++) {
            if (iFusionCurrentCycleObjIDFlag[i] != 1) {
                iFusionObjIDManageArray[i] = -1;
            }
        }
        // step3 : delet index of non-existent Last cycles' objects in current cycle
        for (uint8 i = 0u; i < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; i++) {
            if (iFusionCurrentCycleObjIDFlag[iFusionObjIndexManageArray[i]] ==
                    0 &&
                iFusionObjIndexManageArray[i] != -1) {
                iFusionObjIndexManageArray[i] = -1;
            }
        }
        // step4 : find if object index exits in the last cycle
        for (uint8 i = 0u; i < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX * 2; i++) {
            boolean bfindIndex = FALSE;
            for (uint8 j = 0u; j < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; j++) {
                if (i == iFusionObjIndexManageArray
                             [j] &&  // iObjIDManageArray[i]
                    iFusionObjIDManageArray[i] != -1) {
                    bfindIndex = TRUE;
                    // iFusionObjIndexManageArray[j] = i;
                    break;
                }
            }
            // store the not match obj into iFusionObjIndexManageArray
            if (!bfindIndex) {
                for (uint8 k = 0u; k < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; k++) {
                    if (iFusionObjIDManageArray[i] != -1 &&
                        iFusionObjIndexManageArray[k] == -1) {
                        iFusionObjIndexManageArray[k] = i;
                        break;
                    }
                }
            }
        }

        // clear all the object data before result output
        for (uint8 i = 0u; i < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; i++) {
            Trackable_init(&pFusnObjLists_ManaIDOutput->aTrackable[i]);
        }

        // output to pFusnObjLists_ManaIDOutput from pFusnObjLists_Output
        for (uint8 i = 0u; i < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; i++) {
            for (uint8 j = 0u; j < TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX; j++) {
                if (iFusionObjIndexManageArray[j] != -1 &&
                    iFusionObjIDManageArray[iFusionObjIndexManageArray[j]] ==
                        pFusnObjLists_Output->aTrackable[i].u16ID) {
                    // pFusnObjLists_ManaIDOutput->aTrackable[j].u16ID =
                    //     pFusnObjLists_Output->aTrackable[i].u16ID;
                    memcpy(&pFusnObjLists_ManaIDOutput->aTrackable[j],
                            &pFusnObjLists_Output->aTrackable[i],
                            sizeof(TueObjFusn_TrackableType));
                    break;
                }
            }
        }
    }
}
/************* new feature *************/

//#define SENSORFUSION_SWC_STOP_SEC_VAR_CLEARED_UNSPECIFIED
//#include "SensorFusion_Swc_MemMap.h"
/******************************************************************************
End Of File
*****************************************************************************/