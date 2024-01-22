
#ifndef SFOBJECTFUSIONMPF_FUNC_H_
#define SFOBJECTFUSIONMPF_FUNC_H_

/******************************************************************************

AUTOLIV ELECTRONIC document.

-------------------------------------------------------------------------------

Copyright Tuerme Inc. All rights reserved.

*******************************************************************************
H-File Template Version:
*******************************************************************************

Overview of the interfaces:
   Input interface: Radar, Camera Object lists and Egomotion structs
   Output interface: RaCam Object list

******************************************************************************/
/* PRQA S 0288 ++ */
/*
 * Explanation:
 *    Header file for s-function wrapper Fusion Core in CADS4 simulink
 */
/*
 */
/* PRQA S 0288 -- */
/*****************************************************************************/
/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "fusion_ext.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"
//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
#include "tue_prv_common_types.h"

#include "tue_Fusion.h"
#include "TueObjFusn_TrackableListUtils.h"

#include "TueRACAM_ObjListInput.h"
#include "TueObjFusn_EgoMotionType.h"
#include "tue_prv_common_object.h"
#include "TueObjFusn_ObjectListType.h"

#include "ContiRadarConverter.h"
#include "MVSCamConverter.h"
#include "TueObjFusn_TrackableProps.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
DEFINITION OF CONSTANTS
******************************************************************************/

/******************************************************************************
DECLARATION OF TYPES
******************************************************************************/

/******************************************************************************
DECLARATION OF VARIABLES
******************************************************************************/

/******************************************************************************
DECLARATION OF CONSTANT DATA
******************************************************************************/

/******************************************************************************
DECLARATION OF FUNCTIONS
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
                                    const uint32 u32CoastedSensor);

void sfObjectFusionMpf_Outputs_wrapper(
    const Fusion_Radar_input_lists *const pRadarObjLists_Input,
    const CamObjectList *const pCamObjLists_Input,
    const TueObjFusn_EgoMotionType *const EgoMotion,
    const uint32 u32TimeStamp,
    TueObjFusn_ObjectListType *const pFusnObjLists_Output,
    // TueObjFusn_ObjectListType *const pFusnObjLists_ManaIDOutput,
    TueObjFusn_ErrorBufferType *const pErrorBuffer);

/************* GAC new feature *************/
void Fusion_postprocess(TueObjFusn_ObjectListType *const pFusnObjLists_Output);
void Fusion_IDmanage(const TueObjFusn_ObjectListType *const pFusnObjLists_Output,
                        TueObjFusn_ObjectListType *const pFusnObjLists_ManaIDOutput);
/************* GAC new feature *************/

/******************************************************************************
DECLARATION OF FUNCTION-LIKE MACROS
******************************************************************************/

#ifdef __cplusplus
}
#endif

/******************************************************************************
End Of File
*****************************************************************************/

#endif /* _SFOBJECTFUSIONRACAM_WRAPPER_H_ */