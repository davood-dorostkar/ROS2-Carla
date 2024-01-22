/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_WRAPPER_OBJECTFUSIONMPFCONVERTER_SFOBJECTFUSIONMPFCONVERTER_FUNC_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_WRAPPER_OBJECTFUSIONMPFCONVERTER_SFOBJECTFUSIONMPFCONVERTER_FUNC_H_

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
 *    Header file for s-function wrapper RaCam converter in CADS4 simulink
 */
/*
 */
/* PRQA S 0288 -- */
/*****************************************************************************/
/******************************************************************************
EXTERNAL DEPENDENCIES
******************************************************************************/
#include "envm_ext.h"
#include "envm_consts.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"

#include "Fusion_Std_Types.h"
#include "TueObjFusn_ObjectListType.h"
#include "TueRACAM_ObjListInput.h"
#include "TueObjFusn_EgoMotionType.h"
// #include "ObjFusnObjListToSnsrFusnConvert.h"

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

void sfObjectFusionMpfConverter_Init_wrapper();

boolean sfObjectFusionMpfConverter_Outputs_wrapper(
    TueObjFusn_ObjectListType *const pFusnObjLists_Input,
    TueObjFusn_EgoMotionType *const pEgoMotion,
    const uint32 u32TimeStamp,
    typeRaCamObjectVcc *const pRACAMObjListOut);

/******************************************************************************
DECLARATION OF FUNCTION-LIKE MACROS
******************************************************************************/

#ifdef __cplusplus
}
#endif

/******************************************************************************
End Of File
*****************************************************************************/

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_WRAPPER_OBJECTFUSIONMPFCONVERTER_SFOBJECTFUSIONMPFCONVERTER_FUNC_H_
