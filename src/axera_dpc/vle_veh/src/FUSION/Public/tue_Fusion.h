/** \addtogroup tueFusion
 *  @{
 * \file tue_Fusion.h
 *
 * \brief Inerface for  tue_Fusion.c Main functions
 *
 * \todo list the todo list for this file
 *
 * (C) Copyright Tuerme Inc. All rights reserved.
 */

/*****************************************************************************
COMPANY:                Tuerme
PROJECT-ID:             -
CPU:                    -
COMPONENT:              -
MODULE:                 -
FILENAME:               Tue_Fusion.h
DESCRIPTION:            -
ORIGINATOR:             -
DATE OF CREATION:       -
*****************************************************************************/
#ifndef __TUE_FUSION_H__
#define __TUE_FUSION_H__

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
   INCLUDES
*****************************************************************************/
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
// #include "TM_Global_Types.h"
//#include "Fusion_Std_Types.h"
#include "Std_Types.h"
#include "FusionCompiler.h"
#include "TueObjFusn_ObjectListType.h"
#include "TueObjFusn_EgoMotionType.h"
#include "TueObjFusn_ErrorBufferType.h"

#include "TueObjFusn_ParameterInterface.h"
#include "TueObjFusn_Version.h"

#define ObjFusn_START_SEC_SLOW_CODE

/** Initializes all fusion components.
 * Call this method once prior to executing any fusion cycle.
 * returns TRUE on success */
void Fusion_init(void);
#define ObjFusn_STOP_SEC_SLOW_CODE

#define ObjFusn_START_SEC_CODE

/**
 * @brief Fusion Interface for timer event. To be called everey xx ms.
 *
 * @param[out]    pTrackableList_Output,  new ego motion element
 * @param[in]     f32dt,       age to be added to existing elements in buffer
 * [s]
 * @param[out]    pErrorBuffer, pointer to error buffer. Internal error buffer
 * is copied to this address.
 * @return      TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
uint32 Fusion_performTimerEvent(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectList_out,
    const float32 f32dt,
    CONSTP2VAR(TueObjFusn_ErrorBufferType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pErrorBuffer);

/**
 * @brief   Start Fusion Cycle
 *
 * @param[in]  f32dT, Measurement Latency
 * return      TUEOBJFUSN_ERROR_NOERROr if no error occured and error code
 * otherwise.
 */
uint32 Fusion_startCycle(const uint32 au32SensPatList[],
                         const uint16 u16NumLists,
                         const float32 f32dt,
                         CONSTP2CONST(TueObjFusn_EgoMotionType,
                                      AUTOMATIC,
                                      ObjFusn_VAR_NOINIT) pEgoMotion);

/** @brief Perform one fusion cycle
 * The first input of object fusion is an array to pointers of object lists.
 * Each object list originates from one physical sensor. The order of sensors
 * in the array may be arbitrary. The list may be NULL-terminated but currently
 * only up to two input lists are taken into account (fusion is taylored for 2
 * sensors).
 * f_dt must be greater than zero.
 * fusion cycle fills the memory region of a single object list pointed to
 * by the second parameter.
 * Additionally the Debug Output list may or may not be filled. This output is
 * untested. However, make sure to provide the required memory because the
 * PerformFusionCycle method may write data to this region.
 */

uint32 PerformFusionCycle(
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectLists_Input[],  // PRQA S 1503
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion,
    const float32 f32dt,
    CONSTP2VAR(TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pObjectList_Output,
    CONSTP2VAR(TueObjFusn_ErrorBufferType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pErrorBuffer);

uint32 Fusion_PerformSensorEvent(CONSTP2VAR(TueObjFusn_ObjectListType,
                                            AUTOMATIC,
                                            ObjFusn_VAR_NOINIT) pObjList);

#define ObjFusn_STOP_SEC_CODE

#ifdef __cplusplus
}
#endif

#endif /**@} __TUE_FUSION_H__ */
