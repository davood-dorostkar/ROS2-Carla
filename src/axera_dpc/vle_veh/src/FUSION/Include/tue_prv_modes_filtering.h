/** \addtogroup tuePrvmodes
 *  @{
 * \file        tue_prv_modes_filtering.h
 * \brief header file for tue_prv_modes_filtering.c
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_MODES_FILTERING_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_MODES_FILTERING_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_common_types.h"
#include "TueObjFusn_ObjectListType.h"
#include "TueObjFusn_EgoMotionType.h"

/*==================[macros]================================================*/

/*==================[type definitions]======================================*/
/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/
/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

#define ObjFusn_START_SEC_CODE

/** initializer method */
extern void modes_filtering_init(void);

extern uint32 modes_filtering_preCycle(const float32 f32dt,
                                       CONSTP2CONST(TueObjFusn_EgoMotionType,
                                                    AUTOMATIC,
                                                    ObjFusn_VAR_NOINIT)
                                           pEgoMotion);

uint32 modes_filtering_cycle_SensorEvent(CONSTP2VAR(
    TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT) pObjectList);

extern uint32 modes_filtering_cycle_TimerEvent(CONSTP2VAR(
    TueObjFusn_ObjectListType, AUTOMATIC, ObjFusn_VAR_NOINIT) pObjList_Output);

extern uint32 modes_filtering_postCycle(const float32 f32dt,
                                        const uint32 u32SensorsUpdatePattern);

extern uint32 modes_filtering_removeDeadSensor(const uint32 u32Sensor);

#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUE_PRV_MODES_FILTERING_H_
        /**@}==================[end of
         * file]===========================================*/
