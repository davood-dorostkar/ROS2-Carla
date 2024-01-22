/** \addtogroup objectlist
 *  @{
 * \file    TueObjFusn_TrackableListProps.h
 * \brief  This is a structure definition file for the internal object fusion
 * list
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

#ifndef TUEOBJFUSN_OBJECT_LIST_TYPE_H_
#define TUEOBJFUSN_OBJECT_LIST_TYPE_H_

/*==================[inclusions]============================================*/
// #include "envm_ext.h"
// #include "envm_consts.h"
#include "tue_common_libs.h"
#include "tue_prv_common_types.h"
#include "TueObjFusn_ObjectListProps.h"
#include "TueObjFusn_TrackableType.h"
#include "stddef.h"
// #include "TM_Global_Types.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/

/**
 * Object list for external Fusion interface
 */
typedef struct TueObjFusn_ObjectListTypeTag {
    uint16 u16ListUpdateCounter; /** Counter that is incremented whenever a
                                    module outputs an updated object (starting
                                    from 0, maximum value is 60.000, counter
                                    will overflow to 0 afterwards) */
    uint16 u16NumObjects;        /** Number of trackables which have a non-dying
                                    lifespan */
    uint32 u32SensorPattern;     /** Sensor pattern */
    float32
        f32MeasurementLatency; /** Common age [s] of all object estimations
                                  within the data array: positive latencies
                                  indicate outdated information; negative
                                  latencies indicate predicted object data. */
    TueObjFusn_TrackableType
        aTrackable[TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX]; /** object array
                                                                with limited
                                                                capacity */
} TueObjFusn_ObjectListType;

#endif /**\} TUEOBJFUSN_OBJECT_LIST_TYPE_H_ */