/** \addtogroup objectlist
 *  \{
 * \file    TueObjFusn_TrackableList.h
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

#ifndef TUEOBJFUSN_TRACKABLELIST_H_
#define TUEOBJFUSN_TRACKABLELIST_H_

#include "tue_prv_common_types.h"
#include "TueObjFusn_TrackableType.h"
#include "TueObjFusn_TrackableListProps.h"

/*==================[macros]================================================*/
/*==================[type definitions]======================================*/

/** Variable-sized list of trackables.
 * the trackable list represents object and track lists
 */
typedef struct TueObjFusn_TrackableListTypeTag {
    uint16 u16ListUpdateCounter; /** Counter that is incremented whenever a
                                    module outputs an updated object (starting
                                    from 0, maximum value is 60.000, counter
                                    will overflow to 0 afterwards) */
    uint16 u16ValidTrackables;   /** Number of trackables which have a non-dying
                                    lifespan */
    float32
        f32MeasurementLatency; /** Common age [s] of all object estimations
                                  within the data array: positive latencies
                                  indicate outdated information; negative
                                  latencies indicate predicted object data. */
    uint32 u32SensorsCurr; /** Sensor pattern indicating which sensor updated
                              the internal trackable list in the current cycle.
                              Shall be reset at the beginning of each cycle */
    sint16 as16TrackableMap
        [TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX]; /** mapping of used
                                                              trackables, -1 is
                                                              used if there are
                                                              no more tracks*/
    TueObjFusn_TrackableType aTrackable
        [TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX]; /** object array
                                                              with limited
                                                              capacity */
} TueObjFusn_TrackableListType;

#endif /**\} TUEOBJFUSN_TRACKABLELIST_H_ */
