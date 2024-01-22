/*
* Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
* wen mingshu <wenmingshu@senseauto.com>
*/
/** \addtogroup tueFusion
 *  \{
 * \file TueObjFusn_Assertions.h
 *
 * \brief Checks common \ global constant values needed for Fusion.
 *
 *
 *
 *
 *
 *   (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_ASSERTIONS_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_ASSERTIONS_H_

#include "tue_prv_common_types.h"

#include "TueObjFusn_ConfigAlgorithm.h"
#include "TueObjFusn_ConfigVehicle.h"
#include "TueObjFusn_TrackableListProps.h"
#include "TueObjFusn_TrackableProps.h"

/** the object list shall be large enough for worst case RADAR + CAM (max # of
 * objects from both sensors, no objects fused */
#if ((TUE_PRV_FUSION_MAX_OBJECTS_RADAR + TUE_PRV_FUSION_MAX_OBJECTS_VISION + \
      TUE_PRV_FUSION_MAX_OBJECTS_RADAR_LEFT_FRONT +                          \
      TUE_PRV_FUSION_MAX_OBJECTS_RADAR_LEFT_REAR +                           \
      TUE_PRV_FUSION_MAX_OBJECTS_RADAR_RIGHT_FRONT +                         \
      TUE_PRV_FUSION_MAX_OBJECTS_RADAR_RIGHT_REAR) >                         \
     TUE_PRV_FUSION_MAX_SENSOR_OBJECTS)
#error "Fusion requires TUE Object List to be large enough for worst case"
#endif

/**
 * Verify that the ID space is large enough to provide new ID's in worst case
 * OOSM handling situations (reprocessing with re-setup of tracks for full
 * sensor
 * lists from all supported sensors).
 */
#if (                                                                   \
    (TUEOBJFUSN_TRACKABLE_U16ID_MAX - TUEOBJFUSN_TRACKABLE_U16ID_MIN) < \
    (TUE_PRV_FUSION_MAX_SENSOR_OBJECTS /** (TUE_PRV_FUSION_MAX_INPUTS + 1)*/))
#error \
    "ID management space must be large enough to provide new ID's for all tracks that could be setup!"
#endif

#if (TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX < TUE_PRV_FUSION_MAX_INPUT_OBJECTS)
#error \
    "Number of tracks for input/output list not sufficient, increase TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX"
#endif

/* require enough tracks */
#if (TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX < \
     (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW *           \
      TUE_PRV_FUSION_MAX_SENSOR_OBJECTS))
#error \
    "Number of filter tracks too low for current number of objects. increase TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX!"
#endif
/* ASSERTION: TUEOBJFUSN_OBJLISTINPUT_U16ID_DEFAULT must not be assigned by
 * fusion id management */
#if (TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT) <= (TUEOBJFUSN_TRACKABLE_U16ID_MAX)
#error "TUEOBJFUSN_TRACKABLE_U16ID_DEFAULT must be larger than MAX_FUSION_ID!"
#endif

/** ASSERTION: require enough fusion IDs to be able to be assigned */
#if (TUEOBJFUSN_TRACKABLE_U16ID_MAX + 1) <= \
    (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW * TUE_PRV_FUSION_MAX_SENSOR_OBJECTS)
#error "not enough fusion Ids available for current setup! "
#endif

/** ASSERTION: 'U16LIFESPAN_FREE_SLOT' and 'U16LIFESPAN_DEFAULT' must be set to
 * identical values */
#if (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_FREE_SLOT != \
     TUEOBJFUSN_TRACKABLE_U16LIFESPAN_DEFAULT)
#error "Default and 'free-slot' markers ofr lifespan must be identical"
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_INCLUDE_TUEOBJFUSN_ASSERTIONS_H_
