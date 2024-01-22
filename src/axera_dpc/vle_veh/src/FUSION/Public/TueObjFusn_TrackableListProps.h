/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * zhang guanglin <zhang guanglin@senseauto.com>
 */
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

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_TRACKABLELISTPROPS_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_TRACKABLELISTPROPS_H_

/*==================[inclusions]============================================*/
#include "TueObjFusn_ConfigConstants.h"

/*==================[macros]================================================*/

/******************/
/* u32SensorsCurr */
/******************/
/// \name u32SensorsCurr
/** minimum value */
#define TUEOBJFUSN_TRACKABLELIST_U32SENSORSCURR_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_TRACKABLELIST_U32SENSORSCURR_MAX (0xFFFFFFFFu)
/** default value */
#define TUEOBJFUSN_TRACKABLELIST_U32SENSORSCURR_DEFAULT (0xFFFFFFFFu)
/// \}

/*************************/
/* f32MeasurementLatency */
/*************************/
/// \name f32MeasurementLatency
/** minimum value */
#define TUEOBJFUSN_TRACKABLELIST_F32MEASUREMENTLATENCY_MIN (-5.0f)
/** maximum value */
#define TUEOBJFUSN_TRACKABLELIST_F32MEASUREMENTLATENCY_MAX (5.0f)
/** default value */
#define TUEOBJFUSN_TRACKABLELIST_F32MEASUREMENTLATENCY_DEFAULT (-1000.0f)
/// \}

/**********************/
/* u16ValidTrackables */
/**********************/
/// \name u16ValidTrackables
/** minimum value */
#define TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MIN (0u)
/** maximum value dependent on coasting value*/
#define TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX \
    (TUE_PRV_FUSION_TRACKABLE_LIST_SIZE)
/** default value */
#define TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_DEFAULT (65234u)
/// \}

/********************/
/* as16TrackableMap */
/********************/
/// \name as16Indexas16TrackableMapMap_
/** minimum value */
#define TUEOBJFUSN_TRACKABLELIST_AS16TRACKABLEMAP_MIN (-1)
/** maximum value */
#define TUEOBJFUSN_TRACKABLELIST_AS16TRACKABLEMAP_MAX \
    (TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX)
/** default value */
#define TUEOBJFUSN_TRACKABLELIST_AS16TRACKABLEMAP_DEFAULT (-1)
/// \}

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_PUBLIC_TUEOBJFUSN_TRACKABLELISTPROPS_H_

/** @} */
