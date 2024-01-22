/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wen mingshu <wen mingshu@senseauto.com>
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

#ifndef TUEOBJFUSN_OBJECTLISTPROPS_H_
#define TUEOBJFUSN_OBJECTLISTPROPS_H_

/*==================[inclusions]============================================*/
#include "TueObjFusn_ConfigConstants.h"

/*==================[macros]================================================*/

/********************/
/* u32SensorPattern */
/********************/
/// \name u32SensorPattern
/** minimum value */
#define TUEOBJFUSN_OBJECTLIST_U32SENSORPATTERN_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_OBJECTLIST_U32SENSORPATTERN_MAX (0xFFFFFFFFu)
/** default value */
#define TUEOBJFUSN_OBJECTLIST_U32SENSORPATTERN_DEFAULT (0xFFFFFFFFu)
/** flag indicating if this field will be filled by sensor */
#define TUEOBJFUSN_OBJECTLIST_U32SENSORPATTERN_FILLEDBYSENSOR \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_VISION)
/// \}

/*************************/
/* f32MeasurementLatency */
/*************************/
/// \name f32MeasurementLatency
/** minimum value */
#define TUEOBJFUSN_OBJECTLIST_F32MEASUREMENTLATENCY_MIN (-5.0f)
/** maximum value */
#define TUEOBJFUSN_OBJECTLIST_F32MEASUREMENTLATENCY_MAX (5.0f)
/** default value */
#define TUEOBJFUSN_OBJECTLIST_F32MEASUREMENTLATENCY_DEFAULT (-1000.0f)
/** flag indicating if this field will be filled by sensor */
#define TUEOBJFUSN_OBJECTLIST_F32MEASUREMENTLATENCY_FILLEDBYSENSOR \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_VISION)
/// \}

/********************/
/* u16NumObjects */
/********************/
/// \name u16NumObjects
/** minimum value */
#define TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MIN (0u)
/** maximum value for */
#define TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_MAX \
    (TUE_PRV_FUSION_OBJECT_LIST_SIZE)
/** default value for */
#define TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_DEFAULT (61234u)
/** flag indicating if this field will be filled by sensor */
#define TUEOBJFUSN_OBJECTLIST_U16NUMOBJECTS_FILLEDBYSENSOR \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_VISION)
///\}

/************************/
/* u16ListUpdateCounter */
/************************/
/// \name u16ListUpdateCounter
/** minimum value */
#define TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MIN (0u)
/** maximum value */
#define TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_MAX (60000u)
/** default value */
#define TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_DEFAULT (61234u)
/** flag indicating if this field will be filled by sensor */
#define TUEOBJFUSN_OBJECTLIST_U16LISTUPDATECOUNTER_FILLEDBYSENSOR \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_VISION)
/// \}

#endif /* TUEOBJFUSN_OBJECTLISTPROPS_H_ */

/** @} */
