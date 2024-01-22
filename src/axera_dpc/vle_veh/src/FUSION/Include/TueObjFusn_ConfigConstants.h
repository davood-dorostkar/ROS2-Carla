/** \addtogroup config
 *  @{ */
//* \file        TueObjFusn_ConfigConstants.h
//*
//*
//*
//* <br>=====================================================<br>
//* <b>Copyright 2016 by Tuerme.</b>
//*
//*  All rights reserved. Property of Tuerme.<br>
//*  Restricted rights to use, duplicate or disclose of this code<br>
//*  are granted through contract.
//* <br>=====================================================<br>
//*/

#ifndef TUEOBJFUSN_CONFIGCONSTANTS_H
#define TUEOBJFUSN_CONFIGCONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/

#include "TueObjFusn_ConfigAlgorithm.h"
#include "TueObjFusn_ConfigVehicle.h"
#include "TueObjFusn_TrackableConstants.h"

/*==================[macros]================================================*/

/**
 *    Maximum number of objects inputted in a single cycle assuming that
 *    both sensors provide inputs in the same cycle and no "fused" track is
 * created
 */
#define TUE_PRV_FUSION_MAX_SENSOR_OBJECTS                                   \
    (TUE_PRV_FUSION_MAX_OBJECTS_RADAR + TUE_PRV_FUSION_MAX_OBJECTS_VISION + \
     TUE_PRV_FUSION_MAX_OBJECTS_RADAR_LEFT_FRONT +                          \
     TUE_PRV_FUSION_MAX_OBJECTS_RADAR_LEFT_REAR +                           \
     TUE_PRV_FUSION_MAX_OBJECTS_RADAR_RIGHT_FRONT +                         \
     TUE_PRV_FUSION_MAX_OBJECTS_RADAR_RIGHT_REAR)

/* Maximum number of input objects */
#if (TUE_PRV_FUSION_MAX_OBJECTS_RADAR < TUE_PRV_FUSION_MAX_OBJECTS_VISION)
#define TUE_PRV_FUSION_MAX_INPUT_OBJECTS (TUE_PRV_FUSION_MAX_OBJECTS_VISION)
#else
#define TUE_PRV_FUSION_MAX_INPUT_OBJECTS (TUE_PRV_FUSION_MAX_OBJECTS_RADAR)
#endif

/*
 * Minimum size of internal trackable list assuming that max input objects are
 * provided and not updated in the next cycle
 */
#define TUE_PRV_FUSION_TRACKABLE_LIST_SIZE \
    (TUEOBJFUSN_TRACKABLE_U16LIFESPAN_NEW * TUE_PRV_FUSION_MAX_SENSOR_OBJECTS)

#if (STD_ON == TUE_PRV_OBJECT_SELCTION_ENABLE_OBJECT_SELECTION_FOR_OUTPUT)
#define TUE_PRV_FUSION_MAX_OUTPUT_OBJECTS \
    (TUE_PRV_OBJECT_SELECTION_NUMBER_OF_OUTPUT_OBJECTS)
#else
#define TUE_PRV_FUSION_MAX_OUTPUT_OBJECTS (TUE_PRV_FUSION_TRACKABLE_LIST_SIZE)
#endif

/*
 *    Minimum size of external object list interface.
 *    Can be considered as max(max(number of radar, number of vision), number of
 * output objects).
 */
#if (TUE_PRV_FUSION_MAX_INPUT_OBJECTS > TUE_PRV_FUSION_MAX_OUTPUT_OBJECTS)
#define TUE_PRV_FUSION_OBJECT_LIST_SIZE (TUE_PRV_FUSION_MAX_INPUT_OBJECTS)
#else
#define TUE_PRV_FUSION_OBJECT_LIST_SIZE (TUE_PRV_FUSION_MAX_OUTPUT_OBJECTS)
#endif

/*==================[type definitions]======================================*/
/*==================[functions]============================================*/
/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUEOBJFUSN_CONFIGCONSTANTS_H */
       /**@}==================[end of
        * file]===========================================*/