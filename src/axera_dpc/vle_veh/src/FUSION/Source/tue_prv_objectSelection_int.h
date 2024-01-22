/** \addtogroup Object Selection
 *  @{
 * \file        tue_prv_objectSelection_int.h
 *
 *
 *
 * <br>=====================================================<br>
 * <b>Copyright 2013 by Tuerme.</b>
 *
 *  All rights reserved. Property of Tuerme.<br>
 *  Restricted rights to use, duplicate or disclose of this code<br>
 *  are granted through contract.
 * <br>=====================================================<br>
 */

#ifndef TUE_PRV_OBJECTSELECTION_INT_H
#define TUE_PRV_OBJECTSELECTION_INT_H

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/

#include "TueObjFusn_TrackableType.h"
/*==================[macros]================================================*/

/**
 * Number of bins used in object selection algorithm
 */
#define TUE_PRV_OBJECTSELECTION_MAX_BINS (14u)

#define TUE_PRV_OBJECT_SELECTION_SENSOR_PATTERN_FUSED \
    (TUEOBJFUSN_TRACKABLE_U32SENSOR_MVS_FRONT |       \
     TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR_FRONT_CENTER)

/**
 * define numbers for object distance binning
 */
#define TUE_PRV_OBJECTSELECTION_GLOBAL_OBSZONE_BIN (0u)
#define TUE_PRV_OBJECTSELECTION_RADARONLY_BIN (1u)
#define TUE_PRV_OBJECTSELECTION_OBSZONE1_BIN (2u)
#define TUE_PRV_OBJECTSELECTION_OBSZONE2_BIN (3u)
#define TUE_PRV_OBJECTSELECTION_OBSZONE3_BIN (4u)
#define TUE_PRV_OBJECTSELECTION_OBSZONE4_BIN (5u)

#define TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE1_BIN (6u)
#define TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE2_BIN (7u)
#define TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE3_BIN (8u)
#define TUE_PRV_OBJECTSELECTION_FUSED_OBSZONE4_BIN (9u)

#define TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE1_BIN (10u)
#define TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE2_BIN (11u)
#define TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE3_BIN (12u)
#define TUE_PRV_OBJECTSELECTION_NOTCOASTED_OBSZONE4_BIN (13u)

/*==================[type definitions]======================================*/

/*==================[functions]============================================*/

#define ObjFusn_START_SEC_CODE

LOCAL uint32 calculateObjectRange(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pCurTrkbl,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) f32Range);

LOCAL uint32 calculateObjectAngle(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pCurTrkbl,
    CONSTP2VAR(float32, AUTOMATIC, ObjFusn_VAR_NOINIT) f32Angle);

LOCAL uint32 setTrackableBinPosition(CONSTP2VAR(TueObjFusn_TrackableType,
                                                AUTOMATIC,
                                                ObjFusn_VAR_NOINIT) pCurTrkbl);

LOCAL uint32 setDroppedTrackableBins(
    CONSTP2VAR(uint8, AUTOMATIC, ObjFusn_VAR_NOINIT) u8LastBinToDelete,
    VAR(sint16, ObjFusn_VAR_NOINIT) s16NumberTrackablesToDelete);

LOCAL void initTrackableBinCounter(void);

#define ObjFusn_STOP_SEC_CODE

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif /* TUE_PRV_COMMON_MATRIX_H */
       /**@}==================[end of
        * file]===========================================*/
