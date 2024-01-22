/** \addtogroup tuePrvGainEstimation
 * @{
 * \file        tue_prv_gainEstimation_int.h
 * \brief       internal header for gain estimation
 *
 *
 *
 *
 *          (C) Copyright Tuerme Inc. All rights reserved.
 *
 */

#ifndef SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_GAINESTIMATION_INT_H_
#define SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_GAINESTIMATION_INT_H_

#ifdef __cplusplus
extern "C" {
#endif

/*==================[inclusions]============================================*/
#include "tue_prv_gainEstimation.h"  // for structure stRelativeStates
#include "TueObjFusn_ConfigAlgorithm.h"

/*==================[type definitions]======================================*/

#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON

typedef struct {
    float32 f32PosXRel;
    float32 f32VelXRel;
    float32 f32VelYRel;
    float32 f32EgoVelX;
    float32 f32EgoVelY;
} stRelativeStates;

/*==================[macros]================================================*/

/** When acceleration is reset use this default value */
#define TUE_PRV_GAIN_ESTIMATION_RESET_VAR_ACC (5.0f)

/** minimal distance for gain estimation */
#define TUE_PRV_GAIN_ESTIMATION_MINIMAL_XPOS (5.0f)

/** prediction q value for gain  standard deviation of 0.1cm / 0.1 m/s / 0.1
 * m/s^2*/
#define TUE_PRV_GAIN_PREDICTION_Q (0.01f)

/** number of radar cylces radar is used for update gain  */
#define TUE_PRV_GAIN_RADAR_CYCLES_ESTIMATION (3u)

/** maximal gain for camera for pedestrians */
#define TUE_PRV_GAIN_PREDICTION_CAM_GAIN_PED_MAX (1.1f)
#define TUE_PRV_GAIN_PREDICTION_CAM_GAIN_PED_MIN (0.9f)

/** maximal gain for camera for vehicles  */
#define TUE_PRV_GAIN_PREDICTION_CAM_GAIN_CAR_MAX (1.1f)
#define TUE_PRV_GAIN_PREDICTION_CAM_GAIN_CAR_MIN (0.9f)

/** maximal gain for camera for trucks  */
#define TUE_PRV_GAIN_PREDICTION_CAM_GAIN_TRUCK_MAX (1.1f)
#define TUE_PRV_GAIN_PREDICTION_CAM_GAIN_TRUCK_MIN (0.9f)

/** Default value for maximal camera gain */
#define TUE_PRV_GAIN_PREDICTION_CAM_GAIN_DEFAULT_MAX (1.1f)
#define TUE_PRV_GAIN_PREDICTION_CAM_GAIN_DEFAULT_MIN (0.9f)

/** minimum number of states available (because velY is adjusted - so has to be
 * present) */
#define TUE_PRV_GAIN_PREDICTION_MIN_STATE_SIZE (4u)

/*==================[forward declarations]==================================*/
/*==================[symbolic constants]====================================*/

/*==================[return codes]==========================================*/
/*==================[functions]=============================================*/

/**
 * @fn   increaseP(TueObjFusn_TrackableType * const pTrkbl, stRelativeStates
 * const * const sRelative, f32_t f32Gain, f32_t f32GainVar)
 *
 * @brief   increase P matrix, based on accurancy of gain -> e.g. required to
 * expand P matrix, if fused track is updated by camera only for a long time,
 * e.g. during track merge when no gain when new gain is calulated
 *
 * @param   [in,out] pTrkbl  TueObjFusn_TrackableType * const, Trackable which
 * P-matrix should be increased
 * @param   [in] sRelative   stRelativeStates, relative state of Track (required
 * for adding correct noise, since camera works in realtive system)
 * @param   [in] f32Gain     f32_t, estimated gain for compensation
 * @param   [in] f32GainVar  f32_t, estimated gain variance for compensation
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 increaseP(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrkbl,
    CONSTP2CONST(stRelativeStates, AUTOMATIC, ObjFusn_VAR_NOINIT) sRelative,
    const float32 f32GainVar);
#define ObjFusn_STOP_SEC_CODE

/**
 * @fn   getRelativeStates(TueObjFusn_TrackableType const * const pTrkbl,
 * stRelativeStates * const sRelative, f32_t const f32Latency);
 *
 * @brief   increase P matrix inversly, if fused track is updated by camera only
 * for a long time -> variance has to be increased when converted back to
 * measurement space e.g. used in track merge
 *
 * @param   [in]  pTrkbl      TueObjFusn_TrackableType * const, Trackable from
 * which relative states should be extracted
 * @param   [out] sRelative   stRelativeStates, relative state of Track
 * @param   [in] f32Latency   f32_t, latency of current Trackable -> required to
 * get correct ego motion for transformation in relative states
 *
 * @return  TUEOBJFUSN_ERROR_NOERROR if no error occured and error code
 * otherwise.
 */
#define ObjFusn_START_SEC_CODE

LOCAL uint32 getRelativeStates(
    CONSTP2CONST(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrkbl,
    CONSTP2VAR(stRelativeStates, AUTOMATIC, ObjFusn_VAR_NOINIT) psRelative,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotionItem);
#define ObjFusn_STOP_SEC_CODE

#endif /* TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON */

/*==================[external function declarations]========================*/
/*==================[external constants]====================================*/
/*==================[external data]=========================================*/

#ifdef __cplusplus
}
#endif

#endif  // SENSEAUTO_PILOT_DECISION_PILOT_DECISION_SDK_DECISION_SRC_FUSION_SOURCE_TUE_PRV_GAINESTIMATION_INT_H_
       /**
        * @}
        */
/*==================[end of file]===========================================*/
