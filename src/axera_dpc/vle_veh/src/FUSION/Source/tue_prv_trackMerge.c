/** \defgroup tuePrvTrackMerge TUE Track Merge AAU
 * \brief  performs track merge
 *
 * \addtogroup tuePrvTrackMerge
 * \{
 * \file    tue_prv_trackMerge.c
 * \brief   source code of ego motion module
 */
/* PRQA S 0292 ++ */ /* MKS */
                     /*  */
                     /* PRQA S 0292 -- */
                     /*          (C) Copyright Tuerme Inc. All rights reserved.
                      */
/*==================[inclusions]============================================*/

#include "tue_prv_trackMerge.h"
#include "tue_prv_trackMerge_int.h"
#include "tue_prv_lkf_trackManagement.h"
#include "tue_prv_distance_score.h"
#include "tue_prv_association.h"
#include "tue_prv_gainEstimation.h"
#include "TueObjFusn_ParameterInterface.h"
#include "tue_prv_idProvider.h"
#include "TueObjFusn_TrackableConstants.h"
#include "TueObjFusn_TrackableListUtils.h"
#include "TueObjFusn_TrackableProps.h"
#include "tue_prv_quality_management.h"
#include "tue_prv_common_array_utils.h"
#include "TueObjFusn_ErrorCodes.h"
#include "TueObjFusn_AAU_Codes.h"
#include "tue_prv_error_management.h"
#include "tue_prv_fusion_math.h"
#include "TueObjFusn_Eps.h"

#define ObjFusn_START_SEC_CODE
#ifndef MT_STATE_MEASURED
#define MT_STATE_MEASURED (2U)
#endif

/* PRQA S 1532 2 */ /* Interface function called by external AAU */
uint32 trackMerge_mergeTracks(
    CONSTP2VAR(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackList,
    VAR(boolean, ObjFusn_VAR_NOINIT) abMarkedToDrop[],
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16it = 0u;
    sint16 s16Rad = 0;
    sint16 s16Cam = 0;
    uint16 u16NumRadarOnlyTracks = 0u;
    uint16 u16NumVisionOnlyTracks = 0u;
    uint16 u16DropIdx = 0u;
    boolean bVisionModified = FALSE;

    const float32 _f32MatchGate =
        Fusion_get_f32MatchGate() * TUE_PRV_TRACKMERGE_FACTOR_GATE;

    /* Pointer to internal dist matrix structure */
    CONSTP2VAR(stDistMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT)
    pDistMat = getDistMat();

    /* Store indices to tracks as size of distance matrix is less than number of
     * max trackables */
    VAR(uint16, ObjFusn_VAR_NOINIT)
    au16IdxRadarOnlyTracks[TUE_PRV_FUSION_MAX_OBJECTS_RADAR];
    VAR(uint16, ObjFusn_VAR_NOINIT)
    au16IdxVisionOnlyTracks[TUE_PRV_FUSION_MAX_OBJECTS_VISION];
    VAR(stMatchIndexArrayType, ObjFusn_VAR_NOINIT) stFusionMatchesArray;
    VAR(TueObjFusn_TrackableType, ObjFusn_VAR_NOINIT) stTrkblTmp;

    tue_prv_common_array_utils_defaultInit_abool(
        abMarkedToDrop, TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX, FALSE);

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pTrackList) || (NULL_PTR == pEgoMotion) ||
        (NULL_PTR == abMarkedToDrop)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
            TUEOBJFUSN_AAU_TRACKMERGE_MERGE_TRACKS);
    } else
#endif
        if (1u < pTrackList->u16ValidTrackables) {
        u32Success = trackMerge_findNonFusedTracks(
            pTrackList, au16IdxRadarOnlyTracks,
            TUE_PRV_FUSION_MAX_OBJECTS_RADAR, &u16NumRadarOnlyTracks,
            au16IdxVisionOnlyTracks, TUE_PRV_FUSION_MAX_OBJECTS_VISION,
            &u16NumVisionOnlyTracks);

        u32Success |= trackMerge_distanceCam2Radar(
            pTrackList, _f32MatchGate, pDistMat, au16IdxRadarOnlyTracks,
            u16NumRadarOnlyTracks, au16IdxVisionOnlyTracks,
            u16NumVisionOnlyTracks, pEgoMotion);

        /* call current association algorithm */
        u32Success |= tue_prv_association_runAssociation(
            pDistMat, _f32MatchGate, &stFusionMatchesArray);

        if ((u32Success == TUEOBJFUSN_ERROR_NOERROR) &&
            (0u < stFusionMatchesArray.u16NumMatches)) {
            for (u16it = 0u; u16it < stFusionMatchesArray.u16NumMatches;
                 u16it++) {
                u16DropIdx = 0u;
                s16Rad = (pTrackList->as16TrackableMap)
                    [au16IdxRadarOnlyTracks[stFusionMatchesArray
                                                .aMatchIndexArray[u16it]
                                                .u16IndexCol]];
                s16Cam = (pTrackList->as16TrackableMap)
                    [au16IdxVisionOnlyTracks[stFusionMatchesArray
                                                 .aMatchIndexArray[u16it]
                                                 .u16IndexRow]];

                /* sSdist_mat[camera][radar] */
                if ((0u !=
                     (pTrackList->aTrackable[s16Rad].u32SensorsHist &
                      TUEOBJFUSN_TRACKABLE_U32SENSOR_VISION)) &&  // radar track
                                                                  // was fused
                                                                  // before
                                                                  // (criteria
                                                                  // 2)
                    (((pTrackList->aTrackable[s16Rad].u16Age >
                       pTrackList->aTrackable[s16Cam].u16Age) ||
                      (TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT ==
                       pTrackList->aTrackable[s16Cam].u16Age)) ||  // check age
                                                                   // (criteria
                                                                   // 3) - if
                                                                   // defualt =
                                                                   // new track
                     (0u ==
                      (pTrackList->aTrackable[s16Cam].u32SensorsHist &
                       TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR))))  // radar has to
                                                                 // be updated
                                                                 // by radar
                                                                 // (criteria 3)
                {
                    (void)Trackable_copyTrackable(
                        &stTrkblTmp, &pTrackList->aTrackable[s16Rad]);
                    u32Success |= trackMerge_mergeTrkble(
                        &pTrackList->aTrackable[s16Rad],
                        &pTrackList->aTrackable[s16Cam], pEgoMotion);
                    u16DropIdx =
                        au16IdxVisionOnlyTracks[stFusionMatchesArray
                                                    .aMatchIndexArray[u16it]
                                                    .u16IndexRow];

                    /* Vision shall be dropped -> Radar modified */
                    bVisionModified = FALSE;
                } else /* take camera only (u32SensorCurr) track */
                {
                    (void)Trackable_copyTrackable(
                        &stTrkblTmp, &pTrackList->aTrackable[s16Cam]);
                    u32Success |= trackMerge_mergeTrkble(
                        &pTrackList->aTrackable[s16Cam],
                        &pTrackList->aTrackable[s16Rad], pEgoMotion);
                    u16DropIdx =
                        au16IdxRadarOnlyTracks[stFusionMatchesArray
                                                   .aMatchIndexArray[u16it]
                                                   .u16IndexCol];
                    // set radar motion type input value after radar track
                    // merged to camera object only
                    pTrackList->aTrackable[s16Cam].u8RadarMotionTypeInput =
                        pTrackList->aTrackable[s16Rad].u8RadarMotionTypeInput;
                    /* Radar shall be dropped -> vision modified */
                    bVisionModified = TRUE;
                }

                if (TUEOBJFUSN_ERROR_NOERROR == u32Success) {
                    abMarkedToDrop[u16DropIdx] = TRUE;
                } else if (TRUE == bVisionModified) {
                    /* Error occured, modifications of vision track shall be
                     * reverted */
                    u32Success = Trackable_copyTrackable(
                        &pTrackList->aTrackable[s16Cam], &stTrkblTmp);
                } else {
                    /* Error occured, modifications of radar track shall be
                     * reverted */
                    u32Success = Trackable_copyTrackable(
                        &pTrackList->aTrackable[s16Rad], &stTrkblTmp);
                }
            }
        } else {
            /* MISRA */
        }
    } else {
        u32Success = TUEOBJFUSN_ERROR_NOERROR;
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

/* merge Track B into Track A*/
LOCAL uint32 trackMerge_mergeTrkble(
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrackA,
    CONSTP2VAR(TueObjFusn_TrackableType, AUTOMATIC, ObjFusn_VAR_NOINIT) pTrackB,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) sInvP1P2;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) sMatTemp;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) sMatP1;
    VAR(stf32Matrix_t, ObjFusn_VAR_NOINIT) sMatP2;
    VAR(stf32Vec_t, ObjFusn_VAR_NOINIT) sVecTemp;

    uint16 u16SensPos;
    const boolean bUseCoastingTmp = Fusion_get_bUseCoasting();
#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_OFF
    /* Ensure no compiler warning is triggered */
    float32 f32Tmp = pEgoMotion->f32Speed;
    f32Tmp += FLT_ZERO;
#endif

#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    if ((NULL_PTR == pTrackA) || (NULL_PTR == pTrackB) ||
        (NULL_PTR == pEgoMotion)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
            TUEOBJFUSN_AAU_TRACKMERGE_MERGE_TRKBL);
    } else
#endif
    {
#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON
        if ((0u != (pTrackA->u32SensorsCurr &
                    TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA)) &&
            (FLT_ZERO > pTrackA->f32GainVar)) {
            /* trackA - camera only track */
            u32Success = gain_estimation(pTrackB, pTrackA, &pTrackA->f32Gain,
                                         &pTrackA->f32GainVar);
            u32Success |=
                gain_compensation(pTrackA, pTrackA, pTrackA->f32Gain,
                                  pTrackA->f32GainVar, FALSE, pEgoMotion);
        } else if ((0u != (pTrackB->u32SensorsCurr &
                           TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA)) &&
                   (FLT_ZERO > pTrackB->f32GainVar)) {
            /* trackB - camera only track  -> store in TrackA since this track
             * will not be deleted */
            u32Success = gain_estimation(pTrackA, pTrackB, &pTrackA->f32Gain,
                                         &pTrackA->f32GainVar);
            u32Success |=
                gain_compensation(pTrackB, pTrackB, pTrackA->f32Gain,
                                  pTrackA->f32GainVar, FALSE, pEgoMotion);
        } else { /* no camera only track available -> reset gain, since both are
                    fused -> have same 'model'*/
            pTrackA->f32Gain = TUEOBJFUSN_TRACKABLE_F32GAIN_DEFAULT;
            pTrackA->f32GainVar = TUEOBJFUSN_TRACKABLE_F32GAINVAR_DEFAULT;
        }
#endif

        /* calculate new state and covaraince  */
        /* both matrices have to have same size, since both are tracks - for CT
         * model has to be adapted */
        // xNew = P2 * inv(P1+P2) * x1 + P1 * inv(P1+P2) * x2
        u32Success |= f32SymMatToMat(&(pTrackA->matP), &sMatP1);
        u32Success |= f32SymMatToMat(&(pTrackB->matP), &sMatP2);
        u32Success |= f32MatAdd(&sMatP1, &sMatP2, &sInvP1P2);
        u32Success |= f32MatInv(&sInvP1P2);  // inv(P1+P2)
        u32Success |=
            f32MatMul(&sMatP2, &sInvP1P2, &sMatTemp);  // P2 * inv(P1+P2)
        u32Success |= f32MatMulVec(&sMatTemp, &pTrackA->vecX,
                                   &sVecTemp);  // P2 * inv(P1+P2) * x1
        u32Success |= f32CopyVec(&(pTrackA->vecX), &sVecTemp);
        u32Success |=
            f32MatMul(&sMatP1, &sInvP1P2, &sMatTemp);  // P1 * inv(P1+P2)
        u32Success |= f32MatMulVec(&sMatTemp, &pTrackB->vecX,
                                   &sVecTemp);  // P1 * inv(P1+P2) * x2
        u32Success |= f32VecAdd(&pTrackA->vecX, &sVecTemp,
                                &pTrackA->vecX);  // add second summand

        /* calculate new P matrix */
        // P1 * (inv(P1+P2)) * P2
        u32Success |=
            f32MatMul(&sMatP1, &sInvP1P2, &sMatTemp);  // P1* inv(P1+P2)
        u32Success |=
            f32MatMul(&sMatTemp, &sMatP2, &sMatP1);  // inv(P1+P2) * P2
        u32Success |= f32MatToSymMat(&sMatP1, &pTrackA->matP);

        /* adapt states (in chronologic order) */
        u16SensPos = Trackable_getSensPos(
            pTrackB->u32SensorsCurr);  // is radar/camera only track so max. one
                                       // bit is set
        pTrackA->au16SensorID[u16SensPos] = pTrackB->au16SensorID[u16SensPos];
        pTrackA->au16SensorIDLast[u16SensPos] =
            pTrackB->au16SensorIDLast[u16SensPos];
        pTrackA->bUpdated = TRUE;

        pTrackA->u32SensorsCurr =
            ((pTrackA->u32SensorsCurr) | (pTrackB->u32SensorsCurr));
        pTrackA->u32SensorsHist =
            ((pTrackA->u32SensorsHist) | (pTrackB->u32SensorsHist));
        pTrackA->u8CyclesNoRadar = tue_prv_fusion_min_U8(
            pTrackA->u8CyclesNoRadar, pTrackB->u8CyclesNoRadar);
        pTrackA->u8CyclesNoVision = tue_prv_fusion_min_U8(
            pTrackA->u8CyclesNoVision, pTrackB->u8CyclesNoVision);

        /* if fused object shall be coasted, set lifespan of track to coasting
         * lifespan */
        if ((TRUE == bUseCoastingTmp) &&
            (((pTrackA->u32SensorsCurr) & TUE_PRV_COASTING_SENSOR_PATTERN) ==
             TUE_PRV_COASTING_SENSOR_PATTERN)) {
            pTrackA->u16Lifespan = TUEOBJFUSN_TRACKABLE_U16LIFESPAN_COASTED;
        } else {
            pTrackA->u16Lifespan = tue_prv_fusion_max_U16(pTrackA->u16Lifespan,
                                                          pTrackB->u16Lifespan);
        }

        /* Track Quality and Existence Probabiltiy */
        pTrackA->f32ObstacleProbability = tue_prv_fusion_min_F32(
            pTrackA->f32ObstacleProbability, pTrackB->f32ObstacleProbability);
        pTrackA->f32ExistenceQuality = tue_prv_fusion_max_F32(
            pTrackA->f32ExistenceQuality, pTrackB->f32ExistenceQuality);

        // rcs and mainteance merge
        pTrackA->fRCS = tue_prv_fusion_max_F32(pTrackA->fRCS, pTrackB->fRCS);
        pTrackA->eObjMaintenanceState = MT_STATE_MEASURED;  // object measured

        /* take ID from track longer available, check if one track was set up in
         * this cycle (don't use this!) */
        if (((pTrackB->u16Age > pTrackA->u16Age) &&
             (pTrackB->u16Age != TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT)) ||
            // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/25 changed
            // by guotao 20200515 start
            ((pTrackB->u16Age != TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT) &&
             (pTrackA->u16Age == TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT)))
        // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/25 changed by
        // guotao 20200515 end
        { /* take trackB, since it is older, but not new */
            /* if ID was already set - release ID (not for tacks just created)
             */
            u32Success |= idProvider_releaseFusionId(pTrackA->u16ID);
            uint16 tempId = pTrackA->u16ID;
            pTrackA->u16Age = pTrackB->u16Age;
            pTrackA->u16ID = pTrackB->u16ID;
            pTrackB->u16ID = tempId;
            // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/25 changed
            // by guotao 20200515 start
            pTrackA->f32Heading = pTrackB->f32Heading;
            pTrackA->f32HeadingVar = pTrackB->f32HeadingVar;
            // Ticket:http://119.3.139.124:10101/tickets/DFA-ZT19.git/25 changed
            // by guotao 20200515 end
        } else
        /* if ID from Track A is chosen -> release B */
        {
            u32Success |=
                idProvider_releaseFusionId(pTrackB->u16ID);  // release ID
        }
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

LOCAL uint32 trackMerge_distanceCam2Radar(
    CONSTP2CONST(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackList,
    const float32 f32MatchGate,
    CONSTP2VAR(stDistMatrix_t, AUTOMATIC, ObjFusn_VAR_NOINIT) pDist_mat,
    const uint16 au16IdxRadarOnlyTracks[],
    const uint16 u16NumRadarOnlyTracks,
    const uint16 au16IdxVisionOnlyTracks[],
    const uint16 u16NumVisionOnlyTracks,
    CONSTP2CONST(TueObjFusn_EgoMotionType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pEgoMotion) {
    VAR(TueObjFusn_TrackableType, ObjFusn_VAR_NOINIT) stTrkCamGainCorr;
    VAR(TueObjFusn_TrackableType, ObjFusn_VAR_NOINIT) stTrkRadarCorr;
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    uint16 u16Cam = 0u;
    uint16 u16Rad = 0u;
    sint16 s16Idx = -1;
    float32 f32Dist = FLT_ZERO;
    float32 f32DiffX = FLT_ZERO;
    float32 f32DiffY = FLT_ZERO;
    float32 f32DiffVelX = FLT_ZERO;

#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON
    float32 f32GainTmp = FLT_ZERO;
    float32 f32GainVarTmp = FLT_ZERO;
#else
    /* Prevent compiler warning when gain estimation is disabled */
    float32 f32Tmp = pEgoMotion->f32Speed;
#endif

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    /* input validity checks */
    if ((au16IdxRadarOnlyTracks == NULL_PTR) ||
        (au16IdxVisionOnlyTracks == NULL_PTR)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
            TUEOBJFUSN_AAU_TRACKMERGE_DISTCAM2RADAR);
    } else
#endif
        if (TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX <
            pTrackList->u16ValidTrackables) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
            TUEOBJFUSN_AAU_TRACKMERGE_DISTCAM2RADAR);
    } else if ((u16NumRadarOnlyTracks > TUE_PRV_FUSION_MAX_OBJECTS_RADAR) ||
               (u16NumVisionOnlyTracks > TUE_PRV_FUSION_MAX_OBJECTS_VISION)) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
            TUEOBJFUSN_AAU_TRACKMERGE_DISTCAM2RADAR);
    } else
#endif
        if ((u16NumRadarOnlyTracks == 0u) || (u16NumVisionOnlyTracks == 0u)) {
        /* Init with maximum distance value */
        initDistMatrix(pDist_mat, 0u, 0u, FLT_ZERO);
    } else {
        /* Init with maximum distance value */
        initDistMatrix(pDist_mat, u16NumVisionOnlyTracks, u16NumRadarOnlyTracks,
                       f32MatchGate);

        /** Search for radar only tracks */
        for (u16Rad = 0u; u16Rad < u16NumRadarOnlyTracks; u16Rad++) {
            s16Idx =
                pTrackList->as16TrackableMap[au16IdxRadarOnlyTracks[u16Rad]];
            u32Success |= Trackable_copyTrackable(
                &stTrkRadarCorr, &(pTrackList->aTrackable[s16Idx]));

            if (stTrkRadarCorr.u16Age != TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT) {
                lkfTrackManagement_increaseDiagonalP(&stTrkRadarCorr.matP);
            } else {
                /* MISRA */
            }

            for (u16Cam = 0u; u16Cam < u16NumVisionOnlyTracks; u16Cam++) {
                /* copy camera only track */
                s16Idx =
                    pTrackList
                        ->as16TrackableMap[au16IdxVisionOnlyTracks[u16Cam]];
                u32Success |= Trackable_copyTrackable(
                    &stTrkCamGainCorr, &(pTrackList->aTrackable[s16Idx]));

                if (stTrkCamGainCorr.u16Age !=
                    TUEOBJFUSN_TRACKABLE_U16AGE_DEFAULT) {
                    lkfTrackManagement_increaseDiagonalP(
                        &stTrkCamGainCorr.matP);
                } else {
                    /* MISRA */
                }

#if TUE_PRV_GAIN_ESTIMATION_ENABLE_GAIN_CALCULATION == STD_ON
                if (stTrkCamGainCorr.f32GainVar <
                    TUE_PRV_FUSION_COMPARE_TO_ZERO) {
                    /* camera track is in camera coodinates (GainVar is default
                     * -1.0f) -> calculate new gain*/
                    f32GainTmp = TUEOBJFUSN_TRACKABLE_F32GAIN_DEFAULT;
                    f32GainVarTmp = TUEOBJFUSN_TRACKABLE_F32GAINVAR_DEFAULT;
                    u32Success |=
                        gain_estimation(&stTrkRadarCorr, &stTrkCamGainCorr,
                                        &f32GainTmp, &f32GainVarTmp);
                    u32Success |= gain_compensation(
                        &stTrkCamGainCorr, &stTrkCamGainCorr, f32GainTmp,
                        f32GainVarTmp, FALSE, pEgoMotion);
                } else if (TUE_PRV_GAIN_RADAR_CYCLES_RESET <
                           stTrkCamGainCorr.u8CyclesNoRadar) {
                    /* camera was updated by radar before -> increase P if radar
                     * update is old */
                    /* Gain initialized at this point, no further checks needed
                     */
                    u32Success |=
                        gain_PIncrease(&stTrkCamGainCorr,
                                       stTrkCamGainCorr.f32GainVar, pEgoMotion);
                } else {
                    /* camera track already compensated, radar recently
                     * available, so gain is correct -> do nothing */
                }
#else
                /** Suppress compiler warning */
                f32Tmp += FLT_ZERO;
#endif

                f32DiffX = tue_prv_fusion_abs(
                    stTrkRadarCorr.vecX.data[TRACKABLE_POSX] -
                    stTrkCamGainCorr.vecX.data[TRACKABLE_POSX]);
                f32DiffY = tue_prv_fusion_abs(
                    stTrkRadarCorr.vecX.data[TRACKABLE_POSY] -
                    stTrkCamGainCorr.vecX.data[TRACKABLE_POSY]);
                f32DiffVelX = tue_prv_fusion_abs(
                    stTrkRadarCorr.vecX.data[TRACKABLE_VELX] -
                    stTrkCamGainCorr.vecX.data[TRACKABLE_VELX]);

                if (((f32DiffX < TUE_PRV_DISTANCE_SCORE_MAX_COARSE_DIST_X) &&
                     (f32DiffY < TUE_PRV_DISTANCE_SCORE_MAX_COARSE_DIST_Y)) &&
                    (f32DiffVelX < TUE_PRV_DISTANCE_SCORE_MAX_COARSE_VEL_X)) {
                    u32Success |= distance_score_computeExtendedDist(
                        &stTrkRadarCorr, &stTrkCamGainCorr, f32MatchGate,
                        &f32Dist);

                    if (f32Dist < f32MatchGate) {
                        pDist_mat->data[u16Cam][u16Rad] =
                            convertFloatToFixedtDistMat(f32Dist);
                    } else {
                        /* MISRA */
                    }
                } else {
                    /* MISRA */
                }
            }
        }
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

#define ObjFusn_START_SEC_CODE

LOCAL uint32 trackMerge_findNonFusedTracks(
    CONSTP2CONST(TueObjFusn_TrackableListType, AUTOMATIC, ObjFusn_VAR_NOINIT)
        pTrackList,
    uint16 au16IdxRadarOnlyTracks[],
    const uint16 u16nElementsRadarToDistMatrix,
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) pu16NumRadarOnlyTracks,
    uint16 au16IdxVisionOnlyTracks[],
    const uint16 u16nElementsVisionToDistMatrix,
    CONSTP2VAR(uint16, AUTOMATIC, ObjFusn_VAR_NOINIT) pu16NumVisionOnlyTracks) {
    uint32 u32Success = TUEOBJFUSN_ERROR_NOERROR;
    /* PRQA S 3204 1 */ /* variable may be set to other values depending on
                           run-time check activation */
    uint16 u16i = 0u;
    sint16 s16Idx = -1;
    uint32 u32SensorCurr = 0u;
    uint16 u16CntVision = 0u;
    uint16 u16CntRadar = 0u;

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
#if (TUE_PRV_RUNTIME_POINTER_CHECK == STD_ON)
    /* input validity checks */
    if ((NULL_PTR == pTrackList) || (NULL_PTR == au16IdxRadarOnlyTracks) ||
        (NULL_PTR == au16IdxVisionOnlyTracks) ||
        (NULL_PTR == pu16NumRadarOnlyTracks) ||
        (NULL_PTR == pu16NumVisionOnlyTracks)) {
        u32Success = TUEOBJFUSN_ERROR_NULL_POINTER_EXCEPTION;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
            TUEOBJFUSN_AAU_TRACKMERGE_FIND_NON_FUSED_TRACKS);
    } else
#endif
        if (TUEOBJFUSN_TRACKABLELIST_U16VALIDTRACKABLES_MAX <
            pTrackList->u16ValidTrackables) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
            TUEOBJFUSN_AAU_TRACKMERGE_FIND_NON_FUSED_TRACKS);
    } else if ((u16nElementsRadarToDistMatrix >
                TUE_PRV_FUSION_MAX_OBJECTS_RADAR) ||
               (u16nElementsVisionToDistMatrix >
                TUE_PRV_FUSION_MAX_OBJECTS_VISION)) {
        u32Success = TUEOBJFUSN_ERROR_NUMBER_OF_OBJECTS_EXCEEDS_MAXIMUM_SIZE;
        (void)tue_prv_error_management_addError(
            u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
            TUEOBJFUSN_AAU_TRACKMERGE_FIND_NON_FUSED_TRACKS);
    } else
#endif
    {
        tue_prv_common_array_utils_defaultInit_au16(
            au16IdxRadarOnlyTracks, TUE_PRV_FUSION_MAX_OBJECTS_RADAR, 0u);
        tue_prv_common_array_utils_defaultInit_au16(
            au16IdxVisionOnlyTracks, TUE_PRV_FUSION_MAX_OBJECTS_VISION, 0u);

        for (u16i = 0u; u16i < (pTrackList->u16ValidTrackables); u16i++) {
            s16Idx = pTrackList->as16TrackableMap[u16i];
            u32SensorCurr = pTrackList->aTrackable[s16Idx].u32SensorsCurr;

            if (((u32SensorCurr & TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) >
                 0u) &&
                ((u32SensorCurr & TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) ==
                 0u)) {
                if (u16CntVision < u16nElementsVisionToDistMatrix) {
                    au16IdxVisionOnlyTracks[u16CntVision] = u16i;
                    u16CntVision++;
                } else {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
                    u32Success |= TUEOBJFUSN_ERROR_INTERNAL_ERROR;
                    (void)tue_prv_error_management_addError(
                        u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
                        TUEOBJFUSN_AAU_TRACKMERGE_FIND_NON_FUSED_TRACKS);
#endif
                }
            } else if (((u32SensorCurr &
                         TUEOBJFUSN_TRACKABLE_U32SENSOR_CAMERA) == 0u) &&
                       ((u32SensorCurr & TUEOBJFUSN_TRACKABLE_U32SENSOR_RADAR) >
                        0u)) {
                if (u16CntRadar < u16nElementsRadarToDistMatrix) {
                    au16IdxRadarOnlyTracks[u16CntRadar] = u16i;
                    u16CntRadar++;
                } else {
#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
                    u32Success |= TUEOBJFUSN_ERROR_INTERNAL_ERROR;
                    (void)tue_prv_error_management_addError(
                        u32Success, TUEOBJFUSN_AAU_TRACKMERGE,
                        TUEOBJFUSN_AAU_TRACKMERGE_FIND_NON_FUSED_TRACKS);
#endif
                }
            } else {
                /* MISRA */
            }
        }

#if (TUE_PRV_RUNTIME_ERROR_CHECK == STD_ON)
        if (u32Success != TUEOBJFUSN_ERROR_NOERROR) {
            tue_prv_common_array_utils_defaultInit_au16(
                au16IdxRadarOnlyTracks, TUE_PRV_FUSION_MAX_OBJECTS_RADAR, 0u);
            tue_prv_common_array_utils_defaultInit_au16(
                au16IdxVisionOnlyTracks, TUE_PRV_FUSION_MAX_OBJECTS_VISION, 0u);
            *pu16NumRadarOnlyTracks = 0u;
            *pu16NumVisionOnlyTracks = 0u;
        } else
#endif
        {
            *pu16NumRadarOnlyTracks = u16CntRadar;
            *pu16NumVisionOnlyTracks = u16CntVision;
        }
    }

    return u32Success;
}
#define ObjFusn_STOP_SEC_CODE

/**
 * \}
 */
