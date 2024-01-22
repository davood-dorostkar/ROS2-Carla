
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "cd.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*! @brief       Distance for Curvature Evaluation

    @general     It is the required distance which is multiplied with the
   curvature gradient
                 for the calculation of the curvature

    @conseq      @incp  Curvature value will increase
                 @decp  Curvature value will decrease

    @attention   Not applicable
*/
#define CURVATURE_EVAL_DIST (1.0f)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  FUNCTION
*****************************************************************************/
static boolean CDHypoRunUpObjectFilter(
    ObjNumber_t iObjectIndex,
    const CDObjectData_t *pObjectData,
    const CDInternalObject_t *pObjInternData);

static void CDHypoRunUpCalculateProb(
    ObjNumber_t iObjectIndex,
    CDIntHypothesis_t *pHypothesis,
    float32 fRunUpTTCCorridorWidth,
    const CDInputData_t *pInputData,
    CDInternalObject_t *pObjInternData,
    const CDExternalFunctions_t *pExternalFunctions);

static void CDHypoRunUpReducePropShortBrake(CDIntHypothesis_t *pHypothesis,
                                            const CDInputData_t *pInputData,
                                            ObjNumber_t iObjectIndex,
                                            CDInternalObject_t *pObjInternData);

static boolean CDHypRunUpDetectNarrowCurve(
    const CDInputData_t *pInputData,
    ObjNumber_t iObjectIndex,
    const CDInternalObject_t *pObjInternData);

static float32 CDHypRunUpDetectSmallObject(const CDInputData_t *pInputData,
                                           ObjNumber_t iObjectIndex);

/* ***********************************************************************
  @fn            CDHypoRunUpInitInternalData */ /*!

                             @brief         clears internal data

                             @description   clears internal data of the runup
                           hypothesis object information

                             @param[in]     pRunUpData pointer to the objects
                           runup hypothesis internal information

                           ****************************************************************************
                           */
void CDHypoRunUpInitInternalData(CDInternalObjHypRunUpData_t *pRunUpData) {
    pRunUpData->BrkPropRed.uVrelXAbsDiff = (uint8)0;
    pRunUpData->BrkPropRed.bActive = FALSE;
    pRunUpData->BrkPropRed.uTimeGapVego = (uint16)0;
    pRunUpData->BrkPropRed.uTimeGapVrel = (uint16)0;
    pRunUpData->BrkPropRed.uVrelXAbsDiff = (uint8)0;
}

/* ***********************************************************************
  @fn            CDHypoRunUpObjectFilter */
static boolean CDHypoRunUpObjectFilter(
    ObjNumber_t iObjectIndex,
    const CDObjectData_t *pObjectData,
    const CDInternalObject_t *pObjInternData) {
    boolean bReturn;
    const Envm_t_GenObjAttributes *const pObjAttribs =
        CDGetPointer_Attributes(pObjectData, iObjectIndex);
    const Envm_t_GenObjKinEnvmatics *const pObjKinematic =
        CDGetPointer_Kinematic(pObjectData, iObjectIndex);
    /* Allow hypothesis if
     * a) object has min. required quality and is in course
     * b) object is moving or stopped and seen for min. required cycles in track
     * c) object has min. required TTC and TTB
     * d) braking is required if object was moving before
     * e) previous run-up hypothesis is kept until long. deceleration is greater
     * than threshold
     */

    const boolean bObjIsStopped = (boolean)(
        (((pObjAttribs->eDynamicProperty ==
           (Envm_t_GenObjDynamicProperty)
               Envm_GEN_OBJECT_DYN_PROPERTY_STOPPED) &&
          (pObjAttribs->uiDynConfidence >= CD_COMMON_MIN_STOPPED_CONF) &&
          (pObjInternData->HypothesisHist.WasOncomming == FALSE)) &&
         (pObjInternData->HypothesisHist.WasCrossing == FALSE)) &&
        ((pObjInternData->TrackAssigned &
          (uint8)CD_RUN_UP_MIN_TRACK_ASSIGNED) != 0u));
    const float32 fTmpDistToTraj = pObjInternData->TrajRelation.fDistToTraj;
    const float32 fObjDistX = CD_GET_DIST_X(pObjectData, iObjectIndex);

    if ((fABS(fTmpDistToTraj) < CD_RUN_UP_MAX_LAT_DIST) &&
        ((pObjAttribs->eDynamicProperty ==
          (Envm_t_GenObjDynamicProperty)Envm_GEN_OBJECT_DYN_PROPERTY_MOVING) ||
         (bObjIsStopped != FALSE)) &&
        ((pObjInternData->TTC < CD_COMMON_TTC_THRES) ||
         (pObjInternData->TTBPre < CD_COMMON_TTB_THRES) ||
         (pObjInternData->TTBAcute < CD_COMMON_TTB_THRES) ||
         (((pObjInternData->LongNecAccel < CD_RUNUP_ANECLONG_THRES) ||
           (pObjKinematic->fVrelX < CD_RUNUP_V_REL_X_THRES)) &&
          (pObjInternData->HypothesisHist.RunUpMoving == 1))) &&
        ((fObjDistX > CD_COMMON_MIN_DISTX) ||
         (pObjInternData->HypothesisHist.RunUpMoving == 1) ||
         (pObjInternData->HypothesisHist.RunUpStationary == 1) ||
         (pObjInternData->HypothesisHist.CutIn == 1))) {
        bReturn = TRUE;
    } else {
        bReturn = FALSE;
    }
    return bReturn;
}

/* ***********************************************************************
  @fn            CDHypoRunUpReducePropShortBrake */ /*!
                         @brief         Reduce hypothesis probability for short
                       braking target vehicles

                         @description   Reduce the probability for run-braking
                       hypothesis in case
                                        short brake pedal stroke of preceding
                       vehicle

                         @param[in]     pInputData = (Ego+Obj), Object index,
                       internal object data
                         @param[out]    pHypothesis probability
                         @param[out]    iObjectIndex
                         @param[out]    pObjInternData
                         @return        void

                         @pre           [none]
                         @post          [none]
                       ****************************************************************************
                       */
static void CDHypoRunUpReducePropShortBrake(
    CDIntHypothesis_t *pHypothesis,
    const CDInputData_t *pInputData,
    ObjNumber_t iObjectIndex,
    CDInternalObject_t *pObjInternData) {
    float32 fEgoVelX;
    float32 fObjVrelX;
    float32 fObjDistX;
    float32 fTimeGapVego;

    /* Check input pointer*/
    if ((pInputData != NULL) && (pObjInternData != NULL) &&
        (pHypothesis != NULL)) {
        fEgoVelX = pInputData->pEgoData->pEgoDynObjSync->fVelocityX;
        fObjVrelX =
            CDGetPointer_Kinematic(pInputData->pObjectData, iObjectIndex)
                ->fVrelX;
        fObjDistX = CD_GET_DIST_X(pInputData->pObjectData, iObjectIndex);
        fTimeGapVego = pObjInternData->TTC4;

        /* Initialize filter start value in case of new hypothesis */
        if (pObjInternData->HypothesisHist.RunUpMoving == 0uL) {
            /* As long as no run-up hypothesis is active use current value */
            pObjInternData->sHypRunUpData.BrkPropRed.fVRelXStartRunUp =
                fObjVrelX;
        } else {
            /* Run-up moving or braking as active, since run-up moving
            hypothesis is possible with positive VrelX (due to ArelX),
            updating VrelX as long as first neg. value has occurred */
            float32 fTimeGapVegoUpper = CML_f_CalculatePolygonValue(
                CD_RUNUP_BRK_VEGO_TGAP_NO, CD_RUNUP_BRK_VEGO_TGAP_UPPER,
                fEgoVelX);

            if ((pObjInternData->sHypRunUpData.BrkPropRed.fVRelXStartRunUp >
                 0.0F) ||
                (fTimeGapVego > fTimeGapVegoUpper)) {
                pObjInternData->sHypRunUpData.BrkPropRed.fVRelXStartRunUp =
                    fObjVrelX;
            }
        }

        /* Preceding vehicle must be braking => Run-Up braking hypothesis active
         */
        if ((pHypothesis->fHypothesisProbability > CD_COMMON_MIN_HYP_PROB) &&
            (pHypothesis->eType == CDHypothesisType_RunUpBraking)) {
            float32 fVrelXAbsDiff;
            float32 fFactHyp;
            float32 fTimeGapVrelX;
            float32 fTimeGapVegoMin;
            float32 fNegVRelXStartRunUp;

            fNegVRelXStartRunUp =
                -(pObjInternData->sHypRunUpData.BrkPropRed.fVRelXStartRunUp);

            /* Calculate time gap based on current ego vehicle velocity */
            if (fEgoVelX > C_F32_DELTA) {
                fTimeGapVego =
                    MINMAX_FLOAT(0.0f, CD_TIME_MAX, (fObjDistX / fEgoVelX));
            } else {
                fTimeGapVego = CD_TIME_MAX;
            }

            /* Calculate time gap based on rel. velocity at start of hypothesis
             */
            if (fNegVRelXStartRunUp > C_F32_DELTA) {
                fTimeGapVrelX = MINMAX_FLOAT(0.0F, CD_TIME_MAX,
                                             (fObjDistX / fNegVRelXStartRunUp));
            } else {
                fTimeGapVrelX = CD_TIME_MAX;
            }

            /* Magnitude of reduced rel. velocity since start of run-up */
            fVrelXAbsDiff = MAX_FLOAT(
                pObjInternData->sHypRunUpData.BrkPropRed.fVRelXStartRunUp -
                    fObjVrelX,
                0.0F);

            /* Probability reduction factor in dependency of increased
             * (magnitude) rel. velocity  */
            fFactHyp = CML_f_CalculatePolygonValue(
                CD_RUNUP_BRK_HYP_PROB_RED_OVER_VREL_NO,
                CD_RUNUP_BRK_PROB_VREL_RED, fVrelXAbsDiff);

            /* Time gap threshold in dependency of ego velocity */
            fTimeGapVegoMin = CML_f_CalculatePolygonValue(
                CD_RUNUP_BRK_VEGO_TGAP_NO, CD_RUNUP_BRK_VEGO_TGAP, fEgoVelX);

            /* Reduce hypothesis probability */
            if ((fFactHyp < 1.0f) /* Vrel change start of braking */
                && (fTimeGapVrelX >
                    CD_RUNUP_BRK_TIME_GAP) /* Distance based on ttc */
                && (fTimeGapVego >
                    fTimeGapVegoMin) /* Distance based on headway time */
                ) {
                /* Reduce probability and ensure minimum value to keep
                 * hypothesis alive */
                pHypothesis->fHypothesisProbability *= fFactHyp;
                pHypothesis->fHypothesisProbability =
                    MAX_FLOAT(pHypothesis->fHypothesisProbability,
                              CD_COMMON_MIN_HYP_PROB + C_F32_DELTA);
                pObjInternData->sHypRunUpData.BrkPropRed.bActive = TRUE;
            } else {
                pObjInternData->sHypRunUpData.BrkPropRed.bActive = FALSE;
            }

            /* Supporting signals for data acquisition */
            pObjInternData->sHypRunUpData.BrkPropRed.uVrelXAbsDiff =
                (uint8)ROUND_TO_UINT(fVrelXAbsDiff);
            pObjInternData->sHypRunUpData.BrkPropRed.uTimeGapVrel =
                (uint16)ROUND_TO_UINT(fTimeGapVrelX * 100.0f);
            pObjInternData->sHypRunUpData.BrkPropRed.uTimeGapVego =
                (uint16)ROUND_TO_UINT(fTimeGapVego * 100.0f);
        } else {
            /* Initialize filter start value in case of moving hypothesis */
            pObjInternData->sHypRunUpData.BrkPropRed.uVrelXAbsDiff = (uint8)0uL;
            pObjInternData->sHypRunUpData.BrkPropRed.uTimeGapVrel = (uint16)0uL;
            pObjInternData->sHypRunUpData.BrkPropRed.uTimeGapVego = (uint16)0uL;
            pObjInternData->sHypRunUpData.BrkPropRed.bActive = FALSE;
        }
    } else {
        /* Invalid input pointer received. Should never happen. */
        // TODO: Replace with VLC_ASSERT when available */
        BML_ASSERT(FALSE);
    }
}

/* ***********************************************************************
  @fn            CDHypRunUpDetectNarrowCurve */ /*!

                             @brief         Optimize run-up scenarios in narrow
                           curves

                             @description   Prevent drop-outs during run-up
                           hypothesis due to bad angle
                                            resolution

                             @param[in]     iObjectIndex Pointer to relevant
                           object
                             @param[in]     pInputData Pointer to CD input data
                             @param[in]     pObjInternData Internal CD object
                           array
                             @return        boolean

                             @pre           [none]
                             @post          [none]
                           ****************************************************************************
                           */
static boolean CDHypRunUpDetectNarrowCurve(
    const CDInputData_t *pInputData,
    ObjNumber_t iObjectIndex,
    const CDInternalObject_t *pObjInternData) {
    boolean ret;

    float32 fCurrentCurvature;

    fCurrentCurvature = VLCSen_p_GetVDYDynObjSync()->Lateral.DrvIntCurve.Curve;

    /* Optimize run-up in narrow curves, prevent drop-outs due to bad angle
     * resolution */
    if ((CD_GET_DIST_X(pInputData->pObjectData, iObjectIndex) <
         CD_RUN_UP_NCURVE_MAX_DIST) &&
        (pObjInternData->TTC < CD_RUN_UP_NCURVE_MAX_TTC) &&
        (pObjInternData->HypothesisHist.RunUpMoving ==
         1) /* only keep existing run-up hypothesis */
        && (((fCurrentCurvature >
              CD_RUN_UP_NCURVE_C0_MIN) /* left curve, object left */
             && (pObjInternData->TrajRelation.fDistToTraj >
                 CD_RUN_UP_NCURVE_DIST2TRAJ2) &&
             (CDGetPointer_Dimensions(pInputData->pObjectData, iObjectIndex)
                  ->fOrientation > CD_RUN_UP_NCURVE_ORIENT)) ||
            ((fCurrentCurvature <
              -CD_RUN_UP_NCURVE_C0_MIN) /* right curve, object right */
             && (pObjInternData->TrajRelation.fDistToTraj >
                 CD_RUN_UP_NCURVE_DIST2TRAJ2) &&
             (CDGetPointer_Dimensions(pInputData->pObjectData, iObjectIndex)
                  ->fOrientation < -CD_RUN_UP_NCURVE_ORIENT)))) {
        ret = TRUE;
    } else {
        ret = FALSE;
    }
    return ret;
}

/* ***********************************************************************
  @fn            CDHypRunUpDetectSmallObject */ /*!

                             @brief         Detect small objects for run up
                           hypothesis

                             @description   Detect small objects and then reduce
                           trajectory width for small object class

                             @param[in]     iObjectIndex Pointer to relevant
                           object
                             @param[in]     pInputData Pointer to CD input data
                             @return        float32

                             @pre           [none]
                             @post          [none]
                           ****************************************************************************
                           */
static float32 CDHypRunUpDetectSmallObject(const CDInputData_t *pInputData,
                                           ObjNumber_t iObjectIndex) {
    float32 redFact = 1.0F;
    Envm_t_GenObjClassification objClass =
        CDGetPointer_Attributes(pInputData->pObjectData, iObjectIndex)
            ->eClassification;

    if ((objClass ==
         (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_POINT) ||
        (objClass ==
         (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_PEDESTRIAN) ||
        (objClass ==
         (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_MOTORCYCLE) ||
        (objClass ==
         (Envm_t_GenObjClassification)Envm_GEN_OBJECT_CLASS_BICYCLE)) {
        /* Reduce trajectory width only for small object class like bicycle */
        fDistance_t fminDist;

        /* Get minimum longitudinal distance where trajectory width narrowing
         * will be fully applied */
        fminDist = CML_f_CalculatePolygonValue(
            CD_RUNUP_SOBJ_HYP_OVLC_VEGO_DIST_NO, CD_RUNUP_SOBJ_OVLC_VEGO_DIST,
            pInputData->pEgoData->pEgoDynObjSync->fVelocityX);

        /* Define transition section */
        CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED[0].f0 =
            fminDist * CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED_FACT; /* Long. distance
                                                                starting
                                                                narrowing */
        CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED[1].f0 =
            fminDist; /* Long. distance complete narrowing */

        /* Calculate trajectory width reduction factor */
        redFact = CML_f_CalculatePolygonValue(
            CD_RUNUP_SOBJ_HYP_OVLC_RED_OVER_DIST_NO,
            CD_RUNUP_SOBJ_HYP_OVLC_DIST_RED,
            CD_GET_DIST_X(pInputData->pObjectData, iObjectIndex));
    }

    return redFact;
}

/* ***********************************************************************
  @fn           CDHypoRunUpCalculateProb */
static void CDHypoRunUpCalculateProb(
    ObjNumber_t iObjectIndex,
    CDIntHypothesis_t *pHypothesis,
    float32 fRunUpTTCCorridorWidth,
    const CDInputData_t *pInputData,
    CDInternalObject_t *pObjInternData,
    const CDExternalFunctions_t *pExternalFunctions) {
    float32 fQualityNow;
    float32 fQualityTTC;
    float32 fQualityDistSpeedAccel;
    fTime_t fTTCPred;

    fDistance_t fLatPosTTC;

    float32 fUsedOccupancy;

    EMPDistanceWidth_t DistanceWidth;
    EMPTrajOccupancy_t OccupancyNow;
    EMPTrajOccupancy_t OccupancyTTC;

    float32 smallObjRedFact;

    smallObjRedFact = CDHypRunUpDetectSmallObject(pInputData, iObjectIndex);

    /* set distance structure */
    DistanceWidth.fDistance = pObjInternData->TrajRelation.fDistToTraj;
    DistanceWidth.fDistanceVar = pObjInternData->TrajRelation.fDistToTrajVar;
    DistanceWidth.fTrajectoryWidth = CD_RUN_UP_TRACK_WIDTH * smallObjRedFact;
    DistanceWidth.fTrajectoryWidthVar = 0; /* 0 as long as extensions are
                                              working and trackwidth is not
                                              measured or estimated*/
    DistanceWidth.fObjectWidth =
        CDGetPointer_Dimensions(pInputData->pObjectData, iObjectIndex)->fWidth;
    DistanceWidth.fObjectWidthVar =
        SQR(CD_OBJ_WIDTH_STDDEV(pInputData->pObjectData, iObjectIndex));

    /* calculate current overlap and occupancy */
    EMPCPCalculateOverlap(&DistanceWidth, &OccupancyNow);

    if (CDHypRunUpDetectNarrowCurve(pInputData, iObjectIndex, pObjInternData) !=
        FALSE) {
        /* Disable predicted lane association to prevent drop-out */
        fTTCPred = 0;
    } else {
        fTTCPred = MIN_FLOAT(pObjInternData->TTC, CD_RUN_UP_MAX_PRED_TIME);
    }

    /* calculate distance to course in t=MIN(TTC, CD_RUN_UP_MAX_PRED_TIME)    */
    fLatPosTTC = pObjInternData->TrajRelation.fDistToTraj +
                 (pObjInternData->TrajRelation.fVelocityToTraj * fTTCPred);

    /* set distance structure (for estimation in t=TTC) */
    DistanceWidth.fDistance = fLatPosTTC;
    DistanceWidth.fDistanceVar =
        pObjInternData->TrajRelation.fDistToTrajVar +
        (pObjInternData->TrajRelation.fVelocityToTrajVar * fTTCPred);
    DistanceWidth.fTrajectoryWidth = fRunUpTTCCorridorWidth * smallObjRedFact;
    /* widen corridor by increasing distance of object */
    DistanceWidth.fTrajectoryWidth =
        DistanceWidth.fTrajectoryWidth *
        CML_f_CalculatePolygonValue(CD_RUN_UP_TTC_WIDTH_FAC_NO_POINTS,
                                    CD_RUN_UP_TTC_TRACK_WIDTH_WIDE, fTTCPred);
    /* calculate overlap and occupancy (for estimation in t=TTC) */
    EMPCPCalculateOverlap(&DistanceWidth, &OccupancyTTC);

    /* calculate hypothesis probability*/
    /* estimation:  probability equals product of occupancy - scaling factor *
     * Variance */
    /*           safety factor uses the variances and the error propagation */

    /* Compute hypothesis probability based on current trajectory and object
     * occupancy */

    fUsedOccupancy = MAX_FLOAT((OccupancyNow.fObjectOccupancy -
                                SQRT(OccupancyNow.fObjectOccupancyVar)),
                               (OccupancyNow.fTrajectoryOccupancy -
                                SQRT(OccupancyNow.fTrajectoryOccupancyVar)));

    if (fUsedOccupancy > CD_RUN_UP_THRES_OVLC_1_NOW) {
        fQualityNow = 1.0f;
    } else if (fUsedOccupancy < CD_RUN_UP_THRES_OVLC_0_NOW) {
        fQualityNow = 0;
    } else {
        fQualityNow = (fUsedOccupancy - CD_RUN_UP_THRES_OVLC_0_NOW) /
                      (CD_RUN_UP_THRES_OVLC_1_NOW - CD_RUN_UP_THRES_OVLC_0_NOW);
    }

    /* store information in run-up structure*/

    /* Compute hypothesis probability based on trajectory and object occupancy
     * at TTC */

    fUsedOccupancy = MAX_FLOAT((OccupancyTTC.fObjectOccupancy -
                                SQRT(OccupancyTTC.fObjectOccupancyVar)),
                               (OccupancyTTC.fTrajectoryOccupancy -
                                SQRT(OccupancyTTC.fTrajectoryOccupancyVar)));

    _PARAM_UNUSED(pExternalFunctions);

    if (fUsedOccupancy > CD_RUN_UP_THRES_OVLC_1_TTC) {
        fQualityTTC = 1.0f;
    } else if (fUsedOccupancy < CD_RUN_UP_THRES_OVLC_0_TTC) {
        fQualityTTC = 0;
    } else {
        fQualityTTC = (fUsedOccupancy - CD_RUN_UP_THRES_OVLC_0_TTC) /
                      (CD_RUN_UP_THRES_OVLC_1_TTC - CD_RUN_UP_THRES_OVLC_0_TTC);
    }

    /*store information in run-up structure*/

    fQualityDistSpeedAccel =
        BML_f_BoundedLinInterpol2(
            CD_GET_DIST_X(pInputData->pObjectData, iObjectIndex),
            CD_RUN_UP_THRES_DIST_X0, CD_RUN_UP_THRES_DIST_X1,
            CD_RUN_UP_THRES_DIST_Y0, CD_RUN_UP_THRES_DIST_Y1) *
        BML_f_BoundedLinInterpol2(
            pInputData->pEgoData->pEgoDynRaw->fVelocityX,
            CD_RUN_UP_THRES_SPEED_X0, CD_RUN_UP_THRES_SPEED_X1,
            CD_RUN_UP_THRES_SPEED_Y0, CD_RUN_UP_THRES_SPEED_Y1) *
        BML_f_BoundedLinInterpol2(
            CDGetPointer_Kinematic(pInputData->pObjectData, iObjectIndex)
                    ->fArelX +
                pInputData->pEgoData->pEgoDynObjSync->fAccelX,
            CD_RUN_UP_THRES_ACCEL_X0, CD_RUN_UP_THRES_ACCEL_X1,
            CD_RUN_UP_THRES_ACCEL_Y0, CD_RUN_UP_THRES_ACCEL_Y1);
    fQualityDistSpeedAccel = MIN_FLOAT(1.0f, fQualityDistSpeedAccel);
    pHypothesis->fHypothesisProbability =
        fQualityNow * fQualityTTC * fQualityDistSpeedAccel;

    /* Reduce hypothesis probability in case for short brake strokes of
     * proceeding vehicle */
    CDHypoRunUpReducePropShortBrake(pHypothesis, pInputData, iObjectIndex,
                                    pObjInternData);
}

/* ***********************************************************************
  @fn            CDHypoRunUpMain */ /*!

                                         @brief         handles the CGEB
                                       situation analysis
                                                        hypothesis Run Up

                                         @description   Handles the Run Up
                                       hypothesis
                                                        (testing the hypothesis
                                       for each object)

                                         @param[in]     iObjectIndex The index
                                       of the object
                                         @param[in]     bObjFilterMatched If
                                       TRUE out object filter matched so
                                       hypothesis shall be calculated. If FALSE
                                       reset history (if exists)
                                         @param[in]     pInputData Pointer to CD
                                       input data
                                         @param[in]     pInternalStatus Pointer
                                       to CD internal status data
                                         @param[in]     pExternalFunctions
                                       Pointer to external functions

                                         @return        void

                                         @pre           [none]

                                         @post          [none]

                                       ****************************************************************************
                                       */
void CDHypoRunUpMain(ObjNumber_t iObjectIndex,
                     boolean bObjFilterMatched,
                     const CDInputData_t *pInputData,
                     CDInternalStatus_t *pInternalStatus,
                     const CDExternalFunctions_t *pExternalFunctions) {
    float32 fRunUpTTCCorridorWidth;
    CDIntHypothesis_t Hypothesis;
    float32 fCurvatureAtEvalDist;
    CDInternalObject_t *const pLocalObject =
        &(*pInternalStatus->rgObjInternal)[iObjectIndex];

    fCurvatureAtEvalDist =
        VLCSen_p_GetVDYDynObjSync()->Lateral.DrvIntCurve.Curve;
    fRunUpTTCCorridorWidth =
        CD_COMMON_TRACK_WIDTH_FACT *
        CML_f_CalculatePolygonValue(CD_RUN_UP_TTC_WIDTH_NO_POINTS,
                                    CD_RUN_UP_TTC_TRACK_WIDTH,
                                    fABS(fCurvatureAtEvalDist));

    /* initialize internal data */
    CDHypoRunUpInitInternalData(&pLocalObject->sHypRunUpData);

    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypPresel),
                     (uint32)CDHypothesisType_RunUp);
    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypPresel),
                     (uint32)CDHypothesisType_RunUpBraking);

    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypActive),
                     (uint32)CDHypothesisType_RunUp);
    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypActive),
                     (uint32)CDHypothesisType_RunUpBraking);

    /* handle only hypothesis relevant objects */
    if (bObjFilterMatched != FALSE) {
        VLCSEN_SERVICE_ADD_EVENT(
            e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_HYP_RUNUP_SINGLE,
            (uint8)(iObjectIndex)); /* start profiling for Hypothesis */
        if (CDHypoRunUpObjectFilter(iObjectIndex, pInputData->pObjectData,
                                    pLocalObject) != FALSE) {
            Hypothesis.fRelevance = 0;
            Hypothesis.fHypothesisProbability = 0;
            Hypothesis.fHypothesisLifetime = 0;

            /* link object to hypothesis */
            Hypothesis.iObjectRef = iObjectIndex;

            /* store object class */
            Hypothesis.eObjectClass =
                CDGetPointer_Attributes(pInputData->pObjectData, iObjectIndex)
                    ->eClassification;

            /* check if object is braking and set hypothesis type accordingly */
            if (CDGetPointer_Kinematic(pInputData->pObjectData, iObjectIndex)
                    ->fAabsX < CD_RUN_UP_OBJ_DECEL_BRAKING) {
                Hypothesis.eType = CDHypothesisType_RunUpBraking;
                CD_SET_HYP_BIT(&(pLocalObject->bitHypPresel),
                               (uint32)CDHypothesisType_RunUpBraking);
            } else {
                Hypothesis.eType = CDHypothesisType_RunUp;

                CD_SET_HYP_BIT(&(pLocalObject->bitHypPresel),
                               (uint32)CDHypothesisType_RunUp);
            }
            /* calculate Hypothesis probability */
            CDHypoRunUpCalculateProb(iObjectIndex, &Hypothesis,
                                     fRunUpTTCCorridorWidth, pInputData,
                                     pLocalObject, pExternalFunctions);
            /* store hypothesis (if relevant) */
            if (Hypothesis.fHypothesisProbability > CD_COMMON_MIN_HYP_PROB) {
                if (Hypothesis.eType == CDHypothesisType_RunUpBraking) {
                    CD_SET_HYP_BIT(&(pLocalObject->bitHypActive),
                                   (uint32)CDHypothesisType_RunUpBraking);
                } else {
                    CD_SET_HYP_BIT(&(pLocalObject->bitHypActive),
                                   (uint32)CDHypothesisType_RunUp);
                }
                CDHypothesesSelection(&Hypothesis, pInputData->pObjectData,
                                      pInternalStatus);
                pLocalObject->HypothesisHist.RunUpMoving = 1u;
            } else {
                pLocalObject->HypothesisHist.RunUpMoving = 0u;
            }
        } else {
            pLocalObject->HypothesisHist.RunUpMoving = 0u;
        }
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                 VLCSEN_RTA_CD_HYP_RUNUP_SINGLE,
                                 (uint8)(iObjectIndex));
    } else {
        pLocalObject->HypothesisHist.RunUpMoving = 0u;
    }
}

/* ************************************************************************* */
/*   Copyright Tuerme                     */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */