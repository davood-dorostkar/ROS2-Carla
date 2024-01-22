/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * guotao <guotao1@senseauto.com>
 * wulin <wulin1@senseauto.com> 20220316 add contents related to bicycle
 * collision calculation
 */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "./cd.h"

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief     Structure for Object kinematic history parameters

    @general   This structure is to store each parameter of the Object kinematic
   history(like x position, y position)

    @conseq    [None]

    @attention [None]

    */
typedef struct CDHistoryFloat {
    /*!< Last ID of the parameter of the Object kinematic history */
    uint8 uiLastID;
    /*!< Total valid value counts of the parameter of the Object kinematic
     * history */
    uint8 uiValidValueCount;
    /*!< Total distance values of every object to observe to fit velocities */
    float32 afValues[CD_BICYCLE_COLL_OBSERVE_Y_DIST_N];
} CDHistoryFloat_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief      Target object kinematic history structure

    @general    This structure stores the kinematic history of the target object
   for Bicycle Collision Hypothesis

    @conseq     [None]

    @attention  [None]

    */
typedef struct CDKinHistObj {
    /*!< Y distance of the target object from ego object */
    CDHistoryFloat_t sPosY;
    /*!< Variation in Y distance of the target object from ego object */
    CDHistoryFloat_t sPosYVar;
    /*!< X distance of the target object from ego object */
    CDHistoryFloat_t sPosX;
    /*!< Variation in X distance of the target object from ego object */
    CDHistoryFloat_t sPosXVar;
} CDKinHistObj_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief       Ego object kinematic history structure

    @general     This structure stores the kinematic history of the ego object
   for Bicycle Collision Hypothesis

    @conseq      [None]

    @attention   [None]

    */
typedef struct CDKinHistEgo {
    CDHistoryFloat_t sVel;     /*!< Velocity of the ego object */
    CDHistoryFloat_t sAccel;   /*!< Acceleration of the ego object */
    CDHistoryFloat_t sYawRate; /*!< YawRate of the ego object */
} CDKinHistEgo_t;

/* ****************************************************************
    TYPEDEF
    **************************************************************** */
/*! @brief     Bicycle Collision Hypothesis object history list

    @general   List to store the object history of the total
   objects(Envm_N_OBJECTS) used for Bicycle Collision Hypothesis

    */
typedef CDKinHistObj_t CDBicycleHypInternalObjHistList_t[Envm_N_OBJECTS];

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*! @brief      Oldest factor used in calculating weights for y distance history
 */
#define BICYCLE_VELO_WEIGHT_OLDEST (1.0f)

/*! @brief      Youngest factor used in calculating weights for y distance
 * history */
#define BICYCLE_VELO_WEIGHT_YOUNGEST (3.0f)

/*! @brief      standard deviation of 0.7m for camera measured distances
 * expected */
#define BICYCLE_VELO_WEIGHT_STD_B (0.7f)

/*! @brief     Constant used to calculate allowed error */
#define BICYCLE_VELO_ALLOWED_CONST_ERROR_M (0)

/*! @brief     Probability constant A used to map error to Probability */
#define BICYCLE_VELO_MAP_ERROR_PROB_CONST_A (0.0031f)

/*! @brief     Probability constant B used to map error to Probability */
#define BICYCLE_VELO_MAP_ERROR_PROB_CONST_B (0.0015f)

/*! @brief      Path factor used for calculation of collision corridor velocity
 */
#define BICYCLE_VELO_VEHICLE_PATH_FACTOR (1.0f)

/*! @brief      Maximum weight of measured bicycle velocity added to velocity
 * probability density function */
#define BICYCLE_VELO_ADD_MEASURED_VELO_MAX_WEIGHT (0.8f)

/*****************************************************************************
  VARIABLES
*****************************************************************************/
/*!  @cond Doxygen_Suppress */
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static CDBicycleHypInternalObjHistList_t CDBicycleHypInternalObjHistList;
static CDKinHistEgo_t CDBicycleHypInternalEgoHist;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! @endcond */
/*****************************************************************************
  LOCAL FUNCTIONS Declarations
*****************************************************************************/
static boolean CDHypoBicycleCollObjectFilter(
    ObjNumber_t iObjectIndex,
    const CDObjectData_t *pObjectData,
    const CDInternalObject_t *pCurrentObjIntern);
static void CDHypoBicycleCollCalculateProb(
    ObjNumber_t iObjectIndex,
    CDIntHypothesis_t *pHypothesis,
    const CDInputData_t *pInputData,
    const CDInternalStatus_t *pInternalStatus,
    CDInternalObject_t *pCurrentObjIntern);
static void CDHypoBicycleCollCalculatePSCProb(
    ObjNumber_t iObjectIndex,
    const CDInternalStatus_t *pInternalStatus,
    CDInternalObject_t *pCurrentObjIntern,
    const float32 fObjDistToEgoTraj_m,
    const float32 fObjYProjLength_m,
    const float32 fEgoWidth_m,
    float32 *pOut_fPSCProb);

static boolean HistIntegrate(const float32 afXValues[],
                             const float32 afYValues[],
                             float32 fLeftIntLimX,
                             float32 fRightIntLimX,
                             const uint8 uiLenXYVec,
                             float32 *pOut_fHistIntegralResult);
static void AddSwapParameters(float32 *fFrstValToSwap, float32 *fScndValToSwap);

static void CDHypoBicycleUpdateObjHist(ObjNumber_t iObjectIndex,
                                       const CDObjectData_t *pObjectData);
static void CDTransform2DCoordHistoryFloat(GDBTrafoMatrix2D_t const *M,
                                           CDHistoryFloat_t *FloatHistory_X,
                                           CDHistoryFloat_t *FloatHistory_Y);
static void CDHypoBicycleUpdateEgoHist(const EMPKinEgo_t *pEgoKin);
static void CDPushHistoryFloat(float32 fLatestValue,
                               CDHistoryFloat_t *pIn_FloatHistory);
static void CDResetHistoryFloat(CDHistoryFloat_t *pIn_FloatHistory);
static void CDResetObjHistoryFloat(CDKinHistObj_t *pIn_ObjHistory);
static uint8 CDGetCompleteHistory(const CDHistoryFloat_t *pIn_FloatHistory,
                                  float32 afHistory[],
                                  uint8 uiArrayLength);
static boolean CDGetNthHistoryFloat(uint8 uiSteps,
                                    const CDHistoryFloat_t *pIn_FloatHistory,
                                    float32 *pOut_Result);
static boolean CDCalcHistDistToTraj(const EMPKinEgo_t *pIn_KinHistEgo,
                                    const CDKinHistObj_t *pIn_Obj,
                                    float32 afHistDistToTraj[],
                                    float32 afHistDistOnTraj[]);

static void CDBicycleCollDowngradeProbDominantVelo(
    float32 *out_fDownGradeFac,
    const float32 fObjVrelToEgoTraj_mps,
    const float32 fObjVrelOnEgoTraj_mps,
    const float32 fObjDistToEgoTraj_m,
    const float32 fEgoVeloDepWidth_m);

static void iterateAllPossVelo(const uint8 uiNumberOfValidDataPoints,
                               const float32 fStepSizePossibleVelocity_mps,
                               const float32 afDistToTrajVarHistory[],
                               const float32 afDistToTrajHistory[],
                               const float32 afPossibleVelocity_mps[],
                               float32 afTimeVec_s[],
                               float32 afVelocityProbabilities[]);
/*****************************************************************************
  LOCAL FUNCTIONS Definitions
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    CDHypoBicycleCollObjectFilter */ /*!

                                                                             @brief
                                                                           Checks
                                                                           whether
                                                                           object
                                                                           is
                                                                           relevant
                                                                           for
                                                                           Bicycle
                                                                           Collision
                                                                           Hypothesis

                                                                             @description
                                                                           he
                                                                           function
                                                                           check
                                                                           the
                                                                           object
                                                                           quality
                                                                           and
                                                                           filter
                                                                           out
                                                                           the
                                                                           relevant
                                                                           objects
                                                                           for
                                                                                              calculation of Bicycle Collision Hypothesis

                                                                             @return
                                                                           true
                                                                           if
                                                                           the
                                                                           object
                                                                           is
                                                                           relevant

                                                                             @param[in]
                                                                           iObjectIndex
                                                                           :
                                                                           Index
                                                                           of
                                                                           the
                                                                           object
                                                                           to
                                                                           check
                                                                             @param[in]
                                                                           *pObjectData
                                                                           :
                                                                           Pointer
                                                                           to
                                                                           the
                                                                           object
                                                                           data
                                                                             @param[in]
                                                                           *pCurrentObjIntern
                                                                           :


                                                                           *************************************************************************************************************************/
static boolean CDHypoBicycleCollObjectFilter(
    ObjNumber_t iObjectIndex,
    const CDObjectData_t *pObjectData,
    const CDInternalObject_t *pCurrentObjIntern) {
    boolean bAcceptObj = TRUE;

    if (pCurrentObjIntern->TTC3 > 4.0f) {
        bAcceptObj = FALSE;
    }

    _PARAM_UNUSED(pObjectData);
    _PARAM_UNUSED(pCurrentObjIntern);
    _PARAM_UNUSED(iObjectIndex);
    /* Check for Class Bicycles is already done in generic Class filter */

    return bAcceptObj;
}

/* ***********************************************************************
  @fn            CDHypoBicycleCollCalculateProb */ /*!

                              @brief         Calculates the Bicycle Collision
                            Hypothesis Probability

                              @description   This function calculates the
                            Bicycle Collision Hypothesis Probability by
                                             fusing Probability of Speed
                            Concept, Corridor Approach Probability and
                            EMPCalcCollProbBicycle
                                             (from EMP module) probability
                            values and finally downgrade hypothesis probability
                            in case
                                             the dominent object velocity
                            direction is not crossing the ego vehicle path. This
                            is to ensure
                                             FalsePositive suppressions

                              @param[in]     iObjectIndex The index of the
                            object
                              @param[in,out] pHypothesis
                              @param[in]     pInputData Pointer to CD input data
                              @param[in]     pInternalStatus
                              @param[in]     pCurrentObjIntern

                              @return        void

                            ****************************************************************************
                            */
static void CDHypoBicycleCollCalculateProb(
    ObjNumber_t iObjectIndex,
    CDIntHypothesis_t *pHypothesis,
    const CDInputData_t *pInputData,
    const CDInternalStatus_t *pInternalStatus,
    CDInternalObject_t *pCurrentObjIntern) {
    /* Collect Object Informations */

    const float32 fObjDistToEgoTraj_m =
        pCurrentObjIntern->TrajRelation.fDistToTraj;
    const float32 fObjVrelToEgoTraj_mps =
        pCurrentObjIntern->TrajRelation.fVelocityToTraj;
    const float32 fObjVrelOnEgoTraj_mps =
        pCurrentObjIntern->TrajRelation.fVelocityOnTraj;

    const float32 fObjWidth_m = MIN_FLOAT(
        CD_BICYCLE_WIDTH_SEC,
        CDGetPointer_Dimensions(pInputData->pObjectData, iObjectIndex)->fWidth);
    const float32 fObjLength_m =
        MIN_FLOAT(CD_BICYCLE_LENGTH_SEC,
                  CDGetPointer_Dimensions(pInputData->pObjectData, iObjectIndex)
                      ->fLength);

    const float32 fObjOrientation_rad =
        CDGetPointer_Dimensions(pInputData->pObjectData, iObjectIndex)
            ->fOrientation;

    const float32 fEgoVeloDepWidth_m =
        CD_COMMON_EGO_WIDTH + CML_f_CalculatePolygonValue(
                                  CD_NUMBER_OF_EGO_ADD_WIDTH_VELO_DEP_VALUES,
                                  CD_EGO_ADD_WIDTH_VELO_DEP_M,
                                  pInputData->pEgoData->pEgoDynRaw->fVelocityX);

    /* Calculate Object dependent Informations */
    const float32 fCosObjOrientation = COS_(fObjOrientation_rad);
    const float32 fSinObjOrientation = SIN_(fObjOrientation_rad);

    /* Probability of Speed Concept */
    float32 fPSCProb = 1.0f;
    float32 fCorrApproachProb = 1.0f;
    float32 fObjCollProb = 1.0f; /*i.e., EMPCalcCollProbBicycle probability*/

    {
        const float32 fObjYProjLength_m =
            (fABS(fCosObjOrientation) * fObjWidth_m) +
            (fABS(fSinObjOrientation) *
             fObjLength_m); /* projected width for ego view */
        /********************************************/
        /* Calculate Probability of Speed Concept   */
        /********************************************/
        CDHypoBicycleCollCalculatePSCProb(
            iObjectIndex, pInternalStatus, pCurrentObjIntern,
            fObjDistToEgoTraj_m, fObjYProjLength_m, fEgoVeloDepWidth_m,
            &fPSCProb);
    }

    /***********************************/
    /* Fuse Probs                      */
    /***********************************/
    pHypothesis->fHypothesisProbability = fPSCProb * fCorrApproachProb;

    _PARAM_UNUSED(fObjDistToEgoTraj_m);

    pHypothesis->fHypothesisProbability *= fObjCollProb;

    {
        /* Downgrade hypothesis probability in case of dominant velocity
         * direction is not crossing! */
        float32 out_fDownGradeFac = 1.0f;
        CDBicycleCollDowngradeProbDominantVelo(
            &out_fDownGradeFac, fObjVrelToEgoTraj_mps, fObjVrelOnEgoTraj_mps,
            fObjDistToEgoTraj_m, fEgoVeloDepWidth_m);
        pHypothesis->fHypothesisProbability *= out_fDownGradeFac;
        pCurrentObjIntern->sHypBicycleCollData.fDowngradeFactor =
            out_fDownGradeFac;
    }

    _PARAM_UNUSED(fObjVrelOnEgoTraj_mps);
}

/* ***********************************************************************
  @fn            CDBicycleCollDowngradeProbDominantVelo */ /*!

                      @brief         Downgrade the Bicycle Collision
                    Probability

                      @description   The function downgrade the Bicycle
                    Collision Probability when the dominent
                                     object velocity direction is not crossing
                    the ego vehicle path.This ensures the
                                     some of the FalsePositive suppressions

                      @param[in]     fObjVrelToEgoTraj_mps
                      @param[in]     fObjVrelOnEgoTraj_mps
                      @param[in]     fObjDistToEgoTraj_m
                      @param[in]     fEgoVeloDepWidth_m
                      @param[out]    out_fDownGradeFac

                      @return        void

                    ****************************************************************************
                    */
static void CDBicycleCollDowngradeProbDominantVelo(
    float32 *out_fDownGradeFac,
    const float32 fObjVrelToEgoTraj_mps,
    const float32 fObjVrelOnEgoTraj_mps,
    const float32 fObjDistToEgoTraj_m,
    const float32 fEgoVeloDepWidth_m) {
    /*
    Thoughts:
    Downgrading the Bicycle collision probability is a workaround, to use
    this hypothesis for bicycles also. To downgrade an objects, which move on
    trajectory with the ego-vehicle means: near overtake scenarios with bicycles
    are suppressed.

    FalsePositive suppressions could be:
    - Pedestrian/Bicycle standing -> a minimum velocity required
    - Pedestrian/Bicycle moving inside the ego vehicle path
    */
    const float32 fBicycleDistToVehiclePath_m =
        MAX_FLOAT(0, fABS(fObjDistToEgoTraj_m) - (fEgoVeloDepWidth_m * 0.5f));
    const float32 fMaxRate = CML_f_CalculatePolygonValue(
        CD_NUMBER_OF_DOMINANTDOWNGRADE_MAX_RATE_VALUES,
        CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MAX_RATE_VEC,
        fBicycleDistToVehiclePath_m);
    const float32 fMinFactor = CML_f_CalculatePolygonValue(
        CD_NUMBER_OF_DOMINANTDOWNGRADE_MIN_FACTOR_VALUES,
        CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_FACTOR_VEC,
        fBicycleDistToVehiclePath_m);

    /* Be careful: not the real velocity of the object. SQRT not done because of
     * efficiency */
    const float32 fObjVelo_mps =
        SQR(fObjVrelOnEgoTraj_mps) + SQR(fObjVrelToEgoTraj_mps);
    const boolean bBicycleIsMoving =
        (boolean)(fObjVelo_mps >
                  CD_BICYCLE_COLL_DOMINANT_DOWNGRADE_MIN_OBJ_VELO_MPS);

    const boolean bBicycleIsNotInVehiclePath =
        (boolean)(fABS(fObjDistToEgoTraj_m) > (fEgoVeloDepWidth_m * 0.5f));

    float32 fVrelToOnTrajRate;

    *out_fDownGradeFac = 1.0f;

    if (fABS(fObjVrelOnEgoTraj_mps) > C_F32_DELTA) {
        fVrelToOnTrajRate =
            fABS(fObjVrelToEgoTraj_mps) / fABS(fObjVrelOnEgoTraj_mps);
        if ((FALSE != bBicycleIsMoving) &&
            (FALSE != bBicycleIsNotInVehiclePath)) {
            *out_fDownGradeFac = MAX_FLOAT(
                fMinFactor, MIN_FLOAT(1.0f, fVrelToOnTrajRate / fMaxRate));
        }
    }
}

/* ***********************************************************************
  @fn            CDHypoBicycleCollCalculatePSCProb */ /*!

                           @brief         Bicycle Collision Hypothesis
                         calculation with the Probability of Speed Concept

                                          1. Calculation of the speed range, the
                         Bicycle has to run for a collision
                                          2. Calculation of the probability that
                         the Bicycle run a certain speed (Probability
                         Density)
                                          3. Integration of the Density for the
                         collision speed corridor

                           @param[in]     iObjectIndex
                           @param[in]     pInternalStatus
                           @param[in]     pCurrentObjIntern
                           @param[in]     fObjDistToEgoTraj_m
                           @param[in]     fObjYProjLength_m
                           @param[in]     fEgoWidth_m
                           @param[out]    pOut_fPSCProb

                           @return        void

                         ****************************************************************************
                         */
static void CDHypoBicycleCollCalculatePSCProb(
    ObjNumber_t iObjectIndex,
    const CDInternalStatus_t *pInternalStatus,
    CDInternalObject_t *pCurrentObjIntern,
    const float32 fObjDistToEgoTraj_m,
    const float32 fObjYProjLength_m,
    const float32 fEgoWidth_m,
    float32 *pOut_fPSCProb) {
    /* Probability of Speed Concept */
    float32 fCollCorrVeloLeft_mps;
    float32 fCollCorrVeloRight_mps;
    uint8 i;
    float32 fStepSizePossibleVelocity_mps;
    float32 afPossibleVelocity_mps[CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N];
    float32 afVelocityProbabilities[CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N];

    uint8 uiNumberOfValidDataPoints;
    float32 afDistToTrajHistory[CD_BICYCLE_COLL_OBSERVE_Y_DIST_N];
    float32 afDistOnTrajHistory[CD_BICYCLE_COLL_OBSERVE_Y_DIST_N];
    float32 afDistToTrajVarHistory[CD_BICYCLE_COLL_OBSERVE_Y_DIST_N];
    float32 afYDistVarHistory[CD_BICYCLE_COLL_OBSERVE_Y_DIST_N];
    float32 afTimeVec_s[CD_BICYCLE_COLL_OBSERVE_Y_DIST_N];

    float32 fNormFac;

    const float32 fBicycleVeloToTraj =
        pCurrentObjIntern->TrajRelation.fVelocityToTraj;
    const float32 fBicycleVeloToTrajStd =
        MAX(CD_BICYCLE_COLL_BICYCLE_MIN_VELO_TO_TRAJ_STD,
            SQRT(pCurrentObjIntern->TrajRelation.fVelocityToTrajVar));

    /* Get the minimal and maximal collision corridor velocity */
    if (pCurrentObjIntern->TTC3 > C_F32_DELTA) {
        /*physical constant*/
        fCollCorrVeloLeft_mps =
            -((fObjDistToEgoTraj_m - (fObjYProjLength_m / 2.0f)) -
              ((fEgoWidth_m * BICYCLE_VELO_VEHICLE_PATH_FACTOR) / 2.0f)) /
            pCurrentObjIntern->TTC3;
        fCollCorrVeloRight_mps =
            -((fObjDistToEgoTraj_m + (fObjYProjLength_m / 2.0f)) +
              ((fEgoWidth_m * BICYCLE_VELO_VEHICLE_PATH_FACTOR) / 2.0f)) /
            pCurrentObjIntern->TTC3;
    } else {
        fCollCorrVeloLeft_mps = 100.0f;
        fCollCorrVeloRight_mps = 100.0f;
    }

    /* Get the possible velocities to check */
    fStepSizePossibleVelocity_mps =
        (CD_BICYCLE_COLL_BICYCLE_VELO_MAX_MPS -
         CD_BICYCLE_COLL_BICYCLE_VELO_MIN_MPS) /
        (float32)(CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N - 1u);
    for (i = 0; i < CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N; ++i) {
        afPossibleVelocity_mps[i] = (CD_BICYCLE_COLL_BICYCLE_VELO_MIN_MPS +
                                     (i * fStepSizePossibleVelocity_mps));
    }

    /*
      Start calculation of a velocity probability density function based on the
      last measured y distance values
    */

    /* Get the last y dist values */
    uiNumberOfValidDataPoints = CDGetCompleteHistory(
        &(CDBicycleHypInternalObjHistList[iObjectIndex].sPosYVar),
        afYDistVarHistory, (uint8)CD_BICYCLE_COLL_OBSERVE_Y_DIST_N);

    (void)CDCalcHistDistToTraj(&((*pInternalStatus).sKinEgo),
                               &(CDBicycleHypInternalObjHistList[iObjectIndex]),
                               afDistToTrajHistory, afDistOnTrajHistory);

    for (i = 0; i < CD_BICYCLE_COLL_OBSERVE_Y_DIST_N; ++i) {
        afDistToTrajVarHistory[i] = afYDistVarHistory[i];
    }

    // Iterate over all possible velocities
    iterateAllPossVelo(uiNumberOfValidDataPoints, fStepSizePossibleVelocity_mps,
                       afDistToTrajVarHistory, afDistToTrajHistory,
                       afPossibleVelocity_mps, afTimeVec_s,
                       afVelocityProbabilities);

    /* Normalize Probability */
    if ((FALSE !=
         HistIntegrate(
             afPossibleVelocity_mps, afVelocityProbabilities,
             afPossibleVelocity_mps[0],
             afPossibleVelocity_mps[CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N - 1],
             CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N, &(fNormFac))) &&
        (fNormFac > C_F32_DELTA)) {
        for (i = 0; i < CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N; ++i) {
            afVelocityProbabilities[i] = afVelocityProbabilities[i] / fNormFac;
        }
    }

    /* Add Data to CDInternalObject for Measefreeze */
    pCurrentObjIntern->sHypBicycleCollData.fCollCorrVeloLeft_mps =
        fCollCorrVeloLeft_mps;
    pCurrentObjIntern->sHypBicycleCollData.fCollCorrVeloRight_mps =
        fCollCorrVeloRight_mps;
    for (i = 0; i < CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N; ++i) {
        pCurrentObjIntern->sHypBicycleCollData
            .afVelocityProbabilitiesOnlyFitting[i] =
            (uint8)(afVelocityProbabilities[i] * 100);
    }
    /* Integrate afVelocityProbabilities between CollCorrVelo{left,right} */
    if (FALSE !=
        HistIntegrate(
            afPossibleVelocity_mps, afVelocityProbabilities,
            fCollCorrVeloLeft_mps, fCollCorrVeloRight_mps,
            CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N,
            &(pCurrentObjIntern->sHypBicycleCollData.fHypProbOnlyFitting))) {
        /*the upper limit of probabilities is 1*/

        pCurrentObjIntern->sHypBicycleCollData.fHypProbOnlyFitting = MIN_FLOAT(
            1.0f, (pCurrentObjIntern->sHypBicycleCollData.fHypProbOnlyFitting));
    }

    /* Add measured object lateral velocity as gauss kernel to velocity
     * probability dense function */
    {
        const float32 fWeight =
            ((float32)OBJ_GET_EBA_MOV_PRESEL_QUALITY(iObjectIndex) / 100.0f) *
            BICYCLE_VELO_ADD_MEASURED_VELO_MAX_WEIGHT;
        float32
            afVelocityMeasuredGaussian[CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N];

        for (i = 0; i < CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N; ++i) {
            /* Calculate the Integral of the Gauss Function between the discrete
             * velocity bins */
            const float32 fGaussCDFLeft = BML_f_CalcStdGaussianCDF(
                (afPossibleVelocity_mps[i] -
                 (fStepSizePossibleVelocity_mps / 2.0f)),
                fBicycleVeloToTraj, fBicycleVeloToTrajStd);
            const float32 fGaussCDFRight = BML_f_CalcStdGaussianCDF(
                (afPossibleVelocity_mps[i] +
                 (fStepSizePossibleVelocity_mps / 2.0f)),
                fBicycleVeloToTraj, fBicycleVeloToTrajStd);
            afVelocityMeasuredGaussian[i] = fGaussCDFRight - fGaussCDFLeft;
        }

        if ((FALSE !=
             HistIntegrate(
                 afPossibleVelocity_mps, afVelocityMeasuredGaussian,
                 afPossibleVelocity_mps[0],
                 afPossibleVelocity_mps[CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N -
                                        1],
                 CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N, &(fNormFac))) &&
            (fNormFac > C_F32_DELTA)) {
            for (i = 0; i < CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N; ++i) {
                afVelocityMeasuredGaussian[i] =
                    afVelocityMeasuredGaussian[i] / fNormFac;
            }
        }

        for (i = 0; i < CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N; ++i) {
            afVelocityProbabilities[i] =
                afVelocityProbabilities[i] +
                ((afVelocityMeasuredGaussian[i] - afVelocityProbabilities[i]) *
                 fWeight);
        }

        /* Add Data to CDInternalObject for Measefreeze */
        for (i = 0; i < CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N; ++i) {
            pCurrentObjIntern->sHypBicycleCollData
                .afVelocityProbabilitiesOnlyGaussian[i] =
                (uint8)(afVelocityMeasuredGaussian[i] * 100.0f);
            pCurrentObjIntern->sHypBicycleCollData
                .afVelocityProbabilitiesPlusGaussian[i] =
                (uint8)(afVelocityProbabilities[i] * 100.0f);
        }
        /* Integrate afVelocityMeasuredGaussian between CollCorrVelo{left,right}
         */
        if (FALSE !=
            HistIntegrate(afPossibleVelocity_mps, afVelocityMeasuredGaussian,
                          fCollCorrVeloLeft_mps, fCollCorrVeloRight_mps,
                          CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N,
                          &(pCurrentObjIntern->sHypBicycleCollData
                                .fHypProbOnlyGaussian))) {
            pCurrentObjIntern->sHypBicycleCollData.fHypProbOnlyGaussian =
                MIN_FLOAT(1.0f, (pCurrentObjIntern->sHypBicycleCollData
                                     .fHypProbOnlyGaussian));
        }
    }

    /* Integrate afVelocityProbabilities between CollCorrVelo{left,right} */
    if (FALSE != HistIntegrate(afPossibleVelocity_mps, afVelocityProbabilities,
                               fCollCorrVeloLeft_mps, fCollCorrVeloRight_mps,
                               CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N,
                               pOut_fPSCProb)) {
        *pOut_fPSCProb = MIN_FLOAT(1.0f, *pOut_fPSCProb);
    }
    /* Add Data to CDInternalObject for Measefreeze */
    pCurrentObjIntern->sHypBicycleCollData.fHypProbPlusGaussian =
        *pOut_fPSCProb;

    _PARAM_UNUSED(iObjectIndex);
    _PARAM_UNUSED(pInternalStatus);
} /* End Calculate PSC*/

/* ***********************************************************************
  @fn            CDPushHistoryFloat  */ /*!

                                     @brief         Adds a new float to the
                                   History

                                     @param[in]     fLatestValue
                                     @param[in,out] pIn_FloatHistory

                                     @return        void

                                   ****************************************************************************
                                   */
static void CDPushHistoryFloat(float32 fLatestValue,
                               CDHistoryFloat_t *pIn_FloatHistory) {
    /* Determine ID to save latest Value */
    uint8 uiNextID =
        (pIn_FloatHistory->uiLastID + 1u) % CD_BICYCLE_COLL_OBSERVE_Y_DIST_N;
    /* Update LastID */
    pIn_FloatHistory->afValues[uiNextID] = fLatestValue;
    pIn_FloatHistory->uiLastID = uiNextID;
    pIn_FloatHistory->uiValidValueCount =
        MIN((pIn_FloatHistory->uiValidValueCount + 1),
            CD_BICYCLE_COLL_OBSERVE_Y_DIST_N);
}

/* ***********************************************************************
  @fn            CDResetObjHistoryFloat  */ /*!

                                 @brief         Reset all history values to 0.f

                                 @param[out]    pIn_ObjHistory

                                 @return        void

                               ****************************************************************************
                               */
static void CDResetObjHistoryFloat(CDKinHistObj_t *pIn_ObjHistory) {
    CDResetHistoryFloat(&(pIn_ObjHistory->sPosX));
    CDResetHistoryFloat(&(pIn_ObjHistory->sPosXVar));
    CDResetHistoryFloat(&(pIn_ObjHistory->sPosY));
    CDResetHistoryFloat(&(pIn_ObjHistory->sPosYVar));
}

/* ***********************************************************************
  @fn            CDResetHistoryFloat  */ /*!

                                    @brief         Reset all history values to
                                  0.f

                                    @param[out]    pIn_FloatHistory

                                    @return        void

                                  ****************************************************************************
                                  */
static void CDResetHistoryFloat(CDHistoryFloat_t *pIn_FloatHistory) {
    uint8 uiCurrID = 0u;

    for (uiCurrID = 0; uiCurrID < CD_BICYCLE_COLL_OBSERVE_Y_DIST_N;
         ++uiCurrID) {
        pIn_FloatHistory->afValues[uiCurrID] = 0;
    }

    pIn_FloatHistory->uiLastID = 0u;
    pIn_FloatHistory->uiValidValueCount = 0u;
}

/* ***********************************************************************
  @fn            CDGetCompleteHistory  */ /*!

                                   @brief         Returns the complete history
                                 of the Float as float array

                                   @param[in]     pIn_FloatHistory
                                   @param[out]    afHistory[]
                                   @param[in]     uiArrayLength

                                   @return        Number of valid values

                                 ****************************************************************************
                                 */
static uint8 CDGetCompleteHistory(const CDHistoryFloat_t *pIn_FloatHistory,
                                  float32 afHistory[],
                                  uint8 uiArrayLength) {
    uint8 i;
    uint8 uiValidCount = 0u;
    /* Reset Result Array */
    for (i = 0; i < uiArrayLength; ++i) {
        afHistory[i] = 0;
    }
    /* Save all valid values */
    for (i = 0; i < uiArrayLength; ++i) {
        if (FALSE != CDGetNthHistoryFloat(i, pIn_FloatHistory, &afHistory[i])) {
            uiValidCount += 1u;
        }
    }
    /* Return count of valid values */
    return uiValidCount;
}

/* ***********************************************************************
  @fn            CDGetNthHistoryFloat  */ /*!

                                   @brief         Returns the Nth-History Float.
                                                  If steps >
                                 CD_BICYCLE_COLL_OBSERVE_Y_DIST_N it returns the
                                 oldest value available.

                                   @param[in]     uiSteps
                                   @param[in]     pIn_FloatHistory
                                   @param[out]    pOut_Result

                                   @return        bool

                                 ****************************************************************************
                                 */
static boolean CDGetNthHistoryFloat(uint8 uiSteps,
                                    const CDHistoryFloat_t *pIn_FloatHistory,
                                    float32 *pOut_Result) {
    sint8 iShiftedID;
    uint8 uiTargetID;
    boolean bValidStep = TRUE;

    /* Limit requested history ID to maximum if too big */
    if (uiSteps > (pIn_FloatHistory->uiValidValueCount - 1u)) {
        uiSteps = pIn_FloatHistory->uiValidValueCount - 1u;
        bValidStep = FALSE;
    }

    iShiftedID = (sint8)pIn_FloatHistory->uiLastID - (sint8)uiSteps;

    /* Wrap around */
    if (iShiftedID < 0) {
        uiTargetID = (uint8)(CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - (-iShiftedID));
    } else {
        uiTargetID = (uint8)iShiftedID;
    }

    *pOut_Result = pIn_FloatHistory->afValues[uiTargetID];
    return bValidStep;
}

/* ****************************************************************************

  @fn           CDCalcHistDistToTraj   */ /*!

                                        @brief        Calculates the Distance to
                                      Trajectory for a given ObjectDescription

                                        @param[in]    pIn_KinHistEgo
                                        @param[in]    pIn_Obj
                                        @param[out]   afHistDistToTraj
                                        @param[out]   afHistDistOnTraj

                                        @return       True if no error occurred


                                      ****************************************************************************
                                      */
static boolean CDCalcHistDistToTraj(const EMPKinEgo_t *pIn_KinHistEgo,
                                    const CDKinHistObj_t *pIn_Obj,
                                    float32 afHistDistToTraj[],
                                    float32 afHistDistOnTraj[]) {
    boolean bResultOk = TRUE;
    uint8 uiCurrTimeStep = 0u;
    EMPTrajPred_t sEgoTrajPred;
    EMPResetTrajPred(&sEgoTrajPred);

    /* Predict Ego Trajectory */
    EMPPredictEgoTraj(pIn_KinHistEgo, EMP_MANEUVER_KinematicsUnchanged,
                      &sEgoTrajPred);

    for (uiCurrTimeStep = 0; uiCurrTimeStep < CD_BICYCLE_COLL_OBSERVE_Y_DIST_N;
         ++uiCurrTimeStep) {
        EMPTrajPred_t sObjTrajPred;
        float32 fCurrMinDistTime, fCurrEuclideanDist, fCurrDistOn;
        EMPPos2D_t fPosEgo, fPosObj;

        /* Fill Obj of current TimeStep */

        EMPResetTrajPred(&sObjTrajPred);
        (void)CDGetNthHistoryFloat(uiCurrTimeStep, &(pIn_Obj->sPosX),
                                   &(sObjTrajPred.XofT.fC0));
        (void)CDGetNthHistoryFloat(uiCurrTimeStep, &(pIn_Obj->sPosY),
                                   &(sObjTrajPred.YofT.fC0));

        bResultOk &=
            EMPCalcMinDistTime(&sEgoTrajPred, &sObjTrajPred, &fCurrMinDistTime);

        EMPCalcPositionAtTime(fCurrMinDistTime, &sEgoTrajPred, &fPosEgo);
        EMPCalcPositionAtTime(fCurrMinDistTime, &sObjTrajPred, &fPosObj);

        if ((fCurrMinDistTime < EMP_PREDICTION_TIME_EGO_MAX) &&
            (fCurrMinDistTime > C_F32_DELTA)) {
            fCurrEuclideanDist = EMPCalcObjObjDistAtTime(
                fCurrMinDistTime, &sEgoTrajPred, &sObjTrajPred);
            fCurrDistOn = ((fCurrMinDistTime * fCurrMinDistTime) *
                           sEgoTrajPred.XofT.fC2) +
                          (fCurrMinDistTime * sEgoTrajPred.XofT.fC1) +
                          sEgoTrajPred.XofT.fC0;

            if (fPosEgo.fY > fPosObj.fY) {
                fCurrEuclideanDist *= -1.f;
            }
            /* DistToTraj is signed to indicate the side */
            afHistDistToTraj[uiCurrTimeStep] = fCurrEuclideanDist;
            afHistDistOnTraj[uiCurrTimeStep] = fCurrDistOn;
        } else {
            /* Object is temporally too far away from ego vehicle to calculate a
             * reasonable DistToTraj */
            if (fPosEgo.fY > fPosObj.fY) {
                afHistDistToTraj[uiCurrTimeStep] = -EMP_DIST_TO_TRAJ_MAX;
            } else {
                afHistDistToTraj[uiCurrTimeStep] = EMP_DIST_TO_TRAJ_MAX;
            }

            /* Object is temporally too far away from ego vehicle to calculate a
             * reasonable DistOnTraj */
            if (fPosEgo.fX > fPosObj.fX) {
                afHistDistOnTraj[uiCurrTimeStep] = -EMP_DIST_ON_TRAJ_MAX;
            } else {
                afHistDistOnTraj[uiCurrTimeStep] = EMP_DIST_ON_TRAJ_MAX;
            }
        }
    }

    return bResultOk;
}

/* ***********************************************************************
  @fn                CDTransform2DCoordHistoryFloat  */ /*!

                     @brief             Transform all values in history float to
                   the current coordinate system

                     @param[in]         M
                     @param[in,out]     FloatHistory_X
                     @param[in,out]     FloatHistory_Y


                     @return            Number of valid values

                   ****************************************************************************
                   */
static void CDTransform2DCoordHistoryFloat(GDBTrafoMatrix2D_t const *M,
                                           CDHistoryFloat_t *FloatHistory_X,
                                           CDHistoryFloat_t *FloatHistory_Y) {
    uint8 i;

    for (i = 0; i < CD_BICYCLE_COLL_OBSERVE_Y_DIST_N; ++i) {
        if (i != FloatHistory_X->uiLastID) {
            BML_v_TransformPosition2D(M, &(FloatHistory_X->afValues[i]),
                                      &(FloatHistory_Y->afValues[i]));
        }
    }
}

/* ***********************************************************************
  @fn            CDHypoBicycleUpdateObjHist */ /*!

                                  @brief         Updates the Bicycle
                                Hypothesis internal Object Kinematic History

                                  @param[in]     iObjectIndex     Index of the
                                current object
                                  @param[in]     pObjectData      Pointer to the
                                object data

                                  @return        void

                                  @pre           [none]

                                  @post          [none]

                                ****************************************************************************
                                */
static void CDHypoBicycleUpdateObjHist(ObjNumber_t iObjectIndex,
                                       const CDObjectData_t *pObjectData) {
    if (OBJ_IS_NEW(iObjectIndex)) {
        CDResetObjHistoryFloat(
            &(CDBicycleHypInternalObjHistList[iObjectIndex]));
    }
    if (!OBJ_IS_DELETED(iObjectIndex)) {
        CDPushHistoryFloat(
            pObjectData->pGenObjList->aObject[iObjectIndex].Kinematic.fDistX,
            &(CDBicycleHypInternalObjHistList[iObjectIndex].sPosX));
        CDPushHistoryFloat(
            pObjectData->pGenObjList->aObject[iObjectIndex].Kinematic.fDistXStd,
            &(CDBicycleHypInternalObjHistList[iObjectIndex].sPosXVar));
        CDPushHistoryFloat(
            pObjectData->pGenObjList->aObject[iObjectIndex].Kinematic.fDistY,
            &(CDBicycleHypInternalObjHistList[iObjectIndex].sPosY));
        CDPushHistoryFloat(
            pObjectData->pGenObjList->aObject[iObjectIndex].Kinematic.fDistYStd,
            &(CDBicycleHypInternalObjHistList[iObjectIndex].sPosYVar));

        CDTransform2DCoordHistoryFloat(
            VLCGetTrafoMatrix2DCOFForward(),
            &(CDBicycleHypInternalObjHistList[iObjectIndex].sPosX),
            &(CDBicycleHypInternalObjHistList[iObjectIndex].sPosY));
    } else {
        CDResetObjHistoryFloat(
            &(CDBicycleHypInternalObjHistList[iObjectIndex]));
    }
}

/* ***********************************************************************
  @fn            CDHypoBicycleUpdateEgoHist */ /*!

                                  @brief         Updates the Bicycle
                                Hypothesis internal Ego Kinematic History

                                  @param[in]     pEgoKin Pointer to the
                                EMPKinEgo Struct

                                  @return        void

                                  @pre           [none]

                                  @post          [none]

                                ****************************************************************************
                                */
static void CDHypoBicycleUpdateEgoHist(const EMPKinEgo_t *pEgoKin) {
    CDPushHistoryFloat(pEgoKin->fAccel, &(CDBicycleHypInternalEgoHist.sAccel));
    CDPushHistoryFloat(pEgoKin->fVel, &(CDBicycleHypInternalEgoHist.sVel));
    CDPushHistoryFloat(pEgoKin->fYawRate,
                       &(CDBicycleHypInternalEgoHist.sYawRate));
}

/* ***********************************************************************
  @fn            HistIntegrate */ /*!

                                           @brief         Integrate a discrete
                                         function by linear interpolation

                                           @param[in]     afXValues
                                           @param[in]     afYValues
                                           @param[in]     fLeftIntLimX
                                           @param[in]     fRightIntLimX
                                           @param[in]     uiLenXYVec
                                           @param[out] pOut_fHistIntegralResult

                                           @return        boolean

                                         ****************************************************************************
                                         */
static boolean HistIntegrate(const float32 afXValues[],
                             const float32 afYValues[],
                             float32 fLeftIntLimX,
                             float32 fRightIntLimX,
                             const uint8 uiLenXYVec,
                             float32 *pOut_fHistIntegralResult) {
    uint8 uiCountXYVec = 0;
    float32 fHistIntegralResult = 0;
    float32 fLeftIntLimY = 0;
    float32 fRightIntLimY = 0;
    boolean bResultOk = TRUE;

    /* Securing limits */
    if (fLeftIntLimX > fRightIntLimX) {
        AddSwapParameters(&fLeftIntLimX, &fRightIntLimX);
    }

    fLeftIntLimX = MAX(fLeftIntLimX, afXValues[0]);
    fLeftIntLimX = MIN(fLeftIntLimX, afXValues[uiLenXYVec - 1]);
    fRightIntLimX = MAX(fRightIntLimX, afXValues[0]);
    fRightIntLimX = MIN(fRightIntLimX, afXValues[uiLenXYVec - 1]);

    for (uiCountXYVec = 0; uiCountXYVec < (uiLenXYVec - 1); ++uiCountXYVec) {
        /* Calculate Y Values for left and right integration limit */
        fLeftIntLimY =
            afYValues[uiCountXYVec] +
            (((fLeftIntLimX - afXValues[uiCountXYVec]) /
              (afXValues[uiCountXYVec + 1] - afXValues[uiCountXYVec])) *
             (afYValues[uiCountXYVec + 1] - afYValues[uiCountXYVec]));
        fRightIntLimY =
            afYValues[uiCountXYVec] +
            (((fRightIntLimX - afXValues[uiCountXYVec]) /
              (afXValues[uiCountXYVec + 1] - afXValues[uiCountXYVec])) *
             (afYValues[uiCountXYVec + 1] - afYValues[uiCountXYVec]));

        /* Test if left and right integration limits are between the current two
         * sampling points */
        if ((fLeftIntLimX > afXValues[uiCountXYVec]) &&
            (fLeftIntLimX < afXValues[uiCountXYVec + 1])) {
            if ((fRightIntLimX > afXValues[uiCountXYVec]) &&
                (fRightIntLimX < afXValues[uiCountXYVec + 1])) {
                fHistIntegralResult +=
                    0.5f * (fLeftIntLimY + fRightIntLimY) *
                    (fRightIntLimX - fLeftIntLimX); /*Trapezoidal rule*/
            } else {
                fHistIntegralResult +=
                    0.5f * (fLeftIntLimY + afYValues[uiCountXYVec + 1]) *
                    (afXValues[uiCountXYVec + 1] -
                     fLeftIntLimX); /*Trapezoidal rule*/
            }
        } else if ((fRightIntLimX > afXValues[uiCountXYVec]) &&
                   (fRightIntLimX < afXValues[uiCountXYVec + 1])) {
            fHistIntegralResult +=
                0.5f * (afYValues[uiCountXYVec] + fRightIntLimY) *
                (fRightIntLimX - afXValues[uiCountXYVec]); /*Trapezoidal rule*/
        } else {
            if ((fLeftIntLimX <= afXValues[uiCountXYVec]) &&
                (fRightIntLimX >= afXValues[uiCountXYVec + 1])) {
                fHistIntegralResult +=
                    0.5f *
                    (afYValues[uiCountXYVec] + afYValues[uiCountXYVec + 1]) *
                    (afXValues[uiCountXYVec + 1] -
                     afXValues[uiCountXYVec]); /*Trapezoidal rule*/
            }
        }
    }

    *pOut_fHistIntegralResult = fHistIntegralResult;

    return bResultOk;
}

/*************************************************************************************************************************
  Functionname:    AddSwapParameters */ /*!

                                                                                     @brief           Swap two parameters by adding/subtracting them

                                                                                     @return          void

                                                                                     @param[in,out]   *fFrstValToSwap :
                                                                                     @param[in,out]   *fScndValToSwap :

                                                                                   *************************************************************************************************************************/
static void AddSwapParameters(float32 *fFrstValToSwap,
                              float32 *fScndValToSwap) {
    /* Swap two parameters by adding / subtracting them */
    if (fFrstValToSwap != fScndValToSwap) {
        (*fFrstValToSwap) = (*fFrstValToSwap) + (*fScndValToSwap);
        (*fScndValToSwap) = (*fFrstValToSwap) - (*fScndValToSwap);
        (*fFrstValToSwap) = (*fFrstValToSwap) - (*fScndValToSwap);
    }
}

/* ***********************************************************************
  @fn            CDHypoBicycleCollMain */ /*!

                                       @brief         Handles the Bicycle
                                     Collision Hypothesis

                                       @param[in]     iObjectIndex The index of
                                     the object
                                       @param[in]     bObjFilterMatched If TRUE
                                     out object filter matched so hypothesis
                                     shall be calculated. If FALSE reset history
                                     (if exists)
                                       @param[in]     pInputData Pointer to CD
                                     input data
                                       @param[in]     pInternalStatus Pointer to
                                     CD internal status data
                                       @param[in]     pExternalFunctions Pointer
                                     to external functions

                                       @return        void

                                       @pre           [none]

                                       @post          [none]

                                     ****************************************************************************
                                     */
void CDHypoBicycleCollMain(ObjNumber_t iObjectIndex,
                           boolean bObjFilterMatched,
                           const CDInputData_t *pInputData,
                           CDInternalStatus_t *pInternalStatus,
                           const CDExternalFunctions_t *pExternalFunctions) {
    CDIntHypothesis_t Hypothesis;

    CDInternalObject_t *const pLocalObject =
        &(*pInternalStatus->rgObjInternal)[iObjectIndex];

    _PARAM_UNUSED(pExternalFunctions);
    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypPresel),
                     (uint32)CDHypothesisType_CyclColl);
    CD_CLEAR_HYP_BIT(&(pLocalObject->bitHypActive),
                     (uint32)CDHypothesisType_CyclColl);

    /* Update internal object history (only needed for PSC calculation) */
    CDHypoBicycleUpdateObjHist(iObjectIndex, (pInputData->pObjectData));
    CDHypoBicycleUpdateEgoHist(&(pInternalStatus->sKinEgo));

    /* Default Hypothesis Prob Calculation calling */
    if (bObjFilterMatched != FALSE) {
        VLCSEN_SERVICE_ADD_EVENT(
            e_RTA_EVT_AlgoStart, VLCSEN_RTA_CD_HYP_BICYCLECOLL_SINGLE,
            (uint8)(iObjectIndex)); /* start profiling for Hypothesis */
        if (CDHypoBicycleCollObjectFilter(iObjectIndex, pInputData->pObjectData,
                                          pLocalObject) != FALSE) {
            /* Default Hypothesis handling */
            CD_SET_HYP_BIT(&(pLocalObject->bitHypPresel),
                           (uint32)CDHypothesisType_CyclColl);

            Hypothesis.fRelevance = 0;
            Hypothesis.fHypothesisProbability = 0;
            Hypothesis.fHypothesisLifetime = 0;

            /* link object to hypothesis */
            Hypothesis.iObjectRef = iObjectIndex;

            /* store object class */
            Hypothesis.eObjectClass =
                CDGetPointer_Attributes(pInputData->pObjectData, iObjectIndex)
                    ->eClassification;

            /* set hypothesis type */
            Hypothesis.eType = CDHypothesisType_CyclColl;

            CDHypoBicycleCollCalculateProb(iObjectIndex, &Hypothesis,
                                           pInputData, pInternalStatus,
                                           pLocalObject);

            /* store hypothesis (if relevant) */
            if (Hypothesis.fHypothesisProbability > 0.2f) {
                CD_SET_HYP_BIT(&(pLocalObject->bitHypActive),
                               (uint32)CDHypothesisType_CyclColl);
                CDHypothesesSelection(&Hypothesis, pInputData->pObjectData,
                                      pInternalStatus);
                pLocalObject->HypothesisHist.BicycleColl = 1u;
            } else {
                pLocalObject->HypothesisHist.BicycleColl = 0u;
            }
        }
        VLCSEN_SERVICE_ADD_EVENT(e_RTA_EVT_AlgoEnd,
                                 VLCSEN_RTA_CD_HYP_BICYCLECOLL_SINGLE,
                                 (uint8)(iObjectIndex));
    }
}

/* ***********************************************************************
  @fn            iterateAllPossVelo

  @brief         iterate All Possible Velocity
  @param[in]     uiNumberOfValidDataPoints
  @param[in]     fStepSizePossibleVelocity_mps
  @param[in]     afDistToTrajVarHistory[]
  @param[in]     afDistToTrajHistory
  @param[in]     afPossibleVelocity_mps
  @param[in,out] afTimeVec_s[]
  @param[out]    afVelocityProbabilities[]

  @return        void

  @pre           [none]

  @post          [none]

**************************************************************************** */
static void iterateAllPossVelo(const uint8 uiNumberOfValidDataPoints,
                               const float32 fStepSizePossibleVelocity_mps,
                               const float32 afDistToTrajVarHistory[],
                               const float32 afDistToTrajHistory[],
                               const float32 afPossibleVelocity_mps[],
                               float32 afTimeVec_s[],
                               float32 afVelocityProbabilities[]) {
    for (int i = 0; i < CD_BICYCLE_COLL_BICYCLE_VELO_STEPS_N; ++i) {
        float32 afWeights[CD_BICYCLE_COLL_OBSERVE_Y_DIST_N];
        float32 fWeightSum = 0;
        float32 fMaxWeight = 0;
        float32 fBWeighted = 0; /* Best linear fitted function:
                                   y=bicycle_velo_mps*t+fBWeighted */
        float32 fError = 0;
        uint8 j;

        /* Calculate Weights for y distance history */
        for (j = 0; j < CD_BICYCLE_COLL_OBSERVE_Y_DIST_N; ++j) {
            if (((CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - j) - 1) <
                uiNumberOfValidDataPoints) {
                float32 fWeightStdPart =
                    1.0f / (SQRT(ABS(afDistToTrajVarHistory[(
                                (CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - j) - 1)])) +
                            BICYCLE_VELO_WEIGHT_STD_B);
                float32 fWeightLinearPart =
                    (((BICYCLE_VELO_WEIGHT_YOUNGEST -
                       BICYCLE_VELO_WEIGHT_OLDEST) /
                      (CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - 1)) *
                     j) +
                    BICYCLE_VELO_WEIGHT_OLDEST;
                afWeights[j] = fWeightStdPart * fWeightLinearPart;
                fWeightSum += afWeights[j];
                fMaxWeight = MAX_FLOAT(fMaxWeight, afWeights[j]);
            } else {
                afWeights[j] = 0;
            }
            /* Create the time vector (last Y_DIST_N cycle times) */
            afTimeVec_s[j] =
                -((((float32)CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - (float32)j) -
                   1.0f) *
                  VLC_CYCLE_TIME);
        }

        /* Fit linear function */
        for (j = 0; j < CD_BICYCLE_COLL_OBSERVE_Y_DIST_N; ++j) {
            if (((CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - j) - 1) <
                uiNumberOfValidDataPoints) {
                float32 t = afTimeVec_s[j];
                float32 y = afDistToTrajHistory[(
                    (CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - j) - 1)];
                float32 b = y - (t * afPossibleVelocity_mps[i]);
                fBWeighted += (b * afWeights[j]) / fWeightSum;
            }
        }

        /* Calculate Error */
        for (j = 0; j < CD_BICYCLE_COLL_OBSERVE_Y_DIST_N; ++j) {
            if (((CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - j) - 1) <
                uiNumberOfValidDataPoints) {
                /*
                Allowed Error:
                - Discretization Compensation (linear function starting in
                fBWeighted, with gradient
                Bicycle_velo_mps+fStepSizePossibleVelocity_mps/2)
                - Constant offset
                */
                float32 t = afTimeVec_s[j];
                float32 fAllowedErrorDiscretization =
                    ABS((t * fStepSizePossibleVelocity_mps) /
                        2.0f); /*physical constant*/
                float32 fAllowedError = fAllowedErrorDiscretization +
                                        BICYCLE_VELO_ALLOWED_CONST_ERROR_M;
                float32 fExpectedYDist =
                    (afPossibleVelocity_mps[i] * t) + fBWeighted;
                float32 fErrorAtThisPoint = MAX_FLOAT(
                    0, (ABS(afDistToTrajHistory[(
                                (CD_BICYCLE_COLL_OBSERVE_Y_DIST_N - j) - 1)] -
                            fExpectedYDist) -
                        fAllowedError));
                /* Add weighted error */
                fError += (fErrorAtThisPoint * afWeights[j]) /
                          (CD_BICYCLE_COLL_OBSERVE_Y_DIST_N * fMaxWeight);
            }
        }
        /* Map Error to Probability */
        afVelocityProbabilities[i] =
            BICYCLE_VELO_MAP_ERROR_PROB_CONST_A /
            (BICYCLE_VELO_MAP_ERROR_PROB_CONST_B + (fError * fError * fError));
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */