/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "si.h"
#include "cp_si.h"
#include "si_par.h"
//#include "vlc_par.h"
#include "stddef.h"
#include "TM_Global_Types.h"
/*****************************************************************************
  MODULGLOBALE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULGLOBALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE MAKROS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE TYPEN
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE KONSTANTEN
*****************************************************************************/
#define SI_MIN_DIFF_LANEBORDER_LEFT_RIGHT                      \
    (0.1f) /*!< Minimal difference between fLatTrackLimitL and \
              fLatTrackLimitR */

#define INIT_BRACKET_POSITION_ZERO (0.f)

/*****************************************************************************
  MODULLOKALE TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  MODULLOKALE VARIABLEN
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

static void SIInitBrackets(SIBracketOutput_t* const pBrackets,
                           const float32 InitValue);
static void SIExtendBrackets(SIBracketOutput_t* const pBrackets,
                             const SIBracketOutput_t* const pExtension);
static void SIRestrictBrackets(SIBracketOutput_t* const pBrackets,
                               const SIBracketOutput_t* const pRestriction);
static void SIAddBracketPositions(
    SIBracketOutput_t* const pBrackets,
    const SIBracketOutput_t* const pAdditiveBrackets);

static void SIExtendAndRestrictBrackets(
    const RelTraObjInput_t* const pObjectProperties,
    SICriteriaMatrix_t* const pBracketFuncResults);
static void SIAdjustAndCorrectFinalBrackets(
    const RelTraObjInput_t* const pObjectProperties,
    SICriteriaMatrix_t* const pBracketFuncResults);

static void SIUpdatePreliminaryLaneAssociation(
    const RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    const SIBracketOutput_t* const pBrackets);

/*! Criterias which are equal for all Objects */
static void SISetCriteriaMatrixAllObj(
    SICriteriaMatrix_t* pCriteriaMatrix,
    SICriteriaMatrixAllObj_t const* pCriteriaMatrixAllObj,
    RelTraObjInput_t const* pObj,
    AssTraEnvironment_t const* pEnvironment);

/*************************************************************************************************************************
  Functionname:    SIEvaluateBracketFunctions */
void SIEvaluateBracketFunctions(
    const RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    const RelTraCurve_t* const pTrajectory,
    AssTraEnvironment_t* const pEnvironment,
    SICriteriaMatrix_t* const pBracketFuncResults,
    SIBracketFuncEnable_t* const pBracketFuncEnableFlags,
    const SICriteriaMatrixAllObj_t* const pCritMatrixAllObj) {
    /* Base brackets, Seek and Track case */
    /* This needs to be evaluated first, since the resulting corridor base
     * brackets are used within some of the bracket functions following below */
    SICalculateCorridorBaseBrackets(pObjectProperties, pTrajectory,
                                    &pBracketFuncResults->BaseCorridor,
                                    pBracketFuncEnableFlags);

    /* Preliminary geometric lane association */
    SIUpdatePreliminaryLaneAssociation(pObjectProperties, pObjectVariables,
                                       &pBracketFuncResults->BaseCorridor);

    /* temporarily store BaseBracketPositions in
     * pObjectVariables->dAblage_SpurGrenze */
    pObjectVariables->fLatTrackLimitL =
        pBracketFuncResults->BaseCorridor.BracketPositionLeft;
    pObjectVariables->fLatTrackLimitR =
        pBracketFuncResults->BaseCorridor.BracketPositionRight;

    /* Evaluate (execute) bracket functions. Resulting bracket positions for
       each extension and restriction are stored
       in pBracketFuncResults */
    if ((pObjectProperties->ucDynamicProperty == CR_OBJECT_PROPERTY_MOVING) ||
        ((pObjectProperties->ucDynamicProperty ==
          CR_OBJECT_PROPERTY_STATIONARY) &&
         (OBJ_IS_MOVING_TO_STATIONARY(pObjectProperties->iObjNr)))) {
        SIExecuteRadarRoadBracketFunctions(
            pObjectProperties, pObjectVariables, pTrajectory, pEnvironment,
            pBracketFuncResults, pBracketFuncEnableFlags);

    } /* ONLY for Moving or Stopped objects */

    SIExecuteAdditiveBracketFunctions(
        pObjectProperties, pObjectVariables, pTrajectory, pEnvironment,
        pBracketFuncResults, pBracketFuncEnableFlags);

    /* Execute scene related bracket positions */
    SI_v_ExecuteSceneBracketFuncstions(pObjectProperties, pEnvironment,
                                       pBracketFuncEnableFlags,
                                       pBracketFuncResults);

    /* General corridor properties are applied to local pBracketFuncResults
     * (adjusted to specific object) */
    SISetCriteriaMatrixAllObj(pBracketFuncResults, pCritMatrixAllObj,
                              pObjectProperties, pEnvironment);
}

/*************************************************************************************************************************
  Functionname:    SIDetermineFinalBracketPositions */
void SIDetermineFinalBracketPositions(
    const RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    SICriteriaMatrix_t* const pBracketFuncResults) {
    /* Determination of final bracket positions taking all (active) bracket
     * functions into account */
    SIExtendAndRestrictBrackets(pObjectProperties, pBracketFuncResults);

    /* Adjustment and correction of the final bracket functions for e.g. camera
     * input or EOL test */
    SIAdjustAndCorrectFinalBrackets(pObjectProperties, pBracketFuncResults);

    /* Store FinalBracketPositions in pObjectVariables->dAblage_SpurGrenze */
    pObjectVariables->fLatTrackLimitL =
        pBracketFuncResults->FinalBracketPositions.BracketPositionLeft;
    pObjectVariables->fLatTrackLimitR =
        pBracketFuncResults->FinalBracketPositions.BracketPositionRight;

    /* Update preliminary lane association for the current object.
       The lane association is now based on the resulting final brackets,
       contrary to relying on base brackets only. */
    SIUpdatePreliminaryLaneAssociation(
        pObjectProperties, pObjectVariables,
        &pBracketFuncResults->FinalBracketPositions);
}

/*************************************************************************************************************************
  Functionname:    SIExtendAndRestrictBrackets */
static void SIExtendAndRestrictBrackets(
    const RelTraObjInput_t* const pObjectProperties,
    SICriteriaMatrix_t* const pBracketFuncResults) {
    SIBracketOutput_t intermediateBrackets;
    SIBracketOutput_t AdditiveExtension;
    SIBracketOutput_t AdditiveRestriction;

    /* Obtain corridor base brackets. These will be used in case no extensions
     * or restrictions take effect. */
    /* This is especially the case for stationary objects. */
    intermediateBrackets = pBracketFuncResults->BaseCorridor;

    if ((pObjectProperties->ucDynamicProperty == CR_OBJECT_PROPERTY_MOVING) ||
        (pObjectProperties->ucDynamicProperty ==
         CR_OBJECT_PROPERTY_STATIONARY) ||
        (OBJ_IS_MOVING_TO_STATIONARY(pObjectProperties->iObjNr))) {
        /* Superimpose the results of additive bracket functions to the base
           corridor brackets.
           The largest extension among all additive extensions is selected first
           and then added to the corridor base brackets subsequently.
           Additive restrictions are handled accordingly. */
        SIInitBrackets(&AdditiveExtension, INIT_BRACKET_POSITION_ZERO);
        SIInitBrackets(&AdditiveRestriction, INIT_BRACKET_POSITION_ZERO);

        /***************** RELATIVE EXTENSIONS **************************/
        SIExtendBrackets(&AdditiveExtension,
                         &pBracketFuncResults->AddExtensionRelevantObject);
        SIExtendBrackets(&AdditiveExtension,
                         &pBracketFuncResults->AddExtensionHighspeedApproach);
        SIExtendBrackets(&AdditiveExtension,
                         &pBracketFuncResults->AddExtensionHighTunnelProb);
        SIExtendBrackets(&AdditiveExtension,
                         &pBracketFuncResults->AddExtensionObjectFastCutIn);
        SIAddBracketPositions(&intermediateBrackets, &AdditiveExtension);

        /***************** RELATIVE DISPLACEMENT **************************/
        /* Although this is defined as a restriction, it basically shifts the
         * corridor to one side by a predefined value in
         * SI_LB_STATOBJ_NEXTLANE_SHIFT */

        /***************** RELATIVE REDUCTIONS **************************/
        SIRestrictBrackets(
            &AdditiveRestriction,
            &pBracketFuncResults->AddRestrictionCurveOuterBorder);
        SIAddBracketPositions(&intermediateBrackets, &AdditiveRestriction);

        /* Extend and restrict bracket positions according to the results
         * obtained from bracket functions */
        /* The priorities of extensions and restrictions are handled by the
         * order in which they are applied. */
        /***************************************************************************************************
          The order of the functions is IMPORTANT. Take care when adding or
        changing the order of functions
        ****************************************************************************************************/

        /***************** DEFAULT EXTENSIONS **************************/

        SIExtendBrackets(&intermediateBrackets,
                         &pBracketFuncResults->ExtensionCurveInnerBorder);
        SIExtendBrackets(&intermediateBrackets,
                         &pBracketFuncResults->ExtensionRoadBorderCI);
        SIExtendBrackets(&intermediateBrackets,
                         &pBracketFuncResults->ExtensionFollowObjectIntoCurve);
        SIExtendBrackets(&intermediateBrackets,
                         &pBracketFuncResults->ExtensionRoadBorder);
    }
    /* Always execute lowspeed fused border extension, also for stationary
     * objects */
    SIExtendBrackets(&intermediateBrackets,
                     &pBracketFuncResults->AddExtensionLowSpeedFusedBrd);
    if ((pObjectProperties->ucDynamicProperty == CR_OBJECT_PROPERTY_MOVING) ||
        ((pObjectProperties->ucDynamicProperty ==
          CR_OBJECT_PROPERTY_STATIONARY) &&
         (OBJ_IS_MOVING_TO_STATIONARY(pObjectProperties->iObjNr)))) {
        /***************** DEFAULT RESTRICTIONS ************************/
        SIRestrictBrackets(&intermediateBrackets,
                           &pBracketFuncResults->RestrictionAnalogRoadBorder);
        SIRestrictBrackets(
            &intermediateBrackets,
            &pBracketFuncResults->RestrictionNeighbourhoodRelObj);
        SIRestrictBrackets(
            &intermediateBrackets,
            &pBracketFuncResults->RestrictionTargetOutsideBrackets);
        SIRestrictBrackets(&intermediateBrackets,
                           &pBracketFuncResults->RestrictionCityNearRange);
    }

    /* Store final brackets */
    pBracketFuncResults->FinalBracketPositions = intermediateBrackets;
}

/*************************************************************************************************************************
  Functionname:    SIAdjustAndCorrectFinalBrackets */
static void SIAdjustAndCorrectFinalBrackets(
    const RelTraObjInput_t* const pObjectProperties,
    SICriteriaMatrix_t* const pBracketFuncResults) {
    SIBracketOutput_t* const pFinalBrackets =
        &pBracketFuncResults->FinalBracketPositions;

    /* Moving the center of the Trace Brackets (based on camera information of
     * ego-position in lane) */
    SIRelTraSetTrackWidthScale(pObjectProperties->fRefCourseDistY,
                               &pFinalBrackets->BracketPositionLeft,
                               &pFinalBrackets->BracketPositionRight,
                               &pBracketFuncResults->RatioEgoPositionInLaneCam);

    /* Ensure that the left bracket remains located to the left of the right
     * bracket */
    if (pFinalBrackets->BracketPositionLeft <
        pFinalBrackets->BracketPositionRight) {
        pFinalBrackets->BracketPositionLeft =
            pFinalBrackets->BracketPositionRight +
            SI_MIN_DIFF_LANEBORDER_LEFT_RIGHT;
    }
}

/*************************************************************************************************************************
  Functionname:    SIInitBrackets */
static void SIInitBrackets(SIBracketOutput_t* const pBrackets,
                           const float32 InitValue) {
    pBrackets->BracketPositionLeft = -InitValue;
    pBrackets->BracketPositionRight = InitValue;
}

/*************************************************************************************************************************
  Functionname:    SIUpdatePreliminaryLaneAssociation */
static void SIUpdatePreliminaryLaneAssociation(
    const RelTraObjInput_t* const pObjectProperties,
    RelTraObjOutput_t* const pObjectVariables,
    const SIBracketOutput_t* const pBrackets) {
    /*! Set default */
    pObjectVariables->iObjektSpur_Zyklus = ASSOC_LANE_EGO;

    if (pObjectProperties->fDistY > pBrackets->BracketPositionLeft) {
        pObjectVariables->iObjektSpur_Zyklus = ASSOC_LANE_LEFT;
    } else if (pObjectProperties->fDistY < pBrackets->BracketPositionRight) {
        pObjectVariables->iObjektSpur_Zyklus = ASSOC_LANE_RIGHT;
    } else {
    }
}

/*************************************************************************************************************************
  Functionname:    SIRestrictBrackets */
static void SIRestrictBrackets(SIBracketOutput_t* const pBrackets,
                               const SIBracketOutput_t* const pRestriction) {
    pBrackets->BracketPositionLeft = MIN_FLOAT(
        pBrackets->BracketPositionLeft, pRestriction->BracketPositionLeft);
    pBrackets->BracketPositionRight = MAX_FLOAT(
        pBrackets->BracketPositionRight, pRestriction->BracketPositionRight);
}

/*************************************************************************************************************************
  Functionname:    SIExtendBrackets */
static void SIExtendBrackets(SIBracketOutput_t* const pBrackets,
                             const SIBracketOutput_t* const pExtension) {
    pBrackets->BracketPositionLeft = MAX_FLOAT(pBrackets->BracketPositionLeft,
                                               pExtension->BracketPositionLeft);
    pBrackets->BracketPositionRight = MIN_FLOAT(
        pBrackets->BracketPositionRight, pExtension->BracketPositionRight);
}

/*************************************************************************************************************************
  Functionname:    SIAddBracketPositions */
static void SIAddBracketPositions(
    SIBracketOutput_t* const pBrackets,
    const SIBracketOutput_t* const pAdditiveBrackets) {
    pBrackets->BracketPositionLeft =
        pBrackets->BracketPositionLeft + pAdditiveBrackets->BracketPositionLeft;
    pBrackets->BracketPositionRight = pBrackets->BracketPositionRight +
                                      pAdditiveBrackets->BracketPositionRight;
}

/*************************************************************************************************************************
  Functionname:    SIRelTraCheckCriteriaAllObj */
void SIRelTraCheckCriteriaAllObj(
    SICriteriaMatrixAllObj_t* pCriteriaMatrixAllObj) {
    /* Adaption of trace brackets based on inlane position of ego-vehicle
      (camera information)
      -> active only if visibility of camera is higher than 7m
      (SI_MIN_CAM_COURSE_DIST) */

    if ((SITrajectoryData.State.CamLaneFusion) &&
        (!SITrajectoryData.State.EgoCourseOnly)) {
        /* Fusion with detected camera lane active, i.e. trace bracket adaption
        only if CamLaneFusion is off (reason: combination of both changes leads
        to a worse performance).
        As a result, CamLaneFusion is active mainly during city traffic and
        trace bracket adaption is active mainly on highways */
    } else {
        SIRelTraRatioEgoPositionInLaneCam(
            &(pCriteriaMatrixAllObj->RatioEgoPositionInLaneCam));
    }
}

/*************************************************************************************************************************
  Functionname:    SIInitCriteriaMatrixAllObj */
void SIInitCriteriaMatrixAllObj(
    SICriteriaMatrixAllObj_t* pCriteriaMatrixAllObj) {
    /*! Initialization: Adaption trace brackets based on inlane position of
     * ego-vehicle (camera information) */

    pCriteriaMatrixAllObj->RatioEgoPositionInLaneCam.fScaleBracketLeft =
        INITVALUE_BRACKETPOSITION;
    pCriteriaMatrixAllObj->RatioEgoPositionInLaneCam.fScaleBracketRight =
        INITVALUE_BRACKETPOSITION;
}

/*************************************************************************************************************************
  Functionname:    SISetCriteriaMatrixAllObj */
static void SISetCriteriaMatrixAllObj(
    SICriteriaMatrix_t* pCriteriaMatrix,
    SICriteriaMatrixAllObj_t const* pCriteriaMatrixAllObj,
    RelTraObjInput_t const* pObj,
    AssTraEnvironment_t const* pEnvironment) {
    /*! Adaption trace brackets based on inlane position of ego-vehicle (camera
     * information) */

    /*! Test if the trace bracket adaption should be performed for the specific
     * object */
    if (SIFindObjRelevantForTraceBracketAdaptPosInLaneCam(
            pObj, pEnvironment,
            &pCriteriaMatrixAllObj->RatioEgoPositionInLaneCam)) {
        /*! Adapt the factors for the trace bracket adaption for each specific
         * object */
        pCriteriaMatrix->RatioEgoPositionInLaneCam =
            SIAdaptRatioEgoPosInLaneCamToObj(
                pObj, pCriteriaMatrixAllObj->RatioEgoPositionInLaneCam,
                pEnvironment);
    } else {
        /* no adaption*/
    }
}

/* ************************************************************************* */
/*   Copyright                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */