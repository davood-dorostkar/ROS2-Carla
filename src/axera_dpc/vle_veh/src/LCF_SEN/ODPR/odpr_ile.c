/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "odpr_ile_ext.h"
#include "tue_common_libs.h"
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
STATIc uint8
    ILE_fPreCycIterLeLnObjID_nu;  // Left lane object ID in previous interation
STATIc uint8
    ILE_fPreCycIterEgoLnObjID_nu;  // Ego lane object ID in previous interation
STATIc uint8
    ILE_fPreCycIterRiLnObjID_nu;  // Right lane object ID in previous interation
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

/*****************************************************************************
  Functionname:    ODPR_ILE_Reset                                        */ /*!

              @brief           Reset function of ILE

              @description     All global variables related to ILE are reset in
            this
            function
                               when ILE executes for the first time, or system
            exception
            needs
                               to be reset

              @param[in]       none

              @return          none
            *****************************************************************************/
void ODPR_ILE_Reset(void) {
    ILE_fPreCycIterLeLnObjID_nu = 0xFF;
    ILE_fPreCycIterEgoLnObjID_nu = 0xFF;
    ILE_fPreCycIterRiLnObjID_nu = 0xFF;
}

/*****************************************************************************
  Functionname:    ILEDetermineShPt */ /*!

                                         @brief           Determine shape points

                                         @description     Determine shape
                                         points, including validity check,
                                         obejct
                                         coordinate
                                                    transformation and
                                         shapepoints absolute position
                                         calculation.
                                         This
                                                    function is corresponding to
                                         DetermineShapePointsInCameraCoordinateSystem
                                                    module in MBD.

                                         @param[in]       pGenObjList      Side
                                         radar object list
                                         @param[in,out]   pSSRObjInfoList  side
                                         radar object list information after
                                         modified

                                         @return          none

                                         @uml
                                         @startuml
                                         start
                                         partition ValidityChk {
                                         if (At least one of ShapePoint state is
                                         INVALID) then (yes)
                                         :Object shape point validity \n check
                                         result is FALSE;
                                         else (no)
                                         :Object shape point validity \n check
                                         result is TRUE;
                                         endif
                                         note:Determine object \n shape points
                                         validity;
                                         }
                                         partition OverhangComp {
                                         if (Object is relative to Radar
                                         coordination) then (yes)
                                         :Longitudinal position \n overhang
                                         compensation;
                                         else (no)
                                         :Output longitudinal \n position
                                         directly;
                                         endif
                                         note:Translate object \n reference
                                                    // float32 fObjVabs; partition CoordinateTrans {
                                         if (Shape point PosX) then (yes)
                                         :Output (Shape point PosX + \n object
                                         posX after compensation);
                                         else (no)
                                         :Output (Shape point PosY + \n object
                                         posY);
                                         endif
                                         note:Calculate absolute and translated
                                         \n object shape points;
                                         }
                                         stop
                                         @enduml
                                         *****************************************************************************/
STATIc void ILEDetermineShPt(const ODPRInSSRObjList_t* pGenObjList,
                             const ODPRParam_t* pParam,
                             ILESSRObjInfoList_t* pSSRObjInfoList,
                             ODPRILEDebug_t* pILEDebug) {
    boolean bTempObjShPtStateChk_bool;
    float32 fTempObjPosXOffset_met;
    float32 fTempObjPosXTranslated_met;

    /* Loop over all SSR objects */
    for (uint8 i = 0; i < TUE_CML_Min(pGenObjList->uiNumOfObject,
                                      ILE_INPUT_RADAR_OBJECT_NUM);
         i++) {
        /* Determine object shape points validity */
        bTempObjShPtStateChk_bool =
            ((pGenObjList->ObjectArray[i].uiShapePoint0State_nu ==
              (uint8)INVALID) ||
             (pGenObjList->ObjectArray[i].uiShapePoint1State_nu ==
              (uint8)INVALID) ||
             (pGenObjList->ObjectArray[i].uiShapePoint2State_nu ==
              (uint8)INVALID) ||
             (pGenObjList->ObjectArray[i].uiShapePoint3State_nu ==
              (uint8)INVALID));
        pSSRObjInfoList->sSSRObjInfo[i].bObjShPtValidChk_bool =
            !bTempObjShPtStateChk_bool;

        /* Translate object reference point */
        fTempObjPosXOffset_met =
            ILE_OBJ_REL_RO_RADAR_BOOL ? pParam->fVehOverhangFront_met : 0.f;
        fTempObjPosXTranslated_met =
            pGenObjList->ObjectArray[i].fPosX_met + fTempObjPosXOffset_met;

        /* Calculate absolute and translated object shape points */
        pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[0] =
            pGenObjList->ObjectArray[i].fShapePoint0PosX_met +
            fTempObjPosXTranslated_met;
        pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[0] =
            pGenObjList->ObjectArray[i].fShapePoint0PosY_met +
            pGenObjList->ObjectArray[i].fPosY_met;
        pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[1] =
            pGenObjList->ObjectArray[i].fShapePoint1PosX_met +
            fTempObjPosXTranslated_met;
        pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[1] =
            pGenObjList->ObjectArray[i].fShapePoint1PosY_met +
            pGenObjList->ObjectArray[i].fPosY_met;
        pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[2] =
            pGenObjList->ObjectArray[i].fShapePoint2PosX_met +
            fTempObjPosXTranslated_met;
        pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[2] =
            pGenObjList->ObjectArray[i].fShapePoint2PosY_met +
            pGenObjList->ObjectArray[i].fPosY_met;
        pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[3] =
            pGenObjList->ObjectArray[i].fShapePoint3PosX_met +
            fTempObjPosXTranslated_met;
        pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[3] =
            pGenObjList->ObjectArray[i].fShapePoint3PosY_met +
            pGenObjList->ObjectArray[i].fPosY_met;
    }

    /* Debug output */
    for (uint8 i = 0;
         TUE_CML_Min(pGenObjList->uiNumOfObject, ILE_INPUT_RADAR_OBJECT_NUM);
         i++) {
        pILEDebug->bObjShPtValidChk_bool[i] =
            pSSRObjInfoList->sSSRObjInfo[i].bObjShPtValidChk_bool;
    }
}

STATIc void ILEEvalLaneClothoid(ODPRInABPRLaneInfo_t* pLaneAfterBuild,
                                ILESSRObjInfoList_t* pSSRObjInfoList,
                                uint8 i,
                                uint8 uiShPtXInLane_nu[4]) {
    float32 fShPtXLeftLanePosY_met;
    float32 fShPtXRightLanePosY_met;
    for (uint8 j = 0; j < 4; j++) {
        fShPtXLeftLanePosY_met = ILEEvalLaneBoundary(
            pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[j],
            pLaneAfterBuild->fLeftLaneClothoidPosY0_met,
            pLaneAfterBuild->fLeftLaneClothoidHeading_rad,
            pLaneAfterBuild->fLeftLaneClothoidCurve_1pm,
            pLaneAfterBuild->fLeftLaneClothoidCurveDer_1pm2);

        if (pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[j] >
            (fShPtXLeftLanePosY_met + ILE_DEFAULT_LANE_WIDTH_MET)) {
            uiShPtXInLane_nu[j] = (uint8)LANE_OUTSIDE_LEFT;
        } else if (fShPtXLeftLanePosY_met <
                   pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[j]) {
            uiShPtXInLane_nu[j] = (uint8)LANE_LEFT;
        } else {
            fShPtXRightLanePosY_met = ILEEvalLaneBoundary(
                pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[j],
                pLaneAfterBuild->fRightLaneClothoidPosY0_met,
                pLaneAfterBuild->fRightLaneClothoidHeading_rad,
                pLaneAfterBuild->fRightLaneClothoidCurve_1pm,
                pLaneAfterBuild->fRightLaneClothoidCurveDer_1pm2);

            if (pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[j] >
                fShPtXRightLanePosY_met) {
                uiShPtXInLane_nu[j] = (uint8)LANE_EGO;
            } else if ((fShPtXRightLanePosY_met - ILE_DEFAULT_LANE_WIDTH_MET) >
                       pSSRObjInfoList->sSSRObjInfo[i]
                           .fShPtPosYTranslated_met[j]) {
                uiShPtXInLane_nu[j] = (uint8)LANE_RIGHT;
            } else {
                uiShPtXInLane_nu[j] = (uint8)LANE_OUTSIDE_RIGHT;
            }
        }
    }
}

/*****************************************************************************
  Functionname:    ILEObjInLaneEval */ /*!

                                         @brief           Object in lane
                                         evaluation

                                         @description     Object in lane
                                         evaluation checks whether the side
                                         radar object
                                         is
                                                    is the ego, left or right
                                         lane and calculates the probability
                                         that
                                                    object in corresponding
                                         lane. This function is corresponding to
                                                    ObjectInLaneEvaluation
                                         module in MBD.

                                         @param[in]       pABPRLaneData    Left
                                         and right lanes information
                                         @param[in]       pGenObjList      Side
                                         radar object list
                                         @param[in,out]   pLaneAfterBuild  Left
                                         or right lane information after
                                         building
                                         when
                                                                     one of lane
                                         is invalid
                                         @param[in,out]   pSSRObjInfoList  side
                                         radar object list information after
                                         modified

                                         @return          none
                                         @uml
                                         @startuml
                                         start
                                         partition LaneValidChk {
                                         :LeftLaneInvalidCheck and
                                         RightLaneInvalidCheck;
                                         if (At least one of lane is valid) then
                                         (yes)
                                         if (both lanes are valid) then (yes)
                                             :Lane is valid;
                                         else (no)
                                             :Struct the invalid lane with the
                                         valid one;
                                         endif
                                         :Perform ObjInLaneAssociation;
                                         else (no)
                                         :Lane is invalid;\nOutput
                                         ObjInLaneAssociation\ndefault invalid
                                         value;
                                         endif
                                         note:Lane validity check
                                         }
                                         partition ShPtInLaneEval {
                                         :Calculate LeftLanePos Y based on
                                         ShPtPos X and left lane;
                                         if (ShPtPos Y > (LeftLanePos Y +
                                         \nILE_DEFAULT_LANE_WIDTH_MET)) then
                                         (yes)
                                         :Shape point is located in
                                         \nLANE_OUTSIDE_LEFT;
                                         elseif (ShPtPos Y > LeftLanePos Y) then
                                         (yes)
                                         :Shape point is located in \nLANE_LEFT;
                                         else (no)
                                         :Calculate RightLanePos Y based on
                                         ShPtPos X and right lane;
                                         if (ShPtPos Y > RightLanePos Y) then
                                         (yes)
                                             :Shape point is located in
                                         \nLANE_EGO;
                                         elseif (ShPtPos Y < (RightLanePos Y -
                                         \nILE_DEFAULT_LANE_WIDTH_MET)) then
                                         (yes)
                                             :Shape point is located in
                                         \nLANE_RIGHT;
                                         else (no)
                                             :Shape point is located in
                                         \nLANE_OUTSIDE_RIGHT;
                                         endif
                                         endif
                                         note:Shape point in lane evaluation
                                         }
                                         partition ObjectInLaneEval {
                                         if (All ShPts in the same single lane)
                                         then (yes)
                                         :Object in this lane;
                                         else (no)
                                         if (One ShPt in LANE_OUTSIDE_LEFT) then
                                         (yes)
                                             :Object on \nLeft adjacent lane;
                                         elseif (One ShPt in LANE_LEFT) then
                                         (yes)
                                             :Object on \nLeft ego lane;
                                         elseif (One ShPt in LANE_EGO) then
                                         (yes)
                                             :Object on \nRight ego lane;
                                         else (no)
                                             :Object on \nRight adjacent lane;
                                         endif
                                         endif
                                         note:Object in lane evaluation
                                         }
                                         stop
                                         @enduml
                                         *****************************************************************************/
STATIc void ILEObjInLaneEval(const ODPRInABPRLaneInfo_t* pABPRLaneData,
                             const ODPRInSSRObjList_t* pGenObjList,
                             ODPRInABPRLaneInfo_t* pLaneAfterBuild,
                             ILESSRObjInfoList_t* pSSRObjInfoList,
                             ODPRILEDebug_t* pILEDebug) {
    boolean bTempLeftLaneValid_bool;
    boolean bTempRightLaneValid_bool;
    boolean bTempShPtAndLaneValid_bool;
    boolean bTempLeftLaneValid2_bool;
    boolean bTempRightLaneValid2_bool;
    uint8 uiShPtXInLane_nu[4];
    boolean bTempAllShPtInSameLane_bool;
    uint8 uiShPtIdx;
    float32 fTempShPt0Pos_met[2];
    float32 fTempShPt1Pos_met[2];
    float32 fTempShPt2Pos_met[2];
    float32 fTempShPt3Pos_met[2];
    float32 fTempArea_met2;
    float32 fTempBoundBoxArea_met2;
    uint8 uiTempObjInLaneProb_perc;

    ILELaneClothoidInfo_t sLaneClothoidInfo = {0};

    /* Determine lane validity */
    bTempLeftLaneValid_bool =
        ((pABPRLaneData->uiLeftLaneInvalidCheck_btf & 0x6000) > 0u) ||
        (pABPRLaneData->uiLeftLaneInvalidCheck_btf == 0u);
    bTempRightLaneValid_bool =
        ((pABPRLaneData->uiRightLaneInvalidCheck_btf & 0x6000) > 0u) ||
        (pABPRLaneData->uiRightLaneInvalidCheck_btf == 0u);

    bTempLeftLaneValid2_bool =
        ((pABPRLaneData->uiLeftLaneInvalidCheck_btf & 0x6008) > 0u) ||
        (pABPRLaneData->uiLeftLaneInvalidCheck_btf == 0u);
    bTempRightLaneValid2_bool =
        ((pABPRLaneData->uiRightLaneInvalidCheck_btf & 0x6008) > 0u) ||
        (pABPRLaneData->uiRightLaneInvalidCheck_btf == 0u);

    for (uint8 i = 0; i < TUE_CML_Min(pGenObjList->uiNumOfObject,
                                      ILE_INPUT_RADAR_OBJECT_NUM);
         i++) {
        /* Determine lane and object shape point validity */
        bTempShPtAndLaneValid_bool =
            (bTempLeftLaneValid_bool || bTempRightLaneValid_bool) &&
            pSSRObjInfoList->sSSRObjInfo[i].bObjShPtValidChk_bool;

        /* Object in lane association */
        if (bTempShPtAndLaneValid_bool) {
            /* Shape points to lane association */

            /* 1. Determine ego lane boundaries for shape points association*/
            // If one of lanes is invalid, then rebuild the invalid lane with
            // the valid lane information
            if (!(bTempLeftLaneValid2_bool && bTempRightLaneValid2_bool)) {
                if (bTempLeftLaneValid2_bool) {
                    pLaneAfterBuild->fRightLaneClothoidCurveDer_1pm2 =
                        pABPRLaneData->fLeftLaneClothoidCurveDer_1pm2;
                    pLaneAfterBuild->fRightLaneClothoidCurve_1pm =
                        pABPRLaneData->fLeftLaneClothoidCurve_1pm;
                    pLaneAfterBuild->fRightLaneClothoidHeading_rad =
                        pABPRLaneData->fLeftLaneClothoidHeading_rad;
                    pLaneAfterBuild->fRightLaneClothoidLength_met =
                        pABPRLaneData->fLeftLaneClothoidLength_met;
                    pLaneAfterBuild->uiRightLaneInvalidCheck_btf =
                        pABPRLaneData->uiLeftLaneInvalidCheck_btf;
                    pLaneAfterBuild->uiRightLaneQuality_perc =
                        pABPRLaneData->uiLeftLaneQuality_perc;
                    pLaneAfterBuild->fRightLaneClothoidPosY0_met =
                        pABPRLaneData->fLeftLaneClothoidPosY0_met -
                        ILE_DEFAULT_LANE_WIDTH_MET;
                } else {
                    pLaneAfterBuild->fLeftLaneClothoidCurveDer_1pm2 =
                        pABPRLaneData->fRightLaneClothoidCurveDer_1pm2;
                    pLaneAfterBuild->fLeftLaneClothoidCurve_1pm =
                        pABPRLaneData->fRightLaneClothoidCurve_1pm;
                    pLaneAfterBuild->fLeftLaneClothoidHeading_rad =
                        pABPRLaneData->fRightLaneClothoidHeading_rad;
                    pLaneAfterBuild->fLeftLaneClothoidLength_met =
                        pABPRLaneData->fRightLaneClothoidLength_met;
                    pLaneAfterBuild->uiLeftLaneInvalidCheck_btf =
                        pABPRLaneData->uiRightLaneInvalidCheck_btf;
                    pLaneAfterBuild->uiLeftLaneQuality_perc =
                        pABPRLaneData->uiRightLaneQuality_perc;
                    pLaneAfterBuild->fLeftLaneClothoidPosY0_met =
                        pABPRLaneData->fRightLaneClothoidPosY0_met +
                        ILE_DEFAULT_LANE_WIDTH_MET;
                }
            } else {
                *pLaneAfterBuild = *pABPRLaneData;
            }

            /* 2. Evaluate lane clothoid polynomial at object shape point */
            ILEEvalLaneClothoid(pLaneAfterBuild, pSSRObjInfoList, i,
                                uiShPtXInLane_nu);

            /* Object to lane association */
            for (uiShPtIdx = 0; uiShPtIdx < 4; uiShPtIdx++) {
                if (uiShPtXInLane_nu[uiShPtIdx] != uiShPtXInLane_nu[0]) {
                    break;
                }
            }
            bTempAllShPtInSameLane_bool = (uiShPtIdx == 4);

            if (bTempAllShPtInSameLane_bool) {
                // Shape point lane association
                pSSRObjInfoList->sSSRObjInfo[i].uiObjBelongToLane_nu =
                    uiShPtXInLane_nu[0];
                // Object in lane probability
                pSSRObjInfoList->sSSRObjInfo[i]
                    .uiObjInLaneProb_perc[uiShPtXInLane_nu[0]] = 100u;
            } else {
                /* Build lane according to info. that which lane object on  */
                if (ILEArrayElementChk(uiShPtXInLane_nu,
                                       (uint8)LANE_OUTSIDE_LEFT)) {
                    sLaneClothoidInfo.fLaneClothoidCurveDer_1pm2 =
                        pLaneAfterBuild->fLeftLaneClothoidCurveDer_1pm2;
                    sLaneClothoidInfo.fLaneClothoidCurve_1pm =
                        pLaneAfterBuild->fLeftLaneClothoidCurve_1pm;
                    sLaneClothoidInfo.fLaneClothoidHeading_rad =
                        pLaneAfterBuild->fLeftLaneClothoidHeading_rad;
                    sLaneClothoidInfo.fLaneClothoidPosY0_met =
                        pLaneAfterBuild->fLeftLaneClothoidPosY0_met +
                        ILE_DEFAULT_LANE_WIDTH_MET;
                } else if (ILEArrayElementChk(uiShPtXInLane_nu,
                                              (uint8)LANE_LEFT)) {
                    sLaneClothoidInfo.fLaneClothoidCurveDer_1pm2 =
                        pLaneAfterBuild->fLeftLaneClothoidCurveDer_1pm2;
                    sLaneClothoidInfo.fLaneClothoidCurve_1pm =
                        pLaneAfterBuild->fLeftLaneClothoidCurve_1pm;
                    sLaneClothoidInfo.fLaneClothoidHeading_rad =
                        pLaneAfterBuild->fLeftLaneClothoidHeading_rad;
                    sLaneClothoidInfo.fLaneClothoidPosY0_met =
                        pLaneAfterBuild->fLeftLaneClothoidPosY0_met;
                } else if (ILEArrayElementChk(uiShPtXInLane_nu,
                                              (uint8)LANE_EGO)) {
                    sLaneClothoidInfo.fLaneClothoidCurveDer_1pm2 =
                        pLaneAfterBuild->fRightLaneClothoidCurveDer_1pm2;
                    sLaneClothoidInfo.fLaneClothoidCurve_1pm =
                        pLaneAfterBuild->fRightLaneClothoidCurve_1pm;
                    sLaneClothoidInfo.fLaneClothoidHeading_rad =
                        pLaneAfterBuild->fRightLaneClothoidHeading_rad;
                    sLaneClothoidInfo.fLaneClothoidPosY0_met =
                        pLaneAfterBuild->fRightLaneClothoidPosY0_met;
                } else {
                    sLaneClothoidInfo.fLaneClothoidCurveDer_1pm2 =
                        pLaneAfterBuild->fRightLaneClothoidCurveDer_1pm2;
                    sLaneClothoidInfo.fLaneClothoidCurve_1pm =
                        pLaneAfterBuild->fRightLaneClothoidCurve_1pm;
                    sLaneClothoidInfo.fLaneClothoidHeading_rad =
                        pLaneAfterBuild->fRightLaneClothoidHeading_rad;
                    sLaneClothoidInfo.fLaneClothoidPosY0_met =
                        pLaneAfterBuild->fRightLaneClothoidPosY0_met -
                        ILE_DEFAULT_LANE_WIDTH_MET;
                }

                /* Calculate driving in lane percentage for each relevant object
                 */
                fTempShPt0Pos_met[0] =
                    pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[0];
                fTempShPt0Pos_met[1] =
                    pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[0];
                fTempShPt3Pos_met[0] =
                    pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[3];
                fTempShPt3Pos_met[1] =
                    pSSRObjInfoList->sSSRObjInfo[i].fShPtPosYTranslated_met[3];

                fTempShPt1Pos_met[0] =
                    TUE_CML_Abs(pSSRObjInfoList->sSSRObjInfo[i]
                                    .fShPtPosXTranslated_met[0] +
                                pSSRObjInfoList->sSSRObjInfo[i]
                                    .fShPtPosXTranslated_met[1]) /
                    2.f;
                fTempShPt2Pos_met[0] =
                    TUE_CML_Abs(pSSRObjInfoList->sSSRObjInfo[i]
                                    .fShPtPosXTranslated_met[2] +
                                pSSRObjInfoList->sSSRObjInfo[i]
                                    .fShPtPosXTranslated_met[3]) /
                    2.f;

                fTempShPt1Pos_met[1] = ILEEvalLaneBoundary(
                    fTempShPt1Pos_met[0],
                    sLaneClothoidInfo.fLaneClothoidPosY0_met,
                    sLaneClothoidInfo.fLaneClothoidHeading_rad,
                    sLaneClothoidInfo.fLaneClothoidCurve_1pm,
                    sLaneClothoidInfo.fLaneClothoidCurveDer_1pm2);
                fTempShPt2Pos_met[1] = ILEEvalLaneBoundary(
                    fTempShPt2Pos_met[0],
                    sLaneClothoidInfo.fLaneClothoidPosY0_met,
                    sLaneClothoidInfo.fLaneClothoidHeading_rad,
                    sLaneClothoidInfo.fLaneClothoidCurve_1pm,
                    sLaneClothoidInfo.fLaneClothoidCurveDer_1pm2);

                // Calculate area between points with Gauss's area formula
                fTempArea_met2 =
                    ILECalcAreaBetwPts(fTempShPt0Pos_met, fTempShPt1Pos_met,
                                       fTempShPt2Pos_met, fTempShPt3Pos_met);

                // Calculate object bounding box area
                fTempBoundBoxArea_met2 = TUE_CML_Abs(
                    TUE_CML_SqrtApprox(
                        TUE_CML_Sqr(pSSRObjInfoList->sSSRObjInfo[i]
                                        .fShPtPosXTranslated_met[0] -
                                    pSSRObjInfoList->sSSRObjInfo[i]
                                        .fShPtPosXTranslated_met[1]) +
                        TUE_CML_Sqr(pSSRObjInfoList->sSSRObjInfo[i]
                                        .fShPtPosYTranslated_met[0] -
                                    pSSRObjInfoList->sSSRObjInfo[i]
                                        .fShPtPosYTranslated_met[1])) *
                    TUE_CML_SqrtApprox(
                        TUE_CML_Sqr(pSSRObjInfoList->sSSRObjInfo[i]
                                        .fShPtPosXTranslated_met[0] -
                                    pSSRObjInfoList->sSSRObjInfo[i]
                                        .fShPtPosXTranslated_met[1]) +
                        TUE_CML_Sqr(pSSRObjInfoList->sSSRObjInfo[i]
                                        .fShPtPosYTranslated_met[0] -
                                    pSSRObjInfoList->sSSRObjInfo[i]
                                        .fShPtPosYTranslated_met[1])));

                // Shape point lane association
                uiTempObjInLaneProb_perc = (uint8)TUE_CML_MinMax(
                    0.f, 100.f,
                    ((fTempArea_met2 / SafeDiv(fTempBoundBoxArea_met2)) *
                     100.f));
                pSSRObjInfoList->sSSRObjInfo[i].uiObjBelongToLane_nu =
                    uiTempObjInLaneProb_perc > 50u ? uiShPtXInLane_nu[0]
                                                   : uiShPtXInLane_nu[2];
                // Object in lane probability
                pSSRObjInfoList->sSSRObjInfo[i]
                    .uiObjInLaneProb_perc[uiShPtXInLane_nu[0]] =
                    uiTempObjInLaneProb_perc;
                pSSRObjInfoList->sSSRObjInfo[i]
                    .uiObjInLaneProb_perc[uiShPtXInLane_nu[2]] =
                    100u - uiTempObjInLaneProb_perc;
            }

        }
        /* UNKNOWN shape point or no lane data valid */
        else {
            // Shape point lane association
            pSSRObjInfoList->sSSRObjInfo[i].uiObjBelongToLane_nu =
                (uint8)LANE_UNKNOWN;
            // Object in lane probability
            for (uint8 j = 0; j < 6; j++) {
                pSSRObjInfoList->sSSRObjInfo[i].uiObjInLaneProb_perc[j] = 0u;
            }
        }
    }

    /* Debug output */
    pILEDebug->bTempLeftLaneValid_bool = bTempLeftLaneValid_bool;
    pILEDebug->bTempRightLaneValid_bool = bTempRightLaneValid_bool;
    pILEDebug->bTempLeftLaneValid2_bool = bTempLeftLaneValid2_bool;
    pILEDebug->bTempRightLaneValid2_bool = bTempRightLaneValid2_bool;
    for (uint8 i = 0; i < TUE_CML_Min(pGenObjList->uiNumOfObject,
                                      ILE_INPUT_RADAR_OBJECT_NUM);
         i++) {
        pILEDebug->uiObjBelongToLane_nu[i] =
            pSSRObjInfoList->sSSRObjInfo[i].uiObjBelongToLane_nu;
    }
}

/*****************************************************************************
  Functionname:    ILEEvalLaneBoundary */ /*!

                                      @brief           Evaluate lane clothoid
                                      polynomial at obejct shape point

                                      @description     Evaluate lane clothoid
                                      polynomial at obejct shape point. This
                                               function is corresponding to
                                      EvaluateLateBoundaryAtX
                                               module in MBD.

                                      @param[in]       fShPtPosX       Position
                                      X of shape point
                                      @param[in]       fPosY0          Position
                                      Y offset of left or right lane
                                      @param[in]       fHeading        Heading
                                      angle of left or right lane
                                      @param[in]       fCrv            Curve of
                                      left or right lane
                                      @param[in]       fCrvChng        First
                                      derivative of curve of left or right lane

                                      @return          fPosYOut_met    Position
                                      Y on lane by substituting the position
                                      X
                                                               of shape point
                                      into the left or right lane
                                      *****************************************************************************/
STATIc float32 ILEEvalLaneBoundary(const float32 fShPtPosX,
                                   const float32 fPosY0,
                                   const float32 fHeading,
                                   const float32 fCrv,
                                   const float32 fCrvChng) {
    float32 fPosYOut_met;  // result value
    float32 fTempHeadingTan_nu;
    float32 fTempPosYOut_met;

    fTempHeadingTan_nu =
        TAN_(TUE_CML_MinMax(-TUE_CML_Pi / 4.f, TUE_CML_Pi / 4.f, fHeading));

    fTempPosYOut_met = fTempHeadingTan_nu * fShPtPosX + fPosY0 +
                       0.5f * fCrv * TUE_CML_Sqr(fShPtPosX);
    fPosYOut_met = 0.166666f * fCrvChng * fShPtPosX * TUE_CML_Sqr(fShPtPosX) +
                   fTempPosYOut_met;

    return fPosYOut_met;
}

/*****************************************************************************
  Functionname:    ILEArrayElementChk */ /*!

                                       @brief           Array element check

                                       @description     Array element check. The
                                       function returns TRUE if an element
                                       exists
                                                  in the array. This function is
                                       used in ObjectToLaneEvaluation
                                                  module in MBD.

                                       @param[in]       pArray
                                       Array needs to be checked
                                       @param[in]       uiElement
                                       Element in array

                                       @return          bElementInArray_bool
                                       Flat that element in  the array
                                       *****************************************************************************/
STATIc boolean ILEArrayElementChk(const uint8* pArray, const uint8 uiElement) {
    boolean bElementInArray_bool = FALSE;  // result value

    for (uint8 i = 0; i < 4; i++) {
        if (pArray[i] == uiElement) {
            bElementInArray_bool = TRUE;
            break;
        }
    }

    return bElementInArray_bool;
}

/*****************************************************************************
  Functionname:    ILECalcAreaBetwPts */ /*!

                                       @brief           Calculate area between
                                       points

                                       @description     Use Shoelace formula to
                                       calculate area between points. This
                                                  function is corresponding to
                                       CalculateAreaBetweenPoints
                                                  module in MBD.

                                       @param[in]       pPt0Array      Array
                                       contains posX and posY of point0
                                       @param[in]       pPt1Array      Array
                                       contains posX and posY of point1
                                       @param[in]       pPt2Array      Array
                                       contains posX and posY of point2
                                       @param[in]       pPt3Array      Array
                                       contains posX and posY of point3

                                       @return          fArea_met2     Area of a
                                       polygon formed by four points
                                       *****************************************************************************/
STATIc float32 ILECalcAreaBetwPts(const float32* pPt0Array,
                                  const float32* pPt1Array,
                                  const float32* pPt2Array,
                                  const float32* pPt3Array) {
    float32 fArea_met2;  // result value
    float32 fPt0Area_met2;
    float32 fPt1Area_met2;
    float32 fPt2Area_met2;
    float32 fPt3Area_met2;

    fPt0Area_met2 =
        (pPt0Array[0] - pPt1Array[0]) * (pPt0Array[1] + pPt1Array[1]);
    fPt1Area_met2 =
        (pPt1Array[0] - pPt2Array[0]) * (pPt1Array[1] + pPt2Array[1]);
    fPt2Area_met2 =
        (pPt2Array[0] - pPt3Array[0]) * (pPt2Array[1] + pPt3Array[1]);
    fPt3Area_met2 =
        (pPt3Array[0] - pPt0Array[0]) * (pPt3Array[1] + pPt0Array[1]);

    fArea_met2 = TUE_CML_Abs(
        0.5f * (fPt0Area_met2 + fPt1Area_met2 + fPt2Area_met2 + fPt3Area_met2));

    return fArea_met2;
}

/*****************************************************************************
  Functionname:    ILEOutputDataGene */ /*!

                                        @brief           Output data generation

                                        @description     Output data generation.
                                        This function outputs three key side
                                                 radar targets that located in
                                        ego, left and right lane separately.
                                                 This function is corresponding
                                        to OutputDataGeneration
                                                 module in MBD.

                                        @param[in]       pGenObjList      Side
                                        radar object list
                                        @param[in]       pSSRObjInfoList  side
                                        radar object list information after
                                        modified
                                        @param[in,out]   pILEOutput       Three
                                        key side radar targets

                                        @return          none
                                        @uml
                                        @startuml
                                        start
                                        partition IterForObjID {
                                        repeat
                                        if (Object in relevant lane &\nAt least
                                        one ShPt PosX < thres &\n PreCycle
                                        ObjID is not 0xFF(invalid)) then (yes)
                                          if (PosX of ShPt0 of current obj <
                                        \nPosX of ShPt0 of PreCycle obj) then
                                        (yes)
                                              :Current cycle ObjID is \nthe
                                        desired ObjID ;
                                          else (no)
                                              :PreCycle ObjID is \nthe desired
                                        ObjID;
                                          endif
                                        else (no)
                                          if (Object in relevant lane &\nAt
                                        least one ShPt PosX < thres) then (yes)
                                              :Current cycle ObjID is \nthe
                                        desired ObjID ;
                                          else (no)
                                              :Desired ObjID is \ndefault
                                        value(0xFF);
                                          endif
                                        endif
                                        repeat while (more objects?)
                                        note:Lane validity check
                                        }
                                        partition ObjInfoOutput {
                                        fork
                                        :Left lane object output;
                                        if (Left lane ObjID is 0xFF) then (yes)
                                          :Output default invalid value ;
                                        else (no)
                                          :Output ObjID information;
                                        endif
                                        fork again
                                        :Ego lane object output;
                                        if (Ego lane ObjID is 0xFF) then (yes)
                                          :Output default invalid value ;
                                        else (no)
                                          :Output ObjID information;
                                        endif
                                        fork again
                                        :Right lane object output;
                                        if (Right lane ObjID is 0xFF) then (yes)
                                          :Output default invalid value ;
                                        else (no)
                                          :Output ObjID information;
                                        endif
                                        end fork
                                        }
                                        stop
                                        @enduml
                                        *****************************************************************************/
STATIc void ILEOutputDataGene(const ODPRInSSRObjList_t* pGenObjList,
                              const ILESSRObjInfoList_t* pSSRObjInfoList,
                              ODPRILEOut_t* pILEOutput) {
    boolean bTempObjInReleRange_bool;
    uint8 uiTempLeLnObjID_nu = 0xFF;
    uint8 uiTempEgoLnObjID_nu = 0xFF;
    uint8 uiTempRiLnObjID_nu = 0xFF;
    float32 fTempShPt0PosX_met;
    float32 fTempShPt2PosX_met;
    float32 fTempShPt0PosY_met;
    float32 fTempShPt2PosY_met;

    /* Iteration for relevant object selection */
    for (uint8 i = 0; i < TUE_CML_Min(pGenObjList->uiNumOfObject,
                                      ILE_INPUT_RADAR_OBJECT_NUM);
         i++) {
        /* 1.Check object in relevant range */
        bTempObjInReleRange_bool = FALSE;

        // At least one object shape point longitudinal distance is less the
        // threshold
        for (uint8 j = 0; j < 4; j++) {
            if (pSSRObjInfoList->sSSRObjInfo[i].fShPtPosXTranslated_met[j] <
                ILE_RELEVANT_OBJ_LONG_DIST_MET) {
                bTempObjInReleRange_bool = TRUE;
                break;
            }
        }

        /* 2.Determine lane relevant object */
        uiTempLeLnObjID_nu =
            ILEDeterLaneReleObj(bTempObjInReleRange_bool, pSSRObjInfoList, i,
                                (uint8)LANE_LEFT, &ILE_fPreCycIterLeLnObjID_nu);
        uiTempEgoLnObjID_nu =
            ILEDeterLaneReleObj(bTempObjInReleRange_bool, pSSRObjInfoList, i,
                                (uint8)LANE_EGO, &ILE_fPreCycIterEgoLnObjID_nu);
        uiTempRiLnObjID_nu = ILEDeterLaneReleObj(
            bTempObjInReleRange_bool, pSSRObjInfoList, i, (uint8)LANE_RIGHT,
            &ILE_fPreCycIterRiLnObjID_nu);
    }

    /* Left,ego and right lane object selection */
    /* 1.Left lane object selection */
    if (uiTempLeLnObjID_nu == 0xFF) {
        pILEOutput->fLeftLaneObjPosX_met = 0.f;
        pILEOutput->fLeftLaneObjPosY_met = 0.f;
        pILEOutput->fLeftLaneObjRelVelX_mps = 0.f;
        pILEOutput->fLeftLaneObjRelVelY_mps = 0.f;
        pILEOutput->uiLeftLaneObjDriveEgoLane_perc = 0u;
        pILEOutput->uiLeftLaneObjDriveLeftLane_perc = 0u;
        pILEOutput->uiLeftLaneObjDriveOutLeftLane_perc = 0u;
        pILEOutput->uiLeftLaneObjExistProb_perc = 0u;
        pILEOutput->uiLeftLaneObjID_nu = 0xFF;
    } else {
        fTempShPt0PosX_met = (pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                                  .fShPtPosXTranslated_met[0] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                                  .fShPtPosXTranslated_met[1]) /
                             2.f;
        fTempShPt2PosX_met = (pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                                  .fShPtPosXTranslated_met[2] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                                  .fShPtPosXTranslated_met[3]) /
                             2.f;
        fTempShPt0PosY_met = (pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                                  .fShPtPosYTranslated_met[0] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                                  .fShPtPosYTranslated_met[1]) /
                             2.f;
        fTempShPt2PosY_met = (pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                                  .fShPtPosYTranslated_met[2] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                                  .fShPtPosYTranslated_met[3]) /
                             2.f;

        pILEOutput->fLeftLaneObjPosX_met =
            (fTempShPt0PosX_met < fTempShPt2PosX_met) ? fTempShPt0PosX_met
                                                      : fTempShPt2PosX_met;
        pILEOutput->fLeftLaneObjPosY_met =
            (fTempShPt0PosX_met < fTempShPt2PosX_met) ? fTempShPt0PosY_met
                                                      : fTempShPt2PosY_met;
        pILEOutput->fLeftLaneObjRelVelX_mps =
            pGenObjList->ObjectArray[uiTempLeLnObjID_nu].fRelVelX_mps;
        pILEOutput->fLeftLaneObjRelVelY_mps =
            pGenObjList->ObjectArray[uiTempLeLnObjID_nu].fRelVelY_mps;
        pILEOutput->uiLeftLaneObjDriveEgoLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_EGO];
        ;
        pILEOutput->uiLeftLaneObjDriveLeftLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_LEFT];
        pILEOutput->uiLeftLaneObjDriveOutLeftLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempLeLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_OUTSIDE_LEFT];
        pILEOutput->uiLeftLaneObjExistProb_perc =
            pGenObjList->ObjectArray[uiTempLeLnObjID_nu].uiExistProb_nu;
        pILEOutput->uiLeftLaneObjID_nu =
            pGenObjList->ObjectArray[uiTempLeLnObjID_nu].uiID_nu;
    }

    /* 1.Ego lane object selection */
    if (uiTempEgoLnObjID_nu == 0xFF) {
        pILEOutput->fEgoLaneObjPosX_met = 0.f;
        pILEOutput->fEgoLaneObjPosY_met = 0.f;
        pILEOutput->fEgoLaneObjRelVelX_mps = 0.f;
        pILEOutput->fEgoLaneObjRelVelY_mps = 0.f;
        pILEOutput->uiEgoLaneObjDriveEgoLane_perc = 0u;
        pILEOutput->uiEgoLaneObjDriveLeftLane_perc = 0u;
        pILEOutput->uiEgoLaneObjDriveRightLane_perc = 0u;
        pILEOutput->uiEgoLaneObjExistProb_perc = 0u;
        pILEOutput->uiEgoLaneObjID_nu = 0xFF;
    } else {
        fTempShPt0PosX_met = (pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                                  .fShPtPosXTranslated_met[0] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                                  .fShPtPosXTranslated_met[1]) /
                             2.f;
        fTempShPt2PosX_met = (pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                                  .fShPtPosXTranslated_met[2] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                                  .fShPtPosXTranslated_met[3]) /
                             2.f;
        fTempShPt0PosY_met = (pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                                  .fShPtPosYTranslated_met[0] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                                  .fShPtPosYTranslated_met[1]) /
                             2.f;
        fTempShPt2PosY_met = (pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                                  .fShPtPosYTranslated_met[2] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                                  .fShPtPosYTranslated_met[3]) /
                             2.f;

        pILEOutput->fEgoLaneObjPosX_met =
            (fTempShPt0PosX_met < fTempShPt2PosX_met) ? fTempShPt0PosX_met
                                                      : fTempShPt2PosX_met;
        pILEOutput->fEgoLaneObjPosY_met =
            (fTempShPt0PosX_met < fTempShPt2PosX_met) ? fTempShPt0PosY_met
                                                      : fTempShPt2PosY_met;
        pILEOutput->fEgoLaneObjRelVelX_mps =
            pGenObjList->ObjectArray[uiTempEgoLnObjID_nu].fRelVelX_mps;
        pILEOutput->fEgoLaneObjRelVelY_mps =
            pGenObjList->ObjectArray[uiTempEgoLnObjID_nu].fRelVelY_mps;
        pILEOutput->uiEgoLaneObjDriveEgoLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_EGO];
        pILEOutput->uiEgoLaneObjDriveLeftLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_LEFT];
        pILEOutput->uiEgoLaneObjDriveRightLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempEgoLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_RIGHT];
        pILEOutput->uiEgoLaneObjExistProb_perc =
            pGenObjList->ObjectArray[uiTempEgoLnObjID_nu].uiExistProb_nu;
        pILEOutput->uiEgoLaneObjID_nu =
            pGenObjList->ObjectArray[uiTempEgoLnObjID_nu].uiID_nu;
    }

    /* 1.Right lane object selection */
    if (uiTempRiLnObjID_nu == 0xFF) {
        pILEOutput->fRightLaneObjPosX_met = 0.f;
        pILEOutput->fRightLaneObjPosY_met = 0.f;
        pILEOutput->fRightLaneObjRelVelX_mps = 0.f;
        pILEOutput->fRightLaneObjRelVelY_mps = 0.f;
        pILEOutput->uiRightLaneObjDriveEgoLane_perc = 0u;
        pILEOutput->uiRightLaneObjDriveOutRightLane_perc = 0u;
        pILEOutput->uiRightLaneObjDriveRightLane_perc = 0u;
        pILEOutput->uiRightLaneObjExistProb_perc = 0u;
        pILEOutput->uiRightLaneObjID_nu = 0xFF;
    } else {
        fTempShPt0PosX_met = (pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                                  .fShPtPosXTranslated_met[0] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                                  .fShPtPosXTranslated_met[1]) /
                             2.f;
        fTempShPt2PosX_met = (pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                                  .fShPtPosXTranslated_met[2] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                                  .fShPtPosXTranslated_met[3]) /
                             2.f;
        fTempShPt0PosY_met = (pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                                  .fShPtPosYTranslated_met[0] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                                  .fShPtPosYTranslated_met[1]) /
                             2.f;
        fTempShPt2PosY_met = (pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                                  .fShPtPosYTranslated_met[2] +
                              pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                                  .fShPtPosYTranslated_met[3]) /
                             2.f;

        pILEOutput->fRightLaneObjPosX_met =
            (fTempShPt0PosX_met < fTempShPt2PosX_met) ? fTempShPt0PosX_met
                                                      : fTempShPt2PosX_met;
        pILEOutput->fRightLaneObjPosY_met =
            (fTempShPt0PosX_met < fTempShPt2PosX_met) ? fTempShPt0PosY_met
                                                      : fTempShPt2PosY_met;
        pILEOutput->fRightLaneObjRelVelX_mps =
            pGenObjList->ObjectArray[uiTempRiLnObjID_nu].fRelVelX_mps;
        pILEOutput->fRightLaneObjRelVelY_mps =
            pGenObjList->ObjectArray[uiTempRiLnObjID_nu].fRelVelY_mps;
        pILEOutput->uiRightLaneObjDriveEgoLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_EGO];
        ;
        pILEOutput->uiRightLaneObjDriveOutRightLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_OUTSIDE_RIGHT];
        pILEOutput->uiRightLaneObjDriveRightLane_perc =
            pSSRObjInfoList->sSSRObjInfo[uiTempRiLnObjID_nu]
                .uiObjInLaneProb_perc[(uint8)LANE_RIGHT];
        pILEOutput->uiRightLaneObjExistProb_perc =
            pGenObjList->ObjectArray[uiTempRiLnObjID_nu].uiExistProb_nu;
        pILEOutput->uiRightLaneObjID_nu =
            pGenObjList->ObjectArray[uiTempRiLnObjID_nu].uiID_nu;
    }
}

/*****************************************************************************
  Functionname:    ILEDeterLaneReleObj */ /*!

                                      @brief           Determine lane relevant
                                      object

                                      @description     Determine lane relevant
                                      object. This function is corresponding
                                           to DetermineLaneRelevantObject module
                                      in MBD.

                                      @param[in]       bObjInReleRange
                                      Falg that object in relevant range
                                      @param[in]       pSSRObjInfoList
                                      Side radar object list information after
                                      modified
                                      @param[in]       uiForLoopIdx          For
                                      loop index
                                      @param[in]       uiReleLane Relevant lane
                                      @param[in]       pPreCycLastIterObjID
                                      Global variable address that stores
                                      previous cycle ObjID

                                      @return          uiReleObjID_nu
                                      ObjID in current cycle that explain which
                                      object in relevant lane
                                      *****************************************************************************/
STATIc uint8 ILEDeterLaneReleObj(const boolean bObjInReleRange,
                                 const ILESSRObjInfoList_t* pSSRObjInfoList,
                                 const uint8 uiForLoopIdx,
                                 const uint8 uiReleLane,
                                 uint8* pPreCycLastIterObjID) {
    uint8 uiReleObjID_nu;  // result value
    boolean bTempReleObjExist_bool;

    bTempReleObjExist_bool =
        (bObjInReleRange &&
         (pSSRObjInfoList->sSSRObjInfo[uiForLoopIdx].uiObjBelongToLane_nu ==
          uiReleLane));

    if (bTempReleObjExist_bool && (*pPreCycLastIterObjID != 0xFF)) {
        if (pSSRObjInfoList->sSSRObjInfo[uiForLoopIdx]
                .fShPtPosXTranslated_met[0] <
            pSSRObjInfoList->sSSRObjInfo[*pPreCycLastIterObjID]
                .fShPtPosXTranslated_met[0]) {
            uiReleObjID_nu = uiForLoopIdx;
        } else {
            uiReleObjID_nu = *pPreCycLastIterObjID;
        }
    } else {
        uiReleObjID_nu =
            bTempReleObjExist_bool ? uiForLoopIdx : *pPreCycLastIterObjID;
    }

    *pPreCycLastIterObjID = uiReleObjID_nu;

    return uiReleObjID_nu;
}

/*****************************************************************************
  Functionname:    ODPR_ILE_Exec                                           */ /*!

        @brief           Main function of In Lane Evaluate

        @description     In lane evaluate that peforms side radar objects
      preprocessing
                         and outputs three key side radar targets that located
      in
      ego,
                         left and right lane separately.

        @param[in]       reqPorts              Input structure of ILE function
                              pABPRLaneData    Left and right lanes information
                              pGenObjList      Side radar object list
        @param[in]       param                 Param structure of ILE function
                              pILEParam        ILE parameters
        @param[in,out]   proPorts              Output structure of ILE function
                              pILEOutput       Three key side radar targets
      output
      by ILE
        @param[in,out]   debugInfo             Debug info of ILE function
                              pILEDebug        Debug signal of ILE

        @return          none
      *****************************************************************************/
void ODPR_ILE_Exec(const ILEInReq_t* reqPorts,
                   const ODPRParam_t* param,
                   ILEOutPro_t* proPorts,
                   ILEDebug_t* debugInfo) {
    /* Input and output wrapper */
    const ODPRInABPRLaneInfo_t* pLaneData = reqPorts->pLaneData;
    const ODPRInSSRObjList_t* pObjectList = reqPorts->pObjectList;
    const ODPRParam_t* pParam = param;
    ODPRILEOut_t* pILEOutData = proPorts->pILEOutData;
    ODPRILEDebug_t* pILEDebug = debugInfo->pILEDebug;

    /* VARIABLES */
    ILESSRObjInfoList_t sSSRObjInfoList = {0};
    ODPRInABPRLaneInfo_t sLaneAfterBuild = {0};

    /* Determine shape points */
    ILEDetermineShPt(pObjectList, pParam, &sSSRObjInfoList, pILEDebug);

    /* Object in lane evaluation */
    ILEObjInLaneEval(pLaneData, pObjectList, &sLaneAfterBuild, &sSSRObjInfoList,
                     pILEDebug);

    /* Output data generation */
    ILEOutputDataGene(pObjectList, &sSSRObjInfoList, pILEOutData);
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */