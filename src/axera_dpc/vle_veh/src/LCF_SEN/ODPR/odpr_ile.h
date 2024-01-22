#ifndef ODPR_ILE_H
#define ODPR_ILE_H
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "odpr_ext.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef struct {
    boolean bObjShPtValidChk_bool;
    uint8 uiObjBelongToLane_nu;
    uint8 uiObjInLaneProb_perc[6];
    float32 fShPtPosXTranslated_met[4];
    float32 fShPtPosYTranslated_met[4];
} ILESSRObjInfo_t;

typedef struct {
    ILESSRObjInfo_t sSSRObjInfo[ILE_INPUT_RADAR_OBJECT_NUM];
} ILESSRObjInfoList_t;

typedef enum {
    LANE_UNKNOWN,
    LANE_EGO,
    LANE_LEFT,
    LANE_RIGHT,
    LANE_OUTSIDE_LEFT,
    LANE_OUTSIDE_RIGHT
} ILELaneLocat;

typedef enum {
    INVALID,
    EDGE_MEASURED,
    LANEDGE_ASSUMEDE_LEFT,
    VISIBILITY_EDGE
} ILEShPtState;

typedef struct {
    float32 fLaneClothoidPosY0_met;
    float32 fLaneClothoidHeading_rad;
    float32 fLaneClothoidCurve_1pm;
    float32 fLaneClothoidCurveDer_1pm2;
} ILELaneClothoidInfo_t;

typedef struct {
    const ODPRInABPRLaneInfo_t* pLaneData;
    const ODPRInSSRObjList_t* pObjectList;
} ILEInReq_t;

// typedef struct
//{
//	ODPRILEParam_t* pILEParam;
//} ILEParam_t;

typedef struct { ODPRILEOut_t* pILEOutData; } ILEOutPro_t;

typedef struct { ODPRILEDebug_t* pILEDebug; } ILEDebug_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
STATIc void ILEDetermineShPt(const ODPRInSSRObjList_t* pGenObjList,
                             const ODPRParam_t* pParam,
                             ILESSRObjInfoList_t* pSSRObjInfoList,
                             ODPRILEDebug_t* pILEDebug);
STATIc float32 ILEEvalLaneBoundary(const float32 fShPtPosX,
                                   const float32 fPosY0,
                                   const float32 fHeading,
                                   const float32 fCrv,
                                   const float32 fCrvChng);
STATIc void ILEEvalLaneClothoid(ODPRInABPRLaneInfo_t* pLaneAfterBuild,
                                ILESSRObjInfoList_t* pSSRObjInfoList,
                                uint8 i,
                                uint8 uiShPtXInLane_nu[4]);
STATIc void ILEObjInLaneEval(const ODPRInABPRLaneInfo_t* pABPRLaneData,
                             const ODPRInSSRObjList_t* pGenObjList,
                             ODPRInABPRLaneInfo_t* pLaneAfterBuild,
                             ILESSRObjInfoList_t* pSSRObjInfoList,
                             ODPRILEDebug_t* pILEDebug);
STATIc boolean ILEArrayElementChk(const uint8* pArray, const uint8 uiElement);
STATIc float32 ILECalcAreaBetwPts(const float32* pPt0Array,
                                  const float32* pPt1Array,
                                  const float32* pPt2Array,
                                  const float32* pPt3Array);
STATIc void ILEOutputDataGene(const ODPRInSSRObjList_t* pGenObjList,
                              const ILESSRObjInfoList_t* pSSRObjInfoList,
                              ODPRILEOut_t* pILEOutput);
STATIc uint8 ILEDeterLaneReleObj(const boolean bObjInReleRange,
                                 const ILESSRObjInfoList_t* pSSRObjInfoList,
                                 const uint8 uiForLoopIdx,
                                 const uint8 uiReleLane,
                                 uint8* pPreCycLastIterObjID);

#endif