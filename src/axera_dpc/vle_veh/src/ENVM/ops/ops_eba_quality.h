
#ifndef _OPS_EBA_QUALITY_INCLUDED
#define _OPS_EBA_QUALITY_INCLUDED

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

// move update function to ops main for ACC Quality calculate by
// changchang20200610
//#define CONFIRM_DENSITY_SIZE           8U
//#define CONFIRM_DENSITY_DEAFULT        0U

// hypothesis category
#define OPS_EBA_HYP_CAT_NONE 0U
#define OPS_EBA_HYP_CAT_PED 1U
#define OPS_EBA_HYP_CAT_VCL 2U
#define OPS_EBA_HYP_CAT_XING 4U
#define OPS_EBA_HYP_CAT_ONC 8U
#define OPS_EBA_HYP_CAT_STAT 16U
#define OPS_EBA_HYP_CAT_CYCL 32U
#define OPS_EBA_HYP_CAT_ALL 255U

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/************************************************************************/
/* FUNCTIONS                                                   */
/************************************************************************/
void OPSAEBInit(void);
void OPSInitEBAObject(sint8 iObjNumber);
void OPSEBAPreselPreProc(void);
void OPSEBAPresel(void);
void OPSEBAPreselPostProc(void);

// move update function to ops main for ACC Quality calculate by
// changchang20200610
// void OPSObjectMeasureStatusUpdate(void);
void OPSResetEBAInhibitionMask(sint8 iObjNumber);
static boolean OPSCheckCrossingProperty(sint8 iObjNumber);
static void OPSSetEBAHypCat(sint8 iObjNumber, boolean bIsObjCrossing);
static uint8 OPS_EBA_UpdateNonAbsObservers(
    sint8 iObjNumber,
    uint8 iPerviousThresh,
    uint8 (*pOPS_EBA_GetObserver_Func)(sint8));

static void OPSCalculateObjectQuality(sint8 iObjNumber);
static void OPSUpdateTargetConfirmation(sint8 iObjNumber);
static void OPSUpdateAbsObservers(sint8 iObjNumber, uint8* piOutObjSafe);
static void OPSUpdateNonAbsObservers(sint8 iObjNumber, uint8* piOutThresh);
static float32 OPSTimeFilter(sint8 iObjNumber,
                             uint8 iThresh,
                             uint8 iLimit,
                             ui8_t uiObjSafe);

static ui8_t OPS_EBA_GetOBSObserver(sint8 iObjNumber);
static ui8_t OPS_EBA_GetRCSObserver(sint8 iObjNumber);
static ui8_t OPS_EBA_GetPOEObserver(sint8 iObjNumber);
static ui8_t OPS_EBA_GetLFTObserver(sint8 iObjNumber);

static boolean OPSCheckStatObstacleProbability(sint8 iObjNumber,
                                               uint8 uiObjProbThresh);
static boolean OPSCheckRCS(sint8 iObjNumber, float32 fObjRCSThresh);
static uint8 OPSGetPOEThreshold(sint8 iObjNumber);
static boolean OPSCheckPOE(sint8 iObjNumber, uint8 uiObjPOEThresh);

static uint8 OPS_EBA_GetOncomingObserver(sint8 iObjNumber);
static uint8 OPS_EBA_GetWideObserver(sint8 iObjNumber);

void OPSEBAPreSelCustomFct(sint8 iObjNumber);
static void OPSDynPropHistoryUpdate(sint8 ObjNumber, boolean bIsObjectCrossing);

#endif
