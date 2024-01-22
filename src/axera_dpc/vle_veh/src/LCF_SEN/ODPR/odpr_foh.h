#ifndef ODPR_FOH_H
#define ODPR_FOH_H
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "odpr_ext.h"
#include "odpr_polyfitTgtObjClothoid.h"
#include "odpr_TypeDef.h"
#include "tue_common_libs.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef struct {
    float32 fYawRateObjSync_rps;  // Yaw rate after syncronization
    float32 fYawRateRaw_rps;      // Raw yaw rate
    float32 fEgoCrv_1pm;          // Ego curve
    float32 fEgoVelX_mps;         // Ego velocity X
} FOHEgoMotion_t;

typedef struct {
    boolean bAccObjValid_bool : 1;
    boolean bAccObjFreezeStop_bool : 1;
    boolean bObjCutOut_bool : 1;
    boolean bObjCutIn_bool : 1;
    boolean bLSMInactive_bool : 1;
    boolean bTrajInvalid_bool : 1;
} FOHAccObjStatusChk_t;

typedef struct {
    boolean bSaveNewEntry_bool : 1;
    boolean bResetHistory_bool : 1;
    boolean bEnableHistory_bool : 1;
} FOHHistoryControl_t;

typedef struct {
    boolean bPreCycTrajInvalid_1st_bool;
    boolean bPreCycTrajInvalid_3rd_bool;
    uint8 uiPreCycNumOfValidSamples_nu;
    float32 fPreCycLastStoredPointX_met;
} FOHPreCycHistValid_t;

typedef struct {
    boolean bTransitionEnable_bool : 1;
    boolean bTransitionReset_bool : 1;
    boolean bCutInOngoing_bool : 1;
    boolean bCutOutOngoing_bool : 1;
    boolean bFreezeStopOnging_bool : 1;
    boolean bObjValidOngoing_bool : 1;
} FOHObjTranControl_t;

typedef struct {
    float32 fTransitionFactorA_fac;
    float32 fAccObjPosYTransPF_met;
    float32 fAccObjPosYTrans_met;
    float32 fAccObjRelYawAngTrans_rad;
    float32 fAccObjPosX_met;
    float32 fAccObjPosXStdDev_met;
    float32 fAccObjPosYStdDev_met;
} FOHTgtObjData_t;

typedef struct {
    FOHObjTranControl_t* pObjTranControl;
    FOHTgtObjData_t* pTgtObjData;
} FOHAccObjPreProcessing_t;

typedef struct {
    float32 fDetaMeanDevToTraj_met;
    float32 fHistLength_met;
    float32 fFirstStoredPntX_met;
    float32 fPosX0_met;
} FOHTgtObjCtdInfoAndCoeff_t;

typedef struct {
    float32 fPosY0_3rd_met;
    float32 fHeading_3rd_rad;
    float32 fCrv_1pm;
} FOHPolyfitSelec_t;

typedef struct {
    float32 fCurvature_1pm;
    float32 fPosY0_met;
    float32 fHeading_rad;
} FOHStrgtEstFadCrvLimit_t;

typedef struct {
    float32 fPosY0_met;
    float32 fHeading_rad;
    float32 fCurvature_1pm;
    float32 fChngOfCrv_1pm2;
} FOHLowPassFilter_t;

typedef struct {
    float32 fLengthLSM_met;
    float32 fPosY0LSM_met;
    float32 fHeadLSM_rad;
    float32 fCrvLSM_1pm;
} FOHLSMTgtTrajProcess_t;

typedef struct {
    boolean bTransitionEnable_bool;
    boolean bTransitionReset_bool;
    boolean bLSMTransOngoing_bool;
    float32 fTransitionValueA_fac;
} FOHModeTranControl_t;

typedef struct {
    const ODPRFOPOut_t* pFOPOutData;
    const ODPRInSystemPara_t* pSystemPara;
    const ODPRInVEDVehDyn_t* pEgoVehSig;
} FOHInReq_t;

// typedef struct
//{
//    ODPRFOHParam_t* pFOHParam;
//} FOHParam_t;

typedef struct {
    ODPRFOHOut_t* pFOHOutData;
} FOHOutPro_t;

typedef struct {
    ODPRFOHDebug_t* pFOHDebug;
} FOHDebug_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
STATIc FOHEgoMotion_t FOHEgoMotionCalc(const ODPRInVEDVehDyn_t* pEgoVEDData,
                                       ODPRFOHDebug_t* pFOHDebug);
STATIc FOHAccObjStatusChk_t
FOHAccObjStatusChk(const ODPRInVEDVehDyn_t* pEgoVEDData,
                   const ODPRInSystemPara_t* pSystemPara,
                   const ODPRFOPOut_t* pFOPOutput,
                   ODPRFOHDebug_t* pFOHDebug);
STATIc FOHHistoryControl_t
FOHHistoryControl(const ODPRInVEDVehDyn_t* pEgoVEDData,
                  const ODPRFOPOut_t* pFOPOutput,
                  const FOHAccObjStatusChk_t* pAccObjChk,
                  ODPRFOHOut_t* pFOHOutput,
                  ODPRFOHDebug_t* pFOHDebug);
STATIc void FOHAccObjPreProcessing(const FOHEgoMotion_t* pEgoMotion,
                                   const FOHAccObjStatusChk_t* pAccObjChk,
                                   const ODPRInSystemPara_t* pSystemPara,
                                   const ODPRFOPOut_t* pFOPOutput,
                                   FOHAccObjPreProcessing_t* pAccObjPreProcess,
                                   ODPRFOHDebug_t* pFOHDebug);
STATIc float32 FOHTransFilter(const float32 fInputValue,
                              const FOHObjTranControl_t* pObjTranControl,
                              float32 fTransFactorA,
                              float32* fPreCycInput,
                              float32* fPreCycInputFreeze,
                              float32* fPreCycInputCorr,
                              boolean* bPreCycRSFlipFlop);
STATIc FOHTgtObjPFOutput_t
FOHTgtObjCtdGeneration(const FOHHistoryControl_t* pHistoryControl,
                       const FOHEgoMotion_t* pEgoMotion,
                       const FOHTgtObjData_t* pTgtObjData,
                       const ODPRInSystemPara_t* pSystemPara,
                       ODPRFOHOut_t* pFOHOutput,
                       FOHTgtObjCtdInfoAndCoeff_t* pTgtObjCtdInfoAndCoeff,
                       ODPRFOHDebug_t* pFOHDebug);
STATIc FOHPolyfitSelec_t
FOHPolyfitSelection(const ODPRInVEDVehDyn_t* pEgoVEDData,
                    const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                    ODPRFOHDebug_t* pFOHDebug);
STATIc FOHLSMTgtTrajProcess_t
FOHLSMTgtTrajProcess(const FOHEgoMotion_t* pEgoMotion,
                     const FOHTgtObjData_t* pTgtObjData,
                     ODPRFOHDebug_t* pFOHDebug);
// STATIc uint8 FOHBasicLimitUint8(uint8 uiInput, uint8 uiLowLimit, uint8
// uiHighLimit);
STATIc void FOHTrajAttributes(
    const FOHAccObjStatusChk_t* pAccObjChk,
    const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
    const ODPRInVEDVehDyn_t* pEgoVEDData,
    const FOHPolyfitSelec_t* pPolyfitSelec,
    const FOHTgtObjCtdInfoAndCoeff_t* pTgtObjCtdInfoAndCoeff,
    ODPRFOHOut_t* pFOHOutput,
    ODPRFOHDebug_t* pFOHDebug);
STATIc FOHStrgtEstFadCrvLimit_t
FOHStrgtEstimFadingCrvLimit(const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                            const FOHPolyfitSelec_t* pPolyfitSelec,
                            const ODPRFOHOut_t* pFOHOutput,
                            ODPRFOHDebug_t* pFOHDebug);

static void reset_laneKF(TUE_CML_sMatrix_t* x_laneKFLe,
                         TUE_CML_sMatrix_t* P_laneKFLe);
static void predict_laneKF(float32 dT_laneKFLe,
                           float32 dX_laneKFLe,
                           float32 vehYawRate,
                           TUE_CML_sMatrix_t* x_laneKFLe,
                           TUE_CML_sMatrix_t* P_laneKFLe,
                           float32 P_ABPLBP_LaneKFDynDistYFact_nu,
                           float32 P_ABPLBP_LaneKFDynYawFactor_nu);
static void init_laneKF(const TUE_CML_sMatrix_t* z_laneKFLe,
                        TUE_CML_sMatrix_t* R_laneKFLe,
                        float32 quality,
                        TUE_CML_sMatrix_t* x_laneKFLe,
                        TUE_CML_sMatrix_t* P_laneKFLe,
                        uint8 P_ABPLBP_LaneKFMnInitQual_perc,
                        float32 P_ABPLBP_LaneKFInitRFactor_nu);
static void update_laneKF(const TUE_CML_sMatrix_t* z_laneKFLe,
                          const TUE_CML_sMatrix_t* R_laneKFLe,
                          float32 weight,
                          TUE_CML_sMatrix_t* x_laneKFLe,
                          TUE_CML_sMatrix_t* P_laneKFLe,
                          float32 P_ABPLBP_LaneKFKGainFac_nu);
static void maintenance_laneKF(TUE_CML_sMatrix_t* x_laneKFLe,
                               TUE_CML_sMatrix_t* P_laneKFLe,
                               float32 P_ABPLBP_LaneKFIncQual_1ps,
                               float32 P_ABPLBP_LaneKFDecQualDeg_1ps,
                               float32 P_ABPLBP_LaneKFDecQualPred_1ps,
                               float32 deltaT_sec);
void laneKalmanFilter(const laneKFInTypeV3* inputs, laneKFOutType* outputs);
STATIc FOHLowPassFilter_t
FOHLowPassFilter(const ODPRInVEDVehDyn_t* pEgoVEDData,
                 const FOHStrgtEstFadCrvLimit_t* pStrgtEstFadCrvLimit,
                 const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                 const ODPRInSystemPara_t* pSystemPara,
                 ODPRFOHDebug_t* pFOHDebug);
STATIc float32 FOHCosineTransition(const float32 fInputValue,
                                   const FOHModeTranControl_t* pModeTranControl,
                                   float32* fPreCycInput,
                                   float32* fPreCycInputFreeze,
                                   float32* fPreCycInputCorr,
                                   boolean* bPreCycRSFlipFlop);
STATIc FOHModeTranControl_t
FOHTgtTrajTransition(const ODPRInSystemPara_t* pSystemPara,
                     const FOHLowPassFilter_t* pLowPassFilter,
                     const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                     const FOHLSMTgtTrajProcess_t* pLSMTgtTrajProcess,
                     const FOHAccObjStatusChk_t* pAccObjChk,
                     ODPRFOHOut_t* pFOHOutput,
                     ODPRFOHDebug_t* pFOHDebug);
STATIc void FOHMinObjPosX(const ODPRInVEDVehDyn_t* pEgoVEDData,
                          ODPRFOHOut_t* pFOHOutput);
STATIc void FOHInvalidBitField(const FOHAccObjStatusChk_t* pAccObjChk,
                               const FOHObjTranControl_t* pObjTranControl,
                               const FOHTgtObjPFOutput_t* pTgtObjPFOutput,
                               const FOHHistoryControl_t* pHistoryControl,
                               const FOHModeTranControl_t* pModeTranControl,
                               ODPRFOHOut_t* pFOHOutput);

#endif