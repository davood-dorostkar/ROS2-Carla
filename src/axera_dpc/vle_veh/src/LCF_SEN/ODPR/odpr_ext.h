#ifndef ODPR_H
#define ODPR_H
#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"
#include "odpr_cfg.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef struct {
    boolean bObjectDetected_bool;  // Flag that Acc object is detected
                                   // Range:[0~1]
    float32 fObjRelAclX_mps2;  // The relative longitudial acceleration of Acc
                               // object			Range:[]
    float32 fObjRelVelX_mps;   // The relative longitudial velocity X of Acc
                               // object				Range:[]
    float32 fObjRelAclY_mps2;  // The relative lateral acceleration Y of Acc
                               // object				Range:[]
    float32 fObjRelVelY_mps;   // The relative lateral velocity Y of Acc object
                               // Range:[]
    float32 fObjPosX_met;      // The longitudial position of Acc object
                               // Range:[]
    float32 fObjPosY_met;      // The lateral position of Acc object
                               // Range:[]
    float32 fObjPosXStdDev_met;  // The longitudial position standard deviation
                                 // of Acc object     Range:[]
    float32 fObjPosYStdDev_met;  // The lateral position standard deviation of
                                 // Acc object         Range:[]
    uint8 uiObjQuality_perc;     // The quality of Acc object Range:[0~100]
    uint8 uiObjClassType_nu;     // The class type of Acc object
    // Range:{ACC_OC_CAR=1,ACC_OC_TRUCK,ACC_OC_PEDESTRIAN}
    uint8 uiObjMeasState_nu;          // The measurement state of Acc object
                                      // Range:{STATE_DELETED=0,STATE_NEW,STATE_MEASURED,STATE_PREDICTED,STATE_INACTIVE,STATE_MAX_DIFF_TYPES}
    float32 fObjWidth_met;            // The width of Acc object
                                      // Range:[]
    uint16 uiObjSensorSource_btf;     // The bitfield of sensor source of Acc
                                      // object
                                      // Range:[]
    uint32 uiObjTimeStamp_usec;       // The time stamp of Acc object
                                      // Range:[]
    sint8 uiObjID_nu;                 // The ID of Acc object
                                      // Range:[]
    float32 fObjRelHeadingAngle_rad;  // The relative heading angle of Acc
                                      // object
                                      // Range:[]
} ODPRInAccFRObj_t;

typedef struct {
    float32 fSystemCylceTime_sec;  // System cycle time
                                   // Range:[]
} ODPRInSystemPara_t;

typedef struct {
    float32 fEgoVelX_mps;     // The longitudial velocity X of ego vehicle
                              // Range:[]
    float32 fEgoCurve_1pm;    // The curvature of ego vehicle
                              // Range:[]
    float32 fEgoYawRate_rps;  // The yaw rate of ego vehicle
                              // Range:[]
} ODPRInVEDVehDyn_t;

typedef struct {
    boolean bLaneChangeDetected_bool;      // Flag that lane change is detected
                                           // Range:[0~1]
    uint16 uiLeftLaneInvalidCheck_btf;     // Bitfield of left lane invalid
                                           // qualifier
                                           // Range:[]
    uint8 uiLeftLaneQuality_perc;          // Quality of left lane
                                           // Range:[0~100]
    float32 fLeftLaneClothoidPosY0_met;    // Initial lateral position PosY0 of
                                           // left lane clothoid		Range:[]
    float32 fLeftLaneClothoidHeading_rad;  // Heading angle of left lane
                                           // clothoid
                                           // Range:[]
    float32 fLeftLaneClothoidCurve_1pm;    // Curve of left lane clothoid
                                           // Range:[]
    float32 fLeftLaneClothoidCurveDer_1pm2;  // Curve derivative of left lane
                                             // clothoid
                                             // Range:[]
    float32 fLeftLaneClothoidLength_met;     // Length of left lane clothoid
                                             // Range:[]
    uint16 uiRightLaneInvalidCheck_btf;      // Bitfield of right lane invalid
                                             // qualifier
                                             // Range:[]
    uint8 uiRightLaneQuality_perc;           // Quality of right lane
                                             // Range:[0~100]
    float32 fRightLaneClothoidPosY0_met;    // Initial lateral position PosY0 of
                                            // right lane clothoid
                                            // Range:[]
    float32 fRightLaneClothoidHeading_rad;  // Heading angle of right lane
                                            // clothoid
                                            // Range:[]
    float32 fRightLaneClothoidCurve_1pm;    // Curve of right lane clothoid
                                            // Range:[]
    float32 fRightLaneClothoidCurveDer_1pm2;  // Curve derivative of right lane
                                              // clothoid
                                              // Range:[]
    float32 fRightLaneClothoidLength_met;     // Length of right lane clothoid
                                              // Range:[]
} ODPRInABPRLaneInfo_t;

typedef struct {
    uint8 uiExistProb_nu;          // Probability of object exists
                                   // Range:[0~100]
    float32 fRelVelX_mps;          // Relative longitudial velocity X of object
                                   // Range:[]
    float32 fRelVelY_mps;          // Relative lateral velocity Y of object
                                   // Range:[]
    uint8 uiID_nu;                 // Object ID
                                   // Range:[]
    float32 fPosX_met;             // Longitudial position X of object
                                   // Range:[]
    float32 fPosY_met;             // Lateral position Y of object
                                   // Range:[]
    float32 fShapePoint0PosX_met;  // Longitudial position X of shape point 0 of
                                   // object 		Range:[]
    float32 fShapePoint0PosY_met;  // Lateral position Y of shape point 0 of
                                   // object				Range:[]
    uint8 uiShapePoint0State_nu;   // State of of shape point 0 of object
    // Range:{INVALID=0,EDGE_MEASURED,LANEDGE_ASSUMEDE_LEFT,VISIBILITY_EDGE}
    float32 fShapePoint1PosX_met;  // Longitudial position X of shape point 1 of
                                   // object 		Range:[]
    float32 fShapePoint1PosY_met;  // Lateral position Y of shape point 1 of
                                   // object   			Range:[]
    uint8 uiShapePoint1State_nu;   // State of of shape point 1 of object
    // Range:{INVALID=0,EDGE_MEASURED,LANEDGE_ASSUMEDE_LEFT,VISIBILITY_EDGE}
    float32 fShapePoint2PosX_met;  // Longitudial position X of shape point 2 of
                                   // object 		Range:[]
    float32 fShapePoint2PosY_met;  // Lateral position Y of shape point 2 of
                                   // object				Range:[]
    uint8 uiShapePoint2State_nu;   // State of of shape point 2 of object
    // Range:{INVALID=0,EDGE_MEASURED,LANEDGE_ASSUMEDE_LEFT,VISIBILITY_EDGE}
    float32 fShapePoint3PosX_met;  // Longitudial position X of shape point 3 of
                                   // object 		Range:[]
    float32 fShapePoint3PosY_met;  // Lateral position Y of shape point 3 of
                                   // object   			Range:[]
    uint8 uiShapePoint3State_nu;   // State of of shape point 3 of object
    // Range:{INVALID=0,EDGE_MEASURED,LANEDGE_ASSUMEDE_LEFT,VISIBILITY_EDGE}
} ODPRInSSRObject_t;

typedef struct {
    ODPRInSSRObject_t ObjectArray[ILE_INPUT_RADAR_OBJECT_NUM];  // Object array
    uint8 uiNumOfObject;  // Number of objects from SSR
} ODPRInSSRObjList_t;

typedef struct {
    ODPRInAccFRObj_t sAccObject;     // Structure of Acc object
    ODPRInVEDVehDyn_t sEgoVehSig;    // Structure of ego vehicle
    ODPRInSystemPara_t sSystemPara;  // Structure of system parameter
    ODPRInABPRLaneInfo_t sLaneData;  // Structure of lane data
    ODPRInSSRObjList_t sObjectList;  // Structure of SRR object list
} ODPRInReq_t;

typedef struct {
    float32 fAccObjRelAclX_mps2;  // Relative longitudial acceleration X of Acc
                                  // object
                                  // Range:[]
    float32 fAccObjRelVelX_mps;   // Relative longitudial velocity X of Acc
                                  // object
                                  // Range:[]
    float32 fAccObjRelAclY_mps2;  // Relative lateral acceleration Y of Acc
                                  // object
                                  // Range:[]
    float32 fAccObjRelVelY_mps;   // Relative lateral velocity Y of Acc object
                                  // Range:[]
    float32 fAccObjPosX_met;      // Longitudial position X of Acc object
                                  // Range:[]
    float32 fAccObjPosY_met;      // Lateral position X of Acc object
                                  // Range:[]
    float32 fAccObjPosXStdDev_met;    // The longitudial position standard
                                      // deviation of Acc object
                                      // Range:[]
    float32 fAccObjPosYStdDev_met;    // The lateral position standard deviation
                                      // of Acc object
                                      // Range:[]
    uint16 uiAccObjInvalidCheck_btf;  // Bitfield of Acc object invalid check
    /*Bit0: Flag that Acc object is not detected
      Range:[0~1]
      Bit1: Flag that quality of Acc object is invalid
      Range:[0~1]
      Bit2: Flag that measurement state of Acc object is invalid
      Range:[0~1]
      Bit3: Flag that lateral position Y of Acc object is invalid
      Range:[0~1]
      Bit4: Flag that longitudinal position X of Acc object is invalid
      Range:[0~1]
      Bit5: Flag that width of Acc object is invalid
      Range:[0~1]
      Bit6:	Flag that realtive longitudinal velocity X of Acc object is
      invalid
      Range:[0~1]
      Bit7: Flag that realtive lateral velocity Y of Acc object is invalid
      Range:[0~1]
      Bit8:	Flag that realtive longitudinal acceleration X of Acc object is
      invalid		Range:[0~1]
      Bit9:	Flag that type of Acc object is invalid
      Range:[0~1]
      Bit10:Flag that fusion state of Acc object is invalid
      Range:[0~1]
      Bit11:Flag that ID switch of Acc object is detected
      Range:[0~1]
      Bit12:Flag that lateral moving of Acc object is invalid
      Range:[0~1]
      Bit13~15: Reserved(0 by default)*/
    float32 fAccObjTimeStamp_sec;  // Timestamp of Acc object
                                   // Range:[]
    float32 fEstimateObjPosX_met;  // Estimated longitudinal position X of Acc
                                   // object
                                   // Range:[]
    float32 fEstimateObjPosY_met;  // Estimated lateral position X of Acc object
                                   // Range:[]
    float32 fAccObjRelHeadingAngle_rad;  // Relative heading angle of Acc object
                                         // Range:[]
} ODPRFOPOut_t;

typedef struct {
    float32 fTgtObjPosX0_met;       // Initial longitudial position X of target
                                    // trajectory
                                    // Range:[]
    float32 fTgtObjPosY0_met;       // Initial lateral position Y of target
                                    // trajectory
                                    // Range:[]
    float32 fTgtObjHeading_rad;     // Heading angle of target trajectory
                                    // Range:[]
    float32 fTgtObjCurve_1pm;       // Curve of target trajectory
                                    // Range:[]
    float32 fTgtObjCurveDer_1pm2;   // Curve derivative of target trajectory
                                    // Range:[]
    float32 fTgtObjLength_met;      // Length of target trajectory
                                    // Range:[]
    float32 fAccObjTraceCurve_1pm;  // Trace curve of acc object(Not used, 0 by
                                    // default)
                                    // Range:[]
    uint8 uiAccObjTraceCurveQuality_perc;    // Trace curve quality of acc
                                             // object(Not used, 0 by default)
                                             // Range:[0~100]
    uint16 fTgtObjClothoidInvalidCheck_btf;  // Bitfield of target trajectory
                                             // invalid check
    /*Bit0: Flag that Acc object is invalid as new sample
      Range:[0~1]
      Bit1: Flag that Acc object position freeze stop
      Range:[0~1]
      Bit2: Flag that Object valid transition is ongoing
      Range:[0~1]
      Bit3: Flag that cut in transition is ongoing
      Range:[0~1]
      Bit4: Flag that cut out transition is ongoing
      Range:[0~1]
      Bit5: Flag that freeze stop transition is ongoing
      Range:[0~1]
      Bit6:	Flag that target trajectory is not updated
      Range:[0~1]
      Bit7: Flag that target trajectory is invalid
      Range:[0~1]
      Bit8:	Flag that history mode is inactive
      Range:[0~1]
      Bit9:	Flag that low speed mode is inactive
      Range:[0~1]
      Bit10:Flag that low speed mode transition is ongoing
      Range:[0~1]
      Bit11:(Unused, 0 by default)
      Range:[0~1]
      Bit12:Flag that history is disabled
      Range:[0~1]
      Bit13:Flag that history is reset
      Range:[0~1]
      Bit14:Flag that object history is invalid
      Range:[0~1]
      Bit15:Reserved(0 by default)*/
    uint8 uiAccObjTraceQuality_perc;   // Trace quality of Acc object
                                       // Range:[0~100]
    float32 fLastStoredPointX_met;     // Last stored longitudinal position X
                                       // Range:[]
    float32 fLastStoredPointY_met;     // Last stored lateral position Y
                                       // Range:[]
    float32 fMaxGapEgoToHistory_met;   // Maximum gap of ego to history
                                       // Range:[]
    float32 fMeanDevToTraj_1st_met;    // Mean deviation to 1st order trajectory
                                       // polyfit
                                       // Range:[]
    float32 fMeanDevToTraj_3rd_met;    // Mean deviation to 3rd order trajectory
                                       // polyfit
                                       // Range:[]
    float32 fMinHistoryLength_met;     // Minimum history length
                                       // Range:[]
    float32 fMinHistoryStartPosX_met;  // Minimum history start longitudinal
                                       // position X
                                       // Range:[]
    uint8 uiNumOfValidSamples_nu;      // Number of valid sample points
                                       // Range:[0~32]
    boolean bAddNewSample_bool;        // Flag that add new sample
                                       // Range:[0~1]
    uint8 uiObjTraceStraightProb_perc;  // Probability that object trace going
                                        // straight
                                        // Range:[0~100]
    float32 fLastStoredPointAge_sec;    // Age of last stored sample point
                                        // Range:[]
    float32 fFirstStoredPointAge_sec;   // Age of first stored sample point
                                        // Range:[]
    float32 fFirstStoredPointX_met;   // Longitudinal position X of first stored
                                      // sample point
                                      // Range:[]
    float32 fMeanStoredPointAge_sec;  // Mean age of all stored sample points
                                      // Range:[]
} ODPRFOHOut_t;

typedef struct {
    uint8 uiLeftLaneObjID_nu;      // ID of selected object in left lane
                                   // Range:[0~30]
    float32 fLeftLaneObjPosX_met;  // Longitudinal position X of selected object
                                   // in left lane
                                   // Range:[]
    float32 fLeftLaneObjPosY_met;  // Lateral position Y of selected object in
                                   // left lane
                                   // Range:[]
    float32 fLeftLaneObjRelVelX_mps;  // Relative longitudinal velocity X of
                                      // selected object in left lane
                                      // Range:[]
    float32 fLeftLaneObjRelVelY_mps;  // Relative lateral velocity Y of selected
                                      // object in left lane
                                      // Range:[]
    uint8 uiLeftLaneObjExistProb_perc;  // Probability of object exists in left
                                        // lane
                                        // Range:[0~100]
    uint8 uiLeftLaneObjDriveOutLeftLane_perc;  // Probability of object drives
                                               // in out-left lane
                                               // Range:[0~100]
    uint8 uiLeftLaneObjDriveLeftLane_perc;  // Probability of object drives in
                                            // left lane
                                            // Range:[0~100]
    uint8 uiLeftLaneObjDriveEgoLane_perc;   // Probability of object drives in
                                            // ego lane
                                            // Range:[0~100]

    uint8 uiEgoLaneObjID_nu;      // ID of selected object in ego lane
                                  // Range:[0~30]
    float32 fEgoLaneObjPosX_met;  // Longitudinal position X of selected object
                                  // in ego lane
                                  // Range:[]
    float32 fEgoLaneObjPosY_met;  // Lateral position Y of selected object in
                                  // ego lane
                                  // Range:[]
    float32 fEgoLaneObjRelVelX_mps;  // Relative longitudinal velocity X of
                                     // selected object in ego lane
                                     // Range:[]
    float32 fEgoLaneObjRelVelY_mps;  // Relative lateral velocity Y of selected
                                     // object in ego lane
                                     // Range:[]
    uint8 uiEgoLaneObjExistProb_perc;  // Probability of object exists in ego
                                       // lane
                                       // Range:[0~100]
    uint8 uiEgoLaneObjDriveLeftLane_perc;  // Probability of object drives in
                                           // left lane
                                           // Range:[0~100]
    uint8 uiEgoLaneObjDriveEgoLane_perc;  // Probability of object drives in ego
                                          // lane
                                          // Range:[0~100]
    uint8 uiEgoLaneObjDriveRightLane_perc;  // Probability of object drives in
                                            // right lane
                                            // Range:[0~100]

    uint8 uiRightLaneObjID_nu;      // ID of selected object in right lane
                                    // Range:[0~30]
    float32 fRightLaneObjPosX_met;  // Longitudinal position X of selected
                                    // object in right lane
                                    // Range:[]
    float32 fRightLaneObjPosY_met;  // Lateral position Y of selected object in
                                    // right lane
                                    // Range:[]
    float32 fRightLaneObjRelVelX_mps;    // Relative longitudinal velocity X of
                                         // selected object in right lane
                                         // Range:[]
    float32 fRightLaneObjRelVelY_mps;    // Relative lateral velocity Y of
                                         // selected object in right lane
                                         // Range:[]
    uint8 uiRightLaneObjExistProb_perc;  // Probability of object exists in
                                         // right lane
                                         // Range:[0~100]
    uint8 uiRightLaneObjDriveEgoLane_perc;    // Probability of object drives in
                                              // ego lane
                                              // Range:[0~100]
    uint8 uiRightLaneObjDriveRightLane_perc;  // Probability of object drives in
                                              // right lane
                                              // Range:[0~100]
    uint8 uiRightLaneObjDriveOutRightLane_perc;  // Probability of object drives
                                                 // in out-right lane
                                                 // Range:[0~100]
} ODPRILEOut_t;

typedef struct {
    ODPRFOPOut_t sFOPOutData;  // Structure of FOP output
    ODPRFOHOut_t sFOHOutData;  // Structure of FOH output
    ODPRILEOut_t sILEOutData;  // Structure of ILE output
} ODPROutPro_t;

typedef struct {
    float32 fVehOverhangFront_met;  // Front overhang for transformation from
                                    // radar to front axle coordinate system
    float32 fVehWheelBase_met;      // Vehicle wheelbase
} ODPRParam_t;

#ifndef Rte_TypeDef_ODPRFOPDebug_t
#define Rte_TypeDef_ODPRFOPDebug_t
typedef struct {
    /* FOPChkPossHysteresis */
    boolean
        bObjHysteresisAllowed_bool;  // Flag that object hysteresis is allowed
    boolean bNoObjIDSwitchDetected_bool;  // Flag that no object ID switch is
                                          // detected
    boolean bObjPosYStepDetected_bool;  // Flag that position y step is detected
    boolean bObjPosXStepDetected_bool;  // Flag that position x step is detected
    boolean bResetObjValidity_bool;     // Flag that reset object validity
    float32 fTempAccObjPosX_met;
    float32 fTempObjPosY_met;
    boolean bObjPosXChk_bool;
    boolean bObjPosYChk_bool;
    boolean bNoIDSwitch_bool;
    /* FOPValidateACCObject */
    boolean bACCObjDetected_bool;   // Flag that ACC object is detected
    boolean bObjQualityValid_bool;  // Flag that object quality is valid
    boolean
        bObjMeasStateValid_bool;  // Flag that object measurement state is valid
    boolean bObjPosYValid_bool;   // Flag that object position Y is valid
    boolean bObjPosXValid_bool;   // Flag that object position X is valid
    boolean bObjWidthValid_bool;  // Flag that object width is valid
    boolean
        bObjRelVelXValid_bool;  // Flag that X-axis relative velocity is valid
    boolean
        bObjRelVelYValid_bool;  // Flag that Y-axis relative velocity is valid
    boolean bObjRelAclXValid_bool;  // Flag that X-axis relative acceleration is
                                    // valid
    boolean bObjTypeValid_bool;     // Flag that object type is valid
    boolean bObjFusionStateValid_bool;  // Flag that object fusion state is
                                        // valid
    boolean bObjLatMovInvalid_bool;     // Flag that object latent moving is
                                        // invalid
    boolean
        bDistYOffsetChk_bool;   // Intermediate signal used in position Y check
    float32 fMinObjPosY_met;    // Intermediate signal used in position Y check
    boolean bFusionReset_bool;  // Intermediate signal used in fusion state
                                // check
    boolean bMeasStReset_bool;  // Intermediate signal used in Measurement state
                                // check
    boolean bObjAndEgoStop_bool;  // Intermediate signal used in latent moving
                                  // state check
    boolean bBothStop_bool;  // Intermediate signal used in latent moving state
                             // check
    boolean bLatMovReset_bool;  // Intermediate signal used in latent moving
                                // state check
    float32 fOffsetObjPosY_met;
    float32 fMinCurvature_1pm;
    float32 fDistYOffset_met;
    float32 fTempObjPosX_met;
    float32 fTempLongDistMax_met;
    boolean bTempPosXHyst_bool;
    float32 fMinWidthTgtObj_met;
    float32 fMaxLongVel_met;
    float32 fMaxLatVel_met;
    float32 fMaxLongAcl_met;
    boolean bTempObjTypeChk_bool;
    boolean bTempObjTypeChkWithHyst_bool;
    boolean bTempFusionTurnOn_bool;
    boolean bTempMeasStTurnOn_bool;
    float32 fTempObjVelX_mps;
    boolean bTempObjStandstill_bool;
    boolean bTempEgoStandstill_bool;
    /* FOPValidACCObjPostProcess */
    boolean bPreCycObjDetected_bool;  // Flag that object is detected in
                                      // previous cycle
    /* FOPObjYawAngEstimation */
    float32 fTempPosX_met;             // Own calculated PosX
    boolean bTempReset_bool;           // Reset flag
    boolean bTempEnableCalc_bool;      // Flag that enable calc
    float32 fPosYLowPass_met;          // Own calculated posY
    float32 fTempVehYawAng_rad;        // Own calculated Veh yaw angle
    boolean bYawAngEnableSample_bool;  // Own calculated yaw angle enable sample
                                       // flag
    float32 fOwnCalculatedYawAng_rad;  // Own calculated yaw angle
    float32
        fTempYawAngleCalc_rad;  // Intermediate signal used in Enabled_subsystem
    boolean bTempResetObjChng_bool;  // Flag that ResetObjChng
    float32 fTempPosYIn_met;
    float32 fTempYawAngle_rad;
} ODPRFOPDebug_t;
#endif

#ifndef Rte_TypeDef_ODPRFOHDebug_t
#define Rte_TypeDef_ODPRFOHDebug_t
typedef struct {
    /* FOHEgoMotionCalc */
    float32 fYawRateObjSync_rps;  // Yaw rate after syncronization
    /* FOHAccObjStatusChk */
    boolean bAccObjValid_bool;       // Flag that acc object is valid
    boolean bAccObjFreezeStop_bool;  // Flag that acc object freeze stop
    boolean bObjCutOut_bool;         // Flag that acc object cut out
    boolean bObjCutIn_bool;          // Flag that acc object cut in
    boolean bLSMInactive_bool;       // Flag that low speed mode inactive
    boolean bTrajInvalid_bool;       // Flag that trajectory invalid
    boolean bTempAccObjDtctChk_bool;
    boolean bTempPreHistValidChk_bool;
    boolean bTempPreHistValidTurnOnDelay_bool;
    float32 fTempMaxObjDistLSM_met;
    boolean bTempAccObjPosXHyst_bool;
    /* FOHHistoryControl */
    boolean bSaveNewEntry_bool;   // Flag that save new netry
    boolean bResetHistory_bool;   // Flag that reset history
    boolean bEnableHistory_bool;  // Flag that enable history
    float32 fTempPosXDiff_met;
    float32 fTempPosXFromPos_met;
    float32 fTempPosXFromVel_met;
    boolean bTempPosXFromPosChk_bool;
    boolean bTempPosXFromVelChk_bool;
    boolean bTempPosXChk_bool;
    /* FOHAccObjPreProcessing */
    boolean bTransitionEnable_bool;     // Flag that enable transition
    boolean bTransitionReset_bool;      // Flag that reset transition
    boolean bCutInOngoing_bool;         // Flag that CutIn is ongoing
    boolean bCutOutOngoing_bool;        // Flag that CutOut is ongoing
    boolean bFreezeStopOnging_bool;     // Flag that reezeStop is ongoing
    boolean bObjValidOngoing_bool;      // Flag that ObjValid is ongoing
    float32 fTransitionFactorA_fac;     // Transition Factor A
    float32 fAccObjPosYTransPF_met;     // AccObjPosYTransPF which used in
                                        // FOHTgtObjCtdGeneration
    float32 fAccObjPosYTrans_met;       // AccObjPosYTrans which used in
                                        // FOHLSMTgtTrajProcess
    float32 fAccObjRelYawAngTrans_rad;  // AccObjRelYawAngTrans which used in
                                        // FOHLSMTgtTrajProcess
    boolean bCutInDetected_bool;
    boolean bCutOutDetected_bool;
    boolean bAccObjFreezeStop2_bool;
    boolean bObjValidity_bool;
    boolean bAccObjValidRisingChk_bool;
    float32 fTempPosYCorr_met;
    float32 fTempPosYCorrBef_met;
    float32 fTempPosYCorrAft_met;
    float32 fTempPosYPT1PF_met;
    float32 fTempPosYPT1LSM_met;
    boolean bTempTransAll_bool;
    boolean bTempTransPF_bool;
    float32 fTempFactorA_fct;
    float32 fTempTransPosYPT1PF_met;
    float32 fTempTransPosYPT1LSM_met;
    float32 fTempTransYawAng_met;
    float32 fTempPosYPT1LowpassT_sec;
    float32 fTempPosYLSMLowpassT_sec;
    /* FOHTgtObjCtdGeneration */
    boolean bTrajInvalid1st_bool;  // Flag that 1st trajectory polyfit is valid
    boolean bTrajInvalid3rd_bool;  // Flag that 3rd trajectory polyfit is valid
    float32 fPosY0_1st_met;        // PosY0 in 1st trajectory polyfit
    float32 fPosY0_3rd_met;        // PosY0 in 3rd trajectory polyfit
    float32 fHeading_1st_rad;      // Heading in 1st trajectory polyfit
    float32 fHeading_3rd_rad;      // Heading in 3rd trajectory polyfit
    float32 fCrv_1pm;              // Curve in 3rd trajectory polyfit
    float32 fTempFeatPtsMinPosX_met;
    float32 fTempAlpha_fct;
    float32 fTempYawRate_rps;
    float32 fTempYawRateOut_rps;
    /* FOHPolyfitSelection */
    float32 fPolySelecWeight_fac;       // Weight factor in PolyfitSelection
    float32 fPolySelecPosY0_3rd_met;    // PosY0 after PolyfitSelection
    float32 fPolySelecHeading_3rd_rad;  // Heading after PolyfitSelection
    float32 fPolySelecCrv_1pm;          // Curve after PolyfitSelection
    /* FOHTrajAttributes */
    float32 fStrightProb1_perc;  // Stright Probability 1
    float32 fStrightProb2_perc;  // Stright Probability 2
    float32 fStrightProb3_perc;  // Stright Probability 3
    float32 fTraceQual1_perc;    // Trace Quality 1
    float32 fTraceQual2_perc;    // Trace Quality 2
    float32 fTraceQual3_perc;    // Trace Quality 3
    float32 fTraceQual4_perc;    // Trace Quality 4
    float32 fTraceQual5_perc;    // Trace Quality 5
    float32 fTempEgoPosX_met;
    float32 fTempPosYCrv_met;
    float32 fTempPosYAbs_met;
    uint8 uiTempStrightProb_perc;
    float32 fTempMeanSampleAge_sec;
    float32 fTempMeanDevToTraj_3rd_met;
    float32 fTempNumValidSamples_nu;
    float32 fTempVelXMax_mps;
    float32 fTempHistLength_met;
    float32 fTempTrajLength_met;
    uint8 uiTempTraceQual_perc;
    /* FOHStrgtEstimFadingCrvLimit */
    float32 fStrgtFadCurvature_1pm;  // Curve after StrgtEstimFading
    float32 fStrgtFadPosY0_met;      // PosY0 after StrgtEstimFading
    float32 fStrgtFadHeading_rad;    // Heading after StrgtEstimFading
    float32 fTempFading_fac;
    /* FOHLowPassFilter */
    boolean bLowPassResetFlg_bool;   // Flag that Reset Lowpass filter
    boolean bLowPassEnableFlg_bool;  // Flag that Enable Lowpass filter
    float32 fLowPassPosY0_met;       // PosY0 after LowPassFilter
    float32 fLowPassHeading_rad;     // Heading after LowPassFilter
    float32 fLowPassCurvature_1pm;   // Curve after LowPassFilter
    float32 fLowPassChngOfCrv_1pm2;  // Change of Curve after LowPassFilter
    boolean bTempTrajInvalidChk_bool;
    float32 fTempVehVelX_kph;
    float32 fTempPosYLowpassT_sec;
    float32 fTempHeadingLowpassT_sec;
    float32 fTempCurveLowpassT_sec;
    float32 fTempCrvChngLowpassT_sec;
    /* FOHLSMTgtTrajProcess */
    float32 fLengthLSM_met;  // Length in LSM mode
    float32 fPosY0LSM_met;   // PosY0 in LSM mode
    float32 fHeadLSM_rad;    // Heading in LSM mode
    float32 fCrvLSM_1pm;     // Curve in LSM mode
    float32 fTempC0Calculate_lpm;
    float32 fTempMaxCrv_lpm;
    /* FOHTgtTrajTransition */
    boolean bModeTransitionEnable_bool;  // Flag that enable mode transition
    boolean bModeTransitionReset_bool;   // Flag that reset mode transition
    boolean bModeLSMTransOngoing_bool;   // Flag that LSMTrans is ongoing
    float32 fModeTransitionValueA_fac;   // Factor A in mode transition
    float32 fTempPosY0_met;
    float32 fTempHeading_rad;
    float32 fTempTrajLength2_met;
    float32 fTempCrv_lpm;
    boolean bTempLSMChangeFlag_bool;
    boolean bTempLSMIvdEdge_bool;
    /* FOHPolyfitTgtObjClothoid */
    float32 vecY[SAMPLE_POINTS * 1];
    float32 vecX[SAMPLE_POINTS * 1];
    float32 Weight[SAMPLE_POINTS * 1];
    float32 vecAge[SAMPLE_POINTS * 1];
    float32 fMaxSampleAge_sec;
    boolean bPolyfitEn_bool;
    boolean bStartPointValid_bool;
    boolean bLengthValid_bool;
} ODPRFOHDebug_t;
#endif

#ifndef Rte_TypeDef_ODPRILEDebug_t
#define Rte_TypeDef_ODPRILEDebug_t
typedef struct {
    /* ILEDetermineShPt */
    boolean bObjShPtValidChk_bool[ILE_INPUT_RADAR_OBJECT_NUM];  // Flag that
                                                                // object shape
                                                                // points are
                                                                // valid
    /* ILEObjInLaneEval */
    boolean bTempLeftLaneValid_bool;    // Flag that left lane is valid
    boolean bTempRightLaneValid_bool;   // Flag that right lane is valid
    boolean bTempLeftLaneValid2_bool;   // Flag that left lane2 is valid
    boolean bTempRightLaneValid2_bool;  // Flag that right lane2 is valid
    uint8 uiObjBelongToLane_nu[ILE_INPUT_RADAR_OBJECT_NUM];  // Object belong to
                                                             // lane
} ODPRILEDebug_t;
#endif

#ifndef Rte_TypeDef_ODPRDebug_t
#define Rte_TypeDef_ODPRDebug_t
typedef struct {
    ODPRFOPDebug_t sFOPDebug;
    ODPRFOHDebug_t sFOHDebug;
    ODPRILEDebug_t sILEDebug;
} ODPRDebug_t;
#endif

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
extern void LCF_ODPR_Reset(void);
extern void LCF_ODPR_Exec(const ODPRInReq_t* reqPorts,
                          const ODPRParam_t* param,
                          ODPROutPro_t* proPorts,
                          ODPRDebug_t* debug);
#ifdef __cplusplus
}
#endif
#endif
