#ifndef LCF_SEN_LOCAL_EXT_H
#define LCF_SEN_LOCAL_EXT_H
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <string.h>
#include "TM_Global_Types.h"
#include "VSDP_Ext.h"
#include "odpr_ext.h"
#include "LBP_Ext.h"
#include "lcf_alp_ext.h"
#include "LDWSA_ext.h"
#include "LDPSA_Ext.h"
#include "TJASA/TJASA_ext.h"
#include "MCTLFC_ext.h"
#include "LCFOPS/lcfops_ext.h"
#include "LCCRA/LCCRA_ext.h"
/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef enum {
    LCF_SEN_CAMERA_Color_White,    // 0, Color_White
    LCF_SEN_CAMERA_Color_Yellow_Orange_Red,    // 1, Color_Yellow_Orange_Red
    LCF_SEN_CAMERA_Color_Blue_Green,  // 2, Color_Blue_Green
    LCF_SEN_CAMERA_Color_Unknow,  // 3, Color_Unknow
} LCF_SenCameraLaneColorENUM;

typedef enum {
    LCF_SEN_CAMERA_LANE_LE,    // 0, left lane index
    LCF_SEN_CAMERA_LANE_RI,    // 1, right lane index
    LCF_SEN_CAMERA_LANE_AJLE,  // 2, adjacent left lane index
    LCF_SEN_CAMERA_LANE_AJRI,  // 3, adjacent right lane index
    LCF_SEN_CAMERA_LANE_NUM    // 4
} LCF_SenCameraLaneEnum;

#ifndef Rte_TypeDef_LCF_SenSignalHeader_t
typedef struct {
    uint32 uiTimeStamp;
    uint16 uiMeasurementCounter;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
} LCF_SenSignalHeader_t; /* Common header for all structured data types */
#define Rte_TypeDef_LCF_SenSignalHeader_t
#endif

#ifndef Rte_TypeDef_LCF_ODPRInAccFRObj_t
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
    uint8
        uiObjMeasState_nu;  // The measurement state of Acc object
                            // Range:{STATE_DELETED=0,STATE_NEW,STATE_MEASURED,STATE_PREDICTED,STATE_INACTIVE,STATE_MAX_DIFF_TYPES}
    float32 fObjWidth_met;  // The width of Acc object
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
} LCF_ODPRInAccFRObj_t;
#define Rte_TypeDef_LCF_ODPRInAccFRObj_t
#endif

// typedef struct {
//     uint8 uiExistProb_nu;          // Probability of object exists
//                                    // Range:[0~100]
//     float32 fRelVelX_mps;          // Relative longitudial velocity X of
//     object
//                                    // Range:[]
//     float32 fRelVelY_mps;          // Relative lateral velocity Y of object
//                                    // Range:[]
//     uint8 uiID_nu;                 // Object ID
//                                    // Range:[]
//     float32 fPosX_met;             // Longitudial position X of object
//                                    // Range:[]
//     float32 fPosY_met;             // Lateral position Y of object
//                                    // Range:[]
//     float32 fShapePoint0PosX_met;  // Longitudial position X of shape point 0
//     of
//                                    // object 		Range:[]
//     float32 fShapePoint0PosY_met;  // Lateral position Y of shape point 0 of
//                                    // object				Range:[]
//     uint8 uiShapePoint0State_nu;   // State of of shape point 0 of object
//     // Range:{INVALID=0,EDGE_MEASURED,LANEDGE_ASSUMEDE_LEFT,VISIBILITY_EDGE}
//     float32 fShapePoint1PosX_met;  // Longitudial position X of shape point 1
//     of
//                                    // object 		Range:[]
//     float32 fShapePoint1PosY_met;  // Lateral position Y of shape point 1 of
//                                    // object   			Range:[]
//     uint8 uiShapePoint1State_nu;   // State of of shape point 1 of object
//     // Range:{INVALID=0,EDGE_MEASURED,LANEDGE_ASSUMEDE_LEFT,VISIBILITY_EDGE}
//     float32 fShapePoint2PosX_met;  // Longitudial position X of shape point 2
//     of
//                                    // object 		Range:[]
//     float32 fShapePoint2PosY_met;  // Lateral position Y of shape point 2 of
//                                    // object				Range:[]
//     uint8 uiShapePoint2State_nu;   // State of of shape point 2 of object
//     // Range:{INVALID=0,EDGE_MEASURED,LANEDGE_ASSUMEDE_LEFT,VISIBILITY_EDGE}
//     float32 fShapePoint3PosX_met;  // Longitudial position X of shape point 3
//     of
//                                    // object 		Range:[]
//     float32 fShapePoint3PosY_met;  // Lateral position Y of shape point 3 of
//                                    // object   			Range:[]
//     uint8 uiShapePoint3State_nu;   // State of of shape point 3 of object
//     // Range:{INVALID=0,EDGE_MEASURED,LANEDGE_ASSUMEDE_LEFT,VISIBILITY_EDGE}
// } LCF_ODPRInSSRObject_t;

// typedef struct {
//     LCF_ODPRInSSRObject_t
//         ObjectArray[ILE_INPUT_RADAR_OBJECT_NUM];  // Object array
//     uint8 uiNumOfObject;                          // Number of objects from
//     SSR
// } LCF_ODPRInSSRObjList_t;

#ifndef Rte_TypeDef_LCF_Sen_BaseCtrlData_t
typedef struct {
    uint32 uiVersionNumber;
    LCF_SenSignalHeader_t sSigHeader;
    uint16 eOpMode;
    // uint8 eSchedulingMode;
    // uint8 eSchedulingSubMode;
    float32 fSystemSenCylceTime_sec;
    uint8 bLDWSwitchEn;
    uint8 bLDPSwitchEn;
    uint8 uLDWMode_nu;
    uint8 uLDPMode_nu;
    float32 fLDWStartupSpdHMI_kph;
    float32 fLDPStartupSpdHMI_kph;
    boolean bAEBActive_bool;
    boolean bACCActive_bool;
    uint8 uReserved1;
    uint8 uReserved2;
    uint8 uReserved3;
    uint8 uReserved4;
    uint8 uReserved5;
    float32 fReserved1;
    float32 fReserved2;
    float32 fReserved3;
    float32 fReserved4;
    float32 fReserved5;
} LCF_Sen_BaseCtrlData_t;
#define Rte_TypeDef_LCF_Sen_BaseCtrlData_t
#endif

// typedef struct {
//     LCF_SenSignalHeader_t sSigHeader;
//     LCF_ODPRInSSRObjList_t sSSRObjList;
// } LCF_SSRObjectList_t;

#ifndef Rte_TypeDef_LCF_DMCToSen_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    boolean bSysStOnLatDMC; /* System state on lateral DMC */
} LCF_DMCToSen_t;
#define Rte_TypeDef_LCF_DMCToSen_t
#endif

#ifndef Rte_TypeDef_LCF_ACCOOIData_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    LCF_ODPRInAccFRObj_t sACCOOIData;
} LCF_ACCOOIData_t;
#define Rte_TypeDef_LCF_ACCOOIData_t
#endif

#ifndef Rte_TypeDef_LCF_LaneData_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    uint8 uiCamStuatuQualifier_nu;
    float32 afPosY0_met[LCF_SEN_CAMERA_LANE_NUM]; /* Lateral distance at X = 0.0
                                                     m, (0,
                                                     -15~15, m) */
    float32
        afHeadingAngle_rad[LCF_SEN_CAMERA_LANE_NUM];  /* Heading angle (Yaw
                                                         angle) of a lane
                                                         boundary track, (0,
                                                         -0.7854~0.7854, rad) */
    float32 afCurvature_1pm[LCF_SEN_CAMERA_LANE_NUM]; /* Curvature of a lane
                                                         boundary track, (0,
                                                         -0.1~0.1, 1/m) */
    float32 afCurvatureChange_1pm2[LCF_SEN_CAMERA_LANE_NUM]; /* Curvature rate
                                                                of a lane
                                                                boundary track,
                                                                (0,
                                                                -0.001~0.001,
                                                                1/m^2) */
    float32
        afValidLength_met[LCF_SEN_CAMERA_LANE_NUM]; /* Visibility range of a
                                                       lane boundary
                                                       track, (0, 0~300, m) */
    uint8 auQuality_nu[LCF_SEN_CAMERA_LANE_NUM]; /* Quality of a lane boundary
                                                    track, (0, 0~100, -) */
    uint8 auMarkerType_nu[LCF_SEN_CAMERA_LANE_NUM]; /* Describes the type of a
                                                       lane marker (e.g.road
                                                       edge,…) ,(0, 0~100, -) */
    uint8 auEventType_nu[LCF_SEN_CAMERA_LANE_NUM];  /* Describes the event (e.g.
                                                       construction
                                                       site) ,(0, 0~8, -) */
    float32 afEventDistance_met[LCF_SEN_CAMERA_LANE_NUM]; /* Describes the
                                                             distance to a lane
                                                             event in meters,
                                                             (0, 0~200, m) */
    uint8 auEventQuality_nu[LCF_SEN_CAMERA_LANE_NUM]; /* Describes the quality
                                                         of a detected lane
                                                         event ,(0, 0~100, %) */
    uint8 abAvailable[LCF_SEN_CAMERA_LANE_NUM];       /* Defines whether a lane
                                                         boundary track is available
                                                         or not, (0, 0~1, -) */
    float32 afStdDevPosY0_met[LCF_SEN_CAMERA_LANE_NUM]; /* Standard deviation of
                                                           the lateral
                                                           distance of lane
                                                           marker tracks,
                                                           (0,0~5, m) */
    float32 afStdDevHeadingAngle_rad[LCF_SEN_CAMERA_LANE_NUM]; /* Standard
                                                                  deviation of
                                                                  the yaw angle
                                                                  of
                                                                  lane marker
                                                                  tracks, (0,
                                                                  0~1, rad) */
    float32 afStdDevCurvature_1pm[LCF_SEN_CAMERA_LANE_NUM];    /* Standard
                                                                  deviation of the
                                                                  curvature, (0,
                                                                  0~0.5, 1/m) */
    float32 afStdDevCurvatureChange_1pm2
        [LCF_SEN_CAMERA_LANE_NUM]; /* Standard deviation of the curvature
                                      rate, (0, 0~1, 1/m^2) */
    uint8 uLaneChange_nu; /* Defines whether a lane change has been detected or
                             not, (0, 0~3, -) */
    uint8 auColor_nu[LCF_SEN_CAMERA_LANE_NUM]; /* Defines the color of a lane
                                                  boundary,(0, 0~5, -) */
    uint8 uLeftIndex_nu; /* Index for left-hand any boundaries,(0, -1~10, -) */
    uint8
        uRightIndex_nu; /* Index for right-hand any boundaries,(0, -1~10, -) */
    uint8 uRoadWorks_btf; /* ABD road works bitfield,(0, 0~255, -) */
    uint8 uiCompState_nu; /* Lane detection component state enumeration , (0,
                             0~6, -) */
    float32 fSineWaveDtct_nu;    /* Flag from ABD interface that indicates sine
                                    wave road condition, (0, -10~10, -) */
    boolean bParallelModelActiv; /* Parallel mode active flag, not used in
                                    algorithm now, (0, 0~1, -) */
    float32 fVertSlopeChange_nu; /* Defines slope change change state，(0,
                                    -10~10,  -)  */
    uint8 uWeatherCond_nu; /* Camera blockage (CB) weather condition enum ，(0,
                              0~255,  -) */
    boolean bLDPControlEnable;    // LDP control is enable by camera
    boolean bLDPLeControlEnable;  // LDP left control is enable by camera
    boolean bLDPRiControlEnable;  // LDP right control is enable by camera
    boolean bLDWControlEnable;    // LDW control is enable by camera
    boolean bLDWLeControlEnable;  // LDW left control is enable by camera
    boolean bLDWRiControlEnable;  // LDW right control is enable by camera
    uint8 abVitural[LCF_SEN_CAMERA_LANE_NUM];
} LCF_LaneData_t;
#define Rte_TypeDef_LCF_LaneData_t
#endif

#ifndef Rte_TypeDef_LCF_SenVehDyn_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    float32 fEgoVelX_mps;
    float32 fEgoCurve_1pm;
    float32 fEgoYawRate_rps;
    float32 fEgoVehYawRateStd_rpss;
    float32 fFrontAxleSteerAgl_rad; /* Effective steering angle at front axle */
    uint8 uEgoMotionState_nu;
    float32 fManuActuTrqEPS_nm; /* Actual manual torque of EPS */
    float32 fSteerWheelAgl_rad;
    float32 fSteerWheelAglChng_rps;  // steer wheel angle speed/chang value
    float32 fEgoAccelX_mpss;
    float32 fEgoAccelY_mpss;
} LCF_SenVehDyn_t;
#define Rte_TypeDef_LCF_SenVehDyn_t
#endif

#ifndef Rte_TypeDef_LCF_SenVehSig_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    uint8 uLCFAdasSysWarn_nu; /* State of lateral Adas system */
    float32 fAccPedPstn_Per;  /* Acceleration pedal position */
    boolean bCtrlStAvlbABS;   /* Condition of ABS is available */
    boolean bCtrlStAvlbACC;   /* Condition of ACC is available */
    boolean bCtrlStAvlbEBA;   /* Condition of EBA is available */
    boolean bCtrlStAvlbESC;   /* Condition of ESC is available */
    boolean bCtrlStAvlbTCS;   /* Condition of TSC is available */
    boolean bCtrlStAvlbVSM;   /* Condition of VSM is available */
    boolean bCtrlStEnABS;     /* Condition of ABS in actively controlling */
    boolean bCtrlStEnACC;     /* Condition of ACC in actively controlling */
    boolean bCtrlStEnEBA;     /* Condition of EBA in actively controlling  */
    boolean bCtrlStEnESC;     /* Condition of ESC in actively controlling */
    boolean bCtrlStEnTCS;     /* Condition of TSC in actively controlling */
    boolean bCtrlStEnVSM;     /* Condition of VSM in actively controlling */
    boolean bCtrlStEnARP;     /* Condition of TSC in actively controlling */
    boolean bCtrlStEnHDC;     /* Condition of VSM in actively controlling */
    boolean bDoorOpen;        /* Condition of the door is opened */
    boolean bDrvNoBuckledUp;  /* Condition of the driver is not buckled */
    boolean bDtctOverSte;     /* Condition of over steering has been detected */
    boolean bDtctRollerBench; /* Condition of vehicle is on roller bench */
    boolean bDtctUnderSte;   /* Condition of under steering has been detected */
    boolean bGrNeu;          /* Condition of the neutral gear is engaged */
    boolean bGrPark;         /* Condition of the parking gear is engaged */
    boolean bGrRvs;          /* Condition of the reverse gear is engaged */
    uint8 uStBrightness_nu;  /* Bright mess state */
    boolean bStErrABS;       /* Condition of ABS is in error state */
    boolean bStErrACC;       /* Condition of ACC is in error state */
    boolean bStErrEBA;       /* Condition of EBA is in error state */
    boolean bStErrESC;       /* Condition of ESC is in error state */
    boolean bStErrLatDMC;    /* Condition of LatDMC is in error state */
    boolean bStErrTSC;       /* Condition of TSC is in error state */
    boolean bStErrVDY;       /* Condition of VDY is in error state */
    boolean bStErrVSM;       /* Condition of VSM is in error state */
    boolean bStErrARP;       /* Condition of VSM is in error state */
    boolean bStErrHDC;       /* Condition of VSM is in error state */
    uint8 uStageWiper_St;    /* Wiper state */
    uint8 uStateWiper_St;    /* Wiper State */
    boolean bTrailerExs;     /* Condition of trailer is attached */
    boolean bTrnSglEnLf;     /* Condition of left turn signal is on */
    boolean bTrnSglEnRi;     /* Condition of right turn signal is on */
    boolean bTrnSglHarLigEn; /* Condition of turn hazard light is on */
    boolean bVehRdyToSta;    /* Condition of vehicle is ready to start */
    float32 fSpeedometerVelocity_kmh;  // VCU_SpdVelShow_Kmph
    boolean bLDWErrCondition;          // LDWC_ErrCdtn_B
    boolean bLDPErrCondition;          // LDPSC_ErrCdtn_B

    boolean bBrakePedalApplied;   /*the brake pedal applied flag from IDB*/
    boolean bTurnSignalLeverHold; /* Flag that turn signal level is hold*/
    uint8 uDrivingMode_nu;        /* Driving mode*/
    boolean bLKASwitch;           /* Flag that LKA switch is active*/
    boolean bTJASwitch;           /* Flag tha TJA switch is activet*/
    boolean bErrorStateTJA;       /* Flag that TJA is on error state*/
    boolean bErrorStateLKA;       /* Flag that LKA is on error state*/
    boolean bVehStopped;          /* Flag that vehicle is stopped*/

    boolean bSLCHMISwitch_bool;
    boolean bTJAAudioSwitch_bool;
    boolean LCFRCV_PilotOnLeverSwitch_bool;   // Pilot on levler switch
    boolean LCFRCV_PilotOffLeverSwitch_bool;  // Pilot off levler switch
    uint8 LCFRCV_TurnLightReqSt_nu;

    boolean LCFRCV_DoorOpen_bool;
    boolean LCFRCV_HoodOpen_bool;
    boolean LCFRCV_TrunkUnLock_bool;
    boolean LCFRCV_bPGear_bool;
    boolean VSDPI_BrakeDiscTempSts_B;
    boolean VSDPI_SignalInvalidLongi_B;
    boolean VSDPI_SignalInvalidLat_B;
    boolean bVehMoveBkwd_bool;
    boolean bRCA;
    boolean bRCC;
    boolean bFCA;
    boolean bFCB;
    boolean bPilotOnOff;
    boolean bRCA_LDW;
    boolean bFCA_LDW;
    boolean bRCA_LDP;
    boolean bFCA_LDP;
    uint8 ADCS4_AVM_Sts;
    uint8 ADCS11_Parking_WorkSts;
} LCF_SenVehSig_t;
#define Rte_TypeDef_LCF_SenVehSig_t
#endif

#ifndef Rte_TypeDef_LCF_LBSCheckState_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    boolean LCA_ActiveLeft_bool;    // Left side LCA active flag
    boolean LCA_ActiveRight_bool;   // Right side LCA active flag
    boolean LCA_WarningLeft_bool;   // Left side LCA warning flag
    boolean LCA_WarningRight_bool;  // Right side LCA warning flag
    boolean BSD_ActiveLeft_bool;    // Left side BSD active flag
    boolean BSD_ActiveRight_bool;   // Right side BSD active flag
    boolean BSD_WarningLeft_bool;   // Left side BSD warning flag
    boolean BSD_WarningRight_bool;  // Right side BSD warning flag
} LCF_LBSCheckState_t;
#define Rte_TypeDef_LCF_LBSCheckState_t
#endif

#ifndef Rte_TypeDef_LCF_MCTLFCState_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    uint8 uControllingFunction_nu;  // MCTLFC controlling function state
} LCF_MCTLFCState_t;
#define Rte_TypeDef_LCF_MCTLFCState_t
#endif

#ifndef Rte_TypeDef_LCF_TrajPlanState_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    uint16
        uQuStatusTrajPlan_nu;  // Trajectory plan qualifier--from TRJPLN module
} LCF_TrajPlanState_t;
#define Rte_TypeDef_LCF_TrajPlanState_t
#endif

#ifndef Rte_TypeDef_LCF_TrajCtrlState_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    uint8 uQuServTrajCtr_nu;  // Trajectory control qualifier--from TRJCTR
                              // module
} LCF_TrajCtrlState_t;
#define Rte_TypeDef_LCF_TrajCtrlState_t
#endif

#ifndef Rte_TypeDef_LCF_HandsoffWarn_t
typedef struct {
    UINT8_T HOD_StSysWarning;    /* Level of hands off system warning */
                                 /* 0 NoWarning  */
                                 /* 1 HandsOnRequest */
                                 /* 2 TakeOverRequest */
                                 /* 3 TriggerDegradation */
    boolean HOD_EnaHandsOffCnfm; /* flag of hands off or on */
} LCF_HandsoffWarn_t;
#define Rte_TypeDef_LCF_HandsoffWarn_t
#endif

#ifndef Rte_TypeDef_LCF_SenToSen_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    LCF_MCTLFCState_t
        sMCTLFCState;  // the input data of lcf sen from MCTLFC(sen)
} LCF_SenToSen_t;
#define Rte_TypeDef_LCF_SenToSen_t
#endif

#ifndef Rte_TypeDef_LCF_VehToSen_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    LCF_TrajPlanState_t
        sTrajPlanState;  // the input data of lcf sen from TrajPlan(veh)
    LCF_TrajCtrlState_t
        sTrajCtrlState;  // the input data of lcf sen from TrajCtrl(veh)
    LCF_HandsoffWarn_t
        sHandsoffWarn;  // the input data of lcf sen from HOD(veh)
} LCF_VehToSen_t;
#define Rte_TypeDef_LCF_VehToSen_t
#endif

#ifndef Rte_TypeDef_LCF_SenCUSTOMState_t
typedef struct {
    LCF_SenSignalHeader_t sSigHeader;
    uint16 uPrjSpecQu_btf;  // Specific qualifier--from CUSTOM
} LCF_SenCUSTOMState_t;
#define Rte_TypeDef_LCF_SenCUSTOMState_t
#endif

#ifndef Rte_TypeDef_ST_LCFSEN_NVRAMData_t
typedef struct {
    boolean LCFSen_Nb_LDWPowerOffSwitch_nu;  // LDW power off
    boolean LCFSen_Nb_LDPPowerOffSwitch_nu;  // LDP power off
    boolean LCFSEN_Nb_DCLCSwitchNVRAM_nu;    // DCLC on off
    boolean LCFSen_Nb_SwitchReserved1_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved2_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved3_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved4_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved5_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved6_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved7_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved8_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved9_nu;    // Reserved
    boolean LCFSen_Nb_SwitchReserved10_nu;   // Reserved

    uint8 LCFSen_Nu_LDWSensitivity_nu;        // LDW sensitivity
    uint8 LCFSen_Nu_LDPSensitivity_nu;        // LDP sensitivity
    uint8 LCFSen_Nu_SensitivityReserved1_nu;  // Reserved
    uint8 LCFSen_Nu_SensitivityReserved2_nu;  // Reserved
    uint8 LCFSen_Nu_SensitivityReserved3_nu;  // Reserved
    uint8 LCFSen_Nu_SensitivityReserved4_nu;  // Reserved
    uint8 LCFSen_Nu_SensitivityReserved5_nu;  // Reserved

    float32 LCFSen_Nf_LDWStartupSpd_kph;  // LDW startup speed
    float32 LCFSen_Nf_LDPStartupSpd_kph;  // LDW startup speed
    float32 LCFSen_Nf_Reserved1_nu;       // Reserved
    float32 LCFSen_Nf_Reserved2_nu;       // Reserved
    float32 LCFSen_Nf_Reserved3_nu;       // Reserved
    float32 LCFSen_Nf_Reserved4_nu;       // Reserved
    float32 LCFSen_Nf_Reserved5_nu;       // Reserved
    float32 LCFSen_Nf_Reserved6_nu;       // Reserved
    float32 LCFSen_Nf_Reserved7_nu;       // Reserved
    float32 LCFSen_Nf_Reserved8_nu;       // Reserved
    float32 LCFSen_Nf_Reserved9_nu;       // Reserved
    float32 LCFSen_Nf_Reserved10_nu;      // Reserved
} ST_LCFSEN_NVRAMData_t;
#define Rte_TypeDef_ST_LCFSEN_NVRAMData_t
#endif

#ifndef Rte_TypeDef_LCF_VLCData_t
typedef struct {
    uint8 VLCVEH_ACCStatus_nu;  // Specific qualifier--from CUSTOM
} LCF_VLCData_t;
#define Rte_TypeDef_LCF_VLCData_t
#endif

#ifndef Rte_TypeDef_reqLcfSenPrtList_t
typedef struct {
    LCF_Sen_BaseCtrlData_t sBaseCtrlData;  // Control data giving information
                                           // about the current mode
    // LCF_SSRObjectList_t sSSRObjectList;    // Object list from side radars

    LCF_ACCOOIData_t sLCFACCOOIData;  // ACC OOI-0 data from ACC module
    LCF_LaneData_t sLCFLaneData;      // lane information from camera sensor
    LCF_SenVehDyn_t sLCFVehDyn;       // Vehicle dynamic data (VDY output)
    LCF_SenVehSig_t sLCFVehSig;       // Main Vehicle dynacmis sensor signals
    LCF_LBSCheckState_t
        sLCFLBSCheckState;  // Check state of LCA and BSD from LBS
    LCF_SenToSen_t
        sLcfSenInputFromSenData;  // the input data of lcf sen from lcf sen
    LCF_VehToSen_t
        sLcfSenInputFromVehData;  // the input data of lcf sen from lcf veh
    LCF_DMCToSen_t
        sLcfSenInputFromDMCData;        // the input data of lcf sen from DMC
    LCF_SenCUSTOMState_t sCUSTOMState;  // Custom qualifier
    ST_LCFSEN_NVRAMData_t sNVRAMData;   // NVRAM Data
    LCF_FusionObjectsArray_t sFusionObjectsArray;  // Fusion objects Array
    LCF_VLCData_t sVLCData;                        // ACC releated data
} reqLcfSenPrtList_t;
#define Rte_TypeDef_reqLcfSenPrtList_t
#endif

#ifndef Rte_TypeDef_reqLcfSenParams
typedef struct {
    float32 LCFSen_Kf_SysCycleTime_sec;
    float32 LCFSen_Kf_fEgoWidth_met;
    float32 LCFSen_Kf_fEgoLength_met;
    float32 LCFSen_Kf_fVehOverhangFront_met;  // Front overhang for
                                              // transformation from radar to
                                              // front axle coordinate system
    float32 LCFSen_Kf_fVehWheelBase_met;      // Vehicle wheelbase
    float32 LCFSen_Kf_fEgoVehLength_met;
    float32 LCFSen_Kf_fLatPosStepDtcLimit_met;
    float32 LCFSen_Kf_fDebounceTLatPosStep_sec;
    float32 LCFSen_Kf_fAliveCounterTime_sec;
    uint8 LCFSen_Ku_uiMinAliveCounter_perc;
    boolean LCFSen_Kb_bAllowBridging_bool;
    float32 LCFSen_Kf_LatPosPT1TConst_sec;

} reqLcfSenParams;
#define Rte_TypeDef_reqLcfSenParams
#endif

#ifndef Rte_TypeDef_LCF_SenAlgoCompState_t
typedef struct {
    uint32 uiVersionNumber;
    LCF_SenSignalHeader_t sSigHeader;
    uint32 uiAlgoVersionNumber;
    uint32 uiApplicationNumber;
    uint8 AlgoVersionInfo[50];
    uint8 eCompState;
    uint8 eShedulerSubModeRequest;
    uint32 eErrorCode;
    uint32 eGenAlgoQualifier;
} LCF_SenAlgoCompState_t;
#define Rte_TypeDef_LCF_SenAlgoCompState_t
#endif

#ifndef Rte_TypeDef_LCF_LDWSAOutput_t
typedef struct {
    uint8 LDWC_DgrSide_St;
    uint8 LDWC_RdyToTrig_B;
    uint8 LDWC_SysOut_St;
    real32_T LDVSE_NVRAMVehStartupSpd_kmph;
    boolean_T LDWC_NVRAMLDWSwitch_B;
} LCF_LDWSAOutput_t;
#define Rte_TypeDef_LCF_LDWSAOutput_t
#endif

#ifndef Rte_TypeDef_LCF_LDPSAOutput_t
typedef struct {
    uint8 LDPSC_DgrSide_St;
    uint8 LDPSC_RdyToTrig_B;
    uint8 LDPSC_SysOut_St;
    float32 LDPDT_LnPstnLf_Mi;
    float32 LDPDT_LnPstnRi_Mi;
    float32 LDPDT_LnHeadLf_Rad;
    float32 LDPDT_LnHeadRi_Rad;
    float32 LDPDT_LnCltdCurvLf_ReMi;
    float32 LDPDT_LnCltdCurvRi_ReMi;
    real32_T LDPVSE_NVRAMVehStartupSpd_Kmph;
    boolean_T LDPSC_NVRAMLDPSwitch_B;
} LCF_LDPSAOutput_t;
#define Rte_TypeDef_LCF_LDPSAOutput_t
#endif

#ifndef Rte_TypeDef_LCF_LBPOutput_t
/* LBP Output */
typedef struct {
    uint16 uLaneInvalidQualifierLf; /* Qualifier for left lane invalid, (0,
                                       0~65535, -) */
    /* out1, S_ABPLBP_LeLnInvalidQu_btf */
    uint16 uLaneInvalidQualifierRi; /* Qualifier for right lane invalid, (0,
                                       0~65535, -)*/
    /* out2, S_ABPLBP_RiLnInvalidQu_btf */
    float32
        fLaneWidth; /* Lane width by uncoupled lane processing (0, 0~10, m) */
                    /* out3, S_ABPLBP_LaneWidth_met */
    uint8 bLaneChangeDtct; /* Flag that indicates a detected lane change, (0~1,
                              -, ) */
    /* out4, S_ABPLBP_LaneChangeDetected_bool */
    float32 fPosX0CtrlLf; /* Left lane clothoid X0 position, (0, -300~300, m) */
                          /* out5, S_ABPLBP_LeLnClthPosX0_met */
    float32 fPosY0CtrlLf; /* Left lane clothoid Y0 position  (init +10m), (0,
                             -15~15, m) */
    /* out6, S_ABPLBP_LeLnClthPosY0_met */
    float32 fHeadingCtrlLf; /* Left lane clothoid heading angle, (0,
                               -0.7854~0.7854, rad) */
    /* out7, S_ABPLBP_LeLnClthHeading_rad */
    float32 fCrvCtrlLf; /* Left lane clothoid curvature, (0, -0.1~0.1, 1/m) */
                        /* out8, S_ABPLBP_LeLnClthCrv_1pm */
    float32 fCrvRateCtrlLf; /* Left lane clothoid change of curvature, (0,
                               -0.001~0.001, 1/m^2) */
    /* out9, S_ABPLBP_LeLnClthCrvChng_1pm2 */
    float32 fValidLengthCtrlLf; /* Left lane clothoid length, (0, 0~300, m) */
                                /* out10, S_ABPLBP_LeLnClthLength_met */
    float32
        fPosX0CtrlCntr; /* Center lane clothoid X0 position, (0, -300~300, m) */
                        /* out11, S_ABPLBP_CntrLnClthPosX0_met */
    float32 fPosY0CtrlCntr; /* Center lane clothoid Y0 position (init +10m), (0,
                               -15~15, m) */
    /* out12, S_ABPLBP_CntrLnClthPosY0_met  */
    float32 fHeadingCtrlCntr; /* Center lane clothoid heading angle, (0,
                                 -0.7854~0.7854, rad) */
    /* out13, S_ABPLBP_CntrLnClthHeading_rad */
    float32 fCrvCtrlCntr; /* Center lane clothoid curvature, (0, -0.1~0.1, 1/m*/
                          /* out14, S_ABPLBP_CntrLnClthCrv_1pm */
    float32 fCrvRateCtrlCntr; /* Center lane clothoid change of curvature, (0,
                                 -0.001~0.001, 1/m^2) */
    /* out15, S_ABPLBP_CntrLnClthCrvChng_1pm2 */
    float32
        fValidLengthCtrlCntr; /* Center lane clothoid length, (0, 0~300, m) */
                              /* out16, S_ABPLBP_CntrLnClthLength_met */
    float32
        fPosX0CtrlRi; /* Right lane clothoid X0 position, (0, -300~300, m) */
                      /* out17, S_ABPLBP_RiLnClthPosX0_met */
    float32 fPosY0CtrlRi; /* Right lane clothoid Y0 position (init +10m), (0,
                             -15~15, m) */
    /* out18, S_ABPLBP_RiLnClthPosY0_met */
    float32 fHeadingCtrlRi; /* Right lane clothoid heading angle, (0,
                               -0.7854~0.7854, rad) */
    /* out19, S_ABPLBP_RiLnClthHeading_rad */
    float32 fCrvCtrlRi; /* Right lane clothoid curvature, (0, -0.1~0.1, 1/m) */
                        /* out20, S_ABPLBP_RiLnClthCrv_1pm */
    float32 fCrvRateCtrlRi; /* Right lane clothoid change of curvature, (0,
                               -0.001~0.001, 1/m^2) */
    /* out21, S_ABPLBP_RiLnClthCrvChng_1pm2 */
    float32 fValidLengthCtrlRi; /* Right lane clothoid length, (0, 0~300, m) */
                                /* out22, S_ABPLBP_RiLnClthLength_met */
    float32 fPosY0SafeLf; /* Left lane clothoid Y0 position (safety interface),
                             (-15~15, m)*/
    /* out23, S_ABPLBP_LeLnClthPosY0SIF_met */
    float32 fHeadingSafeLf; /* Left lane clothoid heading angle (safety
                               interface), (-0.7854~0.7854, rad) */
    /* out24, S_ABPLBP_LeLnClthHeadingSIF_rad */
    float32 fCrvSafeLf; /* Left lane clothoid curvature (safety interface),
                           (-0.1~0.1, 1/m) */
    /* out25, S_ABPLBP_LeLnClthCrvSIF_1pm */
    float32 fPosY0SafeRi; /* Right lane clothoid Y0 position (safety interface),
                             (-15~15, m) */
    /* out26, S_ABPLBP_RiLnClthPosY0SIF_met*/
    float32 fHeadingSafeRi; /* Right lane clothoid heading angle (safety
                               interface), (-0.7854~0.7854, rad) */
    /* out27, S_ABPLBP_RiLnClthHeadingSIF_rad */
    float32 fCrvSafeRi; /* Right lane clothoid curvature (safety interface),
                           (-0.1~0.1, 1/m) */
    /* out28, S_ABPLBP_RiLnClthCrvSIF_1pm */
    uint8 uInvalidQualifierSafeLf; /* Left lane invalid qualifier for the safety
                                      interface, (0~255, -) */
    /* out29, S_ABPLBP_LeLnInvalidQuSIF_btf */
    uint8 uInvalidQualifierSafeRi; /* Right lane invalid qualifier for the
                                      safety interface, (0~255, -) */
    /* out30, S_ABPLBP_RiLnInvalidQuSIF_btf */
    uint8
        uLaneValidQualDMC; /* Lane valid qualifier for LatDMC, (0, 0~255, -) */
                           /* out31, S_ABPLBP_LaneValidQualVis_nu */
    uint8 uVisualValidQualifier; /* Qualifier for the visualization, (0, 0~255,
                                    -) */
    /* out32, S_ABPLBP_LaneValidQualVis_nu */
    uint8 uLaneTypeLf; /* Left lane type(eg,9 means road edge) , (0~255, -) */
                       /* out33, S_ABPLBP_LeftLaneType_enum */
    uint8 uLaneTypeRi; /* Right lane type(eg,9 means road edge) , (0~255, -) */
                       /* out34, S_ABPLBP_RightLaneType_enum */
    uint8 bConstructionSiteDtct; /* 1: Construction site detected; 0: No
                                    construction site (0~1, -) */
    /* out35, S_ABPLBP_ConstructionSite_bool */
    uint8 uOverallQualityLf; /* Quality of the left  lane information based on
                                all properties, (0~255, %)*/
    /* out36, S_ABPLBP_LeLnQuality_perc */
    uint8 uOverallQualityRi; /* Quality of the right lane information based on
                                all properties, (0~255, %) */
    /* out37, S_ABPLBP_RiLnQuality_perc */
    uint8 uCrvQualityLf; /* Quality of the left curvature information, (0~255,
                            %) */
    /* out38, S_ABPLBP_LeCrvQuality_perc */
    uint8 uCrvQualityRi; /* Quality of the right curvature information, (0~255,
                            %) */
    /* out39, S_ABPLBP_RiCrvQuality_perc */
    float32 fABDTimeStamp; /* ABD data timestamp in seconds , (0~4295, s) */
                           /* out40, S_ABPLBP_ABDTimeStamp_sec */
    uint16 uRangeCheckQualifier; /* Input range check qualifier bitfield ,
                                    (0~65535, s) */
    /* out41, S_ABPLBP_RangeCheckQualifier_btf */
    float32 fFltQualityCntr; /* Center lane Kalman quality, (0, 0~100, %) */
                             /* out42, S_ABPLBP_CntrLnKalmanQual_perc */
    uint8 uFltStatusCntr;    /* Center lane Kalman filter status, (0, 0~5, -) */
                             /* out43, S_ABPLBP_CntrLnKalmanStatus_nu */
    uint16 uOutRangeCheckQualifier; /* Output data range qualifier, (0, 0~65535,
                                       -) */
    /* out44 S_ABPLBP_OutRangeCheckQuali_btf) */
    uint8 uPercStraightDtct; /* Confidence for straight detection - value from 0
                                to 100, (0, 0~100, %) */
    /* out45 S_ABPLBP_StraightPathDtct_nu */
    uint8 uPercExitLf; /* Exit percent for left side, (0, 0~100, %) */
                       /* out46, S_ABPLBP_ExitLeft_perc */
    uint8 uPercExitRi; /* Exit percent for right side, (0, 0~100, %) */
                       /* out47, S_ABPLBP_ExitRight_perc */
    uint8 bBridgePossibleUnCplLf; /* Left lane uncoupled lane bridging possible,
                                     (0, 0~1, -) */
    /* out48, S_ABPLBP_LeUncoupBrid_bool */
    uint8 bBridgePossibleUnCplRi; /* Right lane uncoupled lane bridging
                                     possible, (0, 0~1, -) */
    /* out49, S_ABPLBP_RiUncoupBrid_bool */
    uint8 uPercUpDownHillDtct; /* Indicates downhill / uphill scenario detection
                                  confidence, (0, 0~100, %) */
    /* out50, S_ABPLBP_UpDownHillDtct_perc */
    float32 fLaneWidthUnCpl; /* Raw lane width of ABD interface uncoupled lane
                                data, (0, 0~10, m) */
    /* out51, S_ABPLBP_UncoupledLaneWidth_met */
    float32 fLaneWidthCpl; /* Raw lane width of ABD interface coupled lane data,
                              (0, 0~10, m) */
    /* out52, S_ABPLBP_CoupledLaneWidth_met */
    uint8
        uOverallQualityUnCplLf; /* Left uncoupled lane quality, (0, 0~100, %) */
                                /* out53, S_ABPLBP_LeUncoupLnQual_perc */
    uint8 uOverallQualityUnCplRi; /* Right uncoupled lane quality, (0, 0~100, %)
                                   */
    /* out54, S_ABPLBP_RiUncoupLnQual_perc */
    uint8 uOverallQualityCplLf; /* Left coupled lane quality, (0, 0~100, %) */
                                /* out55, S_ABPLBP_LeUncoupLnQual_perc */
    uint8 uOverallQualityCplRi; /* Right coupled lane quality, (0, 0~100, %) */
                                /* out56, S_ABPLBP_RiUncoupLnQual_perc */
    uint8 uBtfBridgeUnCpl; /* Uncoupled lane bridge bitfield, (0, 0~100, %) */
                           /* out57, S_ABPLBP_UncoupledLaneBrid_btf */
} LCF_LBPOutput_t;
#define Rte_TypeDef_LCF_LBPOutput_t
#endif

#ifndef Rte_TypeDef_LCF_ALPOutput_t
/* ALP output */
typedef struct {
    boolean bLeftAdjLnStepDeounced_bool;
    boolean bRightAdjLnStepDeounced_bool;
    boolean bLeftAdjLnBridging_bool;
    boolean bRightAdjLnBridging_bool;
    boolean bLeftAdjLnStepOnBridging_bool;
    boolean bRightAdjLnStepOnBridging_bool;

    // 0x01:AnyBndAailable, 0:valid,                                1:invalid;
    // 0x02:StepDeboundced, 0:not in deboundced state,              1:debounced
    // step;
    // 0x04:Bridging,       0:not in bridging state,                1:bridging
    // state;
    // 0x08:StepOnBridging, 0:step check isn't pass after bridging, 1:step check
    // pass after bridging
    uint16 uiLeftAdjLnInvalidQu_btf;
    uint16 uiRightAdjLnInvalidQu_btf;
    uint8 uiLeftAdjLnAliveCount_per;
    uint8 uiRightAdjLnAliveCount_per;

    float32 fLeftAdjLnDistance_met;
    float32 fLeftAdjLnYawAngle_rad;
    float32 fLeftAdjLnCurvature_1pm;
    float32 fLeftAdjLnCrvRate_1pm2;
    float32 fLeftAdjLnValidLength_met;

    float32 fRightAdjLnDistance_met;
    float32 fRightAdjLnYawAngle_rad;
    float32 fRightAdjLnCurvature_1pm;
    float32 fRightAdjLnCrvRate_1pm2;
    float32 fRightAdjLnValidLength_met;
} LCF_ALPOutput_t;
#define Rte_TypeDef_LCF_ALPOutput_t
#endif

#ifndef Rte_TypeDef_LCF_MCTLFCOut_st
typedef struct {
    real32_T CSCLTA_LeCridrBndPosX0_met;    /* '<S7>/Multiport Switch1'
                                             * the PosX0
                                             * value of left corridor bound, [-300,
                                             * 300]
                                             */
    real32_T CSCLTA_LeCridrBndPosY0_met;    /* '<S7>/Multiport Switch1'
                                             * the
                                             * PosY0 value of left corridor bound,
                                             * [-15, 15]
                                             */
    real32_T CSCLTA_LeCridrHeadAng_rad;     /* '<S7>/Multiport Switch1'
                                             * the
                                             * heading angle value of left corridor
                                             * bound, [-0.78539816, 0.78539816]
                                             */
    real32_T CSCLTA_LeCridrBndCrv_1pm;      /* '<S7>/Multiport Switch1'
                                             * the
                                             * curve value of left corridor bound,
                                             * [-0.1, 0.1]
                                             */
    real32_T CSCLTA_LeCridrBndCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                             * the
                                             * curve deviation value of left
                                             * corridor bound, [-0.001, 0.001]
                                             */
    real32_T CSCLTA_LeCridrBndLength_met;   /* '<S7>/Multiport Switch1'
                                             * the
                                             * length value of left corridor
                                             * bound, [0,   150]
                                             */
    real32_T CSCLTA_RiCridrBndPosX0_met;    /* '<S7>/Multiport Switch1'
                                             * the
                                             * PosX0 value of right corridor bound,
                                             * [-300, 300]
                                             */
    real32_T CSCLTA_RiCridrBndPosY0_met;    /* '<S7>/Multiport Switch1'
                                             * the
                                             * PosY0 value of right corridor bound,
                                             * [-15, 15]
                                             */
    real32_T CSCLTA_RiCridrHeadAng_rad;     /* '<S7>/Multiport Switch1'
                                             * the
                                             * heading angle value of right corridor
                                             * bound, [-0.78539816, 0.78539816]
                                             */
    real32_T CSCLTA_RiCridrBndCrv_1pm;      /* '<S7>/Multiport Switch1'
                                             * the
                                             * curve value of right corridor bound,
                                             * [-0.1, 0.1]
                                             */
    real32_T CSCLTA_RiCridrBndCrvChng_1pm2; /* '<S7>/Multiport Switch1'
                                             * the
                                             * curve deviation value of right
                                             * corridor bound, [-0.001, 0.001]
                                             */
    real32_T CSCLTA_RiCridrBndLength_met;   /* '<S7>/Multiport Switch1'
                                             * the
                                             * length value of right corridor
                                             * bound,   [0, 150]
                                             */
    real32_T CSCLTA_TgtTrajPosX0_met;       /* '<S7>/Multiport Switch1'
                                             * the PosX0
                                             * value of target corridor bound, [-300,
                                             * 300]
                                             */
    real32_T CSCLTA_TgtTrajPosY0_met;       /* '<S7>/Multiport Switch1'
                                             * the PosY0
                                             * value of target corridor bound, [-15,
                                             * 15]
                                             */
    real32_T CSCLTA_TgtTrajHeadAng_rad;     /* '<S7>/Multiport Switch1'
                                             * the
                                             * heading angle value of target
                                             * corridor bound, [-0.78539816,
                                             * 0.78539816]
                                             */
    real32_T CSCLTA_TgtTrajCrv_1pm;         /* '<S7>/Multiport Switch1'
                                             * the curve value
                                             * of target corridor bound, [-0.1, 0.1]
                                             */
    real32_T CSCLTA_TgtTrajCrvChng_1pm2;    /* '<S7>/Multiport Switch1'
                                             * the
                                             * curve deviation value of target
                                             * corridor bound, [-0.001, 0.001]
                                             */
    real32_T CSCLTA_TgtTrajLength_met;      /* '<S7>/Multiport Switch1'
                                             * the
                                             * length value of target corridor bound,
                                             * [0, 150]
                                             */
    real32_T CSCLTA_WeightTgtDistY_nu;      /* '<S7>/Multiport Switch1'
                                             * The
                                             * importance factor of the lateral
                                             * deviation, [0,1]
                                             */
    real32_T CSCLTA_WeightEndTime_nu;       /* '<S7>/Multiport Switch1'
                                             * The
                                             * importance factor of the time required
                                             * for the planned trajectory, [0,1]
                                             */
    real32_T
        CSCLTA_DistYToILeTgtArea_met; /* '<S7>/Multiport Switch1'
                                       * lateral
                                       * tolerance left boundary value , [0,10]
                                       */
    real32_T
        CSCLTA_DistYToIRiTgtArea_met;      /* '<S7>/Multiport Switch1'
                                            * lateral
                                            * tolerance right boundary value , [0,10]
                                            */
    real32_T CSCLTA_FTireAclMax_mps2;      /* '<S7>/Multiport Switch1'
                                            * lateral
                                            * acceleration upper limiting value,
                                            * [-20,20]
                                            */
    real32_T CSCLTA_FTireAclMin_mps2;      /* '<S7>/Multiport Switch1'
                                            * lateral
                                            * acceleration lower limiting value,
                                            * [-20,20]
                                            */
    real32_T CSCLTA_PredTimeHeadAng_sec;   /* '<S7>/Multiport Switch1'
                                            * todo,
                                            * [0，60]
                                            */
    real32_T CSCLTA_PredTimeCrv_sec;       /* '<S7>/Multiport Switch1'
                                            * todo,[0，60]
                                            */
    real32_T CSCLTA_PlanningHorzion_sec;   /* '<S7>/Multiport Switch1'
                                            * max
                                            * Planning horizon(time) of the
                                            * trajectory, [0，60]
                                            */
    real32_T CSCLTA_ObstacleVelX_mps;      /* '<S7>/Multiport Switch1'
                                            * the
                                            * obstacle velocity X in target
                                            * trajectory if it is existed , [-20,150]
                                            */
    real32_T CSCLTA_ObstacleAclX_mps2;     /* '<S7>/Multiport Switch1'
                                            * the
                                            * obstacle accel X in target trajectory
                                            * if it is existed , [-20,20]
                                            */
    real32_T CSCLTA_ObstacleWidth_met;     /* '<S7>/Multiport Switch1'
                                            * the
                                            * obstacle width in target trajectory if
                                            * it is existed , [0,150]
                                            */
    real32_T CSCLTA_ObstacleDistX_met;     /* '<S7>/Multiport Switch1'
                                            * the
                                            * obstacle distance X in target
                                            * trajectory if it is existed ,
                                            * [-1000,1000]
                                            */
    real32_T CSCLTA_ObstacleDistY_met;     /* '<S7>/Multiport Switch1'
                                            * the
                                            * obstacle distance X in target
                                            * trajectory if it is existed ,
                                            * [-1000,1000]
                                            */
    real32_T CSCLTA_SensorTStamp_sec;      /* '<S7>/Multiport Switch1'
                                            * time
                                            * stamp of the camera signal from camera
                                            * sensor,[0,4295]
                                            */
    real32_T CSCLTA_MaxCrvTrajGuiCtrl_1pm; /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxCrvGrdBuildup_1pms; /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxCrvGrdRed_1pms;     /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_GrdLimitTgtCrvGC_1pms; /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_StrWhStifLimit_nu;     /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_StrWhStifGrad_1ps;     /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_TrqRampGrad_1ps;       /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxTrqScalLimit_nu;    /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxTrqScalGrad_nu;     /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_LimiterDuration_sec;   /* '<S7>/Multiport Switch1' */
    real32_T CSCLTA_MaxJerkAllowed_mps3;   /* '<S7>/Multiport Switch1'
                                            * Maximum
                                            * Jerk Allowed in the trajectory
                                            * planning,   [0,50]
                                            */
    uint8_T CSCLTA_DeratingLevel_nu;       /* '<S7>/Multiport Switch1' */
    uint8_T CSCLTA_ControllingFunction_nu; /* '<S5>/Switch' */
    uint8_T CSCLTA_SysStateLCF;            /* '<S7>/Multiport Switch1'
                                            * lateral control
                                            * function system state enum value,
                                            * [0,6],E_LCF_SYSSTATE_NOTPRESENT = 0;
                                            * E_LCF_SYSSTATE_DISABLED = 1;
                                            * E_LCF_SYSSTATE_PASSIVE = 2;
                                            * E_LCF_SYSSTATE_REQUEST = 3;
                                            * E_LCF_SYSSTATE_CONTROLLING = 4;
                                            * E_LCF_SYSSTATE_RAMPOUT = 5;
                                            * E_LCF_SYSSTATE_ERROR = 6;
                                            */
    uint8_T CSCLTA_TgtPlanServQu_nu;       /* '<S7>/Multiport Switch1'
                                            * todo
                                            */
    uint8_T CSCLTA_TrajGuiQualifier_nu;    /* '<S7>/Multiport Switch1'
                                                                          *
                                              qualifier value of trajectory
                                              guidence, The qualifier indicates
                                              if/how the target curvature should be
                                              considered, [0,5] .E_LCF_TGQ_REQ_OFF=
                                              0, E_LCF_TGQ_REQ	= 1,
                                              E_LCF_TGQ_REQ_FREEZE= 3,
                                              E_LCF_TGQ_REQ_FFC	= 4,
                                              E_LCF_TGQ_REQ_REFCHNG= 5
   
                                                                          */
    boolean_T CSCLTA_TriggerReplan_nu;     /* '<S7>/Multiport Switch1'
                                            * trigger
                                            * replan signal from CSCLTA
                                            * module, [0，1]
                                            */
    boolean_T
        CSCLTA_LatencyCompActivated_bool; /* '<S7>/Multiport Switch1'
                                           * the trigger flag for latency
                                           * compensation function, [0,1] 1:
                                           * latency compensation enable, 0:
                                           * latency compensation disable
                                           */
    boolean_T CSCLTA_HighStatAccu_bool;   /* '<S7>/Multiport Switch1' */
    boolean_T CSCLTA_LimiterActivated_nu; /* '<S7>/Multiport Switch1' */
} LCF_MCTLFCOut_st;
#define Rte_TypeDef_LCF_MCTLFCOut_st
#endif

#ifndef Rte_TypeDef_LCF_TJASAOutput_t
typedef struct {
    uint8 TJASTM_LatCtrlMode_nu;
} LCF_TJASAOutput_t;
#define Rte_TypeDef_LCF_TJASAOutput_t
#endif

#ifndef Rte_TypeDef_LCF_SENTJASAOutput_t
typedef struct {
    uint8_T TJASTM_SysStateHWA_nu;
    uint8_T TJASTM_NpilotSysInfo;
    uint8_T TJASTM_PilotAudioPlay;
    uint8_T TJASTM_LatCtrlHandsOffReleaseWarn_nu;
    uint8_T TJASLC_LaneChangWarning_nu; /* '<S298>/Data Type Conversion'
    * lane change warning side
    0: no warning
    1: left warning
    2: right warning
    */
    uint8_T TJASLC_SLCAudioPlay_nu;
    uint8_T TJASTM_PilotEnableACCSwitch_nu;
    uint8_T TJASTM_PilotDisableACCSwitch_nu;
} LCF_SENTJASAOutput_t;
#define Rte_TypeDef_LCF_SENTJASAOutput_t
#endif

#ifndef Rte_TypeDef_LCF_SenGenericOutputs_t
typedef struct { /* [Satisfies_rte sws 1191] */

    uint32 uiVersionNumber;
    LCF_SenSignalHeader_t sSigHeader;
    LCF_LDWSAOutput_t sLDWSAOutput;
    LCF_LDPSAOutput_t sLDPSAOutput;
    LCF_LBPOutput_t sLBPOutput;
    LCF_ALPOutput_t sALPOutput;
    LCF_SENTJASAOutput_t sTJAOutput;
} LCF_SenGenericOutputs_t;
#define Rte_TypeDef_LCF_SenGenericOutputs_t
#endif

#ifndef Rte_TypeDef_LCF_SenToVehs_t
typedef struct /* [Satisfies_rte sws 1191] */
{
    uint32 uiVersionNumber;
    LCF_SenSignalHeader_t sSigHeader;
    uint8 ControllingFunction_nu;
    uint8 ActFctEnabled_nu;
    uint8 ActFctLevel_nu;
    uint8 SpecialDrivingCorridor__nu;
    float32 MaxJerkAllowed_mps3;
    uint8 OcObjActSide_nu;
    uint8 AngReqMaxLimitScale_per;
    uint8 AngReqRateMaxLimitScale_per;
    float32 Reserved1_nu;
    float32 Reserved2_nu;
    // LCF_FCT_t LCF_FCT;
    // LCF_SysCoordinator_t LCF_SysCoordinator;
    // ABP_CamRawData_t ABP_CamRawData;
    // EvntDtctnSenEvents__nu_array_t EvntDtctnSenEvents__nu;
    LCF_TJASAOutput_t sTJASAOutput;
    LCF_MCTLFCOut_st sMCTLFCOut;
} LCF_SenToVehs_t;
#define Rte_TypeDef_LCF_SenToVehs_t
#endif

#ifndef Rte_TypeDef_proLcfSenPrtList_t
typedef struct {
    LCF_SenAlgoCompState_t
        pAlgoCompState;  //!< State return values of the algo component
    LCF_SenGenericOutputs_t pLcfSenOutputData;  //!<
    LCF_SenToVehs_t pLcfSenOutputToVehData;     //!<
    ST_LCFSEN_NVRAMData_t sNVRAMData;           // NVRAM Data
} proLcfSenPrtList_t;
#define Rte_TypeDef_proLcfSenPrtList_t
#endif

#ifndef Rte_TypeDef_reqLcfSenDebug_t
typedef struct {
    sLBPDebug_t sLBPDebug;
    sALPDebug_st sALPDebug;
    ODPRDebug_t sODPRDebug;
    sVSDPDebug_t sVSDPDebug;
    sLDWSADebug_t sLDWSADebug;
    sLDPSADebug_t sLDPSADebug;
    sTJASADebug_t sTJASADebug;
    sMCTLFCDebug_st sMCTLFCDebug;
    sLCCRADebug_t sLCCRADebug;
    sLCFOPSDebug_t sLCFOPSDebug;
} reqLcfSenDebug_t;
#define Rte_TypeDef_reqLcfSenDebug_t
#endif

#ifndef Rte_TypeDef_proLcfToVlc_t
typedef struct {
    uint8 TJASTM_PilotEnableACCSwitch_nu;
    uint8 TJASTM_PilotDisableACCSwitch_nu;
    uint8 TJASLC_ManeuverState_nu;
    uint8 TJASLC_LaneChangeTrig_nu;
} proLcfToVlc_t;
#define Rte_TypeDef_proLcfToVlc_t
#endif

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
void LcfSenExec(const reqLcfSenPrtList_t* const reqPorts,
                const reqLcfSenParams* reqParams,
                proLcfSenPrtList_t* const proPorts,
                reqLcfSenDebug_t* proDebugs);
void LcfSenReset(const reqLcfSenParams* reqParams);

#ifdef __cplusplus
}
#endif
#endif