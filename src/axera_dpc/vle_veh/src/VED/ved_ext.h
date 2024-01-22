/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */
#pragma once
#ifndef VED_EXT_H
#define VED_EXT_H

#ifdef __cplusplus
extern "C" {
#endif
#include "TM_Global_TypeDefs.h"
#include "Rte_Type.h"
// #ifndef boolean
// typedef unsigned char boolean;
// #endif
// #ifndef sint8
// typedef signed char sint8;
// #endif
// #ifndef uint8
// typedef unsigned char uint8;
// #endif
// #ifndef int16
// typedef signed short int16;
// #endif
// #ifndef uint16
// typedef unsigned short uint16;
// #endif
// #ifndef sint32
// typedef signed long sint32;
// #endif
// #ifndef uint32
// typedef unsigned long uint32;
// #endif
// #ifndef sint64
// typedef signed long long sint64;
// #endif
// // #ifndef uint64
// // typedef unsigned long long uint64;
// // #endif
// #ifndef float32
// typedef float float32;
// #endif
// #ifndef float64
// typedef double float64;
// #endif

// #ifndef Rte_TypeDef_AlgoInterfaceVersionNumber_t
// typedef uint32 AlgoInterfaceVersionNumber_t;
// #define Rte_TypeDef_AlgoInterfaceVersionNumber_t
// #endif

#ifndef Rte_TypeDef_VEDSignalHeader_t
typedef struct {
    uint32 uiTimeStamp;
    uint16 uiMeasurementCounter;
    uint16 uiCycleCounter;
    uint8 eSigStatus;
} VEDSignalHeader_t;
#define Rte_TypeDef_VEDSignalHeader_t
#endif

#ifndef Rte_TypeDef_BSW_s_VEDCtrlData_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;  // Version number of
                                                   // interface
    VEDSignalHeader_t
        sSigHeader;  // information for synchronisation of SIL simulation
    uint32 uiSystemTimeStamp_us;
    uint16 CycleTime;  // component cycle time. Unit: us
    uint8 CaliMode;    // Calibration inhibition states. Unit: nu
    uint8 CtrlMode;    // Operating modes of VED component. Unit: nu
} BSW_s_VEDCtrlData_t;
#define Rte_TypeDef_BSW_s_VEDCtrlData_t
#endif
// typedef uint8         	State_array_t_4[16];

// typedef float ang_array_t[2];

// typedef float rat_array_t[2];

// typedef float rat_array_t_0[2];

// typedef float vel_array_t[2];

#ifndef Rte_TypeDef_VEDswa_t
typedef struct {
    ang_array_t ang;  // Value: [0.0, 1000.0]
    rat_array_t rat;  // Value: [14.52000046, 17]
} VEDswa_t;
#define Rte_TypeDef_VEDswa_t
#endif

#ifndef Rte_TypeDef_VEDvel_t
typedef struct {
    vel_array_t vel;    // Value: [0.0, 0.0]
    rat_array_t_0 rat;  // Value: [14.52000046, 14.52000046]
} VEDvel_t;
#define Rte_TypeDef_VEDvel_t
#endif

#ifndef Rte_TypeDef_VEDStRatio_t
typedef struct {
    VEDswa_t swa;
    VEDvel_t vel;
} VEDStRatio_t;
#define Rte_TypeDef_VEDStRatio_t
#endif
// typedef struct
//{

//} VEDVehParMain_t;

// typedef struct
//{
//	VEDVehParMain_t VehParMain;
//} VEDVehPar_t;

typedef uint8 State_array_t_0[32];

// V1_7
//  #ifndef Rte_TypeDef_VEDVehSigMain_t
typedef struct {
    State_array_t_0
        State;  // Value:[0,15,0,0,1,1,0,0,15,15,15,15,0,0,0,0,0,0,15,15,15,15,15,15,15,15,15,15,15,15,15,15]
    float YawRate;       // Yaw rate. Unit:	rad/s
    float YawRateTemp;   // Value:0
    float StWheelAngle;  // wheel angel.Unit: rad
    float LatAccel;      // Lateral acceleration. Unit: m/s2, Init Value: -10.23
    float WhlVelFrLeft;  // velocity of front left wheel. Unit: m/s, Value: 0.0
    float WhlVelFrRight;  // velocity of front right wheel. Unit: m/s, Value:
                          // 0.0
    float WhlVelReLeft;   // velocity of rear left wheel. Unit: m/s, Value: 0.0
    float WhlVelReRight;  // velocity of rear right wheel. Unit: m/s, Value: 0.0
    float YawRateInt;     // Unit: rad/s, Value: 0.0
    float YawRateIntTemp;   // Unit: degC, Value: 0.0
    float LongAccel;        // Longitudinal acceleration. Unit: m/s2, Value: 0
    float RearWhlAngle;     // Unit: rad, Value: 0.0
    float VehVelocityExt;   // Unit: m/s
    float VehLongAccelExt;  // Unit: m/s2, Init Value: -10.23
    uint8 WhlDirFrLeft;     // Unit: m/s
    uint8 WhlDirFrRight;    // Unit: m/s
    uint8 WhlDirReLeft;     // Unit: m/s
    uint8 WhlDirReRight;    // Unit: m/s
    unsigned char WhlTicksDevFrLeft;   // Value:0
    unsigned char WhlTicksDevFrRight;  // Value:0
    unsigned char WhlTicksDevReLeft;   // Value:0
    unsigned char WhlTicksDevReRight;  // Value:0
    uint8 ActGearPos;                  // Value:1
    unsigned short BrakeActLevel;      // Value:0
    uint8 ParkBrakeState;              // Value:0
    uint8 VehLongDirExt;               // Value:0
    uint8 VehLongMotStateExt;          // Value:0
    float CurveC0Ext;                  // Unit: 1/m, Value: 0.0
    float CurveC1Ext;                  // Value:0.0
    float SideSlipAngleExt;            // Unit: rad, Value: 0.0
    float fStWheelAngleGradient_nu;    // steer wheel angle gradient,unit: rad/s
} V1_7_VEDVehSigMain_t;
// #define Rte_TypeDef_VEDVehSigMain_t
// #endif

typedef uint8 State_array_t_1[52];
#ifndef Rte_TypeDef_VEDWheelHeightLevel_t
typedef struct {
    int16 FrontLeft;   // Height adjustment value for front left wheel.
                       // Unit: mm, Value: 0.0
    int16 FrontRight;  // Height adjustment value for front right wheel.
                       // Unit: mm, Value: 0.0
    int16 RearLeft;    // Height adjustment value for rear left wheel.
                       // Unit: mm, Value: 0.0
    int16 RearRight;   // Height adjustment value for rear right wheel.
                       // Unit: mm, Value: 0.0
} VEDWheelHeightLevel_t;
#define Rte_TypeDef_VEDWheelHeightLevel_t
#endif

#ifndef Rte_TypeDef_VEDVehSigAdd_t
typedef struct {
    State_array_t_1
        State;  //[15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15]
    float EnvTemp;  // Ambient environmental temperature. Unit: degC, Value: 0.0
    uint8 WiperState;  // Wiper blades activity. Unit: nu, Value: 0.0
    uint8 WiperStage;  // Wiper switch position. Unit: nu, Value: 0.0
    boolean
        WiperOutParkPos;  // Wiper outside park position. Unit: nu, Value: 0.0
    uint8
        WiperWasherFrontState;  // Tip-wipe / washer state. Unit: nu, Value: 0.0
    boolean RainSensor;         // Total driven distance. Unit: nu, Value: 0.0
    uint8
        TurnSignal;  // Turn signal switch state. Unit: nu, Value: 0：turn light
                     // off; 1: left turn light on; 2: right turn light on;
    boolean FogLampFront;  // Fog lamps front active. Unit: nu, Value: 0.0
    boolean FogLampRear;   // Fog lamps rear active. Unit: nu, Value: 0.0
    float RoadWhlAngFr;    // Steering angle (road wheel) at front axle. Unit:
                           // rad, Value: 0.0
    float RoadWhlAngRe;  // Steering angle (road wheel) at rear axle. Unit: rad,
                         // Value: 0.0
    float Odometer;      // Total driven distance. Unit: m, Value: 0.0
    uint8 GasPedalPos;   // FIXED POINT gas pedal position percentage. Unit: %,
                         // Value: 0.0
    boolean KickDown;    // BOOL Kick Down information. Unit: nu, Value: 0.0
    boolean BrakePedalPressed;  // BOOL brake pedal pressed. Unit: nu, Value:
                                // 0.0
    boolean DriverTired;  // BOOL Driver Tired Active. Unit: nu, Value: 0.0
    uint8 SpeedUnit;      // ENUM speedmeter speed unit. Unit: nu, Value: 0.0
    float SpeedoSpeed;    // FIXED POINT ego velocity speedo speed. Unit: km/h,
                          // Value: 0.0
    uint8 TrailerConnection;  // No trailer detected, Trailer detected, not
                              // defined, Trailer signal is not available.
                              // Unit:nu, Value: 0.0
    uint8 TrailerConnBeforeShutDown;  // Trailer Detection State from last Key
                                      // Cycle. Unit: nu, Value: 0.0
    unsigned char
        TrailerLengthInput;       // Length of attached trailer, received
                                  // from driver input. Unit: dm, Value: 0.0
    unsigned char ParkAidDet_L;   // Detection Distance of Rear Left Park Aid
                                  // Sensor (US). Unit: m, Value: 0.0
    unsigned char ParkAidDet_CL;  // Detection Distance of Rear Center Left Park
                                  // Aid Sensor (US). Unit: m, Value: 0.0
    unsigned char ParkAidDet_CR;  // Detection Distance of Rear Center Right
                                  // Park Aid Sensor (US). Unit: m, Value: 0.0
    unsigned char ParkAidDet_R;   // Detection Distance of Rear Right Park Aid
                                  // Sensor (US). Unit: m, Value: 0.0
    uint8 IgnitionSwitch;     // Ignition lockI, gnition off, Ignition accessory
                              // (15r) , Ignition on (15) , Ignition start (50).
                              // Unit: nu, Value: 0.0
    uint8 eSuspensionSystem;  // Availability of active suspension:  0: no
                              // active suspension available, 1 : active
                              // suspension available.Unit :nu, Value: 0.0
    uint8
        eHeightLevel;  // Height adjustment value the car: 0: no height level
                       // available,	1 : height level normal, 2 : height
                       // level low, 3 : height level high. Unit :nu, Value: 0.0
    VEDWheelHeightLevel_t WheelHeightLevel;  // Height level of each wheel
    boolean bDriverSeatBelt_nu;  // the state of driver seat belt. 0: drvier
                                 // seat belt open; 1: driver seat belt closed;
    boolean bDoorAllClosed_nu;   // the state of all the door. 0: at least one
                                 // door is openning; 1: all the door is closed;
    boolean bLowBeamActive_nu;   // the state of low beam light: 0: the low beam
                                 // is not active; 1: the low beam is active;
    boolean
        bHighBeamActive_nu;  // the state of high beam light: 0: the high beam
                             // is not active; 1: the high beam is active;
    boolean bHazardLightActive_nu;  // the state of hazard beam light: 0: the
                                    // hazard beam is not active; 1: the hazard
                                    // beam is active;
    float fBrakePedalPos_nu;  // FIXED POINT brake pedal position percentage.
                              // Unit: %, Value: 0.0
    uint8 uFrontLampState_nu;
    uint8 CDCS11_VoiceMode;
    uint8 CDCS11_ADAS_SndCtrlOnOff;
    float32 EPS1_TorsionBarTorque;
    uint8 CS1_GearPositionReqSt;
    uint8 EPS2_ADAS_Available;
    uint8 EPS2_ADAS_Active;
    float32 EPS2_MotCntrl;
    uint8 EPS2_ADAS_CtrlAbortFeedback;
    uint8 DDCU1_FLDoorAjar;
    uint8 DDCU1_RLDoorAjar;
    uint8 PDCU1_FRDoorAjar;
    uint8 PDCU1_RRDoorAjar;
    uint8 BDCS1_TrunkLockSts;
    uint8 BDCS1_HoodAjarSts;
    uint8 CDCS5_ResetAllSetup;
    uint8 CDCS5_FactoryReset;
} VEDVehSigAdd_t;
#define Rte_TypeDef_VEDVehSigAdd_t
#endif

typedef uint8 State_array_t_2[4];

#ifndef Rte_TypeDef_VEDPowerTrain_t
typedef struct {
    uint8 ActualGear;  // Indicates current used gear position. Unit: nu, Value:
                       // 0.0
    uint8 TargetGear;  // When a gear shift is in progress, this shows the
                       // targeted gear, otherwise it is equal to the Actual
                       // Gear. Unit: nu, Value: 0.0
    boolean EngineRunning;   // Indicates if engine running, needed for
                             // activation of ACC.  Unit: nu, Value: 0.0
    unsigned char FillByte;  // Byte to align VehSig.Powertrain structure. Unit:
                             // nu, Value: 0.0
    State_array_t_2 State;   //[15,15,15,15]
} VEDPowerTrain_t;
#define Rte_TypeDef_VEDPowerTrain_t
#endif

typedef uint8 State_array_t_3[13];

#ifndef Rte_TypeDef_VEDBrake_t
typedef struct {
    State_array_t_3 State;  // Value: [15,15]
    boolean ABSCtrl;        // Attenuation of secondary surface (two-way)   (0:
                            // inactive, 1 : active), Value: 0
    boolean TCSCtrl;  // Traction Control Activity (0: inactive, 1: active),
                      // Value: 0
    boolean bESCActive_nu;
    boolean bESCDisable_nu;
    boolean bTCSDisable_nu;
    uint8 IDB5_ABAavailable;
    uint8 IDB5_JerkActive;
    uint8 IDB5_Jerkfail;
    uint8 IDB5_AEB_Enable;
    boolean IDB7_ARPACTIVE;
    uint8 IDB7_ARPFailure;
    uint8 IDB1_FailedState;
    boolean bHDCEnable_nu;
} VEDBrake_t;
#define Rte_TypeDef_VEDBrake_t
#endif

// V1_7
//  #ifndef Rte_TypeDef_VEDVehSig_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;  // Number of used Version
    VEDSignalHeader_t sSigHeader;     // Contains time stamp, frame stamp, etc.
    V1_7_VEDVehSigMain_t VehSigMain;  // Main Vehicle dynamic sensor signals.
    VEDVehSigAdd_t VehSigAdd;  // Additional Vehicle dynamic sensor signals.
    VEDPowerTrain_t PowerTrain;
    VEDBrake_t Brake;  // Sensor specific parameters.
} V1_7_VEDVehSig_t;
// #define Rte_TypeDef_VEDVehSig_t
// #endif

#ifndef Rte_TypeDef_VEDNvStWhlAngCalc_t
typedef struct {
    float32 ZeroAngle;  // Steering wheel angle zero point offset value
    uint32 CalStatus;   // Steering wheel angle zero point offset status
} VEDNvStWhlAngCalc_t;
#define Rte_TypeDef_VEDNvStWhlAngCalc_t
#endif

#ifndef Rte_TypeDef_VEDNvSlfStGradCalc_t
typedef struct {
    float SlfStGrad;             // Understeer / Oversteer gradient value
    unsigned char SlfStGradMax;  // Understeer / Oversteer gradient max value
    unsigned char SlfStGradMin;  // Understeer / Oversteer gradient min value
    signed char CalStatus;       // Understeer / Oversteer gradient status
    unsigned char Dummy;         // Understeer / Oversteer gradient Dummy
} VEDNvSlfStGradCalc_t;
#define Rte_TypeDef_VEDNvSlfStGradCalc_t
#endif

#ifndef Rte_TypeDef_VEDYwRate_t
typedef struct {
    float32 ZeroRate;  // Yaw rate  zero point offset value. Unit: rad/s
    float32
        ZeroRateMin;  // Yaw rate zero point for minimum detection. Unit: rad/s
    float32
        ZeroRateMax;   // Yaw rate zero point for maximum detection. Unit: rad/s
    uint32 CalStatus;  // Yaw rate zero point offset status
} VEDYwRate_t;
#define Rte_TypeDef_VEDYwRate_t
#endif

#ifndef Rte_TypeDef_VEDLatAcc_t
typedef struct {
    float32 ZeroAccel;  // Lateral acceleration zero point offset estimation.
                        // Unit: m/s^2
    uint32 CalStatus;   // Lateral acceleration zero point offset status
} VEDLatAcc_t;
#define Rte_TypeDef_VEDLatAcc_t
#endif

#ifndef Rte_TypeDef_VEDVelCorr_t
typedef struct {
    float32 CorrFact;  // Corrections factors
    float32 Velo;      // Velocity of this hist. Unit: m/s
    float Dev;         // Confidence of correction factors
} VEDVelCorr_t;
#define Rte_TypeDef_VEDVelCorr_t
#endif

typedef VEDVelCorr_t VelCorr_array_t[3];

#ifndef Rte_TypeDef_VEDNvWldCalc_t
typedef struct {
    float Wld_front;                  // Wheel load dependency front axis
    float Wld_rear;                   // Wheel load dependency rear axis
    unsigned char Wld_front_quality;  // Wheel load dependency quality front
                                      // axis
    unsigned char Wld_rear_quality;   // Wheel load dependency quality rear axis
} VEDNvWldCalc_t;
#define Rte_TypeDef_VEDNvWldCalc_t
#endif

#ifndef Rte_TypeDef_VEDNvIoDatas_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    VEDSignalHeader_t sSigHeader;
    VEDNvStWhlAngCalc_t
        StWhlAng;  // Steering wheel angle zero point offset estimation
    VEDNvSlfStGradCalc_t
        SlfstGrad;       // Understeer / Oversteer gradient estimation
    VEDYwRate_t YwRate;  // Yaw rate  zero point offset value
    VEDLatAcc_t LatAcc;  // Lateral acceleration zero point offset estimation
    VelCorr_array_t VelCorr;  // Longitudinal velocity correction
    VEDNvWldCalc_t Wld;       // Wheel load dependency
    uint32 State;             // Read status respective write request
    uint8 checkSum;
} VEDNvIoDatas_t;
#define Rte_TypeDef_VEDNvIoDatas_t
#endif

typedef unsigned char u_Dummy_array_t_2[3];

#ifndef Rte_TypeDef_VEDALN_t_MonitoringScan_t
typedef struct {
    float f_Misalignment;
    float f_Std;
    boolean b_ResultsAvailable;
    u_Dummy_array_t_2 u_Dummy;
} VEDALN_t_MonitoringScan_t;
#define Rte_TypeDef_VEDALN_t_MonitoringScan_t
#endif

typedef VEDALN_t_MonitoringScan_t Azimuth_array_t[1];

typedef float a_CurrentAzimuthStd_array_t[1];

typedef unsigned char a_Dummy_array_t[3];

#ifndef Rte_TypeDef_VEDDirection_t
typedef struct {
    float f_DirConfidence;
    uint8 e_Direction;
    a_Dummy_array_t a_Dummy;
} VEDDirection_t;
#define Rte_TypeDef_VEDDirection_t
#endif

#ifndef Rte_TypeDef_VEDALN_Monitoring_t
typedef struct {
    AlgoInterfaceVersionNumber_t u_VersionNumber;
    VEDSignalHeader_t sSigHeader;
    Azimuth_array_t Azimuth;
    VEDALN_t_MonitoringScan_t Elevation;
    float f_ObstacleRangeMax;
    a_CurrentAzimuthStd_array_t a_CurrentAzimuthStd;
    float f_CurrentElevationStd;
    float f_EgoSpeed;
    float f_EgoSpeedStandardDeviation;
    float f_ConfirmationEgoSpeed;
    uint32 u_TimeStamp;
    uint8 e_AlignState;
    unsigned char u_UpdateCounter;
    unsigned char u_ConfirmationUpdateCounter;
    unsigned char u_Dummy;
    VEDDirection_t Direction;
} VEDALN_Monitoring_t;
#define Rte_TypeDef_VEDALN_Monitoring_t
#endif

// V1_7
//  #ifndef Rte_TypeDef_reqVEDPrtList_t
typedef struct {
    BSW_s_VEDCtrlData_t*
        pCtrl;  // operation mode, calibration mode and cycle time

    V1_7_VEDVehSig_t* pVehicleInputSignals;  // Vehicle input signals

    VEDNvIoDatas_t* pNVMRead;  // VED NVM read data

    VEDALN_Monitoring_t* pAln_Monitoring;  // Velocity information from ALN

} reqVEDPrtList_t;
// #define Rte_TypeDef_reqVEDPrtList_t
// #endif

#ifndef Rte_TypeDef_VEDMotVarVehDyn_t
typedef struct {
    float Velocity;
    float Accel;
    float varVelocity;
    float varAccel;
} VEDMotVarVehDyn_t;
#define Rte_TypeDef_VEDMotVarVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDVeloCorrVehDyn_t
typedef struct {
    float corrFact;
    float corrVar;
    float corrVelo;
    float corrVeloVar;
    float minVelo;
    float maxVelo;
    uint8 corrQual;
    boolean bRollerTestBench;
} VEDVeloCorrVehDyn_t;
#define Rte_TypeDef_VEDVeloCorrVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDAccelCorrVehDyn_t
typedef struct {
    float corrAccel;
    float corrAccelVar;
} VEDAccelCorrVehDyn_t;
#define Rte_TypeDef_VEDAccelCorrVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDLongitudinal_t
typedef struct {
    VEDMotVarVehDyn_t MotVar;
    VEDVeloCorrVehDyn_t VeloCorr;
    VEDAccelCorrVehDyn_t AccelCorr;
    float fBrakePedalPosGrad_nu;  // Time derivative of brake pedal position
    float fGasPedalPosGrad_nu;    // Time derivative of gas pedal position
} VEDLongitudinal_t;
#define Rte_TypeDef_VEDLongitudinal_t
#endif

#ifndef Rte_TypeDef_VEDYawRateVehDyn_t
typedef struct {
    float YawRate;   // Estimated vehicle angular rate about vertical axis
                     // (offset compensated). Unit: rad/s
    float Variance;  // Estimated vehicle yaw rate variance. Unit: (rad/s)^2
    float Quality;
} VEDYawRateVehDyn_t;
#define Rte_TypeDef_VEDYawRateVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDDrvIntCurveVehDyn_t
typedef struct {
    float Curve;     // Driver intended curvature (derived from steering wheel
                     // angle), related to vehicle CoG
    float Variance;  // Driver intended curvature variance. Unit: N/rad
    float
        Gradient;  // Time derivative of driver intended curvature. Unit: N/rad
} VEDDrvIntCurveVehDyn_t;
#define Rte_TypeDef_VEDDrvIntCurveVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDCurveVehDyn_t
typedef struct {
    float Curve;  // Driven vehicle curve as inverse radius (related to vehicle
                  // CoG)
    float C1;
    float Gradient;
    float varC0;
    float varC1;
    float Quality;
    float CrvError;         // Lateral error of curvature
    unsigned char CrvConf;  // Confidence of curvature
} VEDCurveVehDyn_t;
#define Rte_TypeDef_VEDCurveVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDSideSlipVehDyn_t
typedef struct {
    float SideSlipAngle;  // Vehicle body side slip angle, related to CoG
    float Variance;       // Vehicle side slip angle variance
} VEDSideSlipVehDyn_t;
#define Rte_TypeDef_VEDSideSlipVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDLatAccelVehDyn_t
typedef struct {
    float LatAccel;  // Acceleration perpendicular to travel direction (road
                     // bank angle compensated), related to vehicle CoG
    float Variance;  // Lateral acceleration variance
} VEDLatAccelVehDyn_t;
#define Rte_TypeDef_VEDLatAccelVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDLateral_t
typedef struct {
    VEDYawRateVehDyn_t YawRate;  // Vehicle yaw rate
    float OffCompStWheelAngle;   // Zero point offset compensated Steering wheel
                                 // angle
    VEDCurveVehDyn_t Curve;      // Curve
    VEDDrvIntCurveVehDyn_t DrvIntCurve;  // Driver intended curvature
    VEDLatAccelVehDyn_t Accel;           // Lateral acceleration
    VEDSideSlipVehDyn_t SlipAngle;       // Vehicle body side slip angle
    float32 fStWheelAngleGradient_rps;   // the gradient of steer wheel angle.
                                         // unit: rad per second
} VEDLateral_t;
#define Rte_TypeDef_VEDLatAccelVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDMotionStateVehDyn_t
typedef struct {
    uint8 MotState;
    float Confidence;
} VEDMotionStateVehDyn_t;
#define Rte_TypeDef_VEDLatAccelVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDLegacyVehDyn_t
typedef struct {
    float YawRateMaxJitter;
    boolean bStandStill;
} VEDLegacyVehDyn_t;
#define Rte_TypeDef_VEDLegacyVehDyn_t
#endif

// typedef uint8 State_array_t[12];

#ifndef Rte_TypeDef_VEDAddCalculateSign_t
typedef struct {
    float SpeedOdometerCalculate_mps;
} VEDAddCalculateSign_t;
#define Rte_TypeDef_VEDAddCalculateSign_t
#endif

#ifndef Rte_TypeDef_VEDVehDyn_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    VEDSignalHeader_t sSigHeader;
    VEDLongitudinal_t Longitudinal;      // Longitudinal motion variables
    VEDLateral_t Lateral;                // Lateral motion variables
    VEDMotionStateVehDyn_t MotionState;  // Lateral motion variables
    VEDLegacyVehDyn_t Legacy;
    State_array_t State;
    VEDAddCalculateSign_t AddCalculateSign;
} VEDVehDyn_t;
#define Rte_TypeDef_VEDVehDyn_t
#endif

#ifndef Rte_TypeDef_VEDSignalInputErrors_t
typedef struct {
    uint8 InputSignalError;
    uint8 InputSignalPeakError;
} VEDSignalInputErrors_t;
#define Rte_TypeDef_VEDSignalInputErrors_t
#endif

#ifndef Rte_TypeDef_VEDParInputErrors_t
typedef struct {
    uint8 InputParameterError;
} VEDParInputErrors_t;
#define Rte_TypeDef_VEDParInputErrors_t
#endif

#ifndef Rte_TypeDef_VEDOutPutErrors_t
typedef struct {
    uint8 YwrOffsRg;       // Yaw rate offset range error
    uint8 SwaOffsRg;       // Steering wheel angle offset range error
    uint8 AyOffsRg;        // Lateral acceleration offset range error
    uint8 VelCorrRg;       // Velocity correction factor range error
    uint8 NVMYwrOffsRg;    // Eeprom Yaw rate offset range error
    uint8 NVMSwaOffsRg;    // Eeprom Steering wheel angle offset range error
    uint8 NVMAyOffsRg;     // EEprom Lateral acceleration offset range error
    uint8 NVMVelCorrRg;    // EEprom Velocity correction factor range error
    uint8 VelCorrWin;      // Velocity correction factor window error
    uint8 VelMon;          // Velocity monitoring error
    uint8 VelMonLt;        // Velocity monitoring long-term error
    uint8 VelocityErr;     // Velocity out of physical range error
    uint8 YawRateErr;      // Yaw rate out of physical range error
    uint8 YwrMonVehHalt;   // Yaw rate monitoring during vehicle halt
    uint8 YwrMonVehDOff;   // Yaw rate monitoring during vehicle drive off
    uint8 YwrMonAdmWdrwl;  // Yaw rate monitoring at admission and withdrawal of
                           // vehicle standstill
    uint8 YwrACCMonAlignm;    // ACC Yaw rate offset monitoring over alignment
    uint8 YwrCGMonAlignm;     // CG Yaw rate offset monitoring over alignment
    uint8 YwrMonVehHaltCal;   // Yaw rate calibration monitoring during vehicle
                              // halt
    uint8 VED__FS_YR_VS_WSP;  // Fusi error, difference between yaw rate and
                              // wheel speed yaw rate to high
    uint8 VED_FS_YR_VS_AY;    // Fusi error, difference between yaw rate and
                              // lateral acceleration yaw rate to high
    uint8 VED__FS_YR_VS_SWA;  // Fusi error, difference between yaw rate and
                              // steering wheel yaw rate to high
    uint8 VED_VEH_VEL_NOT_AVAILABLE;  // Global error state for vehicle velocity
                                      // not available
    uint8 VED__VEH_YWR_NOT_AVAILABLE;  // Global error state for yaw rate not
                                       // availabel
    uint8 VED_FS_VEH_CORR_MON;  // Fusi error, velocity difference between
                                // external velocity and velocity from
                                // stationary objects too high
} VEDOutPutErrors_t;
#define Rte_TypeDef_VEDOutPutErrors_t
#endif

#ifndef Rte_TypeDef_VED_Errors_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    VEDSignalHeader_t sSigHeader;
    VEDSignalInputErrors_t SignalInputErrors;  // Signal input errors
    VEDParInputErrors_t ParInputErrors;        // Parameter input errors
    VEDOutPutErrors_t OutPutErrors;  // Output values and internalal errors
} VED_Errors_t;
#define Rte_TypeDef_VED_Errors_t
#endif

#ifndef Rte_TypeDef_VEDYwr_t
typedef struct {
    float32 StandStillOffset;  // Stand still Yaw rate offset
    sint32 StandStillState;    // Stand still Yaw rate offset state
    float32 DynOffset;         // Dynamic Yaw rate offset
    float32 DynVariance;       // Dynamic Yaw rate offset Variance
} VEDYwr_t;
#define Rte_TypeDef_VEDYwr_t
#endif

#ifndef Rte_TypeDef_VEDSwa_t
typedef struct {
    float32 Offset;    // Steering wheel angle offset. Unit: rad
    float32 Variance;  // Steering wheel angle offset variance
    sint32 State;      // Steering wheel angle offset state
} VEDSwa_t;
#define Rte_TypeDef_VEDSwa_t
#endif

#ifndef Rte_TypeDef_VEDAy_t
typedef struct {
    float32 Offset;    // Steering wheel angle offset. Unit: rad
    float32 Variance;  // Steering wheel angle offset variance
    sint32 State;      // Steering wheel angle offset state
} VEDAy_t;
#define Rte_TypeDef_VEDAy_t
#endif

#ifndef Rte_TypeDef_VED_VEDOffsets_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    VEDSignalHeader_t sSigHeader;
    VEDYwr_t Ywr;  // Yaw rate offset result
    VEDSwa_t Swa;  // Steering wheel angle offset result
    VEDAy_t Ay;    // Lateral accelerate offset result
} VED_VEDOffsets_t;
#define Rte_TypeDef_VED_VEDOffsets_t
#endif

#ifndef Rte_TypeDef_VEDWhlCurve_t
typedef struct {
    float32 Curve;  // Curve. Unit: rad/s
} VEDWhlCurve_t;
#define Rte_TypeDef_VEDWhlCurve_t
#endif

#ifndef Rte_TypeDef_VEDWhlFrCurve_t
typedef struct {
    float32 Curve;  // Curve. Unit: rad/s
} VEDWhlFrCurve_t;
#define Rte_TypeDef_VEDWhlFrCurve_t
#endif

#ifndef Rte_TypeDef_VEDWhlReCurve_t
typedef struct {
    float32 Curve;  // Curve. Unit: rad/s
} VEDWhlReCurve_t;
#define Rte_TypeDef_VEDWhlReCurve_t
#endif

#ifndef Rte_TypeDef_VEDYwRateCurve_t
typedef struct {
    float32 Curve;  // Curve. Unit: rad/s
} VEDYwRateCurve_t;
#define Rte_TypeDef_VEDYwRateCurve_t
#endif

#ifndef Rte_TypeDef_VEDAyCurve_t
typedef struct {
    float32 Curve;  // Curve. Unit: rad/s
} VEDAyCurve_t;
#define Rte_TypeDef_VEDAyCurve_t
#endif

#ifndef Rte_TypeDef_VEDSwaCurve_t
typedef struct {
    float32 Curve;  // Curve. Unit: rad/s
} VEDSwaCurve_t;
#define Rte_TypeDef_VEDSwaCurve_t
#endif

#ifndef Rte_TypeDef_VEDDrvIntCurve_t
typedef struct {
    float32 Curve;  // Curve. Unit: rad/s
} VEDDrvIntCurve_t;
#define Rte_TypeDef_VEDDrvIntCurve_t
#endif

#ifndef Rte_TypeDef_VEDVehYawCurve_t
typedef struct {
    float32 Curve;  // Curve. Unit: rad/s
} VEDVehYawCurve_t;
#define Rte_TypeDef_VEDVehYawCurve_t
#endif

#ifndef Rte_TypeDef_VED_EstCurves_t
typedef struct {
    AlgoInterfaceVersionNumber_t uiVersionNumber;
    VEDSignalHeader_t sSigHeader;
    VEDWhlCurve_t Whl;        // Wheel speed curve all axles
    VEDWhlFrCurve_t WhlFr;    // Wheel speed curve front axle
    VEDWhlReCurve_t WhlRe;    // Wheel speed curve rear axle
    VEDYwRateCurve_t YwRate;  // Yaw rate sensor curve
    VEDAyCurve_t Ay;          // Lateral acceleration curve
    VEDSwaCurve_t Swa;        // Steering angle yaw rate curve
    VEDDrvIntCurve_t DrvInt;  // Steering wheel angle curve
    VEDVehYawCurve_t VehYaw;  // Vehicle yaw rate curve
} VED_EstCurves_t;
#define Rte_TypeDef_VED_EstCurves_t
#endif

#ifndef Rte_TypeDef_proVEDPrtList_t
typedef struct {
    VEDVehDyn_t* pVehicleDynamicSignals;  // Vehicle Dynamic signals

    VEDNvIoDatas_t* pNVMWrite;  // VED NVM write data
    VED_Errors_t* pVED_Errors;  // ved_ errors, input signals/parameters and
                                // internal errors
    VED_VEDOffsets_t*
        pVED_Offsets;  // ved_ offsets, for yaw rate steering wheel angle and
                       // lateral acceleration sensor

    VED_EstCurves_t* pVED_EstCurves;          // VED estimated curves
    V1_7_VEDVehSig_t* pOriginVehicleSignals;  // Vehicle input signals
} proVEDPrtList_t;
#define Rte_TypeDef_proVEDPrtList_t
#endif

#ifndef Rte_TypeDef_reqVEDParams_t
typedef struct {
    // VEDVehPar_t* pVehicleParameter;//VehicleParameter data
    // State_array_t_4 State;
    float
        VED_Kf_SelfSteerGrad_nu;  // Rate of change in steering wheel angle with
                                  // respect to change in steady-state lateral
                                  // acceleration. Unit: nu, Value: 0.003734
    VEDStRatio_t SteeringRatio;
    float VED_Kf_WheelBase_met;  // Longitudinal distance between  the center of
                                 // tire contact of pair of wheels on same
                                 // vehicle side. Unit: m, Value: 2.9000001
    float
        VED_Kf_TrackWidthFront_met;  // Lateral distance between the center of
                                     // tire contact of pair of wheels on front
                                     // vehicle axle. Unit: m, Value: 1.69400001
    float
        VED_Kf_TrackWidthRear_met;  // Lateral distance between the center of
                                    // tire contact of pair of wheels on rear
                                    // vehicle axle. Unit: m, Value: 1.72000003
    float VED_Kf_VehWeight_kg;      // Vehicle weight. Unit: kg, Value:
    float
        VED_Kf_CntrOfGravHeight_met;  // Imaginary point where the total weight
                                      // of the body may be thought to be
                                      // concentrated. Unit: m, Value: 0.63422
    float VED_Kf_AxisLoadDistr_per;   // The ratio of the vertical load at rear
                                      // axle to total vehicle vertical load.
                                      // Unit: %, Value: 0.4
    float VED_Kf_WhlLoadDepFrontAxle_nu;  // Change of difference in wheel
                                          // circumferential speed resulting
                                          // from this load transfer during
                                          // cornering at front axle. Unit: nu,
                                          // Value: 3.6329999
    float VED_Kf_WhlLoadDepRearAxle_nu;   // Change of difference in wheel
                                         // circumferential speed resulting from
                                         // this load transfer during cornering
                                         // at rear axle. Unit: nu ,
                                         // Value: 2.92000008
    float VED_Kf_WhlCircumference_met;  // Dynamic wheel rolling circumference.
                                        // Unit: m, Value: 2.16770005
    unsigned char VED_Ku_WhlTcksPerRev_nu;  // Number of sensor pulses per wheel
                                            // revolution. Unit: nu, Value: 96.0
} reqVEDParams_t;
#define Rte_TypeDef_reqVEDParams_t
#endif

/*****************************************************************************
FUNCTION PROTOTYPES
*****************************************************************************/
void VEDExec(const reqVEDPrtList_t* reqPorts,
             const reqVEDParams_t* reqParams,
             proVEDPrtList_t* proPorts);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
