/**********************************Model
 Property********************************
 *
 * Company             : SENSETIME
 *
 * Tool Version        : Ver2.0
 *
 * Model Name          : LCCRA
 *
 * Model Long Name     : LCCRA

 *

 * Model Advisor       : Not Check

 *

 * Model Version       : Ver1.0

 *

 * Model Author        : ZhuHe

 *

 * Model Reviewer      :

 *

 * Model Review Data   :

 *

 * Model Cycle Time    : 50ms


 ************************************Auto
 Coder**********************************
 *
 * File                             : LCCRA_types.h
 *
 * FileType                         : Code Header File
 *
 * Real-Time Workshop file version  : 9.4 (R2020b) 29-Jul-2020
 *
 * TLC version                      : 9.4 (Aug 20 2020)
 *
 * C source code generated on       : Fri Feb  3 11:44:07 2023
 *
 * Copyright (C) by SenseTime Group Limited. All rights reserved.
 *******************************************************************************/

#ifndef RTW_HEADER_LCCRA_types_h_
#define RTW_HEADER_LCCRA_types_h_
#include "rtwtypes.h"

/* Model Code Variants */
#ifndef DEFINED_TYPEDEF_FOR_BusMIO_
#define DEFINED_TYPEDEF_FOR_BusMIO_

typedef struct {
    int32_T ID;
    real32_T Position[2];
    real32_T Velocity[2];
    real32_T Acceleration[2];
    real32_T Length;
    real32_T Width;
    real32_T RangeMagnitude;
    real32_T VelocityMagnitude;
    real32_T AccelerationMagnitude;
    real32_T TTC;
    real32_T TimeGap;
    uint8_T Type;
} BusMIO;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusMIOs_
#define DEFINED_TYPEDEF_FOR_BusMIOs_

typedef struct {
    uint8_T NumMIOs;
    BusMIO MIOObjects[10];
} BusMIOs;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusVehPara_
#define DEFINED_TYPEDEF_FOR_BusVehPara_

typedef struct {
    int32_T ID;
    real32_T Position[2];
    real32_T Velocity[2];
    real32_T Acceleration[2];
    real32_T Length;
    real32_T Width;
    uint8_T Type;
} BusVehPara;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusVehicle_
#define DEFINED_TYPEDEF_FOR_BusVehicle_

typedef struct {
    uint8_T NumTgtVeh;
    BusVehPara Vehicles[40];
} BusVehicle;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusObjPara_
#define DEFINED_TYPEDEF_FOR_BusObjPara_

typedef struct {
    int32_T ID;
    real32_T Position[2];
    real32_T Velocity[2];
    real32_T Acceleration[2];
    real32_T Length;
    real32_T Width;
    real32_T HeadingAngle;
    uint8_T Classification;
    uint32_T Sensorsource;
    real32_T Existence;
} BusObjPara;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusObject_
#define DEFINED_TYPEDEF_FOR_BusObject_

typedef struct {
    uint8_T NumTgtObj;
    BusObjPara Objects[40];
} BusObject;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusLaneSensePara_
#define DEFINED_TYPEDEF_FOR_BusLaneSensePara_

typedef struct {
    real32_T PosY0;
    real32_T HeadingAngle;
    real32_T Curvature;
    real32_T CurvatureDerivative;
    boolean_T Valid;
} BusLaneSensePara;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusConstpara_
#define DEFINED_TYPEDEF_FOR_BusConstpara_

typedef struct {
    real32_T LCCRA_MinHeadingAngle_C_rad;
    real32_T LCCRA_MaxHeadingAngle_C_rad;
    real32_T LCCRA_FrontTTCThreshold_C_sec;
    real32_T LCCRA_RearTTCThreshold_C_sec;
    real32_T LCCRA_NextTTCThreshold_C_sec;
    real32_T LCCRA_TimeGapThreshold_C_sec;
    real32_T LCCRA_TimeThreshold_C_sec;
    real32_T LCCRA_VelocityThreshold_C_mps;
    real32_T LCCRA_TTCSafeDistance_C_met;
    real32_T LCCRA_RadarExistThreshold_C_nu;
    real32_T LCCRA_CamExistThreshold_C_nu;
    real32_T LCCRA_LidarExistThreshold_C_nu;
    real32_T LCCRA_DefaultLaneWidth_C_met;
    real32_T LCCRA_SelectMIOMaxDis_C_met;
    real32_T LCCRA_PredictionTime_K_nu;
    real32_T LCCRA_PredictionTime_M_nu;
    real32_T LCCRA_MaxPredictionTime_C_sec;
    real32_T LCCRA_MinPredictionTime_C_sec;
    uint8_T LCCRA_MaxWeightThreshold_C_nu;
    real32_T LCCRA_DelayTimeValue_C_nu[10];
    real32_T LCCRA_MaxFlagTime_C_sec;
    real32_T LCCRA_MaxFlagRiskTime_C_sec;
    real32_T LCCRA_RiskDistance_C_met;
    real32_T LCCRA_RiskDistanceX_C_met;
    real32_T LCCRA_FrontTimeGapThreshold_C_sec;
    real32_T LCCRA_RearTimeGapThreshold_C_sec;
    real32_T LCCRA_TTCThresholdHys_C_sec;
    real32_T LCCRA_TimeGapThresholdHys_C_sec;
    real32_T LCCRA_RiskDistThresholdHys_C_met;
    real32_T LCCRA_EgoLineOffset_C_met;
    boolean_T LCCRA_UseObjLength_bool;
    real32_T LCCRA_EgoFrontTTCThreshold_C_sec;
    real32_T LCCRA_EgoRearHangDist_C_met;
    real32_T LCCRA_FrontTTCThresholdBus_C_sec;
    real32_T LCCRA_RearTTCThresholdBus_C_sec;
    real32_T LCCRA_EgoFrontTTCThresholdBus_C_sec;
    real32_T LCCRA_FrontTimeGapThresholdBus_C_sec;
    real32_T LCCRA_RearTimeGapThresholdBus_C_sec;
} BusConstpara;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusSenseLane_
#define DEFINED_TYPEDEF_FOR_BusSenseLane_

typedef struct {
    BusLaneSensePara Left;
    BusLaneSensePara Right;
} BusSenseLane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusEgo_
#define DEFINED_TYPEDEF_FOR_BusEgo_

typedef struct {
    real32_T Velocity;
    real32_T YawRate;
    real32_T Curvature;
    real32_T CurvatureDerivative;
} BusEgo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusLanePara_
#define DEFINED_TYPEDEF_FOR_BusLanePara_

typedef struct {
    real32_T PosY0;
    real32_T HeadingAngle;
    real32_T Curvature;
    real32_T CurvatureDerivative;
} BusLanePara;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusLane_
#define DEFINED_TYPEDEF_FOR_BusLane_

typedef struct {
    BusLanePara Left;
    BusLanePara Right;
} BusLane;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusDebugMIOs_
#define DEFINED_TYPEDEF_FOR_BusDebugMIOs_

typedef struct {
    BusMIO DebugMIOs[36];
} BusDebugMIOs;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusWeight_
#define DEFINED_TYPEDEF_FOR_BusWeight_

typedef struct {
    uint8_T LCCRA_egofrontweight_nu;
    uint8_T LCCRA_egorearweight_nu;
    uint8_T LCCRA_leftfrontweight_nu;
    uint8_T LCCRA_leftrearweight_nu;
    uint8_T LCCRA_rightfrontweight_nu;
    uint8_T LCCRA_rightrearweight_nu;
    uint8_T LCCRA_nextleftfrontweight_nu;
    uint8_T LCCRA_nextleftrearweight_nu;
    uint8_T LCCRA_nextrightfrontweight_nu;
    uint8_T LCCRA_nextrightrearweight_nu;
} BusWeight;

#endif

#ifndef DEFINED_TYPEDEF_FOR_BusDelayTime_
#define DEFINED_TYPEDEF_FOR_BusDelayTime_

typedef struct {
    real32_T LCCRA_egofrontdelaytime_sec;
    real32_T LCCRA_egoreardelaytime_sec;
    real32_T LCCRA_leftfrontdelaytime_sec;
    real32_T LCCRA_leftreardelaytime_sec;
    real32_T LCCRA_rightfrontdelaytime_sec;
    real32_T LCCRA_rightreardelaytime_sec;
    real32_T LCCRA_nextleftfrontdelaytime_sec;
    real32_T LCCRA_nextleftreardelaytime_sec;
    real32_T LCCRA_nextrightfrontdelaytime_sec;
    real32_T LCCRA_nextrightreardelaytime_sec;
} BusDelayTime;

#endif

#ifndef DEFINED_TYPEDEF_FOR_E_LCCRA_ObjectClassification_nu_
#define DEFINED_TYPEDEF_FOR_E_LCCRA_ObjectClassification_nu_

typedef enum {
    Vehicle = 0, /* Default value */
    Bus,
    Pedestrian,
    Bike,
    UnknownMovable,
    UnknownStatic
} E_LCCRA_ObjectClassification_nu;

#endif
#endif /* RTW_HEADER_LCCRA_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
