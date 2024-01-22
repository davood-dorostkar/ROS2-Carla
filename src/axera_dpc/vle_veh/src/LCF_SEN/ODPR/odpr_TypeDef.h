#ifndef ODRP_TYPEDEFH
#define ODRP_TYPEDEFH
#ifdef __cplusplus
extern "C" {
#endif
#include "odpr_ext.h"
#include "odpr_polyfitTgtObjClothoid.h"
typedef struct {
    float32 sf_PosY0_met;
    float32 sf_HeadingAngle_rad;
    float32 sf_Crv_1pm;
    float32 sf_CrvChng_1pm2;
    float32 sf_Length_met;

    float32 sf_PosY0StdDev_met;
    float32 sf_HeadingAngleStdDev_rad;
    float32 sf_CrvStdDev_1pm;
    float32 sf_CrvChngStdDev_1pm2;
    float32 sf_VehYawRateStdDev_radps;

    float32 sf_VehVelX_mps;
    float32 sf_VehYawRate_radps;

    float32 sf_DeltaT_sec;
    uint8 sf_LaneDataValid_bool;
    uint8 sf_DegradedUpdate_bool;
    uint8 sf_OverallMeasurementQuality_perc;

    uint8 sf_LaneChange_bool;

    float32 sf_LaneKFErrCoeff1_met;
    float32 sf_LaneKFErrCoeff2_mps;
    float32 sf_LaneKFInitRFactor_nu;
    float32 sf_LaneKFDegradeWeight_nu;
    uint8 sf_LaneKFMnUpdateQual_perc;
    uint8 sf_LaneKFMnInitQual_perc;
    float32 sf_LaneKFIncQual_1ps;
    float32 sf_LaneKFDecQualDeg_1ps;
    float32 sf_LaneKFDecQualPred_1ps;
    float32 sf_LaneKFKGainFac_nu;
    float32 sf_LaneKFDynYawFactor_nu;
    float32 sf_LaneKFDynDistYFact_nu;
    float32 sf_LaneKFDynCrvFact_nu;
    float32 sf_LaneKFDynCrvRateFact_nu;
} laneKFInTypeV3;

//  Output
typedef struct {
    float32 sf_PosY0_met;
    float32 sf_HeadingAngle_rad;
    float32 sf_Crv_1pm;
    float32 sf_CrvChng_1pm2;
    uint8 sf_KFStatus_btf;
    uint8 sf_QualityMeasure_perc;
} laneKFOutType;

#ifdef __cplusplus
}
#endif
#endif 