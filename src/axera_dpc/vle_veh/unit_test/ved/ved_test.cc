#include "gtest/gtest.h"
#include "ved_ext.h"

TEST(VedExec, Test1) {
    BSW_s_VEDCtrlData_t pCtrl{0};              // operation mode, calibration mode and cycle time
    V1_7_VEDVehSig_t pVehicleInputSignals{0};  // Vehicle input signals
    VEDNvIoDatas_t pNVMRead{0};                // VED NVM read data
    VEDALN_Monitoring_t pAln_Monitoring{0};    // Velocity information from ALN

    reqVEDPrtList_t reqPorts = {&pCtrl, &pVehicleInputSignals, &pNVMRead, &pAln_Monitoring};

    reqVEDParams_t reqParams{0};

    VEDVehDyn_t pVehicleDynamicSignals{0};      // Vehicle Dynamic signals
    VEDNvIoDatas_t pNVMWrite{0};                // VED NVM write data
    VED_Errors_t pVED_Errors{0};                // ved_ errors, input signals/parameters and
                                                // internal errors
    VED_VEDOffsets_t pVED_Offsets{0};           // ved_ offsets, for yaw rate steering wheel angle and
                                                // lateral acceleration sensor
    VED_EstCurves_t pVED_EstCurves{0};          // VED estimated curves
    V1_7_VEDVehSig_t pOriginVehicleSignals{0};  // Vehicle input signals

    proVEDPrtList_t proPorts = {&pVehicleDynamicSignals, &pNVMWrite,      &pVED_Errors,
                                &pVED_Offsets,           &pVED_EstCurves, &pOriginVehicleSignals};

    VEDExec(&reqPorts, &reqParams, &proPorts);

    EXPECT_TRUE(1);
}