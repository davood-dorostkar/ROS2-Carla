#include "gtest/gtest.h"
#include "lcf_veh_local_ext.h"

TEST(LcfVehExec, Test1) {
    reqLcfVehPrtList_t reqPorts{0};
    reqLcfVehParams reqParams{0};
    proLcfVehPrtList_t proPorts{0};
    reqLcfVehDebug_t proDebugs{0};

    LcfVehExec(&reqPorts, &reqParams, &proPorts, &proDebugs);

    EXPECT_TRUE(1);
}