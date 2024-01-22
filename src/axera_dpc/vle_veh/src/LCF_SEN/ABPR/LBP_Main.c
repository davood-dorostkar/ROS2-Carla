/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wangsifan <wangsifan@senseauto.com>
 */
#include "tue_common_libs.h"
#include "LBP_Main.h"
#include "LBP_Ext.h"
#include "string.h"

/****************************************************************************************
    @fn           LCF_LBP_Reset
    @brief        Any Boundary Processing Lane Boundary Reset
    @description  LCF_LBP_Reset:Initialization function
    @param[in]    void
    @return       void
    @startuml
    (*)-->Reset function
       -->(*)
    @enduml
 ******************************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static sULPOutput_t sULPOutput = {0};
static sCLPOutput_t sCLPOutput = {0};
static sLFPOutput_t sLFPOutput = {0};
static sELGOutput_t sELGOutput = {0};
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

void LCF_LBP_Reset(void) {
    memset(&sULPOutput, 0, sizeof(sULPOutput_t));
    memset(&sCLPOutput, 0, sizeof(sCLPOutput_t));
    memset(&sLFPOutput, 0, sizeof(sLFPOutput_t));
    memset(&sELGOutput, 0, sizeof(sELGOutput_t));
    UncoupledLaneReset();
    CheckLaneProperityReset();
    LaneFilterReset();
    EgoLaneGenerationReset();
}

/****************************************************************************************
    @fn           LCF_LBP_Exec()
    @brief        Ego lane boundary processing
    @description  LBPProcess:
                      1.Uncoupled lane processing;
                      2.Check lane properity;
                      3.Lane filter processing;
                      4.Ego lane generation.
    @param[in]    pLBPInput : Input for LBPProcess
    @param[in]    pLBPParam : Parameter for LBPProcess
    @param[out]   pLBPOutput: Output for LBPProcess
    @param[out]   pLBPDebug : Debug(measurement) for LBPProcess
    @return       void
    @startuml
    title LCF_LBP_Exec
    (*)--> 1.UncoupledLaneProcessing
       --> 2.CheckLaneProperity
       --> 3.LaneFilterProcessing
       --> 4.EgoLaneGeneration
       --> LBPout
       --> (*)
       1.UncoupledLaneProcessing --> 3.LaneFilterProcessing
       1.UncoupledLaneProcessing --> 4.EgoLaneGeneration
       2.CheckLaneProperity --> 4.EgoLaneGeneration
    (*) --> 2.CheckLaneProperity
    @enduml
 ******************************************************************************************/
void LCF_LBP_Exec(const sLBPInput_t *pLBPInput,
                  const sLBPParam_t *pLBPParam,
                  sLBPOutput_t *pLBPOutput,
                  sLBPDebug_t *pLBPDebug) {
    //  printf("--------------------LBP--------------------\n");
    /* Define ULP module variables */
    sULPInput_t sULPInput = {0};
    sULPParam_t sULPParam = {0};

    sULPDebug_t sULPDebug = {0};

    /* Define CLP module variables */
    sCLPInput_t sCLPInput = {0};
    sCLPParam_t sCLPParam = {0};
    sCLPDebug_t sCLPDebug = {0};

    /* Define LFP module variables */
    sLFPInput_t sLFPInput = {0};
    sLFPParam_t sLFPParam = {0};
    sLFPDebug_t sLFPDebug = {0};

    /* Define ELG module variables */
    sELGInput_t sELGInput = {0};
    sELGParam_t sELGParam = {0};
    sELGDebug_t sELGDebug = {0};

    /***************************1.Uncoupled lane
     * processing********************************/
    INPUT_UncoupledLaneProcessing(pLBPInput, pLBPParam, &sCLPOutput,
                                  &sLFPOutput, &sULPInput, &sULPParam);

    UncoupledLaneProcessing(&sULPInput, &sULPParam, &sULPOutput, &sULPDebug);

    /***************************2.Check lane
     * properity*************************************/
    INPUT_CheckLaneProperity(pLBPInput, pLBPParam, &sULPOutput, &sLFPOutput,
                             &sCLPInput, &sCLPParam);

    CheckLaneProperity(&sCLPInput, &sCLPParam, &sCLPOutput, &sCLPDebug);

    /***************************3.Lane filtering
     * processing********************************/
    INPUT_LaneFilterProcessing(pLBPInput, pLBPParam, &sULPOutput, &sCLPOutput,
                               &sLFPInput, &sLFPParam);

    LaneFilterProcessing(&sLFPInput, &sLFPParam, &sLFPOutput, &sLFPDebug);

    /***************************4.Ego lane
     * generation**************************************/
    INPUT_EgoLaneGeneration(pLBPInput, pLBPParam, &sULPOutput, &sCLPOutput,
                            &sLFPOutput, &sELGInput, &sELGParam);

    EgoLaneGeneration(&sELGInput, &sELGParam, &sELGOutput, &sELGDebug);

    /**************************************output and
     * debug********************************/
    LBPOutput(&sULPOutput, &sULPDebug, &sCLPOutput, &sCLPDebug, &sLFPOutput,
              &sLFPDebug, &sELGOutput, &sELGDebug, pLBPOutput, pLBPDebug);

    TUE_CML_MemoryCopy_M((void *)&sULPInput, (void *)&(pLBPDebug->sULPInput),
                         sizeof(sULPInput_t));
    TUE_CML_MemoryCopy_M((void *)&sULPParam, (void *)&(pLBPDebug->sULPParam),
                         sizeof(sULPParam_t));
    TUE_CML_MemoryCopy_M((void *)&sULPOutput, (void *)&(pLBPDebug->sULPOutput),
                         sizeof(sULPOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sULPDebug, (void *)&(pLBPDebug->sULPDebug),
                         sizeof(sULPDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sCLPInput, (void *)&(pLBPDebug->sCLPInput),
                         sizeof(sCLPInput_t));
    TUE_CML_MemoryCopy_M((void *)&sCLPParam, (void *)&(pLBPDebug->sCLPParam),
                         sizeof(sCLPParam_t));
    TUE_CML_MemoryCopy_M((void *)&sCLPOutput, (void *)&(pLBPDebug->sCLPOutput),
                         sizeof(sCLPOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sCLPDebug, (void *)&(pLBPDebug->sCLPDebug),
                         sizeof(sCLPDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sLFPInput, (void *)&(pLBPDebug->sLFPInput),
                         sizeof(sLFPInput_t));
    TUE_CML_MemoryCopy_M((void *)&sLFPParam, (void *)&(pLBPDebug->sLFPParam),
                         sizeof(sLFPParam_t));
    TUE_CML_MemoryCopy_M((void *)&sLFPOutput, (void *)&(pLBPDebug->sLFPOutput),
                         sizeof(sLFPOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sLFPDebug, (void *)&(pLBPDebug->sLFPDebug),
                         sizeof(sLFPDebug_t));

    TUE_CML_MemoryCopy_M((void *)&sELGInput, (void *)&(pLBPDebug->sELGInput),
                         sizeof(sELGInput_t));
    TUE_CML_MemoryCopy_M((void *)&sELGParam, (void *)&(pLBPDebug->sELGParam),
                         sizeof(sELGParam_t));
    TUE_CML_MemoryCopy_M((void *)&sELGOutput, (void *)&(pLBPDebug->sELGOutput),
                         sizeof(sELGOutput_t));
    TUE_CML_MemoryCopy_M((void *)&sELGDebug, (void *)&(pLBPDebug->sELGDebug),
                         sizeof(sELGDebug_t));
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */