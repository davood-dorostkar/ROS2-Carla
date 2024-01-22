/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDE
*****************************************************************************/
#include <string.h>

#include "stddef.h"
#include "TM_Global_Types.h"

#include "vlc_sen.h"
#include "cd_ext.h"
/*****************************************************************************
  MACROS
*****************************************************************************/
/*! @cond Doxygen_Suppress */
/* Freeze job uses application buffer  -> referenced data has to be kept global
 * or static */

#define MEAS_FREEZE_BUFFERED(VADDR_, ADDR_, SIZE_) /*lint -e{717} */        \
    do {                                                                    \
        MEASInfo_t desc;                                                    \
        desc.VirtualAddress = (VADDR_);                                     \
        desc.Length = (SIZE_);                                              \
        desc.FuncID = VLC_MEAS_FUNC_ID;                                     \
        desc.FuncChannelID = VLC_MEAS_FUNC_CHAN_ID;                         \
        (void)VLC_FREEZE_DATA(&desc, (void *)(ADDR_), &VLCSenMeasCallback); \
    } while (0) /* (no trailing ; ) */

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
/* Static meas freeze variables: To ensure that correct values are freezed in
 * the device */

SET_MEMSEC_VAR(VLCTrajectoriesMeas)
static VLCTrajectoriesMeas_t VLCTrajectoriesMeas;

/****************   Buffers for MeasFreezes *******************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
/****************   Buffers for MeasFreezes *******************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*! Debug information @vaddr:0x20900100 @cycleid:VLC_ENV */
static Com_AlgoParameters_t VLCSEN_BswAlgoParameters;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
static void VLCMeasFreezeTrajectoryData(void);
static void VLCFreezeObjectData(void);
/*! freeze function for VLC-output data (delayed ego signals) */
static void VLCMeasFreezeVLCOutput(void);
static void VLCSenProcessMeasFreezeCD(void);

static void VLCMeasFreezeVLCCDOutputCustom(void);
static void VLCSenFreezeInput(void);

/*! @endcond Doxygen_Suppress */
/*****************************************************************************
  FUNCTION
*****************************************************************************/

/*************************************************************************************************************************
  Functionname:    VLCMeasFreezeTrajectoryData */
static void VLCMeasFreezeTrajectoryData(void) {
    /* Fill in SI trajectory information */
    SIProcessTrajectoriesMeas(&VLCTrajectoriesMeas.SI);
    CDProcessEMPTrajectoryMeasFreeze(&VLCTrajectoriesMeas.EMPTrajPredEgo);
    // MEAS_FREEZE_BUFFERED(VLC_MEAS_ID_TRAJECTORIES, &VLCTrajectoriesMeas,
    //                     sizeof(VLCTrajectoriesMeas));
}

/*************************************************************************************************************************
  Functionname:    VLCFreezeObjectData */
static void VLCFreezeObjectData(void) {
    /* VLC Public Object Data freeze */
    // MEAS_FREEZE_BUFFERED(VLC_MEAS_ID_PUBLIC_OBJECT_LIST,
    //                     GET_VLC_PUB_OBJ_DATA_PTR, sizeof(AssessedObjList_t));

    /* move from em to vlc and clean up */
}

/*************************************************************************************************************************
  Functionname:    VLCSenFreezeInput */
static void VLCSenFreezeInput(void) {}

/*************************************************************************************************************************
  Functionname:    VLCSenFrameFreeze */
void VLCSenFrameFreeze(void) {
// static const MEASInfo_t VLC_SenFrameMeasInfo = {
//     VLC_MEAS_ID_SEN_FRAME_DATA, /* VirtualAddress */
//     sizeof(VLCSenFrame_t),      /* Length */
//      VLC_MEAS_FUNC_ID,           /* FuncID */
//      VLC_MEAS_FUNC_CHAN_ID       /* FuncChannelID */
//  };
// static const MEASInfo_t VLC_SenSyncRefMeasInfo = {
//      VLC_MEAS_ID_SEN_INPUT_SIGHEADERS, /* VirtualAddress */
//      sizeof(VLCSen_SyncRef_t),         /* Length */
//       VLC_MEAS_FUNC_ID,                 /* FuncID */
//       VLC_MEAS_FUNC_CHAN_ID             /* FuncChannelID */
//   };
#ifndef VLCSEN_BSW_ALGOPARAMETERS_VADDR
#define VLCSEN_BSW_ALGOPARAMETERS_VADDR 0x20900100
#endif
    //    static const MEASInfo_t VLCSEN_BSW_ALGO_PARAM_MeasInfo = {
    //       VLCSEN_BSW_ALGOPARAMETERS_VADDR, /* VirtualAddress */
    //       sizeof(Com_AlgoParameters_t),    /* Length */
    //       VLC_MEAS_FUNC_ID,                /* FuncID */
    //      VLC_MEAS_FUNC_CHAN_ID            /* FuncChannelID */
    //  };

    /*VLC_SEN_CFG_FREEZE_USE_CALLBACK*/
    // (void)VLC_FREEZE_DATA(&VLC_SenFrameMeasInfo, &VLCSenFrame, NULL);
    // (void)VLC_FREEZE_DATA(&VLC_SenSyncRefMeasInfo, &VLCSenSyncRef, NULL);

    /* debug freeze of Radar BSW ALGO Params*/
    if (VLCSEN_pBswAlgoParameters) {
        VLCSEN_BswAlgoParameters = *VLCSEN_pBswAlgoParameters;
    } else {
        (void)memset(&VLCSEN_BswAlgoParameters, 0,
                     sizeof(Com_AlgoParameters_t));
    }

    // (void)VLC_FREEZE_DATA(&VLCSEN_BSW_ALGO_PARAM_MeasInfo,
    //                      &VLCSEN_BswAlgoParameters, NULL);
}

/*************************************************************************************************************************
  Functionname:    VLCSenProcessMeasFreezeCD */
static void VLCSenProcessMeasFreezeCD(void) {
    //    static const MEASInfo_t VLCSen_HypothesesOutInfo = {
    //     VLC_MEAS_ID_SEN_HYPO_OUT,     /* VirtualAddress */
    //     sizeof(VLC_HypothesisIntf_t), /* Length */
    //     VLC_MEAS_FUNC_ID,             /* FuncID */
    //     VLC_MEAS_FUNC_CHAN_ID         /* FuncChannelID */
    // };

    /*------------------------------- Output
     * ----------------------------------*/
    /* Freeze Hypotheses Interface output */
    /*VLC_SEN_CFG_FREEZE_USE_CALLBACK*/
    /* Limited size of freeze buffer, fixed address of RTE buffer and timing
     * allows callback mechanism without buffering */
    // (void)VLC_FREEZE_DATA(&VLCSen_HypothesesOutInfo,
    //                      (void *)VLC_pCDHypothesesSen, &VLCSenMeasCallback);
    /*VLC_SEN_CFG_FREEZE_USE_CALLBACK*/
}

/*************************************************************************************************************************
  Functionname:    VLCSenProcessMeasFreeze */
void VLCSenProcessMeasFreeze(const proVLCSenPrtList_t *const pProvidePorts) {
    SET_MEMSEC_CONST(VLC_SenErrorsMeasInfo)
    // static const MEASInfo_t VLC_SenErrorsMeasInfo = {
    //    VLC_MEAS_ID_SEN_ERROR_OUT_VADDR, /* VirtualAddress */
    //   sizeof(DFSErrorOut_t),           /* Length */
    //    VLC_MEAS_FUNC_ID,                /* FuncID */
    //    VLC_MEAS_FUNC_CHAN_ID            /* FuncChannelID */
    // };
    // static const MEASInfo_t VLC_MeasInfoAccDisplayObject = {
    //    VLC_MEAS_ID_AVLC_DISPLAY_OBJ, /* .VirtualAddress */
    //    sizeof(VLC_acc_object_t),     /* .Length */
    //    VLC_MEAS_FUNC_ID,             /* .FuncID */
    //    VLC_MEAS_FUNC_CHAN_ID         /* .FuncChannelID */
    //};
    // static const MEASInfo_t VLC_MeasInfoAccOutData = {
    //    VLC_MEAS_ID_AVLC_OUTPUT_DATA,  /* .VirtualAddress */
    //    sizeof(VLC_acc_output_data_t), /* .Length */
    //    VLC_MEAS_FUNC_ID,              /* .FuncID */
    //   VLC_MEAS_FUNC_CHAN_ID          /* .FuncChannelID */
    //};
    // static const MEASInfo_t VLC_MeasInfoAccOOIData = {
    //    VLC_MEAS_ID_VLC_SEN_AVLC_OOI, /* .VirtualAddress */
    //   sizeof(VLCSenAccOOI_t),       /* .Length */
    //    VLC_MEAS_FUNC_ID,             /* .FuncID */
    //   VLC_MEAS_FUNC_CHAN_ID         /* .FuncChannelID */
    //};

    /*******************/
    /* INPUT-Port-Data */
    /*******************/
    VLCSenFreezeInput();

    /**********************/
    /* OUTPUT PORT-Data   */
    /**********************/
    /*! Freeze delayed input data of VLCOutput data */
    VLCMeasFreezeVLCOutput();

    /*! Freeze CD custom output data*/
    VLCMeasFreezeVLCCDOutputCustom();

    //(void)VLC_FREEZE_DATA(&VLC_MeasInfoAccDisplayObject,
    //                      (void *)pProvidePorts->pAccDisplayObj,
    //                      &VLCSenMeasCallback);
    //(void)VLC_FREEZE_DATA(&VLC_MeasInfoAccOutData,
    //                      (void *)pProvidePorts->pAccOutput,
    //                     &VLCSenMeasCallback);
    //(void)VLC_FREEZE_DATA(&VLC_SenErrorsMeasInfo,
    //                      (void *)pProvidePorts->pErrorOut, NULL);

    VLCSenProcessMeasFreezeCD();

    /*VLC_SEN_CFG_FREEZE_USE_CALLBACK*/
    //(void)VLC_FREEZE_DATA(&VLC_MeasInfoAccOOIData,
    //                      (void *)pProvidePorts->pVLCAccOOIData, NULL);
    /*VLC_SEN_CFG_FREEZE_USE_CALLBACK*/
    /*VLC_CFG_SEN2VEH_AVLC_OOI_PORT*/

    /*****************/
    /* Mixed Data    */
    /*****************/
    /* Freeze assessed object list data */
    VLCFreezeObjectData();

    /*****************/
    /* Internal Data */
    /*****************/
    VLCMeasFreezeTrajectoryData();
    /* SI meas freeze */
    SIFreezeData();
}

/*************************************************************************************************************************
  Functionname:    VLCSenMeasCallback */
void VLCSenMeasCallback(void) {
    /*! Callback function is a dummy */
    return;
}

/*************************************************************************************************************************
  Functionname:    VLCMeasFreezeVLCOutput */
static void VLCMeasFreezeVLCOutput(void) {
    // MEAS_FREEZE_BUFFERED(VLC_MEAS_ID_CUSTOM_OUTPUT, VLCSEN_pCustomOutput,
    //                     sizeof(VLCCustomOutput_t));
}

/*************************************************************************************************************************
  Functionname:    VLCMeasFreezeVLCCDOutputCustom */
static void VLCMeasFreezeVLCCDOutputCustom(void) {
    // MEAS_FREEZE_BUFFERED(VLC_MEAS_ID_CD_CUSTOM_OUTPUT,
    // VLCSEN_pCDCustomOutput,
    //                     sizeof(*VLCSEN_pCDCustomOutput));
}
/* ************************************************************************* */
/*   Copyright Tuerme                                              */
/* ************************************************************************* */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */