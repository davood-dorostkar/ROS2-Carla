// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * wulin <wulin1@senseauto.com>
 */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "fidm_main.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE5_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
uint8 FIDMInit = 0;
static ADCU_FID_Bit_Array ADCU_FID_Bit = {0};
static PDCU_FID_Bit_Array PDCU_FID_Bit = {0};
#define ASW_QM_CORE5_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  FUNCTION
*****************************************************************************/
void FIDConvert2SWC(Envm_FIDMOutPro_t* proPorts) {
    // Multipoint combination flag
    boolean DOW_FCA_multi;
    boolean Pilot_FCA_multi;
    boolean LCA_BSD_multi;

    // ACC output
    if (ADCU_FID_Bit[0][0] == TRUE || PDCU_FID_Bit[0][0] == TRUE) {
        proPorts->FIDMOut_ACC.FIDOut_ACC_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_ACC.FIDOut_ACC_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[0][1] == TRUE || PDCU_FID_Bit[0][1] == TRUE) {
        proPorts->FIDMOut_ACC.FIDOut_ACC_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_ACC.FIDOut_ACC_cdt.bRCA = FALSE;
    }
    proPorts->FIDMOut_ACC.FIDOut_ACC_cdt.bFCB = FALSE;
    proPorts->FIDMOut_ACC.FIDOut_ACC_cdt.bRCC = FALSE;

    // AEB output
    if (ADCU_FID_Bit[1][0] == TRUE || PDCU_FID_Bit[1][0] == TRUE) {
        proPorts->FIDMOut_AEB.FIDOut_AEB_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_AEB.FIDOut_AEB_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[1][1] == TRUE) {
        proPorts->FIDMOut_AEB.FIDOut_AEB_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_AEB.FIDOut_AEB_cdt.bRCA = FALSE;
    }
    proPorts->FIDMOut_AEB.FIDOut_AEB_cdt.bFCB = FALSE;
    proPorts->FIDMOut_AEB.FIDOut_AEB_cdt.bRCC = FALSE;

    // FCW output
    if (ADCU_FID_Bit[2][0] == TRUE || PDCU_FID_Bit[2][0] == TRUE) {
        proPorts->FIDMOut_FCW.FIDOut_FCW_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_FCW.FIDOut_FCW_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[2][1] == TRUE) {
        proPorts->FIDMOut_FCW.FIDOut_FCW_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_FCW.FIDOut_FCW_cdt.bRCA = FALSE;
    }
    proPorts->FIDMOut_FCW.FIDOut_FCW_cdt.bFCB = FALSE;
    proPorts->FIDMOut_FCW.FIDOut_FCW_cdt.bRCC = FALSE;

    // LDW outout
    if (ADCU_FID_Bit[3][0] == TRUE || PDCU_FID_Bit[3][0] == TRUE) {
        proPorts->FIDMOut_LDW.FIDOut_LDW_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_LDW.FIDOut_LDW_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[3][1] == TRUE || PDCU_FID_Bit[3][1] == TRUE) {
        proPorts->FIDMOut_LDW.FIDOut_LDW_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_LDW.FIDOut_LDW_cdt.bRCA = FALSE;
    }
    proPorts->FIDMOut_LDW.FIDOut_LDW_cdt.bFCB = FALSE;
    proPorts->FIDMOut_LDW.FIDOut_LDW_cdt.bRCC = FALSE;

    // LDP output
    if (ADCU_FID_Bit[4][0] == TRUE || PDCU_FID_Bit[4][0] == TRUE) {
        proPorts->FIDMOut_LDP.FIDOut_LDP_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_LDP.FIDOut_LDP_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[4][1] == TRUE || PDCU_FID_Bit[4][1] == TRUE) {
        proPorts->FIDMOut_LDP.FIDOut_LDP_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_LDP.FIDOut_LDP_cdt.bRCA = FALSE;
    }
    proPorts->FIDMOut_LDP.FIDOut_LDP_cdt.bFCB = FALSE;
    proPorts->FIDMOut_LDP.FIDOut_LDP_cdt.bRCC = FALSE;

    // TSR output
    if (ADCU_FID_Bit[5][0] == TRUE || PDCU_FID_Bit[5][0] == TRUE) {
        proPorts->FIDMOut_TSR.FIDOut_TSR_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_TSR.FIDOut_TSR_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[5][1] == TRUE || PDCU_FID_Bit[5][1] == TRUE) {
        proPorts->FIDMOut_TSR.FIDOut_TSR_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_TSR.FIDOut_TSR_cdt.bRCA = FALSE;
    }
    proPorts->FIDMOut_TSR.FIDOut_TSR_cdt.bFCB = FALSE;
    proPorts->FIDMOut_TSR.FIDOut_TSR_cdt.bRCC = FALSE;

    // polit output
    if ((ADCU_FID_Bit[6][2] == TRUE && ADCU_FID_Bit[6][6] == TRUE) ||
        (ADCU_FID_Bit[6][2] == TRUE && ADCU_FID_Bit[7][2] == TRUE) ||
        (ADCU_FID_Bit[6][3] == TRUE && ADCU_FID_Bit[6][7] == TRUE) ||
        (ADCU_FID_Bit[6][3] == TRUE && ADCU_FID_Bit[7][2] == TRUE) ||
        (ADCU_FID_Bit[6][4] == TRUE && ADCU_FID_Bit[7][0] == TRUE) ||
        (ADCU_FID_Bit[6][4] == TRUE && ADCU_FID_Bit[7][3] == TRUE) ||
        (ADCU_FID_Bit[6][5] == TRUE && ADCU_FID_Bit[7][1] == TRUE) ||
        (ADCU_FID_Bit[6][5] == TRUE && ADCU_FID_Bit[7][3] == TRUE)) {
        Pilot_FCA_multi = TRUE;
    } else {
        Pilot_FCA_multi = FALSE;
    }

    if (ADCU_FID_Bit[6][0] == TRUE || Pilot_FCA_multi == TRUE ||
        PDCU_FID_Bit[6][0] == TRUE) {
        proPorts->FIDMOut_Pilot.FIDOut_Pilot_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_Pilot.FIDOut_Pilot_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[6][1] == TRUE || PDCU_FID_Bit[6][1] == TRUE) {
        proPorts->FIDMOut_Pilot.FIDOut_Pilot_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_Pilot.FIDOut_Pilot_cdt.bRCA = FALSE;
    }
    proPorts->FIDMOut_Pilot.FIDOut_Pilot_cdt.bFCB = FALSE;
    proPorts->FIDMOut_Pilot.FIDOut_Pilot_cdt.bRCC = FALSE;

    // NOP output
    if (ADCU_FID_Bit[8][0] == TRUE || PDCU_FID_Bit[8][0] == TRUE) {
        proPorts->FIDMOut_NOP.FIDOut_NOP_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_NOP.FIDOut_NOP_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[8][1] == TRUE || PDCU_FID_Bit[8][1] == TRUE) {
        proPorts->FIDMOut_NOP.FIDOut_NOP_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_NOP.FIDOut_NOP_cdt.bRCA = FALSE;
    }
    if (ADCU_FID_Bit[8][2] == TRUE || PDCU_FID_Bit[8][2] == TRUE) {
        proPorts->FIDMOut_NOP.FIDOut_NOP_cdt.bFCB = TRUE;
    } else {
        proPorts->FIDMOut_NOP.FIDOut_NOP_cdt.bFCB = FALSE;
    }
    proPorts->FIDMOut_NOP.FIDOut_NOP_cdt.bRCC = FALSE;

    // DOW ouput
    if ((ADCU_FID_Bit[9][3] == TRUE && ADCU_FID_Bit[9][5] == TRUE) ||
        (ADCU_FID_Bit[9][4] == TRUE && ADCU_FID_Bit[9][6] == TRUE) ||
        (ADCU_FID_Bit[9][3] == TRUE && ADCU_FID_Bit[9][7] == TRUE) ||
        (ADCU_FID_Bit[9][4] == TRUE && ADCU_FID_Bit[9][7] == TRUE)) {
        DOW_FCA_multi = TRUE;
    } else {
        DOW_FCA_multi = FALSE;
    }

    if (ADCU_FID_Bit[9][0] == TRUE || PDCU_FID_Bit[9][0] == TRUE ||
        DOW_FCA_multi == TRUE) {
        proPorts->FIDMOut_DOW.FIDOut_DOW_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_DOW.FIDOut_DOW_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[9][1] == TRUE || PDCU_FID_Bit[9][1] == TRUE) {
        proPorts->FIDMOut_DOW.FIDOut_DOW_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_DOW.FIDOut_DOW_cdt.bRCA = FALSE;
    }
    if (ADCU_FID_Bit[9][2] == TRUE || PDCU_FID_Bit[9][2] == TRUE) {
        proPorts->FIDMOut_DOW.FIDOut_DOW_cdt.bFCB = TRUE;
    } else {
        proPorts->FIDMOut_DOW.FIDOut_DOW_cdt.bFCB = FALSE;
    }
    proPorts->FIDMOut_DOW.FIDOut_DOW_cdt.bRCC = FALSE;

    // LCA BSD output
    if ((ADCU_FID_Bit[10][3] == TRUE && ADCU_FID_Bit[10][5] == TRUE) ||
        (ADCU_FID_Bit[10][4] == TRUE && ADCU_FID_Bit[10][6] == TRUE) ||
        (ADCU_FID_Bit[10][3] == TRUE && ADCU_FID_Bit[10][7] == TRUE) ||
        (ADCU_FID_Bit[10][4] == TRUE && ADCU_FID_Bit[10][7] == TRUE)) {
        LCA_BSD_multi = TRUE;
    } else {
        LCA_BSD_multi = FALSE;
    }

    if (ADCU_FID_Bit[10][0] == TRUE || PDCU_FID_Bit[10][0] == TRUE ||
        LCA_BSD_multi == TRUE) {
        proPorts->FIDMOut_LCA.FIDOut_LCA_cdt.bFCA = TRUE;
        proPorts->FIDMOut_BSD.FIDOut_BSD_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_LCA.FIDOut_LCA_cdt.bFCA = FALSE;
        proPorts->FIDMOut_BSD.FIDOut_BSD_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[10][1] == TRUE || PDCU_FID_Bit[10][1] == TRUE) {
        proPorts->FIDMOut_LCA.FIDOut_LCA_cdt.bRCA = TRUE;
        proPorts->FIDMOut_BSD.FIDOut_BSD_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_LCA.FIDOut_LCA_cdt.bRCA = FALSE;
        proPorts->FIDMOut_BSD.FIDOut_BSD_cdt.bRCA = FALSE;
    }
    if (ADCU_FID_Bit[10][2] == TRUE || PDCU_FID_Bit[10][2] == TRUE) {
        proPorts->FIDMOut_LCA.FIDOut_LCA_cdt.bFCB = TRUE;
        proPorts->FIDMOut_BSD.FIDOut_BSD_cdt.bFCB = TRUE;
    } else {
        proPorts->FIDMOut_LCA.FIDOut_LCA_cdt.bFCB = FALSE;
        proPorts->FIDMOut_BSD.FIDOut_BSD_cdt.bFCB = FALSE;
    }
    proPorts->FIDMOut_LCA.FIDOut_LCA_cdt.bRCC = FALSE;
    proPorts->FIDMOut_BSD.FIDOut_BSD_cdt.bRCC = FALSE;

    // RCTA output
    if (ADCU_FID_Bit[11][0] == TRUE || PDCU_FID_Bit[11][0] == TRUE) {
        proPorts->FIDMOut_RCTA.FIDOut_RCTA_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_RCTA.FIDOut_RCTA_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[11][1] == TRUE || PDCU_FID_Bit[11][1] == TRUE) {
        proPorts->FIDMOut_RCTA.FIDOut_RCTA_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_RCTA.FIDOut_RCTA_cdt.bRCA = FALSE;
    }
    if (ADCU_FID_Bit[11][2] == TRUE || PDCU_FID_Bit[11][2] == TRUE) {
        proPorts->FIDMOut_RCTA.FIDOut_RCTA_cdt.bFCB = TRUE;
    } else {
        proPorts->FIDMOut_RCTA.FIDOut_RCTA_cdt.bFCB = FALSE;
    }
    proPorts->FIDMOut_RCTA.FIDOut_RCTA_cdt.bRCC = FALSE;

    // FCTA output
    if (ADCU_FID_Bit[13][0] == TRUE || PDCU_FID_Bit[13][0] == TRUE) {
        proPorts->FIDMOut_FCTA.FIDOut_FCTA_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_FCTA.FIDOut_FCTA_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[13][1] == TRUE || PDCU_FID_Bit[13][1] == TRUE) {
        proPorts->FIDMOut_FCTA.FIDOut_FCTA_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_FCTA.FIDOut_FCTA_cdt.bRCA = FALSE;
    }
    if (ADCU_FID_Bit[13][2] == TRUE || PDCU_FID_Bit[13][2] == TRUE) {
        proPorts->FIDMOut_FCTA.FIDOut_FCTA_cdt.bFCB = TRUE;
    } else {
        proPorts->FIDMOut_FCTA.FIDOut_FCTA_cdt.bFCB = FALSE;
    }
    proPorts->FIDMOut_FCTA.FIDOut_FCTA_cdt.bRCC = FALSE;

    // RCW output
    if (ADCU_FID_Bit[12][0] == TRUE || PDCU_FID_Bit[12][0] == TRUE) {
        proPorts->FIDMOut_RCW.FIDOut_RCW_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_RCW.FIDOut_RCW_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[12][1] == TRUE || PDCU_FID_Bit[12][1] == TRUE) {
        proPorts->FIDMOut_RCW.FIDOut_RCW_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_RCW.FIDOut_RCW_cdt.bRCA = FALSE;
    }
    if (ADCU_FID_Bit[12][2] == TRUE || PDCU_FID_Bit[12][2] == TRUE) {
        proPorts->FIDMOut_RCW.FIDOut_RCW_cdt.bFCB = TRUE;
    } else {
        proPorts->FIDMOut_RCW.FIDOut_RCW_cdt.bFCB = FALSE;
    }
    proPorts->FIDMOut_RCW.FIDOut_RCW_cdt.bRCC = FALSE;

    // IHBC output
    if (ADCU_FID_Bit[14][0] == TRUE || PDCU_FID_Bit[14][0] == TRUE) {
        proPorts->FIDMOut_IHBC.FIDOut_IHBC_cdt.bFCA = TRUE;
    } else {
        proPorts->FIDMOut_IHBC.FIDOut_IHBC_cdt.bFCA = FALSE;
    }
    if (ADCU_FID_Bit[14][1] == TRUE || PDCU_FID_Bit[14][1] == TRUE) {
        proPorts->FIDMOut_IHBC.FIDOut_IHBC_cdt.bRCA = TRUE;
    } else {
        proPorts->FIDMOut_IHBC.FIDOut_IHBC_cdt.bRCA = FALSE;
    }
    if (ADCU_FID_Bit[14][2] == TRUE) {
        proPorts->FIDMOut_IHBC.FIDOut_IHBC_cdt.bFCB = TRUE;
    } else {
        proPorts->FIDMOut_IHBC.FIDOut_IHBC_cdt.bFCB = FALSE;
    }
    proPorts->FIDMOut_IHBC.FIDOut_IHBC_cdt.bRCC = FALSE;
}

void FIDM_detect(const Envm_FIDInport_t* reqPorts,
                 Envm_FIDMOutPro_t* proPorts) {
    for (int i = 0; i < 36; i++) {
        ADCU_FID_Bit[i][0] =
            (reqPorts->ADCU_FID[i] & Bitmask_F0) ? TRUE : FALSE;
        ADCU_FID_Bit[i][1] =
            (reqPorts->ADCU_FID[i] & Bitmask_F1) ? TRUE : FALSE;
        ADCU_FID_Bit[i][2] =
            (reqPorts->ADCU_FID[i] & Bitmask_F2) ? TRUE : FALSE;
        ADCU_FID_Bit[i][3] =
            (reqPorts->ADCU_FID[i] & Bitmask_F3) ? TRUE : FALSE;
        ADCU_FID_Bit[i][4] =
            (reqPorts->ADCU_FID[i] & Bitmask_F4) ? TRUE : FALSE;
        ADCU_FID_Bit[i][5] =
            (reqPorts->ADCU_FID[i] & Bitmask_F5) ? TRUE : FALSE;
        ADCU_FID_Bit[i][6] =
            (reqPorts->ADCU_FID[i] & Bitmask_F6) ? TRUE : FALSE;
        ADCU_FID_Bit[i][7] =
            (reqPorts->ADCU_FID[i] & Bitmask_F7) ? TRUE : FALSE;
    }

    for (int i = 0; i < 26; i++) {
        PDCU_FID_Bit[i][0] =
            (reqPorts->PDCU_FID[i] & Bitmask_F0) ? TRUE : FALSE;
        PDCU_FID_Bit[i][1] =
            (reqPorts->PDCU_FID[i] & Bitmask_F1) ? TRUE : FALSE;
        PDCU_FID_Bit[i][2] =
            (reqPorts->PDCU_FID[i] & Bitmask_F2) ? TRUE : FALSE;
        PDCU_FID_Bit[i][3] =
            (reqPorts->PDCU_FID[i] & Bitmask_F3) ? TRUE : FALSE;
        PDCU_FID_Bit[i][4] =
            (reqPorts->PDCU_FID[i] & Bitmask_F4) ? TRUE : FALSE;
        PDCU_FID_Bit[i][5] =
            (reqPorts->PDCU_FID[i] & Bitmask_F5) ? TRUE : FALSE;
        PDCU_FID_Bit[i][6] =
            (reqPorts->PDCU_FID[i] & Bitmask_F6) ? TRUE : FALSE;
        PDCU_FID_Bit[i][7] =
            (reqPorts->PDCU_FID[i] & Bitmask_F7) ? TRUE : FALSE;
    }

    FIDConvert2SWC(proPorts);
}

void FIDM_Init() {
    memset(&ADCU_FID_Bit, 0, sizeof(ADCU_FID_Bit_Array));
    memset(&PDCU_FID_Bit, 0, sizeof(PDCU_FID_Bit_Array));
}

void FIDMProcess(const Envm_FIDInport_t* reqPorts,
                 Envm_FIDMOutPro_t* proPorts) {
    if (FIDMInit != 1) {
        FIDM_Init();
        FIDMInit = 1;
    }

    FIDM_detect(reqPorts, proPorts);
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */