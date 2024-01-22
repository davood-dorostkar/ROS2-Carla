/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * mengchen <mengchen@senseauto.com>
 */

#include "POLYFIT_Processing.h"

boolean LaneFitSuccess(sPFTOutput_t* sPFTOutput) {
    boolean Dev = fABS(sPFTOutput->fDevToTraj3rd / POLYFIT_SAMPLE_POINTS) > 0.5
                      ? FALSE
                      : TRUE;
    boolean C0 = fABS(sPFTOutput->fPosY03rd) > 10.0 ? FALSE : TRUE;
    boolean C1 = fABS(sPFTOutput->fHeading3rd) > 3.14 ? FALSE : TRUE;
    boolean C2 = fABS(sPFTOutput->fCrv3rd) > 1.0 ? FALSE : TRUE;
    boolean C3 = fABS(sPFTOutput->fChngOfCrv3rd) > 0.1 ? FALSE : TRUE;

    if (Dev && C0 && C1 && C2 && C3) {
        return TRUE;
    } else {
        return FALSE;
    }
}

boolean LaneFitProcess(const sPFTLane_t* sPFTLane,
                       sPFTInput_t* sPFTInput,
                       sPFTOutput_t* sPFTOutput) {
    UINT8_T pts_size = sPFTLane->PFTLanePointSize;

    if (pts_size >= POLYFIT_SAMPLE_POINTS_DOUBLE) {
        // Select odd points. eg,0 2 4 --- 38
        for (UINT8_T ii = 0; ii < POLYFIT_SAMPLE_POINTS; ii++) {
            sPFTInput->fFctWeight[ii] = 1.0f;
            sPFTInput->fPosXArray[ii] = sPFTLane->PFTLanePoint[ii * 2].fX_met;
            sPFTInput->fPosYArray[ii] = sPFTLane->PFTLanePoint[ii * 2].fY_met;
        }
    } else if (pts_size > POLYFIT_SAMPLE_POINTS) {
        // Select odd number. When less than 20, fill with even number.
        // Calculation quotient
        UINT8_T pts_size_half = pts_size / 2;
        for (UINT8_T ii = 0; ii < POLYFIT_SAMPLE_POINTS; ii++) {
            sPFTInput->fFctWeight[ii] = 1.0f;
            if (ii < pts_size_half) {
                sPFTInput->fPosXArray[ii] =
                    sPFTLane->PFTLanePoint[ii * 2].fX_met;
                sPFTInput->fPosYArray[ii] =
                    sPFTLane->PFTLanePoint[ii * 2].fY_met;
            } else {
                sPFTInput->fPosXArray[ii] =
                    sPFTLane->PFTLanePoint[(ii - pts_size_half) * 2 + 1].fX_met;
                sPFTInput->fPosYArray[ii] =
                    sPFTLane->PFTLanePoint[(ii - pts_size_half) * 2 + 1].fY_met;
            }
        }

    } else if (pts_size == POLYFIT_SAMPLE_POINTS) {
        for (UINT8_T ii = 0; ii < POLYFIT_SAMPLE_POINTS; ii++) {
            sPFTInput->fFctWeight[ii] = 1.0f;
            sPFTInput->fPosXArray[ii] = sPFTLane->PFTLanePoint[ii].fX_met;
            sPFTInput->fPosYArray[ii] = sPFTLane->PFTLanePoint[ii].fY_met;
        }

    } else if (pts_size >= POLYFIT_SAMPLE_POINTS_HALF) {
        // When less than 20, count backwards and fill.
        for (UINT8_T ii = 0; ii < POLYFIT_SAMPLE_POINTS; ii++) {
            if (ii < pts_size) {
                sPFTInput->fFctWeight[ii] = 1.0f;
                sPFTInput->fPosXArray[ii] = sPFTLane->PFTLanePoint[ii].fX_met;
                sPFTInput->fPosYArray[ii] = sPFTLane->PFTLanePoint[ii].fY_met;
            } else {
                sPFTInput->fFctWeight[ii] = 1.0f;
                sPFTInput->fPosXArray[ii] =
                    sPFTLane->PFTLanePoint[2 * pts_size - ii - 1].fX_met;
                sPFTInput->fPosYArray[ii] =
                    sPFTLane->PFTLanePoint[2 * pts_size - ii - 1].fY_met;
            }
        }
    } else {
        return FALSE;
    }

    // Third-order polynomial fitting
    sPFTInput->bEnable1st = 0U;
    sPFTInput->bEnable2nd = 0U;
    sPFTInput->bEnable3rd = 1U;
    sPFTInput->fFctCrvDecay = 1.0F;
    sPFTInput->fFctCrvChngDecay = 1.0F;

    TUE_CML_PolyFit_M(sPFTInput, sPFTOutput);

    if (LaneFitSuccess(sPFTOutput)) {
        return TRUE;
    } else {
        return FALSE;
    }
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */