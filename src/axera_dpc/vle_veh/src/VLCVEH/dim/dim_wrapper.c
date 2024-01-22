/*
 * Copyright (C) 2022-2024 by SoftwareMotion Group Limited. All rights reserved.
 * guotao 
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include <stddef.h>
#include "dim_cfg.h"
#include "dim.h"
#include "TM_Global_Types.h"
#include "dim_eba.h"

/*! @brief       DIM_OUT_CUSTOM_GAS_PEDAL_THRES
    @general
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     NormalValue   @unit -     @min -   @max -   */
#define DIM_OUT_CUSTOM_GAS_PEDAL_THRES (20.0f)

/*! @brief       DIM_OUT_CUSTOM_DEFAULT_DRVBRAKING
    @general
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     False  @unit -     @min -   @max -   */
#define DIM_OUT_CUSTOM_DEFAULT_DRVBRAKING (FALSE)

/*! @brief       DIM_OUT_CUSTOM_GAS_PEDAL_THRES
    @general
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     95.0f   @unit -     @min -   @max -   */
#define DIM_OUT_CUSTOM_DEFAULT_GASPEDALPOS (95.0f)

/*! @brief       DIM_OUT_CUSTOM_DEFAULT_SPEEDLIMITACTIVE
    @general
    @conseq      @incp  -
                 @decp  -
    @attention   -
    @typical     False   @unit -     @min -   @max -   */
#define DIM_OUT_CUSTOM_DEFAULT_SPEEDLIMITACTIVE (FALSE)

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief DimOutputState_t

    @general Output signal state

    @conseq [ None ]

    @attention [ None ]

    */
typedef enum {
    DIM_OUT_OK = 0,
    DIM_OUT_ERROR = 1,
    DIM_OUT_NOK_NO_ERROR = 2
} DimOutputState_t;

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief DimReportState_t

    @general Output error indication

    @conseq [ None ]

    @attention [ None ]

    */
typedef enum {
    DIM_REP_NO = 0,
    DIM_REP_SILENT = 1,
    DIM_REP_ALERT = 2
} DimReportState_t;

/*****************************************************************************
  PROTOTYPES
*****************************************************************************/
/* functions */
void DIMFillCustomOut(const DIMInputDataGlobal_t* const pInputData);
static void DIMEvalOutputState(const DIMInputDataGlobal_t* const pInputData,
                               DimReportState_t* pReport,
                               DimOutputState_t* pState);
static void DIMFillMonitoringState(
    const DIMInputDataGlobal_t* const pInputData);

static void DIMPostProcEBA(void);
static void DIMPostProcEBAAttention(const sint8 siProb,
                                    eDriverAttentionState_t* const eState);

static void DIMPostProcEBAFeedback(const sint8 siProb,
                                   eDriverFeedbackState_t* const eState);

static void DIMPostProcEBAActivity(const sint8 siProb,
                                   eDriverActivityState_t* const eState);

/*****************************************************************************
  Functionname             DIMFillCustomOut  */ /*!

                                 @brief          DIMFillCustomOut

                                 @description    custom filling of
                               VLC_pDIMCustDataOut

                                 @param[in]      pInputData:
                               [DIMInputDataGlobal_t as per dim.h]

                                 @return		   void

                                 @glob_in         None
                                 @glob_out        None

                                 @c_switch_part	VLC_DIM_CFG_HYPO_EBA
                                 @c_switch_part	VLC_DIM_CFG_HYPO_ALDW
                                 @c_switch_part	VLC_DIM_CFG_HYPO_SI
                                 @c_switch_full
                               VLC_CFG_DRIVER_INTENTION_MONITORING :
                               Configuration switch for enabling driver
                               intention monitoring processing

                                 @pre             None
                                 @post            None

                                 @created         -
                                 @changed         -

                               ******************************************************************************/
void DIMFillCustomOut(const DIMInputDataGlobal_t* const pInputData) {
    DIMFillMonitoringState(pInputData);

    DIMPostProcEBA();
}

/*************************************************************************************************************************
  Functionname:    DIMEvalOutputState */
static void DIMEvalOutputState(const DIMInputDataGlobal_t* const pInputData,
                               DimReportState_t* pReport,
                               DimOutputState_t* pState) {
    /* Condition for error state, causing degradation of successive functional
     * component */
    const boolean statErrOK =
        ((DIMGetInputQuality(pInputData->GasPedalPosition) !=
          DIMInputSignalState_OK) ||
         (DIMGetInputQuality(pInputData->GasPedalGradient) !=
          DIMInputSignalState_OK) ||
         (DIMGetInputQuality(pInputData->VehicleVelocity) !=
          DIMInputSignalState_OK) ||
         (DIMGetInputQuality(pInputData->VehicleAcceleration) !=
          DIMInputSignalState_OK) ||
         (DIMGetInputQuality(pInputData->DriverBraking) !=
          DIMInputSignalState_OK))
            ? TRUE
            : FALSE;

    const boolean statErrBad =
        ((DIMGetInputQuality(pInputData->SteeringWheelAngle) !=
          DIMInputSignalState_OK) ||
         (DIMGetInputQuality(pInputData->SteeringWheelGradient) !=
          DIMInputSignalState_OK))
            ? TRUE
            : FALSE;

    if ((statErrOK == FALSE) && (statErrBad == FALSE)) {
        /* Essential signals are given, report error in case of missing signals,
         * which are uncritical */
        *pState = DIM_OUT_OK;
        *pReport = DIM_REP_NO;

        /* Report error without functional degradation in case of invalid turn
         * indicator or speed limiter */
        if ((VLC_pDIMCustDataIn->eSpeedLimitActive == eVLC_STATE_SIG_INVALID) ||
            (DIMGetInputQuality(pInputData->TurnIndicator) !=
             DIMInputSignalState_OK)) {
            *pReport = DIM_REP_ALERT;
        }
    } else {
        /* set output state to error */
        *pState = DIM_OUT_ERROR;
        *pReport = DIM_REP_ALERT;

        /* Inhibit error indication in case of missing multi-turn detection */
        if ((statErrOK == FALSE) &&
            (DIMGetInputQuality(pInputData->SteeringWheelAngle) ==
             DIMInputSignalState_BadQuality) &&
            (DIMGetInputQuality(pInputData->SteeringWheelGradient) ==
             DIMInputSignalState_BadQuality)) {
            *pReport = DIM_REP_SILENT;
            *pState = DIM_OUT_NOK_NO_ERROR;
        }
    }
}

/* **********************************************************************
 Functionname             DIMFillMonitoringState  */ /*!

                       @brief          Custom filling of eDriverMonitoringState
                     and eDriverMonitoringErrorReport

                       @description    Custom filling of
                     VLC_pDIMCustDataOut->eDriverMonitoringState and
                     VLC_pDIMCustDataOut->eDriverMonitoringErrorReport

                       @param[in]      pInputData:				[DIMInputDataGlobal_t as
                     per dim.h]

                       @return		   void

                       @glob_in         None
                       @glob_out        None

                       @c_switch_part	eDimMonState_NotPossible
                       @c_switch_part	eDimMonReport_NoError
                       @c_switch_full    VLC_CFG_DRIVER_INTENTION_MONITORING :
                     Configuration switch for enabling driver intention
                     monitoring processing

                       @pre             None
                       @post            None

                       @created         -
                       @changed         -

                     ****************************************************************************
                     */
static void DIMFillMonitoringState(
    const DIMInputDataGlobal_t* const pInputData) {
    /* DIM fill output states*/
    DimOutputState_t OutState;
    DimReportState_t OutReport;

    /* Determine output state */
    DIMEvalOutputState(pInputData, &OutReport, &OutState);

    switch (OutState) {
        case DIM_OUT_OK:
            /* Unlimited functionality */
            VLC_pDIMCustDataOut->eDriverMonitoringState =
                eDimMonState_Unlimited;
            break;

        case DIM_OUT_NOK_NO_ERROR:
            /* Limited functionality, signal error to vehicle network */
            VLC_pDIMCustDataOut->eDriverMonitoringState = eDimMonState_Limited;
            break;

        case DIM_OUT_ERROR:
        default:
            /* No reasonable output possible, signal error and enforce
             * degradation (head) */
            VLC_pDIMCustDataOut->eDriverMonitoringState =
                eDimMonState_NotPossible;
            break;
    }
}

/* **********************************************************************
  Functionname              DIMPostProcEBA  */ /*!

                             @brief          Custom filling of attention,
                           feedback and activity state

                             @description    Custom filling of
                           VLC_pDIMCustDataOut->eDriverAttentionState,
                           ->eDriverFeedbackState and ->eDriverActivityState
                           after all DIM modules have been processed

                             @param[in]      void

                             @return		   void

                             @glob_in         None
                             @glob_out        None

                             @c_switch_full
                           VLC_CFG_DRIVER_INTENTION_MONITORING : Configuration
                           switch for enabling driver intention monitoring
                           processing
                             @c_switch_full	VLC_DIM_CFG_HYPO_EBA: Enable DIM
                           for EBA

                             @pre             None
                             @post            None

                             @created         -
                             @changed         -

                           ****************************************************************************
                           */
static void DIMPostProcEBA(void) {
    uint8 uiHypNr;
    GDB_DMHypothesis_t sLocalHyp;

    VLC_pDIMCustDataOut->eDriverAttentionState = eDriverAttentionState_Unknown;
    /*!TODO: Introduce default values for feedback and activity state*/

    for (uiHypNr = 0u; uiHypNr < DIM_NR_OF_EBA_HYPOTHESES; uiHypNr++) {
        sLocalHyp = DIMHypothesisList.rgDimHypEBA[uiHypNr];
        if (GDB_ERROR_NONE == sLocalHyp.eGDBError) {
            switch (uiHypNr) {
                case (uint8)DIM_EBA_HYP_IDX_ATTENTION:
                    /*driver attention*/
                    DIMPostProcEBAAttention(
                        sLocalHyp.Probability,
                        &(VLC_pDIMCustDataOut->eDriverAttentionState));
                    break;

                case (uint8)DIM_EBA_HYP_IDX_FEEDBACK:
                    /*driver feedback*/
                    DIMPostProcEBAFeedback(
                        sLocalHyp.Probability,
                        &(VLC_pDIMCustDataOut->eDriverFeedbackState));
                    break;

                case (uint8)DIM_EBA_HYP_IDX_ACTIVITY:
                    /*driver activity*/
                    DIMPostProcEBAActivity(
                        sLocalHyp.Probability,
                        &(VLC_pDIMCustDataOut->eDriverActivityState));
                    break;

                default:
                    /*this case is never reached*/
                    break;
            }
        }
    }

    VLC_pDIMCustDataOut->sSigHeader.eSigStatus =
        (AlgoSignalState_t)AL_SIG_STATE_OK;
}

static void DIMPostProcEBAAttention(const sint8 siProb,
                                    eDriverAttentionState_t* const eState) {
    if (siProb < DIM_ATTENTION_PAR_PROP_THRES_UNKN) {
        VLC_pDIMCustDataOut->eDriverAttentionState =
            eDriverAttentionState_Unknown;
    } else if (siProb < DIM_ATTENTION_PAR_PROP_THRES_LOW) {
        *eState = eDriverAttentionState_Low;
    } else if (siProb < DIM_ATTENTION_PAR_PROP_THRES_HIGH) {
        *eState = eDriverAttentionState_High;
    } else if (siProb < DIM_ATTENTION_PAR_PROP_THRES_HIGHER) {
        *eState = eDriverAttentionState_Higher;
    } else {
        *eState = eDriverAttentionState_VeryHigh;
    }
}

static void DIMPostProcEBAFeedback(const sint8 siProb,
                                   eDriverFeedbackState_t* const eState) {
    if (siProb < DIM_FEEDBACK_PAR_FdbckWeakNeg) {
        *eState = eDriverFeedbackState_Negative;
    } else if (siProb < DIM_FEEDBACK_PAR_FdbckNoNeg) {
        *eState = eDriverFeedbackState_WeakNegative;
    } else if (siProb < DIM_FEEDBACK_PAR_FdbckNoPos) {
        *eState = eDriverFeedbackState_NoNegative;
    } else if (siProb < DIM_FEEDBACK_PAR_FdbckWeakPos) {
        *eState = eDriverFeedbackState_NoPositive;
    } else if (siProb < DIM_FEEDBACK_PAR_FdbckPos) {
        *eState = eDriverFeedbackState_WeakPositive;
    } else if (siProb < DIM_FEEDBACK_PAR_FdbckStrongPos) {
        *eState = eDriverFeedbackState_Positive;
    } else {
        *eState = eDriverFeedbackState_StrongPositive;
    }
}

static void DIMPostProcEBAActivity(const sint8 siProb,
                                   eDriverActivityState_t* const eState) {
    if (siProb < DIM_ACTIVITY_PAR_MissingConfidenceDelta) {
        *eState = eDriverActivity_Inactive;
    } else if (siProb < DIM_ACTIVITY_PAR_VeryActivePercentage) {
        *eState = eDriverActivity_MissingConf;
    } else if (siProb < DIM_ACTIVITY_PAR_EmergenySteeringPercentage) {
        *eState = eDriverActivity_VeryActive;
    } else {
        *eState = eDriverActivity_EmergencySteer;
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
