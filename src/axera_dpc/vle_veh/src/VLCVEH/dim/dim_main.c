/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "dim_cfg.h"
#include "dim.h"
#include "stddef.h"
#include "TM_Global_Types.h"
#include "TM_Global_Const.h"
#include "dim_eba.h"
/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*! @brief       Number of calibrating hypotheses
    @general     Number of calibrating hypotheses in DIM Module
    @conseq      @incp  DIM will have one more Calibrating hypothesis to check
                 @decp  DIM will have one less Calibrating hypothesis to check
    @attention   [None]
    @typical     NA
    @unit        1 Count
    @min         1       */
#define DIM_NO_CALIB_HYPS 3uL

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/* ****************************************************************
    TYPEDEF STRUCT DIMData_t
 **************************************************************** */
/*! @brief DIM Data Structure

    @general DIM Data Structure with DIMMTSHeaderData_t, DIMInputData_t,
   DIMInteralDataEBA_t, DIMInteralDataALDW_t and DIMInteralDataSI_t members

    @conseq [None]

    @attention [None]

    */
typedef struct {
    DIMMTSHeaderData_t DIMHeaderData; /*!< @name:Header*/

    DIMInputData_t DimInputData;

    DIMInteralDataEBA_t Internal_Data_EBA;

} DIMData_t; /*!< @VADDR: VLC_MEAS_ID_CGEB_DIM_DATA @VNAME: DIM @cycleid:
                VLC_VEH*/

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static DIMData_t
    DIMData; /*!< Declaration of module global data of type DIMData_t*/
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
DIMState_t DIMState = DIM_STATE_INIT; /*!< Definition of module global DIMState
                                         of type DIMState_t */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

/*aliasing to allow efficient data access */
#define ASW_QM_CORE1_MODULE_START_SEC_CONST_UNSPECIFIED
#include "ASW_MemMap.h"
static DIMInputDataGlobal_t *const pDimInputGlobal =
    &(DIMData.DimInputData
          .Global); /*!< Definition of pointer to pDimInputDataGlobal of type
                       DIMInputDataGlobal_t */
#define ASW_QM_CORE1_MODULE_STOP_SEC_CONST_UNSPECIFIED
#include "ASW_MemMap.h"

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static boolean bDIMInitialized =
    FALSE; /*!< Definition of bDIMInitialized of type boolean */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"

/* Dynamic Data - Adding to DIMData will exceed address range */
/* ****************************************************************
    TYPEDEF STRUCT DIM_HYPOUT_CAL_DATA_t
    **************************************************************** */
/*! @brief      Stuck specific hypothesis output value

    @general    This parameter provides the opportunity to calibrate the value
   of specific driver hypothesis. Parameter is indented to use for vehicle test
   in order to avoid driver intention to delay or suppress desired interventions

    @conseq     As soon as calibration mask is set different from zero, the
   output value for hypothesis corresponding to bit position is defined
   parameter value instead of the calculated one.

    @attention  [None]

    */
typedef struct {
    uint8 CalibMask; /*!< bit0: attention, bit1: feedback, bit2: activity */
    sint8 Probability[DIM_NO_CALIB_HYPS];       /*!< -100 -> +100%*/
    percentage_t Confidence[DIM_NO_CALIB_HYPS]; /*!< hypothesis confidence*/
} DIM_HYPOUT_CAL_DATA_t;

#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
// SET_MEMSEC_VAR(DIM_HYPOUT_CAL_DATA_EBA)
static DIM_HYPOUT_CAL_DATA_t
    DIM_HYPOUT_CAL_DATA_EBA; /*!< Declaration of DIM_HYPOUT_CAL_DATA_EBA of type
                                DIM_HYPOUT_CAL_DATA_t*/

/* Hypothesis Output List */
// SET_MEMSEC_VAR(DIMHypothesisList)
DIMHypothesisList_t DIMHypothesisList; /*!< Declaration of DIMHypothesisList of
                                          type DIMHypothesisList_t */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  PROTOTYPES
*****************************************************************************/
/* functions */

static void DIMCollectInputs(void);
static void DIMCopyHypothesis(const GDB_DMHypothesis_t *hypIn,
                              GDB_DMHypothesis_t hypOut[],
                              uint8 uiHypNr,
                              uint8 uiTotalHypNr,
                              const DIM_HYPOUT_CAL_DATA_t *DIM_HYPOUT_CAL_DATA);
static void DIMInitHypothesis(GDB_DMHypothesis_t *const hyp);

static void DIMRunModulesEBA(const float32 fCycleTime);
static void DIMInitEba(void);

/*************************************************************************************************************************
  Functionname:    DIMRunModulesEBA */
static void DIMRunModulesEBA(const float32 fCycleTime) {
    uint8 uiHypNr;
    GDB_DMHypothesis_t sLocalHyp;

    for (uiHypNr = 0u; uiHypNr < DIM_NR_OF_EBA_HYPOTHESES; uiHypNr++) {
        DIMInitHypothesis(&(DIMHypothesisList.rgDimHypEBA[uiHypNr]));
        DIMInitHypothesis(&sLocalHyp);

        switch (uiHypNr) {
            case (uint8)DIM_EBA_HYP_IDX_ATTENTION:
                /*driver attention*/
                /* Use SpeedLimiter in Attention to apply different logic for
                 * calculation. Do NOT manipulate Input values */
                sLocalHyp.eGDBError = DIMRunModuleAttention(
                    fCycleTime, pDimInputGlobal, &sLocalHyp,
                    &DIMData.Internal_Data_EBA.Internal_Attention,
                    &DIM_ATTENTION_PAR_data_eba);
                if (GDB_ERROR_NONE == sLocalHyp.eGDBError) {
                    DIMCopyHypothesis(&sLocalHyp, DIMHypothesisList.rgDimHypEBA,
                                      uiHypNr, DIM_NR_OF_EBA_HYPOTHESES,
                                      &DIM_HYPOUT_CAL_DATA_EBA);
                }
                break;

            case (uint8)DIM_EBA_HYP_IDX_FEEDBACK:
                /*driver feedback*/
                /* Restore original driver pedal value */
                DIMSetInputValueFloat(
                    &(pDimInputGlobal->GasPedalGradient),
                    VLC_pDIMGenericDataIn->fAccelPedalGrad,
                    VLC_pDIMGenericDataIn->eAccelPadelGradStat, 0);
                DIMSetInputValueFloat(&(pDimInputGlobal->GasPedalPosition),
                                      VLC_pDIMGenericDataIn->fAccelPedalPos,
                                      VLC_pDIMGenericDataIn->eAccelPadelStat,
                                      0);
                sLocalHyp.eGDBError = DIMRunModuleFeedback(
                    fCycleTime, pDimInputGlobal, &sLocalHyp,
                    &DIMData.Internal_Data_EBA.Internal_Feedback,
                    &DIM_FEEDBACK_PAR_data_eba);
                if (GDB_ERROR_NONE == sLocalHyp.eGDBError) {
                    DIMCopyHypothesis(&sLocalHyp, DIMHypothesisList.rgDimHypEBA,
                                      uiHypNr, DIM_NR_OF_EBA_HYPOTHESES,
                                      &DIM_HYPOUT_CAL_DATA_EBA);
                }
                break;

            case (uint8)DIM_EBA_HYP_IDX_ACTIVITY:
                /*driver activity*/
                sLocalHyp.eGDBError = DIMRunModuleActivity(
                    fCycleTime, pDimInputGlobal, &sLocalHyp,
                    &DIMData.Internal_Data_EBA.Internal_Activity,
                    &DIM_ACTIVITY_PAR_data_eba);
                if (GDB_ERROR_NONE == sLocalHyp.eGDBError) {
                    DIMCopyHypothesis(&sLocalHyp, DIMHypothesisList.rgDimHypEBA,
                                      uiHypNr, DIM_NR_OF_EBA_HYPOTHESES,
                                      &DIM_HYPOUT_CAL_DATA_EBA);
                }
                break;

            default:
                /*this case is never reached*/
                break;
        }
    }
}

/*************************************************************************************************************************
  Functionname:    DIMInitEba */
static void DIMInitEba(void) {
    uint8 uiI;

    for (uiI = 0u; uiI < DIM_NR_OF_EBA_HYPOTHESES; uiI++) {
        DIMInitHypothesis(&(DIMHypothesisList.rgDimHypEBA[uiI]));
    }

    (void)DIMInitModuleAttention(&DIMData.Internal_Data_EBA.Internal_Attention);
    (void)DIMInitModuleFeedback(&DIMData.Internal_Data_EBA.Internal_Feedback);
    (void)DIMInitModuleActivity(&DIMData.Internal_Data_EBA.Internal_Activity);
}

/*************************************************************************************************************************
  Functionname:    DIMProcess */
void DIMProcess(const float32 fCycleTime) {
    if (bDIMInitialized == FALSE) {
        DIMState = DIM_STATE_INIT;
    }

    switch (DIMState) {
        case DIM_STATE_INIT:
            DIMInit(fCycleTime);
            break;
        case DIM_STATE_OK:
            DIMCollectInputs();

            DIMRunSigPreProc(fCycleTime, pDimInputGlobal);

            DIMRunModulesEBA(fCycleTime);

            DIMFillCustomOut(pDimInputGlobal);

            break;
        default:
            DIMInit(fCycleTime);
            break;
    }
}

/*************************************************************************************************************************
  Functionname:    DIMInit */
/*DIMInit is also called by simulation functions*/
void DIMInit(const float32 fCycleTime) {
    /*clear input data*/
    DIMSetInputValueBool(&(pDimInputGlobal->DriverBraking), FALSE,
                         DIMInputSignalState_Missing);
    DIMSetInputValueFloat(&(pDimInputGlobal->GasPedalGradient), 0,
                          DIMInputSignalState_Missing, 0);
    DIMSetInputValueFloat(&(pDimInputGlobal->GasPedalPosition), 100.0f,
                          DIMInputSignalState_Missing, 0);
    DIMSetInputValueUInt(&(pDimInputGlobal->TurnIndicator), 0,
                         DIMInputSignalState_Missing);
    DIMSetInputValueFloat(&(pDimInputGlobal->VehicleAcceleration), 0,
                          DIMInputSignalState_Missing, 0);
    DIMSetInputValueFloat(&(pDimInputGlobal->VehicleVelocity), 0,
                          DIMInputSignalState_Missing, 0);
    DIMSetInputValueFloat(&(pDimInputGlobal->SteeringWheelGradient), 0,
                          DIMInputSignalState_Missing, 0);
    DIMSetInputValueFloat(&(pDimInputGlobal->SteeringWheelAngle), 0,
                          DIMInputSignalState_Missing, 0);

    bDIMInitialized = TRUE;

    DIMData.DIMHeaderData.uiBaseVersion = (uint32)DIM_AUTOVERSION;
    DIMData.DIMHeaderData.uiStructSize = sizeof(DIMData_t);

    DIMInitEba();

    {
        uint32 uidx;

        DIM_HYPOUT_CAL_DATA_EBA.CalibMask = 0u;

        for (uidx = 0uL; uidx < DIM_NO_CALIB_HYPS; uidx++) {
            DIM_HYPOUT_CAL_DATA_EBA.Probability[uidx] = 0;
            DIM_HYPOUT_CAL_DATA_EBA.Confidence[uidx] = (percentage_t)0;
        }
    }

    DIMInitSigPreProc();
}

/*************************************************************************************************************************
  Functionname:    DIMCollectInputs */
static void DIMCollectInputs(void) {
#if 0
  printf("[func:%s], eSigStatus-->%d\n", __func__, VLC_pDIMGenericDataIn->sSigHeader.eSigStatus);
  printf("[func:%s], fAccelPedalPos-->%f\n", __func__, VLC_pDIMGenericDataIn->fAccelPedalPos); 
  printf("[func:%s], eAccelPadelStat-->%d\n", __func__, VLC_pDIMGenericDataIn->eAccelPadelStat); 
  printf("[func:%s], fAccelPedalGrad-->%f\n", __func__, VLC_pDIMGenericDataIn->fAccelPedalGrad); 
  printf("[func:%s], eSteeringWheelAngleGradStat-->%d\n", __func__, VLC_pDIMGenericDataIn->eSteeringWheelAngleGradStat);
  printf("[func:%s], eDriverBraking-->%d\n", __func__, VLC_pDIMGenericDataIn->eDriverBraking); 
  printf("[func:%s], eTurnIndicator-->%d\n", __func__, VLC_pDIMGenericDataIn->eTurnIndicator); 
  printf("[func:%s], EGO_SPEED_X_RAW-->%f\n", __func__, EGO_SPEED_X_RAW); 
  printf("[func:%s], EGO_ACCEL_X_RAW-->%f\n", __func__, EGO_ACCEL_X_RAW); 
  printf("[func:%s], eSpeedLimitActive-->%d\n", __func__, VLC_pDIMCustDataIn->eSpeedLimitActive);
#endif
    /*Generic data*/
    if (VLC_pDIMGenericDataIn->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        /* Driver actions driving pedal */
        DIMSetInputValueFloat(&(pDimInputGlobal->GasPedalPosition),
                              VLC_pDIMGenericDataIn->fAccelPedalPos,
                              VLC_pDIMGenericDataIn->eAccelPadelStat, 0);
        DIMSetInputValueFloat(&(pDimInputGlobal->GasPedalGradient),
                              VLC_pDIMGenericDataIn->fAccelPedalGrad,
                              VLC_pDIMGenericDataIn->eAccelPadelGradStat, 0);

        /* Driver actions steering wheel */
        DIMSetInputValueFloat(&(pDimInputGlobal->SteeringWheelAngle),
                              VLC_pDIMGenericDataIn->fSteeringWheelAngle,
                              VLC_pDIMGenericDataIn->eSteeringWheelAngleStat,
                              0);
        DIMSetInputValueFloat(
            &(pDimInputGlobal->SteeringWheelGradient),
            VLC_pDIMGenericDataIn->fSteeringWheelAngleGrad,
            VLC_pDIMGenericDataIn->eSteeringWheelAngleGradStat, 0);

        /* Driver actions brake pedal */
        if (VLC_pDIMGenericDataIn->eDriverBraking != eVLC_STATE_SIG_INVALID) {
            DIMSetInputValueBoolTest(
                &(pDimInputGlobal->DriverBraking),
                VLC_pDIMGenericDataIn->eDriverBraking == eVLC_STATE_SIG_ACTIVE,
                DIMInputSignalState_OK);
        } else {
            pDimInputGlobal->DriverBraking.eSignalQuality =
                DIMInputSignalState_Missing;
        }

        if (VLC_pDIMGenericDataIn->eTurnIndicator != eTurnIndicator_Invalid) {
            DIMSetInputValueUInt(&(pDimInputGlobal->TurnIndicator),
                                 VLC_pDIMGenericDataIn->eTurnIndicator,
                                 DIMInputSignalState_OK);
        } else {
            /* Final default value is determined in attention module */
            DIMSetInputValueUInt(&(pDimInputGlobal->TurnIndicator), 0u,
                                 DIMInputSignalState_Missing);
        }

    }
    /*Generic data - signal state not ok!*/
    else {
        /* Driver actions driving pedal */
        DIMSetInputValueFloat(&(pDimInputGlobal->GasPedalPosition), 0,
                              DIMInputSignalState_Missing, 0);
        DIMSetInputValueFloat(&(pDimInputGlobal->GasPedalGradient), 0,
                              DIMInputSignalState_Missing, 0);

        /* Driver actions steering wheel */
        DIMSetInputValueFloat(&(pDimInputGlobal->SteeringWheelAngle), 0,
                              DIMInputSignalState_Missing, 0);
        DIMSetInputValueFloat(&(pDimInputGlobal->SteeringWheelGradient), 0,
                              DIMInputSignalState_Missing, 0);
        /* Driver actions brake pedal */
        pDimInputGlobal->DriverBraking.eSignalQuality =
            DIMInputSignalState_Missing;

        DIMSetInputValueUInt(&(pDimInputGlobal->TurnIndicator), 0u,
                             DIMInputSignalState_Missing);
    }

    if (VLC_pDIMCustDataIn->eSpeedLimitActive != eVLC_STATE_SIG_INVALID) {
        DIMSetInputValueBoolTest(
            &(pDimInputGlobal->SpeedLimiter),
            VLC_pDIMCustDataIn->eSpeedLimitActive == eVLC_STATE_SIG_ACTIVE,
            DIMInputSignalState_OK);
    } else {
        pDimInputGlobal->SpeedLimiter.eSignalQuality =
            DIMInputSignalState_Missing;
    }

    if (GET_EGO_RAW_DATA_PTR->sSigHeader.eSigStatus == AL_SIG_STATE_OK) {
        if ((EGO_SPEED_X_VAR_RAW >= 0) &&
            (VED_IO_STATE_VALID == EGO_SPEED_X_STATE)) {
            DIMSetInputValueFloat(&(pDimInputGlobal->VehicleVelocity),
                                  EGO_SPEED_X_RAW, DIMInputSignalState_OK,
                                  EGO_SPEED_X_VAR_RAW);
        } else {
            DIMSetInputValueFloat(&(pDimInputGlobal->VehicleVelocity), 0,
                                  DIMInputSignalState_Missing, 0);
        }
        if ((EGO_ACCEL_X_VAR_RAW >= 0) &&
            (VED_IO_STATE_VALID == EGO_ACCEL_X_STATE)) {
            DIMSetInputValueFloat(&(pDimInputGlobal->VehicleAcceleration),
                                  EGO_ACCEL_X_RAW, DIMInputSignalState_OK,
                                  EGO_ACCEL_X_VAR_RAW);
        } else {
            DIMSetInputValueFloat(&(pDimInputGlobal->VehicleAcceleration), 0,
                                  DIMInputSignalState_Missing, 0);
        }
    }
    /* VDY data - signal state not ok */
    else {
        DIMSetInputValueFloat(&(pDimInputGlobal->VehicleVelocity), 0,
                              DIMInputSignalState_Missing, 0);
        DIMSetInputValueFloat(&(pDimInputGlobal->VehicleAcceleration), 0,
                              DIMInputSignalState_Missing, 0);
    }
}

/*************************************************************************************************************************
  Functionname:    DIMCopyHypothesis */
static void DIMCopyHypothesis(
    const GDB_DMHypothesis_t *hypIn,
    GDB_DMHypothesis_t hypOut[],
    uint8 uiHypNr,
    uint8 uiTotalHypNr,
    const DIM_HYPOUT_CAL_DATA_t *DIM_HYPOUT_CAL_DATA) {
    /* Check if sufficient space on output is provided */
    if (uiHypNr < uiTotalHypNr) {
        /* Copy whole structure data */
        hypOut[uiHypNr] = *hypIn;

        /* As soon as calibration bit set for current hypothesis number, use
           calibration value instead of calculated one */
        if (DIM_HYPOUT_CAL_DATA != 0) {
            if (((DIM_HYPOUT_CAL_DATA->CalibMask & (1uL << uiHypNr)) != 0uL) &&
                (uiHypNr < DIM_NO_CALIB_HYPS)) {
                hypOut[uiHypNr].Confidence =
                    DIM_HYPOUT_CAL_DATA->Confidence[uiHypNr];
                hypOut[uiHypNr].Probability =
                    DIM_HYPOUT_CAL_DATA->Probability[uiHypNr];
            }
        }
    }
}

/*************************************************************************************************************************
  Functionname:    DIMInitHypothesis */
static void DIMInitHypothesis(GDB_DMHypothesis_t *const hyp) {
    hyp->Confidence = (percentage_t)0;
    hyp->Probability = (sint8)0;
    hyp->eType = DIMHypoType_No;
    hyp->eGDBError = GDB_ERROR_UNKNOWN_TYPE;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */