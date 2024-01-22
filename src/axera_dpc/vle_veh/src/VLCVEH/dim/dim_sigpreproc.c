
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "dim_cfg.h"
#include "dim.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*! @brief       Number of previous input samples to be saved by the
   differentiator
    @general     differentiator will save this number based on how much previous
   input samples are available and processed
    @attention   [none]
    @typical     3        @unit [none]     @min 0   @max 65535   */
#define NO_PRE_INPUT_SAMPLES 3

/*! @brief       Number of previous time samples to be saved by the
   differentiator
    @general     differentiator will save this number based on how much previous
   time samples are available and processed
    @attention   [none]
    @typical     2        @unit [none]     @min 0   @max 65535   */
#define NO_PRE_TIME_SAMPLES 2

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/* ****************************************************************
    TYPEDEF STRUCT DimIirFilt_t
    **************************************************************** */
/*! @brief Differentiator structure for calculation of the DIM steering wheel
   angular velocity

    @general Differentiator structure for calculation of the DIM steering wheel
   angular velocity

    @conseq [None]

    @attention [None]

    */
typedef struct {
    float32
        fInputs[NO_PRE_INPUT_SAMPLES]; /*!< Array of previous input samples */
    float32 fCycleTimes[NO_PRE_TIME_SAMPLES]; /*!< Array of previous cycle time
                                                 samples */
    uint32 uiValues; /*!< Number of values to be considered in calculation */
} DimIirFilt_t;

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/* Static differentiator used for the calculation of the DIM steering wheel
 * angular velocity */
SET_MEMSEC_VAR(DimIirFilt)
/*!  @cond Doxygen_Suppress */
static DimIirFilt_t DimIirFilt;
/*! @endcond */
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"

/*****************************************************************************
  PROTOTYPES
*****************************************************************************/
static void DimDifferentiatorInit(DimIirFilt_t *pFilt);
static boolean DimDifferentiatorRun(float32 *fOutValue,
                                    float32 fInValue,
                                    float32 fCycleTime,
                                    DimIirFilt_t *pFilt);

/* ***********************************************************************
  Functionname:    DimDifferentiatorInit */ /*!

@brief           DIM differentiator initialization

@description     Initialization function of the DIM differentiator

@return         static void

@param[in,out]    pFilt :   The static DIM differentiator \n
pFilt->uiValues: Number of values to be considered in calculation
[full range of unsigned long]\n
pFilt->fInputs: Array of previous input samples
[full range of float]\n
pFilt->fCycleTimes: Array of previous cycle time samples
[full range of float]\n

@c_switch_full    VLC_CFG_DRIVER_INTENTION_MONITORING : VLC driver intention
monitoring switch
@c_switch_full    VLC_CFG_DIM_IN_PREPROC: VLC DIM input preprocessing module

@pre             [None]
@post            [None]

@created         -

@changed         -

**************************************************************************** */
static void DimDifferentiatorInit(DimIirFilt_t *pFilt) {
    pFilt->uiValues = 0u;

    pFilt->fInputs[2] = 0;
    pFilt->fInputs[1] = 0;
    pFilt->fInputs[0] = 0;

    pFilt->fCycleTimes[0] = 0;
    pFilt->fCycleTimes[1] = 0;
}

/* ***********************************************************************
  Functionname:    DimDifferentiatorRun */ /*!

@brief           DIM differentiator function

@description     Main function for calculating the differentiation of DIM
signals

@return         retValid   : A boolean indicating the validity of the calculated
output [0:Invalid , 1: Valid]

@param[in, out]	fOutValue :
[full range of float]\n
@param[in]		fInValue:
[full range of float]\n
@param[in]		fCycleTime:
[full range of float]\n
@param[in, out    pFilt :   The static DIM differentiator \n
pFilt->uiValues: Number of values to be considered in calculation
[full range of unsigned long]\n
pFilt->fInputs: Array of previous input samples
[full range of float]\n
pFilt->fCycleTimes: Array of previous cycle time samples
[full range of float]\n

@c_switch_full    VLC_CFG_DRIVER_INTENTION_MONITORING : VLC driver intention
monitoring switch
@c_switch_full    VLC_CFG_DIM_IN_PREPROC: VLC DIM input preprocessing module

@pre             [None]
@post            [None]

@created         -

@changed         -

**************************************************************************** */
static boolean DimDifferentiatorRun(float32 *fOutValue,
                                    float32 fInValue,
                                    float32 fCycleTime,
                                    DimIirFilt_t *pFilt) {
    boolean retValid;  /* Indicator of output validity */
    float32 diffTime;  /* difference in time (dt) */
    float32 diffValue; /* difference in value (dx) */

    /* At t == 0 the output shall be equal to zero  */
    if (pFilt->uiValues == 0uL) {
        diffTime = fCycleTime;
        diffValue = 0;
    }
    /* At t == dt, only 1 previous input value is considered */
    else if (pFilt->uiValues == 1uL) {
        diffTime = fCycleTime;
        diffValue = fInValue - pFilt->fInputs[0];
    }
    /* At t == 2dt, 2 previous input values shall be considered */
    else if (pFilt->uiValues == 2uL) {
        diffTime = pFilt->fCycleTimes[0] + fCycleTime;
        diffValue = fInValue - pFilt->fInputs[1];
    }
    /* At t >= 3dt, 3 previous input values shall be considered */
    else {
        diffTime = pFilt->fCycleTimes[1] + pFilt->fCycleTimes[0] + fCycleTime;
        diffValue = fInValue - pFilt->fInputs[2];
    }

    /* Determining the validity of the output and accordingly calculating the
     * output value */
    if (diffTime > C_F32_DELTA) {
        *fOutValue = diffValue / diffTime;
        retValid = TRUE;
    } else {
        *fOutValue = 0;
        retValid = FALSE;
    }

    /* Storage of the previous input values to be used in later calculations */
    pFilt->fInputs[2] = pFilt->fInputs[1];
    pFilt->fInputs[1] = pFilt->fInputs[0];
    pFilt->fInputs[0] = fInValue;

    /* Storage of the previous cycle times to be used in later calculations */
    pFilt->fCycleTimes[1] = pFilt->fCycleTimes[0];
    pFilt->fCycleTimes[0] = fCycleTime;

    /* Counter function of the values to be calculated. After t==3dt, no more
     * additional values need to be considered */
    if (pFilt->uiValues < 3U) {
        pFilt->uiValues++;
    }

    return retValid;
}

/* ***********************************************************************
  Functionname:    DIMInitSigPreProc */ /*!

@brief           DIM-Preprocessing initialization

@description     Initialization of the preprocessing of DIM signals

@return         void

@param[in, out]	void

@c_switch_full    VLC_CFG_DRIVER_INTENTION_MONITORING : VLC driver intention
monitoring switch
@c_switch_full    VLC_CFG_DIM_IN_PREPROC: VLC DIM input preprocessing module

@pre             [None]
@post            [None]

@created         -

@changed         -

**************************************************************************** */
void DIMInitSigPreProc(void) {
    /*Initilization of the differentiator*/
    DimDifferentiatorInit(&DimIirFilt);
}

/*************************************************************************************************************************
  Functionname:    DIMRunSigPreProc */
void DIMRunSigPreProc(float32 fCycleTime, DIMInputDataGlobal_t *pDimInput) {
    boolean bGradientValidity;

    /* Checking the validing of the pointer to DIM input structure and the
     * signal quality of the steering wheel angle */
    if ((pDimInput != NULL) && (pDimInput->SteeringWheelAngle.eSignalQuality ==
                                (uint8)DIMInputSignalState_OK)) {
        /* Calculating the steering wheel gradient from the steering wheel
         * angle*/
        bGradientValidity = DimDifferentiatorRun(
            &(pDimInput->SteeringWheelGradient.uValue.fValue),
            pDimInput->SteeringWheelAngle.uValue.fValue, fCycleTime,
            &DimIirFilt);
    } else {
        /* Steering wheel gradient is not valid because of bad steering angle
         * quality or invalid pointer to DIM input structure */
        bGradientValidity = 0;
    }

    if ((pDimInput != NULL) && (bGradientValidity == 1)) {
        /* Gradient is valid */
        pDimInput->SteeringWheelGradient.eSignalQuality =
            (uint8)DIMInputSignalState_OK;
    } else if ((pDimInput != NULL) && (bGradientValidity == 0)) {
        if (pDimInput->SteeringWheelAngle.eSignalQuality !=
            (uint8)DIMInputSignalState_OK) {
            /* Take Quality from Angle (Multiturn-loss is set to BadQuality)*/
            pDimInput->SteeringWheelGradient.eSignalQuality =
                pDimInput->SteeringWheelAngle.eSignalQuality;
        } else {
            /* Gradient is invalid */
            pDimInput->SteeringWheelGradient.eSignalQuality =
                (uint8)DIMInputSignalState_Missing;
        }

        /* Reinitialize Differentiator due to invalid gradient */
        DimDifferentiatorInit(&DimIirFilt);
    } else {
        /* pointer to DIM input structure is invalid */
    }
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */