/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lck.h"

#include "lck_par.h"
//#include "lcf_ext.h"
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
sLCKDebugData_t* pLCKGlobalDebugData;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* operating modes of sub-component used by external framework */
LCKState_t LCKState = LCK_STATE_INIT;

/* flag which indicates initialization status */
static boolean bLCKInitialized = FALSE;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  PROTOTYPES
*****************************************************************************/

static eGDBError_t LCKGetVehSpecParam(void);

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

/* *************************************************************************
  @fn            LCKInit                                               */ /*!
          @brief         Initializes all LCK variables at the beginning of the
                         program run.
          @description   Initializes all variables of the LCK component at the
                         beginning of the program run.
          @param[in]     void
          @return        void
        ****************************************************************************
        */
void LCKReset(void) {
    eGDBError_t eGDBError = GDB_ERROR_NONE;

    //-- Set external status and internal initialization flag
    //-------------------
    if (eGDBError == GDB_ERROR_NONE) {
        bLCKInitialized = TRUE;
        LCKState = LCK_STATE_OK;
    }

    pLCKGlobalDebugData = &(sLCKDebug.sData);
}

/* *************************************************************************
  @fn            LCKProcess                                              */ /*!
      @brief         Runs one LCK loop for all modules
      @description   Runs one LCK loop for all modules.
      @param[in]     void
      @return        void
    ****************************************************************************
    */
void LCKProcess(const sLCKInput_t* reqPorts,
                const sLCKParam_t* proParams,
                sLCKOutput_t* proPorts,
                sLCKDebug_t* proDebugs) {
    eGDBError_t eGDBError = GDB_ERROR_NONE;

    // eGDBError = GetLKSOutPutFromLDP();

    //-- check if LCK was initialized
    //-------------------------------------------
    //   The LCK state can be reset from outside (frame), e.g. to force a
    //   reinitialization of the function. If function internally the
    //   initialization was not performed yet, but the state sat from outside is
    //   other than init, than an initialization has to be performed
    //   regardingless
    //   of the curent state.
    if (bLCKInitialized == FALSE) {
        LCKState = LCK_STATE_INIT;
    }

    switch (LCKState) {
        case LCK_STATE_OK:
            /* get platform specific vehicle parameters */
            eGDBError = LCKGetVehSpecParam();

            /* collect input data */
            if (eGDBError == GDB_ERROR_NONE) {
                // eGDBError = LCKGetInputData();
            }

            /* run LCK */
            if (eGDBError == GDB_ERROR_NONE) {
                eGDBError = LCKRun(reqPorts, proParams, proPorts,
                                   reqPorts->fCycleTime_sec);
            }

            /* write data to output interface */
            if (eGDBError == GDB_ERROR_NONE) {
                // LCKWriteOutput();

                /* MTS data freeze */
                // LCKCustomMTSFreezeData();

            } else {
                bLCKInitialized = FALSE;
                LCKState = LCK_STATE_INIT;
            }
            break;  // end State OK

        case LCK_STATE_INIT:
        default:
            LCKReset();
            break;
    }
}  // end LCKProcess

/* *************************************************************************
  @fn            LCKInitParam                                         */ /*!
            @brief         initialize LCK parameter
            @description   initialize LCK parameter
            @param[in]     fCycleTime : cycle time [s]
            @return        eGDBError_t
          ****************************************************************************
          */

/* *************************************************************************
  @fn            LCKGetVehSpecParam                                        */ /*!
  @brief         Set vehicle specific parameters provided on IPC
  @description   Set vehicle specific parameters provided on IPC. Vehicle
                 specific parameters are provided on IPC and stored in
                 global parameters at camera start up and if hash value
                 changes. If hash value does not change, global parameter
                 values are kept as they are. This ensures, that parameter
                 values can be changed online by CANape.
                 If the pinter to the parameter values on IPC is invalid,
                 an error code will be returned and parameters will not be
                 modified.
                 Note: The internally used hash value is only initialized
                 once at camera start up.
  @param[in]     void
  @return        eGDBError_t
**************************************************************************** */
static eGDBError_t LCKGetVehSpecParam(void) {
    eGDBError_t eGDBError = GDB_ERROR_NONE; /* error code */
    // uint32 uiIdx = 0u;                      /* index for loops */
    // TODO guotao 20200415
    // if(   (FCTVEH_pCPAR_FCT_LKS_Parameters == NULL)
    //   || (pFCTVehServiceFuncts            == NULL) )
    //{
    //  eGDBError = GDB_ERROR_POINTER_NULL;
    //}
    // else
    //{
    //  if( uiLCKVehSpecParamHash !=
    //  FCTVEH_pCPAR_FCT_LKS_Parameters->Hash.LCKCodingParamHash )
    //  {
    //    /* update hash value */
    //    uiLCKVehSpecParamHash =
    //    FCTVEH_pCPAR_FCT_LKS_Parameters->Hash.LCKCodingParamHash;

    //    /* update parameter */
    //    for(uiIdx = 0; uiIdx < LCK_PAR_NO_SUPPORT_POINTS; uiIdx++)
    //    {
    //      /* no range check because range can not be specified */
    //      fLCKParStrAngGainsArray_c[uiIdx]  =
    //      FCTVEH_pCPAR_FCT_LKS_Parameters->LCK.LCKCodingParamStrAngGains[uiIdx];
    //      fLCKParStrVelGainsArray_c[uiIdx]  =
    //      FCTVEH_pCPAR_FCT_LKS_Parameters->LCK.LCKCodingParamStrAngVelGains[uiIdx];
    //      fLCKParYawRateGainsArray_c[uiIdx] =
    //      FCTVEH_pCPAR_FCT_LKS_Parameters->LCK.LCKCodingParamYawRateGains[uiIdx];
    //      fLCKParHeadingGainsArray_c[uiIdx] =
    //      FCTVEH_pCPAR_FCT_LKS_Parameters->LCK.LCKCodingParamHeadingGains[uiIdx];
    //      fLCKParLatDevGainsArray_c[uiIdx]  =
    //      FCTVEH_pCPAR_FCT_LKS_Parameters->LCK.LCKCodingParamLatDevGains[uiIdx];
    //    }

    //    LCK_PAR_STR_ANG_MASTER_GAIN  = 1.0f;
    //    LCK_PAR_STR_VEL_MASTER_GAIN  = 1.0f;
    //    LCK_PAR_YAW_RATE_MASTER_GAIN = 1.0f;
    //    LCK_PAR_HEADING_MASTER_GAIN  = 1.0f;
    //    LCK_PAR_LAT_DEV_MASTER_GAIN  = 1.0f;

    //    if( FCTVEH_pCPAR_FCT_LKS_Parameters->LCK.LCKCodingParamLinFadingCoef <
    //    LCK_VEH_SPEC_PARAM_LIN_FADING_COEF_MIN )
    //    {
    //      eGDBError = GDB_ERROR_VALUE_RANGE;
    //    }
    //    else
    //    {
    //      LCK_PAR_LIN_FADING_COEF =
    //      FCTVEH_pCPAR_FCT_LKS_Parameters->LCK.LCKCodingParamLinFadingCoef;
    //    }

    //  } /* end if hash value changed */

    //  /* output DEM event if parameter values are out of expected range */
    //  //if( eGDBError == GDB_ERROR_VALUE_RANGE )
    //  //{
    //  //  pFCTVehServiceFuncts->pfDem_SetEventStatus(LKS_INPUT_OUT_OF_BOUNDS,
    //  DEM_EVENT_STATUS_FAILED);
    //  //  pFCTVehServiceFuncts->pfDem_SetEventStatus(FCTVEH_DEM_STATIC,
    //  DEM_EVENT_STATUS_FAILED);
    //  //}
    //  //else
    //  //{
    //  //  pFCTVehServiceFuncts->pfDem_SetEventStatus(LKS_INPUT_OUT_OF_BOUNDS,
    //  DEM_EVENT_STATUS_PASSED);
    //  //  pFCTVehServiceFuncts->pfDem_SetEventStatus(FCTVEH_DEM_STATIC,
    //  DEM_EVENT_STATUS_PASSED);
    //  //}

    //} /* end NULL pointer check */

    return eGDBError;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
