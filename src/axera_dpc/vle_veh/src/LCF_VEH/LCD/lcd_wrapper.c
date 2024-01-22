/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "lcd_ext.h"
#include "tue_common_libs.h"
#include "lcd_par.h"
#include "lcd_str_trq_ctrl.h"
/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/
#define EP30_LCD_VERSION
#define LCD_SCALE_COEFFICIENT 1.2f

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  CONSTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
#define ASW_QM_CORE2_MODULE_START_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
static uint8 uiLCDState = LCD_STATE_INIT;

/* flag which indicates initialization status */
static boolean bLCDInitialized = FALSE;
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/* *************************************************************************
  @fn            LCDReset                                               */ /*!
        @brief         Initializes all LCD variables at the beginning of the
                       program run.
        @description   Initializes all variables of the LCK component at the
                       beginning of the program run.
        @param[in]     void
        @return        void
      ****************************************************************************
      */
void LCDReset(void) {
    eGDBError_t eGDBError = GDB_ERROR_NONE;

    //-- Set external status and internal initialization flag
    //-------------------
    if (eGDBError == GDB_ERROR_NONE) {
        bLCDInitialized = TRUE;
        uiLCDState = LCD_STATE_OK;
    }
    LCDPIDControlInit();
    LCDDevControlInit();
}

/* *************************************************************************
  @fn            LCDProcess                                              */ /*!
      @brief         Runs one LCD loop for all modules
      @description   Runs one LCD loop for all modules.
      @param[in]     fCycleTime : cycle time [s]
      @return        void
    ****************************************************************************
    */
void LCDProcess(const sLCDInput_t* reqPorts,
                const sLCDParams_t* proParams,
                sLCDOutput_t* proPorts,
                sLCDDebug_t* proDebugs) {
    eGDBError_t eGDBError = GDB_ERROR_NONE;

    //-- check if LCD was initialized
    //-------------------------------------------
    //   The LCD state can be reset from outside (frame), e.g. to force a
    //   reinitialization of the function. If function internally the
    //   initialization was not performed yet, but the state sat from outside is
    //   other than init, than an initialization has to be performed
    //   regardingless
    //   of the curent state.
    if (bLCDInitialized == FALSE) {
        uiLCDState = LCD_STATE_INIT;
    }

    switch (uiLCDState) {
        case LCD_STATE_OK:

            /* collect input data */
            // eGDBError = LCDGetInputData();
            // TODO check LCD input data
            /* run LCD */
            if (eGDBError == GDB_ERROR_NONE) {
                eGDBError =
                    LCDRun(reqPorts, reqPorts->fCycleTime_sec, proPorts);
            }

            /* write data to output interface */
            if (eGDBError == GDB_ERROR_NONE) {
                // LCDWriteOutput();

                /* MTS data freeze */
                // LCDCustomMTSFreezeData();
            } else {
                bLCDInitialized = FALSE;
                uiLCDState = LCD_STATE_INIT;
            }
            break;

        case LCD_STATE_INIT:
        default:
            LCDReset();
            break;

    }  // end switch LCDState

}  // end LCDProcess
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
