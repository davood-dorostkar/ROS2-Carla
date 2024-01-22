/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/**
@ingroup lcd
@{ */

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcd_ext.h"
#include "tue_common_libs.h"
#include "lcd.h"
#include "lcd_str_trq_ctrl.h"
#include "lcd_par.h"
#ifndef NULL
#define NULL (0)
#endif
/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

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
// SET_MEMSEC_VAR(bRampOut)
static boolean bRampOut = FALSE; /* flag, which controls torque ramp out */

// SET_MEMSEC_VAR(fLastTrqReq)
static float32 fLastTrqReq = LCD_INIT_F_ZERO; /* last torque request */

// SET_MEMSEC_VAR(fLCDOnTimer)
static float32 fLCDOnTimer =
    LCD_INIT_F_ZERO; /* time that LCD control is switched on */
#define ASW_QM_CORE2_MODULE_STOP_SEC_VAR_UNSPECIFIED
#include "ASW_MemMap.h"
/*****************************************************************************
  PROTOTYPES
*****************************************************************************/

/* ***********************************************************************
  @fn            LCDRun                                                  */ /*!
  @brief         runs one LCD loop for all modules
  @description   runs one LCD loop for all modules
  @param[in]     pInputData : All internal input data for LCD module
  @param[in]     CycleTime  : Cycle time [s]
  @param[out]    pLCDOutput : All internal output data from LCD module
  @return        eGDBError_t : error code
  @pre           [none]
  @post          [none]
**************************************************************************** */
eGDBError_t LCDRun(sLCDInput_t const *pInputData,
                   float32 const fCycleTime,
                   sLCDOutput_t *pLCDOutput) {
    //-- declaration and initialization ---------------------------------------
    eGDBError_t eRetValue = GDB_ERROR_NONE;

    //-- check for valid input data (NULL pointer) ----------------------------
    if ((pInputData == NULL) || (pLCDOutput == NULL)) {
        eRetValue = GDB_ERROR_POINTER_NULL;
    } else {
        //-- select controller -------------------------------------------------
        switch (pInputData->eLCDMode) {
            /***********************************************************************
             * SWITCH OFF CONTROLLER (ramp out) *
             ***********************************************************************/
            case LCD_MODE_OFF:

                fLCDOnTimer = LCD_INIT_F_ZERO;

                if (bRampOut == TRUE) {
                    bRampOut = LCDRampOut(fCycleTime, fLastTrqReq, pLCDOutput);
                } else {
                    LCDOff(pLCDOutput);
                }
                break;

            /***********************************************************************
             * SWITCH OFF CONTROLLER (immediately) *
             ***********************************************************************/
            case LCD_MODE_OFF_NOW:

                fLCDOnTimer = LCD_INIT_F_ZERO;

                LCDOff(pLCDOutput);
                break;

            /***********************************************************************
             * SWITCH ON CONTROLLER (use PID controller) *
             ***********************************************************************/
            case LCD_MODE_ON_PID:

                eRetValue = LCDPIDControl(pInputData, fCycleTime, fLCDOnTimer,
                                          pLCDOutput);
                bRampOut = TRUE;
                fLCDOnTimer = fLCDOnTimer + fCycleTime;
                fLCDOnTimer =
                    MIN(fLCDOnTimer, LCD_PAR_FF_DIODE_MAX_ON_TIME + fCycleTime);
                break;

            /***********************************************************************
             * SWITCH ON CONTROLLER (use development controller) *
             ***********************************************************************/
            case LCD_MODE_ON_DEV:

                eRetValue = LCDDevControl(pInputData, fCycleTime, fLCDOnTimer,
                                          pLCDOutput);
                bRampOut = TRUE;
                fLCDOnTimer = fLCDOnTimer + fCycleTime;
                fLCDOnTimer =
                    MIN(fLCDOnTimer, LCD_PAR_FF_DIODE_MAX_ON_TIME + fCycleTime);
                break;

            /***********************************************************************
             * ERROR (switch off immediately and output error) *
             ***********************************************************************/
            default:

                LCDOff(pLCDOutput);
                bRampOut = FALSE;
                eRetValue = GDB_ERROR_VALUE_RANGE;
                break;

        }  // end switch LCDMode

        /* update states */
        fLastTrqReq = pLCDOutput->fTorque;

    }  // end null pointer check

    return eRetValue;
}

/** @} end ingroup */
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE2_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
