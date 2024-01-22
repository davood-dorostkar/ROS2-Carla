
#ifndef _LCD_STR_TRQ_CTRL_H_INCLUDED
#define _LCD_STR_TRQ_CTRL_H_INCLUDED
/*** START OF SINGLE INCLUDE SECTION ****************************************/
#ifdef __cplusplus
extern "C" {
#endif
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "lcd.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  GLOBAL CONSTANTS (EXTERNAL SCOPE)
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES (EXTERNAL SCOPE)
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES (EXTERNAL SCOPE)
*****************************************************************************/
void LCDPIDControlInit(void);
extern eGDBError_t LCDPIDControl(sLCDInput_t const* sLCDInput,
                                 float32 const fCycleTime,
                                 float32 const fLCDOnTime,
                                 sLCDOutput_t* sLCDOutput);

void LCDDevControlInit(void);
extern eGDBError_t LCDDevControl(sLCDInput_t const* sLCDInput,
                                 float32 const fCycleTime,
                                 float32 const fLCDOnTime,
                                 sLCDOutput_t* sLCDOutput);

/* switch off controller with ramp out */
extern boolean LCDRampOut(const float32 fCycleTime,
                          const float32 fLastTrqReq,
                          sLCDOutput_t* sLCDOutput);

/* switch off controller immediately */
extern void LCDOff(sLCDOutput_t* sLCDOutput);

#ifdef __cplusplus
}
#endif
#endif /* _LCD_STR_TRQ_CTRL_H_INCLUDED */
