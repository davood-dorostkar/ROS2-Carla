/**
@defgroup lcd LCD (Lane Control Dynamic)
  @ingroup fct_veh
general component to implement the dynamic layer of lateral vehicle controller

@{ */

#ifndef _LCD_EXT_H_INCLUDED
#define _LCD_EXT_H_INCLUDED
#ifdef __cplusplus
extern "C" {
#endif
/*** START OF SINGLE INCLUDE SECTION ****************************************/

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "TM_Global_Types.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/
#define LCD_MAX_SPEED_ARRAY_SIZE (16u)
/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*! Operations modes for external framework control */
typedef enum {
    LCDOpMode_Running,  /*!< normal operation    */
    LCDOpMode_ShutDown, /*!< normal deactivation */
    LCDOpMode_Stop      /*!< severe deactivation */
} eLCDOpMode_t;

/*! Operating modes of sub-component */
# ifndef Rte_TypeDef_LCDState_t
typedef enum {
    LCD_STATE_INIT = 0u, /*!< initialize all    */
    LCD_STATE_OK = 1u    /*!< normal processing */
} LCDState_t;
# define Rte_TypeDef_LCDState_t
# endif
// typedef enum
//{
//	GDB_ERROR_NONE = 0u,   /*!< no error*/
//	GDB_ERROR_POINTER_NULL = 1u,   /*!< null pointer error*/
//	GDB_ERROR_FUNC_POINTER_NULL = 2u,   /*!< null function pointer error*/
//	GDB_ERROR_CAST_VALUE_TO_HIGH = 3u,   /*!< while casting a variable, the
// value would be truncated because it was to high (example (signed char)200)*/
//	GDB_ERROR_CAST_VALUE_TO_LOW = 4u,   /*!< while casting a variable, the
// value would be truncated because it was to low  (example (signed char)-200)*/
//	GDB_ERROR_UNKNOWN_TYPE = 5u,   /*!< enum type handling unknown*/
//	GDB_ERROR_FILTER_DOESNT_MATCH = 6u,   /*!< a filter function doesn't
// match any result*/
//	GDB_ERROR_LOW_QUALITY = 7u,   /*!< at least one signal didn't have a
// sufficient quality*/
//	GDB_ERROR_ARRAY_OVERFLOW = 8u,   /*!< the array index is out of bounds*/
//	GDB_ERROR_ZERO_DEVISION = 9u,   /*!< Devision by Zero requested*/
//	GDB_ERROR_VALUE_RANGE = 10u   /*!< a value is outside expeccted range */
//} eGDBError_t;

//-- LCD modes ----------------------------------------------------------------
typedef enum {

    LCD_MODE_OFF = 0u,     /*!<  dynamic lateral controller off  (ramp out)   */
    LCD_MODE_OFF_NOW = 1u, /*!<  dynamic lateral controller off  (immediatly) */
    LCD_MODE_ON_PID = 2u,  /*!<  use PID controller                           */
    LCD_MODE_ON_DEV = 3u,  /*!<  use development controller                   */
    LCD_MODE_SYSID = 4u    /*!<  system identification procedure              */

} eLCDMode_t;

//-- main LCD input data structure --------------------------------------------
typedef struct {
    eLCDOpMode_t eOpMode; /*!< external operation mode                     */
    eLCDMode_t eLCDMode;  /*!< mode of dynamic lateral controller          */
    float32 fRefSteerAng; /*!< steering angle to be applied [rad]          */
    float32 fSteerAng;    /*!< current steering angle at front tires [rad] */
    float32 fSteerAngVel; /*!< steering angle velocity [rad/s]             */
    float32 fVhclSpeed;   /*!< vehicle speed [m/s]                         */
    float32 fDrvTrq;      /*!< driver applied steering torque [Nm]         */
    float32 fCycleTime_sec;
} sLCDInput_t;

/*****************************************************************************
 *                                                                           *
 *     LCD output                                                            *
 *                                                                           *
 *****************************************************************************/
typedef struct {
    LCDState_t uiLCDState;
    boolean bTorqueReq;  /*!< flag which indicates torque request */
    float32 fTorque;     /*!< requested steering torque [Nm]      */
    float32 fTorqueGrad; /*!< requested steering torque gradient  */

} sLCDOutput_t;

typedef struct { uint32 uiVersionNum; } sLCDParams_t;

#ifndef Rte_TypeDef_sLCDDebugData_t
typedef struct {
    float32 fTorqueRate;     /*!<  requested torque rate [Nm/s] */
    float32 fRefStrAngFilt;  /*!< filtered reference steering angle [rad] */
    float32 fStrAngVelTrq;   /*!< torque component from steering angle velocity
                                [Nm] */
    float32 fFeedForwardTrq; /*!< torque component from feed forward [Nm] */
    float32 fPIDTrq; /*!< torque component from PID controller [Nm]          */

} sLCDDebugData_t;
#define Rte_TypeDef_sLCDDebugData_t
#endif

#ifndef Rte_TypeDef_sLCDDebugParam_t
typedef struct {
    float32 fVhclSpeedArray[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< vehicle speed array
                                                          for gain
                                                          scheduling [m/s] */
    float32 fMaxStrTrq[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< limit for steering torque
                                                     amplitude [Nm] */
    float32 fMaxStrTrqRate[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< limit for steering
                                                         torque amplitude [Nm]
                                                         */
    float32 fRampOutTrqGrad; /*!< Ramp Out Torque Gradient [Nm/s] */
    float32 fPGains[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< p-gain array [-] */
    float32 fIGains[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< i-gain array [-] */
    float32 fDGains[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< d-gain array [-] */
    float32 fPIDFilterCoeff;    /*!< PID filter coefficient [-] */
    float32 fAWDFeedbackFactor; /*!< anti windup feedback factor [-] */
    float32 fIGainsFB[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< integral gain feedback
                                                    factor array [-] */
    float32 fFFTorque[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< feedforward torque array
                                                    [-] */
    float32
        fFFTorqueFilterCoeff;  /*!< feedforward torque filter coefficient [-] */
    float32 fFFDiodeMaxOnTime; /*!< Max time after activating controller to
                                  enable feed forward diode [s] */
    float32 fPrefiltEigFreq[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< eigen frequencies
                                                          for prefiltering
                                                          reference steering
                                                          angle           */
    float32
        fPrefiltTimeConst[LCD_MAX_SPEED_ARRAY_SIZE]; /*!< time constants for
                                                        prefiltering reference
                                                        steering angle */
    float32 fStrVelTrqDamping; /*!< steering velocity torque damping */

} sLCDDebugParam_t;
#define Rte_TypeDef_sLCDDebugParam_t
#endif

#ifndef Rte_TypeDef_sLCDDebug_t
typedef struct {
    sLCDDebugData_t sData;   /*!<  debug data      */
    sLCDDebugParam_t sParam; /*!<  debug parameter */

} sLCDDebug_t; /* LCD Debug Output */
#define Rte_TypeDef_sLCDDebug_t
#endif

/*****************************************************************************
  GLOBAL CONSTANTS (EXTERNAL SCOPE)
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES (EXTERNAL SCOPE)
*****************************************************************************/

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
/* entry functions for external framework */
void LCDReset(void);
void LCDProcess(const sLCDInput_t *reqPorts,
                const sLCDParams_t *proParams,
                sLCDOutput_t *proPorts,
                sLCDDebug_t *proDebugs);

/*****************************************************************************
  INLINE FUNCTION
*****************************************************************************/

/*** END OF SINLGE INCLUDE SECTION ******************************************/
#ifdef __cplusplus
}
#endif
#endif /* _LCD_EXT_H_INCLUDED */

/** @} end defgroup */
