/**
@defgroup lck LCK (Lane Control Kinematic)
  @ingroup fct_veh
general component to implement the kinematic layer of lateral vehicle controller

@{ */

#ifndef _LCK_EXT_H_INCLUDED
#define _LCK_EXT_H_INCLUDED
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
#define LCK_INIT_F_ZERO (0.0f)
#define LCK_MAX_SPEED_ARRAY_SIZE (16u)

#define LDP_LANE_DEPARTURE_LEFT 1  /*lane left departure danger signal*/
#define LDP_LANE_DEPARTURE_RIGHT 2 /*lane right departure danger signal*/
#define LDP_SYSOUT_NoPresent 0
#define LDP_SYSOUT_Disable 1 /*FCT LDP SysOut signal - disable state*/
#define LDP_SYSOUT_Passive 2 /*FCT LDP SysOut signal - stand by state*/
#define LDP_SYSOUT_Request 3 /*FCT LDP SysOut signal - request control state*/
#define LDP_SYSOUT_Control 4 /*FCT LDP SysOut signal - control state*/
#define LDP_SYSOUT_RampOut 5 /*FCT LDP SysOut signal - ramp out state*/
#define LDP_SYSOUT_Error 6   /*FCT LDP SysOut signal - error state*/

#define LDP_CAMERA_INPUT_SIGN_CORRECT -1.0F
/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*! Operations modes for external framework control */
typedef enum {
    LCKOpMode_Running,  /*!< normal operation    */
    LCKOpMode_ShutDown, /*!< normal deactivation */
    LCKOpMode_Stop      /*!< severe deactivation */
} eLCKOpMode_t;

typedef enum {
    GDB_ERROR_NONE = 0u,               /*!< no error*/
    GDB_ERROR_POINTER_NULL = 1u,       /*!< null pointer error*/
    GDB_ERROR_FUNC_POINTER_NULL = 2u,  /*!< null function pointer error*/
    GDB_ERROR_CAST_VALUE_TO_HIGH = 3u, /*!< while casting a variable, the value
                                          would be truncated because it was to
                                          high (example (signed char)200)*/
    GDB_ERROR_CAST_VALUE_TO_LOW = 4u,  /*!< while casting a variable, the value
                                          would be truncated because it was to
                                          low  (example (signed char)-200)*/
    GDB_ERROR_UNKNOWN_TYPE = 5u,       /*!< enum type handling unknown*/
    GDB_ERROR_FILTER_DOESNT_MATCH =
        6u, /*!< a filter function doesn't match any result*/
    GDB_ERROR_LOW_QUALITY =
        7u, /*!< at least one signal didn't have a sufficient quality*/
    GDB_ERROR_ARRAY_OVERFLOW = 8u, /*!< the array index is out of bounds*/
    GDB_ERROR_ZERO_DEVISION = 9u,  /*!< Devision by Zero requested*/
    GDB_ERROR_VALUE_RANGE = 10u    /*!< a value is outside expeccted range */
} eGDBError_t;

/*! Operating modes of sub-component used by external framwork */
typedef enum {
    LCK_STATE_INIT = 0u, /*!< initialize all    */
    LCK_STATE_OK = 1u    /*!< normal processing */
} LCKState_t;

/* LCK output data for LCD */
/*! Operation modes for dynamic controller (LCD) */
typedef enum {

    LCK_LCD_MODE_OFF =
        0u, /*!<  turn off dynamic lateral controller (ramp out)    */
    LCK_LCD_MODE_OFF_NOW =
        1u, /*!<  turn off dynamic lateral controller (immediately) */
    LCK_LCD_MODE_ON = 2u,   /*!<  turn on dynamic lateral controller */
    LCK_LCD_MODE_SYSID = 3u /*!<  perform system identification procedure */

} eLCKLCDCtrlMode_t;

/* LCK mode of operation */
typedef enum {

    LCK_MODE_OFF = 0u, /*!< kinematic lateral controller off (ramp out)    */
    LCK_MODE_OFF_NOW =
        1u,               /*!< kinematic lateral controller off (immediately) */
    LCK_MODE_ON_SP = 2u,  /*!< state space controller on                      */
    LCK_MODE_ON_CAS = 3u, /*!< cascade controller on                          */
    LCK_MODE_SYSID = 4u   /*!< system identification for kinematic controller */

} eLCKMode_t;

/*! LCK provide port for LCD (dynamic controller) */
typedef struct {
    eLCKLCDCtrlMode_t eLCKLCDCtrlMode; /*!< requested dynamic controller mode */
    float32 fRefStrAng; /*!< requested steering angle [rad]      */
    float32 fStrAng;    /*!< steering angle at front tires [rad] */
    float32 fStrAngVel; /*!< steering angle velocity [rad/s]     */
    float32 fVhclSpeed; /*!< vehicle speed [m/s]                 */
    float32 fDrvTrq;    /*!< driver applied steering torque [Nm] */

} sLCKLCDOutput_t;

/* main LCK input data structure */
typedef struct {
    eLCKOpMode_t eOpMode; /*!< external operation mode                      */
    eLCKMode_t eLCKMode;  /*!< mode of kinematic lateral controller         */
    float32 fVhclSpeed;   /*!< vehicle speed [m/s]                          */
    float32 fCurvature;   /*!< lane curvature [1/m]                         */
    float32 fSteerAng;    /*!< current steering angle at front tires [rad]  */
    float32 fSteerAngVel; /*!< current steering angle velocity [rad/s]      */
    float32 fYawRate;     /*!< yaw rate [rad/s]                             */
    float32 fHeading;     /*!< vehicle's orientation within the lane [rad]  */
    float32 fLatDev;      /*!< lateral deviation from target trajectory [m] */
    float32 fDrvTrq;      /*!< driver applied steering torque [Nm]          */
    float32 fCycleTime_sec;

} sLCKInput_t; /* Input of LKS kinematic lateral controller @vaddr:0x20271200
                  @vaddr_defs: FCT_MEAS_ID_LKS_LCK_INPUT @cycleid:ALDW_VEH
                  @vname:FCTLKSInputLCK */

/*****************************************************************************
 *                                                                           *
 *     LCK parameter                                                         *
 *                                                                           *
 *****************************************************************************/
typedef struct {
    float32 fMass; /*!< vehicle mass [kg]                                  */
    float32 fDistToFrontAxle; /*!< distance between vehicle's CoG and front axle
                                 [m]  */
    float32 fDistToRearAxle;  /*!< distance between vehicle's CoG and rear axle
                                 [m]   */
    float32 fCrnStiffFr;      /*!< cornering stiffness at front tires [N/rad] */
    float32 fCrnStiffRe;      /*!< cornering stiffness at rear tires [N/rad] */
    float32 fSteeringRatio;   /*!< steering ratio [-] */
    float32 fSelfStrGrad;     /*!< self steering gradient [rad/(m/s2) ] */

} sLCKParam_t;

/*****************************************************************************
 *                                                                           *
 *     LCK output data                                                       *
 *                                                                           *
 *****************************************************************************/
typedef struct {
    boolean bRefStrAnglReq; /*!< flag which indicates steering request */
    float32 fRefStrAng; /*!< reference steering angle at front tires [rad] */
    float32
        fRefStrVel; /*!< reference steering velocity at front tires [rad/sec] */

} sLCKOutput_t;

/*****************************************************************************
 *                                                                           *
 *     LCK debug output                                                      *
 *                                                                           *
 *****************************************************************************/
#ifndef Rte_TypeDef_sLCKDebugData_t
typedef struct {
    float32 fLinFadingIn;  /*!< input for linear fading [rad] */
    float32 fLinFadingOut; /*!< output of linear fading [rad] */
    float32 fFFStrAng;  /*!< steering angle feedforward [rad]                 */
    float32 fFFYawRate; /*!< yaw rate feedforward [rad]                       */
    float32 fFeedForward;    /*!< feedforward steering angle [rad] */
    float32 fStateFeedback;  /*!< state feedback steering angle [rad] */
    float32 fStrAngGain;     /*!< steering angle feedback gain factor [-] */
    float32 fStrAngFeedback; /*!< steering angle feedback component [rad] */
    float32
        fStrAngVelGain; /*!< steering angle velocity feedback gain factor [-] */
    float32 fStrAngVelFeedback; /*!< steering angle velocity feedback component
                                   [rad] */
    float32 fYawRateGain;       /*!< yaw rate feedback gain factor [-] */
    float32 fYawRateFeedback;   /*!< yaw rate feedback component [rad] */
    float32 fHeadingGain;     /*!< heading velocity feedback gain factor [-] */
    float32 fHeadingFeedback; /*!< yaw rate feedback component [rad] */
    float32 fLatDevGain;      /*!< lateral deviation feedback gain factor [-] */
    float32 fLatDevFeedback;  /*!< lateral deviation feedback component [rad] */
    float32 fRefStrVel; /*!< input for angle saturation [rad]                 */

} sLCKDebugData_t;
#define Rte_TypeDef_sLCKDebugData_t
#endif

#ifndef Rte_TypeDef_sLCKDebugParam_t
typedef struct {
    float32 fMass; /*!< vehicle mass [kg]                                  */
    float32 fDistToFrontAxle; /*!< distance between vehicle's CoG and front axle
                                 [m]  */
    float32 fDistToRearAxle;  /*!< distance between vehicle's CoG and rear axle
                                 [m]   */
    float32 fCrnStiffFr;      /*!< cornering stiffness at front tires [N/rad] */
    float32 fCrnStiffRe;      /*!< cornering stiffness at rear tires [N/rad] */
    float32 fSteeringRatio;   /*!< steering ratio [-] */
    float32 fSelfStrGrad;     /*!< self steering gradient [rad/(m/s2)] */

    float32 fRefStrAngLimit; /*!< max reference steering angle at tires [rad] */
    float32
        fRefStrVelLimit; /*!< max reference steering angle velocity [rad/s] */

    float32 fVhclSpeedArray[LCK_MAX_SPEED_ARRAY_SIZE]; /*!<  vehicle speed array
                                                          for gain
                                                          scheduling [m/s] */
    float32 fStrAngGains[LCK_MAX_SPEED_ARRAY_SIZE];    /*!< steering angle state
                                                          feedback gains */
    float32 fStrVelGains[LCK_MAX_SPEED_ARRAY_SIZE]; /*!< steering angle velocity
                                                       state feedback gains */
    float32 fYawRateGains[LCK_MAX_SPEED_ARRAY_SIZE]; /*!< yaw rate state
                                                        feedback gains */
    float32 fHeadingGains
        [LCK_MAX_SPEED_ARRAY_SIZE]; /*!< heading state feedback gains */
    float32 fLatDevGains[LCK_MAX_SPEED_ARRAY_SIZE]; /*!< lateral deviation state
                                                       feedback gains */
    float32 fFeedForwardMaster;                     /*!< feed forward master */
    float32 fFFStrAngMaster;  /*!< feed forward master for steering angle */
    float32 fFFYawRateMaster; /*!< feed forward master for yaw rate */
    float32 fLinFadingCoef;   /*!< linear fading coefficient */

} sLCKDebugParam_t;
#define Rte_TypeDef_sLCKDebugParam_t
#endif

#ifndef Rte_TypeDef_sLCKDebug_t
typedef struct {
    sLCKDebugData_t sData;   /*!<  debug data      */
    sLCKDebugParam_t sParam; /*!<  debug parameter */
} sLCKDebug_t;               /* LCK Debug Output @vaddr:0x20272500 @vaddr_defs:
                                FCT_MEAS_ID_LCK_DEBUG @cycleid:ALDW_VEH @vname:FCTLCKDebug */
#define Rte_TypeDef_sLCKDebug_t
#endif

/*****************************************************************************
  GLOBAL CONSTANTS (EXTERNAL SCOPE)
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES (EXTERNAL SCOPE)
*****************************************************************************/
// extern MEMSEC_REF LCKState_t      LCKState;   /* for FCT vehicle frame */
// extern MEMSEC_REF sLCKLCDOutput_t sLCKLCDOutput;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
/* entry functions for external framework */
extern void LCKReset(void);
extern void LCKProcess(const sLCKInput_t* reqPorts,
                       const sLCKParam_t* proParams,
                       sLCKOutput_t* proPorts,
                       sLCKDebug_t* proDebugs);

/*****************************************************************************
  INLINE FUNCTION
*****************************************************************************/

/*** END OF SINLGE INCLUDE SECTION ******************************************/
#ifdef __cplusplus
}
#endif
#endif /* _LCK_EXT_H_INCLUDED */

/** @} end defgroup */
