#ifndef ODPR_FOP_H
#define ODPR_FOP_H

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "odpr_ext.h"

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/
typedef struct {
    boolean bObjHysteresisAllowed_bool : 1;   // Flag that object hysteresis is
                                              // allowed
    boolean bNoObjIDSwitchDetected_bool : 1;  // Flag that no object ID switch
                                              // is detected
    boolean
        bObjPosYStepDetected_bool : 1;  // Flag that position y step is detected
    boolean
        bObjPosXStepDetected_bool : 1;  // Flag that position x step is detected
    boolean bResetObjValidity_bool : 1;  // Flag that reset object validity
} FOPChkPossiHys_t;

typedef struct {
    boolean bACCObjDetected_bool : 1;     // Flag that ACC object is detected
    boolean bObjQualityValid_bool : 1;    // Flag that object quality is valid
    boolean bObjMeasStateValid_bool : 1;  // Flag that object measurement state
                                          // is valid
    boolean bObjPosYValid_bool : 1;     // Flag that object position Y is valid
    boolean bObjPosXValid_bool : 1;     // Flag that object position X is valid
    boolean bObjWidthValid_bool : 1;    // Flag that object width is valid
    boolean bObjRelVelXValid_bool : 1;  // Flag that X-axis relative velocity is
                                        // valid
    boolean bObjRelVelYValid_bool : 1;  // Flag that Y-axis relative velocity is
                                        // valid
    boolean bObjRelAclXValid_bool : 1;  // Flag that X-axis relative
                                        // acceleration is valid
    boolean bObjTypeValid_bool : 1;     // Flag that object type is valid
    boolean bObjFusionStateValid_bool : 1;  // Flag that object fusion state is
                                            // valid
    boolean bObjLatMovInvalid_bool : 1;     // Flag that object latent moving is
                                            // invalid
} FOPValidACCObj_t;

typedef enum { ACC_OC_CAR = 1, ACC_OC_TRUCK, ACC_OC_PEDESTRIAN } FOPObjType;

typedef enum {
    STATE_DELETED,
    STATE_NEW,
    STATE_MEASURED,
    STATE_PREDICTED,
    STATE_INACTIVE,
    STATE_MAX_DIFF_TYPES
} FOPObjMeasState;

typedef struct {
    const ODPRInAccFRObj_t* pAccObject;
    const ODPRInSystemPara_t* pSystemPara;
    const ODPRInVEDVehDyn_t* pEgoVehSig;
} FOPInReq_t;

// typedef struct
//{
//    ODPRFOPParam_t* pFOPParam;
//} FOPParam_t;

typedef struct { ODPRFOPOut_t* pFOPOutData; } FOPOutPro_t;

typedef struct { ODPRFOPDebug_t* pFOPDebug; } FOPDebug_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/
STATIc void FOPTimeConversion(const ODPRInAccFRObj_t* pACCObjectData,
                              const ODPRInSystemPara_t* pSystemPara,
                              ODPRFOPOut_t* pFOPOutput);
STATIc void FOPTransObjXPos(const ODPRInAccFRObj_t* pACCObjectData,
                            const ODPRParam_t* pParam,
                            ODPRFOPOut_t* pFOPOutput);
STATIc FOPChkPossiHys_t
FOPChkPossHysteresis(const ODPRInAccFRObj_t* pACCObjectData,
                     const ODPRInSystemPara_t* pSystemPara,
                     const ODPRFOPOut_t* pFOPOutput,
                     ODPRFOPDebug_t* pFOPDebug);
STATIc FOPValidACCObj_t
FOPValidateACCObject(const ODPRInAccFRObj_t* pACCObjectData,
                     const FOPChkPossiHys_t* pChkPossiHys,
                     const ODPRInSystemPara_t* pSystemPara,
                     const ODPRInVEDVehDyn_t* pEgoVEDData,
                     const ODPRParam_t* pParam,
                     ODPRFOPOut_t* pFOPOutput,
                     ODPRFOPDebug_t* pFOPDebug);
STATIc void FOPValidACCObjPostProcess(const FOPValidACCObj_t* pValidACCObj,
                                      const FOPChkPossiHys_t* pChkPossiHys,
                                      ODPRFOPOut_t* pFOPOutput,
                                      ODPRFOPDebug_t* pFOPDebug);
STATIc void FOPFreezeLateralKinematic(const ODPRInAccFRObj_t* pACCObjectData,
                                      const FOPValidACCObj_t* pValidACCObj,
                                      ODPRFOPOut_t* pFOPOutput);
STATIc void FOPPositionEstimation(ODPRFOPOut_t* pFOPOutput);
STATIc void FOPObjYawAngEstimation(const ODPRInAccFRObj_t* pACCObjectData,
                                   const ODPRInSystemPara_t* pSystemPara,
                                   const ODPRInVEDVehDyn_t* pEgoVEDData,
                                   const FOPChkPossiHys_t* pChkPossiHys,
                                   ODPRFOPOut_t* pFOPOutput,
                                   ODPRFOPDebug_t* pFOPDebug);

extern boolean BASICTurnOnDelay(const boolean bInput,
                                const float32 fDelayTime,
                                const float32 fCycleTime,
                                float32* fGlobalTime);
extern boolean BASICTurnOffDelay(const boolean bInput,
                                 const float32 fDelayTime,
                                 const float32 fCycleTime,
                                 float32* fGlobalTime);
extern float32 BASICDivDflt(const float32 fNumerator,
                            const float32 fDenominator,
                            const float32 fDflt);
#endif