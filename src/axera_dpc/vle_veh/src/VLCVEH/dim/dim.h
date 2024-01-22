

#ifndef _DIM_H_INCLUDED
#define _DIM_H_INCLUDED

#include "vlcVeh_common_utils.h"
#include "TM_Math_Cal.h"

#include "dim_cfg.h"
#include "dim_ext.h"

#ifdef ALGO_INLINE
#undef ALGO_INLINE
#endif
#define ALGO_INLINE static inline

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief    LC_mode_t

    @general more description here if any,otherwise skip this

    @conseq    [ None ]

    @attention [ None ]

    */
typedef enum { LC_mode_complete = 0, LC_mode_only_turn_lights = 1 } LC_mode_t;

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief eDIMInputSignalType_t

    @general more description here if any,otherwise skip this

    @conseq    [ None ]

    @attention [ None ]

    */
typedef enum {
    eDIMInputSignalType_Float = 0,       /*!<value type is a float*/
    eDIMInputSignalType_SignedInt = 1,   /*!<value type is a signed int*/
    eDIMInputSignalType_UnsignedInt = 2, /*!<value type is a unsigned int*/
    eDIMInputSignalType_Bool = 3,        /*!<value type is a bool*/

    eDIMInputSignalType_Max = 4
} eDIMInputSignalType_t;

/*! @brief      uint8 DIMInputSignalType_t   */
typedef uint8 DIMInputSignalType_t;

/* Define signal input type values */
/*! @brief       DIMInputSignalType_Float   */
#define DIMInputSignalType_Float \
    ((DIMInputSignalType_t)eDIMInputSignalType_Float)
/*! @brief       DIMInputSignalType_SignedInt   */
#define DIMInputSignalType_SignedInt \
    ((DIMInputSignalType_t)eDIMInputSignalType_SignedInt)
/*! @brief       DIMInputSignalType_UnsignedInt   */
#define DIMInputSignalType_UnsignedInt \
    ((DIMInputSignalType_t)eDIMInputSignalType_UnsignedInt)
/*! @brief       DIMInputSignalType_Bool   */
#define DIMInputSignalType_Bool ((DIMInputSignalType_t)eDIMInputSignalType_Bool)
/*! @brief       DIMInputSignalType_Max   */
#define DIMInputSignalType_Max ((DIMInputSignalType_t)eDIMInputSignalType_Max)

/* ****************************************************************
    TYPEDEF UNION
    **************************************************************** */
/*! @brief Union uDIMInputValueUnion_t

    @general typical structure for HEAD input values

    @conseq [ None ]

    @attention [ None ]

    */
typedef union {
    float32 fValue; /*!< fValue*/ /*access with get and set functions*/
    sint32 iValue;                /*!< iValue*/
    uint32 uiValue;               /*!< uiValue*/
    uint32 bValue;                /*!< bValue*/
} uDIMInputValueUnion_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief  Structure DIMInputValue_t

    @general more description here if any,otherwise skip this

    @conseq    [ None ]

    @attention [ None ]

    */
typedef struct {
    uDIMInputValueUnion_t uValue;         /*!< uValue*/
    float32 fStdDev;                      /*!< fStdDev*/
    DIMInputSignalState_t eSignalQuality; /*!< eSignalQuality*/
    DIMInputSignalType_t eSignalType;     /*!< eSignalType*/
    uint8 dummy1;                         /*!< explicit padding */
    uint8 dummy2;                         /*!< explicit padding */
} DIMInputValue_t;

/**
 * This macros reads the value from the union according to the type field. The
 * result value is cast into the target type.
 *
 * The goal of this macros is to retain the flexibility of the input value
 * concept, while getting as much type safety as possible.
 * The macro cannot be replaced by a single function safely, because this would
 * imply that all values are cast to the most
 * generic output type (float) and a possibly result-changing cast to the actual
 * target type. If the macro is replaced by multiple
 * functions, the type-dependend "switching" must be executed at each caller
 * site.
 *
 * @param[in] pInputSignal The input value structure from which the data is
 * read.
 * @param[out] outputValue The output value variable where the data is stored.
 * @param[in] defaultValue The default value of the output value if the input
 * value type is not supported.
 * @param[in] outputValueType The type of the output value variable.
 * @param[in] InvalidSignalCount The type of the output value variable.
 * @param[in] GlobalError The type of the output value variable.
 */

/* InvalidSignalCount and GDBError needs to be initialized before calling the
 * macro */
#define DIMGetInputValueMacro(pInputSignal, outputValue, defaultValue,         \
                              outputValueType, InvalidSignalCount,             \
                              GlobalError)                                     \
    do {                                                                       \
        if ((pInputSignal).eSignalQuality == DIMInputSignalState_OK) {         \
            switch ((pInputSignal).eSignalType) {                              \
                case (DIMInputSignalState_t)DIMInputSignalType_Float:          \
                    (outputValue) =                                            \
                        (outputValueType)((pInputSignal).uValue.fValue);       \
                    break;                                                     \
                case (DIMInputSignalState_t)DIMInputSignalType_SignedInt:      \
                    (outputValue) =                                            \
                        (outputValueType)((pInputSignal).uValue.iValue);       \
                    break;                                                     \
                case (DIMInputSignalState_t)DIMInputSignalType_UnsignedInt:    \
                    (outputValue) =                                            \
                        (outputValueType)((pInputSignal).uValue.uiValue);      \
                    break;                                                     \
                case (DIMInputSignalState_t)DIMInputSignalType_Bool:           \
                    (outputValue) = (outputValueType)(                         \
                        (((pInputSignal).uValue.bValue) > 0u) ? TRUE : FALSE); \
                    break;                                                     \
                default:                                                       \
                    (outputValue) = (outputValueType)(defaultValue);           \
                    (InvalidSignalCount)++;                                    \
                    (GlobalError) = GDB_ERROR_VALUE_RANGE;                     \
                    break;                                                     \
            }                                                                  \
        } else {                                                               \
            (outputValue) = (outputValueType)(defaultValue);                   \
            (InvalidSignalCount)++;                                            \
        }                                                                      \
    } while (0)

/*! @brief       DIMGetInputStdDevMacro
    @general     DIM macro to get Input Standard Deviation
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     Refer Macro
    @unit        SI-unit

       */
#define DIMGetInputStdDevMacro(pInputSignal, outputValue, defaultValue,        \
                               outputValueType, InvalidSignalCount,            \
                               GlobalError)                                    \
    do {                                                                       \
        if ((pInputSignal).eSignalQuality == DIMInputSignalState_OK) {         \
            switch ((pInputSignal).eSignalType) {                              \
                case (DIMInputSignalState_t)DIMInputSignalType_Float:          \
                    (outputValue) = (outputValueType)((pInputSignal).fStdDev); \
                    break;                                                     \
                case (DIMInputSignalState_t)DIMInputSignalType_SignedInt:      \
                case (DIMInputSignalState_t)DIMInputSignalType_UnsignedInt:    \
                case (DIMInputSignalState_t)DIMInputSignalType_Bool:           \
                default:                                                       \
                    (outputValue) = (outputValueType)(defaultValue);           \
                    (InvalidSignalCount)++;                                    \
                    (GlobalError) = GDB_ERROR_VALUE_RANGE;                     \
                    VLC_ASSERT(FALSE);                                         \
                    break;                                                     \
            }                                                                  \
        } else {                                                               \
            (outputValue) = (outputValueType)(defaultValue);                   \
            (InvalidSignalCount)++;                                            \
        }                                                                      \
    } while (0)

/*! @brief       DIMGetInputValue
    @general     DIM macro to get Input value
    @conseq      @incp
                 @decp
    @attention   [None]
    @typical     Refer Macro
    @unit        SI-unit

       */
#define DIMGetInputValue(inputValue, outputValue, defaultValue,                \
                         outputValueType)                                      \
    do {                                                                       \
        switch ((inputValue).eSignalType) {                                    \
            case DIMInputSignalType_Float:                                     \
                (outputValue) = (outputValueType)((inputValue).uValue.fValue); \
                break;                                                         \
            case DIMInputSignalType_SignedInt:                                 \
                (outputValue) = (outputValueType)((inputValue).uValue.iValue); \
                break;                                                         \
            case DIMInputSignalType_UnsignedInt:                               \
                (outputValue) =                                                \
                    (outputValueType)((inputValue).uValue.uiValue);            \
                break;                                                         \
            case DIMInputSignalType_Bool:                                      \
                (outputValue) = (outputValueType)(                             \
                    (((inputValue).uValue.bValue) > 0u) ? TRUE : FALSE);       \
                break;                                                         \
            default:                                                           \
                (outputValue) = (outputValueType)(defaultValue);               \
                break;                                                         \
        }                                                                      \
    } while (0)

/*! @brief       DIMGetInputStdDev(inputValue)   */
#define DIMGetInputStdDev(inputValue) ((inputValue).fStdDev)
/*! @brief      DIMGetInputQuality(inputValue)    */
#define DIMGetInputQuality(inputValue) ((inputValue).eSignalQuality)
/*! @brief       DIMIsInputOK(inputValue)   */
#define DIMIsInputOK(inputValue) \
    ((((inputValue).eSignalQuality == DIMInputSignalState_OK)) ? TRUE : FALSE)
/*! @brief       BITMASK_UINT8   */
#define BITMASK_UINT8 (0xFFu)
/*! @brief       CLEAR_BIT_UINT8   */
#define CLEAR_BIT_UINT8(source, bitmask) \
    ((source) &= (uint8)((~(bitmask)) & BITMASK_UINT8))

/* ****************************************************************
    TYPEDEF STRUCT DIMInputDataGlobal_t
    **************************************************************** */
/*! @brief  Structure DIMInputDataGlobal_t

    @general more description here if any,otherwise skip this

    @conseq    [ None ]

    @attention [ None ]

    */
typedef struct {
    DIMInputValue_t
        VehicleVelocity; /*!< longitudinal ego vehicle speed (m/s) */
    DIMInputValue_t
        VehicleAcceleration;       /*!< longitudinal ego vehicle accel (m/s^2)*/
    DIMInputValue_t DriverBraking; /*!< driver is braking (bool)*/
    DIMInputValue_t TurnIndicator; /*!< defines the turnlight activity status
                                      (0: off, 1: left, 2:right, 3:both)*/
    DIMInputValue_t GasPedalPosition; /*!< position of the gas pedal (%)*/
    DIMInputValue_t GasPedalGradient; /*!< GasPedalGradient (%/s)*/
    DIMInputValue_t
        SteeringWheelAngle; /*!< Steering wheel angle (?->left: pos, right:
                               neg)*/
    DIMInputValue_t SteeringWheelGradient; /*!< Steering wheel angle gradient
                                              (?s  ->left: pos, right: neg) */
    DIMInputValue_t SpeedLimiter;          /*!< SpeedLimiter */
} DIMInputDataGlobal_t;

// #if (VLC_CFG_HYPOTHESIS_SPORTSTYLE)
/* ****************************************************************
    TYPEDEF STRUCT DIMInputDataSportStyle_t
    **************************************************************** */
/*! @brief Structure DIMInputDataSportStyle_t

    @general more description here if any,otherwise skip this

    @conseq [ None ]

    @attention [ None ]

    */
typedef struct {
    DIMInputValue_t
        SPSRequestedBrakeTorque;        /*!< Sports Style RequestedBrakeTorque*/
    DIMInputValue_t SPSKickdown;        /*!< Sports Style Kickdown*/
    DIMInputValue_t SPSGearShiftActive; /*!< Sports Style GearShiftActive*/
    /* Camera data */
    DIMInputValue_t
        SPSLateralDisplacement; /*!< Sports Style LateralDisplacement*/
} DIMInputDataSportStyle_t;
// #endif /* VLC_CFG_HYPOTHESIS_SPORTSTYLE */

// #if (VLC_CFG_HYPOTHESIS_LANECHANGE)
/* ****************************************************************
    TYPEDEF STRUCT DIMInputDataLaneChange_t
    **************************************************************** */
/*! @brief Structure DIMInputDataLaneChange_t

    @general more description here if any,otherwise skip this

    @conseq [ None ]

    @attention [ None ]

    */
typedef struct {
    DIMInputValue_t LCOverallCurvature;  /*!< Lane Change overallCurvature*/
    DIMInputValue_t LCCurvature;         /*!< Lane Change Curvature*/
    DIMInputValue_t LCPsi;               /*!< Lane Change Psi*/
    DIMInputValue_t LCLateralVelocity;   /*!< Lane Change LateralVelocity*/
    DIMInputValue_t LCTurnLightOnlyMode; /*!< Lane Change TurnLightOnlyMode*/
} DIMInputDataLaneChange_t;
// #endif

// #if (VLC_CFG_HYPOTHESIS_DISTRACTION)
/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure DIMInputDataDistraction_t

    @general more description here if any,otherwise skip this

    @conseq [ None ]

    @attention [ None ]

    */
typedef struct {
    DIMInputValue_t RadioButtons;  /*!< Radio Buttons pressed (bool)*/
    DIMInputValue_t SeatButtons;   /*!< Seat Buttons pressed (bool)*/
    DIMInputValue_t WindowButtons; /*!< Window Buttons pressed (bool)*/
    DIMInputValue_t MirrorButtons; /*!< Mirror Buttons pressed (bool)*/
    DIMInputValue_t
        InteriorLightsButtons; /*!< InteriorLights Buttons pressed (bool)*/
    DIMInputValue_t
        ExteriorLightsButtons;      /*!< ExteriorLights Buttons pressed (bool)*/
    DIMInputValue_t SunRoofButtons; /*!< SunRoof pressed (bool)*/
    DIMInputValue_t SteeringWheelButtons; /*!< SteeringWheel pressed (bool)*/
    DIMInputValue_t DriverTired;          /*!< DriverTired (bool)*/
    DIMInputValue_t StrColumnButtons;  /*!< StrColumn Buttons pressed (bool)*/
    DIMInputValue_t MidConsoleButtons; /*!< MidConsole Buttons pressed (bool)*/
    DIMInputValue_t OtherButtons;      /*!< Other Buttons pressed (bool)*/
} DIMInputDataDistraction_t;
// #endif /* VLC_CFG_HYPOTHESIS_DISTRACTION */

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure DIMInputData_t

    @general more description here if any,otherwise skip this

    @conseq [ None ]

    @attention [ None ]

    */
typedef struct { DIMInputDataGlobal_t Global; /*!< Global */ } DIMInputData_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure DIMMTSHeaderData_t

    @general more description here if any,otherwise skip this

    @conseq    [ None ]

    @attention [ None ]

    */
typedef struct {
    uint32 uiBaseVersion; /*!< uiBaseVersion*/
    uint32 uiStructSize;  /*!< @unit:bytes*/
} DIMMTSHeaderData_t;

extern void DIMRunSigPreProc(float32 fCycleTime,
                             DIMInputDataGlobal_t *pDimInput);
extern void DIMInitSigPreProc(void);

ALGO_INLINE void DIMSetInputValueFloat(DIMInputValue_t *inputValueStruct,
                                       float32 FloatValue,
                                       DIMInputSignalState_t SignalQuality,
                                       float32 StdDev);
ALGO_INLINE void DIMSetInputValueUInt(DIMInputValue_t *inputValueStruct,
                                      uint32 UIntValue,
                                      DIMInputSignalState_t SignalQuality);
ALGO_INLINE void DIMSetInputValueSInt(DIMInputValue_t *inputValueStruct,
                                      sint32 SIntValue,
                                      DIMInputSignalState_t SignalQuality);
ALGO_INLINE void DIMSetInputValueBool(DIMInputValue_t *inputValueStruct,
                                      boolean BoolValue,
                                      DIMInputSignalState_t SignalQuality);
ALGO_INLINE void DIMSetInputValueBoolTest(DIMInputValue_t *inputValueStruct,
                                          boolean BoolValue,
                                          DIMInputSignalState_t SignalQuality);

/*************************************************************************************************************************
  Functionname:    DIMSetInputValueFloat                                                                            */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE void

      @param[in,out]   *inputValueStruct :
      @param[in]       FloatValue :
      @param[in]       SignalQuality :
      @param[in]       StdDev :
    *************************************************************************************************************************/
ALGO_INLINE void DIMSetInputValueFloat(DIMInputValue_t *inputValueStruct,
                                       float32 FloatValue,
                                       DIMInputSignalState_t SignalQuality,
                                       float32 StdDev) {
    inputValueStruct->eSignalType = DIMInputSignalType_Float;
    inputValueStruct->uValue.fValue = FloatValue;
    inputValueStruct->eSignalQuality = SignalQuality;
    inputValueStruct->fStdDev = StdDev;
}

/*************************************************************************************************************************
  Functionname:    DIMSetInputValueUInt                                                                             */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE void

      @param[in,out]   *inputValueStruct :
      @param[in]       UIntValue :
      @param[in]       SignalQuality :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE void DIMSetInputValueUInt(DIMInputValue_t *inputValueStruct,
                                      uint32 UIntValue,
                                      DIMInputSignalState_t SignalQuality) {
    inputValueStruct->eSignalType = DIMInputSignalType_UnsignedInt;
    inputValueStruct->uValue.uiValue = UIntValue;
    inputValueStruct->eSignalQuality = SignalQuality;
    inputValueStruct->fStdDev = 0.0f;
}

/*************************************************************************************************************************
  Functionname:    DIMSetInputValueSInt                                                                             */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE void

      @param[in,out]   *inputValueStruct :
      @param[in]       SIntValue :
      @param[in]       SignalQuality :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE void DIMSetInputValueSInt(DIMInputValue_t *inputValueStruct,
                                      sint32 SIntValue,
                                      DIMInputSignalState_t SignalQuality) {
    inputValueStruct->eSignalType = DIMInputSignalType_SignedInt;
    inputValueStruct->uValue.iValue = SIntValue;
    inputValueStruct->eSignalQuality = SignalQuality;
    inputValueStruct->fStdDev = 0.0f;
}

/*************************************************************************************************************************
  Functionname:    DIMSetInputValueBool                                                                             */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE void

      @param[in,out]   *inputValueStruct :
      @param[in]       BoolValue :
      @param[in]       SignalQuality :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE void DIMSetInputValueBool(DIMInputValue_t *inputValueStruct,
                                      boolean BoolValue,
                                      DIMInputSignalState_t SignalQuality) {
    inputValueStruct->eSignalType = DIMInputSignalType_Bool;
    inputValueStruct->uValue.bValue = BoolValue;
    inputValueStruct->eSignalQuality = SignalQuality;
    inputValueStruct->fStdDev = 0.0f;
}

/*************************************************************************************************************************
  Functionname:    DIMSetInputValueBoolTest                                                                         */ /*!

      @brief           Functional Summary

      @description     Detailed Design

      @return          ALGO_INLINE void

      @param[in,out]   *inputValueStruct :
      @param[in]       BoolValue :
      @param[in]       SignalQuality :

      @pre             [ None ]
      @post            [ None ]
    *************************************************************************************************************************/
ALGO_INLINE void DIMSetInputValueBoolTest(DIMInputValue_t *inputValueStruct,
                                          boolean BoolValue,
                                          DIMInputSignalState_t SignalQuality) {
    inputValueStruct->eSignalType = DIMInputSignalType_Bool;
    inputValueStruct->uValue.bValue = (BoolValue != FALSE) ? 1u : 0u;
    inputValueStruct->eSignalQuality = SignalQuality;
    inputValueStruct->fStdDev = 0.0f;
}

extern void DIMFillCustomOut(const DIMInputDataGlobal_t *const pInputData);

#endif /* _DIM_H_INCLUDED */
