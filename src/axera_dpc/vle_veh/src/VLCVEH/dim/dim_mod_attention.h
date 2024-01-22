

#ifndef _DIM_MOD_DRV_ATT_INCLUDED
#define _DIM_MOD_DRV_ATT_INCLUDED
#include "dim.h"
#include "dim_mod_attention_par.h"

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief     Structure for DIMInternalDataModAttention_t

    @general

    @conseq    [ None ]

    @attention [ None ]

    */
typedef struct {
    sint16 iProbability;       /*!< save hypothesis probability @unit:%*/
    sint16 iConfidence;        /*!< save hypothesis confidence @unit:%*/
    float32 fVeryHighKeepTime; /*!< current timer for very high attention time
                                  @unit:s*/
    float32 fHigherKeepTime;   /*!< current timer for very high attention time
                                  @unit:s*/
    float32 fHighKeepTime; /*!< current timer for high attention time @unit:s*/
    float32 fLowKeepTime;  /*!< current timer for low attention time @unit:s*/
    float32 fConstVelTime; /*!< current time of const velocity @unit:s*/
    float32 fNoGasPedalGradTime; /*!< current timer of no gas pedal gradients
                                    time @unit:s*/
    float32 fConstVelocity;      /*!< current stored velocity @unit:m/s*/
} DIMInternalDataModAttention_t;

extern eGDBError_t DIMInitModuleAttention(
    DIMInternalDataModAttention_t *pInternalData);
extern eGDBError_t DIMRunModuleAttention(
    const float32 fCycleTime,
    const DIMInputDataGlobal_t *pInputData,
    GDB_DMHypothesis_t *pOutHypothesis,
    DIMInternalDataModAttention_t *pInternalData,
    const DIM_ATTENTION_PAR_struct_t *pDIM_attention_par);

#endif /*_DIM_MOD_DRV_ATT_INCLUDED*/
