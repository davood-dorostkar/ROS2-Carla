

#ifndef _DIM_MOD_DRV_ACT_INCLUDED
#define _DIM_MOD_DRV_ACT_INCLUDED
#include "dim.h"
#include "dim_mod_activity_par.h"

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief DIM Activity hypothesis structure

    @general Structure to save DIM module activity hypothesis parameters

    @conseq [None]

    @attention [None]

    */
typedef struct {
    sint16 iProbability;    /*!< save hypothesis probability @unit:%*/
    sint16 iConfidence;     /*!< save hypothesis confidence @unit:%*/
    float32 fActivityTimer; /*!< current timer for activity time  @unit:s*/
    float32
        fActivityHoldTimer; /*!< current timer for activity hold time  @unit:s*/
    float32 fGradPeakTimer; /*!< current timer for Steering Wheel Gradien Peak
                               @unit:s*/
    float32
        fEmergencySteerTimer;   /*!< current timer for emergency steering time
                                   @unit:s*/
    float32 fGradLowPassTimer;  /*!< current timer for Steering Wheel Gradient
                                   Filter  @unit:s*/
    float32 fGradLowPassOutput; /*!< current output of the Steering Wheel
                                   Gradient Filter @unit:?s*/
} DIMInternalDataModActivity_t;

extern eGDBError_t DIMInitModuleActivity(
    DIMInternalDataModActivity_t *pInternalData);
extern eGDBError_t DIMRunModuleActivity(
    const float32 fCycleTime,
    const DIMInputDataGlobal_t *pInputData,
    GDB_DMHypothesis_t *pOutHypothesis,
    DIMInternalDataModActivity_t *pInternalData,
    const DIM_ACTIVITY_PAR_struct_t *pDIM_activity_par);

#endif /*_DIM_MOD_DRV_ACT_INCLUDED*/
