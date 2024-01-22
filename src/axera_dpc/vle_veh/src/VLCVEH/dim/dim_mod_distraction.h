

#ifndef _DIM_MOD_DRV_DIS_INCLUDED
#define _DIM_MOD_DRV_DIS_INCLUDED

#include "dim.h"
#include "switch_ext.h"
#include "dim_mod_distraction_par.h"

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Structure to store switch(button) parameters

    @general Structure to store switch(button) parameters used in Distraction
   hypothesis of DIM module

    @conseq [None]

    @attention [None]

    */
typedef struct {
    switch_t Switch;                  /*!< information about Switch*/
    uint16 Switch_distraction_cycles; /*!< Switch distraction cycle count*/
    uint16 Switch_confidence_cycles;  /*!< Switch confidence cycle count*/
    uint16 ConfidenceCounter;         /*!< save confidence counter value*/
} button_switch_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief DIM module Distraction Hypothesis dynamic parameters structure

    @general Structure to store dynamic parameters of DIM module distraction
   hypothesis

    @conseq [None]

    @attention [None]

    */
typedef struct {
    button_switch_t RadioB;  /*!< information about Radio Buttons*/
    button_switch_t SeatB;   /*!< information about Seat Buttons*/
    button_switch_t WindowB; /*!< information about Radio Switch*/
    button_switch_t MirrorB; /*!< information about Mirror Buttons*/
    button_switch_t
        InteriorLightsB; /*!< information about Interior Lights Buttons*/
    button_switch_t
        ExteriorLightsB;      /*!< information about Exterior Lights Buttons*/
    button_switch_t SunRoofB; /*!< information about Roof Buttons*/
    button_switch_t
        SteeringWheelB;          /*!< information about Steering Wheel Buttons*/
    button_switch_t TiredB;      /*!< information about Tired Buttons*/
    button_switch_t StrColumnB;  /*!< information about Str Column Buttons*/
    button_switch_t MidConsoleB; /*!< information about Mid Console Buttons*/
    button_switch_t OtherB;      /*!< information about Other Buttons*/
    float32 CycleTimeMin;        /*!< minimum limit for cycle time*/
    float32 CycleTimeMax;        /*!< maximum limit for cycle time*/
} DIMDistractionDynPar_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief DIM module Distraction Hypothesis internal data structure

    @general Structure to store internal data of DIM module distraction
   hypothesis

    @conseq [None]

    @attention [None]

    */
typedef struct {
    sint16 iProbability;            /*!< save hypothesis probability @unit:%*/
    sint16 iConfidence;             /*!< save hypothesis confidence @unit:%*/
    boolean DistractionTimeOK;      /*!< Status of evaluation Result */
    DIMDistractionDynPar_t *DynPar; /*!< @unit:pointer */
} DIMInternalDataModDistraction_t;

/*!  @cond Doxygen_Suppress */
extern eGDBError_t DIMInitModuleDistraction(
    const float32 fCycleTime,
    DIMInternalDataModDistraction_t *pInternalData,
    const DIM_DISTRACTION_PAR_struct_t *pDimDistructionParamData);
extern eGDBError_t DIMRunModuleDistraction(
    const float32 fCycleTime,
    const DIMInputDataDistraction_t *pInputData,
    GDB_DMHypothesis_t *pOutHypothesis,
    DIMInternalDataModDistraction_t *pInternalData,
    const DIM_DISTRACTION_PAR_struct_t *pDimDistructionParamData);
/*! @endcond */
#endif /*_DIM_MOD_DRV_DIS_INCLUDED*/
