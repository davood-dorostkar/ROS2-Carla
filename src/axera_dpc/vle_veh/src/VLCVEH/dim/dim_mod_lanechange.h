

#ifndef _DIM_MOD_DRV_LACH_INCLUDED
#define _DIM_MOD_DRV_LACH_INCLUDED

#ifdef _DIM_MOD_DRV_SI_LACH_INCLUDED
#error \
    "Do not Include Langechange and SI-Lanchange in the same scope without updating either function and struct names"
#endif

#include "dim.h"
#include "dim_mod_lanechange_par.h"

/*! @brief       Turnlight evaluation : Lane Change Start */
#define LC_IN_START (1u)
/*! @brief       Turnlight evaluation : Lane Change Active */
#define LC_IN_ACTIVE (2u)
/*! @brief       Turnlight evaluation : Lane Change Not Active */
#define LC_IN_NOT_ACTIVE (3u)
/*! @brief       Turnlight evaluation : Lane Change Trigger ON */
#define LC_IN_TRIGGER_ON (4u)
/*! @brief       Turnlight evaluation : Lane Change Trigger OFF */
#define LC_IN_TRIGGER_OFF (5u)

/* ****************************************************************
    TYPEDEF STRUCT TurnLightStates_t
   **************************************************************** */
/*! @brief Structure to store status of the turn indicator is stored.

    @general This structure has a timer, to set it back to inactive after a
   default time -> in case of a critical run up situation, we want to avoid a
   warning/ braking,
             if the ego-driver will change the lane. Switching the turn
   indicator on, shows us, that this will happen in the next seconds.
             If the turn indicator stays on longer, this message is not longer
   valid (accidentialy switched on, stayed on ...)

    @conseq [None]

    @attention [None]
    */
typedef struct {
    uint8 is_active_start; /*!< Turn Light State Active Start*/
    uint8 is;              /*!< Turn Light State */
    float32 fTime;         /*!< Active/Inactive timer*/
} TurnLightStates_t;

/* ****************************************************************
    TYPEDEF STRUCT DIMLaneChangeDynPar_t
   **************************************************************** */
/*! @brief Turn Light Indicator Information

    @general Here all informations about the turn indicators are collected.

    @conseq [None]

    @attention [None]

    */
typedef struct {
    /* Dynamic values */
    float32 fOCurvaturePrevious;          /*!< fOCurvaturePrevious */
    float32 fSWCurvaturePrevious;         /*!< fSWCurvaturePrevious */
    sint8 iTLLeftPrevious;                /*!< iTLLeftPrevious */
    sint8 iTLRightPrevious;               /*!< iTLRightPrevious */
    TurnLightStates_t LeftStates;         /*!< LeftStates */
    TurnLightStates_t RightStates;        /*!< RightStates */
    TurnLightStates_t HistoryLeftStates;  /*!< HistoryLeftStates */
    TurnLightStates_t HistoryRightStates; /*!< HistoryRightStates */

    /* Transformed parameter to cycle interval */
    float32 fCycleTime; /*!< fCycleTime */

    /* PT1 Element transformations */
    float32 fOCurvatureX; /*!< fOCurvatureX */
    BML_t_Vector2D DIM_LANECHANGE_PAR_CrvStdDevZPoleTableX
        [DIM_LANECHANGE_PAR_CrvStdDevZPoleTable_POINTS];

} DIMLaneChangeDynPar_t;

/* ****************************************************************
    TYPEDEF STRUCT DIMInternalDataModLaneChange_t
    **************************************************************** */
/*! @brief Lane Change Information.

    @general DIM Internal Data with Lane Change Information

    @conseq [None]

    @attention [None]

    */
typedef struct {
    sint16 iProbability;                 /*!< save hypothesis probability */
    sint16 iConfidence;                  /*!<  save hypothesis confidence*/
    uint16 uiProbabilityLaneChangeLeft;  /*!<  uiProbabilityLaneChangeLeft */
    uint16 uiProbabilityLaneChangeRight; /*!< uiProbabilityLaneChangeRight */
    const DIMLaneChangeParameter_t *Parameter; /*!< DIMLaneChangeParameter_t */
    DIMLaneChangeDynPar_t *DynPar;             /*!< DIMLaneChangeDynPar_t */
    uint8 DIMLpLong;                           /*!< DIMLpLong */
    uint8 DIMLpLat[2];                         /*!< DIMLpLat */
    uint8 DIMLpVDY[2];                         /*!< DIMLpVDY */
    uint8 DIMLCamCourse[2];                    /*!< DIMLCamCourse */
    uint8 DIMLCamAngle[2];                     /*!< DIMLCamAngle */
    uint8 DIMLCamVLat[2];                      /*!< DIMLCamVLat */
    uint8 DIMLCam[2];                          /*!< DIMLCam */
    uint8 DIMLpCurrent[2];                     /*!< DIMLpCurrent */
    uint8 DIMLpTurnLight[2];                   /*!< DIMLpTurnLight */
    uint8 DIMLpHist[2];                        /*!< DIMLpHist */
} DIMInternalDataModLaneChange_t;

/*!  @cond Doxygen_Suppress */
extern eGDBError_t DIMInitModuleLaneChangeDynPar(
    const float32 fCycleTime,
    const DIMLaneChangeParameter_t *pParameter,
    DIMLaneChangeDynPar_t *pDynPar);
extern eGDBError_t DIMInitModuleLaneChange(
    DIMInternalDataModLaneChange_t *pInternalData);
extern eGDBError_t DIMRunModuleLaneChange(
    const DIMInputData_t *pInputData,
    GDB_DMHypothesis_t *pOutHypothesis,
    DIMInternalDataModLaneChange_t *pInternalData);
/*! @endcond */
#endif /*_DIM_MOD_DRV_LACH_INCLUDED*/
