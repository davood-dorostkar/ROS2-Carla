

#ifndef _DIM_MOD_DRV_SPST_INCLUDED
#define _DIM_MOD_DRV_SPST_INCLUDED
#include "dim.h"
#include "dim_mod_sportstyle_par.h"

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief Data module for sportstyle

    @general Data module requires all data related to sportstyle clubbed to
   gather in this structure

    @conseq [none]

    @attention [none]

    */
typedef struct {
    sint16
        iProbability;   /*!< save hypothesis probability -> factor 256 @unit:%
                           @resolution:1.0/256.0*/
    sint16 iConfidence; /*!< save hypothesis confidence  -> factor 256 @unit:%
                           @resolution:1.0/256.0*/
    sint16 iProbabilityLRW;        /*!< @unit:% @resolution:1.0/256.0*/
    sint16 iProbabilityGRAD;       /*!< @unit:% @resolution:1.0/256.0*/
    sint16 iProbabilityPEDAL;      /*!< @unit:% @resolution:1.0/256.0*/
    sint16 iProbabilityGearShift;  /*!< @unit:% @resolution:1.0/256.0*/
    sint16 iProbabilityLaneChange; /*!< @unit:% @resolution:1.0/256.0*/
    float32 fLateralDeviation;     /*!< @unit:m */
    sint16 iDIMPABrakePedal;       /*!< @unit:% @resolution:1.0/256.0*/
    sint16 iDIMPAKickDown;         /*!< @unit:% @resolution:1.0/256.0*/
    sint16 iDIMPAGasPedal;         /*!< @unit:% @resolution:1.0/256.0*/
    sint16 iDIMPAGasPedalAcc;      /*!< @unit:% @resolution:1.0/256.0*/
    const DIMSportStyleParameter_t *Parameter; /*!< @unit:pointer */
    DIMSportStyleDynPar_t *DynPar;             /*!< @unit:pointer */
} DIMInternalDataModSportStyle_t;

/*! @brief       Sportstyle scale
    @general     [none]
    @conseq      [none]
    @attention   [none]
    @typical     [none]     @unit [none]    @min 0    @max 256   */
#define DIM_SPORTSTYLE_DEF_SCALE (256)

extern eGDBError_t DIMInitModuleSportStyleDynPar(
    const float32 fCycleTime,
    const DIMSportStyleParameter_t *pParameter,
    DIMSportStyleDynPar_t *pDynPar);
extern eGDBError_t DIMInitModuleSportStyle(
    DIMInternalDataModSportStyle_t *pInternalData);
extern eGDBError_t DIMRunModuleSportStyle(
    const DIMInputData_t *pInputData,
    GDB_DMHypothesis_t *pOutHypothesis,
    DIMInternalDataModSportStyle_t *pInternalData);

#endif /*_DIM_MOD_DRV_SPST_INCLUDED*/
