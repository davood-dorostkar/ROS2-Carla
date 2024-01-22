

#ifndef _DIM_ALDW_H_INCLUDED
#define _DIM_ALDW_H_INCLUDED

#include "dim_mod_sportstyle.h"
#include "dim_mod_sportstyle_par_aldw.h"

#include "dim_mod_lanechange.h"
#include "dim_mod_lanechange_par_aldw.h"

#include "dim_mod_distraction.h"
#include "dim_mod_distraction_par_aldw.h"

/*! @brief DIM_CFG_ALDW_MODEL_LIGHT */
#define DIM_CFG_ALDW_MODEL_LIGHT SWITCH_OFF
/* ****************************************************************
    TYPEDEF enum
    **************************************************************** */
/*! @brief DimALDWHypothesisIdx_t.

    @general DimALDWHypothesisIdx_t

    */
typedef enum {
    DIM_ALDW_HYP_IDX_SPORTSTYLE = 0u,
    DIM_ALDW_HYP_IDX_DISTRACTION = 1u,
    DIM_ALDW_HYP_IDX_LANECHANGE = 2u
} DimALDWHypothesisIdx_t;

/* ****************************************************************
    TYPEDEF STRUCT
    **************************************************************** */
/*! @brief DIMInteralDataALDW_t.

    @general DIMInteralDataALDW_t

    */
typedef struct {
    DIMInternalDataModDistraction_t
        Internal_Distraction; /*!< Internal_Distraction*/
    DIMInternalDataModSportStyle_t
        Internal_SportStyle; /*!< Internal_SportStyle*/
    DIMInternalDataModLaneChange_t
        Internal_LaneChange; /*!< Internal_LaneChange*/
} DIMInteralDataALDW_t;

#endif
