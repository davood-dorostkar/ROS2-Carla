/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*---

**************************************************************************** */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cp_cd.h"
#include "cp_cd_par.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  CONSTANTS
*****************************************************************************/

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief      Curve filter parameters
    @general    Filter parameters for driven path trajectory used for lane
   association
    @attention  Basic dynamic can be adjusted with inv_dist_settled: lower value
   makes filter more dynamic
                Maneuver detection is tunable by cum_sum_min-max and
   gain_min-max. Lower threshold and higher gain
                increase filter dynamic. Gain:=0 no update with new value,
   Gain:=1 output = input
    @todo Old code had the following virtual information, decide if still needed
   (freeze no longer present)
          currently only modified tag to be ignored by PDO
    @_VADDR:VLC_MEAS_ID_CGEB_CD_WRAP_CD_CRV_PAR _VNAME:CDCrvFiltPar @cycleid:
   VLC_ENV */
const CPCDCurveFilterPar_t CPCDCurveFilterPar = {
    0.2F,  /* cum_sum_min       Lin-Gain-Ramp In:  min value of cumulated sum */
    0.4F,  /* cum_sum_max       Lin-Gain-Ramp In:  max value of cumulated sum */
    0.02F, /* gain_min          Lin-Gain-Ramp Out: min gain at min cumulated sum
              */
    1.0F,  /* gain_max          Lin-Gain-Ramp Out: max gain at max cumulated sum
              */
    (3.0F / 20.0F), /* inv_dist_settled  Inverse distance where filter is
                       settled after a step input */
    1.0F, /* min velo          Minimum velocity used for filter gain calculation
             */
    0.8F, /* cum_sum_limit     Upper bound value for cumsum calculation */
    0.01F /* cum_sum_drift     Aging (forgetting) of cumsum value */
};

/* CPRESTRUCT: Constant moved from function head. this constant is just used
 * once in CPUpdateObjDist2Traj */
/*!  @cond Doxygen_Suppress */
const float32 fCPCDMaxAccelDist2Traj = 5.0f;
/*! @endcond */

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS
*****************************************************************************/
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */