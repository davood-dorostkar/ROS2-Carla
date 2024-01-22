/*---

**************************************************************************** */

#ifndef _CP_CD_PARAMETER_INCLUDED
#define _CP_CD_PARAMETER_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "cd_par.h"
#include "cp_cd_curvefilter.h"

/*****************************************************************************
  SYMBOLIC CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*! @brief       Activates input filtering of ego trajectory
    @general     Use for CP CS ego trajectory filter activation.
    @conseq      [none]

    @attention   [none]
    @typical     [none]   @unit [none]     @min 0   @max 1   */
#define CPCD_FILTER_EGO_TRAJECTORY 1

/*! @brief       Use slip angle in CP_CourseData_t
    @general     course data assignment with slip angle
    @conseq      [none]

    @attention   [none]
    @typical     [none]   @unit [none]     @min 0   @max 1   */
#define CPCD_USE_SLIPANGLE FALSE /* Use slip angle in CP_CourseData_t */

/*! @brief       Tracking width of run up
    @general     [none]
    @conseq      [none]

    @attention   [none]
    @typical     [none]   @unit [none]     @min 0   @max 1.8   */
#define CPCD_RUN_UP_TRACK_WIDTH (1.8f)

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  CONSTANTS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/
/*!  @cond Doxygen_Suppress */
extern const CPCDCurveFilterPar_t CPCDCurveFilterPar;
extern const float32 fCPCDMaxAccelDist2Traj;
/*! @endcond */
/*****************************************************************************
  FUNCTIONS
*****************************************************************************/

#endif /* end of #ifndef _CP_CD_PARAMETER_INCLUDED */
