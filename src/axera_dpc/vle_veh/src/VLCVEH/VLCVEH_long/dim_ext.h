
/** @defgroup dim DIM (Driver Intention Monitoring)
 Interface to the component FCA. The component FCA is responsible for the driver
alert
 in case ACC is not active.

  - A functional specification can be found under <A
HREF="../../../fca_function_spec.mht">fca_function_spec.mht</A>
  - A design specification can be found under <A
HREF="../../../vlc_lat_design_spec.mht">vlc_lat_design_spec.mht</A>
   @ingroup vlc_veh

@{ */

#ifndef _DIM_EXT_INCLUDED
#define _DIM_EXT_INCLUDED

#include "vlc_veh.h"

#if (VLC_CFG_DRIVER_INTENTION_MONITORING)

/*==============================================================================================================================================*/

/*! @brief       Dim internal hypotheses count of EBA
    @general     No. of EBA Hypotheses in DIM Module
    @conseq      @incp  DIM will iterate one more count for EBA hypothesis check
                 @decp  DIM will iterate one less count for EBA hypothesis check
    @attention   [None]
    @typical     NA
    @unit        1 Count
    @min         1

*/
#define DIM_NR_OF_EBA_HYPOTHESES (3u)

/*! @brief       Dim internal hypotheses count of ALDW
    @general     No. of ALDW Hypotheses in DIM Module
    @conseq      @incp  DIM will iterate one more count for ALDW hypothesis
   check
                 @decp  DIM will iterate one less count for ALDW hypothesis
   check
    @attention   [None]
    @typical     NA
    @unit        1 Count
    @min         1

*/
#define DIM_NR_OF_ALDW_HYPOTHESES (3u)

/*! @brief       Dim internal hypotheses count of SI
    @general     No. of SI Hypotheses in DIM Module
    @conseq      @incp  DIM will iterate one more count for SI hypothesis check
                 @decp  DIM will iterate one less count for SI hypothesis check
    @attention   [None]
    @typical     NA
    @unit        1 Count
    @min         1

*/
#define DIM_NR_OF_SI_HYPOTHESES (1u)

/* ****************************************************************
    TYPEDEF ENUM DIMState_t
   **************************************************************** */
/*! @brief DIM operating modes.

    @general operating modes of sub-component.

    @conseq [None]

    @attention [None]

    */
typedef enum {
    DIM_STATE_INIT = 0u, /*!> initialize all    */
    DIM_STATE_OK = 1u    /*!> normal processing */
} DIMState_t;

/*external functions*/

/* ****************************************************************
    TYPEDEF STRUCT DIMHypothesisList_t
   **************************************************************** */
/*! @brief DIM Hypothesis List

    @general It contains total list of DIM_EBA, DIM_ALDW, DIM_SI Hypothesis

    @conseq [None]

    @attention [None]

    */
typedef struct {
    GDB_DMHypothesis_t
        rgDimHypEBA[DIM_NR_OF_EBA_HYPOTHESES]; /*!< Array of the Driver
                                                  Intention Monitor Hypotheses
                                                  for EBA */
} DIMHypothesisList_t;

extern MEMSEC_REF DIMHypothesisList_t DIMHypothesisList;
extern MEMSEC_REF DIMState_t DIMState;
#endif

#ifndef DIM_AUTOVERSION
#define DIM_AUTOVERSION 0
#endif

#endif /* _DIM_EXT_INCLUDED */

/** @} end defgroup */
