
/**
@defgroup head HEAD (Hypothesis Evaluation And Decision)
general component to generate activations by evaluating hypotheses

   @ingroup vlc_veh

@{ */

#ifndef _HEAD_HE_INCLUDED
#define _HEAD_HE_INCLUDED
/*** START OF SINGLE INCLUDE SECTION ****************************************/

/*****************************************************************************
INCLUDES
*****************************************************************************/
//#include "head_autoversion.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

/*****************************************************************************
  MACROS
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief eHEADOpMode_t

    @general Operations modes for external framework control

    @conseq [ None ]

    @attention [ None ]

    */
typedef enum {
    HEADOpMode_Running,  /*!< normal operation */
    HEADOpMode_ShutDown, /*!< normal deactivation */
    HEADOpMode_Stop      /*!< severe deactivation */
} eHEADOpMode_t;

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief HEADState_t

    @general operating modes of sub-component

    @conseq [ None ]

    @attention [ None ]

    */
typedef enum {
    HEAD_STATE_INIT = 0u, /*!< initialize all    */
    HEAD_STATE_OK = 1u    /*!< normal processing */
} HEADState_t;

/*****************************************************************************
  GLOBAL CONSTANTS (EXTERNAL SCOPE)
*****************************************************************************/

/*****************************************************************************
  GLOBAL VARIABLES (EXTERNAL SCOPE)
*****************************************************************************/
/*!  @cond Doxygen_Suppress */
extern HEADState_t HEADState;
/*! @endcond */

/*****************************************************************************
  FUNCTION PROTOTYPES (EXTERNAL SCOPE)
*****************************************************************************/
/* entry functions for external framework */
extern void HEADSetOpMode(eHEADOpMode_t OpMode);

#ifndef HEAD_AUTOVERSION
#define HEAD_AUTOVERSION 0
#endif

/*** END OF SINLGE INCLUDE SECTION ******************************************/
#endif /* _HEAD_HE_INCLUDED */

/** @} end defgroup */
