
/*!
   @defgroup cd CD Collision Detection (CGEB Situation Analysis)
   @ingroup vlc_sen

 The CD component determines the criticality of a situation
 The result is descriped as hypothesis
 The following hypothesis types are implemented:
 "Run-Up"
 The following hypothesis types will be implemented:
 "ACC", "Pass" and "Cut In"


@{ */
#ifndef _CD_EXT_INCLUDED
#define _CD_EXT_INCLUDED

#include "cd_par.h"

/*****************************************************************************
  (SYMBOLIC) CONSTANTS
*****************************************************************************/

#define CD_SW_VERSION_NUMBER (0x034800uL)
/*! Number of CD Hypothesis to be calculated at one point of time */
#define CD_NUMBER_OF_HYPOTHESES ((uint32)MAX_NUM_OF_HYPS)
/*! Number of CD Hypothesis types */
#define CD_NUMBER_OF_HYPOTHESES_TYPES (14u)

/* ****************************************************************
    TYPEDEF ENUM
    **************************************************************** */
/*! @brief     CD Module State Initialization enum

    @general   This enum is used to select different states of CD state machine
               CD_STATE_INIT will initialize CD module (CDInit())
               CD_STATE_OK   will call CDPreProcessing() & CDRun()

    @conseq    Not Applicable

    @attention Not Applicable

*/
typedef enum {
    CD_STATE_INIT = 0u, /*!> initialize all    */
    CD_STATE_OK = 1u    /*!> normal processing */
} CDState_t;

/*****************************************************************************
  FUNCTION PROTOTYPES
*****************************************************************************/

extern MEMSEC_REF CDState_t CDState;
extern void CDInit(void);
extern void VLCCDProcess(void);

extern void CDMergeObjects(ObjNumber_t iObjectToKeep,
                           ObjNumber_t iObjectToDelete);
extern void CDDeleteObject(ObjNumber_t iObjectToDelete);
extern void CDMergeDeleteObjectsSameVLCID(ObjNumber_t const ObjNr);
extern void CDProcessEMPTrajectoryMeasFreeze(EMPTrajPred_t* pEMPTrajPredEgo);

#endif /* _CD_EXT_INCLUDED */
/** @} end defgroup */
