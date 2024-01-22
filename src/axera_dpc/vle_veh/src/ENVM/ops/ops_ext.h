

#ifndef _OPS_EXT_INCLUDED
#define _OPS_EXT_INCLUDED

/*!
  @defgroup ops OPS (Object Preselection)
    @ingroup em

  The ops component computes the ACC object quality and
  decides on which objects are filled into the FCT object list

@{ */

#include "envm_consts.h"
#include "TM_Global_Types.h"
/*****************************************************************************
  INCLUDES
*****************************************************************************/

/*! OPS Version number */
#define OPS_SW_VERSION_NUMBER (20200610)

/*! sub-module state */
extern StateFPS_t StateFPS;
/*****************************************************************************
  FUNCTION
*****************************************************************************/
extern sint8 FPSGetIDRefToMerge(const sint8 FCTObjNr, const sint8 EMObjNr);
extern void TUESelectedObjPostProcess(void);
sint8 FPS_i_GetFCTObjIDLastCycle(uint32 const ui_Index);

#endif
/** @} end defgroup */
