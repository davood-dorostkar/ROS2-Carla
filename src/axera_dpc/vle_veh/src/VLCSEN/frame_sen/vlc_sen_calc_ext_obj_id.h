

/* Bedingte Einbindung */
#ifndef _VLC_SEN_CALC_EXT_OBJ_ID_H_INCLUDED
#define _VLC_SEN_CALC_EXT_OBJ_ID_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#include "vlc_par.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS (COMPONENTENEXTERN)
*****************************************************************************/

/*! If configuration does not define the number of external object-IDs
available,
then assume the ARS310 default of 31 possibilities [0 .. 30] */
#ifndef VLC_NR_EXTERN_OBJECT_ID
#define VLC_NR_EXTERN_OBJECT_ID (31)
#endif

/*! Definition of the extern object ID to assign if no extern object ID has
been assigned */
#ifndef VLC_EXTERN_OBJ_ID_NO_OBJECT
#define VLC_EXTERN_OBJ_ID_NO_OBJECT 255
#endif

/*****************************************************************************
  MACROS (COMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  TYPEDEFS (COMPONENTENEXTERN)
*****************************************************************************/

/*! The numeric type of the external object ID assigned to objects */
typedef uint8 VLCSenExtObjId_t;

/*****************************************************************************
  CONSTANTS (COMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  VARIABLES (COMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  FUNCTIONS (COMPONENTENEXTERN)
*****************************************************************************/

#ifdef __cplusplus
};
#endif

/* Ende der bedingten Einbindung */
#else
#endif
