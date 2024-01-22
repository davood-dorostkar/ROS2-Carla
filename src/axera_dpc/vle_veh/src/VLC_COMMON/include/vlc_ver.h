

#ifndef _VLC_VER_H_INCLUDED
#define _VLC_VER_H_INCLUDED

/*****************************************************************************
  INCLUDES
*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "vlc_project_id.h"
#include "vlc_ver_custom.h"

#define VLCALL_SW_MAIN_VER_NO 0x02

#define VLCALL_SW_SUB_VER_NO 0x44
/*! @brief VLCALL_SW_BUG_FIX_LEV and VLCALL_SW_INT_VER_NO are located in to
 * vlc_ver_cust.h */

#define VLCALL_SW_VERSION_NUMBER               \
    ((((uint32)VLCALL_SW_MAIN_VER_NO) << 24) | \
     (((uint32)VLCALL_SW_SUB_VER_NO) << 16) |  \
     (((uint32)VLCALL_SW_BUG_FIX_LEV) << 8) |  \
     (((uint32)VLCALL_SW_INT_VER_NO)))

/* This part is for filling the array uiAlgoVersionNumber in SEN and VEH
 * components */

#define STR_SW_VERSION_NUMBER_TEXT(x) #x
#define STR_SW_VERSION_INFO(m, s, f, i)                             \
    VLC_ALL_SW_PROJECT_NAME STR_SW_VERSION_NUMBER_TEXT(m)           \
        STR_SW_VERSION_NUMBER_TEXT(s) STR_SW_VERSION_NUMBER_TEXT(f) \
            STR_SW_VERSION_NUMBER_TEXT(i)                           \
                VLCALL_SW_ALGO_OFFICIAL VLCALL_SW_MODIFICATION_SUMMARY
#define VERSION_INFO_AS_TEXT                                         \
    STR_SW_VERSION_INFO(VLCALL_SW_MAIN_VER_NO, VLCALL_SW_SUB_VER_NO, \
                        VLCALL_SW_BUG_FIX_LEV, VLCALL_SW_INT_VER_NO)

/*****************************************************************************
  MACROS
*****************************************************************************/

#ifdef __PDO__
#define X_STR_VLC_VERSION_AS_TEXT(m, s, f, i) \
    STR_VLC_VERSION_AS_TEXT(m, s, f, i)
#define STR_VLC_VERSION_AS_TEXT(m, s, f, i) ui_##m##_##s##_##f##_##i
#ifdef VLCALL_SW_INT_VER_NO
#define VLC_VERSION_AS_TEXT                                                \
    X_STR_VLC_VERSION_AS_TEXT(VLCALL_SW_MAIN_VER_NO, VLCALL_SW_SUB_VER_NO, \
                              VLCALL_SW_BUG_FIX_LEV, VLCALL_SW_INT_VER_NO)
#else
#define VLC_VERSION_AS_TEXT                                                    \
    X_STR_VLC_VERSION_AS_TEXT(xx, VLCALL_SW_MAIN_VER_NO, VLCALL_SW_SUB_VER_NO, \
                              VLCALL_SW_BUG_FIX_LEV)
#endif
#endif

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

typedef uint32 VLCSwVersion_t;

/*****************************************************************************
  VARIABLEN (KOMPONENTENEXTERN)
*****************************************************************************/

/*****************************************************************************
  FUNKTIONEN (KOMPONENTENEXTERN)
*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _VLC_VER_H_INCLUDED */
