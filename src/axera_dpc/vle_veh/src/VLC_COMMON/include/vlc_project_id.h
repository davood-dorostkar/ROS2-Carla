/*! \file **********************************************************************

  COMPANY:                Tuerme

  PROJECT:                All

  CPU:                    All

  COMPONENT:              VLC

  MODULNAME:              vlc_project_id.h

  DESCRIPTION:            VLC types

  CREATION DATE:          11.09.2007


  ---*/
#ifndef VLC_PROJECT_ID_H
#define VLC_PROJECT_ID_H

#include "vlc_ver_custom.h"
/*****************************************************************************
  SYMBOLISCHE KONSTANTEN
*****************************************************************************/

/*****************************************************************************
Table for setting up  Project IDs

Interpretation in HEX:
TCCC PPPP:
T    => Technology
CCC  => Customer
PPPP => Project ID

Example:
000A 0001
T -> 0 ==> Radar
C -> A ==> 10 (Honda)
P -> 1 ==> 1

******************************************************************************/
/*! @cond Doxygen_Suppress */
/* Values from 0 7 */
#define VLC_SW_PROJID_TECH_OFFSET (268435456)
#define VLC_SW_PROJID_RADAR \
    (0) /*!< indicates radar project in Function Version Number*/
#define VLC_SW_PROJID_CAMERA (1)

/* Values from 0 4095 */
#define VLC_SW_PROJID_CUST_OFFSET (65536)
#define VLC_SW_PROJID_BMW \
    (0) /*!< indicates BMW project in Function Version Number*/
#define VLC_SW_PROJID_MERCEDES (1)
#define VLC_SW_PROJID_TOYOTA (2)
#define VLC_SW_PROJID_VW (3)
#define VLC_SW_PROJID_GM (4)
#define VLC_SW_PROJID_FORD (5)
#define VLC_SW_PROJID_M_LKW (6)
#define VLC_SW_PROJID_HYUNDAI (7)
#define VLC_SW_PROJID_MITSUBISHI (8)
#define VLC_SW_PROJID_MAZDA (9)
#define VLC_SW_PROJID_HONDA (10)
#define VLC_SW_PROJID_FIAT (11)
#define VLC_SW_PROJID_VOLVO (12)
#define VLC_SW_PROJID_RENAULT (13)
#define VLC_SW_PROJID_PSA (14)
#define VLC_SW_PROJID_NISSAN (15)
#define VLC_SW_PROJID_BASE (16)
#define VLC_SW_PROJID_GEELY (17)
#define VLC_SW_PROJID_OTHER (4095)

#define VLC_SW_PROJID_ARS4B0                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_BMW) + 0)
#define VLC_SW_PROJID_ARS4T0                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_TOYOTA) + 0)
#define VLC_SW_PROJID_ARS4D0                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_MERCEDES) + 0)
#define VLC_SW_PROJID_ARS4D1                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_MERCEDES) + 1)
#define VLC_SW_PROJID_ARS4D2                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_MERCEDES) + 2)
#define VLC_SW_PROJID_ARS4L0                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_M_LKW) + 0)
#define VLC_SW_PROJID_ARS4L1                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_M_LKW) + 1)
#define VLC_SW_PROJID_ARS4V0                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_VOLVO) + 0)
#define VLC_SW_PROJID_ARS4G0                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_GM) + 0)
#define VLC_SW_PROJID_ARS4G1                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_GM) + 1)
#define VLC_SW_PROJID_ARS410VW16                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_VW) + 0)
#define VLC_SW_PROJID_ARS410VW18                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_VW) + 1)
#define VLC_SW_PROJID_ARS410NN16                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_NISSAN) + 0)
#define VLC_SW_PROJID_ARS430NN17                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_NISSAN) + 1)
#define VLC_SW_PROJID_ARS410HI17                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_HYUNDAI) + 0)
#define VLC_SW_PROJID_ARS410RT17                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_RENAULT) + 0)
#define VLC_SW_PROJID_ARS430MI18                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_MITSUBISHI) + 0)
#define VLC_SW_PROJID_ARS430NCAP18                       \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_BASE) + 0)
#define VLC_SW_PROJID_ARS410RT28                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_RENAULT) + 1)
#define VLC_SW_PROJID_ARS410GY18                         \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_GEELY) + 0)
#define VLC_SW_PROJID_ARS510                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_RADAR) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_BASE) + 1)

/*
Camera
==========
SMFC4B0    -- 1 000 0000
SRLCAM4T0  -- 1 002 0000
MFC4T0     -- 1 002 0001
MFC440GY18 -- 1 00C 0001
*/
#define VLC_SW_PROJID_SMFC4B0                             \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_CAMERA) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_BMW) + 0)
#define VLC_SW_PROJID_MFS430BW16                          \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_CAMERA) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_BMW) + 1)
#define VLC_SW_PROJID_SRLCAM4T0                           \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_CAMERA) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_TOYOTA) + 0)
#define VLC_SW_PROJID_MFC4T0                              \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_CAMERA) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_TOYOTA) + 1)
#define VLC_SW_PROJID_MFC440GY18                          \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_CAMERA) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_VOLVO) + 0)
#define VLC_SW_PROJID_MFC430NCAP18                        \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_CAMERA) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_BASE) + 0)
#define VLC_SW_PROJID_MFC431                              \
    ((VLC_SW_PROJID_TECH_OFFSET * VLC_SW_PROJID_CAMERA) + \
     (VLC_SW_PROJID_CUST_OFFSET * VLC_SW_PROJID_BASE) + 1)

/* Find out the Project Name from its ID */
#if ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4B0))
#define VLC_ALL_SW_PROJECT_NAME "ARS4B0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4T0))
#define VLC_ALL_SW_PROJECT_NAME "ARS4T0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4D0))
#define VLC_ALL_SW_PROJECT_NAME "ARS4D0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4D1))
#define VLC_ALL_SW_PROJECT_NAME "ARS4D1_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4D2))
#define VLC_ALL_SW_PROJECT_NAME "ARS4D2_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4L0))
#define VLC_ALL_SW_PROJECT_NAME "ARS4L0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4L1))
#define VLC_ALL_SW_PROJECT_NAME "ARS4L1_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4V0))
#define VLC_ALL_SW_PROJECT_NAME "ARS4V0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4G0))
#define VLC_ALL_SW_PROJECT_NAME "ARS4G0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS4G1))
#define VLC_ALL_SW_PROJECT_NAME "ARS4G1_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS410VW16))
#define VLC_ALL_SW_PROJECT_NAME "ARS410VW16_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS410VW18))
#define VLC_ALL_SW_PROJECT_NAME "ARS410VW18_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS410NN16))
#define VLC_ALL_SW_PROJECT_NAME "ARS410NN16_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS430NN17))
#define VLC_ALL_SW_PROJECT_NAME "ARS430NN17_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS410HI17))
#define VLC_ALL_SW_PROJECT_NAME "ARS410HI17_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS410RT17))
#define VLC_ALL_SW_PROJECT_NAME "ARS410RT17_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS410RT28))
#define VLC_ALL_SW_PROJECT_NAME "ARS410RT28_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS430MI18))
#define VLC_ALL_SW_PROJECT_NAME "ARS430MI18_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS430NCAP18))
#define VLC_ALL_SW_PROJECT_NAME "ARS430NCAP18_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS410GY18))
#define VLC_ALL_SW_PROJECT_NAME "ARS410GY18_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_ARS510))
#define VLC_ALL_SW_PROJECT_NAME "ARS510_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_SMFC4B0))
#define VLC_ALL_SW_PROJECT_NAME "SMFC4B0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_MFS430BW16))
#define VLC_ALL_SW_PROJECT_NAME "MFS430BW16_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_SRLCAM4T0))
#define VLC_ALL_SW_PROJECT_NAME "SRLCAM4T0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_MFC4T0))
#define VLC_ALL_SW_PROJECT_NAME "MFC4T0_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_MFC440GY18))
#define VLC_ALL_SW_PROJECT_NAME "MFC440GY18_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_MFC430NCAP18))
#define VLC_ALL_SW_PROJECT_NAME "MFC430NCAP18_"
#elif ((VLCALL_SW_PROJ_ID) == (VLC_SW_PROJID_MFC431))
#define VLC_ALL_SW_PROJECT_NAME "MFC431_"
#else
#define VLC_ALL_SW_PROJECT_NAME "NN_"
#endif

/*! @endcond Doxygen_Suppress */
#endif
