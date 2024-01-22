/*! \file **********************************************************************

  COMPONENT:              MAT (math functions)

  MODULENAME:             mat_param_ext.h

  @brief                  This module contains all functions for getting a value
  from a param table

    ---*/
#ifndef MATPARAM_INCLUDED
#define MATPARAM_INCLUDED

/** @defgroup vlc_mat_param VLC_MAT_PARAM (mathematical library for calibration
tables)
containes methods to acces one and more dimensional calibration tables
   @ingroup vlc_veh

@{ */

/* This file assumes that global types are already known, that is: #include
 * "vlc_glob_ext.h" has been done */

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern sint16 MAT_CALCULATE_PARAM_VALUE1D(const sint16 table[],
                                          const uint16 num,
                                          const sint16 x);
extern float32 MAT_CALCULATE_PARAM_VALUE2D(const float32 z_table[],
                                           const float32 x_table[],
                                           const float32 y_table[],
                                           const uint16 numx,
                                           const uint16 numy,
                                           float32 x,
                                           float32 y);

#endif
/** @} end defgroup */
