/*! \file **********************************************************************
  module PHYS */ /*!

                                                                  COMPONENT:
                                                                PHYS (Physical
                                                                Funktions)

                                                                  MODULENAME:
                                                                phys_kin_ext.h

                                                                  @brief
                                                                This module
                                                                contains all
                                                                kinematical
                                                                functions


                                                                *****************************************************************************/
#ifndef PHYS_KYN_HE_INCLUDED
#define PHYS_KYN_HE_INCLUDED

/** @defgroup vlc_phys_kyn VLC_PHYS_KYN ( physics library for kinematical
formulas )
containes methods contvert an acceleration to a torque and vise versa
   @ingroup vlc_veh

   @{ */

#include "vlc_glob_ext.h"

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern acceleration_t PHYS_CALC_NEEDED_DECEL(
    const acceleration_t long_accel,
    const velocity_t long_velocity,
    times_t reaction_time,
    const velocity_t obj_rel_long_velocity,
    const acceleration_t obj_rel_long_accel,
    const velocity_t obj_long_velocity,
    const acceleration_t obj_long_accel,
    const distance_t obj_long_diplacement);
extern times_t PHYS_CALCULATE_TTC(const velocity_t ObjectVelocity,
                                  const acceleration_t ObjectAccel,
                                  const velocity_t HostVelocity,
                                  const acceleration_t HostAccel,
                                  const distance_t ObjectDistance,
                                  velocity_t *ImpactVelocity);

#endif
/** @} end defgroup */
