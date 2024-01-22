/*! \file **********************************************************************

  COMPONENT:              MAT (math functions) */ /*!

                                 MODULENAME:             mat_prob_ext.h

                                 @brief                  This module contains
                                 all needed probability functions


                                 *****************************************************************************/
#ifndef MATPROB_INCLUDED
#define MATPROB_INCLUDED

/** @defgroup vlc_mat_prob VLC_MAT_PROB ( mathematical library probabalistic )
containes methods to link related probabilities (Bayes theory)
   @ingroup vlc_veh

@{ */

#include "vlc_glob_ext.h"

percentage_t MAT_PROB_BAYES2(percentage_t ProbabilityA,
                             percentage_t ProbabilityB,
                             const percentage_t CPT[4]);
percentage_t MAT_PROB_BAYES3(percentage_t ProbabilityA,
                             percentage_t ProbabilityB,
                             percentage_t ProbabilityC,
                             const percentage_t CPT[8]);
percentage_t MAT_PROB_BAYES4(percentage_t ProbabilityA,
                             percentage_t ProbabilityB,
                             percentage_t ProbabilityC,
                             percentage_t ProbabilityD,
                             const percentage_t CPT[16]);

#endif
/** @} end defgroup */
