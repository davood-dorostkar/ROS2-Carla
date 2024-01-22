/*! \file **********************************************************************

  COMPONENT:              MAT (math functions)  */ /*!

                                MODULENAME:             mat_fuzzy_ext.h

                                @brief                  This module contains all
                              fuzzy related functions like logical operations,
                              fuzzification and defuzzification.


                              *****************************************************************************/
#ifndef MATFUZZY_INCLUDED
#define MATFUZZY_INCLUDED

/** @defgroup vlc_mat_fuzzy  VLC_VLC_MAT_FUZZY ( mathematical library fuzzy
logic )
containes methods for fuzzyfication, defuzzyfication and logical operations
   @ingroup vlc_veh

@{ */
#include "vlc_glob_ext.h"
#include "TM_Global_TypeDefs.h"
/*! Defines the maximum amount of rules that can be set*/
#define FUZZY_MAXRULES 16

/*! Defines the scaling value for lingvars*/
#define FUZZY_SCALE1 1024

/*! Defines the scaling value for percentages*/
#define FUZZY_PERCENT 1024

/*! Defines for special values */
#define FUZZY_SMALL (-999999999L)
#define FUZZY_LARGE (999999999L)

/*! Standard type for fuzzy operations */
// typedef sint32 signed_fuzzy_t;

/*! @brief Standard type for fuzzy variable */
typedef struct fuzzy_var {
    signed_fuzzy_t x1, x2, x3, x4;
} fuzzy_var_t;

/* FUNKTION PROTOTYPES (KOMPONENT EXTERNAL) */
extern signed_fuzzy_t FUZZY_GET_FUZZY_VAL(const fuzzy_var_t *var,
                                          signed_fuzzy_t crispValue);
extern signed_fuzzy_t FUZZY_NOT(signed_fuzzy_t val);
extern signed_fuzzy_t FUZZY_AND(signed_fuzzy_t v1, signed_fuzzy_t v2);
extern signed_fuzzy_t FUZZY_OR(signed_fuzzy_t v1, signed_fuzzy_t v2);
extern signed_fuzzy_t FUZZY_PROD(signed_fuzzy_t v1, signed_fuzzy_t v2);
extern signed_fuzzy_t FUZZY_PROBOR(signed_fuzzy_t v1, signed_fuzzy_t v2);
extern void FUZZY_DEFUZZY_INIT(signed_fuzzy_t vmin, signed_fuzzy_t vmax);
extern void FUZZY_DEFUZZY_ADD(const fuzzy_var_t *var,
                              signed_fuzzy_t scale,
                              signed_fuzzy_t fuzzyval,
                              signed_fuzzy_t *pFuzzyAreaAlone,
                              signed_fuzzy_t *pFuzzyAreaPosAlone,
                              signed_fuzzy_t *pFuzzyMidAlone,
                              signed_fuzzy_t *pFuzzyValAlone);
extern signed_fuzzy_t FUZZY_DEFUZZY(void);
extern signed_fuzzy_t FUZZY_GET_CUT_VALUE(const fuzzy_var_t *var,
                                          signed_fuzzy_t scale,
                                          uint8 nr);

#endif
/** @} end defgroup */
