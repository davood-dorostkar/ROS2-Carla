/*
 * Copyright (C) 2017-2021 by SenseTime Group Limited. All rights reserved.
 * He Qiushu <heqiushu@senseauto.com>
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

/*! \file **********************************************************************

  COMPONENT:              MAT (math functions)

  MODULENAME:             mat_fuzzy.c

  @brief                  This module contains all fuzzy related functions like
logical operations, fuzzification and defuzzification.


*****************************************************************************/
#include "mat_fuzzy_ext.h"
#include "mat_std_ext.h"
#define ASW_QM_CORE1_MODULE_START_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
static signed_fuzzy_t FuzzyArea, FuzzyAreaPos;

/*smallest/largest value of the result of the fuzzification*/
static signed_fuzzy_t Fuzzy_ValMin, Fuzzy_ValMax;
#define ASW_QM_CORE1_MODULE_STOP_SEC_VAR_NO_INIT_UNSPECIFIED
#include "ASW_MemMap.h"
/*-------------------------------------------------------------------------------------------*/

static signed_fuzzy_t FUZZY_GET_AREA(const fuzzy_var_t *var,
                                     signed_fuzzy_t scale,
                                     signed_fuzzy_t *mid);
static signed_fuzzy_t FUZZY_GET_AREA_POS(const fuzzy_var_t *var,
                                         signed_fuzzy_t scale,
                                         signed_fuzzy_t *mid);

/*************************************************************************************************************************
  Functionname:    FUZZY_GET_FUZZY_VAL */
signed_fuzzy_t FUZZY_GET_FUZZY_VAL(const fuzzy_var_t *var,
                                   signed_fuzzy_t crispValue) {
    /*values before and after the curve (standard:0)*/
    signed_fuzzy_t minval, maxval;
    signed_fuzzy_t return_val;

    minval = (signed_fuzzy_t)0;
    maxval = (signed_fuzzy_t)0;

    /*if first parameter of curve = 2nd one -> curve begins at value 1 (not 0)*/
    if (var->x1 == var->x2) {
        minval = (signed_fuzzy_t)FUZZY_SCALE1;
    }
    /*if 3rd parameter of curve = 4th one -> curve ends at value 1 (not 0)*/
    if (var->x3 == var->x4) {
        maxval = (signed_fuzzy_t)FUZZY_SCALE1;
    }

    if (crispValue <= var->x1) { /*smaller than begin of curve*/
        return_val = minval;
    } else {
        if (crispValue >= var->x4) { /*larger than end of curve*/
            return_val = maxval;
        } else {
            if ((crispValue >= var->x2) &&
                (crispValue <= var->x3)) { /*at the top of the curve*/
                return_val = (signed_fuzzy_t)FUZZY_SCALE1;
            } else {
                if ((crispValue > var->x1) &&
                    (crispValue < var->x2)) { /*rising part of the curve*/
                    return_val = ((crispValue - var->x1) *
                                  (signed_fuzzy_t)FUZZY_SCALE1) /
                                 (var->x2 - var->x1);
                } else {
                    if ((crispValue > var->x3) &&
                        (crispValue < var->x4)) { /*falling part of the curve*/
                        return_val = ((var->x4 - crispValue) *
                                      (signed_fuzzy_t)FUZZY_SCALE1) /
                                     (var->x4 - var->x3);
                    } else {
                        /*don't reach this!*/
                        return_val = 0;
                    }
                }
            }
        }
    }

    return return_val;
}

/*************************************************************************************************************************
  Functionname:    FUZZY_AND */
signed_fuzzy_t FUZZY_AND(signed_fuzzy_t v1, signed_fuzzy_t v2) {
    return MAT_MIN(v1, v2);
}

/*************************************************************************************************************************
  Functionname:    FUZZY_OR */
signed_fuzzy_t FUZZY_OR(signed_fuzzy_t v1, signed_fuzzy_t v2) {
    return MAT_MAX(v1, v2);
}

/*************************************************************************************************************************
  Functionname:    FUZZY_NOT */
signed_fuzzy_t FUZZY_NOT(signed_fuzzy_t val) {
    return MAT_MAX((signed_fuzzy_t)FUZZY_SCALE1 - val, (signed_fuzzy_t)0);
}

/*************************************************************************************************************************
  Functionname:    FUZZY_PROD */
signed_fuzzy_t FUZZY_PROD(signed_fuzzy_t v1, signed_fuzzy_t v2) {
    return (v1 * v2) / (signed_fuzzy_t)FUZZY_SCALE1;
}

/*************************************************************************************************************************
  Functionname:    FUZZY_PROBOR */
signed_fuzzy_t FUZZY_PROBOR(signed_fuzzy_t v1, signed_fuzzy_t v2) {
    return (v1 + v2) - FUZZY_PROD(v1, v2);
}

/*************************************************************************************************************************
  Functionname:    FUZZY_DEFUZZY_INIT */
void FUZZY_DEFUZZY_INIT(signed_fuzzy_t vmin, signed_fuzzy_t vmax) {
    Fuzzy_ValMin = vmin;
    Fuzzy_ValMax = vmax;
    FuzzyArea = 0;
    FuzzyAreaPos = 0;
}

/*************************************************************************************************************************
  Functionname:    FUZZY_DEFUZZY_ADD */
void FUZZY_DEFUZZY_ADD(const fuzzy_var_t *var,
                       signed_fuzzy_t scale,
                       signed_fuzzy_t fuzzyval,
                       signed_fuzzy_t *pFuzzyAreaAlone,
                       signed_fuzzy_t *pFuzzyAreaPosAlone,
                       signed_fuzzy_t *pFuzzyMidAlone,
                       signed_fuzzy_t *pFuzzyValAlone) {
    signed_fuzzy_t area;
    signed_fuzzy_t mid;

    mid = (signed_fuzzy_t)0;
    area = FUZZY_GET_AREA(
        var, (scale * fuzzyval) / (signed_fuzzy_t)FUZZY_PERCENT, &mid);

    *pFuzzyValAlone = fuzzyval;
    *pFuzzyAreaAlone = area;
    *pFuzzyMidAlone = mid;
    *pFuzzyAreaPosAlone = area * mid;

    FuzzyArea += area;
    FuzzyAreaPos += area * mid;
}

/*************************************************************************************************************************
  Functionname:    FUZZY_GET_CUT_VALUE */
signed_fuzzy_t FUZZY_GET_CUT_VALUE(const fuzzy_var_t *var,
                                   signed_fuzzy_t scale,
                                   uint8 nr) {
    signed_fuzzy_t return_val;
    return_val = (signed_fuzzy_t)0;

    if (nr == (uint8)1) {
        return_val = var->x1 + (((scale * (var->x2 - var->x1))) /
                                (signed_fuzzy_t)FUZZY_SCALE1);
    } else {
        if (nr == (uint8)2) {
            return_val = var->x4 - (((scale * (var->x4 - var->x3))) /
                                    (signed_fuzzy_t)FUZZY_SCALE1);
        } else {
            /*nr not in range - return 0*/
        }
    }

    return return_val;
}

/*************************************************************************************************************************
  Functionname:    FUZZY_GET_AREA */
static signed_fuzzy_t FUZZY_GET_AREA(const fuzzy_var_t *var,
                                     signed_fuzzy_t scale,
                                     signed_fuzzy_t *mid) {
    signed_fuzzy_t return_val;

    scale = MAT_LIM(scale, (sint32)-FUZZY_SCALE1, (sint32)FUZZY_SCALE1);

    if (scale == (signed_fuzzy_t)0) {
        return_val = (signed_fuzzy_t)0;
    } else {
        if (scale > (signed_fuzzy_t)0) {
            return_val = FUZZY_GET_AREA_POS(var, scale, mid);
        } else {
            /*A "NOT" OUTPUT (scale < 0)*/

            return_val = 0;
        }
    }

    return return_val;
}

/*************************************************************************************************************************
  Functionname:    FUZZY_GET_AREA_POS */
static signed_fuzzy_t FUZZY_GET_AREA_POS(const fuzzy_var_t *var,
                                         signed_fuzzy_t scale,
                                         signed_fuzzy_t *mid) {
    signed_fuzzy_t return_val;
    signed_fuzzy_t xc1, xc2;
    signed_fuzzy_t a1, a2, a3;
    signed_fuzzy_t xm1, xm2, xm3;
    signed_fuzzy_t sum;

    xc1 = FUZZY_GET_CUT_VALUE(
        var, scale, (uint8)1);  // find first  x value where the linguistic
                                // variable cuts the scale line
    xc2 = FUZZY_GET_CUT_VALUE(
        var, scale, (uint8)2);  // find second x value where the linguistic
                                // variable cuts the scale line

    xm1 = ((xc1 - var->x1) * (signed_fuzzy_t)2) /
          (signed_fuzzy_t)3; /*centroid of the left triangle part*/
    xm2 = (xc2 - xc1) /
          (signed_fuzzy_t)2; /*centroid of the middle rectangle part*/
    xm3 = (var->x4 - xc2) /
          (signed_fuzzy_t)3; /*centroid of the right triangle part*/

    a1 = ((xc1 - var->x1) * scale) /
         (signed_fuzzy_t)FUZZY_SCALE1; /*area of the left triangle part*/
    a2 = (((xc2 - xc1) * scale) * (signed_fuzzy_t)2) /
         (signed_fuzzy_t)FUZZY_SCALE1;
    a3 = ((var->x4 - xc2) * scale) /
         (signed_fuzzy_t)FUZZY_SCALE1; /*area of the right triangle part*/

    sum = (a1 + a2 + a3);

    if (sum == (signed_fuzzy_t)0) {
        *mid = (signed_fuzzy_t)0;
    } else {
        *mid =
            (((var->x1 + xm1) * a1) + ((xc1 + xm2) * a2) + ((xc2 + xm3) * a3)) /
            sum; /*calculate weighted centroid*/
    }

    /*reduce the area for smoother edges in sum (take upper part from the shape
     * instead of the lower (bigger) part*/

    xc1 = FUZZY_GET_CUT_VALUE(var, (signed_fuzzy_t)FUZZY_SCALE1 - scale,
                              (uint8)1);
    xc2 = FUZZY_GET_CUT_VALUE(var, (signed_fuzzy_t)FUZZY_SCALE1 - scale,
                              (uint8)2);

    a1 = ((var->x2 - xc1) * scale) / (signed_fuzzy_t)FUZZY_SCALE1;
    a2 = (((var->x3 - var->x2) * (signed_fuzzy_t)2) * scale) /
         (signed_fuzzy_t)FUZZY_SCALE1;
    a3 = ((xc2 - var->x3) * scale) / (signed_fuzzy_t)FUZZY_SCALE1;

    sum = a1 + a2 + a3;
    /*end reduce the area for smoother edges in sum*/

    return_val = sum;

    return return_val;
}

/*************************************************************************************************************************
  Functionname:    FUZZY_DEFUZZY */
signed_fuzzy_t FUZZY_DEFUZZY(void) {
    signed_fuzzy_t return_val;

    return_val = 0;

    if (FuzzyArea != (signed_fuzzy_t)0) {
        return_val = FuzzyAreaPos / FuzzyArea;
    } else {
        /*not valid - return 0*/
    }

    return return_val;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */