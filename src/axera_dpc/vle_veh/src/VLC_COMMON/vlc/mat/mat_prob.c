/*! \file **********************************************************************

  COMPONENT:              MAT (math functions)

  MODULENAME:             mat_prob.c

  @brief                  This module contains all needed probability functions


  ---*/

  /* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

//#include "vlc_glob_ext.h"
//#include "vlcSen_ext.h"
#include "mat_prob_ext.h"

/*************************************************************************************************************************
  Functionname:    MAT_PROB_BAYES2 */
percentage_t MAT_PROB_BAYES2(percentage_t ProbabilityA,
                             percentage_t ProbabilityB,
                             const percentage_t CPT[4]) {
    uint32 temp;
    uint32 p2;

    /*range check*/
    if (ProbabilityA > (percentage_t)Percentage_max) {
        ProbabilityA = (percentage_t)Percentage_max;
    }
    if (ProbabilityB > (percentage_t)Percentage_max) {
        ProbabilityB = (percentage_t)Percentage_max;
    }

    /*calculate probability*/
    temp = ((uint32)Percentage_max - ProbabilityA) *
           ((uint32)Percentage_max - ProbabilityB) * (uint32)CPT[0];
    temp += ((uint32)ProbabilityA) * ((uint32)Percentage_max - ProbabilityB) *
            (uint32)CPT[1];
    temp += ((uint32)Percentage_max - ProbabilityA) * ((uint32)ProbabilityB) *
            (uint32)CPT[2];
    temp += ((uint32)ProbabilityA) * ((uint32)ProbabilityB) * (uint32)CPT[3];

    /*fixed point scaling*/
    p2 = (uint32)Percentage_max * (uint32)Percentage_max;
    temp += p2 / (uint32)2;
    temp /= p2;
    if (temp > (uint32)Percentage_max) {
        temp = (uint32)Percentage_max;
    }

    return (percentage_t)temp;
}

/*************************************************************************************************************************
  Functionname:    MAT_PROB_BAYES3 */
percentage_t MAT_PROB_BAYES3(percentage_t ProbabilityA,
                             percentage_t ProbabilityB,
                             percentage_t ProbabilityC,
                             const percentage_t CPT[8]) {
    uint32 temp;

    /*range check*/
    if (ProbabilityC > (percentage_t)Percentage_max) {
        ProbabilityC = (percentage_t)Percentage_max;
    }

    /*calculate probability using MAT_PROB_BAYES2*/
    temp = (uint32)MAT_PROB_BAYES2(ProbabilityA, ProbabilityB, CPT) *
           ((uint32)Percentage_max - ProbabilityC); /*C=FALSE*/
    temp += (uint32)MAT_PROB_BAYES2(ProbabilityA, ProbabilityB, &CPT[4]) *
            ((uint32)ProbabilityC); /*C=TRUE*/

    /*fixed point scaling*/
    temp =
        (temp + ((uint32)Percentage_max / (uint32)2)) / (uint32)Percentage_max;
    if (temp > (uint32)Percentage_max) {
        temp = (uint32)Percentage_max;
    }

    return (percentage_t)temp;
}

/*************************************************************************************************************************
  Functionname:    MAT_PROB_BAYES4 */
percentage_t MAT_PROB_BAYES4(percentage_t ProbabilityA,
                             percentage_t ProbabilityB,
                             percentage_t ProbabilityC,
                             percentage_t ProbabilityD,
                             const percentage_t CPT[16]) {
    uint32 temp;

    /*range check*/
    if (ProbabilityD > (percentage_t)Percentage_max) {
        ProbabilityD = (percentage_t)Percentage_max;
    }

    /*calculate probability using MAT_PROB_BAYES3*/
    temp =
        (uint32)MAT_PROB_BAYES3(ProbabilityA, ProbabilityB, ProbabilityC, CPT) *
        ((uint32)Percentage_max - ProbabilityD); /*D=FALSE*/
    temp += (uint32)MAT_PROB_BAYES3(ProbabilityA, ProbabilityB, ProbabilityC,
                                    &CPT[8]) *
            ((uint32)ProbabilityD); /*D=TRUE*/

    /*fixed point scaling*/
    temp =
        (temp + ((uint32)Percentage_max / (uint32)2)) / (uint32)Percentage_max;
    if (temp > (uint32)Percentage_max) {
        temp = (uint32)Percentage_max;
    }

    return (percentage_t)temp;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */