/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

#include "ved_consts.h"
#include "ved.h"

/*****************************************************************************
  @fn             VED__MAT_PROB_BAYES2 */
uint8 VED__MAT_PROB_BAYES2(uint8 ProbabilityA,
                           uint8 ProbabilityB,
                           const uint8 CPT[4]) {
    uint32 temp;
    uint32 p2;

    /*range check*/
    if (ProbabilityA > (uint8)(Percentage_max)) {
        ProbabilityA = Percentage_max;
    }
    if (ProbabilityB > (uint8)(Percentage_max)) {
        ProbabilityB = Percentage_max;
    }

    /*calculate probability*/
    temp = ((uint32)Percentage_max - (uint32)ProbabilityA) *
           ((uint32)Percentage_max - (uint32)ProbabilityB) * (uint32)CPT[0];
    temp += (uint32)(ProbabilityA) *
            ((uint32)Percentage_max - (uint32)ProbabilityB) * (uint32)CPT[1];
    temp += ((uint32)Percentage_max - (uint32)ProbabilityA) *
            (uint32)(ProbabilityB) * (uint32)CPT[2];
    temp += (uint32)(ProbabilityA) * (uint32)(ProbabilityB) * (uint32)CPT[3];

    /*fixed point scaling*/
    p2 = (uint32)Percentage_max * (uint32)Percentage_max;
    temp += p2 / (uint32)2;
    temp /= p2;
    if (temp > (uint32)Percentage_max) {
        temp = (uint32)Percentage_max;
    }

    return (uint8)temp;
}

/*****************************************************************************
  @fn             VED__MAT_PROB_BAYES3 */
uint8 VED__MAT_PROB_BAYES3(uint8 ProbabilityA,
                           uint8 ProbabilityB,
                           uint8 ProbabilityC,
                           const uint8 CPT[8]) {
    uint32 temp;

    /*range check*/
    if (ProbabilityC > (uint8)(Percentage_max)) {
        ProbabilityC = Percentage_max;
    }

    /*calculate probability using MAT_PROB_BAYES2*/
    temp = (uint32)VED__MAT_PROB_BAYES2(ProbabilityA, ProbabilityB, CPT) *
           ((uint32)Percentage_max - (uint32)ProbabilityC); /*C=False*/
    temp += (uint32)VED__MAT_PROB_BAYES2(ProbabilityA, ProbabilityB, &CPT[4]) *
            (uint32)(ProbabilityC); /*C=True*/

    /*fixed point scaling*/
    temp =
        (temp + ((uint32)Percentage_max / (uint32)2)) / (uint32)Percentage_max;
    if (temp > (uint32)Percentage_max) {
        temp = (uint32)Percentage_max;
    }

    return (uint8)temp;
}

/*****************************************************************************
  @fn             VED__MAT_PROB_BAYES4 */
uint8 VED__MAT_PROB_BAYES4(uint8 ProbabilityA,
                           uint8 ProbabilityB,
                           uint8 ProbabilityC,
                           uint8 ProbabilityD,
                           const uint8 CPT[16]) {
    uint32 temp;

    /*range check*/
    if (ProbabilityD > (uint8)(Percentage_max)) {
        ProbabilityD = Percentage_max;
    }

    /*calculate probability using MAT_PROB_BAYES3*/
    temp = (uint32)VED__MAT_PROB_BAYES3(ProbabilityA, ProbabilityB,
                                        ProbabilityC, CPT) *
           ((uint32)Percentage_max - (uint32)ProbabilityD); /*D=False*/
    temp += (uint32)VED__MAT_PROB_BAYES3(ProbabilityA, ProbabilityB,
                                         ProbabilityC, &CPT[8]) *
            (uint32)(ProbabilityD); /*D=True*/

    /*fixed point scaling*/
    temp = (temp + ((uint32)Percentage_max / (uint32)2UL)) /
           (uint32)Percentage_max;
    if (temp > (uint32)Percentage_max) {
        temp = (uint32)Percentage_max;
    }

    return (uint8)temp;
}
/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */