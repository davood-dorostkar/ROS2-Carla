/*
 * Copyright (C) 2018-2020 by SenseTime Group Limited. All rights reserved.
 * shenzijian <shenzijian@senseauto.com>
 */

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */
/*****************************************************************************
  INCLUDES
*****************************************************************************/
#include "ved_consts.h"       // NOLINT
#include "ved.h"              // NOLINT
#include "tue_common_libs.h"  // NOLINT
// #include <vector>
#include <stdio.h>
#include <string.h>
// #include <iostream>
// #include <string>
// #ifdef __cplusplus
// extern "C" {
// #endif
#if (!CFG_VED__FPM_754)
#include <math.h>
#endif

/*****************************************************************************
  SYMBOLIC CONSTANTS
****************************************************************************/
#define ROUNDING_OFFSET (0.5F)
/* added to round a float calculation to an integer index */
#define THREE_BYTE_SHIFT (23UL)
/* amount of bits to shift a value by　3bytes*/
#define SIGN_BIT_SHIFT (31U)
/* amount of bits to shift to get the sign bit of a long value */
#define SIGN_BIT_MASK (0x7FFFFFFFU)
/* bitmask to to get the sign bit of a long value */
#define HALF_OF_LN_OF_2 (0x3EB17218U)
/* long value of (1/2)*ln(2) */
#define ONE_AND_A_HALF_OF_LN_2 (0x3F851592U)
/* long value of (1.5)*ln(2) */

/*****************************************************************************
  MACROS
*****************************************************************************/

#if (CFG_VED__FPM_754)
#define RE_INTRPR_UI32(val_) (*((uint32 *)&(val_)))
/* #define RE_INTRPR_I32(val_)  ( *((sint32 *) &(val_)) ) */
#define RE_INTRPR_F32(val_) (*((float32 *)&(val_)))
#endif

/*****************************************************************************
  TYPEDEFS
*****************************************************************************/

/*****************************************************************************
  VARIABLES
*****************************************************************************/

/*****************************************************************************
  LOCAL FUNCTION PROTOTYPES
*****************************************************************************/

static sint32 VED_HistGetIndex(const VED_Histogram_t *hist, float32 value);

/* **********************************************************************
  @fn                     VED_HistInit */ /*!
  @brief                  Initialize internal histogram properties

  @description            Sets all bins to 0
                          Sets ranges and volume properties
                          The histogram must be set up correctly, min and max
                          must not be the same, size must be > 0
                          Histograms should be defined by constant values to
                          ensure a valid histogram

  @param[in]              size     = number of bins
  @param[in]              minValue = minimum value of covered range
  @param[in]              maxValue = maximum value of covered range
  @param[in]              volume
  @param[in]              range
  @param[out]             histogram database (valid by definition at calling function)
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_HistInit(VED_Histogram_t *hist,
                  float32 *range,
                  float32 *volume,
                  uint32 size,
                  float32 minValue,
                  float32 maxValue) {
    float32 binWidth;
    uint32 binNo;

    hist->Mean = 0.F;
    hist->Dev = 0.F;
    hist->Sum = 0.F;

    hist->Size = size;

    hist->Range = range;
    hist->Volume = volume;

    binWidth = (maxValue - minValue) / (float32)size;
    hist->InvBinWidth = 1.F / binWidth;

    for (binNo = 0U; binNo < size; binNo++) {
        hist->Range[binNo] =
            (binWidth * ((float32)binNo + ROUNDING_OFFSET)) + minValue;
        hist->Volume[binNo] = 0.F;
    }

    return;
}

/* ***********************************************************************
  @fn                     VED_HistReInit */ /*!
  @brief                  Reinitialize internal histogram properties

  @description            Erases content data (bins and mean/dev/sum)
                          but not fixed data (e.g. range)
                          Histogram pointer is linked static through macro 
                          and therefore valid at runtime

  @param[in]              -
  @param[out]             hist histogram database (valid by definition at calling function)
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_HistReInit(VED_Histogram_t *hist) {
    uint32 binNo;

    hist->Mean = 0.F;
    hist->Dev = 0.F;
    hist->Sum = 0.F;

    /* step through all bins and clear their volumes */
    for (binNo = 0U; binNo < hist->Size; binNo++) {
        hist->Volume[binNo] = 0.F;
    }

    return;
}

/* ***********************************************************************
  @fn                     VED_HistReduce */ /*!
  @brief                  Reduces histogram data

  @description            Reduces histogram bins and sum by using a reduction factor
                          Histogram pointer is linked static through macro 
                          and therefore valid at runtime

  @param[in]              weight
  @param[out]             hist histogram database (valid by definition at calling function)
  @return                 void

  @pre
  @post                   -
**************************************************************************** */
void VED_HistReduce(VED_Histogram_t *hist, float32 weight) {
    uint32 binNo;

    for (binNo = 0U; binNo < hist->Size; binNo++) {
        hist->Volume[binNo] *= weight;
    }

    hist->Sum *= weight;

    return;
}

/* ***********************************************************************
  @fn                     VED_HistGetIndex */ /*!
  @brief                  Determination of bin index corresponding to given 
                          value

  @description            see brief description
                          Histogram pointer is linked static through macro 
                          and therefore valid at runtime

  @param[in]              value
  @param[out]             hist histogram database (valid by definition at calling function)
  @return                 index

  @pre
  @post                   index has to be tested pointing to legal range
**************************************************************************** */
static sint32 VED_HistGetIndex(const VED_Histogram_t *hist, float32 value) {
    sint32 idx;
    float32 fidx;

    fidx = ((value - hist->Range[0]) * hist->InvBinWidth) + ROUNDING_OFFSET;
    idx = (sint32)fidx;

    if ((float32)idx > fidx) {
        idx -= 1; /* decrement to get correctly rounded index */
    }

    return idx;
}

/* **********************************************************************
  @fn                     VED_HistAdd */ /*!
  @brief                  Add new sample to histogram

  @description            Adds a value into histogram and recalculates range,
                          volume and sum
                          Takes the weight of the value into calculation
                          Histogram pointer is linked static through macro 
                          and therefore valid at runtime
                          Weight is checked to be > 0 and volumes are >= 0,
                          so no division by zero can occur

  @param[in]              value  
  @param[in]              weight 
  @param[out]             hist histogram database
  @return                 void

  @pre
  @post                   -
**************************************************************************** */
void VED_HistAdd(VED_Histogram_t *hist, float32 value, float32 weight) {
    sint32 idx;

    idx = VED_HistGetIndex(hist, value);

    if ((fABS(weight) > C_F32_DELTA) && (idx >= 0L) &&
        (idx < (sint32)hist->Size)) {
        float32 invWeight;

        invWeight = 1.0F / (weight + hist->Volume[idx]);

        hist->Range[idx] =
            (hist->Range[idx] * (hist->Volume[idx] * invWeight)) +
            ((weight * value) * invWeight);

        hist->Volume[idx] += weight;
        hist->Sum += weight;
    }

    return;
}

/* **********************************************************************
  @fn                     VED_HistGetVolume */ /*!
  @brief                  Get volume / counts for specified range/bin

  @description            Returns volume for bin if index is in range of
                          histogram, otherwise returns 0
                          Histogram pointer is linked static through macro 
                          and therefore valid at runtime

  @param[in]              value weight
  @param[out]             hist histogram database (valid by definition at calling function)
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
float32 VED_HistGetVolume(const VED_Histogram_t *hist, float32 value) {
    sint32 idx;
    float32 ret;

    idx = VED_HistGetIndex(hist, value);

    if ((idx >= 0L) && (idx < (sint32)hist->Size)) {
        ret = hist->Volume[idx];
    } else {
        ret = 0.F;
    }
    return ret;
}

/* **********************************************************************
  @fn                     VED_HistGetMaxBin */ /*!
  @brief                  Find bin with maximum content of a hisogram. 

  @description            Returns volume (content) of the largest bin
                          Histogram pointer is linked static through macro 
                          and therefore valid at runtime

  @param[in]              -
  @param[out]             hist histogram database (valid by definition at calling function)
  @return                 void

  @pre
  @post                   -
**************************************************************************** */
uint32 VED_HistGetMaxBin(const VED_Histogram_t *hist) {
    uint32 idx;
    uint32 idxMax = 0U;
    float32 maxBin = hist->Volume[idxMax];

    for (idx = 0uL; idx < hist->Size; idx++) {
        if (hist->Volume[idx] > maxBin) {
            maxBin = hist->Volume[idx];
            idxMax = idx;
        }
    }
    return idxMax;
}

/* ***********************************************************************
  @fn                     VED_HistCalcMeanDev */ /*!
  @brief                  Calculate mean and standard deviation of a histogram

  @description            see brief description
                          Histogram pointer is linked static through macro 
                          and therefore valid at runtime

  @param[in]              -
  @param[out]             hist histogram database
  @return                 void

  @pre
  @post                   -
**************************************************************************** */
void VED_HistCalcMeanDev(VED_Histogram_t *hist) {
    uint32 idx;
    float32 mean = 0.F;
    float32 stdev = 0.F;
    float32 volSum = 0.F;

    for (idx = 0uL; idx < hist->Size; idx++) {
        mean += hist->Volume[idx] * hist->Range[idx];
        volSum += hist->Volume[idx];
    }

    if (TUE_CML_IsNonZero(volSum)) {
        mean /= volSum;
    } else {
        mean = 0.F;
    }

    for (idx = 0uL; idx < hist->Size; idx++) {
        stdev += SQR(hist->Range[idx] - mean) * hist->Volume[idx];
    }

    if (TUE_CML_IsNonZero(volSum)) {
        stdev = VED__SQRT(stdev / volSum);
    } else {
        stdev = 0.F;
    }

    /* copy values to output */
    hist->Mean = mean;
    hist->Dev = stdev;

    return;
}

/* ***********************************************************************
  @fn                     VED_HistCalcMeanCenter */ /*!
  @brief                  Calculate mean around center with counts raised 
                          to a power
  @description            see brief description
                          In case the power is > 1, the values are prescaled
                          by 1/1000 to avoid large numbers and possible overflow
                          In case of an empty histogram, 0 is returned as mean

  @param[in]              idxCenter center bin
  @param[in]              width around the center
  @param[in]              pwr
  @param[out]             hist histogram database (valid by definition at calling function)
  @return                 mean

  @pre
  @post                   -
**************************************************************************** */
float32 VED_HistCalcMeanCenter(const VED_Histogram_t *hist,
                               uint32 idxCenter,
                               uint32 width,
                               uint32 pwr) {
    uint32 idx, idx_start, idx_stop;
    float32 mean = 0.F;
    float32 volSum = 0.F;

    if (idxCenter >= width) {
        idx_start = idxCenter - width;
    } else {
        idx_start = 0uL;
    }

    if ((idxCenter + width) < hist->Size) {
        idx_stop = idxCenter + width;
    } else {
        idx_stop = (uint32)(hist->Size - 1u);
    }

    /* step through bins around the center */
    for (idx = idx_start; idx <= idx_stop; idx++) {
        const float32 preScale_c =
            0.001F; /* Prescale to avoid large values in case of power > 1 */
        float32 vol;

        vol = hist->Volume[idx] *
              ((pwr > 1u) ? (hist->Volume[idx] * preScale_c) : (float32)pwr);

        mean += vol * hist->Range[idx];
        volSum += vol;
    }

    if (volSum > 0.F) {
        mean /= volSum;
    } else {
        mean = 0.F;
    }

    return mean;
}

/* ***********************************************************************
  @fn                     VED_HistCalcMedian */ /*!
  @brief                  Calculate linear interpolated median value
                          
  @description            gets median value by summing up the bins until
                          half of the sum is exceeded, index of median
                          is then that found index minus 1
                          Bin of the median index cannot be 0, at least one
                          value is in the histogram (must be ensured by caller)
                          Median value is improved by a linear aproximation

  @param[in]              hist histogram database (valid by definition at calling function)
  @param[out]             -
  @return                 median

  @pre                    -
  @post                   -
**************************************************************************** */
float32 VED_HistCalcMedian(const VED_Histogram_t *hist) {
    uint32 idx;
    float32 median;
    float32 volSum = 0.F;
    float32 rank;
    float32 bwdth;
    boolean checkagain;

    /* Calculate median rank */
    rank = 0.5F * hist->Sum;

    /* First bin exceeding accumulated sum is median */

    checkagain = (boolean)(volSum < rank);
    for (idx = 0uL; (idx < hist->Size) && (checkagain == TRUE); idx++) {
        volSum += hist->Volume[idx];
        checkagain = (boolean)(volSum < rank);
    }

    /* get median range value */
    if (idx > 0) {
        idx--;
    }
    median = hist->Range[idx];

    /* Determine current bin width */
    bwdth = hist->Range[idx + 1U] - hist->Range[idx];

    /* improve value by linear interpolation */
    median += (-0.5F * bwdth) +
              ((((rank - (volSum - hist->Volume[idx])) - 0.5F) * bwdth) /
               hist->Volume[idx]);

    return (median);
}

#if ((!defined(CFG_VED__POLYSPACE_ERROR_DISABLE)) || \
     (!CFG_VED__POLYSPACE_ERROR_DISABLE))
/* ***********************************************************************
  @fn                     VED_HistCalcSum */ /*!
  @brief                  Calculate the sum of all bins

  @description            see brief description

  @param[in]              hist histogram database (valid by definition at calling function)
  @param[out]             -
  @return                 sum of all bins

  @pre                    -
  @post                   -
**************************************************************************** */
float32 VED_HistCalcSum(const VED_Histogram_t *hist) {
    float32 sum = 0.F;
    uint32 idx = 0uL;

    while (idx < hist->Size) {
        sum += hist->Volume[idx];
    }
    return sum;
}

#endif
/* ***********************************************************************
  @fn                     VED_StatIntervalInit */ /*!
  @brief                  Initialize all attributes (mean, deviation etc.)

  @description            Erases calculated data of given histogram

  @param[in]              -
  @param[out]             StatInterval interval = observation database
  @return                 void

  @pre
  @post                   -
**************************************************************************** */
void VED_StatIntervalInit(VED_StatInterval_t *StatInterval) {
    StatInterval->Sum = 0.F;
    StatInterval->SqSum = 0.F;
    StatInterval->Volume = 0.F;
    StatInterval->Mean = 0.F;
    StatInterval->Dev = 0.F;

    return;
}

/* **********************************************************************
  @fn                     VED_StatIntervalInitInput */ /*!
  @brief                  Initialize internal attributes used for acquisition
                          Result mean, dev are not initialized
  @description            see brief description

  @param[in]              -
  @param[out]             StatInterval interval = observation database
  @return                 void

  @pre
  @post                   -
**************************************************************************** */
void VED_StatIntervalInitInput(VED_StatInterval_t *StatInterval) {
    StatInterval->Sum = 0.F;
    StatInterval->SqSum = 0.F;
    StatInterval->Volume = 0.F;

    return;
}

/* ***********************************************************************
  @fn                     VED_StatIntervalAdd */ /*!
  @brief                  Add new input value to interval

  @description            see brief description, updates calculated data,
                          considers the weight

  @param[in]              Value     = input value
  @param[in]              Weight    = weighting of input value
  @param[in,out]          StatInterval interval  = observation database
  @return                 void

  @pre                    If value is not weighted, weightning must be 1.0
  @post                   -
**************************************************************************** */
void VED_StatIntervalAdd(VED_StatInterval_t *StatInterval,
                         float32 Value,
                         float32 Weight) {
    StatInterval->Sum += Value * Weight;
    StatInterval->SqSum += SQR(Value) * Weight;
    StatInterval->Volume += Weight;

    return;
}

/* ***********************************************************************
  @fn                     VED_StatIntervalReduce */ /*!
  @brief                  Reduction of interval

  @description            see brief description

  @param[in]              Factor    = Reduction factor
  @param[in,out]          StatInterval interval  = observation database
                   
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_StatIntervalReduce(VED_StatInterval_t *StatInterval, float32 Factor) {
    StatInterval->Sum *= Factor;
    StatInterval->SqSum *= Factor;
    StatInterval->Volume *= Factor;

    return;
}

/* ***********************************************************************
  @fn                     VED_StatIntervalMerge */ /*!
  @brief                  Combine two intervals to one resulting interval 

  @description            adds one statistic measurement interval
                          to another interval

  @param[in]              StatInterval_2  = 2nd observation interval
  @param[in,out]          StatInterval_1  = 1st onservation interval and result
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_StatIntervalMerge(VED_StatInterval_t *StatInterval_1,
                           const VED_StatInterval_t *StatInterval_2) {
    StatInterval_1->Sum += StatInterval_2->Sum;
    StatInterval_1->SqSum += StatInterval_2->SqSum;
    StatInterval_1->Volume += StatInterval_2->Volume;

    return;
}

/* ***********************************************************************
  @fn                     VED_StatIntervalMeanDev */ /*!
  @brief                  Calculate mean and deviation of interval

  @description            see brief description
                          deviation is set to 0 if it cannot be calculated

  @param[in,out]          StatInterval = observation interval
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_StatIntervalMeanDev(VED_StatInterval_t *StatInterval) {
    if (TUE_CML_IsNonZero(StatInterval->Volume)) {
        float32 Varianz;
        StatInterval->Mean = StatInterval->Sum / StatInterval->Volume;

        Varianz = (StatInterval->SqSum / StatInterval->Volume) -
                  SQR(StatInterval->Mean);

        if ((TUE_CML_IsNonZero(Varianz)) && (Varianz > 0.F)) {
            StatInterval->Dev = VED__SQRT(Varianz);
        } else {
            StatInterval->Dev = 0.F;
        }
    } else {
        StatInterval->Dev = 0.F;
    }

    return;
}

/* ***********************************************************************
  @fn                     VED_StatIntervalPrealloc */ /*!
  @brief                  Preallocate interval 

  @description            Sets interval to defined preallocation values
                          and calculated derived values

  @param[in]              Mean         = predefined mean value
  @param[in]              Volume       = predefined volume
  @param[in]              Dev          = predefined standard deviation
  @param[out]             StatInterval = observation interval
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_StatIntervalPrealloc(VED_StatInterval_t *StatInterval,
                              float32 Mean,
                              float32 Volume,
                              float32 Dev) {
    /* Sum from mean with the assumption that volume equal to number */
    StatInterval->Sum = Mean * Volume;

    /* Recalculate the square sum for standard deviation */
    StatInterval->SqSum = SQR(Dev) + SQR(Mean);
    StatInterval->SqSum = StatInterval->SqSum * Volume;

    /* Weightning is not considered */
    StatInterval->Volume = Volume;

    return;
}

/* ***********************************************************************
  @fn                VED_HpSort */ /*!
  @brief             Heapsort algorithm (see Numerical Recipes)

  @description       Sorts an array into ascending numerical order
                     Because of the upper bound on heapsort's running time and 
                     constant upper bound on its auxiliary storage, heap sort 
                     is used instead of quick sort

  @param[in]         no  = number of samples
  @param[out]        ra  = sorted rearrangement
  @return            void

  @pre               -
  @post              -
**************************************************************************** */
void VED_HpSort(uint32 no, float32 ra[]) {
    if (no > 1u) {
        boolean finish = FALSE;
        uint32 ir, l;

        l = no >> 1uL;
        ir = (no - 1u);

        while (finish == FALSE) {
            float32 rra;
            if (l > 0u) {
                /* Still in hiring phase */
                l--;
                rra = ra[l];
            } else {
                /* In retirement-and-promotion phase    */
                rra = ra[ir];   /*   Clear a space at end of array      */
                ra[ir] = ra[0]; /*   Retire the top of the heap into it */
                if (ir > 0u) {
                    ir--;
                }
                if (ir == 0u) {
                    /*   Done with the last promotion       */
                    ra[0] = rra; /*   The least component worker of all  */
                    finish = TRUE;
                }
            }
            if (finish == FALSE) {
                boolean b_Aborted = FALSE;
                uint32 i, j;

                i = l;
                // Whether in the hiring phase or promotion phase, we
                j = l + l;
                // here set up to sift down element rra to its proper level

                while ((b_Aborted == FALSE) && (j <= ir)) {
                    if ((j < ir) && (ra[j] < ra[j + 1u])) {
                        /* Compare to the better underling */
                        j++;
                    }
                    if (rra < ra[j]) {
                        /* Demote rra */
                        ra[i] = ra[j];
                        i = j;
                        if (j > 0U) {
                            j = j << 1UL;
                        } else {
                            j = 1UL;
                        }
                    } else {
                        /* Found rra's level. Terminate the sift-down */
                        b_Aborted = TRUE;
                    }
                }
                /* Put rra into its slot */
                ra[i] = rra;
            }
        }
    }
    return;
}

/* ***********************************************************************
  @fn                     VED_HpSortInd */ /*!
  @brief                  Heapsort algorithm (see Numerical Recipes)

  @description            Sorts an index array referncing a numerical array 
                          into ascending numerical order

  @param[in]              no  = number of samples
  @param[in]              ra  = array of samples
  @param[out]             idx = sorted index rearrangement
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_HpSortInd(uint32 no, const float32 ra[], uint32 idx[]) {
    uint32 i;

    /* Initialize index array */
    for (i = 0uL; i < no; i++) {
        idx[i] = i;
    }

    if (no > 1u) {
        boolean finish = FALSE;

        uint32 l = no >> 1;
        uint32 ir = (no - 1u);

        while (finish == FALSE) {
            uint32 ridx;
            if (l > 0u) {
                /* Still in hiring phase */
                l--;
                ridx = idx[l];
            } else {
                /* In retirement-and-promotion phase */
                ridx = idx[ir];   /* Clear a space at end of array */
                idx[ir] = idx[0]; /* Retire the top of the heap into it */
                if (ir > 0u) {
                    ir--;
                }
                if (ir == 0u) {
                    /* Done with the last promotion */
                    idx[0] = ridx; /* The least component worker of all */
                    finish = TRUE;
                }
            }
            if (finish == FALSE) {
                boolean b_Aborted = FALSE;

                i = l;
                // Whether in the hiring phase or promotion phase, we
                uint32 j = l + l;
                // here set up to sift down element rra to its proper level */
                while ((b_Aborted == FALSE) && (j <= ir)) {
                    if ((j < ir) && (ra[idx[j]] < ra[idx[j + 1u]])) {
                        /* Compare to the better underling */
                        j++;
                    }
                    if (ra[ridx] < ra[idx[j]]) {
                        /* Demote rra */
                        idx[i] = idx[j];
                        i = j;
                        if (j > 0u) {
                            j = j << 1U;
                        } else {
                            j = 1UL;
                        }
                    } else { /* Found rra's level. Terminate the sift-down */
                        b_Aborted = TRUE;
                    }
                }
                /* Put rra into its slot */
                idx[i] = ridx;
            }
        }
    }
    return;
}

/* ***********************************************************************
  @fn                     VED_HpSortIndU16 */ /*!
  @brief                  Heapsort algorithm (see Numerical Recipes)

  @description            Sorts an index array referencing a numerical array 
                          into ascending numerical order

  @param[in]              no  = number of samples
  @param[in]              ra  = array of samples
  @param[out]             idx = sorted index rearrangement
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_HpSortIndU16(uint32 no, const uint16 ra[], uint8 idx[]) {
    uint32 i;

    /* Initialize index array */
    for (i = 0uL; i < no; i++) {
        idx[i] = (uint8)i;
    }

    if (no > 1u) {
        boolean finish = FALSE;

        uint32 l = no >> 1;
        uint32 ir = (no - 1u);

        while (finish == FALSE) {
            uint32 ridx;
            if (l > 0u) {
                /* Still in hiring phase */
                l--;
                ridx = idx[l];
            } else {
                /* In retirement-and-promotion phase */
                ridx = idx[ir];   /* Clear a space at end of array */
                idx[ir] = idx[0]; /* Retire the top of the heap into it */
                if (ir > 0u) {
                    ir--;
                }
                if (ir == 0u) {
                    /* Done with the last promotion */
                    idx[0] =
                        (uint8)ridx; /* The least component worker of all */
                    finish = TRUE;
                }
            }
            if (finish == FALSE) {
                boolean b_Aborted = FALSE;

                i = l; /* Whether in the hiring phase or promotion phase, we */
                uint32 j = l + l;
                while ((b_Aborted == FALSE) && (j <= ir)) {
                    if ((j < ir) && (ra[idx[j]] < ra[idx[j + 1u]])) {
                        /* Compare to the better underling */
                        j++;
                    }
                    if (ra[ridx] < ra[idx[j]]) {
                        /* Demote rra */
                        idx[i] = idx[j];
                        i = j;
                        if (j > 0u) {
                            j = (j << 1UL);
                        } else {
                            j = 1UL;
                        }
                    } else {
                        /* Found rra's level. Terminate the sift-down */
                        b_Aborted = TRUE;
                    }
                }
                /* Put rra into its slot */
                idx[i] = (uint8)ridx;
            }
        }
    }
    return;
}

/* **********************************************************************
  @fn                     VED_HpSortIndF32 */ /*!
  @brief                  Heapsort algorithm (see Numerical Recipes)

  @description            Sorts an index array referencing a numerical array 
                          into ascending numerical order

  @param[in]              no  = number of samples
  @param[in]              ra  = array of samples
  @param[out]             idx = sorted index rearrangement
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_HpSortIndF32(uint32 no, const float32 ra[], uint8 idx[]) {
    uint32 i;

    /* Initialize index array */
    for (i = 0uL; i < no; i++) {
        idx[i] = (uint8)i;
    }

    if (no > 1u) {
        boolean finish = FALSE;

        uint32 l = no >> 1;
        uint32 ir = (no - 1u);

        while (finish == FALSE) {
            uint32 ridx;
            if (l > 0u) {
                /* Still in hiring phase */
                l--;
                ridx = idx[l];
            } else {
                /* In retirement-and-promotion phase */
                ridx = idx[ir];   /* Clear a space at end of array */
                idx[ir] = idx[0]; /* Retire the top of the heap into it */
                if (ir > 0u) {
                    ir--;
                }
                if (ir == 0u) {
                    /* Done with the last promotion */
                    idx[0] =
                        (uint8)ridx; /* The least component worker of all */
                    finish = TRUE;
                }
            }
            if (finish == FALSE) {
                boolean b_abort = FALSE;

                i = l; /* Whether in the hiring phase or promotion phase, we */
                uint32 j = l + l;
                while ((b_abort == FALSE) && (j <= ir)) {
                    if ((j < ir) && (ra[idx[j]] < ra[idx[j + 1u]])) {
                        /* Compare to the better underling */
                        j++;
                    }
                    if (ra[ridx] < ra[idx[j]]) {
                        /* Demote rra */
                        idx[i] = idx[j];
                        i = j;
                        if (j > 0u) {
                            j = (j << 1UL);
                        } else {
                            j = 1UL;
                        }
                    } else {
                        /* Found rra's level. Terminate the sift-down */
                        b_abort = TRUE;
                    }
                }
                /* Put rra into its slot */
                idx[i] = (uint8)ridx;
            }
        }
    }
    return;
}

/* ***********************************************************************
  @fn                     VED_Discretize */ /*!
  @brief                  Discretize input value at given interval

  @description            Creates discretized value with given quantization 
                          Caller must ensure that quantization interval is not 0,
                          should be a constant value

  @param[in]              value input value
  @param[in]              interval quantization interval
  @param[out]             -
  @return                 disretized value
  
  @pre                    input value within -2^31 .. 2^31-1
  @post                   -
**************************************************************************** */
float32 VED_Discretize(float32 value, float32 interval) {
    sint32 intValue;

    value /= interval;

    intValue = ROUND_TO_INT(value);
    value = interval * (float32)intValue;

    return (value);
}

/* **********************************************************************
  @fn                     VED_InitCurve */ /*!
  @brief                  Initialization of curvature data

  @description            see brief description

  @param[in]              Course
  @param[out]             -
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_InitCurve(VED_OutCurve_t *Course) {
    Course->Curve = 0.F;
    Course->Gradient = 0.F;
    Course->Variance = 0.F;

    return;
}

/* **********************************************************************
  @fn               VED_CalcGradient */ /*!
  @brief            Calculate the gradient

  @description      Calculates gradient for one VED cycle
                    Note: The cycle time is given from system and checked
                    at the beginning of VED.  If a cycle time of 0ms was detected,
                    the cylce time was set to the default of 20ms and will 
                    never be 0 within VED

  @param[in]        NewValue
  @param[in]        OldValue
  @return           Gradient

  @pre              -
  @post             -
**************************************************************************** */
float32 VED_CalcGradient(float32 NewValue, float32 OldValue) {
    float32 CycleTime;
    float32 Gradient;

    CycleTime = VED_GetCycleTime();

    /* Calculate gradient (cycletime cannot be zero, has been set to 20ms in
     * that case) */
    Gradient = (NewValue - OldValue) / (CycleTime);

    return (Gradient);
}

/* ***********************************************************************
 @fn                     VED_CalcDistStblGrad */ /*!
  @brief                  Calculation of driven distance with stable gradient

  @description            Filters the value and adds driven distance to distance
                          if gradient is stable (calculated gradient is smaller
                          than given max gradient), otherwise the driven distance
                          is reset to 0m.
                          The driven distance is limited to 150m.

  @param[in]              GradMax    Maximum abolute value for stable range
  @param[in]              Grad       Absolut value of current gradient
  @param[in]              VehSpeed     Old driven distance with steady gradient
  @param[in,out]          GradAbsOld    Absolut value of last cycle gradient
  @param[out]             DeltaDist     Distance with steady gradient 
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_CalcDistStblGrad(float32 GradMax,
                          float32 Grad,
                          float32 *GradAbsOld,
                          float32 *DeltaDist,
                          float32 VehSpeed) {
    const float32 PeakFilterTime_c =
        1.0F;                        /* Filter time constant of peak filter */
    const float32 maxDist_c = 150.F; /* Distance saturation value */
    float32 CycleTime;

    CycleTime = VED_GetCycleTime();

    /* Peak filtering of absolut gradient value  */
    if (fABS(Grad) > *GradAbsOld) {
        *GradAbsOld = fABS(Grad);
    } else {
        *GradAbsOld = VED__IIR_FILTER(fABS(Grad), *GradAbsOld, PeakFilterTime_c,
                                      CycleTime);
    }

    /* Test if gradient is within stable range */
    if (*GradAbsOld < GradMax) {
        /* If gradient is within stable range accumulate distance with
         * saturation */
        if (*DeltaDist < maxDist_c) {
            *DeltaDist += (float32)(VehSpeed * CycleTime);
        }
    } else {
        /* Gradient is too big i.e. instable, Reset accumulated distance */
        *DeltaDist = (float32)0.F;
    }

    return;
}

/* ***********************************************************************
  @fn                     VED_CalcCycleDistance */ /*!
  @brief                  Calculate the driven distance during cycle

  @description            Distance calculation via zero order approximation

  @param[in]              VehSpeed
  @param[out]             -
  @return                 Driven distance

  @pre                    -
  @post                   -
**************************************************************************** */
float32 VED_CalcCycleDistance(float32 VehSpeed) {
    float32 CycleTime; /* Cycle time */
    float32 CicDist;   /* Driven Distance [m] */

    CycleTime = VED_GetCycleTime();

    CicDist = (float32)(CycleTime * VehSpeed);

    return (CicDist);
}

/* ***********************************************************************
  @fn                     VED_LFunction */ /*!
  @brief                  Linear ramp function

  @description            Calculates function value of input for
                          a linear function defined by 2 points (min and max).
                          The function value is limited to the interval
                          given by the min and max points

  @param[in]              Input   Current input value
  @param[in]              InputMin 
  @param[in]              OutputMin  1st node
  @param[in]              InputMax
  @param[in]              OutputMax  2nd node
  @param[out]             -
  @return                 Value between/equal to output min and max value 

  @pre                    -
  @post                   -
**************************************************************************** */
float32 VED_LFunction(float32 Input,
                      float32 InputMin,
                      float32 InputMax,
                      float32 OutputMin,
                      float32 OutputMax) {
    float32 Output;
    float32 Gradient;
    float32 Section;

    /*        Imin Imax  */

    Gradient = (OutputMax - OutputMin) / (InputMax - InputMin); /* slope */
    Section = OutputMin - (Gradient * InputMin); /* y-axis intercept */

    Output = (Gradient * Input) + Section; /* Linear equation */

    /* Limit values */
    if (OutputMin < OutputMax) {                               /*     /-- */
        Output = TUE_CML_MinMax(OutputMin, OutputMax, Output); /* --/     */
    } else {                                                   /* --\    */
        Output = TUE_CML_MinMax(OutputMax, OutputMin, Output); /*    \-- */
    }
    return (Output);
}

/*****************************************************************************
  @fn             VED_LinInterp1 */ /*!
  @brief          Linear interpolation function

  @description    Calculation of y(x) in a param table with interpolation

  @param[in]      xn table	pointers to param table [x1,x2,..], [y1,y2,..]
  @param[in]      yn table	pointers to param table [x1,x2,..], [y1,y2,..]
  @param[in]      num number of points [x,y] in param table
  @param[in]      xi value
  @return         y(x)

  @pre            -
  @post           -
*****************************************************************************/
float32 VED_LinInterp1(const float32 *xn,
                       const float32 *yn,
                       const uint32 num,
                       float32 xi) {
    uint32 ni;
    float32 yi;

    /* find range on x-axis */
    ni = 1u;

    while ((ni < num) && (xi > *xn)) {
        xn++;
        yn++;
        ni++;
    }

    /* calculate y */
    if (ni > 1U) {
        float32 x2;
        float32 y2;
        float32 x1;
        float32 y1;

        /* take current table position and previous table position for
         * interpolation */
        x1 = *(xn - 1UL);
        y1 = *(yn - 1UL);

        x2 = *(xn);
        y2 = *(yn);

        if (x1 < x2) {
            if (xi < x2) {
                /* interpolation */
                yi = (((xi - x1) * (y2 - y1)) / (x2 - x1)) + y1;
            } else {
                /* if x > current (last) table position use last y value from
                 * table */
                yi = y2;
            }
        } else {
            yi = y1;
        }
    } else {
        /*if x was smaller than than first value in table --> take first y value
         * from table*/
        yi = *(yn);
    }
    return yi;
}

/* ***********************************************************************
  @fn                     VED_FilterCycleTime */ /*!
  @brief                  First order IIR lowpass filter.

  @description            Internal filter coefficients are calculated with
                          respect to cycle time

  @param[in]              New :       new input value
  @param[in]              Old :       old output value
  @param[in]              TimeConst : Filter time constant in seconds
  @param[out]             -
  @return                 filter output value

  @pre                    -
  @post                   -
**************************************************************************** */
float32 VED_FilterCycleTime(float32 New, float32 Old, float32 TimeConst) {
    float32 CycleTime;
    float32 Value;

    CycleTime = VED_GetCycleTime();

    Value = VED__IIR_FILTER(New, Old, TimeConst, CycleTime);

    return (Value);
}

/* ***********************************************************************
  @fn                     VED_DifferentiateCycleTime */ /*!
  @brief                  IIR lowpass differentiator (DT1)

  @description            Internal filter coefficients are calculated with
                          respect to cycle time

  @param[in]              newIn :     current input value
  @param[in]              oldIn :     previous input value
  @param[in]              oldOut :    previous output value
  @param[in]              TimeConst: Filter time constant in seconds
  @param[out]             -
  @return                 filter output value

  @pre                    -
  @post                   -
**************************************************************************** */
float32 VED_DifferentiateCycleTime(float32 newIn,
                                   float32 oldIn,
                                   float32 oldOut,
                                   float32 TimeConst) {
    float32 CycleTime;
    float32 Value;

    /* Get cycle time */
    CycleTime = VED_GetCycleTime();

    /* Calculate new filter output */
    Value = CycleTime + TimeConst;
    if (TUE_CML_IsNonZero(Value) && (Value > 0)) {
        Value = (1.0F / Value) * ((newIn - oldIn) + (TimeConst * oldOut));
    } else {
        Value = oldOut; /* use old value if new value cannot be calculated */
    }

    return (Value);
}

/* ***********************************************************************
  @fn                VED_FilterOffset */ /*!
  @brief             First order IIR lowpass filter for offset estimation

  @description       IIR-Filter discretized PT1 with zero-order hold

  @param[in]         newin :   new measured input value
  @param[in]         oldflt :  last filter output value
  @param[in]         fconst :  filter constant with respect to increment
                              time domain     : time const / time step
                              distance domain : dist const / dist step 
  @param[out]        -
  @return            filtered output value

  @pre               -
  @post              -
**************************************************************************** */
float32 VED_FilterOffset(float32 newin, float32 oldflt, float32 fconst) {
    float32 coeff;
    float32 fltVal;

    coeff = 1.F - VED__EXP(-fconst);

    fltVal = oldflt + (coeff * (newin - oldflt));

    return fltVal;
}

/* **********************************************************************
  @fn                     VED_CheckCurve */ /*!
  @brief                  Verify whether the curvature is within limit or not

  @description            Checks if curve value is less than max curve value
                          If curve quality is less than min curve quality,
                          the curve value cannot be checked and an unknown
                          status is returned

  @param[in]              Course      Input curvature
  @param[in]              MaxCurve    Maximum curvature
  @param[in]              MinQuality  Minimum quality for test
  @param[out]             -
  @return                 status OK, NOT_OK, DONT_KNOW

  @pre                    -
  @post                   -
**************************************************************************** */
VED_CrvStatus_t VED_CheckCurve(const VED_OutCurve_t *Course,
                               float32 MaxCurve,
                               float32 MinQuality) {
    VED_CrvStatus_t Status; /* Output status */

    if (Course->Variance >= MinQuality) {
        /* Curve has sufficient quality for range test */
        if (fABS(Course->Curve) <= MaxCurve) {
            Status = (VED_CrvStatus_t)VED__CRV_OK; /* Curvature within limit */
        } else {
            Status = (VED_CrvStatus_t)
                VED__CRV_NOTOK; /* Curvature outside the limit */
        }
    } else {
        /* Insufficient quality  */
        Status = (VED_CrvStatus_t)VED__CRV_DONT_KNOW;
    }
    return (Status);
}

/* **********************************************************************
  @fn                     VED_GetCurveDir */ /*!
  @brief                  Determination of curvature direction

  @description            Checks the direction of a curve, curve is straight
                          if within straightforward threshold, otherwise
                          left or right based on sign
                          If curve quality is less than min curve quality,
                          the curve value cannot be checked and an unknown
                          status is returned

  @param[in]              Course      Input course
  @param[in]              ThrdCurve   Straightforward threshold
  @param[in]              MinQuality  Minimum quality for determination
  @param[out]             -
  @return                 Left, Right, Straightforward, Unknown

  @pre                    -
  @post                   -
**************************************************************************** */
VED_CrvDirStatus_t VED_GetCurveDir(const VED_OutCurve_t *Course,
                                   float32 ThrdCurve,
                                   float32 MinQuality) {
    VED_CrvDirStatus_t Status; /* Direction output value */

    if (Course->Variance >= MinQuality) {
        /* Curve has sufficient quality for direction decision */
        if (Course->Curve < -ThrdCurve) {
            Status = (VED_CrvDirStatus_t)CRV_DIR_RIGHT;
        } else if (Course->Curve > ThrdCurve) {
            Status = (VED_CrvDirStatus_t)CRV_DIR_LEFT;
        } else {
            Status = (VED_CrvDirStatus_t)CRV_DIR_STRAIGHT;
        }
    } else {
        Status =
            (VED_CrvDirStatus_t)CRV_DIR_DONT_KNOW; /* insufficient quality */
    }
    return (Status);
}

/* ***********************************************************************
  @fn                     VED_FormatCurve */ /*!
  @brief                  Format the curvature

  @description            Limit the curvature within mininum and maximum value

  @param[in,out]          Curve Curvature
  @return                 void

  @pre                    -
  @post                   -
**************************************************************************** */
void VED_FormatCurve(float32 *Curve) {
    if (fABS(*Curve) < (float32)(1.F / VED__MAX_RADIUS)) {
        *Curve = (float32)0.F;
    } else if (*Curve > (float32)(1.F / VED__MIN_RADIUS)) {
        *Curve = (float32)(1.0F / VED__MIN_RADIUS);
    } else if (*Curve < (float32)(-1.F / VED__MIN_RADIUS)) {
        *Curve = (float32)(-1.0F / VED__MIN_RADIUS);
    } else {
        /* do nothing */
    }
    return;
}

/* ***********************************************************************
  @fn                     VED_CurveToRadius */
void VED_CurveToRadius(float32 Curve, float32 *Radius) {
    if (fABS(Curve) < (float32)(1.F / VED__MAX_RADIUS)) {
        /* Absolute curvature below minimum zero threshold value */
        *Radius = (float32)0.F;
    } else if (Curve > (float32)(1.F / VED__MIN_RADIUS)) {
        /* Curvature above pos. maximum threshold value */
        *Radius = VED__MIN_RADIUS;
    } else if (Curve < (float32)(-1.F / VED__MIN_RADIUS)) {
        /* Curvature below neg. maximum threshold value */
        *Radius = -VED__MIN_RADIUS;
    } else {
        *Radius = (float32)((float32)1.F / Curve);
    }
    return;
}

#if (CFG_VED__FPM_754)
/* **********************************************************************
  @fn               VED_InvSqrt */ /*!
  @brief            Fast inverse square root approximation

  @description      Newton iteration,

  @param[in]        x
  @param[out]       -
  @return           Approximation of x^-1/2

  @pre              x > 0, IEEE754 Floating Point format
  @post             none
**************************************************************************** */
float32 VED_InvSqrt(float32 x) {
    float32 xhalf = 0.5F * x;
    uint32 uiVal;

    uiVal = RE_INTRPR_UI32(x);
    uiVal = 0x5F375A86U -
            (uiVal >> 1U); /* Gives initial guess y0 with magic number */

    x = RE_INTRPR_F32(uiVal);

    x *= (1.5F -
          (xhalf * x * x)); /* Newton steps, repeating increases accuracy */
    x *= (1.5F - (xhalf * x * x));
    x *= (1.5F - (xhalf * x * x));

    return x;
}

/* **********************************************************************
  @fn               VED_InvSqrt*/ /*!
  @brief            Fast square root approximation

  @description      Approximation by fast inverse square root approximation

  @param[in]        x
  @param[out]       -
  @return           Approximation by fast inverse square root approximation

  @pre              x > 0, IEEE754 Floating Point format
  @post             none
**************************************************************************** */
float32 VED_Sqrt(float32 x) {
    uint32 uiVal;

    uiVal = (RE_INTRPR_UI32(x) >> THREE_BYTE_SHIFT) & (uint32)255UL;

    if (0U == uiVal) {
        x = 0.0F;
    } else {
        x *= VED_InvSqrt(x);
    }
    return x;
}

/* **********************************************************************
  @fn               VED_Exp */ /*!
  @brief            Fast approximation exponential function

  @description      see brief description

  @param[in]        x
  @param[out]       -
  @return           Approximation of exp(x)

  @pre              x < 80, IEEE754 Floating Point format
  @post             Postcondition: none
**************************************************************************** */
float32 VED_Exp(float32 x) {
    float32 y;       /* Function result value */
    sint32 i2k = 0L; /* Integer part for two base exponent */
    sint32 xsb;      /* Sign bit: xsb=0 -> x>=0, xsb=1 -> x<0 */
    uint32 uiVal;    /* Integer interpretation of float value */

    /* Reinterprete value as Dword */

    uiVal = RE_INTRPR_UI32(x);

    xsb = (sint32)((uiVal >> SIGN_BIT_SHIFT) & (uint32)1U);
    uiVal &= SIGN_BIT_MASK;

    /* Test if argument is outside the desired reduction range */
    if (uiVal > HALF_OF_LN_OF_2) {
        /* Natural logarithm of 2 as high and low value to increase accuracy for
         * both sign. */
        const float32 ln2HiPrt_c[2] = {+6.9313812256e-01F, -6.9313812256e-01F};
        const float32 ln2LoPrt_c[2] = {+9.0580006145e-06F, -9.0580006145e-06F};

        float32 hi = 0.F; /* High part of reduced value */
        float32 lo = 0.F; /* Low part of reduced value */

        if (uiVal < ONE_AND_A_HALF_OF_LN_2) {
            /*  Simple reduction via addition or substraction */
            hi = x - ln2HiPrt_c[xsb];
            lo = ln2LoPrt_c[xsb];
            i2k = 1 - (xsb + xsb);
        } else {
            /* Complete reduction is necessary */
            const float32 invln2_c = 1.4426950216F; /* 1/ln2 */
            const float32 halF_c[2] = {0.5F, -0.5F};

            /* Reduce to 0..ln2 and shift to -0.5*ln2 .. +0.5*ln2 */

            i2k = (sint32)((invln2_c * x) + halF_c[xsb]);
            hi = x - (((float32)i2k) * ln2HiPrt_c[0]);
            lo = ((float32)i2k) * ln2LoPrt_c[0];
        }
        x = hi - lo; /* Combine both parts */
    } else {
        /* Input argument is already within reduction range.  */
        i2k = 0L;
    }

    /* x is now in primary range */
    {
        // Approximation of exp(r) by a polynom on the interval [-0.34658,
        // +0.34658]
        const float32 a1_c = 0.0013793724F;
        const float32 a2_c = 0.0083682816F;
        const float32 a3_c = 0.0416686266F;
        const float32 a4_c = 0.1666652424F;
        const float32 a5_c = 0.4999999297F;

        /* Calculate polynom with horner schema */
        y = (((((((((((x * a1_c) + a2_c) * x) + a3_c) * x) + a4_c) * x) +
                a5_c) *
               x) +
              1.F) *
             x) +
            1.F;
    }

    /*  Scale back to obtain exp(x) = 2^k * exp(r) */

    uiVal = RE_INTRPR_UI32(y);

    uiVal += ((RE_INTRPR_UI32(i2k)) << 23UL);

    y = RE_INTRPR_F32(uiVal);

    return y;
}

// #endif
// #ifdef __cplusplus
// } /* extern "C" */
#endif

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define DLMU5_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */