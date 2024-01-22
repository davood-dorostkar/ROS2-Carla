/*! \file **********************************************************************

  COMPONENT:              MAT (math functions)

  MODULENAME:             mat_std.c

  @brief                  This module contains all standard math functions like
  min/max operators


  ---*/


/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

//#include "vlcSen_ext.h"
//#include "vlc_glob_ext.h"
#include "mat_std_ext.h"
#include "mat_param_ext.h"

#define Mat_1_2_pi 15708
#define Mat_3_2_pi 47123
/*!@todo defined in algo glob for radar projects. Naming should be changed!*/
#ifndef Pi_fixpoint_angle
#define Pi_fixpoint_angle 31416
#endif

/*optimized table (sin(x) with accuracy of 1/factor_s (1000))*/
#define Mat_sin_table MAT_SIN_TABLE
#define Mat_sin_table_points (uint16)15
static const sint16 MAT_SIN_TABLE[2u * Mat_sin_table_points] =
    {/*x, sin(x)*/
     0,     0,   2444,  242, 3974,  387, 5259,  502, 6422,       599,
     7505,  682, 8526,  753, 9493,  813, 10432, 864, 11337,      906,
     12256, 941, 13132, 967, 13974, 985, 14933, 997, Mat_1_2_pi, 1000};

/*************************************************************************************************************************
  Functionname:    MAT_MIN */
sint32 MAT_MIN(sint32 Val1, sint32 Val2) {
    if (Val1 < Val2) {
        Val2 = Val1;
    } else {
        /*Val2 already < Val1*/
    }
    return Val2;
}

/*************************************************************************************************************************
  Functionname:    MAT_MAX */
sint32 MAT_MAX(sint32 Val1, sint32 Val2) {
    if (Val1 > Val2) {
        Val2 = Val1;
    } else {
        /*Val2 already > Val1;*/
    }
    return Val2;
}

/*************************************************************************************************************************
  Functionname:    MAT_LIM */
sint32 MAT_LIM(sint32 Val, sint32 Lim1, sint32 Lim2) {
    sint32 lower, higher;

    lower = MAT_MIN(Lim1, Lim2);

    if (Val < lower) {
        Val = lower;
    } else {
        higher = MAT_MAX(Lim1, Lim2);
        if (Val > higher) {
            Val = higher;
        } else {
            /*value already between Lim1 and Lim2*/
        }
    }
    return Val;
}

/*************************************************************************************************************************
  Functionname:    MAT_FILT */
sint32 MAT_FILT(sint32 NewVal, sint32 OldVal, sint32 FilterDepth) {
    sint32 ret_value;

    ret_value = OldVal;
    ret_value *= FilterDepth;
    ret_value += NewVal;
    ret_value /= (FilterDepth + (sint32)1);
    if ((ret_value == OldVal) && (OldVal != NewVal)) {
        if (OldVal < NewVal) {
            ret_value++;
        } else {
            if (OldVal > NewVal) {
                ret_value--;
            } else {
                /*nothing to do*/
            }
        }
    }

    return ret_value;
}

/*************************************************************************************************************************
  Functionname:    MAT_ABS */
sint32 MAT_ABS(sint32 i) {
    if (i < (sint32)0) {
        i = -i;
    } else {
        /*i already positive*/
    }

    return i;
}

/*************************************************************************************************************************
  Functionname:    MAT_DIFF_DT */
sint32 MAT_DIFF_DT(sint32 New, sint32 Old, times_t Cycle) {
    sint32 Gradient;
    Gradient = New - Old;
    Gradient *= (sint32)Time_s;
    /*comment: div/0 -> cycle time will never be zero*/
    Gradient /= (sint32)Cycle;
    return Gradient;
}

/*************************************************************************************************************************
  Functionname:    MAT_INT_DT */
sint32 MAT_INT_DT(sint32 New, sint32 Old, sint32 OldInt, times_t Cycle) {
    sint32 NewInt;
    NewInt = Old + New;
    NewInt /= (sint32)2;
    NewInt *= (sint32)Cycle;
    NewInt /= (sint32)Time_s;
    NewInt += OldInt;
    return NewInt;
}

/*************************************************************************************************************************
  Functionname:    MAT_LIM_GRAD */
sint32 MAT_LIM_GRAD(sint32 New,
                    sint32 Old,
                    sint32 max_neg_grad,
                    sint32 max_pos_grad,
                    times_t Cycle) {
    sint32 LimVal;

    LimVal = MAT_DIFF_DT(New, Old, Cycle);
    LimVal = MAT_LIM(LimVal, max_neg_grad, max_pos_grad);
    LimVal *= (sint32)Cycle;
    LimVal /= (sint32)Time_s;
    LimVal += Old;

    return LimVal;
}

/*************************************************************************************************************************
  Functionname:    MAT_PT1_FILTER */
sint32 MAT_PT1_FILTER(const times_t cycle_time,
                      times_t time_constant,
                      sint32 input_value,
                      sint32 output_value_last_cycle) {
    sint32 help;
    factor_t filter_constant;
    sint32 output_new;

    /* Initialize local variables */
    output_new = (sint32)0;
    filter_constant = (factor_t)0;

    if ((time_constant > cycle_time) && (time_constant < Time_s)) {
        /* Calculate filter constant T/T1 - cylce_time/time_constant */
        help = (sint32)cycle_time * (sint32)Time_s;
        help /= (sint32)time_constant;
        filter_constant = (factor_t)help;

        /* PT1 filter to prevent oscillations and other discontinuous waveforms
         */
        help = ((sint32)Factor_s - (sint32)filter_constant) *
                   (sint32)output_value_last_cycle +
               (sint32)filter_constant * (sint32)input_value;
        help /= (sint32)Factor_s;
        /* Write output */
        output_new = (acceleration_t)help;
    } else {
        /* Wrong time constant, no filtering applied */
        output_new = input_value;
    }

    return output_new;
}

#define Mat_max_sqrt_val 46340L /*sqrt(2^31)*/

/*************************************************************************************************************************
  Functionname:    MAT_SQRT */
sint32 MAT_SQRT(sint32 Val) {
    sint32 x, y;
    sint32 best_accuracy;

    /*start iteration with x0 = estimated sqrt (take half of bitlength +1 ) -->
     * faster (using d&c)*/
    if (Val > 0) {
        if (Val >= 0x10000L) /*2^16*/
        {
            if (Val >= 0x1000000L) /*2^24*/
            {
                if (Val >= 0x10000000L) /*2^28*/
                {
                    if (Val >= 0x40000000L) /*2^30*/
                    {
                        x = (sint32)
                            Mat_max_sqrt_val; /*>=30 set to Mat_max_sqrt_val
                                                 because of (1<<16)^2 = 65535^2
                                                 > max positive value of
                                                 sint32*/
                    } else {
                        x = (sint32)0x8000L; /*2^15*/
                    }
                } else {
                    if (Val >= 0x4000000L) /*2^26*/
                    {
                        x = 0x4000; /*2^14*/
                    } else {
                        x = 0x2000; /*2^13*/
                    }
                }
            } else {
                if (Val >= 0x100000L) /*2^20*/
                {
                    if (Val >= 0x400000L) /*2^22*/
                    {
                        x = 0x1000; /*2^12*/
                    } else {
                        x = 0x800; /*2^11*/
                    }
                } else {
                    if (Val >= 0x40000L) /*2^18*/
                    {
                        x = 0x400; /*2^10*/
                    } else {
                        x = 0x200; /*2^9*/
                    }
                }
            }
        } else {
            if (Val >= 0x100L) /*2^8*/
            {
                if (Val >= 0x1000L) /*2^12*/
                {
                    if (Val >= 0x4000L) /*2^14*/
                    {
                        x = 0x100; /*2^8*/
                    } else {
                        x = 0x80; /*2^7*/
                    }
                } else {
                    if (Val >= 0x400L) /*2^10*/
                    {
                        x = 0x40; /*2^6*/
                    } else {
                        x = 0x20; /*2^5*/
                    }
                }
            } else {
                if (Val >= 0x10L) /*2^4*/
                {
                    if (Val >= 0x40L) /*2^6*/
                    {
                        x = 0x10; /*2^4*/
                    } else {
                        x = 0x8; /*2^3*/
                    }
                } else {
                    x = 0x4; /*2^2*/
                }
            }
        }

        y = (x * x) - Val;
        best_accuracy = x + x;

        /*as long as new value is better than last value start a new iteration*/
        while (y >= best_accuracy) {
            x -= y / best_accuracy;
            y = (x * x) - Val;
            best_accuracy = x + x;
        }

        /*round down*/
        if (x > 1) {
            x--;
        }
    } else {
        /*return 0 in case Val <= 0*/
        x = 0;
    }
    return x;
}

/*************************************************************************************************************************
  Functionname:    MAT_SIGN */
sint8 MAT_SIGN(sint32 Val) {
    sint8 sign;
    sign = 1;
    if (Val < 0) {
        sign = -1;
    }
    return sign;
}

/*************************************************************************************************************************
  Functionname:    MAT_MUL */
sint32 MAT_MUL(sint32 Val1,
               sint32 Val2,
               sint32 Val1Scale,
               sint32 Val2Scale,
               sint32 ReturnScale) {
    sint32 help;
    if ((Val1Scale != 0) && (Val2Scale != 0)) {
        help = Val1 * Val2;
        help /= MAT_MAX(Val1Scale, Val2Scale);
        help *= ReturnScale;
        help /= MAT_MIN(Val1Scale, Val2Scale);
    } else {
        help = 0;
    }

    return help;
}

/*************************************************************************************************************************
  Functionname:    MAT_DIV */
sint32 MAT_DIV(sint32 Val1,
               sint32 Val2,
               sint32 Val1Scale,
               sint32 Val2Scale,
               sint32 ReturnScale) {
    sint32 help;
    if ((Val1Scale != 0) && (Val2 != 0)) {
        help = Val1;
        help *= MAT_MIN(Val2Scale, ReturnScale);
        help /= MAT_MAX(Val2, Val1Scale);
        help *= MAT_MAX(Val2Scale, ReturnScale);
        help /= MAT_MIN(Val2, Val1Scale);
    } else {
        help = 0;
    }
    return help;
}

/*************************************************************************************************************************
  Functionname:    MAT_SIN */
sint16 MAT_SIN(sint32 Val) {
    sint16 return_value;
    uint32 u_val, u_val2;
    sint32 MVal;

    if (Val >= 0) {
        u_val = (uint32)Val;
        if (u_val >= ((uint32)2 * (uint32)Pi_fixpoint_angle)) {
            u_val %= (uint32)2 * (uint32)Pi_fixpoint_angle;
        }

        if (u_val > (uint32)Pi_fixpoint_angle) {
            if (u_val > (uint32)Mat_3_2_pi) {
                u_val2 = ((uint32)2 * (uint32)Pi_fixpoint_angle) - u_val;
                return_value = -MAT_CALCULATE_PARAM_VALUE1D(
                    Mat_sin_table, Mat_sin_table_points, (sint16)u_val2);
            } else {
                u_val2 = u_val - (uint32)Pi_fixpoint_angle;
                return_value = -MAT_CALCULATE_PARAM_VALUE1D(
                    Mat_sin_table, Mat_sin_table_points, (sint16)u_val2);
            }
        } else {
            if (u_val > (uint32)Mat_1_2_pi) {
                u_val2 = (uint32)Pi_fixpoint_angle - u_val;
                return_value = MAT_CALCULATE_PARAM_VALUE1D(
                    Mat_sin_table, Mat_sin_table_points, (sint16)u_val2);
            } else {
                return_value = MAT_CALCULATE_PARAM_VALUE1D(
                    Mat_sin_table, Mat_sin_table_points, (sint16)u_val);
            }
        }
    } else {
        MVal = -Val;
        u_val = (uint32)MVal;
        if (u_val >= ((uint32)2 * (uint32)Pi_fixpoint_angle)) {
            u_val %= (uint32)2 * (uint32)Pi_fixpoint_angle;
        }

        if (u_val > (uint32)Pi_fixpoint_angle) {
            if (u_val > (uint32)Mat_3_2_pi) {
                u_val2 = ((uint32)2 * (uint32)Pi_fixpoint_angle) - u_val;
                return_value = MAT_CALCULATE_PARAM_VALUE1D(
                    Mat_sin_table, Mat_sin_table_points, (sint16)u_val2);
            } else {
                u_val2 = u_val - (uint32)Pi_fixpoint_angle;
                return_value = MAT_CALCULATE_PARAM_VALUE1D(
                    Mat_sin_table, Mat_sin_table_points, (sint16)u_val2);
            }
        } else {
            if (u_val > (uint32)Mat_1_2_pi) {
                u_val2 = (uint32)Pi_fixpoint_angle - u_val;
                return_value = -MAT_CALCULATE_PARAM_VALUE1D(
                    Mat_sin_table, Mat_sin_table_points, (sint16)u_val2);
            } else {
                return_value = -MAT_CALCULATE_PARAM_VALUE1D(
                    Mat_sin_table, Mat_sin_table_points, (sint16)u_val);
            }
        }
    }

    return return_value;
}

/*************************************************************************************************************************
  Functionname:    MAT_COS */
sint16 MAT_COS(sint32 Val) { return MAT_SIN(Val + Mat_1_2_pi); }

/*************************************************************************************************************************
  Functionname:    MAT_TAN */
sint16 MAT_TAN(sint32 Val) {
    sint16 sin_, cos_;
    sint16 return_val;

    cos_ = MAT_COS(Val);
    sin_ = MAT_SIN(Val);
    if (cos_ != 0) {
        return_val =
            (sint16)MAT_LIM(((sint32)sin_ * (sint32)Factor_s) / (sint32)cos_,
                            Signed_int16_min, Signed_int16_max);
    } else {
        if (sin_ > 0) {
            return_val = Signed_int16_max;
        } else {
            return_val = Signed_int16_min;
        }
    }
    return return_val;
}

/*************************************************************************************************************************
  Functionname:    MAT_QUANT */
sint32 MAT_QUANT(sint32 Val,
                 sint32 LastVal,
                 sint32 MinVal,
                 sint32 MaxVal,
                 sint32 Res,
                 percentage_t DutyCycle) {
    uint32 hyst;
    sint32 interval, ret_val, help_val;
    sint32 upper, lower;

    if ((Res > (sint32)0) && (MaxVal > MinVal)) {
        interval = (MaxVal - MinVal) / Res;

        hyst = (uint32)interval;
        if (DutyCycle != ((percentage_t)Percentage_s * (percentage_t)50)) {
            hyst *= (uint32)DutyCycle;
            hyst /= (uint32)Scale_100;
            hyst /= (uint32)(Percentage_s);
        } else {
            hyst /= (uint32)(2);
            hyst -= (uint32)(1 * Percentage_s);
        }

        lower = MinVal;
        upper = lower + interval;
        help_val = MAT_LIM(Val, MinVal, MaxVal);

        while ((upper < MaxVal) && (help_val > upper) && (interval != 0)) {
            lower += interval;
            upper += interval;
        }

        if ((help_val > (upper - (sint32)hyst)) && (help_val >= LastVal)) {
            ret_val = upper;
        } else {
            if ((help_val < (lower + (sint32)hyst)) && (help_val <= LastVal)) {
                ret_val = lower;
            } else {
                ret_val = LastVal;
            }
        }
        ret_val = MAT_LIM(ret_val, lower, upper);
    } else {
        ret_val = (sint32)LastVal;
    }

    return ret_val;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */