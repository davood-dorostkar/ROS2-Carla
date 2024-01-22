/*! \file **********************************************************************

  COMPONENT:              MAT (math functions)

  MODULENAME:             mat_param.c

  @brief                  This module contains all functions for getting a value
  from a param table


  ---*/

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_START_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */

//#include "vlc_glob_ext.h"
//#include "vlcSen_ext.h"
#include "TM_Global_Types.h"
#include "mat_param_ext.h"
#include "mat_std_ext.h"

#define Pt_scale 1024u

/*************************************************************************************************************************
  Functionname:    MAT_CALCULATE_PARAM_VALUE1D */
sint16 MAT_CALCULATE_PARAM_VALUE1D(const sint16 table[],
                                   const uint16 num,
                                   const sint16 x) {
    uint16 n;
    uint16 table_index = 0u;
    sint16 y;
    sint16 x2;
    sint16 y2;
    sint16 x1;
    sint16 y1;

    /* find range on x-axle */
    n = 1u;
    while ((n < num) && (x > table[table_index])) {
        table_index += 2u;
        n += 1u;
    }

    /* calculate y */
    if (n != 1u) {
        /*take actual table position and previous table position for
         * interpolation*/
        x1 = table[table_index - 2u];
        y1 = table[table_index - 1u];

        x2 = table[table_index];
        y2 = table[table_index + 1u];

        if ((x < x2) && (x1 < x2)) {
            /*interpolation*/
            y = (sint16)(((((sint32)x - (sint32)x1) *
                           ((sint32)y2 - (sint32)y1)) /
                          ((sint32)x2 - (sint32)x1)) +
                         (sint32)y1);
        } else {
            /*if x > actual (last) table position use last y value from table*/
            y = y2;
        }
    } else {
        /*if x was smaller than first value in table --> take first y value from
         * table*/
        y = table[table_index + 1u];
    }

    return y;
}

/*************************************************************************************************************************
  Functionname:    MAT_CALCULATE_PARAM_VALUE2D */
float32 MAT_CALCULATE_PARAM_VALUE2D(const float32 z_table[],
                                    const float32 x_table[],
                                    const float32 y_table[],
                                    const uint16 numx,
                                    const uint16 numy,
                                    float32 x,
                                    float32 y) {
    // test x being in xrange
    if (x < x_table[0]) {
        x = x_table[0];
    } else if (x > x_table[numx - 1]) {
        x = x_table[numx - 1];
    }

    // test y being in yrange
    if (y < y_table[0]) {
        y = y_table[0];
    } else if (y > y_table[numy - 1]) {
        y = y_table[numy - 1];
    }

    // get col/row of actual (x,y)
    uint8 i = 0, j = 0;
    for (i = 0; i < numx - 1; i++) {
        uint8 end_flag = 0;
        if (x >= x_table[i] && x <= x_table[i + 1]) {
            for (j = 0; j < numy - 1; j++) {
                if (y >= y_table[j] && y <= y_table[j + 1]) {
                    end_flag = 1;
                    break;
                }
            }
            if (end_flag == 1) {
                break;
            }
        } else {
            continue;
        }
    }

    // get the confirmed 2 points position
    float32 x1, x2, x3;
    float32 y1, y2, y3;
    float32 z1, z2, z3;
    x1 = x_table[i + 1];
    y1 = y_table[j];
    z1 = z_table[(i + 1) * numy + j];

    x2 = x_table[i];
    y2 = y_table[j + 1];
    z2 = z_table[i * numy + j + 1];

    // choose the 3rd point acording the distance between (x,y) and (x(i),y(i))
    // or (x(i+1),y(i+1)), which is closer.
    if (x - x_table[i] + y - y_table[j] <=
        (x_table[i + 1] - x_table[i] + y_table[j + 1] - y_table[j]) / 2) {
        x3 = x_table[i];
        y3 = y_table[j];
        z3 = z_table[i * numy + j];
    } else {
        x3 = x_table[i + 1];
        y3 = y_table[j + 1];
        z3 = z_table[(i + 1) * numy + j + 1];
    }

    // calculate the coefficients of plane geometry equation
    // a*x1+b*y1+c*z1+d=0
    // a*x2+b*y2+c*z2+d=0
    // a*x3+b*y3+c*z3+d=0
    float32 coef_a, coef_b, coef_c, coef_d;
    coef_a = y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2);
    coef_b = z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2);
    coef_c = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
    coef_d = -x1 * (y2 * z3 - y3 * z2) - x2 * (y3 * z1 - y1 * z3) -
             x3 * (y1 * z2 - y2 * z1);

    // get the z of (x,y)
    float32 z;
    if (coef_c != 0) {
        z = (-coef_a * x - coef_b * y - coef_d) / coef_c;
    } else {
        z = 0;
    }

    return z;
}

/* **********************************************************************
Autosar Memory Map
**************************************************************************** */
// #define CORE1_MODULE_STOP_CODE
// #include "Mem_Map.h" /* PRQA S 5087 */ /* MD_MSR_MemMap */