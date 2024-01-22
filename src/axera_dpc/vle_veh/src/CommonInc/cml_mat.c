

/*****************************************************************************
INCLUDES
*****************************************************************************/
// #define LMURAM2_START_CODE
// #include "Mem_Map.h" 

#include "TM_Global_Types.h"
#include "TM_CML_Types.h"
#include "TM_Base_Mat.h"
#include "TM_Base_Emul.h"

/*****************************************************************************
FUNCTIONS
*****************************************************************************/

/*****************************************************************************
  Functionname:    CML_TrafoMatrix2DInvert                              */
BML_t_TrafoMatrix2D CML_TrafoMatrix2DInvert(BML_t_TrafoMatrix2D MatrixA) {
    BML_t_TrafoMatrix2D Mat;

    Mat.TrafoType = DEFAULT_;
    Mat.f00 = MatrixA.f00;
    Mat.f02 = (-MatrixA.f02 * MatrixA.f00) - (MatrixA.f12 * MatrixA.f10);
    Mat.f10 = -MatrixA.f10;
    Mat.f12 = (MatrixA.f02 * MatrixA.f10) - (MatrixA.f12 * MatrixA.f00);

    return Mat;
}

/*****************************************************************************
  Functionname:    CML_TrafoMatrix2DMult                                */
BML_t_TrafoMatrix2D CML_TrafoMatrix2DMult(BML_t_TrafoMatrix2D MatrixA,
                                          BML_t_TrafoMatrix2D MatrixB) {
    BML_t_TrafoMatrix2D Mat;

    if (MatrixA.TrafoType == TRANSLATION_) {
        if (MatrixB.TrafoType == TRANSLATION_) {
            Mat.TrafoType = MatrixA.TrafoType;
            Mat.f00 = 1.0f;
            Mat.f02 = MatrixA.f02 + MatrixB.f02;
            Mat.f10 = 0.0f;
            Mat.f12 = MatrixA.f12 + MatrixB.f12;
        } else if (MatrixB.TrafoType == ROTATION_) {
            Mat.TrafoType = DEFAULT_;
            Mat.f00 = MatrixB.f00;
            Mat.f02 = MatrixA.f02;
            Mat.f10 = MatrixB.f10;
            Mat.f12 = MatrixA.f12;
        } else {
            Mat.TrafoType = DEFAULT_;
            Mat.f00 = MatrixB.f00;
            Mat.f02 = MatrixA.f02 + MatrixB.f02;
            Mat.f10 = MatrixB.f10;
            Mat.f12 = MatrixA.f12 + MatrixB.f12;
        }
    } else if (MatrixA.TrafoType == ROTATION_) {
        if (MatrixB.TrafoType == TRANSLATION_) {
            Mat.TrafoType = DEFAULT_;
            Mat.f00 = MatrixA.f00;
            Mat.f02 = (MatrixA.f00 * MatrixB.f02) - (MatrixA.f10 * MatrixB.f12);
            Mat.f10 = MatrixA.f10;
            Mat.f12 = (MatrixA.f10 * MatrixB.f02) + (MatrixA.f00 * MatrixB.f12);
        } else if (MatrixB.TrafoType == ROTATION_) {
            Mat.TrafoType = ROTATION_;
            Mat.f00 = (MatrixA.f00 * MatrixB.f00) - (MatrixA.f10 * MatrixB.f10);
            Mat.f02 = 0.0f;
            Mat.f10 = (MatrixA.f00 * MatrixB.f10) + (MatrixA.f10 * MatrixB.f00);
            Mat.f12 = 0.0f;
        } else {
            Mat.TrafoType = DEFAULT_;
            Mat.f00 = (MatrixA.f00 * MatrixB.f00) - (MatrixA.f10 * MatrixB.f10);
            Mat.f02 = (MatrixA.f00 * MatrixB.f02) - (MatrixA.f10 * MatrixB.f12);
            Mat.f10 = (MatrixA.f00 * MatrixB.f10) + (MatrixA.f10 * MatrixB.f00);
            Mat.f12 = (MatrixA.f10 * MatrixB.f02) + (MatrixA.f00 * MatrixB.f12);
        }
    } else {
        if (MatrixB.TrafoType == TRANSLATION_) {
            Mat.TrafoType = DEFAULT_;
            Mat.f00 = MatrixA.f00;
            Mat.f02 =
                ((MatrixA.f00 * MatrixB.f02) - (MatrixA.f10 * MatrixB.f12)) +
                MatrixA.f02;
            Mat.f10 = MatrixA.f10;
            Mat.f12 =
                ((MatrixA.f10 * MatrixB.f02) + (MatrixA.f00 * MatrixB.f12)) +
                MatrixA.f12;
        } else if (MatrixB.TrafoType == ROTATION_) {
            Mat.TrafoType = DEFAULT_;
            Mat.f00 = (MatrixA.f00 * MatrixB.f00) - (MatrixA.f10 * MatrixB.f10);
            Mat.f02 = MatrixA.f02;
            Mat.f10 = (MatrixA.f00 * MatrixB.f10) + (MatrixA.f10 * MatrixB.f00);
            Mat.f12 = MatrixA.f12;
        } else {
            Mat.TrafoType = DEFAULT_;
            Mat.f00 = (MatrixA.f00 * MatrixB.f00) - (MatrixA.f10 * MatrixB.f10);
            Mat.f02 =
                ((MatrixA.f00 * MatrixB.f02) - (MatrixA.f10 * MatrixB.f12)) +
                MatrixA.f02;
            Mat.f10 = (MatrixA.f00 * MatrixB.f10) + (MatrixA.f10 * MatrixB.f00);
            Mat.f12 =
                ((MatrixA.f10 * MatrixB.f02) + (MatrixA.f00 * MatrixB.f12)) +
                MatrixA.f12;
        }
    }
    return Mat;
}

/*****************************************************************************
  Functionname:    BML_v_TransformPosition2D                            */ /*!

  @brief           transform coordinates using transformation matrix M

  @description     This function calculates the transform coordinates using 
                   transformation matrix M. Let the transform cordinates be 
                   calculated for x,y with the two rows of transformation matrix,
                   M = |f00  f01 f02|
                       |f10  f11 f12|,
                   then new cordinates will be
                   x1 = ( x * f00 - y * f10 ) + f02
                   y1 = ( x * f10 + y * f00 ) + f12

  @param[in]       p_Matrix :  transformation matrix
                              Range for p_Matrix.f00 is [0,..,1]
                              Range for p_Matrix.f02 is [Full range of float32]
                              Range for p_Matrix.f10 is [0,..,1]
                              Range for p_Matrix.f12 is [Full range of float32]
  @param[in,out]   p_DistX :  X-coordinate
                              [Full range of float32]
  @param[in,out]   p_DistY :  Y-coordinate
                              [Full range of float32]
                              Overflow may occur when one or more input values 
                              are at the defined range extremities.

  @return          void


*****************************************************************************/
void BML_v_TransformPosition2D(BML_t_TrafoMatrix2D const *p_Matrix,
                               float32 *p_DistX,
                               float32 *p_DistY) {
    float32 f_DistanceY = *p_DistY;

    *p_DistY = (((*p_DistX * p_Matrix->f10) + (f_DistanceY * p_Matrix->f00)) +
                p_Matrix->f12);
    *p_DistX = (((*p_DistX * p_Matrix->f00) - (f_DistanceY * p_Matrix->f10)) +
                p_Matrix->f02);
}

/*****************************************************************************
  Functionname:    CML_v_TransformXPosition2D                           */ /*!

  @brief           transform X-coordinate using transformation matrix M

  @description     This function does the transform of X-coordinate using 
                   transformation matrix, and is calculated depending on
                   the matrix type: Translation, Rotation or Default.
                   Let the transform cordinate be calculated for x,y 
                   with the two rows of transformation matrix,
                   M = |f00  f01 f02|
                       |f10  f11 f12|,
                   then new cordinate will be
                   x1 = ( x * f00 - y * f10 ) + f02

  @param[in]       p_Matrix :  transformation matrix
                              Supported values for p_Matrix.TrafoType are 0,1,2.
                              Range for p_Matrix.f00 is [0,..,1]
                              Range for p_Matrix.f02 is [Full range of float32]
                              Range for p_Matrix.f10 is [0,..,1]
                              Range for p_Matrix.f12 is [Full range of float32]
  @param[in]       f_DistX :  X-coordinate
                              [Full range of float32]
  @param[in]       f_DistY :  Y-coordinate
                              [Full range of float32]
                              Overflow may occur when one or more input values 
                              are at the defined range extremities.

  @return          transformed X-coordinate


*****************************************************************************/
float32 BML_f_TransformXPosition2D(BML_t_TrafoMatrix2D const *p_Matrix,
                                   float32 f_DistX,
                                   float32 f_DistY) {
    float32 f_ret;

    if (p_Matrix->TrafoType == TRANSLATION_) {
        f_ret = f_DistX + p_Matrix->f02;
    } else if (p_Matrix->TrafoType == ROTATION_) {
        f_ret = ((f_DistX * p_Matrix->f00) - (f_DistY * p_Matrix->f10));
    } else {
        f_ret = (((f_DistX * p_Matrix->f00) - (f_DistY * p_Matrix->f10)) +
                 p_Matrix->f02);
    }
    return f_ret;
}

/*****************************************************************************
  Functionname:    CML_v_TransformYPosition2D                           */ /*!

  @brief           transform Y-coordinate using transformation matrix M

  @description     This function does the transform of Y-coordinate using 
                   transformation matrix, and is calculated depending on
                   the matrix type: Translation, Rotation or Default.
                   Let the transform cordinate be calculated for x,y 
                   with the two rows of transformation matrix,
                   M = |f00  f01 f02|
                       |f10  f11 f12|,
                   then new cordinate will be
                   y1 = ( x * f10 + y * f00 ) + f12

  @param[in]       p_Matrix :  transformation matrix
                              Supported values for p_Matrix.TrafoType are 0,1,2.
                              Range for p_Matrix.f00 is [0,..,1]
                              Range for p_Matrix.f02 is [Full range of float32]
                              Range for p_Matrix.f10 is [0,..,1]
                              Range for p_Matrix.f12 is [Full range of float32]
  @param[in]       f_DistX :  X-coordinate
                              [Full range of float32]
  @param[in]       f_DistY :  Y-coordinate
                              [Full range of float32]
                              Overflow may occur when one or more input values 
                              are at the defined range extremities.

  @return          transformed Y-coordinate


*****************************************************************************/
float32 BML_f_TransformYPosition2D(BML_t_TrafoMatrix2D const *p_Matrix,
                                   float32 f_DistX,
                                   float32 f_DistY) {
    float32 f_ret;

    if (p_Matrix->TrafoType == TRANSLATION_) {
        f_ret = f_DistY + p_Matrix->f12;
    } else if (p_Matrix->TrafoType == ROTATION_) {
        f_ret = ((f_DistX * p_Matrix->f10) + (f_DistY * p_Matrix->f00));
    } else {
        f_ret = (((f_DistX * p_Matrix->f10) + (f_DistY * p_Matrix->f00)) +
                 p_Matrix->f12);
    }
    return f_ret;
}

/*****************************************************************************
  Functionname:    BML_GetTrafoMatrixByAngle                            */
BML_t_TrafoMatrix2D BML_GetTrafoMatrixByAngle(float32 f_Angle) {
    BML_t_TrafoMatrix2D TrafoMat;

    TrafoMat.TrafoType = ROTATION_;
    GDBsincos(f_Angle, &TrafoMat.f10, &TrafoMat.f00);
    TrafoMat.f02 = 0.0f;
    TrafoMat.f12 = 0.0f;
    return TrafoMat;
}

/*****************************************************************************
  Functionname:    BML_GetTrafoMatrixByDisplacEnvment                     */ /*!

  @brief           Get translation matrix with given displacement

  @description     Get translation matrix with given displacement
                   The function prepares a translation matrix with given 
                   displacement. If the displacement is 
                   (f_XDisplacement, f_YDisplacement), then the translation
                   matrix would be as follows:
                   |1    0    f_XDisplacement|
                   |0    1    f_YDisplacement|
                   |0    0           1       |

  @param[in]       f_XDisplacement :  X-displacement
                                      [Full range of float32]
  @param[in]       f_YDisplacement :  Y-displacement
                                      [Full range of float32]

  @return          translation matrix with given displacement


*****************************************************************************/
BML_t_TrafoMatrix2D BML_GetTrafoMatrixByDisplacEnvment(
    float32 f_XDisplacement, float32 f_YDisplacement) {
    BML_t_TrafoMatrix2D TrafoMat;

    TrafoMat.TrafoType = TRANSLATION_;
    TrafoMat.f00 = 1.0f;
    TrafoMat.f10 = 0.0f;
    TrafoMat.f02 = f_XDisplacement;
    TrafoMat.f12 = f_YDisplacement;
    return TrafoMat;
}

/*****************************************************************************
  Functionname:    BML_u_MatrixCopy                                     */ /*!

  @brief           Copy a given matrix

  @description     This function copies the contents of one matrix to another
                   if the number of elements is non-zero. The function will 
                   always return TRUE.

  @param[in]       a_MatrixA :    original matrix
                                  [Full range of float32]
  @param[out]      a_MatrixCpy :  copied matrix
                                  [Full range of float32]
  @param[in]       u_dim_Ax :     x-dimension of matrix A
                                  [Full range of uint8]
  @param[in]       u_dim_Ay :     y-dimension of matrix A
                                  [Full range of uint8]

  @return          true

*****************************************************************************/
uint8 BML_u_MatrixCopy(float32 const a_MatrixA[],
                       float32 a_MatrixCpy[],
                       uint8 u_dim_Ax,
                       uint8 u_dim_Ay) {
    uint16 n;
    uint16 p;
    n = ((uint16)u_dim_Ay * (uint16)u_dim_Ax);
    p = 0u;
    while (n > 0UL) {
        n--;
        a_MatrixCpy[p] = a_MatrixA[p];
        p++;
    }
    return TRUE; /*ever*/
}

/*****************************************************************************
  Functionname:    CML_u_MatrixTranspose                                */
uint8 CML_u_MatrixTranspose(float32 const a_MatrixA[],
                            float32 a_ATranspose[],
                            uint8 u_dim_Ax,
                            uint8 u_dim_Ay) {
    uint8 u_ret;
    uint8 u_Idx2, u_Idx1;
    uint16 u_pos1, u_pos2, u_tmp;
    float32 f_save;
    u_ret = TRUE;

    if (u_dim_Ax == u_dim_Ay) {
        /*square matrix*/
        if (a_ATranspose !=
            a_MatrixA) /*in case of different pointers: copy matrix first*/
        {
            u_ret =
                BML_u_MatrixCopy(a_MatrixA, a_ATranspose, u_dim_Ax, u_dim_Ay);
        }

        for (u_Idx1 = 0u; u_Idx1 < u_dim_Ay; u_Idx1++) {
            const uint16 c1 = (uint16)u_Idx1 * (uint16)u_dim_Ax;
            for (u_Idx2 = u_Idx1 + 1u; u_Idx2 < u_dim_Ax; u_Idx2++) {
                u_pos1 = (uint16)(c1 + u_Idx2);
                u_tmp = ((uint16)u_Idx2 * (uint16)u_dim_Ax);
                u_tmp = u_tmp + u_Idx1;
                u_pos2 =
                    (uint16)u_tmp; /*position of the element to switch with*/
                /*switch values*/
                f_save = a_MatrixA[u_pos2]; /*save in case of A==a_ATransp*/
                a_ATranspose[u_pos2] = a_MatrixA[u_pos1];
                a_ATranspose[u_pos1] = f_save;
            }
        }
    } else {
        if (a_MatrixA == a_ATranspose) /*it's not possible to transpose the
                                          matrix to its own*/
        {
            u_ret = FALSE;
        } else {
            u_pos1 = 0u;
            for (u_Idx1 = 0u; u_Idx1 < u_dim_Ax; u_Idx1++) {
                for (u_Idx2 = 0u; u_Idx2 < u_dim_Ay; u_Idx2++) {
                    u_tmp = (uint16)u_Idx2 * (uint16)u_dim_Ax;
                    u_tmp = u_tmp + u_Idx1;
                    u_pos2 = (uint16)
                        u_tmp; /*position of the element to switch with*/
                    /*switch values*/
                    a_ATranspose[u_pos2] = a_MatrixA[u_pos1];
                    u_pos1++;
                }
            }
        }
    }
    return u_ret;
}

/*****************************************************************************
  Functionname:    BML_u_MatrixMultiplication                           */ /*!

  @brief           multiplies two matrices A and B

  @description     This function multiplies two matrices A and B (AxB)
                   NOTE: The function expects the following matrix dimensions:
                   number of rows and number of columns of the first matrix A,
                   and just number of columns of second matrix B. 
                   The function assumes the prerequisite for matrix multiplication,
                   i.e. no. of columns of A = no. of rows of B, is satisfied.

  @param[in]     a_MatrixA :  first matrix
                              [Full range of float32]
                              Overflow may occur when input values 
                              are at the defined range extremities.
  @param[in]     a_MatrixB :  second matrix
                              [Full range of float32]
                              Overflow may occur when input values 
                              are at the defined range extremities.
  @param[in]     u_dim_Ax :   x-dimension of A
                              [Full range of uint8]
  @param[in]     u_dim_Ay :   y-dimension of A
                              [Full range of uint8]
  @param[in]     u_dim_Bx :   x-dimension of B
                              [Full range of uint8]
  @param[out]    a_AxB :      product A*B
                              [Full range of float32]

  @return        true if operation was successful


*****************************************************************************/
uint8 BML_u_MatrixMultiplication(float32 const a_MatrixA[],
                                 float32 const a_MatrixB[],
                                 float32 a_AxB[],
                                 uint8 u_dim_Ax,
                                 uint8 u_dim_Ay,
                                 uint8 u_dim_Bx) {
    uint8 u_ret;
    uint8 i, j, k;
    float32 f_save;

    u_ret = TRUE;
    if ((a_AxB != a_MatrixA) && (a_AxB != a_MatrixB)) {
        for (i = 0u; i < u_dim_Ay; i++) {
            const uint16 colA = (uint16)i * (uint16)u_dim_Ax;
            const uint16 colAxB = (uint16)i * (uint16)u_dim_Bx;
            for (j = 0u; j < u_dim_Bx; j++) {
                f_save = 0.f;
                for (k = 0u; k < u_dim_Ax; k++) {
                    f_save +=
                        a_MatrixA[colA + k] * a_MatrixB[j + (k * u_dim_Bx)];
                }
                a_AxB[j + colAxB] = f_save;
            }
        }
    } else {
        u_ret = FALSE;
    }

    return u_ret;
}

void BML_u_MatrixInversionRoateRows(uint8 u_row,
                                    uint8 u_prow,
                                    uint8 u_dim_xy,
                                    float32 a_tempMatrix[],
                                    float32 a_InverseA[]) {
    uint8 j;
    if (u_prow != u_row) /*rotate rows*/
    {
        float32 help;
        for (j = 0u; j < u_dim_xy; j++) {
            help = a_InverseA[(u_row * u_dim_xy) + j];
            a_InverseA[(u_row * u_dim_xy) + j] =
                a_InverseA[(u_prow * u_dim_xy) + j];
            a_InverseA[(u_prow * u_dim_xy) + j] = help;

            if (j >= u_row) {
                help = a_tempMatrix[(u_row * u_dim_xy) + j];
                a_tempMatrix[(u_row * u_dim_xy) + j] =
                    a_tempMatrix[(u_prow * u_dim_xy) + j];
                a_tempMatrix[(u_prow * u_dim_xy) + j] = help;
            }
        }
    }
}

uint8 BML_u_MatrixInversion(float32 const a_MatrixA[],
                            float32 a_tempMatrix[],
                            float32 a_InverseA[],
                            uint8 u_dim_xy) {
    uint8 u_ret;
    uint8 i, j;
    uint8 u_row, u_prow;
    uint16 u_it_row;
    float32 max_val, fak;
    const float32 f_e = 0.00001F;

    u_ret = BML_u_MatrixCopy(
        a_MatrixA, a_tempMatrix, u_dim_xy,
        u_dim_xy); /*copy to destination because we want to work in it*/

    /*create identity matrix*/
    for (i = 0u; i < u_dim_xy; i++) {
        const uint16 p = (uint16)i * (uint16)u_dim_xy;
        for (j = 0u; j < u_dim_xy; j++) {
            a_InverseA[p + j] = (i == j ? 1.f : 0.f);
            //     a_InverseA[p+j] = 0.f;
            //     if (i == j)
            //     {
            //         a_InverseA[p+j] = 1.f;
            //     }
        }
    }

    /*start iteration*/
    u_row = 0u;
    do {
        u_ret = FALSE;

        max_val = BML_f_Abs(a_tempMatrix[u_row + (u_row * u_dim_xy)]);

        u_prow = u_row;
        for (i = u_row + 1u; i < u_dim_xy; i++) {
            if (BML_f_Abs(a_tempMatrix[(i * u_dim_xy) + u_row]) > max_val) {
                max_val = BML_f_Abs(a_tempMatrix[(i * u_dim_xy) + u_row]);
                u_prow = i;
            }
        }

        if (max_val >= f_e) {
            BML_u_MatrixInversionRoateRows(u_row, u_prow, u_dim_xy,
                                           a_tempMatrix, a_InverseA);

            /*elimination: division by a_Ai[it,it]*/
            u_it_row = (uint16)u_row * (uint16)u_dim_xy;
            fak = a_tempMatrix[u_row + u_it_row];
            for (j = 0u; j < u_dim_xy; j++) {
                if (j >= u_row) {
                    a_tempMatrix[u_it_row + j] /= fak;
                }
                a_InverseA[u_it_row + j] /= fak;
            }

            /*further elimination*/
            for (i = 0u; i < u_dim_xy; i++) {
                if (i != u_row) {
                    fak = -a_tempMatrix[(i * u_dim_xy) + u_row];
                    for (j = 0u; j < u_row; j++) {
                        a_InverseA[(i * u_dim_xy) + j] +=
                            fak * a_InverseA[(u_row * u_dim_xy) + j];
                    }

                    for (j = u_row; j < u_dim_xy; j++) {
                        a_tempMatrix[(i * u_dim_xy) + j] +=
                            fak * a_tempMatrix[(u_row * u_dim_xy) + j];
                        a_InverseA[(i * u_dim_xy) + j] +=
                            fak * a_InverseA[(u_row * u_dim_xy) + j];
                    }
                }
            }

            u_row++;
            u_ret = TRUE;
        }
    } while ((u_ret == TRUE) && (u_row < u_dim_xy));

    return u_ret;
}

/*****************************************************************************
  Functionname:    CML_u_MatrixAddition                                 */ /*!

  @brief           addition of two matrices A and B

  @description     This function performs addition of two matrices A and B.
                   The function assumes both matrices are of same order and 
                   hence accepts only one set of dimension.

  @param[in]       a_MatrixA :  first matrix (dim_x*dim_y)
                                [Full range of float32]
                                Overflow may occur when input values 
                                are at the defined range extremities.
  @param[in]       a_MatrixB :  second matrix (dim_x*dim_y)
                                [Full range of float32]
                                Overflow may occur when input values 
                                are at the defined range extremities.
  @param[out]      a_AplusB :   the sum A+B
                                [Full range of float32]
  @param[in]       u_dim_x :    x-dimension of A and B
                                [Full range of uint8]
  @param[in]       u_dim_y :    y-dimension of A and B
                                [Full range of uint8]

  @return          true


*****************************************************************************/
uint8 CML_u_MatrixAddition(float32 const a_MatrixA[],
                           float32 const a_MatrixB[],
                           float32 a_AplusB[],
                           uint8 u_dim_x,
                           uint8 u_dim_y) {
    uint8 u_Index = 0u;
    uint16 u_NumEntries = (uint16)u_dim_x * (uint16)u_dim_y;
    while (u_NumEntries > 0UL) {
        u_NumEntries--;
        a_AplusB[u_Index] = a_MatrixA[u_Index] + a_MatrixB[u_Index];
        u_Index++;
    }
    return TRUE; /*always*/
}

/*****************************************************************************
  Functionname:    BML_u_MatrixSubtraction                              */ /*!

  @brief           subtraction of two matrices A and B

  @description     This function performs subtraction of two matrices A and B.
                   The function assumes both matrices are of same order and 
                   hence accepts only one set of dimension.

  @param[in]       a_MatrixA :  first matrix (dim_x*dim_y)
                                [Full range of float32]
                                Overflow may occur when input values 
                                are at the defined range extremities.
  @param[in]       a_MatrixB :  second matrix (dim_x*dim_y)
                                [Full range of float32]
                                Overflow may occur when input values 
                                are at the defined range extremities.
  @param[out]      a_AminusB :  the difference A-B
                                [Full range of float32]
  @param[in]       u_dim_x :    x-dimension of A and B
                                [Full range of uint8]
  @param[in]       u_dim_y :    y-dimension of A and B
                                [Full range of uint8]

  @return          true


*****************************************************************************/
uint8 BML_u_MatrixSubtraction(float32 const a_MatrixA[],
                              float32 const a_MatrixB[],
                              float32 a_AminusB[],
                              uint8 u_dim_x,
                              uint8 u_dim_y) {
    uint16 u_Index = 0u;
    uint16 u_NumEntries = (uint16)u_dim_x * (uint16)u_dim_y;
    while (u_NumEntries > 0UL) {
        u_NumEntries--;
        a_AminusB[u_Index] = a_MatrixA[u_Index] - a_MatrixB[u_Index];
        u_Index++;
    }
    return TRUE; /*always*/
}

// #define LMURAM2_STOP_CODE
// #include "Mem_Map.h"