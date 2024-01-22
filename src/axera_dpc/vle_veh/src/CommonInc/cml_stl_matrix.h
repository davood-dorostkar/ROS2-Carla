    

#ifndef CML_STL_MATRIX_H__
#define CML_STL_MATRIX_H__

#include "cml_stl_array2d.h"
#include "cml_stl_algorithm.h"
#include <limits>

namespace cml
{
    /// CMatrix is a CArray2D providing additional math operations for the Matrix.
    /// @tparam T       The type of the matrix elements
    /// @tparam ROWS    The number of rows of the matrix
    /// @tparam COLS    The number of columns of the matrix
    /// @tparam EXT_MEM Flag whether the memory of the elements of the matrix is inside
    ///                 of this class (false) or outside of this class (true).
    ///                 Default is false, meaning internal memory.
    template < class T, sint32 ROWS, sint32 COLS, boolean_t EXT_MEM = false >
    class CMatrix
        : public CArray2D<T, ROWS, COLS, EXT_MEM>
    {
    public:
        typedef CMatrix <T, ROWS, COLS, EXT_MEM> self;          ///< Abbreviation for the type of the class.
        typedef CMatrix <T, ROWS, COLS>          self_internal; ///< Abbreviation for the type of the class but with internal memory.
        typedef CArray2D<T, ROWS, COLS, EXT_MEM> super;         ///< Base class of this CMatrix template configuration.
        typedef typename super::super            super_super;
        typedef sint32                           size_type;
        typedef typename remove_const<T>::type   type;
        typedef typename    add_const<T>::type   const_type;
        typedef typename remove_const<T>::type&  reference;
        typedef typename    add_const<T>::type&  const_reference;
        typedef typename remove_const<T>::type*  pointer;
        typedef typename    add_const<T>::type*  const_pointer;
        typedef int                              difference_type;

        // iterator typedefs
        typedef typename super::it                        rowIt;
        typedef typename super::cIt                       cRowIt;
#ifndef NDEBUG
        typedef typename cml::CLinMemIt< type, false, super_super, COLS >  colIt;
        typedef typename cml::CLinMemIt< type, true , super_super, COLS > cColIt;
#else
        typedef typename cml::CLinMemIt< type, false, COLS >  colIt;
        typedef typename cml::CLinMemIt< type, true , COLS > cColIt;
#endif

        // This is only required for TI-Compiler to be able to compile
        using super::SIZE;
        // End of section required for TI-Compiler

        // Redefinition of constructors since they are not inherited
        CMatrix(){}

        /// Constructor with pointer to a buffer and size.
        /// @param[in] data Pointer to constant data
        /// @param[in] size Constant size of the buffer
        CMatrix(       pointer const data, const sint32 size ) : super(data, size) {}

        /// Constructor with pointer to constant data.
        /// @param[in] data Constant pointer to constant data
        /// @param[in] size Constant size of the buffer
        CMatrix( const_pointer const data, const sint32 size ) : super(data, size) {}

        /// Constructor with reference to matrix of possibly different size than this.
        /// @tparam    SIZE2   The size of the external matrix
        /// @param[in] data    Reference to data of SIZE2
        template< sint32 SIZE2 >
        CMatrix( type (&data)[SIZE2] ) : super(data) {}

        /// Constructor with constant reference to matrix of possibly different size than this.
        /// @tparam    SIZE2   The size of the external matrix
        /// @param[in] data    Constant reference to data of SIZE2
        template< sint32 SIZE2 >
        CMatrix( const_type (&data)[SIZE2] ) : super(data) {}

        /// Constructor with reference to constant CArrayStorage object of non constant data
        /// and possibly different size and external memory setting.
        /// @tparam    SIZE2      The size of CArrayStorage
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CArrayStorage object of SIZE2		
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CMatrix( const CArrayStorage<type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Constructor with reference to constant CArrayStorage object of constant data
        /// and possibly different size and external memory setting.
        /// @tparam    SIZE2      The size of CArrayStorage
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CArrayStorage object of SIZE2		
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CMatrix( const CArrayStorage<const_type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Constructor with reference to CArrayStorage object of non constant data
        /// and possibly different size and external memory setting.
        /// @tparam    SIZE2      The size of CArrayStorage
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Reference to CArrayStorage object of SIZE2	
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CMatrix( CArrayStorage<type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Constructor with reference to CArrayStorage object of constant data
        /// and possibly different size and external memory setting.
        /// @tparam    SIZE2      The size of CArrayStorage
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Reference to CArrayStorage object of SIZE2 with constant data	
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CMatrix( CArrayStorage<const_type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        // This is only required for TI-Compiler to be able to compile
        using super::operator[];
        using super::operator();
        using super::fill;
        // End of section required for TI-Compiler

        /// Copy the external CMatrix content to the buffer of this
		/// [this] = [B]
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size ROWS X COLS	
        /// @return               A reference to this matrix.
        template< typename TYPE2, boolean_t EXT_MEM2 >
        CMatrix& operator=(const CMatrix<TYPE2, ROWS, COLS, EXT_MEM2>& rhs)
        {
            super::operator=(rhs);
            return *this;
        }

        /// Add the external CMatrix content to the buffer of this
		/// Perform matrix addition (inplace) of two matrices, this and B
        /// with same dimensions and store the result in this matrix. 
        /// [this] += [B]
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size ROWS X COLS	
        /// @return               A reference to this matrix.
        template< typename TYPE2, boolean_t EXT_MEM2 >
        CMatrix& operator+=(const CMatrix<TYPE2, ROWS, COLS, EXT_MEM2>& rhs)
        {
            for (sint32 i = 0; i < SIZE; ++i)
            {
                operator[](i) += rhs[i];
            }
            return *this;
        }

        /// Add the external CMatrix content to the buffer of this without allocating result to this.		
		/// Perform matrix addition (outplace) of two matrices, A and B
        /// with same dimensions and store the result in resultant matrix. 
        /// [Res] = [A] + [B]
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size ROWS X COLS	
        /// @return               A reference to the resultant matrix of size ROWS * COLS
        template< typename TYPE2, boolean_t EXT_MEM2 >
        CMatrix<type, ROWS, COLS, false> operator+(const CMatrix<TYPE2, ROWS, COLS, EXT_MEM2>& rhs) const
        {
            return CMatrix<type, ROWS, COLS, false>(*this).operator+=(rhs);
        }

        /// Subtract the external CMatrix content from the buffer of this
		/// Perform matrix subtraction (inplace) of two matrices, this and B
        /// with same dimensions and store the result in this matrix. 
        /// [this] -= [B]
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size ROWS X COLS	
        /// @return               A reference to this matrix.
        template< typename TYPE2, boolean_t EXT_MEM2 >
        CMatrix& operator-=(const CMatrix<TYPE2, ROWS, COLS, EXT_MEM2>& rhs)
        {
            for (sint32 i = 0; i < SIZE; ++i)
            {
                operator[](i) -= rhs[i];
            }
            return *this;
        }

        /// Subtract the external CMatrix content from the buffer of this without allocating result to this.
		/// Perform matrix subtraction (outplace) of two matrices, A and B
        /// with same dimensions and store the result in resultant matrix. 
        /// [Res] = [A] - [B]
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size ROWS X COLS	
        /// @return               A reference to the resultant matrix of size ROWS * COLS
        template< typename TYPE2, boolean_t EXT_MEM2 >
        CMatrix<type, ROWS, COLS,false> operator-(const CMatrix<TYPE2, ROWS, COLS, EXT_MEM2>& rhs) const
        {
            return CMatrix<type, ROWS, COLS, false>(*this).operator-=(rhs);
        }

        /// Negate the matrix without allocating result to this.	
		/// Perform negation (outplace) of a matrix B and store the result 
        /// in resultant matrix. 
        /// [Res] = -[B]
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @return               A reference to the resultant matrix of size ROWS * COLS
        //template< boolean_t EXT_MEM2 >
        CMatrix<type, ROWS, COLS,false> operator-() const
        {
            CMatrix<type, ROWS, COLS,false> result;
            for (sint32 i = 0; i < SIZE; ++i)
            {
                result[i] = -(operator[](i));
            }
            return result;
        }

        /// Multiply the external CMatrix content to the buffer of this.
		/// Perform matrix multiplication (inplace) of two matrices, this and B
        /// and store the result in this matrix. 
        /// [this] *= [B]
		/// Matrix dimensions must be ROWS X COLS, COLS X COLS!
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size COLS X COLS	
        /// @return               A reference to this matrix.
        template< typename TYPE2, boolean_t EXT_MEM2 >
        CMatrix& operator*= ( const CMatrix<TYPE2, COLS, COLS, EXT_MEM2>& rhs )
        {
            *this = (*this) * rhs;
            return *this;
        }

        /// Multiply the external CMatrix content to the buffer of this without allocating result to this.
		/// Perform matrix multiplication (outplace) of two matrices, A and B
        /// and store the result in resultant matrix. 
        /// [Res] = [A] * [B]
		/// Matrix dimensions must be ROWS X COLS, COLS X COLS2!
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size COLS X COLS2	
        /// @return               A reference to the resultant matrix of size ROWS * COLS2
        template< typename TYPE2, sint32 COLS2, boolean_t EXT_MEM2>
        CMatrix<type, ROWS, COLS2, false> operator* ( const CMatrix<TYPE2, COLS, COLS2, EXT_MEM2>& rhs ) const
        {
            CMatrix<type, ROWS, COLS2, false> result;

            size_type rc1 = 0;
            size_type rc2 = 0;

            for (size_type i = 0; i < ROWS; i++)
            {
                for (size_type j = 0; j < COLS2; j++)
                {
                    type resultElement = 0;
                    size_type nc2c = j;

                    for (size_type k = 0; k < COLS; k++)
                    {
                        resultElement += operator[](rc1 + k) * rhs[nc2c];

                        nc2c += COLS2;
                    }

                    result[rc2 + j] = resultElement;
                }
                rc2 += COLS2;
                rc1 += COLS;
            }

#if !defined NDEBUG && !defined (CML_TEST)
            /* C Implementation */
            CMatrix<type, ROWS, COLS2, false> result_CVersion;
            BML_t_Matrix resMatrixImpl = result_CVersion.getCMLMatrix();
            const BML_t_Matrix lhsMatrixImpl = getCMLMatrix();
            const BML_t_Matrix rhsMatrixImpl = rhs.getCMLMatrix();
            CML_v_MultiplyMatrices( &resMatrixImpl, &lhsMatrixImpl, &rhsMatrixImpl );

            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }

        /// @name Elementary Arithmetic with a scalar.
        /// @{

        /// Multiply all elements of the matrix by a scalar.
		/// Multiply all elements of the matrix (inplace) by a given scalar value. 
		/// [this] = rhs * [this]
        /// @param[in] rhs Scalar value to multiply.
        /// @return        A reference to this matrix.
        self& operator*= (const_reference rhs)
        {
            for (sint32 i = 0; i < SIZE; ++i)
            {
                operator[](i) *= rhs;
            }
            return *this;
        }

        /// Divide all elements of the matrix by a scalar.
		/// Divide all elements of the matrix (inplace) by a given scalar value. 
		/// [this] = [this] / rhs
        /// @param[in] rhs Scalar value to devide.
        /// @return        A reference to this matrix.
        self& operator/= (const_reference rhs)
        {
            assert( BML_f_Abs(rhs) > std::numeric_limits<float32>::epsilon() );
            for (sint32 i = 0; i < SIZE; ++i)
            {
                operator[](i) /= rhs;
            }
            return *this;
        }

        /// Add a scalar to all elements of the matrix.
		/// Add all elements of the matrix (inplace) by a given scalar value. 
		/// [this] = rhs + [this] 
        /// @param[in] rhs Scalar value to add.
        /// @return        A reference to this matrix.
        self& operator+= (const_reference rhs)
        {
            for (sint32 i = 0; i < SIZE; ++i)
            {
                operator[](i) += rhs;
            }
            return *this;
        }

        /// Subtract a scalar from all elements of the matrix.
		/// Subtract all elements of the matrix (inplace) by a given scalar value. 
		/// [this] = [this] - rhs 
        /// @param[in] rhs Scalar value to subtract.
        /// @return        A reference to this matrix.
        self& operator-= (const_reference rhs)
        {
            for (sint32 i = 0; i < SIZE; ++i)
            {
                operator[](i) -= rhs;
            }
            return *this;
        }

        /// Multiply all elements of the matrix by a scalar.
		/// Multiply all elements of the matrix (outplace) by a given scalar value. 
		/// [Res] = rhs * [this]
        /// @param[in] rhs Scalar value to multiply.
        /// @return        A Matrix of same size and type, but with internal memory
        ///                having all elements multiplied by rhs.
        self_internal operator* (const_reference rhs) const
        {
            return self_internal(*this).operator*=(rhs);
        }

        /// Divide all elements of the matrix by a scalar.
		/// Divide all elements of the matrix (outplace) by a given scalar value. 
		/// [Res] = [this] / rhs 
        /// @param[in] rhs Scalar value to devide.
        /// @return        A Matrix of same size and type, but with internal memory
        ///                having all elements divided by rhs.
        self_internal operator/ (const_reference rhs) const
        {
            return self_internal(*this).operator/=(rhs);
        }

        /// Add a scalar to all elements of the matrix.
		/// Add all elements of the matrix (outplace) by a given scalar value. 
		/// [Res] = rhs + [this]
        /// @param[in] rhs Scalar value to add.
        /// @return        A Matrix of same size and type, but with internal memory
        ///                having all elements added by rhs.
        self_internal operator+ (const_reference rhs) const
        {
            return self_internal(*this).operator+=(rhs);
        }

        /// Subtract a scalar from all elements of the matrix.
		/// Subtract all elements of the matrix (outplace) by a given scalar value. 
		/// [Res] = [this] - rhs 
        /// @param[in] rhs Scalar value to subtract.
        /// @return        A Matrix of same size and type, but with internal memory
        ///                having all elements subtracted by rhs.
        self_internal operator- (const_reference rhs) const
        {
            return self_internal(*this).operator-=(rhs);
        }

        /// @}

        /// Create an identity matrix of size ROWS X COLS.
		/// Initialize this matrix with an identity matrix of input dimensions.
		/// [A] = I[A]
		/// Input matrix should be a square matrix!
        /// @return        void
        void setIdentity()
        {
            assert(ROWS == COLS);
            fill(0.0F);
            for ( size_type i = 0; i < ROWS; ++i )
            {
                operator()(i,i) = 1.0F;
            }
        }

        /// Transpose of a matrix.
		/// Performs the transpose of a matrix (outplace)
        ///                |a11 a12|
        ///            A = |a21 a22| 
		///
        ///                |a11 a21|
        /// Transpose(A) = |a12 a22|
        /// @return        A reference to the resultant matrix of size COLS X ROWS.
        CMatrix<type, COLS, ROWS, false> transpose() const
        {
            CMatrix<type, COLS, ROWS> result;
            for ( size_type i = 0; i < ROWS; ++i )
            {
                for ( size_type j = 0; j < COLS; ++j )
                {
                    result(j,i) = operator()(i,j);
                }
            }
            return result;
        }
		        
        /// Matrix multiplication if the result is known to be a symmetric matrix
		/// Perform matrix multiplication (outplace) of two matrices A and B and 
		/// store the result in a resultant matrix. 
		/// The result is known to be a symmetric!        
        /// [Res] = [A] X [B]        
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    COLS2      Number of columns of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size ROWS X COLS2.
        /// @return               A reference to the resultant matrix of size ROWS X COLS2.
        template<typename TYPE2, sint32 COLS2, boolean_t EXT_MEM2>
        CMatrix<type, ROWS, COLS2, false> mulSym ( const CMatrix<TYPE2, COLS, COLS2, EXT_MEM2>& rhs ) const
        {
            assert( ROWS == COLS2 );
            CMatrix<type, ROWS, COLS2, false> result;

            size_type rci     = 0;
            size_type rci_res = 0;

            /* Perform calculations over upper triangular of result matrix */
            for (size_type i = 0; i < ROWS; i++)
            {
                size_type rcj_res = i * COLS2;
                for (size_type j = i; j < COLS2; j++)
                {
                    size_type nc2c = j;

                    type resultElement = 0; /* Local instance to reduces unnecessary stores as common matrix types (e.g. float) are kept in register */
                    for (size_type k = 0; k < COLS; k++)
                    {
                        resultElement += operator[](rci + k) * rhs[nc2c];
                        nc2c += COLS2;
                    }
                    result[rci_res + j] = resultElement;
                    result[rcj_res + i] = resultElement; //redundant at the diagonal but should be faster on pipelined DSPs

                    rcj_res += COLS2;
                }
                rci     += COLS;
                rci_res += COLS2;
            }

#if !defined NDEBUG && !defined (CML_TEST)
            /* C implementation*/
            CMatrix<type, ROWS, COLS2, false> result_CVersion;
            BML_t_Matrix resMatrixImpl = result_CVersion.getCMLMatrix();
            const BML_t_Matrix lhsMatrixImpl = getCMLMatrix();
            const BML_t_Matrix rhsMatrixImpl = rhs.getCMLMatrix();
            CML_v_MultiplyMatricesToSymResult(&resMatrixImpl, &lhsMatrixImpl, &rhsMatrixImpl);

            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }

        /// Matrix multiplication with transpose.
		/// Perform matrix multiplication (outplace) of matrix A with 
		/// transpose of matrix B and store the result in a resultant matrix.
        /// [Res] = [A] X [B]'
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    ROWS2      Number of rows of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size ROWS2 X COLS	
        /// @return               A reference to the resultant matrix of size ROWS X ROWS2.
        template<typename TYPE2, sint32 ROWS2, boolean_t EXT_MEM2>
        CMatrix<type, ROWS, ROWS2, false> mulTrnsp( const CMatrix<TYPE2, ROWS2, COLS, EXT_MEM2>& rhs ) const
        {
            CMatrix<type, ROWS, ROWS2, false> result;

            size_type rc_res = 0;
            size_type rc1    = 0;

            for (size_type i = 0; i < ROWS; i++)
            {
                size_type rc2 = 0;

                for (size_type j = 0; j < ROWS2; j++)
                {
                    type resultElement = 0; /* Local instance to reduces unnecessary stores as common matrix types (e.g. float) are kept in register */

                    for (size_type k = 0; k < COLS; k++)
                    {
                        resultElement += operator[](rc1 + k) * rhs[rc2 + k];
                    }

                    result[rc_res + j] = resultElement;

                    rc2 += COLS;
                }
                rc1    += COLS;
                rc_res += ROWS2;
            }

#if !defined NDEBUG && !defined (CML_TEST)
            /* C Implementation */
            CMatrix<type, ROWS, ROWS2, false> result_CVersion;
            BML_t_Matrix resMatrixImpl = result_CVersion.getCMLMatrix();
            const BML_t_Matrix lhsMatrixImpl = getCMLMatrix();
            const BML_t_Matrix rhsMatrixImpl = rhs.getCMLMatrix();
            CML_v_MultiplyMatrixWithTranspose( &resMatrixImpl, &lhsMatrixImpl, &rhsMatrixImpl );

            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }


        /// Transpose matrix before multiplication with rhs (outplace).
        /// Res = A' x B
        /// @tparam    TYPE2      The type of the matrix
        /// @tparam    COLS2      Number of columns of the matrix
        /// @tparam    EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                       of this class (false) or outside of this class (true).
        ///                       Default is false, meaning internal memory.
        /// @param[in] rhs        Constant reference to CMatrix object of size ROWS X COLS2.
        /// @return               A reference to the resultant matrix of size COLS X COLS2.
        template<typename TYPE2, sint32 COLS2, boolean_t EXT_MEM2>
        CMatrix<type, COLS, COLS2, false> trnspMul( const CMatrix<TYPE2, ROWS, COLS2, EXT_MEM2>& rhs ) const
        {
            CMatrix<type, COLS, COLS2, false> result;

            size_type rc_res = 0;

            for (size_type i = 0; i < COLS; i++)
            {
                for (size_type j = 0; j < COLS2; j++)
                {
                    type resultElement = 0; /* Local instance to reduces unnecessary stores as common matrix types (e.g. float) are kept in register */

                    size_type nc1c = i;
                    size_type nc2c = j;

                    for (size_type k = 0; k < ROWS; k++)
                    {
                        resultElement += operator[](nc1c) * rhs[nc2c];
                        nc1c += COLS;
                        nc2c += COLS2;
                    }

                    result[rc_res + j] = resultElement;
                }
                rc_res += COLS2;
            }

#if !defined NDEBUG && !defined (CML_TEST)
            /* C Implementation */
            CMatrix<type, COLS, COLS2, false> result_CVersion;
            CMatrix<type, COLS, ROWS, false>  lhsTranspose_CVersion;
            BML_t_Matrix resMatrixImpl = result_CVersion.getCMLMatrix();
            BML_t_Matrix lhsTransposeMatrixImpl = lhsTranspose_CVersion.getCMLMatrix();
            const BML_t_Matrix lhsMatrixImpl = getCMLMatrix();
            const BML_t_Matrix rhsMatrixImpl = rhs.getCMLMatrix();
            CML_v_TransposeMatrix(&lhsTransposeMatrixImpl, &lhsMatrixImpl);
            CML_v_MultiplyMatrices( &resMatrixImpl, &lhsTransposeMatrixImpl, &rhsMatrixImpl );

            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }

        /// Implements inverse of a matrix.		
        /// Uses Gauss-Jordan elimination with partial pivoting.
        /// For matrices upto 3x3, determinant is found, singularity is
        /// checked and processing is done, whereas for higher order
        /// matrices, matrix singularity is determined during the processing. 
		/// Res = inv(A)
        /// Struct needed, because partial specialization of functions is not allowed!
        /// @tparam     TYPE2      The type of the matrix
        /// @tparam     ROWS2      Number of rows of the matrix
        /// @tparam     COLS2      Number of columns of the matrix
        /// @tparam     EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                        of this class (false) or outside of this class (true).
        ///                        Default is false, meaning internal memory.
        /// @param[in]  inp        Constant reference to input CMatrix object of size ROWS2 X COLS2	
        /// @param[out] res        Reference to resultant CMatrix object of size ROWS2 X COLS2	
        /// @return                True if  matrix is invertible else False.
        template <typename TYPE2, sint32 ROWS2, sint32 COLS2, boolean_t EXT_MEM2>
        struct SInvert
        {
            static inline boolean_t invert(CMatrix<TYPE2, ROWS2, COLS2, false>& res, const CMatrix<TYPE2, ROWS2, COLS2, EXT_MEM2>& inp)
            {
                static const float32 fTol = 1e-10F;  /* tolerance */
                boolean_t bRet = true;
                TYPE2 fPivElem = static_cast<TYPE2>(1.0F);
                CMatrix<TYPE2, ROWS2, COLS2, false> A(inp);

                /* Gauss-Jordan elimination with partial pivoting */
                size_type col = 0;
                res.setIdentity();

                do
                {
                    /* find largest element on the selected column */
                    /* and use as pivot element                    */
                    size_type row = col;
                    size_type pos1 = col + (col * COLS2);
                    size_type pos2;

                    TYPE2 MaxElem = 0;
                    TYPE2 Temp;

                    for (size_type i = col; i < COLS2; i++)
                    {
                        Temp = BML_f_Abs(A[pos1]);
                        if (Temp > MaxElem)
                        {
                            MaxElem = Temp;
                            fPivElem = static_cast<float32>(A[pos1]);
                            row = i;
                        }
                        pos1 += COLS2;
                    }

                    /* exit routine if pivot element is very small => matrix not inversible */
                    if (MaxElem >= fTol)
                    {
                        bRet = true;
                    }
                    else
                    {
                        bRet = false;
                    }

                    /* do pivoting to reduce column to identity matrix */
                    if (bRet)
                    {
                        /* now swap rows to put the pivot element on the diagonal */
                        /* do the same operation for the result matrix */
                        if (row != col)
                        {
                            /* get pointer to matrix data */
                            pos1 = row * COLS2;
                            pos2 = col * COLS2;

                            for (size_type i = col; i < COLS2; i++)  /* only nonzero elements */
                            {
                                Temp = A[i + pos1];
                                A[i + pos1] = A[i + pos2];
                                A[i + pos2] = Temp;
                            }
                            for (size_type i = 0; i < COLS2; i++)    /* all elements */
                            {
                                Temp = res[i + pos1];
                                res[i + pos1] = res[i + pos2];
                                res[i + pos2] = Temp;
                            }
                        }

                        /* divide row by the pivot element => pivot becomes 1 */
                        /* do the same operation for the result matrix */
                        pos1 = col*COLS2;
                        const float32 fInvPivElem = 1.0F/fPivElem;
                        for (size_type i = col; i < COLS2; i++)  /* only nonzero elements */
                        {
                            A[i+pos1] *= fInvPivElem;
                        }
                        for (size_type i = 0; i < COLS2; i++)  /* all elements */
                        {
                            res[i+pos1] *= fInvPivElem;
                        }

                        /* now multiply the row by the right amount and substract from    */
                        /* each other row to make all the remaining elements in the pivot */
                        /* column zero                                                    */
                        for (size_type i = 0; i < COLS2; i++)   /* loop other rows */
                        {
                            if (i != col)
                            {
                                pos1  = i  * COLS2;
                                pos2  = col* COLS2;

                                /* use first element is row as scaling coefficient */
                                Temp = A[col + pos1];

                                /* subtract pivot row multiplied by scaling from other row */
                                /* do the same operation for the result matrix */
                                for (size_type j = col; j < COLS2; j++)  /* only nonzero elements */
                                {
                                    A[j + pos1] -= A[j + pos2] * Temp;
                                }
                                for (size_type j = 0; j  < COLS2; j++)  /* all elements */
                                {
                                    res[j + pos1] -= res[j + pos2] * Temp;
                                }
                            }
                        }

                        /* goto next column */
                        col++;
                    }
                } while ( bRet && (col < COLS2));  /* quit if finished or if matrix isn't invertible */

                return bRet;
            }
        };

        /// Implements inverse of a matrix with specialized version for matrix of size 1 X 1.
        /// @tparam     TYPE2      The type of the matrix
        /// @tparam     EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                        of this class (false) or outside of this class (true).
        ///                        Default is false, meaning internal memory.
        /// @param[in]  inp        Constant reference to input CMatrix object of size 1 X 1	
        /// @param[out] res        Reference to resultant CMatrix object of size 1 X 1		
        /// @return                True if matrix is invertible else False.
        template <typename TYPE2, boolean_t EXT_MEM2>
        struct SInvert<TYPE2, 1, 1, EXT_MEM2>
        {
            static inline boolean_t invert(CMatrix<TYPE2, 1, 1, false>& res, const CMatrix<TYPE2, 1, 1, EXT_MEM2>& inp)
            {
                boolean_t retVal = false;
                if( BML_f_IsNonZero(inp[0]) )
                {
                    res[0] = 1.0F / (inp[0]); /*Use cct::invert here (apply changes to C-Version) */
                    retVal = true;
                }
                return retVal;
            }
        };

        /// Implements inverse of a matrix with specialized version for matrix of size 2 X 2.
        /// @tparam     TYPE2      The type of the matrix
        /// @tparam     EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                        of this class (false) or outside of this class (true).
        ///                        Default is false, meaning internal memory.
        /// @param[in]  inp        Constant reference to input CMatrix object of size 2 X 2.	
        /// @param[out] res        Reference to resultant CMatrix object of size 2 X 2.		
        /// @return                True if matrix is invertible else False.
        template <typename TYPE2, boolean_t EXT_MEM2>
        struct SInvert<TYPE2, 2, 2, EXT_MEM2>
        {
            static inline boolean_t invert(CMatrix<TYPE2, 2, 2, false>& res, const CMatrix<TYPE2, 2, 2, EXT_MEM2>& inp)
            {
                boolean_t retVal = false;
                /* Local variable for determinant as common matrix types (e.g. float) are kept in register */
                TYPE2 Temp = (inp[0]*inp[3]) - (inp[1]*inp[2]);

                if(BML_f_IsNonZero(Temp))
                {
                    Temp = 1.0F / Temp; /*Use cct::invert here (apply changes to C-Version) */
                    res[0] =  inp[3] * Temp;
                    res[1] = -inp[1] * Temp;
                    res[2] = -inp[2] * Temp;
                    res[3] =  inp[0] * Temp;
                    retVal = true;
                }
                return retVal;
            }
        };

        /// Implements inverse of a matrix with specialized version for matrix of size 3 X 3.
        /// @tparam     TYPE2      The type of the matrix
        /// @tparam     EXT_MEM2   Flag whether the memory of the elements of the matrix is inside
        ///                        of this class (false) or outside of this class (true).
        ///                        Default is false, meaning internal memory.
        /// @param[in]  inp        Constant reference to input CMatrix object of size 3 X 3.	
        /// @param[out] res        Reference to resultant CMatrix object of size 3 X 3.		
        /// @return                True if  matrix is invertible else False.
        template <typename TYPE2, boolean_t EXT_MEM2>
        struct SInvert<TYPE2, 3, 3, EXT_MEM2>
        {
            static inline boolean_t invert(CMatrix<TYPE2, 3, 3, false>& res, const CMatrix<TYPE2, 3, 3, EXT_MEM2>& inp)
            {
                boolean_t retVal = false;

                /* Cramers Rule for matrix size == 3 */
                TYPE2 Temp = (( (inp[0]*inp[4]) - (inp[3]*inp[1]) )*inp[8]) + (( (inp[3]*inp[7]) - (inp[6]*inp[4]) )*inp[2]) + (( (inp[6]*inp[1]) - (inp[0]*inp[7]) )*inp[5]);

                if(BML_f_IsNonZero(Temp))
                {
                    Temp = 1.0F / Temp; /*Use cct::invert here (apply changes to C-Version) */
                    res[0] = ( (inp[4]*inp[8]) - (inp[5]*inp[7]) ) * Temp;
                    res[1] = ( (inp[2]*inp[7]) - (inp[1]*inp[8]) ) * Temp;
                    res[2] = ( (inp[1]*inp[5]) - (inp[2]*inp[4]) ) * Temp;
                    res[3] = ( (inp[5]*inp[6]) - (inp[3]*inp[8]) ) * Temp;
                    res[4] = ( (inp[0]*inp[8]) - (inp[2]*inp[6]) ) * Temp;
                    res[5] = ( (inp[2]*inp[3]) - (inp[0]*inp[5]) ) * Temp;
                    res[6] = ( (inp[3]*inp[7]) - (inp[4]*inp[6]) ) * Temp;
                    res[7] = ( (inp[1]*inp[6]) - (inp[0]*inp[7]) ) * Temp;
                    res[8] = ( (inp[0]*inp[4]) - (inp[1]*inp[3]) ) * Temp;
                    retVal = true;
                }
                return retVal;
            }
        };

        /// Compute matrix inverse by calling different variants of matrix inverse
		/// implemenatations accoridng to input matrix dimensions.
		/// Res = inv(A)        	
        /// @return                A reference to resultant matrix of size COLS X ROWS.
        CMatrix<type, COLS, ROWS, false> invert() const
        {
            assert( ROWS == COLS );
            CMatrix<type, COLS, ROWS, false> result;
            if (!SInvert<type, ROWS, COLS, EXT_MEM>::invert(result, *this))
            {
                // Inversion should never fail.
                assert(false);
            }

#if !defined NDEBUG && !defined (CML_TEST)
            CMatrix<type, COLS, ROWS, false> result_CVersion;
            CMatrix<type, COLS, ROWS, false> cache_CVersion(*this);
            BML_t_Matrix resMatrixImpl   = result_CVersion.getCMLMatrix();
            BML_t_Matrix cacheMatrixImpl = cache_CVersion.getCMLMatrix();
            CML_v_InvertMatrix( &resMatrixImpl, &cacheMatrixImpl );
            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }

        /// Compute matrix inverse by calling different variants of matrix inverse
		/// implemenatations accoiidng to input matrix dimensions.
		/// This variant is with input argument as reference to resultant matrix
		/// Res = inv(A) 
        /// @param[in]  result     Reference to resultant matrix of size COLS X ROWS.
        /// @return                Return True if  matrix is invertible else False.
        boolean_t invert(CMatrix<type, COLS, ROWS, false>& result) const
        {
            assert( ROWS == COLS );

            const boolean_t canInvert = SInvert<type, ROWS, COLS, EXT_MEM>::invert(result, *this);
#if !defined NDEBUG && !defined (CML_TEST)
            if (canInvert)
            {
                CMatrix<type, COLS, ROWS, false> result_CVersion;
                CMatrix<type, COLS, ROWS, false> cache_CVersion(*this);
                BML_t_Matrix resMatrixImpl   = result_CVersion.getCMLMatrix();
                BML_t_Matrix cacheMatrixImpl = cache_CVersion.getCMLMatrix();
                CML_v_InvertMatrix( &resMatrixImpl, &cacheMatrixImpl );
                /* Perform matrix comparison */
                assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
            }
#endif

            return canInvert;
        }

        /// Compute Cholesky factor of a positive definite hermitian matrix.
		/// The Cholesky factorization is decomposing the hermitian positive definite matrix (A)
        /// into product of a lower triangular matrix (L) and its conjugare transpose (L*).
        /// A = LL*
        /// This function returns the lower triangular square root of the positive definite 
        /// real symmetric matrix.         
        /// No exception handling for any kind of rank deficiency rather direct regularization!
        /// (associated m-file for unit testing: slow_chol2.m)		
        /// @return                A reference to resultant matrix of size COLS X ROWS.
        CMatrix<type, COLS, ROWS, false> getCholesky() const
        {
            assert( ROWS == COLS );
            static const float32 fTol = 1e-10F;

            size_type rci = 0;

            /* Copy to result matrix */
            CMatrix<type, COLS, ROWS, false> result(*this);


            /* Add pseudo regularization */
            for ( size_type i = 0; i < ROWS; i++)
            {
                if (result[rci + i] < fTol) // should never happen for p.d. input matrix
                {
                    result[rci + i]  = fTol; // just "brutal" pseudo regularization good for KAFI
                }
                rci += COLS;
            }

            /* Cholesky decomposition with Cholesky–Banachiewicz algorithm */
            rci = 0;

            for ( size_type i = 0; i < ROWS; i++)
            {
                size_type rcj = 0;

                /* Calculate elements in row not on diagonal */
                for ( size_type j = 0; j < i; j++)
                {
                    type resultElement = result[rci + j]; /* Local instance to reduces unnecessary stores as common matrix types (e.g. float) are kept in register */

                    for ( size_type k = 0; k < j; k++)
                    {
                        resultElement -= result[rci + k] * result[rcj + k];
                    }

                    resultElement /= result[rcj + j]; /*Speedup: use cct::invert here (also adapt C-Version of Cholesky) */
                    result[rci + j] = resultElement;
                    result[rcj + i] = 0; /* set upper triangular part to zero */
                    rcj += COLS;
                }

                /* Calculate diagonal element */
                {
                    type resultElement = result[rci + i]; /* Local instance to reduces unnecessary stores as common matrix types (e.g. float) are kept in register */

                    for ( size_type k = 0; k < i; k++)
                    {
                        resultElement -= cml::pow<2>(result[k + rci]);
                    }

                    /* Check for tolerance */
                    if ( resultElement > fTol)
                    {
                        result[rci + i] = BML_f_Sqrt(resultElement)     ;
                    }
                    else
                    { // this should never happen for p.d. input matrix
                        result[rci + i] = fTol; // it is caused by rank-deficient input matrix
                    }
                }

                rci += COLS;
            }

#if !defined NDEBUG && !defined (CML_TEST)
            /* C Implementation */
            CMatrix<type, COLS, ROWS, false> result_CVersion;
            BML_t_Matrix cacheMatrixImpl = getCMLMatrix();
            BML_t_Matrix resMatrixImpl   = result_CVersion.getCMLMatrix();
            BML_v_CholeskyMatrix( &resMatrixImpl, &cacheMatrixImpl );
            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }

        /// Inverse of positive definite lower triangular matrix  by forward substitution.
        /// The method of choice for inversion of positive definite symmetric matrices
        /// consists of Cholesky factorization followed by forward substitution
        /// which is efficiently implemented in this function. No exception
        /// handling in case of indefinite input rather brute force regularization
        /// which is appropriate for square-root Kalman filter applications
        /// but does a poor job in approximating the Pseudo-Inverse for least-squares	
        /// @return                A reference to resultant matrix of size COLS X ROWS.
        CMatrix<type, COLS, ROWS, false> getLowTriaInverse() const
        {
            assert( ROWS == COLS );
            static const float32 fTol = 1e-10F;
            CMatrix<type, COLS, ROWS, false> result;


            /* Vector for storing pre-calculated divisions */
            CMatrix<type, COLS, 1, false> diag;

            size_type rcj = 0;

            for (size_type j = 0; j < COLS; j++)
            {
                if (operator[](rcj + j) < fTol)
                {
                    diag[j] = 1.0F/fTol; // regularization
                }
                else
                {
                    diag[j] = 1.0F/operator[](rcj + j);  //precalc divisions, probably use cct::inverse here (adapt in C-Version too!)
                }

                for (size_type i = j; i < COLS; i++)
                {
                    result[rcj + i] = 0.0F;
                }

                rcj += COLS;
            }

            for (size_type j = 0; j < COLS; j++)
            {
                size_type rci = j * COLS;
                for (size_type i = j; i < ROWS; i++)
                {
                    type resultElement = 0.0F;

                    for (size_type k = 0; k < i; k++)
                    {
                        resultElement += operator[](rci + k) * result[(k * COLS) + j]; //k<i -> does not use elements on diagonal
                    }
                    if (i == j)
                    {
                        result[rci + j] = diag[i] * (1.0F - resultElement);
                    }
                    else
                    {
                        result[rci + j] = diag[i] * (-resultElement);
                    }

                    rci += COLS;
                }
            }

#if !defined NDEBUG && !defined (CML_TEST)
            /* C Implementation */
            CMatrix<type, COLS, ROWS, false> result_CVersion;
            const BML_t_Matrix cacheMatrixImpl = getCMLMatrix();
            BML_t_Matrix resMatrixImpl   = result_CVersion.getCMLMatrix();
            BML_v_LowTriaInvMatrix( &resMatrixImpl, &cacheMatrixImpl ); /*Changes cacheMatrix -> !!Changes member variable!! */
            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }

        /// Computes the product of the lower traingular matrix and its transpose.
		/// Inversion of Cholesky Res = A*transpose(A), with A lower triangular.        	
		/// The method of choice for squaring of positive definite symmetric
        /// matrices is from Cholesky method. 
        /// @return                A reference to resultant matrix of size COLS X ROWS.
        CMatrix<type, COLS, ROWS, false> getLowTriaSqr() const
        {
            assert( ROWS == COLS );
            CMatrix<type, COLS, ROWS, false> result;

            size_type rci = 0;

            for (size_type i = 0; i < ROWS; i++)
            {
                size_type rcj = 0;
                for (size_type j = 0; j <= i; j++)
                {
                    type resultElement = 0.0F; /* Local instance to reduces unnecessary stores as common matrix types (e.g. float) are kept in register */
                    for (size_type k = 0; k <= j; k++)
                    {
                        resultElement += operator[](rci + k) * operator[](rcj + k);
                    }
                    result[rcj + i] = resultElement;
                    result[rci + j] = resultElement;  //redundant at the diagonal but should be faster on pipelined DSPs

                    rcj += COLS;
                }
                rci += COLS;
            }

#if !defined NDEBUG && !defined (CML_TEST)
            /* C Implementation */
            CMatrix<type, COLS, ROWS, false> result_CVersion;
            const BML_t_Matrix cacheMatrixImpl = getCMLMatrix();
            BML_t_Matrix       resMatrixImpl   = result_CVersion.getCMLMatrix();
            BML_v_LowTriaSqrMatrix( &resMatrixImpl, &cacheMatrixImpl );
            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }

		/// Computes the product of the upper traingular matrix and its transpose.
        /// Res = A*transpose(A), with A upper triangular.			
        /// @return                A reference to resultant matrix of size COLS X ROWS.
        CMatrix<type, COLS, ROWS, false> getUppTriaSqr() const
        {
            assert( ROWS == COLS );
            CMatrix<type, COLS, ROWS, false> result;

            size_type rci = 0;

            for (size_type i = 0; i < ROWS; i++)
            {
                size_type rcj = 0;
                for (size_type j = 0; j <= i; j++)  // <lint left brace
                {
                    type resultElement = 0.0F; /* Local instance to reduces unnecessary stores as common matrix types (e.g. float) are kept in register */
                    for (size_type k = i; k < COLS; k++)
                    {
                        resultElement += operator[](rci + k) * operator[](rcj + k);
                    }
                    result[rcj + i] = resultElement;
                    result[rci + j] = resultElement;  //redundant at the diagonal but should be faster on pipelined DSPs

                    rcj += COLS;
                }
                rci += COLS;
            }

#if !defined NDEBUG && !defined (CML_TEST)
            /* C Implementation */
            CMatrix<type, COLS, ROWS, false> result_CVersion;
            const BML_t_Matrix cacheMatrixImpl = getCMLMatrix();
            BML_t_Matrix       resMatrixImpl   = result_CVersion.getCMLMatrix();
            BML_v_UppTriaSqrMatrix( &resMatrixImpl, &cacheMatrixImpl );
            /* Perform matrix comparison */
            assert( result.getAlmostEqualRelativeAndAbs(result_CVersion) );
#endif

            return result;
        }

#if !defined NDEBUG && !defined (CML_TEST)
        /// Write to a const matrix.
        /// be careful: getCMLMatrix() enables you to write to a const matrix	
        /// @return                Reference to the resultant matrix.
        BML_t_Matrix getCMLMatrix() const
        {
            BML_t_Matrix ret = {{COLS, ROWS, static_cast<uint16>(SIZE)}, const_cast<pointer>(&operator[](0))};
            return ret;
        }
#endif

#if !defined NDEBUG || defined (CML_TEST)
        /// Checks if two matrices are equal up to a certain precision
        /// @param[in]  rhs        Constant reference to matrix of identical size
        /// @return                Return True if  matrices are equal else False.
        boolean_t getAlmostEqualRelativeAndAbs( const self& rhs ) const
        {
            boolean_t result = true;
            size_type rci = 0;

            for (size_type i = 0; i < ROWS; i++)
            {
                for (size_type j = 0; j < COLS; j++)
                {
                    if( !getElementsAlmostEqualRelativeAndAbs( operator[](rci+j), rhs[rci+j]) )
                    {
                        result = false;
                    }
                }
                rci += COLS;
            }

            return result;
        }
#endif

    private:
#if !defined NDEBUG || defined (CML_TEST)
        /// Checks if two floating point numbers are almost equal
        /// The functions performs an absolute and relative comparison of the
        /// floating point numbers. The numbers are regarded equal if their
        /// abs difference is smaller than 10*FLT_MIN (required for numbers
        /// close to zero) or if their relative difference is smaller than
        /// 5 * FLT_EPSILON
        /// @param[in]  A    Constant reference to value A	
        /// @param[in]  B    Constant reference to value B
        /// @return          Return True if  values are equal else False.
        static boolean_t getElementsAlmostEqualRelativeAndAbs(const_reference A, const_reference B)
        {
            boolean_t ret = false;

            // Check if the numbers are really close -- needed
            // when comparing numbers near zero.
            const_type diff = BML_f_Abs(A - B);

            ret = diff <= (std::numeric_limits<float32>::min()*10);

            if ( !ret )
            {
                const_type absA    = BML_f_Abs(A);
                const_type absB    = BML_f_Abs(B);
                const_type largest = cml::max(absA, absB);

                ret = diff <= (largest * std::numeric_limits<float32>::epsilon() * 5);
            }

            return ret;
        }
#endif
    };

    /// MULTIPLY value type with allocating to this.
    /// @tparam     T          The type of the element
    /// @tparam     ROWS       Number of rows of the Matrix
    /// @tparam     COLS       Number of columns of the Matrix
    /// @tparam     EXT_MEM    Flag whether the memory of the elements of the matrix is inside
    ///                        of this class (false) or outside of this class (true).
    ///                        Default is false, meaning internal memory.
    /// @param[in]  lhs        Reference to the value to multiply
    /// @param[in]  rhs        Constant reference to input CMatrix object of size ROWS X COLS
    /// @return                Reference to resultant matrix
    template<class T, sint32 ROWS, sint32 COLS, boolean_t EXT_MEM>
    static CMatrix<T, ROWS, COLS, false> operator* ( const T& lhs, const CMatrix<T,ROWS,COLS,EXT_MEM>& rhs )
    {
        typedef typename remove_const<T>::type type;
        CMatrix<type, ROWS, COLS, false> result(rhs);
        result.operator*=(lhs);
        return result;
    }
}

#endif //CML_STL_MATRIX_H__
