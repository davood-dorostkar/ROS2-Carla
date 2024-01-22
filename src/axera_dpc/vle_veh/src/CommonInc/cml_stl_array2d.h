    

#ifndef CML_STL_ARRAY2D_H__
#define CML_STL_ARRAY2D_H__

#include "cml_stl_array.h"
#include <limits>

namespace cml
{

    /// Two-dimensional implementation of an array
    /// @tparam T       The type of the elements
    /// @tparam ROWS    The number of rows of the array
    /// @tparam COLS    The number of columns of the array
    /// @tparam EXT_MEM Flag whether the memory of the elements of the array is inside
    ///                 of this class (false) or outside of this class (true).
    ///                 Default is false, meaning internal memory.
    template < class T, sint32 ROWS, sint32 COLS, boolean_t EXT_MEM = false >
    class CArray2D
        : public CArray<T, ROWS*COLS, EXT_MEM>
    {
    public:
        static const sint32 SIZE = ROWS * COLS;

        typedef CArray2D<T, ROWS, COLS, EXT_MEM> self;
        typedef CArray  <T, SIZE,       EXT_MEM> super;
        typedef sint32                           size_type;
        typedef typename remove_const<T>::type   type;
        typedef typename    add_const<T>::type   const_type;
        typedef typename remove_const<T>::type&  reference;
        typedef typename    add_const<T>::type&  const_reference;
        typedef typename remove_const<T>::type*  pointer;
        typedef typename    add_const<T>::type*  const_pointer;
        // iterator typedefs
#ifndef NDEBUG
        typedef cml::CLinMemIt< type, false, super >  it;
        typedef cml::CLinMemIt< type, true , super > cIt;
#else
        typedef cml::CLinMemIt< type, false >  it;
        typedef cml::CLinMemIt< type, true  > cIt;
#endif
        typedef it  iterator;
        typedef cIt const_iterator;


        /// Default empty constructor.
        CArray2D(){}

        /// Copy constructor with non-constant pointer to a constant data and constant size. 
        /// Copies external data to the internal data storage.
        /// @param[in]       data    Non-constant pointer to the constant external data. 
        /// @param[in]       size    Constant size of the external buffer.
        CArray2D(       pointer const data, const sint32 size ) : super(data, size) {}

        /// Copy constructor with constant pointer to a constant data constant size.  
        /// Copies external data to the internal data storage.
        /// @param[in]       data    Constant pointer to the constant external data. 
        /// @param[in]       size    Constant size of the external buffer.
        CArray2D( const_pointer const data, const sint32 size ) : super(data, size) {}

        /// Copy constructor with non-constant reference to non-constant external data 
        /// of possibly different size than this. 
		/// Copies external data to the internal data storage.
        /// @tparam SIZE2    The size of the non-constant external data.    
        /// @param[in]       data    Non-constant reference to the external data of SIZE2.
        template< sint32 SIZE2 >
        CArray2D(       type (&data)[SIZE2] ) : super(data) {}

        /// Copy constructor with constant reference to non-constant external data 
        /// of possibly different size than this. 
		/// Copies external data to the internal data storage.
        /// @tparam SIZE2    The size of the non-constant external data.     
        /// @param[in]       data    Constant reference to the external data of SIZE2.
        template< sint32 SIZE2 >
        CArray2D( const_type (&data)[SIZE2] ) : super(data) {}

        /// Copy constructor with constant reference to identical non-constant external data 
        /// of possibly different size and external memory settings.  
        /// @tparam SIZE2    The size of the non-constant external data.   
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Constant reference to the non-constant external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArray2D( const CArrayStorage<      type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Copy constructor with constant reference to identical constant external data 
        /// of possibly different size and external memory settings.  
        /// @tparam SIZE2    The size of the constant external data.   
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Constant reference to the constant external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArray2D( const CArrayStorage<const_type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Copy constructor with non-constant reference to identical non-constant external data 
        /// of possibly different size and external memory settings.  
        /// @tparam SIZE2    The size of the non-constant external data.   
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Non-constant reference to the non-constant external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArray2D( CArrayStorage<      type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Copy constructor with non-constant reference to identical constant external data 
        /// of possibly different size and external memory settings.  
        /// @tparam SIZE2    The size of the constant external data.   
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Non-constant reference to the constant external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArray2D( CArrayStorage<const_type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Returns number of rows of this 2d array.
        /// @return          Number of Rows in the 2d array.
        static sint32 rows() { return ROWS; }

        /// Returns number of columns of this 2d array.
        /// @return          Number of Columns in the 2d array.
        static sint32 cols() { return COLS; }

        /// Copies the CArray2D content to the buffer of this.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Constant reference to non-constant external data buffer of identical size.
        /// @return          Reference to this pointer.
        template< boolean_t EXT_MEM2 >
        CArray2D& operator=(const CArray2D<      type, ROWS, COLS, EXT_MEM2>& rhs)
        {
            super::operator=(rhs);
            return *this;
        }

        /// Copies the constant type CArray2D content to the buffer of this.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Constant reference to constant external data buffer of identical size.
        /// @return          Reference to this pointer.
        template< boolean_t EXT_MEM2 >
        CArray2D& operator=(const CArray2D<const_type, ROWS, COLS, EXT_MEM2>& rhs)
        {
            super::operator=(rhs);
            return *this;
        }

        /// This is only required for TI-Compiler to be able to compile
        using super::operator[];
        // End of section required for TI-Compiler

        /// Returns the element at row*COLS + col.
        /// Asserts if given row and column positions are within available 
        /// ROWS and COLS.
        /// @param[in]       row    Constant row position of the element. 
        /// @param[in]       col    Constant column position of the element. 
        /// @return          Reference to Array element at row*COLS + col.
        reference operator()(const size_type row, const size_type col)
        {
            // In operator[] only the whole array is checked
            assert( (row < ROWS) && (col < COLS) );
            return operator[]( (row*COLS) + col );
        }

        /// Returns a constant reference of the element at row*COLS + col with constant object type.
        /// Asserts if given row and column positions are within bounds!
        /// @param[in]       row    Constant row position of the element 
        /// @param[in]       col    Constant column position of the element 
        /// @return          Constant reference to Array element at row*COLS + col
        const_reference operator()(const size_type row, const size_type col) const
        {
            // In operator[] only the whole array is checked
            assert( (row < ROWS) && (col < COLS) );
            return operator[]( (row*COLS) + col );
        }

        /// Returns a reference of the element at given row and column position.
        /// @param[in]       row    Constant row position of the element 
        /// @param[in]       col    Constant column position of the element 
        /// @return          Reference to Array element at given row and column position
        reference at(const size_type row, const size_type col)
        {
            return operator()(row, col);
        }

        /// Returns a constant reference of the element at given row 
        /// and column position with constant object type.
        /// @param[in]       row    Constant row position of the element. 
        /// @param[in]       col    Constant column position of the element. 
        /// @return          Constant reference to Array element at given row and column position.
        const_reference at(const size_type row, const size_type col) const
        {
            return operator()(row, col);
        }


        typedef it                                                   rowIt; ///< iterator iterating along a single row
        typedef cIt                                                 cRowIt; ///< const iterator iterating along a single row
#ifndef NDEBUG
        typedef typename cml::CLinMemIt< type, false, super, COLS >  colIt; ///< iterator iterating along a single column
        typedef typename cml::CLinMemIt< type, true , super, COLS > cColIt; ///< const iterator iterating along a single column
#else
        typedef typename cml::CLinMemIt< type, false, COLS >         colIt; ///< iterator iterating along a single column
        typedef typename cml::CLinMemIt< type, true , COLS >        cColIt; ///< const iterator iterating along a single column
#endif

        /// Returns an iterator to the first row of this array.
        /// In debug case asserts that row is within the bounds.				   
        /// @param[in]       row    Row position. 
        /// @return          An iterator to the first row of this array.
#ifndef NDEBUG
        rowIt  rowBegin(sint32 row)        { assert(row < ROWS); return rowIt (&m_data[row*COLS], this, &m_data[row*COLS], &m_data[(row+1)*COLS]); }
#else
        rowIt  rowBegin(sint32 row)        { return rowIt (&m_data[row*COLS]); }
#endif

        /// Returns a constant iterator to the first row of this array with constant object type.
        /// In debug case asserts that row is within the bounds.
        /// @param[in]       row    Row position. 
        /// @return          A constant iterator to the first row of this array.
#ifndef NDEBUG
        cRowIt rowBegin(sint32 row)  const { assert(row < ROWS); return cRowIt(&m_data[row*COLS], this, &m_data[row*COLS], &m_data[(row+1)*COLS]); }
#else
        cRowIt rowBegin(sint32 row)  const { return cRowIt(&m_data[row*COLS]); }
#endif

        /// Returns a constant iterator to the first row of this array with constant object type.  
        /// In debug case asserts that row is within the bounds.
        /// @param[in]       row    Row position. 
        /// @return          A constant iterator to the first row of this array.
#ifndef NDEBUG
        cRowIt cRowBegin(sint32 row) const { assert(row < ROWS); return cRowIt(&m_data[row*COLS], this, &m_data[row*COLS], &m_data[(row+1)*COLS]); }
#else
        cRowIt cRowBegin(sint32 row) const { return cRowIt(&m_data[row*COLS]); }
#endif

        /// Returns an iterator to the last row of this array.  
        /// In debug case asserts that row is within the bounds.
        /// @param[in]       row    Row position.  
        /// @return          An iterator to last row of this array.
#ifndef NDEBUG
        rowIt  rowEnd(sint32 row)        { assert(row < ROWS); return rowIt (&m_data[(row+1)*COLS], this, &m_data[row*COLS], &m_data[(row+1)*COLS]); }
#else
        rowIt  rowEnd(sint32 row)        { return rowIt (&m_data[(row+1)*COLS]); }
#endif

        /// Returns a constant iterator to the last row of this array with constant object type.  
        /// In debug case asserts that row is within the bounds.
        /// @param[in]       row    Row position 
        /// @return          A constant iterator to the last row of this array.
#ifndef NDEBUG
        cRowIt rowEnd(sint32 row)  const { assert(row < ROWS); return cRowIt(&m_data[(row+1)*COLS], this, &m_data[row*COLS], &m_data[(row+1)*COLS]); }
#else
        cRowIt rowEnd(sint32 row)  const { return cRowIt(&m_data[(row+1)*COLS]); }
#endif

        /// Returns a constant iterator to the last row of this array with constant object type.
        /// In debug case asserts that row is within the bounds.
        /// @param[in]       row    Row position. 
        /// @return          A constant iterator to the last row of this array.
#ifndef NDEBUG
        cRowIt cRowEnd(sint32 row) const { assert(row < ROWS); return cRowIt(&m_data[(row+1)*COLS], this, &m_data[row*COLS], &m_data[(row+1)*COLS]); }
#else
        cRowIt cRowEnd(sint32 row) const { return cRowIt(&m_data[(row+1)*COLS]); }
#endif

        /// Returns an iterator to the first column of this array.
        /// In debug case asserts that column is within the bounds.
        /// @param[in]       col    Column position. 
        /// @return          An iterator to the first column of this array.
#ifndef NDEBUG
        colIt  colBegin(sint32 col)        { assert(col < COLS); return colIt (&m_data[col], this); }
#else
        colIt  colBegin(sint32 col)        { return colIt (&m_data[col]); }
#endif

        /// Returns a constant iterator to the first column of this array with constant object type.
        /// In debug case asserts that column is within the bounds.
        /// @param[in]       col    Column position. 
        /// @return          A constant iterator to the first column of this array.
#ifndef NDEBUG
        cColIt colBegin(sint32 col)  const { assert(col < COLS); return cColIt(&m_data[col], this); }
#else
        cColIt colBegin(sint32 col)  const { return cColIt(&m_data[col]); }
#endif

        /// Returns a constant iterator to the first column of this array with constant object type.
        /// In debug case asserts that column is within the bounds.
        /// @param[in]       col    Column position. 
        /// @return          A constant iterator to the first column of this array.
#ifndef NDEBUG
        cColIt cColBegin(sint32 col) const { assert(col < COLS); return cColIt(&m_data[col], this); }
#else
        cColIt cColBegin(sint32 col) const { return cColIt(&m_data[col]); }
#endif

        /// Returns an iterator to the last column of this array.
        /// In debug case asserts that column is within the bounds.
        /// @param[in]       col    Column position.
        /// @return          Returns an iterator to the last column of this array.
#ifndef NDEBUG
        colIt  colEnd(sint32 col)        { assert(col < COLS); return colIt (&m_data[col+SIZE], this); }
#else
        colIt  colEnd(sint32 col)        { return colIt (&m_data[col+SIZE]); }
#endif

        /// Returns a constant iterator to the last column of this array with constant object type.
        /// In debug case asserts that column is within the bounds.
        /// @param[in]       col    Column position.
        /// @return          Returns a constant iterator to the last column of this array.
#ifndef NDEBUG
        cColIt colEnd(sint32 col)  const { assert(col < COLS); return cColIt(&m_data[col+SIZE], this); }
#else
        cColIt colEnd(sint32 col)  const { return cColIt(&m_data[col+SIZE]); }
#endif

        /// Returns a constant iterator to the last column of this array with constant object type.
        /// In debug case asserts that column is within the bounds.
        /// @param[in]       col    Column position.
        /// @return          Returns a constant iterator to the last column of this array.
#ifndef NDEBUG
        cColIt cColEnd(sint32 col) const { assert(col < COLS); return cColIt(&m_data[col+SIZE], this); }
#else
        cColIt cColEnd(sint32 col) const { return cColIt(&m_data[col+SIZE]); }
#endif

        // This is only required for TI-Compiler to be able to compile
    protected:
        using super::m_data;
        // End of section required for TI-Compiler
    };
} //End of cml namespace


#endif //CML_STL_ARRAY2D_H__

