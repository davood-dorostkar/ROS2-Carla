    

#ifndef CML_STL_ARRAY_H__
#define CML_STL_ARRAY_H__

#include "cml_stl_iterator.h"
#include "TM_Global_Types.h"
#include <algorithm>
#include <cstring>

namespace cml
{

    /// Provide Storage for Array specialized for internal memory
    /// @tparam T       The type of the elements
    /// @tparam SIZE    The size of the array
    /// @tparam EXT_MEM Flag whether the memory of the elements of the array is inside
    ///                 of this class (false) or outside of this class (true).
    ///                 Default is false, meaning internal memory.
    template<class T, sint32 SIZE, boolean_t EXT_MEM = false>
    class CArrayStorage;

    template<class T, sint32 SIZE>
    class CArrayStorage<T, SIZE, false>
    {
    public:
        typedef CArrayStorage<T, SIZE, false>   self;
        typedef int                             size_type;
        typedef typename remove_const<T>::type  type;
        typedef typename    add_const<T>::type  const_type;
        typedef typename remove_const<T>::type& reference;
        typedef typename    add_const<T>::type& const_reference;

        enum { MAX_SIZE = SIZE };

        /// Default empty constructor.
        //   Our strategy is to have a late init using the reset call.
        //   This is necessary to be able to fulfill the SIL-SIL requirement
        CArrayStorage() {}

        /// Copy constructor that copies elements of the given constant pointer 
        /// constant data external array into the internal storage.
        /// Caller should ensure that s_size >= SIZE.
        /// @param[in] p_data Constant pointer to external constant data buffer.
        /// @param[in] s_size Number of elements in the external data buffer.
        explicit CArrayStorage(const T* const p_data, const sint32 s_size)
        {
            (void) s_size;
            assert( SIZE <= s_size );
            std::memcpy( &operator[](0), p_data, SIZE*sizeof(type) );
        }

        /// Template copy constructor that copies elements of the given  
        /// constant data external array into the internal storage.
        /// Caller should ensure that SIZE2 >= SIZE.
        /// @tparam    SIZE2 The size of the given external array.
        /// @param[in] data  Constant reference to the external constant Array of SIZE2.
        template<sint32 SIZE2>
        explicit CArrayStorage( const T (&data)[SIZE2] )
        {
            assert( SIZE <= SIZE2 );
            std::memcpy( &operator[](0), &data[0], SIZE*sizeof(type) );
        }
		
		/// Template copy constructor that copies elements of the given  
        /// non-constant data external array into the internal storage.
        /// Caller should ensure that SIZE2 >= SIZE.
        /// @tparam    SIZE2 The size of the given external array.
        /// @param[in] data  Reference to the external non-constant Array of SIZE2.
		template<sint32 SIZE2>
        explicit CArrayStorage(T (&data)[SIZE2] )
        {
            assert( SIZE <= SIZE2 );
            std::memcpy( &operator[](0), &data[0], SIZE*sizeof(type) );
        }

        /// Copy constructor that copies external identical data
        /// of same size to the internal data storage.
        /// @param[in] rhs Constant reference to the identical external Array of SIZE.
        explicit CArrayStorage( const self& rhs )
        {
            std::memcpy( &operator[](0), &rhs[0], SIZE*sizeof(type) );
        }

        /// Template copy constructor that copies non-constant external identical data 
        /// of different size and memory specializations to the internal data storage.
        /// Asserts that right hand size is always equal or larger than internal size.
        /// @tparam SIZE2    The size of the given external array.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs   Constant reference to the identical external Array of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArrayStorage( const CArrayStorage<type,SIZE2,EXT_MEM2>& rhs )
        {
            assert( SIZE <= SIZE2 );
            std::memcpy( &operator[](0), &rhs[0], SIZE*sizeof(type) );
        }

        /// Template copy constructor that copies constant external identical data 
        /// of different size and memory specializations to the internal data storage.
        /// Asserts that right hand size is always equal or larger than internal size.
        /// @tparam SIZE2    The size of the given external array.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs   Constant reference to the identical external constant Array of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArrayStorage( const CArrayStorage<const_type,SIZE2,EXT_MEM2>& rhs )
        {
            assert( SIZE <= SIZE2 );
            std::memcpy( &operator[](0), &rhs[0], SIZE*sizeof(type) );
        }

        /// Returns the reference of the element at given position.
        /// Asserts that position is inside of the array.
        /// @param[in]       position   Constant position of the element.
        /// @return          Reference to the array element at given position.
        reference operator[](const size_type position)
        {
            //make sure position is between limits - otherwise return zero's element
            assert( (position >= 0) && (position < size()) );
            return m_data[position];
        }

        /// Returns the constant reference to the element at given position of constant object.
        /// Asserts that position is inside of the array.
        /// @param[in]       position   Constant position of the element.
        /// @return          Constant reference to Array element at given position.
        const_reference operator[](const size_type position) const
        {
            assert( (position >= 0) && (position < size()) );
            return m_data[position];
        }

        /// Returns number of elements in the array.
        /// @return          Number of elements in the Array.
        static sint32 size()
        {
            return max_size();
        }

        /// Returns number of elements in the array.
        /// @return          Number of elements in the Array.
        static sint32 max_size()
        {
            return SIZE;
        }

        protected:
            T m_data[SIZE];
    };


    /// This class variant is not usable, since an array with only constant data
    /// inside can't be initialized in cpp.
    /// @tparam T       The type of the elements
    /// @tparam SIZE    The size of the array
    template<class T, sint32 SIZE>
    class CArrayStorage<const T, SIZE, false>;


    /// Provide Storage for Array, specialized for external memory and non-constant data.
    template<class T, sint32 SIZE>
    class CArrayStorage<T,SIZE,true>
    {
    public:
        typedef CArrayStorage<T,SIZE,true> self;
        typedef int                        size_type;

        enum { MAX_SIZE = SIZE };

        /// Default constructor for specialization of external memory
        /// Makes the storage invalid in debug mode.
        CArrayStorage()
#ifndef NDEBUG
                : m_data(0)
#endif
            {}

        /// Copy constructor that attaches external data of type non-constant pointer
		/// constant data to the internal pointer.
        /// Caller should ensure that s_size >= SIZE.
        /// @param[in] p_data   Non-constant pointer to the constant external data buffer.
        /// @param[in] s_size   Constant size of the external data buffer.
        CArrayStorage(T* const p_data, const sint32 s_size)
        {
            attach(p_data, s_size);
        }

        /// Template copy constructor that attaches external data of type non-constant reference
		/// non-constant data to the internal pointer.
        /// Caller should ensure that SIZE2 >= SIZE.
        /// @tparam SIZE2    The size of the external array.
        /// @param[in]       data  Reference to the external non-constant data buffer of SIZE2.
        template<sint32 SIZE2>
        explicit CArrayStorage( T (&data)[SIZE2] )
        {
            attach(&data[0], SIZE2);
        }

        /// Template copy constructor to attach external identical non-constant data of different size and memory 
		/// specializations to the internal pointer.
        /// @tparam SIZE2    The size of the external array.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs   Reference to the external non-constant data buffer of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArrayStorage( CArrayStorage<T,SIZE2,EXT_MEM2>& rhs )
        {
            attach(&rhs[0], SIZE2);
        }

        /// Copy external identical constant reference non-constant data of different size and memory 
		/// specializations to the internal storage.        
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs   Constant reference to external non-constant data buffer of identical size.
        /// @return          Reference to this pointer.
        template< boolean_t EXT_MEM2 >
        CArrayStorage& operator=(const CArrayStorage<T,SIZE,EXT_MEM2>& rhs)
        {
            std::memcpy( &operator[](0), &rhs[0], SIZE*sizeof(T) );
            return *this;
        }

        /// Copy external identical constant reference constant data of different size and memory 
		/// specializations to the internal storage.        
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs   Constant reference to external constant data buffer of identical size.
        /// @return          Reference to this pointer.
        template< boolean_t EXT_MEM2 >
        CArrayStorage& operator=(const CArrayStorage<const T,SIZE,EXT_MEM2>& rhs)
        {
            std::memcpy( &operator[](0), &rhs[0], SIZE*sizeof(T) );
            return *this;
        }

        /// Returns the reference to the element at given constant position.
        /// @param[in]       position   Constant position of the element.
        /// @return          Reference to the array element at given position.
        T& operator[](const size_type position)
        {
            // make sure data was initialized
            assert( m_data != 0 );
            // make sure position is between limits
            assert( (position >= 0) && (position < size()) );
            return m_data[position];
        }

        /// Returns the constant reference to the element at given constant position with constant object.
        /// @param[in]       position  Const position of the element
        /// @return          Constant reference to Array element at given position
        const T& operator[](const size_type position) const
        {
            // make sure data was initialized
            assert( m_data != 0 );
            // make sure position is between limits
            assert( (position >= 0) && (position < size()) );
            return m_data[position];
        }

        /// Attaches non-constant pointer constant external data to the internal pointer.
        /// @param[in] p_data   Pointer to the external constant data.
        /// @param[in] s_size   Constant size of the external array.
        void attach(T* const p_data, const sint32 s_size)
        {
            (void) s_size;
            assert (p_data != 0    );
            assert (SIZE  <= s_size);
            m_data = p_data;
        }

        /// Returns number of elements in the array.
        /// @return Number of elements in the Array
        static sint32 size()
        {
            return max_size();
        }

        /// Returns number of elements in the array.
        /// @return Number of elements in the Array
        static sint32 max_size()
        {
            return SIZE;
        }


        protected:
            T* m_data;
    };


    /// Provide Storage for Array, Specialized for external memory and constant data.
    /// @tparam T       The type of the elements
    /// @tparam SIZE    The size of the array
    template<class T, sint32 SIZE>
    class CArrayStorage<const T,SIZE,true>
    {
    public:
        typedef CArrayStorage<const T,SIZE,true> self;
        typedef int                              size_type;

        enum { MAX_SIZE = SIZE };

        /// Default constructor for specialization of external memory.
        /// Makes the storage invalid in debug mode.
        CArrayStorage()
#ifndef NDEBUG
            : m_data(0)
#endif
        {}


        /// Attaches constant pointer constant external data to the internal pointer.
        /// @param[in] p_data   Constant pointer to the external constant data.
        /// @param[in] s_size   Constant size of the external array.
        void attach(const T* const p_data, const sint32 s_size)
        {
            (void) s_size;
            assert (p_data != 0    );
            assert (SIZE  <= s_size);
            m_data = p_data;
        }

        /// Copy constructor that attaches external data of type constant pointer
		/// constant data to the internal pointer.
        /// Caller should ensure that s_size >= SIZE.
        /// @param[in] p_data   Constant pointer to the constant external data buffer.
        /// @param[in] s_size   Constant size of the external data buffer.
        CArrayStorage(const T* const p_data, const sint32 s_size)
        {
            attach(p_data, s_size);
        }

        /// Template copy constructor that attaches external data of type non-constant reference
		/// non-constant data to the internal pointer.
        /// Caller should ensure that SIZE2 >= SIZE.
        /// @tparam SIZE2    The size of the external array.
        /// @param[in]       data  Reference to the external non-constant data buffer of SIZE2.
        template< sint32 SIZE2 >
        CArrayStorage(       T (&data)[SIZE2] )
        {
            attach(&data[0], SIZE2);
        }

        /// Template copy constructor that attaches external data of type constant reference
		/// non-constant data to the internal pointer.
        /// Caller should ensure that SIZE2 >= SIZE.
        /// @tparam SIZE2    The size of the external array.
        /// @param[in]       data  Constant reference to the external non-constant data buffer of SIZE2.
        template< sint32 SIZE2 >
        CArrayStorage( const T (&data)[SIZE2] )
        {
            attach(&data[0], SIZE2);
        }

        /// Template copy constructor to attach external identical constant reference non-constant data  
		/// of different size and memory specializations to the internal pointer.
        /// @tparam SIZE2    The size of the external array.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs   Constant reference to the external non-constant data buffer of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArrayStorage( const CArrayStorage<T,SIZE2,EXT_MEM2>& rhs )
        {
            attach (&rhs[0], SIZE2);
        }

        /// Template copy constructor to attach external identical constant reference constant data  
		/// of different size and memory specializations to the internal pointer.
        /// @tparam SIZE2    The size of the external array.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs   Constant reference to the external constant data buffer of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArrayStorage( const CArrayStorage<const T,SIZE2,EXT_MEM2>& rhs )
        {
            attach (&rhs[0], SIZE2);
        }

        /// Returns the constant reference to the element at given constant position with constant object.
        /// Asserts that position is inside of the array.
        /// @param[in]       position  Constant position of the element
        /// @return          Constant reference to Array element at given position
        const T& operator[](const size_type position) const
        {
            assert( m_data != 0 );
            assert( (position >= 0) && (position < size()) );
            return m_data[position];
        }

        /// Returns number of elements in the array.
        /// @return          Number of elements in the Array
        static sint32 size()
        {
            return max_size();
        }

        /// Returns number of elements in the array.
        /// @return          Number of elements in the Array
        static sint32 max_size()
        {
            return SIZE;
        }


        protected:
            const T* m_data;
            // forbid shallow copy
            self& operator=(const self&);
    };

    /// Reimplementation of stl::array with fixed amount of memory
    /// @tparam T       The type of the elements
    /// @tparam SIZE    The size of the array
    /// @tparam EXT_MEM Flag whether the memory of the elements of the array is inside
    ///                 of this class (false) or outside of this class (true).
    ///                 Default is false, meaning internal memory.
    template < class T, sint32 SIZE, boolean_t EXT_MEM = false >
    class CArray
        : public CArrayStorage<T,SIZE,EXT_MEM>
    {
    public:
        typedef CArray       <T, SIZE, EXT_MEM> self;
        typedef CArrayStorage<T, SIZE, EXT_MEM> super;
        typedef sint32                          size_type;
        typedef typename remove_const<T>::type  type;
        typedef typename    add_const<T>::type  const_type;
        typedef typename remove_const<T>::type& reference;
        typedef typename    add_const<T>::type& const_reference;
        typedef typename remove_const<T>::type* pointer;
        typedef typename    add_const<T>::type* const_pointer;
        // iterator typedefs
#ifndef NDEBUG
        typedef cml::CLinMemIt< type, false, self >  it;
        typedef cml::CLinMemIt< type, true , self > cIt;
#else
        typedef cml::CLinMemIt< type, false >  it;
        typedef cml::CLinMemIt< type, true  > cIt;
#endif
        typedef it  iterator;
        typedef cIt const_iterator;

        
        // QAC Bug, since this using declaration is within the class scope.
        
        // (EXCLUDING CLASS SCOPE OR FUNCTION SCOPE USING-DECLARATIONS) shall not be used in header files."
        // parent imports
        using super::operator[];

        /// Default empty constructor.
        
        //   Our strategy is to have a late init using the reset call.
        //   This is necessary to be able to fulfill the SIL-SIL requirement
        CArray(){}

        /// Derived copy constructor that copies the given non-constant pointer 
        /// constant external data to the internal memory. 
        /// @param[in]       data    Non-constant pointer to the constant external data.
        /// @param[in]       size    Constant size of the external data buffer.
        CArray(       pointer const data, const sint32 size ) : super(data, size) {}

        /// Derived copy constructor that copies the given constant pointer 
        /// constant external data to the internal memory. 
        /// @param[in]       data    Constant pointer to the constant external data.
        /// @param[in]       size    Constant size of the external data buffer.
        CArray( const_pointer const data, const sint32 size ) : super(data, size) {}

        /// Derived template copy constructor that copies the given non-constant reference 
        /// non-constant external data to the internal memory. 
        /// @tparam SIZE2    The size of the external data.
        /// @param[in]       data    Non-constant reference to the non-constant external data of SIZE2.
        
        /// Constructor with pointer to array of possibly different size than this.
        template< sint32 SIZE2 >
        CArray(       type (&data)[SIZE2] ) : super(data) {}

        /// Derived template copy constructor that copies the given non-constant reference 
        /// constant external data to the internal memory. 
        /// @tparam SIZE2    The size of the external data.
        /// @param[in]       data    Non-constant reference to the constant external data of SIZE2.
        template< sint32 SIZE2 >
        CArray( const_type (&data)[SIZE2] ) : super(data) {}

        /// Derived template copy constructor that copies or attaches constant reference external identical data 
        /// with non-constant data of different size and memory specializations to the internal data storage.
        /// @tparam SIZE2    The size of the external data.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Constant reference to the non-constant identical external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArray( const CArrayStorage<      type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Derived template copy constructor that copies or attaches constant reference external identical data 
        /// with constant data of different size and memory specializations to the internal data storage.
        /// @tparam SIZE2    The size of the external data.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Constant reference to the constant identical external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArray( const CArrayStorage<const_type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Derived template copy constructor that copies or attaches non-constant reference external identical data 
        /// with non-constant data of different size and memory specializations to the internal data storage.
        /// @tparam SIZE2    The size of the external data.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Non-constant reference to the non-constant identical external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArray( CArrayStorage<      type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Derived template copy constructor that copies or attaches non-constant reference external identical data 
        /// with constant data of different size and memory specializations to the internal data storage.
        /// @tparam SIZE2    The size of the external data.
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Non-constant reference to the constant identical external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CArray( CArrayStorage<const_type, SIZE2, EXT_MEM2>& rhs ) : super(rhs) {}

        /// Copy external identical constant reference non-constant data of identical size 
		/// to the internal storage. 
        /// This is not automatically generated by templated version
        
        /// @param[in]       rhs    Constant reference to the external data buffer of identical size.
        /// @return          Reference to this pointer.
        self& operator=(const self& rhs)
        {
            super::operator=(rhs);
            return *this;
        }

        /// Copy external identical constant reference non-constant data of identical size  
		/// and memory specializations to the internal storage. 
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Constant reference to the external non-constant data of identical size.
        /// @return          Reference to this pointer.
        template< boolean_t EXT_MEM2 >
        self& operator=(const CArray<type,SIZE,EXT_MEM2>& rhs)
        {
            super::operator=(rhs);
            return *this;
        }

        /// Copy external identical constant reference constant data of identical size  
		/// and memory specializations to the internal storage. 
        /// @tparam EXT_MEM2 Flag whether the memory of the elements of the array is inside
        ///                  of this class (false) or outside of this class (true).
        ///                  Default is false, meaning internal memory.
        /// @param[in]       rhs    Constant reference to the external constant data of identical size.
        /// @return          Reference to this pointer.
        template< boolean_t EXT_MEM2 >
        self& operator=(const CArray<const_type,SIZE,EXT_MEM2>& rhs)
        {
            super::operator=(rhs);
            return *this;
        }

        /// Reference to the element at given constant position.
        /// @param[in]       position    Constant position of the element.
        /// @return          Reference to the element at given position.
        reference at(const size_type position)
        {
            return operator[](position);
        }

        /// Constant reference to the element at given position.
        /// @param[in]       position    Constant position of the element.
        /// @return          Constant reference to the element at given position.
        const_reference at(const size_type position) const
        {
            return operator[](position);
        }

        /// Reference to the first element of internal storage.
        /// @return          Reference to the element at first position.
        reference front()
        {
            return operator[](0);
        }

        /// Constant reference to the first element of internal storage.
        /// @return          Constant reference to the element at first position.
        const_reference front() const
        {
            return operator[](0);
        }

        /// Reference to the first element of internal storage.
        /// @return          Reference of element at last position.
        reference back()
        {
            return operator[](SIZE-1);
        }

        /// Constant reference to the last element of internal storage.
        /// @return          Constant reference of element at last position.
        const_reference back() const
        {
            return operator[](SIZE-1);
        }

        /// Fills the whole array with input value.
        /// @param[in]       val   Constant reference to the input value.
        /// @return          void
        void fill(const_reference val)
        {
            for (sint32 i=0; i<SIZE; i++)
            {
                operator[](i) = val;
            }
        }

        /// Returns an iterator to the beginning of the CArray.
        /// In debug case, it is ensured via assert that the array
        /// bounds are never exceeded.
        /// @return          Returns an iterator to the beginning of the CArray.
#ifndef NDEBUG
        it  begin()        { return it (&m_data[0], this); }
#else
        it  begin()        { return it (&m_data[0]); }
#endif

        /// Returns a constant iterator to the beginning of the CArray.
        /// In debug case, it is ensured via assert that the array
        /// bounds are never exceeded.
        /// @return          Returns a constant iterator to the beginning of the CArray.
#ifndef NDEBUG
        cIt begin()  const { return cIt(&m_data[0], this); }
#else
        cIt begin()  const { return cIt(&m_data[0]); }
#endif

        /// Returns a constant iterator to the beginning of the constant CArray.
        /// In debug case, it is ensured via assert that the array
        /// bounds are never exceeded.
        /// @return          Returns a constant iterator to the beginning of the constant CArray.
#ifndef NDEBUG
        cIt cbegin() const { return cIt(&m_data[0], this); }
#else
        cIt cbegin() const { return cIt(&m_data[0]); }
#endif

        /// Returns an iterator to the position after the end of the CArray.
        /// In debug case, it is ensured via assert that the array
        /// bounds are never exceeded.
        /// @return          Returns an iterator to the position after the end of the CArray.
#ifndef NDEBUG
        it  end()        { return it (&m_data[SIZE], this); }
#else
        it  end()        { return it (&m_data[SIZE]); }
#endif

        /// Returns a constant iterator to the position after the the end of the CArray.
        /// In debug case, it is ensured via assert that the array
        /// bounds are never exceeded.
        /// @return          Returns a constant iterator to the position after the the end of the CArray.
#ifndef NDEBUG
        cIt end()  const { return cIt(&m_data[SIZE], this); }
#else
        cIt end()  const { return cIt(&m_data[SIZE]); }
#endif

        /// Returns a constant iterator to the position after the the end of the CArray.
        /// In debug case, it is ensured via assert that the array
        /// bounds are never exceeded.
        /// @return          Returns a constant iterator to the position after the the end of the CArray.
#ifndef NDEBUG
        cIt cend() const { return cIt(&m_data[SIZE], this); }
#else
        cIt cend() const { return cIt(&m_data[SIZE]); }
#endif

            // This is only required for TI-Compiler to be able to compile
        protected:
            
            // QAC Bug, since this using declaration is within the class scope.
            
            // (EXCLUDING CLASS SCOPE OR FUNCTION SCOPE USING-DECLARATIONS) shall not be used in header files."
            using super::m_data;
            // End of section required for TI-Compiler
    };
}

#endif //CML_STL_ARRAY_H__

