    

#ifndef CML_STL_VECTOR_H__
#define CML_STL_VECTOR_H__

#include "cml_stl_array.h"

namespace cml
{

    /// Stores a list of "things" and gives the possibility to insert, append, clear.
    /// @tparam T       The type of the elements
    /// @tparam SIZE    The size of the array    
    /// @tparam EXT_MEM Flag whether the memory of the elements of the array is inside
    ///                 of this class (false) or outside of this class (true).
    ///                 Default is false, meaning internal memory.
    template <class T, sint32 SIZE, boolean_t EXT_MEM = false >
    class CVector
        : public CArray<T, SIZE, EXT_MEM>
    {
    public:
        typedef CVector<T, SIZE, EXT_MEM>       self;
        typedef CArray <T, SIZE, EXT_MEM>       super;
        typedef sint32                          size_type;
        typedef typename remove_const<T>::type  type;
        typedef typename    add_const<T>::type  const_type;
        typedef typename remove_const<T>::type& reference;
        typedef typename    add_const<T>::type& const_reference;
        typedef typename remove_const<T>::type* pointer;
        typedef typename    add_const<T>::type* const_pointer;
        typedef int                             difference_type;
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


        /// Default empty constructor. 
        CVector() : super(), m_size(0) {}

        /// Copy Constructor with reference to a non-constant external data and size. 
        /// Copies external data to the internal storage.
        /// @tparam          SIZE2   The size of the array 
        /// @param[in]       data    Reference to the external data of SIZE2 
        template< sint32 SIZE2 >
        explicit CVector(       type (&data)[SIZE2] ) : super(data), m_size(0) {}

        /// Copy Constructor with reference to constant external data.
        /// Copies external data to the internal storage.
        /// @tparam          SIZE2   The size of the array
        /// @param[in]       data    Reference to constant external data of SIZE2 
        template< sint32 SIZE2 >
        explicit CVector( const_type (&data)[SIZE2] ) : super(data), m_size(0) {}

        /// Copy constructor with reference to non-constant external data of possibly different size than this.
		/// Copies external data to the internal storage.
        /// @tparam          SIZE2     The size of the external data.
        /// @param[in]       p_data    Reference to the non-constant external data of SIZE2.
        /// @param[in]       s_size    Constant external data size.
        template< sint32 SIZE2 >
        explicit CVector(       type (&p_data)[SIZE2], const size_type s_size ) : super(p_data), m_size(s_size) {}

        /// Copy constructor with reference to constant external data of possibly different size than this.
		/// Copies external data to the internal storage.
        /// @tparam          SIZE2     The size of the external data.
        /// @param[in]       p_data    Reference to constant external data of SIZE2.
        /// @param[in]       s_size    Constant size
        template< sint32 SIZE2 >
        explicit CVector( const_type (&p_data)[SIZE2], const size_type s_size ) : super(p_data), m_size(s_size) {}

        /// Copy constructor with constant reference to non-constant external data of possibly 
		/// different size and memory setting than this.
		/// Copies external data to the internal storage.
        /// @tparam          SIZE2     The size of the external data.   
        /// @tparam          EXT_MEM2  Flag whether the memory of the elements of the array is inside
        ///                            of this class (false) or outside of this class (true).
        ///                            Default is false, meaning internal memory.
        /// @param[in]       rhs       Constant reference to non-constant external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CVector( const CArrayStorage<      type, SIZE2, EXT_MEM2>& rhs ) : super(rhs), m_size(0) {}

        /// Copy constructor with constant reference to constant external data of possibly 
		/// different size and memory setting than this.
		/// Copies external data to the internal storage.
        /// @tparam          SIZE2     The size of the external data.    
        /// @tparam          EXT_MEM2  Flag whether the memory of the elements of the array is inside
        ///                            of this class (false) or outside of this class (true).
        ///                            Default is false, meaning internal memory.
        /// @param[in]       rhs       Constant reference to constant external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CVector( const CArrayStorage<const_type, SIZE2, EXT_MEM2>& rhs ) : super(rhs), m_size(0) {}

        /// Copy constructor with non-constant reference to non-constant external data of possibly 
		/// different size and memory setting than this.
		/// Copies external data to the internal storage.
        /// @tparam          SIZE2     The size of the external data.
        /// @tparam          EXT_MEM2  Flag whether the memory of the elements of the array is inside
        ///                            of this class (false) or outside of this class (true).
        ///                            Default is false, meaning internal memory.
        /// @param[in]       rhs       Non-constant reference to non-constant external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CVector( CArrayStorage<      type, SIZE2, EXT_MEM2>& rhs ) : super(rhs), m_size(0) {}

        /// Copy constructor with non-constant reference to constant external data of possibly 
		/// different size and memory setting than this.
		/// Copies external data to the internal storage.
        /// @tparam          SIZE2     The size of the external data.  
        /// @tparam          EXT_MEM2  Flag whether the memory of the elements of the array is inside
        ///                            of this class (false) or outside of this class (true).
        ///                            Default is false, meaning internal memory.
        /// @param[in]       rhs       Non-constant reference to constant external data of SIZE2.
        template< sint32 SIZE2, boolean_t EXT_MEM2 >
        explicit CVector( CArrayStorage<const_type, SIZE2, EXT_MEM2>& rhs ) : super(rhs), m_size(0) {}

        /// Makes a copy of constant reference non-constant external identical data of 
		/// same size to the internal data storage.
        /// This is not automatically generated by templated version 
        
        /// @param[in]       rhs    Constant reference to external data of identical size.
        /// @return          Reference to this pointer. 
        self& operator=(const self& rhs)
        {
            super::operator=(rhs);
            m_size = rhs.size();
            return *this;
        }

        /// Makes a copy of constant reference non-constant external data of possibly 
		/// different size and memory settings to internal storage.
        /// @tparam          EXT_MEM2  Flag whether the memory of the elements of the array is inside
        ///                            of this class (false) or outside of this class (true).
        ///                            Default is false, meaning internal memory.
        /// @param[in]       rhs       Reference to non-constant external data of identical size.
        /// @return          Reference to this pointer.
        template< boolean_t EXT_MEM2 >
        self& operator=(const CVector<      type, SIZE, EXT_MEM2>& rhs)
        {
            super::operator=(rhs);
            m_size = rhs.size();
            return *this;
        }

        /// Makes a copy of constant reference constant external data of possibly 
		/// different size and memory settings to internal storage.
        /// @tparam          EXT_MEM2  Flag whether the memory of the elements of the array is inside
        ///                            of this class (false) or outside of this class (true).
        ///                            Default is false, meaning internal memory.
        /// @param[in]       rhs       Reference to constant external data of identical size.
        /// @return          Reference to this pointer.
        template< boolean_t EXT_MEM2 >
        self& operator=(const CVector<const_type, SIZE, EXT_MEM2>& rhs)
        {
            super::operator=(rhs);
            m_size = rhs.size();
            return *this;
        }

        /// Clear the vector by setting size to zero.
        /// Memory is not cleared.
        /// @return          void 
        void clear()
        {
            m_size = 0;
        }

        // This is only required for TI-Compiler to be able to compile
        
        // QAC Bug, since this using declaration is within the class scope.
        
        // (EXCLUDING CLASS SCOPE OR FUNCTION SCOPE USING-DECLARATIONS) shall not be used in header files."
        using super::max_size;
        // End of section required for TI-Compiler

        /// Return number of elements in the array with constant object type.
        /// In this member function, no member data is modified.
        /// @return          Number of elements in the Array 
        size_type size() const
        {
            return m_size;
        }

        /// Check whether the vector list is empty or not with constant object type.        
        /// In this member function, no member data is modified.
        /// @return          Return 1 if vector list is empty, 0 if not 
        boolean_t empty() const
        {
            return (m_size == 0);
        }

        /// Check whether the vector list is full or not with constant object type.        
        /// In this member function, no member data is modified.
        /// @return          Return 1 if vector list is full, 0 if not 
        boolean_t full() const
        {
            return (m_size >= SIZE);
        }

        /// Returns the current last element of the vector.
        /// @return          Reference to current last element of the vector 
        reference back()
        {
            return operator[](size()-1);
        }

        /// Returns the current last element of the vector
        /// Constant member function returning constant reference to  
        /// the current last element of the vector.
        /// @return          Constant reference to current last element of the vector 
        const_reference back() const
        {
            return operator[](size()-1);
        }

        /// Resizes the vector to new size and fills the remaining elements 
        /// with constant value.
        /// Asserts if input size is less than or equal to SIZE.
        /// @param[in]       sz    Constant reference to new constant size of the vector.
        /// @param[in]       c     Constant reference to value to be filled.
        /// @return          None
        void resize( const size_type& sz, const_reference c = type() )
        {
            assert(sz <= SIZE);
            if (m_size < sz)
            {
                std::fill(&m_data[m_size], &m_data[sz], c);
            }
            m_size = sz;
        }

        /// Resizes the vector to new size without initialization.
        /// Asserts if input size is less than or equal to SIZE.
        /// @param[in]       sz    Constant reference to new size of the vector.
        /// @return          None  
        void resize_uninitialized ( const size_type& sz )
        {
            assert(sz <= SIZE);
            m_size = sz;
        }

        /// Assign value given number of times in a new, cleared vector.        
        /// Any previously containing data is lost.
        /// Asserts if input size is less than or equal to SIZE.
        /// @param[in]       n     Number of times the value to be filled.
        /// @param[in]       u     Constant reference to the value to be filled.
        /// @return          None 
        void assign(const size_type n, const_reference u)
        {
            assert(n <= SIZE);
            std::fill(&operator[](0), &operator[](n), u);
            m_size = n;
        }

        /// Append input element at the end of vector.
        /// Asserts if size is less than SIZE.
        /// @param[in]       x     Constant reference to value to be filled.
        /// @return          None  
        void push_back(const_reference x)
        {
            assert (size() < SIZE);
            ++m_size;
            operator[](m_size - 1) = x;
        }

        /// Enlarge the vector by one and give back the new element.
        /// Asserts if size is less than SIZE.
        /// @return          Reference to new element after push back.
        reference push_back()
        {
            assert (size() < SIZE);
            ++m_size;
            return operator[](m_size - 1);
        }

        /// Delete the last element in vector by decrementing the pointer.
        /// Memory is not cleared.
        /// Asserts if size is greater than zero.
        /// @return          void 
        void pop_back()
        {
            assert (size() > 0);
            --m_size;
        }

        /// Delete the last given number of elements in vector by decrementing 
		/// the pointer by number of elements.
        /// Memory is not cleared.
        /// Asserts if size is greater than or equal to n.
        /// @param[in]       n    Number of elements to be deleted.
        /// @return          None  
        void pop_back(size_type n)
        {
            assert (size() >= n);
            m_size -= n;
        }

        /// Insert an element at given position, by pushing the next 
        /// elements to the higher position.
        /// Memory is not cleared.
        /// Asserts if size is less than max_size and position within array boundary.
        /// @param[in]       position    Constant iterator reference to position of the element.
        /// @param[in]       p           Constant reference to value to be inserted.
        /// @return          Iterator to the position of inserted element 
        it insert(const cIt& position, const_reference p)
        {
            //check if array has enough space
            assert ( ( size()    <  max_size() )
                &&     ( position  >= begin()    )
                &&     ( position  <= end()      ) );

            // increment size of the list
            // invalidates all iterators!
            ++m_size;

            // move all following elements 1 position to the right
            it i = end().operator-(1);
            for(; i.operator>(position); --i)
            {
                *i = i[-1];
            }
            // write value to new position
            *i = p;

            return i;
        }

        /// Insert elements from iterator first to last, by pushing the next 
        /// elements to the higher position.
        /// Asserts if new size is less than max_size and position within array boundary.
        /// @tparam          InputIterator  Iterator to the input vector.
        /// @param[in]       position       Iterator to the start position of the elements to be inserted.
        /// @param[in]       first          Iterartor to the first element to be inserted.
        /// @param[in]       last           Iterartor to the last element to be inserted.
        /// @return          None 
        template <class InputIterator>
        void insert (it position, InputIterator first, InputIterator last)
        {
            const difference_type numInsertElements = last - first;

            //check if array has enough space
            assert ( ( size() + numInsertElements <= max_size() )
                &&     ( position  >= begin()    )
                &&     ( position  <= end()      ) );

            // increment size of the list
            // invalidates all iterators!
            m_size += numInsertElements;

            // move all following elements numInsertElements positions to the right
            it i = end().operator-(1);
            for(; i.operator>(position.operator+(numInsertElements-1)); --i)
            {
                *i = i[-numInsertElements];
            }
            // write values to new position
            for (sint32 k=0; k<numInsertElements; k++)
            {
                position[k] = *first;
                ++first;
            }
        }

        /// Delete element from the position of array, move back the next 
        /// elements to the previous position.
        /// Asserts if size is greater than zero and position within array boundary
        /// @param[in]       position    Constant iterator reference to position of the element to be deleted.
        /// @return          Iterator to the position of the deleted element. 
        it erase(const cIt& position)
        {
            //check if array has enough space
            assert ( ( size()    >  0       )
                &&     ( position  >= begin() )
                &&     ( position  <  end()   ) );

            // move all following elements 1 position to the right
            difference_type idxStart = position - begin();
            difference_type idxEnd   = static_cast<difference_type>(m_size) - 1;
            for(difference_type i = idxStart; i < idxEnd; ++i)
            {
                operator[](i) = operator[](i+1);
            }

            // decrement size of the list
            // invalidates all iterators!
            --m_size;

            return begin() + idxStart;
        }

#ifndef NDEBUG
        /// Returns the element at given position
        /// Asserts that position is inside of the array
        /// @param[in]       position    Constant position of the element 
        /// @return          Reference to the Array element at given position 
        reference operator[](const size_type position)
        {
            //make sure position is between limits - otherwise return zero's element
            assert(position >= 0 && position < size());
            return m_data[position];
        }

        /// Returns a constant reference to the element at given position
        /// Asserts that position is inside of the array
        /// @param[in]       position    Constant position of the element 
        /// @return          Constant reference to the Array element at given position
        const_reference operator[](const size_type position) const
        {
            assert(position >= 0 && position < size());
            return m_data[position];
        }

        /// Returns a reference of the element at position.
        /// @param[in]       position    Constant position of the element 
        /// @return          Reference to the Array element at given position 
        reference       at(const size_type position)       { return operator[](position); }

        /// Returns a constant reference of the element at given position.
        /// @param[in]       position    Constant position of the element 
        /// @return          Constant reference to the Array element at given position 
        const_reference at(const size_type position) const { return operator[](position); }

        /// Returns a reference of the first element.
        /// @return          Reference to the Array element at first position 
        reference       front()       { return operator[](0); }

        /// Constant member function which returns a constant reference 
        /// of the first element.
        /// @return          Constant reference of Array element at first position 
        const_reference front() const { return operator[](0); }

        /// Returns an iterator to the begin of the CArray
        /// In debug case, it is ensured via assert that the array 
        /// bounds are never exceeded
        /// @return          An iterator to the begin of the CArray
        it  begin()        { return it (&m_data[0], this); }

        /// Constant member function which returns a constant iterator to the 
        /// begin of the CArray.
        /// @return          A constant iterator to the begin of the CArray
        cIt begin()  const { return cIt(&m_data[0], this); }

        /// Constant member function which returns a constant iterator 
        /// to the begin of the CArray.
        /// @return          A constant iterator to the begin of the CArray
        cIt cbegin() const { return cIt(&m_data[0], this); }

        /// Returns an iterator to the position after the end of the CArray.
        /// @return          An iterator to the position after the end of the CArray.
        it  end()        { return it (&m_data[size()], this); }

        /// Constant member function which returns a constant iterator 
        /// to the position after the end of the CArray.
        /// @return          A constant iterator to the position after the end of the CArray.
        cIt end()  const { return cIt(&m_data[size()], this); }

        /// Constant member function which returns a constant iterator 
        /// to the position after the end of the CArray.
        /// @return          A constant iterator to the position after the end of the CArray.
        cIt cend() const { return cIt(&m_data[size()], this); }


#else //#ifndef NDEBUG
        // import functionality
        
        // QAC Bug, since this using declaration is within the class scope.
        
        // (EXCLUDING CLASS SCOPE OR FUNCTION SCOPE USING-DECLARATIONS) shall not be used in header files."
        using super::operator[];
        using super::at;
        using super::front;

        using super::begin;
        using super::cbegin;

        /// Returns an iterator to the position after the end of the vector.
        /// @param[in]       None
        /// @return          An iterator to the position after the end of the vector.
        it  end()        { return it (&m_data[size()]); }

        /// Returns a constant iterator to the position after the end of the 
        /// vector with constant object type.              
        /// @param[in]       None
        /// @return          A constant iterator to the position after the end of the vector.
        cIt end()  const { return cIt(&m_data[size()]); }

        /// Returns a constant iterator to the position after the end of the 
        /// constant vector with constant object type.                               
        /// @param[in]       None
        /// @return          A constant iterator to the position after the end of the vector.
        cIt cend() const { return cIt(&m_data[size()]); }
#endif //#ifndef NDEBUG

    protected:
        size_type m_size;
        // This is only required for TI-Compiler to be able to compile
        
        // QAC Bug, since this using declaration is within the class scope.
        
        // (EXCLUDING CLASS SCOPE OR FUNCTIN SCOPE USING-DECLARATIONS) shall not be used in header files."
        using super::m_data;
        // End of section required for TI-Compiler

    private:
        //Prevent from being used
        void fill(const_reference val);
    };
}

#endif //CML_STL_VECTOR_H__

