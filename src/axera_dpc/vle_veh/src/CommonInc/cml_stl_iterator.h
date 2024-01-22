    

#ifndef CML_STL_ITERATOR_H__
#define CML_STL_ITERATOR_H__

#include <cassert>
#include <iterator>

#include "cml_stl_type_traits.h"

namespace cml
{

    /// @tparam T                 The type of the element
    /// @tparam IS_CONST_ITERATOR Flag whether the iterator is constant or not.
    /// @tparam CONTAINER_TYPE    The type of the container

#ifndef NDEBUG
    template<class T, boolean_t IS_CONST_ITERATOR, class CONTAINER_TYPE>
#else
    template<class T, boolean_t IS_CONST_ITERATOR>
#endif
    class CBaseIt
    {
    public:
#ifndef NDEBUG
        typedef CBaseIt<T,IS_CONST_ITERATOR,CONTAINER_TYPE> self;
        typedef CONTAINER_TYPE                              container_type;
#else
        typedef CBaseIt<T,IS_CONST_ITERATOR>                self;
#endif

        typedef typename conditional<IS_CONST_ITERATOR, typename add_const<T>::type, T>::type value_type;
        typedef value_type* pointer;
        typedef value_type& reference;
        typedef int         difference_type;

#ifndef NDEBUG

        /// Empty constructor. 
        CBaseIt () : m_p(0), m_pContainer(0) {}

        /// Explicit empty constructor with additional arguments for 
        /// initial memory position, static container for dynamic 
        /// boundary and container equality checking.
        /// @param[in]       _p             Pointer to initial memory position  
        /// @param[in]       _pContainer    Constant pointer to static container for dynamic 
        ///   			                    boundary and container equality checking
        explicit CBaseIt (
            pointer               _p,           // initial memory position
            const container_type* _pContainer ) // static container for dynamic boundary and container equality checking
            : m_p         ( _p )
            , m_pContainer( _pContainer )
        {}

        /// Empty constructor with an identical iterator.
        /// @param[in]       rhs    Constant reference to identical iterator. 
        CBaseIt( const CBaseIt<T,false,container_type>& rhs )
            : m_p         ( rhs.m_p )
            , m_pContainer( rhs.m_pContainer )
        {}

#else
        /// Empty constructor. 
        CBaseIt () {}
        
        /// Empty constructor with additional argument for initial memory 
        /// position.
        /// @param[in]       _p : Pointer to initial memory position
        explicit CBaseIt (pointer _p)
            : m_p( _p )
        {}

        /// Empty constructor.
        /// @param[in]       rhs : Constant reference to identical iterator.  
        CBaseIt( const CBaseIt<T,false>& rhs )
            : m_p ( rhs.m_p )
        {}
#endif //#ifndef NDEBUG


#ifndef NDEBUG
        template<class, boolean_t, class>
#else
        template<class, boolean_t>
#endif
        friend class CBaseIt;
        
        /// Dereferencing of the element.
        /// Asserts if iterators are within container boundary.
        /// @return          Current pointer
        pointer operator->() const
        {
            assert ( (m_pContainer != 0)
                &&     (m_p          != 0)
                &&     ((*this) >= m_pContainer->begin())
                &&     ((*this) <  m_pContainer->end()  ) );
            return m_p;
        }

        /// Indirection of the element.
        /// @return             Reference to the element.
        reference operator*() const
        {
            return *(operator->());
        }

        /// Returns comparison result of two iterator objects.
        /// True, if both object references are same else False.
        /// Asserts if iterator object references are equal.
        /// @tparam      IS_CONST_ITERATOR_2 Flag whether the iterator is constant or not.
        /// @param[in]       rhs    Constant reference to identical iterator object.   
        /// @return                 True, if both object references are same, else False.
        template<boolean_t IS_CONST_ITERATOR_2>
#ifndef NDEBUG
        boolean_t operator== (const CBaseIt<T,IS_CONST_ITERATOR_2,CONTAINER_TYPE>& rhs) const
#else
        boolean_t operator== (const CBaseIt<T,IS_CONST_ITERATOR_2>& rhs) const
#endif
        {
            // check that rhs is from the same memory
            assert(m_pContainer != NULL);
            assert(m_pContainer == rhs.m_pContainer);

            return m_p == rhs.m_p;
        }

        /// Returns comparison result of two iterator objects.
        /// True, if both object references are not same, else False.
        /// Asserts if iterator object references are equal.
        /// @tparam      IS_CONST_ITERATOR_2 Flag whether the iterator is constant or not.
        /// @param[in] rhs   Constant reference to identical iterator object.   
        /// @return          True, if both object references are not same, else False.
        template<boolean_t IS_CONST_ITERATOR_2>
#ifndef NDEBUG
        boolean_t operator!= (const CBaseIt<T,IS_CONST_ITERATOR_2,CONTAINER_TYPE>& rhs) const
#else
        boolean_t operator!= (const CBaseIt<T,IS_CONST_ITERATOR_2>& rhs) const
#endif
        {
            // check that rhs is from the same memory
            assert(m_pContainer != NULL);
            assert(m_pContainer == rhs.m_pContainer);

            return m_p != rhs.m_p;
        }

        /// Returns comparison result of two iterator objects.
        /// True, if first iterator object reference is greater than or 
        /// equal to second, else False.
        /// Asserts if iterator object references are equal.
        /// @tparam      IS_CONST_ITERATOR_2 Flag whether the iterator is constant or not.
        /// @param[in] rhs   Constant reference to identical iterator object.   
        /// @return          True, if first iterator object reference is greater than or 
        ///                  equal to second, else False. 
        template<boolean_t IS_CONST_ITERATOR_2>
#ifndef NDEBUG
        boolean_t operator>= (const CBaseIt<T,IS_CONST_ITERATOR_2,CONTAINER_TYPE>& rhs) const
#else
        boolean_t operator>= (const CBaseIt<T,IS_CONST_ITERATOR_2>& rhs) const
#endif
        {
            // check that rhs is from the same memory
            assert(m_pContainer != NULL);
            assert(m_pContainer == rhs.m_pContainer);

            return m_p >= rhs.m_p;
        }

        /// Returns comparison result of two iterator objects.
        /// True, if first iterator object reference is less than or 
        /// equal to second, else False.
        /// Asserts if iterator object references are equal.
        /// @tparam      IS_CONST_ITERATOR_2 Flag whether the iterator is constant or not.
        /// @param[in] rhs   Constant reference to identical iterator object.   
        /// @return          True, if first iterator object reference is less than or 
        ///  	             equal to second, else False.
        template<boolean_t IS_CONST_ITERATOR_2>
#ifndef NDEBUG
        boolean_t operator<= (const CBaseIt<T,IS_CONST_ITERATOR_2,CONTAINER_TYPE>& rhs) const
#else
        boolean_t operator<= (const CBaseIt<T,IS_CONST_ITERATOR_2>& rhs) const
#endif
        {
            // check that rhs is from the same memory
            assert(m_pContainer != NULL);
            assert(m_pContainer == rhs.m_pContainer);

            return m_p <= rhs.m_p;
        }

        /// Returns comparison result of two iterator objects.
        /// True, if first iterator object reference is greater than  
        /// second, else False.
        /// Asserts if iterator object references are equal.
        /// @tparam      IS_CONST_ITERATOR_2 Flag whether the iterator is constant or not.
        /// @param[in] rhs   Constant reference to identical iterator object.   
        /// @return          True, if first iterator object reference is greater than  
        ///                  second, else False.  
        template<boolean_t IS_CONST_ITERATOR_2>
#ifndef NDEBUG
        boolean_t operator> (const CBaseIt<T,IS_CONST_ITERATOR_2,CONTAINER_TYPE>& rhs) const
#else
        boolean_t operator> (const CBaseIt<T,IS_CONST_ITERATOR_2>& rhs) const
#endif
        {
            // check that rhs is from the same memory
            assert(m_pContainer != NULL);
            assert(m_pContainer == rhs.m_pContainer);

            return m_p > rhs.m_p;
        }

        /// Returns comparison result of two iterator objects.
        /// True, if first iterator object reference is less than  
        /// second, else False.
        /// Asserts if iterator object references are equal.
        /// @tparam      IS_CONST_ITERATOR_2 Flag whether the iterator is constant or not.
        /// @param[in]  rhs  Constant reference to identical iterator object.   
        /// @return          True, if first iterator object reference is less than  
        /// 				 second, else False.
        template<boolean_t IS_CONST_ITERATOR_2>
#ifndef NDEBUG
        boolean_t operator< (const CBaseIt<T,IS_CONST_ITERATOR_2,CONTAINER_TYPE>& rhs) const
#else
        boolean_t operator< (const CBaseIt<T,IS_CONST_ITERATOR_2>& rhs) const
#endif
        {
            // check that rhs is from the same memory
            assert(m_pContainer != NULL);
            assert(m_pContainer == rhs.m_pContainer);

            return m_p < rhs.m_p;
        }

        /// Returns the distance `n` in memory of two iterator objects.
        /// Where distance is defined as `n * sizeof(T)`.
        /// Asserts if iterator object references are equal.
        /// @tparam      IS_CONST_ITERATOR_2 Flag whether the iterator is constant or not.
        /// @param[in]       rhs   Constant reference to identical iterator object.  
        /// @return                The distance `n` in memory of two iterator objects.
        template<boolean_t IS_CONST_ITERATOR_2>
#ifndef NDEBUG
        difference_type memory_distance(const CBaseIt<T,IS_CONST_ITERATOR_2,CONTAINER_TYPE>& rhs) const
#else
        difference_type memory_distance(const CBaseIt<T,IS_CONST_ITERATOR_2>& rhs) const
#endif
        {
            // check that rhs is from the same memory
            assert(m_pContainer != NULL);
            assert(m_pContainer == rhs.m_pContainer);

            return static_cast<difference_type>(m_p - rhs.m_p);
        }

        protected:
            pointer m_p;

#ifndef NDEBUG
        const container_type* m_pContainer;
#endif
    };
    
    /// Generic iterator for elements in class of class T which lie linearly in memory
    /// @tparam T                 The type of the element
    /// @tparam IS_CONST_ITERATOR Flag whether the iterator is constant or not.
    /// @tparam CONTAINER_TYPE    The type of the container
    /// @tparam STRIDE            The stride of the container
#ifndef NDEBUG
    template <class T, boolean_t IS_CONST_ITERATOR, class CONTAINER_TYPE, sint32 STRIDE = 1>
    class CLinMemIt : public CBaseIt<T,IS_CONST_ITERATOR,CONTAINER_TYPE>
#else
    template <class T, boolean_t IS_CONST_ITERATOR, sint32 STRIDE = 1>
    class CLinMemIt : public CBaseIt<T,IS_CONST_ITERATOR>
#endif
    {
    public:
#ifndef NDEBUG
        typedef CLinMemIt<T,IS_CONST_ITERATOR,CONTAINER_TYPE,STRIDE> self;
        typedef CLinMemIt<T,false            ,CONTAINER_TYPE,STRIDE> self_non_const;
        typedef CBaseIt  <T,IS_CONST_ITERATOR,CONTAINER_TYPE>        super;
#else
        typedef CLinMemIt<T,IS_CONST_ITERATOR,STRIDE>                self;
        typedef CLinMemIt<T,false            ,STRIDE>                self_non_const;
        typedef CBaseIt  <T,IS_CONST_ITERATOR>                       super;
#endif

        typedef          std::random_access_iterator_tag                 iterator_category;
        typedef typename remove_const<typename super::value_type>::type* pointer;
        typedef typename add_const   <typename super::value_type>::type* const_pointer;
        typedef typename super::pointer                                  const_noconst_pointer;
        typedef typename super::reference                                reference;
        typedef typename super::difference_type                          difference_type;

        // Begin section needed only for TI compiler
        
        // reason: QAC Bug, since this using declaration is within the class scope.
        
        //         (EXCLUDING CLASS SCOPE OR FUNCTION SCOPE USING-DECLARATIONS) shall not be used in header files.
        using super::operator ->;
        // End section needed only for TI compiler
        
#ifndef NDEBUG
        /// Empty constructor.
        CLinMemIt () : super() {}

        /// Explicit empty constructor with additional arguments for 
        /// initial memory position, static container for dynamic 
        /// boundary and container equality checking.
        /// @param[in]       _p            Pointer to initial memory position  
        /// @param[in]       _pContainer   Constant pointer to static container for dynamic 
        ///								   boundary and container equality checking
        explicit CLinMemIt (
            const_noconst_pointer _p,           // initial memory position
            const CONTAINER_TYPE* _pContainer ) // static container for dynamic boundary and container equality checking
            : super( _p, _pContainer )
        {}

        /// Explicit empty constructor with additional arguments for 
        /// initial memory position, static container for dynamic 
        /// boundary and container equality checking.
        /// @param[in]       _p             Pointer to initial memory position  
        /// @param[in]       _pContainer    Constant pointer to static container for dynamic 
        ///								    boundary and container equality checking 
        explicit CLinMemIt (
            const_noconst_pointer _p,           // initial memory position
            const CONTAINER_TYPE* _pContainer,  // static container for dynamic boundary and container equality checking
            const_pointer,
            const_pointer )
            : super( _p, _pContainer )
        {}

        /// Explicit empty constructor.
        /// @tparam IS_CONST_ITERATOR2 Flag whether the iterator is constant or not.
        /// @param[in]       rhs   Constant reference to identical iterator.  
        template<boolean_t IS_CONST_ITERATOR2>
        explicit CLinMemIt( const CBaseIt<T,IS_CONST_ITERATOR2,CONTAINER_TYPE>& rhs )
            : super(rhs)
        {}

        /// Empty constructor.
        /// @param[in]       rhs   Reference to identical iterator.    
        CLinMemIt( const self_non_const& rhs )
            : super( rhs.m_p, rhs.m_pContainer )
        {}

#else

        /// Empty constructor. 
        CLinMemIt () {}

        /// Explicit empty constructor.
        /// @param[in]       _p   Pointer to initial memory position.   
        explicit CLinMemIt (const_noconst_pointer _p) // initial memory position
            : super( _p )
        {}

        /// Explicit empty constructor.
        /// @tparam IS_CONST_ITERATOR2 Flag whether the iterator is constant or not.
        /// @param[in]       rhs   Constant reference to identical iterator.
        template<boolean_t IS_CONST_ITERATOR2>
        explicit CLinMemIt( const CBaseIt<T,IS_CONST_ITERATOR2>& rhs )
            : super(rhs)
        {}

        /// Empty constructor.
        /// @param[in]       rhs   Reference to identical iterator.   
        CLinMemIt( const self_non_const& rhs )
            : super( rhs.m_p )
        {}
#endif

#ifndef NDEBUG
        template<class, boolean_t, class, sint32>
#else
        template<class, boolean_t, sint32>
#endif
        friend class CLinMemIt;

        /// Returns the element at given position.
        /// Asserts that position is inside of the container!
        /// @param[in]   n   Constant reference to input position.  
        /// @return          Reference to the element at given position. 
        reference operator[](const difference_type& n) const
        {
            // assert that container is inside the bounds.
            assert ( (m_pContainer != 0)
                &&     (m_p          != 0)
                &&     (self(m_p + n * STRIDE, m_pContainer) >= m_pContainer->begin())
                &&     (self(m_p + n * STRIDE, m_pContainer) <  m_pContainer->end()  ) );
            return operator->()[n*STRIDE];
        }

        /// Increments the member data pointer by STRIDE.
        /// @return          Reference to this pointer 
        self& operator++()
        {
            m_p += STRIDE;
            return *this;
        }

        /// Increments member data pointer and returns value of this 
        /// before incrementing.
        /// @return          Value of this before incrementing
        self operator++(difference_type)
        {
            self result(*this);
            ++(*this);
            return result;
        }

        /// Decrements the member data pointer by STRIDE.
        /// @return          Reference to this pointer
        self& operator--()
        {
            m_p -= STRIDE;
            return *this;
        }

        /// Decrements member data pointer and returns value of this 
        /// before decrementing.
        /// @return          Value of this before decrementing 
        self operator--(difference_type)
        {
            self result(*this);
            --(*this);
            return result;
        }

        /// Increments member data pointer by increment*STRIDE.
        /// @param[in]       increment   Constant reference to increment value.  
        /// @return          Reference to this pointer
        self& operator+=(const difference_type increment)
        {
            m_p += increment * STRIDE;
            return (*this);
        }

        /// Decrements member data pointer by decrement*STRIDE.
        /// @param[in]       decrement   Constant reference to decrement value.  
        /// @return          Reference to this pointer  
        self& operator-=(const difference_type decrement)
        {
            m_p -= decrement * STRIDE;
            return (*this);
        }

        /// Returns a new iterator whose member data pointer is
        /// incremented by n*STRIDE compared to current iterator.
        /// @param[in]    n  Constant reference to increment value  
        /// @return          A new iterator whose member data pointer is incremented
        /// 		         by n*STRIDE compared to current iterator. 
        self operator+(const difference_type& n) const
        {
            CLinMemIt result(*this);
            result += n;
            return result;
        }

        /// Returns a new iterator whose member data pointer is
        /// decremented by n*STRIDE compared to current iterator.
        /// @param[in]    n  Constant reference to decrement value  
        /// @return          A new iterator whose member data pointer is decremented
        /// 			     by n*STRIDE compared to current iterator. 
        self operator-(const difference_type& n) const
        {
            CLinMemIt result(*this);
            result -= n;
            return result;
        }

        /// Returns difference between two iterators giving the distance in memory.
        /// @tparam IS_CONST_ITERATOR2 Flag whether the iterator is constant or not.
        /// @param[in]       rhs   Constant reference to identical iterator object  
        /// @return                Difference between two iterators giving the distance in memory. 
        template<boolean_t IS_CONST_ITERATOR2>
#ifndef NDEBUG
        difference_type operator-(const CLinMemIt<T, IS_CONST_ITERATOR2, CONTAINER_TYPE, STRIDE>& rhs) const
#else
        difference_type operator-(const CLinMemIt<T, IS_CONST_ITERATOR2, STRIDE>& rhs) const
#endif
        {
            return memory_distance(rhs) / static_cast<difference_type>(STRIDE);
        }

        
        // reason: QAC Bug, since this using declaration is within the class scope.
        
        //         (EXCLUDING CLASS SCOPE OR FUNCTION SCOPE USING-DECLARATIONS) shall not be used in header files.

        // Begin section only needed for TI compiler
        using super::memory_distance;

    protected:
        using super::m_p;

#ifndef NDEBUG
        using super::m_pContainer;
#endif
        // End section only needed for TI compiler
    };

} //End of cml namespace

#endif //CML_STL_ITERATOR_H__

