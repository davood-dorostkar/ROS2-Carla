#ifndef CML_STL_MEMORY_h__
#define CML_STL_MEMORY_h__

#include "cml_stl_type_traits.h"
#include "TM_Global_Types.h"
#include <cassert>

namespace cml
{
    /// Makes a c++-language pointer smarter by checking the validity of the memory
    /// pointed to in debug mode. No performance drawback in release mode, though.
    /// @tparam T       The type of the value
    template < typename T >
    class checked_ptr
    {
    private:
        /// Own type with const data type.
        /// Will be used to compare us to our const version.
        /// @see operator==()
        /// @see operator!=()
        typedef checked_ptr<typename add_const<T>::type> const_self;

        /// Own type with non const data type.
        /// Will be used to compare us to our non-const version and to copy-construct us from our non-const version.
        /// @see operator==()
        /// @see operator!=()
        /// @see checked_ptr(const non_const_self&)
        typedef checked_ptr<typename remove_const<T>::type> non_const_self;

    public:
        typedef T           value_type;
        typedef value_type* pointer;
        typedef value_type& reference;

        /// Empty constructor		
        checked_ptr ()
#ifndef NDEBUG
            : m_p(NULL)
#endif
        {}
                
        /// Explicit one element constructor.
        /// @param[in] _p    Pointer to init value 
        explicit checked_ptr (pointer _p)
            : m_p( _p )
        {}
                
        /// Copy constructor for non-const to const checked_ptr.
        /// @param[in] rhs : Reference to identical value
        checked_ptr( const non_const_self& rhs )
            : m_p( rhs.m_p )
        {}

        /// const and non-const checked_ptr must be friends
        template<typename>
        friend class checked_ptr;
                      
        /// Copies the input pointer to the pointer of this class
        /// @param[in] p  Constant pointer to the input value
        /// @return       None
        void operator= ( const pointer& p )
        {
            m_p = p;
        }
                
        /// Dereferencing of the checked pointer objects.
        /// @return             Pointer to the checked pointer objects.
        pointer operator->() const
        {
            assert ( m_p != NULL );
            return m_p;
        }

        /// Indirection of the checked pointer objects.
        /// @return             Reference to the checked pointer objects. 
        reference operator*() const
        {
            return *(operator->());
        }
        
        /// Returns comparison result of two checked pointer objects.
        /// @param[in]    rhs  Constant reference to identical pointer object.		
        /// @return            True, if both object references are same, else False.	
        boolean_t operator== (const const_self& rhs) const
        {
            return m_p == rhs.m_p;
        }

        /// Returns comparison result of two checked pointer objects.
        /// @param[in]    rhs  Constant reference to identical pointer object.		
        /// @return            True, if both object references are same, else False.	
        boolean_t operator== (const non_const_self& rhs) const
        {
            return m_p == rhs.m_p;
        }

        /// Returns comparison result of two checked pointer objects.
        /// @param[in]    rhs  Constant reference to identical pointer object.		
        /// @return            True, if both object references are not same, else False.	 
        boolean_t operator!= (const const_self& rhs) const
        {
            return m_p != rhs.m_p;
        }

        /// Returns comparison result of two checked pointer objects.
        /// @param[in]    rhs  Constant reference to identical pointer object.		
        /// @return            True, if both object references are not same, else False.	
        boolean_t operator!= (const non_const_self& rhs) const
        {
            return m_p != rhs.m_p;
        }


    protected:
        pointer m_p;
    }; //End of cml namespace
}

#endif // CML_STL_MEMORY_h__
