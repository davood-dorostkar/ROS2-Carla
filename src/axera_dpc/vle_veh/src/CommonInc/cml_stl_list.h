    

#ifndef CML_STL_LIST_H__
#define CML_STL_LIST_H__

#include "cml_stl_vector.h"
#include "cml_stl_array.h"
#include "cml_stl_iterator.h"

namespace cml
{

    /// ListNode is the base element of the double linked list
    /// @tparam T                 The type of the elements
    /// @tparam CONTAINER_SIZE    The size of the container
    
#ifndef NDEBUG
    template <class T, sint32 CONTAINER_SIZE>
#else
    template <class T>
#endif
    struct ListNode
    {
#ifndef NDEBUG
        typedef ListNode< T, CONTAINER_SIZE >                          self;
        typedef CBaseIt < self, false, CArray< self, CONTAINER_SIZE> > iterator;
#else
        typedef ListNode< T >           self;
        typedef CBaseIt < self, false > iterator;
#endif

        typedef typename remove_const<T>::type value_type;

        value_type data;

        iterator itPrev;
        iterator itNext;
    };

    // Forward declaration the container class CList
    template <class T, sint32 SIZE>
    class CList;

    /// CBidirectionalIt is the bi-directional iterator to the double linked list
    /// @tparam T                     The type of the elements
    /// @tparam IS_CONST_ITERATOR     Flag whether the iterator is of constant type (true) 
    ///                               or non constant (false).
    ///                               Default is false, meaning non constant.
    /// @tparam CONTAINER_TYPE        The type of the container

#ifndef NDEBUG
    template <class T, boolean_t IS_CONST_ITERATOR, class CONTAINER_TYPE>
#else
    template <class T, boolean_t IS_CONST_ITERATOR>
#endif
    class CBidirectionalIt
    {
    public:

#ifndef NDEBUG
        typedef CBidirectionalIt < T             , IS_CONST_ITERATOR, CONTAINER_TYPE > self;
        typedef CBidirectionalIt < T             , false            , CONTAINER_TYPE > self_non_const;
        typedef ListNode         < T             , CONTAINER_TYPE::MAX_SIZE          > list_node_type;
        typedef CBaseIt          < list_node_type, IS_CONST_ITERATOR, CONTAINER_TYPE > list_node_iterator;
#else
        typedef CBidirectionalIt < T             , IS_CONST_ITERATOR > self;
        typedef CBidirectionalIt < T             , false             > self_non_const;
        typedef ListNode         < T                                 > list_node_type;
        typedef CBaseIt          < list_node_type, IS_CONST_ITERATOR > list_node_iterator;
#endif

        typedef typename conditional<IS_CONST_ITERATOR, typename add_const<T>::type, T>::type value_type;
        typedef value_type* pointer;
        typedef value_type& reference;

        typedef int difference_type;

        typedef std::bidirectional_iterator_tag iterator_category;

        /// The const variant needs access to non-const protected member data
        /// Hence, allow self_const access to us
        /// Theoretically, this should be a specialized template to <T,true>, but TI-compiler does not understand
#ifndef NDEBUG
        template<class T2, boolean_t IS_CONST_ITERATOR_2, class CONTAINER_TYPE_2>
#else
        template<class T2, boolean_t IS_CONST_ITERATOR_2>
#endif
        friend class cml::CBidirectionalIt;

        /// The class CList needs to be able to access the node directly
        template<class, sint32>
        friend class CList;

        /// Empty constructor
        CBidirectionalIt () {}

        /// Constructor with reference to an iterator.
        /// @param[in] _rhs    Constant reference to identical iterator
        CBidirectionalIt ( const self_non_const&     _rhs    ) : m_itNode(_rhs.m_itNode) {}

        /// Constructor with reference to an iterator.
        /// @tparam    IS_CONST_ITERATOR_2   Flag whether the iterator is of constant type (true) 
        ///                                  or non constant (false).
        ///                                  Default is false, meaning non constant.
        /// @param[in] _itNode               Constant reference to base iterator
        template<boolean_t IS_CONST_ITERATOR_2>
#ifndef NDEBUG
        CBidirectionalIt ( const CBaseIt< list_node_type, IS_CONST_ITERATOR_2, CONTAINER_TYPE >& _itNode )
#else
        CBidirectionalIt ( const CBaseIt< list_node_type, IS_CONST_ITERATOR_2                 >& _itNode )
#endif
            : m_itNode(_itNode) {}

        /// Indirection of the element.
        /// @return             Reference to the element.
        reference operator*() const
        {
            return m_itNode->data;
        }

        /// Dereferencing of the element.
        /// @return             Pointer to the element.
        pointer operator->() const 
        {
            return &operator*();
        }

        /// Increments member data pointer to next element.
        /// @return             Returns the reference to this element.
        self& operator++()
        {
            m_itNode = m_itNode->itNext;
            return *this;
        }

        /// Increments member data pointer to next element.
        /// @return             Returns the new element.
        self operator++(difference_type)
        {
            self result(*this);
            m_itNode = m_itNode->itNext;
            return result;
        }

        /// Decrements member data pointer to previous element.
        /// @return             Returns reference to this element.
        self& operator--()
        {
            m_itNode = m_itNode->itPrev;
            return *this;
        }

        /// Decrements member data pointer to previous element.
        /// @return             Returns the new element.
        self operator--(difference_type)
        {
            self result(*this);
            m_itNode = m_itNode->itPrev;
            return result;
        }

        /// Returns comparison result of two list objects.
        /// @param[in]    rhs   Constant reference to identical iterator.		
        /// @return             True, if both list objects are same reference else False.
        boolean_t operator== (const self& rhs) const
        {
            return m_itNode.operator==(rhs.m_itNode);
        }

        /// Returns comparison result of list objects.
        /// @param[in]    rhs   Constant reference to identical iterator.		
        /// @return             True, if both list objects are not same reference else False.
        boolean_t operator!= (const self& rhs) const
        {
            return m_itNode.operator!=(rhs.m_itNode);
        }

    protected:
        list_node_iterator m_itNode;
    };


    /// The container class CList	
    /// @tparam T       The type of the elements
    /// @tparam SIZE    The size of the container
    template <class T, sint32 SIZE>
    class CList
    {
    public:

        // the number of elements the container can store
        enum { MAX_SIZE = SIZE };

        // the number of elements the container has reserved internally
        enum { CAPACITY = SIZE+1 };

        typedef sint32 size_type;

        typedef typename remove_const<T>::type  type;
        typedef typename    add_const<T>::type  const_type;
        typedef typename remove_const<T>::type& reference;
        typedef typename    add_const<T>::type& const_reference;
        typedef typename remove_const<T>::type* pointer;
        typedef typename    add_const<T>::type* const_pointer;

        typedef int difference_type;

        // iterator typedefs
#ifndef NDEBUG
        typedef CBidirectionalIt <type, false, CArray<ListNode<type, CAPACITY>, CAPACITY> >  it;
        typedef CBidirectionalIt <type, true , CArray<ListNode<type, CAPACITY>, CAPACITY> > cIt;
#else
        typedef CBidirectionalIt <type, false>  it;
        typedef CBidirectionalIt <type, true > cIt;
#endif
        typedef it  iterator;
        typedef cIt const_iterator;

        /// Constructor which puts all nodes into the idle list.
        CList ()
        {
            clear();
        }

        /// Constructor with reference to an iterator, copies rhs to this.
        /// @param[in] rhs    Constant reference to identical iterator
        explicit CList( const CList& rhs )
        {
            operator=(rhs);
        }

        /// Operator= makes a copy of the data.
        /// This is a quite expensive operation for this container because
        /// full regeneration of the list is required
        /// @param[in] rhs      Constant reference to identical iterator.		
        /// @return             Reference to this iterator.
        CList& operator=(const CList& rhs)
        {
            clear();
            for (const_iterator iter = rhs.begin(); iter.operator !=(rhs.end()); ++iter)
            {
                push_back(*iter);
            }
            return *this;
        }

        /// Set's all Next and Prev Pointers to beginn address,
        /// and puts them in idle mode.	
        /// @return             void.
        void clear()
        {
            // the first node is reserved as the end of the linked list (in order to avoid a lot of checks whether previous or next node exist)
            m_listNodes[0].itNext = m_listNodes.begin();
            m_listNodes[0].itPrev = m_listNodes.begin();

            // put all nodes into the idle list
            m_idleListNodes.clear();
            for (typename ListNodeArray::iterator iter = m_listNodes.begin()+1; iter.operator!=(m_listNodes.end()); ++iter)
            {
                m_idleListNodes.push_back(iter);
            }
        }

        /// Returns size of ListNode, asserts if size is greater than list node size.
        /// @return             Returns size of ListNode.
        size_type size() const
        {
            assert (SIZE >= m_idleListNodes.size());
            return (SIZE - m_idleListNodes.size());
        }

        /// Checks if list is empty.		
        /// @return             Returns True if list is empty, else False.
        boolean_t empty() const
        {
            return begin().operator==(end());
        }

        /// Checks if ListNodes array is full.
        /// @return             Returns True if list is full, else False.
        boolean_t full() const
        {
            return (m_idleListNodes.empty());
        }

        /// Returns reference to first element of the list.
        /// @return             Reference to first element of the list.
        reference front()
        {
            return *begin();
        }

        /// Returns constant reference to first element of the list.
        /// @return             Constant reference to first element of the list.
        const_reference front() const
        {
            return *begin();
        }

        /// Returns reference to last element of the list.
        /// @return             Reference to last element of the list.
        reference back()
        {
            return *(--end());
        }

        /// Returns constant reference to last element of the list.
        /// @return             Constant reference to last element of the list.
        const_reference back() const
        {
            return *(--end());
        }

        /// Sets reference to input element and inserts element at first position.
        /// @param[in]    x     Constant reference to input element.		
        /// @return             None
        void push_front(const_reference x)
        {
            push_front() = x;
        }

        /// Inserts element at first position of array and returns reference to it.
        /// @return             Reference to element inserted.
        reference push_front()
        {
            return *insert(begin());
        }

        /// Sets reference to input element and inserts element at last position.
        /// @param[in]   x      Constant reference to input element.		
        /// @return             void
        void push_back(const_reference x)
        {
            push_back() = x;
        }

        /// Inserts element at last position and returns reference of last element.
        /// @return             Reference to element inserted.
        reference push_back()
        {
            return *insert(end());
        }

        /// Erases the first element.		
        /// @return             void
        void pop_front()
        {
            erase(begin());
        }

        /// Erases the last element.	
        /// @return             void
        void pop_back()
        {
            erase(--end());
        }

        /// Inserts new element at position with input value.
        /// @param[in]  position  Position of the element
        /// @param[in]  x         Constant reference to the element
        /// @return               Returns new iterator
        iterator insert(iterator position, const_reference x)
        {
            iterator itNewNode = insert(position);
            *itNewNode = x;
            return itNewNode;
        }

        /// Inserts new element at position 
        /// asserts if element is outside the list bounds.
        /// @param[in]  position  Position of the element		
        /// @return               Returns new iterator
        iterator insert(iterator position)
        {
            assert (m_idleListNodes.size() > 0);

            typename ListNodeArray::iterator::super itNewNode = m_idleListNodes.back();
            m_idleListNodes.pop_back();

            itNewNode->itPrev = position.m_itNode->itPrev;
            itNewNode->itNext = position.m_itNode;

            itNewNode->itPrev->itNext = itNewNode;
            itNewNode->itNext->itPrev = itNewNode;

            return iterator(itNewNode);
        }

        /// Erases iterator at position.
        /// @param[in]  position  Position of the element		
        /// @return               Returns new iterator
        iterator erase(iterator position)
        {
            assert(position != end());

            position.m_itNode->itPrev->itNext = position.m_itNode->itNext;
            position.m_itNode->itNext->itPrev = position.m_itNode->itPrev;
            m_idleListNodes.push_back(position.m_itNode);

            return (iterator(position.m_itNode->itNext));
        }

        /// Erases elements from first to last.
        /// @param[in]  first  Iterator to the first element
        /// @param[in]  last   Iterator to the last element
        /// @return            Returns iterator to last
        iterator erase(iterator first, iterator last)
        {
            while (first.operator!=(last))
            {
                first = erase(first);
            }
            return last;
        }

        /// Returns an iterator to the beginning of the list.
        /// @return            Iterator to the beginning of the list 
        iterator begin()
        {
            return iterator(m_listNodes[0].itNext);
        }

        /// Returns a constant iterator to the beginning of the list.
        /// @return            Constant iterator to the beginning of the list
        const_iterator begin() const
        {
            return const_iterator(m_listNodes[0].itNext);
        }

        /// Returns an iterator to the position after the end of the list.
        /// @return            Iterator to the position after the end of the list
        iterator end()
        {
            return iterator(m_listNodes.begin());
        }

        /// Returns a constant iterator to the position after the end of the list.
        /// @return            Constant iterator to the position after the end of the list
        const_iterator end() const
        {
            return const_iterator(m_listNodes.begin());
        }

    protected:

#ifndef NDEBUG
        typedef CArray<ListNode<type, CAPACITY>, CAPACITY> ListNodeArray;
#else
        typedef CArray<ListNode<type>, CAPACITY> ListNodeArray;
#endif
        typedef CVector<typename ListNodeArray::iterator::super, SIZE> IdleListNodeVector;

        ListNodeArray      m_listNodes;
        IdleListNodeVector m_idleListNodes;

    };
}

#endif //CML_STL_LIST_H__
