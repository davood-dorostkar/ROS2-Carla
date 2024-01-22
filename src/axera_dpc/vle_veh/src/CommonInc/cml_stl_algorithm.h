    

#ifndef cml_stl_algorithm_h__
#define cml_stl_algorithm_h__

#include <algorithm>

#include "cml_stl_vector.h"

namespace cml
{

    /// Partially sort the Array of elements, of any type in the
    /// range based on the Pivot element(nth).
    /// Rearrange the elements in the range [first,last), in such 
    /// a way that the element at the nth position is the element 
    /// that would be in that position in a sorted sequence. 
    /// The other elements are left without any specific order, 
    /// except that none of the elements preceding nth are greater 
    /// than it, and none of the elements following it are less.
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @param[in,out]   first   Iterator to the start address of Array of elements 
    /// @param[in]       nth     Iterator to nth element to be identified in the Array  
    /// @param[in]       last    Iterator to the end of the Array(points to the element 
    ///                          after the last element of the Array) 
    /// @return          None 
    template <class RandomAccessIterator>
    inline static void nth_element (RandomAccessIterator first, RandomAccessIterator nth, RandomAccessIterator last)
    {
        --last;
        for (;;)
        {
            if (last <= first)
            {
                /* One element only */
                return;
            }

            if (last == first + 1)
            {
                /* Two elements only */
                if (*first > *last)
                {
                    std::swap(*first, *last);
                }
                return;
            }

            /* Find median of low, middle and high items; swap into position low */
            RandomAccessIterator middle = first + (last - first) / 2;
            if (*middle > *last)    std::swap(*middle, *last);
            if (*first  > *last)    std::swap(*first,  *last);
            if (*middle > *first)   std::swap(*middle, *first);

            /* Swap low item (now in position middle) into position (low+1) */
            std::swap(*middle, *(first+1));

            /* Nibble from each end towards middle, swapping items when stuck */
            RandomAccessIterator ll = first + 1;
            RandomAccessIterator hh = last;
            for (;;)
            {
                do ll++; while (*first > *ll);
                do hh--; while (*hh    > *first);

                if (hh < ll)
                {
                    break;
                }

                std::swap(*ll, *hh);
            }

            /* Swap middle item (in position low) back into correct position */
            std::swap(*first, *hh);

            /* Re-set active partition */
            if (hh <= nth)
            {
                first = ll;
            }
            if (hh >= nth)
            {
                last = hh - 1;
            }
        }
    }
     
    /// Find out the middle element in an array of elements of any type.
    /// Returns element in the middle of first and last as if the
    /// array of elements is sorted in ascending order.
    /// If n is Even -> (n/2)th element
    /// If n is Odd  -> (n/2 + 1)th element
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @param[in]       first   Iterator to the start address of Array of elements 
    /// @param[in]       last    Iterator to the end of the Array(points to the element 
    ///							 after the last element of the Array)   
    /// @return          Element in the middle of first and last
    template <class RandomAccessIterator>
    inline static typename RandomAccessIterator::value_type median (RandomAccessIterator first, RandomAccessIterator last)
    {
        assert(last - first > 0);

        RandomAccessIterator nth = first + (last - first - 1) / 2;
        nth_element(first, nth, last);

        return *nth;
    }
               
    /// Partition sort the sub array elements in the range in ascending order
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @param[out]      _pong       Partition sorted sub array of elements 
    /// @param[in]       _ping       Input subarray to be partition sorted
    /// @param[in]       left        Left Index of the current partition
    /// @param[in]       right       Right Index of the current partition
    /// @param[in]       pivotIndex  Index of the current pivot element
    /// @return          New pivot index after partitioning
    template <class RandomAccessIterator>
    static sint32 _sort_TiOpt_partition (RandomAccessIterator _ping, RandomAccessIterator _pong, sint32 left, sint32 right, sint32 pivotIndex)
    {
#if defined(CML_USE_INTRINSICS) && defined(NDEBUG)
        typename RandomAccessIterator::value_type* restrict ping = &_ping[0];
        typename RandomAccessIterator::value_type* restrict pong = &_pong[0];
#else
        RandomAccessIterator ping = _ping;
        RandomAccessIterator pong = _pong;
#endif

        sint32 storeIndex = left;
        sint32 rightIndex = right;
        const typename RandomAccessIterator::value_type pivotValue = ping[pivotIndex];

        ping[pivotIndex] = ping[right];

        // This loop should be perfectly optimized by TI software pipelining. 
        // Check that the restrict keyword is properly set on ping and pong buffers. */
        for (sint32 i=left; i<right; i++)
        {
            if (ping[i] < pivotValue)
            {
                pong[storeIndex++] = ping[i];
            }
            else
            {
                pong[rightIndex--] = ping[i];
            }
        }

        // Copy pivotValue back into the ping and pong buffer (must be in both because it is excluded in next recursion)
        pong[storeIndex] = pivotValue;
        ping[storeIndex] = pivotValue;

        return storeIndex;
    }
        
    /// Partition sort the sub array elements in the range in  
    /// ascending/descending order based on the given comparison 
    /// function object.
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @tparam          Compare               Compare function object
    /// @param[out]      _pong        Partition sorted sub array of elements 
    /// @param[in]       _ping        Input subarray to be partition sorted
    /// @param[in]       left         Left Index of the current partition
    /// @param[in]       right        Right Index of the current partition
    /// @param[in]       pivotIndex   Index of the current pivot element 
    /// @param[in]       comp         Compare function object to dynamically change the sorting direction
    /// @return          New pivot index after partitioning
    template <class RandomAccessIterator, class Compare>
    static sint32 _sort_TiOpt_partition (RandomAccessIterator _ping, RandomAccessIterator _pong, sint32 left, sint32 right, sint32 pivotIndex, Compare comp)
    {
#if defined(CML_USE_INTRINSICS) && defined(NDEBUG)
        typename RandomAccessIterator::value_type* restrict ping = &_ping[0];
        typename RandomAccessIterator::value_type* restrict pong = &_pong[0];
#else
        RandomAccessIterator ping = _ping;
        RandomAccessIterator pong = _pong;
#endif

        sint32 storeIndex = left;
        sint32 rightIndex = right;
        const typename RandomAccessIterator::value_type pivotValue = ping[pivotIndex];

        ping[pivotIndex] = ping[right];

        // This loop should be perfectly optimized by TI software pipelining. 
        // Check that the restrict keyword is properly set on ping and pong buffers. */
        for (sint32 i=left; i<right; i++)
        {
            if (comp(ping[i], pivotValue))
            {
                pong[storeIndex++] = ping[i];
            }
            else
            {
                pong[rightIndex--] = ping[i];
            }
        }

        // Copy pivotValue back into the ping and pong buffer (must be in both because it is excluded in next recursion)
        pong[storeIndex] = pivotValue;
        ping[storeIndex] = pivotValue;

        return storeIndex;
    }

    /// Iteration sort the sub array elements in the range in  
    /// ascending order using recursive partition sort 
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @param[in,out]   ping    Array of elements
    /// @param[in]       pong    Scratch memory used for storing sorted elements 
    /// @param[in]       left    Left Index of the current partition
    /// @param[in]       right   Right Index of the current partition
    /// @param[in]       depth   Recursion depth
    /// @param[in]       idx     Index of the current pivot element 
    /// @return          None
    template <class RandomAccessIterator>
    static void _sort_TiOpt_iteration (RandomAccessIterator ping, RandomAccessIterator pong, sint32 left, sint32 right, sint32 depth, sint32 idx)
    {
        if (right <= left)
        {
            // only one remaining element
            if (depth & 1)
            {
                // ping is the scratch memory now. Copy value back into the pong buffer
                pong[idx] = ping[idx];
            }
        }
        else
        {
            // Quick Sort
            const sint32 pivotIndex = (left + right) / 2;
            const sint32 pivotNewIndex = _sort_TiOpt_partition(ping, pong, left, right, pivotIndex);
            _sort_TiOpt_iteration(pong, ping, left, pivotNewIndex - 1, depth+1, left);
            _sort_TiOpt_iteration(pong, ping, pivotNewIndex + 1, right, depth+1, right);
        }
    }

    /// Iteration sort the sub array elements in the range in  
    /// ascending/descending order using recursive partition sort   
    /// based on the given comparison function object comp. 
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @tparam          Compare               Compare function object
    /// @param[in,out]   ping    Array of elements
    /// @param[in]       pong    Scratch memory used for storing sorted elements 
    /// @param[in]       left    Left Index of the current partition
    /// @param[in]       right   Right Index of the current partition
    /// @param[in]       depth   Recursion depth
    /// @param[in]       idx     Index of the current pivot element 
    /// @param[in]       comp    Compare function object to dynamically change the sorting direction
    /// @return          None 
    template <class RandomAccessIterator, class Compare>
    static void _sort_TiOpt_iteration (RandomAccessIterator ping, RandomAccessIterator pong, sint32 left, sint32 right, sint32 depth, sint32 idx, Compare comp)
    {
        if (right <= left)
        {
            // only one remaining element
            if (depth & 1)
            {
                // ping is the scratch memory now. Copy value back into the pong buffer
                pong[idx] = ping[idx];
            }
        }
        else
        {
            // Quick Sort
            const sint32 pivotIndex = (left + right) / 2;
            const sint32 pivotNewIndex = _sort_TiOpt_partition(ping, pong, left, right, pivotIndex, comp);
            _sort_TiOpt_iteration(pong, ping, left, pivotNewIndex - 1, depth+1, left, comp);
            _sort_TiOpt_iteration(pong, ping, pivotNewIndex + 1, right, depth+1, right, comp);
        }
    }

    /// Iteration sort the array elements in the range in ascending 
    /// order using recursive partition sort.
    /// Iterative sort the sub array elements in the range.
    /// This quick sort implementation is using scratch memory to 
    /// enable a software pipelining optimization of the inner loop 
    /// for TI c6x platform!
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @param[in,out]   first       Iterator to the start address of Array of elements 
    /// @param[in]       last        Iterator to the end of the Array(points to the element 
    ///						         after the last element of the Array!) 
    /// @param[in]       scratchMem  Iterator to the Scratch memory space required by partition sort  
    /// @return          None 
    template <class RandomAccessIterator>
    static void sort_TiOpt (RandomAccessIterator first, RandomAccessIterator last, RandomAccessIterator scratchMem)
    {
        _sort_TiOpt_iteration(first, scratchMem, 0, last-first-1, 0, 0);
    }
    
    /// Iterative sort the array elements of any type in the range.
    /// Iteration sort the array elements in the range in  
    /// ascending/descending order using recursive partition sort based on 
    /// the given comparison function object comp.
    /// This quick sort implementation is using scratch memory to 
    /// enable a software pipelining optimization of the inner loop 
    /// for TI c6x platform!
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @tparam          Compare               Compare function object
    /// @param[in,out]   first       Iterator to the start address of Array of elements 
    /// @param[in]       last        Iterator to the end of the Array(points to the element 
    ///						         after the last element of the Array!) 
    /// @param[in]       scratchMem  Iterator to the Scratch memory space required by partition sort  
    /// @param[in]       comp        Compare function object to dynamically change the sorting direction
    /// @return          None	
    template <class RandomAccessIterator, class Compare>
    static void sort_TiOpt (RandomAccessIterator first, RandomAccessIterator last, RandomAccessIterator scratchMem, Compare comp)
    {
        _sort_TiOpt_iteration(first, scratchMem, 0, last-first-1, 0, 0, comp);
    }
  
    /// Iterative Quick Sort based on partitioning and Insertion sort.
    /// Iterative (non recursive) Quick Sort implementation.
    /// Partition Sort for Larger arrays
    /// Insertion Sort for array size less than 7
    /// 1. Requires an internal stack of 64 Size!
    /// 2. Unfortunately not much faster compared to the original 
    ///    recursive TI implementation.
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @param[in,out]   first   Iterator to the start address of Array of elements  
    /// @param[in]       last    Iterator to the end of the Array(points to the element 
    ///  				         after the last element of the Array!) 
    /// @return          None
    template <class RandomAccessIterator>
    inline static void sort (RandomAccessIterator first, RandomAccessIterator last)
    {
        sint32 i;
        sint32 j;
        sint32 right = (last - first) - 1;
        sint32 left = 0;
        typename RandomAccessIterator::value_type temp;
        cml::CVector<sint32, 64> stack;   // stack contains 32 left/right values: sufficient for sorting 2^32 elements

        for (;;)
        {
            //Insertion sort when subarray small enough.
            if ( (right-left) < 7)
            {
                for (j = left+1; j <= right; j++)
                {
                    temp = first[j];
                    for (i = j-1; i >= left; i--)
                    {
                        if (first[i] < temp)
                        {
                            break;
                        }
                        first[i+1] = first[i];
                    }
                    first[i+1] = temp;
                }

                if (stack.empty()) 
                {
                    break;
                }

                //Pop stack and begin a new round of partitioning.
                right = stack.back(); stack.pop_back();
                left  = stack.back(); stack.pop_back();
            }
            else
            {
                //Choose median of left, center, and right elements as partitioning element a. Also rearrange so that a[l] <= a[l+1] <= a[ir].
                std::swap(first[(left + right) >> 1], first[left+1]);
                if (first[right] < first[left])
                {
                    std::swap(first[left], first[right]);
                }
                if (first[right] < first[left+1])
                {
                    std::swap(first[left+1], first[right]);
                }
                if (first[left+1] < first[left])
                {
                    std::swap(first[left], first[left+1]);
                }

                //Initialize pointers for partitioning.
                i = left+1;
                j = right;
                temp = first[left+1];

                //Partitioning element.
                for (;;)
                {
                    //Scan up to find element > a
                    do
                    {
                        i++;
                    }while (first[i] < temp);
                    //Scan down to find element < a
                    do
                    {
                        j--;
                    }while (temp < first[j]);
                    //Pointers crossed. Partitioning complete
                    if (j < i)
                    {
                        break;
                    }
                    //Exchange elements.
                    std::swap(first[i], first[j]);
                }

                //Insert partitioning element.
                first[left+1] = first[j];
                first[j] = temp;

                //Push pointers to larger subarray on stack, process smaller subarray immediately.
                if ( (right-i+1) >= (j-left) )
                {
                    stack.push_back(i);
                    stack.push_back(right);
                    right = j-1;
                }
                else
                {
                    stack.push_back(left);
                    stack.push_back(j-1);
                    left = i;
                }
            }
        }
    }

    /// Find a value in range
    /// Returns an iterator to the first element in the range [first,last) 
    /// that compares equal to input value.
    /// If no such element is found, the function returns iterartor 
    /// to last(points to the element after the last element of the Array).
    /// @tparam          InputIterator  Iterator to the element array
    /// @tparam          T              Type of the value
    /// @param[in]       first   Iterator to the start address of Array of elements 
    /// @param[in]       last    Iterator to the end of the Array(points to the element 
    ///						     after the last element of the Array!) 
    /// @param[in]       val     Reference to Value to search for in the range
    /// @return          An iterator to the element in the range that compares 
    ///				     equal to value. If no elements match, return last
    template<class InputIterator, class T>
    static InputIterator find (InputIterator first, InputIterator last, const T& val)
    {
        while (first != last)
        {
            if (*first == val)
            {
                return first;
            }
            ++first;
        }
        return last;
    }
  
    /// Return iterator to lower bound.
    /// Returns an iterator pointing to the first element in the 
    /// range [first, last) that is not less than the input value.
    /// The elements in the range should be already sorted according 
    /// to this same criterion (operator <), or at least partitioned
    /// with respect to input value.
    /// @tparam          RandomAccessIterator  Random access iterator to the element array
    /// @tparam          T                     Type of the value
    /// @param[in]       first   Iterator to the start address of Array of elements 
    /// @param[in]       last    Iterator to the end of the Array(points to the element 
    ///						     after the last element of the Array!).
    /// @param[in]       val     Value of the lower bound to search for in the range.
    /// @return          An iterator to the lower bound of input value in the range.
    ///				     If all the element in the range compare less than input value, 
    ///				     the function returns last(points to the element after the last element of the Array!).
    template <class RandomAccessIterator, class T>
    static RandomAccessIterator lower_bound (RandomAccessIterator first, RandomAccessIterator last, const T& val)
    {
        RandomAccessIterator it;
        typename RandomAccessIterator::difference_type count;
        typename RandomAccessIterator::difference_type step;
        count = last - first;
        while (count > 0)
        {
            it = first;
            step = count / 2;
            it += step;
            if (*it < val)
            {
                first = ++it;
                count -= step + 1;
            }
            else
            {
                count = step;
            }
        }
        return first;
    }

    /// Implements the power of input value using a recursive template.
    /// Struct needed, because partial specialization of function is 
    ///	not allowed.
    /// @tparam          POWER  Power of the input value
    /// @tparam          T      Type of the input value
    /// @param[in]       val    Value whose power to be evaluated
    /// @return          Power of the input value based on the template parameter POWER
    template <uint32 POWER, typename T>
    struct SPow
    {
        static inline T pow(const T& val)
        {
            return val * SPow<POWER-1, T>::pow(val);
        }
    };

    /// Implements the first exponent of a Real value.
    /// Implements the first exponent of a Real value which evaluates to 'val' always.
    /// Struct needed, because partial specialization of function is 
    /// not allowed.
    /// @tparam          POWER  Power of the input value
    /// @param[in]       val    Value whose first exponent to be evaluated
    /// @return          Returns value itself, since it's a first exponent
    template <typename T>
    struct SPow<1U, T>
    {
        static inline T pow(const T& val)
        {
            return val;
        }
    };

    /// Implements the zero exponent of a Real value.
    /// Implements the zero exponent of a Real value which evaluates to '1' always.
    /// Struct needed, because partial specialization of function is 
    /// not allowed.
    /// @tparam          POWER  Power of the input value
    /// @param[in]       val    Value whose zero exponent to be evaluated
    /// @return          Returns 1 always, since it's a zero exponent
    template <typename T>
    struct SPow<0U, T>
    {
        static inline T pow(const T& val)
        {
            return static_cast<T>(1);
        }
    };
     
    /// Power function to call static method from struct.
    /// @tparam          POWER  Power of the input value
    /// @tparam          T      Type of the input value
    /// @param[in]       val    Value whose power to be evaluated
    /// @return          Power of the val based on the template parameter POWER
    template <uint32 POWER, typename T>
    inline static T pow(const T& val)
    {
        return SPow<POWER, T>::pow(val);
    }

    /// Limit to range using template method (equals MIN MAX).
    /// Limits the value to the MIN MAX range specified in the
    /// function arguments
    /// @tparam          T      Type of the input value
    /// @param[in]       min_   Reference to the MIN value 
    /// @param[in]       max_   Reference to the MAX value 
    /// @param[in]       val    Reference to the value to be clamped 
    /// @return          Value after clamping
    template<typename T>
    inline static T clamp(const T& min_, const T& max_, const T& val)
    {
        return (val < min_) ? min_ : ((val > max_) ? max_ : val);
    }

    /// Returns the maximum of two arguments if not equal,
    /// otherwise the rhs value
    /// @tparam          T      Type of the input value
    /// @param[in]       lhs    Reference to the left hand side value
    /// @param[in]       rhs    Reference to the right hand side value
    /// @return          Maximum value if not equal, otherwise the rhs value
    template <typename T>
    inline static T max(const T& lhs, const T& rhs)
    {
        return ((lhs>rhs) ? lhs : rhs );
    }

    /// Returns the minimum of two arguments if not equal,
    /// otherwise the rhs value
    /// @tparam          T      Type of the input value
    /// @param[in]       lhs    Reference to the left hand side value
    /// @param[in]       rhs    Reference to the right hand side value
    /// @return          Minimum value if not equal, otherwise the rhs value
    template <typename T>
    inline static T min(const T& lhs, const T& rhs)
    {
        return ((lhs<rhs) ? lhs : rhs );
    }
  
    /// Returns the sign of the given value of any Type.
    /// Returns -1 for negative, +1 for positive and 0 if value is zero.
    /// @tparam          T      Type of the input value
    /// @param[in]       val    The value the sign is extracted from.
    /// @return          The sign of the given value.
    template <typename T>
    inline static sint32 sgn(const T& val)
    {
        return static_cast<sint32>((static_cast<T>(0) < val)) - static_cast<sint32>((static_cast<T>(0) > val));
    }

} // End of cml namespace

#endif // cml_stl_algorithm_h__

