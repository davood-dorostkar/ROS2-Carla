#ifndef cml_stl_type_traits_h__
#define cml_stl_type_traits_h__

#include "TM_Global_Types.h"

namespace cml
{
  typedef bool boolean_t;

  /// Obtains the signed type corresponding to T, keeping any _cv-qualifiers_.
  /// The transformed type is aliased as member type `make_signed::type` as follows:
  /// - If T is an unsigned integral type, this is the corresponding signed type.
  /// - If T is already a signed integral type, this is T unchanged.
  /// Notice that this class merely obtains a type using another type as model, but it does not transform values or objects between
  /// those types. To explicitly cast an integral type to its signed equivalent, you can use `static_cast`.
  
  /// @tparam T An integer type (except `bool`), or an _enumeration type_.
  /// @tparam S Size of signed type specified by type. Defaults to `sizeof(T)`. Should not be specified explicitly
  template< typename T, sint32 S = sizeof(T) > struct make_signed{};

  /// Specialization for 1 byte sized types
  /// @see make_signed
  template<typename T> struct make_signed<T,1> { typedef sint8  type; };
  /// Specialization for 2 byte sized types
  /// @see make_signed
  template<typename T> struct make_signed<T,2> { typedef sint16 type; };
  /// Specialization for 4 byte sized types
  /// @see make_signed
  template<typename T> struct make_signed<T,4> { typedef sint32 type; };

  /// Obtains the unsigned type corresponding to T, keeping any _cv-qualifiers_.
  /// The transformed type is aliased as member type `make_signed::type` as follows:
  /// - If T is an unsigned integral type, this is the corresponding signed type.
  /// - If T is already a signed integral type, this is T unchanged.
  /// Notice that this class merely obtains a type using another type as model, but it does not transform values or objects between
  /// those types. To explicitly cast an integral type to its unsigned equivalent, you can use `static_cast`.
  
  /// @tparam T An integer type (except `bool`), or an _enumeration type_.
  /// @tparam S Size of signed type specified by type. Defaults to `sizeof(T)`. Should not be specified explicitly
  template< typename T, sint32 S = sizeof(T) > struct make_unsigned{};

  /// Specialization for 1 byte sized types
  /// @see make_unsigned
  template<typename T> struct make_unsigned<T,1> { typedef uint8  type; };
  /// Specialization for 2 byte sized types
  /// @see make_unsigned
  template<typename T> struct make_unsigned<T,2> { typedef uint16 type; };
  /// Specialization for 4 byte sized types
  /// @see make_unsigned
  template<typename T> struct make_unsigned<T,4> { typedef uint32 type; };
  /// Specialization for 8 byte sized types
  /// @see make_unsigned
  template<typename T> struct make_unsigned<T,8> { typedef uint64 type; };

  /// Obtains the type T with const qualification.
  /// The transformed type is aliased as member type `add_const::type`.
  /// If T is not already _const-qualified_, and is neither a reference nor a function (which cannot be const-qualified),
  /// this is the same type as `T const`. Otherwise, it is `T` unchanged.
  /// Notice that this class merely obtains a type using another type as model, but it does not transform values or 
  /// objects between those types. To explicitly add const-qualification to an object, `const_cast` can be used.
  
  /// @tparam T A type.
  template<typename T> struct add_const          { typedef const T type; };
  /// Specialization for const types
  /// @see add_const
  template<typename T> struct add_const<const T> { typedef const T type; };

  /// Obtains the type `T` without top-level const qualification.
  /// The transformed type is aliased as member type `remove_const::type`.
  /// If `T` is _const-qualified_, this is the same type as `T` but with its _const-qualification_ removed. Otherwise,
  /// it is `T` unchanged.
  /// Notice that this class merely obtains a type using another type as model, but it does not transform values or objects
  /// between those types. To explicitly remove the const-qualifier of an object, `const_cast` can be used.
  
  /// @tparam T A type.
  template<typename T> struct remove_const          { typedef T type; };
  /// Specialization for const types
  /// @see remove_const
  template<typename T> struct remove_const<const T> { typedef T type; };

  /// Obtains either `T1` or `T2`, depending on whether `COND` is `true` or `false`.
  /// If `COND` is `true`, member type `conditional::type` is defined as an alias of `T1`.
  /// If `COND` is `false`, member type `conditional::type` is defined as an alias of `T2`.
  
  /// @tparam COND A condition.
  /// @tparam T1   A type.
  /// @tparam T2   Another type.
  template<boolean_t COND, typename T1, typename T2> struct conditional                { typedef T1 type; };
  /// Specialization for `COND` equals `false`.
  /// @see conditional
  template<typename T1, typename T2>                 struct conditional<false, T1, T2> { typedef T2 type; };
}

#endif // cml_stl_type_traits_h__
