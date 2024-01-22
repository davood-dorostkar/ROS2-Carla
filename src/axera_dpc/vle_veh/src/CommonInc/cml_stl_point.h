/// Implementation of special column vector variants of cml::CMatrix that are called points since the mathematical
/// expression of a vector is already used by the stack-like container cml:CVector.

#ifndef cml_stl_point_h__
#define cml_stl_point_h__

#include "cml_stl_matrix.h"
#include "cml_stl_cmath.h"
#include "cml_stl_algorithm.h"

namespace cml
{
    /// Represents a two dimensional column vector that adds the scalar product,
    /// cross product and norm to the standard CMatrix operations. Also introduces
    /// named getters x() and y() for the two coordinates.
    /// @tparam T Content type of the coordinates.
    template <typename T>
    class CartesianPoint2D : public cml::CMatrix<T, 2, 1, false>
    {
        typedef CartesianPoint2D<T>           self;  ///< Our type. Reduces amount of code to write.
        typedef cml::CMatrix<T, 2, 1, false>  super; ///< Parent type. Reduces amount of code to write.

        /// Assigns coordinate names to indexes.
        enum CoordinateNames
        {
            X = 0, ///< Position of the X coordinate in the point
            Y      ///< Position of the Y coordinate in the point
        };

    public:
        /// Default constructor
        CartesianPoint2D()
            : super()
        {}

        /// Constructor with member initialization.
        /// @param[in] _x Reference to value to initialize X coordinate of the point.
        /// @param[in] _y Reference to value to initialize Y coordinate of the point.
        CartesianPoint2D(const T& _x, const T& _y)
            : super()
        {
            x() = _x;
            y() = _y;
        }

        /// Constructor using CMatrix to enable usage of methods on results of operations defined in CMatrix
        /// @param[in] mat   Constant reference to vector
        CartesianPoint2D(const super& mat)
            : super(mat)
        {}

        /// The x getter.
		/// Get the X coordinate of the point.
		/// @return   Reference to X co-ordinate.
        T& x(){return super::operator[](X);}

        /// The y getter
		/// Get the Y coordinate of the point.
		/// @return   Reference to Y co-ordinate.
        T& y(){return super::operator[](Y);}

        /// The const x getter
		/// Get the constant reference to X coordinate of the point.
		/// @return   Constant reference to X co-ordinate.
        const T& x() const {return super::operator[](X);}

        /// The const y getter
		/// Get the constant reference to Y coordinate of the point.
		/// @return   Constant reference to Y co-ordinate.
        const T& y() const {return super::operator[](Y);}

        /// Calculate the length of the vector
        /// @return Length of the vector
        float32 norm() const
        {
            return cml::sqrt( normSq() );
        }

        /// Calculate the squared length of the vector
        /// @return Squared length of the vector
        float32 normSq() const
        {
            return cml::pow<2>(x()) + cml::pow<2>(y());
        }

        /// Calculate the scalar product of self and input vector 
		/// (self * rhs)
        /// @param[in] rhs Right hand side cartesian point
        /// @return        Resulting scalar product
        T scalarProduct(const self& rhs) const
        {
            return super::trnspMul(rhs).at(0, 0);
        }

        /// Calculate the cross product of self and input vector 
		/// (self x rhs)
        /// @param[in] rhs Right hand side vector
        /// @return        Scalar value representing the z co-ordinate of the resulting vector
        T crossProduct(const self& rhs) const
        {
            return ((x() * rhs.y()) - (y() * rhs.x()));
        }
    };

    /// Represents a three dimensional column vector that adds the scalar product,
    /// cross product and norm to the standard CMatrix operations. Also introduces
    /// named getters x(), y() and z() for the three coordinates.
    /// @tparam T Content type of the coordinates.
    template <typename T>
    class CartesianPoint3D : public cml::CMatrix<T, 3, 1, false>
    {
        typedef CartesianPoint3D<T>           self;  ///< Our type. Reduces amount of code to write.
        typedef cml::CMatrix<T, 3, 1, false>  super; ///< Parent type. Reduces amount of code to write.

        /// Assigns coordinate names to indexes.
        enum CoordinateNames
        {
            X = 0, ///< Position of the X coordinate in the point
            Y,     ///< Position of the Y coordinate in the point
            Z      ///< Position of the Z coordinate in the point
        };
    public:
        /// Default constructor
        CartesianPoint3D()
            : super()
        {}

        /// Constructor with member initialization
        /// @param[in] _x Reference to value to initialize X coordinate of the point.
        /// @param[in] _y Reference to value to initialize Y coordinate of the point.
        /// @param[in] _z Reference to value to initialize Z coordinate of the point.		
        CartesianPoint3D(const T& _x, const T& _y, const T& _z)
            : super()
        {
            x() = _x;
            y() = _y;
            z() = _z;
        }

        /// Constructor using CMatrix to enable usage of methods on results of operations defined in CMatrix
        /// @param[in] mat Constant reference to vector
        CartesianPoint3D(const super& mat)
            : super(mat)
        {}

        /// The x getter
		/// Get the X coordinate of the point.
		/// @return   Reference to X co-ordinate.
        T& x(){return super::operator[](X);}

        /// The y getter
		/// Get the Y coordinate of the point.
		/// @return   Reference to Y co-ordinate.
        T& y(){return super::operator[](Y);}

        /// The z getter
		/// Get the Z coordinate of the point.
		/// @return   Reference to Z co-ordinate.
        T& z(){return super::operator[](Z);}

        /// The const x getter
		/// Get the constant reference to X coordinate of the point.
		/// @return   Constant reference to X co-ordinate.
        const T& x() const {return super::operator[](X);}

        /// The const y getter
		/// Get the constant reference to Y coordinate of the point.
		/// @return   Constant reference to Y co-ordinate.
        const T& y() const {return super::operator[](Y);}

        /// The const z getter
		/// Get the constant reference to Z coordinate of the point.
		/// @return   Constant reference to Z co-ordinate.
        const T& z() const {return super::operator[](Z);}

        /// Calculate the length of the vector.
        /// @return Length of vector.
        float32 norm() const
        {
            return cml::sqrt( normSq() );
        }

        /// Calculate the squared length of the vector.
        /// @return Squared length of the vector.
        float32 normSq() const
        {
            return cml::pow<2>(x()) + cml::pow<2>(y()) + cml::pow<2>(z());
        }

        /// Calculate the scalar product of self and input vector. 
		/// (self * rhs)
        /// @param[in] rhs Right hand side carthesian point.
        /// @return        Resulting scalar product.
        T scalarProduct(const self& rhs) const
        {
            return super::trnspMul(rhs).at(0, 0);
        }

        /// Calculate the cross product of self and input vector. 
		/// (self x rhs)
        /// @param[in] rhs Right hand side vector.
        /// @return        Resulting cross product vector.
        self crossProduct(const self& rhs) const
        {
            self result;
            result.x() = (y() * rhs.z()) - (z() * rhs.y());
            result.y() = (z() * rhs.x()) - (x() * rhs.z());
            result.z() = (x() * rhs.y()) - (y() * rhs.x());
            return result;
        }
    };

}

#endif // cml_stl_point_h__
