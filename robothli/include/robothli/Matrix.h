#ifndef MATRIX_H
#define MATRIX_H

#include <array>
#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace Kinematics
{
/**
 * @brief The Matrix class is an implementation of the mathematical concept of a matrix (not the movie).
 *
 * @see https://en.wikipedia.org/wiki/Matrix_(mathematics) for more information.
 * typename T: T must be an arithmetic type, i.e. an integral or floating point type
 * const std::size_t M: rows, the rows/number of rows of the matrix
 * const std::size_t N: columns, the columns/number of columns of the matrix
 */
template<typename T, const std::size_t M /* number of rows */, const std::size_t N /* number of columns */>
class Matrix
{
public:
    /**
     * @name Compile-time assertion checking: see http://en.cppreference.com/w/cpp/language/static_assert
     */
    //@{
    /**
     *
     */
    static_assert(std::is_arithmetic<T>::value,
                  "Value T must be arithmetic, see http://en.cppreference.com/w/cpp/types/is_arithmetic");
    /**
     *
     */
    static_assert(M > 0 && N > 0, "M (rows) and N (columns) must both be greater than 0");
    //@}
    /**
     * @name Constructors and destructor
     */
    //@{
    /**
     * Default ctor. Initialises all element with 0.
     */
    explicit Matrix(T value = 0);
    /**
     * Ctor with a linear list of values
     */
    Matrix(const std::initializer_list<T> &aList);
    /**
     * Ctor with a list of lists of values
     */
    Matrix(const std::initializer_list<std::initializer_list<T>> &aList);
    /**
     * Cpy ctor
     */
    Matrix(const Matrix<T, M, N> &aMatrix);
    /**
     * Dtor
     */
    virtual ~Matrix() = default;

    //@}
    /**
     * @name Dimension access
     */
    //@{
    /**
     *
     */
    inline std::size_t
    getRows() const
    {
        return M;
    }
    /**
     *
     */
    inline std::size_t
    getColumns() const
    {
        return N;
    }
    //@}
    /**
     * @name Element access
     */
    //@{
    /**
     *
     */
    std::array<T, N> &
    at(std::size_t row);
    /**
     *
     */
    const std::array<T, N> &
    at(std::size_t row) const;
    /**
     * @return cell at (row,column)
     */
    T &
    at(std::size_t row, std::size_t column);
    /**
     * @return cell at (row,column)
     */
    const T &
    at(std::size_t row, std::size_t column) const;
    /**
     * @return row at anIndex
     */
    std::array<T, N> &
    operator[](std::size_t anIndex);
    /**
     * @return row at anIndex
     */
    const std::array<T, N> &
    operator[](std::size_t anIndex) const;
    //@}
    /**
     * @name Matrix operators
     */
    //@{
    /**
     * Assignment operator
     */
    Matrix<T, M, N> &
    operator=(const Matrix<T, M, N> &rhs);
    /**
     * Assignment operator
     */
    Matrix<T, M, N> &
    operator=(const std::vector<std::vector<T>> &rhs);
    /**
     * Comparison operator
     */
    bool
    operator==(const Matrix<T, M, N> &rhs) const;
    /**
     * Greater than operator
     */
    template<class T2 = T> bool
    operator>(const T2 &rhs) const;
    /**
     * Smaller than operator
     */
    template<class T2 = T> bool
    operator<(const T2 &rhs) const;
    //@}
    /**
     * @name Scalar arithmetic operations supporting only rhs-scalars
     */
    //@{
    /**
     *
     */
    template<class T2 = T> Matrix<T, M, N> &
    operator*=(const T2 &scalar);
    /**
     *
     */
    template<class T2 = T> Matrix<T, M, N>
    operator*(const T2 &scalar) const;
    /**
     *
     */
    template<class T2 = T> Matrix<T, M, N> &
    operator/=(const T2 &scalar);
    /**
     *
     */
    template<class T2 = T> Matrix<T, M, N>
    operator/(const T2 &scalar) const;
    //@}
    /**
     * @name Matrix arithmetic operations
     */
    //@{
    /**
     *
     */
    Matrix<T, M, N> &
    operator+=(const Matrix<T, M, N> &rhs);
    /**
     *
     */
    Matrix<T, M, N>
    operator+(const Matrix<T, M, N> &rhs) const;
    /**
     *
     */
    Matrix<T, M, N> &
    operator-=(const Matrix<T, M, N> &rhs);
    /**
     *
     */
    Matrix<T, M, N>
    operator-(const Matrix<T, M, N> &rhs) const;
    /**
     * (M, N) * (N, O) -> (M, O)
     */
    template<std::size_t columns> Matrix<T, M, columns>
    operator*(const Matrix<T, N, columns> &rhs) const;
    //@}
    /**
     * @name Matrix functions
     */
    //@{
    /**
     * @see https://en.wikipedia.org/wiki/Transpose
     */
    Matrix<T, N, M>
    transpose() const;
    /**
     * @see https://en.wikipedia.org/wiki/Identity_matrix
     */
    Matrix<T, M, N>
    identity() const;
    /**
     * @see https://en.wikipedia.org/wiki/Gaussian_elimination
     */
    Matrix<T, M, N>
    gauss() const;
    /**
     * @see https://en.wikipedia.org/wiki/Invertible_matrix
     */
    Matrix<T, M, N>
    gaussJordan() const;
    /**
     * @see https://en.wikipedia.org/wiki/Invertible_matrix
     */
    Matrix<T, M, N>
    inverse() const;

    /**
     * @see https://en.wikipedia.org/wiki/Singular_value_decomposition
     *
     *  @return std::tuple< U, S, V> where:
     *		- the matrix U is a matrix[M][N]
     *		- the matrix S (the diagonal matrix of singular values) as a matrix[N][N]
     *		- the matrix V (NOT the transpose of V!!!) is a matrix[N][N]
     *
     *
     *	Adapted from http://svn.lirec.eu/libs/magicsquares/src/SVD.cpp, see implementation
     */
    std::tuple<Matrix<T, M, N>, Matrix<T, N, N>, Matrix<T, N, N>>
    svd() const;

    /**
     * @see https://en.wikipedia.org/wiki/Moore–Penrose_inverse
     */
    Matrix<T, M, N>
    moorePenroseInverse() const;
    //@}
    /**
     * @name Other methods
     */
    Matrix<T, M, 1>
    solve() const;
    /**
     *
     */
    bool
    equals(const Matrix<T, M, N> &rhs, double factor) const;

    /**
     * @brief Calculates the cosine simularity between 2 matrices
     *
     * @param rhs a matrix to be compared to
     * @return T the amount of simularity, the closer to 1 this number is the more simular the matrices are.
     */

    T
    cosineSim(const Matrix<T, M, N> &rhs);
    //@{
    /**
     * @return a string representation of the matrix for printing on the screen
     */
    std::string
    to_string() const;
    /**
     * @return a Matrix representation of the string used as argument
     */
    Matrix<T, M, N>
    from_string(std::istream &stream) const;
    //@}
private:
    /**
     * @return the appropriate row number that can be used for the current pivotColumn. This is the row with the
     * maximum absolute value in the pivotColumn
     */
    std::size_t
    getPivotRow(const Matrix<T, M, N> &aMatrix, std::size_t pivotColumn) const;
    /**
     *
     */
    T
    getLineariseFactor(const Matrix<T, M, N> &aMatrix, std::size_t aPivotColumn, std::size_t aCurrentRow) const;

    std::array<std::array<T, N>, M> matrix;
};

/**
 *
 */
template<typename T, std::size_t M, std::size_t N>
inline bool
operator==(const Matrix<T, M, N> &lhs, const Matrix<T, M, N> &rhs)
{
    return lhs.matrix==rhs.matrix;
}
/**
 *
 */
template<typename T, std::size_t M, std::size_t N>
inline std::ostream &
operator<<(std::ostream &stream, const Matrix<T, M, N> &aMatrix)
{
    return stream << aMatrix.to_string();
}
/**
 *
 */
template<typename T, std::size_t M, std::size_t N>
inline std::istream &
operator>>(std::istream &stream, Matrix<T, M, N> &aMatrix)
{
    aMatrix = aMatrix.from_string(stream);
    return stream;
}
}
#include "Matrix.ipp"

#endif /* MATRIX_H */