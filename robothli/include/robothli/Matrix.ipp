#include <cassert>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace Kinematics
{
/**
 * helper function for svd
 */
inline double
sign(double a, double b)
{
    return b >= 0.0 ? std::abs(a) : -std::abs(a);
}

/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(T value)
{
    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t column = 0; column < N; ++column)
        {
            matrix.at(row).at(column) = value;
        }
    }
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const std::initializer_list<T> &aList)
{
    // Check the arguments
    assert(aList.size()==M*N);

    auto row_iter = aList.begin();
    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t column = 0; column < N; ++column, ++row_iter)
        {
            matrix.at(row).at(column) = *row_iter;
        }
    }
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const std::initializer_list<std::initializer_list<T>> &aList)
{
    // Check the arguments, the static assert assures that there is at least 1 M
    // and 1 N!
    assert(aList.size()==M && (*aList.begin()).size()==N);

    auto row_iter = aList.begin();
    for (std::size_t row = 0; row < aList.size(); ++row, ++row_iter)
    {
        auto column_iter = (*row_iter).begin();
        for (std::size_t column = 0; column < (*row_iter).size(); ++column, ++column_iter)
        {
            matrix.at(row).at(column) = *column_iter;
        }
    }
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix<T, M, N>::Matrix(const Matrix <T, M, N> &aMatrix) : matrix(aMatrix.matrix)
{
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> std::array<T, N> &
Matrix<T, M, N>::at(std::size_t row)
{
    return matrix.at(row);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> const std::array<T, N> &
Matrix<T, M, N>::at(std::size_t row) const
{
    return matrix.at(row);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> T &
Matrix<T, M, N>::at(std::size_t row, std::size_t column)
{
    return matrix.at(row).at(column);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> const T &
Matrix<T, M, N>::at(std::size_t row, std::size_t column) const
{
    return matrix.at(row).at(column);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> std::array<T, N> &
Matrix<T, M, N>::operator[](std::size_t anIndex)
{
    return matrix[anIndex];
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
const std::array<T, N> &
Matrix<T, M, N>::operator[](std::size_t anIndex) const
{
    return matrix[anIndex];
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> Matrix <T, M, N> &
Matrix<T, M, N>::operator=(const Matrix <T, M, N> &rhs)
{
    if (this!=&rhs)
    {
        matrix = rhs.matrix;
    }
    return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix <T, M, N> &
Matrix<T, M, N>::operator=(const std::vector<std::vector<T>> &rhs)
{
    for (size_t row = 0; row < rhs.size(); ++row)
    {
        for (size_t col = 0; col < rhs[row].size(); ++col)
        {
            matrix[row][col] = rhs[row][col];
        }
    }
    return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> bool
Matrix<T, M, N>::operator==(const Matrix <T, M, N> &rhs) const
{
    // return matrix == rhs.matrix;
    return equals(rhs, std::numeric_limits<T>::epsilon()*500.0);
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
bool
Matrix<T, M, N>::operator>(const T2 &rhs) const
{
    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t col = 0; col < N; ++col)
        {
            if (matrix.at(row).at(col) <= rhs)
                return false;
        }
    }
    return true;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
bool
Matrix<T, M, N>::operator<(const T2 &rhs) const
{
    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t col = 0; col < N; ++col)
        {
            if (matrix.at(row).at(col) >= rhs)
                return false;
        }
    }
    return true;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
Matrix <T, M, N> &
Matrix<T, M, N>::operator*=(const T2 &scalar)
{
    static_assert(std::is_arithmetic<T2>::value,
                  "Value T2 must be arithmetic, see "
                  "http://en.cppreference.com/w/cpp/types/is_arithmetic");

    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t column = 0; column < N; ++column)
        {
            matrix.at(row).at(column) *= scalar;
        }
    }
    return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
Matrix <T, M, N>
Matrix<T, M, N>::operator*(const T2 &scalar) const
{
    static_assert(std::is_arithmetic<T2>::value,
                  "Value T2 must be arithmetic, see "
                  "http://en.cppreference.com/w/cpp/types/is_arithmetic");

    return Matrix<T, M, N>(*this) *= scalar;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
Matrix <T, M, N> &
Matrix<T, M, N>::operator/=(const T2 &aScalar)
{
    static_assert(std::is_arithmetic<T2>::value,
                  "Value T2 must be arithmetic, see "
                  "http://en.cppreference.com/w/cpp/types/is_arithmetic");

    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t column = 0; column < N; ++column)
        {
            matrix.at(row).at(column) /= aScalar;
        }
    }
    return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
template<class T2>
Matrix <T, M, N>
Matrix<T, M, N>::operator/(const T2 &aScalar) const
{
    static_assert(std::is_arithmetic<T2>::value,
                  "Value T2 must be arithmetic, see "
                  "http://en.cppreference.com/w/cpp/types/is_arithmetic");

    return Matrix<T, M, N>(*this) /= aScalar;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix <T, M, N> &
Matrix<T, M, N>::operator+=(const Matrix <T, M, N> &rhs)
{
    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t column = 0; column < N; ++column)
        {
            matrix[row][column] += rhs.at(row, column);
        }
    }
    return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix <T, M, N>
Matrix<T, M, N>::operator+(const Matrix <T, M, N> &rhs) const
{
    return Matrix<T, M, N>(*this) += rhs;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix <T, M, N> &
Matrix<T, M, N>::operator-=(const Matrix <T, M, N> &rhs)
{
    // std::cout << "Debug: " << rhs.to_string() << std::endl;
    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t column = 0; column < N; ++column)
        {
            matrix[row][column] -= rhs.at(row, column);
        }
    }
    return *this;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix <T, M, N>
Matrix<T, M, N>::operator-(const Matrix <T, M, N> &rhs) const
{
    return Matrix<T, M, N>(*this) -= rhs;
}
/**
 * (M, N) * (N, O) -> (M, O)
 */
template<typename T, std::size_t M, std::size_t N>
template<std::size_t columns>
Matrix <T, M, columns>
Matrix<T, M, N>::operator*(const Matrix <T, N, columns> &rhs) const
{
    Matrix<T, M, columns> result;
    for (std::size_t row = 0; row < M; ++row)
    {
        for (std::size_t j = 0; j < columns; ++j)
        {
            for (std::size_t column = 0; column < N; ++column)
            {
                result[row][j] += matrix[row][column]*rhs[column][j];
                if (isnan(result[row][j]))
                    result[row][j] = std::numeric_limits<T>::epsilon();
            }
        }
    }
    return result;
}

/**
 *
 */
template<class T, std::size_t M, std::size_t N> Matrix <T, N, M>
Matrix<T, M, N>::transpose() const
{
    Matrix<T, N, M> result;
    for (size_t row = 0; row < M; ++row)
    {
        for (size_t col = 0; col < N; ++col)
        {
            result[col][row] = matrix[row][col];
        }
    }
    return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> Matrix <T, M, N>
Matrix<T, M, N>::identity() const
{
    Matrix<T, M, N> result;
    for (std::size_t m = 0; m < M; ++m)
    {
        for (std::size_t n = 0; n < N; ++n)
        {
            if (m==n)
                result[m][n] = 1;
            else
                result[m][n] = 0;
        }
    }
    return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> Matrix <T, M, N>
Matrix<T, M, N>::gauss() const
{
    Matrix<T, M, N> result(*this);

    // pivotColumn == pivotRow == diagonal
    for (std::size_t pivotColumn = 0; pivotColumn < M; ++pivotColumn)
    {
        // set the correct pivotRow
        std::size_t maxPivotRow = getPivotRow(result, pivotColumn);
        std::swap(result.matrix[pivotColumn], result.matrix[maxPivotRow]);

        // Linearise the rows
        for (std::size_t currentRow = pivotColumn; currentRow < M; ++currentRow)
        {
            T lineariseFactor = getLineariseFactor(result, pivotColumn, currentRow);
            for (std::size_t column = 0; column < N; ++column)
            {
                result[currentRow][column] -= result[pivotColumn][column]*lineariseFactor;
            }
        }
    }
    return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> Matrix <T, M, N>
Matrix<T, M, N>::gaussJordan() const
{
    Matrix<T, M, N> result(*this);

    // pivotColumn == pivotRow == diagonal
    for (std::size_t pivotColumn = 0; pivotColumn < M; ++pivotColumn)
    {
        // set the correct pivotRow
        std::size_t maxPivotRow = getPivotRow(result, pivotColumn);
        std::swap(result.matrix[pivotColumn], result.matrix[maxPivotRow]);

        // Linearise the rows
        for (std::size_t currentRow = 0; currentRow < M; ++currentRow)
        {
            T lineariseFactor = getLineariseFactor(result, pivotColumn, currentRow);
            for (std::size_t column = 0; column < N; ++column)
            {
                result[currentRow][column] -= result[pivotColumn][column]*lineariseFactor;
            }
        }
    }
    return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> Matrix <T, M, N>
Matrix<T, M, N>::inverse() const
{
    static_assert(M==N,
                  "Inverse is only possible on square matrices. Use a "
                  "pseude inverse like moorePenRoseInverse(), svd() or "
                  "the transpose.");
    Matrix<T, M, N> lhs(*this);
    Matrix<T, M, N> rhs(identity());

    // pivotColumn == pivotRow == diagonal
    for (std::size_t pivotColumn = 0; pivotColumn < M; ++pivotColumn)
    {
        // set the correct pivotRow
        std::size_t maxPivotRow = getPivotRow(lhs, pivotColumn);
        std::swap(lhs.matrix[pivotColumn], lhs.matrix[maxPivotRow]);
        std::swap(rhs.matrix[pivotColumn], rhs.matrix[maxPivotRow]);

        // Linearise the rows
        for (std::size_t currentRow = 0; currentRow < M; ++currentRow)
        {
            T lineariseFactor = getLineariseFactor(lhs, pivotColumn, currentRow);
            for (std::size_t column = 0; column < N; ++column)
            {
                lhs[currentRow][column] -= lhs[pivotColumn][column]*lineariseFactor;
                rhs[currentRow][column] -= rhs[pivotColumn][column]*lineariseFactor;
            }
        }
    }
    return rhs;
}

template<class T, std::size_t M, std::size_t N> Matrix <T, M, N>
Matrix<T, M, N>::moorePenroseInverse() const
{
    Matrix<T, M, N> mat(*this);
    return (mat.transpose()*(mat*mat.transpose()).inverse()).transpose();
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N> Matrix<T, M, 1>
Matrix<T, M, N>::solve() const
{
    Matrix<T, M, 1> result;
    Matrix<T, M, N> toSolve = gaussJordan();
    for (std::size_t i = 0; i < M; ++i)
    {
        result[i][0] = toSolve[i][N - 1];
    }
    return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
bool
Matrix<T, M, N>::equals(const Matrix <T, M, N> &rhs, double factor) const
{
    for (std::size_t m = 0; m < M; ++m)
    {
        for (std::size_t n = 0; n < N; ++n)
        {
            if (std::abs(matrix[m][n] - rhs[m][n]) >= factor)
                return false;
        }
    }
    return true;
}

template<class T, std::size_t M, std::size_t N> T
Matrix<T, M, N>::cosineSim(const Matrix <T, M, N> &rhs)
{
    T dotProduct = 0;
    // check if is colVector or rowVector
    if ((M==1 && rhs.getRows()==1) || (N==1 && rhs.getColumns()==1))
    {
        for (std::size_t row = 0; row < M; row++)
        {
            for (std::size_t col = 0; col < N; col++)
            {
                dotProduct += matrix[row][col]*rhs[row][col];
            }
        }

        T squaredSum = 0;
        T squaredSumRHS = 0;
        for (std::size_t row = 0; row < M; row++)
        {
            for (std::size_t cols = 0; cols < N; cols++)
            {
                // pythagoras
                squaredSum += std::pow(matrix[row][cols], 2);
                squaredSumRHS += std::pow(rhs[row][cols], 2);
            }
        }
        return dotProduct/(std::sqrt(squaredSum)*std::sqrt(squaredSumRHS));
    }
    else
    {
        return -1;
    }
}

/**
 *
 */
template<class T, std::size_t M, std::size_t N> std::string
Matrix<T, M, N>::to_string() const
{
    std::string result = "Matrix<" + std::to_string(N) + "," + std::to_string(M) + ">\n{\n";
    for (std::size_t i = 0; i < M; ++i)
    {
        for (std::size_t j = 0; j < N; ++j)
        {
            result += std::to_string(matrix[i][j]) + ",";
        }
        result += "\n";
    }
    result += "}";
    return result;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
Matrix <T, M, N>
Matrix<T, M, N>::from_string(std::istream &stream) const
{
    Matrix<T, M, N> lhs(*this);
    std::vector<std::vector<T>> outputMatrix;

    for (std::string line; std::getline(stream, line);)
    {
        if (line.back()==',')
        {
            line.pop_back();
            std::stringstream ss(line);
            std::string num;
            std::vector<T> cols;
            while (std::getline(ss, num, ','))
            {
                cols.push_back(std::stod(num));
            }
            outputMatrix.push_back(cols);
        }
    }
    lhs = outputMatrix;

    return lhs;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
std::size_t
Matrix<T, M, N>::getPivotRow(const Matrix <T, M, N> &aMatrix, std::size_t pivotColumn) const
{
    T maxPivotValue = 0;
    std::size_t maxPivotRow = 0;

    for (std::size_t row = pivotColumn; row < M; ++row)
    {
        if (maxPivotValue < std::abs(aMatrix.at(row).at(pivotColumn)))
        {
            maxPivotValue = std::abs(aMatrix.at(row).at(pivotColumn));
            maxPivotRow = row;
        }
    }
    return maxPivotRow;
}
/**
 *
 */
template<class T, std::size_t M, std::size_t N>
T
Matrix<T, M, N>::getLineariseFactor(const Matrix <T, M, N> &aMatrix,
                                    std::size_t aPivotColumn,
                                    std::size_t aCurrentRow) const
{
    T lineariseFactor = 0;
    if (aMatrix[aCurrentRow][aPivotColumn]==0.0)
    {
        // No change for a 0-row
        lineariseFactor = 0;
    }
    else if (aCurrentRow==aPivotColumn)
    {
        // use (a-1)/pivot value
        lineariseFactor = (aMatrix[aCurrentRow][aPivotColumn] - 1.0)/aMatrix[aPivotColumn][aPivotColumn];
    }
    else
    {
        // us a/pivot value
        lineariseFactor = aMatrix[aCurrentRow][aPivotColumn]/aMatrix[aPivotColumn][aPivotColumn];
    }
    return lineariseFactor;
}
}