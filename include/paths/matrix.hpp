#ifndef VEXROBOTICS_MATRIX_HPP
#define VEXROBOTICS_MATRIX_HPP

#include <iostream>
#include <utility>
#include <vector>
#include <map>

/**
 * @brief A class to represent a matrix and perform operations on it.
 * 
 * The `Matrix` class is a data structure that represents a matrix and provides
 * methods to perform operations on it. The matrix is stored as a vector of vectors
 * of doubles, where each element is a double. The outer vector represents the rows
 * of the matrix, and each inner vector represents the columns of the matrix.
 * 
 * The class provides methods to perform basic matrix operations such as addition,
 * subtraction, multiplication, and division. It also provides methods to compute
 * the determinant, inverse, and reduced row echelon form of the matrix.
 * 
 * Auxiliary functions are provided to create identity, zeros, and ones matrices.
 */
class Matrix {

    /**
     * @brief The dimensions of the matrix.
     * 
     * The first element of the pair represents the number of rows in the matrix.
     * The second element of the pair represents the number of columns in the matrix.
     */
    std::pair<int, int> matrix_dimensions;

    /**
     * @brief A 2D matrix represented as a vector of vectors of doubles.
     * 
     * This data structure is used to store a matrix where each element is a double.
     * The outer vector represents the rows of the matrix, and each inner vector 
     * represents the columns of the matrix.
     * `internal_matrix_memory[i][j]` represents the element at row `i` and column `j`, starting from index `0`.
     */
    std::vector<std::vector<double>> internal_matrix_memory;

    /**
     * @brief The precision of the output stream.
     * 
     * The number of decimal places to output when printing the matrix.
     */
    int output_precision = 6;

    /**
     * @brief Swaps two rows of the matrix.
     * 
     * @param idx_row_1 The index of the first row to swap.
     * @param idx_row_2 The index of the second row to swap.
     */
    void swap_row(int idx_row_1, int idx_row_2);

    /**
     * @brief Performs a row operation on the matrix.
     * 
     * @param idx_row The index of the row to perform the operation on.
     * @param row_operations_map A map of row indices to scalar values to perform the operation with.
     * A row operation is defined as adding a scalar multiple of one row to another row.
     * An entry `{i, s}` in the map means that `s` times row `i` will be added to the row `idx_row`.
     */
    void row_operation(int idx_row, std::map<int, double> row_operations_map);

public:

    /**
     * @brief Set the precision of the output stream.
     * 
     * @param precision The number of decimal places to output when printing the matrix.
     * @return The new precision of the output stream.
     */
    int set_output_precision(int precision);

    /**
     * @brief Get the dimensions of the matrix.
     * 
     * @return The dimensions of the matrix as a pair of integers.
     */
    const std::pair<int, int> & dimensions();

    /**
     * @brief Access the row at the given index.
     * 
     * @param idx The index of the row to access.
     * @return A reference to the row at the given index.
     */
    std::vector<double>& operator[] (int idx);

    /**
     * @brief Access the element at the given row and column indices, with bounds checking.
     * 
     * @param i The row index.
     * @param j The column index.
     * @return A reference to the element at the given row and column indices.
     * @throws std::out_of_range if the row or column index is out of bounds.
     */
    double & at(int i, int j);

    /**
     * @brief Default constructor for the `Matrix` class.
     * The matrix's dimension is initialized to (0, 0), and its internal memory is default-initialized via `std::vector`.
     */
    Matrix();

    /**
     * @brief Copy constructor for the `Matrix` class.
     * 
     * @param copy_from The matrix to copy from.
     */
    Matrix(Matrix& copy_from);

    /**
     * @brief Move constructor for the `Matrix` class.
     * The old `Matrix` object is left in a valid but unspecified state.
     * Its memory and members should not be accessed.
     * 
     * @param move_from The matrix to move from.
     */
    Matrix(Matrix&& move_from);

    /**
     * @brief Constructor for the `Matrix` class.
     * 
     * @param matrix_dimensions The dimensions of the matrix.
     * @param fill_value The value to fill the matrix with.
     */
    Matrix(std::pair<int, int> matrix_dimensions, double flood_value);

    /**
     * @brief Constructor for the `Matrix` class.
     * 
     * @param matrix_dimensions The dimensions of the matrix.
     * @param fill_values_unsafe_ptr A pointer to the values to fill the matrix with.
     * The values are copied from the pointer to the internal memory of the matrix.
     * The pointer is assumed to be valid and to point to a contiguous block of memory of the correct size.
     * This is not checked, and the behavior is undefined if the pointer is invalid;
     * hence, the operation is unsafe.
     */
    Matrix(std::pair<int, int> matrix_dimensions, double * fill_values_unsafe_ptr);

    /**
     * @brief Constructor for the `Matrix` class.
     * 
     * @param data_values A vector of vectors of doubles to initialize the matrix with.
     * The dimensions of the matrix are set to the dimensions of the vector of vectors.
     * @throws `std::logic_error` if the contained `std::vector`s have different sizes.
     */
    Matrix(std::vector<std::vector<double>> data_values);

    /**
     * @brief Copy assignment operator for the `Matrix` class.
     * 
     * @param copy_from The matrix to copy from.
     * @return A reference to the newly assigned matrix.
     */
    Matrix& operator= (Matrix& copy_from);

    /**
     * @brief Move assignment operator for the `Matrix` class.
     * The old `Matrix` object is left in a valid but unspecified state.
     * Its memory and members should not be accessed.
     * 
     * @param move_from The matrix to move from.
     * @return A reference to the newly assigned matrix.
     */
    Matrix& operator= (Matrix&& move_from);

    /**
     * @brief Unary negation operator for the `Matrix` class.
     * 
     * @return The negated matrix.
     */
    Matrix operator- ();

    /**
     * @brief Addition operator for the `Matrix` class.
     * 
     * @param addend The matrix to add.
     * @return The sum of the two matrices.
     * @throws `std::logic_error` if the dimensions of the two matrices do not match.
     */
    Matrix operator+ (Matrix& addend);

    /**
     * @brief Subtraction operator for the `Matrix` class.
     * 
     * @param subtrahend The matrix to subtract.
     * @return The difference of the two matrices.
     * @throws `std::logic_error` if the dimensions of the two matrices do not match.
     */
    Matrix operator- (Matrix& subtrahend);

    /**
     * @brief Multiplication operator for the `Matrix` class.
     * 
     * @param multiplicand The matrix to multiply.
     * @return The product of the two matrices.
     * @throws `std::logic_error` if the number of columns of the first matrix does 
     * not match the number of rows of the second matrix.
     */
    Matrix operator* (Matrix& multiplicand);

    /**
     * @brief Augmentation operator for the `Matrix` class.
     * The provided matrix is augmented to the right or bottom of the current matrix,
     * depending on the axis.
     * 
     * @param augmentation_values The matrix representing the values of the augmented section.
     * @param axis The axis to augment along. Either 1 (append columns) or 0 (append rows).
     * Default is 1.
     * @return The augmented matrix.
     * @throws `std::logic_error` if `axis` is neither 0 nor 1.
     * @throws `std::logic_error` if the number of rows of the two matrices do not match.
     * @throws `std::logic_error` if the number of columns of the two matrices do not match.
     */
    Matrix augment(Matrix& augmentation_values, int axis = 1);

    /**
     * @brief Transpose operator for the `Matrix` class.
     * The rows and columns of the matrix are interchanged: `Matrix` `A` is
     * said to be the transpose of `Matrix` `B` if `A[i][j] = B[j][i]` for 
     * all `i` and `j`.
     * 
     * @return The transposed matrix.
     */
    Matrix transpose();

    /**
     * @brief Reduced row echelon form operator for the `Matrix` class.
     * The matrix is transformed into its reduced row echelon form.
     * The reduced row echelon form of a matrix is unique and is the result of
     * a series of row operations on the matrix. It is a matrix in which the leading
     * entry of each row is `1`, and the leading entry of each row is to the right of
     * the leading entry of the previous row. Additionally, the leading entry of each
     * row is the only non-zero entry in its column.
     * 
     * @return The reduced row echelon form of the matrix.
     */
    Matrix rref();

    /**
     * @brief Inverse operator for the `Matrix` class.
     * The inverse of a square matrix `A` is a matrix `B` such that `A * B = B * A = I`,
     * where `I` is the identity matrix.
     * 
     * @return The inverse of the matrix.
     * @throws `std::logic_error` if the matrix is not square.
     * @throws `std::logic_error` if the matrix is singular (i.e., its determinant is zero).
     * The threshold for the absolute value of the determinant to be considered zero is `1e-10`.
     */
    Matrix inverse();

    /**
     * @brief Scalar multiplication operator for the `Matrix` class.
     * 
     * @param scalar The scalar to multiply the matrix with.
     * @return The matrix multiplied by the scalar.
     */
    Matrix operator* (double scalar);

    /**
     * @brief Scalar division operator for the `Matrix` class.
     * 
     * @param scalar The scalar to divide the matrix by.
     * @return The matrix divided by the scalar.
     */
    Matrix operator/ (double scalar);

    /**
     * @brief Determinant operator for the `Matrix` class.
     * The determinant of a square matrix is a scalar-valued function that can be computed
     * from its elements. It is denoted as `det(A)` or `|A|`.
     * 
     * @return The determinant of the matrix.
     * @throws `std::logic_error` if the matrix is not square.
     */
    double determinant();

    /**
     * @brief Output stream operator for the `Matrix` class.
     * 
     * @param os The output stream to write to.
     * @param matrix The matrix to write to the output stream.
     * @return The output stream with the matrix written to it.
     */
    friend std::ostream& operator << (std::ostream& os, Matrix& matrix);

    /**
     * @brief Output stream operator for the `Matrix` class.
     * 
     * @param os The output stream to write to.
     * @param matrix The matrix to write to the output stream.
     * @return The output stream with the matrix written to it.
     */
    friend std::ostream& operator << (std::ostream& os, Matrix&& matrix);
    
};

/**
 * @brief Identity matrix constructor.
 * The identity matrix is a square matrix with ones on the main diagonal and zeros elsewhere.
 * 
 * @param n_dimensions The number of dimensions of the identity matrix.
 * @return The identity matrix of the given dimensions.
 */
Matrix identity(int n_dimensions);

/**
 * @brief Zeros matrix constructor.
 * The zeros matrix is a matrix with all elements set to zero.
 * 
 * @param row_count The number of rows of the zeros matrix.
 * @param column_count The number of columns of the zeros matrix.
 * @return The zeros matrix of the given dimensions.
 */
Matrix zeros(int row_count, int column_count);

/**
 * @brief Zeros matrix constructor.
 * The zeros matrix is a matrix with all elements set to zero.
 * 
 * @param reference_matrix The matrix to copy the dimensions from.
 * @return The zeros matrix of the same dimensions as the reference matrix.
 */
Matrix zeros_like(Matrix& reference_matrix);

/**
 * @brief Ones matrix constructor.
 * The ones matrix is a matrix with all elements set to one.
 * 
 * @param row_count The number of rows of the ones matrix.
 * @param column_count The number of columns of the ones matrix.
 * @return The ones matrix of the given dimensions.
 */
Matrix ones(int row_count, int column_count);

/**
 * @brief Ones matrix constructor.
 * The ones matrix is a matrix with all elements set to one.
 * 
 * @param reference_matrix The matrix to copy the dimensions from.
 * @return The ones matrix of the same dimensions as the reference matrix.
 */
Matrix ones_like(Matrix& reference_matrix);

#endif // VEXROBOTICS_MATRIX_HPP