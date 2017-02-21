/*
 * Matlab.h
 *
 *  Created on: Feb 10, 2014
 *      Author: Mingjie Qian
 */

#ifndef MATLAB_H_
#define MATLAB_H_

#include "Vector.h"
#include "Matrix.h"
#include "DenseVector.h"
#include "SparseVector.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include <string>
#include <cmath>

/*
#ifndef EPS
#define EPS
// double eps = (double)1e-64;
#endif
*/

/*
#ifndef MY_INFTY
// #define inf std::numeric_limits<double>::max();
#define MY_INFTY
// double inf = 1.0 / 0.0;
double MY_POSITIVE_INFINITY = 1.0 / 0.0;
double MY_NEGATIVE_INFINITY = -1.0 / 0.0;
#endif
 */

/**
 * Set submatrix of A with selected rows and selected columns by elements of B.
 * B should have the same shape to the submatrix of A to be set. It is equivalent
 * to the syntax A(selectedRows, selectedColumns) = B.
 *
 * @param A a matrix whose submatrix is to be set
 *
 * @param selectedRows {@code int[]} holding indices of selected rows
 *
 * @param numSelRows number of selected rows
 *
 * @param selectedColumns {@code int[]} holding indices of selected columns
 *
 * @param numSelColumns number of selected columns
 *
 * @param B a matrix to set the submatrix of A
 *
 */
void setSubMatrix(Matrix& A, int* selectedRows, int numSelRows,
		int* selectedColumns, int numSelColumns, Matrix& B);

/**
 * Reshape a matrix to a new shape specified number of rows and
 * columns.
 *
 * @param A a matrix
 *
 * @param M number of rows of the new shape
 *
 * @param N number of columns of the new shape
 *
 * @return a new M-by-N matrix whose elements are taken columnwise
 *         from A
 */
Matrix& reshape(Matrix& A, int M, int N);


/**
 * Reshape a matrix to a new shape specified by a two dimensional
 * integer array.
 *
 * @param A a matrix
 *
 * @param size a two dimensional integer array describing a new shape
 *
 * @return a new matrix with a shape specified by size
 *
 */
Matrix& reshape(Matrix& A, int* size);

/**
 * Reshape a vector to a new shape specified number of rows and
 * columns.
 *
 * @param V a vector
 *
 * @param M number of rows of the new shape
 *
 * @param N number of columns of the new shape
 *
 * @return a new M-by-N matrix whose elements are taken from V
 *
 */
Matrix& reshape(Vector& V, int M, int N);

/**
 * Reshape a vector to a matrix with a shape specified by a two dimensional
 * integer array.
 *
 * @param V a vector
 *
 * @param size a two dimensional integer array describing a new shape
 *
 * @return a new matrix with a shape specified by size
 *
 */
Matrix& reshape(Vector& V, int* size);

/**
 * Vectorize a matrix A.
 *
 * @param A a matrix
 *
 * @return Vectorization of a matrix A
 */
Matrix& vec(Matrix& A);

/**
 * Compute the "economy size" matrix singular value decomposition.
 *
 * @param A a real matrix
 *
 * @return a matrix array [U, S, V] where U is left orthonormal matrix, S is a
 * 		   a diagonal matrix, and V is the right orthonormal matrix such that
 *         A = U * S * V'
 *
 */
Matrix** svd(Matrix& A);

/**
 * Replicate and tile an array.
 *
 * @param A a matrix
 *
 * @param M number of rows to replicate
 *
 * @param N number of columns to replicate
 *
 * @return repmat(A, M, N)
 *
 */
Matrix& repmat(Matrix& A, int M, int N);

/**
 * Replicate and tile an array.
 *
 * @param A a matrix
 *
 * @param size a int[2] vector [M N]
 *
 * @return repmat(A, size)
 *
 */
Matrix& repmat(Matrix& A, int* size);

/**
 * Get the dimensionality on dim-th dimension for a matrix A.
 *
 * @param A a matrix
 *
 * @param dim dimension order
 *
 * @return size(A, dim)
 *
 */
int size(Matrix& A, int dim);

/**
 * Compute the sum of elements for each row or each column of
 * a real matrix.
 *
 * @param A a real matrix
 *
 * @param dim 1: column-wise; 2: row-wise
 *
 * @return a dense vector containing the sum of elements of
 * 		   each row or each column of A
 */
DenseVector& sum(Matrix& A, int dim);

/**
 * Compute the sum of elements for each row of a real matrix.
 *
 * @param A a real matrix
 *
 * @return a dense vector containing the sum of elements of
 *         each row of A, i.e. sum(A, 1)
 */
DenseVector& sum(Matrix& A);

/**
 * Compute the sum of a 1D {@code double} array.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 *
 * @return sum(V)

double sum(double* V, int len);*/

/**
 * Compute the sum of all elements of a vector.
 *
 * @param V a real dense or sparse vector
 *
 * @return sum(V)
 */
double sum(Vector& V);

/**
 * M = mean(A, dim) returns the mean values for elements
 * along the dimension of A specified by scalar dim.
 * For matrices, mean(A, 2) is a column vector containing
 * the mean value of each row.
 *
 * @param X a real matrix
 *
 * @param dim dimension order
 *
 * @return mean(A, dim)
 *
 */
Vector& mean(Matrix& X, int dim);

/**
 * Compute eigenvalues and eigenvectors of a symmetric real matrix.
 *
 * @param A a symmetric real matrix
 *
 * @param K number of eigenvalues selected
 *
 * @param sigma either "lm" (largest magnitude) or "sm" (smallest magnitude)
 *
 * @return a matrix array [V, D], V is the selected K eigenvectors (normalized
 *         to 1), and D is a diagonal matrix holding selected K eigenvalues.
 *
 */
Matrix** eigs(Matrix& A, int K, std::string sigma);

/**
 * Construct a sparse diagonal matrix from a vector.
 *
 * @param V a real vector
 *
 * @return diag(V)
 */
SparseMatrix& diag(Vector& V);

/**
 * Generate a diagonal matrix with its elements of a 1D {@code double}
 * array on its main diagonal.
 *
 * @param V a 1D {@code double} array holding the diagonal elements
 *
 * @param len length of V
 *
 * @return diag(V)
 *
 */
Matrix& diag(double* V, int len);

/**
 * Calculate the element-wise logarithm of a matrix.
 *
 * @param A a matrix
 *
 * @return log(A)
 *
 */
DenseMatrix& log(Matrix& A);

/**
 * Calculate TFIDF of a doc-term-count matrix, each column
 * is a data sample.
 *
 * @param docTermCountMatrix a matrix, each column is a data sample
 *
 * @return TFIDF of docTermCountMatrix
 *
 */
Matrix& getTFIDF(Matrix& docTermCountMatrix);

/**
 * Normalize A by columns.
 *
 * @param A a matrix
 *
 * @return a column-wise normalized matrix
 */
Matrix& normalizeByColumns(Matrix& A);

/**
 * Calculate square root for all elements of a vector V.
 *
 * @param V a real vector
 *
 * @return sqrt(V)
 */
Vector& sqrt(Vector& V);

/**
 * Calculate square root for all elements of a matrix A.
 *
 * @param A a matrix
 *
 * @return sqrt(A)
 *
 */
Matrix& sqrt(Matrix& A);

/**
 * Random permutation.
 * </br>
 * randperm(n) returns a random permutation of the integers 1:n.
 *
 * @param n an integer
 *
 * @return randperm(n)
 */
int* randperm(int n);

/**
 * <h4>A wrapper for the output of find function.</h4>
 * There are three data members:<br/>
 * rows: row indices array for non-zero elements of a matrix<br/>
 * cols: column indices array for non-zero elements of a matrix<br/>
 * vals: values array for non-zero elements of a matrix<br/>
 */
class FindResult {

public:

	int* rows;
	int* cols;
	double* vals;
	int len;

	FindResult(int* rows, int* cols, double* vals, int len) {
		this->rows = rows;
		this->cols = cols;
		this->vals = vals;
		this->len = len;
	}

	~FindResult() {
		delete[] rows;
		delete[] cols;
		delete[] vals;
	}

};

/**
 * Find nonzero elements and return their indices.
 *
 * @param V a real vector
 *
 * @return an integer array of indices of nonzero elements of V
 *
 */
int* find(Vector& V);

/**
 * Find nonzero elements and return their value, row and column indices.
 *
 * @param A a matrix
 *
 * @return a {@code FindResult} data structure which has three instance
 * data members:<br/>
 * rows: row indices array for non-zero elements of a matrix<br/>
 * cols: column indices array for non-zero elements of a matrix<br/>
 * vals: values array for non-zero elements of a matrix<br/>
 *
 */
FindResult& find(Matrix& A);

/**
 * Compute the element-wise exponential of a matrix
 *
 * @param A a matrix
 *
 * @return exp(A)
 *
 */
Matrix& exp(Matrix& A);

/**
 * Concatenate matrices vertically. All matrices in the argument
 * list must have the same number of columns.
 *
 * @param As matrices to be concatenated vertically
 *
 * @param numMatrix number of matrices
 *
 * @return [A1; A2; ...]
 *
 */
Matrix& vertcat(int numMatrix, ...);

/**
 * Concatenate matrices vertically. All matrices in the argument
 * list must have the same number of columns.
 *
 * @param As matrices to be concatenated vertically
 *
 * @param numMatrix number of matrices
 *
 * @return [A1; A2; ...]
 *
 */
Matrix& vertcat(Matrix** As, int numMatrix);

/**
 * Concatenate matrices horizontally. All matrices in the argument
 * list must have the same number of rows.
 *
 * @param As matrices to be concatenated horizontally
 *
 * @param numMatrix number of matrices
 *
 * @return [A1 A2 ...]
 *
 */
Matrix& horzcat(int numMatrix, ...);

/**
 * Concatenate matrices horizontally. All matrices in the argument
 * list must have the same number of rows.
 *
 * @param As matrices to be concatenated horizontally
 *
 * @param numMatrix number of matrices
 *
 * @return [A1 A2 ...]
 *
 */
Matrix& horzcat(Matrix** As, int numMatrix);

/**
 * Concatenate matrices along specified dimension.
 *
 * @param dim specified dimension, can only be either 1 or 2 currently
 *
 * @param As matrices to be concatenated
 *
 * @param numMatrix number of matrices
 *
 * @return a concatenation of all the matrices in the argument list
 *
 */
Matrix& cat(int dim, Matrix** As, int numMatrices);

/**
 * Compute the sum of all elements of a matrix.
 *
 * @param A a matrix
 *
 * @return sum(sum(A))
 *
 */
double sumAll(Matrix& A);

/**
 * If A is a 1-row or 1-column matrix, then diag(A) is a
 * sparse diagonal matrix with elements of A as its main diagonal,
 * else diag(A) is a column matrix holding A's diagonal elements.
 *
 * @param A a matrix
 *
 * @return diag(A)
 *
 */
Matrix& diag(Matrix& A);

/**
 * Right array division.
 *
 * @param A a matrix
 *
 * @param v a scalar
 *
 * @return A ./ v
 */
Matrix& rdivide(Matrix& A, double v);

/**
 * Generate an nRow-by-nCol matrix containing pseudo-random values drawn
 * from the standard uniform distribution on the open interval (0,1).
 *
 * @param nRow number of rows
 *
 * @param nCol number of columns
 *
 * @return rand(nRow, nCol)
 *
 */
Matrix& rand(int nRow, int nCol);

/**
 * Generate an n-by-n matrix containing pseudo-random values drawn
 * from the standard uniform distribution on the open interval (0,1).
 *
 * @param n number of rows or columns
 *
 * @return rand(n, n)
 *
 */
Matrix& rand(int n);

/**
 * Generate an nRow-by-nCol matrix containing pseudo-random values drawn
 * from the standard normal distribution.
 *
 * @param nRow number of rows
 *
 * @param nCol number of columns
 *
 * @return randn(nRow, nCol)
 *
 */
Matrix& randn(int nRow, int nCol);

/**
 * Generate an n-by-n matrix containing pseudo-random values drawn
 * from the standard normal distribution.
 *
 * @param n number of rows or columns
 *
 * @return randn(n, n)
 *
 */
Matrix& randn(int n);

/**
 * Signum function.
 * <p>
 * For each element of X, SIGN(X) returns 1 if the element
 * is greater than zero, 0 if it equals zero and -1 if it is
 * less than zero.
 * </p>
 *
 * @param A a real matrix
 *
 * @return sign(A)
 *
 */
Matrix& sign(Matrix& A);

/**
 * Compute the squared l2 distance matrix between row vectors in matrix X
 * and row vectors in matrix Y.
 *
 * @param X
 *        Data matrix with each row being a feature vector.
 *
 * @param Y
 *        Data matrix with each row being a feature vector.
 *
 * @return an n_x X n_y matrix with its (i, j) entry being the squared l2
 * distance between i-th feature vector in X and j-th feature
 * vector in Y, i.e., || X(i, :) - Y(j, :) ||_2^2
 *
 */
Matrix& l2DistanceSquare(Matrix& X, Matrix& Y);

/**
 * Compute the squared l2 distance vector between a vector V
 * and row vectors in matrix Y.
 *
 * @param V
 *        a feature vector
 *
 * @param Y
 *        Data matrix with each row being a feature vector
 *
 * @return an n_y dimensional vector with its i-th entry being the squared l2
 * distance between V and the i-th feature vector in Y, i.e., || V - Y(i, :) ||_2^2
 *
 */
Vector& l2DistanceSquare(Vector& V, Matrix& Y);

/**
 * Compute the squared l2 distance matrix between column vectors in matrix X
 * and column vectors in matrix Y.
 *
 * @param X
 *        Data matrix with each column being a feature vector.
 *
 * @param Y
 *        Data matrix with each column being a feature vector.
 *
 * @return an n_x X n_y matrix with its (i, j) entry being the squared l2
 * distance between i-th feature vector in X and j-th feature
 * vector in Y, i.e., || X(:, i) - Y(:, j) ||_2^2
 *
 */
Matrix& l2DistanceSquareByColumns(Matrix& X, Matrix& Y);

/**
 * Compute the squared l2 distance matrix between two sets of vectors X and Y.
 *
 * @param X
 *        Data vectors
 *
 * @param Y
 *        Data vectors
 *
 * @return an n_x X n_y matrix with its (i, j) entry being the squared l2
 * distance between i-th feature vector in X and j-th feature
 * vector in Y, i.e., || X[i] - Y[j] ||_2^2
 *
 */
Matrix& l2DistanceSquare(Vector** X, int lenX, Vector** Y, int lenY);

/**
 * Compute the l2 distance matrix between column vectors in matrix X
 * and column vectors in matrix Y.
 *
 * @param X
 *        Data matrix with each column being a feature vector.
 *
 * @param Y
 *        Data matrix with each column being a feature vector.
 *
 * @return an n_x X n_y matrix with its (i, j)th entry being the l2
 * distance between i-th feature vector in X and j-th feature
 * vector in Y, i.e., || X(:, i) - Y(:, j) ||_2
 *
 */
Matrix& l2DistanceByColumns(Matrix& X, Matrix& Y);

/**
 * Compute the l2 distance matrix between row vectors in matrix X
 * and row vectors in matrix Y.
 *
 * @param X
 *        Data matrix with each row being a feature vector.
 *
 * @param Y
 *        Data matrix with each row being a feature vector.
 *
 * @return an n_x X n_y matrix with its (i, j)th entry being the l2
 * distance between i-th feature vector in X and j-th feature
 * vector in Y, i.e., || X(i, :) - Y(j, :) ||_2
 *
 */
Matrix& l2Distance(Matrix& X, Matrix& Y);

/**
 * Compute the l2 distance vector between a vector V
 * and row vectors in matrix Y.
 *
 * @param V
 *        a feature vector
 *
 * @param Y
 *        Data matrix with each row being a feature vector
 *
 * @return an n_y dimensional vector with its i-th entry being the l2
 * distance between V and the i-th feature vector in Y, i.e., || V - Y(i, :) ||_2
 *
 */
Vector& l2Distance(Vector& V, Matrix& Y);

/**
 * Compute the l2 distance matrix between two sets of vectors X and Y.
 *
 * @param X
 *        Data vectors
 *
 * @param Y
 *        Data vectors
 *
 * @return an n_x X n_y matrix with its (i, j)th entry being the l2
 * distance between i-th feature vector in X and j-th feature
 * vector in Y, i.e., || X[i] - Y[j] ||_2
 *
 */
Matrix& l2Distance(Vector** X, int lenX, Vector** Y, int lenY);

/**
 * Compute the inner product of two vectors, i.e. res = <V1, V2>.
 *
 * @param V1 the first vector
 *
 * @param V2 the second vector
 *
 * @return <V1, V2>
 */
double innerProduct(Vector& V1, Vector& V2);

/**
 * Compute the sum of all elements of a matrix.
 *
 * @param A a matrix
 *
 * @return sum(sum(A))
 *
 */
double innerProduct(Matrix& A, Matrix& B);

/**
 * Calculate element by element division between a scalar and a vector.
 *
 * @param v a real scalar
 *
 * @param V a real vector
 *
 * @return v ./ V
 */
Vector& dotDivide(double v, Vector& V);

/**
 * Generate nRow by nCol all zero matrix.
 *
 * @param nRow number of rows
 *
 * @param nCol number of columns
 *
 * @return zeros(nRow, nCol)
 *
 */
Matrix& zeros(int nRow, int nCol);

/**
 * Generate an all zero matrix with its size
 * specified by a two dimensional integer array.
 *
 * @param size a two dimensional integer array
 *
 * @return an all zero matrix with its shape specified by size
 *
 */
Matrix& zeros(int* size);

/**
 * Generate an n by n all zero matrix.
 *
 * @param n number of rows and columns
 *
 * @return ones(n)
 *
 */
Matrix& zeros(int n);

/**
 * Generate an all one matrix with nRow rows and nCol columns.
 *
 * @param nRow number of rows
 *
 * @param nCol number of columns
 *
 * @return ones(nRow, nCol)
 *
 */
Matrix& ones(int nRow, int nCol);

/**
 * Generate an all one matrix with its size
 * specified by a two dimensional integer array.
 *
 * @param size a two dimensional integer array
 *
 * @return an all one matrix with its shape specified by size
 *
 */
Matrix& ones(int* size);

/**
 * Generate an n by n all one matrix.
 *
 * @param n number of rows and columns
 *
 * @return ones(n)
 *
 */
Matrix& ones(int n);

/**
 * Compute the determinant of a real square matrix.
 *
 * @param A a real square matrix
 *
 * @return det(A)
 */
double det(Matrix& A);

/**
 * Compute the inverse of a real square matrix.
 *
 * @param A a real square matrix
 *
 * @return inv(A)
 */
Matrix& inv(Matrix& A);

/**
 * Sort elements of a vector V in place with an increasing order.
 *
 * @param V a real vector, it will be sorted in place.
 *
 * @return sorted indices represented by a 1D {@code double} array
 */
double* sort(Vector& V);

/**
 * Sort elements of a vector V in place with a specified order.
 *
 * @param V a real vector, it will be sorted in place.
 *
 * @param order a {@code String} variable either "ascend" or "descend"
 *
 * @return sorted indices represented by a 1D {@code double} array
 */
double* sort(Vector& V, std::string order);

/**
 * Sort elements of a matrix A on a direction in a specified order.
 * A will not be modified.
 *
 * @param A a matrix to be sorted
 *
 * @param dim sorting direction, 1 for column-wise, 2 for row-wise
 *
 * @param order sorting order, either "ascend" or "descend"
 *
 * @return a {@code Matrix} array:
 *         res[0] is the sorted matrix
 *         res[1] is the sorted indices
 *
 */
Matrix** sort(Matrix& A, int dim, std::string order);

/**
 * Sort elements of a matrix A on a direction in an increasing order.
 *
 * @param A a matrix to be sorted
 *
 * @param dim sorting direction, 1 for column-wise, 2 for row-wise
 *
 * @return a {@code Matrix} array:
 *         res[0] is the sorted matrix
 *         res[1] is the sorted indices
 *
 */
Matrix** sort(Matrix& A, int dim);

/**
 * Sort elements of a matrix A by columns in a specified order.
 * A will not be modified.
 *
 * @param A a matrix to be sorted
 *
 * @param order sorting order, either "ascend" or "descend"
 *
 * @return a {@code Matrix} array:
 *         res[0] is the sorted matrix
 *         res[1] is the sorted indices
 *
 */
Matrix** sort(Matrix& A, std::string order);

/**
 * Sort elements of a matrix A by columns in an increasing order.
 * A will not be modified.
 *
 * @param A a matrix to be sorted
 *
 * @return a {@code Matrix} array:
 *         res[0] is the sorted matrix
 *         res[1] is the sorted indices
 *
 */
Matrix** sort(Matrix& A);

/**
 * Get a two dimensional integer array for size of a matrix A.
 *
 * @param A a matrix
 *
 * @return size(A)
 *
 */
int* size(Matrix& A);

/**
 * Compute maximum between elements of A and a real number and return
 * as a matrix with the same shape of A.
 *
 * @param A a real matrix
 *
 * @param v a real number
 *
 * @return max(A, v)
 *
 */
Matrix& max(Matrix& A, double v);

/**
 * Compute maximum between elements of A and a real number and return
 * as a matrix with the same shape of A.
 *
 * @param v a real number
 *
 * @param A a real matrix
 *
 * @return max(v, A)
 *
 */
Matrix& max(double v, Matrix& A);

/**
 * Compute maximum between two real matrices A and B.
 *
 * @param A a real matrix
 *
 * @param B a real matrix
 *
 * @return max(A, B);
 */
Matrix& max(Matrix& A, Matrix& B);

/**
 * Compute minimum between elements of A and a real number and return
 * as a matrix with the same shape of A.
 *
 * @param A a real matrix
 *
 * @param v a real number
 *
 * @return min(A, v)
 *
 */
Matrix& min(Matrix& A, double v);

/**
 * Compute minimum between elements of A and a real number and return
 * as a matrix with the same shape of A.
 *
 * @param v a real number
 *
 * @param A a real matrix
 *
 * @return min(v, A)
 *
 */
Matrix& min(double v, Matrix& A);

/**
 * Compute minimum between two real matrices A and B.
 *
 * @param A a real matrix
 *
 * @param B a real matrix
 *
 * @return max(A, B);
 */
Matrix& min(Matrix& A, Matrix& B);

/**
 * Compute maximum between elements of V and a real number
 * and return as a vector with the same shape of V.
 *
 * @param V a real vector
 *
 * @param v a real number
 *
 * @return max(V, v)
 *
 */
Vector& max(Vector& V, double v);

/**
 * Compute maximum between elements of V and a real number
 * and return as a vector with the same shape of V.
 *
 * @param v a real number
 *
 * @param V a real vector
 *
 * @return max(v, V)
 *
 */
Vector& max(double v, Vector& V);

/**
 * Compute maximum between two vectors U and V.
 *
 * @param U a real vector
 *
 * @param V a real vector
 *
 * @return max(U, V)
 *
 */
Vector& max(Vector& U, Vector& V);

/**
 * Compute minimum between elements of V and a real number
 * and return as a vector with the same shape of V.
 *
 * @param V a real vector
 *
 * @param v a real number
 *
 * @return min(V, v)
 *
 */
Vector& min(Vector& V, double v);

/**
 * Compute minimum between elements of V and a real number
 * and return as a vector with the same shape of V.
 *
 * @param v a real number
 *
 * @param V a real vector
 *
 * @return min(v, V)
 *
 */
Vector& min(double v, Vector& V);

/**
 * Compute minimum between two vectors U and V.
 *
 * @param U a real vector
 *
 * @param V a real vector
 *
 * @return min(U, V)
 *
 */
Vector& min(Vector& U, Vector& V);

/**
 * Logical indexing A by B for the syntax A(B). A logical matrix
 * provides a different type of array indexing in MATLAB. While
 * most indices are numeric, indicating a certain row or column
 * number, logical indices are positional. That is, it is the
 * position of each 1 in the logical matrix that determines which
 * array element is being referred to.
 *
 * @param A a real matrix
 *
 * @param B a logical matrix with elements being either 1 or 0
 *
 * @return A(B)
 *
 */
Matrix& logicalIndexing(Matrix& A, Matrix& B);

/**
 * Linear indexing V by an index array.
 *
 * @param V an {@code int} array
 *
 * @param indices an {@code int} array of selected indices
 *
 * @param len length of the index array
 *
 * @return V(indices)
 *
 */
int* linearIndexing(int* V, int* indices, int len);

/**
 * Linear indexing V by an index array.
 *
 * @param V an {@code double} array
 *
 * @param indices an {@code int} array of selected indices
 *
 * @param len length of the index array
 *
 * @return V(indices)
 *
 */
double* linearIndexing(double* V, int* indices, int len);

/**
 * Linear indexing A by an index array.
 *
 * @param A a real matrix
 *
 * @param indices an {@code int} array of selected indices
 *
 * @param len length of the index array
 *
 * @return A(indices)
 *
 */
Matrix& linearIndexing(Matrix& A, int* indices, int len);

/**
 * Matrix assignment by linear indexing for the syntax A(B) = V.
 *
 * @param A a matrix to be assigned
 *
 * @param indices a linear index array
 *
 * @param len length of the linear index array
 *
 * @param V a column matrix to assign A(idx)
 *
 */
void linearIndexingAssignment(Matrix& A, int* indices, int len, Matrix& V);

/**
 * Matrix assignment by linear indexing for the syntax A(B) = v.
 *
 * @param A a matrix to be assigned
 *
 * @param indices a linear index array
 *
 * @param len length of the linear index array
 *
 * @param v a real scalar to assign A(idx)
 *
 */
void linearIndexingAssignment(Matrix& A, int* indices, int len, double v);

/**
 * Matrix assignment by logical indexing for the syntax A(B) = v.
 *
 * @param A a matrix to be assigned
 *
 * @param B a logical matrix where position of each 1 determines
 *          which array element is being assigned
 *
 * @param v a real scalar to assign A(B)
 *
 */
void logicalIndexingAssignment(Matrix& A, Matrix& B, double v);

/**
 * Matrix assignment by logical indexing for the syntax A(B) = V.
 *
 * @param A a matrix to be assigned
 *
 * @param B a logical matrix where position of each 1 determines
 *          which array element is being assigned
 *
 * @param V a column matrix to assign A(B)
 *
 */
void logicalIndexingAssignment(Matrix& A, Matrix& B, Matrix& V);

/**
 * Get the nonnegative part of a matrix A.
 *
 * @param A a matrix
 *
 * @return a matrix which is the nonnegative part of a matrix A
 *
 */
Matrix& subplus(Matrix& A);

/**
 * Returns an array that contains 1's where
 * the elements of X are NaN's and 0's where they are not.
 *
 * @param A a matrix
 *
 * @return a 0-1 matrix: isnan(A)
 *
 */
Matrix& isnan(Matrix& A);

/**
 * returns an array that contains 1's where the
 * elements of X are +Inf or -Inf and 0's where they are not.
 *
 * @param A a real matrix
 *
 * @return a 0-1 matrix: isinf(A)
 *
 */
Matrix& isinf(Matrix& A);

Matrix& minus(Matrix& A, double v);

/**
 * res = v - A.
 * @param v
 * @param A
 * @return v - A
 */
Matrix& minus(double v, Matrix& A);

Matrix& times(Matrix& A, double v);

Matrix& mtimes(Matrix& M1, Matrix& M2);

Matrix& times(double v, Matrix& A);

Matrix& plus(Matrix& A, double v);

Matrix& plus(double v, Matrix& A);

/**
 * Set all the elements of X to be those of Y, this is particularly
 * useful when we want to change elements of the object referred by X
 * rather than the reference X itself.
 *
 * @param X a matrix to be set
 *
 * @param Y a matrix to set X
 *
 */
void setMatrix(Matrix& X, Matrix& Y);

/**
 * Unary minus.
 *
 * @param A a matrix
 *
 * @return -A
 */
Matrix& uminus(Matrix& A);

/**
 * Do element by element comparisons between A and B and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 * A and B must have the same dimensions.
 *
 * @param A a matrix
 *
 * @param B a matrix
 *
 * @return A | B or or(A, B)
 *
 */
Matrix& _or(Matrix& A, Matrix& B);

/**
 * Do element by element comparisons between A and B and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 * A and B must have the same dimensions.
 *
 * @param A a matrix
 *
 * @param B a matrix
 *
 * @return A & B or and(A, B)
 *
 */
Matrix& _and(Matrix& A, Matrix& B);

/**
 * Performs a logical NOT of input array A, and returns an array
 * containing elements set to either 1 (TRUE) or 0 (FALSE). An
 * element of the output array is set to 1 if A contains a zero
 * value element at that same array location. Otherwise, that
 * element is set to 0.
 *
 * @param A a matrix
 *
 * @return ~A or not(A)
 *
 */
Matrix& _not(Matrix& A);

/**
 * Do element by element comparisons between X and Y and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param X a matrix
 *
 * @param Y a matrix
 *
 * @return X ~= Y or ne(X, Y)
 *
 */
Matrix& ne(Matrix& X, Matrix& Y);

/**
 * Do element by element comparisons between X and x and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param X a matrix
 *
 * @param x a real scalar
 *
 * @return X ~= x or ne(X, x)
 *
 */
Matrix& ne(Matrix& X, double x);

/**
 * Do element by element comparisons between X and x and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param x a real scalar
 *
 * @param X a matrix
 *
 * @return X ~= x or ne(X, x)
 *
 */
Matrix& ne(double x, Matrix& X);

/**
 * Do element by element comparisons between X and Y and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param X a matrix
 *
 * @param Y a matrix
 *
 * @return X == Y or eq(X, Y)
 *
 */
Matrix& eq(Matrix& X, Matrix& Y);

/**
 * Do element by element comparisons between X and x and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param X a matrix
 *
 * @param x a real scalar
 *
 * @return X == x or eq(X, x)
 *
 */
Matrix& eq(Matrix& X, double x);

/**
 * Do element by element comparisons between X and x and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param x a real scalar
 *
 * @param X a matrix
 *
 * @return X == x or eq(X, x)
 *
 */
Matrix& eq(double x, Matrix& X);

/**
 * Do element by element comparisons between X and x and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param X a matrix
 *
 * @param x a real scalar
 *
 * @return X >= x or ge(X, x)
 *
 */
Matrix& ge(Matrix& X, double x);

/**
 * Do element by element comparisons between x and X and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param x a real scalar
 *
 * @param X a matrix
 *
 * @return x >= X or ge(x, X)
 */
Matrix& ge(double x, Matrix& X);

/**
 * Do element by element comparisons between X and Y and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 * X and Y must have the same dimensions.
 *
 * @param X a matrix
 *
 * @param Y a matrix
 *
 * @return X >= Y or ge(X, Y)
 *
 */
Matrix& ge(Matrix& X, Matrix& Y);

/**
 * Do element by element comparisons between X and x and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param X a matrix
 *
 * @param x a real scalar
 *
 * @return X <= x or le(X, x)
 *
 */
Matrix& le(Matrix& X, double x);

/**
 * Do element by element comparisons between x and X and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param x a real scalar
 *
 * @param X a matrix
 *
 * @return x <= X or le(x, X)
 *
 */
Matrix& le(double x, Matrix& X);

/**
 * Do element by element comparisons between X and Y and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 * X and Y must have the same dimensions.
 *
 * @param X a matrix
 *
 * @param Y a matrix
 *
 * @return X <= Y or le(X, Y)
 *
 */
Matrix& le(Matrix& X, Matrix& Y);

/**
 * Do element by element comparisons between X and x and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param X a matrix
 *
 * @param x a real scalar
 *
 * @return X > x or gt(X, x)
 *
 */
Matrix& gt(Matrix& X, double x);

/**
 * Do element by element comparisons between x and X and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param x a real scalar
 *
 * @param X a matrix
 *
 * @return x > X or gt(x, X)
 */
Matrix& gt(double x, Matrix& X);

/**
 * Do element by element comparisons between X and Y and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 * X and Y must have the same dimensions.
 *
 * @param X a matrix
 *
 * @param Y a matrix
 *
 * @return X > Y or gt(X, Y)
 *
 */
Matrix& gt(Matrix& X, Matrix& Y);

/**
 * Do element by element comparisons between X and x and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param X a matrix
 *
 * @param x a real scalar
 *
 * @return X < x or lt(X, x)
 *
 */
Matrix& lt(Matrix& X, double x);

/**
 * Do element by element comparisons between x and X and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 *
 * @param x a real scalar
 *
 * @param X a matrix
 *
 * @return x < X or lt(x, X)
 *
 */
Matrix& lt(double x, Matrix& X);

/**
 * Do element by element comparisons between X and Y and returns
 * a matrix of the same size with elements set to 1 where the
 * relation is true and elements set to 0 where it is not.
 * X and Y must have the same dimensions.
 *
 * @param X a matrix
 *
 * @param Y a matrix
 *
 * @return X < Y or lt(X, Y)
 *
 */
Matrix& lt(Matrix& X, Matrix& Y);

/**
 * Compute element-wise absolute value of all elements of matrix.
 *
 * @param A a matrix
 *
 * @return abs(A)
 */
Matrix& abs(Matrix& A);

/**
 * Compute element-wise absolute value of all elements of a vector.
 *
 * @param V a vector
 *
 * @return abs(V)
 */
Vector& abs(Vector& V);

/**
 * Compute element-wise exponentiation of a vector.
 *
 * @param A a real matrix
 *
 * @param p exponent
 *
 * @return A.^p
 */
Matrix& pow(Matrix& A, double p);

/**
 * Compute element-wise exponentiation of a vector.
 *
 * @param V a real vector
 *
 * @param p exponent
 *
 * @return V.^p
 */
Vector& pow(Vector& V, double p);

/**
 * Compute the maximal value and the corresponding row
 * index of a vector V. The index starts from 0.
 *
 * @param V a real vector
 *
 * @return a {@code double} array [M, IX] where M is the
 *         maximal value, and IX is the corresponding
 * 		   index
 */
double* max(Vector& V);

/**
 * Compute the maximal value for each column and the
 * corresponding row indices of a matrix A. The row index
 * starts from 0.
 *
 * @param A a real matrix
 *
 * @return a {@code Vector} array [M, IX] where M contains
 * 		   the maximal values, and IX contains the corresponding
 * 		   indices
 */
Vector** max(Matrix& A);

/**
 * Compute the maximal value for each row or each column
 * and the corresponding indices of a matrix A. The row or
 * column indices start from 0.
 *
 * @param A a 2D {@code double} array
 *
 * @param numRows number of rows
 *
 * @param numColumns number of columns
 *
 * @param dim 1: column-wise; 2: row-wise
 *
 * @return a {@code double[]} array [M, IX] where M contains
 * 		   the maximal values, and IX contains the corresponding
 * 		   indices
 */
double** max(double** A, int numRows, int numColumns, int dim);

/**
 * Compute the maximal value for each row or each column
 * and the corresponding indices of a matrix A. The row or
 * column indices start from 0.
 *
 * @param A a real matrix
 *
 * @param dim 1: column-wise; 2: row-wise
 *
 * @return a {@code Vector} array [M, IX] where M contains
 * 		   the maximal values, and IX contains the corresponding
 * 		   indices
 */
Vector** max(Matrix& A, int dim);

/**
 * Compute the minimal value and the corresponding row
 * index of a vector V. The index starts from 0.
 *
 * @param V a real vector
 *
 * @return a {@code double} array [M, IX] where M is the
 *         minimal value, and IX is the corresponding
 * 		   index
 */
double* min(Vector& V);

/**
 * Compute the minimal value for each column and the
 * corresponding row indices of a matrix A. The row index
 * starts from 0.
 *
 * @param A a real matrix
 *
 * @return a {@code Vector} array [M, IX] where M contains
 * 		   the minimal values, and IX contains the corresponding
 * 		   indices
 */
Vector** min(Matrix& A);

/**
 * Compute the minimal value for each row or each column
 * and the corresponding indices of a matrix A. The row or
 * column indices start from 0.
 *
 * @param A a real matrix
 *
 * @param dim 1: column-wise; 2: row-wise
 *
 * @return a {@code Vector} array [M, IX] where M contains
 * 		   the minimal values, and IX contains the corresponding
 * 		   indices
 */
Vector** min(Matrix& A, int dim);

/**
 * Compute the Euclidean norm of a matrix.
 *
 * @param A a real matrix
 *
 * @return ||A||_2
 */
double norm(Matrix& A);

/**
 * Compute the induced vector norm of a matrix (row or column
 * matrices are allowed).
 *
 * @param A a matrix or a vector
 *
 * @param type 1, 2, or, inf for a matrix or a positive real
 *             number for a row or column matrix
 *
 * @return ||A||_{type}
 *
 */
double norm(Matrix& A, double type);

/**
 * Compute the norm of a matrix (row or column matrices
 * are allowed).
 *
 * @param A a matrix
 *
 * @param type 1, 2
 *
 * @return ||A||_{type}
 *
 */
double norm(Matrix& A, int type);

/**
 * Calculate the Frobenius norm of a matrix A.
 *
 * @param A a matrix
 *
 * @param type can only be "fro"
 *
 * @return ||A||_F
 *
 */
double norm(Matrix& A, std::string type);

/**
 * Compute the norm of a vector.
 *
 * @param V a real vector
 *
 * @param p a positive {@code double} value
 *
 * @return ||V||_p
 */
double norm(Vector& V, double p);

/**
 * Compute the Euclidean norm of a vector.
 *
 * @param V a real vector
 *
 * @return ||V||_2
 */
double norm(Vector& V);

/**
 * Calculate the sigmoid of a matrix A by rows. Specifically, supposing
 * that the input activation matrix is [a11, a12; a21, a22], the output
 * value is
 * <p>
 * [exp(a11) / exp(a11) + exp(a12), exp(a12) / exp(a11) + exp(a12);
 * </br>
 * exp(a21) / exp(a21) + exp(a22), exp(a22) / exp(a21) + exp(a22)].
 *
 * @param A a real matrix
 *
 * @return sigmoid(A)
 */
Matrix& sigmoid(Matrix& A);

/**
 * Compute the maximum of elements in a 1D {@code double} array.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 *
 * @return max(V)
 */
double max(double* V, int len);

/**
 * Compute the maximum of elements in an interval
 * of a 1D {@code double} array.
 *
 * @param V a 1D {@code double} array
 *
 * @param start start index (inclusive)
 *
 * @param end end index (inclusive)
 *
 * @return max(V(start:end))
 */
double max(double* V, int start, int end);

/**
 * Calculate the left division of the form A \ B. A \ B is the
 * matrix division of A into B, which is roughly the same as
 * INV(A)*B , except it is computed in a different way. For
 * implementation, we actually solve the system of linear
 * equations A * X = B.
 *
 * @param A divisor
 *
 * @param B dividend
 *
 * @return A \ B
 *
 */
Matrix& mldivide(Matrix& A, Matrix& B);

/**
 * Calculate the right division of B into A, i.e., A / B. For
 * implementation, we actually solve the system of linear
 * equations X * B = A.
 * <p>
 * Note: X = A / B <=> X * B = A <=> B' * X' = A' <=> X' = B' \ A'
 * <=> X = (B' \ A')'
 * </p>
 *
 * @param A dividend
 *
 * @param B divisor
 *
 * @return A / B
 *
 */
Matrix& mrdivide(Matrix& A, Matrix& B);

/**
 * Compute the rank of a matrix. The rank function provides
 * an estimate of the number of linearly independent rows or
 * columns of a matrix.
 *
 * @param A a matrix
 *
 * @return rank of the given matrix
 */
int rank(Matrix& A);

/**
 * Construct an m-by-n dense identity matrix.
 *
 * @param m number of rows
 *
 * @param n number of columns
 *
 * @return an m-by-n dense identity matrix
 *
 */
DenseMatrix& eye(int m, int n);

/**
 * Construct an n-by-n dense identity matrix.
 *
 * @param n number of rows and columns
 *
 * @return an n-by-n dense identity matrix
 *
 */
DenseMatrix& eye(int n);

/**
 * Generate a dense identity matrix with its size
 * specified by a two dimensional integer array.
 *
 * @param size a two dimensional integer array
 *
 * @return a dense identity matrix with its shape specified by size
 *
 */
Matrix& eye(int* size);

/**
 * Construct an m-by-n sparse identity matrix.
 *
 * @param m number of rows
 *
 * @param n number of columns
 *
 * @return an m-by-n sparse identity matrix
 *
 */
SparseMatrix& speye(int m, int n);

/**
 * Construct an n-by-n sparse identity matrix.
 *
 * @param n number of rows and columns
 *
 * @return an n-by-n sparse identity matrix
 *
 */
SparseMatrix& speye(int n);

/**
 * Generate a sparse identity matrix with its size
 * specified by a two dimensional integer array.
 *
 * @param size a two dimensional integer array
 *
 * @return a sparse identity matrix with its shape specified by size
 *
 */
Matrix& speye(int* size);





// namespace matlab {

/**
 * Construct an m-by-n dense identity matrix.
 *
 * @param m number of rows
 *
 * @param n number of columns
 *
 * @return an m-by-n dense identity matrix
 *
 */
DenseMatrix& eye(int m, int n);

/**
 * Create a m-by-n Hilbert matrix. The elements of the
 * Hilbert matrices are H(i, j) = 1 / (i + j + 1), where
 * i and j are zero based indices.
 *
 * @param m number of rows
 *
 * @param n number of columns
 *
 * @return a m-by-n Hilbert matrix
 */
Matrix& hilb(int m, int n);

Vector& times(Vector& V1, Vector& V2);

Vector& plus(Vector& V1, Vector& V2);

Vector& minus(Vector& V1, Vector& V2);

Matrix& times(Matrix& A, Matrix& B);

Matrix& plus(Matrix& A, Matrix& B);

Matrix& minus(Matrix& A, Matrix& B);

DenseVector& full(Vector& V);

SparseVector& sparse(Vector& V);

DenseMatrix& full(Matrix& S);

SparseMatrix& sparse(Matrix& A);

Vector* sparseMatrix2SparseRowVectors0(Matrix& S);

Vector* sparseMatrix2SparseColumnVectors0(Matrix& S);

Matrix& sparseRowVectors2SparseMatrix0(Vector* Vs, int numRows);

Matrix& sparseColumnVectors2SparseMatrix0(Vector* Vs, int numColumns);

Vector** sparseMatrix2SparseRowVectors(Matrix& S);

Vector** sparseMatrix2SparseColumnVectors(Matrix& S);

Matrix& sparseRowVectors2SparseMatrix(Vector** Vs, int numRows);

Matrix& sparseColumnVectors2SparseMatrix(Vector** Vs, int numColumns);

Matrix& rowVector2RowMatrix(Vector& V);

Matrix& columnVector2ColumnMatrix(Vector& V);

Matrix& denseRowVectors2DenseMatrix(Vector** Vs, int numRows);

Matrix& denseColumnVectors2DenseMatrix(Vector** Vs, int numColumns);

Vector** denseMatrix2DenseRowVectors(Matrix& A);

Vector** denseMatrix2DenseColumnVectors(Matrix& A);


// }

#endif /* MATLAB_H_ */
