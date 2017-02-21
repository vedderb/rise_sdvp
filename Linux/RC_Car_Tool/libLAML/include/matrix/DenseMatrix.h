/*
 * DenseMatrix.h
 *
 *  Created on: Feb 4, 2014
 *      Author: Mingjie Qian
 */

#ifndef DENSEMATRIX_H_
#define DENSEMATRIX_H_

#include "Matrix.h"

class DenseMatrix : public Matrix {

private:
	int M;
	int N;
	double** data;

public:

	DenseMatrix();
	/**
	 * Constructor.
	 *
	 * @param M number of rows
	 * @param N number of columns
	 */
	DenseMatrix(int M, int N);
	DenseMatrix(int M, int N, double v);
	DenseMatrix(double data[], int len, int dim);
	DenseMatrix(int* size);
	DenseMatrix(double** data, int M, int N);
	// DenseMatrix(double data[][], int M, int N);
	/*template<int numColumns>
	DenseMatrix(double data[][numColumns], int M, int N) {
		this->data = new double*[M];
		for (int i = 0; i < M; i++) {
			this->data[i] = data[i];
		}
		this->M = M;
		this->N = N;
	}*/
	template<int numColumns>
	static Matrix& createDenseMatrix(double data[][numColumns], int M, int N) {
		DenseMatrix* res = new DenseMatrix();
		res->data = new double*[M];
		for (int i = 0; i < M; i++) {
			res->data[i] = data[i];
		}
		res->M = M;
		res->N = N;
		return *res;
	}

	template<int numColumns>
	static Matrix& createDenseMatrix(double data[][numColumns], int M) {
		DenseMatrix* res = new DenseMatrix();
		res->data = new double*[M];
		for (int i = 0; i < M; i++) {
			res->data[i] = data[i];
		}
		res->M = M;
		res->N = numColumns;
		return *res;
	}

	~DenseMatrix();

	/**
	 * Get number of rows.
	 *
	 * @return M
	 */
	int getRowDimension() {
		return M;
	}

	/**
	 * Get number of columns.
	 *
	 * @return N
	 */
	int getColumnDimension() {
		return N;
	}

	double getEntry(int r, int c);

	void setEntry(int r, int c, double v);

	double** getData() {
		return data;
	}

	Matrix* mtimes(Matrix* A);
	/**
	 * This * A.
	 *
	 * @param A a matrix
	 *
	 * @return this * A
	 */
	Matrix& mtimes(Matrix& A);

	/**
	 * This .* A.
	 *
	 * @param A
	 *
	 * @return this .* A
	 */
	Matrix& times(Matrix& A);

	/**
	 * This .* v.
	 *
	 * @param v a real scalar
	 *
	 * @return this * v
	 */
	Matrix& times(double v);

	/**
	 * This + A.
	 *
	 * @param A a real matrix
	 *
	 * @return this + A
	 */
	Matrix& plus(Matrix& A);

	/**
	 * This + v.
	 *
	 * @param v v a real scalar
	 *
	 * @return this + v
	 */
	Matrix& plus(double v);

	/**
	 * This - A.
	 *
	 * @param A a real matrix
	 *
	 * @return this - A
	 */
	Matrix& minus(Matrix& A);

	/**
	 * This - v.
	 *
	 * @param v a real scalar
	 *
	 * @return this - v
	 */
	Matrix& minus(double v);

	/**
	 * The transpose of this matrix.
	 *
	 * @return this<sup>T</sup>
	 */
	Matrix& transpose();

	/**
	 * Get a deep copy of this matrix.
	 *
	 * @return a copy of this matrix
	 */
	Matrix& copy();

	/**
	 * Matrix vector operation, i.e., res = This * b.
	 *
	 * @param b a dense or sparse vector
	 *
	 * @return This * b
	 */
	Vector& operate(Vector& b);

	/**
	 * Clear this matrix.
	 */
	void clear();

	/**
	 * Get a submatrix. Row index and column index start from 0.
	 *
	 * @param startRow Initial row index
	 *
	 * @param endRow Final row index (inclusive)
	 *
	 * @param startColumn Initial column index
	 *
	 * @param endColumn Final column index (inclusive)
	 *
	 * @return The subMatrix containing the data of the
	 *         specified rows and columns
	 */
	Matrix& getSubMatrix(int startRow, int endRow, int startColumn, int endColumn);

	/**
	 * Get a submatrix. Row index and column index start from 0.
	 *
	 * @param selectedRows Array of row indices
	 *
	 * @param numSelRows number of selected rows
	 *
	 * @param selectedColumns Array of column indices
	 *
	 * @param numSelColumns number of selected columns
	 *
	 * @return The subMatrix containing the data in the
	 *         specified rows and columns
	 */
	Matrix& getSubMatrix(int* selectedRows, int numSelRows, int* selectedColumns, int numSelColumns);

	/**
	 * Get rows from startRow to endRow as a matrix.
	 *
	 * @param startRow start index
	 *
	 * @param endRow end index (inclusive)
	 *
	 * @return a real matrix containing the specified rows
	 */
	Matrix& getRows(int startRow, int endRow);

	/**
	 * Get rows of specified row indices as a matrix.
	 *
	 * @param selectedRows a sequence of row indices
	 *
	 * @param numSelRows number of selected rows
	 *
	 * @return a real matrix containing the specified rows
	 */
	Matrix& getRows(int* selectedRows, int numSelRows);

	/**
	 * Get rows from startRow to endRow as a 1D {@code Vector} array.
	 *
	 * @param startRow start index
	 *
	 * @param endRow end index (inclusive)
	 *
	 * @return a 1D {@code Vector} array containing the specified rows
	 */
	Vector** getRowVectors(int startRow, int endRow);

	/**
	 * Get rows of specified row indices as a 1D {@code Vector} array.
	 *
	 * @param selectedRows a sequence of row indices
	 *
	 * @param numSelRows number of selected rows
	 *
	 * @return a 1D {@code Vector} array containing the specified rows
	 */
	Vector** getRowVectors(int* selectedRows, int numSelRows);

	/**
	 * Get a row matrix.
	 *
	 * @param r row index
	 *
	 * @return the r-th row matrix
	 */
	Matrix& getRowMatrix(int r);

	/**
	 * Set the r-th row by a row matrix A.
	 *
	 * @param r row index
	 *
	 * @param A a row matrix
	 */
	void setRowMatrix(int r, Matrix& A);

	/**
	 * Get a row vector.
	 *
	 * @param r row index
	 *
	 * @return the r-th row vector
	 */
	Vector& getRowVector(int r);

	/**
	 * Set the r-th row by a row vector V.
	 *
	 * @param r row index
	 *
	 * @param V a row vector
	 */
	void setRowVector(int r, Vector& V);

	/**
	 * Get columns from startColumn to endColumn as a matrix.
	 *
	 * @param startColumn start index
	 *
	 * @param endColumn end index (inclusive)
	 *
	 * @return a real matrix containing the specified columns
	 */
	Matrix& getColumns(int startColumn, int endColumn);

	/**
	 * Get columns of specified column indices as a matrix.
	 *
	 * @param selectedColumns a sequence of column indices
	 *
	 * @param numSelColumns number of selected columns
	 *
	 * @return a real matrix containing the specified columns
	 */
	Matrix& getColumns(int* selectedColumns, int numSelColumns);

	/**
	 * Get columns from startColumn to endColumn as a 1D {@code Vector} array.
	 *
	 * @param startColumn start index
	 *
	 * @param endColumn end index (inclusive)
	 *
	 * @return a 1D {@code Vector} array containing the specified columns
	 */
	Vector** getColumnVectors(int startColumn, int endColumn);

	/**
	 * Get columns of specified column indices as a 1D {@code Vector} array.
	 *
	 * @param selectedColumns a sequence of column indices
	 *
	 * @param numSelColumns number of selected columns
	 *
	 * @return a 1D {@code Vector} array containing the specified columns
	 */
	Vector** getColumnVectors(int* selectedColumns, int numSelColumns);

	/**
	 * Get a column matrix.
	 *
	 * @param c column index
	 *
	 * @return the c-th column matrix
	 */
	Matrix& getColumnMatrix(int c);

	/**
	 * Set the c-th column by a column matrix A.
	 *
	 * @param c column index
	 *
	 * @param A a column matrix
	 */
	void setColumnMatrix(int c, Matrix& A);

	/**
	 * Get a column vector.
	 *
	 * @param c column index
	 *
	 * @return the c-th column vector
	 */
	Vector& getColumnVector(int c);

	/**
	 * Set the c-th column by a column vector V.
	 *
	 * @param c column index
	 *
	 * @param V a column vector
	 */
	void setColumnVector(int c, Vector& V);

};


#endif /* DENSEMATRIX_H_ */
