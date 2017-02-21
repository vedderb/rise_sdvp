/*
 * Matrix.h
 *
 *  Created on: Feb 4, 2014
 *      Author: Mingjie Qian
 */

#ifndef MATRIX_H_
#define MATRIX_H_

class Vector;

class Matrix {

public:
	Matrix() {};

	/* A base class destructor should be either and virtual,
	 * or protected and nonvirtual.
	 */
public:
	// ~Matrix() {cout << "Matrix destructor was called";};
	// If the curly braces are removed, then the vtable will be problematic.
	virtual ~Matrix() {};

public:
	/**
	 * Get number of rows.
	 *
	 * @return M
	 */
	virtual int getRowDimension() = 0;

	/**
	 * Get number of columns.
	 *
	 * @return N
	 */
	virtual int getColumnDimension() = 0;

	/**
	 * Get the (r, c) entry.
	 *
	 * @param r row index
	 *
	 * @param c column index
	 *
	 * @return this(r, c)
	 */
	virtual double getEntry(int r, int c) = 0;

	/**
	 * Note that it will be slow for sparse matrices,
	 * even if bisection search is used.
	 *
	 * @param r row index
	 *
	 * @param c column index
	 *
	 * @param v value to set
	 */
	virtual void setEntry(int r, int c, double v) = 0;

	virtual Matrix* mtimes(Matrix* A) = 0;

	/**
	 * This * A.
	 *
	 * @param A a matrix
	 *
	 * @return this * A
	 */
	virtual Matrix& mtimes(Matrix& A) = 0;

	/**
	 * This .* A.
	 *
	 * @param A
	 *
	 * @return this .* A
	 */
	virtual Matrix& times(Matrix& A) = 0;

	/**
	 * This .* v.
	 *
	 * @param v a real scalar
	 *
	 * @return this * v
	 */
	virtual Matrix& times(double v) = 0;

	/**
	 * This + A.
	 *
	 * @param A a real matrix
	 *
	 * @return this + A
	 */
	virtual Matrix& plus(Matrix& A) = 0;

	/**
	 * This + v.
	 *
	 * @param v v a real scalar
	 *
	 * @return this + v
	 */
	virtual Matrix& plus(double v) = 0;

	/**
	 * This - A.
	 *
	 * @param A a real matrix
	 *
	 * @return this - A
	 */
	virtual Matrix& minus(Matrix& A) = 0;

	/**
	 * This - v.
	 *
	 * @param v a real scalar
	 *
	 * @return this - v
	 */
	virtual Matrix& minus(double v) = 0;

	/**
	 * The transpose of this matrix.
	 *
	 * @return this<sup>T</sup>
	 */
	virtual Matrix& transpose() = 0;

	/**
	 * Get a deep copy of this matrix.
	 *
	 * @return a copy of this matrix
	 */
	virtual Matrix& copy() = 0;

	/**
	 * Matrix vector operation, i.e., res = This * b.
	 *
	 * @param b a dense or sparse vector
	 *
	 * @return This * b
	 */
	virtual Vector& operate(Vector& b) = 0;

	/**
	 * Clear this matrix.
	 */
	virtual void clear() = 0;

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
	virtual Matrix& getSubMatrix(int startRow, int endRow, int startColumn, int endColumn) = 0;

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
	virtual Matrix& getSubMatrix(int* selectedRows, int numSelRows, int* selectedColumns, int numSelColumns) = 0;

	/**
	 * Get rows from startRow to endRow as a matrix.
	 *
	 * @param startRow start index
	 *
	 * @param endRow end index (inclusive)
	 *
	 * @return a real matrix containing the specified rows
	 */
	virtual Matrix& getRows(int startRow, int endRow) = 0;

	/**
	 * Get rows of specified row indices as a matrix.
	 *
	 * @param selectedRows a sequence of row indices
	 *
	 * @param numSelRows number of selected rows
	 *
	 * @return a real matrix containing the specified rows
	 */
	virtual Matrix& getRows(int* selectedRows, int numSelRows) = 0;

	/**
	 * Get rows from startRow to endRow as a 1D {@code Vector} array.
	 *
	 * @param startRow start index
	 *
	 * @param endRow end index (inclusive)
	 *
	 * @return a 1D {@code Vector} array containing the specified rows
	 */
	virtual Vector** getRowVectors(int startRow, int endRow) = 0;

	/**
	 * Get rows of specified row indices as a 1D {@code Vector} array.
	 *
	 * @param selectedRows a sequence of row indices
	 *
	 * @param numSelRows number of selected rows
	 *
	 * @return a 1D {@code Vector} array containing the specified rows
	 */
	virtual Vector** getRowVectors(int* selectedRows, int numSelRows) = 0;

	/**
	 * Get a row matrix.
	 *
	 * @param r row index
	 *
	 * @return the r-th row matrix
	 */
	virtual Matrix& getRowMatrix(int r) = 0;

	/**
	 * Set the r-th row by a row matrix A.
	 *
	 * @param r row index
	 *
	 * @param A a row matrix
	 */
	virtual void setRowMatrix(int r, Matrix& A) = 0;

	/**
	 * Get a row vector.
	 *
	 * @param r row index
	 *
	 * @return the r-th row vector
	 */
	virtual Vector& getRowVector(int r) = 0;

	/**
	 * Set the r-th row by a row vector V.
	 *
	 * @param r row index
	 *
	 * @param V a row vector
	 */
	virtual void setRowVector(int r, Vector& V) = 0;

	/**
	 * Get columns from startColumn to endColumn as a matrix.
	 *
	 * @param startColumn start index
	 *
	 * @param endColumn end index (inclusive)
	 *
	 * @return a real matrix containing the specified columns
	 */
	virtual Matrix& getColumns(int startColumn, int endColumn) = 0;

	/**
	 * Get columns of specified column indices as a matrix.
	 *
	 * @param selectedColumns a sequence of column indices
	 *
	 * @param numSelColumns number of selected columns
	 *
	 * @return a real matrix containing the specified columns
	 */
	virtual Matrix& getColumns(int* selectedColumns, int numSelColumns) = 0;

	/**
	 * Get columns from startColumn to endColumn as a 1D {@code Vector} array.
	 *
	 * @param startColumn start index
	 *
	 * @param endColumn end index (inclusive)
	 *
	 * @return a 1D {@code Vector} array containing the specified columns
	 */
	virtual Vector** getColumnVectors(int startColumn, int endColumn) = 0;

	/**
	 * Get columns of specified column indices as a 1D {@code Vector} array.
	 *
	 * @param selectedColumns a sequence of column indices
	 *
	 * @param numSelColumns number of selected columns
	 *
	 * @return a 1D {@code Vector} array containing the specified columns
	 */
	virtual Vector** getColumnVectors(int* selectedColumns, int numSelColumns) = 0;

	/**
	 * Get a column matrix.
	 *
	 * @param c column index
	 *
	 * @return the c-th column matrix
	 */
	virtual Matrix& getColumnMatrix(int c) = 0;

	/**
	 * Set the c-th column by a column matrix A.
	 *
	 * @param c column index
	 *
	 * @param A a column matrix
	 */
	virtual void setColumnMatrix(int c, Matrix& A) = 0;

	/**
	 * Get a column vector.
	 *
	 * @param c column index
	 *
	 * @return the c-th column vector
	 */
	virtual Vector& getColumnVector(int c) = 0;

	/**
	 * Set the c-th column by a column vector V.
	 *
	 * @param c column index
	 *
	 * @param V a column vector
	 */
	virtual void setColumnVector(int c, Vector& V) = 0;

};


#endif /* MATRIX_H_ */
