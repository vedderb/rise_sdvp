/*
 * SparseMatrix.h
 *
 *  Created on: Feb 4, 2014
 *      Author: Mingjie Qian
 */

#ifndef SPARSEMATRIX_H_
#define SPARSEMATRIX_H_

#include "Matrix.h"
#include "DenseMatrix.h"
#include "Utility.h"
#include "Vector.h"
#include <map>
#include <cstring>
#include <typeinfo>
#include <list>

class SparseMatrix : public Matrix {

private:
	int M;
	int N;
	int* ir;
	int* jc;
	int* ic;
	int* jr;
	int* valCSRIndices;
	double* pr;
	int nnz;
	int nzmax;

public:

	SparseMatrix() {
		M = 0;
		N = 0;
		nzmax = 0;
		nnz = 0;
		ir = NULL;
		jc = NULL;
		ic = NULL;
		jr = NULL;
		valCSRIndices = NULL;
		pr = NULL;
	};

	/**
	 * Constructor for an M x N sparse matrix.
	 *
	 * @param M number of rows
	 *
	 * @param N number of columns
	 */
	SparseMatrix(int M, int N);

	SparseMatrix(SparseMatrix& A);

	SparseMatrix(int* rIndices, int* cIndices, double* values, int numRows, int numColumns, int nzmax);

	SparseMatrix(std::list<int>& rIndexList, std::list<int>& cIndexList, std::list<double>& valueList, int numRows, int numColumns, int nzmax);

	~SparseMatrix();

	/**
	 * Assign this sparse matrix by a sparse matrix A in the sense that
	 * all interior arrays of this matrix are deep copy of the given
	 * sparse matrix A.
	 *
	 * @param A a sparse Matrix
	 */
	void assignSparseMatrix(SparseMatrix A);

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

	int* getIr() {
		return ir;
	}

	int* getJc() {
		return jc;
	}

	int* getIc() {
		return ic;
	}

	int* getJr() {
		return jr;
	}

	double* getPr() {
		return pr;
	}

	int* getValCSRIndices() {
		return valCSRIndices;
	}

	int getNZMax() {
		return nzmax;
	}

	int getNNZ() {
		return nnz;
	}

	double getEntry(int r, int c);

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
	void setEntry(int r, int c, double v);

	Matrix* mtimes(Matrix* A);

	Matrix& mtimes(Matrix& A);

	/**
	 * Create a sparse matrix with the compressed sparse column format.
	 *
	 * @param ir a 1D {@code int} array of row indices for
	 *           non-zero elements
	 *
	 * @param jc a 1D {@code int} array of number of non-zero
	 *           elements in each column
	 *
	 * @param pr a 1D {@code double} array of non-zero elements
	 *
	 * @param M number of rows
	 *
	 * @param N number of columns
	 *
	 * @param nzmax maximal number of non-zero entries
	 *
	 * @return a sparse matrix
	 */
	static SparseMatrix& createSparseMatrixByCSCArrays(int* ir, int* jc, double* pr, int M, int N, int nzmax);

	/**
	 * Create a sparse matrix with the compressed sparse column format
	 * using compressed sparse row information.
	 *
	 * @param ic a 1D {@code int} array of column indices for
	 *           non-zero elements
	 *
	 * @param jr a 1D {@code int} array of number of non-zero
	 *           elements in each row
	 *
	 * @param pr a 1D {@code double} array of non-zero elements
	 *
	 * @param M number of rows
	 *
	 * @param N number of columns
	 *
	 * @param nzmax maximal number of non-zero entries
	 *
	 * @return a sparse matrix
	 */
	static SparseMatrix& createSparseMatrixByCSRArrays(int* ic, int* jr, double* pr, int M, int N, int nzmax);

	/**
	 * Create a sparse matrix from index and value arrays.
	 *
	 * @param rIndices a 1D {@code int} array of row indices for
	 *           	   non-zero elements
	 *
	 * @param cIndices a 1D {@code int} array of column indices for
	 *                 non-zero elements
	 *
	 * @param values a 1D {@code double} array of non-zero elements
	 *
	 * @param numRows number of rows
	 *
	 * @param numColumns number of columns
	 *
	 * @param nzmax maximal number of non-zero elements
	 *
	 * @return a sparse matrix
	 */
	static SparseMatrix& createSparseMatrix(int* rIndices, int* cIndices, double* values, int numRows, int numColumns, int nzmax);

	/**
	 * Create a sparse matrix from a map from index pairs to values.
	 *
	 * @param inputMap a mapping from row-column pair to value
	 *
	 * @param numRows number of rows
	 *
	 * @param numColumns number of columns
	 *
	 * @return a sparse matrix
	 */
	static SparseMatrix& createSparseMatrix(std::map<std::pair<int, int>, double> inputMap, int numRows, int numColumns);

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
	 * Clean entries so that zero entries are removed.
	 */
	void clean();

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

private:

	/**
	 * Insert a new entry (r, c) = v and its insertion position is pos in pr.
	 * @param r
	 * @param c
	 * @param v
	 * @param pos insertion position in pr, i.e., the new entry's
	 *            position in pr
	 */
	void insertEntry(int r, int c, double v, int pos);

	/**
	 * Delete the entry indexed by (r, c) whose index of pr is pos.
	 *
	 * @param r row index of the entry to be deleted
	 *
	 * @param c column index of the entry to be deleted
	 *
	 * @param pos index in pr of the (r, c) entry
	 */
	void deleteEntry(int r, int c, int pos);

};


#endif /* SPARSEMATRIX_H_ */
