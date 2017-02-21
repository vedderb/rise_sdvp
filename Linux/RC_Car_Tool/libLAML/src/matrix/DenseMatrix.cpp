/*
 * DenseMatrix.cpp
 *
 *  Created on: Feb 4, 2014
 *      Author: Mingjie Qian
 */

#include "Matrix.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Vector.h"
#include "DenseVector.h"
#include "SparseVector.h"
#include "Printer.h"
#include <typeinfo>
#include <cstdlib>
#include <cstring>
#include <iostream>

DenseMatrix::DenseMatrix() {
	data = NULL;
	M = 0;
	N = 0;
}

DenseMatrix::DenseMatrix(int M, int N) : Matrix() {
	data = new double*[M];
	for (int i = 0; i < M; i++) {
		data[i] = new double[N];
	}
	this->M = M;
	this->N = N;
}

DenseMatrix::DenseMatrix(int M, int N, double v) : Matrix() {
	data = new double*[M];
	for (int i = 0; i < M; i++) {
		data[i] = new double[N];
		for (int j = 0; j < N; j++)
			data[i][j] = v;
	}
	this->M = M;
	this->N = N;
}

DenseMatrix::DenseMatrix(double data[], int len, int dim) {
	if (dim == 2) {
		M = 1;
		N = len;
		this->data = new double*[M];
		this->data[0] = data;
	} else if (dim == 1) {
		M = len;
		N = 1;
		this->data = new double*[M];
		for (int i = 0; i < M; i++) {
			this->data[i] = new double[N];
			this->data[i][0] = data[i];
		}
	} else {
		err("dim should be either 1 or 2.");
		exit(1);
	}
}

DenseMatrix::DenseMatrix(int* size) : Matrix() {
	/*if (sizeof(size) / sizeof(int) != 2) {
		std::cerr << "The input integer array should have exactly two entries!" << std::endl;
		exit(1);
	}*/
	int M = size[0];
	int N = size[1];
	data = new double*[M];
	for (int i = 0; i < M; i++) {
		data[i] = new double[N];
		for (int j = 0; j < N; j++) {
			data[i][j] = 0.0;
		}
	}
	this->M = M;
	this->N = N;
}

DenseMatrix::DenseMatrix(double** data, int M, int N) : Matrix() {
	this->data = data;
	this->M = M;
	this->N = N;
}

/*DenseMatrix::DenseMatrix(double data[][], int M, int N) {
	this->data = data;
	this->M = M;
	this->N = N;
}*/

DenseMatrix::~DenseMatrix() {
	// ~Matrix();
	for (int i = 0; i < M; i++) {
		delete[] data[i];
		data[i] = null;
	}
	delete[] data;
	data = null;
	// cout << "DenseMatrix destructor was called.";
	// disp("This dense matrix is released.");
}

Matrix& DenseMatrix::transpose() {
	double** resData = new double*[N];
	for (int i = 0; i < N; i++) {
		resData[i] = new double[M];
		for (int j = 0; j < M; j++) {
			resData[i][j] = data[j][i];
		}
	}
	return *new DenseMatrix(resData, N, M);
}

Matrix& DenseMatrix::mtimes(Matrix& B) {
	int MB = B.getRowDimension();
	int NB = B.getColumnDimension();
	if (MB != N) {
		exit(1);
	}
	if (typeid(B) == typeid(DenseMatrix)) {
		Matrix* res = new DenseMatrix(M, NB);
		double** resPr = ((DenseMatrix*) res)->data;
		// double** BPr = dynamic_cast<DenseMatrix&>(B).pr;
		double** BPr = ((DenseMatrix&) B).data;
		double* columnB = new double[MB];
		for (int j = 0; j < NB; j++) {
			for (int i = 0; i < MB; i++) {
				columnB[i] = BPr[i][j];
			}
			for (int i = 0; i < M; i++) {
				double* ARowPr = data[i];
				double s = 0;
				for (int k = 0; k < N; k++) {
					s += ARowPr[k] * columnB[k];
				}
				resPr[i][j] = s;
			}
		}
		return *res;
	} else if (typeid(B) == typeid(SparseMatrix)) {
		Matrix* res = new DenseMatrix(M, NB);
		double** resData = ((DenseMatrix*) res)->data;
		double* rowData = null;
		int* ir = null;
		int* jc = null;
		double* pr = null;
		ir = ((SparseMatrix&) B).getIr();
		jc = ((SparseMatrix&) B).getJc();
		pr = ((SparseMatrix&) B).getPr();
		int r = -1;
		double s = 0;

		for (int i = 0; i < M; i++) {
			rowData = data[i];
			for (int j = 0; j < NB; j++) {
				s = 0;
				for (int k = jc[j]; k < jc[j + 1]; k++) {
					r = ir[k];
					// A[r][j] = pr[k]
					s += rowData[r] * pr[k];
				}
				resData[i][j] = s;
			}
		}
		return *res;
	} else {
		Matrix* res = new DenseMatrix(0, 0);
		return *res;
	}

}

Matrix* DenseMatrix::mtimes(Matrix* B) {
	Matrix* res = NULL;
	int MB = B->getRowDimension();
	int NB = B->getColumnDimension();
	if (MB != N) {
		exit(1);
	}
	if (typeid(*B) == typeid(DenseMatrix)) {
		res = new DenseMatrix(M, NB);
		double** resPr = ((DenseMatrix*) res)->data;
		double** BPr = ((DenseMatrix*) B)->data;
		double* columnB = new double[MB];
		for (int j = 0; j < NB; j++) {
			for (int i = 0; i < MB; i++) {
				columnB[i] = BPr[i][j];
			}
			for (int i = 0; i < M; i++) {
				double* ARowPr = data[i];
				double s = 0;
				for (int k = 0; k < N; k++) {
					s += ARowPr[k] * columnB[k];
				}
				resPr[i][j] = s;
			}
		}
	}
	return res;
}

double DenseMatrix::getEntry(int r, int c) {
	return data[r][c];
}

void DenseMatrix::setEntry(int r, int c, double v) {
	data[r][c] = v;
}

Matrix& DenseMatrix::plus(Matrix& A) {

	if (A.getRowDimension() != M || A.getColumnDimension() != N) {
		err("Dimension doesn't match.");
		exit(1);
	}

	DenseMatrix& res = (DenseMatrix&) this->copy();

	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;
		double* row = null;
		for (int i = 0; i < M; i++) {
			row = res.data[i];
			ARow = AData[i];
			for (int j = 0; j < N; j++) {
				row[j] += ARow[j];
			}
		}
	} else {
		int* ir = null;
		int* jc = null;
		double* pr = null;
		ir = ((SparseMatrix&) A).getIr();
		jc = ((SparseMatrix&) A).getJc();
		pr = ((SparseMatrix&) A).getPr();
		int r = -1;

		for (int j = 0; j < A.getColumnDimension(); j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				r = ir[k];
				// A[r][j] = pr[k]
				res.data[r][j] += pr[k];
			}
		}
	}

	return res;

}

Matrix& DenseMatrix::minus(Matrix& A) {

	if (A.getRowDimension() != M || A.getColumnDimension() != N) {
		err("Dimension doesn't match.");
		exit(1);
	}

	DenseMatrix& res = (DenseMatrix&) this->copy();

	if (typeid(A) == typeid(DenseMatrix)) {

		for (int i = 0; i < M; i++) {
			for (int j = 0; j < N; j++) {
				res.data[i][j] -= ((DenseMatrix&) A).data[i][j];
			}
		}

	} else {
		int* ir = null;
		int* jc = null;
		double* pr = null;
		ir = ((SparseMatrix&) A).getIr();
		jc = ((SparseMatrix&) A).getJc();
		pr = ((SparseMatrix&) A).getPr();
		int r = -1;

		for (int j = 0; j < A.getColumnDimension(); j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				r = ir[k];
				// A[r][j] = pr[k]
				res.data[r][j] -= pr[k];
			}
		}
	}

	return res;

}

Matrix& DenseMatrix::times(Matrix& A) {

	if (A.getRowDimension() != M || A.getColumnDimension() != N) {
		err("Dimension doesn't match.");
		exit(1);
	}

	// double** resData = ((DenseMatrix&) this.copy()).getData();
	DenseMatrix* res = new DenseMatrix(M, N, 0);
	double** resData = res->data;

	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;
		double* thisRow = null;
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			thisRow = data[i];
			ARow = AData[i];
			resRow = resData[i];
			thisRow = data[i];
			for (int j = 0; j < N; j++) {
				resRow[j] = thisRow[j] * ARow[j];
			}
		}
	} else {
		int* ir = null;
		int* jc = null;
		double* pr = null;
		ir = ((SparseMatrix&) A).getIr();
		jc = ((SparseMatrix&) A).getJc();
		pr = ((SparseMatrix&) A).getPr();
		int r = -1;

		for (int j = 0; j < A.getColumnDimension(); j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				r = ir[k];
				// A[r][j] = pr[k]
				resData[r][j] = data[r][j] * pr[k];
			}
		}
	}

	return *res;

}

Matrix& DenseMatrix::times(double v) {
	DenseMatrix& res = (DenseMatrix&) this->copy();
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			res.data[i][j] *= v;
		}
	}
	return res;
}

Matrix& DenseMatrix::copy() {
	DenseMatrix& res = *new DenseMatrix();
	res.M = M;
	res.N = N;
	res.data = new double*[M];
	for (int i = 0; i < M; i++) {
		res.data[i] = clone(data[i], N);
	}
	return res;
}

Matrix& DenseMatrix::plus(double v) {
	DenseMatrix& res = (DenseMatrix&) this->copy();
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			res.data[i][j] += v;
		}
	}
	return res;
}

Matrix& DenseMatrix::minus(double v) {
	DenseMatrix& res = (DenseMatrix&) this->copy();
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			res.data[i][j] -= v;
		}
	}
	return res;
}

Vector& DenseMatrix::operate(Vector& b) {

	if (N != b.getDim()) {
		err("Dimension does not match.");
		exit(1);
	}
	double* V = new double[M];

	if (typeid(b) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) b).getPr();
		double* thisRow = null;
		for (int i = 0; i < M; i++) {
			thisRow = data[i];
			double s = 0;
			for (int j = 0; j < N; j++) {
				s += thisRow[j] * pr[j];
			}
			V[i] = s;
		}
	} else {
		int* ir = ((SparseVector&) b).getIr();
		double* pr = ((SparseVector&) b).getPr();
		int nnz = ((SparseVector&) b).getNNZ();
		int idx = 0;
		double* row_i = null;
		for (int i = 0; i < M; i++) {
			row_i = data[i];
			double s = 0;
			for (int k = 0; k < nnz; k++) {
				idx = ir[k];
				s += row_i[idx] * pr[k];
			}
			V[i] = s;
		}
	}

	return *new DenseVector(V, M);

}

void DenseMatrix::clear() {
	for (int i = 0; i < M; i++) {
		double* thisRow = data[i];
		for (int j = 0; j < N; j++) {
			thisRow[j] = 0;
		}
	}
}

Matrix& DenseMatrix::getSubMatrix(int startRow, int endRow, int startColumn, int endColumn) {
	int nRow = endRow - startRow + 1;
	int nCol = endColumn - startColumn + 1;
	double** resData = new double*[nRow];
	double* resRow = null;
	double* thisRow = null;
	for (int r = 0, i = startRow; r < nRow; r++, i++) {
		resRow = new double[nCol];
		thisRow = data[i];
		// System.arraycopy(thisRow, startColumn, resRow, 0, nCol);
		std::copy(thisRow + startColumn, thisRow + startColumn + nCol, resRow);
		resData[r] = resRow;
	}
	return *new DenseMatrix(resData, nRow, nCol);
}

Matrix& DenseMatrix::getSubMatrix(int* selectedRows, int numSelRows, int* selectedColumns, int numSelColumns) {
	int nRow = numSelRows;
	int nCol = numSelColumns;
	double** resData = new double*[nRow];
	double* resRow = null;
	double* thisRow = null;
	for (int r = 0; r < nRow; r++) {
		resRow = new double[nCol];
		thisRow = data[selectedRows[r]];
		for (int c = 0; c < nCol; c++) {
			resRow[c] = thisRow[selectedColumns[c]];
		}
		resData[r] = resRow;
	}
	return *new DenseMatrix(resData, nRow, nCol);
}

Matrix& DenseMatrix::getRows(int startRow, int endRow) {
	int numRows = endRow - startRow + 1;
	double** resData = new double*[numRows];
	for (int r = startRow, i = 0; r <= endRow; r++, i++) {
		resData[i] = clone(data[r], N);
	}
	return *new DenseMatrix(resData, numRows, N);
}

Matrix& DenseMatrix::getRows(int* selectedRows, int numSelRows) {
	int numRows = numSelRows;
	double** resData = new double*[numRows];
	for (int i = 0; i < numRows; i++) {
		resData[i] = clone(data[selectedRows[i]], N);
	}
	return *new DenseMatrix(resData, numRows, N);
}

Vector** DenseMatrix::getRowVectors(int startRow, int endRow) {
	int numRows = endRow - startRow + 1;
	Vector** res = new Vector*[numRows];
	for (int r = startRow, i = 0; r <= endRow; r++, i++) {
		res[i] = new DenseVector(data[r], N);
	}
	return res;
}

Vector** DenseMatrix::getRowVectors(int* selectedRows, int numSelRows) {
	int numRows = numSelRows;
	Vector** res = new Vector*[numRows];
	for (int i = 0; i < numRows; i++) {
		res[i] = new DenseVector(data[selectedRows[i]], N);
	}
	return res;
}

Matrix& DenseMatrix::getRowMatrix(int r) {
	return *new DenseMatrix(data[r], N, 2);
}

void DenseMatrix::setRowMatrix(int r, Matrix& A) {
	if (A.getRowDimension() != 1) {
		err("Input matrix should be a row matrix.");
		exit(1);
	}
	double* thisRow = data[r];
	if (typeid(A) == typeid(DenseMatrix)) {
		double* ARow = ((DenseMatrix&) A).data[0];
		// System.arraycopy(ARow, 0, thisRow, 0, N);
		std::copy(ARow, ARow + N, thisRow);
	} else {
		int* jc = ((SparseMatrix&) A).getJc();
		double* pr = ((SparseMatrix&) A).getPr();
		for (int j = 0; j < N; j++) {
			if (jc[j + 1] == jc[j]) {
				thisRow[j] = 0;
			} else {
				thisRow[j] = pr[jc[j]];
			}
		}
	}
}

Vector& DenseMatrix::getRowVector(int r) {
	return *new DenseVector(data[r], N);
}

void DenseMatrix::setRowVector(int r, Vector& V) {
	double* thisRow = data[r];
	if (typeid(V) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) V).getPr();
		// System.arraycopy(pr, 0, thisRow, 0, N);
		std::copy(pr, pr + N, thisRow);
	} else {
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz = ((SparseVector&) V).getNNZ();
		int lastIdx = -1;
		int currentIdx = 0;
		for (int k = 0; k < nnz; k++) {
			currentIdx = ir[k];
			for (int j = lastIdx + 1; j < currentIdx; j++) {
				thisRow[j] = 0;
			}
			thisRow[currentIdx] = pr[k];
			lastIdx = currentIdx;
		}
		for (int j = lastIdx + 1; j < N; j++) {
			thisRow[j] = 0;
		}
	}
}

Matrix& DenseMatrix::getColumns(int startColumn, int endColumn) {
	int nRow = M;
	int nCol = endColumn - startColumn + 1;
	double** resData = new double*[nRow];
	double* resRow = null;
	double* thisRow = null;
	for (int r = 0; r < nRow; r++) {
		resRow = new double[nCol];
		thisRow = data[r];
		// System.arraycopy(thisRow, startColumn, resRow, 0, nCol);
		std::copy(thisRow + startColumn, thisRow + startColumn + nCol, resRow);
		resData[r] = resRow;
	}
	return *new DenseMatrix(resData, nRow, nCol);
}

Matrix& DenseMatrix::getColumns(int* selectedColumns, int numSelColumns) {
	int nRow = M;
	int nCol = numSelColumns;
	double** resData = new double*[nRow];
	double* resRow = null;
	double* thisRow = null;
	for (int r = 0; r < nRow; r++) {
		resRow = new double[nCol];
		thisRow = data[r];
		for (int c = 0; c < nCol; c++) {
			resRow[c] = thisRow[selectedColumns[c]];
		}
		resData[r] = resRow;
	}
	return *new DenseMatrix(resData, nRow, nCol);
}

/**
 * Get columns from startColumn to endColumn as a 1D {@code Vector} array.
 *
 * @param startColumn start index
 *
 * @param endColumn end index (inclusive)
 *
 * @return a 1D {@code Vector} array containing the specified columns
 */
Vector** DenseMatrix::getColumnVectors(int startColumn, int endColumn) {
	int numColumns = endColumn - startColumn + 1;
	Vector** res = new Vector*[numColumns];
	for (int c = startColumn, i = 0; c <= endColumn; c++, i++) {
		res[i] = &getColumnVector(c);
	}
	return res;
}

/**
 * Get columns of specified column indices as a 1D {@code Vector} array.
 *
 * @param selectedColumns a sequence of column indices
 *
 * @param numSelColumns number of selected columns
 *
 * @return a 1D {@code Vector} array containing the specified columns
 */
Vector** DenseMatrix::getColumnVectors(int* selectedColumns, int numSelColumns) {
	int numColumns = numSelColumns;
	Vector** res = new Vector*[numColumns];
	for (int j = 0; j < numColumns; j++) {
		res[j] = &getColumnVector(selectedColumns[j]);
	}
	return res;
}

/**
 * Get a column matrix.
 *
 * @param c column index
 *
 * @return the c-th column matrix
 */
Matrix& DenseMatrix::getColumnMatrix(int c) {
	DenseMatrix& res = *new DenseMatrix(M, 1);
	double** resData = res.data;
	for (int i = 0; i < M; i++) {
		resData[i][0] = data[i][c];
	}
	return res;
}

/**
 * Set the c-th column by a column matrix A.
 *
 * @param c column index
 *
 * @param A a column matrix
 */
void DenseMatrix::setColumnMatrix(int c, Matrix& A) {
	if (A.getColumnDimension() != 1) {
		err("Input matrix should be a column matrix.");
		exit(1);
	}
	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).data;
		for (int i = 0; i < M; i++) {
			data[i][c] = AData[i][0];
		}
	} else {
		int* jc = ((SparseMatrix&) A).getJc();
		if (jc[1] == 0) {
			for (int i = 0; i < M; i++) {
				data[i][c] = 0;
			}
			return;
		}
		int* ir = ((SparseMatrix&) A).getIr();
		double* pr = ((SparseMatrix&) A).getPr();
		int lastIdx = -1;
		int currentIdx = 0;
		for (int k = 0; k < jc[1]; k++) {
			currentIdx = ir[k];
			for (int i = lastIdx + 1; i < currentIdx; i++) {
				data[i][c] = 0;
			}
			data[currentIdx][c] = pr[k];
			lastIdx = currentIdx;
		}
		for (int i = lastIdx + 1; i < M; i++) {
			data[i][c] = 0;
		}
	}
}

/**
 * Get a column vector.
 *
 * @param c column index
 *
 * @return the c-th column vector
 */
Vector& DenseMatrix::getColumnVector(int c) {
	DenseVector& res = *new DenseVector(M);
	double* pr = res.getPr();
	for (int i = 0; i < M; i++) {
		pr[i] = data[i][c];
	}
	return res;
}

/**
 * Set the c-th column by a column vector V.
 *
 * @param c column index
 *
 * @param V a column vector
 */
void DenseMatrix::setColumnVector(int c, Vector& V) {
	if (typeid(V) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) V).getPr();
		for (int i = 0; i < M; i++) {
			data[i][c] = pr[i];
		}
	} else {
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz = ((SparseVector&) V).getNNZ();
		int lastIdx = -1;
		int currentIdx = 0;
		for (int k = 0; k < nnz; k++) {
			currentIdx = ir[k];
			for (int i = lastIdx + 1; i < currentIdx; i++) {
				data[i][c] = 0;
			}
			data[currentIdx][c] = pr[k];
			lastIdx = currentIdx;
		}
		for (int i = lastIdx + 1; i < M; i++) {
			data[i][c] = 0;
		}
	}
}
