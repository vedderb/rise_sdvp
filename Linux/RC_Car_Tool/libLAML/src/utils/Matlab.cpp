/*
 * Matlab.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: Mingjie Qian
 */

#include "Matlab.h"
#include "Printer.h"
#include "Matrix.h"
#include "DenseMatrix.h"
#include "ArrayOperator.h"
#include "SVD.h"
#include "EVD.h"
#include "Utility.h"
#include <map>
#include <set>
#include <cmath>
//#include <random>
#include <ctime>
#include <algorithm>
#include <vector>
#include "LU.h"
#include "InPlaceOperator.h"
#include "QR.h"
#include <cstdarg>

void setSubMatrix(Matrix& A, int* selectedRows, int numSelRows,
		int* selectedColumns, int numSelColumns, Matrix& B) {
	int r, c;
	for (int i = 0; i < numSelRows; i++) {
		for (int j = 0; j < numSelColumns; j++) {
			r = selectedRows[i];
			c = selectedColumns[j];
			A.setEntry(r, c, B.getEntry(i, j));
		}
	}
}

Matrix& reshape(Matrix& A, int M, int N) {
	if (M * N != A.getRowDimension() * A.getColumnDimension()) {
		err("Wrong shape!");
		exit(1);
	}

	Matrix* res = null;

	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double** resData = new double*[M];
		double* resRow = null;
		int r, c;
		for (int i = 0, shiftI = 0; i < M; i++, shiftI++) {
			resData[i] = new double[N];
			resRow = resData[i];
			for (int j = 0, shiftJ = shiftI; j < N; j++, shiftJ += M) {
				r = shiftJ % A.getRowDimension();
				c = shiftJ / A.getRowDimension();
				resRow[j] = AData[r][c];
			}
		}
		res = new DenseMatrix(resData, M, N);
	} else {
		int* ir = ((SparseMatrix&) A).getIr();
		int* jc = ((SparseMatrix&) A).getJc();
		double* pr = ((SparseMatrix&) A).getPr();
		int nnz = ((SparseMatrix&) A).getNNZ();
		int* resIr = new int[nnz];
		int* resJc = new int[N + 1];
		double* resPr = new double[nnz];
		std::copy(pr, pr + nnz, resPr);
		// System.arraycopy(pr, 0, resPr, 0, nnz);
		int lastColIdx = -1;
		int currentColIdx = 0;
		int idx = 0;
		for (int j = 0, shiftJ = 0; j < A.getColumnDimension(); j++, shiftJ += A.getRowDimension()) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				idx = ir[k] + shiftJ;
				currentColIdx = idx / M;
				resIr[k] = idx % M;
				while (lastColIdx < currentColIdx) {
					resJc[lastColIdx + 1] = k;
					lastColIdx++;
				}
			}
		}
		resJc[N] = nnz;
		res = &SparseMatrix::createSparseMatrixByCSCArrays(resIr, resJc, resPr, M, N, nnz);
	}

	return *res;

}

Matrix& reshape(Matrix& A, int* size) {
	return reshape(A, size[0], size[1]);
}

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
Matrix& reshape(Vector& V, int M, int N) {

	int dim = V.getDim();

	if (M * N != dim) {
		err("Wrong shape!");
	}

	Matrix* res = null;

	if (typeid(V) == typeid(DenseVector)) {
		double** resData = new double*[M];
		double* resRow = null;
		double* pr = ((DenseVector&) V).getPr();
		for (int i = 0, shiftI = 0; i < M; i++, shiftI++) {
			resData[i] = new double[N];
			resRow = resData[i];
			for (int j = 0, shiftJ = shiftI; j < N; j++, shiftJ += M) {
				resRow[j] = pr[shiftJ];
			}
		}
		res = new DenseMatrix(resData, M, N);
	} else {
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz = ((SparseVector&) V).getNNZ();
		int* resIr = new int[nnz];
		int* resJc = new int[N + 1];
		double* resPr = new double[nnz];
		std::copy(pr, pr + nnz, resPr);
		// System.arraycopy(pr, 0, resPr, 0, nnz);
		int lastColIdx = -1;
		int currentColIdx = 0;
		int idx = 0;
		for (int k = 0; k < nnz; k++) {
			idx = ir[k];
			currentColIdx = idx / M;
			resIr[k] = idx % M;
			while (lastColIdx < currentColIdx) {
				resJc[lastColIdx + 1] = k;
				lastColIdx++;
			}
		}
		resJc[N] = nnz;
		res = &SparseMatrix::createSparseMatrixByCSCArrays(resIr, resJc, resPr, M, N, nnz);
	}

	return *res;

}

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
Matrix& reshape(Vector& V, int* size) {
	return reshape(V, size[0], size[1]);
}

/**
 * Vectorize a matrix A.
 *
 * @param A a matrix
 *
 * @return Vectorization of a matrix A
 */
Matrix& vec(Matrix& A) {

	int M = A.getRowDimension();
	int N = A.getColumnDimension();

	Matrix* res = null;
	int dim = M * N;
	if (typeid(A) == typeid(DenseMatrix)) {
		double** resData = new double*[dim];
		double** AData = ((DenseMatrix&) A).getData();
		for (int j = 0, shift = 0; j < N; j++, shift += M) {
			for (int i = 0, shiftI = shift; i < M; i++, shiftI++) {
				resData[shiftI] = new double[1];
				resData[shiftI][0] = AData[i][j];
			}
		}
		res = new DenseMatrix(resData, dim, 1);
	} else {
		int* ir = ((SparseMatrix&) A).getIr();
		int* jc = ((SparseMatrix&) A).getJc();
		double* pr = ((SparseMatrix&) A).getPr();
		int nnz = ((SparseMatrix&) A).getNNZ();
		int* resIr = new int[nnz];
		int resJc[2] = {0, nnz};
		double* resPr = new double[nnz];
		std::copy(pr, pr + nnz, resPr);
		// System.arraycopy(pr, 0, resPr, 0, nnz);
		int cnt = 0;
		for (int j = 0, shift = 0; j < N; j++, shift += M) {
			for (int k = jc[j]; k <jc[j + 1]; k++) {
				resIr[cnt++] = ir[k] + shift;
			}
		}
		res = &SparseMatrix::createSparseMatrixByCSCArrays(resIr, resJc, resPr, dim, 1, nnz);
	}

	return *res;

}

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
Matrix** svd(Matrix& A) {
	Matrix** res = SVD::decompose(A);
	return res;
}

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
Matrix& repmat(Matrix& A, int M, int N) {

	Matrix* res = null;
	int nRow = M * A.getRowDimension();
	int nCol = N * A.getColumnDimension();
	if (typeid(A) == typeid(DenseMatrix)) {
		double** resData = allocate2DArray(nRow, nCol);
		double* resRow = null;
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;
		int r;
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			r = i % A.getRowDimension();
			ARow = AData[r];
			for (int k = 0, shift = 0; k < N; k++, shift += A.getColumnDimension()) {
				// System.arraycopy(ARow, 0, resRow, shift, A.getColumnDimension());
				std::copy(ARow, ARow + A.getColumnDimension(), resRow + shift);
			}
		}
		res = new DenseMatrix(resData, nRow, nCol);
	} else {
		int* ir = ((SparseMatrix&) A).getIr();
		int* jc = ((SparseMatrix&) A).getJc();
		double* pr = ((SparseMatrix&) A).getPr();
		int nnz = ((SparseMatrix&) A).getNNZ();

		int resNNZ = nnz * N * M;
		int* resIr = new int[resNNZ];
		int* resJc = new int[nCol + 1];
		double* resPr = new double[resNNZ];
		int* nnzPerColumn = new int[A.getColumnDimension()];
		for (int j = 0; j < A.getColumnDimension(); j++) {
			nnzPerColumn[j] = M * (jc[j + 1] - jc[j]);
		}
		resJc[0] = 0;
		for (int c = 0; c < nCol; c++) {
			int j = c % A.getColumnDimension();
			resJc[c + 1] = resJc[c] + nnzPerColumn[j];
		}
		for (int j = 0, shiftA = 0; j < A.getColumnDimension(); j++) {
			int numNNZACol_j = (jc[j + 1] - jc[j]);
			int* irACol_j = new int[numNNZACol_j];
			for (int k = 0, shift = shiftA * M; k < N; k++, shift += nnz * M) {
				// System.arraycopy(ir, shiftA, irACol_j, 0, numNNZACol_j);
				std::copy(ir + shiftA, ir + shiftA + numNNZACol_j, irACol_j);
				for (int i = 0, shift2 = shift; i < M; i++, shift2 += numNNZACol_j) {
					// System.arraycopy(irACol_j, 0, resIr, shift2, numNNZACol_j);
					std::copy(irACol_j, irACol_j + numNNZACol_j, resIr + shift2);
					if (i < M - 1)
						for (int t = 0; t < numNNZACol_j; t++)
							irACol_j[t] += A.getRowDimension();
					// System.arraycopy(pr, shiftA, resPr, shift2, numNNZACol_j);
					std::copy(pr + shiftA, pr + shiftA + numNNZACol_j, resPr + shift2);
				}
			}
			shiftA += numNNZACol_j;
		}
		res = &SparseMatrix::createSparseMatrixByCSCArrays(resIr, resJc, resPr, nRow, nCol, resNNZ);
	}
	return *res;

}

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
Matrix& repmat(Matrix& A, int* size) {
	return repmat(A, size[0], size[1]);
}

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
int size(Matrix& A, int dim) {
	if (dim == 1) {
		return A.getRowDimension();
	} else if (dim == 2) {
		return A.getColumnDimension();
	} else {
		err("Dim error!");
		exit(1);
	}
}

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
DenseVector& sum(Matrix& A, int dim) {
	double* sumValues = null;
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	double s = 0;
	int len = 0;
	if (dim == 1)
		len = N;
	else if (dim == 2)
		len = M;
	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;
		if (dim == 1) {
			sumValues = allocate1DArray(N, 0);
			for (int i = 0; i < M; i++) {
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					s = ARow[j];
					if (s != 0)
						sumValues[j] += s;
				}
			}
		} else if (dim == 2) {
			sumValues = allocate1DArray(M, 0);
			for (int i = 0; i < M; i++) {
				ARow = AData[i];
				s = 0;
				for (int j = 0; j < N; j++) {
					s += ARow[j];
				}
				sumValues[i] = s;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) A).getPr();
		if (dim == 1) {
			sumValues = allocate1DArray(N, 0);
			// int* ir = ((SparseMatrix&) A).getIr();
			int* jc = ((SparseMatrix&) A).getJc();
			for (int j = 0; j < N; j++) {
				if (jc[j] == jc[j + 1]) {
					sumValues[j] = 0;
					continue;
				}
				s = 0;
				for (int k = jc[j]; k < jc[j + 1]; k++) {
					s += pr[k];
				}
				sumValues[j] = s;
			}
		} else if (dim == 2) {
			sumValues = allocate1DArray(M, 0);
			// int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			for (int i = 0; i < M; i++) {
				if (jr[i] ==  jr[i + 1]) {
					sumValues[i] = 0;
					continue;
				}
				s = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					s += pr[valCSRIndices[k]];
				}
				sumValues[i] = s;
			}
		}
	}
	return *new DenseVector(sumValues, len);
}

/**
 * Compute the sum of elements for each row of a real matrix.
 *
 * @param A a real matrix
 *
 * @return a dense vector containing the sum of elements of
 *         each row of A, i.e. sum(A, 1)
 */
DenseVector& sum(Matrix& A) {
	return sum(A, 1);
}

/**
 * Compute the sum of a 1D {@code double} array.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 *
 * @return sum(V)

double sum(double* V, int len) {
	double res = 0;
	for (int i = 0; i < len; i++)
		res += V[i];
	return res;
}*/

/**
 * Compute the sum of all elements of a vector.
 *
 * @param V a real dense or sparse vector
 *
 * @return sum(V)
 */
double sum(Vector& V) {
	double res = 0;
	if (typeid(V) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) V).getPr();
		for (int k = 0; k < V.getDim(); k++) {
			res += pr[k];
		}

	} else {
		double* pr = ((SparseVector&) V).getPr();
		int nnz = ((SparseVector&) V).getNNZ();
		for (int k = 0; k < nnz; k++) {
			res += pr[k];
		}
	}
	return res;
}

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
Vector& mean(Matrix& X, int dim) {
	int N = size(X, dim);
	double* S = sum(X, dim).getPr();
	divideAssign(S, size(X, 3 - dim), N);
	return *new DenseVector(S, size(X, 3 - dim));
}

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
Matrix** eigs(Matrix& A, int K, std::string sigma) {

	Matrix** VD = EVD::decompose(A);
	Matrix& eigV = *VD[0];
	Matrix& eigD = *VD[1];

	/*disp(eigV);
	disp(eigD);*/

	int N = A.getRowDimension();
	Matrix** res = new Matrix*[2];

	Vector& eigenValueVector = *new DenseVector(K);
	Matrix* eigenVectors = null;
	if (sigma == "lm") {
		for (int k = 0; k < K; k++)
			eigenValueVector.set(k, eigD.getEntry(k, k));
		eigenVectors = &eigV.getSubMatrix(0, N - 1, 0, K - 1);
	} else if (sigma == "sm") {
		for (int k = 0; k < K; k++)
			eigenValueVector.set(k, eigD.getEntry(N - 1 - k, N - 1 - k));
		// eigenVectors = new DenseMatrix(N, K);
		double** eigenVectorsData = allocate2DArray(N, K);
		double** eigVData = ((DenseMatrix&) eigV).getData();
		double* eigenVectorsRow = null;
		double* eigVRow = null;
		// eigenVectors.setColumnVector(k, eigV.getColumnVector(j));
		for (int i = 0; i < N; i++) {
			eigenVectorsRow = eigenVectorsData[i];
			eigVRow = eigVData[i];
			for(int j = N - 1, k = 0; k < K ; j--, k++) {
				eigenVectorsRow[k] = eigVRow[j];
			}
		}
		eigenVectors = new DenseMatrix(eigenVectorsData, N, K);
	} else {
		err("sigma should be either \"lm\" or \"sm\"");
		exit(-1);
	}

	res[0] = eigenVectors;
	res[1] = &diag(eigenValueVector);

	delete &eigV;
	delete &eigD;
	delete[] VD;
	delete &eigenValueVector;

	return res;

}

/**
 * Construct a sparse diagonal matrix from a vector.
 *
 * @param V a real vector
 *
 * @return diag(V)
 */
SparseMatrix& diag(Vector& V) {
	int dim = V.getDim();
	SparseMatrix& res = *new SparseMatrix(dim, dim);
	for (int i = 0; i < dim; i++) {
		res.setEntry(i, i, V.get(i));
	}
	return res;
}

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
Matrix& diag(double* V, int len) {

	int d = len;
	Matrix& res = *new SparseMatrix(d, d);

	for (int i = 0; i < d; i++) {
		res.setEntry(i, i, V[i]);
	}

	return res;

}

/**
 * Calculate the element-wise logarithm of a matrix.
 *
 * @param A a matrix
 *
 * @return log(A)
 *
 */
DenseMatrix& log(Matrix& A) {

	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();

	double** resData = allocate2DArray(nRow, nCol);
	double* resRow = null;

	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			ARow = AData[i];
			for (int j = 0; j < nCol; j++) {
				resRow[j] = log(ARow[j]);
			}
		}
	} else {
		int* ic = ((SparseMatrix&) A).getIc();
		int* jr = ((SparseMatrix&) A).getJr();
		int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
		double* pr = ((SparseMatrix&) A).getPr();
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			if (jr[i + 1] == jr[i]) {
				/*for (int j = 0; j < nCol; j++)
					resRow[j] = NEGATIVE_INFINITY;*/
				assignVector(resRow, nCol, NEGATIVE_INFINITY);
				continue;
			}
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = jr[i]; k < jr[i + 1]; k++) {
				currentIdx = ic[k];
				for (int j = lastIdx + 1; j < currentIdx; j++) {
					resRow[j] = NEGATIVE_INFINITY;
				}
				resRow[currentIdx] = log(pr[valCSRIndices[k]]);
				lastIdx = currentIdx;
			}
			for (int j = lastIdx + 1; j < nCol; j++) {
				resRow[j] = NEGATIVE_INFINITY;
			}
		}
	}

	/*for (int i = 0; i < nRow; i++) {
			for (int j = 0; j < nCol; j++) {
				res.setEntry(i, j, log(A.getEntry(i, j)));
			}
		}*/

	return *new DenseMatrix(resData, nRow, nCol);

}

/**
 * Calculate TFIDF of a doc-term-count matrix, each column
 * is a data sample.
 *
 * @param docTermCountMatrix a matrix, each column is a data example
 *
 * @return TFIDF of docTermCountMatrix
 *
 */
Matrix& getTFIDF(Matrix& docTermCountMatrix) {

	int NTerm = docTermCountMatrix.getRowDimension();
	int NDoc = docTermCountMatrix.getColumnDimension();

	// Get TF vector
	double* tfVector = new double[NTerm];
	for (int i = 0; i < docTermCountMatrix.getRowDimension(); i++) {
		tfVector[i] = 0;
		for (int j = 0; j < docTermCountMatrix.getColumnDimension(); j++) {
			tfVector[i] += docTermCountMatrix.getEntry(i, j) > 0 ? 1 : 0;
		}
	}

	Matrix& res = docTermCountMatrix.copy();
	for (int i = 0; i < docTermCountMatrix.getRowDimension(); i++) {
		for (int j = 0; j < docTermCountMatrix.getColumnDimension(); j++) {
			if (res.getEntry(i, j) > 0) {
				res.setEntry(i, j, res.getEntry(i, j) * (tfVector[i] > 0 ? log(NDoc / tfVector[i]) : 0));
			}
		}
	}

	return res;

}

/**
 * Normalize A by columns.
 *
 * @param A a matrix
 *
 * @return a column-wise normalized matrix
 */
Matrix& normalizeByColumns(Matrix& A) {
	// double* AA = full(sqrt(sum(A.times(A)))).getPr();
	Matrix& AXA = A.times(A);
	DenseVector& S = sum(AXA);
	double* AA = S.getPr();
	for (int j = 0; j < A.getColumnDimension(); j++) {
		AA[j] = sqrt(AA[j]);
	}
	Matrix& res = A.copy();
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				resRow[j] /= AA[j];
			}
		}
	} else  {
		// int* ir = ((SparseMatrix) res).getIr();
		int* jc = ((SparseMatrix&) res).getJc();
		double* pr = ((SparseMatrix&) res).getPr();
		double v = 0;
		for (int j = 0; j < N; j++) {
			v = AA[j];
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				pr[k] /= v;
			}
		}
	}
	// delete[] AA;
	delete &AXA;
	delete &S;
	return res;
}

/**
 * Calculate square root for all elements of a vector V.
 *
 * @param V a real vector
 *
 * @return sqrt(V)
 */
Vector& sqrt(Vector& V) {
	int dim = V.getDim();
	Vector& res = V.copy();
	if (typeid(V) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) res).getPr();
		for (int k = 0; k < dim; k++) {
			pr[k] = sqrt(pr[k]);
		}
	} else {
		double* pr = ((SparseVector&) res).getPr();
		int nnz = ((SparseVector&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			pr[k] = sqrt(pr[k]);
		}
	}
	return res;
}

/**
 * Calculate square root for all elements of a matrix A.
 *
 * @param A a matrix
 *
 * @return sqrt(A)
 *
 */
Matrix& sqrt(Matrix& A) {
	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();

	Matrix& res = A.copy();

	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			for (int j = 0; j < nCol; j++) {
				resRow[j] = sqrt(resRow[j]);
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		for (int k = 0; k < ((SparseMatrix&) res).getNNZ(); k++) {
			pr[k] = sqrt(pr[k]);
		}
	}

	return res;
}

/**
 * Random permutation.
 * </br>
 * randperm(n) returns a random permutation of the integers 1:n.
 *
 * @param n an integer
 *
 * @return randperm(n)
 */
//int* randperm(int n) {

//	int* res = new int[n];

//	std::srand(unsigned(std::time(0)));
//	std::vector<int> myvector;
//	for (int i = 1; i <= n; i++) {
//		myvector.push_back(i);
//	}
//	std::random_shuffle(myvector.begin(), myvector.end());
//	int k = 0;
//	for (std::vector<int>::iterator iter = myvector.begin(); iter != myvector.end(); iter++) {
//		res[k++] = *iter;
//	}

//	return res;

//	/*int* res = new int[n];

//	Set<Integer> leftSet = new TreeSet<Integer>();
//	for (int i = 0; i < n; i++) {
//		leftSet.add(i);
//	}

//	Random generator = new Random();
//	for (int i = 0; i < n; i++) {
//		double* uniformDist = allocateVector(n - i, 1.0 / (n - i));

//		double rndRealScalor = generator.nextDouble();
//		double sum = 0;
//		for (int j = 0, k = 0; j < n; j++) {
//			if (!leftSet.contains(j))
//				continue;
//			sum += uniformDist[k];
//			if (rndRealScalor <= sum) {
//				res[i] = j + 1;
//				leftSet.remove(j);
//				break;
//			} else {
//				k++;
//			}
//		}
//	}*/

//	return res;

//}

/**
 * Find nonzero elements and return their indices.
 *
 * @param V a real vector
 *
 * @return an integer array of indices of nonzero elements of V
 *
 */
int* find(Vector& V) {

	int* indices = null;
	if (typeid(V) == typeid(DenseVector)) {
		// ArrayList<Integer> idxList = new ArrayList<Integer>();
		std::list<int> idxList;
		double* pr = ((DenseVector&) V).getPr();
		double v = 0;
		for (int k = 0; k < V.getDim(); k++) {
			v = pr[k];
			if (v != 0) {
				idxList.push_back(k);
			}
		}
		int nnz = idxList.size();
		indices = new int[nnz];
		std::list<int>::iterator idxIter = idxList.begin();
		int cnt = 0;
		while (idxIter != idxList.end()) {
			indices[cnt++] = *idxIter;
			idxIter++;
		}
	} else {
		((SparseVector&) V).clean();
		int nnz = ((SparseVector&) V).getNNZ();
		int* ir = ((SparseVector&) V).getIr();
		indices = new int[nnz];
		// System.arraycopy(ir, 0, indices, 0, nnz);
		std::copy(ir, ir + nnz, indices);
	}

	return indices;

}

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
FindResult& find(Matrix& A) {
	int* rows = null;
	int* cols = null;
	double* vals = null;
	int len = 0;
	if (typeid(A) == typeid(SparseMatrix)) {
		((SparseMatrix&) A).clean();
		int nnz = ((SparseMatrix&) A).getNNZ();
		rows = new int[nnz];
		cols = new int[nnz];
		vals = new double[nnz];
		int* ir = ((SparseMatrix&) A).getIr();
		int* jc = ((SparseMatrix&) A).getJc();
		double* pr = ((SparseMatrix&) A).getPr();
		int cnt = 0;
		for (int j = 0; j < A.getColumnDimension(); j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				rows[cnt] = ir[k];
				cols[cnt] = j;
				vals[cnt] = pr[k];
				cnt++;
			}
		}
		len = nnz;
	} else {
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		std::list<int> rowIdxList;
		std::list<int> colIdxList;
		std::list<double> valList;
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;
		double v = 0;
		for (int i = 0; i < M; i++) {
			ARow = AData[i];
			for (int j = 0; j < N; j++) {
				v = ARow[j];
				if (v != 0) {
					rowIdxList.push_back(i);
					colIdxList.push_back(j);
					valList.push_back(v);
				}
			}
		}
		int nnz = valList.size();
		rows = new int[nnz];
		cols = new int[nnz];
		vals = new double[nnz];
		std::list<int>::iterator rowIdxIter = rowIdxList.begin();
		std::list<int>::iterator colIdxIter = colIdxList.begin();
		std::list<double>::iterator valIter = valList.begin();
		int cnt = 0;
		while (valIter != valList.end()) {
			rows[cnt] = *rowIdxIter;
			cols[cnt] = *colIdxIter;
			vals[cnt] = *valIter;
			cnt++;
			rowIdxIter++;
			colIdxIter++;
			valIter++;
		}
		len = nnz;
	}
	return *new FindResult(rows, cols, vals, len);

}

/**
 * Compute the element-wise exponential of a matrix
 *
 * @param A a matrix
 *
 * @return exp(A)
 *
 */
Matrix& exp(Matrix& A) {

	int M = A.getRowDimension();
	int N = A.getColumnDimension();

	Matrix& res = *new DenseMatrix(M, N, 1);
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;

	if (typeid(A) == typeid(DenseMatrix)) {
		double** data = ((DenseMatrix&) A).getData();
		double* row = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			row = data[i];
			for (int j = 0; j < N; j++) {
				resRow[j] = exp(row[j]);
			}
		}
	} else {
		double* pr = ((SparseMatrix&) A).getPr();
		int* ir = ((SparseMatrix&) A).getIr();
		int* jc = ((SparseMatrix&) A).getJc();
		for (int j = 0; j < N; j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				resData[ir[k]][j] = exp(pr[k]);
			}
		}
	}

	return res;

}

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
Matrix& vertcat(int numMatrix, ...) {
	va_list arguments;                     // A place to store the list of arguments
	std::list<Matrix*> list;
	va_start(arguments, numMatrix);           // Initializing arguments to store all values after num
	for (int i = 0; i < numMatrix; i++) {
		Matrix* argument = va_arg(arguments, Matrix*);
		list.push_back(argument);
	}
	va_end(arguments);
	Matrix** matArr = new Matrix*[list.size()];
	int k = 0;
	for (std::list<Matrix*>::iterator iter = list.begin(); iter != list.end(); iter++) {
		matArr[k++] = *iter;
	}
	Matrix& res = vertcat(matArr, numMatrix);
	delete[] matArr;
	return res;
}

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
Matrix& vertcat(Matrix** As, int numMatrix) {
	int nM = numMatrix;
	int nRow = 0;
	int nCol = 0;
	for (int i = 0; i < nM; i++) {
		if (As[i] == null)
			continue;
		nRow += As[i]->getRowDimension();
		nCol = As[i]->getColumnDimension();
	}

	for (int i = 1; i < nM; i++) {
		if (As[i] != null && nCol != As[i]->getColumnDimension()) {
			err("Any matrix in the argument list should either be empty matrix or have the same number of columns to the others!");
			exit(1);
		}
	}

	if (nRow == 0 || nCol == 0) {
		exit(1);
	}

	DenseMatrix& res = *new DenseMatrix(nRow, nCol);
	double** resData = res.getData();
	double* resRow = null;
	int idx = 0;
	for (int i = 0; i < nM; i++) {
		if (i > 0 && As[i - 1] != null)
			idx += As[i - 1]->getRowDimension();
		if (As[i] == null)
			continue;
		if (typeid(*As[i]) == typeid(DenseMatrix)) {
			DenseMatrix* A = (DenseMatrix*) As[i];
			double** AData = A->getData();
			for (int r = 0; r < A->getRowDimension(); r++) {
				// res.setRow(idx + r, As[i].getRow(r));
				// resData[idx + r] = AData[r].clone();
				resData[idx + r] = clone(AData[r], nCol);
			}
		} else {
			SparseMatrix* A = (SparseMatrix*) As[i];
			double* pr = A->getPr();
			int* ic = A->getIc();
			int* jr = A->getJr();
			int* valCSRIndices = A->getValCSRIndices();
			for (int r = 0; r < A->getRowDimension(); r++) {
				resRow = allocate1DArray(nCol, 0);
				for (int k = jr[r]; k < jr[r + 1]; k++) {
					resRow[ic[k]] = pr[valCSRIndices[k]];
				}
				resData[idx + r] = resRow;
			}
		}
	}
	return res;
}

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
Matrix& horzcat(int numMatrix, ...) {
	va_list arguments;                     // A place to store the list of arguments
	std::list<Matrix*> list;
	va_start(arguments, numMatrix);           // Initializing arguments to store all values after num
	for (int i = 0; i < numMatrix; i++) {
		Matrix* argument = va_arg(arguments, Matrix*);
		list.push_back(argument);
	}
	va_end(arguments);
	Matrix** matArr = new Matrix*[list.size()];
	int k = 0;
	for (std::list<Matrix*>::iterator iter = list.begin(); iter != list.end(); iter++) {
		matArr[k++] = *iter;
	}
	Matrix& res = horzcat(matArr, numMatrix);
	delete[] matArr;
	return res;
}

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
Matrix& horzcat(Matrix** As, int numMatrix) {
	int nM = numMatrix;
	int nCol = 0;
	int nRow = 0;
	for (int i = 0; i < nM; i++) {
		if (As[i] != null) {
			nCol += As[i]->getColumnDimension();
			nRow = As[i]->getRowDimension();
		}
	}

	for (int i = 1; i < nM; i++) {
		if (As[i] != null && nRow != As[i]->getRowDimension()) {
			err("Any matrix in the argument list should either be empty matrix or have the same number of rows to the others!");
			exit(1);
		}
	}

	if (nRow == 0 || nCol == 0) {
		exit(1);
	}

	DenseMatrix& res = *new DenseMatrix(nRow, nCol, 0);
	double** resData = res.getData();
	double* resRow = null;
	int idx = 0;

	for (int r = 0; r < nRow; r++) {
		resRow = resData[r];
		idx = 0;
		for (int i = 0; i < nM; i++) {
			if (i > 0 && As[i - 1] != null)
				idx += As[i - 1]->getColumnDimension();
			if (As[i] == null)
				continue;
			if (typeid(*As[i]) == typeid(DenseMatrix)) {
				DenseMatrix* A = (DenseMatrix*) As[i];
				// System.arraycopy(A->getData()[r], 0, resRow, idx, A->getColumnDimension());
				std::copy(A->getData()[r], A->getData()[r] + A->getColumnDimension(), resRow + idx);
			} else {
				SparseMatrix* A = (SparseMatrix*) As[i];
				double* pr = A->getPr();
				int* ic = A->getIc();
				int* jr = A->getJr();
				int* valCSRIndices = A->getValCSRIndices();
				for (int k = jr[r]; k < jr[r + 1]; k++) {
					resRow[idx + ic[k]] = pr[valCSRIndices[k]];
				}
			}
		}
	}

	return res;

}

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
Matrix& cat(int dim, Matrix** As, int numMatrices) {
	Matrix* res = null;
	if (dim == 1)
		res = &vertcat(As, numMatrices);
	else if (dim == 2)
		res = &horzcat(As, numMatrices);
	else {
		err("Specified dimension can only be either 1 or 2 currently!");
		exit(1);
	}
	return *res;
}

/**
 * Compute the sum of all elements of a matrix.
 *
 * @param A a matrix
 *
 * @return sum(sum(A))
 *
 */
double sumAll(Matrix& A) {
	DenseVector& S = sum(A);
	double res = sum(S);
	delete &S;
	return res;
}

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
Matrix& diag(Matrix& A) {
	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();
	Matrix* res = null;

	if (nRow == 1) {
		res = new SparseMatrix(nCol, nCol);
		for (int i = 0; i < nCol; i++) {
			res->setEntry(i, i, A.getEntry(0, i));
		}
	} else if (nCol == 1) {
		res = new SparseMatrix(nRow, nRow);
		for (int i = 0; i < nRow; i++) {
			res->setEntry(i, i, A.getEntry(i, 0));
		}
		// disp(*res);
	} else if (nRow == nCol) {
		res = new DenseMatrix(nRow, 1);
		for (int i = 0; i < nRow; i++) {
			res->setEntry(i, 0, A.getEntry(i, i));
		}
	}

	return *res;
}

/**
 * Right array division.
 *
 * @param A a matrix
 *
 * @param v a scalar
 *
 * @return A ./ v
 */
Matrix& rdivide(Matrix& A, double v) {
	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();

	Matrix& res = A.copy();

	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			for (int j = 0; j < nCol; j++) {
				resRow[j] /= v;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		for (int k = 0; k < ((SparseMatrix&) res).getNNZ(); k++) {
			pr[k] /= v;
		}
	}

	return res;
}

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
//Matrix& rand(int nRow, int nCol) {
//	std::default_random_engine generator(time(NULL));
//	std::uniform_real_distribution<double> uniform_real(0.0, 1.0);
//	// double randnumber = uniform_real(generator);
//	Matrix& res = *new DenseMatrix(nRow, nCol);
//	for (int i = 0; i < nRow; i++) {
//		for (int j = 0; j < nCol; j++) {
//			res.setEntry(i, j, uniform_real(generator));
//		}
//	}
//	return res;
//}

/**
 * Generate an n-by-n matrix containing pseudo-random values drawn
 * from the standard uniform distribution on the open interval (0,1).
 *
 * @param n number of rows or columns
 *
 * @return rand(n, n)
 *
 */
//Matrix& rand(int n) {
//	return rand(n, n);
//}

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
//Matrix& randn(int nRow, int nCol) {
//	// Random generator = new Random();
//	std::default_random_engine generator(time(NULL));
//	std::normal_distribution<double> normal(0.0, 1.0);
//	Matrix& res = *new DenseMatrix(nRow, nCol);
//	for (int i = 0; i < nRow; i++) {
//		for (int j = 0; j < nCol; j++) {
//			res.setEntry(i, j, normal(generator));
//		}
//	}
//	return res;
//}

/**
 * Generate an n-by-n matrix containing pseudo-random values drawn
 * from the standard normal distribution.
 *
 * @param n number of rows or columns
 *
 * @return randn(n, n)
 *
 */
//Matrix& randn(int n) {
//	return randn(n, n);
//}

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
Matrix& sign(Matrix& A) {
	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();

	Matrix& res = A.copy();
	double v = 0;
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			for (int j = 0; j < nCol; j++) {
				v = resRow[j];
				if (v > 0) {
					resRow[j] = 1;
				} else if (v < 0) {
					resRow[j] = -1;
				}
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		for (int k = 0; k < ((SparseMatrix&) res).getNNZ(); k++) {
			v = pr[k];
			if (v > 0) {
				pr[k] = 1;
			} else if (v < 0) {
				pr[k] = -1;
			}
		}
	}

	return res;
}

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
Matrix& l2DistanceSquare(Matrix& X, Matrix& Y) {

	int nX = X.getRowDimension();
	int nY = Y.getRowDimension();

	Matrix& XXX = times(X, X);
	Matrix& YXY = times(Y, Y);
	DenseVector& SumX = sum(XXX, 2);
	DenseVector& SumY = sum(YXY, 2);
	delete &XXX;
	delete &YXY;

	double* XX = SumX.getPr();
	double* YY = SumY.getPr();

	Matrix& YT = Y.transpose();
	Matrix& XYT = X.mtimes(YT);
	delete &YT;
	Matrix& XYTm2 = XYT.times(-2);
	delete &XYT;
	DenseMatrix& fullXYTm2 = full(XYTm2);

	DenseMatrix& dist = fullXYTm2;
	if (typeid(XYTm2) == typeid(SparseMatrix))
		delete &XYTm2;

	double** resData = dist.getData();
	double* resRow = null;
	double s = 0;
	double v = 0;
	for (int i = 0; i < nX; i++) {
		resRow = resData[i];
		s = XX[i];
		for (int j = 0; j < nY; j++) {
			v = resRow[j] + s + YY[j];
			// resRow[j] += s + YY[j];
			if (v >= 0)
				resRow[j] = v;
			else
				resRow[j] = 0;
		}
	}

	/*Matrix I = lt(dist, 0);
		logicalIndexingAssignment(dist, I, 0);*/
	delete &SumX;
	delete &SumY;

	return dist;

}

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
Vector& l2DistanceSquare(Vector& V, Matrix& Y) {

	// int nX = 1;
	int nY = Y.getRowDimension();

	Vector& VXV = times(V, V);
	Matrix& YXY = times(Y, Y);
	DenseVector& SumY = sum(YXY, 2);
	double XX = sum(VXV);
	double* YY = SumY.getPr();
	delete &VXV;
	delete &YXY;

	Vector& YV = Y.operate(V);
	Vector& YVm2 = YV.times(-2);
	delete& YV;
	DenseVector& fullYVm2 = full(YVm2);

	DenseVector& dist = fullYVm2;
	if (typeid(YVm2) == typeid(SparseVector))
		delete &YVm2;

	double* pr = dist.getPr();
	double v = 0;
	for (int j = 0; j < nY; j++) {
		v = pr[j] + XX + YY[j];
		if (v >= 0)
			pr[j] = v;
		else
			pr[j] = 0;
		// pr[j] += XX + YY[j];
	}

	/*Matrix I = lt(dist, 0);
		logicalIndexingAssignment(dist, I, 0);*/
	delete &SumY;
	return dist;

}

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
Matrix& l2DistanceSquareByColumns(Matrix& X, Matrix& Y) {

	int nX = X.getColumnDimension();
	int nY = Y.getColumnDimension();

	/*double* XX = sum(times(X, X)).getPr();
	double* YY = sum(times(Y, Y)).getPr();*/
	Matrix& XXX = times(X, X);
	Matrix& YXY = times(Y, Y);
	DenseVector& SumX = sum(XXX);
	DenseVector& SumY = sum(YXY);
	delete &XXX;
	delete &YXY;

	double* XX = SumX.getPr();
	double* YY = SumY.getPr();

	// DenseMatrix& dist = full(X.transpose().mtimes(Y).times(-2));
	Matrix& YT = Y.transpose();
	Matrix& XYT = X.mtimes(YT);
	delete &YT;
	Matrix& XYTm2 = XYT.times(-2);
	delete &XYT;
	DenseMatrix& fullXYTm2 = full(XYTm2);

	DenseMatrix& dist = fullXYTm2;
	if (typeid(XYTm2) == typeid(SparseMatrix))
		delete &XYTm2;

	double** resData = dist.getData();
	double* resRow = null;
	double s = 0;
	double v = 0;
	for (int i = 0; i < nX; i++) {
		resRow = resData[i];
		s = XX[i];
		for (int j = 0; j < nY; j++) {
			v = resRow[j] + s + YY[j];
			// resRow[j] += XX[i] + YY[j];
			if (v >= 0)
				resRow[j] = v;
			else
				resRow[j] = 0;
		}
	}

	/*Matrix I = lt(dist, 0);
		logicalIndexingAssignment(dist, I, 0);*/
	delete[] XX;
	delete[] YY;
	return dist;

}

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
Matrix& l2DistanceSquare(Vector** X, int lenX, Vector** Y, int lenY) {

	int nX = lenX;
	int nY = lenY;

	double* XX = new double[nX];
	Vector* V = null;
	for (int i = 0; i < nX; i++) {
		V = X[i];
		XX[i] = sum(V->times(*V));
	}
	double* YY = new double[nY];
	for (int i = 0; i < nY; i++) {
		V = Y[i];
		YY[i] = sum(V->times(*V));
	}

	double** resData = allocate2DArray(nX, nY, 0);
	double* resRow = null;
	double s = 0;
	double v = 0;
	for (int i = 0; i < nX; i++) {
		resRow = resData[i];
		V = X[i];
		s = XX[i];
		for (int j = 0; j < nY; j++) {
			v = s + YY[j] - 2 * innerProduct(*V, *Y[j]);;
			// resRow[j] = s + YY[j] - 2 * innerProduct(V, Y[j]);
			if (v >= 0)
				resRow[j] = v;
			else
				resRow[j] = 0;
		}
	}

	/*Matrix I = lt(dist, 0);
		logicalIndexingAssignment(dist, I, 0);*/
	Matrix& dist = *new DenseMatrix(resData, nX, nY);
	delete[] XX;
	delete[] YY;
	return dist;

}

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
Matrix& l2DistanceByColumns(Matrix& X, Matrix& Y) {
	// return sqrt(l2DistanceSquareByColumns(X, Y));
	Matrix& D = l2DistanceSquareByColumns(X, Y);
	Matrix& res = sqrt(D);
	delete &D;
	return res;
}

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
Matrix& l2Distance(Matrix& X, Matrix& Y) {
	Matrix& D = l2DistanceSquare(X, Y);
	Matrix& res = sqrt(D);
	delete &D;
	return res;
}

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
Vector& l2Distance(Vector& V, Matrix& Y) {
	Vector& D = l2DistanceSquare(V, Y);
	Vector& res = sqrt(D);
	delete &D;
	return res;
}

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
Matrix& l2Distance(Vector** X, int lenX, Vector** Y, int lenY) {
	Matrix& D = l2DistanceSquare(X, lenX, Y, lenY);
	Matrix& res = sqrt(D);
	delete &D;
	return res;
}

/**
 * Compute the inner product of two vectors, i.e. res = <V1, V2>.
 *
 * @param V1 the first vector
 *
 * @param V2 the second vector
 *
 * @return <V1, V2>
 */
double innerProduct(Vector& V1, Vector& V2) {
	if (V1.getDim() != V2.getDim()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	double res = 0;
	double v = 0;
	if (typeid(V1) == typeid(DenseVector)) {
		double* pr1 = ((DenseVector&) V1).getPr();
		if (typeid(V2) == typeid(DenseVector)) {
			if (&V1 == &V2) {
				for (int k = 0; k < V1.getDim(); k++) {
					v = pr1[k];
					res += v * v;
				}
				return res;
			}

			double* pr2 = ((DenseVector&) V2).getPr();
			for (int k = 0; k < V1.getDim(); k++) {
				res += pr1[k] * pr2[k];
			}
		} else {
			int* ir = ((SparseVector&) V2).getIr();
			double* pr2 = ((SparseVector&) V2).getPr();
			int nnz = ((SparseVector&) V2).getNNZ();
			for (int k = 0; k < nnz; k++) {
				res += pr1[ir[k]] * pr2[k];
			}
		}
	} else {
		if (typeid(V2) == typeid(DenseVector)) {
			return innerProduct(V2, V1);
		} else {
			int* ir1 = ((SparseVector&) V1).getIr();
			double* pr1 = ((SparseVector&) V1).getPr();
			int nnz1 = ((SparseVector&) V1).getNNZ();
			if (&V1 == &V2) {
				for (int k = 0; k < nnz1; k++) {
					v = pr1[k];
					res += v * v;
				}
				return res;
			} else {
				int* ir2 = ((SparseVector&) V2).getIr();
				double* pr2 = ((SparseVector&) V2).getPr();
				int nnz2 = ((SparseVector&) V2).getNNZ();
				int k1 = 0;
				int k2 = 0;
				int i1 = 0;
				int i2 = 0;
				while (true) {
					if (k1 >= nnz1 || k2 >= nnz2) {
						break;
					}
					i1 = ir1[k1];
					i2 = ir2[k2];
					if (i1 < i2) {
						k1++;
					} else if (i1 > i2) {
						k2++;
					} else {
						res += pr1[k1] * pr2[k2];
						k1++;
						k2++;
					}
				}
			}
		}
	}
	return res;
}

/**
 * Compute the sum of all elements of a matrix.
 *
 * @param A a matrix
 *
 * @return sum(sum(A))
 *
 */
double innerProduct(Matrix& A, Matrix& B) {

	double s = 0;

	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		if (typeid(B) == typeid(DenseMatrix)) {
			double** BData = ((DenseMatrix&) B).getData();
			double* BRow = null;
			double* ARow = null;
			// double* resRow = null;
			for (int i = 0; i < M; i++) {
				ARow = AData[i];
				BRow = BData[i];
				for (int j = 0; j < N; j++) {
					s += ARow[j] * BRow[j];
				}
			}
		} else {
			int* ir = null;
			int* jc = null;
			double* pr = null;
			ir = ((SparseMatrix&) B).getIr();
			jc = ((SparseMatrix&) B).getJc();
			pr = ((SparseMatrix&) B).getPr();
			int r = -1;
			for (int j = 0; j < B.getColumnDimension(); j++) {
				for (int k = jc[j]; k < jc[j + 1]; k++) {
					r = ir[k];
					// A[r][j] = pr[k]
					s += AData[r][j] * pr[k];
				}
			}
		}
	} else {
		if (typeid(B) == typeid(DenseMatrix)) {
			return innerProduct(B, A);
		} else {
			int* ir1 = null;
			int* jc1 = null;
			double* pr1 = null;
			ir1 = ((SparseMatrix&) A).getIr();
			jc1 = ((SparseMatrix&) A).getJc();
			pr1 = ((SparseMatrix&) A).getPr();
			int* ir2 = null;
			int* jc2 = null;
			double* pr2 = null;
			ir2 = ((SparseMatrix&) B).getIr();
			jc2 = ((SparseMatrix&) B).getJc();
			pr2 = ((SparseMatrix&) B).getPr();

			int k1 = 0;
			int k2 = 0;
			int r1 = -1;
			int r2 = -1;
			// int i = -1;
			double v = 0;

			for (int j = 0; j < N; j++) {
				k1 = jc1[j];
				k2 = jc2[j];

				// If the j-th column of A or this is empty, we don't need to compute.
				if (k1 == jc1[j + 1] || k2 == jc2[j + 1])
					continue;

				while (k1 < jc1[j + 1] && k2 < jc2[j + 1]) {

					r1 = ir1[k1];
					r2 = ir2[k2];
					if (r1 < r2) {
						k1++;
					} else if (r1 == r2) {
						// i = r1;
						v = pr1[k1] * pr2[k2];
						k1++;
						k2++;
						if (v != 0) {
							s += v;
						}
					} else {
						k2++;
					}

				}

			}
		}
	}
	return s;

}

/**
 * Calculate element by element division between a scalar and a vector.
 *
 * @param v a real scalar
 *
 * @param V a real vector
 *
 * @return v ./ V
 */
Vector& dotDivide(double v, Vector& V) {
	int dim = V.getDim();
	DenseVector& res = (DenseVector&) full(V).copy();
	double* pr = ((DenseVector&) res).getPr();
	for (int k = 0; k < dim; k++) {
		pr[k] = v / pr[k];
	}
	return res;
}

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
Matrix& zeros(int nRow, int nCol) {
	if (nRow == 0 || nCol == 0) {
		err("nRow is 0 or nCol is 0");
		exit(1);
	}
	return *new DenseMatrix(nRow, nCol, 0);
}

/**
 * Generate an all zero matrix with its size
 * specified by a two dimensional integer array.
 *
 * @param size a two dimensional integer array
 *
 * @return an all zero matrix with its shape specified by size
 *
 */
Matrix& zeros(int* size) {
	return zeros(size[0], size[1]);
}

/**
 * Generate an n by n all zero matrix.
 *
 * @param n number of rows and columns
 *
 * @return ones(n)
 *
 */
Matrix& zeros(int n) {
	return zeros(n, n);
}

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
Matrix& ones(int nRow, int nCol) {
	if (nRow == 0 || nCol == 0) {
		err("nRow is 0 or nCol is 0");
		exit(1);
	}
	return *new DenseMatrix(nRow, nCol, 1);
}

/**
 * Generate an all one matrix with its size
 * specified by a two dimensional integer array.
 *
 * @param size a two dimensional integer array
 *
 * @return an all one matrix with its shape specified by size
 *
 */
Matrix& ones(int* size) {
	return ones(size[0], size[1]);
}

/**
 * Generate an n by n all one matrix.
 *
 * @param n number of rows and columns
 *
 * @return ones(n)
 *
 */
Matrix& ones(int n) {
	return ones(n, n);
}

/**
 * Compute the determinant of a real square matrix.
 *
 * @param A a real square matrix
 *
 * @return det(A)
 */
double det(Matrix& A) {
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (M != N) {
		err("Input should be a square matrix.");
		exit(1);
	}
	LU LUDecomp(A);
	double res = LUDecomp.det();
	// LUDecomp.~LU();
	// delete &LUDecomp;
	return res;
}

/**
 * Compute the inverse of a real square matrix.
 *
 * @param A a real square matrix
 *
 * @return inv(A)
 */
Matrix& inv(Matrix& A) {
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (M != N) {
		err("Input should be a square matrix.");
		exit(1);
	}
	LU LUDecomp(A);
	if (LUDecomp.det() == 0) {
		err("The input matrix is not invertible.");
		exit(1);
	}
	return LUDecomp.inverse();
}

/**
 * Sort elements of a vector V in place with an increasing order.
 *
 * @param V a real vector, it will be sorted in place.
 *
 * @return sorted indices represented by a 1D {@code double} array
 */
double* sort(Vector& V) {
	return sort(V, "ascend");
}

/**
 * Sort elements of a vector V in place with a specified order.
 *
 * @param V a real vector, it will be sorted in place.
 *
 * @param order a {@code String} variable either "ascend" or "descend"
 *
 * @return sorted indices represented by a 1D {@code double} array
 */
double* sort(Vector& V, std::string order) {
	double* indices = null;
	int dim = V.getDim();
	if (typeid(V) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) V).getPr();
		indices = new double[dim];
		for (int k = 0; k < dim; k++) {
			indices[k] = k;
		}
		quickSort(pr, indices, 0, dim - 1, order);
	} else {
		double* pr = ((SparseVector&) V).getPr();
		int* ir = ((SparseVector&) V).getIr();
		int nnz = ((SparseVector&) V).getNNZ();
		indices = new double[dim];

		int insertionPos = nnz;
		if (order == "ascend") {
			for (int k = 0; k < nnz; k++) {
				if (pr[k] >= 0) {
					insertionPos--;
				}
			}
		} else if (order == "descend") {
			for (int k = 0; k < nnz; k++) {
				if (pr[k] <= 0) {
					insertionPos--;
				}
			}
		}

		int lastIdx = -1;
		int currentIdx = 0;
		int cnt = insertionPos;
		for (int k = 0; k < nnz; k++) {
			currentIdx = ir[k];
			for (int idx = lastIdx + 1; idx < currentIdx; idx++) {
				indices[cnt++] = idx;
			}
			lastIdx = currentIdx;
		}
		for (int idx = lastIdx + 1; idx < dim; idx++) {
			indices[cnt++] = idx;
		}

		quickSort(pr, ir, 0, nnz - 1, order);

		for (int k = 0; k < insertionPos; k++) {
			indices[k] = ir[k];
		}
		for (int k = insertionPos; k < nnz; k++) {
			indices[k + dim - nnz] = ir[k];
		}

		for (int k = 0; k < nnz; k++) {
			if (k < insertionPos) {
				ir[k] = k;
			} else {
				ir[k] = k + dim - nnz;
			}
		}
	}
	return indices;
}

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
Matrix** sort(Matrix& A, int dim, std::string order) {

	if (dim != 1 && dim != 2) {
		err("Dimension should be either 1 or 2.");
		exit(1);
	}

	if (order != "ascend" && order != "descend") {
		err("Order should be either \"ascend\" or \"descend\".");
		exit(1);
	}

	Matrix** res = new Matrix*[2];
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	Matrix* sortedValues = null;
	Matrix* sortedIndices = null;
	double** sortedIndexData = null;
	if (typeid(A) == typeid(DenseMatrix)) {
		sortedValues = &A.copy();
		sortedIndices = null;
		double** data = ((DenseMatrix*) sortedValues)->getData();
		if (dim == 2) {
			sortedIndexData = new double*[M];
			double* values = null;
			double* indices = null;
			for (int i = 0; i < M; i++) {
				values = data[i];
				indices = new double[N];
				for (int j = 0; j < N; j++) {
					indices[j] = j;
				}
				quickSort(values, indices, 0, N - 1, order);
				sortedIndexData[i] = indices;
			}
			sortedIndices = new DenseMatrix(sortedIndexData, M, N);
		} else if (dim == 1) {
			Matrix& AT = A.transpose();
			Matrix** res2 = sort(AT, 2, order);
			sortedValues = &res2[0]->transpose();
			sortedIndices = &res2[1]->transpose();
			delete &AT;
			delete res2[0];
			delete res2[1];
			delete[] res2;
		}
	} else {
		if (dim == 2) {
			Vector** rowVectors = sparseMatrix2SparseRowVectors(A);
			sortedIndexData = new double*[M];
			for (int i = 0; i < M; i++) {
				sortedIndexData[i] = sort(*rowVectors[i], order);
			}
			sortedIndices = new DenseMatrix(sortedIndexData, M, N);
			sortedValues = &sparseRowVectors2SparseMatrix(rowVectors, M);
		} else if (dim == 1) {
			Matrix& AT = A.transpose();
			Matrix** res2 = sort(AT, 2, order);
			sortedValues = &res2[0]->transpose();
			sortedIndices = &res2[1]->transpose();
			delete &AT;
			delete res2[0];
			delete res2[1];
			delete[] res2;
		}
	}

	res[0] = sortedValues;
	res[1] = sortedIndices;
	return res;

}

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
Matrix** sort(Matrix& A, int dim) {
	return sort(A, dim, "ascend");
}

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
Matrix** sort(Matrix& A, std::string order) {
	return sort(A, 1, order);
}

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
Matrix** sort(Matrix& A) {
	return sort(A, "ascend");
}

/**
 * Get a two dimensional integer array for size of a matrix A.
 *
 * @param A a matrix
 *
 * @return size(A)
 *
 */
int* size(Matrix& A) {
	int* res = new int[2];
	res[0] = A.getRowDimension();
	res[1] = A.getColumnDimension();
	return res;
}

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
Matrix& max(Matrix& A, double v) {

	Matrix* res = null;
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (typeid(A) == typeid(DenseMatrix)) {
		res = &A.copy();
		double** resData = ((DenseMatrix*) res)->getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				if (resRow[j] < v) {
					resRow[j] = v;
				}
			}
		}
	} else {
		if (v != 0) {
			res = new DenseMatrix(M, N, v);
			double** resData = ((DenseMatrix*) res)->getData();
			double* resRow = null;
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					if (v < 0)
						assignVector(resRow, N, 0);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					if (v < 0)
						for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
							resRow[c] = 0;
						}
					resRow[currentColumnIdx] = max(pr[valCSRIndices[k]], v);
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					if (v < 0)
						resRow[c] = 0;
				}
			}
		} else { // v == 0
			SparseMatrix& ZeroMat = *new SparseMatrix(M, N);
			res = &max(A, ZeroMat);
			delete &ZeroMat;
		}
	}
	return *res;
}

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
Matrix& max(double v, Matrix& A) {
	return max(A, v);
}

/**
 * Compute maximum between two real matrices A and B.
 *
 * @param A a real matrix
 *
 * @param B a real matrix
 *
 * @return max(A, B);
 */
Matrix& max(Matrix& A, Matrix& B) {
	Matrix* res = null;
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	double v = 0;
	if (typeid(A) == typeid(DenseMatrix)) {
		res = &A.copy();
		double** resData = ((DenseMatrix*) res)->getData();
		double* resRow = null;
		if (typeid(B) == typeid(DenseMatrix)) {
			double** BData = ((DenseMatrix&) B).getData();
			double* BRow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				BRow = BData[i];
				resRow = resData[i];
				for (int j = 0; j < N; j++) {
					v = BRow[j];
					if (resRow[j] < v)
						resRow[j] = v;
				}
			}
		} else {
			int* ic = ((SparseMatrix&) B).getIc();
			int* jr = ((SparseMatrix&) B).getJr();
			int* valCSRIndices = ((SparseMatrix&) B).getValCSRIndices();
			double* pr = ((SparseMatrix&) B).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					for (int j = 0; j < N; j++) {
						if (resRow[j] < 0)
							resRow[j] = 0;
					}
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						if (resRow[c] < 0)
							resRow[c] = 0;
					}
					resRow[currentColumnIdx] = max(pr[valCSRIndices[k]], resRow[currentColumnIdx]);
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					if (resRow[c] < 0)
						resRow[c] = 0;
				}
			}
		}
	} else {
		if (typeid(B) == typeid(DenseMatrix)) {
			return max(B, A);
		} else {

			res = new SparseMatrix(M, N);
			int* ir1 = null;
			int* jc1 = null;
			double* pr1 = null;
			ir1 = ((SparseMatrix&) A).getIr();
			jc1 = ((SparseMatrix&) A).getJc();
			pr1 = ((SparseMatrix&) A).getPr();
			int* ir2 = null;
			int* jc2 = null;
			double* pr2 = null;
			ir2 = ((SparseMatrix&) B).getIr();
			jc2 = ((SparseMatrix&) B).getJc();
			pr2 = ((SparseMatrix&) B).getPr();

			int k1 = 0;
			int k2 = 0;
			int r1 = -1;
			int r2 = -1;
			int i = -1;
			// double v = 0;

			for (int j = 0; j < N; j++) {
				k1 = jc1[j];
				k2 = jc2[j];

				// Both A and B's j-th columns are empty.
				if (k1 == jc1[j + 1] && k2 == jc2[j + 1])
					continue;

				while (k1 < jc1[j + 1] || k2 < jc2[j + 1]) {

					if (k2 == jc2[j + 1]) { // B's j-th column has been processed.
						i = ir1[k1];
						v = pr1[k1];
						if (v < 0)
							v = 0;
						k1++;
					} else if (k1 == jc1[j + 1]) { // A's j-th column has been processed.
						i = ir2[k2];
						v = pr2[k2];
						if (v < 0)
							v = 0;
						k2++;
					} else { // Both A and B's j-th columns have not been fully processed.
						r1 = ir1[k1];
						r2 = ir2[k2];
						if (r1 < r2) {
							i = r1;
							v = pr1[k1];
							if (v < 0)
								v = 0;
							k1++;
						} else if (r1 == r2) {
							i = r1;
							v = max(pr1[k1], pr2[k2]);
							k1++;
							k2++;
						} else {
							i = r2;
							v = pr2[k2];
							if (v < 0)
								v = 0;
							k2++;
						}
					}
					if (v != 0)
						res->setEntry(i, j, v);
				}
			}
		}
	}

	return *res;

}

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
Matrix& min(Matrix& A, double v) {

	Matrix* res = null;
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (typeid(A) == typeid(DenseMatrix)) {
		res = &A.copy();
		double** resData = ((DenseMatrix*) res)->getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				if (resRow[j] > v) {
					resRow[j] = v;
				}
			}
		}
	} else {
		if (v != 0) {
			res = new DenseMatrix(M, N, v);
			double** resData = ((DenseMatrix*) res)->getData();
			double* resRow = null;
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					if (v > 0)
						assignVector(resRow, N, 0);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					if (v > 0)
						for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
							resRow[c] = 0;
						}
					resRow[currentColumnIdx] = min(pr[valCSRIndices[k]], v);
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					if (v > 0)
						resRow[c] = 0;
				}
			}
		} else { // v == 0
			// return min(A, *new SparseMatrix(M, N));
			SparseMatrix& ZeroMat = *new SparseMatrix(M, N);
			res = &min(A, ZeroMat);
			delete &ZeroMat;
		}
	}
	return *res;
}

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
Matrix& min(double v, Matrix& A) {
	return min(A, v);
}

/**
 * Compute minimum between two real matrices A and B.
 *
 * @param A a real matrix
 *
 * @param B a real matrix
 *
 * @return max(A, B);
 */
Matrix& min(Matrix& A, Matrix& B) {
	Matrix* res = null;
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	double v = 0;
	if (typeid(A) == typeid(DenseMatrix)) {
		res = &A.copy();
		double** resData = ((DenseMatrix*) res)->getData();
		double* resRow = null;
		if (typeid(B) == typeid(DenseMatrix)) {
			double** BData = ((DenseMatrix&) B).getData();
			double* BRow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				BRow = BData[i];
				resRow = resData[i];
				for (int j = 0; j < N; j++) {
					v = BRow[j];
					if (resRow[j] > v)
						resRow[j] = v;
				}
			}
		} else {
			int* ic = ((SparseMatrix&) B).getIc();
			int* jr = ((SparseMatrix&) B).getJr();
			int* valCSRIndices = ((SparseMatrix&) B).getValCSRIndices();
			double* pr = ((SparseMatrix&) B).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					for (int j = 0; j < N; j++) {
						if (resRow[j] > 0)
							resRow[j] = 0;
					}
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						if (resRow[c] > 0)
							resRow[c] = 0;
					}
					resRow[currentColumnIdx] = min(pr[valCSRIndices[k]], resRow[currentColumnIdx]);
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					if (resRow[c] > 0)
						resRow[c] = 0;
				}
			}
		}
	} else {
		if (typeid(B) == typeid(DenseMatrix)) {
			return min(B, A);
		} else {

			res = new SparseMatrix(M, N);
			int* ir1 = null;
			int* jc1 = null;
			double* pr1 = null;
			ir1 = ((SparseMatrix&) A).getIr();
			jc1 = ((SparseMatrix&) A).getJc();
			pr1 = ((SparseMatrix&) A).getPr();
			int* ir2 = null;
			int* jc2 = null;
			double* pr2 = null;
			ir2 = ((SparseMatrix&) B).getIr();
			jc2 = ((SparseMatrix&) B).getJc();
			pr2 = ((SparseMatrix&) B).getPr();

			int k1 = 0;
			int k2 = 0;
			int r1 = -1;
			int r2 = -1;
			int i = -1;
			// double v = 0;

			for (int j = 0; j < N; j++) {
				k1 = jc1[j];
				k2 = jc2[j];

				// Both A and B's j-th columns are empty.
				if (k1 == jc1[j + 1] && k2 == jc2[j + 1])
					continue;

				while (k1 < jc1[j + 1] || k2 < jc2[j + 1]) {

					if (k2 == jc2[j + 1]) { // B's j-th column has been processed.
						i = ir1[k1];
						v = pr1[k1];
						if (v > 0)
							v = 0;
						k1++;
					} else if (k1 == jc1[j + 1]) { // A's j-th column has been processed.
						i = ir2[k2];
						v = pr2[k2];
						if (v > 0)
							v = 0;
						k2++;
					} else { // Both A and B's j-th columns have not been fully processed.
						r1 = ir1[k1];
						r2 = ir2[k2];
						if (r1 < r2) {
							i = r1;
							v = pr1[k1];
							if (v > 0)
								v = 0;
							k1++;
						} else if (r1 == r2) {
							i = r1;
							v = min(pr1[k1], pr2[k2]);
							k1++;
							k2++;
						} else {
							i = r2;
							v = pr2[k2];
							if (v > 0)
								v = 0;
							k2++;
						}
					}
					if (v != 0)
						res->setEntry(i, j, v);
				}
			}
		}
	}

	return *res;

}

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
Vector& max(Vector& V, double v) {
	Vector* res = null;
	int dim = V.getDim();
	if (typeid(V) == typeid(DenseVector)) {
		res = &V.copy();
		double* pr = ((DenseVector*) res)->getPr();
		for (int k = 0; k < dim; k++) {
			if (pr[k] < v)
				pr[k] = v;
		}
	} else {
		if (v != 0) {
			res = new DenseVector(dim, v);
			double* prRes = ((DenseVector*) res)->getPr();
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = -1;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int idx = lastIdx + 1; idx < currentIdx; idx++) {
					if (v < 0)
						prRes[idx] = 0;
				}
				if (v < pr[k])
					prRes[currentIdx] = pr[k];
				lastIdx = currentIdx;
			}
			for (int idx = lastIdx + 1; idx < dim; idx++) {
				if (v < 0)
					prRes[idx] = 0;
			}
		} else { // v == 0
			res = &V.copy();
			double* pr = ((SparseVector*) res)->getPr();
			int nnz = ((SparseVector*) res)->getNNZ();
			for (int k = 0; k < nnz; k++) {
				if (pr[k] < 0)
					pr[k] = 0;
			}
			((SparseVector*) res)->clean();
		}
	}
	return *res;
}

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
Vector& max(double v, Vector& V) {
	return max(V, v);
}

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
Vector& max(Vector& U, Vector& V) {
	Vector* res = null;
	int dim = U.getDim();
	double v = 0;
	if (typeid(U) == typeid(DenseVector)) {
		res = &U.copy();
		double* prRes = ((DenseVector*) res)->getPr();
		if (typeid(V) == typeid(DenseVector)) {
			// double* prU = ((DenseVector&) U).getPr();
			double* prV = ((DenseVector&) V).getPr();
			for (int k = 0; k < dim; k++) {
				v = prV[k];
				if (prRes[k] < v)
					prRes[k] = v;
			}
		} else {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = -1;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int idx = lastIdx + 1; idx < currentIdx; idx++) {
					if (prRes[idx] < 0)
						prRes[idx] = 0;
				}
				v = pr[k];
				if (prRes[currentIdx] < v)
					prRes[currentIdx] = v;
				lastIdx = currentIdx;
			}
			for (int idx = lastIdx + 1; idx < dim; idx++) {
				if (prRes[idx] < 0)
					prRes[idx] = 0;
			}
		}
	} else {
		if (typeid(V) == typeid(DenseVector)) {
			return max(V, U);
		} else {
			res = new SparseVector(dim);
			int* ir1 = ((SparseVector&) V).getIr();
			double* pr1 = ((SparseVector&) V).getPr();
			int nnz1 = ((SparseVector&) V).getNNZ();
			int* ir2 = ((SparseVector&) U).getIr();
			double* pr2 = ((SparseVector&) U).getPr();
			int nnz2 = ((SparseVector&) U).getNNZ();
			if (!(nnz1 == 0 && nnz2 == 0)) {
				int k1 = 0;
				int k2 = 0;
				int r1 = 0;
				int r2 = 0;
				// double v = 0;
				int i = -1;
				while (k1 < nnz1 || k2 < nnz2) {
					if (k2 == nnz2) { // V has been processed.
						i = ir1[k1];
						v = pr1[k1];
						if (v < 0)
							v = 0;
						k1++;
					} else if (k1 == nnz1) { // this has been processed.
						i = ir2[k2];
						v = pr2[k2];
						if (v < 0)
							v = 0;
						k2++;
					} else { // Both this and V have not been fully processed.
						r1 = ir1[k1];
						r2 = ir2[k2];
						if (r1 < r2) {
							i = r1;
							v = pr1[k1];
							if (v < 0)
								v = 0;
							k1++;
						} else if (r1 == r2) {
							i = r1;
							v = max(pr1[k1], pr2[k2]);
							k1++;
							k2++;
						} else {
							i = r2;
							v = pr2[k2];
							if (v < 0)
								v = 0;
							k2++;
						}
					}
					if (v != 0) {
						res->set(i, v);
					}
				}
			}
		}
	}
	return *res;
}

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
Vector& min(Vector& V, double v) {
	Vector* res = null;
	int dim = V.getDim();
	if (typeid(V) == typeid(DenseVector)) {
		res = &V.copy();
		double* pr = ((DenseVector*) res)->getPr();
		for (int k = 0; k < dim; k++) {
			if (pr[k] > v)
				pr[k] = v;
		}
	} else {
		if (v != 0) {
			res = new DenseVector(dim, v);
			double* prRes = ((DenseVector*) res)->getPr();
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = -1;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int idx = lastIdx + 1; idx < currentIdx; idx++) {
					if (v > 0)
						prRes[idx] = 0;
				}
				if (v > pr[k])
					prRes[currentIdx] = pr[k];
				lastIdx = currentIdx;
			}
			for (int idx = lastIdx + 1; idx < dim; idx++) {
				if (v > 0)
					prRes[idx] = 0;
			}
		} else { // v == 0
			res = &V.copy();
			double* pr = ((SparseVector*) res)->getPr();
			int nnz = ((SparseVector*) res)->getNNZ();
			for (int k = 0; k < nnz; k++) {
				if (pr[k] > 0)
					pr[k] = 0;
			}
			((SparseVector*) res)->clean();
		}
	}
	return *res;
}

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
Vector& min(double v, Vector& V) {
	return min(V, v);
}

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
Vector& min(Vector& U, Vector& V) {
	Vector* res = null;
	int dim = U.getDim();
	double v = 0;
	if (typeid(U) == typeid(DenseVector)) {
		res = &U.copy();
		double* prRes = ((DenseVector*) res)->getPr();
		if (typeid(V) == typeid(DenseVector)) {
			// double* prU = ((DenseVector&) U).getPr();
			double* prV = ((DenseVector&) V).getPr();
			for (int k = 0; k < dim; k++) {
				v = prV[k];
				if (prRes[k] > v)
					prRes[k] = v;
			}
		} else {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = -1;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int idx = lastIdx + 1; idx < currentIdx; idx++) {
					if (prRes[idx] > 0)
						prRes[idx] = 0;
				}
				v = pr[k];
				if (prRes[currentIdx] > v)
					prRes[currentIdx] = v;
				lastIdx = currentIdx;
			}
			for (int idx = lastIdx + 1; idx < dim; idx++) {
				if (prRes[idx] > 0)
					prRes[idx] = 0;
			}
		}
	} else {
		if (typeid(V) == typeid(DenseVector)) {
			return max(V, U);
		} else {
			res = new SparseVector(dim);
			int* ir1 = ((SparseVector&) V).getIr();
			double* pr1 = ((SparseVector&) V).getPr();
			int nnz1 = ((SparseVector&) V).getNNZ();
			int* ir2 = ((SparseVector&) U).getIr();
			double* pr2 = ((SparseVector&) U).getPr();
			int nnz2 = ((SparseVector&) U).getNNZ();
			if (!(nnz1 == 0 && nnz2 == 0)) {
				int k1 = 0;
				int k2 = 0;
				int r1 = 0;
				int r2 = 0;
				// double v = 0;
				int i = -1;
				while (k1 < nnz1 || k2 < nnz2) {
					if (k2 == nnz2) { // V has been processed.
						i = ir1[k1];
						v = pr1[k1];
						if (v > 0)
							v = 0;
						k1++;
					} else if (k1 == nnz1) { // this has been processed.
						i = ir2[k2];
						v = pr2[k2];
						if (v > 0)
							v = 0;
						k2++;
					} else { // Both this and V have not been fully processed.
						r1 = ir1[k1];
						r2 = ir2[k2];
						if (r1 < r2) {
							i = r1;
							v = pr1[k1];
							if (v > 0)
								v = 0;
							k1++;
						} else if (r1 == r2) {
							i = r1;
							v = min(pr1[k1], pr2[k2]);
							k1++;
							k2++;
						} else {
							i = r2;
							v = pr2[k2];
							if (v > 0)
								v = 0;
							k2++;
						}
					}
					if (v != 0) {
						res->set(i, v);
					}
				}
			}
		}
	}
	return *res;
}

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
Matrix& logicalIndexing(Matrix& A, Matrix& B) {

	int nA = A.getColumnDimension();
	int dA = A.getRowDimension();
	int nB = B.getColumnDimension();
	int dB = B.getRowDimension();
	if (nA != nB || dA != dB) {
		err("The input matrices should have same size!");
		exit(1);
	}

	std::list<double> vals;

	double b;
	for (int j = 0; j < nA; j++) {
		for (int i = 0; i < dA; i++) {
			b = B.getEntry(i, j);
			if (b == 1)
				vals.push_back(A.getEntry(i, j));
			else if (b != 0)
				err("Elements of the logical matrix should be either 1 or 0!");
		}
	}

	double* data = new double[vals.size()];
	int k = 0;
	for (std::list<double>::iterator iter = vals.begin(); iter != vals.end(); iter++) {
		data[k++] = *iter;
	}
	// vals.toArray(Data);

	/*double* data = new double[vals.size()];
	for (int i = 0; i < vals.size(); i++) {
		data[i] = Data[i];
	}*/

	if (vals.size() != 0)
		return *new DenseMatrix(data, vals.size(), 1);
	else
		return *new SparseMatrix(0, 1);

}

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
int* linearIndexing(int* V, int* indices, int len) {

	if (indices == null || len == 0) {
		return null;
	}

	int* res = new int[len];
	for (int i = 0; i < len; i++) {
		res[i] = V[indices[i]];
	}

	return res;

}

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
double* linearIndexing(double* V, int* indices, int len) {

	if (indices == null || len == 0) {
		return null;
	}

	double* res = new double[len];
	for (int i = 0; i < len; i++) {
		res[i] = V[indices[i]];
	}

	return res;

}

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
Matrix& linearIndexing(Matrix& A, int* indices, int len) {

	if (indices == null || len == 0) {
		return *new SparseMatrix(0, 0);
	}

	Matrix* res = null;
	if (typeid(A) == typeid(DenseMatrix)) {
		res = new DenseMatrix(len, 1);
	} else {
		res = new SparseMatrix(len, 1);
	}

	int nRow = A.getRowDimension();
	// int nCol = A.getColumnDimension();
	int r = -1;
	int c = -1;
	int index = -1;
	for (int i = 0; i < len; i++) {
		index = indices[i];
		r = index % nRow;
		c = index / nRow;
		res->setEntry(i, 0, A.getEntry(r, c));
	}

	return *res;

}


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
void linearIndexingAssignment(Matrix& A, int* indices, int len, Matrix& V) {

	int nV = V.getColumnDimension();
	int dV = V.getRowDimension();

	if (nV != 1) {
		err("Assignment matrix should be a column matrix!");
		exit(1);
	}

	if (len != dV) {
		err("Assignment with different number of elements!");
		exit(1);
	}

	int nRow = A.getRowDimension();
	// int nCol = A.getColumnDimension();
	int r = -1;
	int c = -1;
	int index = -1;
	for (int i = 0; i < len; i++) {
		index = indices[i];
		r = index % nRow;
		c = index / nRow;
		A.setEntry(r, c, V.getEntry(i, 0));
	}

}

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
void linearIndexingAssignment(Matrix& A, int* indices, int len, double v) {

	int nRow = A.getRowDimension();
	// int nCol = A.getColumnDimension();
	int r = -1;
	int c = -1;
	int index = -1;
	for (int i = 0; i < len; i++) {
		index = indices[i];
		r = index % nRow;
		c = index / nRow;
		A.setEntry(r, c, v);
	}

}

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
void logicalIndexingAssignment(Matrix& A, Matrix& B, double v) {

	int nA = A.getColumnDimension();
	int dA = A.getRowDimension();
	int nB = B.getColumnDimension();
	int dB = B.getRowDimension();
	if (nA != nB || dA != dB) {
		err("The input matrices for logical indexing should have same size!");
		exit(1);
	}

	double b;
	if (typeid(B) == typeid(SparseMatrix)) {
		int* ir = ((SparseMatrix&) B).getIr();
		int* jc = ((SparseMatrix&) B).getJc();
		double* pr = ((SparseMatrix&) B).getPr();
		for (int j = 0; j < nB; j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				b = pr[k];
				if (b == 1)
					A.setEntry(ir[k], j, v);
				else if (b != 0)
					err("Elements of the logical matrix should be either 1 or 0!");
			}
		}
	} else {
		double** BData = ((DenseMatrix&) B).getData();
		double* BRow = null;
		for (int i = 0; i < dA; i++) {
			BRow = BData[i];
			for (int j = 0; j < nA; j++) {
				b = BRow[j];
				if (b == 1) {
					A.setEntry(i, j, v);
				}
				else if (b != 0)
					err("Elements of the logical matrix should be either 1 or 0!");
			}
		}
	}

}

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
void logicalIndexingAssignment(Matrix& A, Matrix& B, Matrix& V) {
	int nA = A.getColumnDimension();
	int dA = A.getRowDimension();
	int nB = B.getColumnDimension();
	int dB = B.getRowDimension();
	if (nA != nB || dA != dB) {
		err("The input matrices for logical indexing should have same size!");
		exit(1);
	}

	int nV = V.getColumnDimension();
	int dV = V.getRowDimension();

	if (nV != 1) {
		err("Assignment matrix should be a column matrix!");
		exit(1);
	}

	/*double b;
		int cnt = 0;
		for (int j = 0; j < nA; j++) {
			for (int i = 0; i < dA; i++) {
				b = B.getEntry(i, j);
				if (b == 1) {
					A.setEntry(i, j, V.getEntry(cnt++, 0));
				}
				else if (b != 0)
					err("Elements of the logical matrix should be either 1 or 0!");
			}
		}*/

	double b;
	int cnt = 0;
	if (typeid(B) == typeid(SparseMatrix)) {
		int* ir = ((SparseMatrix&) B).getIr();
		int* jc = ((SparseMatrix&) B).getJc();
		double* pr = ((SparseMatrix&) B).getPr();
		for (int j = 0; j < nB; j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				b = pr[k];
				if (b == 1)
					A.setEntry(ir[k], j, V.getEntry(cnt++, 0));
				else if (b != 0)
					err("Elements of the logical matrix should be either 1 or 0!");
			}
		}
	} else {
		double** BData = ((DenseMatrix&) B).getData();
		for (int j = 0; j < nA; j++) {
			for (int i = 0; i < dA; i++) {
				b = BData[i][j];
				if (b == 1) {
					A.setEntry(i, j, V.getEntry(cnt++, 0));
				}
				else if (b != 0)
					err("Elements of the logical matrix should be either 1 or 0!");
			}
		}
	}

	if (cnt != dV)
		err("Assignment with different number of elements!");

}

/**
 * Get the nonnegative part of a matrix A.
 *
 * @param A a matrix
 *
 * @return a matrix which is the nonnegative part of a matrix A
 *
 */
Matrix& subplus(Matrix& A) {
	Matrix& res = A.copy();
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				if (resRow[j] < 0)
					resRow[j] = 0;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			if (pr[k] < 0)
				pr[k] = 0;
		}
		((SparseMatrix&) res).clean();
	}
	return res;
}

/**
 * Returns an array that contains 1's where
 * the elements of X are NaN's and 0's where they are not.
 *
 * @param A a matrix
 *
 * @return a 0-1 matrix: isnan(A)
 *
 */
Matrix& isnan(Matrix& A) {
	Matrix& res = A.copy();
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				if (std::isnan(resRow[j]))
					resRow[j] = 1;
				else
					resRow[j] = 0;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			if (std::isnan(pr[k]))
				pr[k] = 1;
			else
				pr[k] = 0;
		}
		((SparseMatrix&) res).clean();
	}
	return res;
}

/**
 * returns an array that contains 1's where the
 * elements of X are +Inf or -Inf and 0's where they are not.
 *
 * @param A a real matrix
 *
 * @return a 0-1 matrix: isinf(A)
 *
 */
Matrix& isinf(Matrix& A) {
	Matrix& res = A.copy();
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				if (std::isinf(resRow[j]))
					resRow[j] = 1;
				else
					resRow[j] = 0;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			if (std::isinf(pr[k]))
				pr[k] = 1;
			else
				pr[k] = 0;
		}
		((SparseMatrix&) res).clean();
	}
	return res;
}

Matrix& minus(Matrix& A, double v) {
	return A.minus(v);
}

/**
 * res = v - A.
 * @param v
 * @param A
 * @return v - A
 */
Matrix& minus(double v, Matrix& A) {
	Matrix& uminusA = uminus(A);
	Matrix& res = uminusA.plus(v);
	delete &uminusA;
	return res;
	// return uminus(A).plus(v);
}



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
void setMatrix(Matrix& X, Matrix& Y) {
	assign(X, Y);
}

/**
 * Unary minus.
 *
 * @param A a matrix
 *
 * @return -A
 */
Matrix& uminus(Matrix& A) {
	return A.times(-1);
}

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
Matrix& _or(Matrix& A, Matrix& B) {
	Matrix& res = A.plus(B);
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				if (resRow[j] > 1)
					resRow[j] = 1;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			if (pr[k] > 1)
				pr[k] = 1;
		}
	}
	return res;
}

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
Matrix& _and(Matrix& A, Matrix& B) {
	Matrix& res = A.times(B);
	return res;
}

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
Matrix& _not(Matrix& A) {
	Matrix& res = minus(1, A);
	return res;
}

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
Matrix& ne(Matrix& X, Matrix& Y) {
	Matrix& res = X.minus(Y);
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				if (resRow[j] != 0)
					resRow[j] = 1;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			if (pr[k] != 0)
				pr[k] = 1;
		}
	}
	return res;
}

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
Matrix& ne(Matrix& X, double x) {
	Matrix& res = X.minus(x);
	// res must be a dense matrix.
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] != 0)
				resRow[j] = 1;
		}
	}
	return res;
}

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
Matrix& ne(double x, Matrix& X) {
	Matrix& res = X.minus(x);
	// res must be a dense matrix.
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] != 0)
				resRow[j] = 1;
		}
	}
	return res;
}

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
Matrix& eq(Matrix& X, Matrix& Y) {
	return minus(1, ne(X, Y));
}

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
Matrix& eq(Matrix& X, double x) {
	Matrix& res = X.minus(x);
	// res must be a dense matrix.
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] != 0)
				resRow[j] = 0;
			else
				resRow[j] = 1;
		}
	}
	return res;
}

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
Matrix& eq(double x, Matrix& X) {
	return eq(X, x);
}

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
Matrix& ge(Matrix& X, double x) {
	Matrix& res = X.minus(x);
	// res must be a dense matrix.
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] >= 0)
				resRow[j] = 1;
			else
				resRow[j] = 0;
		}
	}
	return res;
}

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
Matrix& ge(double x, Matrix& X) {
	Matrix& res = minus(x, X);
	// res must be a dense matrix.
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] >= 0)
				resRow[j] = 1;
			else
				resRow[j] = 0;
		}
	}
	return res;
}

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
Matrix& ge(Matrix& X, Matrix& Y) {
	Matrix& res = full(X.minus(Y));
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] >= 0)
				resRow[j] = 1;
			else
				resRow[j] = 0;
		}
	}
	return res;
}

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
Matrix& le(Matrix& X, double x) {
	return ge(x, X);
}

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
Matrix& le(double x, Matrix& X) {
	return ge(X, x);
}

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
Matrix& le(Matrix& X, Matrix& Y) {
	return ge(Y, X);
}

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
Matrix& gt(Matrix& X, double x) {
	Matrix& res = X.minus(x);
	// res must be a dense matrix.
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] > 0)
				resRow[j] = 1;
			else
				resRow[j] = 0;
		}
	}
	return res;
}

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
Matrix& gt(double x, Matrix& X) {
	Matrix& res = minus(x, X);
	// res must be a dense matrix.
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] > 0)
				resRow[j] = 1;
			else
				resRow[j] = 0;
		}
	}
	return res;
}

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
Matrix& gt(Matrix& X, Matrix& Y) {
	Matrix& res = full(X.minus(Y));
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		for (int j = 0; j < N; j++) {
			if (resRow[j] > 0)
				resRow[j] = 1;
			else
				resRow[j] = 0;
		}
	}
	return res;
}

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
Matrix& lt(Matrix& X, double x) {
	return gt(x, X);
}

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
Matrix& lt(double x, Matrix& X) {
	return gt(X, x);
}

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
Matrix& lt(Matrix& X, Matrix& Y) {
	return gt(Y, X);
}

/**
 * Compute element-wise absolute value of all elements of matrix.
 *
 * @param A a matrix
 *
 * @return abs(A)
 */
Matrix& abs(Matrix& A) {

	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();

	Matrix& res = A.copy();

	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			for (int j = 0; j < nCol; j++) {
				resRow[j] = fabs(resRow[j]);
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		for (int k = 0; k < ((SparseMatrix&) res).getNNZ(); k++) {
			pr[k] = fabs(pr[k]);
		}
	}

	return res;

}

/**
 * Compute element-wise absolute value of all elements of a vector.
 *
 * @param V a vector
 *
 * @return abs(V)
 */
Vector& abs(Vector& V) {
	Vector& res = V.copy();
	if (typeid(res) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) res).getPr();
		for (int k = 0; k < res.getDim(); k++) {
			pr[k] = fabs(pr[k]);
		}
	} else {
		double* pr = ((SparseVector&) res).getPr();
		int nnz = ((SparseVector&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			pr[k] = fabs(pr[k]);
		}
	}
	return res;
}

/**
 * Compute element-wise exponentiation of a vector.
 *
 * @param A a real matrix
 *
 * @param p exponent
 *
 * @return A.^p
 */
Matrix& pow(Matrix& A, double p) {

	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();

	Matrix& res = A.copy();

	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			for (int j = 0; j < nCol; j++) {
				resRow[j] = pow(resRow[j], p);
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		for (int k = 0; k < ((SparseMatrix&) res).getNNZ(); k++) {
			pr[k] = pow(pr[k], p);
		}
	}

	return res;

}

/**
 * Compute element-wise exponentiation of a vector.
 *
 * @param V a real vector
 *
 * @param p exponent
 *
 * @return V.^p
 */
Vector& pow(Vector& V, double p) {
	Vector& res = V.copy();
	if (typeid(res) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) res).getPr();
		for (int k = 0; k < res.getDim(); k++) {
			pr[k] = pow(pr[k], p);
		}
	} else {
		double* pr = ((SparseVector&) res).getPr();
		int nnz = ((SparseVector&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			pr[k] = pow(pr[k], p);
		}
	}
	return res;
}

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
double* max(Vector& V) {
	double* res = new double[2];
	double maxVal = NEGATIVE_INFINITY;
	double maxIdx = -1;
	double v = 0;
	if (typeid(V) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) V).getPr();
		for (int k = 0; k < ((DenseVector&) V).getDim(); k++) {
			v = pr[k];
			if (maxVal < v) {
				maxVal = v;
				maxIdx = k;
			}
		}

	} else {
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz = ((SparseVector&) V).getNNZ();
		int dim = V.getDim();
		if (nnz == 0) {
			maxVal = 0;
			maxIdx = 0;
		} else {
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx;) {
					if (maxVal < 0) {
						maxVal = 0;
						maxIdx = i;
					}
					break;
				}
				v = pr[k];
				if (maxVal < v) {
					maxVal = v;
					maxIdx = currentIdx;
				}
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim;) {
				if (maxVal < 0) {
					maxVal = 0;
					maxIdx = i;
				}
				break;
			}
		}
	}
	res[0] = maxVal;
	res[1] = maxIdx;
	return res;
}

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
Vector** max(Matrix& A) {
	return max(A, 1);
}

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
double** max(double** A, int numRows, int numColumns, int dim) {
	double** res = new double*[2];
	double** AData = A;
	int M = numRows;
	int N = numColumns;
	double maxVal = 0;
	int maxIdx = -1;
	double v = 0;
	double* maxValues = null;
	maxValues = allocate1DArray(N, NEGATIVE_INFINITY);
	double* maxIndices = null;
	maxIndices = allocate1DArray(N, 0);

	double* ARow = null;

	if (dim == 1) {
		maxValues = allocate1DArray(N, NEGATIVE_INFINITY);
		maxIndices = allocate1DArray(N, 0);
		for (int i = 0; i < M; i++) {
			ARow = AData[i];
			for (int j = 0; j < N; j++) {
				v = ARow[j];
				if (maxValues[j] < v) {
					maxValues[j] = v;
					maxIndices[j] = i;
				}
			}
		}
	} else if (dim == 2) {
		maxValues = allocate1DArray(M, NEGATIVE_INFINITY);
		maxIndices = allocate1DArray(M, 0);
		for (int i = 0; i < M; i++) {
			ARow = AData[i];
			maxVal = ARow[0];
			maxIdx = 0;
			for (int j = 1; j < N; j++) {
				v = ARow[j];
				if (maxVal < v) {
					maxVal = v;
					maxIdx = j;
				}
			}
			maxValues[i] = maxVal;
			maxIndices[i] = maxIdx;
		}
	}

	res[0] = maxValues;
	res[1] = maxIndices;
	return res;
}

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
Vector** max(Matrix& A, int dim) {
	Vector** res = new Vector*[2];
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	double maxVal = 0;
	int maxIdx = -1;
	double v = 0;
	double* maxValues = null;
	maxValues = allocate1DArray(N, NEGATIVE_INFINITY);
	double* maxIndices = null;
	maxIndices = allocate1DArray(N, 0);
	int len = 0;

	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;

		if (dim == 1) {
			maxValues = allocate1DArray(N, NEGATIVE_INFINITY);
			maxIndices = allocate1DArray(N, 0);
			len = N;
			for (int i = 0; i < M; i++) {
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					v = ARow[j];
					if (maxValues[j] < v) {
						maxValues[j] = v;
						maxIndices[j] = i;
					}
				}
			}
		} else if (dim == 2) {
			maxValues = allocate1DArray(M, NEGATIVE_INFINITY);
			maxIndices = allocate1DArray(M, 0);
			len = M;
			for (int i = 0; i < M; i++) {
				ARow = AData[i];
				maxVal = ARow[0];
				maxIdx = 0;
				for (int j = 1; j < N; j++) {
					v = ARow[j];
					if (maxVal < v) {
						maxVal = v;
						maxIdx = j;
					}
				}
				maxValues[i] = maxVal;
				maxIndices[i] = maxIdx;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) A).getPr();
		if (dim == 1) {
			maxValues = allocate1DArray(N, NEGATIVE_INFINITY);
			maxIndices = allocate1DArray(N, 0);
			len = N;
			int* ir = ((SparseMatrix&) A).getIr();
			int* jc = ((SparseMatrix&) A).getJc();
			for (int j = 0; j < N; j++) {
				if (jc[j] == jc[j + 1]) {
					maxValues[j] = 0;
					maxIndices[j] = 0;
					continue;
				}
				maxVal = NEGATIVE_INFINITY;
				maxIdx = -1;
				int lastRowIdx = -1;
				int currentRowIdx = 0;
				for (int k = jc[j]; k < jc[j + 1]; k++) {
					currentRowIdx = ir[k];
					for (int r = lastRowIdx + 1; r < currentRowIdx;) {
						if (maxVal < 0) {
							maxVal = 0;
							maxIdx = r;
						}
						break;
					}
					v = pr[k];
					if (maxVal < v) {
						maxVal = v;
						maxIdx = ir[k];
					}
					lastRowIdx = currentRowIdx;
				}
				for (int r = lastRowIdx + 1; r < M;) {
					if (maxVal < 0) {
						maxVal = 0;
						maxIdx = r;
					}
					break;
				}
				maxValues[j] = maxVal;
				maxIndices[j] = maxIdx;
			}
		} else if (dim == 2) {
			maxValues = allocate1DArray(M, NEGATIVE_INFINITY);
			maxIndices = allocate1DArray(M, 0);
			len = M;
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			for (int i = 0; i < M; i++) {
				if (jr[i] ==  jr[i + 1]) {
					maxValues[i] = 0;
					maxIndices[i] = 0;
					continue;
				}
				maxVal = NEGATIVE_INFINITY;
				maxIdx = -1;
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx;) {
						if (maxVal < 0) {
							maxVal = 0;
							maxIdx = c;
						}
						break;
					}
					v = pr[valCSRIndices[k]];
					if (maxVal < v) {
						maxVal = v;
						maxIdx = ic[k];
					}
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N;) {
					if (maxVal < 0) {
						maxVal = 0;
						maxIdx = c;
					}
					break;
				}
				maxValues[i] = maxVal;
				maxIndices[i] = maxIdx;
			}
		}
	}
	res[0] = new DenseVector(maxValues, len);
	res[1] = new DenseVector(maxIndices, len);
	return res;
}

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
double* min(Vector& V) {
	double* res = new double[2];
	double minVal = POSITIVE_INFINITY;
	double minIdx = -1;
	double v = 0;
	if (typeid(V) == typeid(DenseVector)) {
		double* pr = ((DenseVector&) V).getPr();
		for (int k = 0; k < ((DenseVector&) V).getDim(); k++) {
			v = pr[k];
			if (minVal > v) {
				minVal = v;
				minIdx = k;
			}
		}

	} else {
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz = ((SparseVector&) V).getNNZ();
		int dim = V.getDim();
		if (nnz == 0) {
			minVal = 0;
			minIdx = 0;
		} else {
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx;) {
					if (minVal > 0) {
						minVal = 0;
						minIdx = i;
					}
					break;
				}
				v = pr[k];
				if (minVal > v) {
					minVal = v;
					minIdx = currentIdx;
				}
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim;) {
				if (minVal > 0) {
					minVal = 0;
					minIdx = i;
				}
				break;
			}
		}
	}
	res[0] = minVal;
	res[1] = minIdx;
	return res;
}

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
Vector** min(Matrix& A) {
	return min(A, 1);
}

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
Vector** min(Matrix& A, int dim) {
	Vector** res = new Vector*[2];
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	double minVal = 0;
	int minIdx = -1;
	double v = 0;
	double* minValues = null;
	minValues = allocate1DArray(N, POSITIVE_INFINITY);
	double* minIndices = null;
	minIndices = allocate1DArray(N, 0);
	int len = 0;

	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;

		if (dim == 1) {
			minValues = allocate1DArray(N, POSITIVE_INFINITY);
			minIndices = allocate1DArray(N, 0);
			len = N;
			for (int i = 0; i < M; i++) {
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					v = ARow[j];
					if (minValues[j] > v) {
						minValues[j] = v;
						minIndices[j] = i;
					}
				}
			}
		} else if (dim == 2) {
			minValues = allocate1DArray(M, POSITIVE_INFINITY);
			minIndices = allocate1DArray(M, 0);
			len = M;
			for (int i = 0; i < M; i++) {
				ARow = AData[i];
				minVal = ARow[0];
				minIdx = 0;
				for (int j = 1; j < N; j++) {
					v = ARow[j];
					if (minVal > v) {
						minVal = v;
						minIdx = j;
					}
				}
				minValues[i] = minVal;
				minIndices[i] = minIdx;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) A).getPr();
		if (dim == 1) {
			minValues = allocate1DArray(N, POSITIVE_INFINITY);
			minIndices = allocate1DArray(N, 0);
			len = N;
			int* ir = ((SparseMatrix&) A).getIr();
			int* jc = ((SparseMatrix&) A).getJc();
			for (int j = 0; j < N; j++) {
				if (jc[j] == jc[j + 1]) {
					minValues[j] = 0;
					minIndices[j] = 0;
					continue;
				}
				minVal = POSITIVE_INFINITY;
				minIdx = -1;
				int lastRowIdx = -1;
				int currentRowIdx = 0;
				for (int k = jc[j]; k < jc[j + 1]; k++) {
					currentRowIdx = ir[k];
					for (int r = lastRowIdx + 1; r < currentRowIdx;) {
						if (minVal > 0) {
							minVal = 0;
							minIdx = r;
						}
						break;
					}
					v = pr[k];
					if (minVal > v) {
						minVal = v;
						minIdx = ir[k];
					}
					lastRowIdx = currentRowIdx;
				}
				for (int r = lastRowIdx + 1; r < M;) {
					if (minVal > 0) {
						minVal = 0;
						minIdx = r;
					}
					break;
				}
				minValues[j] = minVal;
				minIndices[j] = minIdx;
			}
		} else if (dim == 2) {
			minValues = allocate1DArray(M, POSITIVE_INFINITY);
			minIndices = allocate1DArray(M, 0);
			len = M;
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			for (int i = 0; i < M; i++) {
				if (jr[i] ==  jr[i + 1]) {
					minValues[i] = 0;
					minIndices[i] = 0;
					continue;
				}
				minVal = POSITIVE_INFINITY;
				minIdx = -1;
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx;) {
						if (minVal > 0) {
							minVal = 0;
							minIdx = c;
						}
						break;
					}
					v = pr[valCSRIndices[k]];
					if (minVal > v) {
						minVal = v;
						minIdx = ic[k];
					}
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N;) {
					if (minVal > 0) {
						minVal = 0;
						minIdx = c;
					}
					break;
				}
				minValues[i] = minVal;
				minIndices[i] = minIdx;
			}
		}
	}
	res[0] = new DenseVector(minValues, len);
	res[1] = new DenseVector(minIndices, len);
	return res;
}

/**
 * Compute the Euclidean norm of a matrix.
 *
 * @param A a real matrix
 *
 * @return ||A||_2
 */
double norm(Matrix& A) {
	return norm(A, 2);
}

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
double norm(Matrix& A, double type) {

	double res = 0;

	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();

	if (nRow == 1) {
		if (std::isinf(type)) {
			Vector** maxARes = max(A, 2);
			double* maxRes = max(*maxARes[0]);
			res = maxRes[0];
			delete maxARes[0];
			delete maxARes[1];
			delete[] maxARes;
			delete[] maxRes;
			return res;
			// return max(*max(A, 2)[0])[0];
		} else if (type > 0) {
			Matrix& absA = abs(A);
			Matrix& powAbsA = pow(absA, type);
			double s = sumAll(powAbsA);
			res = pow(s, 1.0 / type);
			delete &absA;
			delete &powAbsA;
			return res;
			// return pow(sum(sum(pow(abs(A), type))), 1.0 / type);
		} else {
			err(sprintf("Error norm type: %f\n", type));
			exit(1);
		}
	}

	if (nCol == 1) {
		if (std::isinf(type)) {
			Vector** maxARes = max(A, 1);
			double* maxRes = max(*maxARes[0]);
			res = maxRes[0];
			delete maxARes[0];
			delete maxARes[1];
			delete[] maxARes;
			delete[] maxRes;
			return res;
			// return max(*max(A, 1)[0])[0];
		} else if (type > 0) {
			Matrix& absA = abs(A);
			Matrix& powAbsA = pow(absA, type);
			double s = sumAll(powAbsA);
			res = pow(s, 1.0 / type);
			delete &absA;
			delete &powAbsA;
			return res;
			// return pow(sum(sum(pow(abs(A), type))), 1.0 / type);
		} else {
			err(sprintf("Error norm type: %f\n", type));
			exit(1);
		}
	}

	if (type == 2) {
		Matrix& AT = A.transpose();
		Matrix& ATA = AT.mtimes(A);
		double* eigenvalues = EVD::computeEigenvalues(ATA);
		double eigenvalue = eigenvalues[0];
		delete[] eigenvalues;
		delete &AT;
		delete &ATA;
		res = eigenvalue <= 0 ? 0 : sqrt(eigenvalue);
		// res = SingularValueDecomposition.computeSingularValues(A)[0];
	} else if (std::isinf(type)) {
		Matrix& absA = abs(A);
		DenseVector& S = sum(absA, 2);
		double* maxRes = max(S);
		res = maxRes[0];
		delete &absA;
		delete &S;
		delete[] maxRes;
		// res = max(sum(abs(A), 2))[0];
	} else if (type == 1) {
		Matrix& absA = abs(A);
		DenseVector& S = sum(absA, 1);
		double* maxRes = max(S);
		res = maxRes[0];
		delete &absA;
		delete &S;
		delete[] maxRes;
		// res = max(sum(abs(A), 1))[0];
	} else {
		err(sprintf("Sorry, %f-norm of a matrix is not supported currently.\n", type));
		exit(1);
	}

	return res;

}

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
double norm(Matrix& A, int type) {
	return norm(A, (double)type);
}

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
double norm(Matrix& A, std::string type) {
	double res = 0;
	if (type == "fro") {
		res = sqrt(innerProduct(A, A));
	} else if (type == "inf") {
		res = norm(A, inf);
	} else if (type == "nuclear") {
		int m = A.getRowDimension();
		int n = A.getColumnDimension();
		int len = m >= n ? n : m;
		double* singularValues = SVD::computeSingularValues(A);
		res = sum(singularValues, len);
		delete[] singularValues;
		// res = sum(SVD::computeSingularValues(A), len);
	} else {
		errf("Norm %s unimplemented!\n" , type.c_str());
		exit(1);
	}
	return res;
}

/**
 * Compute the norm of a vector.
 *
 * @param V a real vector
 *
 * @param p a positive {@code double} value
 *
 * @return ||V||_p
 */
double norm(Vector& V, double p) {
	if (p == 1) {
		return sum(abs(V));
	} else if (std::isinf(p)) {
		Vector& absV = abs(V);
		double* maxRes = max(absV);
		double res = maxRes[0];
		delete &absV;
		delete[] maxRes;
		// return max(abs(V))[0];
		return res;
	} else if (p > 0) {
		// return pow(sum(pow(abs(V), p)), 1.0 / p);
		Vector& absV = abs(V);
		Vector& powAbsV = pow(absV, p);
		double s = sum(powAbsV);
		double res = pow(s, 1.0 / p);
		delete &absV;
		delete &powAbsV;
		return res;
	} else {
		err("Wrong argument for p");
		exit(1);
	}
	return -1;
}

/**
 * Compute the Euclidean norm of a vector.
 *
 * @param V a real vector
 *
 * @return ||V||_2
 */
double norm(Vector& V) {
	return norm(V, 2);
}

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
Matrix& sigmoid(Matrix& A) {
	DenseMatrix& res = typeid(A) == typeid(DenseMatrix) ? (DenseMatrix&) A.copy() : full(A);
	double** data = res.getData();
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	double* row_i = null;
	double old = 0;
	double current = 0;
	double max = 0;
	double sum = 0;
	double v = 0;
	for (int i = 0; i < M; i++) {
		row_i = data[i];
		old = row_i[0];
		current = 0;
		max = old;
		for (int j = 1; j < N; j++) {
			current = row_i[j];
			if (max < current)
				max = current;
			old = current;
		}
		sum = 0;
		for (int j = 0; j < N; j++) {
			v = exp(row_i[j] - max);
			sum += v;
			row_i[j] = v;
		}
		for (int j = 0; j < N; j++) {
			row_i[j] /= sum;
		}
	}
	// return *new DenseMatrix(data, M, N);
	return res;
}

/**
 * Compute the maximum of elements in a 1D {@code double} array.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 *
 * @return max(V)
 */
double max(double* V, int len) {
	// double max = max(V, 0, V.length - 1);
	double max = V[0];
	for (int i = 1; i < len; i++) {
		if (max < V[i])
			max = V[i];
	}
	return max;
}

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
double max(double* V, int start, int end) {
	if (start == end)
		return V[start];
	if (start == end - 1)
		return max(V[start], V[end]);

	int middle = (start + end) / 2;
	double leftMax = max(V, start, middle);
	double rightMax = max(V, middle + 1, end);
	return max(leftMax, rightMax);
}

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
Matrix& mldivide(Matrix& A, Matrix& B) {
	QR QRDecomp(A);
	Matrix& res = QRDecomp.solve(B);
	return res;
}

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
Matrix& mrdivide(Matrix& A, Matrix& B) {
	Matrix& BT = B.transpose();
	Matrix& AT = A.transpose();
	Matrix& XT = mldivide(BT, AT);
	Matrix& res = XT.transpose();
	// return mldivide(B.transpose(), A.transpose()).transpose();
	delete &BT;
	delete &AT;
	delete &XT;
	return res;
}

/**
 * Compute the rank of a matrix. The rank function provides
 * an estimate of the number of linearly independent rows or
 * columns of a matrix.
 *
 * @param A a matrix
 *
 * @return rank of the given matrix
 */
int rank(Matrix& A) {
	return SVD::rank(A);
}

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
DenseMatrix& eye(int m, int n) {
	double** res = allocate2DArray(m, n, 0);
	int len = m >= n ? n : m;
	for (int i = 0; i < len; i++) {
		res[i][i] = 1;
	}
	return *new DenseMatrix(res, m, n);
}

/**
 * Construct an n-by-n dense identity matrix.
 *
 * @param n number of rows and columns
 *
 * @return an n-by-n dense identity matrix
 *
 */
DenseMatrix& eye(int n) {
	return eye(n, n);
}

/**
 * Generate a dense identity matrix with its size
 * specified by a two dimensional integer array.
 *
 * @param size a two dimensional integer array
 *
 * @return a dense identity matrix with its shape specified by size
 *
 */
Matrix& eye(int* size) {
	return eye(size[0], size[1]);
}

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
SparseMatrix& speye(int m, int n) {
	SparseMatrix& res = *new SparseMatrix(m, n);
	int len = m >= n ? n : m;
	for (int i = 0; i < len; i++) {
		res.setEntry(i, i, 1.0);
	}
	return res;
}

/**
 * Construct an n-by-n sparse identity matrix.
 *
 * @param n number of rows and columns
 *
 * @return an n-by-n sparse identity matrix
 *
 */
SparseMatrix& speye(int n) {
	return speye(n, n);
}

/**
 * Generate a sparse identity matrix with its size
 * specified by a two dimensional integer array.
 *
 * @param size a two dimensional integer array
 *
 * @return a sparse identity matrix with its shape specified by size
 *
 */
Matrix& speye(int* size) {
	return speye(size[0], size[1]);
}

Matrix& hilb(int m, int n) {
	DenseMatrix& A = *new DenseMatrix(m, n);
	double** data = A.getData();
	double* A_i = null;
	for (int i = 0; i < m; i++) {
		A_i = data[i];
		for (int j = 0; j < n; j++) {
			A_i[j] = 1.0 / (i + j + 1);
		}
	}
	return A;
}

Vector& times(Vector& V1, Vector& V2) {
	return V1.times(V2);
}

Vector& plus(Vector& V1, Vector& V2) {
	return V1.plus(V2);
}

Vector& minus(Vector& V1, Vector& V2) {
	return V1.minus(V2);
}

Matrix& times(Matrix& A, Matrix& B) {
	return A.times(B);
}

Matrix& mtimes(Matrix& M1, Matrix& M2) {
	return M1.mtimes(M2);
}

Matrix& plus(Matrix& A, Matrix& B) {
	return A.plus(B);
}

Matrix& minus(Matrix& A, Matrix& B) {
	return A.minus(B);
}

Matrix& times(Matrix& A, double v) {
	return A.times(v);
}

Matrix& times(double v, Matrix& A) {
	return A.times(v);
}

Matrix& plus(Matrix& A, double v) {
	return A.plus(v);
}

Matrix& plus(double v, Matrix& A) {
	return A.plus(v);
}

DenseVector& full(Vector& V) {
	if (typeid(V) == typeid(SparseVector)) {
		int dim = V.getDim();
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		double* values = new double[dim];
		for (int k = 0; k < dim; k++) {
			values[k] = 0;
		}
		for (int k = 0; k < ((SparseVector&) V).getNNZ(); k++) {
			values[ir[k]] = pr[k];
		}
		return *new DenseVector(values, dim);
	} else {
		return (DenseVector&) V;
	}
}

SparseVector& sparse(Vector& V) {
	if (typeid(V) == typeid(DenseVector)) {
		double* values = ((DenseVector&) V).getPr();
		int dim = V.getDim();
		std::map<int, double> map;
		for (int k = 0; k < dim; k++) {
			if (values[k] != 0) {
				map.insert(std::make_pair(k, values[k]));
			}
		}
		int nnz = map.size();
		int* ir = new int[nnz];
		double* pr = new double[nnz];
		int ind = 0;
		for (std::map<int, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
			ir[ind] = iter->first;
			pr[ind] = iter->second;
			ind++;
		}
		return *new SparseVector(ir, pr, nnz, dim);
	} else {
		return (SparseVector&) V;
	}
}

DenseMatrix& full(Matrix& S) {
	if (typeid(S) == typeid(SparseMatrix)) {
		int M = S.getRowDimension();
		int N = S.getColumnDimension();
		double** data = new double*[M];
		int* ic = ((SparseMatrix&) S).getIc();
		int* jr = ((SparseMatrix&) S).getJr();
		int* valCSRIndices = ((SparseMatrix&) S).getValCSRIndices();
		double* pr = ((SparseMatrix&) S).getPr();
		for (int i = 0; i < M; i++) {
			double* rowData = new double[N];
			for (int k = 0; k < N; k++) {
				rowData[k] = 0;
			}
			for (int k = jr[i]; k < jr[i + 1]; k++) {
				rowData[ic[k]] = pr[valCSRIndices[k]];
			}
			data[i] = rowData;
		}
		return *new DenseMatrix(data, M, N);
	} else {
		return (DenseMatrix&) S;
	}
}

SparseMatrix& sparse(Matrix& A) {
	if (typeid(A) == typeid(DenseMatrix)) {
		int rIdx = 0;
		int cIdx = 0;
		int nzmax = 0;
		double value = 0;
		std::map<std::pair<int, int>, double> map;
		int numRows = A.getRowDimension();
		int numColumns = A.getColumnDimension();
		double** data = ((DenseMatrix&) A).getData();
		for (int j = 0; j < numColumns; j++) {
			cIdx = j;
			for (int i = 0; i < numRows; i++) {
				rIdx = i;
				value = data[i][j];
				if (value != 0) {
					map.insert(std::make_pair(std::make_pair(cIdx, rIdx), value));
					nzmax++;
				}
			}
		}
		int* ir = new int[nzmax];
		int* jc = new int[numColumns + 1];
		double* pr = new double[nzmax];
		int k = 0;
		jc[0] = 0;
		int currentColumn = 0;
		for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
			rIdx = iter->first.second;
			cIdx = iter->first.first;
			pr[k] = iter->second;
			ir[k] = rIdx;
			if (currentColumn < cIdx) {
				jc[currentColumn + 1] = k;
				currentColumn++;
			}
			k++;
		}
		while (currentColumn < numColumns) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		// jc[numColumns] = k;
		return SparseMatrix::createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);
	} else {
		return (SparseMatrix&) A;
	}
}

Vector* sparseMatrix2SparseRowVectors0(Matrix& S) {
	if (typeid(S) != typeid(SparseMatrix)) {
		err("SparseMatrix input is expected.");
		exit(1);
	}
	int M = S.getRowDimension();
	int N = S.getColumnDimension();
	Vector* Vs = new SparseVector[M];
	int* ic = ((SparseMatrix&) S).getIc();
	int* jr = ((SparseMatrix&) S).getJr();
	double* pr = ((SparseMatrix&) S).getPr();
	int* valCSRIndices = ((SparseMatrix&) S).getValCSRIndices();
	int* indices = null;
	double* values = null;
	int nnz = 0;
	int dim = N;
	for (int r = 0; r < M; r++) {
		nnz = jr[r + 1] - jr[r];
		indices = new int[nnz];
		values = new double[nnz];
		int idx = 0;
		for (int k = jr[r]; k < jr[r + 1]; k++) {
			indices[idx] = ic[k];
			values[idx] = pr[valCSRIndices[k]];
			idx++;
		}
		// SparseVector& temp = *new SparseVector(indices, values, nnz, dim);
		((SparseVector*) Vs)[r].setSparseVector(*new SparseVector(indices, values, nnz, dim));
		// Vs[r].setSparseVector(*new SparseVector(indices, values, nnz, dim));
	}
	return Vs;
}

Vector* sparseMatrix2SparseColumnVectors0(Matrix& S) {
	if (typeid(S) != typeid(SparseMatrix)) {
		err("SparseMatrix input is expected.");
		exit(1);
	}
	int M = S.getRowDimension();
	int N = S.getColumnDimension();
	Vector* Vs = new SparseVector[N];
	int* ir = ((SparseMatrix&) S).getIr();
	int* jc = ((SparseMatrix&) S).getJc();
	double* pr = ((SparseMatrix&) S).getPr();
	int* indices = null;
	double* values = null;
	int nnz = 0;
	int dim = M;
	for (int c = 0; c < N; c++) {
		nnz = jc[c + 1] - jc[c];
		indices = new int[nnz];
		values = new double[nnz];
		int idx = 0;
		for (int k = jc[c]; k < jc[c + 1]; k++) {
			indices[idx] = ir[k];
			values[idx] = pr[k];
			idx++;
		}
		((SparseVector*) Vs)[c].setSparseVector(*new SparseVector(indices, values, nnz, dim));
	}
	return Vs;
}

Matrix& sparseRowVectors2SparseMatrix0(Vector* Vs, int numRows) {
	if (typeid(Vs[0]) != typeid(SparseVector)) {
		disp("Vs should point to an array of sparse vectors.");
		exit(1);
	}
	int nnz = 0;
	int numColumns = Vs[0].getDim();
	for (int i = 0; i < numRows; i++) {
		if (typeid(((SparseVector*) Vs)[i]) != typeid(SparseVector)) {
			fprintf("Vs[%d] should be a sparse vector.\n", i);
			exit(1);
		}
		nnz += ((SparseVector*) Vs)[i].getNNZ();
		if (numColumns != ((SparseVector*) Vs)[i].getDim()) {
			fprintf("Vs[%d]'s dimension doesn't match.\n", i);
			exit(1);
		}
	}

	// int numRows = Vs.length;
	int nzmax = nnz;
	int* ic = new int[nzmax];
	int* jr = new int[numRows + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int cnt = 0;
	jr[0] = 0;
	int currentRow = 0;
	for (int i = 0; i < numRows; i++) {
		int* indices = ((SparseVector*) Vs)[i].getIr();
		double* values = ((SparseVector*) Vs)[i].getPr();
		nnz = ((SparseVector*) Vs)[i].getNNZ();
		for (int k = 0; k < nnz; k++) {
			cIdx = indices[k];
			rIdx = i;
			pr[cnt] = values[k];
			ic[cnt] = cIdx;
			while (currentRow < rIdx) {
				jr[currentRow + 1] = cnt;
				currentRow++;
			}
			cnt++;
		}
	}
	while (currentRow < numRows) {
		jr[currentRow + 1] = cnt;
		currentRow++;
	}
	// jr[numColumns] = k;

	return SparseMatrix::createSparseMatrixByCSRArrays(ic, jr, pr, numRows, numColumns, nzmax);
}

Matrix& sparseColumnVectors2SparseMatrix0(Vector* Vs, int numColumns) {
	if (typeid(Vs[0]) != typeid(SparseVector)) {
		disp("Vs should point to an array of sparse vectors.");
		exit(1);
	}
	int nnz = 0;
	int numRows = Vs[0].getDim();
	for (int i = 0; i < numColumns; i++) {
		if (typeid(((SparseVector*) Vs)[i]) != typeid(SparseVector)) {
			fprintf("Vs[%d] should be a sparse vector.\n", i);
			exit(1);
		}
		nnz += ((SparseVector*) Vs)[i].getNNZ();
		if (numRows != ((SparseVector*) Vs)[i].getDim()) {
			fprintf("Vs[%d]'s dimension doesn't match.\n", i);
			exit(1);
		}
	}

	// int numColumns = Vs.length;
	int nzmax = nnz;
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (int c = 0; c < numColumns; c++) {
		int* indices = ((SparseVector*) Vs)[c].getIr();
		double* values = ((SparseVector*) Vs)[c].getPr();
		nnz = ((SparseVector*) Vs)[c].getNNZ();
		for (int r = 0; r < nnz; r++) {
			rIdx = indices[r];
			cIdx = c;
			pr[k] = values[r];
			ir[k] = rIdx;
			while (currentColumn < cIdx) {
				jc[currentColumn + 1] = k;
				currentColumn++;
			}
			k++;
		}
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	return SparseMatrix::createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);
}

Vector** sparseMatrix2SparseRowVectors(Matrix& S) {
	if (typeid(S) != typeid(SparseMatrix)) {
		err("SparseMatrix input is expected.");
		exit(1);
	}
	int M = S.getRowDimension();
	int N = S.getColumnDimension();
	Vector** Vs = new Vector*[M];
	int* ic = ((SparseMatrix&) S).getIc();
	int* jr = ((SparseMatrix&) S).getJr();
	double* pr = ((SparseMatrix&) S).getPr();
	int* valCSRIndices = ((SparseMatrix&) S).getValCSRIndices();
	int* indices = null;
	double* values = null;
	int nnz = 0;
	int dim = N;
	for (int r = 0; r < M; r++) {
		nnz = jr[r + 1] - jr[r];
		indices = new int[nnz];
		values = new double[nnz];
		int idx = 0;
		for (int k = jr[r]; k < jr[r + 1]; k++) {
			indices[idx] = ic[k];
			values[idx] = pr[valCSRIndices[k]];
			idx++;
		}
		// SparseVector& temp = *new SparseVector(indices, values, nnz, dim);
		// ((SparseVector*) Vs)[r].setSparseVector(*new SparseVector(indices, values, nnz, dim));
		// Vs[r].setSparseVector(*new SparseVector(indices, values, nnz, dim));
		Vs[r] = new SparseVector(indices, values, nnz, dim);
	}
	return Vs;
}

Vector** sparseMatrix2SparseColumnVectors(Matrix& S) {
	if (typeid(S) != typeid(SparseMatrix)) {
		err("SparseMatrix input is expected.");
		exit(1);
	}
	int M = S.getRowDimension();
	int N = S.getColumnDimension();
	Vector** Vs = new Vector*[N];
	int* ir = ((SparseMatrix&) S).getIr();
	int* jc = ((SparseMatrix&) S).getJc();
	double* pr = ((SparseMatrix&) S).getPr();
	int* indices = null;
	double* values = null;
	int nnz = 0;
	int dim = M;
	for (int c = 0; c < N; c++) {
		nnz = jc[c + 1] - jc[c];
		indices = new int[nnz];
		values = new double[nnz];
		int idx = 0;
		for (int k = jc[c]; k < jc[c + 1]; k++) {
			indices[idx] = ir[k];
			values[idx] = pr[k];
			idx++;
		}
		// ((SparseVector*) Vs)[c].setSparseVector(*new SparseVector(indices, values, nnz, dim));
		Vs[c] = new SparseVector(indices, values, nnz, dim);
	}
	return Vs;
}

Matrix& sparseRowVectors2SparseMatrix(Vector** Vs, int numRows) {
	/*if (typeid(Vs[0]) != typeid(SparseVector)) {
		disp("Vs should point to an array of sparse vectors.");
		exit(1);
	}*/
	int nnz = 0;
	int numColumns = Vs[0]->getDim();
	for (int i = 0; i < numRows; i++) {
		if (typeid(*Vs[i]) != typeid(SparseVector)) {
			fprintf("Vs[%d] should point to a sparse vector.\n", i);
			exit(1);
		}
		nnz += ((SparseVector*) Vs[i])->getNNZ();
		if (numColumns != Vs[i]->getDim()) {
			fprintf("*Vs[%d]'s dimension doesn't match.\n", i);
			exit(1);
		}
	}

	// int numRows = Vs.length;
	int nzmax = nnz;
	int* ic = new int[nzmax];
	int* jr = new int[numRows + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int cnt = 0;
	jr[0] = 0;
	int currentRow = 0;
	for (int i = 0; i < numRows; i++) {
		int* indices = ((SparseVector*) Vs[i])->getIr();
		double* values = ((SparseVector*) Vs[i])->getPr();
		nnz = ((SparseVector*) Vs[i])->getNNZ();
		for (int k = 0; k < nnz; k++) {
			cIdx = indices[k];
			rIdx = i;
			pr[cnt] = values[k];
			ic[cnt] = cIdx;
			while (currentRow < rIdx) {
				jr[currentRow + 1] = cnt;
				currentRow++;
			}
			cnt++;
		}
	}
	while (currentRow < numRows) {
		jr[currentRow + 1] = cnt;
		currentRow++;
	}
	// jr[numColumns] = k;

	return SparseMatrix::createSparseMatrixByCSRArrays(ic, jr, pr, numRows, numColumns, nzmax);
}

Matrix& sparseColumnVectors2SparseMatrix(Vector** Vs, int numColumns) {
	/*if (typeid(Vs[0]) != typeid(SparseVector)) {
		disp("Vs should point to an array of sparse vectors.");
		exit(1);
	}*/
	int nnz = 0;
	int numRows = Vs[0]->getDim();
	for (int i = 0; i < numColumns; i++) {
		if (typeid(*Vs[i]) != typeid(SparseVector)) {
			fprintf("Vs[%d] should point to a sparse vector.\n", i);
			exit(1);
		}
		nnz += ((SparseVector*) Vs[i])->getNNZ();
		if (numRows != Vs[i]->getDim()) {
			fprintf("*Vs[%d]'s dimension doesn't match.\n", i);
			exit(1);
		}
	}

	// int numColumns = Vs.length;
	int nzmax = nnz;
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (int c = 0; c < numColumns; c++) {
		int* indices = ((SparseVector*) Vs[c])->getIr();
		double* values = ((SparseVector*) Vs[c])->getPr();
		nnz = ((SparseVector*) Vs[c])->getNNZ();
		for (int r = 0; r < nnz; r++) {
			rIdx = indices[r];
			cIdx = c;
			pr[k] = values[r];
			ir[k] = rIdx;
			while (currentColumn < cIdx) {
				jc[currentColumn + 1] = k;
				currentColumn++;
			}
			k++;
		}
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	return SparseMatrix::createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);
}

Matrix& rowVector2RowMatrix(Vector& V) {
	Matrix* res = null;
	if (typeid(V) == typeid(DenseVector)) {
		Vector** denseRowVectors = new Vector*[1];
		denseRowVectors[0] = &V;
		res = &denseRowVectors2DenseMatrix(denseRowVectors, 1);
	} else {
		Vector** sparseRowVectors = new Vector*[1];
		sparseRowVectors[0] = &V;
		res = &sparseRowVectors2SparseMatrix(sparseRowVectors, 1);
	}
	return *res;
}

Matrix& columnVector2ColumnMatrix(Vector& V) {
	Matrix* res = null;
	if (typeid(V) == typeid(DenseVector)) {
		Vector** denseColumnVectors = new Vector*[1];
		denseColumnVectors[0] = &V;
		res = &denseColumnVectors2DenseMatrix(denseColumnVectors, 1);
	} else {
		Vector** sparseColumnVectors = new Vector*[1];
		sparseColumnVectors[0] = &V;
		res = &sparseColumnVectors2SparseMatrix(sparseColumnVectors, 1);
	}
	return *res;
}

Matrix& denseRowVectors2DenseMatrix(Vector** Vs, int numRows) {
	int M = numRows;
	int N = Vs[0]->getDim();
	double** resData = new double*[M];
	for (int i = 0; i < M; i++) {
		// resData[i] = ((DenseVector&) *Vs[i]).getPr().clone();
		resData[i] = clone(((DenseVector&) *Vs[i]).getPr(), N);
	}
	return *new DenseMatrix(resData, M, N);
}

Matrix& denseColumnVectors2DenseMatrix(Vector** Vs, int numColumns) {
	int N = numColumns;
	int M = Vs[0]->getDim();
	double** resData = new double*[M];
	for (int i = 0; i < M; i++) {
		resData[i] = new double[N];
	}
	for (int j = 0; j < N; j++) {
		// double* column = ((DenseVector) Vs[j]).getPr().clone();
		double* column = clone(((DenseVector&) *Vs[j]).getPr(), M);
		for (int i = 0; i < M; i++) {
			resData[i][j] = column[i];
		}
	}
	return *new DenseMatrix(resData, M, N);
}

Vector** denseMatrix2DenseRowVectors(Matrix& A) {
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	Vector** res = new Vector*[M];
	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		for (int i = 0; i < M; i++) {
			res[i] = new DenseVector(AData[i], N);
		}
	} else {
		err("The input matrix should be a dense matrix.");
		exit(1);
	}
	return res;
}

Vector** denseMatrix2DenseColumnVectors(Matrix& A) {
	int N = A.getColumnDimension();
	int M = A.getRowDimension();
	Vector** res = new Vector*[N];
	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		for (int j = 0; j < N; j++) {
			double* column = new double[M];
			for (int i = 0; i < M; i++) {
				column[i] = AData[i][j];
			}
			res[j] = new DenseVector(column, M);
		}
	} else {
		err("The input matrix should be a dense matrix.");
		exit(1);
	}
	return res;
}
