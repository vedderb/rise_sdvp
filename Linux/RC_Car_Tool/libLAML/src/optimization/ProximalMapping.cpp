/*
 * ProximalMapping.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Mingjie Qian
 */

#include "ProximalMapping.h"
#include "InPlaceOperator.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Printer.h"
#include "ArrayOperator.h"
#include "Matlab.h"

/**
 * res = prox_th(X) where h = 0.
 *
 * @param res result matrix
 *
 * @param t a real scalar
 *
 * @param X a real matrix
 */
void Prox::compute(Matrix& res, double t, Matrix& X) {
	assign(res, X);
}

/**
 * res = prox_th(X) where h = || X ||_1.
 *
 * @param res result matrix
 *
 * @param t a nonnegative real scalar
 *
 * @param X a real matrix
 */
void ProxL1::compute(Matrix& res, double t, Matrix& X) {
	assign(res, X);
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double v = 0;
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				v = resRow[j];
				if (v > t)
					resRow[j] = v - t;
				else if (v < -t) {
					resRow[j] = v + t;
				} else
					resRow[j] = 0;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			v = pr[k];
			if (v > t) {
				pr[k] = v - t;
			} else if (v < -t) {
				pr[k] = v + t;
			} else {
				pr[k] = 0;
			}
		}
		((SparseMatrix&) res).clean();
	}
}

/**
 * Compute prox_th(X) where h = || X ||_F. For a
 * vector, h(X) is the l_2 norm of X, for a matrix
 * h(X) is the Frobenius norm of X.
 *
 * @param res result matrix
 *
 * @param t a nonnegative real scalar
 *
 * @param X a real column matrix
 */
void ProxL2::compute(Matrix& res, double t, Matrix& X) {
	double n = norm(X, "fro");
	if (n <= t) {
		res.clear();
	} else {
		times(res, 1 - t / n, X);
	}
}

/**
 * Compute prox_th(X) where h = || X ||_{\infty}.
 *
 * @param res result matrix
 *
 * @param t a nonnegative real scalar
 *
 * @param X a real column matrix
 */
void ProxLInfinity::compute(Matrix& res, double t, Matrix& X) {
	if (t < 0) {
		err("The first input should be a nonnegative real scalar.");
		exit(-1);
	}

	if (X.getColumnDimension() > 1) {
		err("The second input should be a vector.");
		exit(-1);
	}

	Matrix& U = X;
	Matrix& V0 = abs(X);
	if (sum(sum(V0)) <= t) {
		// res = zeros(size(V0));
		res.clear();
	}
	int d = size(X, 1);
	Matrix** sortRes = sort(V0);
	Matrix& V = *sortRes[0];
	delete sortRes[1];
	delete[] sortRes;
	delete &V0;
	double* Delta = new double[d - 1];
	for (int k = 0; k < d - 1; k++) {
		Delta[k] = V.getEntry(k + 1, 0) - V.getEntry(k, 0);
	}
	double* S = times(Delta, colon(d - 1.0, -1.0, 1.0), d - 1);

	double a = V.getEntry(d - 1, 0);
	double n = 1;
	double sum = S[d - 2];
	for (int j = d - 1; j >= 1; j--) {
		if (sum < t) {
			if (j > 1) {
				sum += S[j - 2];
			}
			a += V.getEntry(j - 1, 0);
			n++;
		} else {
			break;
		}
	}
	double alpha = (a - t) / n;
	delete &V;
	delete[] Delta;
	delete[] S;

	Matrix& signU = sign(U);
	Matrix& absU = abs(U);
	Matrix& minRes = min(alpha, absU);
	times(res, signU, minRes);
	delete &signU;
	delete &absU;
	delete &minRes;
}

/**
 * Compute prox_th(X) where h = I_+(X).
 *
 * @param res result matrix
 *
 * @param t a real scalar
 *
 * @param X a real matrix
 */
void ProxPlus::compute(Matrix& res, double t, Matrix& X) {
	subplus(res, X);
}

/**
 * Soft-thresholding (shrinkage) operator, which is defined as
 * S_{t}[x] = argmin_u 1/2 * || u - x ||^2 + t||u||_1</br>
 * which is actually prox_{t||.||_1}(x). The analytical form is</br>
 * S_{t}[x] =</br>
 * | x - t, if x > t</br>
 * | x + t, if x < -t</br>
 * | 0, otherwise</br>
 *
 * @param res result matrix
 *
 * @param t threshold
 *
 * @param X a real matrix
 */
void ShrinkageOperator::shrinkage(Matrix& res, double t, Matrix& X) {
	assign(res, X);
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double v = 0;
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				v = resRow[j];
				if (v > t)
					resRow[j] = v - t;
				else if (v < -t) {
					resRow[j] = v + t;
				} else
					resRow[j] = 0;
			}
		}
	} else {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			v = pr[k];
			if (v > t) {
				pr[k] = v - t;
			} else if (v < -t) {
				pr[k] = v + t;
			} else {
				pr[k] = 0;
			}
		}
		((SparseMatrix&) res).clean();
	}
}
