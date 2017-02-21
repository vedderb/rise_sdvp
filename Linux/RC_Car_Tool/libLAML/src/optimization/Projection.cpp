/*
 * Projection.cpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Mingjie Qian
 */

#include "Projection.h"
#include "InPlaceOperator.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Printer.h"
#include "ArrayOperator.h"
#include "Matlab.h"

/**
 * Compute proj_{tC}(X) where C = {X: || X ||_1 <= 1}.
 *
 * @param res result matrix
 *
 * @param t a nonnegative real scalar
 *
 * @param X a real matrix
 */
void ProjL1::compute(Matrix& res, double t, Matrix& X) {
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
		// res = zeros(size(V));
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
	minus(res, absU, alpha);
	subplusAssign(res);
	timesAssign(res, signU);
	delete &signU;
	delete &absU;
}

/**
 * Compute proj_{tC}(X) where C = {X: || X ||_2 <= 1}.
 *
 * @param res result matrix
 *
 * @param t a nonnegative real scalar
 *
 * @param X a real column matrix
 */
void ProjL2::compute(Matrix& res, double t, Matrix& X) {
	double n = norm(X, "fro");
	if (n <= t) {
		assign(res, X);
	} else {
		times(res, t / n, X);
	}
}

/**
 * Compute proj_{tC}(X) where C = {X: || X ||_{\infty} <= 1}.
 *
 * @param res result matrix
 *
 * @param t a nonnegative real scalar
 *
 * @param X a real column matrix
 */
void ProjLInfinity::compute(Matrix& res, double t, Matrix& X) {
	if (t < 0) {
		err("The first input should be a nonnegative real scalar.");
		exit(-1);
	}

	if (X.getColumnDimension() > 1) {
		err("The second input should be a vector.");
		exit(-1);
	}
	Matrix& signX = sign(X);
	Matrix& absX = abs(X);
	Matrix& minRes = min(absX, t);
	times(res, signX, minRes);
	delete &signX;
	delete &absX;
	delete &minRes;
}
