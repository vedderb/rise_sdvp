/*
 * LogisticRegression.cpp
 *
 *  Created on: Feb 22, 2014
 *      Author: Mingjie Qian
 */

#include "LogisticRegression.h"
#include "Matrix.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Utility.h"
#include "Matlab.h"
#include "InPlaceOperator.h"
#include "ArrayOperator.h"
#include "LBFGS.h"
#include "Printer.h"

LogisticRegression::LogisticRegression() {

}

LogisticRegression::LogisticRegression(double epsilon) {
	this->epsilon = epsilon;
}

void LogisticRegression::loadModel(std::string filePath) {

}

void LogisticRegression::saveModel(std::string filePath) {

}

void LogisticRegression::train() {

	double fval = 0;

	/* Minimize the cross-entropy error function defined by
	 * E (W) = -ln p (T|w1,w2,...,wK) / nSample
	 * Gradient: G = X * (V - Y) / nSample
	 */
	// W = repmat(zeros(nFea, 1), new int[]{1, K});
	DenseMatrix& W = *new DenseMatrix(nFeature + 1, nClass, 0);
	Matrix& A = *new DenseMatrix(nExample, nClass, 0);
	// A = X.transpose().multiply(W);
	computeActivation(A, *X, W);
	Matrix& V = sigmoid(A);
	Matrix& G = W.copy();

	// G = X.multiply(V.subtract(Y)).scalarMultiply(1.0 / nSample);
	Matrix&  VMinusY = *new DenseMatrix(nExample, nClass, 0);
	minus(VMinusY, V, *Y);
	// mtimes(G, XT, VMinusY);
	computeGradient(G, *X, VMinusY);
	timesAssign(G, 1.0 / nExample);


	// fval = -sum(sum(times(Y, log(plus(V, eps))))).getEntry(0, 0) / nSample;
	Matrix& YLogV = *new DenseMatrix(nExample, nClass);
	Matrix& VPlusEps = *new DenseMatrix(nExample, nClass);
	Matrix& LogV = *new DenseMatrix(nExample, nClass);
	plus(VPlusEps, V, eps);
	log(LogV, VPlusEps);
	times(YLogV, *Y, LogV);
	fval = -sum(sum(YLogV)) / nExample;

	bool* flags = new bool[2];
	while (true) {
		LBFGS::run(G, fval, epsilon, W, flags);
		/*disp("W:");
		disp(W);*/
		if (flags[0])
			break;
		// A = X.transpose().multiply(W);
		computeActivation(A, *X, W);
		/*disp("A:");
		disp(A);*/
		// V = sigmoid(A);
		sigmoid(V, A);
		// fval = -sum(sum(times(Y, log(plus(V, eps))))).getEntry(0, 0) / nSample;
		plus(VPlusEps, V, eps);
		/*disp("V:");
		disp(V);*/
		log(LogV, VPlusEps);
		times(YLogV, *Y, LogV);
		/*disp("YLogV:");
		disp(YLogV);*/
		fval = -sum(sum(YLogV)) / nExample;
		// fprintf("fval: %.4f\n", fval);
		if (flags[1]) {
			// G = rdivide(X.multiply(V.subtract(Y)), nSample);
			minus(VMinusY, V, *Y);
			// mtimes(G, XT, VMinusY);
			computeGradient(G, *X, VMinusY);
			timesAssign(G, 1.0 / nExample);
		}
	}
	double** WData = W.getData();
	double** thisWData = new double*[nFeature];
	for (int feaIdx = 0; feaIdx < nFeature; feaIdx++) {
		thisWData[feaIdx] = WData[feaIdx];
	}
	this->W = new DenseMatrix(thisWData, nFeature, nClass);
	b = WData[nFeature];

	// delete &W;
	delete &A;
	delete &V;
	delete &G;
	delete &VMinusY;
	delete &YLogV;
	delete &VPlusEps;
	delete &LogV;
	delete[] flags;

}

Matrix& LogisticRegression::predictLabelScoreMatrix(Matrix& Xt) {
	DenseMatrix& ScoreMatrix = (DenseMatrix&) Xt.mtimes(*W);
	double** scoreData = ScoreMatrix.getData();
	for (int i = 0; i < Xt.getRowDimension(); i++) {
		plusAssign(scoreData[i], b, nClass);
	}
	return sigmoid(ScoreMatrix);
}

void LogisticRegression::computeGradient(Matrix& res, Matrix& A, Matrix& B) {
	double** resData = ((DenseMatrix&) res).getData();
	double* resRow = null;
	// double* rowA = null;
	int NB = B.getColumnDimension();
	int N = A.getRowDimension();
	int M = A.getColumnDimension();
	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		if (typeid(B) == typeid(DenseMatrix)) {

			double** BData = ((DenseMatrix&) B).getData();
			// double* columnB = new double[B.getRowDimension()];
			// double* columnA = new double[A.getRowDimension()];
			double* BRow = null;
			// double s = 0;
			double A_ki = 0;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				clear(resRow, NB);
				for (int k = 0; k < N; k++) {
					BRow = BData[k];
					A_ki = AData[k][i];
					for (int j = 0; j < NB; j++) {
						resRow[j] += A_ki * BRow[j];
					}
				}
			}
			resRow = resData[M];
			clear(resRow, NB);
			for (int k = 0; k < N; k++) {
				BRow = BData[k];
				for (int j = 0; j < NB; j++) {
					resRow[j] += BRow[j];
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
			double s = 0;
			double* columnA = new double[A.getRowDimension()];
			for (int i = 0; i < M; i++) {
				for (int t = 0; t < N; t++) {
					columnA[t] = AData[t][i];
				}
				resRow = resData[i];
				for (int j = 0; j < NB; j++) {
					s = 0;
					for (int k = jc[j]; k < jc[j + 1]; k++) {
						r = ir[k];
						// A[r][j] = pr[k]
						s += columnA[r] * pr[k];
					}
					resRow[j] = s;
				}
			}

			resRow = resData[M];
			for (int j = 0; j < NB; j++) {
				s = 0;
				for (int k = jc[j]; k < jc[j + 1]; k++) {
					// r = ir[k];
					// A[r][j] = pr[k]
					s += pr[k];
				}
				resRow[j] = s;
			}

		}
	} else {

		if (typeid(B) == typeid(DenseMatrix)) {
			int* ir = ((SparseMatrix&) A).getIr();
			int* jc = ((SparseMatrix&) A).getJc();
			double* pr = ((SparseMatrix&) A).getPr();
			// int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double** BData = ((DenseMatrix&) B).getData();
			double* BRow = null;
			int c = -1;
			double s = 0;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				for (int j = 0; j < NB; j++) {
					s = 0;
					for (int k = jc[i]; k < jc[i + 1]; k++) {
						c = ir[k];
						s += pr[k] * BData[c][j];
					}
					resRow[j] = s;
				}
			}
			resRow = resData[M];
			clear(resRow, NB);
			for (int k = 0; k < N; k++) {
				BRow = BData[k];
				for (int j = 0; j < NB; j++) {
					resRow[j] += BRow[j];
				}
			}
		} else {
			int* ir1 = ((SparseMatrix&) A).getIr();
			int* jc1 = ((SparseMatrix&) A).getJc();
			double* pr1 = ((SparseMatrix&) A).getPr();
			int* ir2 = ((SparseMatrix&) B).getIr();
			int* jc2 = ((SparseMatrix&) B).getJc();
			double* pr2 = ((SparseMatrix&) B).getPr();
			// rowIdx of the right sparse matrix
			int rr = -1;
			// colIdx of the left sparse matrix
			int cl = -1;
			double s = 0;
			int kl = 0;
			int kr = 0;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				for (int j = 0; j < NB; j++) {
					s = 0;
					kl = jc1[i];
					kr = jc2[j];
					while (true) {
						if (kl >= jc1[i + 1] || kr >= jc2[j + 1]) {
							break;
						}
						cl = ir1[kl];
						rr = ir2[kr];
						if (cl < rr) {
							kl++;
						} else if (cl > rr) {
							kr++;
						} else {
							s += pr1[kl] * pr2[kr];
							kl++;
							kr++;
						}
					}
					resRow[j] = s;
				}
			}
			resRow = resData[M];
			for (int j = 0; j < NB; j++) {
				s = 0;
				for (int k = jc2[j]; k < jc2[j + 1]; k++) {
					// r = ir[k];
					// A[r][j] = pr[k]
					s += pr2[k];
				}
				resRow[j] = s;
			}
		}
	}
}

void LogisticRegression::computeActivation(Matrix& A, Matrix& X, Matrix& W) {
	double** AData = ((DenseMatrix&) A).getData();
	double* ARow = null;
	double** WData = ((DenseMatrix&) W).getData();
	double* WColumn = new double[W.getRowDimension()];
	double* WRow = null;
	if (typeid(X) == typeid(DenseMatrix)) {
		double** XData = ((DenseMatrix&) X).getData();
		double* XRow = null;
		double s = 0;
		for (int j = 0; j < nClass; j++) {
			for (int r = 0; r < W.getRowDimension(); r++) {
				WColumn[r] = WData[r][j];
			}
			for (int i = 0; i < nExample; i++) {
				XRow = XData[i];
				s = 0;
				for (int k = 0; k < nFeature; k++) {
					s += XRow[k] * WColumn[k];
				}
				AData[i][j] = s + WColumn[nFeature];
			}
		}
	} else {
		int* ic = ((SparseMatrix&) X).getIc();
		int* jr = ((SparseMatrix&) X).getJr();
		int* valCSRIndices = ((SparseMatrix&) X).getValCSRIndices();
		double* pr = ((SparseMatrix&) X).getPr();
		int feaIdx = -1;
		double v = 0;
		for (int i = 0; i < nExample; i++) {
			ARow = AData[i];
			clear(ARow, nClass);
			for (int k = jr[i]; k < jr[i + 1]; k++) {
				feaIdx = ic[k];
				WRow = WData[feaIdx];
				v = pr[valCSRIndices[k]];
				for (int j = 0; j < nClass; j++) {
					ARow[j] += v * WRow[j];
				}
			}
			WRow = WData[nFeature];
			for (int j = 0; j < nClass; j++) {
				ARow[j] += WRow[j];
			}
		}
	}
}
