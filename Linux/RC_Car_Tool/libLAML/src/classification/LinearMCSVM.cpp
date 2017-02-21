/*
 * LinearMCSVM.cpp
 *
 *  Created on: Feb 21, 2014
 *      Author: Mingjie Qian
 */

#include "LinearMCSVM.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "ArrayOperator.h"
#include "Utility.h"
#include "Printer.h"
#include <typeinfo>
#include <cmath>
#include <fstream>

LinearMCSVM::LinearMCSVM() : Classifier() {
	C = 1.0;
	eps = 1e-2;
}

LinearMCSVM::LinearMCSVM(double C, double eps) : Classifier() {
	this->C = C;
	this->eps = eps;
}

std::ostream& operator<<(std::ostream& ost, const LinearMCSVM& svm) {
	Matrix* W = svm.W;
	int nFeature = W->getRowDimension();
	int nClass = W->getColumnDimension();
	ost << nFeature;
	ost << nClass;
	for (int i = 0; i < nFeature; i++) {
		for (int c = 0; c < nClass; c++)
			ost << W->getEntry(i, 0) << ' ';
		ost << "\n";
	}
	double* b = svm.b;
	for (int c = 0; c < nClass; c++)
		ost << b[c] << " ";
	return ost;
}

std::istream& operator>>(std::istream& ist, LinearMCSVM& svm) {
	int nFeature = 0;
	int nClass = 0;
	ist >> nFeature;
	ist >> nClass;
	Matrix* W = new DenseMatrix(nFeature, nClass);
	double v = 0;
	for (int i = 0; i < nFeature; i++) {
		for (int c = 0; c < nClass; c++) {
			ist >> v;
			W->setEntry(i, c, v);
		}
	}
	svm.W = W;
	double* b = new double[nClass];
	for (int c = 0; c < nClass; c++) {
		ist >> b[c];
		// b[c] = v;
	}
	svm.b = b;
	svm.nFeature = nFeature;
	svm.nClass = nClass;
	return ist;
}

void LinearMCSVM::loadModel(std::string filePath) {
	std::ifstream modelFileSteam(filePath.c_str());
	if (modelFileSteam.is_open()) {
		modelFileSteam >> *this;
		modelFileSteam.close();
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

void LinearMCSVM::saveModel(std::string filePath) {
	std::ofstream modelFileSteam(filePath.c_str());
	if (modelFileSteam.is_open()) {
		modelFileSteam << *this;
		modelFileSteam.close();
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

void LinearMCSVM::train() {
	double* pr_CSR = null;
	if (typeid(*X) == typeid(SparseMatrix)) {
		double* pr = ((SparseMatrix&) *X).getPr();
		int* valCSRIndices = ((SparseMatrix&) *X).getValCSRIndices();
		int nnz = ((SparseMatrix&) *X).getNNZ();
		pr_CSR = allocate1DArray(nnz);
		for (int k = 0; k < nnz; k++) {
			pr_CSR[k] = pr[valCSRIndices[k]];
		}
	}

	double** Ws = new double*[nClass];
	for (int c = 0; c < nClass; c++) {
		Ws[c] = allocateVector(nFeature + 1, 0);
	}
	double* Q = computeQ(*X, pr_CSR);

	double** Alpha = allocate2DArray(nExample, nClass, 0);

	int Np = nExample * nClass;

	double M = NEGATIVE_INFINITY;
	double m = POSITIVE_INFINITY;
	double Grad = 0;
	double alpha_old = 0;
	double alpha_new = 0;
	double PGrad = 0;
	int i, p, q;
	int C = nClass;
	int* y = labelIDs;
	double delta = 0;
	int cnt = 1;
	while (true) {
		M = NEGATIVE_INFINITY;
		m = POSITIVE_INFINITY;
		for (int k = 0; k < Np; k++) {
			q = k % C;
			i = (k - q) / C;
			p = y[i];
			if (q == p) {
				continue;
			}
			// G = K(i, :) * (Beta(:, p) - Beta(:, q)) - 1;
			// G = Xs{i}' * (W(:, p) - W(:, q)) - 1;
			// Grad = computeGradient(Xs[i], Ws[p], Ws[q]);
			// Grad = computeGradient(X, i, Ws[p], Ws[q]);
			Grad = computeGradient(*X, i, pr_CSR, Ws[p], Ws[q]);
			alpha_old = Alpha[i][q];
			if (alpha_old == 0) {
				PGrad = min(Grad, 0);
			} else if (alpha_old == C) {
				PGrad = max(Grad, 0);
			} else {
				PGrad = Grad;
			}
			M = max(M, PGrad);
			m = min(m, PGrad);
			if (PGrad != 0) {
				alpha_new = min(max(alpha_old - Grad / Q[i], 0), C);
				Alpha[i][q] = alpha_new;
				delta = alpha_new - alpha_old;
				// Beta[i][p] += delta;
				// Beta[i][q] -= delta;
				// W(:, p) = W(:, p) + (Alpha(i, q) - alpha) * Xs{i};
				// W(:, q) = W(:, q) - (Alpha(i, q) - alpha) * Xs{i};
				// updateW(Ws[p], Ws[q], delta, Xs[i]);
				updateW(Ws[p], Ws[q], delta, *X, i, pr_CSR);
			}

		}
		if (cnt % 20 == 0)
			fprintf(".");
		if (cnt % 400 == 0)
			fprintf("\n");
		cnt++;
		if (fabs(M - m) <= eps) {
			fprintf("\n");
			break;
		}

	}

	double** weights = new double*[nClass];
	b = new double[nClass];
	for (int c = 0; c < nClass; c++) {
		weights[c] = new double[nFeature];
		// System.arraycopy(Ws[c], 0, weights[c], 0, nFeature);
		std::copy(Ws[c], Ws[c] + nFeature, weights[c]);
		b[c] = Ws[c][nFeature];
	}
	this->W = &((new DenseMatrix(weights, nClass, nFeature))->transpose());
}

Matrix& LinearMCSVM::predictLabelScoreMatrix(Matrix& Xt) {
	int n = Xt.getRowDimension();
	Matrix* ScoreMatrix = null;
	Matrix& Bias = (*new DenseMatrix(n, 1, 1.0)).mtimes(*new DenseMatrix(b, nClass, 2));
	ScoreMatrix = &Xt.mtimes(*W).plus(Bias);
	return *ScoreMatrix;
}

double* LinearMCSVM::computeQ(Matrix& X, double* pr_CSR) {
	int l = X.getRowDimension();
	double* Q = new double[l];
	double s = 0;
	double v = 0;
	int M = X.getRowDimension();
	int N = X.getColumnDimension();
	if (typeid(X) == typeid(DenseMatrix)) {
		double** XData = ((DenseMatrix&) X).getData();
		double* XRow = null;
		for (int i = 0; i < M; i++) {
			XRow = XData[i];
			s = 1;
			for (int j = 0; j < N; j++) {
				v = XRow[j];
				s += v * v;
			}
			Q[i] = 2 * s;
		}
	} else {
		// int* ic = ((SparseMatrix&) X).getIc();
		int* jr = ((SparseMatrix&) X).getJr();
		// int* valCSRIndices = ((SparseMatrix&) X).getValCSRIndices();
		// double* pr = ((SparseMatrix&) X).getPr();
		for (int i = 0; i < M; i++) {
			s = 1;
			for (int k = jr[i]; k < jr[i + 1]; k++) {
				// v = pr[valCSRIndices[k]];
				v = pr_CSR[k];
				s += v * v;
			}
			Q[i] = 2 * s;
		}
	}
	return Q;
}

void LinearMCSVM::updateW(double* Wp, double* Wq, double delta, Matrix& X, int i, double* pr_CSR) {
	int N = X.getColumnDimension();
	double v = 0;
	if (typeid(X) == typeid(DenseMatrix)) {
		double** XData = ((DenseMatrix&) X).getData();
		double* XRow = null;
		XRow = XData[i];
		for (int j = 0; j < N; j++) {
			// W[j] += v * XRow[j];
			v = delta * XRow[j];
			Wp[j] += v;
			Wq[j] -= v;
		}
		// W[N] += v;
		Wp[N] += delta;
		Wq[N] -= delta;
	} else {
		int* ic = ((SparseMatrix&) X).getIc();
		int* jr = ((SparseMatrix&) X).getJr();
		// int* valCSRIndices = ((SparseMatrix&) X).getValCSRIndices();
		// double* pr = ((SparseMatrix&) X).getPr();
		int idx = 0;
		for (int k = jr[i]; k < jr[i + 1]; k++) {
			// W[ic[k]] += v * pr[valCSRIndices[k]];
			idx = ic[k];
			// v = delta * pr[valCSRIndices[k]];
			v = delta * pr_CSR[k];
			Wp[idx] += v;
			Wq[idx] -= v;
		}
		// W[N] += v;
		Wp[N] += delta;
		Wq[N] -= delta;
	}
}

double LinearMCSVM::computeGradient(Matrix& X, int i, double* pr_CSR, double* Wp, double* Wq) {
	int N = X.getColumnDimension();
	double res = 0;
	double s = -1;
	if (typeid(X) == typeid(DenseMatrix)) {
		double** XData = ((DenseMatrix&) X).getData();
		double* XRow = null;
		XRow = XData[i];
		for (int j = 0; j < N; j++) {
			s += (Wp[j] - Wq[j]) * XRow[j];
		}
		res = s + Wp[N] - Wq[N];
	} else {
		int* ic = ((SparseMatrix&) X).getIc();
		int* jr = ((SparseMatrix&) X).getJr();
		// int* valCSRIndices = ((SparseMatrix&) X).getValCSRIndices();
		// double* pr = ((SparseMatrix&) X).getPr();
		int idx = 0;
		for (int k = jr[i]; k < jr[i + 1]; k++) {
			idx = ic[k];
			// s += (Wp[idx] - Wq[idx]) * pr[valCSRIndices[k]];
			s += (Wp[idx] - Wq[idx]) * pr_CSR[k];
		}
		res = s + Wp[N] - Wq[N];
	}
	return res;
}



