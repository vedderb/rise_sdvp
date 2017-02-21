/*
 * LinearBinarySVM.cpp
 *
 *  Created on: Feb 16, 2014
 *      Author: Mingjie Qian
 */

#include "LinearBinarySVM.h"
#include "ArrayOperator.h"
#include <limits>
#include <cmath>
#include <fstream>
#include "Printer.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Utility.h"
#include "Matlab.h"

LinearBinarySVM::LinearBinarySVM() : Classifier() {
	C = 1.0;
	eps = 1e-2;
}

LinearBinarySVM::LinearBinarySVM(double C, double eps) : Classifier() {
	this->C = C;
	this->eps = eps;
}

std::ostream& operator<<(std::ostream& ost, const LinearBinarySVM& svm) {

	// Matrix* W = ((Classifier&) svm).getProjectionMatrix();
	Matrix* W = svm.W;
	int nFeature = W->getRowDimension();
	ost << nFeature << "\n";
	for (int i = 0; i < nFeature; i++) {
		ost << W->getEntry(i, 0) << "\n";
	}
	double* b = svm.b;
	ost << b[0];
	return ost;

}

std::istream& operator>>(std::istream& ist, LinearBinarySVM& svm) {
	int nFeature = 0;
	ist >> nFeature;
	Matrix* W = new DenseMatrix(nFeature, 1);
	double v = 0;
	for (int i = 0; i < nFeature; i++) {
		ist >> v;
		W->setEntry(i, 0, v);
	}
	svm.W = W;
	double* b = new double[1];
	ist >> b[0];
	// b[0] = v;
	svm.b = b;
	svm.nFeature = nFeature;
	svm.nClass = 2;
	return ist;
}

void LinearBinarySVM::loadModel(std::string filePath) {
	std::ifstream modelFileSteam(filePath.c_str());
	if (modelFileSteam.is_open()) {
		modelFileSteam >> *this;
		modelFileSteam.close();
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

void LinearBinarySVM::saveModel(std::string filePath) {
	std::ofstream modelFileSteam(filePath.c_str());
	if (modelFileSteam.is_open()) {
		modelFileSteam << *this;
		modelFileSteam.close();
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

double LinearBinarySVM::innerProduct(double* W, Matrix& X, int i, double* pr_CSR) {
	int N = X.getColumnDimension();
	double res = 0;
	double s = 0;
	if (typeid(X) == typeid(DenseMatrix)) {
		double** XData = ((DenseMatrix&) X).getData();
		double* XRow = null;
		XRow = XData[i];
		for (int j = 0; j < N; j++) {
			s += W[j] * XRow[j];
		}
		res = s + W[N];
	} else {
		int* ic = ((SparseMatrix&) X).getIc();
		int* jr = ((SparseMatrix&) X).getJr();
		// int* valCSRIndices = ((SparseMatrix&) X).getValCSRIndices();
		// double* pr = ((SparseMatrix&) X).getPr();
		for (int k = jr[i]; k < jr[i + 1]; k++) {
			// s += W[ic[k]] * pr[valCSRIndices[k]];
			s += W[ic[k]] * pr_CSR[k];
		}
		res = s + W[N];
	}
	return res;
}

double* LinearBinarySVM::computeQ(Matrix& X, double* pr_CSR) {
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
			Q[i] = s;
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
			Q[i] = s;
		}
	}
	return Q;
}

void LinearBinarySVM::updateW(double* W, double v, Matrix& X, int i, double* pr_CSR) {
	int N = X.getColumnDimension();
	if (typeid(X) == typeid(DenseMatrix)) {
		double** XData = ((DenseMatrix&) X).getData();
		double* XRow = null;
		XRow = XData[i];
		for (int j = 0; j < N; j++) {
			W[j] += v * XRow[j];
		}
		W[N] += v;
	} else {
		int* ic = ((SparseMatrix&) X).getIc();
		int* jr = ((SparseMatrix&) X).getJr();
		// int* valCSRIndices = ((SparseMatrix&) X).getValCSRIndices();
		// double* pr = ((SparseMatrix&) X).getPr();
		for (int k = jr[i]; k < jr[i + 1]; k++) {
			// W[ic[k]] += v * pr[valCSRIndices[k]];
			W[ic[k]] += v * pr_CSR[k];
		}
		W[N] += v;
	}
}

void LinearBinarySVM::train() {
	double* Y = new double[nExample];
	for (int i = 0; i < nExample; i++) {
		Y[i] = -2 * (labelIDs[i] - 0.5);
	}

	// Vector* Xs = getVectors(X);

	double* pr_CSR = null;
	if (typeid(*X) == typeid(SparseMatrix)) {
		double* pr = ((SparseMatrix*) X)->getPr();
		int* valCSRIndices = ((SparseMatrix*) X)->getValCSRIndices();
		int nnz = ((SparseMatrix*) X)->getNNZ();
		pr_CSR = allocate1DArray(nnz);
		for (int k = 0; k < nnz; k++) {
			pr_CSR[k] = pr[valCSRIndices[k]];
		}
	}

	double* W = allocate1DArray(nFeature + 1, 0);
	double* Q = computeQ(*X, pr_CSR);

	double* alphas = allocateVector(nExample, 0);

	/*double M = std::numeric_limits<double>::min();
	double m = std::numeric_limits<double>::max();*/
	double M = -1e10;
	double m = 1e10;
	double Grad = 0;
	// double alpha = 0;
	double alpha_old = 0;
	double alpha_new = 0;
	double PGrad = 0;
	int cnt = 1;
	while (true) {
		/*M = std::numeric_limits<double>::min();
		m = std::numeric_limits<double>::max();*/
		M = -1.0 / 0.0;
		m = 1.0 / 0.0;
		for (int i = 0; i < nExample; i++) {
			// Grad = Y[i] * innerProduct(W, Xs[i]) - 1;
			Grad = Y[i] * innerProduct(W, *X, i, pr_CSR) - 1;
			alpha_old = alphas[i];
			if (alpha_old == 0) {
				PGrad = min(Grad, 0);
			} else if (alpha_old == C) {
				PGrad = max(Grad, 0);
			} else {
				PGrad = Grad;
			}
			M = max(M, PGrad);
			// M = M > PGrad ? M : PGrad;
			m = min(m, PGrad);
			// m = m < PGrad ? m : PGrad;
			if (PGrad != 0) {
				alpha_new = min(max(alpha_old - Grad / Q[i], 0), C);
				// W <- W + (alpha_i_new - alpha_i_old) * Y_i * X_i
				// updateW(W, Y[i] * (alphas[i] - alpha), Xs[i]);
				updateW(W, Y[i] * (alpha_new - alpha_old), *X, i, pr_CSR);
				alphas[i] = alpha_new;
			}
		}
		if (cnt % 20 == 0)
			fprintf(".");
		if (cnt % 400 == 0)
			fprintf("\n");
		cnt++;
		/*double temp = M - m;
		fprintf("M - m = %.4f\n", temp);*/
		if (fabs(M - m) <= eps) {
			fprintf("\n");
			break;
		}
	}

	double* weights = new double[nFeature];
	// System.arraycopy(W, 0, weights, 0, X->getColumnDimension());
	std::copy(W, W + nFeature, weights);
	this->W = new DenseMatrix(W, nFeature, 1);
	b = new double[1];
	b[0] = W[X->getColumnDimension()];
}

Matrix& LinearBinarySVM::predictLabelScoreMatrix(Matrix& Xt) {
	int n = Xt.getRowDimension();
	double** ScoreData = ((DenseMatrix&) Xt.mtimes(*W).plus(b[0])).getData();
	DenseMatrix& ScoreMatrix = *new DenseMatrix(n, 2);
	double** scores = ScoreMatrix.getData();
	for (int i = 0; i < n; i++) {
		scores[i][0] = ScoreData[i][0];
		scores[i][1] = -ScoreData[i][0];
	}
	return ScoreMatrix;
}
