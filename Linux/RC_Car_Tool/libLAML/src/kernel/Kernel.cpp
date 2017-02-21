/*
 * Kernel.cpp
 *
 *  Created on: Feb 28, 2014
 *      Author: Mingjie Qian
 */

#include "Kernel.h"
#include "Utility.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "ArrayOperator.h"
#include "Matlab.h"
#include "InPlaceOperator.h"
#include <cmath>

/**
 * Computes Gram matrix of a specified kernel. Given a data matrix
 * X (n x d), it returns Gram matrix K (n x n).
 *
 * @param kernelType 'linear' | 'poly' | 'rbf' | 'cosine'
 *
 * @param kernelParam   --    | degree | sigma |    --
 *
 * @param X a matrix with each row being a feature vector

 * @return Gram matrix (n x n)
 *
 */
Matrix& calcKernel(std::string kernelType,
		double kernelParam, Matrix& X) {
	return calcKernel(kernelType, kernelParam, X, X);
}

/**
 * Computes Gram matrix of a specified kernel. Given two sets of vectors
 * A (n1 vectors) and B (n2 vectors), it returns Gram matrix K (n1 x n2).
 *
 * @param kernelType 'linear' | 'poly' | 'rbf' | 'cosine'
 *
 * @param kernelParam   --    | degree | sigma |    --
 *
 * @param A a 1D {@code Vector} array
 *
 * @param B a 1D {@code Vector} array
 *
 * @return Gram matrix (n1 x n2)
 */
Matrix& calcKernel(std::string kernelType,
		double kernelParam, Vector** A, int nA, Vector** B, int nB) {

	Matrix* K = null;
	if (kernelType == "linear") {
		double** resData = allocate2DArray(nA, nB, 0);
		double* resRow = null;
		Vector* V = null;
		for (int i = 0; i < nA; i++) {
			resRow = resData[i];
			V = A[i];
			for (int j = 0; j < nB; j++) {
				resRow[j] = innerProduct(*V, *B[j]);
			}
		}
		K = new DenseMatrix(resData, nA, nB);
		// K = A.transpose().mtimes(B);
	} else if (kernelType == "cosine") {
		double* AA = new double[nA];
		Vector* V = null;
		for (int i = 0; i < nA; i++) {
			V = A[i];
			// AA[i] = sum(V->times(*V));
			AA[i] = innerProduct(*V, *V);
		}
		double* BB = new double[nB];
		for (int i = 0; i < nB; i++) {
			V = B[i];
			// BB[i] = sum(V->times(*V));
			BB[i] = innerProduct(*V, *V);
		}
		double** resData = allocate2DArray(nA, nB, 0);
		double* resRow = null;
		for (int i = 0; i < nA; i++) {
			resRow = resData[i];
			V = A[i];
			for (int j = 0; j < nB; j++) {
				resRow[j] = innerProduct(*V, *B[j]) / sqrt(AA[i] * BB[j]);
			}
		}
		K = new DenseMatrix(resData, nA, nB);
		delete[] AA;
		delete[] BB;
		// K = dotMultiply(scalarDivide(1, sqrt(kron(AA.transpose(), BB))), AB);
	} else if (kernelType == "poly") {
		double** resData = allocate2DArray(nA, nB, 0);
		double* resRow = null;
		Vector* V = null;
		for (int i = 0; i < nA; i++) {
			resRow = resData[i];
			V = A[i];
			for (int j = 0; j < nB; j++) {
				resRow[j] = pow(innerProduct(*V, *B[j]), kernelParam);
			}
		}
		K = new DenseMatrix(resData, nA, nB);
		// K = pow(A.transpose().mtimes(B), kernelParam);
	} else if (kernelType == "rbf") {
		K = &l2DistanceSquare(A, nA, B, nB);
		timesAssign(*K, -1 / (2 * pow(kernelParam, 2)));
		expAssign(*K);
		// K = exp(l2DistanceSquare(X1, X2).times(-1 / (2 * Math.pow(kernelParam, 2))));
	}
	return *K;

}

/**
 * Computes Gram matrix of a specified kernel. Given two data matrices
 * X1 (n1 x d), X2 (n2 x d), it returns Gram matrix K (n1 x n2).
 *
 * @param kernelType 'linear' | 'poly' | 'rbf' | 'cosine'
 *
 * @param kernelParam   --    | degree | sigma |    --
 *
 * @param X1 a matrix with each row being a feature vector
 *
 * @param X2 a matrix with each row being a feature vector
 *
 * @return Gram matrix (n1 x n2)
 *
 */
Matrix& calcKernel(std::string kernelType,
		double kernelParam, Matrix& X1, Matrix& X2) {

	Matrix* K = null;
	if (kernelType == "linear") {
		K = &X1.transpose().mtimes(X2);
	} else if (kernelType == "cosine") {
		Matrix& A = X1;
		Matrix& B = X2;
		Matrix& AXA = times(A, A);
		Matrix& BXB = times(B, B);
		DenseVector& SumA = sum(AXA, 2);
		DenseVector& SumB = sum(BXB, 2);
		delete &AXA;
		delete &BXB;

		double* AA = SumA.getPr();
		double* BB = SumB.getPr();

		/*double* AA = sum(times(A, A), 2).getPr();
			double* BB = sum(times(B, B), 2).getPr();*/
		Matrix& BT = B.transpose();

		Matrix& AB = A.mtimes(BT);
		delete &BT;
		/*double* AA = sum(times(A, A), 2).getPr();
		double* BB = sum(times(B, B), 2).getPr();
		Matrix& AB = A.mtimes(B.transpose());*/
		int M = AB.getRowDimension();
		int N = AB.getColumnDimension();
		double v = 0;
		if (typeid(AB) == typeid(DenseMatrix)) {
			double** resData = ((DenseMatrix&) AB).getData();
			double* resRow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				v = AA[i];
				for (int j = 0; j < N; j++) {
					resRow[j] /= sqrt(v * BB[j]);
				}
			}
		} else {
			double* pr = ((SparseMatrix&) AB).getPr();
			int* ir = ((SparseMatrix&) AB).getIr();
			int* jc = ((SparseMatrix&) AB).getJc();
			for (int j = 0; j < N; j++) {
				v = BB[j];
				for (int k = jc[j]; k < jc[j + 1]; k++) {
					pr[k] /= sqrt(AA[ir[k]] * v);
				}
			}
		}
		delete &SumA;
		delete &SumB;
		K = &AB;
		// K = dotMultiply(scalarDivide(1, sqrt(kron(AA.transpose(), BB))), AB);
	} else if (kernelType == "poly") {
		Matrix& X2T = X2.transpose();
		Matrix& X1XX2T = X1.mtimes(X2T);
		K = &pow(X1XX2T, kernelParam);
		delete &X2T;
		delete &X1XX2T;
		// K = &pow(X1.mtimes(X2.transpose()), kernelParam);
	} else if (kernelType == "rbf") {
		K = &l2DistanceSquare(X1, X2);
		timesAssign(*K, -1 / (2 * pow(kernelParam, 2)));
		expAssign(*K);
		// K = exp(l2DistanceSquare(X1, X2).times(-1 / (2 * Math.pow(kernelParam, 2))));
	}
	return *K;

}

