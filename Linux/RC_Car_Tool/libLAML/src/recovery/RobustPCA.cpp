/*
 * RobustPCA.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#include "RobustPCA.h"

/**
 * Constructor for Robust PCA.
 *
 * @param lambda a positive weighting parameter, larger value leads to sparser
 * 				 error matrix
 */
RobustPCA::RobustPCA(double lambda) {
	this->lambda = lambda;
}

/**
 * Feed an observation matrix.
 *
 * @param D a real matrix
 */
void RobustPCA::feedData(Matrix& D) {
	this->D = &D;
}

/**
 * Run robust PCA.
 */
void RobustPCA::run() {
	Matrix** res = run(*D, lambda);
	A = res[0];
	E = res[1];
}

/**
 * Get the low-rank matrix recovered from the corrupted data
 * observation matrix.
 *
 * @return low-rank approximation
 */
Matrix& RobustPCA::GetLowRankEstimation() {
	return *A;
}

/**
 * Get the error matrix between the original observation matrix
 * and its low-rank recovered matrix.
 *
 * @return error matrix
 */
Matrix& RobustPCA::GetErrorMatrix() {
	return *E;
}

/**
 * Compute robust PCA for an observation matrix which solves the
 * following convex optimization problem:
 * </br>
 * min ||A||_* + lambda ||E||_1</br>
 * s.t. D = A + E</br>
 * where ||.||_* denotes the nuclear norm of a matrix (i.e.,
 * the sum of its singular values), and ||.||_1 denotes the
 * sum of the absolute values of matrix entries.</br>
 * </br>
 * Inexact augmented Lagrange multipliers is used to solve the optimization
 * problem due to its empirically fast convergence speed and proved convergence
 * to the true optimal solution.
 *
 * @param D a real observation matrix
 *
 * @param lambda a positive weighting parameter, larger value leads to sparser
 * 				 error matrix
 * @return a {@code Matrix} array [A, E] where A is the recovered low-rank
 * 		   approximation of D, and E is the error matrix between A and D
 *
 */
Matrix** RobustPCA::run(Matrix& D, double lambda) {

	Matrix& Y = rdivide(D, J(D, lambda));
	Matrix& E = zeros(size(D));
	Matrix& A = minus(D, E);
	/*fprintf("norm(D, inf) = %f\n", norm(D, inf));
	fprintf("norm(D, 2) = %f\n", norm(D, 2));*/
	double mu = 1.25 / norm(D, 2);
	double rou = 1.6;
	int k = 0;
	double norm_D = norm(D, "fro");
	double e1 = 1e-7;
	double e2 = 1e-6;
	double c1 = 0;
	double c2 = 0;
	double mu_old = 0;
	Matrix& E_old = E.copy();
	Matrix** SVD = null;
	Matrix& DMinusA = E.copy();
	Matrix& DMinusE = E.copy();
	Matrix& DMinusAMinusE = E.copy();
	Matrix& DeltaE = E.copy();
	Matrix& T = DMinusA.copy();
	Matrix& T2 = DMinusE.copy();
	Matrix& A2 = A.copy();
	Matrix& US = DMinusA.copy();
	int numSingularValues = min(size(T2, 1), size(T2, 2));
	while (true) {

		minus(DMinusA, D, A);

		// Stopping criteria
		if (k > 0) {
			minus(DMinusAMinusE, DMinusA, E);
			c1 = norm(DMinusAMinusE, "fro") / norm_D;
			c2 = mu_old * norm(DeltaE, "fro") / norm_D;
			// fprintf("k = %d, c2: %.4f%n", k, c2);
			if (c1 <= e1 && c2 <= e2)
				break;
		}

		assign(E_old, E);
		mu_old = mu;

		// E_{k+1} = argmin_E L(A_k, E, Y_k, mu_k)
		affine(T, DMinusA, 1 / mu, Y);
		/*disp("T:");
		disp(T);*/
		ShrinkageOperator::shrinkage(E, lambda / mu, T);

		minus(DMinusE, D, E);
		affine(T2, DMinusE, 1 / mu, Y);
		// A_{k+1} = argmin_A L(A, E_{k+1}, Y_k, mu_k)
		SVD = svd(T2);
		Matrix& U = *SVD[0];
		Matrix& S = *SVD[1];
		Matrix& V = *SVD[2];
		Matrix& VT = V.transpose();
		/*disp("T:");
		disp(T);
		disp("USVt:");
		disp(U.mtimes(S).mtimes(VT));*/
		// disp(*SVD[1]);
		// fprintf("1 / mu = %f\n", 1 / mu);
		// ShrinkageOperator::shrinkage(S, 1 / mu, S);
		double t = 1 / mu;
		for (int i = 0; i < numSingularValues; i++) {
			double v = S.getEntry(i, i);
			if (v > t)
				S.setEntry(i, i, v - t);
			else if (v < -t)
				S.setEntry(i, i, v + t);
			else
				S.setEntry(i, i, 0);
		}
		// disp(*SVD[1]);
		// Matrix& US = U.mtimes(S);
		mtimes(US, U, S);
		// Matrix& A2 = US.mtimes(VT);
		mtimes(A2, US, VT);
		assign(A, A2);
		delete &VT;
		// delete &A2;
		// delete &US;
		delete SVD[0];
		delete SVD[1];
		delete SVD[2];
		delete[] SVD;

		// Y = Y.plus(times(mu, D.minus(A).minus(E)));
		minus(DMinusA, D, A);
		minus(DMinusAMinusE, DMinusA, E);
		plusAssign(Y, mu, DMinusAMinusE);

		// Update mu_k to mu_{k+1}
		minus(DeltaE, E, E_old);
		if (norm(DeltaE, "fro") * mu / norm_D < e2)
			mu = rou * mu;

		k = k + 1;

	}

	delete &E_old;
	delete &DMinusA;
	delete &DMinusE;
	delete &DMinusAMinusE;
	delete &DeltaE;
	delete &T;
	delete &T2;
	delete &A2;
	delete &US;
	delete &Y;

	Matrix** res = new Matrix*[2];
	res[0] = &A;
	res[1] = &E;
	return res;

}

double RobustPCA::J(Matrix& Y, double lambda) {

	double s = norm(Y, 2);
	int M = Y.getRowDimension();
	int N = Y.getColumnDimension();
	if (typeid(Y) == typeid(DenseMatrix)) {
		double** YData = ((DenseMatrix&) Y).getData();
		double* YRow = null;
		for (int i = 0; i < M; i++) {
			YRow = YData[i];
			for (int j = 0; j < N; j++) {
				s = max(s, fabs(YRow[j]));
			}
		}
	} else {
		double* pr = ((SparseMatrix&) Y).getPr();
		int nnz = ((SparseMatrix&) Y).getNNZ();
		for (int k = 0; k < nnz; k++)
			s = max(s, fabs(pr[k]));
	}
	return s;

	/*Matrix& absY = abs(Y);
	Vector** maxRes = max(absY);
	double* maxRes2 = max(*maxRes[0]);
	double res = max(norm(Y, 2), maxRes2[0]);
	delete[] maxRes2;
	delete maxRes[0];
	delete maxRes[1];
	delete[] maxRes;
	delete &absY;
	return res;*/

	// return max(norm(Y, 2), max(*max(abs(Y))[0])[0] / lambda);
}


