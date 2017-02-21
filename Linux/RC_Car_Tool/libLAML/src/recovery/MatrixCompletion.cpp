/*
 * MatrixCompletion.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Mingjie Qian
 */

#include "MatrixCompletion.h"

/**
 * Constructor.
 */
MatrixCompletion::MatrixCompletion() {
}

/**
 * Feed an observation matrix.
 *
 * @param D a real matrix
 */
void MatrixCompletion::feedData(Matrix& D) {
	this->D = &D;
}

/**
 * Feed indices of samples.
 *
 * @param Omega a sparse or dense logical matrix indicating the indices of samples
 *
 */
void MatrixCompletion::feedIndices(Matrix& Omega) {
	this->Omega = &Omega;
}

/**
 * Feed indices of samples.
 *
 * @param indices an {@code int} array for the indices of samples
 *
 * @param len length of the index array
 */
void MatrixCompletion::feedIndices(int* indices, int len) {
	Omega = new SparseMatrix(size(*D, 1), size(*D, 2));
	linearIndexingAssignment(*Omega, indices, len, 1);
}

/**
 * Run matrix completion.
 */
void MatrixCompletion::run() {
	Matrix** res = run(*D, *Omega);
	A = res[0];
	E = res[1];
}

/**
 * Get the low-rank completed matrix.
 *
 * @return the low-rank completed matrix
 */
Matrix& MatrixCompletion::GetLowRankEstimation() {
	return *A;
}

/**
 * Get the error matrix between the original observation matrix and
 * its low-rank completed matrix.
 *
 * @return error matrix
 */
Matrix& MatrixCompletion::GetErrorMatrix() {
	return *E;
}

/**
 * Do matrix completion which solves the following convex
 * optimization problem:
 * </br>
 * min ||A||_*</br>
 * s.t. D = A + E</br>
 *      E(Omega) = 0</br>
 * where ||.||_* denotes the nuclear norm of a matrix (i.e.,
 * the sum of its singular values).</br>
 * </br>
 * Inexact augmented Lagrange multipliers is used to solve the optimization
 * problem due to its empirically fast convergence speed and proved convergence
 * to the true optimal solution.
 *
 * @param D a real observation matrix
 *
 * @param Omega a sparse or dense logical matrix indicating the indices of samples
 *
 * @return a {@code Matrix} array [A, E] where A is the low-rank
 * 		   completion from D, and E is the error matrix between A and D
 *
 */
Matrix** MatrixCompletion::run(Matrix& D, Matrix& Omega) {

	Matrix& Y = zeros(size(D));
	Matrix& E = zeros(size(D));
	Matrix& A = minus(D, E);
	int m = size(D, 1);
	int n = size(D, 2);
	double mu = 1 / norm(D, 2);
	// Sampling density
	double rou_s = sumAll(gt(Omega, 0)) / (m * n);
	// The relation between rou and rou_s is obtained by regression
	double rou = 1.2172 + 1.8588 * rou_s;
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
	// Matrix& T = DMinusA.copy();
	Matrix& T2 = DMinusE.copy();
	Matrix& A2 = A.copy();
	Matrix& US = DMinusA.copy();
	int numSingularValues = min(size(T2, 1), size(T2, 2));
	while (true) {

		minus(DMinusA, D, A);

		// Stopping criteria
		if (k > 1) {
			minus(DMinusAMinusE, DMinusA, E);
			c1 = norm(DMinusAMinusE, "fro") / norm_D;
			c2 = mu_old * norm(DeltaE, "fro") / norm_D;
			// fprintf("k = %d, c2: %.4f\n", k, c2);
			if (c1 <= e1 && c2 <= e2)
				break;
		}

		// E_old = E;
		assign(E_old, E);
		mu_old = mu;

		// A_{k+1} = argmin_A L(A, E_k, Y_k, mu_k)
		/*disp(plus(minus(D, E), rdivide(Y, mu)));
			Matrix SVDInput = plus(minus(D, E), rdivide(Y, mu));
			IO.saveMatrix(SVDInput, "SVDInput");*/
		minus(DMinusE, D, E);
		affine(T2, DMinusE, 1 / mu, Y);

		// SVD = svd(plus(minus(D, E), rdivide(Y, mu)));
		SVD = svd(T2);
		Matrix& U = *SVD[0];
		Matrix& S = *SVD[1];
		Matrix& V = *SVD[2];
		Matrix& VT = V.transpose();
		// disp(full(SVD[1]));
		// A = SVD[0].mtimes(shrinkage(SVD[1], 1 / mu)).mtimes(SVD[2].transpose());
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
		mtimes(US, U, S);
		mtimes(A2, US, VT);
		assign(A, A2);
		delete &VT;
		delete SVD[0];
		delete SVD[1];
		delete SVD[2];
		delete[] SVD;

		// E_{k+1} = argmin_E L(A_{k+1}, E, Y_k, mu_k)
		// E = D.minus(A);
		minus(E, D, A);
		logicalIndexingAssignment(E, Omega, 0);

		// plusAssign(Y, mu, D.minus(A).minus(E));
		minus(DMinusA, D, A);
		minus(DMinusAMinusE, DMinusA, E);
		plusAssign(Y, mu, DMinusAMinusE);
		/*disp("Y:");
		    disp(Y);*/

		// disp(E);
		/*if (Double.isNaN(sumAll(E))) {
		    	IO.saveMatrix(D, "D.txt");
		    	IO.saveMatrix(Omega, "Omega.txt");
		    	exit(1);
		    }*/
		// Update mu_k to mu_{k+1}
		minus(DeltaE, E, E_old);
		if (norm(DeltaE, "fro") * mu / norm_D < e2)
			mu = rou * mu;

		// fprintf("mu: %f%n", mu);

		k = k + 1;

	}

	delete &E_old;
	delete &DMinusA;
	delete &DMinusE;
	delete &DMinusAMinusE;
	delete &DeltaE;
	// delete &T;
	delete &T2;
	delete &A2;
	delete &US;
	delete &Y;

	Matrix** res = new Matrix*[2];
	res[0] = &A;
	res[1] = &E;
	return res;

}

/**
 * Do matrix completion which solves the following convex
 * optimization problem:
 * </br>
 * min ||A||_*</br>
 * s.t. D = A + E</br>
 *      E(Omega) = 0</br>
 * where ||.||_* denotes the nuclear norm of a matrix (i.e.,
 * the sum of its singular values).</br>
 * </br>
 * Inexact augmented Lagrange multipliers is used to solve the optimization
 * problem due to its empirically fast convergence speed and proved convergence
 * to the true optimal solution.
 *
 * @param D a real observation matrix
 *
 * @param indices an {@code int} array for the indices of samples
 *
 * @param len length of the index array
 *
 * @return a {@code Matrix} array [A, E] where A is the low-rank
 * 		   completion from D, and E is the error matrix between A and D
 *
 */
Matrix** MatrixCompletion::run(Matrix& D, int* indices, int len) {
	Matrix& Omega = *new SparseMatrix(size(D, 1), size(D, 2));
	linearIndexingAssignment(Omega, indices, len, 1);
	Matrix** res = run(D, Omega);
	delete &Omega;
	return res;
}
