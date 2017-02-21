/*
 * MCTest.cpp
 *
 *  Created on: Mar 4, 2014
 *      Author: Mingjie Qian
 */

#include "Utility.h"
#include "Printer.h"
#include "Matrix.h"
#include "Matlab.h"
#include "MyTime.h"
#include "MatrixCompletion.h"
#include "ArrayOperator.h"
#include <cmath>

int main(void) {

	int m = 6;
	int r = 1;
	int p = (int) round(m * m * 0.3);

	Matrix& L = randn(m, r);
	Matrix& R = randn(m, r);
	Matrix& A_star = mtimes(L, R.transpose());

	int* indices0 = randperm(m * m);
	int len = m * m;
	minusAssign(indices0, len, 1);
	int* indices = linearIndexing(indices0, colon(0, p - 1), p);

	Matrix& Omega = zeros(size(A_star));
	linearIndexingAssignment(Omega, indices, p, 1);

	Matrix& D = zeros(size(A_star));
	linearIndexingAssignment(D, indices, p, linearIndexing(A_star, indices, p));

	Matrix& E_star = D.minus(A_star);
	logicalIndexingAssignment(E_star, Omega, 0);

	/*D = loadMatrix("D.txt");
	Omega = loadMatrix("Omega.txt");*/

	/*disp("D:");
	disp(D);
	disp("Omega:");
	disp(Omega);*/

	// Run matrix completion
	MatrixCompletion& matrixCompletion = *new MatrixCompletion();
	matrixCompletion.feedData(D);
	matrixCompletion.feedIndices(Omega);
	tic();
	matrixCompletion.run();
	fprintf("Elapsed time: %.2f seconds.\n", toc());

	// Output
	Matrix& A_hat = matrixCompletion.GetLowRankEstimation();

	fprintf("A*:\n");
	disp(A_star, 4);
	fprintf("A^:\n");
	disp(A_hat, 4);
	fprintf("D:\n");
	disp(D, 4);
	fprintf("rank(A*): %d\n", rank(A_star));
	fprintf("rank(A^): %d\n", rank(A_hat));
	fprintf("||A* - A^||_F: %.4f\n", norm(A_star.minus(A_hat), "fro"));

	return EXIT_SUCCESS;

}


