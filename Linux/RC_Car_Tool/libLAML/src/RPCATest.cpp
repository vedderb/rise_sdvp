/*
 * RPCATest.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#include "Utility.h"
#include "Printer.h"
#include "Matrix.h"
#include "Matlab.h"
#include "MyTime.h"
#include "RobustPCA.h"
#include <cmath>

int main(void) {

	/*Matrix& A0 = *new DenseMatrix(new double[4] {1, 2, 3, 4}, 4, 1);
	disp(A0);
	Matrix& A1 = reshape(A0, new int[2] {2, 2});
	disp(A1);*/

	int m = 8;
	int r = m / 4;

	Matrix& L = randn(m, r);
	Matrix& R = randn(m, r);

	Matrix& A_star = mtimes(L, R.transpose());
	Matrix& E_star0 = zeros(size(A_star));
	int* indices = randperm(m * m);
	int nz = m * m / 20;
	int* nz_indices = new int[nz];
	for (int i = 0; i < nz; i++) {
		nz_indices[i] = indices[i] - 1;
	}
	Matrix& E_vec = vec(E_star0);
	Matrix& Temp = (minus(rand(nz, 1), 0.5).times(100));
	// disp(Temp);
	setSubMatrix(E_vec, nz_indices, nz, new int[1] {0}, 1, Temp);
	// disp(E_vec);
	Matrix& E_star = reshape(E_vec, size(E_star0));
	// disp(E_star);

	// Input
	Matrix& D = A_star.plus(E_star);
	double lambda = 1 * pow(m, -0.5);
	RobustPCA& robustPCA = *new RobustPCA(lambda);
	robustPCA.feedData(D);
	tic();
	robustPCA.run();
	fprintf("Elapsed time: %.2f seconds.\n", toc());

	// Output
	Matrix& A_hat = robustPCA.GetLowRankEstimation();
	Matrix& E_hat = robustPCA.GetErrorMatrix();

	fprintf("A*:\n");
	disp(A_star, 4);
	fprintf("A^:\n");
	disp(A_hat, 4);
	fprintf("E*:\n");
	disp(E_star, 4);
	fprintf("E^:\n");
	disp(E_hat, 4);
	fprintf("rank(A*): %d\n", rank(A_star));
	fprintf("rank(A^): %d\n", rank(A_hat));
	fprintf("||A* - A^||_F: %.4f\n", norm(A_star.minus(A_hat), "fro"));
	fprintf("||E* - E^||_F: %.4f\n", norm(E_star.minus(E_hat), "fro"));

	return EXIT_SUCCESS;

}


