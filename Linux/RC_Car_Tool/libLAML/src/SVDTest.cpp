/*
 * SVDTest.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#include "SVD.h"
#include <stdlib.h>
#include "MyTime.h"
#include "Printer.h"
#include "Matlab.h"

int main(void) {

	int m = 6;
	int n = 4;
	Matrix& A = hilb(m, n);

	// A = new DenseMatrix(new double[][] { {1d, 2d}, {2d, 0d}, {1d, 7d}});

	/*A = new DenseMatrix(new double[][] {
					{1, 2, 3},
					{4, 5, 6},
					{7, 8, 9},
					{10, 11, 12}
			});*/
	// printMatrix(SingularValueDecomposition.bidiagonalize(A)[1]);

	// A = IO.loadMatrix("SVDInput");

	/*fprintf("When A is full:\n\n");

			fprintf("A:\n");
			printMatrix(A);*/

	disp("A:");
	printMatrix(A);

	tic();

	bool computeUV = !false;
	Matrix** USV = SVD::decompose(A, computeUV);

	fprintf("Elapsed time: %.4f seconds.\n", toc());
	fprintf("*****************************************\n");

	Matrix& U = *USV[0];
	Matrix& S = *USV[1];
	Matrix& V = *USV[2];

	if (computeUV) {
		fprintf("USV':\n");
		disp(U.mtimes(S).mtimes(V.transpose()));

		/*disp("US:");
		Matrix& US = U.mtimes(S);
		disp(US);

		Matrix& VT = V.transpose();
		disp("VT:");
		disp(VT);

		disp("USV':");
		disp(US.mtimes(VT));*/

		fprintf("A:\n");
		printMatrix(A);

		fprintf("U'U:\n");
		printMatrix(U.transpose().mtimes((U)));

		fprintf("V'V:\n");
		printMatrix(V.transpose().mtimes((V)));

		fprintf("U:\n");
		printMatrix(U);

		fprintf("V:\n");
		printMatrix(V);

	}

	fprintf("S:\n");
	printMatrix(S);

	fprintf("rank(A): %d\n", SVD::rank(A));
	return EXIT_SUCCESS;

}


