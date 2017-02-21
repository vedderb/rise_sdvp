//============================================================================
// Name        : LAML.cpp
// Author      : Mingjie Qian
// Version     :
// Copyright   : Copyright Reserved!
// Description : Hello World in C, Ansi-style
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include "Matrix.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "MyTime.h"
#include <ctime>

int main(void) {

	puts("!!!Hello World!!!");

	int M = 1000;
	int N = 1000;
	Matrix* A = new DenseMatrix(M, N, 1);
	Matrix* B = new DenseMatrix(M, N, 3.5);

	clock_t clock_start = clock();
	tic();
	Matrix* C = A->mtimes(B);
	clock_t clock_end = clock();
	double elapsedTime = 0;
	elapsedTime = (double) (clock_end - clock_start) / CLOCKS_PER_SEC;
	printf("Elapsed time: %.2f\n", toc());
	printf("C(M - 1, N - 1) = %.2f\n", C->getEntry(M - 1, N - 1));

	delete A;
	delete B;

	/*DenseMatrix D(M, N, 1);
	DenseMatrix E(M, N, 3.5);
	clock_start = clock();
	Matrix* F = D.mtimes(&E);
	clock_end = clock();
	elapsedTime = (double)(clock_end - clock_start) / CLOCKS_PER_SEC;
	printf("Elapsed time: %.2f\n", elapsedTime);
	printf("F(M - 1, N - 1) = %.2f\n", F->getEntry(M - 1, N - 1));*/

	/*DenseMatrix G(M, N, 1);
	DenseMatrix H(M, N, 3.5);*/
	Matrix& G = *new DenseMatrix(M, N, 1);
	Matrix& H = *new DenseMatrix(M, N, 3.5);
	clock_start = clock();
	Matrix& I = G.mtimes(H);
	clock_end = clock();
	elapsedTime = (double)(clock_end - clock_start) / CLOCKS_PER_SEC;
	printf("Elapsed time: %.2f\n", elapsedTime);
	printf("I(M - 1, N - 1) = %.2f\n", I.getEntry(M - 1, N - 1));

	Matrix& J = *new DenseMatrix(M, N, 1);
	Matrix& K = *new SparseMatrix(M, N);
	clock_start = clock();
	Matrix& L = J.mtimes(K);
	clock_end = clock();
	elapsedTime = (double)(clock_end - clock_start) / CLOCKS_PER_SEC;
	printf("Elapsed time: %.2f\n", elapsedTime);
	printf("L(M - 1, N - 1) = %.2f\n", L.getEntry(M - 1, N - 1));

	return EXIT_SUCCESS;

}
