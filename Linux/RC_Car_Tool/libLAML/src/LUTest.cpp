/*
 * LUTest.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: Mingjie Qian
 */

#include "LU.h"
#include "Matrix.h"
#include "DenseMatrix.h"
#include "DenseVector.h"
#include "Printer.h"
#include "MyTime.h"
#include "Matlab.h"
#include "LU.h"

int main(void) {

	double AData[][3] = {
			{1, -2, 3},
			{2, -5, 12},
			{0, 2, -10}
			};

	Matrix& A = DenseMatrix::createDenseMatrix<3>(AData, 3, 3);

	fprintf("A:\n");
	printMatrix(A);

	Matrix** LUP = LU::decompose(A);
	Matrix* L = LUP[0];
	Matrix* U = LUP[1];
	Matrix* P = LUP[2];

	fprintf("L:\n");
	printMatrix(*L);

	fprintf("U:\n");
	printMatrix(*U);

	fprintf("P:\n");
	printMatrix(*P);

	fprintf("PA:\n");
	printMatrix(P->mtimes(A));

	fprintf("LU:\n");
	printMatrix(L->mtimes(*U));

	tic();

	LU* LUDecomp = new LU(sparse(A));
	double bData[] = {2, 3, 4};
	Vector& b = *new DenseVector(bData, 3);
	Vector& x = LUDecomp->solve(b);
	fprintf("Solution for Ax = b:\n");
	printVector(x);
	fprintf("b = \n");
	printVector(b);
	fprintf("Ax = \n");
	printVector(A.operate(x));

	fprintf("A^{-1}:\n");
	printMatrix(LUDecomp->inverse());

	fprintf("det(A) = %.2f\n", LUDecomp->det());
	fprintf("Elapsed time: %.2f seconds.\n", toc());
	fprintf("**********************************\n");

	A = sparse(A);
	fprintf("A:\n");
	printMatrix(A);

	LUP = LU::decompose(sparse(A));
	L = LUP[0];
	U = LUP[1];
	P = LUP[2];

	fprintf("L:\n");
	printMatrix(*L);

	fprintf("U:\n");
	printMatrix(*U);

	fprintf("P:\n");
	printMatrix(*P);

	fprintf("PA:\n");
	/*Matrix& Temp = P->mtimes(A);
	printMatrix(Temp);*/
	printMatrix(P->mtimes(A));

	fprintf("LU:\n");
	printMatrix(L->mtimes(*U));

	tic();

	LUDecomp = new LU(sparse(A));
	b = *new DenseVector(bData, 3);
	x = LUDecomp->solve(b);
	fprintf("Solution for Ax = b:\n");
	printVector(x);
	fprintf("Ax = \n");
	printVector(A.operate(x));
	fprintf("b = \n");
	printVector(b);

	double BData[][2] = {
			{2, 4},
			{3, 3},
			{4, 2}
			};

	Matrix& B = DenseMatrix::createDenseMatrix<2>(BData, 3, 2);
	Matrix& X = LUDecomp->solve(B);
	fprintf("Solution for AX = B:\n");
	printMatrix(X);
	fprintf("AX = \n");
	printMatrix(A.mtimes(X));
	fprintf("B = \n");
	printMatrix(B);

	fprintf("A^{-1}:\n");
	printMatrix(LUDecomp->inverse());

	fprintf("det(A) = %.2f\n", LUDecomp->det());
	fprintf("Elapsed time: %.2f seconds.\n", toc());

	disp("Mission complete.");

	return EXIT_SUCCESS;

}
