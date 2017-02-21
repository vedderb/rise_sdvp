/*
 * MatrixTest.cpp
 *
 *  Created on: Feb 7, 2014
 *      Author: Aaron
 */

#include "Matrix.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Printer.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include "Matlab.h"
#include "LU.h"

// using namespace std;

void fun(Matrix& S) {
	// DenseMatrix& dist = full(S.mtimes(S.transpose()).times(-2));
	SparseMatrix S2((SparseMatrix&) S.transpose());
	disp(S2);
}

int main(void) {

	std::cout << "Matrix Test..." << std::endl;
	std::cout << std::setiosflags(std::ios::fixed);

	/*
	 * 10	0	0	0
	 * 3	9	0	0
	 * 0	7	8	7
	 * 3	0	7	7
	 */
	int rIndices[] = {0, 1, 3, 1, 2, 2, 3, 2, 3};
	int cIndices[] = {0, 0, 0, 1, 1, 2, 2, 3, 3};
	double values[] = {10, 3.2, 3, 9, 7, 8, 7, 7, 7};
	int numRows = 4;
	int numColumns = 4;
	int nzmax = 9;

	Matrix& S = SparseMatrix::createSparseMatrix(rIndices, cIndices, values, numRows, numColumns, nzmax);

	disp("S:");
	printMatrix(S);

	fprintf("det(S) = %.4f\n", det(S));

	LU LUDecomp(S);
	disp("L:");
	disp(*LUDecomp.getL());
	disp("U:");
	disp(*LUDecomp.getU());
	disp("inv(S):");
	disp(LUDecomp.inverse());

	Matrix& invS = inv(S);
	disp("inv(S):");
	disp(invS);

	disp("invS * S:");
	disp(invS.mtimes(S));

	fun(S);

	std::cout << "S(2, 1): " << S.getEntry(2, 1) << std::endl;

	S.setEntry(1, 3, -2);
	disp("S(2, 4) = -2:");
	disp(S);

	disp("Destroying S...");
	S.~Matrix();

	// Matrix MPtr[] = {*new DenseMatrix(), *new SparseMatrix()};
	// Matrix* &M = {new DenseMatrix(), new SparseMatrix()};
	Matrix** Ms = new Matrix*[2];
	Ms[0] = new DenseMatrix(2, 3, 2);
	Ms[1] = new SparseMatrix(2, 3);
	for (int i = 0; i < 2; i++) {
		fprintf("Ms[%d]:\n", i);
		disp(*Ms[i]);
		if (typeid(*Ms[i]) == typeid(DenseMatrix)) {
			fprintf("Ms[%d] points to a dense matrix.\n", i);
		} else
			fprintf("Ms[%d] points to a sparse matrix.\n", i);
	}

	/*for (int i = 0; i < 10; i++) {
		Matrix& A = Ms[0]->plus(*Ms[1]);
		printMatrix(A);
	}*/

	return EXIT_SUCCESS;

}



