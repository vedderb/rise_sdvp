/*
 * IOTest.cpp
 *
 *  Created on: Feb 14, 2014
 *      Author: Mingjie Qian
 */

#include "IO.h"
#include <stdlib.h>
#include "Printer.h"
#include "Matlab.h"
#include <fstream>
#include <string>

int main(void) {

	/*
	 * 10	0	0	0
	 * 3	9	0	0
	 * 0	7	8	7
	 * 3	0	8	7
	 */
	int rIndices[] = {0, 1, 3, 1, 2, 2, 3, 2, 3};
	int cIndices[] = {0, 0, 0, 1, 1, 2, 2, 3, 3};
	double values[] = {10, 3.2, 3, 9, 7, 8, 8, 7, 7};
	int numRows = 4;
	int numColumns = 4;
	int nzmax = 9;

	Matrix& S = SparseMatrix::createSparseMatrix(rIndices, cIndices, values, numRows, numColumns, nzmax);

	disp("S:");
	printMatrix(S);

	/*Matrix& S2 = sparse(A);
		fprintf("S2:\n");
		printMatrix(S2, 4);*/

	std::string filePath = "";

	filePath = "SparseMatrix.txt";
	saveMatrix(S, filePath);

	fprintf("Loaded S:\n");
	printMatrix(loadMatrix(filePath));

	Matrix& A = full(S);
	fprintf("A:\n");
	printMatrix(A, 4);

	filePath = "DenseMatrix.txt";
	saveMatrix(A, filePath);
	fprintf("Loaded A:\n");
	printMatrix(loadMatrix(filePath));

	return EXIT_SUCCESS;

}


