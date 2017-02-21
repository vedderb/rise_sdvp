/*
 * EVDTest.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#include "EVD.h"
#include <stdlib.h>
#include "MyTime.h"
#include "Printer.h"
#include "Matlab.h"

int main(void) {

	int m = 4;
	int n = 4;
	Matrix& A = hilb(m, n);
	/*A = new DenseMatrix(new double[][] {
					{1, 3, 4},
					{3, 2, 8},
					{4, 8, 3} });

			A = IO.loadMatrix("D.txt");
			A = A.transpose().mtimes(A);*/
	fprintf("A:\n");
	disp(A);
	tic();
	Matrix** VD = EVD::decompose(A);
	fprintf("Elapsed time: %.4f seconds.\n", toc());
	fprintf("*****************************************\n");

	Matrix& V = *VD[0];
	Matrix& D = *VD[1];

	fprintf("V:\n");
	printMatrix(V);

	fprintf("D:\n");
	printMatrix(D);

	fprintf("VDV':\n");
	disp(V.mtimes(D).mtimes(V.transpose()));

	fprintf("A:\n");
	printMatrix(A);

	fprintf("V'V:\n");
	printMatrix(V.transpose().mtimes((V)));

	return EXIT_SUCCESS;

}


