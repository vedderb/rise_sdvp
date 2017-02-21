/*
 * QRTest.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#include "QR.h"
#include "MyTime.h"
#include "Printer.h"
#include "Matlab.h"

int main(void) {

	int m = 4;
	int n = 3;
	Matrix* A = &hilb(m, n);
	// Matrix& A2 = hilb(m, n);

	fprintf("When A is full:\n");

	fprintf("A:\n");
	printMatrix(*A);

	tic();

	Matrix** QRP = QR::decompose(*A);
	Matrix* Q = QRP[0];
	Matrix* R = QRP[1];
	Matrix* P = QRP[2];

	fprintf("Q:\n");
	printMatrix(*Q);

	fprintf("R:\n");
	printMatrix(*R);

	fprintf("P:\n");
	printMatrix(*P);

	fprintf("AP:\n");
	printMatrix(A->mtimes(*P));

	fprintf("QR:\n");
	printMatrix(Q->mtimes(*R));

	fprintf("Q'Q:\n");
	printMatrix(Q->transpose().mtimes(*Q));

	fprintf("Elapsed time: %.2f seconds.\n", toc());
	fprintf("**********************************\n");

	// fprintf("|AP - QR| = ");

	A = &sparse(hilb(m, n));

	fprintf("When A is sparse:\n");

	fprintf("A:\n");
	printMatrix(*A);

	tic();

	QRP = QR::decompose(*A);
	Q = QRP[0];
	R = QRP[1];
	P = QRP[2];

	fprintf("Q:\n");
	printMatrix(*Q);

	fprintf("R:\n");
	printMatrix(*R);

	fprintf("P:\n");
	printMatrix(*P);

	fprintf("AP:\n");
	printMatrix(A->mtimes(*P));

	fprintf("QR:\n");
	printMatrix(Q->mtimes(*R));

	fprintf("Q'Q:\n");
	printMatrix(Q->transpose().mtimes(*Q));

	fprintf("Elapsed time: %.2f seconds.\n", toc());

	QR* QRDecomp = new QR(*A);
	double bData[] = {2, 3, 4, 9};
	Vector& b = *new DenseVector(bData, 4);
	Vector& x = QRDecomp->solve(b);
	fprintf("Solution for Ax = b:\n");
	printVector(x);
	fprintf("b = \n");
	printVector(b);
	fprintf("Ax = \n");
	printVector(A->operate(x));

	// disp("Mission complete.");

	return EXIT_SUCCESS;

}


