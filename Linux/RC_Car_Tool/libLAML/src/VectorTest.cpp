/*
 * VectorTest.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: Mingjie Qian
 */

#include "Vector.h"
#include "DenseVector.h"
#include "SparseVector.h"
#include "SparseMatrix.h"
#include "Printer.h"
#include "Matlab.h"
#include "Utility.h"

int main(void) {

	double v = 0.0;
	if (v == 0) {
		disp("v == 0!");
	}

	if (std::isinf(1.0 / 0.0)) {
		disp("1.0 / 0.0 is infinite.");
		fprintf("1.0 / 0.0 = %f\n", 1.0 / 0.0);
		fprintf("-1.0 / 0.0 = %f\n", -1.0 / 0.0);
		// fprintf("1.0 / 0.0 = %f\n", std::numeric_limits<double>::max());
	}

	if (std::isinf(POSITIVE_INFINITY)) {
		fprintf("POSITIVE_INFINITY = %f\n", POSITIVE_INFINITY);
	}

	v = MYNAN;
	if (std::isnan(v)) {
		fprintf("sqrt(-1) = %f\n", v);
	}



	/*double* pr = new double[3] {1, 2, 3};
	disp(pr, 3);*/

	double pr2[] = {1, 2, 3};
	disp(pr2, 3);

	disp("Matrix Test...");
	double V1Data[] = {2, 0, 4};
	Vector& V1 = *new DenseVector(V1Data, 3);
	double V2Data[] = {1, 2, 0};
	Vector& V2 = *new DenseVector(V2Data, 3);

	disp("V1 .* V2:");
	disp(times(V1, V2));

	fprintf("V1 .* sparse(V2):\n");
	disp(times(V1, sparse(V2)));

	fprintf("sparse(V1) .* sparse(V2):\n");
	disp(times(sparse(V1), sparse(V2)));

	fprintf("V1 + V2:\n");
	disp(plus(V1, V2));

	fprintf("V1 + sparse(V2):\n");
	disp(plus(V1, sparse(V2)));

	fprintf("sparse(V1) + sparse(V2):\n");
	disp(plus(sparse(V1), sparse(V2)));

	fprintf("V1 - V2:\n");
	disp(minus(V1, V2));

	fprintf("V1 - sparse(V2):\n");
	disp(minus(V1, sparse(V2)));

	fprintf("sparse(V1) - sparse(V2):\n");
	disp(minus(sparse(V1), sparse(V2)));

	int dim = 4;
	Vector& V = *new SparseVector(dim);

	for (int i = 0; i < dim; i++) {
		fprintf("V(%d):	%.2f\n", i + 1, V.get(i));
	}

	V.set(3, 4.5);
	fprintf("V(%d):	%.2f\n", 3 + 1, V.get(3));
	V.set(1, 2.3);
	fprintf("V(%d):	%.2f\n", 1 + 1, V.get(1));
	V.set(1, 3.2);
	fprintf("V(%d):	%.2f\n", 1 + 1, V.get(1));
	V.set(3, 2.5);
	fprintf("V(%d):	%.2f\n", 3 + 1, V.get(3));

	fprintf("V:\n");
	disp(V);

	// disp(full(V));

	for (int i = 0; i < dim; i++) {
		fprintf("V(%d):	%.2f\n", i + 1, V.get(i));
	}

	int rIndices[] = {0, 1, 3, 1, 2, 2, 3, 2, 3};
	int cIndices[] = {0, 0, 0, 1, 1, 2, 2, 3, 3};
	double values[] = {10, 3.2, 3, 9, 7, 8, 8, 7, 7};
	int numRows = 4;
	int numColumns = 4;
	int nzmax = 9;

	Matrix& A = *new SparseMatrix(rIndices, cIndices, values, numRows, numColumns, nzmax);

	disp("A:");
	printMatrix(A);

	disp("AV:");
	disp(A.operate(V));

	disp("V'A':");
	disp(V.operate(A.transpose()));

	disp(A.operate(full(V)));

	disp(full(A).operate(V));

	disp(full(A).operate(full(V)));

	Vector* vptr = new SparseVector();
	if (typeid(*vptr) == typeid(SparseVector)) {
		disp("vptr points to a sparse vector.");
	} else {
		disp("vptr points to a dense vector.");
	}

	vptr = new DenseVector();
	if (typeid(*vptr) == typeid(SparseVector)) {
		disp("vptr points to a sparse vector.");
	} else {
		disp("vptr points to a dense vector.");
	}

	Vector* vptr2 = new SparseVector[2];

	if (typeid(vptr2[0]) == typeid(SparseVector)) {
		disp("vptr2[0] points to a sparse vector.");
	} else {
		disp("vptr2[0] points to a dense vector.");
	}

	if (typeid(((SparseVector*) vptr2)[1]) != typeid(SparseVector)) {
		err("vptr2 should point to a sparse vector array.");
	} else {
		disp("Good vptr2:)");
	}

	// SparseVector* vptr3 = vptr2;
	/* A vector pointer cannot be conversed to sparse vector pointer or
	 * dense vector pointer.
	 */

	Vector* Vs = sparseMatrix2SparseRowVectors0(A);
	for (int i = 0; i < A.getRowDimension(); i++) {
		fprintf("Vs[%d]:\n", i);
		disp(((SparseVector*) Vs)[i]);
	}

	Matrix& S = sparseRowVectors2SparseMatrix0(Vs, A.getRowDimension());
	disp("S:");
	printMatrix(S);

	for (int i = 0; i < A.getRowDimension(); i++) {
		delete &((SparseVector*) Vs)[i];
	}
	// delete[] Vs;
	delete &S;

	Vector** Vss = sparseMatrix2SparseRowVectors(A);
	for (int i = 0; i < A.getRowDimension(); i++) {
		fprintf("Vss[%d]:\n", i);
		disp(*Vss[i]);
	}

	Matrix& S2 = sparseRowVectors2SparseMatrix(Vss, A.getRowDimension());
	disp("S2:");
	printMatrix(S2);

	Vector* Vss1 = Vss[1];
	Vss[1] = &full(*Vss[1]);
	delete Vss1;
	for (int i = 0; i < A.getRowDimension(); i++) {
		fprintf("Vss[%d]:\n", i);
		disp(*Vss[i]);
	}

	for (int i = 0; i < A.getRowDimension(); i++) {
		delete Vss[i];
	}
	delete[] Vss;
	delete &S2;

	disp("Mission complete.");

	return EXIT_SUCCESS;

}




