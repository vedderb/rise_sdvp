/*
 * SpectralClustering.cpp
 *
 *  Created on: Mar 2, 2014
 *      Author: Mingjie Qian
 */

#include "SpectralClustering.h"

void SpectralClustering::clustering() {

	Matrix& X = *dataMatrix;
	std::string TYPE = options->graphType;
	double PARAM = options->graphParam;
	PARAM = ceil(log(size(X, 1)) + 1);
	if (PARAM == size(X, 1))
		PARAM--;
	std::string DISTANCEFUNCTION = options->graphDistanceFunction;
	Matrix& A0 = adjacencyDirected(X, TYPE, PARAM, DISTANCEFUNCTION);

	Vector** maxRes = max(A0, 2);
	Vector& Z = *maxRes[0];
	double WEIGHTPARAM = options->graphWeightParam;
	WEIGHTPARAM = sum(Z) / Z.getDim();
	delete maxRes[0];
	delete maxRes[1];
	delete[] maxRes;

	Matrix& A0T = A0.transpose();

	// A = max(A0, A.transpose());
	Matrix& A = max(A0, A0T);

	// W could be viewed as a similarity matrix
	Matrix& W = A.copy();
	delete &A0;
	delete &A0T;

	// Disassemble the sparse matrix
	FindResult& findResult = find(A);
	int* A_i = findResult.rows;
	int* A_j = findResult.cols;
	double* A_v = findResult.vals;
	int len = findResult.len;

	std::string WEIGHTTYPE = options->graphWeightType;

	if (WEIGHTTYPE == "distance") {
		for (int i = 0; i < len; i++) {
			W.setEntry(A_i[i], A_j[i], A_v[i]);
		}
	} else if (WEIGHTTYPE == "inner") {
		for (int i = 0; i < len; i++) {
			W.setEntry(A_i[i], A_j[i], 1 - A_v[i] / 2);
		}
	} else if (WEIGHTTYPE == "binary") {
		for (int i = 0; i < len; i++) {
			W.setEntry(A_i[i], A_j[i], 1);
		}
	} else if (WEIGHTTYPE == "heat") {
		double t = -2 * WEIGHTPARAM * WEIGHTPARAM;
		for (int i = 0; i < len; i++) {
			W.setEntry(A_i[i], A_j[i],
					exp(A_v[i] * A_v[i] / t));
		}
	} else {
		err("Unknown Weight Type.");
		exit(1);
	}

	// Construct L_sym
	Vector& D = sum(W, 2);
	Vector& sqrtD = sqrt(D);
	Vector& dotDivideSqrtD = dotDivide(1, sqrtD);
	// Matrix DSqrt = diag(dotDivide(1, sqrt(D)));
	Matrix& DSqrt = diag(dotDivideSqrtD);
	Matrix& I = speye(size(W, 1));
	Matrix& DSqrtW = DSqrt.mtimes(W);
	Matrix& DSqrtWDSqrt = DSqrtW.mtimes(DSqrt);
	// Matrix L_sym = speye(size(W, 1)).minus(Dsqrt.mtimes(W).mtimes(Dsqrt));
	Matrix& L_sym = I.minus(DSqrtWDSqrt);
	delete &D;
	delete &sqrtD;
	delete &dotDivideSqrtD;
	delete &I;
	delete &DSqrtW;
	delete &DSqrtWDSqrt;
	// System.out.println(L_sym.getEntry(34, 110));
	/*disp("L_sym:");
	disp(L_sym);*/
	Matrix** eigRes = eigs(L_sym, options->nClus, "sm");
	delete &L_sym;
	Matrix& V = *eigRes[0];// display(V);
	/*disp("V:");
	disp(V);;
	disp("DSqrt:");
	disp(DSqrt);*/
	Matrix& U = DSqrt.mtimes(V);// display(U);
	/*disp("U:");
	disp(U);*/

	KMeansOptions& kMeansOptions = *new KMeansOptions();
	kMeansOptions.nClus = options->nClus;
	kMeansOptions.maxIter = options->maxIter;
	kMeansOptions.verbose = options->verbose;

	Clustering& kmeans = *new KMeans(kMeansOptions);
	kmeans.feedData(U);
	kmeans.initialize(null);
	kmeans.clustering();
	this->indicatorMatrix = kmeans.getIndicatorMatrix();

	delete &DSqrt;
	delete &U;
	delete eigRes[0];
	delete eigRes[1];
	delete[] eigRes;
	println("Spectral clustering complete.");

}


