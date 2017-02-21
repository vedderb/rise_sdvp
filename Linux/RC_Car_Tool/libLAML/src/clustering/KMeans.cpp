/*
 * KMeans.cpp
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#include "KMeans.h"

KMeans::KMeans(int nClus) : Clustering(nClus) {
	options.maxIter = 100;
	options.verbose = false;
}

KMeans::KMeans(int nClus, int maxIter) : Clustering(nClus) {
	options.maxIter = maxIter;
	options.verbose = false;
}

KMeans::KMeans(int nClus, int maxIter, bool verbose) : Clustering(nClus) {
	options.maxIter = maxIter;
	options.verbose = verbose;
}

KMeans::KMeans(KMeansOptions& options) : Clustering(options.nClus) {
	// this->options = options;
	this->options.maxIter = options.maxIter;
	this->options.nClus = options.nClus;
	this->options.verbose = options.verbose;
}

KMeans::~KMeans() {
}

/**
 * Initializer needs not be explicitly specified. If the initial
 * indicator matrix is not given, random initialization will be
 * used.
 */
void KMeans::clustering() {

	if (indicatorMatrix == null) {
		initialize(null);
	}

	Vector** maxRes = max(*indicatorMatrix, 2);
	double* indicators = new double[nExample];
	for (int i = 0; i < nExample; i++) {
		if (maxRes[0]->get(i) != 0)
			indicators[i] = maxRes[1]->get(i);
		else
			indicators[i] = -1;
	}

	double* clusterSizes = allocate1DArray(nClus, 0);

	// double* indicators = full(max(indicatorMatrix, 2)[1]).getPr();
	Vector** examples = null;
	if (typeid(*dataMatrix) == typeid(SparseMatrix)) {
		examples = sparseMatrix2SparseRowVectors(*dataMatrix);
	} else {
		double** data = ((DenseMatrix*) dataMatrix)->getData();
		examples = new Vector*[nExample];
		for (int i = 0; i < nExample; i++) {
			examples[i] = new DenseVector(data[i], nFeature);
		}
	}

	Vector** centers = new Vector*[nClus];
	if (typeid(*dataMatrix) == typeid(DenseMatrix)) {
		for (int k = 0; k < nClus; k++) {
			centers[k] = new DenseVector(nFeature);
		}
	} else {
		for (int k = 0; k < nClus; k++) {
			centers[k] = new SparseVector(nFeature);
		}
	}

	// Compute the initial centers
	for (int i = 0; i < nExample; i++) {
		int k = (int) indicators[i];
		if (k == -1)
			continue;
		plusAssign(*centers[k], *examples[i]);
		clusterSizes[k]++;
	}
	for (int k = 0; k < nClus; k++) {
		timesAssign(*centers[k], 1 / clusterSizes[k]);
	}

	int cnt = 0;

	// Matrix* DistMatrix = null;
	double mse = 0;

	while (cnt < options.maxIter) {

		Matrix* indOld = indicatorMatrix;

		tic();

		Matrix& DistMatrix = l2DistanceSquare(centers, nClus, examples, nExample);
		// Printer.disp(DistMatrix);

		Vector** minRes = min(DistMatrix);
		Vector* minVals = minRes[0];
		Vector* IX = minRes[1];

		indicatorMatrix = new SparseMatrix(nExample, nClus);
		for (int i = 0; i < nExample; i++) {
			indicatorMatrix->setEntry(i, (int)IX->get(i), 1);
			indicators[i] = IX->get(i);
		}

		mse = sum(*minVals) / nExample;

		// Debug
		/*if (Double.isNaN(sse)) {
				int a = 1;
				a = a + 1;
				Matlab.display(DistMatrix);
				Matlab.display(dataMatrix.getColumnMatrix(6));
				Matlab.display(centers.getColumnMatrix(0));
				Matlab.display(Matlab.norm(dataMatrix.getColumnMatrix(6).minus(centers.getColumnMatrix(0))));
				Matlab.display(Matlab.l2Distance(dataMatrix.getColumnMatrix(6), centers));
				Matlab.display(Matlab.l2DistanceSquare(dataMatrix.getColumnMatrix(6), centers));
			}*/

		if (norm(indOld->minus(*indicatorMatrix), "fro") == 0) {
			println("KMeans complete.");
			break;
		}

		delete indOld;
		delete &DistMatrix;
		delete minRes[0];
		delete minRes[1];
		delete[] minRes;

		double elapsedTime = toc();

		cnt += 1;

		if (options.verbose) {
			fprintf("Iter %d: mse = %.3f (%.3f secs)\n", cnt, mse, elapsedTime);
		}

		// Compute the centers
		clear(clusterSizes, nClus);
		for (int k = 0; k < nClus; k++) {
			centers[k]->clear();
		}
		for (int i = 0; i < nExample; i++) {
			int k = (int) indicators[i];
			plusAssign(*centers[k], *examples[i]);
			clusterSizes[k]++;
		}
		for (int k = 0; k < nClus; k++) {
			timesAssign(*centers[k], 1 / clusterSizes[k]);
		}

	}

	if (typeid(*dataMatrix) == typeid(SparseMatrix)) {
		this->centers = &sparseRowVectors2SparseMatrix(centers, nClus);
	} else {
		this->centers = &denseRowVectors2DenseMatrix(centers, nClus);
	}

}


