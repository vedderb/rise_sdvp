/*
 * L1NMFTest.cpp
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#include "Utility.h"
#include "Printer.h"
#include "Options.h"
#include "Clustering.h"
#include "KMeans.h"
#include "Matlab.h"
#include "IO.h"
#include "L1NMF.h"
#include "DenseMatrix.h"

int main(void) {

	std::string filePath = "CNN-DocTermCountMatrix.txt";
	tic();
	Matrix* X = null;
	X = &loadMatrix(filePath);
	/*Matrix& Y = X->getSubMatrix(0, 29, 0, 999);
	saveMatrix(Y, "CNN-SubDocTermCountMatrix.txt");*/

	/*double data[3][4] = {
				{3.5, 5.3, 0.2, -1.2},
				{4.4, 2.2, 0.3, 0.4},
				{1.3, 0.5, 4.1, 3.2}
		};
	X = &DenseMatrix::createDenseMatrix<4>(data, 3, 4);*/

	X = &X->transpose();
	X = &getTFIDF(*X);
	X = &normalizeByColumns(*X);
	X = &X->transpose();

	KMeansOptions& kMeansOptions = *new KMeansOptions();
	kMeansOptions.nClus = 10;
	kMeansOptions.maxIter = 50;
	kMeansOptions.verbose = true;

	Clustering& kmeans = *new KMeans(kMeansOptions);
	kmeans.feedData(*X);
	// KMeans.initialize(null);
	kmeans.clustering();

	Matrix* G0 = kmeans.getIndicatorMatrix();
	/*disp("G0:");
	printMatrix(*G0);*/

	// Matrix X = Data.loadSparseMatrix("X.txt");
	// G0 = loadDenseMatrix("G0.txt");
	L1NMFOptions& l1NMFOptions = *new L1NMFOptions();
	l1NMFOptions.nClus = 10;
	l1NMFOptions.gamma = 1 * 0.0001;
	l1NMFOptions.mu = 1 * 0.1;
	l1NMFOptions.maxIter = 50;
	l1NMFOptions.verbose = true;
	l1NMFOptions.calc_OV = !true;
	l1NMFOptions.epsilon = 1e-5;
	Clustering& l1NMF = *new L1NMF(l1NMFOptions);
	l1NMF.feedData(*X);
	// L1NMF.initialize(G0);

	// Matlab takes 12.5413 seconds
	// jblas takes 29.368 seconds
	// Commons-math takes 129 seconds (Using Array2DRowRealMatrix)
	// Commons-math takes 115 seconds (Using DenseMatrix)
	// start = System.currentTimeMillis();

	l1NMF.clustering(G0); // Use null for random initialization

	fprintf("Elapsed time: %.3f seconds\n", toc());
	/*disp(*l1NMF.getCenters());
	disp(*l1NMF.getIndicatorMatrix());*/

	saveMatrix("F.txt", *l1NMF.getCenters());
	saveMatrix("G.txt", *l1NMF.getIndicatorMatrix());

	return EXIT_SUCCESS;

}


