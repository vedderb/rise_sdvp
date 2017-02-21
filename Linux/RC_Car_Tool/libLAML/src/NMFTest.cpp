/*
 * NMFTest.cpp
 *
 *  Created on: Mar 2, 2014
 *      Author: Mingjie Qian
 */

#include "Utility.h"
#include "Printer.h"
#include "Options.h"
#include "Clustering.h"
#include "KMeans.h"
#include "Matlab.h"
#include "IO.h"
#include "NMF.h"
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
	NMFOptions& nmfOptions = *new NMFOptions();
	nmfOptions.nClus = 10;
	nmfOptions.maxIter = 300;
	nmfOptions.verbose = true;
	nmfOptions.calc_OV = !true;
	nmfOptions.epsilon = 1e-5;
	Clustering& nmf = *new NMF(nmfOptions);
	nmf.feedData(*X);
	// L1NMF.initialize(G0);

	// Matlab takes 12.5413 seconds
	// jblas takes 29.368 seconds
	// Commons-math takes 129 seconds (Using Array2DRowRealMatrix)
	// Commons-math takes 115 seconds (Using DenseMatrix)
	// start = System.currentTimeMillis();

	nmf.clustering(G0); // Use null for random initialization

	fprintf("Elapsed time: %.3f seconds\n", toc());
	/*disp(*nmf.getCenters());
	disp(*nmf.getIndicatorMatrix());*/

	saveMatrix("F.txt", *nmf.getCenters());
	saveMatrix("G.txt", *nmf.getIndicatorMatrix());

	return EXIT_SUCCESS;

}


