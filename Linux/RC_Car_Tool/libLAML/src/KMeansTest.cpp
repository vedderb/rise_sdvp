/*
 * KMeansTest.cpp
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

int main(void) {

	int n = 10;
	int* indList = randperm(n);
	disp(indList, n);

	double data[3][4] = {
			{3.5, 5.3, 0.2, -1.2},
			{4.4, 2.2, 0.3, 0.4},
			{1.3, 0.5, 4.1, 3.2}
	};

	KMeansOptions& options = *new KMeansOptions();
	options.nClus = 2;
	options.verbose = true;
	options.maxIter = 100;

	Clustering& kmeans= *new KMeans(options);

	kmeans.feedData<4>(data, 3, 4);
	// KMeans.initialize(null);
	Matrix* initializer = null;
	initializer = new SparseMatrix(3, 2);
	initializer->setEntry(0, 0, 1);
	initializer->setEntry(1, 1, 1);
	initializer->setEntry(2, 0, 1);
	kmeans.clustering(initializer); // Use null for random initialization

	println("Indicator Matrix:");
	printMatrix(full(*kmeans.getIndicatorMatrix()));

	return EXIT_SUCCESS;

}


