/*
 * SpecClusTest.cpp
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
#include "SpectralClustering.h"
#include "DenseMatrix.h"

int main(void) {

	tic();

	int nClus = 2;
	bool verbose = false;
	int maxIter = 100;
	std::string graphType = "nn";
	double graphParam = 6;
	std::string graphDistanceFunction = "euclidean";
	std::string graphWeightType = "heat";
	double graphWeightParam = 1;
	ClusteringOptions& options = *new SpectralClusteringOptions(
			nClus,
			verbose,
			maxIter,
			graphType,
			graphParam,
			graphDistanceFunction,
			graphWeightType,
			graphWeightParam);

	Clustering& spectralClustering = *new SpectralClustering(options);

	/*std::string dataMatrixFilePath = "CNN - DocTermCount.txt";
			Matrix X = Data.loadMatrix(dataMatrixFilePath);*/

	double data[3][4] = {
			{3.5, 5.3, 0.2, -1.2},
			{4.4, 2.2, 0.3, 0.4},
			{1.3, 0.5, 4.1, 3.2}
	};
	/*Matrix X = new DenseMatrix(data);
			X = X.transpose();*/

	spectralClustering.feedData<4>(data, 3, 4);
	spectralClustering.clustering(null);
	display(full(*spectralClustering.getIndicatorMatrix()));

	/*std::string labelFilePath = "GroundTruth.txt";
			Matrix groundTruth = loadMatrix(labelFilePath);
			getAccuracy(spectralClustering.indicatorMatrix, groundTruth);*/

	fprintf("Elapsed time: %.3f seconds\n", toc());

	return EXIT_SUCCESS;

}


