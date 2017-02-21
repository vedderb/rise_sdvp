/*
 * Clustering.cpp
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#include "Clustering.h"
#include <ctime>
#include <vector>

/**
 * Default constructor for this clustering algorithm.
 */
Clustering::Clustering() {
	this->nClus = 0;
	nExample = 0;
	nFeature = 0;
	dataMatrix = null;
	centers = null;
	indicatorMatrix = null;
}

Clustering::~Clustering() {
	delete this->centers;
	delete this->indicatorMatrix;
	this->dataMatrix = null;
}

/**
 * Constructor for this clustering algorithm initialized with options
 * wrapped in a {@code ClusteringOptions} object.
 *
 * @param clusteringOptions clustering options
 *
 */
Clustering::Clustering(ClusteringOptions& clusteringOptions) {
	this->nClus = clusteringOptions.nClus;
	nExample = 0;
	nFeature = 0;
	dataMatrix = null;
	centers = null;
	indicatorMatrix = null;
}

/**
 * Constructor for this clustering algorithm given number of
 * clusters to be set.
 *
 * @param nClus number of clusters
 *
 */
Clustering::Clustering(int nClus) {
	if (nClus < 1) {
		err("Number of clusters less than one!");
		exit(1);
	}
	this->nClus = nClus;
	nExample = 0;
	nFeature = 0;
	dataMatrix = null;
	centers = null;
	indicatorMatrix = null;
}

/**
 * Feed training data for this clustering algorithm.
 *
 * @param dataMatrix an nExample x nFeature data matrix with each row being
 *                   a data example
 *
 */
void Clustering::feedData(Matrix& dataMatrix) {
	this->dataMatrix = &dataMatrix;
	nExample = dataMatrix.getRowDimension();
	nFeature = dataMatrix.getColumnDimension();
}

/**
 * Feed training data for this feature selection algorithm.
 *
 * @param data an nExample x nFeature 2D {@code double} array with each
 *             row being a data example
 *@param n number of training examples
 *
 * @param d number of features
 */
void Clustering::feedData(double** data, int n, int d) {
	this->feedData(*new DenseMatrix(data, n, d));
}

/**
 * Initialize the indicator matrix.
 *
 * @param G0 initial indicator matrix
 *
 */
void Clustering::initialize(Matrix* G0) {

	if (G0 != null) {
		this->indicatorMatrix = G0;
		return;
	}
	// int* indList = randperm(nExample);
	int* indList = new int[nExample];
	std::srand(unsigned(std::time(0)));
		std::vector<int> myvector;
		for (int i = 0; i < nExample; i++) {
			myvector.push_back(i);
		}
		std::random_shuffle(myvector.begin(), myvector.end());
		int k = 0;
		for (std::vector<int>::iterator iter = myvector.begin(); iter != myvector.end(); iter++) {
			indList[k++] = *iter;
		}

	indicatorMatrix = new SparseMatrix(nExample, nClus);

	for (int i = 0; i < nClus; i++) {
		indicatorMatrix->setEntry(indList[i], i, 1);
	}

	delete[] indList;

}

/**
 * Do clustering with a specified initializer. Please use null if
 * you want to use random initialization.
 *
 * @param G0 initial indicator matrix, if null random initialization
 *           will be used
 */
void Clustering::clustering(Matrix* G0) {
	initialize(G0);
	clustering();
}
