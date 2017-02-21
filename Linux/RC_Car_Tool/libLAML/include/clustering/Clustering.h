/*
 * Clustering.h
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include "Matrix.h"
#include "Printer.h"
#include "Utility.h"
#include "Matlab.h"
#include "Options.h"

/**
 * Abstract class for clustering algorithms.
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 1st, 2014
 */
class Clustering {

public:

	/**
	 * Number of clusters.
	 */
	int nClus;

	/**
	 * Number of features.
	 */
	int nFeature;

	/**
	 * Number of examples.
	 */
	int nExample;

protected:

	/**
	 * Data matrix (nExample x nFeature), each row is a feature vector
	 */
	Matrix* dataMatrix;

	/**
	 * Cluster indicator matrix (nExample x nClus).
	 */
	Matrix* indicatorMatrix;

	/**
	 * Basis matrix (nClus x nFeature).
	 */
	Matrix* centers;

public:

	/**
	 * Default constructor for this clustering algorithm.
	 */
	Clustering();

	virtual ~Clustering();

	/**
	 * Constructor for this clustering algorithm initialized with options
	 * wrapped in a {@code ClusteringOptions} object.
	 *
	 * @param clusteringOptions clustering options
	 *
	 */
	Clustering(ClusteringOptions& clusteringOptions);

	/**
	 * Constructor for this clustering algorithm given number of
	 * clusters to be set.
	 *
	 * @param nClus number of clusters
	 *
	 */
	Clustering(int nClus);

	/**
	 * Feed training data for this clustering algorithm.
	 *
	 * @param dataMatrix an nExample x nFeature data matrix with each row being
	 *                   a data example
	 *
	 */
	void feedData(Matrix& dataMatrix);

	/**
	 * Feed training data for this feature selection algorithm.
	 *
	 * @param data an nExample x nFeature 2D {@code double} array with each
	 *             row being a data example
	 * @param n number of training examples
	 *
	 * @param d number of features
	 */
	void feedData(double** data, int n, int d);

	template<int d>
	void feedData(double data[][d], int n) {
		feedData(DenseMatrix::createDenseMatrix<d>(data, n));
	}

	template<int numFeature>
	void feedData(double data[][numFeature], int n, int d) {
		feedData(DenseMatrix::createDenseMatrix<numFeature>(data, n, d));
	}

	/**
	 * Initialize the indicator matrix.
	 *
	 * @param G0 initial indicator matrix
	 *
	 */
	virtual void initialize(Matrix* G0);

	/**
	 * Do clustering. Please call initialize() before
	 * using this method.
	 */
	virtual void clustering() = 0;

	/**
	 * Do clustering with a specified initializer. Please use null if
	 * you want to use random initialization.
	 *
	 * @param G0 initial indicator matrix, if null random initialization
	 *           will be used
	 */
	void clustering(Matrix* G0);

	/**
	 * Fetch data matrix.
	 *
	 * @return an nExample x nFeature data matrix
	 *
	 */
	Matrix* getData() {
		return dataMatrix;
	}

	/**
	 * Get cluster centers.
	 *
	 * @return an nClus x nFeature basis matrix
	 *
	 */
	Matrix* getCenters() {
		return centers;
	}

	/**
	 * Get cluster indicator matrix.
	 *
	 * @return an nExample x nClus cluster indicator matrix
	 *
	 */
	Matrix* getIndicatorMatrix() {
		return indicatorMatrix;
	}

	/**
	 * Evaluating the clustering performance of this clustering algorithm
	 * by using the ground truth.
	 *
	 * @param G predicted cluster indicator matrix
	 *
	 * @param groundTruth true cluster assignments
	 *
	 * @return evaluation metrics
	 *
	 */
	static double getAccuracy(Matrix& G, Matrix& groundTruth) {
		// To do
		err("Sorry, this function has not been implemented yet...");
		return 0;
	}

};

#endif /* CLUSTERING_H_ */
