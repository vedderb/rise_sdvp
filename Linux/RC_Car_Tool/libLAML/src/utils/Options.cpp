/*
 * Options.cpp
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#include "Options.h"
#include "Printer.h"
#include <typeinfo>

Options::Options(Options& o) {

	verbose = o.verbose;
	nFeature = o.nFeature;
	nClass = o.nClass;
	nTopic = o.nTopic;
	nTerm = o.nTerm;
	nDoc = o.nDoc;
	nClus = o.nClus;

	nTopTerm = o.nTopTerm;

	epsilon = o.epsilon;
	maxIter = o.maxIter;
	gamma = o.gamma;
	mu = o.mu;
	calc_OV = o.calc_OV;

	lambda = o.lambda;

}

Options::Options() {

	verbose = false;
	nFeature = 1;
	nClass = 1;
	nTopic = 1;
	nTerm = 1;
	nDoc = 0;

	nTopTerm = nTerm;

	epsilon = 1e-6;
	maxIter = 300;
	gamma = 0.0001;
	mu = 0.1;
	calc_OV = false;

	lambda = 1.0;

	nClus = 1;

}

ClusteringOptions::ClusteringOptions() {
	nClus = 0;
	verbose = false;
	maxIter = 100;
}

ClusteringOptions::ClusteringOptions(int nClus) {
	if (nClus < 1) {
		err("Number of clusters less than one!");
		exit(1);
	}
	this->nClus = nClus;
	verbose = false;
	maxIter = 100;
}

ClusteringOptions::ClusteringOptions(int nClus, bool verbose, int maxIter) {
	this->nClus = nClus;
	this->verbose = verbose;
	this->maxIter = maxIter;
}

ClusteringOptions::ClusteringOptions(ClusteringOptions& options) {
	nClus = options.nClus;
	verbose = options.verbose;
	maxIter = options.maxIter;
}

/**
 * Generate/alter {@code GraphOptions} structure to build a data graph.
 * <pre>
 *              Field Name:  Description                                         : default
 * -------------------------------------------------------------------------------------------
 *             'graphType':  'nn' | 'epsballs'                                   : 'nn'
 *            'graphParam':  number of nearest neighbor size of 'epsballs'       :  6
 *            'kernelType':  'linear' | 'rbf' | 'poly' | 'cosine'                : 'linear'
 *           'kernelParam':     --    | sigma | degree |    --                   :  1
 * 'graphDistanceFunction':  distance function for graph: 'euclidean' | 'cosine' : 'euclidean'
 *       'graphWeightType':  'binary' | 'distance' | 'heat'                      : 'binary'
 *      'graphWeightParam':  e.g. for heat kernel, width to use                  :  1
 *        'graphNormalize':  Use normalized graph Laplacian (1) or not (0)       :  1
 *            'classEdges':  Disconnect edges across classes:yes(1) no (0)       :  0
 *               'gamma_A':  RKHS norm regularization parameter (Ambient)        :  1
 *               'gamma_I':  Manifold regularization parameter  (Intrinsic)      :  1
 * -------------------------------------------------------------------------------------------
 * Note: Kernel and KernelParam are meant for calcKernel function.
 * </pre>
 */
GraphOptions::GraphOptions() {
	graphType = "nn";
	kernelType = "linear";
	kernelParam = 1;
	graphParam = 6;
	graphDistanceFunction = "euclidean";
	graphWeightType = "binary";
	graphWeightParam = 1;
	graphNormalize = true;
	classEdges = false;
}

GraphOptions::GraphOptions(GraphOptions& graphOtions) {
	graphType = graphOtions.graphType;
	kernelType = graphOtions.kernelType;
	kernelParam = graphOtions.kernelParam;
	graphParam = graphOtions.graphParam;
	graphDistanceFunction = graphOtions.graphDistanceFunction;
	graphWeightType = graphOtions.graphWeightType;
	graphWeightParam = graphOtions.graphWeightParam;
	graphNormalize = graphOtions.graphNormalize;
	classEdges = graphOtions.classEdges;
}

KMeansOptions::KMeansOptions() {
	nClus = -1;
	maxIter = 100;
	verbose = false;
}

KMeansOptions::KMeansOptions(KMeansOptions& options) {
	nClus = options.nClus;
	maxIter = options.maxIter;
	verbose = options.verbose;
}

KMeansOptions::KMeansOptions(int nClus, int maxIter, bool verbose) {
	this->nClus = nClus;
	this->maxIter = maxIter;
	this->verbose = verbose;
}

KMeansOptions::KMeansOptions(int nClus, int maxIter) {

	this->nClus = nClus;
	this->maxIter = maxIter;
	this->verbose = false;

}

L1NMFOptions::L1NMFOptions() : ClusteringOptions() {
	gamma = 0.0001;
	mu = 0.1;
	epsilon = 1e-6;
	calc_OV = false;
}

L1NMFOptions::L1NMFOptions(L1NMFOptions& L1NMFOptions) : ClusteringOptions((ClusteringOptions&) L1NMFOptions){
	gamma = L1NMFOptions.gamma;
	mu = L1NMFOptions.mu;
	epsilon = L1NMFOptions.epsilon;
	calc_OV = L1NMFOptions.calc_OV;
}

L1NMFOptions::L1NMFOptions(int nClus) : ClusteringOptions(nClus) {
	gamma = 0.0001;
	mu = 0.1;
	epsilon = 1e-6;
	calc_OV = false;
}

L1NMFOptions::L1NMFOptions(int nClus, bool verbose, int maxIter) : ClusteringOptions(nClus, verbose, maxIter) {
	gamma = 0.0001;
	mu = 0.1;
	epsilon = 1e-6;
	calc_OV = false;
}

L1NMFOptions::L1NMFOptions(ClusteringOptions& options) : ClusteringOptions(options) {
	gamma = 0.0001;
	mu = 0.1;
	epsilon = 1e-6;
	calc_OV = false;
}

NMFOptions::NMFOptions() : ClusteringOptions() {
	epsilon = 1e-6;
	calc_OV = false;
}

NMFOptions::NMFOptions(NMFOptions& NMFOptions) {
	ClusteringOptions((ClusteringOptions&) NMFOptions);
	epsilon = NMFOptions.epsilon;
	calc_OV = NMFOptions.calc_OV;
}

NMFOptions::NMFOptions(int nClus) : ClusteringOptions(nClus) {
	epsilon = 1e-6;
	calc_OV = false;
}

NMFOptions::NMFOptions(int nClus, bool verbose, int maxIter) : ClusteringOptions(nClus, verbose, maxIter) {
	epsilon = 1e-6;
	calc_OV = false;
}

NMFOptions::NMFOptions(ClusteringOptions& options) : ClusteringOptions(options) {
	epsilon = 1e-6;
	calc_OV = false;
}

/**
 * Constructor with default data member values:
 * <p>
 * graphType = "nn";</br>
 * graphParam = 6;</br>
 * graphDistanceFunction = "euclidean";</br>
 * graphWeightType = "heat";</br>
 * graphWeightParam = 1;</br>
 * </p>
 */
SpectralClusteringOptions::SpectralClusteringOptions() : ClusteringOptions() {
	graphType = "nn";
	graphParam = 6;
	graphDistanceFunction = "euclidean";
	graphWeightType = "heat";
	graphWeightParam = 1;
}

SpectralClusteringOptions::SpectralClusteringOptions(int nClus) : ClusteringOptions(nClus) {
	graphType = "nn";
	graphParam = 6;
	graphDistanceFunction = "euclidean";
	graphWeightType = "heat";
	graphWeightParam = 1;
}

SpectralClusteringOptions::SpectralClusteringOptions(int nClus,
		bool verbose,
		int maxIter,
		std::string graphType,
		double graphParam,
		std::string graphDistanceFunction,
		std::string graphWeightType,
		double graphWeightParam) : ClusteringOptions(nClus, verbose, maxIter) {
	this->graphType = graphType;
	this->graphParam = graphParam;
	this->graphDistanceFunction = graphDistanceFunction;
	this->graphWeightType = graphWeightType;
	this->graphWeightParam = graphWeightParam;
}

SpectralClusteringOptions::SpectralClusteringOptions(ClusteringOptions& clusteringOptions) : ClusteringOptions(clusteringOptions) {
	if (typeid(clusteringOptions) == typeid(SpectralClusteringOptions)) {
		SpectralClusteringOptions& options = (SpectralClusteringOptions&) clusteringOptions;
		graphType = options.graphType;
		graphParam = options.graphParam;
		graphDistanceFunction = options.graphDistanceFunction;
		graphWeightType = options.graphWeightType;
		graphWeightParam = options.graphWeightParam;
	} else {
		graphType = "nn";
		graphParam = 6;
		graphDistanceFunction = "euclidean";
		graphWeightType = "heat";
		graphWeightParam = 1;
	}

}
