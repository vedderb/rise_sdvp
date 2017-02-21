/*
 * Options.h
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#ifndef OPTIONS_H_
#define OPTIONS_H_

#include <string>

class Options {

public:
	bool verbose;
	int nFeature;
	int nClass;
	int nTopic;
	int nTerm;
	int nDoc;
	int nTopTerm;
	double epsilon;
	int maxIter;

	double gamma;
	double mu;
	double lambda;

	bool calc_OV;

	int nClus;

	Options(Options& o);

	Options();

};

class ClusteringOptions {

public:

	int nClus;
	bool verbose;
	int maxIter;

	ClusteringOptions();

	ClusteringOptions(int nClus);

	ClusteringOptions(int nClus, bool verbose, int maxIter);

	ClusteringOptions(ClusteringOptions& options);

};

/**
 * A structure to build a data graph.
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
 *
 * @version 1.0 Mar. 1st, 2014
 * @author Mingjie Qian
 */
class GraphOptions {

public:
	std::string graphType;
	double graphParam;
	std::string kernelType;
	double kernelParam;
	std::string graphDistanceFunction;
	std::string graphWeightType;
	double graphWeightParam;
	bool graphNormalize;
	bool classEdges;

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
	GraphOptions();

	GraphOptions(GraphOptions& graphOtions);

};

class KMeansOptions {

public:

	int nClus;
	int maxIter;
	bool verbose;

	KMeansOptions();

	KMeansOptions(KMeansOptions& options);

	KMeansOptions(int nClus, int maxIter, bool verbose);

	KMeansOptions(int nClus, int maxIter);

};

class L1NMFOptions : public ClusteringOptions {

public:

	double gamma;
	double mu;
	double epsilon;
	bool calc_OV;

	L1NMFOptions();

	L1NMFOptions(L1NMFOptions& L1NMFOptions);

	L1NMFOptions(int nClus);

	L1NMFOptions(int nClus, bool verbose, int maxIter);

	L1NMFOptions(ClusteringOptions& options);

};

class NMFOptions : public ClusteringOptions {

public:

	double epsilon;
	bool calc_OV;

	NMFOptions();

	NMFOptions(NMFOptions& NMFOptions);

	NMFOptions(int nClus);

	NMFOptions(int nClus, bool verbose, int maxIter);

	NMFOptions(ClusteringOptions& options);

};

class SpectralClusteringOptions : public ClusteringOptions {

public:

	std::string graphType;
	double graphParam;
	std::string graphDistanceFunction;
	std::string graphWeightType;
	double graphWeightParam;

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
	SpectralClusteringOptions();

	SpectralClusteringOptions(int nClus);

	SpectralClusteringOptions(int nClus,
			bool verbose,
			int maxIter,
			std::string graphType,
			double graphParam,
			std::string graphDistanceFunction,
			std::string graphWeightType,
			double graphWeightParam);

	SpectralClusteringOptions(ClusteringOptions& clusteringOptions);

};

#endif /* OPTIONS_H_ */
