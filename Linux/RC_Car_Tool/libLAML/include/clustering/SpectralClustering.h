/*
 * SpectralClustering.h
 *
 *  Created on: Mar 2, 2014
 *      Author: Mingjie Qian
 */

#ifndef SPECTRALCLUSTERING_H_
#define SPECTRALCLUSTERING_H_

#include "Matrix.h"
#include "Clustering.h"
#include "Options.h"
#include <string>
#include "Printer.h"
#include "Utility.h"
#include "Matlab.h"
#include "Manifold.h"
#include "KMeans.h"

/***
 * A C++ implementation for spectral clustering.
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 2nd, 2014
 */
class SpectralClustering : public Clustering {

public:

	SpectralClusteringOptions* options;

	SpectralClustering() : Clustering() {
		options = new SpectralClusteringOptions();
	}

	SpectralClustering(int nClus) : Clustering(nClus) {
		options = new SpectralClusteringOptions(nClus);
	}

	SpectralClustering(ClusteringOptions& options) : Clustering(options) {
		this->options = new SpectralClusteringOptions(options);
	}

	SpectralClustering(SpectralClusteringOptions& options) {
		this->options = &options;
	}

	~SpectralClustering() {
		delete options;
	}

	/**
	 * For spectral clustering, we don't need initialization in the
	 * current implementation.
	 */
	void initialize(Matrix* G0) {};

	void clustering();

};


#endif /* SPECTRALCLUSTERING_H_ */
