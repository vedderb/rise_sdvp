/*
 * KMeans.h
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#ifndef KMEANS_H_
#define KMEANS_H_

#include "Clustering.h"
#include "Matrix.h"
#include "Matlab.h"
#include "ArrayOperator.h"
#include "InPlaceOperator.h"
#include "MyTime.h"
#include "Printer.h"
#include "Options.h"

/***
 * A C++ implementation for KMeans.
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 1st, 2014
 */
class KMeans : public Clustering {

private:

	KMeansOptions options;

public:

	KMeans(int nClus);

	KMeans(int nClus, int maxIter);

	KMeans(int nClus, int maxIter, bool verbose);

	KMeans(KMeansOptions& options);

	~KMeans();

	/**
	 * Initializer needs not be explicitly specified. If the initial
	 * indicator matrix is not given, random initialization will be
	 * used.
	 */
	void clustering();

};


#endif /* KMEANS_H_ */
