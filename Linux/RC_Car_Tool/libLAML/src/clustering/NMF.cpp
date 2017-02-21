/*
 * NMF.cpp
 *
 *  Created on: Mar 2, 2014
 *      Author: Mingjie Qian
 */

#include "NMF.h"

NMF::NMF(Options options) : L1NMF(options) {
	gamma = 0;
	mu = 0;
}

NMF::NMF(NMFOptions& NMFOptions) {
	nClus = NMFOptions.nClus;
	maxIter = NMFOptions.maxIter;
	epsilon = NMFOptions.epsilon;
	verbose = NMFOptions.verbose;
	calc_OV = NMFOptions.calc_OV;
	gamma = 0;
	mu = 0;
}

NMF::NMF() {
	Options& options = *new Options();
	nClus = options.nClus;
	maxIter = options.maxIter;
	epsilon = options.epsilon;
	verbose = options.verbose;
	calc_OV = options.calc_OV;
	gamma = 0;
	mu = 0;
}
