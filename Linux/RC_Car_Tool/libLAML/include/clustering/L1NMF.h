/*
 * L1NMF.h
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#ifndef L1NMF_H_
#define L1NMF_H_

#include "Utility.h"
#include "Printer.h"
#include "Matrix.h"
#include "Matlab.h"
#include "Clustering.h"
#include "InPlaceOperator.h"
#include "KMeans.h"
#include <cmath>

/***
 * A C++ implementation for L1NMF which solves the following
 * optimization problem:
 * <p>
 * min || X - G * F ||_F^2 + gamma * ||F||_{sav} + nu * ||G||_{sav}</br>
 * s.t. G >= 0, F >= 0
 * </p>
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 1st, 2014
 */
class L1NMF : public Clustering {

public:

	double epsilon;
	int maxIter;

	double gamma;
	double mu;

	bool calc_OV;
	bool verbose;

	std::list<double> valueList;

	Matrix* initializer;

	L1NMF(Options& options);

	L1NMF(L1NMFOptions& L1NMFOptions);

	L1NMF();

	void initialize(Matrix* G0);

	void clustering();

	void clustering(Matrix* G0);

private:

	void UpdateG(Matrix& X, Matrix& F, double mu, Matrix& G);

	void UpdateF(Matrix& X, Matrix& G, double gamma, Matrix& F);

	double f(Matrix& X, Matrix& F, Matrix& G, Matrix& E_F, Matrix& E_G);

};

#endif /* L1NMF_H_ */
