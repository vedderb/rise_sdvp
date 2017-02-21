/*
 * Projection.h
 *
 *  Created on: Feb 25, 2014
 *      Author: Mingjie Qian
 */

#ifndef PROJECTION_H_
#define PROJECTION_H_

#include "Matrix.h"

/**
 * Compute proj_tC(X), which is defined by
 * proj_tC(x) := argmin_{u \in tC} 1/2 * || u - x ||^2.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class Projection {

public:

	/**
	 * res = proj_tC(X).
	 *
	 * @param res result matrix
	 *
	 * @param t a real scalar
	 *
	 * @param X a real matrix
	 */
	void compute(Matrix& res, double t, Matrix& X);

};

/**
 * Compute proj_tC(X) where C = {X: || X ||_1 <= 1}.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class ProjL1 : public Projection {

public:

	/**
	 * Compute proj_{tC}(X) where C = {X: || X ||_1 <= 1}.
	 *
	 * @param res result matrix
	 *
	 * @param t a nonnegative real scalar
	 *
	 * @param X a real matrix
	 */
	void compute(Matrix& res, double t, Matrix& X);

};

/**
 * Compute proj_tC(X) where C = {X: || X ||_2 <= 1}.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class ProjL2 : public Projection {

public:

	/**
	 * Compute proj_{tC}(X) where C = {X: || X ||_2 <= 1}.
	 *
	 * @param res result matrix
	 *
	 * @param t a nonnegative real scalar
	 *
	 * @param X a real column matrix
	 */
	void compute(Matrix& res, double t, Matrix& X);

};

/**
 * Compute proj_tC(X) where C = {X: || X ||_{\infty} <= 1}.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class ProjLInfinity : public Projection {

public:

	/**
	 * Compute proj_{tC}(X) where C = {X: || X ||_{\infty} <= 1}.
	 *
	 * @param res result matrix
	 *
	 * @param t a nonnegative real scalar
	 *
	 * @param X a real column matrix
	 */
	void compute(Matrix& res, double t, Matrix& X);

};

#endif /* PROJECTION_H_ */
