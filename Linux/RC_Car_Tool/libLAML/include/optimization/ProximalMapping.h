/*
 * ProximalMapping.h
 *
 *  Created on: Feb 25, 2014
 *      Author: Mingjie Qian
 */

#ifndef PROXIMALMAPPING_H_
#define PROXIMALMAPPING_H_

#include "Matrix.h"

/**
 * Compute prox_th(X), which is defined by
 * prox_th(x) := argmin_u 1/2 * || u - x ||^2 + t * h(u).
 *
 * @author Mingjie Qian
 * @version 1.0, Feb. 25th, 2014
 */
class ProximalMapping {

public:

	ProximalMapping() {};

	virtual ~ProximalMapping() {};

	/**
	 * res = prox_th(X).
	 *
	 * @param res result matrix
	 *
	 * @param t a real scalar
	 *
	 * @param X a real matrix
	 */
	virtual void compute(Matrix& res, double t, Matrix& X) = 0;

};

/**
 * Compute prox_th(X) where h = 0.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class Prox : public ProximalMapping {

public:

	/**
	 * res = prox_th(X) where h = 0.
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
 * Compute prox_th(X) where h = || X ||_1.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class ProxL1 : public ProximalMapping {

public:

	/**
	 * res = prox_th(X) where h = || X ||_1.
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
 * Compute prox_th(X) where h = || X ||_F.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class ProxL2 : public ProximalMapping {

public:

	/**
	 * Compute prox_th(X) where h = || X ||_F. For a
	 * vector, h(X) is the l_2 norm of X, for a matrix
	 * h(X) is the Frobenius norm of X.
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
 * Compute prox_th(X) where h = || X ||_{\infty}.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class ProxLInfinity : public ProximalMapping {

public:

	/**
	 * Compute prox_th(X) where h = || X ||_{\infty}.
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
 * Compute prox_th(X) where h = I_+(X).
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2014
 */
class ProxPlus : public ProximalMapping {

public:

	/**
	 * Compute prox_th(X) where h = I_+(X).
	 *
	 * @param res result matrix
	 *
	 * @param t a real scalar
	 *
	 * @param X a real matrix
	 */
	void compute(Matrix& res, double t, Matrix& X);

};

/***
 * Soft-thresholding (shrinkage) operator, which is defined as
 * S_{t}[x] = argmin_u 1/2 * || u - x ||^2 + t||u||_1</br>
 * which is actually prox_{t||.||_1}(x). The analytical form is</br>
 * S_{t}[x] =</br>
 * | x - t, if x > t</br>
 * | x + t, if x < -t</br>
 * | 0, otherwise</br>
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 25th, 2013
 */
class ShrinkageOperator {

public:

	/**
	 * Soft-thresholding (shrinkage) operator, which is defined as
	 * S_{t}[x] = argmin_u 1/2 * || u - x ||^2 + t||u||_1</br>
	 * which is actually prox_{t||.||_1}(x). The analytical form is</br>
	 * S_{t}[x] =</br>
	 * | x - t, if x > t</br>
	 * | x + t, if x < -t</br>
	 * | 0, otherwise</br>
	 *
	 * @param res result matrix
	 *
	 * @param t threshold
	 *
	 * @param X a real matrix
	 */
	static void shrinkage(Matrix& res, double t, Matrix& X);
};

#endif /* PROXIMALMAPPING_H_ */
