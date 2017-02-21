/*
 * LBFGS.h
 *
 *  Created on: Feb 21, 2014
 *      Author: Mingjie Qian
 */

#ifndef LBFGS_H_
#define LBFGS_H_

#include "Matrix.h"
#include <list>

/**
 * A C++ implementation for the limited-memory BFGS algorithm.
 * It is a general algorithm interface, only gradient and objective
 * function value are needed to compute outside the class.
 * </p>
 * A simple example: </br></br>
 * <code>
 * double epsilon = ...; // Convergence tolerance</br>
 * Matrix W = ...; // Initial matrix (vector) you want to optimize</br>
 * Matrix G = ...; // Gradient at the initial matrix (vector) you want to optimize</br>
 * double fval = ...; // Initial objective function value</br>
 * </br>
 * boolean flags[] = null; </br>
 * while (true) { </br>
 * &nbsp flags = LBFGS.run(G, fval, epsilon, W); // Update W in place</br>
 * &nbsp if (flags[0]) // flags[0] indicates if L-BFGS converges</br>
 * &nbsp &nbsp break; </br>
 * &nbsp fval = ...; // Compute the new objective function value at the updated W</br>
 * &nbsp if (flags[1])  // flags[1] indicates if gradient at the updated W is required</br>
 * &nbsp &nbsp G = ...; // Compute the gradient at the new W</br>
 * } </br>
 * </br>
 * </code>
 *
 * @version 1.0 Feb. 22nd, 2014
 *
 * @author Mingjie Qian
 */
class LBFGS {

private :

	/**
	 * Current gradient.
	 */
	static Matrix* G;

	/**
	 * Last gradient.
	 */
	static Matrix* G_pre;

	/**
	 * Current matrix variable that we want to optimize.
	 */
	static Matrix* X;

	/**
	 * Last matrix variable that we want to optimize.
	 */
	static Matrix* X_pre;

	/**
	 * Decreasing step.
	 */
	static Matrix* p;

	/**
	 * The last objective function value.
	 */
	static double fval;

	/**
	 * If gradient is required for the next step.
	 */
	static bool gradientRequired;

	/**
	 * If the algorithm converges or not.
	 */
	static bool converge;

	/**
	 * State for the automata machine.
	 * 0: Initialization
	 * 1: Before backtracking line search
	 * 2: Backtracking line search
	 * 3: After backtracking line search
	 * 4: Convergence
	 */
	static int state;

	/**
	 * Step length for backtracking line search.
	 */
	static double t;

	/**
	 * A temporary variable holding the inner product of the decreasing step p
	 * and the gradient G, it should be always non-positive.
	 */
	static double z;

	/**
	 * Iteration counter.
	 */
	static int k;

	static double alpha;

	static double beta;

	static int m;

	static double H;

	static Matrix* s_k;
	static Matrix* y_k;
	static double rou_k;

	static std::list<Matrix*> s_ks;
	static std::list<Matrix*> y_ks;
	static std::list<double> rou_ks;

	/**
	 * An array holding the sequence of objective function values.
	 */
	static std::list<double> J;

public :

	/**
	 * Main entry for the LBFGS algorithm. The matrix variable to be
	 * optimized will be updated in place to a better solution point
	 * with lower objective function value.
	 *
	 * @param Grad_t gradient at original X_t, required on the
	 *               first revocation
	 *
	 * @param fval_t objective function value on original X_t
	 *
	 * @param epsilon convergence precision
	 *
	 * @param X_t current matrix variable to be optimized, will be
	 *            updated in place to a better solution point with
	 *            lower objective function value
	 *
	 * @param res a {@code boolean} array with two elements: {converge, gradientRequired}
	 *
	 */
	static void run(Matrix& Grad_t, double fval_t, double epsilon, Matrix& X_t, bool* res);

};


#endif /* LBFGS_H_ */
