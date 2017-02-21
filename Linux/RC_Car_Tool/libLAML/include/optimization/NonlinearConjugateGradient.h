/*
 * NonlinearConjugateGradient.h
 *
 *  Created on: Feb 27, 2014
 *      Author: Mingjie Qian
 */

#ifndef NONLINEARCONJUGATEGRADIENT_H_
#define NONLINEARCONJUGATEGRADIENT_H_

#include "Matrix.h"
#include <list>

class NonlinearConjugateGradient {

private:

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

	static double rou;

	/**
	 * Formula used to calculate beta.
	 * 1: FR
	 * 2: PR
	 * 3: PR+
	 * 4: HS
	 */
	static int formula;

	/**
	 * An array holding the sequence of objective function values.
	 */
	static std::list<double> J;

	static Matrix* y_k;

public:

	/**
	 * Main entry for the algorithm. The matrix variable to be
	 * optimized will be updated in place to a better solution
	 * point with lower objective function value.
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
	 *            lower objective function value.
	 *
	 * @param res a {@code boolean} array with two elements: {converge, gradientRequired}
	 */
	static void run(Matrix& Grad_t, double fval_t, double epsilon, Matrix& X_t, bool* res);

};

#endif /* NONLINEARCONJUGATEGRADIENT_H_ */
