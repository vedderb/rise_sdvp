/*
 * AcceleratedGradientDescent.h
 *
 *  Created on: Feb 26, 2014
 *      Author: Mingjie Qian
 */

#ifndef ACCELERATEDGRADIENTDESCENT_H_
#define ACCELERATEDGRADIENTDESCENT_H_

#include "ProximalMapping.h"
#include "Matrix.h"
#include <list>

/**
 * A C++ implementation for the accelerated gradient descent method.
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
 * &nbsp flags = AcceleratedGradientDescent.run(G, fval, epsilon, W); // Update W in place</br>
 * &nbsp if (flags[0]) // flags[0] indicates if it converges</br>
 * &nbsp &nbsp break; </br>
 * &nbsp fval = ...; // Compute the new objective function value at the updated W</br>
 * &nbsp if (flags[1])  // flags[1] indicates if gradient at the updated W is required</br>
 * &nbsp &nbsp G = ...; // Compute the gradient at the new W</br>
 * } </br>
 * </br>
 * </code>
 *
 * @version 1.0 Feb. 26th, 2014
 * @author Mingjie Qian
 */
class AcceleratedGradientDescent {

private:

	/**
	 * Current gradient.
	 */
	static Matrix* Grad_Y_k;

	/**
	 * Current matrix variable that we want to optimize.
	 */
	static Matrix* X;

	/**
	 * Last matrix variable that we want to optimize.
	 */
	static Matrix* X_pre;

	/**
	 * Current matrix variable that we want to optimize.
	 */
	static Matrix* Y;

	/**
	 * X_{k + 1} = prox_th(Y_k - t * Grad_Y_k) = y_k - t * G_Y_k.
	 */
	static Matrix* G_Y_k;

	/**
	 * g(Y_k).
	 */
	static double gval_Y_k;

	/**
	 * h(Y_k).
	 */
	static double hval_Y_k;

	/**
	 * f(Y_k) = g(Y_k) + h(Y_k).
	 */
	static double fval_Y_k;

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

	static double beta;

	/**
	 * Iteration counter.
	 */
	static int k;

	static double xi;

	/**
	 * An array holding the sequence of objective function values.
	 */
	static std::list<double> J;

	/**
	 * *T = minus(*Y, times(t, *Grad_Y_k))
	 */
	static Matrix* T;

	/**
	 * Proximity operator: prox_th(X).
	 */
	static ProximalMapping* prox;

public:

	~AcceleratedGradientDescent();

	/**
	 * 0: y_{k+1} = x_{k + 1} + (k / (k + 3)) * (x_{k + 1} - x_k)
	 * 1: y_{k+1} = x_{k + 1} + u * (x_{k + 1} - x_k),
	 *    u := 2 * (xi_{k + 1} - 1) / (1 + sqrt(1 + 4 * xi_{k + 1}^2))
	 */
	static int type;

	/**
	 * Main entry for the accelerated proximal gradient algorithm.
	 * The matrix variable to be optimized will be updated in place
	 * to a better solution point with lower objective function value.
	 *
	 * @param Grad_t gradient at X_t, required on the first revocation
	 *
	 * @param gval_t g(X_t)
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
	static void run(Matrix& Grad_t, double gval_t, double epsilon, Matrix& X_t, bool* res);

};

#endif /* ACCELERATEDGRADIENTDESCENT_H_ */
