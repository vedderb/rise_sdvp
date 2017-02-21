/*
 * NonlinearConjugateGradient.cpp
 *
 *  Created on: Feb 27, 2014
 *      Author: Mingjie Qian
 */

#include "NonlinearConjugateGradient.h"
#include "Utility.h"
#include "Printer.h"
#include "InPlaceOperator.h"
#include "Matlab.h"

/**
 * Current gradient.
 */
Matrix* NonlinearConjugateGradient::G = null;

/**
 * Last gradient.
 */
Matrix* NonlinearConjugateGradient::G_pre = null;

/**
 * Current matrix variable that we want to optimize.
 */
Matrix* NonlinearConjugateGradient::X = null;

/**
 * Decreasing step.
 */
Matrix* NonlinearConjugateGradient::p = null;

/**
 * The last objective function value.
 */
double NonlinearConjugateGradient::fval = 0;

/**
 * If gradient is required for the next step.
 */
bool NonlinearConjugateGradient::gradientRequired = false;

/**
 * If the algorithm converges or not.
 */
bool NonlinearConjugateGradient::converge = false;

/**
 * State for the automata machine.
 * 0: Initialization
 * 1: Before backtracking line search
 * 2: Backtracking line search
 * 3: After backtracking line search
 * 4: Convergence
 */
int NonlinearConjugateGradient::state = 0;

/**
 * Step length for backtracking line search.
 */
double NonlinearConjugateGradient::t = 1;

/**
 * A temporary variable holding the inner product of the decreasing step p
 * and the gradient G, it should be always non-positive.
 */
double NonlinearConjugateGradient::z = 0;

/**
 * Iteration counter.
 */
int NonlinearConjugateGradient::k = 0;

double NonlinearConjugateGradient::alpha = 0.05;

double NonlinearConjugateGradient::rou = 0.9;

/**
 * Formula used to calculate beta.
 * 1: FR
 * 2: PR
 * 3: PR+
 * 4: HS
 */
int NonlinearConjugateGradient::formula = 4;

/**
 * An array holding the sequence of objective function values.
 */
std::list<double> NonlinearConjugateGradient::J;

Matrix* NonlinearConjugateGradient::y_k = null;

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
void NonlinearConjugateGradient::run(Matrix& Grad_t, double fval_t, double epsilon, Matrix& X_t, bool* res) {
	// If the algorithm has converged, we do a new job
	if (state == 4) {
		delete X;
		X = null;
		delete G_pre;
		G_pre = null;
		delete G;
		G = null;
		delete p;
		p = null;
		delete y_k;
		y_k = null;
		J.clear();
		k = 0;
		state = 0;
	}

	if (state == 0) {

		X = &X_t.copy();
		/*if (Grad_t == null) {
			err("Gradient is required on the first call!");
			exit(1);
		}*/
		G = &Grad_t.copy();
		fval = fval_t;
		if (std::isnan(fval)) {
			err("Object function value is nan!");
			exit(1);
		}
		fprintf("Initial ofv: %g\n", fval);

		if (p == null)
			p = &uminus(*G);
		else
			uminus(*p, *G);

		state = 1;

	}

	if (state == 1) {

		double norm_Grad = norm(*G);
		if (norm_Grad < epsilon) {
			converge = true;
			gradientRequired = false;
			state = 4;
			fprintf("CG converges with norm(Grad) %f\n", norm_Grad);
			res[0] = converge;
			res[1] = gradientRequired;
			return;
			// return new boolean[] {converge, gradientRequired};
		}

		t = 1;
		// z is always less than 0
		z = innerProduct(*G, *p);

		state = 2;

		// X_t.setSubMatrix(plus(X, times(t, p)).getData(), 0, 0);
		// setMatrix(X_t, plus(X, times(t, p)));
		affine(X_t, *X, t, *p);

		converge = false;
		gradientRequired = false;
		res[0] = converge;
		res[1] = gradientRequired;
		return;
		// return new boolean[] {converge, gradientRequired};

	}

	// Backtracking line search
	if (state == 2) {

		converge = false;

		if (fval_t <= fval + alpha * t * z) {
			gradientRequired = true;
			state = 3;
		} else {
			t = rou * t;
			gradientRequired = false;
			// X_t.setSubMatrix(plus(X, times(t, p)).getData(), 0, 0);
			// setMatrix(X_t, plus(X, times(t, p)));
			affine(X_t, *X, t, *p);
		}

		// We don't need to compute X_t again since the X_t has already
		// satisfied the Armijo condition.
		// X_t.setSubMatrix(plus(X, times(t, p)).getData(), 0, 0);
		res[0] = converge;
		res[1] = gradientRequired;
		return;
		// return new boolean[] {converge, gradientRequired};

	}

	if (state == 3) {

		// X_pre = X.copy();

		// G_pre = G.copy();
		if (G_pre == null)
			G_pre = &G->copy();
		else
			assign(*G_pre, *G);

		/*if (Math.abs(fval_t - fval) < 1e-256) {
					converge = true;
					gradientRequired = false;
					out.printf("Objective function value doesn't decrease, iteration stopped!\n");
					fprintf("Iter %d, ofv: %g, norm(Grad): %g\n", k + 1, fval, norm(G));
					return new boolean[] {converge, gradientRequired};
				}*/

		fval = fval_t;
		J.push_back(fval);
		fprintf("Iter %d, ofv: %g, norm(Grad): %g\n", k + 1, fval, norm(*G));

		// X = X_t.copy();
		assign(*X, X_t);
		// G = Grad_t.copy();
		assign(*G, Grad_t);

		// Matrix y_k = null;
		if (y_k == null)
			y_k = &minus(*G, *G_pre);
		else
			minus(*y_k, *G, *G_pre);
		double beta = 0;
		switch (formula) {
		case 1:
			beta = innerProduct(*G, *G) / innerProduct(*G_pre, *G);
			break;
		case 2:
			beta = innerProduct(*G, *y_k) / innerProduct(*G_pre, *G_pre);
			break;
		case 3:
			beta = max(innerProduct(*G, *y_k) / innerProduct(*G_pre, *G_pre), 0);
			break;
		case 4:
			beta = innerProduct(*G, *y_k) / innerProduct(*y_k, *p);
			break;
		case 5:
			beta = innerProduct(*G, *G) / innerProduct(*y_k, *p);
			break;
		default:
			beta = innerProduct(*G, *y_k) / innerProduct(*y_k, *p);
			break;
		}

		// p_{k+1} = -G + beta * p_{k}
		// p = uminus(G).plus(times(beta, p));
		affine(*p, beta, *p, '-', *G);
		/*timesAssign(p, beta);
				minusAssign(p, G);*/

		k = k + 1;

		state = 1;

	}

	converge = false;
	gradientRequired = false;
	res[0] = converge;
	res[1] = gradientRequired;
	return;
	// return new boolean[] {converge, gradientRequired};

}


