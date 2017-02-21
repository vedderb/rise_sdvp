/*
 * AcceleratedGradientDescent.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Mingjie Qian
 */

#include "AcceleratedGradientDescent.h"
#include "Utility.h"
#include "InPlaceOperator.h"
#include "Matlab.h"
#include "Printer.h"
#include <cmath>

Matrix* AcceleratedGradientDescent::Grad_Y_k = null;

Matrix* AcceleratedGradientDescent::X = null;

Matrix* AcceleratedGradientDescent::X_pre = null;

Matrix* AcceleratedGradientDescent::Y = null;

Matrix* AcceleratedGradientDescent::G_Y_k = null;

double AcceleratedGradientDescent::gval_Y_k = 0;

double AcceleratedGradientDescent::hval_Y_k = 0;

double AcceleratedGradientDescent::fval_Y_k = 0;

bool AcceleratedGradientDescent::gradientRequired = false;

bool AcceleratedGradientDescent::converge = false;

int AcceleratedGradientDescent::state = 0;

double AcceleratedGradientDescent::t = 1;

double AcceleratedGradientDescent::beta = 0.95;

int AcceleratedGradientDescent::k = 1;

double AcceleratedGradientDescent::xi = 1;

Matrix* AcceleratedGradientDescent::T = null;

int AcceleratedGradientDescent::type = 0;

std::list<double> AcceleratedGradientDescent::J;

ProximalMapping* AcceleratedGradientDescent::prox = new Prox();

AcceleratedGradientDescent::~AcceleratedGradientDescent() {
	if (prox != null)
		delete prox;
	prox = null;
	delete X;
	X = null;
	delete Y;
	Y = null;
	delete X_pre;
	X_pre = null;
	delete Grad_Y_k;
	Grad_Y_k = null;
	delete G_Y_k;
	G_Y_k = null;
	delete T;
	T = null;
}

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
void AcceleratedGradientDescent::run(Matrix& Grad_t, double gval_t, double epsilon, Matrix& X_t, bool* res) {
	// If the algorithm has converged, we do a new job
	if (state == 4) {
		J.clear();
		delete X;
		X = null;
		delete Y;
		Y = null;
		delete X_pre;
		X_pre = null;
		delete Grad_Y_k;
		Grad_Y_k = null;
		delete G_Y_k;
		G_Y_k = null;
		delete T;
		T = null;
		t = 1;
		k = 1;
		state = 0;
	}

	if (state == 0) {

		X = &X_t.copy();
		Y = &X_t.copy();

		T = &X_t.copy();

		gval_Y_k = gval_t;
		hval_Y_k = 0;
		fval_Y_k = gval_Y_k + hval_Y_k;
		if (std::isnan(fval_Y_k)) {
			err("Object function value is nan!");
			exit(1);
		}
		fprintf("Initial ofv: %g\n", fval_Y_k);

		k = 1;
		xi = 1;
		t = 1;
		state = 1;

	}

	if (state == 1) {

		/*if (Grad_t == null) {
			err("Gradient is required!");
			exit(1);
		}*/

		if (Grad_Y_k == null)
			Grad_Y_k = &Grad_t.copy();
		else
			assign(*Grad_Y_k, Grad_t);

		/*if (Grad_Y_k == null)
					Grad_Y_k = Grad_t.copy();
				else
					assign(Grad_Y_k, Grad_t);*/

		gval_Y_k = gval_t;
		hval_Y_k = 0;

		state = 2;

		// X_t.setSubMatrix(plus(X, times(t, p)).getData(), 0, 0);
		// setMatrix(X_t, prox.compute(t, minus(Y, times(t, Grad_Y_k))));
		affine(*T, *Y, -t, *Grad_Y_k);
		// prox->compute(X_t, t, minus(*Y, times(t, *Grad_Y_k)));
		prox->compute(X_t, t, *T);

		if (G_Y_k == null)
			G_Y_k = &Grad_t.copy();

		affine(*G_Y_k, 1 / t, *Y, -1 / t, X_t);
		// G_Y_k = rdivide(minus(Y, X_t), t);
		/*if (G_Y_k == null)
					G_Y_k = rdivide(minus(Y, X_t), t);
				else
					affine(G_Y_k, 1 / t, Y, -1 / t, X_t);*/

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

		if (gval_t <= gval_Y_k - t * innerProduct(*Grad_Y_k, *G_Y_k) + t / 2 * innerProduct(*G_Y_k, *G_Y_k) + eps) {
			gradientRequired = true;
			state = 3;
		} else {
			t = beta * t;
			gradientRequired = false;
			// setMatrix(X_t, prox->compute(t, minus(Y, times(t, Grad_Y_k))));
			affine(*T, *Y, -t, *Grad_Y_k);
			// prox->compute(X_t, t, minus(*Y, times(t, *Grad_Y_k)));
			prox->compute(X_t, t, *T);
			// G_Y_k = &rdivide(minus(*Y, X_t), t);
			affine(*G_Y_k, 1 / t, *Y, -1 / t, X_t);
			res[0] = converge;
			res[1] = gradientRequired;
			return;
			// return new boolean[] {converge, gradientRequired};
		}

	}

	if (state == 3) {

		double norm_G_Y = norm(*G_Y_k);

		if (norm_G_Y < epsilon) {
			converge = true;
			gradientRequired = false;
			state = 4;
			fprintf("Accelerated proximal gradient method converges with norm(G_Y_k) %f\n", norm_G_Y);
			res[0] = converge;
			res[1] = gradientRequired;
			return;
			// return new boolean[] {converge, gradientRequired};
		}

		fval_Y_k = gval_Y_k + hval_Y_k;
		J.push_back(fval_Y_k);
		fprintf("Iter %d, ofv: %g, norm(G_Y_k): %g\n", k, fval_Y_k, norm(*G_Y_k));

		// X_pre = X.copy();
		if (X_pre == null)
			X_pre = &X->copy();
		else
			assign(*X_pre, *X);

		// X = X_t.copy();
		assign(*X, X_t);

		if (type == 0) {
			double s = (double)(k) / (k + 3);
			// Y = plus(X, times((double)(k) / (k + 3), minus(X, X_pre)));
			affine(*Y, 1 + s, *X, -s, *X_pre);
		} else if (type == 1) {
			double u = 2 * (xi - 1) / (1 + sqrt(1 + 4 * xi * xi));
			affine(*Y, 1 + u, *X, -u, *X_pre);
		}

		// setMatrix(X_t, Y);
		assign(X_t, *Y);

		k = k + 1;

		state = 1;

	}

	converge = false;
	gradientRequired = true;
	res[0] = converge;
	res[1] = gradientRequired;
	return;
	// return new boolean[] {converge, gradientRequired};

}
