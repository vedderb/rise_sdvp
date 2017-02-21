/*
 * AGDTest.cpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Mingjie Qian
 */

#include "AcceleratedGradientDescent.h"
#include <cstdlib>
#include "Matlab.h"
#include "Printer.h"
#include "MyTime.h"
#include "InPlaceOperator.h"

int main(void) {

	int n = 10;
	Matrix& t = rand(n);
	Matrix& C = minus(t.mtimes(t.transpose()), times(0.1, eye(n)));
	Matrix& CT = C.transpose();
	Matrix& y = times(3, minus(0.5, rand(n, 1)));
	double epsilon = 1e-4;
	double gamma = 0.01;

	/*AcceleratedGradientDescent::prox = new ProxPlus();
	AcceleratedGradientDescent::type = 0;*/

	tic();

	/*
	 *      min_x || C * x - y ||_2 + gamma * || x ||_2
	 * s.t. x >= 0
	 *
	 * g(x) = || C * x - y ||_2 + gamma * || x ||_2
	 * h(x) = I_+(x)
	 */
	Matrix& x0 = rdivide(ones(n, 1), n);
	Matrix& x = x0.copy();

	Matrix& r_x = C.mtimes(x).minus(y);
	double f_x = 0;
	double phi_x = 0;
	double gval = 0;
	double hval = 0;
	double fval = 0;

	f_x = norm(r_x);
	phi_x = norm(x);
	gval = f_x + gamma * phi_x;
	hval = 0;
	fval = gval + hval;

	Matrix& Grad_f_x = rdivide(C.transpose().mtimes(r_x), f_x);
	Matrix& Grad_phi_x = rdivide(x, phi_x);
	Matrix& Grad = plus(Grad_f_x, times(gamma, Grad_phi_x));

	bool* flags = new bool[2];
	int k = 0;
	int maxIter = 10000;
	hval = 0;
	while (true) {

		AcceleratedGradientDescent::run(Grad, gval, epsilon, x, flags);
		// flags = AcceleratedGradientDescent.run(Grad, gval, epsilon, x);

		if (flags[0])
			break;

		if (sum(sum(isnan(x))) > 0) {
			int a = 1;
			a = a + 1;
		}

		/*
		 *  Compute the objective function value, if flags[1] is true
		 *  gradient will also be computed.
		 */
		affine(r_x, C, x, '-', y);
		// r_x = C.mtimes(x).minus(y);
		f_x = norm(r_x);
		phi_x = norm(x);
		gval = f_x + gamma * phi_x;
		hval = 0;
		fval = gval + hval;

		if (flags[1]) {

			k = k + 1;

			// Compute the gradient
			if (k > maxIter)
				break;

			mtimes(Grad_f_x, CT, r_x);
			rdivideAssign(Grad_f_x, f_x);
			// Grad_f_x = rdivide(C.transpose().mtimes(r_x), f_x);
			if (phi_x != 0)
				// Grad_phi_x = rdivide(x, phi_x);
				times(Grad_phi_x, 1 / phi_x, x);
			else
				// Grad_phi_x = times(0, Grad_phi_x);
				Grad_phi_x.clear();

			affine(Grad, Grad_f_x, gamma, Grad_phi_x);
			// Grad = plus(Grad_f_x, times(gamma, Grad_phi_x));

			/*if ( Math.abs(fval_pre - fval) < eps)
						break;
					fval_pre = fval;*/

		}

	}

	Matrix& x_accelerated_proximal_gradient = x;
	double f_accelerated_proximal_gradient = fval;
	fprintf("fval_accelerated_proximal_gradient: %g\n\n", f_accelerated_proximal_gradient);
	fprintf("x_accelerated_proximal_gradient:\n");
	display(x_accelerated_proximal_gradient.transpose());

	fprintf("Elapsed time: %.3f seconds\n", toc());

	return EXIT_SUCCESS;

}


