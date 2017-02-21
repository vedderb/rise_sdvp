/*
 * LBFGSForVectorForVector.cpp
 *
 *  Created on: Feb 22, 2014
 *      Author: Mingjie Qian
 */

#include "LBFGSForVector.h"
#include <cstdlib>
#include <Matlab.h>
#include "InPlaceOperator.h"
#include "Printer.h"
#include "Vector.h"

Vector* LBFGSForVector::G = NULL;

Vector* LBFGSForVector::G_pre = NULL;

Vector* LBFGSForVector::X = NULL;

Vector* LBFGSForVector::X_pre = NULL;

Vector* LBFGSForVector::p = NULL;

double LBFGSForVector::fval = 0;

bool LBFGSForVector::gradientRequired = false;

bool LBFGSForVector::converge = false;

int LBFGSForVector::state = 0;

double LBFGSForVector::t = 1;

double LBFGSForVector::z = 0;

int LBFGSForVector::k = 0;

double LBFGSForVector::alpha = 0.2;

double LBFGSForVector::beta = 0.75;

int LBFGSForVector::m = 30;

double LBFGSForVector::H = 0;

Vector* LBFGSForVector::s_k = NULL;

Vector* LBFGSForVector::y_k = NULL;

double LBFGSForVector::rou_k = 0;

std::list<Vector*> LBFGSForVector::s_ks;
std::list<Vector*> LBFGSForVector::y_ks;
std::list<double> LBFGSForVector::rou_ks;
std::list<double> LBFGSForVector::J;

void LBFGSForVector::run(Vector& Grad_t, double fval_t, double epsilon, Vector& X_t, bool* res) {
	// If the algorithm has converged, we do a new job
	if (state == 4) {
		s_ks.clear();
		y_ks.clear();
		rou_ks.clear();
		J.clear();
		delete X_pre;
		X_pre = null;
		delete G_pre;
		G_pre = null;
		delete X;
		X = null;
		delete p;
		p = null;
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

		k = 0;
		state = 1;

	}

	if (state == 1) {

		double norm_Grad = norm(*G, inf);
		if (norm_Grad < epsilon) {
			converge = true;
			gradientRequired = false;
			state = 4;
			fprintf("L-BFGS converges with norm(Grad) %f\n", norm_Grad);
			res[0] = converge;
			res[1] = gradientRequired;
			return;
			// return new boolean[] {converge, gradientRequired};
		}

		if (k == 0) {
			H = 1;
		} else {
			H = innerProduct(*s_k, *y_k) / innerProduct(*y_k, *y_k);
		}

		Vector* s_k_i = null;
		Vector* y_k_i = null;
		double rou_k_i = 0;

		std::list<Vector*>::iterator iter_s_ks;
		std::list<Vector*>::iterator iter_y_ks;
		std::list<double>::iterator iter_rou_ks;

		double* a = new double[m];
		double b = 0;

		Vector* q = null;
		Vector* r = null;

		q = &G->copy();
		iter_s_ks = s_ks.end();
		iter_y_ks = y_ks.end();
		iter_rou_ks = rou_ks.end();
		for (int i = s_ks.size() - 1; i >= 0; i--) {
			iter_s_ks--;
			iter_y_ks--;
			iter_rou_ks--;
			s_k_i = *iter_s_ks;
			y_k_i = *iter_y_ks;
			rou_k_i = *iter_rou_ks;
			/*disp("s_k_i:");
			disp(*s_k_i);
			disp("q:");
			disp(*q);*/
			a[i] = rou_k_i * innerProduct(*s_k_i, *q);
			// q = q.minus(times(a[i], y_k_i));
			minusAssign(*q, a[i], *y_k_i);
		}
		// r = times(H, q);
		r = q;
		timesAssign(*r, H);
		iter_s_ks = s_ks.begin();
		iter_y_ks = y_ks.begin();
		iter_rou_ks = rou_ks.begin();
		for (size_t i = 0; i < s_ks.size(); i++) {
			s_k_i = *(iter_s_ks++);
			y_k_i = *(iter_y_ks++);
			rou_k_i = *(iter_rou_ks++);
			b = rou_k_i * innerProduct(*y_k_i, *r);
			// r = r.plus(times(a[i] - b, s_k_i));
			plusAssign(*r, a[i] - b, *s_k_i);
		}
		// p is a decreasing step
		// p = uminus(r);
		if (p != null) {
			assign(*p, *r);
			delete r;
		} else
			p = r;

		uminusAssign(*p);

		t = 1;
		// z is always less than 0
		z = innerProduct(*G, *p);

		state = 2;

		// X_t.setSubVector(plus(X, times(t, p)).getData(), 0, 0);
		// setVector(X_t, plus(X, times(t, p)));
		affine(X_t, *X, t, *p);


		converge = false;
		gradientRequired = false;

		res[0] = converge;
		res[1] = gradientRequired;

		// delete q;
		// delete r;
		delete[] a;

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
			t = beta * t;
			gradientRequired = false;
			// X_t.setSubVector(plus(X, times(t, p)).getData(), 0, 0);
			// setVector(X_t, plus(X, times(t, p)));
			affine(X_t, *X, t, *p);
		}

		// We don't need to compute X_t again since the X_t has already
		// satisfied the Armijo condition.
		// X_t.setSubVector(plus(X, times(t, p)).getData(), 0, 0);
		res[0] = converge;
		res[1] = gradientRequired;
		return;
		// return new boolean[] {converge, gradientRequired};

	}

	if (state == 3) {

		// X_pre = X.copy();
		if (X_pre == null)
			X_pre = &X->copy();
		else
			assign(*X_pre, *X);
		// G_pre = G.copy();
		if (G_pre == null)
			G_pre = &G->copy();
		else
			assign(*G_pre, *G);

		if (fabs(fval_t - fval) < 1e-32) {
			converge = true;
			gradientRequired = false;
			print("Objective function value doesn't decrease, iteration stopped!\n");
			fprintf("Iter %d, ofv: %g, norm(Grad): %g\n", k + 1, fval, norm(*G, inf));
			res[0] = converge;
			res[1] = gradientRequired;
			return;
			// return new boolean[] {converge, gradientRequired};
		}

		fval = fval_t;
		J.push_back(fval);
		fprintf("Iter %d, ofv: %g, norm(Grad): %g\n", k + 1, fval, norm(*G, inf));

		// X = X_t.copy();
		assign(*X, X_t);
		// G = Grad_t.copy();
		assign(*G, Grad_t);

		// Now s_ks, y_ks, and rou_ks all have k elements
		if (k >= m) {
			s_k = s_ks.front();
			s_ks.pop_front();
			y_k = y_ks.front();
			y_ks.pop_front();
			rou_ks.pop_front();
			minus(*s_k, *X, *X_pre);
			minus(*y_k, *G, *G_pre);
		} else { // if (k < m)
			s_k = &X->minus(*X_pre);
			y_k = &G->minus(*G_pre);
		}
		rou_k = 1 / innerProduct(*y_k, *s_k);

		s_ks.push_back(s_k);
		y_ks.push_back(y_k);
		rou_ks.push_back(rou_k);

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


