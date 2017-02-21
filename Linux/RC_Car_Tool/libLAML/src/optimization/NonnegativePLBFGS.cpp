/*
 * NonnegativePNonnegativePLBFGS.cpp
 *
 *  Created on: Feb 23, 2014
 *      Author: Mingjie Qian
 */

#include "NonnegativePLBFGS.h"
#include <cstdlib>
#include <Matlab.h>
#include "InPlaceOperator.h"
#include "Printer.h"
#include "Matrix.h"

Matrix* NonnegativePLBFGS::G = NULL;

Matrix* NonnegativePLBFGS::PG = NULL;

Matrix* NonnegativePLBFGS::G_pre = NULL;

Matrix* NonnegativePLBFGS::X = NULL;

Matrix* NonnegativePLBFGS::X_pre = NULL;

Matrix* NonnegativePLBFGS::p = NULL;

double NonnegativePLBFGS::fval = 0;

bool NonnegativePLBFGS::gradientRequired = false;

bool NonnegativePLBFGS::converge = false;

int NonnegativePLBFGS::state = 0;

double NonnegativePLBFGS::t = 1;

double NonnegativePLBFGS::z = 0;

int NonnegativePLBFGS::k = 0;

double NonnegativePLBFGS::alpha = 0.2;

double NonnegativePLBFGS::beta = 0.75;

int NonnegativePLBFGS::m = 30;

double NonnegativePLBFGS::H = 0;

Matrix* NonnegativePLBFGS::s_k = NULL;

Matrix* NonnegativePLBFGS::y_k = NULL;

double NonnegativePLBFGS::rou_k = 0;

std::list<Matrix*> NonnegativePLBFGS::s_ks;
std::list<Matrix*> NonnegativePLBFGS::y_ks;
std::list<double> NonnegativePLBFGS::rou_ks;
std::list<double> NonnegativePLBFGS::J;

double NonnegativePLBFGS::tol = 1;

void NonnegativePLBFGS::run(Matrix& Grad_t, double fval_t, double epsilon, Matrix& X_t, bool* res) {
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
		delete G;
		G = null;
		delete PG;
		PG = null;
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

		tol = epsilon * norm(*G, inf);

		k = 0;
		state = 1;

	}

	if (state == 1) {

		Matrix* I_k = null;
		Matrix* I_k_com = null;

		Matrix& ltG = lt(*G, 0);
		Matrix& gtX = gt(*X, 0);
		I_k = &_or(ltG, gtX);
		delete &ltG;
		// I_k = &_or(lt(*G, 0), gt(*X, 0));
		I_k_com = &_not(*I_k);
		// PG = G.copy();
		if (PG == null)
			PG = &G->copy();
		else
			assign(*PG, *G);
		logicalIndexingAssignment(*PG, *I_k_com, 0);
		delete I_k;
		delete I_k_com;

		/*disp("PG:");
		disp(*PG);*/

		double norm_PGrad = norm(*PG, inf);
		if (norm_PGrad < tol) {
			converge = true;
			gradientRequired = false;
			state = 4;
			fprintf("PLBFGS converges with norm(PGrad) %f\n", norm_PGrad);
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

		Matrix* s_k_i = null;
		Matrix* y_k_i = null;
		double rou_k_i = 0;

		std::list<Matrix*>::iterator iter_s_ks;
		std::list<Matrix*>::iterator iter_y_ks;
		std::list<double>::iterator iter_rou_ks;

		double* a = new double[m];
		double b = 0;

		Matrix* q = null;
		Matrix* r = null;

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

		Matrix* HG = r;
		Matrix* PHG = &HG->copy();
		Matrix& ltHG = lt(*HG, 0);
		I_k = &_or(ltHG, gtX);
		// I_k = &_or(lt(*HG, 0), gt(*X, 0));
		I_k_com = &_not(*I_k);
		logicalIndexingAssignment(*PHG, *I_k_com, 0);
		delete I_k;
		delete I_k_com;
		delete r;
		delete &ltHG;
		delete &gtX;

		/*if (p != null)
			delete p;*/
		if (innerProduct(*PHG, *G) <= 0) {
			if (p == null)
				p = &uminus(*PG);
			else {
				assign(*p, *PG);
				uminusAssign(*p);
			}
		} else {
			if (p == null)
				p = &uminus(*PHG);
			else {
				assign(*p, *PHG);
				uminusAssign(*p);
			}
		}

		/*disp("p:");
		disp(*p);*/

		delete PHG;

		t = 1;
		// z is always less than 0
		// z = innerProduct(*G, *p);

		state = 2;

		// X_t.setSubMatrix(plus(X, times(t, p)).getData(), 0, 0);
		// setMatrix(X_t, plus(X, times(t, p)));
		affine(X_t, *X, t, *p);
		/*disp("X_t:");
		disp(X_t);*/
		subplusAssign(X_t);
		/*disp("X_t^+:");
		disp(X_t);*/

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

		if (fval_t <= fval + alpha * innerProduct(*G, minus(X_t, *X))) {
			gradientRequired = true;
			state = 3;
		} else {
			t = beta * t;
			gradientRequired = false;
			// X_t.setSubMatrix(plus(X, times(t, p)).getData(), 0, 0);
			// setMatrix(X_t, plus(X, times(t, p)));
			affine(X_t, *X, t, *p);
			subplusAssign(X_t);
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
			fprintf("Iter %d, ofv: %g, norm(PGrad): %g\n", k + 1, fval, norm(*PG, inf));
			res[0] = converge;
			res[1] = gradientRequired;
			return;
			// return new boolean[] {converge, gradientRequired};
		}

		fval = fval_t;
		J.push_back(fval);
		fprintf("Iter %d, ofv: %g, norm(PGrad): %g\n", k + 1, fval, norm(*PG, inf));

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


