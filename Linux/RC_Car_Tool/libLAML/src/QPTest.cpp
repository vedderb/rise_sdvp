/*
 * QPTest.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: Mingjie Qian
 */

#include <stdlib.h>
#include <limits>
#include "Classifier.h"
#include "Printer.h"
#include "DataSet.h"
#include "MyTime.h"
#include "Utility.h"
#include "GeneralQP.h"
#include "IO.h"
#include "Matlab.h"
#include "Matrix.h"
#include "SparseMatrix.h"
#include <iostream>

int main(void) {

	/*int a = -234;
	std::string str = int2str(a);
	std::cout << str << std::endl;

	a = -a;
	std::cout << int2str(a) << std::endl;

	Vector& d = *new DenseVector(3);
	d.set(0, 1);
	d.set(1, 2);
	d.set(2, 3);
	Matrix& D = diag(d);
	disp("D:");
	disp(D);
	Matrix& T0 = D.copy();
	disp(T0);
	Matrix& T = uminus(D);
	disp("T:");
	disp(T);
	double t = T.getEntry(2, 2);
	// disp(t);
	std::cout << t << std::endl;
	disp(size(T), 2);*/

	/*
	 * Number of unknown variables
	 */
	int n = 5;

	/*
	 * Number of inequality constraints
	 */
	int m = 6;

	/*
	 * Number of equality constraints
	 */
	int p = 3;

	/*Matrix x = rand(n, n);
			Matrix Q = x.mtimes(x.transpose()).plus(times(rand(1), eye(n)));
			Matrix c = rand(n, 1);

			double HasEquality = 1;
			Matrix A = times(HasEquality, rand(p, n));
			x = rand(n, 1);
			Matrix b = A.mtimes(x);
			Matrix B = rand(m, n);
			double rou = -2;
			Matrix d = plus(B.mtimes(x), times(rou, ones(m, 1)));*/

	double rou = -2;
	double HasEquality = 1;

	bool generate = false;
	if (generate) {
		Matrix& x = rand(n, n);
		Matrix& Q = x.mtimes(x.transpose()).plus(times(rand(1), eye(n)));
		Matrix& c = rand(n, 1);

		Matrix& A = times(HasEquality, rand(p, n));
		Matrix& y = rand(n, 1);
		Matrix& b = A.mtimes(y);
		Matrix& B = rand(m, n);
		Matrix& d = plus(B.mtimes(y), times(rou, ones(m, 1)));

		/*
		 * General quadratic programming:
		 *
		 *      min 2 \ x' * Q * x + c' * x
		 * s.t. A * x = b
		 *      B * x <= d
		 */
		GeneralQP::solve(Q, c, A, b, B, d);

		saveMatrix("Q", Q);
		saveMatrix("c", c);
		saveMatrix("A", A);
		saveMatrix("b2", b);
		saveMatrix("B", B);
		saveMatrix("d", d);
	} else {
		Matrix& Q = loadMatrix("Q");
		Matrix& c = loadMatrix("c");
		Matrix& A = loadMatrix("A");
		Matrix& b = loadMatrix("b2");
		Matrix& B = loadMatrix("B");
		Matrix& d = loadMatrix("d");

		/*
		 * General quadratic programming:
		 *
		 *      min 2 \ x' * Q * x + c' * x
		 * s.t. A * x = b
		 *      B * x <= d
		 */
		GeneralQP::solve(Q, c, A, b, B, d);

		/*Matrix* Q = &loadMatrix("Q");
		// Matrix* c = &loadMatrix("c");
		Matrix* A = &loadMatrix("A");
		Matrix* b = &loadMatrix("b2");
		Matrix* B = &loadMatrix("B");
		Matrix* d = &loadMatrix("d");

		 * Number of unknown variables

		int n = A->getColumnDimension();


		 * Number of equality constraints

		int p = A->getRowDimension();


		 * Number of inequality constraints

		int m = B->getRowDimension();

		Matrix* A_ori = A;
		Matrix* B_ori = B;
		Matrix* d_ori = d;

		Matrix* zerosMat = &zeros(n, 1);
			Matrix* onesMat = &ones(m, 1);

		Matrix& c = vertcat(2, &zeros(n, 1), &ones(m, 1));
		A = &horzcat(2, A, &zeros(p, m));
		B = &vertcat(2, &horzcat(2, B, &uminus(eye(m))), &horzcat(2, &zeros(m, n), &uminus(eye(m))));
		d = &vertcat(2, d, &zeros(m, 1));

		int n_ori = n;
		int m_ori = m;

		n = n + m;
		m = 2 * m;

		// Matrix x0 = rand(n_ori, 1);
		Matrix* x0 = &ones(n_ori, 1);
		Matrix* s0 = &B_ori->mtimes(*x0).minus(*d_ori).plus(ones(m_ori, 1));
		x0 = &vertcat(2, x0, s0);
		Matrix* v0 = &zeros(p, 1);

		// Parameter setting

		double mu = 1.8;
		double epsilon = 1e-10;
		double epsilon_feas = 1e-10;
		double alpha = 0.1;
		double beta = 0.98;

		tic();

		Matrix* l0 = &rdivide(ones(m, 1), m);

		Matrix& x = *x0;
		Matrix& l = *l0;
		Matrix& v = *v0;

		Matrix& F_x_0 = B->mtimes(x).minus(*d);

		double eta_t = -innerProduct(F_x_0, *l0);
		double t = 1;
		double f_x = 0;
		Matrix* G_f_x = null;
		Matrix* F_x = null;
		Matrix* DF_x = null;
		Matrix* H_x = &times(1e-10, eye(n));
		Matrix* r_prim = null;
		Matrix* r_dual = null;
		Matrix* r_cent = null;
		Matrix* matrix = null;
		Matrix* vector = null;

		double residual = 0;
		double residual_prim = 0;
		double residual_dual = 0;

		Matrix* z_pd = null;
		Matrix* x_nt = null;
		Matrix* l_nt = null;
		Matrix* v_nt = null;

		Matrix* x_s = &zeros(size(*x0));
		Matrix* l_s = &zeros(size(*l0));
		Matrix* v_s = &zeros(size(*v0));

		double s = 0;
		Matrix* G_f_x_s = null;
		Matrix* F_x_s = null;
		Matrix* DF_x_s = null;

		Matrix* r_prim_s = null;
		Matrix* r_dual_s = null;
		Matrix* r_cent_s = null;
		double residual_s = 0;

		//
		t = mu * m / eta_t;
		f_x = innerProduct(c, x);

		// Calculate the gradient of f(x)
		G_f_x = &c;

		F_x = &(B->mtimes(x).minus(*d));
		DF_x = B;

		r_prim = &A->mtimes(x).minus(*b);
		r_dual = &G_f_x->plus(DF_x->transpose().mtimes(l)).plus(A->transpose().mtimes(v));
		r_cent = &uminus(times(l, *F_x)).minus(rdivide(ones(m, 1), t));

		disp("l:");
		disp(l);
		Vector& V1 = sum(l, 2);
		disp("V1:");
		disp(V1);
		Matrix& T1 = diag(V1);
		disp("T1:");
		disp(T1);
		double* pr = new double[2];
		SparseMatrix* res = new SparseMatrix();
		Matrix& T11 = T1.copy();
		Matrix& T2 = mtimes(T11, *DF_x);
		Matrix& T3 = uminus(T2);

		Matrix& T = uminus(mtimes(diag(l), *DF_x));
		disp("T:");
		disp(T);

		matrix = &vertcat(3,
				&horzcat(3, H_x, &DF_x->transpose(), &A->transpose()),
				&horzcat(3, &uminus(mtimes(diag(l), *DF_x)), &uminus(diag(*F_x)), &zeros(m, p)),
				&horzcat(3, A, &zeros(p, m), &zeros(p, p))
		);
		vector = &uminus(vertcat(3, r_dual, r_cent, r_prim));

		disp(*matrix);*/
	}

	return EXIT_SUCCESS;

}


