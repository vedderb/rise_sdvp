/*
 * GeneralQP.cpp
 *
 *  Created on: Feb 24, 2014
 *      Author: Mingjie Qian
 */

#include "GeneralQP.h"
#include "Matlab.h"
#include "InPlaceOperator.h"
#include "Printer.h"
#include "MyTime.h"

PhaseIResult& GeneralQP::phaseI(Matrix* A, Matrix* b, Matrix* B, Matrix* d) {

	/*
	 * Number of unknown variables
	 */
	int n = A->getColumnDimension();

	/*
	 * Number of equality constraints
	 */
	int p = A->getRowDimension();

	/*
	 * Number of inequality constraints
	 */
	int m = B->getRowDimension();

	Matrix* A_ori = A;
	Matrix* B_ori = B;
	Matrix* d_ori = d;

	/*Matrix* zerosMat = &zeros(n, 1);
	Matrix* onesMat = &ones(m, 1);*/

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
	// int k = 0;
	while (true) {

		t = mu * m / eta_t;
		f_x = innerProduct(c, x);

		// Calculate the gradient of f(x)
		G_f_x = &c;

		// Calculate F(x) and DF(x)
		// Matrix& Bx = B->mtimes(x);
		/*disp("Bx:");
		disp(Bx);*/
		// Matrix& Bxmd = Bx.minus(*d);
		/*disp("d:");
		disp(*d);
		disp("Bxmd:");
		disp(Bxmd);*/
		F_x = &B->mtimes(x).minus(*d);
		DF_x = B;

		// Calculate the Hessian matrix of f(x) and fi(x)
		// H_x = times(1e-10, eye(n));

		r_prim = &A->mtimes(x).minus(*b);
		r_dual = &G_f_x->plus(DF_x->transpose().mtimes(l)).plus(A->transpose().mtimes(v));
		r_cent = &uminus(times(l, *F_x)).minus(rdivide(ones(m, 1), t));

		// Matrix& T1 = zeros(m, p);
		/*disp("T1:");
		disp(T1);*/
		// disp(*F_x);
		// Matrix& A1 = diag(*F_x);
		// disp("A1:");
		// disp(A1);
		// Matrix& T2 = A1.times(-1); // uminus(A1);
		// Matrix& T2 = diag(F_x->times(-1));
		/*disp("T2:");
		disp(T2);*/
		/*Matrix* C1 = &diag(l);
		// disp(*C1);
		Matrix& C2 = *DF_x;
		// disp(C2);
		Matrix& B1 = C1->mtimes(C2);
		disp("B1:");
		disp(B1);
		Matrix& T3 = uminus(B1);
		disp("T3:");
		disp(T3);*/
		/*Matrix& T = horzcat(3, &T3, &T2, &T1);
		disp("T:");
		disp(T);*/

		matrix = &vertcat(3,
				&horzcat(3, H_x, &DF_x->transpose(), &A->transpose()),
				&horzcat(3, &uminus(mtimes(diag(l), *DF_x)), &uminus(diag(*F_x)), &zeros(m, p)),
				&horzcat(3, A, &zeros(p, m), &zeros(p, p))
		);
		vector = &uminus(vertcat(3, r_dual, r_cent, r_prim));

		residual = norm(*vector);
		residual_prim = norm(*r_prim);
		residual_dual = norm(*r_dual);
		eta_t = -innerProduct(*F_x, l);

		// fprintf("f_x: %g, residual: %g\n", f_x, residual);
		if (residual_prim <= epsilon_feas &&
				residual_dual <= epsilon_feas &&
				eta_t <= epsilon) {
			fprintf("Terminate successfully.\n\n");
			break;
		}

		z_pd = &mldivide(*matrix, *vector);
		/*fprintf("k = %d%n", k++);
		        disp(z_pd.transpose());*/
		/*x_nt = z_pd.getSubMatrix(0, n - 1, 0, 0);
		        l_nt = z_pd.getSubMatrix(n, n + m - 1, 0, 0);
		        v_nt = z_pd.getSubMatrix(n + m, n + m + p - 1, 0, 0);*/
		/*x_nt = getRows(z_pd, 0, n - 1);
		l_nt = getRows(z_pd, n, n + m - 1);
		v_nt = getRows(z_pd, n + m, n + m + p - 1);*/
		x_nt = &z_pd->getRows(0, n - 1);
		l_nt = &z_pd->getRows(n, n + m - 1);
		v_nt = &z_pd->getRows(n + m, n + m + p - 1);

		// Backtracking line search

		s = 1;
		// Ensure lambda to be nonnegative
		while (true) {
			// l_s = plus(l, times(s, l_nt));
			affine(*l_s, s, *l_nt, '+', l);
			if (sumAll(lt(*l_s, 0)) > 0)
				s = beta * s;
			else
				break;
		}

		// Ensure f_i(x) <= 0, i = 1, 2, ..., m
		while (true) {
			// x_s = plus(x, times(s, x_nt));
			affine(*x_s, s, *x_nt, '+', x);
			if (sumAll(lt(d->minus(B->mtimes(*x_s)), 0)) > 0)
				s = beta * s;
			else
				break;
		}

		while (true) {

			/*x_s = plus(x, times(s, x_nt));
		        	l_s = plus(l, times(s, l_nt));
		        	v_s = plus(v, times(s, v_nt));*/
			affine(*x_s, s, *x_nt, '+', x);
			affine(*l_s, s, *l_nt, '+', l);
			affine(*v_s, s, *v_nt, '+', v);

			// Template {

			// Calculate the gradient of f(x_s)
			G_f_x_s = &c;

			// Calculate F(x_s) and DF(x_s)
			F_x_s = &B->mtimes(*x_s).minus(*d);
			DF_x_s = B;

			// }

			r_prim_s = &A->mtimes(*x_s).minus(*b);
			r_dual_s = &G_f_x_s->plus(DF_x_s->transpose().mtimes(*l_s)).plus(A->transpose().mtimes(*v_s));
			r_cent_s = &uminus(times(*l_s, *F_x_s)).minus(rdivide(ones(m, 1), t));

			residual_s = norm(vertcat(3, r_dual_s, r_cent_s, r_prim_s));
			if (residual_s <= (1 - alpha * s) * residual)
				break;
			else
				s = beta * s;

		}

		/*x = x_s;
		        l = l_s;
		        v = v_s;*/
		assign(x, *x_s);
		assign(l, *l_s);
		assign(v, *v_s);

	}

	double t_sum_of_inequalities = toc();

	Matrix& x_opt = x.getRows(0, n_ori - 1);
	fprintf("x_opt:\n");
	disp(x_opt.transpose());

	Matrix& s_opt = x.getRows(n_ori, n - 1);
	fprintf("s_opt:\n");
	disp(s_opt.transpose());

	Matrix& lambda_s = l.getRows(m_ori, m - 1);
	fprintf("lambda for the inequalities s_i >= 0:\n");
	disp(lambda_s.transpose());

	Matrix& e = B_ori->mtimes(x_opt).minus(*d_ori);
	fprintf("B * x - d:\n");
	disp(e.transpose());

	Matrix& lambda_ineq = l.getRows(0, m_ori - 1);
	fprintf("lambda for the inequalities fi(x) <= s_i:\n");
	disp(lambda_ineq.transpose());

	Matrix& v_opt = v;
	fprintf("nu for the equalities A * x = b:\n");
	disp(v_opt.transpose());

	fprintf("residual: %g\n\n", residual);
	fprintf("A * x - b:\n");
	disp(A_ori->mtimes(x_opt).minus(*b).transpose());
	fprintf("norm(A * x - b, \"fro\"): %f\n\n", norm(A_ori->mtimes(x_opt).minus(*b), "fro"));

	double fval_opt = f_x;
	fprintf("fval_opt: %g\n\n", fval_opt);
	bool feasible = false;
	if (fval_opt <= epsilon) {
		feasible = true;
		fprintf("The problem is feasible.\n\n");
	} else {
		feasible = false;
		fprintf("The problem is infeasible.\n\n");
	}
	fprintf("Computation time: %f seconds\n\n", t_sum_of_inequalities);

	/*if (!feasible)
			    return null;*/

	// x0 = x_opt;

	int pause_time = 1;
	fprintf("halt execution temporarily in %d seconds...\n\n", pause_time);
	// pause(pause_time);

	return *new PhaseIResult(feasible, &x_opt, fval_opt);
}

QPSolution& GeneralQP::phaseII(Matrix& Q, Matrix& c, Matrix& A, Matrix& b, Matrix& B, Matrix& d, Matrix& x0) {
	/*
	 * Number of unknown variables
	 */
	int n = A.getColumnDimension();

	/*
	 * Number of equality constraints
	 */
	int p = A.getRowDimension();

	/*
	 * Number of inequality constraints
	 */
	int m = B.getRowDimension();


	Matrix& v0 = zeros(p, 1);

	// Parameter setting

	double mu = 1.8;
	double epsilon = 1e-10;
	double epsilon_feas = 1e-10;
	double alpha = 0.1;
	double beta = 0.98;

	tic();

	Matrix& l0 = rdivide(ones(m, 1), m);

	Matrix& x = x0.copy();
	Matrix& l = l0;
	Matrix& v = v0;

	Matrix& F_x_0 = B.mtimes(x).minus(d);

	double eta_t = - innerProduct(F_x_0, l0);
	double t = 1;
	double f_x = 0;
	Matrix& G_f_x = zeros(n, 1);
	Matrix& F_x = zeros(m, 1);
	Matrix& DF_x = B;
	Matrix& H_x = Q;
	Matrix& r_prim = zeros(p, 1);
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

	Matrix& x_s = zeros(size(x0));
	Matrix& l_s = zeros(size(l0));
	Matrix& v_s = zeros(size(v0));

	double s = 0;
	Matrix& G_f_x_s = zeros(n, 1);
	Matrix& F_x_s = zeros(m, 1);
	Matrix& DF_x_s = B;

	Matrix& r_prim_s = zeros(p, 1);
	Matrix* r_dual_s = null;
	Matrix* r_cent_s = null;
	double residual_s = 0;

	while (true) {

		t = mu * m / eta_t;
		f_x = innerProduct(x, Q.mtimes(x)) / 2 + innerProduct(c, x);

		// Calculate the gradient of f(x)
		// G_f_x = Q.mtimes(x).plus(c);
		affine(G_f_x, Q, x, '+', c);

		// Calculate F(x) and DF(x)
		// F_x = B.mtimes(x).minus(d);
		affine(F_x, B, x, '-', d);

		// DF_x = B;

		// Calculate the Hessian matrix of f(x) and fi(x)
		// H_x = times(1e-10, eye(n));

		// r_prim = A.mtimes(x).minus(b);
		affine(r_prim, A, x, '-', b);
		r_dual = &G_f_x.plus(DF_x.transpose().mtimes(l)).plus(A.transpose().mtimes(v));
		r_cent = &uminus(times(l, F_x)).minus(rdivide(ones(m, 1), t));

		matrix = &vertcat(3,
				&horzcat(3, &H_x, &DF_x.transpose(), &A.transpose()),
				&horzcat(3, &uminus(mtimes(diag(l),DF_x)), &uminus(diag(F_x)), &zeros(m, p)),
				&horzcat(3, &A, &zeros(p, m), &zeros(p, p))
		);
		vector = &uminus(vertcat(3, r_dual, r_cent, &r_prim));

		residual = norm(*vector);
		residual_prim = norm(r_prim);
		residual_dual = norm(*r_dual);
		eta_t = -innerProduct(F_x, l);

		// fprintf("f_x: %g, residual: %g\n", f_x, residual);
		if (residual_prim <= epsilon_feas &&
				residual_dual <= epsilon_feas &&
				eta_t <= epsilon) {
			fprintf("Terminate successfully.\n\n");
			break;
		}

		z_pd = &mldivide(*matrix, *vector);
		/*x_nt = z_pd.getSubMatrix(0, n - 1, 0, 0);
		        l_nt = z_pd.getSubMatrix(n, n + m - 1, 0, 0);
		        v_nt = z_pd.getSubMatrix(n + m, n + m + p - 1, 0, 0);*/
		x_nt = &z_pd->getRows(0, n - 1);
		l_nt = &z_pd->getRows(n, n + m - 1);
		v_nt = &z_pd->getRows(n + m, n + m + p - 1);

		// Backtracking line search

		s = 1;
		// Ensure lambda to be nonnegative
		while (true) {
			// l_s = plus(l, times(s, l_nt));
			affine(l_s, s, *l_nt, '+', l);
			if (sumAll(lt(l_s, 0)) > 0)
				s = beta * s;
			else
				break;
		}

		// Ensure f_i(x) <= 0, i = 1, 2, ..., m
		while (true) {
			// x_s = plus(x, times(s, x_nt));
			affine(x_s, s, *x_nt, '+', x);
			if (sumAll(lt(d.minus(B.mtimes(x_s)), 0)) > 0)
				s = beta * s;
			else
				break;
		}

		while (true) {

			/*x_s = plus(x, times(s, x_nt));
		        	l_s = plus(l, times(s, l_nt));
		        	v_s = plus(v, times(s, v_nt));*/
			affine(x_s, s, *x_nt, '+', x);
			affine(l_s, s, *l_nt, '+', l);
			affine(v_s, s, *v_nt, '+', v);

			// Template {

			// Calculate the gradient of f(x_s)
			// G_f_x_s = Q.mtimes(x_s).plus(c);
			affine(G_f_x_s, Q, x_s, '+', c);

			// Calculate F(x_s) and DF(x_s)
			// F_x_s = B.mtimes(x_s).minus(d);
			affine(F_x_s, B, x_s, '-', d);
			// DF_x_s = B;

			// }

			// r_prim_s = A.mtimes(x_s).minus(b);
			affine(r_prim_s, A, x_s, '-', b);
			r_dual_s = &G_f_x_s.plus(DF_x_s.transpose().mtimes(l_s)).plus(A.transpose().mtimes(v_s));
			r_cent_s = &uminus(times(l_s, F_x_s)).minus(rdivide(ones(m, 1), t));

			residual_s = norm(vertcat(3, r_dual_s, r_cent_s, &r_prim_s));
			if (residual_s <= (1 - alpha * s) * residual)
				break;
			else
				s = beta * s;

		}

		/*x = x_s;
		        l = l_s;
		        v = v_s;*/
		assign(x, x_s);
		assign(l, l_s);
		assign(v, v_s);

	}

	double t_primal_dual_interior_point = toc();

	double fval_primal_dual_interior_point = f_x;
	Matrix& x_primal_dual_interior_point = x;
	Matrix& lambda_primal_dual_interior_point = l;
	Matrix& v_primal_dual_interior_point = v;

	fprintf("residual: %g\n\n", residual);
	fprintf("Optimal objective function value: %g\n\n", fval_primal_dual_interior_point);
	fprintf("Optimizer:\n");
	disp(x_primal_dual_interior_point.transpose());

	Matrix& e = B.mtimes(x).minus(d);
	fprintf("B * x - d:\n");
	disp(e.transpose());

	fprintf("lambda:\n");
	disp(lambda_primal_dual_interior_point.transpose());

	fprintf("nu:\n");
	disp(v_primal_dual_interior_point.transpose());

	fprintf("norm(A * x - b, \"fro\"): %f\n\n", norm(A.mtimes(x_primal_dual_interior_point).minus(b), "fro"));
	fprintf("Computation time: %f seconds\n\n", t_primal_dual_interior_point);

	return *new QPSolution(&x, &l, &v, f_x);
}

QPSolution& GeneralQP::solve(Matrix& Q, Matrix& c, Matrix& A, Matrix& b, Matrix& B, Matrix& d) {
	fprintf("Phase I:\n\n");
	PhaseIResult& phaseIResult = phaseI(&A, &b, &B, &d);
	if (phaseIResult.feasible) {
		fprintf("Phase II:\n\n");
		Matrix* x0 = phaseIResult.optimizer;
		return phaseII(Q, c, A, b, B, d, *x0);
	} else {
		err("The QP problem is infeasible!\n");
		exit(1);
	}
}
