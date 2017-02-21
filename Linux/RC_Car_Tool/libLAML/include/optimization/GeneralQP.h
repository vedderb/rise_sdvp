/*
 * GeneralQP.h
 *
 *  Created on: Feb 24, 2014
 *      Author: Mingjie Qian
 */

#ifndef GENERALQP_H_
#define GENERALQP_H_

#include <cstdlib>
#include "Matrix.h"

class QPSolution {

public:

	Matrix* optimizer;

	Matrix* lambda_opt;

	Matrix* nu_opt;

	double optimum;

	QPSolution(Matrix* optimizer, Matrix* lambda_opt, Matrix* nu_opt, double optimum) {
		this->optimizer = optimizer;
		this->lambda_opt = lambda_opt;
		this->nu_opt = nu_opt;
		this->optimum = optimum;
	}

	~QPSolution() {
		delete optimizer;
		delete lambda_opt;
		delete nu_opt;
	}

};

class PhaseIResult {

public:

	bool feasible;

	Matrix* optimizer;

	double optimum;

	PhaseIResult(Matrix* optimizer, double optimum) {
		this->optimizer = optimizer;
		this->optimum = optimum;
	}

	PhaseIResult(bool feasible, Matrix* optimizer, double optimum) {
		this->feasible = feasible;
		this->optimizer = optimizer;
		this->optimum = optimum;
	}

	~PhaseIResult() {
		if (optimizer != NULL)
			delete optimizer;
	}

};

/**
 * General quadratic programming:
 * <p>
 *      min 2 \ x' * Q * x + c' * x </br>
 * s.t. A * x = b </br>
 *      B * x <= d </br>
 * </p>
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 24th, 2014
 */
class GeneralQP {

private:

	/**
	 * We demonstrate the implementation of phase I via primal-dual interior
	 * point method to test whether the following problem is feasible:
	 * </p>
	 *      min f(x) </br>
	 * s.t. A * x = b </br>
	 *      B * x <= d </br>
	 * </p>
	 * We seek the optimizer for the following phase I problem:
	 * </p>
	 *      min 1's </br>
	 * s.t. A * x = b </br>
	 *      B * x - d <= s </br>
	 *      s >= 0 </br>
	 * </p>
	 * <=> </br>
	 *      min cI'y </br>
	 * s.t. AI * y = b </br>
	 *      BI * y <= dI </br>
	 * </p>
	 * cI = [zeros(n, 1); ones(m, 1)] </br>
	 * AI = [A zeros(p, m)] </br>
	 * BI = [B -eye(m); zeros(m, n) -eye(m)] </br>
	 * dI = [d; zeros(m, 1)] </br>
	 * y = [x; s] </br>
	 *
	 * @param A a p x n real matrix
	 *
	 * @param b a p x 1 real matrix
	 *
	 * @param B an m x n real matrix
	 *
	 * @param d an m x 1 real matrix
	 *
	 * @return a {@code PhaseIResult} instance if feasible or null if infeasible
	 *
	 */
	static PhaseIResult& phaseI(Matrix* A, Matrix* b, Matrix* B, Matrix* d);

	/**
	 * Phase II for solving a general quadratic programming problem formulated as
	 * <p>
	 *      min 2 \ x' * Q * x + c' * x </br>
	 * s.t. A * x = b </br>
	 *      B * x <= d </br>
	 * </p>
	 *
	 * @param Q an n x n positive definite or semi-definite matrix
	 *
	 * @param c an n x 1 real matrix
	 *
	 * @param A a p x n real matrix
	 *
	 * @param b a p x 1 real matrix
	 *
	 * @param B an m x n real matrix
	 *
	 * @param d an m x 1 real matrix
	 *
	 * @param x0 starting point
	 *
	 * @return a {@code QPSolution} instance
	 *
	 */
	static QPSolution& phaseII(Matrix& Q, Matrix& c, Matrix& A, Matrix& b, Matrix& B, Matrix& d, Matrix& x0);

public:

	/**
	 * Solve a general quadratic programming problem formulated as
	 * <p>
	 *      min 2 \ x' * Q * x + c' * x </br>
	 * s.t. A * x = b </br>
	 *      B * x <= d </br>
	 * </p>
	 *
	 * @param Q an n x n positive definite or semi-definite matrix
	 *
	 * @param c an n x 1 real matrix
	 *
	 * @param A a p x n real matrix
	 *
	 * @param b a p x 1 real matrix
	 *
	 * @param B an m x n real matrix
	 *
	 * @param d an m x 1 real matrix
	 *
	 * @return a {@code QPSolution} instance if the general QP problems
	 *         is feasible or null otherwise
	 *
	 */
	static QPSolution& solve(Matrix& Q, Matrix& c, Matrix& A, Matrix& b, Matrix& B, Matrix& d);

};

#endif /* GENERALQP_H_ */
