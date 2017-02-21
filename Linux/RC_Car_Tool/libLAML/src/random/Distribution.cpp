/*
 * Distribution.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#include "Distribution.h"

/**
 * Generate random samples chosen from the multivariate Gaussian
 * distribution with mean MU and covariance SIGMA.
 *
 * X ~ N(u, Lambda) => Y = B * X + v ~ N(B * u + v, B * Lambda * B')
 * Therefore, if X ~ N(0, Lambda),
 * then Y = B * X + MU ~ N(MU, B * Lambda * B').
 * We only need to do the eigen decomposition: SIGMA = B * Lambda * B'.
 *
 * @param MU 1 x d mean vector
 *
 * @param SIGMA covariance matrix
 *
 * @param cases number of d dimensional random samples
 *
 * @return cases-by-d sample matrix subject to the multivariate
 *         Gaussian distribution N(MU, SIGMA)
 *
 */
Matrix& mvnrnd(Matrix& MU, Matrix& SIGMA, int cases) {

	int d = MU.getColumnDimension();

	if (MU.getRowDimension() != 1) {
		errf("MU is expected to be 1 x %d matrix!\n", d);
	}

	if (norm(SIGMA.transpose().minus(SIGMA)) > 1e-10)
		errf("SIGMA should be a %d x %d real symmetric matrix!\n", d);

	Matrix** eigenDecompostion = eigs(SIGMA, d, "lm");

	Matrix& B = *eigenDecompostion[0];
	Matrix& Lambda = *eigenDecompostion[1];

	/*disp(B);
		disp(Lambda);*/

	Matrix& X = *new DenseMatrix(d, cases);
	std::default_random_engine generator(time(NULL));
	std::normal_distribution<double> normal(0.0, 1.0);
	double sigma = 0;
	for (int i = 0; i < d; i++) {
		sigma = Lambda.getEntry(i, i);
		if (sigma == 0) {
			X.setRowMatrix(i, zeros(1, cases));
			continue;
		}
		if (sigma < 0) {
			errf("Covariance matrix should be positive semi-definite!\n");
			exit(1);
		}
		for (int n = 0; n < cases; n++) {
			X.setEntry(i, n, normal(generator) * pow(sigma, 0.5));
		}
	}

	Matrix& Y = plus(mtimes(B, X), repmat(MU.transpose(), 1, cases)).transpose();

	return Y;

}

/**
 * Generate random samples chosen from the multivariate Gaussian
 * distribution with mean MU and covariance SIGMA.
 *
 * X ~ N(u, Lambda) => Y = B * X + v ~ N(B * u + v, B * Lambda * B')
 * Therefore, if X ~ N(0, Lambda),
 * then Y = B * X + MU ~ N(MU, B * Lambda * B').
 * We only need to do the eigen decomposition: SIGMA = B * Lambda * B'.
 *
 * @param MU a 1D {@code double} array holding the mean vector
 *
 * @param d dimensionality of the mean vector
 *
 * @param SIGMA a 2D {@code double} array holding the covariance matrix
 *
 * @param cases number of d dimensional random samples
 *
 * @return cases-by-d sample matrix subject to the multivariate
 *         Gaussian distribution N(MU, SIGMA)
 *
 */
Matrix& mvnrnd(double* MU, int d, double** SIGMA, int cases) {
	return mvnrnd(*new DenseMatrix(MU, d, 2), *new DenseMatrix(SIGMA, d, d), cases);
}

/**
 * Generate random samples chosen from the multivariate Gaussian
 * distribution with mean MU and a diagonal covariance SIGMA.
 *
 * X ~ N(u, Lambda) => Y = B * X + v ~ N(B * u + v, B * Lambda * B')
 * Therefore, if X ~ N(0, Lambda),
 * then Y = B * X + MU ~ N(MU, B * Lambda * B').
 * We only need to do the eigen decomposition: SIGMA = B * Lambda * B'.
 *
 * @param MU a 1D {@code double} array holding the mean vector
 *
 * @param d dimensionality of the mean vector
 *
 * @param SIGMA a 1D {@code double} array holding the diagonal elements
 *        of the covariance matrix
 *
 * @param cases number of d dimensional random samples
 *
 * @return cases-by-d sample matrix subject to the multivariate
 *         Gaussian distribution N(MU, SIGMA)
 *
 */
Matrix& mvnrnd(double* MU, int d, double* SIGMA, int cases) {
	return mvnrnd(*new DenseMatrix(MU, d, 2), diag(SIGMA, d), cases);
}
