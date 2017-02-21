/*
 * Distribution.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#ifndef DISTRIBUTION_H_
#define DISTRIBUTION_H_

#include "Matrix.h"
#include "Printer.h"
#include "Matlab.h"
#include <cmath>
#include <random>
#include <ctime>

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
Matrix& mvnrnd(Matrix& MU, Matrix& SIGMA, int cases);

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
Matrix& mvnrnd(double* MU, int d, double** SIGMA, int cases);

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
Matrix& mvnrnd(double* MU, int d, double* SIGMA, int cases);

#endif /* DISTRIBUTION_H_ */
