/*
 * RobustPCA.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#ifndef ROBUSTPCA_H_
#define ROBUSTPCA_H_

#include "Matlab.h"
#include "ProximalMapping.h"
#include "Printer.h"
#include "InPlaceOperator.h"
#include "SVD.h"

/***
 * A C++ implementation of robust PCA which solves the
 * following convex optimization problem:
 * </br>
 * min ||A||_* + lambda ||E||_1</br>
 * s.t. D = A + E</br>
 * where ||.||_* denotes the nuclear norm of a matrix (i.e.,
 * the sum of its singular values), and ||.||_1 denotes the
 * sum of the absolute values of matrix entries.</br>
 * </br>
 * Inexact augmented Lagrange multipliers is used to solve the optimization
 * problem due to its empirically fast convergence speed and proved convergence
 * to the true optimal solution.
 * </br>
 * <b>Input:</b></br>
 *    D: an observation matrix</br>
 *    lambda: a positive weighting parameter</br>
 *
 * <b>Output:</b></br>
 *    A: a low-rank matrix recovered from the corrupted data matrix D</br>
 *    E: error matrix between D and A</br>
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 3rd, 2014
 */
class RobustPCA {

private:

	/**
	 * A positive weighting parameter.
	 */
	double lambda;

	/**
	 * Observation real matrix.
	 */
	Matrix* D;

	/**
	 * A low-rank matrix recovered from the corrupted data observation matrix D.
	 */
	Matrix* A;

	/**
	 * Error matrix between the original observation matrix D and the low-rank
	 * recovered matrix A.
	 */
	Matrix* E;

public:

	/**
	 * Constructor for Robust PCA.
	 *
	 * @param lambda a positive weighting parameter, larger value leads to sparser
	 * 				 error matrix
	 */
	RobustPCA(double lambda);

	/**
	 * Feed an observation matrix.
	 *
	 * @param D a real matrix
	 */
	void feedData(Matrix& D);

	/**
	 * Run robust PCA.
	 */
	void run();

	/**
	 * Get the low-rank matrix recovered from the corrupted data
	 * observation matrix.
	 *
	 * @return low-rank approximation
	 */
	Matrix& GetLowRankEstimation();

	/**
	 * Get the error matrix between the original observation matrix
	 * and its low-rank recovered matrix.
	 *
	 * @return error matrix
	 */
	Matrix& GetErrorMatrix();

	/**
	 * Compute robust PCA for an observation matrix which solves the
	 * following convex optimization problem:
	 * </br>
	 * min ||A||_* + lambda ||E||_1</br>
	 * s.t. D = A + E</br>
	 * where ||.||_* denotes the nuclear norm of a matrix (i.e.,
	 * the sum of its singular values), and ||.||_1 denotes the
	 * sum of the absolute values of matrix entries.</br>
	 * </br>
	 * Inexact augmented Lagrange multipliers is used to solve the optimization
	 * problem due to its empirically fast convergence speed and proved convergence
	 * to the true optimal solution.
	 *
	 * @param D a real observation matrix
	 *
	 * @param lambda a positive weighting parameter, larger value leads to sparser
	 * 				 error matrix
	 * @return a {@code Matrix} array [A, E] where A is the recovered low-rank
	 * 		   approximation of D, and E is the error matrix between A and D
	 *
	 */
	static Matrix** run(Matrix& D, double lambda);

private:

	static double J(Matrix& Y, double lambda);

};

#endif /* ROBUSTPCA_H_ */
