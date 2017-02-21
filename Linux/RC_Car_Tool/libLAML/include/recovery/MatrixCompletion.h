/*
 * MatrixCompletion.h
 *
 *  Created on: Mar 4, 2014
 *      Author: Mingjie Qian
 */

#ifndef MATRIXCOMPLETION_H_
#define MATRIXCOMPLETION_H_

#include "Matlab.h"
#include "ProximalMapping.h"
#include "Printer.h"
#include "InPlaceOperator.h"
#include "SVD.h"

/***
 * A C++ implementation of matrix completion which solves the
 * following convex optimization problem:
 * </br>
 * min ||A||_*</br>
 * s.t. D = A + E</br>
 *      E(Omega) = 0</br>
 * where ||.||_* denotes the nuclear norm of a matrix (i.e.,
 * the sum of its singular values).</br>
 * </br>
 * Inexact augmented Lagrange multiplers is used to solve the optimization
 * problem due to its empirically fast convergence speed and proved convergence
 * to the true optimal solution.
 *
 * <b>Input:</b></br>
 *    D: an observation matrix with columns as data vectors</br>
 *    Omega: a sparse or dense logical matrix indicating the indices of samples</br>
 *
 * <b>Output:</b></br>
 *    A: a low-rank matrix completed from the data matrix D</br>
 *    E: error matrix between D and A</br>
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 4th, 2014
 */
class MatrixCompletion {

private:

	/**
	 * Observation real matrix.
	 */
	Matrix* D;

	/**
	 * a sparse or dense logical matrix indicating the indices of samples
	 */
	Matrix* Omega;

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
	 * Constructor.
	 */
	MatrixCompletion();

	/**
	 * Feed an observation matrix.
	 *
	 * @param D a real matrix
	 */
	void feedData(Matrix& D);

	/**
	 * Feed indices of samples.
	 *
	 * @param Omega a sparse or dense logical matrix indicating the indices of samples
	 *
	 */
	void feedIndices(Matrix& Omega);

	/**
	 * Feed indices of samples.
	 *
	 * @param indices an {@code int} array for the indices of samples
	 *
	 * @param len length of the index array
	 */
	void feedIndices(int* indices, int len);

	/**
	 * Run matrix completion.
	 */
	void run();

	/**
	 * Get the low-rank completed matrix.
	 *
	 * @return the low-rank completed matrix
	 */
	Matrix& GetLowRankEstimation();

	/**
	 * Get the error matrix between the original observation matrix and
	 * its low-rank completed matrix.
	 *
	 * @return error matrix
	 */
	Matrix& GetErrorMatrix();

	/**
	 * Do matrix completion which solves the following convex
	 * optimization problem:
	 * </br>
	 * min ||A||_*</br>
	 * s.t. D = A + E</br>
	 *      E(Omega) = 0</br>
	 * where ||.||_* denotes the nuclear norm of a matrix (i.e.,
	 * the sum of its singular values).</br>
	 * </br>
	 * Inexact augmented Lagrange multipliers is used to solve the optimization
	 * problem due to its empirically fast convergence speed and proved convergence
	 * to the true optimal solution.
	 *
	 * @param D a real observation matrix
	 *
	 * @param Omega a sparse or dense logical matrix indicating the indices of samples
	 *
	 * @return a {@code Matrix} array [A, E] where A is the low-rank
	 * 		   completion from D, and E is the error matrix between A and D
	 *
	 */
	static Matrix** run(Matrix& D, Matrix& Omega);

	/**
	 * Do matrix completion which solves the following convex
	 * optimization problem:
	 * </br>
	 * min ||A||_*</br>
	 * s.t. D = A + E</br>
	 *      E(Omega) = 0</br>
	 * where ||.||_* denotes the nuclear norm of a matrix (i.e.,
	 * the sum of its singular values).</br>
	 * </br>
	 * Inexact augmented Lagrange multipliers is used to solve the optimization
	 * problem due to its empirically fast convergence speed and proved convergence
	 * to the true optimal solution.
	 *
	 * @param D a real observation matrix
	 *
	 * @param indices an {@code int} array for the indices of samples
	 *
	 * @param len length of the index array
	 *
	 * @return a {@code Matrix} array [A, E] where A is the low-rank
	 * 		   completion from D, and E is the error matrix between A and D
	 *
	 */
	static Matrix** run(Matrix& D, int* indices, int len);

};

#endif /* MATRIXCOMPLETION_H_ */
