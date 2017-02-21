/*
 * CRF.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Mingjie Qian
 */

#ifndef CRF_H_
#define CRF_H_

#include "Matrix.h"
#include "Vector.h"
#include "Matlab.h"
#include "ArrayOperator.h"
#include "InPlaceOperator.h"
#include "Utility.h"
#include "LBFGSForVector.h"
#include "Printer.h"
#include <cmath>

/***
 * A C++ implementation for the basic Conditional Random Field (CRF).
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 5th, 2014
 */
class CRF {

private:

	/**
	 * A 3D {@code Matrix} array, where F[k][i][j] is the sparse
	 * feature matrix for the j-th feature of the k-th observation sequence
	 * at position i, i.e., f_{j}^{{\bf x}_k, i}.
	 */
	Matrix**** Fs;

	/**
	 * Number of data sequences for training.
	 */
	int D;

	/**
	 * An integer array holding the length of each training sequence.
	 */
	int* lengths;

	/**
	 * Number of parameters or number of features
	 */
	int d;

	/**
	 * Number of states in the state space.
	 */
	int numStates;

	/**
	 * Index for the start state in the state space.
	 */
	int startIdx;

	/**
	 * A 2D integer array, where Ys[k][i] is the label index for the label
	 * of the k-th training data sequence at position i in the label space.
	 */
	int** Ys;

	/**
	 * d x 1 parameter vector.
	 */
	Vector* W;

	/**
	 * Convergence precision.
	 */
	double epsilon;

	/**
	 * Regularization parameter
	 */
	double sigma;

	/**
	 * Maximal number of iterations.
	 */
	int maxIter;

public:

	/**
	 * Constructor for a CRF instance.
	 */
	CRF();

	/**
	 * Constructor for a CRF instance.
	 *
	 * @param sigma regularization parameter
	 */
	CRF(double sigma);

	/**
	 * Constructor for a CRF instance.
	 *
	 * @param sigma regularization parameter
	 *
	 * @param epsilon convergence precision
	 */
	CRF(double sigma, double epsilon);

	/**
	 * Constructor for a CRF instance.
	 *
	 * @param sigma regularization parameter
	 *
	 * @param epsilon convergence precision
	 *
	 * @param maxIter maximal number of iterations
	 */
	CRF(double sigma, double epsilon, int maxIter);

	/**
	 * Feed data sequences for training.
	 *
	 * @param Fs a 3D {@code Matrix} array, where F[k][i][j] is the sparse
	 * 			 feature matrix for the j-th feature of the k-th observation sequence
	 * 		     at position i, i.e., f_{j}^{{\bf x}_k, i}
	 *
	 * @param D number of training data sequences
	 *
	 * @param d number of feature functions
	 *
	 * @param lengths the integer array holding the length of each training sequence
	 */
	void feedData(Matrix**** Fs, int D, int d, int* lengths);

	/**
	 * Feed labels for training data sequences.
	 *
	 * @param Ys a 2D integer array, where Ys[k][i] is the label index for the label
	 *           of the k-th training data sequence at position i in the label space
	 *
	 */
	void feedLabels(int** Ys);

	/**
	 * Estimate parameters for the basic CRF by a maximum conditional
	 * log-likelihood estimation principle.
	 */
	void train();

	/**
	 * Predict the single best label sequence given the features for an
	 * observation sequence by Viterbi algorithm.
	 *
	 * @param Fs a 2D {@code Matrix} array, where F[i][j] is the sparse
	 * 			 feature matrix for the j-th feature of the observation sequence
	 *	 	 	 at position i, i.e., f_{j}^{{\bf x}, i}
	 *
	 * @return the single best label sequence for an observation sequence
	 *
	 */
	int* predict(Matrix*** Fs, int length);

private:

	int computeMaxSequenceLength();

	Vector& computeGlobalFeatureVector();

	/**
	 * Compute the objective function value (the mean log-likelihood on training
	 * data for CRFs). Gradient is also calculated if required.
	 *
	 * @param F d x 1 feature vector for D training data sequences
	 *
	 * @param Ms transition matrices
	 *
	 * @param calcGrad if gradient required
	 *
	 * @param Grad gradient to be assigned in place if required
	 *
	 * @param W model parameters
	 *
	 * @return objective function value
	 *
	 */
	double computeObjectiveFunctionValue(Vector& F, Matrix** Ms, bool calcGrad, Vector& Grad, Vector& W);

	/**
	 * Compute transition matrix set, i.e., {M^{\bf x}_i}, i = 1, 2, ..., n_x
	 *
	 * @param Fs A 2D {@code Matrix} array, where F[i][j] is the sparse
	 * 			 feature matrix for the j-th feature of the observation sequence
	 *	 	 	 at position i, i.e., f_{j}^{{\bf x}, i}
	 *
	 * @return a transition matrix sequences of length n_x
	 *
	 */
	Matrix** computeTransitionMatrix(Matrix*** Fs, int length);

};

#endif /* CRF_H_ */
