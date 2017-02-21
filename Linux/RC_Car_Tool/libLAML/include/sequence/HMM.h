/*
 * HMM.h
 *
 *  Created on: Mar 6, 2014
 *      Author: Mingjie Qian
 */

#ifndef HMM_H_
#define HMM_H_

#include "Printer.h"
#include "ArrayOperator.h"
#include "Utility.h"
#include <random>
#include <ctime>
#include <cmath>

/***
 * A C++ implementation of basic Hidden Markov Model.
 * Observation symbols and state symbols are discrete
 * integers starting from 0. It is the responsibility
 * of users to maintain the mapping from IDs to
 * observation symbols and the mapping from IDs to
 * state symbols.
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 6th, 2013
 */
class HMM {

private:

	/**
	 * Number of data sequences for training.
	 */
	int D;

	/**
	 * An integer array holding the length of each training sequence.
	 */
	int* lengths;

	/**
	 * Observation sequences for training.
	 * Os[n][t] = O_t^n, n = 0,...,D - 1, t = 0,...,T_n - 1.
	 */
	int** Os;

	/**
	 * Hidden state sequences for training data.
	 * Qs[n][t] = q_t^n, n = 0,...,D - 1, t = 0,...,T_n - 1.
	 */
	int** Qs;

	// **************** Model Parameters: **************** //

	/**
	 * Number of states in the model.
	 */
	int N;

	/**
	 * Number of distinct observation symbols per state.
	 */
	int M;

	/**
	 * Initial state distribution. pi[i] = P(q_1 = S_i).
	 */
	double* pi;

	/**
	 * State transition probability matrix.
	 * A[i][j] = P(q_{t+1} = S_j|q_t = S_i).
	 */
	double** A;

	/**
	 * Observation probability matrix. B[j][k] = P(v_k|S_j).
	 */
	double** B;

	// *************************************************** //

	/**
	 * Convergence precision.
	 */
	double epsilon;

	/**
	 * Maximal number of iterations.
	 */
	int maxIter;

private:

	double** initializeB();

	double** initializeA();

	double* initializePi();

public:

	/**
	 * Default constructor.
	 */
	HMM();

	/**
	 * Construct an HMM.
	 *
	 * @param N number of states in the model
	 *
	 * @param M number of distinct observation symbols per state
	 *
	 * @param epsilon convergence precision
	 *
	 * @param maxIter maximal number of iterations
	 *
	 */
	HMM(int N, int M, double epsilon, int maxIter);

	/**
	 * Construct an HMM with default convergence precision being
	 * 1e-6 and maximal number of iterations being 1000.
	 *
	 * @param N number of states in the model
	 *
	 * @param M number of distinct observation symbols per state
	 *
	 */
	HMM(int N, int M);

	/**
	 * Feed observation sequences for training.
	 * Os[n][t] = O_t^n, n = 0,...,D - 1, t = 0,...,T_n - 1.
	 *
	 * @param Os observation sequences
	 *
	 * @param D number of training data sequences
	 *
	 * @param lengths the integer array holding the length of each training sequence
	 */
	void feedData(int** Os, int D, int* lengths);

	/**
	 * Feed state sequences for training data.
	 * Qs[n][t] = q_t^n, n = 0,...,D - 1, t = 0,...,T_n - 1.
	 *
	 * @param Qs state sequences
	 *
	 */
	void feedLabels(int** Qs);

	/**
	 * Compute P(O|Theta), the probability of the observation
	 * sequence given the model, by forward recursion with
	 * scaling.
	 *
	 * @param O an observation sequence
	 *
	 * @param length length of O
	 *
	 * @return P(O|Theta)
	 */
	double evaluate(int* O, int length);

	/**
	 * Predict the best single state path for a given observation sequence
	 * using Viterbi algorithm with logarithms.
	 *
	 * @param O an observation sequence
	 *
	 * @param length length of O
	 *
	 * @return the most probable state path
	 *
	 */
	int* predict(int* O, int length);

	/**
	 * Inference the basic HMM with scaling. Memory complexity
	 * is O(TN) + O(N^2) + O(NM), and computation complexity is
	 * O(tDTN^2), where t is the number of outer iterations.
	 */
	void train();

	/**
	 * Generate a discrete distribution with sample size of n.
	 *
	 * @param n sample size
	 *
	 * @return a double array with sum 1
	 */
	double* genDiscreteDistribution(int n);

	void setQs(int** Qs) {
		this->Qs = Qs;
	}

	void setOs(int** Os) {
		this->Os = Os;
	}

	void setPi(double* pi) {
		this->pi = pi;
	}

	void setA(double** A) {
		this->A = A;
	}

	void setB(double** B) {
		this->B = B;
	}

	double* getPi() {
		return this->pi;
	}

	double** getA() {
		return this->A;
	}

	double** getB() {
		return this->B;
	}

	/**
	 * Show a state sequence.
	 *
	 * @param Q a state sequence represented by a 1D
	 *          {@code int} array
	 *
	 * @param length length of Q
	 */
	void showStateSequence(int* Q, int length);

	/**
	 * Show an observation sequence.
	 *
	 * @param O an observation sequence represented by a 1D
	 *          {@code int} array
	 *
	 * @param length length of O
	 */
	void showObservationSequence(int* O, int length);

	/**
	 * Generate observation sequences with hidden state sequences
	 * given model parameters and number of data sequences.
	 *
	 * @param D number of data sequences to be generated
	 *
	 * @param T_min minimal sequence length
	 *
	 * @param T_max maximal sequence length
	 *
	 * @param pi initial state distribution
	 *
	 * @param A state transition probability matrix
	 *
	 * @param B observation probability matrix
	 *
	 * @return a 3D integer array composed of two 2D integer array with
	 *         the first one being the observation sequences and second
	 *         one being the hidden state sequences
	 *
	 */
	static int*** generateDataSequences(int D, int T_min, int T_max, double* pi, double** A, double** B, int N, int M, int* lengths);

};

#endif /* HMM_H_ */
