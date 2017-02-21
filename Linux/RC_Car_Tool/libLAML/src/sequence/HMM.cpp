/*
 * HMM.cpp
 *
 *  Created on: Mar 6, 2014
 *      Author: Mingjie Qian
 */

#include "HMM.h"

double** HMM::initializeB() {
	double** res = new double*[N];
	for (int i = 0; i < N; i++) {
		res[i] = genDiscreteDistribution(M);;
	}
	return res;
}

double** HMM::initializeA() {
	double** res = new double*[N];
	for (int i = 0; i < N; i++) {
		res[i] = genDiscreteDistribution(N);
	}
	return res;
}

double* HMM::initializePi() {
	return genDiscreteDistribution(N);
}

/**
 * Default constructor.
 */
HMM::HMM() {
	N = 0;
	M = 0;
	pi = null;
	A = null;
	B = null;
	Os = null;
	Qs = null;
	epsilon = 1e-3;
	maxIter = 500;
}

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
HMM::HMM(int N, int M, double epsilon, int maxIter) {
	this->N = N;
	this->M = M;
	pi = new double[N];
	for (int i = 0; i < N; i++) {
		pi[i] = 0;
	}
	A = new double*[N];
	for (int i = 0; i < N; i++) {
		A[i] = new double[N];
		for (int j = 0; j < N; j++)
			A[i][j] = 0;
	}
	B = new double*[N];
	for (int i = 0; i < N; i++) {
		B[i] = new double[M];
		for (int k = 0; k < M; k++)
			B[i][k] = 0;
	}
	Os = null;
	Qs = null;

	this->epsilon = epsilon;
	this->maxIter = maxIter;
}

/**
 * Construct an HMM with default convergence precision being
 * 1e-6 and maximal number of iterations being 1000.
 *
 * @param N number of states in the model
 *
 * @param M number of distinct observation symbols per state
 *
 */
HMM::HMM(int N, int M) {
	this->N = N;
	this->M = M;
	pi = new double[N];
	for (int i = 0; i < N; i++) {
		pi[i] = 0;
	}
	A = new double*[N];
	for (int i = 0; i < N; i++) {
		A[i] = new double[N];
		for (int j = 0; j < N; j++)
			A[i][j] = 0;
	}
	B = new double*[N];
	for (int i = 0; i < N; i++) {
		B[i] = new double[M];
		for (int k = 0; k < M; k++)
			B[i][k] = 0;
	}
	Os = null;
	Qs = null;

	epsilon = 1e-6;
	maxIter = 1000;
}

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
void HMM::feedData(int** Os, int D, int* lengths) {
	this->Os = Os;
	this->D = D;
	this->lengths = lengths;
}

/**
 * Feed state sequences for training data.
 * Qs[n][t] = q_t^n, n = 0,...,D - 1, t = 0,...,T_n - 1.
 *
 * @param Qs state sequences
 *
 */
void HMM::feedLabels(int** Qs) {
	this->Qs = Qs;
}

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
double HMM::evaluate(int* O, int length) {

	// Forward Recursion with Scaling

	int T = length;
	double* c = allocateVector(T);
	double* alpha_hat_t = allocateVector(N);
	double* alpha_hat_t_plus_1 = allocateVector(N);
	double* temp_alpha = null;
	double log_likelihood = 0;

	for (int t = 0; t < T; t++) {
		if (t == 0) {
			for (int i = 0; i < N; i++) {
				alpha_hat_t[i] = pi[i] * B[i][O[0]];
			}
		} else {
			clearVector(alpha_hat_t_plus_1, N);
			for (int j = 0; j < N; j++) {
				for (int i = 0; i < N; i++) {
					alpha_hat_t_plus_1[j] += alpha_hat_t[i] * A[i][j] * B[j][O[t]];
				}
			}
			temp_alpha = alpha_hat_t;
			alpha_hat_t = alpha_hat_t_plus_1;
			alpha_hat_t_plus_1 = temp_alpha;
		}
		c[t] = 1.0 / sum(alpha_hat_t, N);
		timesAssign(alpha_hat_t, N, c[t]);
		log_likelihood -= log(c[t]);
	}

	return exp(log_likelihood);
}

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
int* HMM::predict(int* O, int length) {

	int T = length;
	int* Q = new int[T];

	double* phi_t = allocateVector(N);
	double* phi_t_plus_1 = allocateVector(N);
	double* temp_phi = null;

	int** psi = new int*[T];
	for (int t = 0; t < T; t++) {
		psi[t] = new int[N];
	}

	double* V = allocateVector(N);

	// Viterbi algorithm using logarithms

	for (int i = 0; i < N; i++) {
		phi_t[i] = log(pi[i]) + log(B[i][O[0]]);
	}

	int t = 1;
	int maxIdx = -1;
	double maxVal = 0;
	do {
		for (int j = 0; j < N; j++) {
			for (int i = 0; i < N; i++) {
				V[i] = phi_t[i] + log(A[i][j]);
			}
			maxIdx = argmax(V, N);
			maxVal = V[maxIdx];
			phi_t_plus_1[j] = maxVal + log(B[j][O[t]]);
			psi[t][j] = maxIdx;
		}
		temp_phi = phi_t;
		phi_t = phi_t_plus_1;
		phi_t_plus_1 = temp_phi;
		t = t + 1;
	} while (t < T);

	// display(psi);

	int i_t = argmax(phi_t, N);
	Q[T - 1] = i_t;
	t = T;
	do {
		i_t = psi[t - 1][i_t];
		Q[t - 1 - 1] = i_t;
		t = t - 1;
	} while (t > 1);

	return Q;

}

/**
 * Inference the basic HMM with scaling. Memory complexity
 * is O(TN) + O(N^2) + O(NM), and computation complexity is
 * O(tDTN^2), where t is the number of outer iterations.
 */
void HMM::train() {

	// int D = Os.length;
	int T_n = 0;
	double log_likelihood = 0;
	double log_likelihood_new = 0;
	double epsilon = this->epsilon;
	int maxIter = this->maxIter;

	// Initialization

	clearVector(pi, N);
	clearMatrix(A, N, N);
	clearMatrix(B, N, M);

	double* a = allocateVector(N);
	double* b = allocateVector(N);

	int* Q_n = null;
	int* O_n = null;

	if (Qs == null) {

		pi = initializePi();
		A = initializeA();
		B = initializeB();

	} else {

		for (int n = 0; n < D; n++) {
			Q_n = Qs[n];
			O_n = Os[n];
			T_n = lengths[n];
			for (int t = 0; t < T_n; t++) {
				if (t < T_n - 1) {
					A[Q_n[t]][Q_n[t + 1]] += 1;
					a[Q_n[t]] += 1;
					if (t == 0) {
						pi[Q_n[0]] += 1;
					}
				}
				B[Q_n[t]][O_n[t]] += 1;
				b[Q_n[t]] += 1;
			}
		}
		divideAssign(pi, N, D);
		for (int i = 0; i < N; i++) {
			divideAssign(A[i], N, a[i]);
			divideAssign(B[i], M, b[i]);
		}

	}

	int s = 0;
	double* pi_new = allocateVector(N);
	double** A_new = allocateMatrix(N, N);
	double** B_new = allocateMatrix(N, M);
	double* temp_pi = null;
	double** temp_A = null;
	double** temp_B = null;
	double** alpha_hat = null;
	double** beta_hat = null;
	double* c_n = null;
	double** xi = allocateMatrix(N, N);
	double* gamma = allocateVector(N);
	do {

		// Clearance
		clearVector(pi_new, N);
		clearMatrix(A_new, N, N);
		clearMatrix(B_new, N, M);
		clearVector(a, N);
		clearVector(b, N);
		/*clearMatrix(xi);
									clearVector(gamma);*/
		log_likelihood_new = 0;

		for (int n = 0; n < D; n++) {

			// Q_n = Qs[n];
			O_n = Os[n];
			T_n = lengths[n];
			c_n = allocateVector(T_n);
			alpha_hat = allocateMatrix(T_n, N);
			beta_hat = allocateMatrix(T_n, N);

			// Forward Recursion with Scaling

			for (int t = 0; t <= T_n - 1; t++) {
				if (t == 0) {
					for (int i = 0; i < N; i++) {
						alpha_hat[0][i] = pi[i] * B[i][O_n[0]];
					}
				} else {
					for (int j = 0; j < N; j++) {
						for (int i = 0; i < N; i++) {
							alpha_hat[t][j] += alpha_hat[t - 1][i] * A[i][j] * B[j][O_n[t]];
						}
					}
				}
				c_n[t] = 1.0 / sum(alpha_hat[t], N);
				timesAssign(alpha_hat[t], N, c_n[t]);
			}

			// Backward Recursion with Scaling

			for (int t = T_n + 1; t >= 2; t--) {
				if (t == T_n + 1) {
					for (int i = 0; i < N; i++) {
						beta_hat[t - 2][i] = 1;
					}
				}
				if (t <= T_n) {
					for (int i = 0; i < N; i++) {
						for (int j = 0; j < N; j++) {
							beta_hat[t - 2][i] += A[i][j] * B[j][O_n[t - 1]] * beta_hat[t - 1][j];
						}
					}
				}
				timesAssign(beta_hat[t - 2], N, c_n[t - 2]);
			}

			// Expectation Variables and Updating Model Parameters

			for (int t = 0; t <= T_n - 1; t++) {
				if (t < T_n - 1) {
					for (int i = 0; i < N; i++) {
						for (int j = 0; j < N; j++) {
							xi[i][j] = alpha_hat[t][i] * A[i][j] * B[j][O_n[t + 1]] * beta_hat[t + 1][j];
							// A_new[i][j] += xi[i][j];
						}
						plusAssign(A_new[i], xi[i], N);
						gamma[i] = sum(xi[i], N);
					}
					if (t == 0) {
						plusAssign(pi_new, gamma, N);
					}
					plusAssign(a, gamma, N);
				} else {
					assignVector(gamma, alpha_hat[t], N);
				}
				for (int j = 0; j < N; j++) {
					B_new[j][O_n[t]] += gamma[j];
				}
				plusAssign(b, gamma, N);
				log_likelihood_new += -log(c_n[t]);
			}

		}

		// Normalization (Sum to One)

		sum2one(pi_new, N);

		for (int i = 0; i < N; i++) {
			divideAssign(A_new[i], N, a[i]);
		}

		for (int j = 0; j < N; j++) {
			divideAssign(B_new[j], M, b[j]);
		}

		temp_pi = pi;
		pi = pi_new;
		pi_new = temp_pi;

		temp_A = A;
		A = A_new;
		A_new = temp_A;

		temp_B = B;
		B = B_new;
		B_new = temp_B;
		// display(B);

		s = s + 1;

		if (s > 1) {
			if (fabs((log_likelihood_new - log_likelihood) / log_likelihood) < epsilon) {
				fprintf("log[P(O|Theta)] does not increase.\n\n");
				break;
			}
		}

		log_likelihood = log_likelihood_new;
		fprintf("Iter: %d, log[P(O|Theta)]: %f\n", s, log_likelihood);

	} while (s < maxIter);

}

/**
 * Generate a discrete distribution with sample size of n.
 *
 * @param n sample size
 *
 * @return a double array with sum 1
 */
double* HMM::genDiscreteDistribution(int n) {
	std::default_random_engine generator(time(NULL));
	std::normal_distribution<double> normal(0.0, 1.0);
	double* res = allocateVector(n);
	do {
		for (int i = 0; i < n; i++) {
			res[i] = normal(generator);
		}
	} while (sum(res, n) == 0);
	sum2one(res, n);
	return res;
}

/**
 * Show a state sequence.
 *
 * @param Q a state sequence represented by a 1D
 *          {@code int} array
 *
 * @param length length of Q
 */
void HMM::showStateSequence(int* Q, int length) {
	for (int t = 0; t < length; t++) {
		fprintf("%d ", Q[t]);
	}
	println();
}

/**
 * Show an observation sequence.
 *
 * @param O an observation sequence represented by a 1D
 *          {@code int} array
 *
 * @param length length of O
 */
void HMM::showObservationSequence(int* O, int length) {
	for (int t = 0; t < length; t++) {
		fprintf("%d ", O[t]);
	}
	println();
}

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
int*** HMM::generateDataSequences(int D, int T_min, int T_max, double* pi, double** A, double** B, int N, int M, int* lengths) {

	int*** res = new int**[2];

	int** Os = new int*[D];
	int** Qs = new int*[D];

	double* distribution = null;
	double sum = 0;

	std::default_random_engine generator(time(NULL));
	std::normal_distribution<double> normal(0.0, 1.0);
	std::uniform_int_distribution<int> uniform_int(0, T_max - T_min);
	double rndRealScalor = 0;

	for (int n = 0; n < D; n++) {

		int T_n = uniform_int(generator) + T_min;
		lengths[n] = T_n;
		int* O_n = new int[T_n];
		int* Q_n = new int[T_n];

		for (int t = 0; t < T_n; t++) {

			rndRealScalor = normal(generator);

			if (t == 0) { // Initial state
				distribution = pi;
			} else { // Following states
				distribution = A[Q_n[t - 1]];
			}

			// Generate a state sequence
			sum = 0;
			for (int i = 0; i < N; i++) {
				sum += distribution[i];
				if (rndRealScalor <= sum) {
					Q_n[t] = i;
					break;
				}
			}

			rndRealScalor = normal(generator);

			// Generate an observation sequence
			distribution = B[Q_n[t]];
			sum = 0;
			for (int k = 0; k < M; k++) {
				sum += distribution[k];
				if (rndRealScalor <= sum) {
					O_n[t] = k;
					break;
				}
			}

		}
		Os[n] = O_n;
		Qs[n] = Q_n;
	}

	res[0] = Os;
	res[1] = Qs;

	return res;

}
