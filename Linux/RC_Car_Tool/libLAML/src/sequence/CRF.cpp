/*
 * CRF.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Mingjie Qian
 */

#include "CRF.h"

/**
 * Constructor for a CRF instance.
 */
CRF::CRF() {
	sigma = 1;
	epsilon = 1e-4;
	startIdx = 0;
	maxIter = 50;
}

/**
 * Constructor for a CRF instance.
 *
 * @param sigma regularization parameter
 */
CRF::CRF(double sigma) {
	this->sigma = sigma;
	epsilon = 1e-4;;
	startIdx = 0;
	maxIter = 50;
}

/**
 * Constructor for a CRF instance.
 *
 * @param sigma regularization parameter
 *
 * @param epsilon convergence precision
 */
CRF::CRF(double sigma, double epsilon) {
	this->sigma = sigma;
	this->epsilon = epsilon;
	startIdx = 0;
	maxIter = 50;
}

/**
 * Constructor for a CRF instance.
 *
 * @param sigma regularization parameter
 *
 * @param epsilon convergence precision
 *
 * @param maxIter maximal number of iterations
 */
CRF::CRF(double sigma, double epsilon, int maxIter) {
	this->sigma = sigma;
	this->epsilon = epsilon;
	startIdx = 0;
	this->maxIter = maxIter;
}

/**
 * Feed data sequences for training.
 *
 * @param Fs a 3D {@code Matrix} array, where F[k][i][j] is the sparse
 * 			 feature matrix for the j-th feature of the k-th observation sequence
 * 		     at position i, i.e., f_{j}^{{\bf x}_k, i}
 * @param D number of training data sequences
 *
 * @param d number of feature functions
 *
 * @param lengths the integer array holding the length of each training sequence
 */
void CRF::feedData(Matrix**** Fs, int D, int d, int* lengths) {
	this->Fs = Fs;
	this->D = D;
	this->d = d;
	this->lengths = lengths;
	numStates = Fs[0][0][0]->getRowDimension();
}

/**
 * Feed labels for training data sequences.
 *
 * @param Ys a 2D integer array, where Ys[k][i] is the label index for the label
 *           of the k-th training data sequence at position i in the label space
 *
 */
void CRF::feedLabels(int** Ys) {
	this->Ys = Ys;
}

/**
 * Estimate parameters for the basic CRF by a maximum conditional
 * log-likelihood estimation principle.
 */
void CRF::train() {

	double fval = 0;

	int maxSequenceLength = computeMaxSequenceLength();

	/*
	 * Transition matrices
	 */
	Matrix** Ms = new Matrix*[maxSequenceLength];
	for (int i = 0; i < maxSequenceLength; i++) {
		Ms[i] = new DenseMatrix(numStates, numStates);
	}
	Vector& F = computeGlobalFeatureVector();

	/*
	 * Initialize the parameters
	 */
	// W = times(10, ones(d, 1));
	W = new DenseVector(d, 10);

	Vector& Grad = *new DenseVector(d);

	fval = computeObjectiveFunctionValue(F, Ms, true, Grad, *W);

	bool* flags = new bool[2];
	// double fval_pre = 0;
	int k = 0;
	while (true) {
		LBFGSForVector::run(Grad, fval, epsilon, *W, flags);
		if (flags[0])
			break;
		// display(W);
		/*if (sumAll(isnan(W)) > 0) {
					int a = 1;
					a = a + 1;
				}*/

		/*
		 *  Compute the objective function value, if flags[1] is true
		 *  gradient will also be computed.
		 */
		fval = computeObjectiveFunctionValue(F, Ms, flags[1], Grad, *W);
		if (flags[1]) {
			k = k + 1;
			// Compute the gradient
			if (k > maxIter)
				break;

			/*if ( Math.abs(fval_pre - fval) < eps)
						break;
					fval_pre = fval;*/
		}

	}
}

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
int* CRF::predict(Matrix*** Fs, int length) {

	Matrix** Ms = computeTransitionMatrix(Fs, length);

	/*
	 * Alternative backward recursion with scaling for the Viterbi
	 * algorithm
	 */
	int n_x = length;
	double* b = allocateVector(n_x);
	// Matrix Beta_tilta = new BlockRealMatrix(numStates, n_x);
	Vector** Beta_tilta = new Vector*[n_x];

	for (int i = n_x - 1; i >= 0; i--) {
		if ( i == n_x - 1) {
			// Beta_tilta.setColumnMatrix(i, ones(numStates, 1));
			Beta_tilta[i] = new DenseVector(numStates, 1);
		} else {
			// Beta_tilta.setColumnMatrix(i, mtimes(Ms[i + 1], Beta_tilta.getColumnMatrix(i + 1)));
			Beta_tilta[i] = &Ms[i + 1]->operate(*Beta_tilta[i + 1]);
		}
		b[i] = 1.0 / sum(*Beta_tilta[i]);
		// Beta_tilta.setColumnMatrix(i, times(b[i], Beta_tilta.getColumnMatrix(i)));
		timesAssign(*Beta_tilta[i], b[i]);
	}

	/*fprintf("Beta:\n");
				display(Beta_tilta);*/

	/*
	 * Gammas[i](y_{i-1}, y_[i]) is P(y_i|y_{i-1}, Lambda), thus each row of
	 * Gammas[i] should be sum to one.
	 */

	double** Gamma_i = allocate2DArray(numStates, numStates, 0);
	double** Phi =  allocate2DArray(n_x, numStates, 0);
	double** Psi =  allocate2DArray(n_x, numStates, 0);
	double** M_i = null;
	double* M_i_Row = null;
	double* Gamma_i_Row = null;
	double* Beta_tilta_i = null;
	double* Phi_i = null;
	double* Phi_im1 = null;
	double** maxResult = null;
	for (int i = 0; i < n_x; i++) {
		M_i = ((DenseMatrix*) Ms[i])->getData();
		Beta_tilta_i = ((DenseVector*) Beta_tilta[i])->getPr();
		for (int y_im1 = 0; y_im1 < numStates; y_im1++) {
			M_i_Row = M_i[y_im1];
			Gamma_i_Row = Gamma_i[y_im1];
			assign(Gamma_i_Row, M_i_Row, numStates);
			timesAssign(Gamma_i_Row, Beta_tilta_i, numStates);
			sum2one(Gamma_i_Row, numStates);
		}
		Phi_i = Phi[i];
		if (i == 0) { // Initialization
			log(Phi_i, Gamma_i[startIdx], numStates);
		} else {
			Phi_im1 = Phi[i - 1];
			for (int y_im1 = 0; y_im1 < numStates; y_im1++) {
				Gamma_i_Row = Gamma_i[y_im1];
				logAssign(Gamma_i_Row, numStates);
				plusAssign(Gamma_i_Row, Phi_im1[y_im1], numStates);
			}
			maxResult = max(Gamma_i, numStates, numStates, 1);
			Phi[i] = maxResult[0];
			Psi[i] = maxResult[1];
		}

	}

	/*
	 *  Predict the single best label sequence.
	 */
	// double[] phi_n_x = Phi.getRow(n_x - 1);
	double* phi_n_x = Phi[n_x - 1];
	int* YPred = allocateIntegerVector(n_x);
	for (int i = n_x - 1; i >= 0; i--) {
		if (i == n_x - 1) {
			YPred[i] = argmax(phi_n_x, numStates);
		} else {
			// YPred[i] = (int)Psi.getEntry(i + 1, YPred[i + 1]);
			YPred[i] = (int) Psi[i + 1][YPred[i + 1]];
		}
	}

	/*display(Phi);
				display(Psi);*/

	/*
	 *  Predict the optimal conditional probability: P*(y|x)
	 */
	double p = exp(phi_n_x[YPred[n_x - 1]]);
	fprintf("P*(YPred|x) = %g\n", p);

	return YPred;

}

int CRF::computeMaxSequenceLength() {
	int maxSeqLen = 0;
	int n_x = 0;
	for (int k = 0; k < D; k++) {
		n_x = lengths[k];
		if (maxSeqLen < n_x) {
			maxSeqLen = n_x;
		}

	}
	return maxSeqLen;
}

Vector& CRF::computeGlobalFeatureVector() {

	/*
	 * Compute global feature vector for all training data sequences,
	 * i.e., sum_k F(y_k, x_k)
	 */

	/*
	 * d x 1 feature vector for D training data sequences
	 */
	Vector& F = *new DenseVector(d, 1);

	int n_x = 0;
	double f = 0;
	int* Y = null;
	Matrix*** Fs_k = null;
	for (int j = 0; j < d; j++) {
		f = 0;
		for (int k = 0; k < D; k++) {
			Y = Ys[k];
			Fs_k = Fs[k];
			n_x = lengths[k];
			for (int i = 0; i < n_x; i++) {
				if (i == 0)
					f += Fs_k[i][j]->getEntry(0, Y[i]);
				else
					f += Fs_k[i][j]->getEntry(Y[i - 1], Y[i]);
			}
		}
		// F.setEntry(j, 0, f);
		F.set(j, f);
	}
	return F;
}

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
double CRF::computeObjectiveFunctionValue(Vector& F, Matrix** Ms, bool calcGrad, Vector& Grad, Vector& W) {

	double fval = 0;

	// int[] Y = null;

	/*
	 * d x 1 array conditional expectation of feature functions
	 * for D training data sequences, i.e.,
	 * EF = sum_k E_{P_{\bf \lambda}(Y|x_k)}[F(Y, x_k)]
	 */
	Vector* EF = null;
	double* EFArr = allocateVector(d);

	/*
	 * Compute global feature vector for all training data sequences,
	 * i.e., sum_k F(y_k, x_k)
	 */
	int n_x = 0;
	Matrix*** Fs_k = null;

	Matrix* f_j_x_i = null;

	for (int k = 0; k < D; k++) {
		Fs_k = Fs[k];
		n_x = lengths[k];
		// Ms = new Matrix[n_x];

		/*
		 * Compute transition matrix set
		 */
		for (int i = 0; i < n_x; i++) {
			// Ms[i] = new DenseMatrix(numStates, numStates, 0);
			Ms[i]->clear();
			for (int j = 0; j < d; j++) {
				f_j_x_i = Fs_k[i][j];
				if (j == 0)
					// Ms[i] = times(W.get(j), f_j_x_i);
					times(*Ms[i], W.get(j), *f_j_x_i);
				else
					// Ms[i] = plus(Ms[i], times(W.get(j), f_j_x_i));
					plusAssign(*Ms[i], W.get(j), *f_j_x_i);
			}
			expAssign(*Ms[i]);
			/*for (int s = 0; s < numStates; s++) {
								if (sum(Ms[i].getRow(s)) == 0) {
									Ms[i].setRow(s, allocateVector(numStates, 1e-10));
								}
							}*/
		}

		/*
		 * Forward recursion with scaling
		 */
		/*Matrix Alpha_hat = new BlockRealMatrix(numStates, n_x);
						Matrix Alpha_hat_0 = new BlockRealMatrix(numStates, 1);
						RealVector e_start = new ArrayRealVector(numStates);
						e_start.setEntry(startIdx, 1);*/

		Vector** Alpha_hat = new Vector*[n_x];
		/*for (int i = 0; i < n_x; i++) {
							Alpha_hat[i] = new DenseVector(numStates);
						}*/
		// Vector* Alpha_hat_0 = null;
		Vector& e_start = *new SparseVector(numStates);
		e_start.set(startIdx, 1);


		double* c = allocateVector(n_x);

		// Alpha_hat_0.setColumnVector(0, e_start);
		Vector& Alpha_hat_0 = e_start;

		for (int i = 0; i < n_x; i++) {
			if (i == 0) {
				// Alpha_hat.setColumnMatrix(i, Ms[i].transpose().multiply(Alpha_hat_0));
				Alpha_hat[i] = &Alpha_hat_0.operate(*Ms[i]);
			} else {
				// Alpha_hat.setColumnMatrix(i, Ms[i].transpose().multiply(Alpha_hat.getColumnMatrix(i - 1)));
				Alpha_hat[i] = &Alpha_hat[i - 1]->operate(*Ms[i]);
			}
			// c[i] = 1.0 / sum(Alpha_hat.getColumnVector(i));
			c[i] = 1.0 / sum(*Alpha_hat[i]);
			/*if (Double.isInfinite(c[i])) {
								int a = 1;
								a = a + 1;
							}*/
			// Alpha_hat.setColumnMatrix(i, times(c[i], Alpha_hat.getColumnMatrix(i)));
			timesAssign(*Alpha_hat[i], c[i]);
		}

		/*
		 * Backward recursion with scaling
		 */
		// Matrix Beta_hat = new BlockRealMatrix(numStates, n_x);
		Vector** Beta_hat = new Vector*[n_x];
		for (int i = n_x - 1; i >= 0; i--) {
			if ( i == n_x - 1) {
				// Beta_hat.setColumnMatrix(i, ones(numStates, 1));
				Beta_hat[i] = new DenseVector(numStates, 1);
			} else {
				// Beta_hat.setColumnMatrix(i, mtimes(Ms[i + 1], Beta_hat.getColumnMatrix(i + 1)));
				Beta_hat[i] = &Ms[i + 1]->operate(*Beta_hat[i + 1]);
			}
			// Beta_hat.setColumnMatrix(i, times(c[i], Beta_hat.getColumnMatrix(i)));
			timesAssign(*Beta_hat[i], c[i]);
		}

		/*
		 * Accumulate the negative conditional log-likelihood on the
		 * D training data sequences
		 */
		for (int i = 0; i < n_x; i++) {
			fval -= log(c[i]);
		}

		/*if (Double.isNaN(fval)) {
							int a = 1;
							a = a + 1;
						}*/

		if (!calcGrad)
			continue;
		/*
		 * Compute E_{P_{\bf \lambda}(Y|x_k)}[F(Y, x_k)]
		 */
		for (int j = 0; j < d; j++) {
			/*
			 * Compute E_{P_{\bf \lambda}(Y|x_k)}[F_{j}(Y, x_k)]
			 */
			for (int i = 0; i < n_x; i++) {
				if (i == 0) {
					// EFArr[j] += Alpha_hat_0.transpose().multiply(times(Ms[i], f_j_x_i)).multiply(Beta_hat.getColumnMatrix(i)).getEntry(0, 0);
					EFArr[j] += innerProduct(Alpha_hat_0, Ms[i]->times(*f_j_x_i).operate(*Beta_hat[i]));
				} else {
					// EFArr[j] += Alpha_hat.getColumnMatrix(i - 1).transpose().multiply(times(Ms[i], f_j_x_i)).multiply(Beta_hat.getColumnMatrix(i)).getEntry(0, 0);
					EFArr[j] += innerProduct(*Alpha_hat[i - 1], Ms[i]->times(*f_j_x_i).operate(*Beta_hat[i]));
				}
			}
		}
	}

	/*
	 * Calculate the eventual negative conditional log-likelihood
	 */
	fval -= innerProduct(W, F);
	fval += sigma * innerProduct(W, W);
	fval /= D;

	if (!calcGrad) {
		return fval;
	}

	/*
	 * Calculate the gradient of negative conditional log-likelihood
	 * w.r.t. W. on the D training data sequences
	 */
	// EF.setColumn(0, EFArr);
	EF = new DenseVector(EFArr, d);
	times(Grad, 1.0 / D, plus(minus(*EF, F), W.times(2 * sigma)));

	return fval;

}

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
Matrix** CRF::computeTransitionMatrix(Matrix*** Fs, int length) {

	int n_x = length;
	Matrix* f_j_x_i = null;
	Matrix** Ms = new Matrix*[n_x];
	for (int i = 0; i < n_x; i++) {
		Ms[i] = new DenseMatrix(numStates, numStates, 0);
		for (int j = 0; j < d; j++) {
			f_j_x_i = Fs[i][j];
			if (j == 0)
				// Ms[i] = times(W.get(j), f_j_x_i);
				times(*Ms[i], W->get(j), *f_j_x_i);
			else
				// Ms[i] = plus(Ms[i], times(W.get(j), f_j_x_i));
				plusAssign(*Ms[i], W->get(j), *f_j_x_i);
		}
		// Ms[i] = exp(Ms[i]);
		expAssign(*Ms[i]);
		/*for (int s = 0; s < numStates; s++) {
						if (sum(Ms[i].getRow(s)) == 0) {
							Ms[i].setRow(s, allocateVector(numStates, 1e-10));
						}
					}*/
	}
	return Ms;

}
