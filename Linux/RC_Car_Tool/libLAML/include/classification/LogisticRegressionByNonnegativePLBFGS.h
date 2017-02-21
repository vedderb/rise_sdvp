/*
 * LogisticRegressionByNonnegativePLBFGS.h
 *
 *  Created on: Feb 23, 2014
 *      Author: Mingjie Qian
 */

#ifndef LOGISTICREGRESSIONBYNONNEGATIVEPLBFGS_H_
#define LOGISTICREGRESSIONBYNONNEGATIVEPLBFGS_H_

#include "Classifier.h"
#include "Matrix.h"

/**
 * Multi-class logistic regression by using projected limited-memory BFGS
 * method. Projection matrix is constrained to be non-negative.
 * <p/>
 * We aim to minimize the cross-entropy error function defined by
 * <p/>
 * E(W) = -ln{p(T|w1, w2,..., wK)} / N = -sum_n{sum_k{t_{nk}ln(v_nk)}} / N,
 * <p/>where \nabla E(W) = X * (V - T) / N and v_nk = P(C_k|x_n).
 *
 * @version 1.0 Feb. 23rd, 2014
 * @author Mingjie Qian
 */
class LogisticRegressionByNonnegativePLBFGS : public Classifier {

private :

	/**
	 * G = [A ones(nSample, 1)]' * B.
	 *
	 * @param A
	 * @param B
	 */
	void computeGradient(Matrix& res, Matrix& A, Matrix& B);

	void computeActivation(Matrix& A, Matrix& X, Matrix& W);

public :

	LogisticRegressionByNonnegativePLBFGS();

	LogisticRegressionByNonnegativePLBFGS(double epsilon);

	void loadModel(std::string filePath);

	void saveModel(std::string filePath);

	void train();

	Matrix& predictLabelScoreMatrix(Matrix& Xt);

};

#endif /* LOGISTICREGRESSIONBYNONNEGATIVEPLBFGS_H_ */
