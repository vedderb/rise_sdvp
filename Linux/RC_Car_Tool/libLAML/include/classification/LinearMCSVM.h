/*
 * LinearMCSVM.h
 *
 *  Created on: Feb 21, 2014
 *      Author: Mingjie Qian
 */

#ifndef LINEARMCSVM_H_
#define LINEARMCSVM_H_

#include "Classifier.h"
#include "Matrix.h"
#include <iostream>

/***
 * A C++ implementation of fast linear multi-class SVM by
 * Crammer-Singer formulation. It uses sparse representation of
 * the feature vectors, and updates the weight vectors and dual
 * variables by dual coordinate descent. For heart_scale data,
 * for C = 1.0 and eps = 1e-2, the average running time is 0.08
 * seconds using an Intel(R) Core(TM) i7 CPU M620 @ 2.67GHz with
 * 4.00GB memory and 64-bit Windows 7 operating system.
 *
 * <p>
 * The memory complexity is O(l*d_s) + O(d*K) and the
 * computation complexity is O(l*d_s), where d_s is the average
 * number of non-zero features, d is the feature size, l is
 * the training sample size, and K is the number of classes.
 * </p>
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 21st, 2014
 */
class LinearMCSVM : public Classifier {

private:

	/**
	 * Parameter for loss term.
	 */
	double C;

	/**
	 * Convergence tolerance.
	 */
	double eps;

	double* computeQ(Matrix& X, double* pr_CSR);

	void updateW(double* Wp, double* Wq, double delta, Matrix& X, int i, double* pr_CSR);

	double computeGradient(Matrix& X, int i, double* pr_CSR, double* Wp, double* Wq);

public:

	LinearMCSVM();

	LinearMCSVM(double C, double eps);

	friend std::ostream& operator<<(std::ostream&, const LinearMCSVM&);

	friend std::istream& operator>>(std::istream&, LinearMCSVM&);

	void loadModel(std::string filePath);

	void saveModel(std::string filePath);

	void train();

	Matrix& predictLabelScoreMatrix(Matrix& Xt);

};

#endif /* LINEARMCSVM_H_ */
