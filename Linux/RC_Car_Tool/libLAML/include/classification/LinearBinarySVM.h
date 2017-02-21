/*
 * LinearBinarySVM.h
 *
 *  Created on: Feb 16, 2014
 *      Author: Mingjie Qian
 */

#ifndef LINEARBINARYSVM_H_
#define LINEARBINARYSVM_H_

#include "Classifier.h"
#include "Matrix.h"
#include <iostream>

/***
 * A C++ implementation of fast linear binary SVM. It uses
 * sparse representation of the feature vectors, and updates
 * the weight vectors and dual variables by dual coordinate
 * descent.
 *
 * <p>
 * The memory complexity is O(l*d_s) + O(d) + O(l*d_s) and the
 * computation complexity is O(l*d_s), where d_s is the average
 * number of non-zero features, d is the feature size, and l is
 * the training sample size.
 * </p>
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 16th, 2014
 */
class LinearBinarySVM : public Classifier {

private:

	/**
	 * Parameter for loss term.
	 */
	double C;

	/**
	 * Convergence tolerance.
	 */
	double eps;

	/**
	 * Compute the inner product <W, [X(i, :) 1]>.
	 *
	 * @param W
	 * @param X
	 * @param i
	 * @param pr_CSR
	 * @return <W, [X(i, :) 1]>
	 */
	double innerProduct(double* W, Matrix& X, int i, double* pr_CSR);

	double* computeQ(Matrix& X, double* pr_CSR);

	/**
	 * Update W in place, i.e., W <- W + v * [X(i, :) 1]'.
	 * @param W
	 * @param v
	 * @param X
	 * @param i
	 * @param pr_CSR
	 */
	void updateW(double* W, double v, Matrix& X, int i, double* pr_CSR);

public:

	LinearBinarySVM();

	LinearBinarySVM(double C, double eps);

	friend std::ostream& operator<<(std::ostream&, const LinearBinarySVM&);

	friend std::istream& operator>>(std::istream&, LinearBinarySVM&);

	void loadModel(std::string filePath);

	void saveModel(std::string filePath);

	void train();

	Matrix& predictLabelScoreMatrix(Matrix& Xt);

};


#endif /* LINEARBINARYSVM_H_ */
