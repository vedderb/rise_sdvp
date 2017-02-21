/*
 * Regression.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#ifndef REGRESSION_H_
#define REGRESSION_H_

#include <string>
#include "Matrix.h"
#include "Utility.h"
#include "Options.h"
#include "DenseMatrix.h"
#include "Printer.h"

/***
 * Base class for all regression methods.
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 3rd, 2014
 */
class Regression {

public:

	/**
	 * Number of dependent variables.
	 */
	int ny;

	/**
	 * Number of independent variables.
	 */
	int p;

	/**
	 * Number of samples.
	 */
	int n;

	/**
	 * Training data matrix (n x p) with each row being
	 * a data example.
	 */
	Matrix* X;

	/**
	 * Dependent variable matrix for training (n x ny).
	 */
	Matrix* Y;

	/**
	 * Unknown parameters represented as a matrix (p x ny).
	 */
	Matrix* W;

	/**
	 * Convergence tolerance.
	 */
	double epsilon;

	/**
	 * Maximal number of iterations.
	 */
	int maxIter;

	Regression();

	Regression(double epsilon);

	Regression(int maxIter, double epsilon);

	Regression(Options& options);

	virtual ~Regression();

	/**
	 * Feed training data for the regression model.
	 *
	 * @param X data matrix with each row being a data example
	 */
	void feedData(Matrix& X);

	/**
	 * Feed training data for this regression model.
	 *
	 * @param data an n x d 2D {@code double} array with each
	 *             row being a data example
	 *
	 * @param n number of training examples
	 *
	 * @param d number of features
	 */
	void feedData(double** data, int n, int d);

	template<int d>
	void feedData(double data[][d], int n) {
		feedData(DenseMatrix::createDenseMatrix<d>(data, n));
	}

	template<int numFeature>
	void feedData(double data[][numFeature], int n, int d) {
		feedData(DenseMatrix::createDenseMatrix<numFeature>(data, n, d));
	}

	/**
	 * Feed training dependent variables for this regression model.
	 *
	 * @param Y dependent variable matrix for training with each row being
	 *          the dependent variable vector for each data training data
	 *          example
	 */
	void feedDependentVariables(Matrix& Y);

	/**
	 * Feed training dependent variables for this regression model.
	 *
	 * @param depVars an n x c 2D {@code double} array
	 *
	 * @param n number of training examples
	 *
	 * @param c number of dependent variables
	 */
	void feedDependentVariables(double** depVars, int n, int c);

	template<int d>
	void feedDependentVariables(double depVars[][d], int n) {
		feedDependentVariables(DenseMatrix::createDenseMatrix<d>(depVars, n));
	}

	template<int numFeature>
	void feedDependentVariables(double depVars[][numFeature], int n, int d) {
		feedDependentVariables(DenseMatrix::createDenseMatrix<numFeature>(depVars, n, d));
	}

	/**
	 * Train the regression model.
	 */
	virtual void train() = 0;

	virtual Matrix& train(Matrix& X, Matrix& Y) = 0;

	/**
	 * Predict the dependent variables for test data Xt.
	 *
	 * @param Xt test data matrix with each row being a
	 *           data example.
	 *
	 * @return dependent variables for Xt
	 *
	 */
	Matrix& predict(Matrix& Xt);

	/**
	 * Predict the dependent variables for test data Xt.
	 *
	 * @param Xt an n x d 2D {@code double} array with each
	 *           row being a data example
	 *
	 * @param nt number of test examples
	 *
	 * @param d number of features
	 *
	 * @return dependent variables for Xt
	 *
	 */
	Matrix& predict(double** Xt, int nt, int d);

	template<int d>
	Matrix& predict(double Xt[][d], int nt) {
		return predict(DenseMatrix::createDenseMatrix<d>(Xt, nt));
	}

	template<int numFeature>
	Matrix& predict(double Xt[][numFeature], int nt, int d) {
		return predict(DenseMatrix::createDenseMatrix<numFeature>(Xt, nt, d));
	}

	virtual void loadModel(std::string filePath) = 0;

	virtual void saveModel(std::string filePath) = 0;

};

#endif /* REGRESSION_H_ */
