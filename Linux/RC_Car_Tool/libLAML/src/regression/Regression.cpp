/*
 * Regression.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#include "Regression.h"

Regression::Regression() {
	ny = 0;
	p = 0;
	n = 0;
	X = null;
	Y = null;
	W = null;
	epsilon = 1e-6;
	maxIter = 600;
}

Regression::Regression(double epsilon) {
	ny = 0;
	p = 0;
	n = 0;
	X = null;
	Y = null;
	W = null;
	this->epsilon = epsilon;
	maxIter = 600;
}

Regression::Regression(int maxIter, double epsilon) {
	ny = 0;
	p = 0;
	n = 0;
	X = null;
	Y = null;
	W = null;
	this->epsilon = epsilon;
	this->maxIter = maxIter;
}

Regression::Regression(Options& options) {
	ny = 0;
	p = 0;
	n = 0;
	X = null;
	Y = null;
	W = null;
	epsilon = options.epsilon;
	maxIter = options.maxIter;
}

Regression::~Regression() {
	if (X != null)
		delete X;
	if (Y != null)
		delete Y;
	if (W != null)
		delete W;
}

/**
 * Feed training data for the regression model.
 *
 * @param X data matrix with each row being a data example
 */
void Regression::feedData(Matrix& X) {
	this->X = &X;
	p = X.getColumnDimension();
	n = X.getRowDimension();
	if (Y != null && X.getRowDimension() != Y->getRowDimension()) {
		err("The number of dependent variable vectors and the number of data samples do not match!");
		exit(1);
	}
}

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
void Regression::feedData(double** data, int n, int d) {
	feedData(*new DenseMatrix(data, n, d));
}

/**
 * Feed training dependent variables for this regression model.
 *
 * @param Y dependent variable matrix for training with each row being
 *          the dependent variable vector for each data training data
 *          example
 */
void Regression::feedDependentVariables(Matrix& Y) {
	this->Y = &Y;
	ny = Y.getColumnDimension();
	if (X != null && Y.getRowDimension() != n) {
		err("The number of dependent variable vectors and the number of data samples do not match!");
		exit(1);
	}
}

/**
 * Feed training dependent variables for this regression model.
 *
 * @param depVars an n x c 2D {@code double} array
 *
 * @param n number of training examples
 *
 * @param c number of dependent variables
 */
void Regression::feedDependentVariables(double** depVars, int n, int c) {
	feedDependentVariables(*new DenseMatrix(depVars, n, c));
}

/**
 * Predict the dependent variables for test data Xt.
 *
 * @param Xt test data matrix with each row being a
 *           data example.
 *
 * @return dependent variables for Xt
 *
 */
Matrix& Regression::predict(Matrix& Xt) {
	if (Xt.getColumnDimension() != p) {
		err("Dimensionality of the test data doesn't match with the training data!");
		exit(1);
	}
	return Xt.mtimes(*W);
}

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
Matrix& Regression::predict(double** Xt, int nt, int d) {
	return predict(*new DenseMatrix(Xt, nt, d));
}
