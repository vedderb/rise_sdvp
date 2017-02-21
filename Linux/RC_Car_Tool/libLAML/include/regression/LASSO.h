/*
 * LASSO.h
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#ifndef LASSO_H_
#define LASSO_H_

#include "Regression.h"
#include "Matrix.h"
#include "Matlab.h"
#include "InPlaceOperator.h"
#include "Printer.h"
#include <string>
#include <list>

/***
 * A C++ implementation of LASSO, which solves the following
 * convex optimization problem:
 * </p>
 * min_W 2\1 || Y - X * W ||_F^2 + lambda * || W ||_1</br>
 * where X is an n-by-p data matrix with each row bing a p
 * dimensional data vector and Y is an n-by-ny dependent
 * variable matrix.
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 3rd, 2014
 */
class LASSO : public Regression {

private:

	/**
	 * Regularization parameter.
	 */
	double lambda;

	/**
	 * If compute objective function values during
	 * the iterations or not.
	 */
	bool calc_OV;

	/**
	 * If show computation detail during iterations or not.
	 */
	bool verbose;

public:

	LASSO();

	LASSO(double epsilon);

	LASSO(int maxIter, double epsilon);

	LASSO(double lambda, int maxIter, double epsilon);

	LASSO(Options& options);

	void train();

	void loadModel(std::string filePath);

	void saveModel(std::string filePath);

	static Matrix& train(Matrix& X, Matrix& Y, Options& options);

	Matrix& train(Matrix& X, Matrix& Y);

};

#endif /* LASSO_H_ */
