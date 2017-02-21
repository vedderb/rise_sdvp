/*
 * LASSO.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#include "LASSO.h"

LASSO::LASSO() : Regression() {
	lambda = 1;
	calc_OV = false;
	verbose = false;
}

LASSO::LASSO(double epsilon) : Regression(epsilon) {
	lambda = 1;
	calc_OV = false;
	verbose = false;
}

LASSO::LASSO(int maxIter, double epsilon) : Regression(maxIter, epsilon) {
	lambda = 1;
	calc_OV = false;
	verbose = false;
}

LASSO::LASSO(double lambda, int maxIter, double epsilon) : Regression(maxIter, epsilon) {
	this->lambda = lambda;
	calc_OV = false;
	verbose = false;
}

LASSO::LASSO(Options& options) : Regression(options) {
	lambda = options.lambda;
	calc_OV = options.calc_OV;
	verbose = options.verbose;
}

void LASSO::train() {
	W = &train(*X, *Y);
}

void LASSO::loadModel(std::string filePath) {

}

void LASSO::saveModel(std::string filePath) {

}

Matrix& LASSO::train(Matrix& X, Matrix& Y, Options& options) {

	int p = size(X, 2);
	int ny = size(Y, 2);
	double epsilon = options.epsilon;
	int maxIter = options.maxIter;
	double lambda = options.lambda;
	bool calc_OV = options.calc_OV;
	bool verbose = options.verbose;

	/*XNX = [X, -X];
		H_G = XNX' * XNX;
		D = repmat(diag(H_G), [1, n_y]);
		XNXTY = XNX' * Y;
	    A = (X' * X + lambda  * eye(p)) \ (X' * Y);*/

	Matrix& XNX = horzcat(2, &X, &uminus(X));
	Matrix& H_G = XNX.transpose().mtimes(XNX);
	double* Q = new double[size(H_G, 1)];
	for (int i = 0; i < size(H_G, 1); i++) {
		Q[i] = H_G.getEntry(i, i);
	}
	Matrix& XNXTY = XNX.transpose().mtimes(Y);
	Matrix& A = mldivide(
			plus(X.transpose().mtimes(X), times(lambda, eye(p))),
			X.transpose().mtimes(Y)
	);

	/*AA = [subplus(A); subplus(-A)];
		C = -XNXTY + lambda;
		Grad = C + H_G * AA;
		tol = epsilon * norm(Grad);
		PGrad = zeros(size(Grad));*/

	Matrix& AA = vertcat(2, &subplus(A), &subplus(uminus(A)));
	Matrix& C = plus(uminus(XNXTY), lambda);
	Matrix& Grad = plus(C, mtimes(H_G, AA));
	double tol = epsilon * norm(Grad);
	Matrix& PGrad = zeros(size(Grad));

	std::list<double> J;
	double fval = 0;
	// J(1) = sum(sum((Y - X * A).^2)) / 2 + lambda * sum(sum(abs(A)));
	if (calc_OV) {
		fval = sum(sum(pow(minus(Y, mtimes(X, A)), 2))) / 2 +
				lambda * sum(sum(abs(A)));
		J.push_back(fval);
	}

	Matrix& I_k = Grad.copy();
	double d = 0;
	int k = 0;

	DenseVector& SFPlusCi = *new DenseVector(AA.getColumnDimension());
	Matrix& S = H_G;
	Vector** SRows = null;
	if (typeid(H_G) == typeid(DenseMatrix))
		SRows = denseMatrix2DenseRowVectors(S);
	else
		SRows = sparseMatrix2SparseRowVectors(S);

	Vector** CRows = null;
	if (typeid(C) == typeid(DenseMatrix))
		CRows = denseMatrix2DenseRowVectors(C);
	else
		CRows = sparseMatrix2SparseRowVectors(C);

	double** FData = ((DenseMatrix&) AA).getData();
	double* FRow = null;
	double* pr = null;
	int K = 2 * p;

	while (true) {

		/*I_k = Grad < 0 | AA > 0;
		    I_k_com = not(I_k);
		    PGrad(I_k) = Grad(I_k);
		    PGrad(I_k_com) = 0;*/

		_or(I_k, lt(Grad, 0), gt(AA, 0));
		Matrix& I_k_com = _not(I_k);
		assign(PGrad, Grad);
		logicalIndexingAssignment(PGrad, I_k_com, 0);

		d = norm(PGrad, inf);
		if (d < tol) {
			if (verbose)
				println("Converge successfully!");
			break;
		}

		/*for i = 1:2*p
		            AA(i, :) = max(AA(i, :) - (C(i, :) + H_G(i, :) * AA) ./ (D(i, :)), 0);
		    end
		    A = AA(1:p,:) - AA(p+1:end,:);*/

		for (int i = 0; i < K; i++) {
			// SFPlusCi = SRows[i].operate(AA);
			operate(SFPlusCi, *SRows[i], AA);
			plusAssign(SFPlusCi, *CRows[i]);
			timesAssign(SFPlusCi, 1 / Q[i]);
			pr = SFPlusCi.getPr();
			// F(i, :) = max(F(i, :) - (S(i, :) * F + C(i, :)) / D[i]), 0);
			// F(i, :) = max(F(i, :) - SFPlusCi, 0)
			FRow = FData[i];
			for (int j = 0; j < AA.getColumnDimension(); j++) {
				FRow[j] = max(FRow[j] - pr[j], 0);
			}
		}

		// Grad = plus(C, mtimes(H_G, AA));
		plus(Grad, C, mtimes(H_G, AA));

		k = k + 1;
		if (k > maxIter) {
			if (verbose)
				println("Maximal iterations");
			break;
		}

		if (calc_OV) {
			fval = sum(sum(pow(minus(Y, mtimes(XNX, AA)), 2))) / 2 +
					lambda * sum(sum(abs(AA)));
			J.push_back(fval);
		}

		if (k % 10 == 0 && verbose) {
			if (calc_OV)
				fprintf("Iter %d - ||PGrad||: %f, ofv: %f\n", k, d, J.back());
			else
				fprintf("Iter %d - ||PGrad||: %f\n", k, d);

		}

	}

	Matrix& res = minus(
			AA.getSubMatrix(0, p - 1, 0, ny - 1),
			AA.getSubMatrix(p, 2 * p - 1, 0, ny - 1)
	);

	return res;

}

Matrix& LASSO::train(Matrix& X, Matrix& Y) {

	/*XNX = [X, -X];
		H_G = XNX' * XNX;
		D = repmat(diag(H_G), [1, n_y]);
		XNXTY = XNX' * Y;
	    A = (X' * X + lambda  * eye(p)) \ (X' * Y);*/

	Matrix& XNX = horzcat(2, &X, &uminus(X));
	Matrix& H_G = XNX.transpose().mtimes(XNX);
	double* Q = new double[size(H_G, 1)];
	for (int i = 0; i < size(H_G, 1); i++) {
		Q[i] = H_G.getEntry(i, i);
	}
	Matrix& XNXTY = XNX.transpose().mtimes(Y);
	Matrix& A = mldivide(
			plus(X.transpose().mtimes(X), times(lambda, eye(p))),
			X.transpose().mtimes(Y)
	);

	/*AA = [subplus(A); subplus(-A)];
		C = -XNXTY + lambda;
		Grad = C + H_G * AA;
		tol = epsilon * norm(Grad);
		PGrad = zeros(size(Grad));*/

	Matrix& AA = vertcat(2, &subplus(A), &subplus(uminus(A)));
	Matrix& C = plus(uminus(XNXTY), lambda);
	Matrix& Grad = plus(C, mtimes(H_G, AA));
	double tol = epsilon * norm(Grad);
	Matrix& PGrad = zeros(size(Grad));

	std::list<double> J;
	double fval = 0;
	// J(1) = sum(sum((Y - X * A).^2)) / 2 + lambda * sum(sum(abs(A)));
	if (calc_OV) {
		fval = sumAll(pow(minus(Y, mtimes(X, A)), 2)) / 2 +
				lambda * sum(sum(abs(A)));
		J.push_back(fval);
	}

	/*Matrix I_k = null;
		Matrix I_k_com = null;*/
	Matrix& I_k = Grad.copy();
	double d = 0;
	int k = 0;

	DenseVector& SFPlusCi = *new DenseVector(AA.getColumnDimension());
	Matrix& S = H_G;
	Vector** SRows = null;
	if (typeid(H_G) == typeid(DenseMatrix))
		SRows = denseMatrix2DenseRowVectors(S);
	else
		SRows = sparseMatrix2SparseRowVectors(S);

	Vector** CRows = null;
	if (typeid(C) == typeid(DenseMatrix))
		CRows = denseMatrix2DenseRowVectors(C);
	else
		CRows = sparseMatrix2SparseRowVectors(C);

	double** FData = ((DenseMatrix&) AA).getData();
	double* FRow = null;
	double* pr = null;
	int K = 2 * p;

	while (true) {

		/*I_k = Grad < 0 | AA > 0;
		    I_k_com = not(I_k);
		    PGrad(I_k) = Grad(I_k);
		    PGrad(I_k_com) = 0;*/

		_or(I_k, lt(Grad, 0), gt(AA, 0));
		Matrix& I_k_com = _not(I_k);
		assign(PGrad, Grad);
		logicalIndexingAssignment(PGrad, I_k_com, 0);

		d = norm(PGrad, inf);
		if (d < tol) {
			if (verbose)
				println("Converge successfully!");
			break;
		}

		/*for i = 1:2*p
		            AA(i, :) = max(AA(i, :) - (C(i, :) + H_G(i, :) * AA) ./ (D(i, :)), 0);
		    end
		    A = AA(1:p,:) - AA(p+1:end,:);*/

		for (int i = 0; i < K; i++) {
			// SFPlusCi = SRows[i].operate(AA);
			operate(SFPlusCi, *SRows[i], AA);
			plusAssign(SFPlusCi, *CRows[i]);
			timesAssign(SFPlusCi, 1 / Q[i]);
			pr = SFPlusCi.getPr();
			// F(i, :) = max(F(i, :) - (S(i, :) * F + C(i, :)) / D[i]), 0);
			// F(i, :) = max(F(i, :) - SFPlusCi, 0)
			FRow = FData[i];
			for (int j = 0; j < AA.getColumnDimension(); j++) {
				FRow[j] = max(FRow[j] - pr[j], 0);
			}
		}

		// Grad = plus(C, mtimes(H_G, AA));
		plus(Grad, C, mtimes(H_G, AA));

		k = k + 1;
		if (k > maxIter) {
			if (verbose)
				println("Maximal iterations");
			break;
		}

		if (calc_OV) {
			fval = sum(sum(pow(minus(Y, mtimes(XNX, AA)), 2))) / 2 +
					lambda * sum(sum(abs(AA)));
			J.push_back(fval);
		}

		if (k % 10 == 0 && verbose) {
			if (calc_OV)
				fprintf("Iter %d - ||PGrad||: %f, ofv: %f\n", k, d, J.back());
			else
				fprintf("Iter %d - ||PGrad||: %f\n", k, d);

		}

	}

	Matrix& res = minus(
			AA.getSubMatrix(0, p - 1, 0, ny - 1),
			AA.getSubMatrix(p, 2 * p - 1, 0, ny - 1)
	);
	return res;

}
