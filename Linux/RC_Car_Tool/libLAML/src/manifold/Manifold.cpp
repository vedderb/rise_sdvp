/*
 * Manifold.cpp
 *
 *  Created on: Feb 28, 2014
 *      Author: Mingjie Qian
 */

#include "Manifold.h"
#include "Utility.h"
#include "SparseVector.h"
#include "Printer.h"
#include "Matlab.h"
#include "Kernel.h"
#include "InPlaceOperator.h"
#include <cmath>

/**
 * Calculate the graph Laplacian of the adjacency graph of a data set
 * represented as rows of a matrix X.
 *
 * @param X data matrix with each row being a sample
 *
 * @param type graph type, either "nn" or "epsballs"
 *
 * @param options
 *        data structure containing the following fields
 *        NN - integer if type is "nn" (number of nearest neighbors),
 *             or size of "epsballs"
 *        DISTANCEFUNCTION - distance function used to make the graph
 *        WEIGHTTYPPE = "binary" | "distance" | "heat" | "inner"
 * 	      WEIGHTPARAM = width for heat kernel
 * 	      NORMALIZE = 0 | 1 whether to return normalized graph Laplacian or not
 *
 * @return a sparse symmetric N x N matrix
 *
 */
Matrix& laplacian(Matrix& X, std::string type, GraphOptions& options) {

	println("Computing Graph Laplacian...");

	double NN = options.graphParam;
	std::string DISTANCEFUNCTION = options.graphDistanceFunction;
	std::string WEIGHTTYPE = options.graphWeightType;
	double WEIGHTPARAM = options.graphWeightParam;
	bool NORMALIZE = options.graphNormalize;

	if (WEIGHTTYPE == "inner" && DISTANCEFUNCTION != "cosine")
		err("WEIGHTTYPE and DISTANCEFUNCTION mismatch.");

	// Calculate the adjacency matrix for DATA
	Matrix& A = adjacency(X, type, NN, DISTANCEFUNCTION);

	// W could be viewed as a similarity matrix
	Matrix& W = A.copy();

	// Disassemble the sparse matrix
	FindResult& findResult = find(A);
	int* A_i = findResult.rows;
	int* A_j = findResult.cols;
	double* A_v = findResult.vals;
	int length = findResult.len;

	if (WEIGHTTYPE == "distance") {
		for (int i = 0; i < length; i++) {
			W.setEntry(A_i[i], A_j[i], A_v[i]);
		}
	} else if (WEIGHTTYPE == "inner") {
		for (int i = 0; i < length; i++) {
			W.setEntry(A_i[i], A_j[i], 1 - A_v[i] / 2);
		}
	} else if (WEIGHTTYPE == "binary") {
		for (int i = 0; i < length; i++) {
			W.setEntry(A_i[i], A_j[i], 1);
		}
	} else if (WEIGHTTYPE == "heat") {
		double t = -2 * WEIGHTPARAM * WEIGHTPARAM;
		for (int i = 0; i < length; i++) {
			W.setEntry(A_i[i], A_j[i],
					exp(A_v[i] * A_v[i] / t));
		}
	} else {
		err("Unknown Weight Type.");
	}
	delete &findResult;

	Vector& V = sum(W, 2);
	Matrix* L = null;
	if (!NORMALIZE) {
		Matrix& diagV = diag(V);
		L = &diagV.minus(W);
		delete &diagV;
		// L = &diag(V).minus(W);
	} else {
		// Normalized Laplacian
		Vector& sqrtV = sqrt(V);
		Vector& dotDivideSqrtV = dotDivide(1.0, sqrtV);
		Matrix& D = diag(dotDivideSqrtV);
		// Matrix& D = diag(dotDivide(1, sqrt(V)));
		Matrix& DW = D.mtimes(W);
		Matrix& DWD = DW.mtimes(D);
		Matrix& I = speye(size(W, 1));
		L = &I.minus(DWD);
		delete &sqrtV;
		delete &dotDivideSqrtV;
		delete &D;
		delete &DW;
		delete &DWD;
		delete &I;
		// L = &speye(size(W, 1)).minus(D.mtimes(W).mtimes(D));
	}
	delete &V;
	return *L;

}

/**
 * Compute the symmetric adjacency matrix of the data set represented as
 * a real data matrix X. The diagonal elements of the sparse symmetric
 * adjacency matrix are all zero indicating that a sample should not be
 * a neighbor of itself. Note that in some cases, neighbors of a sample
 * may coincide with the sample itself, we set eps for those entries in
 * the sparse symmetric adjacency matrix.
 *
 * @param X data matrix with each row being a feature vector
 *
 * @param type graph type, either "nn" or "epsballs" ("eps")
 *
 * @param param integer if type is "nn", real number if type is "epsballs" ("eps")
 *
 * @param distFunc function mapping an (M x D) and a (N x D) matrix
 *                 to an M x N distance matrix (D: dimensionality)
 *                 either "euclidean" or "cosine"
 *
 * @return a sparse symmetric N x N  matrix of distances between the
 *         adjacent points
 *
 */
Matrix& adjacency(Matrix& X, std::string type, double param, std::string distFunc) {
	Matrix& A = adjacencyDirected(X, type, param, distFunc);
	Matrix& AT = A.transpose();
	Matrix& res = max(A, AT);
	delete &A;
	delete &AT;
	return res;
	// return max(A, A.transpose());
}

/**
 * Compute the directed adjacency matrix of the data set represented as
 * a real data matrix X. The diagonal elements of the sparse directed
 * adjacency matrix are all zero indicating that a sample should not be
 * a neighbor of itself. Note that in some cases, neighbors of a sample
 * may coincide with the sample itself, we set eps for those entries in
 * the sparse directed adjacency matrix.
 *
 * @param X data matrix with each row being a feature vector
 *
 * @param type graph type, either "nn" or "epsballs" ("eps")
 *
 * @param param integer if type is "nn", real number if type is "epsballs" ("eps")
 *
 * @param distFunc function mapping an (M x D) and an (N x D) matrix
 *                 to an M x N distance matrix (D: dimensionality)
 *                 either "euclidean" or "cosine"
 *
 * @return a sparse N x N matrix of distances between the
 *         adjacent points, not necessarily symmetric
 *
 */
Matrix& adjacencyDirected(Matrix& X, std::string type, double param, std::string distFunc) {

	println("Computing directed adjacency graph...");

	int n = size(X, 1);

	if (type == "nn") {
		fprintf("Creating the adjacency matrix. Nearest neighbors, N = %d.\n", (int)param);
	} else if (type == "epsballs" || type == "eps") {
		fprintf("Creating the adjacency matrix. Epsilon balls, eps = %f.\n", param);
	} else {
		err("type should be either \"nn\" or \"epsballs\" (\"eps\")");
		exit(1);
	}

	Matrix& A = *new SparseMatrix(n, n);

	Vector* Zt = null;
	// Vector* X_i = null;
	for (int i = 0; i < n; i++) {

		/*if (i == 9) {
				int a = 0;
				a = a + 1;
			}*/
		Vector& X_i = X.getRowVector(i);
		// disp(X_i);
		if (distFunc == "euclidean") {
			Zt = &euclidean(X_i, X);
		} else if (distFunc == "cosine") {
			Zt = &cosine(X_i, X);
		}

		if (typeid(X_i) == typeid(SparseVector))
			delete &X_i;
		Vector& Z = *Zt;

		/*disp("Z:");
		disp(Z);*/
		double* IX = sort(Z);

		/*disp("Z:");
		disp(Z);
		disp(IX, Z.getDim());*/

		if (type == "nn") {
			for (int j = 0; j <= param; j++ ) {
				if (IX[j] != i)
					A.setEntry(i, (int)IX[j], Z.get(j) + eps);
			}
		} else if (type == "epsballs" || type == "eps") {
			int j = 0;
			while (Z.get(j) <= param) {
				if (IX[j] != i)
					A.setEntry(i, (int)IX[j], Z.get(j) + eps);
				j++;
			}
		}

		delete[] IX;
		delete &Z;

	}

	return A;

}

/**
 * Compute the cosine distance matrix between row vectors in matrix A
 * and row vectors in matrix B.
 *
 * @param A data matrix with each row being a feature vector
 *
 * @param B data matrix with each row being a feature vector
 *
 * @return an n_A X n_B matrix with its (i, j) entry being the cosine
 * distance between i-th feature vector in A and j-th feature
 * vector in B, i.e.,
 * ||A(i, :) - B(j, :)|| = 1 - A(i, :)' * B(j, :) / || A(i, :) || * || B(j, :)||
 */
Matrix& cosine(Matrix& A, Matrix& B) {

	Matrix& AXA = times(A, A);
	Matrix& BXB = times(B, B);
	DenseVector& SumA = sum(AXA, 2);
	DenseVector& SumB = sum(BXB, 2);
	delete &AXA;
	delete &BXB;

	double* AA = SumA.getPr();
	double* BB = SumB.getPr();

	/*double* AA = sum(times(A, A), 2).getPr();
	double* BB = sum(times(B, B), 2).getPr();*/
	Matrix& BT = B.transpose();
	Matrix& AB = A.mtimes(BT);
	delete &BT;
	int M = AB.getRowDimension();
	int N = AB.getColumnDimension();
	Matrix* res = null;
	double v = 0;
	if (typeid(AB) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) AB).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				v = resRow[j];
				resRow[j] = 1 - v / sqrt(AA[i] * BB[j]);
			}
		}
		res = &AB;
	} else {
		double* pr = ((SparseMatrix&) AB).getPr();
		int* ir = ((SparseMatrix&) AB).getIr();
		int* jc = ((SparseMatrix&) AB).getJc();
		for (int j = 0; j < N; j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				pr[k] /= -sqrt(AA[ir[k]] * BB[j]);
			}
		}
		res = new DenseMatrix(M, N, 1);
		// AB = AB.plus(1);
		plusAssign(*res, AB);
		delete &AB;
	}

	delete &SumA;
	delete &SumB;

	return *res;

}

/**
 * Compute the cosine distance matrix between a vector V
 * and row vectors in matrix B.
 *
 * @param V a feature vector
 *
 * @param B data matrix with each row being a feature vector
 *
 * @return an n_B dimensional vector with its i-th entry being the cosine
 * distance between V and the i-th feature vector in B, i.e.,
 * ||V - B(j, :)|| = 1 - V' * B(j, :) / || V || * || B(j, :)||
 */
Vector& cosine(Vector& V, Matrix& B) {

	Vector& VXV = times(V, V);
	Matrix& BXB = times(B, B);
	DenseVector& SumB = sum(BXB, 2);
	double AA = sum(VXV);
	double* BB = SumB.getPr();
	delete &VXV;
	delete &BXB;

	// double AA = sum(times(V, V));
	// double* BB = sum(times(B, B), 2).getPr();
	Vector& BV = B.operate(V);
	DenseVector& AB = full(BV);
	if (typeid(BV) == typeid(SparseVector))
		delete &BV;
	int dim = AB.getDim();
	double v = 0;
	double* pr = ((DenseVector&) AB).getPr();
	for (int j = 0; j < dim; j++) {
		v = pr[j];
		pr[j] = 1 - v / sqrt(AA * BB[j]);
	}
	delete &SumB;
	return AB;

}

/**
 * Compute the Euclidean distance matrix between row vectors in matrix A
 * and row vectors in matrix B.
 *
 * @param A data matrix with each row being a feature vector
 *
 * @param B data matrix with each row being a feature vector
 *
 * @return an n_A X n_B matrix with its (i, j) entry being Euclidean
 * distance between i-th feature vector in A and j-th feature
 * vector in B, i.e., || X(i, :) - Y(j, :) ||_2
 *
 */
Matrix& euclidean(Matrix& A, Matrix& B) {
	return l2Distance(A, B);
}

/**
 * Compute the Euclidean distance matrix between a vector V
 * and row vectors in matrix B.
 *
 * @param V a feature vector
 *
 * @param B data matrix with each row being a feature vector
 *
 * @return an n_B dimensional vector with its i-th entry being Euclidean
 * distance between V and the i-th feature vector in B, i.e., || V - Y(i, :) ||_2
 *
 */
Vector& euclidean(Vector& V, Matrix& B) {
	return l2Distance(V, B);
}

/**
 * Compute local learning regularization matrix. Local learning
 * regularization only depends on kernel selection, distance
 * function, and neighborhood size.
 *
 * @param X data matrix with each row being a feature vector
 *
 * @param NN number of nearest neighbor
 *
 * @param distFunc function mapping an (M x D) and an (N x D) matrix
 *        to an M x N distance matrix (D: dimensionality)
 *        either "euclidean" or "cosine"
 *
 * @param kernelType  'linear' | 'poly' | 'rbf' | 'cosine'
 *
 * @param kernelParam    --    | degree | sigma |    --
 *
 * @param lambda graph regularization parameter
 *
 * @return local learning regularization matrix
 *
 */
Matrix& calcLLR(Matrix& X,
		double NN, std::string distFunc, std::string kernelType,
		double kernelParam, double lambda) {

	std::string type = "nn";
	double param = NN;
	Matrix& A = adjacencyDirected(X, type, param, distFunc);
	Matrix& K = calcKernel(kernelType, kernelParam, X);

	int NSample = size(X, 1);
	int n_i = (int)param;
	Matrix& I_i = eye(n_i);
	Matrix& I = speye(NSample);

	Matrix* G = &A.copy();

	int* neighborIndices_i = null;
	// Matrix& neighborhood_X_i = null;
	Vector** neighborhood_X_i = null;
	// Matrix& K_i = null;
	Vector* x_i = null;
	// int* IDs = colon(0, NFeature - 1);
	Vector** Vs = sparseMatrix2SparseRowVectors(A);
	Vector** Xs = null;
	if (typeid(X) == typeid(DenseMatrix)) {
		Xs = denseMatrix2DenseRowVectors(X);
	} else {
		Xs = sparseMatrix2SparseRowVectors(X);
	}
	Vector** Gs = new Vector*[NSample];
	for (int i = 0; i < NSample; i++) {
		// neighborIndices_i = find(A.getRowVector(i));
		neighborIndices_i = find(*Vs[i]);
		int length = ((SparseVector*) Vs[i])->getNNZ();
		// neighborhood_X_i = X.getSubMatrix(IDs, neighborIndices_i);
		neighborhood_X_i = new Vector*[length];
		for (int k = 0; k < length; k++) {
			neighborhood_X_i[k] = Xs[neighborIndices_i[k]];
		}
		Matrix& K_i = K.getSubMatrix(neighborIndices_i, length, neighborIndices_i, length);
		// x_i = X.getColumnMatrix(i);
		x_i = Xs[i];
		Vector** X_i = new Vector*[1];
		X_i[0] = x_i;
		Matrix& k_i = calcKernel(kernelType, kernelParam, X_i, 1, neighborhood_X_i, length);
		Matrix& alpha_i = mrdivide(k_i, I_i.times(n_i * lambda).plus(K_i));
		// setSubMatrix(G, new int*{i}, neighborIndices_i, alpha_i);
		Gs[i] = new SparseVector(neighborIndices_i, ((DenseMatrix&) alpha_i).getData()[0], length, NSample);
	}
	G = &sparseRowVectors2SparseMatrix(Gs, NSample);
	Matrix& T = G->minus(I);
	Matrix& L = T.transpose().mtimes(T);

	return L;

}
