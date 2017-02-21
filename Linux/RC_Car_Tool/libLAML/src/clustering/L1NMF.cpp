/*
 * L1NMF.cpp
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#include "L1NMF.h"

L1NMF::L1NMF(Options& options) {
	maxIter = options.maxIter;
	epsilon = options.epsilon;
	gamma = options.gamma;
	mu = options.mu;
	verbose = options.verbose;
	calc_OV = options.calc_OV;
	nClus = options.nClus;
	initializer = null;
}

L1NMF::L1NMF(L1NMFOptions& L1NMFOptions) {
	maxIter = L1NMFOptions.maxIter;
	epsilon = L1NMFOptions.epsilon;
	gamma = L1NMFOptions.gamma;
	mu = L1NMFOptions.mu;
	verbose = L1NMFOptions.verbose;
	calc_OV = L1NMFOptions.calc_OV;
	nClus = L1NMFOptions.nClus;
	initializer = null;
}

L1NMF::L1NMF() {
	L1NMFOptions& options = *new L1NMFOptions();
	maxIter = options.maxIter;
	epsilon = options.epsilon;
	gamma = options.gamma;
	mu = options.mu;
	verbose = options.verbose;
	calc_OV = options.calc_OV;
	nClus = options.nClus;
	initializer = null;
}

void L1NMF::initialize(Matrix* G0) {

	if (G0 != null) {
		initializer = G0;
		return;
	}

	KMeansOptions& kMeansOptions = *new KMeansOptions();
	kMeansOptions.nClus = nClus;
	kMeansOptions.maxIter = 50;
	kMeansOptions.verbose = true;

	println("Using KMeans to initialize...");
	Clustering& kmeans = *new KMeans(kMeansOptions);
	kmeans.feedData(*dataMatrix);
	// KMeans.initialize(null);
	kmeans.clustering();

	initializer = &kmeans.getIndicatorMatrix()->copy();
	delete &kmeans;

}

void L1NMF::clustering() {
	if (initializer == null) {
		initialize(null);
		// initializer = indicatorMatrix;
	}
	clustering(initializer);
}

void L1NMF::clustering(Matrix* G0) {

	if (G0 == null) {
		initialize(null);
		G0 = initializer;
	}

	Matrix& X = *dataMatrix;
	// disp(*G0);
	Matrix& G = full(*G0);
	// Matrix F = X.mtimes(G).mtimes(new LUDecompositionImpl(G.transpose().mtimes(G)).getSolver().getInverse());
	Matrix& F0 = mldivide(G.transpose().mtimes(G), G.transpose().mtimes(X));
	// G = full(G);

	Matrix& F_pos = subplus(F0);
	Matrix& F = F_pos.plus(0.2 * sumAll(F_pos) / find(F_pos).len);

	Matrix& E_F = ones(size(F)).times(gamma / 2);
	Matrix& E_G = ones(size(G)).times( mu / 2 );

	if (calc_OV) {
		valueList.push_back(f(X, F, G, E_F, E_G));
	}

	int ind = 0;
	Matrix* G_old = new DenseMatrix(size(G));
	double d = 0;

	while (true) {

		// G_old.setSubMatrix(G.getData(), 0, 0);
		assign(*G_old, G);

		// Fixing F, updating G
		UpdateG(X, F, mu, G);

		// Fixing G, updating F
		UpdateF(X, G, gamma, F);

		ind = ind + 1;
		// fprintf("Iter: %d\n", ind);
		if (ind > maxIter) {
			println("Maximal iterations");
			break;
		}

		d = norm(G.minus(*G_old), "fro");

		if (calc_OV) {
			valueList.push_back(f(X, F, G, E_F, E_G));
		}

		if (ind % 10 == 0 && verbose) {
			if (calc_OV) {
				fprintf("Iteration %d, delta G: %f, J: %f\n", ind, d, valueList.back());
				// System.out.flush();
			} else {
				fprintf("Iteration %d, delta G: %f\n", ind, d);
				// System.out.flush();
			}
		}

		if (calc_OV) {
			std::list<double>::iterator iter = valueList.end();
			if (fabs(*(--iter) - *(--iter)) < epsilon && d < epsilon) {
				println("Converge successfully!");
				break;
			}
		} else if (d < epsilon) {
			println("Converge successfully!");
			break;
		}

		if (sumAll(isnan(G)) > 0) {
			break;
		}

	}

	centers = &F;
	indicatorMatrix = &G;

}

void L1NMF::UpdateG(Matrix& X, Matrix& F, double mu, Matrix& G) {

	// min|| X - G * F ||_F^2 + mu * || G ||_1
	// s.t. G >= 0

	int MaxIter = 10000;
	double epsilon = 1e-1;

	int K = size(F, 1);
	int nExample = size(X, 1);

	Matrix& FT = F.transpose();
	Matrix& S = F.mtimes(FT);
	Matrix& C = X.mtimes(FT);
	delete &FT;
	timesAssign(C, -1);
	plusAssign(C, mu / 2);

	int nClus = size(F, 1);
	double* D = new double[nClus];
	for (int i = 0; i < nClus; i++)
		D[i] = S.getEntry(i, i);

	// double* D = ((DenseVector) diag(S).getColumnVector(0)).getPr();

	// Matrix& G = G0;
	int ind = 0;
	double d = 0;

	Matrix& G_old = *new DenseMatrix(size(G));
	Vector* GSPlusCj = new DenseVector(nExample);;
	Vector** SColumns = denseMatrix2DenseColumnVectors(S);
	Vector** CColumns = denseMatrix2DenseColumnVectors(C);
	double** GData = ((DenseMatrix&) G).getData();
	double* pr = null;
	Matrix& GDiff = G_old.copy();
	Matrix& AbsGDiff = GDiff.copy();
	while (true) {

		assign(G_old, G);

		for (int j = 0; j < K; j++) {
			// GSPlusCj = &G.operate(*SColumns[j]);
			operate(*GSPlusCj, G, *SColumns[j]);
			plusAssign(*GSPlusCj, *CColumns[j]);
			timesAssign(*GSPlusCj, 1 / D[j]);
			pr = ((DenseVector*) GSPlusCj)->getPr();
			// G(:, j) = max(G(:, j) - (G * S(:, j) + C(:, j)) / D[j]), 0);
			// G(:, j) = max(G(:, j) - GSPlusC_j, 0)
			for (int i = 0; i < nExample; i++) {
				GData[i][j] = max(GData[i][j] - pr[i], 0);
			}
			// delete GSPlusCj;
		}

		ind = ind + 1;
		if (ind > MaxIter) {
			break;
		}

		minus(GDiff, G, G_old);
		// Matrix& AbsGDiff = abs(GDiff);
		abs(AbsGDiff, GDiff);
		d = sumAll(AbsGDiff);
		// delete &AbsGDiff;
		// d = sumAll(abs(G.minus(G_old)));

		if (d < epsilon) {
			break;
		}

	}

	delete &AbsGDiff;
	delete GSPlusCj;
	delete &S;
	delete &C;
	delete[] D;
	for (int j = 0; j < K; j++) {
		delete SColumns[j];
		SColumns[j] = null;
		delete CColumns[j];
		CColumns[j] = null;
	}
	delete[] SColumns;
	SColumns = null;
	delete[] CColumns;
	CColumns = null;

	delete &G_old;
	delete &GDiff;

	// return G;

}

void L1NMF::UpdateF(Matrix& X, Matrix& G, double gamma, Matrix& F) {

	// min|| X - G * F ||_F^2 + gamma * || F ||_1
	// s.t. F >= 0

	int MaxIter = 10000;
	double epsilon = 1e-1;

	int K = size(G, 2);
	int NFea = size(X, 2);
	int nClus = size(G, 2);

	Matrix& GT = G.transpose();
	Matrix& S = GT.mtimes(G);
	Matrix& C = GT.mtimes(X);
	delete &GT;
	timesAssign(C, -1);
	plusAssign(C, gamma / 2);

	double* D = new double[nClus];
	for (int i = 0; i < nClus; i++)
		D[i] = S.getEntry(i, i);
	// double* D = ((DenseVector) diag(S).getColumnVector(0)).getPr();

	// Matrix F = F0;
	int ind = 0;
	double d = 0;
	Matrix& F_old = *new DenseMatrix(size(F));
	Vector* SFPlusCi = new DenseVector(NFea);
	Vector** SRows = denseMatrix2DenseRowVectors(S);
	Vector** CRows = denseMatrix2DenseRowVectors(C);
	double** FData = ((DenseMatrix&) F).getData();
	double* FRow = null;
	double* pr = null;
	Matrix& FDiff = F_old.copy();
	Matrix& AbsFDiff = FDiff.copy();
	while (true) {

		assign(F_old, F);

		for (int i = 0; i < K; i++) {
			// SFPlusCi = &SRows[i]->operate(F);
			operate(*SFPlusCi, *SRows[i], F);
			plusAssign(*SFPlusCi, *CRows[i]);
			timesAssign(*SFPlusCi, 1 / D[i]);
			pr = ((DenseVector*) SFPlusCi)->getPr();
			// F(i, :) = max(F(i, :) - (S(i, :) * F + C(i, :)) / D[i]), 0);
			// F(i, :) = max(F(i, :) - SFPlusCi, 0)
			FRow = FData[i];
			for (int j = 0; j < NFea; j++) {
				FRow[j] = max(FRow[j] - pr[j], 0);
			}
			// delete SFPlusCi;
		}

		ind = ind + 1;
		if (ind > MaxIter) {
			break;
		}

		minus(FDiff, F, F_old);
		// Matrix& AbsFDiff = abs(FDiff);
		abs(AbsFDiff, FDiff);
		d = sumAll(AbsFDiff);
		// delete &AbsFDiff;
		// d = sumAll(abs(F.minus(F_old)));
		if (d < epsilon) {
			break;
		}

	}

	delete &AbsFDiff;
	delete SFPlusCi;
	delete &S;
	delete &C;
	delete[] D;
	/*for (int i = 0; i < K; i++) {
		if (SRows[i] != null) {
			delete SRows[i];
			SRows[i] = null;
		}
		if (CRows[i] != null) {
			delete CRows[i];
			CRows[i] = null;
		}
	}*/
	delete[] SRows;
	SRows = null;
	delete[] CRows;
	CRows = null;

	delete &F_old;
	delete &FDiff;

	// return F;

}

double L1NMF::f(Matrix& X, Matrix& F, Matrix& G, Matrix& E_F, Matrix& E_G) {
	double fval = pow(norm(X.minus(G.mtimes(F)), "fro"), 2) +
			2 * sumAll(E_F.times(F)) +
			2 * sumAll(E_G.times(G));
	/*return Math.pow(Matlab.norm( X.minus(F.mtimes(G.transpose())), "fro"), 2)
			+ 2 * Matlab.trace( E_F.transpose().mtimes(F) )
			+ 2 * Matlab.trace( E_G.transpose().mtimes(G) );*/
	return fval;
}


