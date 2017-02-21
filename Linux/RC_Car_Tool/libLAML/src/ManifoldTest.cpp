/*
 * ManifoldTest.cpp
 *
 *  Created on: Mar 1, 2014
 *      Author: Mingjie Qian
 */

#include "Manifold.h"
#include <string>
#include "IO.h"
#include <cmath>
#include "Printer.h"
#include "MyTime.h"
#include "Matlab.h"
#include "Options.h"
#include <cstdlib>

int main(void) {

	std::string filePath = "CNN-DocTermCountMatrix.txt";
	Matrix& X_Ori = loadMatrix(filePath);
	int NSample = min(20, X_Ori.getRowDimension());
	Matrix& X = X_Ori.getSubMatrix(0, NSample - 1, 0, X_Ori.getColumnDimension() - 1);
	// disp(X.getSubMatrix(0, 10, 0, 100));
	println(sprintf("%d samples loaded", X.getRowDimension()));
	GraphOptions& options = *new GraphOptions();
	options.graphType = "nn";
	std::string type = options.graphType;
	double NN = options.graphParam;
	fprintf("Graph type: %s with NN: %d\n", type.c_str(), (int)NN);

	// Parameter setting for text data
	options.kernelType = "cosine";
	options.graphDistanceFunction = "cosine";

	// Parameter setting for image data
	/*options.kernelType = "rbf";
			options.graphDistanceFunction = "euclidean";*/

	options.graphNormalize = true;
	options.graphWeightType = "heat";

	bool show = true && !false;

	// Test adjacency function - pass
	tic();
	std::string DISTANCEFUNCTION = options.graphDistanceFunction;
	Matrix& A = adjacency(X, type, NN, DISTANCEFUNCTION);
	fprintf("Elapsed time: %.2f seconds.\n", toc());
	std::string adjacencyFilePath = "adjacency.txt";
	saveMatrix(adjacencyFilePath, A);
	if (show)
		disp(A.getSubMatrix(0, 4, 0, 4));

	// Test laplacian function - pass
	tic();
	Matrix& L = laplacian(X, type, options);
	fprintf("Elapsed time: %.2f seconds.\n", toc());
	std::string LaplacianFilePath = "Laplacian.txt";
	saveMatrix(LaplacianFilePath, L);
	if (show)
		disp(L.getSubMatrix(0, 4, 0, 4));

	// Test local learning regularization - pass
	NN = options.graphParam;
	std::string DISTFUNC = options.graphDistanceFunction;
	std::string KernelType = options.kernelType;
	double KernelParam = options.kernelParam;
	double lambda = 0.001;
	tic();
	Matrix& LLR_text = calcLLR(X, NN, DISTFUNC, KernelType, KernelParam, lambda);
	fprintf("Elapsed time: %.2f seconds.\n", toc());
	std::string LLRFilePath = "localLearningRegularization.txt";
	saveMatrix(LLRFilePath, LLR_text);
	if (show)
		display(LLR_text.getSubMatrix(0, 4, 0, 4));

	return EXIT_SUCCESS;

}
