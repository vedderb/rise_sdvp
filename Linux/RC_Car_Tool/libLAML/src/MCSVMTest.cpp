/*
 * MCSVMTest.cpp
 *
 *  Created on: Feb 21, 2014
 *      Author: Mingjie Qian
 */

#include "LinearMCSVM.h"
#include <stdlib.h>
#include <limits>
#include "Classifier.h"
#include "Printer.h"
#include "DataSet.h"
#include "MyTime.h"
#include "Utility.h"

int main(void) {

	/*double max = POSITIVE_INFINITY;
	double min = NEGATIVE_INFINITY;
	fprintf("max = %.4f\n", max);
	fprintf("min = %.4f\n", min);*/

	double C = 1.0;
	double eps = 1e-4;
	Classifier* linearMCSVM = new LinearMCSVM(C, eps);

	int* pred_labels = null;
	double data[][4] = {
			{3.5, 4.4, 1.3, 2.3},
			{5.3, 2.2, 0.5, 4.5},
			{0.2, 0.3, 4.1, -3.1},
			{-1.2, 0.4, 3.2, 1.6}
	};

	int labels[] = {1, 2, 3, 4};

	linearMCSVM->feedData<4>(data, 4, 4);
	linearMCSVM->feedLabels(labels, 4);
	linearMCSVM->train();
	fprintf("W:\n");
	printMatrix(*linearMCSVM->W);
	fprintf("b:\n");
	printVector(linearMCSVM->b, 4);
	pred_labels = linearMCSVM->predict<4>(data, 4);
	Classifier::getAccuracy(pred_labels, labels, 4);

	// Get elapsed time in seconds
	tic();

	std::string trainDataFilePath = "heart_scale";
	C = 1;
	eps = 0.01;
	linearMCSVM = new LinearMCSVM(C, eps);
	DataSet& dataSet = readDataSetFromFile(trainDataFilePath);
	linearMCSVM->feedData(*dataSet.X);
	linearMCSVM->feedLabels(dataSet.Y, dataSet.nExample);
	linearMCSVM->train();

	Matrix* XTest = dataSet.X;
	pred_labels = linearMCSVM->predict(*XTest);
	int nt = XTest->getRowDimension();
	Classifier::getAccuracy(pred_labels, linearMCSVM->labels, nt);

	fprintf("Elapsed time: %.2f seconds.\n", toc());

	return EXIT_SUCCESS;

}


