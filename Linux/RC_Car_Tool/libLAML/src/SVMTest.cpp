/*
 * SVMTest.cpp
 *
 *  Created on: Feb 16, 2014
 *      Author: Mingjie Qian
 */

#include "LinearBinarySVM.h"
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
	Classifier* linearBinarySVM = new LinearBinarySVM(C, eps);

	int* pred_labels = null;
	double data[][4] = {
			{3.5, 4.4, 1.3, 2.3},
			{5.3, 2.2, 0.5, 4.5},
			{0.2, 0.3, 4.1, -3.1},
			{-1.2, 0.4, 3.2, 1.6}
	};

	int labels[] = {1, 1, -1, -1};

	linearBinarySVM->feedData<4>(data, 4, 4);
	linearBinarySVM->feedLabels(labels, 4);
	linearBinarySVM->train();
	fprintf("W:\n");
	printMatrix(*linearBinarySVM->W);
	fprintf("b:\n");
	printVector(linearBinarySVM->b, 1);
	pred_labels = linearBinarySVM->predict<4>(data, 4);
	Classifier::getAccuracy(pred_labels, labels, 4);

	// Get elapsed time in seconds
	tic();

	std::string trainDataFilePath = "heart_scale";
	C = 1;
	eps = 0.01;
	linearBinarySVM = new LinearBinarySVM(C, eps);
	DataSet& dataSet = readDataSetFromFile(trainDataFilePath);
	linearBinarySVM->feedData(*dataSet.X);
	linearBinarySVM->feedLabels(dataSet.Y, dataSet.nExample);
	linearBinarySVM->train();

	Matrix* XTest = dataSet.X;
	pred_labels = linearBinarySVM->predict(*XTest);
	int nt = XTest->getRowDimension();
	Classifier::getAccuracy(pred_labels, linearBinarySVM->labels, nt);

	fprintf("Elapsed time: %.2f seconds.\n", toc());

	return EXIT_SUCCESS;

}


