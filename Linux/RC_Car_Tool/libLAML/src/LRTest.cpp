/*
 * LRTest.cpp
 *
 *  Created on: Feb 22, 2014
 *      Author: Mingjie Qian
 */

#include "LogisticRegression.h"
#include <stdlib.h>
#include <limits>
#include "Classifier.h"
#include "Printer.h"
#include "DataSet.h"
#include "MyTime.h"
#include "Utility.h"

int main(void) {

	double data[][4] = {
			{3.5, 5.3, 0.2, -1.2},
			{4.4, 2.2, 0.3, 0.4},
			{1.3, 0.5, 4.1, 3.2}
	};

	int labels[] = {1, 2, 3};

	Classifier* logReg = new LogisticRegression();
	logReg->epsilon = 1e-5;
	logReg->feedData<4>(data, 3, 4);
	logReg->feedLabels(labels, 3);

	// Get elapsed time in seconds
	tic();
	logReg->train();
	fprintf("Elapsed time: %.3f seconds.\n", toc());

	fprintf("W:\n");
	printMatrix(*logReg->W);
	fprintf("b:\n");
	printVector(logReg->b, 3);

	// double** dataTest = data;
	double dataTest[][4] = {
			{3.5, 5.3, 0.2, -1.2},
			{4.4, 2.2, 0.3, 0.4},
			{1.3, 0.5, 4.1, 3.2}
	};

	fprintf("Ground truth:\n");
	printMatrix(*logReg->Y);
	fprintf("Predicted probability matrix:\n");
	Matrix& Prob_pred = logReg->predictLabelScoreMatrix<4>(dataTest, 3, 4);
	disp(Prob_pred);
	fprintf("Predicted label matrix:\n");
	Matrix& Y_pred = logReg->predictLabelMatrix<4>(dataTest, 3, 4);
	printMatrix(Y_pred);
	int* pred_labels = logReg->predict(dataTest, 3, 4);
	Classifier::getAccuracy(pred_labels, labels, 3);

	tic();
	std::string trainDataFilePath = "heart_scale";
	logReg = new LogisticRegression();
	DataSet& dataSet = readDataSetFromFile(trainDataFilePath);
	logReg->feedData(*dataSet.X);
	logReg->feedLabels(dataSet.Y, dataSet.nExample);
	logReg->train();

	Matrix& XTest = *dataSet.X;
	pred_labels = logReg->predict(XTest);
	int nt = XTest.getRowDimension();
	Classifier::getAccuracy(pred_labels, logReg->labels, nt);

	fprintf("Elapsed time: %.3f seconds.\n", toc());

	return EXIT_SUCCESS;

}


