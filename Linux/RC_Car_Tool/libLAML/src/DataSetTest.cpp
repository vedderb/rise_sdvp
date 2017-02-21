/*
 * DataSetTest.cpp
 *
 *  Created on: Feb 14, 2014
 *      Author: Mingjie Qian
 */

#include "DataSet.h"
#include <stdlib.h>
#include <Printer.h>

int main(void) {

	std::string filePath = "heart_scale";
	DataSet& dataSet = readDataSetFromFile(filePath);

	disp("X:");
	printMatrix(*dataSet.X);
	disp("Y:");
	disp(dataSet.Y, dataSet.X->getRowDimension());

	return EXIT_SUCCESS;

}


