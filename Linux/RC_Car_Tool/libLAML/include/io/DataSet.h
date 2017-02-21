/*
 * DataSet.h
 *
 *  Created on: Feb 14, 2014
 *      Author: Mingjie Qian
 */

#ifndef DATASET_H_
#define DATASET_H_

#include "Matrix.h"
#include <list>
#include <string>

class DataSet {
public:
	Matrix* X;
	int* Y;
	int nExample;
	DataSet();
	DataSet(Matrix* X, int* Y);
};

/**
 * Read a data set from a string array.
 *
 * @param feaArray a {@code ArrayList<String>}, each element
 *                 is a string with LIBSVM data format
 *
 * @return a {@code DataSet} instance
 *
 * @throws InvalidInputDataException
 *
 */
DataSet& readDataSetFromStringList(std::list<std::string>& feaList);

/**
 * Read a data set from a LIBSVM formatted file.
 * Data format (index starts from 1):<p/>
 * <pre>
 * +1 1:0.708333 2:1 3:1 4:-0.320755 5:-0.105023 6:-1 7:1 8:-0.419847 9:-1 10:-0.225806 12:1 13:-1
 * -1 1:0.583333 2:-1 3:0.333333 4:-0.603774 5:1 6:-1 7:1 8:0.358779 9:-1 10:-0.483871 12:-1 13:1
 * </pre>
 * @param filePath file path
 *
 * @return a {@code DataSet} instance
 *
 * @throws IOException
 *
 * @throws InvalidInputDataException
 *
 */
DataSet& readDataSetFromFile(const std::string filePath);

#endif /* DATASET_H_ */
