/*
 * DataSet.cpp
 *
 *  Created on: Feb 14, 2014
 *      Author: Mingjie Qian
 */

#include "DataSet.h"
#include "Utility.h"
#include "SparseMatrix.h"
#include "Printer.h"
#include <list>
#include <map>
#include <fstream>
#include <string>

DataSet::DataSet(Matrix* X, int* Y) {
	this->X = X;
	this->Y = Y;
	nExample = X->getRowDimension();
}

DataSet& readDataSetFromStringList(std::list<std::string*>& feaList) {
	DataSet* dataSet = NULL;
	size_t numRows = feaList.size();
	int* Y = new int[numRows];
	std::string* line = NULL;
	int max_index = 0;
	std::map<std::pair<int, int>, double> map;
	int exampleIndex = 0;
	int featureIndex = -1;
	double value = 0;
	int nzmax = 0;
	std::string delim = " \t";
	std::string label = "";
	size_t pos = 0;
	std::string idxString = "";
	std::string valString = "";
	std::string token = "";
	for (std::list<std::string*>::iterator iter = feaList.begin(); iter != feaList.end(); iter++) {
		line = *iter;
		StringTokenizer* tokenizer = new StringTokenizer(*line, delim);
		token = tokenizer->nextToken();
		pos = token.find_first_of(':');
		if (pos == std::string::npos) {
			label = token;
			Y[exampleIndex] = atoi(label);
		} else {
			Y[exampleIndex] = 0;
			idxString = token.substr(0, pos);
			valString = token.substr(pos + 1);
			featureIndex = atoi(idxString) - 1;
			value = atof(valString);
			max_index = max(max_index, featureIndex);
			if (value != 0) {
				map.insert(std::make_pair(std::make_pair(featureIndex, exampleIndex), value));
				nzmax++;
			}
		}
		size_t cnt = tokenizer->countTokens();
		for (size_t i = 0; i < cnt; i++) {
			token = tokenizer->nextToken();
			pos = token.find_first_of(':');
			idxString = token.substr(0, pos);
			valString = token.substr(pos + 1);
			featureIndex = atoi(idxString) - 1;
			value = atof(valString);
			max_index = max(max_index, featureIndex);
			if (value != 0) {
				map.insert(std::make_pair(std::make_pair(featureIndex, exampleIndex), value));
				nzmax++;
			}
		}
		delete tokenizer;
		exampleIndex++;
	}
	int numColumns = max_index + 1;
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
		rIdx = iter->first.second;
		cIdx = iter->first.first;
		pr[k] = iter->second;
		ir[k] = rIdx;
		while (currentColumn < cIdx) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		k++;
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}

	Matrix* X = &SparseMatrix::createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);
	dataSet = new DataSet(X, Y);
	return *dataSet;
}

DataSet& readDataSetFromFile(const std::string filePath) {
	DataSet* dataSet = NULL;
	std::list<int> labelList;
	std::string line = "";
	int max_index = 0;
	std::map<std::pair<int, int>, double> map;
	int exampleIndex = 0;
	int featureIndex = -1;
	double value = 0;
	int nzmax = 0;
	std::string delim = " \t";
	std::string label = "";
	size_t pos = 0;
	std::string idxString = "";
	std::string valString = "";
	std::string token = "";
	std::ifstream dataFile;
	dataFile.open(filePath.c_str());
	if (!dataFile.is_open()) {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
	while (getline(dataFile, line)) {
		StringTokenizer* tokenizer = new StringTokenizer(line, delim);
		token = tokenizer->nextToken();
		pos = token.find_first_of(':');
		if (pos == std::string::npos) {
			label = token;
			labelList.push_back(atoi(label));
		} else {
			labelList.push_back(0);
			idxString = token.substr(0, pos);
			valString = token.substr(pos + 1);
			featureIndex = atoi(idxString) - 1;
			value = atof(valString);
			max_index = max(max_index, featureIndex);
			if (value != 0) {
				map.insert(std::make_pair(std::make_pair(featureIndex, exampleIndex), value));
				nzmax++;
			}
		}
		size_t cnt = tokenizer->countTokens();
		for (size_t i = 0; i < cnt; i++) {
			token = tokenizer->nextToken();
			pos = token.find_first_of(':');
			idxString = token.substr(0, pos);
			valString = token.substr(pos + 1);
			featureIndex = atoi(idxString) - 1;
			value = atof(valString);
			max_index = max(max_index, featureIndex);
			if (value != 0) {
				map.insert(std::make_pair(std::make_pair(featureIndex, exampleIndex), value));
				nzmax++;
			}
		}
		delete tokenizer;
		exampleIndex++;
	}
	dataFile.close();
	int numRows = exampleIndex;
	int numColumns = max_index + 1;
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
		rIdx = iter->first.second;
		cIdx = iter->first.first;
		pr[k] = iter->second;
		ir[k] = rIdx;
		while (currentColumn < cIdx) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		k++;
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}

	Matrix* X = &SparseMatrix::createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);
	int* Y = new int[numRows];
	for (int i = 0; i < numRows; i++) {
		Y[i] = labelList.front();
		labelList.pop_front();
	}
	dataSet = new DataSet(X, Y);
	return *dataSet;
}
