/*
 * IO.cpp
 *
 *  Created on: Feb 14, 2014
 *      Author: Mingjie Qian
 */

#ifndef IO_CPP_
#define IO_CPP_

#include "IO.h"
#include <fstream>
#include <string>
#include <typeinfo>
#include <list>
#include "Utility.h"
#include "Printer.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"

/**
 * Write a matrix into a text file.
 *
 * @param A a real matrix
 *
 * @param filePath file path to write a matrix into
 *
 */
void saveMatrix(Matrix& A, const std::string& filePath) {
	if (typeid(A) == typeid(DenseMatrix)) {
		saveDenseMatrix(A, filePath);
	} else {
		saveSparseMatrix(A, filePath);
	}
}

/**
 * Write a matrix into a text file.
 *
 * @param filePath file path to write a matrix into
 *
 * @param A a real matrix
 *
 */
void saveMatrix(const std::string& filePath, Matrix& A) {
	if (typeid(A) == typeid(DenseMatrix)) {
		saveDenseMatrix(A, filePath);
	} else {
		saveSparseMatrix(A, filePath);
	}
}

/**
 * Write a dense matrix into a text file. Each line corresponds
 * to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param A a dense matrix
 *
 * @param filePath file path to write a dense matrix into
 *
 */
void saveDenseMatrix(Matrix& A, const std::string& filePath) {
	std::ofstream matfile;
	matfile.open(filePath.c_str());
	if (matfile.is_open()) {
		int nRow = A.getRowDimension();
		int nCol = A.getColumnDimension();
		double** data = ((DenseMatrix&) A).getData();
		double* rowData = null;
		for (int i = 0; i < nRow; i++) {
			rowData = data[i];
			for (int j = 0; j < nCol; j++) {
				matfile << sprintf("%.8g\t", rowData[j]);
			}
			matfile << std::endl;
		}
		println("Data matrix file written: " + filePath + "\n");
		matfile.close();
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

/**
 * Write a dense matrix into a text file. Each line corresponds
 * to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param filePath file path to write a dense matrix into
 *
 * @param A a dense matrix
 *
 */
void saveDenseMatrix(const std::string& filePath, Matrix& A) {
	saveDenseMatrix(A, filePath);
}

/**
 * Write a sparse matrix into a text file. Each line
 * corresponds to a non-zero entry with the format "%d %d %.8g".
 *
 * @param A a sparse matrix
 *
 * @param filePath file path to write a sparse matrix into
 *
 */
void saveSparseMatrix(Matrix& A, const std::string& filePath) {
	std::ofstream matfile;
	matfile.open(filePath.c_str());
	if (matfile.is_open()) {
		int nRow = A.getRowDimension();
		int nCol = A.getColumnDimension();
		matfile << sprintf("numRows: %d\n", nRow);
		matfile << sprintf("numColumns: %d\n", nCol);
		int* ir = ((SparseMatrix&) A).getIr();
		int* jc = ((SparseMatrix&) A).getJc();
		double* pr = ((SparseMatrix&) A).getPr();
		int rIdx = -1;
		int cIdx = -1;
		double value = 0;
		for (int j = 0; j < nCol; j++) {
			cIdx = j + 1;
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				rIdx = ir[k] + 1;
				value = pr[k];
				if (value != 0) {
					matfile << sprintf("%d %d %.8g\n", rIdx, cIdx, value);
				}
			}
		}
		println("Data matrix file written: " + filePath + "\n");
		matfile.close();
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

/**
 * Write a sparse matrix into a text file. Each line
 * corresponds to a non-zero entry with the format "%d %d %.8g".
 *
 * @param A a sparse matrix
 *
 * @param filePath file path to write a sparse matrix into
 *
 */
void saveSparseMatrix(const std::string& filePath, Matrix& A) {
	saveSparseMatrix(A, filePath);
}

/**
 * Read a matrix from a text file. Sparseness will be automatically detected.
 *
 * @param filePath file path to read a matrix from
 *
 * @return a real matrix
 *
 */
Matrix& loadMatrix(const std::string& filePath) {
	Matrix* res = NULL;
	std::ifstream matfile;
	matfile.open(filePath.c_str());
	std::string line = "";
	if (matfile.is_open()) {
		int ind = 0;
		bool isSparseMatrix = false;
		while (getline(matfile, line)) {
			if (line.find('#') != std::string::npos)
				continue;
			if (line.find("numRows:") != std::string::npos) {
				isSparseMatrix = true;
				break;
			} else if (line.find("numColumns:") != std::string::npos) {
				isSparseMatrix = true;
				break;
			}
			ind++;
			if (ind == 2)
				break;
		}
		matfile.close();
		if (isSparseMatrix) {
			res = &loadSparseMatrix(filePath);
		} else {
			res = &loadDenseMatrix(filePath);
		}
		return *res;
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

/**
 * Read a dense matrix from a text file. Each line
 * corresponds to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param filePath file path to read a dense matrix from
 *
 * @return a dense matrix
 *
 */
Matrix& loadDenseMatrix(const std::string& filePath) {
	Matrix* res = NULL;
	std::ifstream matfile;
	matfile.open(filePath.c_str());
	std::string line = "";
	std::list<double*> list;
	int numColumns = 0;
	int numRows = 0;
	std::string delim = " \t\n\r";
	if (matfile.is_open()) {
		while (getline(matfile, line)) {
			if (line.find('#') != std::string::npos)
				continue;
			StringTokenizer* tokenizer = new StringTokenizer(line, delim);
			numColumns = tokenizer->countTokens();
			double* row = new double[numColumns];
			for (int j = 0; j < numColumns; j++) {
				row[j] = atof(tokenizer->nextToken());
			}
			list.push_back(row);
			delete tokenizer;
		}
		matfile.close();
		numRows = list.size();
		double** data = new double*[numRows];
		for (int i = 0; i < numRows; i++) {
			data[i] = list.front();
			list.pop_front();
		}
		res = new DenseMatrix(data, numRows, numColumns);
		// println("Data matrix loaded from file " + filePath + "\n");
		return *res;
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

/**
 * Load a {@code Matrix} from a text file.
 *
 * @param filePath
 *        a {@code std::string&} specifying the location of the text file holding matrix data.
 *        Each line is an entry with the format (without double quotes)
 *        "(rowIdx,[whitespace]colIdx):[whitespace]value". rowIdx and colIdx
 *        start from 1 as in MATLAB.
 *
 * @return a sparse matrix
 *
 */
Matrix& loadSparseMatrix(const std::string& filePath) {
	Matrix* res = NULL;
	std::ifstream matfile;
	matfile.open(filePath.c_str());
	std::string line = "";
	std::list<int> rIndexList;
	std::list<int> cIndexList;
	std::list<double> valueList;
	int numColumns = 0;
	int numRows = 0;
	int rIdx = 0;
	int cIdx = 0;
	int nzmax = 0;
	double value = 0;
	std::string delim = " \t\n\r";
	if (matfile.is_open()) {
		int ind = 0;
		while (getline(matfile, line)) {
			// std::cout << line << std::endl << std::flush;
			if (line.find('#') != std::string::npos)
				continue;
			if (line.find("numRows:") != std::string::npos) {
				/*int a = 0;
				a = a + 1;
				std::cout << line << std::endl;
				size_t pos = line.find("numRows:");
				std::cout << pos << std::endl;
				std::string substring = line.substr(pos + 8);
				std::cout << line.substr(pos + 8) << std::endl;*/
				numRows = atoi(line.substr(line.find("numRows:") + 8));
				ind++;
			} else if (line.find("numColumns:") != std::string::npos) {
				/*int a = 0;
				a = a + 1;
				std::cout << line << std::endl;
				size_t pos = line.find("numColumns:");
				std::cout << pos << std::endl;
				std::string substring = line.substr(pos + 11);
				std::cout << line.substr(pos + 11) << std::endl;*/
				numColumns = atoi(line.substr(line.find("numColumns:") + 11));
				ind++;
			}
			if (ind == 2)
				break;
		}
		while (getline(matfile, line)) {
			StringTokenizer* tokenizer = new StringTokenizer(line, delim);
			rIdx = atoi(tokenizer->nextToken()) - 1;
			cIdx = atoi(tokenizer->nextToken()) - 1;
			value = atof(tokenizer->nextToken());
			delete tokenizer;
			if (value == 0)
				continue;
			rIndexList.push_back(rIdx);
			cIndexList.push_back(cIdx);
			valueList.push_back(value);
			nzmax++;
		}
		matfile.close();
		res = new SparseMatrix(rIndexList, cIndexList, valueList, numRows, numColumns, nzmax);
		// println("Data matrix loaded from file " + filePath + "\n");
		return *res;
	} else {
		err("Cannot open file " + filePath + "\n");
		exit(1);
	}
}

#endif /* IO_CPP_ */
