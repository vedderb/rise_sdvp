/*
 * IO.h
 *
 *  Created on: Feb 14, 2014
 *      Author: Mingjie Qian
 */

#ifndef IO_H_
#define IO_H_

#include "Matrix.h"
#include <string>

/**
 * Write a matrix into a text file.
 *
 * @param A a real matrix
 *
 * @param filePath file path to write a matrix into
 *
 */
void saveMatrix(Matrix& A, const std::string& filePath);

/**
 * Write a matrix into a text file.
 *
 * @param filePath file path to write a matrix into
 *
 * @param A a real matrix
 *
 */
void saveMatrix(const std::string& filePath, Matrix& A);

/**
 * Write a dense matrix into a text file. Each line corresponds
 * to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param A a dense matrix
 *
 * @param filePath file path to write a dense matrix into
 *
 */
void saveDenseMatrix(Matrix& A, const std::string& filePath);

/**
 * Write a dense matrix into a text file. Each line corresponds
 * to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param filePath file path to write a dense matrix into
 *
 * @param A a dense matrix
 *
 */
void saveDenseMatrix(const std::string& filePath, Matrix& A);

/**
 * Write a sparse matrix into a text file. Each line
 * corresponds to a non-zero entry with the format "%d %d %.8g".
 *
 * @param A a sparse matrix
 *
 * @param filePath file path to write a sparse matrix into
 *
 */
void saveSparseMatrix(Matrix& A, const std::string& filePath);

/**
 * Write a sparse matrix into a text file. Each line
 * corresponds to a non-zero entry with the format "%d %d %.8g".
 *
 * @param A a sparse matrix
 *
 * @param filePath file path to write a sparse matrix into
 *
 */
void saveSparseMatrix(const std::string& filePath, Matrix& A);

/**
 * Read a matrix from a text file. Sparseness will be automatically detected.
 *
 * @param filePath file path to read a matrix from
 *
 * @return a real matrix
 *
 */
Matrix& loadMatrix(const std::string& filePath);

/**
 * Read a dense matrix from a text file. Each line
 * corresponds to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param filePath file path to read a dense matrix from
 *
 * @return a dense matrix
 *
 */
Matrix& loadDenseMatrix(const std::string& filePath);

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
Matrix& loadSparseMatrix(const std::string& filePath);

/**
 * Write a matrix into a text file.
 *
 * @param A a real matrix
 *
 * @param filePath file path to write a matrix into
 *

void saveMatrix(Matrix& A, std::string& filePath);

*
 * Write a matrix into a text file.
 *
 * @param filePath file path to write a matrix into
 *
 * @param A a real matrix
 *

void saveMatrix(std::string& filePath, Matrix& A);

*
 * Write a dense matrix into a text file. Each line corresponds
 * to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param A a dense matrix
 *
 * @param filePath file path to write a dense matrix into
 *

void saveDenseMatrix(Matrix& A, std::string& filePath);

*
 * Write a dense matrix into a text file. Each line corresponds
 * to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param filePath file path to write a dense matrix into
 *
 * @param A a dense matrix
 *

void saveDenseMatrix(std::string& filePath, Matrix& A);

*
 * Write a sparse matrix into a text file. Each line
 * corresponds to a non-zero entry with the format "%d %d %.8g".
 *
 * @param A a sparse matrix
 *
 * @param filePath file path to write a sparse matrix into
 *

void saveSparseMatrix(Matrix& A, std::string& filePath);

*
 * Write a sparse matrix into a text file. Each line
 * corresponds to a non-zero entry with the format "%d %d %.8g".
 *
 * @param A a sparse matrix
 *
 * @param filePath file path to write a sparse matrix into
 *

void saveSparseMatrix(std::string& filePath, Matrix& A);

*
 * Read a matrix from a text file. Sparseness will be automatically detected.
 *
 * @param filePath file path to read a matrix from
 *
 * @return a real matrix
 *

Matrix& loadMatrix(std::string& filePath);

*
 * Read a dense matrix from a text file. Each line
 * corresponds to a row with the format "%.8g\t%.8g\t%.8g\t... \t%.8g".
 *
 * @param filePath file path to read a dense matrix from
 *
 * @return a dense matrix
 *

Matrix& loadDenseMatrix(std::string& filePath);

*
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

Matrix& loadSparseMatrix(std::string& filePath);*/

#endif /* IO_H_ */
