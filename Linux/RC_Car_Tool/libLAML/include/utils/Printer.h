/*
 * Printer.h
 *
 *  Created on: Feb 8, 2014
 *      Author: Aaron
 */

#ifndef PRINTER_H_
#define PRINTER_H_

#include "Matrix.h"
#include "Vector.h"
#include <cstdlib>
#include <string>
// #include <iostream>

class Printer {

public:

	/**
	 * Print a matrix with specified precision. A sparse matrix
	 * will be printed like a dense matrix except that zero entries
	 * will be left blank.
	 *
	 * @param A a dense or sparse matrix
	 *
	 * @param p number of digits after decimal point with rounding
	 */
	static void printMatrix(Matrix& A, int p);

	/**
	 * Print a matrix. A sparse matrix will be printed like a dense
	 * matrix except that zero entries will be left blank.
	 *
	 * @param A a dense or sparse matrix
	 *
	 */
	static void printMatrix(Matrix& A) {
		printMatrix(A, 4);
	}

};

std::string int2str(int number);

void print(std::string input);

void println(void);

void println(std::string input);

void printf(const std::string fmt, ...);

void fprintf(const std::string fmt, ...);

std::string sprintf(const std::string fmt, ...);

void disp(std::string input);

/**
 * Print the error information in standard output.
 *
 * @param input a {@code String} representing the error
 */
void err(std::string input);

void errf(const std::string fmt, ...);

/**
 * Print a matrix with specified precision. A sparse matrix
 * will be printed like a dense matrix except that zero entries
 * will be left blank.
 *
 * @param A a dense or sparse matrix
 *
 * @param p number of digits after decimal point with rounding
 */
void printMatrix(Matrix& A, int p);

/**
 * Print a matrix. A sparse matrix will be printed like a dense
 * matrix except that zero entries will be left blank.
 *
 * @param A a dense or sparse matrix
 *
 */
void printMatrix(Matrix& A);

/**
 * Print a dense matrix with specified precision.
 *
 * @param A a dense matrix
 *
 * @param p number of digits after decimal point with rounding
 */
void printDenseMatrix(Matrix& A, int p);

/**
 * Print a dense matrix.
 *
 * @param A a dense matrix
 */
void printDenseMatrix(Matrix& A);

/**
 * Print a sparse matrix with specified precision.
 *
 * @param A a sparse matrix
 *
 * @param p number of digits after decimal point with rounding
 */
void printSparseMatrix(Matrix& A, int p);

/**
 * Print a sparse matrix.
 *
 * @param A a sparse matrix
 */
void printSparseMatrix(Matrix& A);

/**
 * Print a dense vector with specified precision.
 *
 * @param V a dense vector
 *
 * @param p number of digits after decimal point with rounding
 *
 */
void printDenseVector(Vector& V, int p);

/**
 * Print a dense vector.
 *
 * @param V a dense vector.
 */
void printDenseVector(Vector& V);

/**
 * Print a sparse vector with specified precision.
 *
 * @param V a sparse vector
 *
 * @param p number of digits after decimal point with rounding
 *
 */
void printSparseVector(Vector& V, int p);

/**
 * Print a sparse vector.
 *
 * @param V a sparse vector
 */
void printSparseVector(Vector& V);

/**
 * Print a row vector with specified precision.
 *
 * @param V a dense or sparse vector
 *
 * @param p number of digits after decimal point with rounding
 *
 */
void printVector(Vector& V, int p);

/**
 * Print a row vector.
 *
 * @param V a dense or sparse vector
 */
void printVector(Vector& V);

/**
 * Print a row vector with specified precision.
 *
 * @param V a 1D {@code double} array
 *
 * @param dim dimensionality
 *
 * @param p number of digits after decimal point with rounding
 */
void printVector(double* V, int dim, int p);

/**
 * Print a row vector.
 *
 * @param V a 1D {@code double} array
 *
 * @param dim dimensionality
 */
void printVector(double* V, int dim);

/**
 * Display a matrix with specified precision.
 *
 * @param A a dense or sparse matrix
 *
 * @param p number of digits after decimal point with rounding
 */
void display(Matrix& A, int p);

/**
 * Display a matrix.
 *
 * @param A a dense or sparse matrix
 */
void display(Matrix& A);

/**
 * Display a vector with specified precision.
 *
 * @param V a dense or sparse vector
 *
 * @param p number of digits after decimal point with rounding
 */
void display(Vector& V, int p);

/**
 * Display a vector.
 *
 * @param V a dense or sparse vector
 */
void display(Vector& V);

/**
 * Display a 1D {@code double} array with specified precision.
 *
 * @param V a 1D {@code double} array
 *
 * @param dim dimensionality
 *
 * @param p number of digits after decimal point with rounding
 */
void display(double* V, int dim, int p);

/**
 * Display a 1D {@code double} array.
 *
 * @param V a 1D {@code double} array
 *
 * @param dim dimensionality
 */
void display(double* V, int dim);

/**
 * Display a matrix with specified precision.
 *
 * @param A a dense or sparse matrix
 *
 * @param p number of digits after decimal point with rounding
 */
void disp(Matrix& A, int p);

/**
 * Display a matrix.
 *
 * @param A a dense or sparse matrix
 */
void disp(Matrix& A);

/**
 * Display a vector with specified precision.
 *
 * @param V a dense or sparse vector
 *
 * @param p number of digits after decimal point with rounding
 */
void disp(Vector& V, int p);

/**
 * Display a vector.
 *
 * @param V a dense or sparse vector
 */
void disp(Vector& V);

/**
 * Display a 1D {@code double} array with specified precision.
 *
 * @param V a 1D {@code double} array
 *
 * @param p number of digits after decimal point with rounding
 */
void disp(double* V, int dim, int p);

/**
 * Display a 1D {@code double} array.
 *
 * @param V a 1D {@code double} array
 */
void disp(double* V, int dim);

/**
 * Display a 1D integer array.
 *
 * @param V a 1D integer array
 *
 * @param len length of V
 */
void disp(int* V, int len);

#endif /* PRINTER_H_ */
