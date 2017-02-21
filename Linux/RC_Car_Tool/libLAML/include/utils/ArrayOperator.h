/*
 * ArrayOperator.h
 *
 *  Created on: Feb 11, 2014
 *      Author: Mingjie Qian
 */

#ifndef ARRAYOPERATOR_H_
#define ARRAYOPERATOR_H_

#include <string>

/**
 * Allocate memory for a 2D {@code double} array.
 *
 * @param nRows number of rows
 *
 * @param nCols number of columns
 *
 * @return a nRows by nCols 2D {@code double} array
 *
 */
double** allocateMatrix(int nRows, int nCols);

/**
 * Allocate continuous memory block for a 1D {@code int}
 * array.
 *
 * @param n number of elements to be allocated
 *
 * @return a 1D {@code int} array of length n
 *
 */
int* allocateIntegerVector(int n);

/**
 * Allocate continuous memory block for a 1D {@code int}
 * array.
 *
 * @param n number of elements to be allocated
 *
 * @param v an integer to initialize the vector
 *
 * @return a 1D {@code int} array of length n
 *
 */
int* allocateIntegerVector(int n, int v);

/**
 * Assign a 1D {@code int} array by a real scalar.
 *
 * @param V a 1D {@code int} array
 *
 * @param len length of V
 *
 * @param v a real scalar
 *
 */
void assignIntegerVector(int* V, int len, int v);

/**
 * Clear all elements of a 1D {@code double} array to zero.
 *
 * @param V a {@code double} array
 *
 * @param len length of V
 *
 */
void clearVector(double* V, int len);

/**
 * Clear all elements of a 2D {@code double} array to zero.
 *
 * @param A a 2D {@code double} array
 *
 * @param M number of rows
 *
 * @param N number of columns
 *
 */
void clearMatrix(double** A, int M, int N);

/**
 * Round towards zero.
 *
 * @param x a real number
 *
 * @return fix(x)
 *
 */
int fix(double x);

/**
 * Generates a linearly spaced integer array with distance of
 * D between two consecutive numbers. colon(J, D, K) is
 * the same as [J, J+D, ..., J+m*D] where m = fix((K-J)/D).
 *
 * @param begin starting point (inclusive)
 *
 * @param d distance between two consecutive numbers
 *
 * @param end ending point (inclusive if possible)
 *
 * @return indices array for the syntax begin:d:end
 *
 */
int* colon(int begin, int d, int end);

/**
 * Same as colon(begin, 1, end).
 *
 * @param begin starting point (inclusive)
 *
 * @param end ending point (inclusive)
 *
 * @return indices array for the syntax begin:end
 *
 */
int* colon(int begin, int end);

/**
 * Generates a linearly spaced integer array with distance of
 * D between two consecutive numbers. colon(J, D, K) is
 * the same as [J, J+D, ..., J+m*D] where m = fix((K-J)/D).
 *
 * @param begin starting point (inclusive)
 *
 * @param d distance between two consecutive numbers
 *
 * @param end ending point (inclusive if possible)
 *
 * @return indices array for the syntax begin:d:end
 *
 */
double* colon(double begin, double d, double end);

/**
 * Sort a {@code double} array in a specified order.
 *
 * @param V a {@code double} array
 *
 * @param len length of V
 *
 * @param order "ascend" or "descend"
 *
 * @return original indices for the result
 */
int* sort(double* V, int len, std::string order);

/**
 * Sort a {@code double} array in an ascending order.
 *
 * @param V a {@code double} array
 *
 * @param len length of V
 *
 * @return original indices for the result
 */
int* sort(double* V, int len);

/**
 * Sort a a 1D {@code double} array in a specified order.
 *
 * @param values a 1D {@code double} array containing the values to be sort
 *
 * @param indices index array
 *
 * @param start start index (inclusive)
 *
 * @param end end index (inclusive)
 *
 * @param order a {@code String} variable either "descend" or "ascend"
 *
 */
void quickSort(double* values, int* indices, int start, int end, std::string order);

/**
 * Sort a a 1D {@code double} array in a specified order.
 *
 * @param values a 1D {@code double} array containing the values to be sort
 *
 * @param indices index array
 *
 * @param start start index (inclusive)
 *
 * @param end end index (inclusive)
 *
 * @param order a {@code String} variable either "descend" or "ascend"
 *
 */
void quickSort(double* values, double* indices, int start, int end, std::string order);




/**
 * Assign a 1D {@code double} array by a real scalar.
 *
 * @param res a 1D {@code double} array
 *
 * @param len length of res
 *
 * @param v a real scalar
 *
 */
void assign(double* res, int len, double v);

/**
 * Element-wise division and assignment operation. It divides
 * the first argument with the second argument and assign
 * the result to the first argument, i.e., V = V / v.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 *
 * @param v a real scalar
 *
 */
void divideAssign(double* V, int len, double v);

/**
 * Element-wise division and assignment operation. It divides
 * the first argument with the second argument and assign
 * the result to the first argument, i.e., V1 = V1 ./ V2.
 *
 * @param V1 a 1D {@code double} array
 *
 * @param V2 a 1D {@code double} array
 *
 * @param len length of V1 and V2
 */
void divideAssign(double* V1, double* V2, int len);

/**
 * Inner product of two vectors, i.e., <V1, V2>.
 *
 * @param V1 a 1D {@code double} array
 *
 * @param V2 a 1D {@code double} array
 *
 * @param len length of the input arrays
 *
 * @return <V1, V2>
 *
 */
double innerProduct(double* V1, double* V2, int len);

/**
 * Element-wise multiplication and assignment operation.
 * It multiplies the first argument with the second argument
 * and assign the result to the first argument, i.e., V = V * v.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of the array
 *
 * @param v a real scalar
 *
 */
void timesAssign(double* V, int len, double v);

/**
 * Compute the maximum argument.
 *
 * @param V a {@code double} array
 *
 * @param length number of elements in this array
 *
 * @return maximum argument
 *
 */
int argmax(double* V, int length);

/**
 * Assign a 1D {@code double} array by a real scalar.
 *
 * @param V a 1D {@code double} array
 *
 * @param length number of elements in this array
 *
 * @param v a real scalar
 *
 */
void assignVector(double* V, int length, double v);

/**
 * Allocate continuous memory block for a 1D {@code double}
 * array and assign all elements with a given value.
 *
 * @param n number of elements to be allocated
 *
 * @param v a real scalar to assign the 1D {@code double} array
 *
 * @return a 1D {@code double} array of length n
 *
 */
double* allocateVector(int n, double v);

/**
 * Allocate continuous memory block for a 1D {@code double}
 * array and assign all elements with a given value.
 *
 * @param n number of elements to be allocated
 *
 * @param v a real scalar to assign the 1D {@code double} array
 *
 * @return a 1D {@code double} array of length n
 *
 */
double* allocate1DArray(int n, double v);

/**
 * Allocate continuous memory block for a 1D {@code double}
 * array.
 *
 * @param n number of elements to be allocated
 *
 * @return a 1D {@code double} array of length n
 *
 */
double* allocate1DArray(int n);

/**
 * Allocate continuous memory block for a 1D {@code double}
 * array.
 *
 * @param n number of elements to be allocated
 *
 * @return a 1D {@code double} array of length n
 *
 */
double* allocateVector(int n);

/**
 * Allocate a 2D {@code double} array and assign all
 * elements with a given value.
 *
 * @param m number of rows
 *
 * @param n number of columns
 *
 * @param v a real scalar
 *
 * @return a 2D {@code double} array
 */
double** allocate2DArray(int m, int n, double v);

/**
 * Allocate a 2D {@code double} array and assign all
 * elements with zero by default.
 *
 * @param m number of rows
 *
 * @param n number of columns
 *
 * @return a 2D {@code double} array
 *
 */
double** allocate2DArray(int m, int n);

/**
 * Element-wise multiplication.
 * It multiplies the first argument with the second argument,
 * i.e., res = V1 .* V2.
 *
 * @param V1 a 1D {@code double} array
 *
 * @param V2 a 1D {@code double} array
 *
 * @param len length of V1 and V2
 *
 * @return V1 .* V2
 *
 */
double* times(double* V1, double* V2, int len);

/**
 * Element-wise addition.
 * It adds the first argument with the second argument,
 * i.e., res = V1 + V2.
 *
 * @param V1 a 1D {@code double} array
 *
 * @param V2 a 1D {@code double} array
 *
 * @param len length of V1 and V2
 *
 * @return V1 + V2
 *
 */
double* plus(double* V1, double* V2, int len);

/**
 * Element-wise subtraction.
 * It subtracts the first argument by the second argument,
 * i.e., res = V1 - V2.
 *
 * @param V1 a 1D {@code double} array
 *
 * @param V2 a 1D {@code double} array
 *
 * @param len length of V1 and V2
 *
 * @return V1 - V2
 *
 */
double* minus(double* V1, double* V2, int len);

/**
 * res = V
 * @param res
 * @param V
 * @param len length of res and V
 */
void assignVector(double* res, double* V, int len);

/**
 * Element-wise addition and assignment operation.
 * It adds the first argument by the second argument
 * and assign the result to the first argument, i.e., V = V + v.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 *
 * @param v a real scalar
 *
 */
void plusAssign(double* V, int len, double v);

/**
 * Element-wise addition and assignment operation.
 * It adds the first argument by the second argument
 * and assign the result to the first argument, i.e., V1 = V1 + V2.
 *
 * @param V1 a 1D {@code double} array
 *
 * @param V2 a 1D {@code double} array
 *
 * @param len length of V1 and V2
 *
 */
void plusAssign(double* V1, double* V2, int len);

/**
 * Element-wise subtraction and assignment operation.
 * It subtracts the first argument by the second argument
 * and assign the result to the first argument, i.e., V = V - v.
 *
 * @param V a 1D {@code int} array
 *
 * @param len length of V
 *
 * @param v an integer
 *
 */
void minusAssign(int* V, int len, int v);

/**
 * Element-wise subtraction and assignment operation.
 * It subtracts the first argument by the second argument
 * and assign the result to the first argument, i.e., V = V - v.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 *
 * @param v a real scalar
 *
 */
void minusAssign(double* V, int len, double v);

/**
 * Element-wise subtraction and assignment operation.
 * It subtracts the first argument by the second argument
 * and assign the result to the first argument, i.e., V1 = V1 - V2.
 *
 * @param V1 a 1D {@code double} array
 *
 * @param V2 a 1D {@code double} array
 *
 * @param len length of V1 and V2
 *
 */
void minusAssign(double* V1, double* V2, int len);

/**
 * Compute the sum of a 1D {@code double} array.
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 *
 * @return sum(V)
 *
 */
double sum(double* V, int len);

/**
 * Sum a 1D {@code double} array to one, i.e., V[i] = V[i] / sum(V).
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 */
void sum2one(double* V, int len);

/**
 * Element-wise multiplication and assignment operation.
 * It multiplies the first argument with the second argument
 * and assign the result to the first argument, i.e., V1 = V1 .* V2.
 *
 * @param V1 a 1D {@code double} array
 *
 * @param V2 a 1D {@code double} array
 *
 * @param len length of V1 and V2
 */
void timesAssign(double* V1, double* V2, int len);

#endif /* ARRAYOPERATOR_H_ */
