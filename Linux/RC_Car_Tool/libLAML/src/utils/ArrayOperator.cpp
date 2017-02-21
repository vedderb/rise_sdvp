/*
 * ArrayOperator.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: Mingjie Qian
 */

#include "ArrayOperator.h"
#include <cmath>
#include "Printer.h"

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
double** allocateMatrix(int nRows, int nCols) {
	double** res = new double*[nRows];
	for (int i = 0; i < nRows; i++) {
		res[i] = allocateVector(nCols);
	}
	return res;
}

/**
 * Allocate continuous memory block for a 1D {@code int}
 * array.
 *
 * @param n number of elements to be allocated
 *
 * @return a 1D {@code int} array of length n
 *
 */
int* allocateIntegerVector(int n) {
	return allocateIntegerVector(n, 0);
}

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
int* allocateIntegerVector(int n, int v) {
	int* res = new int[n];
	assignIntegerVector(res, n, v);
	return res;
}

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
void assignIntegerVector(int* V, int len, int v) {
	for (int i = 0; i < len; i++)
		V[i] = v;
}

/**
 * Clear all elements of a 1D {@code double} array to zero.
 *
 * @param V a {@code double} array
 *
 * @param len length of V
 *
 */
void clearVector(double* V, int len) {
	assignVector(V, len, 0);
}

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
void clearMatrix(double** A, int M, int N) {
	for (int i = 0; i < M; i++) {
		assignVector(A[i], N, 0);
	}
}

/**
 * Round towards zero.
 *
 * @param x a real number
 *
 * @return fix(x)
 *
 */
int fix(double x) {

	if (x > 0) {
		return (int) floor(x);
	} else {
		return (int) ceil(x);
	}

}

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
int* colon(int begin, int d, int end) {

	int m = fix((end - begin) / d);
	if (m < 0) {
		err("Difference error!");
		exit(1);
	}

	int* res = new int[m + 1];

	for (int i = 0; i <= m; i++) {
		res[i] = begin + i * d;
	}

	return res;

}

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
int* colon(int begin, int end) {
	return colon(begin, 1, end);
}

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
double* colon(double begin, double d, double end) {

	int m = fix((end - begin) / d);
	if (m < 0) {
		err("Difference error!");
		exit(1);
	}

	double* res = new double[m + 1];

	for (int i = 0; i <= m; i++) {
		res[i] = begin + i * d;
	}

	return res;

}

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
int* sort(double* V, int len, std::string order) {
	// int len = V.length;
	int* indices = colon(0, 1, len - 1);
	int start = 0;
	int end = len - 1;
	quickSort(V, indices, start, end, order);
	return indices;
}

/**
 * Sort a {@code double} array in an ascending order.
 *
 * @param V a {@code double} array
 *
 * @param len length of V
 *
 * @return original indices for the result
 */
int* sort(double* V, int len) {
	return sort(V, len, "ascend");
}

/**
 * Sort a 1D {@code double} array in a specified order.
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
void quickSort(double* values, int* indices, int start, int end, std::string order) {

	int	i,j;
	double temp;
	i = start;
	j = end;
	temp = values[i];
	int tempV = indices[i];
	do{
		if (order == "ascend") {
			while(((values[j]) > (temp)) && (j > i))
				j--;
		} else if (order == "descend") {
			while(((values[j]) < (temp)) && (j > i))
				j--;
		}
		if(j > i){
			values[i] = values[j];
			indices[i] = indices[j];
			i++;
		}
		if (order == "ascend") {
			while(((values[i]) < (temp)) && (j > i))
				i++;
		} else if (order == "descend") {
			while(((values[i]) > (temp)) && (j > i))
				i++;
		}
		if(j > i){
			values[j] = values[i];
			indices[j] = indices[i];
			j--;
		}
	} while(i != j);
	values[i] = temp;
	indices[i] = tempV;
	i++;
	j--;
	if(start < j)
		quickSort(values, indices, start, j, order);
	if(i < end)
		quickSort(values, indices, i, end, order);

}

/**
 * Sort a 1D {@code double} array in a specified order.
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
void quickSort(double* values, double* indices, int start, int end, std::string order) {

	int	i,j;
	double temp;
	i = start;
	j = end;
	temp = values[i];
	double tempV = indices[i];
	do{
		if (order == "ascend") {
			while(((values[j]) > (temp)) && (j > i))
				j--;
		} else if (order == "descend") {
			while(((values[j]) < (temp)) && (j > i))
				j--;
		}
		if(j > i){
			values[i] = values[j];
			indices[i] = indices[j];
			i++;
		}
		if (order == "ascend") {
			while(((values[i]) < (temp)) && (j > i))
				i++;
		} else if (order == "descend") {
			while(((values[i]) > (temp)) && (j > i))
				i++;
		}
		if(j > i){
			values[j] = values[i];
			indices[j] = indices[i];
			j--;
		}
	} while(i != j);
	values[i] = temp;
	indices[i] = tempV;
	i++;
	j--;
	if(start < j)
		quickSort(values, indices, start, j, order);
	if(i < end)
		quickSort(values, indices, i, end, order);

}

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
void assign(double* res, int len, double v) {
	for (int i = 0; i < len; i++)
		res[i] = v;
}

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
void divideAssign(double* V, int len, double v) {
	for (int i = 0; i < len; i++)
		V[i] = V[i] / v;
}

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
void divideAssign(double* V1, double* V2, int len) {
	for (int i = 0; i < len; i++)
		V1[i] = V1[i] / V2[i];
}

double innerProduct(double* V1, double* V2, int len) {
	double res = 0;
	for (int i = 0; i < len; i++) {
		res += V1[i] * V2[i];
	}
	return res;
}

void timesAssign(double* V, int len, double v) {
	for (int i = 0; i < len; i++)
		V[i] = V[i] * v;
}

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
int argmax(double* V, int length) {

	int maxIdx = 0;
	double maxVal = V[0];
	for (int i = 1; i < length; i++) {
		if (maxVal < V[i]) {
			maxVal = V[i];
			maxIdx = i;
		}
	}
	return maxIdx;

}

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
void assignVector(double* V, int length, double v) {
	for (int i = 0; i < length; i++)
		V[i] = v;
}

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
double* allocateVector(int n, double v) {
	double* res = new double[n];
	assignVector(res, n, v);
	return res;
}

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
double* allocate1DArray(int n, double v) {
	return allocateVector(n, v);
}

/**
 * Allocate continuous memory block for a 1D {@code double}
 * array.
 *
 * @param n number of elements to be allocated
 *
 * @return a 1D {@code double} array of length n
 *
 */
double* allocate1DArray(int n) {
	return allocateVector(n, 0);
}

/**
 * Allocate continuous memory block for a 1D {@code double}
 * array.
 *
 * @param n number of elements to be allocated
 *
 * @return a 1D {@code double} array of length n
 *
 */
double* allocateVector(int n) {
	return allocateVector(n, 0);
}

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
double** allocate2DArray(int m, int n, double v) {
	double** res = new double*[m];
	for (int i = 0; i < m; i++) {
		res[i] = new double[n];
		for (int j = 0; j < n; j++) {
			res[i][j] = v;
		}
	}
	return res;
}

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
double** allocate2DArray(int m, int n) {
	return allocate2DArray(m, n, 0);
}

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
double* times(double* V1, double* V2, int len) {
	double* res = new double[len];
	for (int i = 0; i < len; i++)
		res[i] = V1[i] * V2[i];
	return res;
}

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
double* plus(double* V1, double* V2, int len) {
	double* res = new double[len];
	for (int i = 0; i < len; i++)
		res[i] = V1[i] + V2[i];
	return res;
}

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
double* minus(double* V1, double* V2, int len) {
	double* res = new double[len];
	for (int i = 0; i < len; i++)
		res[i] = V1[i] - V2[i];
	return res;
}

/**
 * res = V
 * @param res
 * @param V
 * @param len length of res and V
 */
void assignVector(double* res, double* V, int len) {
	// System.arraycopy(V, 0, res, 0, len);
	std::copy(V, V + len, res);
}

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
void plusAssign(double* V, int len, double v) {
	for (int i = 0; i < len; i++)
		V[i] += v;
}

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
void plusAssign(double* V1, double* V2, int len) {
	for (int i = 0; i < len; i++)
		V1[i] = V1[i] + V2[i];
}

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
void minusAssign(int* V, int len, int v) {
	for (int i = 0; i < len; i++)
		V[i] = V[i] - v;
}

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
void minusAssign(double* V, int len, double v) {
	for (int i = 0; i < len; i++)
		V[i] = V[i] - v;
}

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
void minusAssign(double* V1, double* V2, int len) {
	for (int i = 0; i < len; i++)
		V1[i] = V1[i] - V2[i];
}

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
double sum(double* V, int len) {
	double res = 0;
	for (int i = 0; i < len; i++)
		res += V[i];
	return res;
}

/**
 * Sum a 1D {@code double} array to one, i.e., V[i] = V[i] / sum(V).
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of V
 */
void sum2one(double* V, int len) {
	divideAssign(V, len, sum(V, len));
}

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
void timesAssign(double* V1, double* V2, int len) {
	for (int i = 0; i < len; i++)
		V1[i] = V1[i] * V2[i];
}
