/*
 * InPlaceOperator.h
 *
 *  Created on: Feb 20, 2014
 *      Author: Mingjie Qian
 */

#ifndef INPLACEOPERATOR_H_
#define INPLACEOPERATOR_H_

#include "Matrix.h"

/**
 * res = A * V.
 *
 * @param res
 * @param A
 * @param V
 */
void operate(Vector& res, Matrix& A, Vector& V);

/**
 * res' = V' * A.
 *
 * @param res
 * @param V
 * @param A
 */
void operate(Vector& res, Vector& V, Matrix& A);

/**
 * res = abs(A).
 *
 * @param res
 * @param A
 */
void abs(Matrix& res, Matrix& A);

/**
 * res = A
 * @param res
 * @param A
 */
void assign(Matrix& res, Matrix& A);

/**
 * res = subplus(A).
 *
 * @param res
 * @param A
 */
void subplus(Matrix& res, Matrix& A);

/**
 * res = subplus(res).
 *
 * @param res
 */
void subplusAssign(Matrix& res);

/**
 * res = A | B.
 *
 * @param res
 * @param A
 * @param B
 */
void _or(Matrix& res, Matrix& A, Matrix& B);

/**
 * res = a * V + b * U.
 *
 * @param res
 * @param a
 * @param V
 * @param b
 * @param U
 */
void affine(Vector& res, double a, Vector& V, double b, Vector& U);

/**
 * res = a * V + U if operator is '+',</br>
 * res = a * V - U if operator is '-'.
 *
 * @param res
 * @param a
 * @param V
 * @param _operator a {@code char} variable: '+' or '-'
 * @param U
 */
void affine(Vector& res, double a, Vector& V, char _operator, Vector& U);

/**
 * res = V + b * U.
 *
 * @param res
 * @param V
 * @param b
 * @param U
 */
void affine(Vector& res, Vector& V, double b, Vector& U);

/**
 * res = -res.
 *
 * @param res
 */
void uminusAssign(Vector& res);

/**
 * res = -V.
 *
 * @param res
 * @param V
 */
void uminus(Vector& res, Vector& V);

/**
 * res = V .* U.
 *
 * @param res
 * @param V
 * @param U
 */
void times(Vector& res, Vector& V, Vector& U);

/**
 * res = v * V.
 *
 * @param res
 * @param v
 * @param V
 */
void times(Vector& res, double v, Vector& V);

/**
 * res *= V.
 *
 * @param res
 * @param V
 */
void timesAssign(Vector& res, Vector& V);

/**
 * res *= v.
 *
 * @param res
 * @param v
 */
void timesAssign(Vector& res, double v);

/**
 * res = V.
 *
 * @param res
 * @param V
 */
void assign(Vector& res, Vector& V);

/**
 * res = V - U.
 *
 * @param res
 * @param V
 * @param U
 */
void minus(Vector& res, Vector& V, Vector& U);

/**
 * res = V - v.
 *
 * @param res
 * @param V
 * @param v
 */
void minus(Vector& res, Vector& V, double v);

/**
 * res = v - V.
 *
 * @param res
 * @param v
 * @param V
 */
void minus(Vector& res, double v, Vector& V);

/**
 * res -= v.
 *
 * @param res
 * @param v
 */
void minusAssign(Vector& res, double v);

/**
 * res -= V
 * @param res
 * @param V
 */
void minusAssign(Vector& res, Vector& V);

/**
 * res -= a * V.
 *
 * @param res
 * @param a
 * @param V
 */
void minusAssign(Vector& res, double a, Vector& V);

/**
 * res = V + U.
 *
 * @param res
 * @param V
 * @param U
 */
void plus(Vector& res, Vector& V, Vector& U);

/**
 * res = V + v.
 *
 * @param res
 * @param V
 * @param v
 */
void plus(Vector& res, Vector& V, double v);

/**
 * res = v + V.
 *
 * @param res
 * @param v
 * @param V
 */
void plus(Vector& res, double v, Vector& V);

/**
 * res += v.
 *
 * @param res
 * @param v
 */
void plusAssign(Vector& res, double v);

/**
 * res += V
 * @param res
 * @param V
 */
void plusAssign(Vector& res, Vector& V);

/**
 * res += a * V.
 *
 * @param res
 * @param a
 * @param V
 */
void plusAssign(Vector& res, double a, Vector& V);

/**
 * res = A * B if operator is ' ',</br>
 * res = A<sup>T</sup> * B if operator is 'T'.
 *
 * @param res
 * @param A
 * @param _operator a {@code char} variable: 'T' or ' '
 * @param B
 */
void mtimes(Matrix& res, Matrix& A, char _operator, Matrix& B);

/**
 * res = A * B.
 * @param res result matrix
 * @param A	a real matrix
 * @param B a real matrix
 */
void mtimes(Matrix& res, Matrix& A, Matrix& B);

/**
 * res = A .* B
 * @param res
 * @param A
 * @param B
 */
void times(Matrix& res, Matrix& A, Matrix& B);

/**
 * res = a * V
 * @param res
 * @param a
 * @param V
 * @param len length of res and V
 */
void times(double* res, double a, double* V, int len);

/**
 * res = res .* A
 * @param res
 * @param A
 */
void timesAssign(Matrix& res, Matrix& A);

/**
 * res = v * res
 * @param res
 * @param v
 */
void timesAssign(Matrix& res, double v);

/**
 * res = v * A
 * @param res
 * @param v
 * @param A
 */
void times(Matrix& res, double v, Matrix& A);

/**
 * Clear the input matrix.
 *
 * @param res a real matrix
 */
void clear(Matrix& res);

/**
 * Clear the input 2D {@code double} array.
 *
 * @param res a 2D {@code double} array
 *
 * @param M number of rows
 *
 * @param N number of columns
 */
void clear(double** res, int M, int N);

/**
 * Clear the input vector.
 *
 * @param res a real vector
 */
void clear(Vector& res);

/**
 * Clear the input 1D {@code double} array.
 *
 * @param res a 1D {@code double} array
 *
 * @param len length of res
 */
void clear(double* res, int len);

/**
 * res = V
 * @param res
 * @param V
 * @param len length of res and V
 */
void assign(double* res, double* V, int len);

/**
 * Assign a 1D {@code double} array by a real scalar.
 *
 * @param res a 1D {@code double} array
 *
 * @param len length of res
 *
 * @param v a real scalar
 *

void assign(double* res, int len, double v);*/

/**
 * res = -res
 * @param res
 */
void uminusAssign(Matrix& res);

/**
 * res = -A
 * @param res
 * @param A
 */
void uminus(Matrix& res, Matrix& A);

/**
 * res = -V
 * @param res
 * @param V
 * @param len length of res and V
 */
void uminus(double* res, double* V, int len);

/**
 * res = v \ res
 * @param res
 * @param v
 */
void divide(Matrix& res, double v);

/**
 * res = res / v
 * @param res
 * @param v
 */
void rdivideAssign(Matrix& res, double v);

/**
 * res = A * B + v * C
 * @param res
 * @param A
 * @param B
 * @param v
 * @param C
 */
void affine(Matrix& res, Matrix& A, Matrix& B, double v, Matrix& C);

/**
 * res = A * B + C if operator is '+',</br>
 * res = A * B - C if operator is '-'.
 *
 * @param res
 * @param A
 * @param B
 * @param _operator a {@code char} variable: '+' or '-'
 * @param C
 */
void affine(Matrix& res, Matrix& A, Matrix& B, char _operator, Matrix& C);

/**
 * res = A * B + v
 * @param res
 * @param A
 * @param B
 * @param v
 */
void affine(Matrix& res, Matrix& A, Matrix& B, double v);

/**
 * res = a * A + b * B
 * @param res
 * @param a
 * @param A
 * @param b
 * @param B
 */
void affine(Matrix& res, double a, Matrix& A, double b, Matrix& B);

/**
 * res = a * A + B if operator is '+',</br>
 * res = a * A - B if operator is '-'.
 *
 * @param res
 * @param a
 * @param A
 * @param _operator a {@code char} variable: '+' or '-'
 * @param B
 */
void affine(Matrix& res, double a, Matrix& A, char _operator, Matrix& B);

/**
 * res = A + b * B
 * @param res
 * @param A
 * @param b
 * @param B
 */
void affine(Matrix& res, Matrix& A, double b, Matrix& B);

/**
 * res = a * A + b
 * @param res
 * @param a
 * @param A
 * @param b
 */
void affine(Matrix& res, double a, Matrix& A, double b);

/**
 * res = A + B
 * @param res
 * @param A
 * @param B
 */
void plus(Matrix& res, Matrix& A, Matrix& B);

/**
 * res = A + v;
 * @param res
 * @param A
 * @param v
 */
void plus(Matrix& res, Matrix& A, double v);

/**
 * res = res + A
 * @param res
 * @param A
 */
void plusAssign(Matrix& res, Matrix& A);

/**
 * Element-wise addition and assignment operation.
 * It adds the first argument by the second argument
 * and assign the result to the first argument, i.e., res += V2.
 *
 * @param res a 1D {@code double} array
 *
 * @param V a 1D {@code double} array
 *
 * @param len length of res and V
 *

void plusAssign(double* res, double* V, int len);*/

/**
 * res = res + v
 * @param res
 * @param v
 */
void plusAssign(Matrix& res, double v);

/**
 * res += a * A.
 *
 * @param res
 * @param a
 * @param A
 */
void plusAssign(Matrix& res, double a, Matrix& A);

/**
 * res = A - B
 * @param res
 * @param A
 * @param B
 */
void minus(Matrix& res, Matrix& A, Matrix& B);

/**
 * res = A - v
 * @param res
 * @param A
 * @param v
 */
void minus(Matrix& res, Matrix& A, double v);

/**
 * res = v - A
 * @param res
 * @param v
 * @param A
 */
void minus(Matrix& res, double v, Matrix& A);

/**
 * res = res - A
 * @param res
 * @param A
 */
void minusAssign(Matrix& res, Matrix& A);

/**
 * res = res - v
 * @param res
 * @param v
 */
void minusAssign(Matrix& res, double v);

/**
 * res -= a * A.
 *
 * @param res
 * @param a
 * @param A
 */
void minusAssign(Matrix& res, double a, Matrix& A);

/**
 * res = log(A).
 * @param res
 * @param A
 */
void log(Matrix& res, Matrix& A);

/**
 * res = log(V).
 *
 * @param res
 * @param V
 * @param len length of res and V
 */
void log(double* res, double* V, int len);

/**
 * res = log(res).
 *
 * @param res
 *
 * @param len length of res
 */
void logAssign(double* res, int len);

/**
 * res = log(res).
 * @param res
 */
void logAssign(Matrix& res);

/**
 * res = exp(A).
 * @param res
 * @param A
 */
void exp(Matrix& res, Matrix& A);

/**
 * res = exp(res).
 *
 * @param res
 */
void expAssign(Matrix& res);

/**
 * Calculate the sigmoid of a matrix A by rows. Specifically, supposing
 * that the input activation matrix is [a11, a12; a21, a22], the output
 * value is
 * <p>
 * [exp(a11) / exp(a11) + exp(a12), exp(a12) / exp(a11) + exp(a12);
 * </br>
 * exp(a21) / exp(a21) + exp(a22), exp(a22) / exp(a21) + exp(a22)].
 *
 * @param res resulted matrix
 *
 * @param A a real matrix
 *
 */
void sigmoid(Matrix& res, Matrix& A);

#endif /* INPLACEOPERATOR_H_ */
