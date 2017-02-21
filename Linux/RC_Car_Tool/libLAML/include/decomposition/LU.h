/*
 * LU.h
 *
 *  Created on: Feb 11, 2014
 *      Author: Mingjie Qian
 */

#ifndef LU_H_
#define LU_H_

#include "Matrix.h"

/***
 * A C++ implementation of LU decomposition using
 * Gaussian elimination with row pivoting.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 11th, 2014
 */
class LU {

private:

	/**
	 * The lower triangular matrix in the LU decomposition such that
	 * PA = LU.
	 */
	Matrix* L;

	/**
	 * The upper triangular matrix in the LU decomposition such that
	 * PA = LU.
	 */
	Matrix* U;

	/**
	 * The row permutation matrix in the LU decomposition such that
	 * PA = LU.
	 */
	Matrix* P;

	/**
	 * Number of row exchanges.
	 */
	int numRowExchange;

	/**
	 * Do LU decomposition from a square real matrix.
	 *
	 * @param A a dense or sparse real square matrix
	 *
	 * @return a {@code Matrix} array [L, U, P]
	 */
	Matrix** run(Matrix& A);

public:

	/**
	 * Construct an LUDecomposition instance from a square real matrix.
	 *
	 * @param A a dense or sparse real square matrix
	 */
	LU(Matrix& A);

	~LU();

	/**
	 * Get the lower triangular matrix in the LU decomposition.
	 *
	 * @return L
	 */
	Matrix* getL() {
		return L;
	}

	/**
	 * Get the upper triangular matrix in the LU decomposition.
	 *
	 * @return U
	 */
	Matrix* getU() {
		return U;
	}

	/**
	 * Get the row permutation matrix in the LU decomposition.
	 *
	 * @return P
	 */
	Matrix* getP() {
		return P;
	}

	/**
	 * Do LU decomposition from a square real matrix.
	 *
	 * @param A a dense or sparse real square matrix
	 *
	 * @return a {@code Matrix} array [L, U, P]
	 */
	static Matrix** decompose(Matrix& A);

	/**
	 * Solve the system of linear equations Ax = b for a real
	 * square matrix A.
	 *
	 * @param b a dense or sparse real vector
	 *
	 * @return x s.t. Ax = b
	 */
	Vector& solve(Vector& b);

	/**
	 * Solve the system of linear equations AX = B for a real
	 * square matrix A.
	 *
	 * @param B a dense or sparse real matrix
	 *
	 * @return X s.t. AX = B
	 */
	Matrix& solve(Matrix& B);

	/**
	 * Compute the inverse of this real square matrix.
	 *
	 * @return this<sup>-1</sup>
	 */
	Matrix& inverse();

	/**
	 * Compute the determinant of this real square matrix.
	 *
	 * @return det(this)
	 */
	double det();

};

#endif /* LU_H_ */
