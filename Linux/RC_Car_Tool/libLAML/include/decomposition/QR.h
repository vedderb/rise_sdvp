/*
 * QR.h
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#ifndef QR_H_
#define QR_H_

#include "Matrix.h"

/***
 * A C++ implementation of QR decomposition using
 * Householder transformations with column pivoting.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 12th, 2014
 */
class QR {

private:

	/**
	 * The orthogonal matrix in the QR decomposition
	 * such that AP = QR.
	 */
	Matrix* Q;

	/**
	 * The upper triangular matrix in the QR decomposition
	 * such that AP = QR.
	 */
	Matrix* R;

	/**
	 * The column permutation matrix in the QR decomposition
	 * such that AP = QR.
	 */
	Matrix* P;

public:

	/**
	 * Construct a QRDecomposition instance from an arbitrary
	 * real matrix A.
	 *
	 * @param A a general dense or sparse real matrix
	 */
	QR(Matrix& A);

	~QR();

	/**
	 * QR decomposition with column pivoting. That is, AP = QR.
	 *
	 * @param A a dense or sparse real matrix
	 *
	 * @return a {@code Matrix} array [Q, R, P]
	 */
	static Matrix** decompose(Matrix& A);

	/**
	 * Get the orthogonal matrix in the QR decomposition.
	 *
	 * @return Q
	 */
	Matrix* getQ() {
		return Q;
	}

	/**
	 * Get the upper triangular matrix in the QR decomposition.
	 *
	 * @return R
	 */
	Matrix* getR() {
		return R;
	}

	/**
	 * Get the column permutation matrix in the QR decomposition.
	 *
	 * @return P
	 */
	Matrix* getP() {
		return P;
	}

	/**
	 * Solve the system of linear equations Ax = b in the least
	 * square sense, i.e. X minimizes norm(Ax - b). The rank k
	 * of A is determined from the QR decomposition with column
	 * pivoting. The computed solution X has at most k nonzero
	 * elements per column. If k < n, this is usually not the
	 * same solution as x = pinv(A) * b, which returns a least
	 * squares solution.
	 *
	 * @param b a dense or sparse real vector
	 *
	 * @return this \ b
	 */
	Vector& solve(Vector& b);

	/**
	 * Solve the system of linear equations AX = B in the least
	 * square sense, i.e. X minimizes norm(AX - B). The rank k
	 * of A is determined from the QR decomposition with column
	 * pivoting. The computed solution X has at most k nonzero
	 * elements per column. If k < n, this is usually not the
	 * same solution as X = pinv(A) * B, which returns a least
	 * squares solution.
	 *
	 * @param B a dense or sparse real matrix
	 *
	 * @return this \ B
	 */
	Matrix& solve(Matrix& B);

private:

	/**
	 * Do QR decomposition for an arbitrary real matrix.
	 *
	 * @param A a general dense or sparse real matrix
	 *
	 * @return a {@code Matrix} array [Q, R, P]
	 */
	Matrix** run(Matrix& A);

	/**
	 * Compute Q from A, the QR algorithm with column pivoting result.
	 *
	 * @param A QR algorithm with column pivoting result
	 *
	 * @return Q
	 */
	static Matrix& computeQ(Matrix& A);

	/**
	 * Compute the upper triangular matrix from A, the QR algorithm
	 * with column pivoting result, the diagonal elements are in the
	 * 1D {@code double} array d.
	 *
	 * @param A QR algorithm with column pivoting result
	 *
	 * @param d a 1D {@code double} array containing the diagonal
	 * 		    elements of the upper triangular matrix
	 *
	 * @return R
	 */
	static Matrix& computeR(Matrix& A, double* d);

};


#endif /* QR_H_ */
