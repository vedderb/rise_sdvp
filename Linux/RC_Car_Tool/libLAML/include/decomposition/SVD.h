/*
 * SVD.h
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#ifndef SVD_H_
#define SVD_H_

#include "Matrix.h"
#include <string>

/***
 * A C++ implementation for the singular value decomposition
 * of a general m-by-n matrix.
 * <p/>
 * The input matrix is first reduced to bidiagonal
 * matrix and then is diagonalized by hybrid of standard
 * implicit shifted QR algorithm and implicit zero-shift
 * QR algorithm. All the singular values are computed to
 * high relative accuracy independent of their magnitudes.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 12th, 2014
 */
class SVD {

private:

	/**
	 * Left singular vectors.
	 */
	Matrix* U;

	/**
	 * A sparse matrix S with its diagonal being all
	 * singular values in decreasing order.
	 */
	Matrix* S;

	/**
	 * Right singular vectors.
	 */
	Matrix* V;

public:

	/**
	 * Tolerance.
	 */
	static double tol;

	/**
	 * Maximum number of iterations.
	 */
	static int maxIter;

	/**
	 * Construct this singular value decomposition instance
	 * from a matrix.
	 *
	 * @param A a real matrix
	 */
	SVD(Matrix& A);

	~SVD();

	/**
	 * Construct this singular value decomposition instance
	 * from a matrix.
	 *
	 * @param A a real matrix
	 *
	 * @param computeUV if U and V are to be computed
	 */
	SVD(Matrix& A, bool computeUV);

	/**
	 * Do singular value decompose for a general real matrix A, i.e.
	 * A = U * S * V'.
	 *
	 * @param A an m x n real matrix
	 *
	 * @return a {@code Matrix} array [U, S, V]
	 */
	static Matrix** decompose(Matrix& A);

	/**
	 * Do singular value decompose for a general real matrix A, i.e.
	 * A = U * S * V'.
	 *
	 * @param A an m x n real matrix
	 *
	 * @param computeUV if U and V are to be computed
	 *
	 * @return a {@code Matrix} array [U, S, V]
	 */
	static Matrix** decompose(Matrix& A, bool computeUV);

	/**
	 * Only singular values of a matrix are computed.
	 *
	 * @param A a matrix
	 *
	 * @return a 1D {@code double} array containing the singular values
	 * 				in decreasing order
	 */
	static double* computeSingularValues(Matrix& A);

	/**
	 * Compute the rank of a matrix. The rank function provides
	 * an estimate of the number of linearly independent rows or
	 * columns of a matrix.
	 *
	 * @param A a matrix
	 *
	 * @return rank of the given matrix
	 */
	static int rank(Matrix& A);

	/**
	 * Get the left singular vectors.
	 *
	 * @return U
	 */
	Matrix* getU() {
		return U;
	}

	/**
	 * Get the matrix with its diagonal being all
	 * singular values in decreasing order.
	 *
	 * @return S
	 */
	Matrix* getS() {
		return S;
	}

	/**
	 * Get the right singular vectors.
	 *
	 * @return V
	 */
	Matrix* getV() {
		return V;
	}

private:

	/***
	 * Bidiagonalize a matrix A, i.e. A = U * B * V' such that
	 * U and V are orthogonal matrices and B is a bidiagonal matrix.
	 *
	 * @param A a dense or sparse matrix
	 *
	 * @param computeUV if U and V are to be computed
	 *
	 * @return a {@code Matrix} array [U, B, V]
	 */
	static Matrix** bidiagonalize(Matrix& A, bool computeUV);

	/**
	 * Unpack U, B, and V from the result of bidiagonalization.
	 *
	 * @param A bidiagonalization result
	 *
	 * @param d diagonal
	 *
	 * @param e superdiagonal
	 *
	 * @param computeUV if U and V are to be computed
	 *
	 * @return a {@code Matrix} array [U, B, V]
	 */
	static Matrix** unpack(Matrix& A, double* d, double* e, bool computeUV);

	/**
	 * Do singular value decompose for an m x n bidiagonal real matrix B, i.e.
	 * B = U * S * V'.
	 *
	 * @param B an m x n bidiagonal real matrix
	 *
	 * @param computeUV if U and V are to be computed
	 *
	 * @return a {@code Matrix} array [U, S, V]
	 */
	static Matrix** diagonalizeBD(Matrix& B, bool computeUV);

	/**
	 * Sort the singular values in a specified order. If computeUV is true,
	 * left and right singular vectors will also be sorted.
	 *
	 * @param s a 1D {@code double} array containing the singular values
	 *
	 * @param Ut left singular vectors
	 *
	 * @param Vt right singular vectors
	 *
	 * @param start start index (inclusive)
	 *
	 * @param end end index (inclusive)
	 *
	 * @param order a {@code string} either "descend" or "ascend"
	 *
	 * @param computeUV if U and V are to be computed
	 */
	static void quickSort(double* s, double** Ut, double** Vt, int start, int end, std::string order, bool computeUV);

	/**
	 * Implicit zero-shift QR algorithm on B_hat which is
	 * the bottommost unreduced submatrix of B begin from
	 * i_start (inclusive) to i_end (inclusive).
	 *
	 * @param s diagonal elements
	 *
	 * @param e superdiagonal elements
	 *
	 * @param Ut left singular matrix
	 *
	 * @param Vt right singular matrix
	 *
	 * @param i_start start index of B_hat (inclusive)
	 *
	 * @param i_end end index of B_hat (inclusive)
	 *
	 * @param computeUV if U and V are to be computed
	 *
	 * @param m	number of rows in Ut
	 *
	 * @param n number of rows in Vt
	 */
	static void implicitZeroShiftQR(double* s, double* e, double** Ut, double** Vt, int i_start, int i_end, bool computeUV, int m, int n);

	static Matrix& buildS(double* s, int m, int n);

	/**
	 * Update two 1D {@code double} arrays V1 and V2 by Givens rotation
	 * parameterized by cs and sn, i.e.
	 * [V1 V2] * |cs -sn| or |cs  sn| * |V1'|
	 *           |sn  cs|    |-sn cs|   |V2'|
	 * @param cs cos(theta)
	 *
	 * @param sn sin(theta)
	 *
	 * @param V1 a 1D {@code double} arrays
	 *
	 * @param V2 a 1D {@code double} arrays
	 *
	 * @param len length of the arrays
	 */
	static void update(double cs, double sn, double* V1, double* V2, int len);

};


#endif /* SVD_H_ */
