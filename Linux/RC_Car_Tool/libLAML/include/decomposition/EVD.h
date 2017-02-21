/*
 * EVD.h
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#ifndef EVD_H_
#define EVD_H_

#include <string>
#include "Matrix.h"

/***
 * A C++ implementation of the eigenvalue decomposition
 * for a symmetric real matrix.
 * <p/>
 * The input matrix is first reduced to tridiagonal
 * matrix and then is diagonalized by implicit symmetric
 * shifted QR algorithm.
 *
 * @author Mingjie Qian
 * @version 1.0 Feb. 12th, 2014
 */
class EVD {

private:

	/**
	 * Eigenvectors.
	 */
	Matrix* V;

	/**
	 * A sparse diagonal matrix D with its diagonal being all
	 * eigenvalues in decreasing order (absolute value).
	 */
	Matrix* D;


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
	 * Construct this eigenvalue decomposition instance
	 * from a real symmetric matrix.
	 *
	 * @param A a real symmetric matrix
	 */
	EVD(Matrix& A);

	~EVD();

	/**
	 * Construct this eigenvalue decomposition instance
	 * from a real symmetric matrix.
	 *
	 * @param A a real symmetric matrix
	 *
	 * @param tol tolerance
	 */
	EVD(Matrix& A, double tol);

	/**
	 * Construct this eigenvalue decomposition instance
	 * from a real symmetric matrix.
	 *
	 * @param A a real symmetric matrix
	 *
	 * @param computeV if V is to be computed
	 */
	EVD(Matrix& A, bool computeV);

	/**
	 * Do eigenvalue decomposition for a real symmetric matrix,
	 * i.e. AV = VD.
	 *
	 * @param A a real symmetric matrix
	 *
	 * @return a {@code Matrix} array [V, D]
	 */
	static Matrix** decompose(Matrix& A);

	/**
	 * Do eigenvalue decomposition for a real symmetric matrix,
	 * i.e. AV = VD.
	 *
	 * @param A a real symmetric matrix
	 *
	 * @param computeV if V is to be computed
	 *
	 * @return a {@code Matrix} array [V, D]
	 */
	static Matrix** decompose(Matrix& A, bool computeV);

	/**
	 * Only eigenvalues of a symmetric real matrix are computed.
	 *
	 * @param A a symmetric real matrix
	 *
	 * @return a 1D {@code double} array containing the eigenvalues
	 * 				in decreasing order (absolute value)
	 */
	static double* computeEigenvalues(Matrix& A);

	/**
	 * Get eigenvectors.
	 *
	 * @return V
	 */
	Matrix* getV() {
		return V;
	}

	/**
	 * Get a diagonal matrix containing the eigenvalues
	 * in decreasing order.
	 *
	 * @return D
	 */
	Matrix* getD() {
		return D;
	}

private:

	/**
	 * Tridiagonalize a real symmetric matrix A, i.e. A = Q * T * Q'
	 * such that Q is an orthogonal matrix and T is a tridiagonal matrix.
	 * <p>
	 * A = QTQ' <=> Q'AQ = T
	 *
	 * @param A a real symmetric matrix
	 *
	 * @return a {@code Matrix} array [Q, T]
	 */
	static Matrix** tridiagonalize(Matrix& A);

	/**
	 * Tridiagonalize a real symmetric matrix A, i.e. A = Q * T * Q'
	 * such that Q is an orthogonal matrix and T is a tridiagonal matrix.
	 * <p>
	 * A = QTQ' <=> Q'AQ = T
	 *
	 * @param A a real symmetric matrix
	 *
	 * @param computeV if V is to be computed
	 *
	 * @return a {@code Matrix} array [Q, T]
	 */
	static Matrix** tridiagonalize(Matrix& A, bool computeV);

	/**
	 * Unpack Q and T from the result of tridiagonalization.
	 *
	 * @param A tridiagonalization result
	 *
	 * @param a diagonal
	 *
	 * @param b superdiagonal
	 *
	 * @param computeV if V is to be computed
	 *
	 * @return a {@code Matrix} array [Q, T]
	 */
	static Matrix** unpack(Matrix& A, double* a, double* b, bool computeV);

	/**
	 * Do eigenvalue decomposition for a real symmetric tridiagonal
	 * matrix, i.e. T = VDV'.
	 *
	 * @param T a real symmetric tridiagonal matrix
	 *
	 * @return a {@code Matrix} array [V, D]
	 */
	static Matrix** diagonalizeTD(Matrix& T);

	/**
	 * Do eigenvalue decomposition for a real symmetric tridiagonal
	 * matrix, i.e. T = VDV'.
	 *
	 * @param T a real symmetric tridiagonal matrix
	 *
	 * @param computeV  if V is to be computed
	 *
	 * @return a {@code Matrix} array [V, D]
	 */
	static Matrix** diagonalizeTD(Matrix& T, bool computeV);

	/**
	 * Sort the eigenvalues in a specified order. If computeV is true,
	 * eigenvectors will also be sorted.
	 *
	 * @param s a 1D {@code double} array containing the eigenvalues
	 *
	 * @param Vt eigenvectors
	 *
	 * @param start start index (inclusive)
	 *
	 * @param end end index (inclusive)
	 *
	 * @param order a {@code String} either "descend" or "ascend"
	 *
	 * @param computeV if V is to be computed
	 */
	static void quickSort(double* s, double** Vt, int start, int end, std::string order, bool computeV);

	/**
	 * Build the diagonal matrix containing all eigenvalues.
	 *
	 * @param s eigenvalues
	 *
	 * @param m number of rows
	 *
	 * @param n number of columns
	 *
	 * @return a diagonal matrix containing all eigenvalues
	 */
	static Matrix& buildD(double* s, int m, int n);

	/**
	 * Implicit symmetric shifted QR algorithm on B_hat which is
	 * the bottommost unreduced submatrix of B begin from
	 * i_start (inclusive) to i_end (inclusive).
	 *
	 * @param s diagonal elements
	 *
	 * @param e superdiagonal elements
	 *
	 * @param Vt transposition of eigenvector matrix
	 *
	 * @param i_start start index of B_hat (inclusive)
	 *
	 * @param i_end end index of B_hat (inclusive)
	 *
	 * @param computeV if V is to be computed
	 *
	 * @param n number of rows in Vt
	 */
	static void implicitSymmetricShiftedQR(double* s, double* e,
			double** Vt, int i_start, int i_end, bool computeV, int n);

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


#endif /* EVD_H_ */
