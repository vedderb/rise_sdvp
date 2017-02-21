/*
 * EVD.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#include "EVD.h"
#include "Printer.h"
#include "ArrayOperator.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Matlab.h"
#include "Printer.h"
#include <cmath>
#include <map>
#include <typeinfo>

double EVD::tol = 1e-16;

int EVD::maxIter = 10;

EVD::EVD(Matrix& A) {
	Matrix** VD = decompose(A, true);
	V = VD[0];
	D = VD[1];
}

EVD::EVD(Matrix& A, double tol) {
	EVD::tol = tol;
	Matrix** VD = decompose(A, true);
	V = VD[0];
	D = VD[1];
}

EVD::~EVD() {
	delete V;
	delete D;
	// disp("Eigenvalue decomposition is released.");
}

EVD::EVD(Matrix& A, bool computeV) {
	Matrix** VD = decompose(A, computeV);
	V = VD[0];
	D = VD[1];
}

Matrix** EVD::decompose(Matrix& A) {
	return decompose(A, true);
}

Matrix** EVD::decompose(Matrix& A, bool computeV) {
	int m = A.getRowDimension();
	int n = A.getColumnDimension();
	if (m != n) {
		err("Input should be a square matrix.");
		exit(1);
	}
	maxIter = 30 * n * n;
	// A = QTQ' <=> Q'AQ = T
	Matrix** QT = tridiagonalize(A, computeV);
	Matrix* Q = QT[0];
	Matrix* T = QT[1];

	// IO.saveMatrix(full(T), "T.txt");

	/*disp(A);
			printMatrix(Q);
			printMatrix(T);*/

	// T = VDV' <=> V'TV = D
	Matrix** VD = diagonalizeTD(*T, computeV);
	Matrix* V = VD[0];
	Matrix* D = VD[1];
	/*fprintf("VDV':%n");
			disp(V.mtimes(D).mtimes(V.transpose()));*/

	// A = QTQ' = QVDV'Q' = (QV)D(QV)'
	Matrix** res = new Matrix*[2];
	res[0] = computeV ? &Q->mtimes(*V) : null;
	res[1] = D;

	if (computeV) {
		delete Q;
		delete V;
	}
	delete T;

	return res;
}

double* EVD::computeEigenvalues(Matrix& A) {
	Matrix* S = decompose(A, false)[1];
	int m = S->getRowDimension();
	int n = S->getColumnDimension();
	int len = m >= n ? n : m;
	double* s = allocateVector(len, 0);
	for (int i = 0; i < len; i++) {
		s[i] = S->getEntry(i, i);
	}
	delete S;
	return s;
}

Matrix** EVD::tridiagonalize(Matrix& A) {
	return tridiagonalize(A, true);
}

Matrix** EVD::tridiagonalize(Matrix& M, bool computeV) {
	Matrix& A = typeid(M) == typeid(DenseMatrix) ? M.copy() : full(M);
	int m = A.getRowDimension();
	int n = A.getColumnDimension();
	Matrix** QT = new Matrix*[2];
	double* a = allocateVector(n, 0);
	double* b = allocateVector(n, 0);
	double** AData = ((DenseMatrix&) A).getData();
	double c = 0;
	double s = 0;
	double r = 0;
	for (int j = 0; j < n - 2; j++) {
		a[j] = AData[j][j];
		// Householder transformation on columns of A(j+1:m, j+1:n)
		// Compute the norm of A(j+1:m, j)
		c = 0;
		for (int i = j + 1; i < m; i++) {
			c += pow(AData[i][j], 2);
		}
		if (c == 0)
			continue;
		s = sqrt(c);
		b[j] = AData[j + 1][j] > 0 ? -s : s;
		r = sqrt(s * (s + fabs(AData[j + 1][j])));

		AData[j + 1][j] -= b[j];
		for (int k = j + 1; k < m; k++) {
			AData[k][j] /= r;
		}

		double* w = new double[n - j - 1];
		double* u = new double[n - j - 1];
		double* v = new double[n - j - 1];

		for (int i = j + 1, t = 0; i < m; i++, t++) {
			u[t] = AData[i][j];
		}

		// v = B33 * u
		for (int i = j + 1, t = 0; i < m; i++, t++) {
			double* ARow_i = AData[i];
			s = 0;
			for (int k = j + 1, l = 0; k < n; k++, l++) {
				s += ARow_i[k] * u[l];
			}
			v[t] = s;
		}

		c = innerProduct(u, v, n - j - 1) / 2;
		for (int i = j + 1, t = 0; i < m; i++, t++) {
			w[t] = v[t] - c * u[t];
		}

		for (int i = j + 1, t = 0; i < m; i++, t++) {
			double* ARow_i = AData[i];
			for (int k = j + 1, l = 0; k < n; k++, l++) {
				ARow_i[k] = ARow_i[k] - (u[t] * w[l] + w[t] * u[l]);
			}
		}
		// disp(A);

		/*fprintf("Householder transformation on n - 1 columns:%n");
				disp(A);*/
		// disp(A);
		// Householder transformation on rows of A(j:m, j+1:n)
		delete[] w;
		delete[] u;
		delete[] v;
	}
	a[n - 2] = AData[n - 2][n - 2];
	a[n - 1] = AData[n - 1][n - 1];
	b[n - 2] = AData[n - 1][n - 2];
	QT = unpack(A, a, b, computeV);

	delete[] a;
	delete[] b;
	delete &A;

	return QT;
}

Matrix** EVD::unpack(Matrix& A, double* a, double* b, bool computeV) {
	Matrix** QT = new Matrix*[3];
	int m = A.getRowDimension();
	int n = A.getColumnDimension();
	DenseMatrix* Q = null;
	if (computeV) {
		Q = new DenseMatrix(m, m, 0);
		double** QData = Q->getData();
		double s = 0;
		double* y = null;
		for (int i = 0; i < m; i++) {
			// Compute U^T * e_i
			y = QData[i];
			y[i] = 1;
			for (int j = 0; j < n - 2; j++) {
				s = 0;
				for (int k = j + 1; k < m; k++) {
					s += A.getEntry(k, j) * y[k];
				}
				for (int k = j + 1; k < m; k++) {
					y[k] -= A.getEntry(k, j) * s;
				}
			}
		}
		/*fprintf("Q:%n");
				disp(Q);*/
	}
	std::map<std::pair<int, int>, double> map;
	for (int i = 0; i < m; i++) {
		if (i < n)
			map.insert(std::make_pair(std::make_pair(i, i), a[i]));
		if (i < n - 1) {
			map.insert(std::make_pair(std::make_pair(i, i + 1), b[i]));
			map.insert(std::make_pair(std::make_pair(i + 1, i), b[i]));
		}
	}
	Matrix* T = &SparseMatrix::createSparseMatrix(map, m, n);
	/*fprintf("T:%n");
			printMatrix(T);
			T = new SparseMatrix(m, n);
			for (int i = 0; i < m; i++) {
				if (i < n)
					T.setEntry(i, i, a[i]);
				if (i < n - 1) {
					T.setEntry(i, i + 1, b[i]);
					T.setEntry(i + 1, i, b[i]);
				}
			}*/
	if (computeV) {
		/*fprintf("T:%n");
				printMatrix(T);*/

		/*fprintf("A:%n");
				printMatrix(A);

				fprintf("Q'Q:%n");
				disp(Q.transpose().mtimes(Q));*/

		/*fprintf("QTQ':%n");
				disp(Q.mtimes(T).mtimes(Q.transpose()));*/
	}

	QT[0] = Q;
	QT[1] = T;
	return QT;
}

Matrix** EVD::diagonalizeTD(Matrix& T) {
	return diagonalizeTD(T, true);
}

Matrix** EVD::diagonalizeTD(Matrix& T, bool computeV) {
	int m = T.getRowDimension();
	int n = T.getColumnDimension();
	int len = m >= n ? n : m;
	int idx = 0;

	/*
	 * The tridiagonal matrix T is
	 * s[0] e[0]
	 * e[0] s[1] e[1]
	 *      e[1] ...
	 *               s[len - 2] e[len - 2]
	 *               e[len - 2] s[len - 1]
	 */
	double* s = allocateVector(len, 0);
	double* e = allocateVector(len, 0);

	for (int i = 0; i < len - 1; i++) {
		s[i] = T.getEntry(i, i);
		e[i] = T.getEntry(i, i + 1);
	}
	s[len - 1] = T.getEntry(len - 1, len - 1);

	// V': each row of V' is a right singular vector
	double** Vt = null;
	DenseMatrix* VtMatrix = null;
	if (computeV) {
		VtMatrix = &eye(n, n);
		Vt = VtMatrix->getData();
	}

	double* mu = allocate1DArray(len, 0);

	/*
	 * T0 = ITI', V = I
	 * T = IT0I' = VT0GG1'Vt = VT0G1(G1'Vt)
	 *   = VG1G1'T0G1(G1'Vt) = (VG1)(G1'T0G1)(G1'Vt)
	 *   = (VG1)T1(G1'Vt)
	 *   ...
	 *   = (Gn-1...G2G1)D(G1G2...Gn-1)
	 *
	 * where G = |cs  sn|
	 *           |-sn cs|
	 * G' = |cs -sn|
	 *      |sn  cs|
	 */

	/*
	 * Find B_hat, i.e. the bottommost unreduced submatrix of B.
	 * Index starts from 0.
	 */
	// *********************************************************

	int i_start = 0;
	int i_end = len - 1;
	// int cnt_shifted = 0;
	int ind = 1;
	while (true) {

		idx = len - 2;
		while (idx >= 0) {
			if (e[idx] == 0) {
				idx--;
			} else {
				break;
			}
		}
		i_end = idx + 1;
		// Now idx = -1 or e[idx] != 0
		// If idx = -1, then e[0] = 0, i_start = i_end = 0, e = 0
		// Else if e[idx] != 0, then e[i] = 0 for i_end = idx + 1 <= i <= len - 1
		while (idx >= 0) {
			if (e[idx] != 0) {
				idx--;
			} else {
				break;
			}
		}
		i_start = idx + 1;
		// Now idx = -1 or e[idx] = 0
		// If idx = -1 i_start = 0
		// Else if e[idx] = 0, then e[idx + 1] != 0, e[i_end - 1] != 0
		// i.e. e[i] != 0 for i_start <= i <= i_end - 1

		if (i_start == i_end) {
			break;
		}

		// Apply the stopping criterion to B_hat
		// If any e[i] is set to zero, return to loop

		bool set2Zero = false;
		mu[i_start] = fabs(s[i_start]);
		for (int j = i_start; j < i_end; j++) {
			mu[j + 1] = fabs(s[j + 1]) * mu[j] / (mu[j] + fabs(e[j]));
			if (fabs(e[j]) <= mu[j] * tol) {
				e[j] = 0;
				set2Zero = true;
			}
		}
		if (set2Zero) {
			continue;
		}

		implicitSymmetricShiftedQR(s, e, Vt, i_start, i_end, computeV, n);
		// cnt_shifted++;

		if (ind == maxIter) {
			break;
		}

		ind++;

	}

	// fprintf("cnt_shifted: %d%n", cnt_shifted);
	// *********************************************************

	// Quick sort eigenvalues and eigenvectors
	quickSort(s, Vt, 0, len - 1, "descend", computeV);

	Matrix** VD = new Matrix*[2];
	if (computeV) {
		VD[0] = &VtMatrix->transpose();
		delete VtMatrix;
	} else
		VD[0] = null;
	VD[1] = &buildD(s, m, n);

	/*disp("T:");
			printMatrix(T);
			disp("VDV':");
			Matrix V = VD[0];
			Matrix D = VD[1];
			disp(V.mtimes(D).mtimes(V.transpose()));*/

	delete[] s;
	delete[] e;
	delete[] mu;

	return VD;
}

void EVD::quickSort(double* s, double** Vt, int start, int end, std::string order, bool computeV) {
	int	i,j;
	double temp;
	i = start;
	j = end;
	temp = s[i];
	// double[] tempU = computeUV ? Ut[i] : null;
	double* tempV = computeV ? Vt[i] : null;
	do{
		if (order == "ascend") {
			while((fabs(s[j]) > fabs(temp)) && (j > i))
				j--;
		} else if (order == "descend") {
			while((fabs(s[j]) < fabs(temp)) && (j > i))
				j--;
		}
		if(j > i){
			s[i] = s[j];
			if (computeV) {
				// Ut[i] = Ut[j];
				Vt[i] = Vt[j];
			}
			i++;
		}
		if (order == "ascend") {
			while((fabs(s[i]) < fabs(temp)) && (j > i))
				i++;
		} else if (order == "descend") {
			while((fabs(s[i]) > fabs(temp)) && (j > i))
				i++;
		}
		if(j > i){
			s[j] = s[i];
			if (computeV) {
				// Ut[j] = Ut[i];
				Vt[j] = Vt[i];
			}
			j--;
		}
	} while(i != j);
	s[i] = temp;
	if (computeV) {
		// Ut[i] = tempU;
		Vt[i] = tempV;
	}
	i++;
	j--;
	if(start < j)
		quickSort(s, Vt, start, j, order, computeV);
	if(i < end)
		quickSort(s, Vt, i, end, order, computeV);
}

Matrix& EVD::buildD(double* s, int m, int n) {
	std::map<std::pair<int, int>, double> map;
	for (int i = 0; i < m; i++) {
		if (i < n)
			map.insert(std::make_pair(std::make_pair(i, i), s[i]));
	}
	return SparseMatrix::createSparseMatrix(map, m, n);
}

void EVD::implicitSymmetricShiftedQR(double* s, double* e,
		double** Vt, int i_start, int i_end, bool computeV, int n) {
	/*
	 * B(i_start:i_end, i_start:i_end) is unreduced bidiagonal matrix
	 */
	double d = 0;

	d = (s[i_end - 1] - s[i_end]) / 2;
	double c = e[i_end - 1] * e[i_end - 1];
	double shift = sqrt(d * d + c);
	shift = d > 0 ? shift : -shift;
	shift =  c / (d + shift);
	double f = s[i_start] - s[i_end] + shift;

	double g = e[i_start];
	double cs = 0, sn = 0, r = 0;
	double t, tt;
	double h = 0;

	for (int i = i_start; i < i_end; i++) {
		// ROT(f, g, cs, sn, r)
		if (f == 0) {
			cs = 0;
			sn = 1;
			r = g;
			/*sn = g > 0 ? 1 : -1;
					r = g > 0 ? g : -g;*/
		} else if (fabs(f) > fabs(g)) {
			t = g / f;
			tt = sqrt(1 + t * t);
			/*if (f < 0) {
						tt = -tt;
					}*/
			cs = 1 / tt;
			sn = t * cs;
			r = f * tt;
		} else {
			t = f / g;
			tt = sqrt(1 + t * t);
			/*if (g < 0) {
						tt = -tt;
					}*/
			sn = 1 / tt;
			cs = t * sn;
			r = g * tt;
		}
		// UPDATE(cs, sn, vi, vi+1)
		if (computeV)
			update(cs, sn, Vt[i], Vt[i + 1], n);

		if (i != i_start) { // Note that i != i_start rather than i != 0!!!
			e[i - 1] = r;
		}

		/*Bk = buildB(s, e, m, n);
				fprintf("Bk:%n");
				printMatrix(Bk);*/

		// Givens rotation on column_i and column_i+1
		f = cs * s[i] + sn * e[i];
		h = -sn * s[i] + cs * e[i];
		g = cs * e[i] + sn * s[i + 1];
		s[i + 1] = -sn * e[i] + cs * s[i + 1];

		// Givens rotation on row_i and row_i+1
		r = cs * f + sn * g;
		s[i] = r;
		e[i] = -sn * f + cs * g;

		s[i + 1] = -sn * h + cs * s[i + 1];
		h = sn * e[i + 1];
		e[i + 1] *= cs;

		if (i < i_end - 1) {
			f = e[i]; // f = T(i+1, i)
			g = h;    // g = T(i+2, i)
		}

		/*Bk = buildB(s, e, m, n);
				fprintf("Bk:%n");
				printMatrix(Bk);*/

	}
	// e[i_end - 1] = f;
}

void EVD::update(double cs, double sn, double* V1, double* V2, int len) {
	double temp;
	for (int i = 0; i < len; i++) {
		temp = V1[i];
		V1[i] = cs * temp + sn * V2[i];
		V2[i] = -sn * temp + cs * V2[i];
	}
}
