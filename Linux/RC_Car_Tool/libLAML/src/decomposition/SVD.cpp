/*
 * SVD.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#include "SVD.h"
#include "ArrayOperator.h"
#include "Matrix.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Utility.h"
#include "Matlab.h"
#include "Printer.h"
#include <cmath>
#include <map>

double SVD::tol = 1e-12;
int SVD::maxIter = 10;

SVD::SVD(Matrix& A) {
	Matrix** USV = decompose(A, true);
	U = USV[0];
	S = USV[1];
	V = USV[2];
}

SVD::~SVD() {
	delete U;
	delete S;
	delete V;
	// disp("SVD is released.");
}

SVD::SVD(Matrix& A, bool computeUV) {
	Matrix** USV = decompose(A, computeUV);
	U = USV[0];
	S = USV[1];
	V = USV[2];
}

Matrix** SVD::decompose(Matrix& A) {
	return decompose(A, true);
}

Matrix** SVD::decompose(Matrix& A, bool computeUV) {
	// int m = A.getRowDimension();
	int n = A.getColumnDimension();
	maxIter = 3 * n * n;

	// A = U1BV1'
	Matrix** UBV = bidiagonalize(A, computeUV);
	Matrix& B = *UBV[1];

	// B = U2SV2'
	Matrix** USV = diagonalizeBD(B, computeUV);

	// A = U1BV1' = U1U2SV2'V1' = (U1U2)S(V1V2)'
	Matrix** res = new Matrix*[3];

	if (computeUV)
		res[0] = &UBV[0]->mtimes(*USV[0]);
	else
		res[0] = null;
	res[1] = USV[1];
	if (computeUV)
		res[2] = &UBV[2]->mtimes(*USV[2]);
	else
		res[2] = null;

	for (int i = 0; i < 3; i++) {
		delete UBV[i];
		UBV[i] = NULL;
	}
	delete[] UBV;
	UBV = NULL;
	delete USV[0];
	USV[0] = NULL;
	delete USV[2];
	USV[2] = NULL;
	delete[] USV;
	USV = NULL;

	return res;

}

double* SVD::computeSingularValues(Matrix& A) {
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

int SVD::rank(Matrix& A) {
	int r = 0;
	double* s = computeSingularValues(A);
	int m = A.getRowDimension();
	int n = A.getColumnDimension();
	int len = m >= n ? n : m;
	double t = m >= n ? m * pow(2, -52) : n * pow(2, -52);
	for (int i = 0; i < len; i++) {
		if (s[i] > t) {
			r++;
		}
	}
	delete[] s;
	return r;
}

Matrix** SVD::bidiagonalize(Matrix& M, bool computeUV) {

	Matrix& A = typeid(M) == typeid(DenseMatrix) ? M.copy() : full(M);
	int m = A.getRowDimension();
	int n = A.getColumnDimension();
	Matrix** UBV = new Matrix*[3];
	double* d = allocateVector(n, 0);
	double* e = allocateVector(n, 0);

	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double c = 0;
		double s = 0;
		double r = 0;
		for (int j = 0; j < n; j++) {
			if (j >= m) {
				break;
			}
			// Householder transformation on columns of A(j:m, j:n)
			// Compute the norm of A(j:m, j)
			c = 0;
			for (int i = j; i < m; i++) {
				c += pow(AData[i][j], 2);
			}
			if (c != 0) {
				s = sqrt(c);
				d[j] = AData[j][j] > 0 ? -s : s;
				r = sqrt(s * (s + fabs(AData[j][j])));
				AData[j][j] -= d[j];
				for (int k = j; k < m; k++) {
					AData[k][j] /= r;
				}
				for (int k = j + 1; k < n; k++) {
					s = 0;
					for (int t = j; t < m; t++) {
						s += AData[t][j] * AData[t][k];
					}
					for (int t = j; t < m; t++) {
						AData[t][k] -= s * AData[t][j];
					}
				}
			}
			/*fprintf("Householder transformation on n - 1 columns:%n");
					disp(A);*/
			// disp(A);
			// Householder transformation on rows of A(j:m, j+1:n)
			if (j >= n - 1) // We do row-wise HouseHolder transformation n - 1 times
				continue;
			c = 0;
			double* ARow_j = AData[j];
			for (int k = j + 1; k < n; k++) {
				c += pow(ARow_j[k], 2);
			}
			if (c != 0) {
				s = sqrt(c);
				e[j + 1] = ARow_j[j + 1] > 0 ? -s : s;
				r = sqrt(s * (s + fabs(ARow_j[j + 1])));
				ARow_j[j + 1] -= e[j + 1];
				for (int k = j + 1; k < n; k++) {
					ARow_j[k] /= r;
				}
				double* ARow_k = null;
				for (int k = j + 1; k < m; k++) {
					ARow_k = AData[k];
					s = 0;
					for (int t = j + 1; t < n; t++) {
						s += ARow_j[t] * ARow_k[t];
					}
					for (int t = j + 1; t < n; t++) {
						ARow_k[t] -= s * ARow_j[t];
					}
				}
			}
			/*fprintf("Householder transformation on rows:%n");
					disp(A);*/
		}
	} else {
	}

	/*disp("Processed A:");
	printMatrix(A);*/

	UBV = unpack(A, d, e, computeUV);
	delete[] d;
	delete[] e;
	delete &A;
	return UBV;
}

Matrix** SVD::unpack(Matrix& A, double* d, double* e, bool computeUV) {
	Matrix** UBV = new Matrix*[3];
	int m = A.getRowDimension();
	int n = A.getColumnDimension();
	DenseMatrix* U = null;
	if (computeUV) {
		U = new DenseMatrix(m, m, 0);
		double** UData = U->getData();
		double s = 0;
		double* y = null;
		for (int i = 0; i < m; i++) {
			// Compute U^T * e_i
			y = UData[i];
			y[i] = 1;
			for (int j = 0; j < n; j++) {
				s = 0;
				for (int k = j; k < m; k++) {
					s += A.getEntry(k, j) * y[k];
				}
				for (int k = j; k < m; k++) {
					y[k] -= A.getEntry(k, j) * s;
				}
			}
		}
		/*fprintf("U:%n");
				disp(U);*/
	}

	std::map<std::pair<int, int>, double> map;
	for (int i = 0; i < m; i++) {
		if (i < n)
			map.insert(std::make_pair(std::make_pair(i, i), d[i]));
		if (i < n - 1) {
			map.insert(std::make_pair(std::make_pair(i, i + 1), e[i + 1]));
		}

	}
	Matrix* B = &SparseMatrix::createSparseMatrix(map, m, n);
	/*fprintf("B:%n");
			printMatrix(B);*/

	DenseMatrix* V = null;
	if (computeUV) {
		V = new DenseMatrix(n, n, 0);
		double** VData = V->getData();
		double s = 0;
		double* y = null;
		for (int i = 0; i < n; i++) {
			// Compute V^T * e_i
			y = VData[i];
			y[i] = 1;
			for (int j = 0; j < n - 1; j++) { // why not n - 2? Because we do
				if (j == n - 2) {
					int a = 0;
					a += a;
				}
				s = 0;
				for (int k = j + 1; k < n; k++) {
					s += A.getEntry(j, k) * y[k];
				}
				for (int k = j + 1; k < n; k++) {
					y[k] -= A.getEntry(j, k) * s;
				}
			}
		}
		/*fprintf("V:%n");
				disp(V);*/
	}

	UBV[0] = U;
	UBV[1] = B;
	UBV[2] = V;

	if (computeUV) {
		/*fprintf("U'U:%n");
				disp(U.transpose().mtimes(U));

				fprintf("V'V:%n");
				disp(V.transpose().mtimes(V));*/

		/*fprintf("UBV':%n");
				disp(U.mtimes(B).mtimes(V.transpose()));*/

		/*disp("B:");
				printMatrix(B);*/
		// IO.saveMatrix(full(B), "SVD-B");
	}

	return UBV;
}

Matrix** SVD::diagonalizeBD(Matrix& B, bool computeUV) {
	int m = B.getRowDimension();
	int n = B.getColumnDimension();
	int len = m >= n ? n : m;
	int idx = 0;

	/*
	 * The bidiagonal matrix B is
	 * s[0] e[0]
	 *      s[1] e[1]
	 *           ...
	 *               s[len - 2] e[len - 2]
	 *                          s[len - 1]
	 */
	double* s = allocateVector(len, 0);
	double* e = allocateVector(len, 0);

	/*double[] pr = ((SparseMatrix) B).getPr();
			int nnz = ((SparseMatrix) B).getNNZ();
			int k = 0;
			while (true) {
				s[idx] = pr[k++];
				if (k == nnz)
					break;
				e[idx] = pr[k++];
				idx++;
			}*/
	for (int i = 0; i < len - 1; i++) {
		s[i] = B.getEntry(i, i);
		e[i] = B.getEntry(i, i + 1);
	}
	s[len - 1] = B.getEntry(len - 1, len - 1);

	/*
	 * B = USV' where U is the left singular vectors,
	 * and V is the right singular vectors.
	 */

	// U': each row of U' is a left singular vector
	double** Ut = null;
	DenseMatrix* UtMatrix = null;
	if (computeUV) {
		UtMatrix = &eye(m, m);
		Ut = UtMatrix->getData();
	}

	// V': each row of V' is a right singular vector
	double** Vt = null;
	DenseMatrix* VtMatrix = null;
	if (computeUV) {
		VtMatrix = &eye(n, n);
		Vt = VtMatrix->getData();
	}

	double* mu = allocate1DArray(len, 0);

	double sigma_min = 0;
	double sigma_max = 0;

	/*
	 * B = IBI'
	 * Therefore, when pre-multiplying B by Givens rotation transform
	 * on i-th and j-th rows, we need to change the i-th and j-th rows
	 * of U'.
	 * B0 = IB0I' = UG'GBVt = (GUt)'BkVt
	 * (Ut)'BVt = (Ut)'BG'GVt = (Lt)'Bk(GVt)
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
	/*int cnt_zero_shift = 0;
			int cnt_shifted = 0;*/
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

		// Estimate the smallest singular value sigma_min and
		// the largest singular value sigma_max of B_hat

		sigma_min = fabs(mu[i_start]);
		for (int j = i_start; j <= i_end; j++) {
			if (sigma_min > fabs(mu[j])) {
				sigma_min = fabs(mu[j]);
			}
		}

		sigma_max = fabs(s[i_start]);
		for (int j = i_start; j <= i_end; j++) {
			if (sigma_max < fabs(s[j])) {
				sigma_max = fabs(s[j]);
			}
		}
		for (int j = i_start; j < i_end; j++) {
			if (sigma_max < fabs(e[j])) {
				sigma_max = fabs(e[j]);
			}
		}

		/*double ss = MIN_VALUE;
		double sss = tol;
		double ssss = max(MIN_VALUE, 0.01);*/

		// fprintf("Iter %d:%n", ind);
		/*if (n * sigma_max < sigma_min * fmax(MIN_VALUE / tol, 0.01)) {
			implicitZeroShiftQR(s, e, Ut, Vt, i_start, i_end, computeUV);
			// cnt_zero_shift++;
		} else {

		 * Standard shifted QR might converge very slowly and return
		 * wrong results, i.e., for the matrix saved in the file named
		 * "SVDInput"!!!

			// standardShiftedQR(s, e, Ut, Vt, i_start, i_end, computeUV);
			implicitZeroShiftQR(s, e, Ut, Vt, i_start, i_end, computeUV);
			// cnt_shifted++;
		}*/
		implicitZeroShiftQR(s, e, Ut, Vt, i_start, i_end, computeUV, m, n);

		if (ind == maxIter) {
			break;
		}

		ind++;

	}
	/*fprintf("cnt_zero_shift: %d%n", cnt_zero_shift);
			fprintf("cnt_shifted: %d%n", cnt_shifted);*/
	// *********************************************************

	// Make sure that all elements of s are nonnegative
	for (int i = 0; i < len; i++) {
		if (s[i] < 0) {
			if (computeUV)
				timesAssign(Ut[i], m, -1);
			s[i] *= -1;
		}
	}

	// Quick sort singular values and singular vectors
	quickSort(s, Ut, Vt, 0, len - 1, "descend", computeUV);

	Matrix** USV = new Matrix*[3];
	if (computeUV) {
		USV[0] = &UtMatrix->transpose();
		delete UtMatrix;
	} else
		USV[0] = null;
	USV[1] = &buildS(s, m, n);
	if (computeUV) {
		USV[2] = &VtMatrix->transpose();
		delete VtMatrix;
	} else
		USV[2] = null;

	delete[] s;
	delete[] e;
	delete[] mu;

	return USV;
}

void SVD::quickSort(double* s, double** Ut, double** Vt, int start, int end, std::string order, bool computeUV) {
	int	i,j;
	double temp;
	i = start;
	j = end;
	temp = s[i];
	double* tempU = computeUV ? Ut[i] : null;
	double* tempV = computeUV ? Vt[i] : null;
	do{
		if (order == "ascend") {
			while((s[j] > temp) && (j > i))
				j--;
		} else if (order == "descend") {
			while((s[j] < temp) && (j > i))
				j--;
		}
		if(j > i){
			s[i] = s[j];
			if (computeUV) {
				Ut[i] = Ut[j];
				Vt[i] = Vt[j];
			}
			i++;
		}
		if (order == "ascend") {
			while((s[i] < temp) && (j > i))
				i++;
		} else if (order == "descend") {
			while((s[i] > temp) && (j > i))
				i++;
		}
		if(j > i){
			s[j] = s[i];
			if (computeUV) {
				Ut[j] = Ut[i];
				Vt[j] = Vt[i];
			}
			j--;
		}
	} while(i != j);
	s[i] = temp;
	if (computeUV) {
		Ut[i] = tempU;
		Vt[i] = tempV;
	}
	i++;
	j--;
	if(start < j)
		quickSort(s, Ut, Vt, start, j, order, computeUV);
	if(i < end)
		quickSort(s, Ut, Vt, i, end, order, computeUV);
}

void SVD::implicitZeroShiftQR(double* s, double* e, double** Ut, double** Vt, int i_start, int i_end, bool computeUV, int m, int n) {

	/*int m = Ut.length;
			int n =Vt.length;
			int len = m >= n ? n : m;*/

	// Matrix Bk = null;

	double oldcs = 1;
	double oldsn = 0;
	double f = s[i_start];
	double g = e[i_start];
	double h = 0;
	double cs = 0, sn = 0, r = 0;
	double t, tt;

	for (int i = i_start; i < i_end; i++) {
		// ROT(f, g, cs, sn, r)
		if (f == 0) {
			cs = 0;
			sn = 1;
			r = g;
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
		if (computeUV) {
			update(cs, sn, Vt[i], Vt[i + 1], n);
		}

		if (i != i_start) { // Note that i != i_start rather than i != 0!!!
			e[i - 1] = oldsn* r;
		}

		/*Bk = buildB(s, e, m, n);
				fprintf("Bk:%n");
				printMatrix(Bk);*/

		f = oldcs * r;
		g = s[i + 1] * sn;
		h = s[i + 1] * cs;

		// ROT(f, g, cs, sn, r)
		if (f == 0) {
			cs = 0;
			sn = 1;
			r = g;
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
		// UPDATE(cs, sn, Ui, Ui+1)
		if (computeUV) {
			update(cs, sn, Ut[i], Ut[i + 1], m);
		}

		s[i] = r;
		f = h;
		g = e[i + 1];
		oldcs = cs;
		oldsn = sn;

		/*Bk = buildB(s, e, m, n);
				fprintf("Bk:%n");
				printMatrix(Bk);*/

	}
	e[i_end - 1] = h * sn;
	s[i_end] = h * cs;

	/*Bk = buildB(s, e, m, n);
			fprintf("Bk:%n");
			printMatrix(Bk);*/
}

Matrix& SVD::buildS(double* s, int m, int n) {
	std::map<std::pair<int, int>, double> map;
	for (int i = 0; i < m; i++) {
		if (i < n)
			map.insert(std::make_pair(std::make_pair(i, i), s[i]));
	}
	return SparseMatrix::createSparseMatrix(map, m, n);
}

void SVD::update(double cs, double sn, double* V1, double* V2, int len) {
	double temp;
	for (int i = 0; i < len; i++) {
		temp = V1[i];
		V1[i] = cs * temp + sn * V2[i];
		V2[i] = -sn * temp + cs * V2[i];
	}
}

