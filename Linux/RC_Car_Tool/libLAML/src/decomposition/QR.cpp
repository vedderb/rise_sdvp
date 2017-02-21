/*
 * QR.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: Mingjie Qian
 */

#include "QR.h"
#include "Matlab.h"
#include "ArrayOperator.h"
#include "Printer.h"
#include <cmath>

QR::QR(Matrix& A) {
	Matrix** QRP = run(A);
	Q = QRP[0];
	R = QRP[1];
	P = QRP[2];
}

QR::~QR() {
	delete Q;
	delete R;
	delete P;
	// disp("QR decomposition is released.");
}

Matrix** QR::run(Matrix& A) {
	return decompose(A);
}

Matrix** QR::decompose(Matrix& M) {
	Matrix* A = &M.copy();
	int m = A->getRowDimension();
	int n = A->getColumnDimension();
	Matrix** QRP = new Matrix*[3];
	double* d = allocateVector(n, 0);
	Vector** PVs = sparseMatrix2SparseColumnVectors(*new SparseMatrix(n, n));
	// Initialize P = eye(n)
	for (int i = 0; i < n; i++) {
		PVs[i]->set(i, 1);
	}
	if (typeid(*A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix*) A)->getData();
		double* c = allocateVector(n, 0);

		for (int j = 0; j < n; j++) {
			if (j >= m) {
				break;
			}
			for (int jj = j; jj < n; jj++) {
				double s = 0;
				for (int i = j; i < m; i++) {
					s += pow(AData[i][jj], 2);
				}
				c[jj] = s;
			}
			int i = j;
			double maxVal = c[j];
			for (int k = j + 1; k < n; k++) {
				if (maxVal < c[k]) {
					i = k;
					maxVal = c[k];
				}
			}
			if (maxVal == 0) {
				disp("Rank(A) < n.");
				QRP[0] = &computeQ(*A);
				QRP[1] = &computeR(*A, d);
				QRP[2] = &sparseRowVectors2SparseMatrix(PVs, n);
				return QRP;
			}
			if (i != j) {
				// Swap A(:, i) <=> A(:, j)
				double temp = 0;
				for (int k = 0; k < m; k++) {
					temp = AData[k][i];
					AData[k][i] = AData[k][j];
					AData[k][j] = temp;
				}
				// Swap c[i] <=> c[j]
				temp = c[i];
				c[i] = c[j];
				c[j] = temp;
				// Swap P(:, i) <=> P(:, j)
				Vector* V = PVs[i];
				PVs[i] = PVs[j];
				PVs[j] = V;
			}
			// Compute the norm of A(j:m, j), which is always sqrt(c[j])
			// since the array c will be updated
			double s = sqrt(c[j]);
			d[j] = AData[j][j] > 0 ? -s : s;
			double r = sqrt(s * (s + fabs(AData[j][j])));
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
				// c[k] -= pow(AData[j][k], 2);
			}
			/*fprintf("Processed A, j = %d:\n", j);
			printMatrix(*A);*/
		}
		delete[] c;
	} else {
		Vector** AVs = sparseMatrix2SparseColumnVectors(*A);
		double* c = allocateVector(n, 0);

		for (int j = 0; j < n; j++) {
			if (j >= m) {
				break;
			}
			for (int jj = j; jj < n; jj++) {
				SparseVector* A_j = (SparseVector*) AVs[jj];
				double* pr = A_j->getPr();
				int* ir = A_j->getIr();
				int nnz = A_j->getNNZ();
				double s = 0;
				int idx = -1;
				for (int k = 0; k < nnz; k++) {
					idx = ir[k];
					if (idx >= j)
						s += pow(pr[k], 2);
				}
				c[jj] = s;
			}
			int i = j;
			double maxVal = c[j];
			for (int k = j + 1; k < n; k++) {
				if (maxVal < c[k]) {
					i = k;
					maxVal = c[k];
				}
			}
			if (maxVal == 0) {
				disp("Rank(A) < n.");
				QRP[0] = &computeQ(*A);
				QRP[1] = &computeR(*A, d);
				QRP[2] = &sparseRowVectors2SparseMatrix(PVs, n);
				return QRP;
			}
			if (i != j) {
				// Swap A(:, i) <=> A(:, j)
				double temp = 0;
				Vector* V = null;
				V = AVs[i];
				AVs[i] = AVs[j];
				AVs[j] = V;
				// Swap c[i] <=> c[j]
				temp = c[i];
				c[i] = c[j];
				c[j] = temp;
				// Swap P(:, i) <=> P(:, j)
				V = PVs[i];
				PVs[i] = PVs[j];
				PVs[j] = V;
			}
			// Compute the norm of A(j:m, j), which is always sqrt(c[j])
			// since the array c will be updated
			double s = sqrt(c[j]);
			SparseVector* A_j = (SparseVector*) AVs[j];
			double Ajj = A_j->get(j);
			d[j] = Ajj > 0 ? -s : s;
			double r = sqrt(s * (s + fabs(Ajj)));
			A_j->set(j, Ajj - d[j]);
			// AData[j][j] -= d[j];
			int* ir = A_j->getIr();
			double* pr = A_j->getPr();
			int nnz = A_j->getNNZ();
			int idx = 0;
			for (int k = 0; k < nnz; k++) {
				idx = ir[k];
				if (idx < j)
					continue;
				else {
					pr[k] /= r;
				}
			}
			/*for (int k = j; k < m; k++) {
						AData[k][j] /= r;
					}*/
			for (int k = j + 1; k < n; k++) {
				SparseVector* A_k = (SparseVector*) AVs[k];
				s = 0;
				int* ir2 = A_k->getIr();
				double* pr2 = A_k->getPr();
				int nnz2 = A_k->getNNZ();
				if (nnz != 0 && nnz2 != 0) {
					int k1 = 0;
					int k2 = 0;
					int r1 = 0;
					int r2 = 0;
					double v = 0;
					while (k1 < nnz && k2 < nnz2) {
						r1 = ir[k1];
						r2 = ir2[k2];
						if (r1 < r2) {
							k1++;
						} else if (r1 == r2) {
							v = pr[k1] * pr2[k2];
							k1++;
							k2++;
							if (r1 >= j)
								s += v;
						} else {
							k2++;
						}
					}
				}
				/*s = 0;
						for (int t = j; t < m; t++) {
							s += AData[t][j] * AData[t][k];
						}*/
				for (int t = j; t < m; t++) {
					A_k->set(t, A_k->get(t) - s * A_j->get(t));
				}
				/*for (int t = j; t < m; t++) {
							AData[t][k] -= s * AData[t][j];
						}*/
				// c[k] -= pow(A_k->get(j), 2);
			}
		}
		A = &sparseColumnVectors2SparseMatrix(AVs, n);
		delete[] c;
		for (int j = 0; j < n; j++) {
			delete AVs[j];
		}
		delete[] AVs;
	}

	/*disp("Processed A:");
	printMatrix(*A);*/

	QRP[0] = &computeQ(*A);
	QRP[1] = &computeR(*A, d);
	QRP[2] = &sparseColumnVectors2SparseMatrix(PVs, n);

	for (int j = 0; j < n; j++) {
		delete PVs[j];
	}
	delete[] PVs;

	return QRP;

}

Matrix& QR::computeQ(Matrix& A) {
	int m = A.getRowDimension();
	int n = A.getColumnDimension();
	DenseMatrix* res = new DenseMatrix(m, m, 0);
	double** Q = res->getData();
	double s = 0;
	double* y = null;
	for (int i = 0; i < m; i++) {
		// Compute Q^T * e_i
		y = Q[i];
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
	return *res;
}

Matrix& QR::computeR(Matrix& A, double* d) {
	int m = A.getRowDimension();
	int n = A.getColumnDimension();
	Matrix* R = null;
	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		for (int i = 0; i < m; i++) {
			double* A_i = AData[i];
			if (i < n)
				A_i[i] = d[i];
			int len = i < n ? i : n;
			for (int j = 0; j < len; j++) {
				A_i[j] = 0;
			}
		}
		R = &A;
	} else {
		std::map<std::pair<int, int>, double> map;
		for (int i = 0; i < m; i++) {
			if (i < n)
				map.insert(std::make_pair(std::make_pair(i, i), d[i]));
			for (int j = i + 1; j < n; j++) {
				map.insert(std::make_pair(std::make_pair(i, j), A.getEntry(i, j)));
			}
		}
		R = &SparseMatrix::createSparseMatrix(map, m, n);
	}
	return *R;
}

Vector& QR::solve(Vector& b) {
	// AP = QR
	// Ax = b
	// APP'x = b
	// QRP'x = b
	// Ry = Q'b = d
	double* d = full(b.operate(*Q)).getPr();

	// y = R \ d
	int rank = 0;
	int m = R->getRowDimension();
	int n = R->getColumnDimension();
	int len = m < n ? m : n;
	for (int i = 0; i < len; i++) {
		if (R->getEntry(i, i) == 0) {
			rank = i;
			break;
		} else {
			rank++;
		}
	}
	double* y = allocate1DArray(n, 0);

	if (typeid(*R) == typeid(DenseMatrix)) {
		double** RData = ((DenseMatrix*) R)->getData();
		double* RData_i = null;
		double v = 0;
		for (int i = rank - 1; i > -1; i--) {
			RData_i = RData[i];
			v = d[i];
			for (int k = n - 1; k > i; k--) {
				v -= RData_i[k] * y[k];
			}
			y[i] = v / RData_i[i];
		}

	} else {
		Vector** RVs = sparseMatrix2SparseRowVectors(*R);
		Vector* RRow_i = null;
		double v = 0;
		for (int i = rank - 1; i > -1; i--) {
			RRow_i = RVs[i];
			v = d[i];
			int* ir = ((SparseVector*) RRow_i)->getIr();
			double* pr = ((SparseVector*) RRow_i)->getPr();
			int nnz = ((SparseVector*) RRow_i)->getNNZ();
			int idx = -1;
			int k = nnz - 1;
			while (true) {
				idx = ir[k];
				if (idx <= i) {
					break;
				}
				v -= pr[k] * y[idx];
				k--;
			}
			/*for (int k = n - 1; k > i; k--) {
						v -= URow_i[k] * x[k];
					}*/
			y[i] = v / RRow_i->get(i);
		}
	}

	// x = Py
	Vector& x = P->operate(*new DenseVector(y, n));
	return x;
}

Matrix& QR::solve(Matrix& B) {
	// AP = QR
	// Ax = B
	// APP'X = B
	// QRP'X = B
	// RY = Q'B = D
	Matrix& QT = Q->transpose();
	double** D = full(QT.mtimes(B)).getData();
	delete &QT;
	double* DRow_i = null;

	// Y = R \ D
	int rank = 0;
	int m = R->getRowDimension();
	int n = R->getColumnDimension();
	int len = m < n ? m : n;
	for (int i = 0; i < len; i++) {
		if (R->getEntry(i, i) == 0) {
			rank = i;
			break;
		} else {
			rank++;
		}
	}
	double** Y = allocate2DArray(n, B.getColumnDimension(), 0);
	double* YRow_i = null;

	if (typeid(*R) == typeid(DenseMatrix)) {
		double** RData = ((DenseMatrix*) R)->getData();
		double* RData_i = null;
		double v = 0;

		for (int i = rank - 1; i > -1; i--) {
			RData_i = RData[i];
			DRow_i = D[i];
			YRow_i = Y[i];
			for (int j = 0; j < B.getColumnDimension(); j++) {
				v = DRow_i[j];
				for (int k = n - 1; k > i; k--) {
					v -= RData_i[k] * Y[k][j];
				}
				YRow_i[j] = v / RData_i[i];
			}
		}
	} else {
		Vector** RVs = sparseMatrix2SparseRowVectors(*R);
		Vector* RRow_i = null;
		double v = 0;
		for (int i = rank - 1; i > -1; i--) {
			RRow_i = RVs[i];
			DRow_i = D[i];
			YRow_i = Y[i];
			for (int j = 0; j < B.getColumnDimension(); j++) {
				v = DRow_i[j];
				int* ir = ((SparseVector*) RRow_i)->getIr();
				double* pr = ((SparseVector*) RRow_i)->getPr();
				int nnz = ((SparseVector*) RRow_i)->getNNZ();
				int idx = -1;
				int k = nnz - 1;
				while (true) {
					idx = ir[k];
					if (idx <= i) {
						break;
					}
					v -= pr[k] * Y[idx][j];
					k--;
				}
				YRow_i[j] = v / RRow_i->get(i);
			}
		}
	}

	// X = PY
	Matrix& X = P->mtimes(*new DenseMatrix(Y, n, B.getColumnDimension()));
	return X;
}
