/*
 * LU.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: Mingjie Qian
 */

#include "LU.h"
#include "Printer.h"
#include "DenseVector.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "ArrayOperator.h"
#include "Matlab.h"
#include <iostream>

LU::LU(Matrix& A) {
	Matrix** LUP = run(A);
	L = LUP[0];
	U = LUP[1];
	P = LUP[2];
}

LU::~LU() {
	delete L;
	delete U;
	delete P;
	// disp("LU decomposition is released.");
}

Matrix** LU::run(Matrix& A) {
	int n = A.getRowDimension();
	if (n != A.getColumnDimension()) {
		err("A should be a square matrix.");
		exit(1);
	}
	numRowExchange = 0;
	Matrix** LUP = new Matrix*[3];

	/*
	 * LU = PA
	 */

	if (typeid(A) == typeid(DenseMatrix)) {
		double** L = (new DenseMatrix(n, n, 0))->getData();
		double** AData = ((DenseMatrix&) A.copy()).getData();
		double** P = (new DenseMatrix(n, n, 0))->getData();
		// Initialize P = eye(n)
		for (int i = 0; i < n; i++) {
			P[i][i] = 1;
		}
		for (int i = 0; i < n; i++) {
			// j = argmax_{k \in {i, i+1, ..., n}} |A_{ki}|
			double maxVal = AData[i][i];
			int j = i;
			for (int k = i + 1; k < n; k++) {
				if (maxVal < AData[k][i]) {
					j = k;
					maxVal = AData[k][i];
				}
			}
			if (maxVal == 0) {
				err("Matrix A is singular.");
				LUP[0] = null;
				LUP[1] = null;
				LUP[2] = null;
				return LUP;
			}
			if (j != i) {
				// Swap rows i and j of A, L, and P
				// A: Swap columns from i to n - 1
				// swap(AData[i], AData[j], i, n);
				double* temp = null;
				temp = AData[i];
				AData[i] = AData[j];
				AData[j] = temp;
				// L: Swap columns from 0 to i
				// swap(L[i], L[j], 0, i + 1);
				temp = L[i];
				L[i] = L[j];
				L[j] = temp;
				// P: Swap non-zero columns
				/*double[] P_i = P[i];
					double[] P_j = P[j];
					for (int k = 0; k < n; k++) {
						if (P_i[k] == 1) {
							P_i[k] = 0;
							P_j[k] = 1;
						} else if (P_j[k] == 1) {
							P_i[k] = 1;
							P_j[k] = 0;
						}
					}*/
				// swap(P[i], P[j], 0, n);
				temp = P[i];
				P[i] = P[j];
				P[j] = temp;
				numRowExchange++;
			}
			// Elimination step
			L[i][i] = 1;
			double* A_i = AData[i];
			double L_ki = 0;
			for (int k = i + 1; k < n; k++) {
				// L[k][i] = AData[k][i] / AData[i][i];
				L_ki = AData[k][i] / maxVal;
				L[k][i] = L_ki;
				double* A_k = AData[k];
				// AData[k][i] = 0;
				A_k[i] = 0;
				for (int l = i + 1; l < n; l++) {
					// AData[k][l] -= L[k][i] * AData[i][l];
					// AData[k][l] -= L_ki * A_i[l];
					A_k[l] -= L_ki * A_i[l];
				}
			}
		}
		LUP[0] = new DenseMatrix(L, n, n);
		LUP[1] = new DenseMatrix(AData, n, n);
		LUP[2] = new DenseMatrix(P, n, n);
	} else {
		Vector** LVs = sparseMatrix2SparseRowVectors(*new SparseMatrix(n, n));
		Vector** AVs = sparseMatrix2SparseRowVectors(A);
		Vector** PVs = sparseMatrix2SparseRowVectors(*new SparseMatrix(n, n));

		// Initialize P = eye(n)
		for (int i = 0; i < n; i++) {
			PVs[i]->set(i, 1);
		}
		for (int i = 0; i < n; i++) {
			// j = argmax_{k \in {i, i+1, ..., n}} |A_{ki}|
			double maxVal = AVs[i]->get(i);
			int j = i;
			for (int k = i + 1; k < n; k++) {
				double v = AVs[k]->get(i);
				if (maxVal < v) {
					j = k;
					maxVal = v;
				}
			}
			if (maxVal == 0) {
				err("Matrix A is singular.");
				LUP[0] = null;
				LUP[1] = null;
				LUP[2] = null;
				return LUP;
			}
			if (j != i) {
				// Swap rows i and j of A, L, and P
				// A: Swap columns from i to n - 1
				// swap(AData[i], AData[j], i, n);
				Vector* temp = null;
				temp = AVs[i];
				AVs[i] = AVs[j];
				AVs[j] = temp;
				// L: Swap columns from 0 to i
				// swap(L[i], L[j], 0, i + 1);
				temp = LVs[i];
				LVs[i] = LVs[j];
				LVs[j] = temp;
				// P: Swap non-zero columns
				/*double[] P_i = P[i];
					double[] P_j = P[j];
					for (int k = 0; k < n; k++) {
						if (P_i[k] == 1) {
							P_i[k] = 0;
							P_j[k] = 1;
						} else if (P_j[k] == 1) {
							P_i[k] = 1;
							P_j[k] = 0;
						}
					}*/
				// swap(P[i], P[j], 0, n);
				temp = PVs[i];
				PVs[i] = PVs[j];
				PVs[j] = temp;
				numRowExchange++;
			}
			// Elimination step
			LVs[i]->set(i, 1);
			Vector* A_i = AVs[i];
			double L_ki = 0;
			for (int k = i + 1; k < n; k++) {
				// L[k][i] = AData[k][i] / AData[i][i];
				L_ki = AVs[k]->get(i) / maxVal;
				// L[k][i] = L_ki;
				LVs[k]->set(i, L_ki);
				Vector* A_k = AVs[k];
				// AData[k][i] = 0;
				A_k->set(i, 0);
				for (int l = i + 1; l < n; l++) {
					// AData[k][l] -= L[k][i] * AData[i][l];
					// AData[k][l] -= L_ki * A_i[l];
					A_k->set(l, A_k->get(l) - L_ki * A_i->get(l));
				}
			}
		}
		LUP[0] = &sparseRowVectors2SparseMatrix(LVs, n);
		LUP[1] = &sparseRowVectors2SparseMatrix(AVs, n);
		LUP[2] = &sparseRowVectors2SparseMatrix(PVs, n);
		for (int i = 0; i < n; i++) {
			delete LVs[i];
			delete AVs[i];
			delete PVs[i];
		}
		delete[] LVs;
		delete[] AVs;
		delete[] PVs;
	}
	return LUP;
}

Matrix** LU::decompose(Matrix& A) {
	int n = A.getRowDimension();
	if (n != A.getColumnDimension()) {
		err("A should be a square matrix.");
		exit(1);
	}
	// numRowExchange = 0;
	Matrix** LUP = new Matrix*[3];

	/*
	 * LU = PA
	 */

	if (typeid(A) == typeid(DenseMatrix)) {
		double** L = (new DenseMatrix(n, n, 0))->getData();
		double** AData = ((DenseMatrix&) A.copy()).getData();
		double** P = (new DenseMatrix(n, n, 0))->getData();
		// Initialize P = eye(n)
		for (int i = 0; i < n; i++) {
			P[i][i] = 1;
		}
		for (int i = 0; i < n; i++) {
			// j = argmax_{k \in {i, i+1, ..., n}} |A_{ki}|
			double maxVal = AData[i][i];
			int j = i;
			for (int k = i + 1; k < n; k++) {
				if (maxVal < AData[k][i]) {
					j = k;
					maxVal = AData[k][i];
				}
			}
			if (maxVal == 0) {
				err("Matrix A is singular.");
				LUP[0] = null;
				LUP[1] = null;
				LUP[2] = null;
				return LUP;
			}
			if (j != i) {
				// Swap rows i and j of A, L, and P
				// A: Swap columns from i to n - 1
				// swap(AData[i], AData[j], i, n);
				double* temp = null;
				temp = AData[i];
				AData[i] = AData[j];
				AData[j] = temp;
				// L: Swap columns from 0 to i
				// swap(L[i], L[j], 0, i + 1);
				temp = L[i];
				L[i] = L[j];
				L[j] = temp;
				// P: Swap non-zero columns
				/*double[] P_i = P[i];
					double[] P_j = P[j];
					for (int k = 0; k < n; k++) {
						if (P_i[k] == 1) {
							P_i[k] = 0;
							P_j[k] = 1;
						} else if (P_j[k] == 1) {
							P_i[k] = 1;
							P_j[k] = 0;
						}
					}*/
				// swap(P[i], P[j], 0, n);
				temp = P[i];
				P[i] = P[j];
				P[j] = temp;
				// numRowExchange++;
			}
			// Elimination step
			L[i][i] = 1;
			double* A_i = AData[i];
			double L_ki = 0;
			for (int k = i + 1; k < n; k++) {
				// L[k][i] = AData[k][i] / AData[i][i];
				L_ki = AData[k][i] / maxVal;
				L[k][i] = L_ki;
				double* A_k = AData[k];
				// AData[k][i] = 0;
				A_k[i] = 0;
				for (int l = i + 1; l < n; l++) {
					// AData[k][l] -= L[k][i] * AData[i][l];
					// AData[k][l] -= L_ki * A_i[l];
					A_k[l] -= L_ki * A_i[l];
				}
			}
		}
		LUP[0] = new DenseMatrix(L, n, n);
		LUP[1] = new DenseMatrix(AData, n, n);
		LUP[2] = new DenseMatrix(P, n, n);
	} else {
		Vector** LVs = sparseMatrix2SparseRowVectors(*new SparseMatrix(n, n));
		Vector** AVs = sparseMatrix2SparseRowVectors(A);
		Vector** PVs = sparseMatrix2SparseRowVectors(*new SparseMatrix(n, n));

		// Initialize P = eye(n)
		for (int i = 0; i < n; i++) {
			PVs[i]->set(i, 1);
		}
		for (int i = 0; i < n; i++) {
			// j = argmax_{k \in {i, i+1, ..., n}} |A_{ki}|
			double maxVal = AVs[i]->get(i);
			int j = i;
			for (int k = i + 1; k < n; k++) {
				double v = AVs[k]->get(i);
				if (maxVal < v) {
					j = k;
					maxVal = v;
				}
			}
			if (maxVal == 0) {
				err("Matrix A is singular.");
				LUP[0] = null;
				LUP[1] = null;
				LUP[2] = null;
				return LUP;
			}
			if (j != i) {
				// Swap rows i and j of A, L, and P
				// A: Swap columns from i to n - 1
				// swap(AData[i], AData[j], i, n);
				Vector* temp = null;
				temp = AVs[i];
				AVs[i] = AVs[j];
				AVs[j] = temp;
				// L: Swap columns from 0 to i
				// swap(L[i], L[j], 0, i + 1);
				temp = LVs[i];
				LVs[i] = LVs[j];
				LVs[j] = temp;
				// P: Swap non-zero columns
				/*double[] P_i = P[i];
					double[] P_j = P[j];
					for (int k = 0; k < n; k++) {
						if (P_i[k] == 1) {
							P_i[k] = 0;
							P_j[k] = 1;
						} else if (P_j[k] == 1) {
							P_i[k] = 1;
							P_j[k] = 0;
						}
					}*/
				// swap(P[i], P[j], 0, n);
				temp = PVs[i];
				PVs[i] = PVs[j];
				PVs[j] = temp;
				// numRowExchange++;
			}
			// Elimination step
			LVs[i]->set(i, 1);
			Vector* A_i = AVs[i];
			double L_ki = 0;
			for (int k = i + 1; k < n; k++) {
				// L[k][i] = AData[k][i] / AData[i][i];
				L_ki = AVs[k]->get(i) / maxVal;
				// L[k][i] = L_ki;
				LVs[k]->set(i, L_ki);
				Vector* A_k = AVs[k];
				// AData[k][i] = 0;
				A_k->set(i, 0);
				for (int l = i + 1; l < n; l++) {
					// AData[k][l] -= L[k][i] * AData[i][l];
					// AData[k][l] -= L_ki * A_i[l];
					A_k->set(l, A_k->get(l) - L_ki * A_i->get(l));
				}
			}
		}
		LUP[0] = &sparseRowVectors2SparseMatrix(LVs, n);
		LUP[1] = &sparseRowVectors2SparseMatrix(AVs, n);
		LUP[2] = &sparseRowVectors2SparseMatrix(PVs, n);
		for (int i = 0; i < n; i++) {
			delete LVs[i];
			delete AVs[i];
			delete PVs[i];
		}
		delete[] LVs;
		delete[] AVs;
		delete[] PVs;
	}
	return LUP;
}

Vector& LU::solve(Vector& b) {
	Vector* res = null;
	if (typeid(*L) == typeid(DenseMatrix)) {
		// PAx = Pb = d
		// LUx = d
		DenseVector& dVector = full(P->operate(b));
		double* d = dVector.getPr();
		// Ly = d
		int n = L->getColumnDimension();
		double** LData = ((DenseMatrix&) full(*L)).getData();
		double* LData_i = null;
		double* y = new double[n];
		double v = 0;
		for (int i = 0; i < n; i++) {
			v = d[i];
			LData_i = LData[i];
			for (int k = 0; k < i; k++) {
				v -= LData_i[k] * y[k];
			}
			y[i] = v;
		}
		// Ux = y
		double** UData = full(*U).getData();
		double* UData_i = null;
		double* x = new double[n];
		v = 0;
		for (int i = n - 1; i > -1; i--) {
			UData_i = UData[i];
			v = y[i];
			for (int k = n - 1; k > i; k--) {
				v -= UData_i[k] * x[k];
			}
			x[i] = v / UData_i[i];
		}
		res = new DenseVector(x, n);
		delete &dVector;
		delete[] y;
	} else {
		// PAx = Pb = d
		// LUx = d
		DenseVector& dVector = full(P->operate(b));
		double* d = dVector.getPr();
		// Ly = d
		int n = L->getColumnDimension();
		// double[][] LData = ((DenseMatrix) full(L)).getData();
		Vector** LVs = sparseMatrix2SparseRowVectors(*L);
		Vector* LRow_i = null;
		double* y = new double[n];
		double v = 0;
		for (int i = 0; i < n; i++) {
			v = d[i];
			LRow_i = LVs[i];
			int* ir = ((SparseVector*) LRow_i)->getIr();
			double* pr = ((SparseVector*) LRow_i)->getPr();
			int nnz = ((SparseVector*) LRow_i)->getNNZ();
			int idx = -1;
			for (int k = 0; k < nnz; k++) {
				idx = ir[k];
				if (idx >= i) {
					break;
				}
				v -= pr[k] * y[idx];
			}
			/*for (int k = 0; k < i; k++) {
						v -= LRow_i[k] * y[k];
					}*/
			y[i] = v;
		}
		// Ux = y
		// double[][] UData = ((DenseMatrix) full(U)).getData();
		Vector** UVs = sparseMatrix2SparseRowVectors(*U);
		Vector* URow_i = null;
		double* x = new double[n];
		v = 0;
		for (int i = n - 1; i > -1; i--) {
			URow_i = UVs[i];
			v = y[i];
			int* ir = ((SparseVector*) URow_i)->getIr();
			double* pr = ((SparseVector*) URow_i)->getPr();
			int nnz = ((SparseVector*) URow_i)->getNNZ();
			int idx = -1;
			int k = nnz - 1;
			while (true) {
				idx = ir[k];
				if (idx <= i) {
					break;
				}
				v -= pr[k] * x[idx];
				k--;
			}
			/*for (int k = n - 1; k > i; k--) {
						v -= URow_i[k] * x[k];
					}*/
			x[i] = v / URow_i->get(i);
		}
		res = new DenseVector(x, n);
		delete &dVector;
		delete[] y;
		for (int i = 0; i < n; i++) {
			delete LVs[i];
			delete UVs[i];
		}
		delete[] LVs;
		delete[] UVs;
	}
	return *res;
}

Matrix& LU::solve(Matrix& B) {
	Matrix* res = null;
	if (typeid(*L) == typeid(DenseMatrix)) {
		// PAX = PB = D
		// LUX = D
		DenseMatrix& DMatrix = full(P->mtimes(B));
		double** D = DMatrix.getData();
		double* DRow_i = null;
		// LY = D
		int n = L->getColumnDimension();
		double** LData = full(*L).getData();
		double* LRow_i = null;
		double** Y = allocate2DArray(n, B.getColumnDimension(), 0);
		double* YRow_i = null;

		double v = 0;
		for (int i = 0; i < n; i++) {
			LRow_i = LData[i];
			DRow_i = D[i];
			YRow_i = Y[i];
			for (int j = 0; j < B.getColumnDimension(); j++) {
				v = DRow_i[j];
				for (int k = 0; k < i; k++) {
					v -= LRow_i[k] * Y[k][j];
				}
				YRow_i[j] = v;
			}
		}
		// UX = Y
		double** UData = full(*U).getData();
		double* URow_i = null;
		double** X = allocate2DArray(n, B.getColumnDimension(), 0);
		double* XRow_i = null;
		for (int i = n - 1; i > -1; i--) {
			URow_i = UData[i];
			YRow_i = Y[i];
			XRow_i = X[i];
			for (int j = 0; j < B.getColumnDimension(); j++) {
				v = YRow_i[j];
				for (int k = n - 1; k > i; k--) {
					v -= URow_i[k] * X[k][j];
				}
				XRow_i[j] = v / URow_i[i];
			}
		}
		res = new DenseMatrix(X, n, B.getColumnDimension());
		delete &DMatrix;
		for (int i = 0; i < n; i++) {
			delete[] Y[i];
		}
		delete[] Y;
	} else {
		// PAX = PB = D
		// LUX = D
		DenseMatrix& DMatrix = full(P->mtimes(B));
		disp(DMatrix);
		std::cout << std::flush;
		double** D = DMatrix.getData();
		double* DRow_i = null;
		// LY = D
		int n = L->getColumnDimension();
		Vector** LVs = sparseMatrix2SparseRowVectors(*L);
		Vector* LRow_i = null;
		double** Y = allocate2DArray(n, B.getColumnDimension(), 0);
		double* YRow_i = null;

		double v = 0;
		for (int i = 0; i < n; i++) {
			LRow_i = LVs[i];
			int* ir = ((SparseVector*) LRow_i)->getIr();
			double* pr = ((SparseVector*) LRow_i)->getPr();
			int nnz = ((SparseVector*) LRow_i)->getNNZ();
			int idx = -1;
			DRow_i = D[i];
			YRow_i = Y[i];
			for (int j = 0; j < B.getColumnDimension(); j++) {
				v = DRow_i[j];
				for (int k = 0; k < nnz; k++) {
					idx = ir[k];
					if (idx >= i) {
						break;
					}
					v -= pr[k] * Y[idx][j];
				}
				/*for (int k = 0; k < i; k++) {
							v -= LRow_i[k] * y[k];
						}*/
				YRow_i[j] = v;
			}
		}
		delete &DMatrix;
		// UX = Y
		Vector** UVs = sparseMatrix2SparseRowVectors(*U);
		Vector* URow_i = null;
		double** X = allocate2DArray(n, B.getColumnDimension(), 0);
		double* XRow_i = null;

		for (int i = n - 1; i > -1; i--) {
			URow_i = UVs[i];
			int* ir = ((SparseVector*) URow_i)->getIr();
			double* pr = ((SparseVector*) URow_i)->getPr();
			int nnz = ((SparseVector*) URow_i)->getNNZ();
			int idx = -1;
			YRow_i = Y[i];
			XRow_i = X[i];
			for (int j = 0; j < B.getColumnDimension(); j++) {
				v = YRow_i[j];
				int k = nnz - 1;
				while (true) {
					idx = ir[k];
					if (idx <= i) {
						break;
					}
					v -= pr[k] * X[idx][j];
					k--;
				}
				/*for (int k = n - 1; k > i; k--) {
							v -= URow_i[k] * x[k];
						}*/
				XRow_i[j] = v / URow_i->get(i);
			}
		}
		res = new DenseMatrix(X, n, B.getColumnDimension());
		// Release Y, LVs, and UVs
		for (int i = 0; i < n; i++) {
			delete[] Y[i];
			delete LVs[i];
			delete UVs[i];
		}
		delete[] Y;
		delete[] LVs;
		delete[] UVs;
	}
	return *res;
}

Matrix& LU::inverse() {
	if (U == null) {
		err("LU decomposition has not been computed yet.");
		exit(1);
	}
	/*
	 * When computing A^{-1}, A * A^{-1} = I.
	 * Thus we have A^{-1} is the solutions for A * X = [e_1, e_2, ..., e_n].
	 */
	int n = L->getColumnDimension();
	double** AInverseTransposeData = new double*[n];
	double** eye = new double*[n];
	for (int i = 0; i < n; i++) {
		eye[i] = allocateVector(n, 0);
		eye[i][i] = 1;
	}
	for (int i = 0; i < n; i++) {
		AInverseTransposeData[i] = full(solve(*new DenseVector(eye[i], n))).getPr();
	}
	Matrix* temp = null;
	temp = new DenseMatrix(AInverseTransposeData, n, n);
	Matrix* res = null;
	res = &temp->transpose();
	delete temp;
	return *res;
}

double LU::det() {
	if (U == null) {
		// err("LU decomposition has not been computed yet.");
		// exit(1);
		return 0;
	}
	double s = 1;
	for (int k = 0; k < U->getColumnDimension(); k++) {
		s *= U->getEntry(k, k);
		if (s == 0) {
			break;
		}
	}
	return numRowExchange % 2 == 0 ? s : -s;
}
