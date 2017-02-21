/*
 * InPlaceOperator.cpp
 *
 *  Created on: Feb 20, 2014
 *      Author: Mingjie Qian
 */

#include "InPlaceOperator.h"
#include "Printer.h"
#include "ArrayOperator.h"
#include "SparseMatrix.h"
#include "DenseMatrix.h"
#include "Matlab.h"
#include <typeinfo>

/**
 * res = A * V.
 *
 * @param res
 * @param A
 * @param V
 */
void operate(Vector& res, Matrix& A, Vector& V) {
	int dim = V.getDim();
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (N != dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(DenseVector)) {
		double* resPr = ((DenseVector&) res).getPr();
		if (typeid(A) == typeid(DenseMatrix)) {
			double** data = ((DenseMatrix&) A).getData();
			if (typeid(V) == typeid(DenseVector)) {
				// ArrayOperator.operate(resPr, data, ((DenseVector&) V).getPr());
				double* VPr = ((DenseVector&) V).getPr();
				double s = 0;
				for (int i = 0; i < M; i++) {
					double* ARow = data[i];
					s = 0;
					for (int j = 0; j < N; j++) {
						s += ARow[j] * VPr[j];
					}
					resPr[i] = s;
				}
			} else {
				int* ir = ((SparseVector&) V).getIr();
				double* pr = ((SparseVector&) V).getPr();
				int nnz = ((SparseVector&) V).getNNZ();
				int idx = 0;
				double* row_i = null;
				for (int i = 0; i < M; i++) {
					row_i = data[i];
					double s = 0;
					for (int k = 0; k < nnz; k++) {
						idx = ir[k];
						s += row_i[idx] * pr[k];
					}
					resPr[i] = s;
				}
			}
		} else {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			if (typeid(V) == typeid(DenseVector)) {
				double* VPr = ((DenseVector&) V).getPr();
				double s = 0;
				int c = 0;
				for (int r = 0; r < M; r++) {
					s = 0;
					for (int k = jr[r]; k < jr[r + 1]; k++) {
						c = ic[k];
						s += pr[valCSRIndices[k]] * VPr[c];
					}
					resPr[r] = s;
				}
			} else {
				int* ir = ((SparseVector&) V).getIr();
				double* VPr = ((SparseVector&) V).getPr();
				int nnz = ((SparseVector&) V).getNNZ();
				double s = 0;
				int kl = 0;
				int kr = 0;
				int cl = 0;
				int rr = 0;
				for (int i = 0; i < M; i++) {
					kl = jr[i];
					kr = 0;
					s = 0;
					while (true) {
						if (kl >= jr[i + 1] || kr >= nnz) {
							break;
						}
						cl = ic[kl];
						rr = ir[kr];
						if (cl < rr) {
							kl++;
						} else if (cl > rr) {
							kr++;
						} else {
							s += pr[valCSRIndices[kl]] * VPr[kr];
							kl++;
							kr++;
						}
					}
					resPr[i] = s;
				}
			}
		}
	} else {
		err("Sparse vector is not supported for res.");
		exit(1);
	}
}

/**
 * res' = V' * A.
 *
 * @param res
 * @param V
 * @param A
 */
void operate(Vector& res, Vector& V, Matrix& A) {
	int dim = V.getDim();
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (M != dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(DenseVector)) {
		double* resPr = ((DenseVector&) res).getPr();
		if (typeid(A) == typeid(DenseMatrix)) {
			clear(resPr, N);
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			if (typeid(V) == typeid(DenseVector)) {
				double* pr = ((DenseVector&) V).getPr();
				double v = 0;
				for (int i = 0; i < M; i++) {
					ARow = AData[i];
					v = pr[i];
					for (int j = 0; j < N; j++) {
						resPr[j] += v * ARow[j];
					}
				}
			} else {
				int* ir = ((SparseVector&) V).getIr();
				double* pr = ((SparseVector&) V).getPr();
				int nnz = ((SparseVector&) V).getNNZ();
				double v = 0;
				for (int k = 0; k < nnz; k++) {
					int i = ir[k];
					ARow = AData[i];
					v = pr[k];
					for (int j = 0; j < N; j++) {
						resPr[j] += v * ARow[j];
					}
				}
			}
		} else {
			int* ir = ((SparseMatrix&) A).getIr();
			int* jc = ((SparseMatrix&) A).getJc();
			double* pr = ((SparseMatrix&) A).getPr();
			if (typeid(V) == typeid(DenseVector)) {
				clear(resPr, N);
				double* VPr = ((DenseVector&) V).getPr();
				for (int j = 0; j < N; j++) {
					for (int k = jc[j]; k < jc[j + 1]; k++) {
						resPr[j] += VPr[ir[k]] * pr[k];
					}
				}
			} else {
				int* VIr = ((SparseVector&) V).getIr();
				double* VPr = ((SparseVector&) V).getPr();
				int nnz = ((SparseVector&) V).getNNZ();
				double s = 0;
				int k1 = 0;
				int k2 = 0;
				int c = 0;
				int r = 0;
				for (int j = 0; j < N; j++) {
					k1 = 0;
					k2 = jc[j];
					s = 0;
					while (true) {
						if (k2 >= jc[j + 1] || k1 >= nnz) {
							break;
						}
						c = VIr[k1];
						r = ir[k2];
						if (r < c) {
							k2++;
						} else if (r > c) {
							k1++;
						} else {
							s += VPr[k1] * pr[k2];
							k1++;
							k2++;
						}
					}
					resPr[j] = s;
				}
			}
		}
	} else {
		err("Sparse vector is not supported for res.");
		exit(1);
	}
}

/**
 * res = abs(A).
 *
 * @param res
 * @param A
 */
void abs(Matrix& res, Matrix& A) {
	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < nRow; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < nCol; j++) {
					resRow[j] = fabs(ARow[j]);
				}
			}
		} else {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < nRow; i++) {
				resRow = resData[i];
				clear(resRow, nCol);
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					resRow[ic[k]] = fabs(pr[valCSRIndices[k]]);
				}
			}
		}
	} else {
		((SparseMatrix&) res).assignSparseMatrix((SparseMatrix&) abs(A));
		/*err("");
			exit(1);*/
	}
}

/**
 * res = A
 * @param res
 * @param A
 */
void assign(Matrix& res, Matrix& A) {
	if (&res == &A)
		return;
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {
		((SparseMatrix&) res).assignSparseMatrix(sparse(A));
	} else {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = ARow[j];
				}
			}
		} else {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					assignVector(resRow, N, 0);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						resRow[c] = 0;
					}
					resRow[currentColumnIdx] = pr[valCSRIndices[k]];
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					resRow[c] = 0;
				}
			}
		}
	}
}

/**
 * res = subplus(A).
 *
 * @param res
 * @param A
 */
void subplus(Matrix& res, Matrix& A) {
	assign(res, A);
	subplusAssign(res);
}

/**
 * res = subplus(res).
 *
 * @param res
 */
void subplusAssign(Matrix& res) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				if (resRow[j] < 0)
					resRow[j] = 0;
			}
		}
	} else if (typeid(res) == typeid(SparseMatrix)) {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			if (pr[k] < 0)
				pr[k] = 0;
		}
		((SparseMatrix&) res).clean();
	}
}

/**
 * res = A | B.
 *
 * @param res
 * @param A
 * @param B
 */
void _or(Matrix& res, Matrix& A, Matrix& B) {
	double** resData = null;
	if (typeid(res) == typeid(DenseMatrix)) {
		resData = ((DenseMatrix&) res).getData();
	} else {
		err("res should be a dense matrix.");
		exit(1);
	}
	double** AData = null;
	if (typeid(A) == typeid(DenseMatrix)) {
		AData = ((DenseMatrix&) A).getData();
	} else {
		err("A should be a dense matrix.");
		exit(1);
	}
	double** BData = null;
	if (typeid(B) == typeid(DenseMatrix)) {
		BData = ((DenseMatrix&) B).getData();
	} else {
		err("B should be a dense matrix.");
		exit(1);
	}
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	double* resRow = null;
	double* ARow = null;
	double* BRow = null;
	for (int i = 0; i < M; i++) {
		resRow = resData[i];
		ARow = AData[i];
		BRow = BData[i];
		for (int j = 0; j < N; j++) {
			resRow[j] = ARow[j] + BRow[j] >= 1 ? 1 : 0;
		}
	}
}

/**
 * res = a * V + b * U.
 *
 * @param res
 * @param a
 * @param V
 * @param b
 * @param U
 */
void affine(Vector& res, double a, Vector& V, double b, Vector& U) {
	if (b == 0) {
		times(res, a, V);
		return;
	} else if (b == 1) {
		affine(res, a, V, '+', U);
		return;
	} else if (b == -1) {
		affine(res, a, V, '-', U);
		return;
	}
	if (a == 0) {
		times(res, b, U);
		return;
	} else if (a == 1) {
		affine(res, b, U, '+', V);
		return;
	} else if (a == -1) {
		affine(res, b, U, '-', V);
		return;
	}
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) { // res = a * V + b * U
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			if (typeid(U) == typeid(DenseVector)) {
				double* UData = ((DenseVector&) U).getPr();
				for (int i = 0; i < dim; i++) {
					resData[i] = a * VData[i] + b * UData[i];
				}
			} else if (typeid(U) == typeid(SparseVector)) {
				int* ir = ((SparseVector&) U).getIr();
				double* pr = ((SparseVector&) U).getPr();
				int nnz = ((SparseVector&) U).getNNZ();
				int lastIdx = -1;
				int currentIdx = 0;
				for (int k = 0; k < nnz; k++) {
					currentIdx = ir[k];
					for (int i = lastIdx + 1; i < currentIdx; i++) {
						resData[i] = a * VData[i];
					}
					resData[currentIdx] = a * VData[currentIdx] + b * pr[k];
					lastIdx = currentIdx;
				}
				for (int i = lastIdx + 1; i < dim; i++) {
					resData[i] = a * VData[i];
				}
			}
		} else if (typeid(V) == typeid(SparseVector)) { // res = a * V +b * U
			int* ir1 = ((SparseVector&) V).getIr();
			double* pr1 = ((SparseVector&) V).getPr();
			int nnz1 = ((SparseVector&) V).getNNZ();
			if (typeid(U) == typeid(DenseVector)) {
				double* UData = ((DenseVector&) U).getPr();
				int lastIdx = -1;
				int currentIdx = 0;
				for (int k = 0; k < nnz1; k++) {
					currentIdx = ir1[k];
					for (int i = lastIdx + 1; i < currentIdx; i++) {
						resData[i] = b * UData[i];
					}
					resData[currentIdx] = a * pr1[k] + b * UData[currentIdx];
					lastIdx = currentIdx;
				}
				for (int i = lastIdx + 1; i < dim; i++) {
					resData[i] = b * UData[i];
				}
			} else if (typeid(U) == typeid(SparseVector)) { // res = a * V + b * U
				int* ir2 = ((SparseVector&) U).getIr();
				double* pr2 = ((SparseVector&) U).getPr();
				int nnz2 = ((SparseVector&) U).getNNZ();
				clear(resData, dim);
				if (!(nnz1 == 0 && nnz2 == 0)) {
					int k1 = 0;
					int k2 = 0;
					int r1 = 0;
					int r2 = 0;
					double v = 0;
					int i = -1;
					while (k1 < nnz1 || k2 < nnz2) {
						if (k2 == nnz2) { // V has been processed.
							i = ir1[k1];
							v = a * pr1[k1];
							k1++;
						} else if (k1 == nnz1) { // this has been processed.
							i = ir2[k2];
							v = b * pr2[k2];
							k2++;
						} else { // Both this and V have not been fully processed.
							r1 = ir1[k1];
							r2 = ir2[k2];
							if (r1 < r2) {
								i = r1;
								v = a * pr1[k1];
								k1++;
							} else if (r1 == r2) {
								i = r1;
								v = a * pr1[k1] + b * pr2[k2];
								k1++;
								k2++;
							} else {
								i = r2;
								v = b * pr2[k2];
								k2++;
							}
						}
						if (v != 0) {
							resData[i] = v;
						}
					}
				}
			}
		}
	}
}

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
void affine(Vector& res, double a, Vector& V, char _operator, Vector& U) {
	if (_operator == '+') {
		if (a == 0) {
			assign(res, U);
			return;
		} else if (a == 1) {
			plus(res, V, U);
			return;
		} else if (a == -1) {
			minus(res, U, V);
			return;
		}
		int dim = res.getDim();
		if (typeid(res) == typeid(SparseVector)) {

		} else if (typeid(res) == typeid(DenseVector)) { // res = a * V + U
			double* resData = ((DenseVector&) res).getPr();
			if (typeid(V) == typeid(DenseVector)) {
				double* VData = ((DenseVector&) V).getPr();
				if (typeid(U) == typeid(DenseVector)) {
					double* UData = ((DenseVector&) U).getPr();
					for (int i = 0; i < dim; i++) {
						resData[i] = a * VData[i] + UData[i];
					}
				} else if (typeid(U) == typeid(SparseVector)) {
					int* ir = ((SparseVector&) U).getIr();
					double* pr = ((SparseVector&) U).getPr();
					int nnz = ((SparseVector&) U).getNNZ();
					int lastIdx = -1;
					int currentIdx = 0;
					for (int k = 0; k < nnz; k++) {
						currentIdx = ir[k];
						for (int i = lastIdx + 1; i < currentIdx; i++) {
							resData[i] = a * VData[i];
						}
						resData[currentIdx] = a * VData[currentIdx] + pr[k];
						lastIdx = currentIdx;
					}
					for (int i = lastIdx + 1; i < dim; i++) {
						resData[i] = a * VData[i];
					}
				}
			} else if (typeid(V) == typeid(SparseVector)) { // res = a * V + U
				int* ir1 = ((SparseVector&) V).getIr();
				double* pr1 = ((SparseVector&) V).getPr();
				int nnz1 = ((SparseVector&) V).getNNZ();
				if (typeid(U) == typeid(DenseVector)) {
					double* UData = ((DenseVector&) U).getPr();
					int lastIdx = -1;
					int currentIdx = 0;
					for (int k = 0; k < nnz1; k++) {
						currentIdx = ir1[k];
						for (int i = lastIdx + 1; i < currentIdx; i++) {
							resData[i] = UData[i];
						}
						resData[currentIdx] = a * pr1[k] + UData[currentIdx];
						lastIdx = currentIdx;
					}
					for (int i = lastIdx + 1; i < dim; i++) {
						resData[i] = UData[i];
					}
				} else if (typeid(U) == typeid(SparseVector)) { // res = a * V + U
					int* ir2 = ((SparseVector&) U).getIr();
					double* pr2 = ((SparseVector&) U).getPr();
					int nnz2 = ((SparseVector&) U).getNNZ();
					clear(resData, dim);
					if (!(nnz1 == 0 && nnz2 == 0)) {
						int k1 = 0;
						int k2 = 0;
						int r1 = 0;
						int r2 = 0;
						double v = 0;
						int i = -1;
						while (k1 < nnz1 || k2 < nnz2) {
							if (k2 == nnz2) { // V has been processed.
								i = ir1[k1];
								v = a * pr1[k1];
								k1++;
							} else if (k1 == nnz1) { // this has been processed.
								i = ir2[k2];
								v = pr2[k2];
								k2++;
							} else { // Both this and V have not been fully processed.
								r1 = ir1[k1];
								r2 = ir2[k2];
								if (r1 < r2) {
									i = r1;
									v = a * pr1[k1];
									k1++;
								} else if (r1 == r2) {
									i = r1;
									v = a * pr1[k1] + pr2[k2];
									k1++;
									k2++;
								} else {
									i = r2;
									v = pr2[k2];
									k2++;
								}
							}
							if (v != 0) {
								resData[i] = v;
							}
						}
					}
				}
			}
		}
	} else if (_operator == '-') {
		if (a == 0) {
			uminus(res, U);
			return;
		} else if (a == 1) {
			minus(res, V, U);
			return;
		}
		int dim = res.getDim();
		if (typeid(res) == typeid(SparseVector)) {

		} else if (typeid(res) == typeid(DenseVector)) { // res = a * V - U
			double* resData = ((DenseVector&) res).getPr();
			if (typeid(V) == typeid(DenseVector)) {
				double* VData = ((DenseVector&) V).getPr();
				if (typeid(U) == typeid(DenseVector)) {
					double* UData = ((DenseVector&) U).getPr();
					for (int i = 0; i < dim; i++) {
						resData[i] = a * VData[i] - UData[i];
					}
				} else if (typeid(U) == typeid(SparseVector)) {
					int* ir = ((SparseVector&) U).getIr();
					double* pr = ((SparseVector&) U).getPr();
					int nnz = ((SparseVector&) U).getNNZ();
					int lastIdx = -1;
					int currentIdx = 0;
					for (int k = 0; k < nnz; k++) {
						currentIdx = ir[k];
						for (int i = lastIdx + 1; i < currentIdx; i++) {
							resData[i] = a * VData[i];
						}
						resData[currentIdx] = a * VData[currentIdx] - pr[k];
						lastIdx = currentIdx;
					}
					for (int i = lastIdx + 1; i < dim; i++) {
						resData[i] = a * VData[i];
					}
				}
			} else if (typeid(V) == typeid(SparseVector)) {
				int* ir1 = ((SparseVector&) V).getIr();
				double* pr1 = ((SparseVector&) V).getPr();
				int nnz1 = ((SparseVector&) V).getNNZ();
				if (typeid(U) == typeid(DenseVector)) { // res = a * V - U
					double* UData = ((DenseVector&) U).getPr();
					int lastIdx = -1;
					int currentIdx = 0;
					for (int k = 0; k < nnz1; k++) {
						currentIdx = ir1[k];
						for (int i = lastIdx + 1; i < currentIdx; i++) {
							resData[i] = -UData[i];
						}
						resData[currentIdx] = a * pr1[k] - UData[currentIdx];
						lastIdx = currentIdx;
					}
					for (int i = lastIdx + 1; i < dim; i++) {
						resData[i] = -UData[i];
					}
				} else if (typeid(U) == typeid(SparseVector)) { // res = a * V - U
					int* ir2 = ((SparseVector&) U).getIr();
					double* pr2 = ((SparseVector&) U).getPr();
					int nnz2 = ((SparseVector&) U).getNNZ();
					clear(resData, dim);
					if (!(nnz1 == 0 && nnz2 == 0)) {
						int k1 = 0;
						int k2 = 0;
						int r1 = 0;
						int r2 = 0;
						double v = 0;
						int i = -1;
						while (k1 < nnz1 || k2 < nnz2) {
							if (k2 == nnz2) { // V has been processed.
								i = ir1[k1];
								v = a * pr1[k1];
								k1++;
							} else if (k1 == nnz1) { // this has been processed.
								i = ir2[k2];
								v = -pr2[k2];
								k2++;
							} else { // Both this and V have not been fully processed.
								r1 = ir1[k1];
								r2 = ir2[k2];
								if (r1 < r2) {
									i = r1;
									v = a * pr1[k1];
									k1++;
								} else if (r1 == r2) {
									i = r1;
									v = a * pr1[k1] - pr2[k2];
									k1++;
									k2++;
								} else {
									i = r2;
									v = -pr2[k2];
									k2++;
								}
							}
							if (v != 0) {
								resData[i] = v;
							}
						}
					}
				}
			}
		}
	}
}

/**
 * res = V + b * U.
 *
 * @param res
 * @param V
 * @param b
 * @param U
 */
void affine(Vector& res, Vector& V, double b, Vector& U) {
	affine(res, b, U, '+', V);
}

/**
 * res = -res.
 *
 * @param res
 */
void uminusAssign(Vector& res) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {
		double* pr = ((SparseVector&) res).getPr();
		int nnz = ((SparseVector&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			pr[k] = -pr[k];
		}
	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		for (int i = 0; i < dim; i++) {
			resData[i] = -resData[i];
		}
	}
}

/**
 * res = -V.
 *
 * @param res
 * @param V
 */
void uminus(Vector& res, Vector& V) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] = -VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx; i++) {
					resData[i] = 0;
				}
				resData[currentIdx] = -pr[k];
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim; i++) {
				resData[i] = 0;
			}
		}
	}
}

/**
 * res = V .* U.
 *
 * @param res
 * @param V
 * @param U
 */
void times(Vector& res, Vector& V, Vector& U) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			if (typeid(U) == typeid(DenseVector)) {
				double* UData = ((DenseVector&) U).getPr();
				for (int i = 0; i < dim; i++) {
					resData[i] = VData[i] * UData[i];
				}
			} else if (typeid(U) == typeid(SparseVector)) {
				int* ir = ((SparseVector&) U).getIr();
				double* pr = ((SparseVector&) U).getPr();
				int nnz = ((SparseVector&) U).getNNZ();
				int idx = -1;
				res.clear();
				for (int k = 0; k < nnz; k++) {
					idx = ir[k];
					resData[idx] = VData[idx] * pr[k];
				}
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir1 = ((SparseVector&) V).getIr();
			double* pr1 = ((SparseVector&) V).getPr();
			int nnz1 = ((SparseVector&) V).getNNZ();
			if (typeid(U) == typeid(DenseVector)) {
				double* UData = ((DenseVector&) U).getPr();
				int lastIdx = -1;
				int currentIdx = 0;
				for (int k = 0; k < nnz1; k++) {
					currentIdx = ir1[k];
					for (int i = lastIdx + 1; i < currentIdx; i++) {
						resData[i] = 0;
					}
					resData[currentIdx] = pr1[k] * UData[currentIdx];
					lastIdx = currentIdx;
				}
				for (int i = lastIdx + 1; i < dim; i++) {
					resData[i] = 0;
				}
			} else if (typeid(U) == typeid(SparseVector)) {
				int* ir2 = ((SparseVector&) U).getIr();
				double* pr2 = ((SparseVector&) U).getPr();
				int nnz2 = ((SparseVector&) U).getNNZ();
				res.clear();
				if (nnz1 != 0 && nnz2 != 0) {
					int k1 = 0;
					int k2 = 0;
					int r1 = 0;
					int r2 = 0;
					double v = 0;
					int i = -1;
					while (k1 < nnz1 && k2 < nnz2) {
						r1 = ir1[k1];
						r2 = ir2[k2];
						if (r1 < r2) {
							k1++;
						} else if (r1 == r2) {
							i = r1;
							v = pr1[k1] * pr2[k2];
							k1++;
							k2++;
							if (v != 0) {
								resData[i] = v;
							}
						} else {
							k2++;
						}
					}
				}
			}
		}
	}
}

/**
 * res = v * V.
 *
 * @param res
 * @param v
 * @param V
 */
void times(Vector& res, double v, Vector& V) {
	if (v == 0) {
		res.clear();
		return;
	} else if (v == 1) {
		assign(res, V);
		return;
	} else if (v == -1) {
		uminus(res, V);
		return;
	}
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] = v * VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx; i++) {
					resData[i] = 0;
				}
				resData[currentIdx] = v * pr[k];
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim; i++) {
				resData[i] = 0;
			}
		}
	}
}

/**
 * res *= V.
 *
 * @param res
 * @param V
 */
void timesAssign(Vector& res, Vector& V) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] *= VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx; i++) {
					resData[i] = 0;
				}
				resData[currentIdx] *= pr[k];
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim; i++) {
				resData[i] = 0;
			}
		}
	}
}

/**
 * res *= v.
 *
 * @param res
 * @param v
 */
void timesAssign(Vector& res, double v) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {
		if (v == 0) {
			res.clear();
			return;
		}
		double* pr = ((SparseVector&) res).getPr();
		int nnz = ((SparseVector&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			pr[k] *= v;
		}
	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		for (int i = 0; i < dim; i++) {
			resData[i] *= v;
		}
	}
}

/**
 * res = V.
 *
 * @param res
 * @param V
 */
void assign(Vector& res, Vector& V) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {
		((SparseVector&) res).assignSparseVector((SparseVector&) V);
	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			// System.arraycopy(VData, 0, resData, 0, dim);
			std::copy(VData, VData + dim, resData);
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx; i++) {
					resData[i] = 0;
				}
				resData[currentIdx] = pr[k];
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim; i++) {
				resData[i] = 0;
			}
		}
	}
}

/**
 * res = V - U.
 *
 * @param res
 * @param V
 * @param U
 */
void minus(Vector& res, Vector& V, Vector& U) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			if (typeid(U) == typeid(DenseVector)) {
				double* UData = ((DenseVector&) U).getPr();
				for (int i = 0; i < dim; i++) {
					resData[i] = VData[i] - UData[i];
				}
			} else if (typeid(U) == typeid(SparseVector)) {
				int* ir = ((SparseVector&) U).getIr();
				double* pr = ((SparseVector&) U).getPr();
				int nnz = ((SparseVector&) U).getNNZ();
				// System.arraycopy(VData, 0, resData, 0, dim);
				std::copy(VData, VData + dim, resData);
				for (int k = 0; k < nnz; k++) {
					resData[ir[k]] -= pr[k];
				}
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir1 = ((SparseVector&) V).getIr();
			double* pr1 = ((SparseVector&) V).getPr();
			int nnz1 = ((SparseVector&) V).getNNZ();
			if (typeid(U) == typeid(DenseVector)) {
				double* UData = ((DenseVector&) U).getPr();
				int lastIdx = -1;
				int currentIdx = 0;
				for (int k = 0; k < nnz1; k++) {
					currentIdx = ir1[k];
					for (int i = lastIdx + 1; i < currentIdx; i++) {
						resData[i] = -UData[i];
					}
					resData[currentIdx] = pr1[k] - UData[currentIdx];
					lastIdx = currentIdx;
				}
				for (int i = lastIdx + 1; i < dim; i++) {
					resData[i] = -UData[i];
				}
			} else if (typeid(U) == typeid(SparseVector)) {
				int* ir2 = ((SparseVector&) U).getIr();
				double* pr2 = ((SparseVector&) U).getPr();
				int nnz2 = ((SparseVector&) U).getNNZ();
				clear(resData, dim);
				if (!(nnz1 == 0 && nnz2 == 0)) {
					int k1 = 0;
					int k2 = 0;
					int r1 = 0;
					int r2 = 0;
					double v = 0;
					int i = -1;
					while (k1 < nnz1 || k2 < nnz2) {
						if (k2 == nnz2) { // V has been processed.
							i = ir1[k1];
							v = pr1[k1];
							k1++;
						} else if (k1 == nnz1) { // this has been processed.
							i = ir2[k2];
							v = -pr2[k2];
							k2++;
						} else { // Both this and V have not been fully processed.
							r1 = ir1[k1];
							r2 = ir2[k2];
							if (r1 < r2) {
								i = r1;
								v = pr1[k1];
								k1++;
							} else if (r1 == r2) {
								i = r1;
								v = pr1[k1] - pr2[k2];
								k1++;
								k2++;
							} else {
								i = r2;
								v = -pr2[k2];
								k2++;
							}
						}
						if (v != 0) {
							resData[i] = v;
						}
					}
				}
			}
		}
	}
}

/**
 * res = V - v.
 *
 * @param res
 * @param V
 * @param v
 */
void minus(Vector& res, Vector& V, double v) {
	int dim = res.getDim();
	double minusv = -v;
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] = VData[i] - v;
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx; i++) {
					resData[i] = minusv;
				}
				resData[currentIdx] = pr[k] - v;
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim; i++) {
				resData[i] = minusv;
			}
		}
	}
}

/**
 * res = v - V.
 *
 * @param res
 * @param v
 * @param V
 */
void minus(Vector& res, double v, Vector& V) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] = v - VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx; i++) {
					resData[i] = v;
				}
				resData[currentIdx] = v - pr[k];
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim; i++) {
				resData[i] = v;
			}
		}
	}
}

/**
 * res -= v.
 *
 * @param res
 * @param v
 */
void minusAssign(Vector& res, double v) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		for (int i = 0; i < dim; i++) {
			resData[i] -= v;
		}
	}
}

/**
 * res -= V
 * @param res
 * @param V
 */
void minusAssign(Vector& res, Vector& V) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] -= VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			for (int k = 0; k < nnz; k++) {
				resData[ir[k]] -= pr[k];
			}
		}
	}
}

/**
 * res -= a * V.
 *
 * @param res
 * @param a
 * @param V
 */
void minusAssign(Vector& res, double a, Vector& V) {
	if (a == 1) {
		minusAssign(res, V);
		return;
	}
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] -= a * VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			for (int k = 0; k < nnz; k++) {
				resData[ir[k]] -= a * pr[k];
			}
		}
	}
}

/**
 * res = V + U.
 *
 * @param res
 * @param V
 * @param U
 */
void plus(Vector& res, Vector& V, Vector& U) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {
		((SparseVector&) res).assignSparseVector((SparseVector&) V.plus(U));
	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			if (typeid(U) == typeid(DenseVector)) {
				double* UData = ((DenseVector&) U).getPr();
				for (int i = 0; i < dim; i++) {
					resData[i] = VData[i] + UData[i];
				}
			} else if (typeid(U) == typeid(SparseVector)) {
				int* ir = ((SparseVector&) U).getIr();
				double* pr = ((SparseVector&) U).getPr();
				int nnz = ((SparseVector&) U).getNNZ();
				// System.arraycopy(VData, 0, resData, 0, dim);
				std::copy(VData, VData + dim, resData);
				for (int k = 0; k < nnz; k++) {
					resData[ir[k]] += pr[k];
				}
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir1 = ((SparseVector&) V).getIr();
			double* pr1 = ((SparseVector&) V).getPr();
			int nnz1 = ((SparseVector&) V).getNNZ();
			if (typeid(U) == typeid(DenseVector)) {
				double* UData = ((DenseVector&) U).getPr();
				int lastIdx = -1;
				int currentIdx = 0;
				for (int k = 0; k < nnz1; k++) {
					currentIdx = ir1[k];
					for (int i = lastIdx + 1; i < currentIdx; i++) {
						resData[i] = UData[i];
					}
					resData[currentIdx] = pr1[k] + UData[currentIdx];
					lastIdx = currentIdx;
				}
				for (int i = lastIdx + 1; i < dim; i++) {
					resData[i] = UData[i];
				}
			} else if (typeid(U) == typeid(SparseVector)) {
				int* ir2 = ((SparseVector&) U).getIr();
				double* pr2 = ((SparseVector&) U).getPr();
				int nnz2 = ((SparseVector&) U).getNNZ();
				clear(resData, dim);
				if (!(nnz1 == 0 && nnz2 == 0)) {
					int k1 = 0;
					int k2 = 0;
					int r1 = 0;
					int r2 = 0;
					double v = 0;
					int i = -1;
					while (k1 < nnz1 || k2 < nnz2) {
						if (k2 == nnz2) { // V has been processed.
							i = ir1[k1];
							v = pr1[k1];
							k1++;
						} else if (k1 == nnz1) { // this has been processed.
							i = ir2[k2];
							v = pr2[k2];
							k2++;
						} else { // Both this and V have not been fully processed.
							r1 = ir1[k1];
							r2 = ir2[k2];
							if (r1 < r2) {
								i = r1;
								v = pr1[k1];
								k1++;
							} else if (r1 == r2) {
								i = r1;
								v = pr1[k1] + pr2[k2];
								k1++;
								k2++;
							} else {
								i = r2;
								v = pr2[k2];
								k2++;
							}
						}
						if (v != 0) {
							resData[i] = v;
						}
					}
				}
			}
		}
	}
}

/**
 * res = V + v.
 *
 * @param res
 * @param V
 * @param v
 */
void plus(Vector& res, Vector& V, double v) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] = VData[i] + v;
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx; i++) {
					resData[i] = v;
				}
				resData[currentIdx] = pr[k] + v;
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim; i++) {
				resData[i] = v;
			}
		}
	}
}

/**
 * res = v + V.
 *
 * @param res
 * @param v
 * @param V
 */
void plus(Vector& res, double v, Vector& V) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] = v + VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			int lastIdx = -1;
			int currentIdx = 0;
			for (int k = 0; k < nnz; k++) {
				currentIdx = ir[k];
				for (int i = lastIdx + 1; i < currentIdx; i++) {
					resData[i] = v;
				}
				resData[currentIdx] = v + pr[k];
				lastIdx = currentIdx;
			}
			for (int i = lastIdx + 1; i < dim; i++) {
				resData[i] = v;
			}
		}
	}
}

/**
 * res += v.
 *
 * @param res
 * @param v
 */
void plusAssign(Vector& res, double v) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		for (int i = 0; i < dim; i++) {
			resData[i] += v;
		}
	}
}

/**
 * res += V
 * @param res
 * @param V
 */
void plusAssign(Vector& res, Vector& V) {
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {
		((SparseVector&) res).assignSparseVector((SparseVector&) res.plus(V));
	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] += VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			for (int k = 0; k < nnz; k++) {
				resData[ir[k]] += pr[k];
			}
		}
	}
}

/**
 * res += a * V.
 *
 * @param res
 * @param a
 * @param V
 */
void plusAssign(Vector& res, double a, Vector& V) {
	if (a == 1) {
		minusAssign(res, V);
		return;
	}
	int dim = res.getDim();
	if (typeid(res) == typeid(SparseVector)) {

	} else if (typeid(res) == typeid(DenseVector)) {
		double* resData = ((DenseVector&) res).getPr();
		if (typeid(V) == typeid(DenseVector)) {
			double* VData = ((DenseVector&) V).getPr();
			for (int i = 0; i < dim; i++) {
				resData[i] += a * VData[i];
			}
		} else if (typeid(V) == typeid(SparseVector)) {
			int* ir = ((SparseVector&) V).getIr();
			double* pr = ((SparseVector&) V).getPr();
			int nnz = ((SparseVector&) V).getNNZ();
			for (int k = 0; k < nnz; k++) {
				resData[ir[k]] += a * pr[k];
			}
		}
	}
}

/**
 * res = A * B if operator is ' ',</br>
 * res = A<sup>T</sup> * B if operator is 'T'.
 *
 * @param res
 * @param A
 * @param _operator a {@code char} variable: 'T' or ' '
 * @param B
 */
void mtimes(Matrix& res, Matrix& A, char _operator, Matrix& B) {
	if (_operator == ' ') {
		mtimes(res, A, B);
	} else if (_operator == 'T') {

		if (typeid(res) == typeid(SparseMatrix)) {
			((SparseMatrix&) res).assignSparseMatrix(sparse(A.transpose().mtimes(B)));
		} else if (typeid(res) == typeid(DenseMatrix)) {
			double** resData = ((DenseMatrix&) res).getData();
			// double* rowA = null;
			int NB = B.getColumnDimension();
			int N = A.getRowDimension();
			int M = A.getColumnDimension();
			if (typeid(A) == typeid(DenseMatrix)) {
				double** AData = ((DenseMatrix&) A).getData();
				if (typeid(B) == typeid(DenseMatrix)) {

					double** BData = ((DenseMatrix&) B).getData();
					// double* columnB = new double[B.getRowDimension()];
					// double* columnA = new double[A.getRowDimension()];
					double* resRow = null;
					double* BRow = null;
					// double s = 0;
					double A_ki = 0;
					for (int i = 0; i < M; i++) {
						resRow = resData[i];
						clear(resRow, N);
						for (int k = 0; k < N; k++) {
							BRow = BData[k];
							A_ki = AData[k][i];
							for (int j = 0; j < NB; j++) {
								resRow[j] += A_ki * BRow[j];
							}
						}
					}

					/*for (int j = 0; j < NB; j++) {
							for (int r = 0; r < B.getRowDimension(); r++) {
								columnB[r] = BData[r][j];
							}

							for (int i = 0; i < M; i++) {
								for (int r = 0; r < A.getRowDimension(); r++) {
									columnA[r] = AData[r][i];
								}
								s = 0;
								for (int k = 0; k < N; k++) {
									// Using AData[k][j] costs 16.8 seconds
									// Referring AData[k][j] involves one integer multiplication!
									// s += rowData[k] * AData[k][j];
									// Using columnA[j] costs 3.4 seconds
									s += columnA[k] * columnB[k];
								}
								resData[i][j] = s;
							}
						}*/

				} else if (typeid(B) == typeid(SparseMatrix)) {

					int* ir = null;
					int* jc = null;
					double* pr = null;
					ir = ((SparseMatrix&) B).getIr();
					jc = ((SparseMatrix&) B).getJc();
					pr = ((SparseMatrix&) B).getPr();
					int r = -1;
					double s = 0;
					double* columnA = new double[A.getRowDimension()];
					for (int i = 0; i < M; i++) {
						for (int t = 0; t < N; t++) {
							columnA[t] = AData[t][i];
						}
						for (int j = 0; j < NB; j++) {
							s = 0;
							for (int k = jc[j]; k < jc[j + 1]; k++) {
								r = ir[k];
								// A[r][j] = pr[k]
								s += columnA[r] * pr[k];
							}
							resData[i][j] = s;
						}
					}

				}
			} else if (typeid(A) == typeid(SparseMatrix)) {

				if (typeid(B) == typeid(DenseMatrix)) {
					int* ir = ((SparseMatrix&) A).getIr();
					int* jc = ((SparseMatrix&) A).getJc();
					double* pr = ((SparseMatrix&) A).getPr();
					// int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
					double** BData = ((DenseMatrix&) B).getData();
					int c = -1;
					double s = 0;
					for (int i = 0; i < M; i++) {
						for (int j = 0; j < NB; j++) {
							s = 0;
							for (int k = jc[i]; k < jc[i + 1]; k++) {
								c = ir[k];
								s += pr[k] * BData[c][j];
							}
							resData[i][j] = s;
						}
					}
				} else if (typeid(B) == typeid(SparseMatrix)) {
					double* resRow = null;
					int* ir1 = ((SparseMatrix&) A).getIr();
					int* jc1 = ((SparseMatrix&) A).getJc();
					double* pr1 = ((SparseMatrix&) A).getPr();
					int* ir2 = ((SparseMatrix&) B).getIr();
					int* jc2 = ((SparseMatrix&) B).getJc();
					double* pr2 = ((SparseMatrix&) B).getPr();
					// rowIdx of the right sparse matrix
					int rr = -1;
					// colIdx of the left sparse matrix
					int cl = -1;
					double s = 0;
					int kl = 0;
					int kr = 0;
					for (int i = 0; i < M; i++) {
						resRow = resData[i];
						for (int j = 0; j < NB; j++) {
							s = 0;
							kl = jc1[i];
							kr = jc2[j];
							while (true) {
								if (kl >= jc1[i + 1] || kr >= jc2[j + 1]) {
									break;
								}
								cl = ir1[kl];
								rr = ir2[kr];
								if (cl < rr) {
									kl++;
								} else if (cl > rr) {
									kr++;
								} else {
									s += pr1[kl] * pr2[kr];
									kl++;
									kr++;
								}
							}
							resRow[j] = s;
						}
					}
				}
			}
		}
	}
}

/**
 * res = A * B.
 * @param res result matrix
 * @param A	a real matrix
 * @param B a real matrix
 */
void mtimes(Matrix& res, Matrix& A, Matrix& B) {
	if (typeid(res) == typeid(SparseMatrix)) {
		((SparseMatrix&) res).assignSparseMatrix(sparse(A.mtimes(B)));
	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* rowA = null;
		int NB = B.getColumnDimension();
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			if (typeid(B) == typeid(DenseMatrix)) {

				double** BData = ((DenseMatrix&) B).getData();
				double* columnB = new double[B.getRowDimension()];
				double s = 0;
				for (int j = 0; j < NB; j++) {
					for (int r = 0; r < B.getRowDimension(); r++) {
						columnB[r] = BData[r][j];
					}
					for (int i = 0; i < M; i++) {
						rowA = AData[i];
						s = 0;
						for (int k = 0; k < N; k++) {
							// Using AData[k][j] costs 16.8 seconds
							// Referring AData[k][j] involves one integer multiplication!
							// s += rowData[k] * AData[k][j];
							// Using columnA[j] costs 3.4 seconds
							s += rowA[k] * columnB[k];
						}
						resData[i][j] = s;
					}
				}

			} else if (typeid(B) == typeid(SparseMatrix)) {

				int* ir = null;
				int* jc = null;
				double* pr = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr = ((SparseMatrix&) B).getPr();
				int r = -1;
				double s = 0;

				for (int i = 0; i < M; i++) {
					rowA = AData[i];
					for (int j = 0; j < NB; j++) {
						s = 0;
						for (int k = jc[j]; k < jc[j + 1]; k++) {
							r = ir[k];
							// A[r][j] = pr[k]
							s += rowA[r] * pr[k];
						}
						resData[i][j] = s;
					}
				}

			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			double* pr = ((SparseMatrix&) A).getPr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) B).getData();
				int c = -1;
				double s = 0;
				for (int i = 0; i < M; i++) {
					for (int j = 0; j < NB; j++) {
						s = 0;
						for (int k = jr[i]; k < jr[i + 1]; k++) {
							c = ic[k];
							s += pr[valCSRIndices[k]] * BData[c][j];
						}
						resData[i][j] = s;
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				double* resRow = null;
				int* ir = null;
				int* jc = null;
				double* pr2 = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr2 = ((SparseMatrix&) B).getPr();
				// rowIdx of the right sparse matrix
				int rr = -1;
				// colIdx of the left sparse matrix
				int cl = -1;
				double s = 0;
				int kl = 0;
				int kr = 0;
				for (int i = 0; i < M; i++) {
					resRow = resData[i];
					for (int j = 0; j < NB; j++) {
						s = 0;
						kl = jr[i];
						kr = jc[j];
						while (true) {
							if (kl >= jr[i + 1] || kr >= jc[j + 1]) {
								break;
							}
							cl = ic[kl];
							rr = ir[kr];
							if (cl < rr) {
								kl++;
							} else if (cl > rr) {
								kr++;
							} else {
								s += pr[valCSRIndices[kl]] * pr2[kr];
								kl++;
								kr++;
							}
						}
						resRow[j] = s;
					}
				}
			}
		}
	}
}

/**
 * res = A .* B
 * @param res
 * @param A
 * @param B
 */
void times(Matrix& res, Matrix& A, Matrix& B) {
	if (typeid(res) == typeid(SparseMatrix)) {
		((SparseMatrix&) res).assignSparseMatrix(sparse(A.times(B)));
	} else if (typeid(res) == typeid(DenseMatrix)) {
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		double** resData = ((DenseMatrix&) res).getData();
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) B).getData();
				double* BRow = null;
				double* ARow = null;
				double* resRow = null;
				for (int i = 0; i < M; i++) {
					ARow = AData[i];
					BRow = BData[i];
					resRow = resData[i];
					for (int j = 0; j < N; j++) {
						resRow[j] = ARow[j] * BRow[j];
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				// ArrayOperator.clearMatrix(resData);
				res.clear();
				int* ir = null;
				int* jc = null;
				double* pr = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr = ((SparseMatrix&) B).getPr();
				int r = -1;
				for (int j = 0; j < B.getColumnDimension(); j++) {
					for (int k = jc[j]; k < jc[j + 1]; k++) {
						r = ir[k];
						// A[r][j] = pr[k]
						resData[r][j] = AData[r][j] * pr[k];
					}
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			if (typeid(B) == typeid(DenseMatrix)) {
				times(res, B, A);
			} else if (typeid(B) == typeid(SparseMatrix)) {
				int* ir1 = null;
				int* jc1 = null;
				double* pr1 = null;
				ir1 = ((SparseMatrix&) A).getIr();
				jc1 = ((SparseMatrix&) A).getJc();
				pr1 = ((SparseMatrix&) A).getPr();
				int* ir2 = null;
				int* jc2 = null;
				double* pr2 = null;
				ir2 = ((SparseMatrix&) B).getIr();
				jc2 = ((SparseMatrix&) B).getJc();
				pr2 = ((SparseMatrix&) B).getPr();

				// ArrayOperator.clearMatrix(resData);
				res.clear();

				int k1 = 0;
				int k2 = 0;
				int r1 = -1;
				int r2 = -1;
				int i = -1;
				double v = 0;

				for (int j = 0; j < N; j++) {
					k1 = jc1[j];
					k2 = jc2[j];

					// If the j-th column of A or this is empty, we don't need to compute.
					if (k1 == jc1[j + 1] || k2 == jc2[j + 1])
						continue;

					while (k1 < jc1[j + 1] && k2 < jc2[j + 1]) {

						r1 = ir1[k1];
						r2 = ir2[k2];
						if (r1 < r2) {
							k1++;
						} else if (r1 == r2) {
							i = r1;
							v = pr1[k1] * pr2[k2];
							k1++;
							k2++;
							if (v != 0) {
								resData[i][j] = v;
							}
						} else {
							k2++;
						}

					}

				}
			}
		}
	}
}

/**
 * res = a * V
 * @param res
 * @param a
 * @param V
 * @param len length of res and V
 */
void times(double* res, double a, double* V, int len) {
	for (int i = 0; i < len; i++) {
		res[i] = a * V[i];
	}
}

/**
 * res = res .* A
 * @param res
 * @param A
 */
void timesAssign(Matrix& res, Matrix& A) {
	if (typeid(res) == typeid(SparseMatrix)) {
		((SparseMatrix&) res).assignSparseMatrix(sparse(res.times(A)));
	} else if (typeid(res) == typeid(DenseMatrix)) {
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				ARow = AData[i];
				resRow = resData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] *= ARow[j];
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					clearVector(resRow, N);
					continue;
				}

				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						resRow[c] = 0;
					}
					resRow[currentColumnIdx] *= pr[valCSRIndices[k]];
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					resRow[c] = 0;
				}
			}
		}
	}
}

/**
 * res = v * res
 * @param res
 * @param v
 */
void timesAssign(Matrix& res, double v) {
	if (v == 0) {
		res.clear();
	}
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(SparseMatrix)) {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			pr[k] *= v;
		}
	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				resRow[j] *= v;
			}
		}
	}
}

/**
 * res = v * A
 * @param res
 * @param v
 * @param A
 */
void times(Matrix& res, double v, Matrix& A) {
	if (v == 1) {
		assign(res, A);
		return;
	}
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = ARow[j] * v;
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					assignVector(resRow, N, 0);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						resRow[c] = 0;
					}
					resRow[currentColumnIdx] = pr[valCSRIndices[k]] * v;
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					resRow[c] = 0;
				}
			}
		}
	}
}

/**
 * Clear the input matrix.
 *
 * @param res a real matrix
 */
void clear(Matrix& res) {
	res.clear();
}

/**
 * Clear the input 2D {@code double} array.
 *
 * @param res a 2D {@code double} array
 *
 * @param M number of rows
 *
 * @param N number of columns
 */
void clear(double** res, int M, int N) {
	clearMatrix(res, M, N);
}

/**
 * Clear the input vector.
 *
 * @param res a real vector
 */
void clear(Vector& res) {
	res.clear();
}

/**
 * Clear the input 1D {@code double} array.
 *
 * @param res a 1D {@code double} array
 *
 * @param len length of res
 */
void clear(double* res, int len) {
	clearVector(res, len);
}

/**
 * res = V
 * @param res
 * @param V
 * @param len length of res and V
 */
void assign(double* res, double* V, int len) {
	// System.arraycopy(V, 0, res, 0, len);
	std::copy(V, V + len, res);
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

void assign(double* res, int len, double v) {
	for (int i = 0; i < len; i++)
		res[i] = v;
}*/

/**
 * res = -res
 * @param res
 */
void uminusAssign(Matrix& res) {
	if (typeid(res) == typeid(SparseMatrix)) {
		double* pr = ((SparseMatrix&) res).getPr();
		int nnz = ((SparseMatrix&) res).getNNZ();
		for (int k = 0; k < nnz; k++) {
			pr[k] = -pr[k];
		}
	} else if (typeid(res) == typeid(DenseMatrix)) {
		int M = res.getRowDimension();
		int N = res.getColumnDimension();
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				resRow[j] = -resRow[j];
			}
		}
	}
}

/**
 * res = -A
 * @param res
 * @param A
 */
void uminus(Matrix& res, Matrix& A) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = -ARow[j];
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					assignVector(resRow, N, 0);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						resRow[c] = 0;
					}
					resRow[currentColumnIdx] = -pr[valCSRIndices[k]];
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					resRow[c] = 0;
				}
			}
		}
	}
}

/**
 * res = -V
 * @param res
 * @param V
 * @param len length of res and V
 */
void uminus(double* res, double* V, int len) {
	for (int i = 0; i < len; i++) {
		res[i] = -V[i];
	}
}

/**
 * res = v \ res
 * @param res
 * @param v
 */
void divide(Matrix& res, double v) {

}

/**
 * res = res / v
 * @param res
 * @param v
 */
void rdivideAssign(Matrix& res, double v) {
	int nRow = res.getRowDimension();
	int nCol = res.getColumnDimension();

	if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < nRow; i++) {
			resRow = resData[i];
			for (int j = 0; j < nCol; j++) {
				resRow[j] /= v;
			}
		}
	} else if (typeid(res) == typeid(SparseMatrix)) {
		double* pr = ((SparseMatrix&) res).getPr();
		for (int k = 0; k < ((SparseMatrix&) res).getNNZ(); k++) {
			pr[k] /= v;
		}
	}
}

/**
 * res = A * B + v * C
 * @param res
 * @param A
 * @param B
 * @param v
 * @param C
 */
void affine(Matrix& res, Matrix& A, Matrix& B, double v, Matrix& C) {
	if (typeid(res) == typeid(SparseMatrix)) {
		// ((SparseMatrix&) res).assignSparseMatrix(sparse(A.mtimes(B)));
		err("Sparse matrix for res is not supported.");
		exit(1);
	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* rowA = null;
		int NB = B.getColumnDimension();
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			if (typeid(B) == typeid(DenseMatrix)) {

				double** BData = ((DenseMatrix&) B).getData();
				double* columnB = new double[B.getRowDimension()];
				double s = 0;
				for (int j = 0; j < NB; j++) {
					for (int r = 0; r < B.getRowDimension(); r++) {
						columnB[r] = BData[r][j];
					}
					for (int i = 0; i < M; i++) {
						rowA = AData[i];
						s = v * C.getEntry(i, j);
						for (int k = 0; k < N; k++) {
							// Using AData[k][j] costs 16.8 seconds
							// Referring AData[k][j] involves one integer multiplication!
							// s += rowData[k] * AData[k][j];
							// Using columnA[j] costs 3.4 seconds
							s += rowA[k] * columnB[k];
						}
						resData[i][j] = s;
					}
				}

			} else if (typeid(B) == typeid(SparseMatrix)) {

				int* ir = null;
				int* jc = null;
				double* pr = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr = ((SparseMatrix&) B).getPr();
				int r = -1;
				double s = 0;

				for (int i = 0; i < M; i++) {
					rowA = AData[i];
					for (int j = 0; j < NB; j++) {
						s = v * C.getEntry(i, j);
						for (int k = jc[j]; k < jc[j + 1]; k++) {
							r = ir[k];
							// A[r][j] = pr[k]
							s += rowA[r] * pr[k];
						}
						resData[i][j] = s;
					}
				}

			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			double* pr = ((SparseMatrix&) A).getPr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) B).getData();
				int c = -1;
				double s = 0;
				for (int i = 0; i < M; i++) {
					for (int j = 0; j < NB; j++) {
						s = v * C.getEntry(i, j);
						for (int k = jr[i]; k < jr[i + 1]; k++) {
							c = ic[k];
							s += pr[valCSRIndices[k]] * BData[c][j];
						}
						resData[i][j] = s;
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				double* resRow = null;
				int* ir = null;
				int* jc = null;
				double* pr2 = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr2 = ((SparseMatrix&) B).getPr();
				// rowIdx of the right sparse matrix
				int rr = -1;
				// colIdx of the left sparse matrix
				int cl = -1;
				double s = 0;
				int kl = 0;
				int kr = 0;
				for (int i = 0; i < M; i++) {
					resRow = resData[i];
					for (int j = 0; j < NB; j++) {
						s = v * C.getEntry(i, j);
						kl = jr[i];
						kr = jc[j];
						while (true) {
							if (kl >= jr[i + 1] || kr >= jc[j + 1]) {
								break;
							}
							cl = ic[kl];
							rr = ir[kr];
							if (cl < rr) {
								kl++;
							} else if (cl > rr) {
								kr++;
							} else {
								s += pr[valCSRIndices[kl]] * pr2[kr];
								kl++;
								kr++;
							}
						}
						resRow[j] = s;
					}
				}
			}
		}
	}
}

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
void affine(Matrix& res, Matrix& A, Matrix& B, char _operator, Matrix& C) {
	if (typeid(res) == typeid(SparseMatrix)) {
		// ((SparseMatrix&) res).assignSparseMatrix(sparse(A.mtimes(B)));
		err("Sparse matrix for res is not supported.");
		exit(1);
	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* rowA = null;
		int NB = B.getColumnDimension();
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			if (typeid(B) == typeid(DenseMatrix)) {

				double** BData = ((DenseMatrix&) B).getData();
				double* columnB = new double[B.getRowDimension()];
				double s = 0;
				for (int j = 0; j < NB; j++) {
					for (int r = 0; r < B.getRowDimension(); r++) {
						columnB[r] = BData[r][j];
					}
					for (int i = 0; i < M; i++) {
						rowA = AData[i];
						if (_operator == '+')
							s = C.getEntry(i, j);
						else if (_operator == '-')
							s = -C.getEntry(i, j);
						for (int k = 0; k < N; k++) {
							// Using AData[k][j] costs 16.8 seconds
							// Referring AData[k][j] involves one integer multiplication!
							// s += rowData[k] * AData[k][j];
							// Using columnA[j] costs 3.4 seconds
							s += rowA[k] * columnB[k];
						}
						resData[i][j] = s;
					}
				}

			} else if (typeid(B) == typeid(SparseMatrix)) {

				int* ir = null;
				int* jc = null;
				double* pr = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr = ((SparseMatrix&) B).getPr();
				int r = -1;
				double s = 0;

				for (int i = 0; i < M; i++) {
					rowA = AData[i];
					for (int j = 0; j < NB; j++) {
						if (_operator == '+')
							s = C.getEntry(i, j);
						else if (_operator == '-')
							s = -C.getEntry(i, j);
						for (int k = jc[j]; k < jc[j + 1]; k++) {
							r = ir[k];
							// A[r][j] = pr[k]
							s += rowA[r] * pr[k];
						}
						resData[i][j] = s;
					}
				}

			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			double* pr = ((SparseMatrix&) A).getPr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) B).getData();
				int c = -1;
				double s = 0;
				for (int i = 0; i < M; i++) {
					for (int j = 0; j < NB; j++) {
						if (_operator == '+')
							s = C.getEntry(i, j);
						else if (_operator == '-')
							s = -C.getEntry(i, j);
						for (int k = jr[i]; k < jr[i + 1]; k++) {
							c = ic[k];
							s += pr[valCSRIndices[k]] * BData[c][j];
						}
						resData[i][j] = s;
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				double* resRow = null;
				int* ir = null;
				int* jc = null;
				double* pr2 = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr2 = ((SparseMatrix&) B).getPr();
				// rowIdx of the right sparse matrix
				int rr = -1;
				// colIdx of the left sparse matrix
				int cl = -1;
				double s = 0;
				int kl = 0;
				int kr = 0;
				for (int i = 0; i < M; i++) {
					resRow = resData[i];
					for (int j = 0; j < NB; j++) {
						if (_operator == '+')
							s = C.getEntry(i, j);
						else if (_operator == '-')
							s = -C.getEntry(i, j);
						kl = jr[i];
						kr = jc[j];
						while (true) {
							if (kl >= jr[i + 1] || kr >= jc[j + 1]) {
								break;
							}
							cl = ic[kl];
							rr = ir[kr];
							if (cl < rr) {
								kl++;
							} else if (cl > rr) {
								kr++;
							} else {
								s += pr[valCSRIndices[kl]] * pr2[kr];
								kl++;
								kr++;
							}
						}
						resRow[j] = s;
					}
				}
			}
		}
	}
}

/**
 * res = A * B + v
 * @param res
 * @param A
 * @param B
 * @param v
 */
void affine(Matrix& res, Matrix& A, Matrix& B, double v) {
	if (typeid(res) == typeid(SparseMatrix)) {
		// ((SparseMatrix&) res).assignSparseMatrix(sparse(A.mtimes(B)));
		err("Sparse matrix for res is not supported.");
		exit(1);
	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* rowA = null;
		int NB = B.getColumnDimension();
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			if (typeid(B) == typeid(DenseMatrix)) {

				double** BData = ((DenseMatrix&) B).getData();
				double* columnB = new double[B.getRowDimension()];
				double s = 0;
				for (int j = 0; j < NB; j++) {
					for (int r = 0; r < B.getRowDimension(); r++) {
						columnB[r] = BData[r][j];
					}
					for (int i = 0; i < M; i++) {
						rowA = AData[i];
						s = v;
						for (int k = 0; k < N; k++) {
							// Using AData[k][j] costs 16.8 seconds
							// Referring AData[k][j] involves one integer multiplication!
							// s += rowData[k] * AData[k][j];
							// Using columnA[j] costs 3.4 seconds
							s += rowA[k] * columnB[k];
						}
						resData[i][j] = s;
					}
				}

			} else if (typeid(B) == typeid(SparseMatrix)) {

				int* ir = null;
				int* jc = null;
				double* pr = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr = ((SparseMatrix&) B).getPr();
				int r = -1;
				double s = 0;

				for (int i = 0; i < M; i++) {
					rowA = AData[i];
					for (int j = 0; j < NB; j++) {
						s = v;
						for (int k = jc[j]; k < jc[j + 1]; k++) {
							r = ir[k];
							// A[r][j] = pr[k]
							s += rowA[r] * pr[k];
						}
						resData[i][j] = s;
					}
				}

			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			double* pr = ((SparseMatrix&) A).getPr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) B).getData();
				int c = -1;
				double s = 0;
				for (int i = 0; i < M; i++) {
					for (int j = 0; j < NB; j++) {
						s = v;
						for (int k = jr[i]; k < jr[i + 1]; k++) {
							c = ic[k];
							s += pr[valCSRIndices[k]] * BData[c][j];
						}
						resData[i][j] = s;
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				double* resRow = null;
				int* ir = null;
				int* jc = null;
				double* pr2 = null;
				ir = ((SparseMatrix&) B).getIr();
				jc = ((SparseMatrix&) B).getJc();
				pr2 = ((SparseMatrix&) B).getPr();
				// rowIdx of the right sparse matrix
				int rr = -1;
				// colIdx of the left sparse matrix
				int cl = -1;
				double s = 0;
				int kl = 0;
				int kr = 0;
				for (int i = 0; i < M; i++) {
					resRow = resData[i];
					for (int j = 0; j < NB; j++) {
						s = v;
						kl = jr[i];
						kr = jc[j];
						while (true) {
							if (kl >= jr[i + 1] || kr >= jc[j + 1]) {
								break;
							}
							cl = ic[kl];
							rr = ir[kr];
							if (cl < rr) {
								kl++;
							} else if (cl > rr) {
								kr++;
							} else {
								s += pr[valCSRIndices[kl]] * pr2[kr];
								kl++;
								kr++;
							}
						}
						resRow[j] = s;
					}
				}
			}
		}
	}
}

/**
 * res = a * A + b * B
 * @param res
 * @param a
 * @param A
 * @param b
 * @param B
 */
void affine(Matrix& res, double a, Matrix& A, double b, Matrix& B) {
	if (b == 0) {
		times(res, a, A);
		return;
	} else if (b == 1) {
		affine(res, a, A, '+', B);
		return;
	} else if (b == -1) {
		affine(res, a, A, '-', B);
		return;
	}
	if (a == 0) {
		times(res, b, B);
		return;
	} else if (a == 1) {
		affine(res, b, B, '+', A);
		return;
	} else if (a == -1) {
		affine(res, b, B, '-', A);
		return;
	}
	if (typeid(res) == typeid(DenseMatrix)) { // res = a * A + b * B
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) B).getData();
				double* BRow = null;
				for (int i = 0; i < M; i++) {
					ARow = AData[i];
					BRow = BData[i];
					resRow = resData[i];
					for (int j = 0; j < N; j++) {
						resRow[j] = a * ARow[j] + b * BRow[j];
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				int* ic = ((SparseMatrix&) B).getIc();
				int* jr = ((SparseMatrix&) B).getJr();
				int* valCSRIndices = ((SparseMatrix&) B).getValCSRIndices();
				double* pr = ((SparseMatrix&) B).getPr();
				int j = 0;
				for (int i = 0; i < M; i++) {
					ARow = AData[i];
					resRow = resData[i];
					times(resRow, a, ARow, N);
					for (int k = jr[i]; k < jr[i + 1]; k++) {
						j = ic[k];
						resRow[j] += b * pr[valCSRIndices[k]];
					}
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) A).getData();
				double* BRow = null;
				int* ic = ((SparseMatrix&) A).getIc();
				int* jr = ((SparseMatrix&) A).getJr();
				int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
				double* pr = ((SparseMatrix&) A).getPr();
				int j = 0;
				for (int i = 0; i < M; i++) {
					BRow = BData[i];
					resRow = resData[i];
					times(resRow, b, BRow, N);
					for (int k = jr[i]; k < jr[i + 1]; k++) {
						j = ic[k];
						resRow[j] += a * pr[valCSRIndices[k]];
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				res.clear();
				// res = a * A + b * B where both A and B are sparse matrices
				int* ir1 = null;
				int* jc1 = null;
				double* pr1 = null;
				ir1 = ((SparseMatrix&) A).getIr();
				jc1 = ((SparseMatrix&) A).getJc();
				pr1 = ((SparseMatrix&) A).getPr();
				int* ir2 = null;
				int* jc2 = null;
				double* pr2 = null;
				ir2 = ((SparseMatrix&) B).getIr();
				jc2 = ((SparseMatrix&) B).getJc();
				pr2 = ((SparseMatrix&) B).getPr();

				int k1 = 0;
				int k2 = 0;
				int r1 = -1;
				int r2 = -1;
				int i = -1;
				double v = 0;

				for (int j = 0; j < N; j++) {
					k1 = jc1[j];
					k2 = jc2[j];

					// Both A and B's j-th columns are empty.
					if (k1 == jc1[j + 1] && k2 == jc2[j + 1])
						continue;

					while (k1 < jc1[j + 1] || k2 < jc2[j + 1]) {

						if (k2 == jc2[j + 1]) { // B's j-th column has been processed.
							i = ir1[k1];
							v = a * pr1[k1];
							k1++;
						} else if (k1 == jc1[j + 1]) { // A's j-th column has been processed.
							i = ir2[k2];
							v = b * pr2[k2];
							k2++;
						} else { // Both A and B's j-th columns have not been fully processed.
							r1 = ir1[k1];
							r2 = ir2[k2];
							if (r1 < r2) {
								i = r1;
								v = a * pr1[k1];
								k1++;
							} else if (r1 == r2) {
								i = r1;
								v = a * pr1[k1] + b * pr2[k2];
								k1++;
								k2++;
							} else {
								i = r2;
								v = b * pr2[k2];
								k2++;
							}
						}
						if (v != 0)
							resData[i][j] = v;
					}
				}
			}
		}
	}
}

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
void affine(Matrix& res, double a, Matrix& A, char _operator, Matrix& B) {
	if (_operator == '+') {
		if (a == 0) {
			assign(res, B);
			return;
		} else if (a == 1) {
			plus(res, A, B);
			return;
		} else if (a == -1) {
			minus(res, B, A);
			return;
		}
		if (typeid(res) == typeid(DenseMatrix)) { // res = a * A + B
			int M = A.getRowDimension();
			int N = A.getColumnDimension();
			double** resData = ((DenseMatrix&) res).getData();
			double* resRow = null;
			if (typeid(A) == typeid(DenseMatrix)) {
				double** AData = ((DenseMatrix&) A).getData();
				double* ARow = null;
				if (typeid(B) == typeid(DenseMatrix)) {
					double** BData = ((DenseMatrix&) B).getData();
					double* BRow = null;
					for (int i = 0; i < M; i++) {
						ARow = AData[i];
						BRow = BData[i];
						resRow = resData[i];
						for (int j = 0; j < N; j++) {
							resRow[j] = a * ARow[j] + BRow[j];
						}
					}
				} else if (typeid(B) == typeid(SparseMatrix)) {
					int* ic = ((SparseMatrix&) B).getIc();
					int* jr = ((SparseMatrix&) B).getJr();
					int* valCSRIndices = ((SparseMatrix&) B).getValCSRIndices();
					double* pr = ((SparseMatrix&) B).getPr();
					int j = 0;
					for (int i = 0; i < M; i++) {
						ARow = AData[i];
						resRow = resData[i];
						times(resRow, a, ARow, N);
						for (int k = jr[i]; k < jr[i + 1]; k++) {
							j = ic[k];
							resRow[j] += pr[valCSRIndices[k]];
						}
					}
				}
			} else if (typeid(A) == typeid(SparseMatrix)) {
				if (typeid(B) == typeid(DenseMatrix)) {
					double** BData = ((DenseMatrix&) A).getData();
					double* BRow = null;
					int* ic = ((SparseMatrix&) A).getIc();
					int* jr = ((SparseMatrix&) A).getJr();
					int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
					double* pr = ((SparseMatrix&) A).getPr();
					int j = 0;
					for (int i = 0; i < M; i++) {
						BRow = BData[i];
						resRow = resData[i];
						assign(resRow, BRow, N);
						for (int k = jr[i]; k < jr[i + 1]; k++) {
							j = ic[k];
							resRow[j] += a * pr[valCSRIndices[k]];
						}
					}
				} else if (typeid(B) == typeid(SparseMatrix)) {
					res.clear();
					// res = a * A + B where both A and B are sparse matrices
					int* ir1 = null;
					int* jc1 = null;
					double* pr1 = null;
					ir1 = ((SparseMatrix&) A).getIr();
					jc1 = ((SparseMatrix&) A).getJc();
					pr1 = ((SparseMatrix&) A).getPr();
					int* ir2 = null;
					int* jc2 = null;
					double* pr2 = null;
					ir2 = ((SparseMatrix&) B).getIr();
					jc2 = ((SparseMatrix&) B).getJc();
					pr2 = ((SparseMatrix&) B).getPr();

					int k1 = 0;
					int k2 = 0;
					int r1 = -1;
					int r2 = -1;
					int i = -1;
					double v = 0;

					for (int j = 0; j < N; j++) {
						k1 = jc1[j];
						k2 = jc2[j];

						// Both A and B's j-th columns are empty.
						if (k1 == jc1[j + 1] && k2 == jc2[j + 1])
							continue;

						while (k1 < jc1[j + 1] || k2 < jc2[j + 1]) {

							if (k2 == jc2[j + 1]) { // B's j-th column has been processed.
								i = ir1[k1];
								v = a * pr1[k1];
								k1++;
							} else if (k1 == jc1[j + 1]) { // A's j-th column has been processed.
								i = ir2[k2];
								v = pr2[k2];
								k2++;
							} else { // Both A and B's j-th columns have not been fully processed.
								r1 = ir1[k1];
								r2 = ir2[k2];
								if (r1 < r2) {
									i = r1;
									v = a * pr1[k1];
									k1++;
								} else if (r1 == r2) {
									i = r1;
									v = a * pr1[k1] + pr2[k2];
									k1++;
									k2++;
								} else {
									i = r2;
									v = pr2[k2];
									k2++;
								}
							}
							if (v != 0)
								resData[i][j] = v;
						}
					}
				}
			}
		}
	} else if (_operator == '-') { // res = a * A - B
		if (a == 0) {
			uminus(res, B);
			return;
		} else if (a == 1) {
			minus(res, A, B);
			return;
		}
		if (typeid(res) == typeid(DenseMatrix)) {
			int M = A.getRowDimension();
			int N = A.getColumnDimension();
			double** resData = ((DenseMatrix&) res).getData();
			double* resRow = null;
			if (typeid(A) == typeid(DenseMatrix)) {
				double** AData = ((DenseMatrix&) A).getData();
				double* ARow = null;
				if (typeid(B) == typeid(DenseMatrix)) {
					double** BData = ((DenseMatrix&) B).getData();
					double* BRow = null;
					for (int i = 0; i < M; i++) {
						ARow = AData[i];
						BRow = BData[i];
						resRow = resData[i];
						for (int j = 0; j < N; j++) {
							resRow[j] = a * ARow[j] - BRow[j];
						}
					}
				} else if (typeid(B) == typeid(SparseMatrix)) {
					int* ic = ((SparseMatrix&) B).getIc();
					int* jr = ((SparseMatrix&) B).getJr();
					int* valCSRIndices = ((SparseMatrix&) B).getValCSRIndices();
					double* pr = ((SparseMatrix&) B).getPr();
					int j = 0;
					for (int i = 0; i < M; i++) {
						ARow = AData[i];
						resRow = resData[i];
						times(resRow, a, ARow, N);
						for (int k = jr[i]; k < jr[i + 1]; k++) {
							j = ic[k];
							resRow[j] -= pr[valCSRIndices[k]];
						}
					}
				}
			} else if (typeid(A) == typeid(SparseMatrix)) {
				if (typeid(B) == typeid(DenseMatrix)) {
					double** BData = ((DenseMatrix&) A).getData();
					double* BRow = null;
					int* ic = ((SparseMatrix&) A).getIc();
					int* jr = ((SparseMatrix&) A).getJr();
					int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
					double* pr = ((SparseMatrix&) A).getPr();
					int j = 0;
					for (int i = 0; i < M; i++) {
						BRow = BData[i];
						resRow = resData[i];
						uminus(resRow, BRow, N);
						for (int k = jr[i]; k < jr[i + 1]; k++) {
							j = ic[k];
							resRow[j] += a * pr[valCSRIndices[k]];
						}
					}
				} else if (typeid(B) == typeid(SparseMatrix)) {
					res.clear();
					// res = a * A - B where both A and B are sparse matrices
					int* ir1 = null;
					int* jc1 = null;
					double* pr1 = null;
					ir1 = ((SparseMatrix&) A).getIr();
					jc1 = ((SparseMatrix&) A).getJc();
					pr1 = ((SparseMatrix&) A).getPr();
					int* ir2 = null;
					int* jc2 = null;
					double* pr2 = null;
					ir2 = ((SparseMatrix&) B).getIr();
					jc2 = ((SparseMatrix&) B).getJc();
					pr2 = ((SparseMatrix&) B).getPr();

					int k1 = 0;
					int k2 = 0;
					int r1 = -1;
					int r2 = -1;
					int i = -1;
					double v = 0;

					for (int j = 0; j < N; j++) {
						k1 = jc1[j];
						k2 = jc2[j];

						// Both A and B's j-th columns are empty.
						if (k1 == jc1[j + 1] && k2 == jc2[j + 1])
							continue;

						while (k1 < jc1[j + 1] || k2 < jc2[j + 1]) {

							if (k2 == jc2[j + 1]) { // B's j-th column has been processed.
								i = ir1[k1];
								v = a * pr1[k1];
								k1++;
							} else if (k1 == jc1[j + 1]) { // A's j-th column has been processed.
								i = ir2[k2];
								v = -pr2[k2];
								k2++;
							} else { // Both A and B's j-th columns have not been fully processed.
								r1 = ir1[k1];
								r2 = ir2[k2];
								if (r1 < r2) {
									i = r1;
									v = a * pr1[k1];
									k1++;
								} else if (r2 < r1) {
									i = r2;
									v = -pr2[k2];
									k2++;
								} else { // if (r1 == r2)
									i = r1;
									v = a * pr1[k1] - pr2[k2];
									k1++;
									k2++;
								}
							}
							if (v != 0)
								resData[i][j] = v;
						}
					}
				}
			}
		}
	}
}

/**
 * res = A + b * B
 * @param res
 * @param A
 * @param b
 * @param B
 */
void affine(Matrix& res, Matrix& A, double b, Matrix& B) {
	affine(res, b, B, '+', A);
}

/**
 * res = a * A + b
 * @param res
 * @param a
 * @param A
 * @param b
 */
void affine(Matrix& res, double a, Matrix& A, double b) {
	if (a == 1) {
		plus(res, A, b);
		return;
	}
	if (b == 0) {
		times(res, a, A);
		return;
	}
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = a * ARow[j] + b;
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					assignVector(resRow, N, b);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						resRow[c] = b;
					}
					resRow[currentColumnIdx] = a * pr[valCSRIndices[k]] + b;
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					resRow[c] = b;
				}
			}
		}
	}
}

/**
 * res = A + B
 * @param res
 * @param A
 * @param B
 */
void plus(Matrix& res, Matrix& A, Matrix& B) {
	if (typeid(res) == typeid(SparseMatrix)) {
		((SparseMatrix&) res).assignSparseMatrix(sparse(A.plus(B)));
	} else if (typeid(res) == typeid(DenseMatrix)) {
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) B).getData();
				double* BRow = null;
				for (int i = 0; i < M; i++) {
					ARow = AData[i];
					BRow = BData[i];
					resRow = resData[i];
					for (int j = 0; j < N; j++) {
						resRow[j] = ARow[j] + BRow[j];
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				int* ic = ((SparseMatrix&) B).getIc();
				int* jr = ((SparseMatrix&) B).getJr();
				int* valCSRIndices = ((SparseMatrix&) B).getValCSRIndices();
				double* pr = ((SparseMatrix&) B).getPr();
				int j = 0;
				for (int i = 0; i < M; i++) {
					ARow = AData[i];
					resRow = resData[i];
					assign(resRow, ARow, N);
					for (int k = jr[i]; k < jr[i + 1]; k++) {
						j = ic[k];
						resRow[j] += pr[valCSRIndices[k]];
					}
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			if (typeid(B) == typeid(DenseMatrix)) {
				plus(res, B, A);
			} else if (typeid(B) == typeid(SparseMatrix)) {
				res.clear();
				// res = A + B where both A and B are sparse matrices
				int* ir1 = null;
				int* jc1 = null;
				double* pr1 = null;
				ir1 = ((SparseMatrix&) A).getIr();
				jc1 = ((SparseMatrix&) A).getJc();
				pr1 = ((SparseMatrix&) A).getPr();
				int* ir2 = null;
				int* jc2 = null;
				double* pr2 = null;
				ir2 = ((SparseMatrix&) B).getIr();
				jc2 = ((SparseMatrix&) B).getJc();
				pr2 = ((SparseMatrix&) B).getPr();

				int k1 = 0;
				int k2 = 0;
				int r1 = -1;
				int r2 = -1;
				int i = -1;
				double v = 0;

				for (int j = 0; j < N; j++) {
					k1 = jc1[j];
					k2 = jc2[j];

					// Both A and B's j-th columns are empty.
					if (k1 == jc1[j + 1] && k2 == jc2[j + 1])
						continue;

					while (k1 < jc1[j + 1] || k2 < jc2[j + 1]) {

						if (k2 == jc2[j + 1]) { // B's j-th column has been processed.
							i = ir1[k1];
							v = pr1[k1];
							k1++;
						} else if (k1 == jc1[j + 1]) { // A's j-th column has been processed.
							i = ir2[k2];
							v = pr2[k2];
							k2++;
						} else { // Both A and B's j-th columns have not been fully processed.
							r1 = ir1[k1];
							r2 = ir2[k2];
							if (r1 < r2) {
								i = r1;
								v = pr1[k1];
								k1++;
							} else if (r1 == r2) {
								i = r1;
								v = pr1[k1] + pr2[k2];
								k1++;
								k2++;
							} else {
								i = r2;
								v = pr2[k2];
								k2++;
							}
						}
						if (v != 0)
							resData[i][j] = v;
					}
				}
			}
		}
	}
}

/**
 * res = A + v;
 * @param res
 * @param A
 * @param v
 */
void plus(Matrix& res, Matrix& A, double v) {
	if (v == 0) {
		assign(res, A);
		return;
	}
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = ARow[j] + v;
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					assignVector(resRow, N, v);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						resRow[c] = v;
					}
					resRow[currentColumnIdx] = pr[valCSRIndices[k]] + v;
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					resRow[c] = v;
				}
			}
		}
	}
}

/**
 * res = res + A
 * @param res
 * @param A
 */
void plusAssign(Matrix& res, Matrix& A) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] += ARow[j];
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			int j = 0;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					j = ic[k];
					resRow[j] += pr[valCSRIndices[k]];
				}
			}
		}
	}
}

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

void plusAssign(double* res, double* V, int len) {
	for (int i = 0; i < len; i++)
		res[i] += V[i];
}*/

/**
 * res = res + v
 * @param res
 * @param v
 */
void plusAssign(Matrix& res, double v) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(SparseMatrix)) {
		/*double* pr = ((SparseMatrix&) res).getPr();
			int nnz = ((SparseMatrix&) res).getNNZ();
			for (int k = 0; k < nnz; k++) {
				pr[k] += v;
			}*/
	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				resRow[j] += v;
			}
		}
	}
}

/**
 * res += a * A.
 *
 * @param res
 * @param a
 * @param A
 */
void plusAssign(Matrix& res, double a, Matrix& A) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (a == 0) {
		return;
	} else if (a == 1) {
		plusAssign(res, A);
		return;
	} else if (a == -1) {
		minusAssign(res, A);
		return;
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] += a * ARow[j];
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			int j = 0;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					j = ic[k];
					resRow[j] += a * pr[valCSRIndices[k]];
				}
			}
		}
	}
}

/**
 * res = A - B
 * @param res
 * @param A
 * @param B
 */
void minus(Matrix& res, Matrix& A, Matrix& B) {
	if (typeid(res) == typeid(SparseMatrix)) {
		((SparseMatrix&) res).assignSparseMatrix(sparse(A.minus(B)));
	} else if (typeid(res) == typeid(DenseMatrix)) {
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) B).getData();
				double* BRow = null;
				for (int i = 0; i < M; i++) {
					ARow = AData[i];
					BRow = BData[i];
					resRow = resData[i];
					for (int j = 0; j < N; j++) {
						resRow[j] = ARow[j] - BRow[j];
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				int* ic = ((SparseMatrix&) B).getIc();
				int* jr = ((SparseMatrix&) B).getJr();
				int* valCSRIndices = ((SparseMatrix&) B).getValCSRIndices();
				double* pr = ((SparseMatrix&) B).getPr();
				int j = 0;
				for (int i = 0; i < M; i++) {
					ARow = AData[i];
					resRow = resData[i];
					assign(resRow, ARow, N);
					for (int k = jr[i]; k < jr[i + 1]; k++) {
						j = ic[k];
						resRow[j] -= pr[valCSRIndices[k]];
					}
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			if (typeid(B) == typeid(DenseMatrix)) {
				double** BData = ((DenseMatrix&) A).getData();
				double* BRow = null;
				int* ic = ((SparseMatrix&) A).getIc();
				int* jr = ((SparseMatrix&) A).getJr();
				int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
				double* pr = ((SparseMatrix&) A).getPr();
				int j = 0;
				for (int i = 0; i < M; i++) {
					BRow = BData[i];
					resRow = resData[i];
					uminus(resRow, BRow, N);
					for (int k = jr[i]; k < jr[i + 1]; k++) {
						j = ic[k];
						resRow[j] += pr[valCSRIndices[k]];
					}
				}
			} else if (typeid(B) == typeid(SparseMatrix)) {
				res.clear();
				// res = A - B where both A and B are sparse matrices
				int* ir1 = null;
				int* jc1 = null;
				double* pr1 = null;
				ir1 = ((SparseMatrix&) A).getIr();
				jc1 = ((SparseMatrix&) A).getJc();
				pr1 = ((SparseMatrix&) A).getPr();
				int* ir2 = null;
				int* jc2 = null;
				double* pr2 = null;
				ir2 = ((SparseMatrix&) B).getIr();
				jc2 = ((SparseMatrix&) B).getJc();
				pr2 = ((SparseMatrix&) B).getPr();

				int k1 = 0;
				int k2 = 0;
				int r1 = -1;
				int r2 = -1;
				int i = -1;
				double v = 0;

				for (int j = 0; j < N; j++) {
					k1 = jc1[j];
					k2 = jc2[j];

					// Both A and B's j-th columns are empty.
					if (k1 == jc1[j + 1] && k2 == jc2[j + 1])
						continue;

					while (k1 < jc1[j + 1] || k2 < jc2[j + 1]) {

						if (k2 == jc2[j + 1]) { // B's j-th column has been processed.
							i = ir1[k1];
							v = pr1[k1];
							k1++;
						} else if (k1 == jc1[j + 1]) { // A's j-th column has been processed.
							i = ir2[k2];
							v = -pr2[k2];
							k2++;
						} else { // Both A and B's j-th columns have not been fully processed.
							r1 = ir1[k1];
							r2 = ir2[k2];
							if (r1 < r2) {
								i = r1;
								v = pr1[k1];
								k1++;
							} else if (r2 < r1) {
								i = r2;
								v = -pr2[k2];
								k2++;
							} else { // if (r1 == r2)
								i = r1;
								v = pr1[k1] - pr2[k2];
								k1++;
								k2++;
							}
						}
						if (v != 0)
							resData[i][j] = v;
					}
				}
			}
		}
	}
}

/**
 * res = A - v
 * @param res
 * @param A
 * @param v
 */
void minus(Matrix& res, Matrix& A, double v) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = ARow[j] - v;
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					assignVector(resRow, N, -v);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						resRow[c] = -v;
					}
					resRow[currentColumnIdx] = pr[valCSRIndices[k]] - v;
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					resRow[c] = -v;
				}
			}
		}
	}
}

/**
 * res = v - A
 * @param res
 * @param v
 * @param A
 */
void minus(Matrix& res, double v, Matrix& A) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = v - ARow[j];
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				if (jr[i] ==  jr[i + 1]) {
					assignVector(resRow, N, v);
					continue;
				}
				int lastColumnIdx = -1;
				int currentColumnIdx = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					currentColumnIdx = ic[k];
					for (int c = lastColumnIdx + 1; c < currentColumnIdx; c++) {
						resRow[c] = v;
					}
					resRow[currentColumnIdx] = v - pr[valCSRIndices[k]];
					lastColumnIdx = currentColumnIdx;
				}
				for (int c = lastColumnIdx + 1; c < N; c++) {
					resRow[c] = v;
				}
			}
		}
	}
}

/**
 * res = res - A
 * @param res
 * @param A
 */
void minusAssign(Matrix& res, Matrix& A) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] -= ARow[j];
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			int j = 0;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					j = ic[k];
					resRow[j] -= pr[valCSRIndices[k]];
				}
			}
		}
	}
}

/**
 * res = res - v
 * @param res
 * @param v
 */
void minusAssign(Matrix& res, double v) {
	if (v == 0) {
		return;
	}
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (typeid(res) == typeid(SparseMatrix)) {
		/*double* pr = ((SparseMatrix&) res).getPr();
			int nnz = ((SparseMatrix&) res).getNNZ();
			for (int k = 0; k < nnz; k++) {
				pr[k] -= v;
			}*/
	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				resRow[j] -= v;
			}
		}
	}
}

/**
 * res -= a * A.
 *
 * @param res
 * @param a
 * @param A
 */
void minusAssign(Matrix& res, double a, Matrix& A) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (a == 0) {
		return;
	} else if (a == 1) {
		minusAssign(res, A);
		return;
	} else if (a == -1) {
		plusAssign(res, A);
		return;
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] -= a * ARow[j];
				}
			}
		} else if (typeid(A) == typeid(SparseMatrix)) {
			int* ic = ((SparseMatrix&) A).getIc();
			int* jr = ((SparseMatrix&) A).getJr();
			int* valCSRIndices = ((SparseMatrix&) A).getValCSRIndices();
			double* pr = ((SparseMatrix&) A).getPr();
			int j = 0;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					j = ic[k];
					resRow[j] -= a * pr[valCSRIndices[k]];
				}
			}
		}
	}
}

/**
 * res = log(A).
 * @param res
 * @param A
 */
void log(Matrix& res, Matrix& A) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = log(ARow[j]);
				}
			}
		}
	}
}

/**
 * res = log(V).
 *
 * @param res
 * @param V
 * @param len length of res and V
 */
void log(double* res, double* V, int len) {
	for (int i = 0; i < len; i++) {
		res[i] = log(V[i]);
	}
}

/**
 * res = log(res).
 *
 * @param res
 *
 * @param len length of res
 */
void logAssign(double* res, int len) {
	for (int i = 0; i < len; i++) {
		res[i] = log(res[i]);
	}
}

/**
 * res = log(res).
 * @param res
 */
void logAssign(Matrix& res) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();

	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				resRow[j] = log(resRow[j]);
			}
		}
	}
}

/**
 * res = exp(A).
 * @param res
 * @param A
 */
void exp(Matrix& res, Matrix& A) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();
	if (M != A.getRowDimension() || N != A.getColumnDimension()) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(res) == typeid(SparseMatrix)) {

	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		if (typeid(A) == typeid(DenseMatrix)) {
			double** AData = ((DenseMatrix&) A).getData();
			double* ARow = null;
			for (int i = 0; i < M; i++) {
				resRow = resData[i];
				ARow = AData[i];
				for (int j = 0; j < N; j++) {
					resRow[j] = exp(ARow[j]);
				}
			}
		}
	}
}

/**
 * res = exp(res).
 *
 * @param res
 */
void expAssign(Matrix& res) {
	int M = res.getRowDimension();
	int N = res.getColumnDimension();

	if (typeid(res) == typeid(SparseMatrix)) {
		err("The expAssign routine doesn't support sparse matrix.");
		exit(1);
	} else if (typeid(res) == typeid(DenseMatrix)) {
		double** resData = ((DenseMatrix&) res).getData();
		double* resRow = null;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < N; j++) {
				resRow[j] = exp(resRow[j]);
			}
		}
	}
}

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
void sigmoid(Matrix& res, Matrix& A) {
	if (typeid(res) == typeid(DenseMatrix)) {
		assign(res, A);
		double** data = ((DenseMatrix&) res).getData();
		int M = A.getRowDimension();
		int N = A.getColumnDimension();
		double* row_i = null;
		double old = 0;
		double current = 0;
		double max = 0;
		double sum = 0;
		double v = 0;
		for (int i = 0; i < M; i++) {
			row_i = data[i];
			old = row_i[0];
			current = 0;
			max = old;
			for (int j = 1; j < N; j++) {
				current = row_i[j];
				if (max < current)
					max = current;
				old = current;
			}
			sum = 0;
			for (int j = 0; j < N; j++) {
				v = exp(row_i[j] - max);
				sum += v;
				row_i[j] = v;
			}
			for (int j = 0; j < N; j++) {
				row_i[j] /= sum;
			}
		}
	} else {
		err("Sorry, sparse matrix is not support for res.");
		exit(1);
	}
}
