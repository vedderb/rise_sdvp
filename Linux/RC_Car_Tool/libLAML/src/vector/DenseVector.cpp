/*
 * DenseVector.cpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Mingjie Qian
 */

#include "DenseVector.h"
#include "SparseVector.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Utility.h"
#include "Printer.h"
#include <typeinfo>

DenseVector::DenseVector() : Vector() {
}

DenseVector::DenseVector(int dim, double v) {
	pr = new double[dim];
	for (int k = 0; k < dim; k++) {
		pr[k] = v;
	}
	this->dim = dim;
}

DenseVector::DenseVector(int dim) : Vector() {
	pr = new double[dim];
	for (int k = 0; k < dim; k++) {
		pr[k] = 0;
	}
	this->dim = dim;
}

DenseVector::DenseVector(double* pr, int dim) : Vector() {
	this->pr = pr;
	this->dim = dim;
}

/*DenseVector::DenseVector(double pr[], int dim) {
	this->pr = pr;
	this->dim = dim;
}*/

double* DenseVector::getPr() {
	return pr;
}

DenseVector::~DenseVector() {
	if (pr != null) {
		delete[] pr;
		pr = null;
	}
	// disp("This dense vector is released.");
}

Vector& DenseVector::copy() {
	return *new DenseVector(clone(pr, dim), dim);
}

Vector& DenseVector::times(Vector& V) {
	if (V.getDim() != dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(V) == typeid(DenseVector)) {
		Vector& res = *new DenseVector(dim);
		double* resPr = ((DenseVector&) res).getPr();
		double* VPr = ((DenseVector&) V).getPr();
		for (int k = 0; k < dim; k++) {
			resPr[k] = pr[k] * VPr[k];
		}
		return res;
	} else {
		Vector& res = V.copy();
		double* resPr = ((SparseVector&) res).getPr();
		int* resIr = ((SparseVector&) res).getIr();
		for (int k = 0; k < ((SparseVector&) V).getNNZ(); k++) {
			resPr[k] *= pr[resIr[k]];
		}
		((SparseVector&) res).clean();
		return res;
	} /*else {
		err("Wrong vector type!");
		exit(1);
	}*/
}

Vector& DenseVector::plus(Vector& V) {
	if (V.getDim() != dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(V) == typeid(DenseVector)) {
		Vector& res = *new DenseVector(dim);
		double* resPr = ((DenseVector&) res).getPr();
		double* VPr = ((DenseVector&) V).getPr();
		for (int k = 0; k < dim; k++) {
			resPr[k] = pr[k] + VPr[k];
		}
		return res;
	} else {
		double* res = clone(pr, dim);
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		for (int k = 0; k < ((SparseVector&) V).getNNZ(); k++) {
			res[ir[k]] += pr[k];
		}
		return *new DenseVector(res, dim);
	}
}

Vector& DenseVector::minus(Vector& V) {
	if (V.getDim() != dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(V) == typeid(DenseVector)) {
		Vector& res = *new DenseVector(dim);
		double* resPr = ((DenseVector&) res).getPr();
		double* VPr = ((DenseVector&) V).getPr();
		for (int k = 0; k < dim; k++) {
			resPr[k] = pr[k] - VPr[k];
		}
		return res;
	} else {
		double* res = clone(pr, dim);
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		for (int k = 0; k < ((SparseVector&) V).getNNZ(); k++) {
			res[ir[k]] -= pr[k];
		}
		return *new DenseVector(res, dim);
	}
}

double DenseVector::get(int i) {
	if (i < 0 || i > dim - 1) {
		err("Index is out of range.");
		exit(1);
	}
	return pr[i];
}

void DenseVector::set(int i, double v) {
	if (i < 0 || i > dim - 1) {
		err("Index is out of range.");
		exit(1);
	}
	pr[i] = v;
}

Vector& DenseVector::operate(Matrix& A) {

	int dim = getDim();
	int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (M != dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	double* res = new double[N];
	for (int k = 0; k < dim; k++) {
		res[k] = 0;
	}

	if (typeid(A) == typeid(DenseMatrix)) {
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;
		double v = 0;
		for (int i = 0; i < M; i++) {
			ARow = AData[i];
			v = this->pr[i];
			for (int j = 0; j < N; j++) {
				res[j] += v * ARow[j];
			}
		}
	} else {
		int* ir = ((SparseMatrix&) A).getIr();
		int* jc = ((SparseMatrix&) A).getJc();
		double* pr = ((SparseMatrix&) A).getPr();
		for (int j = 0; j < N; j++) {
			for (int k = jc[j]; k < jc[j + 1]; k++) {
				res[j] += this->pr[ir[k]] * pr[k];
			}
		}
	}
	return *new DenseVector(res, N);
}

void DenseVector::clear() {
	for (int k = 0; k < dim; k++) {
		pr[k] = 0;
	}
}

Vector& DenseVector::times(double v) {
	if (v == 0) {
		return *new DenseVector(getDim(), 0);
	}
	double* resData = clone(pr, dim);
	for (int i = 0; i < dim; i++) {
		resData[i] *= v;
	}
	return *new DenseVector(resData, dim);
}
