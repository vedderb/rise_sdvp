/*
 * SparseVector.cpp
 *
 *  Created on: Feb 9, 2014
 *      Author: Mingjie Qian
 */

#include "SparseVector.h"
#include "DenseVector.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Utility.h"
#include "Printer.h"
#include <typeinfo>
#include <list>
#include <map>

using namespace std;

SparseVector::SparseVector() {
	ir = NULL;
	pr = NULL;
	nnz = 0;
	nzmax = 0;
	dim = 0;
}

SparseVector::SparseVector(int dim){
	ir = new int[0];
	pr = new double[0];
	nnz = 0;
	this->dim = dim;
	nzmax = 0;
}

SparseVector::SparseVector(int* ir, double* pr, int nnz, int dim) : Vector() {
	this->ir = ir;
	this->pr = pr;
	this->nnz = nnz;
	this->dim = dim;
	nzmax = nnz;
}

SparseVector::SparseVector(SparseVector& V) {
	ir = V.ir;
	pr = V.pr;
	nnz = V.nnz;
	dim = V.dim;
	nzmax = V.nzmax;
}

SparseVector::~SparseVector() {
	delete[] ir;
	delete[] pr;
	// disp("This sparse vector is released.");
}

void SparseVector::setSparseVector(SparseVector& V) {
	ir = V.ir;
	pr = V.pr;
	nnz = V.nnz;
	dim = V.dim;
	nzmax = V.nzmax;
}

/**
 * Assign this sparse vector by a sparse vector V in the sense that
 * all interior arrays of this vector are deep copy of the given
 * sparse vector V.
 *
 * @param V a sparse Vector
 */
void SparseVector::assignSparseVector(SparseVector& V) {
	ir = clone(V.ir, V.getNNZ());
	pr = clone(V.pr, V.getNNZ());
	nnz = V.nnz;
	dim = V.dim;
	nzmax = V.nzmax;
}

/**
 * Change dimensionality of this sparse vector.
 *
 * @param dim new dimensionality
 */
void SparseVector::setDim(int dim) {
	if (dim > this->dim)
		this->dim = dim;
}

Vector& SparseVector::copy() {
	return *new SparseVector(clone(ir, nnz), clone(pr, nnz), nnz, dim);
}

Vector& SparseVector::times(Vector& V) {
	if (V.getDim() != this->dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(V) == typeid(DenseVector)) {
		return V.times((Vector&)(*this));
	} else {
		list<pair<int, double> > list;
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz2 = ((SparseVector&) V).getNNZ();
		if (this->nnz != 0 && nnz2 != 0) {
			int k1 = 0;
			int k2 = 0;
			int r1 = 0;
			int r2 = 0;
			double v = 0;
			int i = -1;
			while (k1 < this->nnz && k2 < nnz2) {
				r1 = this->ir[k1];
				r2 = ir[k2];
				if (r1 < r2) {
					k1++;
				} else if (r1 == r2) {
					i = r1;
					v = this->pr[k1] * pr[k2];
					k1++;
					k2++;
					if (v != 0) {
						list.push_back(make_pair(i, v));
					}
				} else {
					k2++;
				}
			}
		}
		int nnz = list.size();
		int dim = this->getDim();
		int* ir_res = new int[nnz];
		double* pr_res = new double[nnz];
		int k = 0;
		for (std::list<pair<int, double> >::iterator iter = list.begin(); iter != list.end(); iter++) {
			ir_res[k] = iter->first;
			pr_res[k] = iter->second;
			k++;
		}
		return *new SparseVector(ir_res, pr_res, nnz, dim);
	}
}

Vector& SparseVector::plus(Vector& V) {
	if (V.getDim() != this->dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(V) == typeid(DenseVector)) {
		return V.plus((Vector&)*this);
	} else {
		list<pair<int, double> > list;
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz2 = ((SparseVector&) V).getNNZ();
		if (!(this->nnz == 0 && nnz2 == 0)) {
			int k1 = 0;
			int k2 = 0;
			int r1 = 0;
			int r2 = 0;
			double v = 0;
			int i = -1;
			while (k1 < this->nnz || k2 < nnz2) {
				if (k2 == nnz2) { // V has been processed.
					i = this->ir[k1];
					v = this->pr[k1];
					k1++;
				} else if (k1 == this->nnz) { // this has been processed.
					i = ir[k2];
					v = pr[k2];
					k2++;
				} else { // Both this and V have not been fully processed.
					r1 = this->ir[k1];
					r2 = ir[k2];
					if (r1 < r2) {
						i = r1;
						v = this->pr[k1];
						k1++;
					} else if (r1 == r2) {
						i = r1;
						v = this->pr[k1] + pr[k2];
						k1++;
						k2++;
					} else {
						i = r2;
						v = pr[k2];
						k2++;
					}
				}
				if (v != 0) {
					list.push_back(make_pair(i, v));
				}
			}
		}
		int nnz = list.size();
		int dim = this->getDim();
		int* ir_res = new int[nnz];
		double* pr_res = new double[nnz];
		int k = 0;
		for (std::list<pair<int, double> >::iterator iter = list.begin(); iter != list.end(); iter++) {
			ir_res[k] = iter->first;
			pr_res[k] = iter->second;
			k++;
		}
		return *new SparseVector(ir_res, pr_res, nnz, dim);
	}
}

Vector& SparseVector::minus(Vector& V) {
	if (V.getDim() != this->dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(V) == typeid(DenseVector)) {
		double* VPr = ((DenseVector&) V).getPr();
		double* resPr = new double[dim];
		for (int k = 0; k < dim; k++) {
			resPr[k] = -VPr[k];
		}
		for (int k = 0; k < nnz; k++) {
			resPr[ir[k]] += pr[k];
		}
		return *new DenseVector(resPr, dim);
	} else {
		list<pair<int, double> > list;
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz2 = ((SparseVector&) V).getNNZ();
		if (!(this->nnz == 0 && nnz2 == 0)) {
			int k1 = 0;
			int k2 = 0;
			int r1 = 0;
			int r2 = 0;
			double v = 0;
			int i = -1;
			while (k1 < this->nnz || k2 < nnz2) {
				if (k2 == nnz2) { // V has been processed.
					i = this->ir[k1];
					v = this->pr[k1];
					k1++;
				} else if (k1 == this->nnz) { // this has been processed.
					i = ir[k2];
					v = -pr[k2];
					k2++;
				} else { // Both this and V have not been fully processed.
					r1 = this->ir[k1];
					r2 = ir[k2];
					if (r1 < r2) {
						i = r1;
						v = this->pr[k1];
						k1++;
					} else if (r1 == r2) {
						i = r1;
						v = this->pr[k1] - pr[k2];
						k1++;
						k2++;
					} else {
						i = r2;
						v = -pr[k2];
						k2++;
					}
				}
				if (v != 0) {
					list.push_back(make_pair(i, v));
				}
			}
		}
		int nnz = list.size();
		int dim = this->getDim();
		int* ir_res = new int[nnz];
		double* pr_res = new double[nnz];
		int k = 0;
		for (std::list<pair<int, double> >::iterator iter = list.begin(); iter != list.end(); iter++) {
			ir_res[k] = iter->first;
			pr_res[k] = iter->second;
			k++;
		}
		return *new SparseVector(ir_res, pr_res, nnz, dim);
	}
}

double SparseVector::get(int i) {

	if (i < 0 || i >= dim) {
		err("Wrong index.");
		exit(1);
	}

	if (nnz == 0) {
		return 0;
	}

	int u = nnz - 1;
	int l = 0;
	int idx = -1;
	int k = 0;
	while (true) {
		if (l > u) {
			break;
		}
		k = (u + l) / 2;
		idx = ir[k];
		if (idx == i) { // Hits
			return pr[k];
		} else if (idx < i) {
			l = k + 1;
		} else {
			u = k - 1;
		}
	}

	return 0;

}

void SparseVector::set(int i, double v) {

	if (i < 0 || i >= dim) {
		err("Wrong index.");
		exit(1);
	}

	if (nnz == 0) {
		insertEntry(i, v, 0);
		return;
	}

	int u = nnz - 1;
	int l = 0;
	int idx = -1;
	int k = 0;

	int flag = 0;
	while (true) {
		if (l > u) {
			break;
		}
		k = (u + l) / 2;
		idx = ir[k];
		if (idx == i) { // Hits
			if (v == 0)
				deleteEntry(k);
			else
				pr[k] = v;
			return;
		} else if (idx < i) {
			l = k + 1;
			flag = 1;
		} else {
			u = k - 1;
			flag = 2;
		}
	}
	if (flag == 1) {
		k++;
	}
	insertEntry(i, v, k);

}

void SparseVector::insertEntry(int r, double v, int pos) {

	if (v == 0) {
		return;
	}

	int len_old = nzmax;

	int new_space = len_old < dim - 10 ? 10 : dim - len_old;

	if (nnz + 1 > len_old) {
		double* pr_new = new double[len_old + new_space];
		std::copy(pr, pr + pos, pr_new);
		// System.arraycopy(pr, 0, pr_new, 0, pos);
		pr_new[pos] = v;
		if (pos < len_old)
			// System.arraycopy(pr, pos, pr_new, pos + 1, len_old - pos);
			std::copy(pr + pos, pr + len_old, pr_new + pos + 1);
		delete[] pr;
		pr = pr_new;
	} else {
		for (int i = nnz - 1; i >= pos; i--) {
			pr[i + 1] = pr[i];
		}
		pr[pos] = v;
	}

	if (nnz + 1 > len_old) {
		int* ir_new = new int[len_old + new_space];
		// System.arraycopy(ir, 0, ir_new, 0, pos);
		std::copy(ir, ir + pos, ir_new);
		ir_new[pos] = r;
		if (pos < len_old)
			// System.arraycopy(ir, pos, ir_new, pos + 1, len_old - pos);
			std::copy(ir + pos, ir + len_old, ir_new + pos + 1);
		delete[] ir;
		ir = ir_new;
	} else {
		for (int i = nnz - 1; i >= pos; i--) {
			ir[i + 1] = ir[i];
		}
		ir[pos] = r;
	}

	nnz++;
	if (nnz + 1 > len_old) {
		nzmax = len_old + new_space;
	}

}

/**
 * Clean entries so that zero entries are removed.
 */
void SparseVector::clean() {
	for (int k = nnz - 1; k >= 0; k--) {
		if (pr[k] == 0) {
			deleteEntry(k);
		}
	}
}

void SparseVector::deleteEntry(int pos) {
	// The pos-th entry in pr must exist
	for (int i = pos; i < nnz - 1; i++) {
		pr[i] = pr[i + 1];
		ir[i] = ir[i + 1];
	}
	nnz--;
}

Vector& SparseVector::operate(Matrix& A) {
	// int M = A.getRowDimension();
	int N = A.getColumnDimension();
	if (N != dim) {
		err("Dimension doesn't match.");
		exit(1);
	}
	if (typeid(A) == typeid(DenseMatrix)) {
		double* res = new double[N];
		for (int k = 0; k < N; k++)
			res[k] = 0;
		double** AData = ((DenseMatrix&) A).getData();
		double* ARow = null;
		double v = 0;
		for (int k = 0; k < nnz; k++) {
			int i = ir[k];
			ARow = AData[i];
			v = this->pr[k];
			for (int j = 0; j < N; j++) {
				res[j] += v * ARow[j];
			}
		}
		return *new DenseVector(res, N);
	} else {
		int* ir = ((SparseMatrix&) A).getIr();
		int* jc = ((SparseMatrix&) A).getJc();
		double* pr = ((SparseMatrix&) A).getPr();
		double s = 0;
		int k1 = 0;
		int k2 = 0;
		int c = 0;
		int r = 0;
		map<int, double> map;
		for (int j = 0; j < N; j++) {
			k1 = 0;
			k2 = jc[j];
			s = 0;
			while (true) {
				if (k2 >= jc[j + 1] || k1 >= nnz) {
					break;
				}
				c = this->ir[k1];
				r = ir[k2];
				if (r < c) {
					k2++;
				} else if (r > c) {
					k1++;
				} else {
					s += this->pr[k1] * pr[k2];
					k1++;
					k2++;
				}
			}
			if (s != 0) {
				map.insert(make_pair(j, s));
			}
		}
		int nnz = map.size();
		ir = new int[nnz];
		pr = new double[nnz];
		int ind = 0;
		for (std::map<int, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
			ir[ind] = iter->first;
			pr[ind] = iter->second;
			ind++;
		}
		return *new SparseVector(ir, pr, nnz, N);
	}
}

void SparseVector::clear() {
	delete[] ir;
	delete[] pr;
	ir = new int[0];
	pr = new double[0];
	nnz = 0;
	nzmax = 0;
}

Vector& SparseVector::times(double v) {
	if (v == 0) {
		return *new SparseVector(dim);
	}
	SparseVector& res = (SparseVector&) this->copy();
	double* pr = res.pr;
	for (int k = 0; k < nnz; k++) {
		pr[k] *= v;
	}
	return res;
}
