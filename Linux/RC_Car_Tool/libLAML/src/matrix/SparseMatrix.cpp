/*
 * SparseMatrix.cpp
 *
 *  Created on: Feb 4, 2014
 *      Author: Mingjie Qian
 */

#include "SparseMatrix.h"
#include "Vector.h"
#include "DenseVector.h"
#include "SparseVector.h"
#include "Printer.h"
#include <list>
#include <map>
#include <iostream>

SparseMatrix::SparseMatrix(int M, int N) : Matrix() {
	this->M = M;
	this->N = N;
	this->nzmax = 0;
	this->nnz = 0;
	this->jc = new int[N + 1];
	for (int j = 0; j < N + 1; j++) {
		jc[j] = 0;
	}
	this->jr = new int[M + 1];
	for (int i = 0; i < M + 1; i++) {
		jr[i] = 0;
	}
	ir = new int[0];
	pr = new double[0];
	ic = new int[0];
	valCSRIndices = new int[0];
}

SparseMatrix::SparseMatrix(SparseMatrix& A) : Matrix() {
	ir = A.ir;
	jc = A.jc;
	pr = A.pr;
	ic = A.ic;
	jr = A.jr;
	valCSRIndices = A.valCSRIndices;
	M = A.M;
	N = A.N;
	nzmax = A.nzmax;
	nnz = A.nnz;
}

SparseMatrix::SparseMatrix(int* rIndices, int* cIndices, double* values, int numRows, int numColumns, int nzmax) {
	SparseMatrix& A = createSparseMatrix(rIndices, cIndices, values, numRows, numColumns, nzmax);
	ir = A.ir;
	jc = A.jc;
	pr = A.pr;
	ic = A.ic;
	jr = A.jr;
	valCSRIndices = A.valCSRIndices;
	M = A.M;
	N = A.N;
	this->nzmax = A.nzmax;
	nnz = A.nnz;
}

SparseMatrix::SparseMatrix(std::list<int>& rIndexList, std::list<int>& cIndexList, std::list<double>& valueList, int numRows, int numColumns, int nzmax) {

	// int length = nzmax;
	size_t k = -1;
	std::map<std::pair<int, int>, double> map;
	int rIdx = 0;
	int cIdx = 0;
	double v = 0;
	size_t len = rIndexList.size();
	for (k = 0; k < len; k++) {
		rIdx = rIndexList.front();
		cIdx = cIndexList.front();
		v = valueList.front();
		rIndexList.pop_front();
		cIndexList.pop_front();
		valueList.pop_front();
		if (v == 0) {
			continue;
		}
		map.insert(std::make_pair(std::make_pair(cIdx, rIdx), v));
	}

	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
		rIdx = iter->first.second;
		cIdx = iter->first.first;
		pr[k] = iter->second;
		ir[k] = rIdx;
		while (currentColumn < cIdx) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		k++;
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	SparseMatrix& A = createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

	this->ir = A.ir;
	this->jc = A.jc;
	this->pr = A.pr;
	ic = A.ic;
	jr = A.jr;
	valCSRIndices = A.valCSRIndices;
	M = A.M;
	N = A.N;
	this->nzmax = A.nzmax;
	nnz = A.nnz;

}

void SparseMatrix::assignSparseMatrix(SparseMatrix A) {
	int nnz = A.nnz;
	ir = clone(A.ir, nnz);
	jc = clone(A.jc, nnz);
	pr = clone(A.pr, nnz);
	ic = clone(A.ic, nnz);
	jr = clone(A.jr, nnz);
	valCSRIndices = clone(A.valCSRIndices, nnz);
	M = A.M;
	N = A.N;
	nzmax = A.nzmax;
	this->nnz = A.nnz;
}

double SparseMatrix::getEntry(int r, int c) {
	if (r < 0 || r > M - 1 || c < 0 || c > N - 1) {
		std::cerr << "Wrong index." << std::endl;
		exit(1);
	}
	double res = 0;
	int idx = -1;
	if (r <= c) {
		// Retrieve the entry by the c-th column
		int u = jc[c + 1] - 1;
		int l = jc[c];
		if (u < l) { // The c-th column is empty
			return 0;
		}
		int k = jc[c];
		while (true) {
			if (l > u) {
				break;
			}
			k = (u + l) / 2;
			idx = ir[k];
			if (idx == r) { // Hits
				return pr[k];
			} else if (idx < r) {
				l = k + 1;
			} else {
				u = k - 1;
			}
		}
		/*for (int k = jc[c]; k < jc[c + 1]; k++) {
					idx = ir[k];
					if (idx == r) {
						res = pr[k];
						break;
					} else if (idx > r){
						break;
					}
				}*/
	} else {
		// Retrieve the entry by the r-th row
		int u = jr[r + 1] - 1;
		int l = jr[r];
		if (u < l) { // The c-th column is empty
			return 0;
		}
		int k = jr[r];
		while (true) {
			if (l > u) {
				break;
			}
			k = (u + l) / 2;
			idx = ic[k];
			if (idx == c) { // Hits
				return pr[valCSRIndices[k]];
			} else if (idx < c) {
				l = k + 1;
			} else {
				u = k - 1;
			}
		}
	}
	return res;
}

Matrix* SparseMatrix::mtimes(Matrix* A) {
    (void)A;
	return NULL;
}

SparseMatrix::~SparseMatrix() {
	delete[] ir;
	delete[] jc;
	delete[] ic;
	delete[] jr;
	delete[] valCSRIndices;
	delete[] pr;
	// disp("This sparse matrix is released.");
}

SparseMatrix& SparseMatrix::createSparseMatrixByCSCArrays(int* ir, int* jc, double* pr, int M, int N, int nzmax) {

	int length = nzmax;
	// int nzmax = length;
	// int length = nzmax;

	SparseMatrix& res = *new SparseMatrix();
	res.ir = ir;
	res.jc = jc;
	res.pr = pr;
	res.M = M;
	res.N = N;
	res.nzmax = nzmax;

	int* ic = new int[length];
	int* jr = new int[M + 1];
	int* valCSRIndices = new int[length];

	/*
	 * Transform compressed sparse column format to
	 * compressed sparse row format.
	 */
	int* rIndices = ir;
	int* cIndices = new int[length];
	int k = 0;
	int j = 0;
	while (k < length && j < N) {
		if (jc[j] <= k && k < jc[j + 1]) {
			cIndices[k] = j;
			k++;
		} else {
			j++;
		}
	}

	std::map<std::pair<int, int>, int, PairFirstThenSecondComp<int, int> > map;

	for (k = 0; k < length; k++) {
		if (pr[k] == 0) {
			continue;
		}
		// Order by row then column
		map.insert(std::make_pair(std::make_pair(rIndices[k], cIndices[k]), k));
	}
	delete[] cIndices;

	int rIdx = -1;
	int cIdx = -1;
	int vIdx = -1;
	k = 0;
	jr[0] = 0;
	int currentRow = 0;
	for (std::map<std::pair<int, int>, int>::iterator iter = map.begin(); iter != map.end(); iter++) {
		rIdx = iter->first.first;
		cIdx = iter->first.second;
		vIdx = iter->second;
		ic[k] = cIdx;
		valCSRIndices[k] = vIdx;
		while (currentRow < rIdx) {
			jr[currentRow + 1] = k;
			currentRow++;
		}
		k++;
	}
	while (currentRow < M) {
		jr[currentRow + 1] = k;
		currentRow++;
	}
	// jr[M] = k;

	res.ic = ic;
	res.jr = jr;
	res.valCSRIndices = valCSRIndices;
	res.nnz = map.size();

	return res;

}

SparseMatrix& SparseMatrix::createSparseMatrixByCSRArrays(int* ic, int* jr, double* pr, int M, int N, int nzmax) {

	int length = nzmax;
	// int nzmax = length;

	/*
	 * Transform compressed sparse row format to
	 * compressed sparse column format.
	 */
	// int* rIndices = new int[length];
	int rIndices[length];
	int* cIndices = ic;
	int k = 0;
	int i = 0;
	while (k < length && i < M) {
		if (jr[i] <= k && k < jr[i + 1]) {
			rIndices[k] = i;
			k++;
		} else {
			i++;
		}
	}

	std::map<std::pair<int, int>, double> map;
	for (k = 0; k < length; k++) {
		if (pr[k] == 0) {
			continue;
		}
		// Order by column then row
		map.insert(std::make_pair(std::make_pair(cIndices[k], rIndices[k]), pr[k]));
	}

	int numRows = M;
	int numColumns = N;
	int* ir = new int[nzmax];
	// int ir[nzmax];
	int* jc = new int[numColumns + 1];
	// int jc[numColumns + 1];
	pr = new double[nzmax];
	// double pr2[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
		rIdx = iter->first.second;
		cIdx = iter->first.first;
		pr[k] = iter->second;
		ir[k] = rIdx;
		while (currentColumn < cIdx) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		k++;
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	return createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

}

SparseMatrix& SparseMatrix::createSparseMatrix(int* rIndices, int* cIndices, double* values, int numRows, int numColumns, int nzmax) {

	int length = nzmax;
	int k = -1;
	std::map<std::pair<int, int>, double> map;
	for (k = 0; k < length; k++) {
		if (values[k] == 0) {
			continue;
		}
		// Order by column then row
		map.insert(std::make_pair(std::make_pair(cIndices[k], rIndices[k]), values[k]));
	}

	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
		rIdx = iter->first.second;
		cIdx = iter->first.first;
		pr[k] = iter->second;
		ir[k] = rIdx;
		while (currentColumn < cIdx) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		k++;
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	return createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

}

SparseMatrix& SparseMatrix::createSparseMatrix(std::map<std::pair<int, int>, double> inputMap, int numRows, int numColumns) {

	std::map<std::pair<int, int>, double> map;
	int nzmax = 0;
	for (std::map<std::pair<int, int>, double>::iterator iter = inputMap.begin(); iter != inputMap.end(); iter++) {
		if (iter->second != 0) {
			map.insert(std::make_pair(std::make_pair(iter->first.second, iter->first.first), iter->second));
			nzmax++;
		}
	}
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
		rIdx = iter->first.second;
		cIdx = iter->first.first;
		pr[k] = iter->second;
		ir[k] = rIdx;
		while (currentColumn < cIdx) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		k++;
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	return createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

}

Matrix& SparseMatrix::mtimes(Matrix& A) {

	Matrix* res = null;
	int NA = A.getColumnDimension();

	if (typeid(A) == typeid(DenseMatrix)) {

		double** resData = new double*[M];
		for (int i = 0; i < M; i++) {
			resData[i] = new double[NA];
		}
		double* resRow = null;
		double** data = ((DenseMatrix&) A).getData();

		int c = -1;
		double s = 0;
		// double v = 0;
		for (int i = 0; i < M; i++) {
			resRow = resData[i];
			for (int j = 0; j < NA; j++) {
				s = 0;
				for (int k = jr[i]; k < jr[i + 1]; k++) {
					c = ic[k];
					s += pr[valCSRIndices[k]] * data[c][j];
				}
				resRow[j] = s;
			}
		}

		res = new DenseMatrix(resData, M, NA);

	} else if (typeid(A) == typeid(SparseMatrix)) {

		/*
		 * When this and A are all sparse matrices,
		 * the result is also a sparse matrix.
		 */
		int* ir = null;
		int* jc = null;
		double* pr = null;
		ir = ((SparseMatrix&) A).getIr();
		jc = ((SparseMatrix&) A).getJc();
		pr = ((SparseMatrix&) A).getPr();
		// rowIdx of the right sparse matrix
		int rr = -1;
		// colIdx of the left sparse matrix
		int cl = -1;
		double s = 0;
		int kl = 0;
		int kr = 0;
		int nzmax = 0;

		std::map<std::pair<int, int>, double> map;

		for (int i = 0; i < M; i++) {
			for (int j = 0; j < NA; j++) {
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
						s += this->pr[valCSRIndices[kl]] * pr[kr];
						kl++;
						kr++;
					}
				}

				// Order by column then row
				if (s != 0) {
					nzmax++;
					map.insert(std::make_pair(std::make_pair(j, i), s));
				}
			}
		}

		int numRows = this->M;
		int numColumns = NA;
		ir = new int[nzmax];
		jc = new int[numColumns + 1];
		pr = new double[nzmax];

		int rIdx = -1;
		int cIdx = -1;
		int k = 0;
		jc[0] = 0;
		int currentColumn = 0;
		for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
			rIdx = iter->first.second;
			cIdx = iter->first.first;
			pr[k] = iter->second;
			ir[k] = rIdx;
			while (currentColumn < cIdx) {
				jc[currentColumn + 1] = k;
				currentColumn++;
			}
			k++;
		}
		while (currentColumn < numColumns) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		// jc[numColumns] = k;

		res = &createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

	}

	return *res;

}

void SparseMatrix::insertEntry(int r, int c, double v, int pos) {

	if (v == 0) {
		return;
	}

	int len_old = nzmax;

	int new_space = len_old < M * N - 10 ? 10 : M * N - len_old;

	if (nnz + 1 > len_old) {
		double* pr_new = new double[len_old + new_space];
		memcpy(pr_new, pr, pos * sizeof(double));
		pr_new[pos] = v;
		if (pos < len_old)
			memcpy(pr_new + pos + 1, pr + pos, (len_old - pos) * sizeof(double));
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
		memcpy(ir_new, ir, pos * sizeof(int));
		ir_new[pos] = r;
		if (pos < len_old)
			memcpy(ir_new + pos + 1, ir + pos, (len_old - pos) * sizeof(int));
		delete[] ir;
		ir = ir_new;
	} else {
		for (int i = nnz - 1; i >= pos; i--) {
			ir[i + 1] = ir[i];
		}
		ir[pos] = r;
	}

	for (int j = c + 1; j < N + 1; j++) {
		jc[j]++;
	}

	int u = jr[r + 1] - 1; // u is the index of the last entry in r-th row
	int l = jr[r];		   // l is the index of the first entry in r-th row
	/* If jr[r] == jr[r + 1] (meaning that the r-th row is empty),
	 * jr[r] will be the insertion position
	 */
	int k = jr[r];
	int idx = -1;
	int flag = 0;
	while (true) { // Assume there exists a k such that ic[k] = c.
		if (l > u) { // If l > u in the first round, the r-th row is empty,
			break;   // and k = jr[r] is the insertion position.
		}
		k = (u + l) / 2;
		idx = ic[k]; // Until now ic[l] <= c <= ic[u] and l <= u (0)
		if (idx == c) { // Hits (Actually, it won't hit)

		} else if (idx < c) {
			l = k + 1; // Assume there exists a k such that ic[k] = c,
			flag = 1;  // then ic[l] <= c (1).
		} else {
			u = k - 1; // Assume there exists a k such that ic[k] = c,
			flag = 2;  // then ic[u] >= c (2).
		}
		// Assume there exists a k such that ic[k] = c, then l <= u.
		// If l > u, it means that there doesn't exist a k such that ic[k] = c
		// Since l <= u before updating by (1) and (2), if l > u after either
		// (1) or (2), we must have that only one of (1) or (2) breaks (0).
		// If (1) breaks (0), we have ic[l] > c and
		// l = k + 1 is the insertion position.
		// If (2) breaks (0), we have ic[u] < c and
		// u + 1 = k is the insertion position.
	}
	if (flag == 1) {
		k++;
	}

	if (nnz + 1 > len_old) {
		int* ic_new = new int[len_old + new_space];
		memcpy(ic_new, ic, k * sizeof(int));
		ic_new[k] = c;
		if (k < len_old)
			memcpy(ic_new + k + 1, ic + k, (len_old - k) * sizeof(int));
		delete[] ic;
		ic = ic_new;
	} else {
		for (int i = nnz - 1; i >= k; i--) {
			ic[i + 1] = ic[i];
		}
		ic[k] = c;
	}

	for (int i = r + 1; i < M + 1; i++) {
		jr[i]++;
	}

	for (int i = 0; i < nnz; i++) {
		if (valCSRIndices[i] >= pos)
			valCSRIndices[i]++;
	}

	if (nnz + 1 > len_old) {
		int* valCSRIndices_new = new int[len_old + new_space];
		memcpy(valCSRIndices_new, valCSRIndices, k * sizeof(int));
		valCSRIndices_new[k] = pos;
		if (k < len_old)
			memcpy(valCSRIndices_new + k + 1, valCSRIndices + k, (len_old - k) * sizeof(int));
		delete[] valCSRIndices;
		valCSRIndices = valCSRIndices_new;
	} else {
		for (int i = nnz - 1; i >= k; i--) {
			valCSRIndices[i + 1] = valCSRIndices[i];
		}
		valCSRIndices[k] = pos;
	}

	nnz++;
	if (nnz > len_old) {
		nzmax = len_old + new_space;
	}

}

void SparseMatrix::deleteEntry(int r, int c, int pos) {

	// The pos-th entry in pr must exist

	for (int i = pos; i < nnz - 1; i++) {
		pr[i] = pr[i + 1];
		ir[i] = ir[i + 1];
	}

	for (int j = c + 1; j < N + 1; j++) {
		jc[j]--;
	}

	int u = jr[r + 1] - 1;
	int l = jr[r];
	/* If jr[r] == jr[r + 1] (meaning that the r-th row is empty),
	 * jr[r] will be the insertion position
	 */
	int k = jr[r];
	int idx = -1;
	// int flag = 0; // flag = 1: ic[k] < c; flag = 2: ic[k] > c
	while (true) {
		if (l > u) {
			break;
		}
		k = (u + l) / 2;
		idx = ic[k];
		if (idx == c) { // Hits
			// flag = 0;
			break;
		} else if (idx < c) {
			l = k + 1;
			// flag = 1;
		} else {
			u = k - 1;
			// flag = 2;
		}
	}
	/*if (flag == 1) {
				k++;
			}*/

	for (int i = 0; i < nnz; i++) {
		if (valCSRIndices[i] > pos)
			valCSRIndices[i]--;
	}

	for (int j = k; j < nnz - 1; j++) {
		ic[j] = ic[j + 1];
		valCSRIndices[j] = valCSRIndices[j + 1];
	}

	for (int i = r + 1; i < M + 1; i++) {
		jr[i]--;
	}

	nnz--;
}

void SparseMatrix::setEntry(int r, int c, double v) {

	if (r < 0 || r > M - 1 || c < 0 || c > N - 1) {
		std::cerr <<"Wrong index." << std::endl;
		exit(1);
	}
	int u = jc[c + 1] - 1;
	int l = jc[c];
	if (u < l) { // The c-th column is empty
		insertEntry(r, c, v, jc[c]);
		return;
	}
	int idx = -1;
	/* If jc[c] == jc[c + 1] (meaning that the c-th column is empty),
	 * jc[c] will be the insertion position
	 */
	int k = jc[c];
	int flag = 0;
	while (true) {
		if (l > u) {
			break;
		}
		k = (u + l) / 2;
		idx = ir[k];
		if (idx == r) { // Hits
			if (v == 0)
				deleteEntry(r, c, k);
			else
				pr[k] = v;
			return;
		} else if (idx < r) {
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
	insertEntry(r, c, v, k);

}

Matrix& SparseMatrix::transpose() {
	SparseMatrix& res = *new SparseMatrix();
	res.M = N;
	res.N = M;
	res.nnz = nnz;
	res.nzmax = nzmax;
	res.ir = clone(ic, nnz);
	res.jc = clone(jr, M + 1);
	res.ic = clone(ir, nnz);
	res.jr = clone(jc, N + 1);
	double* pr_new = new double[nzmax];
	int k = 0;
	for (k = 0; k < nnz; k++) {
		pr_new[k] = pr[valCSRIndices[k]];
	}
	res.pr = pr_new;

	int* valCSRIndices_new = new int[nnz];
	int j = 0;
	int rIdx = -1;
	int cIdx = -1;
	k = 0;
	int k2 = 0;
	int numBeforeThisEntry = 0;
	while (k < nnz && j < N) {
		if (jc[j] <= k && k < jc[j + 1]) {
			rIdx = ir[k];
			cIdx = j;
			numBeforeThisEntry = jr[rIdx];
			for (k2 = jr[rIdx]; k2 < jr[rIdx + 1]; k2++) {
				if (ic[k2] == cIdx) {
					break;
				} else {
					numBeforeThisEntry++;
				}
			}
			valCSRIndices_new[k] = numBeforeThisEntry;
			k++;
		} else {
			j++;
		}
	}
	res.valCSRIndices = valCSRIndices_new;

	return res;
}

Matrix& SparseMatrix::plus(Matrix& A) {

	if (A.getRowDimension() != M || A.getColumnDimension() != N) {
		err("Dimension doesn't match.");
		exit(1);
	}

	if (typeid(A) == typeid(DenseMatrix)) {
		Matrix& res = A.plus(*this);
		return res;
	} else {

		/*
		 * When this and A are all sparse matrices,
		 * the result is also a sparse matrix.
		 */
		int* ir = null;
		int* jc = null;
		double* pr = null;
		ir = ((SparseMatrix&) A).getIr();
		jc = ((SparseMatrix&) A).getJc();
		pr = ((SparseMatrix&) A).getPr();

		int k1 = 0;
		int k2 = 0;
		int r1 = -1;
		int r2 = -1;
		int nzmax = 0;
		int i = -1;
		double v = 0;
		std::map<std::pair<int, int>, double> map;

		for (int j = 0; j < N; j++) {
			k1 = this->jc[j];
			k2 = jc[j];

			// Both this and A's j-th columns are empty.
			if (k1 == this->jc[j + 1] && k2 == jc[j + 1])
				continue;

			while (k1 < this->jc[j + 1] || k2 < jc[j + 1]) {

				if (k2 == jc[j + 1]) { // A's j-th column has been processed.
					i = this->ir[k1];
					v = this->pr[k1];
					k1++;
				} else if (k1 == this->jc[j + 1]) { // this j-th column has been processed.
					i = ir[k2];
					v = pr[k2];
					k2++;
				} else { // Both this and A's j-th columns have not been fully processed.
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
					map.insert(std::make_pair(std::make_pair(j, i), v));
					nzmax++;
				}

			}

		}

		int numRows = M;
		int numColumns = N;
		ir = new int[nzmax];
		jc = new int[numColumns + 1];
		pr = new double[nzmax];

		int rIdx = -1;
		int cIdx = -1;
		int k = 0;
		jc[0] = 0;
		int currentColumn = 0;
		for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
			rIdx = iter->first.second;
			cIdx = iter->first.first;
			pr[k] = iter->second;
			ir[k] = rIdx;
			while (currentColumn < cIdx) {
				jc[currentColumn + 1] = k;
				currentColumn++;
			}
			k++;
		}
		while (currentColumn < numColumns) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		// jc[numColumns] = k;

		Matrix& res = SparseMatrix::createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);
		return res;
	}

}

Matrix& SparseMatrix::minus(Matrix& A) {

	if (A.getRowDimension() != M || A.getColumnDimension() != N) {
		err("Dimension doesn't match.");
		exit(1);
	}

	if (typeid(A) == typeid(DenseMatrix)) {

		Matrix& res = A.copy();
		double** resData = ((DenseMatrix&) res).getData();
		int r = -1;
		int k = 0;
		for (int j = 0; j < N; j++) {
			for (int i = 0; i < M; i++) {
				resData[i][j] = -resData[i][j];
			}
			for (k = jc[j]; k < jc[j + 1]; k++) {
				r = ir[k];
				// A[r][j] = pr[k]
				resData[r][j] += pr[k];
			}
		}
		return res;
	} else {

		/*
		 * When this and A are all sparse matrices,
		 * the result is also a sparse matrix.
		 */
		int* ir = null;
		int* jc = null;
		double* pr = null;
		ir = ((SparseMatrix&) A).getIr();
		jc = ((SparseMatrix&) A).getJc();
		pr = ((SparseMatrix&) A).getPr();

		int k1 = 0;
		int k2 = 0;
		int r1 = -1;
		int r2 = -1;
		int nzmax = 0;
		int i = -1;
		double v = 0;
		std::map<std::pair<int, int>, double> map;

		for (int j = 0; j < N; j++) {
			k1 = this->jc[j];
			k2 = jc[j];

			// Both this and A's j-th columns are empty.
			if (k1 == this->jc[j + 1] && k2 == jc[j + 1])
				continue;

			while (k1 < this->jc[j + 1] || k2 < jc[j + 1]) {

				if (k2 == jc[j + 1]) { // A's j-th column has been processed.
					i = this->ir[k1];
					v = this->pr[k1];
					k1++;
				} else if (k1 == this->jc[j + 1]) { // this j-th column has been processed.
					i = ir[k2];
					v = -pr[k2];
					k2++;
				} else { // Both this and A's j-th columns have not been fully processed.
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
					map.insert(std::make_pair(std::make_pair(j, i), v));
					nzmax++;
				}

			}

		}

		int numRows = M;
		int numColumns = N;
		ir = new int[nzmax];
		jc = new int[numColumns + 1];
		pr = new double[nzmax];

		int rIdx = -1;
		int cIdx = -1;
		int k = 0;
		jc[0] = 0;
		int currentColumn = 0;
		for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
			rIdx = iter->first.second;
			cIdx = iter->first.first;
			pr[k] = iter->second;
			ir[k] = rIdx;
			while (currentColumn < cIdx) {
				jc[currentColumn + 1] = k;
				currentColumn++;
			}
			k++;
		}
		while (currentColumn < numColumns) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		// jc[numColumns] = k;

		Matrix& res = createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);
		return res;
	}
}

Matrix& SparseMatrix::times(Matrix& A) {

	if (A.getRowDimension() != M || A.getColumnDimension() != N) {
		err("Dimension doesn't match.");
		exit(1);
	}

	if (typeid(A) == typeid(DenseMatrix)) {
		Matrix& res = A.times(*this);
		return res;
	} else {

		/*
		 * When this and A are all sparse matrices,
		 * the result is also a sparse matrix.
		 */
		int* ir = null;
		int* jc = null;
		double* pr = null;
		ir = ((SparseMatrix&) A).getIr();
		jc = ((SparseMatrix&) A).getJc();
		pr = ((SparseMatrix&) A).getPr();

		int k1 = 0;
		int k2 = 0;
		int r1 = -1;
		int r2 = -1;
		int nzmax = 0;
		int i = -1;
		double v = 0;
		std::map<std::pair<int, int>, double> map;

		for (int j = 0; j < N; j++) {
			k1 = this->jc[j];
			k2 = jc[j];

			// If the j-th column of A or this is empty, we don't need to compute.
			if (k1 == this->jc[j + 1] || k2 == jc[j + 1])
				continue;

			while (k1 < this->jc[j + 1] && k2 < jc[j + 1]) {

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
						map.insert(std::make_pair(std::make_pair(j, i), v));
						nzmax++;
					}
				} else {
					k2++;
				}

			}

		}

		int numRows = M;
		int numColumns = N;
		ir = new int[nzmax];
		jc = new int[numColumns + 1];
		pr = new double[nzmax];

		int rIdx = -1;
		int cIdx = -1;
		int k = 0;
		jc[0] = 0;
		int currentColumn = 0;
		for (std::map<std::pair<int, int>, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
			rIdx = iter->first.second;
			cIdx = iter->first.first;
			pr[k] = iter->second;
			ir[k] = rIdx;
			while (currentColumn < cIdx) {
				jc[currentColumn + 1] = k;
				currentColumn++;
			}
			k++;
		}
		while (currentColumn < numColumns) {
			jc[currentColumn + 1] = k;
			currentColumn++;
		}
		// jc[numColumns] = k;

		Matrix& res = createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);
		return res;
	}
}

Matrix& SparseMatrix::times(double v) {
	/*int a = 1;
	a = a + 1;*/

	// disp(*this);
	if (v == 0) {
		Matrix& res = *new SparseMatrix(M, N);
		return res;
	} else {
		SparseMatrix& res = (SparseMatrix&) this->copy();
		/*disp(res.ir, res.nnz);
		disp(res.pr, res.nnz);
		disp(res.valCSRIndices, res.nnz);
		disp(res);*/
		for (int k = 0; k < nnz; k++) {
			((SparseMatrix&) res).pr[k] *= v;
		}
		return res;
	}
}

Matrix& SparseMatrix::copy() {
	SparseMatrix& res = *new SparseMatrix();
	res.ir = clone(ir, nnz);
	/*disp("ir:");
	disp(ir, nnz);
	disp(res.ir, nnz);*/
	res.jc = clone(jc, N + 1);
	/*disp("jc:");
	disp(jc, nnz);
	disp(res.jc, nnz);*/
	res.pr = clone(pr, nnz);
	/*disp("pr:");
	disp(pr, nnz);
	disp(res.pr, nnz);*/
	res.ic = clone(ic, nnz);
	res.jr = clone(jr, M + 1);
	res.valCSRIndices = clone(valCSRIndices, nnz);
	res.M = M;
	res.N = N;
	res.nzmax = nnz;
	res.nnz = nnz;
	// disp(res);
	return res;
}

Matrix& SparseMatrix::plus(double v) {
	Matrix& res = *new DenseMatrix(M, N, v);
	double** data = ((DenseMatrix&) res).getData();
	for (int j = 0; j < N; j++) {
		for (int k = jc[j]; k < jc[j + 1]; k++) {
			data[ir[k]][j] += pr[k];
		}
	}
	return res;
}

Matrix& SparseMatrix::minus(double v) {
	return this->plus(-v);
}

void SparseMatrix::clear() {
	this->nzmax = 0;
	this->nnz = 0;
	if (this->jc == NULL)
		this->jc = new int[N + 1];
	for (int j = 0; j < N + 1; j++) {
		jc[j] = 0;
	}
	if (this->jr == NULL)
		this->jr = new int[M + 1];
	for (int i = 0; i < M + 1; i++) {
		jr[i] = 0;
	}
	delete[] ir;
	delete[] ic;
	delete[] pr;
	delete[] valCSRIndices;
	ir = new int[0];
	pr = new double[0];
	ic = new int[0];
	valCSRIndices = new int[0];
}

/**
 * Clean entries so that zero entries are removed.
 */
void SparseMatrix::clean() {

	std::list<std::pair<int, int> > list;

	for (int k = 0; k < nnz; k++) {
		if (pr[k] == 0) {
			list.push_back(std::make_pair(ir[k], ic[k]));
		}
	}

	for(std::list<std::pair<int, int> >::iterator iter = list.begin(); iter != list.end(); iter++) {
		setEntry(iter->first, iter->second, 0);
	}

}

Vector& SparseMatrix::operate(Vector& b) {

	if (N != b.getDim()) {
		err("Dimension does not match.");
		exit(1);
	}

	if (typeid(b) == typeid(DenseVector)) {
		double* V = new double[M];
		double* pr = ((DenseVector&) b).getPr();
		double s = 0;
		int c = 0;
		for (int r = 0; r < M; r++) {
			s = 0;
			for (int k = this->jr[r]; k < this->jr[r + 1]; k++) {
				c = ic[k];
				s += this->pr[valCSRIndices[k]] * pr[c];
			}
			V[r] = s;
		}
		Vector& res = *new DenseVector(V, M);
		return res;
	} else {
		int* ir = ((SparseVector&) b).getIr();
		double* pr = ((SparseVector&) b).getPr();
		int nnz = ((SparseVector&) b).getNNZ();
		double s = 0;
		int kl = 0;
		int kr = 0;
		int cl = 0;
		int rr = 0;
		std::map<int, double> map;
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
					s += this->pr[valCSRIndices[kl]] * pr[kr];
					kl++;
					kr++;
				}
			}

			if (s != 0) {
				map.insert(std::make_pair(i, s));
			}
		}
		nnz = map.size();
		ir = new int[nnz];
		pr = new double[nnz];
		int ind = 0;
		for (std::map<int, double>::iterator iter = map.begin(); iter != map.end(); iter++) {
			ir[ind] = iter->first;
			pr[ind] = iter->second;
			ind++;
		}
		Vector& res = *new SparseVector(ir, pr, nnz, M);
		return res;
	}
}

Matrix& SparseMatrix::getSubMatrix(int startRow, int endRow, int startColumn, int endColumn) {
	int nnz = 0;
	int numRows = endRow - startRow + 1;
	int numColumns = endColumn - startColumn + 1;
	// Calculate nnz
	int rowIdx = -1;
	for (int j = startColumn; j <= endColumn; j++) {
		for (int k = this->jc[j]; k < this->jc[j + 1]; k++) {
			rowIdx = this->ir[k];
			if (rowIdx < startRow)
				continue;
			else if (rowIdx > endRow)
				break;
			else
				nnz++;
		}
	}

	int nzmax = nnz;
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	int currentColumn = startColumn;
	for (int j = startColumn; j <= endColumn; j++) {
		for (int t = this->jc[j]; t < this->jc[j + 1]; t++) {
			rowIdx = this->ir[t];
			if (rowIdx < startRow)
				continue;
			else if (rowIdx > endRow)
				break;
			else {
				rIdx = rowIdx - startRow;
				cIdx = j;
				pr[k] = this->pr[t];
				ir[k] = rIdx;
				while (currentColumn < cIdx) {
					jc[currentColumn + 1 - startColumn] = k;
					currentColumn++;
				}
				k++;
			}
		}
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1 - startColumn] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	return SparseMatrix::createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

}

Matrix& SparseMatrix::getSubMatrix(int* selectedRows, int numSelRows, int* selectedColumns, int numSelColumns) {
	int nRow = numSelRows;
	int nCol = numSelColumns;
	double v = 0;
	int r, c;
	SparseMatrix& res = *new SparseMatrix(nRow, nCol);
	for (int j = 0; j < nCol; j++) {
		c = selectedColumns[j];
		for (int i = 0; i < nRow; i++) {
			r = selectedRows[i];
			v = this->getEntry(r, c);
			if (v != 0)
				res.setEntry(i, j, v);
		}
	}
	return res;
}

/**
 * Get rows from startRow to endRow as a matrix.
 *
 * @param startRow start index
 *
 * @param endRow end index (inclusive)
 *
 * @return a real matrix containing the specified rows
 */
Matrix& SparseMatrix::getRows(int startRow, int endRow) {
	int nnz = 0;
	int numRows = endRow - startRow + 1;
	int numColumns = N;
	// Calculate nnz
	int rowIdx = -1;
	for (int j = 0; j < numColumns; j++) {
		for (int k = this->jc[j]; k < this->jc[j + 1]; k++) {
			rowIdx = this->ir[k];
			if (rowIdx < startRow)
				continue;
			else if (rowIdx > endRow)
				break;
			else
				nnz++;
		}
	}

	int nzmax = nnz;
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	int currentColumn = 0;
	for (int j = 0; j < numColumns; j++) {
		for (int t = this->jc[j]; t < this->jc[j + 1]; t++) {
			rowIdx = this->ir[t];
			if (rowIdx < startRow)
				continue;
			else if (rowIdx > endRow)
				break;
			else {
				rIdx = rowIdx - startRow;
				cIdx = j;
				pr[k] = this->pr[t];
				ir[k] = rIdx;
				while (currentColumn < cIdx) {
					jc[currentColumn + 1] = k;
					currentColumn++;
				}
				k++;
			}
		}
	}
	while (currentColumn < numColumns) {
		jc[currentColumn + 1] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	return createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

}

/**
 * Get rows of specified row indices as a matrix.
 *
 * @param selectedRows a sequence of row indices
 *
 * @param numSelRows number of selected rows
 *
 * @return a real matrix containing the specified rows
 */
Matrix& SparseMatrix::getRows(int* selectedRows, int numSelRows) {
	int nRow = numSelRows;
	int nCol = N;
	double v = 0;
	int r, c;
	SparseMatrix& res = *new SparseMatrix(nRow, nCol);
	for (int j = 0; j < nCol; j++) {
		c = j;
		for (int i = 0; i < nRow; i++) {
			r = selectedRows[i];
			v = this->getEntry(r, c);
			if (v != 0)
				res.setEntry(i, j, v);
		}
	}
	return res;
}

/**
 * Get rows from startRow to endRow as a 1D {@code Vector} array.
 *
 * @param startRow start index
 *
 * @param endRow end index (inclusive)
 *
 * @return a 1D {@code Vector} array containing the specified rows
 */
Vector** SparseMatrix::getRowVectors(int startRow, int endRow) {
	int numRows = endRow - startRow + 1;
	Vector** res = new Vector*[numRows];
	for (int r = startRow, i = 0; r <= endRow; r++, i++) {
		res[i] = &getRowVector(r);
	}
	return res;
}

/**
 * Get rows of specified row indices as a 1D {@code Vector} array.
 *
 * @param selectedRows a sequence of row indices
 *
 * @param numSelRows number of selected rows
 *
 * @return a 1D {@code Vector} array containing the specified rows
 */
Vector** SparseMatrix::getRowVectors(int* selectedRows, int numSelRows) {
	int numRows = numSelRows;
	Vector** res = new Vector*[numRows];
	for (int i = 0; i < numRows; i++) {
		res[i] = &getRowVector(selectedRows[i]);
	}
	return res;
}

/**
 * Get a row matrix.
 *
 * @param r row index
 *
 * @return the r-th row matrix
 */
Matrix& SparseMatrix::getRowMatrix(int r) {
	int nnz = this->jr[r + 1] - this->jr[r];
	if (nnz == 0)
		return *new SparseMatrix(1, N);
	int* ic = new int[nnz];
	int jr[] = {0, nnz};
	double* pr = new double[nnz];
	for (int k = this->jr[r], j = 0; k < this->jr[r + 1]; k++, j++) {
		ic[j] = this->ic[k];
		pr[j] = this->pr[valCSRIndices[k]];
	}
	return createSparseMatrixByCSRArrays(ic, jr, pr, 1, N, nnz);
}

/**
 * Set the r-th row by a row matrix A.
 *
 * @param r row index
 *
 * @param A a row matrix
 */
void SparseMatrix::setRowMatrix(int r, Matrix& A) {
	if (A.getRowDimension() != 1) {
		err("Input matrix should be a row matrix.");
		exit(1);
	}
	for (int j = 0; j < N; j++) {
		setEntry(r, j, A.getEntry(0, j));
	}
}

/**
 * Get a row vector.
 *
 * @param r row index
 *
 * @return the r-th row vector
 */
Vector& SparseMatrix::getRowVector(int r) {
	int dim = N;
	int nnz = jr[r + 1] - jr[r];
	if (nnz == 0)
		return *new SparseVector(dim);
	int* ir = new int[nnz];
	double* pr = new double[nnz];
	for (int k = jr[r], j = 0; k < jr[r + 1]; k++, j++) {
		ir[j] = this->ic[k];
		pr[j] = this->pr[valCSRIndices[k]];
	}
	return *new SparseVector(ir, pr, nnz, dim);
}

/**
 * Set the r-th row by a row vector V.
 *
 * @param r row index
 *
 * @param V a row vector
 */
void SparseMatrix::setRowVector(int r, Vector& V) {
	for (int j = 0; j < N; j++) {
		setEntry(r, j, V.get(j));
	}
}

/**
 * Get columns from startColumn to endColumn as a matrix.
 *
 * @param startColumn start index
 *
 * @param endColumn end index (inclusive)
 *
 * @return a real matrix containing the specified columns
 */
Matrix& SparseMatrix::getColumns(int startColumn, int endColumn) {
	int nnz = 0;
	int numRows = M;
	int numColumns = endColumn - startColumn + 1;
	// Calculate nnz
	int rowIdx = -1;
	for (int j = startColumn; j <= endColumn; j++) {
		nnz += this->jc[j + 1] - this->jc[j];
	}

	int nzmax = nnz;
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	int currentColumn = startColumn;
	for (int j = startColumn; j <= endColumn; j++) {
		for (int t = this->jc[j]; t < this->jc[j + 1]; t++) {
			rowIdx = this->ir[t];
			rIdx = rowIdx;
			cIdx = j;
			pr[k] = this->pr[t];
			ir[k] = rIdx;
			while (currentColumn < cIdx) {
				jc[currentColumn + 1 - startColumn] = k;
				currentColumn++;
			}
			k++;
		}
	}
	while (currentColumn <= endColumn) {
		jc[currentColumn + 1 - startColumn] = k;
		currentColumn++;
	}
	// jc[numColumns] = k;

	return createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

}

/**
 * Get columns of specified column indices as a matrix.
 *
 * @param selectedColumns a sequence of column indices
 *
 * @param numSelColumns number of selected columns
 *
 * @return a real matrix containing the specified columns
 */
Matrix& SparseMatrix::getColumns(int* selectedColumns, int numSelColumns) {
	int nnz = 0;
	int numRows = M;
	int numColumns = numSelColumns;
	// Calculate nnz
	int rowIdx = -1;
	int j = -1;
	for (int c = 0; c < numColumns; c++) {
		j = selectedColumns[c];
		nnz += this->jc[j + 1] - this->jc[j];
	}

	int nzmax = nnz;
	int* ir = new int[nzmax];
	int* jc = new int[numColumns + 1];
	double* pr = new double[nzmax];

	int rIdx = -1;
	// int cIdx = -1;
	int k = 0;
	jc[0] = 0;
	for (int c = 0; c < numColumns; c++) {
		j = selectedColumns[c];
		jc[c + 1] = jc[c] + this->jc[j + 1] - this->jc[j];
		for (int t = this->jc[j]; t < this->jc[j + 1]; t++) {
			rowIdx = this->ir[t];
			rIdx = rowIdx;
			// cIdx = c;
			pr[k] = this->pr[t];
			ir[k] = rIdx;
			k++;
		}
	}
	// jc[numColumns] = k;

	return createSparseMatrixByCSCArrays(ir, jc, pr, numRows, numColumns, nzmax);

}

/**
 * Get columns from startColumn to endColumn as a 1D {@code Vector} array.
 *
 * @param startColumn start index
 *
 * @param endColumn end index (inclusive)
 *
 * @return a 1D {@code Vector} array containing the specified columns
 */
Vector** SparseMatrix::getColumnVectors(int startColumn, int endColumn) {
	int numColumns = endColumn - startColumn + 1;
	Vector** res = new Vector*[numColumns];
	for (int c = startColumn, i = 0; c <= endColumn; c++, i++) {
		res[i] = &getColumnVector(c);
	}
	return res;
}

/**
 * Get columns of specified column indices as a 1D {@code Vector} array.
 *
 * @param selectedColumns a sequence of column indices
 *
 * @param numSelColumns number of selected columns
 *
 * @return a 1D {@code Vector} array containing the specified columns
 */
Vector** SparseMatrix::getColumnVectors(int* selectedColumns, int numSelColumns) {
	int numColumns = numSelColumns;
	Vector** res = new Vector*[numColumns];
	for (int j = 0; j < numColumns; j++) {
		res[j] = &getColumnVector(selectedColumns[j]);
	}
	return res;
}

/**
 * Get a column matrix.
 *
 * @param c column index
 *
 * @return the c-th column matrix
 */
Matrix& SparseMatrix::getColumnMatrix(int c) {
	int nnz = this->jc[c + 1] - this->jc[c];
	if (nnz == 0)
		return *new SparseMatrix(M, 1);
	int* ir = new int[nnz];
	int jc[] = {0, nnz};
	double* pr = new double[nnz];
	for (int k = this->jc[c], i = 0; k < this->jc[c + 1]; k++, i++) {
		ir[i] = this->ir[k];
		pr[i] = this->pr[k];
	}
	return createSparseMatrixByCSCArrays(ir, jc, pr, M, 1, nnz);
}

/**
 * Set the c-th column by a column matrix A.
 *
 * @param c column index
 *
 * @param A a column matrix
 */
void SparseMatrix::setColumnMatrix(int c, Matrix& A) {
	if (A.getColumnDimension() != 1) {
		err("Input matrix should be a column matrix.");
		exit(1);
	}
	for (int i = 0; i < M; i++) {
		setEntry(i, c, A.getEntry(i, 0));
	}
}

/**
 * Get a column vector.
 *
 * @param c column index
 *
 * @return the c-th column vector
 */
Vector& SparseMatrix::getColumnVector(int c) {
	int dim = M;
	int nnz = jc[c + 1] - jc[c];
	if (nnz == 0)
		return *new SparseVector(dim);
	int* ir = new int[nnz];
	double* pr = new double[nnz];
	for (int k = jc[c], i = 0; k < jc[c + 1]; k++, i++) {
		ir[i] = this->ir[k];
		pr[i] = this->pr[k];
	}
	return *new SparseVector(ir, pr, nnz, dim);
}

/**
 * Set the c-th column by a column vector V.
 *
 * @param c column index
 *
 * @param V a column vector
 */
void SparseMatrix::setColumnVector(int c, Vector& V) {
	for (int i = 0; i < M; i++) {
		setEntry(i, c, V.get(i));
	}
}
