/*
 * SparseVector.h
 *
 *  Created on: Feb 9, 2014
 *      Author: Mingjie Qian
 */

#ifndef SPARSEVECTOR_H_
#define SPARSEVECTOR_H_

#include "Vector.h"

class SparseVector : public Vector {

private:

	int* ir;

	double* pr;

	int nnz;

	int dim;

	int nzmax;

public:

	SparseVector();

	SparseVector(SparseVector& V);

	SparseVector(int dim);

	SparseVector(int* ir, double* pr, int nnz, int dim);

	~SparseVector();

	/**
	 * this = V such that they refer to the same sparse vector.
	 *
	 * @param V a sparse vector
	 */
	void setSparseVector(SparseVector& V);

	/**
	 * Assign this sparse vector by a sparse vector V in the sense that
	 * all interior arrays of this vector are deep copy of the given
	 * sparse vector V.
	 *
	 * @param V a sparse Vector
	 */
	void assignSparseVector(SparseVector& V);

	int* getIr() {
		return ir;
	}

	double* getPr() {
		return pr;
	}

	int getNNZ() {
		return nnz;
	}

	/**
	 * Get the dimensionality of this vector.
	 *
	 * @return dimensionality of this vector
	 */
	int getDim() {return dim;};

	/**
	 * Change dimensionality of this sparse vector.
	 *
	 * @param dim new dimensionality
	 */
	void setDim(int dim);

	/**
	 * Get a deep copy of this vector.
	 *
	 * @return a copy of this vector
	 */
	Vector& copy();

	/**
	 * Element-wise multiplication, i.e. res = this .* V.
	 *
	 * @param V a real vector
	 *
	 * @return this .* V
	 */
	Vector& times(Vector& V);

	/**
	 * res = v * this.
	 *
	 * @param v a real scalar
	 *
	 * @return v * this
	 */
	Vector& times(double v);

	/**
	 * Vector addition, i.e. res = this + V.
	 *
	 * @param V a real vector
	 *
	 * @return this + V
	 */
	Vector& plus(Vector& V);

	/**
	 * Vector subtraction, i.e. res = this - V.
	 *
	 * @param V a real vector
	 *
	 * @return this - V
	 */
	Vector& minus(Vector& V);

	/**
	 * Get the value of the i-th entry.
	 *
	 * @param i index
	 *
	 * @return this(i)
	 */
	double get(int i);

	/**
	 * Set the value of the i-th entry.
	 *
	 * @param i index
	 *
	 * @param v value to set
	 */
	void set(int i, double v);

	/**
	 * Vector matrix multiplication, i.e. res = this' * A.
	 *
	 * @param A a real matrix
	 *
	 * @return this<sup>T</sup> * A
	 */
	Vector& operate(Matrix& A);

	/**
	 * Clear this vector.
	 */
	void clear();

	/**
	 * Clean entries so that zero entries are removed.
	 */
	void clean();

private:

	void insertEntry(int r, double v, int pos);

	void deleteEntry(int pos);

};


#endif /* SPARSEVECTOR_H_ */
