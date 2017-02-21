/*
 * DenseVector.h
 *
 *  Created on: Feb 9, 2014
 *      Author: Mingjie Qian
 */

#ifndef DENSEVECTOR_H_
#define DENSEVECTOR_H_

#include "Vector.h"
#include "Matrix.h"

class DenseVector : public Vector {

private:

	double* pr;

	int dim;

public:

	DenseVector();

	DenseVector(int dim);

	DenseVector(int dim, double v);

	DenseVector(double* pr, int dim);

	// DenseVector(double pr[], int dim);

	~DenseVector();

	double* getPr();

	/**
	 * Get the dimensionality of this vector.
	 *
	 * @return dimensionality of this vector
	 */
	int getDim() {return dim;};

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

};


#endif /* DENSEVECTOR_H_ */
