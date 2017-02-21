/*
 * Vector.h
 *
 *  Created on: Feb 9, 2014
 *      Author: Mingjie Qian
 */

#ifndef VECTOR_H_
#define VECTOR_H_

class Matrix;

class Vector {

public:

	Vector() {};

public:

	virtual ~Vector() {};

	/*
	 * Note that if there is a virtual function member that doesn't
	 * have a definition, the compiler failed to create the vtable
	 * for this class which will lead to undefined reference to
	 * `vtable for Vector' linking error.
	 */

	/**
	 * Get the dimensionality of this vector.
	 *
	 * @return dimensionality of this vector
	 */
	virtual int getDim() = 0;

	/**
	 * Get a deep copy of this vector.
	 *
	 * @return a copy of this vector
	 */
	virtual Vector& copy() = 0;

	/**
	 * Element-wise multiplication, i.e. res = this .* V.
	 *
	 * @param V a real vector
	 *
	 * @return this .* V
	 */
	virtual Vector& times(Vector& V) = 0;

	/**
	 * res = v * this.
	 *
	 * @param v a real scalar
	 *
	 * @return v * this
	 */
	virtual Vector& times(double v) = 0;

	/**
	 * Vector addition, i.e. res = this + V.
	 *
	 * @param V a real vector
	 *
	 * @return this + V
	 */
	virtual Vector& plus(Vector& V) = 0;

	/**
	 * Vector subtraction, i.e. res = this - V.
	 *
	 * @param V a real vector
	 *
	 * @return this - V
	 */
	virtual Vector& minus(Vector& V) = 0;

	/**
	 * Get the value of the i-th entry.
	 *
	 * @param i index
	 *
	 * @return this(i)
	 */
	virtual double get(int i) = 0;

	/**
	 * Set the value of the i-th entry.
	 *
	 * @param i index
	 *
	 * @param v value to set
	 */
	virtual void set(int i, double v) = 0;

	/**
	 * Clear this vector.
	 */
	virtual void clear() = 0;

	/**
	 * Vector matrix multiplication, i.e. res = this' * A.
	 *
	 * @param A a real matrix
	 *
	 * @return this<sup>T</sup> * A
	 */
	virtual Vector& operate(Matrix& A) = 0;

};

#endif /* VECTOR_H_ */
