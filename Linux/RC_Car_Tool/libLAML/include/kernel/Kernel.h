/*
 * Kernel.h
 *
 *  Created on: Feb 28, 2014
 *      Author: Mingjie Qian
 */

#ifndef KERNEL_H_
#define KERNEL_H_

#include "Matrix.h"
#include <string>

/***
 * C++ implementation of commonly used kernel functions.
 *
 * @version 1.0 Feb. 28th, 2014
 * @author Mingjie Qian
 */

/**
 * Computes Gram matrix of a specified kernel. Given a data matrix
 * X (n x d), it returns Gram matrix K (n x n).
 *
 * @param kernelType 'linear' | 'poly' | 'rbf' | 'cosine'
 *
 * @param kernelParam   --    | degree | sigma |    --
 *
 * @param X a matrix with each row being a feature vector

 * @return Gram matrix (n x n)
 *
 */
Matrix& calcKernel(std::string kernelType,
		double kernelParam, Matrix& X);

/**
 * Computes Gram matrix of a specified kernel. Given two sets of vectors
 * A (n1 vectors) and B (n2 vectors), it returns Gram matrix K (n1 x n2).
 *
 * @param kernelType 'linear' | 'poly' | 'rbf' | 'cosine'
 *
 * @param kernelParam   --    | degree | sigma |    --
 *
 * @param A a 1D {@code Vector} array
 *
 * @param B a 1D {@code Vector} array
 *
 * @return Gram matrix (n1 x n2)
 */
Matrix& calcKernel(std::string kernelType,
		double kernelParam, Vector** A, int nA, Vector** B, int nB);

/**
 * Computes Gram matrix of a specified kernel. Given two data matrices
 * X1 (n1 x d), X2 (n2 x d), it returns Gram matrix K (n1 x n2).
 *
 * @param kernelType 'linear' | 'poly' | 'rbf' | 'cosine'
 *
 * @param kernelParam   --    | degree | sigma |    --
 *
 * @param X1 a matrix with each row being a feature vector
 *
 * @param X2 a matrix with each row being a feature vector
 *
 * @return Gram matrix (n1 x n2)
 *
 */
Matrix& calcKernel(std::string kernelType,
		double kernelParam, Matrix& X1, Matrix& X2);

#endif /* KERNEL_H_ */
