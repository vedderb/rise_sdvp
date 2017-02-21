/*
 * Manifold.h
 *
 *  Created on: Feb 28, 2014
 *      Author: Mingjie Qian
 */

#ifndef MANIFOLD_H_
#define MANIFOLD_H_

#include "Matrix.h"
#include "Options.h"
#include <string>

/**
 * Calculate the graph Laplacian of the adjacency graph of a data set
 * represented as rows of a matrix X.
 *
 * @param X data matrix with each row being a sample
 *
 * @param type graph type, either "nn" or "epsballs"
 *
 * @param options
 *        data structure containing the following fields
 *        NN - integer if type is "nn" (number of nearest neighbors),
 *             or size of "epsballs"
 *        DISTANCEFUNCTION - distance function used to make the graph
 *        WEIGHTTYPPE = "binary" | "distance" | "heat" | "inner"
 * 	      WEIGHTPARAM = width for heat kernel
 * 	      NORMALIZE = 0 | 1 whether to return normalized graph Laplacian or not
 *
 * @return a sparse symmetric N x N matrix
 *
 */
Matrix& laplacian(Matrix& X, std::string type, GraphOptions& options);

/**
 * Compute the symmetric adjacency matrix of the data set represented as
 * a real data matrix X. The diagonal elements of the sparse symmetric
 * adjacency matrix are all zero indicating that a sample should not be
 * a neighbor of itself. Note that in some cases, neighbors of a sample
 * may coincide with the sample itself, we set eps for those entries in
 * the sparse symmetric adjacency matrix.
 *
 * @param X data matrix with each row being a feature vector
 *
 * @param type graph type, either "nn" or "epsballs" ("eps")
 *
 * @param param integer if type is "nn", real number if type is "epsballs" ("eps")
 *
 * @param distFunc function mapping an (M x D) and a (N x D) matrix
 *                 to an M x N distance matrix (D: dimensionality)
 *                 either "euclidean" or "cosine"
 *
 * @return a sparse symmetric N x N  matrix of distances between the
 *         adjacent points
 *
 */
Matrix& adjacency(Matrix& X, std::string type, double param, std::string distFunc);

/**
 * Compute the directed adjacency matrix of the data set represented as
 * a real data matrix X. The diagonal elements of the sparse directed
 * adjacency matrix are all zero indicating that a sample should not be
 * a neighbor of itself. Note that in some cases, neighbors of a sample
 * may coincide with the sample itself, we set eps for those entries in
 * the sparse directed adjacency matrix.
 *
 * @param X data matrix with each row being a feature vector
 *
 * @param type graph type, either "nn" or "epsballs" ("eps")
 *
 * @param param integer if type is "nn", real number if type is "epsballs" ("eps")
 *
 * @param distFunc function mapping an (M x D) and an (N x D) matrix
 *                 to an M x N distance matrix (D: dimensionality)
 *                 either "euclidean" or "cosine"
 *
 * @return a sparse N x N matrix of distances between the
 *         adjacent points, not necessarily symmetric
 *
 */
Matrix& adjacencyDirected(Matrix& X, std::string type, double param, std::string distFunc);

/**
 * Compute the cosine distance matrix between row vectors in matrix A
 * and row vectors in matrix B.
 *
 * @param A data matrix with each row being a feature vector
 *
 * @param B data matrix with each row being a feature vector
 *
 * @return an n_A X n_B matrix with its (i, j) entry being the cosine
 * distance between i-th feature vector in A and j-th feature
 * vector in B, i.e.,
 * ||A(i, :) - B(j, :)|| = 1 - A(i, :)' * B(j, :) / || A(i, :) || * || B(j, :)||
 */
Matrix& cosine(Matrix& A, Matrix& B);

/**
 * Compute the cosine distance matrix between a vector V
 * and row vectors in matrix B.
 *
 * @param V a feature vector
 *
 * @param B data matrix with each row being a feature vector
 *
 * @return an n_B dimensional vector with its i-th entry being the cosine
 * distance between V and the i-th feature vector in B, i.e.,
 * ||V - B(j, :)|| = 1 - V' * B(j, :) / || V || * || B(j, :)||
 */
Vector& cosine(Vector& V, Matrix& B);

/**
 * Compute the Euclidean distance matrix between row vectors in matrix A
 * and row vectors in matrix B.
 *
 * @param A data matrix with each row being a feature vector
 *
 * @param B data matrix with each row being a feature vector
 *
 * @return an n_A X n_B matrix with its (i, j) entry being Euclidean
 * distance between i-th feature vector in A and j-th feature
 * vector in B, i.e., || X(i, :) - Y(j, :) ||_2
 *
 */
Matrix& euclidean(Matrix& A, Matrix& B);

/**
 * Compute the Euclidean distance matrix between a vector V
 * and row vectors in matrix B.
 *
 * @param V a feature vector
 *
 * @param B data matrix with each row being a feature vector
 *
 * @return an n_B dimensional vector with its i-th entry being Euclidean
 * distance between V and the i-th feature vector in B, i.e., || V - Y(i, :) ||_2
 *
 */
Vector& euclidean(Vector& V, Matrix& B);

/**
 * Compute local learning regularization matrix. Local learning
 * regularization only depends on kernel selection, distance
 * function, and neighborhood size.
 *
 * @param X data matrix with each row being a feature vector
 *
 * @param NN number of nearest neighbor
 *
 * @param distFunc function mapping an (M x D) and an (N x D) matrix
 *        to an M x N distance matrix (D: dimensionality)
 *        either "euclidean" or "cosine"
 *
 * @param kernelType  'linear' | 'poly' | 'rbf' | 'cosine'
 *
 * @param kernelParam    --    | degree | sigma |    --
 *
 * @param lambda graph regularization parameter
 *
 * @return local learning regularization matrix
 *
 */
Matrix& calcLLR(Matrix& X,
		double NN, std::string distFunc, std::string kernelType,
		double kernelParam, double lambda);




#endif /* MANIFOLD_H_ */
