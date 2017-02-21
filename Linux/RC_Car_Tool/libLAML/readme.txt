libLAML

libLAML (Linear Algebra and Machine Learning) is a stand-alone pure C++ library for linear algebra and machine learning. libLAML can be compiled on MinGW, Linux, and Mac OS X. The goal is to build efficient and easy-to-use linear algebra and machine learning libraries. libLAML allows full control of the basic data structures for matrices and vectors, which is required to have fast implementation for machine learning methods. Additionally, libLAML provides a lot of commonly used matrix functions in the same signature to MATLAB, thus can also be used to manually convert MATLAB code to C++ code.

The built-in linear algebra library supports well-designed dense or sparse matrices and vectors. Standard compressed sparse column (CSC) and compressed sparse row (CSR) are used to design and implement sparse matrices. Unlike other linear algebra libraries in C++, the built-in linear algebra library in libLAML gives users full control of sparse matrices and vectors (e.g., the interior arrays in sparse matrices), which is crucial to make efficient high level implementations.

libLAML v.s. LAML

libLAML is at least 4 times faster than LAML, though I merely convert LAML in Java to libLAML in C++. I believe the only reason is programming language. For runtime performance, I guess C++ is better than Java. Actually, it won't take so much time to manually convert Java code to C++ code.

Features:
Stand-alone C++ library
Built-in Linear Algebra (LA) library
Full control of matrices and vectors
Fast implementation of Machine Learning (ML) methods
Matrix functions with almost the same signature to MATLAB
Well documented source code and friendly API, very easy to use

Packages:
./decomposition
	LU, QR, eigenvalue decomposition, and SVD
./matrix
	Sparse and dense matrix implementation
./vector
	Sparse and dense vector implementation
./utils
    Matlab-style functions, printer, time, array operations, and fast in-place operations
./optimization
	L-BFGS, BoundConstrainedPLBFGS, NonnegativePLBFGS, Projection, ProximalMapping, ShrinkageOperator, accelerated proximal gradient, accelerated gradient descent, nonlinear conjugate gradient
./classification
	Linear SVM, linear multi-class SVM, logistic regression
./kernel
	Commonly used kernel functions ('linear' | 'poly' | 'rbf' | 'cosine')
./manifold
	Commonly used manifold learning functions such as computing adjacency matrix, Laplacian matrix, and local learning regularization matrix
./clustering
	K-means, L1NMF, NMF, and spectral clustering
./regression
	LASSO
./random
	Multivariate Gaussian distribution
./recovery
	Matrix completion and robust PCA
./sequence
	Hidden Markov Models (HMM) and Conditional Random Fields (CRF)

-----------------------------------
Version: 1.4
Author: Mingjie Qian
Date: Mar. 5th, 2014
