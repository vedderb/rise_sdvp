/*
 * Classifier.h
 *
 *  Created on: Feb 15, 2014
 *      Author: Mingjie Qian
 */

#ifndef CLASSIFIER_H_
#define CLASSIFIER_H_

#include "Matrix.h"
#include "DenseMatrix.h"
#include <string>
#include <map>

class Classifier {

public:

	/**
	 * Number of classes.
	 */
	int nClass;

	/**
	 * Number of features, without bias dummy features,
	 * i.e., for SVM.
	 */
	int nFeature;

	/**
	 * Number of examples.
	 */
	int nExample;

	/**
	 * Training data matrix (nExample x nFeature),
	 * each row is a feature vector. The data
	 * matrix should not include bias dummy features.
	 */
	Matrix* X;

	/**
	 * Label matrix for training (nExample x nClass).
	 * Y_{i,k} = 1 if x_i belongs to class k, and 0 otherwise.
	 */
	Matrix* Y;

	/**
	 * LabelID array for training data, starting from 0.
	 * The label ID array for the training data is latent,
	 * and we don't need to know them. They are only meaningful
	 * for reconstructing the integer labels by using IDLabelMap
	 * structure.
	 */
	int* labelIDs;

	/**
	 * Label array for training data with original integer code.
	 */
	int* labels;

	/**
	 * Projection matrix (nFeature x nClass), column i is the projector for class i.
	 */
	Matrix* W;

	/**
	 * The biases in the linear model.
	 */
	double* b;

	/**
	 * Convergence tolerance.
	 */
	double epsilon;

	/**
	 * An ID to integer label mapping array. IDs start from 0.
	 */
	int* IDLabelMap;

public:

	/**
	 * Default constructor for a classifier.
	 */
	Classifier();

	virtual ~Classifier();

	/**
	 * Load the model for a classifier.
	 *
	 * @param filePath file path to load the model
	 *
	 */
	virtual void loadModel(std::string filePath) {};

	/**
	 * Save the model for a classifier.
	 *
	 * @param filePath file path to save the model
	 *
	 */
	virtual void saveModel(std::string filePath) {};

	/**
	 * Feed training data with original data matrix for this classifier.
	 *
	 * @param X original data matrix without bias dummy features, each row
	 *          is a feature vector
	 */
	void feedData(Matrix& X);

	/**
	 * Feed training data for this classification method.
	 *
	 * @param data an n x d 2D {@code double} array with each
	 *             column being a data sample
	 *
	 * @param n number of training examples
	 *
	 * @param d number of features
	 */
	void feedData(double** data, int n, int d);

	template<int d>
	void feedData(double data[][d], int n) {
		feedData(DenseMatrix::createDenseMatrix<d>(data, n));
	}

	template<int numFeature>
	void feedData(double data[][numFeature], int n, int d) {
		feedData(DenseMatrix::createDenseMatrix<numFeature>(data, n, d));
	}

	/**
	 * Infer the number of classes from a given label sequence.
	 *
	 * @param labels any integer array holding the original
	 *               integer labels
	 *
	 * @param len length of the label array
	 *
	 * @return number of classes
	 *
	 */
	static int calcNumClass(int* labels, int len);

	/**
	 * Get an ID to integer label mapping array. IDs start from 0.
	 *
	 * @param labels any integer array holding the original
	 *               integer labels
	 *
	 * @param len length of the label array
	 *
	 * @return ID to integer label mapping array
	 *
	 */
	static int* getIDLabelMap(int* labels, int len);

	/**
	 * Get a mapping from labels to IDs. IDs start from 0.
	 *
	 * @param labels any integer array holding the original
	 *               integer labels
	 *
	 * @param len length of the label array
	 *
	 * @return a mapping from labels to IDs
	 *
	 */
	static std::map<int, int>& getLabelIDMap(int* labels, int len);

	/**
	 * Feed labels of training data to the classifier.
	 *
	 * @param labels any integer array holding the original
	 *               integer labels
	 *
	 * @param len length of the label array
	 */
	void feedLabels(int* labels, int len);

	/**
	 * Feed labels for training data from a matrix.
	 * Note that if we feed the classifier with only
	 * label matrix, then we don't have original integer
	 * labels actually. In this case, label IDs will be
	 * inferred according to the label matrix. The first
	 * observed label index will be assigned ID 0, the second
	 * observed label index will be assigned ID 1, and so on.
	 * And labels will be the label indices in the given
	 * label matrix
	 *
	 * @param Y an N x K label matrix, where N is the number of
	 *          training examples, and K is the number of classes
	 *
	 */
	void feedLabels(Matrix& Y);

	/**
	 * Feed labels for this classification method.
	 *
	 * @param labels an n x c 2D {@code double} array
	 *
	 * @param n number of training examples
	 *
	 * @param c number of classes
	 */
	void feedLabels(double** labels, int n, int c);

	/**
	 * Train the classifier.
	 */
	virtual void train() {};

	/**
	 * Predict the labels for the test data formated as an original data matrix.
	 * The original data matrix should not include bias dummy features.
	 *
	 * @param Xt test data matrix with each row being a feature vector
	 *
	 * @return predicted label array with original integer label code
	 *
	 */
	int* predict(Matrix& Xt);

	/**
	 * Predict the labels for the test data formated as an original 2D
	 * {@code double} array. The original data matrix should not
	 * include bias dummy features.
	 *
	 * @param Xt an n x d 2D {@code double} array with each
	 *           row being a data sample
	 *
	 * @param nt number of test examples
	 *
	 * @param d number of features
	 *
	 * @return predicted label array with original integer label code
	 *
	 */
	int* predict(double** Xt, int nt, int d);

	template<int d>
	int* predict(double Xt[][d], int nt) {
		return predict(DenseMatrix::createDenseMatrix<d>(Xt, nt));
	}

	template<int numFeature>
	int* predict(double Xt[][numFeature], int nt, int d) {
		return predict(DenseMatrix::createDenseMatrix<numFeature>(Xt, nt));
	}

	/**
	 * Predict the label score matrix given test data formated as an
	 * original data matrix.
	 *
	 * Note that if a method of an abstract class is declared as
	 * abstract, it is implemented as an interface function in Java.
	 * Thus subclass needs to implement this abstract method rather
	 * than to override it.
	 *
	 * @param Xt test data matrix with each row being a feature vector
	 *
	 * @return predicted N x K label score matrix, where N is the number of
	 *         test examples, and K is the number of classes
	 *
	 */
	virtual Matrix& predictLabelScoreMatrix(Matrix& Xt) = 0;

	/**
	 * Predict the label score matrix given test data formated as an
	 * original data matrix.
	 *
	 * @param Xt an n x d 2D {@code double} array with each
	 *           row being a data sample
	 *
	 * @param nt number of test examples
	 *
	 * @param d number of features
	 *
	 * @return predicted N x K label score matrix, where N is the number of
	 *         test examples, and K is the number of classes
	 *
	 */
	Matrix& predictLabelScoreMatrix(double** Xt, int nt, int d);

	template<int d>
	Matrix& predictLabelScoreMatrix(double Xt[][d], int nt) {
		return predictLabelScoreMatrix(DenseMatrix::createDenseMatrix<d>(Xt, nt));
	}

	template<int numFeature>
	Matrix& predictLabelScoreMatrix(double Xt[][numFeature], int nt, int d) {
		return predictLabelScoreMatrix(DenseMatrix::createDenseMatrix<numFeature>(Xt, nt));
	}

	/**
	 * Predict the label matrix given test data formated as an
	 * original data matrix.
	 *
	 * Note that if a method of an abstract class is declared as
	 * abstract, it is implemented as an interface function in Java.
	 * Thus subclasses need to implement this abstract method rather
	 * than to override it.
	 *
	 * @param Xt test data matrix with each row being a feature vector
	 *
	 * @return predicted N x K label matrix, where N is the number of
	 *         test examples, and K is the number of classes
	 *
	 */
	Matrix& predictLabelMatrix(Matrix& Xt);

	/**
	 * Predict the label matrix given test data formated as an
	 * original 2D {@code double} array.
	 *
	 * @param Xt an n x d 2D {@code double} array with each
	 *           row being a data sample
	 *
	 * @param nt number of test examples
	 *
	 * @param d number of features
	 *
	 * @return predicted N x K label matrix, where N is the number of
	 *         test examples, and K is the number of classes
	 *
	 */
	Matrix& predictLabelMatrix(double** Xt, int nt, int d);

	template<int d>
	Matrix& predictLabelMatrix(double Xt[][d], int nt) {
		return predictLabelMatrix(DenseMatrix::createDenseMatrix<d>(Xt, nt));
	}

	template<int numFeature>
	Matrix& predictLabelMatrix(double Xt[][numFeature], int nt, int d) {
		return predictLabelMatrix(DenseMatrix::createDenseMatrix<numFeature>(Xt, nt));
	}

	/**
	 * Get accuracy for a classification task.
	 *
	 * @param pre_labels predicted labels
	 *
	 * @param labels true labels
	 *
	 * @param len length of the label arrays
	 *
	 * @return accuracy
	 *
	 */
	static double getAccuracy(int* pre_labels, int* labels, int len);

	/**
	 * Get projection matrix for this classifier.
	 *
	 * @return a d x c projection matrix
	 *
	 */
	Matrix* getProjectionMatrix() {
		return W;
	}

	/**
	 * Get ground truth label matrix for training data.
	 *
	 * @return an n x c label matrix
	 *
	 */
	Matrix* getTrainingLabelMatrix() {
		return Y;
	}


	/**
	 * Convert a label matrix to a label index array. Label indices start from 0.
	 *
	 * @param Y label matrix
	 *
	 * @return a label index array
	 *
	 */
	static int* labelScoreMatrix2LabelIndexArray(Matrix& Y);

	/**
	 * Convert a label index array to a label matrix. Label indices start from 0.
	 *
	 * @param labelIndices a label index array
	 *
	 * @param len length of the label index array
	 *
	 * @param nClass number of classes
	 *
	 * @return label matrix
	 *
	 */
	static Matrix& labelIndexArray2LabelMatrix(int* labelIndices, int len, int nClass);

};

#endif /* CLASSIFIER_H_ */
