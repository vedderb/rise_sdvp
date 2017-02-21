/*
 * Classifier.cpp
 *
 *  Created on: Feb 15, 2014
 *      Author: Mingjie Qian
 */

#include "Classifier.h"
#include <set>
#include <list>
#include <limits>
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "Printer.h"
#include "Matlab.h"

Classifier::Classifier() {
	nClass = 0;
	nFeature = 0;
	nExample = 0;
	X = null;
	W = null;
	Y = NULL;
	b = NULL;
	labels = NULL;
	IDLabelMap = NULL;
	labelIDs = NULL;
	epsilon = 1e-4;
}

Classifier::~Classifier() {
	delete X;
	delete Y;
	delete W;
	delete b;
	delete[] labels;
	delete[] IDLabelMap;
	delete[] labelIDs;
}

void Classifier::feedData(Matrix& X) {
	this->X = &X;
	nFeature = X.getColumnDimension();
	nExample = X.getRowDimension();
}

void Classifier::feedData(double** data, int n, int d) {
	feedData(*new DenseMatrix(data, n, d));
}

int Classifier::calcNumClass(int* labels, int len) {
	std::set<int> set;
	int label = -1;
	for (int i = 0; i < len; i++) {
		label = labels[i];
		if (set.find(label) == set.end())
			set.insert(label);
	}
	int nClass = set.size();
	return nClass;
}

int* Classifier::getIDLabelMap(int* labels, int len) {
	std::set<int> set;
	std::list<int> list;
	int label = -1;
	for (int i = 0; i < len; i++) {
		label = labels[i];
		if (set.find(label) == set.end()) {
			set.insert(label);
			list.push_back(label);
		}
	}
	int nClass = set.size();
	int ID = 0;
	int* IDLabelArray = new int[nClass];
	for (std::list<int>::iterator iter = list.begin(); iter != list.end(); iter++) {
		IDLabelArray[ID++] = *iter;
	}
	return IDLabelArray;
}

std::map<int, int>& Classifier::getLabelIDMap(int* labels, int len) {
	std::map<int, int>* labelIDMap = new std::map<int, int>;
	int ID = 0;
	int label = -1;
	for (int i = 0; i < len; i++) {
		label = labels[i];
		if (labelIDMap->find(label) == labelIDMap->end()) {
			(*labelIDMap)[label] = ID++;
		}
	}
	return *labelIDMap;
}

void Classifier::feedLabels(int* labels, int len) {
	nClass = calcNumClass(labels, len);
	IDLabelMap = getIDLabelMap(labels, len);
	std::map<int, int>& labelIDMap = getLabelIDMap(labels, len);
	int* labelIDs = new int[len];
	for (int i = 0; i < len; i++) {
		labelIDs[i] = labelIDMap[labels[i]];
	}
	int* labelIndices = labelIDs;
	Y = &labelIndexArray2LabelMatrix(labelIndices, len, nClass);
	this->labels = labels;
	this->labelIDs = labelIndices;
}

void Classifier::feedLabels(Matrix& Y) {
	this->Y = &Y;
	nClass = Y.getColumnDimension();
	if (nExample != Y.getRowDimension()) {
		err("Number of labels error!");
		exit(1);
	}
	int* labelIndices = labelScoreMatrix2LabelIndexArray(Y);
	labels = labelIndices;
	IDLabelMap = getIDLabelMap(labels, nExample);
	labelIDs = labelIndices;
}

void Classifier::feedLabels(double** labels, int n, int c) {
	feedLabels(*new DenseMatrix(labels, n, c));
}

int* Classifier::predict(Matrix& Xt) {
	Matrix& Yt = predictLabelScoreMatrix(Xt);
	/*
	 * Because column vectors of W are arranged according to the
	 * order of observation of class labels, in this case, label
	 * indices predicted from the label score matrix are identical
	 * to the latent label IDs, and labels can be inferred by the
	 * IDLabelMap structure.
	 */
	int* labelIndices = labelScoreMatrix2LabelIndexArray(Yt);
	int nt = Xt.getRowDimension();
	int* labels = new int[nt];
	for (int i = 0; i < nt; i++) {
		labels[i] = IDLabelMap[labelIndices[i]];
	}
	return labels;
}

int* Classifier::predict(double** Xt, int nt, int d) {
	return predict(*new DenseMatrix(Xt, nt, d));
}

Matrix& Classifier::predictLabelScoreMatrix(double** Xt, int nt, int d) {
	return predictLabelScoreMatrix(*new DenseMatrix(Xt, nt, d));
}

Matrix& Classifier::predictLabelMatrix(Matrix& Xt) {
	Matrix& Yt = predictLabelScoreMatrix(Xt);
	int* labelIndices = labelScoreMatrix2LabelIndexArray(Yt);
	return labelIndexArray2LabelMatrix(labelIndices, Yt.getRowDimension(), nClass);
}

Matrix& Classifier::predictLabelMatrix(double** Xt, int nt, int d) {
	return predictLabelMatrix(*new DenseMatrix(Xt, nt, d));
}

double Classifier::getAccuracy(int* pre_labels, int* labels, int len) {
	int N = len;
	int cnt_correct = 0;
	for ( int i = 0; i < N; i ++ ) {
		if ( pre_labels[i] == labels[i] )
			cnt_correct ++;
	}
	double accuracy = (double) cnt_correct / (double) N;
	println(sprintf("Accuracy: %.2f%%\n", accuracy * 100));
	return accuracy;
}

int* Classifier::labelScoreMatrix2LabelIndexArray(Matrix& Y) {
	int* labelIndices = new int[Y.getRowDimension()];

	if (typeid(Y) == typeid(SparseMatrix)) {
		int* ic = ((SparseMatrix&) Y).getIc();
		int* jr = ((SparseMatrix&) Y).getJr();
		int* valCSRIndices = ((SparseMatrix&) Y).getValCSRIndices();
		double* pr = ((SparseMatrix&) Y).getPr();
		for (int i = 0; i < Y.getRowDimension(); i++) {
			double max = -1.0 / 0.0;
			labelIndices[i] = 0;
			for (int k = jr[i]; k < jr[i + 1]; k++) {
				if (max < pr[valCSRIndices[k]]) {
					max = pr[valCSRIndices[k]];
					labelIndices[i] = ic[k];
				}
			}
		}
	} else {
		double** YData = ((DenseMatrix&) Y).getData();
		for (int i = 0; i < Y.getRowDimension(); i++) {
			double max = -1.0 / 0.0;
			labelIndices[i] = 0;
			for (int j = 0; j < Y.getColumnDimension(); j++) {
				if (max < YData[i][j]) {
					max = YData[i][j];
					labelIndices[i] = j;
				}
			}
		}
	}

	return labelIndices;
}

Matrix& Classifier::labelIndexArray2LabelMatrix(int* labelIndices, int len, int nClass) {
	int* rIndices = new int[len];
	int* cIndices = new int[len];
	double* values = new double[len];
	for (int i = 0; i < len; i++) {
		rIndices[i] = i;
		cIndices[i] = labelIndices[i];
		values[i] = 1;
	}
	return *new SparseMatrix(rIndices, cIndices, values, len, nClass, len);
}
