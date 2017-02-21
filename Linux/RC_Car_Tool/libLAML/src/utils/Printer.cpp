/*
 * Printer.cpp
 *
 *  Created on: Feb 8, 2014
 *      Author: Aaron
 */

#include "Printer.h"
#include "DenseMatrix.h"
#include "SparseMatrix.h"
#include "DenseVector.h"
#include "SparseVector.h"
#include <stdio.h>
#include <stdarg.h>
#include <cmath>
#include <sstream>
#include <iostream>
/*
#include <memory>
#include <cstdlib>
#include <iostream>
 */
// using namespace std;

std::string int2str(int number) {

	if (number == 0)
		return "0";

	bool positive = number < 0 ? false : true;
	if (!positive)
		number *= -1;

	std::list<char> list;
	int digit = 0;
	while (number != 0) {
		digit = number % 10;
		list.push_back('0' + digit);
		number /= 10;
	}
	int size = positive ? list.size() + 1 : list.size() + 2;
	char* resArr = new char[size];
	std::list<char>::iterator iter = list.end();
	size_t k = 0;
	if (!positive)
		resArr[k++] = '-';
	while (iter != list.begin())
		resArr[k++] = *(--iter);
	resArr[k] = 0;
	std::string res(resArr);
	delete[] resArr;
	return res;
	/*
	 * Don't use stringstream, there might be a weird error
	 * about alternative resource deadlock.
	 */
	/*std::ostringstream ss;//create a stringstream
	ss << number;//add number to the stream
	std::string res = ss.str();
	return res;//return a string with the contents of the stream*/
}

void print(std::string input) {
	std::cout << input << std::flush;
}

void println(void) {
	std::cout << std::endl <<std::flush;
}

void println(std::string input) {
	std::cout << input << std::endl <<std::flush;
}

void printf(const std::string fmt, ...) {
	int size = 100;
	std::string str;
	va_list ap;
	while (1) {
		str.resize(size);
		va_start(ap, fmt);
		int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
		va_end(ap);
		if (n > -1 && n < size) {
			str.resize(n);
			print(str);
			return;
		}
		if (n > -1)
			size = n + 1;
		else
			size *= 2;
	}
	print(str);
}

void fprintf(const std::string fmt, ...) {
	int size = 100;
	std::string str;
	va_list ap;
	while (1) {
		str.resize(size);
		va_start(ap, fmt);
		int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
		va_end(ap);
		if (n > -1 && n < size) {
			str.resize(n);
			print(str);
			return;
		}
		if (n > -1)
			size = n + 1;
		else
			size *= 2;
	}
	print(str);
}

void errf(const std::string fmt, ...) {
	int size = 100;
	std::string str;
	va_list ap;
	while (1) {
		str.resize(size);
		va_start(ap, fmt);
		int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
		va_end(ap);
		if (n > -1 && n < size) {
			str.resize(n);
			std::cout << str << std::flush;
			return;
		}
		if (n > -1)
			size = n + 1;
		else
			size *= 2;
	}
	print(str);
}

std::string sprintf(const std::string fmt, ...) {
	int size = 100;
	std::string str;
	va_list ap;
	while (1) {
		str.resize(size);
		va_start(ap, fmt);
		int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
		va_end(ap);
		if (n > -1 && n < size) {
			str.resize(n);
			return str;
		}
		if (n > -1)
			size = n + 1;
		else
			size *= 2;
	}
	return str;
}

void disp(std::string input) {
	println(input);
}

void err(std::string input) {
	std::cerr << input << std::endl << std::flush;
}

void Printer::printMatrix(Matrix& A, int p) {
	if (typeid(A) == typeid(SparseMatrix)) {
		if (((SparseMatrix&) A).getNNZ() == 0) {
			std::cout << "Empty sparse matrix." << std::endl << std::flush;
			return;
		}
		SparseMatrix& S = (SparseMatrix&) A;
		int* ic = S.getIc();
		int* jr = S.getJr();
		double* pr = S.getPr();
		int* valCSRIndices = S.getValCSRIndices();
		int M = S.getRowDimension();
		std::string valueString = "";
		for (int r = 0; r < M; r++) {
			std::cout << "  ";
			int currentColumn = 0;
			int lastColumn = -1;
			for (int k = jr[r]; k < jr[r + 1]; k++) {
				currentColumn = ic[k];
				while (lastColumn < currentColumn - 1) {
					printf(sprintf("%%%ds", 8 + p - 4), " ");
					print("  ");
					lastColumn++;
				}
				lastColumn = currentColumn;
				double v = pr[valCSRIndices[k]];
				int rv = (int) round(v);
				if (v != rv)
					valueString = sprintf(sprintf("%%.%df", p), v);
				else
					valueString = sprintf("%d", rv);
				printf(sprintf("%%%ds", 8 + p - 4), valueString.c_str());
				print("  ");
			}
			println();
		}
		println();
		return;
	} else if (typeid(A) == typeid(DenseMatrix)) {
		if (((DenseMatrix&) A).getData() == null) {
			println("Empty matrix.");
			return;
		}
		for (int i = 0; i < A.getRowDimension(); i++) {
			print("  ");
			for (int j = 0; j < A.getColumnDimension(); j++) {
				std::string valueString = "";
				double v = A.getEntry(i, j);
				int rv = (int) round(v);
				if (v != rv)
					valueString = sprintf(sprintf("%%.%df", p), v);
				else
					valueString = sprintf("%d", rv);
				printf(sprintf("%%%ds", 8 + p - 4), valueString.c_str());
				print("  ");
			}
			println();
		}
		println();
	}
}

void printMatrix(Matrix& A, int p) {
	if (typeid(A) == typeid(SparseMatrix)) {
		if (((SparseMatrix&) A).getNNZ() == 0) {
			std::cout << "Empty sparse matrix." << std::endl << std::flush;
			return;
		}
		SparseMatrix& S = (SparseMatrix&) A;
		int* ic = S.getIc();
		int* jr = S.getJr();
		double* pr = S.getPr();
		int* valCSRIndices = S.getValCSRIndices();
		int M = S.getRowDimension();
		std::string valueString = "";
		for (int r = 0; r < M; r++) {
			std::cout << "  ";
			int currentColumn = 0;
			int lastColumn = -1;
			for (int k = jr[r]; k < jr[r + 1]; k++) {
				currentColumn = ic[k];
				while (lastColumn < currentColumn - 1) {
					printf(sprintf("%%%ds", 8 + p - 4), " ");
					print("  ");
					lastColumn++;
				}
				lastColumn = currentColumn;
				double v = pr[valCSRIndices[k]];
				int rv = (int) round(v);
				if (v != rv)
					valueString = sprintf(sprintf("%%.%df", p), v);
				else
					valueString = sprintf("%d", rv);
				printf(sprintf("%%%ds", 8 + p - 4), valueString.c_str());
				print("  ");
			}
			println();
		}
		println();
		return;
	} else if (typeid(A) == typeid(DenseMatrix)) {
		if (((DenseMatrix&) A).getData() == null) {
			println("Empty matrix.");
			return;
		}
		for (int i = 0; i < A.getRowDimension(); i++) {
			print("  ");
			for (int j = 0; j < A.getColumnDimension(); j++) {
				std::string valueString = "";
				double v = A.getEntry(i, j);
				int rv = (int) round(v);
				if (v != rv)
					valueString = sprintf(sprintf("%%.%df", p), v);
				else
					valueString = sprintf("%d", rv);
				printf(sprintf("%%%ds", 8 + p - 4), valueString.c_str());
				print("  ");
			}
			println();
		}
		println();
	}
}

void printMatrix(Matrix& A) {
	printMatrix(A, 4);
}

void printDenseMatrix(Matrix& A, int p) {
	if (typeid(A) != typeid(DenseMatrix)) {
		err("DenseMatrix input is expected.");
		return;
	}
	if (((DenseMatrix&) A).getData() == null) {
		println("Empty matrix.");
		return;
	}
	for (int i = 0; i < A.getRowDimension(); i++) {
		print("  ");
		for (int j = 0; j < A.getColumnDimension(); j++) {
			std::string valueString = "";
			double v = A.getEntry(i, j);
			int rv = (int) round(v);
			if (v != rv)
				valueString = sprintf(sprintf("%%.%df", p), v);
			else
				valueString = sprintf("%d", rv);
			printf(sprintf("%%%ds", 8 + p - 4), valueString.c_str());
			print("  ");
		}
		println();
	}
	println();
}

void printDenseMatrix(Matrix& A) {
	printDenseMatrix(A, 4);
}

void printSparseMatrix(Matrix& A, int p) {

	if (typeid(A) != typeid(SparseMatrix)) {
		err("SparseMatrix input is expected.");
		return;
	}
	if (((SparseMatrix&) A).getNNZ() == 0) {
		println("Empty sparse matrix.");
		println();
		return;
	}
	int nRow = A.getRowDimension();
	int nCol = A.getColumnDimension();
	std::string leftFormat = sprintf("  %%%ds, ", int2str(nRow).length() + 1);
	std::string rightFormat = sprintf("%%-%ds", int2str(nCol).length() + 2);
	std::string format = leftFormat + rightFormat + sprintf("%%%ds", 8 + p - 4);
	SparseMatrix& S = (SparseMatrix&) A;
	int* ir = S.getIr();
	int* jc = S.getJc();
	double* pr = S.getPr();
	int N = S.getColumnDimension();
	std::string valueString = "";
	int i = -1;
	for (int j = 0; j < N; j++) {
		for (int k = jc[j]; k < jc[j + 1]; k++) {
			print("  ");
			i = ir[k];
			double v = pr[k];
			int rv = (int) round(v);
			if (v != rv)
				valueString = sprintf(sprintf("%%.%df", p), v);
			else
				valueString = sprintf("%d", rv);
			/*print(sprintf(sprintf("(%d, %d)%%%ds", i + 1, j + 1, 8 + p - 4), valueString));
				println();*/
			std::string leftString = sprintf("(%d", i + 1);
			std::string rightString = sprintf("%d)", j + 1);
			println(sprintf(format, leftString.c_str(), rightString.c_str(), valueString.c_str()));
		}
	}
	println();

}

void printSparseMatrix(Matrix& A) {
	printSparseMatrix(A, 4);
}

void printDenseVector(Vector& V, int p) {
	if (typeid(V) == typeid(DenseVector)) {
		int dim = V.getDim();
		double* pr = ((DenseVector&) V).getPr();
		for (int k = 0; k < dim; k++) {
			print("  ");
			// std::cout << "  ";
			double v = pr[k];
			int rv = (int) round(v);
			std::string valueString;
			if (v != rv)
				valueString = sprintf(sprintf("%%.%df", p), v);
			else
				valueString = sprintf("%d", rv);
			print(sprintf(sprintf("%%%ds", 8 + p - 4), valueString.c_str()));
			println();
		}
		println();
	} else {
		err("The input vector should be a DenseVector instance");
		exit(1);
	}
}

void printDenseVector(Vector& V) {
	printDenseVector(V, 4);
}

void printSparseVector(Vector& V, int p) {
	if (typeid(V) == typeid(SparseVector)) {
		int* ir = ((SparseVector&) V).getIr();
		double* pr = ((SparseVector&) V).getPr();
		int nnz = ((SparseVector&) V).getNNZ();
		for (int k = 0; k < nnz; k++) {
			print("  ");
			int idx = ir[k];
			double v = pr[k];
			int rv = (int) round(v);
			std::string valueString;
			if (v != rv)
				valueString = sprintf(sprintf("%%.%df", p), v);
			else
				valueString = sprintf("%d", rv);
			print(sprintf(sprintf("(%d, 1)%%%ds", idx + 1, 8 + p - 4), valueString.c_str()));
			println();
		}
		println();
	} else {
		err("The input vector should be a SparseVector instance");
		exit(1);
	}
}

void printVector(Vector& V, int p) {
	if (typeid(V) == typeid(DenseVector)) {
		printDenseVector(V, p);
	} else {
		printSparseVector(V, p);
	}
}

void printVector(Vector& V) {
	printVector(V, 4);
}

void printVector(double* V, int dim, int p) {
	for (int i = 0; i < dim; i++) {
		print("  ");
		std::string valueString = "";
		double v = V[i];
		int rv = (int) round(v);
		if (v != rv)
			valueString = sprintf(sprintf("%%.%df", p), v);
		else
			valueString = sprintf("%d", rv);
		print(sprintf(sprintf("%%%ds", 8 + p - 4), valueString.c_str()));
	}
	println();
	println();
}

void printVector(double* V, int dim) {
	printVector(V, dim, 4);
}

void printSparseVector(Vector& V) {
	printSparseVector(V, 4);
}

void display(Matrix& A, int p) {
	if (typeid(A) == typeid(DenseMatrix)) {
		printDenseMatrix(A, p);
	} else if (typeid(A) == typeid(SparseMatrix)) {
		printSparseMatrix(A, p);
	}
}

void display(Matrix& A) {
	display(A, 4);
}

void display(Vector& V, int p) {
	printVector(V, p);
}

void display(Vector& V) {
	display(V, 4);
}

void display(double* V, int dim, int p) {
	printVector(*new DenseVector(V, dim), p);
}

void display(double* V, int dim) {
	display(V, dim, 4);
}

void disp(Matrix& A, int p) {
	display(A, p);
}

void disp(Matrix& A) {
	display(A, 4);
}

void disp(Vector& V, int p) {
	display(V, p);
}

void disp(Vector& V) {
	display(V, 4);
}

void disp(double* V, int dim, int p) {
	display(*new DenseVector(V, dim), p);
}

void disp(double* V, int dim) {
	display(*new DenseVector(V, dim), 4);
}

void disp(int* V, int len) {

	if (V == NULL) {
		println("Empty vector!");
		return;
	}

	for (int i = 0; i < len; i++) {
		print("  ");
		std::string valueString = "";
		double v = V[i];
		int rv = (int) round(v);
		if (v != rv)
			valueString = sprintf("%.4f", v);
		else
			valueString = sprintf("%d", rv);
		fprintf("%7s", valueString.c_str());
		print("  ");
		// System.out.println();
	}
	println();

}
