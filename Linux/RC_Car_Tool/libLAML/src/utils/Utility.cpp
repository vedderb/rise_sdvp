/*
 * Utility.cpp
 *
 *  Created on: Feb 7, 2014
 *      Author: Aaron
 */

#include "Utility.h"
#include <cmath>

const double inf = 1.0 / 0.0;
const double POSITIVE_INFINITY = 1.0 / 0.0;
const double NEGATIVE_INFINITY = -1.0 / 0.0;

const double MYNAN = sqrt(-1);

const double eps = 1e-128;

const double PI = 2 * acos(0.0);

bool StringTokenizer::hit(char& c) {
	std::string& delim = *this->delim;
	for (size_t i = 0; i < delim.length(); i++) {
		if (c == delim[i]) {
			return true;
		}
	}
	return false;
}

size_t StringTokenizer::findFirst(std::string& inputString, size_t prev) {
	if (prev >= inputString.length()) {
		return std::string::npos;
	}
	for (size_t i = prev; i < inputString.length(); i++) {
		if (hit(inputString[i])) {
			return i;
		}
	}
	return std::string::npos;
}

StringTokenizer::StringTokenizer(std::string& inputString, std::string& delim) {
	str = &inputString;
	this->delim = &delim;
	pos = 0;
}

StringTokenizer::~StringTokenizer() {
	str = NULL;
	delim = NULL;
}

std::string StringTokenizer::nextToken0() {
	std::string& str = *this->str;
	size_t prev = pos;
	size_t next = findFirst(str, prev);
	pos = next + 1;
	if (next == std::string::npos && prev < str.length()) {
		return str.substr(prev);
	}
	if (next != std::string::npos && next != prev) {
		return str.substr(prev, next - prev);
	} else {
		return "";
	}
}

std::string StringTokenizer::nextToken() {
	std::string& str = *this->str;
	size_t prev = pos;
	size_t next = findFirst(str, prev);
	while ((next = findFirst(str, prev)) != std::string::npos) {
		if (next != prev) {
			pos = next + 1;
			return str.substr(prev, next - prev);
		}
		prev = next + 1;
	}
	if (prev < str.length()) {
		pos = str.length();
		return str.substr(prev);
	} else {
		pos = str.length();
		return "";
	}
}

int StringTokenizer::countTokens() {
	std::string& str = *this->str;
	size_t prev = pos;
	size_t next = 0;
	size_t cnt = 0;
	while ((next = findFirst(str, prev)) != std::string::npos) {
		if (next != prev)
			cnt++;
		prev = next + 1;
	}
	if (prev < str.length())
		cnt++;
	return cnt;
}

/**
 * Convert a string into a real scalar.
 *
 * @param numString the string to parse for the double value
 *
 * @return double value represented by numString
 */
double atof(std::string numString) {

	double res = 0;
	size_t k = 0;
	char ch = 0;

	char sign = '+';

	// Convert string to integer
	while (k < numString.length()) {

		ch = numString[k++];
		if (ch == '.') {
			break;
		}

		if (ch == ' ') {
			continue;
		}

		if (ch == '\n') {
			continue;
		}

		if (ch == '\r') {
			continue;
		}

		if (ch == '+' || ch == '-') {
			sign = ch;
			continue;
		}

		if (ch < '0' || ch > '9') {
			exit(1);
		}

		res = 10 * res + (ch - '0');

	}

	// Convert the remaining substring to a decimal fraction
	if (ch == '.') {
		double base = 0.1;
		while (k < numString.length()) {
			ch = numString[k++];
			if (ch == ' ') {
				continue;
			}
			if (ch == '\n') {
				continue;
			}
			if (ch == '\r') {
				continue;
			}
			if (ch < '0' || ch > '9') {
				exit(1);
			}
			res = res + base * (ch - '0');
			base = 0.1 * base;
		}
	}

	if (sign == '-')
		res *= -1;

	return res;

}

/**
 * Convert a string into an integer.
 *
 * @param numString the string to parse for the integer value
 *
 * @return int value represented by numString
 */
int atoi(std::string numString) {
	int res = 0;
	size_t k = 0;
	char ch = 0;

	char sign = '+';

	// Convert string to integer
	while (k < numString.length()) {

		ch = numString[k++];
		if (ch == '.') {
			break;
		}

		if (ch == ' ') {
			continue;
		}

		if (ch == '\n') {
			continue;
		}

		if (ch == '\r') {
			continue;
		}

		if (ch == '+' || ch == '-') {
			sign = ch;
			continue;
		}

		if (ch < '0' || ch > '9') {
			exit(1);
		}

		res = 10 * res + (ch - '0');

	}
	if (sign == '-')
		res *= -1;
	return res;
}

double max(double a, double b) {
	return a > b ? a : b;
}

double min(double a, double b) {
	return a < b ? a : b;
}

/*using namespace std;


template <class T1, class T2>
class PairSecondComp : public binary_function<pair<T1, T2>, pair<T1, T2>, bool> {
public:
	bool operator()(pair<T1, T2> const& lhs, pair<T1, T2> const& rhs) const {
		return lhs.second < rhs.second;
	}
};

template <class T1, class T2>
class PairFirstThenSecondComp : public binary_function<pair<T1, T2>, pair<T1, T2>, bool> {
public:
	bool operator()(pair<T1, T2> const& lhs, pair<T1, T2> const& rhs) const {
		bool res = false;
		if (lhs.first == rhs.first)
			res = lhs.second < rhs.second;
		else
			res = lhs.first <= rhs.first;
		return res;
	}
};

template <class T1, class T2>
class PairSecondThenFirstComp : public binary_function<pair<T1, T2>, pair<T1, T2>, bool> {
public:
	bool operator()(pair<T1, T2> const& lhs, pair<T1, T2> const& rhs) const {
		bool res = false;
		if (lhs.second == rhs.second)
			res = lhs.first <= rhs.first;
		else
			res = lhs.second <= rhs.second;
		return res;
	}
};

class comp : public binary_function<pair<int, int>, pair<int, int>, bool> {
public:
	bool operator()(pair<int, int> const& lhs, pair<int, int> const& rhs) const {
		bool res = false;
		if (lhs.first == rhs.first)
			res = lhs.second <= rhs.second;
		else
			res = lhs.first <= rhs.first;
		return res;
	}
};*/


