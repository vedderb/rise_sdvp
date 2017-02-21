/*
 * Pair.h
 *
 *  Created on: Feb 7, 2014
 *      Author: Aaron
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <map>
#include <algorithm>
#include <functional>
#include <string>
#include <limits>
#include <cstdlib>

#ifndef null
#define null NULL
#endif

#ifndef MIN_VALUE
#define MIN_VALUE (double)1e-312;
#endif

/*
#ifndef POSITIVE_INFINITY
#define POSITIVE_INFINITY std::numeric_limits<double>::max();
#endif

#ifndef NEGATIVE_INFINITY
#define NEGATIVE_INFINITY -std::numeric_limits<double>::max();
#endif
*/

#ifndef MY_INFTY
#define MY_INFTY
extern const double inf;
extern const double POSITIVE_INFINITY;
extern const double NEGATIVE_INFINITY;
#endif

#ifndef MY_NAN
#define MY_NAN
extern const double MYNAN;
#endif

#ifndef EPS
#define EPS
extern const double eps;
#endif

#ifndef M_PI
#define M_PI
extern const double PI;
#endif

class StringTokenizer {
private:
	std::string* str;
	std::string* delim;
	size_t pos;
	bool hit(char& c);
	size_t findFirst(std::string& inputString, size_t prev);
public:
	StringTokenizer(std::string& inputString, std::string& delim);
	~StringTokenizer();
	std::string nextToken0();
	std::string nextToken();
	int countTokens();
};

/**
 * Convert a string into a real scalar.
 *
 * @param numString the string to parse for the double value
 *
 * @return double value represented by numString
 */
double atof(std::string numString);

/**
 * Convert a string into an integer.
 *
 * @param numString the string to parse for the integer value
 *
 * @return int value represented by numString
 */
int atoi(std::string numString);

double max(double a, double b);

double min(double a, double b);

template <class T>
T* clone(T* input, int len) {
	T* res = new T[len];
	/*for (int i = 0; i < len; i++)
		res[i] = input[i];*/
	std::copy(input, input + len, res);
	return res;
};

template <class T1, class T2>
class PairSecondComp : public std::binary_function<std::pair<T1, T2>, std::pair<T1, T2>, bool> {
public:
	bool operator()(std::pair<T1, T2> const& lhs, std::pair<T1, T2> const& rhs) const {
		return lhs.second < rhs.second;
	}
};

template <class T1, class T2>
class PairFirstThenSecondComp : public std::binary_function<std::pair<T1, T2>, std::pair<T1, T2>, bool> {
public:
	bool operator()(std::pair<T1, T2> const& lhs, std::pair<T1, T2> const& rhs) const {
		bool res = false;
		if (lhs.first == rhs.first)
			res = lhs.second < rhs.second;
		else
			res = lhs.first <= rhs.first;
		return res;
	}
};

template <class T1, class T2>
class PairSecondThenFirstComp : public std::binary_function<std::pair<T1, T2>, std::pair<T1, T2>, bool> {
public:
	bool operator()(std::pair<T1, T2> const& lhs, std::pair<T1, T2> const& rhs) const {
		bool res = false;
		if (lhs.second == rhs.second)
			res = lhs.first <= rhs.first;
		else
			res = lhs.second <= rhs.second;
		return res;
	}
};

class comp : public std::binary_function<std::pair<int, int>, std::pair<int, int>, bool> {
public:
	bool operator()(std::pair<int, int> const& lhs, std::pair<int, int> const& rhs) const {
		bool res = false;
		if (lhs.first == rhs.first)
			res = lhs.second <= rhs.second;
		else
			res = lhs.first <= rhs.first;
		return res;
	}
};

/*using namespace std;
template <class T1, class T2>
class PairSecondComp;

template <class T1, class T2>
class PairFirstThenSecondComp;

template <class T1, class T2>
class PairSecondThenFirstComp;

class comp;*/

#endif /* UTILITY_H_ */
