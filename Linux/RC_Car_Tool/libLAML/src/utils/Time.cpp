/*
 * Time.cpp
 *
 *  Created on: Feb 8, 2014
 *      Author: Aaron
 */

#include "MyTime.h"
#include <ctime>

double Time::t = 0;

double Time::tic() {
	t = (double) clock() / CLOCKS_PER_SEC;
	return t;
}

double Time::toc() {
	return (double) clock() / CLOCKS_PER_SEC - t;
}

double Time::toc(double TSTART) {
	return (double) clock() / CLOCKS_PER_SEC - TSTART;
}

double tic() {
	Time::t = (double) clock() / CLOCKS_PER_SEC;
	return Time::t;
}

double toc() {
	return (double) clock() / CLOCKS_PER_SEC - Time::t;
}

double toc(double TSTART) {
	return (double) clock() / CLOCKS_PER_SEC - TSTART;
}

/*void Time::pause(double n) {
	std::this_thread::sleep_for(std::chrono::milliseconds((long)n * 1000));
}*/


