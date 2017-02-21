/*
 * Time.h
 *
 *  Created on: Feb 8, 2014
 *      Author: Aaron
 */

#ifndef MyTIME_H_
#define MyTIME_H_

/*
#include <chrono>
#include <thread>
*/

class Time {

public:
	static double t;

public:

	/**
	 * TSTART = TIC saves the time to an output argument, TSTART.
	 * The numeric value of TSTART is only useful as an input
	 * argument for a subsequent call to TOC.
	 *
	 * @return time, in seconds, when TIC is called
	 */
	static double tic();

	/**
	 * Calculate the elapsed time, in seconds, since the most
	 * recent execution of the TIC command.
	 *
	 * @return elapsed time, in seconds, since the most recent
	 *         execution of the TIC command
	 */
	static double toc();

	/**
	 * TOC(TSTART) measures the time elapsed since the TIC command that
	 * generated TSTART.
	 *
	 * @return elapsed time, in seconds, since the TIC command that
	 *         generated TSTART
	 */
	static double toc(double TSTART);

	/**
	 * PAUSE(n) pauses for n seconds before continuing, where n can also be a
	 * fraction. The resolution of the clock is platform specific. Fractional
	 * pauses of 0.01 seconds should be supported on most platforms.
	 *
	 * @param n time, in seconds, to pause
	 */
	static void pause(double n);

};

/**
 * TSTART = TIC saves the time to an output argument, TSTART.
 * The numeric value of TSTART is only useful as an input
 * argument for a subsequent call to TOC.
 *
 * @return time, in seconds, when TIC is called
 */
double tic();

/**
 * Calculate the elapsed time, in seconds, since the most
 * recent execution of the TIC command.
 *
 * @return elapsed time, in seconds, since the most recent
 *         execution of the TIC command
 */
double toc();

/**
 * TOC(TSTART) measures the time elapsed since the TIC command that
 * generated TSTART.
 *
 * @return elapsed time, in seconds, since the TIC command that
 *         generated TSTART
 */
double toc(double TSTART);

#endif /* MyTIME_H_ */
