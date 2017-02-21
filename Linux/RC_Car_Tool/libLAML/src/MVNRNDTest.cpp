/*
 * MVNRNDTest.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#include "Utility.h"
#include "Printer.h"
#include "Options.h"
#include "Distribution.h"
#include "Matlab.h"
#include <cmath>

int main(void) {

	int n = 10;
	int d = 2;
	Matrix& t = rand(d);
	Matrix& SIGMA = plus(t.mtimes(t.transpose()), times(diag(rand(d, 1)), eye(d)));
	double theta = rand(1).getEntry(0, 0) * PI;
	double PData[2][2] = {{cos(theta), sin(theta)}, {sin(theta), cos(theta)}};
	Matrix& P = DenseMatrix::createDenseMatrix(PData, 2, 2);
	// SIGMA = new DenseMatrix(new double[][]{{10, 0}, {0, 0}});
	SIGMA = P.mtimes(SIGMA).mtimes(P.transpose());
	Matrix& MU = times(3, rand(1, d));

	/*saveMatrix(MU, "MU");
			saveMatrix(SIGMA, "SIGMA");*/
	/*MU = loadMatrix("MU");
			SIGMA = loadMatrix("SIGMA");*/

	Matrix& X = mvnrnd(MU, SIGMA, n);
	disp(X);

	return EXIT_SUCCESS;

}
