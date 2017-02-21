/*
 * LASSOTest.cpp
 *
 *  Created on: Mar 3, 2014
 *      Author: Mingjie Qian
 */

#include "Utility.h"
#include "Printer.h"
#include "Options.h"
#include "Regression.h"
#include "LASSO.h"
#include "MyTime.h"

int main(void) {

	double data[3][4] = {
			{1, 2, 3, 2},
			{4, 2, 3, 6},
			{5, 1, 4, 1}
			};

	double depVars[3][2] = {
			{3, 2},
			{2, 3},
			{1, 4}};

	Options& options = *new Options();
	options.maxIter = 600;
	options.lambda = 0.1;
	options.verbose = !true;
	options.calc_OV = !true;
	options.epsilon = 1e-5;

	Regression& lasso = *new LASSO(options);
	lasso.feedData<4>(data, 3, 4);
	lasso.feedDependentVariables<2>(depVars, 3, 2);

	tic();
	lasso.train();
	fprintf("Elapsed time: %.3f seconds\n\n", toc());

	fprintf("Projection matrix:\n");
	display(*lasso.W);

	Matrix& Yt = lasso.predict<4>(data, 3, 4);
	fprintf("Predicted dependent variables:\n");
	display(Yt);

	return EXIT_SUCCESS;

}


