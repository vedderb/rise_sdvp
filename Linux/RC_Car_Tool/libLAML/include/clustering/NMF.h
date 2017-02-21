/*
 * NMF.h
 *
 *  Created on: Mar 2, 2014
 *      Author: Mingjie Qian
 */

#ifndef NMF_H_
#define NMF_H_

#include "L1NMF.h"

/***
 * A C++ implementation for NMF which solves the following
 * optimization problem:
 * <p>
 * min || X - G * F ||_F^2</br>
 * s.t. G >= 0, F >= 0
 * </p>
 *
 * @author Mingjie Qian
 * @version 1.0 Mar. 2nd, 2014
 */
class NMF : public L1NMF {

public:

	NMF(Options options);

	NMF(NMFOptions& NMFOptions);

	NMF();

};


#endif /* NMF_H_ */
