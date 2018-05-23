#ifndef DECAWAVE_DECA_RANGE_TABLES_H_
#define DECAWAVE_DECA_RANGE_TABLES_H_

#include "deca_param_types.h"

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: dwt_getrangebias()
 *
 * Description: This function is used to return the range bias correction need for TWR with DW1000 units.
 *
 * input parameters:
 * @param chan  - specifies the operating channel (e.g. 1, 2, 3, 4, 5, 6 or 7)
 * @param range - the calculated distance before correction
 * @param prf   - this is the PRF e.g. DWT_PRF_16M or DWT_PRF_64M
 *
 * output parameters
 *
 * returns correction needed in meters
 */
float dwt_getrangebias(uint8 chan, float range, uint8 prf);

#endif /* DECAWAVE_DECA_RANGE_TABLES_H_ */
