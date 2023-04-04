/*
 * fastSin.h
 *
 *  Created on: Mar 29, 2023
 *      Author: LGW
 */

#ifndef USER_DEVICES_INC_FASTSIN_H_
#define USER_DEVICES_INC_FASTSIN_H_


#define fast_constrain(x, low, high)    ((x)<(low)?(low):((x) >(high)?(high):(x)))

/*!
    \brief      fast calculation of cosine
    \param[in]  x: angle to be calculated
    \retval     cosine value of angle theta
*/
#define fast_cos(x)                     fast_sin(1.5707963f - x);

extern float fast_sin(float theta);



#endif /* USER_DEVICES_INC_FASTSIN_H_ */
