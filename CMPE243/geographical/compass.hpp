/*
 * compass.hpp
 *
 *  Created on: 2015/10/15
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_COMPASS_HPP_
#define L5_APPLICATION_COMPASS_HPP_


/*
https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
Library for HMC5883L
*/

#ifdef ECU_GEOGRAPHICAL 

typedef void(*compass_data_ready_cb)(void);

bool compass_init(compass_data_ready_cb cb);
float get_heading(void);

#endif // ECU_GEOGRAPHICAL
#endif /* L5_APPLICATION_COMPASS_HPP_ */
