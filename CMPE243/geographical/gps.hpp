/*
 * gps.hpp
 *
 *  Created on: 2015¦~11¤ë19¤é
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_GPS_HPP_
#define L5_APPLICATION_GPS_HPP_

#ifdef ECU_GEOGRAPHICAL

bool gps_init(void);
float* get_bearing_and_distance(void); //

#endif // ECU_GEOGRAPHICAL

#endif /* L5_APPLICATION_GPS_HPP_ */
