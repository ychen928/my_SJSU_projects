/*
 * ledswitch.hpp
 *
 *  Created on: Sep 7, 2017
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_APP_LEDSWITCH_HPP_
#define L5_APPLICATION_APP_LEDSWITCH_HPP_

#include "gpio.hpp"


class LED_SWITCH
{
public:
	LED_SWITCH(LPC1758_GPIO_Type pin);
	~LED_SWITCH();

	bool poll_pin(void);
private:
	GPIO *p_switch;
};

class LED_BOARD
{
public:
	LED_BOARD(LPC1758_GPIO_Type pin);
	~LED_BOARD();

	void toggle(void);
	void on(void);
	void off(void);
private:
	GPIO *p_led;
};
#endif /* L5_APPLICATION_APP_LEDSWITCH_HPP_ */
