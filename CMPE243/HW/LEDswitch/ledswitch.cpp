/*
 * ledswitch.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: YuYu
 */

#include "app/ledswitch.hpp"


LED_SWITCH::LED_SWITCH(LPC1758_GPIO_Type pin) : p_switch(new GPIO(pin))
{
	p_switch->setAsInput();
}
LED_SWITCH::~LED_SWITCH()
{
	p_switch->~GPIO();
}

bool LED_SWITCH::poll_pin(void)
{
	return p_switch->read();
}

LED_BOARD::LED_BOARD(LPC1758_GPIO_Type pin) : p_led(new GPIO(pin))
{
	p_led->setAsOutput();
}
LED_BOARD::~LED_BOARD()
{
	p_led->~GPIO();
}
void LED_BOARD::toggle(void)
{
	p_led->toggle();
}
void LED_BOARD::on(void)
{
	p_led->setHigh();
}
void LED_BOARD::off(void)
{
	p_led->setLow();
}


