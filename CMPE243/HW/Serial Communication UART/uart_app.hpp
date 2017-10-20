/*
 * uart_app.hpp
 *
 *  Created on: Sep 14, 2017
 *      Author: YuYu
 */

#ifndef L5_APPLICATION_APP_UART_APP_HPP_
#define L5_APPLICATION_APP_UART_APP_HPP_

#include "uart2.hpp"
#include "uart3.hpp"
#include "singleton_template.hpp"  // Singleton Template
#include "headers.hpp"

class UART_APP : public SingletonTemplate<UART_APP>
{
public:
	void init_uart2(unsigned int baud_rate);
	void init_uart3(unsigned int baud_rate);
	void run();

	bool get_char(char *rx_buff);
	bool put_char(char* tx_buff);

	void LDC_init(void);
	void TPP_init(void);

	void LDC_10hz_task(void);
	void TPP_10hz_task(void);
private:

	UART_APP(){};

	Uart2 *u2;
	Uart3 *u3;

	bool uart2_used = false;
	bool uart3_used = false;
	bool message_available = false;

	char rx[100] = {'\0'};

	void init_uart(unsigned int baud_rate);
	friend class SingletonTemplate<UART_APP>;  ///< Friend class used for Singleton Template
};



#endif /* L5_APPLICATION_APP_UART_APP_HPP_ */
