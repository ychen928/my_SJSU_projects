/*
 * uart_app.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: YuYu
 */

#include "app/uart_app.hpp"
#include "io.hpp"
#include <stdio.h>

#define UART_BAUD 9600
#define TX_TIMER_LIM (100U)  // (1/10hz) * 1 second
#define LD_VALUE_LIM = 100U

// producer
typedef struct {
    Uart3 *uart_ref;
    uint8_t tx_timer;
} tppvars_S;

tppvars_S tppvars;

// consumer
typedef struct {
    Uart2 *uart_ref;
} ldcvars_S;

ldcvars_S ldcsvars;


void UART_APP::init_uart2(unsigned int baud_rate)
{
	uart2_used = true;
	init_uart(baud_rate);

}
void UART_APP::init_uart3(unsigned int baud_rate)
{
	uart3_used = true;
	init_uart(baud_rate);
}

void UART_APP::init_uart(unsigned int baud_rate)
{
	if(uart2_used)
	{
		u2 = &(Uart2::getInstance());
		u2->init(baud_rate);
		ldcsvars.uart_ref = u2;
	}

	if(uart3_used)
	{
		u3 = &(Uart3::getInstance());
		u3->init(baud_rate);
		tppvars.uart_ref = u3;
	}
}

void UART_APP::LDC_init(void)
{
//    ldcsvars.uart_ref = &Uart2::getInstance();
//    ldcsvars.uart_ref->init(9600);  // Using default Rx/Tx queue size
	init_uart2(UART_BAUD);
}

void UART_APP::TPP_init(void)
{
//    tppvars.uart_ref = &Uart3::getInstance();
//    tppvars.uart_ref->init(9600);  // Using default Rx/Tx queue size
	init_uart3(UART_BAUD);
    tppvars.tx_timer = 0U;
}

void UART_APP::run()
{
	if(SW.getSwitch(1))
	{
		puts("Sending from switch 1");
		put_char("hello\r\n");
	}
	if(SW.getSwitch(2))
	{
		puts("Sending from switch 2");
		put_char("cmpe243\r\n");
	}
	if(SW.getSwitch(3))
	{
		puts("Sending from switch 3");
		put_char("UART lab\r\n");
	}
	if(SW.getSwitch(4))
	{
		puts("Sending from switch 4");
		put_char("meow meow\r\n");
	}

#if UART2
	LDC_10hz_task();
#endif

	message_available = get_char(rx);
	if(message_available)
	{
		printf("%s", rx);
	}
#if UART3
	TPP_10hz_task();
#endif

}

bool UART_APP::get_char(char *rx_buff)
{
	bool char_available = false;

	if(uart2_used)
	{
		char_available = u2->getChar(rx_buff, 0);

		if(char_available)
		{
			return char_available;
		}
	}

	if(uart3_used)
	{
		char_available = u3->getChar(rx_buff, 0);

		if(char_available)
		{
			return char_available;
		}
	}
	return char_available;
}
bool UART_APP::put_char(char *tx_buff)
{
	bool success = false;
	if(uart2_used)
	{
		success = u2->put(tx_buff);

		if(success)
		{
			return success;
		}
	}

	if(uart3_used)
	{
		success = u3->put(tx_buff);

		if(success)
		{
			return success;
		}
	}

	return success;
}

void UART_APP::LDC_10hz_task(void)
{
    char rxx;
    if (ldcsvars.uart_ref->getChar(&rxx, 0U)) {
        if ((uint8_t)rxx < 100U) {
            LD.setNumber(rxx);
        }
    }
}

void UART_APP::TPP_10hz_task(void)
{
    if (++tppvars.tx_timer > TX_TIMER_LIM) {
        tppvars.tx_timer = 0U;
        uint8_t temp_value = TS.getCelsius();
        (void)tppvars.uart_ref->putChar((char)temp_value, 0U);
    }
}
