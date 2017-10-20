#include "app/temp_producer.hpp"
#include "io.hpp"
#include "uart3.hpp"
#include <stdio.h>

#define UART_BAUD 9600
#define TX_TIMER_LIM (100U)  // (1/10hz) * 1 second

typedef struct {
    Uart3 *uart_ref;
    uint8_t tx_timer;
} tppvars_S;

tppvars_S tppvars;


/*
 * Public functions
 */

void TPP_init(void)
{
    tppvars.uart_ref = &Uart3::getInstance();
    tppvars.uart_ref->init(9600);  // Using default Rx/Tx queue size
    tppvars.tx_timer = 0U;
}


void TPP_10hz_task(void)
{
    if (++tppvars.tx_timer > TX_TIMER_LIM) {
        tppvars.tx_timer = 0U;
        uint8_t temp_value = TS.getCelsius();
        (void)tppvars.uart_ref->putChar((char)temp_value, 0U);
        printf("%d\r\n", temp_value);
    }
}
