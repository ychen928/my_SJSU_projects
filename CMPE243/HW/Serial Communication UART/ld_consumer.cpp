#include "app/ld_consumer.hpp"
#include "io.hpp"
#include "uart2.hpp"
#include <stdio.h>

#define UART_BAUD 9600
#define LD_VALUE_LIM = 100U

typedef struct {
    Uart2 *uart_ref;
} ldcvars_S;

ldcvars_S ldcsvars;


/*
 * Public functions
 */

void LDC_init(void)
{
    ldcsvars.uart_ref = &Uart2::getInstance();
    ldcsvars.uart_ref->init(9600);  // Using default Rx/Tx queue size
}


void LDC_10hz_task(void)
{
    char rx;
    if (ldcsvars.uart_ref->getChar(&rx, 0U)) {
        if ((uint8_t)rx < 100U) {
            LD.setNumber(rx);
            printf("rx%d\r\n", rx);
        }
    }
}
