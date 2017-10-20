/*
===============================================================================
 Name        : Lab1.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/
#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#define DEBUG 0
#define POLL_RX 0

#define PAYLOAD_SIZE	88
#define SYNC_SIZE 		256
#define RX_BUFFER_SIZE  1024
#define PAYLOAD_BUFFER_SIZE 256


// firmware
void start_timer(void);
bool enable_timer(uint32_t ms_delay);
void delayMs(uint32_t ms_delay);
void enable_interrupt(void);
void disable_interrupt(void);

// LISA
void decode_payload(void);
void set_confidence(void);
bool match_and_get_payload(void); // LISA algorithm
int sync(int p);

// scrambler/descrambler
void set_scramble_scramble_nth_order(void);
void scramble(int *payload);
void descramble(int *payload);


int syncdata[SYNC_SIZE] = 	{
								1,0,1,0,0,0,0,0,	//A prefix
								1,0,1,0,0,0,0,1,	//0xA1
								1,0,1,0,0,0,1,0,	//0xA2
								1,0,1,0,0,0,1,1,	//0xA3
								1,0,1,0,0,1,0,0,	//0xA4
								1,0,1,0,0,1,0,1,	//0xA5
								1,0,1,0,0,1,1,0,	//0xA6
								1,0,1,0,0,1,1,1,	//0xA7
								1,0,1,0,1,0,0,0,	//0xA8
								1,0,1,0,1,0,0,1,	//0xA9
								1,0,1,0,1,0,1,0,	//0xAA
								1,0,1,0,1,0,1,1,	//0xAB
								1,0,1,0,1,1,0,0,	//0xAC
								1,0,1,0,1,1,0,1,	//0xAD
								1,0,1,0,1,1,1,0,	//0xAE
								1,0,1,0,1,1,1,1,	//0xAF
								0,1,0,1,0,0,0,0,	//5 prefix
								0,1,0,1,0,0,0,1,	//0x51
								0,1,0,1,0,0,1,0,	//0x52
								0,1,0,1,0,0,1,1,	//0x53
								0,1,0,1,0,1,0,0,	//0x54
								0,1,0,1,0,1,0,1,	//0x55
								0,1,0,1,0,1,1,0,	//0x56
								0,1,0,1,0,1,1,1,	//0x57
								0,1,0,1,1,0,0,0,	//0x58
								0,1,0,1,1,0,0,1,	//0x59
								0,1,0,1,1,0,1,0,	//0x5A
								0,1,0,1,1,0,1,1,	//0x5B
								0,1,0,1,1,1,0,0,	//0x5C
								0,1,0,1,1,1,0,1,	//0x5D
								0,1,0,1,1,1,1,0,	//0x5E
								0,1,0,1,1,1,1,1		//0x5F
	};

int payload[PAYLOAD_SIZE] =
{
	0,1,1,0,1,0,0,0, 		// h
	0,1,1,0,0,1,0,1,		// e
	0,1,1,0,1,1,0,0,		// l
	0,1,1,0,1,1,0,0,		// l
	0,1,1,0,1,1,1,1,		// o
	0,0,1,0,0,0,0,0,		// _
	0,1,1,1,0,1,1,1,		// w
	0,1,1,0,1,1,1,1,		// o
	0,1,1,1,0,0,1,0,		// r
	0,1,1,0,1,1,0,0,		// l
	0,1,1,0,0,1,0,0,		// d
};

int rx_buffer[RX_BUFFER_SIZE] = {0};
int payload_buffer[PAYLOAD_BUFFER_SIZE] = {0};
char decoded_payload[PAYLOAD_BUFFER_SIZE] = {'\0'};

static uint8_t confidence = 0;
static int jump =0;
static int prev_jump =0;

volatile bool intr_triggered = false;

volatile bool rx_busy = false;
static int rx_buffer_index = 0;

// scrambler
static uint8_t delay_nth_first[PAYLOAD_SIZE] = {0};
static uint8_t delay_nth_second[PAYLOAD_SIZE] = {0};
static int scramble_out[PAYLOAD_SIZE] = {0};
static int descramble_out[PAYLOAD_SIZE] = {0};
static int order = 0;

bool init_pins(void)
{
	LPC_PINCON->PINSEL4 &= ~(3 << 12); // pin mux set to GPIO Port 2.6
	LPC_GPIO2->FIODIR |= (1 << 6);  // set GPIO Port 2.6 external LED as output


	LPC_PINCON->PINSEL4 &= ~(3 << 14); // pin mux set to GPIO Port 2.7
	LPC_GPIO2->FIODIR &= ~(1 << 7); // set GPIO Port 2.7 on external switch as input

	LPC_PINCON->PINSEL0 &= ~(3 << 4); // pin mux set to GPIO Port 0.2
	LPC_GPIO0->FIODIR |= (1 << 2); // set GPIO Port 0.2 as TX

	LPC_PINCON->PINSEL0 &= ~(3 << 6); // pin mux set to GPIO Port 0.3
	LPC_GPIO0->FIODIR &= ~(1 << 3); // set GPIO Port 0.3 RX

//	enable_interrupt();
	disable_interrupt();

	return true;
}

void EINT3_IRQHandler(void)
{

	if((LPC_GPIOINT->IO0IntStatR  & (1 << 3)) || LPC_GPIOINT->IO0IntStatF & (1 << 3))
	{
		intr_triggered = true;

	}
	// Clear the status register of interrupt on P0.3
	LPC_GPIOINT->IO0IntClr |= (1 << 3);
}

void enable_interrupt(void)
{
	LPC_GPIOINT->IO0IntEnR |= (1 << 3); // rising edge of P0.3
	LPC_GPIOINT->IO0IntEnF |= (1 << 3); // falling edge of P0.3
}

void disable_interrupt(void)
{
	LPC_GPIOINT->IO0IntEnR = 0x0; // disable rising edge
	LPC_GPIOINT->IO0IntEnF = 0x0; // disable falling edge
}

bool init_timer(void)
{
	LPC_SC->PCONP |= (1 << 1); // Power up timer0

//	LPC_SC->PCLKSEL0 &= ~(3 << 2);
//	LPC_SC->PCLKSEL0 |= (1 << 2); // PCLK_peripheral  = CCLK


	LPC_TIM0->PR = (SystemCoreClock / 4000) -1;
	LPC_TIM0->TCR =  (0 << 0);  // Initially disable counting

	return true;
}

bool tx_data(int *sync, int *data)
{
	int i;
	int message_to_send[SYNC_SIZE + PAYLOAD_SIZE] = {0};

	bool success = false;
	int success_count = 0;

	scramble(data);

	// Message to Send out of TX
	for(i=0; i < SYNC_SIZE+ PAYLOAD_SIZE; i++)
	{
		message_to_send[i] = sync[i];

	}
	for(i = SYNC_SIZE; i < SYNC_SIZE + PAYLOAD_SIZE; i++)
	{
		message_to_send[i] = scramble_out[i-SYNC_SIZE];
	}

	if(LPC_GPIO2->FIOPIN & (1 << 7))
	{
		LPC_GPIO2->FIOSET = (1 << 6);

		for(int i=0; i < SYNC_SIZE + PAYLOAD_SIZE; i++)
		{
			if(message_to_send[i] == 1)
			{
				LPC_GPIO0->FIOSET = (1 << 2);
				success_count++;
			}
			else if(message_to_send[i] == 0)
			{
				LPC_GPIO0->FIOCLR = (1 << 2);
				success_count++;
			}
			delayMs(10);
	}

		if (success_count == SYNC_SIZE+PAYLOAD_SIZE)
		{
			success = true;
//			enable_interrupt();
		}
	}
	else
	{
		LPC_GPIO2->FIOCLR = (1 << 6);
	}
	return success;
}

// polling rx with delay
bool rx_data(int *buffer)
{
	bool success = false;
	enable_interrupt();
	uint32_t i=1;

	// edge detected
	if(intr_triggered)
	{
		puts("disabling");
		disable_interrupt();
		intr_triggered = false;
		buffer[0] = 1; // edge detected? counts as 1 bit?
		puts("1");
		// poll rest of data after rising edge
		while(i < SYNC_SIZE+PAYLOAD_SIZE)
		{
			if(LPC_GPIO0->FIOPIN & (1 << 3))
			{
				buffer[i % RX_BUFFER_SIZE] = 1;
				i++;
			}
			else
			{
				buffer[i % RX_BUFFER_SIZE] = 0;
				i++;
			}
			delayMs(10);
		}
//		puts("");
//		printf("i: %d\r\n", i);

		success = match_and_get_payload();

		if(success)
		{
			decode_payload();
		}
	}

	return success;
}

// interrupt driven based on timer
bool rx_data_ir(int *buffer)
{
	bool success = false;
	enable_interrupt();

	// edge detected
	if(intr_triggered)
	{
		puts("disabling");
		disable_interrupt();
		intr_triggered = false;
		buffer[0] = 1; // edge detected? counts as 1 bit?
		start_timer();
		rx_busy = true;
		printf("busy: %d\r\n", rx_busy);
		while(rx_busy)
		{
			// wait for it to finish, let interrupt do the job
		}
		LPC_TIM0->TCR =  (0 << 0); // stop timer
//		puts("");
//		printf("i: %d\r\n", rx_buffer_index);
		rx_buffer_index = 0;
		success = match_and_get_payload();

		if(success)
		{
			decode_payload();
		}
	}

	return success;
}

void set_confidence(void)
{
	printf("Please enter a confidence level between 1 and 32\n");
	printf("A 1 would lead to a 3%% confidence and 32 would lead to a 100%% confidence\n");
	scanf("%d", &confidence);
}

int sync(int p)
{
	// look only for 0x5? or 0xA? pattern
	if((rx_buffer[p] == rx_buffer[p + 2] && rx_buffer[p + 1] == rx_buffer[p + 3]) &&
			(rx_buffer[p] != rx_buffer[p+1] && rx_buffer[p] != rx_buffer[p+3] &&
			rx_buffer[p+2] != rx_buffer[p+1] && rx_buffer[p+2] != rx_buffer[p+3]))
	{
		jump = 8 * rx_buffer[p + 4] + 4 * rx_buffer[p + 5] +
		2 * rx_buffer[p + 6] + 1 * rx_buffer[p + 7];
#if DEBUG
			printf("jump: %d\r\n", jump);
#endif
		// Any sequence of 0x5? or 0xA?
		if((jump == (prev_jump + 1)) || (jump == (prev_jump - 15)) || (jump == 0))
		{
			prev_jump = jump;
#if DEBUG
				printf("########prev jump: %d\r\n", prev_jump);
#endif
			return 1;
		}
		else
			return 0;
	}
	else
		return 0;
}

bool match_and_get_payload(void)
{
	bool success = false;
	int position = 0;
	int limit = 768-(8*(confidence));

	int i = 0;
	for(i = 0; i < limit; i++)
	{
		int match = 0, locate = 0;
		position = i;
#if DEBUG
		printf("outer position: %d\r\n",position);
#endif
		while(match != confidence && position < limit)
		{
			locate = sync(position);
			if(locate == 0)
			{
				break;
			}
			match = match + locate;
			position = position + 8;
#if DEBUG
			printf("inner position: %d\r\n", position);
#endif
		}
		if(match == confidence)
		{
			success = true;
			break;
		}

	}

	if(success)
	{
		position = position - 8;
		int location = rx_buffer[position + 4] * 8 + rx_buffer[position + 5] * 4 +
				rx_buffer[position + 6] * 2 + rx_buffer[position + 7];
		if(rx_buffer[position] == 0)
		{
			printf("You are at location 0x5%x\r\n", location);
			location = 128 - (8 * location);
		}
		if(rx_buffer[position] == 1)
		{
			printf("You are at location 0xA%x\r\n", location);
			location = 256 - (8 * location);
		}

		int index = position + location;
		printf("location: %d  position: %d  index: %d\r\n", location, position, index);

		i=0;
		for(int p = index; p < index + PAYLOAD_SIZE; p++)
		{
			payload_buffer[i] = rx_buffer[p];
			i++;
		}
	}
	else
	{
		puts("No match found");
	}

	return success;
}

void decode_payload(void)
{
	uint8_t bits = 0;
	int i;

	descramble(payload_buffer);

	for(i = 0; i < SYNC_SIZE+PAYLOAD_SIZE; i++) //PAYLOAD_SIZE
	{
		if(i != 0 && i%8 == 0)
		{
			int index = i/8 - 1;
			decoded_payload[index] = ((char)bits);
			bits = 0;
		}
		else
		{
			bits = bits <<= 1;
			bits = bits | descramble_out[i]; // payload_buffer
		}
	}

	printf("payload: %s\r\n", decoded_payload);

	puts("");
}

void TIMER0_IRQHandler(void)
{
	if ( LPC_TIM0->IR & (0x1<<0) ) // Check MR0 interrupt flag
	{
	LPC_TIM0->IR = 0xff;
	}
//	printf("tick\r\n");
//	LPC_GPIO0->FIOPIN ^= (1 << 2);

	if(rx_busy)
	{
		if(LPC_GPIO0->FIOPIN & (1 << 3))
		{
			//				printf("1");
			rx_buffer[rx_buffer_index % RX_BUFFER_SIZE] = 1;
			rx_buffer_index++;
		}
		else
		{
			//				printf("0");
			rx_buffer[rx_buffer_index % RX_BUFFER_SIZE] = 0;
			rx_buffer_index++;
		}

		if(rx_buffer_index == SYNC_SIZE + PAYLOAD_SIZE)
		{
			rx_busy = false;
		}
	}
}

bool enable_timer(uint32_t ms_delay)
{
//	LPC_TIM0->MCR |= (1<<0) | (1<<1); // enable interrupt, reset tick on MR0
	LPC_TIM0->TCR = (1 << 1); // reset timer
	LPC_TIM0->PR  = 0x00;	  // set prescaler to zero
	LPC_TIM0->MR0 = (ms_delay * (SystemCoreClock/4000)) -1;
	LPC_TIM0->IR  = 0xff;     // reset all interrrupts
//	LPC_TIM0->TCR = 1;		  // start timer

	return true;
}

void start_timer(void)
{
	LPC_TIM0->MCR |= (1<<0) | (1<<1) | (0<<2); // enable interrupt, reset tick on MR0, do not stop on MR0
	LPC_TIM0->TCR = 1;		  // start timer
}

void delayMs(uint32_t ms_delay)
{
	LPC_TIM0->TCR = (1 << 1); // reset timer
	LPC_TIM0->PR  = 0x00;	  // set prescaler to zero
	LPC_TIM0->MR0 = (ms_delay * (SystemCoreClock/4000)) -1;
	LPC_TIM0->IR  = 0xff;     // reset all interrrupts
	LPC_TIM0->MCR = (1 << 2); // stop timer on match
	LPC_TIM0->TCR = 1;		  // start timer

	while(LPC_TIM0->TCR & 0x01); // wait until delay time has elapsed

}

void set_scramble_scramble_nth_order(void)
{
	printf("Enter what nth order to scramble/descramble payload\r\n");
	scanf("%u8", &order);
}

void scramble(int *payload)
{
	int i=0;
	uint8_t scramble_buffer = 0;
	int payload_index = 0;
	int nth_delay_bit = order/2; // order will be the 2nd delay used
	uint16_t delay_buffer = 0;

	bool out = 0;
	int scramble_out_index = 0;
	int delay_nth_first_index = 0;
	int delay_nth_second_index = 0;

	memset(delay_nth_first, 0, sizeof(delay_nth_first)); // reset delay arrays
	memset(delay_nth_second, 0, sizeof(delay_nth_second));
	puts("scramble");
	while(payload_index < PAYLOAD_SIZE)
	{
		scramble_buffer = 0; // reset buffer to 0
		for(i=7; i >= 0; i--) // copy payload 8 bits at a time into buffer
		{
			scramble_buffer |= payload[payload_index++] << i;
		}
		for(i=0; i < 8; i++)
		{
			delay_nth_first[delay_nth_first_index] = ((delay_buffer & (1 << nth_delay_bit)) >> nth_delay_bit);
			delay_nth_second[delay_nth_second_index] = ((delay_buffer & (1 << (order-1))) >> (order-1));
			out = delay_nth_first[delay_nth_first_index++] ^ delay_nth_second[delay_nth_second_index++]; // delays
			out ^= ((scramble_buffer >> (7-i)) & 0x1); // xor with input
			scramble_out[scramble_out_index++] = out;
			delay_buffer = (delay_buffer << 1) | (out << 0);
		}
	}
	printf("index: %d  scrambler_buffer: 0x%x\r\n",payload_index, scramble_buffer);
	printf("first: %d second: %d scramble_index: %d\r\n", delay_nth_first_index, delay_nth_second_index, scramble_out_index);
	printf("delay_buffer: %d\r\n", delay_buffer);
	printf("pay: ");
	for(i=0; i < PAYLOAD_SIZE; i++)
	{
		printf("%d",payload[i]);
	}
	puts("\r");
	printf("D5t: ");
	for(i=0; i < PAYLOAD_SIZE; i++)
	{
		printf("%d",delay_nth_second[i]);
	}
	puts("\r");
	printf("D3t: ");
	for(i=0; i < PAYLOAD_SIZE; i++)
	{
		printf("%d",delay_nth_first[i]);
	}
	puts("\r");
	printf("out: ");
	for(i=0; i < PAYLOAD_SIZE; i++)
	{
		printf("%d",scramble_out[i]);
	}
	puts("\r");

}

void descramble(int *payload)
{
	int i=0;
	uint8_t descramble_buffer = 0;
	int payload_index = 0;
	int nth_delay_bit = order/2; // order will be the 2nd delay used
	uint16_t delay_buffer = 0;

	bool out = 0;
	int descramble_out_index = 0;
	int delay_nth_first_index = 0;
	int delay_nth_second_index = 0;

	memset(delay_nth_first, 0, sizeof(delay_nth_first)); // reset delay arrays
	memset(delay_nth_second, 0, sizeof(delay_nth_second));
	puts("descramble");
	while(payload_index < PAYLOAD_SIZE)
	{
		descramble_buffer = 0; //reset 8-bit buffer
		for(i=7; i >=0; i--)
		{
			descramble_buffer |= payload[payload_index++] << i;
		}
		for(i=0; i < 8; i++)
		{
			delay_nth_first[delay_nth_first_index] = ((delay_buffer & (1 << nth_delay_bit)) >> nth_delay_bit);
			delay_nth_second[delay_nth_second_index] = ((delay_buffer & (1 << (order-1))) >> (order-1));
			out = delay_nth_first[delay_nth_first_index++] ^ delay_nth_second[delay_nth_second_index++]; // delays
			out ^= ((descramble_buffer >> (7-i)) & 0x1); // xor with input
			descramble_out[descramble_out_index++] = out;
			delay_buffer = (delay_buffer << 1) | (((descramble_buffer >> (7-i)) & 0x1) << 0);
		}
	}
	printf("index: %d  scrambler_buffer: 0x%x\r\n",payload_index, descramble_buffer);
	printf("first: %d second: %d scramble_index: %d\r\n", delay_nth_first_index, delay_nth_second_index, descramble_out_index);
	printf("delay_buffer: %d\r\n", delay_buffer);
	printf("pay: ");
	for(i=0; i < PAYLOAD_SIZE; i++)
	{
		printf("%d",payload[i]);
	}
	puts("\r");
	printf("D5t: ");
	for(i=0; i < PAYLOAD_SIZE; i++)
	{
		printf("%d",delay_nth_second[i]);
	}
	puts("\r");
	printf("D3t: ");
	for(i=0; i < PAYLOAD_SIZE; i++)
	{
		printf("%d",delay_nth_first[i]);
	}
	puts("\r");
	printf("out: ");
	for(i=0; i < PAYLOAD_SIZE; i++)
	{
		printf("%d",descramble_out[i]);
	}
	puts("\r");
}

int main(void) {

    int option =0;
    bool success;
    bool success1;
	printf("Hello World\n");
    init_pins();
    init_timer();
#if !(POLL_RX)
    enable_timer(10);
#endif
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);

    set_confidence();

    while(1)
    {
    	printf("Option1: send data   Option2: receive data\r\n");
    	scanf( "%d", &option);
    	switch(option)
    	{
    	case 1:
    		success = tx_data(syncdata, payload);
    	    printf("success: %d\r\n", success);
    	    break;
    	case 2:
#if POLL_RX
    	    success1 = rx_data(rx_buffer);
    	    printf("success1: %d\r\n", success1);
#else
        	success1 = rx_data_ir(rx_buffer);
        	printf("success1: %d\r\n", success1);
#endif
    	    break;
    	case 3:
    		set_scramble_scramble_nth_order();
    		scramble(payload);
    		descramble(scramble_out);
    		break;
    	default:
    		success = tx_data(syncdata, payload);
    		printf("success: %d\r\n", success);
    		break;
    	}
//    	delayMs(1000);

//    	LPC_GPIO0->FIOPIN ^= (1 << 2);
//    	delayMs(100);
    }
    return 0 ;
}
