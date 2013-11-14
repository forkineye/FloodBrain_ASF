/*
 * Project: FloodBrain_ASF
 * Copyright (c) 2013 Shelby Merrick 
 * http://www.forkineye.com
 * 
 *  This program is provided free for you to use in any way that you wish,
 *  subject to the laws and regulations where you are using it.  Due diligence
 *  is strongly suggested before using this code.  Please give credit where due.
 *
 *  The Author makes no warranty of any kind, express or implied, with regard
 *  to this program or the documentation contained in this document.  The
 *  Author shall not be liable in any event for incidental or consequential
 *  damages in connection with, or arising out of, the furnishing, performance
 *  or use of these programs.
 *
 * ---------------------------------------------------------------------------
 *
 * Credits:
 *  Phil Short for creation of the Renard Protocol.
 *  The great communities at doityourselfchristmas.com and diychristmas.org.
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include "config.h"
#include "renard.h"


// function prototypes
void init(void);
static void pwm_callback(void);
void procRenard(void);


// global vars
uint8_t compare[NUM_CHANNELS];
volatile uint8_t compbuff[NUM_CHANNELS];


// setup the board instead of board_init() as recommended by ASF.. because christmas lights, that's why.
void init (void) {
	static usart_serial_options_t usart_options = {
		.baudrate = USART_SERIAL_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	// initialize ASF stuff
	board_init();
	sysclk_init();
	ioport_init();
	pmic_init();
	pmic_set_scheduling(PMIC_SCH_FIXED_PRIORITY);
	
	// remap, enable TX, and configure USART on PORT C
	PORTC.REMAP |= PR_USART0_bm;
	PORTC.DIR |= (1 << PIN7_bp);
		
	sysclk_enable_module(SYSCLK_PORT_C, PR_USART0_bm);
	usart_init_rs232(USART_SERIAL, &usart_options);
	
	// setup timer for PWM
	tc45_enable(&TCC4);
	tc45_set_overflow_interrupt_callback(&TCC4, pwm_callback);
	tc45_set_wgm(&TCC4, TC45_WG_NORMAL);
	tc45_write_period(&TCC4, 256);
	tc45_set_overflow_interrupt_level(&TCC4, TC45_INT_LVL_MED);
		
	// enable all channels and turn off (high)
	ioport_set_port_dir(IOPORT_PORTA, PORTA_MASK, IOPORT_DIR_OUTPUT);
	ioport_set_port_dir(IOPORT_PORTD, PORTD_MASK, IOPORT_DIR_OUTPUT);
	ioport_set_port_dir(IOPORT_PORTR, PORTR_MASK, IOPORT_DIR_OUTPUT);
	ioport_set_port_level(IOPORT_PORTA, PORTA_MASK, 0xFF);
	ioport_set_port_level(IOPORT_PORTD, PORTD_MASK, 0xFF);
	ioport_set_port_level(IOPORT_PORTR, PORTR_MASK, 0xFF);
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		compare[i] = 0;
		compbuff[i] = 0;
	}
	
	// enable status LEDs and turn off
	ioport_set_pin_dir(LED_STATUS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(LED_DATA, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_STATUS, 1);
	ioport_set_pin_level(LED_DATA, 1);
	
	// enable interrupts and start timer for PWM
	cpu_irq_enable();
	tc45_write_clock_source(&TCC4, TC45_CLKSEL_DIV2_gc);	
}

// Software PWM routine from AVR136
static void pwm_callback(void) {
	static uint8_t portlevelA=PORTA_MASK, portlevelD=PORTD_MASK, portlevelR=PORTR_MASK;
	static uint8_t softcount=0xFF;
	
	// update outputs
	PORTA.OUT = portlevelA;
	PORTD.OUT = portlevelD;
	PORTR.OUT = portlevelR;
	// unrolled for speed -- process for NUM_CHANNELS
	if (++softcount == 0) {
		compare[0] = compbuff[0];
		compare[1] = compbuff[1];
		compare[2] = compbuff[2];
		compare[3] = compbuff[3];
		compare[4] = compbuff[4];
		compare[5] = compbuff[5];
		compare[6] = compbuff[6];
		compare[7] = compbuff[7];
		compare[8] = compbuff[8];
		compare[9] = compbuff[9];
		compare[10] = compbuff[10];
		compare[11] = compbuff[11];
		compare[12] = compbuff[12];
		compare[13] = compbuff[13];
		compare[14] = compbuff[14];
		compare[15] = compbuff[15];
		compare[16] = compbuff[16];
		compare[17] = compbuff[17];


		#ifdef PWM_SINK
		// set all pins low
		portlevelA &= ~PORTA_MASK;
		portlevelD &= ~PORTD_MASK;
		portlevelR &= ~PORTR_MASK;
		#else
		// set all pins high
		portlevelA |= PORTA_MASK;
		portlevelD |= PORTD_MASK;
		portlevelR |= PORTR_MASK;
		#endif
	}
	
	if (compare[0] == softcount) CH1_SET;
	if (compare[1] == softcount) CH2_SET;
	if (compare[2] == softcount) CH3_SET;
	if (compare[3] == softcount) CH4_SET;
	if (compare[4] == softcount) CH5_SET;
	if (compare[5] == softcount) CH6_SET;
	if (compare[6] == softcount) CH7_SET;
	if (compare[7] == softcount) CH8_SET;
	if (compare[8] == softcount) CH9_SET;
	if (compare[9] == softcount) CH10_SET;
	if (compare[10] == softcount) CH11_SET;
	if (compare[11] == softcount) CH12_SET;
	if (compare[12] == softcount) CH13_SET;
	if (compare[13] == softcount) CH14_SET;
	if (compare[14] == softcount) CH15_SET;
	if (compare[15] == softcount) CH16_SET;
	if (compare[16] == softcount) CH17_SET;
	if (compare[17] == softcount) CH18_SET;
	
	tc45_clear_overflow(&TCC4);
}

void procRenard (void) {
	uint8_t rxByte;
	uint8_t channel = 0;
	
	// process bytes for each channel
	while (channel < NUM_CHANNELS) {
		usart_serial_getchar(USART_SERIAL, &rxByte);
		// map values for escaped bytes
		switch (rxByte) {
			case RENARD_PAD:
				break;
			case RENARD_ESCAPE:
				usart_serial_getchar(USART_SERIAL, &rxByte);
				switch (rxByte) {
					case RENARD_ESC_7D:
						compbuff[channel] = 0x7D;
						break;
					case RENARD_ESC_7E:
						compbuff[channel] = 0x7E;
						break;
					case RENARD_ESC_7F:
						compbuff[channel] = 0x7F;
						break;
				}
				channel++;
				break;
			default:
				compbuff[channel] = rxByte;
				channel++;
		}
	}
}

int main(void) {
	uint8_t rxByte;

	init();

	// main loop
	while (1) {
		// poll and retransmit while waiting for the sync byte
		while ((rxByte = usart_getchar(USART_SERIAL)) != RENARD_SYNC) {
			usart_serial_putchar(USART_SERIAL, rxByte);
		}

		// retransmit the sync byte
		usart_serial_putchar(USART_SERIAL, RENARD_SYNC);
			
		// evaluate command byte
		usart_serial_getchar(USART_SERIAL, &rxByte);
		if (rxByte == RENARD_ADDR) {			// process renard packet for this device
			procRenard();
		} else if (rxByte > RENARD_ADDR) {	// decrement command/address byte and transmit
			rxByte--;
			usart_serial_putchar(USART_SERIAL, rxByte);
		} else {								// unsupported / reserved.  retransmit
			usart_serial_putchar(USART_SERIAL, rxByte);
		}
	}
}