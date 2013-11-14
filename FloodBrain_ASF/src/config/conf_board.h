/**
 * \file
 *
 * \brief User board configuration template
 *
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

/* Pin configuration for FloodBrain
 *    PA0  ........ CH1
 *    PA1  ........ CH2
 *    PA2  ........ CH3
 *    PA3  ........ CH4
 *    PA4  ........ CH5
 *    PA5  ........ CH6
 *    PA6  ........ CH7
 *    PA7  ........ CH8
 *
 *    PC0  ........ STATUS LED
 *    PC1  ........ DATA LED
 *    PC2  ........ NRF IRQ
 *    PC3  ........ NRF CE
 *    PC4  ........ NRF SEL
 *    PC5  ........ NRF SCLK
 *    PC6  ........ MISO/RX
 *    PC7  ........ MOSI/TX
 *
 *    PD0  ........ CH9
 *    PD1  ........ CH10
 *    PD2  ........ CH11
 *    PD3  ........ CH12
 *    PD4  ........ CH13
 *    PD5  ........ CH14
 *    PD6  ........ CH15
 *    PD7  ........ CH16
 *
 *    PR0  ........ CH17
 *    PR1  ........ CH18
 *
 */

// port definitions
#define NUM_CHANNELS 18

#ifdef PWM_SINK
#define PWM_BITOP |=
#else
#define PWM_BITOP &= ~
#endif

#define CH1_SET (portlevelA PWM_BITOP (1 << PIN0_bp))
#define CH2_SET (portlevelA PWM_BITOP (1 << PIN1_bp))
#define CH3_SET (portlevelA PWM_BITOP (1 << PIN2_bp))
#define CH4_SET (portlevelA PWM_BITOP (1 << PIN3_bp))
#define CH5_SET (portlevelA PWM_BITOP (1 << PIN4_bp))
#define CH6_SET (portlevelA PWM_BITOP (1 << PIN5_bp))
#define CH7_SET (portlevelA PWM_BITOP (1 << PIN6_bp))
#define CH8_SET (portlevelA PWM_BITOP (1 << PIN7_bp))

#define CH9_SET (portlevelD PWM_BITOP (1 << PIN0_bp))
#define CH10_SET (portlevelD PWM_BITOP (1 << PIN1_bp))
#define CH11_SET (portlevelD PWM_BITOP (1 << PIN2_bp))
#define CH12_SET (portlevelD PWM_BITOP (1 << PIN3_bp))
#define CH13_SET (portlevelD PWM_BITOP (1 << PIN4_bp))
#define CH14_SET (portlevelD PWM_BITOP (1 << PIN5_bp))
#define CH15_SET (portlevelD PWM_BITOP (1 << PIN6_bp))
#define CH16_SET (portlevelD PWM_BITOP (1 << PIN7_bp))

#define CH17_SET (portlevelR PWM_BITOP (1 << PIN0_bp))
#define CH18_SET (portlevelR PWM_BITOP (1 << PIN1_bp))

#define PORTA_MASK 0xFFu
#define PORTD_MASK 0xFFu
#define PORTR_MASK (1 << PIN0_bp)|(1 << PIN1_bp)

#define LED_STATUS IOPORT_CREATE_PIN(PORTC, 0)
#define LED_DATA IOPORT_CREATE_PIN(PORTC, 1)

#define NRF_SEL IOPORT_CREATE_PIN(PORTC, 4)
#define NRF_CE IOPORT_CREATE_PIN(PORTC, 3)
#define NRF_IRQ IOPORT_CREATE_PIN(PORTC, 2)
#endif // CONF_BOARD_H
