/*
 * renard.h
 *
 * Created: 9/26/2013 10:43:54 AM
 *  Author: sporadic
 */ 


#ifndef RENARD_H_
#define RENARD_H_

// Special Renard bytes
#define RENARD_PAD 0x7D		// pad byte to be discarded
#define	RENARD_SYNC	0x7E	// sync byte to reset state machine
#define RENARD_ESCAPE 0X7F	// escape byte
#define	RENARD_ADDR 0x80	// command / address byte. 0x80 designates packet for this device
#define RENARD_ESC_7D 0x2F	// encoded value for data byte of 7D
#define RENARD_ESC_7E 0x30	// encoded value for data byte of 7E
#define RENARD_ESC_7F 0x31	// encoded value for data byte of 7F

// Renard state machine flags
typedef enum {
	RENSTATE_SYNC,
	RENSTATE_ESCAPE,
	RENSTATE_ADDR
} renstate_t;

#endif /* RENARD_H_ */