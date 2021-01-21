/*
 * CPPFile1.cpp
 *
 * Created: 8/10/2020 17:53:08
 *  Author: tellSlater
 */ 

/*
 *  ZNEAR - note producing controlled by potentiometer - DDS technique used
 *
 *  Author: WindFish
 *
 
 OUTPUT of DDS at pin 12 - OC0A. Sample rate approximately 39KHz
 
 chip ATMega328P
 
 */


#define F_CPU   20000000
#define BUAD    9600
#define BRC     ((F_CPU/16/BUAD) - 1)

#define INPsize   32

#define RAND_MAX 255

#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <math.h>

// #define GLCD_DEVICE_AVR8
// #define GLCD_CONTROLLER_PCD8544
// #define GLCD_USE_SPI
// #define GLCD_USE_AVR_DELAY
// #define __DELAY_BACKWARD_COMPATIBLE__

extern "C"
{	 
	#include "nokia5110.h"
	#include "nokia5110_chars.h"
};


const uint8_t wf_Width    = 84;
const uint8_t wf_Height   = 48;
const uint8_t PROGMEM wf_Bitmap[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x40, 0x40, 0x60, 0x20, 0x20, 0x20, 0x10, 0x90, 0xd0, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0x60, 0x30, 0xf8, 0x80, 0x00, 0x00, 0x80, 0xc0, 0x40, 0x60, 0x20, 0x30, 0x10, 0x18, 0x0c, 0x04, 0x06, 0x02, 0x03, 0x01, 0x01, 0x00, 0x80, 0xe0, 0x30, 0x1c, 0x06, 0x02, 0x03, 0x01, 0x01, 0x00, 0x00, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xc0, 0x40, 0x60, 0x20, 0x20, 0x30, 0x10, 0x10, 0x18, 0x08, 0x08, 0x0c, 0x04, 0x04, 0x06, 0x82, 0x83, 0x41, 0x61, 0x30, 0x18, 0x08, 0x0c, 0x1f, 0xf2, 0x83, 0xe1, 0x38, 0xfc, 0x00, 0x00, 0x00, 0xfc, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x7c, 0x07, 0x00, 0x80, 0x80, 0xc0, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0x30, 0x18, 0x0c, 0x06, 0x02, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x37, 0x1f, 0x80, 0x80, 0xb1, 0xef, 0xbe, 0xc3, 0x41, 0x3f, 0xf0, 0x90, 0x10, 0x08, 0x08, 0x04, 0x07, 0x3e, 0xf2, 0x01, 0x01, 0x00, 0x08, 0x08, 0x08, 0x78, 0xc0, 0x80, 0xc0, 0xc0, 0x80, 0xe0, 0x38, 0x00, 0xd0, 0x00, 0xc0, 0x80, 0x80, 0x00, 0xc0, 0x40, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x40, 0x60, 0x20, 0x30, 0x10, 0x10, 0x18, 0x08, 0x08, 0x08, 0x08, 0x0c, 0x04, 0x04, 0x04, 0x04, 0x04, 0x06, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0e, 0x18, 0x30, 0x20, 0x20, 0x30, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0xc0, 0x79, 0x49, 0x48, 0x08, 0x00, 0xd1, 0x00, 0x41, 0x20, 0xa1, 0x00, 0xf1, 0x41, 0xc1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x06, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

PROGMEM const unsigned char waveTable[5][256]  ={
	//Random numbers wave
	{156,164, 53, 27,167,174,146, 99,157, 61, 85, 46, 80, 79, 58,159,
	56,115,234,119,182,127,150,170,133,173,219, 66,227,249,  6, 57,
	203,255, 45,229, 29, 30, 19, 91,202,158,201, 98,172, 63, 69,235,
	106, 95,220, 35,131, 59,248, 81,154, 86, 31,243, 62,238,242,171,
	90,147,109, 60, 94,130, 43,103,122,132, 84,143, 22,247,105, 15,
	169,187,  3, 51,254,251,110,114,108, 49, 36, 13,181,225,188,104,
	230, 41,144,168,183,  9,252, 93,124,107,141,177,140,  1,175, 28,
	26,233,138,231,185,  8,221,214,  4, 89,208, 47,  0,237, 34,176,
	40,120,212,  2,236, 64,153,134,224,209, 44, 97, 16,218,200,165,
	178,180,102,137,118,125,161,245,226, 48, 12, 65, 83,101,190,116,
	217,198,142,192,199,155, 38,126,184, 25,189, 17,151,232,207,240,
	96,186, 74,128,196,211,129,163, 21, 73,117,135,197, 14, 24,179,
	20,113, 10,152, 77,160, 70,149,121, 33, 75,112, 88,195,194, 42,
	136,100, 78, 32,191,193, 68, 55,223,204,  7,215,162, 11, 52, 92,
	250,239, 23, 87,  5,205, 54,139, 71,228,222, 37,241,206,210,213,
	148,244, 67,253, 50, 76, 82, 18,246,166, 72, 39,111,216,145,123,},
	
	//square wave
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255},
	
	
	//left saw (negative slope)
	{255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,
	212,211,210,209,208,207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,
	176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,159,158,157,156,155,154,153,152,151,150,149,148,147,146,145,144,143,142,141,
	140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,95,94,93,92,91,90,
	89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,
	41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0},
	
	//triangle wave
	{1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,33,35,37,39,41,43,45,47,49,51,53,55,57,59,61,63,65,67,69,71,73,75,77,79,81,83,85,
	87,89,91,93,95,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,127,129,131,133,135,137,139,141,143,145,147,149,151,153,155,157,159,161,163,165,167,169,171,173,175,177,179,181,183,185,187,189,191,193,195,
	197,199,201,203,205,207,209,211,213,215,217,219,221,223,225,227,229,231,233,235,237,239,241,243,245,247,249,251,253,255,255,253,251,249,247,
	245,243,241,239,237,235,233,231,229,227,225,223,221,219,217,215,213,211,209,207,205,203,201,199,197,195,193,191,189,187,185,183,181,179,177,
	175,173,171,169,167,165,163,161,159,157,155,153,151,149,147,145,143,141,139,137,135,133,131,129,127,125,123,121,119,117,115,113,111,109,107,
	105,103,101,99,97,95,93,91,89,87,85,83,81,79,77,75,73,71,69,67,65,63,61,59,57,55,53,51,49,47,45,43,41,39,37,35,33,31,29,27,25,23,21,19,17,15,
	13,11,9,7,5,3,1},	
	
	//sine wave
	{0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,
	70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124,127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,
	221,223,225,227,229,231,233,234,236,238,239,240,242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,
	253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,221,219,217,215,212,210,208,205,
	203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,
	102,99,96,93,90,87,84,81,78,76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,
	3,2,2,1,1,1,0,0,0} };
	


//Note frequencies

PROGMEM float const keyFreq[88] = {
	27.5, 29.1352, 30.8677,																					   //Octave 0
	32.7032, 34.6478, 36.7081, 38.8909, 41.2034, 43.6535, 46.2493, 48.9994, 51.9131, 55, 58.2075, 61.7354,     //Octave 1
	65.4064, 69.2957, 73.4162, 77.7817, 82.4069, 87.3071, 92.4986, 97.9989, 103.826, 110, 116.541, 123.471,    //Octave 2
	130.813, 138.591, 146.832, 155.563, 164.814, 174.614, 184.997, 195.998, 207.652, 220, 233.082, 246.942,    //Octave 3
	261.626, 277.183, 293.665, 311.127, 329.628, 349.228, 369.994, 394.995, 415.305, 440, 466.164, 493.883,    //Octave 4
	523.251, 554.365, 587.330, 622.254, 659.255, 698.456, 739.989, 783.991, 830.609, 880, 932.328, 987.767,    //Octave 5
	1046.50, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22, 1760, 1864.66, 1975.53,   //Octave 6
	2093.00, 2217.46, 2349.32, 2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44, 3520, 3729.31, 3951.07,   //Octave 7
	4186.01                                                                                                    //Octave 8
};


volatile uint16_t	refclk=39063;		// refclk=3906 = 20MHz / 510 ... increase to compensate for higher pitch
										// 510 comes from formula for calculating phase correct PWM frequency
										
volatile uint16_t	phaccu[2] = {0, 0};
volatile uint16_t	tword[2] = {0, 0};
volatile uint8_t	key[2] = {48, 48};
volatile uint8_t	osc[2] = {0, 0};

volatile uint8_t	waveKind[2] = {4, 4};

volatile uint16_t	dutyCycle;
volatile uint16_t	ADCinputs[INPsize];
volatile uint8_t	inputsi;
volatile uint32_t	rollingMeanADC;

volatile uint8_t	smallTimer = 0;				//Counts 1/40ms - every timer0 overflow interrupt
volatile uint16_t	millisecs = 0;				//Counts 1 ms every 40 smallTimer clicks
volatile uint16_t	millisecsLast = 0;			//Counts ms - used to mark last time note calculations were made
volatile uint16_t	millisecsAutoButton = 0;	//Counts ms - used for AutoButton function
volatile uint16_t	millisecsSerial = 0;		//Counts ms - used for timing some serial debugging commands

volatile uint8_t	notePlaying[2] = {0, 0};	//Boolean  - enables oscillators if a note is playing
volatile uint8_t	notePlayingSum = 0;			//Boolean  - enables oscillators if a note is playing

volatile uint8_t	volume[2] = {0, 0};
volatile uint8_t	volumeRelease[2] = {0, 0};

volatile uint8_t	mainNoteADC[2] = {48, 48};		//INPUT - main note
volatile float		fine[2] = {1, 1};				//INPUT - fine
volatile uint16_t	finalNoteADC[2] = {550, 550};	//Note after sweep and vibrato are applied

volatile uint8_t	attack[2] = {20, 35};			//INPUT - Envelope attack
volatile uint8_t	decay[2] = {90, 70};			//INPUT - Envelope decay
volatile uint8_t	sustain[2] = {100, 100};		//INPUT - Envelope sustain
volatile uint8_t	release[2] = {150, 100};		//INPUT - Envelope release
volatile uint8_t	releaseMode[2] = {1, 1};		//INPUT - Release mode (0->linear or 1->nonlinear)

volatile uint8_t	envelopeStage[2] = {4, 4};		//Tracks the current stage of the envelope
volatile uint16_t	millisecsEnvelope[2] = {0, 0};	//Timer for the envelope incremented in ms

volatile uint8_t	sweepSpeed[2] = {70, 70};		//INPUT - Sweep speed (0->one semi a second, 255->fast!)
volatile uint8_t	sweepDirection[2] = {0, 0};		//INPUT - Sweep direction (0->down, 1->up, 2->from down, 3->from up)
volatile float		sweepMultiplier[2] = {1, 1};	//Note frequency multiplied with this number for sweep
volatile uint16_t	millisecsSweep[2] = {0, 0};		//Timer for the sweep incremented in ms

volatile uint16_t  vibratoSpeed[2] = {400, 400};	//INPUT - Vibration speed
volatile uint8_t   vibratoDepth[2] = {20, 20};		//INPUT - Vibration intensity
volatile uint8_t   vibratoWaveKind[2] = {4, 4};		//INPUT - Vibration
volatile uint16_t  vibratoPhacc[2] = {0, 0};		//Vibrato phase accumulator

volatile uint8_t   buttonPin[4] = {0x80, 0x04, 0x08, 0x10};
volatile uint8_t   button[4] = {0, 0, 0, 0};
volatile uint8_t   timerb[4] = {0, 0, 0, 0};

// volatile uint8_t buttonPin[4] = {1 << PORTB0, 1 << PORTB1, 1 << PORTB2, 1 << PORTB3};
// volatile uint8_t buttonState[4] = {0, 0, 0, 0};
// volatile uint8_t millisecsButton[4] = {0, 0, 0, 0};

//Display stuff
volatile uint8_t   millisecsFPS=0;				// Timer for 20FPS
volatile uint8_t   itemSelected[3]={0,0,0};		// Keeps track of selections across pages
volatile const uint8_t   pageItems[3]={1,5,5};	// Number of page items is +1 of the value of this variable. pageItems[0] has a value of 1 - therefore page 0 has 2 items
volatile uint8_t   page=0;						// Current page



void setupSerial() //serial for buade rate 9600 (UART)
{
	UBRR0H = (BRC >> 8);
	UBRR0L =  BRC;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void setupSPI()
{
	SPCR  = 0b11110000;
}

void setupPINS() //Sets DDRs and PORT registers
{
	DDRB  = 0b00101111;
	PORTB = 0b11010000;
	
	DDRD  = 0b01100000;
	PORTD = 0b10011111;
}

void setupTIMER0() //Sets up TIMER0 overflow interrupt and PWM
{
	TIMSK0 = (1 << TOIE0);   //overflow interrupt
	TCCR0A = (1 << COM0A1) | (1 << WGM00);   //PWM mode
	TCCR0B = (1 << CS00);   //clock source = CLK, start PWM

	OCR0A  = 0;
}

void startConversion() //Sets the bit that starts a new ADC conversion
{
	ADCSRA |= (1 << ADSC);
}

void setupADC() //Sets up ADC for and conversion complete interrupt. Then the first conversion starts. In the interrupt the ADC output is read and a new conversion is started
{
	ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX2);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
	DIDR0 = (1 << ADC5D);
	
	startConversion();
}


void setADC(uint8_t ADCno) //Sets ADMUX last 3 bits that choose the conversion's target. Acceptable values for ADCno: 0..7 for ADC0...ADC7
{
	ADMUX = (ADMUX & 0b11111000) | ADCno;
}

void sweepADC(	uint8_t ADCno1, uint8_t ADCno2, uint8_t ADCno3=0, uint8_t ADCno4=0,			//Sweeps between several ADC pins to be converted. Should be used in ADC interrupt
				uint8_t ADCno5=0, uint8_t ADCno6=0, uint8_t ADCno7=0, uint8_t ADCno8=0 )
{
	if (!ADCno1 || !ADCno2) return;
	if (ADCno1 == (ADMUX & 0b00000111))
		setADC(ADCno2);
	else if (ADCno2 == (ADMUX & 0b00000111))
		ADCno3? setADC(ADCno3) : setADC(ADCno1);
	else if (ADCno3 == (ADMUX & 0b00000111))
		ADCno4? setADC(ADCno4) : setADC(ADCno1);
	else if (ADCno4 == (ADMUX & 0b00000111))
		ADCno5? setADC(ADCno5) : setADC(ADCno1);
	else if (ADCno5 == (ADMUX & 0b00000111))
		ADCno6? setADC(ADCno6) : setADC(ADCno1);
	else if (ADCno6 == (ADMUX & 0b00000111))
		ADCno7? setADC(ADCno7) : setADC(ADCno1);
	else if (ADCno7 == (ADMUX & 0b00000111))
		ADCno8? setADC(ADCno8) : setADC(ADCno1);
	else if (ADCno8 == (ADMUX & 0b00000111))
		setADC(ADCno1);
}

void ButtonCheck(volatile uint8_t& button,volatile uint8_t& timer,volatile const uint8_t& buttonPin) //Software debounced button check
{	
	if ((PIND&buttonPin) == 0){
		if (!button && (timer>10))
		{
			button = 1;
			timer= 0;
			
			itemSelected[page] < pageItems[page]? ++itemSelected[page] : itemSelected[page] = 0;
			
			for (uint8_t i=0; i<2; ++i)
			{
				phaccu[i] = 0;
				vibratoPhacc[i] = 0;

				millisecsSweep[i] = 0;
				envelopeStage[i] = 0;
				millisecsEnvelope[i] = 0;
				volumeRelease[i] = 0;
				waveKind[i]>=4 ? waveKind[i]=0 : ++waveKind[i];
				volume[i] = 0;
				notePlaying[i] = 1;
			}
		}
	}
	else if ((PIND & buttonPin) != 0){
		if (button && (timer > 10)){
			button = 0;
			timer = 0;
			
		}
	}
}

void CalcRollingMeanADC() //Calculating rolling mean
{
	rollingMeanADC=0;
	for (uint8_t i = 0; i < INPsize; ++i)
		rollingMeanADC += ADCinputs[i];
	rollingMeanADC = rollingMeanADC >> 5;
}

void AutoButton(volatile uint8_t& button, uint16_t onDuration = 10000, uint16_t offDuration = 3000) //emulates button push - durations in ms
{
// 	UDR0 = '0';
// 	_delay_ms(20);
	
	if (millisecsAutoButton > onDuration && button == 1)
	{
// 		UDR0 = '1';
// 		_delay_ms(20);
		
		button = 0;
		millisecsAutoButton = 0;
	}
	else if (millisecsAutoButton > offDuration && button == 0)
	{
// 		UDR0 = '2';
// 		_delay_ms(20);
		
		button = 1;
		millisecsAutoButton = 0;
		
		for (uint8_t i=0; i<2; ++i)
		{
			phaccu[i] = 0;
			vibratoPhacc[i] = 0;
			
			sweepMultiplier[i] = 1;
			millisecsSweep[i] = 0;
			envelopeStage[i] = 0;
			millisecsEnvelope[i] = 0;
			volumeRelease[i] = 0;
			waveKind[i]>=4 ? waveKind[i]=0 : ++waveKind[i];
			volume[i] = 0;
			notePlaying[i] = 1;
		}
	}
}

void ADCtoSerial() //outputs the ADC to serial TxD
{
	uint16_t gj = rollingMeanADC;
	
	UDR0=(gj/1000)+48;
	gj%=1000;
	_delay_ms(10);
	UDR0=(gj/100)+48;
	gj%=100;
	_delay_ms(10);
	UDR0=(gj/10)+48;
	gj%=10;
	_delay_ms(10);
	UDR0=gj+48;
	_delay_ms(10);
	
	UDR0='\n';
	_delay_ms(100);
}

void BYTEtoSerial(uint8_t gj) //Sends byte to serial
{
	UDR0=(gj/100)+48;
	gj%=100;
	_delay_ms(20);
	UDR0=(gj/10)+48;
	gj%=10;
	_delay_ms(20);
	UDR0=gj+48;
	_delay_ms(20);
	UDR0='\n';
	_delay_ms(100);
}

inline void setNoteFreq(uint16_t Freq, uint8_t oscillator)
{
	tword[oscillator] = 0x10000*Freq/refclk;
}

uint16_t setNoteADC(uint16_t inp10bit, uint8_t oscillator)
{
	if (inp10bit < 93)
		setNoteFreq(( inp10bit * 15 ) / 93, oscillator);
	else if (inp10bit < 186)
		setNoteFreq(( inp10bit * 30 ) / 186, oscillator);
	else if (inp10bit < 279)
		setNoteFreq(( (inp10bit-93) * 60 ) / 186, oscillator);
	else if (inp10bit < 372)
		setNoteFreq(( (inp10bit-186) * 120 ) / 186, oscillator);
	else if (inp10bit < 465)
		setNoteFreq(( (inp10bit-279) * 240 ) / 186, oscillator);
	else if (inp10bit < 558)
		setNoteFreq(( (inp10bit-372) * 480L ) / 186, oscillator);
	else if (inp10bit < 651)
		setNoteFreq(( (inp10bit-465) * 960L ) / 186, oscillator);
	else if (inp10bit < 744)
		setNoteFreq(( (inp10bit-558) * 1920L ) / 186, oscillator);
	else if (inp10bit < 837)
		setNoteFreq(( (inp10bit-651) * 3840L ) / 186, oscillator);
	else if (inp10bit < 930)
		setNoteFreq(( (inp10bit-744) * 7680L ) / 186, oscillator);
	else
		setNoteFreq(( (inp10bit-838) * 16000L ) / 194, oscillator);
}

void doSweepADC(uint8_t oscillator)
{
	if (sweepSpeed[oscillator] > 2)
	{
		if (sweepDirection[oscillator] == 0)
		{
			if (mainNoteADC[oscillator] > (static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125)
				finalNoteADC[oscillator] = mainNoteADC[oscillator] - (static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125;
			else
				finalNoteADC[oscillator] = 0;
		}
		else if (sweepDirection[oscillator] == 1)
		{
			finalNoteADC[oscillator] = mainNoteADC[oscillator] + (static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125;
		}
		else if (sweepDirection[oscillator] == 2)
		{
			if ((static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125 <= 93)
				finalNoteADC[oscillator] = mainNoteADC[oscillator] - (93 - (static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125);
			else
				finalNoteADC[oscillator] = mainNoteADC[oscillator];
		}
		else if (sweepDirection[oscillator] == 3)
		{
			if ((static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125 <= 93)
				finalNoteADC[oscillator] = mainNoteADC[oscillator] + (93 - (static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125);
			else
				finalNoteADC[oscillator] = mainNoteADC[oscillator];
		}
	}
	
}

void doSweep(uint8_t oscillator)
{
	if (sweepSpeed[oscillator] > 2)
	{
		if (sweepDirection[oscillator] == 0)
		{
			
			sweepMultiplier[oscillator] = 1 + static_cast<float>(sweepSpeed[oscillator]) * millisecsSweep[oscillator] / 2048; //1 + sweepSpeed/256 * millisecsSweep/8
		
		}
		else if (sweepDirection[oscillator] == 1)
		{
			finalNoteADC[oscillator] = mainNoteADC[oscillator] + (static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125;
		}
		else if (sweepDirection[oscillator] == 2)
		{
			if ((static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125 <= 93)
			finalNoteADC[oscillator] = mainNoteADC[oscillator] - (93 - (static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125);
			else
			finalNoteADC[oscillator] = mainNoteADC[oscillator];
		}
		else if (sweepDirection[oscillator] == 3)
		{
			if ((static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125 <= 93)
			finalNoteADC[oscillator] = mainNoteADC[oscillator] + (93 - (static_cast<uint32_t>(millisecsSweep[oscillator]) * sweepSpeed[oscillator]) / 125);
			else
			finalNoteADC[oscillator] = mainNoteADC[oscillator];
		}
	}
	
	
}


inline void doEnvelope(uint8_t oscillator)
{
	switch (envelopeStage[oscillator])
	{
		case 0:
		if (!button[0])
		{
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		else if (millisecsEnvelope[oscillator] >= static_cast<uint16_t>(attack[oscillator] << 4))
		{
			volume[oscillator] = 255;
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		else
		{
			volume[oscillator] = (millisecsEnvelope[oscillator] << 4) / attack[oscillator]; // millisecsEnvelope / (attack/16)
		}
		break;
	
		case 1:
		if (!button[0] || sustain[oscillator] == 0)
		{
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		else if (volume[oscillator] <= sustain[oscillator])
		{
			volume[oscillator] = sustain[oscillator];
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		else
		{
			volume[oscillator] = 255 - ((millisecsEnvelope[oscillator] << 4) / decay[oscillator] ) ;// 255 - millisecsEnvelope / (decay/16);
		}
		break;
	
		case 2:
		if (!button[0] || sustain[oscillator] == 0)
		{
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		break;
	
		case 3:
		if (volume[oscillator] <= 1)
		{
			volume[oscillator] = 0;
			millisecsEnvelope[oscillator] = 0;
			++envelopeStage[oscillator];
			notePlaying[oscillator] = 0;
		}
		else if (release[oscillator] > 253)
		{
			++envelopeStage[oscillator];
			break;
		}
		else
		{
			if (!volumeRelease[oscillator]) volumeRelease[oscillator] = volume[oscillator];
		
			if (!releaseMode[oscillator])
				if (volumeRelease[oscillator] > ((millisecsEnvelope[oscillator] << 4) / release[oscillator]))
					volume[oscillator] = volumeRelease[oscillator] - ((millisecsEnvelope[oscillator] << 4) / release[oscillator]); //Linear release
				else volume[oscillator] = 0;
			else
				volume[oscillator] = (volumeRelease[oscillator] * release[oscillator]) / ((millisecsEnvelope[oscillator] >> 1) + release[oscillator]) - (millisecsEnvelope[oscillator]/1000); //Non linear release
		}
	}
}

bool BitmapXYaccess(const uint8_t* bitmap, const uint8_t& x = 0, const uint8_t& y = 0) //Returns the 0 or 1 of the bitmap on its x,y position
{
	
	return (pgm_read_byte( &bitmap[x + (y >> 3) * 84] ) >> (y % 8)) & 0x01;
}

void nokia_lcd_write_bitmap(const uint8_t* bitmap, const uint8_t& bwidth, const uint8_t& bheight, const uint8_t& x = 0, const uint8_t& y = 0) //Writes a bitmap of dimensions bwidth,bheighton the display, starting at x,y
{
	for (uint8_t i = x; i <= bwidth; ++i)
	{
		for (uint8_t j = y; j <= bheight; ++j)
		{
			nokia_lcd_set_pixel(i,j, BitmapXYaccess(bitmap, i, j) );
		}
	}
}

uint16_t _map(uint16_t X, uint16_t A, uint16_t B, uint16_t C, uint16_t D)
{
	uint16_t Y = 0;
	if (X < A)
		C < D? Y = C : Y = D;
	else if (X > B)
		C < D? Y = D : Y = C;
	else
		Y = (X-A)/(B-A) * (D-C) + C;
	return Y;
}


int main(void)
{
 	setupSerial();
 	
// 	setupSPI();
 	
 	setupPINS();
	
	setupTIMER0();
	
	setupADC();
	startConversion();
	
	
// 	CLKPR = (1 << CLKPCE); // Prescale clock by 1/256
// 	CLKPR = (1 << CLKPS3);
		
	
// 	nokia_lcd_init();
// 	nokia_lcd_clear();
// 	
// 	nokia_lcd_write_bitmap(wf_Bitmap, wf_Width, wf_Height);
// 	nokia_lcd_render();
// 	_delay_ms(3000);
// 	
// 	nokia_lcd_clear();
// 	nokia_lcd_set_cursor(0, 0);
// 	nokia_lcd_write_string("ZNEAR", 3);
// 	nokia_lcd_set_cursor(5, 23);
// 	nokia_lcd_write_string("Synth", 3);
// 	nokia_lcd_render();
// 	_delay_ms(4000);
	
	
// 	
// 	_delay_ms(3000);
	sei();
	
	while(1)
    {		
// 		for (uint8_t i=0; i < 3; ++i)
// 			ButtonCheck(button[i] , timerb[i], buttonPin[i]);
			
			
// 		if (millisecsFPS > 50)
// 		{
// 			millisecsFPS = 0;
// 			
// 			nokia_lcd_clear();
// 			
// // 			switch page
// 			
// 			nokia_lcd_set_cursor(16, 0);
// 			nokia_lcd_write_string("OSC1", 3);
// 			nokia_lcd_set_cursor(16, 23);
// 			nokia_lcd_write_string("OSC2", 3);
// 			
// 			nokia_lcd_set_cursor(1, itemSelected[page] * 23);
// 			nokia_lcd_write_char(0x80 , 3);
// 			
// 			nokia_lcd_render();
// 			
// 		}
		
		
			
 		if (millisecs != millisecsLast)
		{
			millisecsLast = millisecs;
			
			AutoButton(button[0], 3000, 5000);
			
			//BYTEtoSerial(notePlayingSum);

 			CalcRollingMeanADC();

//================INPUST=============================

// 		if (rollingMeanADC < 200) waveKind = 0;								//WAVE KIND INPUT
// 		else if (rollingMeanADC < 400) waveKind = 1;
// 		else if (rollingMeanADC < 600) waveKind = 2;
// 		else if (rollingMeanADC < 800) waveKind = 3;
// 		else waveKind = 4;
//
//		key[0] = (rollingMeanADC) * 87 / 1024;								//NOTE INPUT
// 
// 		if (rollingMeanADC < 450)											//FINE INPUT
// 			fine[0] = 1 / (1 + (0.06 * (450 - rollingMeanADC) / 450));
// 		else if (rollingMeanADC < 550)
// 			fine[0] = 1;
// 		else
// 			fine[0] = 1 + (0.06 * (rollingMeanADC - 550) / 474);
//
// 		attack = (rollingMeanADC >> 2);		//ATTACK INPUT
// 		if (attack < 3) attack = 3;
//		
// 		decay = (rollingMeanADC >> 2);		//DECAY INPUT
// 		if (decay < 2) decay = 2;
//		
// 		sustain = (rollingMeanADC >> 2);	//SUSTAIN INPUT
// 		if (sustain < 2) sustain = 0;
//		
// 		release = (rollingMeanADC >> 2);	//RELEASE INPUT
// 		if (release < 5) release = 5;
//
//		releaseMode = rollingMeanADC >> 9;	//RELEASE MODE INPUT
//
//		sweepDirection = rollingMeanADC >> 8;	//SWEEP DIRECTION INPUT
		sweepSpeed[0] = rollingMeanADC >> 2;		//SWEEP SPEED INPUT
//











//===================================================
				
		
// 		key[0] = 48 + (rollingMeanADC)/1024.0*13; //POT sweeps an octave of notes from 48th in freq table(A440) to higher
//		key[0] = (rollingMeanADC) * 87 / 1024; //POT sweeps all notes
		
		//setNoteADC(rollingMeanADC, 0);
		
 		key[0] = 41;
 		tword[0] = 0x10000 * (pgm_read_float(&(keyFreq[key[0]])) * fine[0]) / refclk; //tword calculated from freq table		
  		key[1] = 48;
 		tword[1] = 0x10000 * (pgm_read_float(&(keyFreq[key[1]])) * fine[1]) / refclk; //tword calculated from freq table
		
// 		volume[0] = 0xff;
// 		volume[1] = 0xff;
		
// 		volume = rollingMeanADC >> 2;
// 		
//   	tword[0]=0x10000/refclk*( rollingMeanADC << 4 );// *16384/1024; Sweeps from 0 to 16384Hz

 		
//		doSweepADC(0);
		 
// 		mainNoteADC = 650;
		
// 		sustain = 255;
// 		
// 		release = 253;
				
// 		sweepDirection = 2;
			
// 		tword[0]=0x10000/refclk*( 95*exp(0.005*rollingMeanADC));// y=a*exp(b*ADC), in order to sweep from 0 to 16000 for ADC sweeping from 0 to 1024
// 		
// 		volume = button1 * 255;
		
// 		tword[0]=0x10000*440/refclk;
		
		
				
		
		
		
// 		finalNoteADC[0] = mainNoteADC[0];
// 		
// 		vibratoDepth[0] = 255;
// 		vibratoWaveKind[0] = 4;
// 		vibratoSpeed[0] = 400;
		
// 		if (vibratoSpeed < 160)
// 			vibratoPhacc += (vibratoSpeed >> 4);
// 		else if (vibratoSpeed < 352)
// 			vibratoPhacc += ((vibratoSpeed - 160) >> 3) + 10;
// 		else if (vibratoSpeed < 512)
// 			vibratoPhacc += ((vibratoSpeed - 352) >> 2) + 34;
// 		else if (vibratoSpeed < 680)
// 			vibratoPhacc += ((vibratoSpeed - 512) >> 1) + 74;
// 		else if (vibratoSpeed < 850)
// 			vibratoPhacc += ((vibratoSpeed - 680) << 2) + 158;
// 		else
// 			vibratoPhacc += ((vibratoSpeed - 850) << 3) + 838;
			
// 		finalNoteADC += ((pgm_read_byte( &waveTable[ vibratoWaveKind ][ vibratoPhacc >> 8 ] ) - 128) * vibratoDepth) / 255;
		
// 		setNoteADC(finalNoteADC[0],0);
	
		doEnvelope(0);
		doEnvelope(1);
		
		
		
		notePlayingSum = 0;
		for (uint8_t i=0; i<2; ++i)
			notePlayingSum += notePlaying[i];		
	}
	
	
// 		tword[0]=0x10000*pgm_read_float(&(keyFreq[key[0]]))/refclk;
// 		tword[1]=0x10000*pgm_read_float(&(keyFreq[key[1]]))/refclk;
// 		_delay_ms(1000);
		
// 	if (millisecsSerial > 20)
// 	{
// 		UDR0 = button1 + 48;
// 		millisecsSerial = 0;
// 	}
	
	}
}




ISR (TIMER0_OVF_vect)
{
// 	PORTD ^= (1 << PORTD5);
// 	UDR0='3';
//	OCR0A ^= 0xff;


  	if (notePlayingSum)
	{
 		phaccu[0] += tword[0];//738;440Hz
 		osc[0] = (pgm_read_byte( &waveTable[ waveKind[0] ][ phaccu[0] >> 8 ]) * 128) / 256;
		
 		osc[1] = (pgm_read_byte( &waveTable[ waveKind[1] ][ phaccu[1] >> 8 ]) * 128) / 256;
 		phaccu[1] += tword[1];
		

//		OCR0A = ( osc[0] * volume[0] ) >> 8; //;pgm_read_byte(&(waveTable[tremolo]))/0xff;
//		OCR0A = ( osc[1] * volume[1] ) >> 8; //;pgm_read_byte(&(waveTable[tremolo]))/0xff;
		
		OCR0A = (( osc[0] * volume[0] ) >> 8) + (( osc[1] * volume[1] ) >> 8); //;pgm_read_byte(&(waveTable[tremolo]))/0xff;
 	}

	
	smallTimer++; //increment small timer
	if(smallTimer > 40)
	{		
		smallTimer = 0;
		++millisecs;
		
		for (uint8_t i=0; i < 3; ++i)
			++timerb[i];
		
		++millisecsAutoButton;
		++millisecsSerial;
		++millisecsFPS;
		
		for (uint8_t i=0; i<2; ++i)
		{
			++millisecsEnvelope[i];
			++millisecsSweep[i];
		}
 		
		ADCinputs[inputsi] = dutyCycle; //INPsize inputs for the rolling mean
		inputsi>INPsize? inputsi=0 : inputsi++;
		startConversion();
 	}	
}

ISR(ADC_vect)
{
	dutyCycle = ADC;
	
//	sweepADC(ADCused1, ADCused2);
//	if (ADMUX & 0b00000111 != ADCused1)
//  startConversion();
}

//ISR(SPI_STC_vect)






