/*
 *  ZNEAR - note producing controlled by potentiometer - DDS technique used
 *
 *  Author	: WindFish
 *
 *	Rel Date: 7/11/2020
 *
 *	OUTPUT of DDS at pin 12 - OC0A. Sample rate approximately 39KHz
 *
 *	chip ATMega328P
 */

#define F_CPU   20000000 //CPU speed in Hz for use with util/delay library and Serial communication
#define BUAD    9600	//For Serial communication
#define BRC     ((F_CPU/16/BUAD) - 1) //For Serial communication

#define RAND_MAX 255 //for random generation (not actually used)

#define INPsize   32 //for rolling mean filtering of the ADC input

//C++ libraries
#include <stdint.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <avr/eeprom.h>
#include "helping_functions.h"

//Imporetd C libraries
extern "C"
{	 
	#include "nokia5110.h"
	#include "nokia5110_chars.h"
};

//Welcome screen bitmap
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

// 'legendOSCbitmap', 11x48px
const uint8_t leg_Width    = 11;
const uint8_t leg_Height   = 48;
const uint8_t PROGMEM leg_Bitmap[] = {
	0xfe, 0xff, 0x03, 0x03, 0x03, 0xff, 0xfe, 0x00, 0x00, 0x33, 0x44,
	0xc1, 0xe3, 0x63, 0x6b, 0x63, 0x63, 0x61, 0x00, 0x00, 0xe7, 0x08,
	0x63, 0x67, 0x66, 0x66, 0x66, 0x7e, 0x3c, 0x00, 0x80, 0xb9, 0x42,
	0xf8, 0xfc, 0x0c, 0x0d, 0x0c, 0x1c, 0x18, 0x00, 0x01, 0x9d, 0x42,
	0x07, 0x0f, 0x0c, 0x0c, 0x0c, 0x0e, 0x06, 0x00, 0x00, 0xe7, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcc, 0x22
};

// 'legend1', 7x9px
const uint8_t leg1_Width    = 7;
const uint8_t leg1_Height   = 10;
const uint8_t PROGMEM leg1_Bitmap[] = {
	0x00, 0x00, 0x06, 0xff, 0xff, 0x00, 0x00,
	0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x00
};

// 'legend2', 7x9px
const uint8_t leg2_Width    = 7;
const uint8_t leg2_Height   = 10;
const uint8_t PROGMEM leg2_Bitmap[] = {
	0x0e, 0x8e, 0xc3, 0xe3, 0x73, 0x3e, 0x1e,
	0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03
};

const uint8_t wave_Width    = 15;
const uint8_t wave_Height   = 8;
const uint8_t PROGMEM rand_Bitmap[] = {
	0x40, 0x06, 0x21, 0x10, 0x42, 0x20, 0x0c, 0x42, 0x10, 0x22, 0x09, 0x44, 0x40, 0x12, 0x44
};

const uint8_t PROGMEM square_Bitmap[] = {
	0x40, 0x40, 0x40, 0x40, 0x7f, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x7f, 0x40, 0x40, 0x40
};

const uint8_t PROGMEM saw_Bitmap[] = {
	0x7f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x7f, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20
};

const uint8_t PROGMEM triangle_Bitmap[] = {
	0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02
};

const uint8_t PROGMEM sin_Bitmap[] = {
	0x40, 0x40, 0x60, 0x30, 0x1c, 0x06, 0x03, 0x01, 0x01, 0x03, 0x06, 0x1c, 0x30, 0x60, 0x40
};

//wave tables
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
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,15,30,45,60,75,90,105,120,135, 150,165,180,195,210,225,240, 
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15},
	
	
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
	


//Note frequencies but in TWORD values
PROGMEM uint16_t const twordNotes[88] = {
	46, 49, 52,																	//Octave 0
	55, 58, 62, 66, 69, 74, 78, 82, 87, 92, 98, 104,							//Octave 1
	110, 116, 123, 131, 138, 147, 155, 164, 174, 184, 195, 207,					//Octave 2
	219, 232, 246, 261, 276, 293, 310, 328, 348, 368, 390, 414,					//Octave 3
	438, 464, 492, 521, 552, 585, 619, 661, 695, 735, 779, 826,					//Octave 4
	875, 927, 982, 1040, 1102, 1168, 1237, 1311, 1389, 1470, 1558, 1651,		//Octave 5
	1749, 1853, 1963, 2080, 2203, 2334, 2473, 2620, 2776, 2940, 3115, 3301,		//Octave 6
	3497, 3704, 3925, 4158, 4405, 4667, 4945, 5238, 5550, 5880, 6230, 6600,		//Octave 7
	6993																		//Octave 8
};


volatile uint16_t	refclk=39185;			// refclk=3906 = 20MHz / 510 ... increase to compensate for higher pitch
											// 510 comes from formula for calculating phase correct PWM frequency
																			
volatile uint16_t	phaccu[2] = {0, 0};		//These variables are used for 
volatile uint16_t	tword[2] = {0, 0};		//outputting them with DDS
uint16_t			twordcalc[2] = {0, 0};	//
uint8_t				osc[2] = {0, 0};		//
				

uint16_t	dutyCycle;						//Variables for rollingMeanADC calculation
uint16_t	ADCinputs[INPsize];				//
uint8_t		inputsi;						//
uint32_t	rollingMeanADC;					//

bool	inputActive = false;				//When true rollingMean is being used to input values to an INPUT setting variable

uint8_t				smallTimer = 0;			//Counts 1/40ms - every timer0 overflow interrupt - starts at 0
volatile uint8_t	wait = 1;				//A part of note calculations is done once after every small timer tick. After the calculation this is set to 0
volatile uint8_t	millisecs = 0;			//Counts 1 ms every 40 smallTimer clicks

uint16_t	millisecsAutoButton = 0;		//Counts ms - used for AutoButton function
uint16_t	millisecsSerial = 0;			//Counts ms - used for timing some serial debugging commands
bool		serialGO = false;					//Starts the serial out

uint8_t	notePlaying[2] = {0, 0};		//Boolean  - enables oscillators if a note is playing
uint8_t	notePlayingSum = 0;				//Sum of notes that play

uint8_t		volume[2] = {0, 0};				//Current oscillator volume - calculated in envelope
uint8_t		volumecalc[2] = {0, 0};			//Used for calculations before assigning it to volume
uint8_t		volumeRelease[2] = {0, 0};		//Used for envelope calculations

uint8_t		waveKind[10] = {4,4, 4,4, 4,4, 4,4, 4,4};						//INPUT - oscillator wave used - for every button
uint8_t		phase0[10] = {0,0, 0,0, 0,0, 0,0, 0,0,};						//INPUT - beginning phase of a wave - for every button
uint8_t		key[10] = {57,57, 55,55, 52,52, 50,50, 48,48};					//INPUT - main note key - for every button
uint16_t	noteADC[10] = {550,550, 550,550, 550,550, 550,550, 550,550, };	//INPUT - main note ADC - for every button
bool		noteSource[10] = {1,1, 1,1, 1,1, 1,1, 1,1};						//INPUT - note source (key or ADC) - for every button
uint8_t		fine[10] = {127,127, 127,127, 127,127, 127,127, 127,127};		//INPUT - fine - for every button
uint8_t		oscVolume[10] = {255,255, 255,255, 255,255, 255,255, 255,255};	//INPUT - oscillator volume - for every button
uint8_t*	pwaveKind = waveKind;			//points to setting in use
uint8_t*	pphase0 = phase0;				//points to setting in use
uint8_t*	pkey = key;						//points to setting in use
uint16_t*	pnoteADC = noteADC;				//points to setting in use
bool*		pnoteSource = noteSource;		//points to setting in use
uint8_t*	pfine = fine;					//points to setting in use
uint8_t*	poscVolume = oscVolume;			//points to setting in use

uint8_t		attack[10] = {50,50, 50,50, 50,50, 50,50, 50,50};				//INPUT - Envelope attack - for every button
uint8_t		decay[10] = {190,190, 190,190, 190,190, 190,190, 190,190};		//INPUT - Envelope decay - for every button
uint8_t		sustain[10] = {70,70, 70,70, 70,70, 70,70, 70,70};				//INPUT - Envelope sustain - for every button
uint8_t		release[10] = {150,150, 150,150, 150,150, 150,150, 150,150};	//INPUT - Envelope release - for every button
uint8_t		releaseMode[10] = {1,1, 1,1, 1,1, 1,1, 1,1};					//INPUT - Release mode (0->linear or 1->nonlinear) - for every button
uint8_t*	pattack = attack;				//points to setting in use
uint8_t*	pdecay = decay;					//points to setting in use
uint8_t*	psustain = sustain;				//points to setting in use
uint8_t*	prelease = release;				//points to setting in use
uint8_t*	preleaseMode = releaseMode;		//points to setting in use

uint8_t		sweepSpeed[10] = {0,0, 0,0, 0,0, 0,0, 0,0};				//INPUT - Sweep speed (0->one semi a second, 255->fast!) - for every button
uint8_t		sweepDirection[10] = {0,0, 0,0, 0,0, 0,0, 0,0};			//INPUT - Sweep direction (0->down, 1->up, 2->from down, 3->from up) - for every button
uint8_t*	psweepSpeed = sweepSpeed;								//points to setting in use
uint8_t*	psweepDirection = sweepDirection;						//points to setting in use
bool		sweepStop[10] = {0, 0};									//Sweep should stop when 0 or 20KHz are reached
uint16_t	millisecsSweep[10] = {0, 0};							//Timer for the sweep incremented in ms

uint16_t			vibratoSpeed[10] = {0,0, 0,0, 0,0, 0,0, 0,0};				//INPUT - Vibration speed - for every button
uint8_t				vibratoDepth[10] = {35,35, 35,35, 35,35, 35,35, 35,35};		//INPUT - Vibration intensity - for every button
uint8_t				vibratoWaveKind[10] = {4,4, 4,4, 4,4, 4,4, 4,4};			//INPUT - Vibration wave kind - for every button
uint16_t*			pvibratoSpeed = vibratoSpeed;		//points to setting in use
uint8_t*			pvibratoDepth = vibratoDepth;		//points to setting in use
uint8_t*			pvibratoWaveKind = vibratoWaveKind;	//points to setting in use
uint16_t			vibratoPhacc[2] = {0, 0};			//Vibrato phase accumulator
volatile uint8_t	millisecsVibrato[2] = {0, 0};		//Vibrato millisecs

#define BTNno 9 //Number of buttons
volatile uint8_t*	buttonPort[BTNno] = {&PIND, &PIND, &PIND, &PIND, &PINC, &PINC, &PINC, &PINC, &PINC};	//Port of buttons in IC
uint8_t				buttonPin[BTNno] = {PIND0, PIND2, PIND3, PIND4, PINC0, PINC1, PINC2, PINC3, PINC4};		//Pin of buttons in IC
bool				buttonState[BTNno] = {0, 0, 0, 0, 0, 0, 0, 0, 0};										//Button states
uint8_t				millisecsButton[BTNno] = {0, 0, 0, 0, 0, 0, 0, 0, 0};									//Timers for muttons in ms for software debouncing

uint8_t		envelopeStage[2] = {4, 4};		//Tracks the current stage of the envelope
bool*		pEnvButton = &buttonState[0];	//The button envelope tracks for releasing
uint16_t	millisecsEnvelope[2] = {0, 0};	//Timer for the envelope incremented in ms

//Display stuff
uint8_t			millisecsFPS=0;						//Timer for 20FPS
bool			doRefresh = true;					//One frame refreshes after a button press
uint8_t			itemSelected[7]={0,0,0,0,0,0,0};	//Keeps track of selections across pages
const uint8_t   pageItems[7]={2,3,4,4,1,2,0};		//Number of page items is +1 of the value of this variable. pageItems[0] has a value of 1 - therefore page 0 has 2 items
uint8_t			page=0;								//Current page

bool EEPROMsaved = false;	//Turns true after save


inline void setupSerial() //serial for buade rate 9600 (UART)
{
	UBRR0H = (BRC >> 8);
	UBRR0L =  BRC;
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

inline void setupPINS() //Sets DDRs and PORT registers
{
	DDRB  = 0b00101111;
	PORTB = 0b11010000;
	
	DDRC  = 0b00000000;
	PORTC = 0b00011111;
	
	DDRD  = 0b01000000;
	PORTD = 0b00011101;
}

inline void setupTIMERS() //Sets up TIMER0 overflow interrupt and PWM
{
	OCR0A  = 0;
	TIMSK0 = (1 << TOIE0);   //overflow interrupt - timer0
	TCCR0A = (1 << COM0A1) | (1 << WGM00);   //PWM and timer modes - timer0
	
	TIMSK1 = (0 << TOIE1);   //overflow interrupt - timer1
	TCCR1A = (0 << WGM10);   //timer mode - timer1
	
	TCNT0 = 0x00;
	TCNT1 = 0xff;
	TCCR0B = (1 << CS00);   //clock source = CLK, start PWM - timer0
	TCCR1B = (0 << CS10);   //clock source = CLK, start PWM - timer1
}

inline void setupADC() //Sets up ADC for and conversion complete interrupt. Then the first conversion starts. In the interrupt the ADC output is read and a new conversion is started
{
	ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX2);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
	DIDR0 = (1 << ADC5D);
}

inline void startConversion() //Sets the bit that starts a new ADC conversion
{
	ADCSRA |= (1 << ADSC);
}

void uint8toSerial(uint8_t gg) //Sends a uint8 to serial
{
	static uint8_t level = 0;
	if (millisecsSerial > 20)
	{
		millisecsSerial = 0;
		switch (level)
		{
			case 0:
			UDR0 = gg/100+48;
			break;
			
			case 1:
			UDR0 = (gg%100)/10+48;
			break;
			
			case 2:
			UDR0 = gg%10+48;
			break;
			
			case 3:
			UDR0 = '\n';
			break;
		}
		level > 3? level = 0 : ++level;
	}
}

inline void waitForSample() //Pauses execution until next sample output
{
	wait = 1;
	while (wait);
}

void readEEPROMsettings() //Reads all settings from EEPROM and uses them
{
	eeprom_read_block((void*)waveKind, (const void*)20, 10);
	eeprom_read_block((void*)oscVolume, (const void*)30, 10);
	eeprom_read_block((void*)key, (const void*)40, 10);
	eeprom_read_block((void*)fine, (const void*)50, 10);
	eeprom_read_block((void*)phase0, (const void*)60, 10);
	
	eeprom_read_block((void*)attack, (const void*)70, 10);
	eeprom_read_block((void*)decay, (const void*)80, 10);
	eeprom_read_block((void*)sustain, (const void*)90, 10);
	eeprom_read_block((void*)release, (const void*)100, 10);
	eeprom_read_block((void*)releaseMode, (const void*)110, 10);
	
	eeprom_read_block((void*)sweepDirection, (const void*)120, 10);
	eeprom_read_block((void*)sweepSpeed, (const void*)130, 10);
	
	eeprom_read_block((void*)vibratoWaveKind, (const void*)140, 10);
	eeprom_read_block((void*)vibratoDepth, (const void*)150, 10);
	eeprom_read_block((void*)vibratoSpeed, (const void*)160, 20);
}

void updateEEPROMsettings() //Writes all settings in use to the EEPROM
{
	eeprom_update_block((const void*)waveKind, (void*)20, 10);
	eeprom_update_block((const void*)oscVolume, (void*)30, 10);
	eeprom_update_block((const void*)key, (void*)40, 10);
	eeprom_update_block((const void*)fine, (void*)50, 10);
	eeprom_update_block((const void*)phase0, (void*)60, 10);
	
	eeprom_update_block((const void*)attack, (void*)70, 10);
	eeprom_update_block((const void*)decay, (void*)80, 10);
	eeprom_update_block((const void*)sustain, (void*)90, 10);
	eeprom_update_block((const void*)release, (void*)100, 10);
	eeprom_update_block((const void*)releaseMode, (void*)110, 10);
	
	eeprom_update_block((const void*)sweepDirection, (void*)120, 10);
	eeprom_update_block((const void*)sweepSpeed, (void*)130, 10);
	
	eeprom_update_block((const void*)vibratoWaveKind, (void*)140, 10);
	eeprom_update_block((const void*)vibratoDepth, (void*)150, 10);
	eeprom_update_block((const void*)vibratoSpeed, (void*)160, 20);
}

inline void incrementTimers() //Increments all timers in use and push another input in the queue for the rolling mean of the ADC
{
	waitForSample();
	uint8_t ms = millisecs;
	millisecs = 0;
	
	for (uint8_t i=0; i < BTNno; ++i)
	millisecsButton[i] += ms;
	
	millisecsAutoButton += ms;
	millisecsSerial += ms;
	
	millisecsFPS += ms;
	
	millisecsEnvelope[0] += ms;
	millisecsEnvelope[1] += ms;
	
	millisecsSweep[0] += ms;
	millisecsSweep[1] += ms;
	
	millisecsVibrato[0] += ms;
	millisecsVibrato[1] += ms;
	
	millisecsSerial += ms;
	
	ADCinputs[inputsi] = dutyCycle; //INPsize amount of inputs for the rolling mean
	inputsi > INPsize? inputsi=0 : inputsi++;
	startConversion();
}

void silenceOsc(uint8_t oscillator) //Silences one oscillator
{
	volumecalc[oscillator] = 0;
	while (volume[oscillator])
	{
		--volume[oscillator];
		_delay_us(32);
	}
}

void silenceOscs() //Silences all oscillators
{
// 	volumecalc[0] = 0;
// 	volumecalc[1] = 0;
	while (volume[0] + volume[1])
	{
		if (volume[0]>0) --volume[0];
		if (volume[1]>0) --volume[1];
		_delay_us(160);
	}
}

void play() //Starts playing a tone with the settings in use
{
	silenceOscs();
	waitForSample();
	notePlayingSum = 0;
	for (uint8_t i=0; i<2; ++i)
	{
		phaccu[i] = pphase0[i] << 8;
		vibratoPhacc[i] = 0;

		millisecsSweep[i] = 0;
		sweepStop[i] = false;
		millisecsEnvelope[i] = 0;
		volumeRelease[i] = 0;
		volume[i] = 0;
		volumecalc[i] = 0;
		notePlaying[i] = 1;
		envelopeStage[i] = 0;
		millisecs = 0;
	}
} 

void doInput() //Does all the Potentiometer inputs depending on menu item selected
{
	if (inputActive)
	{
		if (page==2)
		{
			switch (itemSelected[2])
			{
				case 0:
				if (rollingMeanADC < 200) pwaveKind[itemSelected[0]] = 0;//WAVE KIND INPUT
				else if (rollingMeanADC < 400) pwaveKind[itemSelected[0]] = 1;
				else if (rollingMeanADC < 600) pwaveKind[itemSelected[0]] = 2;
				else if (rollingMeanADC < 800) pwaveKind[itemSelected[0]] = 3;
				else pwaveKind[itemSelected[0]] = 4;
				break;
				
				case 1:
				poscVolume[itemSelected[0]] = rollingMeanADC >> 2;		//OSCILLATOR VOLUME INPUT
				break;
				
				case 2:
				pkey[itemSelected[0]] = (rollingMeanADC) * 87 / 1024;	//NOTE INPUT
				//twordcalc[0]=0x10000/refclk*( rollingMeanADC << 4 );// *16384/1024; Sweeps from 0 to 16384Hz
				break;
				
				case 3:
				pfine[itemSelected[0]]	= rollingMeanADC >> 2;			//FINE INPUT
				break;
				
				case 4:
				pphase0[itemSelected[0]] = rollingMeanADC >> 2;			//phase0 INPUT
				break;
			}
		}
		else if (page == 3)
		{
			switch (itemSelected[3])
			{
				case 0:
				(rollingMeanADC >> 2) < 1? pattack[itemSelected[0]] = 1 : pattack[itemSelected[0]] = (rollingMeanADC >> 2);		//ATTACK INPUT
				break;
				
				case 1:
				pdecay[itemSelected[0]] = (rollingMeanADC >> 2);			//DECAY INPUT
				if (pdecay[itemSelected[0]] < 2) pdecay[itemSelected[0]] = 2;
				break;
				
				case 2:
				psustain[itemSelected[0]] = (rollingMeanADC >> 2);			//SUSTAIN INPUT
				if (psustain[itemSelected[0]] < 2) psustain[itemSelected[0]] = 0;
				break;
				
				case 3:
				prelease[itemSelected[0]] = (rollingMeanADC >> 2);			//RELEASE INPUT
				if (prelease[itemSelected[0]] < 5) prelease[itemSelected[0]] = 5;
				break;
				
				case 4:
				preleaseMode[itemSelected[0]] = rollingMeanADC >> 9;		//RELEASE MODE INPUT
				break;
			}
		}
		else if (page == 4)
		{
			switch (itemSelected[4])
			{
				case 0:
				psweepDirection[itemSelected[0]] = rollingMeanADC >> 8;		//SWEEP DIRECTION INPUT
				break;
				
				case 1:
				psweepSpeed[itemSelected[0]] = rollingMeanADC >> 2;		//SWEEP SPEED INPUT
				break;
				
			}
			
			
		}
		else if (page == 5)
		{
			switch (itemSelected[5])
			{
				case 0:
				if (rollingMeanADC < 200) pvibratoWaveKind[itemSelected[0]] = 0;			//VIBRATO WAVE KIND INPUT
				else if (rollingMeanADC < 400) pvibratoWaveKind[itemSelected[0]] = 1;
				else if (rollingMeanADC < 600) pvibratoWaveKind[itemSelected[0]] = 2;
				else if (rollingMeanADC < 800) pvibratoWaveKind[itemSelected[0]] = 3;
				else pvibratoWaveKind[itemSelected[0]] = 4;
				break;
				
				case 1:
				pvibratoDepth[itemSelected[0]] = rollingMeanADC >> 2;		//VIBRATO DEPTH INPUT
				break;
				
				case 2:
				pvibratoSpeed[itemSelected[0]] = rollingMeanADC;			//VIBRATO SPEED INPUT
				break;
			}
		}
	}
} 

void doButtonPointers(const uint8_t button) //Changes pointers in use to one of the 5 sets of pointers for each "note button"
{
	pwaveKind = waveKind + button * 2;			
	poscVolume = oscVolume + button * 2;			
	pkey = key + button * 2;						
	pfine = fine + button * 2;					
	pphase0 = phase0 + button * 2;				

	pattack = attack + button * 2;
	pdecay = decay + button * 2;
	psustain = sustain + button * 2;
	prelease = release + button * 2;
	preleaseMode = releaseMode + button * 2;

	psweepDirection = sweepDirection + button * 2;
	psweepSpeed = sweepSpeed + button * 2;

	pvibratoWaveKind = vibratoWaveKind + button * 2;
	pvibratoDepth = vibratoDepth + button * 2;		
	pvibratoSpeed = vibratoSpeed + button * 2;	
	
	pEnvButton = &buttonState[button + 4];	
}

void action(uint8_t actionNum) //Every different button press action is in this function
{
	switch (actionNum)
	{
		case 0: //button up
		//UDR0 = 'U';
		doRefresh = true;
		EEPROMsaved = false;
		inputActive = false;
		if (itemSelected[page] > 0) --itemSelected[page];
		break;
		
		case 1: //button down
		//UDR0 = 'D';
		//EEPROMsaved = false;
		doRefresh = true;
		inputActive = false;
		if (itemSelected[page] < pageItems[page]) ++itemSelected[page];
		break;
		
		case 2: //button right
		//UDR0 = 'R';
		doRefresh = true;
		if (page == 0) 
		{
 			if (itemSelected[0] < 2)
				page = 1;
 			else if (EEPROMsaved == false)
			{
				updateEEPROMsettings();
				EEPROMsaved = true;
			}
		}
		else if (page == 1)
		{
			switch (itemSelected[page])
			{
				case 0:
				page = 2;
				break;
				
				case 1:
				page = 3;
				break;
				
				case 2:
				page = 4;
				break;
				
				case 3:
				page = 5;
				break;
			}	
		}
		else if (page <= 5)
		{
			inputActive ^= true;		//Toggle input!
		}
		break;
		
		case 3: //button left
		//UDR0 = 'L';
		doRefresh = true;
		inputActive = false;
		if (page == 1 || page == 6)
			page = 0;
		else if (page >= 2 && page <=5)
			page = 1;
		break;
		
		case 4: //button 0
		//UDR0 = '0';
		if (pwaveKind != waveKind)
		{
			inputActive = false;
			doButtonPointers(0);
		}
		play();
		break;
		
		case 5: //button 1
		//UDR0 = '1';
		if (pwaveKind != waveKind + 2)
		{
			inputActive = false;
			doButtonPointers(1);
		}
		play();
		break;
		
		case 6: //button 2
		//UDR0 = '2';
		if (pwaveKind != waveKind + 4)
		{
			inputActive = false;
			doButtonPointers(2);
		}
		play();
		break;
		
		case 7: //button 3
		//UDR0 = '3';
		if (pwaveKind != waveKind + 6)
		{
			inputActive = false;
			doButtonPointers(3);
		}
		play();
		break;
		
		case 8: //button 4
		//UDR0 = '4';
		if (pwaveKind != waveKind + 8)
		{
			inputActive = false;
			doButtonPointers(4);
		}
		play();
		break;
	}
} 

void buttonAction(volatile const uint8_t* PINX, const uint8_t PINXno) //Links microcontroller buttons to their actions
{
	for (uint8_t i=0; i<BTNno; ++i)
	{
		if (PINX == buttonPort[i] && PINXno == buttonPin[i])
		{
			action(i);
		}
	}
} 

void buttonCheck(bool& bState, uint8_t& timer, volatile const uint8_t* PINX, const uint8_t PINXno, const uint8_t msDebounce = 50) //Software debounced button check
{
	waitForSample();
	if (((*PINX & (1 << PINXno))) == 0)
	{
		if (!bState && timer > msDebounce)
		{
			bState = 1;
			timer= 0;
			
			//UDR0 = 'P';
			buttonAction(PINX, PINXno);
		}
	}
	else
	{
		if (bState && (timer > msDebounce))
		{
			bState = 0;
			timer = 0;
		}
	}
}

inline void CalcRollingMeanADC() //Calculating rolling mean for filtering(Low Pass) the potentiometer ADC input
{
	rollingMeanADC=0;
	for (uint8_t i = 0; i < INPsize; ++i)
		rollingMeanADC += ADCinputs[i];
	rollingMeanADC = rollingMeanADC >> 5;
}

int16_t fineAdj(uint8_t oscillator) //adjust the notes Fine
{
	if (pkey[oscillator] == 0)
	{
		if (pfine[oscillator] <= 120)
		return (-48 * (120 - pfine[oscillator]))/120;
		else if (pfine[oscillator] < 135)
		return 0L;
		else
		return (48 * (pfine[oscillator] - 135))/120;
	}
	else if (pkey[oscillator] < 87)
	{
		if (pfine[oscillator] <= 120)
			return -static_cast<int16_t>(pgm_read_word(&(twordNotes[pkey[oscillator]+1])) - pgm_read_word(&(twordNotes[pkey[oscillator]]))) * (120 - pfine[oscillator]) / 120;
		else if (pfine[oscillator] < 135)
			return 0L;
		else
			return (pgm_read_word(&(twordNotes[pkey[oscillator]+1])) - pgm_read_word(&(twordNotes[pkey[oscillator]]))) * (pfine[oscillator] - 135) / 120;
	}
	else
	{
		if (pfine[oscillator] <= 120)
			return (-420 * (120 - pfine[oscillator]))/120;
		else if (pfine[oscillator] < 135)
			return 0L;
		else
			return (420 * (pfine[oscillator] - 135))/120;
	}
}


uint16_t ADCtoTWORD(uint16_t ADC10bit) //Converts from ADC note to TWORD note
{
	if (ADC10bit < 93)
		return ( ADC10bit * 26 ) / 93;//
	else if (ADC10bit < 186)
		return ( ADC10bit * 52 ) / 186;
	else if (ADC10bit < 279)
		return ( (ADC10bit-93) * 104 ) / 186;
	else if (ADC10bit < 372)
		return ( (ADC10bit-186) * 207 ) / 186;
	else if (ADC10bit < 465)
		return ( (ADC10bit-279) * 414UL ) / 186;
	else if (ADC10bit < 558)
		return ( (ADC10bit-372) * 826UL ) / 186;
	else if (ADC10bit < 651)
		return ( (ADC10bit-465) * 1651UL ) / 186;
	else if (ADC10bit < 744)
		return ( (ADC10bit-558) * 3301UL ) / 186;
	else if (ADC10bit < 837)
		return ( (ADC10bit-651) * 6600UL ) / 186;
	else if (ADC10bit < 930)
		return ( (ADC10bit-744) * 13200UL ) / 186;
	else
		return ( (ADC10bit-838) * 26400UL ) / 194;
}

uint16_t TWORDtoADC(uint16_t TWORD16bit) //Converts from TWORD note to ADC note
{
	if (TWORD16bit < 26)
	return ( TWORD16bit * 93 ) / 26;
	else if (TWORD16bit < 52)
	return ( TWORD16bit * 186 ) / 52;
	else if (TWORD16bit < 104)
	return ( (TWORD16bit * 93) / 52 ) + 93;
	else if (TWORD16bit < 207)
	return ( (TWORD16bit * 93) / 104) + 186;
	else if (TWORD16bit < 414)
	return ( (TWORD16bit * 93) / 207) + 279;
	else if (TWORD16bit < 826)
	return ( (TWORD16bit * 93UL) / 414) + 372;
	else if (TWORD16bit < 1651)
	return ( (TWORD16bit * 93UL) / 826) + 465;
	else if (TWORD16bit < 3301)
	return ( (TWORD16bit * 93UL) / 1651) + 558;
	else if (TWORD16bit < 6600)
	return ( (TWORD16bit * 93UL) / 3301) + 651;
	else if (TWORD16bit < 13200)
	return ( (TWORD16bit * 93UL) / 6600) + 744;
	else
	return ( (TWORD16bit * 93UL) / 13200) + 838;
}

void doMainNote(uint8_t oscillator) //Uses key and fine settings to define the primary tuning word - this tword might be altered by sweep and vibrato effects before finally being output
{
	waitForSample();
	
// 	if (pnoteSource[oscillator])
// 	{
		twordcalc[oscillator] = pgm_read_word(&(twordNotes[pkey[oscillator]])) + fineAdj(oscillator);
// 	}
// 	else
// 	{
// 		twordcalc[oscillator] = ADCtoTWORD(0x10000/refclk*( pnoteADC[oscillator] << 4 )) + fineAdj(oscillator);
// 		
// 	}
} 

void doSweep(uint8_t oscillator) //Sweep effect
{
	waitForSample();
	
	if (psweepSpeed[oscillator] > 0 && !sweepStop[oscillator])
	switch (psweepDirection[oscillator])
	{
		case 0:
		if ((static_cast<uint32_t>(millisecsSweep[oscillator]) * psweepSpeed[oscillator] >> 7) > TWORDtoADC(twordcalc[oscillator]))
		{
			silenceOsc(oscillator);
			envelopeStage[oscillator] = 4;
			notePlaying[oscillator] = 0;
			sweepStop[oscillator] = true;
		}
		twordcalc[oscillator] = ADCtoTWORD( TWORDtoADC(twordcalc[oscillator]) - (static_cast<uint32_t>(millisecsSweep[oscillator]) * psweepSpeed[oscillator] >> 7) );
		break;
		
		case 1:
		if (TWORDtoADC(twordcalc[oscillator]) + (static_cast<uint32_t>(millisecsSweep[oscillator]) * psweepSpeed[oscillator] >> 7) > 1024)
		{
			silenceOsc(oscillator);
			envelopeStage[oscillator] = 4;
			notePlaying[oscillator] = 0;
			sweepStop[oscillator] = true;
		}
		twordcalc[oscillator] = ADCtoTWORD( TWORDtoADC(twordcalc[oscillator]) + (static_cast<uint32_t>(millisecsSweep[oscillator]) * psweepSpeed[oscillator] >> 7) );
		break;
		
		case 2:
		if (93 - (millisecsSweep[oscillator] * psweepSpeed[oscillator] >> 8) > TWORDtoADC(twordcalc[oscillator]))
			twordcalc[oscillator] = 0;
		else if ((millisecsSweep[oscillator] * psweepSpeed[oscillator] >> 8) < 93)
			twordcalc[oscillator] = ADCtoTWORD( TWORDtoADC(twordcalc[oscillator]) - 93 + (millisecsSweep[oscillator] * psweepSpeed[oscillator] >> 8));
		else 
			sweepStop[oscillator] = true;
		break;
		
		case 3:
		if ((millisecsSweep[oscillator] * psweepSpeed[oscillator] >> 8) < 93)
			twordcalc[oscillator] = ADCtoTWORD( TWORDtoADC(twordcalc[oscillator]) + 93 - (millisecsSweep[oscillator] * psweepSpeed[oscillator] >> 8));
		else 
			sweepStop[oscillator] = true;
		break;
	}
}

void doVibrato(uint8_t oscillator) //Vibrato effect
{
	waitForSample();
	
	if (pvibratoSpeed[oscillator] > 1)
	{
		uint8_t ms = millisecsVibrato[oscillator];
		millisecsVibrato[oscillator] = 0;
		
		if (pvibratoSpeed[oscillator] < 160)
		vibratoPhacc[oscillator] += ((pvibratoSpeed[oscillator] >> 4)) * ms;
		else if (pvibratoSpeed[oscillator] < 352)
		vibratoPhacc[oscillator] += (((pvibratoSpeed[oscillator] - 160) >> 3) + 10) * ms;
		else if (pvibratoSpeed[oscillator] < 512)
		vibratoPhacc[oscillator] += (((pvibratoSpeed[oscillator] - 352) >> 2) + 34) * ms;
		else if (pvibratoSpeed[oscillator] < 680)
		vibratoPhacc[oscillator] += (((pvibratoSpeed[oscillator] - 512) >> 1) + 74) * ms;
		else if (pvibratoSpeed[oscillator] < 850)
		vibratoPhacc[oscillator] += (((pvibratoSpeed[oscillator] - 680) << 2) + 158) * ms;
		else
		vibratoPhacc[oscillator] += (((pvibratoSpeed[oscillator] - 850) << 3) + 838) * ms;
		
		twordcalc[oscillator] = ADCtoTWORD( TWORDtoADC(twordcalc[oscillator]) + (((pgm_read_byte( &waveTable[ pvibratoWaveKind[oscillator] ][ vibratoPhacc[oscillator] >> 8 ] ) - 128) * pvibratoDepth[oscillator]) / 255));
	}
}

void doEnvelope(uint8_t oscillator, bool* bState) //Envelope
{
	waitForSample();
	
	switch (envelopeStage[oscillator])
	{
		case 0:
		if ((*bState) && ((millisecsEnvelope[oscillator] << 4) / pattack[oscillator] < 255))
		{
			volumecalc[oscillator] = (millisecsEnvelope[oscillator] << 4) / pattack[oscillator];		// millisecsEnvelope[oscillator] / (attack/16)
		}
		else if (*bState)
		{
			volumecalc[oscillator] = 255;
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		else
		{
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		break;
	
		case 1:
		if ((*bState) && (volumecalc[oscillator] > psustain[oscillator]) && (psustain[oscillator] > 0))
		{
			volumecalc[oscillator] = 255 - ((millisecsEnvelope[oscillator] * 16) / pdecay[oscillator]) ;	// 255 - millisecsEnvelope[oscillator] / (decay/16);
		}
		else if ((*bState) && (psustain[oscillator] > 0))
		{
			volumecalc[oscillator] = psustain[oscillator];
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		else
		{
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		break;
	
		case 2:
		if (!(*bState) || (psustain[oscillator] == 0))
		{
			++envelopeStage[oscillator];
			millisecsEnvelope[oscillator] = 0;
		}
		break;
	
		case 3:
		if (volumecalc[oscillator] <= 1)
		{
			volumecalc[oscillator] = 0;
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
			if (!volumeRelease[oscillator]) volumeRelease[oscillator] = volumecalc[oscillator];
		
			if (!preleaseMode[oscillator])
				if (volumeRelease[oscillator] > ((millisecsEnvelope[oscillator] << 4) / prelease[oscillator]))
					volumecalc[oscillator] = volumeRelease[oscillator] - ((millisecsEnvelope[oscillator] << 4) / prelease[oscillator]); //Linear release
				else volumecalc[oscillator] = 0;
			else
				volumecalc[oscillator] = (volumeRelease[oscillator] * prelease[oscillator]) / ((millisecsEnvelope[oscillator] >> 1) + prelease[oscillator]) - (millisecsEnvelope[oscillator]/1000); //Non linear release
		}
		break;
	}
}

inline void updateNotes() //Updates note in use for the sampler
{
	waitForSample();
	tword[0] = twordcalc[0];
	volume[0] = volumecalc[0] * poscVolume[0] / 510UL;
	
	tword[1] = twordcalc[1];
	volume[1] = volumecalc[1] * poscVolume[1] / 510UL;
	
	notePlayingSum = notePlaying[0] + notePlaying[1];
}

bool BitmapXYaccess(const uint8_t* bitmap, const uint8_t& bwidth, const uint8_t& x = 0, const uint8_t& y = 0) //Returns the 0 or 1 of the bitmap on its x,y position
{
	return (pgm_read_byte( &bitmap[x + (y >> 3) * bwidth] ) >> (y % 8)) & 0x01;
}

void nokia_lcd_write_bitmap(const uint8_t* bitmap, const uint8_t& bwidth, const uint8_t& bheight, const uint8_t& x = 0, const uint8_t& y = 0) //Writes a bitmap of dimensions bwidth,bheight on the display, starting at x,y
{
	for (uint8_t i = 0; i < bwidth; ++i)
	{
		for (uint8_t j = 0; j < bheight; ++j)
		{
			nokia_lcd_set_pixel(i+x,j+y, BitmapXYaccess(bitmap, bwidth, i, j) );
		}
	}
}

uint16_t _map(uint16_t X, uint16_t A, uint16_t B, uint16_t C, uint16_t D) //Standard mapping function
{
	uint16_t Y = 0;
	if (X < A)
		C < D? Y = C : Y = D;
	else if (X > B)
		C < D? Y = D : Y = C;
	else
		Y = (X-A)*(D-C)/(B-A) + C;
	return Y;
}

inline void introDisplay() //Does the intro of the display after power on
{
	nokia_lcd_clear();
	nokia_lcd_write_bitmap(wf_Bitmap, wf_Width, wf_Height);
	nokia_lcd_render();
	_delay_ms(3000);
	
	nokia_lcd_clear();
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("ZNEAR", 3);
	nokia_lcd_set_cursor(5, 23);
	nokia_lcd_write_string("Synth", 3);
	nokia_lcd_render();
	_delay_ms(3000);
}

void drawCursor(uint8_t xpos, uint8_t fontSize) //Draws the cursor on the display
{
	nokia_lcd_set_cursor(xpos, itemSelected[page] * (8 * fontSize));
	if (inputActive) nokia_lcd_write_char(0x80 , fontSize); //20- ,80-?, 81-? void
	else nokia_lcd_write_char(0x81 , fontSize); //20- ,80-?, 81-? void
}

void drawLegend() //Draws the oscillator legend
{
	nokia_lcd_write_bitmap(leg_Bitmap, leg_Width, leg_Height);
	if (itemSelected[0] == 0)
		nokia_lcd_write_bitmap(leg1_Bitmap, leg1_Width, leg1_Height, 0, 38);
	else
		nokia_lcd_write_bitmap(leg2_Bitmap, leg2_Width, leg2_Height, 0, 38);
}

void drawNote(const uint8_t& oscillator) //draws the note in use in the NTE field
{	
	if (pkey[oscillator] < 3)
		nokia_lcd_write_char('0', 1);
	else
		nokia_lcd_write_char(((pkey[oscillator] - 3)/12 + 1)+48, 1);
	
	switch (pkey[oscillator]%12)
	{
		case 0:
		nokia_lcd_write_string("A",1);
		break;
		
		case 1:
		nokia_lcd_write_string("A#",1);
		break;
		
		case 2:
		nokia_lcd_write_string("B",1);
		break;
		
		case 3:
		nokia_lcd_write_string("C",1);
		break;
		
		case 4:
		nokia_lcd_write_string("C#",1);
		break;
		
		case 5:
		nokia_lcd_write_string("D",1);
		break;
		
		case 6:
		nokia_lcd_write_string("D#",1);
		break;
		
		case 7:
		nokia_lcd_write_string("E",1);
		break;
		
		case 8:
		nokia_lcd_write_string("F",1);
		break;
		
		case 9:
		nokia_lcd_write_string("F#",1);
		break;
		
		case 10:
		nokia_lcd_write_string("G",1);
		break;
		
		case 11:
		nokia_lcd_write_string("G#",1);
		break;
		
	}
	
}

void draw8bitBar(const uint8_t& oscillator, const uint8_t* target) //Draws a bar for certain setting on the menu
{
	nokia_lcd_write_char(0x84, 1);
	for (uint8_t i=0; i<6; ++i)
	{
		if (target[oscillator] > i*42+21)
			nokia_lcd_write_char(0x83, 1);
		else
			nokia_lcd_write_char(0x82, 1);
	}
	nokia_lcd_write_char(0x85, 1);
}

inline void drawRmo(const uint8_t& oscillator) //Draws the release mode used
{
	if (preleaseMode[oscillator])
		nokia_lcd_write_string("NON-LIN", 1);
	else
		nokia_lcd_write_string("LINEAR", 1);
}

inline void drawSweepDir(const uint8_t& oscillator) //Draws the sweep direction used
{
	switch (psweepDirection[oscillator])
	{
		case 0:
		nokia_lcd_write_string("<--", 1);
		nokia_lcd_write_char(0x86, 1);
		break;
		
		case 1:
		nokia_lcd_write_char(0x86, 1);
		nokia_lcd_write_string("-->", 1);
		break;
		
		case 2:
		nokia_lcd_write_string("-->", 1);
		nokia_lcd_write_char(0x86, 1);
		break;
		
		case 3:
		nokia_lcd_write_char(0x86, 1);
		nokia_lcd_write_string("<--", 1);
		break;
	}
}

void drawWaveKind(const uint8_t& oscillator, const uint8_t* target) //Draws the wave kind used
{
	switch (target[oscillator])
	{
		case 0:
		nokia_lcd_write_char(' ', 1);
		nokia_lcd_write_bitmap(rand_Bitmap, wave_Width, wave_Height, 42, 0);
		break;
		
		case 1:
		nokia_lcd_write_char(' ', 1);
		nokia_lcd_write_bitmap(square_Bitmap, wave_Width, wave_Height, 42, 0);
		break;
		
		case 2:
		nokia_lcd_write_char(' ', 1);
		nokia_lcd_write_bitmap(saw_Bitmap, wave_Width, wave_Height, 42, 0);
		break;
		
		case 3:
		nokia_lcd_write_char(' ', 1);
		nokia_lcd_write_bitmap(triangle_Bitmap, wave_Width, wave_Height, 42, 0);
		break;
		
		case 4:
		nokia_lcd_write_char(' ', 1);
		nokia_lcd_write_bitmap(sin_Bitmap, wave_Width, wave_Height, 42, 0);
		break;
	}
}

inline void drawFine(const uint8_t& oscillator) //Draws the fine adjustment used in the FIN field
{
	nokia_lcd_write_char(' ', 1);
	for (uint8_t i=0; i<6; ++i)
	{
		if (i<3)
		{
			if (i==2 && pfine[oscillator] <= 120)
				nokia_lcd_write_char(0x83, 1);
			else if (pfine[oscillator] < i*42+21)
				nokia_lcd_write_char(0x83, 1);
			else
				nokia_lcd_write_char(0x82, 1);
		}
		else
		{
			if (i==3 && pfine[oscillator] >= 135)
				nokia_lcd_write_char(0x83, 1);
			else if (pfine[oscillator] > i*42+21)
				nokia_lcd_write_char(0x83, 1);
			else
				nokia_lcd_write_char(0x82, 1);
		}
	}
	for (uint8_t i=0; i<7; ++i)
		nokia_lcd_set_pixel(59, 24 + i, 1);
}

void drawPhase(const uint8_t& oscillator) //Draws the initial phase of the wave that is used in the field PHA
{
	uint16_t temp = pphase0[oscillator] * 360UL / 255;
	nokia_lcd_write_char(temp/100+48, 1);
	temp %= 100;
	nokia_lcd_write_char(temp/10+48, 1);
	temp %= 10;
	nokia_lcd_write_char(temp+48, 1);
} 

#define MENU_X 18 
void doDisplay() //Draws on the display depending on what page it is on
{
	waitForSample();
		
	nokia_lcd_clear();
				
	switch (page)
	{
		case 0:
		nokia_lcd_set_cursor(12, 0);
		nokia_lcd_write_string("OSC1", 2);
		nokia_lcd_set_cursor(12, 16);
		nokia_lcd_write_string("OSC2", 2);
		nokia_lcd_set_cursor(12, 32);
		if (EEPROMsaved)
			nokia_lcd_write_string("OK", 2);
		else
			nokia_lcd_write_string("SAVE", 2);
			
 		drawCursor(1, 2);
		break;
		//////////////////////////////////////////////////////////////////////////
		case 1:
 		drawLegend();
			
		nokia_lcd_set_cursor(MENU_X,0);
		nokia_lcd_write_string("NOTE", 1);
			
		nokia_lcd_set_cursor(MENU_X,8);
		nokia_lcd_write_string("ENVELOPE", 1);
			
		nokia_lcd_set_cursor(MENU_X,16);
		nokia_lcd_write_string("SWEEP", 1);
			
		nokia_lcd_set_cursor(MENU_X,24);
		nokia_lcd_write_string("VIBRATO", 1);
			
 		drawCursor(13, 1);
		break;
		//////////////////////////////////////////////////////////////////////////
		case 2:
		drawLegend();
			
		nokia_lcd_set_cursor(MENU_X,0);
		nokia_lcd_write_string("WAV", 1);
		drawWaveKind(itemSelected[0], pwaveKind);
			
		nokia_lcd_set_cursor(MENU_X,8);
		nokia_lcd_write_string("VOL", 1);
		draw8bitBar(itemSelected[0], poscVolume);
			
		nokia_lcd_set_cursor(MENU_X,16);
		nokia_lcd_write_string("NTE ", 1);
		drawNote(itemSelected[0]);
			
		nokia_lcd_set_cursor(MENU_X,24);
		nokia_lcd_write_string("FIN", 1);
		drawFine(itemSelected[0]);
			
		nokia_lcd_set_cursor(MENU_X,32);
		nokia_lcd_write_string("PHA ", 1);
		drawPhase(itemSelected[0]);
		nokia_lcd_write_string("deg", 1);
			
 		drawCursor(13, 1);
		break;
		//////////////////////////////////////////////////////////////////////////
		case 3:
		drawLegend();
			
		nokia_lcd_set_cursor(MENU_X,0);
		nokia_lcd_write_string("ATK", 1);
		draw8bitBar(itemSelected[0], pattack);
			
		nokia_lcd_set_cursor(MENU_X,8);
		nokia_lcd_write_string("DEC", 1);
		draw8bitBar(itemSelected[0], pdecay);
			
		nokia_lcd_set_cursor(MENU_X,16);
		nokia_lcd_write_string("SUS", 1);
		draw8bitBar(itemSelected[0], psustain);
			
		nokia_lcd_set_cursor(MENU_X,24);
		nokia_lcd_write_string("REL", 1);
		draw8bitBar(itemSelected[0], prelease);
			
		nokia_lcd_set_cursor(MENU_X,32);
		nokia_lcd_write_string("Rmo", 1);
		nokia_lcd_write_char(' ', 1);
		drawRmo(itemSelected[0]);
			
 		drawCursor(13, 1);
		break;
		//////////////////////////////////////////////////////////////////////////
		case 4:
		drawLegend();
		nokia_lcd_set_cursor(MENU_X,0);
		nokia_lcd_write_string("DIR  ", 1);
		drawSweepDir(itemSelected[0]);
			
		nokia_lcd_set_cursor(MENU_X,8);
		nokia_lcd_write_string("SPE", 1);
		draw8bitBar(itemSelected[0], psweepSpeed);
			
 		drawCursor(13, 1);
		break;
		//////////////////////////////////////////////////////////////////////////
		case 5:
		drawLegend();
		nokia_lcd_set_cursor(MENU_X,0);
		nokia_lcd_write_string("WAV", 1);
		drawWaveKind(itemSelected[0], pvibratoWaveKind);
			
		nokia_lcd_set_cursor(MENU_X,8);
		nokia_lcd_write_string("DEP", 1);
		draw8bitBar(itemSelected[0], pvibratoDepth);
			
		nokia_lcd_set_cursor(MENU_X,16);
		nokia_lcd_write_string("SPE", 1);
		uint8_t temp = (pvibratoSpeed[itemSelected[0]]) >> 2;
		draw8bitBar(itemSelected[0], &temp-itemSelected[0]);
			
 		drawCursor(13, 1);
		break;
		//////////////////////////////////////////////////////////////////////////
// 		case 6:
// 		break;
			
	}
nokia_lcd_render();
}

int main(void)
{
 	setupSerial();
 	
 	setupPINS();
	
	setupADC();
	startConversion();
	
	setupTIMERS();
	
	
	nokia_lcd_init();
	//introDisplay();
	
	readEEPROMsettings();
	
	sei();
	
	doDisplay();
	
	
	
	while(1)
    {		
		incrementTimers();
		
		if ((millisecsFPS > 200 && (inputActive || (!notePlayingSum))) || doRefresh)
		{
			millisecsFPS = 0;
 			doDisplay();
			doRefresh = false;
		}
	
		for (uint8_t i=0; i<BTNno; ++i)
		{
			buttonCheck(buttonState[i], millisecsButton[i], buttonPort[i], buttonPin[i]);
		}
		
		CalcRollingMeanADC();
		//ADCtoSerial();      //For Debugging purposes
		
		doInput();
		
		
		doMainNote(0);
		doMainNote(1);
		
		doEnvelope(0, pEnvButton);
		doEnvelope(1, pEnvButton);
		
		doSweep(0);
		doSweep(1);
		
		doVibrato(0);
		doVibrato(1);

		
		updateNotes();
		
		//uint8toSerial(notePlaying[0]);  //Debugging purpose
	}
}

ISR (TIMER0_OVF_vect) //This is the interrupt routine for the DDS and after calculations it occupies almost 1/3 of the total CPU time
{
  	if (notePlayingSum)
	{
 		phaccu[0] += tword[0];
 		osc[0] = (pgm_read_byte( &waveTable[ pwaveKind[0] ][ phaccu[0] >> 8 ]));
		 
		phaccu[1] += tword[1];
 		osc[1] = (pgm_read_byte( &waveTable[ pwaveKind[1] ][ phaccu[1] >> 8 ]));
		
		OCR0A = (( osc[0] * volume[0] ) >> 8) + (( osc[1] * volume[1] ) >> 8);
	}
	
	smallTimer++; //increment small timer
	if(smallTimer > 40)
	{				
		smallTimer = 0;
		++millisecs; //increment milliseconds every 40 small timer ticks
	} 
	wait = 0;
}

ISR(ADC_vect) //Interrupt subroutine for extracting the ADC converted value every time the conversion is completed
{
	dutyCycle = ADC;
}

