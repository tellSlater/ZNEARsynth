/*
 * CPPFile1.cpp
 *
 * Created: 9/10/2020 17:18:17
 *  Author: tellSlater
 */

#define F_CPU   20000000
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

extern volatile uint16_t	refclk;		// refclk=3906 = 20MHz / 510 ... increase to compensate for higher pitch
// 510 comes from formula for calculating phase correct PWM frequency

extern volatile uint16_t	phaccu[2];
extern volatile uint8_t		phase0[2];
extern volatile uint16_t	tword[2];
extern volatile uint8_t		key[2];
extern volatile uint8_t		osc[2];

extern volatile uint8_t	waveKind[2];

extern volatile uint16_t	dutyCycle;
extern volatile uint16_t	ADCinputs[INPsize];
extern volatile uint8_t	inputsi;
extern volatile uint32_t	rollingMeanADC;

extern volatile uint8_t	smallTimer;					//Counts 1/40ms - every timer0 overflow interrupt
extern volatile uint16_t	millisecs;				//Counts 1 ms every 40 smallTimer clicks
extern volatile uint16_t	millisecsLast;			//Counts ms - used to mark last time note calculations were made
extern volatile uint16_t	millisecsAutoButton;	//Counts ms - used for AutoButton function
extern volatile uint16_t	millisecsSerial;		//Counts ms - used for timing some serial debugging commands

extern volatile uint8_t	notePlaying[2];			//Boolean  - enables oscillators if a note is playing
extern volatile uint8_t	notePlayingSum;			//Boolean  - enables oscillators if a note is playing

extern volatile uint8_t	volume[2];
extern volatile uint8_t	volumeRelease[2];

extern volatile uint8_t		mainNoteADC[2];		//INPUT - main note
extern volatile float		fine[2];			//INPUT - fine
extern volatile uint16_t	finalNoteADC[2];	//Note after sweep and vibrato are applied

extern volatile uint8_t	attack[2];			//INPUT - Envelope attack
extern volatile uint8_t	decay[2];			//INPUT - Envelope decay
extern volatile uint8_t	sustain[2];			//INPUT - Envelope sustain
extern volatile uint8_t	release[2];			//INPUT - Envelope release
extern volatile uint8_t	releaseMode[2];		//INPUT - Release mode (0->linear or 1->nonlinear)

extern volatile uint8_t		envelopeStage[2];		//Tracks the current stage of the envelope
extern volatile uint16_t	millisecsEnvelope[2];	//Timer for the envelope incremented in ms

extern volatile uint8_t		sweepSpeed[2];			//INPUT - Sweep speed (0->one semi a second, 255->fast!)
extern volatile uint8_t		sweepDirection[2];		//INPUT - Sweep direction (0->down, 1->up, 2->from down, 3->from up)
extern volatile bool		sweepStop[2];
extern volatile uint16_t	millisecsSweep[2];		//Timer for the sweep incremented in ms

extern volatile uint16_t  vibratoSpeed[2];		//INPUT - Vibration speed
extern volatile uint8_t   vibratoDepth[2];		//INPUT - Vibration intensity
extern volatile uint8_t   vibratoWaveKind[2];	//INPUT - Vibration
extern volatile uint16_t  vibratoPhacc[2];		//Vibrato phase accumulator

extern volatile uint8_t*	buttonPort[4];
extern uint8_t				buttonPin[4];
extern bool					buttonState[4];
extern uint8_t				millisecsButton[4];

//Display stuff
extern volatile uint8_t   millisecsFPS;				// Timer for 20FPS
extern volatile uint8_t   itemSelected[3];			// Keeps track of selections across pages
extern volatile const uint8_t   pageItems[3];		// Number of page items is +1 of the value of this variable. pageItems[0] has a value of 1 - therefore page 0 has 2 items
extern volatile uint8_t   page;						// Current page

void prescaleCLK256()
{
	CLKPR = (1 << CLKPCE); // Prescale clock by 1/256
	CLKPR = (1 << CLKPS3);
}

void setupSPI()
{
	SPCR  = 0b11110000;
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
	_delay_ms(20);
}

inline void setNoteFreq(uint16_t Freq, uint8_t oscillator)
{
	tword[oscillator] = 0x10000*Freq/refclk;
}

void setNoteADC(uint16_t inp10bit, uint8_t oscillator)
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

void AutoButton(volatile uint8_t& button, uint16_t onDuration = 5000, uint16_t offDuration = 5000) //emulates button push - durations in ms
{
	
	if (millisecsAutoButton > onDuration && button == 1)
	{
		button = 0;
		millisecsAutoButton = 0;
	}
	else if (millisecsAutoButton > offDuration && button == 0)
	{
		
		button = 1;
		millisecsAutoButton = 0;
		
		while (volume[0])
		{
			--volume[0];
			_delay_us(28);
		}
		while (volume[1])
		{
			--volume[1];
			_delay_us(28);
		}
		
		for (uint8_t i=0; i<2; ++i)
		{
			phaccu[i] = phase0[i];
			vibratoPhacc[i] = 0;
			
			millisecsEnvelope[i] = 0;
			envelopeStage[i] = 0;
			millisecsSweep[i] = 0;
			sweepStop[i] = false;
 			waveKind[i]>=4 ? waveKind[i]=0 : ++waveKind[i];
			volumeRelease[i] = 0;
			notePlaying[i] = 1;
		}
	}
}