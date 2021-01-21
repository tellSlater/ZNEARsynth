/*
 * CPPFile1.cpp
 *
 * Created: 9/10/2020 17:41:14
 *  Author: tellSlater
 */ 


void setupSPI();

void setADC(uint8_t ADCno);		 //Sets ADMUX last 3 bits that choose the conversion's target. Acceptable values for ADCno: 0..7 for ADC0...ADC7

void sweepADC(	uint8_t ADCno1, uint8_t ADCno2, uint8_t ADCno3=0, uint8_t ADCno4=0,			//Sweeps between several ADC pins to be converted. Should be used in ADC interrupt
uint8_t ADCno5=0, uint8_t ADCno6=0, uint8_t ADCno7=0, uint8_t ADCno8=0 );

void AutoButton(volatile uint8_t& button, uint16_t onDuration = 10000, uint16_t offDuration = 3000); //emulates button push - durations in ms

void ADCtoSerial(); //outputs the ADC to serial TxD

void BYTEtoSerial(uint8_t gj); //Sends byte to serial

inline void setNoteFreq(uint16_t Freq, uint8_t oscillator); 

void setNoteADC(uint16_t inp10bit, uint8_t oscillator);

void doSweepADC(uint8_t oscillator);

