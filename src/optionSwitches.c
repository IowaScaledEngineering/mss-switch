#include <stdint.h>
#include "optionSwitches.h"

#define ADC_AVERAGING_CYCLES 8

void initializeOptions(DebounceState8_t* optionsDebouncer)
{
	uint8_t bits = 0;
	
	ADMUX  = 0b00100000;  // VCC reference voltage; left-adjust; ADC0
	ADCSRA = 0b10000111;  // ADC enabled; Manual trigger; 1/128 prescaler
	ADCSRB = 0b00000000;  // Unipolar; 1x gain; Free running mode
	DIDR0 |= _BV(ADC0D) | _BV(ADC1D);  // Disable ADC0 (PA0) and ADC1 (PA1) digital input buffer
	ADCSRA |= _BV(ADSC); // Start next conversion
	
	if (PINB & _BV(PB3))
		bits |= OPTION_COMMON_ANODE;
	
	initDebounceState8(optionsDebouncer, bits);
	
}

uint8_t optionsADCValueToBits(uint8_t adcVal)
{
	// This ADC routine assumes a structure like this:
	//
	//                 ----ADC Input
	//         19k    |          10k
	//  VCC --/\/\/-------------/\/\/---(SW2)----GND
	//                  |  
	//                   -------/\/\/---(SW1)----GND
	//                           20k
	//
	// For a 5V supply, that yields:
	// 
	//  SW1  SW2   Voltage
	//  Off  Off   5V
	//  Off  On    3.3V
	//  On   Off   2.5V
	//  On   On    2V

	if(adcVal > 212)
	{
		return 0x00;
	}
	else if(adcVal > 149)
	{
		return 0x01;
	}
	else if(adcVal > 115)
	{
		return 0x02;
	}

	return 0x03;
}

void readOptions(DebounceState8_t* optionsDebouncer)
{
	// Options is now an ADC input based on PA0 and PA1
	// Options 1 is now 
	static uint16_t options1Accumulator = 0;
	static uint16_t options2Accumulator = 0;
	static uint8_t accumulatorCycles = 0;
	
	// ADC conversion still running?  Why?
	if (ADCSRA & _BV(ADSC))
		return;
	
	switch(ADMUX & 0x1F)
	{
		case 0:
			options1Accumulator += ADCH;
			// Set to channel 1
			ADMUX = (ADMUX & 0xE0) | 0x01;
			break;

		case 1:
			options2Accumulator += ADCH;
			// Set to channel 0
			ADMUX = (ADMUX & 0xE0) | 0x00;
			accumulatorCycles++;
			break;

		default:  // Should never get here, but if we do...
			ADMUX &= ~(0x1F);
			options1Accumulator = options2Accumulator = 0;
			accumulatorCycles = 0;
			break;
	}	

	ADCSRA |= _BV(ADSC); // Start next conversion

	if (ADC_AVERAGING_CYCLES == accumulatorCycles)
	{
		uint8_t bits = optionsADCValueToBits((uint8_t)(options1Accumulator / ADC_AVERAGING_CYCLES))
			| (optionsADCValueToBits((uint8_t)(options2Accumulator / ADC_AVERAGING_CYCLES))<<2);

		if (PINB & _BV(PB3))
			bits |= OPTION_COMMON_ANODE;

		debounce8(0x1F & bits, optionsDebouncer);
		options1Accumulator = options2Accumulator = accumulatorCycles = 0;
	}
}

