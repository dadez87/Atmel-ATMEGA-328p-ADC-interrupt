// Davide Cavaliere
// www.monocilindro.com
// dadez87-at-gmail-com
// ADC aquisition library for Atmel ATmega 328p
// Date: 7 January 2017
// This library is used, on Fuelino project, to acquire data from sensors (Throttle, Lambda) continuously using ADC interrupt

#ifndef ADCmgr_cpp
#define ADCmgr_cpp

#include "ADCmgr.h"

volatile uint8_t ADCmgr_pin_read_now_index = 0; // Start from the first element of the array
const uint8_t ADCmgr_pins_order[ADCMGR_PINS_ORDER_SIZE] = {ADCMGR_DI1_PIN, ADCMGR_THROTTLE_PIN, ADCMGR_LAMBDA_PIN, ADCMGR_VBATTERY_PIN, ADCMGR_ENABLE_OR_INT_PIN, ADCMGR_TEMPERATURE_PIN};
volatile bool ADCmgr_pins_buffer_busy[ADCMGR_PINS_ORDER_SIZE]; // buffer busy status. This flag is set when reading the status
volatile uint16_t ADCmgr_measures[ADCMGR_PINS_ORDER_SIZE]; // Analog value (0 .. 1023)
volatile uint8_t ADCmgr_meas_binary[ADCMGR_PINS_ORDER_SIZE]; // Digital value (0 .. 1)

// For debugging purpose (calculation of cycle time)
volatile unsigned long ADCmgr_read_time_start = 0;
volatile unsigned long ADCmgr_read_time_end = 0;
volatile unsigned long ADCmgr_read_time_start_buf = 0;
volatile unsigned long ADCmgr_read_time_end_buf = 0;
volatile bool ADCmgr_read_time_buf_busy = false;

// For Lambda sensor buffer acquisition
uint8_t ADCmgr_lambda_acq_buf[ADCMGR_LAMBDA_ACQ_BUF_SIZE];
uint8_t ADCmgr_lambda_acq_buf_index = 0; // Index when reading Lambda sensor voltage
bool ADCmgr_lambda_acq_buf_fill_req = false; // Request of start filling the buffer. It comes from an external module
bool ADCmgr_lambda_acq_buf_filled = false; // Becomes true after the buffer is completely filled
volatile unsigned long ADCmgr_lambda_acq_buf_time_start = 0;
volatile unsigned long ADCmgr_lambda_acq_buf_time_end = 0;
volatile unsigned long ADCmgr_lambda_acq_buf_time_start_buf = 0;
volatile unsigned long ADCmgr_lambda_acq_buf_time_end_buf = 0;
volatile bool ADCmgr_lambda_acq_buf_busy = false; // Buffer access semaphore (turned "true" in Main Loop) to avoid data corruption


// Initializes the ADC
void ADCmgr_init(){
	
	pinMode(A0, INPUT_PULLUP); // Input pullup on DI1 pin (A0)
	
	for (uint8_t i = 0; i<ADCMGR_PINS_ORDER_SIZE; i++){
		ADCmgr_pins_buffer_busy[i] = false; // Buffer is not occupied
	}
	
	// ADMUX - ADC Multiplexer Selection Register 
	ADMUX = 0b01000000; // AVcc (5V) as reference, ADLAR = 0 (ADLAR: ADC Left Adjust Result)
	
	// ADCSRA - ADC Control and Status Register A
	ADCSRA = ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)); //Prescaler at 128 so we have an 125Khz clock source, Bits 2:0 ? ADPS[2:0]: ADC Prescaler Select Bits
	// Bit 3 ? ADIE: ADC Interrupt Enable
	// Bit 4 ? ADIF: ADC Interrupt Flag
	// Bit 5 ? ADATE: ADC Auto Trigger Enable
	// Bit 6 ? ADSC: ADC Start Conversion
	// Bit 7 ? ADEN: ADC Enable
	
	// ADCSRB - ADC Control and Status Register B
	ADCSRB = 0;
	
}


// Reads the ADC pin status, and returns the ADC value. Each conversion requires about 25 clock cycles (at 125kHz).
uint16_t ADCmgr_read_pin_now(uint8_t ADC_pin){

	ADMUX &= 0xF0; // Removes previous pin setting
	ADMUX |= (ADC_pin & 0x0F); // Pin number setting
	
	ADCSRA |= (1<<ADEN); // Enables the ADC - Bit 7 ? ADEN: ADC Enable
	ADCSRA &= ~(1<<ADIE); // Disables interrupt - Bit 3 ? ADIE: ADC Interrupt Enable
	ADCSRA |= (1<<ADSC); // Start converting

	while ((ADCSRA & _BV(ADSC))); // Wait until the end of the conversion

	// Read ADC measurement
	uint8_t adc_L = ADCL; // low part
	uint8_t adc_H = ADCH; // high part
	
	ADCSRA &= ~(1<<ADEN); // Disables the ADC - Bit 7 ? ADEN: ADC Enable
	
	uint16_t adc_value = ((uint16_t)adc_H << 8) | (uint16_t)adc_L; // Calculates the ADC value
	
	return adc_value;
}


// Programs the reading of an ADC pin. Reading must be done using the interrupt. Each conversion requires about 25 clock cycles (at 125kHz).
void ADCmgr_program_pin_read(uint8_t ADC_pin){

	ADMUX &= 0xF0; // Removes previous pin setting
	ADMUX |= (ADC_pin & 0x0F); // Pin number setting

	ADCSRA |= (1<<ADEN); // Enables the ADC - Bit 7 ? ADEN: ADC Enable
	ADCSRA |= (1<<ADIE); // Enables interrupt - Bit 3 ? ADIE: ADC Interrupt Enable
	ADCSRA |= (1<<ADSC); // Start converting

}


// Interrupt service routine for the ADC completion
ISR(ADC_vect){
 
  // Read ADC measurement
  uint8_t adc_L = ADCL; // low part
  uint8_t adc_H = ADCH; // high part
  ADCSRA &= ~(1<<ADEN); // Disables the ADC - Bit 7 ? ADEN: ADC Enable
  if (ADCmgr_pins_buffer_busy[ADCmgr_pin_read_now_index] == false) ADCmgr_measures[ADCmgr_pin_read_now_index] = ((uint16_t)adc_H << 8) | (uint16_t)adc_L; // Calculates the ADC value (2 bytes) and stores it into the buffer, if not busy
  if (adc_H == 0) {
	  ADCmgr_meas_binary[ADCmgr_pin_read_now_index] = 0; // OFF, if <= 255
  }else{
	  ADCmgr_meas_binary[ADCmgr_pin_read_now_index] = 1; // ON, if >= 256
  }
  
  // Check if there is anything special to do with the present measured signal
  switch (ADCmgr_pins_order[ADCmgr_pin_read_now_index]){
    case ADCMGR_LAMBDA_PIN: // Lambda sensor signal acquired
		if ((ADCmgr_lambda_acq_buf_fill_req == true) && (ADCmgr_lambda_acq_buf_filled == false)){ // Filling request coming from external module, and the buffer is not yet filled completely
			if (ADCmgr_lambda_acq_buf_index == 0) ADCmgr_lambda_acq_buf_time_start = micros(); // Save starting time stamp
			if (adc_H != 0) { // Over 1.25V
				ADCmgr_lambda_acq_buf[ADCmgr_lambda_acq_buf_index] = 0xFF; // Saturation to max value, about 1.25V
			}else{
				ADCmgr_lambda_acq_buf[ADCmgr_lambda_acq_buf_index] = adc_L; // Save low byte (0 - 255), corresponds to a max value of about 1.25V
			}
			ADCmgr_lambda_acq_buf_index++; // increase counter
			if (ADCmgr_lambda_acq_buf_index == ADCMGR_LAMBDA_ACQ_BUF_SIZE){ // the Lambda buffer has been completely filled
				ADCmgr_lambda_acq_buf_index = 0; // Set the counter to zero, for next use
				ADCmgr_lambda_acq_buf_time_end = micros(); // Save ending time stamp
				ADCmgr_lambda_acq_buf_fill_req = false; // Filling request turned OFF
				ADCmgr_lambda_acq_buf_filled = true; // Indicates that the buffer has been completely filled, and is now ready to use
				if (ADCmgr_lambda_acq_buf_busy == false){ // if the buffer is not being used, update the buffer
					ADCmgr_lambda_acq_buf_time_start_buf = ADCmgr_lambda_acq_buf_time_start; // update buffer
					ADCmgr_lambda_acq_buf_time_end_buf = ADCmgr_lambda_acq_buf_time_end; // update buffer
				}
			}
		}else{
			ADCmgr_lambda_acq_buf_index = 0; // Set the counter to zero, just to make sure
		}
		break;
	  
    default:
      break;
	  
  }

  // Programs the next ADC reading
  ADCmgr_pin_read_now_index++; // Increase pin
  if (ADCmgr_pin_read_now_index == ADCMGR_PINS_ORDER_SIZE) { // Read all pins
	  ADCmgr_pin_read_now_index = 0; // restart from the first pin
	  // Cycle time calculation
	  ADCmgr_read_time_start = ADCmgr_read_time_end; // starting time is the time at which it ended last measurement cycle
	  ADCmgr_read_time_end = micros(); // reads present time
	  if (ADCmgr_read_time_buf_busy == false){ // if the buffer is not being used, update the buffer
		  ADCmgr_read_time_start_buf = ADCmgr_read_time_start; // update buffer
		  ADCmgr_read_time_end_buf = ADCmgr_read_time_end; // update buffer
	  }
  }
  ADCmgr_program_pin_read(ADCmgr_pins_order[ADCmgr_pin_read_now_index]); // programs the next ADC read

}

#endif