// Davide Cavaliere
// www.monocilindro.com
// dadez87-at-gmail-com
// ADC aquisition library for Atmel ATmega 328p
// Date: 7 January 2017
// This library is used, on Fuelino project, to acquire data from sensors (Throttle, Lambda) continuously using ADC interrupt

#ifndef ADCmgr_h
#define ADCmgr_h

#include <Arduino.h>

#define ADCMGR_DI1_PIN 0 // Digital Input DI1 (A0)
#define ADCMGR_THROTTLE_PIN 2 // Throttle Pin (A2)
#define ADCMGR_LAMBDA_PIN 3 // Lambda Pin (A3)
#define ADCMGR_VBATTERY_PIN 6 // Voltage Battery Pin (A6)
#define ADCMGR_ENABLE_OR_INT_PIN 7 // ENABLE_OR_INT (A6)
#define ADCMGR_TEMPERATURE_PIN 8 // Internal Temperature
#define ADCMGR_PINS_ORDER_SIZE 6 // Number of pins to be read continuously
#define ADCMGR_LAMBDA_ACQ_BUF_SIZE 8 // Total acquisitions of Lambda Sensor voltage (single byte, 0 - 255)

// Variables
extern volatile uint8_t ADCmgr_pin_read_now_index; // Increasing number of pin to be read [0 .. (ADCMGR_PINS_ORDER_SIZE-1)]
extern const uint8_t ADCmgr_pins_order[]; // Contains the physical number of the pin to be read
extern volatile bool ADCmgr_pins_buffer_busy[]; // Buffer busy status. This flag is set when reading the status
extern volatile uint16_t ADCmgr_measures[]; // Contains the value read
extern volatile uint8_t ADCmgr_meas_binary[]; // Digital value (0 .. 1)

// For debugging purpose (time calculation of the complete reading time for one cycle)
extern volatile unsigned long ADCmgr_read_time_start_buf; // Time when start reading (buffer value)
extern volatile unsigned long ADCmgr_read_time_end_buf; // Time when finishing reading (buffer value)
extern volatile bool ADCmgr_read_time_buf_busy; // becomes "true" when Main Loop is reading the buffer

// For Lambda sensor buffer acquisition
extern uint8_t ADCmgr_lambda_acq_buf[];
extern bool ADCmgr_lambda_acq_buf_fill_req; // Request of start filling the buffer. It comes from an external module
extern bool ADCmgr_lambda_acq_buf_filled; // Becomes true after the buffer is completely filled
extern volatile unsigned long ADCmgr_lambda_acq_buf_time_start_buf; // ADC buffer acquisition starting time
extern volatile unsigned long ADCmgr_lambda_acq_buf_time_end_buf; // ADC buffer acquisition ending time

// Functions
extern void ADCmgr_init(); // Initialization
extern uint16_t ADCmgr_read_pin_now(uint8_t ADC_pin); // Read pin voltage, waits until the ADC process has finished, then returns the value
extern void ADCmgr_program_pin_read(uint8_t ADC_pin); // Read pin voltage, by starting the ADC process. The reading will be done during interrupt event

#endif