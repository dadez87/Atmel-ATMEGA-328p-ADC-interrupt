// Davide Cavaliere
// www.monocilindro.com
// Analog Read using Atmel ATmega 328p and interrupt

#include "ADCmgr/ADCmgr.h" // ADC measurement management

void setup() {
  Serial.begin(57600);
  ADCmgr_init(); // Initializes ADC
  ADCmgr_program_pin_read(ADCmgr_pins_order[0]); // Starts the ADC conversion (which will then continue indefinitely)
}


void loop() {

  // Sends, via Serial, the results of ADC sensors measurements, which are performed in the "ADCmgr" library using interrupts
  ADCmgr_read_time_buf_busy = true; // locks the buffer access (semaphore)
  Serial.print("Scan Time Required (all ADC signals): ");
  Serial.print(ADCmgr_read_time_end_buf - ADCmgr_read_time_start_buf);
  Serial.println("us");
  for (uint8_t i = 0; i<ADCMGR_PINS_ORDER_SIZE; i++){
    Serial.print("A");
    Serial.print(ADCmgr_pins_order[i]);
    Serial.print(",Analog:");
    Serial.print(ADCmgr_measures[i]);
    Serial.print(",");
    Serial.print("Digital:");
    Serial.println(ADCmgr_meas_binary[i]);
  }
  ADCmgr_read_time_buf_busy = false; // unlocks the buffer access (semaphore)
  Serial.println();

  // Lamda Sensor buffer acquired values
  Serial.print("Scan Time Required (Lambda buffer filling): ");
  Serial.print(ADCmgr_lambda_acq_buf_time_end_buf - ADCmgr_lambda_acq_buf_time_start_buf);
  Serial.println("us");
  Serial.print("Samples: ");
  for (uint8_t i = 0; i<ADCMGR_LAMBDA_ACQ_BUF_SIZE; i++){
    Serial.print(ADCmgr_lambda_acq_buf[i]);
    Serial.print(",");
  }
  Serial.println();
  
  Serial.println();
  Serial.println();
  ADCmgr_lambda_acq_buf_filled = false; // To allow the buffer to be re-filled again
  ADCmgr_lambda_acq_buf_fill_req = true; // Buffer fill request ON (request buffer filling for Lambda sensor measurements)
  
  delay(2000);

}
