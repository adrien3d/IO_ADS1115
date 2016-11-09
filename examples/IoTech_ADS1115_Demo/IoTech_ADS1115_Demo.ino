/***************************************************************************
  This is a library for the ADS1115 precision ADC

  Designed to work with all kinds of ADS1115 Breakout

  These sensors use I2C to communicate, 2 pins are required
  to interface.

  Written by Adrien Chapelet for Iotech
 ***************************************************************************/
 
#include <Arduino.h>
#include "IO_ADS1115.h"

IO_ADS1115 ads;

void setup(){
  Serial.begin(115200);
  Serial.println("Setup...");
  ads.begin();
  ads.setGain(ADS1115_GAIN_TWO);
}

void loop(){
    int16_t adc0, adc1, adc2, adc3;

    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);
    Serial.print("AIN0: "); Serial.println(adc0);
    Serial.print("AIN1: "); Serial.println(adc1);
    Serial.print("AIN2: "); Serial.println(adc2);
    Serial.print("AIN3: "); Serial.println(adc3);
    Serial.println(" ");
  

    delay(500); // wait a second before printing again
}