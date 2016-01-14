/*
  LibTemperature - A Temperature Library for Arduino.

  Supported Temperature Sensor modules:
    TMP421-Breakout Module - http://shop.moderndevice.com/products/tmp421-temperature-sensor

  code by Christopher Ladden 2009. 
  revised Paul Badger 2012
  

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <inttypes.h>
#include <Wire.h>
#include <Arduino.h>
#include "LibTempTMP421.h"

/******************************************************************************
 * Constructors 
 ******************************************************************************/

/**
 * Initialize the Wire library interface, optionally providing
 * the chip power via the A2 and A3 pins (default off).
 *
 * The analog_pin_power option turns on Arduino pins A2, A3 (aka 16, 17) to
 * power the sensor.  This is necessary due to the fact that Wire.begin() is
 * called in the constructor and needs to talk to the chip, which needs to be
 * powered.  If not using these pins for power, leave the parameter at its
 * default false value.
 */
 LibTempTMP421::LibTempTMP421(bool analog_pin_power) {
    if (analog_pin_power) {
        pinMode(A2, OUTPUT);
        digitalWrite(A2, LOW);       // GND pin
        pinMode(A3, OUTPUT);
        digitalWrite(A3, HIGH);      // VCC pin
    }

    Wire.begin();
}

/******************************************************************************
 * Global Functions
 ******************************************************************************/

/**********************************************************
 * GetTemperature
 *  Gets the current temperature from the sensor.
 *
 * @return float - The local temperature in degrees C
 **********************************************************/
float LibTempTMP421::GetTemperature(void) {
    int8_t high;  // signed: high byte is returned as signed 2's complement
    uint8_t low;

    setPtrLoc(0x00);                //high-byte (integer part)
    high = getRegisterValue();

    setPtrLoc(0x10);                //low-byte (fractional part)
    low = getRegisterValue();
    low >>=4;                       //shift-off the unused bits

    return high + (low / 16.0);
}

/**********************************************************
 * getRegisterValue
 *  Get the TMP421 register value via I2C
 *
 * @return uint8_t - The register value
 **********************************************************/
uint8_t LibTempTMP421::getRegisterValue(void) {

    Wire.requestFrom(0x2A, 1);
    while(Wire.available() <= 0) {
      ; //wait
    }

    return Wire.read();
}

/**********************************************************
 * setPtrLoc
 *  Sets the TMP421 pointer register location via I2C
 *
 * @param ptrLoc - The pointer register address
 **********************************************************/
void LibTempTMP421::setPtrLoc(uint8_t ptrLoc) {

    //Set the pointer location
    Wire.beginTransmission(0x2A);   //begin
    Wire.write(ptrLoc);             //send the pointer location
    Wire.endTransmission();         //end
    delay(8);
}

