/****************************
 * GetLocalTemperature
 *  An example sketch that prints the
 *  local temperature to the PC's serial port
 *
 *  Tested with the TMP421-Breakout
 *  Temperature Sensor from Modern Device
 *****************************/
#include "Wire.h"
#include <LibTempTMP421.h>


/* The LibTemp421 library optionally turns on Arduino pins A2, A3 (aka 16, 17)
 * to power the sensor.  This is necessary due to the fact that
 * Wire.begin() is called in the constructor and needs to talk to the
 * chip, which needs to be powered. If you are using the sensor in a
 * different location and powering it from dedicated GND and +5V lines
 * then set the constructor's parameter to false. */
LibTempTMP421 temp = LibTempTMP421(true);

void setup() {
    Serial.begin(9600);
}

void loop() {
    Serial.print("Temp: ");
    Serial.print(temp.GetTemperature());
    Serial.println(" degC");
    delay(100);
}

