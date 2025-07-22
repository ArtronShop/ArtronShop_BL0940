#include "MCM_BL0940.h"

BL0940 bl0940;

void setup() {
  Serial.begin(115200);

  bl0940.begin(Serial1, RX, TX); // RX pin - TX pin 
  bl0940.Reset();
  bl0940.setFrequency(50); // 50 Hz
  bl0940.setUpdateRate(400); // 400 mS
  bl0940.setCurrentOffset(-52);
  bl0940.setActivePowerOffset(80);
}

void loop() {

  float voltage;
  bl0940.getVoltage( &voltage );
  Serial.printf("%.2f [V]\n", voltage );

  float current;
  bl0940.getCurrent( &current );
  Serial.printf("%.4f [A]\n", current );

  float activePower;
  bl0940.getActivePower( &activePower );
  Serial.printf("%.2f [W]\n", activePower );

  float activeEnergy;
  bl0940.getActiveEnergy( &activeEnergy );
  Serial.printf("%.6f [kWh]\n", activeEnergy );

  float powerFactor;
  bl0940.getPowerFactor( &powerFactor );
  Serial.printf("%.1f [%%]\n", powerFactor );

  float temperature;
  bl0940.getTemperature( &temperature );
  Serial.printf("%.1f [deg C]\n", temperature );

  Serial.println("");
  delay(1000);

}
