#include "MCM_BL6523GX.h"

BL6523GX BL;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  BL.begin(4800, D0, D1); //Baud rate, RX pin, TX pin

  delay(250);

  BL.setGain();
  delay(25);
  BL.setMode(); // enabling the voltage high pass filter
  delay(25);
  BL.setCFOutputMode();
}

void loop() {
  // put your main code here, to run repeatedly:
  float voltage;
  BL.getVoltage(&voltage);
  Serial.printf("%.2f [V]\n", voltage);

  float currentA, currentB;
  BL.getCurrent(&currentA, &currentB);
  Serial.printf("IA: %.4f [A], IB: %.4f [A], \n", currentA, currentB);

  float frequency;
  BL.getFrequency(&frequency);
  Serial.printf("%.2f [Hz]\n", frequency);

  float pf;
  BL.getPowerFactor(&pf);
  Serial.printf("%.2f [PF]\n", pf);

  float powerA, powerB;
  BL.getActivePower(&powerA, &powerB);
  Serial.printf("A: %.2f [W], B: %.2f [W]\n", powerA, powerB);

  float Apower;
  BL.getAparentPower(&Apower);
  Serial.printf("%.2f [VA]\n", Apower);

  float energy;
  BL.getActiveEnergy(&energy);
  Serial.printf("%.2f [W/h]\n", energy);    // in one hour with a resistive load 53.52 W (0.4789 A) the energy was 429.00
  delay(1000);

  Serial.println();
}
