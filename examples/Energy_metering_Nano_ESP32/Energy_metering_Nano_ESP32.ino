#include "MCM_BL6523GX.h"

BL6523GX BL;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  BL.setGain();
  delay(25);
  BL.setCFOutputMode();
}

void loop() {
  // put your main code here, to run repeatedly:
  float voltage;
  BL.getVoltage(&voltage);
  Serial.printf("%.2f [V]\n", voltage);
  delay(1000);
  float currentA, currentB;
  BL.getCurrent(&currentA, &currentB);
  Serial.printf("IA: %.4f [A], IB: %.4f [A], \n", currentA, currentB);
  delay(1000);
  float frequency;
  BL.getFrequency(&frequency);
  Serial.printf("%.2f [Hz]\n", frequency);
  delay(1000);
  float pf;
  BL.getPowerFactor(&pf);
  Serial.printf("%.2f [PF]\n", pf);
  delay(1000);
  float powerA, powerB;
  BL.getActivePower(&powerA, &powerB);
  Serial.printf("A: %.2f [W], B: %.2f [W]\n", powerA, powerB);
  delay(1000);
  float Apower;
  BL.getAparentPower(&Apower);
  Serial.printf("%.2f [VA]\n", Apower);
  delay(1000);
  float energy;
  BL.getActiveEnergy(&energy);
  Serial.printf("%.2f [W/h]\n", energy);    // in one hour with a resistive load 53.52 W (0.4789 A) the energy was 429.00
  delay(1000);

}
