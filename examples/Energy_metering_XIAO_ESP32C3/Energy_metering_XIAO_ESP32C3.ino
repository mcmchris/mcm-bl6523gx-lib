#include "MCM_BL6523GX.h"

BL6523GX BL;

unsigned long previousMillis = 0;
int lock = 0;
// constants won't change:
const long interval = 1000;

#define CF_DIV 4
#define V_GAIN 2
#define IB_GAIN 1
#define IA_GAIN 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  BL.begin(4800, D7, D6);  //Baud rate, RX pin, TX pin

  BL.setCal(50355, 55559, 481);  //Voltage Cal, Current Cal, Power Cal
  delay(25);
  BL.setGain(V_GAIN, IB_GAIN, IA_GAIN);  //Voltage Gain, Current B Gain, Current A Gain
  delay(25);
  BL.setMode(1, 0, 0, 1);  // CH, CF_MODE, DIS_OUT, ENERGY_MATH
  delay(25);
  BL.setCFOutputMode(CF_DIV);  // select the CF output divider
  delay(25);
}

void loop() {

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

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
    Serial.printf("%.0f [W/h]\n", energy);  // in one hour with a resistive load 53.52 W (0.4789 A) the energy was 429.00

    float Aenergy;
    BL.getAparentEnergy(&Aenergy);
    Serial.printf("%.0f [VA/h]\n", Aenergy);  // in one hour with a resistive load 53.52 W (0.4789 A) the energy was 429.00
    Serial.println();
  }

  // This is an example on how to reset the chip and it's energy count using an external button.
  /*
  if (digitalRead(USR_BTN) == LOW && lock == 0) { // assign USR_BTN variable to an actual input
    Serial.println("Reseting the W/h count and clearing Registers");
    delay(200);
    BL.Reset();
    delay(200);
    BL.setGain(V_GAIN, IB_GAIN, IA_GAIN);
    delay(25);
    BL.setMode();  // enabling the voltage high pass filter
    delay(25);
    BL.setCFOutputMode(CF_DIV);  // select the CF output divider
    lock = 1;
  } else {
    lock = 0;
  */
}
