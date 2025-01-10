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

#define USR_BTN A1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  pinMode(USR_BTN, INPUT);
  BL.begin(4800, D7, D6);  //Baud rate, RX pin, TX pin
  BLSetup();
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

    float cf_count;
    BL.getCFCount(&cf_count);
    Serial.printf("%.2f [CF Pulses]\n", cf_count);  // in one hour with a resistive load 53.52 W (0.4789 A) the energy was 429.00

    //float cf_to_energy = (cf_count)*pow(2,(CF_DIV-4))*CF_DIV/7.81;
    //Serial.printf("%.4f [CF Energy]\n", cf_to_energy);  // in one hour with a resistive load 53.52 W (0.4789 A) the energy was 429.00

    float energy;
    BL.getActiveEnergy(&energy);
    Serial.printf("%.4f [W/h]\n", energy); /// 8437.0);  


    float line_energy;
    BL.getLineWattHr(&line_energy);
    Serial.printf("%.4f [W/h]\n", line_energy); /// 8437.0);  
    Serial.println();
  }

  // This is an example on how to reset the chip and it's energy count using an external button.

  if (digitalRead(USR_BTN) == LOW && lock == 0) {  // assign USR_BTN variable to an actual input
    Serial.println("Reseting the W/h count and clearing Registers");
    delay(200);
    BL.Reset();
    delay(200);
    BLSetup();
    lock = 1;
  } else {
    lock = 0;
  }
}

void BLSetup() {
  BL.setCal(50355, 55559, 481);  //Voltage Cal, Current Cal, Power Cal
  delay(25);
  BL.setGain(V_GAIN, IB_GAIN, IA_GAIN);  //Voltage Gain, Current B Gain, Current A Gain
  delay(25);
  BL.setMode(1, 0, 0, 1);  // CH, CF_MODE, DIS_OUT, ENERGY_MATH
  delay(25);
  BL.setCFOutputMode(CF_DIV);  // select the CF output divider
}
