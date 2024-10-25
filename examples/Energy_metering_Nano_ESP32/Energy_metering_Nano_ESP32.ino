#include "MCM_BL6523GX.h"
#include <Modulino.h>

ModulinoPixels leds;

int bright = 5;

#define USR_BTN A1

BL6523GX BL;

unsigned long previousMillis = 0;
int lock = 0;
// constants won't change:
const long interval = 1000;

#define CF_DIV 4
#define V_GAIN 2
#define IB_GAIN 2
#define IA_GAIN 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Modulino.begin();
  leds.begin();

  pinMode(USR_BTN, INPUT);

  BL.begin(4800, D0, D1);  //Baud rate, RX pin, TX pin

  delay(250);

  BL.setGain(V_GAIN, IB_GAIN, IA_GAIN);  //Voltage Gain, Current B Gain, Current A Gain
  delay(25);
  BL.setMode();  // enabling the voltage high pass filter
  delay(25);
  BL.setCFOutputMode(CF_DIV);  // select the CF output divider
  delay(25);
  delay(250);
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

    Feedback(powerB);

    float Apower;
    BL.getAparentPower(&Apower);
    Serial.printf("%.2f [VA]\n", Apower);

    float energy;
    BL.getActiveEnergy(&energy);
    Serial.printf("%.2f [W/h]\n", energy);  // in one hour with a resistive load 53.52 W (0.4789 A) the energy was 429.00

    Serial.println();

  }

  if (digitalRead(USR_BTN) == LOW && lock == 0) {
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
  }
}

void Feedback(int power) {
  int count = map(power, 0, 3200, 1, 8);
  if(count > 8){
    count = 8;
  }
  Serial.printf("LEDs to control: %d", count);
  Serial.println();
  for (int i = 0; i < count; i++) {
    if (i < 3) {
      leds.set(i, GREEN, bright);
    } else if (i < 6) {
      leds.set(i, 255, 255, 0, bright);
    } else if (i <= 8) {
      leds.set(i, RED, bright);
    }
    //leds.set(i, colorPower, 127);
    leds.show();
  }
  for (int i = count; i < (9 - count); i++) {
    leds.set(i, 0, 0, 0, bright);
    leds.show();
  }
}