#include <EEPROM.h>
#include "MCP_DAC.h"


MCP4911 MCP;  // HW SPI

volatile int x;
uint32_t start, stop;

const int pedalPin = A3;
const int purplePin = A7;
const int grayPin = A6;
const int rocker1 = 2;
const int rocker2 = 4;

unsigned int pedalValue = 0;
unsigned int purpleValue = 0;
unsigned int grayValue = 0;

const int buzzerPin = 7;

const int idle = 0;            // 168//5 volts is 1023, so (1023/5)*.822 = 168
const int topthrottle = 1023;  // 863//(4.22/5)*1023
const int crossingpoint = (topthrottle - idle) / 2;

float scalefactor = .15;

unsigned int rpmbasestep = 175;  //(4.22/5)*1023 = 863, so full throttle is 4.22V, 5V VRef, = 863 at
//full throttle 863-168=695, 695/4=174
unsigned int basethrottle = 0;
unsigned int buzzerValue = 0;

unsigned int maxPedal = 0;
unsigned int minPedal = 1023;
unsigned int maxThrot = 0;
unsigned int minThrot = 1023;
unsigned int maxPedalold = 0;
unsigned int minPedalold = 1023;
unsigned int maxThrotold = 0;
unsigned int minThrotold = 1023;

byte checkEEPROMadd = 0;
byte checkEEPROM = 0;
byte checkEEPROMval = 144;
const unsigned int maxPedaladd = 1;
unsigned int maxPedalEEPROM = 850;
const unsigned int minPedaladd = 3;
unsigned int minPedalEEPROM = 150;
const unsigned int rockervalEEPROMadd = 5;
unsigned int rockervalEEPROM = 500;

unsigned int lowadd;
unsigned int highadd;

int rockerState = HIGH;     // the current state of the output pin
int rockerswitchState;      // the current reading from the input pin
int lastrockerState = LOW;  // the previous reading from the input pin

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay =
500;  // the debounce time; increase if the output flickers
unsigned long longdebounceDelay = 1000;

unsigned int rockerval = 0;
unsigned int lastrockerval = 0;
const unsigned int shortrockerval = 50;
const unsigned int longrockerval = 100;

void rocker1Int(void);
void rocker2Int(void);

void setup() {
    pinMode(rocker1, INPUT_PULLUP);
    pinMode(rocker2, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, buzzerValue);

    Serial.begin(9600);

    MCP.begin(8);

    checkEEPROM = EEPROM.read(checkEEPROMadd);
    // checkEEPROM = checkEEPROM;

    if (checkEEPROM != 144)  // check to see if EEPROM has valid data
    {
        lowadd = lowByte(maxPedaladd);
        highadd = highByte(lowadd + sizeof(unsigned int));
        EEPROM.put(lowadd, maxPedalEEPROM);
        EEPROM.put(highadd, maxPedalEEPROM);
        lowadd = lowByte(minPedaladd);
        highadd = highByte(lowadd + sizeof(unsigned int));
        EEPROM.put(lowadd, minPedalEEPROM);
        EEPROM.put(highadd, minPedalEEPROM);
        EEPROM.write(checkEEPROMadd, checkEEPROMval);
        lowadd = lowByte(rockervalEEPROMadd);
        highadd = highByte(lowadd + sizeof(unsigned int));
        EEPROM.put(lowadd, rockervalEEPROM);
        EEPROM.put(highadd, rockervalEEPROM);
        EEPROM.update(maxPedaladd, maxPedalEEPROM);
        EEPROM.update(minPedaladd, minPedalEEPROM);
        EEPROM.update(checkEEPROMadd, checkEEPROMval);
        EEPROM.update(rockervalEEPROMadd, rockervalEEPROM);
    }

    else {
        EEPROM.get(maxPedaladd, maxPedalEEPROM);
        EEPROM.get(minPedaladd, minPedalEEPROM);
        EEPROM.get(checkEEPROMadd, checkEEPROM);
        EEPROM.get(rockervalEEPROMadd, rockervalEEPROM);
    }

    maxPedal = maxPedalEEPROM;
    minPedal = minPedalEEPROM;
    rockerval = rockervalEEPROM;

    Serial.println(checkEEPROM);
    Serial.println(maxPedalEEPROM);
    Serial.println(minPedalEEPROM);
    Serial.println(rockervalEEPROM);

    attachInterrupt(digitalPinToInterrupt(rocker1), rocker1Int, FALLING);
    //attachInterrupt(digitalPinToInterrupt(rocker2), rocker2Int, FALLING);
}


void loop() {
    // Serial.println(analogRead(A0));
    digitalWrite(buzzerPin, 0);

    pedalValue = analogRead(pedalPin);
    grayValue = analogRead(grayPin);
    purpleValue = analogRead(purplePin);

    // pedalValue = (float)map(pedalValue, minPedal, maxPedal, basethrottle,
    // maxPedal);
    EEPROM.get(rockervalEEPROMadd, rockervalEEPROM);
    EEPROM.get(maxPedaladd, maxPedalEEPROM);
    EEPROM.get(minPedaladd, minPedalEEPROM);

    if (rockerval > (rockervalEEPROM + 10) ||
        rockerval < (rockervalEEPROM - 10)) {
        lowadd = lowByte(rockervalEEPROMadd);
        highadd = highByte(lowadd + sizeof(unsigned int));
        rockervalEEPROM = rockerval;
    }

    // pedalValue = pedalValue + rockerval;

    if (pedalValue > (maxPedalEEPROM + 10) ||
        pedalValue < (maxPedalEEPROM - 10)) {
        lowadd = lowByte(maxPedaladd);
        highadd = highByte(lowadd + sizeof(unsigned int));
        maxPedalEEPROM = pedalValue;
    }

    if (pedalValue > (minPedalEEPROM + 10) ||
        pedalValue < (minPedalEEPROM - 10)) {
        lowadd = lowByte(minPedaladd);
        highadd = highByte(lowadd + sizeof(unsigned int));
        minPedalEEPROM = pedalValue;
    }
    // try try a lateropen settings later
    pedalValue = (float)map(pedalValue, minPedal, maxPedal, basethrottle,
        maxPedal);


    if (pedalValue <= purpleValue) {
        purpleValue = pedalValue;
    }
    else {
        purpleValue = purpleValue;
    }

    MCP.analogWrite(purpleValue, 0);

    // delay(10);
}

void rocker1Int(void) {
    // read the state of the switch into a local variable:
    // int reading = digitalRead(rocker1);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    // if (reading != lastrockerState)
    {
        // reset the debouncing timer
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay &&
        (millis() - lastDebounceTime) < longdebounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        rockerval = rockerval + shortrockerval;

    }
    else {
        rockerval = rockerval + longrockerval;
    }

    //  results for things work in most need go through vespa again this code save the reading. Next time through the loop, it'll be the lastButtonState:
      // lastrockerState = reading;
}

void rocker2Int(void) {
    // read the state of the switch into a local variable:
    // int reading = digitalRead(rocker2);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    // if (reading != lastrockerState) {
    // reset the debouncing timer
    lastDebounceTime = millis();

    if ((millis() - lastDebounceTime) > debounceDelay &&
        (millis() - lastDebounceTime) < longdebounceDelay) {
        // whatever the reading is at, it's been there for longer than the debounce
        // delay, so take it as the actual current state:

        rockerval = rockerval + shortrockerval;
    }
    else {
        rockerval = rockerval + longrockerval;
    }

    // save the reading. Next time through the loop, it'll be the lastButtonState:
    // lastrockerState = reading;
}
