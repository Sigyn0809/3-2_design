#include <Wire.h>

//LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Current sensor
#include <Adafruit_INA219.h>
#define I2C_ADDRESS 0x40
Adafruit_INA219 ina219;

// Digital to Analog Convertor
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;
#define DAC_RESOLUTION (9)

const float loadR = 1;            // Load Resistor (5W)

void setup() {
 
  Serial.begin(9600);
  Serial.println("*** Please connect batteries correctly !!! ***");  
  Serial.println(" ");  

  // DAC
  dac.begin(0x60); 
  if (!dac.begin(0x60)) {
    Serial.println("Couldn't find MCP4725!");
    while (1);
  }
  
  // LCD
  lcd.begin();
}

void loop() {
  int long adcOff = 0;
  int long adcOn = 0;
  int long sampleOff = 0;
  int long sampleOn = 0;
  int long curTime = millis();

  dac.setVoltage(0, false);

  while(millis()-curTime < 1000) 
  {      // Battery Voltage without Load
    adcOff += analogRead(A0) * 2;
    sampleOff += 1;
  }
  float load_ocv = (float)adcOff/sampleOff/1023.0*5.0;
  float voltageOn = load_ocv;  
  
  if(load_ocv > 1.0)  
  {
    dac.setVoltage(500, false);
    while(millis()-curTime < 2000) {    // Battery Voltage with Load
      adcOn += analogRead(A0) * 2;
      sampleOn += 1;
    }
    voltageOn = (float)adcOn/sampleOn/1023.0*5.0;    
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Please connect");
    lcd.setCursor(0,1);
    lcd.print("battery correctly !");
  }

  if(load_ocv > 1.0 && voltageOn > 1.0 && load_ocv > voltageOn) 
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(String(load_ocv) + "V" + "->" + String(voltageOn) + "V" + "(" + String(voltageOn/load_ocv*100.0) + "%)");
    lcd.setCursor(0,1);
    lcd.print("Internal R = " + String((load_ocv/voltageOn-1.0)*loadR) + "ohm");
    Serial.print(load_ocv, 3);
    Serial.print(" --> ");
    Serial.print(voltageOn, 3);
    Serial.print(" V (");
    Serial.print(voltageOn/load_ocv*100.0, 1);
    Serial.print("%), Internal R = ");
    Serial.print((load_ocv/voltageOn-1.0)*loadR, 3);
    Serial.println(" ohm");  
  }
}