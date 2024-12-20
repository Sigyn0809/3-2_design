#define MAX_CURRENT 2500     //mA
#define MAX_VOLTAGE 24       //V
#define MAX_POWER 60         //W
#define MAX_TEMPERATURE 100  //도

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

// Actual
double ActualCurrent;
double ActualPower;
double ActualResistance;
double ActualTemperature;
double loadVoltage;
double Perr;

// setting variables
#define up 5
#define down 6
double setCurrent = 300;    // mA
double setResistance = 10;  // ohm
double setPower = 2;        // W

// Control variables
double error;
double dac_Vref = 100;

// ratary encoder
#define CLK 2  // 2번핀을 CLK로 지정
#define DT 3   // 3번핀을 DT로 지정
#define SW 4   // 4번핀을 스위치핀으로 지정


String Modes[] = { "CC", "CP", "CR" };  //used to identify which mode
int modeIndex = 0;
double setVariables[] = { setCurrent, setPower, setResistance };
int setVariableIndex = 0;
String setTypes[] = { "mA", "W", "ohm" };  //selects either mA, W or ohm
int setTypeIndex = 0;

int currentStateCLK;                // CLK의 현재 신호상태 저장용 변수
int lastStateCLK;                   // 직전 CLK의 신호상태 저장용 변수
unsigned long lastButtonPress = 0;  // 버튼 눌림 상태 확인용 변수
int buttoncounter = 0;

// 모드변경 인터럽트 발생 시
void updateEncoder();

void setup() {
  // 엔코더의 핀들을 입력으로 설정
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);  // 스위치핀은 내부 풀업저항 사용

  // 세팅값 조절핀
  pinMode(up, INPUT);
  pinMode(down, INPUT);

  Serial.begin(9600);
  Wire.begin();

  lastStateCLK = digitalRead(CLK);  // CLK핀 현재상태 확인

  //외부 인터럽트 등록, 핀의 상태가 변할 때(HIGH에서 LOW 또는 LOW에서 HIGH) 마다 updateEncoder함수가 실행됨.
  // 인터럽트 0번은 2번핀과 연결되어 있고 1번은 3번 핀과 연결되어 있음
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  // ina219
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  ina219.setCalibration_32V_2A();

  // DAC
  dac.begin(0x60);
  if (!dac.begin(0x60)) {
    Serial.println("Couldn't find MCP4725!");
    while (1)
      ;
  }

  // LCD
  lcd.begin(20, 4);
  lcd.backlight();
}

void loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Mode:");
  lcd.setCursor(8, 0);
  lcd.print("set");
  lcd.setCursor(0, 1);
  lcd.print("Current: ");
  lcd.setCursor(0, 2);
  lcd.print("Voltage: ");
  lcd.setCursor(0, 3);
  lcd.print("Power: ");
  lcd.setCursor(5, 0);
  lcd.print(Modes[modeIndex]);
  lcd.setCursor(11, 0);
  lcd.print(setVariables[setVariableIndex] + setTypes[setTypeIndex]);

  // varaibles
  double sensorValue = analogRead(A0);
  loadVoltage = 6 * sensorValue * (5.0 / 1023.0);
  ActualCurrent = ina219.getCurrent_mA();
  ActualPower = loadVoltage * (ActualCurrent / 1000);
  ActualTemperature = analogRead(A1) * (5.0 / 1023.0) * 100.0;

  int upvalue = digitalRead(up);
  int downvalue = digitalRead(down);

  // V_reference input
  dac.setVoltage(dac_Vref, false);

  Serial.print("Current:       ");
  Serial.print(ActualCurrent);
  Serial.println(" mA");
  Serial.print("Voltage:       ");
  Serial.print(loadVoltage);
  Serial.println(" V");
  Serial.print("Power:       ");
  Serial.print(ActualPower);
  Serial.println(" W");
  Serial.print("Load:       ");
  Serial.print(loadVoltage / (ActualCurrent / 1000));
  Serial.println(" ohm");
  Serial.print("Temperature:       ");
  Serial.print(ActualTemperature);
  Serial.println(" 도");
  Serial.print("Control -> dac_Vref:         ");
  Serial.println(dac_Vref);
  Serial.print("setCurrent:  ");
  Serial.println(String(setCurrent) + "mA");
  Serial.print("setPower:  ");
  Serial.println(String(setPower) + "W");
  Serial.print("setResistance:  ");
  Serial.println(String(setResistance) + "ohm");
  Serial.println("------------------------------------------");

  //그래프
  Serial.print(setCurrent);
  Serial.print(',');
  Serial.println(ActualCurrent);


  // load check
  if (loadVoltage > 1) {
    // CC mode
    if (Modes[modeIndex] == "CC") {
      if (ActualCurrent <= MAX_CURRENT && loadVoltage <= MAX_VOLTAGE && ActualPower <= MAX_POWER && ActualTemperature <= MAX_TEMPERATURE) {
        if (upvalue == HIGH) {
          if (setCurrent < 2000) {
            setCurrent = setCurrent + 50;
            lcd.setCursor(11, 0);
            lcd.print(setCurrent);
            lcd.print(setTypes[setTypeIndex]);
          }
        } else if (downvalue == HIGH) {
          if (setCurrent > 50) {
            setCurrent = setCurrent - 50;
            lcd.setCursor(11, 0);
            lcd.print(setCurrent);
            lcd.print(setTypes[setTypeIndex]);
          }
        }
        setVariables[setVariableIndex] = setCurrent;
        if (ActualCurrent < setCurrent || ActualCurrent > setCurrent) {
          Perr = (setCurrent - ActualCurrent) / setCurrent;
          if (Perr > 1) {
            Perr = 1;
          } else if (Perr < -1) {
            Perr = -1;
          }
          if (abs(Perr) > 0.01) {
            if (dac_Vref < 10) {
              dac_Vref += 100;
            }
            dac_Vref = dac_Vref * (1 + Perr / 2) + 1;
          }
          if (dac_Vref > 4095) {
            dac_Vref = 4095;
          } else if (dac_Vref < 0) {
            dac_Vref = 0;
          }
        }
        lcd.setCursor(9, 1);
        lcd.print(String(ActualCurrent) + "mA");
        lcd.setCursor(9, 2);
        lcd.print(String(loadVoltage) + "V");
        lcd.setCursor(9, 3);
        lcd.print(String(ActualPower) + "W");
      } else {
        dac_Vref = 0;
        lcd.setCursor(9, 1);
        lcd.print("Stop");
        lcd.setCursor(9, 2);
        lcd.print("Stop");
        lcd.setCursor(9, 3);
        lcd.print("Stop");
      }
    }
    // CR mode
    if (Modes[modeIndex] == "CR") {
      ActualResistance = loadVoltage / (ActualCurrent / 1000);  //ohm

      if (ActualCurrent <= MAX_CURRENT && loadVoltage <= MAX_VOLTAGE && ActualPower <= MAX_POWER && ActualTemperature <= MAX_TEMPERATURE) {
        if (upvalue == HIGH) {
          setResistance += 0.5;
          lcd.setCursor(11, 0);
          lcd.print(setResistance);
          lcd.print(setTypes[setTypeIndex]);
        } else if (downvalue == HIGH) {
          if (setResistance > 2) {
            setResistance -= 0.5;
            lcd.setCursor(11, 0);
            lcd.print(setResistance);
            lcd.print(setTypes[setTypeIndex]);
          }
        }
        setVariables[setVariableIndex] = setResistance;

        if (ActualResistance > setResistance || ActualResistance < setResistance) {
          Perr = (setResistance - ActualResistance) / setResistance;
          if (Perr > 1) {
            Perr = 1;
          } else if (Perr < -1) {
            Perr = -1;
          }

          if (abs(Perr) > 0.05) {
            if (dac_Vref < 10) {
              dac_Vref += 100;
            }
            dac_Vref = dac_Vref * (1 - Perr / 2) + 1;
          }
          if (dac_Vref > 4095) {
            dac_Vref = 4095;
          }
        }
        lcd.setCursor(9, 1);
        lcd.print(String(ActualCurrent) + "mA");
        lcd.setCursor(9, 2);
        lcd.print(String(loadVoltage) + "V");
        lcd.setCursor(9, 3);
        lcd.print(String(ActualPower) + "W");
      } else {
        dac_Vref = 0;
        lcd.setCursor(9, 1);
        lcd.print("Stop");
        lcd.setCursor(9, 2);
        lcd.print("Stop");
        lcd.setCursor(9, 3);
        lcd.print("Stop");
      }
    }
    // CP mode
    if (Modes[modeIndex] == "CP") {
      ActualPower = loadVoltage * (ActualCurrent / 1000);  //W

      if (ActualCurrent <= MAX_CURRENT && loadVoltage <= MAX_VOLTAGE && ActualPower <= MAX_POWER && ActualTemperature <= MAX_TEMPERATURE) {
        if (upvalue == HIGH) {
          if (setPower < 16) {
            setPower += 0.5;
            lcd.setCursor(11, 0);
            lcd.print(setPower);
            lcd.print(setTypes[setTypeIndex]);
          }
        } else if (downvalue == HIGH) {
          if (setPower >= 0.5) {
            setPower -= 0.5;
            lcd.setCursor(11, 0);
            lcd.print(setPower);
            lcd.print(setTypes[setTypeIndex]);
          }
        }
        setVariables[setVariableIndex] = setPower;

        if (ActualPower < setPower || ActualPower > setPower) {
          Perr = (setPower - ActualPower) / setPower;
          if (Perr > 1) {
            Perr = 1;
          } else if (Perr < -1) {
            Perr = -1;
          }

          if (abs(Perr) > 0.05) {
            if (dac_Vref < 10) {
              dac_Vref += 100;
            }
            dac_Vref = dac_Vref * (1 + Perr / 2) + 1;
          }
          if (dac_Vref > 4095) {
            dac_Vref = 4095;
          }
        }
        lcd.setCursor(9, 1);
        lcd.print(String(ActualCurrent) + "mA");
        lcd.setCursor(9, 2);
        lcd.print(String(loadVoltage) + "V");
        lcd.setCursor(9, 3);
        lcd.print(String(ActualPower) + "W");
      } else {
        dac_Vref = 0;
        lcd.setCursor(9, 1);
        lcd.print("Stop");
        lcd.setCursor(9, 2);
        lcd.print("Stop");
        lcd.setCursor(9, 3);
        lcd.print("Stop");
      }
    }
  } else {
    dac_Vref = 0;
  }

  delay(500);
}

void updateEncoder() {
  // CLK의 현재 상태를 읽어서
  currentStateCLK = digitalRead(CLK);

  // CLK핀의 신호가 바뀌었고(즉, 로터리엔코더의 회전이 발생했했고), 그 상태가 HIGH이면(최소 회전단위의 회전이 발생했다면)
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
    // DT핀의 신호를 확인해서 엔코더의 회전 방향을 확인함.
    if (digitalRead(DT) != currentStateCLK) {             // 신호가 다르다면 시계방향 회전
      modeIndex = (modeIndex + 1) % 3;                    // mode 변경 (순환)
      setVariableIndex = (setVariableIndex + 1) % 3;      // setVariables 변경(순환)
      setTypeIndex = (setTypeIndex + 1) % 3;              // setType 변경(순환)
    } else {                                              // 신호가 같다면 반시계방향 회전
      modeIndex = (modeIndex - 1 + 3) % 3;                // mode 변경 (순환)
      setVariableIndex = (setVariableIndex - 1 + 3) % 3;  // setVariables 변경(순환)
      setTypeIndex = (setTypeIndex - 1 + 3) % 3;          // setType 변경(순환)
    }
  }
  // 마지막 상태 변수 저장
  lastStateCLK = currentStateCLK;

  Serial.print("Mode: ");
  Serial.println(Modes[modeIndex]);
  Serial.println("**********************");
}







/*
double K = 2; // 보정계수 

// PID constants -> tuning
double kp = 10;
double ki = 1;
double kd = 10;

unsigned long currentTime, previousTime;
double elapsedTime;
double lastError;
double input, output;
double setPoint = 1;
double cumError, rateError;
double dac_ref = 100;

double computePID(double inp)
{
  currentTime = millis();  // get current time
  elapsedTime = (double)(currentTime - previousTime);  // coupute time elasped from previous computation

  error = setPoint - inp; // determine error
  cumError += error * elapsedTime; // compute integral
  rateError = (error - lastError)/elapsedTime; // compute derivative

  double out = kp*error + ki*cumError + kd*rateError; // PID output
  dac_ref = out;

  lastError = error;  // remember current error
  previousTime = currentTime; // remember current time

  return out;
}*/

/*if(error > setCurrent*0.8)
          {
            if(setCurrent > ActualCurrent)
            {
              dac_Vref = dac_Vref + 75;
            }
            else if(setCurrent < ActualCurrent)
            {
              dac_Vref = dac_Vref - 75;
            }
          }
          else if(error > setCurrent*0.6)
          {
            if(setCurrent > ActualCurrent)
            {
              dac_Vref = dac_Vref + 60;
            }
            else if(setCurrent < ActualCurrent)
            {
              dac_Vref = dac_Vref - 60;
            }
          }
          else if(error > setCurrent*0.4)
          {
            if(setCurrent > ActualCurrent)
            {
              dac_Vref = dac_Vref +40;
            }
            else if(setCurrent < ActualCurrent)
            {
              dac_Vref = dac_Vref - 40;
            }
          }
          else if(error > setCurrent*0.3)
          {
            if(setCurrent > ActualCurrent)
            {
              dac_Vref = dac_Vref + 25;
            }
            else if(setCurrent < ActualCurrent)
            {
              dac_Vref = dac_Vref - 25;
            }
          }
          else if(error > setCurrent*0.2)
          {
            if(setCurrent > ActualCurrent)
            {
              dac_Vref = dac_Vref + 20;
            }
            else if(setCurrent < ActualCurrent)
            {
              dac_Vref = dac_Vref - 20;
            }
          }
          else if(error > setCurrent*0.1)
          {
            if(setCurrent > ActualCurrent)
            {
              dac_Vref = dac_Vref + 15;
            }
            else if(setCurrent < ActualCurrent)
            {
              dac_Vref = dac_Vref - 15;
            }
          }
          else if(error > setCurrent*0.05)
          {
            if(setCurrent > ActualCurrent)
            {
              dac_Vref = dac_Vref + 10;
            }
            else if(setCurrent < ActualCurrent)
            {
              dac_Vref = dac_Vref - 10;
            }
          }
          else if(error > setCurrent*0.01)
          {
            if(setCurrent > ActualCurrent)
            {
              dac_Vref = dac_Vref + 5;
            }
            else if(setCurrent < ActualCurrent)
            {
              dac_Vref = dac_Vref - 5;
            }
          }*/

/*if(error > setResistance*0.8)
          {
            if(setResistance > ActualResistance)
            {
              dac_Vref = dac_Vref - 75;
            }
            if(setResistance < ActualResistance)
            {
              dac_Vref = dac_Vref + 75;
            }
          }
          else if(error > setResistance*0.6)
          {
            if(setResistance > ActualResistance)
            {
              dac_Vref = dac_Vref - 60;
            }
            if(setResistance < ActualResistance)
            {
              dac_Vref = dac_Vref + 60;
            }
          }
          else if(error > setResistance*0.4)
          {
            if(setResistance > ActualResistance)
            {
              dac_Vref = dac_Vref - 40;
            }
            if(setResistance < ActualResistance)
            {
              dac_Vref = dac_Vref + 40;
            }
          }
          else if(error > setResistance*0.3)
          {
            if(setResistance > ActualResistance)
            {
              dac_Vref = dac_Vref - 25;
            }
            if(setResistance < ActualResistance)
            {
              dac_Vref = dac_Vref + 25;
            }
          }
          else if(error > setResistance*0.2)
          {
            if(setResistance > ActualResistance)
            {
              dac_Vref = dac_Vref - 20;
            }
            if(setResistance < ActualResistance)
            {
              dac_Vref = dac_Vref + 20;
            }
          }
          else if(error > setResistance*0.1)
          {
            if(setResistance > ActualResistance)
            {
              dac_Vref = dac_Vref - 15;
            }
            if(setResistance < ActualResistance)
            {
              dac_Vref = dac_Vref + 15;
            }
          }
          else if(error > setResistance*0.05)
          {
            if(setResistance > ActualResistance)
            {
              dac_Vref = dac_Vref - 10;
            }
            if(setResistance < ActualResistance)
            {
              dac_Vref = dac_Vref + 10;
            }
          }
          else if(error > setResistance*0.01)
          {
            if(setResistance > ActualResistance)
            {
              dac_Vref = dac_Vref - 5;
            }
            if(setResistance < ActualResistance)
            {
              dac_Vref = dac_Vref + 5;
            }
          }*/

/*if(error > setPower*0.8)
          {
            if(setPower > ActualPower)
            {
              dac_Vref = dac_Vref + 75;
            }
            if(setPower < ActualPower)
            {
              dac_Vref = dac_Vref - 75;
            }
          }
          else if(error > setPower*0.6)
          {
            if(setPower > ActualPower)
            {
              dac_Vref = dac_Vref + 60;
            }
            if(setPower < ActualPower)
            {
              dac_Vref = dac_Vref - 60;
            }
          }
          else if(error > setPower*0.4)
          {
            if(setPower > ActualPower)
            {
              dac_Vref = dac_Vref + 40;
            }
            if(setPower < ActualPower)
            {
              dac_Vref = dac_Vref - 40;
            }
          }
          else if(error > setPower*0.3)
          {
            if(setPower > ActualPower)
            {
              dac_Vref = dac_Vref + 25;
            }
            if(setPower < ActualPower)
            {
              dac_Vref = dac_Vref - 25;
            }
          }
          else if(error > setPower*0.2)
          {
            if(setPower > ActualPower)
            {
              dac_Vref = dac_Vref + 20;
            }
            if(setPower < ActualPower)
            {
              dac_Vref = dac_Vref - 20;
            }
          }
          else if(error > setPower*0.1)
          {
            if(setPower > ActualPower)
            {
              dac_Vref = dac_Vref + 15;
            }
            if(setPower < ActualPower)
            {
              dac_Vref = dac_Vref - 15;
            }
          }
          else if(error > setPower*0.05)
          {
            if(setPower > ActualPower)
            {
              dac_Vref = dac_Vref + 10;
            }
            if(setPower < ActualPower)
            {
              dac_Vref = dac_Vref - 10;
            }
          }
          else if(error > setPower*0.01)
          {
            if(setPower > ActualPower)
            {
              dac_Vref = dac_Vref + 5;
            }
            if(setPower < ActualPower)
            {
              dac_Vref = dac_Vref - 5;
            }
          }*/

/*
          if(ActualCurrent < setCurrent || ActualCurrent > setCurrent)
          {
            Perr = (setCurrent - ActualCurrent)/ActualCurrent;
            if(Perr > 1)
            {
              Perr = 1;

            }
            else if (Perr < -1)
            {
              Perr = -1;
            }

            if(abs(Perr) > 0.01)
            {
              if(dac_Vref < 10)
              {
                dac_Vref += 100;
              }
              dac_Vref = dac_Vref * (1 + Perr / 2)+1;
            }
          }*/
