#include <Arduino.h>

// 적외선 및 통신 관련 
#if !defined(ARDUINO_ESP32C3_DEV) // This is due to a bug in RISC-V compiler, which requires unused function sections :-(.
#define DISABLE_CODE_FOR_RECEIVER // Disables static receiver code like receive timer ISR handler and static IRReceiver and irparams data. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not required.
#endif
#include <IRremote.hpp>

#define IR_SEND_PIN              4

// 온,습도 센서 관련  DHT 센서 설정
#include "DHT.h"

#define DHTPIN 14      
#define DHTTYPE DHT11 // 사용 중인 센서 유형 설정하기 (DHT11, DHT22 등)

DHT dht(DHTPIN, DHTTYPE);

// LCD 세팅
#include<LiquidCrystal_I2C_Hangul.h>
#include<Wire.h>

LiquidCrystal_I2C_Hangul lcd(0x27,16,2); //LCD 클래스 초기화



#define FORWARD_COMMAND_PIN      34
#define REVERSE_COMMAND_PIN      35

// 전송 상태를 알리는 RGB 핀
#define RED 25
#define GREEN 26
#define BLUE 27

// 전역 상수
const uint8_t irAddress = 0x78; // 주소 8비트 -> 내 생일 값으로 아무렇게나 설정, 수신부와 공유할 예정
const uint8_t forwardCommand = 0x11; // 별 의미 없음. decimal로는 17
const uint8_t reverseCommand = 0x12; // 별 의미 없음. decimal로는 18
const uint8_t repeat = 2; // 적당히 있는게 안정성에 도움이 된다길래 2 정도로...

uint8_t activeCommand = 0; 
uint8_t prevCommand = 0;

void IRAM_ATTR commandForwardWithInterrupt() {
  Serial.println("commandForward with interrupt");
  activeCommand = forwardCommand;
}

void IRAM_ATTR commandReverseWithInterrupt() {
  Serial.println("commandReverse with interrupt");
  activeCommand = reverseCommand;
}

void setup() {
  dht.begin(); // DHT 센서 초기화
  // 스위치 핀
  pinMode(FORWARD_COMMAND_PIN, INPUT);
  pinMode(REVERSE_COMMAND_PIN, INPUT); 
  // 스위치 핀에 interrupt 세팅
  attachInterrupt(digitalPinToInterrupt(FORWARD_COMMAND_PIN),commandForwardWithInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(REVERSE_COMMAND_PIN),commandReverseWithInterrupt, RISING);


  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  turnOnRgb(0,0,0);

  Serial.begin(115200);
  while (!Serial) // 시리얼 초기화될 때까지 기다리기
  Serial.print(IR_SEND_PIN);
  Serial.println(F("을 통해 데이터 송신"));

  IrSender.begin(IR_SEND_PIN);
  disableLEDFeedback(); // Disable feedback LED at default feedback LED pin


  lcd.init();
  lcd.backlight();
  lcd.print("Hello Park!");
}
// 주로 통제하는 전역 변수 모음
float humidity; // 습도 저장
float temperature; // 온도 저장

// 타이밍 관련 값들
// 온습도 센서
unsigned long prevSensorTime = 0;
const unsigned int sensorInterval = 5000; // 5초에 한번 새로운 값

// LCD
unsigned long prevLcdTime = 0;
const unsigned int lcdInterval = 1000; // 1초에 한번 화면 업데이트

// IR 및 LED 제어
unsigned long prevIrTime = 0;
const unsigned int irInterval = 500;


void loop() {
  unsigned long currentMillis = millis();

  // 온습도 센서 읽기
  if (currentMillis - prevSensorTime > sensorInterval) {
    Serial.println("온습도 5초에 한번");
    prevSensorTime = currentMillis;
    readAndUpdateTmepAndHumidity();
    if (humidity < 40) {
      Serial.println("40언더");
      commandForward();
    } else {
      Serial.println("40이상");
      commandReverse();
    }
  }

  if (currentMillis - prevLcdTime > lcdInterval) {
    Serial.println("LCD 업뎃 1초에 한번");
    prevLcdTime = currentMillis;
    updateLcd();
  }

  // 이 간격으로 Ir 보낼지 체크하기
  if (currentMillis - prevIrTime > irInterval) {
    prevIrTime = currentMillis;
    Serial.println("IR 체크 0.5초");
    Serial.println(activeCommand);
    Serial.println(prevCommand);
    Serial.println("----");
    // 현재 Active가 존재하고, 직전 명령과 다르면 실행
    if (activeCommand != 0 && activeCommand != prevCommand) {
      Serial.println("IR 신호 보내야 함");
      if (activeCommand == forwardCommand) {
        turnOnRgb(0,255,0);
        IrSender.sendNEC(irAddress, forwardCommand, repeat); // FORWARD 명령 송신
      } else if (activeCommand == reverseCommand) {
        turnOnRgb(0,0,255);
        IrSender.sendNEC(irAddress, reverseCommand, repeat); // REVERSE 명령 송신
      } else {
        // 요상한 명령이 들어왔어요 turnDownRgb
        turnOnRgb(0,0,0);
      }
      prevCommand = activeCommand;
      activeCommand = 0;
   } else {
    // ActiveCommand가 없는 상황이라면 조용히 Led를 꺼주자.
    turnOnRgb(0,0,0);
   }
  }
}


void commandForward() {
  Serial.println("commandForward");
  activeCommand = forwardCommand;
}



void commandReverse() {
  Serial.println("commandReverse");
  activeCommand = reverseCommand;
}

void readAndUpdateTmepAndHumidity() {
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}

void updateLcd() {
  if (isnan(temperature) || isnan(humidity)) {
    displayError();
  } else {
    int countdownForSensor = (millis() - prevSensorTime)/1000;
    displayInfo(temperature, humidity, countdownForSensor);
  }
}

void turnOnRgb(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED, 255-r);
  analogWrite(GREEN, 255-g);
  analogWrite(BLUE, 255-b);
}

void displayInfo(float temperature, float humidity, int countdown) {
  lcd.clear();
  lcd.print(temperature, 1); // 온도 소수점 1자리까지 표시
  lcd.print("C, / ");
  lcd.print(humidity, 1); // 습도 소수점 1자리까지 표시
  lcd.print("%");

  lcd.setCursor(0, 1);
  if (countdown == 0) {
    lcd.print("New Value!");
  } else {
    lcd.print(5-countdown);
    lcd.print("...");
  }
}

void displayError() {
  lcd.clear();
  lcd.print("에러 발생");
  lcd.setCursor(0,1);
  lcd.print("센서 값 XX");
}
