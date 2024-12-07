#include <Arduino.h>

#if !defined(ARDUINO_ESP32C3_DEV) // This is due to a bug in RISC-V compiler, which requires unused function sections :-(.
#define DISABLE_CODE_FOR_RECEIVER // Disables static receiver code like receive timer ISR handler and static IRReceiver and irparams data. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not required.
#endif
#include <IRremote.hpp>

#define IR_SEND_PIN              4

#define FORWARD_COMMAND_PIN      34
#define REVERSE_COMMAND_PIN      35

// 전송 상태를 알리는 RGB 핀
#define RED 25
#define GREEN 26
#define BLUE 27


const uint8_t address = 0x78; // 주소 8비트 -> 내 생일 값으로 아무렇게나 설정, 수신부와 공유할 예정
const uint8_t forwardCommand = 0x11; // 별 의미 없음. decimal로는 17
const uint8_t reverseCommand = 0x12; // 별 의미 없음. decimal로는 18
const uint8_t repeat = 2; // 적당히 있는게 안정성에 도움이 된다길래 2 정도로...


void setup() {
  pinMode(FORWARD_COMMAND_PIN, INPUT);
  pinMode(REVERSE_COMMAND_PIN, INPUT); 

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
}

void loop() {
    
  // FORWARD_COMMAND_PIN이 눌렸는지 확인
  if (digitalRead(FORWARD_COMMAND_PIN) == HIGH) { 
    turnOnRgb(0,255,0);
    Serial.println(F("FORWARD 명령 송신 중..."));
    IrSender.sendNEC(address, forwardCommand, repeat); // FORWARD 명령 송신
    delay(500); // 신호 중복 방지를 위한 딜레이
    turnOnRgb(0,0,0);
  }

  // REVERSE_COMMAND_PIN이 눌렸는지 확인
  if (digitalRead(REVERSE_COMMAND_PIN) == HIGH) { 
  turnOnRgb(0,0,255);
    Serial.println(F("REVERSE 명령 송신 중..."));
    IrSender.sendNEC(address, reverseCommand, repeat); // REVERSE 명령 송신
    delay(500); // 신호 중복 방지를 위한 딜레이
    turnOnRgb(0,0,0);
  }
}

void turnOnRgb(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED, 255-r);
  analogWrite(GREEN, 255-g);
  analogWrite(BLUE, 255-b);
}
