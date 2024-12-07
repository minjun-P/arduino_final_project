#include <IRremote.h>

#define IR_RECEIVE_PIN 7

// 전송 상태를 알리는 RGB 핀
#define RED 9
#define GREEN 10
#define BLUE 11


const uint8_t address = 0x78; // 주소 8비트 -> 내 생일 값으로 아무렇게나 설정, 수신부와 공유할 예정 decimal로는 120
const uint8_t forwardCommand = 0x11; // 별 의미 없음. decimal로는 17
const uint8_t reverseCommand = 0x12; // 별 의미 없음. decimal로는 18

void setup() {
  Serial.begin(115200);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  IrReceiver.begin(IR_RECEIVE_PIN);
  turnOnRgb(0,0,0);
}

void loop() {
  if (IrReceiver.decode()) {
    IrReceiver.resume(); // 다음 패킷 받을 준비
    const uint16_t receivedAddr = IrReceiver.decodedIRData.address;
    if (receivedAddr == address) {
      const uint16_t command = IrReceiver.decodedIRData.command;
      if (command == forwardCommand) {
        // 직진
        turnOnRgb(0,255,0);
        delay(500);
        turnOnRgb(0,0,0);
      } else if (command == reverseCommand) {
        // 후진
        turnOnRgb(0,0,255);
        delay(500);
        turnOnRgb(0,0,0);
      } else {
        // do nothing
      }
    }
    Serial.print("Address : ");
    Serial.println(address);

    Serial.println(address == 0x78);

    Serial.print("Command : ");
    Serial.println(IrReceiver.decodedIRData.command);
  }
}


void turnOnRgb(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED, 255-r);
  analogWrite(GREEN, 255-g);
  analogWrite(BLUE, 255-b);
}
