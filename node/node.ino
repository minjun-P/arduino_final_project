#include <IRremote.h>
#include <NewPing.h>


// 핀 설정
#define DISTANCE_PIN 12
#define IR_RECEIVE_PIN 7
#define RED 9
#define GREEN 10
#define BLUE 11
#define PWMA 6
#define AIN2 5
#define AIN1 4
#define STBY 3

const uint8_t address = 0x78;       // IR 송신 주소
const uint8_t forwardCommand = 0x11; // 전진 명령
const uint8_t reverseCommand = 0x12; // 후진 명령

// 이후 명령 임시 저장 버퍼 변수
uint8_t activeMotorCommand = 0; // 현재 모터가 실행중이면 0이 아님 
uint8_t commandBuffer = 0;

unsigned long motorStartTime = 0; // 모터 시작 시간
const unsigned long motorDuration = 2000; // 모터 동작 시간 (2초)

const int maxDistance = 50;
NewPing distanceSensor(DISTANCE_PIN, DISTANCE_PIN, 400);

unsigned long prevDistanceSensorTime = 0;
const unsigned long distanceSensorInterval = 100;
float distance = -1; // 의도적으로 음수로 초기화

void setup() {
    Serial.begin(115200);
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
    IrReceiver.begin(IR_RECEIVE_PIN);
    turnOnRgb(0, 0, 0);

    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
}

void loop() {

    // 1. IR 신호 수신
    if (IrReceiver.decode()) {
        const uint16_t receivedAddr = IrReceiver.decodedIRData.address;
        if (receivedAddr == address) {
            const uint16_t command = IrReceiver.decodedIRData.command;
            handleCommand(command); // 명령 처리
        }
        IrReceiver.resume(); // 다음 신호 수신 준비
    }

    
   const boolean motorRunning = (activeMotorCommand != 0);

    // 거리 측정 및 모터 컨트롤
    if (millis() - prevDistanceSensorTime >=distanceSensorInterval) {
      prevDistanceSensorTime = millis();
      distance = distanceSensor.ping_cm();
      Serial.print("거리 :");
      Serial.println(distance);
      if (distance <=2) {
        Serial.println("2 미만 거리!!");
        if (activeMotorCommand == forwardCommand) {
          Serial.println("모터 긴급 정지");
          stopMotor();
          turnOnRgb(0, 0, 0);
        }
        
      }
    }

   

    //  모터 동작 관리
    if (motorRunning && millis() - motorStartTime >= motorDuration) {
        stopMotor();
        turnOnRgb(0, 0, 0);

        // 큐에 명령이 있으면 다음 명령 실행
        if (commandBuffer != 0) {
          // 버퍼 처리
          uint8_t nextCommand = commandBuffer;
          commandBuffer = 0;
          executeCommand(nextCommand);
        }
    }
}

// 명령 처리 함수
void handleCommand(uint8_t command) {
  const boolean motorRunning = activeMotorCommand != 0;
  if (!motorRunning) {
    // 모터가 동작 중이 아닐 때 바로 실행
    executeCommand(command);
  } else {
    // 모터가 동작 중이면, 현재 동작 중인 명령과 다른 명령일 경우에만 저장
    if (activeMotorCommand != command) {
      commandBuffer = command;
      Serial.println("명령이 버퍼에 저장됨");
    } else {
      Serial.println("새로 명령이 들어왔지만 지금 하고 있는거랑 같은 명령이라 무시");
      Serial.print("현재 실행 중 :");
      if (activeMotorCommand == forwardCommand){
        Serial.println("전진");
      } else if (activeMotorCommand == reverseCommand) {
        Serial.println("후진");
      }
    }
  }
}

// 명령 실행 함수
void executeCommand(uint8_t command) {
    if (command == forwardCommand) {
        Serial.println("전진 명령 실행");
        turnOnRgb(0, 255, 0);
        startMotor(true);
    } else if (command == reverseCommand) {
        Serial.println("후진 명령 실행");
        turnOnRgb(0, 0, 255);
        startMotor(false);
    }
}

// 모터 전진 또는 후진 시작
void startMotor(bool forward) {
    motorStartTime = millis();
    digitalWrite(STBY, HIGH);
    analogWrite(PWMA, 255);
    if (forward) {
      activeMotorCommand = forwardCommand;
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
      activeMotorCommand = reverseCommand;
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
}

// 모터 정지
void stopMotor() {
  activeMotorCommand = 0;
  digitalWrite(STBY, LOW);
  Serial.println("모터 정지");
}

// RGB 제어 함수
void turnOnRgb(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED, 255 - r);
  analogWrite(GREEN, 255 - g);
  analogWrite(BLUE, 255 - b);
}
