#include <Servo.h>

// =========================
// 센서 핀
// =========================
const int tempPin = A0;
const int currentPin = A1;
const int batteryPin = A2;

// 초음파
const int trigPin = A3;
const int echoPin = A4;

// =========================
// LED / 부저
// =========================
const int greenLED = 4;
const int yellowLED = 5;
const int redLED = 6;
const int buzzer = 7;

// =========================
// 서보
// =========================
Servo servo1;
Servo servo2;

// =========================
// L293D 모터
// =========================
const int motorEnable = 3;

const int motor1A = 10;
const int motor1B = 11;

const int motor2A = 12;
const int motor2B = 13;

// =========================
// 위험 기준
// =========================
const float TEMP_RISE_LIMIT = 8.0;

const float HIGH_CURRENT = 16.0;
const float LOW_VOLTAGE = 8.0;

const float WARNING_TEMP = 50.0;
const float WARNING_CURRENT = 12.0;
const float WARNING_VOLTAGE = 10.0;

// 장애물 기준
const float OBSTACLE_DISTANCE = 30.0;

// =========================
// 온도 상승 속도
// =========================
float previousTemperature = 0;
unsigned long previousTempTime = 0;

// =========================
// 상태 변수
// =========================
bool escapeDone = false;
bool emergencyLatched = false;


// =========================
// 초기 설정
// =========================
void setup()
{
  Serial.begin(9600);

  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // 초음파
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // 서보
  servo1.attach(8);
  servo2.attach(9);

  servo1.write(0);
  servo2.write(0);

  // 모터
  pinMode(motorEnable, OUTPUT);

  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);

  previousTempTime = millis();
}


// =========================
// 초음파 거리 측정
// =========================
float getDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration =
    pulseIn(echoPin, HIGH, 30000);

  // 초음파 신호가 안 돌아온 경우
  if (duration == 0)
  {
    return 999;
  }

  float distance =
    duration * 0.0343 / 2;

  return distance;
}


// =========================
// NORMAL 직진
// =========================
void driveNormal()
{
  analogWrite(motorEnable, 255);

  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);

  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
}


// =========================
// WARNING 감속
// =========================
void driveWarning()
{
  analogWrite(motorEnable, 128);

  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);

  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
}


// =========================
// ESCAPE 저속 주행
// =========================
void driveEscape()
{
  analogWrite(motorEnable, 90);

  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);

  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
}


// =========================
// 우회전
// =========================
void turnRight()
{
  analogWrite(motorEnable, 160);

  // 왼쪽 모터 전진
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);

  // 오른쪽 모터 후진
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
}


// =========================
// 모터 정지
// =========================
void stopMotors()
{
  analogWrite(motorEnable, 0);

  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);

  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
}


// =========================
// 메인
// =========================
void loop()
{
  // --------------------------------
  // 1. 온도
  // --------------------------------
  int tempValue =
    analogRead(tempPin);

  float tempVoltage =
    tempValue * (5.0 / 1023.0);

  float temperature =
    (tempVoltage - 0.5) * 100.0;


  // --------------------------------
  // 2. 전류
  // --------------------------------
  int currentValue =
    analogRead(currentPin);

  float current =
    currentValue * (20.0 / 1023.0);


  // --------------------------------
  // 3. 배터리 전압
  // --------------------------------
  int batteryValue =
    analogRead(batteryPin);

  float batteryVoltage =
    batteryValue * (12.0 / 1023.0);


  // --------------------------------
  // 4. 장애물 거리
  // --------------------------------
  float distance =
    getDistance();


  // =================================
  // 온도 상승 속도
  // =================================
  unsigned long currentTime =
    millis();

  float timeDifference =
    (currentTime - previousTempTime)
    / 1000.0;

  float tempRiseRate = 0;

  if (timeDifference >= 1.0)
  {
    tempRiseRate =
      (temperature - previousTemperature)
      / timeDifference;

    previousTemperature =
      temperature;

    previousTempTime =
      currentTime;
  }


  // =================================
  // 위험 조건
  // =================================
  bool rapidTemperatureRise =
    tempRiseRate >= TEMP_RISE_LIMIT;

  bool overCurrent =
    current >= HIGH_CURRENT;

  bool voltageDrop =
    batteryVoltage <= LOW_VOLTAGE;

  bool dangerCondition =
    rapidTemperatureRise &&
    (overCurrent || voltageDrop);


  // DANGER 발생 시 고정
  if (dangerCondition)
  {
    emergencyLatched = true;
  }


  String state;


  // =================================
  // EMERGENCY
  // =================================
  if (emergencyLatched)
  {
    state = "EMERGENCY";

    digitalWrite(greenLED, LOW);
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, HIGH);

    // 분리 상태 유지
    servo1.write(90);
    servo2.write(90);


    if (escapeDone == false)
    {
      // 먼저 정지
      stopMotors();

      tone(buzzer, 1500);

      Serial.println(
        "!!! DANGER DETECTED !!!");

      Serial.println(
        "!!! BATTERY RELEASE !!!");

      // 2초 대기
      delay(2000);

      noTone(buzzer);

      Serial.println(
        "STATE: ESCAPE");

      // 대피
      driveEscape();

      delay(3000);

      // 대피 완료
      stopMotors();

      Serial.println(
        "ESCAPE COMPLETE");

      Serial.println(
        "SYSTEM LOCKED");

      escapeDone = true;
    }

    else
    {
      stopMotors();
      noTone(buzzer);
    }
  }


  // =================================
  // WARNING
  // =================================
  else if (
           temperature >= WARNING_TEMP ||
           current >= WARNING_CURRENT ||
           batteryVoltage <= WARNING_VOLTAGE
          )
  {
    state = "WARNING";

    digitalWrite(greenLED, LOW);
    digitalWrite(yellowLED, HIGH);
    digitalWrite(redLED, LOW);

    tone(buzzer, 800);

    // 감속
    driveWarning();

    // 배터리 LOCK
    servo1.write(0);
    servo2.write(0);
  }


  // =================================
  // NORMAL
  // =================================
  else
  {
    // 배터리 LOCK
    servo1.write(0);
    servo2.write(0);

    noTone(buzzer);

    digitalWrite(greenLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(redLED, LOW);


    // -------------------------
    // 장애물 있음
    // -------------------------
    if (distance <= OBSTACLE_DISTANCE)
    {
      state = "OBSTACLE";

      Serial.println(
        "!!! OBSTACLE DETECTED !!!");

      // 일단 정지
      stopMotors();

      delay(500);

      // 오른쪽으로 회전
      turnRight();

      delay(1000);

      // 다시 정지
      stopMotors();

      delay(300);
    }

    // -------------------------
    // 장애물 없음
    // -------------------------
    else
    {
      state = "NORMAL";

      driveNormal();
    }
  }


  // =================================
  // 시리얼 모니터
  // =================================
  Serial.print("Temp: ");
  Serial.print(temperature);

  Serial.print(" C | Rise: ");
  Serial.print(tempRiseRate);

  Serial.print(" C/s | Current: ");
  Serial.print(current);

  Serial.print(" A | Battery: ");
  Serial.print(batteryVoltage);

  Serial.print(" V | Distance: ");
  Serial.print(distance);

  Serial.print(" cm | STATE: ");
  Serial.println(state);


  delay(200);
}
