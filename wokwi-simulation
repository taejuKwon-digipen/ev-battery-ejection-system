# EV 배터리 능동형 안전 분리 시스템 - Wokwi 시뮬레이션

## 프로젝트 소개

본 프로젝트는 EV 배터리의 열폭주를 조기에 감지하고, 위험 상황에서 배터리를 차량으로부터 물리적으로 분리하여 2차 피해를 최소화하는 것을 목표로 한다.

온도 센서와 전류 센서를 이용하여 배터리 상태를 실시간으로 모니터링하고, FSM(Finite State Machine) 기반 알고리즘을 통해 위험 상태를 판단한다.

위험 상태가 감지되면 LED와 부저를 통해 경고를 발생시키고, 서보모터를 이용한 배터리 분리 메커니즘을 작동시킨다.

---

## 하드웨어 구성

| 부품 | 수량 |
| --- | --- |
| Arduino Uno | 1 |
| NTC 온도 센서 | 1 |
| 가변저항 | 1 |
| LED | 1 |
| 부저 | 1 |
| 서보모터 | 2 |

---

## 회로 구성

![회로도](./project_circuit.png)

---

## 핀 연결

| 부품 | 아두이노 핀 |
| --- | --- |
| NTC 온도 센서 | A0 |
| 가변저항 | A1 |
| LED | D7 |
| 부저 | D8 |
| 서보모터 1 | D9 |
| 서보모터 2 | D10 |

---

## FSM 상태 전이

```text
NORMAL
↓
WARNING
↓
DANGER
```

---

## 상태 설명

| 상태 | 동작 |
| --- | --- |
| NORMAL | 정상 상태 |
| WARNING | 위험 감지 및 경고 |
| DANGER | 배터리 분리 장치 작동 |

---

## 위험 감지 알고리즘

```cpp
#include <Servo.h>

const int tempPin = A0;
const int currentPin = A1;

Servo servo1;
Servo servo2;

enum State {
  NORMAL,
  WARNING,
  DANGER
};

State state = NORMAL;

void setup() {
  Serial.begin(9600);

  servo1.attach(9);
  servo2.attach(10);

  pinMode(7, OUTPUT);

  servo1.write(0);
  servo2.write(0);
}

void loop() {
  int temp = analogRead(tempPin);
  int current = analogRead(currentPin);

  if (temp < 650 && current > 500) {
    state = DANGER;
  } else if (temp < 700) {
    state = WARNING;
  } else {
    state = NORMAL;
  }

  if (state == NORMAL) {
    digitalWrite(7, LOW);

    servo1.write(0);
    servo2.write(0);
  }

  if (state == WARNING) {
    digitalWrite(7, HIGH);

    servo1.write(0);
    servo2.write(0);
  }

  if (state == DANGER) {
    digitalWrite(7, HIGH);

    servo1.write(90);
    servo2.write(90);
  }

  Serial.print("Temperature: ");
  Serial.print(temp);

  Serial.print(" Current: ");
  Serial.print(current);

  Serial.print(" State: ");

  if (state == NORMAL)
    Serial.println("NORMAL");

  if (state == WARNING)
    Serial.println("WARNING");

  if (state == DANGER)
    Serial.println("DANGER");

  delay(100);
}
```

---

## 배터리 분리 메커니즘

두 개의 서보모터가 배터리를 지지한다.

위험 상태가 감지되면 두 개의 서보모터가 동시에 90° 회전하여 배터리를 분리한다.

```text
잠금
↓
해제
↓
배터리 분리
```

---

## 시리얼 모니터

![시리얼 모니터](./serial_monitor.png)

---

## 시뮬레이션 동작 과정

```text
센서 데이터 수집
↓
온도 모니터링
↓
전류 모니터링
↓
WARNING
↓
LED 경고
↓
DANGER
↓
서보모터 회전
↓
배터리 분리
```

---


## 시뮬레이션 결과

- NTC 센서를 이용한 실시간 온도 모니터링
- 가변저항을 이용한 전류 이상 상황 시뮬레이션
- FSM 기반 위험 상태 판단
- LED를 이용한 시각적 경고
- 서보모터를 이용한 배터리 분리
- 시리얼 모니터를 통한 실시간 상태 확인
