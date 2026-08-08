# Tinkercad Simulation

EV 배터리 열폭주 조기 감지 및 능동형 안전 분리 시스템의
Arduino Uno 기반 Tinkercad 시뮬레이션입니다.

## 구현 기능

- TMP36 기반 배터리 온도 모사
- 가변저항 기반 전류 데이터 모사
- 가변저항 기반 배터리 전압 데이터 모사
- NORMAL / WARNING / DANGER 상태 판단
- 온도 급상승 + 과전류/전압 강하 복합 위험 판단
- 상태별 LED 표시
- Piezo Buzzer 경고
- WARNING 상태 차량 감속
- DANGER 상태 차량 정지
- Servo Motor 2개를 이용한 Battery Release 모사
- 배터리 분리 후 ESCAPE 주행
- Emergency Latch
- HC-SR04 기반 장애물 감지 및 회피

## 상태 제어

### NORMAL
- Green LED ON
- 정상 속도 주행
- 장애물 감지 및 회피
- Battery LOCK

### WARNING
- Yellow LED ON
- Piezo Buzzer 경고
- 차량 감속
- Battery LOCK 유지

### DANGER
다음 복합 조건을 이용하여 위험 상태를 판단합니다.

- 온도 상승 속도 ≥ 8°C/s
- AND
- 전류 ≥ 16A 또는 전압 ≤ 8V

위험 감지 시:

- Red LED ON
- 차량 정지
- 경고음 발생
- Servo Motor 작동
- Battery RELEASE

### ESCAPE
Battery Release 이후:

1. 차량 정지
2. Battery Release
3. 2초 대기
4. 저속 ESCAPE 주행
5. 3초 후 정지
6. SYSTEM LOCKED

## 장애물 회피

HC-SR04 초음파 센서를 이용합니다.

- Distance > 30cm → 정상 주행
- Distance ≤ 30cm → 장애물 감지
- 정지 후 회전 동작 수행

## 시뮬레이션 부품

| 부품 | 수량 |
|---|---:|
| Arduino Uno R3 | 1 |
| TMP36 | 1 |
| Potentiometer | 2 |
| HC-SR04 | 1 |
| L293D | 1 |
| DC Motor | 2 |
| Micro Servo | 2 |
| Piezo Buzzer | 1 |
| LED | 3 |
| 220Ω Resistor | 3 |

## 핀 구성

| Arduino | 기능 |
|---|---|
| A0 | TMP36 |
| A1 | Current Simulation |
| A2 | Battery Voltage Simulation |
| A3 | HC-SR04 TRIG |
| A4 | HC-SR04 ECHO |
| D3 | Motor Enable PWM |
| D4 | Green LED |
| D5 | Yellow LED |
| D6 | Red LED |
| D7 | Piezo Buzzer |
| D8 | Servo 1 |
| D9 | Servo 2 |
| D10 | L293D Input 1 |
| D11 | L293D Input 2 |
| D12 | L293D Input 3 |
| D13 | L293D Input 4 |

## 실제 하드웨어와의 차이

Tinkercad에서는 실제 센서 일부를 사용할 수 없어 다음과 같이 대체하여
동작 로직을 검증했습니다.

| 실제 구현 | Simulation |
|---|---|
| NTC Thermistor | TMP36 |
| ACS712 Current Sensor | Potentiometer |
| Voltage Divider | Potentiometer |

본 시뮬레이션은 실제 EV 배터리를 분리하는 장치가 아니라
센서 판단, FSM, 모터 제어 및 능동 분리 로직을 검증하기 위한 프로토타입입니다.
