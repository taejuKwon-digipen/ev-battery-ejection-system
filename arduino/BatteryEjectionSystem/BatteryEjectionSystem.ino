/*
  EV 배터리 열폭주 조기 감지 및 능동형 안전 분리(Ejection) 시스템
  ------------------------------------------------------------
  핀 배선
    A0  - NTC 서미스터 (분압, SERIES_RESISTOR와 직렬)
    A1  - ACS712 전류센서 OUT
    A2  - 배터리 전압 분압 회로 OUT
    D2  - 리셋 버튼 (INPUT_PULLUP, 눌리면 LOW) - EJECTED 상태 복귀용
    D3  - 냉각팬 구동 (릴레이/트랜지스터 게이트) - WARNING 시 예방적 냉각
    D4  - 정상 LED (녹색)
    D5  - 경고 LED (황색)
    D6  - 위험 LED (적색)
    D7  - 부저
    D8  - 모터 드라이버 EN (LOW = 주행 모터 정지)
    D9  - 래치 서보 (SG90 등)
    D10 - 충전 릴레이 제어 (HIGH = 충전 허용, LOW = 충전 회로 차단)
    D11 - 초음파 센서 TRIG (HC-SR04) - 후방/하부 클리어런스 확인
    D12 - 초음파 센서 ECHO (HC-SR04)
    D13 - 분리 확인 버튼 (INPUT_PULLUP, 눌리면 LOW) - 사람이 있을 때 수동으로 즉시 분리 확정

  충전 감지: ACS712는 양방향 센서이므로, 배터리를 향해 흐르는 방전 전류를 양(+)으로
  캘리브레이션했다고 가정하면 충전 전류는 음(-)으로 측정된다(CHARGE_CURRENT_THRESHOLD_A).
  실제 배선 방향에 따라 부호가 반대일 수 있으니 실측 후 확인할 것.

  상태 전이(FSM)
    NORMAL  -> WARNING : 온도/전류/전압이 경고 임계값 초과 -> 냉각팬 가동 + 충전 중이면 즉시 충전 차단(예방 시도)
    WARNING -> NORMAL  : 냉각팬 가동 후 모든 값이 정상 범위로 복귀 (예방 성공, 충전 재개)
    WARNING -> STOPPED : 냉각/충전차단에도 불구하고 위험 임계값(과열/온도급상승/과전류/전압이상) 도달 (예방 실패 -> 격리 절차 시작)
    NORMAL  -> STOPPED : 위험 임계값에 바로 도달한 경우 (예: 충전 중 과전압/과전류 급증)
    STOPPED -> ARMED   : 대피 유예시간(EJECT_DELAY_MS) 경과 - 분리 준비 완료, 실제 분리는 아직 보류
    ARMED   -> EJECTED : (후방 초음파로 확인한 안전거리 확보 OR 사람이 확인버튼 누름) 조건 충족 시에만 실제 분리
    ARMED   -> NORMAL  : 리셋 버튼 - 오탐이었을 경우 분리 취소
    EJECTED -> NORMAL  : 리셋 버튼(재장착 후 테스트용)

  설계 원칙
    1) 격리(Ejection)는 최후 수단이다. WARNING 단계에서 먼저 냉각팬 + 충전 차단으로
       열폭주 진행 자체를 막는 것을 시도하고, 그래도 진행되면 STOPPED로 넘어간다.
       (주차장에서 충전 중 과충전으로 발화하는 시나리오가 충전 차단을 넣은 계기)
    2) "정차했다고 바로 떨어뜨린다"는 위험하다 - 지하주차장 등에서 옆 차량/사람에게
       배터리가 떨어질 수 있다. 그래서 STOPPED 이후 바로 EJECTED로 가지 않고 ARMED를
       거친다: 분리 실행 = (냉각 실패로 STOPPED까지 옴) AND (후방 클리어 확인 OR 수동 확인).
       초음파가 무응답이면 "안전 미확인"으로 간주해 분리를 보류하는 fail-safe 설계.
*/

#include <Servo.h>

// ---------------- 핀 정의 ----------------
const uint8_t PIN_NTC        = A0;
const uint8_t PIN_CURRENT    = A1;
const uint8_t PIN_VOLTAGE    = A2;
const uint8_t PIN_RESET_BTN  = 2;
const uint8_t PIN_COOLING_FAN = 3;
const uint8_t PIN_LED_NORMAL = 4;
const uint8_t PIN_LED_WARN   = 5;
const uint8_t PIN_LED_DANGER = 6;
const uint8_t PIN_BUZZER     = 7;
const uint8_t PIN_MOTOR_EN   = 8;
const uint8_t PIN_SERVO      = 9;
const uint8_t PIN_CHARGE_RELAY = 10;
const uint8_t PIN_ULTRASONIC_TRIG   = 11;
const uint8_t PIN_ULTRASONIC_ECHO   = 12;
const uint8_t PIN_EJECT_CONFIRM_BTN = 13;

// ---------------- 센서 캘리브레이션 (실측 후 조정) ----------------
// NTC: B=3950, 25도 기준 저항 10k, 분압저항 10k
const float NTC_NOMINAL_RES    = 10000.0;
const float NTC_NOMINAL_TEMP_C = 25.0;
const float NTC_B_COEFFICIENT  = 3950.0;
const float NTC_SERIES_RESISTOR = 10000.0;

// ACS712 5A 모듈 기준 (185mV/A, 무전류 시 Vcc/2 출력)
const float ACS712_SENSITIVITY_V_PER_A = 0.185;
const float ACS712_ZERO_CURRENT_V      = 2.5;

// 전압 분압비 (예: 100k + 10k 분압 -> 최대 약 55V 측정 가능)
const float VOLTAGE_DIVIDER_RATIO = 11.0;

const float ADC_REF_VOLTAGE = 5.0;
const int   ADC_MAX_COUNT   = 1023;

// ---------------- 안전 임계값 (실측 후 조정) ----------------
const float TEMP_WARNING_C      = 55.0;
const float TEMP_CRITICAL_C     = 70.0;
const float TEMP_RISE_RATE_C_S  = 1.0;   // 초당 상승률이 이 값을 넘으면 열폭주 전조로 판단
const float CURRENT_WARNING_A   = 6.0;
const float CURRENT_CRITICAL_A  = 10.0;
const float VOLTAGE_MIN_V       = 9.0;   // 급격한 전압강하 = 내부 단락 의심
const float VOLTAGE_MAX_V       = 16.8;  // 과충전 의심
const float CHARGE_CURRENT_THRESHOLD_A = -0.3; // 이보다 더 음수면 충전 중으로 판단
const float SAFE_EJECT_DISTANCE_CM = 30.0; // 후방/하부가 이 거리보다 가까이 막혀 있으면 분리 보류
const unsigned long ULTRASONIC_TIMEOUT_US = 30000UL; // 약 5m 초과 무응답 시 타임아웃

// ---------------- 타이밍 ----------------
const unsigned long SENSOR_SAMPLE_INTERVAL_MS = 100;   // 센서 샘플링 주기
const unsigned long RISE_RATE_WINDOW_MS       = 2000;  // 온도 상승률 계산 기준 구간
const unsigned long EJECT_DELAY_MS            = 3000;  // STOPPED 후 분리까지 대피 유예시간
const unsigned long TELEMETRY_INTERVAL_MS     = 200;   // 시리얼 로그 주기
const unsigned long BUTTON_DEBOUNCE_MS        = 50;

// ---------------- 서보 각도 ----------------
const int SERVO_LOCK_ANGLE    = 0;
const int SERVO_RELEASE_ANGLE = 90;

// ---------------- 이동평균 필터 ----------------
const uint8_t FILTER_SIZE = 8;

struct MovingAverage {
  float buffer[FILTER_SIZE];
  uint8_t index = 0;
  uint8_t count = 0;
  float sum = 0;

  float update(float value) {
    sum -= buffer[index];
    buffer[index] = value;
    sum += value;
    index = (index + 1) % FILTER_SIZE;
    if (count < FILTER_SIZE) count++;
    return sum / count;
  }
};

MovingAverage tempFilter;
MovingAverage currentFilter;
MovingAverage voltageFilter;

// ---------------- FSM ----------------
enum SystemState { NORMAL, WARNING, STOPPED, ARMED, EJECTED };
SystemState state = NORMAL;
unsigned long stateEnteredAt = 0;

long lastDistanceCm = -1; // 마지막으로 측정한 후방 거리(cm), -1 = 무응답/미측정

Servo latchServo;

unsigned long lastSampleAt = 0;
unsigned long lastTelemetryAt = 0;

float filteredTempC = 25.0;
float filteredCurrentA = 0.0;
float filteredVoltageV = 12.0;

// 온도 상승률 계산용 이력
float tempAtWindowStart = 25.0;
unsigned long windowStartAt = 0;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_LED_NORMAL, OUTPUT);
  pinMode(PIN_LED_WARN, OUTPUT);
  pinMode(PIN_LED_DANGER, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_EN, OUTPUT);
  pinMode(PIN_COOLING_FAN, OUTPUT);
  pinMode(PIN_CHARGE_RELAY, OUTPUT);
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
  pinMode(PIN_EJECT_CONFIRM_BTN, INPUT_PULLUP);
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  latchServo.attach(PIN_SERVO);
  latchServo.write(SERVO_LOCK_ANGLE);

  digitalWrite(PIN_MOTOR_EN, HIGH); // 정상 주행 가능
  enterState(NORMAL);

  windowStartAt = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastSampleAt >= SENSOR_SAMPLE_INTERVAL_MS) {
    lastSampleAt = now;
    sampleSensors(now);
    updateStateMachine(now);
  }

  handleResetButton();
  handleEjectConfirmButton();
  updateIndicators(now);

  if (now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryAt = now;
    printTelemetry();
  }
}

// ---------------- 센서 읽기 ----------------
void sampleSensors(unsigned long now) {
  filteredTempC    = tempFilter.update(readTemperatureC());
  filteredCurrentA = currentFilter.update(readCurrentA());
  filteredVoltageV = voltageFilter.update(readVoltageV());

  if (now - windowStartAt >= RISE_RATE_WINDOW_MS) {
    windowStartAt = now;
    tempAtWindowStart = filteredTempC;
  }
}

float readTemperatureC() {
  int raw = analogRead(PIN_NTC);
  float voltage = raw * (ADC_REF_VOLTAGE / ADC_MAX_COUNT);
  float resistance = NTC_SERIES_RESISTOR * (ADC_REF_VOLTAGE / voltage - 1.0);

  float steinhart = resistance / NTC_NOMINAL_RES;
  steinhart = log(steinhart);
  steinhart /= NTC_B_COEFFICIENT;
  steinhart += 1.0 / (NTC_NOMINAL_TEMP_C + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  return steinhart;
}

float readCurrentA() {
  int raw = analogRead(PIN_CURRENT);
  float voltage = raw * (ADC_REF_VOLTAGE / ADC_MAX_COUNT);
  return (voltage - ACS712_ZERO_CURRENT_V) / ACS712_SENSITIVITY_V_PER_A;
}

float readVoltageV() {
  int raw = analogRead(PIN_VOLTAGE);
  float voltage = raw * (ADC_REF_VOLTAGE / ADC_MAX_COUNT);
  return voltage * VOLTAGE_DIVIDER_RATIO;
}

float currentTempRiseRate() {
  float elapsedSec = RISE_RATE_WINDOW_MS / 1000.0;
  return (filteredTempC - tempAtWindowStart) / elapsedSec;
}

// HC-SR04로 후방/하부 거리(cm) 측정. 무응답(범위 밖/센서 이상)이면 -1.
long readDistanceCm() {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) return -1;
  return (long)(duration / 58UL);
}

// "투하 가능 조건": 후방이 비어 있는지 확인. 무응답은 안전 미확인으로 간주해
// 분리를 보류시키는 fail-safe (모르면 떨어뜨리지 않는다).
bool isAreaClear() {
  lastDistanceCm = readDistanceCm();
  return lastDistanceCm >= 0 && lastDistanceCm >= (long)SAFE_EJECT_DISTANCE_CM;
}

// ---------------- FSM 로직 ----------------
// 충전 중에는 ACS712 부호가 반대(음수)로 나오므로, 절대값으로 비교해야
// "충전 중 과전류(과충전)"도 "주행 중 과전류(과방전)"와 동일하게 잡아낸다.
bool isCharging() {
  return filteredCurrentA <= CHARGE_CURRENT_THRESHOLD_A;
}

bool isCriticalCondition() {
  return filteredTempC >= TEMP_CRITICAL_C
      || currentTempRiseRate() >= TEMP_RISE_RATE_C_S
      || fabs(filteredCurrentA) >= CURRENT_CRITICAL_A
      || filteredVoltageV <= VOLTAGE_MIN_V
      || filteredVoltageV >= VOLTAGE_MAX_V;
}

bool isWarningCondition() {
  return filteredTempC >= TEMP_WARNING_C
      || fabs(filteredCurrentA) >= CURRENT_WARNING_A;
}

void updateStateMachine(unsigned long now) {
  switch (state) {
    case NORMAL:
      if (isCriticalCondition()) {
        enterState(STOPPED);
      } else if (isWarningCondition()) {
        enterState(WARNING);
      }
      break;

    case WARNING:
      if (isCriticalCondition()) {
        enterState(STOPPED);
      } else if (!isWarningCondition()) {
        enterState(NORMAL);
      }
      break;

    case STOPPED:
      if (now - stateEnteredAt >= EJECT_DELAY_MS) {
        enterState(ARMED); // 분리 준비 완료. 실제 분리는 안전 확인 후.
      }
      break;

    case ARMED:
      if (isAreaClear()) {
        enterState(EJECTED); // 무인 상황: 후방 클리어 자동 확인되면 분리 실행
      }
      // 클리어 안 되면 계속 ARMED 유지, 매 샘플마다 재확인
      // (수동 확인 버튼은 handleEjectConfirmButton()에서 즉시 처리)
      break;

    case EJECTED:
      // 리셋 버튼으로만 복귀 (handleResetButton)
      break;
  }
}

void enterState(SystemState newState) {
  state = newState;
  stateEnteredAt = millis();

  switch (newState) {
    case NORMAL:
      digitalWrite(PIN_MOTOR_EN, HIGH);
      digitalWrite(PIN_COOLING_FAN, LOW); // 예방 성공(정상 복귀) -> 냉각 종료
      digitalWrite(PIN_CHARGE_RELAY, HIGH); // 정상 범위 -> 충전 재개 허용
      latchServo.write(SERVO_LOCK_ANGLE);
      noTone(PIN_BUZZER);
      break;

    case WARNING:
      digitalWrite(PIN_MOTOR_EN, HIGH);
      digitalWrite(PIN_COOLING_FAN, HIGH); // 예방 1차 대응: 냉각팬 가동
      digitalWrite(PIN_CHARGE_RELAY, LOW); // 충전 중이었다면 즉시 차단(과충전 예방, 충전 중 아니어도 무해)
      break;

    case STOPPED:
      digitalWrite(PIN_MOTOR_EN, LOW); // 즉시 정차
      digitalWrite(PIN_COOLING_FAN, HIGH); // 냉각은 계속 유지(격리 전까지 온도 저감 시도)
      digitalWrite(PIN_CHARGE_RELAY, LOW); // 충전 차단 유지
      break;

    case ARMED:
      digitalWrite(PIN_MOTOR_EN, LOW); // 정차 유지
      digitalWrite(PIN_COOLING_FAN, HIGH); // 분리 순간까지 냉각 유지
      digitalWrite(PIN_CHARGE_RELAY, LOW); // 충전 차단 유지
      break;

    case EJECTED:
      latchServo.write(SERVO_RELEASE_ANGLE); // 래치 해제 -> 배터리 팩 분리
      digitalWrite(PIN_MOTOR_EN, LOW);
      digitalWrite(PIN_COOLING_FAN, LOW); // 배터리가 차체에서 분리됨 -> 냉각 대상 없음
      digitalWrite(PIN_CHARGE_RELAY, LOW); // 배터리가 분리됨 -> 충전 회로도 무의미, 차단 유지
      break;
  }

  Serial.print(F("[STATE] -> "));
  Serial.println(stateName(newState));
}

const char* stateName(SystemState s) {
  switch (s) {
    case NORMAL:  return "NORMAL";
    case WARNING: return "WARNING";
    case STOPPED: return "STOPPED";
    case ARMED:   return "ARMED";
    case EJECTED: return "EJECTED";
  }
  return "?";
}

// ---------------- 출력 (LED / 부저) ----------------
void updateIndicators(unsigned long now) {
  digitalWrite(PIN_LED_NORMAL, state == NORMAL);

  bool blinkSlow = (now / 500) % 2 == 0; // 0.5s 주기 점멸
  bool blinkFast = (now / 150) % 2 == 0; // 0.15s 주기 점멸

  digitalWrite(PIN_LED_WARN, state == WARNING && blinkSlow);
  digitalWrite(PIN_LED_DANGER, (state == STOPPED && blinkFast) || state == ARMED || state == EJECTED);

  if (state == WARNING && blinkSlow) {
    tone(PIN_BUZZER, 1000);
  } else if (state == STOPPED) {
    tone(PIN_BUZZER, 2500);
  } else if (state == ARMED) {
    tone(PIN_BUZZER, 2800); // STOPPED보다 급박한 톤 - 분리 임박, 안전 확인 대기중
  } else if (state == EJECTED) {
    if (blinkFast) tone(PIN_BUZZER, 3000); else noTone(PIN_BUZZER);
  } else {
    noTone(PIN_BUZZER);
  }
}

// ---------------- 리셋 버튼 (디바운스) ----------------
// EJECTED: 재장착 후 테스트 재시작. ARMED: 오탐 취소(분리 직전 보류 -> 정상 복귀).
unsigned long lastResetChangeAt = 0;
int lastResetReading = HIGH;

void handleResetButton() {
  if (state != EJECTED && state != ARMED) return;

  int reading = digitalRead(PIN_RESET_BTN);
  unsigned long now = millis();

  if (reading != lastResetReading) {
    lastResetChangeAt = now;
    lastResetReading = reading;
  }

  if (now - lastResetChangeAt > BUTTON_DEBOUNCE_MS && reading == LOW) {
    enterState(NORMAL);
  }
}

// ---------------- 분리 확인 버튼 (디바운스) ----------------
// ARMED 상태에서 사람이 육안으로 안전을 확인했을 때, 초음파 판정을 기다리지 않고
// 즉시 분리를 확정하는 수동 오버라이드.
unsigned long lastConfirmChangeAt = 0;
int lastConfirmReading = HIGH;

void handleEjectConfirmButton() {
  if (state != ARMED) return;

  int reading = digitalRead(PIN_EJECT_CONFIRM_BTN);
  unsigned long now = millis();

  if (reading != lastConfirmReading) {
    lastConfirmChangeAt = now;
    lastConfirmReading = reading;
  }

  if (now - lastConfirmChangeAt > BUTTON_DEBOUNCE_MS && reading == LOW) {
    enterState(EJECTED); // 수동 확정 - 초음파 판정 결과와 무관하게 즉시 분리
  }
}

// ---------------- 시리얼 텔레메트리 (시리얼 플로터용) ----------------
void printTelemetry() {
  Serial.print(F("Temp:")); Serial.print(filteredTempC, 1);
  Serial.print(F("\tRiseRate:")); Serial.print(currentTempRiseRate(), 2);
  Serial.print(F("\tCurrent:")); Serial.print(filteredCurrentA, 2);
  Serial.print(F("\tVoltage:")); Serial.print(filteredVoltageV, 2);
  Serial.print(F("\tFan:")); Serial.print(digitalRead(PIN_COOLING_FAN) ? F("ON") : F("OFF"));
  Serial.print(F("\tCharging:")); Serial.print(isCharging() ? F("YES") : F("NO"));
  Serial.print(F("\tChargeRelay:")); Serial.print(digitalRead(PIN_CHARGE_RELAY) ? F("ON") : F("OFF"));
  Serial.print(F("\tDistanceCm:")); Serial.print(lastDistanceCm);
  Serial.print(F("\tState:")); Serial.println(stateName(state));
}
