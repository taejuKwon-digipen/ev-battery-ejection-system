/*
  EV 배터리 열폭주 조기 감지 및 능동형 안전 분리(Ejection) 시스템
  ------------------------------------------------------------
  핀 배선
    A0  - NTC 서미스터 (분압, SERIES_RESISTOR와 직렬)
    A1  - ACS712 전류센서 OUT
    A2  - 배터리 전압 분압 회로 OUT
    D2  - 리셋 버튼 (INPUT_PULLUP, 눌리면 LOW) - EJECTED 상태 복귀용
    D4  - 정상 LED (녹색)
    D5  - 경고 LED (황색)
    D6  - 위험 LED (적색)
    D7  - 부저
    D8  - 모터 드라이버 EN (LOW = 주행 모터 정지)
    D9  - 래치 서보 (SG90 등)

  상태 전이(FSM)
    NORMAL  -> WARNING : 온도/전류/전압이 경고 임계값 초과
    WARNING -> NORMAL  : 모든 값이 정상 범위로 복귀
    WARNING -> STOPPED : 위험 임계값(과열/온도급상승/과전류/전압이상) 도달
    NORMAL  -> STOPPED : 위험 임계값에 바로 도달한 경우
    STOPPED -> EJECTED : 대피 유예시간(EJECT_DELAY_MS) 경과 후 자동 분리
    EJECTED -> NORMAL  : 리셋 버튼(재장착 후 테스트용)
*/

#include <Servo.h>

// ---------------- 핀 정의 ----------------
const uint8_t PIN_NTC        = A0;
const uint8_t PIN_CURRENT    = A1;
const uint8_t PIN_VOLTAGE    = A2;
const uint8_t PIN_RESET_BTN  = 2;
const uint8_t PIN_LED_NORMAL = 4;
const uint8_t PIN_LED_WARN   = 5;
const uint8_t PIN_LED_DANGER = 6;
const uint8_t PIN_BUZZER     = 7;
const uint8_t PIN_MOTOR_EN   = 8;
const uint8_t PIN_SERVO      = 9;

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
enum SystemState { NORMAL, WARNING, STOPPED, EJECTED };
SystemState state = NORMAL;
unsigned long stateEnteredAt = 0;

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
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);

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

// ---------------- FSM 로직 ----------------
bool isCriticalCondition() {
  return filteredTempC >= TEMP_CRITICAL_C
      || currentTempRiseRate() >= TEMP_RISE_RATE_C_S
      || filteredCurrentA >= CURRENT_CRITICAL_A
      || filteredVoltageV <= VOLTAGE_MIN_V
      || filteredVoltageV >= VOLTAGE_MAX_V;
}

bool isWarningCondition() {
  return filteredTempC >= TEMP_WARNING_C
      || filteredCurrentA >= CURRENT_WARNING_A;
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
        enterState(EJECTED);
      }
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
      latchServo.write(SERVO_LOCK_ANGLE);
      noTone(PIN_BUZZER);
      break;

    case WARNING:
      digitalWrite(PIN_MOTOR_EN, HIGH);
      break;

    case STOPPED:
      digitalWrite(PIN_MOTOR_EN, LOW); // 즉시 정차
      break;

    case EJECTED:
      latchServo.write(SERVO_RELEASE_ANGLE); // 래치 해제 -> 배터리 팩 분리
      digitalWrite(PIN_MOTOR_EN, LOW);
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
  digitalWrite(PIN_LED_DANGER, (state == STOPPED && blinkFast) || state == EJECTED);

  if (state == WARNING && blinkSlow) {
    tone(PIN_BUZZER, 1000);
  } else if (state == STOPPED) {
    tone(PIN_BUZZER, 2500);
  } else if (state == EJECTED) {
    if (blinkFast) tone(PIN_BUZZER, 3000); else noTone(PIN_BUZZER);
  } else {
    noTone(PIN_BUZZER);
  }
}

// ---------------- 리셋 버튼 (디바운스) ----------------
unsigned long lastButtonChangeAt = 0;
int lastButtonReading = HIGH;

void handleResetButton() {
  if (state != EJECTED) return;

  int reading = digitalRead(PIN_RESET_BTN);
  unsigned long now = millis();

  if (reading != lastButtonReading) {
    lastButtonChangeAt = now;
    lastButtonReading = reading;
  }

  if (now - lastButtonChangeAt > BUTTON_DEBOUNCE_MS && reading == LOW) {
    enterState(NORMAL);
  }
}

// ---------------- 시리얼 텔레메트리 (시리얼 플로터용) ----------------
void printTelemetry() {
  Serial.print(F("Temp:")); Serial.print(filteredTempC, 1);
  Serial.print(F("\tRiseRate:")); Serial.print(currentTempRiseRate(), 2);
  Serial.print(F("\tCurrent:")); Serial.print(filteredCurrentA, 2);
  Serial.print(F("\tVoltage:")); Serial.print(filteredVoltageV, 2);
  Serial.print(F("\tState:")); Serial.println(stateName(state));
}
