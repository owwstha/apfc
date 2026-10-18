#include <Arduino.h>

const int PIN_PWM   = 25;
const int PIN_VAC   = 34;
const int PIN_VDC   = 35;
const int PIN_IL    = 32;

const uint32_t PWM_FREQ = 50000;
const uint8_t  PWM_RES  = 10;
const uint32_t PWM_MAX  = (1u << PWM_RES) - 1;
const uint8_t  PWM_CH   = 0;

const float VAC_SCALE    = 5.1f  / 1000.0f;
const float VDC_SCALE    = 10.0f / 1000.0f;
const float IL_OFFSET_MV = 0.0f;
const float IL_SCALE     = 1.0f  / 1000.0f;

const float VREF_FINAL     = 20.0f;
const float SOFTSTART_RATE = 10.0f;

const uint32_t CURRENT_LOOP_HZ  = 10000;
const float    DT_I             = 1.0f / CURRENT_LOOP_HZ;
const uint32_t VOLTAGE_LOOP_DIV = 100;
const float    DT_V             = DT_I * VOLTAGE_LOOP_DIV;

float KP_V = 0.10f;
float KI_V = 2.0f;
const float PDEMAND_MIN = 0.0f;
const float PDEMAND_MAX = 3.0f;

float KP_I = 0.05f;
float KI_I = 300.0f;
const float DUTY_MIN = 0.0f;
const float DUTY_MAX = 0.90f;

const float VDC_OVP      = 24.0f;
const float IL_OCP       = 4.0f;
const float VAC_PEAK_MIN = 3.0f;

const float VDC_LPF_ALPHA    = 0.02f;
const float PEAK_DECAY_PER_S = 2.0f;

struct PIController {
  float kp, ki, outMin, outMax;
  float integ = 0.0f;

  PIController(float kp_, float ki_, float mn, float mx)
    : kp(kp_), ki(ki_), outMin(mn), outMax(mx) {}

  float update(float error, float dt) {
    float p = kp * error;
    float iCandidate = integ + ki * error * dt;
    float u = p + iCandidate;

    if (u > outMax) {
      u = outMax;
      if (error < 0) integ = iCandidate;
    } else if (u < outMin) {
      u = outMin;
      if (error > 0) integ = iCandidate;
    } else {
      integ = iCandidate;
    }
    if (integ > outMax) integ = outMax;
    if (integ < outMin) integ = outMin;
    return u;
  }

  void reset() { integ = 0.0f; }
};

PIController piV(KP_V, KI_V, PDEMAND_MIN, PDEMAND_MAX);
PIController piI(KP_I, KI_I, DUTY_MIN, DUTY_MAX);

volatile float g_vac = 0, g_vdc = 0, g_il = 0;
volatile float g_vdcFilt = 0, g_vacPeak = 0;
volatile float g_vref = 0, g_pdemand = 0, g_iref = 0, g_duty = 0;
volatile bool  g_fault = false;

hw_timer_t   *ctrlTimer   = nullptr;
TaskHandle_t  ctrlTaskHdl = nullptr;

void pwmInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWM, 0);
#else
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM, PWM_CH);
  ledcWrite(PWM_CH, 0);
#endif
}

inline void pwmWriteDuty(float d) {
  if (d < 0) d = 0;
  if (d > DUTY_MAX) d = DUTY_MAX;
  uint32_t cnt = (uint32_t)(d * PWM_MAX + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_PWM, cnt);
#else
  ledcWrite(PWM_CH, cnt);
#endif
}

inline float readVac() { return analogReadMilliVolts(PIN_VAC) * VAC_SCALE; }
inline float readVdc() { return analogReadMilliVolts(PIN_VDC) * VDC_SCALE; }
inline float readIL()  { return (analogReadMilliVolts(PIN_IL) - IL_OFFSET_MV) * IL_SCALE; }

void IRAM_ATTR onControlTimer() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(ctrlTaskHdl, &woken);
  if (woken) portYIELD_FROM_ISR();
}

void shutdown() {
  pwmWriteDuty(0);
  piV.reset();
  piI.reset();
  g_fault = true;
}

void controlTask(void *arg) {
  uint32_t tick = 0;
  float vdcFilt = 0, vacPeak = 0, vref = 0, pdemand = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    float vac = readVac();
    float vdc = readVdc();
    float il  = readIL();

    if (vdc > VDC_OVP || il > IL_OCP) {
      shutdown();
      g_vdc = vdc; g_il = il;
      continue;
    }
    if (g_fault) { pwmWriteDuty(0); continue; }

    vdcFilt += VDC_LPF_ALPHA * (vdc - vdcFilt);

    if (vac > vacPeak) vacPeak = vac;
    else               vacPeak -= vacPeak * PEAK_DECAY_PER_S * DT_I;

    if (++tick >= VOLTAGE_LOOP_DIV) {
      tick = 0;

      if (vref < vacPeak) vref = vacPeak;
      if (vref < VREF_FINAL) {
        vref += SOFTSTART_RATE * DT_V;
        if (vref > VREF_FINAL) vref = VREF_FINAL;
      }

      float vErr = vref - vdcFilt;
      pdemand = piV.update(vErr, DT_V);
    }

    float duty = 0.0f;
    float iref = 0.0f;
    if (vacPeak > VAC_PEAK_MIN) {
      float vNorm = vac / vacPeak;
      if (vNorm > 1.0f) vNorm = 1.0f;
      if (vNorm < 0.0f) vNorm = 0.0f;
      iref = pdemand * vNorm;

      float iErr = iref - il;
      duty = piI.update(iErr, DT_I);
    } else {
      piI.reset();
    }

    pwmWriteDuty(duty);

    g_vac = vac; g_vdc = vdc; g_il = il;
    g_vdcFilt = vdcFilt; g_vacPeak = vacPeak;
    g_vref = vref; g_pdemand = pdemand; g_iref = iref; g_duty = duty;
  }
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pwmInit();
  pwmWriteDuty(0);

  xTaskCreatePinnedToCore(controlTask, "pfc_ctrl", 4096, nullptr,
                          configMAX_PRIORITIES - 1, &ctrlTaskHdl, 1);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ctrlTimer = timerBegin(1000000);
  timerAttachInterrupt(ctrlTimer, &onControlTimer);
  timerAlarm(ctrlTimer, 1000000 / CURRENT_LOOP_HZ, true, 0);
#else
  ctrlTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(ctrlTimer, &onControlTimer, true);
  timerAlarmWrite(ctrlTimer, 1000000 / CURRENT_LOOP_HZ, true);
  timerAlarmEnable(ctrlTimer);
#endif

  Serial.println("PFC controller started. Send 'r' to clear a fault.");
}

void loop() {
  static uint32_t last = 0;

  if (Serial.available() && Serial.read() == 'r') {
    g_fault = false;
    Serial.println("Fault cleared.");
  }

  if (millis() - last >= 500) {
    last = millis();
    Serial.printf("Vref=%.2f Vdc=%.2f (filt %.2f) VacPk=%.2f Pdem=%.3f "
                  "Iref=%.3f IL=%.3f D=%.3f %s\n",
                  g_vref, g_vdc, g_vdcFilt, g_vacPeak, g_pdemand,
                  g_iref, g_il, g_duty, g_fault ? "FAULT" : "");
  }
}
