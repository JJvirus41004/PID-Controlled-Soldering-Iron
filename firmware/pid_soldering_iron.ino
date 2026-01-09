
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define THERMISTOR_PIN 34      // ADC pin for temperature input
#define RELAY_PIN 4            // Output pin to control relay
#define BUTTON_PIN 2          // Button to trigger AutoTune

unsigned long autoTuneLastSwitch = 0;
const unsigned long autoTuneRelayCycle = 3000;  // 3 seconds relay ON/OFF interval

unsigned long lastPrint = 0;

// === PID variables ===
double Setpoint = 100.0;
double Input = 0.0;
double Output = 0.0;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// === PID and AutoTune objects ===
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID_ATune aTune(&Input, &Output);

const unsigned long windowSize = 2000;  // Relay time-proportional window
unsigned long windowStartTime = 0;

bool autoTuneRunning = false;
bool lastButtonState = HIGH;

double readTemperature() {
  int adc = analogRead(THERMISTOR_PIN);
  if (adc <= 0 || adc >= 4095) return -273.15;  // prevent divide-by-zero or infinite

  // For pull-down configuration (thermistor on top)
  double resistance = 10000.0 * adc / (4095.0 - adc);  // Corrected formula

  // Steinhart-Hart calculation
  double steinhart = resistance / 10000.0;  // R/R0
  steinhart = log(steinhart);              // ln(R/R0)
  steinhart /= 3950.0;                     // 1/B * ln(R/R0)
  steinhart += 1.0 / (25.0 + 273.15);      // + 1/T0
  steinhart = 1.0 / steinhart;             // Invert
  steinhart -= 273.15;                     // Kelvin to Celsius

  return steinhart;
}

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  windowStartTime = millis();

  myPID.SetOutputLimits(0, windowSize);
  myPID.SetMode(AUTOMATIC);

  Serial.println("System Ready. Press button to start AutoTune.");
}

void loop() {
  Input = readTemperature();  // Replace with real sensor logic

  // === Button press detection === N
  // === Button press detection with toggle ===
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    if (!autoTuneRunning) {

      if ((now - windowStartTime) < Output) {
          digitalWrite(RELAY_PIN, HIGH);
        } else {
          digitalWrite(RELAY_PIN, LOW);
        }
      } else {
        digitalWrite(RELAY_PIN, LOW);  // Keep relay OFF during AutoTune
      }

      aTune.SetOutputStep(50);       // Current step size is okay
      aTune.SetControlType(1);       // PID mode
      aTune.SetLookbackSec(20);      // Already set
      aTune.SetNoiseBand(1);         // Already set
      aTune.SetOutputStep(50);       // Reduce step if overshoot too fast




      // START AUTOTUNE
      autoTuneRunning = true;
      myPID.SetMode(MANUAL);

      aTune.SetNoiseBand(1.0);
      aTune.SetOutputStep(500);
      aTune.SetLookbackSec(10);
      aTune.SetControlType(1); // PI

      Serial.println("🔁 AutoTune STARTED");
    } else {
      // STOP AUTOTUNE
      autoTuneRunning = false;
      myPID.SetMode(AUTOMATIC);

      Serial.println("⛔ AutoTune STOPPED manually.");
    }
  }
  lastButtonState = currentButtonState;

  

  unsigned long now = millis();
  if ((now - windowStartTime) > windowSize) {
    windowStartTime += windowSize;
  }

  // === AutoTune Mode ===
  if (autoTuneRunning) {

    if (aTune.Runtime()) {
      // AutoTune complete — apply tuned values
    }

    // Only control relay ON/OFF every few seconds:
    if ((now - autoTuneLastSwitch) > autoTuneRelayCycle) {
      autoTuneLastSwitch = now;
      
      if (Output > 0) {
        digitalWrite(RELAY_PIN, HIGH);
      } else {
        digitalWrite(RELAY_PIN, LOW);
      }
    }

    if (aTune.Runtime()) {
      // AutoTune finished!
      autoTuneRunning = false;

      // Fetch tuned values
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();

      myPID.SetTunings(Kp, Ki, Kd);
      myPID.SetMode(AUTOMATIC);  // resume normal PID

      // Inform the user
      Serial.println("AutoTune finished!");
      Serial.print("Kp: "); Serial.println(Kp);
      Serial.print("Ki: "); Serial.println(Ki);
      Serial.print("Kd: "); Serial.println(Kd);

      Serial.println("==== AutoTune COMPLETE ====");
      Serial.println("👉 Copy these values into your code:");
      Serial.print("double Kp = "); Serial.print(Kp); Serial.println(";");
      Serial.print("double Ki = "); Serial.print(Ki); Serial.println(";");
      Serial.print("double Kd = "); Serial.print(Kd); Serial.println(";");
      Serial.println("================================");

      Serial.println("AutoTune complete ✅ — Now resuming normal PID control.");
      Serial.println("// 🔄 You may edit the Kp, Ki, Kd values below if tuning wasn’t ideal.");
    } else {
      if (millis() - lastPrint > 1000) {
        Serial.println("AutoTune running... please wait");
        lastPrint = millis();
      }  
    }
  } else {
    myPID.Compute();
  }

  // === Time-proportional relay control ===
  if ((now - windowStartTime) < Output) {
    digitalWrite(RELAY_PIN, HIGH);
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }

  // === Debug output ===
  if (!autoTuneRunning) {
    Serial.print("Temp: ");
    Serial.print(Input);
    Serial.print("°C | Output: ");
    Serial.print(Output);
    Serial.print(" | Relay: ");
    Serial.println(digitalRead(RELAY_PIN) ? "ON" : "OFF");
  }
  delay(200);
}

// === TEMP SENSOR MOCK (replace with real thermistor calc) ===




