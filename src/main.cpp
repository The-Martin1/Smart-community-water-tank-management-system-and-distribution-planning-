#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// ── Credentials ───────────────────────────────────────────────
// Real values live in include/secrets.h, which is gitignored.
// Copy include/secrets.h.example to include/secrets.h to get started.
#include "secrets.h"

// ── Hardware Pins ─────────────────────────────────────────────
#define FLOW_SENSOR_PIN 21 // Yellow Signal Wire of the YF-S401
#define TRIG_A 5
#define ECHO_A 17
 
// ── Tank A calibration (measured) ──────────────────────────────
const float A_FULL = 1.66; // distance when tank is full
const float A_EMPTY = 16.33; // distance when tank is empty

// ── Flow Tracking Variables ───────────────────────────────────
volatile int pulseCount = 0;
float flowRateLPM = 0.0;
unsigned long oldTime = 0;

// ── Firebase Objects ──────────────────────────────────────────
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

// ── Interrupt Service Routine (Runs instantly on every pulse) ──
void IRAM_ATTR pulseCounter() { 
pulseCount++;
}

// ── Ultrasonic Measure Functions ────────────────────────────────
float measureDistanceCM(int trigPin, int echoPin) {
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
long duration = pulseIn(echoPin, HIGH, 30000);
if (duration == 0) return -1;
return (duration * 0.0343) / 2.0;
}

float measureDistanceSmoothed(int trigPin, int echoPin, int samples = 9) {
float readings[9];
int validCount = 0;
for (int i = 0; i < samples; i++) {
float d = measureDistanceCM(trigPin, echoPin);
if (d > 0) {
readings[validCount++] = d;
}
delay(20);
}
if (validCount == 0) return -1;

// Sort so we can find the median and reject splash/echo outliers
for (int i = 0; i < validCount - 1; i++) {
for (int j = i + 1; j < validCount; j++) {
if (readings[j] < readings[i]) {
float tmp = readings[i];
readings[i] = readings[j];
readings[j] = tmp;
}
}
}
float median = readings[validCount / 2];

// Trimmed mean: drop samples far from the median (splash spikes / stray echoes)
const float MAX_DEVIATION_CM = 0.6;
float total = 0;
int keptCount = 0;
for (int i = 0; i < validCount; i++) {
if (fabs(readings[i] - median) <= MAX_DEVIATION_CM) {
total += readings[i];
keptCount++;
}
}
if (keptCount == 0) return median;
return total / keptCount;
}

// ── Persistent smoothing across loop iterations ─────────────────
// Distance range for Tank A is only ~14.7cm, so raw sensor jitter of
// a few mm swings the percentage by several points. An EMA filter
// remembers the previous reading and only moves partway toward each
// new one, so it climbs steadily while filling instead of jittering.
float distanceEMA_A = -1.0;
const float EMA_ALPHA = 0.2;

float smoothDistance(float rawDistance, float &emaState) {
if (rawDistance < 0) return emaState; // keep last good value on echo failure
if (emaState < 0) {
emaState = rawDistance; // first valid reading seeds the filter
} else {
emaState = EMA_ALPHA * rawDistance + (1.0 - EMA_ALPHA) * emaState;
}
return emaState;
}

int getPercent(float distance, float fullDist, float emptyDist) {
float percent = ((emptyDist - distance) / (emptyDist - fullDist)) * 100.0;
return (int)constrain(percent, 0.0, 100.0);
}

void printBar(int percent) {
int filled = percent / 5;
Serial.print("[");
for (int i = 0; i < 20; i++) {
Serial.print(i < filled ? "█" : "░");
}
Serial.print("]");
}

// ── Setup ─────────────────────────────────────────────────────
void setup() { 
Serial.begin(115200); 
delay(1000); 
pinMode(TRIG_A, OUTPUT);
pinMode(ECHO_A, INPUT);
// Configure Flow Sensor Pin with internal pullup resistor
pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP); 
// Attach background hardware interrupt to capture pulses
attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING); 
Serial.println("\n=== Smart Water Tank Monitor (Tank A) ===\n");

// Initialize Wi-Fi
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
Serial.print("Connecting to Wi-Fi");
while(WiFi.status() != WL_CONNECTED){
Serial.print("."); 
delay(300);
}
Serial.println();
Serial.print("Connected with IP: ");
Serial.println(WiFi.localIP());
Serial.println();

// Firebase Init Block
config.api_key = API_KEY;
config.database_url = DATABASE_URL;
if(Firebase.signUp(&config, &auth, "", "")){
Serial.println("signUp OK");
signupOK = true;
} else {
Serial.printf("%s\n", config.signer.signupError.message.c_str());
}
config.token_status_callback = tokenStatusCallback;
Firebase.begin(&config, &auth);
Firebase.reconnectWiFi(true);
oldTime = millis();
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
// Print and calculate readings every 1 second (1000ms) for high-speed tracking
if ((millis() - oldTime) > 1000) { 
// Pause interrupts briefly while copying data to prevent calculation corruption
detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN)); 
float timeElapsedSec = (millis() - oldTime) / 1000.0; 
// YF-S401 Conversion Formula: (Pulses / 98.0) / Time = Liters Per Minute
if (timeElapsedSec > 0) { 
flowRateLPM = ((float)pulseCount / 98.0) / timeElapsedSec; 
// ── NOISE CANCELLATION FILTER ── 
if (pulseCount <= 15 || flowRateLPM < 0.1) {
flowRateLPM = 0.00; 
} 
} else { 
flowRateLPM = 0.00; 
} 
// Print results to the Serial Monitor
Serial.printf("Pulses Caught: %3d | Converted Flow Rate: %5.2f L/min\n", pulseCount, flowRateLPM); 
// Reset count and timers for the next second window
int currentPulses = pulseCount;
pulseCount = 0; 
oldTime = millis(); 
// Re-enable hardware interrupts immediately
attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), pulseCounter, FALLING); 

// ── Real-Time Ultrasonic Pipeline ──
float rawDistance = measureDistanceSmoothed(TRIG_A, ECHO_A);
float distance = smoothDistance(rawDistance, distanceEMA_A);
if (distance < 0) {
Serial.println("Tank A — ERROR (no echo)\n");
} else {
int percent = getPercent(distance, A_FULL, A_EMPTY);

// Print to Serial Monitor
Serial.println("Tank A");
if (percent <= 10) Serial.println(" ⚠ Tank A is empty!");
else if (percent <= 25) Serial.println(" ⚠ Tank A — refill soon");
else if (percent >= 95) Serial.println(" ✓ Tank A is FULL");

Serial.print("Water level ");
printBar(percent);
Serial.printf(" %3d%%\n", percent);

// Real-Time Upload Matrix to Firebase
if (Firebase.ready() && signupOK) {
// Upload Tank Volume
if(Firebase.RTDB.setInt(&fbdo, "Sensor/tank_percentage", percent)){
Serial.println(); 
Serial.print(percent);
Serial.print(" - successfully saved to: " + fbdo.dataPath());
Serial.println(" (" + fbdo.dataType() + ")");
} else {
Serial.println("FAILED TANK UPDATE: " + fbdo.errorReason());
}

// Upload Flow Rate Live
if(Firebase.RTDB.setFloat(&fbdo, "Sensor/flow_rate", flowRateLPM)){
Serial.printf("%.2f L/min - successfully saved to: %s\n", flowRateLPM, fbdo.dataPath().c_str());
} else {
Serial.println("FAILED FLOW UPDATE: " + fbdo.errorReason());
}
}
Serial.println("--------------------------------");
}
}
}

