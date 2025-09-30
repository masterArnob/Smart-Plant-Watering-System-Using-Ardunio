<h1 align="center">🌱 Smart Plant Watering System Using Arduino</h1>


<p align="center">
  <img src="https://img.shields.io/badge/Arduino-Uno-blue?logo=arduino&logoColor=white" alt="Arduino">
  <img src="https://img.shields.io/badge/Language-C++-green?logo=c%2B%2B&logoColor=white" alt="C++">
  <img src="https://img.shields.io/badge/Platform-IoT-lightgrey?logo=internetofthings&logoColor=white" alt="IoT">
  <img src="https://img.shields.io/badge/License-MIT-yellow" alt="License">
</p>

<p align="center">
  A modern IoT-based smart irrigation & monitoring system powered by Arduino 🌿💧
</p>

---

## 📑 Table of Contents
[🎯 Objectives](#-objectives) • 
[🛠️ Components](#-components) • 
[🔌 Wire Connections](#-wire-connections) • 
[⚙️ Project Setup](#-project-setup) • 
[💻 Code](#-code)




---

## 🎯 Objectives
This system will:
- 🌱 **Auto-irrigation** based on soil moisture  
- 💧 **Water flow monitoring**  
- 🌡️ **Environment logging** (temperature & humidity)  
- 🚪 **Smart gate automation** using ultrasonic detection  

---

## 🛠️ Components
| Component | Purpose |
|-----------|---------|
| Arduino Uno | Main microcontroller |
| Ultrasonic Sensor | Gate automation, distance detection |
| Soil Moisture Sensor | Irrigation control |
| Water Flow Sensor | Measure water usage |
| DHT11 | Temperature & humidity logging |
| Servo Motor | Gate control |
| Mini Water Pump | Irrigation |
| Relay Module | Pump switching |
| Breadboard + Jumper Wires | Circuit connections |

---

## 🔌 Wire Connections

### Soil Moisture Sensor
- VCC → 5V  
- GND → GND  
- AO → A0  

### Ultrasonic Sensor (HC-SR04)
- VCC → 5V  
- GND → GND  
- Trig → D8  
- Echo → D9  

### DHT11 Sensor
- VCC → 5V  
- GND → GND  
- Data → D2 *(10k pull-up recommended)*  

### Servo Motor
- VCC (Red) → 5V  
- GND (Brown/Black) → GND  
- Signal (Orange/Yellow) → D6  

### Water Flow Sensor (YF-S201 or similar)
- Red (VCC) → 5V  
- Black (GND) → GND  
- Yellow (Signal) → D3  

### Relay Module + Pump
- Relay VCC → 5V  
- Relay GND → GND  
- Relay IN → D7  
- Pump → Connected via relay to external power (5V/9V depending on pump)  

---

## ⚙️ Project Setup
<p align="center">
  <img src="/assets/components.jpg" alt="Arduino Setup" width="500">

</p>

<div class="display-flex justify-content-center align-items-center">
     <img src="/assets/project_showcase.gif" alt="Arduino Setup" width="500">
   <img src="/assets/Serial_monitor.gif" alt="Arduino Setup" width="500">
</div>


   

---

## 💻 Code
```cpp
#include <Servo.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11
#define MOISTURE_PIN A0
#define TRIG 8
#define ECHO 9
#define RELAY 7
#define SERVO_PIN 6
#define FLOW_SENSOR 3

Servo gateServo;
DHT dht(DHTPIN, DHTTYPE);

volatile int flowCount = 0;
unsigned long oldTime = 0;

// Flow sensor interrupt
void flowISR() {
  flowCount++;
}

void setup() {
  Serial.begin(9600);

  pinMode(MOISTURE_PIN, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(RELAY, OUTPUT);

  gateServo.attach(SERVO_PIN);
  gateServo.write(0);  // Gate closed initially

  dht.begin();

  pinMode(FLOW_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), flowISR, RISING);

  digitalWrite(RELAY, HIGH); // Make sure pump is OFF at start
}

void loop() {
  // 🌱 Soil Moisture Reading
  int moisture = analogRead(MOISTURE_PIN);
  Serial.print("Soil Moisture: ");
  Serial.print(moisture);

  // 💧 Pump Control (adjust threshold as needed)
  // Dry soil = high value (~800+), Wet = low (~300)
  if (moisture > 500) {   // Soil is dry → Pump ON
    digitalWrite(RELAY, LOW);   // Active LOW relay → Pump ON
    Serial.println(" -> Dry, Pump ON");
  } else {
    digitalWrite(RELAY, HIGH);  // Soil is wet → Pump OFF
    Serial.println(" -> Wet, Pump OFF");
  }

  // 🌡 Temperature & Humidity
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  Serial.print("Temp: "); Serial.print(t);
  Serial.print(" °C  Humidity: "); Serial.println(h);

  // 🚪 Ultrasonic Gate Control
  long duration, distance;
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = duration * 0.034 / 2;

  if (distance < 20) {
    gateServo.write(90);   // Open gate
    Serial.println("Gate Open");
  } else {
    gateServo.write(0);    // Close gate
    Serial.println("Gate Closed");
  }

  // ⏱ Flow Sensor Measurement
  if ((millis() - oldTime) > 1000) {
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR));
    float flowRate = (flowCount / 7.5); // L/min (depends on sensor calibration)
    Serial.print("Flow Rate: ");
    Serial.print(flowRate);
    Serial.println(" L/min");
    flowCount = 0;
    oldTime = millis();
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), flowISR, RISING);
  }

  delay(500);
}
```
