### Smart Plant Watering System Using Ardunio

### Components
Arduino Uno

Ultrasonic sensor (for gate automation, distance detection)

Soil moisture sensor (for irrigation)

Water flow sensor (to measure water usage)

DHT11 (temperature & humidity logging)

Servo motor (for gate)

Mini water pump (for irrigation)

Relay module (to control pump safely)

Breadboard + jumper wires

### Wire Connections
1. Soil Moisture Sensor

VCC → 5V

GND → GND

AO (Analog Out) → A0

2. Ultrasonic Sensor (HC-SR04)

VCC → 5V

GND → GND

Trig → D8

Echo → D9

3. DHT11 Sensor

VCC → 5V

GND → GND

Data → D2 (with 10k pull-up resistor recommended)

4. Servo Motor

VCC (Red) → 5V

GND (Brown/Black) → GND

Signal (Orange/Yellow) → D6

5. Water Flow Sensor (YF-S201 or similar)

Red (VCC) → 5V

Black (GND) → GND

Yellow (Signal) → D3

6. Relay Module + Water Pump

Relay VCC → 5V

Relay GND → GND

Relay IN → D7

Pump connected via relay to external power (e.g. 5V/9V supply depending on pump rating).

### Projct Setup
![ardunio](/assets/ardunio.jpeg)
### Code
```
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
  gateServo.write(0);  // Gate closed

  dht.begin();

  pinMode(FLOW_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), flowISR, RISING);
}

void loop() {
  // 🌱 Soil Moisture Reading
  int moisture = analogRead(MOISTURE_PIN);
  Serial.print("Soil Moisture: ");
  Serial.println(moisture);

  // 🌡️ Temperature & Humidity
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  Serial.print("Temp: "); Serial.print(t);
  Serial.print(" °C  Humidity: "); Serial.println(h);

  // 💧 Water Pump Control
  if (moisture < 500) {   // Adjust threshold
    digitalWrite(RELAY, LOW);  // Pump ON
    Serial.println("Pump ON");
  } else {
    digitalWrite(RELAY, HIGH); // Pump OFF
    Serial.println("Pump OFF");
  }

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

  // ⏱️ Flow Sensor Measurement
  if ((millis() - oldTime) > 1000) {
    detachInterrupt(digitalPinToInterrupt(FLOW_SENSOR));
    float flowRate = (flowCount / 7.5); // L/min (depends on sensor)
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
