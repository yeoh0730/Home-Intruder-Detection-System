#include "VOneMqttClient.h"
#include <ESP32Servo.h>

// Device ID for PIR sensor in V-One platform
const char* PIRsensor = "ea194f19-ca73-4ea4-9299-54ca9c4eaa07";  

// Pin Definitions
const int servoPin = 5;     // Servo motor pin
const int pirPin = 4;       // PIR sensor pin
const int ledPin = 9;       // LED pin

// Variables
Servo doorServo;                  // Servo motor object
int pirState = LOW;                // Initial state of PIR sensor
bool ledState = false;             // LED state flag (false = OFF, true = ON)
bool intruderDetected = false;     // Flag to indicate intruder detection

// Timer Variables
unsigned long previousMillisPIR = 0;     // Timer for PIR updates
const long intervalPIR = 1000;           // Interval for PIR sensor checks (1 second)

// V-One Client Instance
VOneMqttClient voneClient;

// Wi-Fi Setup
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Wi-Fi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Setup
void setup() {
  setup_wifi();                    // Connect to Wi-Fi
  voneClient.setup();              // Initialize V-One Client

  // Configure pins
  pinMode(pirPin, INPUT);              // Set PIR sensor as input
  pinMode(ledPin, OUTPUT);             // Set LED as output
  doorServo.attach(servoPin);          // Attach servo motor
  doorServo.write(0);                  // Initialize door in locked position (0 degrees)

  Serial.begin(9600);                  // Start serial communication
  Serial.println("Intruder detection system initialized");
}

// Loop
void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  // Ensure V-One connection is active
  if (!voneClient.connected()) {
    voneClient.reconnect();
    voneClient.publishDeviceStatusEvent(PIRsensor, true);
  }
  voneClient.loop(); // Maintain MQTT connection

  // PIR Sensor Updates
  if (currentMillis - previousMillisPIR >= intervalPIR) { // Check if interval has passed
    previousMillisPIR = currentMillis; // Update timer

    int motionDetected = digitalRead(pirPin); // Read PIR sensor state
    if (motionDetected == HIGH && pirState == LOW) { // Detect motion transition
      Serial.println("Motion detected! Waiting for verification..."); // Send command to Python script
      pirState = HIGH;

      // Send motion detected status to V-One platform
      voneClient.publishTelemetryData(PIRsensor, "Motion", 1);
      Serial.println(pirState);
    } else if (motionDetected == LOW && pirState == HIGH) { // No motion detected
      Serial.println("No motion detected. Locking door."); // Send command to Python script
      pirState = LOW;
      intruderDetected = false;

      // Send no motion status to V-One platform
      voneClient.publishTelemetryData(PIRsensor, "Motion", 0);
    }
  }

  // Handle serial input from Python script for commands (whether intruder is detected or not)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read command from serial input
    command.trim();

    if (command == "UNLOCK") { // Command to unlock door (authorized person detected)
      Serial.println("Authorized person detected. Unlocking door.");
      doorServo.write(90);      // Move servo to unlock position (90 degrees)
      digitalWrite(ledPin, LOW); // Turn off LED
      intruderDetected = false;  // Reset intruder flag
    } 

    else if (command == "LOCK") { // Command to lock door (intruder detected)
      Serial.println("Intruder detected. Locking door.");
      doorServo.write(0);         // Move servo to lock position (0 degrees)
      intruderDetected = true;   // Set intruder flag
    }
  }

  // LED blinking for intruder alert
  if (intruderDetected) {
    if (currentMillis - previousMillisPIR >= 500) { // Blink every 500ms
      previousMillisPIR = currentMillis;
      ledState = !ledState;                         // Toggle LED state
      digitalWrite(ledPin, ledState);               // Update LED state
    }
  } else {
    digitalWrite(ledPin, LOW); // Ensure LED is off when no intruder
  }

  delay(100);
}