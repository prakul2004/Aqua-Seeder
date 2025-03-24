#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Servo pins
#define WATER_SERVO_PIN 5
#define DIGGING_SERVO_PIN 19
#define SEEDING_SERVO_PIN 18

// Moisture sensor and relay pins
#define MOISTURE_SENSOR_PIN 34  // Analog pin for moisture sensor
#define RELAY_PIN 33            // Relay control pin

// Signal pin from Arduino
#define SIGNAL_PIN 16

// Servo objects
Servo waterServo;
Servo diggingServo;
Servo seedingServo;

// Current process state
String currentProcess = "watering";  // Default process
String userSelectedProcess = "watering";  // User-selected process

// Wi-Fi credentials
const char* ssid = "project";
const char* password = "123456789";

// GPS module setup
TinyGPSPlus gps;
HardwareSerial mySerial(1);

// Variables to store GPS data
double latitude = 0.0;
double longitude = 0.0;

// Web server
AsyncWebServer server(80);

// Moisture threshold (user can adjust in the web interface)
int moistureThreshold = 500;  // Default threshold (adjust based on your sensor)

// Google Maps API key (replace with your own API key)
const char* googleMapsAPIKey = "AIzaSyDjsvQC6XbrkjAoQ6HJp_0ZpP4q_L4fthwY";  // Replace with your API key

// Function to check GPS and update the coordinates
void readGPS() {
  while (mySerial.available() > 0) {
    gps.encode(mySerial.read());

    if (gps.location.isUpdated()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Attach servos
  waterServo.attach(WATER_SERVO_PIN);
  diggingServo.attach(DIGGING_SERVO_PIN);
  seedingServo.attach(SEEDING_SERVO_PIN);

  // Set initial servo positions
  waterServo.write(0);
  diggingServo.write(0);
  seedingServo.write(0);

  // Configure pins
  pinMode(SIGNAL_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Turn off the relay initially

  // Start GPS serial communication on UART1 (TX: GPIO 1, RX: GPIO 3)
  mySerial.begin(9600, SERIAL_8N1, 1, 3);  // Initialize HardwareSerial1 on GPIO 1 and GPIO 3

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set up web server routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><body><h1>Robot Control Interface</h1>";
    html += "<form action='/set_process' method='get'>";
    html += "<label for='process'>Select Process:</label>";
    html += "<select name='process'>";
    html += "<option value='watering'";
    if (userSelectedProcess == "watering") {
      html += " selected";
    }
    html += ">Watering</option>";
    html += "<option value='seed'";
    if (userSelectedProcess == "seed") {
      html += " selected";
    }
    html += ">Seeding</option>";
    html += "</select>";
    html += "<button type='submit'>Set Process</button></form>";

    // Display current moisture level
    int moistureLevel = analogRead(MOISTURE_SENSOR_PIN);
    html += "<p>Current Soil Moisture Level: " + String(moistureLevel) + "</p>";

    // Add moisture threshold adjustment
    html += "<form action='/set_threshold' method='get'>";
    html += "<label for='threshold'>Moisture Threshold:</label>";
    html += "<input type='number' name='threshold' value='" + String(moistureThreshold) + "'>";
    html += "<button type='submit'>Set Threshold</button></form>";

    // Add GPS coordinates and map
    html += "<h2>Current Location</h2>";
    if (latitude != 0.0 && longitude != 0.0) {
      html += "<p>Latitude: " + String(latitude, 6) + "</p>";
      html += "<p>Longitude: " + String(longitude, 6) + "</p>";
      html += "<div id=\"map\" style=\"height: 400px; width: 100%;\"></div>";
      html += "<script src=\"https://maps.googleapis.com/maps/api/js?key=" + String(googleMapsAPIKey) + "&callback=initMap\" async defer></script>";
      html += "<script>";
      html += "function initMap() {";
      html += "  var location = {lat: " + String(latitude, 6) + ", lng: " + String(longitude, 6) + "};";
      html += "  var map = new google.maps.Map(document.getElementById('map'), {";
      html += "    zoom: 15,";
      html += "    center: location";
      html += "  });";
      html += "  var marker = new google.maps.Marker({";
      html += "    position: location,";
      html += "    map: map";
      html += "  });";
      html += "}";
      html += "</script>";
    } else {
      html += "<p>Waiting for GPS fix...</p>";
    }

    html += "<p>Current Process: " + currentProcess + "</p>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/set_process", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("process")) {
      userSelectedProcess = request->getParam("process")->value();
      Serial.println("User selected process: " + userSelectedProcess);
    }
    request->redirect("/");
  });

  server.on("/set_threshold", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("threshold")) {
      moistureThreshold = request->getParam("threshold")->value().toInt();
      Serial.println("Updated moisture threshold: " + String(moistureThreshold));
    }
    request->redirect("/");
  });

  server.begin();
}

void loop() {
  // Continuously read GPS data
  readGPS();

  if (digitalRead(SIGNAL_PIN) == HIGH) {  // Signal from Arduino
    Serial.println("Signal received from Arduino");
    currentProcess = userSelectedProcess;  // Update process based on user selection

    if (currentProcess == "watering") {
      performWatering();
    } else if (currentProcess == "seed") {
      performSeeding();
    }

    // Wait until signal is LOW before continuing
    while (digitalRead(SIGNAL_PIN) == HIGH) {
      delay(10);
    }
  }
}

void performWatering() {
  Serial.println("Performing watering process");

  int moistureLevel = analogRead(MOISTURE_SENSOR_PIN);
  Serial.println("Soil moisture level: " + String(moistureLevel));

  if (moistureLevel < moistureThreshold) {
    Serial.println("Soil is dry, starting watering");
    waterServo.write(90);  // Move water servo down
    delay(5000);           // Wait for 5 seconds
    waterServo.write(0);   // Move water servo back up
    delay(1000);           // Stabilization delay

    digitalWrite(RELAY_PIN, HIGH);  // Turn on relay
    delay(5000);                   // Keep relay on for 5 seconds
    digitalWrite(RELAY_PIN, LOW);  // Turn off relay
  } else {
    Serial.println("Soil is moist, skipping watering");
  }
}

void performSeeding() {
  Serial.println("Performing seeding process");
  diggingServo.write(90);  // Move digging servo down
  delay(10000);            // Wait for 10 seconds
  diggingServo.write(0);   // Move digging servo back up
  delay(1000);             // Stabilization delay

  seedingServo.write(90);  // Move seeding servo down
  delay(1000);             // Wait for 1 second
  seedingServo.write(0);   // Move seeding servo back up
  delay(1000);             // Stabilization delay
}