#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WebServer.h>
#include <DNSServer.h>

#define SCL_PIN 22
#define SDA_PIN 21
#define READY_LED 32
#define LOCAL_LASER_LED 33
#define CONN_FAIL_LED 25
#define CONN_SUCCESS_LED 26
#define BUZZER 18
#define LASER_SENSOR 19

// CORRECTED: Using the MAC address from the start line module code
uint8_t startMac[] = {0x08, 0xA6, 0xF7, 0x65, 0xEF, 0x18};

// I2C LCD setup
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Message structure - must match start line module
typedef struct struct_message {
  uint8_t msgType;  // 0 for heartbeat, 1 for start timing
  bool localAligned;
  bool cooldown;
} struct_message;

// State variables
volatile bool finishTriggered = false;
bool localAligned = false;
bool remoteAligned = false;
bool connectionActive = false;
bool cooldown = false;
volatile bool timingActive = false;

// Timing variables
unsigned long lastHeartbeatReceived = 0;
volatile unsigned long startTime = 0;
unsigned long cooldownStart = 0;
unsigned long lastHeartbeatSent = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;
const unsigned long CONNECTION_TIMEOUT = 3000;
const unsigned long COOLDOWN_DURATION = 60000;
float lastElapsedTime = 0.0;

// Access Point configuration
const char* ssid = "TimeTracker";  // Name of the WiFi network
const char* password = "password123";  // Password for the WiFi network

// DNS and Web Server
DNSServer dnsServer;
WebServer server(80);

// Variables for web interface
bool cancelRequested = false;
bool skipCooldownRequested = false;

// Interrupt service routine for finish line detection
void IRAM_ATTR finishLaserInterrupt() {
  // This is called when the laser beam is broken at the finish line
  if (timingActive) {
    finishTriggered = true;
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optional: implement connection status LED updates here
}

// Callback when data is received - using the newer ESP-NOW API format
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  struct_message *msg = (struct_message *)incomingData;
  
  // Update based on heartbeat message
  if (msg->msgType == 0) {
    remoteAligned = msg->localAligned;
    lastHeartbeatReceived = millis();
    connectionActive = true;
  } 
  // Start timing when triggered by start line
  else if (msg->msgType == 1 && !cooldown && !timingActive) {
    timingActive = true;
    startTime = micros();
    
    // Clear LCD and show "Timing..."
    lcd.clear();
    lcd.print("Timing...");
  }
}

// Function to setup the Access Point
void setupAP() {
  // Configure ESP32 as access point
  WiFi.softAP(ssid, password);
  Serial.print("Access Point started: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // Start DNS server for captive portal
  dnsServer.start(53, "*", WiFi.softAPIP());

  // Setup web server routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/cancel", HTTP_GET, handleCancel);
  server.on("/skipcooldown", HTTP_GET, handleSkipCooldown);
  server.onNotFound(handleRoot);
  
  // Start web server
  server.begin();
  Serial.println("HTTP server started");
}

// Handle root path - serve the main page
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial; text-align: center; margin: 0; padding: 20px; }";
  html += ".container { max-width: 500px; margin: 0 auto; }";
  html += "h1 { color: #333; }";
  html += ".time { font-size: 48px; margin: 20px 0; font-weight: bold; }";
  html += ".status { margin: 15px 0; padding: 10px; border-radius: 5px; background-color: #f0f0f0; }";
  html += "button { background-color: #4CAF50; color: white; border: none; padding: 15px 25px; margin: 10px; border-radius: 5px; cursor: pointer; font-size: 16px; }";
  html += "button.cancel { background-color: #f44336; }";
  html += "button:disabled { background-color: #cccccc; }";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>Time Tracker</h1>";
  html += "<div id='time' class='time'>0.000s</div>";
  html += "<div id='status' class='status'>Initializing...</div>";
  html += "<div><button id='cancelBtn' class='cancel' onclick='cancelTiming()'>Cancel Timing</button></div>";
  html += "<div><button id='cooldownBtn' onclick='skipCooldown()'>Skip Cooldown</button></div>";
  html += "</div>";
  
  // JavaScript for updating time and status
  html += "<script>";
  html += "let timingActive = false;";
  html += "let cooldownActive = false;";
  html += "function updateData() {";
  html += "  fetch('/data').then(response => response.json()).then(data => {";
  html += "    document.getElementById('time').innerHTML = data.time + 's';";
  html += "    document.getElementById('status').innerHTML = data.status;";
  html += "    timingActive = data.timingActive;";
  html += "    cooldownActive = data.cooldown;";
  html += "    document.getElementById('cancelBtn').disabled = !timingActive;";
  html += "    document.getElementById('cooldownBtn').disabled = !cooldownActive;";
  html += "  });";
  html += "}";
  html += "function cancelTiming() {";
  html += "  fetch('/cancel');";
  html += "  document.getElementById('cancelBtn').disabled = true;";
  html += "}";
  html += "function skipCooldown() {";
  html += "  fetch('/skipcooldown');";
  html += "  document.getElementById('cooldownBtn').disabled = true;";
  html += "}";
  html += "setInterval(updateData, 100);"; // Update 10 times per second
  html += "updateData();";
  html += "</script></body></html>";
  
  server.send(200, "text/html", html);
}

// Handle data requests (AJAX)
void handleData() {
  String status;
  
  if (timingActive) {
    status = "Timing in progress...";
  } else if (cooldown) {
    unsigned long remaining = (COOLDOWN_DURATION - (millis() - cooldownStart)) / 1000;
    status = "Cooldown: " + String(remaining) + "s";
  } else if (localAligned && remoteAligned && connectionActive) {
    status = "Ready to time";
  } else if (!connectionActive) {
    status = "No connection";
  } else if (!localAligned) {
    status = "Align local laser";
  } else if (!remoteAligned) {
    status = "Align start laser";
  } else {
    status = "Setting up...";
  }
  
  String json = "{";
  json += "\"time\":" + String(lastElapsedTime, 3) + ",";
  json += "\"status\":\"" + status + "\",";
  json += "\"timingActive\":" + String(timingActive ? "true" : "false") + ",";
  json += "\"cooldown\":" + String(cooldown ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}

// Handle cancel request
void handleCancel() {
  cancelRequested = true;
  server.send(200, "text/plain", "OK");
}

// Handle skip cooldown request
void handleSkipCooldown() {
  skipCooldownRequested = true;
  server.send(200, "text/plain", "OK");
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(READY_LED, OUTPUT);
  pinMode(LOCAL_LASER_LED, OUTPUT);
  pinMode(CONN_FAIL_LED, OUTPUT);
  pinMode(CONN_SUCCESS_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LASER_SENSOR, INPUT);
  
  // Attach interrupt for laser break detection - FALLING edge when beam is broken
  attachInterrupt(digitalPinToInterrupt(LASER_SENSOR), finishLaserInterrupt, FALLING);

  // Initialize I2C and LCD
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100); // Add delay for stability
  
  // Try initializing the LCD with multiple attempts
  bool lcdInitialized = false;
  for (int attempt = 0; attempt < 3 && !lcdInitialized; attempt++) {
    try {
      lcd.init();
      lcdInitialized = true;
    } catch (...) {
      delay(500);
    }
  }
  
  if (lcdInitialized) {
    lcd.backlight();
    lcd.clear();
    lcd.print("Setting up...");
  } else {
    Serial.println("Failed to initialize LCD");
  }
  delay(1000); // Give LCD time to initialize properly

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, startMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
  
  // Set up access point for phone connectivity
  setupAP();
  
  // Disable WiFi sleep to reduce latency (optional timing improvement)
  WiFi.setSleep(false);
  
  Serial.println("Finish Line Module Initialized");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Handle DNS and web server requests
  dnsServer.processNextRequest();
  server.handleClient();
  
  // Update laser aligned status
  // Laser sensor returns HIGH when beam is detected, LOW when broken
  localAligned = digitalRead(LASER_SENSOR) == HIGH;
  digitalWrite(LOCAL_LASER_LED, localAligned ? HIGH : LOW);

  // Update connection status
  connectionActive = (currentMillis - lastHeartbeatReceived) < CONNECTION_TIMEOUT;
  digitalWrite(CONN_SUCCESS_LED, connectionActive ? HIGH : LOW);
  digitalWrite(CONN_FAIL_LED, connectionActive ? LOW : HIGH);

  // Update ready to time status
  bool readyToTime = localAligned && remoteAligned && connectionActive && !cooldown;
  digitalWrite(READY_LED, readyToTime ? HIGH : LOW);

  // Handle finish line trigger
  if (finishTriggered && timingActive) {
    float elapsed = (micros() - startTime) / 1000000.0;
    lastElapsedTime = elapsed;
    
    // Update LCD with time
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Time: ");
    lcd.print(elapsed, 3);
    lcd.print("s");
    
    // Sound buzzer
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    
    // Reset timing state and start cooldown
    timingActive = false;
    cooldown = true;
    cooldownStart = currentMillis;
    finishTriggered = false;
  }

  // Handle cancel request from phone
  if (cancelRequested && timingActive) {
    timingActive = false;
    lcd.clear();
    lcd.print("Timing Cancelled");
    cooldown = true;
    cooldownStart = currentMillis;
    cancelRequested = false;
  }

  // Handle skip cooldown request from phone
  if (skipCooldownRequested && cooldown) {
    cooldown = false;
    skipCooldownRequested = false;
    lcd.clear();
    lcd.print("Cooldown Skipped");
    delay(1000);
  }

  // Check cooldown status
  if (cooldown && (currentMillis - cooldownStart >= COOLDOWN_DURATION)) {
    cooldown = false;
  }

  // Update LCD display (only update periodically to avoid flicker)
  static unsigned long lastLCDUpdate = 0;
  if (currentMillis - lastLCDUpdate >= 500) {
    if (!timingActive && !finishTriggered) {
      lcd.clear();
      lcd.setCursor(0, 0);
      
      if (cooldown) {
        unsigned long remaining = (COOLDOWN_DURATION - (currentMillis - cooldownStart)) / 1000;
        lcd.print("Cooldown: ");
        lcd.print(remaining);
        lcd.print("s");
        lcd.setCursor(0, 1);
        lcd.print("Last time: ");
        lcd.print(lastElapsedTime, 3);
        lcd.print("s");
      } else if (readyToTime) {
        lcd.print("Ready to time");
        if (lastElapsedTime > 0) {
          lcd.setCursor(0, 1);
          lcd.print("Last time: ");
          lcd.print(lastElapsedTime, 3);
          lcd.print("s");
        }
      } else {
        if (!connectionActive) {
          lcd.print("No connection");
        } else if (!localAligned) {
          lcd.print("Align local laser");
        } else if (!remoteAligned) {
          lcd.print("Align start laser");
        } else {
          lcd.print("Setting up...");
        }
      }
    }
    lastLCDUpdate = currentMillis;
  }

  // Send periodic heartbeats
  if (currentMillis - lastHeartbeatSent >= HEARTBEAT_INTERVAL) {
    struct_message heartbeatMsg;
    heartbeatMsg.msgType = 0;
    heartbeatMsg.localAligned = localAligned;
    heartbeatMsg.cooldown = cooldown;
    esp_now_send(startMac, (uint8_t *)&heartbeatMsg, sizeof(heartbeatMsg));
    lastHeartbeatSent = currentMillis;
  }
  
  // Small delay to prevent CPU hogging - using yield() instead of delay
  // for potentially better timing accuracy
  yield();
}
