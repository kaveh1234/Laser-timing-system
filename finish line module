#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

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
bool timingActive = false;

// Timing variables
unsigned long lastHeartbeatReceived = 0;
unsigned long startTime = 0;
unsigned long cooldownStart = 0;
unsigned long lastHeartbeatSent = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;
const unsigned long CONNECTION_TIMEOUT = 3000;
const unsigned long COOLDOWN_DURATION = 60000;
float lastElapsedTime = 0.0;

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
  
  Serial.println("Finish Line Module Initialized");
}

void loop() {
  unsigned long currentMillis = millis();
  
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
  
  // Small delay to prevent CPU hogging
  delay(10);
}
