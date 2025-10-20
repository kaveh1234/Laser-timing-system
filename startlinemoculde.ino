#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Define pin assignments
#define READY_LED 32
#define LASER_ALIGNED_LED 33
#define CONNECTION_FAIL_LED 25
#define CONNECTION_SUCCESS_LED 26
#define BUZZER 22
#define LASER_SENSOR 23

// CORRECTED: Using the MAC address from the finish line module code
uint8_t finishLineMacAddress[] = {0xE8, 0x6B, 0xEA, 0xDF, 0xA2, 0x68};

// Timing variables
volatile bool laserAligned = false;
bool connectionActive = false;
bool readyToTime = false;
bool timingCycleActive = false;

// Communication variables
typedef struct struct_message {
  uint8_t msgType;  // 0 for heartbeat, 1 for start timing
  bool localAligned;
  bool cooldown;
} struct_message;

struct_message outgoingMsg;
struct_message incomingMsg;

// Last time we received a heartbeat from the finish line
unsigned long lastHeartbeatReceived = 0;
const unsigned long HEARTBEAT_TIMEOUT = 3000;  // 3 seconds timeout
const unsigned long HEARTBEAT_INTERVAL = 1000; // Send heartbeat every 1 second
unsigned long lastHeartbeatSent = 0;
bool remoteAligned = false;
bool remoteCooldown = false;

// Function prototypes
void sendHeartbeat();
void sendStartSignal();
void initESPNow();
void updateReadyToTime();

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    digitalWrite(CONNECTION_FAIL_LED, HIGH);
    digitalWrite(CONNECTION_SUCCESS_LED, LOW);
    connectionActive = false;
  } else {
    digitalWrite(CONNECTION_FAIL_LED, LOW);
    digitalWrite(CONNECTION_SUCCESS_LED, HIGH);
  }
}

// Callback when data is received - using the newer ESP-NOW API format
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&incomingMsg, incomingData, sizeof(incomingMsg));
  
  if (incomingMsg.msgType == 0) {  // Heartbeat message
    lastHeartbeatReceived = millis();
    connectionActive = true;
    remoteAligned = incomingMsg.localAligned;
    remoteCooldown = incomingMsg.cooldown;
    
    // Update ready to time status
    updateReadyToTime();
  }
}

// Interrupt service routine for laser break detection
void IRAM_ATTR laserBreakISR() {
  // This will be called when beam is broken (sensor goes from HIGH to LOW)
  if (readyToTime && !timingCycleActive) {
    timingCycleActive = true;
    sendStartSignal();
    tone(BUZZER, 1000, 100);  // Beep to indicate timing start
  }
}

// Update the laser aligned status
void updateLaserStatus() {
  // Laser sensor returns HIGH when beam is detected, LOW when broken
  laserAligned = digitalRead(LASER_SENSOR) == HIGH;
  
  // Update LED
  digitalWrite(LASER_ALIGNED_LED, laserAligned ? HIGH : LOW);
}

// Update the ready to time status
void updateReadyToTime() {
  readyToTime = laserAligned && remoteAligned && connectionActive && !remoteCooldown && !timingCycleActive;
  digitalWrite(READY_LED, readyToTime ? HIGH : LOW);
}

// Initialize ESP-NOW
void initESPNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, finishLineMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}

// Send heartbeat message to finish line module
void sendHeartbeat() {
  outgoingMsg.msgType = 0;
  outgoingMsg.localAligned = laserAligned;
  outgoingMsg.cooldown = false;
  
  esp_err_t result = esp_now_send(finishLineMacAddress, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
  
  if (result != ESP_OK) {
    connectionActive = false;
    digitalWrite(CONNECTION_FAIL_LED, HIGH);
    digitalWrite(CONNECTION_SUCCESS_LED, LOW);
  }
}

// Send start timing signal
void sendStartSignal() {
  outgoingMsg.msgType = 1;  // Start timing message
  outgoingMsg.localAligned = laserAligned;
  outgoingMsg.cooldown = false;
  
  esp_err_t result = esp_now_send(finishLineMacAddress, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
  
  if (result != ESP_OK) {
    connectionActive = false;
    digitalWrite(CONNECTION_FAIL_LED, HIGH);
    digitalWrite(CONNECTION_SUCCESS_LED, LOW);
  }
}

void checkConnection() {
  // Check if we haven't received a heartbeat for too long
  if (millis() - lastHeartbeatReceived > HEARTBEAT_TIMEOUT) {
    connectionActive = false;
    digitalWrite(CONNECTION_FAIL_LED, HIGH);
    digitalWrite(CONNECTION_SUCCESS_LED, LOW);
  }
  
  // Send periodic heartbeats
  if (millis() - lastHeartbeatSent > HEARTBEAT_INTERVAL) {
    sendHeartbeat();
    lastHeartbeatSent = millis();
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(READY_LED, OUTPUT);
  pinMode(LASER_ALIGNED_LED, OUTPUT);
  pinMode(CONNECTION_FAIL_LED, OUTPUT);
  pinMode(CONNECTION_SUCCESS_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LASER_SENSOR, INPUT);
  
  // Set initial LED states
  digitalWrite(READY_LED, LOW);
  digitalWrite(LASER_ALIGNED_LED, LOW);
  digitalWrite(CONNECTION_FAIL_LED, HIGH);
  digitalWrite(CONNECTION_SUCCESS_LED, LOW);
  
  // Initialize ESP-NOW
  initESPNow();
  
  // Attach interrupt for laser break detection - FALLING edge when beam is broken
  attachInterrupt(digitalPinToInterrupt(LASER_SENSOR), laserBreakISR, FALLING);
  
  Serial.println("Start Line Module Initialized");
}

void loop() {
  // Update sensor status
  updateLaserStatus();
  
  // Check connection status
  checkConnection();
  
  // Update ready to time status
  updateReadyToTime();
  
  // Reset timing cycle if finish line is in cooldown mode
  if (timingCycleActive && remoteCooldown) {
    timingCycleActive = false;
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}
