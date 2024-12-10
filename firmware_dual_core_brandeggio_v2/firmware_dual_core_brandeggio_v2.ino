{\rtf1\ansi\ansicpg1252\cocoartf2759
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\paperw11900\paperh16840\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs24 \cf0 #include <esp_task_wdt.h>\
#include <ESP32_Servo.h>\
#include <EEPROM.h>\
\
#define EEPROM_SIZE 64\
#define MIN_TILT 1000\
#define MAX_TILT 2000\
#define MIN_PAN 500\
#define MAX_PAN 2500\
#define SERIAL_BAUD_RATE 115200\
#define CAM_BAUD_RATE 38400\
#define SERVO_TILT_PIN 19\
#define SERVO_PAN_PIN 21\
#define LED_PIN 2\
\
Servo tiltServo;\
Servo panServo;\
\
int tiltValue;\
int panValue;\
int currentCommand = 0;\
bool commandComplete = false;\
int movementSpeedTilt;\
int movementSpeedPan;\
\
byte checksum;\
\
HardwareSerial cam(1); // Camera Serial Port\
\
// Utility function for checksum calculation\
byte calculateChecksum(const int *data, size_t length) \{\
  byte sum = 0;\
  for (size_t i = 0; i < length; ++i) \{\
    sum += data[i];\
  \}\
  return sum - 512;\
\}\
\
// Function to read and validate EEPROM data\
void loadServoPositionsFromEEPROM() \{\
  tiltValue = (EEPROM.read(0) << 8) | EEPROM.read(1);\
  panValue = (EEPROM.read(2) << 8) | EEPROM.read(3);\
\
  // Clamp values within valid ranges\
  tiltValue = constrain(tiltValue, MIN_TILT, MAX_TILT);\
  panValue = constrain(panValue, MIN_PAN, MAX_PAN);\
\}\
\
// Function to save servo positions to EEPROM\
void saveServoPositionsToEEPROM(int tilt, int pan) \{\
  EEPROM.write(0, highByte(tilt));\
  EEPROM.write(1, lowByte(tilt));\
  EEPROM.write(2, highByte(pan));\
  EEPROM.write(3, lowByte(pan));\
  EEPROM.commit();\
\}\
\
// Move servos to a specific position\
void moveToPosition(int tilt, int pan) \{\
  tiltServo.writeMicroseconds(constrain(tilt, MIN_TILT, MAX_TILT));\
  panServo.writeMicroseconds(constrain(pan, MIN_PAN, MAX_PAN));\
\}\
\
// Handle incoming serial commands\
void handleIncomingCommand() \{\
  static int commandBuffer[128];\
  static int bufferIndex = 0;\
\
  while (cam.available() > 0) \{\
    int incomingByte = cam.read();\
\
    // Process valid command sequence\
    if (incomingByte == 0xA8) \{\
      commandBuffer[bufferIndex++] = incomingByte;\
      if (bufferIndex > 1) \{\
        commandComplete = true;\
      \}\
    \} else \{\
      bufferIndex = 0; // Reset buffer on invalid data\
    \}\
\
    if (commandComplete) \{\
      currentCommand = commandBuffer[1]; // Process command based on the second byte\
      bufferIndex = 0;\
      commandComplete = false;\
    \}\
  \}\
\}\
\
// Handle movement based on commands\
void executeMovement() \{\
  switch (currentCommand) \{\
    case 1: // Move up\
      tiltValue = min(tiltValue + movementSpeedTilt, MAX_TILT);\
      break;\
    case 2: // Move down\
      tiltValue = max(tiltValue - movementSpeedTilt, MIN_TILT);\
      break;\
    case 3: // Move right\
      panValue = min(panValue + movementSpeedPan, MAX_PAN);\
      break;\
    case 4: // Move left\
      panValue = max(panValue - movementSpeedPan, MIN_PAN);\
      break;\
    // Additional cases for diagonal movements, etc.\
    default:\
      break;\
  \}\
  moveToPosition(tiltValue, panValue);\
  currentCommand = 0; // Reset command after execution\
\}\
\
// Task for periodic servo updates\
void movementTask(void *parameter) \{\
  while (true) \{\
    if (currentCommand != 0) \{\
      executeMovement();\
    \}\
    vTaskDelay(pdMS_TO_TICKS(20)); // Delay to simulate movement speed\
  \}\
\}\
\
void setup() \{\
  // Setup serial communication\
  Serial.begin(SERIAL_BAUD_RATE);\
  cam.begin(CAM_BAUD_RATE, SERIAL_8N1, 22, 23);\
\
  // Initialize servos\
  tiltServo.attach(SERVO_TILT_PIN, 1000, 2000);\
  panServo.attach(SERVO_PAN_PIN, 500, 2500);\
\
  // Setup LED\
  pinMode(LED_PIN, OUTPUT);\
\
  // Initialize EEPROM\
  if (!EEPROM.begin(EEPROM_SIZE)) \{\
    Serial.println("Failed to initialize EEPROM. Restarting...");\
    ESP.restart();\
  \}\
\
  // Load initial servo positions\
  loadServoPositionsFromEEPROM();\
  moveToPosition(tiltValue, panValue);\
\
  // Create movement task\
  xTaskCreatePinnedToCore(movementTask, "MovementTask", 10000, NULL, 1, NULL, 1);\
\
  Serial.println("Setup complete.");\
\}\
\
void loop() \{\
  handleIncomingCommand();\
  esp_task_wdt_reset(); // Reset watchdog timer\
\}\
}