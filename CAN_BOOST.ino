
// i2c
#include <Wire.h>
// screen rendering libs
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// CAN BUS libs
#include <SPI.h>
#include <mcp_can.h>

// Button Def
#define RESET_PIN 26
// Screen Def
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDRESS 0x3C
#define OLED_SDA 6
#define OLED_SCL 7
// CAN def
#define CAN_CS D7   // P1 in case       // Chip Select pin for MCP2515
#define CAN_INT 10  // INT pin on MCP2515

MCP_CAN CAN(CAN_CS);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long buttonPressTime = 0;
unsigned long lastTapTime = 0;
bool buttonPressed = false;
bool waitingForSecondPress = false;
bool freezeUpdates = false;

const unsigned long tapThreshold = 300;  // ms between taps for double tap
const unsigned long longPressThreshold = 500;

// Boost gauge config
float psi = 0.0;
float maxPsi = -99.9;          // allow for negative readings
const float minScale = -14.7;  // full vacuum
const float maxScale = 30.0;   // max boost

void setup() {
  // init serial for logging
  Serial.begin(115200);
  Serial.println("Waiting for Serial...");
  while (!Serial && millis() < 3000)
    ;
  Serial.println("Serial connected");

  // init i2c for screen
  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();

  // init screen
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found");
    while (1)
      ;
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.display();

  // init CAN @ 500 kbps
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("CAN Bus Initialized");
  } else {
    Serial.println("CAN Bus Init Failed");
    display.clearDisplay();
    display.setCursor(SCREEN_WIDTH - 50, 0);
    display.setTextSize(1);
    display.print("CAN Bus Init Failed");
    display.display();
    while (1)
      ;
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);

  // init button
  pinMode(RESET_PIN, INPUT_PULLUP);
}

void drawBoostGauge(float psi, float maxPsi) {
  display.clearDisplay();

  // --- Draw PSI readout at top-left ---
  display.setTextSize(2);
  display.setCursor(0, 0);
  if (psi >= 0) {
    // Account for spacing with -
    display.print(" ");
  }
  if (abs(psi) < 10) {
    display.print(" ");
  }
  display.print(psi, 1);
  display.print(" PSI");

  // --- Draw max PSI (bottom right) ---
  display.setTextSize(1);
  display.setCursor(SCREEN_WIDTH - 60, SCREEN_HEIGHT - 7);
  display.print("Max: ");
  display.print(maxPsi, 1);

  // --- Draw boost bar ---
  // Scale psi from minScale → maxScale → bar from 0 to SCREEN_WIDTH
  int zeroPoint = map(0 * 10, minScale * 10, maxScale * 10, 0, SCREEN_WIDTH);  // midpoint
  int barX = map(psi * 10, minScale * 10, maxScale * 10, 0, SCREEN_WIDTH);
  barX = constrain(barX, 0, SCREEN_WIDTH);

  // Draw center zero line
  display.drawLine(zeroPoint, 16, zeroPoint, 25, WHITE);

  // Draw bar from zero to current PSI
  if (barX >= zeroPoint) {
    // Positive boost (right side)
    display.fillRect(zeroPoint, 18, barX - zeroPoint, 6, WHITE);
  } else {
    // Vacuum (left side)
    display.fillRect(barX, 18, zeroPoint - barX, 6, WHITE);
  }

  // Draw outline
  display.drawRect(0, 18, SCREEN_WIDTH, 6, WHITE);

  display.display();
}

void loop() {
  int buttonState = digitalRead(RESET_PIN);
  unsigned long currentTime = millis();

  if (buttonState == LOW && !buttonPressed) {
    // Button just pressed
    buttonPressed = true;
    buttonPressTime = currentTime;
  }

  if (buttonState == HIGH && buttonPressed) {
    // Button released
    buttonPressed = false;
    unsigned long pressDuration = currentTime - buttonPressTime;

    if (pressDuration >= longPressThreshold) {
      // LONG press
      freezeUpdates = !freezeUpdates;  // toggle freeze state
    } else {
      // Short press or double press detect
      if (waitingForSecondPress && (currentTime - lastTapTime) <= tapThreshold) {
        // Double press detected
        waitingForSecondPress = false;
        freezeUpdates = !freezeUpdates;
      } else {
        // Single press detected, waiting on second?
        waitingForSecondPress = true;
        lastTapTime = currentTime;
      }
    }
  }

  if (waitingForSecondPress && (currentTime - lastTapTime) > tapThreshold) {
    // Single press confirmed
    waitingForSecondPress = false;
    maxPsi = -14.7;
  }

  if (!freezeUpdates) {
    
    if (!digitalRead(CAN_INT)) {
      long unsigned int rxId;
      unsigned char len = 0;
      unsigned char buf[8];
      int base = 0x5F0;
      // 1520 base id
      // 02 - BARO, MAP, MAT, CLT
      // 03 - TPS, BATT, EGO1, EGO2
      // 47 is eth

      if (CAN.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
        if (rxId == base+2) {
          float baro = ((buf[0] << 8) | buf[1]) / 10.0;
          float map = ((buf[2] << 8) | buf[3]) / 10.0;
          float mat = ((buf[4] << 8) | buf[5]) / 10.0;
          float clt = ((buf[6] << 8) | buf[7]) / 10.0;
          Serial.print("Baro: ");
          Serial.print(baro);
          Serial.print(" kPa, ");
          Serial.print("MAP: ");
          Serial.print(map);
          Serial.print(" kPa, ");
          Serial.print("MAT: ");
          Serial.print(mat);
          Serial.print(" °C, ");
          Serial.print("CLT: ");
          Serial.print(clt);
          Serial.println(" °C");
          psi = (map - baro) * 0.145038;
          if (psi > maxPsi) maxPsi = psi;
          drawBoostGauge(psi, maxPsi);
        }
        if (rxId == base+3) {
          float tps = ((buf[0] << 8) | buf[1]) / 10.0;
          float batt = ((buf[2] << 8) | buf[3]) / 10.0;
          float ego1 = ((buf[4] << 8) | buf[5]) / 10.0;
          float ego2 = ((buf[6] << 8) | buf[7]) / 10.0;
          Serial.print("tps: ");
          Serial.print(tps);
          Serial.print(" %, ");
          Serial.print("batt: ");
          Serial.print(batt);
          Serial.print(" v, ");
          Serial.print("ego1: ");
          Serial.print(ego1);
          Serial.print(" ?, ");
          Serial.print("ego2: ");
          Serial.print(ego2);
          Serial.println(" ?");
        }
      }
    }
  }

  if (freezeUpdates) {
    display.clearDisplay();
    display.setCursor(SCREEN_WIDTH - 50, 0);
    display.setTextSize(1);
    display.print("[HOLD]");
    display.display();
    delay(1000);
    freezeUpdates = false;
  }
}
