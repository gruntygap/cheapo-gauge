// i2c
#include <Wire.h>
// screen rendering libs
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBoldOblique9pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
// CAN BUS libs
#include <SPI.h>
#include <mcp_can.h>
#include <LittleFS.h>

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

// CAN Output config - match these to your remote device settings
#define REMOTE_CAN_ID 0x05       // CAN ID for outgoing port data
#define REMOTE_TABLE 7           // Remote table number for ports data
#define REMOTE_OFFSET 75         // Remote table offset for ports data (bytes)
#define CAN_SEND_INTERVAL 100    // ms between CAN transmits

// State persistence
#define STATE_FILE "/port_state.bin"

MCP_CAN CAN(CAN_CS);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long buttonPressTime = 0;
unsigned long lastTapTime = 0;
bool buttonPressed = false;
bool waitingForSecondPress = false;
bool longPressDetected = false;

const unsigned long tapThreshold = 300;
const unsigned long longPressThreshold = 500;

// Modes
enum DisplayMode { BOOST, ETHANOL, VOLTAGE, TPS, MATCLT, BAROMAP, EGO, PORT1, PORT2, PORT3 };
#define NUM_MODES 10
DisplayMode currentMode = BOOST;

// Sensor data
float psi = 0.0, maxPsi = -14.7;
float baro = 0.0, maps = 0.0, mat = 0.0, clt = 0.0;
float ethanolContent = 0.0;
double tps = 0.0;
float batt = 0.0, ego1 = 0.0, ego2 = 0.0;

// Port output states (persisted to flash)
bool port1State = false;
bool port2State = false;
uint8_t port3State = 0x00;
int port3Cursor = 0;
unsigned long lastCANSend = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ;
  Serial.println("Serial connected");

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found");
    while (1)
      ;
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.display();

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("CAN Bus Initialized");
  } else {
    Serial.println("CAN Bus Init Failed");
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("CAN Bus Init Failed");
    display.display();
    while (1)
      ;
  }

  CAN.setMode(MCP_NORMAL);
  pinMode(CAN_INT, INPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);

  loadState();
}

void drawBoost2() {
  display.clearDisplay();

  // --- Draw Boost Bar at top ---
  int barY = 0;
  int barHeight = 6;
  int zeroPoint = map(0 * 10, -147, 200, 0, SCREEN_WIDTH);
  int barX = map(psi * 10, -147, 200, 0, SCREEN_WIDTH);
  barX = constrain(barX, 0, SCREEN_WIDTH);

  display.drawLine(zeroPoint, barY, zeroPoint, barY + barHeight, WHITE);
  if (barX >= zeroPoint)
    display.fillRect(zeroPoint, barY, barX - zeroPoint, barHeight, WHITE);
  else
    display.fillRect(barX, barY, zeroPoint - barX, barHeight, WHITE);

  display.drawRect(0, barY, SCREEN_WIDTH, barHeight, WHITE);

  // --- Draw Max label and value at top right ---
  display.setFont();
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds("Max:", 0, 0, &x1, &y1, &w, &h);
  int maxLabelX = SCREEN_WIDTH - (w + x1);
  int maxLabelY = SCREEN_HEIGHT - (h * 2 + 1);
  display.setCursor(maxLabelX, maxLabelY);
  display.print("Max:");

  char maxStr[6];
  snprintf(maxStr, sizeof(maxStr), "%.1f", maxPsi);
  display.getTextBounds(maxStr, 0, 0, &x1, &y1, &w, &h);
  int maxValX = SCREEN_WIDTH - (w + x1);
  int maxValY = SCREEN_HEIGHT - h;
  display.setCursor(maxValX, maxValY);
  display.print(maxStr);

  // --- Draw current PSI (left-aligned, bottom-left) ---
  display.setFont(&FreeMonoBold12pt7b);
  char psiStr[8];
  snprintf(psiStr, sizeof(psiStr), "%5.1f", psi);
  display.getTextBounds("00.0", 0, 0, &x1, &y1, &w, &h);
  int psiX = 0;
  int psiY = SCREEN_HEIGHT - 1;
  display.setCursor(psiX, psiY);
  display.print(psiStr);

  // --- Optional: Draw "PSI" label just after number ---
  display.setFont(&FreeSansBoldOblique9pt7b);
  display.setCursor((SCREEN_WIDTH / 2) + 4, psiY);
  display.print("PSI");

  display.setFont();  // Reset font
  display.display();
}

void drawBoost() {
  display.clearDisplay();
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.printf("%5.1f PSI", psi);

  display.setTextSize(1);
  display.setCursor(SCREEN_WIDTH - 60, SCREEN_HEIGHT - 7);
  display.print("Max: ");
  display.print(maxPsi, 1);

  int zeroPoint = map(0 * 10, -147, 300, 0, SCREEN_WIDTH);
  int barX = map(psi * 10, -147, 300, 0, SCREEN_WIDTH);
  barX = constrain(barX, 0, SCREEN_WIDTH);

  display.drawLine(zeroPoint, 16, zeroPoint, 25, WHITE);
  if (barX >= zeroPoint)
    display.fillRect(zeroPoint, 18, barX - zeroPoint, 6, WHITE);
  else
    display.fillRect(barX, 18, zeroPoint - barX, 6, WHITE);

  display.drawRect(0, 18, SCREEN_WIDTH, 6, WHITE);
  display.display();
}

void drawSingleStat(const char* value, const char* unit, const char* label) {
  int16_t x1, y1;
  uint16_t w, h;
  display.clearDisplay();

  // --- Main numeric value ---
  display.setFont(&FreeSans18pt7b);

  // Calculate position for big value
  display.getTextBounds(value, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(0, SCREEN_HEIGHT - (h + y1));
  display.print(value);

  // --- Unit symbol (e.g. %, PSI) next to value ---
  display.setFont(&FreeSans9pt7b);
  display.print(unit);

  // --- Bottom right label (e.g. "Ethanol") ---
  display.setFont(&FreeSansBoldOblique9pt7b);
  display.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
  int cursorX = SCREEN_WIDTH - (w + x1);
  int cursorY = 0 - y1;
  display.setCursor(cursorX, cursorY);
  display.print(label);

  // Reset and display
  display.setFont();
  display.display();
}

void drawSensor2(const char* value, const char* unit, const char* label, const char* value2, const char* unit2, const char* label2) {
  int16_t x1, y1;
  uint16_t w, h;
  display.clearDisplay();

  // First Sensor Value
  display.setFont(&FreeSans9pt7b);
  // Calculate position for value
  display.getTextBounds(String(value) + label, 0, 0, &x1, &y1, &w, &h);
  // TOP LEFT
  display.setCursor(0, 0 - y1);
  display.print(value);
  // --- Unit symbol (e.g. %, PSI) next to value ---
  display.print(unit);

  // --- right label ---
  display.setFont(&FreeSansBoldOblique9pt7b);
  display.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
  int cursorX = SCREEN_WIDTH - (w + x1);  // Right
  int cursorY = 0 - y1;                   // TOP
  display.setCursor(cursorX, cursorY);
  display.print(label);

  // Second Sensor Value
  display.setFont(&FreeSans9pt7b);
  // Calculate position for value
  display.getTextBounds(String(value2) + label2, 0, 0, &x1, &y1, &w, &h);
  // BOTTOM LEFT
  display.setCursor(0, SCREEN_HEIGHT - (h + y1));
  display.print(value2);
  // --- Unit symbol (e.g. %, PSI) next to value ---
  display.print(unit2);

  // --- right lower label ---
  display.setFont(&FreeSansBoldOblique9pt7b);
  display.getTextBounds(label2, 0, 0, &x1, &y1, &w, &h);
  cursorX = SCREEN_WIDTH - (w + x1);   // Right
  cursorY = SCREEN_HEIGHT - (h + y1);  // Bottom
  display.setCursor(cursorX, cursorY);
  display.print(label2);

  // Reset and display
  display.setFont();
  display.display();
}

// --- State Persistence ---

void loadState() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed, formatting...");
    LittleFS.format();
    LittleFS.begin();
  }
  File f = LittleFS.open(STATE_FILE, "r");
  if (f) {
    port1State = f.read() != 0;
    port2State = f.read() != 0;
    port3State = f.read();
    f.close();
    Serial.println("Port state loaded from flash");
  } else {
    Serial.println("No saved port state found");
  }
}

void saveState() {
  File f = LittleFS.open(STATE_FILE, "w");
  if (f) {
    f.write((uint8_t)(port1State ? 1 : 0));
    f.write((uint8_t)(port2State ? 1 : 0));
    f.write(port3State);
    f.close();
  }
}

// --- CAN Output ---

void sendCANPorts() {
  unsigned long now = millis();
  if (now - lastCANSend < CAN_SEND_INTERVAL) return;
  lastCANSend = now;

  byte data[8] = {0};
  // Byte 0: table number
  data[0] = REMOTE_TABLE;
  // Bytes 1-2: offset (16-bit big-endian)
  data[1] = (REMOTE_OFFSET >> 8) & 0xFF;
  data[2] = REMOTE_OFFSET & 0xFF;
  // Byte 3: Port 1 (0=OFF, 1=ON)
  data[3] = port1State ? 1 : 0;
  // Byte 4: Port 2 (0=OFF, 1=ON)
  data[4] = port2State ? 1 : 0;
  // Byte 5: Port 3 (8 digital signals bitmask)
  data[5] = port3State;

  CAN.sendMsgBuf(REMOTE_CAN_ID, 0, 8, data);
}

// --- Port Display Functions ---

void drawPort1() {
  drawSingleStat(port1State ? "ON" : "OFF", "", "Port 1");
}

void drawPort2() {
  drawSingleStat(port2State ? "ON" : "OFF", "", "Port 2");
}

void drawPort3() {
  display.clearDisplay();

  // Title + selected bit info
  display.setFont();
  display.setCursor(0, 0);
  display.print("Port 3");
  display.setCursor(76, 0);
  display.print("Bit");
  display.print(port3Cursor + 1);
  display.print(":");
  display.print((port3State & (1 << port3Cursor)) ? "ON" : "OFF");

  // Draw 8 bit boxes
  int boxW = 12, boxH = 12;
  int gap = 2;
  int startX = (SCREEN_WIDTH - (8 * boxW + 7 * gap)) / 2;
  int boxY = 10;

  for (int i = 0; i < 8; i++) {
    int x = startX + i * (boxW + gap);
    bool on = port3State & (1 << i);

    if (on) {
      display.fillRect(x, boxY, boxW, boxH, WHITE);
      display.setTextColor(BLACK);
      display.setCursor(x + 3, boxY + 2);
      display.print(i + 1);
      display.setTextColor(WHITE);
    } else {
      display.drawRect(x, boxY, boxW, boxH, WHITE);
      display.setCursor(x + 3, boxY + 2);
      display.print(i + 1);
    }

    // Cursor triangle below selected bit
    if (i == port3Cursor) {
      int cx = x + boxW / 2;
      display.fillTriangle(cx - 3, boxY + boxH + 4, cx + 3, boxY + boxH + 4, cx, boxY + boxH + 1, WHITE);
    }
  }

  display.display();
}

void updateCAN() {
  if (!digitalRead(CAN_INT)) {
    long unsigned int rxId;
    byte len = 0;
    byte buf[8];

    // 1520 base id
    // 02 - BARO, MAP, MAT, CLT
    // 03 - TPS, BATT, EGO1, EGO2
    // 47 is eth

    if (CAN.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      int base = 0x5F0;

      if (rxId == base + 2) {
        baro = ((buf[0] << 8) | buf[1]) / 10.0;
        maps = ((buf[2] << 8) | buf[3]) / 10.0;
        mat = ((buf[4] << 8) | buf[5]) / 10.0;
        clt = ((buf[6] << 8) | buf[7]) / 10.0;
        psi = (maps - baro) * 0.145038;
        if (psi > maxPsi) maxPsi = psi;
      }
      if (rxId == base + 3) {
        tps = (int16_t)((buf[0] << 8) | buf[1]) / 10.0;
        batt = ((buf[2] << 8) | buf[3]) / 10.0;
        ego1 = ((buf[4] << 8) | buf[5]) / 10.0;
        ego2 = ((buf[6] << 8) | buf[7]) / 10.0;
      }
      if (rxId == base + 47) {
        //         Serial.print("ID: 0x");
        // Serial.print(rxId, HEX);
        // Serial.print("  Data: ");
        // for (byte i = 0; i < len; i++) {
        //   if (buf[i] < 0x10) Serial.print("0");
        //   Serial.print(buf[i], HEX);
        //   Serial.print(" ");
        // }
        // Serial.println();

        ethanolContent = ((buf[0] << 8) | buf[1]) / 10.0;
      }
    }
  }
}

void handleButton() {
  static bool inLongPress = false;
  int state = digitalRead(RESET_PIN);
  unsigned long now = millis();

  if (state == LOW && !buttonPressed) {
    buttonPressed = true;
    buttonPressTime = now;
    inLongPress = false;
  }

  if (state == HIGH && buttonPressed) {
    buttonPressed = false;
    unsigned long duration = now - buttonPressTime;

    if (duration >= longPressThreshold) {
      // Long press: toggle port value on port screens, reset max on others
      if (currentMode == PORT1) {
        port1State = !port1State;
        saveState();
      } else if (currentMode == PORT2) {
        port2State = !port2State;
        saveState();
      } else if (currentMode == PORT3) {
        port3State ^= (1 << port3Cursor);
        saveState();
      } else {
        maxPsi = -14.7;
        psi = -14.7;
      }
      inLongPress = true;
    } else {
      if (waitingForSecondPress && (now - lastTapTime) <= tapThreshold) {
        // Double press: previous screen
        waitingForSecondPress = false;
        if (currentMode == PORT3) port3Cursor = 0;
        currentMode = (DisplayMode)(((int)currentMode - 1 + NUM_MODES) % NUM_MODES);
      } else {
        waitingForSecondPress = true;
        lastTapTime = now;
      }
    }
  }

  if (waitingForSecondPress && (now - lastTapTime) > tapThreshold) {
    if (!inLongPress) {
      // Single press: advance cursor on PORT3, or next screen
      if (currentMode == PORT3 && port3Cursor < 7) {
        port3Cursor++;
      } else {
        if (currentMode == PORT3) port3Cursor = 0;
        currentMode = (DisplayMode)(((int)currentMode + 1) % NUM_MODES);
      }
    }
    waitingForSecondPress = false;
  }
}

void loop() {
  handleButton();
  updateCAN();

  switch (currentMode) {
    case BOOST:
      drawBoost2();
      break;
    case ETHANOL:
      char ethBuffer[8];
      snprintf(ethBuffer, sizeof(ethBuffer), "E%2.1f", ethanolContent);
      drawSingleStat(ethBuffer, "%", "Eth");
      break;
    case VOLTAGE:
      char battBuffer[8];
      snprintf(battBuffer, sizeof(battBuffer), "%02.1f", batt);
      drawSingleStat(battBuffer, "v", "Battery");
      break;
    case TPS:
      char tpsBuffer[8];
      if (tps < 100.0) {
        snprintf(tpsBuffer, sizeof(tpsBuffer), "%04.1f", tps);  // Keep one decimal for < 100
      } else {
        snprintf(tpsBuffer, sizeof(tpsBuffer), "%.0f", tps);  // Truncate for >= 100
      }
      drawSingleStat(tpsBuffer, "%", "Throttl");
      break;
    case MATCLT:
      char matBuffer[8];
      snprintf(matBuffer, sizeof(matBuffer), "%.1f", mat);
      char cltBuffer[8];
      snprintf(cltBuffer, sizeof(cltBuffer), "%.1f", clt);
      drawSensor2(matBuffer, " C", "MAT", cltBuffer, " C", "CLT");
      break;
    case BAROMAP:
      char baroBuffer[8];
      snprintf(baroBuffer, sizeof(baroBuffer), "%.1f", baro);
      char mapsBuffer[8];
      snprintf(mapsBuffer, sizeof(mapsBuffer), "%.1f", maps);
      drawSensor2(baroBuffer, "kpa", "BARO", mapsBuffer, "kpa", "MAP");
      break;
    case EGO:
      char ego1Buf[8];
      snprintf(ego1Buf, sizeof(ego1Buf), "%.1f", ego1);
      char ego2Buf[8];
      snprintf(ego2Buf, sizeof(ego2Buf), "%.1f", ego2);
      drawSensor2(ego1Buf, "", "EGO1", ego2Buf, "", "EGO2");
      break;
    case PORT1:
      drawPort1();
      break;
    case PORT2:
      drawPort2();
      break;
    case PORT3:
      drawPort3();
      break;
  }

  sendCANPorts();
}
