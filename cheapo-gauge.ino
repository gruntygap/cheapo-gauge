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
#include <ACAN2515.h>
#include <LittleFS.h>

// Button Def
#define RESET_PIN D0
// Screen Def
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDRESS 0x3C
#define OLED_SDA D4
#define OLED_SCL D5

// CAN Output config - match these to your remote device settings
#define REMOTE_CAN_ID 2          // This device's CAN ID (set in TunerStudio "Remote CAN ID")
#define MS2_CAN_ID 0             // Megasquirt's CAN ID (usually 0)
#define PORTS_DATA_TABLE 7
#define PORT_1_OFFSET 166
#define PORT_2_OFFSET 167
#define PORT_3_OFFSET 168

// AEM X-Series UEGO / AEMnet config
// "ID 1" on the gauge corresponds to arbitration ID 0x180, "ID 16" -> 0x18F.
#define AEMNET_GAUGE_ID 1
#define AEMNET_BASE_ID 0x180UL
#define AEMNET_ADC_TABLE PORTS_DATA_TABLE
#define AEMNET_ADC0_OFFSET 104
#define AEMNET_ADC1_OFFSET 106
#define AEMNET_ADC2_OFFSET 108
#define AEMNET_ADC3_OFFSET 110

// MS2/Extra CAN protocol - 29-bit extended frames
// ID layout: [var_offset(11)][msg_type(3)][from_id(4)][to_id(4)][var_blk(4)][spare(3)]
#define MSG_CMD 0    // Set variable on remote processor
#define MSG_REQ 1    // Request data from remote processor
#define MSG_RSP 2    // Response with requested data

// State persistence
#define STATE_FILE "/port_state.bin"

static const byte INT_PIN  = D6;
static const byte CS_PIN   = D7;
static const byte SCK_PIN  = D8;
static const byte MISO_PIN = D9;
static const byte MOSI_PIN = D10;

volatile uint32_t irqCount = 0;

ACAN2515 can(CS_PIN, SPI, INT_PIN);

static void onCanInterrupt() {
  irqCount++;
  can.isr();
}

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long buttonPressTime = 0;
unsigned long lastTapTime = 0;
bool buttonPressed = false;
bool waitingForSecondPress = false;

const unsigned long tapThreshold = 200;
const unsigned long longPressThreshold = 500;

// Modes
enum DisplayMode { LAMBDAO2, BOOST, ETHANOL, VOLTAGE, TPS, MATCLT, BAROMAP, EGO, PORT1, PORT2, PORT3 };
#define NUM_MODES 11
DisplayMode currentMode = LAMBDAO2;

// Sensor data
float psi = 0.0, maxPsi = -14.7;
float baro = 0.0, maps = 0.0, mat = 0.0, clt = 0.0;
float ethanolContent = 0.0;
double tps = 0.0;
float batt = 0.0, ego1 = 0.0, ego2 = 0.0;
float lambdaValue = 0.0;
float oxygenPercent = 0.0;
float aemSystemVolts = 0.0;
uint16_t aemLambdaRaw = 0;
int16_t aemOxygenRaw = 0;
uint8_t aemSystemVoltsRaw = 0;
uint16_t aemAdc0 = 0;
uint16_t aemAdc1 = 0;
uint16_t aemAdc2 = 0;
uint16_t aemAdc3 = 0;
bool aemLambdaValid = false;
bool aemSensorFault = false;
unsigned long lastAemLogMs = 0;

// Port output states (persisted to flash)
// USED FOR MAIN FAN TRIGGER
bool port1State = false;
// USED FOR AC FAN TRIGGER
bool port2State = false;
// Table switching
uint8_t port3State = 0xFF;
int port3Cursor = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("Serial connected");

  SPI.setSCK(SCK_PIN);
  SPI.setTX(MOSI_PIN);
  SPI.setRX(MISO_PIN);
  SPI.setCS(CS_PIN);
  SPI.begin();
  pinMode(INT_PIN, INPUT_PULLUP);

  ACAN2515Settings settings(
    16UL * 1000UL * 1000UL,
    500UL * 1000UL
  );

  settings.mRequestedMode = ACAN2515Settings::NormalMode;
  settings.mReceiveBufferSize = 16;
  settings.mTransmitBuffer0Size = 8;
  settings.mTransmitBuffer1Size = 8;
  settings.mTransmitBuffer2Size = 8;



  const uint16_t errorCode = can.begin(settings, onCanInterrupt);

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED not found");
    while (1)
      ;
  }

  display.setTextColor(WHITE);
  display.display();

  if (errorCode == 0) {
    Serial.println("CAN init OK");
  } else {
    Serial.print("CAN init FAIL: 0x");
    Serial.println(errorCode, HEX);
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("CAN Bus Init Failed");
    display.display();
    while (1) {}
  }

  pinMode(RESET_PIN, INPUT_PULLUP);

  loadState();
}

void drawBoost2() {
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

// --- Port Display Functions ---

void drawPort1() {
  drawSingleStat(port1State ? "ON" : "OFF", "", "Fan 1");
}

void drawPort2() {
  drawSingleStat(port2State ? "ON" : "OFF", "", "Fan 2");
}

void drawPort3() {

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

uint32_t aemnetMessageId() {
  return AEMNET_BASE_ID + (uint32_t)(AEMNET_GAUGE_ID - 1);
}

uint16_t clampTo10Bit(long value) {
  if (value < 0) return 0;
  if (value > 1023) return 1023;
  return (uint16_t)value;
}

uint16_t voltsToAdc10(float volts) {
  const float adc = (volts / 5.0f) * 1023.0f;
  return clampTo10Bit((long)(adc + 0.5f));
}

uint16_t lambdaToAdc10(float lambda) {
  // Match the gauge's 0.5V-4.5V linear analog output scaling:
  // 0.58 lambda = 0.5V, 1.23 lambda = 4.5V.
  // 8.50 stoich = 0.5V, 18.0 lambda = 4.5V.
  const float clampedLambda = constrain(lambda, 0.58f, 1.23f);
  const float volts = 0.5f + ((clampedLambda - 0.58f) * (4.0f / 0.65f));
  return voltsToAdc10(volts);
}

uint16_t oxygenToAdc10(float oxygen) {
  // Exhaust oxygen is physically non-negative; clamp to a practical 0-20.9% range
  // and scale that range across a 0-5V-equivalent 10-bit ADC.
  const float clampedOxygen = constrain(oxygen, 0.0f, 20.9f);
  const float volts = (clampedOxygen / 20.9f) * 5.0f;
  return voltsToAdc10(volts);
}

uint8_t wordHighByte(uint16_t value) {
  return (uint8_t)(value >> 8);
}

uint8_t wordLowByte(uint16_t value) {
  return (uint8_t)(value & 0xFF);
}

bool handleAEMnetFrame(const CANMessage &msg) {
  if (!msg.ext || msg.rtr || msg.len < 8) return false;
  if (msg.id != aemnetMessageId()) return false;

  aemLambdaRaw = ((uint16_t)msg.data[0] << 8) | msg.data[1];
  aemOxygenRaw = (int16_t)(((uint16_t)msg.data[2] << 8) | msg.data[3]);
  aemSystemVoltsRaw = msg.data[4];
  lambdaValue = aemLambdaRaw * 0.0001f;
  oxygenPercent = aemOxygenRaw * 0.001f;
  aemSystemVolts = aemSystemVoltsRaw * 0.1f;
  aemLambdaValid = (msg.data[6] & 0x80) != 0;
  aemSensorFault = (msg.data[7] & 0x40) != 0;
  if (aemLambdaValid && !aemSensorFault) {
    aemAdc0 = lambdaToAdc10(lambdaValue);
    aemAdc1 = oxygenToAdc10(oxygenPercent);
    aemAdc2 = voltsToAdc10(aemSystemVolts);
  } else {
    aemAdc0 = 0;
    aemAdc1 = 0;
    aemAdc2 = 0;
  }
  aemAdc3 = 0;

  const unsigned long now = millis();
  if (now - lastAemLogMs >= 10) {
    lastAemLogMs = now;
    Serial.print("AEM lambda=");
    Serial.print(lambdaValue, 4);
    Serial.print(" oxygen=");
    Serial.print(oxygenPercent, 3);
    Serial.print("% volts=");
    Serial.print(aemSystemVolts, 1);
    Serial.print(" adc0=");
    Serial.print(aemAdc0);
    Serial.print(" adc1=");
    Serial.print(aemAdc1);
    Serial.print(" adc2=");
    Serial.print(aemAdc2);
    Serial.print(" valid=");
    Serial.print(aemLambdaValid ? "1" : "0");
    Serial.print(" fault=");
    Serial.println(aemSensorFault ? "1" : "0");
  }

  return true;
}

// Build 29-bit extended CAN ID per MS2/Extra protocol
uint32_t buildMSCANId(uint16_t var_offset, uint8_t msg_type, uint8_t from_id, uint8_t to_id, uint8_t var_blk) {
  uint32_t id = 0;
  id |= ((uint32_t)(var_offset & 0x7FF)) << 18;  // 11 bits at [28:18]
  id |= ((uint32_t)(msg_type & 0x07)) << 15;      // 3 bits at [17:15]
  id |= ((uint32_t)(from_id & 0x0F)) << 11;       // 4 bits at [14:11]
  id |= ((uint32_t)(to_id & 0x0F)) << 7;          // 4 bits at [10:7]
  id |= ((uint32_t)(var_blk & 0x0F)) << 3;        // 4 bits at [6:3]
  return id;
}

bool decodePollRequest(const CANMessage &msg,
                       uint8_t &fromId,
                       uint8_t &toId,
                       uint8_t &table,
                       uint16_t &offset,
                       uint8_t &count) {
  if (!msg.ext) return false;
  // if (msg.len == 0 || msg.len > 8) return false;
  if (msg.len != 3) return false;

  const uint32_t id = msg.id;

  const uint8_t msgType = (id >> 15) & 0x07;
  fromId = (id >> 11) & 0x0F;
  toId   = (id >> 7)  & 0x0F;

  if (msgType != MSG_REQ) return false;
  if (toId != REMOTE_CAN_ID) return false;

  // Request payload format:
  // data[0] = table/page
  // data[1] = offset bits 10:3
  // data[2] = offset bits 2:0 in bits 7:5, count in bits 4:0
  table  = msg.data[0];
  offset = ((uint16_t)msg.data[1] << 3) | (msg.data[2] >> 5);
  count  = msg.data[2] & 0x1F;

  return true;
}

void sendPollResponse(uint8_t requesterId, uint8_t table, uint16_t offset, uint8_t count) {
  CANMessage tx;
  tx.ext = true;
  tx.rtr = false;
  tx.len = count;

  tx.id = buildMSCANId(offset, MSG_RSP, REMOTE_CAN_ID, requesterId, table);

  for (uint8_t i = 0; i < count; i++) {
    uint16_t idx = offset + i;
    uint8_t value = 0x00;

    // Table 7 carries both the ADC block at offsets 104-111 and port states at 166-168.
    if (table == AEMNET_ADC_TABLE) {
      if (idx == AEMNET_ADC0_OFFSET) value = wordHighByte(aemAdc0);
      else if (idx == (AEMNET_ADC0_OFFSET + 1)) value = wordLowByte(aemAdc0);
      else if (idx == AEMNET_ADC1_OFFSET) value = wordHighByte(aemAdc1);
      else if (idx == (AEMNET_ADC1_OFFSET + 1)) value = wordLowByte(aemAdc1);
      else if (idx == AEMNET_ADC2_OFFSET) value = wordHighByte(aemAdc2);
      else if (idx == (AEMNET_ADC2_OFFSET + 1)) value = wordLowByte(aemAdc2);
      else if (idx == AEMNET_ADC3_OFFSET) value = wordHighByte(aemAdc3);
      else if (idx == (AEMNET_ADC3_OFFSET + 1)) value = wordLowByte(aemAdc3);

      else if (idx == PORT_1_OFFSET) value = port1State ? 1 : 0;
      else if (idx == PORT_2_OFFSET) value = port2State ? 1 : 0;
      else if (idx == PORT_3_OFFSET) value = port3State & 0xFF;
    }

    tx.data[i] = value;
  }

  bool ok = can.tryToSend(tx);

  Serial.print("RSP ");
  Serial.print(ok ? "OK " : "FAIL ");
  Serial.print("ID=0x");
  Serial.print(tx.id, HEX);
  Serial.print(" LEN=");
  Serial.print(tx.len);
  Serial.print(" DATA=");
  for (uint8_t i = 0; i < tx.len; i++) {
    if (i) Serial.print(" ");
    if (tx.data[i] < 0x10) Serial.print("0");
    Serial.print(tx.data[i], HEX);
  }
  Serial.println();
}

void updateCAN() {
  CANMessage rx;

  while (can.receive(rx)) {
    // --- DEBUG: print every CAN frame ---
    Serial.print("RX ID=0x");
    Serial.print(rx.id, HEX);
    Serial.print(" EXT=");
    Serial.print(rx.ext);
    Serial.print(" LEN=");
    Serial.print(rx.len);
    Serial.print(" DATA=");
    for (uint8_t i = 0; i < rx.len; i++) {
      if (i) Serial.print(" ");
      if (rx.data[i] < 0x10) Serial.print("0");
      Serial.print(rx.data[i], HEX);
    }
    Serial.println();

    if (handleAEMnetFrame(rx)) {
      continue;
    }

    // ------------------------------------------------------------
    // Handle MS2 poll requests (29-bit extended frames)
    // ------------------------------------------------------------
    if (rx.ext) {
      uint8_t fromId, toId, table, count;
      uint16_t offset;

      if (decodePollRequest(rx, fromId, toId, table, offset, count)) {
        Serial.print("Poll request from ");
        Serial.print(fromId);
        Serial.print(" to ");
        Serial.print(toId);
        Serial.print(" table=");
        Serial.print(table);
        Serial.print(" offset=");
        Serial.print(offset);
        Serial.print(" count=");
        Serial.println(count);

        sendPollResponse(fromId, table, offset, count);
      }

      continue;
    }

    // ------------------------------------------------------------
    // Handle MS2 broadcast data (standard 11-bit frames)
    // ------------------------------------------------------------
    const uint16_t base = 0x5F0;

    if (rx.id == (base + 2) && rx.len >= 8) {
      baro = ((rx.data[0] << 8) | rx.data[1]) / 10.0;
      maps = ((rx.data[2] << 8) | rx.data[3]) / 10.0;
      mat  = ((rx.data[4] << 8) | rx.data[5]) / 10.0;
      clt  = ((rx.data[6] << 8) | rx.data[7]) / 10.0;

      psi = (maps - baro) * 0.145038;
      if (psi > maxPsi) {
        maxPsi = psi;
      }
    }

    if (rx.id == (base + 3) && rx.len >= 8) {
      tps  = (int16_t)((rx.data[0] << 8) | rx.data[1]) / 10.0;
      batt = ((rx.data[2] << 8) | rx.data[3]) / 10.0;
      ego1 = ((rx.data[4] << 8) | rx.data[5]) / 10.0;
      ego2 = ((rx.data[6] << 8) | rx.data[7]) / 10.0;
    }

    if (rx.id == (base + 47) && rx.len >= 2) {
      ethanolContent = ((rx.data[0] << 8) | rx.data[1]) / 10.0;
    }
  }
}

void handleButton() {
  int state = digitalRead(RESET_PIN);
  unsigned long now = millis();

  // Button pressed
  if (state == LOW && !buttonPressed) {
    buttonPressed = true;
    buttonPressTime = now;
  }

  // Button released
  if (state == HIGH && buttonPressed) {
    buttonPressed = false;
    unsigned long duration = now - buttonPressTime;

    if (duration >= longPressThreshold) {
      // Long press
      waitingForSecondPress = false;

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
    } else {
      // Short press: either start waiting for double tap, or consume second tap
      if (waitingForSecondPress && (now - lastTapTime <= tapThreshold)) {
        waitingForSecondPress = false;

        // Double press: previous screen
        if (currentMode == PORT3) {
          port3Cursor = 0;
        }
        currentMode = (DisplayMode)(((int)currentMode - 1 + NUM_MODES) % NUM_MODES);
      } else {
        waitingForSecondPress = true;
        lastTapTime = now;
      }
    }
  }

  // Single tap confirmed after timeout
  if (!buttonPressed && waitingForSecondPress && (now - lastTapTime > tapThreshold)) {
    waitingForSecondPress = false;

    if (currentMode == PORT3 && port3Cursor < 7) {
      port3Cursor++;
    } else {
      if (currentMode == PORT3) {
        port3Cursor = 0;
      }
      currentMode = (DisplayMode)(((int)currentMode + 1) % NUM_MODES);
    }
  }
}

void loop() {
  display.clearDisplay();
  updateCAN();
  handleButton();

  switch (currentMode) {
    case LAMBDAO2:
      char lambdaBuf[8];
      snprintf(lambdaBuf, sizeof(lambdaBuf), "%.3f", lambdaValue);
      char oxygenBuf[8];
      snprintf(oxygenBuf, sizeof(oxygenBuf), "%.2f", oxygenPercent);
      drawSensor2(lambdaBuf, "", "Lam", oxygenBuf, "%", "O2");
      break;
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
}
