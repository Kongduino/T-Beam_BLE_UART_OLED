#include <string> // std::string, std::to_string
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#include <Arduino.h>
#include <OneBitDisplay.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <axp20x.h>

TinyGPSPlus gps;
HardwareSerial GPS(1);
AXP20X_Class axp;

OBDISP obd;
#define SDA_PIN 21
#define SCL_PIN 22
// no reset pin needed
#define RESET_PIN -1
// let OneBitDisplay find the address of our display
#define OLED_ADDR 0x3c
#define FLIP180 0
#define INVERT 0
// Use the default Wire library
#define USE_HW_I2C 1

#include "HW_AES.h"

bool needDecoding = true;
bool needJSON = true;
unsigned char key[32];
uint16_t pingCount = 0; // used by the sendPing function
unsigned char decBuf[256];
unsigned char buff[256];

BLECharacteristic *TXChar = nullptr;
BLECharacteristic *RXChar = nullptr;
bool deviceConnected = false, hasNewLoc = false;

#define REG_OCP 0x0B
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0C
#define REG_OP_MODE 0x01
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_MODEM_CONFIG_3 0x26
#define REG_PA_DAC 0x4D
#define PA_DAC_HIGH 0x87
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define SS      18
#define RST     14
#define DI0     26
#define BAND    431E6
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
//#define myUUID "25e9a831-2042-4307-a993-5ad11a89b7ba"
#define myUUID "25e9-a831-2042-4307"
#define PING_DELAY 30
#define FRAME_INTERVAL 5
#define FRAME_COUNT 3
#define IS_LAT true
#define IS_LNG false

uint8_t frameCounter = 0;
unsigned long t0, t1, t2;
// t0 is for PING_DELAY
// t1 is for FRAME_INTERVAL

char txString[23] = {0};
float newLat, newLng;

String getdms(double ang, bool isLat = true) {
  bool neg(false);
  if (ang < 0.0) {
    neg = true;
    ang = -ang;
  }
  int deg = (int)ang;
  double frac = ang - (double)deg;
  frac *= 60.0;
  int min = (int)frac;
  frac = frac - (double)min;
  double sec = nearbyint(frac * 600000.0);
  sec /= 10000.0;
  if (sec >= 60.0) {
    min++;
    sec -= 60.0;
  }
  String oss;
  if (neg) oss = "-";
  oss += String(deg) + "d " + String(min) + "' " + String(sec) + "\"";
  if (isLat) {
    if (neg) oss += "S";
    else oss += "N";
  } else {
    if (neg) oss += "W";
    else oss += "E";
  }
  return oss;
}

void sendNumber(uint8_t min, uint8_t max, char cmd) {
  uint8_t a = random(min, max), b = random(0, 99);
  float c = ((float)a + (float)b / 100.0);
  Serial.printf("a = %d, b = %d, c = %.2f\n", a, b, c);
  sprintf(txString, "%.2f", float(a + b / 100.0));
  uint16_t d = a * 100 + b;
  a = d / 255;
  b = d - (a * 255);
  memset(txString, 0, 23);
  txString[0] = cmd;
  txString[1] = b;
  txString[2] = a;
  TXChar->setValue(txString);
  //  TXChar->writeValue(txString); // Tobozo
  TXChar->notify();
  Serial.printf("Sent TX notification : %s\n", txString);
}

void sendTemp() {
  sendNumber(25, 29, 'T');
}

void sendHum() {
  // Fake Humidity.
  sendNumber(45, 65, 'H');
}

class UARTServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
      // should stop advertising when a peer is connected
      pServer->getAdvertising()->stop();
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
      // should resume advertising when peer disconnects
      pServer->getAdvertising()->start();
    }
};

class RXCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
      const char* rxValue = pChar->getValue().c_str();
      uint16_t len = strlen(rxValue);
      if (len > 0) {
        Serial.println("*********");
        esp_aes_hw_hexDump((unsigned char *)rxValue, len);
        obdFill(&obd, 0, 1);
        obdWriteString(&obd, 0, 0, 3, (char *)rxValue, FONT_NORMAL, 0, 1);
        if (strcmp(rxValue, "T?") == 0) {
          sendTemp();
        }
        Serial.println("Sending txString to LoRa:\n");
        esp_aes_hw_hexDump((unsigned char *)txString, 256);
        LoRa.beginPacket();
        LoRa.print(rxValue);
        LoRa.endPacket();
        Serial.println();
        Serial.println("*********");
      }
    }
};

void setup() {
  t0 = millis();
  t2 = millis();
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n\nStart...");
  int rc;
  rc = obdI2CInit(&obd, OLED_128x64, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); // Standard HW I2C bus at 400Khz
  if (rc == OLED_NOT_FOUND) {
    Serial.println("No OLED!");
    while (true) {
      ;
    }
  }
  char *msgs[] = {
    (char *)"SSD1306 @ 0x3C",
    (char *)"SSD1306 @ 0x3D",
    (char *)"SH1106 @ 0x3C",
    (char *)"SH1106 @ 0x3D"
  };
  obdFill(&obd, 0, 1);
  obdWriteString(&obd, 0, 0, 0, msgs[rc], FONT_NORMAL, 0, 1);
  delay(3000);
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    obdWriteString(&obd, 0, 0, 6, "LoRa Failed!", FONT_NORMAL, 0, 1);
    while (1);
  }
  // BW = 6: 62.5 kHz, CR = 1: 4/5, HM = 0
  uint8_t reg1 = 0x62;
  // SF = 12: 12, CRC = 1
  uint8_t reg2 = 0xC4;
  // LDRO = 1, AGCAutoOn = 0-->LNA gain set by register LnaGain
  uint8_t reg3 = 0x08;
  // 7:5 LnaGain 001 -> G1 = maximum gain
  // 1:0 LnaBoostHf 11 -> Boost on, 150% LNA current
  uint8_t reglna = 0b00100011;

  LoRa.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  delay(10);
  LoRa.writeRegister(REG_PA_CONFIG, 0xFF);
  LoRa.writeRegister(REG_MODEM_CONFIG_1, reg1);
  LoRa.writeRegister(REG_MODEM_CONFIG_2, reg2);
  LoRa.writeRegister(REG_MODEM_CONFIG_3, reg3);
  LoRa.writeRegister(REG_LNA, reglna);
  LoRa.writeRegister(REG_PA_DAC, PA_DAC_HIGH); // That's for the transceiver
  LoRa.writeRegister(REG_OCP, 0b00111111);
  delay(10);
  LoRa.writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  delay(10);

  Serial.println("LoRa Initialization OK!");
  obdWriteString(&obd, 0, 0, 6, "LoRa[o]", FONT_SMALL, 0, 1);
  Serial.println("\n Setting up Bluetooth...");
  // Create the BLE Device
  BLEDevice::setMTU(64);
  BLEDevice::init("LoRa UART Service");
  // Create the BLE Server
  BLEServer *UARTServer = BLEDevice::createServer();
  UARTServer->setCallbacks(new UARTServerCallbacks());
  // Create the BLE Service
  BLEService *UARTService = UARTServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  TXChar = UARTService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  //  TXChar = UARTService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  TXChar->addDescriptor(new BLE2902());
  TXChar->setNotifyProperty(true);
  TXChar->setReadProperty(true);
  RXChar = UARTService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  RXChar->setCallbacks(new RXCallbacks());
  // Start the service
  UARTService->start();
  // Start advertising
  UARTServer->getAdvertising()->start();
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(23);
  if (local_mtu_ret) {
    Serial.println("set local  MTU failed, error code = " + String(local_mtu_ret));
  }

  /*
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read it in your phone!");
  */
  obdWriteString(&obd, 0, 48, 6, "BLE[o]", FONT_SMALL, 0, 1);
  Serial.println("Waiting a client connection to notify...");
  if (needDecoding) {
    memcpy(key, "YELLOW SUBMARINEENIRAMBUS WOLLEY", 32);
    Serial.println("Setting up AES key");
  }
  if (axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin FAIL");
    obdWriteString(&obd, 0, 96, 7, "GPS[x]", FONT_SMALL, 0, 1);
    while (1) ;
  }
  Serial.println("AXP192 Begin PASS");
  obdWriteString(&obd, 0, 88, 7, "GPS[o]", FONT_SMALL, 0, 1);
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  GPS.begin(9600, SERIAL_8N1, 34, 12); //17-TX 18-RX
  hasNewLoc = false;
}

uint8_t which = 0;
void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // try to parse packet
    // received a packet
    Serial.println("Received packet:");
    obdFill(&obd, 0, 1);
    obdWriteString(&obd, 0, 0, 5, "Packet!", FONT_NORMAL, 1, 1);
    // read packet
    uint8_t ix = 0;
    while (LoRa.available()) {
      char c = LoRa.read();
      buff[ix++] = c;
    }
    // Let's flip a switch
    if (needDecoding) {
      Serial.println("Decoding packet");
      uint16_t len = ix;
      if (len % 16 > 0) {
        if (len < 16) len = 16;
        else len += 16 - (len % 16);
      }
      memcpy(decBuf, buff, len);
      esp_aes_hw_hexDump((unsigned char*)decBuf, len);
      uint8_t rslt = esp_aes_hw_multiple_blocks(ESP_AES_DECRYPT, key, decBuf, buff, len);
    }
    // Add a string terminator
    buff[ix++] = '\0';
    esp_aes_hw_hexDump((unsigned char*)buff, ix);
    StaticJsonBuffer<256> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(buff);
    if (!root.success()) {
      Serial.println("parseObject() failed");
      obdWriteString(&obd, 0, 0, 6, "Parse Fail", FONT_NORMAL, 1, 1);
      return;
    }
    /*
      {
        "from":E22230T22S",
        "to": "*",
        "msg": "PING",
        "sendCount": 0
      }
    */
    const char* from = root["from"];
    const char* to = root["to"];
    const char* msg = root["msg"];
    const char* sendCount = root["sendCount"];
    // Print values.
    Serial.println(from);
    Serial.println(msg);
    // print RSSI of packet
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());
    obdWriteString(&obd, 0, 0, 0, "cnt: ", FONT_LARGE, 0, 1);
    obdWriteString(&obd, 0, 64, 0, (char*)sendCount, FONT_LARGE, 0, 1);
    obdWriteString(&obd, 0, 0, 4, "rssi", FONT_LARGE, 0, 1);
    obdWriteString(&obd, 0, 66, 4, (char*)String(LoRa.packetRssi()).c_str(), FONT_LARGE, 0, 1);
  }
  if (deviceConnected) {
    t1 = millis();
    if (t1 > t0 + 10000) {
      Serial.print("which = "); Serial.println(which);
      if (which == 0)  sendTemp();
      else sendHum();
      which = 255 - which;
      t0 = millis();
    }
  }
  t1 = millis();
  if (t1 > t2 + 10000) {
    Serial.print("Latitude  : ");
    Serial.println(gps.location.lat(), 5);
    newLat = gps.location.lat();
    String stringLat = getdms(newLat, IS_LAT);
    Serial.print("Longitude : ");
    Serial.println(gps.location.lng(), 4);
    newLng = gps.location.lng();
    String stringLng = getdms(newLng, IS_LNG);
    Serial.println(stringLat + ", " + stringLng);
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.print("Altitude  : ");
    Serial.print(gps.altitude.feet() / 3.2808);
    Serial.println("M");
    Serial.print("Time      : ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
    Serial.print("Speed     : ");
    Serial.println(gps.speed.kmph());
    Serial.println("**********************");
    obdWriteString(&obd, 0, 0, 8, "gps[o]", FONT_SMALL, 0, 1);
    smartDelay(1000);
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
      obdWriteString(&obd, 0, 88, 6, "GPS[x]", FONT_SMALL, 0, 1);
    }
    if ((gps.location.lat() > 0.0)) {
      hasNewLoc = true;
      obdFill(&obd, 0, 1);
      obdWriteString(&obd, 0, 0, 0, (char*)String(gps.location.lat(), 4).c_str(), FONT_LARGE, 0, 1);
      obdWriteString(&obd, 0, 0, 4, (char*)String(gps.location.lng(), 4).c_str(), FONT_LARGE, 0, 1);
      obdWriteString(&obd, 0, 0, 7, "Loc[o]", FONT_SMALL, 0, 1);
      delay(2000);
    } else {
      obdWriteString(&obd, 0, 0, 7, "Loc [x]", FONT_SMALL, 0, 1);
    }
    t2 = millis();
  }
  if (hasNewLoc) {
    hasNewLoc = false;
    memset(txString, 0, 23);
    String X = 'L' + String(newLat, 5) + "," + String(newLng, 5);
    X.toCharArray(txString, X.length() + 1);
    TXChar->setValue(txString);
    // TXChar->writeValue(txString); // Tobozo
    TXChar->notify();
    Serial.println("Sent Location notification:");
    esp_aes_hw_hexDump((unsigned char *)txString, 23);
  }
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (GPS.available()) gps.encode(GPS.read());
  } while (millis() - start < ms);
}
