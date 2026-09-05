#include <Arduino.h>

// ---- DWIN DGUS Display Verkabelung ----
// ESP32-TX (IO22) -> Display-RXD, ESP32-RX (IO21) -> Display-TXD, GND <-> GND.
constexpr int DWIN_TX_PIN = 22;
constexpr int DWIN_RX_PIN = 21;
constexpr uint32_t DWIN_BAUD = 115200; // muss mit "Baud Rate" im DGUS System Setting uebereinstimmen

// Im DGUS-Projekt (System Configuration) ist "CRC: On" aktiviert -> jeder Frame
// braucht 2 CRC-Bytes (CRC-16/MODBUS) am Ende, sonst verwirft das Display die Daten.
constexpr bool DWIN_CRC_ENABLED = true;

HardwareSerial dwinSerial(2); // ESP32 UART2

uint16_t crc16Modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// DGUS-Rahmen (Kommando 0x82) tragen die Gesamtlaenge in einem einzigen Byte
// (max. 255). Nach Abzug von Kommando(1)+VP(2)+CRC(2) bleiben max. 250 Byte
// Nutzdaten pro Frame. Felder wie MessageText/WlanInstructionsText (256 B)
// passen daher nicht in einen einzigen Schreib-Frame und werden hier in
// Chunks à DWIN_MAX_DATA_CHUNK Byte aufgeteilt, jeder Chunk an die passend
// hochgezaehlte VP-Adresse (Wortoffset = Byteoffset/2).
constexpr size_t DWIN_MAX_DATA_CHUNK = 200;

void dwinWriteVP(uint16_t vp, const uint8_t* data, size_t dataLen) {
  size_t offset = 0;
  while (offset < dataLen) {
    size_t chunkLen = dataLen - offset;
    if (chunkLen > DWIN_MAX_DATA_CHUNK) chunkLen = DWIN_MAX_DATA_CHUNK;

    uint8_t payload[DWIN_MAX_DATA_CHUNK + 5];
    size_t p = 0;
    uint16_t chunkVp = vp + (offset / 2);
    payload[p++] = 0x82;
    payload[p++] = chunkVp >> 8;
    payload[p++] = chunkVp & 0xFF;
    memcpy(payload + p, data + offset, chunkLen);
    p += chunkLen;

    size_t frameLen = p;
    if (DWIN_CRC_ENABLED) {
      uint16_t crc = crc16Modbus(payload, p);
      payload[p++] = crc & 0xFF;        // CRC low byte zuerst (Modbus-Konvention)
      payload[p++] = (crc >> 8) & 0xFF;
      frameLen = p;
    }

    uint8_t frame[DWIN_MAX_DATA_CHUNK + 8];
    size_t idx = 0;
    frame[idx++] = 0x5A;
    frame[idx++] = 0xA5;
    frame[idx++] = frameLen;
    memcpy(frame + idx, payload, frameLen);
    idx += frameLen;

    dwinSerial.write(frame, idx);
    offset += chunkLen;
  }
}

// Schreibt einen ASCII-String in eine DGUS "Text Display"-Control.
// Der Text wird bis fieldLen mit Leerzeichen aufgefuellt, damit alter, laengerer
// Text vollstaendig ueberschrieben wird, und bei Bedarf auf gerade Byteanzahl ergaenzt.
void dwinSendText(uint16_t vp, const char* text, uint16_t fieldLen) {
  static uint8_t data[256];
  if (fieldLen > sizeof(data)) fieldLen = sizeof(data);

  size_t len = strnlen(text, fieldLen);
  memcpy(data, text, len);
  for (size_t i = len; i < fieldLen; i++) data[i] = ' ';

  size_t dataLen = fieldLen;
  if (dataLen % 2 != 0 && dataLen < sizeof(data)) data[dataLen++] = ' ';

  dwinWriteVP(vp, data, dataLen);
}

// Schreibt einen UTF-8-String in ein DGUS "Text Display"-Control mit
// Encoding Mode 0x05=UNICODE. Dekodiert UTF-8 nach UCS-2 (reicht fuer
// Tuerkisch/Latin Extended-A, keine Surrogate-Paare noetig) und sendet jedes
// Zeichen als 2 Byte little-endian (Low-Byte zuerst, UTF-16LE). Das ist die
// Byte-Reihenfolge, die auf diesem Display bereits getestet/bestaetigt wurde
// (siehe dwinReadVP-Diagnose) - der VP-Adressplan spricht an anderer Stelle
// von "UTF-16BE", das ist nur eine ungenaue Doku-Angabe, kein Widerspruch,
// der hier aufgeloest werden muesste. Falls ein neues Feld auf dem Display
// spiegelverkehrt/falsch erscheint: dwinReadVP() gegen die VP-Adresse nutzen
// und Byte-Reihenfolge im Rueckgabe-Dump pruefen.
void dwinSendTextUnicode(uint16_t vp, const char* utf8Text, uint16_t charCount) {
  static uint8_t data[256];
  if ((size_t)charCount * 2 > sizeof(data)) charCount = sizeof(data) / 2;

  const uint8_t* s = (const uint8_t*)utf8Text;
  size_t o = 0;
  uint16_t chars = 0;

  while (*s && chars < charCount) {
    uint32_t cp;
    uint8_t b0 = *s;
    if (b0 < 0x80) {
      cp = b0; s += 1;
    } else if ((b0 & 0xE0) == 0xC0 && s[1]) {
      cp = ((b0 & 0x1F) << 6) | (s[1] & 0x3F); s += 2;
    } else if ((b0 & 0xF0) == 0xE0 && s[1] && s[2]) {
      cp = ((b0 & 0x0F) << 12) | ((s[1] & 0x3F) << 6) | (s[2] & 0x3F); s += 3;
    } else {
      cp = '?'; s += 1;
    }
    data[o++] = cp & 0xFF;         // Low-Byte zuerst (UTF-16LE-Konvention)
    data[o++] = (cp >> 8) & 0xFF;
    chars++;
  }
  while (chars < charCount) {
    data[o++] = 0x20; // Leerzeichen
    data[o++] = 0x00;
    chars++;
  }

  dwinWriteVP(vp, data, o);
}

// Liest VP zurueck und dumpt die rohen Bytes zur Diagnose (z.B. Byte-Reihenfolge
// bei Unicode-Feldern pruefen).
void dwinReadVP(uint16_t vp, uint8_t wordCount) {
  uint8_t payload[6];
  size_t p = 0;
  payload[p++] = 0x83;
  payload[p++] = vp >> 8;
  payload[p++] = vp & 0xFF;
  payload[p++] = wordCount;

  size_t frameLen = p;
  if (DWIN_CRC_ENABLED) {
    uint16_t crc = crc16Modbus(payload, p);
    payload[p++] = crc & 0xFF;
    payload[p++] = (crc >> 8) & 0xFF;
    frameLen = p;
  }

  uint8_t frame[3 + 6];
  size_t idx = 0;
  frame[idx++] = 0x5A;
  frame[idx++] = 0xA5;
  frame[idx++] = frameLen;
  memcpy(frame + idx, payload, frameLen);
  idx += frameLen;

  while (dwinSerial.available()) dwinSerial.read();
  dwinSerial.write(frame, idx);

  unsigned long start = millis();
  uint8_t resp[64];
  size_t respLen = 0;
  while (millis() - start < 300 && respLen < sizeof(resp)) {
    if (dwinSerial.available()) resp[respLen++] = dwinSerial.read();
  }

  Serial.print("VP 0x");
  Serial.print(vp, HEX);
  Serial.print(" READ (");
  Serial.print(respLen);
  Serial.print(" bytes): ");
  for (size_t i = 0; i < respLen; i++) {
    if (resp[i] < 0x10) Serial.print('0');
    Serial.print(resp[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// ================================================================
// VP-Adressplan Seite 1-7 (siehe VP-Adressplan-Dokument, 2026-08-30)
// ================================================================

// ---- Seite 1: Uhr/Gebetszeiten (0x1000-0x11FF) ----
constexpr uint16_t VP_IMSAK           = 0x1000; // 16 B ASCII
constexpr uint16_t VP_GUNES           = 0x1010; // 16 B ASCII (Sonnenaufgang)
constexpr uint16_t VP_OGLE            = 0x1020; // 16 B ASCII (Mittag)
constexpr uint16_t VP_IKINDI          = 0x1030; // 16 B ASCII (Nachmittag)
constexpr uint16_t VP_AKSAM           = 0x1040; // 16 B ASCII (Abend/Maghrib)
constexpr uint16_t VP_YATSI           = 0x1050; // 16 B ASCII (Nacht)
constexpr uint16_t VP_COUNTDOWN_VAKIT = 0x1060; // 16 B ASCII
constexpr uint16_t VP_TIME            = 0x1070; // 16 B ASCII (wiederverwendet auf Seite 2/5/7)
constexpr uint16_t VP_DAY_S           = 0x1080; //  8 B ASCII
constexpr uint16_t VP_MONTH_YEAR      = 0x1090; // 24 B Unicode
constexpr uint16_t VP_WEEK_DAY        = 0x10B0; // 24 B Unicode
constexpr uint16_t VP_PLACE           = 0x10D0; // 64 B Unicode
constexpr uint16_t VP_HICRI           = 0x1110; // 64 B Unicode

// ---- Seite 2: Meldungen (0x1200-0x13FF) ----
constexpr uint16_t VP_LABEL_MESSAGE = 0x1200; // 20 B Unicode
constexpr uint16_t VP_MESSAGE_TEXT  = 0x1220; // 256 B Unicode

// ---- Seite 3: Audio-Player (0x1400-0x15FF) ----
constexpr uint16_t VP_AUDIO_TITLE_ONE   = 0x1400; // 30 B ASCII
constexpr uint16_t VP_AUDIO_TITLE_TWO   = 0x1420; // 30 B ASCII
constexpr uint16_t VP_AUDIO_SPEAKER     = 0x1440; // 64 B Unicode
constexpr uint16_t VP_AUDIO_DESCRIPTION = 0x1480; // 128 B Unicode
constexpr uint16_t VP_DURATION_MINUS    = 0x1500; //  8 B ASCII
constexpr uint16_t VP_AUDIO_PROGRESS    = 0x1510; // 1 Wort, Icon/Balken - KEIN Text, wird nicht beschrieben
constexpr uint16_t VP_DURATION_PLUS     = 0x1520; //  8 B ASCII

// ---- Seite 4: Systeminfo (0x1600-0x17FF) ----
constexpr uint16_t VP_MQTT_STATUS       = 0x1600; // 16 B ASCII ("connected"/"not connected")
constexpr uint16_t VP_IP_ADDRESS        = 0x1610; // 16 B ASCII (auf Seite 6 wiederverwendet)
constexpr uint16_t VP_WIFI_SSID         = 0x1620; // 32 B ASCII
constexpr uint16_t VP_WIFI_RSSI_DBM     = 0x1640; // 16 B ASCII
constexpr uint16_t VP_MQTT_BROKER_HOST  = 0x1650; // 48 B ASCII
constexpr uint16_t VP_FIRMWARE_VERSION  = 0x1680; // 32 B ASCII
constexpr uint16_t VP_UPTIME            = 0x16A0; // 16 B ASCII
constexpr uint16_t VP_DEVICE_ID         = 0x16B0; // 40 B ASCII
constexpr uint16_t VP_LABEL_TITLE       = 0x16E0; // 16 B ASCII "SYSTEMINFO"
constexpr uint16_t VP_LABEL_WLAN        = 0x1700; // 12 B ASCII "WLAN"
constexpr uint16_t VP_LABEL_RSSI        = 0x1720; //  8 B ASCII "RSSI"
constexpr uint16_t VP_LABEL_IP          = 0x1740; //  8 B ASCII "IP"
constexpr uint16_t VP_LABEL_MQTT        = 0x1760; //  8 B ASCII "MQTT"
constexpr uint16_t VP_LABEL_BROKER      = 0x1780; // 16 B ASCII "Broker"
constexpr uint16_t VP_LABEL_FIRMWARE    = 0x17A0; // 16 B ASCII "Firmware"
constexpr uint16_t VP_LABEL_UPTIME      = 0x17C0; // 12 B ASCII "Laufzeit"
constexpr uint16_t VP_LABEL_DEVICE_ID   = 0x17E0; // 16 B ASCII "Geraete-ID"

// ---- Seite 5: Fehler-/OTA-Update-Status (0x1800-0x19FF) ----
constexpr uint16_t VP_LABEL_OTA_STATUS     = 0x1800; //  20 B Unicode "STATUS"
constexpr uint16_t VP_OTA_STATUS_TEXT      = 0x1820; // 128 B Unicode
constexpr uint16_t VP_OTA_PROGRESS_PERCENT = 0x18A0; //  16 B ASCII
constexpr uint16_t VP_OTA_PROGRESS_BAR     = 0x18B0; // 1 Wort, Icon/Balken - KEIN Text, wird nicht beschrieben
constexpr uint16_t VP_OTA_STATUS_ICON      = 0x18C0; // 1 Wort, Icon - KEIN Text, wird nicht beschrieben

// ---- Seite 6: WLAN-Ersteinrichtung (0x1A00-0x1BFF) ----
constexpr uint16_t VP_LABEL_WLAN_TITLE          = 0x1A00; //  20 B Unicode "WLAN-EINRICHTUNG"
constexpr uint16_t VP_LABEL_WLAN_AP_NAME        = 0x1A20; //  20 B Unicode "WLAN-NAME"
constexpr uint16_t VP_AP_SSID                   = 0x1A40; //  40 B ASCII
constexpr uint16_t VP_WLAN_INSTRUCTIONS_TEXT    = 0x1A70; // 256 B Unicode
constexpr uint16_t VP_PROVISIONING_STATUS_ICON  = 0x1B70; // 1 Wort, Icon - KEIN Text, wird nicht beschrieben
constexpr uint16_t VP_PROVISIONING_STATUS_TEXT  = 0x1B80; //  32 B Unicode

// ---- Seite 7: Ramadan/Sonderzeiten (0x1C00-0x1DFF) ----
constexpr uint16_t VP_LABEL_RAMADAN_TITLE   = 0x1C00; // 20 B Unicode "RAMADAN"
constexpr uint16_t VP_RAMADAN_DAY_TEXT      = 0x1C20; // 16 B ASCII
constexpr uint16_t VP_LABEL_COUNTDOWN_IFTAR = 0x1C50; // 20 B Unicode "BIS IFTAR"
constexpr uint16_t VP_COUNTDOWN_IFTAR       = 0x1C70; // 14 B ASCII
constexpr uint16_t VP_LABEL_SAHUR           = 0x1C80; // 20 B Unicode "SAHUR"
constexpr uint16_t VP_LABEL_IFTAR           = 0x1CA0; // 20 B Unicode "IFTAR"
constexpr uint16_t VP_RAMADAN_GREETING_TEXT = 0x1CC0; // 64 B Unicode

// ================================================================
// Statische Labels: einmal in setup() geschrieben, aendern sich nicht.
// ================================================================
struct StaticLabel {
  uint16_t vp;
  uint16_t len;
  bool unicode;
  const char* text;
};

const StaticLabel STATIC_LABELS[] = {
  { VP_LABEL_MESSAGE,        20, true,  "MELDUNG" },
  { VP_LABEL_TITLE,          16, false, "SYSTEMINFO" },
  { VP_LABEL_WLAN,           12, false, "WLAN" },
  { VP_LABEL_RSSI,            8, false, "RSSI" },
  { VP_LABEL_IP,              8, false, "IP" },
  { VP_LABEL_MQTT,            8, false, "MQTT" },
  { VP_LABEL_BROKER,         16, false, "Broker" },
  { VP_LABEL_FIRMWARE,       16, false, "Firmware" },
  { VP_LABEL_UPTIME,         12, false, "Laufzeit" },
  { VP_LABEL_DEVICE_ID,      16, false, "Geraete-ID" },
  { VP_LABEL_OTA_STATUS,     20, true,  "STATUS" },
  // Laenge auf 32 B angehoben: "WLAN-EINRICHTUNG" hat 16 Zeichen, die im
  // Plan angegebenen 20 B (=10 Zeichen Unicode) waeren zu kurz gewesen;
  // die Adressluecke zu LabelWlanApName (0x1A20) ist 64 B breit, also passt
  // das ohne Kollision.
  { VP_LABEL_WLAN_TITLE,     32, true,  "WLAN-EINRICHTUNG" },
  { VP_LABEL_WLAN_AP_NAME,   20, true,  "WLAN-NAME" },
  { VP_LABEL_RAMADAN_TITLE,  20, true,  "RAMADAN" },
  { VP_LABEL_COUNTDOWN_IFTAR,20, true,  "BIS IFTAR" },
  { VP_LABEL_SAHUR,          20, true,  "SAHUR" },
  { VP_LABEL_IFTAR,          20, true,  "IFTAR" },
};

void sendStaticLabels() {
  for (const StaticLabel& l : STATIC_LABELS) {
    if (l.unicode) {
      dwinSendTextUnicode(l.vp, l.text, l.len / 2);
    } else {
      dwinSendText(l.vp, l.text, l.len);
    }
  }
}

// ================================================================
// Dummy-Werte: rotieren durch ROTATE_COUNT Beispielsaetze, damit sich jedes
// Feld auf dem Display sichtbar aendert und die VP-Verdrahtung geprueft
// werden kann. Icon-/Balken-Felder (1 Wort, kein Text) sind hier absichtlich
// NICHT enthalten.
// ================================================================
constexpr uint8_t ROTATE_COUNT = 3;

struct RotatingField {
  uint16_t vp;
  uint16_t len;
  bool unicode;
  const char* values[ROTATE_COUNT];
};

const RotatingField ROTATING_FIELDS[] = {
  // Seite 1
  { VP_IMSAK,           16, false, { "05:12", "05:13", "05:14" } },
  { VP_GUNES,           16, false, { "06:45", "06:46", "06:47" } },
  { VP_OGLE,            16, false, { "13:05", "13:06", "13:07" } },
  { VP_IKINDI,          16, false, { "16:40", "16:41", "16:42" } },
  { VP_AKSAM,           16, false, { "19:55", "19:56", "19:57" } },
  { VP_YATSI,           16, false, { "21:20", "21:21", "21:22" } },
  { VP_COUNTDOWN_VAKIT, 16, false, { "00:12:34", "00:12:33", "00:12:32" } },
  { VP_TIME,            16, false, { "12:00:00", "12:00:01", "12:00:02" } },
  { VP_DAY_S,            8, false, { "05", "06", "07" } },
  { VP_MONTH_YEAR,      24, true,  { "Eylul 2026", "Ekim 2026", "Kasim 2026" } },
  { VP_WEEK_DAY,        24, true,  { "Cumartesi", "Pazar", "Pazartesi" } },
  { VP_PLACE,           64, true,  { "Istanbul, Turkiye", "Ankara, Turkiye", "Izmir, Turkiye" } },
  { VP_HICRI,           64, true,  { "12 Rebiulevvel 1448", "13 Rebiulevvel 1448", "14 Rebiulevvel 1448" } },

  // Seite 2
  { VP_MESSAGE_TEXT, 256, true, { "Test Meldung Eins", "Test Meldung Zwei", "Test Meldung Drei - Sonderzeichen: sgioc" } },

  // Seite 3
  { VP_AUDIO_TITLE_ONE,   30, false, { "Test Titel Eins", "Test Titel Zwei", "Test Titel Drei" } },
  { VP_AUDIO_TITLE_TWO,   30, false, { "Kategorie A", "Kategorie B", "Kategorie C" } },
  { VP_AUDIO_SPEAKER,     64, true,  { "Sprecher: Testname Eins", "Sprecher: Testname Zwei", "Sprecher: Testname Drei" } },
  { VP_AUDIO_DESCRIPTION,128, true,  { "Test-Beschreibung Nummer eins fuer den Audio-Player.", "Test-Beschreibung Nummer zwei fuer den Audio-Player.", "Test-Beschreibung Nummer drei fuer den Audio-Player." } },
  { VP_DURATION_MINUS,     8, false, { "00:15", "00:30", "00:45" } },
  { VP_DURATION_PLUS,      8, false, { "03:45", "03:30", "03:15" } },

  // Seite 4
  { VP_WIFI_SSID,        32, false, { "Testnetz_1", "Testnetz_2", "Testnetz_3" } },
  { VP_WIFI_RSSI_DBM,    16, false, { "-55 dBm", "-67 dBm", "-72 dBm" } },
  { VP_IP_ADDRESS,       16, false, { "192.168.1.42", "10.0.0.5", "172.16.0.9" } },
  { VP_MQTT_STATUS,      16, false, { "connected", "not connected", "connected" } },
  { VP_MQTT_BROKER_HOST, 48, false, { "broker.example.com", "192.168.1.10", "mqtt.local" } },
  { VP_FIRMWARE_VERSION, 32, false, { "v1.0.0-test1", "v1.0.0-test2", "v1.0.0-test3" } },
  { VP_UPTIME,           16, false, { "0d 00:01:00", "0d 00:02:00", "0d 00:03:00" } },
  { VP_DEVICE_ID,        40, false, { "ESP32-TEST-0001", "ESP32-TEST-0002", "ESP32-TEST-0003" } },

  // Seite 5
  { VP_OTA_STATUS_TEXT,     128, true,  { "Update wird geladen...", "Installation laeuft...", "Update abgeschlossen" } },
  { VP_OTA_PROGRESS_PERCENT, 16, false, { "10 %", "55 %", "100 %" } },

  // Seite 6
  { VP_AP_SSID,                 40, false, { "MAARIF-SETUP-01", "MAARIF-SETUP-02", "MAARIF-SETUP-03" } },
  { VP_WLAN_INSTRUCTIONS_TEXT, 256, true,  { "Bitte mit dem WLAN MAARIF-SETUP verbinden.", "Danach im Browser 192.168.4.1 oeffnen.", "Heimnetz auswaehlen und Passwort eingeben." } },
  { VP_PROVISIONING_STATUS_TEXT, 32, true, { "Warte auf WLAN", "Verbindet...", "Verbunden" } },

  // Seite 7
  { VP_RAMADAN_DAY_TEXT,      16, false, { "TAG 1 / 30", "TAG 15 / 30", "TAG 30 / 30" } },
  { VP_COUNTDOWN_IFTAR,       14, false, { "05:12:33", "03:00:00", "00:00:01" } },
  { VP_RAMADAN_GREETING_TEXT, 64, true,  { "Ramazan Mubarek Olsun", "Hayirli Ramazanlar", "Iyi Ramazanlar" } },
};

void sendRotatingFields(uint8_t rotationIndex) {
  for (const RotatingField& f : ROTATING_FIELDS) {
    const char* value = f.values[rotationIndex % ROTATE_COUNT];
    if (f.unicode) {
      dwinSendTextUnicode(f.vp, value, f.len / 2);
    } else {
      dwinSendText(f.vp, value, f.len);
    }
  }
}

constexpr unsigned long ROTATE_INTERVAL_MS = 3000;
unsigned long lastRotateAt = 0;
uint8_t rotateIndex = 0;

void setup() {
  Serial.begin(115200);
  dwinSerial.begin(DWIN_BAUD, SERIAL_8N1, DWIN_RX_PIN, DWIN_TX_PIN);
  delay(3000); // grosszuegige Boot-Zeit fuer das Display (SD-Karten-Ladevorgang)

  sendStaticLabels();
  sendRotatingFields(rotateIndex);
  lastRotateAt = millis();

  Serial.println("Dummy-Daten Rotation 0 gesendet (Seite 1-7, ohne Icons).");
}

void loop() {
  if (millis() - lastRotateAt >= ROTATE_INTERVAL_MS) {
    rotateIndex = (rotateIndex + 1) % ROTATE_COUNT;
    sendRotatingFields(rotateIndex);
    lastRotateAt = millis();

    Serial.print("Dummy-Daten Rotation ");
    Serial.print(rotateIndex);
    Serial.println(" gesendet.");
  }
}
