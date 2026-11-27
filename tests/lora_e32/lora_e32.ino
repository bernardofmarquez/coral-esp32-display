// Hardware test for the Ebyte E32-433T20D (LoRa, 433 MHz) on the LabSis board:
// ESP32 -> UART2 -> E32 -> antenna. Standalone sketch: flashing it replaces
// coral_display, so reflash the normal firmware afterwards (sub/README.md).
//
// Wiring (free pins in Esquema_Ilustrado_ESP32.pdf and on the board):
//   E32 TXD  -> GPIO35 (UART2 RX, input-only pin)
//   E32 RXD  <- GPIO2  (UART2 TX)
//   E32 AUX  -> GPIO34 (input-only pin, HIGH = idle)
//   E32 M0+M1 (solder bridge) <- GPIO5 (LOW = normal mode, HIGH = configuration)
//   E32 VCC  <- 5 V (VIN rail), E32 GND -> GND
//
// Build and run from the repository root:
//   arduino-cli compile --fqbn esp32:esp32:esp32 tests/lora_e32
//   arduino-cli upload  --fqbn esp32:esp32:esp32 --port <PORT> tests/lora_e32
//   arduino-cli monitor --port <PORT> --config baudrate=115200
//
// Monitor commands (one per line): /info, /tx, /auto, /padrao, /ajuda.
// Any other line is transmitted as typed. Never transmit without an antenna.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// GPIO5 is a strapping pin pulled HIGH at reset, so the module stays in configuration
// mode (no RF) while the ESP32 boots. GPIO2 is also a strapping pin: the bootloader
// needs it LOW, so if an upload fails with "Wrong boot mode", the E32 RXD pull-up on
// GPIO2 is the suspect.
constexpr int PIN_E32_TXD = 35;   // ESP32 RX
constexpr int PIN_E32_RXD = 2;    // ESP32 TX
constexpr int PIN_E32_AUX = 34;
constexpr int PIN_E32_MODE = 5;   // M0 and M1 bridged on the module header
constexpr uint32_t AUTO_PERIOD_MS = 2000;
// Range-test mode: transmit every 2 s right after power-up, with no serial monitor.
// The antenna must be connected before powering the board. false = only /tx and /auto.
constexpr bool AUTO_TX_AT_BOOT = true;

HardwareSerial e32(2);
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

uint8_t params[6] = {0};
bool moduleOk = false, oledOk = false, autoTx = false;
uint32_t uartBaud = 0, uartFormat = 0;   // currently open UART settings
uint32_t normalBaud = 9600, normalFormat = SERIAL_8N1;
uint32_t txCount = 0, rxCount = 0, lastAuto = 0, lastRadioByte = 0;
String consoleLine, radioLine, lastRx = "-";

bool auxHigh() { return digitalRead(PIN_E32_AUX) == HIGH; }

// Returns how long AUX took to go HIGH, or -1 on timeout.
long waitAux(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (!auxHigh()) {
    if (millis() - start >= timeoutMs) return -1;
    delay(1);
  }
  return long(millis() - start);
}

void openUart(uint32_t baud, uint32_t format) {
  if (baud == uartBaud && format == uartFormat) return;
  e32.begin(baud, format, PIN_E32_TXD, PIN_E32_RXD);
  uartBaud = baud;
  uartFormat = format;
}

// Configuration mode (M0 = M1 = 1) always talks 9600 8N1; normal mode uses SPED.
bool setMode(bool config) {
  e32.flush();
  waitAux(1000);  // the module finishes pending work before switching
  digitalWrite(PIN_E32_MODE, config ? HIGH : LOW);
  delay(5);
  bool ready = waitAux(1500) >= 0;  // leaving configuration reloads parameters
  delay(10);                        // datasheet: mode valid 2 ms after AUX rises
  if (config) openUart(9600, SERIAL_8N1);
  else openUart(normalBaud, normalFormat);
  while (e32.available()) e32.read();
  return ready;
}

// No default arguments in this sketch: Arduino's generated prototypes would repeat them.
size_t exchange(const uint8_t *command, size_t length, uint8_t *answer, size_t want,
                uint32_t timeoutMs) {
  while (e32.available()) e32.read();
  e32.write(command, length);
  e32.flush();
  size_t got = 0;
  uint32_t start = millis();
  while (got < want && millis() - start < timeoutMs) {
    if (e32.available()) answer[got++] = uint8_t(e32.read());
    else delay(1);
  }
  waitAux(500);
  delay(5);
  return got;
}

void printHex(const uint8_t *bytes, size_t count) {
  if (!count) { Serial.print("(nada)"); return; }
  for (size_t i = 0; i < count; ++i) Serial.printf("%02X ", bytes[i]);
}

uint32_t spedBaud(uint8_t sped) {
  static const uint32_t table[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
  return table[(sped >> 3) & 7];
}

uint32_t spedFormat(uint8_t sped) {
  switch (sped >> 6) {
    case 1: return SERIAL_8O1;
    case 2: return SERIAL_8E1;
    default: return SERIAL_8N1;
  }
}

const char *spedFormatName(uint8_t sped) {
  switch (sped >> 6) {
    case 1: return "8O1";
    case 2: return "8E1";
    default: return "8N1";
  }
}

const char *airRate(uint8_t sped) {
  static const char *table[] = {"0.3", "1.2", "2.4", "4.8", "9.6", "19.2", "19.2", "19.2"};
  return table[sped & 7];
}

unsigned channelMhz() { return 410 + (params[4] & 0x1F); }

void drawOled() {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("LoRa E32 - teste");
  if (moduleOk) oled.printf("%u MHz  %s kbps\n", channelMhz(), airRate(params[3]));
  else oled.println("E32 sem resposta");
  oled.printf("TX %lu   RX %lu\n", (unsigned long)txCount, (unsigned long)rxCount);
  oled.setCursor(0, 32);
  oled.println(lastRx.substring(0, 21));
  oled.setCursor(0, 52);  // yellow band with the board's 180 degree rotation
  oled.println(autoTx ? "TX auto a cada 2 s" : "Parado (/auto)");
  oled.display();
}

bool readModule() {
  Serial.println("\n== Leitura do E32 (modo de configuração, 9600 8N1) ==");
  if (!setMode(true)) Serial.println("  Aviso: AUX não subiu ao entrar na configuração.");
  const uint8_t version[] = {0xC3, 0xC3, 0xC3}, read[] = {0xC1, 0xC1, 0xC1};
  uint8_t info[4], answer[6];
  size_t infoCount = exchange(version, sizeof(version), info, sizeof(info), 400);
  size_t paramCount = exchange(read, sizeof(read), answer, sizeof(answer), 400);

  Serial.print("  Versão (C3 C3 C3):     ");
  printHex(info, infoCount);
  Serial.println(infoCount == 4 && info[0] == 0xC3 ? " -> ok (E32 de 433 MHz responde C3 32 ..)" : "");
  Serial.print("  Parâmetros (C1 C1 C1): ");
  printHex(answer, paramCount);
  Serial.println();

  // C0 = parameters saved in flash, C2 = temporary ones; both carry the same 5 bytes.
  moduleOk = paramCount == 6 && (answer[0] == 0xC0 || answer[0] == 0xC2);
  if (moduleOk) {
    memcpy(params, answer, sizeof(params));
    uint8_t sped = params[3], option = params[5];
    static const int powerT20[] = {20, 17, 14, 10};
    normalBaud = spedBaud(sped);
    normalFormat = spedFormat(sped);
    Serial.printf("    endereço 0x%02X%02X | UART %lu %s | ar %s kbps | canal 0x%02X = %u MHz\n",
                  params[1], params[2], (unsigned long)normalBaud, spedFormatName(sped),
                  airRate(sped), params[4], channelMhz());
    Serial.printf("    %s | %s | FEC %s | potência: bits %u%u (%d dBm no T20D)\n",
                  option & 0x80 ? "transmissão fixa" : "transparente",
                  option & 0x40 ? "push-pull" : "dreno aberto",
                  option & 0x04 ? "ligado" : "desligado",
                  unsigned((option >> 1) & 1), unsigned(option & 1), powerT20[option & 3]);
    static const uint8_t factory[] = {0x00, 0x00, 0x1A, 0x17, 0x44};
    if (memcmp(params + 1, factory, sizeof(factory)) != 0)
      Serial.println("    Diferente do padrão de fábrica (C0 00 00 1A 17 44). /padrao restaura.");
    Serial.println("  OK: ESP32 e E32 conversando pela UART.");
  } else if (!infoCount && !paramCount) {
    Serial.println("  Nenhuma resposta. Confira, nesta ordem:");
    Serial.println("    1. VCC do E32 em 5 V e GND em comum com o ESP32");
    Serial.println("    2. TXD do E32 no GPIO35 e RXD no GPIO2 (não invertidos)");
    Serial.println("    3. M0 e M1 em ponte e ligados ao GPIO5");
    Serial.printf("    4. AUX agora: %s (deve ficar ALTO com o módulo parado)\n",
                  auxHigh() ? "ALTO" : "BAIXO");
  } else {
    Serial.println("  Resposta incompleta ou inesperada: mau contato em TXD/RXD ou M0/M1 fora do nível alto.");
  }
  setMode(false);
  Serial.printf("  Modo normal (M0 = M1 = 0), UART %lu.\n", (unsigned long)normalBaud);
  drawOled();
  return moduleOk;
}

void writeDefaults() {
  Serial.println("\nGravando o padrão de fábrica C0 00 00 1A 17 44 (433 MHz, 2.4 kbps, potência máxima)...");
  const uint8_t defaults[] = {0xC0, 0x00, 0x00, 0x1A, 0x17, 0x44};
  uint8_t echoed[6];
  setMode(true);
  exchange(defaults, sizeof(defaults), echoed, sizeof(echoed), 600);  // the module echoes the frame
  readModule();
}

void transmit(String text) {
  if (!text.length()) return;
  if (text.length() > 56) text.remove(56);  // one radio packet carries up to 58 bytes
  text += '\n';
  waitAux(1000);
  uint32_t start = millis();
  e32.write(reinterpret_cast<const uint8_t *>(text.c_str()), text.length());
  long lowAt = -1, highAt = -1;
  while (millis() - start < 3000) {
    bool high = auxHigh();
    if (lowAt < 0 && !high) lowAt = long(millis() - start);
    if (lowAt >= 0 && high) { highAt = long(millis() - start); break; }
    delay(1);
  }
  ++txCount;
  Serial.printf("TX #%lu (%u bytes): %s", (unsigned long)txCount, text.length(), text.c_str());
  if (lowAt < 0)
    Serial.println("  AUX não baixou: o E32 não recebeu os bytes (RXD <- GPIO2?) ou o AUX está solto.");
  else if (highAt < 0)
    Serial.println("  AUX baixou e não voltou em 3 s: módulo ocupado ou alimentação caindo.");
  else
    Serial.printf("  AUX baixo de %ld a %ld ms: o módulo aceitou o pacote.\n", lowAt, highAt);
  drawOled();
}

void sendTestPacket() {
  char text[40];
  snprintf(text, sizeof(text), "LABSIS #%lu t=%lus", (unsigned long)(txCount + 1),
           (unsigned long)(millis() / 1000));
  transmit(text);
}

void printHelp() {
  Serial.println("\nComandos (uma linha por vez):");
  Serial.println("  /info    lê de novo versão e parâmetros do E32");
  Serial.println("  /tx      envia um pacote de teste");
  Serial.println("  /auto    liga ou desliga o envio a cada 2 s");
  Serial.println("  /padrao  grava o padrão de fábrica no E32");
  Serial.println("  qualquer outro texto é transmitido como está");
  Serial.println("Conecte a antena antes de transmitir.");
}

void handleConsole(String line) {
  line.trim();
  if (!line.length()) return;
  if (line == "/info") readModule();
  else if (line == "/tx") sendTestPacket();
  else if (line == "/auto") {
    autoTx = !autoTx;
    lastAuto = millis() - AUTO_PERIOD_MS;
    Serial.println(autoTx ? "Envio automático LIGADO." : "Envio automático DESLIGADO.");
    drawOled();
  } else if (line == "/padrao") writeDefaults();
  else if (line == "/ajuda" || line == "/help") printHelp();
  else if (line[0] == '/') Serial.println("Comando desconhecido. /ajuda lista os comandos.");
  else transmit(line);
}

void pollConsole() {
  while (Serial.available()) {
    char c = char(Serial.read());
    if (c == '\n' || c == '\r') {
      if (consoleLine.length()) handleConsole(consoleLine);
      consoleLine = "";
    } else if (consoleLine.length() < 200) {
      consoleLine += c;
    }
  }
}

void finishRadioLine() {
  ++rxCount;
  lastRx = radioLine;
  Serial.printf("RX #%lu: %s\n", (unsigned long)rxCount, radioLine.c_str());
  radioLine = "";
  drawOled();
}

void pollRadio() {
  while (e32.available()) {
    uint8_t b = uint8_t(e32.read());
    lastRadioByte = millis();
    if (b == '\n') {
      if (radioLine.length()) finishRadioLine();
    } else if (b != '\r' && radioLine.length() < 120) {
      radioLine += (b >= 32 && b < 127) ? char(b) : '.';
    }
  }
  // Packets without a newline are shown after 100 ms of silence.
  if (radioLine.length() && millis() - lastRadioByte > 100) finishRadioLine();
}

void setup() {
  pinMode(PIN_E32_MODE, OUTPUT);
  digitalWrite(PIN_E32_MODE, HIGH);  // configuration mode first: nothing goes on air
  pinMode(PIN_E32_AUX, INPUT);
  Serial.begin(115200);
  delay(400);
  Serial.println("\n\nTeste LoRa E32: ESP32 -> E32 -> antena");
  Serial.printf("Pinos: TXD->GPIO%d  RXD<-GPIO%d  AUX->GPIO%d  M0+M1<-GPIO%d\n",
                PIN_E32_TXD, PIN_E32_RXD, PIN_E32_AUX, PIN_E32_MODE);

  Wire.begin(21, 22);
  Wire.setTimeOut(100);
  for (uint8_t address : {uint8_t(0x3C), uint8_t(0x3D)}) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      oledOk = oled.begin(SSD1306_SWITCHCAPVCC, address, false, false);
      break;
    }
  }
  if (oledOk) oled.setRotation(2);  // same orientation as coral_display

  openUart(9600, SERIAL_8N1);
  long auxMs = waitAux(2000);
  if (auxMs >= 0) Serial.printf("AUX alto após %ld ms: módulo alimentado e pronto.\n", auxMs);
  else Serial.println("AUX baixo após 2 s: módulo sem alimentação, AUX solto ou ainda iniciando.");

  readModule();
  // On a power bank both boards start together: give the module two more chances.
  for (int attempt = 1; AUTO_TX_AT_BOOT && !moduleOk && attempt < 3; ++attempt) {
    delay(1000);
    readModule();
  }
  autoTx = AUTO_TX_AT_BOOT && moduleOk;
  printHelp();
  if (autoTx) Serial.println("AUTO_TX_AT_BOOT: transmitindo a cada 2 s.");
  drawOled();
}

void loop() {
  pollConsole();
  pollRadio();
  if (autoTx && millis() - lastAuto >= AUTO_PERIOD_MS) {
    lastAuto = millis();
    sendTestPacket();
  }
  delay(1);
}
