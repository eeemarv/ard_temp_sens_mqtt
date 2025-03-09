#include <Arduino.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <EthernetENC.h>
#include <Watchdog.h>
#include <.env.h>
#include <env_defaults.h>

#define WS_T0H 1  //6 3 400 ns // cycles
#define WS_T1H 8  //12 9 800 ns
#define WS_T0L 6  //13 9 7 850 ns
#define WS_T1L 1   //7 3  450 ns

#define WS_BITMASK B00000001 // PC0
#define WS_PORT PORTC
#define WS_DDR DDRC
#define WS_LOW WS_PORT &= ~WS_BITMASK;
#define WS_HIGH WS_PORT |= WS_BITMASK;
#define WS_MIN_INTERVAL 1  // milliseconds
#define WS_PULSE_INTERVAL 20
#define WS_MAX_INTERVAL 60111

#define DS_RAW_TO_C_MUL 0.0078125f

#define DS_INIT_TIME 2000
#define DS_REQUEST_TIME 2000 // 900
#define DS_READ_TIME 50
#define DS_INDEX_TIME 50

#define DS_INDEX_INIT 0
#define DS_MAX_DEVICE_COUNT 16

#define DS_REQUEST 0x01
#define DS_READ 0x02
#define DS_INDEX 0x08

#define PWM_PD PD3
#define PWM_LEVEL 60

#define MQTT_CONNECT_RETRY_TIME 5000

#define OW_PRE_RESET_STABILIZE_TIME 100
#define OW_RESET_PULSE_TIME 490
#define OW_RESET_RELEASE_TIME 240
#define OW_WRITE_INIT_TIME 10
#define OW_WRITE_ONE_TIME 60
#define OW_WRITE_ZERO_TIME 60
#define OW_READ_INIT_TIME 3
#define OW_READ_SAMPLE_TIME 12
#define OW_READ_RELEASE_TIME 45
#define OW_IDLE_TIME 200

#define OW_PORT PORTD
#define OW_DDR DDRD

#define OW_PDU_BITMASK B00000100 // PD2
#define OW_DRIVE_BITMASK B00010000 // PD4
#define OW_OUTPUT_BITMASK OW_PDU_BITMASK | OW_DRIVE_BITMASK

#define OW_OUTPUT_TO_HZ OW_PORT |= OW_PDU_BITMASK; OW_PORT &= ~OW_DRIVE_BITMASK;

#define OW_OUTPUT_UP_TO_HZ OW_PORT |= OW_PDU_BITMASK;
#define OW_OUTPUT_DOWN_TO_HZ OW_PORT &= ~OW_DRIVE_BITMASK;
#define OW_OUTPUT_TO_UP OW_PORT &= (~OW_PDU_BITMASK & ~OW_DRIVE_BITMASK);
#define OW_OUTPUT_TO_DOWN OW_PORT |= OW_PDU_BITMASK | OW_DRIVE_BITMASK;

#define OW_OUTPUT_TRANS_TIME 2

#define ACO_BITMASK B00100000
#define OW_SAMPLE (ACSR & ACO_BITMASK)  // comparator output
#define OW_ADDRESS_BIT_SIZE 64
#define OW_ADDRESS_BYTE_SIZE 8

#define PREV_DISC_BIT_POS_INIT -1
#define PREV_DISC_BIT_POS_END -1
#define EEPROM_DS_DEVICE_COUNT 0x00
#define EEPROM_DS_DEVICE_ADDRESSES 0x01

// test timings on oscilloscope
#define TEST_TIME_BITMASK B00100000 // PD5
#define TEST_TIME_PORT PORTD
#define TEST_TIME_DDR DDRD
#define TEST_TIME_LOW OW_PORT &= ~TEST_TIME_BITMASK;
#define TEST_TIME_HIGH OW_PORT |= TEST_TIME_BITMASK;
#define TEST_TIME_RELEASE OW_DDR &= ~TEST_TIME_BITMASK;
#define TEST_TIME_PULL OW_DDR |= TEST_TIME_BITMASK;
// #define TEST_TIME_PIN_ENABLE

#define DS_SEARCH_ROM_COMMAND 0xf0
#define DS_READ_ROM_COMMAND 0x33
#define DS_MATCH_ROM_COMMAND 0x55
#define DS_SKIP_ROM_COMMAND 0xcc
#define DS_ALARM_SEARCH_COMMAND 0xec
#define DS_CONVERT_TEMP_COMMAND 0x44
#define DS_READ_SCRATCHPAD_COMMAND 0xbe
#define DS_WRITE_SCRATCHPAD_COMMAND 0x4e
#define DS_COPY_SCRATCHPAD_COMMAND 0x48
#define DS_RECALL_EE_COMMAND 0xb8
#define DS_READ_POWER_SUPPLY_COMMAND 0xb4

#define DS_CONFIG_12_BIT 0x7f
#define DS_SCRATCHPAD_TEMP_LSB 0
#define DS_SCRATCHPAD_TEMP_MSB 1
#define DS_SCRATCHPAD_REG_TH 2
#define DS_SCRATCHPAD_REG_TL 3
#define DS_SCRATCHPAD_CONFIG 4
#define DS_SCRATCHPAD_RESERVED_5 5
#define DS_SCRATCHPAD_RESERVED_6 6
#define DS_SCRATCHPAD_RESERVED_7 7
#define DS_SCRATCHPAD_CRC 8
#define DS_SCRATCHPAD_SIZE 9
typedef uint8_t DsScratchPad[DS_SCRATCHPAD_SIZE];

#define CYCLES_MICROSEC (F_CPU / 1000000)

#define DS_NEXT(DS_STATUS, DS_INTERVAL) \
  dsStatus = DS_STATUS; \
  dsInterval = DS_INTERVAL; \
  dsLastRequest = millis(); \
  return;

#define DS_RESTART_CYCLE \
  dsError = true; \
  dsIndex = dsDeviceCount; \
  DS_NEXT(DS_INDEX, DS_INDEX_TIME);

#ifdef SERIAL_EN
  #define SERIAL_PRINT_HEX(N) \
    if (N < 16){ \
      Serial.print("0"); \
    } \
    Serial.print(N, HEX); \
    Serial.print(" ");
#else
  #define SERIAL_PRINT_HEX(N)
#endif

// See https://stackoverflow.com/a/63468969
template< unsigned N >
inline static void nops(){
  asm ("nop");
  nops< N - 1 >();
}
template<> inline void nops<0>(){};

#define OW_TRANS_TIME nops<CYCLES_MICROSEC * OW_OUTPUT_TRANS_TIME>();

byte mac[] = {0xDE, 0xAD, MAC_4_LAST};
const IPAddress selfIP(SELF_IP);
const IPAddress mqttServerIP(MQTT_SERVER_IP);
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

uint8_t wsStatusColor = 1;
uint32_t wsLast = 0;
uint16_t wsInterval = 0;
uint8_t wsRGBDef[][3] = {
// permanent color
  {0x00, 0x00, 0x10}, // power
  {0x10, 0x00, 0x00}, // setup
  {0x80, 0x00, 0x20}, // no ethernet
  {0x80, 0x20, 0x00}, // no sensor(s)
// pulse for 200ms
  {0x00, 0x40, 0x00},  // read one sensor
  {0xff, 0xff, 0xff},  // calc/send temp
  {0x80, 0x00, 0xff},  // error sensor
  {0xff, 0x00, 0x00},  // presence pulse
};

#ifdef WATCHDOG_EN
  Watchdog watchdog;
#endif

uint32_t mqttLastReconnectAttempt = 0;
bool mqttPublishTrig = false;

uint32_t dsLastRequest = 0;
uint16_t dsInterval = DS_INIT_TIME;
uint8_t dsStatus = DS_REQUEST;
uint8_t dsIndex = DS_INDEX_INIT;
uint8_t dsDeviceCount;
int8_t prevDiscBitPos = -1;
bool dsError = false;
int16_t dsTemp[DS_MAX_DEVICE_COUNT];
uint8_t dsSort[DS_MAX_DEVICE_COUNT];
DsScratchPad dsScratchPad;
float temperature;

uint8_t mqttConnectAttempts0 = 0;
uint8_t mqttConnectAttempts1 = 0;
uint32_t lastPresence = 0;

uint16_t dsErrorConnectCount = 0;
uint16_t dsError85Count = 0;
uint32_t dsLastErrorPub = 0;

const uint8_t* adr;

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
// And https://github.com/PaulStoffregen/OneWire
static const uint8_t PROGMEM dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

inline uint8_t oneWireCrc8(uint8_t crc){
  return pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
    pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
}

/**
 * @brief put in endless loop to trigger Watchdog.
 */
void stop(){
  while(true){
    delay(100);
  }
}

/**
 * @brief Write one byte to the WS2812 LED
*/
inline void wsWriteColor(uint8_t wsColor){
  uint8_t bitMask;
  for (bitMask = 0x80; bitMask; bitMask >>= 1){
    WS_HIGH;
    if (wsColor & bitMask){
      nops<WS_T1H>();
      WS_LOW;
      nops<WS_T1L>();
      continue;
    }
    nops<WS_T0H>();
    WS_LOW;
    nops<WS_T0L>();
  }
}

/**
 * Write a color to the LED, the order is R-G-B
 */
inline void wsWrite(uint8_t color){
  noInterrupts();
    wsWriteColor(wsRGBDef[color][0]);
    wsWriteColor(wsRGBDef[color][1]);
    wsWriteColor(wsRGBDef[color][2]);
  interrupts();
  wsLast = millis();
}

inline void wsWriteStatus(uint8_t color){
  wsStatusColor = color;
  wsInterval = WS_MIN_INTERVAL;
}

inline void wsWriteStatusNoLoop(uint8_t color){
  wsWriteStatus(color);
  wsWrite(color);
  wsInterval = WS_MAX_INTERVAL;
}

inline void wsWritePulse(uint8_t color){
  if (!(millis() - wsLast > WS_MIN_INTERVAL)){
    return;
  }
  wsInterval = WS_PULSE_INTERVAL;
  wsWrite(color);
}

inline void wsWriteStatusPower(){
  wsWriteStatusNoLoop(0);
}
inline void wsWriteStatusSetup(){
  wsWriteStatusNoLoop(1);
}
inline void wsWriteStatusNoEthernet(){
  wsWriteStatusNoLoop(2);
}
inline void wsWriteStatusNoSensors(){
  wsWriteStatusNoLoop(3);
}
inline void wsWritePulseReadSensor(){
  wsWritePulse(4);
}
inline void wsWritePulseSendTemp(){
  wsWritePulse(5);
}
inline void wsWritePulseErrorSensor(){
  wsWritePulse(6);
}
inline void wsWritePulsePresence(){
  wsWritePulse(7);
}

/**
 * @return true Devices found
 * @return false No devices found
 */
inline bool oneWireReset(){
	uint8_t retryCount = 200;
  uint8_t releaseTime;

	// ensure the bus is high
  while (!OW_SAMPLE){
    delayMicroseconds(20);
    retryCount--;
    if (!retryCount){
      return false;
    }
  }
  OW_OUTPUT_TO_UP; // n

  delayMicroseconds(OW_PRE_RESET_STABILIZE_TIME);

  OW_OUTPUT_UP_TO_HZ;  // n
  OW_TRANS_TIME;

  noInterrupts();

  OW_OUTPUT_TO_DOWN;

	delayMicroseconds(OW_RESET_PULSE_TIME);

  OW_OUTPUT_DOWN_TO_HZ;
  OW_TRANS_TIME;
  OW_OUTPUT_TO_UP;

  nops<CYCLES_MICROSEC * 10>();
  OW_OUTPUT_UP_TO_HZ;
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_HIGH;
#endif
  delayMicroseconds(60);
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_LOW;
#endif
  if (OW_SAMPLE){
    delayMicroseconds(240);
    OW_OUTPUT_TO_UP;
    interrupts();
    delayMicroseconds(240 + OW_IDLE_TIME);
    return false; // no ds response
  }
  delayMicroseconds(240);
  for (releaseTime = OW_RESET_RELEASE_TIME; releaseTime; releaseTime--){
    if (OW_SAMPLE){
      break;
    }
    nops<(CYCLES_MICROSEC * 1) - 4>();
  }

  OW_OUTPUT_TO_UP;

  interrupts();
  if (releaseTime){
    delayMicroseconds(releaseTime);
  }
  delayMicroseconds(OW_IDLE_TIME);
  return true;
}

inline void oneWireWriteBit(bool b){
  // slot 90µs + idle 20µs
  noInterrupts();

  OW_OUTPUT_UP_TO_HZ;
  OW_TRANS_TIME;
  OW_OUTPUT_TO_DOWN;

  if (b){
    nops<CYCLES_MICROSEC * OW_WRITE_INIT_TIME>();
    OW_OUTPUT_DOWN_TO_HZ;
    OW_TRANS_TIME;
    OW_OUTPUT_TO_UP;
    interrupts();
    delayMicroseconds(OW_WRITE_ONE_TIME + OW_IDLE_TIME);
    return;
  }
  delayMicroseconds(OW_WRITE_INIT_TIME + OW_WRITE_ZERO_TIME);
  OW_OUTPUT_DOWN_TO_HZ;
  OW_TRANS_TIME;
  OW_OUTPUT_TO_UP;
  interrupts();
  delayMicroseconds(OW_IDLE_TIME);
}

inline void oneWireWriteByte(uint8_t v){
  uint8_t bitMask = 0x01;
  for(bitMask = 0x01; bitMask; bitMask <<= 1){
    oneWireWriteBit(v & bitMask);
  }
}

inline bool oneWireReadBit(){
  bool b = false;
  uint8_t releaseTime;

  noInterrupts();
  OW_OUTPUT_UP_TO_HZ;
  OW_TRANS_TIME;
  OW_OUTPUT_TO_DOWN;

#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_HIGH;
#endif
  nops<CYCLES_MICROSEC * OW_READ_INIT_TIME>();
  OW_OUTPUT_DOWN_TO_HZ;

#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_LOW;
#endif
  nops<(CYCLES_MICROSEC * OW_READ_SAMPLE_TIME) - 4>();
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_HIGH;
#endif
  if (OW_SAMPLE){
    b = true;
    OW_OUTPUT_TO_UP;
  }
  for (releaseTime = OW_READ_RELEASE_TIME; releaseTime; releaseTime--){
    if (OW_SAMPLE){
      break;
    }
    nops<(CYCLES_MICROSEC * 1) - 4>();
  }
  OW_OUTPUT_TO_UP;
  interrupts();
  if (releaseTime){
    delayMicroseconds(releaseTime);
  }
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_LOW;
#endif
  delayMicroseconds(OW_IDLE_TIME);
  return b;
}

inline uint8_t oneWireReadByte(){
  uint8_t bitMask;
  uint8_t v = 0x00;

  for (bitMask = 0x01; bitMask; bitMask <<= 1){
    if (oneWireReadBit()){
      v |= bitMask;
    }
  }
  return v;
}

/**
 * @brief select ROM based on dsIndex.
 * ROM device address is read from EEPROM.
 */
inline void oneWireRomSelect(){
  uint8_t eepromaAdrStart = EEPROM_DS_DEVICE_ADDRESSES + (dsIndex * OW_ADDRESS_BYTE_SIZE);

  oneWireWriteByte(DS_MATCH_ROM_COMMAND);

  for (uint8_t jjj = 0; jjj < OW_ADDRESS_BYTE_SIZE; jjj++){
    oneWireWriteByte(EEPROM.read(eepromaAdrStart + jjj));
  }
}

inline bool oneWireReadDsScratchPath(){
  uint8_t i;
  uint8_t crc8 = 0;

  oneWireWriteByte(DS_READ_SCRATCHPAD_COMMAND);
  for (i = 0; i < DS_SCRATCHPAD_SIZE; i++){
    dsScratchPad[i] = oneWireReadByte();
  }
  for (i = 0; i < DS_SCRATCHPAD_SIZE; i++){
		if (dsScratchPad[i] != 0) {
			break;
		}
    return false;
	}
  for(i = 0; i < DS_SCRATCHPAD_SIZE - 1; i++){
    crc8 ^= dsScratchPad[i];
    crc8 = oneWireCrc8(crc8);
  }
  if (crc8 == dsScratchPad[DS_SCRATCHPAD_CRC]){
    return true;
  }
  return false;
}

/**
 * @brief Searches ROM address of devices and updates in EEPROM,
 * start with global vars dsIndex = 0 and prevDiscBitPos = -1,
 * when prevDiscBitPos is -1 on return, the search is finished.
 * Note 1: increase dsIndex after call.
 * Note 2: no crc8-verification performed yet.
 * See Maxim/Dallas One wire "Application Note 187"
 * @return true ROM address updated in EEPROM, dsIndex increased
 * @return false Error occured
 */
inline bool oneWireSearchRomOneAddr(){
  uint8_t bitPos;
  int8_t prevDiscZeroBitPos = -1;
  bool readBit;
  bool readBitComp;
  uint8_t prevRomByte;
  uint8_t curRomByte;
  uint8_t eepromCurRomByteIndex;

  if (!oneWireReset()){
    #ifdef SERIAL_EN
      Serial.println("No devices found on reset.");
    #endif
    return false;
  }

  oneWireWriteByte(DS_SEARCH_ROM_COMMAND);

  #ifdef SERIAL_EN
    Serial.print("adr.");
    Serial.print(dsIndex, HEX);
    Serial.print(": ");
  #endif

  for (bitPos = 0; bitPos < OW_ADDRESS_BIT_SIZE; bitPos++){
    uint8_t romByteMask = 1 << (bitPos & 0x07);
    uint8_t romByteId = bitPos >> 3;

    if (romByteMask == 0x01){
      if (bitPos){
        SERIAL_PRINT_HEX(curRomByte);
        EEPROM.update(eepromCurRomByteIndex, curRomByte);
      }
      eepromCurRomByteIndex = EEPROM_DS_DEVICE_ADDRESSES + (dsIndex * OW_ADDRESS_BYTE_SIZE) + romByteId;
      curRomByte = 0x00;
      if (dsIndex){
        prevRomByte = EEPROM.read(eepromCurRomByteIndex - OW_ADDRESS_BYTE_SIZE);
      }
    }

    readBit = oneWireReadBit();
    readBitComp = oneWireReadBit();

    if (readBit && readBitComp){
      #ifdef SERIAL_EN
        Serial.println("search ROM: no devices.");
      #endif
      return false;
    }

    if (readBit != readBitComp){
      if (readBit){
        curRomByte |= romByteMask;
      }
      oneWireWriteBit(readBit);
      continue;
    }

    // bit discrepancy occurred

    if (bitPos < prevDiscBitPos){
      if (dsIndex == 0){
        #ifdef SERIAL_EN
          Serial.println("Search error: discrepancy on dsIndex 0.");
        #endif
        return false;
      }
      if (romByteMask & prevRomByte){
        oneWireWriteBit(true);
        curRomByte |= romByteMask;
        continue;
      }
      prevDiscZeroBitPos = bitPos;
      oneWireWriteBit(false);
      continue;
    }

    if (bitPos > prevDiscBitPos){
      prevDiscZeroBitPos = bitPos;
      prevDiscBitPos = bitPos;
      oneWireWriteBit(false);
      continue;
    }

    oneWireWriteBit(true);
    curRomByte |= romByteMask;
  }

  prevDiscBitPos = prevDiscZeroBitPos;
  SERIAL_PRINT_HEX(curRomByte);
  EEPROM.update(eepromCurRomByteIndex, curRomByte);
  return true;
}

/**
 * @brief search addresses of all connected
 * one wire devices and update in EEPROM and
 * check CRC bytes.
 */
void oneWireSearchRomAllAddr(){
  #ifdef SERIAL_EN
    Serial.println();
    Serial.println("Search ROM");
  #endif
  prevDiscBitPos = PREV_DISC_BIT_POS_INIT;

  for (dsIndex = DS_INDEX_INIT; dsIndex < DS_MAX_DEVICE_COUNT; dsIndex++){
    if (oneWireSearchRomOneAddr()){
      // search succesful, check crc
      uint8_t crc8 = 0x00;
      for (uint8_t jjj; jjj < OW_ADDRESS_BYTE_SIZE; jjj++){
        crc8 ^= EEPROM.read(jjj + EEPROM_DS_DEVICE_ADDRESSES + (dsIndex * OW_ADDRESS_BYTE_SIZE));
        crc8 = oneWireCrc8(crc8);
      }
      if (crc8){
        #ifdef SERIAL_EN
          Serial.println("CRC error");
        #endif

        wsWriteStatusNoSensors();
        stop();
      }
      #ifdef SERIAL_EN
        Serial.println("CRC ok");
      #endif

      if (prevDiscBitPos >= 0){
        // search next
        continue;
      }
      // last device found
      dsDeviceCount = dsIndex + 1;
      break;
    }
    #ifdef SERIAL_EN
      Serial.print("error on search ROM");
    #endif
    wsWriteStatusNoSensors();
    stop();
  }
  EEPROM.update(EEPROM_DS_DEVICE_COUNT, dsDeviceCount);
  #ifdef SERIAL_EN
    Serial.print("Device count:");
    Serial.print(dsDeviceCount);
    Serial.println();
  #endif
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
}

bool mqttReconnect() {
  uint8_t iii;
  uint8_t charPos;
  char clientId[10] = CLIENT_ID_PREFIX;
  for (iii = 0; iii < 4; iii++){
    charPos = strlen(clientId);
    clientId[charPos] = '0' + random(0, 10);
    clientId[charPos + 1] = '\0';
  }
  if (mqttClient.connect(clientId)) {
    // subs
  }
  return mqttClient.connected();
}

void calcTemperature(){
  uint8_t jjj;
  uint8_t dvc;
  int32_t accTemp = 0;
  uint8_t accTempDiv = 0;

  for (jjj = 0; jjj < dsDeviceCount; jjj++){
    dsSort[jjj] = jjj;
  }

  for (jjj = 0; jjj < (dsDeviceCount - 1); jjj++){
    if (dsTemp[dsSort[jjj]] > dsTemp[dsSort[jjj + 1]]){
      dsSort[jjj] ^= dsSort[jjj + 1];
      dsSort[jjj + 1] ^= dsSort[jjj];
      dsSort[jjj] ^= dsSort[jjj + 1];
    }
  }

  #ifdef SERIAL_EN
    Serial.print("Sort select: ");
  #endif

  for (jjj = 0; jjj < dsDeviceCount; jjj++){
    if (dsDeviceCount > 5){
      dvc = dsDeviceCount / 2;
      if (jjj > (dvc + 1)){
        continue;
      }
      if (jjj < (dvc - 2)){
        continue;
      }
    }
    #ifdef SERIAL_EN
      Serial.print(dsSort[jjj]);
      Serial.print(" ");
    #endif
    accTemp += dsTemp[dsSort[jjj]];
    accTempDiv++;
  }

  temperature = (float) ((((float) accTemp) / ((float) accTempDiv)) * DS_RAW_TO_C_MUL);
}

bool publishTemp() {
  char m1[8];

  #ifdef SERIAL_EN
    Serial.println();
    Serial.print("pub: ");
    Serial.print(PUB_WATER_TEMP);
    Serial.print(" : ");
    Serial.println(temperature);
  #endif

  dtostrf(temperature, 4, 2, m1);

  return mqttClient.publish(PUB_WATER_TEMP, m1);
}

/**
 * @brief
 * UNO pins
 * PC0 data ws2812-LED (shows status)
 * PD2 one wire DPU
 * PD3 PWM out
 * PD4 one wire DRIVE
 * A6 (AIN0) one wire in (pos comp)
 * A7 (AIN1) PWM in (filtered with resistor and capicator)
 */
void setup() {
  delay(200);
  WS_LOW;
  WS_DDR |= WS_BITMASK; // Ws output
  delay(10);
  wsWriteStatusSetup();
  delay(50);
  /** SETUP PWM */
  DDRD |= 1 << PWM_PD; // set to output
  analogWrite(PWM_PD, PWM_LEVEL); // pinMode output is included in this
  TCCR2B = (TCCR2B & 0xf0) | 0x01; // PWM freq 31kHz+
  DDRD &= B00111111; // PD7 & PD6 as input for comparator
  PORTD &= B00111111; // PD7 & PD6 no pull ups
  ADCSRB = 0x00; // disable MUX analog inputs for analog comparator.
  DIDR1 = 0x00; // disable PD7 & PD6 digital inputs
  ACSR = 0x00; // analog comparator enable, no interrupts, no capture
  delay(500);

  OW_OUTPUT_TO_UP; // default state, always enter and leave functions in UP state.
  OW_DDR |= OW_OUTPUT_BITMASK;

#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_LOW;
  TEST_TIME_PULL;
#endif

  /** SETUP ETHERNET CONNECTION */
#ifndef MQTT_DIS
  mqttClient.setServer(mqttServerIP, 1883);
  mqttClient.setCallback(mqttCallback);
  Ethernet.init(ETH_CS_PIN);
  SPI.begin();
  Ethernet.begin(mac, selfIP);
#endif

  #ifdef SERIAL_EN
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  #endif

  delay(250);
  #ifdef WATCHDOG_EN
    watchdog.enable(Watchdog::TIMEOUT_4S);
  #endif

  oneWireSearchRomAllAddr();

  #ifdef SERIAL_EN
    Serial.println("Set devices to 12bit mode");
  #endif
  for (dsIndex = 0; dsIndex < dsDeviceCount; dsIndex++){
    oneWireReadDsScratchPath();
    #ifdef SERIAL_EN
      Serial.print("adr.");
      Serial.print(dsIndex, HEX);
      Serial.print(": ");
    #endif
    if (dsScratchPad[DS_SCRATCHPAD_CONFIG] == DS_CONFIG_12_BIT){
      #ifdef SERIAL_EN
        Serial.println("no update.");
      #endif
      continue;
    }
    oneWireReset();
    oneWireRomSelect();
    oneWireWriteByte(DS_WRITE_SCRATCHPAD_COMMAND);
    oneWireWriteByte(dsScratchPad[DS_SCRATCHPAD_REG_TH]);
    oneWireWriteByte(dsScratchPad[DS_SCRATCHPAD_REG_TL]);
    oneWireWriteByte(DS_CONFIG_12_BIT);
    delay(10);
    oneWireReset();
    oneWireRomSelect();
    oneWireWriteByte(DS_COPY_SCRATCHPAD_COMMAND);
    delay(50);
    #ifdef SERIAL_EN
      Serial.println("updated.");
    #endif
  }
  dsIndex = DS_INDEX_INIT;

#ifndef MQTT_DIS
  if (Ethernet.hardwareStatus() == EthernetHardwareStatus::EthernetNoHardware) {
    #ifdef SERIAL_EN
      Serial.println("ENC28J60 not found.");
    #endif
    wsWriteStatusNoEthernet();
    while(1){
      delay(1);
    }
  }

  if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
    #ifdef SERIAL_EN
      Serial.println("Ethernet not connected.");
    #endif
    wsWriteStatusNoEthernet();
  } else {
    #ifdef SERIAL_EN
      Serial.println("Ethernet ok.");
    #endif
  }
#endif

  dsLastRequest = millis();
  wsWriteStatusPower();
}

void loop() {
  char msg1[16];
  char msg2[8];
  int16_t tmprtr;

  #ifdef WATCHDOG_EN
    watchdog.reset();
  #endif

  #ifndef MQTT_DIS
  Ethernet.maintain();

  if (mqttClient.connected()) {
    mqttConnectAttempts0 = 0;
    mqttConnectAttempts1 = 0;
    if (mqttPublishTrig) {
      publishTemp();
      wsWritePulseSendTemp();
      mqttPublishTrig = false;
    }

    if (millis() - lastPresence >  PRESENCE_INTERVAL){
      #ifdef SERIAL_EN
      Serial.print("pub: ");
      Serial.println(PUB_PRESENCE);
      #endif
      mqttClient.publish(PUB_PRESENCE, "1");
      wsWritePulsePresence();
      lastPresence = millis();
    }

    if (millis() - dsLastErrorPub > ERRORS_INTERVAL){
      #ifdef SERIAL_EN
      Serial.print("pub: ");
      Serial.println(PUB_ERRORS);
      Serial.print("e85: ");
      Serial.print(dsError85Count);
      Serial.print(" econn: ");
      Serial.println(dsErrorConnectCount);
      #endif
      strcpy(msg1, "e85: ");
      itoa(dsError85Count, msg2, 10);
      strcat(msg1, msg2);
      strcat(msg1, " econn: ");
      itoa(dsErrorConnectCount, msg2, 10);
      strcat(msg1, msg2);
      mqttClient.publish(PUB_ERRORS, msg1);
      dsLastErrorPub = millis();
    }

    mqttClient.loop();
  } else {
    if (!(mqttConnectAttempts0 || mqttConnectAttempts1)){
      #ifdef SERIAL_EN
        Serial.print(".m");
      #endif
    } else if (!mqttConnectAttempts0){
      #ifdef SERIAL_EN
        Serial.print(".");
      #endif
      mqttConnectAttempts1++;
    }
    mqttConnectAttempts0++;
    if (millis() - mqttLastReconnectAttempt > MQTT_CONNECT_RETRY_TIME) {
      if (mqttReconnect()) {
        #ifdef SERIAL_EN
          Serial.println("connected");
        #endif
        mqttLastReconnectAttempt = 0;
      } else {
        #ifdef SERIAL_EN
          Serial.print("failed, rc=");
          Serial.print(mqttClient.state());
          Serial.print(" try again in ");
          Serial.print(MQTT_CONNECT_RETRY_TIME);
          Serial.println(" ms.");
        #endif
        mqttLastReconnectAttempt = millis();
      }
    }
  }
  #endif

  if (millis() - wsLast > wsInterval){
    wsInterval = WS_MAX_INTERVAL;
    wsWrite(wsStatusColor);
  }

  if (dsDeviceCount && (millis() - dsLastRequest > dsInterval)) {
    if (dsStatus & DS_READ){
      if (!oneWireReset()){
        #ifdef SERIAL_EN
        Serial.println("ERR oneWire reset");
        #endif
        dsErrorConnectCount++;
        DS_RESTART_CYCLE;
      }

      oneWireRomSelect();

      if (oneWireReadDsScratchPath()){
        wsWritePulseReadSensor();
      } else {

        #ifdef SERIAL_EN
        Serial.print("ERR scratchpad");
        #endif

        wsWritePulseErrorSensor();

        dsErrorConnectCount++;
        DS_RESTART_CYCLE;
      }

      tmprtr = (((int16_t) dsScratchPad[DS_SCRATCHPAD_TEMP_MSB]) << 11)
        | (((int16_t) dsScratchPad[DS_SCRATCHPAD_TEMP_LSB]) << 3);

      if (tmprtr >=  DS_MAX_RAW){
        wsWritePulseErrorSensor();
        dsError85Count++;
        DS_RESTART_CYCLE;
      }
      if (tmprtr <= DS_MIN_RAW){
        wsWritePulseErrorSensor();
        dsError85Count++;
        DS_RESTART_CYCLE;
      }

      dsTemp[dsIndex] = tmprtr;

      #ifdef SERIAL_EN
        Serial.print("*");
        Serial.print((float) (tmprtr * DS_RAW_TO_C_MUL));
        Serial.print("* ");
        Serial.print(tmprtr);
        Serial.print(", ");
      #endif

      DS_NEXT(DS_INDEX, DS_READ_TIME);

    } else if (dsStatus & DS_INDEX) {

      dsIndex++;

      if (dsIndex >= dsDeviceCount){
        dsIndex = 0;
        if (!dsError){
          calcTemperature();
          mqttPublishTrig = true;
        }
        dsError = false;
      }

      DS_NEXT(DS_REQUEST, DS_INDEX_TIME);

    } else if (dsStatus & DS_REQUEST) {

      #ifdef SERIAL_EN
        Serial.print("i");
        Serial.print(dsIndex);
        Serial.print(": ");
      #endif

      if (!oneWireReset()){
        #ifdef SERIAL_EN
          Serial.println("ERR req reset conn");
        #endif

        wsWritePulseErrorSensor();

        dsErrorConnectCount++;
        DS_RESTART_CYCLE;
      }

      oneWireRomSelect();
      oneWireWriteByte(DS_CONVERT_TEMP_COMMAND);
      DS_NEXT(DS_READ, DS_REQUEST_TIME);
    }

    dsLastRequest = millis();
  }
}
