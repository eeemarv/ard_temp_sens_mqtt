#include <Arduino.h>
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <EthernetENC.h>
#include <Watchdog.h>
#include <.env.h>

#ifndef MAC_4_LAST
  #define MAC_4_LAST 0xbe, 0xef, 0x13, 0x00
#endif
#ifndef MQTT_SERVER_IP
  #define MQTT_SERVER_IP 192, 168, 0, 20
#endif
#ifndef ETHERNET_SELECT_PIN 
  #define ETHERNET_SELECT_PIN 10
#endif
#ifndef SERIAL_BAUD 
  #define SERIAL_BAUD 115200
#endif

#ifndef TOPIC_SUB_REQ
  #define TOPIC_SUB_REQ "req/sens/w0"
#endif
#ifndef TOPIC_PUB
  #define TOPIC_PUB "sens/w0"
#endif

#ifndef MQTT_CLIENT_ID_PREFIX
  #define MQTT_CLIENT_ID_PREFIX "w0_"
#endif

#define DS_TEMP_LSB 0
#define DS_TEMP_MSB 1
#define DS_RAW_TO_C_MUL 0.0078125f
#define DS_FILTER_ADAPT_STEP 4

#define DS_INIT_TIME 2000
#define DS_REQUEST_TIME 850
#define DS_RETRY_REQUEST_TIME 900
#define DS_READ_TIME 50
#define DS_RETRY_TIME 100
#define DS_FAIL_TIME 50
#define DS_INDEX_TIME 1000
#define DS_MAX_RETRY 64
#define DS_RETRY_INCREASE_TIME 25
#define DS_CONNECTION_ERROR_TIME 50
#define DS_RETRY_AFTER_CONNECTION_ERROR_TIME 1000

#define DS_INDEX_INIT 0
#define DS_MAX_DEVICE_COUNT 8
#define DS_DEVICE_SORT_SELECT B00011000

#define DS_REQUEST 0x01
#define DS_READ 0x02
#define DS_FAIL 0x04
#define DS_INDEX 0x08
#define DS_CONNECTION_ERROR 0x10

#define LOW_NIBBLE 0x0f
#define HIGH_NIBBLE 0xf0

#define PWM_PD PD3
#define PWM_LEVEL 60

#define MQTT_CONNECT_RETRY_TIME 5000

#define ONE_WIRE_PRE_RESET_STABILIZE_TIME 10
#define ONE_WIRE_RESET_PULSE_TIME 490
#define ONE_WIRE_RESET_RELEASE_TIME 240
#define ONE_WIRE_WRITE_INIT_TIME 10
#define ONE_WIRE_WRITE_ONE_TIME 60
#define ONE_WIRE_WRITE_ZERO_TIME 60
#define ONE_WIRE_READ_INIT_TIME 3
#define ONE_WIRE_READ_SAMPLE_TIME 12
#define ONE_WIRE_READ_RELEASE_TIME 45
#define ONE_WIRE_IDLE_TIME 200

#define ONE_WIRE_BITMASK B00010000 // PD4
#define ONE_WIRE_PORT PORTD
#define ONE_WIRE_DDR DDRD
#define ONE_WIRE_LOW ONE_WIRE_PORT &= ~ONE_WIRE_BITMASK;
#define ONE_WIRE_HIGH ONE_WIRE_PORT |= ONE_WIRE_BITMASK;
#define ONE_WIRE_RELEASE ONE_WIRE_DDR &= ~ONE_WIRE_BITMASK;
#define ONE_WIRE_PULL ONE_WIRE_DDR |= ONE_WIRE_BITMASK;
#define ACO_BITMASK B00100000
#define ONE_WIRE_SAMPLE (ACSR & ACO_BITMASK)  // comparator output
#define ONE_WIRE_ADDRESS_BIT_SIZE 64
#define ONE_WIRE_ADDRESS_BYTE_SIZE 8

#define PREV_DISC_BIT_POS_INIT -1
#define PREV_DISC_BIT_POS_END -1
#define EEPROM_DS_DEVICE_COUNT 0x00
#define EEPROM_DS_DEVICE_ADDRESSES 0x01

// test timings on oscilloscope
#define TEST_TIME_BITMASK B00100000 // PD5
#define TEST_TIME_PORT PORTD
#define TEST_TIME_DDR DDRD
#define TEST_TIME_LOW ONE_WIRE_PORT &= ~TEST_TIME_BITMASK;
#define TEST_TIME_HIGH ONE_WIRE_PORT |= TEST_TIME_BITMASK;
#define TEST_TIME_RELEASE ONE_WIRE_DDR &= ~TEST_TIME_BITMASK;
#define TEST_TIME_PULL ONE_WIRE_DDR |= TEST_TIME_BITMASK;
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

const byte mac[] = {0xDE, 0xAD, MAC_4_LAST};
const IPAddress mqttServerIP(MQTT_SERVER_IP);
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

#ifdef WATCHDOG_EN
  Watchdog watchdog;
#endif

uint32_t mqttLastReconnectAttempt = 0;
bool mqttPublishReq = false;
bool mqttPublishTrig = false;

uint32_t dsLastRequest = 0;
uint16_t dsInterval = DS_INIT_TIME;
uint8_t dsRetryCount = 0;
uint8_t dsStatus = DS_REQUEST;
uint8_t dsIndex = DS_INDEX_INIT;
uint8_t dsDeviceCount;
int8_t prevDiscBitPos = -1;
bool dsReady = false;
bool dsError = false;
int16_t dsTemp[DS_MAX_DEVICE_COUNT];
uint8_t dsSort[DS_MAX_DEVICE_COUNT];
DsScratchPad dsScratchPad;

uint8_t mqttConnectAttempts0 = 0;
uint8_t mqttConnectAttempts1 = 0;

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
 * @return true Devices found
 * @return false No devices found
 */
inline bool oneWireReset(){
	uint8_t retryCount = 200;
  uint8_t releaseTime;

  ONE_WIRE_RELEASE; 

	// ensure the bus is high 
  while (!ONE_WIRE_SAMPLE){
    delayMicroseconds(20);
    retryCount--;
    if (!retryCount){
      return 0;
    }
  }
  ONE_WIRE_HIGH;  
  ONE_WIRE_PULL;
  delayMicroseconds(ONE_WIRE_PRE_RESET_STABILIZE_TIME);
  noInterrupts();  
  ONE_WIRE_LOW;
	delayMicroseconds(ONE_WIRE_RESET_PULSE_TIME);
  ONE_WIRE_HIGH; // ds responds after 15 to 60µS for 60µs to 240µs
  nops<CYCLES_MICROSEC * 10>();
  ONE_WIRE_RELEASE;
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_HIGH;
#endif
  delayMicroseconds(60);
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_LOW;
#endif 
  if (ONE_WIRE_SAMPLE){
    delayMicroseconds(240);
    ONE_WIRE_HIGH;
    ONE_WIRE_PULL;
    interrupts();
    delayMicroseconds(240 + ONE_WIRE_IDLE_TIME);
    return false; // no ds response
  }
  delayMicroseconds(240);
  for (releaseTime = ONE_WIRE_RESET_RELEASE_TIME; releaseTime; releaseTime--){
    if (ONE_WIRE_SAMPLE){
      break;
    }
    nops<(CYCLES_MICROSEC * 1) - 4>();      
  }
  ONE_WIRE_HIGH;
  ONE_WIRE_PULL;  
  interrupts();
  if (releaseTime){
    delayMicroseconds(releaseTime);
  }
  delayMicroseconds(ONE_WIRE_IDLE_TIME);
  return true;
}

inline void oneWireWriteBit(bool b){
  // slot 90µs + idle 20µs
  noInterrupts();
  ONE_WIRE_LOW;
  ONE_WIRE_PULL;
  if (b){
    nops<CYCLES_MICROSEC * ONE_WIRE_WRITE_INIT_TIME>();     
    ONE_WIRE_HIGH;
    interrupts();
    delayMicroseconds(ONE_WIRE_WRITE_ONE_TIME + ONE_WIRE_IDLE_TIME);
    return;
  }
  delayMicroseconds(ONE_WIRE_WRITE_INIT_TIME + ONE_WIRE_WRITE_ZERO_TIME);
  ONE_WIRE_HIGH;
  interrupts();
  delayMicroseconds(ONE_WIRE_IDLE_TIME); 
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
  ONE_WIRE_LOW;
  ONE_WIRE_PULL;
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_HIGH;
#endif
  nops<CYCLES_MICROSEC * ONE_WIRE_READ_INIT_TIME>(); 
  ONE_WIRE_RELEASE;
  ONE_WIRE_HIGH;
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_LOW;
#endif 
  nops<(CYCLES_MICROSEC * ONE_WIRE_READ_SAMPLE_TIME) - 4>();
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_HIGH;
#endif 
  if (ONE_WIRE_SAMPLE){
    b = true;
    ONE_WIRE_PULL;
  }
  for (releaseTime = ONE_WIRE_READ_RELEASE_TIME; releaseTime; releaseTime--){
    if (ONE_WIRE_SAMPLE){
      break;
    }
    nops<(CYCLES_MICROSEC * 1) - 4>();      
  }
  ONE_WIRE_PULL;
  interrupts();
  if (releaseTime){
    delayMicroseconds(releaseTime);      
  }
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_LOW;
#endif 
  delayMicroseconds(ONE_WIRE_IDLE_TIME);
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
  uint8_t eepromaAdrStart = EEPROM_DS_DEVICE_ADDRESSES + (dsIndex * ONE_WIRE_ADDRESS_BYTE_SIZE);

  oneWireWriteByte(DS_MATCH_ROM_COMMAND);

  for (uint8_t jjj = 0; jjj < ONE_WIRE_ADDRESS_BYTE_SIZE; jjj++){
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

  for (bitPos = 0; bitPos < ONE_WIRE_ADDRESS_BIT_SIZE; bitPos++){
    uint8_t romByteMask = 1 << (bitPos & 0x07);
    uint8_t romByteId = bitPos >> 3;

    if (romByteMask == 0x01){
      if (bitPos){
        SERIAL_PRINT_HEX(curRomByte);
        EEPROM.update(eepromCurRomByteIndex, curRomByte);
      }
      eepromCurRomByteIndex = EEPROM_DS_DEVICE_ADDRESSES + (dsIndex * ONE_WIRE_ADDRESS_BYTE_SIZE) + romByteId;
      curRomByte = 0x00;
      if (dsIndex){
        prevRomByte = EEPROM.read(eepromCurRomByteIndex - ONE_WIRE_ADDRESS_BYTE_SIZE);
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
      for (uint8_t jjj; jjj < ONE_WIRE_ADDRESS_BYTE_SIZE; jjj++){
        crc8 ^= EEPROM.read(jjj + EEPROM_DS_DEVICE_ADDRESSES + (dsIndex * ONE_WIRE_ADDRESS_BYTE_SIZE));
        crc8 = oneWireCrc8(crc8);
      }
      if (crc8){
        #ifdef SERIAL_EN
          Serial.println("CRC error");
        #endif
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
  #ifdef SERIAL_EN
  uint8_t iii;
  #endif

  if (strcmp(topic, TOPIC_SUB_REQ) == 0){
    #ifdef SERIAL_EN
      Serial.print("sub rec: ");
      Serial.print(topic);
      Serial.print(" : ");
      for (iii = 0; iii < length; iii++) {
        Serial.print((char) payload[iii]);
      }
    #endif
    mqttPublishReq = true;
  }
}

bool mqttReconnect() {
  String clientId = MQTT_CLIENT_ID_PREFIX;
  clientId += String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str())) {
    mqttClient.subscribe(TOPIC_SUB_REQ);
  }
  return mqttClient.connected();
}

bool publishTemp() {
  uint8_t jjj;
  String temp;
  uint32_t accTemp = 0;
  uint8_t accTempDiv = 0;
  String topic = TOPIC_PUB;

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

  for (jjj = 0; jjj < dsDeviceCount; jjj++){
    if (DS_DEVICE_SORT_SELECT & jjj){
      continue;
      accTemp += dsTemp[dsSort[jjj]];
      accTempDiv++;     
    }
  }

  temp = (String)(float) ((accTemp / accTempDiv) * DS_RAW_TO_C_MUL);

  #ifdef SERIAL_EN
    Serial.print("pub: ");
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(temp);
  #endif

  return mqttClient.publish(topic.c_str(), temp.c_str());
}

void setup() {
  delay(250);
  DDRD |= 1 << PWM_PD; // set to output
  analogWrite(PWM_PD, PWM_LEVEL); // pinMode output is included in this
  TCCR2B = (TCCR2B & 0xf0) | 0x01; // PWM freq 31kHz+
  DDRD &= B00111111; // PD7 & PD6 as input for comparator
  PORTD &= B00111111; // PD7 & PD6 no pull ups
  ADCSRB = 0x00; // disable MUX analog inputs for analog comparator.
  DIDR1 = 0x00; // disable PD7 & PD6 digital inputs
  ACSR = 0x00; // analog comparator enable, no interrupts, no capture
  delay(500);
  ONE_WIRE_HIGH;  
  ONE_WIRE_PULL;
#ifdef TEST_TIME_PIN_ENABLE
  TEST_TIME_LOW;
  TEST_TIME_PULL;
#endif
  mqttClient.setServer(mqttServerIP, 1883);
  mqttClient.setCallback(mqttCallback);
  Ethernet.init(ETHERNET_SELECT_PIN);
  SPI.begin();
  Ethernet.begin(mac);
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

  if (Ethernet.hardwareStatus() == EthernetHardwareStatus::EthernetNoHardware) {
    #ifdef SERIAL_EN
      Serial.println("ENC28J60 not found.");
    #endif
    while(1){
      delay(1);
    }
  }

  if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
    #ifdef SERIAL_EN
      Serial.println("Ethernet not connected.");
    #endif
  } else {
    #ifdef SERIAL_EN
      Serial.println("Ethernet ok.");
    #endif
  }

  dsLastRequest = millis();
}

void loop() {
  #ifdef WATCHDOG_EN
    watchdog.reset();
  #endif
  Ethernet.maintain();

  if (mqttClient.connected()) {
    mqttConnectAttempts0 = 0;
    mqttConnectAttempts1 = 0;
    if (dsReady){
      if (mqttPublishReq){
        publishTemp();
        mqttPublishReq = false;
      } else if (mqttPublishTrig) {
        publishTemp();      
        mqttPublishTrig = false;
      }
    }
    mqttClient.loop();
  } else {
    if (!(mqttConnectAttempts0 || mqttConnectAttempts1)){
      #ifdef SERIAL_EN
        Serial.print("Attempting MQTT connection");
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

  if (dsDeviceCount && (millis() - dsLastRequest > dsInterval)) {
    if (dsStatus & DS_READ){
      if (!oneWireReset()){
        DS_NEXT(DS_CONNECTION_ERROR, DS_CONNECTION_ERROR_TIME);
      }

      oneWireRomSelect();

      if (!oneWireReadDsScratchPath()){

        #ifdef SERIAL_EN
          Serial.print("ERR");
          Serial.print(dsRetryCount);
          Serial.print(" ");
        #endif

        dsRetryCount++;

        if (dsRetryCount >= DS_MAX_RETRY){
          DS_NEXT(DS_FAIL, DS_RETRY_TIME);
        } 
        if (dsRetryCount & LOW_NIBBLE){
          DS_NEXT(DS_READ, DS_RETRY_TIME + (dsRetryCount * DS_RETRY_INCREASE_TIME));
        } 
        DS_NEXT(DS_REQUEST, DS_RETRY_TIME + (dsRetryCount * DS_RETRY_INCREASE_TIME));
      }

      dsTemp[dsIndex] = (((int16_t) dsScratchPad[DS_SCRATCHPAD_TEMP_MSB]) << 11)
        | (((int16_t) dsScratchPad[DS_SCRATCHPAD_TEMP_LSB]) << 3);

      if (!dsIndex){
        #ifdef SERIAL_EN
        Serial.print("*");
        Serial.print((float) dsTemp[dsIndex] * DS_RAW_TO_C_MUL);
        Serial.print("* ");
        #endif
      }

      #ifdef SERIAL_EN
        Serial.print(dsTemp[dsIndex]);
        Serial.print(", ");
      #endif

      DS_NEXT(DS_INDEX, DS_READ_TIME);

    } else if (dsStatus & DS_FAIL){

      dsError = true;
      dsReady = false;
      DS_NEXT(DS_INDEX, DS_FAIL_TIME);
  
    } else if (dsStatus & DS_INDEX) {

      dsRetryCount = 0;
      dsIndex++;

      if (dsIndex >= dsDeviceCount){       
        dsIndex = 0;
        if (!dsError){
          dsReady = true;
          mqttPublishTrig = true;
        }
        dsError = false;
      }

      DS_NEXT(DS_REQUEST, DS_INDEX_TIME);

    } else if (dsStatus & DS_REQUEST) {
     
      #ifdef SERIAL_EN
        if (!dsRetryCount){
          Serial.print("i");
          Serial.print(dsIndex);
          Serial.print(": ");
        }
      #endif

      if (!oneWireReset()){
        DS_NEXT(DS_CONNECTION_ERROR, DS_CONNECTION_ERROR_TIME);
      }

      oneWireRomSelect();
      oneWireWriteByte(DS_CONVERT_TEMP_COMMAND);

      #ifdef SERIAL_EN
        if (dsRetryCount){
          Serial.print("REQ");
          Serial.print(dsRetryCount / 16);
          Serial.print(" ");
        }
      #endif

      DS_NEXT(DS_READ, DS_REQUEST_TIME + (dsRetryCount * DS_RETRY_INCREASE_TIME));

    } else if (dsStatus & DS_CONNECTION_ERROR){

      DS_NEXT(DS_REQUEST, DS_RETRY_AFTER_CONNECTION_ERROR_TIME);
    }

    dsLastRequest = millis();
  }
}
