#include <Arduino.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <EthernetENC.h>
#include <Watchdog.h>
#include <.env.h>

#define ETHERNET_SELECT_PIN 10
#define SERIAL_BAUD 115200
#define DS_READ_TEST

#define TOPIC_SUB_TEMP_REQ "water:req"
#define TOPIC_PUB_TEMP_RESP "water:resp"
#define TOPIC_PUB_TEMP_CONTINOUS "water:cont"

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
#define DS_DEVICE_COUNT 8
#define DS_DEVICE_SORT_SELECT B00011000
#define DS_ADDRESS_SIZE 8

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

#define DS_NEXT(DS_STATUS, DS_INTERVAL)\        
  dsStatus = DS_STATUS;\
  dsInterval = DS_INTERVAL;\
  dsLastRequest = millis();\
  return;

// See https://stackoverflow.com/a/63468969
template< unsigned N > 
inline static void nops(){
  asm ("nop");
  nops< N - 1 >();
}
template<> inline void nops<0>(){};

const byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
const IPAddress ip(192, 168, 0, 80);
const IPAddress mqttServerIP(192, 168, 0, 100);
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

Watchdog watchdog;

uint32_t mqttLastReconnectAttempt = 0;
bool mqttPublishReq = false;
bool mqttPublishTrig = false;

uint32_t dsLastRequest = 0;
uint16_t dsInterval = DS_INIT_TIME;
uint8_t dsRetryCount = 0;
uint8_t dsStatus = DS_REQUEST;
uint8_t dsIndex = DS_INDEX_INIT;
bool dsReady = false;
bool dsError = false;
int16_t dsTemp[DS_DEVICE_COUNT];
int16_t workAry[DS_DEVICE_COUNT];
uint8_t dsSort[DS_DEVICE_COUNT];
DsScratchPad dsScratchPad;

const uint8_t* adr;

const PROGMEM uint8_t dsProgMemAddr[] = {
  0x28, 0x18, 0x39, 0x15, 0x12, 0x21, 0x01, 0x74,
  0x28, 0xDC, 0x17, 0xCA, 0x11, 0x21, 0x01, 0x12,
  0x28, 0x22, 0x0C, 0x11, 0x12, 0x21, 0x01, 0x0E,
  0x28, 0x32, 0xDB, 0x09, 0x12, 0x21, 0x01, 0x7E,
  0x28, 0xF1, 0x3F, 0x00, 0x12, 0x21, 0x01, 0xEE,
  0x28, 0xB9, 0xC1, 0x10, 0x12, 0x21, 0x01, 0x06,
  0x28, 0xAD, 0x0A, 0xBA, 0x11, 0x21, 0x01, 0x85,
  0x28, 0x2B, 0x9F, 0x04, 0x12, 0x21, 0x01, 0xD8
};

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

inline uint8_t oneWireReset(){
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
    return 0; // no ds response
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
  return 1;
}

inline void oneWireWrite(uint8_t v){
  uint8_t bitMask = 0x01;
  // slot 90µs + idle 20µs
  for(bitMask = 0x01; bitMask; bitMask <<= 1){
    noInterrupts();
    ONE_WIRE_LOW;
    ONE_WIRE_PULL;
    if (v & bitMask){
      nops<CYCLES_MICROSEC * ONE_WIRE_WRITE_INIT_TIME>();     
      ONE_WIRE_HIGH;
      interrupts();
      delayMicroseconds(ONE_WIRE_WRITE_ONE_TIME + ONE_WIRE_IDLE_TIME);
      continue;
    }
    delayMicroseconds(ONE_WIRE_WRITE_INIT_TIME + ONE_WIRE_WRITE_ZERO_TIME);
    ONE_WIRE_HIGH;
    interrupts();
    delayMicroseconds(ONE_WIRE_IDLE_TIME);      
  }
}

inline uint8_t oneWireRead(){
  uint8_t bitMask;
  uint8_t v = 0x00;
  uint8_t releaseTime;

  for (bitMask = 0x01; bitMask; bitMask <<= 1){
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
      v |= bitMask;
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
  }
  return v;
}

inline void oneWireRomSelect(){
  uint8_t i;
  const uint8_t* adr;

  oneWireWrite(DS_MATCH_ROM_COMMAND);
  adr = dsProgMemAddr + (dsIndex * DS_ADDRESS_SIZE);
  for (i = 0; i < DS_ADDRESS_SIZE; i++){
    oneWireWrite(pgm_read_byte_near(adr + i));
  }
}

inline uint8_t oneWireReadDsScratchPath(){
  uint8_t i;
  uint8_t crc = 0;

  oneWireWrite(DS_READ_SCRATCHPAD_COMMAND);
  for (i = 0; i < DS_SCRATCHPAD_SIZE; i++){
    dsScratchPad[i] = oneWireRead();
  }
  for (i = 0; i < DS_SCRATCHPAD_SIZE; i++){
		if (dsScratchPad[i] != 0) {
			break;
		}
    return 0;
	}
  for(i = 0; i < DS_SCRATCHPAD_SIZE - 1; i++){
    crc ^= dsScratchPad[i];
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
		  pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
  }
  if (crc == dsScratchPad[DS_SCRATCHPAD_CRC]){
    return 1;
  }
  return 0;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  uint8_t iii;

  if (topic == TOPIC_SUB_TEMP_REQ){
    Serial.print("sub rec: ");
    Serial.print(topic);
    Serial.print(" : ");
    for (iii = 0; iii < length; iii++) {
      Serial.print((char) payload[iii]);
    }
    mqttPublishReq = true;
  }
}

bool mqttReconnect() {
  String clientId = "temp_sens_";
  clientId += String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str())) {
    mqttClient.subscribe(TOPIC_SUB_TEMP_REQ);
  }
  return mqttClient.connected();
}

bool publishTemp(String topic) {
  uint8_t jjj;
  String temp;
  uint32_t accTemp = 0;
  uint8_t accTempDiv = 0;

  for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
    dsSort[jjj] = jjj;
  }

  for (jjj = 0; jjj < (DS_DEVICE_COUNT - 1); jjj++){
    if (dsTemp[dsSort[jjj]] > dsTemp[dsSort[jjj + 1]]){
      dsSort[jjj] ^= dsSort[jjj + 1];
      dsSort[jjj + 1] ^= dsSort[jjj];
      dsSort[jjj] ^= dsSort[jjj + 1];
    }
  }

  for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
    if (DS_DEVICE_SORT_SELECT & jjj){
      continue;
      accTemp += dsTemp[dsSort[jjj]];
      accTempDiv++;     
    }
  }

  temp = (String)(float) ((accTemp / accTempDiv) * DS_RAW_TO_C_MUL);
  Serial.print("pub: ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(temp);

  return mqttClient.publish(topic.c_str(), temp.c_str());
}

void setup() {
  delay(500);
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
  Ethernet.begin(mac, ip);
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  delay(250);
  
  // set temperature resolution to 12 bit
  for (dsIndex = 0; dsIndex < DS_DEVICE_COUNT; dsIndex++){
    oneWireReadDsScratchPath();
    if (dsScratchPad[DS_SCRATCHPAD_CONFIG] == DS_CONFIG_12_BIT){
      delay(10);
      continue;
    }
    oneWireReset();
    oneWireRomSelect();
    oneWireWrite(DS_WRITE_SCRATCHPAD_COMMAND);
    oneWireWrite(dsScratchPad[DS_SCRATCHPAD_REG_TH]);
    oneWireWrite(dsScratchPad[DS_SCRATCHPAD_REG_TL]);
    oneWireWrite(DS_CONFIG_12_BIT);
    delay(10);
    oneWireReset();
    oneWireRomSelect();
    oneWireWrite(DS_COPY_SCRATCHPAD_COMMAND);
    delay(25);
  }
  dsIndex = DS_INDEX_INIT;

  if (Ethernet.hardwareStatus() == EthernetHardwareStatus::EthernetNoHardware) {
    Serial.println("ENC28J60 not found.");
    while(1){
      delay(1);
    }
  }

  if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
    Serial.println("Ethernet not connected.");
  } else {
    Serial.println("Ethernet ok.");
  }

  delay(500);
  dsLastRequest = millis();
  watchdog.enable(Watchdog::TIMEOUT_4S);
}

void loop() {
  watchdog.reset();
  Ethernet.maintain();

  if (mqttClient.connected()) {
    if (dsReady){
      if (mqttPublishReq){
        publishTemp(TOPIC_PUB_TEMP_RESP);
        mqttPublishReq = false;
      } else if (mqttPublishTrig) {
        publishTemp(TOPIC_PUB_TEMP_CONTINOUS);      
        mqttPublishTrig = false;
      }
    }
    mqttClient.loop();
  } else {
    Serial.print("Attempting MQTT connection... ");
    if (millis() - mqttLastReconnectAttempt > MQTT_CONNECT_RETRY_TIME) {
      if (mqttReconnect()) {
        Serial.println("connected");
        mqttLastReconnectAttempt = 0;
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.print(" try again in ");
        Serial.print(MQTT_CONNECT_RETRY_TIME);
        Serial.println(" ms.");
        mqttLastReconnectAttempt = millis();
      }
    }
  }

  if (DS_DEVICE_COUNT && (millis() - dsLastRequest > dsInterval)) {
    if (dsStatus & DS_READ){
      if (!oneWireReset()){
        DS_NEXT(DS_CONNECTION_ERROR, DS_CONNECTION_ERROR_TIME);
      }

      oneWireRomSelect();

      if (!oneWireReadDsScratchPath()){

        Serial.print("ERR");
        Serial.print(dsRetryCount);
        Serial.print(" ");

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
        Serial.print("*");
        Serial.print((float) dsTemp[dsIndex] * DS_RAW_TO_C_MUL);
        Serial.print("* ");
      }

      Serial.print(dsTemp[dsIndex]);
      Serial.print(", ");

      DS_NEXT(DS_INDEX, DS_READ_TIME);

    } else if (dsStatus & DS_FAIL){

      dsError = true;
      dsReady = false;
      DS_NEXT(DS_INDEX, DS_FAIL_TIME);
  
    } else if (dsStatus & DS_INDEX) {

      dsRetryCount = 0;
      dsIndex++;

      if (dsIndex >= DS_DEVICE_COUNT){       
        dsIndex = 0;
        if (!dsError){
          dsReady = true;
          mqttPublishTrig = true;
        }
        dsError = false;
      }

      DS_NEXT(DS_REQUEST, DS_INDEX_TIME);

    } else if (dsStatus & DS_REQUEST) {
     
      if (!dsRetryCount){
        Serial.print("i");
        Serial.print(dsIndex);
        Serial.print(": ");
      }

      if (!oneWireReset()){
        DS_NEXT(DS_CONNECTION_ERROR, DS_CONNECTION_ERROR_TIME);
      }

      oneWireRomSelect();
      oneWireWrite(DS_CONVERT_TEMP_COMMAND);

      if (dsRetryCount){
        Serial.print("REQ");
        Serial.print(dsRetryCount / 16);
        Serial.print(" ");
      }

      DS_NEXT(DS_READ, DS_REQUEST_TIME + (dsRetryCount * DS_RETRY_INCREASE_TIME));

    } else if (dsStatus & DS_CONNECTION_ERROR){

      DS_NEXT(DS_REQUEST, DS_RETRY_AFTER_CONNECTION_ERROR_TIME);
    }

    dsLastRequest = millis();
  }
}
