#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>
#include <EthernetENC.h>
#include <DallasTemperature.h>
#include <Watchdog.h>

#define SERIAL_BAUD 115200
// #define SERIAL_EN
// #define DEBUG_SERVER
// #define SEARCH_DS_ADDR
#define DS_READ_TEST

#define EEPROM_BOOT_COUNT 0x00
// #define SET_BOOT_COUNT 0
#define EEPROM_ERROR_COUNT 0x20
#define EEPROM_FAIL_COUNT 0x40
// #define SET_ERROR_AND_FAIL_COUNTS 0

#define DS_BUFF_SIZE 128
#define DS_TEMP_LSB 0
#define DS_TEMP_MSB 1
#define ONE_WIRE_BUS 2
#define DS_RAW_TO_C_MUL 0.0078125f
#define DS_FILTER_ADAPT_STEP 4

#define DS_INIT_TIME 2000
#define DS_REQUEST_TIME 800
#define DS_RETRY_REQUEST_TIME 900
#define DS_READ_TIME 100
#define DS_RETRY_TIME 150
#define DS_FAIL_TIME 50
#define DS_INDEX_TIME 100
#define DS_MAX_RETRY 64
#define DS_RETRY_INCREASE_TIME 25

#define DS_DEVICE_COUNT 8
#define DS_ADDRESS_SIZE 8

#define DS_REQUEST 0x01
#define DS_READ 0x02
#define DS_FAIL 0x04
#define DS_INDEX 0x08

#define LO_NIBBLE 0x0f
#define HI_NIBBLE 0xf0

#define PAGE_PREVIOUS_CHAR_INIT 0x00
#define PAGE_CHAR_INIT 0x00

#define PAGE_CHAR_POS_INIT 0x00
#define PAGE_CHAR_POS_MAX 0xff

#define PAGE_HISTORY_SIZE_NONE 0x00
#define PAGE_HISTORY_SIZE_DEFAULT DS_BUFF_SIZE

#define PAGE_SENSORS_NONE 0x00
#define PAGE_SENSORS_DEFAULT 0x18

#define PAGE_PRECISION_DEFAULT 2

#define PAGE_SWITCH_SORTED 0x80
#define PAGE_SWITCH_SORT_SENSORS 0x40
#define PAGE_SWITCH_FIRST_ITERATION 0x20
#define PAGE_SWITCH_GET_PRECISION 0x08
#define PAGE_SWITCH_GET_HISTORY_SIZE 0x04
#define PAGE_SWITCH_GET_PATH 0x02
#define PAGE_SWITCH_BLANK_LINE 0x01
#define PAGE_SWITCHES_INIT 0x00;

#define SERV_ERROR_SERVICE_UNAVAILABLE 0x04
#define SERV_ERROR_NOT_FOUND 0x02
#define SERV_ERROR_BAD_REQUEST 0x01
#define SERV_ERROR_NONE 0x00

#define SERV_TEXT 0x80
#define SERV_AVG 0x40
#define SERV_SENSOR_ERRORS 0x20
#define SERV_HISTORY 0x10
#define SERV_DELTA 0x08
#define SERV_LOG 0x04
#define SERV_SENSORS 0x02
#define SERV_ERR 0x01
#define SERV_UNDEFINED 0x00

#define SERV_CHECK_CONN_INTERVAL 60000

#define ACC_COUNT_RESET 0
#define ACC_RESET 0

typedef uint8_t ScratchPad[9];

const byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// const IPAddress ip(10, 200, 125, 80);
const IPAddress ip(192, 168, 1, 180);
EthernetServer server(80);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds(&oneWire);

Watchdog watchdog;

uint32_t servLastCheck = 0;
uint32_t dsLastRequest = 0;
uint16_t dsInterval = DS_INIT_TIME;
uint32_t dsCycleCount = 0;
uint8_t dsRetryCount = 0;
uint8_t dsStatus = DS_REQUEST;

uint8_t dsIndex = 0;
int16_t dsTemp[DS_DEVICE_COUNT];
int16_t dsBufferedTemp[DS_DEVICE_COUNT];
int16_t workAry[DS_DEVICE_COUNT];
uint8_t dsSort[DS_DEVICE_COUNT];
uint8_t dsWeightCount[DS_DEVICE_COUNT];
uint8_t dsBuffIndex = 0;
int8_t dsBuffChange[DS_BUFF_SIZE];
int8_t dsBuffAmount[DS_BUFF_SIZE];
uint16_t bootCount;

int16_t i16;
int32_t i32;
uint16_t ui16;

uint8_t aaa;
uint8_t iii;
uint8_t jjj;

uint8_t readSensorIndex;
uint8_t readBuffIndex;
uint8_t prevClientChar;
uint8_t clientChar;
uint8_t clientCharPos;
uint8_t historySize;
uint8_t sensors;
uint8_t precision;
uint8_t pageSwitch;
uint8_t serv;
uint8_t servError;

const uint8_t* adr;
DeviceAddress dsAddr;

#ifndef SEARCH_DS_ADDR
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
#endif 

inline void setPrevReadBuffIndex(){
  if (!readSensorIndex){
    readSensorIndex = DS_DEVICE_COUNT;
    if (!readBuffIndex){
      readBuffIndex = DS_BUFF_SIZE;
    }
    readBuffIndex--;
  }
  readSensorIndex--;
}

inline void swapSort(){
  dsSort[jjj] ^= dsSort[jjj + 1];
  dsSort[jjj + 1] ^= dsSort[jjj];
  dsSort[jjj] ^= dsSort[jjj + 1];
  pageSwitch &= ~PAGE_SWITCH_SORTED; 
}

void setup() {
  delay(1000);
  Ethernet.init(10);
  SPI.begin();
  Ethernet.begin(mac, ip);
  #ifdef SERIAL_EN
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.print("Dallas temp lib version: ");
    Serial.println(DALLASTEMPLIBVERSION);
    Serial.println();
  #endif
  oneWire.depower();
  ds.begin();
  
  #ifdef SEARCH_DS_ADDR
    aaa = ds.getDS18Count();
    Serial.println("{");
    for (iii = 0; iii < aaa; iii++){
      ds.getAddress(dsAddr, iii);
      Serial.print("{");
      for (jjj = 0; jjj < 8; jjj++){
        Serial.print("0x");
        if ((dsAddr)[jjj] < 16){
          Serial.print(0);
        }
        Serial.print((dsAddr)[jjj], HEX);
        if (jjj < 7){
          Serial.print(", ");
        }    
      }

      Serial.print("}");
      if (iii < 7){
        Serial.print(",");
      }
      Serial.println();    
      ds.setResolution(dsAddr, 12);
    }
    Serial.println("}");
    Serial.print("Device count:");
    Serial.println(aaa);
  #else
    for (iii = 0; iii < DS_DEVICE_COUNT; iii++){
      adr = dsProgMemAddr + (iii * DS_ADDRESS_SIZE);
      #ifdef SERIAL_EN
        Serial.println();
        Serial.print(iii);
        Serial.print(": ");
      #endif 
      for (jjj = 0; jjj < DS_ADDRESS_SIZE; jjj++){
        (dsAddr)[jjj] = pgm_read_byte_near(adr + jjj);
        #ifdef SERIAL_EN
          if ((dsAddr)[jjj] < 16){
            Serial.print(0);
          }
          Serial.print((dsAddr)[jjj], HEX);
          if (jjj < 7){
            Serial.print(", ");
          }
        #endif
      }
      ds.setResolution(dsAddr, 12);    
    }
    #ifdef SERIAL_EN
      Serial.println(); 
    #endif 
  #endif

  ds.setWaitForConversion(false);
  dsLastRequest = millis();

  if (Ethernet.hardwareStatus() == EthernetHardwareStatus::EthernetNoHardware) {
    #ifdef SERIAL_EN
      Serial.println("ENC28J60 not found.");
    #endif
    while(1){
      delay(1);
    }
  }

  #ifdef SERIAL_EN
    if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
      Serial.println("Ethernet not connected.");
    } else {
      Serial.println("Ethernet ok.");
    }
  #endif 

  server.begin();

  #if !defined(SEARCH_DS_ADDR) && defined(SERIAL_EN)
    Serial.println();
    Serial.print("Device count: ");
    Serial.print(DS_DEVICE_COUNT);
    Serial.print(", Buffersize: ");
    Serial.println(DS_BUFF_SIZE);
  #endif

  #ifdef SET_ERROR_AND_FAIL_COUNTS
    ui16 = SET_ERROR_AND_FAIL_COUNTS;
    for (iii = 0; iii < DS_DEVICE_COUNT; iii++){
      EEPROM.put(EEPROM_ERROR_COUNT + (iii * 4), ui16);
      EEPROM.put(EEPROM_FAIL_COUNT + (iii * 4), ui16);
    }
    #ifdef SERIAL_EN
      Serial.print("Set sensor error ");
      Serial.print("and fail counts to ");
      Serial.println(SET_ERROR_AND_FAIL_COUNTS);
    #endif
  #endif 

  #ifdef SET_BOOT_COUNT
    bootCount = SET_BOOT_COUNT;
  #else
    EEPROM.get(EEPROM_BOOT_COUNT, bootCount);
    bootCount++;   
  #endif

  EEPROM.put(EEPROM_BOOT_COUNT, bootCount);

  #ifdef SERIAL_EN
    Serial.print("Boot count: ");
    Serial.println(bootCount);
  #endif

  delay(1000);

  watchdog.enable(Watchdog::TIMEOUT_2S);
}

void loop() {
  watchdog.reset();

  #ifndef SEARCH_DS_ADDR
  if (DS_DEVICE_COUNT && (millis() - dsLastRequest > dsInterval)) {
    if (dsStatus & DS_READ){
      ScratchPad scratchPad;

      if (ds.isConnected(dsAddr, scratchPad)) {

        dsTemp[dsIndex] = (((int16_t) scratchPad[DS_TEMP_MSB]) << 11)
          | (((int16_t) scratchPad[DS_TEMP_LSB]) << 3);

        #ifdef SERIAL_EN
          if (!dsIndex){ 
            Serial.print("*");
            Serial.print((float) dsTemp[dsIndex] * DS_RAW_TO_C_MUL);
            Serial.print("* ");
          }
        #endif

        if (dsCycleCount){
          if (dsTemp[dsIndex] == dsBufferedTemp[dsIndex]){
            bitClear(dsBuffChange[dsBuffIndex], dsIndex);
          } else if (dsTemp[dsIndex] < dsBufferedTemp[dsIndex]){
            dsBufferedTemp[dsIndex] -= DS_FILTER_ADAPT_STEP;
            bitSet(dsBuffChange[dsBuffIndex], dsIndex);
            bitClear(dsBuffAmount[dsBuffIndex], dsIndex);  
          } else {
            dsBufferedTemp[dsIndex] += DS_FILTER_ADAPT_STEP;
            bitSet(dsBuffChange[dsBuffIndex], dsIndex);
            bitSet(dsBuffAmount[dsBuffIndex], dsIndex);             
          }
        } else {
          dsBufferedTemp[dsIndex] = dsTemp[dsIndex];
          bitClear(dsBuffChange[dsBuffIndex], dsIndex);
        }

        #ifdef SERIAL_EN
          Serial.print(dsTemp[dsIndex]);
          if (dsCycleCount){
            Serial.print(" (");
            if (bit_is_set(dsBuffChange[dsBuffIndex], dsIndex)){
              if (bit_is_set(dsBuffAmount[dsBuffIndex], dsIndex)){
                Serial.print("+");
              } else {
                Serial.print("-");
              }
              Serial.print(DS_FILTER_ADAPT_STEP);
            } else {
              Serial.print("0");            
            }
            Serial.print(")");
          }
          Serial.print(", ");
        #endif

        dsStatus = DS_INDEX;
        dsInterval = DS_READ_TIME;
      } else {

        #ifdef SERIAL_EN
          Serial.print("ERR");
          Serial.print(dsRetryCount);
          Serial.print(" ");
        #endif

        dsRetryCount++;

        aaa = EEPROM_ERROR_COUNT + (dsIndex * 4);
        EEPROM.get(aaa, ui16);
        ui16++;
        EEPROM.put(aaa, ui16);

        if (dsRetryCount >= DS_MAX_RETRY){
          dsStatus = DS_FAIL;      
          dsInterval = DS_RETRY_TIME;
        } else {
          if (dsRetryCount & LO_NIBBLE){
            dsStatus = DS_READ;
          } else {
            dsStatus = DS_REQUEST;            
          }
          dsInterval = DS_RETRY_TIME + (dsRetryCount * DS_RETRY_INCREASE_TIME);
        }
      }

    } else if (dsStatus & DS_FAIL){

      // keep same temperature
      bitClear(dsBuffChange[dsBuffIndex], dsIndex);

      aaa = EEPROM_FAIL_COUNT + (dsIndex * 4);
      EEPROM.get(aaa, ui16);
      ui16++;
      EEPROM.put(aaa, ui16);

      dsStatus = DS_INDEX;
      dsInterval = DS_FAIL_TIME; 
   
    } else if (dsStatus & DS_INDEX) {

      dsRetryCount = 0;
      dsIndex++;

      if (dsIndex >= DS_DEVICE_COUNT){
        dsCycleCount++;        
        dsIndex = 0;
        dsBuffIndex++;

        if (dsBuffIndex >= DS_BUFF_SIZE){
          dsBuffIndex = 0;

          #ifdef SERIAL_EN
            Serial.println("BUFF0 ");
          #endif
        }
      }

      dsInterval = DS_INDEX_TIME;
      dsStatus = DS_REQUEST;

    } else if (dsStatus & DS_REQUEST) {

      #ifdef SERIAL_EN      
        if (!dsRetryCount){
          if (!dsIndex){
            Serial.println();
            Serial.print("Cycle #");
            Serial.print(dsCycleCount);
            Serial.print(" ");           
          }

          Serial.print("i");
          Serial.print(dsIndex);
          Serial.print(": ");
        }
      #endif

      adr = dsProgMemAddr + (dsIndex * DS_ADDRESS_SIZE);
      for (aaa = 0; aaa < DS_ADDRESS_SIZE; aaa++){
        (dsAddr)[aaa] = pgm_read_byte_near(adr + aaa);
      }

      ds.requestTemperaturesByAddress(dsAddr);
      
      #ifdef SERIAL_EN
        if (dsRetryCount){
          Serial.print("REQ");
          Serial.print(dsRetryCount / 16);
          Serial.print(" ");
        } 
      #endif

      dsInterval = DS_REQUEST_TIME + (dsRetryCount % DS_RETRY_INCREASE_TIME);
      dsStatus = DS_READ;
    }

    dsLastRequest = millis();
  }

  /**
   * Listen ethernet clients
   */

  if (millis() - servLastCheck > SERV_CHECK_CONN_INTERVAL) {
    if (Ethernet.linkStatus() == EthernetLinkStatus::LinkOFF) {
      #ifdef SERIAL_EN
        Serial.print("Ethernet ");
        Serial.print("not connected, ");
        Serial.println("reset shield.");
      #endif
      Ethernet.begin(mac, ip);      
    } else {
      #ifdef SERIAL_EN
        Serial.print("{>Eth.ok<}");
      #endif
    }
    servLastCheck = millis();
  }

  EthernetClient client = server.available();

  if (!client) {
    return;
  }

  #ifdef SERIAL_EN
    Serial.println("HTTP ");
    Serial.println("client >>");
  #endif

  prevClientChar = PAGE_PREVIOUS_CHAR_INIT;
  clientChar = PAGE_CHAR_INIT;
  clientCharPos = PAGE_CHAR_POS_INIT;
  historySize = PAGE_HISTORY_SIZE_NONE;
  sensors = PAGE_SENSORS_NONE;
  precision = PAGE_PRECISION_DEFAULT;
  pageSwitch = PAGE_SWITCHES_INIT;
  serv = SERV_UNDEFINED;
  servError = SERV_ERROR_NONE;

  while (client.connected()) {
    if (!client.available()) {
      continue;
    }

    clientChar = client.read();

    if (clientCharPos < PAGE_CHAR_POS_MAX){
      clientCharPos++;      
    }

    #ifdef SERIAL_EN
      Serial.print((char) clientChar);
    #endif

    if (pageSwitch & PAGE_SWITCH_GET_PATH){
      switch (clientChar){
        case ' ':
          pageSwitch &= ~PAGE_SWITCH_GET_PATH;
          serv |= SERV_SENSORS;
          break;
        case '/':
          pageSwitch &= ~PAGE_SWITCH_GET_HISTORY_SIZE;
          pageSwitch &= ~PAGE_SWITCH_GET_PRECISION;
          break;
        case 'a':
          pageSwitch |= PAGE_SWITCH_GET_HISTORY_SIZE;
          serv |= SERV_AVG;
          break;
        case 't':
          serv |= SERV_TEXT;
          break;
        case 'e':
          serv |= SERV_SENSOR_ERRORS;
          break;
        case 's':
          pageSwitch |= PAGE_SWITCH_SORT_SENSORS;
          break;
        case 'p':
          pageSwitch |= PAGE_SWITCH_GET_PRECISION;
          break;
        case 'h':
          pageSwitch |= PAGE_SWITCH_GET_HISTORY_SIZE;
          serv |= SERV_HISTORY;
          break;
        case 'd':
          pageSwitch |= PAGE_SWITCH_GET_HISTORY_SIZE;
          serv |= SERV_DELTA;
          break;
        #ifdef EEPROM_LOG
          case 'l':
            serv |= SERV_LOG;
            break;
        #endif 
        #ifdef DEBUG_SERVER
          case 'n':
            serv |= SERV_ERR;
            servError |= SERV_ERROR_NOT_FOUND;
            break;
          case 'u':
            serv |= SERV_ERR;
            servError |= SERV_ERROR_SERVICE_UNAVAILABLE;
            break;
          case 'b':
            serv |= SERV_ERR;
            servError |= SERV_ERROR_BAD_REQUEST;
            break;
          case 'i':
            serv |= SERV_ERR;
            break;
        #endif
        case '0' ... '9':
          if (pageSwitch & PAGE_SWITCH_GET_PRECISION){
            pageSwitch &= ~PAGE_SWITCH_GET_PRECISION;
            pageSwitch &= ~PAGE_SWITCH_GET_HISTORY_SIZE;
            if (clientChar > '3'){
              break;
            }
            precision = clientChar - '0';
            break;
          }
          if (pageSwitch & PAGE_SWITCH_GET_HISTORY_SIZE){
            if (historySize == PAGE_HISTORY_SIZE_NONE){
              historySize = clientChar - '0';
              break;
            }
            historySize = (historySize * 10) + (clientChar - '0');
            if (historySize > 99){
              pageSwitch &= ~PAGE_SWITCH_GET_HISTORY_SIZE;              
            }
            break; 
          }
          if (clientChar > '7'){
            break;
          }
          sensors |= 1 << (clientChar - '0');
          break;
        default:
          break;
      }
    } else if (serv == SERV_UNDEFINED){
      if (clientCharPos < 5){
        if (clientChar == "_GET "[clientCharPos]) continue;
      } else if (clientChar == '/'){
          pageSwitch |= PAGE_SWITCH_GET_PATH;
          continue;
      }

      serv |= SERV_ERR;
      servError = SERV_ERROR_BAD_REQUEST;        
    }

    if (prevClientChar == '\n'){
      pageSwitch |= PAGE_SWITCH_BLANK_LINE;
    } else if (prevClientChar != '\r') {
      pageSwitch &= ~PAGE_SWITCH_BLANK_LINE;
    }

    prevClientChar = clientChar;

    if (clientChar != '\n'){
      continue;
    }    

    if (~pageSwitch & PAGE_SWITCH_BLANK_LINE){
      continue;
    }

    /**
     * Serve page to client
     */

    if (sensors == PAGE_SENSORS_NONE){
      sensors = PAGE_SENSORS_DEFAULT;
    }

    if (historySize == PAGE_HISTORY_SIZE_NONE){
      historySize = PAGE_HISTORY_SIZE_DEFAULT;
    } else if (historySize > DS_BUFF_SIZE){
      historySize = PAGE_HISTORY_SIZE_DEFAULT;
    }

    if (historySize > dsCycleCount){
      // prevent buffer overflow
      historySize = dsCycleCount;
    }

    if (serv & SERV_AVG){
      for (iii = 0; iii < DS_DEVICE_COUNT; iii++){
        workAry[iii] = dsBufferedTemp[iii];
        dsWeightCount[iii] = 0;
      }

      readBuffIndex = dsBuffIndex;
      readSensorIndex = dsIndex;
   
      if (!(dsStatus && DS_INDEX)){
        setPrevReadBuffIndex();
      }

      i16 = ACC_COUNT_RESET;
      i32 = ACC_RESET;

      for (iii = 0; iii < historySize; iii++){
        // sort if required
        if (!iii || pageSwitch & PAGE_SWITCH_SORT_SENSORS){
          for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
            dsSort[jjj] = jjj;
          }          
        }

        if (pageSwitch & PAGE_SWITCH_SORT_SENSORS){
          pageSwitch &= ~PAGE_SWITCH_SORTED;
          while (!(pageSwitch & PAGE_SWITCH_SORTED)){
            pageSwitch |= PAGE_SWITCH_SORTED;
            for (jjj = 0; jjj < (DS_DEVICE_COUNT - 1); jjj++){
              if (workAry[dsSort[jjj]] > workAry[dsSort[jjj + 1]]){
                swapSort();
              }
            }
          }
        }

        for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
          // accumulate 
          if (bit_is_clear(sensors, jjj)){
            continue;
          }
          i32 += workAry[dsSort[jjj]];
          i16++;
          dsWeightCount[dsSort[jjj]]++;
        }

        for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
          // get previous sensor temperatures from delta buffer
          if (bit_is_set(dsBuffChange[readBuffIndex], readSensorIndex)){
            if (bit_is_set(dsBuffAmount[readBuffIndex], readSensorIndex)){
              workAry[readSensorIndex] -= DS_FILTER_ADAPT_STEP;
            } else {
              workAry[readSensorIndex] += DS_FILTER_ADAPT_STEP;
            }
          }

          setPrevReadBuffIndex();
        }
      }

      #ifdef SERIAL_EN
        Serial.print("-- avg");
        if (pageSwitch & PAGE_SWITCH_SORT_SENSORS){
          Serial.print(", sorted");
        }
        Serial.println(" --");
        Serial.print("acc: ");
        Serial.print(i32);
        Serial.print(", n: ");
        Serial.print(i16);
        Serial.print(", used: ");
        for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
          if (dsWeightCount[DS_DEVICE_COUNT - jjj - 1]){
            Serial.print('1');
            continue;
          } 
          Serial.print('0');  
        }
        Serial.println();
        for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
          if (!dsWeightCount[jjj]){
            continue;
          }
          Serial.print("i");
          Serial.print(jjj);
          Serial.print(": ");
          Serial.print(dsWeightCount[jjj]);
          Serial.print(", ");   
        }
        Serial.println();
        Serial.println(((float) i32 / i16) * DS_RAW_TO_C_MUL, precision);
        Serial.println("-- --");
      #endif
    } // if SERV_AVG

    client.print("HTTP/1.1");
    client.print(" ");

    if (serv & SERV_ERR){
      if (servError & SERV_ERROR_SERVICE_UNAVAILABLE){
        client.print("503 ");
        client.print("Service ");
        client.print("Unavai");
        client.println("lable");
      } else if (servError & SERV_ERROR_BAD_REQUEST) {  
        client.print("400 Bad ");
        client.println("Request");     
      } else if (servError & SERV_ERROR_NOT_FOUND) {
        client.print("404 Not ");
        client.println("Found");
      } else {
        client.print("500 Inter");
        client.print("nal Server ");
        client.println("Error");        
      }
    } else {
      client.println("200 OK");
    }

    client.print("Content-");
    client.print("Type: ");
       
    if (serv & SERV_TEXT){
      client.print("text/");
      client.println("plain");
      client.println();

      if (serv & SERV_ERR){
        if (servError & SERV_ERROR_SERVICE_UNAVAILABLE){
          client.print("503 ");
          client.print("Service ");
          client.print("Unavai");
          client.print("lable");
          break;
        }

        if (servError & SERV_ERROR_BAD_REQUEST){
          client.print("400 Bad ");
          client.print("Request");
          break;      
        }

        if (servError & SERV_ERROR_NOT_FOUND){
          client.print("404 Not ");
          client.print("Found");
          break;      
        }

        client.print("500 Inter");
        client.print("nal Server ");
        client.print("Error");
        break; 
      }

      if (serv & SERV_AVG){
        client.print(((float) i32 / i16) * DS_RAW_TO_C_MUL, precision);
        break;
      }

    } else { // JSON

      client.print("applicat");
      client.println("ion/json");
      client.println();
      client.print("{\"boot\":");
      client.print(bootCount);
      client.print(",\"cycle\":");
      client.print(dsCycleCount);
      client.print(",");

      if (serv & SERV_ERR){
        if (servError & SERV_ERROR_SERVICE_UNAVAILABLE){
          client.print("\"code\":");
          client.print("503,");
          client.print("\"msg\":");
          client.print("\"Service ");
          client.print("Unavai");
          client.print("lable\"}");
          break;
        }
  
        if (servError & SERV_ERROR_BAD_REQUEST){
          client.print("\"code\":");
          client.print("400,");
          client.print("\"msg\":");
          client.print("\"Bad ");
          client.print("Request\"}");
          break;      
        }

        if (servError & SERV_ERROR_NOT_FOUND){
          client.print("\"code\":");
          client.print("404,");
          client.print("\"msg\":");
          client.print("\"Not ");
          client.print("Found\"}");
          break;      
        }

        client.print("\"code\":");
        client.print("500,");
        client.print("\"msg\":");
        client.print("\"Internal ");
        client.print("Server Error\"}");
        break;  
      }

      if (serv & SERV_AVG){
        client.print("\"avg\":");
        client.print(((float) i32 / i16) * DS_RAW_TO_C_MUL, precision);
        client.print(",\"sorted\":");
        if (pageSwitch & PAGE_SWITCH_SORT_SENSORS){
          client.print("true");
        } else {
          client.print("false");
        }
        client.print(",\"acc\":");
        client.print(i32);
        client.print(",\"div\":");
        client.print(i16);
        client.print(",\"history\":");
        client.print(historySize);
        client.print(",\"used\":\"");
        for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
          if (dsWeightCount[DS_DEVICE_COUNT - jjj -1]){
            client.print('1');
            continue;              
          }
          client.print('0');
        }
        client.print("\",\"");
        client.print("wei");
        client.print("ght\":{");
        pageSwitch |= PAGE_SWITCH_FIRST_ITERATION;
        for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
          if (!dsWeightCount[jjj]){
            continue;
          }
          if (pageSwitch & PAGE_SWITCH_FIRST_ITERATION){
            pageSwitch &= ~PAGE_SWITCH_FIRST_ITERATION;
          } else {
            client.print(",");           
          }
          client.print("\"i");
          client.print(jjj);
          client.print("\":");
          client.print(dsWeightCount[jjj]);
        }        
        client.print("}}");
        break;
      } 
    } // JSON

    if (serv & SERV_SENSOR_ERRORS){
      if (!(serv & SERV_TEXT)){
        client.print("\"sensors\":{");        
      }
      pageSwitch |= PAGE_SWITCH_FIRST_ITERATION;
      for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
        if (bit_is_clear(sensors, jjj)){
          continue;
        }
        aaa = EEPROM_ERROR_COUNT + (jjj * 4);
        EEPROM.get(aaa, ui16);
        if (serv & SERV_TEXT){
          client.print("i");
          client.print(jjj);
          client.print(" err:");
        } else {
          if (pageSwitch & PAGE_SWITCH_FIRST_ITERATION){
            pageSwitch &= ~PAGE_SWITCH_FIRST_ITERATION;
          } else {
            client.print(",");           
          }
          client.print("\"i");
          client.print(jjj);
          client.print("\":{\"err\":");        
        }
        client.print(ui16);
        aaa = EEPROM_FAIL_COUNT + (jjj * 4);
        EEPROM.get(aaa, ui16);
        if (serv & SERV_TEXT){
          client.print(" fail:");
          client.println(ui16);
        } else {
          client.print(",\"fail\":");
          client.print(ui16);
          client.print("}");      
        }    
      }
      if (!(serv & SERV_TEXT)){
        client.print("}}");
      }
      break;
    } // SERV_SENSOR_ERRORS

    for (jjj = 0; jjj < DS_DEVICE_COUNT; jjj++){
      dsSort[jjj] = jjj;
    }

    if ((serv & SERV_HISTORY)
      || (serv & SERV_DELTA)){

      if (pageSwitch & PAGE_SWITCH_SORT_SENSORS){
        pageSwitch &= ~PAGE_SWITCH_SORTED;
        while (!(pageSwitch & PAGE_SWITCH_SORTED)){
          pageSwitch |= PAGE_SWITCH_SORTED;
          for (jjj = 0; jjj < (DS_DEVICE_COUNT - 1); jjj++){
            if (dsBufferedTemp[dsSort[jjj]] > dsBufferedTemp[dsSort[jjj + 1]]){
              swapSort();
            }               
          }
        }
        if (!(serv & SERV_TEXT)){
          client.print("\"sorted\":true,");   
        }
      }

      if (!(serv & SERV_TEXT)){
        client.print("\"sensors\":{");        
      }

      pageSwitch |= PAGE_SWITCH_FIRST_ITERATION;

      for (iii = 0; iii < DS_DEVICE_COUNT; iii++){
        if (bit_is_clear(sensors, dsSort[iii])){
          continue;
        }
        i16 = dsBufferedTemp[dsSort[iii]];
        if (serv & SERV_TEXT){
          client.print((float) i16 * DS_RAW_TO_C_MUL, precision); 
          client.print(" ");          
        } else {
          if (pageSwitch & PAGE_SWITCH_FIRST_ITERATION){
            pageSwitch &= ~PAGE_SWITCH_FIRST_ITERATION;
          } else {
            client.print(",");           
          }
          client.print("\"i");
          client.print(dsSort[iii]);
          client.print("\":{");
          if (pageSwitch & PAGE_SWITCH_SORT_SENSORS){
            client.print("\"pos\":");
            client.print(iii);
            client.print(",");            
          }
          client.print("\"temp\":");
          client.print((float) i16 * DS_RAW_TO_C_MUL, precision); 
          client.print(",\"buff\":[");
        }
        readBuffIndex = dsBuffIndex;
        if (dsSort[iii] > dsIndex || (dsSort[iii] == dsIndex && (dsStatus & DS_INDEX))){
          if (!readBuffIndex){
            readBuffIndex = DS_BUFF_SIZE;
          }
          readBuffIndex--;
        }
        for (jjj = 0; jjj < historySize; jjj++){
          if (!(serv & SERV_TEXT) && jjj){
            client.print(",");
          }
          if (serv & SERV_DELTA){
            if (bit_is_set(dsBuffChange[readBuffIndex], dsSort[iii])){
              if (bit_is_set(dsBuffAmount[readBuffIndex], dsSort[iii])){
                if (serv & SERV_TEXT){
                  client.print("+");                  
                } else {
                  client.print("1");
                }
              } else {
                if (serv & SERV_TEXT){
                  client.print("-");                  
                } else {
                  client.print("-1");
                }
              }
            } else {
              client.print("0");                
            }
          } else {
            if (bit_is_set(dsBuffChange[readBuffIndex], dsSort[iii])){
              if (bit_is_set(dsBuffAmount[readBuffIndex], dsSort[iii])){
                i16 -= DS_FILTER_ADAPT_STEP;
              } else {
                i16 += DS_FILTER_ADAPT_STEP;
              }
            }
            if (serv & SERV_TEXT){
              client.print((float) i16 * DS_RAW_TO_C_MUL, precision);
              client.print(" ");              
            } else {
              client.print((float) i16 * DS_RAW_TO_C_MUL, precision);
            }
          }
          if (!readBuffIndex){
            readBuffIndex = DS_BUFF_SIZE;
          }
          readBuffIndex--;
        }
        if (serv & SERV_TEXT){
          client.println();          
        } else {
          client.print("]}");
        };
        watchdog.reset();
      }
      if (!(serv & SERV_TEXT)){
        client.print("}}");
      }   
      break;
    }

    if (pageSwitch && PAGE_SWITCH_SORT_SENSORS){
      pageSwitch &= ~PAGE_SWITCH_SORTED;
      while (!(pageSwitch & PAGE_SWITCH_SORTED)){
        pageSwitch |= PAGE_SWITCH_SORTED;
        for (jjj = 0; jjj < (DS_DEVICE_COUNT - 1); jjj++){
          if (dsTemp[dsSort[jjj]] > dsTemp[dsSort[jjj + 1]]){
            swapSort();
          }
        }
      }
    }

    if (serv & SERV_TEXT){
      for (iii = 0; iii < DS_DEVICE_COUNT; iii++){
        if (bit_is_clear(sensors, dsSort[iii])){
          continue;
        }
        client.println((float) dsTemp[dsSort[iii]] * DS_RAW_TO_C_MUL, precision);
      }
      break;    
    }

    if (pageSwitch & PAGE_SWITCH_SORT_SENSORS){
      client.print("\"sorted\":true,");       
    }

    pageSwitch |= PAGE_SWITCH_FIRST_ITERATION;
    client.print("\"sensors\":{");
    for (iii = 0; iii < DS_DEVICE_COUNT; iii++){
      if (bit_is_clear(sensors, dsSort[iii])){
        continue;
      }
      if (pageSwitch & PAGE_SWITCH_FIRST_ITERATION){
        pageSwitch &= ~PAGE_SWITCH_FIRST_ITERATION;
      } else {
        client.print(",");           
      }
      client.print("\"i");
      client.print(dsSort[iii]);
      client.print("\":{");
      if (pageSwitch & PAGE_SWITCH_SORT_SENSORS){
        client.print("\"pos\":");
        client.print(iii);
        client.print(",");        
      }
      client.print("\"temp\":");
      client.println((float) dsTemp[dsSort[iii]] * DS_RAW_TO_C_MUL, precision);
      client.print("}");
    }
    client.print("}}");
    break;
  }
  // time for client
  delay(1);
  client.stop();
  #endif
}
