#ifndef MAC_4_LAST
  #define MAC_4_LAST 0xbe, 0xef, 0x13, 0x40
#endif
#ifndef SELF_IP
  #define SELF_IP 192, 168, 0, 40
#endif
#ifndef MQTT_SERVER_IP
  #define MQTT_SERVER_IP 192, 168, 0, 70
#endif
#ifndef ETH_CS_PIN
  #define ETH_CS_PIN 10
#endif
#ifndef SERIAL_BAUD
  #define SERIAL_BAUD 115200
#endif
#ifndef PUB_WATER_TEMP
  #define PUB_WATER_TEMP "sens/water_temp"
#endif
#ifndef PUB_PRESENCE
  #define PUB_PRESENCE "sens/p"
#endif
#ifndef PUB_ERRORS
  #define PUB_ERRORS "sens/err"
#endif
#ifndef PRESENCE_INTERVAL
  #define PRESENCE_INTERVAL 6666
#endif
#ifndef ERRORS_INTERVAL
  #define ERRORS_INTERVAL 10000
#endif
#ifndef CLIENT_ID_PREFIX
  #define CLIENT_ID_PREFIX "w0_"
#endif
#ifndef DS_MAX_RAW
#define DS_MAX_RAW 5760  // 45°C
#endif
#ifndef DS_MIN_RAW
#define DS_MIN_RAW -2560   // -20°C
#endif
