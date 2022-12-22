/*
* custom definitions that 
* can be put 
* into .env.h 
*/

#define MQTT_SERVER_IP 192, 168, 0, 20
#define MQTT_PORT 1883
#define SELF_IP 192, 168, 0, 40
#define MAC_4_LAST 0xbe, 0xef, 0x13, 0x40
#define ETH_CS_PIN 10
#define SERIAL_BAUD 115200
#define SERIAL_EN
#define WATCHDOG_EN
#define TOPIC_SUB_REQ "req/sens/w0"
#define TOPIC_PUB "sens/w0"
#define CLIENT_ID_PREFIX "ts_"