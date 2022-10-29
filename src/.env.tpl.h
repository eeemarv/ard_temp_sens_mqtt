/*
* custom definitions that 
* can be put 
* into .env.h 
*/

#define MQTT_SERVER_IP 192, 168, 0, 40
#define IP_SELF 192, 168, 0, 80  // do not define when using DHCP
#define MAC_4_LAST 0xbe, 0xef, 0xfe, 0xed
#define ETHERNET_SELECT_PIN 10
#define SERIAL_BAUD 9600
#define SERIAL_EN
#define WATCHDOG_EN
#define TOPIC_SUB_REQ "req/sens/w0"
#define TOPIC_PUB "sens/w0"