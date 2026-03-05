#ifndef UROS_WIFI_CONFIG_H
#define UROS_WIFI_CONFIG_H


//Includes
#include <stdint.h>
#include <Arduino.h>

// Wi-Fi configuration
char ssid[] = "YOUR_WIFI_SSID";
char psk[] = "YOUR_WIFI_PASSWORD";

//microROS agent config
IPAddress agent_ip(0, 0, 0, 0);     //  YOUR PC IP
size_t agent_port = 8888;           //  UROS AGENT PORT (Default 8888)




#endif