#ifndef UROS_WIFI_CONFIG_H
#define UROS_WIFI_CONFIG_H

//Includes
#include <stdint.h>
#include <Arduino.h>

char ssid[] = "SSID_HERE";
char psk[] = "SSID_PASSWORD_HERE";

//uros agent
IPAddress agent_ip(0, 0, 0, 0);
size_t agent_port = 0000;               //IP address and port of the uROS agent, to which the micro-ROS client will connect. The IP address and port must be the same as those used by the uROS agent.


#endif