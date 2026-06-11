/*
 * Minibot — autonomous differential-drive robot for ROS 2 teaching
 *
 * Copyright (c) 2026 Óscar Pons Fernández
 * Copyright (c) 2026 José Galeas
 * Copyright (c) 2026 Alberto J. Tudela Roldán
 * Copyright (c) 2026 Guillermo Martos
 * Grupo Avispa, DTE, Universidad de Málaga
 *
 * This file is part of the Minibot project.
 *
 * Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
 * 4.0 International License (CC BY-NC-SA 4.0). You may use, share, and adapt
 * this work for non-commercial purposes, with attribution, distributing any
 * derivative works under the same license. Commercial use requires prior
 * written permission from the authors.
 *
 * You should have received a copy of the license with this project (see the
 * LICENSE file). If not, see: https://creativecommons.org/licenses/by-nc-sa/4.0/
 */
#ifndef UROS_WIFI_CONFIG_H
#define UROS_WIFI_CONFIG_H


//Includes
#include <stdint.h>
#include <Arduino.h>


char ssid[] = "Robotito WIFI";
char psk[] = "MiniROS2026";

//uros agent
IPAddress agent_ip(192, 168, 0, 100);
size_t agent_port = 8888;



#endif // UROS_WIFI_CONFIG_H