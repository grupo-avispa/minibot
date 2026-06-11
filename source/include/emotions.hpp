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
#ifndef EMOTIONS_HPP
#define EMOTIONS_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
#include "DFRobotDFPlayerMini.h"
#include <math.h> 

// micro-ROS includes
#include <rcl/rcl.h>
#include <std_msgs/msg/int32.h>

#define EMOTION_SDA_PIN 13
#define EMOTION_SCL_PIN 14
#define DF_RX_PIN 18 
#define DF_TX_PIN 5  

#define LED_R 15
#define LED_G 12
#define LED_B 22

enum RobotState {
    STATE_START = 0,
    STATE_CONNECTED = 1,
    STATE_GOAL_RECEIVED = 2,
    STATE_PATH = 3,
    STATE_REACHED = 4,
    STATE_ABORTED = 5,
    STATE_RECOVERY = 6,
    STATE_LOST = 7,
    STATE_OBSTACLE = 8,
    STATE_ROTATING = 9,
    STATE_ACCELERATING = 10,
    STATE_ALMOST_THERE = 11
};

enum LedPattern {
    LED_OFF, LED_STILL, LED_BREATHE, LED_FLASH, 
    LED_RAINBOW, LED_POLICE, LED_FADE_PULSE, LED_STROBE
};

class ShuffleBag {
private:
    bool played[5];
    int count;
public:
    ShuffleBag() { reset(); }
    void reset() { for(int i=0; i<5; i++) played[i] = false; count = 0; }
    int next() {
        if(count >= 5) reset();
        int r;
        do { r = random(0, 5); } while(played[r]);
        played[r] = true;
        count++;
        return r;
    }
};

class RobotEmotions {
public:
    RobotEmotions();
    void init();
    void update_task(); 
    void emotion_sub_callback(const void * msgin);
    void changeState(RobotState newState);

    rcl_subscription_t emotion_sub;
    std_msgs__msg__Int32 emotion_msg;

private:
    void updateLed();
    void setLedPattern(LedPattern pattern, int r, int g, int b, int speed = 5);
    void draw_eyes(String expr_left, String expr_right);
    void draw_single_eye(Adafruit_SSD1306 *display, String emotion, bool is_left);
    void executeReaction(RobotState state);
    int getPriority(RobotState s);

    RobotState currentState;
    unsigned long stateTimer;
    unsigned long idleTimer;
    unsigned long randomIdleTimeout;
    int globalVolume;
    bool face_reset_;

    // --- SISTEMA DE PRIORIDADES DIRECTAS ---
    int currentLockPriority;
    unsigned long lockDuration;
    unsigned long lockTimer;
    unsigned long lastGoalReceivedTime; 

    LedPattern currentLedPattern;
    int ledR, ledG, ledB, ledSpeed;
    float ledPhase;

    ShuffleBag reactionBags[12];

    Adafruit_SSD1306 displayRight;
    Adafruit_SSD1306 displayLeft;
    DFRobotDFPlayerMini myDFPlayer;
    HardwareSerial myDFSerial;
};

#endif // EMOTIONS_HPP