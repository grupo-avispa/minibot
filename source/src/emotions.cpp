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
#include "../include/emotions.hpp" 

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define MAX_BRIGHTNESS 50

RobotEmotions::RobotEmotions() 
    : displayRight(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1),
      displayLeft(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1),
      myDFSerial(1), currentState(STATE_START), stateTimer(0), idleTimer(0),
      randomIdleTimeout(4000), globalVolume(23), face_reset_(false),
      currentLockPriority(-1), lockDuration(0), lockTimer(0), lastGoalReceivedTime(0),
      currentLedPattern(LED_OFF), ledR(0), ledG(0), ledB(0), ledSpeed(5), ledPhase(0)
{}

void RobotEmotions::init() {
    pinMode(LED_R, OUTPUT); pinMode(LED_G, OUTPUT); pinMode(LED_B, OUTPUT);
    
    Wire.begin(EMOTION_SDA_PIN, EMOTION_SCL_PIN);
    displayLeft.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    displayRight.begin(SSD1306_SWITCHCAPVCC, 0x3D);

    myDFSerial.begin(9600, SERIAL_8N1, DF_RX_PIN, DF_TX_PIN);
    delay(500);
    if (myDFPlayer.begin(myDFSerial)) { myDFPlayer.volume(globalVolume); }

    randomSeed(analogRead(0));
    changeState(STATE_START);
}

void RobotEmotions::emotion_sub_callback(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    if(msg->data >= 0 && msg->data <= 11 && msg->data != static_cast<int32_t>(currentState)) {
        changeState(static_cast<RobotState>(msg->data));
    }
}

// ASIGNACIÓN DE PRIORIDADES 
int RobotEmotions::getPriority(RobotState s) {
    if (s == STATE_REACHED || s == STATE_ABORTED) return 60; // prioridad maxima: Nunca es pisado por nada
    if (s == STATE_GOAL_RECEIVED) return 50; 
    if (s == STATE_ALMOST_THERE) return 40; 
    if (s == STATE_ROTATING) return 30; // Giro
    if (s == STATE_ACCELERATING) return 20; // Acelerar
    if (s == STATE_LOST) return 15; // AMCL
    if (s == STATE_RECOVERY || s == STATE_OBSTACLE) return 10; // Otros críticos
    return 0; // Idles de charleta (PATH y CONNECTED)
}

void RobotEmotions::changeState(RobotState newState) {
    unsigned long now = millis();

    // Filtro para viajes súper cortos
    if (newState == STATE_GOAL_RECEIVED) lastGoalReceivedTime = now;
    if (newState == STATE_ALMOST_THERE && (now - lastGoalReceivedTime < 6000)) return;

    int newPrio = getPriority(newState);
    bool isSilent = (newState == STATE_CONNECTED || newState == STATE_PATH);
    bool lockActive = (now - lockTimer < lockDuration);

    // SISTEMA DE INTERRUPCIÓN (OVERRIDE)
    if (lockActive && newPrio <= currentLockPriority) {
        // Solo relajamos gráficos si no hay audios mega-importantes (>= 40) en curso
        if (isSilent && currentLockPriority < 40) {
            // Se permite ir a silencio
        } else {
            return; // ESTADO IGNORADO: Respeta la prioridad
        }
    }

    // ACEPTADO
    currentState = newState;
    stateTimer = now;
    idleTimer = now;

    if (isSilent) {
        face_reset_ = true;
        draw_eyes("normal", "normal");
        if (newState == STATE_CONNECTED) setLedPattern(LED_RAINBOW, 0, 0, 0, 120);
        else setLedPattern(LED_STILL, 0, 255, 0);
        
        currentLockPriority = -1; // Libera el candado
        lockDuration = 0;
    } 
    else if (newState == STATE_START) {
        face_reset_ = false;
        draw_eyes("happy", "wink");
        setLedPattern(LED_RAINBOW, 0, 0, 0, 120);
        myDFPlayer.play(1);
        
        currentLockPriority = 60;
        lockDuration = 6000;
        lockTimer = now;
    } 
    else {
        // Corta y ejecuta nueva reacción
        face_reset_ = false;
        executeReaction(newState);

        currentLockPriority = newPrio;
        lockDuration = (newPrio >= 40) ? 7000 : 7000; // Protege más tiempo los estados top
        lockTimer = now;
    }
}

void RobotEmotions::update_task() {
    updateLed(); 
    unsigned long now = millis();
    unsigned long elapsed_state = now - stateTimer;

    // Liberamos candado si expiró el tiempo
    if (now - lockTimer >= lockDuration) {
        currentLockPriority = -1;
    }

    // RESETEO FACIAL (A los 4.5 segundos exactos relaja la cara a Rainbow o Verde)
    if (elapsed_state > 4500 && !face_reset_) {
        draw_eyes("normal", "normal");
        if (currentState == STATE_REACHED || currentState == STATE_ABORTED || currentState == STATE_LOST || currentState == STATE_START || currentState == STATE_CONNECTED) {
            setLedPattern(LED_RAINBOW, 0, 0, 0, 120); 
        } else {
            setLedPattern(LED_STILL, 0, 255, 0);
        }
        face_reset_ = true; 
    }

    // IDLES CASUALES ACELERADOS
    if (currentState == STATE_CONNECTED || currentState == STATE_PATH) {
        bool lockActive = (now - lockTimer < lockDuration);
        if (now - idleTimer > randomIdleTimeout && face_reset_ && !lockActive) {
            executeReaction(currentState); 
            stateTimer = now; 
            idleTimer = now;
            face_reset_ = false; 
            
            currentLockPriority = 0; // Prioridad Idle (para que los giros lo corten sin problema)
            lockDuration = 7000;
            lockTimer = now;

            // Diferentes velocidades de charleta
            if (currentState == STATE_CONNECTED) {
                randomIdleTimeout = random(10000, 14000);   // En reposo habla más (cada 4-8s)
            } else {
                randomIdleTimeout = random(9000, 17000);  // En ruta un poco menos (cada 9-17s)
            }
        }
    }
}

// ==========================================
// EL CEREBRO DE LAS 5 REACCIONES POR ESTADO
// ==========================================
void RobotEmotions::executeReaction(RobotState state) {
    int r = reactionBags[state].next(); 
    int base_track = 2 + ((state - 1) * 5); 
    
    myDFPlayer.play(base_track + r);

    switch(state) {
        case STATE_CONNECTED: 
            if(r==0){ draw_eyes("normal", "normal"); setLedPattern(LED_BREATHE, 0, 200, 255, 3); } 
            if(r==1){ draw_eyes("love", "happy"); setLedPattern(LED_RAINBOW, 0, 0, 0, 12); } 
            if(r==2){ draw_eyes("sleepy", "sleepy"); setLedPattern(LED_FADE_PULSE, 50, 0, 150, 2); } 
            if(r==3){ draw_eyes("suspicious", "look_right"); setLedPattern(LED_STILL, 255, 150, 0); } 
            if(r==4){ draw_eyes("surprised", "focused"); setLedPattern(LED_FLASH, 0, 255, 100, 6); } 
            break;

        case STATE_GOAL_RECEIVED: 
            if(r==0){ draw_eyes("focused", "focused"); setLedPattern(LED_STILL, 255, 255, 255); } 
            if(r==1){ draw_eyes("happy", "happy"); setLedPattern(LED_STROBE, 0, 255, 0, 15); } 
            if(r==2){ draw_eyes("angry", "suspicious"); setLedPattern(LED_BREATHE, 255, 50, 0, 6); } 
            if(r==3){ draw_eyes("scared", "scared"); setLedPattern(LED_FADE_PULSE, 255, 200, 0, 8); } 
            if(r==4){ draw_eyes("surprised", "happy"); setLedPattern(LED_RAINBOW, 0, 0, 0, 120); } 
            break;

        case STATE_PATH: 
            if(r==0){ draw_eyes("sleepy", "happy"); setLedPattern(LED_BREATHE, 255, 100, 200, 4); } 
            if(r==1){ draw_eyes("happy", "wink"); setLedPattern(LED_RAINBOW, 0, 0, 0, 120); } 
            if(r==2){ draw_eyes("normal", "normal"); setLedPattern(LED_STILL, 0, 150, 255); } 
            if(r==3){ draw_eyes("love", "suspicious"); setLedPattern(LED_FADE_PULSE, 255, 0, 255, 5); } 
            if(r==4){ draw_eyes("look_left", "suspicious"); setLedPattern(LED_FLASH, 200, 200, 0, 4); } 
            break;

        case STATE_REACHED: 
            if(r==0){ draw_eyes("love", "love"); setLedPattern(LED_RAINBOW, 0, 0, 0, 120); } 
            if(r==1){ draw_eyes("happy", "surprised"); setLedPattern(LED_STROBE, 0, 255, 255, 15); } 
            if(r==2){ draw_eyes("focused", "focused"); setLedPattern(LED_STILL, 0, 255, 0); } 
            if(r==3){ draw_eyes("suspicious", "happy"); setLedPattern(LED_BREATHE, 255, 215, 0, 8); } 
            if(r==4){ draw_eyes("look_right", "wink"); setLedPattern(LED_FADE_PULSE, 150, 0, 255, 5); } 
            break;

        case STATE_ABORTED: 
            if(r==0){ draw_eyes("angry", "look_right"); setLedPattern(LED_STILL, 255, 0, 0); } 
            if(r==1){ draw_eyes("sad", "sad"); setLedPattern(LED_BREATHE, 0, 0, 200, 4); } 
            if(r==2){ draw_eyes("dead", "dead"); setLedPattern(LED_FLASH, 255, 0, 0, 15); } 
            if(r==3){ draw_eyes("suspicious", "angry"); setLedPattern(LED_POLICE, 0, 0, 0, 10); } 
            if(r==4){ draw_eyes("scared", "scared"); setLedPattern(LED_STROBE, 255, 255, 255, 20); } 
            break;

        case STATE_RECOVERY: 
            if(r==0){ draw_eyes("sad", "focused"); setLedPattern(LED_FADE_PULSE, 255, 100, 0, 6); } 
            if(r==1){ draw_eyes("dizzy", "scared"); setLedPattern(LED_FLASH, 255, 255, 0, 15); } 
            if(r==2){ draw_eyes("angry", "angry"); setLedPattern(LED_STILL, 200, 0, 0); } 
            if(r==3){ draw_eyes("surprised", "confused"); setLedPattern(LED_STROBE, 255, 0, 255, 10); } 
            if(r==4){ draw_eyes("focused", "focused"); setLedPattern(LED_POLICE, 0, 0, 0, 12); } 
            break;

        case STATE_LOST: 
            if(r==0){ draw_eyes("scared", "scared"); setLedPattern(LED_STROBE, 150, 200, 255, 20); } 
            if(r==1){ draw_eyes("confused", "focused"); setLedPattern(LED_FLASH, 255, 0, 0, 10); } 
            if(r==2){ draw_eyes("suspicious", "sad"); setLedPattern(LED_BREATHE, 150, 0, 255, 5); } 
            if(r==3){ draw_eyes("sad", "look_left"); setLedPattern(LED_STILL, 200, 150, 0); } 
            if(r==4){ draw_eyes("dizzy", "suspicious"); setLedPattern(LED_FADE_PULSE, 255, 120, 0, 4); } 
            break;

        case STATE_OBSTACLE: 
            if(r==0){ draw_eyes("focused", "focused"); setLedPattern(LED_FLASH, 255, 0, 0, 15); } 
            if(r==1){ draw_eyes("surprised", "surprised"); setLedPattern(LED_STROBE, 255, 255, 0, 10); } 
            if(r==2){ draw_eyes("angry", "suspicious"); setLedPattern(LED_STILL, 255, 100, 0); } 
            if(r==3){ draw_eyes("sad", "wink"); setLedPattern(LED_BREATHE, 255, 0, 255, 8); } 
            if(r==4){ draw_eyes("scared", "scared"); setLedPattern(LED_POLICE, 0, 0, 0, 15); } 
            break;

        case STATE_ROTATING: 
            if(r==0){ draw_eyes("look_left", "look_left"); setLedPattern(LED_STILL, 0, 150, 255); } 
            if(r==1){ draw_eyes("dizzy", "surprised"); setLedPattern(LED_RAINBOW, 0, 0, 0, 120); } 
            if(r==2){ draw_eyes("love", "happy"); setLedPattern(LED_FADE_PULSE, 0, 255, 255, 6); } 
            if(r==3){ draw_eyes("normal", "normal"); setLedPattern(LED_FLASH, 0, 255, 0, 5); } 
            if(r==4){ draw_eyes("wink", "happy"); setLedPattern(LED_STROBE, 255, 200, 0, 12); } 
            break;

        case STATE_ACCELERATING: 
            if(r==0){ draw_eyes("angry", "happy"); setLedPattern(LED_STROBE, 0, 255, 255, 20); } 
            if(r==1){ draw_eyes("focused", "focused"); setLedPattern(LED_POLICE, 0, 0, 0, 18); } 
            if(r==2){ draw_eyes("normal", "normal"); setLedPattern(LED_STILL, 255, 255, 255); } 
            if(r==3){ draw_eyes("happy", "surprised"); setLedPattern(LED_FLASH, 0, 255, 0, 10); } 
            if(r==4){ draw_eyes("wink", "suspicious"); setLedPattern(LED_RAINBOW, 0, 0, 0, 120); } 
            break;

        case STATE_ALMOST_THERE: 
            if(r==0){ draw_eyes("focused", "focused"); setLedPattern(LED_BREATHE, 255, 150, 0, 10); } 
            if(r==1){ draw_eyes("happy", "happy"); setLedPattern(LED_FLASH, 0, 255, 0, 8); } 
            if(r==2){ draw_eyes("sleepy", "suspicious"); setLedPattern(LED_STILL, 150, 0, 255); } 
            if(r==3){ draw_eyes("love", "happy"); setLedPattern(LED_RAINBOW, 0, 0, 0, 120); } 
            if(r==4){ draw_eyes("surprised", "happy"); setLedPattern(LED_STROBE, 0, 255, 255, 10); } 
            break;
    }
}

// ==========================================
// BIBLIOTECA DE LEDS Y OJOS (Igual a la tuya)
// ==========================================
void RobotEmotions::setLedPattern(LedPattern pattern, int r, int g, int b, int speed) {
    currentLedPattern = pattern; ledR = r; ledG = g; ledB = b; ledSpeed = speed; ledPhase = 0;
}

void RobotEmotions::updateLed() {
    ledPhase += ledSpeed * 0.05; 
    int r=0, g=0, b=0;
    
    switch(currentLedPattern) {
        case LED_OFF: break;
        case LED_STILL: r=ledR; g=ledG; b=ledB; break;
        case LED_BREATHE: {
            float intensity = (sin(ledPhase * 0.1) + 1.0) / 2.0;
            r = ledR * intensity; g = ledG * intensity; b = ledB * intensity;
            break;
        }
        case LED_FLASH: {
            if(((int)ledPhase % 10) < 5) { r=ledR; g=ledG; b=ledB; }
            break;
        }
        case LED_STROBE: { 
            if(((int)ledPhase % 4) < 2) { r=ledR; g=ledG; b=ledB; }
            break;
        }
        case LED_FADE_PULSE: { 
            float intensity = abs(sin(ledPhase * 0.15));
            r = ledR * intensity; g = ledG * intensity; b = ledB * intensity;
            break;
        }
        case LED_RAINBOW: {
            int pos = ((int)ledPhase) % 256;
            if(pos < 85){ r = 255-pos*3; g = 0; b = pos*3; } 
            else if(pos < 170){ pos -= 85; r = 0; g = pos*3; b = 255-pos*3; } 
            else{ pos -= 170; r = pos*3; g = 255-pos*3; b = 0; }
            break;
        }
        case LED_POLICE: { 
            if(((int)ledPhase % 10) < 5) { r = 255; g = 0; b = 0; } else { r = 0; g = 0; b = 255; }
            break;
        }
    }
    analogWrite(LED_R, map(r, 0, 255, 0, MAX_BRIGHTNESS));
    analogWrite(LED_G, map(g, 0, 255, 0, MAX_BRIGHTNESS));
    analogWrite(LED_B, map(b, 0, 255, 0, MAX_BRIGHTNESS));
}

void RobotEmotions::draw_eyes(String expr_left, String expr_right) {
    draw_single_eye(&displayLeft, expr_left, true);
    draw_single_eye(&displayRight, expr_right, false);
    displayLeft.display();
    displayRight.display();
}

void RobotEmotions::draw_single_eye(Adafruit_SSD1306 *display, String emotion, bool is_left) {
    display->clearDisplay();
    int oX = 34, oY = 4, oW = 60, oH = 56, oR = 28;
    
    if (emotion == "happy") { display->fillRoundRect(oX, 10, oW, 54, 27, SSD1306_WHITE); display->fillRect(0, 38, 128, 30, SSD1306_BLACK); }
    else if (emotion == "sad") { display->fillRoundRect(oX, oY, oW, oH, oR, SSD1306_WHITE); if(is_left) display->fillTriangle(0,0,128,0,0,40,SSD1306_BLACK); else display->fillTriangle(128,0,0,0,128,40,SSD1306_BLACK); }
    else if (emotion == "angry") { display->fillRoundRect(oX, oY, oW, oH, oR, SSD1306_WHITE); if(is_left) display->fillTriangle(128,0,0,0,128,40,SSD1306_BLACK); else display->fillTriangle(0,0,128,0,0,40,SSD1306_BLACK); }
    else if (emotion == "scared") { display->fillRoundRect(oX, oY, oW, oH, oR, SSD1306_WHITE); if(is_left) display->fillTriangle(0,0,128,0,0,40,SSD1306_BLACK); else display->fillTriangle(128,0,0,0,128,40,SSD1306_BLACK); for(int i=0; i<3; i++) display->drawLine(oX+5, oY+oH-3+i*4, oX+oW-5, oY+oH-3+i*4, SSD1306_WHITE); }
    else if (emotion == "sleepy") { int baseY=28; for(int i=0; i<6; i++){ display->drawLine(oX-5, baseY+5+i, oX+15, baseY-5+i, SSD1306_WHITE); display->drawLine(oX+15, baseY-5+i, oX+oW-15, baseY+15+i, SSD1306_WHITE); display->drawLine(oX+oW-15, baseY+15+i, oX+oW+5, baseY+5+i, SSD1306_WHITE); } }
    else if (emotion == "surprised") { for(int r=4; r<=30; r+=8) display->drawRoundRect(64-r, 32-r, r*2, r*2, r/2, SSD1306_WHITE); }
    else if (emotion == "dead") { for(int i=-6; i<=6; i++){ display->drawLine(34+i, 4, 94+i, 60, SSD1306_WHITE); display->drawLine(94+i, 4, 34+i, 60, SSD1306_WHITE); } }
    else if (emotion == "normal") { display->fillRoundRect(oX, oY, oW, oH, oR, SSD1306_WHITE); }
    else if (emotion == "wink") { display->fillRect(34, 28, 60, 8, SSD1306_WHITE); }
    else if (emotion == "dizzy") { for(int i=0; i<360; i+=20){ float rad = i*PI/180.0; display->drawPixel(64+cos(rad)*(i/15.0), 32+sin(rad)*(i/15.0), SSD1306_WHITE); } }
    else if (emotion == "love") { display->fillCircle(64-12, 32-5, 15, SSD1306_WHITE); display->fillCircle(64+12, 32-5, 15, SSD1306_WHITE); display->fillTriangle(64-27, 32, 64+27, 32, 64, 55, SSD1306_WHITE); }
    else if (emotion == "suspicious") { display->fillRoundRect(oX, 20, oW, 20, 10, SSD1306_WHITE); }
    else if (emotion == "look_left") { display->fillRoundRect(oX-15, oY, oW, oH, oR, SSD1306_WHITE); }
    else if (emotion == "look_right") { display->fillRoundRect(oX+15, oY, oW, oH, oR, SSD1306_WHITE); }
    else if (emotion == "confused") { if(is_left) display->fillRoundRect(oX, 20, oW, 20, 10, SSD1306_WHITE); else display->fillRoundRect(oX, oY, oW, oH, oR, SSD1306_WHITE); }
    else if (emotion == "focused") { display->fillRoundRect(oX+10, oY+10, oW-20, oH-20, oR/2, SSD1306_WHITE); } 
}