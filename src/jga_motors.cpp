#include "../include/jga_motors.hpp"
#include <rmw_microros/rmw_microros.h>

// =============================================================================
//  Constructor / Destructor
// =============================================================================

JgaMotors::JgaMotors()
    : x(0.0f), y(0.0f), theta(0.0f),
      prev_ticks_left(0), prev_ticks_right(0),
      prev_time_ms(0)
{
    // Los objetos MX1508 se crean en init() para evitar problemas con el
    // framework Arduino antes de que configure los canales LEDC en setup().
}

JgaMotors::~JgaMotors()
{
    delete motor_l;
    delete motor_r;
}

// =============================================================================
//  init()
// =============================================================================

void JgaMotors::init()
{
    // ── 1. Motores ────────────────────────────────────────────────────────────
    motor_l = new MX1508(IN1_L, IN2_L, SLOW_DECAY);
    motor_r = new MX1508(IN1_R, IN2_R, SLOW_DECAY);
    motor_l->motorStop();
    motor_r->motorStop();

    Serial.printf("[JGA] Motores: L(IN1=%d IN2=%d)  R(IN1=%d IN2=%d)\n",
                  IN1_L, IN2_L, IN1_R, IN2_R);

    // ── 2. Encoders ───────────────────────────────────────────────────────────
    pinMode(ENC_A_L, INPUT_PULLUP);
    pinMode(ENC_B_L, INPUT_PULLUP);
    pinMode(ENC_A_R, INPUT_PULLUP);
    pinMode(ENC_B_R, INPUT_PULLUP);

    encoder_l.attachFullQuad(ENC_A_L, ENC_B_L);
    encoder_r.attachFullQuad(ENC_A_R, ENC_B_R);
    encoder_l.clearCount();
    encoder_r.clearCount();
    encoder_l.setFilter(1023);
    encoder_r.setFilter(1023);

    Serial.printf("[JGA] Encoders: L(A=%d B=%d)  R(A=%d B=%d)\n",
                  ENC_A_L, ENC_B_L, ENC_A_R, ENC_B_R);

    // ── 3. LEDs de diagnóstico ────────────────────────────────────────────────
    pinMode(LED_RED,  OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_RED,  LOW);
    digitalWrite(LED_BLUE, LOW);
    led_red_state  = false;
    led_blue_state = false;

    Serial.printf("[JGA] LEDs: ROJO=%d (izq)  AZUL=%d (der)\n", LED_RED, LED_BLUE);

    // ── 4. Reserva de memoria TF ──────────────────────────────────────────────
    tf_msg.transforms.capacity = 1;
    tf_msg.transforms.size     = 1;
    tf_msg.transforms.data     = (geometry_msgs__msg__TransformStamped *)malloc(
        sizeof(geometry_msgs__msg__TransformStamped));
    memset(tf_msg.transforms.data, 0, sizeof(geometry_msgs__msg__TransformStamped));

    // ── 5. Info de arranque ───────────────────────────────────────────────────
    Serial.printf("[JGA] MAX_VEL=%.4f m/s  MIN_PWM=%d  m/tick=%.6f  ticks/rev=%.0f\n",
                  MAX_VEL_M_S, MIN_PWM_REAL, METROS_POR_TICK, TICKS_POR_VUELTA_TOTAL);
    Serial.printf("[JGA] PID: Kp=%.1f  Ki=%.1f  Kd=%.1f  intLim=200\n",
                  pid_l.kp, pid_l.ki, pid_l.kd);
    Serial.printf("[JGA] Rampa=%.2f m/s²  VelAlpha=%.2f  BandaMuerta=%.3f m/s\n",
                  RAMP_RATE_M_S2, VEL_ALPHA, BANDA_MUERTA_M_S);
    Serial.println("[JGA] Init OK");
}

// =============================================================================
//  emergency_stop()
// =============================================================================

void JgaMotors::emergency_stop()
{
    goal_vel_l   = 0.0f;
    goal_vel_r   = 0.0f;
    ramped_vel_l = 0.0f;
    ramped_vel_r = 0.0f;
    pid_l.reset();
    pid_r.reset();
    filtered_vel_l   = 0.0f;
    filtered_vel_r   = 0.0f;
    arranque_ciclos_l = 0;
    arranque_ciclos_r = 0;
    ancla            = false;

    if (motor_l) motor_l->motorStop();
    if (motor_r) motor_r->motorStop();

    Serial.println("[JGA] Emergency stop — integral y rampa reseteadas.");
}

// =============================================================================
//  pwmFromPid()
//  Convierte salida PID [-255..255] a PWM con zona muerta aplicada.
//  Escala [1..255] → [MIN_PWM_REAL..255] para que el motor siempre arranque.
// =============================================================================

int JgaMotors::pwmFromPid(float pid_output)
{
    if (fabsf(pid_output) < 1.0f) return 0;

    const int dir = (pid_output > 0) ? 1 : -1;
    const int raw = constrain((int)fabsf(pid_output), 1, 255);
    const int pwm = (int)map((long)raw, 1L, 255L, (long)MIN_PWM_REAL, 255L);
    return pwm * dir;
}

// =============================================================================
//  applyMotorPwm()
// =============================================================================

void JgaMotors::applyMotorPwm(int pwm_l, int pwm_r)
{
    if (pwm_l == 0) motor_l->motorStop();
    else            motor_l->motorGo(pwm_l);

    // Motor derecho invertido por montaje en espejo
    if (pwm_r == 0) motor_r->motorStop();
    else            motor_r->motorGo(-pwm_r);
}

// =============================================================================
//  updateRamp()
//  Aproxima ramped_vel_* hacia goal_vel_* a una tasa máxima de RAMP_RATE_M_S2.
// =============================================================================

void JgaMotors::updateRamp(float dt)
{
    const float step = RAMP_RATE_M_S2 * dt;

    // Izquierdo
    const float diff_l = goal_vel_l - ramped_vel_l;
    if      (diff_l >  step) ramped_vel_l += step;
    else if (diff_l < -step) ramped_vel_l -= step;
    else                     ramped_vel_l  = goal_vel_l;

    // Derecho
    const float diff_r = goal_vel_r - ramped_vel_r;
    if      (diff_r >  step) ramped_vel_r += step;
    else if (diff_r < -step) ramped_vel_r -= step;
    else                     ramped_vel_r  = goal_vel_r;
}

// =============================================================================
//  checkFullTurns()
//  Detecta cada vuelta completa y conmuta el LED correspondiente.
// =============================================================================

void JgaMotors::checkFullTurns(int64_t d_l, int64_t d_r)
{
    total_ticks_left  += d_l;
    total_ticks_right += d_r;

    const int64_t TICKS_REV = (int64_t)TICKS_POR_VUELTA_TOTAL;

    // ── Motor izquierdo ───────────────────────────────────────────────────────
    const int64_t new_turns_l = total_ticks_left / TICKS_REV;
    if (new_turns_l != full_turns_left) {
        full_turns_left = new_turns_l;
        led_red_state   = !led_red_state;
        digitalWrite(LED_RED, led_red_state ? HIGH : LOW);
        Serial.printf("[ENC] Motor IZQUIERDO vuelta #%lld"
                      "  (ticks_acum=%lld  vel=%.4f m/s  %.1f RPM)\n",
                      full_turns_left,
                      total_ticks_left,
                      filtered_vel_l,
                      filtered_vel_l * RPM_POR_M_S);
    }

    // ── Motor derecho ─────────────────────────────────────────────────────────
    const int64_t new_turns_r = total_ticks_right / TICKS_REV;
    if (new_turns_r != full_turns_right) {
        full_turns_right = new_turns_r;
        led_blue_state   = !led_blue_state;
        digitalWrite(LED_BLUE, led_blue_state ? HIGH : LOW);
        Serial.printf("[ENC] Motor DERECHO   vuelta #%lld"
                      "  (ticks_acum=%lld  vel=%.4f m/s  %.1f RPM)\n",
                      full_turns_right,
                      total_ticks_right,
                      filtered_vel_r,
                      filtered_vel_r * RPM_POR_M_S);
    }
}

// =============================================================================
//  cmd_vel_sub_callback()
//  Solo actualiza los setpoints objetivo. El PID se ejecuta en el timer.
// =============================================================================

void JgaMotors::cmd_vel_sub_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;

    const float lin = msg->linear.x;
    const float ang = msg->angular.z;

    float new_goal_l = lin - (ang * WHEEL_BASE / 2.0f);
    float new_goal_r = lin + (ang * WHEEL_BASE / 2.0f);

    // Fix 1: clamp al rango físico del motor.
    // Sin esto, combinaciones de lin+ang grandes saturan el PID, lo resetean
    // continuamente y producen comportamiento asimétrico entre ruedas.
    new_goal_l = constrain(new_goal_l, -MAX_VEL_M_S, MAX_VEL_M_S);
    new_goal_r = constrain(new_goal_r, -MAX_VEL_M_S, MAX_VEL_M_S);

    // Fix 2: zona muerta de setpoint.
    // Si el setpoint es menor que la velocidad mínima que el motor puede generar
    // físicamente (la que corresponde a MIN_PWM_REAL), el PID nunca puede
    // alcanzarlo → error permanente → la integral crece sin límite → windup →
    // oscilaciones. Solución: redondear a cero esos setpoints inviables.
    // vel_min_fisica ≈ MAX_VEL * (MIN_PWM / 255)
    static constexpr float SP_DEADBAND =
        MAX_VEL_M_S * ((float)MIN_PWM_REAL / 255.0f);
    if (fabsf(new_goal_l) < SP_DEADBAND) new_goal_l = 0.0f;
    if (fabsf(new_goal_r) < SP_DEADBAND) new_goal_r = 0.0f;

    // ── Determinar si hace falta arranque suave ───────────────────────────────
    auto needSoftStart = [&](float prev_goal, float new_goal,
                              float vel_filtered, int &signo_ant,
                              int &ciclos_arranque) {
        const int new_sign = (new_goal > 0.0f) ? 1 : ((new_goal < 0.0f) ? -1 : 0);
        const bool desde_parado = fabsf(vel_filtered) < BANDA_MUERTA_M_S * 2.0f;
        const bool cambio_signo = (signo_ant != 0 && new_sign != 0 && new_sign != signo_ant);

        if (new_sign != 0 && (desde_parado || cambio_signo)) {
            ciclos_arranque = ARRANQUE_CICLOS;
        }
        if (new_sign != 0) signo_ant = new_sign;
    };

    needSoftStart(goal_vel_l, new_goal_l, filtered_vel_l,
                  signo_anterior_l, arranque_ciclos_l);
    needSoftStart(goal_vel_r, new_goal_r, filtered_vel_r,
                  signo_anterior_r, arranque_ciclos_r);

    goal_vel_l = new_goal_l;
    goal_vel_r = new_goal_r;

    // Reset integral si el setpoint va a cero (corte limpio)
    if (fabsf(goal_vel_l) < 0.001f) { pid_l.reset(); ramped_vel_l = 0.0f; }
    if (fabsf(goal_vel_r) < 0.001f) { pid_r.reset(); ramped_vel_r = 0.0f; }

    // Cuando llega setpoint=0, el ancla se evaluará en el timer
    if (fabsf(goal_vel_l) > 0.001f || fabsf(goal_vel_r) > 0.001f) ancla = false;

    Serial.printf("[CMD] lin=%.3f ang=%.3f | goalL=%.3f goalR=%.3f m/s\n",
                  lin, ang, goal_vel_l, goal_vel_r);
}

// =============================================================================
//  update_pose()
//  Odometría diferencial. Llamado desde timer_odometry_callback().
// =============================================================================

void JgaMotors::update_pose()
{
    const int64_t cur_l =  encoder_l.getCount();
    const int64_t cur_r = -encoder_r.getCount();   // invertido por montaje espejo

    const int64_t d_l = cur_l - prev_ticks_left;
    const int64_t d_r = cur_r - prev_ticks_right;
    prev_ticks_left  = cur_l;
    prev_ticks_right = cur_r;

    // Detección de vueltas y conmutación de LEDs
    checkFullTurns(d_l, d_r);

    // Odometría diferencial
    const float dL     = (float)d_l * METROS_POR_TICK;
    const float dR     = (float)d_r * METROS_POR_TICK;
    const float dC     = (dR + dL) / 2.0f;
    const float dTheta = (dR - dL) / WHEEL_BASE;

    x     += dC * cosf(theta + dTheta / 2.0f);
    y     += dC * sinf(theta + dTheta / 2.0f);
    theta += dTheta;

    if (theta >  PI) theta -= 2.0f * PI;
    if (theta < -PI) theta += 2.0f * PI;
}

// =============================================================================
//  timer_odometry_callback()
//  Se ejecuta cada 30 ms desde el executor micro-ROS.
//
//  Orden:
//    1. Calcular dt real
//    2. Leer encoders → velocidad medida → filtro paso-bajo
//    3. Actualizar rampa de setpoint
//    4. Lógica de ancla (freno cuando setpoint=0 y ya parado)
//    5. PID (o arranque suave si ciclos > 0)
//    6. Aplicar PWM
//    7. Publicar odometría y TF
// =============================================================================

void JgaMotors::timer_odometry_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL) return;

    // ── 1. Δt ─────────────────────────────────────────────────────────────────
    const int64_t t_ms = ros_synced_time_ms + (millis() - ms_before_sync);
    float dt = (prev_time_ms > 0)
               ? (float)(t_ms - prev_time_ms) / 1000.0f
               : 0.030f;
    if (dt <= 0.0f || dt > 1.0f) dt = 0.030f;
    prev_time_ms = t_ms;

    // ── 2. Leer encoders y velocidad medida ───────────────────────────────────
    const int64_t cur_l =  encoder_l.getCount();
    const int64_t cur_r = -encoder_r.getCount();

    const int64_t d_l = cur_l - prev_ticks_left;
    const int64_t d_r = cur_r - prev_ticks_right;

    // Velocidad cruda (m/s) — no actualiza prev_ticks_* aquí,
    // lo hace update_pose() a continuación para evitar doble descuento.
    const float raw_vel_l = ((float)d_l * METROS_POR_TICK) / dt;
    const float raw_vel_r = ((float)d_r * METROS_POR_TICK) / dt;

    // Filtro paso-bajo (igual que el .ino de referencia)
    filtered_vel_l = VEL_ALPHA * raw_vel_l + (1.0f - VEL_ALPHA) * filtered_vel_l;
    filtered_vel_r = VEL_ALPHA * raw_vel_r + (1.0f - VEL_ALPHA) * filtered_vel_r;

    // Actualizar pose y detectar vueltas (usa su propia lectura de getCount())
    update_pose();

    // ── 3. Rampa ──────────────────────────────────────────────────────────────
    updateRamp(dt);

    // ── 4. Lógica de ancla ────────────────────────────────────────────────────
    const bool goal_zero = (fabsf(goal_vel_l) < 0.001f && fabsf(goal_vel_r) < 0.001f);
    if (goal_zero &&
        fabsf(filtered_vel_l) <= BANDA_MUERTA_M_S &&
        fabsf(filtered_vel_r) <= BANDA_MUERTA_M_S) {
        ancla = true;
    }

    // ── 5 & 6. PID / arranque suave / ancla → PWM ─────────────────────────────
    int pwm_l_val = 0;
    int pwm_r_val = 0;

    if (goal_zero && ancla) {
        // Freno activo: ambos motores frenados, integral congelada
        motor_l->motorBrake();
        motor_r->motorBrake();
        pid_l.reset();
        pid_r.reset();
        ramped_vel_l = 0.0f;
        ramped_vel_r = 0.0f;

    } else if (arranque_ciclos_l > 0 || arranque_ciclos_r > 0) {
        // Arranque suave: pulso de MIN_PWM directo para vencer inercia estática
        // El PID sigue corriendo pero no se aplica su salida aún
        pid_l.compute(ramped_vel_l, filtered_vel_l, dt);
        pid_r.compute(ramped_vel_r, filtered_vel_r, dt);

        if (arranque_ciclos_l > 0) {
            pwm_l_val = MIN_PWM_REAL * signo_anterior_l;
            arranque_ciclos_l--;
        } else {
            pwm_l_val = pwmFromPid(pid_l.compute(ramped_vel_l, filtered_vel_l, dt));
        }
        if (arranque_ciclos_r > 0) {
            pwm_r_val = MIN_PWM_REAL * signo_anterior_r;
            arranque_ciclos_r--;
        } else {
            pwm_r_val = pwmFromPid(pid_r.compute(ramped_vel_r, filtered_vel_r, dt));
        }
        applyMotorPwm(pwm_l_val, pwm_r_val);

    } else {
        // Control PID normal
        const float out_l = pid_l.compute(ramped_vel_l, filtered_vel_l, dt);
        const float out_r = pid_r.compute(ramped_vel_r, filtered_vel_r, dt);

        // Fix 3: reset independiente por saturación.
        // Antes se reseteaban AMBOS PIDs si cualquiera saturaba. Eso causaba que
        // el motor izquierdo (con setpoint casi cero y sin saturar) se
        // desestabilizara cada vez que el derecho saturaba en giros cerrados.
        if (fabsf(out_l) >= 254.0f) pid_l.reset();
        if (fabsf(out_r) >= 254.0f) pid_r.reset();

        pwm_l_val = pwmFromPid(out_l);
        pwm_r_val = pwmFromPid(out_r);
        applyMotorPwm(pwm_l_val, pwm_r_val);
    }

    // ── 7. Debug PID cada ~0.5 s (16 ciclos × 30 ms) ─────────────────────────
    static uint8_t dbg = 0;
    if (++dbg >= 16) {
        dbg = 0;
        Serial.printf("[PID] goalL=%+.3f rampL=%+.3f measL=%+.3f(%.1fRPM) pwmL=%d | "
                      "goalR=%+.3f rampR=%+.3f measR=%+.3f(%.1fRPM) pwmR=%d\n",
                      goal_vel_l, ramped_vel_l, filtered_vel_l,
                      filtered_vel_l * RPM_POR_M_S, pwm_l_val,
                      goal_vel_r, ramped_vel_r, filtered_vel_r,
                      filtered_vel_r * RPM_POR_M_S, pwm_r_val);
    }

    // ── 8. Publicar odometría ─────────────────────────────────────────────────
    odom_msg.header.stamp.sec     = (int32_t)(t_ms / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((t_ms % 1000) * 1000000UL);
    odom_msg.header.frame_id.data = const_cast<char *>("odom");
    odom_msg.header.frame_id.size = 4;
    odom_msg.child_frame_id.data  = const_cast<char *>("base_link");
    odom_msg.child_frame_id.size  = 9;

    odom_msg.pose.pose.position.x    = x;
    odom_msg.pose.pose.position.y    = y;
    odom_msg.pose.pose.position.z    = 0.0f;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(theta / 2.0f);

    odom_msg.twist.twist.linear.x  = (filtered_vel_l + filtered_vel_r) / 2.0f;
    odom_msg.twist.twist.angular.z = (filtered_vel_r - filtered_vel_l) / WHEEL_BASE;

    if (odom_pub.impl != NULL)
        rcl_publish(&odom_pub, &odom_msg, NULL);

    // ── 9. Publicar TF ────────────────────────────────────────────────────────
    tf_msg.transforms.data[0].header.stamp.sec        = odom_msg.header.stamp.sec;
    tf_msg.transforms.data[0].header.stamp.nanosec    = odom_msg.header.stamp.nanosec;
    tf_msg.transforms.data[0].header.frame_id.data    = const_cast<char *>("odom");
    tf_msg.transforms.data[0].header.frame_id.size    = 4;
    tf_msg.transforms.data[0].child_frame_id.data     = const_cast<char *>("base_link");
    tf_msg.transforms.data[0].child_frame_id.size     = 9;
    tf_msg.transforms.data[0].transform.translation.x = x;
    tf_msg.transforms.data[0].transform.translation.y = y;
    tf_msg.transforms.data[0].transform.translation.z = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.x    = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.y    = 0.0f;
    tf_msg.transforms.data[0].transform.rotation.z    = odom_msg.pose.pose.orientation.z;
    tf_msg.transforms.data[0].transform.rotation.w    = odom_msg.pose.pose.orientation.w;

    if (tf_pub.impl != NULL)
        rcl_publish(&tf_pub, &tf_msg, NULL);
}