#include "../include/jga_motors.hpp"

// =============================================================================
//  Constructor / Destructor
// =============================================================================

JgaMotors::JgaMotors()
    : x(0.0f), y(0.0f), theta(0.0f),
      posAnterior_l(0), posAnterior_r(0),
      tAnterior(0)
{
}

JgaMotors::~JgaMotors()
{
    delete motor_l;
    delete motor_r;
}

// =============================================================================
//  Funciones Auxiliares calcadas del .ino
// =============================================================================

float JgaMotors::msToRPM(float m_s) {
    return (m_s * 60.0f) / PERIMETRO_RUEDA_M;
}

int JgaMotors::aplicarZonaMuerta(int pwm) {
    if (pwm == 0) return 0;
    int signo = (pwm > 0) ? 1 : -1;
    return map(abs(pwm), 1, 255, MIN_PWM_REAL, 255) * signo;
}

void JgaMotors::resetPID() {
    pid_l.setTunings(global_kp, global_ki, global_kd);
    pid_r.setTunings(global_kp, global_ki, global_kd);
    pid_l.reset();
    pid_r.reset();
}

void JgaMotors::arrancarMotor(float sp_l, float sp_r) {
    resetPID();
    setpointRampaRPM_l = ((sp_l > 0) ? 1 : -1) * RAMPA_CAMBIO_RPM;
    setpointRampaRPM_r = ((sp_r > 0) ? 1 : -1) * RAMPA_CAMBIO_RPM;
    ciclosArranque_l = CICLOS_ARRANQUE;
    ciclosArranque_r = CICLOS_ARRANQUE;
}

// =============================================================================
//  init()
// =============================================================================

void JgaMotors::init()
{
    motor_l = new MX1508(IN1_L, IN2_L, SLOW_DECAY);
    motor_r = new MX1508(IN1_R, IN2_R, SLOW_DECAY);

    pinMode(LED_RED,  OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    digitalWrite(LED_RED,  LOW);
    digitalWrite(LED_BLUE, LOW);

    pinMode(ENC_A_L, INPUT_PULLUP);
    pinMode(ENC_B_L, INPUT_PULLUP);
    encoder_l.attachFullQuad(ENC_A_L, ENC_B_L);
    encoder_l.clearCount();
    encoder_l.setFilter(1023);

    pinMode(ENC_A_R, INPUT_PULLUP);
    pinMode(ENC_B_R, INPUT_PULLUP);
    encoder_r.attachFullQuad(ENC_A_R, ENC_B_R);
    encoder_r.clearCount();
    encoder_r.setFilter(1023);

    motor_l->motorBrake();
    motor_r->motorBrake();

    // Reserva de memoria TF micro-ROS
    tf_msg.transforms.capacity = 1;
    tf_msg.transforms.size     = 1;
    tf_msg.transforms.data     = (geometry_msgs__msg__TransformStamped *)malloc(
        sizeof(geometry_msgs__msg__TransformStamped));
    memset(tf_msg.transforms.data, 0, sizeof(geometry_msgs__msg__TransformStamped));

    estadoActual = ESTADO_STOPPED;
    tAnterior = millis();
}

// =============================================================================
//  emergency_stop()
// =============================================================================

void JgaMotors::emergency_stop()
{
    estadoActual = ESTADO_STOPPED;
    if (motor_l) motor_l->motorBrake();
    if (motor_r) motor_r->motorBrake();
    setpointObjetivoRPM_l = 0;
    setpointObjetivoRPM_r = 0;
    setpointRampaRPM_l = 0;
    setpointRampaRPM_r = 0;
    resetPID();
}

// =============================================================================
//  cmd_vel_sub_callback() - Transiciones Iniciales y Zonas de Parada
// =============================================================================

void JgaMotors::cmd_vel_sub_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    float lin = msg->linear.x;
    float ang = msg->angular.z;

    // Cinemática diferencial a m/s
    float v_l = lin - (ang * WHEEL_BASE / 2.0f);
    float v_r = lin + (ang * WHEEL_BASE / 2.0f);

    float limiteSup = VEL_MIN_MS - 0.01f;
    float limiteInf = -VEL_MIN_MS + 0.01f;

    bool req_parada_l = (v_l > limiteInf && v_l < limiteSup);
    bool req_parada_r = (v_r > limiteInf && v_r < limiteSup);

    if (req_parada_l && req_parada_r) {
        // Zona de parada global
        if (estadoActual == ESTADO_NORMAL) {
            estadoActual = ESTADO_PARANDO;
            
            int signo_l = (setpointRampaRPM_l >= 0.0f) ? 1 : -1;
            int signo_r = (setpointRampaRPM_r >= 0.0f) ? 1 : -1;
            if (v_l > 0.001f) signo_l = 1; else if (v_l < -0.001f) signo_l = -1;
            if (v_r > 0.001f) signo_r = 1; else if (v_r < -0.001f) signo_r = -1;

            setpointObjetivoRPM_l = msToRPM(VEL_MIN_MS) * signo_l;
            setpointObjetivoRPM_r = msToRPM(VEL_MIN_MS) * signo_r;
        }
    } else {
        // Zona ESTADO_NORMAL
        v_l = constrain(v_l, -VEL_MAX_MS, VEL_MAX_MS);
        v_r = constrain(v_r, -VEL_MAX_MS, VEL_MAX_MS);
        
        float nuevoSP_l = msToRPM(v_l);
        float nuevoSP_r = msToRPM(v_r);
        int nuevoSigno_l = (nuevoSP_l > 0) ? 1 : -1;
        int nuevoSigno_r = (nuevoSP_r > 0) ? 1 : -1;

        if (estadoActual == ESTADO_STOPPED || estadoActual == ESTADO_FRENO || 
           (signoAnterior_l != 0 && nuevoSigno_l != signoAnterior_l) ||
           (signoAnterior_r != 0 && nuevoSigno_r != signoAnterior_r)) {
            arrancarMotor(nuevoSP_l, nuevoSP_r);
        }

        setpointObjetivoRPM_l = nuevoSP_l;
        setpointObjetivoRPM_r = nuevoSP_r;
        estadoActual = ESTADO_NORMAL;
        signoAnterior_l = nuevoSigno_l;
        signoAnterior_r = nuevoSigno_r;
    }
}

// =============================================================================
//  Odometría y Cinemática
// =============================================================================

void JgaMotors::update_pose(long deltaTicks_l, long deltaTicks_r)
{
    float dL = ((float)deltaTicks_l / TICKS_POR_VUELTA) * PERIMETRO_RUEDA_M;
    float dR = ((float)deltaTicks_r / TICKS_POR_VUELTA) * PERIMETRO_RUEDA_M;
    float dC = (dR + dL) / 2.0f;
    float dTheta = (dR - dL) / WHEEL_BASE;

    x += dC * cosf(theta + dTheta / 2.0f);
    y += dC * sinf(theta + dTheta / 2.0f);
    theta += dTheta;

    if (theta >  PI) theta -= 2.0f * PI;
    if (theta < -PI) theta += 2.0f * PI;
}

// =============================================================================
//  timer_odometry_callback() - Corazón del Control (.ino Loop)
// =============================================================================

void JgaMotors::timer_odometry_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    if (timer == NULL) return;

    // --- 2. PERIODO DE CONTROL ---
    unsigned long tActual = millis();
    if ((tActual - tAnterior) < PERIODO_MS) return;
    unsigned long dt = tActual - tAnterior;
    tAnterior = tActual;

    const int64_t t_ms = ros_synced_time_ms + (tActual - ms_before_sync);

    // --- 3. LECTURA DE ENCODERS ---
    long posActual_l = encoder_l.getCount();
    long posActual_r = -encoder_r.getCount();
    
    long deltaTicks_l = posActual_l - posAnterior_l;
    long deltaTicks_r = posActual_r - posAnterior_r;

    posAnterior_l = posActual_l;
    posAnterior_r = posActual_r;

    float rpmActual_l = ((float)deltaTicks_l / TICKS_POR_VUELTA) * (60000.0f / dt);
    float rpmActual_r = ((float)deltaTicks_r / TICKS_POR_VUELTA) * (60000.0f / dt);

    rpmFiltrado_l = ALPHA * rpmActual_l + (1.0f - ALPHA) * rpmFiltrado_l;
    rpmFiltrado_r = ALPHA * rpmActual_r + (1.0f - ALPHA) * rpmFiltrado_r;

    digitalWrite(LED_RED,  (posActual_l / (long)TICKS_POR_VUELTA) % 2 == 0 ? HIGH : LOW);
    digitalWrite(LED_BLUE, (posActual_r / (long)TICKS_POR_VUELTA) % 2 == 0 ? HIGH : LOW);

    // --- 4. TRANSICIONES INTERNAS DE LA MÁQUINA DE ESTADOS ---
    if (estadoActual == ESTADO_PARANDO) {
        float rpmTargetAbs = msToRPM(VEL_MIN_MS);
        
        bool rampaLista_l = fabsf(fabsf(setpointRampaRPM_l) - rpmTargetAbs) <= 0.1f;
        bool rampaLista_r = fabsf(fabsf(setpointRampaRPM_r) - rpmTargetAbs) <= 0.1f;
        bool rpmLista_l = fabsf(rpmFiltrado_l) <= rpmTargetAbs + BANDA_MUERTA_RPM;
        bool rpmLista_r = fabsf(rpmFiltrado_r) <= rpmTargetAbs + BANDA_MUERTA_RPM;

        if (rampaLista_l && rampaLista_r && rpmLista_l && rpmLista_r) {
            estadoActual = ESTADO_FRENO;
        }
    }

    if (estadoActual == ESTADO_FRENO) {
        motor_l->motorBrake();
        motor_r->motorBrake();
        
        setpointRampaRPM_l    = 0.0f;
        setpointRampaRPM_r    = 0.0f;
        setpointObjetivoRPM_l = 0.0f;
        setpointObjetivoRPM_r = 0.0f;
        ciclosArranque_l = 0;
        ciclosArranque_r = 0;
        
        resetPID();
        estadoActual = ESTADO_STOPPED;
    }

    // --- 5. RAMPA DE ACELERACIÓN ---
    if (estadoActual == ESTADO_NORMAL || estadoActual == ESTADO_PARANDO) {
        float errorRampa_l = setpointObjetivoRPM_l - setpointRampaRPM_l;
        if (fabsf(errorRampa_l) <= RAMPA_CAMBIO_RPM) setpointRampaRPM_l = setpointObjetivoRPM_l;
        else setpointRampaRPM_l += (errorRampa_l > 0) ? RAMPA_CAMBIO_RPM : -RAMPA_CAMBIO_RPM;

        float errorRampa_r = setpointObjetivoRPM_r - setpointRampaRPM_r;
        if (fabsf(errorRampa_r) <= RAMPA_CAMBIO_RPM) setpointRampaRPM_r = setpointObjetivoRPM_r;
        else setpointRampaRPM_r += (errorRampa_r > 0) ? RAMPA_CAMBIO_RPM : -RAMPA_CAMBIO_RPM;
    }

    // --- 6. ACTUACIÓN DE LOS MOTORES ---
    if (estadoActual == ESTADO_STOPPED) {
        motor_l->motorBrake();
        motor_r->motorBrake();
    } 
    else if (ciclosArranque_l > 0 || ciclosArranque_r > 0) {
        // PID Interno corriendo mientras aplica PWM ciego
        pid_l.compute(setpointRampaRPM_l, rpmFiltrado_l);
        pid_r.compute(setpointRampaRPM_r, rpmFiltrado_r);

        int signo_l = (setpointObjetivoRPM_l > 0) ? 1 : -1;
        int signo_r = (setpointObjetivoRPM_r > 0) ? 1 : -1;
        
        if (ciclosArranque_l > 0) { motor_l->motorGo(MIN_PWM_REAL * signo_l);  ciclosArranque_l--; }
        if (ciclosArranque_r > 0) { motor_r->motorGo(-(MIN_PWM_REAL * signo_r)); ciclosArranque_r--; }
    } 
    else {
        float output_l = pid_l.compute(setpointRampaRPM_l, rpmFiltrado_l);
        float output_r = pid_r.compute(setpointRampaRPM_r, rpmFiltrado_r);

        int pwm_l = constrain((int)output_l, -255, 255);
        int pwm_r = constrain((int)output_r, -255, 255);

        motor_l->motorGo(aplicarZonaMuerta(pwm_l));
        motor_r->motorGo(-aplicarZonaMuerta(pwm_r));
    }

    // --- 7. ACTUALIZAR ODOMETRÍA Y PUBLICAR ROS ---
    update_pose(deltaTicks_l, deltaTicks_r);

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

    float velReal_l_ms = (rpmFiltrado_l / 60.0f) * PERIMETRO_RUEDA_M;
    float velReal_r_ms = (rpmFiltrado_r / 60.0f) * PERIMETRO_RUEDA_M;

    odom_msg.twist.twist.linear.x  = (velReal_l_ms + velReal_r_ms) / 2.0f;
    odom_msg.twist.twist.angular.z = (velReal_r_ms - velReal_l_ms) / WHEEL_BASE;

    if (odom_pub.impl != NULL)
        rcl_publish(&odom_pub, &odom_msg, NULL);

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