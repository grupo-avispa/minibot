# Minibot

Proyecto de control y sensorización para un robot diferencial basado en ESP32, con integración de micro-ROS por WiFi, motores NEMA y LiDAR Xiaomi. El repositorio incluye tres firmwares de PlatformIO: uno completo (motores + LiDAR) y dos variantes de prueba (solo LiDAR y solo motores).

## Estructura

- [minibot/](minibot): firmware completo (motores + LiDAR) con micro-ROS.
- [minibot_lidar/](minibot_lidar): firmware solo LiDAR publicando `sensor_msgs/LaserScan`.
- [minibot_nema/](minibot_nema): firmware solo motores NEMA con odometría y `cmd_vel`.

## Requisitos

- ESP32
- LiDAR Xiaomi (serie compatible con protocolo 0xAA/0x61)
- Motores NEMA + drivers tipo STEP/DIR
- PlatformIO
- micro-ROS Agent en un PC con ROS 2
- Python 3 (solo para el script de visualización del LiDAR)

## Configuración

1. Ajusta WiFi y el agente micro-ROS:
	- [minibot/src/main.cpp](minibot/src/main.cpp)
	- [minibot_lidar/src/main.cpp](minibot_lidar/src/main.cpp)
	- [minibot_nema/src/main.cpp](minibot_nema/src/main.cpp)

2. Revisa los pines de hardware:
	- Motores NEMA en [minibot/src/nema_motors.cpp](minibot/src/nema_motors.cpp) y [minibot_nema/src/main.cpp](minibot_nema/src/main.cpp).
	- LiDAR en [minibot/src/xiaomi_lidar.cpp](minibot/src/xiaomi_lidar.cpp) y [minibot_lidar/src/main.cpp](minibot_lidar/src/main.cpp).

## Compilación y carga

Cada carpeta es un proyecto independiente de PlatformIO. En la carpeta correspondiente puedes compilar, subir y monitorizar el firmware.

Ejemplo (en la carpeta del proyecto):

```
pio run
pio run -t upload
pio device monitor -b 115200
```

## Topics y frames (micro-ROS)

### Firmware completo: [minibot/](minibot)

- `sensor_msgs/LaserScan` en `/scan`
- `nav_msgs/Odometry` en `/minibot/odom`
- `tf2_msgs/TFMessage` en `/tf`
- `geometry_msgs/Twist` en `/minibot/cmd_vel`

Frames principales:
- `base_laser_link`
- `base_link`
- `odom`

### Solo LiDAR: [minibot_lidar/](minibot_lidar)

- Publica `sensor_msgs/LaserScan` en `/scan`

### Solo motores: [minibot_nema/](minibot_nema)

- Suscribe `geometry_msgs/Twist` en `/minibot/cmd_vel`
- Publica `nav_msgs/Odometry` en `/minibot/odom`
- Publica `tf2_msgs/TFMessage` en `/tf`

## Visualización del LiDAR por serie

Script: [minibot_lidar/plot_lidar_serial.py](minibot_lidar/plot_lidar_serial.py)

Dependencias:

```
pip install pyserial numpy matplotlib
```

Notas:
- Ajusta `SERIAL_PORT` si es necesario.
- El script grafica en 2D las distancias recibidas por serie.

## Notas de implementación

- El firmware completo crea tareas FreeRTOS para lectura del LiDAR, publicación ROS y ejecución del `executor`.
- La odometría se calcula por pasos de los motores NEMA.
- La sincronización temporal usa un temporizador para alineación con el agente micro-ROS.

## Licencia

Pendiente de definir.