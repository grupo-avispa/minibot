from socket import timeout
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import struct
import time
#Serial port variables
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 115200

#Delta-2G Frame Characteristics
#Constant Frame Parts values
FRAME_HEADER = 0xAA			#Frame Header value
PROTOCOL_VERSION = 0x01		#Protocol Version value
FRAME_TYPE = 0x61			#Frame Type value
#Scan Characteristics
SCAN_STEPS = 15				#How many steps/frames each full 360deg scan is composed of
#Received value scaling
ROTATION_SPEED_SCALE = 0.05 * 60	#Convert received value to RPM (LSB: 0.05 rps)
ANGLE_SCALE = 0.01			#Convert received value to degrees (LSB: 0.01 degrees)
RANGE_SCALE = 0.25 * 0.001	#Convert received value to meters (LSB: 0.25 mm)

plt.ion()  # Modo interactivo para actualizar sin bloquear
fig, ax = plt.subplots(figsize=(6, 6))

# Inicializamos el scatter con datos vacíos
scatter = ax.scatter([], [], s=5, c='blue')
ax.set_xlim(-11, 11)
ax.set_ylim(-11, 11)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Lecturas LiDAR en 2D')
ax.axis('equal')
ax.grid(True)

				
def main():
	try:
		lidarSerial = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=10)
	except serial.serialutil.SerialException:
		print("ERROR: Serial Connect Error")
		return
	last_frame_time = time.time()
	while True:
		header = lidarSerial.read(1)
		if not header or header[0] != FRAME_HEADER:
			continue
		print("Frame Header received: 0x{:02X}".format(header[0]))
  
		length_bytes = lidarSerial.read(2)
		if not length_bytes or len(length_bytes) < 2:
			continue
		frame_length = struct.unpack('>H', length_bytes)[0]
		if frame_length < 544 or frame_length > 550:
			print(f"Invalid frame size: {frame_length}")
			continue
		print(f"Frame Size: {frame_length}")
  
		start_angle_bytes = lidarSerial.read(2)
		if len(start_angle_bytes) < 2:
			print("ERROR: Start angle not received")
			continue
		startAngle_raw = struct.unpack('>H', start_angle_bytes)[0]
		startAngle = startAngle_raw * ANGLE_SCALE
		print(f"StartAngle: {startAngle}")

		raw_distances = lidarSerial.read(frame_length)
		if len(raw_distances) < frame_length:
			print(f"ERROR: Not enough distance data received. Expected {frame_length}, got {len(raw_distances)}")
			continue
		
		number_of_distances = len(raw_distances) // 2
		distances = list(struct.unpack('>' + 'H' * number_of_distances, raw_distances)) 
		distances = np.array(distances) * RANGE_SCALE
  
		# Crear ángulos uniformemente distribuidos de 0 a 360 grados (en radianes)
		angles = np.deg2rad(np.linspace(startAngle, startAngle + 360, len(distances), endpoint=False))
		# Convertir coordenadas polares (distancia, ángulo) a cartesianas (x, y)
		x = distances * np.cos(angles)
		y = distances * np.sin(angles)
		puntos = np.c_[x, y]

		scatter.set_offsets(puntos)  # Actualizamos las posiciones de los puntos
		fig.canvas.draw_idle()
		plt.pause(0.001)  # Permite que la ventana se refresque
		print(f"Lectre elapsed: {time.time() - last_frame_time:.4f} seconds")
		last_frame_time = time.time()
			

if __name__ == "__main__":
	main()

