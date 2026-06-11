// #include <Arduino.h>
// #include <vector>
// #include <stdint.h>

// #define RXD2 17  // Rx pin for Serial2
// #define TXD2 -1  // Tx pin for Serial2 (not used in this example)

// // Data frame constants
// const uint8_t FRAME_HEADER = 0xAA;
// const uint8_t PROTOCOL_VERSION = 0x01;
// const uint8_t FRAME_TYPE = 0x61;
// const int SCAN_STEPS = 15;
// const uint8_t FINAL_FRAME = 0xFF;

// // Post processing constants
// const float ROTATION_SPEED_SCALE = 0.05f * 60.0f;
// const float ANGLE_SCALE = 0.01f;
// const float RANGE_SCALE = 0.25f * 0.001f;

// // Data buffer
// std::vector<uint16_t> distance_vector;
// uint16_t initial_angle = 65535; // Initial angle in degrees
// uint16_t initial_angle_lower_range, initial_angle_upper_range; // Initial angle lower and upper range in degrees

// // Data structures for scan samples
// struct XiaomiLidarSample {
//     uint8_t frameHeader = 0;
//     uint16_t frameLength = 0;
//     uint8_t protocolVersion = 0;
//     uint8_t frameType = 0;
//     uint8_t commandWord = 0;
//     uint16_t parameterLength = 0;
//     std::vector<uint8_t> parameters;
//     uint16_t checksum = 0;
// };

// // Process data frame after receiving it
// void LiDARFrameProcessing(const XiaomiLidarSample& frame) {
    
//     switch (frame.commandWord) {
//         // 0xAE indicates that the frame only contains the rotation speed of the Lidar (RPM).
//         case 0xAE: {
//             break;
//         }
//         // 0xAD indicates that the frame contains RPM, start angle and the distance samples.
//         case 0xAD: {
//             uint16_t raw_start_angle = (frame.parameters[3] << 8) | frame.parameters[4];
//             // Initial angle is set to 65535 if not set before, which indicates that the start angle is not known.
//             if (initial_angle == 65535) {
//                 initial_angle = raw_start_angle;
//                 initial_angle_lower_range = raw_start_angle - 2000; // 20 degrees lower range
//                 initial_angle_upper_range = raw_start_angle + 2000; // 20 degrees upper range
//                 if (initial_angle < 2000) {
//                     initial_angle_lower_range = 0; // 20 degrees lower range
//                 }
//                 if (initial_angle > (65535 - 2000)) {
//                     initial_angle_upper_range = 65535; // 20 degrees upper range
//                 }
//             }

//             int sampleCnt = static_cast<int>((frame.parameterLength - 5) / 3);

//             // if the start angle is within the initial angle range, clear the distance vector
//             // because a complete scan has been done and the next scan will start from the current sample.
//             if (raw_start_angle > initial_angle_lower_range && raw_start_angle < initial_angle_upper_range) {
//                 // send the distance vector to the PC
//                 Serial.write(FRAME_HEADER); // Frame Header
//                 uint16_t distance_length = distance_vector.size() * 2; // Each distance sample is 2 bytes
//                 Serial.write((uint8_t)(distance_length >> 8)); // MSB of frame length
//                 Serial.write((uint8_t)(distance_length & 0x00FF)); // LSB of frame length
//                 //Serial.printf("Frame size: %d\n", distance_vector.size());
//                 //Serial.printf("Start angle: %d\n", raw_start_angle);
                

//                 Serial.write((uint8_t)(raw_start_angle >> 8));   // MSB
//                 Serial.write((uint8_t)(raw_start_angle & 0x00FF)); // LSB
//                 if (!distance_vector.empty()) {
//                     for (uint16_t value : distance_vector) {
//                         Serial.write((uint8_t)(value >> 8));   // MSB
//                         Serial.write((uint8_t)(value & 0x00FF)); // LSB
//                         //Serial.printf("Distance: %d\n", value);
//                     }
//                 }
//                 distance_vector.clear();
//             }
//             // Push the nuew samples to the distance vector
//             for (int i = 0; i < sampleCnt; i++) {
//                 uint16_t distance = (frame.parameters[5 + i * 3 + 1] << 8) | frame.parameters[5 + i * 3 + 2];
//                 distance_vector.push_back(distance);
//             }
//             break;
//         }
//         default:
//             //Serial.println("UNKNOWN COMMAND WORD");
//             break;
//     }
// }

// void setup() {
//     // Serial used to send data to PC
//     Serial.begin(115200);       
//     // Serial2 used to communicate with the Lidar
//     Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  
//     // Serial.println("Initialized ESP32 Xiaomi Lidar Scanner"); 
//     // Reserve space for distance samples vector
//     distance_vector.reserve(510); // Reserve space for 510 samples (255 * 2 bytes each)
// }

// void loop() {
//     static int status = 0;
//     static uint16_t checksum = 0;
//     static XiaomiLidarSample frame;
//     /* A dataframe is formed by:
//         + The header, which is 0xAA
//         + The frame length, which is 2 bytes (MSB and LSB)
//         + The protocol version, which is 1 byte
//         + The frame type, which is 1 byte
//         + The command word, which is 1 byte
//         + The payload length, which is 2 bytes (MSB and LSB)
//         + The payload data, which is variable length
//         + The checksum, which is 2 bytes (MSB and LSB)
//         + The checksum calculation does not include the last 2 bytes of the frame (the proper checksum bytes)
//     */
//     // Reserve space for parameters which enables fastest access. 
//     // The number of parameters is variable, but we reserve a maximum size that will never be exceeded.
//     frame.parameters.reserve(255); 
//     while (Serial2.available()) {
//         uint8_t by = Serial2.read();
//         switch (status) {
//             // 1st frame byte: Frame Header
//             case 0:
//                 frame.frameHeader = by;
//                 if (frame.frameHeader == FRAME_HEADER) {
//                     // Initialize checksum
//                     checksum = by;
//                     status = 1;
//                 } else {
//                     // Serial.println("ERROR: Frame Header Failed");
//                 }
//                 break;
//             // 2nd frame byte: Frame Length MSB
//             case 1:
//                 frame.frameLength = by << 8;
//                 checksum += by;
//                 status = 2;
//                 break;
//             // 3rd frame byte: Frame Length LSB
//             case 2:
//                 frame.frameLength |= by;
//                 checksum += by;
//                 status = 3;
//                 break;
//             // 4th frame byte: Protocol Version
//             case 3:
//                 frame.protocolVersion = by;
//                 checksum += by;
//                 if (frame.protocolVersion == PROTOCOL_VERSION) {
//                     status = 4;
//                 } else {
//                     // Serial.println("ERROR: Protocol Version");
//                     status = 0;
//                 }
//                 break;
//             // 5th frame byte: Frame Type
//             case 4:
//                 frame.frameType = by;
//                 checksum += by;
//                 if (frame.frameType == FRAME_TYPE) {
//                     status = 5;
//                 } else {
//                     // Serial.println("ERROR: Frame Type");
//                     status = 0;
//                 }
//                 break;
//             // 6th frame byte: Command Word
//             case 5:
//                 frame.commandWord = by;
//                 checksum += by;
//                 status = 6;
//                 break;
//             // 7th frame byte: Payload Length MSB
//             case 6:
//                 frame.parameterLength = by << 8;
//                 checksum += by;
//                 status = 7;
//                 break;
//             // 8th frame byte: Payload Length LSB
//             case 7:
//                 frame.parameterLength |= by;
//                 checksum += by;
//                 frame.parameters.clear();
//                 status = 8;
//                 break;
//             // 9th to N-3th frame bytes: Payload Data
//             case 8:
//                 frame.parameters.push_back(by);
//                 checksum += by;
//                 if (frame.parameters.size() == frame.parameterLength) {
//                     status = 9;
//                 }
//                 break;
//             // N-2th frame byte: Checksum MSB
//             case 9:
//                 frame.checksum = by << 8;
//                 status = 10;
//                 break;
//             // N-1th frame byte: Checksum LSB
//             case 10:
//                 frame.checksum |= by;
//                 // Checksum does not include last 2 bytes
//                 if (frame.checksum == checksum) {
//                     // Process the frame
//                     LiDARFrameProcessing(frame);
//                 } else {
//                     // Serial.println("ERROR: Checksum mismatch");
//                 }
//                 status = 0;
//                 break;
//         }
//         if (status < 10 && status > 0) {
//             checksum %= 0xFFFF;
//         }
//     }
// }