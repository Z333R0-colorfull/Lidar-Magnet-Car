#include <WiFi.h>
#include <math.h>
#include <Adafruit_MMC56x3.h>


const char* ssid = "RC_CAR_AP";
const char* password = "12345678";

WiFiServer controlServer(8889);   
WiFiServer lidarServer(8888);    
WiFiServer magServer(8890);       

WiFiClient controlClient;
WiFiClient lidarClient;
WiFiClient magClient;

bool controlConnected = false;
bool lidarConnected = false;
bool magConnected = false;


const int pwmPins[3] = {3, 6, 2};
const int dirPins[3] = {10, 7, 4};

const int ledcChannels[3] = {0, 1, 2};
const int ledcFreq = 5000;
const int ledcRes = 10;  

const float robotRadius = 0.1;   
const float maxSpeed = 1.0;      
#define SQRT3_2 0.86602540378f


#define LIDAR_BAUD 115200
#define LIDAR_RX_PIN 5
#define LIDAR_TX_PIN 11

enum LidarParseState {
  WAIT_HEADER1, WAIT_HEADER2, READ_CT, READ_LSN, READ_FSA_L, READ_FSA_H,
  READ_LSA_L, READ_LSA_H, READ_CS_L, READ_CS_H, READ_DATA
};

LidarParseState parseState = WAIT_HEADER1;
uint8_t packetBuffer[2560];
uint16_t packetIndex = 0;
uint8_t expectedLSN = 0;
uint16_t expectedPacketLen = 0;
unsigned long lastLidarTime = 0;
const unsigned long lidarTimeout = 100;


Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
unsigned long lastMagRead = 0;
const int magIntervalMs = 10;   // 100 Hz 采样率


void processLidarByte(uint8_t c);
void parseAndSendLidarData(uint8_t* buf, uint16_t len);
void sendLidarDataViaWiFi(float* distances, float* angles, int count);
void setWheelOutput(float speeds[3]);
void kinematics(float vx, float vy, float omega);
void readAndSendMagnetometer();


void setup() {
  
 

  
  for (int i = 0; i < 3; i++) {
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(dirPins[i], LOW);
    ledcSetup(ledcChannels[i], ledcFreq, ledcRes);
    ledcAttachPin(pwmPins[i], ledcChannels[i]);
    ledcWrite(ledcChannels[i], 0);
  }

  
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  

  
  Wire.begin(8, 9);
  if (!mmc.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
    
  } else {
    mmc.printSensorDetails();
    mmc.setDataRate(100);          
    mmc.setContinuousMode(true);
    
  }

  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  
  
  controlServer.begin();
  lidarServer.begin();
  magServer.begin();
  
}


void loop() {
  
  if (!controlConnected) {
    controlClient = controlServer.available();
    if (controlClient && controlClient.connected()) {
      controlConnected = true;
      
    }
  } else {
    if (!controlClient.connected()) {
      controlClient.stop();
      controlConnected = false;
      
    } else {
      if (controlClient.available() >= 12) {
        uint8_t buf[12];
        controlClient.readBytes(buf, 12);
        float vx, vy, omega;
        memcpy(&vx, buf, 4);
        memcpy(&vy, buf+4, 4);
        memcpy(&omega, buf+8, 4);
        vx = constrain(vx, -maxSpeed, maxSpeed);
        vy = constrain(vy, -maxSpeed, maxSpeed);
        omega = constrain(omega, -maxSpeed/robotRadius, maxSpeed/robotRadius);
        kinematics(vx, vy, omega);
      }
    }
  }

  
  if (!lidarConnected) {
    lidarClient = lidarServer.available();
    if (lidarClient && lidarClient.connected()) {
      lidarConnected = true;
      
    }
  } else {
    if (!lidarClient.connected()) {
      lidarClient.stop();
      lidarConnected = false;
      
    }
  }

  
  if (!magConnected) {
    magClient = magServer.available();
    if (magClient && magClient.connected()) {
      magConnected = true;
      
    }
  } else {
    if (!magClient.connected()) {
      magClient.stop();
      magConnected = false;
      
    }
  }

  
  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();
    processLidarByte(c);
  }

  
  readAndSendMagnetometer();
}


void setWheelOutput(float speeds[3]) {
  for (int i = 0; i < 3; i++) {
    float spd = constrain(speeds[i], -maxSpeed, maxSpeed);
    if(spd<0)
    {
      spd*=1.1;
    }
    uint16_t duty = (uint16_t)((fabs(spd) / maxSpeed) * 1023.0);
    if (spd > 0.15) {
      digitalWrite(dirPins[i], LOW);
      ledcWrite(ledcChannels[i], duty);
    } else if (spd < -0.15) {
      digitalWrite(dirPins[i], HIGH);
      ledcWrite(ledcChannels[i], 1023 - duty);
    } else {
      ledcWrite(ledcChannels[i], 1023);
      digitalWrite(dirPins[i], HIGH);
    }
  }
}

void kinematics(float vx, float vy, float omega) {
  float wheelSpeeds[3];
  float wR = omega * robotRadius;
  wheelSpeeds[0] = 1.2*vy + wR;
  wheelSpeeds[1] = -0.5f * vy + SQRT3_2 * vx + wR;
  wheelSpeeds[2] = -0.5f * vy - SQRT3_2 * vx + wR;
  wheelSpeeds[2] *= 1.2;   // 电机特性补偿
  setWheelOutput(wheelSpeeds);
}


void processLidarByte(uint8_t c) {
  if (millis() - lastLidarTime > lidarTimeout) {
    parseState = WAIT_HEADER1;
    packetIndex = 0;
  }
  lastLidarTime = millis();

  switch (parseState) {
    case WAIT_HEADER1:
      if (c == 0xAA) {
        packetBuffer[0] = c;
        packetIndex = 1;
        parseState = WAIT_HEADER2;
      }
      break;
    case WAIT_HEADER2:
      if (c == 0x55) {
        packetBuffer[1] = c;
        packetIndex = 2;
        parseState = READ_CT;
      } else {
        parseState = WAIT_HEADER1;
      }
      break;
    case READ_CT:
      packetBuffer[2] = c;
      packetIndex = 3;
      parseState = READ_LSN;
      break;
    case READ_LSN:
      packetBuffer[3] = c;
      packetIndex = 4;
      expectedLSN = c;
      expectedPacketLen = 2 + 1 + 1 + 2 + 2 + 2 + 2 * expectedLSN;
      parseState = READ_FSA_L;
      break;
    case READ_FSA_L:
      packetBuffer[4] = c;
      packetIndex = 5;
      parseState = READ_FSA_H;
      break;
    case READ_FSA_H:
      packetBuffer[5] = c;
      packetIndex = 6;
      parseState = READ_LSA_L;
      break;
    case READ_LSA_L:
      packetBuffer[6] = c;
      packetIndex = 7;
      parseState = READ_LSA_H;
      break;
    case READ_LSA_H:
      packetBuffer[7] = c;
      packetIndex = 8;
      parseState = READ_CS_L;
      break;
    case READ_CS_L:
      packetBuffer[8] = c;
      packetIndex = 9;
      parseState = READ_CS_H;
      break;
    case READ_CS_H:
      packetBuffer[9] = c;
      packetIndex = 10;
      parseState = READ_DATA;
      break;
    case READ_DATA:
      packetBuffer[packetIndex++] = c;
      if (packetIndex >= expectedPacketLen) {
        parseAndSendLidarData(packetBuffer, expectedPacketLen);
        parseState = WAIT_HEADER1;
        packetIndex = 0;
      }
      break;
  }
}

void parseAndSendLidarData(uint8_t* buf, uint16_t len) {
  if (len < 10) return;

  uint8_t lsn = buf[3];
  uint16_t fsa = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
  uint16_t lsa = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

  float angle_fsa = (fsa >> 1) / 64.0f;
  float angle_lsa = (lsa >> 1) / 64.0f;
  float diff_angle = angle_lsa - angle_fsa;
  if (diff_angle < 0) diff_angle += 360.0f;

  const int maxPoints = 256;
  float distances_m[maxPoints];
  float angles_deg[maxPoints];
  int pointCount = 0;

  for (int i = 0; i < lsn && i < maxPoints; i++) {
    int offset = 10 + i * 2;
    if (offset + 1 >= len) break;

    uint16_t si = (uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8);
    float dist_mm = si / 4.0f;
    if (dist_mm < 1.0f) continue;

    float angle_deg;
    if (lsn == 1) {
      angle_deg = angle_fsa;
    } else {
      angle_deg = angle_fsa + diff_angle * i / (lsn - 1);
    }

    // 角度修正（YDLIDAR X2）
    float correct_rad = atan2f(21.8f * (155.3f - dist_mm), 155.3f * dist_mm);
    float correct_deg = correct_rad * 180.0f / M_PI;
    angle_deg += correct_deg;

    distances_m[pointCount] = dist_mm / 1000.0f;
    angles_deg[pointCount] = angle_deg;
    pointCount++;
  }

  if (pointCount == 0) return;

  sendLidarDataViaWiFi(distances_m, angles_deg, pointCount);
}

void sendLidarDataViaWiFi(float* distances, float* angles, int count) {
  if (!lidarConnected || !lidarClient.connected()) return;

  uint16_t totalLen = 2 + 2 + 12 + 2 + count * 8;
  uint8_t outBuf[totalLen];

  outBuf[0] = 0xA5;
  outBuf[1] = 0xA5;
  outBuf[2] = totalLen & 0xFF;
  outBuf[3] = (totalLen >> 8) & 0xFF;

  float pose[3] = {0.0f, 0.0f, 0.0f};
  memcpy(outBuf + 4, pose, 12);

  outBuf[16] = count & 0xFF;
  outBuf[17] = (count >> 8) & 0xFF;

  int idx = 18;
  for (int i = 0; i < count; i++) {
    memcpy(outBuf + idx, &distances[i], 4);
    idx += 4;
    memcpy(outBuf + idx, &angles[i], 4);
    idx += 4;
  }

  lidarClient.write(outBuf, totalLen);
}

// ==================== 磁力计读取与发送 ====================
void readAndSendMagnetometer() {
  if (!magConnected || !magClient.connected()) return;

  unsigned long now = millis();
  if (now - lastMagRead >= magIntervalMs) {
    lastMagRead = now;
    sensors_event_t event;
    mmc.getEvent(&event);

    // 格式: 时间戳(ms), X(uT), Y(uT), Z(uT)
    char buf[64];
    snprintf(buf, sizeof(buf), "%lu,%.2f,%.2f,%.2f\n",
             now, event.magnetic.x, event.magnetic.y, event.magnetic.z);
    magClient.print(buf);
  }
}