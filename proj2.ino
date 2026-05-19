#include <WiFi.h>
#include <driver/ledc.h>
#include <Wire.h>
#include <math.h>   // 用于 atan2 等数学运算
#include <TinyGPSPlus.h>
#include <Adafruit_MMC56x3.h>
#include "SPI.h"
#include "NRFLite.h"
#include <Adafruit_NeoPixel.h>

#define LED_PIN     48          // 数据引脚（根据实际接线修改）
#define LED_COUNT   1           // LED数量

const uint8_t colors[][4] = {
  {0, 50, 0, 0},   // 红
  {0, 0, 50, 0},   // 绿
  {0, 0, 0, 50},   // 蓝
  {0, 0, 0, 0}     // 关
};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

const static uint8_t RADIO_ID = 0;
const static uint8_t DESTINATION_RADIO_ID = 1;
const static uint8_t PIN_RADIO_CE = 35;
const static uint8_t PIN_RADIO_CSN = 36;
const static uint8_t PIN_RADIO_SCK = 45;
const static uint8_t PIN_RADIO_MOSI = 37;
const static uint8_t PIN_RADIO_MISO = 38;

struct __attribute__((packed)) RadioPacket // Note the packed attribute.
{
    uint8_t msgtype;
    float msgdata[3];
};
NRFLite _radio;
RadioPacket _radioData;
// ==================== WiFi AP 配置 ====================
const char* ssid = "ESP32";
const char* password = "12345678";
WiFiServer server(8888);
WiFiClient client;
bool clientConnected = false;

// ==================== 激光雷达配置 ====================
HardwareSerial SerialLidar(1);  // UART1: RX=44, TX=43 (只用RX)
// YDLIDAR X2 数据包解析状态机
enum LidarParseState {
  WAIT_HEADER1,
  WAIT_HEADER2,
  READ_CT,
  READ_LSN,
  READ_FSA_L,
  READ_FSA_H,
  READ_LSA_L,
  READ_LSA_H,
  READ_CS_L,
  READ_CS_H,
  READ_DATA
};
LidarParseState parseState = WAIT_HEADER1;
uint8_t packetBuffer[2560];
uint16_t packetIndex = 0;
uint8_t expectedLSN = 0;        // 采样点数
uint16_t expectedPacketLen = 0;  // 根据LSN计算的总字节数
unsigned long lastLidarTime = 0;
const unsigned long lidarTimeout = 100; // 100ms无数据重置状态机
float robotx=0.0;
float roboty=0.0;
float omega=0.0;
int lastimutime = 0;
int realtime = 0;

// LED 状态
int led_state = 0;
// ==================== 电机引脚定义 ====================
#define MOTOR_FL_A 4
#define MOTOR_FL_B 5
#define MOTOR_FR_A 6
#define MOTOR_FR_B 7
#define MOTOR_BL_A 15
#define MOTOR_BL_B 16
#define MOTOR_BR_A 17
#define MOTOR_BR_B 18

// PWM参数
const int pwmFreq = 5000;
const int pwmResolution = 8;  // 8位分辨率 0-255

// 电机PWM通道
const int motorChannels[8] = {0, 1, 2, 3, 4, 5, 6, 7};
const int motorPins[8] = {MOTOR_FL_A, MOTOR_FL_B, MOTOR_FR_A, MOTOR_FR_B,
                          MOTOR_BL_A, MOTOR_BL_B, MOTOR_BR_A, MOTOR_BR_B};

// ==================== 控制数据结构 ====================
struct ControlData {
  float throttle = 0;
  float x = 0;
  float y = 0;
  float yaw = 0;
  uint32_t timestamp = 0;
} controlData;

portMUX_TYPE dataMutex = portMUX_INITIALIZER_UNLOCKED;

// ==================== GPS 配置 ====================
#define GPS_BAUD 9600
HardwareSerial SerialGPS(2);  // UART2: RX=10, TX=11
TinyGPSPlus gps;
unsigned long lastGPSTime = 0;
const unsigned long gpsInterval = 1000;  // 每秒发送一次GPS数据

// ==================== 磁力计配置 ====================
Adafruit_MMC5603 mmc = Adafruit_MMC5603(12345);
bool magReady = false;
float magX = 0, magY = 0, magZ = 0;
sensors_event_t event;
portMUX_TYPE magMutex = portMUX_INITIALIZER_UNLOCKED;
// ==================== 函数声明 ====================
void processLidarByte(uint8_t c);
void parseAndSendLidarData(uint8_t* buf, uint16_t len);
void sendLidarDataViaWiFi(float* distances, float* angles, int count);
void initLEDC();
void updateMotors();
void setMotorSpeed(int motorIndex, float speed);
void mecanumKinematics(float vx, float vy, float omega, float* wheelSpeeds);
void sendGPS();
void sendMag(float x,float y,float z);
// ==================== 初始化 ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== 麦克纳姆轮惯性导航小车启动 ===");

  /*pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);*/

  // 初始化激光雷达串口
  SerialLidar.begin(115200, SERIAL_8N1, 44, 43);  // RX=44, TX=43 (TX不接)
  Serial.println("激光雷达串口初始化完成");

  // 启动WiFi AP
  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP地址: ");
  Serial.println(ip);
  server.begin();
  Serial.println("TCP服务器已启动，端口8888");
  initLEDC();
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, 10, 11);
  Serial.println("GPS串口初始化完成");

  // 初始化磁力计
  Wire.begin(39, 40);  // SDA=39, SCL=40
  if (mmc.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {
    mmc.setDataRate(100);  // 100Hz
    mmc.setContinuousMode(true);
    magReady = true;
    Serial.println("MMC5603 初始化成功");
  } else {
    Serial.println("MMC5603 未检测到，请检查连接");
  }
  SPI.begin(PIN_RADIO_SCK, PIN_RADIO_MISO, PIN_RADIO_MOSI, PIN_RADIO_CSN);
  if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE2MBPS, 100))
    {
        Serial.println("Cannot communicate with radio");
       
    }
    realtime = millis();
    lastimutime = millis();
    strip.begin();           // 初始化引脚
  strip.setBrightness(255); // 亮度可调，这里设为最大
  strip.show();            // 初始关闭所有LED
}

// ==================== 主循环 ====================
void loop() {
  // 处理WiFi客户端连接
  if (!clientConnected) {
    client = server.available();
    if (client) {
      if (client.connected()) {
        clientConnected = true;
        Serial.println("新客户端已连接");
      } else {
        client.stop();
      }
    }
  } else {
    if (!client.connected()) {
      client.stop();
      clientConnected = false;
      Serial.println("客户端断开连接");
    }
  }
  mmc.getEvent(&event);
  magX = event.magnetic.x;
  magY = event.magnetic.y;
  magZ = event.magnetic.z;
  sendMag(magX,magY,magZ);
  // 读取激光雷达数据并解析
  while (SerialLidar.available() > 0) {
    uint8_t c = SerialLidar.read();
    processLidarByte(c);
  }
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
  }
  if (gps.location.isValid())
  {
    sendGPS();
  }
  while (_radio.hasData())
    {
        _radio.readData(&_radioData);
        if(_radioData.msgtype==0)
        {
          controlData.throttle = _radioData.msgdata[0];   // 前后（油门）
          controlData.x=_radioData.msgdata[1];           // 左右平移
          controlData.yaw=_radioData.msgdata[2]; 
          updateMotors();
        }
    }
  
  // 可选：LED闪烁指示状态
  // 这里可加入WiFi连接状态指示等
}

// ==================== 激光雷达数据解析与转发 ====================
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
      // LED闪烁，表示正在接收数据     
      strip.setPixelColor(colors[led_state][0], strip.Color(colors[led_state][1], colors[led_state][2], colors[led_state][3]));
      strip.show();
      led_state++;
      led_state%=3;
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
  // 解析 YDLIDAR X2 数据包，提取距离（米）和角度（度）
  if (len < 10) return;

  uint8_t lsn = buf[3];                     // 采样点数
  uint16_t fsa = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
  uint16_t lsa = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

  // 一级角度解析
  float angle_fsa = (fsa >> 1) / 64.0f;
  float angle_lsa = (lsa >> 1) / 64.0f;
  float diff_angle = angle_lsa - angle_fsa;
  if (diff_angle < 0) diff_angle += 360.0f;

  // 临时存储解析出的距离（米）和角度（度）
  const int maxPoints = 256;
  float distances_m[maxPoints];
  float angles_deg[maxPoints];
  int pointCount = 0;

  for (int i = 0; i < lsn && i < maxPoints; i++) {
    int offset = 10 + i * 2;
    if (offset + 1 >= len) break;

    uint16_t si = (uint16_t)buf[offset] | ((uint16_t)buf[offset + 1] << 8);
    float dist_mm = si / 4.0f;                // 距离，单位 mm
    if (dist_mm < 1.0f) continue;             // 忽略无效点

    // 线性插值角度
    float angle_deg;
    if (lsn == 1) {
      angle_deg = angle_fsa;
    } else {
      angle_deg = angle_fsa + diff_angle * i / (lsn - 1);
    }

    // 二级角度修正（根据YDLIDAR X2手册）
    // AngleCorrect = atan(21.8 * (155.3 - dist_mm) / (155.3 * dist_mm))   (结果弧度)
    float correct_rad = atan2f(21.8f * (155.3f - dist_mm), 155.3f * dist_mm);
    float correct_deg = correct_rad * 180.0f / PI;
    angle_deg += correct_deg;

    distances_m[pointCount] = dist_mm / 1000.0f;   // 转换为米
    angles_deg[pointCount] = angle_deg;
    pointCount++;
  }

  if (pointCount == 0) return;

  // 通过WiFi发送数据
  sendLidarDataViaWiFi(distances_m, angles_deg, pointCount);

  // 可选调试：打印点数
  // Serial.printf("LiDAR processed packet, %d points\n", pointCount);
}

void sendLidarDataViaWiFi(float* distances, float* angles, int count) {
  if (!clientConnected || !client.connected()) return;

  // 构建数据包格式：
  // 魔数(2字节) + 总长度(2字节) + 位姿(x,y,theta各float) + 点数(2字节) + 点数据(距离+角度各float，共8N字节)
  uint16_t totalLen = 2 + 2 + 12 + 2 + count * 8;  // 魔数+长度+位姿+点数+点数据
  uint8_t outBuf[totalLen];

  outBuf[0] = 0xA5;
  outBuf[1] = 0xA5;          // 魔数
  outBuf[2] = totalLen & 0xFF;
  outBuf[3] = (totalLen >> 8) & 0xFF;  // 总长度（包括头部）


  // 位姿，当前默认 (0,0,0)
  float pose[3] = {controlData.throttle,controlData.x, controlData.yaw};
  memcpy(outBuf + 4, pose, 12);

  outBuf[16] = count & 0xFF;
  outBuf[17] = (count >> 8) & 0xFF;    // 点数

  int idx = 18;
  for (int i = 0; i < count; i++) {
    memcpy(outBuf + idx, &distances[i], 4);
    idx += 4;
    memcpy(outBuf + idx, &angles[i], 4);
    idx += 4;
  }

  client.write(outBuf, totalLen);
  // 可添加flush
}



void initLEDC() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = pwmFreq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  for (int i = 0; i < 8; i++) {
    ledc_channel_config_t ledc_channel = {
      .gpio_num = motorPins[i],
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = (ledc_channel_t)motorChannels[i],
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
  }
  Serial.println("LEDC初始化完成");
}

void updateMotors() {
  
  float throttle = controlData.throttle;
  float x = controlData.x;
  float y = controlData.y;
  float yaw = controlData.yaw;
  
  // 速度矢量映射（根据实际坐标系调整符号）
  float vx = -throttle;   // 前后（油门）
  float vy = x;           // 左右平移
  float omega = yaw;      // 旋转

  float wheelSpeeds[4];
  mecanumKinematics(vx, vy, omega, wheelSpeeds);

  for (int i = 0; i < 4; i++) {
    setMotorSpeed(i, wheelSpeeds[i]);
  }
}

void setMotorSpeed(int motorIndex, float speed) {
  speed = constrain(speed, -1.0, 1.0);
  int pwmValue = (int)(abs(speed) * 255);

  int chA, chB;
  switch (motorIndex) {
    case 0: chA = 0; chB = 1; break;  // 左前
    case 1: chA = 2; chB = 3; break;  // 右前
    case 2: chA = 4; chB = 5; break;  // 左后
    case 3: chA = 6; chB = 7; break;  // 右后
    default: return;
  }

  if (speed > 0) {
    ledcWrite(chA, pwmValue);
    ledcWrite(chB, 0);
  } else if (speed < 0) {
    ledcWrite(chA, 0);
    ledcWrite(chB, pwmValue);
  } else {
    ledcWrite(chA, 0);
    ledcWrite(chB, 0);
  }
}

void mecanumKinematics(float vx, float vy, float omega, float* wheelSpeeds) {
  float L = 0.2;   // 半长
  float W = 0.12;   // 半宽
  wheelSpeeds[0] =  vx - vy - (L + W) * omega;  // 左前
  wheelSpeeds[1] =  vx + vy + (L + W) * omega;  // 右前
  wheelSpeeds[2] =  vx + vy - (L + W) * omega;  // 左后
  wheelSpeeds[3] =  vx - vy + (L + W) * omega;  // 右后

  float maxSpeed = 0;
  for (int i = 0; i < 4; i++) {
    if (abs(wheelSpeeds[i]) > maxSpeed) maxSpeed = abs(wheelSpeeds[i]);
  }
  if (maxSpeed > 1.0) {
    for (int i = 0; i < 4; i++) wheelSpeeds[i] /= maxSpeed;
  }
}

void sendGPS()
{
    RadioPacket radioData;
    radioData.msgtype = 2;
    radioData.msgdata[0] =  gps.location.lat();
    radioData.msgdata[1] =  gps.location.lng();
    radioData.msgdata[2] =  gps.altitude.meters();
    if (_radio.send(DESTINATION_RADIO_ID, &radioData, sizeof(radioData))) // 'send' puts the radio into Tx mode.
    {
        Serial.println("...Success");
    }
    else
    {
        Serial.println("...Failed");
    }
}
void sendMag(float x,float y,float z)
{
    RadioPacket radioData;
    radioData.msgtype = 1;
    radioData.msgdata[0] = x;
    radioData.msgdata[1] = y;
    radioData.msgdata[2] = z;
    if (_radio.send(DESTINATION_RADIO_ID, &radioData, sizeof(radioData))) // 'send' puts the radio into Tx mode.
    {
        Serial.println("...Success");
    }
    else
    {
        Serial.println("...Failed");
    }
}