/*

Demonstrates simple RX operation with an ESP32.
Any of the Basic_TX examples can be used as a transmitter.

ESP's require the use of '__attribute__((packed))' on the RadioPacket data structure
to ensure the bytes within the structure are aligned properly in memory.

The ESP32 SPI library supports configurable SPI pins and NRFLite's mechanism to support this is shown.

Radio    ESP32 module
CE    -> 4
CSN   -> 5
MOSI  -> 23
MISO  -> 19
SCK   -> 18
IRQ   -> No connection
VCC   -> No more than 3.6 volts
GND   -> GND

*/

#include "SPI.h"
#include "NRFLite.h"
#include <TinyGPSPlus.h>

#include <Button.h>

#define GPS_BAUD 9600
HardwareSerial SerialGPS(2); 
TinyGPSPlus gps;
unsigned long lastGPSTime = 0;
const unsigned long gpsInterval = 1000;  // 每秒发送一次GPS数据
double lantser;
double lngtser;
int ValIdx = 0;


bool magReady = false;
float magX = 0, magY = 0, magZ = 0;
portMUX_TYPE magMutex = portMUX_INITIALIZER_UNLOCKED;

/*const static uint8_t RADIO_ID = 1;
const static uint8_t DESTINATION_RADIO_ID = 0;
const static uint8_t PIN_RADIO_CE = 4;
const static uint8_t PIN_RADIO_CSN = 5;
const static uint8_t PIN_RADIO_MOSI = 23;
const static uint8_t PIN_RADIO_MISO = 19;
const static uint8_t PIN_RADIO_SCK = 18;*/

const static uint8_t RADIO_ID = 1;
const static uint8_t DESTINATION_RADIO_ID = 0;
const static uint8_t PIN_RADIO_CE = 2;
const static uint8_t PIN_RADIO_CSN = 3;
const static uint8_t PIN_RADIO_MOSI = 6;
const static uint8_t PIN_RADIO_MISO = 5;
const static uint8_t PIN_RADIO_SCK = 4;

struct __attribute__((packed)) RadioPacket // Note the packed attribute.
{
    uint8_t msgtype;
    float msgdata[3];
};

NRFLite _radio;
RadioPacket _radioData;


#define STEP_PIN1 4
#define STEP_PIN2 5
#define STEP_PIN3 6
#define STEP_PIN4 7
#define STEP_PIN5 16
#define STEP_PIN6 17
#define STEP_PIN7 18
#define STEP_PIN8 3

// PWM公共参数
#define PWM_FREQ       5000   // 频率 5 kHz
#define PWM_RESOLUTION 8      // 分辨率 8位（占空比0~255）

int MotorStep = 0;
int MicroStep = 0;
int MotorStepY = 0;
int MicroStepY = 0;
int YawDric = 1;//北->西
int PitchDric = 1;//北->西
double CurrentAngle = 0;
double StepDeg = 360/4096;
double MicroStepDeg = (360/4096)/256;
double tarDeg = 0;
void Ledcsetup();
double xbias= 0;
double ybias= 0;
double zbias= 0;
int lasttime = 0;
char c=0;
int btnnum = 0;
Button button1(2);


const uint8_t stepSequence[8][4] = {
  {255, 0, 0, 0}, // A
  {255, 255, 0, 0}, // AB
  {0, 255, 0, 0}, // B
  {0, 255, 255, 0}, // BC
  {0, 0, 255, 0}, // C
  {0, 0, 255, 255}, // CD
  {0, 0, 0, 255}, // D
  {255, 0, 0, 255}  // DA
};




void MotorApplyYaw(int MotorSteps,int MicroSteps);
void MotorApplyPitch(int MotorSteps,int MicroSteps);

void readGPS();

void RTKself(double lat,double lng);
void Readxy(void);
void sendCTRL(float vx, float vy,float  omega);
void TESTang();

void setup()
{
    Serial.begin(115200);
    
    // Configure SPI pins.
    SPI.begin(PIN_RADIO_SCK, PIN_RADIO_MISO, PIN_RADIO_MOSI, PIN_RADIO_CSN);
   
    // Indicate to NRFLite that it should not call SPI.begin() during initialization since it has already been done.
    uint8_t callSpiBegin = 0;
    
    if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN, NRFLite::BITRATE2MBPS, 100, callSpiBegin))
    {
      //while (1)
        Serial.println("Cannot communicate with radio");
         // Wait here forever.
    }
    else{
      Serial.println("communicate with radio");
      
    }

      // 初始化GPS串口
 /* SerialGPS.begin(GPS_BAUD, SERIAL_8N1);
  Serial.println("GPS串口初始化完成");*/
  //pinMode(2,INPUT);
  //button1.begin();
  /*pinMode(34,INPUT);
  pinMode(35,INPUT);*/
  pinMode(0,INPUT);
  pinMode(1,INPUT);
  Ledcsetup();
  TESTang();
  

  
}

void loop()
{

  /*while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
    if(gps.location.isValid())
    {
      RTKself(gps.location.lat(),gps.location.lng());
    }
  }*/
  /*if(Serial.available())
  {
    btnnum^=1;
    Serial.read();
  }
  if(millis()-lasttime>3)
  {
    lasttime = millis();
    Readxy();
  }*/
   
  
    
}


void RTKself(double lat,double lng)
{
  double x = (double)ValIdx/((double)ValIdx+1.0);
  lantser = lantser*x+lat*(1.0-x);
  lngtser = lngtser*x+lng*(1.0-x);
}


void Ledcsetup() {
   // 1. 为每个通道设置相同的频率和分辨率（自动分配/复用同一个定时器）
   for(int i=0;i<8;i++)
   {
    ledcSetup(i, PWM_FREQ, PWM_RESOLUTION);
   }
  

  // 2. 将通道绑定到指定 GPIO 引脚
  ledcAttachPin(STEP_PIN1, 0);
  ledcAttachPin(STEP_PIN2, 1);
  ledcAttachPin(STEP_PIN3, 2);
  ledcAttachPin(STEP_PIN4, 3);
  ledcAttachPin(STEP_PIN5, 4);
  ledcAttachPin(STEP_PIN6, 5);
  ledcAttachPin(STEP_PIN7, 6);
  ledcAttachPin(STEP_PIN8, 7);

  for(int i=0;i<8;i++)
   {
    ledcWrite(i, 0);  
   }
   
}

void MotorApplyYaw(int MotorSteps,int MicroSteps)
{
  MotorSteps = (MotorSteps%8);
  if(MotorSteps<0)
  {
    MotorSteps+=8;
  }
  int steps[4]={0};
  for(int i=0;i<4;i++)
  {
    steps[i]=(stepSequence[MotorSteps%8][i]*(256-MicroSteps)+stepSequence[(MotorSteps+1)%8][i]*(MicroSteps))/256;
    ledcWrite(i, steps[i]);
  }
}

void MotorApplyPitch(int MotorSteps,int MicroSteps)
{
  MotorSteps = (MotorSteps%8);
  if(MotorSteps<0)
  {
    MotorSteps+=8;
  }
  int steps[8]={0};
  for(int i=4;i<8;i++)
  {
    steps[i]=(stepSequence[MotorSteps%8][i]*(256-MicroSteps)+stepSequence[(MotorSteps+1)%8][i]*(MicroSteps))/256;
    ledcWrite(i, steps[i]);
  }
}

void sendCTRL(float vx, float vy, float omega) {
    RadioPacket packet;
    packet.msgtype = 0;           // 控制指令类型
    packet.msgdata[0] = vx;
    packet.msgdata[1] = vy;
    packet.msgdata[2] = omega;

    if (_radio.send(DESTINATION_RADIO_ID, &packet, sizeof(packet))) {
        Serial.println("发送成功");
    } else {
        Serial.println("发送失败");
    }
}

void Readxy(void)
{
  /*int Ax = analogRead(34);
  int Ay = analogRead(35);*/
  int Ax = analogRead(0);
  int Ay = analogRead(1);
  float Anax = 0;
  float Anay = 0;
  Ax = 2048-Ax;
  Ay-=2048;
  Anax = (float)Ax/2048.0;
  if(fabs(Anax)<0.2)
  {
    Anax = 0;
  }
  Anay = (float)Ay/2048.0;
  if(fabs(Anay)<0.2)
  {
    Anay = 0;
  }
  /*Serial.print(Ax);
  Serial.print(" ");
  Serial.println(Ay);*/
  
  if(btnnum == 0)
  {
    sendCTRL(Anax, Anay,0);
  }
  else
  {
    sendCTRL(Anax, 0,Anay);
  }
}

void TESTang()
{
  for(int i=0;i<200;i++)
  {
    MotorStep++;
    MotorApplyYaw(MotorStep,MicroStep);
    delay(1);
  }
  for(int i=0;i<200;i++)
  {
    MotorStep--   ;
    MotorApplyYaw(MotorStep,MicroStep);
    delay(1);
  }
}