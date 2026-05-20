int MOTOR_N = 0;
int SERVO_N = 0;
const int pwmPins[5] = {3, 6, 2,11,8};
const int dirPins[5] = {10, 7, 4,12,9};
int CtrlIntev = 0;
int  LEGSmode = 0;//0 stdby 1 move 2 rotate



void LEDCsetUP(void);
void setWheelOutput(float* speeds);
void kinematics(float vx, float vy, float omega);

#define SQRT3_2 0.86602540378f

const float wR =0.1;//set radius


#ifdef MCNM 
MOTOR_N = 4;
CtrlIntev = 0;
#endif

#ifdef OMNI3 
MOTOR_N = 3;
CtrlIntev = 0;
#endif

#ifdef OMNI4 
MOTOR_N = 3;
CtrlIntev = 0;
#endif


#ifdef 4WHL 
MOTOR_N = 4;
CtrlIntev = 0;
#endif


#ifdef 2WHL 
MOTOR_N = 2;
CtrlIntev = 0;
#endif

#ifdef LEGS 
MOTOR_N = 0;
SERVO_N = 4;
CtrlIntev = 15;
#endif

#ifdef SERVO_N0 
SERVO_N+=1;
#endif

#ifdef SERVO_N1 
SERVO_N+=2;
#endif

#ifdef SERVO_N2 
SERVO_N+=4;
#endif

void setWheelOutput(float* speeds) {
  for (int i = 0; i < MOTOR_N; i++) {
    float spd = constrain(speeds[i], -maxSpeed, maxSpeed);
    if(spd<0)
    {
      spd*=1.1;//inv bias
    }
    uint16_t duty = (uint16_t)((fabs(spd) / maxSpeed) * 1023.0);
    if (spd > 0.15) {//dead zone proct
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

void setServoAngle(int* angles) {
  for (int i = 0; i < SERVO_N; i++) {
  	duty = 103+(409*angles[i]/180);
    ledcWrite(ledcChannels[i+MOTOR_N], duty);
  }
}

void kinematics(float vx, float vy, float omega) {
  float wheelSpeeds[5];
  #ifdef MCNM 
  wheelSpeeds[0] = vx + vy + omega*wR;
  wheelSpeeds[1] = vx - vy + omega*wR;
  wheelSpeeds[2] = -vx - vy + omega*wR;
  wheelSpeeds[3] = -vx + vy + omega*wR;
  #endif
  #ifdef OMNI3
  wheelSpeeds[0] = 1.2*vy + wR;
  wheelSpeeds[1] = -0.5f * vy + SQRT3_2 * vx + wR;
  wheelSpeeds[2] = -0.5f * vy - SQRT3_2 * vx + wR;
  #endif
  #ifdef OMNI4
  wheelSpeeds[0] = vx  + omega*wR;
  wheelSpeeds[1] = - vy + omega*wR;
  wheelSpeeds[2] = -vx + omega*wR;
  wheelSpeeds[3] = + vy + omega*wR;
  #endif
  #ifdef 4WHL 
  wheelSpeeds[0] = vx  + omega*wR;
  wheelSpeeds[1] = vx  + omega*wR;
  wheelSpeeds[2] = -vx  + omega*wR;
  wheelSpeeds[3] = -vx  + omega*wR;
  #endif
  #ifdef 2WHL 
  wheelSpeeds[0] = vx  + omega*wR;
  wheelSpeeds[1] = -vx  + omega*wR;
  #endif
  #ifdef LEGS
  int time = millis()/CtrlIntev; 
  int angle[4];
  if(vx > 0.2)
  {
  	for(int i=0;i<4;i++)
  	{
  		angle[i] = 90+30*((time%2)^(i%2));
	}
  }
  else if(vx > 0.2)
  {
  	for(int i=0;i<4;i++)
  	{
  		angle[i] = 90-30*((time%2)^(i%2));
	}
  }
  else if(omega > 0.2)
  {
  	if(time%2 == 0)
  	{
  	  angle[0] = 90+30;
  	  angle[1] = 90;
  	  angle[2] = 90;
  	  angle[3] = 90+30;
  	  
	}
	else
	{
	  angle[0] = 90;
  	  angle[1] = 90-30;
  	  angle[2] = 90-30;
  	  angle[3] = 90;
	}
  	
	
  }
  else if(omega<0.2)
  {
  	if(time%2 == 0)
  	{
  	  angle[0] = 90-30;
  	  angle[1] = 90;
  	  angle[2] = 90;
  	  angle[3] = 90-30;
  	  
	}
	else
	{
	  angle[0] = 90;
  	  angle[1] = 90+30;
  	  angle[2] = 90+30;
  	  angle[3] = 90;
	}
  }
  #endif
  setWheelOutput(wheelSpeeds);
}

void LEDCsetUP(void)
{
	for (int i = 0; i < MOTOR_N; i++) {
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(dirPins[i], LOW);
    ledcSetup(ledcChannels[i], 3000, 10);
    ledcAttachPin(pwmPins[i], ledcChannels[i]);
    ledcWrite(ledcChannels[i], 0);
  }
  for (int j = 0; j < SERVO_N; i++) {
    pinMode(dirPins[i+j], OUTPUT);
    digitalWrite(dirPins[i+j], LOW);
    ledcSetup(ledcChannels[i+j], 50, 12);
    ledcAttachPin(pwmPins[i+j], ledcChannels[i+j]);
    ledcWrite(ledcChannels[i+j], 0);
  }
}
