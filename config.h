#include<Arduino.h>
class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral,last_u;
  public:
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
     void reset_all()
    {
        eintegral=0;
        eprev=0;
    } 
    void setParams(float kpIn, float kiIn, float kdIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
      reset_all();
    }
    float compute(int value, int target, float deltaT){
      if(target==0){
        reset_all();
        return 0;
      }
      int e = target - value;
      float dedt = (e-eprev)/(deltaT);
      if(abs((int)last_u) >= umax && (((e >= 0) && (eintegral >= 0)) || ((e < 0) && (eintegral < 0))))
      {
        eintegral = eintegral;
      }
      else{
        eintegral+=e*deltaT;
      }
      eintegral=constrain(eintegral,-120,120);
      float u = kp*e + kd*dedt + ki*eintegral;
      if( u > umax)u = umax;
      else if(u<-umax)u=-umax;
      last_u=u;
      eprev = e;
      return u;
    }
    float GetKp(){return kp;}
    float GetKi(){return ki;}
    float GetKd(){return kd;}
   
};
typedef struct{
  float x;
  float y;
  float theta;
}coordinate;
typedef enum{
  SLOW_SPEED=0,
  NORMAL_SPEED=1,
  SPEED_UP=2
}RobotSpeedEnum;

bool PinStateChanged(int pin, int *lastButtonState, int *buttonRisingEdge) {
  //Get pin state
  int buttonState =pin;

  //Here starts the code for detecting an edge
  if (buttonState != *lastButtonState) {
    if (buttonState == LOW) {
      *buttonRisingEdge = 0;
    } else {
      *buttonRisingEdge = 1;
    }
    *lastButtonState = buttonState;
    return true;
  }

  return false;
}
typedef void (*CallbackFunction)();
void callFunctionPeriodically(CallbackFunction functionToCall, unsigned long intervalTime, unsigned long &previousMillis) {
  unsigned long currentMillis = millis();  //
  if (currentMillis - previousMillis >= intervalTime) {
    functionToCall();
    previousMillis = currentMillis;
  }
}
#define SPEED_RUN 100 //cm/s
#define SPEED_SLOW 25 //cm/s
#define SPEED_MAX 100 //cm/s
#define NMOTORS 4
#define M_LEFT_DOWN 2
#define M_LEFT_UP 3
#define M_RIGHT_DOWN 1
#define M_RIGHT_UP 0
#define ENCODER_TOTAL 450.5 /// do duoc < thuc te -> giam 
#define x_offset 1.0
#define y_offset 1.0
#define theta_offset 0.874
#define WHEEL_DIAMETER 9.5 // cm
//macro for detection af rasing edge
#define RE(signal, state) (state=(state<<1)|(signal&1)&3)==1
#define length 16.5 //cm
#define width 23.2 //
#define total_length (length+width)
//macro for detection af falling edge
#define FE(signal, state) (state=(state<<1)|(signal&1)&3)==2
#define SHOOT_L1 44
#define SHOOT_L2 45
#define SPEED_SHOOT 200
#define STEP_GO 300
#define SPEED_STEP 400
#define MOVING 0
#define SHOOTING 1
#define STOP_ALL 2
#define SHOOTING_ALIGN 3
int state_robot_all=STOP_ALL;
float P_speed=10.4;
float D_speed=0.0;
float I_speed=55.5;
const bool test_ff=0;
const int enca[] = {18,19,20,21};
const int encb[]= {17,16,15,14};
const int pwm[] = {9,7,5,3}; //{10,11}
const int dir[] = {8,6,4,2};
const int dir_encod_M1=1;
const int dir_encod_M0=1;
const float LOOP_FREQUENCY=100.0;
const float speed_ff=10.0; // 100 Hz
const bool serial_tune = 0;
const int time_run_test=2000;
const float LOOP_CONTROL=1.0/LOOP_FREQUENCY;
const float cm_per_count = (PI * WHEEL_DIAMETER) / ENCODER_TOTAL;
const float delta_cvt = LOOP_CONTROL/cm_per_count;
// #define delta_cvt(speed) (speed * LOOP_CONTROL) / cm_per_count  // // cm/s -> delta send
// const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
// const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const unsigned long micros_interval=LOOP_CONTROL*1e6;


