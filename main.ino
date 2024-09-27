#include "gamepad.h"
// #include "config.h"

#include <util/atomic.h>
#include "digitalWriteFast.h"
#include <Wire.h>
#include "step_motor.h"

bool start_test = 0;
int m_pwm[NMOTORS];
int speed_desired_left = 0;
int speed_desired_right = 0;
volatile int speed_desired[] = { 0, 0, 0, 0 };
int ff = 0;
volatile int encoder_count[NMOTORS];
int index_motor_pid = 0;
volatile long pos[NMOTORS] = { 0, 0, 0, 0 };
coordinate robot_pose;
coordinate robot_speed;
SimplePID pid[NMOTORS];
int encod_state[4];
float yaw_rate = 0;
float yaw_incre = 0;
float yaw_desired = 0;
float p_yaw = 1.35; //1.35
float d_yaw = 0.09; //0.09
float err_yaw, last_err_yaw;
RobotSpeedEnum RobotSpeedState;
void set_state_robot(int state) {
  if (state_robot_all == STOP_ALL) state_robot_all = state;
}
void stopAll() {
  analogWrite(SHOOT_L1, 0);
  analogWrite(SHOOT_L2, 0);
  run_step_linear(0);
}
void reset_yaw_pd() {
  err_yaw = 0;
  last_err_yaw = 0;
  robot_speed.theta = 0;
  // robot_speed.x=0;
  // robot_speed.y=0;
  // robot_speed.theta=0;
  yaw_desired = 0;
}
void DebouncButCount(bool &button, unsigned long time_mill) {
  if (millis() - time_mill > 200) {
    button = !button;
  }
}
void moving_control() {
  
  if (!stateBut.L1) RobotSpeedState = NORMAL_SPEED;
  else RobotSpeedState = SLOW_SPEED;
  // else if (stateBut.R1) RobotSpeedState = SPEED_UP;
  // else if(stateBut.R1)RobotSpeedState=SPEED_UP;
  float v[3];
  switch (RobotSpeedState) {
    case NORMAL_SPEED:
      for (int i = 0; i < 3; i++) v[i] = SPEED_RUN;  //cm/s
      break;
    case SLOW_SPEED:
      for (int i = 0; i < 3; i++) v[i] = SPEED_SLOW;  //cm/s
      break;
      // case SPEED_UP:
      //   for (int i = 0; i < 3; i++) v[i] = SPEED_MAX;  //cm/s
      //   break;
  }
  bool running_now = false;

  static unsigned long time_stop_x = millis();
  static unsigned long time_stop_y = millis();
  static unsigned long time_stop_yaw = millis();
  bool press_but = false;
  if (!stateBut.PAD_UP && !stateBut.PAD_DOWN) {
    bool stop_x = true;
    // DebouncButCount(stop_x, time_stop_x);
    if (stop_x) v[0] = 0;
  } else {
    if (stateBut.PAD_DOWN) v[0] = -v[0];
    time_stop_x = millis();
  }
  if (!stateBut.PAD_LEFT && !stateBut.PAD_RIGHT) {
    bool stop_y = true; //false
    // DebouncButCount(stop_y, time_stop_y);
    if (stop_y) v[1] = 0;
  } else {
    time_stop_y = millis();
    if (stateBut.PAD_RIGHT) v[1] = -v[1];
  }
  if (!stateBut.SQUARE && !stateBut.CIRCLE) {
    // bool stop_yaw = false;
    bool stop_yaw=true;
    // DebouncButCount(stop_yaw, time_stop_yaw);
    if (stop_yaw) {
      v[2] = 0;
      reset_yaw_pd();
    }
  } else {
    if (stateBut.SQUARE) v[2] = -v[2];
    time_stop_yaw = millis();
  }

  // v[2]=v[2]*0.8;
  // v[2]=0.7*last_vz+0.3*(float)stateBut.RX;
  // last_vz=v[2];
  // for(int i=0;i<3;i++){
  //   Serial.print(v[i]);
  //   Serial.print(",");
  // }
  // Serial.println(pos[M_RIGHT_UP]);
  for (int i = 0; i < 3; i++) {
    if (v[i] != 0) running_now = true;
  }
  if (running_now == false && state_robot_all == MOVING) {
    state_robot_all = STOP_ALL;
    // Serial.println("moving stop");
  } else if (running_now) {
    robot_speed.x = v[0];
    robot_speed.y = v[1];
    yaw_desired = v[2] * M_PI / 180.0;
    // Serial.println(running_now);
    set_state_robot(MOVING);
  }
  // return running_now;

  // Serial.println("v1:"+String(v[0]));
  // Serial.println("v2:"+String(v[1]));
  // calSpeedRobot(v[0], v[1], v[2]);
}
void button_state_process() {
  moving_control();
  static bool start_shoot = false;
  bool stop_shoot = false;
  static bool first_go = false;
  static unsigned long time_start_shoot = millis();
  static unsigned long time_stop_shoot = millis();
  static unsigned long last_time_shoot = millis();
  // switch(state_robot_all)
  // {
  //   case MOVING:

  // }
  if (stateBut.R1 && !start_shoot) {
    time_stop_shoot = millis();
    start_shoot = true;
    set_state_robot(SHOOTING);
  }
  
  bool start_nap = false;
  if (state_robot_all == SHOOTING) {
    analogWrite(SHOOT_L1, SPEED_SHOOT);
    analogWrite(SHOOT_L2, SPEED_SHOOT);
    static bool wait_nap = false;
    if (millis() - last_time_shoot > 300 && !first_go) {
      first_go = true;
      start_nap = true;
    }
    if (start_nap) {
      run_step_rotate(STEP_GO, 0);
      start_nap = false;
      wait_nap = true;
    }
    bool step_done=!stepper_rotate.isRunning();
    if (step_done && wait_nap) {
      
      state_robot_all = STOP_ALL;
      wait_nap = false;
      analogWrite(SHOOT_L1, 0);
      analogWrite(SHOOT_L2, 0);
      // Serial.println("done");
      
    }
  } else {
    start_shoot=false;
    first_go = false;
    last_time_shoot = millis();
    analogWrite(SHOOT_L1, 0);
    analogWrite(SHOOT_L2, 0);
    // Serial.println("stop shoot!!!");
  }
  if ((!stateBut.TRIANGLE && !stateBut.CROSS)) {
    run_step_linear(0);
    if (state_robot_all == SHOOTING_ALIGN) state_robot_all = STOP_ALL;
    // Serial.println("STOP");
  } else {
    if (stateBut.TRIANGLE) {
      run_step_linear(SPEED_STEP);
      // Serial.println("UP");
    } else if (stateBut.CROSS) {
      run_step_linear(-SPEED_STEP);
      // Serial.println("DOWN");
    }
    // state_robot_all=SHOOTING_ALIGN;
    set_state_robot(SHOOTING_ALIGN);
  }
  static bool start_speed_step = false;
  if (stateBut.R2) {
    run_step_rotate(-SPEED_STEP, SPEED_STEP);
    start_speed_step = true;
    // Serial.println("RUNNING HOME");
  } else if (start_speed_step) {
    run_step_rotate(0, SPEED_STEP);
    start_speed_step = false;
    // Serial.println("STOP HOME");
  }
}
void test_speed_robot(float x, float y, float theta) {
  static unsigned long time_count = millis();
  if (x == 0 && y == 0 && theta == 0) {
    time_count = millis();
    return;
  }
  int time_stop;
  if (x != 0) {
    time_stop = x / 25;
    calSpeedRobot(25, 0, 0);
  }
  if (y != 0) {
    time_stop = y / 25;
    calSpeedRobot(0, 25, 0);
  }
  if (theta != 0) {
    time_stop = theta / 25;
    calSpeedRobot(0, 0, 25);
  }
  if (millis() - time_count > time_stop * 1000) {
    for (int i = 0; i < NMOTORS; i++) speed_desired[i] = 0;
  }
}
void calSpeedRobot(float vx, float vy, float dtheta) {
  // cm/s
  float speed_cm_s[NMOTORS];
  // Serial.println(vx,5);
  dtheta = dtheta * PI / 180.0;
  speed_cm_s[M_LEFT_UP] = (vx + vy - dtheta * total_length);
  speed_cm_s[M_RIGHT_UP] = (vx - vy + dtheta * total_length);
  speed_cm_s[M_LEFT_DOWN] = (vx - vy - dtheta * total_length);
  speed_cm_s[M_RIGHT_DOWN] = (vx + vy + dtheta * total_length);
  // Serial.println(speed_cm_s[1],5);
  for (int i = 0; i < NMOTORS; i++) speed_desired[i] = speed_cm_s[i] * delta_cvt;

  // Serial.println(speed_desired[1]);
}
void updateRobotpos(float delta[]) {
  float dx = (delta[0] + delta[1] + delta[2] + delta[3]) / 4.0;
  float dy = (-delta[M_RIGHT_UP] - delta[M_LEFT_DOWN] + delta[M_LEFT_UP] + delta[M_RIGHT_DOWN]) / 4.0;
  float dtheta = (delta[M_RIGHT_UP] - delta[M_LEFT_DOWN] - delta[M_LEFT_UP] + delta[M_RIGHT_DOWN]) / (4.0 * total_length);
  robot_pose.x += dx * x_offset;
  robot_pose.y += dy * y_offset;
  yaw_incre = dtheta * theta_offset;

  robot_pose.theta += yaw_incre;
  yaw_rate = yaw_incre / 0.01;
}
void readEncoderM0() {
  static bool old_a = false;
  bool newA = digitalReadFast(enca[M_LEFT_UP]);
  bool newB = digitalReadFast(encb[M_LEFT_UP]);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M1 : -dir_encod_M1;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M1 : dir_encod_M1;
  }
  encoder_count[M_LEFT_UP] -= delta;
  old_a = newA;
}
void readEncoderM1() {
  static bool old_a = false;
  bool newA = digitalReadFast(enca[M_LEFT_DOWN]);
  bool newB = digitalReadFast(encb[M_LEFT_DOWN]);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M1 : -dir_encod_M1;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M1 : dir_encod_M1;
  }
  encoder_count[M_LEFT_DOWN] -= delta;
  old_a = newA;
}
void readEncoderM3() {
  static bool old_a = false;
  bool newA = digitalReadFast(enca[M_RIGHT_UP]);
  bool newB = digitalReadFast(encb[M_RIGHT_UP]);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M1 : -dir_encod_M1;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M1 : dir_encod_M1;
  }
  encoder_count[M_RIGHT_UP] += delta;
  old_a = newA;
}
void readEncoderM2() {
  static bool old_a = false;
  bool newA = digitalReadFast(enca[M_RIGHT_DOWN]);
  bool newB = digitalReadFast(encb[M_RIGHT_DOWN]);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M1 : -dir_encod_M1;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M1 : dir_encod_M1;
  }
  encoder_count[M_RIGHT_DOWN] += delta;
  old_a = newA;
}
bool receive_uart() {
  if (Serial.available()) {
    String c = Serial.readStringUntil(";");
    int index_now = c.indexOf("/");
    int index_kp_desired = c.indexOf(":");
    int dir_mor = c.indexOf(".");
    int index_ff = c.indexOf("f");

    if (index_now != -1) {
      // speed_linear = c.substring(0, index_now).toFloat();
      // angular_speed = (c.substring(index_now + 1).toFloat());

      int index_motor = c.substring(0, index_now).toInt();
      int speed_now = c.substring(index_now + 1).toInt();
      // for (int j = 0; j < 4; j++) speed_desired[j] = speed_now;
      // speed_desired[index_motor]=speed_now;
      if (index_motor == 1) robot_speed.x = speed_now;
      else if (index_motor == 2) robot_speed.y = speed_now;
      else if (index_motor == 3) yaw_desired = speed_now * M_PI / 180.0;
      index_motor_pid = index_motor;
      // Serial.print("speed:");

      // Serial.print(speed_desired[0]);
      // Serial.print(",");
      // Serial.println(speed_desired[1]);
      return 1;
    } else if (index_ff != -1) {
      ff = c.substring(0, index_ff).toInt();
      int dir_motor = 1;

      int speed = c.substring(index_ff + 1, dir_mor).toInt();

      m_pwm[ff] = speed;
      Serial.println("ff " + String(ff) + "@" + String(speed) + "%" + String(m_pwm[ff]));
      // return 1;
    } else if (index_kp_desired != -1) {
      int index_cal = c.indexOf("#");
      if (index_cal != -1) {
        float new_kp = c.substring(0, index_kp_desired).toFloat();
        float new_ki = c.substring(index_kp_desired + 1, index_cal).toFloat();
        float new_kd = c.substring(index_cal + 1).toFloat();

        // for (int i = 0; i < 4; i++) pid[i].setParams(new_kp, new_ki, new_kd, 255);
        p_yaw = new_kp;
        d_yaw = new_kd;
        // pid[M_LEFT_DOWN].setParams(new_kp, new_ki, new_kd, 255);

        // Serial.print(pid[M_LEFT_DOWN].GetKp());
        // Serial.print(" ");
        // Serial.print(pid[M_LEFT_DOWN].GetKi());
        // Serial.print(" ");
        // Serial.println(pid[M_LEFT_DOWN].GetKd());
        Serial.print(p_yaw);
        Serial.print(" ");
        Serial.println(d_yaw);
      }
    }
  }
  return 0;
}
void control_motor(int motor, int speed) {
  bool direct = speed > 0 ? 1 : 0;
  // digitalWrite(dir[motor],direct);
  if (motor == M_LEFT_DOWN || motor == M_RIGHT_DOWN) direct = 1 - direct;
  if (direct) {
    analogWrite(pwm[motor], 255 - abs(speed));
    digitalWrite(dir[motor], 1);
  } else {
    analogWrite(pwm[motor], abs(speed));
    digitalWrite(dir[motor], 0);
  }
}

void setup() {
  Serial.begin(9600);
  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT_PULLUP);
    pinMode(encb[k], INPUT_PULLUP);
    pinMode(pwm[k], OUTPUT);
    pinMode(dir[k], OUTPUT);
  }
  pinMode(SHOOT_L1, OUTPUT);
  pinMode(SHOOT_L2, OUTPUT);
  pid[M_LEFT_DOWN].setParams(P_speed, I_speed, D_speed, 255);   //39.2 34.6
  pid[M_LEFT_UP].setParams(P_speed, I_speed, D_speed, 255);     //39.2 34.6
  pid[M_RIGHT_DOWN].setParams(P_speed, I_speed, D_speed, 255);  //39.2 34.6
  pid[M_RIGHT_UP].setParams(P_speed, I_speed, D_speed, 255);    //39.2 34.6
  setup_step();
  setup_gamepad();
  attachInterrupt(digitalPinToInterrupt(enca[M_LEFT_DOWN]), readEncoderM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[M_LEFT_UP]), readEncoderM0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[M_RIGHT_DOWN]), readEncoderM2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[M_RIGHT_UP]), readEncoderM3, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(encb[M1]), readEncoderM3, CHANGE);
}
void loop() {
  static unsigned long time_now = micros();
  static bool start_run_motor = 0;
  static unsigned long time_start = millis();
  static unsigned long time_rec = millis();
  static unsigned long time_print = millis();
  static unsigned long time_but = millis();
  int delta_now = 0;
  static uint8_t count_loop2 = 0;

  unsigned long time_count_now = micros() - time_now;
  if (time_count_now >= micros_interval) {

    time_now = micros();
    int delta[NMOTORS];

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      for (int i = 0; i < NMOTORS; i++) {
        delta[i] = encoder_count[i];
        encoder_count[i] = 0;
      }
    }
    static float new_delta[NMOTORS];
    static int last_delta[NMOTORS];
    float delta_cm_s[NMOTORS];
    float dt_pos = (float)time_count_now / 1000000.0;
    // Serial.println(dt_pos);
    count_loop2 += 1;
    for (int i = 0; i < NMOTORS; i++) {
      // pos[i] += delta[i];
      delta_cm_s[i] = delta[i] * cm_per_count;
      m_pwm[i] = pid[i].compute(new_delta[i], speed_desired[i], (1.0 / LOOP_FREQUENCY));  // speed >0 ->  delta must >0
      new_delta[i] = new_delta[i] * 0.22826091 + 0.38586955 * (float)delta[i] + 0.38586955 * (float)last_delta[i];
      last_delta[i] = delta[i];
    }
    updateRobotpos(delta_cm_s);

    if (count_loop2 >= 2) {

      count_loop2 = 0;
      float err_now = yaw_desired - yaw_rate;  //yaw_desired > 0 nguoc chieu kim dong ho
      err_yaw += err_now;
      robot_speed.theta = p_yaw * err_yaw + (err_yaw - last_err_yaw) * d_yaw;
      last_err_yaw = err_yaw;
    }
    if (state_robot_all != MOVING) {
      for (int i = 0; i < NMOTORS; i++) {
        pid[i].reset_all();
        speed_desired[i] = 0;
      }
      reset_yaw_pd();
      robot_speed.x = 0;
      robot_speed.y = 0;
      robot_speed.theta = 0;
    }
    calSpeedRobot(robot_speed.x, robot_speed.y, robot_speed.theta);
    //  Serial.print(pos[1]);
    if (serial_tune) {
      if (receive_uart() && start_run_motor == 0) {
        start_run_motor = 1;
        for (int i = 0; i < NMOTORS; i++) pid[i].reset_all();
        time_start = millis();
      }
      if (start_run_motor == 1) {
        static uint16_t count_print = 0;
        count_print += 1;
        if (count_print >= 2) {
          // Serial.println("M0:" + String(pos[M0]));
          // Serial.println(pos[M1]);
          // Serial.print(new_delta[M_RIGHT_UP]);
          // Serial.print(",");
          // Serial.print(speed_desired[M_RIGHT_UP]);
          // Serial.print(",");
          // Serial.print(new_delta[M_LEFT_DOWN]);
          // Serial.print(",");
          // Serial.println(new_delta[M_LEFT_UP]);

          Serial.print(yaw_desired);
          Serial.print(",");
          Serial.println(yaw_rate);
          // Serial.println(yaw_rate);
          count_print = 0;
        }
        static int period = time_run_test / 2;
        uint32_t time_now = millis() - time_start;
        // Serial.println(millis());
        // Serial.println(time_start);
        float sinus = sin(2 * PI * time_now / period);  // base pattern
        //  speed_desired_left = (2 * 0.16 / PI) * asin(sinus); // triangle

        if (time_now >= time_run_test) {
          start_run_motor = 0;
          for (int i = 0; i < NMOTORS; i++) {
            speed_desired[i] = 0;
          }
          robot_speed.x = 0;
          robot_speed.y = 0;
          reset_yaw_pd();
          Serial.println("done");
        }
      }
    }
  }
  for (int i = 0; i < NMOTORS; i++) {
    if (state_robot_all != MOVING)m_pwm[i]=0;
    control_motor(i, m_pwm[i]);
  }
  callFunctionPeriodically(process_gamepad, 50, time_rec);
  // process_gamepad();

  callFunctionPeriodically(button_state_process, 100, time_but);
  loop_step();
  //  Serial.println(pos[M_LEFT_DOWN]);
}