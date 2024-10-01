#include <AccelStepper.h>
#define step_bot 37
#define dir_bot 35
#define step_up 49
#define dir_up 47
AccelStepper stepper_linear(1, step_up, dir_up);  // pin 5 step, pin 4 dir
AccelStepper stepper_rotate(1, step_bot, dir_bot);  // pin 5 step, pin 4 dir
bool step_run_speed=0;
void setup_step()
{
    stepper_linear.setMaxSpeed(1000);
    stepper_linear.setAcceleration(800);
    stepper_rotate.setMaxSpeed(1000);
    stepper_rotate.setAcceleration(800);
    stepper_linear.setSpeed(0);
}
void run_step_linear(int steps)
{
  // if(!stepper_linear.isRunning())stepper_linear.move(steps);
  stepper_linear.setSpeed(steps);
}
void run_step_rotate(int steps,int speed_step)
{
  if(speed_step==0)
  {
    stepper_rotate.move(steps);
    step_run_speed=0;
  }
  else 
  {
    stepper_rotate.setSpeed(steps);
    step_run_speed=1;
  }
}
void loop_step()
{
    // stepper.run();
    // stepper_linear.run();
    if(!step_run_speed)stepper_rotate.run();
    else stepper_rotate.runSpeed();
    stepper_linear.runSpeed();
    // stepper_rotate.runSpeed();
}
