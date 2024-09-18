#include <AccelStepper.h>
#define step_bot 37
#define dir_bot 35
#define step_up 49
#define dir_up 47
AccelStepper stepper_linear(1, step_bot, dir_bot);  // pin 5 step, pin 4 dir
AccelStepper stepper_rotate(1, step_up, dir_up);  // pin 5 step, pin 4 dir
void setup_step()
{
    stepper_linear.setMaxSpeed(1000);
    stepper_linear.setAcceleration(500);
    stepper_rotate.setMaxSpeed(1000);
    stepper_rotate.setAcceleration(500);
}
void run_step_linear(int steps)
{
  stepper_linear.move(steps);
}
void run_step_rotate(int steps)
{
  stepper_rotate.move(steps);
}
void loop_step()
{
    // stepper.run();
    stepper_linear.run();
    stepper_rotate.run();
}
