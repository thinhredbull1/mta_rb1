#include <PS2X_lib.h>
#include "config.h"
PS2X ps2;
#define PS2_DAT 13
#define PS2_CMD 11
#define PS2_SEL 10
#define PS2_CLK 12
#define pressures false
#define rumble false
#define DEBOUNCE_DELAY 50.0  // Thời gian debounce 50ms
struct ButtonState {
  bool debouncedState = false;
  unsigned long lastPressTime = 0;
  bool awaitingDebounce = false;
  bool waitRelease = false;
};

ButtonState buttonStates[19];

void debounceButton(int buttonIndex, int buttonCode, bool &stateVar, const char *buttonName) {
  bool pressed = ps2.ButtonPressed(buttonCode);
  bool released = ps2.ButtonReleased(buttonCode);
  // if (buttonIndex == 0 && released) Serial.println("REL");
  float incre=1;
  float incre_rel=1.5;
  if(buttonIndex==14||buttonIndex==17)
  {
    incre=0.5;
    incre_rel=0.4;
  }
  if (pressed && !buttonStates[buttonIndex].debouncedState && !buttonStates[buttonIndex].awaitingDebounce) {
    buttonStates[buttonIndex].lastPressTime = millis();
    buttonStates[buttonIndex].awaitingDebounce = true;
  }


  if (buttonStates[buttonIndex].awaitingDebounce) {
    if (millis() - buttonStates[buttonIndex].lastPressTime > DEBOUNCE_DELAY*incre) {


      buttonStates[buttonIndex].debouncedState = true;
      stateVar = true;
      Serial.print(buttonName);
      Serial.println(" Pressed");


      buttonStates[buttonIndex].awaitingDebounce = false;
    } else {
      if (released) {
        buttonStates[buttonIndex].debouncedState = false;
        buttonStates[buttonIndex].awaitingDebounce = false;
      }
    }
  }

  if (released && buttonStates[buttonIndex].debouncedState && !buttonStates[buttonIndex].waitRelease) {
    buttonStates[buttonIndex].waitRelease = true;
    buttonStates[buttonIndex].lastPressTime = millis();
    // buttonStates[buttonIndex].debouncedState = false;
    // stateVar = false;
    // Serial.print(buttonName);
    // Serial.println(" Released");
  }
  if (buttonStates[buttonIndex].waitRelease) {
    if (millis() - buttonStates[buttonIndex].lastPressTime > (DEBOUNCE_DELAY*incre_rel)) {
      buttonStates[buttonIndex].debouncedState = false;
      buttonStates[buttonIndex].waitRelease = false;
      stateVar = false;
      Serial.print(buttonName);
      Serial.println(" Released");
    } else if (pressed) buttonStates[buttonIndex].waitRelease = false;
  }
}
struct GamepadState {
  bool SELECT;
  bool L3;
  bool R3;
  bool START;
  bool PAD_UP;
  bool PAD_RIGHT;
  bool PAD_DOWN;
  bool PAD_LEFT;
  bool L2;
  bool R2;
  bool L1;
  bool R1;
  bool GREEN;
  bool RED;
  bool BLUE;
  bool PINKI;
  bool TRIANGLE;
  bool CIRCLE;
  bool CROSS;
  bool SQUARE;
  int RX;
  int LX;
};
GamepadState stateBut;
void setup_gamepad() {
  ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);
  int type = ps2.readType();
  switch (type) {
    case 0:
      Serial.print("Tay điều khiển không phù hợp ");
      break;
    case 1:
      Serial.print("Đã tìm thấy DualShock ");
      break;
    case 2:
      Serial.print("Đã tìm thấy GuitarHero ");
      break;
    case 3:
      Serial.print("Không dây của Sony DualShock điều khiển tìm thấy ");
      break;
  }
}
void process_gamepad() {
  ps2.read_gamepad();

    if (state_robot_all == MOVING) {
      debounceButton(0, PSB_PAD_UP, stateBut.PAD_UP, "PAD_UP");
      debounceButton(1, PSB_PAD_RIGHT, stateBut.PAD_RIGHT, "PAD_RIGHT");
      debounceButton(2, PSB_PAD_DOWN, stateBut.PAD_DOWN, "PAD_DOWN");
      debounceButton(3, PSB_PAD_LEFT, stateBut.PAD_LEFT, "PAD_LEFT");
      debounceButton(18, PSB_SQUARE, stateBut.SQUARE, "PSB_SQUARE");
      debounceButton(13, PSB_CIRCLE, stateBut.CIRCLE, "CIRCLE");
      debounceButton(6, PSB_L1, stateBut.L1, "L1");
      stateBut.L2=false;
      stateBut.R2=false;
      stateBut.R1=false;
      stateBut.TRIANGLE=false;
      stateBut.CROSS=false;

    } else if(state_robot_all ==SHOOTING) {
      debounceButton(4, PSB_L2, stateBut.L2, "L2");
      debounceButton(5, PSB_R2, stateBut.R2, "R2");
      debounceButton(7, PSB_R1, stateBut.R1, "R1");
    }

    else if(state_robot_all==SHOOTING_ALIGN)
    {
      debounceButton(14, PSB_TRIANGLE, stateBut.TRIANGLE, "TRIANGLE");
      debounceButton(17, PSB_CROSS, stateBut.CROSS, "PSB_CROSS");
    }
    else if(state_robot_all==STOP_ALL)
    {
      debounceButton(0, PSB_PAD_UP, stateBut.PAD_UP, "PAD_UP");
      debounceButton(1, PSB_PAD_RIGHT, stateBut.PAD_RIGHT, "PAD_RIGHT");
      debounceButton(2, PSB_PAD_DOWN, stateBut.PAD_DOWN, "PAD_DOWN");
      debounceButton(3, PSB_PAD_LEFT, stateBut.PAD_LEFT, "PAD_LEFT");
      debounceButton(18, PSB_SQUARE, stateBut.SQUARE, "PSB_SQUARE");
      debounceButton(13, PSB_CIRCLE, stateBut.CIRCLE, "CIRCLE");
      debounceButton(6, PSB_L1, stateBut.L1, "L1");
      debounceButton(4, PSB_L2, stateBut.L2, "L2");
      debounceButton(5, PSB_R2, stateBut.R2, "R2");
      debounceButton(7, PSB_R1, stateBut.R1, "R1");
       debounceButton(14, PSB_TRIANGLE, stateBut.TRIANGLE, "TRIANGLE");
      debounceButton(17, PSB_CROSS, stateBut.CROSS, "PSB_CROSS");
    }
    
       
      // debounceButton(8, PSB_SELECT, stateBut.SELECT, "SELECT");
      // debounceButton(9, PSB_START, stateBut.START, "START");
      // debounceButton(10, PSB_GREEN, stateBut.GREEN, "GREEN");
      // debounceButton(11, PSB_RED, stateBut.RED, "RED");
      // debounceButton(12, PSB_BLUE, stateBut.BLUE, "BLUE");

     
      // debounceButton(15, PSB_L3, stateBut.L3, "L3");
      // debounceButton(16, PSB_R3, stateBut.R3, "R3");
  // if (ps2.ButtonPressed(PSB_PINK)) {
  //   stateBut.PINKI = true;
  //   Serial.println("PINK (Square) Pressed");
  // } else if (ps2.ButtonReleased(PSB_PINK)) {
  //   stateBut.PINKI = false;
  //   Serial.println("PINK (Square) Released");
  // }
  // int analogRX = ps2.Analog(PSS_RX);
  // analogRX = analogRX - 132;
  // stateBut.RX = analogRX;
  // int analogLX = ps2.Analog(PSS_LX);
  // analogLX = analogLX - 132;
  // stateBut.LX = analogLX;
}