#include <PS2X_lib.h>
PS2X ps2;
#define PS2_DAT 13
#define PS2_CMD 11
#define PS2_SEL 10
#define PS2_CLK 12
#define pressures false
#define rumble false
bool running = 0;
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
  static int count_release=0;
  // count_relase+=1;
  if (ps2.ButtonPressed(PSB_PAD_UP)) {
    stateBut.PAD_UP = true;
    Serial.println("PAD_UP Pressed");
  } else if (ps2.ButtonReleased(PSB_PAD_UP)) {
    stateBut.PAD_UP = false;
    Serial.println("PAD_UP Released");
  }

  if (ps2.ButtonPressed(PSB_PAD_RIGHT)) {
    stateBut.PAD_RIGHT = true;
    Serial.println("PAD_RIGHT Pressed");
  } else if (ps2.ButtonReleased(PSB_PAD_RIGHT)) {
    stateBut.PAD_RIGHT = false;
    Serial.println("PAD_RIGHT Released");
  }

  if (ps2.ButtonPressed(PSB_PAD_DOWN)) {
    stateBut.PAD_DOWN = true;
    Serial.println("PAD_DOWN Pressed");
  } else if (ps2.ButtonReleased(PSB_PAD_DOWN)) {
    stateBut.PAD_DOWN = false;
    Serial.println("PAD_DOWN Released");
  }

  if (ps2.ButtonPressed(PSB_PAD_LEFT)) {
    stateBut.PAD_LEFT = true;
    Serial.println("PAD_LEFT Pressed");
  } else if (ps2.ButtonReleased(PSB_PAD_LEFT)) {
    stateBut.PAD_LEFT = false;
    Serial.println("PAD_LEFT Released");
  }
  if (!running) {
    if (ps2.ButtonPressed(PSB_SELECT)) {
      stateBut.SELECT = true;
      Serial.println("SELECT Pressed");
    } else if (ps2.ButtonReleased(PSB_SELECT)) {
      stateBut.SELECT = false;
      Serial.println("SELECT Released");
    }
    if (ps2.ButtonPressed(PSB_L3)) {
      stateBut.L3 = true;
      Serial.println("L3 Pressed");
    } else if (ps2.ButtonReleased(PSB_L3)) {
      stateBut.L3 = false;
      Serial.println("L3 Released");
    }

    if (ps2.ButtonPressed(PSB_R3)) {
      stateBut.R3 = true;
      Serial.println("R3 Pressed");
    } else if (ps2.ButtonReleased(PSB_R3)) {
      stateBut.R3 = false;
      Serial.println("R3 Released");
    }

    if (ps2.ButtonPressed(PSB_START)) {
      stateBut.START = true;
      Serial.println("START Pressed");
    } else if (ps2.ButtonReleased(PSB_START)) {
      stateBut.START = false;
      Serial.println("START Released");
    }
    if (ps2.ButtonPressed(PSB_L2)) {
      stateBut.L2 = true;
      Serial.println("L2 Pressed");
    } else if (ps2.ButtonReleased(PSB_L2)) {
      
      stateBut.L2 = false;
      Serial.println("L2 Released");
    }

    if (ps2.ButtonPressed(PSB_R2)) {
      stateBut.R2 = true;
      Serial.println("R2 Pressed");
    } else if (ps2.ButtonReleased(PSB_R2)) {
      stateBut.R2 = false;
      Serial.println("R2 Released");
    }

    if (ps2.ButtonPressed(PSB_L1)) {
      stateBut.L1 = true;
      Serial.println("L1 Pressed");
    } else if (ps2.ButtonReleased(PSB_L1)) {
      stateBut.L1 = false;
      Serial.println("L1 Released");
    }

    if (ps2.ButtonPressed(PSB_R1)) {
      stateBut.R1 = true;
      Serial.println("R1 Pressed");
    } else if (ps2.ButtonReleased(PSB_R1)) {
      stateBut.R1 = false;
      Serial.println("R1 Released");
    }

    if (ps2.ButtonPressed(PSB_GREEN)) {
      stateBut.GREEN = true;
      Serial.println("GREEN (Triangle) Pressed");
    } else if (ps2.ButtonReleased(PSB_GREEN)) {
      stateBut.GREEN = false;
      Serial.println("GREEN (Triangle) Released");
    }

    if (ps2.ButtonPressed(PSB_RED)) {
      stateBut.RED = true;
      Serial.println("RED (Circle) Pressed");
    } else if (ps2.ButtonReleased(PSB_RED)) {
      stateBut.RED = false;
      Serial.println("RED (Circle) Released");
    }

    if (ps2.ButtonPressed(PSB_BLUE)) {
      stateBut.BLUE = true;
      Serial.println("BLUE (Cross) Pressed");
    } else if (ps2.ButtonReleased(PSB_BLUE)) {
      stateBut.BLUE = false;
      Serial.println("BLUE (Cross) Released");
    }
    if (ps2.ButtonPressed(PSB_SQUARE)) {
      stateBut.SQUARE = true;
      Serial.println("SQUARE (Cross) Pressed");
    } else if (ps2.ButtonReleased(PSB_SQUARE)) {
      stateBut.SQUARE = false;
      Serial.println("SQUARE (Cross) Released");
    }
    if (ps2.ButtonPressed(PSB_CIRCLE)) {
      stateBut.CIRCLE = true;
      Serial.println("PSB_CIRCLE (Cross) Pressed");
    } else if (ps2.ButtonReleased(PSB_CIRCLE)) {
      stateBut.CIRCLE = false;
      Serial.println("PSB_CIRCLE (Cross) Released");
    }
    if (ps2.ButtonPressed(PSB_TRIANGLE)) {
      stateBut.TRIANGLE = true;
      Serial.println("PSB_TRIANGLE (Cross) Pressed");
    } else if (ps2.ButtonReleased(PSB_TRIANGLE)) {
      stateBut.TRIANGLE = false;
      Serial.println("PSB_TRIANGLE (Cross) Released");
    }

    // if (ps2.ButtonPressed(PSB_PINK)) {
    //   stateBut.PINKI = true;
    //   Serial.println("PINK (Square) Pressed");
    // } else if (ps2.ButtonReleased(PSB_PINK)) {
    //   stateBut.PINKI = false;
    //   Serial.println("PINK (Square) Released");
    // }
    int analogRX = ps2.Analog(PSS_RX);
    analogRX = analogRX - 132;
    stateBut.RX = analogRX;
    int analogLX = ps2.Analog(PSS_LX);
    analogLX = analogLX - 132;
    stateBut.LX = analogLX;
  }
}