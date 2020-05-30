
// NOTE: Requires the PS2X_lib installed.
// 1) open Sketch -> Include Library -> Add .ZIP Library
// 2) select "PS2X_lib.zip"
#include <PS2X_lib.h>

//PS2
#define PS2_DAT 52 //14
#define PS2_CMD 51 //15
#define PS2_SEL 53 //16
#define PS2_CLK 50 //17

// #define pressures   true
#define pressures false
#define rumble true
//#define rumble      false
PS2X ps2x; // create PS2 Controller Class

int error = 0;
byte type = 0;

void (*resetFunc)(void) = 0;

void ps2_init()
{

    //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

    if (error == 0)
    {
        Serial.println("M1, M2, M3, M4");
        //    Serial.println("Found Controller, configuration successful ");
        //    Serial.println();
        //    Serial.println("SPDMotor control by Aaron Hilton of Steampunk Digital");
        //    Serial.println("=====================================================");
        //    Serial.println("Holding L1 or R1 will activate analog joystick control.");
        //    Serial.println("Left analog stick for forward/back and turning.");
        //    Serial.println("Right analog stick for sideways movement.");
        //    Serial.println("Hold both L1 and R1 for full-speed.");
    }
    else if (error == 1)
    {
        Serial.println("No controller found, check PS2 receiver is inserted the correct way around.");
        resetFunc();
    }
    else if (error == 2)
        Serial.println("Controller found but not accepting commands.");

    else if (error == 3)
        Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

    type = ps2x.readType();
    switch (type)
    {
    case 0:
        Serial.println("M1, M2, M3, M4");

        break;
    case 1:
        Serial.println("DualShock Controller found ");
        break;
    case 2:
        Serial.println("GuitarHero Controller found ");
        break;
    case 3:
        Serial.println("Wireless Sony DualShock Controller found ");
        break;
    }
}

static inline void ps2_update()
{
    uint32_t t_now = millis();
    static uint32_t last_ps2 = t_now;

    if (t_now - last_ps2 > 100)
    {



        last_ps2 = t_now;

        if (error == 1) //skip loop if no controller found
            return;

        if (type == 2)
        { //Guitar Hero Controller
            return;
        }
        else
        { //DualShock Controller

            ps2x.read_gamepad(false, 0); //read controller and set large motor to spin at 'vibrate' speed

            state.rc.L1 = ps2x.Button(PSB_L1);
            state.rc.R1 = ps2x.Button(PSB_R1);
            state.rc.L2 = ps2x.Button(PSB_L2);
            state.rc.R2 = ps2x.Button(PSB_R2);

            
            state.rc.LX = ps2x.Analog(PSS_LX);
            state.rc.LY = ps2x.Analog(PSS_LY);
            state.rc.RX = ps2x.Analog(PSS_RX);
            state.rc.RY = ps2x.Analog(PSS_RY);
            state.rc.last_update = t_now;

            // String typing = String(state.rc.LX) + "\t" + String(state.rc.LY) + "\t" + String(state.rc.RX) + "\t" + String(state.rc.RY);
            // Serial.println(typing);
        }
    }
}

static inline bool is_ps2_ok() {
    return (millis() - state.rc.last_update < 3000);
}