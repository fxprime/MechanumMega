#include <Arduino.h>
#include <avr/wdt.h>

#include "config.h"
#include "protocal.h"
#include "accel_gyro.h"
#include "ps2.h"
#include "motor_control.h"


void setup()
{
  Serial1.begin(115200);
  Serial.begin(115200);
  delay(300); //added delay to give wireless ps2 module some time to startup, before configuring it
  ps2_init();
  accel_gyro_init();

}


void loop()
{

  accel_gyro_update();

  {
    uint8_t buf[1];
    memset(&buf[0], 0, sizeof(buf));

    buf[0] = Serial1.read();
    serial_handle(&buf[0], 1);
  }

  uint32_t t_now = millis();
  uint64_t t_now_us = micros();
  static uint64_t last_t = 0;
  static double x = 0;
  static double y = 0;

  if (t_now_us - last_t > 10000UL)
  {
    double dt = (double)(t_now_us - last_t) / 1000000.0;
    last_t = t_now_us;
    static long last_pos[4] = {0};
    long pos[4] = {motorLF->getEncoderPosition(),
                   motorRF->getEncoderPosition(),
                   motorLR->getEncoderPosition(),
                   motorRR->getEncoderPosition()};
    double spd[4];
    for (int i = 0; i < 4; i++)
    {
      spd[i] = (double)(pos[i] - last_pos[i]) / (encoder_slot * dt); //rpm
      spd[i] = spd[i]*RPM_TO_RADPS; //rad/s
    }


    //Mecanum forward kinematic
    double vxfb = 0.25 * R * (spd[0] + spd[1] + spd[2] + spd[3]);
    double vyfb = 0.25 * R * (spd[0] - spd[1] - spd[2] + spd[3]);
    double w0fb = 0.25 * R * wheelk*(-spd[0] + spd[1] - spd[2] + spd[3]);

    memcpy(&last_pos[0], &pos[0], sizeof(long) * 4);

    // String msg = String(spd[0]) + "\t"
    //           + String(spd[1]) + "\t"
    //           + String(spd[2]) + "\t"
    //           + String(spd[3]);

    // String msg = String(vxfb) + "\t" + String(vyfb) + "\t" + String(w0fb);
    // String msg = String(AccX) + "\t" + String(AccY)+ "\t" + String(AccZ) + "\t" + String(GyroZ);
    // String msg = String(w0fb) + "\t" + String(GyroZ);
    String msg = String(AccX) + "\t" + String(AccY);
    Serial.println(msg);
  }

  ps2_update();

  if (is_ps2_ok())
  {

    static int wheel_num = 0;
    static bool last_mode = false;
    if (last_mode != ps2x.Button(PSB_L2))
    {
      last_mode = ps2x.Button(PSB_L2);
      if (ps2x.Button(PSB_L2))
        wheel_num = (wheel_num + 1) % 4;
      Serial.println("Mode changed " + String(wheel_num));
    }

    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
    {
      int LY = ps2x.Analog(PSS_RY);
      int LX = ps2x.Analog(PSS_LX);
      int RX = ps2x.Analog(PSS_RX);
      float forwardNormalized = (float)(-LY + 128) / 127.f;

      forwardNormalized = constrain(forwardNormalized, -1.f, 1.f);
      float multiplier = (ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1)) ? 255.f : 125.f;
      int forward = (int)(pow(forwardNormalized, 2.0) * multiplier);

      // Preserve the direction of movement.
      if (forwardNormalized < 0)
      {
        forward = -forward;
      }

      int right = -RX + 127;
      int ccwTurn = (LX - 127) / 2;

      float wds = -ccwTurn * 0.5;
      float vx = forward * 0.05;
      float vy = -right * 0.05;

      float ro = (L1 + L2) * wds;

      float w1 = (vx + vy - ro) / R;
      float w2 = (vx - vy + ro) / R;
      float w3 = (vx - vy - ro) / R;
      float w4 = (vx + vy + ro) / R;

      motorLF->speed(w1);
      motorRF->speed(w2);
      motorLR->speed(w3);
      motorRR->speed(w4);

      //        motorLF->speed(forward + ccwTurn - right); motorRF->speed(forward - ccwTurn + right);
      //        motorLR->speed(forward - ccwTurn - right); motorRR->speed(forward + ccwTurn + right);
    }
    else
    {
      // If there's motor power, try to hard-stop briefly.
      if (motorLF->getSpeed() != 0 || motorRF->getSpeed() != 0 || motorLR->getSpeed() != 0 || motorRR->getSpeed() != 0)
      {
        motorLF->hardStop();
        motorRF->hardStop();
        motorLR->hardStop();
        motorRR->hardStop();
      }
    }
  }
}