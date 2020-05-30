#define USE_PROTOCAL

#include <Arduino.h>
#include <avr/wdt.h>

#include "protocal.h"
#include "config.h"
#include "kinematic.h"
#include "accel_gyro.h"
#include "ps2.h"
#include "motor_control.h"
static inline void modeHandle();

void setup()
{
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

    buf[0] = Serial.read();
    serial_handle(&buf[0], 1);
  }

  uint32_t t_now = millis();
  uint64_t t_now_us = micros();
  static uint64_t last_t = 0;
  static double x = 0;
  static double y = 0;


  /**
   * 
   * 
   * 
   * 
   * TODO Floating point the system variable but convert later to protocal related variables*/

  if (t_now_us - last_t > 10000UL)
  {
    double dt = (double)(t_now_us - last_t) / 1000000.0;
    last_t = t_now_us;
    static long last_pos[4] = {0};
    long pos[4] = {motorLF->getEncoderPosition(),
                   motorRF->getEncoderPosition(),
                   motorLR->getEncoderPosition(),
                   motorRR->getEncoderPosition()};
    for (int i = 0; i < 4; i++)
    {
      state.wheel.speed[i] = (double)(pos[i] - last_pos[i]) / (encoder_slot * dt); //rpm
      state.wheel.speed[i] = state.wheel.speed[i]*RPM_TO_RADPS; //rad/s
    }
    memcpy(&last_pos[0], &pos[0], sizeof(long) * 4);


    //Mecanum forward kinematic
    fwd_kinematic(state.wheel, state.vel_est);

    {
      sensor_status_s sensorMsg;
      sensorMsg.imu.imu_accel[0] = AccX*100;
      sensorMsg.imu.imu_accel[1] = AccY*100;
      sensorMsg.imu.imu_accel[2] = AccZ*100;
      sensorMsg.imu.imu_gyro[0] = GyroX*100;
      sensorMsg.imu.imu_gyro[1] = GyroY*100;
      sensorMsg.imu.imu_gyro[2] = GyroZ*100;
      sensorMsg.imu.last_update = t_now - last_imu_update;
      sensorMsg.wheel_speed.speed[0] = state.wheel.speed[0]*100;
      sensorMsg.wheel_speed.speed[1] = state.wheel.speed[1]*100;
      sensorMsg.wheel_speed.speed[2] = state.wheel.speed[2]*100;
      sensorMsg.wheel_speed.speed[3] = state.wheel.speed[3]*100;
      sensorMsg.wheel_speed.last_update = 0;
      sensorMsg.est_speed.vx = state.vel_est.vx*100;
      sensorMsg.est_speed.vy = state.vel_est.vy*100;
      sensorMsg.est_speed.wz = state.vel_est.wz*100;
      send_sensor_status(this_quid, sensorMsg);
    }

    
    {
      rc_status_s rcMsg;
      memcpy(&rcMsg, &state.rc, sizeof(rcMsg));
      rcMsg.last_update = t_now - state.rc.last_update;
      send_rc_status(this_quid, rcMsg);
    }


    {
      static uint32_t last_sent = millis();
      if( t_now - last_sent > 33) {
        last_sent = t_now;
        cnt_status_s msg;
        msg.vel_cnt.vx = state.veld.vx*100;
        msg.vel_cnt.vy = state.veld.vy*100;
        msg.vel_cnt.wz = state.veld.wz*100;
        msg.vel_cnt.last_update = t_now - state.veld.last_update;
        send_cnt_status(this_quid, msg);
      }
    }
    

    


    // String typing = String(state.wheel.speed[0]) + "\t"
    //           + String(state.wheel.speed[1]) + "\t"
    //           + String(state.wheel.speed[2]) + "\t"
    //           + String(state.wheel.speed[3]);

    // // String typing = String(vxfb) + "\t" + String(vyfb) + "\t" + String(w0fb);
    // // String typing = String(AccX) + "\t" + String(AccY)+ "\t" + String(AccZ) + "\t" + String(GyroZ);
    // String typing = String(state.vel_est.wz) + "\t" + String(GyroZ);
    // // String typing = String(AccX) + "\t" + String(AccY);
    // Serial.println(typing);
  }


  /**************
   * Update Joy |
   *************/ 
  ps2_update();



  // static uint32_t last_gg = millis();
  // if(millis() - last_gg > 100) {
  //   last_gg = millis();
  //   float val = -200.0*(millis()%2000>1000 ? 1.0:-1.0);
  //   motorRR->speed(val);



  //   Serial.println(val);
  
  // }







  /**
   * Allow control motor while joy is ok (for now) 
   */
  if (is_ps2_ok())
  {

    modeHandle();

    /**
     * Manual control 
     */
    if ( (state.rc.L1 || state.rc.R1) )
    {
      //Force manual when rc is engaged
      state.mode = mMANUAL;
      int LY = state.rc.RY;
      int LX = state.rc.LX;
      int RX = state.rc.RX;
      float forwardNormalized = (float)(-LY + 128) / 127.f;

      forwardNormalized = constrain(forwardNormalized, -1.f, 1.f);
      float multiplier = (state.rc.L1 && state.rc.R1) ? 255.f : 125.f;
      int forward = (int)(pow(forwardNormalized, 2.0) * multiplier);

      // Preserve the direction of movement.
      if (forwardNormalized < 0)
      {
        forward = -forward;
      }

      int right = -RX + 127;
      int ccwTurn = (LX - 127) ;

      
      const float cmd_scale = 0.5;
      state.veld.vx = cmd_scale*forward*max_linear_spd/127.0;
      state.veld.vy = -cmd_scale*right*max_linear_spd/127.0;
      state.veld.wz = -cmd_scale*ccwTurn*max_angular_spd/127.0;
      state.veld.last_update = t_now;


      // float test_force = (float)state.veld.vx*255.0/10.0;
      // Serial.println(test_force);
      // motorLF->speed(test_force);
      // motorRF->speed(test_force);
      // motorLR->speed(test_force);
      // motorRR->speed(test_force);
      // String typing = String(state.wheeld.speed[0]) + "\t"
      //   + String(state.wheeld.speed[1]) + "\t"
      //   + String(state.wheeld.speed[2]) + "\t"
      //   + String(state.wheeld.speed[3]); 
      // String typing = String(state.veld.vx) + "\t"
      //   + String(state.veld.vy) + "\t"
      //   + String(state.veld.wz);
      // Serial.println(typing);
    }
    else if(state.mode == mAUTO){
      state.veld.vx =0;
      state.veld.vy =0;
      state.veld.wz =0;
      state.veld.last_update = t_now;
      
    }
  }



  /**
   * Motor out
   */
  {
    inv_kinematic(state.veld, state.wheeld);

    // String typing = String(state.wheeld.speed[0]) + "\t"
    //     + String(state.wheeld.speed[1]) + "\t"
    //     + String(state.wheeld.speed[2]) + "\t"
    //     + String(state.wheeld.speed[3]); 
    // Serial.println(typing);
    /**
     * TODO
     * Convert between veld and pwm (Using pid?)
     */ 

    if( t_now - state.veld.last_update < 1000 ) {
      motorLF->speed(wspd_to_pwm(state.wheeld.speed[0]));
      motorRF->speed(wspd_to_pwm(state.wheeld.speed[1]));
      motorLR->speed(wspd_to_pwm(state.wheeld.speed[2]));
      motorRR->speed(wspd_to_pwm(state.wheeld.speed[3]));


      // Hard stop if power less than thresold to drain all current back to source faster.
      if (motorLF->getSpeed() != 0 && fabs(state.wheeld.speed[0])<5) motorLF->hardStop(); 
      if (motorRF->getSpeed() != 0 && fabs(state.wheeld.speed[1])<5) motorRF->hardStop(); 
      if (motorLR->getSpeed() != 0 && fabs(state.wheeld.speed[2])<5) motorLR->hardStop(); 
      if (motorRR->getSpeed() != 0 && fabs(state.wheeld.speed[3])<5) motorRR->hardStop(); 
      


      
    }else{
      
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


/**
 * Mode circling handle when press Button X 
 */ 
static inline void modeHandle() {
  static bool last_but = false;
  if ( !last_but && state.rc.but_X ) {
    state.mode = (mode_enum)((state.mode+1)%mNum);
    Serial.println("Mode chaged! to " + String(mode_interpret(state.mode)));
  }
  last_but = state.rc.but_X;
}