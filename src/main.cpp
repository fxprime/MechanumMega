#define USE_PROTOCAL

#include <Arduino.h>
#include <avr/wdt.h>

#include "protocal.h"
#include "config.h"
#include "kinematic.h"
#include "accel_gyro.h"
#include "ps2.h"
#include "motor_control.h"
#include "pid.hpp"
static inline void modeHandle();
static inline void speedHandle();

MPID mpid_LF(_Mkp, _Mki, _Mkd, false);
MPID mpid_RF(_Mkp, _Mki, _Mkd, false);
MPID mpid_LR(_Mkp, _Mki, _Mkd, false);
MPID mpid_RR(_Mkp, _Mki, _Mkd, false);
HPID headingPID(_Hkp, _Hki, _Hkd);

void setup()
{
  Serial.begin(115200);
  delay(300); //added delay to give wireless ps2 module some time to startup, before configuring it
  ps2_init();
  accel_gyro_init();
  mpid_LF.init();
  mpid_RF.init();
  mpid_LR.init();
  mpid_RR.init();
  headingPID.init();

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
  static uint64_t last_tprotocal = 0;
  static double x = 0;
  static double y = 0;


  /**
   * 
   * 
   * 
   * 
   * TODO Floating point the system variable but convert later to protocal related variables*/

  if (t_now_us - last_t > 1000UL)
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
      double theata_percent = (double)(pos[i] - last_pos[i]) / (60*encoder_slot); // ratio of motor to wheel is 1:60
      double rpm = theata_percent*60.0/dt; // (theata_percent / time) = round per sec  --- > *60 = rpm
      state.wheel.speed[i] = state.wheel.speed[i]*0.85 + 0.15*rpm*RPM_TO_RADPS; //rad/s
      state.wheeldth.theata[i] = 2*M_PI*theata_percent;
    }

    memcpy(&last_pos[0], &pos[0], sizeof(long) * 4);

    //Mecanum forward kinematic
    spos_s dpos_bf;
    fwd_kinematic(state.wheeldth, dpos_bf);
    state.pos_est.thz = wrap_pi(state.pos_est.thz + dpos_bf.thz);
    state.pos_est.x += (dpos_bf.x*cosf(state.pos_est.thz)-dpos_bf.y*sinf(state.pos_est.thz));
    state.pos_est.y += (dpos_bf.x*sinf(state.pos_est.thz)+dpos_bf.y*cosf(state.pos_est.thz));
    fwd_kinematic(state.wheel, state.vel_est);
  }



  if (t_now_us - last_tprotocal > 10000UL)
  {
    last_tprotocal = t_now_us;

    

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
      sensorMsg.est_pos.x = state.pos_est.x*1000;
      sensorMsg.est_pos.y = state.pos_est.y*1000;
      sensorMsg.est_pos.thz = state.pos_est.thz*100;
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
    

    
    String typing = String(state.wheeld.speed[0]) + "," 
                  + String(state.wheel.speed[0]);
    // String typing = String(10*state.vel_est.vx) + "," 
    //               + String(10*state.veld.vx);
    Serial.println(typing);
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
    speedHandle();
    float veldx = 0;
    float veldy = 0;
    float veldwz= 0;

    /**
     * Manual control 
     */
    if ( (state.rc.L1 || state.rc.R1) )
    {
      //Force manual when rc is engaged
      state.mode = mMANUAL;

      float cmd_scale = (state.rc.L1 && state.rc.R1) ? 0.6f : 0.3f;
      float max_speed = cmd_scale * max_linear_spd;
      float max_rotate = cmd_scale * max_turn_spd;

      float forward = (float)(-state.rc.RY + 128) / 127.f;
      float right = -(state.rc.RX - 128)/127;
      float ccwTurn = -(state.rc.LX - 128)/127 ;
      applyDeadbandf(forward, 0.07);
      applyDeadbandf(right, 0.07);
      applyDeadbandf(ccwTurn, 0.07);
      
      
      veldx = forward*max_speed;
      veldy = -right*max_speed;
      veldwz = ccwTurn*max_rotate;
      float vel_norm = sqrtf( veldx*veldx + veldy*veldy );
      if( vel_norm > max_speed ) {
        veldx = max_speed*(veldx / vel_norm);
        veldy = max_speed*(veldy / vel_norm);
      }

    }
    else if(state.mode == mAUTO){
      veldx =0;
      veldy =0;
      veldwz =0;
      state.heading_lock = true;
      
    }
    else {
      veldx =0;
      veldy =0;
      veldwz =0;
      state.heading_lock = true;
    }



    /* -------------------------------------------------------------------------- */
    /*              Gradually velocity to match with velocity desired             */
    /* -------------------------------------------------------------------------- */

    static uint32_t last_accel_t = t_now;
    const float dt = (t_now - last_accel_t)/1000.0;
    last_accel_t = t_now;
    if( veldx==0 && fabs(state.veld.vx - veldx) < max_rc_accel*dt  ) state.veld.vx = 0;
    if( veldy==0 && fabs(state.veld.vy - veldy) < max_rc_accel*dt  ) state.veld.vy = 0;
    state.veld.vx += ( state.veld.vx > veldx ? -max_rc_accel*dt : max_rc_accel*dt );
    state.veld.vy += ( state.veld.vy > veldy ? -max_rc_accel*dt : max_rc_accel*dt );
    if( veldwz==0 && fabs(state.veld.wz - veldwz) < max_rc_waccel*dt  ) {
      state.heading_lock = true;
      state.veld.wz = 0;
    }else{
      state.heading_lock = false;
      state.veld.wz += ( state.veld.wz > veldwz ? -max_rc_waccel*dt : max_rc_waccel*dt );
    }
    state.veld.last_update = t_now;




    /* -------------------------------------------------------------------------- */
    /*                  if not in rate control, let save current                  */
    /* -------------------------------------------------------------------------- */
    static bool last_heading_lock = state.heading_lock;
    if(state.heading_lock && !last_heading_lock) {
      state.pos_d.thz = state.pos_est.thz + state.vel_est.wz*0.2;
    }
    if(!state.heading_lock)
      state.pos_d.thz = state.pos_est.thz;
    last_heading_lock = state.heading_lock;

    
  }



  {

    /**
     * TODO
     * Convert between veld and pwm (Using pid?)
     */ 

    if( t_now - state.veld.last_update < 1000 ) {


      /* -------------------------------------------------------------------------- */
      /*                              Motor control pid                             */
      /* -------------------------------------------------------------------------- */
      {
        static uint32_t last_cnt = millis();
        if( t_now - last_cnt > 10) {
          last_cnt = t_now;
          

          /* -------------------------------------------------------------------------- */
          /*                    keep control yaw when in heading lock                   */
          /* -------------------------------------------------------------------------- */
          if(state.heading_lock)
            state.veld.wz = headingPID.getYawrateD(state.pos_d.thz, state.pos_est.thz);
          else
            headingPID.init();
          
          inv_kinematic(state.veld, state.wheeld);

          /* -------------------------------------------------------------------------- */
          /*                                  Motor Out                                 */
          /* -------------------------------------------------------------------------- */
          motorLF->speed(mpid_LF.getPWM(state.wheeld.speed[0], state.wheel.speed[0]));
          motorRF->speed(mpid_RF.getPWM(state.wheeld.speed[1], state.wheel.speed[1]));
          motorLR->speed(mpid_LR.getPWM(state.wheeld.speed[2], state.wheel.speed[2]));
          motorRR->speed(mpid_RR.getPWM(state.wheeld.speed[3], state.wheel.speed[3]));


          // String typing = String(pwm_rc) + ","
          //             + String(13.4425703667*expf(0.1542795496*state.wheel.speed[0])) + ","
          //             + String(state.wheel.speed[0]);
          // String typing = String(state.wheeld.speed[0]) + "," 
          //               + String(state.wheel.speed[0]) + "," 
          //               + String(10.0*mpid_LF.getPOUT()) + ","
          //               + String(10.0*mpid_LF.getDOUT()) + ","
          //               + String(10.0*mpid_LF.getIOUT());
          // String typing = String(state.wheeld.speed[0]-state.wheel.speed[0]) + "," 
          //               + String(state.wheeld.speed[1]-state.wheel.speed[1]) + "," 
          //               + String(state.wheeld.speed[2]-state.wheel.speed[2]) + "," 
          //               + String(state.wheeld.speed[3]-state.wheel.speed[3]);
          // String typing = String(10*state.veld.wz) + ","
          //               + String(10*state.vel_est.wz);
          // String typing = String(state.wheeld.speed[0]) + "," 
          //               + String(state.wheel.speed[0]);
          // String typing = String(state.vel_est.vx) + "," 
          //               + String(state.veld.vx);
          // Serial.println(typing);


        }
      }
      // motorLF->speed(pwm_rc);
      // motorRF->speed(pwm_rc);
      // motorLR->speed(pwm_rc);
      // motorRR->speed(pwm_rc);


      // motorLF->speed(wspd_to_pwm(state.wheeld.speed[0]));
      // motorRF->speed(wspd_to_pwm(state.wheeld.speed[1]));
      // motorLR->speed(wspd_to_pwm(state.wheeld.speed[2]));
      // motorRR->speed(wspd_to_pwm(state.wheeld.speed[3]));


      // Hard stop if power less than thresold to drain all current back to source faster.
      if (motorLF->getSpeed() != 0 && fabs(state.wheeld.speed[0])<0.3) motorLF->hardStop(); 
      if (motorRF->getSpeed() != 0 && fabs(state.wheeld.speed[1])<0.3) motorRF->hardStop(); 
      if (motorLR->getSpeed() != 0 && fabs(state.wheeld.speed[2])<0.3) motorLR->hardStop(); 
      if (motorRR->getSpeed() != 0 && fabs(state.wheeld.speed[3])<0.3) motorRR->hardStop(); 
      


      
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
 * Speed circling handle when press Button X  or Button O
 */ 
static inline void speedHandle() {
  // static bool last_butX = false;
  // if ( !last_butX && state.rc.but_X ) {
  //   pwm_rc += 3;
  // }
  // last_butX = state.rc.but_X;


  // static bool last_butY = false;
  // if ( !last_butY && state.rc.but_O ) {
  //   pwm_rc -= 3;
  // }
  // last_butY = state.rc.but_O;

  // pwm_rc = constrain(pwm_rc, -255, 255);
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