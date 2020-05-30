/**
 * ArduinoNa : Lightsaber project
 * Author : Thanabadee Bulunseechart
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <paheyisoicus@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Poul-Henning Kamp
 */
 
// #include <Wire.h>
#include <WSWire.h>

#include "mpu6050.h"


void accel_gyro_init() {
  Wire.begin();
  mpu6050_initialize();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz 


}

static inline void accel_gyro_update() {


  // mpu6050_Accel_Values();
  // mpu6050_Gyro_Values();

  uint32_t t_now = micros();
  static uint32_t last_t = micros();

  if(t_now - last_t > 1001) {
    last_t = millis();
    mpu6050_readGyroSum();
    mpu6050_readAccelSum();
  }

  if(gyroSamples > 4) {

    mpu6050_Get_accel();
    mpu6050_Get_gyro();
    state.imu.imu_accel[0] = AccX;
    state.imu.imu_accel[1] = AccY;
    state.imu.imu_accel[2] = AccZ;
    state.imu.imu_gyro[0] = GyroX;
    state.imu.imu_gyro[1] = GyroY;
    state.imu.imu_gyro[2] = GyroZ;
    
  }




//   float norm_a = (float)(fabs(accelRaw[XAXIS])) + (float)(fabs(accelRaw[YAXIS])) + (float)(fabs(accelRaw[ZAXIS]));
//   float norm_g = (float)(fabs(gyroRaw[XAXIS])) + (float)(fabs(gyroRaw[YAXIS])) + (float)(fabs(gyroRaw[ZAXIS]));

//   if (norm_a > 8000 && _light_st == light_on) {
//     _light_rq = light_crash;
//   }
//   if (norm_g > 2000 && norm_g < 3000 && _light_st == light_on && _light_rq == light_req_done ) {
//     _light_rq = light_swing_low;
//   }
//   else if (norm_g >= 3000 && _light_st == light_on && _light_rq == light_req_done) {
//     _light_rq = light_swing_fast;
//   }
}