
const float this_quid = 33;
const float R = 0.026;
const float L1 =0.078;
const float L2 =0.078;
const float wheelk = (1/(L1+L2));
const double encoder_slot = 46;
const double RPM_TO_RADPS = 2.0*M_PI/60.0;

const float max_wspd = 17.5; //rad/s
const float max_linear_spd = 0.4; //m/s
const float max_angular_spd = 2.5; //rad/s

/**
 * Use absolute timestamp for last_update variable here
 * 
 */ 
struct swheel_speed_s {
    double speed[4];        // rad/s
    uint32_t last_update;  // ms ago
};
struct simu_s {
    float imu_accel[3];    // m/s^2
    float imu_gyro[3];     // rad/s
    uint32_t last_update;  // ms ago
};
struct svel_s {
    float     vx;          // m/s
    float     vy;          // m/s
    float     wz;          // rad/s
    uint32_t  last_update; // ms ago
};


typedef struct {
    
    swheel_speed_s  wheel;
    swheel_speed_s  wheeld;
    simu_s          imu;
    svel_s          vel_est;
    svel_s          veld;
    rc_status_s     rc;
    mode_enum       mode;

}state_s;


state_s state;


