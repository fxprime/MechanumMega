
const float this_quid = 33;
const float R = 0.03;
const float L1 =0.01*20.0/2; //Wheel to center sideway
const float L2 =0.01*18.8/2; //Wheel to center front way
const float magic=  1.04022989;
const float wheelk = (1/(L1+L2));
const double encoder_slot = 46;
const double RPM_TO_RADPS = 2.0*M_PI/60.0;

const float max_wspd = 20.0; //rad/s
const float max_linear_spd = 0.4; //m/s
const float max_angular_spd = 2.5; //rad/s


const float _Mkp = 0.2;
const float _Mki = 0.01;
const float _Mkd = 0.05;

const float _Hkp = 0.2;
const float _Hki = 0.01;
const float _Hkd = 0.0;

/**
 * Use absolute timestamp for last_update variable here
 * 
 */ 
struct swheel_speed_s {
    double speed[4];        // rad/s
    uint32_t last_update;  // ms ago
};
struct swheel_theata_s {
    double theata[4];        // rad/s
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
struct spos_s {
    float     x;          // m
    float     y;          // m
    float     thz;          // rad
    uint32_t  last_update; // ms ago
};


typedef struct {
    
    swheel_speed_s  wheel;
    swheel_speed_s  wheeld;     //wheel speed desired 
    swheel_theata_s wheeldth;   //Instant theata of discreate time
    simu_s          imu;
    svel_s          vel_est;
    spos_s          pos_est;
    spos_s          pos_d;
    svel_s          veld;
    rc_status_s     rc;
    mode_enum       mode;
    bool            heading_lock;

}state_s;


state_s state;



#define wrap_pi(x) (x < -M_PI ? x+2*M_PI : (x > M_PI ? x - 2*M_PI: x))


float pwm_rc = 0;