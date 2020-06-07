#define HEADER 0xf5

#define TEXT_OUT            5
#define ACK                 10
#define SENSOR_STATUS       50
#define SYSTEM_STATUS       51
#define RC_STATUS           52
#define VEL_CNT             53

#define CMD_NAV_VEL        101


#pragma pack(push, 1)


typedef struct {
    uint16_t quad_uid:14;
    uint16_t broadcast:1;
    uint16_t forward:1;
} quad_uid_s;

/* General: Header */
typedef struct {
    uint8_t   header;
    uint16_t   length;
    uint8_t   msg;
    quad_uid_s   id;
} ulink_header_t;

/* General: Checksum */
typedef struct {
    uint16_t   ck;
} ulink_checksum_t ;


/* AUX Struct */
struct wheel_speed_s {
    int16_t speed[4];     // crad/s
    uint16_t last_update; // ms ago
};
struct imu_s {
    int16_t imu_accel[3]; // cm/s^2
    int16_t imu_gyro[3];  // crad/s
    uint16_t last_update; // ms ago
};
typedef enum:uint8_t  {
    mMANUAL=0,
    mAUTO,
    mNum
}mode_enum;
struct vel_s {
    uint32_t  last_update; // ms ago
    int16_t   vx;          // cm/s
    int16_t   vy;          // cm/s
    int16_t   wz;          // crad/s
};

struct pos_s {
    uint32_t  last_update; // ms ago
    int32_t   x;          // cm/s
    int32_t   y;          // cm/s
    int16_t   thz;          // crad/s
};



/* TEXT_OUT */
struct text_out_s {
    char text[58];
};

/* SENSOR_STATUS */
struct sensor_status_s {
    wheel_speed_s   wheel_speed;
    imu_s           imu;
    vel_s           est_speed;
    pos_s           est_pos;
};


/* SYSTEM_STATUS */
struct system_status_s {
    mode_enum mode;
    uint8_t  imu_ok:1;
    uint8_t  joy_ok:1;
    uint8_t  encoder_ok:1;
};


/* RC_STATUS */
struct rc_status_s {
    uint32_t last_update;
    int16_t  LX;
    int16_t  LY;
    int16_t  RX;
    int16_t  RY;
    uint8_t  but_X:1;
    uint8_t  but_O:1;
    uint8_t  but_A:1;
    uint8_t  but_S:1;
    uint8_t  L1:1;
    uint8_t  L2:1;
    uint8_t  R1:1;
    uint8_t  R2:1;
};
/* CNT_STATUS */
struct cnt_status_s {
    uint32_t  last_update;
    vel_s vel_cnt;
};


struct ack_s {
    uint8_t ACK_RESERVED :1;
    uint8_t ACK_POS_DES :1;
    uint8_t ACK_CMD_DO_SET_CONFIG :1;
    uint8_t ACK_CMD_DO_SET_PATTERN:1;
    uint8_t ACK_CMD_DO_SET_MODE   :1;
    uint8_t ACK_CMD_DO_TAKEOFF    :1;
};


#pragma pack(pop)


const char * mode_interpret(mode_enum mode)
{
    switch(mode)
    {
        case mMANUAL:          return "MANUAL";
        case mAUTO:            return "AUTO";
        default: return "Invalid";
    }
}