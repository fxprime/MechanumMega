#define HEADER 0xf5

#define ACK                 10
#define SENSOR_STATUS       50
#define SYSTEM_STATUS       51
#define RC_STATUS           52
#define CMD_DO_1            100
#define CMD_DO_2            100


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
struct est_speed_s {
    int32_t x;
    int32_t y;
    int32_t omega_z;
};
struct wheel_speed_s {
    int32_t M1[4];
};
struct imu_s {
    int32_t imu_accel[3];
    int32_t imu_gyro[3];
};


/* SENSOR_STATUS */
struct status_packed_s {
    wheel_speed_s   wheel_speed;
    imu_s           imu;
    est_speed_s     est_speed;
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


