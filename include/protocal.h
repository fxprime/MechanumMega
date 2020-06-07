
#define toUint32(X)    *(uint32_t*)(&X)
#define toUint8(X)    *(uint8_t*)(&X)
#define toSint8(X)    *(int8_t*)(&X)
#define toUint(X)    *(uint16_t*)(&X)
#define toSint(X)    *(int16_t*)(&X)
#define toFlt(X)    *(float*)(&X)
#define toLong(X)    *(int32_t*)(&X)

uint8_t st = 0, _msgid, _pay[2048], _c[2],_lenp[2], _uid[2];
uint16_t _lenpsum,_idp, _sum_pay, _csum;
uint16_t _quad_uid = 0;


static inline void packet_decode(const uint16_t& quad_id);
static inline void send_text_out(const String& str);

inline void serial_handle(uint8_t *buf, uint16_t len)
{

    uint8_t msgReceived = false;

    for (int i =0;i<len;i++)
    {

        // Check if a message could be decoded, return the message in case yes
        unsigned char data = buf[i];

        switch (st) {
            case 0:
                _sum_pay = 0;
                if (data == HEADER) {
                    _sum_pay += data;
                    st++;
                    // send_text_out("header");
                }
                break;


                //Collect payload length(2bytes)
            case 1 :
                _lenp[0] = data;
                _sum_pay += data;
                st++;
                // send_text_out("l0");
                break;
            case 2 :
                _lenp[1] = data;
                _lenpsum = toUint(_lenp);
                _sum_pay += data;
                // send_text_out("l1 len=" +String(_lenpsum));
                if(_lenpsum < 256)
                    st++;
                else{
                    // send_text_out("Over length!!");
                    _lenpsum = 0;
                    st=0;
                }
                break;

                //collect msgid 1byte
            case 3 :
                _msgid = data;
                _sum_pay += data;
                st++;
                // send_text_out("msgid " + String(_msgid));
                break;

                //collect uid 2 bytes
            case 4 :
                _sum_pay += data;
                _uid[0] = data;
                st++;
                // send_text_out("u0");
                break;
            case 5 :
                _sum_pay += data;
                _uid[1] = data;
                _quad_uid = toUint(_uid);
                _idp = 0;
                // send_text_out("u1 uid="+String(_quad_uid));
                st++;
                break;


                //collect payload
            case 6 :
                _pay[_idp++] = data;
                _sum_pay += data;
                // send_text_out(String(_idp));
                if (_idp >= _lenpsum) st++;
                break;

                //collect checksum
            case 7 :
                _c[0] = data;
                st++;
                // send_text_out("chk1");
                break;
            case 8 :
                _c[1] = data;
                _csum = toUint(_c);
                st = 0;
                // send_text_out("chk2");

                // Serial.printf("_msgid %d _lenp %d vs %d quid %d chksum %d %d", _msgid, _lenpsum, _idp, _quad_uid, _sum_pay, _csum);
                if (_sum_pay == _csum) {

                    packet_decode(_quad_uid);
                    msgReceived = true;
                }
                break;
        }
    }
}



void
calcChecksum(const uint8_t *buffer, const uint16_t length, ulink_checksum_t *checksum)
{
    for (uint16_t i = 0; i < length; i++) {
        checksum->ck = checksum->ck + buffer[i];
    }
}


inline bool
sendMessage(const uint8_t msg, const uint16_t id, const uint8_t *payload, const uint16_t length)
{ 

    #ifndef USE_PROTOCAL
        return true;
    #endif
    ulink_header_t   header = {0};
    ulink_checksum_t checksum = {0};

    // Populate header
    header.header = HEADER;
    header.msg  = msg;
    header.id.quad_uid = id;
    header.length = length;
 
    // Calculate checksum
    calcChecksum(((uint8_t *)&header) , sizeof(header), &checksum);

    if (payload != nullptr) {
        calcChecksum(payload, length, &checksum);
    }
    
    
    if (Serial.write((char *)&header, sizeof(header)) != sizeof(header)) {
        // Serial.println("Write failed closing port..");
        delay(1000);

        // Serial.println("Done..Exit");
        delay(1000);
        exit(0);
        return false;
    }


    if (payload && Serial.write((char *)payload, length) != length) {
        // Serial.println("write payload failed");
        return false;
    }

    if (Serial.write((char *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
        // Serial.println("write checksum failed");
        return false;
    }
    

    return true;
}















static inline void packet_decode(const uint16_t& quad_uid )
{

 
    if (_msgid == ACK) {  
    }

    else if (_msgid == CMD_NAV_VEL) {
        cnt_status_s msg;
        memcpy(&msg, &_pay[0], sizeof(msg));
        state.veld_navi.vx = msg.vel_cnt.vx*0.01;
        state.veld_navi.vy = msg.vel_cnt.vy*0.01;
        state.veld_navi.wz = msg.vel_cnt.wz*0.01;
        state.veld_navi.last_update = millis();
        // send_text_out("veld navi " + String(state.veld_navi.vx,2) + ","  + String(state.veld_navi.vy,2) + ","  + String(state.veld_navi.wz,2));
    }
}




static inline void send_sensor_status(uint16_t quad_id, sensor_status_s &msg) {
    sendMessage(SENSOR_STATUS, quad_id, (uint8_t *)&msg, sizeof(msg)); 
}

static inline void send_system_status(uint16_t quad_id, system_status_s &msg) {
    sendMessage(SYSTEM_STATUS, quad_id, (uint8_t *)&msg, sizeof(msg)); 
}
static inline void send_rc_status(uint16_t quad_id, rc_status_s &msg) {
    sendMessage(RC_STATUS, quad_id, (uint8_t *)&msg, sizeof(msg)); 
}
static inline void send_cnt_status(uint16_t quad_id, cnt_status_s &msg) {
    sendMessage(VEL_CNT, quad_id, (uint8_t *)&msg, sizeof(msg)); 
}
static inline void send_text_out(const String& str) {

    
    text_out_s msg;
    memcpy(&msg.text[0], &str.c_str()[0], sizeof(msg.text));
    sendMessage(TEXT_OUT, this_quid, (uint8_t *)&msg, sizeof(msg)); 
    
}
static inline void send_text_out(const float& hz, const String& str) {

    uint32_t t_now = millis();
    static uint32_t last_send = t_now;
    
    if( t_now - last_send > 1000.0/hz ){
        last_send = t_now;
        text_out_s msg;
        memcpy(&msg.text[0], &str.c_str()[0], sizeof(msg.text));
        sendMessage(TEXT_OUT, this_quid, (uint8_t *)&msg, sizeof(msg)); 
    }
    
}