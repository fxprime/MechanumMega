#ifndef _PID_HPP_
#define _PID_HPP_


  
/* -------------------------------------------------------------------------- */
/*                          ฟังก์ชัน map ค่าแบบทศนิยม                             */
/* -------------------------------------------------------------------------- */
static inline double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



/* -------------------------------------------------------------------------- */
/*    ฟังก์ชันทำ deadband สำหรับตัดค่าที่ต่ำกว่า deadband ให้กลายเป็น 0 สำหรับช่วง [-1,1]  */
/* -------------------------------------------------------------------------- */
static inline float applyDeadbandf(const float& value, const float& d) {
  float out=0;
  if (value>d) 
    out = (value-d)/(1-d);
  else if (value>-d) 
    out = 0;
  else 
    out = (value+d)/(1-d);
  return out;
}

/* -------------------------------------------------------------------------- */
/*          ฟังก์ชันดูเครื่องหมาย >0 ให้ค่าเป็น 1 , <0 ให้ค่าเป็น -1                       */
/* -------------------------------------------------------------------------- */

template <typename type>
type sign(type value) {
    return type((value>0)-(value<0));
}

class MPID{
  public:

    float kP,kI,kD;
    float error;
    float currentError;
    float lastError;
    float throttle;
    float p_out;
    float i_out;
    float d_out;
    float pid_out;
    float pwm_out;
    unsigned long lastTime;
    unsigned long currentTime;
    
    bool reverse;

    float deltaT;

    int min_pwm;


  MPID(float kp,float ki,float kd,bool revs){

    kP=kp;
    kI=ki;
    kD=kd;

    
    
    error=0;
    currentError=0;
    lastError=0;
    reverse=revs;
    deltaT=0.1;//TODO
    
    min_pwm = 70;
  }

  inline void init() {
    lastTime=millis();
  }

  inline float predictPWM(const float& wheeld) {
    if(wheeld >=0 ) return 13.4425703667*expf(0.1542795496*wheeld);
    else return -13.4425703667*expf(0.1542795496*fabs(wheeld));
    return ;
  }

  inline float getPWM(const float& wheeld, const float& cur_wheeld){
    currentTime = millis();
    deltaT=(currentTime-lastTime)/1000.0;

    error = wheeld-cur_wheeld;
    
    currentError += kI*error;
    if( fabs(wheeld) < 0.5 && fabs(error) < 0.1){
      currentError = 0;
    }
    throttle = predictPWM(wheeld) / 255.0;

    p_out   = kP*error;
    i_out   = constrain(currentError, -0.8, 0.8);
    d_out   = constrain(kD*(error-lastError), -0.8,0.8);
    pid_out = constrain(throttle + p_out + i_out + d_out,-1.0,1.0);
    pwm_out = pid_out*255.0;
    if( fabs(sign(wheeld) - sign(pwm_out)) > 1 ) pwm_out = 0;

    lastError=error;
    return pwm_out;
  }

  inline float getPIDOUT() {
    return pid_out;
  }
  inline float getPOUT() {
    return p_out;
  }
  inline float getIOUT() {
    return i_out;
  }
  inline float getDOUT() {
    return d_out;
  }
  inline float getPWMOUT() {
    return pwm_out;
  }
};



class MPID2{
  public:

    float kP,kI;
    float error;
    float currentError;
    float lastError;
    float p_out;
    float i_out;
    float Ui;
    float U0;
    float pwm_out;
    unsigned long lastTime;
    unsigned long currentTime;
    
    float deltaT;


  MPID2(float kp,float ki){

    kP=kp;
    kI=ki;

      
    Ui=0;
    U0=0;

    error=0;
    currentError=0;
    lastError=0;
    deltaT=0.1;//TODO
    
  }

  inline void init() {
    lastTime=millis();
  }

  inline float getPWM(const float& wheeld, const float& cur_wheeld){
    currentTime = millis();
    deltaT=(currentTime-lastTime)/1000.0;

    error=wheeld-cur_wheeld;
    


    if ( fabs(error) < 0.05 ) {
      error = 0;
    }
    else {
      error = error;
    }
    float d_error = error - lastError;
    lastError = error;
    Ui = U0 + (kP * d_error) + (kI * error);
    U0 = Ui;
    pwm_out = Ui;
    pwm_out = constrain(pwm_out, -255, 255);

    



    return pwm_out;
  }

  inline float getPIDOUT() {
    return U0;
  }
  inline float getPOUT() {
    return p_out;
  }
  inline float getIOUT() {
    return i_out;
  }
  inline float getPWMOUT() {
    return pwm_out;
  }
};







class HPID{
  public:

    float kP,kI,kD;
    float error;
    float currentError;
    float lastError;
    float p_out;
    float i_out;
    float d_out;
    float pid_out;
    float yawrate_out;
    unsigned long lastTime;
    unsigned long currentTime;
    

    float deltaT;



  HPID(float kp,float ki,float kd){

    kP=kp;
    kI=ki;
    kD=kd;

    
    
    error=0;
    currentError=0;
    lastError=0;
    deltaT=0.1;//TODO
  }

  inline void init() {
    lastTime=millis();
    currentError=0;
  }
  

  inline float getYawrateD(const float& yawd, const float& cur_yaw){
    currentTime = millis();
    deltaT=(currentTime-lastTime)/1000.0;

    error= wrap_pi(yawd-cur_yaw);
    if( fabs(error) < 0.005) {
      currentError = 0;
      return 0;
    }
    
    currentError+=kI*error;

    p_out = kP*error;
    i_out = constrain(currentError, -0.8, 0.8);
    d_out = constrain(kD*(error-lastError), -0.8,0.8);
    pid_out = constrain(p_out + i_out + d_out,-1.0,1.0);
    yawrate_out = pid_out*max_wspd;

    lastError=error;
    return yawrate_out;
  }

  inline float getPIDOUT() {
    return pid_out;
  }
  inline float getPOUT() {
    return p_out;
  }
  inline float getIOUT() {
    return i_out;
  }
  inline float getDOUT() {
    return d_out;
  }
  inline float getPWMOUT() {
    return yawrate_out;
  }
};
#endif