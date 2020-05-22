
// NOTE: requires the Encoder library.
// 1) open Tools -> Manage Libraries...
// 2) install "Encoder" by Paul Stoffregen v1.4.1
#include <Encoder.h>



// --- SPD Motor ---

class SPDMotor {
  public:
    SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );

    /// Set the PWM speed and direction pins.
    /// pwm = 0, stop (no active control)
    /// pwm = 1 to 255, proportion of CCW rotation
    /// pwm = -1 to -255, proportion of CW rotation
    void speed( int pwm );

    /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
    void hardStop();

    /// Get the current speed.
    int getSpeed();

    /// Get the current rotation position from the encoder.
    long getEncoderPosition();

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;

    // Current speed setting.
    int _speed;
};

SPDMotor::SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 ) {
  _encoder = new Encoder(encoderA, encoderB);
  _encoderReversed = encoderReversed;

  _motorPWM = motorPWM;
  pinMode( _motorPWM, OUTPUT );
  _motorDir1 = motorDir1;
  pinMode( _motorDir1, OUTPUT );
  _motorDir2 = motorDir2;
  pinMode( _motorDir2, OUTPUT );
}

/// Set the PWM speed and direction pins.
/// pwm = 0, stop (no active control)
/// pwm = 1 to 255, proportion of CCW rotation
/// pwm = -1 to -255, proportion of CW rotation
void SPDMotor::speed( int speedPWM ) {
  _speed = speedPWM;
  if ( speedPWM == 0 ) {
    digitalWrite(_motorDir1, LOW);
    digitalWrite(_motorDir2, LOW);
    analogWrite( _motorPWM, 255);
  } else if ( speedPWM > 0 ) {
    digitalWrite(_motorDir1, LOW );
    digitalWrite(_motorDir2, HIGH );
    analogWrite( _motorPWM, speedPWM < 255 ? speedPWM : 255);
  } else if ( speedPWM < 0 ) {
    digitalWrite(_motorDir1, HIGH );
    digitalWrite(_motorDir2, LOW );
    analogWrite( _motorPWM, (-speedPWM) < 255 ? (-speedPWM) : 255);
  }
}

/// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
void SPDMotor::hardStop() {
  _speed = 0;
  digitalWrite(_motorDir1, HIGH);
  digitalWrite(_motorDir2, HIGH);
  analogWrite( _motorPWM, 0);
}

/// Get the current speed.
int SPDMotor::getSpeed() {
  return _speed;
}

/// Get the current rotation position from the encoder.
long SPDMotor::getEncoderPosition() {
  long position = _encoder->read();
  return _encoderReversed ? -position : position;
}


SPDMotor *motorLF = new SPDMotor(18, 31, false, 12, 34, 35); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 36, 37); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor( 3, 49, false,  9, 43, 42); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 2, A1, true, 5, A5, A4); // <- NOTE: Motor Dir pins reversed for opposite operation
