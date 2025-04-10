/*
This sketch combines magnetometer readings from an LIS3MDL and accelerometer
readings from an LSM6 to calculate a tilt-compensated magnetic heading. It
requires Pololu's LSM6 Arduino library to be installed:

https://github.com/pololu/lsm6-arduino

This program can be used with a board that includes both sensors, like the
Pololu MinIMU-9 v5 and AltIMU-10 v5, or with separate carrier boards for the two
sensors, both connected to the same I2C bus. If you are using separate boards,
make sure the axes are oriented the same way on both (i.e. the X, Y, and Z axes
of both boards should point in the same direction, and the surfaces of the
boards should be as close to parallel as possible).
*/

#include <Wire.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <Zumo32U4.h>

LIS3MDL mag;
LSM6 imu;

Zumo32U4OLED display;
Zumo32U4Buzzer buzzer;

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

#define SPEED           100 // Maximum motor speed when going straight; variable speed when turning
#define TURN_BASE_SPEED 100 // Base speed when turning (added to variable speed)
int leftmotorSpeed;
int rightmotorSpeed;

// A couple of simple tunes, stored in program space.
const char beepBrownout[] PROGMEM = "<c8";
const char beepWelcome[] PROGMEM = ">g32>>c32";
const char beepThankYou[] PROGMEM = ">>c32>g32";
const char beepFail[] PROGMEM = "<g-8r8<g-8r8<g-8";
const char beepPass[] PROGMEM = ">l32c>e>g>>c8";


class PID {
public:
  PID(double kp, double ki, double kd) : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}
  
    double calculate(double setpoint, double measured_value) {
    double error = setpoint - measured_value;
    integral_ += error;
    double derivative = error - prev_error_;
    prev_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
  double kp_, ki_, kd_;
  double prev_error_, integral_;
};

//PID pid(1.0, 0.1, 0.01); // Example PID constants
//A rule-of-thumb process:
//Set I and D to 0. Increase P until you get oscillation off the system. Then cut P in half
//increase I until you are returning to the set-point quick enough for your application. Too much integral gain can cause a lot of overshoot.
//increase D until you’ve started to dampen your response as you approach the set poin
//PID pid(2.3, 0.01, 0.005);
PID pid(2.3, 0.00, 0.000);
double setpoint = 180.0; // Desired heading direction in degrees

/*
Calibration values; the default values of +/-32767 for each axis
lead to an assumed magnetometer bias of 0. Use the Calibrate example
program to determine appropriate values for your particular unit.
*/
LIS3MDL::vector<int16_t> m_min = {-12396, -8924, +26085};
LIS3MDL::vector<int16_t> m_max = {-5335, -4409, +32767};

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  display.setLayout21x8();

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize LIS3MDL magnetometer!");
    while (1);
  }
  mag.enableDefault();

  if (!imu.init())
  {
    Serial.println("Failed to detect and initialize LSM6 IMU!");
    while (1);
  }
  imu.enableDefault();
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  buzzer.playFromProgramSpace(beepWelcome);
  buttonA.waitForButton();
  //Serial.println("Waiting");
  //buttonA.waitForButton();
  Serial.println("Starting");
   buzzer.playFromProgramSpace(beepThankYou);
}

void loop()
{
 

  
  mag.read();
  imu.read();

  /*
  When given no arguments, the heading() function returns the angular
  difference in the horizontal plane between a default vector (the
  +X axis) and north, in degrees.
  */
  float current_heading = computeHeading();
  // Go to the next line
  display.clear();// Print a string
  display.print("Compass");
  display.gotoXY(0, 1);
  display.print(current_heading);

  double control_signal = pid.calculate(setpoint, current_heading);
  // Convert control signal to motor speed and direction
  int motorSpeed = abs(control_signal);
 
  if (motorSpeed > 255) motorSpeed = 255; // Limit motor speed to 255
  
  display.gotoXY(0, 2);
  display.print("Control Signal");
  display.gotoXY(0, 3);
  display.print(control_signal);

  if (control_signal > 0)
  {
      // curve right
      leftmotorSpeed = map(motorSpeed,-360,+360,0,255);
      rightmotorSpeed = map(-motorSpeed,-360,+360,0,255);
      motors.setLeftSpeed(leftmotorSpeed);
      motors.setRightSpeed(rightmotorSpeed);
    }
    else 
    {
      //curve left
      leftmotorSpeed = map(-motorSpeed,-360,+360,0,255);
      rightmotorSpeed = map(motorSpeed,-360,+360,0,255);
      motors.setLeftSpeed(leftmotorSpeed);
      motors.setRightSpeed(rightmotorSpeed);
    }

  Serial.print(current_heading);
  Serial.print(":");
  Serial.println(control_signal);
  //delay(50);
}

/*
Returns the angular difference in the horizontal plane between the
"from" vector and north, in degrees.

Description of heading algorithm:
Shift and scale the magnetic reading based on calibration data to find
the North vector. Use the acceleration readings to determine the Up
vector (gravity is measured as an upward acceleration). The cross
product of North and Up vectors is East. The vectors East and North
form a basis for the horizontal plane. The From vector is projected
into the horizontal plane and the angle between the projected vector
and horizontal north is returned.
*/
template <typename T> float computeHeading(LIS3MDL::vector<T> from)
{
  LIS3MDL::vector<int32_t> temp_m = {mag.m.x, mag.m.y, mag.m.z};

  // copy acceleration readings from LSM6::vector into an LIS3MDL::vector
  LIS3MDL::vector<int16_t> a = {imu.a.x, imu.a.y, imu.a.z};

  // subtract offset (average of min and max) from magnetometer readings
  temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
  temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
  temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

  // compute E and N
  LIS3MDL::vector<float> E;
  LIS3MDL::vector<float> N;
  LIS3MDL::vector_cross(&temp_m, &a, &E);
  LIS3MDL::vector_normalize(&E);
  LIS3MDL::vector_cross(&a, &E, &N);
  LIS3MDL::vector_normalize(&N);

  // compute heading
  float heading = atan2(LIS3MDL::vector_dot(&E, &from), LIS3MDL::vector_dot(&N, &from)) * 180 / PI;
  if (heading < 0) heading += 360;
  return heading;
}

/*
Returns the angular difference in the horizontal plane between a
default vector (the +X axis) and north, in degrees.
*/
float computeHeading()
{
  return computeHeading((LIS3MDL::vector<int>){1, 0, 0});
}
