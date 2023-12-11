#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

// Robotic arm parameters.
// These will need to be modified based on your arm
////// Theta 1 //////
#define J0_min 92   // min pulselength
#define J0_max 500  // max pulselength
#define d0_min -90  // corresponding limit in degrees (min)
#define d0_max 90   // max degrees

////// Theta 2 //////
#define J1_min 108  // min pulselength
#define J1_max 460  // max pulselength
#define d1_min -1   // corresponding limit in degrees (min)
#define d1_max 160  // max degrees

////// Theta 3 //////
#define J2_min 72   // min pulselength
#define J2_max 528  // max pulselength
#define d2_min -93  // corresponding limit in degrees (min)
#define d2_max 93   // max degrees

////// End Effector //////
#define J3_min 176  // pulselength in open position
#define J3_max 312  // pulselength in closed position
#define d3_min 60   // corresponding min distance in mm
#define d3_max 80   // max distance in mm


char incomingByte = 0;  // for incoming serial data

// Reference configuration of the robotic arm
double x = 255;  // mm
double y = 0;    // mm
double z = 110;  //mm

// variables used to calculate the inverse kinematics
double r = 0;

double t1 = 0;
int d1 = 0;

double t2 = 0;
int d2 = 0;

double t3 = 0;
int d3 = 0;

int grip = 200;

int l3Var = 0;

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // Move the robotic arm to the reference configuration
  pwm.setPWM(0, 0, map(0, d0_min, d0_max, J0_min, J0_max));
  pwm.setPWM(1, 0, J1_min);
  pwm.setPWM(2, 0, J2_min);
  pwm.setPWM(3, 0, grip);
}

void loop() {

  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.println(incomingByte);

    switch (incomingByte) {
      case 'q':  // move in x
        x += 5;
        break;
      case 'a':
        x -= 5;
        break;

      case 'w':  // move in y
        y += 5;
        break;
      case 's':
        y -= 5;
        break;

      case 'e':  // move in z
        z += 5;
        break;
      case 'd':
        z -= 5;
        break;

      case 'r':  // open/close the end effector
        grip += 5;
        break;
      case 'f':
        grip -= 5;
        break;

      case 'h':  // move the robot to the reference configuration
        x = 235;
        y = 0;
        z = 110;
        grip = 200;
        break;
    }

    /////////////// Compute the inverse kinematics of the robotic arm ///////////////

    l3Var = map(grip, J3_min, J3_max, d3_min, d3_max);  //variable length of l3 as defined by the status of the end effector

    r = sqrt(pow(x, 2) + pow(y, 2) + pow((z - 110), 2));  //Distance between the end of link 1 and the end effector

    ////// inverse kinematics (joint 1) //////

    t1 = atan2(y, x);         // [radians]
    d1 = t1 * 180 / 3.14159;  // [degrees]
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(0, 0, map(d1, d0_min, d0_max, J0_min, J0_max));

    ////// inverse kinematics (joint 2) //////
    t2 = atan2((z - 110), sqrt(pow(x, 2) + pow(y, 2))) + acos((pow((70 + l3Var), 2) - pow(r, 2) - pow(105, 2)) / ((-2) * r * 105));  // [radians]
    d2 = t2 * 180 / 3.14159;                                                                                                         // [degrees]
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(1, 0, map(d2, d1_min, d1_max, J1_min, J1_max));

    ////// inverse kinematics (joint 3) //////
    t3 = 3.14159 - acos((-pow((70 + l3Var), 2) + pow(r, 2) - pow(105, 2)) / (-2 * (70 + l3Var) * 104));  // [radians]
    d3 = t3 * 180 / 3.14159;                                                                             // [degrees]
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(2, 0, map(d3, d2_min, d2_max, J2_min, J2_max));

    /////// inverse kinematics (joint 3) //////
    // send pulselength value to the end effector
    pwm.setPWM(3, 0, grip);
  }
}
