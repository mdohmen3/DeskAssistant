#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Pixy2.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

// Robotic arm parameters.
// These will need to be modifed based on your arm
////// Theta 1 //////
#define J0_min 92   // min pulselength
#define J0_max 500  // max pulselength
#define d0_min -90  // corresponding limit in degrees (min)
#define d0_max 90   // max degrees

////// Theta 2 //////
#define J1_min 112  // min pulselength
#define J1_max 314  // max pulselength
#define d1_min -4   // corresponding limit in degrees (min)
#define d1_max 90   // max degrees

////// Theta 3 //////
#define J2_min 236  // min pulselength
#define J2_max 532  // max pulselength
#define d2_min -27  // corresponding limit in degrees (min)
#define d2_max 108  // max degrees

////// End Effector //////
#define J3_min 140  // pulselength in open position
#define J3_max 284  // pulselength in closed position
#define d3_min 60   // corresponding min distance in mm
#define d3_max 80   // max distance in mm

char incomingByte = 0;  // for incoming serial data
int i = 0;
int global_time = 0;


// current position of the robitc arm
double x = 255;  // mm
double y = 0;    // mm
double z = 110;  //mm

double D1 = 110;
double L1 = 105;
double L3 = 150;

double start_time = 0;
double current_time = 0;
bool run_bool = false;
bool set_start_time = true;

////// Trajectory generation vars //////
double t1_init = 0;
double t2_init = 0;
double t3_init = 0;
double t1_final = 0;
double t2_final = 0;
double t3_final = 0;
double t_velocity_max = 0;
double t_collapsed_max = 0;
double tf = 0;
double tb1 = 0;
double tb2 = 0;
double tb3 = 0;

////// Reference Configuration of the robotic arm //////
double x_reference = 255;  // mm
double y_reference = 0;    // mm
double z_reference = 110;  // mm

////// Reference Configuration of the robotic arm in joint space //////
double t1_reference = 45;  // degrees
double t2_reference = 90;  // degrees
double t3_reference = 0;   // degrees

// variables used to calculate the inverse kinematics
double t1 = 0;
int d1 = 0;
double d1dot = 0;
double d1double = 0;
double l3Var = 0;
double r = 0;

double beta = 0;
double t2 = 0;
int d2 = 0;
double d2dot = 0;
double d2double = 0;

double c3 = 0;
double s3 = 0;
double t3 = 0;
int d3 = 0;
double d3dot = 0;
double d3double = 0;

int gripPL = 0;

int grip = 5;  //Grip from 1-10 (10 being most closed)

Pixy2 pixy;

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  Serial.print(t1_reference);

  x = 102;
  y = -102;
  z = 260;
  grip = 10;
  t_velocity_max = 30;
  trapezoidal_interp();

  pixy.init();

  delay(2000);
}

void loop() {

  pixy.ccc.getBlocks();  //Looks for item

  ///// While the pixy cam sees the pattern, do nothing /////
  while (pixy.ccc.numBlocks) {
    pixy.ccc.getBlocks();
  }


  ///// After the pattern is covered, do the following items /////

  ///// Define a desired x, y, z, grip value, max velocity, and run trapezoidal_interp();

  x = 20;
  y = 152;
  z = 100;
  grip = 10;
  t_velocity_max = 45;
  trapezoidal_interp();

  pixy.ccc.getBlocks();  //Let the actions complete before checking the pixy cam again

  ///// While the pixy cam doesn't see the pattern, do nothing /////

  while (!(pixy.ccc.numBlocks)) {
    pixy.ccc.getBlocks();
  }

  ///// After the pattern is revealed, do the following items /////

  ///// Define the desired x, y, z, grip, t_velocity_max, and then run trapezoidal_interp();

  //Default Position

  x = 102;
  y = -102;
  z = 260;
  grip = 10;
  t_velocity_max = 45;
  trapezoidal_interp();

  delay(2000);  //Wait 2 seconds before looking for the pattern again
}

//The following function combines the trapezoidal interpolation code with the inverse kinematics.
int trapezoidal_interp() {

  /////////////// Compute the inverse kinematics of the robotic arm ///////////////

  l3Var = map(grip, 1, 10, d3_min, d3_max);

  r = sqrt(pow(x, 2) + pow(y, 2) + pow((z - 110), 2));

  ////// inverse kinematics (joint 1) //////

  t1 = atan2(y, x);               // [radians]
  t1_final = t1 * 180 / 3.14159;  // [degrees]

  ////// inverse kinematics (joint 2) //////
  t2 = atan2((z - 110), sqrt(pow(x, 2) + pow(y, 2))) + acos((pow((70 + l3Var), 2) - pow(r, 2) - pow(105, 2)) / ((-2) * r * 105));  // [radians]
  t2_final = t2 * 180 / 3.14159;                                                                                                   // [degrees]

  ////// inverse kinematics (joint 3) //////
  t3 = 3.14159265359 - acos((-pow((70 + l3Var), 2) + pow(r, 2) - pow(105, 2)) / (-2 * (70 + l3Var) * 105));  // [radians]
  t3_final = t3 * 180 / 3.14159;                                                                             // [degrees]

  gripPL = map(grip, 1, 10, J3_min, J3_max);
  //Serial.println(gripPL);

  Serial.print(r);
  Serial.print(", ");
  Serial.print(t1);
  Serial.print(", ");
  Serial.print(t2);
  Serial.print(", ");
  Serial.print(t3);
  Serial.println(", ");
  //End of inverse kinematics//

  pwm.setPWM(3, 0, gripPL);

  tf = max((max(abs(t1_final - t1_init), abs(t2_final - t2_init))), abs(t3_final - t3_init)) / t_velocity_max + 0.25;

  tb1 = (-abs(t1_init - t1_final) + t_velocity_max * tf) / t_velocity_max;
  tb2 = (-abs(t2_init - t2_final) + t_velocity_max * tf) / t_velocity_max;
  tb3 = (-abs(t3_init - t3_final) + t_velocity_max * tf) / t_velocity_max;

  set_start_time = true;
  current_time = 0;

  while (current_time <= tf) {
    if (set_start_time == true) {
      start_time = millis();
      set_start_time = false;
    }

    current_time = (millis() - start_time) / 1000;

    //Serial.print("Time = ");
    //Serial.print(current_time);

    ///// Theta 1 /////

    if (tb1 >= 0.5 * tf) {
      t_collapsed_max = 2 * (t1_final - t1_init) / tf;
      if (current_time < 0.5 * tf) {
        d1 = t_collapsed_max / tf * pow(current_time, 2) + t1_init;
        d1dot = 2 * t_collapsed_max / tf * current_time;
        d1double = 2 * t_collapsed_max / tf;
      }
      if ((current_time >= 0.5 * tf) && (current_time <= tf)) {
        d1 = t_collapsed_max * (-1 / tf * pow(current_time, 2) + 2 * current_time - tf) + t1_final;
        d1dot = t_collapsed_max * (-2 / tf * current_time + 2);
        d1double = -2 * t_collapsed_max / tf;
      }
    } else {
      //Check for positive or negative velocity
      if ((t1_final - t1_init) < 0) {
        t_velocity_max = -abs(t_velocity_max);
      } else {
        t_velocity_max = abs(t_velocity_max);
      }
      //Trapezoidal Interpolation
      if (current_time <= tb1) {
        d1 = t1_init + 0.5 * t_velocity_max / tb1 * pow(current_time, 2);
        d1dot = t_velocity_max / tb1 * current_time;
        d1double = t_velocity_max / tb1;
      }
      if ((current_time > tb1) && (current_time <= tf - tb1)) {
        d1 = t1_init - 0.5 * tb1 * t_velocity_max + t_velocity_max * current_time;
        d1dot = t_velocity_max;
        d1double = 0;
      }
      if ((current_time > tf - tb1) && (current_time <= tf)) {
        d1 = t1_final - 0.5 * t_velocity_max / tb1 * pow((current_time - tf), 2);
        d1dot = -t_velocity_max / tb1 * (current_time - tf);
        d1double = -t_velocity_max / tb1;
      }
    }


    ///// Theta 2 //////


    if (tb2 >= 0.5 * tf) {
      t_collapsed_max = 2 * (t2_final - t2_init) / tf;
      if (current_time < 0.5 * tf) {
        d2 = t_collapsed_max / tf * pow(current_time, 2) + t2_init;
        d2dot = 2 * t_collapsed_max / tf * current_time;
        d2double = 2 * t_collapsed_max / tf;
      }
      if ((current_time >= 0.5 * tf) && (current_time <= tf)) {
        d2 = t_collapsed_max * (-1 / tf * pow(current_time, 2) + 2 * current_time - tf) + t2_final;
        d2dot = t_collapsed_max * (-2 / tf * current_time + 2);
        d2double = -2 * t_collapsed_max / tf;
      }
    } else {
      if ((t2_final - t2_init) < 0) {
        t_velocity_max = -abs(t_velocity_max);
      } else {
        t_velocity_max = abs(t_velocity_max);
      }


      if (current_time <= tb2) {
        d2 = t2_init + 0.5 * t_velocity_max / tb2 * pow(current_time, 2);
        d2dot = t_velocity_max / tb2 * current_time;
        d2double = t_velocity_max / tb2;
      }
      if ((current_time > tb2) && (current_time <= tf - tb2)) {
        d2 = t2_init - 0.5 * tb2 * t_velocity_max + t_velocity_max * current_time;
        d2dot = t_velocity_max;
        d2double = 0;
      }
      if ((current_time > tf - tb2) && (current_time <= tf)) {
        d2 = t2_final - 0.5 * t_velocity_max / tb2 * pow((current_time - tf), 2);
        d2dot = -t_velocity_max / tb2 * (current_time - tf);
        d2double = -t_velocity_max / tb2;
      }
    }

    if (tb3 >= 0.5 * tf) {
      t_collapsed_max = 2 * (t3_final - t3_init) / tf;
      if (current_time < 0.5 * tf) {
        d3 = t_collapsed_max / tf * pow(current_time, 2) + t3_init;
        d3dot = 2 * t_collapsed_max / tf * current_time;
        d3double = 2 * t_collapsed_max / tf;
      }
      if ((current_time >= 0.5 * tf) && (current_time <= tf)) {
        d3 = t_collapsed_max * (-1 / tf * pow(current_time, 2) + 2 * current_time - tf) + t3_final;
        d3dot = t_collapsed_max * (-2 / tf * current_time + 2);
        d3double = -2 * t_collapsed_max / tf;
      }
    } else {
      if ((t3_final - t3_init) < 0) {
        t_velocity_max = -abs(t_velocity_max);
      } else {
        t_velocity_max = abs(t_velocity_max);
      }

      if (current_time <= tb3) {
        d3 = t3_init + 0.5 * t_velocity_max / tb3 * pow(current_time, 2);
        d3dot = t_velocity_max / tb3 * current_time;
        d3double = t_velocity_max / tb3;
      }
      if ((current_time > tb3) && (current_time <= tf - tb3)) {
        d3 = t3_init - 0.5 * tb3 * t_velocity_max + t_velocity_max * current_time;
        d3dot = t_velocity_max;
        d3double = 0;
      }
      if ((current_time > tf - tb3) && (current_time <= tf)) {
        d3 = t3_final - 0.5 * t_velocity_max / tb3 * pow((current_time - tf), 2);
        d3dot = -t_velocity_max / tb3 * (current_time - tf);
        d3double = -t_velocity_max / tb3;
      }
    }

    pwm.setPWM(0, 0, map(d1, d0_min, d0_max, J0_min, J0_max));
    pwm.setPWM(1, 0, map(d2, d1_min, d1_max, J1_min, J1_max));
    pwm.setPWM(2, 0, map(d3, d2_min, d2_max, J2_min, J2_max));
  }
  t1_init = d1;
  t2_init = d2;
  t3_init = d3;
}
