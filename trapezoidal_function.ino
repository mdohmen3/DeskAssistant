#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

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
#define J1_min 108  // min pulselength
#define J1_max 460  // max pulselength
#define d1_min -4   // corresponding limit in degrees (min)
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
double tf = 0;
double tb1 = 0;
double tb2 = 0;
double tb3 = 0;

////// Reference Configuration of the robotic arm //////
double x_reference = 255;  // mm
double y_reference = 0;    // mm
double z_reference = 110;  // mm

////// Reference Configuration of the robotic arm in joint space //////
double t1_reference = 0;  // degrees
double t2_reference = 0;  // degrees
double t3_reference = 0;  // degrees

// variables used to calculate the inverse kinematics
double t1 = 0;
int d1 = 0;

double beta = 0;
double t2 = 0;
int d2 = 0;

double c3 = 0;
double s3 = 0;
double t3 = 0;
int d3 = 0;

int grip = 200;

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // Move the robotic arm to the reference configuration (starting position)
  pwm.setPWM(0, 0, map(t1_reference, d0_min, d0_max, J0_min, J0_max));
  pwm.setPWM(1, 0, map(t2_reference, d1_min, d1_max, J1_min, J1_max));
  pwm.setPWM(2, 0, map(t3_reference, d2_min, d2_max, J2_min, J2_max));
  pwm.setPWM(3, 0, grip);
}

void loop() {


  // send data to start or end the trajectory
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    Serial.println(incomingByte);

    // print the current x, y, z value
    switch (incomingByte) {
      case 's':  // start trajectory
        //Definition of final angles and maximum angular velocity
        t1_final = 45;
        t2_final = 60;
        t3_final = 60;
        t_velocity_max = 15;
        trapezoidal_interp(); //Run the trapezoidal interpolation function defined below
        break;
      case 'e':  // end trajectory
        run_bool = false;
        break;

      case 'h':  // move the robot to the reference configuration
        t1_final = 0;
        t2_final = 0;
        t3_final = 0;
        t_velocity_max = 30;
        trapezoidal_interp();
        break;
    }
  }
}

int trapezoidal_interp() {  //Trapezoidal interpolation function
  //Calculation of total time, based on the maximum change in angle
  tf = max((max(abs(t1_final - t1_init), abs(t2_final - t2_init))), abs(t3_final - t3_init)) / t_velocity_max + 0.25;
  //Calculation of the blend times for each joint
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

    if (tb1 >= 0.5 * tf) {                                           //Check for collapsed trapezoidal interpolation
      t_collapsed_max = 2 * (t1_final - t1_init) / tf;               //Redefine maximum angular velocity
      if (current_time < 0.5 * tf) {                                 //First half of collapsed trapezoidal interpolation
        d1 = t_collapsed_max / tf * pow(current_time, 2) + t1_init;  //Updated angle
        d1dot = 2 * t_collapsed_max / tf * current_time;             //angular velocity
        d1double = 2 * t_collapsed_max / tf;                         //angular acceleration
      }
      //Second half
      if ((current_time >= 0.5 * tf) && (current_time <= tf)) {
        d1 = t_collapsed_max * (-1 / tf * pow(current_time, 2) + 2 * current_time - tf) + t1_final;  //Angle
        d1dot = t_collapsed_max * (-2 / tf * current_time + 2);                                      //Angular Velocity
        d1double = -2 * t_collapsed_max / tf;                                                        //Angular Acceleration
      }
      //If Collapsed trapezoidal interpolation is not required:
    } else {
      //Check for positive or negative velocity
      if ((t1_final - t1_init) < 0) {  //Define the maximum velocity as either positive or negative
        t_velocity_max = -abs(t_velocity_max);
      } else {
        t_velocity_max = abs(t_velocity_max);
      }

      /////Trapezoidal Interpolation//////
      if (current_time <= tb1) {                                           //Time before blend time
        d1 = t1_init + 0.5 * t_velocity_max / tb1 * pow(current_time, 2);  //Angle
        d1dot = t_velocity_max / tb1 * current_time;                       //Angular Velocity
        d1double = t_velocity_max / tb1;                                   //Angular Acceleration
      }
      if ((current_time > tb1) && (current_time <= tf - tb1)) {  //Time between blend time and (total time - blend time)
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
    //Code for thetas 2 and 3 are identical to theta 1, but with corresponding angles and blend times

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

    //set the servos to the correct angles
    pwm.setPWM(0, 0, map(d1, d0_min, d0_max, J0_min, J0_max));
    pwm.setPWM(1, 0, map(d2, d1_min, d1_max, J1_min, J1_max));
    pwm.setPWM(2, 0, map(d3, d2_min, d2_max, J2_min, J2_max));
  }
  //Change the initial thetas to the current position so the code calculates values correctly the next time it runs
  t1_init = d1;
  t2_init = d2;
  t3_init = d3;
}