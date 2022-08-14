/*
Goal: create a simple PID controller using Arduino to manipulate the tilt to keep a ball around a set-point value.
Also provide features to control the speed with which the servo changes angles to not damage parts.
Date: Aug 14, 2022
Author: Vincent Stevenson
*/

#include <Servo.h> // library for micro servo
#include <NewPing.h> // library for the proximity sensor

int servo_pin = 9;
int trigger_pin = 8;
int echo_pin = 7;
int max_dist_cm = 25; // max distance that the proximity sensor should read out to (length of the tilt)

double servo_lower_lim_deg = 80;
double servo_upper_lim_deg = 120;
double old_tilt_deg = servo_lower_lim_deg; // initialize the previous tilt degree
double cumulative_error = 0; // units are cm, this is needed for I-term in PID 
double previous_error = 0; // this is for D term in PID

double Kp = 1.5;
double Ki = 0.05;
double Kd = 0.1;

NewPing sonar(trigger_pin, echo_pin, max_dist_cm); // init an object for the proximity sensor (input)
Servo myservo;  // create servo object to control a servo (output)
void setup() {
  myservo.attach(servo_pin); 
  Serial.begin(9600); // baud rate
  delay(50); 
  
  myservo.write(servo_lower_lim_deg);
}


void loop() {
  
  double dist_cm = sonar.ping_cm();
  double delay_time_ms = 5;
  if (dist_cm > 0) {
    // the prox sensor sometimes throws random 0's that will mess us the controller
    // the if statement lets us ignore those
    double new_tilt_deg = pid(dist_cm);
    update_servo(new_tilt_deg, old_tilt_deg, delay_time_ms);
    old_tilt_deg = new_tilt_deg;
  } 
}

void update_servo(double new_tilt_deg, double old_tilt_deg, double delay_time_ms) {
  // delay time is how to control the speed the servo will change angles
  // I added this because the servo is moving too fast for the hot glue and paperclips
 
  // make the servo change from old to new degrees in increments, not all at once
  if (new_tilt_deg > old_tilt_deg) {
    for (double pos = old_tilt_deg; pos <= new_tilt_deg; pos += 1) {
      // in steps of 1 degree
      myservo.write(pos);             
      delay(delay_time_ms);                       
    }
  }

  if (new_tilt_deg < old_tilt_deg) {
    for (double pos = old_tilt_deg; pos >= new_tilt_deg; pos -= 1) {
      // in steps of 1 degree
      myservo.write(pos);             
      delay(delay_time_ms);                       
    }
  }
}


double pid(double distance_cm) {
  // input == distance from proximity sensor to ball
  // output == new angle to move the servo motor to get ball closer to setpoint
  // never let the ball get closer than 4 cm to the proximity sensor - else it isn't accurate

  double setpoint_cm = 10;
  Serial.print("setpoint_cm:");
  Serial.print(setpoint_cm);
  Serial.print(","); 

  double error = setpoint_cm - distance_cm;
  Serial.print("error_cm:");
  Serial.print(error);
  Serial.print(","); 
  
  double p_value = error * Kp;

  double i_value = cumulative_error * Ki;

  double d_value = (error - previous_error) * Kd;
  // typically we would divide by the elapsed time as the D-term is checking the error rate
  // so if the ball is moving really fast the wrong way (away from setpoint), the correction will be bigger than P-only
  
  double pid_value = p_value + i_value + d_value;
  
  Serial.print("pid_value:");
  Serial.println(pid_value);

  cumulative_error += error; // note that error can be + or -, this i term seeks to eliminate the offset when the P-only controller stalls
  previous_error = error; // for the next cycle, remember what this cycle's error was
  // map the pid value to a new angle for the servo to go to
  double servo_range = servo_upper_lim_deg - servo_lower_lim_deg;
  double new_servo_angle = map(pid_value, -1 * servo_range, servo_range, servo_lower_lim_deg, servo_upper_lim_deg);

  // make sure that any curveballs don't make the servo try to go to an angle outside of its operating range
  if (new_servo_angle > servo_upper_lim_deg) {
    new_servo_angle = servo_upper_lim_deg;
  }
  
  if (new_servo_angle < servo_lower_lim_deg) {
    new_servo_angle = servo_lower_lim_deg;
  }
  
  return new_servo_angle;
}
