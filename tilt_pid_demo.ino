#include <Servo.h> // library for micro servo
#include <NewPing.h> // library for the proximity sensor
#include <PID_v1.h> // library for calculating new input based on output and setpoint

int pos = 0;    // variable to store the servo position
int servo_pin = 9;
int trigger_pin = 8;
int echo_pin = 7;
int max_dist_cm = 25; // max distance that the proximity sensor should read out to

NewPing sonar(trigger_pin, echo_pin, max_dist_cm); // init an object for the proximity sensor (input)
Servo myservo;  // create servo object to control a servo (output)

double Setpoint, Input, Output; // define the vars to store params for the PID class
double Kp=0.1, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  myservo.attach(servo_pin); 
  Serial.begin(9600); // baud rate
  delay(50); // wait 50 ms
  
  
  myPID.SetMode(AUTOMATIC);
}


void loop() {
    int dist_cm = sonar.ping_cm();
    int servo_lower_lim_deg = 105;
    int servo_upper_lim_deg = 130;
    Setpoint = 10; // cm from ball to the proximity sensor on tilt
    // Serial.println(dist_cm);
    Input = dist_cm;
    myPID.Compute();
    int new_tilt_deg = map(Output, 0, 10, servo_lower_lim_deg, servo_upper_lim_deg);
    Serial.print("Pid Output: ");
    Serial.print(Output);
    Serial.print(", ");
    Serial.print("Distance to proximity sensor: ");
    Serial.print(dist_cm);
    Serial.print(", ");
    Serial.print("New tilt degrees: ");
    Serial.print(new_tilt_deg);
    Serial.print(", ");
    Serial.print("Input to Pid: ");
    Serial.println(Input);
//    myservo.write(new_tilt_deg);
    delay(1000); 
}


void tilt_ball() {
  int lower_lim_deg = 105;
  int upper_lim_deg = 130;
  
  int delay_time_ms = 40;
  
  for (pos = lower_lim_deg; pos <= upper_lim_deg; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    int dist_cm = sonar.ping_cm();
    Serial.println(dist_cm);
    delay(delay_time_ms);                       // waits 15 ms for the servo to reach the position
  }
  delay(2000);
  for (pos = upper_lim_deg; pos >= lower_lim_deg; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);  
    int dist_cm = sonar.ping_cm();
    Serial.println(dist_cm);
    delay(delay_time_ms);                       // waits 15 ms for the servo to reach the position
  }
  delay(2000);
}