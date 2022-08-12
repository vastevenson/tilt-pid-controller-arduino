#include <Servo.h> // library for micro servo
#include <NewPing.h> // library for the proximity sensor

int servo_pin = 9;
int trigger_pin = 8;
int echo_pin = 7;
int max_dist_cm = 25; // max distance that the proximity sensor should read out to (length of the tilt)

NewPing sonar(trigger_pin, echo_pin, max_dist_cm); // init an object for the proximity sensor (input)
Servo myservo;  // create servo object to control a servo (output)

void setup() {
	myservo.attach(servo_pin); 
	Serial.begin(9600); // baud rate
	delay(50); 
	double cumulative_error = 0; // units are cm, this is needed for I-term in PID 
}


void loop() {
    double dist_cm = sonar.ping_cm();
	if (dist_cm > 0) {
		// the prox sensor sometimes throws random 0's that will mess us the controller
		// the if statement lets us ignore those
		double new_tilt_deg = pid(dist_cm);
		myservo.write(new_tilt_deg);
	}
	
    delay(1000); 
}


double pid(double distance_cm) {
	// input == distance from proximity sensor to ball
	// output == new angle to move the servo motor to get ball closer to setpoint
	
	double Kp = 0.1;
	double Ki = 0;
	
	// never let the ball get closer than 4 cm to the proximity sensor - else it isn't accurate
	
	
	double setpoint_cm = 10;
	
	double servo_lower_lim_deg = 105;
    double servo_upper_lim_deg = 130;
	
	double error = setpoint_cm - distance_cm;
	Serial.print("error: ");
	Serial.print(error);
	
	double min_distance_limit_cm = 4; // distance from ball to prox sensor
	double max_distance_limit_cm = 20; 
	
	// these limits are for P-only!
	double min_pid_val = (setpoint_cm - max_distance_limit_cm) * Kp; // what is the largest error possible * Kp 
	// smallest_possible_error == ball is as far from proximity sensor as possible - reading 25 cm
	// error == 10 - 18 = -8 cm
	// largest_possible_error == ball is as close to proximity sensor as possible - reading 4 cm
	// error == 10 - 4 = 6 cm
	double max_pid_val = (setpoint_cm - min_distance_limit_cm) * Kp; // what is the smallest/most negative error possible * Kp
	
	double p_value = error * Kp;
	Serial.print("p_value: ");
	Serial.print(p_value);
	
	// double i_value = cumulative_error * Ki;
	// cumulative_error += error;
	
	double pid_value = p_value + i_value;
	Serial.print("pid_value: ");
	Serial.print(pid_value);
	
	// map the pid value to a new angle for the servo to go to
	double new_servo_angle = map(pid_value, min_pid_val, max_pid_val, servo_lower_lim_deg, servo_upper_lim_deg)
	Serial.print("new_servo_angle: ");
	Serial.print(new_servo_angle);
	
	// make sure that any curveballs don't make the servo try to go to an angle outside of its operating range
	if (new_servo_angle > servo_upper_lim_deg) {
		new_servo_angle = servo_upper_lim_deg
	}
	
	if (new_servo_angle < servo_lower_lim_deg) {
		new_servo_angle = servo_lower_lim_deg
	}
	
	return new_servo_angle;
}