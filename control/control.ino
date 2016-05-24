/* TODO: Get a much more powerful battery. */
/* TODO: Get two optical mice, and use their internal sensors. Look into ADNS-2610. */
#include <Wire.h>
const int left_forward_pin=6;
const int left_backward_pin=5;
const int right_forward_pin=9;
const int right_backward_pin=10; // for the wheel driver unit
const int interrupt_pin=0, interrupt_pin2=1; //note: pins 0 and 1 are not used. Since this is an arduino uno it's actually pins 2 and 3. Arduino has a weird way of labelling interrupt pins
const int left_echo_pin=11, left_trig_pin=12; // for the left sonar unit
const int right_echo_pin=4, right_trig_pin=3; // for the right sonar unit (not currently used).
const double body_length=18.0; //cm
const double wheel_radius=3.5; //cm
const double max_speed=12.5; //rads / sec
const double pi=3.14159265358979;//32384626433832790528841971633993 <- adjust as needed.
const int ticks_per_revolution=38; //This isn't the real number of ticks. It's actually twice that amount since a wheel encoder will go from low to high and then from high to low for each 'tick'
const double tick_unit=2.0*pi*wheel_radius/double(ticks_per_revolution);
volatile int num_ticks_left=0, num_ticks_right=0;
volatile bool left_direction=true, right_direction=true;
const int min_pwm=0;
const int sensor_id=8, sensor_ready_pin=2;
const int sensor_reset=7, power_meter=A0;

#define DEBUG

double current_x();
double current_y();
double current_angle();

#ifdef DEBUG
#define println(X) Serial.println(X)
#define print(X) Serial.print(X)
#else
#define println(X) (void) 0
#define print(X) (void) 0
#endif

int state_i2c_dev(int id, char *msg)
{
	Wire.beginTransmission(id);
	Wire.write(msg);
	Wire.endTransmission();
}

void set_left_direction(bool dir)
{
	left_direction=dir;
	if (dir)
		state_i2c_dev(sensor_id, ".left=true");
	else
		state_i2c_dev(sensor_id, ".left=false");
}

void set_right_direction(bool dir)
{
	right_direction=dir;
	if (dir)
		state_i2c_dev(sensor_id, ".right=true");
	else
		state_i2c_dev(sensor_id, ".right=false");
}

#define DEBUG
void setup_pins()
{
	pinMode(left_echo_pin, INPUT);
	pinMode(left_trig_pin, OUTPUT);
	pinMode(right_backward_pin, OUTPUT);
	pinMode(right_forward_pin, OUTPUT);
	pinMode(left_forward_pin, OUTPUT);
	pinMode(left_backward_pin, OUTPUT);
	pinMode(sensor_ready_pin, INPUT);
	pinMode(sensor_reset, OUTPUT);
	pinMode(power_meter, INPUT);
}

void halt()
{
	analogWrite(left_forward_pin, 0);
	analogWrite(right_forward_pin, 0);
	analogWrite(left_backward_pin, 0);
	analogWrite(right_backward_pin, 0);
}

void setup()
{
	setup_pins();
	digitalWrite(sensor_reset, HIGH);
	delay(1000);
	digitalWrite(sensor_reset, LOW);
	delay(1000);
	digitalWrite(sensor_reset, HIGH);
	halt();
	Wire.begin();
	#ifdef DEBUG
	Serial.begin(9600);
	#endif
	println("Waiting for sensor subsystem to come on...");
	while (!digitalRead(sensor_ready_pin)) {
	}
	println("Done");
	String response=query_i2c_dev(sensor_id, "?ready");
	if (response==".ready\n") {
		println("subsystem ready");
	} else {
		println(response);
		panic();
	}
}




void control_wheels(double velocity, double turn_rate)
{
        double Vl=(2.0*velocity+turn_rate*body_length)/(2.0*wheel_radius);
        double Vr=(2.0*velocity-turn_rate*body_length)/(2.0*wheel_radius);

	if (Vl > max_speed)
		Vl=max_speed;
	else if (Vl <= -max_speed)
		Vl=-max_speed;

	if (Vr > max_speed)
		Vr=max_speed;
	else if (Vr <= -max_speed)
		Vr=-max_speed;
	

	int right=(int) ((255.0)*Vr/max_speed);
	int left=(int) ((255.0)*Vl/max_speed);

	set_left_direction(left > 0 ? true : false);
	set_right_direction(right > 0 ? true : false);

	analogWrite(left_forward_pin, left > 0 ? left : 0);
	analogWrite(left_backward_pin, left <= 0 ? -left : 0);
	analogWrite(right_forward_pin, right > 0 ? right : 0);
	analogWrite(right_backward_pin, right <= 0 ? -right : 0);
}

double fix_angle(double angle)
{
	return atan2(sin(angle), cos(angle));
}

void set_angle(double desired_angle)
{
	double proportional=1.0, integral=5.0, derivative=0.1; //Why does the derivative value need to be so high???
	double int_angle=0.0;
	double eangle=fix_angle(desired_angle-current_angle());
	const double wait_time=0.01;//in seconds
	double  prev_angle=eangle;
	const double acceptable_angle_error=2.0*pi/double(ticks_per_revolution);
	double turn_rate;

	while (abs(fix_angle(eangle)) >= acceptable_angle_error) {
		eangle=fix_angle(desired_angle-current_angle());
		double int_part=0.0;
		int_part=integral*int_angle;
		
		turn_rate=proportional*eangle+int_part+derivative*(eangle-prev_angle)/wait_time;
		prev_angle=eangle;
		int_angle=int_angle+wait_time*eangle;
		control_wheels(0.0, turn_rate);
		delay((int) (wait_time*1000.0));
	}
	halt();
}

void turn(double angle)
{
	set_angle(fix_angle(current_angle()+angle));
}

int goto_goal(double desired_x, double desired_y, int (*exit_func) ())
{
	double dx=desired_x-current_x(), dy=desired_y-current_y();
	double distance=sqrt(dx*dx+dy*dy);
	const double max_dist=1.0;//cm
	const double max_angle_error=2.0*pi/double(ticks_per_revolution);
	double eangle=fix_angle(atan2(dy, dx)-current_angle());
	while (distance > max_dist) {
		println(distance);
		if (exit_func!=NULL) {
			int ret=exit_func();
			if (ret!=0) {
				halt();
				return ret;
			}
		}

		if (abs(eangle) > max_angle_error) {
			halt();
			set_angle(atan2(dy, dx));
		}
		control_wheels(-10.0, 0.0);
		dx=desired_x-current_x(); dy=desired_y-current_y();
		distance=sqrt(dx*dx+dy*dy);
		eangle=fix_angle(atan2(dy, dx)-current_angle());
		delay(1);
	}
	halt();
	return 0;
}

int forward(double distance, int (*exit_func) ()) //cm
{
	return goto_goal(current_x()+distance*cos(current_angle()), current_y()+distance*sin(current_angle()), exit_func);
}

double measure_distance(int timeout, int trig_pin, int echo_pin)
{
	digitalWrite(trig_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(trig_pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig_pin, LOW);
	long int duration=pulseIn(echo_pin, HIGH);
	double distance = (double(duration)/2)/29.1;
	return distance;
}

void panic()
{
	println("Entered panic");
	while (1) {}
}

String query_i2c_dev(int id, char *query)
{
	Wire.beginTransmission(id);
	Wire.write(query);
	Wire.endTransmission();
	String response="";
	int x=0;
	while (!response.endsWith("\n")) {
		Wire.requestFrom(id, 1);
		response+=char(Wire.read());
		x++;
		if (x>20)
			panic();
	}
	return response;
}

double current_x()
{
	String response=query_i2c_dev(sensor_id, "?x\n");
	double ret=response.toFloat();
	response="";
	return ret;
}

double current_y()
{
	String response=query_i2c_dev(sensor_id, "?y\n");
	double ret=response.toFloat();
	response="";
	return ret;
}

double current_angle()
{
	String response=query_i2c_dev(sensor_id, "?theta\n");
	double ret=response.toFloat();
	response="";
	return ret;
}

double get_battery_voltage()
{
	return (12.0/5.0)*(5.0/1023.0)*double(analogRead(power_meter));
}

void loop()
{
	/* goto_goal(50.0, 50.0);
	goto_goal(-50.0, 50.0);
	goto_goal(-50.0, -50.0);
	goto_goal(50.0, -50.0); */
	/* forward(50.0, until_wall_gone);
	if (Serial.available())
		while (1) {}
	turn(pi/2.0);
	if (Serial.available())
		while (1) {}
	forward(50.0, until_wall_appears);
	if (Serial.available())
		while (1) {} */

	println(get_battery_voltage());
	//forward(100.0, NULL);
}
