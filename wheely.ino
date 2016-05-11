/* TODO: Get a much more powerful battery. */
/* TODO: Get two optical mice, and use their internal sensors. Look into ADNS-2610. */
const int left_forward_pin=6;
const int left_backward_pin=5;
const int right_forward_pin=9;
const int right_backward_pin=10;
const int right_wheel_input=A1;
const int left_wheel_input=A0;
const int interrupt_pin=0, interrupt_pin2=1;
const int left_echo_pin=11, left_trig_pin=12;
const int right_echo_pin=4, right_trig_pin=3;
const double body_length=18.0; //cm
const double wheel_radius=3.5; //cm
const double max_speed=12.5; //rads / sec
const double pi=3.14159265358979;//32384626433832790528841971633993 <- adjust as needed.
const int ticks_per_revolution=38;
const double tick_unit=2.0*pi*wheel_radius/double(ticks_per_revolution);
volatile int num_ticks_left=0, num_ticks_right=0;
volatile double current_x=0.0, current_y=0.0, current_angle=0.0;
volatile bool left_direction=true, right_direction=true;
const int min_pwm=0;
const int front_an=A5;

void setup_pins()
{
	pinMode(left_echo_pin, INPUT);
	pinMode(left_trig_pin, OUTPUT);
	pinMode(right_backward_pin, OUTPUT);
	pinMode(right_forward_pin, OUTPUT);
	pinMode(left_forward_pin, OUTPUT);
	pinMode(left_backward_pin, OUTPUT);
	pinMode(right_wheel_input, INPUT);
	pinMode(left_wheel_input, INPUT);
}
#define DEBUG
void halt()
{
	analogWrite(left_forward_pin, 0);
	analogWrite(right_forward_pin, 0);
	analogWrite(left_backward_pin, 0);
	analogWrite(right_backward_pin, 0);
}

void interrupt_handle()
{
	noInterrupts();
	int left=num_ticks_left;
	int right=num_ticks_right;
  
	if (digitalRead(left_wheel_input)) {
		if (left_direction)
			num_ticks_left++;
		else
			num_ticks_left--;
	}

	if (digitalRead(right_wheel_input)) {
		if (right_direction)
			num_ticks_right++;
		else
			num_ticks_right--;
	}

	double dist_left=((double) (left-num_ticks_left))*tick_unit;
	double dist_right=((double) (right-num_ticks_right))*tick_unit;
	double center_distance=(dist_left+dist_right)/2.0;

	current_angle+=(dist_right-dist_left)/body_length;
	current_angle=fix_angle(current_angle);
	current_x+=center_distance*cos(current_angle);
	current_y+=center_distance*sin(current_angle);
	interrupts();
}

void setup()
{
	setup_pins();
	halt();
	attachInterrupt(interrupt_pin, interrupt_handle, CHANGE);
	attachInterrupt(interrupt_pin2, interrupt_handle, HIGH);
	interrupts();
	delay(1000);
	#ifdef DEBUG
	Serial.begin(9600);
	#endif
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

	noInterrupts();
	left_direction=left > 0 ? true : false;
	right_direction = right > 0 ? true : false;

	analogWrite(left_forward_pin, left > 0 ? left : 0);
	analogWrite(left_backward_pin, left <= 0 ? -left : 0);
	analogWrite(right_forward_pin, right > 0 ? right : 0);
	analogWrite(right_backward_pin, right <= 0 ? -right : 0);
	interrupts();
}

double fix_angle(double angle)
{
	return atan2(sin(angle), cos(angle));
}

void set_angle(double desired_angle)
{
	double proportional=1.0, integral=5.0, derivative=0.1; //Why does the derivative value need to be so high???
	double int_angle=0.0;
	double eangle=fix_angle(desired_angle-current_angle);
	const double wait_time=0.01;//in seconds
	double  prev_angle=eangle;
	const double acceptable_angle_error=2.0*pi/double(ticks_per_revolution);
	double turn_rate;

	while (abs(fix_angle(eangle)) >= acceptable_angle_error) {
		eangle=fix_angle(desired_angle-current_angle);
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
	set_angle(fix_angle(current_angle+angle));
}

int goto_goal(double desired_x, double desired_y, int (*exit_func) ())
{
	double dx=desired_x-current_x, dy=desired_y-current_y;
	double distance=sqrt(dx*dx+dy*dy);
	const double max_dist=1.0;//cm
	const double max_angle_error=2.0*pi/double(ticks_per_revolution);
	double eangle=fix_angle(atan2(dy, dx)-current_angle);
	while (distance > max_dist) {
		int ret=exit_func();
		if (ret!=0) {
			halt();
			return ret;
		}

		if (abs(eangle) > max_angle_error) {
			halt();
			set_angle(atan2(dy, dx));
		}
		control_wheels(-10.0, 0.0);
		dx=desired_x-current_x; dy=desired_y-current_y;
		distance=sqrt(dx*dx+dy*dy);
		eangle=fix_angle(atan2(dy, dx)-current_angle);
		delay(1);
	}
	halt();
	return 0;
}

int forward(double distance, int (*exit_func) ()) //cm
{
	return goto_goal(current_x+distance*cos(current_angle), current_y+distance*sin(current_angle), exit_func);
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

int until_wall_gone()
{
	double d=measure_distance(50.0, left_trig_pin, left_echo_pin);
	if (d >= 50.0)
		return 1;
	else
		return 0;
}

int until_wall_appears()
{
	double d=measure_distance(50.0, left_trig_pin, left_echo_pin);
	if (d < 5.0)
		return 1;
	else
		return 0;
}

void loop()
{
	/* goto_goal(50.0, 50.0);
	goto_goal(-50.0, 50.0);
	goto_goal(-50.0, -50.0);
	goto_goal(50.0, -50.0); */
	forward(50.0, until_wall_gone);
	turn(pi/2.0);
	forward(50.0, until_wall_appears);
	while (1) {}
}
