/* TODO: Get a much more powerful battery. */
const int left_forward_pin=6;
const int left_backward_pin=5;
const int right_forward_pin=9;
const int right_backward_pin=10;
const int right_wheel_input=A1;
const int left_wheel_input=A0;
const int interrupt_pin=0;
const int led_pin=8;
const double body_length=18.0; //cm
const double wheel_radius=3.5; //cm
const double max_speed=12.5; //rads / sec
const double pi=3.141592;
const int ticks_per_revolution=19;
volatile int num_ticks_left=0, num_ticks_right=0;
volatile double current_x=0.0, current_y=0.0, current_angle=0.0;
volatile bool left_direction=true, right_direction=true;
const int min_pwm=0;

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

	double dist_left=2.0*pi*wheel_radius*((double) (left-num_ticks_left))/((double) ticks_per_revolution);
	double dist_right=2.0*pi*wheel_radius*((double) (right-num_ticks_right))/((double) ticks_per_revolution);
	double center_distance=(dist_left+dist_right)/2.0;

	current_angle+=(dist_right-dist_left)/body_length;
	current_angle=fix_angle(current_angle);
	current_x+=center_distance*cos(current_angle);
	current_y+=center_distance*sin(current_angle);
	interrupts();
}

void setup()
{
	pinMode(right_backward_pin, OUTPUT);
	pinMode(right_forward_pin, OUTPUT);
	pinMode(left_forward_pin, OUTPUT);
	pinMode(left_backward_pin, OUTPUT);
	pinMode(right_wheel_input, INPUT);
	pinMode(left_wheel_input, INPUT);
	pinMode(led_pin, OUTPUT);
	analogWrite(led_pin, 0);
	halt();
	attachInterrupt(interrupt_pin, interrupt_handle, CHANGE);
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

	halt();
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

void goto_goal(double desired_x, double desired_y)
{
	double dx=desired_x-current_x, dy=desired_y-current_y;
	double distance=sqrt(dx*dx+dy*dy);
	const double max_dist=1.0;//cm
	const double max_angle_error=2.0*pi/double(ticks_per_revolution);
	double eangle=fix_angle(atan2(dy, dx)-current_angle);
	while (distance > max_dist) {
		if (abs(eangle) > max_angle_error) {
			halt();
			set_angle(atan2(dy, dx));
		}
		control_wheels(-100.0, 0.0);
		dx=desired_x-current_x; dy=desired_y-current_y;
		distance=sqrt(dx*dx+dy*dy);
		eangle=fix_angle(atan2(dy, dx)-current_angle);
		delay(10);
	}
	halt();
}


void loop()
{
	goto_goal(0.0, 100.0);
	while (1) {}
}
