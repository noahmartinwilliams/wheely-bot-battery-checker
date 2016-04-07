const int left_forward_pin=5;
const int left_backward_pin=3;
const int right_forward_pin=9;
const int right_backward_pin=10;
const int right_wheel_input=A1;
const int left_wheel_input=A0;
const int interrupt_pin=0;
const int led_pin=8;
const double body_length=18.0; //meters
const double wheel_radius=3.5; //meters
const double max_speed=0.93; //rads / sec
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

void interrupt_handle ()
{
	int left=num_ticks_left;
	int right=num_ticks_right;
  
	if (digitalRead(left_wheel_input)) 
		if (left_direction)
			num_ticks_left++;
		else
			num_ticks_left--;
	if (digitalRead(right_wheel_input))
		if (right_direction)
			num_ticks_right++;
		else
			num_ticks_right--;

	double dist_left=2.0*pi*wheel_radius*((double) (left-num_ticks_left))/((double) ticks_per_revolution);
	double dist_right=2.0*pi*wheel_radius*((double) (right-num_ticks_right))/((double) ticks_per_revolution);
	double center_distance=(dist_left+dist_right)/2.0;

	current_angle=current_angle+(dist_right-dist_left)/body_length;
	current_x+=center_distance*cos(current_angle);
	current_y+=center_distance*sin(current_angle);
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
	attachInterrupt(interrupt_pin, interrupt_handle, RISING);
	interrupts();
	delay(1000);
}


void wheel(bool is_left, int amount)
{
	int selected_zero_pin, selected_amount_pin;
	if (!is_left && amount > 0) {
		right_direction=true;
		selected_zero_pin=right_backward_pin;
		selected_amount_pin=right_forward_pin;
	} else if (!is_left && amount <=0) {
		right_direction=false;
		selected_zero_pin=right_forward_pin;
		selected_amount_pin=right_backward_pin;
	} else if (is_left && amount > 0) {
		left_direction=true;
		selected_zero_pin=left_backward_pin;
		selected_amount_pin=left_forward_pin;
	} else if (is_left && amount <= 0) {
		left_direction=false;
		selected_zero_pin=left_forward_pin;
		selected_amount_pin=left_backward_pin;
	}
	amount=abs(amount);
	if (amount < min_pwm)
		amount=min_pwm;
	analogWrite(selected_zero_pin, 0);
	analogWrite(selected_amount_pin, amount);
}


void control_wheels(double velocity, double turn_rate)
{
	turn_rate=-turn_rate;
        double Vr=(2.0*velocity+turn_rate*body_length)/(2.0*wheel_radius);
        double Vl=(2.0*velocity-turn_rate*body_length)/(2.0*wheel_radius);

	if (Vl >= max_speed)
		Vl=max_speed;
	else if (Vl <= -max_speed)
		Vl=-max_speed;

	if (Vr >= max_speed)
		Vr=max_speed;
	else if (Vr <= -max_speed)
		Vr=-max_speed;

	wheel(true, (int) ((255.0)*Vl/max_speed));

	wheel(false, (int) ((255.0)*Vr/max_speed));
}

double fix_angle(double angle)
{
	return atan2(sin(angle), cos(angle));
}


void set_angle(double desired_angle)
{
	double proportional=1.0, integral=1.0, derivative=5.0; //Why does the derivative value need to be so high???
	double int_angle=0.0;
	double eangle=fix_angle(desired_angle-current_angle);
	const double wait_time=0.01;//in seconds
	double  prev_angle=eangle;
	const double acceptable_angle_error=pi*2.0/180.0;
	double turn_rate;

		while (abs(fix_angle(eangle)) > acceptable_angle_error) {
			eangle=fix_angle(desired_angle-current_angle);
			turn_rate=proportional*eangle+integral*int_angle+derivative*(eangle-prev_angle)/wait_time;
			turn_rate=fix_angle(turn_rate);
			prev_angle=eangle;
			int_angle=fix_angle(int_angle+wait_time*eangle);
			control_wheels(0.0, turn_rate);
			delay((int) (wait_time*1000.0));
		}
		halt();
}

void turn(double angle)
{
	set_angle(fix_angle(current_angle+angle));
}

void loop()
{
	//control_wheels(0.0, 1.0);
	turn(pi/2.0);
	delay(1000);
}
