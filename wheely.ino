const int left_forward_pin=6;
const int left_backward_pin=5;
const int right_forward_pin=10;
const int right_backward_pin=11;
const int right_wheel_input=A4;
const int left_wheel_input=A3;
const int interrupt_pin=1;
const int led_pin=8;
const double body_length=0.18; //meters
const double wheel_radius=0.035; //meters
const double max_speed=0.93; //rads / sec
const double pi=3.141592;
const int ticks_per_revolution=38;
volatile int num_ticks_left=0, num_ticks_right=0;
volatile double current_x=0.0, current_y=0.0, current_angle=0.0;
volatile bool left_direction=true, right_direction=true;

void halt()
{
	analogWrite(left_forward_pin, 0);
	analogWrite(right_forward_pin, 0);
	analogWrite(left_backward_pin, 0);
	analogWrite(right_backward_pin, 0);
}

void interrupt_handle ()
{
	noInterrupts();
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

	double dist_left=2*pi*wheel_radius*(left-num_ticks_left)/ticks_per_revolution;
	double dist_right=2*pi*wheel_radius*(left-num_ticks_right)/ticks_per_revolution;
	double center_distance=(dist_left+dist_right)/2.0;

	current_angle=fix_angle(current_angle+(dist_right-dist_left)/body_length);
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
}


/*TODO: remove is_forward argument and allow negative amounts. */
void wheel(bool is_left, bool is_forward, int amount)
{
	amount=abs(amount);
	if (!is_left && is_forward) {
		right_direction=true;
		analogWrite(right_backward_pin, 0);
		analogWrite(right_forward_pin, amount);
		digitalWrite(led_pin, HIGH);
	} else if (!is_left && !is_forward) {
		right_direction=false;
		analogWrite(right_backward_pin, amount);
		analogWrite(right_forward_pin, 0);
	} else if (is_left && is_forward) {
		left_direction=true;
		analogWrite(left_forward_pin, amount);
		analogWrite(left_backward_pin, 0);
	} else if (is_left && !is_forward) {
		left_direction=false;
		analogWrite(left_forward_pin, 0);
		analogWrite(left_backward_pin, amount);
	}
}


void control_wheels(double velocity, double turn_rate)
{
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
        if (Vl > 0.0)
                 wheel(true, true, (int) ((255.0)*Vl/max_speed));
        else
                wheel(true, false, (int) ((255.0)*Vl/max_speed));

        if (Vr > 0.0)
                wheel(false, true, (int) ((255.0)*Vr/max_speed));
        else
                wheel(false, false, (int) ((255.0)*Vr/max_speed));
}

double fix_angle(double angle)
{
	return atan2(sin(angle), cos(angle));
}


void goto_goal(double x, double y)
{
	double ex=x-current_x, ey=y-current_y;
	double pd=1.0, id=1.0, dd=0.1;
	double pa=1.0, ia=0.0, da=0.0;
	double int_dist=0.0, int_angle=0.0;
	double dist=sqrt(ex*ex+ey*ey), eangle=fix_angle(atan2(ey, ex)-current_angle);
	const double wait_time=0.001;//in seconds
	double prev_dist=dist, prev_angle=eangle;

	//while (dist > 0.1) {
		while (abs(fix_angle(eangle)) >pi*5.0/180.0) {
			ex=x-current_x;
			ey=y-current_y;

			eangle=fix_angle(atan2(ey, ex)-current_angle);
			double turn_rate=fix_angle(pa*eangle+ia*int_angle+da*(eangle-prev_angle)/wait_time);
			prev_angle=eangle;
			int_angle=fix_angle(int_angle+wait_time*eangle);
			control_wheels(0.0, turn_rate);
			delay((int) (wait_time*1000.0));
		}
		halt();
		/*double velocity=
		prev_angle=angle;
		prev_dist=dist;
		int_dist+=dist;
		ex=x-current_x;
		ey=y-current_y;
		dist=sqrt(ex*ex+ey*ey);
	} */
}

void loop()
{
	//control_wheels(0.0, 1.0);
	goto_goal(0.0, 1.0);
	//while(1) {}
}
