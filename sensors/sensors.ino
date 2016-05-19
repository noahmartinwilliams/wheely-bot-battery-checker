#include <Wire.h>
/* Running on Arduino micro. */
const int ticks_per_revolution=38;
const double pi= 3.141592; //65358979323846264338379052884197163993;
const double wheel_radius=3.5; //cm
const double body_width=18.0;
const double tick_unit=2.0*pi*wheel_radius/double(ticks_per_revolution);
volatile int num_ticks_left=0, num_ticks_right=0;
volatile double current_x=0.0, current_y=0.0, current_angle=0.0;
volatile bool left_direction=true, right_direction=false;
const int decimal_places=3;
String response=String(0.0, 3)+String("\n"), command;
volatile int response_index=0;



#ifdef DEBUG
const int right_int_pin=0; // pin 7
const int left_int_pin=1; //pin 0
#else
const int right_int_pin=2;
const int left_int_pin=3;
#endif 

#ifdef DEBUG 
#define print(X) Serial.print(X)
#define println(X) Serial.println(X)
#else
#define print(X) (void) 0
#define println(X) (void) 0
#endif

void left_interrupt()
{
	noInterrupts();
	println("entering left interrupt");
	int left=num_ticks_left;
	if (left_direction)
		num_ticks_left++;
	else
		num_ticks_left--;
	
	double dist_left=double(left-num_ticks_left)*tick_unit;
	double center_distance=dist_left;
	current_angle+=-dist_left/body_width;
	current_x+=center_distance*cos(current_angle);
	current_y+=center_distance*sin(current_angle);
	interrupts();
}

void right_interrupt()
{
	noInterrupts();
	println("entering right interrupt");
	int right=num_ticks_right;
	if (right_direction)
		num_ticks_right++;
	else
		num_ticks_right--;
	double dist_right=double(right-num_ticks_right)*tick_unit;
	double center_distance=dist_right;
	current_angle+=dist_right/body_width;
	current_x+=center_distance*cos(current_angle);
	current_y+=center_distance*sin(current_angle);
	interrupts();
}

void write_float(double f)
{
	String s=String(f, 3);
	int x;
	for (x=0; s[x]!='\0'; x++) {
		Wire.write(s[x]);
	}
}

void handle_request()
{
	Wire.write(response.c_str()[response_index]);
	response_index++;
}

void handle_receive(int numBytes)
{
	while (Wire.available() > 0) {
		command+=char(Wire.read());
	}

	if (command=="?x") {
		response=String(current_x, 3)+"\n";
		response_index=0;
		command="";
	}
}

void setup()
{
#ifdef DEBUG
	Serial.begin(9600);
#else
	Wire.begin(8);
	Wire.onRequest(handle_request);
	Wire.onReceive(handle_receive);
#endif
	current_x=1.0;
	pinMode(left_int_pin, INPUT);
	pinMode(right_int_pin, INPUT);
	attachInterrupt(right_int_pin, right_interrupt, CHANGE);
	attachInterrupt(left_int_pin, left_interrupt, CHANGE);
	interrupts();
}

void loop()
{
	print(current_x);
	print(":");
	print(current_y);
	print(":");
	println(current_angle);
	delay(500);
}
