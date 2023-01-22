//for line follower experiment
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

//sensor readings
unsigned char l = 0;
unsigned char c = 0;
unsigned char r = 0;

void motion_pin_config (void)
{
	DDRB = DDRB | 0x0F; //set direction of the PORTB3 to PORTB0 pins as output
	PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
	DDRD = DDRD | 0x30; //Setting PD4 and PD5 pins as output for PWM generation
	PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char newPORTB = 0;
	Direction &= 0x0F; // removing upper nibble as it is not needed
	newPORTB = PORTB; // reading the PORTB's original status
	newPORTB &= 0xF0; // setting lower direction nibble to 0
	newPORTB |= Direction; // adding lower nibble for direction command and restoring the PORTB status
	PORTB = newPORTB; // setting the command to the port
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void hard_stop (void) //hard stop(stop suddenly)
{
	motion_set(0x00);
}

void soft_stop (void) //soft stop(stops slowly)
{
	motion_set(0x0F);
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADMUX = 0x20;
	ACSR = 0x80;
	ADCSRA = 0x86;
}

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRA = 0x00; //set PORTF direction as input
	PORTA = 0x00; //set PORTF pins floating
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
}

void timer1_init(void)
{
	TCCR1B = 0x00; //stop
	TCNT1H = 0xFF; //setup
	TCNT1L = 0x01;
	OCR1AH = 0x00;
	OCR1AL = 0xFF;
	OCR1BH = 0x00;
	OCR1BL = 0xFF;
	ICR1H = 0x00;
	ICR1L = 0xFF;
	TCCR1A = 0xA1;
	TCCR1B = 0x0D; //start Timer
}

void init_devices (void)
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer1_init();
	sei(); //Enables the global interrupts
}

//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40; //Set start conversion bit
	while((ADCSRA&0x10)==0); //Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	return a;
}

//custom functions

void velocity(unsigned char left_motor, unsigned char right_motor){
	//PWM control
	OCR1AL = left_motor;
	OCR1BL = right_motor;
}

void speed_control(int left_motor, int right_motor){
	unsigned char l_val, r_val;
	unsigned char newPORTB = PORTB;
	newPORTB &= 0xF0;
	
	if(left_motor < 0){
		//take absolute value
		l_val = -left_motor;
		//perform soft left 2
		newPORTB |= 0x01;
	}
	else{
		l_val = left_motor;
		//perform soft right
		newPORTB |= 0x02;
	}
	
	if(right_motor < 0){
		//take absolute value
		r_val = -right_motor;
		//perform soft right 2
		newPORTB |= 0x08;
	}
	else{
		r_val = right_motor;
		//perform soft left
		newPORTB |= 0x04;
	}
	
	l_val = l_val > 255 ? 255 : l_val;
	r_val = r_val > 255 ? 255 : r_val;
	PORTB = newPORTB;
	velocity(l_val, r_val);
}

void correction(double translate, double rotate){
	int l_val, r_val;
	
	if(rotate < 0){
		l_val = translate + 2*translate*rotate;
		r_val = translate;
	}
	else{
		l_val = translate;
		r_val = translate - 2*translate*rotate;
	}
	speed_control(l_val, r_val);
}

//main Function
int main(void)
{
	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	// control value constants
	double kp = 1.05; 
	double ki = 0.00000028;
	double kd = 0.49;
	double dt = 1;
	
	double prev_err = 0;
	double prev_rotate = 0;
	double integral = 0;
	
	while(1){
		
		double cl, cc, cr;
		double offset = 0.5;
		
		l=ADC_Conversion(3);
		c=ADC_Conversion(4);
		r=ADC_Conversion(5);
		
		//Display The Sensor Values
		// lcd_print(1, 1, l, 3);
		// lcd_print(1, 5, c, 3);
		// lcd_print(1, 9, r, 3);
		
		cl = l;
		cc = c;
		cr = r;
		
		double average = (cl + cc + cr)/3;
		double err = (cr - cl);
		double control_sig = kp * err + ki * integral + kd * (err - prev_err)/dt;
		
		integral += err * dt;
		prev_err = err;
		
		//set the translate
		double translate = 40 + average*0.85;
		//check if its in range of 255
		translate = (translate > 255) ? 255 : translate;
		
		//set the value of rotate
		double rotate = prev_rotate + (control_sig/(cc + offset));
		//check if its in range of -1 to 1
		rotate = (rotate < -1) ? -1 : rotate;
		rotate = (rotate > 1) ? 1 : rotate;
		
		if((cl > 50 && cr > 50) && cc < 120) {
			rotate = 0;
			translate /= 8;
			correction(translate, rotate);
			_delay_ms(100);
		}
		
		correction(translate, rotate);
		prev_rotate = 0.2*rotate;
		
	}
}
