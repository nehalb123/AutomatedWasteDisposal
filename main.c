/*
 * WM.c
 *
 * Created: 12-Mar-17 8:32:52 AM
 * Author : Nehal
 */ 

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
#include "servo.c"
#include "Arm.c"
#define cut_off  20
#define up 0x10
#define down 0x20
#define open 0x40
#define close 0x80
#define  halt 0x00
//#define Junction (((Sensor1>cut_off) && (Sensor2>cut_off))||((Sensor2>cut_off) && (Sensor3>cut_off)))

void port_init();  
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void line_follow();
void line_follow2(unsigned int); //Function parameter is ShaftCount
void go_tocentre();    // robot will wait at center till it receives a signal
void bin1_path();
void bin2_path();
void bin3_path();
void pick_up();
void trace();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Sensor3 = 0;  //Left
unsigned char Sensor2 = 0;  //Center
unsigned char Sensor1 = 0;  //Right
//int visited[6]={0};         //Array to store visited junctions
int junction_count=0;       //stores the count of junction	
volatile unsigned long int ShaftCountLeft=0;
volatile unsigned long int ShaftCountRight=0;
volatile unsigned int Degrees;
unsigned int flag_bin1=0;
unsigned int flag_bin3=0;

void lcd_port_config (void){
 DDRC = DDRC | 0xF7;    //LCD pins as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

void adc_pin_config (void){
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

void motion_pin_config (void) {
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}
//Function to configure INT4 (PORTE 4) pin as input for the left encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 5 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 5 pin
}

void buzzer_pin_config (void){
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void port_init(){
	lcd_port_config();
	motion_pin_config();	
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	arm12_pin_config(); //Configure PORTH and PORTL for arm
	buzzer_pin_config(); //Configure PORTC pin for buzzer
	left_encoder_pin_config(); //left encoder pin configure
	right_encoder_pin_config(); //right encoder pin configure
	adc_pin_config();
}

void rfinterrupt_pin_config(){
	
	DDRD=DDRD & 0xF0;
	PORTD=PORTD|0x01;   //Pull up
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); 
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); 
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   
}

void rf_init_interrupt1(){   //Interrupt 0 enable
	
	cli();
	EICRA =EICRA | 0x02     ;  //INT0 is set to falling edge
	EIMSK =EIMSK | 0x01     ;  //Enable interrupt 0
	sei();
}

//ISR for radio frequency module
ISR(INT0_vect){   
	/*PORTJ=0x00;
	_delay_ms(1000);
	PORTJ=0xFF;
	_delay_ms(1000);
	lcd_string("I love india");*/
	buzzer_on();
	_delay_ms(1000);
	buzzer_off();
	flag_bin3=1;
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}
// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init(){
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init(){
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external  ADLAR=1 MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x87;		//ADEN=1 ADIE=1 ADPS2:0 = 1 1 1(Prescalar:128)
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) {
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

void print_sensor(char row, char coloumn,unsigned char channel){
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

void velocity (unsigned char left_motor, unsigned char right_motor){
	    OCR5AL =  (unsigned char)left_motor;
		OCR5BL = (unsigned char)right_motor;
}

void motion_set (unsigned char Direction){
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		    // removing upper nibble for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibble to 0
	PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void)  
{
  motion_set (0x06);
}
void back(void){
	motion_set(0x09);
}
void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void dleft()  //Left wheel back,right forward
{
	motion_set(0x05);
}
void dright()   //left wheel forward,right backward
{
	motion_set(0x0A);
}
void soft_right()
{
	motion_set(0x02);
}
void soft_left()
{
	motion_set(0x04);
}
 void stop (void)
 {
   motion_set (0x00);
 }
 
 //Function to turn robot left till it finds black line
 void adjust_left()
 {
	 while(1)
	 {
		 
		 Sensor3 = ADC_Conversion(3);	
		 Sensor2 = ADC_Conversion(2);	
		 Sensor1 = ADC_Conversion(1);	

		 if(Sensor3<cut_off && Sensor2>cut_off && Sensor1<cut_off) //this condition will check whether robot is on black line or not and if it is on black line than it will break the loop
		 {
			 break;
		 }
		 velocity(120,120);
		 dleft();    //this will take left turn until it find black line
	 }
 }
 
 //Function to turn robot right till it finds black line
 void adjust_right(){
	 
	 while(1)
	 {
		 
		 Sensor3 = ADC_Conversion(3);	
		 Sensor2 = ADC_Conversion(2);	
		 Sensor1 = ADC_Conversion(1);	

		 if(Sensor3<cut_off && Sensor2>cut_off && Sensor1<cut_off) //this condition will check whether robot is on black line or not and if it is on black line than it will break the loop
		 {
			 break;
		 }
		 velocity(120,120);
		 dright();   //this will take right turn until it find black line
	 }
 } 
 
 
//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees){
	
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{


		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{

			break;
		}
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM){
	
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM){
	
	forward();
	linear_distance_mm(DistanceInMM);
	
}

void back_mm(unsigned int DistanceInMM){
	
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees){
	
	dleft(); //Turn left
	angle_rotate(Degrees);
}

void right_degrees(unsigned int Degrees)
{
	
	dright(); //Turn right
	angle_rotate(Degrees);
}

void soft_left_degrees(unsigned int Degrees)
{
	
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void buzzer_on (void){
	unsigned char restore = 0;
	restore = PINC;
	restore = restore | 0x08;
	PORTC = restore;
}

void buzzer_off (void){
	unsigned char restore = 0;
	restore = PINC;
	restore = restore & 0xF7;
	PORTC = restore;
}

void turnoff_rf(){
	PORTD=PORTD & 0xBF;  //PD6 is low
	_delay_ms(1000);
}

void turnon_rf(){
	//PD6 is connected to receiver's supply
	DDRD=DDRD | 0x40;    //PD6 pin as output
	PORTD=PORTD|0x40;    //PD6 is high
}

void init_devices (void){
	
 	cli(); 
	port_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	adc_init();
	timer5_init();
	timer1_init();
	sei();  
}

void junction_detect(){
	
	Sensor3 = ADC_Conversion(3);  //store data from left sensor
	Sensor2 = ADC_Conversion(2);  //store data from middle sensor
	Sensor1 = ADC_Conversion(1);  //store data from right sensor

		
	if(ShaftCountLeft>63 || ShaftCountRight>63){
			
		  ShaftCountLeft=0; ShaftCountRight=0;
		  junction_count++;
		  switch(junction_count){
			  
			case 1:
			forward_mm(25);
			right_degrees(80);  //turn 1
			_delay_ms(1000);
			adjust_right();
			forward_mm(140);
			left_degrees(85);
			_delay_ms(1000);
			adjust_left();  
			stop();
			_delay_ms(4000);
			pick_up();
			right_degrees(75);
			_delay_ms(1000);
			adjust_right();
			
			
			stop();
			_delay_ms(1000);
			
			if(1){                    //if called
				line_follow2(29);
				left_degrees(80);    //turn 2
				_delay_ms(1000);
				adjust_left();
				stop();
				_delay_ms(500);
				line_follow2(57);
				
				forward_mm(20);
				right_degrees(85); //turn 3
				_delay_ms(1000);
				adjust_right();
				stop();
				_delay_ms(500);
				line_follow2(26);
				stop();
				_delay_ms(1000);
				left_degrees(75);    //turn to pick bin
				_delay_ms(1000);
				adjust_left();
				//pick_up();
				stop();
				_delay_ms(1000);
				right_degrees(70);
				_delay_ms(1000);
				adjust_right();
				stop();
				_delay_ms(1000);
				line_follow2(31);
				
				right_degrees(75); //turn 4
				_delay_ms(1000);
				adjust_right();
				stop();
				_delay_ms(500);
			
				line_follow2(57);
				stop();
				_delay_ms(500);
				
				left_degrees(85);    //turn 5
				_delay_ms(1000);
				adjust_left();
				line_follow2(27);
				left_degrees(80);    //turn to pick bin
				_delay_ms(1000);
				adjust_left();
				//pick_up();
				stop();
				_delay_ms(1000);
				right_degrees(70);
				_delay_ms(1000);
				adjust_right();
				stop();
				_delay_ms(1000);
				
				line_follow2(61); //dump
				left_degrees(150);
				_delay_ms(1000);
				adjust_left();
				stop();
				_delay_ms(1000);
				back_mm(50);
				stop();
				_delay_ms(6000);
			
				go_tocentre();  //equidistant position where it can serve the bins
				
			}
			
		}
	}
	
}

void line_follow(){
	while(1){
		Sensor3 = ADC_Conversion(3);  //store data from left sensor 
		Sensor2 = ADC_Conversion(2);  //store data from middle sensor
		Sensor1 = ADC_Conversion(1);  //store data from right sensor
		
        junction_detect();
		//flag=0;
		print_sensor(1,1,3);
		print_sensor(1,5,2);
		print_sensor(1,9,1);
		
		forward();
	
		if((Sensor2>cut_off)&&(Sensor1<cut_off)&&(Sensor3<cut_off))
		{
			flag=1;
			forward();
			velocity(200,200);
		}

		if((Sensor3>cut_off)&&(Sensor2<cut_off)&&(Sensor1<cut_off))
		{
			flag=1;
			forward();
			velocity(120,180);  //left turn
		}

		if((Sensor1>cut_off)&&(Sensor2<cut_off)&&(Sensor3<cut_off))
		{
			flag=1;
			forward();
			velocity(180,120);   //right turn
		}

		if(Sensor1<cut_off && Sensor2<cut_off && Sensor3<cut_off)
		{
			stop();
		}
	}
		
}

void line_follow2(unsigned int i){
	ShaftCountLeft=0;
	ShaftCountRight=0;
	while(1){
		Sensor3 = ADC_Conversion(3);  //store data from left sensor
		Sensor2 = ADC_Conversion(2);  //store data from middle sensor
		Sensor1 = ADC_Conversion(1);  //store data from right sensor
		
		
		//flag=0;
		print_sensor(1,1,3);
		print_sensor(1,5,2);
		print_sensor(1,9,1);
		
		forward();
		
		if((Sensor2>cut_off)&&(Sensor1<cut_off)&&(Sensor3<cut_off))
		{
			flag=1;
			forward();
			velocity(200,200);
		}

		if((Sensor3>cut_off)&&(Sensor2<cut_off)&&(Sensor1<cut_off))
		{
			flag=1;
			forward();
			velocity(120,180);  //left turn
		}

		if((Sensor1>cut_off)&&(Sensor2<cut_off)&&(Sensor3<cut_off))
		{
			flag=1;
			forward();
			velocity(180,120);   //right turn
		}

		if(Sensor1<cut_off && Sensor2<cut_off && Sensor3<cut_off)
		{
			stop();
		}
		if(ShaftCountLeft>i || ShaftCountRight>i)
		{
			break;
		}
		
	}
	
}
void go_tocentre(){
	
	line_follow2(136);
	stop();
	_delay_ms(1000);
	
	turnon_rf();
	rfinterrupt_pin_config();
	rf_init_interrupt1();
	while(1){
	
	/*if(flag_bin1){
		bin1_path();
		continue;
	}
	if(bin2_calls){
		bin2_path();
		continue;
	}
	if(flag_bin3){
		bin3_path();
		continue;
	}*/
		
  }
}

/*void bin1_path(){
	line_follow2();
	right_degrees(70);
	stop();
	_delay_ms(500);
	adjust_right();
	stop();
	_delay_ms(1000);
	pick_up();
}*/

void bin2_path(){
	
}

void bin3_path(){
	left_degrees(165);  //go to bin 3 when full
	adjust_left();
	line_follow2(55);
	stop();
	_delay_ms(1000);
	left_degrees(70);
	adjust_left();
	stop();
	_delay_ms(1000);
}

void pick_up(){
	arm_motion_set2(open);
	_delay_ms(500);
	arm_motion_set2(halt);
	_delay_ms(1000);
	arm_motion_set1(down);
	_delay_ms(1500);
	arm_motion_set1(halt);
	_delay_ms(500);
	arm_motion_set2(close);
	_delay_ms(700);
	arm_motion_set2(halt);
	_delay_ms(500);
	arm_motion_set1(up);
	//_delay_ms(2000);
	_delay_ms(4200);   //with garbage
	arm_motion_set1(halt);
	_delay_ms(2000);
	arm_motion_set1(down);
	_delay_ms(2000);  
	arm_motion_set1(halt);
	_delay_ms(500);
	arm_motion_set1(down);  //release the bin
	_delay_ms(1200);
	arm_motion_set1(halt);
	_delay_ms(500);
	arm_motion_set2(open);
	_delay_ms(500);
	arm_motion_set2(halt);
	_delay_ms(500);
	arm_motion_set1(up);  //bring arm to normal position
	_delay_ms(1300);
	arm_motion_set1(halt);
	_delay_ms(500);
	arm_motion_set2(close);
	_delay_ms(500);
	arm_motion_set2(halt);
	_delay_ms(300);
	
}
/*void trace(){
		if(visited[0]==0) {
		forward_mm(330);
		adjust_right();
		forward_mm(330);
		adjust_left();
		forward_mm(310);
		visited[1]=1;
		//line_follow();
		}
}*/
int main(){
	init_devices();
	lcd_set_4bit();
	lcd_init();
    
	servo_1(0);
	_delay_ms(1000);
	velocity(180,180);
	forward();
    line_follow();
	// trace();
}