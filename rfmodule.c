/*
 * rf_module.c
 *
 * Created: 21-Mar-17 9:13:14 PM
 * Author : Nehal
 */ 
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"

void port_init();
void rf_module_init();
void rfinterrupt_pin_config();

void lcd_port_config (void){
	
	DDRC = DDRC | 0xF7;    //LCD pins as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

ISR(INT0_vect){

	PORTJ=0x00;
	_delay_ms(1000);
	PORTJ=0xFF;
	_delay_ms(1000);
	lcd_string("I love india");
}

/*ISR(INT1_vect){
	
	lcd_string("Data received");
	while(1);
}

ISR(INT2_vect){
	
	lcd_string("Data received");
	while(1);
	
}

ISR(INT3_vect){
	
	lcd_string("Data received");
	while(1);
}*/

void rf_init_interrupt1(){   //Interrupt 0 enable
	
	cli();
	EICRA =EICRA | 0x02     ;  //INT0 is set to falling edge
	EIMSK =EIMSK | 0x01     ;  //Enable interrupt 0
	sei();	
}

/*void rf_init_interrupt2(){   //Interrupt 1 enable
	cli();
	EICRA =EICRA | 0x08     ;  //INT1 is set to falling edge
	EIMSK =EIMSK | 0x02     ;  //Enable interrupt 1
	sei();
}

void rf_init_interrupt3(){   //Interrupt 2 enable
	cli();
	EICRA =EICRA | 0x20     ;  //INT2 is set to falling edge
	EIMSK =EIMSK | 0x04     ;  //Enable interrupt 2
	sei();
}

void rf_init_interrupt4(){   //Interrupt 3 enable
	cli();
	EICRA =EICRA | 0x80    ;  //INT3 is set to falling edge
	EIMSK =EIMSK | 0x08   ;  //Enable interrupt 3
	sei();
}*/

void rfinterrupt_pin_config(){
	
	DDRD=DDRD & 0xF0;
	PORTD=PORTD|0x01;   //Pull up
}

void rf_module_init(){
       DDRJ=0xFF;
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
void port_init(){
	
   lcd_port_config();
   rfinterrupt_pin_config();
   //rf_module_init();	
   
}

void init_devices(void){
	cli();
	port_init();
	rf_init_interrupt1();
	//rf_init_interrupt2();
	//rf_init_interrupt3();
	//rf_init_interrupt4();
	sei();
}

int main(){
	turnon_rf();
	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	
   _delay_ms(6000);
	
	/*PORTD=PORTD & 0xBF;  //PD6 is low
	_delay_ms(1000);*/
	turnoff_rf();
	
	turnon_rf();   
   while(1);
}