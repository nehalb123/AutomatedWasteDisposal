/*
 * Arm.c
 *
 * Created: 25-Mar-17 3:28:36 PM
 * Author : Nehal
 */ 

#define  F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define up 0x10
#define down 0x20
#define open 0x40
#define close 0x80
#define  halt 0x00 
//void port_init();
void arm_motion_set1(unsigned char Direction){
	unsigned char PortHRestore = 0;

	Direction &= 0xF0; 
	PortHRestore = PORTH;
	PortHRestore &= 0x0F;
	PortHRestore |= Direction;
	PORTH = PortHRestore;	
}

void arm_motion_set2(unsigned char Direction){
	unsigned char PortLRestore = 0;

	Direction &= 0xF0;
	PortLRestore = PORTL;
	PortLRestore &= 0x0F;
	PortLRestore |= Direction;
	PORTL = PortLRestore;
}
/*void init_devices(){
	cli();
	port_init();
	sei();
}*/
void arm12_pin_config(){
	DDRH= DDRH | 0x30;  //PH4 and  PH5 is set as output
	DDRL= DDRL | 0xC0;
	
}

/*int main(void)
{
   //IN1->PH4(PIN4)  IN2->PH5(PIN5)
   
   while(1){
	   arm_motion_set1(0x10);  //arm moves in upward direction
	   _delay_ms(2000);
       arm_motion_set1(0x20); //arm moves in downward direction
 	   _delay_ms(2000);
	  
	   
	   
   //IN3->PL6(PIN18)   IN4->PL7(PIN17)
    arm_motion_set2(0x40);  //arm opens
   _delay_ms(2000);
   arm_motion_set2(0x80);  //arm closes
   _delay_ms(2000);
   }
  
  
//   arm_motion_set2(open);
//   _delay_ms(2000);
//   arm_motion_set1(down);
//   _delay_ms(3000);
//   arm_motion_set1(halt);
//   arm_motion_set2(close);
//   _delay_ms(5000);
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
_delay_ms(2100);  //add down if necessary
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
_delay_ms(1200);
arm_motion_set1(halt);
_delay_ms(500);
arm_motion_set2(close);
_delay_ms(500);
arm_motion_set2(halt);
_delay_ms(300);
  //_delay_ms(3000);
  //arm_motion_set1(0x20);
  //_delay_ms(1000);
   
}
*/

