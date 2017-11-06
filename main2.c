/* Copyright (c) 2012, Ulf Kulau
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "uart.h"

#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "erik/erik.h"
#include "interfaces/pressure-bmp085.h"
#include "interfaces/gyro-l3g4200d.h"
#include <math.h>


/*
 * RX Frame:
 * rxbuffer[0] = byte1 = alternating byte to indicate new frame
 * rxbuffer[1] = byte2 = Checksum
 * rxbuffer[2] = byte3 = current temperature value
 * rxbuffer[3] = current OSCALL value
 * rxbuffer[4] = fact_default_osccal;
 * rxbuffer[5] = reset_flag
 *
 * TX Frame:
 * txbuffer[0] = lock or unlock byte for the SI
 * txbuffer[1] = OSCALL
 * txbuffer[2] = register value of the digital potentiometer / voltage level
 * txbuffer[3 + 4] = INFORMATION - delta_t counter information
 * txbuffer[5] = debug_info;
 *
 *
 * */
 

#define ATINY_WRITE_MODE 0x5A
#define ATINY_READ_MODE 0x5B

///Attiny Reply Addresses 
#define ATINY_LOCK_ADDR 0x00
#define ATINY_OSCCAL_ADDR 0x01
#define ATINY_VOLT_ADDR 0x02
#define ATINY_DELTA_T_ADDR 0x03
#define ATINY_DEBUG_ADDR 0x05


///Attiny Send Addresses
#define ATINY_TX_BUFFER 0x00
#define ATINY_FACT_ADDR 0x04


///Magnetometer Temperature Sensor Definitions
#define w_slave_add 0x1C //0b10010000
#define r_slave_add 0x1D //0b10010001
#define die_temp 0x0F

///Reset Configurations
static uint8_t alt_byte=1;
static uint8_t fac_def_osc;
static uint8_t reset_volt;
static uint8_t reset_flag=0;
static uint8_t count=1;//Just for checking reset

///Function Definitions
void init();
void check_reset();
uint8_t read_mag_temp();
void atiny_send_msg(uint8_t alt_byte1,uint8_t a_c1,uint8_t temp1,uint8_t cal1,uint8_t reset_flag1);
uint8_t atiny_read_msg(uint8_t addr);
void atiny_send_fact(uint8_t a);

///Debugging Data
static char str[10];

///ALU data
volatile float a_c = 0;
volatile float b_c = 0;

///Temperature Calliberation
int8_t temp;
int8_t temp_min=-60;
int8_t temp_max=15;
int8_t map(int8_t x, int8_t in_min, int8_t in_max, int8_t out_min, int8_t out_max);

int main(void) {
	static uint8_t cal;
	static uint8_t new_osccal,new_volt=200,debug_info;
	static uint16_t delta_t;
	
	init(); //Initialisations
	//uart_TXstring("Hello ");
	check_reset(); //Checking the source of the reset
	TCCR1B |= (1 << CS12); // Set up timer
	
	
	while (1) {
		//So that every 1 sec
	    if (TCNT1 >= 15624){
					TCNT1=0;

					//uart_TXstring("Wilkommen ");
					
					cal = OSCCAL;  //Current OSCCAL value
					
					
					
					
					//Calculating Square root
					a_c = 17.64;
					b_c = sqrt(a_c);
					a_c = 10 * b_c;
					a_c += 20;
					
					
					///Printing Temperatures from various sensors
					/*
					uart_TXstring("Temp Mag::");
					uart_TXstring(itoa((int8_t)read_mag_temp(),str,10));
					
					uart_TXstring("Temp Gyro::");
					uart_TXstring(itoa((int8_t)l3g4200d_get_temp(),str,10));*/
					
					temp=read_mag_temp(); ///Temperature to be used
					 
					//uart_TXstring("Temp Press::");
					//uart_TXstring(itoa((int8_t)temp,str,10));
					/*
					uart_TXchar(temp);
					uart_TXchar(l3g4200d_get_temp());
					uart_TXchar(bmp085_read_comp_temperature()/10);*/
					
					uart_TXstring(itoa((int8_t)temp,str,10));
					uart_TXstring(",");
					uart_TXstring(itoa((int8_t)l3g4200d_get_temp(),str,10));
					uart_TXstring(",");
					uart_TXstring(itoa((int8_t)bmp085_read_comp_temperature()/10,str,10));
					uart_TXstring(",");
					
					
					if (temp<temp_min || temp>temp_max){
						temp=101;
						//uart_TXstring("Temperature not valid.");
					}
					else{
						temp=map(temp,temp_min,temp_max,0,100);
					}
					
					
					
					/*
					///Just for checking reset problem
					if(new_volt==205 && count==1){
						_delay_ms(4500);
						count=0;
						TCNT1=0;
					}
					*/
					atiny_send_msg(alt_byte,a_c,temp,cal,reset_flag);
					
					while(!atiny_read_msg(ATINY_LOCK_ADDR)){_delay_ms(1);} //waiting till the calculation is over
					
					///Reading respective values
					new_osccal= atiny_read_msg(ATINY_OSCCAL_ADDR);
					new_volt=atiny_read_msg(ATINY_VOLT_ADDR);
					delta_t = (((uint16_t) atiny_read_msg(ATINY_DELTA_T_ADDR + 1) << 8) + atiny_read_msg(ATINY_DELTA_T_ADDR));
					debug_info=	atiny_read_msg(ATINY_DEBUG_ADDR);			
					///Debugging Information				
					
					utoa(new_volt, str, 10);
					//uart_TXstring("new_volt: ");
					uart_TXstring(str);
					uart_TXstring(",");

					utoa(new_osccal, str, 10);
					//uart_TXstring(",new_osccal: ");
					uart_TXstring(str);
					uart_TXstring(",");

					utoa(delta_t, str, 10);
					//uart_TXstring(",delta_t: ");
					uart_TXstring(str);
					uart_TXstring(",");
		
					utoa(debug_info, str, 10);
					//uart_TXstring(",last fill ");
					uart_TXstring(str);
					//uart_TXstring(",");
					
					/*
					uart_TXchar(new_volt);
					uart_TXchar(new_osccal);
					uart_TXchar(debug_info);
					*/
					uart_TXstring("\n");
					
					///Updating Values 
					OSCCAL = new_osccal;
					ad5242_write_radc(AD5242_CHN_A, new_volt);
					reset_flag=0;
					
					///Updating alternating byte
					if(alt_byte==2){
						alt_byte=1;
					}
					else{
						alt_byte ^= 1;
					} 
					
				}
		}
}

void init(){
	uart_init();
	i2c_init();
	//bmp085_init(); 
	//l3g4200d_init();
	OSC_SET_CPU_FRQ_NORMAL();
	_delay_ms(500);
}


void check_reset(){
	if(MCUSR & 0b00000001){
		fac_def_osc=OSCCAL;
		MCUSR &= 0b11111110;
		//uart_TXstring("Yes,Power On ");
		//uart_TXstring("\n");
		reset_flag=0;
		atiny_send_fact(fac_def_osc);
	}
	else if(MCUSR & 0b00000010){
		MCUSR &= 0b11111101;
		//uart_TXstring("Hardware Reset ");
		//uart_TXstring("\n");
		reset_flag=1;
	}
	else{
		alt_byte=2;
		reset_volt=atiny_read_msg(ATINY_VOLT_ADDR);
		ad5242_write_radc(AD5242_CHN_A,reset_volt-2);
		//uart_TXstring("Software Reset ");
		//uart_TXstring("\n");
		reset_flag=2;
	}
}

void atiny_send_msg(uint8_t alt_byte1,uint8_t a_c1,uint8_t temp1,uint8_t cal1,uint8_t reset_flag1){
	i2c_start(ATINY_WRITE_MODE);
	i2c_write(ATINY_TX_BUFFER);
	i2c_write(alt_byte1);
	i2c_write(a_c1);
	i2c_write(temp1);
	i2c_write(cal1);
	i2c_write(reset_flag1);
	i2c_stop();
	
	///Debugging info
	
	utoa(alt_byte1, str, 10);
	//uart_TXstring(",alt_byte ");
	uart_TXstring(str);
	uart_TXstring(",");
	
	utoa(a_c1, str, 10);
	//uart_TXstring(",a_c ");
	uart_TXstring(str);
	uart_TXstring(",");
	
	utoa(temp1, str, 10);
	//uart_TXstring(",temp ");
	uart_TXstring(str);
	uart_TXstring(",");
	
	utoa(cal1, str, 10);
	//uart_TXstring(",cal ");
	uart_TXstring(str);
	uart_TXstring(",");
	
	utoa(reset_flag1, str, 10);
	//uart_TXstring(",reset_flag ");
	uart_TXstring(str);
	uart_TXstring(",");
	
	/*
	uart_TXchar(alt_byte1);
	uart_TXchar(a_c1);
	uart_TXchar(temp1);
	uart_TXchar(cal1);
	uart_TXchar(reset_flag1);
	*/
}

void atiny_send_fact(uint8_t a){
	i2c_start(ATINY_WRITE_MODE);
	i2c_write(ATINY_FACT_ADDR);
	i2c_write(a);
	i2c_stop();
	
	///Debugging info
	utoa(a, str, 10);
	//uart_TXstring(",fact_default ");
	//uart_TXstring(str);
}

uint8_t atiny_read_msg(uint8_t addr) {
	i2c_start(ATINY_WRITE_MODE);
	i2c_write(addr);
	i2c_rep_start(ATINY_READ_MODE);
	uint8_t ret;
	i2c_read_nack(&ret);
	i2c_stop();
	return ret;
}

uint8_t read_mag_temp() {
	i2c_start(w_slave_add);
	i2c_write(die_temp);
	i2c_rep_start(r_slave_add);
	uint8_t ret;
	i2c_read_nack(&ret);
	i2c_stop();
	return ret;
}

int8_t map(int8_t x, int8_t in_min, int8_t in_max, int8_t out_min, int8_t out_max) 
{ 
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
}
