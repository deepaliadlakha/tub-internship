#include <util/delay.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/io.h>
//################################################################## USI-TWI-I2C

#include "twi_master.h" ///Secure Instance as master
#include "usiTwiSlave.h" ///Secure Instance as slave


struct entry {
	uint8_t voltage;
	uint8_t osccal;
};

#define 	SLAVE_ADDR_ATTINY       0x5A

#ifndef 	F_CPU
#define 	F_CPU 1000000UL
#endif


uint8_t byte1 = 0,byte2,obyte1 = 0,byte3,byte4,byte5;

volatile uint16_t this_delta_t = 0;
static uint8_t prv_temp=51,curr_temp=51;
static uint8_t first, current_voltage, iteration;
static uint16_t delta_t = 55050;
static uint8_t state;

///For deadlock reset
static uint8_t count_overflows=0;
static uint8_t fact_default;

struct entry table[51];

#define SI_INIT						0
#define SI_TRANSIENT				1
#define SI_MAIN						2

#define SI_STARTUP_DELAY			7
#define	SI_VOLT_REG_OFFSET			200

#define SI_RIGHT_CHECKSUM 			62
#define SI_ADAPTION_INTERVAL 		10
#define SI_DELTA_T_MARGIN			100

#define SI_LOCK()					txbuffer[0] = 0;
#define SI_UNLOCK()					txbuffer[0] = 1;

#define SI_REPLY_OSCCAL_FB(feedback)	txbuffer[1] = feedback;
#define SI_REPLY_VOLTAGE(voltage)	txbuffer[2] = voltage;
#define SI_REPLY_DELTA_T(cnt_value)	txbuffer[3] = cnt_value;\
									txbuffer[4] = (cnt_value >> 8);
									
#define SI_REPLY_DEBUG(debug)	txbuffer[5] = debug;

#define EEPROM_ADD 0x00

///Deadlock Reset
#define DEADLOCK_THRESHOLD 5 //overflows corresponds to a deadlock situation

///Digital Potentiometer Reset Software i2c
#define AD5242_DEV_ADDR_W		0x58
#define AD5242_CHN_A			0x00
									
///Function Definitions
void init();
void main_mcu_reset(char val);
//################################################################# Main routine

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
int main(void) {

	init();
	//eeprom_read_block(table,EEPROM_ADD,102); //EEPROM_ADD to table
	
	//eeprom_write_block(table,EEPROM_ADD,102);
	eeprom_read_block(table,EEPROM_ADD,102);		
	while (1) {
		obyte1 = byte1;
		byte1 = rxbuffer[0]; //alternating-byte
		
		//SI_REPLY_DEBUG(count_overflows);
		///Complete Reset
				if(count_overflows>DEADLOCK_THRESHOLD){
					DDRB |= 0b00000100;
					PORTB |= 0b00000100; //set PB2 to 1
					
					cli(); //Interrupts disable
					_delay_ms(50);
					main_mcu_reset(200); //secure instance as master
					current_voltage=200;
					byte4=fact_default;
					
					if(table[(curr_temp/2)].voltage!=255){ //update the table if needed
						table[(curr_temp/2)].voltage=255; //resetting the node
						table[(curr_temp/2)].osccal=byte4;
						eeprom_write_byte((int8_t)(curr_temp/2)*2,255);
						eeprom_write_byte((int8_t)(curr_temp/2)*2+1,byte4);
						SI_REPLY_DEBUG(curr_temp/2);
					} 
					
					count_overflows=0;		
					usiTwiSlaveInit(SLAVE_ADDR_ATTINY);	// secure instance as slave	
					sei(); //interrupts enable
					_delay_ms(50);
					
					PORTB &= ~(0b00000100); // set PB2 to 0
						
				}
				
		///Main slave implementation 
		if (byte1 != obyte1) {
			SI_LOCK();
			count_overflows=0;
			_delay_ms(200);
			fact_default = rxbuffer[4];
			byte2 = rxbuffer[1]; //checksum
			
			///updating values after reset
			if(byte1==2){
				SI_REPLY_DEBUG(curr_temp/2); 
				table[(curr_temp/2)].voltage=current_voltage-2; 
				table[(curr_temp/2)].osccal=byte4;
				eeprom_write_byte((int8_t)(curr_temp/2)*2,current_voltage-2);
				eeprom_write_byte((int8_t)(curr_temp/2)*2+1,byte4);
			}
			
			
			byte3 = rxbuffer[2]; //current-temp
			if(byte3==101){ //Temperature invalid
				TCNT1 = 0;
				SI_UNLOCK();
				continue;
			}
				
			byte4 = rxbuffer[3]; //current-OSCCAL
			byte5 = rxbuffer[4]; //reset_flag
			
			if(byte5==2){
				current_voltage-=2;
			}
			
			curr_temp=byte3;
		
			switch (state) {
			case SI_INIT:	//initialisation
				init();
				state = SI_TRANSIENT;
				break;
		
		
			case SI_TRANSIENT:	//transient phase				
				if (first > 0) {
					first--;
					delta_t = TCNT1;
					SI_REPLY_DELTA_T(delta_t);
				} else {
					state = SI_MAIN;
				}
				SI_REPLY_OSCCAL_FB(byte4);
				
				//current voltage thing
				if (byte2 != SI_RIGHT_CHECKSUM) {
					current_voltage -= 2; //decreases reg value but increases voltage level
				} else {
					if(++iteration > SI_ADAPTION_INTERVAL){
						current_voltage += 1;//increases reg value but decreases voltage level
						iteration = 0;
					}
				}
				SI_REPLY_VOLTAGE(current_voltage);
				prv_temp=curr_temp;
				break;

			case SI_MAIN:	//stable phase
			
				this_delta_t = TCNT1;
				SI_REPLY_DELTA_T(this_delta_t);

				if (this_delta_t > (delta_t + SI_DELTA_T_MARGIN)) {
					SI_REPLY_OSCCAL_FB(byte4+0x01);
				} else if (this_delta_t < (delta_t - SI_DELTA_T_MARGIN)) {
					SI_REPLY_OSCCAL_FB(byte4-0x01);
				} else {
					SI_REPLY_OSCCAL_FB(byte4);
				}
				
				
				/// Current_voltage
				if (byte2 != SI_RIGHT_CHECKSUM) { //Error
					current_voltage -= 2; //decreases reg value but increases voltage level
					if(current_voltage<SI_VOLT_REG_OFFSET)
					{
						current_voltage=SI_VOLT_REG_OFFSET;
						SI_REPLY_OSCCAL_FB(fact_default);
						table[(curr_temp/2)].voltage=255; //resetting the node
						table[(curr_temp/2)].osccal=byte4;
						eeprom_write_byte((int8_t)(curr_temp/2)*2,255);
						eeprom_write_byte((int8_t)(curr_temp/2)*2+1,byte4);
						SI_REPLY_DEBUG(curr_temp/2); 	
					}
					else
					{
						table[(curr_temp/2)].voltage=current_voltage; //updating table entry
						table[(curr_temp/2)].osccal=byte4;
						eeprom_write_byte((int8_t)(curr_temp/2)*2,current_voltage);
						eeprom_write_byte((int8_t)(curr_temp/2)*2+1,byte4);
						SI_REPLY_DEBUG(curr_temp/2); 
					}
					prv_temp=curr_temp;
				} 
				
				
				else { //No Error
					if(curr_temp+2<prv_temp || curr_temp>prv_temp+2){
						if(table[(curr_temp/2)].voltage!=255){
							current_voltage=table[(curr_temp/2)].voltage;
							SI_REPLY_OSCCAL_FB(table[(curr_temp/2)].osccal);
							prv_temp=curr_temp;
						}
						else{
							if(curr_temp+2<prv_temp){
								current_voltage=SI_VOLT_REG_OFFSET;
								prv_temp=curr_temp;
							}
							else if(curr_temp<prv_temp){
									table[(curr_temp/2)].voltage=current_voltage; //updating table entry
									table[(curr_temp/2)].osccal=byte4;
									eeprom_write_byte((int8_t)(curr_temp/2)*2,current_voltage);
									eeprom_write_byte((int8_t)(curr_temp/2)*2+1,byte4);
									SI_REPLY_DEBUG(curr_temp/2);
									prv_temp=curr_temp;
							}
							else if(++iteration > SI_ADAPTION_INTERVAL){
								current_voltage += 1;//increases reg value but decreases voltage level
								//Reached the minimum possible limit
								if(current_voltage>=254){
									current_voltage=254;
									table[(curr_temp/2)].voltage=current_voltage; //updating table entry
									table[(curr_temp/2)].osccal=byte4;
									eeprom_write_byte((int8_t)(curr_temp/2)*2,current_voltage);
									eeprom_write_byte((int8_t)(curr_temp/2)*2+1,byte4);
									SI_REPLY_DEBUG(curr_temp/2);
								} 
									
								iteration = 0;
							}
						}
					}
					else if(table[(curr_temp/2)].voltage!=255){
							current_voltage=table[(curr_temp/2)].voltage;
							SI_REPLY_OSCCAL_FB(table[(curr_temp/2)].osccal);
							prv_temp=curr_temp;
					}
					else if(++iteration > SI_ADAPTION_INTERVAL){
						current_voltage += 1;//increases reg value but decreases voltage level
						iteration = 0;
						//Reached the minimum possible limit
								if(current_voltage>=254){
									current_voltage=254;
									table[(curr_temp/2)].voltage=current_voltage; //updating table entry
									table[(curr_temp/2)].osccal=byte4;
									eeprom_write_byte((int8_t)(curr_temp/2)*2,current_voltage);
									eeprom_write_byte((int8_t)(curr_temp/2)*2+1,byte4);
									SI_REPLY_DEBUG(curr_temp/2);
								}
					}
				}
				
				//SI_REPLY_DEBUG(table[(curr_temp/2)].osccal);
				SI_REPLY_VOLTAGE(current_voltage);
				break;
			}

			TCNT1 = 0;
			SI_UNLOCK();
		}
	}

}

void init() {
	cli();	// Disable interrupts
	usiTwiSlaveInit(SLAVE_ADDR_ATTINY);	// TWI slave init
	byte1 = 0;
	sei();
	TCCR1A = 0;
	TCCR1B = 2; //prescaler 8
	TIMSK1|=(1<<TOIE1);
	TCNT1 = 0;
	sei();
	first = SI_STARTUP_DELAY;
	iteration = 0;
	current_voltage = SI_VOLT_REG_OFFSET;
	state = 1;
	for(int i=0;i<51;i++){ ///Initialising the table
		table[i].voltage=255;
	}
}

ISR(TIM1_OVF_vect){
	count_overflows++;
	TCNT1=0;
	/*
	///Complete Reset
				if(count_overflows>DEADLOCK_THRESHOLD){
					DDRB |= 0b00000100;
					PORTB |= 0b00000100; //set PB2 to 1
					
					cli(); //Interrupts disable
					_delay_ms(50);
					main_mcu_reset(200); //secure instance as master
					current_voltage=200;
					byte4=fact_default;
					count_overflows=0;		
					usiTwiSlaveInit(SLAVE_ADDR_ATTINY);	// secure instance as slave	
					sei(); //interrupts enable
					_delay_ms(50);
					
					PORTB &= ~(0b00000100); // set PB2 to 0
						
				}
				*/
}

void main_mcu_reset(char val){
	char i2c_transmit_buffer[3];
	char i2c_transmit_buffer_len = 3;

	i2c_transmit_buffer[0] = AD5242_DEV_ADDR_W ;//Write Slave Address

	i2c_transmit_buffer[1] = AD5242_CHN_A;  //Internal address

	i2c_transmit_buffer[2] = val;  //Value to write

	//Transmit the I2C message
	USI_I2C_Master_Transceiver_Start(i2c_transmit_buffer,i2c_transmit_buffer_len);
}
