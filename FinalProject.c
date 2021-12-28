/*
===============================================================================
 Name        : Final_Project.c
 Author      : $ Amilton Pensamento & Derek Stahl
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here

#define PCONP (*(volatile unsigned int *) 0x400FC0C4)	//Power control for peripherals
#define CCR (*(volatile unsigned int *) 0x40024008)	//Clock control register
#define CIIR (*(volatile unsigned int *) 0x4002400C) //Counter increment interrupt register
#define CTIME0 (*(volatile unsigned int *) 0x40024014)	//Consolidated Time register
#define CALIBRATION (*(volatile unsigned int *) 0x40024040)	//Calibration register
#define SEC (*(volatile unsigned int *) 0x40024020)	//SECOND register that is also able to be written to
#define MIN (*(volatile unsigned int *) 0x40024024)	//MIN register that is also able to be written to
#define HOUR (*(volatile unsigned int *) 0x40024028)	//HOUR register that is also able to be written to

#define AMR	(*(volatile unsigned int *) 0x4002 4010)	//Alarm mask register, prevents certain alarm registers from comparing values
#define ALSEC (*(volatile unsigned int *) 0x40024060)	//Alarm register for the seconds, generates an interrupt
#define ALMIN (*(volatile unsigned int *) 0x40024064)	//Alarm register for the minutes, generates an interrupt
#define ALHOUR (*(volatile unsigned int *) 0x40024068)	//Alarm register for the hours, generates an interrupt

#define ISER0 (*(volatile unsigned int *) 0xE000E100)	//set enable register 0
#define PCLKSEL0 (*(volatile unsigned int *) 0x400FC1A8) //Peripheral Clock selection

#define T0TCR (*(volatile unsigned int *) 0x40004004)	//
#define T0TC (*(volatile unsigned int *) 0x40004008)	//

#define PWM1LER (*(volatile unsigned int*) 0x40018050)//Latch enable register for PWM
#define PWM1MR0 (*(volatile unsigned int*) 0x40018018)//PWM Match Register 0 for PWM1. Can be used to control the frequency
#define PWM1MR1 (*(volatile unsigned int*) 0x4001801C)//PWM Match Register 1 for PWM1. Can be used to control the duty cycle
#define PWM1MCR (*(volatile unsigned int*) 0x40018014)//PWM Match Control Register
#define PWM1PCR (*(volatile unsigned int*) 0x4001804C)//PWM Control Register
#define PWM1TCR (*(volatile unsigned int*) 0x40018004)//PWM timer control register
#define PINSEL4 (*(volatile unsigned int*) 0x4002C010)//Pin select register to set one of the pins as an output for pulse width modulation
#define PINSEL1 (*(volatile unsigned int*) 0x4002C004)//Pin select register that we are using to select AOUT on P0.26 (set 10 to bits 21 and 20) and AIN(set 01 to bit 14 and 15)
#define PWM1IR (*(volatile unsigned int*) 0x40018000)//PWM Match Register 0 for PWM1. Can be used to control the frequency
#define DACR (*(volatile unsigned int*) 0x4008C000)//D to A converter register

#define I2C0SCLH (*(volatile unsigned int *) 0x4001C010) //High duty cycle for clock
#define I2C0SCLL (*(volatile unsigned int *) 0x4001C014) //Low duty cycle time for clock
#define I2C0CONSET (*(volatile unsigned int *) 0x4001C000)  //This is the control set register
#define I2C0CONCLR (*(volatile unsigned int *) 0x4001C018) //This is the control clear register
#define I2C0DAT (*(volatile unsigned int *) 0x4001C008) //This is the I2C Data register
#define I2C0STAT (*(volatile unsigned int *) 0x4001C004) //This is the I2C Status register

#define FIO0DIR (*(volatile unsigned int *) 0x2009C000)	//direction register for port 0
#define FIO0PIN (*(volatile unsigned int *) 0x2009C014)	//Pin register for port 0

#define IODIRA 0x00 //This IODIRA register
#define IODIRB 0x01 //This IODIRB register
#define GPIOA 0x12 //input output register for port a
#define GPIOB 0x13 //input output register for port b

int io_opcode = 0b0100000;
int amplitude = 1023;

//TODO Set PCLK value to 100kHz somewhere in the main function for the io expander

void wait_ms(float ms)
{
  int us = ms*1000;
  int start = T0TC;  // note starting time
  T0TCR |= (1<<0);   // start timer
  while ((T0TC-start)<us) {} // wait for time to pass
}


void PWM1_IRQHandler(void) {
	// automatically calls and sets DACR value

	//PWM1IR = (1<<0); // MRO - rising edge
	//PWM1IR = (1<<1); // MR1 - falling edge

	if((PWM1IR>>0) & 1){
		DACR = (amplitude<<6);
		PWM1IR = (1<<0);
	} else if((PWM1IR>>1) & 1){
		DACR = (0<<6);
		PWM1IR = (1<<1);
	}


}


void pwm_init(int period){
	PCONP |= (1<<6);	//enable power for pwm1

	// Initialize PWM to 4 MHz / 6069.80 = 659Hz
	PCLKSEL0 |= (1<<12); // PWM PCLK = CCLK/1

	PWM1MR0 = period; // 6069.80 PCLK cycle period (8-bit equivalent)
	PWM1MR1 = period/2;	// 6070/2-1 - --3035 PCLK rising edge = falling edge (50% duty cycle)
	PWM1LER = (0b11<<0); // set bit 0 to use new MR0 and MR1 value

	PWM1MCR = (1<<1); // Reset counter on MR0 match
	PWM1PCR = (1<<9); // Single edge mode and enable PWM1.1 only
	PWM1TCR = (1<<0) | (1<<3); // Enable counter and PWM mode

	PWM1MCR |= (1<<0); // MCR - enable MRO
	PWM1MCR |= (1<<3); // MCR - enable MR1

	ISER0 |= (1<<9); // enable PWM IR
}


void start(){
//START CONDITION
                                                                                                     I2C0CONSET = (1<<3);// This sets the SI bit in the control register to 1
I2C0CONSET = (1<<5);//This sets the start bit to 1 in the control register to 1
I2C0CONCLR = (1<<3);//This clears the SI bit

while(((I2C0CONSET>>3) & 1) != 1){   //do nothing while SI bit is 0
}

I2C0CONCLR = (1<<5);//This clears the start bit
//START CONDITION FINISHED
}



//this is the small write function
void write_data(int data){

I2C0DAT = data;
I2C0CONCLR = (1<<3);//This clears the SI bit

while(((I2C0CONSET>>3) & 1) != 1){   //do nothing while SI bit is 0
}

}


//this is the small read function
int read_data(){

I2C0CONCLR = (1<<2);//This clears the assert acknowledge bit in the Control register
I2C0CONCLR = (1<<3);//This clears the SI bit in the control register

while(((I2C0CONSET>>3) & 1) != 1){   //do nothing while SI bit is 0
}

int data_read = I2C0DAT;//assign the values in the data register to data_read for later
return data_read;
}

//This is the stop function
void stop(){

I2C0CONSET = (1<<4);//This stops the control register
I2C0CONCLR = (1<<3);//This clears the SI bit

while(((I2C0CONSET>>1) & 1) == 1){   //wait for stop bit to go to zero
}

}

void write_seq(int OP, int ADDR, int Din) {
start(); //start condition from initiator
int W = 0;
int OPW = (OP<<1)|W; // 8 bits, OPW is step two in the write sequence. In start sequence of write sequence

write_data(OPW);
write_data(ADDR);
write_data(Din);
stop();
}


//reads data from whatever device selected
int read_seq(int OP, int ADDR){
start(); //start condition
int OPR;
int R = 1;
int W = 0;
int OPW = (OP<<1)|W;
write_data(OPW);
write_data(ADDR);
start();
OPR = (OP<<1)|R;
write_data(OPR);
int data = read_data();
stop();
return data;
}

void LCD_write_command(int command){

	write_seq(io_opcode, GPIOB, command);

	FIO0PIN &= ~(1<<22);	//RS for display is set to low
	FIO0PIN |= (1<<21);	//Enable for display is set to high
	FIO0PIN &= ~(1<<21);	//Enable for display is set to low

	wait_ms(0.1);	//wait 100 microseconds
}


void LCD_init(){

	FIO0DIR |= (1<<22);	//sets E and RS as outputs
	FIO0DIR |= (1<<21);

	FIO0PIN &= ~(1<<22);	//RS for display is set to low
	FIO0PIN &= ~(1<<21);	//Enable for display is set to low

	write_seq(io_opcode, IODIRB, 0b00000000);//everything on port b on IOExpander is an output

	wait_ms(4);

	LCD_write_command(0x38);	//this selects using the full 8 bit bus and a 2 line display
	LCD_write_command(0x06);	//this selects automatically using the cursor
	LCD_write_command(0x0c);	//this enables the display and displays no cursor

	LCD_write_command(0x01);	//clears the display

	wait_ms(4);
}

void LCD_write_data(int data){

	write_seq(io_opcode, GPIOB, data);	//load data of ASCII code for character

	FIO0PIN |= (1<<22);	//RS for display is set to high
	FIO0PIN |= (1<<21);	//Enable for display is set to high
	FIO0PIN &= ~(1<<21);	//Enable for display is set to low

	wait_ms(0.1);	//wait 100 microseconds
}

void LCDwriteString(char * buff) {
	// when string completes it holds 0 in the last place
	for (volatile int idx = 0; buff[idx] != 0; idx++) {
		LCD_write_data(buff[idx]);
	}
}

int time[6] = {0,0,0,0,0,0};
char time2[6];

//converts
char* time_to_char(int sec, int min, int hours){

	time2[4] = (sec/10) + '0';
	time2[5] = (sec%10) + '0';

	time2[2] = (min/10) + '0';
	time2[3] = (min%10) + '0';

	time2[0] = (hours/10) + '0';
	time2[1] = (hours%10) + '0';

	return time2;
}

void LCDwriteString2(char * str, int num_elem) {
	// when string completes it holds 0 in the last place

	//writes the hours
	for(volatile int idx = 0; idx < 2; idx++){
		LCD_write_data(str[idx]);
	}

	LCD_write_data(':');

	//writes the minutes
	for (volatile int idx = 2; idx < 4; idx++) {
		LCD_write_data(str[idx]);
	}

	LCD_write_data(':');

	//writes the seconds
	for (volatile int idx = 4; idx < num_elem; idx++) {
		LCD_write_data(str[idx]);
	}
}

//-------------------------------------------

//This sets up the oscillator to the correct time
void RTC_setup(int *clocktime) {
	// Disable the clock
	CCR = ~(1<<0);
	// reset the clock
	CCR = (1<<1);
	// Assign the time to clock
	HOUR = clocktime[0]; MIN = clocktime[1];SEC = clocktime[2];
	// re-enable the clock
	CCR = (1<<0);
}










int read_keypad_switches() {

	// set all the pins to pull-down
	// array of row bits set as outputs
	int dir_arr[4] = {0b11111110, 0b11111101, 0b11111011, 0b11110111};
	// array of row bits set to high
	int pin_arr[4] = {0b00000001, 0b00000010, 0b00000100, 0b00001000};
	int idx=0;

	//Scan rows
	for (volatile int rows=0; rows<4; rows++){

		//set all rows to inputs -- IODIR -- write
		write_seq(io_opcode, IODIRA, 0b11111111);

		//set curr_row to output -- IODIR -- write
		write_seq(io_opcode, IODIRA, dir_arr[rows]);

		//set curr_row to high -- GPIOA -- write
		write_seq(io_opcode, GPIOA, pin_arr[rows]);

		//scan columns - cols always input
		int read_data = read_seq(io_opcode, GPIOA); // 8-bits
		for (volatile int cols=4; cols<8; cols++){
			int current_col = (((read_data>>cols) & 1) == 1); // read current column value
			// if current_col is high -- switch is pressed in this column -- & row
			if (current_col == 1) {
				while (current_col == 1) {
					read_data = read_seq(io_opcode, GPIOA); // 8-bits
					current_col = (((read_data>>cols) & 1) == 1); // read current column value
				}
				return idx; // only if switch is pressed (0-15)
			}
			idx++;
		}
	}
	return -1; // no switch is pressed
}




// Mapping the index in the read_keypad_switches() to the arrays bellow
// keypad numeric keys
int num_keys[10] = {8,15,11,7,14,10,6,13,9,5};
// keypad alpha keys
int alpha_keys[6] = {3,2,1,0,12,4};

int keypad_numbers[16] = {-5, -5, -5, -5, -5, 9, 6, 3, 0, 8, 5, 2, -5, 7, 4, 1};	//how the keypad values are accessed in the correct index position

//this function uses the read keypad switches functions in do while loops to wait until a button has been pressed
int get_keypad(){

	int tempkeypadread;
	int oneValue;
	int tenValue;


		//read the switches while the switches have not been pressed
		do {
			tempkeypadread = read_keypad_switches();
		} while (tempkeypadread == -1 );


		tenValue = keypad_numbers[tempkeypadread] * 10;		//change the first value read to a power of 10 so we can add with the next value later
		LCD_write_data(keypad_numbers[tempkeypadread] + '0');	//write the number typed to the display
		wait_ms(500);

		do {
			tempkeypadread = read_keypad_switches();
			} while (tempkeypadread == -1 );

			oneValue = keypad_numbers[tempkeypadread];
			LCD_write_data(keypad_numbers[tempkeypadread] + '0');
			wait_ms(500);






		return tenValue + oneValue;
	}






int alarm_time[3] = {0,0,0};
int alarm_flg = 0;

void set_alarm(void) {
	int counts = 0;	//tracks how many numbers have been entered into for the alarm
	int alarm_array[6] = {0,0,0,0,0,0};	//stores the values of the alarm
	int LCD_pos_arr[6] = {0X80+0X20, 0X80+0X21, 0X80+0X23, 0X80+0X24, 0X80+0X26, 0X80+0X27};

	LCD_write_command(0x94);
	LCDwriteString("ALARM: ");

	LCD_write_command( 0X80+0X22); LCD_write_data(':');
	LCD_write_command( 0X80+0X25); LCD_write_data(':');
	//wait_ms(300);
	while (counts < 6) {
		display_clk();
		display_elap(1);
		int key = read_keypad_switches();	//read from the keypad
		if (key == 15 || key == 11|| key == 7|| key == 14|| key == 10
				|| key == 6|| key == 13|| key == 9|| key == 5|| key == 8) {

			// pressed number key
			alarm_array[counts] = keypad_numbers[key];	//uses keypad_numbers function which stores the correct values that correspond to the keypad indexes and stores them in alarm_array
			LCD_write_command(LCD_pos_arr[counts]); // MOVE THE CURSOR
			LCD_write_data(keypad_numbers[key] + '0');	//show on the display what was pressed
			counts++;
		}
	}

	// it will have 6 numbers in alarm_array
	alarm_time[0] = (alarm_array[0] * 10) + alarm_array[1];	//alarm hours
	alarm_time[1] = (alarm_array[2] * 10) + alarm_array[3];	//alarm minutes
	alarm_time[2] = (alarm_array[4] * 10) + alarm_array[5];	//alarm seconds
	alarm_flg = 1; // alarm is set --  enable
}

int start_alarm = 0;
void check_alarm() {
	if (HOUR == alarm_time[0] && MIN == alarm_time[1] && SEC == alarm_time[2]) {
		// play the alarm;
		amplitude = 1023;
		alarm_flg = 0; // disable
		start_alarm = 1;
		pwm_init(6070);
		// clear the LCD alarm time
	}
}


int clk_strt = 0;
//these store the initial values of the clock to be compared later
int elap_hour;
int elap_min;
int elap_sec;


void display_elap(int restart){


	int curr_hour;
	int curr_min;
	int curr_sec;
	int elap_curr;

	int actual_sec;
	int actual_hour;
	int actual_min;
	int rem_sec;

	//stores the current times
	if(restart == 0){
		elap_hour = HOUR;
		elap_min = MIN;
		elap_sec = SEC;
	}

	//the starting time stored in seconds
	int elap_start = (3600 * elap_hour) + (elap_min * 60) + elap_sec;	//elapsed start time



	curr_hour = HOUR;
	curr_min = MIN;
	curr_sec = SEC;

	elap_curr = (3600 * curr_hour) + (curr_min * 60) + curr_sec;	//elapsed current time

	actual_sec = elap_curr - elap_start;

	actual_hour = actual_sec/3600;
	actual_min = actual_sec/60;
	rem_sec = actual_sec%60;

	LCD_write_command(0xC0);
	LCDwriteString("ELAP TIME: ");
	LCD_write_command(0xCC);
	LCDwriteString2(time_to_char(rem_sec, actual_min, actual_hour), 6);


}

void display_clk(){

	LCD_write_command(0x80);
	LCDwriteString("CLOCK TIME: ");
	LCDwriteString2(time_to_char(SEC, MIN, HOUR), 6);
}

int main(void) {

	PINSEL1 &= ~(1<<23);	//This sets bit 22 and 23 in the pin select register 1
	PINSEL1 |= (1<<22);		//to 1 and 0 respectively (SDA0)
	PINSEL1 &= ~(1<<25); 	//This sets bit 24 and 25 in the pin select register 1
	PINSEL1 |= (1<<24);  	//to 1 and 0 respectively (SCL0)

	PINSEL1 |= (1<<21);		//Port 0 bit 26 as AOUT
	PINSEL4 |= (1<<0); // Configure P2.0 as PWM1.1

	I2C0SCLL = 5; 			//this sets the duty cycle to be 50% and the frequency 100KHz in conjunction with
	I2C0SCLH = 5; 			//each other

	I2C0CONCLR = (1<<6); //This clears enable the I2C0 register
	I2C0CONSET = (1<<6); //This set enable the I2C0 register

	write_seq(io_opcode, IODIRA, 0b11111111); //Setting IOexpander GPIOA as inputs

	LCD_init();	//initialize the LCD

	//THIS IS THE END OF INITIALIZATION

	LCD_write_command(0x80);//moves the cursor to the top left
	wait_ms(1);

	LCDwriteString("SET CLOCK:  ");
	//LCDwriteString2(time2, 6);

	int clocktime[3] = {0, 0, 0};	//this array sets the clock time
	clocktime[0] = get_keypad();
	LCD_write_data(':');

	clocktime[1] = get_keypad();
	LCD_write_data(':');
	clocktime[2] = get_keypad();


	RTC_setup(clocktime);			//actual setup of clock time

	LCD_write_command(0x80);//moves the cursor to the top left
	wait_ms(1);
	LCDwriteString("CLOCK TIME: ");

 	while(1){
// 		elapsed_time(0);
 		/*int sec = x & (0b111111);
 		int min = (x>>8) & (0b111111);
 		int hours = (x>>16) & (0b11111);*/
 		// displayTime();
 		LCD_write_command(0x8C);

 		LCDwriteString2(time_to_char(SEC, MIN, HOUR), 6);


 		int key_pad_idx = read_keypad_switches();
 		display_clk();

 		if (key_pad_idx == 3) {
 			set_alarm();
 			display_elap(1);
 		}

 		else if(key_pad_idx == 4 ){
 			display_elap(0);
 		}

 		else if(key_pad_idx == 12){
 			clk_strt = !clk_strt;
 		}

 		else{
 			;
 		}


 		//stop the elapsed time clock and write zeroes to the display in its place
 		if(clk_strt == 0){

 			LCD_write_command(0xC0);
 			LCDwriteString("ELAP TIME: ");
 			LCD_write_command(0xCC);
 			display_elap(0);
 		}

 		//start the elapsed time clock back up at 0
 		else if(clk_strt == 1){
 			display_elap(1);

 		}

 		//display elapsed time
 		else{
 			display_elap(1);//do nothing if a non valid key is pressed
 		}


 		if  (alarm_flg == 1) {
 			check_alarm();
 		}



 		if(start_alarm == 1){
 			amplitude = amplitude - 20;
 		}

 		if(amplitude <= 0){
 			amplitude = 0;
 			start_alarm = 0;
 		}


 	}
    return 0;
}
