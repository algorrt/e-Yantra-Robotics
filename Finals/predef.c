 /*
 *
 * Team Id: eYRC#1348-CA
 * Author List: 
  * Filename: predef.c
 * Theme: Cargo Alignment
 * Functions: ADC_Conversion, adc_init, adc_pin_config, angle_rotate, arm, back_mm, buzzer_off, buzzer_on, buzzer_pin_config, CurPos, init_devices, lcd_port_config, left_degrees, left_encoder_pin_config, linear_distance_mm, MOSFET_switch_config, motion_pin_config, motion_set, port_init, print_sensor, right_degrees, right_encoder_pin_config, servo_1, servo_2, servo1_pin_config, servo2_pin_config, Sharp_Detection, Sharp_GP2D12_estimation, spi_master_tx_and_rx, stop, timer1_init, timer5_init, velocity
 * Global Variables: ShaftCountLeft, ShaftCountRight, Degrees, ADC_Conversion, ADC_Value,flag, Left_white_line, Center_white_line, Right_white_line, Left_Sharp_Sensor, Right_Sharp_Sensor, distance, adc_reading, Sensor_read, BATT_Voltage, BATT_V, Blk_Pos[7][5], Cur_Pos[2];
 */
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include <stdlib.h>
#include "lcd.c"
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();


volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag=0;
unsigned char Left_white_line = 0; 
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char distance, adc_reading;
volatile unsigned char Sensor_read;

volatile unsigned short Blk_Pos[7][5] =	{{0,0,0,0,0},		//7x5 Array for storing block locations and orientation for both D1 and D2
									{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0},
									{0,0,0,0,0}};
volatile unsigned short int Cur_Pos[2]={2,0};				//Keeps track of current position
unsigned short int count1=0;								//Counter for D1
unsigned short int count2=0;								//Counter for D2

/*
*
* Function Name: CurPos
* Input: fwd, bk, r and l=> integers which update Cur_Pos accordingly
* Output: void
* Logic: Updates robot's current position as it traverses the arena during cargo alignment, giving proper mnagement.  
* Example Call: CurPos(1,0,0,0); //Increments Cur_Pos[1] by 1.
*
*/	
void CurPos(int fwd, int bk, int r, int l)
{
		if(fwd!=0)
		{Cur_Pos[1]+=fwd;}
		if(bk!=0){Cur_Pos[1]-=bk;}
		if(r!=0){Cur_Pos[0]-=r;}
		if(l!=0)
		{Cur_Pos[0]+=l;}		
}

/*
*
* Function Name: servo1_pin_config
* Input: void
* Output: void
* Logic: Makes PORTB 5 pin as output and sets it to logic 1.
* Example Call: servo1_pin_config();
*
*/				
void servo1_pin_config (void)
{
	DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
	PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*
*
* Function Name: servo2_pin_config
* Input: void
* Output: void
* Logic: Makes PORTB 6 pin as output and sets it to logic 1.
* Example Call: servo2_pin_config();
*
*/	
//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation

//Function to configure ports to enable robot's motion
/*
*
* Function Name: motion_pin_config
* Input: void
* Output: void
* Logic: initializes ports for PWM generation and application.
* Example:motion_pin_config();
*
*/	
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*
*
* Function Name: adc_pin_config
* Input: void
* Output: void
* Logic: Initializes ports for analog to digital conversion for further processing required by the sensors.
* Example Call: adc_pin_config();
*
*/	
void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

/*
*
* Function Name: velocity
* Input: datatype "int" parameters as required to provide velocity magnitude allocations for required movement.
* Output: void => Sets the left and right motor velocities to those passed. 
* Logic: Uses PWM to set the motor velocities to those specified.
* Example Call: velocity(215,215);
*
*/	
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/*
*
* Function Name: timer5_init
* Input: void
* Output: void
* Logic: Timer 5 initialized in PWM mode for velocity control of DC Motors, Prescale:256, PWM 8bit fast, TOP=0x00FF, Timer Frequency:225.000Hz. Timer 5 can be used for PWM generation for controlling speed of motors. The duty cycle of square wave generated by the Timer5 can be varied to produce different average DC values for motors.
* Example Call: timer5_init();
*
*/	
void timer5_init(void)
{
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


/*
*
* Function Name: timer1_init
* Input: void
* Output: void
* Logic: TIMER1 initialization in 10 bit fast PWM mode, prescale:256, WGM: 7 PWM 10bit fast, TOP=0x03FF, Actual value: 52.25Hz. Timer 1 can be used for PWM generation for controlling speed of motors. The duty cycle of square wave generated by the Timer5 can be varied to produce different average DC values for motors.
* Example Call: timer1_init();
*
*/	
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*
*
* Function Name: buzzer_on
* Input: void
* Output: void => Turns the buzzer on
* Logic: When called, initializes port pin C to switch ON buzzer.
* Example Call: buzzer_on();
*
*/	
void buzzer_on(void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}


/*
*
* Function Name: buzzer_off
* Input: void
* Output: void
* Logic: When called , switches OFF buzzer , if already in ON state , by manipulating pin C.
* Example Call: buzzer_off();
*
*/	
void buzzer_off(void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.

/*
*
* Function Name: Sharp_GP2D12_estimation
* Input: unsigned char adc_reading  
* Output: int
* Logic: Reads non converted value as parameter and returns converted value so as for easy manipulation and apllication.
* Example Call: Sharp_GP2D12_estimation()
*
*/	
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

/*
*
* Function Name: adc_init
* * Input: void
* Output: void
* Logic: Initializes ADC
* Example Call: adc_init();
*
*/	
void adc_init(void)
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*
*
* Function Name:
* Input: ch => an unsigned character for accepting channel number
* Output: unsigned char
* Logic: Converts analog data from the peripherals to digital data for processing by microcontroller.
* Example Call: unsigned char n;
				n=ADC_Conversion(11); //Stores value from front SHARP sensor in variable n.
				
*
*/
//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch)
{
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


/*
*
* Function Name: print_sensor
* Input: (char,char,char)=> co-ordinates on the LCD with the channel name whose values need to be displayed.
* Output: <Return value with description if any>
* Logic: This function prints the function needed to be displayed on the LCD.
* Example Call: print_sensor(1,1,7);
*
*/	
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/*
*
* Function Name: left_encoder_pin_config
* Input: void
* Output: void
* Logic:Initializes left encoder pin configuration for counting/useful for keeping track of the robots movements. 
* Example Call: left_encoder_pin_config();
*
*/	
//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{ 
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

/*
*
* Function Name: right_encoder_pin_config
* Input: void
* Output: void
* Logic: Initializes right encoder pin configuration for counting/useful for keeping track of the robots movements.
* Example Call: right_encoder_pin_config();
*
*/	
//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
//Function to initialize Buzzer

/*
*
* Function Name: buzzer_pin_config
* Input: void
* Output: void
* Logic: Initializes port C for buzzer application.
* Example Call: buzzer_pin_config();
*
*/	
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}
/*
*
* Function Name: lcd_port_config
* Input: void
* Output: void
* Logic: Initializes LCD ports useful for depicting values on the display by accessing port C.
* Example Call: lcd_port_config();
*
*/	
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}
/*
*
* Function Name: MOSFET_switch_config
* Input: void
* Output: void
* Logic: Useful to switch ON and switch OFF sensors for better energy management.
* Example Call: MOSFET_switch_config();
*
*/	
void MOSFET_switch_config (void)
{
	DDRH = DDRH | 0x0C; //make PORTH 3 and PORTH 1 pins as output
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0

	DDRG = DDRG | 0x04; //make PORTG 2 pin as output
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}

/*
*
* Function Name: port_init
* Input: void
* Output: void
* Logic: Initializes all the given (required ) functions . 
* Example Call: port_init();
*
*/	
//Function to initialize ports
void port_init()
{	buzzer_pin_config();// for buzzer
	lcd_port_config();// for lCD
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	adc_pin_config();
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
	
	MOSFET_switch_config();
}

/*
*
* Function Name: left_position_encoder_interrupt_init
* Input: void
* Output: void
* Example Call: left_position_encoder_interrupt_init();
*
*/	
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}
/*
*
* Function Name: right_position_encoder_interrupt_init
* Input: void
* Output: void
* Example Call: right_position_encoder_interrupt_init();
*
*/	
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}
/*
*
* Function Name: ISR
* Logic: Gets called whenever position encoder counts increment on right motor.
*
*/	
//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}

/*
*
* Function Name: <Function Name>
* Logic: Gets called whenever position encoder increments a count on the left motor.
* Example Call: <Example of how to call this function>
*
*/	
//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

/*
*
* Function Name: motion_set
* Input: <Inputs (or Parameters) list with description if any>
* Output: void
* Logic:	Direction 	PA(0)	PA(1)	PA(2)	PA(3)
						LB		LF		RF		RB
			Forward 	0		1		1 		0
			Backward	1		0		0 		1
			Left 		1 		0 		1 		0
			Right 		0 		1 		0 		1
			Soft Left 	0 		0 		1 		0
			Soft Right 	0 		1 		0 		0
			Stop 		0 		0 		0 		0
* 			
* Example Call: motion_set(0x06); //Sets motion to forward
*
*/	

void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}
/*
*
* Function Name: init_devices
* Input: void
* Output: void
* Logic: This function is used to initialize all the devices.
* Example Call: init_devices();
*/	

void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	timer5_init();
	timer1_init();
	adc_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
}

/*
*
* Function Name: turn_on_sharp234_wl
* Input: void
* Output: void
* Logic: Turns on Sharp IR range sensors 2, 3, 4 and white line sensor's red LED by logical anding of PORTG's contents with FB hexadecimal and setting PORTG to the resulting value.
* Example Call: turn_on_sharp234_wl();
*
*/	
void turn_on_sharp234_wl (void)
{
	PORTG = PORTG & 0xFB;
}
/*
*
* Function Name: turn_off_sharp234_wl
* Input: void
* Output: void
* Logic: Turns off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED by logical anding of PORTG's contents with FB hexadecimal and setting PORTG to the resulting value.

* Example Call: turn_off_sharp234_wl
*
*/	
void turn_off_sharp234_wl (void) //turn off Sharp IR range sensors 2, 3, 4 and white line sensor's red LED
{
	PORTG = PORTG | 0x04;
}
/*
*
* Function Name: turn_on_sharp15
* Input: void
* Output: void
* Logic: Turns on Sharp IR range sensors 1,5 by logical anding of PORTH's contents with FB hexadecimal and setting PORTH to the resulting value.
* Example Call: turn_on_sharp15();
*
*/	
void turn_on_sharp15 (void) 
{
	PORTH = PORTH & 0xFB;
}
/*
*
* Function Name: turn_off_sharp15
* Input: void
* Output: void
* Logic: Turns on Sharp IR range sensors 1,5 and white line sensor's red LED by logical anding of PORTG's contents with FB hexadecimal and setting PORTG to the resulting value.
* Example Call: turn_off_sharp15();
*
*/	
void turn_off_sharp15 (void) //turn off Sharp IR range sensors 1,5
{
	PORTH = PORTH | 0x04;
}
/*
*
* Function Name: turn_on_ir_proxi_sensors
* Input: void
* Output: void
* Logic: turn on all IR proximity sensor for further detection or application.
* Example Call: turn_on_ir_proxi_sensors();
*
*/	
void turn_on_ir_proxi_sensors (void) //turn on IR Proximity sensors
{
	PORTH = PORTH & 0xF7;
}
/*
*
* Function Name: turn_off_ir_proxi_sensors
* Input: void
* Output: void
* Logic: turns off IR proximity sensor for further application.
* Example Call: turn_off_ir_proxi_sensors();
*
*/	
void turn_off_ir_proxi_sensors (void) //turn off IR Proximity sensors
{
	PORTH = PORTH | 0x08;
}
/*
*
* Function Name: turn_on_all_proxy_sensors
* Input: void
* Output: void
* Logic: turns all IR proximity sensor ON.
* Example Call: turn_on_all_proxy_sensors();
*
*/	
void turn_on_all_proxy_sensors (void) // turn on Sharp 2, 3, 4, red LED of the white line sensors
// Sharp 1, 5 and IR proximity sensor
{
	PORTH = PORTH & 0xF3; //set PORTH 3 and PORTH 1 pins to 0
	PORTG = PORTG & 0xFB; //set PORTG 2 pin to 0
}
/*
* Function Name:	 twist
* Input:			 x and y -> integer.Actually stores the even and odd values.  
* Output:			 Servo turns through 180 degrees and value of x is incremented by 1	 
* Logic:			 this function consists of two sections,first section makes the servo rotate from 0 to 180 degrees and in second section
*					 it makes the servo rotate back from 180 to 0 degree.By using odd and even number concept both the sections are called alternately.
* Example Call:		 x=twist(x);
*
*/
/*
*
* Function Name: servo_1
* Input: degrees => unsigned character which specifies the number of degrees to be rotated.
* Example Call: void servo_1(90); //Rotates servo_1  by 90 degrees
*
*/	
//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
	float PositionPanServo = 0;
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionPanServo;
}

/*
*
* Function Name: servo_2
* Input: degrees => unsigned character which specifies the number of degrees to be rotated.
* Output: void
* Example Call: void servo_2(90); //Rotates servo_2  by 90 degrees
*
*/	
//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.
/*
*
* Function Name: servo_1_free
* Input: void
* Output: void
* Logic: frees servo 1 motors.
* Example Call: servo_1_free();
*
*/	
void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}
/*
*
* Function Name: servo_2_free
* Input: void
* Output: void
* Logic: frees servo2 motors.
* Example Call: servo_2_free();
*
*/	
void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}
/*
*
* Function Name: forward
* Input: void
* Output: void
* Logic:	Direction 	PA(0)	PA(1)	PA(2)	PA(3)
*						LB		LF		RF		RB
*			Forward 	0		1		1 		0
* Example Call: forward();
*
*/	
void forward (void) //both wheels forward
{
	motion_set(0x06);
}
/*
*
* Function Name: back
* Input: void
* Output: void
* Logic:	Direction 	PA(0)	PA(1)	PA(2)	PA(3)
						LB		LF		RF		RB
			Backward 		1		0		0 		1
* Example Call: back();
*
*/	
void back (void) //both wheels backward
{
	motion_set(0x09);
}
/*
*
* Function Name: left
* Input: void
* Output: void
* Logic:	Direction 	PA(0)	PA(1)	PA(2)	PA(3)
						LB		LF		RF		RB
			Left 		0		1		0 		1
* Example Call: left();
*
*/	
void left(void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}
/*

* Function Name: rightA
* Input: void
* Output: void
* Logic:	Direction 	PA(0)	PA(1)	PA(2)	PA(3)
						LB		LF		RF		RB
			Right	 	1		0		1 		0
* Example Call: right();
*/	
void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}
/*
*
* Function Name: stop
* Input: void
* Output: void
* Logic:	Direction 	PA(0)	PA(1)	PA(2)	PA(3)
						LB		LF		RF		RB
			Stop	 	0		0		0 		0
* Example Call: stop();
*
*/	
void stop (void)
{
	motion_set(0x00);
}
/*
*
* Function Name: linear_distance_mm
* Input: DistanceInMM => unsigned integer which specifies the distance in millimeters.
* Output: void
* Logic: This function used for moving robot forward by the specified distance in millimeters Counts ShaftCount of 
* Example Call: linear_distance_mm(200);
*
*/	

void linear_distance_mm(unsigned int DistanceInMM)
{
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
/*
*
* Function Name: angle_rotate
* Input: Degrees => unsigned integer which specifies the number of degrees to be rotated.
* Output: void
* Logic: Rotates by the angle specified using position encoders
* Example Call: left(); //Turn left
*				velocity(205,200);
*				angle_rotate(90);
*				//Turns left by 90 degrees
*
*/	
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

/*
*
* Function Name: forward_mm
* Input: DistanceInMM => unsigned integer which specifies the distance in millimeters.
* Output: void
* Logic: Goes forward by the distance specified using position encoders.
* Example Call: forward_mm(200) // Goes forward by 200 millimeters.
*
*/	
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	velocity(205,200);
	linear_distance_mm(DistanceInMM);
	CurPos(DistanceInMM/200,0,0,0);
}
/*
*
* Function Name: back_mm
* Input: DistanceInMM => unsigned integer which specifies the distance in millimeters.
* Output: void
* Logic: Goes forward by the distance specified using position encoders.
* Example Call: back_mm(200) // Goes backward by 200 millimeters.
*
*/	
void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
	CurPos(0,DistanceInMM/200,0,0);
}
/*
*
* Function Name: left_degrees
* Input: Degrees => unsigned integer which specifies the number of degrees to be rotated.
* Output: void
* Logic: Rotates left (Takes a hard-left turn) by the specified number of degrees using position encoders.
* Example Call: left_degrees(45); //Rotates left by 45 degrees
*
*/	
void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	velocity(205,200);
	angle_rotate(Degrees);
}
/*
*
* Function Name: right_degrees
* Input: Degrees => unsigned integer which specifies the number of degrees to be rotated.
* Output: void
* Logic: Rotates right  (Takes a hard-right turn) by the specified number of degrees using position encoders.
* Example Call: right_degrees(45); //Rotates left by 45 degrees
*
*/	
void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	velocity(205,200);
	angle_rotate(Degrees);
}
