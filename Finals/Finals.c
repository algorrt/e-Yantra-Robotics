 /*
 *
 * Team Id: eYRC#1348-CA
 * Author List:  Rajanikant Tenguria, Nilesh Modak, Rishikesh Madan, Saurabh Parkhedkar, Sachitanand Malewar, NEX ROBOTICS PVT LTD.
 * Filename: final.c
 * Theme: Cargo Alignment
 * Functions: arm, exit1, exit2, IRCorridor, left_wls, lf, lift, main, rf, right_wls, Sharp_Detection,  Start, SharpSide, twist
 * Global Variables: None
 */
#include "predef.c"

				
/*
*
* Function Name: Sensor
* Input: void
* Output: void
* Logic: Checks value of specified SHARP sensor and matches it to the node location and placement position accordingly. 
* Example Call: Sensor(10);
*
*/	

void Sensor(unsigned char a)
{
	unsigned char value;
	Sensor_read = ADC_Conversion(a);
	value = Sharp_GP2D12_estimation(Sensor_read);				//Stores Distance calculated in a variable "value".
	uint8_t tmp=Cur_Pos[1]+1;
	uint8_t tmp2=tmp+1;
	if(a==12)
	{
		if(700<value&&value<=790)		//Block two diagonals to the right, placed correctly.
		{
			count1++;
			buzzer_on();_delay_ms(500);buzzer_off();
		}
		else if(550<value&&value<=600)	//Block two diagonals to the right, placed incorrectly.
		{
			Blk_Pos[tmp2][0]=1;
		}
		else if(150<value&&value<190)	//Block one diagonal to the left , placed incorrectly.
		{
			Blk_Pos[tmp][1]=1;
		}
		else if(200<value&&value<240)	//Block one diagonal to the right, placed correctly.
		{
			count1++;
			buzzer_on();_delay_ms(500);buzzer_off();
		}
	}
	else if(a==10)
	{
		
		if(700<value&&value<=790)	//Block two diagonals to the left, placed incorrectly.
		{
			Blk_Pos[tmp2][4]=1;
		}
		else if(550<value&&value<=600)	//Block two diagonals to the left, placed correctly.
		{
			count1++;
			buzzer_on();_delay_ms(500);buzzer_off();
		}
		else if(150<value&&value<190)	//Block one diagonal to the left, placed correctly.
		{
			count1++;
			buzzer_on();_delay_ms(500);buzzer_off();
		}
		
		else if(200<value&&value<240)	//Block one diagonal to the left, placed incorrectly.
		{
			Blk_Pos[tmp][3]=1;
		}
		
		
	}
	
}
/*
*
* Function Name: Sharp_Detection
* Input: void
* Output: void
* Logic: <Description of the function performed and the logic used
* in the function>
* Example Call: Sharp_Detection();
*
*/
void Sharp_Detection(void)
{
	Sensor(10);     //Calls SHARP IR with ADC conversion value 10, ie, sharp sensor placed at 45 degree to the left
	Sensor(12);		//Calls SHARP IR with ADC conversion value 12, ie., SHARP IR placed 45 degrees to the right
}
/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the nodes specified 
* Example Call: forward_wlsD2(2) //Goes forward by two nodes
*
*/	

void forward_wls( unsigned char node)
{	
	unsigned char flagw=0;
	while(1)
	{	
		

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		forward();
		velocity(225,230);
	//Function to detect node
		 
		if( ( Center_white_line >0x0C && Left_white_line >0x0C) || (  Right_white_line>0x0C && Left_white_line>0x0C) || (  Center_white_line>0x0C && Right_white_line>0x0C))
		{		 velocity(0,0);
			forward();
			_delay_ms(1000);
			flagw=flagw+1;	//			increments number of nodes detected by the robot.
			velocity(225,230);
			forward_mm(30);
			stop();
			_delay_ms(500);
				if (node==flagw)
				{
					break;				//stops line following if no. of nodes specified by user and detecetd by the robot is same 
				}
					
				
		}		
		if((Center_white_line>=0x0C) )				// goes forward if center white line detects reading more than "0X0C"
		{
			buzzer_off();
			forward();
			velocity(225,230);
		}

		if((Left_white_line>=0x0C))				// turns when lefts white line detects value more than 0x0C
		{
			buzzer_off();
			forward();
			velocity(80,190);
		}

		if((Right_white_line>=0x0C) )				// turns when right white line detects value more than 0x0C
		{
			buzzer_off();
			forward();
			velocity(190,80);
		}
	CurPos(node,0,0,0);
	}
}
/*
*
* Function Name: SharpSide
* Input: void
* Output: void
* Logic: Used for scanning nodes to the right of the bot when it if going leftwards in D2
* Example Call: SharpSide();
*
*/	void SharpSide(void)
{
	unsigned char value2;
	Sensor_read = ADC_Conversion(12);
	value2 = Sharp_GP2D12_estimation(Sensor_read);
	if(700<value2&&value2<=790)
	{
		if((Cur_Pos[0]<3))						
		{
			Blk_Pos[Cur_Pos[1]+2][Cur_Pos[0]+2]=2;
			buzzer_on();_delay_ms(500);buzzer_off();
			count2++;
		}
	}
	else if(550<value2&&value2<=600)
	{
		if((Cur_Pos[0]<3))
		{
			Blk_Pos[Cur_Pos[1]+2][Cur_Pos[0]+2]=1;
			
		}
	}
	else if(200<value2&&value2<240)
	{
			Blk_Pos[Cur_Pos[1]+1][Cur_Pos[0]+1]=2 ;
			buzzer_on();_delay_ms(500);buzzer_off();
			count2++;
	}
	else if(150<value2&&value2<190)
	{
			Blk_Pos[Cur_Pos[1]+1][Cur_Pos[0]+1]=1;
	}
}

/*
*
* Function Name: forward_wlsD2
* Input: node
* Output: void
* Logic: Version of forward_wls used in  D2, calls SharpSide at each node in order to detect and log blocks.
* Example Call: forward_wlsD2(2) //Goes forward by two nodes calling SharpSide at 3 nodes (starting, first , second)
*
*/	
void forward_wlsD2( unsigned char node)			// forward function for traversing on D2.
{	unsigned char flagw=0;
	while(1)
	{	
		

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		
	//Function to detect node 
		 
		if( ( Center_white_line >0x0C && Left_white_line >0x0C) || (  Right_white_line>0x0C && Left_white_line>0x0C) || (  Center_white_line>0x0C && Right_white_line>0x0C))
		{	
			velocity(0,0);
			forward();
			_delay_ms(1000);
			flagw=flagw+1;
			velocity(215,210);
			forward_mm(30);
			stop();
			_delay_ms(500);
				if (node==flagw+1)					//checks nodes
				{
					SharpSide();
				}
				else if (node==flagw+2)
				{
					SharpSide();
				}
				else if (node==flagw+3)
				{
					SharpSide();
				}
				else if (node==flagw)
				{
					SharpSide();
					break;
				}
			}		
		if((Center_white_line>=0x0C) )				//if center white line detects value more tan 0x0C , goes forward.
		{
			//flag1=1;
			buzzer_off();
			forward();
			velocity(215,210);
		}

		if((Left_white_line>=0x0C))				// turns when lefts white line detects value more than 0x0C
		{
			//flag1=1;
			buzzer_off();
			forward();
			velocity(80,190);
		}

		if((Right_white_line>=0x0C) )				// turns when right white line detects value more than 0x0C
		{
			//flag1=1;
			buzzer_off();
			forward();
			velocity(190,80);
		}
//CurPos(node,0,0,0);
	}
}

/*
*
* Function Name: right_wls
* Input: void
* Output: void
* Logic: Goes forward by 30mm from the node so that the axle is aligned and then takes a right turn
* Example Call: right_wls();
*
*/
void right_wls()			// function for taking the right turn when on a node.
{
	velocity(215,210);
	forward_mm(30);
	stop();
	_delay_ms(500);
	velocity(215,210);
	right_degrees(90);
	stop();
	_delay_ms(500);
	
	
}

/*
*
* Function Name: left_wls
* Input: void
* Output: void
* Logic: Goes forward by 30mm from the node so that the axle is aligned and then takes a left turn
* Example Call: left_wls();
*
*/
void left_wls()				// function for taking a left turn while on a node 
{
	velocity(215,210);
	forward_mm(30);
	stop();
	_delay_ms(500);
	velocity(215,210);
	left_degrees(90);
	stop();
	_delay_ms(500);
	
}


/*
*
* Function Name: SharpTwist
* Input: void
* Output: integer returning the column number of the detected block
* Logic: Turns 45 degrees, and detects the presence of blocks, acts accordingly
* Example Call: n=SharpTwist(); //Returns the column number
*
*/	
uint8_t SharpTwist(void)
{
	uint8_t tmp=Cur_Pos[1],ret=0;
	if(Blk_Pos[tmp][2]!=0) //If block is already known to exist in the current row and column G.
		{
			if(Blk_Pos[tmp][2]==2)	
			{
				return 0;
			}
			else
			{
				left_wls();
				return 2;
			}
		}
	else if(Blk_Pos[tmp][3]!=0) //If blocks already known to exist in the current row and column H.
		{
			if(Blk_Pos[tmp][3]==2)
			{
				return 0;
			}
			else
			{
				left_wls();
				return 3;
			}
		}
	else if(Blk_Pos[tmp][4]!=0) //If block is known to already exists in the current row and column I.
		{
			if(Blk_Pos[tmp][4]==2)
			{
				return 0;
			}
			else
			{
			left_wls();
			return 4;
			}
		}	
	else //if block is not known to exist.
	{
		if(Cur_Pos[1]==0)
		{
			left_degrees(45);
			unsigned char value;
			Sensor_read = ADC_Conversion(12);
			value = Sharp_GP2D12_estimation(Sensor_read);	
			
			if(150<value&&value<180)			//Block is placed incorrectly at intersection of current row and column G
			{
				Blk_Pos[Cur_Pos[1]][2]=1;	
				ret=2;
			}
			else if(350<value&&value<400)		//Block is placed incorrectly at intersection of current row and column H
			{
				Blk_Pos[Cur_Pos[1]][3]=1;	
				ret=3;	
			}
			else if(700<value&&value<790)		//Block is placed incorrectly at intersection of current row and column I
			{
				Blk_Pos[Cur_Pos[1]][4]=1;
				ret=4;	
			}
			else if(120<value&&value<145)	//Block is placed correctly at intersection of current row and column G
			{
				Blk_Pos[Cur_Pos[1]][2]=2;	//Set value of block position as 2
				count2++;					//Increment counter for D2	
				ret=0;
			}
			else if(220<value&&value<250)	//Block is placed correctly at intersection of current row and column H
			{
				Blk_Pos[Cur_Pos[1]][3]=2;	//Set value of block position as 2
				count2++;					//Increment counter for D2
				ret=0;	
			}
			else if(530<value&&value<600)	//Block is placed correctly at intersection of current row and column I
			{
				Blk_Pos[Cur_Pos[1]][4]=2;	//Set value of block position as 2
				count2++;					//Increment counter for D2
				ret=0;	
			}
			right_degrees(45);
			if(ret!=0)
			{
				left_wls();
			}
		
		}
		else
		{
			left_degrees(45);
			unsigned char value;
			Sensor_read = ADC_Conversion(10);
			value = Sharp_GP2D12_estimation(Sensor_read);	
			
			if(150<value&&value<180)
			{
				Blk_Pos[Cur_Pos[1]][2]=1;
				ret=2;
			}
			else if(350<value&&value<400)
			{
				Blk_Pos[Cur_Pos[1]][3]=1;
				ret=3;	
			}
			else if(700<value&&value<790)
			{
				Blk_Pos[Cur_Pos[1]][4]=1;
				ret=4;	
			}
			else if(120<value&&value<145)
			{
				Blk_Pos[Cur_Pos[1]][2]=2;
				ret=0;
			}
			else if(220<value&&value<250)
			{
				Blk_Pos[Cur_Pos[1]][3]=2;
				ret=0;	
			}
			else if(530<value&&value<600)
			{
				Blk_Pos[Cur_Pos[1]][4]=2;
				ret=0;	
			}
			if(ret==0)
			{
				right_degrees(45);
			}
			else
			{
				left_degrees(45);
			}
		}
		
	}
	return ret;
}


/*
*
* Function Name: twist
* Input: <Inputs (or Parameters) list with description if any>
* Output: <Return value with description if any>
* Logic: <Description of the function performed and the logic used
* in the function>
* Example Call: <Example of how to call this function>
*
*/	
int twist (int x)            
{
	if(x%2!=0)							//first section makes the servo rotate by 180 degrees
	{
		servo_1(0);
		_delay_ms(500);
		servo_1(180);
		_delay_ms(1000);
		servo_1_free();
	}
	else								//second section makes the servo rotate back by 180 degrees
	{
		servo_1(180);
		_delay_ms(500);
		servo_1(0);
		_delay_ms(1000);
		servo_1_free();
	}
	return x+1;							//value of x is incremented by 1 i.e every time x is made either even or odd
}

/*
* Function Name:	 lift
* Input:			 int x and y -> Stores the even and odd values.
*					 int k and l -> Represents the degrees  from 0 to 90
* Output:			 Servo turns through 90 degrees
* Logic:			 this function consists of two sections,first section makes the servo rotate from 0 to 90 degrees and in second section
*					 it makes the servo rotate back from 90 to 0 degree.By using odd and even number concept both the sections are called alternately.
*
* Example Call:		 y=twist(y);
*
*/

int lift (int x)
{
	if(x%2!=0)							//first section makes the servo rotate by 90 degrees
	{
		
		unsigned char k = 0;
		
		for (k = 0; k <90; k++)			//k is slowly incremented from 0 to 90 degrees by introducing delay of 30 milliseconds after each increment.
		{
			servo_2(k);
			_delay_ms(30);
		}
		_delay_ms(500);
		servo_2_free();
	}
	else								//second section makes the servo rotate back to 0 degrees
	{
		unsigned char l = 90;
		
		for (l = 90; l >0; l--)			//l is slowly decremented from 90 to 0 degrees by introducing delay of 30 milliseconds after each decrement.
		{
			servo_2(l);
			_delay_ms(30);
		}
		_delay_ms(500);
		servo_2_free();

	}
	return x+1;	//value of x is incremented by 1 i.e every time x is made either even or odd.
}

/*
* Function Name:	 arm
* Input:			 integer x and y -> Stores the even and odd values.
* Output:			 Controls twist and lift functions.
* Logic:			 this function controls the twist and lift functions, first it calls the lift function which lifts the arm and then twist function is called
*					 which rotates the claw by 180 degrees, and finally again lift function is called to lift the arm back to initial position.
* Example Call:		 arm(0);_delay_ms(500);
*
*/

void arm (int i)
{
	static int x,y;
	y=lift(y);
	_delay_ms(500);
	x=twist(x);
	_delay_ms(500);
	y=lift(y);
	_delay_ms(500);
	if(i==0)
		count1++;
	else if(i==1)
		count2++;
	else
		return;
}



/*
*
* Function Name: rf
* Input: node
* Output: void
* Logic: Used to take a ninety degree right turn using the function right_wls and subsequently go forward by the number of nodes specified in the input.Updates current postition in D2.
* Example Call: rf(2); //Takes a right and goes forward by two nodes.
*
*/	void rf(unsigned int node, unsigned int i)
{
	if(i==0)
	{
		right_degrees(90);
	}
	else
	right_wls();_delay_ms(500);
	forward_wls(node);_delay_ms(500);
	if(flag==2)
	CurPos(0,node,node,0);
}

/*
*
* Function Name: lf
* Input: node
* Output: void
* Logic: Used to take a ninety degree left turn using the function left_wls and subsequently go forward by the number of nodes specified in the input. Updates current postition in D2.
* Example Call: lf(2); //Takes a left and goes forward by two nodes.
*
*/	
void lf(unsigned int node,int i)
{
	if(i==0)
	{
		left_degrees(90);
	}
	else
	left_wls();_delay_ms(500);
	forward_wls(node);_delay_ms(500);
	if(flag==2)
	CurPos(0,node,0,node);
}

/*
* Function Name: Start
* Input: void
* Output: void
* Logic: This function exists to adequately implement the starting case where the SHARP sensors can not detect blokcs placed at A6 and E6
* Example Call: Start();
*
*/	

void Start(void)
{
	rf(1,1);
	unsigned int value;
	_delay_ms(500);
	Sensor_read = ADC_Conversion(10);
	value = Sharp_GP2D12_estimation(Sensor_read);				//Stores Distance calculated in a variable "value".
	
	if(150<value&&value<190)
	{
		forward_wls(1);
		left_wls();
		arm(0);
		left_degrees(90);
		forward_wls(3);
	}
	else if(200<value&&value<240)
	{
		
		count1++;
		buzzer_on();_delay_ms(500);buzzer_off();
		right_degrees(180);
		forward_wls(2);
	}
	else
	{
		right_degrees(180);
		forward_wls(2);
	}
	_delay_ms(500);
	Sensor_read = ADC_Conversion(12);
	value = Sharp_GP2D12_estimation(Sensor_read);				//Stores Distance calculated in a variable "value".
	print_sensor(2,4,10);
	if(150<value&&value<190)
	{
		count1++;
		buzzer_on();_delay_ms(500);buzzer_off();
		right_degrees(180);
		forward_wls(1);
		left_wls();
		}
	else if(200<value&&value<240)
	{
		forward_wls(1);
		right_wls();
		arm(0);
		right_degrees(90);
		forward_wls(1);
		left_wls();
	}
	else
	{
		right_degrees(180);
		forward_wls(1);
		left_wls();
	
	}
	Cur_Pos[0]=2;
	Cur_Pos[1]=0;
}

/*
*
* Function Name: IRCorridor
* Input: void
* Output: void
* Logic: Traverses the corridor using analog IR sensors, sets Cur_Pos[1] to the appropriate row number and resets Cur_Pos[0] to zero.
* Example Call: IRCorridor();
*
*/	
void IRCorridor(void)
{	unsigned int IR_1,IR_3,IR_5;
	unsigned int flag5=0;
	unsigned int flag6=0;
	turn_on_ir_proxi_sensors();
	ShaftCountRight=0;
	forward_wls(1);
	while(1)
	{
		IR_1=ADC_Conversion(4);//ADC conversion initializations
		IR_5=ADC_Conversion(8);//ADC conversion initializations
		IR_3=ADC_Conversion(6);//ADC conversion initializations
		forward();
		velocity(225,225);
		if ( (IR_3<= 130) && flag6==0) // checks whether IR proximity is less than 130 mm., and stops if obstacle is detected.
		{	
			flag6=1;
			flag5=1;
			forward();
			velocity(0,0);     
			_delay_ms(500);
			velocity(225,225);
			left_degrees(90);
			stop();
			_delay_ms(500);
			
			
		}
		else if(( IR_5>=154 ) && flag5==1 && flag6==1)  //checks whether right  IR proxmity is greater than 154 if true , turns right.
		{ 
			_delay_ms(870);
			forward();
			velocity(0,0);
			_delay_ms(500);
			velocity(225,225);
			right_degrees(90);
			flag5=1;
			forward_mm(150);
			velocity(220,220);
			break;
		}
		//avoiding collision with the walls
		else if(IR_5<143 && flag6==1 && flag5==1)  
		{
			forward();
			velocity(150,220);
		}
		else if (IR_1< 143 && flag6==1 && flag5==1)
		{   forward();
			velocity(220,150);
		}											
	}
		 // Cases for calculation of distance travelled by the robot for determination of gate through which the robot enters d2.
		if(88<ShaftCountRight&&ShaftCountRight<97)
		{
			Cur_Pos[0]=0;
			Cur_Pos[1]=5;
		}
		else if(125<ShaftCountRight&&ShaftCountRight<143)
		{
			Cur_Pos[0]=0;
			Cur_Pos[1]=4;
		}
		else if(162<ShaftCountRight&&ShaftCountRight<181)
		{
			Cur_Pos[0]=0;
			Cur_Pos[1]=3;
		}
		else if(200<ShaftCountRight&&ShaftCountRight<218)
		{
			Cur_Pos[0]=0;
			Cur_Pos[1]=2;
		}
		else if(237<ShaftCountRight&&ShaftCountRight<256)
		{
			Cur_Pos[0]=0;
			Cur_Pos[1]=1;
		}
		buzzer_on();
		_delay_ms(1000);
		buzzer_off();							//Buzzer at end of Corridor and start of D2
	forward_wls(1);				//Following code takes the robot to position H7 and updates Cur_Pos accordingly. 
	Cur_Pos[1]-=1;
	left_wls();
	forward_wls(Cur_Pos[1]-1);
	Cur_Pos[1]=0;
	flag=2;
	int i,j;
	for(i=0; i<8; i++)				//Re-initializes the array for D2 storage
	{
		for(j=0; i<6; j++)
		{
			Blk_Pos[i][j]=0;
		}		
	}	
}


/*
*
* Function Name: exit1
* Input: void
* Output: void
* Logic: Called on row 1 for exiting D1 or when count exceeds six(Maximum number of blocks in D1). Takes appropriate exit route according to current position and block positions.* Example Call: <Example of how to call this function>
* Example Call: exit1();
*
*/	
void exit1(int i)
{
	if(Cur_Pos[1]==5)																			//If block is on row 2
	{
		if(Blk_Pos[5][3]==0)																	//Checks whether D2 is empty, if yes, exits directly.
		{
				lf(5-Cur_Pos[0], i);															//Goes left to reach node E2.
						flag=2;																				//Sets flag to 2 indicating end of D1.
		}
		else if(Blk_Pos[5][3]==1)																// If not empty it goes round D2
		{
			forward_wls(1);
			lf(2, 1);
			lf(1, 1);
			right_wls();
			flag=2;

		}
	}
	else if(Cur_Pos[1]==6)
	{
		
						lf(4-Cur_Pos[0], i);
						left_wls();
						forward_wls(1);
						flag=2;
	}
	buzzer_on();_delay_ms(1000);buzzer_off();					//Buzzer at end of D1

}
/*
*
* Function Name: exit2
* Input: void
* Output: void
* Logic: Called on row 2 for exiting D2 or when count exceeds four(Maximum number of blocks in D2). Subtracts current y co-ordinate with that of row 1 and subtracts current x co-ordinate with that of column 4.
* Example Call: exit2();
*	
*/	
void exit2(void)
{
	forward_wls(6-Cur_Pos[1]);	//Goes forward from current row position to row 1.
	lf(4-Cur_Pos[0], 1);			//Goes left from current column to column I.
	flag=3;
	buzzer_on();_delay_ms(5000);buzzer_off();
}
/*
*
* Function Name: D1
* Input: void
* Output: void
* Logic: Function for sorting blocks in D1
* Example Call: D1();
*	
*/
void D1(void)
{
		while(1)
		
		{
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			if(flag==0) // Flag is set to zero initially, when this state occurs 
			{
				if(Cur_Pos[1]==0)
						{
							
							forward_wls(1);				//Goes forward from START position.
							Cur_Pos[1]=0;				//Sets y co-ordinate to row 7.
							Sharp_Detection();_delay_ms(500); //For detection of blocks at B6 and D6.
							Start();					//Runs Start function as it is a special case where A6 and E6 are blind spots.
							flag=1;						//Sets flag to 1 to indicate end of special case
						}
			}
			if(flag==1)
			{
				
				if(count1<7)							//Sorts until count is 7
				{	
						if(Cur_Pos[0]==2&&(Cur_Pos[1]>=0&&Cur_Pos[1]<6)) //Proceeds only if bot is on column 2 and on rows 2 to 6. 
						{	
							
							
							if(count1<7)
							{
							unsigned short int tmp=Cur_Pos[1]+1;				//Stores value of next row, in order to reduce execution time when accessing block position.
								if(Blk_Pos[tmp][0]==0)							// Checks if block position at column A in the next row is zero. Proceeds to check in the next columns in order to determine the exact configuration of the blocks in the next row.  
								{
									if(Blk_Pos[tmp][1]==0)						//Checks if block in column B in the next row is zero, proceeds to form individual cases.
									{
										if(Blk_Pos[tmp][3]==0)					//Checks if block in column D in the next row is zero, proceeds to form individual cases.
										{
											if(Blk_Pos[tmp][4]==0)				//Checks if block in column E in the next row is zero, proceeds to form individual cases. The case formed thus is when there are no blocks in the next row. Henceforth denoted as (0, 0, 0, 0, 0).
											{
												if(Cur_Pos[1]==5)			//This case runs for (0,0,0,0,0) in row 1, where instead of going to row 1 it exits D1 from row 2 instead.
												{
													exit1(1);
												}
												else						//Case for (0,0,0,0,0), goes forward and calls Sharp_Detection();
												{
													forward_wls(1);_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
													
													
												}
											}
											else								//Case for (1,0,0,0,0)
											{
												forward_wls(1);_delay_ms(500);
												Cur_Pos[1]+=1;
												Sharp_Detection();_delay_ms(500); 
												lf(1,1);_delay_ms(500);
												arm(0);_delay_ms(500);
												right_degrees(90);
												rf(1,1);
												left_wls();_delay_ms(500);
												if(Cur_Pos[1]==6)			//This case runs for (0,0,0,0,1) in row 1, where instead of going to row 1 it exits D1 from row 2 instead.
												{
													forward_wls(1);
													exit1(1);
												}
											}
										}
										else								
										{
											if(Blk_Pos[tmp][4]==0)				//Case for (0,1,0,0,0)
											{
												if(Cur_Pos[1]==5)
												{
													lf(1,1);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													exit1(0);
												}
												else
												{
													forward_wls(1);_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													right_degrees(90);_delay_ms(500);																
												}
											}
											else								//Case for (1,1,0,0,0)
											{
												if(Cur_Pos[1]==5)
												{
													lf(1,1);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(1, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													exit1(0);
												}
												else
												{
													lf(2,1);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													rf(2,0);_delay_ms(500);
													left_wls();_delay_ms(500);
													forward_wls(1);_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													right_degrees(90);_delay_ms(500);
												}
											}
										}
									}
									else								
									{
										if(Blk_Pos[tmp][3]==0)
										{
											if(Blk_Pos[tmp][4]==0)				//Case for (0,0,0,1,0)
											{									
												
												if(Cur_Pos[1]==5)
												{
													forward_wls(1);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													left_degrees(90);_delay_ms(500);
													exit1(1);
												}
												else
												{
													forward_wls(1);_delay_ms(500);
													right_wls();_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
													arm(0);_delay_ms(500);
													left_degrees(90);_delay_ms(500);
												}
												
											}
											else								//Case for (1,0,0,1,0)
											{	
												if(Cur_Pos[1]==5)
												{
													lf(1,1);
													right_wls();
													arm(0);_delay_ms(500);
													lf(3,0);_delay_ms(500);
													arm(0);_delay_ms(500);
													exit1(0);
												}
												else
												{
													forward_wls(1);_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													left_wls();_delay_ms(500); 
													lf(1,1);_delay_ms(500);
													arm(0);_delay_ms(500);
													right_wls();
													rf(1,1);
													left_wls();_delay_ms(500);
													
												}
											}
										}
										else									
										{
											if(Blk_Pos[tmp][4]==0)				//Case for (0,1,0,1,0)
											{
												if(Cur_Pos[1]==5)
												{
														rf(1,1);
														left_wls();
														arm(0);_delay_ms(500);
														lf(2,0);
														right_wls();
														arm(0);_delay_ms(500);
														exit1(0);
												}
												else
												{
														forward_wls(1);_delay_ms(500);		//No detection following forward as placement rules forbid placing of blocks in the next row if blocks are placed in the given configuration in a certain row.
														left_wls();_delay_ms(500);
														arm(0);_delay_ms(500);
														right_degrees(180);
														arm(0);_delay_ms(500);
														left_degrees(90);_delay_ms(500);
														forward_wls(1);_delay_ms(500);
														Cur_Pos[1]+=1;
														Sharp_Detection();_delay_ms(500);
												}
											}
											else								//Case for (1,1,0,1,0)
											{
												if(Cur_Pos[1]==5)
												{
													rf(1,1);
													left_wls();
													arm(0);_delay_ms(500);
													lf(1,0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(3,0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													exit1(0);
												}
												else
												{
													lf(2,1);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													rf(2,0);_delay_ms(500);
													left_wls();_delay_ms(500);
													forward_wls(1);_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													right_degrees(180);
													arm(0);_delay_ms(500);
													left_degrees(90);_delay_ms(500);
													forward_wls(1);_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
												}
											}
										}
									}
									
								}
								else
								{
									if(Blk_Pos[tmp][1]==0)
									{
										if(Blk_Pos[tmp][3]==0)
										{
											if(Blk_Pos[tmp][4]==0)				//Case for (0,0,0,0,1);
											{
												forward_wls(1);_delay_ms(500);
												Cur_Pos[1]+=1;
												Sharp_Detection();_delay_ms(500);
												rf(1,1);
												arm(0);_delay_ms(500);
												left_degrees(90);_delay_ms(500);
												lf(1, 1);_delay_ms(500);
												right_wls();_delay_ms(500);
												if(Cur_Pos[1]==6)
												{
													exit1(1);
												}
											
											}
											else								//Case for (1,0,0,0,1)
											{
												forward_wls(1);_delay_ms(500);
												Cur_Pos[1]+=1;
												Sharp_Detection();_delay_ms(500);
												rf(1, 1);
												arm(0);_delay_ms(500);
												left_degrees(90);_delay_ms(500);
												lf(2, 1);_delay_ms(500);
												arm(0);_delay_ms(500);
												right_degrees(90);_delay_ms(500);
												if(Cur_Pos[1]==6)
												{
													exit1(1);
												}
												else
												{
													rf(1,1);
													left_wls();_delay_ms(500);
												}
											}
										}  
										else
										{
											if(Blk_Pos[tmp][4]==0)				//Case for (0,1,0,0,1)
											{
												
												forward_wls(1);_delay_ms(500);
												Cur_Pos[1]+=1;
												Sharp_Detection();_delay_ms(500);
												rf(1, 1);_delay_ms(500);
												arm(0);_delay_ms(500);
												left_degrees(90);_delay_ms(500);
												lf(1, 1);_delay_ms(500);
												arm(0);_delay_ms(500);
												right_degrees(90);_delay_ms(500);
												if(Cur_Pos[1]==5)
												{
													back_mm(200);
													exit1(1);
												}
																		

											}
											else								//Case for (1,1,0,0,1)
											{
												if(Cur_Pos[1]==5)
												{
													rf(2,1);
													left_wls();
													arm(0);
													lf(3, 0);
													right_wls();
													arm(0);
													lf(1,0);
													right_wls();
													arm(0);
													exit1(0);													
												}
												else
												{
													lf(2,1);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													rf(2, 0);_delay_ms(500);
													left_wls();_delay_ms(500);
													forward_wls(1);_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
													rf(1, 1);
													arm(0);_delay_ms(500);
													left_degrees(90);_delay_ms(500);
													lf(1, 1);_delay_ms(500);
													arm(0);_delay_ms(500);
													right_degrees(90);_delay_ms(500);
												}												
								
											}
										}
									}
									else
									{
										if(Blk_Pos[tmp][3]==0)
										{
											if(Blk_Pos[tmp][4]==0)				//Case for (0,0,0,1,1)
											{
												rf(2,1);_delay_ms(500);
												left_wls();_delay_ms(500);
												arm(0);_delay_ms(500);
												if(Cur_Pos[1]==5)
												{
													lf(1, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													exit1(0);
												}
												else
												{
												lf(2, 0);_delay_ms(500);
												right_wls();_delay_ms(500);
												forward_wls(1);_delay_ms(500);
												Cur_Pos[1]+=1;
												Sharp_Detection();_delay_ms(500);
												right_wls();_delay_ms(500);
												arm(0);_delay_ms(500);
												left_wls();_delay_ms(500);
												
												}
											}				
											else								//Case for (1,0,0,1,1)
											{
												rf(2,1);_delay_ms(500);
												left_wls();_delay_ms(500);
												arm(0);_delay_ms(500);
												if(Cur_Pos[1]==5)
												{
													lf(1, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(3, 1);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													exit1(0);
												}
												else
												{
												lf(4, 0);_delay_ms(500);
												right_wls();_delay_ms(500);
												arm(0);_delay_ms(500);
												rf(2,0);_delay_ms(500);
												left_wls();_delay_ms(500);
												forward_wls(1);_delay_ms(500);
												Cur_Pos[1]+=1;
												Sharp_Detection();_delay_ms(500);
												right_wls();_delay_ms(500);
												arm(0);_delay_ms(500);
												left_degrees(90);_delay_ms(500);
												}
											}
										}
										else
										{
											if(Blk_Pos[tmp][4]==0)				//Case for (0,1,0,1,1)
											{
												if(Cur_Pos[1]==5)
												{
													rf(2,1);_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(1, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(2, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													exit1(0);
												}
												else
												{
													rf(2,1);_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(2, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													forward_wls(1);_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													right_degrees(180);
													arm(0);_delay_ms(500);
													left_degrees(90);_delay_ms(500);
													forward_wls(1);_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
												}

											}
											else								//Case for (1,1,0,1,1)
											{
												if(Cur_Pos[1]==5)
												{
													rf(2,1);_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(1, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(2, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(1, 0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													exit1(0);
												}
												else
												{
													rf(2, 1);_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													lf(4,0);_delay_ms(500);
													right_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													rf(2, 0);_delay_ms(500);
													left_wls();_delay_ms(500);
													forward_wls(1);_delay_ms(500);
													left_wls();_delay_ms(500);
													arm(0);_delay_ms(500);
													right_degrees(180);
													arm(0);_delay_ms(500);
													left_degrees(90);_delay_ms(500);
													forward_wls(1);_delay_ms(500);
													Cur_Pos[1]+=1;
													Sharp_Detection();_delay_ms(500);
												}
											}
										}
									}
								}
							}					
						}					
					}
			else
			{
				/*Actual Buzzer sound*/buzzer_on();_delay_ms(1000);buzzer_off();
				forward_wls(5-Cur_Pos[0]);
				exit1(1);
				Cur_Pos[0]=0;
			}
			}
			else
				break;
			}
}
/*
*
* Function Name: D2
* Input: void
* Output: void
* Logic: Function for sorting blocks in D1
* Example Call: D2();
*	
*/							
void D2(void)
{
	while(1)
	{
			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			if(flag==2)								//Runs only of flag is 2
			{
				if(count2<5)						//Exits if the the count exceeds 4
				{
					int n=SharpTwist();				//Calls SharpTwist in order to find the blocks in the current row.
					if(n!=0)						//Enters condition, if incorrectly placed blocks are found
					{
						//Go and check
						//Stop at every node and check
						forward_wlsD2(n-1);
						arm(1);
						if(Blk_Pos[Cur_Pos[1]+1][Cur_Pos[0]]==1)	//If block exists to the north of the current block, it sorts it and moves to position (CurrentRow+2)7 or else if it is on row 2 it sorts both blocks and exits.
						{
									right_wls();
									arm(1);
									Blk_Pos[Cur_Pos[1]+1][Cur_Pos[0]]=2;
									if(Cur_Pos[1]==4)
									{
										rf(1, 0);
										left_wls();
										forward_wls(1);
										exit2();
									}
									else
									{
										rf(Cur_Pos[0]-1, 0);
										left_wls();
										forward_wls(2);
									}
									
						}
						else if(Blk_Pos[Cur_Pos[1]+1][Cur_Pos[0]-1]==1)	//If block exists to the north-east of the current block, it sorts it and moves to position (CurrentRow+2)7 or else if it is on row 2 it sorts both blocks and exits.
						{
							if(Cur_Pos[1]==4)
									{
										right_wls();
										forward_wls(1);
										right_wls();
										arm(1);
										left_wls();
										exit2();
									}
							else
							{
										right_wls();																			
										rf(1,0);
										left_wls();
										arm(1);
										Blk_Pos[Cur_Pos[1]+1][Cur_Pos[0]]=2;
										rf(1, 0);
										left_wls();
										forward_wls(2);
							}
							
						}
						else if(Blk_Pos[Cur_Pos[1]+1][Cur_Pos[0]+1]==1)	//If block exists to the north-west of the current block, it sorts it and moves to position (CurrentRow+2)7 or else if it is on row 2 it sorts both blocks and exits.
						{
							right_wls();
							forward_wls(1);
							left_wls();
							arm(1);
							Blk_Pos[Cur_Pos[1]][Cur_Pos[0]+1]=2;
							right_degrees(90);
							if(Cur_Pos[1]==5)
							{
								exit2();
							}
							else
							{
								rf(Cur_Pos[0]-1,1);
								left_wls();
								forward_wls(1);
							}
														
						}
												
					}
					else											//If no blocks exist in the current row, move forward by one node.
					{
						forward_wls(1);
					}
				}
				else												//If count exceeds four blocks then proceed towards I1.
				{
					exit2();
				}
			}
			else													//Break when flag is not equal to 2.
			{
				break;
			}
			
	}
}



/*
 * Function Name: main
 * Input: None
 * Output: int to inform the caller that the program exited correctly or
 * incorrectly (C code standard)
 * Logic: Call functions D1 followed by functions IRCorridor and D2
 * Example Call: Called automatically by the Operating System
 *
 */


int main(void)
{
	init_devices();										//initializes all devices
	turn_on_sharp234_wl();								//turns on sharp 2,3,4 and white line sensors .
	turn_off_sharp15();									//turns off sharp 1,5 sensors for battery optimization.
	turn_off_ir_proxi_sensors();						//turns off IR analog sensors for battery optimization.
	D1();												//calls D1, traverses D1
	IRCorridor();										//calls IRCorridor, traverses IRCorridor
	D2();												//calls D2, traverses D2
	return 0;
}				
