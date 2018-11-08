/**  Control of the intersection model  **/

#include "driverlib.h"		//Libraries used in program
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>			//Used for debugging 
#include <stdlib.h>		//Used for debugging 	
#include <string.h>		//Used for debugging 
#include "msp.h"

//INITIALIZATION FUNCTIONS
void initializeAll();                 //  Function used to initialize all components
void timer32_init();                  //  Initializes instance 1 and instance two of timer32
void trafficLightPin_Init();          //  Initialize the traffic light and cross walk pins
void timerA_init();                   //  Initialize timerA0.1 (Servo) and timerA0.2 (Sounder)
void lcdDisplay_init();               //  Initializes the LCD
void SysTick_Init();                  //  Initializes the systick timer used for basic delays
void lcdPin_Init();                   //  Initializes the LCD pins
void servoPin_init();                 //  Initializes the Servo pin 6.7 TimerA0.1 for PWM
void stepperPin_Init();               //  Initializes the stepper motor pins GPIO
void buzzer_init();                   //  Initializes P2.5 to use for the sounder
void gpioHandler_Init();    //  Initializes port 1, used for the cross walk GPIO interupts

//LCD CURSOR / SCREEN MANIPULATION FUNCTIONS
void nextLine();                      //  Pass 0xC0 as an instruction to go to next line
void cursorToHome();                  //  Pass 0x01 instruction send cursor home position
void clearScreen();                   //  Will clear the screen of the LCD

//LCD DATA PASSING FUNCTIONS
void instructionWrite(uint8_t command);     //  Assigns the RS to 0
void dataWrite(uint8_t data);               //  Assigns the RS value to 1
void PulseEnablePin();                      //  Used to initiate the transaction of bits to LCD
void pushNibble(uint8_t nibble);            //  Passes four bits at a time to the LCD
void pushByte(uint8_t byte);                //  Masks appropriate bits and sends to pushNibble()

//TRAFFIC LIGHT FUNCTIONS
void ns_toggleYellowLED();           //  Toggle the yellow LED for NS traffic
void ew_toggleYellowLED();           //  Toggle the yellow LED for EW traffic
void checkNS_crossWalk();            //  Checks the cross walk flags
void checkEW_crossWalk();            //  Checks the cross walk flags
void NS_crosswalkFlash();            //  Does the toggle act (manipulates the pin output
void EW_crosswalkFlash();            //  Does the toggle act (manipulates the pin output

//BUZER FUNCTIONS
void buzzerTone1();              //  Sets a low tone for sounder  (cross walk is green)
void buzzerTone2();		   //  Sets a medium tone for the sounder  
void buzzerTone3();              //  Sets a high tone for the sounder  
void buzzOff();                  //  Sets the sounder to off  (cross walk is off)

void fridgeTone();               //  Fun tone sounds when the program starts

//SYSTIC TIMER FUNCTIONS
void delay_ms(unsigned ms);           //  Uses the clock cycle period to determine milli
void delay_micro(unsigned microsec);  //  Uses the clock cycle period to determine micros

//HANDLERS
void T32_INT1_IRQHandler(void); //  Instance 1 of timer32 interupt handler used to track the durration of each light which is then set to the LCD to update

void T32_INT2_IRQHandler(void); //  Instance 2 of timer32 interrupt handler increments the flags which control the change of states

void TA0_N_IRQHandler (void);   //  Timer A0 Used to capture the incomming IR signal

void PORT1_IRQHandler(void);
void PORT2_IRQHandler(void);//Handler used to control the angle of the LCD; button presses
void PORT4_IRQHandler(void);
void PORT5_IRQHandler(void);

//STEPPER FUNCTIONS
void directionUp();                          //  Stepper motor
void directionDown();                          //  Stepper motor

//STATE FUNCTIONS
void state1();     //  Sets NS Straight lights to green  / NS left flashing yellow
void state2();     //  Sets NS Straight lights to yellow  / NS left flashing yellow
void state3();     //  Sets NS Straight lights to red / Continues to flash NS left yellows
void state4();     //  Sets NS Left green
void state5();     //  Sets NS Left lights to yellow
void state6();     //  Sets NS Left lights to red / ALL LIGHTS RED
void state7();     //  Sets EW Straight lights to green / EW left flashing yellow
void state8();     //  Sets EW Straight to green / EW left flashing yellow
void state9();     //  Sets EW Straight to red / EW left green
void state10();    //  Sets EW Left to yellow
void state11();    //  Sets EW Left to red
void state12();    //  Sets All lights red for 2 seconds

//LCD DISPLAY FUNCTIONS
void displayTopLine(char* topLine);     //  Used to pass strings to display on the top line of the LCD
void displayBottomLine(char* bottomLine);   //  Used to pass strings to the bottom line of the LCD
void tocks_displayTopLine(char* topLine);       //  Used to pass the counter variable tocks to display the current length the light has been in state
void tocks_displayBottomLine(char* bottomLine); //  Used to pass the counter vairable tocks to the bottom position of the LCD display

//LCD DISPLAY LINES
char ns[] = "N/S ";
char ew[] = "E/W ";
char s1[] = "S. green ";
char s2[] = "S. yellow";
char s3[] = "S. red   ";
char s4[] = "L. green ";
char s5[] = "L. yellow";
char s6[] = "L. red   ";
char s7[] = "S. green ";
char s8[] = "S. yellow";
char s9[] = "S. red   ";
char s10[] = "L. green ";
char s11[] = "L. yellow";
char s12[] = "L. red   ";

uint8_t nsBuzzerFlag = 0;
uint8_t ewBuzzerFlag =0;
uint8_t buzzFlag = 0;

uint8_t dFlag = 0, checkFlagy = 0;          // Ensures the dislpay is only updated once per state to save time
uint8_t nsTocks = 0;        //Used to control the count of the NS
uint8_t ewTocks = 0;        //Used to control the count of EW lights

uint32_t flagRavel = 1175;
uint8_t safetyFlag = 0;
void checkPole();

uint32_t toggle = 0;
uint32_t NS_toggleDurringGreen = 20;
uint32_t EW_toggleDurringGreen = 40;
float toggleDurringYellow = 10;
float toggleDurringRed = 2.5;

uint8_t state = 1;
uint32_t i = 0;
uint8_t NS_crosswalk = 0;
uint8_t EW_crosswalk = 0;

uint32_t servoPos = 1900;

uint32_t period = 60000;
uint32_t dCycle = 1900;
uint8_t stepperSpeed = 3;
uint8_t ticks = 0;
uint32_t on_off = 200;          // 200 * 2 is a single cycle of ms for the flashing yellow

static uint16_t capturedCyclesCurrent;
static uint16_t capturedCyclesPast = 0;
uint16_t capturedPeriod;

  int main(void){

	initializeAll();

	delay_micro(100);
	displayTopLine(ns);
	delay_micro(100);
	nextLine();
	delay_micro(100);
	displayBottomLine(ew);

	fridgeTone();

	directionUp();

	//directionDown();

	while (1){

		 switch(state){
		 case 1:
			  state1();                           // Straight N.S. green
			  break;
		 case 2:
			  state2();                           // Straight N.S. yellow
			  break;
		 case 3:
			  state3();                           //  Straight N.S. red
			  break;                              //  Smooth transition from straght red to left green
		 case 4:
			  state4();                           //  N.S. left green
			  break;
		 case 5:
			  state5();                           //  N.S. left yellow
			  break;
		 case 6:
			  state6();                           //  N.S. left red / ALL LIGHTS RED
			  break;
		 case 7:
			  state7();                           //  E.W. straight green
			  break;
		 case 8:
			  state8();                           //  E.W. straight yellow
			  break;
		 case 9:
			  state9();                           //  E.W. straight red
			  break;
		 case 10:
			  state10();                          //  E.W. left green
			  break;
		 case 11:
			  state11();                          //  E.W. left yellow
			  break;
		 case 12:
			  state12();                          //  E.W. left red // ALL LIGHTS RED
			  break;
		 }
}
}

void checkPole(){
 if(safetyFlag == 1){
	  safetyFlag = 0;
	  P3->OUT |= 0X88;        //  E.W. Red
	  P6->OUT |= 0X90;        //  N.S. Left has red // ALL LIGHTS ARE RED

	  directionDown();

	  P3->OUT &=~ 0X88;        //  E.W. Red
	  P6->OUT &=~ 0X90;        //  N.S. Left has red // ALL LIGHTS ARE RED
	  state = 1;
 }
 if(safetyFlag == 2){
	  safetyFlag = 0;
	  P3->OUT |= 0X88;        //  E.W. Red
	  P6->OUT |= 0X90;        //  N.S. Left has red // ALL LIGHTS ARE RED

	  directionUp();

	  P3->OUT &=~ 0X88;        //  E.W. Red
	  P6->OUT &=~ 0X90;        //  N.S. Left has red // ALL LIGHTS ARE RED
	  state = 1;
 }
}

void TA0_N_IRQHandler( void ) {
	MAP_Timer_A_clearCaptureCompareInterrupt (TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3);
	capturedCyclesCurrent = MAP_Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3);

	 if (capturedCyclesCurrent > capturedCyclesPast){
		 capturedPeriod = (capturedCyclesCurrent - capturedCyclesPast);
	 }
	 else{
		  capturedPeriod = 65535 - (capturedCyclesPast + capturedCyclesCurrent);
	 }

	 if ((35635 < capturedPeriod) && (capturedPeriod < 39375) ){  // within 5% of 10Hz period detect10Hz=1;
		  P2->OUT |= BIT5;
	 }
	 if ((25446 < capturedPeriod ) && (capturedPeriod < 28055) ){  // within 5% of 14Hz period detect14Hz=1;
		  state = 1;
	 }

	 capturedCyclesPast = capturedCyclesCurrent ;            // save for next interrupt
}

//STATES
void state1(){

	checkPole();
	if(dFlag == 0){
		 instructionWrite(0x84);
		 delay_micro(37);
		 displayTopLine(s1);
		 delay_micro(37);
		 instructionWrite(0xC4);
		 delay_micro(37);
		 displayBottomLine(s9);
		 delay_micro(37);
		 dFlag = 1;  //  Used to toggle the specific display for the state of the lights
	}

	nsTocks = 0;
	ewTocks = 0;

	P7->OUT |= BIT4;       //  EW GREEN Crosswalk off for safety
	P7->OUT &=~ BIT5;       //  Crosswalk off for safety
	P7->OUT |= BIT6;       //  EW GREEN Crosswalk off for safety
	P7->OUT &=~ BIT7;       //  Crosswalk off for safety

	P6->OUT &=~ 0X90;        //  Turn NS red off from state 12
	P6->OUT |= 0X41;        //  N.S. Straight has green
	P3->OUT |= 0X88;        //  E.W. Red

	toggle = NS_toggleDurringGreen;     // Set the durration of the flashing yellow
	ns_toggleYellowLED();
	state++;                            // Increment the state
}
void state2(){
	//checkPole();
	if(dFlag == 1){
		 instructionWrite(0x84);
		 delay_micro(37);
		 displayTopLine(s2);
		 dFlag = 0;
	}

	nsTocks = 0;

	P6->OUT &=~ 0X41;       //  N.S. green off
	P6->OUT |= 0X42;        //  N.S. Straight has yellow

	if(NS_crosswalk == 2){
		 NS_crosswalk = 3;
	}
	toggle = toggleDurringYellow;       // Set the durration of the flashing yellow lights
	ns_toggleYellowLED();
	state++;                            // Increment the state
	ticks = 0;
}
void state3(){
	//checkPole();
	if(dFlag == 0){
		 instructionWrite(0x84);
		 delay_micro(37);
		 displayTopLine(s3);
		 dFlag = 1;
	}

	nsTocks = 0;

	P6->OUT &=~ 0X42;       //  N.S. straight yellow off
	P6->OUT |= 0X50;        //  N.S. Straight has red

	if(NS_crosswalk == 3){
		 P7->OUT |= BIT6;
		 NS_crosswalk = 0;
	}

	if(nsBuzzerFlag > 10){
		 buzzerTone3();
		 nsBuzzerFlag = 0;
	}
	state++;                       //  Increment state
	ticks = 0;                     //  Reset the counter ticks
}
void state4(){
	//checkPole();
	if(dFlag == 1){
		 instructionWrite(0x84);
		 delay_micro(37);
		 displayTopLine(s4);
		 dFlag = 0;
	}

	nsTocks = 0;

	P6->OUT &=~ 0X50;        //  N.S. Straight has red
	P6->OUT |= 0X30;        //  N.S. Left has green

	if(ticks == 5){                     //  Hold N.S. left green
		 state++;
		 ticks = 0;                      //  Reset the counter ticks
	}
}
void state5(){
	//checkPole();
	if(dFlag == 0){
		 instructionWrite(0x84);
		 delay_micro(37);
		 displayTopLine(s5);
		 dFlag = 1;
	}

	nsTocks = 0;

	P6->OUT &=~ 0X30;       //  Turn off
	P6->OUT |= 0X50;        //  N.S. Left has yellow

	if(NS_crosswalk == 3){
			  NS_crosswalk = 0;
	}

	if(ticks == 3){                     //  Hold N.S. left yellow for 3 seconds
		 state++;                        //  Increment state
		 ticks = 0;                      //  Reset the counter ticks
	}
}
void state6(){

	if(dFlag == 1){
		 instructionWrite(0x84);
		 delay_micro(37);
		 displayTopLine(s6);
		 dFlag = 0;
	}

	nsTocks = 0;

	P7->OUT |= BIT4;       //  EW GREEN Crosswalk off for safety
	P7->OUT &=~ BIT5;       //  Crosswalk off for safety
	P7->OUT |= BIT6;       //  EW GREEN Crosswalk off for safety
	P7->OUT &=~ BIT7;       //  Crosswalk off for safety

	buzzOff();              //  Turn off crosswalk buzzer
	//nsBuzzerFlag = 0;
	checkPole();
	P6->OUT &=~ 0X50;
	P6->OUT |= 0X90;        //  N.S. Left has red // ALL LIGHTS ARE RED

	if(ticks == 2){                     //  Hold all lights red for 2 seconds
		 state++;                        //  Increment state
		 ticks = 0;                      //  Reset the counter ticks
	}
}
void state7(){
	//checkPole();
	if(dFlag == 0){
		 instructionWrite(0xC4);
		 delay_micro(37);
		 displayBottomLine(s7);
		 dFlag = 1;
	}

	ewTocks = 0;

	P3->OUT &=~ 0X88;       //  E.W. Red off
	P3->OUT |= 0x41;        //  E.W. Straight has green

	P5->OUT &=~ BIT1;       //  Turns off cross walk for safety

	toggle = EW_toggleDurringGreen;        //  Set the durration of the flashing yellow lights
	ew_toggleYellowLED();               //  Flash the yellow lights
	ticks = 0;                      //  Reset the counter ticks
	state++;                            //

}
void state8(){
  // checkPole();
	if(dFlag == 1){
		 instructionWrite(0xC4);
		 delay_micro(37);
		 displayBottomLine(s8);
		 dFlag = 0;
	}

	ewTocks = 0;

	P3->OUT &=~ 0X41;       //  E.W. straight green off
	P3->OUT |= 0X44;        //  E.W. Straight has yellow

	if(EW_crosswalk == 2){
		 EW_crosswalk = 3;
	}
	toggle = toggleDurringYellow;       // Set the durration of the flashing yellow lights
	ew_toggleYellowLED();
	state++;                            //  Increment the state
	ticks = 0;                      //  Reset the counter ticks
}
void state9(){
	//checkPole();
	if(dFlag == 0){
		 instructionWrite(0xC4);
		 delay_micro(37);
		 displayBottomLine(s9);
		 dFlag = 1;
	}

	ewTocks = 0;

	P3->OUT &=~ 0X44;       //  E.W. Straight yellow off
	P3->OUT |= 0X48;        //  E.W. Straight has red

	if(EW_crosswalk == 3){
		 P7->OUT |= BIT4;
		 EW_crosswalk = 0;
	}

	if(ewBuzzerFlag > 10){
		 buzzerTone3();
		 ewBuzzerFlag = 0;
	}
	state++;                      //  Increment state
	ticks = 0;                    //  Reset the counter ticks
}
void state10(){
	//checkPole();
	if(dFlag == 1){
		 instructionWrite(0xC4);
		 delay_micro(37);
		 displayBottomLine(s10);
		 dFlag = 0;
	}

	ewTocks = 0;

	P3->OUT &=~ 0X48;       //  E.W. Straight red off
	P3->OUT |= 0X28;        //  E.W. Left has green

	if(ticks == 5){                     //  Hold E.W. left green
		 state++;                        //  Increment state
		 ticks = 0;                      //  Reset the counter ticks
	}
}
void state11(){
	//checkPole();
	if(dFlag == 0){
		 instructionWrite(0xC4);
		 delay_micro(37);
		 displayBottomLine(s11);
		 dFlag = 1;
	}

	ewTocks = 0;

	P3->OUT &=~ 0X28;       //  E.W. Left green off
	P3->OUT |= 0X48;        //  E.W. Left has yellow

	if(EW_crosswalk == 3){
			  EW_crosswalk = 0;
		 }

	if(ticks == 3){                     //  Hold E.W. left green for 3 seconds
		 state++;                        //  Increment state
		 ticks = 0;                      //  Reset the counter ticks
	}
}
void state12(){
	checkPole();
	if(dFlag == 1){
		 instructionWrite(0xC5);
		 delay_micro(37);
		 displayBottomLine(s12);
		 dFlag = 0;
	}

	ewTocks = 0;

	P7->OUT |= BIT4;       //  EW GREEN Crosswalk off for safety
	P7->OUT &=~ BIT5;       //  Crosswalk off for safety
	P7->OUT |= BIT6;       //  EW GREEN Crosswalk off for safety
	P7->OUT &=~ BIT7;       //  Crosswalk off for safety

	buzzOff();              //  Turn off crosswalk buzzer
//    ewBuzzerFlag = 0;
	checkPole();
	P3->OUT &=~ 0X48;       //  E.W. Left yellow off
	P3->OUT |= 0X88;        //  E.W. Red // ALL RED

	if(ticks == 2){                     //  Hold all lights red for 2 seconds
		 state = 1;                        //  Increment state
		 ticks = 0;                      //  Reset the counter ticks
	}
}

void directionUp(){
for(i = 0; i< flagRavel; i++){
		 P5OUT |= BIT4;                          // Assigns pin 0001 0000 high
		 delay_ms(stepperSpeed);                 // stepper speed determined by user
		 P5OUT &=~ BIT4;                         // Turn pin low

		 P5OUT |= BIT5;                          // Assigns pin 0010 0000 high
		 delay_ms(stepperSpeed);                 // stepper speed determined by user
		 P5OUT &= ~BIT5;                         // Turn pin low

		 P5OUT |= BIT6;                          // Assigns pin 0100 0000 high
		 delay_ms(stepperSpeed);                 // stepper speed determined by user
		 P5OUT &= ~BIT6;                         // Turn pin low

		 P5OUT |= BIT7;                          // Assigns pin 1000 0000 high
		 delay_ms(stepperSpeed);                 // stepper speed determined by user
		 P5OUT &= ~BIT7;                         // Turn pin low
	}
}
void directionDown(){
	for(i = 0; i< flagRavel; i++){
		 P5OUT |= 0x80;                          //Assigns pin 1000 0000 high
		 delay_ms(stepperSpeed);                 // stepper speed determined by user
		 P5OUT &= ~0x80;                         // Turn pin low

		 P5OUT |= 0x40;                          // Assigns pin 0100 0000 high
		 delay_ms(stepperSpeed);                 // stepper speed determined by user
		 P5OUT &= ~0x40;                         // Turn pin low

		 P5OUT |= 0x20;                          // Assigns pin 0010 0000 high
		 delay_ms(stepperSpeed);                 // stepper speed determined by user
		 P5OUT &= ~0x20;                         // Turn pin low

		 P5OUT |= 0x10;                          // Assigns pin 0001 0000 high
		 delay_ms(stepperSpeed);                 // stepper speed determined by user
		 P5OUT &= ~0x10;                         // Turn pin low
}
}

void T32_INT1_IRQHandler(void){                          //Interrupt Handler for Timer 2
	 TIMER32_1->INTCLR = 1;                          //Clear interrupt flag so it does not interrupt again immediately.
	 TIMER32_1->LOAD = 3000000 - 1;                  //Set to a count down of 1 second on 3 MHz clock

	 nsTocks++;
	 ewTocks++;

	 instructionWrite(0x8E);  //15th post
	 delay_micro(37);
		  switch(nsTocks){
		  case 1:
				dataWrite(0x30);
				dataWrite(0x31);                      // Send each character to the function datawrite()

				break;

		  case 2:
				dataWrite(0x30);
				dataWrite(0x32);                      // Send each character to the function datawrite()
				break;

		  case 3:
				dataWrite(0x30);
				dataWrite(0x33);                      // Send each character to the function datawrite()
				break;

		  case 4:
				dataWrite(0x30);
				dataWrite(0x34);                      // Send each character to the function datawrite()
				break;

		  case 5:
				dataWrite(0x30);
				dataWrite(0x35);                      // Send each character to the function datawrite()
				break;

		  case 6:
				dataWrite(0x30);
				dataWrite(0x36);                      // Send each character to the function datawrite()
				break;

		  case 7:
				dataWrite(0x30);
				dataWrite(0x37);                      // Send each character to the function datawrite()
				break;

		  case 8:
				dataWrite(0x30);
				dataWrite(0x38);                      // Send each character to the function datawrite()
				break;

		  case 9:
				dataWrite(0x30);
				dataWrite(0x39);                      // Send each character to the function datawrite()
				break;

		  case 10:
				dataWrite(0x30);                      // Send each character to the function datawrite()
				dataWrite(0x31);
				break;

		  case 11:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x32);
				break;

		  case 12:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x33);
				break;

		  case 13:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x34);
				break;

		  case 14:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x35);
				break;

		  case 15:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x36);
				break;

		  case 16:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x37);
				break;

		  case 17:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x38);
				break;

		  case 18:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x39);
				break;

		  case 19:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x30);
				break;

		  case 20:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x31);
				break;

		  case 21:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x32);
				break;

		  case 22:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x33);
				break;

		  case 23:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x4);
				break;

		  case 24:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x35);
				break;

		  case 25:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x36);
				break;
		  case 26:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x37);
				break;
		  case 27:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x38);
				break;
		  case 28:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x39);
				break;
		  case 29:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x30);
				break;
		  case 30:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x31);
				break;
		  case 31:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x32);
				break;
		  case 32:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x33);
				break;
		  case 33:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x34);
				break;
		  case 34:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x35);
				break;

		  }

		instructionWrite(0xCE);
		delay_micro(37);
		  switch(ewTocks){
		  case 1:
				dataWrite(0x30);
				dataWrite(0x31);                      // Send each character to the function datawrite()
				break;

		  case 2:
				dataWrite(0x30);
				dataWrite(0x32);                      // Send each character to the function datawrite()
				break;

		  case 3:
				dataWrite(0x30);
				dataWrite(0x33);                      // Send each character to the function datawrite()
				break;

		  case 4:
				dataWrite(0x30);
				dataWrite(0x34);                      // Send each character to the function datawrite()
				break;

		  case 5:
				dataWrite(0x30);
				dataWrite(0x35);                      // Send each character to the function datawrite()
				break;

		  case 6:
				dataWrite(0x30);
				dataWrite(0x36);                      // Send each character to the function datawrite()
				break;

		  case 7:
				dataWrite(0x30);
				dataWrite(0x37);                      // Send each character to the function datawrite()
				break;

		  case 8:
				dataWrite(0x30);
				dataWrite(0x38);                      // Send each character to the function datawrite()
				break;

		  case 9:
				dataWrite(0x30);
				dataWrite(0x39);                      // Send each character to the function datawrite()
				break;

		  case 10:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x31);
				break;

		  case 11:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x32);
				break;

		  case 12:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x33);
				break;

		  case 13:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x34);
				break;

		  case 14:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x35);
				break;

		  case 15:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x36);
				break;

		  case 16:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x37);
				break;

		  case 17:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x38);
				break;

		  case 18:
				dataWrite(0x31);                      // Send each character to the function datawrite()
				dataWrite(0x39);
				break;

		  case 19:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x30);
				break;

		  case 20:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x31);
				break;

		  case 21:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x32);
				break;

		  case 22:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x33);
				break;

		  case 23:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x4);
				break;

		  case 24:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x35);
				break;

		  case 25:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x36);
				break;

		  case 26:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x37);
				break;

		  case 27:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x38);
				break;

		  case 28:
				dataWrite(0x32);                      // Send each character to the function datawrite()
				dataWrite(0x39);
				break;

		  case 29:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x30);
				break;

		  case 30:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x31);
				break;

		  case 31:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x32);
				break;

		  case 32:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x33);
				break;

		  case 33:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x34);
				break;

		  case 34:
				dataWrite(0x33);                      // Send each character to the function datawrite()
				dataWrite(0x35);
				break;

		  }

}
void displayTopLine(char* topLine){
	for (i = 0; topLine[i] != '\0'; i++){       // While i doesn't equal the null character increment through the top array
		 dataWrite(topLine[i]);                  // Send each character to the function datawrite();
	}
}
void displayBottomLine(char* bottomLine){
	for (i = 0; bottomLine[i] != '\0'; i++){            // While i doesn't equal the null character increment through the bottom array
		 dataWrite(bottomLine[i]);                       // Send each character to the function datawrite();
	}
}
void buzzerTone1(){
	TIMER_A0->CCR[0] = 4000;
	TIMER_A0->CCR[2] = 2000;
}
void buzzerTone2(){
	TIMER_A0->CCR[0] = 5000;;
	TIMER_A0->CCR[2] = 2500;
}
void buzzerTone3(){
	TIMER_A0->CCR[0] = 6000;
	TIMER_A0->CCR[2] = 3000;
}
void buzzOff(){
	TIMER_A0->CCR[0] = 0;
	TIMER_A0->CCR[2] = 0;
}

//TRAFFIC LIGHT / CROSSWALK FUNCTIONS
void ns_toggleYellowLED(){
	for(i = 0; i < toggle; i++){

		 checkNS_crossWalk();
		 //checkPole();

		 P6->OUT |= 0x40;
		 if(NS_crosswalk == 3){
			  NS_crosswalkFlash();
		 }
		 delay_ms(on_off);

		 P6->OUT &=~ 0x40;
		 if(NS_crosswalk == 3){
			  NS_crosswalkFlash();
		 }
		 delay_ms(on_off);
	}
}
void checkNS_crossWalk(){

	if(state == 1 && NS_crosswalk == 1){

		 P7->OUT &=~ BIT6;       //  EW GREEN Crosswalk off for safety

		 toggle = toggle + 15;
		 P7->OUT |= BIT7;            //  Turn on Green Cross walk
		 NS_crosswalk = 2;           //  Increment so it will not continue to increment the toggle time
	}
	if(NS_crosswalk == 3){
		 P7->OUT &=~ BIT7;            //  Turn off green cross
	}
	if(EW_crosswalk == 1){
		 EW_crosswalk = 2;           //  Will monitor the state and hold until light changes
		 toggle = 15;                //  Decreases the current light toggle time
	}

	if(!(P1->IN & BIT5)){
		 nsBuzzerFlag++;
	}
	if(nsBuzzerFlag > 10){
		 buzzerTone1();
	}
	if(NS_crosswalk == 3 & nsBuzzerFlag > 10){
		 buzzerTone2();
	}
	if(!(P1->IN & BIT7)){
		 ewBuzzerFlag++;
	}
}
void NS_crosswalkFlash(){
	//P7->OUT &=~ BIT7;           //  Turn off the Walk light
	P7->OUT ^= BIT6;            //  Toggle the Amber Light
}
void ew_toggleYellowLED(){          //P3->OUT |= BIT6;
	for(i = 0; i < toggle; i++){

		 checkEW_crossWalk();
		 //checkPole();

		 P3->OUT |= BIT6;
		 if(EW_crosswalk == 3){
			  EW_crosswalkFlash();
		 }
		 delay_ms(on_off);

		 P3->OUT &=~ BIT6;
		 if(EW_crosswalk == 3){
			  EW_crosswalkFlash();
		 }
		 delay_ms(on_off);
	}
}
void checkEW_crossWalk(){
	if(state == 7 && EW_crosswalk == 1){
		 P7->OUT &=~ BIT6;
		 toggle = toggle + 15;
		 P7->OUT |= BIT5;
		 EW_crosswalk = 2;
	}

	if(EW_crosswalk == 3){
		 P7->OUT &=~ BIT5;
	}

	if(NS_crosswalk == 1){
		 EW_crosswalk = 2;
		 toggle = 15;
	}

	if(!(P1->IN & BIT7)){
		 ewBuzzerFlag++;
	}
	if(ewBuzzerFlag > 10){
		 buzzerTone1();
	}
	if(EW_crosswalk == 3 & ewBuzzerFlag > 10){
		 buzzerTone2();
	}
	if(!(P1->IN & BIT7)){
		 nsBuzzerFlag++;
	}

}
void EW_crosswalkFlash(){
	//P7->OUT &=~ BIT4;           //  Turn off the Walk light
	P7->OUT ^= BIT4;            //  Toggle the Amber Light
}

//LCD CURSOR / SCREEN MANIPULATION FUNCTIONS
void nextLine(){
	instructionWrite(0xC0);                  // Pass 0xC0 as an instruction
	delay_micro(50);                       // Delay 100 microseconds
}
void cursorToHome(){
	instructionWrite(0x01);         // CLEAR DISPLAY, CUSROR TO HOME POS
	delay_micro(50);               // Must delay 100 microseconds
}
void clearScreen(){
	instructionWrite(0x01);         // CLEAR DISPLAY, CUSROR TO HOME POS
	delay_micro(50);               // Must delay 100 microseconds
}

//SYSTICK TIMER FUNCTIONS
void delay_ms(unsigned ms){
	SysTick->LOAD = (ms * 3000 - 1);                // Convert clock cycles to time in milliseconds
	SysTick->VAL = 0;                               // Any write to CVR clears it and COUNTFLAG in CSR
	while ((SysTick->CTRL & 0x00010000) == 0);      // Fancy delay
}
void delay_micro(unsigned microsec){
	SysTick->LOAD = (microsec * 3 - 1);         // Convert clock cycles to time in microseconds
	SysTick->VAL = 0;                           // Any write to CVR clears it and COUNTFLAG in CSR
	while ((SysTick->CTRL & 0x00010000) == 0);  // Fancy delay
}

//TIMER_32 ISR
void T32_INT2_IRQHandler(void){                          //Interrupt Handler for Timer 2
	 TIMER32_2->INTCLR = 1;                          //Clear interrupt flag so it does not interrupt again immediately.
	 TIMER32_2->LOAD = 3000000 - 1;                  //Set to a count down of 1 second on 3 MHz clock
	 ticks++;

//     if(p2-out == 0){
//         crosswalk satet++
//         If(crosswalk satet == 3){
//             buzztone2;
//         }
//     }

}

//GPIO ISR
void PORT1_IRQHandler(void){
	uint32_t status;

	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

		if(status & GPIO_PIN5){
		NS_crosswalk = 1;
		}

		if(status & GPIO_PIN7){
		EW_crosswalk = 1;
		}
}
void PORT2_IRQHandler(void){
	uint32_t status;

	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, status);

		if(status & GPIO_PIN6){

			 //TIMER_A0->CCR[1] = servoPos;
			 TIMER_A0->CCR[0] = 60000;                      // PWM Period (# cycles of the clock)
			 TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;     // Output mode, count up
			 TIMER_A0->CCR[1] = 8100;                      // CCR1 PWM duty cycle in 10ths of percent
			 TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;

		}

		if(status & GPIO_PIN7){
			 TIMER_A0->CCR[0] = 60000;                      // PWM Period (# cycles of the clock)
			 TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;     // Output mode, count up
			 TIMER_A0->CCR[1] = 4200;                      // CCR1 PWM duty cycle in 10ths of percent
			 TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;
		}
		if(status & GPIO_PIN3){
			 P10->OUT ^= BIT0;   //Street Lights
		}

}
void PORT4_IRQHandler(void){

}
void PORT5_IRQHandler(void){
	uint32_t status;

	status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

		if(status & GPIO_PIN2){
		safetyFlag = 1;
		}
		if(status & GPIO_PIN0){
		safetyFlag = 2;
		}

}
//SERVO FUNCTIONS
//void servoUp(){
//    TIMER_A0->CCR[1] = upServo;
//}
//void servoDown(){
//    TIMER_A0->CCR[1] = downServo;
//}

//LCD DATA PASSING FUNCTIONS
void instructionWrite(uint8_t command){
	P4OUT &= ~ BIT4;                                // RS Pin: Set RS pin as 0;  LCD expects instructions
	pushByte(command);                              // Passes the instructions to pushByte 1 byte at a time
}
void dataWrite(uint8_t data){
	P4OUT |= BIT4;                                  // RS Pin: Set RS pin as 0;  LCD expects instructions
	pushByte(data);                             // Passes the data to pushByte 1 byte at a time
	}
void PulseEnablePin(){
	P4OUT &= ~BIT5;                         // make sure pulse starts out at 0V
	delay_micro(10);                        // Must delay 10 microseconds
	P4OUT |= BIT5;                          // Send a pulse (pin 2.4 high)
	delay_micro(10);                        // Must delay 10 microseconds
	P4OUT &= ~BIT5;                         // End pulse (pin 2.4 low)
	delay_micro(10);                        // Must delay 10 microseconds
}
void pushNibble(uint8_t nibble){
	P4OUT &= ~0x0F; // Clear pins 4.0 - 4.3 just to make sure starting from all zeros
	P4OUT |= (nibble & 0x0F); // Associates the nibble to the pins only the pins with the nibbles will be turned high
	delay_ms(5);                         // Must delay 10 milliseconds
	PulseEnablePin(); // Used to send data, data is loaded and waiting for enable to be released
}
void pushByte(uint8_t byte){
	uint8_t nibble;                       // Declare the nibble variable
	nibble = (byte & 0xF0) >> 4; // Retain the MSB and shift them to the right 4 places
	pushNibble(nibble); // Send the variable nibble to pushNibble as 0000xxxx
	nibble = byte & 0x0F;                   // Retain the LSB
	pushNibble(nibble);           // Calls pushNibble and passes the LSB
}

//FRIDGE
void fridgeTone(){
	TIMER_A0->CCR[0] = 6000;
	TIMER_A0->CCR[2] = 3000;

	delay_ms(50);

	TIMER_A0->CCR[0] = 5500;
	TIMER_A0->CCR[2] = 2750;

	delay_ms(50);

	TIMER_A0->CCR[0] = 5000;
	TIMER_A0->CCR[2] = 2500;

	delay_ms(50);

	TIMER_A0->CCR[0] = 4500;
	TIMER_A0->CCR[2] = 2750;

	delay_ms(50);

	TIMER_A0->CCR[0] = 4000;
	TIMER_A0->CCR[2] = 2000;

	delay_ms(50);

	TIMER_A0->CCR[0] = 3000;
	TIMER_A0->CCR[2] = 1500;

	delay_ms(50);

	TIMER_A0->CCR[0] = 8000;
	TIMER_A0->CCR[2] = 4000;

	delay_ms(50);
	buzzOff();

}
void fridgeTone2(){
	for(i = 0; i < 3000; i++){
		 TIMER_A0->CCR[0] = 6000 - i;
		 TIMER_A0->CCR[2] = 3000 - i;
	}
	for(i = 0; i < 3000; i++){
		 TIMER_A0->CCR[0] = 6000 + i;
		 TIMER_A0->CCR[2] = 3000 + i;
	}

}

/**********************************************************************************************************************************************
//--------------------------------------------------INITIALIZATIONS----------------------------------------------------------------------------
/*********************************************************************************************************************************************/


void initializeAll(){

	SysTick_Init();                 // Called first from main because all functions use the timer
	lcdPin_Init();                  // Set all LCD pins to GPIO
	lcdDisplay_init();              // LCD initialization calls instructionoWrite and delay functions
	delay_ms(100);

	timer32_init();
	timerA_init();                  // Used to initialize the timer and update CCR1

	servoPin_init();                // Initialized the PWM pin for the Servo -> Timer A used for PWM
	stepperPin_Init();
	gpioHandler_Init();
	buzzer_init();
	trafficLightPin_Init();

}
void stepperPin_Init(){
	P5->DIR |= 0xF0;            // P6.7 set to 1 as output
	P5SEL0 &= ~0xF0;            // Clear P6.7
	P5SEL1 &= ~0xF0;            // Enable Timer A
	P5OUT &= ~0xF0;             // Turn pin to high

	//GPIO INTERUPT PINs

	P4->DIR |= BIT6;
	P4->OUT &=~ BIT6;   //North to south crossing light
	P4SEL0 &=~ BIT6;
	P4SEL1 &=~ BIT6;

}
void timer32_init(){
	TIMER32_2->CONTROL = 0b11100011;                //Sets timer 2 for Enabled, Periodic, With Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
	__enable_interrupt();
	NVIC_EnableIRQ(T32_INT2_IRQn);
	TIMER32_2->LOAD = 3000000 - 1;                  //Set to a count down of 1 second on 3 MHz clock

	TIMER32_1->CONTROL = 0b11100011;                //Sets timer 1 for Enabled, Periodic, With Interrupt, No Prescaler, 32 bit mode, One Shot Mode.  See 589 of the reference manual
	__enable_interrupt();
	NVIC_EnableIRQ(T32_INT1_IRQn);
	TIMER32_1->LOAD = 3000000 - 1;                  //Set to a count down of 1 second on 3 MHz clock

}
void timerA_init(){
	TIMER_A0->CCR[0] = 60000;                      // PWM Period (# cycles of the clock)
	TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;     // Output mode, count up
	TIMER_A0->CCR[1] = 4200;                      // CCR1 PWM duty cycle in 10ths of percent
	TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;

	TIMER_A0->CCR[0] = 0;                      // PWM Period (# cycles of the clock)
	TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;     // Output mode, count up
	TIMER_A0->CCR[2] = 0;                      // CCR1 PWM duty cycle in 10ths of percent
	TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;
}
void SysTick_Init(){
	SysTick->CTRL = 0;              // Disable SysTick durring step
	SysTick->LOAD = 0xFFFFFFFF;     // Max reload value
	SysTick->VAL = 0;               // Anywrite to current clears it
	SysTick->CTRL = 0x00000005;     // Enable SysTick, 3MHz, no interuptions
}
void lcdPin_Init(){
	P4->DIR |= 0xFF;                // Set all pins as outputs
	P4OUT &= ~0xFF;                 // Initial condition is LOW
	P4SEL0 &= ~0xFF;                //Set to 0 for GPIO
	P4SEL1 &= ~0xFF;                //Set to 0 for GPIO

}
void lcdDisplay_init(){
	instructionWrite(0x03);         // RESET SEQUENCE
	delay_ms(100);                   // Must delay 10 milliseconds
	instructionWrite(0x03);         // Must be 0x03 hex
	delay_micro(200);               // Must delay 200 microseconds
	instructionWrite(0x03);         // Must be 0x03 hex
	delay_ms(100);                   // Must delay 10 mS
	instructionWrite(0x02);         // SET TO 4 BIT MODE
	delay_micro(100);               // Must delay 100 microseconds
	instructionWrite(0x02);         // Must be 0x02 hex
	instructionWrite(0x08);         // 2 LINES, 5x8 FORMAT 40 PIXELS per CHAR
	delay_micro(100);               // Must delay 100 microseconds
	instructionWrite(0x0F);         // DISPLAY ON, CURSOR ON and BLINKING
	delay_micro(100);               // Must delay 100 microseconds
	instructionWrite(0x01);         // CLEAR DISPLAY, CUSROR TO HOME POS
	delay_micro(100);               // Must delay 100 microseconds
	instructionWrite(0x06);         // INCREMENT CURSOR
	delay_micro(100);               // Must delay 100 microseconds
}
void servoPin_init(){
	P2->DIR |= BIT4;                    // P2.4 set TA0.1 P2->SEL0 |= BIT4;
	P2->SEL0 |= BIT4;                   // Set to zero
	P2->SEL1 &=~(BIT4);                 // Enable timer (set to 1)
	P2->OUT |= BIT4;                    //turns on pin
}
void buzzer_init(){
	P2->DIR |= BIT5;                // P6.7 set to 1 as output
	P2SEL0 |= BIT5;                 // Clear P6.7
	P2SEL1 &= ~BIT5;                // Enable Timer A
	P2OUT |= BIT5;                  // Turn pin to high
}
void gpioHandler_Init(){
	volatile uint32_t ii;

	MAP_WDT_A_holdTimer();                                  //Halting the Watchdog
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);       // Configuring P1.0 as output and P1.1 (switch) as input

	MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN5 | GPIO_PIN7);      /* Configuring P1.1 as an input and enabling interrupts */
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN5 | GPIO_PIN7);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN5 | GPIO_PIN7);
	MAP_Interrupt_enableInterrupt(INT_PORT1);

	MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN3 | GPIO_PIN6 | GPIO_PIN7);      /* Configuring P1.1 as an input and enabling interrupts */
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2, GPIO_PIN3 | GPIO_PIN6 | GPIO_PIN7);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN3 | GPIO_PIN6 | GPIO_PIN7);
	MAP_Interrupt_enableInterrupt(INT_PORT2);

	MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN6);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P4, GPIO_PIN6);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN6);
	MAP_Interrupt_enableInterrupt(INT_PORT4);

	MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN2);
	MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN2);
	MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN0 | GPIO_PIN2);
	MAP_Interrupt_enableInterrupt(INT_PORT5);

	MAP_SysCtl_enableSRAMBankRetention(SYSCTL_SRAM_BANK1);                              /* Enabling SRAM Bank Retention */
	MAP_Interrupt_enableMaster();                                                       /* Enabling MASTER interrupts */

}
void trafficLightPin_Init(){
	P3->DIR |= 0xFF;
	P3->OUT &=~ 0xFF;   //East to west traffic light pins
	P3SEL0 &=~ 0xFF;
	P3SEL1 &=~ 0XFF;
	P6->DIR |= 0xFF;
	P6->OUT &=~ 0xFF;   //North to south traffic light pins
	P6SEL0 &=~ 0xFF;
	P6SEL1 &=~ 0XFF;

	// Cross walks
	P7->DIR |= BIT4;
	P7->OUT &=~ BIT4;   //NS crossing green light WORKS
	P7SEL0 &=~ BIT4;
	P7SEL1 &=~ BIT4;

	P7->DIR |= BIT5;
	P7->OUT &=~  BIT5;   ////NS crossing red light WORKS
	P7SEL0 &=~ BIT5;
	P7SEL1 &=~ BIT5;

	P7->DIR |= BIT6;
	P7->OUT &=~ BIT6;   //EW crossing light red
	P7SEL0 &=~ BIT6;
	P7SEL1 &=~ BIT6;

	P7->DIR |= BIT7;
	P7->OUT &=~ BIT7;   //North to south crossing light
	P7SEL0 &=~ BIT7;
	P7SEL1 &=~ BIT7;

	P10->DIR |= BIT0;
	P10->OUT &=~ BIT0;   //Street Lights
	P10SEL0 &=~ BIT0;
	P10SEL1 &=~ BIT0;

}


