// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// Linked data structure
struct State {
  unsigned long Out_6LED;  // West green South red 0x0C ,West yellow South red 0x14, South green West red 0x21, South yellow West red 0x22
	// both red 0x24
	unsigned long Out_Walk;  // walk green 0x08, don't walk red 0x02, OFF 0x00
  unsigned long Time;  
  unsigned long Next[18];}; 
typedef const struct State STyp;

// I/O
#define LED_6 (*((volatile unsigned long *)0x400053FC))// PORTB bits 0-5
#define Input_Sensors (*((volatile unsigned long *)0x4002401C))//PORTE bits 0&1&2
#define Walk_lights (*((volatile unsigned long *)0x40025028))//PORTF bits 1&3
	
// walk macros
#define Green 0x08
#define Red 0x02
#define OFF 0x00
// define states
#define GoWest 0
#define WaitWest 1
#define GoSouth 2
#define WaitSouth 3
#define WestOff_Walk 4
#define WestOff_Flash_Off_1 5
#define WestOff_Flash_On_1 6
#define WestOff_Flash_Off_2 7
#define WestOff_Flash_On_2 8
#define WestOff_Flash_Off_3 9
#define WestOff_Flash_On_3 10
#define SouthOff_Walk 11
#define SouthOff_Flash_Off_1 12
#define SouthOff_Flash_On_1 13
#define SouthOff_Flash_Off_2 14
#define SouthOff_Flash_On_2 15
#define SouthOff_Flash_Off_3 16
#define SouthOff_Flash_On_3 17
// time definition
#define time_state 100
#define time_wait  50
#define time_flash 25

STyp FSM[18]={
{0x0C,Red,time_state,{GoWest, GoWest, WaitWest, WaitWest, WaitWest, WaitWest, WaitWest, WaitWest}}, 
{0x14,Red,time_wait,{GoSouth, GoSouth, GoSouth, GoSouth, WestOff_Walk, WestOff_Walk, WestOff_Walk, WestOff_Walk}},
{0x21,Red,time_state,{GoSouth, WaitSouth, GoSouth, WaitSouth, WaitSouth, WaitSouth, WaitSouth, WaitSouth}},
{0x22,Red,time_wait,{GoWest,GoWest,GoWest,GoWest,SouthOff_Walk,SouthOff_Walk,SouthOff_Walk,SouthOff_Walk}},
{0x24,Green,time_state,{WestOff_Walk,WestOff_Flash_Off_1,WestOff_Flash_Off_1,WestOff_Flash_Off_1,WestOff_Walk,WestOff_Flash_Off_1,WestOff_Flash_Off_1,WestOff_Flash_Off_1}},
{0x24,Red,time_flash,{WestOff_Flash_On_1,WestOff_Flash_On_1,WestOff_Flash_On_1,WestOff_Flash_On_1,WestOff_Flash_On_1,WestOff_Flash_On_1,WestOff_Flash_On_1,WestOff_Flash_On_1}},
{0x24,OFF,time_flash,{WestOff_Flash_Off_2,WestOff_Flash_Off_2,WestOff_Flash_Off_2,WestOff_Flash_Off_2,WestOff_Flash_Off_2,WestOff_Flash_Off_2,WestOff_Flash_Off_2,WestOff_Flash_Off_2}},
{0x24,Red,time_flash,{WestOff_Flash_On_2,WestOff_Flash_On_2,WestOff_Flash_On_2,WestOff_Flash_On_2,WestOff_Flash_On_2,WestOff_Flash_On_2,WestOff_Flash_On_2,WestOff_Flash_On_2}},
{0x24,OFF,time_flash,{WestOff_Flash_Off_3,WestOff_Flash_Off_3,WestOff_Flash_Off_3,WestOff_Flash_Off_3,WestOff_Flash_Off_3,WestOff_Flash_Off_3,WestOff_Flash_Off_3,WestOff_Flash_Off_3}},
{0x24,Red,time_flash,{WestOff_Flash_On_3,WestOff_Flash_On_3,WestOff_Flash_On_3,WestOff_Flash_On_3,WestOff_Flash_On_3,WestOff_Flash_On_3,WestOff_Flash_On_3,WestOff_Flash_On_3}},
{0x24,OFF,time_flash,{GoSouth,GoWest,GoSouth,GoSouth,WestOff_Walk,GoWest,GoSouth,GoSouth}},
{0x24,Green,time_state,{SouthOff_Walk,SouthOff_Flash_Off_1,SouthOff_Flash_Off_1,SouthOff_Flash_Off_1,SouthOff_Walk,SouthOff_Flash_Off_1,SouthOff_Flash_Off_1,SouthOff_Flash_Off_1}},
{0x24,Red,time_flash,{SouthOff_Flash_On_1,SouthOff_Flash_On_1,SouthOff_Flash_On_1,SouthOff_Flash_On_1,SouthOff_Flash_On_1,SouthOff_Flash_On_1,SouthOff_Flash_On_1,SouthOff_Flash_On_1}},
{0x24,OFF,time_flash,{SouthOff_Flash_Off_2,SouthOff_Flash_Off_2,SouthOff_Flash_Off_2,SouthOff_Flash_Off_2,SouthOff_Flash_Off_2,SouthOff_Flash_Off_2,SouthOff_Flash_Off_2,SouthOff_Flash_Off_2}},
{0x24,Red,time_flash,{SouthOff_Flash_On_2,SouthOff_Flash_On_2,SouthOff_Flash_On_2,SouthOff_Flash_On_2,SouthOff_Flash_On_2,SouthOff_Flash_On_2,SouthOff_Flash_On_2,SouthOff_Flash_On_2}},
{0x24,OFF,time_flash,{SouthOff_Flash_Off_3,SouthOff_Flash_Off_3,SouthOff_Flash_Off_3,SouthOff_Flash_Off_3,SouthOff_Flash_Off_3,SouthOff_Flash_Off_3,SouthOff_Flash_Off_3,SouthOff_Flash_Off_3}},
{0x24,Red,time_flash,{SouthOff_Flash_On_3,SouthOff_Flash_On_3,SouthOff_Flash_On_3,SouthOff_Flash_On_3,SouthOff_Flash_On_3,SouthOff_Flash_On_3,SouthOff_Flash_On_3,SouthOff_Flash_On_3}},
{0x24,OFF,time_flash,{GoWest,GoWest,GoSouth,GoWest,SouthOff_Walk,GoWest,GoSouth,GoWest}}
};

unsigned long S;  // index to the current state 
unsigned long Input; 
// ***** 3. Subroutines Section *****
void PORTB_Init(void){
	
	//  GPIO_PORTB_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port B
	
// 2.Disable Analog function
GPIO_PORTB_AMSEL_R&=~0x0000003F; // disable analog on PB0-5
// 3.Disable alt functions
GPIO_PORTB_AFSEL_R&=~0x0000003F;
	
		GPIO_PORTB_PCTL_R &= ~0x00FFFFFF;   // 4) PCTL GPIO on PB0-5
	
// 4.Set up the direction for each pin on the GPIO
 GPIO_PORTB_DIR_R |= 0x3F; // set  PB0-5 as Output
// 5.Enable GPIO pins as digital I/Os
GPIO_PORTB_DEN_R|=0x3F;

}


void PORTE_Init(void){

	 GPIO_PORTE_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port E
	
// 2.Disable Analog function
GPIO_PORTE_AMSEL_R&=~0x00000007; // disable analog on PE0 & PE1 & PE2
// 3.Disable alt functions
GPIO_PORTE_AFSEL_R&=~0x00000007; 
	
	GPIO_PORTE_PCTL_R &= ~0x000000FF;   // 4) PCTL GPIO on PE0 & PE1 & PE2
	
// 4.Set up the direction for each pin on the GPIO
GPIO_PORTE_DIR_R=0x00000000; // set  PE0 & PE1 & PE2 as Input
// 5.Enable GPIO pins as digital I/Os
GPIO_PORTE_DEN_R=0x00000007;

}
void PORTF_Init(void){

	  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
	
// 2.Disable Analog function
GPIO_PORTF_AMSEL_R&=~0x0000000A; // disable analog on PF1 & PF3
// 3.Disable alt functions
GPIO_PORTF_AFSEL_R&=~0x0000000A; 
	
		GPIO_PORTF_PCTL_R &= ~0x000000FF;   // 4) PCTL GPIO on PF4-0
	
// 4.Set up the direction for each pin on the GPIO
GPIO_PORTF_DIR_R|=0x0000000A; // set  PF1 & PF3 as output
// 5.Enable GPIO pins as digital I/Os
GPIO_PORTF_DEN_R=0x0000000A;

}

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = 0x00FFFFFF;        // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it             
  NVIC_ST_CTRL_R = 0x00000005;          // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 10000us equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

int main(void){ 
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	// 1.Set up the system clock
	volatile unsigned long delay;
	SYSCTL_RCGC2_R|=0x32; 		/* enable clock for Ports B&E&F */
	delay = SYSCTL_RCGC2_R;           // allow time for clock to start
	//initialize ports
	PORTB_Init();
  PORTE_Init();
	PORTF_Init();
	
	// initialize Systick
	SysTick_Init();
	S=GoWest;	//initialize state
  EnableInterrupts();
	
  while(1){
		// machine driver
		//1.outputs
		 LED_6 = FSM[S].Out_6LED; // 6 LED output 
		 Walk_lights = FSM[S].Out_Walk; // walk lights output
		//2.wait
     SysTick_Wait10ms(FSM[S].Time); // wait
		//3.Input
		 Input = Input_Sensors;
		//4.next state
		 S = FSM[S].Next[Input];
  }
}

