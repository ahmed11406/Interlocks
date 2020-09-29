/* Code Starts Here...
Author: Substation Project Team
Date: 12/2019
Version: 1.0.3 OneButton per device only.
Subject: Substation Equipment Interlock Control.
Description: Implementing the Interlocking function of a system consisting of the following:
2 coupled bus bars through a bus-tie BT,
a Transformer per each bus for a total of 2 units labeled T1,T2 below,
a Generator per each bus for a total of 2 units labeled G1,G2 below.
Pins from 2-6 map to switches T1,T2,G1,G2,BT respectively.
Pins from 8-12 map to devices T1,T2,G1,G2,BT respectively.
Notes:
1)you need to specify variables (T1sw, ..etc) not writing directly to ports
because the program will check it many times for each if statement
in the following code. This way uses more space (2 bytes) but faster.
Performance: 1168 (3.6%) bytes Flash, 2 bytes (0.1%) SRAM, extremely fast
since it utilizes the interrupts of both the pins and timer for debouncing.
This way the program has only one simple assignment instruction in the main code.
*/
#include<avr/interrupt.h> // Library for Port and Timer Interrupts ISR Handling.
#include <avr/io.h>
//Register0: [0,2]OverflowCount, [3]PositiveEdge, [5:6]ValidReading.
volatile uint8_t Register0;
volatile uint8_t KeyReg;
int main(void)
{
	// Initialize ports to I/O directions:
	Register0 = 0;
	KeyReg=0x1F;
	MCUCR |= 0x10;         // 0x10 = 00010000 Disabling Pull-ups resistors.
	DDRD = ((DDRD & 0x3F) | 0x30); // 0x3F = 00111111, pins: 7,6,5 are inputs, 2,3,4 outs initially.
	PORTD = ((PIND & 0x3F) | 0x30); //Initially all inputsD are LOWs, outsD highs.
	DDRB |= 0x1F;          // Masking to perserve pins 13,14,15. (ones mark outputs).
	PORTB = (PINB & 0xC0);         // 0xC0 = 11000000 OutputsB initially are LOWS. (NO pushbutton pressed).
	SREG |= (1u << 7);   // Enabling Global Interrupt.
	PCICR  |= 0x4;    // Enabling the portD interrupt.
	PCMSK2 = ((PCMSK2 | 0xC0) & 0xFC); //Which pins? 0xC0 = B11000000; 0xFC = 11111100 Setting RX, TX to 0;
	PCIFR |= 0x4;    // Ensuring no portD interrupt initially (Reseting Flag)
	TCCR1A &= 0xFC;  //to override the default arduino settings for timer and select the normal mode.
	TCCR1B &= 0xE7;
	while (1)
	{
	}
}

ISR(PCINT2_vect)
{ // the program enters Pin Change ISR 3 times:
	/*1st: once key is pressed.
	2nd: after updating the output and return to the main loop(since inputs go from low to high again, still pressed).
	3rd: once the key is released.
	Initializing a timer in normal mode, WGM bits in TCCR1A & TCCR1B along with the prescaling bit
	should be adjusted ( WGM = 0, Prescaling=1 (No prescaller), =0 for stopping)
	(not resetting, for resetting put TCNT value to zero)
	and put initial counter TCNT value to 0.
	it counts ,once clocked, from the given written TCNT value to 0 once then start from 0 to max.
	*/
	Register0 &= 0xF8; // overflow to 0;
	TIMSK1 |= 0x1;   // Enabling Timer Interrupt.
	TCCR1B |= 0x1;   // Timer Clocking: (1/(16 MHz)) second/clock * 2^16 clocks to overflow =~ 4 ms.
	/*
	Detection complete and the timer Enabled, scaled for debouncing, and reset.
	Disabling the PINs interrupt for now frees the CPU from over-interrupting needlessly
	and must be closed so the Positive edge detection can work properly.
	*/
	PCMSK2 ^= 0xC0;
	TCNT1 = 0;    //Initial timer value..

}
ISR(TIMER1_OVF_vect)
{
	/* ACTIVATED FROM THE PIN CHANGE INTERRUPT.
	The Keyboard scanning starts after Positive is detected and the bouncing has finished.
	The scan method is:
	1) toggle the DDRD of input pin driven high (7 or 6 or 5) to output sending high.
	2) toggle the DDRD of pins (4, 3, 2) to inputs.
	3) Read the PORTD of pins (4, 3, 2) respectively.
	4) the following conditions yield the pressed key:
	7+4 = 1, 7+3=2, 7+2=3
	6+4 = 4, 6+3=5, 6+2=6
	5+4 = 7, 5+3=8, 5+2=9
	if all false -> Key 10 pressed.
	5) if the respective keys is pressed check the output PORTB and realize the logic equations:
	for Transformer-1: (1,2= T1on,T1off) maps to ( G1`.(G2`+BT`), zero)
	for Transformer-2: (3,4 T2on,T2off)  maps to ( G2`.(G1`+BT`), zero)
	for Generator-1: (5,6 G1on,G1off)    maps to ( T1`.(G2`.T2` + BT`), zero)
	for Generator-2: (7,8 G2on,G2off)    maps to ( T2`.(G1`.T1` + BT`), zero)
	for Bus Tie-1: (9,10 B.Ton,B.Toff)   maps to ( G1`.G2`+ T1`.T2`.(G1`+G2`), zero)
	IMPORTANT NOTE: if only key is present for start and stop, we can maintain a KeyStateRegister global variable
	(a single byte unsigned int has 8 bits= 8 flags for 8 keys, used as a Flag register-
	and manipulate each bit individually through masks.) that is initially 0,
	and XORed with 1 after the key is pressed and the timer interrupt begins and press debounced,
	for ON, 1st press, it's 0 ^ 1 = 1. for OFF, 2nd press, it's 1 ^ 1 = 0.
	followed by an if condition to detect the value of the Key Flag, if it's 1, then  execute the ON-code.
	if it's 0, then execute the OFF-code.
	6) Toggle back the DDRD of pins (7 or 6 or 5) to inputs and DDRD of (4,3,2) to outputs.
	7) set the PORTD of pins (4,3,2) to high again.
	finally the timer is stopped, reset and Disabled.
	The PIN Change is Re-enabled and its flag is cleared
	to ensure no-false interrupts because of manipulating pins above.
	By debouncing of the key release done in 3st PINCT INT, The code is reset and ready for the next key press.
	*/
	if (((~Register0) & (1u<<3)) && ((Register0 & 0x07) == 4)) {
		// 3 Overflows = ~12ms for clearing the bouncing.
		uint8_t portd = (PIND & 0xC0);
		uint8_t portb = (PINB & 0x1F);
		Register0 |= portd>>1; //For setting in validReading.
		Register0 &= ((portd>>1) | 0x9F); //for  resetting in validReading.
		uint8_t T1sw = ((~portb) & (1u << 0));
		uint8_t T2sw = ((~portb) & (1u << 1));
		uint8_t G1sw = ((~portb) & (1u << 2));
		uint8_t G2sw = ((~portb) & (1u << 3));
		uint8_t BTsw = ((~portb) & (1u << 4));
		if (portd & (1u << 7)) {
			DDRD = ((DDRD & 0xCF) | (1u << 7));
			PORTD = (PIND | (1u << 7));
			portd = (PIND & 0x30);
			if (portd & (1u << 5)) {
				//Key_1: T1
				if (KeyReg & (1u))
				{
					// T1 ON
					if ( G1sw && (G2sw || BTsw) ) {
						portb |= 1u;
						KeyReg ^= 1u;
					}
					
				}
				else if (!(KeyReg & (1u)))
				{
					//T1 OFF
					portb  &= ~(1u);
					KeyReg ^= 1u;
				}

			}
			else if (portd & (1u << 4)) {
				//Key_2: T2
				if (KeyReg & (1u<<1))
				{
					// T2 ON
					if ( G2sw && (G1sw || BTsw) ) {
						portb |= (1u << 1);
						KeyReg ^= 1u<<1;
					}
				}
				else if (!(KeyReg & (1u<<1)))
				{
					// T2 OFF
					portb  &= ~(1u << 1);
					KeyReg ^= 1u<<1;
				}
			}
			else  {
				//Key_3: G1
				if (KeyReg & (1u<<2))
				{
					// G1 ON
					if ( T1sw && ((G2sw && T2sw) || BTsw) ) {
						portb |= (1u << 2);
						KeyReg ^= 1u<<2;
					}
				}
				else if (!(KeyReg & (1u<<2)))
				{
					// G1 OFF
					portb  &= ~(1u << 2);
					KeyReg ^= 1u<<2;
				}
			}
		}
		else if (portd & (1u << 6)) {
			DDRD = ((DDRD & 0xCF) | (1u << 6));
			PORTD = (PIND | (1u << 6));
			portd = (PIND & 0x30);
			if (portd & (1u << 5)) {
				//Key_4: G2
				if (KeyReg & (1u<<3))
				{
					// G2 ON
					if ( T2sw && ((G1sw && T1sw) || BTsw)  ) {
						portb |= (1u << 3);
						KeyReg ^= 1u<<3;
					}
				}
				else if (!(KeyReg & (1u<<3)))
				{
					// G2 OFF
					portb  &= ~(1u << 3);
					KeyReg ^= 1u<<3;
				}
			}
			else if (portd & (1u << 4)) {
				//Key_5: BT
				if (KeyReg & (1u<<4))
				{
					// BT ON
					if ((G1sw && G2sw) || ((T1sw && T2sw) && (G1sw || G2sw))) {
						portb |= (1u << 4);
						KeyReg ^= 1u<<4;
					}
					
				}
				else if (!(KeyReg & (1u<<4)))
				{
					// BT OFF
					portb  &= ~(1u << 4);
					KeyReg ^= 1u<<4;
				}
			}
		}
		DDRD = ((DDRD & 0x3F) | 0x30); //0x3F = 00011111, pins: 7,6,5 are inputs, 2,3,4 outputs.
		PORTD = ((PIND & 0x3F) | 0x30); //0x30=00110000 all inputsD are LOWs, outsD highs.
		PORTB = portb; // update the output if pressed.
		TCCR1B &= 0xF8;// B11111000 = Stopping(Un-clocking) the Timer.
		TIMSK1 ^= 0x1; // Disabling Timer Interrupt.
		TCNT1 = 0;     // Resetting for future use elsewhere in the program.
		PCMSK2 |= 0xC0;// Re-enabling the PINs (7,6,5) change interrupt.
		PCIFR |= 0x4; // Clearing the PINS flag.
		Register0 |= (1u<<3); // setting positiveEdge to 1.
	}
	else if ( (Register0 & (1u<<3)) && ((Register0 & 0x70) == ((PIND & 0xC0)>>1)) ){
		//Second-Entry once the it returned back from 1st entry.
		TCCR1B &= 0xF8;// B11111000 = Stopping(Un-clocking) the Timer.
		TIMSK1 ^= 0x1; // Disabling Timer Interrupt.
		TCNT1 = 0;     // Resetting for future use elsewhere in the program.
		PCMSK2 |= 0xC0;// Re-enabling the PINs (7,6,5) change interrupt.
		PCIFR |= 0x4; // Clearing the PINS flag.
	}
	else if ( (Register0 & (1u<<3)) && ((Register0 & 0x07) == 4) && ((Register0 & 0x70) != ((PIND & 0xC0)>>1)) ) {
		//Release Debouncing: 3rd entry upon releasing the key by 20ms releasing debouncing period.
		Register0 &= ~(1u<<3); // reset positiveEdge to zero.
		Register0 |= (PIND & 0xC0)>>1;   //For setting in validReading.
		Register0 &= (((PIND & 0xC0)>>1) | 0x9F); //for  resetting in validReading.
		TCCR1B &= 0xF8;// B11111000 = Stopping(Un-clocking) the Timer.
		TIMSK1 ^= 0x1; // Disabling Timer Interrupt.
		TCNT1 = 0;     // Resetting for future use elsewhere in the program.
		PCMSK2 |= 0xC0;// Re-enabling the PINs (7,6,5) change interrupt.
		PCIFR |= 0x4; // Clearing the PINS flag.
	}
	
	else {
		Register0 += 1;
	}
}
//Code Ends Here...

