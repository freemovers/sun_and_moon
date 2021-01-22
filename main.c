/*
* SunAndMoon.c
*
* Created: 10/17/2020 11:57:47 AM
* Author : Sander (California STEAM)
*
* LED1: PA1 (2) HIGH, PA2 (3) LOW      b0110 0x06, b0010 0x02
* LED2: PA1 (2) LOW , PA2 (3) HIGH     b0110 0x06, b0100 0x04
* LED3: PA2 (2) HIGH, PA3 (4) LOW      b1100 0x0C, b0100 0x04
* LED4: PA2 (2) LOW , PA3 (4) HIGH     b1100 0x0C, b1000 0x08
* LED5: PA1 (3) HIGH, PA3 (4) LOW      b1010 0x0A, b0010 0x02
*/

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "main.h"

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
#define USERROW_OFFSET	(0x1400)

int8_t buttonpressed = 0;
int8_t buttonStillPressed = 0;
uint8_t buttonTimePressed = 0;
int8_t enterSleepMode = 0;


/*left
int8_t ledPinDir[] = {
	0x06, 0x06, 0x0C, 0x0C, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //single led control (0 .. 15)
	0x0E,		// LED 1 AND 5 (16)
	0x0E,		// LED 2 AND 3 (17)
	0x0E,		// LED 4 AND 6 (18)
	0x0E,		// LED 3 AND 5 (19)
};

int8_t ledPinOut[] = {
	0x02, 0x04, 0x04, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //single led control (0 .. 15)
	0x02,		// LED 1 AND 5 (16)
	0x04,		// LED 2 AND 3 (17)
	0x08,		// LED 4 AND 6 (18)
	0x06,		// LED 3 AND 5 (19)
};
*/

int8_t ledPinDir[] = {
	0x06, 0x06, 0x0A, 0x0A, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //single led control (0 .. 15)
	0x0E,		// LED 1 AND 5 (16)
	0x0E,		// LED 2 AND 3 (17)
	0x0E,		// LED 4 AND 6 (18)
	0x0E,		// LED 3 AND 5 (19)
};

int8_t ledPinOut[] = {
	0x02, 0x04, 0x02, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //single led control (0 .. 15)
	0x02,		// LED 1 AND 5 (16)
	0x04,		// LED 2 AND 3 (17)
	0x08,		// LED 4 AND 6 (18)
	0x06,		// LED 3 AND 5 (19)
};

int8_t fadeLed = 1;
int8_t fadeIn = 1;
int8_t fadeComplete = 1;
int8_t temp_pin = 0;
uint8_t endOfPattern = 1;
int8_t randomPatternNumber = 0;
int8_t randomPlayTime = 2;

typedef void (*SimplePatternList[])();		// List of patterns to cycle through.  Each is defined as a separate function below.
SimplePatternList gPatterns = { spin, pingpong, all, doublespin, sparkle, randomPattern };
uint8_t gCurrentPatternNumber = 0;			// Index number of which pattern is current

void RTC_init(void)
{
	RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;		// 32.768kHz Internal Crystal Oscillator (INT32K)
	while (RTC.STATUS > 0);				// Wait for all register to be synchronized
	RTC.PER = 0xff;					// Max for overflow (128 Hz, 7.78ms)
	RTC.CMP = 0x08;					// Compare at 244us
	RTC.CNT = 0x0;
	RTC.INTCTRL |= RTC_OVF_bm;			// Enable overflow Interrupt which will trigger ISR
	RTC.INTCTRL |= RTC_CMP_bm;			// Enable compare Interrupt which will trigger ISR
	RTC.CTRLA = RTC_PRESCALER_DIV1_gc		// 32768 / 1 = 32768 (Hz)
	| RTC_RTCEN_bm					// Enable: enabled
	| RTC_RUNSTDBY_bm;				// Run In Standby: enabled

	while (RTC.STATUS > 0 || RTC.PITSTATUS);	// Wait for all register to be synchronized
	RTC.PITINTCTRL = RTC_PI_bm;			// Periodic Interrupt: enabled
	RTC.PITCTRLA = RTC_PERIOD_CYC512_gc		// 32768 / 512 = 64Hz, 15.6ms
	| RTC_PITEN_bm;							// Enable: enabled
}

ISR(RTC_CNT_vect)
{
	if (RTC.INTFLAGS & RTC_CMP_bm)	{		// Interrupt created by the
		RTC.INTFLAGS = RTC_CMP_bm;          	// Clear flag by writing '1':
		PORTA.DIR = 0x00;                   	// Set all pin to inputs to turn off LED's
	}
	if (RTC.INTFLAGS & RTC_OVF_bm) {
		RTC.INTFLAGS = RTC_OVF_bm;          	// Clear flag by writing '1':
		PORTA.DIR = ledPinDir[fadeLed];     	// PA1 & PA2 outputs
		PORTA_OUT = ledPinOut[fadeLed];     	// PA1 SET
	}
}

ISR(RTC_PIT_vect)
{
	RTC.PITINTFLAGS = RTC_PI_bm;			// Clear flag by writing '1'
	
	if(enterSleepMode > 15) sleepPattern();
	else if(buttonStillPressed) showPattern();
	else gPatterns[gCurrentPatternNumber]();
	
	if(!(PORTA.IN & PIN6_bm)) push_button_action();
	else {
		if(enterSleepMode > 15) {			// enter sleep mode
			enterSleepMode++;				// Debounce the release of the button
			if(enterSleepMode > 20) {
				sleep_device();
			}
		}
		else if(enterSleepMode > 1) {
			enterSleepMode++;				// Debounce the release of the button
			if(enterSleepMode > 8) {
				wake_device();
			}
		}
		else {
			buttonStillPressed = 0;	
		}
	}
}

void push_button_action(void)
{
	if(!buttonStillPressed)	{
		buttonpressed++;
		buttonTimePressed = 0;
		// add one to the current pattern number, and wrap around at the end
		gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
		RTC.CMP = 0x08;						// Reset to avoid overflow on some patterns
	}
	buttonStillPressed = 1;
	buttonTimePressed++;
	if(buttonTimePressed > 128) enterSleepMode = 16;
}

void sleep_device(void)
{
	RTC.PITINTCTRL &= ~(RTC_PITEN_bm);		// stop the PIT in sleep mode to conserve energy
	gCurrentPatternNumber = ((gCurrentPatternNumber - 1) + ARRAY_SIZE( gPatterns)) % ARRAY_SIZE( gPatterns);
	enterSleepMode = 2;
	buttonTimePressed = 0;
	PORT_init();
	PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(&USERROW.USERROW0 - USERROW_OFFSET), 0x01);
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(&USERROW.USERROW1 - USERROW_OFFSET), gCurrentPatternNumber);
	fadeLed =15;
	PORTA.DIR = 0x00;
	PORTA.OUT = 0x00;
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void wake_device(void)
{
	enterSleepMode = 0;
	for (uint8_t pin = 1; pin < 3; pin++) {
		(&PORTA.PIN0CTRL)[pin] = PORT_ISC_INPUT_DISABLE_gc; // Disable input buffer and pull-up resistor on PAx output pins to conserve energy
	}
	PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
}

// Wake up routine
ISR(PORTA_PORT_vect)
{
	PORTA.INTFLAGS = PIN6_bm;				// clear interrupt flag
	PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
	RTC.PITINTCTRL = RTC_PI_bm;	
	set_sleep_mode(SLEEP_MODE_STANDBY);		// Set sleep mode to IDLE mode
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t*)(&USERROW.USERROW0 - USERROW_OFFSET), 0x00);
}


void PORT_init(void)
{
	for (uint8_t pin = 0; pin < 8; pin++) {
		(&PORTA.PIN0CTRL)[pin] = PORT_PULLUPEN_bm | PORT_ISC_INPUT_DISABLE_gc; // Disable input buffer and enable the internal pull-up on PAx pins to conserve energy
	}
	PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;
}

int main(void)
{
	RTC_init();
	PORT_init();
	set_sleep_mode(SLEEP_MODE_STANDBY);		// Set sleep mode to STANDBY mode
	sleep_enable();
	sei();
	eeprom_busy_wait();
	if(eeprom_read_byte((uint8_t*)(&USERROW.USERROW0 - USERROW_OFFSET))) sleep_device();
	else wake_device();

	while (1) {
		sleep_cpu();						// Nothing to do here, sleep the cpu
	}
}

void spin(void)
{
	RTC.CMP = 0xEF;
	if(fadeComplete++>3) {
		fadeLed++;
		if(fadeLed > 10) {
			fadeLed = 0;
			endOfPattern++;
		}
		fadeComplete = 0;
	}
}

void pingpong(void)
{
	RTC.CMP = 0xEF;
	if(fadeComplete++>3) {
		if (fadeIn) {
			fadeLed++;
			if(fadeLed >= 4) fadeIn = 0;
		}
		else {
			fadeLed--;
			if(fadeLed <= 0) {
				fadeIn = 1;
				endOfPattern++;
			}
		}
		fadeComplete = 0;
	}
}

void all(void)
{
	RTC.CMP = 0x10;
	if(fadeLed < 15 || fadeLed > 17) fadeLed = 15;
	if(fadeComplete++>4) endOfPattern++;
	fadeLed++;
}

void doublespin(void)
{
	int8_t reversePin[] = {
		4, 3, 2, 1, 0,
	};
	RTC.CMP = 0xEF;
	if(fadeComplete++>3) {
		temp_pin++;
		if(temp_pin > 10) {
			temp_pin = 0;
			endOfPattern++;
		}
		fadeComplete = 0;
	}
	if (fadeIn) {
		fadeIn = 0;
		fadeLed = temp_pin;
	}
	else {
		fadeIn = 1;
		if(temp_pin<5) fadeLed = reversePin[temp_pin];
	}
}

void sparkle(void)
{
	if (!fadeComplete) {
		if (fadeIn) {
			if (RTC.CMP > 0xEF) {
				fadeIn = 0;
			}
			else {
				RTC.CMP += 0x0A;
			}
		}
		else {
			if (RTC.CMP < 0x10) {
				fadeIn = 1;
				fadeComplete = 1;
				PORTA.DIR = 0x00;			// Set all pin to inputs to turn off LED's
				endOfPattern++;
			}
			else {
				RTC.CMP -= 0x0A;
			}
		}
	}
	else { // select a new pattern
		fadeLed = rand() % 0x0A;
		fadeComplete = 0;
	}
}

void randomPattern(void)
{
	if(endOfPattern > randomPlayTime) {
		randomPatternNumber = rand() % 0x05;
		randomPlayTime = 1 + (rand() % 0x04);
		endOfPattern = 0;
	}
	gPatterns[randomPatternNumber]();
}

void sleepPattern(void)
{
	RTC.CMP = 0xEF;
	if(fadeComplete++>5) {
		fadeLed = gCurrentPatternNumber;
	}
	if(fadeComplete>10) {
		fadeLed = 5;
		fadeComplete = 0;
	}
}

void showPattern(void)
{
	RTC.CMP = 0xEF;
	if(gCurrentPatternNumber<5)	fadeLed = gCurrentPatternNumber;
	else fadeLed = 19;
}
