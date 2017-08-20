/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

/**
 * Import Libraries from LPC17
 */
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

/**
 * Import Libraries from Baseboard
 */
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "light.h"

/**
 * Import Libraries from C
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * Define Constants
 */
#define LIGHT_LOW_WARNING 50 // In Lux
#define TEMP_HIGH_WARNING 26 // In Celcius

/**
 * Define the Limits
 */
const uint32_t interruptDarkLowerLimit = LIGHT_LOW_WARNING;		// Interrupt to occur when below this warning
const uint32_t interruptDarkUpperLimit = 3891;					// In accordance to the data sheet
const uint32_t interruptLightLowerLimit = 0;
const uint32_t interruptLightUpperLimit = LIGHT_LOW_WARNING - 1;// Interrupt to occur when above this warning

/**
 * Define the two different types of mode
 */
typedef enum{
	MODE_STABLE, MODE_MONITOR
} system_mode;

/**
 * Define the 4 types of warnings
 */
typedef enum{
	MOVEMENT_IN_LOW_LIGHT, HIGH_TEMPERATURE, HIGH_TERMPATURE_AND_MOVEMENT_IN_LOW_LIGHT, NONE
} warning_issued;

/***** Initialize Variables *****/
// System
volatile system_mode mode = MODE_STABLE;
volatile warning_issued warning;
uint32_t msTicks = 0;
uint8_t isFirstTimeEnterStable = 1;
uint8_t isFirstTimeEnterMonitor = 1;

// Light Interrupts
int lightLowWarning = 0;

// Timer Interrupt
int oneSecondHasReached = 0;

// RGB
uint8_t RGB_RED_AND_BLUE = 0x03;
int onOrOff = 1;

// 7 Segment
int segCount = 0;
const char displayValues[] = "0123456789ABCDEF";

// SwitchButton 4
uint8_t sw4 = 0;
uint32_t sw4PressTicks;

// Initialize OLED
char OLED_TEMPERATURE[15];
char OLED_LIGHT[15];
char OLED_X[15];
char OLED_Y[15];
char OLED_Z[15];

// Initialization of Accelerometer Variables
int32_t xoff = 0;
int32_t yoff = 0;
int32_t zoff = 0;
int8_t x = 0;
int8_t y = 0;
int8_t z = 0;
int8_t prevX = 0;
int8_t prevY = 0;
int8_t prevZ = 0;
int isThereMovement = 0;

// UART
const char messageEnterMonitor[] = "Entering MONITOR mode.\r\n";
unsigned char displayValuesToUART[100] = "";
unsigned char displayHighTempWarning[] = "Fire was Detected.\r\n";
unsigned char displayMovementInLowLight[] = "Movement in Darkness was Detected.\r\n";
int message = 0;

// Light and Temperature Values
uint32_t light = 0;
volatile uint32_t temperature = 0;

// Temperature Variables
uint8_t tempState = 0;
uint32_t temp1 = 0;
uint32_t temp2 = 0;

/***** End of Variables *****/

/**
 * Run Warnings
 * High Temperature: 		Blink Red
 * Movement and Low Light: 	Blink Blue
 * Do note that both can occur at the same time
 */
void runWarning(){
	if(warning != NONE){
		if(onOrOff == 1){
			switch(warning){
				case HIGH_TERMPATURE_AND_MOVEMENT_IN_LOW_LIGHT:
					rgb_setLeds(RGB_RED_AND_BLUE);
				break;

				case HIGH_TEMPERATURE:
					rgb_setLeds(RGB_RED);
				break;

				case MOVEMENT_IN_LOW_LIGHT:
					rgb_setLeds(RGB_BLUE);
				break;

				case NONE:
				default: // Fall Through
				break;

			}
		} else {
			rgb_setLeds(0);
		}
		onOrOff = !onOrOff; // Change the state
	} else {
		rgb_setLeds(0);
	}
}

/**
 * Define the pins used for UART
 */
void pinsel_uart3(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

/**
 * Initialize UART terminal
 */
void init_uart(void){
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;
    //pin select for uart3;
    pinsel_uart3();
    //supply power & setup working parameters for uart3
    UART_Init(LPC_UART3, &uartCfg);
    //enable transmit for uart3
    UART_TxCmd(LPC_UART3, ENABLE);
}

/**
 * Initialization of i2c
 */
static void init_i2c(void){
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

/**
 * Initialize Standard Private SSP Interrupt handler
 */
static void init_ssp(void){
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

/**
 * Initialization of GPIO
 */
static void init_GPIO(void){
	// Initialize button SW4 (not really necessary since default configuration)
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);
}

/**
 * Function to initialize the light interrupt
 */
void init_lightInterrupt(){
    light_setRange(LIGHT_RANGE_4000);				// In accordance to data sheet
	light_setLoThreshold(interruptDarkLowerLimit);	// Will interrupt below this threshold
	light_setHiThreshold(interruptDarkUpperLimit);	// Will interrupt above this threshold
	light_setIrqInCycles(LIGHT_CYCLE_1);			// Number of cycles before the interrupt kicks in
	light_clearIrqStatus();							// Clear the Interrupt Status
	LPC_GPIOINT->IO2IntClr = 1 << 5;				// Clear the interrupt flag of a pin
	LPC_GPIOINT->IO2IntEnF |= 1 << 5;				// Set the interrupt to occur at falling edge

	NVIC_ClearPendingIRQ(EINT3_IRQn);				// Clear any pending interrupt occuring at that pin
	NVIC_EnableIRQ(EINT3_IRQn);						// Enable the interrupt
}

/**
 * Function of Tick Handler
 * Increase tick count by 1 every millisecond
 */
void SysTick_Handler(void) {
    msTicks++;
}

/**
 * Function to get the tick count
 */
uint32_t getTicks() {
    return msTicks;
}

/**
 * Function to display 7 Seg
 */
void change7Seg(){
	led7seg_setChar(displayValues[segCount], FALSE);
	if(segCount == 15)
		segCount = 0;
	else
		segCount++;
}

/**
 * Function run the blinking of RGB
 */
void runBlinkRGB(uint8_t colour){
	rgb_setLeds(colour);
}

/**
 * Function to flip the limits of the Light Sensor
 */
void flipLightLimits(){
	if(lightLowWarning == 1){
		// Set the interrupt to occur again when light level is high
		light_setLoThreshold(interruptLightLowerLimit);
		light_setHiThreshold(interruptLightUpperLimit);
	} else {
		// Set the interrupt to occur again when light level is low
		light_setLoThreshold(interruptDarkLowerLimit);
		light_setHiThreshold(interruptDarkUpperLimit);
	}
	// Clear the interrupt status so that it can occur again
	light_clearIrqStatus();
}

/**
 * Function to obtain the Prescalar value from the given timer Peripheral Clock Bit
 * Credits given to: https://exploreembedded.com/wiki/LPC1768:_Timers
 */
unsigned int getPrescalar(uint8_t timerPeripheralClockBit){
    unsigned int peripheralClock, prescalar;

    // get the peripheral clock info for required timer
    if(timerPeripheralClockBit < 10)
    	peripheralClock = (LPC_SC->PCLKSEL0 >> timerPeripheralClockBit) & 0x03;
    else
    	peripheralClock = (LPC_SC->PCLKSEL1 >> timerPeripheralClockBit) & 0x03;

    // Decode the bits to determine the peripheral clock, obtained from the LPC Timer
    switch ( peripheralClock ){
		case 0x00:
			peripheralClock = SystemCoreClock/4;
			break;

		case 0x01:
			peripheralClock = SystemCoreClock;
			break;

		case 0x02:
			peripheralClock = SystemCoreClock/2;
			break;

		case 0x03:
			peripheralClock = SystemCoreClock/8;
			break;
    }

    // Prescalar for 1us (1000000Counts/sec)
    prescalar = peripheralClock/1000000 - 1;

    return prescalar;
}

/**
 * Function to initialize the timer1 Interrupt
 * timer1 will run at every 1 second interval
 */
void init_timer1Interrupt(){
	LPC_SC->PCONP |= (1 << 2);			// Power Up Timer 1
	LPC_SC->PCLKSEL0 |= 0x01 << 4;		// Select the Timer 1 pin
    LPC_TIM1->MCR  = (1<<0) | (1<<1);	// Clear Timer Counter on Match Register 0 match and Generate Interrupt
    LPC_TIM1->PR   = getPrescalar(4);	// Prescalar for 1us
    LPC_TIM1->MR0  = 1000000;   		// Load timer value to generate 1s delay
    LPC_TIM1->TCR  = (1 << 0);			// Start timer by setting the Counter Enable
}

/**
 * Function to initialize the timer2 Interrupt
 * timer 2 will run at every 0.333333 second interval
 */
void init_timer2Interrupt(){
    LPC_SC->PCONP |= (1 << 22);			// Power Up Timer 2
    LPC_SC->PCLKSEL1 |= 0x01 << 12;		// Select the Timer 2 pin
    LPC_TIM2->MCR  = (1<<0) | (1<<1);	// Clear Timer Counter on Match Register 0 match and Generate Interrupt
    LPC_TIM2->PR   = getPrescalar(12);	// Prescalar for 1us
    LPC_TIM2->MR0  = 333333;   			// Load timer value to generate 1s delay
    LPC_TIM2->TCR  = (1 << 0);			// Start timer by setting the Counter Enable
}

/**
 * Function to check if switch 4 is pressed
 */
int isSwitch4Pressed(){
	// Read the switch
	sw4 = (GPIO_ReadValue(1) >> 31) & 0x01;
	// De-bounce the switch to determined if it is press
	if((sw4 == 0) && (getTicks()- sw4PressTicks >= 500)){
		sw4PressTicks = getTicks();
		return 1;
	} else {
		return 0;
	}
}

/**
 * Function to change the mode of the LPC
 */
void changeMode(){
	if(mode == MODE_STABLE){
		mode = MODE_MONITOR;
	}else{
		mode = MODE_STABLE;
	}
}

/**
 * Function to read the accelerometer
 */
void readAccelerometer(){
	acc_read(&x, &y, &z);
	x = x + xoff;
	y = y + yoff;
	z = z + zoff;
}

/**
 * Store previous value of the accelerometer
 */
void storePreviousAccelerometerValues(){
	prevX = x;
	prevY = y;
	prevZ = z;
}

/**
 * Set the current position of the accelerometer to Zero-G
 * Only done during start up to determine the orientation of the board
 * From there it will read the offset
 */
void setAccelerometerAtZeroG(){
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 0-z;
}

/**
 * Function to disable all the interrupts
 */
void disableAllInterrupts(){
	light_setLoThreshold(interruptDarkLowerLimit);
	light_setHiThreshold(interruptDarkUpperLimit);
    NVIC_DisableIRQ(TIMER1_IRQn);
    NVIC_DisableIRQ(TIMER2_IRQn);
    NVIC_DisableIRQ(EINT3_IRQn);
}

/**
 * Turn off all operations
 * Note: Interrupts are also disabled
 */
void turnOffAllOperations(){
	// Disable Interrupts
	disableAllInterrupts();

	// Reset the flags
	lightLowWarning = 0;
	isThereMovement = 0;
	onOrOff = 1;

	// Turns the OLED Screen off
	oled_clearScreen(OLED_COLOR_BLACK);

	// Turns the OLED Screen off
	led7seg_setChar(' ', FALSE);

	// Reset the count of the 7 Seg
	segCount = 0;

	// Ensure that RGB is off
	rgb_setLeds(0);

	// Set Warning to none
	warning = NONE;

	// Indicate that the screen is cleared
	isFirstTimeEnterStable = 0;

	// Change the flag to display monitor mode at UART
	isFirstTimeEnterMonitor = 1;
}

/**
 * Function to enable all devices
 */
void enableAllOperations(){
	// Display monitor on the OLED
	oled_putString(0,  0, (uint8_t*) "    MONITOR    ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	// Read and store accelerometer value
	readAccelerometer();
	storePreviousAccelerometerValues();

	// Enable Timer Interrupt
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
	light_clearIrqStatus();

	// Send message to UART
	UART_Send(LPC_UART3, (uint8_t *) messageEnterMonitor, strlen(messageEnterMonitor), BLOCKING);

	// Change Flag
	isFirstTimeEnterMonitor = 0;

	// Flag to set clearScreen when change mode
	isFirstTimeEnterStable = 1;
}

/**
 * Check the accelerometer to see if there is movement
 * A movement is considered to be of a value more than 10 in one direction
 * This is due the small vibrations detected by the accelerometer when stationary
 */
int checkForMovement(){
	if(abs(x-prevX) > 10 || abs(y-prevY) > 10 || abs(z-prevZ) > 10){
		storePreviousAccelerometerValues();
		return 1;
	}
	storePreviousAccelerometerValues();
	return 0;
}

/**
 * Depending on the Sensors determine the warning to issue to the system
 */
void determineWarningToIssue(){
	// Read accelerometer value and check for movement
	readAccelerometer();
	isThereMovement = checkForMovement();

	if(warning == NONE){
		// Check to see if there are any warnings
		if(temperature/10.0 > TEMP_HIGH_WARNING && isThereMovement && lightLowWarning == 1)
			warning = HIGH_TERMPATURE_AND_MOVEMENT_IN_LOW_LIGHT;
		else if (temperature/10.0 > TEMP_HIGH_WARNING)
			warning = HIGH_TEMPERATURE;
		else if (lightLowWarning == 1 && isThereMovement)
			warning = MOVEMENT_IN_LOW_LIGHT;
	} else if (warning == MOVEMENT_IN_LOW_LIGHT){
		// Check temperature
		if (temperature/10.0 > TEMP_HIGH_WARNING)
			warning = HIGH_TERMPATURE_AND_MOVEMENT_IN_LOW_LIGHT;
	} else if (warning == HIGH_TEMPERATURE){
		// Check for light
		if (lightLowWarning == 1 && isThereMovement)
			warning = HIGH_TERMPATURE_AND_MOVEMENT_IN_LOW_LIGHT;
	}
}

/**
 * Enable the interrupts if necessary, else disable them
 */
void enableInterruptsDependingOnWarning(){
	if(warning == NONE){
		rgb_setLeds(0);
		NVIC_DisableIRQ(TIMER2_IRQn);
	} else {
		NVIC_EnableIRQ(TIMER2_IRQn);
	}
}

/**
 * Create the STrings to Display on the OLED
 */
void createStringsToDisplayOnOLED(){
	sprintf(OLED_TEMPERATURE, "Temp: %-5.1f", temperature/10.0);
	sprintf(OLED_LIGHT, "Light: %-5lu", light);
	sprintf(OLED_X, "X: %-5d", x);
	sprintf(OLED_Y, "y: %-5d", y);
	sprintf(OLED_Z, "z: %-5d", z);
}

/**
 * Display the Strings in the Array to the OLED
 */
void displayStringsOnOLED(){
	oled_putString(0, 10, (uint8_t*) OLED_TEMPERATURE, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 20, (uint8_t*) OLED_LIGHT, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 30, (uint8_t*) OLED_X, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 40, (uint8_t*) OLED_Y, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 50, (uint8_t*) OLED_Z, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

/**
 * Function to initialize the temperature interrupt
 */
void init_temp_interrupt() {

	LPC_GPIOINT->IO0IntClr |= (1 << 2);
	LPC_GPIOINT->IO0IntEnF |= (1 << 2);

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
}

/**
 * Display the sensor values on the OLED
 */
void displayValuesOnOLED(){
	createStringsToDisplayOnOLED();
	displayStringsOnOLED();
}

/**
 * Check to see if it has reached 5, 10, 15 seconds
 * segCount is 1 value higher as it is incremented by run7seg
 */
int isFiveOrTenOrFifteenSeconds(){
	if(segCount == 6 || segCount == 11 || segCount == 0)
		return 1;

	return 0;
}

/**
 * Check to see if it has reached 15 seconds
 * segCount is 1 value higher as it is incremented by run7seg
 */
int isFifteenSeconds(){
	if(segCount == 0)
		return 1;

	return 0;
}

/**
 * Display the sensor values on the UART
 */
void displayResultsOnUART(){
	if(warning == HIGH_TERMPATURE_AND_MOVEMENT_IN_LOW_LIGHT || warning == HIGH_TEMPERATURE)
		UART_Send(LPC_UART3, (uint8_t *) displayHighTempWarning, strlen(displayHighTempWarning), BLOCKING);

	if(warning == HIGH_TERMPATURE_AND_MOVEMENT_IN_LOW_LIGHT || warning == MOVEMENT_IN_LOW_LIGHT)
		UART_Send(LPC_UART3, (uint8_t *) displayMovementInLowLight, strlen(displayMovementInLowLight), BLOCKING);

	sprintf(displayValuesToUART, "%03d_-_T%-5.1f_L%-5lu_AX%-5d_AY%-5d_AZ%-5d\r\n", message, temperature/10.0, light, x, y, z);
	UART_Send(LPC_UART3, (uint8_t *) displayValuesToUART, strlen(displayValuesToUART), BLOCKING);
	message++;
}

/**
 * Every time the GPIO interrupts are fired (regardless of which pin), this subroutine is called.
 * ISR Implementation
 */
void EINT3_IRQHandler (void){
	// Change the flag when interrupt occurs
	if ((LPC_GPIOINT->IO2IntStatF >> 5) & 0x1) {
		LPC_GPIOINT->IO2IntClr = 1 << 5;
		lightLowWarning = !lightLowWarning;
		flipLightLimits();
	}else if ((LPC_GPIOINT->IO0IntStatF >> 2) & 0x1){
		if (temp1 == 0 && temp2 == 0) {
			temp1 = getTicks();
		}
		else if (temp1 != 0 && temp2 == 0) {
			tempState++;
			if (tempState == 170) {
				temp2 = getTicks();
				if (temp2 > temp1) {
					temp2 = temp2 - temp1;
				}
				else {
					temp2 = (0xFFFFFFFF - temp1 + 1) + temp2;
				}
				temperature = ((2*1000*temp2) / (340*1) - 2731);
				temp2 = 0;
				temp1 = 0;
				tempState = 0;
			}
		}
		LPC_GPIOINT->IO0IntClr = (1 << 2);
	}
}

/**
 * Function to handle the Timer1 interrupt
 * Credits given to: https://exploreembedded.com/wiki/LPC1768:_Timers
 */
void TIMER1_IRQHandler(void){
    unsigned int isrMask;
    isrMask = LPC_TIM1->IR;
    LPC_TIM1->IR = isrMask;		// Clear the Interrupt Bit
	change7Seg();				// Change the 7 Segment
	oneSecondHasReached = 1;	// Change Flag
}

/**
 * Function to handle the Timer2 interrupt
 * Credits given to: https://exploreembedded.com/wiki/LPC1768:_Timers
 */
void TIMER2_IRQHandler(void){
    unsigned int isrMask;
    isrMask = LPC_TIM2->IR;
    LPC_TIM2->IR = isrMask;		// Clear the Interrupt Bit
    runWarning();				// Run the necessary warnings at the interrupt
}

/**
 * Main Function
 */
int main (void) {
	// Setup SysTick Timer to interrupt at 1msec intervals
	if (SysTick_Config(SystemCoreClock / 1000))
	    while (1);  // Capture error

	// Initialize the device
    init_i2c();
    init_ssp();
    init_GPIO();
    init_uart();
    init_timer1Interrupt();
    init_timer2Interrupt();
    init_lightInterrupt();
    pca9532_init();
    joystick_init();
    oled_init();
    rgb_init();
    acc_init();
    light_enable();
    led7seg_init();
    init_temp_interrupt();

    // Initialize Accelerometer to 0
    setAccelerometerAtZeroG();

    // Set start time for the De-bouncing of SW4
    sw4PressTicks = getTicks();

    while (1){
    	// Switch to determine the changing of mode
    	if(isSwitch4Pressed())
    		changeMode();

    	// Configure the operations to run depending on the mode
    	switch(mode){
    		case MODE_STABLE:
    			if(isFirstTimeEnterStable)
    				turnOffAllOperations();
    		break;

    		case MODE_MONITOR:
    			if(isFirstTimeEnterMonitor)
    				enableAllOperations();

    	        if(oneSecondHasReached){
    	        	determineWarningToIssue();
    	        	enableInterruptsDependingOnWarning();
    	        	if(isFiveOrTenOrFifteenSeconds()){
    	        		light = light_read();
    	        		readAccelerometer();
						displayValuesOnOLED();
    	        	}

    	        	if(isFifteenSeconds())
    	        		displayResultsOnUART();

    	        	oneSecondHasReached = 0; // Reset the one second flag
    	        }
    		break;
    	}
    }
}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
