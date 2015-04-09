/*
 * Sigmux.h
 *
 * Created: 3/10/15 1:08:31 AM
 *  Author: Krystian
 */ 


#ifndef SIGMUX_H_
#define SIGMUX_H_

/*
 *Fuse Configuration:
 *
 *-U lfuse:w:0x82:m 
 *-U hfuse:w:0x99:m 
 *-U efuse:w:0xf3:m 
 *
 *All Fuse calculations from http://www.engbedded.com/fusecalc/
 *
 *Purpose/Description:
 *	The purpose of the Sigmux is to be the central decision system of the robot. Its task is to constantly
 * check on the Rx and Tx pins for communications over the wi-fi, and appropriately changing modes, or forwarding * communications. At the same time the Sigmux is to listen for physical button presses on pins (TO BE ADDED) and * changing which mode it is in. Added functionality at a later time will be to analyze power consumption voltage * analysis and DC current analysis through a DC current transducer on pins(TO BE ADDED).
 * Focusing more on the actual multiplexing component, the Sigmux is designed to forward any and all motor
 * controller commands that are sent through the wi-fi interface, if they are from a controller, or from the
 * on-board computer system if the unit is set into 'Autonomous Mode'. 
 *
 *
 *Procedure:
 *	1)
 *
 *	2)
 *
 *	3) 
 *
 *
 *
 *Revisions:
 * 	0.1 (09/05/2013) - File created.
 *	0.2 (09/11/2013) - Micro-controller revised, not using Atmega8_16PU. Now using Atmega16U4.	
 *	0.3 (09/12/2013) - Created all compiler flags, function prototypes, and main.
 *	0.4 (09/15/2013) - Created an initialization inline function for reading simplicity.
 *	0.5 (09/18/2013) - Halting of all energy analysis code (Design change).
 *	0.6 (09/19/2013) - Begin CC3000 code.
 *	0.7 (10/01/2013) - Begin the initialization of the CC3000 through wlan_init(). 
 *	0.8 (10/07/2013) - Continue converting ccspi adafruit library to an AVR library in SPI.
 *	0.9 (10/08/2013) - Finished converting ccspi adafruit library to an AVR library in SPI.
 *	1.0 (10/09/2013) - Set up socket for UDP communication over the CC3000, and finished draft of wlan 
 *                         intitialization.
 *	1.1 (10/11/2013) - Begining setup of serial communication for motor controllers.
 *	1.2 (10/16/2013) - Added function to control muxes and demuxes for motor select.
 *	1.3 (10/30/2013) - Modified the Initialization for spi_init, since it is no longer a function, 
 *                         but an inline.
 *	1.4 (11/05/2013) - Disabled all Energy Analysis systems. 
 *	1.5 (11/06/2013) - Set up the Timer 1 Compare B interrupt and configured its initialization. 
 *	1.6 (11/22/2013) - CC3000 communications finished, the device now initializes without any issues, 
 *                         though more testing needs to be done to make sure the device is connecting 
 *                         appropriately.
 *	1.7 (11/26/2013) - Discovered that the MAC address of the CC3000 unit is:  08:00:28:57:55:16.
 *	1.8 (12/20/2013) - Reconfigured the wlan_connect() parameters (in initialization) based of TI examples.
 *	1.9 (01/03/2014) - Removed all the static functions since they only waste space and unused static 
 *                         variables. Also added the function Recieve_Wifi_Data() to collect the UDP packets 
 *                         and decipher them. Was added to main. 
 *	2.0 (01/03/2014) - Added a new volatile global variable to keep track of if DHCP is complete or not.
 *			   now we do not need to wait five seconds to see if DHCP completed.
 *	2.1 (01/07/2014) - Finished USART_Transmit macro (in macro section). Now transmits a char array.
 *	2.2 (01/08/2014) - Finished USART_Recieve macro (in macro section). 
 *	2.3 (01/09/2014) - Modified USART_Recieve macro to take a pointer to hold the number of characters
 *			   that were recieved. Also corrected the registers that were used to non-arbitrary 
 *		 	   registers (they were 'n's instead of '1's).
 *	2.4 (01/14/2014) - Modified UDP protocol to be one byte of data.
 *	2.5 (01/15/2014) - Added some preprocessor flag if statements to wi-fi functions. Included a USART baud 
 * 			   configuration library to minimize the design. And removed USART debug code in the
 *			   initialization code. 	
 *	2.6 (01/16/2014) - Determined the proper operating frequency for both the CC3000 and USART to be 
 *			   8MHz from the internal clock.
 *	2.7 (01/17/2014) - Set up USART debug test by broadcasting "GO!" on boot. Still does not work unless
 *			   data is sent in. So the RX line has been disabled to try to fix it. 
 *	2.8 (01/17/2014) - Added the fuse configuration to this file right after 'Last Date Modified'.
 *	2.9 (01/17/2014) - Added SERIAL_CONVERT preprocessor flag. Modified USART_Transmit to convert the input 
 *			   to be usable with transmission.
 *	3.0 (01/21/2014) - Commented out the serial initialization code and made a few corrections to make sure
 *			   the serial transmits only up to the '\r' character.
 *	3.1 (01/22/2014) - Fixed the issue with the socket timing out. The wireless device now will not need 
 *			   to communicate to sustain connection.
 *	3.2 (02/05/2014) - Added a Power LED on port E2 and a status LED on port C7.
 *	3.3 (02/06/2014) - Removed the Power LED. It is now based on the power rail.
 *	3.4 (03/12/2014) - Included the kill to linear actuators in safety mode.
 *	3.5 (03/27/2014) - Redesigned the decrypting protocols to accomodate the MDC22XX series.
 *	3.6 (04/02/2014) - Disabled Rx line to be repurposed as the external interrupt for the CC3000. The old 
 * 			   interrupt pin is to be repurposed for Two Wire Interface (I2C).
 *	3.7 (05/06/2014) - Removed all unused global variables to reduce space consumption. 
 *			   Modified Decrypt_Data to utilize one switch statement and to fit the 
 *			   two dual-channeled RoboteQ design.
 *	3.8 (05/07/2014) - Optimized the Decrypt_Data to run faster and consume less space. 
 *			   Also removed some unused pins.		
 *	3.9 (05/15/2014) - Redesigned Decrypt_Data to accomodate the operation of new dual channeled RoboteQ. 
 *	4.0 (05/17/2014) - Added the Manual E-GO command to the case statement in Decrypt_Data.
 *	4.1 (05/21/2014) - Added a pseudo watchdog system that uses timer 1 to reset the microcontroller if 
 *			   there is no activity for 30 seconds. Also optimized the modes using ECE 465 state
 *			   assignment methods (look in preprocessor definitions). Removed current_motor global
 *			   variable due to its lack of utility. Changed DHCP_Complete to a boolean definition
 *			   rather than making it a global variable. Added an Unassigned_Mode definition to 
 *			   be used on initialization. Updated Pin Assignments. 
 *	4.2 (05/22/2014) - Moved the initial SAFETY_MODE configuration to after UART initialization. Also 
 *			   a one second delay before it enters.	
 *	4.3 (06/27/2014) - Added a preprocessor flag for the TWI system called TWI_ENABLED. Added TWI related
 *			   macros. Added PinB4 as TWI Error LED.
 *	
 *TODO/NOTES:
 *	- See if there is a faster way to do delay commands rather than through 'util/delay.h' (if needed).  
 *	- Should the Sigmux do the power consumption check? Regardless, if we are going to do it would be best
 * 	  done with a microcontroller with a LARGE resolution with a small prescaler, an off-unit voltage source
 *	  (other than the battery) for reference voltage, and a DC current transducer to calculate power.
 *	- We will be using an external oscillator at 16 MHz.
 *
 *
 *
 *------------------------------------------------------------------------------------------------------*/

/*======================================================================================================
 *Pin Assignments:
 *
 * PB0 (SS)   :				[OUTPUT]  	{CC3000 CS} 
 * PB1 (SCLK) :				[OUTPUT]  	{CC3000 SCK}
 * PB2 (MOSI) :				[OUTPUT]  	{CC3000 MOSI}
 * PB3 (MISO) :				[INPUT]   	{CC3000 MISO}
 * PB5 (PCINT5/OC1A/!OC4B/ADC12) :	[OUTPUT]  	{CC3000 VBEN}
 * PB4 () :				[OUTPUT]	{TWI Error LED}
 * PB6 (...) :				[OUTPUT]	{CC3000 WLAN_Enabled LED}
 * PB7 () :				[OUTPUT]	{Demux Select Line 1}
 *
 * PC6 (OC3A/!OC4B) :  			[OUTPUT]  	{CC3000 DHCP Complete LED}
 * PC7 (ICP3/CLK0/OC4A) :		[OUTPUT]	{Status 0 LED}
 * 
 * PD0 (OC0B/SCL/INT0) :		[UNUSED]	{TWI SCL Line}
 * PD1 (SDA/INT1) :			[UNUSED]	{TWI SDA Line}
 * PD2 (RXD1/INT2) :			[INPUT]  	{CC3000 IRQ}
 * PD3 (TXD1) : 			[OUTPUT] 	{RoboteQ RS232 Tx}
 * PD4 (ICP1/ADC8) :		[OUTPUT]	{Multiplexer Select A}
 * PD5 (XCK1/!) : 			[OUTPUT] 	{Demux Select Line 0}
 * PD6 (T1/OC4D/ADC9) :			[UNUSED]	{Status 1 LED}
 * PD7 (T0/OC4D/ADC10) :		[UNUSED]	{Status 2 LED}
 *
 * PF0 (ADC0) :				[OUTPUT]	{Mux Select Line}
 * PF1 (ADC1) :				[UNUSED]  	{Battery Voltage}
 *
 * Pin 2 (UVcc) :			[USB ]
 * Pin 3 (D-  ) :			[USB Data]
 * Pin 4 (D+  ) :			[USB Data]
 * Pin 5 (UGnd) :			[USB ]
 * Pin 6 (UCap) :			[USB ]
 * Pin 7 (VBus) :			[USB ]
 *
 * Pin 16 (XTAL_2) :			[External Clock] {NOT USED}
 * Pin 17 (XTAL_1) :			[External Clock] {NOT USED}
 *
 * Pin 42 (ARef  ) :			[Reference Voltage]
 *
 *======================================================================================================*/

//----------------------------------------------------------------------------------------------------------
//Preprocesor Definitions:
//----------------------------------------------------------------------------------------------------------

/*
 * @brief	Onboard Operating Frequency of uC in Hertz. This is required to be defined before importing the delay header to prevent warnings on compilation.
 *			View pages 29, 209, and 348 to determine the default configurations. The device is prescaled by 8 by default. 
 */
#ifndef F_CPU
	//#define F_CPU		32000000UL
    //#define F_CPU		16000000UL
	  #define F_CPU		8000000UL
	//#define F_CPU		4000000
	//#define F_CPU		2000000
#endif

/*
 * 	@brief	DEBUG_ENABLED is a preprocessor flag that is used to identify what fragments of code are used for debugging and should be included in the code for compilation, otherwise it will entirely ignore (or remove) the code fragments (i.e. not compile them). 
 *			By default this flag is enabled. To disable all debugging code fragments, replace #define with #undef. 
 */
#define DEBUG_ENABLED


/*
 *	@brief	WATCHDOG_ENABLED is a preprocessor flag that is used to enable/disable the watchdog timer and any calls to feed it. 
 *			By default this flag is enabled. To disable all watchdog related code, replace #define with #undef. 
 */
#define WATCHDOG_ENABLED


/*
 *	@brief	SKIP_BOOT is a preprocessor flag that is used to skip the boot sequence to identify that the unit is 
 *			operating appropriately when turned on. Though it may consume time that can be better spent running the core code.
 *			By default this flag is disabled. To enable the boot sequence, replace #define with #undef.
 */
#undef SKIP_BOOT


/*
 *	@brief	ENERGY_ANALYSIS_ENABLED is a preprocessor flag that is ued to enable/disable power analysis of the
 *			battery system periodically.
 *			By default this flag is enabled. To disable power analysis, replace #define with #undef.
 */
#undef ENERGY_ANALYSIS_ENABLED

/*
 *	@brief POWER_SAVING is a preprocessor flag that is used to enable/disable MCU sleep commands for power consumption saving.
 *	By default this flag is disabled. To enable sleep commands, replace #undef with #define
 */
#undef POWER_SAVING

/*
 *	@brief CC3000_ENABLED is a preprocessor flag that is used to enable/disable the Adafruit CC3000 WiFi breakout board for debugging purposes.
 *	By default this flag is enabled. To disable the wifi, replace #define with #undef
 */
#define CC3000_ENABLED

/**
	@brief USB_ENABLED is a preprocessor flag that is used to enable/diable all USB communications (i.e. diable the autonomous mode).
	By default this flag is enabled. To disable the USB controller, replace #define with #undef
*/
#undef USB_ENABLED

/**

	@brief PWM_ENABLED is a preprocessor flag that is used to enable/disable PWM communication to the motor controllers. This will override the serial communication for controling the motor controller, though the serial lines are still used to query the motor controller.
	By default this flag is disabled. To use the PWM, replace #undef with #define
*/
#undef PWM_ENABLED

/**

	@brief SERIAL_CONVERT is a preprocessor flag that is used to convert the inputs for RS-232 into a usable output. 
	By default this flag is enabled. To take ascii value in as they are with no modification, replace #define with #undef

*/
#define SERIAL_CONVERT

/**
	@brief ROUTER_WATCHDOG_ENABLED is a preprocessor flag that enables a timer interrupt system that counts
up the time without a command to determine if the connection to the router has been lost if after thirty seconds
no command has come in, then assume that the router has rebooted and reboot the microcontroller.

	By default this flag is enabled. To disable this pseudo-watchdog system, replace #define with #undef
*/

#undef ROUTER_WATCHDOG_ENABLED

/**
	@brief TWI_ENABLED is a preprocessor flag that enables the Two-Wire-Interface (Atmel's equivalent 
to I2C). This is used to interface the Sigmux to the energy monitor system and any other peripherals over TWI.
	
	By default this flag is enabled. To disable the TWI system, replace #define with #undef
*/

#define TWI_ENABLED


/**
	@brief	TEST_TWI is a preprocessor flag that is dependent upon TWI_ENABLED preprocessor flag. If TEST_TWI
			is enabled then test code will be compiled otherwise it will be ignored.
			By default this flag is disabled. To enable the TWI test code, replace #undef with #define
*/
#undef TEST_TWI



//------------------------------------------------------------------------------------------------------------
//Preprocessor Definitions:
//------------------------------------------------------------------------------------------------------------
#define Energy_Analysis_Interval 		(100)		//100ms
#define Energy_Analysis_Reference 		(12)		//12V
#define Energy_Analysis_Resistance 		(10)   		//10 Ohms

//UART Definitions:
#define BAUD 					115200		//115200 bits/second


//CC3000 Definitions:
#define NETAPP_IPCONFIG_MAC_OFFSET		(20)
#define PORT							(2390)		
#define HEX_PORT_1						(0x09)
#define HEX_PORT_2						(0x56)
#define ROUTER_SSID						"Scipio_AP"	
#define SSID_LENGTH						(9)	
#define CC3000_APP_BUFFER_SIZE			(1)		
#define CC3000_RX_BUFFER_OVERHEAD_SIZE	(20)
#define DHCP_Complete					(PORTC & (1 << PC6))	

#define MAX_COMMAND_SIZE				23  // For example:   @00!G 1 600_@00!G 2 500

//TWI Definitions:
#define Energy_Monitor_Address			(0x00)

//Sigmux Mode Definitions:
#define U 								(4) //00000100			
#define SAFETY_MODE               		(0) //00000000		//(1)		//00000001
#define AUTONOMOUS_MODE           		(3) //00000011		//(2)		//00000010
#define RC_MODE               	  		(1) //00000001		//(4)		//00000100
#define Upload_Mode 		  			(2) //00000010		//(8)		//00001000

//Multiplexer Selection Definitions:
#define ARM_FTDI_SELECT	(1)
#define ATMEGA_TX		(0)
//------------------------------------------------------------------------------------------------------------


#include <avr/io.h>                     //Needed for standard Micorcontroller I/O operations.
#include <stdbool.h>                    //Needed to use Boolean data types.
#include <avr/interrupt.h>              //Needed to use all types of interrupts.
#include <util/delay.h>                 //Needed to call the delay command.
#include <util/setbaud.h>				//Needed to test if we can set the baud rate correctly. 


#ifdef CC3000_ENABLED
	#include "CC3000/spi.h"
	#include "CC3000/hci.h"		
	#include "CC3000/cc3000_common.h"
	#include "CC3000/wlan.h"
	#include "CC3000/socket.h"
	#include "CC3000/evnt_handler.h"
	#include "CC3000/netapp.h"
	#include "CC3000/host_driver_version.h"
	#include "CC3000/nvmem.h"	
#endif	

#ifdef WATCHDOG_ENABLED
        #include <avr/wdt.h>
#endif	

#ifdef POWER_SAVING
        #include <avr/sleep.h>
#endif	

#ifdef TWI_ENABLED
	#include <util/twi.h>
#endif 	


//************************************************************************************************************
//Preprocessor Macros:
//************************************************************************************************************

/**
 *	@brief The SET macro is used as a means to simplify the code so that it is easier to read without
 *	having to make process consuming functions.
 *	
 *	The functionality of this is to set a register.
 *	
 *	Original design taken from signal_mux.c.
 *
 *	@author Anup Kotadia anup.kotadia@gmail.com
*/
#define SET(mask, reg) (reg |= (mask))

/**
 *	@brief The CLEAR macro is used as a means to simplify the code so that it is easier to read.
 *      @author Anup Kotadia anup.kotadia@gmail.com
*/
#define CLEAR(mask, reg) (reg &= ~(mask))

/**
 *	@brief	This function takes in data and transmit it over serial.
 *			Transmit one char at a time, and roboteq wait until it receives a '\r' which signifies the end of the roboteq command
 *
 *	@param	_Data	: Array of roboteq data command (i.e !EX\r would be 4 indexed because \r is end line character)
 *			_Index	: Size of data array
 *
 *	@author John Sabino johntsabino@gmail.com
*/
#ifdef SERIAL_CONVERT
	#define USART_Transmit(_Data, _Index) {uint8_t j; for (j = 0; j < _Index; ++j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = ((_Data[j] << 1) ^ 0xff);}}
#else
	#define USART_Transmit(_Data, _Index) {uint8_t j; for (j = 0; j < _Index; ++j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = _Data[j];}}
#endif


/**
 *	@brief This function scans for serial data input. Currently not working according to John.
 *	@author John Sabino johntsabino@gmail.com
*/
#define USART_Recieve(_Output, _Num_Char) {uint8_t _k = 0; memset((unsigned char *)&_Output, 0, sizeof(_Output)); while(_Output[_k] != '\r') {while (!(UCSR1A & (1<<RXC1))); _Output[_k++] = UDR1;} *_Num_Char = k;}


/**
 *	@brief Feed_Watchdog() call the Watchdog Reset command from assemble to reset the watchdog timer.
 *	@author John Sabino johntsabino@gmail.com
*/
#define Feed_Watchdog()		{asm("WDR");}


/*
 *	@brief	The TWI related macros are to better understand what the system is doing. See the datasheet page
 *			237 for more details.
 *	@author John Sabino johntsabino@gmail.com
 */
#define TWI_ERROR()			{PORTB |= (1 << PB4);}
#define TWI_SEND_START()	{TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);}
#define TWI_SEND_STOP()		{TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);}
#define TWI_TRANSMIT()      {TWCR = (1 << TWINT) | (1 << TWEN);}
#define TWI_RECIEVE()		{TWCR = (1 << TWINT) | (1 << TWEA)  | (1 << TWEN);}

#define TWI_WAIT_FOR_START()	{while (!(TWCR & (1<<TWINT)));}
#define TWI_CHECK_START()		{if ((TWSR & 0xF8) != 0x08) {TWI_ERROR();}}
#define TWI_CHECK_RECIEVE()		{if ((TWSR & 0xF8) != 0x40) {TWI_ERROR();}}
#define TWI_SEND_SLA_R()		{TWDR = Energy_Monitor_Address | TW_READ;}


inline void Initialization(void);
uint8_t	Set_Mode(uint8_t Mode);
void Select_Motor_Controller(uint8_t Motor);

void Mux_Select(uint8_t selection); 

#ifdef USB_ENABLED
	inline void Enable_USB_Controller(void);
	inline void Disable_USB_Controller(void);
#endif

#ifdef CC3000_ENABLED
	//Required for CC3000 Initialization:	
	char*	Send_Driver_Patch(unsigned long *usLength);
	char*	Send_Boot_Loader_Patch(unsigned long *usLength);
	char*	Send_WLFW_Patch(unsigned long *usLength);
	
	void	CC3000_Unsynch_Call_Back(long event_type, char * data, unsigned char length);
	
	long	Read_WLAN_Interrupt_Pin(void);
	void	Write_WLAN_Pin(unsigned char val);
	
	void	WLAN_Interrupt_Enable(void);
	void	WLAN_Interrupt_Disable(void);
	
	uint8_t Recieve_WiFi_Data(void);
	uint8_t Decrypt_Data(unsigned char Data);

#endif

#ifdef TWI_ENABLED
	inline void Transmit_Energy_Data(void);
#endif  

//############################################################################################################
//Global Variables:
	
#ifdef ENERGY_ANALYSIS_ENABLED
	static volatile uint32_t Samples_Taken = 0;	 
	static volatile uint32_t Total_Power   = 0;
#endif

static long Socket_Handle;
static sockaddr Mission_Control_Address;
static volatile uint8_t Current_Mode;

#ifdef ROUTER_WATCHDOG_ENABLED
	static volatile uint8_t Count;		// = 0;
#endif	//End ROUTER_WATCHDOG_ENABLED

//############################################################################################################

#endif 