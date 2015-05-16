///////////////////////////////////////////////////////////////
//
//  Organization:			UIC Chicago EDT
//  Engineer(s):			Krystian Gebis		Ammar Subei
//  E-Mail(s):				krgebis@gmail.com	ammarsubei@gmail.com
// 
//  Project Title:			Sigmux.c
//  Micro controller:		Atmega32U4
// 
///////////////////////////////////////////////////////////////

#include "Sigmux.h"

int main (void)
{
	Initialization();

	while(1)
	{
		#ifdef CC3000_ENABLED
			Recieve_WiFi_Data(); // We pull for data 
		#endif

		#ifdef ROUTER_WATCHDOG_ENABLED
			//Call the reset vector to restart the system from the beginning.
            if (Count >= 6)         {((void (*)(void))0)();}

		#endif

		#ifdef POWER_SAVING
			//COULD IT BE BETTER TO NOT JUST SLEEP, BUT POWER DOWN THE CPU?
			sleep_cpu(); /*Enter sleep mode.*/
		#endif
	}

	return 0;
}



/*
 *	@brief	Initialization is used to configure all of the registers of the microcontroller
 *			Steps:
 *				1) Initialize CC3000
 *				2) Set MUX Select to LOW, so we can send the Kill command from Atmega TX line 
 *				3) Set Mode to Safety Mode
 *				4) Set MUX Select to HIGH, so we get into Autonomous mode by default
 */
inline void Initialization (void)
{
	 #ifdef WATCHDOG_ENABLED
		wdt_enable(WDTO_8S);	// WDTO_8S means set the watchdog to 8 seconds.
	 #endif	

    //Set up the LEDs for WLAN_ON and DHCP:
    DDRC |= (1 << DDC6);    	// WLAN_INIT LED
    DDRC |= (1 << DDC7);    	// DHCP_Complete LED. This will very slowly blink

    DDRD |= (1 << DDD7);		// DDRD7 set outbound for CC3000 VBEN pin
    DDRB |= (1 << DDB7); 		// DDRB7 set outbound for Mux Select Line

    DDRE |= (1 << DDE2);		// DDRE2 set outbound for Safe Mode LED
    DDRD |= (1 << DDD6);		// DDRD6 set outbound for Manual Mode LED
    DDRD |= (1 << DDD4);		// DDRD4 set outbound for Auto Mode LED

    DDRF |= (1 << DDF0);		// Extra GPIO Pin
    DDRF |= (1 << DDF1);		// Extra GPIO Pin

	// #ifndef SKIP_BOOT
	// 	DDRB |= (1 << DDB4);
	// 	DDRD |= (1 << DDD7);
	// 	DDRD |= (1 << DDD6);

	// 	PORTB |= (1 << PB4);
	// 	_delay_ms(200);
	// 	PORTD |= (1 << PD7);
	// 	_delay_ms(200);
	// 	PORTD |= (1 << PD6);
	// 	_delay_ms(200);
	// 	PORTB &= ~(1 << PB4);
	// 	_delay_ms(200);
	// 	PORTD &= ~(1 << PD7);
	// 	_delay_ms(200);
	// 	PORTD &= ~(1 << PD6);
	// #endif

	// _delay_ms(500);
	// PORTF &= ~(1 << PF0);
	// PORTF &= ~(1 << PF1);

	// #ifdef ENERGY_ANALYSIS_ENABLED
	// 	//Enable Timer/Counter0 Interrupt on compare match of OCR0A:
	// 	TIMSK0 = (1 << OCIE0A); 		

	// 	//Set the Output Compare Register for the timer to compare against:
	// 	OCR0A = Energy_Analysis_Interval;

	// 	//Configure the ADC to have the reference pin be AREF on pin 21, and make sure everything is set to defaults:
	// 	ADMUX = 0x00;	
		
	// 	//Enable the Analog to Digital Conversion (ADC):
	// 	ADCSRA = (1 << ADEN);		//25 Clock cycles to initialize.	
	// #endif	

	#ifdef CC3000_ENABLED

		//Enable the CC3000, and setup the SPI configurations.
		init_spi();

		//Set up the CC3000 API for communication.
		wlan_init(CC3000_Unsynch_Call_Back, 
			  Send_WLFW_Patch, 
			  Send_Driver_Patch, 
			  Send_Boot_Loader_Patch, 
			  Read_WLAN_Interrupt_Pin, 
			  WLAN_Interrupt_Enable, 
			  WLAN_Interrupt_Disable, 
			  Write_WLAN_Pin);
 
		PORTC |= (1 << PC6);	//Set the WLAN_INIT LED on.
		sei();

		//Enable the CC3000, and wait for initialization process to finish.
		wlan_start(0);

		wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE|HCI_EVNT_WLAN_UNSOL_INIT|HCI_EVNT_WLAN_ASYNC_PING_REPORT);

		//Make sure we disconnect from any previous routers before we connect to a new one to prevent confusion on the device.
		wlan_disconnect();

		wlan_connect(WLAN_SEC_UNSEC, ROUTER_SSID, SSID_LENGTH, NULL, NULL, 0);

		while(!DHCP_Complete)
		{
			_delay_ms(1000);
		}
		
	    #ifdef WATCHDOG_ENABLED
			wdt_reset();
		#endif

		//Bind a socket to receive data:
		//sockaddr Mission_Control_Address;
		memset((char *) &Mission_Control_Address, 0, sizeof(Mission_Control_Address));
		Mission_Control_Address.sa_family = AF_INET;
		
		//The Source Port:
		Mission_Control_Address.sa_data[0] = (char)HEX_PORT_1;		//(char)0x09;
		Mission_Control_Address.sa_data[1] = (char)HEX_PORT_2;		//(char)0x56;

		//Configure the socket to not time out to keep the connection active.
		//--------------------------------------------------------------------
   		unsigned long aucDHCP       = 14400;
        unsigned long aucARP        = 3600;
        unsigned long aucKeepalive  = 10;
        unsigned long aucInactivity = 0;

		netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive, &aucInactivity);

		//TODO:
		//Should check the CC3000's profiles. In the case that there are no profiles found, then 
		//inform the PC system, or use an LED.

		//Open a UDP socket that grabs datagram:
		Socket_Handle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

		switch(Socket_Handle)
		{
			case -1:		//Error
				//Flag somehow.
			break;

			default:		//Success
				//Set the socket configuration for blocking (since it is the only thing that is allowed).
				switch( bind(Socket_Handle, &Mission_Control_Address, sizeof(sockaddr)))
	    		{
	        		case -1:
	            		//Flag as ERROR.
	        			break;

	        		default:
	            		//Flag as good.
	        			break;
	    		}

			break;
		}
	#endif

	// NEED TO SETUP A QUICK REMOVAL FLAG FOR THIS CODE TO TEST THE CC3000.
	// #ifdef MOTOR_CONTROL_FLAG
	// Set up our Motor Controller Selection lines and the output for the RS232 lines:
	// DDRD |= (1 << DDD3) | (1 << DDD4) | (1 << DDD5);
	DDRD |= (1 << DDD3) | (1 << DDD5);

	// Initialize the UART (RS-232 communications) for the motor controller interface:
	
	// Set the Baud rate to 115200 bits/s.  ((System Oscillator clock frequency / (2 * BAUD) ) - 1)
	// NOTE: The value may not be correct, according to the data sheet (pg. 213).
	// With the value 16, the error is 2.1% (lower than 8, being -3.5%).
	// This comes from util/setbaud.h

	UBRR1H = UBRRH_VALUE; /*Set baud rate*/
	UBRR1L = UBRRL_VALUE; /*Set baud rate*/

	//Defined in util/setbaud.h:
	#if USE_2X
		UCSR1A |= (1 << U2X1);	//Double the baud rate for asynchronous communication.
	#else
		UCSR1A &= ~(1 << U2X1);
	#endif	    

	// Set to no parity and in Asynchronous mode.
    // 1 Stop bit.
    // 1 Start bit.
    // Set to 8-bit data.
    UCSR1C |= (1 << UCSZ11) | (1 << UCSZ10); 

    //Enable the Rx and Tx lines.
    UCSR1B |= (1 << TXEN1);

#ifdef TWI_ENABLED
	//Set the SCL frequency to 200 KHz. From the equation: f(SCL) = F_CPU/(16 + (2*TWBR) * (4^TWPS))
	TWBR = 12;		
	DDRB |= (1 << DDB4);	//Setup PortB4 as the TWI error LED.
#endif	//End TWI_ENABLED

	_delay_ms(1000);			//Wait for one second for the RoboteQs to finish booting.	
	Set_Mode(SAFETY_MODE); 		// Set to Safe Mode to send Kill Command to Roboteq's
	Set_Mode(AUTONOMOUS_MODE);


	#ifdef ROUTER_WATCHDOG_ENABLED
		Count  = 0;					//Clear the Count variable out.
		TCNT1  = 0;					//Clear the TCNT register.
		TCCR1B = (1 << CS12) | (1 << CS10);		//Set the prescaler for 1024.
		TIMSK1 = (1 << OCIE1A);				//Enable output compare for 1A.
		OCR1A  = 39063;					//Set the system to interrupt every 5 seconds.
	
		//OCR1A = (Multiplier) * (F_CPU) / (Prescaler)		
		//39063 = (5) * (8000000) / (1024) 	

	#endif


}

//==========================================================================================================

void Mux_Select(uint8_t selection)
{
	switch (selection)
	{
		case ATMEGA_TX:
			PORTB &= ~(1 << PB7); // Set MUX Select A low to allow Atmega TX line to go through MUX
			break;
		case ARM_FTDI_SELECT:
			PORTB |= (1 << PB7); // Set MUX Select A high to allow FTDI to go through MUX
			break;
	}
}

/*Set_Mode*/
/**
  * @brief 
**/
uint8_t Set_Mode(uint8_t New_Mode)
{

	unsigned char Kill_Command[] = {'!', 'E', 'X', '\r'};//"!EX\r";
	unsigned char Go_Command[] = {'!', 'M', 'G', '\r'};//"!MG\r";

	if (New_Mode != Current_Mode)
	{
		switch (New_Mode)
		{
			case SAFETY_MODE:

				Mux_Select(ATMEGA_TX);	
						
				USART_Transmit (Kill_Command, 4);		// 4 is size of command
				// Stop the Linear Actuator 01/11
                // PORTD &= ~(1 << PD7);
                // PORTD &= ~(1 << PD6);

				PORTE |= (1 << PORTE2);					// Safe Mode LED on
				_delay_ms(500);
				Current_Mode = SAFETY_MODE;
				break;

			case AUTONOMOUS_MODE:            
				//Listen to all communications by the computer.
				//Do not forward any CC3000 motor controller commands.

				Mux_Select(ARM_FTDI_SELECT);
				USART_Transmit (Kill_Command, 4);
				USART_Transmit (Go_Command, 4);

				PORTD |= (1 << PORTD4);					// Autonomous Mode LED on
				_delay_ms(500);
				Current_Mode = AUTONOMOUS_MODE;
				
				break;

			case RC_MODE:

				Mux_Select(ATMEGA_TX);

				USART_Transmit (Kill_Command, 4);
	    		USART_Transmit(Go_Command, 4);
				
				PORTD |= (1 << PORTD6);		// Manual Mode LED on
				Current_Mode = RC_MODE;
				break;

			default:
				return -1;
				break;
		}

		return Current_Mode; // After setting the mode, this should now be the current mode we return
	}
	else
	{
		return Current_Mode; // The current mode is not different from the new requested mode
	}
	
}

//==========================================================================================================
//==========================================================================================================

// #ifdef USB_ENABLED
// 	inline void Enable_USB_Controller()
// 	{
// 		USBCON = (1 << USBE);		//Enable the USB Controller.
// 		UDCON &= ~(1 << LSM);		//Make sure the USB is in full speed mode.
// 		UDCON &= ~(1 << DETACH);	//Re-attach all devices.
// 	}//End Enable_USB_Controller

// 	inline void Disable_USB_Controller()
// 	{
// 		UDCON |= (1 << DETACH);		//Detach all devices on the USB line.
// 	}//End Diable_USB_Controller
// #endif

//==========================================================================================================
//==========================================================================================================

#ifdef CC3000_ENABLED

char* Send_Driver_Patch(unsigned long *usLength)
{
	*usLength = 0;
	return NULL;
}

char* Send_Boot_Loader_Patch(unsigned long *usLength)
{
	*usLength = 0;
	return NULL;
}

char* Send_WLFW_Patch(unsigned long *usLength)
{
	*usLength = 0;
	return NULL;
}

void CC3000_Unsynch_Call_Back(long Event_Type, char * Data, unsigned char Length)
{
	switch (Event_Type)
	{
		case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:	//First-time configuration process is complete.
			break;
		case HCI_EVNT_WLAN_KEEPALIVE:					//Periodic keep-alive event.
			break;
		case HCI_EVNT_WLAN_UNSOL_CONNECT:				//WLAN-connected event.
			break;
		case HCI_EVNT_WLAN_UNSOL_DISCONNECT:			//CC3000 disconnected from AP.
			break;
		////COMPILER CANNOT FIND THESE.
		case HCI_EVNT_WLAN_UNSOL_DHCP:					//DHCP state change.

			if ( * (Data + NETAPP_IPCONFIG_MAC_OFFSET) == 0)
			{
				//DHCP_Complete = 1;
				PORTC |= (1 << PC7);
			}
			else
			{
				//DHCP_Complete = 0;
				PORTC &= ~(1 << PC7);
			}		
			break;

		//case HCI_EVENT_CC300_CAN_SHUT_DOWN:
			//break;	
		case HCI_EVNT_WLAN_ASYNC_PING_REPORT:		//Notification of ping results.
			break;
		//case HCI_EVNT_BSD_TCP_CLOSE_WAIT:	
			//break;	
		case HCI_EVNT_WLAN_UNSOL_INIT:				//CC3000 finished the initialization process.
			break;	

		default:
			break;
	}
}

/*Read_Interrupt_Pin*/
/**
  *	@brief This function listens to the interrupt pin, and if it is high, return 1, or low return 0.
**/
long Read_WLAN_Interrupt_Pin()
{
	return bit_is_set(PIND, PIND2);
}

void Write_WLAN_Pin(unsigned char val)
{
	switch (val)
	{
		case 0:
			PORTD &= ~(1 << PORTD7);		// VBEN
			break;
		case 1:
			PORTD |= (1 << PORTD7);			// VBEN
			break;
	}
}

void WLAN_Interrupt_Enable()
{
	//Set the interrupt to occur when the pinout is falling:
	EICRA = (1 << ISC61);
	EIMSK |= (1 << INT6);

}

void WLAN_Interrupt_Disable()
{
	EIMSK &= ~(1 << INT6);
}

uint8_t Recieve_WiFi_Data()
{
	unsigned char Temp_Buffer[1];
    unsigned long Rx_Packet_Length = 0;
	//uint8_t Index = 0;
	//uint8_t End_Found = 0;
	
	switch(Socket_Handle)
	{
		case -1:
			return -1;
		break;

		case 0:
			// return -1;
		break;
	}
		
	Temp_Buffer[0] = 0;

    // Begin receiving data:
    recvfrom(Socket_Handle, Temp_Buffer, CC3000_APP_BUFFER_SIZE, 0, &Mission_Control_Address, &Rx_Packet_Length);	
	// Since it matches the protocol, we need to keep scanning to find the deliminating character.


	// Check if Temp_Buffer index 0 
	//Decipher the data transmitted to see if it matches the protocol (if it doesn't ERROR):

	if(Temp_Buffer[0]) 
		Decrypt_Data(Temp_Buffer[0]); 

	return 0;
}//End Recieve_WiFi_Data
#endif


void Select_Motor_Controller(uint8_t Motor)
{
	switch(Motor)
	{
		case 0:		//Wheel Controller
		case 1:		
			PORTD &= ~(1 << PORTD5);		// Does nothing...
		break;

		case 2:		//Mechanism & Linear Actuator Controller
			PORTD |= (1 << PORTD5);			// Does nothing...
		break;

		default:	//Invalid intput
			return;
		break;

	}


	_delay_ms(10);
}

uint8_t Decrypt_Data(unsigned char Data)
{
	
	// @00 means broadcast to ALL CAN ID's
	unsigned char Output_Command[MAX_COMMAND_SIZE] = {'@', '0', '0', '!', 'G', ' '};
	unsigned char dataIndexSize;

	#ifdef WATCHDOG_ENABLED
		wdt_reset();
	#endif 
/*
	switch(Data)
	{
		case 71:
			Output_Command[5] 	= '1';
			Output_Command[6] 	= '0';
			Output_Command[7] 	= '0';
			Output_Command[8] 	= '0';
			Output_Command[9]	= '\r';
			dataIndexSize 			= 10;
		break;

	}
*/
	dataIndexSize = (sizeof(Output_Command) / sizeof(char));
	
	USART_Transmit(Output_Command, dataIndexSize);

	return 0;
}


// #ifdef TWI_ENABLED
// /*Transmit_Energy_Data*/
// inline void Transmit_Energy_Data()
// {
// 	unsigned char Energy_Data [4];
	
// 	//Use TWI to recieve data from energy monitor:

// 	TWI_SEND_START();
// 	TWI_WAIT_FOR_START();
// 	TWI_CHECK_START();
	
// 	TWI_SEND_SLA_R();
// 	TWI_TRANSMIT();
// 	TWI_WAIT_FOR_START();
// 	TWI_CHECK_RECIEVE();

// //FOR THIS CASE LET US ASSUME THAT THERE WILL BE NO ERRORS DURRING COMMUNICATION.
// //--------------------------------------------------------------------------------
// 	TWI_RECIEVE();
// 	TWI_WAIT_FOR_START();
// 	Energy_Data[0] = TWDR;

// 	TWI_CHECK_RECIEVE();
// 	TWI_WAIT_FOR_START();
// 	Energy_Data[1] = TWDR;

// 	TWI_RECIEVE();
//         TWI_WAIT_FOR_START();
// 	Energy_Data[2] = TWDR;
	
// 	TWI_RECIEVE();
//         TWI_WAIT_FOR_START();
// 	Energy_Data[3] = TWDR;
// //--------------------------------------------------------------------------------

// 	TWI_SEND_STOP();

// 	//Transmit Data from Energy Monitor:
// 	sendto(Socket_Handle, Energy_Data, 4, 0, &Mission_Control_Address, (socklen_t)sizeof(Mission_Control_Address));
	
			
// }//End Transmit_Energy_Data
// #endif //End TWI_ENABLED


//==========================================================================================================
//==========================================================================================================


//***********************************************************************************************************
//Interrupts:
//***********************************************************************************************************

////NEEDS TO BE REVISED:
//Energy Analysis:
// #ifdef ENERGY_ANALYSIS_ENABLED
// 	ISR (TIMER0_COMPA_vect)
// 	{
	
// 		uint16_t Temp_Power = 0;
// 		//Set the ADMUX to collect data from ADC0 and compare it against AREF to find the battery voltage: 
// 		ADMUX = 0x00;

// 		ADCSRA |= (1 << ADSC); //Trigger ADC conversion.

// 		//Wait until the conversion is complete (ADSC = 0) then complete:
// 		while (ADCSRA & (1 << ADSC));

// 		//Collect the data:
// 		Temp_Power = 0;
	
// 	}//End Timer/Counter 0 Compare Match
// #endif //End ENERGY_ANALYSIS_ENABLED
//------------------------------------------------------------------------------------------------------

//CC3000 Data Output Request:
ISR (INT6_vect)
{
	SPI_IRQ();
	return;
}//End External Interrupt (INT0) [CC3000 IRQ]

//------------------------------------------------------------------------------------------------------

//Pseudo_Watchdog_System:
#ifdef ROUTER_WATCHDOG_ENABLED
	ISR (TIMER1_COMPA_vect)
	{
		///DEBUG:
		PORTC ^= (1 << PORTC7);	
		///
		++Count;		//Multiply count by five to get the number of seconds that have passed.
	}					//End Timer1_Compare_A Match 
#endif //End ROUTER_WATCHDOG_ENABLED