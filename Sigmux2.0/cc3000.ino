#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"

#define	LOG_DATA
// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   7      // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  6
#define ADAFRUIT_CC3000_CS    17     //  http://forum.arduino.cc/index.php?topic=241369.0

// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11

Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS,
					 ADAFRUIT_CC3000_IRQ,
				         ADAFRUIT_CC3000_VBAT,
					 SPI_CLOCK_DIVIDER);

// cannot be longer than 32 characters!
#define WLAN_SSID       "chicagoedt"        
#define WLAN_PASS       "notrightnow"
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define EDT_UDP_SERVICE	5002

// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2

Adafruit_CC3000_Client udpClient;

#define SERIAL_CONVERT
#define MUX_TELEOP         0
#define MUX_AUTONOMOUS     1
#define PB7            11    // MUX select pin
#define PD4            4     // Autonomous mode LED
#define PD6            12    // Manual mode LED
#define PC6            5     // WLAN LED

#define SAFE_MODE       0
#define AUTONOMOUS_MODE       1
#define MANUAL_MODE     2

#ifdef SERIAL_CONVERT
  #define USART_Transmit(_Data, _Index) {uint8_t j; for (j = 0; j < _Index; ++j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = ((_Data[j] << 1) ^ 0xff);}}
#else
  #define USART_Transmit(_Data, _Index) {uint8_t j; for (j = 0; j < _Index; ++j) { while (!( UCSR1A & (1<<UDRE1))); UDR1 = _Data[j];}}
#endif

static volatile uint8_t currentMode;


void setup()
{
	Serial.begin(115200);
    // Check that cc3000.begin() returns true
	while (!cc3000.begin())
	{
		Serial.println(F("Unable to initialize the CC3000! Check your wiring?"));
		delay(500);
	}
	
    // Disable TCP timeout
	disableIdleTimout();
	
    // Check Firmware version
	uint16_t firmware = checkFirmwareVersion();
	if (firmware < 0x113)
	{
		Serial.println(F("Wrong firmware version!"));
		while(1);
	}
  
    // Print out some debugging info
    displayDriverMode();
	displayMACAddress();
  
    // Attempt connection to AP

	// NOTE: Secure connections are not available in 'Tiny' mode!
	// By default connectToAP will retry indefinitely, however you can pass an
	//  optional maximum number of retries (greater than zero) as the fourth parameter.
     
	// ALSO NOTE: By default connectToAP will retry forever until it can connect to
	// the access point.  This means if the access point doesn't exist the call
	// will _never_ return!  You can however put in an optional maximum retry count
	// by passing a 4th parameter to the connectToAP function below.  This should
	// be a number of retries to make before giving up, for example 5 would retry
	// 5 times and then fail if a connection couldn't be made.

	Serial.print(F("Connecting to AP - "));
        
        //wlan_disconnect();
	
    if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY))
	{
		Reboot("Connecting to AP - Failed", 1);
		return;
	}
   
	Serial.println(F("OK"));
  
    // Check for DHCP and timeout after 20 seconds
    unsigned long dhcpTimeout = 20000;	// Try for 20 sec
	unsigned long retry       = 0;
   
	for(unsigned long t = millis(); ((millis() - t) <= dhcpTimeout);)
	{
		Serial.print(F("Querying DHCP - "));
		
		if(cc3000.checkDHCP())
		{
			Serial.println(F("OK"));
			break;
		}
		else
		{
			if(retry >= 10)
				Reboot("FAILED - Rebooting!!!", 2);
			
			Serial.println(retry+1);
		}
		
		++retry;
		delay(1000);
	}
	
    // Attempt connection to UDP
	uint32_t bindIP = displayConnectionDetails();
	uint32_t ip = cc3000.IP2U32(192, 168, 1, 192);	// EDT Panel IP
	udpClient   = cc3000.connectUDP(ip, EDT_UDP_SERVICE, 0);
	
	if( udpClient.connected() )
          Serial.println(F("UDP Kurwa Connected"));
    else
		Serial.println(F("UDP Kurwa is dead"));
}

void loop()
{
		if(!udpClient.connected())
			Reboot("Lost connection in main loop", 10);
		
		if( udpClient.available() )
		{
            // TODO need to read commands from CC3000 and 
            // decide what exactly to do (switch mode or 
            // send more data to motor controllers)
			// char val = 0;
			// udpClient.read(&val, 1);
			// Serial.println(char(val+48));
   //                      if(val == 0)
   //                      {
   //                        digitalWrite(4, HIGH);
   //                        digitalWrite(12, LOW);
   //                      }
   //                      if(val == 1)
   //                      {
   //                        digitalWrite(4, LOW);
   //                        digitalWrite(12, HIGH);
   //                      }

            // parse command
            uint16_t command    = 0; udpClient.read(&command, 2);

            uint8_t actuator    = 0;
            uint8_t dig         = 0;
            uint8_t left        = 0;
            uint8_t right       = 0;
             
            parseCommand(command, &actuator, &dig, &currentMode, &left, &right)
            // for debugging
            Serial.print("command bytes received: "); Serial.println(command);
            Serial.print("actuator: "); Serial.println(actuator);
            Serial.print("dig: "); Serial.println(dig);
            Serial.print("left: "); Serial.println(left);
            Serial.print("right: "); Serial.println(right);
            // switch modes, check if we fail
            if(!ModeSet(currentMode))
            {
                // TODO PRINT OUT ERROR
            }

            // TODO if manual mode,
            // construct RoboteQ commands and transmit over USART
            if(currentMode == MANUAL_MODE)
            {

            }
		}
		
		delay(100);
}

uint16_t checkFirmwareVersion(void)
{
	uint8_t	 major, minor;
	uint16_t version;
  
#ifndef CC3000_TINY_DRIVER  
	if(!cc3000.getFirmwareVersion(&major, &minor))
		Reboot("Unable to retrieve the firmware version", 3);
	else
	{
		Serial.print(F("Firmware : "));
		Serial.print(major);
		Serial.print(F("."));
		Serial.println(minor);
		
		version = major; version <<= 8; version |= minor;
	}
#endif
	return version;
}

uint32_t displayConnectionDetails(void)
{
	uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
	if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
		Reboot("Failed to query IP", 4);
	else
	{
		Serial.print(F(  " IP  : ")); cc3000.printIPdotsRev(ipAddress);
		Serial.print(F("\n Mask: ")); cc3000.printIPdotsRev(netmask);
		Serial.print(F("\n GW  : ")); cc3000.printIPdotsRev(gateway);
		Serial.print(F("\n DHCP: ")); cc3000.printIPdotsRev(dhcpserv);
		Serial.print(F("\n DNS : ")); cc3000.printIPdotsRev(dnsserv);
		Serial.println();
		return ipAddress;
	}
}

void Reboot(const char* errMsg, uint32_t errCode)
{
	Serial.println(errMsg);
	Serial.println(F("--REBOOTING--"));
	while(true);
	//cc3000.reboot();
        for(int x = 0; x < errCode; x++);
        {
          digitalWrite(PC6, HIGH);
          delay(300);
          digitalWrite(PC6, LOW);
          delay(300);
        }
}

void displayDriverMode(void)
{
#ifdef CC3000_TINY_DRIVER
	Serial.println(F("CC3000 is configured in 'Tiny' mode"));
#else
	Serial.print(F("RX Buffer : "));
	Serial.print(CC3000_RX_BUFFER_SIZE);
	Serial.println(F(" bytes"));
	Serial.print(F("TX Buffer : "));
	Serial.print(CC3000_TX_BUFFER_SIZE);
	Serial.println(F(" bytes"));
#endif
}

void displayMACAddress(void)
{
	uint8_t macAddress[6];
	
	if(!cc3000.getMacAddress(macAddress))
	{
		Serial.println(F("Unable to retrieve MAC Address!"));
	}
	else
	{
		Serial.print(F("MAC Address : "));
		cc3000.printHex((byte*)&macAddress, 6);
	}
}

void disableIdleTimout() {
    // Per http://e2e.ti.com/support/low_power_rf/f/851/t/292664.aspx
    // aucInactivity needs to be set to 0 (never timeout) or the socket will close after
    // 60 seconds of no activity
    unsigned long aucDHCP       = 14400;
    unsigned long aucARP        = 3600;
    unsigned long aucKeepalive  = 30;
    unsigned long aucInactivity = 0;
    
    if(aucInactivity == 0)
        Serial.println(F("Setting netapp to not timeout"));
    else
    {
        Serial.print(F("Setting netapp to timeout in "));
        Serial.print(aucInactivity);
        Serial.println(F(" Seconds"));
    }
    
    long iRet = netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive, &aucInactivity);
    
    if (iRet != 0)
    {
        Serial.print(F("Could not set netapp option, iRet = "));
        Serial.println(iRet);
        Serial.println(F(", aborting..."));
        while(1);
    }
}

/* parses commands from CC3000 to control motors
 *
 * Commands come as two bytes:
 * _____________________________________       __________________________________________
 * | x | x | x | A1 | A2 | D | M1 | M2 |       | LW | LW | LW | LW | RW  | RW | RW | RW |
 * -------------------------------------       ------------------------------------------
 *                
 *  3 empty, 2 actuator, 1 dig, 2 mode bits... 1 left sign bit, 3 left bits, 1 right sign bit, 3 right bits
 *
 *
 */
parseCommand(uint16_t command, uint8_t* actuator, uint8_t* dig, uint8_t* mode, uint8_t* left, uint8_t* right)
{
    uint16_t ACTUATOR_MASK = 0x1800 ; // 0b 00011000 00000000
    uint8_t ACTUATOR_OFFSET = 11;
    *actuator = (command & ACTUATOR_MASK) >> ACTUATOR_OFFSET;

    uint16_t DIG_MASK = 0x0400 ; // 0b 00000100 00000000
    uint8_t DIG_OFFSET = 10;
    *dig = (command & DIG_MASK) >> DIG_OFFSET;

    uint16_t MODE_MASK = 0x0300 ; // 0b 00000011 00000000
    uint8_t MODE_OFFSET = 8;
    uint8_t temp = (command & MODE_MASK) >> MODE_OFFSET;
    if(temp == SAFE_MODE)
        *mode = SAFE_MODE;
    else if(temp == AUTONOMOUS_MODE)
        *mode = AUTONOMOUS_MODE;
    else if(temp == MANUAL_MODE)
        *mode = MANUAL_MODE;
    // else
        // TODO ERROR

    uint16_t LEFT_MASK = 0x00F0 ; // 0b 00000000 11110000
    uint8_t LEFT_OFFSET = 4;
    temp = (command & LEFT_MASK) >> LEFT_OFFSET;
    *left = (temp & 0x07);  // ignore sign bit (4th bit), only take lower 3
    // if sign bit is on, then use negative value
    if(temp & 0x08)
        *left = -temp;

    uint16_t RIGHT_MASK = 0x000F ; // 0b 00000000 00001111
    temp = command & RIGHT_MASK;
    *right = (temp & 0x07);  // ignore sign bit (4th bit), only take lower 3
    // if sign bit is on, then use negative value
    if(temp & 0x08)
        *right = -temp;

}

void MuxSelect(uint8_t selection)
{
  switch (selection)
  {
    case MUX_TELEOP:
        digitalWrite(PB7, LOW);     //Set MUX Select low to allow Atmega TX line to go through MUX
        break;
    case MUX_AUTONOMOUS:
        digitalWrite(PB7, HIGH);    // Set MUX Select high to allow FTDI to go through MUX
        break;
  }
}

uint8_t ModeSet(uint8_t newMode)
{
    unsigned char Kill_Command[] = {'!', 'E', 'X', '\r'};//"!EX\r";
    unsigned char Go_Command[] = {'!', 'M', 'G', '\r'};//"!MG\r";

    if (newMode != currentMode)
    {
        switch (newMode)
        {
            case SAFE_MODE:
                MuxSelect(MUX_TELEOP);	
                USART_Transmit (Kill_Command, 4);	// 4 is size of command
                currentMode = SAFE_MODE;
                delay(500);
                break;

            case AUTONOMOUS_MODE:            
                // Listen to all communications by the computer.
                // Do not forward any CC3000 motor controller commands.
                MuxSelect(MUX_AUTONOMOUS);
                USART_Transmit (Kill_Command, 4);
                USART_Transmit (Go_Command, 4);
                digitalWrite(PD4, HIGH);    	        // Autonomous Mode LED on
                delay(500);
                currentMode = AUTONOMOUS_MODE;
                break;

            case MANUAL_MODE:
                MuxSelect(MUX_TELEOP);
                USART_Transmit (Kill_Command, 4);
                USART_Transmit(Go_Command, 4);
                digitalWrite(PD6, HIGH);		// Manual Mode LED on
                delay(500);
                currentMode = MANUAL_MODE;
                break;

            default:
                return -1;
                break;
        }

        return currentMode; // After setting the mode, this should now be the current mode we return
    }
    else
        return currentMode; // The current mode is not different from the new requested mode
}
