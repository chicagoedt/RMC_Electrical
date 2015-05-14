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
#define TELEOP         0
#define AUTONOMOUS     1
#define PB7            11    // MUX select pin
#define PD4            4     // Autonomous mode LED
#define PD6            12    // Manual mode LED
#define PC6            5     // WLAN LED

#define SafeMode       0
#define AutoMode       1
#define ManualMode     2

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
			char val = 0;
			udpClient.read(&val, 1);
			Serial.println(char(val+48));
                        if(val == 0)
                        {
                          digitalWrite(4, HIGH);
                          digitalWrite(12, LOW);
                        }
                        if(val == 1)
                        {
                          digitalWrite(4, LOW);
                          digitalWrite(12, HIGH);
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

void MuxSelect(uint8_t selection)
{
  switch (selection)
  {
    case TELEOP:
      digitalWrite(PB7, LOW);     //Set MUX Select low to allow Atmega TX line to go through MUX
      break;
    case AUTONOMOUS:
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
      case SafeMode:
        MuxSelect(TELEOP);	
       // USART_Transmit (Kill_Command, 4);	// 4 is size of command
        // Stop the Linear Actuator 01/11
        // PORTD &= ~(1 << PD7);
        // PORTD &= ~(1 << PD6);
//        PORTE |= (1 << PORTE2);			// Safe Mode LED on
//        delay(500);
        currentMode = SafeMode;
        break;
      
      case AutoMode:            
        //Listen to all communications by the computer.
        //Do not forward any CC3000 motor controller commands.
      
        MuxSelect(AUTONOMOUS);
       // USART_Transmit (Kill_Command, 4);
       // USART_Transmit (Go_Command, 4);
        digitalWrite(PD4, HIGH);    	        // Autonomous Mode LED on
        delay(500);
        currentMode = AutoMode;
        break;
      
      case ManualMode:
        MuxSelect(TELEOP);
       // USART_Transmit (Kill_Command, 4);
       // USART_Transmit(Go_Command, 4);
        digitalWrite(PD6, HIGH);		// Manual Mode LED on
        delay(500);
        currentMode = ManualMode;
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
