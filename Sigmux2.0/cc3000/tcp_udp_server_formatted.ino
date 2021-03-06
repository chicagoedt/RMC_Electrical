nclude <Adafruit_CC3000.h>
#include <SPI.h>
#include "utility/debug.h"
#include "utility/socket.h"

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   7  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  6
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIV2); // you can change this clock speed

#define WLAN_SSID       "chicagoedt"           // cannot be longer than 32 characters!
#define WLAN_PASS       "notrightnow"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define LISTEN_PORT           5000 // What TCP port to listen on for connections.  The echo protocol uses port 7.
#define LISTEN_PORT_UDP       5002 // What UDP Port to listen on for connections
#define MAX_CLIENTS           3   // The CC3000 docs advise that it has 4 sockets available to client code.
                                   // One socket is consumed listening for new connections, and the remaining sockets
                                   // are available for client connections.
                                   // In practice it appears you can go higher than 4 sockets (up to about 7), but
                                   // be careful as it might cause the CC3000 to behave unexpectedly.

#define SERIAL_CONVERT
#define MUX_TELEOP         0
#define MUX_AUTONOMOUS     1
#define PB7            11    // MUX select pin
#define PD6            3     // Manual mode LED
#define PD4            4     // Autonomous mode LED
#define PC6            5     // WLAN LED

#define SAFE_MODE       0
#define AUTONOMOUS_MODE       1
#define MANUAL_MODE     2

#define SAFE_DELAY 1000      // delay in ms every time we switch to/from SAFE_MODE

static volatile uint8_t currentMode;

int lastActuatorVal = 0;
int CANFlag = 0;
// constants multiplied by value from panel udp socket
#define LEFT_WHEEL_CONSTANT 100
#define RIGHT_WHEEL_CONSTANT 100
#define ACTUATOR_CONSTANT 100
#define DIG_CONSTANT 500

    // Emergency stop command (!EX) to all motor controllers
    const unsigned char KILL_COMMAND[] = {'@', '0', '1', '!', 'E', 'X',
        '_', '@', '0', '2', '!', 'E', 'X',
        '_', '@', '0', '3', '!', 'E', 'X',
        '_', '@', '0', '4', '!', 'E', 'X',
        '_', '@', '0', '5', '!', 'E', 'X', '\r'};  // length 35 chars
    // Resume command to re-activate and come out of emergency stop
    const unsigned char RESUME_COMMAND[] = {'@', '0', '1', '!', 'M', 'G',
        '_', '@', '0', '2', '!', 'M', 'G',
        '_', '@', '0', '3', '!', 'M', 'G',
        '_', '@', '0', '4', '!', 'M', 'G',
        '_', '@', '0', '5', '!', 'M', 'G', '\r'};  // length 35 chars
    // current command being sent to roboteq motor controller
    // example "@01!G 1 1000_@01!G 2 -1000_@02!G 1 1000_@02!G 2 -1000_@03!G 1 1000_@03!G 2 -1000_@04!G 1 700_@04!G 2 700_@05!G 1 500\r"
    // max length: 14 bytes for each channel for each roboteq (4 of them have 2 channels, 1 has one channel, total 9 channels), so 14*9 = 126 bytes, add one for safety
//    unsigned char roboteqCommand = "@01!G 1 1000_@01!G 2 -1000_@02!G 1 1000_@02!G 2 -1000_@03!G 1 1000_@03!G 2 -1000_@04!G 1 700_@04!G 2 700_@05!G 1 500\r";
    

int loopCount = 0;


// Define a simple forward linked list structure to keep track of 
// connected clients.  This client list is kept as a linked list 
// because the CC3000 client instances have a non-trivial amount of 
// internal state (buffers, etc.) which we don't want to spend memory 
// consuming unless a client is connected.  Because the size of the list
// is small and random access to entries is rare (happens only on
// disconnect) a forward list (i.e. no back/previous pointer) is
// sufficient.
struct ClientList {
  int socket;
  Adafruit_CC3000_Client client;
  ClientList* next;
};

// Global variable to hold the ID of the listening socket.
int listenSocket;
int listenSocketUDP;
int udpExists;

// Global variable to hold the start of the connected client list.
ClientList* clients;

// Global variable to keep track of how many clients are connected.
int clientCount;

// Function to add a new client to the connected client list.
void addNewClient(int socket) {
  // Allocate memory for a new list entry.
  // In general be very careful with heap/dynamic memory use in an Arduino sketch
  // because the function stack and heap share the same limited amount of memory and
  // can easily overflow.  In this case the number of connected clients is low
  // so the risk of using up all the memory is low.
  ClientList* client = (ClientList*) malloc(sizeof(ClientList));
  if (client == NULL) {
    Serial.println(F("Error! Couldn't allocate space to store a new client."));
    return;
  }
  // Setup the new client as the front of the connected client list.
  client->next = clients;
  client->socket = socket;
  client->client = Adafruit_CC3000_Client(socket);
  clients = client;
  // Increment the count of connected clients.
  clientCount++;
}

// Remove a client from the connected client list.
void removeClient(struct ClientList* client) {
  if (client == NULL) {
    // Handle null client to delete.  This should never happen
    // but is good practice as a precaution.
    return;
  }
  if (clients == client) {
    // Handle the client to delete being at the front of the list.
    clients = client->next;
  }
  else {
    // Handle the client to delete being somewhere inside the list.
    // Iterate through the list until we find the entry before the
    // client to delete.
    ClientList* i = clients;
    while (i != NULL && i->next != client) {
      i = i->next;
    }
    if (i == NULL) {
      // Couldn't find the client to delete, do nothing.
      return;
    }
    // Remove the client from the list.
    i->next = client->next;
  }
  // Free the memory associated with the client and set the
  // pointer to null as a precaution against dangling references.
  free(client);
  client = NULL;
  // Decrement the count of connected clients.
  clientCount--;
}

// Set up the echo server and start listening for connections.  Should be called once
// in the setup function of the Arduino sketch.
void echoSetup() {
  // Most of the calls below are to CC3000 firmware API's. You can find the documentaiton for these calls at:
  //   http://software-dl.ti.com/ecs/simplelink/cc3000/public/doxygen_API/v1.11.1/html/index.html
  
  // Set the CC3000 inactivity timeout to 0 (never timeout).  This will ensure the CC3000
  // does not close the listening socket when it's idle for more than 60 seconds (the
  // default timeout).  See more information from:
  //   http://e2e.ti.com/support/low_power_rf/f/851/t/292664.aspx
  unsigned long aucDHCP       = 14400;
  unsigned long aucARP        = 3600;
  unsigned long aucKeepalive  = 30;
  unsigned long aucInactivity = 0;
  if (netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive, &aucInactivity) != 0) {
    Serial.println(F("Error setting inactivity timeout!"));
    while(1);
  }
  // Create a TCP socket
  listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (listenSocket < 0) {
    Serial.println(F("Couldn't create listening socket!"));
    while(1);
  }
  
  listenSocketUDP = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (listenSocketUDP < 0) {
    Serial.println(F("Couldn't create listening socket!"));
    while(1);
  }
  // Set the socket's accept call as non-blocking.
  // This is required to support multiple clients accessing the server at once.  If the listening
  // port is not set as non-blocking your code can't do anything while it waits for a client to connect.
  if (setsockopt(listenSocket, SOL_SOCKET, SOCKOPT_ACCEPT_NONBLOCK, SOCK_ON, sizeof(SOCK_ON)) < 0) {
    Serial.println(F("Couldn't set TCP socket as non-blocking!"));
    while(1);
  }

  if (setsockopt(listenSocketUDP, SOL_SOCKET, SOCKOPT_ACCEPT_NONBLOCK, SOCK_ON, sizeof(SOCK_ON)) < 0) {
    Serial.println(F("Couldn't set UDP socket as non-blocking!"));
    while(1);
  }
  // Bind the socket to a TCP address.
  sockaddr_in address;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(0);     // Listen on any network interface, equivalent to INADDR_ANY in sockets programming.
  address.sin_port = htons(LISTEN_PORT);  // Listen on the specified port.
  if (bind(listenSocket, (sockaddr*) &address, sizeof(address)) < 0) {
    Serial.println(F("Error binding TCP listen socket to address!"));
    while(1);
  }

  // Bind the socket to a UDP address.
  sockaddr_in addressUDP;
  addressUDP.sin_family = AF_INET;
  addressUDP.sin_addr.s_addr = htonl(0);     // Listen on any network interface, equivalent to INADDR_ANY in sockets programming.
  addressUDP.sin_port = htons(LISTEN_PORT_UDP);  // Listen on the specified port.
  if (bind(listenSocketUDP, (sockaddr*) &addressUDP, sizeof(addressUDP)) < 0) {
    Serial.println(F("Error binding UDP listen socket to address!"));
    while(1);
  }
  // Start listening for connectings.
  // The backlog parameter is 0 as it is not supported on TI's CC3000 firmware.
  if (listen(listenSocket, 0) < 0) {
    Serial.println(F("Error opening TCP socket for listening!"));
    while(1);
  }
  else{
    Serial.println(F("TCP port is listening"));
  }

  if (listen(listenSocketUDP, 0) < 0) {
    Serial.println(F("Error opening UDP socket for listening!"));
    while(1);
  }
  // Initialize client list as empty.
  clients = NULL;
  clientCount = 0;
  Serial.println(F("Listening for connections..."));
}

// Update the state of clients, and accept new client connections.  Should be called
// by the Arduino sketch's loop function.
void echoLoop() {  
  // Iterate through all the connected clients.
  ClientList* i = clients;
  
  while (i != NULL) {
    // Save the next client so the current one can be removed when it disconnects
    // without breaking iteration through the clients.
    //ClientList* next = i->next;
    // If there's data available, read it a character at a time from the
    // CC3000 library's internal buffer.
    if (i->client.available() > 0) {

      //Serial.println("attempting to read from TCP socket");
            byte command[2];
            i->client.read(&command, 2);

            if( command[0] == 'a') {
              digitalWrite(4, HIGH);
            }

            uint8_t actuator    = 0;
            uint8_t dig         = 0;
            int8_t left        =  0;
            int8_t right       =  0;
            
            uint8_t mode = currentMode;
            parseCommand(command, &actuator, &dig, &mode, &left, &right);
            currentMode = mode;  // workaround because currentMode is volatile
            
              // for debugging
              // Serial.print("command bytes received: "); 
              cc3000.printHexChar(command, 2);
//            Serial.print(command[0], BIN); Serial.print(command[1], BIN); Serial.println();
//            Serial.print("actuator: "); Serial.println(actuator);
//            Serial.print("dig: "); Serial.println(dig);
//            Serial.print("mode: "); Serial.println(currentMode);
//            Serial.print("left: "); Serial.println(left);
//            Serial.print("right: "); Serial.println(right);

               String canCommand; // Start string off with "@0" 
               int leftMotorVal = left * LEFT_WHEEL_CONSTANT;
               int rightMotorVal = right * RIGHT_WHEEL_CONSTANT;
               int actuatorVal = actuator * ACTUATOR_CONSTANT;
               int digVal = dig * DIG_CONSTANT;
              
              
              // append motor value to canCommand if not zero
              if ( leftMotorVal != 0 || rightMotorVal != 0)
              { 
                for (int i = 1; i <= 3; i++)
                {
                    if ( i != 1)
                    {
                       canCommand += "_";
                    }
                    //Serial.println(i);
                    canCommand += "@0";
                    canCommand += i;
                    canCommand += "!G 1 ";
                    
                    canCommand += leftMotorVal;
                    canCommand += "_@0";
                    canCommand += i;
                    canCommand += "!G 2 ";
                    canCommand += rightMotorVal;
                   
                }
                  
              }
              
              // append actuatorVal to canCommand if last one has changed (no watchdog for actuators)
              if( actuatorVal != lastActuatorVal)
              {
                 canCommand += "@04!G 1 ";
                 canCommand += actuatorVal;
                 canCommand += "_@04!G 2 ";
                 canCommand += actuatorVal; 
              }
              lastActuatorVal = actuatorVal;  // update last actuator value
              
              // append digVal to canCommand
              if(dig > -0.8){
                canCommand += "@05!G 1 ";
                canCommand += digVal;
                canCommand += "\r";
              }

            
            // switch modes, check if we fail
            if(!ModeSet(currentMode))
            {
                // TODO PRINT OUT ERROR
                Serial.print("Cannot change mode to "); Serial.println(currentMode);
            }

            
            //Serial.print("CAN command is: ");
            Serial.println(canCommand); //Send TX here
            
            // TODO if manual mode,
            // construct RoboteQ commands and transmit over USART
            if(currentMode == MANUAL_MODE)
            {       
                //if(CANFlag == 0){
                  Serial.print("Transmitting CAN command from socket ");
                  Serial.println(i->socket);
                  //CANFlag++;
                //}
                canCommand = "!G 1 ";
                canCommand += leftMotorVal;
                canCommand += "\r";
                
                Serial1.write(canCommand.c_str());

                //FOR ROBOTEQ DEBUGGING 
                /*
                while(Serial1.available() > 0)
                {
                  Serial.println("reading from roboteq");
                  char ch1 = Serial1.read();
                  Serial.print(ch1);
                }
                */
                  
                // USART_Transmit(canCommand, canCommand.length());
            } else {
                CANFlag--;
                Serial.print("Not transmitting CAN command because we are in mode: ");
                Serial.println(currentMode);
            }
      
      //uint8_t ch = i->client.read();
      // Echo the read byte back out to the client immediately.
      //Serial.println(ch);
      /*
      if (i->client.write(ch) == 0) {
        Serial.println(F("Error writing character to client!"));
      }
      */
    }
    // Check if the client is disconnected and remove it from the active client list. DOES NOT WORK
    // COMMENT OUT until removal is fixed
    /*if (!i->client.connected()) {
      Serial.print(F("Client on socket "));
      Serial.print(i->socket);
      Serial.println(F(" disconnected."));
      removeClient(i);
      // Note that i is now NULL!  Don't try to dereference it or you will
      // have a bad day (your Arduino will reset).
    }*/
    // Continue iterating through clients.
     i =  i->next;
  }// end of while (clients)
  
  // Handle new client connections if we aren't at the limit of connected clients.
  if (clientCount < MAX_CLIENTS) {
    // Accept a new socket connection.  Because we set the listening socket to be non-blocking 
    // this will quickly return a result with either a new socket, an indication that nothing
    // is trying to connect, or an error.
    // The NULL parameters allow you to read the address of the connected client but are
    // unused in this sketch.  See TI's documentation (linked in the setup function) for
    // more details.

    if(udpExists == 0)
    {
      // Check if a client is connected to a new socket.
      if (listenSocketUDP > -1) {
        Serial.print(F("UDP client connected on socket "));
        Serial.println(listenSocketUDP);
        // Add the client to the list of connected clients.
        addNewClient(listenSocketUDP);
        udpExists = 1;
      }
    }
    else
    {
      // Check if a client is connected to a new socket.
      int tcpSocket = accept(listenSocket, NULL, NULL);
      // Check if a client is connected to a new socket.
      if (tcpSocket > -1) {
        Serial.print(F("TCP client connected on socket "));
        Serial.println(tcpSocket);
        // Add the client to the list of connected clients.
        addNewClient(tcpSocket);
      }
    }
  }
}

void setup(void)
{
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  udpExists = 0;
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println(F("Hello, CC3000!\n")); 

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */
  Serial.println(F("\nInitializing..."));
  while (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    delay(500);
  }
/*
  // Check Firmware version
  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113)
  {
    Serial.println(F("Wrong firmware version!"));
    while(1);
  }
  */

  //display some info
  //displayDriverMode();
  //displayMACAddress();

  Serial.print(F("Connecting to AP - "));
  
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
// Check for DHCP and timeout after 20 seconds
    unsigned long dhcpTimeout = 20000;  // Try for 20 sec
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
        //Reboot("FAILED - Rebooting!!!", 2);
      
      Serial.println(retry+1);
    }
    
    ++retry;
    delay(1000);
  }
  
    // Attempt connection to UDP
  // uint32_t bindIP = displayConnectionDetails();
  displayConnectionDetails();
  
  // Initialize the echo server
  echoSetup();
}

void loop(void)
{
  echoLoop();
  /*
  // Update the echo server.
  if(loopCount < 70)
  {
    echoLoop();
    delay(500);
    Serial.println(loopCount);
    loopCount++;
  }
  else
  {
    closesocket(listenSocket);
  }
  */

}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}


void Reboot(const char* errMsg, uint32_t errCode)
{
  Serial.println(errMsg);
  Serial.println(F("--REBOOTING--"));
  //while(true);
  //cc3000.reboot();
        for(uint32_t x = 0; x < errCode; x++);
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

/* parses commands from CC3000 to control 
motors
 *
 * Commands come as two bytes (big-endian):
 * _____________________________________       __________________________________________
 * | x | x | x | A1 | A2 | D | M1 | M2 |       | LW | LW | LW | LW | RW  | RW | RW | RW |
 * -------------------------------------       ------------------------------------------
 *                
 *  3 empty, 2 actuator, 1 dig, 2 mode bits... 1 left sign bit, 3 left bits, 1 right sign bit, 3 right bits
 *
 * We have to convert them to little-endian. Note that left and right values are signed (2's complement).
 *
 */
void parseCommand(byte* comm, uint8_t* actuator, uint8_t* dig, uint8_t* mode, int8_t* left, int8_t* right)
{
    // convert command bytes to little-endian
    uint16_t command = (uint16_t)( (comm[0] << 8) | (comm[1]) );
    
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

    // note: left and right values are signed (2's complement)
    uint16_t LEFT_MASK = 0x00F0 ; // 0b 00000000 11110000
    //uint8_t LEFT_OFFSET = 4;
    *left = (int8_t)((command & LEFT_MASK)) / 16;


    uint16_t RIGHT_MASK = 0x000F ; // 0b 00000000 00001111
    *right = ((int8_t)((command & RIGHT_MASK) << 4) ) / 16;

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

    // don't do anything if we're in same mode
    if (newMode != currentMode)
    {
        switch (newMode)
        {
            case SAFE_MODE:
                // USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_TELEOP);  
                currentMode = SAFE_MODE;
                delay(500);
                break;

            case AUTONOMOUS_MODE:            
                // Listen to all communications by the computer.
                // Do not forward any CC3000 motor controller commands.
                // USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_AUTONOMOUS);
                // USART_Transmit (RESUME_COMMAND, 35);  // re-activate motor controllers
                digitalWrite(PD4, HIGH);              // Autonomous Mode LED on
                delay(500);  // TODO why do we need this delay? we don't want this to let the udp buffer fill up?
                currentMode = AUTONOMOUS_MODE;
                break;

            case MANUAL_MODE:
                // USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_TELEOP);
                // USART_Transmit (RESUME_COMMAND, 35);  // re-activate motor controllers
                digitalWrite(PD6, HIGH);    // Manual Mode LED on
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
