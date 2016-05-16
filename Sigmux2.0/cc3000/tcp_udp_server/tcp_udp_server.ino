
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Organization:           UIC Chicago EDT
//  Engineer(s):            Krystian Gebis      Ammar Subei             Basheer Subei             Oliver Panasewicz
//  E-Mail(s):              krgebis@gmail.com   ammarsubei@gmail.com    basheersubei@gmail.com    opanasewicz@gmail.com
// 
//  Project Title:          cc3000.ino
//  Micro controller:       Arduino UNO
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_CC3000.h>
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
#define MANUAL_LED         4     // Manual mode LED
#define AUTO_LED           3     // Autonomous mode LED
#define SAFE_LED           2     // Safe mode LED
#define WIFI_LED           18    // Wifi LED
#define DATA_LED           19    // Data LED
#define CTRL               15    // MUX Select Pin
#define SAFE_MODE          0
#define AUTONOMOUS_MODE    1
#define MANUAL_MODE        2

#define SAFE_DELAY 1000      // delay in ms every time we switch to/from SAFE_MODE

byte lastCommand[2];

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
int cMode_LED; //current mode LED 

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

  Serial.print("Client Removed ");
  Serial.println(clientCount);
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


void processBuffer(ClientList* j)
{
    uint8_t actuator    = 0;
    uint8_t dig         = 0;
    int8_t left        =  0;
    int8_t right       =  0;
    
    uint8_t mode = currentMode;
    parseCommand(lastCommand, &actuator, &dig, &mode, &left, &right);
    if(mode != currentMode)
    {
      ModeSet(mode);
    }   
    
    // for debugging
    // Serial.print("command bytes: "); 
    //OP cc3000.printHexChar(lastCommand, 2);
    //            Serial.print(lastCommand[0], BIN); Serial.print(lastCommand[1], BIN); Serial.println();
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
      /*
      for (int i = 1; i <= 3; i++)
      {
          if ( i != 1)
             canCommand += "_";
      */
      
          //Serial.println(i);
          canCommand += "@01";
          //canCommand += "1";
          canCommand += "!G 1 ";
          
          canCommand += leftMotorVal;
          canCommand += "_@01";
          //canCommand += "1";
          canCommand += "!G 2 ";
          canCommand += rightMotorVal;
          canCommand += "\r";
         
      //}
    }
    
    // append actuatorVal to canCommand if last one has changed (no watchdog for actuators)
    if( actuatorVal != lastActuatorVal)
    {
         canCommand += "@04!G 1 ";
         canCommand += actuatorVal;
         canCommand += "_@04!G 2 ";
         canCommand += actuatorVal;
         canCommand += "\r"; 
    }
    
    lastActuatorVal = actuatorVal;  // update last actuator value
    
    // append digVal to canCommand
    if(dig == 1)
    {
        canCommand += "@05!G 1 ";
        canCommand += digVal;
        canCommand += "\r";
    }
    
    // switch modes, check if we fail
    /*
    if(!ModeSet(currentMode))
    {
      // TODO PRINT OUT ERROR
      Serial.print("Cannot change mode to "); Serial.println(currentMode);
    }
    */
    
    // TODO if manual mode,
    // construct RoboteQ commands and transmit over USART
    if(currentMode == MANUAL_MODE)
    {       
      if(CANFlag == 0)
      {
         Serial.print("Transmitting CAN command from socket ");
         Serial.println(j->socket);
        CANFlag++;
      }
      
      //canCommand = "!G 1 ";
      //canCommand += leftMotorVal;
      //canCommand += "\r";

      if(canCommand != "")
      {
        //canCommand = "@01!G 1 200\r";
        Serial.print("CAN: ");
        Serial.println(canCommand); //Send TX here
        Serial1.write(canCommand.c_str());
      }
      
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
    } 
    else 
    {
        CANFlag--;
        //OP Serial.print("Not transmitting CAN command because we are in mode: ");
        //OP Serial.println(currentMode);
    }
}

int  msgCount = 0;
// Update the state of clients, and accept new client connections.  Should be called
// by the Arduino sketch's loop function.
void loop(void) 
{  
    // Iterate through all the connected clients.
    ClientList* i = clients;
    
    while (i != NULL) 
    {
        // Save the next client so the current one can be removed when it disconnects
        // without breaking iteration through the clients.
        //ClientList* next = i->next;
        // If there's data available, read it a character at a time from the
        // CC3000 library's internal buffer.
        if (i->client.available() > 0) 
        {
            byte command[2];
            
            if( i->client.read(&command, 2) <= 0)
            {
              removeClient(i);
              return;
            }

          

            msgCount++;
            //Serial.print("Msg from socket ");
            //Serial.print(i->socket);
            //Serial.println(msgCount);
            
            lastCommand[0] = command[0];
            lastCommand[1] = command[1];
        
        }
        processBuffer(i);
       if(Serial2.available() > 0)
       {

          //char oMsg[2];
          //char incomingByte1, incomingByte2;
       
          char oMsg = Serial2.read();
          //Serial.print("Sent message: ");
          //Serial.println(oMsg, HEX);
          //incomingByte2 = Serial2.read();
          
          //oMsg[0] = incomingByte1;
          //oMsg[1] = incomingByte2;

          
          if(send(i->socket, &oMsg, 1, 0) < 1)
          {
            Serial.println("Message failed to send on socket");
          }
          else
          {
            Serial.print("Sent message: ");
            Serial.println(oMsg);
            //Serial.print(" ");
            //Serial.println(oMsg[1]);
          }
          
        }
         i =  i->next;
      
    }

  
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
        Serial.print(F("TCP client connected on socket i"));
        Serial.println(tcpSocket);
        char* buf = "1";
        send(tcpSocket, buf, 1, 0);
        // Add the client to the list of connected clients.
        addNewClient(tcpSocket);
      }
    }
  }

  delay(100);  // 1000 / UI UPS
}

void setup(void)
{
  pinMode(SAFE_LED, OUTPUT);
  pinMode(AUTO_LED, OUTPUT);
  pinMode(MANUAL_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  digitalWrite(WIFI_LED, LOW);
  pinMode(DATA_LED, OUTPUT);
  pinMode(CTRL, OUTPUT);

  lastCommand[0] = lastCommand[0] | 0x01;

  cMode_LED = AUTO_LED;
  ModeSet(AUTONOMOUS_MODE);
  //digitalWrite(CTRL, HIGH);
  udpExists = 1;
  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
  
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

  Serial.print("Connecting to AP - ");
  
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println("Failed!");
    while(1);
  }
   
  Serial.println("Connected!");
  
  // Check for DHCP and timeout after 20 seconds
  unsigned long dhcpTimeout = 20000;  // Try for 20 sec
  unsigned long retry       = 0;
   
  for(unsigned long t = millis(); ((millis() - t) <= dhcpTimeout);)
  {
    Serial.print("Querying DHCP - ");
    
    if(cc3000.checkDHCP())
    {
      Serial.println(F("OK"));
      digitalWrite(WIFI_LED, HIGH);
      break;
    }
    else
      Serial.println(retry+1);
    
    ++retry;
    delay(1000);
  }
  
    // Attempt connection to UDP
  // uint32_t bindIP = displayConnectionDetails();
  displayConnectionDetails();
  
  // Initialize the echo server
  echoSetup();
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
          digitalWrite(WIFI_LED, HIGH);
          delay(300);
          digitalWrite(WIFI_LED, LOW);
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
        digitalWrite(CTRL, LOW);     //Set MUX Select low to allow Atmega TX line to go through MUX
        Serial.println("Mux set to low");
        break;
    case MUX_AUTONOMOUS:
        digitalWrite(CTRL, HIGH);    // Set MUX Select high to allow FTDI to go through MUX
        Serial.println("Mux set to high");
        break;
  }
}

uint8_t ModeSet(uint8_t newMode)
{

    Serial.print("Mode change request is: "); Serial.println(newMode);
    Serial.print("Current mode is: "); Serial.println(currentMode);
    // don't do anything if we're in same mode
    if (newMode != currentMode)
    {
        switch (newMode)
        {
            case SAFE_MODE:
                //DEFAULTING SAFE TO AUTO MODE
                
                // USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_TELEOP);  
                currentMode = SAFE_MODE;
                digitalWrite(cMode_LED, LOW);
                digitalWrite(SAFE_LED, HIGH);    // Manual Mode LED on
                cMode_LED = SAFE_LED;
                //digitalWrite(CTRL, LOW);
                delay(500);
                Serial.println("Now in Safe Mode");
                break;
                
                /*
                // Listen to all communications by the computer.
                // Do not forward any CC3000 motor controller commands.
                // USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_AUTONOMOUS);
                // USART_Transmit (RESUME_COMMAND, 35);  // re-activate motor controllers
                digitalWrite(cMode_LED, LOW);
                digitalWrite(AUTO_LED, HIGH);              // Autonomous Mode LED on
                cMode_LED = AUTO_LED;
                //digitalWrite(CTRL, HIGH);
                delay(500);  // TODO why do we need this delay? we don't want this to let the udp buffer fill up?
                currentMode = AUTONOMOUS_MODE;
                Serial.println("Now in Autonomous Mode");
                break;
                */

            case AUTONOMOUS_MODE:            
                // Listen to all communications by the computer.
                // Do not forward any CC3000 motor controller commands.
                // USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_AUTONOMOUS);
                // USART_Transmit (RESUME_COMMAND, 35);  // re-activate motor controllers
                digitalWrite(cMode_LED, LOW);
                digitalWrite(AUTO_LED, HIGH);              // Autonomous Mode LED on
                cMode_LED = AUTO_LED;
                //digitalWrite(CTRL, HIGH);
                delay(500);  // TODO why do we need this delay? we don't want this to let the udp buffer fill up?
                currentMode = AUTONOMOUS_MODE;
                Serial.println("Now in Autonomous Mode");
                break;

            case MANUAL_MODE:
                // USART_Transmit (KILL_COMMAND, 35);
                delay(SAFE_DELAY);
                MuxSelect(MUX_TELEOP);
                // USART_Transmit (RESUME_COMMAND, 35);  // re-activate motor controllers
                digitalWrite(cMode_LED, LOW);
                digitalWrite(MANUAL_LED, HIGH);    // Manual Mode LED on
                cMode_LED = MANUAL_LED;
                //digitalWrite(CTRL, LOW);
                delay(500);
                currentMode = MANUAL_MODE;
                Serial.println("Now in Manual Mode");
                break;

            default:
                return -1;
                break;
        }

        return currentMode; // After setting the mode, this should now be the current mode we return
    }
    else
    {
        Serial.println("Not switching");
        return currentMode; // The current mode is not different from the new requested mode
    }      
}
