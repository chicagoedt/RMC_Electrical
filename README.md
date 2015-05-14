# RMC_Electrical

Anything RMC Specific should be in this repo.


# Sigmux 2.0

## CC3000 library
We decided to use the Adafruit Arduino library for the CC3000. We are using a modified version of the library found [here](https://github.com/oxoocoffee/Adafruit_CC3000_Library).

## Overview
* Initialize
    * Start in safe mode
    * Establish connection to CC3000 (command center)
* Loop: listen to commands from command center:
    * if auto mode:
        * first engage safe mode for x seconds
        * then switch MUX select pin to HIGH (now ARM can talk to motor controller)
    * if manual mode:
        * first engage safe mode for x seconds
        * then switch MUX select pin to LOW (now Serial line to motor controllers is open)
        * relay/translate any commands you receive from command center (CC3000) to motor controller
    * if safe mode:
        * switch MUX select pin to LOW
        * but don't send anything on the Serial line (or send empty command)

## CC3000 Initialization
* Create cc3000 object in global context
* Setup:
    * Check that cc3000.begin() returns true
    * Disable netapp timeout
    * Check Firmware version
    * Print out some debugging info
    * Attempt connection to AP
    * Check for DHCP and timeout after 20 seconds
    * Attempt connection to UDP once
* Loop:
    * if UDP not connected, reboot (disabled?)
    * if UDP available, read a value, switch LEDs on
    * delay(100)
