# Thermode PWM code

  
This is the Arduino driver code for a Thermode using PWM.
	

## Using the code

The commands that are available are:

* DIAG          - Get diagnostics
* MOVE;XX       - Move temp up/down for XX us
* START         - Send 100ms TTL pulse to start program
* SHOCK;xx;yy   - Set Digitimer stimulus duration (xx) and interval between pulses (yy) in ms
* GETTIME       - Get Arduino time in ms
* DEBUG;XX      - Set debug state (0: OFF)
* HELP          - This command
* INITCTC;xx    - Initialize complex time courses (cTC) with cycle xx in ms (500 max)
* LOADCTC;xx    - add pulse to cTC queue (xx in ms) -xx means temperature decrease, max 3000 items
* QUERYCTC      - status of the cTC queue
* EXECCTC       - old way to execute cTC queue using loop(), use EXECCTCPWM instead
* EXECCTCPWM    - new way to execute cTC queue using precise PWM functionality
* FLUSHCTC      - clear cTC queue
* STATUSCTC     - check whether EXECCTC(PWM) is running"));


## General Operation

See Thermoino_PWM.pdf for details