# Thermode PWM code

  
This is the Arduino driver code for a Thermode using PWM.
	

## Using the code

The commands that are available are:

* DIAG          - Get diagnostics
* MOVE;XX       - Move temp up/down for XX us
* START         - Send 100ms TTL pulse to start program
* SHOCK;xx;yy   - Set Digitimer stimulus duration (xx) and interval between pulses (yy) in ms
* PULSE8        - Send a 1ms pulse to pin 8 (async ie should be possible during complex waveform)
* PULSE9        - Send a 1ms pulse to pin 9 (see above)   
* GETTIME       - Get Arduino time in ms
* DEBUG;XX      - Set debug state (0: OFF)
* HELP          - This command
* INITCTC;xx    - Initialize complex time courses (cTC) with cycle xx in ms (500 max)
* LOADCTC;xx    - add pulse to cTC queue (xx in ms) -xx means temperature decrease, max 2500 items
* QUERYCTC      - status of the cTC queue
* EXECCTC       - new way to execute cTC queue using precise PWM functionality
* FLUSHCTC      - clear cTC queue
* STATUS        - check whether EXECCTC(PWM) is running"));
* READVAS       - returns time in ms and the reading of ADC0;


## General Operation

See Thermoino_PWM.pdf for details