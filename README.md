# Thermode PWM code

  
This is the Arduino driver code for a Thermode using PWM.
	
Version 3.50

## Using the code

The commands that are available are:

* VER           - Print version
* DIAG          - Get diagnostics
* MOVE;XX       - Move temp up/down for XX us
* START         - Send 40ms TTL pulse to start thermode
* SHOCK;nn(;yy) - Digitimer stimuli number (nn) @interval 1100us OR additionally specify interval between pulses (yy) in us (>1000) 
* PULSE8        - Send 1ms TTL pulse to pin 8
* PULSE9        - Send 1ms TTL pulse to pin 9
* GETTIME       - Get Arduino time in ms 
* DEBUG;XX      - Set debug state (0: OFF) (1: ON)
* HELP          - This command
* INITCTC;xx    - Initialize complex time courses (ctc_data) with cycle xx in ms (500 max)
* LOADCTC;xx    - add pulse to ctc_data queue (xx in ms) -xx means temperature decrease, max 2500 items
* QUERYCTC(;yy) - status of the ctc_data queue (yy=3 to get all entries)
* EXECCTC       - execute ctc_data queue using precise PWM
* FLUSHCTC      - reset ctc and clear ctc_data queue
* STATUS        - check whether anything (thermode or digitimer) is running
* STATUS_D      - check whether digitimer is running
* STATUS_T      - check whether thermode is running
* KILL          - stop all activity (thermode and digitimer)
* KILL_D        - stop activity of digitimer
* KILL_T        - stop activity of thermode
* READVAS       - returns time in ms and the reading of the VAS potentiometer 0..1023
* SETID         - enter new name for thermoino, saved permanently in EEPROM
* GETID         - return name of thermoino from EEPROM

## General Operation

See Thermoino_PWM.docx for details