# open_evse
Firmware for an Arduino Uno board that limits the current level available to the car by reducing the duty cycle
of an incoming master pilot signal (TTL) and generates a phase synchrous output pilot (TTL). There is no interference with
any error signaling since output needs ANDed with input by two external diodes. 
Furthermore an external energy meters S0 pulses are counted.
