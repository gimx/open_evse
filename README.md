# EVSE Solar Limiter

Load balancing for commercial EVSE, requires access to and interruption of the pilot signal at TTL

Firmware for an Arduino Uno board that limits the charge current level available to the car by reducing the duty cycle
phase synchrous output pilot signal based on an incoming master pilot signal. 

There is no interference with
any error signaling since output needs ANDed with input by two external diodes. 
Furthermore an external energy meters S0 pulses are counted.
