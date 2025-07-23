# Polaris-GENERAL---CANBUS-Bridge
Multitasking Canbus bridge on my Polaris EV Conversion using RTOS

While the code is intended to hack the CANBUS on a Polaris General, the approach and hardware could be used to hack the CANBUS on pretty much any vehicle.

It uses a board made by: https://copperhilltech.com/teensy-4-0-triple-can-bus-board-with-two-can-2-0b-and-one-can-fd-port/
I bought mine in Europe from: https://www.skpang.co.uk/products/teensy-4-0-triple-can-board-include-teensy-4-0?_pos=1&_sid=9e22a7094&_ss=r

The board that arrived is updated slightly from the one pictured - it has a load of GPIO pins exposed, which I've used for switching various things.

In my case, I've cut the CAN wires where they go in to the main ECU.  One port of the bridge connects only to the ECU, one to the rest of the vehicle and the third to an 
isolated CAN network for the Electric Vehicle components (Hyper9 + Orion BMS + Elcon Charger).

Most CAN messages are passed transparently between the ECU and the Vehicle.  Some, however are intercepted on receive.  The data in the message is altered to reflect what 
I need and it's sent on it's way.  The intercepted messages include RPM, Speed, fuel level, dash warning lights & engine temperature.  Values for these are read or calculated from packets received 
from the EV CAN network.

Initially, I had problems with the rate messages were sent.  The dash expected messages to arrive at a certain frequency for different displays.  Too infrequent & the dash didn't update
too often and it jammed the bus.  My solution is only to send data when a message arrives from the ECU.  It obviously knows how often they need sending, so I just use it's timing.

*Update* This branch is for the RTOS multitasking version.  In the previous version the rate at which messages were being received, effectively blocked transmissions.  I had to limit the packets sent to non-essential things, like the tachometer.
