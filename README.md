# Polaris-GENERAL---CANBUS-Bridge
Used to hack the canbus on my Polaris EV Conversion

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

The whole thing works really well!

One problem I had was the ECU stopped sending road-speed data once the engine was removed (no idea why).  It is (I think) requesting the data from the ABS controller, calculating speed
and sending it over CAN.  Once the engine was gone, it stopped requesting and sending the speed data - though the ABS continues to work OK.
I solved this by adding a GPS and reading speed data from that.  A GPS based speedo is no bad thing really - at least it's accurate.
*Update* I found speed data being sent by the ABS Controller.  Now that data is used to drive the speedo until the GPS has signal, then switches to GPS speed.

I've yet to figure out how the odometer on the dash was driven - and implement a way of spoofing the data.  Ping me a message if you have any info on how CANBUS odometers work?  
Is it a 'tick' message sent, say every few feet or an absolute distance travelled since the ECU was installed?
*Update* I found someone on line had figured it out for another vehicle.  There was a byte in the Check Engine Light frame which incremented by one for every 0.05 miles (I'm guessing it's actually every 0.1km).  I've looked for this, but not found it YET!
