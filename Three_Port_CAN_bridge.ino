/*
  Board = Teensy 4.0
  A three port CAN bridge (with j1939 if you uncomment it) decoding for the Arduino Teensy 4

  Developed to integrate an Electric Drive Train into a 2021 Polaris General XP 1000
  
  CAN_Vehicle Port connects to the Vehicle CAN Harness
  CAN_EV Port connects to the EV CAN Harness (Orion BMS, Elcon Charger & HyperDrive)
  CAN_ECU Port connects to the original ECU only.
  
  Requires the following library for Teensy 4.0:

  https://github.com/tonton81/FlexCAN_T4

  Ports 1 & 3 are mirrored - with some data filtered out from the ECU (CAN_ECU) to the vehicle (CAN_Vehicle) 
  CAN_EV connects to the EV Drive train & extracts useful information.
  This is used to generate new messages to replace the ones filtered out.

  For example, with the ICE removed, the RPM will always read 0.  We read the motor RPM
  and generate a suitable frame to send to the vehicle, but not back to the ECU
*/

#include <FlexCAN_T4.h>
#include "canframe.h"
//#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <TeensyThreads.h>

FlexCAN_T4<CAN1, RX_SIZE_8, TX_SIZE_8> CAN_Vehicle;  //To the vehicle
FlexCAN_T4<CAN2, RX_SIZE_8, TX_SIZE_8> CAN_EV;  //To the EV Drivetrain
FlexCAN_T4<CAN3, RX_SIZE_8, TX_SIZE_8> CAN_ECU;  //To the ECU

// Mutex for protecting shared resources
Threads::Mutex canMutex;

//GPS Stuff
//static const int GPSRxPin = 19, GPSTxPin = 18;
//static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
//TinyGPSPlus gps;

//each scale factor converts RPMs to 500*MPH, (i.e. rpm*scaleFactor = mph*500)
const float RPM_SCALE_LOW     = 20.74; //low gear
const float RPM_SCALE_HIGH    = 11.88; //high gear

//Things we might want to read and send to the EV Drive Train:  (Byte count starting from Zero [0])
const uint32_t CAN_ID_SHIFTER        = 0x18F00500; //Byte [5] - 4C=L, 48=H, 4E=N, 52=R, 50=P - Gear Selector
const uint32_t CAN_ID_THROTTLE       = 0x18FEF200; //Byte [3] - 00=Minimum, ED=Maximum
const uint32_t CAN_ID_HANDBRAKE      = 0x18FEF100; //Byte [0] - 37=Applied, 33=Released

const uint32_t CAN_ID_FOOTBRAKE      = 0x18FEF1FE; //Byte [3] - DF=Applied, CF=Released
const uint32_t CAN_ID_SEATBELT       = 0x18FF6CFE; //Byte [1] - F7=Secured, F3=Released - Seatbelt
const uint32_t CAN_ID_DIFFLOCK       = 0x18F006FE; //Byte [1] - DF=Locked, CF=Open
const uint32_t CAN_ID_4WD            = 0x18FDDFFE; //Byte [0] - FC=2WD, FD=4WD
const uint32_t CAN_ID_MOTION         = 0x185217FE; //Byte [0] Distance Covered,  Byte [0] LSB, Byte [3] MSB - 4 bytes 5m per bit
const uint32_t CAN_ID_OILPRESSURE    = 0x18FD08FE; //

//Things we might want to filter from the ECU data, generate new & write to the vehicle:  (Byte count starting from Zero [0])
const uint32_t CAN_ID_SPEED          = 0x18FEF100; //Byte [1] LSB, Byte [2] MSB - Speed mph x 500
const uint32_t CAN_ID_RPM            = 0x18FF6600; //Byte [0] LSB, Byte [1] MSB - RPM
const uint32_t CAN_ID_CEL            = 0x18FECA00; //Various warning lights, including Check Engine
const uint32_t CAN_ID_ENG_TEMP       = 0x18FEEE00; //Byte [0] - Engine Temperature (Degrees C + 40)
const uint32_t CAN_ID_FUEL_LEVEL     = 0x18FEFC00; //Byte [1] - Fuel level 0-254
const uint32_t CAN_ID_ENG_HOURS      = 0x18FEE500; //Byte [0] LSB, Byte [1] MSB - Total Engine Hours

const uint32_t CAN_ID_ABS_SPEED      = 0x18FEBF0B; //Byte [0] LSB, Byte [1] MSB - Speed from ABS Controller mph x 405

const uint32_t CAN_ID_DISTANCE      = 0x18F51300; //Byte [0] LSB, Byte [3] MSB
const uint32_t CAN_ID_TRIP          = 0x18F91600; //Byte [0] LSB, Byte [3] MSB

//This is a packet sent to a remote display, running on a Waterproof M5Core2
const uint32_t CAN_ID_TEENSY         = 0x18FE66FE; //Message from Teensy Bridge (Battery main switch PGN, repurposed)

//Variables to hold corresponding values:
uint8_t nSHIFTER, nTHROTTLE, nHANDBRAKE, nFOOTBRAKE, nSEATBELT, nDIFFLOCK, n4WD;
uint8_t  nENG_TEMP, nFUEL_LEVEL, nENG_HOURS;

//Pins controlling Motor Inverter (which has limited CAN options)
const int SPEED_LIMIT = 16;
const int THROTTLE_MAP = 4;

uint32_t PID_List[100];  //List of incoming PIDs
int PID_Count[100];
int PID_ListPtr = 0;

String sSHIFTER, sSPEED, sRPM, sCEL, sFUEL_LEVEL, sENG_TEMP, s4WD, sDIFFLOCK, sTHROTTLE, sHANDBRAKE, sFOOTBRAKE, sSEATBELT, sMESSAGE; 
int nCURRENT, nPACKVOLTS, nSOC, nSOH, nHIGHTEMP, nCCL, nDCL, nECU_Speed, nOdoCtr;
byte nORrion_Flags, nPolaris_Flags;
double dSpeed,dLast_LAT, dLast_LNG;
long nRPM_Substitute, nSpeed_Substitute, nTemp_Substitute, nFuel_Substitute, lDistance, lEEPROM_Distance, lEEPROM_Distance_Last;

unsigned long start_time;
long lDispTimer = millis();
long lSendTimer = millis();
long lSendFast = millis();
long lSpeedTimer = millis();
long lSendOrion = millis();
long nLastDist = 0xFFFF; 

static inline void print_hex(uint8_t *data, int len) {
  char temp[10];
  for (int b=0;b < len; b++) {
    sprintf(temp, "%.2x",data[b]);
    Serial.print(temp);
  }
  Serial.println("");
}

void got_ecu_frame(CANFrame &frame, uint8_t which_interface) {
  //Extract some bits of data from ECU messages
  uint32_t lPID = (frame.get_id() & 0x00FFFF00);
  unsigned int Value, ValueA, ValueB;  

  if(lPID==(CAN_ID_SHIFTER & 0x00FFFF00)) {
    Value = frame.buf[5];
    if (Value == 0x4C) {
      sSHIFTER = "L";
    }
    if (Value == 0x48) {
      sSHIFTER = "H";
    }
    if (Value == 0x4e) {
      sSHIFTER = "N";
    }
    if (Value == 0x52) {
      sSHIFTER = "R";
    }
    if (Value == 0x50) {
      sSHIFTER = "P";
    }
  }
  if(lPID==(CAN_ID_HANDBRAKE & 0x00FFFF00)) {
    Value = frame.buf[0];
    static bool lastHandbrakeState = false;
    
    if (Value == 0x37) {
      sHANDBRAKE = "APP"; 
      nPolaris_Flags = nPolaris_Flags | 0B00100000; //Set Flag
      
      // Emergency fault reset: Single handbrake application
      if(!lastHandbrakeState) { // Just applied handbrake
        Serial.println("EMERGENCY BMS FAULT RESET TRIGGERED!");
        RequestOrionData("F"); // Clear faults immediately
      }
      lastHandbrakeState = true;
      
    } else {
      sHANDBRAKE = "REL";
      nPolaris_Flags = nPolaris_Flags & 0B11011111; //Reset Flag
      lastHandbrakeState = false;
    }
  }
  
  if(lPID==(CAN_ID_SEATBELT & 0x00FFFF00)) {
    Value = frame.buf[1];
    if (Value == 0xF7) {
      sSEATBELT = "SEC";
      nPolaris_Flags = nPolaris_Flags | 0B00010000; //Set Flag
    } else if(Value == 0xF3) {
      sSEATBELT = "REL";
      nPolaris_Flags = nPolaris_Flags & 0B11101111; //Reset Flag
    } else sSEATBELT = String(Value,HEX);
  }
  
  if(lPID==(CAN_ID_FOOTBRAKE & 0x00FFFF00)) {
    Value = frame.buf[3];
    if (Value == 0xDF) {
      sFOOTBRAKE = "APP";
    } else if(Value == 0xCF) {
      sFOOTBRAKE = "REL"; 
    }
  }
}

void got_ev_frame(CANFrame &frame, uint8_t which_interface) {
  //Extract some bits of data from EV Bus messages
  uint32_t lPID = (frame.get_id() & 0x00FFFF00);
  unsigned int Value, ValueA, ValueB;  
  
  //##### ORION BMS DATA #####
  if(frame.get_id() == 0x7EB) {
    //This is a frame from the Orion
    //Match byte 5 to the function
    if(frame.buf[3] == 0x15) {  //Current
      ValueA = frame.buf[4];
      ValueB = frame.buf[5];
      nCURRENT = ((((ValueA*256)+ValueB)-32767.0)/10.0)*-1; 
    }
    if(frame.buf[3] == 0x0D) {  //Voltage
      ValueA = frame.buf[4];
      ValueB = frame.buf[5];
      nPACKVOLTS = ((ValueA*256)+ValueB)/10.0;   
    }
    if(frame.buf[3] == 0x0F) {  //SOC
      ValueA = frame.buf[4];
      ValueB = frame.buf[5];
      nSOC = ValueA/2;    
    }
    if(frame.buf[3] == 0x13) {  //SOH
      ValueA = frame.buf[4];
      ValueB = frame.buf[5];
      nSOH = ValueA;    
    }
    if(frame.buf[3] == 0x28) {  //Temperature
      ValueA = frame.buf[4];
      ValueB = frame.buf[5];
      nHIGHTEMP = ValueA;   
    } 
    if(frame.buf[3] == 0x0A) {  //CCL Charge Limit
      ValueA = frame.buf[4];
      ValueB = frame.buf[5];
      nCCL = ValueA*256+ValueB;
    } 
    if(frame.buf[3] == 0x0B) {  //DCL Discharge Limit
      ValueA = frame.buf[4];
      ValueB = frame.buf[5];
      nDCL = ValueA*256+ValueB;   
    } 
    if(frame.buf[3] == 0x04) {  //Flags
      ValueA = frame.buf[4];
      ValueB = frame.buf[5];
      nORrion_Flags = ValueB;   
    } 
  }
  //###### HYPER9 DATA ######
  if(frame.get_id()==0x401) {
    ValueA = frame.buf[3];
    ValueB = frame.buf[2];
    float dRPM = ValueA*256 + ValueB;
    //Turn this into mph
    if(sSHIFTER == "H") {
      //Wheel dia 673mm
      dSpeed = dRPM * 0.0788 / RPM_SCALE_HIGH; 
    } else if(sSHIFTER == "L" || sSHIFTER == "R") {
      dSpeed = dRPM * 0.788 / RPM_SCALE_LOW;
    } else if(sSHIFTER == "N") {
      //Display RPM/100
      dSpeed = dRPM / 100;  
    }
  } 
}

void vehicle_got_frame(const CAN_message_t &orig_frame) {
  CANFrame frame = orig_frame;

  frame.seq = 1; 

  if(((frame.get_id() & 0x00FFFF00)==(CAN_ID_ABS_SPEED & 0x00FFFF00))) {
    //This is the Speed sent by the ABS Controller
    nSpeed_Substitute = (frame.buf[1]*256 + frame.buf[0]);
  }

  //Everything from the Vehicle needs to be passed to the ECU
  canMutex.lock();
  CAN_ECU.write(frame);
  canMutex.unlock();
}

void ev_got_frame(const CAN_message_t &orig_frame) {
  CANFrame frame = orig_frame;

  frame.seq = 1; 

  got_ev_frame(frame, orig_frame.bus);
}

void ecu_got_frame(const CAN_message_t &orig_frame) {
  //Frame has arrived from the ECU
  CANFrame frame = orig_frame;

  frame.seq = 1; 

  //Some messages need to be passed on, others processed.
  uint32_t lPID = (frame.get_id() & 0x00FFFF00);

  //These are the frames to block & substitute with values from EV CANBUS / GPS
  if(lPID==(CAN_ID_SPEED & 0x00FFFF00)) {
    //Substitute speed values & send.  Values updated in Loop.
    frame.buf[1] = byte(int(nSpeed_Substitute) & 0x00FF);  //mph * 405
    frame.buf[2] = byte(int(int(nSpeed_Substitute) & 0xFF00)/256);    
  }
  if(lPID==(CAN_ID_RPM & 0x00FFFF00)) {
    frame.buf[0] = byte(nRPM_Substitute & 0x00FF);  //1000 rpm
    frame.buf[1] = byte(int(nRPM_Substitute & 0xFF00)/256);  

    //Add a couple of extra bytes of info for the TFT Display
    frame.buf[4] = nORrion_Flags;
    frame.buf[5] = nPolaris_Flags;
    frame.buf[6] = nSOC;
    frame.buf[7] = nSOH;
  }
  if(lPID==(CAN_ID_ENG_TEMP & 0x00FFFF00)) {
    frame.buf[0] = byte(nTemp_Substitute);  //Byte [0] - Engine Temperature (Degrees C + 40)    
  }
  if(lPID==(CAN_ID_FUEL_LEVEL & 0x00FFFF00)) {
    frame.buf[1] = byte(nFuel_Substitute);  //SOC o to 100.  Byte [1] - Fuel level 0-240    
  }
  
  //A couple of PIDs just need blocking
  if((lPID!=(0x1CECFF00 & 0x00FFFF00)) && (lPID!=(CAN_ID_OILPRESSURE & 0x00FFFF00)) && (lPID!=(CAN_ID_DISTANCE & 0x00FFFF00)))  {
    //Pass on everything else
    canMutex.lock();
    CAN_Vehicle.write(frame);
    canMutex.unlock();

    //A subset need data extracting
    if((lPID==(CAN_ID_SHIFTER & 0x00FFFF00)) || (lPID==(CAN_ID_FOOTBRAKE & 0x00FFFF00)) || (lPID==(CAN_ID_SEATBELT & 0x00FFFF00)) || (lPID==(CAN_ID_HANDBRAKE & 0x00FFFF00))) {
      got_ecu_frame(frame, orig_frame.bus);
    }
  }
}

// RTOS Thread Functions
void vehicleReceiveThread() {
  while (1) {
    CAN_message_t msg;
    if (CAN_Vehicle.read(msg)) {
      vehicle_got_frame(msg);
    }
    threads.yield();
  }
}

void evReceiveThread() {
  while (1) {
    CAN_message_t msg;
    if (CAN_EV.read(msg)) {
      ev_got_frame(msg);
    }
    threads.yield();
  }
}

void ecuReceiveThread() {
  while (1) {
    CAN_message_t msg;
    if (CAN_ECU.read(msg)) {
      ecu_got_frame(msg);
    }
    threads.yield();
  }
}

void transmitThread() {
  while (1) {
    //Request Orion data every 1050mS
    if(millis()-lSendOrion > 1050) { 
      lSendOrion = millis();
      RequestOrionData(""); //Request all data
    }
    
    //Send signal to disable the CEL dash every 200ms
    if(millis()-lSendFast > 200) { 
      lSendFast = millis();

      CAN_message_t SendMessage;
      SendMessage.len = 8;
      SendMessage.flags.extended = true;
      
      SendMessage.buf[0] = 0b00000000;
      SendMessage.buf[1] = 0b00000000;
      SendMessage.buf[2] = 0b00000000;
      SendMessage.buf[3] = 0b00000000;

      if(nHIGHTEMP>60) SendMessage.buf[0] = SendMessage.buf[0] + 1;  //Set temp warning if over 60C 
      
      bool bTriangle = false;
      if(nSOC==0) nSOC = 50; //nSOC has not been set yet, don't limit anything based on SOC until it has.
      
      if((sSHIFTER == "H") && (nSOC > 30)) {
        bTriangle = false;  //Set Triangle speed warning
      } else if((sSHIFTER == "L") && (nSOC > 30)) {
        bTriangle = false;  //Set Triangle speed warning
      } else if(((sSHIFTER == "L") || (sSHIFTER == "H")) && (nSOC <= 30) && (nSOC > 20)) {
        bTriangle = false;  //Set Triangle speed warning
      } else {
        bTriangle = true;  //Set Triangle speed warning
      }

      if(sHANDBRAKE == "APP") {
        bTriangle = true;  //Set Triangle speed warning if handbrake applied
      }

      if(bTriangle) SendMessage.buf[0] = SendMessage.buf[0] + 4;  //Set Triangle warning if speed limited 

      // Set warnings based on Orion Flags
      if((nORrion_Flags & 0b00001000) == 0b00001000) SendMessage.buf[0] = SendMessage.buf[0] +128;  //Set Engine warning if BMS Errors present
      
      SendMessage.id = CAN_ID_CEL;
      canMutex.lock();
      CAN_Vehicle.write(SendMessage);
      canMutex.unlock();
    }
    threads.yield();
  }
}

void mainProcessingThread() {
  while (1) {
    if(millis()-lDispTimer > 30000) {
      lDispTimer = millis();
      // Debug output commented out - uncomment if needed
      //Serial.print("Gear="); Serial.print(sSHIFTER);
      //Serial.print("|Speed="); Serial.print(sSPEED);
      //etc...
    }

    //Several things affect throttle map and speed limit.  I hope this logic is sound!
    //Throttle map just reduces the peak torque / current to 50%
    if(nSOC==0) nSOC = 50; //nSOC has not been set yet, don't limit anything based on SOC until it has.

    nPolaris_Flags = nPolaris_Flags & 0B00111111; //Clear the top two bits only
    if((sSHIFTER == "H") && (nSOC > 30)) {
      digitalWrite(THROTTLE_MAP, LOW); //Change the throttle curve to be more aggressive  
      nPolaris_Flags = nPolaris_Flags & 0B10111111; //Reset Flag
      digitalWrite(SPEED_LIMIT, LOW); //Don't limit the speed
      nPolaris_Flags = nPolaris_Flags & 0B01111111; //Reset Flag
    } else if((sSHIFTER == "L") && (nSOC > 30)) {
      digitalWrite(THROTTLE_MAP, HIGH); //Change the throttle curve to be more gentle  
      nPolaris_Flags = nPolaris_Flags | 0B0100000; //Set Flag
      digitalWrite(SPEED_LIMIT, LOW); //Don't limit the speed
      nPolaris_Flags = nPolaris_Flags & 0B01111111; //Reset Flag
    } else if(((sSHIFTER == "L") || (sSHIFTER == "H")) && (nSOC <= 30) && (nSOC > 20)) {
      digitalWrite(THROTTLE_MAP, HIGH); //Change the throttle curve to be more gentle  
      nPolaris_Flags = nPolaris_Flags | 0B0100000; //Set Flag
      digitalWrite(SPEED_LIMIT, LOW); //Don't limit the speed
      nPolaris_Flags = nPolaris_Flags & 0B01111111; //Reset Flag
    } else {
      digitalWrite(THROTTLE_MAP, HIGH); //Change the throttle curve to be more gentle  
      nPolaris_Flags = nPolaris_Flags & 0B10111111; //Reset Flag
      digitalWrite(SPEED_LIMIT, HIGH); //limit the speed to 10mph
      nPolaris_Flags = nPolaris_Flags | 0B10000000; //Set Flag
    }

    if(sHANDBRAKE == "APP") {
      //If handbrake applied, limit torque & speed regardless
      digitalWrite(THROTTLE_MAP, HIGH); //Change the throttle curve to be more gentle  
      nPolaris_Flags = nPolaris_Flags | 0B0100000; //Set Flag
      digitalWrite(SPEED_LIMIT, HIGH); //limit the speed to 10mph
      nPolaris_Flags = nPolaris_Flags & 0B10111111; //Reset Flag
    }

    //###### Set substitute variables for Polaris Dash ######
    //RPM
    //RPM Substituted with Power
    nRPM_Substitute = nCURRENT * nPACKVOLTS / 4.4 +705; //Value 1000 to 10,000 for 0 to 40kW

/*
    //SPEED Only send if signal quality good and speed >=2mph
    if((gps.hdop.value() < 160)) {
      nSpeed_Substitute = gps.speed.mph() * 405.0;

      //Update the odometer & trip meters
      //Store initial values
      if(dLast_LAT == 0) dLast_LAT = gps.location.lat();
      if(dLast_LNG == 0) dLast_LNG = gps.location.lng();

      //Calculate distance between last & current fixes
      lDistance = gps.distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        dLast_LAT,
        dLast_LNG);

      if(lEEPROM_Distance==0) { //Read from EEPROM
        lEEPROM_Distance = EEPROM.read(3)<<24 + EEPROM.read(2)<<16 + EEPROM.read(1)<<8 + EEPROM.read(0); 
      }

      if(nSOC>93) {  //Fully charged, save distance travelled since last full charge & reset EEPROM
          EEPROM.write(4, (lEEPROM_Distance & 0x000000FF)); //LSB; 
          EEPROM.write(5, (lEEPROM_Distance & 0x0000FF00)>>8); //MSB; 
          EEPROM.write(0, 0); 
          EEPROM.write(1, 0); 
          EEPROM.write(2, 0); 
          EEPROM.write(3, 0); 
          lEEPROM_Distance = 0;
      }
      
      if(lDistance > 10) { //We have travelled at least 10m
        dLast_LAT = gps.location.lat();
        dLast_LNG = gps.location.lng();
        lEEPROM_Distance = lEEPROM_Distance + lDistance;
        
        if(lEEPROM_Distance_Last - lEEPROM_Distance > 10) {
          //Write new distance to EEPROM.  Need to limit writes to EEPROM, so only update every 500m (should last 1,000 km)
          lEEPROM_Distance_Last = lEEPROM_Distance;
          EEPROM.write(0, ((lEEPROM_Distance/5) & 0x000000FF)); //LSB
          EEPROM.write(1, ((lEEPROM_Distance/5) & 0x0000FF00)>>8);
          EEPROM.write(2, ((lEEPROM_Distance/5) & 0x00FF0000)>>16);
          EEPROM.write(3, ((lEEPROM_Distance/5) & 0xFF000000)>>24); //MSB
          EEPROM.write(6, (95-nSOC)); //Charge used for this distance 
        }
      }  
    } else {
      //nSpeed_Substitute = lEEPROM_Distance / 1609; //Get the distance travelled in miles since the last full charge
      //nSpeed_Substitute = nECU_Speed;  //Use speed value from ECU if GPS not available
    }
*/  
    
    //ENG_TEMP
    nTemp_Substitute = byte(nHIGHTEMP+40);  //Byte [0] - Engine Temperature (Degrees C + 40)
    
    //FUEL_LEVEL
    nFuel_Substitute = int(float(nSOC-10) * 3.0);
    if(nSOC <20) nFuel_Substitute=0; //Show tank empty is below 20%
    if(nFuel_Substitute>250) nFuel_Substitute=250;
    if(nFuel_Substitute<0) nFuel_Substitute=0;

    threads.yield();
  }
}

void RequestOrionData(String sDataType) {
  //To get data from the Orion BMS, you have to ask for it!
  //Data is requested only after a packet is received from the ECU - to limit the data volume

  CAN_message_t SendMessage;

  //Request data from Orion maximum every 1000ms
  //Request more rapidly for the first 10 sec after startup, to populate the gauges quickly
  if((millis()-lSendTimer > 1000) || ((millis()-start_time <10000) && (millis()-lSendTimer > 200))) { 
    lSendTimer = millis();
    //Send a SYNC Request to CANOPEN Nodes
    SendMessage.id = 0x0080;
    canMutex.lock();
    CAN_EV.write(SendMessage);
    canMutex.unlock();

    //Request data from Orion

    //Message to clear fault codes. Only called on startup
    if(sDataType=="F") {
      // Clear Diagnostic Information (Service 0x14) - Standard UDS/OBD2 approach
      SendMessage.id = 0x07E3;
      SendMessage.len = 8;
      SendMessage.buf[0] = 0x04;  //Length 4 Bytes
      SendMessage.buf[1] = 0x14;  //Clear Diagnostic Information service
      SendMessage.buf[2] = 0xFF;  //Clear all DTCs (Group of DTC = 0xFFFFFF)
      SendMessage.buf[3] = 0xFF;  //Clear all DTCs
      SendMessage.buf[4] = 0xFF;  //Clear all DTCs
      SendMessage.buf[5] = 0x00;  //Padding
      SendMessage.buf[6] = 0x00;  //Padding
      SendMessage.buf[7] = 0x00;  //Padding
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
      
      delay(50); // Give BMS time to process
      
      // Alternative approach - Clear all fault flags using Control Service
      SendMessage.buf[0] = 0x02;  //Length 2 Bytes
      SendMessage.buf[1] = 0x31;  //Routine Control Service
      SendMessage.buf[2] = 0x01;  //Start routine
      SendMessage.buf[3] = 0x00;  //Padding
      SendMessage.buf[4] = 0x00;  //Padding
      SendMessage.buf[5] = 0x00;  //Padding
      SendMessage.buf[6] = 0x00;  //Padding
      SendMessage.buf[7] = 0x00;  //Padding
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    }
    
    //This is effectively the header, common to all requests
    SendMessage.id = 0x07E3;
    SendMessage.buf[0] = 0x03;  //Length 3 Bytes
    SendMessage.buf[1] = 0x22;  //Request Data Mode

    //Status
    if(sDataType=="L" || sDataType=="") {
      SendMessage.buf[2] = 0xF0;  //PID
      SendMessage.buf[3] = 0x04;  //PID
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    }

    //Current
    if(sDataType=="C" || sDataType=="") {
      SendMessage.buf[2] = 0xF0;  //PID
      SendMessage.buf[3] = 0x15;  //PID
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    }
    
    //Voltage
    if(sDataType=="V" || sDataType=="") {
      SendMessage.buf[2] = 0xF0;  //PID
      SendMessage.buf[3] = 0x0D;  //PID
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    }
    
    //SOC
    if(sDataType=="S" || sDataType=="") {
      SendMessage.buf[2] = 0xF0;  //PID
      SendMessage.buf[3] = 0x0F;  //PID
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    }

    //SOH
    if(sDataType=="H" || sDataType=="") {
      SendMessage.buf[2] = 0xF0;  //PID
      SendMessage.buf[3] = 0x13;  //PID
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    }
    
    //Temperature
    if(sDataType=="T" || sDataType=="") {
      SendMessage.buf[2] = 0xF0;  //PID
      SendMessage.buf[3] = 0x28;  //PID
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    } 

    //CCL
    if(sDataType=="CCL" || sDataType=="") {
      SendMessage.buf[2] = 0xF0;  //PID
      SendMessage.buf[3] = 0x0A;  //PID
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    }
       
    //DCL
    if(sDataType=="DCL" || sDataType=="") {
      SendMessage.buf[2] = 0xF0;  //PID
      SendMessage.buf[3] = 0x0B;  //PID
      canMutex.lock();
      CAN_EV.write(SendMessage);
      canMutex.unlock();
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("3 Port CAN bridge - Polaris General (RTOS Version)");
  start_time=millis();
  
  Serial3.begin(9600);

  pinMode(SPEED_LIMIT, OUTPUT);  //I've accidentally shared the Serial3 Tx pin
  pinMode(THROTTLE_MAP, OUTPUT);
  digitalWrite(SPEED_LIMIT, LOW);
  digitalWrite(THROTTLE_MAP, HIGH);  //HIGH for gentle response, LOW for aggressive

  //Teensy FlexCAN_T4 setup
  CAN_Vehicle.begin();
  CAN_Vehicle.setBaudRate(250000);
  CAN_Vehicle.setMaxMB(32);
  CAN_Vehicle.enableFIFO();

  CAN_EV.begin();
  CAN_EV.setBaudRate(250000);
  CAN_EV.setMaxMB(32);
  CAN_EV.enableFIFO();

  CAN_ECU.begin();
  CAN_ECU.setBaudRate(250000);
  CAN_ECU.setMaxMB(32);
  CAN_ECU.enableFIFO();

  CAN_Vehicle.enableFIFOInterrupt();
  CAN_Vehicle.mailboxStatus();
  CAN_EV.enableFIFOInterrupt();
  CAN_EV.mailboxStatus();
  CAN_ECU.enableFIFOInterrupt();
  CAN_ECU.mailboxStatus();

  //Clear fault codes multiple times on startup with delays to ensure BMS is ready
  Serial.println("Clearing BMS fault codes...");
  delay(1000); // Give BMS time to boot up
  for(int i = 0; i < 3; i++) {
    RequestOrionData("F"); //Clear BMS Fault codes
    delay(500); // Wait between attempts
    Serial.print("Fault clear attempt ");
    Serial.println(i + 1);
  }
  Serial.println("Fault clearing complete");

  // Start RTOS threads
  threads.addThread(vehicleReceiveThread);
  threads.addThread(evReceiveThread);
  threads.addThread(ecuReceiveThread);
  threads.addThread(transmitThread);
  threads.addThread(mainProcessingThread);
}

void loop(){
  //Loop is now used only for threads processing - main logic moved to threads
  threads.yield();

  //Keep all the commented code for future reference:
/*
  //Loop is used to send data periodically
  CAN_message_t SendMessage;
  int nCalcValue = 0;
  char cRead;
  bool bTriangle = false; //Show the triangle dash warning

  CAN_Vehicle.events();  //Process can events
  CAN_ECU.events();  //Process can events
  CAN_EV.events();  //Process can events
*/
}

//Other bits of potentially useful data - not used in this case
/*
  if(lPID==(CAN_ID_SPEED & 0x00FFFF00)) { 
    //Serial.println("Speed");
    Value = int(frame.buf[1])*256 + int(frame.buf[0]);
    Value=Value/500; //Speed scale factor
    //sSPEED = String(Value); 
    bBlockFrame=true; //Substitute this value
  }

  if(lPID==(CAN_ID_RPM & 0x00FFFF00)) {
    //Serial.println("RPM");
    Value = int(frame.buf[1])*256 + int(frame.buf[0]);
    sRPM = String(Value); 
    bBlockFrame=true; //Substitute this value
  }
  //if(lPID==(CAN_ID_CEL & 0x00FFFF00)) bBlockFrame=true;
  //if(lPID==(0x18FF2200 & 0x00FFFF00)) bBlockFrame=true;
  //if(lPID==(0x0CF00400 & 0x00FFFF00)) bBlockFrame=true;
  if(lPID==(0x1CECFF00 & 0x00FFFF00)) bBlockFrame=true;

  //Possible
  if(lPID==(CAN_ID_OILPRESSURE & 0x00FFFF00)) bBlockFrame=true;
  
  if(lPID==(CAN_ID_FUEL_LEVEL & 0x00FFFF00)) {
    //Serial.println("FUEL");
    Value = frame.buf[1]; //0-254
    sFUEL_LEVEL = String(Value); 
  }
    
  if(lPID==(CAN_ID_ENG_TEMP & 0x00FFFF00)) {
    //Serial.println("ENG TEMP");
    Value = frame.buf[0]; //Temp in C + 40C
    sENG_TEMP = String(Value-40); 
  }
  if(lPID==(CAN_ID_4WD & 0x00FFFF00)) {
    //Serial.println("4WD");
    Value = frame.buf[0];
    if (Value == 0xFD) {
      s4WD = "4WD"; 
    } else if(Value == 0xFC) {
      s4WD = "2WD"; 
    } else {
      s4WD = String(Value, HEX);  //Unknown
    }
  }
  if(lPID==(CAN_ID_DIFFLOCK & 0x00FFFF00)) {
    //Serial.println("Difflock");
    Value = frame.buf[1];
    if(Value == 0xDF) {
      sDIFFLOCK = "LOCK"; 
    } else if(Value == 0xCF) {
      sDIFFLOCK = "OPEN";
    }
  }

  if(lPID==(CAN_ID_THROTTLE & 0x00FFFF00)) {
    //Serial.println("Throttle");
    Value = frame.buf[6];  //Range 25 to 250
    sTHROTTLE = String(Value); 
  }

  long calcMicrosecODO(float speedKMH){
  long uS;
  float freq;
  float speedMPH;
   
  Serial.print("Speed = ");
  Serial.print(speedKMH/100);
  Serial.println(" km/h");
  speedMPH = speedKMH / 160.934;
  // Required frequency for timer 1 ISR
  //  1.15 is 4140 (Pulse per Mile) / 3600 (1hr in seconds)
  //  0.7146 is 2572.5 (pulse per KM) / 3600
  freq = speedMPH * 1.15; 
  Serial.print("Freq = ");
  Serial.print(freq);
  Serial.println(" Hz");
  uS = 1000000/freq;
  if(uS < 4500000 && uS > 0){
    return (uS);}
  else {
    return (4500000);
  }
*/
