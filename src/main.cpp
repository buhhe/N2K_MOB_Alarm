/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// NMEA2000 Man over board button--
// Version 1.0, 18.03.2023, buhhe (https://github.com/buhhe)
// https://github.com/buhhe/NMEA200-Man-Over-Board-Button


#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

#include <Arduino.h>
#include <Preferences.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>

// to be printed to USB-serial
const char Description[] = "MOB-Alarm Button. Comes without any warranty or committed reliability. Only for test purposes.";


#define ALARM_BUTTON 13     // GPIO pin to be connected to GND when the alarm button is pressed
#define MYMMSI  211794060     // set your MMSI in here

int NodeAddress;            // To store last Node Address
Preferences preferences;    // Nonvolatile storage on ESP32 - To store LastDeviceAddress

// structure which contains all data needed to set the mob alarm
struct PGN127233
{
  unsigned char                     SID;                      // sequenz id
  uint32_t                          EmitterID;                // Identifier for each MOB emitter, unique to the vessel
  tN2kMOBStatus                     MOBStatus;                // MOBStatus: MOBEmitterActivated=0, ManualOnBoardMOBButtonActivation=1, TestMode=2, MOBNotActive=3
  double                            ActivationTime;           // Time of day (UTC) when MOB was activated
  tN2kMOBPositionSource             PositionSource;           // Position Source: PositionEstimatedByVessel=0,PositionReportedByMOBEmitter=1 
  uint16_t                          DateOfMobPosition;        // Date of MOB position
  double                            TimeOfMobPosition;        // Time of day of MOB position (UTC)
  double                            LatitudeOfMob;            // Latitude in degrees
  double                            LongitudeOfMob;           // Longitude in degrees
  tN2kHeadingReference              COGReference;             // True or Magnetic: N2khr_true=0, N2khr_magnetic=1, N2khr_error=2, N2khr_Unavailable=3
  double                            COG;                      // Course Over Ground in radians
  double                            SOG;                      // Speed Over Ground in m/s
  uint32_t                          MMSI;                     // MMSI
  tN2kMOBEmitterBatteryStatus       MOBEmitterBatteryStatus;  // Battery status: Good=0, Low=1
} ;

PGN127233 PGNOut;

// Set the information for other bus devices, which messages we support
const unsigned long  ReceiveMessages[] PROGMEM = { 129029L, 129026L, 0};  // get navigational data
const unsigned long TransmitMessages[] PROGMEM = {127233L, 0};            // send man over board alarm

// forward declarations
void          SayHello(void);
bool          IsTimeToUpdate(unsigned long);
unsigned long InitNextUpdate(unsigned long, unsigned long);
void          SendN2kMOBAlarm(void);
void          CheckSourceAddressChange(void);
void          MyParsePGN129029(const tN2kMsg);
void          MyParsePGN129026(const tN2kMsg);
void          MyHandleNMEA2000Msg(const tN2kMsg &);
void          DispMessage();


//*****************************************************************************
void setup()
{
  uint8_t chipid[6];
  uint32_t id = 0;
  int i = 0, j = 0; 

 // DeviceAddress tempDeviceAddress;

  // Init USB serial port
  Serial.begin(115200);
  delay(400);

  SayHello();  // print some useful information to USB-serial

  /*   NMEA2000 initialisation section */
  NMEA2000.SetN2kCANMsgBufSize(8);
  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetN2kCANSendFrameBufSize(150);

  // Generate unique number from chip id
  esp_efuse_mac_get_default(chipid);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));

// set some static information 
  PGNOut.SID          = 0xff;
  PGNOut.EmitterID    = 1;
  PGNOut.MOBStatus    = ManualOnBoardMOBButtonActivation;   // MOBEmitterActivated=0,ManualOnBoardMOBButtonActivation=1, TestMode=2, MOBNotActive=3
  PGNOut.MMSI         = MYMMSI;
  PGNOut.MOBEmitterBatteryStatus = Good;



  // Set product information
  NMEA2000.SetProductInformation("1", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "MOB-Alarm Button",  // Manufacturer's Model ID
                                 "SW-Vers:  1.0 (2023-03-18)",  // Manufacturer's Software version code
                                 "Mod-Vers: 1.0 (2023-03-18)" // Manufacturer's Model version
                                );
  // Set device information
  NMEA2000.SetDeviceInformation(id, // Unique number. Use e.g. Serial number.
                                135, // Device function= Man Overboard detection/reporting. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                20,  // Device class=Safety Systems. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                               );

  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", 36);  // Read stored last NodeAddress, default 34
  preferences.end();
  Serial.printf("NodeAddress=%d\n", NodeAddress);

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.SetMsgHandler(MyHandleNMEA2000Msg);
  NMEA2000.Open();
  
  pinMode(ALARM_BUTTON, INPUT_PULLUP);  
}


//*****************************************************************************
void loop() 
{
  static unsigned long myTime;
  NMEA2000.ParseMessages();
  CheckSourceAddressChange();

  if (digitalRead(ALARM_BUTTON)==LOW)
  {
    myTime = millis();
    while ( digitalRead(ALARM_BUTTON) == LOW && (millis() < myTime + 5000))
    {}
    if (digitalRead(ALARM_BUTTON)==LOW)
    {
      SendN2kMOBAlarm();
    }
  }

  // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
  if ( Serial.available() ) 
  {
    Serial.read();
  }
}

//*****************************************************************************
void MyHandleNMEA2000Msg(const tN2kMsg &N2kMsg)   // callback function for incoming N2K messages
{
  switch (N2kMsg.PGN)
  {
    case 129029L: MyParsePGN129029(N2kMsg); 
                  break;
    case 129026L: MyParsePGN129026(N2kMsg); 
    default:      break;
  }
}


//********* retrieve some navigational data, PGN129029 *************************************
void MyParsePGN129029(const tN2kMsg N2kMsg) 
{
  unsigned char   SID;
  double          Altitude;
  tN2kGNSStype    GNSStype;
  tN2kGNSSmethod  GNSSmethod;
  unsigned char   nSatellites;
  double          HDOP;
  double          PDOP;
  double          GeoidalSeparation;
  unsigned char   nReferenceStations;
  tN2kGNSStype    ReferenceStationType;
  uint16_t        ReferenceSationID;
  double          AgeOfCorrection;

  ParseN2kPGN129029(N2kMsg, SID, PGNOut.DateOfMobPosition, PGNOut.TimeOfMobPosition, PGNOut.LatitudeOfMob, PGNOut.LongitudeOfMob, Altitude, GNSStype, GNSSmethod,
                     nSatellites, HDOP, PDOP, GeoidalSeparation, nReferenceStations, ReferenceStationType, ReferenceSationID, AgeOfCorrection);

  PGNOut.ActivationTime = PGNOut.TimeOfMobPosition;                 // Time of day (UTC) when MOB was activated
  PGNOut.PositionSource = (tN2kMOBPositionSource)0;                 // Position Source
  PGNOut.MOBEmitterBatteryStatus = (tN2kMOBEmitterBatteryStatus)0;
}

//********* retrieve some navigational data, PGN129026 *************************************
void MyParsePGN129026(const tN2kMsg N2kMsg)
{
  unsigned char SID;
  ParseN2kPGN129026(N2kMsg, SID, PGNOut.COGReference, PGNOut.COG, PGNOut.SOG);
}

//********* send the man overt board alarm to the N2K bus, PGN127233  **********************

void SendN2kMOBAlarm(void)
{
  tN2kMsg N2kMsg;
  int i;
    Serial.println("!!!MOB Alarm!!!");
    
    SetN2kPGN127233(N2kMsg, 
                          PGNOut.SID,                       // Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages to link this PGN to other related PGNs. When no linkage exists, the value of the SID shall be set to 25   
                          PGNOut.EmitterID,                 // Identifier for each MOB emitter, unique to the vessel
                          PGNOut.MOBStatus,                 // MOBEmitterActivated=0,ManualOnBoardMOBButtonActivation=1, TestMode=2, MOBNotActive=3
                          PGNOut.ActivationTime,            // Time of day (UTC) when MOB was activated
                          PGNOut.PositionSource,            // Position Source
                          PGNOut.DateOfMobPosition,         // Date of MOB position
                          PGNOut.TimeOfMobPosition,         // time of day UTC, get from N2K  
                          PGNOut.LatitudeOfMob,             // Latitude in degrees
                          PGNOut.LongitudeOfMob,            // Longitude in degrees
                          PGNOut.COGReference,
                          PGNOut.COG,
                          PGNOut.SOG,
                          PGNOut.MMSI,
                          PGNOut.MOBEmitterBatteryStatus);
                                          
    NMEA2000.SendMsg(N2kMsg);
    delay(1000);

    Serial.println("MOB-Alarm sent!");
    
}

//*****************************************************************************
// Function to check if SourceAddress has changed (due to address conflict on bus)
void CheckSourceAddressChange()
{
  int SourceAddress = NMEA2000.GetN2kSource();

  if (SourceAddress != NodeAddress) { // Save potentially changed Source Address to NVS memory
    NodeAddress = SourceAddress;      // Set new Node Address (to save only once)
    preferences.begin("nvs", false);
    preferences.putInt("LastNodeAddress", SourceAddress);
    preferences.end();
    Serial.printf("Address Change: New Address=%d\n", SourceAddress);
  }
}

//*****************************************************************************
void SayHello()
{
  char Sketch[80], buf[256], Version[80];
  int i;

  // Get source code filename
  strcpy(buf, __FILE__);
  i = strlen(buf);

  // remove path and suffix
  while (buf[i] != '.' && i >= 0)
    i--;
  buf[i] = '\0';
  while ( buf[i] != '\\' && i >= 0)
    i--;
  i++;
  strcpy(Sketch, buf + i);

  // Sketch date/time of compliation
  sprintf(buf, "\nSketch: \"%s\", compiled %s, %s\n", Sketch, __DATE__, __TIME__);
  Serial.println(buf);
  Serial.println(Description);
}


void DispMessage()
{
  Serial.printf("\n\nSID:%d \nEmitterID:%d \nMOBStatus:%d \nActivationTime:%f \nPositionSource:%d \nDateOfMobPosition:%ld \nTimeOfMobPosition: %f \nLatitudeOfMob:%f \nLongitudeOfMob%f \nCOGReference:%d \nCOG: %.2f \nSOG:%.2f \nMMSI:%d \nMOBEmitterBatteryStatus%d\n",
                            PGNOut.SID,                       // Sequence ID. If your device is e.g. boat speed and heading at same time, you can set same SID for different messages to link this PGN to other related PGNs. When no linkage exists, the value of the SID shall be set to 25   
                            PGNOut.EmitterID,                 // Identifier for each MOB emitter, unique to the vessel
                            PGNOut.MOBStatus,                 // MOBEmitterActivated=0,ManualOnBoardMOBButtonActivation=1, TestMode=2, MOBNotActive=3
                            PGNOut.ActivationTime,            // Time of day (UTC) when MOB was activated
                            PGNOut.PositionSource,            // Position Source
                            PGNOut.DateOfMobPosition,         // Date of MOB position
                            PGNOut.TimeOfMobPosition,         // time of day UTC, get from N2K  
                            PGNOut.LatitudeOfMob,             // Latitude in degrees
                            PGNOut.LongitudeOfMob,            // Longitude in degrees
                            PGNOut.COGReference,
                            PGNOut.COG,
                            PGNOut.SOG,
                            PGNOut.MMSI,
                            PGNOut.MOBEmitterBatteryStatus);
}




