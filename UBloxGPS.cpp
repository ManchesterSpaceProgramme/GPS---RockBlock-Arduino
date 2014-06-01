/*
  UBloxGPS.cpp

  UBlox GPS Library 
  Developed for the Manchester Space Program
  4/2014
  
  Additional Code by J Coxon (http://ukhas.org.uk/guides:falcom_fsa03)
*/

#include "UBloxGPS.h"
#include <SoftwareSerial.h>

UBloxGPS::UBloxGPS() {
  
  gps_set_sucess = 0;
  GPSBufferLen = 0;
  Latest.NewData = false;
  
  GPS = new SoftwareSerial(8, 7);
  
  GPS->begin(9600); 
  Serial.println("Initialising....");
  //
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
  // THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
  //
  GPS->print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
  GPS->begin(4800);
  GPS->flush();
  
  //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  Serial.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
 
  // THE FOLLOWING COMMANDS DO WHAT THE $PUBX ONES DO BUT WITH CONFIRMATION
  // UNCOMMENT AS NEEDED
  
   Serial.println("Switching off NMEA GLL: ");
   uint8_t setGLL[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B                   };
   while(!gps_set_sucess)
   {		
   sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGLL);
   }
   gps_set_sucess=0;
   
   Serial.println("Switching off NMEA GSA: ");
   uint8_t setGSA[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32                   };
   while(!gps_set_sucess)
   {	
   sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGSA);
   }
   gps_set_sucess=0;
   
   /*Serial.println("Switching off NMEA GSV: ");
   uint8_t setGSV[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39                   };
   while(!gps_set_sucess)
   {
   sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGSV);
   }
   gps_set_sucess=0;*/
   
   Serial.print("Switching off NMEA RMC: ");
   uint8_t setRMC[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40                   };
   while(!gps_set_sucess)
   {
   sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setRMC);
   }
}

bool UBloxGPS::parseGPGGA(char *Data) {

  // find the positions of the 13 delimiting commas and turn them into string terminators
  char DataLen = strlen(Data);
  char ComPos[13];
  char cC = 0;
  for (int i=0; i<DataLen; ++i)
    if (Data[i] == ',') {
      ComPos[cC++] = i+1;
      Data[i] = '\0';
    }
  
  //GGASentence Latest;
  
  Latest.UTCTime = atof(Data);
  
  Serial.print("Time: ");
  Serial.println(Latest.UTCTime);
  
  float LatitudeMinutes = atof(&Data[ComPos[0]+2]);
  Data[ComPos[0]+2] = '\0';
  int LatitudeDegrees = atoi(&Data[ComPos[0]]);
  Latest.Latitude = LatitudeDegrees + LatitudeMinutes/60;
  Serial.print("Lat: ");
  Serial.println(Latest.Latitude, 8);
  
  Latest.NS = Data[ComPos[1]];
  Serial.print("NS: ");
  Serial.println(Latest.NS);
  
  float LongitudeMinutes = atof(&Data[ComPos[2]+3]);
  Data[ComPos[2]+3] = '\0';
  int LongitudeDegrees = atoi(&Data[ComPos[2]]);
  Latest.Longitude = LongitudeDegrees + LongitudeMinutes/60;
  Serial.print("Long: ");
  Serial.println(Latest.Longitude, 8);
  
  Latest.EW = Data[ComPos[3]];
  Serial.print("EW: ");
  Serial.println(Latest.EW);
  
  Latest.Fix = (Data[ComPos[4]] == '1');
  if (Latest.Fix)
    Serial.println("Fix: true");
  else
    Serial.println("Fix: false");
  
  Latest.NoSats = atoi(&Data[ComPos[5]]);
  Serial.print("No Sats: ");
  Serial.println(Latest.NoSats);
  
  Latest.HDOP = atof(&Data[ComPos[6]]);
  Serial.print("HDOP: ");
  Serial.println(Latest.HDOP);
  
  Latest.MSLAlt = atof(&Data[ComPos[7]]);
  Serial.print("MSL Alt: ");
  Serial.println(Latest.MSLAlt);
  
  Latest.GeoidSeparation = atof(&Data[ComPos[9]]);
  Serial.print("Geoid Separation: ");
  Serial.println(Latest.GeoidSeparation);
  
  Latest.NewData = true;
}

bool UBloxGPS::parseGPGSV(char *Data) {

  Serial.print("about to parse GPGSV sentence --[");
  Serial.print(Data);
  Serial.println("]--");
  
  // find the positions of the 13 delimiting commas and turn them into string terminators
  char DataLen = strlen(Data);
  char ComPos[13];
  char cC = 0;
  for (int i=0; i<DataLen; ++i)
    if (Data[i] == ',') {
      ComPos[cC++] = i+1;
      Data[i] = '\0';
    }



}

bool UBloxGPS::parseGPVTG(char *Data) {
  
  Serial.print("about to parse GPVTG sentence --[");
  Serial.print(Data);
  Serial.println("]--");
  
}

bool UBloxGPS::parseMNEAmsg(char* Buffer, int Len) {

  // remove $ from start of buffer
  Buffer = &Buffer[1];
  --Len;
  
  // split off checksum
  char CheckSumStr[3];
  CheckSumStr[0] = Buffer[Len-4];
  CheckSumStr[1] = Buffer[Len-3];
  CheckSumStr[2] = '\0';
  char CheckSum = strtoul(CheckSumStr, 0, 16);
  Len-=2;
  
  // reduce length of buffer and terminate it as a string
  Buffer[Len-=3] = '\0';
  
  // verify checksum
  char CalcCS = Buffer[0];
  for (int c=1;c<Len;++c)
    CalcCS^=Buffer[c];
  
  if (CheckSum != CalcCS) {
    Serial.print("checksum failed on --[");
    Serial.print(Buffer);
    Serial.println("]--");
    return false;
  } 
  
  if (strncmp(Buffer, "GPGGA",5) == 0)
    return parseGPGGA(&Buffer[6]);
  
  if (strncmp(Buffer, "GPVTG",5) == 0)
    return parseGPVTG(&Buffer[6]);
    
  if (strncmp(Buffer, "GPGSV",5) == 0)
    return parseGPGSV(&Buffer[6]);
  return false;
}

bool UBloxGPS::Read() {
  
  char data;
  //while(1)
  //{
    if(GPS->available())
    {
      // THIS IS THE MAIN LOOP JUST READS IN FROM THE GPS SERIAL AND ECHOS OUT TO THE ARDUINO SERIAL.
      data = GPS->read();
      Serial.write(data);
      
      if (GPSBufferLen<128)
        GPSBuffer[GPSBufferLen++] = data;
      
      // if the buffer ends in a carridge return and line feed split out and parse message
      if (GPSBufferLen>=2 && GPSBuffer[GPSBufferLen-2] == 13 && GPSBuffer[GPSBufferLen-1] == 10) {
        parseMNEAmsg(GPSBuffer, GPSBufferLen);
        
        GPSBufferLen = 0;
      }
    }
  //}
}

// Send a byte array of UBX protocol to the GPS
void UBloxGPS::sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    GPS->write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  GPS->println();
}
 
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
bool UBloxGPS::getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (GPS->available()) {
      b = GPS->read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}

