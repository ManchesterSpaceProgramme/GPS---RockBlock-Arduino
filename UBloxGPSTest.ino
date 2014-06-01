/*

*/
#include <IridiumSBD.h>
#include <SoftwareSerial.h>
#include "UBloxGPS.h"

// init RockBlock pins
//SoftwareSerial nss(10, 11); // (Rx, Tx)
//IridiumSBD isbd(nss, 9);
//static const int ledPin = 13;

// ptr for GPS object
UBloxGPS *GPS;
unsigned long LastGPSTime=0;

void setup()
{
  // setup GPS
  Serial.begin(9600);
  GPS = new UBloxGPS();
  
  // setup Rock Block
  //pinMode(ledPin, OUTPUT);

  //Serial.begin(115200);
  //nss.begin(19200);

  //isbd.attachConsole(Serial);
  //isbd.setPowerProfile(1);
  //isbd.begin();
}

struct RBMessage {
  char Type[10];
  char Fix;
  float Lat;
  float Long;
  float MSLAlt;
};

void CheckGPSandSend()
{
  //Serial.println("in the loop");
  
  RBMessage msg;
  
  int err;
  
  GPS->Read();
  
  //Serial.println("Read gps port!");
  
  /*if (GPS->Latest.NewData) {
    GPS->Latest.NewData = false;
    if (millis() > LastGPSTime + 30000) {
      LastGPSTime = millis();
      // There's a new GPS location and we haven't sent one for 30 seconds
      
      // format message for rock block
      if (GPS->Latest.Fix) {
        
        msg.Type[0] = 'P';
        msg.Type[1] = '0';
        msg.Type[2] = 's';
        msg.Type[3] = ' ';
        msg.Type[4] = 'F';
        msg.Type[5] = 'i';
        msg.Type[6] = 'x';
        msg.Type[7] = 0;
        "Pos Fix";
        
        float Lat = GPS->Latest.Latitude;
        if (GPS->Latest.EW == 'W')
          Lat = 0-Lat;
        float Long = GPS->Latest.Longitude;
        if (GPS->Latest.NS == 'S')
          Long = 0-Long;
          
        msg.Lat = Lat;
        msg.Long = Long;
        msg.MSLAlt = GPS->Latest.MSLAlt;
      } else {
        msg.Type[0] = 'N';
        msg.Type[1] = 'o';
        msg.Type[2] = 'F';
        msg.Type[3] = 'i';
        msg.Type[4] = 'x';
        msg.Type[5] = 0; 
      }
      
      Serial.println("About to send message.");
      
      // send message
      /*err = isbd.sendSBDBinary((const uint8_t*)&msg, sizeof(msg));
      if (err != 0)
        {
          Serial.print("#Debug# sendSBD failed: error ");
          Serial.println(err);
        } else {
        Serial.println("#Debug# successfully sentSBD message");
      }*/
    }
  }*/
}

void loop()
{
  CheckGPSandSend();
}

bool ISBDCallback()
{
  CheckGPSandSend();
  
//  digitalWrite(ledPin, (millis() / 200) % 2 == 1 ? HIGH : LOW);
//   return true;
}

