/*
  UBloxGPS.h

  UBlox GPS Library 
  Developed for the Manchester Space Program
  4/2014
  
  Additional Code by J Coxon (http://ukhas.org.uk/guides:falcom_fsa03)
*/

#include <SoftwareSerial.h>
#include <arduino.h>

class GGASentence
{
  public:
    float UTCTime;
    double Latitude;
    char NS;
    double Longitude;
    char EW;
    bool Fix;
    int NoSats;
    float HDOP;
    float MSLAlt;
    float GeoidSeparation;
    
    bool NewData;
};

class UBloxGPS {
  
  private:
    //Serial dSerial;
    SoftwareSerial *GPS;
    char gps_set_sucess;
    char GPSBuffer[128];
    int GPSBufferLen;
    
    bool parseGPGGA(char *Data);
    bool parseGPVTG(char *Data);
    bool parseGPGSV(char *Data);
    bool parseMNEAmsg(char* Buffer, int Len);
    
    void sendUBX(uint8_t *MSG, uint8_t len);
    bool getUBX_ACK(uint8_t *MSG);
    
  public:
    GGASentence Latest;
  
    UBloxGPS();
    bool Read();
    
};
