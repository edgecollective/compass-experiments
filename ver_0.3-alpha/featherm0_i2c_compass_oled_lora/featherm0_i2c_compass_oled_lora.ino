/*
  Read NMEA sentences over I2C using u-blox module SAM-M8Q, NEO-M8P, etc
  By: Nathan Seidle
  SparkFun Electronics
  Date: August 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads the NMEA characters over I2C and pipes them to MicroNMEA
  This example will output your current long/lat and satellites in view
 
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  For more MicroNMEA info see https://github.com/stevemarple/MicroNMEA

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  Go outside! Wait ~25 seconds and you should see your lat/long
*/

#include <Wire.h> //Needed for I2C to GNSS
#include <U8g2lib.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;
#include <LSM303.h>
#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#include <LSM303.h>
LSM303 compass;

#include <RHSoftwareSPI.h>  // http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.113.zip
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>

#define RF95_FREQ 915.0
#define gatewayNode 1

RHMesh *manager;

static const uint32_t GPSBaud = 9600;
typedef struct {
  float lat;
  float lon;
} Payload;

Payload theData;

// Radio pins for feather M0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LED 13

RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define waitTime 1000

float flat=0.;
float flon=0.;
float lat1;
float lon1;
float lat2;
float lon2;

float heading;
float bearing; 

#define r 22
#define mx 120-r
#define my 32

//end of conant driveway
//lat2 = 42.41152626206619;
//lon2 = -71.29830149978494;

void setup()
{
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

    
u8g2.begin();
u8g2.clearBuffer();
  u8g2.setFontDirection(0);
   u8g2.setFont(u8g2_font_6x10_tf); 
   
compass.init();
  compass.enableDefault();

   compass.m_min = (LSM303::vector<int16_t>){-1022, -829, -699};
  compass.m_max = (LSM303::vector<int16_t>){+378, +375, +320};

  manager = new RHMesh(rf95, gatewayNode);

   if (!manager->init()) {
    Serial.println(F("mesh init failed"));
    
  }
  rf95.setTxPower(23, false);
  rf95.setFrequency(915.0);
  rf95.setCADTimeout(500);

    // long range configuration requires for on-air time
  boolean longRange = false;
  if (longRange) {
    RH_RF95::ModemConfig modem_config = {
      0x78, // Reg 0x1D: BW=125kHz, Coding=4/8, Header=explicit
      0xC4, // Reg 0x1E: Spread=4096chips/symbol, CRC=enable
      0x08  // Reg 0x26: LowDataRate=On, Agc=Off.  0x0C is LowDataRate=ON, ACG=ON
    };
    rf95.setModemRegisters(&modem_config);
    if (!rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096)) {
      Serial.println(F("set config failed"));
    }
  }

  Serial.println("RF95 ready");
  
}

void loop()
{

  //update gps
  
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  if(nmea.isValid() == true)
  {
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();

    lat1 = latitude_mdeg / 1000000.;
    lon1 = longitude_mdeg / 1000000.;
    
    Serial.print("Latitude (deg): ");
    Serial.println(latitude_mdeg / 1000000., 6);
    Serial.print("Longitude (deg): ");
    Serial.println(longitude_mdeg / 1000000., 6);
  }
  else
  {
    Serial.print("No Fix - ");
    Serial.print("Num. satellites: ");
    Serial.println(nmea.getNumSatellites());
  }

//update compass
compass.read();
float heading = compass.heading();
Serial.println(heading);


//update display
u8g2.clearBuffer();
u8g2.setCursor(0,05);
u8g2.print("compass:");
u8g2.setCursor(0,15);
u8g2.print(round(heading));
u8g2.print("");

u8g2.drawCircle(mx,my,r);
   
int cdx = round(r*sin((heading+180)*PI/180.));
int cdy = round(r*cos((heading+180)*PI/180.));

u8g2.setCursor(mx+cdx, my+cdy);
u8g2.print("N");

float R = 6371000; // metres
float phi1 = lat1 * PI/180.; // φ, λ in radians
float phi2 = lat2 * PI/180.;
float lambda1 = lon1 * PI/180.;
float lambda2 = lon2 * PI/180.;
float y = sin(lambda2-lambda1)*cos(phi2);
float x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(lambda2-lambda1);
float theta = atan2(y,x);
bearing = fmod((theta*180/PI + 360),360);

float distance_meters = acos(sin(lat1*PI/180.)*sin(lat2*PI/180.) + cos(lat1*PI/180.)*cos(lat2*PI/180.)*cos(lon2*PI/180.-lon1*PI/180.) ) * 6371000;
float distance_feet = 3.281*distance_meters;

float degree_diff = heading-bearing;

u8g2.setCursor(0,55);
u8g2.print(round(distance_feet));
u8g2.print(" ft away");

int dx = round(r*sin((degree_diff+180)*PI/180.));
int dy = round(r*cos((degree_diff+180)*PI/180. ));

u8g2.drawLine(mx, my, mx+dx, my+dy);

u8g2.sendBuffer();

//radio
uint8_t buf[sizeof(Payload)];
  uint8_t len = sizeof(buf);
  uint8_t from;

if (manager->recvfromAckTimeout((uint8_t *)buf, &len, waitTime, &from)) {  // this runs until we receive some message
      // entering this block means the message is for us

    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(10);                       // wait for a second
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(10);                       // wait for a second
    
     // the rest of this code only runs if we were the intended recipient; which means we're the gateway
      theData = *(Payload*)buf;

      lat2 = theData.lat;
      lon2 = theData.lon;
}




//delay(300); //Don't pound too hard on the I2C bus


}



//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}
