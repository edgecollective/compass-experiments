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

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

#define interval_sec 30
 
#include <SPI.h>

#include "Adafruit_EPD.h"
#define EPD_CS     7
#define EPD_DC     9
#define SRAM_CS     10
#define EPD_RESET   11 // can set to -1 and share with microcontroller Reset!
#define EPD_BUSY    -1 // can set to -1 to not use a pin (will wait a fixed delay)

Adafruit_IL0373 display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY);

#define COLOR1 EPD_BLACK
#define COLOR2 EPD_RED

int x = 10; // x position of the marker
int y = 10; // y "
 
int max_x = 150; // width of the gridsquare in pixels on the display
int max_y = 100; // height "

float xfrac=0; // fractional position (of max_x) of the marker on the gridsquare
float yfrac=0; // " (of max_y) "

char locator[9]; // gridsquare locator

float unit = 0.004166666666666667;

char * getLocator(float lat, float lon, int precision) {
  
  float ydiv_arr[] = {10,1,0.04166666666,0.00416666666,0.00017361111};
  char d1[]="ABCDEFGHIJKLMNOPQR";
  char d2[]="ABCDEFGHIJKLMNOPQRSTUVWX";

  int this_precision = 4;

  strcpy(locator,"");
  
  float q = lon;
  float p = lat;
  float rlon,rlat;
  char cstr[]="1";
  int num;
  
  while (q < -180) {q += 360;}
      while (q > 180) {q -=360;}
      q = q + 180;
      p = p + 90;
      
      strncat(locator,&d1[(int) floor(q/20)],1);
      
      strncat(locator, &d1[(int) floor(p/10)],1);
  
   
      for (int i=0; i<4; i=i+1) {
    if (this_precision > i+1) {
        rlon = fmod(q,ydiv_arr[i]*2);
        rlat = fmod(p,ydiv_arr[i]);
      if ((i%2)==0) {
        num = floor(rlon/(ydiv_arr[i+1]*2));
        itoa(num, cstr, 10);

        strcat(locator,cstr);
        strcat(locator,"");
        num = floor(rlat/(ydiv_arr[i+1]));
        itoa(num, cstr, 10);
        strcat(locator,cstr);

      } else {

        strncat(locator,&d2[(int) floor(rlon/(ydiv_arr[i+1]*2))],1);
        strcat(locator,"");
        strncat(locator,&d2[(int) floor(rlat/(ydiv_arr[i+1]))],1);
      }
    }
    }  
    return (locator);
}

void eink_setup() {
  display.begin();
  Serial.println("eink setup.");
}
void eink_write(char *text) {
  /// display
  Serial.println("eink test");
  //
  // large block of text
  display.clearBuffer();
  display.setCursor(0, 0);
  display.setTextColor(COLOR1);
  display.setTextWrap(true);
  display.print(text);
  display.display();
}

void display_map(float lat, float lon, uint8_t max_x, uint8_t max_y, uint8_t x, uint8_t y, char locator[]) {

  display.clearBuffer();

  //big frame
  display.drawLine(0,0,max_x,0,COLOR1);
  display.drawLine(0,max_y,max_x,max_y,COLOR1);

  //minor frame
  display.drawLine(0,round(max_y/2),max_x,round(max_y/2),COLOR2);
  display.drawLine(round(max_x/2),0,round(max_x/2),max_y,COLOR2);


  display.drawCircle(x, y, 6, COLOR1);
  display.fillCircle(x, y, 3, COLOR2);

  // info display
  //display.drawLine(90,max_y,90,150,COLOR2);

  int current_y = max_y;
  int step_y = 10;
  int current_x = 5;

  display.setTextWrap(false);

  display.setTextSize(2);
  current_y=current_y+5;
  display.setCursor(current_x, current_y);
  display.setTextColor(COLOR2);
  //display.print(" GS: ");
  display.print(locator);

  display.setTextSize(1);
  current_y=current_y+step_y*2;
  display.setCursor(current_x,current_y);
  display.setTextColor(COLOR1);
  display.print("lat: ");
  display.print(lat);
  
  current_y=current_y+step_y;
  display.setCursor(current_x,current_y);
  display.setTextColor(COLOR1);
  display.print("lon: ");
  display.print(lon);
  


  
  
  display.display();

  /*
  for (int16_t i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, COLOR1);
  }

  for (int16_t i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, COLOR2);  // on grayscale this will be mid-gray
  }
  */
  
}

void setup()
{

  Serial.begin(115200);
  while(!Serial) {
    ;
    
  }
  
  Serial.println("SparkFun u-blox Example");
  
  display.begin();
  display.clearBuffer();
  display.setCursor(0, 0);
  display.setTextColor(COLOR1);
  display.setTextWrap(true);
  display.print("boohaa");
  display.display();

  


  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
}

void loop()
{
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  if(nmea.isValid() == true)
  {
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();

    Serial.print("Latitude (deg): ");
    Serial.println(latitude_mdeg / 1000000., 6);
    Serial.print("Longitude (deg): ");
    Serial.println(longitude_mdeg / 1000000., 6);
    
    float flat = latitude_mdeg / 1000000.;
    float flon = longitude_mdeg / 1000000.;
    
    float myLat = flat;
  if (myLat > 85) myLat = 85;
  if (myLat < -85) myLat = -85;

  //find lat/lon bounds of the gridsquare
  float lon_left = floor(flon/(unit*2))*(unit*2);
  float lon_right = ceil(flon/(unit*2))*(unit*2);
  float lat_top = ceil(myLat/unit)*unit;
  float lat_bottom=floor(myLat/unit)*unit;

  float xfrac = (flon-lon_left)/(lon_right-lon_left);
  
  float yfrac = 1-(flat-lat_bottom)/(lat_top-lat_bottom);

  x=round(xfrac*max_x);
  y=round(yfrac*max_y);
    
  Serial.println(getLocator(flat,flon, 4));

  display_map(flat,flon,max_x,max_y,x,y,getLocator(flat,flon, 4));

  delay(interval_sec*1000);
  
  }
  else
  {
    Serial.print("No Fix - ");
    Serial.print("Num. satellites: ");
    Serial.println(nmea.getNumSatellites());
  }

  delay(250); //Don't pound too hard on the I2C bus
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
