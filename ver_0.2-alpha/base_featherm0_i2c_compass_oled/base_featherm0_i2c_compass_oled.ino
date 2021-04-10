#include <LSM303.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <math.h>
#include <Arduino.h>

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));


#include <RHSoftwareSPI.h>  // http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.113.zip
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>

#define RF95_FREQ 915.0
#define gatewayNode 1

RHMesh *manager;

#include <U8g2lib.h>

#define r 22
#define mx 120-r
#define my 32

#define interval_sec 10

long waitTime = 10*1000;

//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

LSM303 compass;

float flat=0.;
float flon=0.;


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

//conant

//float lat2 = 42.4115194014139;
//float lon2 = -71.298282872227;

//boston
//float lat2 = 42.358306550939965;
//float lon2 = -71.06660358413588;

//conant and weston
//float lat2 = 42.41255203831599;
//float lon2 = -71.29994755724634;


//due north
//float lat2 = 42.41293317045196;
//float lon2 = -71.29791687460762;

//north east
//float lat2 = 42.41359384882139;
//float lon2 = -71.29661415361875;

//end of conant driveway
float lat2 = 42.41152626206619;
float lon2 = -71.29830149978494;

void setup(void) {

 
  
  
  Wire.begin();

 Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  
  Serial.begin(115200);
u8g2.begin();
  u8g2.clearBuffer();
  
  u8g2.setFontDirection(0);
   u8g2.setFont(u8g2_font_6x10_tf); 

   u8g2.setCursor(0,0); 
   u8g2.print("Starting...");
   u8g2.sendBuffer();
  
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
  
u8g2.clearBuffer();
  
}

void loop(void) {

// update gps
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

}

compass.read();
float current_heading = compass.heading();

u8g2.clearBuffer();
u8g2.setCursor(0,45);
u8g2.print("compass:");
u8g2.setCursor(0,55);
u8g2.print(round(current_heading));
u8g2.print("");
u8g2.sendBuffer();


u8g2.drawCircle(mx,my,r);
   
//u8g2.drawLine(mx, my, mx+dx, my+dy);

//u8g2.drawDisc(mx+dx,my+dy,2);

int cdx = round(r*cos((current_heading-90)*PI/180.));
int cdy = round(r*sin((current_heading-90)*PI/180.));

u8g2.setCursor(mx+cdx, my+cdy);
u8g2.print("N");

u8g2.sendBuffer();

   
delay(300);
}
