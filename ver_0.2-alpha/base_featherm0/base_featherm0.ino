#include <LSM303.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <TinyGPS.h>
#include <math.h>
#include <Arduino.h>

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

TinyGPS gps;
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

  Serial1.begin(GPSBaud);
  
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

uint8_t buf[sizeof(Payload)];
    uint8_t len = sizeof(buf);
    uint8_t from;
    
if (manager->recvfromAckTimeout((uint8_t *)buf, &len, waitTime, &from)) {  // this runs until we receive some message
      // entering this block means the message is for us

        digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(100);                       // wait for a second
    
     // the rest of this code only runs if we were the intended recipient; which means we're the gateway
      theData = *(Payload*)buf;

      lat2 = theData.lat;
      lon2 = theData.lon;

      Serial.print("lat2=");
      Serial.println(lat2,6);
      Serial.print("lon2=");
      Serial.println(lon2,6);
      
   bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    u8g2.clearBuffer();
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

    float lat1 = flat;
    float lon1 = flon;

    float R = 6371000; // metres
    float phi1 = lat1 * PI/180.; // φ, λ in radians
    float phi2 = lat2 * PI/180.;
    float lambda1 = lon1 * PI/180.;
    float lambda2 = lon2 * PI/180.;
    float y = sin(lambda2-lambda1)*cos(phi2);
    float x = cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(lambda2-lambda1);
    float theta = atan2(y,x);
    float brng = fmod((theta*180/PI + 360),360);

    Serial.print("brng=");
    Serial.println(brng);

    float bearing_rad =atan2(cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1),sin(lon2-lon1)*cos(lat2)); 
    
    float bearing_deg = bearing_rad*180./PI;
    float bearing_compass = fmod(bearing_deg+360,360);
    //float bearing_compass = fmod(bearing_deg+360,360);

   float distance_meters = acos(sin(lat1*PI/180.)*sin(lat2*PI/180.) + cos(lat1*PI/180.)*cos(lat2*PI/180.)*cos(lon2*PI/180.-lon1*PI/180.) ) * 6371000;
   float distance_feet = 3.281*distance_meters;

  float Pi = 3.14159;

  compass.read();
  float heading = compass.heading();

  Serial.print("Compass Heading: ");
  Serial.println(heading);
   Serial.print("bearing_rad");
  Serial.println(bearing_rad);
  Serial.print("bearing_degree");
  Serial.println(bearing_deg);
  Serial.print("bearing_compass: ");
  Serial.println(bearing_compass);
  
  float degree_diff = -1*bearing_compass+heading;
    Serial.print("deg_diff:");
  Serial.println(degree_diff);
    
   u8g2.setCursor(0,15); 
   //u8g2.print("   ");
   //u8g2.print(round(degree_diff));
   //u8g2.print(" ");
   u8g2.print(round(distance_feet));
   u8g2.print(" ft away");
   
   //u8g2.setCursor(0,25); 
   //u8g2.print("hdng: ");
   //u8g2.print(heading);
   
   float theta_graph=fmod(degree_diff+180,360);

   
   int dx = round(r*sin(theta_graph*PI/180.));
   int dy = round(r*cos(theta_graph*PI/180. ));

   
   Serial.print(dx);
   Serial.print(",");
   Serial.println(dy);
   Serial.print(mx);
   Serial.print(",");
   Serial.println(my);

   u8g2.drawCircle(mx,my,r);
   
   u8g2.drawLine(mx, my, mx+dx, my+dy);
   
   //u8g2.setCursor(0,0); 

   
   //u8g2.print(degree_diff);
   
    
   u8g2.sendBuffer();
   
  delay(500);
  }

  gps.stats(&chars, &sentences, &failed);
  
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

}
}
