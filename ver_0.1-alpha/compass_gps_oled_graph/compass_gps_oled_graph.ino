#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <TinyGPS.h>
#include <math.h>
#include <Arduino.h>

#include <U8g2lib.h>

#define r 22
#define mx 120-r
#define my 32

//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

TinyGPS gps;
static const uint32_t GPSBaud = 9600;

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
float lat2 = 42.41293317045196;
float lon2 = -71.29791687460762;


void setup(void) {
  Serial1.begin(GPSBaud);
  
  Serial.begin(115200);
u8g2.begin();
  u8g2.clearBuffer();
  
  u8g2.setFontDirection(0);
   u8g2.setFont(u8g2_font_6x10_tf); 

   u8g2.setCursor(0,0); 
   u8g2.print("Starting...");
   u8g2.sendBuffer();

  Serial.println("Magnetometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
  u8g2.clear();
}

void loop(void) {


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
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  float Pi = 3.14159;

  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;



  // Normalize to 0-360
  if (heading < 0) {
    heading = 360 + heading;
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading);
   Serial.print("bearing_rad");
  Serial.println(bearing_rad);
  Serial.print("bearing_degree");
  Serial.println(bearing_deg);
  Serial.print("bearing_compass: ");
  Serial.println(bearing_compass);
  
  float degree_diff = bearing_compass-heading;
    Serial.print("deg_diff:");
  Serial.println(degree_diff);
    
   u8g2.setCursor(0,15); 
   //u8g2.print("   ");
   //u8g2.print(round(degree_diff));
   u8g2.print(" ");
   u8g2.print(round(distance_feet));
   u8g2.print(" feet");
   
   float theta_graph=degree_diff;

   
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
