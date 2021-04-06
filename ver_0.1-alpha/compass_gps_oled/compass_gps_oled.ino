#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <U8x8lib.h>
#include <TinyGPS.h>
#include <math.h>

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

TinyGPS gps;
static const uint32_t GPSBaud = 9600;

float R=5;

//conant
//float lat2 = 42.411922994545044;
//float lon2 = -71.29791385108904;

//boston
float lat2 = 42.358306550939965;
float lon2 = -71.06660358413588;

void setup(void) {
  Serial1.begin(GPSBaud);
  
  Serial.begin(115200);
u8x8.begin();
  
  u8x8.setFont(u8x8_font_7x14B_1x2_f);
   u8x8.clear();
   u8x8.setCursor(0,0); 
   u8x8.print("Starting...");
   

  Serial.println("Magnetometer Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }
  u8x8.clear();
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
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(",");
    Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);

    float lat1 = flat;
    float lon1 = flon;
    float bearing_rad =atan2(cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1),sin(lon2-lon1)*cos(lat2)); 
    float bearing_deg = bearing_rad*180/PI;
    float bearing_compass = fmod(bearing_deg+360,360);

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
  Serial.print("bearing_compass: ");
  Serial.println(bearing_compass);
 
  float degree_diff = bearing_compass-heading;
    Serial.print("deg_diff:");
  Serial.println(degree_diff);
    
   u8x8.setCursor(0,2); 
   u8x8.print("   ");
   u8x8.print(degree_diff);

   float theta=degree_diff;

   float dx = R*sin(theta);
   float dy = R*cos(theta);
   
   
  delay(500);
  }

  gps.stats(&chars, &sentences, &failed);
  
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

    
}
