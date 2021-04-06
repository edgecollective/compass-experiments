
#include <RHSoftwareSPI.h> // http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.113.zip
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#define RH_HAVE_SERIAL
#include <SPI.h>
#include <TinyGPS.h>

TinyGPS gps;

static const uint32_t GPSBaud = 9600;

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define gatewayNode 1
#define this_node_id 2


// Class to manage message delivery and receipt, using the driver declared above
RHMesh *manager;

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

void setup() {


  Serial1.begin(GPSBaud);
  
  pinMode(LED, OUTPUT);

   
  Serial.begin(115200);

   
  manager = new RHMesh(rf95, this_node_id);

  if (!manager->init()) {
    Serial.println(F("mesh init failed"));
    
  } else {
    delay(1000);
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

 
    
void loop() {

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

     theData.lat = flat;
    theData.lon = flon;
    uint8_t error = manager->sendtoWait((uint8_t *)&theData, sizeof(theData), gatewayNode);
    
    digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
    delay(100);                       // wait for a second
    
  delay(500);
  }

  gps.stats(&chars, &sentences, &failed);
  
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

    
}
