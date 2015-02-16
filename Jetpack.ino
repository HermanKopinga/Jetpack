/*************************************************** 
  This is a example sketch demonstrating the graphics
  capabilities of the SSD1331 library  for the 0.96" 
  16-bit Color OLED with SSD1331 driver chip

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/684

  These displays use SPI to communicate, 4 or 5 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Other credits:
// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada


#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h> // Maybe we can get rid of this one?
#include "Adafruit_FONA.h"

#define FONA_RST 4
char replybuffer[255];
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);



// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, 6, NEO_GRB + NEO_KHZ800);

// You should make the following connections with the Due and GPS module:
// GPS power pin to Arduino Due 3.3V output.
// GPS ground pin to Arduino Due ground.
// For hardware serial 1 (recommended):
//   GPS TX to Arduino Due Serial1 RX pin 19
//   GPS RX to Arduino Due Serial1 TX pin 18
#define GPSSerial Serial3

Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Display pins
// You can use any (4 or) 5 pins 
#define sclk 13
#define mosi 11
#define cs   10
#define rst  9
#define dc   5


// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

// Option 1: use any pins but a little slower
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);  

// Option 2: must use the hardware SPI pins 
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be 
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, rst);

float p = 3.1415926;

// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS0 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS0(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup(void) {
  while (!Serial);
  
  Serial.begin(115200);  
  Serial.println(F("Adafruit LSM9DS0 9 DOF Board AHRS Example")); Serial.println("");
  Serial.println("Adafruit GPS library basic test!");
  Serial.println(F("FONA basic test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  Serial1.begin(4800); // FONA if you're using hardware serial

  // See if the FONA is responding
  if (! fona.begin(Serial1)) {           // can also try fona.begin(Serial1) 
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPSSerial.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!

#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  
  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  configureLSM9DS0();  
  
  // Neopixels
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  // Initialize display
  display.begin();
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__



// For GPS
uint32_t timer = millis();
uint32_t lastMillis = 0;

void loop() {
   sensors_vec_t   orientation;
 
  // Use the simple AHRS function to get the current orientation.
  if (millis() - lastMillis > 2000 && ahrs.getOrientation(&orientation))
  {
    display.fillScreen(BLACK);
    display.setCursor(0,0);
    display.print("Spindle Jetpack\n\n");
    /* 'orientation' should have valid .roll and .pitch fields */
    display.print(orientation.roll);
    display.print(F(" "));
    display.print(orientation.pitch);
    display.print(F(" "));
    display.print(orientation.heading);
    display.println(F(""));
    lastMillis = millis();
  }
  
  // GPS
  
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    display.print(GPS.hour, DEC); display.print(':');
    display.print(GPS.minute, DEC); display.print(':');
    display.print(GPS.seconds, DEC); display.print(" GMT");
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }  
  
  // NeoPixels constant value.
  strip.setPixelColor(0, strip.Color(GPS.minute/4,GPS.hour/2,GPS.seconds/4));
  strip.setPixelColor(1, strip.Color(1,0,0));
  strip.setPixelColor(3, strip.Color(3,0,0));
  strip.setPixelColor(5, strip.Color(5,0,0));
  strip.setPixelColor(7, strip.Color(7,0,0));
  strip.show();
}

