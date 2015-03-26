// Jetpack for devhouse Spindle.
// By: herman@kopinga.nl
// BSD license

// Stands of the shoulders of:
// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
// Example sketch demonstrating the graphics capabilities of the SSD1331 library
// Adafruit LSM9DS0 9 DOF Board AHRS Example
// Adafruit Bluefruit Low Energy nRF8001 Print echo demo
// Adafruit GPS library basic test!
// Adafruit FONA basic test

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1331.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h> // Maybe we can get rid of this one?
#include "Adafruit_FONA.h"
#include "Adafruit_BLE_UART.h"
#include <stdio.h> // for function sprintf
#include "keys.h"
#include "Location.h"
#include <Bounce.h>


/**********
 FONA
***********/
#define FONA_RST 4
char replybuffer[255];
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint16_t battpercent = 0;
Location current_location = Location();

/**********
 BLUETOOTH
***********/
// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 15
#define ADAFRUITBLE_RDY 17     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 16

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
// Constantly checks for new events on the nRF8001
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

/**********
 GPS 
***********/
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
#define GPSECHO  false

/**********
 OLED
***********/
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// Display pins
// You can use any (4 or) 5 pins 
#define sclk 20
#define mosi 21
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
#define GREY            0x38E7

// Option 1: use any pins but a little slower
Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, mosi, sclk, rst);  

// Option 2: must use the hardware SPI pins 
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be 
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
//Adafruit_SSD1331 display = Adafruit_SSD1331(cs, dc, rst);

/************
 9DOF & AHRS
*************/
float p = 3.1415926;
sensors_vec_t orientation; 

// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS0 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS0(void)
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);    // 1.) Set the accelerometer range
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);     // 2.) Set the magnetometer sensitivity
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);  // 3.) Setup the gyroscope
}

/*************
 Housekeeping
*************/ 
uint32_t timer = millis();
uint32_t lastMillis9dof = 0;
uint32_t lastMillisFona = 0;
Bounce button0 = Bounce(22, 10);
Bounce button1 = Bounce(23, 10);
int largebatt = 0;

void setup(void) {
  // Initialize display first
  display.begin();
  display.fillScreen(BLACK);  
  display.setTextColor(YELLOW);
  display.print("Spindle");
  display.setTextColor(WHITE, BLACK);  
  display.print(" Jetpack\n\n");
  
  display.print("Serial");   
  
  // This boudrate is ignored by Teensy, always runs at full USB speed.
  Serial.begin(115200);  
  Serial.println(F("Initializing FONA.... (May take 3 seconds)"));

  display.print("...\nFona");

  Serial1.begin(4800); // FONA if you're using hardware serial

  // See if the FONA is responding
  if (! fona.begin(Serial1)) {           // can also try fona.begin(Serial1) 
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  display.print("...\nBluetooth");

  BTLEserial.begin();

  display.print("...\nGPS");

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

  display.print("...\n9DOF");
  
  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  // Setup the sensor gain and integration time.
  configureLSM9DS0();    
  display.print("...");  

  // Housekeeping
  pinMode(22, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);  
}


void loop() {
  // Buttons!
  button0.update();
  button1.update();
  
  if (button0.fallingEdge()) {
    clearMiddle();
    largebatt = !largebatt;
    disMinute();
  }
  if (button1.fallingEdge()) {
  }

  // Headlight code.
  analogWrite(6,map(analogRead(A0), 0,500,128,0));
  
/**********
9DOF & AHRS
***********/  
  // Use the simple AHRS function to get the current orientation.
  if (millis() - lastMillis9dof > 1000 && ahrs.getOrientation(&orientation)) {
    disSecond();
  }
  
/**********
 GPS
***********/
  
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

  // approximately every 10 seconds or so, print out the current stats
  if (millis() - timer > 10000) { 
    dis10S();
  }  

/**********
 GSM/FONA
***********/  
  // Every minute update battery percentage.
  if (millis() - lastMillisFona > 10000) {
    disMinute();
  }

  if (Serial.available()) {
    doFona();
  }

/**********
 BLUETOOTH
***********/
 // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  aci_evt_opcode_t status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
        display.drawChar(91,8,'b',WHITE,BLACK,1);        
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
        display.drawChar(91,8,'B',BLUE,BLACK,1);        
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
        display.drawChar(91,8,'b',WHITE,BLACK,1);        
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial.available()) {
      // Read a line from Serial
      Serial.setTimeout(100); // 100 millisecond timeout
      String s = Serial.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial.print(F("\n* Sending -> \"")); Serial.print((char *)sendbuffer); Serial.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }
}

#ifdef __AVR__ // ToDo: Reminder to fix this properly.
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
