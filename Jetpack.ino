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

  // Print SIM card IMEI number.
  char imei[15] = {0}; // MUST use a 16 character buffer for IMEI!
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("SIM card IMEI: "); Serial.println(imei);
  }

  printMenu();


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

void printMenu(void) {
   Serial.println(F("-------------------------------------"));
   Serial.println(F("[?] Print this menu"));
   Serial.println(F("[a] read the ADC (2.8V max)"));
   Serial.println(F("[b] read the Battery V and % charged"));
   Serial.println(F("[C] read the SIM CCID"));
   Serial.println(F("[U] Unlock SIM with PIN code"));
   Serial.println(F("[i] read RSSI"));
   Serial.println(F("[n] get Network status"));
   Serial.println(F("[v] set audio Volume"));
   Serial.println(F("[V] get Volume"));
   Serial.println(F("[H] set Headphone audio"));
   Serial.println(F("[e] set External audio"));
   Serial.println(F("[T] play audio Tone"));
   Serial.println(F("[f] tune FM radio"));
   Serial.println(F("[F] turn off FM"));
   Serial.println(F("[m] set FM volume"));
   Serial.println(F("[M] get FM volume"));
   Serial.println(F("[q] get FM station signal level"));
   Serial.println(F("[P] PWM/Buzzer out"));
   Serial.println(F("[c] make phone Call"));
   Serial.println(F("[h] Hang up phone"));
   Serial.println(F("[p] Pick up phone"));
   Serial.println(F("[N] Number of SMSs"));
   Serial.println(F("[r] Read SMS #"));
   Serial.println(F("[R] Read All SMS"));
   Serial.println(F("[d] Delete SMS #"));
   Serial.println(F("[s] Send SMS"));
   Serial.println(F("[y] Enable network time sync"));   
   Serial.println(F("[Y] Enable NTP time sync (GPRS)"));   
   Serial.println(F("[t] Get network time"));   
   Serial.println(F("[G] Enable GPRS"));
   Serial.println(F("[g] Disable GPRS"));
   Serial.println(F("[l] Query GSMLOC (GPRS)"));
   Serial.println(F("[w] Read webpage (GPRS)"));
   Serial.println(F("[W] Post to website (GPRS)"));
   Serial.println(F("[S] create Serial passthru tunnel"));
   Serial.println(F("-------------------------------------"));
   Serial.println(F(""));
  
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

  if (Serial.available()) {
  char command = Serial.read();
    Serial.println(command);
    
    
    switch (command) {
      case '?': {
        printMenu();
        break;
      }
      
      case 'a': {
        // read the ADC
        uint16_t adc;
        if (! fona.getADCVoltage(&adc)) {
          Serial.println(F("Failed to read ADC"));
        } else {
          Serial.print(F("ADC = ")); Serial.print(adc); Serial.println(F(" mV"));
        }
        break;
      }
      
      case 'b': {
          // read the battery voltage and percentage
          uint16_t vbat;
          if (! fona.getBattVoltage(&vbat)) {
            Serial.println(F("Failed to read Batt"));
          } else {
            Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
          }
   
  
          if (! fona.getBattPercent(&vbat)) {
            Serial.println(F("Failed to read Batt"));
          } else {
            Serial.print(F("VPct = ")); Serial.print(vbat); Serial.println(F("%"));
          }
   
          break;
      }
  
      case 'U': {
          // Unlock the SIM with a PIN code
          char PIN[5];
          flushSerial();
          Serial.println(F("Enter 4-digit PIN"));
          readline(PIN, 3);
          Serial.println(PIN);
          Serial.print(F("Unlocking SIM card: "));
          if (! fona.unlockSIM(PIN)) {
            Serial.println(F("Failed"));
          } else {
            Serial.println(F("OK!"));
          }        
          break;
      }
  
      case 'C': {
          // read the CCID
          fona.getSIMCCID(replybuffer);  // make sure replybuffer is at least 21 bytes!
          Serial.print(F("SIM CCID = ")); Serial.println(replybuffer);
          break;
      }
  
      case 'i': {
          // read the RSSI
          uint8_t n = fona.getRSSI();
          int8_t r;
          
          Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
          if (n == 0) r = -115;
          if (n == 1) r = -111;
          if (n == 31) r = -52;
          if ((n >= 2) && (n <= 30)) {
            r = map(n, 2, 30, -110, -54);
          }
          Serial.print(r); Serial.println(F(" dBm"));
         
          break;
      }
      
      case 'n': {
          // read the network/cellular status
          uint8_t n = fona.getNetworkStatus();
          Serial.print(F("Network status ")); 
          Serial.print(n);
          Serial.print(F(": "));
          if (n == 0) Serial.println(F("Not registered"));
          if (n == 1) Serial.println(F("Registered (home)"));
          if (n == 2) Serial.println(F("Not registered (searching)"));
          if (n == 3) Serial.println(F("Denied"));
          if (n == 4) Serial.println(F("Unknown"));
          if (n == 5) Serial.println(F("Registered roaming"));
          break;
      }
      
      /*** Audio ***/
      case 'v': {
        // set volume
        flushSerial();
        Serial.print(F("Set Vol %"));
        uint8_t vol = readnumber();
        Serial.println();
        if (! fona.setVolume(vol)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }
  
      case 'V': {
        uint8_t v = fona.getVolume();
        Serial.print(v); Serial.println("%");
      
        break; 
      }
      
      case 'H': {
        // Set Headphone output
        if (! fona.setAudio(FONA_HEADSETAUDIO)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        fona.setMicVolume(FONA_HEADSETAUDIO, 15);
        break;
      }
      case 'e': {
        // Set External output
        if (! fona.setAudio(FONA_EXTAUDIO)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
  
        fona.setMicVolume(FONA_EXTAUDIO, 10);
        break;
      }
  
      case 'T': {
        // play tone
        flushSerial();
        Serial.print(F("Play tone #"));
        uint8_t kittone = readnumber();
        Serial.println();
        // play for 1 second (1000 ms)
        if (! fona.playToolkitTone(kittone, 1000)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }
      
      /*** FM Radio ***/
      
      case 'f': {
        // get freq
        flushSerial();
        Serial.print(F("FM Freq (eg 1011 == 101.1 MHz): "));
        uint16_t station = readnumber();
        Serial.println();
        // FM radio ON using headset
        if (fona.FMradio(true, FONA_HEADSETAUDIO)) {
          Serial.println(F("Opened"));
        }
       if (! fona.tuneFMradio(station)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Tuned"));
        }
        break;
      }
      case 'F': {
        // FM radio off
        if (! fona.FMradio(false)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }
      case 'm': {
        // Set FM volume.
        flushSerial();
        Serial.print(F("Set FM Vol [0-6]:"));
        uint8_t vol = readnumber();
        Serial.println();
        if (!fona.setFMVolume(vol)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }
      case 'M': {
        // Get FM volume.
        uint8_t fmvol = fona.getFMVolume();
        if (fmvol < 0) {
          Serial.println(F("Failed"));
        } else {
          Serial.print(F("FM volume: "));
          Serial.println(fmvol, DEC);
        }
        break;
      }
      case 'q': {
        // Get FM station signal level (in decibels).
        flushSerial();
        Serial.print(F("FM Freq (eg 1011 == 101.1 MHz): "));
        uint16_t station = readnumber();
        Serial.println();
        int8_t level = fona.getFMSignalLevel(station);
        if (level < 0) {
          Serial.println(F("Failed! Make sure FM radio is on (tuned to station)."));
        } else {
          Serial.print(F("Signal level (dB): "));
          Serial.println(level, DEC);
        }
        break;
      }
      
      /*** PWM ***/
      
      case 'P': {
        // PWM Buzzer output @ 2KHz max
        flushSerial();
        Serial.print(F("PWM Freq, 0 = Off, (1-2000): "));
        uint16_t freq= readnumber();
        Serial.println();
        if (! fona.PWM(freq)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;
      }
  
      /*** Call ***/
      case 'c': {      
        // call a phone!
        char number[30];
        flushSerial();
        Serial.print(F("Call #"));
        readline(number, 30);
        Serial.println();
        Serial.print(F("Calling ")); Serial.println(number);
        if (!fona.callPhone(number)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
        
        break;
      }
      case 'h': {
         // hang up! 
        if (! fona.hangUp()) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;     
      }
  
      case 'p': {
         // pick up! 
        if (! fona.pickUp()) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("OK!"));
        }
        break;     
      }
      
      /*** SMS ***/
      
      case 'N': {
          // read the number of SMS's!
          int8_t smsnum = fona.getNumSMS();
          if (smsnum < 0) {
            Serial.println(F("Could not read # SMS"));
          } else {
            Serial.print(smsnum); 
            Serial.println(F(" SMS's on SIM card!"));
          }
          break;
      }
      case 'r': {
        // read an SMS
        flushSerial();
        Serial.print(F("Read #"));
        uint8_t smsn = readnumber();
        Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
  
        // Retrieve SMS sender address/phone number.
        if (! fona.getSMSSender(smsn, replybuffer, 250)) {
          Serial.println("Failed!");
          break;
        }
        Serial.print(F("FROM: ")); Serial.println(replybuffer);
  
        // Retrieve SMS value.
        uint16_t smslen;
        if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
          Serial.println("Failed!");
          break;
        }
        Serial.print(F("***** SMS #")); Serial.print(smsn); 
        Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
        Serial.println(replybuffer);
        Serial.println(F("*****"));
        
        break;
      }
      case 'R': {
        // read all SMS
        int8_t smsnum = fona.getNumSMS();
        uint16_t smslen;
        for (int8_t smsn=1; smsn<=smsnum; smsn++) {
          Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
          if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
             Serial.println(F("Failed!"));
             break;
          }
          // if the length is zero, its a special case where the index number is higher
          // so increase the max we'll look at!
          if (smslen == 0) {
            Serial.println(F("[empty slot]"));
            smsnum++;
            continue;
          }
          
          Serial.print(F("***** SMS #")); Serial.print(smsn); 
          Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
          Serial.println(replybuffer);
          Serial.println(F("*****"));
        }
        break;
      }
  
      case 'd': {
        // delete an SMS
        flushSerial();
        Serial.print(F("Delete #"));
        uint8_t smsn = readnumber();
        
        Serial.print(F("\n\rDeleting SMS #")); Serial.println(smsn);
        if (fona.deleteSMS(smsn)) {
          Serial.println(F("OK!"));
        } else {
          Serial.println(F("Couldn't delete"));
        }
        break;
      }
      
      case 's': {
        // send an SMS!
        char sendto[21], message[141];
        flushSerial();
        Serial.print(F("Send to #"));
        readline(sendto, 20);
        Serial.println(sendto);
        Serial.print(F("Type out one-line message (140 char): "));
        readline(message, 140);
        Serial.println(message);
        if (!fona.sendSMS(sendto, message)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("Sent!"));
        }
        
        break;
      }
  
      /*** Time ***/
  
      case 'y': {
        // enable network time sync
        if (!fona.enableNetworkTimeSync(true))
          Serial.println(F("Failed to enable"));
        break;
      }
  
      case 'Y': {
        // enable NTP time sync
        if (!fona.enableNTPTimeSync(true, F("pool.ntp.org")))
          Serial.println(F("Failed to enable"));
        break;
      }
  
      case 't': {
          // read the time
          char buffer[23];
  
          fona.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!
          Serial.print(F("Time = ")); Serial.println(buffer);
          break;
      }
  
      /*********************************** GPRS */
      
      case 'g': {
         // turn GPRS off
         if (!fona.enableGPRS(false))  
           Serial.println(F("Failed to turn off"));
         break;
      }
      case 'G': {
         // turn GPRS on
         if (!fona.enableGPRS(true))  
           Serial.println(F("Failed to turn on"));
         break;
      }
      case 'l': {
         // check for GSMLOC (requires GPRS)
         uint16_t returncode;
         
         if (!fona.getGSMLoc(&returncode, replybuffer, 250))
           Serial.println(F("Failed!"));
         if (returncode == 0) {
           Serial.println(replybuffer);
         } else {
           Serial.print(F("Fail code #")); Serial.println(returncode);
         }
         
         break;
      }
      case 'w': {
        // read website URL
        uint16_t statuscode;
        int16_t length;
        char url[80];
        
        flushSerial();
        Serial.println(F("NOTE: in beta! Use small webpages to read!"));
        Serial.println(F("URL to read (e.g. www.adafruit.com/testwifi/index.html):"));
        Serial.print(F("http://")); readline(url, 79);
        Serial.println(url);
        
         Serial.println(F("****"));
         if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
           Serial.println("Failed!");
           break;
         }
         while (length > 0) {
           while (fona.available()) {
             char c = fona.read();
             
             // Serial.write is too slow, we'll write directly to Serial register!
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
             loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
             UDR0 = c;
  #else
             Serial.write(c);
  #endif
             length--;
             if (! length) break;
           }
         }
         Serial.println(F("\n****"));
         fona.HTTP_GET_end();
         break;
      }
      
      case 'W': {
        // Post data to website
        uint16_t statuscode;
        int16_t length;
        char url[80];
        char data[80];
        
        flushSerial();
        Serial.println(F("NOTE: in beta! Use simple websites to post!"));
        Serial.println(F("URL to post (e.g. httpbin.org/post):"));
        Serial.print(F("http://")); readline(url, 79);
        Serial.println(url);
        Serial.println(F("Data to post (e.g. \"foo\" or \"{\"simple\":\"json\"}\"):"));
        readline(data, 79);
        Serial.println(data);
        
         Serial.println(F("****"));
         if (!fona.HTTP_POST_start(url, F("text/plain"), (uint8_t *) data, strlen(data), &statuscode, (uint16_t *)&length)) {
           Serial.println("Failed!");
           break;
         }
         while (length > 0) {
           while (fona.available()) {
             char c = fona.read();
             
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
             loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
             UDR0 = c;
  #else
             Serial.write(c);
  #endif
             
             length--;
             if (! length) break;
           }
         }
         Serial.println(F("\n****"));
         fona.HTTP_POST_end();
         break;
      }
      /*****************************************/
        
      case 'S': {
        Serial.println(F("Creating SERIAL TUBE"));
        while (1) {
          while (Serial.available()) {
            fona.write(Serial.read());
          }
          if (fona.available()) {
            Serial.write(fona.read());
          }
        }
        break;
      }
      
      default: {
        Serial.println(F("Unknown command"));
        printMenu();
        break;
      }
    }
    // flush input
    flushSerial();
    while (fona.available()) {
      Serial.write(fona.read());
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

void flushSerial() {
    while (Serial.available()) 
    Serial.read();
}

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}
  
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;
  
  while (true) {
    if (buffidx > maxbuff) {
      //Serial.println(F("SPACE"));
      break;
    }

    while(Serial.available()) {
      char c =  Serial.read();

      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;
        
        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }
    
    if (timeoutvalid && timeout == 0) {
      //Serial.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}
