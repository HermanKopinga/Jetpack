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

// Housekeeping
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
    largebatt = !largebatt;
  }
  if (button1.fallingEdge()) {
  }

  // Headlight code.
  analogWrite(6,map(analogRead(A0), 0,500,128,0));
  
/**********
9DOF & AHRS
***********/  
  sensors_vec_t orientation; 
  // Use the simple AHRS function to get the current orientation.
  if (millis() - lastMillis9dof > 1000 && ahrs.getOrientation(&orientation))
  {
    int heading = orientation.heading + 180;
    display.setCursor(0,57);
    if (heading < 0) {
      display.print("??");
      display.print(heading);                
    } else if (heading < 23) {
      display.print("W ");
    } else if (heading < 68) {
      display.print("ZW");      
    } else if (heading < 113) { 
      display.print("Z ");
    } else if (heading < 158) {
      display.print("ZO");      
    } else if (heading < 203) {
      display.print("O ");
    } else if (heading < 248) {
      display.print("NO");      
    } else if (heading < 293) {
      display.print("N ");
    } else if (heading < 338) {
      display.print("NW");      
    } else if (heading < 361) {
      display.print("W ");
    } else {
      display.print("??");
      display.print(heading);          
    }
    lastMillis9dof = millis();
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
    timer = millis(); // reset the timer
    char time[5];
    // ToDo: timezone implementation
    sprintf(time, "%2d:%02d", GPS.hour+1, GPS.minute);
    display.setCursor(65,57);
    display.print(time);
    /*
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
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); */
    if (GPS.fix) {
      display.drawChar(91,16,'G',WHITE,BLACK,1);
      /*display.print(GPS.latitude, 4); display.print(':');
      display.println(GPS.longitude, 4);*/
      /*Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);*/      
    }
    else {
      display.drawChar(91,16,'g',WHITE,BLACK,1);
    }
  }  

/**********
 GSM/FONA
***********/  
  // Every minute update battery percentage.
  if (millis() - lastMillisFona > 10000) {
    // +/- 125 msec
    fona.getBattPercent(&battpercent);
    Serial.print(F("Batterij:"));
    Serial.println(battpercent);
    display.setCursor(18,57);
    display.print(battpercent);
    display.print("%");
    lastMillisFona = millis(); // reset the timer    
    byte RSSI = fona.getRSSI();
    
    uint8_t r = map(fona.getRSSI(), 0, 31, 1, 5);
    display.setCursor(48,57);
    display.print(r);
    lastMillisFona = millis(); // reset the timer    

    if (GPS.fix && GPS.HDOP < 5 && GPS.HDOP != 0) {
      current_location.set(GPS);
      Serial.print(F("Location: "));
      Serial.print(current_location.latitude, 6);
      Serial.print(F(", "));
      Serial.print(current_location.longitude, 6);
      Serial.print(F(", "));
      Serial.println(current_location.altitude, 2);
      Serial.print(F("HDOP: ")); Serial.println(GPS.HDOP);
      sendLocation(orientation.heading + 180, battpercent);
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
          if (n == 5) {
            Serial.println(F("Registered roaming"));
            display.drawChar(0,8,'-',GREY,BLACK,1);
          }
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

void sendLocation (uint16_t heading, uint16_t vbat) {
  char url[160];
  uint16_t statuscode;
  int16_t length;

  uint32_t latint;
  uint32_t longint;
  char vbatc[4];
  char headingc[4];
  char Lat_c1[3];
  char Lat_c2[8];
  char Long_c1[3];
  char Long_c2[8];

  fona.getBattPercent(&vbat);
  itoa(vbat, vbatc, 10);
  itoa(heading, headingc, 10);
  
  itoa(current_location.latitude,Lat_c1,10);
  itoa(current_location.longitude,Long_c1,10);
  
  latint = (current_location.latitude-int(current_location.latitude))*10000000;
  itoa(latint,Lat_c2,10);
  
  longint = (current_location.longitude-int(current_location.longitude))*10000000;
  itoa(longint,Long_c2,10);

  sprintf (url, "http://data.sparkfun.com/input/%s?private_key=%s&latitude=%s.%s&longitude=%s.%s&battery=%s&heading=%s&herman=%s",
    SPARKFUN_PUBLIC_KEY, SPARKFUN_PRIVATE_KEY, Lat_c1, Lat_c2, Long_c1, Long_c2, vbatc, headingc, "1");
  
  Serial.print(F("Sending: ")); Serial.println(url);

  uint8_t rssi = fona.getRSSI();

  if (rssi > 5) {
    // Make an attempt to turn GPRS mode on.  Sometimes the FONA gets freaked out and GPRS doesn't turn off.
    // When this happens you can't turn it on aagin, but you don't need to because it's on.  So don't sweat
    // the error case here -- GPRS could already be on -- just keep on keeping on and let HTTP_GET_start()
    // error if there's a problem with GPRS.
    display.setCursor(0,8);
    display.print("GPRS       ");      
    if (!fona.enableGPRS(true)) {
      Serial.println(F("Failed to turn GPRS on!"));
    }

    if (fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) {
      display.setCursor(0,8);
      while (length > 0) {
        while (fona.available()) {
          char c = fona.read();
          display.print(c);
          Serial.print(c);
          length--;
          if (! length) break;
        }
      }
      fona.HTTP_GET_end();
    } else {
      Serial.println(F("Failed to send GPRS data!"));
    }

    if (!fona.enableGPRS(false)) {
      Serial.println(F("Failed to turn GPRS off!"));
    }
  } else {
    Serial.println(F("Can't transmit, network signal strength is crap!"));
  }

}
