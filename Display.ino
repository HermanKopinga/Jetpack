void disMinute () {
    // +/- 125 msec
    fona.getBattPercent(&battpercent);
    Serial.print(F("Batterij:"));
    Serial.println(battpercent);
    display.setCursor(18,57);
    display.print(battpercent);
    display.print("%");
    lastMillisFona = millis(); // reset the timer    
    
    uint8_t r = map(fona.getRSSI(), 0, 31, 1, 5);
    display.setCursor(48,57);
    display.print(r);
    lastMillisFona = millis(); // reset the timer    

    if (largebatt) {
      display.setTextSize(4);
      display.setCursor(8,18);
      display.print(battpercent);
      display.print("%");
      display.setTextSize(1);
    }

    if (GPS.fix && GPS.HDOP < 5 && GPS.HDOP != 0) {
      current_location.set(GPS);
      Serial.print(F("Location: "));
      Serial.print(current_location.latitude, 6);
      Serial.print(F(", "));
      Serial.print(current_location.longitude, 6);
      Serial.print(F(", "));
      Serial.println(current_location.altitude, 2);
      Serial.print(F("HDOP: ")); Serial.println(GPS.HDOP);
      sendLocation(battpercent);
    }
}

void dis10S() {
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

void disSecond() {
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

void clearMiddle() {
  display.fillRect(0,8,89,49,BLACK);
}
