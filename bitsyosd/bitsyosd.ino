/**
   ___  _ __           ____  _______ 
  / _ )(_) /____ __ __/ __ \/ __/ _ \
 / _  / / __(_-</ // / /_/ /\ \/ // /
/____/_/\__/___/\_, /\____/___/____/ 
               /___/                 

BitsyOSD - A minimalistic OSD firmware for the MinimOSD Hardware.

Designed for Multicopters, Airplanes and UGV's.

LEGAL INFORMATION

http://github.com/FPVTools/BitsyOSD

Copyright (c) 2016. All rights reserved.
Author: Andreas Schwarz (flyandi)
Version: 1.4

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>

**/


/** 
  * PROGMEM Header, we don't really use the EEPROM (yet)
  */
/*
#undef PROGMEM
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 
*/

/** 
  * (Includes)
  */


// Global 
#include <FastSerial.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

// Arduino
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "wiring.h"
#endif

// Local
#include "const.h"
#include "config.h"
#include "defaults.h"
#include "max7456.h"
#include "osd.h"
#include "gps.h"

#ifdef ENABLE_433
  #include <RH_ASK.h>
  #include <SPI.h>
  #include "rf433.h"

  RH_ASK driver(2000, CURRENT_PIN, 11, 5);
#endif


/**
  * (Variables)
  */

// Classes
FastSerialPort0(Serial);
OSD osd;

// Runtime
static struct OSD_RUNTIME_VALUES runtime;
static struct GPS_DATA gpsdata;    // Global object that stores the gpsdata
static struct RF433_DATA rf433data; //Global object that stores de RF 433 data

/** 
  * (Setup)
  */

void setup() {
    // select osd output
    pinMode(MAX7456_SELECT,  OUTPUT); 
    
    // led
    pinMode(LED_PIN, OUTPUT);
    
    // start serial
    Serial.begin(GPS_SPEED);
    
    // Prepare OSD for displaying 
    unplugSlaves();
    
    osd.init();
    osd.clear();         
    
    // some good starting values
    runtime.gpswarn = false;
    runtime.gpshashome = false;
    runtime.gpsaltprev = -1;
    runtime.osdredraw = true; 
    runtime.distance = 0;
    runtime.tdistance = 0;
    runtime.rf433error = false;
    runtime.rf433warn = false;
    runtime.laps = 0;
    runtime.max_Groundspeed = 0;
    runtime.lap_max_Groundspeed = 0;
    runtime.start_time = millis();
    runtime.best_laptime = 0;
    runtime.last_laptime = 0;
    runtime.laptime_diff = 0;
    runtime.laptime_symbol = 0;
    runtime.has_lastlap = false;

        //Start 433
#ifdef ENABLE_433
    if (!driver.init())
         runtime.rf433error = true;
#endif

    // pre boot delay
    delay(500);

    // boot message
    MessageAlert(true, " BOOT");
    DrawLogo();
    
    // boot time
    delay(BOOTTIME);
    
    // osd time
    runtime.tosdtime =  millis();
} 


/** 
  * (Loop) Here is where the magic happens, keep it simple and fast
  */

void loop() {
  
  // Main loop
  Update();
  
  // Mandatory delay for proper display update
  delay(50);
}

/**
  * (Update)
  */
void Update() {

  // Update the GPS Information
  UpdateGPS();

  //Update RF433 info
  Update433();
  
  // Update Values
  UpdateValues();

  // Update the Display
  UpdateDisplay();
}


/**
  * (UpdateGPS) Updates the GPS Data
  */

void UpdateGPS() {

  #ifdef GPS_ENABLED

    // process data
    if(!gps_process()) {
      return; // nothing yet 
    }
    
    // update values
    GPS_DATA tempdata = gps_values();
    
    if(runtime.gpsacquired && tempdata.Satellites < GPS_REQUIRED_SAT) {
      // timer that prevents to show the splashscreen after we lost temporarily the GPS
      if(runtime.gpstimeout == -1) {
        // start
        runtime.gpstimeout = millis();
      } else {
         if(runtime.gpstimeout + GPS_TIMEOUT < millis()) {
           gpsdata = tempdata; // apply 
           runtime.gpsacquired = false;
         }    
      }
    } else {
      runtime.gpstimeout = -1;
      runtime.gpsacquired = true;
      gpsdata = tempdata;
    }
    
    // calculate gps data
    if(!runtime.gpshashome) {
      // find gps home position under certain conditions
      if(gpsdata.Status >= GPS_ACCURACY && gpsdata.Satellites >= GPS_REQUIRED_SAT) {
        // check old vs 
        if(runtime.gpsaltprev == -1) {
          runtime.gpsaltprev = gpsdata.Altitude;
        } else if(fabs(runtime.gpsaltprev - gpsdata.Altitude) > GPS_HOME_SENSITIVITY) {
          // reset 
          runtime.gpsaltcount = 0;
          runtime.gpsaltprev = gpsdata.Altitude;
        } else {
          runtime.gpsaltcount = runtime.gpsaltcount + 1;
          // check
          if(runtime.gpsaltcount >= GPS_HOME_WAIT) {
            // we have a home position
            runtime.gpshashome = true;
            runtime.gpshomelat = gpsdata.Latitude;
            runtime.gpshomelon = gpsdata.Longitude;
            runtime.gpsasl = gpsdata.Altitude;
            runtime.osdredraw = true;
          } else { 
            delay(1);
          }
        }
      }
    } else { 
      // Climb Rate
      runtime.gpsclimb = gpsdata.Climbrate < (-1 * GPS_CLIMB_VALUE) ? SYMBOL_CLIMBUP : (gpsdata.Climbrate > GPS_CLIMB_VALUE ? SYMBOL_CLIMBDOWN : SYMBOL_NOCLIMB);
      
      // Home Calculations
      float rads = fabs(runtime.gpshomelat) * 0.0174532925;
      double scaleLongDown = cos(rads);
      double scaleLongUp   = 1.0f/cos(rads);

      // Home Distance
      float dstlat = fabs(runtime.gpshomelat - gpsdata.Latitude) * 111319.5;
      float dstlon = fabs(runtime.gpshomelon - gpsdata.Longitude) * 111319.5 * scaleLongDown;

      runtime.gpshomedistance = sqrt(sq(dstlat) + sq(dstlon));

      // Home Bearing
      dstlat = (runtime.gpshomelat - gpsdata.Latitude);
      dstlon = (runtime.gpshomelon - gpsdata.Longitude) * scaleLongUp;  
      long bearing = 90 + (atan2(dstlat, -dstlon) * 57.295775); 

      // calculate bearing
      if (bearing < 0) bearing += 360;      // normalization
      bearing = bearing - 180;              // absolut return direction
      if (bearing < 0) bearing += 360;      // normalization
      
      if(HOME_BEARING == 1) {
        bearing = bearing - gpsdata.Heading;  // relative home direction
        if (bearing < 0) bearing += 360;      // normalization
      }
      
      runtime.gpshomebearing = bearing;     // save bearing

      // this is for the fancy heading
      runtime.gpshomedirection = round((float)(bearing/360.0f) * 16.0f);

      // update travel distance
      if (runtime.tdistance + GPS_MEASURE_PERIOD <= millis()) {
        if (gpsdata.Groundspeed > 1.0) {
          runtime.distance = runtime.distance + (gpsdata.Groundspeed * ((millis() - runtime.tdistance) / 1000.0)); 
        }
        runtime.tdistance = millis();
      }
     
      
      // update flytime
      if(gpsdata.Altitude - runtime.gpsasl > GPS_MEASURE_FLYTIME) {   
        runtime.flytime = runtime.flytime + (millis() - runtime.flytimelast);
      }
      runtime.flytimelast = millis();
    }
    //RC Car max speed
    #ifdef CAR_MODE
      if (gpsdata.Groundspeed > runtime.max_Groundspeed) {
        runtime.max_Groundspeed = gpsdata.Groundspeed;
      }
      if (gpsdata.Groundspeed > runtime.lap_max_Groundspeed) {
        runtime.lap_max_Groundspeed = gpsdata.Groundspeed;
      }
    #endif
  #endif
}

/**
  * (Update433) Updates the 433 Data
  */

void Update433() {
#ifdef ENABLE_433
    // process data
    if(!rf433_process()) {
      return; // nothing yet 
    }
    
    // update values
    RF433_DATA tempdata = rf433_values();
    
    if(tempdata.Status) {
      rf433data = tempdata;
      if (rf433data.command == 1) { // Reset ALL values
        runtime.max_Groundspeed = 0;
        runtime.lap_max_Groundspeed = 0;
        runtime.laps = 0;
        runtime.best_laptime = 0;
        runtime.start_time = millis();
        runtime.has_lastlap = false;
      } else if (rf433data.command == 2) { //New lap
        runtime.laps++;
        runtime.lap_max_Groundspeed = 0;
        int previous_laptime = runtime.last_laptime;
        runtime.last_laptime = millis()-runtime.start_time;
        if (runtime.best_laptime > runtime.last_laptime || !runtime.has_lastlap) {
          runtime.best_laptime = runtime.last_laptime;
          runtime.has_lastlap = true;
        }
        runtime.laptime_diff = abs(previous_laptime-runtime.last_laptime);
        int sec_diff = round((previous_laptime-runtime.last_laptime)/1000)*1000;
        if (sec_diff>0) {
          runtime.laptime_symbol = 1;
        } else if (sec_diff<0) {
          runtime.laptime_symbol = 2;
        } else {
          runtime.laptime_symbol = 0;
        }
        runtime.start_time = millis();
      }
    }
#endif
}

/** 
  * (UpdateValues) Updates certain values like voltage, rssi, etc
  */

void UpdateValues() {
        
    // Voltages
    #ifdef SHOW_VOLTAGE 
  
      if(SHOW_VOLTAGE == 1 || SHOW_VOLTAGE == 3) {
        runtime.voltage1 = ReadVoltage(VOLTAGE_PIN_BATTERY1, VOLTAGE_DIVIDER_BATTERY1);     
      }
  
      if(SHOW_VOLTAGE == 2 || SHOW_VOLTAGE == 3) {
        runtime.voltage2 = ReadVoltage(VOLTAGE_PIN_BATTERY2, VOLTAGE_DIVIDER_BATTERY2);
      }
  
    #endif 
  
    // RSSI
    #ifdef SHOW_RSSI 
  
      if(SHOW_RSSI == 1) {
        runtime.rssi = analogRead(RSSI_PIN) / 4; 
      }
  
    #endif
}


/**
  * (UpdateDisplay) Draws the OSD Screen
  */

void UpdateDisplay() {
  
  // Permanent OSD Display
  DrawTimer(vma(LAYOUT_OSDTIME_X, -1), vma(LAYOUT_OSDTIME_Y, 3), millis()-runtime.start_time, NO_SYMBOL, true);
  #ifdef CAR_MODE
  if (runtime.has_lastlap) {
      // Best Lap time
      DrawTimer(vma(LAYOUT_BEST_LAPTIME_X, -1), vma(LAYOUT_BEST_LAPTIME_Y, 3), runtime.best_laptime, SYMBOL_HEARTBEAT, true);
      // last lap time

      DrawTimer(vma(LAYOUT_LAST_LAPTIME_X, -1), vma(LAYOUT_LAST_LAPTIME_Y, 3), runtime.last_laptime, SYMBOL_FLAG, true);
        // last laptime diff
      int symbol = SYMBOL_NOCLIMB;
      if (runtime.laptime_symbol == 1) {
        symbol = SYMBOL_CLIMBUP;
      } else if (runtime.laptime_symbol == 2) {
        symbol = SYMBOL_CLIMBDOWN;
      }
      DrawTimer(vma(LAYOUT_LAST_LAPTIME_DIFF_X, -1), vma(LAYOUT_LAST_LAPTIME_DIFF_Y, 3), runtime.laptime_diff, symbol, true);
    }
  #endif
  
  // Permanent GPS Display
  DrawStatus(LAYOUT_GPS_X, LAYOUT_GPS_Y, true, SYMBOL_GPS);
  DrawOneSmallValue(LAYOUT_GPS_X + 1, LAYOUT_GPS_Y, gpsdata.Satellites > 9 ? 9 : gpsdata.Satellites);

  #ifdef ENABLE_433
    DrawStatus(LAYOUT_LAPS_X, LAYOUT_LAPS_Y, true, SYMBOL_FLAG);
    DrawOneSmallValue(LAYOUT_LAPS_X + 1, LAYOUT_LAPS_Y, runtime.laps);
  #endif

  // Permanent Voltage Display
  #ifdef SHOW_VOLTAGE 

    if(SHOW_VOLTAGE == 1 || SHOW_VOLTAGE == 3) {
      DrawBattery(vma(LAYOUT_BATTERY1_X, -1), LAYOUT_BATTERY1_Y, runtime.voltage1, TYPE_BATTERY1, DISPLAY_MODE_BATTERY1);
      
       #ifdef SHOW_BATTERY_ALERT
          DrawBatteryAlert(SHOW_BATTERY_ALERT == 1, runtime.voltage1, TYPE_BATTERY1);
       #endif  
    }

    if(SHOW_VOLTAGE == 2 || SHOW_VOLTAGE == 3) {
      DrawBattery(vma(LAYOUT_BATTERY2_X, -1), LAYOUT_BATTERY2_Y - (SHOW_VOLTAGE == 2 ? 1 : 0), runtime.voltage2, TYPE_BATTERY2, DISPLAY_MODE_BATTERY2);
      
    }

  #endif 

  #ifndef GPS_ENABLED
     // clear screen
     if(runtime.osdredraw) {
      // clear
      runtime.osdredraw = false;
      osd.clear();
     }

  #else
    // Check GPS Status
    if(gpsdata.Status >= GPS_ACCURACY && gpsdata.Satellites >= GPS_REQUIRED_SAT) {
      
      // check if we have home position
      if(!runtime.gpshashome) {
        // clear osd
        if(runtime.osdredraw) {
          // clear
          runtime.osdredraw = false;
          osd.clear();
       
          // initialize
          MessageAlert(true, " HOME");
          
          // draw fancy progressbar
          DrawBox(vma(LAYOUT_PROGRESS_X - 1, -1), LAYOUT_PROGRESS_Y - 1, 8, 3);
        }
          
        // set warn

        // open warning
        runtime.gpswarn = true;
       
        // calculate progress
        int p = (runtime.gpsaltcount * 100 / GPS_HOME_WAIT);
        
        for(int i = 0; i < 5; i++) {
          osd.writexy(vma(LAYOUT_PROGRESS_X, -1) + i, LAYOUT_PROGRESS_Y, (p / 20 > i) ? CHAR_WHITE: CHAR_CLEAR);
        }
        
      } else {

        // we get positive GPS Values
        if(runtime.gpswarn) {
          // clear gps warn mesage
          runtime.gpswarn = false;
          runtime.osdredraw = true;      
        }
      
        // one time redraw
        if(runtime.osdredraw) {
          // clear 
          runtime.osdredraw = false;   
          // clear screen
          osd.clear();
          
          // draw boxes for display
          if(SHOW_LABELS) {
            DrawLabelBox(vma(LAYOUT_SPEED_X - 1, -1), vma(LAYOUT_SPEED_Y - 1, 1), 6, 3, 0x60, GetUnitSpeedSymbol(UNIT_SPEED, true));
            #ifndef CAR_MODE
              DrawLabelBox(LAYOUT_ALT_X - 1, vma(LAYOUT_ALT_Y - 1, 1), 6, 3, GetUnitSymbol(UNIT_ALTITUDE, true), 0x65);
            #endif
          } else {
            DrawBox(vma(LAYOUT_SPEED_X - 1, -1), vma(LAYOUT_SPEED_Y - 1, 1), 6,3);
            #ifndef CAR_MODE
              DrawBox(LAYOUT_ALT_X - 1, vma(LAYOUT_ALT_Y - 1, 1), 6, 3);
            #endif
          }
        }
        
        // draw current heading with interpolation
        if(gpsdata.Heading < (runtime.gpsheading - GPS_HEADING_INTERPOLATE) || gpsdata.Heading > (runtime.gpsheading + GPS_HEADING_INTERPOLATE)) { 
          DrawThreeDigitValue(vma(LAYOUT_HEADING_X, -1), LAYOUT_HEADING_Y, gpsdata.Heading, SYMBOL_DEGREE);
          runtime.gpsheading = gpsdata.Heading;
        }
        
        // draw speed
        DrawFourDigitValue(vma(LAYOUT_SPEED_X, -1), vma(LAYOUT_SPEED_Y, 1), fabs(gpsdata.Groundspeed * UNIT_SPEED), NO_SYMBOL, NO_SYMBOL, FONT_LARGE);

        #ifdef CAR_MODE
          //Draw Max Speed
          DrawThreeDigitValue(vma(LAYOUT_MAX_SPEED_X, -1), vma(LAYOUT_MAX_SPEED_Y, 1), fabs(runtime.max_Groundspeed * UNIT_SPEED), SYMBOL_CLIMBUP);
          //Draw Lap Max Speed
          //DrawThreeDigitValue(vma(LAYOUT_LAP_MAX_SPEED_X, -1), vma(LAYOUT_LAP_MAX_SPEED_Y, 1), fabs(runtime.lap_max_Groundspeed * UNIT_SPEED), NO_SYMBOL);
        #endif

        // calculate and draw altitude
        #ifndef CAR_MODE
          float alt = gpsdata.Altitude;
          if(GPS_ALTITUDE_TYPE == 0) {
            alt = gpsdata.Altitude - runtime.gpsasl;
            if(alt < 0 && alt > GPS_AGL_NEGATIVE) alt = 0;
          }
       
          DrawFourDigitValue(LAYOUT_ALT_X, vma(LAYOUT_ALT_Y, 1), fabs(alt * UNIT_ALTITUDE), NO_SYMBOL, NO_SYMBOL, FONT_LARGE);
        #endif
        
        // draw vertical indicator
        #ifndef CAR_MODE
          DrawStatus(LAYOUT_CLIMB_X, vma(LAYOUT_CLIMB_Y, 1), true, runtime.gpsclimb);
        #endif
        
        // draw home arrow
        DrawFancyHeading(vma(LAYOUT_HOMEBEARING_X, -1), LAYOUT_HOMEBEARING_Y, runtime.gpshomedirection);

        // draw home distance
        #ifndef CAR_MODE
          DrawDistance(LAYOUT_HOMEDISTANCE_X, vma(LAYOUT_HOMEDISTANCE_Y, 3), runtime.gpshomedistance, SYMBOL_HOME);
        #endif

        // draw total flight distance
        DrawDistance(LAYOUT_DISTANCE_X, vma(LAYOUT_DISTANCE_Y, 3), runtime.distance, SYMBOL_FLAG);
        
        #ifndef CAR_MODE        
          // travel distance
          DrawTimer(vma(LAYOUT_FLYTIME_X, -1), vma(LAYOUT_FLYTIME_Y, 3), runtime.flytime, SHOW_LABELS ? SYMBOL_TIMEFLY : NO_SYMBOL, false);
        #endif
      }
      
      
      // gps coordinates
      #ifdef GPS_COORDINATES_SHOW
        // Latitude 
        DrawCoordinates(vma(LAYOUT_GPSCOORD_X, -1), vma(LAYOUT_GPSCOORD_Y, 3), gpsdata.Latitude, GPS_PRECISION, SHOW_LABELS ? SYMBOL_LAT : NO_SYMBOL);
        // Longitude
        DrawCoordinates(vma(LAYOUT_GPSCOORD_X, -1), vma(LAYOUT_GPSCOORD_Y + 1, 3), gpsdata.Longitude, GPS_PRECISION, SHOW_LABELS ? SYMBOL_LON : NO_SYMBOL);
      #endif

      
    } else {
      
      if(!runtime.gpswarn) {
        // clear screen
        osd.clear();
        
        DrawLogo();
        
        // show message
        runtime.gpswarn = true;
        char* message = "NO GPS";
        if (runtime.rf433error && !runtime.rf433warn) {
          message = "NO GPS-NO 433";
          runtime.rf433warn = true;
        }
        MessageAlert(true, message);
      }
    }
  #endif
  #ifdef ENABLE_433
    if(runtime.rf433error && !runtime.rf433warn) {
        // clear screen
        osd.clear();
        
        DrawLogo();
        
        // show message
        runtime.rf433warn = true;
        MessageAlert(true, "NO 433");
      }
   #endif
}


/**
  * (Heartbeat) Heartbeat Actions
  */

void Heartbeat() {
  
  if(millis() - runtime.ledtimeout > LED_HEARTBEAT) {
    
     runtime.ledtimeout = millis();
  
     runtime.ledstatus = !runtime.ledstatus;
  
     digitalWrite(LED_PIN, runtime.ledstatus ? HIGH : LOW);
  }
}
