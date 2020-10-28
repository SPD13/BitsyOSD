/**
   ___  _ __           ____  _______ 
  / _ )(_) /____ __ __/ __ \/ __/ _ \
 / _  / / __(_-</ // / /_/ /\ \/ // /
/____/_/\__/___/\_, /\____/___/____/ 
               /___/                 

GPS NEMA Parser                                      

**/

#include "rf433.h"

static struct RF433_DATA RF433Values;


/** 
  * (rf433_process) Processes the RF433 Data
  */
  
boolean rf433_process(void) {
  // read serial

  uint8_t buf[4];
  uint8_t buflen = sizeof(buf);
  RF433Values.Status = false;
  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    // Message with a good checksum received
    //check sequence, if differentt from previous one, update message, otherwise it's a repeated message
    if (RF433Values.seq_id != buf[0] && buf[1] == RF433_PILOT_ID) {
      RF433Values.Status = true;
      RF433Values.seq_id = buf[0];
      RF433Values.pilotID = buf[1];
      RF433Values.command = buf[2];
      RF433Values.parameter = buf[3];
      return true;
    }
  }

  return false;
}


/**
  * (gps_values) Returns the GPS Values
  */
  
RF433_DATA rf433_values() {
  return RF433Values;
}
