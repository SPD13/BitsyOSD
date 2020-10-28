/**
   ___  _ __           ____  _______ 
  / _ )(_) /____ __ __/ __ \/ __/ _ \
 / _  / / __(_-</ // / /_/ /\ \/ // /
/____/_/\__/___/\_, /\____/___/____/ 
               /___/                 

RF433 Information                                          

**/

#ifndef RF433
#define RF433


/** 
  * (struct) RF433_DATA
  * Primary structure for RF433 data
  */

typedef struct RF433_DATA {
    boolean	Status;
    uint8_t seq_id;
    uint8_t	pilotID;
    uint8_t command;
    uint8_t parameter;
};

#endif /* RF433 */
