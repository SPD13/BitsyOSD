#include <RH_ASK.h>
#include <SPI.h>

RH_ASK driver(2000, 2, 11, 5); //Not transmitting, just receiving

int seq_id = 1;
#define RF433_PILOT_ID 1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Debugging only
  if (!driver.init())
      Serial.println("init failed");
  Serial.println("Started");
}

void loop() {
  uint8_t buf[4];
  uint8_t buflen = sizeof(buf);

  if (driver.recv(buf, &buflen)) // Non-blocking
  {
    // Message with a good checksum received
    //check sequence, if differentt from previous one, update message, otherwise it's a repeated message
    if (seq_id != buf[0] && buf[1] == RF433_PILOT_ID) {
      seq_id = buf[0];
      int pilotID = buf[1];
      int command = buf[2];
      int parameter = buf[3];
      Serial.print("Message Received. pilotID:");
      Serial.print(pilotID);
      Serial.print(" Command:");
      Serial.print(command);
      Serial.print(" Parameter:");
      Serial.println(parameter);
    } else {
      Serial.println("Wrong Seq or pilotID");
    }
  }

}
